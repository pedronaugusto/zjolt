//===----------------------------------------------------------------------===//
// zjolt — saving and restoring the state of a simulation.
//
// Jolt's own StateRecorderImpl is a std::stringstream, which would drag
// <sstream> into this library and copy the payload twice on the way out. A
// StateRecorder is just a StreamIn and a StreamOut with two flags, so the two
// that are actually needed are written here over a flat buffer — the same
// shape as CountingStreamOut and ConstStreamIn in zjolt_internal.h, which do
// the same job for shapes.
//
// Around Jolt's payload goes the same container zjoltShapeSave uses, and one
// field more. Jolt validates its own reads inside a state restore, but its
// reaction to a world that has changed shape underneath is
// `JPH_ASSERT(false, "Restoring state for non-existing body")`
// (`BodyManager.cpp:790`) followed by returning false — an abort in a build
// with asserts on, for something an ordinary caller reaches by destroying a
// body between the save and the restore. So the container also records a
// digest of the body set, and a restore compares it before Jolt reads a byte.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Core/QuickSort.h>
#include <Jolt/Physics/StateRecorder.h>

namespace {

//===----------------------------------------------------------------------===//
// Byte-buffer state recorders
//===----------------------------------------------------------------------===//

/// Counts bytes when `buffer` is null and writes them when it is not, so the
/// size query and the write are one code path and cannot disagree.
///
/// StateRecorder inherits StreamIn as well as StreamOut, so ReadBytes has to
/// exist. Nothing in a save calls it; if anything ever does, the save fails
/// rather than quietly writing a payload built out of zeroes.
class StateRecorderOut final : public JPH::StateRecorder {
 public:
  StateRecorderOut(void *buffer, size_t capacity)
      : buffer_(static_cast<JPH::uint8 *>(buffer)), capacity_(capacity) {}

  void WriteBytes(const void *inData, size_t inNumBytes) override {
    if (buffer_ != nullptr) {
      if (written_ + inNumBytes > capacity_) {
        overflowed_ = true;
      } else {
        std::memcpy(buffer_ + written_, inData, inNumBytes);
      }
    }
    written_ += inNumBytes;
  }

  void ReadBytes(void *outData, size_t inNumBytes) override {
    std::memset(outData, 0, inNumBytes);
    misused_ = true;
  }

  bool IsEOF() const override { return true; }
  bool IsFailed() const override { return overflowed_ || misused_; }

  size_t Size() const { return written_; }

 private:
  JPH::uint8 *buffer_;
  size_t capacity_;
  size_t written_ = 0;
  bool overflowed_ = false;
  bool misused_ = false;
};

/// Reads from a borrowed buffer, reporting end-of-file rather than handing
/// back uninitialised bytes. Zero-filling past the end keeps a count Jolt has
/// already read from being interpreted as garbage in the window before its own
/// check.
class StateRecorderIn final : public JPH::StateRecorder {
 public:
  StateRecorderIn(const void *data, size_t size)
      : data_(static_cast<const JPH::uint8 *>(data)), size_(size) {}

  void ReadBytes(void *outData, size_t inNumBytes) override {
    JPH::uint8 *out = static_cast<JPH::uint8 *>(outData);
    const size_t available = read_ < size_ ? size_ - read_ : 0;
    const size_t n = inNumBytes < available ? inNumBytes : available;
    if (n != 0) std::memcpy(out, data_ + read_, n);
    if (n < inNumBytes) {
      std::memset(out + n, 0, inNumBytes - n);
      eof_ = true;
    }
    read_ += inNumBytes;
  }

  void WriteBytes(const void *, size_t) override { misused_ = true; }

  bool IsEOF() const override { return eof_; }
  bool IsFailed() const override { return misused_; }

  /// True when the whole payload was consumed. A restore that stopped short
  /// read something other than what was written.
  bool ConsumedAll() const { return !eof_ && read_ == size_; }

 private:
  const JPH::uint8 *data_;
  size_t size_;
  size_t read_ = 0;
  bool eof_ = false;
  bool misused_ = false;
};

//===----------------------------------------------------------------------===//
// The container
//
// Deliberately a different magic tag and a different layout from the shape
// container in zjolt_shape.cpp: a shape buffer handed to a state restore, or
// the other way round, has to be refused on the tag rather than parsed.
//===----------------------------------------------------------------------===//

constexpr uint8_t kMagic[4] = {'Z', 'J', 'S', 'T'};
constexpr uint32_t kFormatVersion = 1;
constexpr size_t kHeaderSize = 48;

/// CRC-32, the usual reflected polynomial, computed without a table. A state
/// buffer is written once and read once; the table is not worth the cache
/// line.
uint32_t Crc32(const uint8_t *data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xedb88320u & mask);
    }
  }
  return ~crc;
}

void WriteU32(uint8_t *out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t ReadU32(const uint8_t *in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

void WriteU64(uint8_t *out, uint64_t value) {
  WriteU32(out, static_cast<uint32_t>(value));
  WriteU32(out + 4, static_cast<uint32_t>(value >> 32));
}

uint64_t ReadU64(const uint8_t *in) {
  return static_cast<uint64_t>(ReadU32(in)) |
         (static_cast<uint64_t>(ReadU32(in + 4)) << 32);
}

uint32_t JoltVersionStamp() {
  return (static_cast<uint32_t>(JPH_VERSION_MAJOR) << 16) |
         (static_cast<uint32_t>(JPH_VERSION_MINOR) << 8) |
         static_cast<uint32_t>(JPH_VERSION_PATCH);
}

/// Identifies WHICH bodies the world holds, not how many.
///
/// A count alone would pass a world that destroyed one body and created
/// another, which is precisely the case Jolt asserts on. Folding the ids,
/// sorted so that creation order does not matter, is what makes the two
/// distinguishable.
uint32_t BodySetDigest(const JPH::PhysicsSystem &system, uint32_t *out_count) {
  JPH::BodyIDVector ids;
  system.GetBodies(ids);
  JPH::QuickSort(ids.begin(), ids.end());

  uint32_t hash = 2166136261u;  // FNV-1a offset basis
  for (const JPH::BodyID &id : ids) {
    const uint32_t value = id.GetIndexAndSequenceNumber();
    for (int byte = 0; byte < 4; ++byte) {
      hash ^= static_cast<uint8_t>(value >> (byte * 8));
      hash *= 16777619u;
    }
  }

  *out_count = static_cast<uint32_t>(ids.size());
  return hash;
}

}  // namespace

extern "C" {

ZJoltResult zjoltPhysicsSystemSaveState(const ZJoltPhysicsSystem *system,
                                        uint32_t state, void *buffer,
                                        size_t capacity, size_t *out_size) {
  ZJOLT_ENTER(out_size);
  if (!zjolt::Present(system, out_size)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Jolt's own mask is a uint8, so a bit outside the four this ABI names does
  // not merely mean nothing — it is truncated on the way in and then written
  // into the payload for a restore to act on. Refused rather than masked.
  if ((state & ~(uint32_t)ZJOLT_STATE_RECORDER_STATE_ALL) != 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "state carries bits outside ZJOLT_STATE_RECORDER_STATE_ALL");
  }

  uint8_t *bytes = static_cast<uint8_t *>(buffer);

  // A buffer that cannot even hold the header is counted, not written into.
  // `bytes + kHeaderSize` would otherwise form a pointer past the end of the
  // caller's allocation, which is undefined before anything is stored through
  // it.
  const bool count_only = bytes == nullptr || capacity < kHeaderSize;

  StateRecorderOut stream(count_only ? nullptr : bytes + kHeaderSize,
                          count_only ? 0 : capacity - kHeaderSize);
  system->system.SaveState(stream,
                           static_cast<JPH::EStateRecorderState>(state));

  const size_t payload_size = stream.Size();
  *out_size = kHeaderSize + payload_size;
  if (bytes == nullptr) return ZJOLT_RESULT_OK;
  if (count_only || capacity < *out_size || stream.IsFailed())
    return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  uint32_t body_count = 0;
  const uint32_t body_digest = BodySetDigest(system->system, &body_count);

  std::memcpy(bytes, kMagic, sizeof(kMagic));
  WriteU32(bytes + 4, kFormatVersion);
  WriteU32(bytes + 8, static_cast<uint32_t>(ZJOLT_CONFIG_ID));
  WriteU32(bytes + 12, JoltVersionStamp());
  WriteU64(bytes + 16, static_cast<uint64_t>(payload_size));
  WriteU32(bytes + 24, Crc32(bytes + kHeaderSize, payload_size));
  WriteU32(bytes + 28, state);
  WriteU32(bytes + 32, body_count);
  WriteU32(bytes + 36, body_digest);
  WriteU64(bytes + 40, 0);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemRestoreState(ZJoltPhysicsSystem *system,
                                           const void *data, size_t size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, data)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  if (size < kHeaderSize) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "too short to be a saved simulation state");
  }
  if (std::memcmp(bytes, kMagic, sizeof(kMagic)) != 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "not a state saved by zjoltPhysicsSystemSaveState");
  }
  if (ReadU32(bytes + 4) != kFormatVersion) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "saved by a different zjolt container version");
  }
  if (ReadU32(bytes + 8) != static_cast<uint32_t>(ZJOLT_CONFIG_ID)) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "saved by a zjolt built with different layout-affecting settings");
  }
  if (ReadU32(bytes + 12) != JoltVersionStamp()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "saved against a different Jolt version");
  }

  const uint64_t payload_size = ReadU64(bytes + 16);
  if (payload_size != static_cast<uint64_t>(size - kHeaderSize)) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "the recorded payload length does not match the buffer");
  }
  if (ReadU32(bytes + 24) !=
      Crc32(bytes + kHeaderSize, static_cast<size_t>(payload_size))) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the state payload failed its checksum");
  }

  // The check Jolt does not make. Its restore looks every saved body id up and
  // asserts when one is gone, so a body destroyed since the save aborts the
  // process before the returned `false` can be read. Comparing the body set
  // first turns that into a refusal, and refusing here also means the world is
  // still untouched — once Jolt has started reading, it is not.
  const uint32_t state = ReadU32(bytes + 28);
  if ((state & ZJOLT_STATE_RECORDER_STATE_BODIES) != 0) {
    uint32_t body_count = 0;
    const uint32_t body_digest = BodySetDigest(system->system, &body_count);
    if (body_count != ReadU32(bytes + 32) ||
        body_digest != ReadU32(bytes + 36)) {
      return zjolt::SetError(
          ZJOLT_RESULT_BAD_FORMAT,
          "the system no longer holds the same bodies this state was saved "
          "from; a state restore puts motion back, it does not create or "
          "destroy bodies");
    }
  }

  StateRecorderIn stream(bytes + kHeaderSize,
                         static_cast<size_t>(payload_size));
  if (!system->system.RestoreState(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "Jolt refused the saved state");
  }
  if (stream.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the state data ended before the state did");
  }
  if (!stream.ConsumedAll()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "trailing bytes after the state");
  }
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
