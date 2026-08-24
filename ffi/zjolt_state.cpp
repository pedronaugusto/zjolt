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
#include <Jolt/Physics/Body/BodyLock.h>
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
// The shared framing in zjolt_internal.h, with a magic tag deliberately
// different from the shape container's in zjolt_shape.cpp: a shape buffer
// handed to a state restore, or the other way round, has to be refused on the
// tag rather than parsed.
//===----------------------------------------------------------------------===//

/// The shared container from zjolt_internal.h, with this subsystem's own tag
/// and twenty bytes of its own after the common header: the state mask at 28,
/// the body count at 32, the body-set digest at 36, and eight reserved.
constexpr zjolt::ContainerFormat kContainer = {
    /*magic=*/{'Z', 'J', 'S', 'T'},
    /*version=*/1,
    /*extra_size=*/20,
    /*too_short=*/"too short to be a saved simulation state",
    /*wrong_magic=*/"not a state saved by zjoltPhysicsSystemSaveState",
    /*bad_checksum=*/"the state payload failed its checksum",
};

constexpr size_t kHeaderSize = kContainer.HeaderSize();

/// Offsets into the twenty bytes above, from the start of the extra fields.
constexpr size_t kStateMaskOffset = 0;
constexpr size_t kBodyCountOffset = 4;
constexpr size_t kBodyDigestOffset = 8;

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

//===----------------------------------------------------------------------===//
// Per-body container
//
// The same shared framing again, under a magic tag different from both the
// whole-system state container above and the shape container in
// zjolt_shape.cpp, for the same reason those two differ from each other: a
// buffer meant for one restore has to be refused by the other rather than
// parsed. The fields it adds of its own are the body's motion type, which is
// what decides how many bytes Jolt reads.
//===----------------------------------------------------------------------===//

constexpr zjolt::ContainerFormat kBodyContainer = {
    /*magic=*/{'Z', 'J', 'B', 'S'},
    /*version=*/1,
    /*extra_size=*/4,
    /*too_short=*/"too short to be a saved body state",
    /*wrong_magic=*/
    "not a body state saved by zjoltPhysicsSystemSaveBodyStateLocked",
    /*bad_checksum=*/"the body state payload failed its checksum",
};

constexpr size_t kBodyHeaderSize = kBodyContainer.HeaderSize();

/// What Body::SaveState / RestoreState's byte count actually depends on:
/// whether there are motion properties to read at all, and, if so, whether
/// they belong to a soft body. Folded into one number purely so a restore has
/// something to compare against the body it is about to write into.
uint32_t BodyCategory(const JPH::Body &body) {
  uint32_t category = static_cast<uint32_t>(body.GetMotionType());
  if (body.IsSoftBody()) category |= 0x100u;
  return category;
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

  uint8_t extra[kContainer.extra_size] = {};
  zjolt::WriteLE32(extra + kStateMaskOffset, state);
  zjolt::WriteLE32(extra + kBodyCountOffset, body_count);
  zjolt::WriteLE32(extra + kBodyDigestOffset, body_digest);
  zjolt::WriteContainerHeader(kContainer, bytes, payload_size, extra);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemRestoreState(ZJoltPhysicsSystem *system,
                                           const void *data, size_t size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, data)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::ContainerContents contents;
  const ZJoltResult framed =
      zjolt::ReadContainer(kContainer, data, size, &contents);
  if (framed != ZJOLT_RESULT_OK) return framed;

  // The check Jolt does not make. Its restore looks every saved body id up and
  // asserts when one is gone, so a body destroyed since the save aborts the
  // process before the returned `false` can be read. Comparing the body set
  // first turns that into a refusal, and refusing here also means the world is
  // still untouched — once Jolt has started reading, it is not.
  const uint32_t state = zjolt::ReadLE32(contents.extra + kStateMaskOffset);
  if ((state & ZJOLT_STATE_RECORDER_STATE_BODIES) != 0) {
    uint32_t body_count = 0;
    const uint32_t body_digest = BodySetDigest(system->system, &body_count);
    if (body_count != zjolt::ReadLE32(contents.extra + kBodyCountOffset) ||
        body_digest != zjolt::ReadLE32(contents.extra + kBodyDigestOffset)) {
      return zjolt::SetError(
          ZJOLT_RESULT_BAD_FORMAT,
          "the system no longer holds the same bodies this state was saved "
          "from; a state restore puts motion back, it does not create or "
          "destroy bodies");
    }
  }

  StateRecorderIn stream(contents.payload, contents.payload_size);
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

ZJoltResult zjoltPhysicsSystemSaveBodyStateLocked(
    const ZJoltPhysicsSystem *system, const ZJoltBody *body, void *buffer,
    size_t capacity, size_t *out_size) {
  ZJOLT_ENTER(out_size);
  if (!zjolt::Present(system, body, out_size)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::Body *jbody = zjolt::ToJolt(body);
  uint8_t *bytes = static_cast<uint8_t *>(buffer);
  const bool count_only = bytes == nullptr || capacity < kBodyHeaderSize;

  StateRecorderOut stream(count_only ? nullptr : bytes + kBodyHeaderSize,
                          count_only ? 0 : capacity - kBodyHeaderSize);
  system->system.SaveBodyState(*jbody, stream);

  const size_t payload_size = stream.Size();
  *out_size = kBodyHeaderSize + payload_size;
  if (bytes == nullptr) return ZJOLT_RESULT_OK;
  if (count_only || capacity < *out_size || stream.IsFailed())
    return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  uint8_t extra[kBodyContainer.extra_size] = {};
  zjolt::WriteLE32(extra, BodyCategory(*jbody));
  zjolt::WriteContainerHeader(kBodyContainer, bytes, payload_size, extra);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemRestoreBodyStateLocked(ZJoltPhysicsSystem *system,
                                                     ZJoltBody *body,
                                                     const void *data,
                                                     size_t size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, body, data)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  zjolt::ContainerContents contents;
  const ZJoltResult framed =
      zjolt::ReadContainer(kBodyContainer, data, size, &contents);
  if (framed != ZJOLT_RESULT_OK) return framed;

  JPH::Body *jbody = zjolt::ToJolt(body);
  if (zjolt::ReadLE32(contents.extra) != BodyCategory(*jbody)) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "saved from a body of a different motion type; a body state restore "
        "puts motion back onto the same kind of body it was saved from");
  }

  StateRecorderIn stream(contents.payload, contents.payload_size);
  system->system.RestoreBodyState(*jbody, stream);
  if (stream.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the body state data ended before the state did");
  }
  if (!stream.ConsumedAll()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "trailing bytes after the body state");
  }
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
