//===----------------------------------------------------------------------===//
// zjolt — saving and restoring the state of a simulation.
//
// Jolt's own StateRecorderImpl is a std::stringstream, dragging in
// <sstream> and copying the payload twice. Instead every save/restore
// runs through zjolt_internal.h's HostStream, forwarding a
// StateRecorder's virtuals to a ZJoltStream's four function pointers —
// one implementation for both the buffer and host-stream forms.
//
// The same container zjoltShapeSave uses wraps Jolt's payload, plus a
// body-set digest: Jolt's own reaction to a world that changed shape
// underneath a restore is an abort with asserts on, so the container
// records the digest and a restore compares it first. The stream forms keep only that one check (cheap without a resident blob) — see ZJoltStream in zjolt_core.h.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Core/QuickSort.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/StateRecorder.h>

namespace {

//===----------------------------------------------------------------------===//
// The container
//
// The shared framing in zjolt_internal.h, with a magic tag deliberately different from the shape container's (zjolt_shape.cpp): a shape buffer handed to a state restore, or vice versa, must be refused on the tag rather than parsed.
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
/// Whether this save was PARTIAL (a ZJoltStateFilter's save-side
/// questions were consulted) — one byte of the eight reserved. A
/// restore skips the body-set digest entirely when this is set, since
/// the digest would otherwise compare the save's deliberately
/// incomplete body set against the system's whole one and refuse a
/// good partial restore. A save predating this field reads it as 0 (not partial), which is exactly what it was.
constexpr size_t kIsPartialOffset = 12;

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
// The same shared framing, under a magic tag different from both the
// whole-system container above and the shape container (zjolt_shape.cpp) — a buffer meant for one restore must be refused by the other, not parsed. Adds the body's motion type, which decides how many bytes Jolt reads.
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

/// A constraint handle, the way every other ffi/*.cpp translation unit that
/// hands one out builds it: ZJoltConstraint is a bare reinterpret_cast tag
/// over JPH::Constraint, never a wrapper struct, so the pointer a filter is
/// handed here is the exact one the host already holds from whichever
/// zjolt_constraint.h/zjolt_vehicle.h/zjolt_ragdoll.h call created it.
inline ZJoltConstraint *ToC(const JPH::Constraint *constraint) {
  return reinterpret_cast<ZJoltConstraint *>(
      const_cast<JPH::Constraint *>(constraint));
}

/// Forwards Jolt's four save/restore questions to the host's callbacks. A
/// NULL field accepts everything, same as a NULL ZJoltStateFilter* altogether.
class StateFilterAdapter final : public JPH::StateRecorderFilter {
 public:
  explicit StateFilterAdapter(const ZJoltStateFilter &filter)
      : filter_(filter) {}

  bool ShouldSaveBody(const JPH::Body &inBody) const override {
    if (filter_.should_save_body == nullptr) return true;
    return filter_.should_save_body(filter_.user, zjolt::ToC(inBody.GetID()));
  }

  bool ShouldSaveConstraint(const JPH::Constraint &inConstraint) const override {
    if (filter_.should_save_constraint == nullptr) return true;
    return filter_.should_save_constraint(filter_.user, ToC(&inConstraint));
  }

  bool ShouldSaveContact(const JPH::BodyID &inBody1,
                         const JPH::BodyID &inBody2) const override {
    if (filter_.should_save_contact == nullptr) return true;
    return filter_.should_save_contact(filter_.user, zjolt::ToC(inBody1),
                                       zjolt::ToC(inBody2));
  }

  bool ShouldRestoreContact(const JPH::BodyID &inBody1,
                            const JPH::BodyID &inBody2) const override {
    if (filter_.should_restore_contact == nullptr) return true;
    return filter_.should_restore_contact(filter_.user, zjolt::ToC(inBody1),
                                          zjolt::ToC(inBody2));
  }

 private:
  ZJoltStateFilter filter_;
};

/// True if `filter` changes what a SAVE writes — the three save-side
/// questions, not should_restore_contact, which only matters on the way back
/// in and says nothing about whether this save covers every body.
bool IsPartialSave(const ZJoltStateFilter *filter) {
  return filter != nullptr &&
         (filter->should_save_body != nullptr ||
          filter->should_save_constraint != nullptr ||
          filter->should_save_contact != nullptr);
}

}  // namespace

extern "C" {

ZJoltResult zjoltPhysicsSystemSaveState(const ZJoltPhysicsSystem *system,
                                        uint32_t state,
                                        const ZJoltStateFilter *filter,
                                        void *buffer, size_t capacity,
                                        size_t *out_size) {
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

  zjolt::MemoryCursor cursor;
  cursor.out = count_only ? nullptr : bytes + kHeaderSize;
  cursor.capacity = count_only ? 0 : capacity - kHeaderSize;
  zjolt::HostStream stream(zjolt::StreamOverMemory(&cursor));
  const bool is_partial = IsPartialSave(filter);
  if (filter != nullptr) {
    StateFilterAdapter adapter(*filter);
    system->system.SaveState(stream, static_cast<JPH::EStateRecorderState>(state),
                             &adapter);
  } else {
    system->system.SaveState(stream, static_cast<JPH::EStateRecorderState>(state));
  }

  const size_t payload_size = cursor.written;
  *out_size = kHeaderSize + payload_size;
  if (bytes == nullptr) return ZJOLT_RESULT_OK;
  if (count_only || capacity < *out_size || stream.IsFailed())
    return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  // A partial save's body count/digest describe a set nobody will ever
  // compare against — restore skips that check outright when the flag below
  // is set — but they are still written rather than left as zero. Zero would
  // otherwise be indistinguishable from "a save of a world with zero bodies",
  // and this way a saved container never has silently-meaningless fields.
  uint32_t body_count = 0;
  const uint32_t body_digest = BodySetDigest(system->system, &body_count);

  uint8_t extra[kContainer.extra_size] = {};
  zjolt::WriteLE32(extra + kStateMaskOffset, state);
  zjolt::WriteLE32(extra + kBodyCountOffset, body_count);
  zjolt::WriteLE32(extra + kBodyDigestOffset, body_digest);
  extra[kIsPartialOffset] = is_partial ? 1 : 0;
  zjolt::WriteContainerHeader(kContainer, bytes, payload_size, extra);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemRestoreState(ZJoltPhysicsSystem *system,
                                           const void *data, size_t size,
                                           const ZJoltStateFilter *filter,
                                           bool is_last_part) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, data)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::ContainerContents contents;
  const ZJoltResult framed =
      zjolt::ReadContainer(kContainer, data, size, &contents);
  if (framed != ZJOLT_RESULT_OK) return framed;

  // The check Jolt does not make: its restore looks up every saved body
  // id and asserts when one is gone, aborting before the returned
  // `false` can be read. Comparing the body set first turns that into a
  // refusal, with the world still untouched (once Jolt starts reading,
  // it is not). Skipped entirely for a PARTIAL save: its body count and
  // digest describe a deliberately incomplete set (see ZJoltStateFilter), so comparing against the whole system would refuse a good chunked-restore part — getting disjoint parts wrong is the host's job here, as in Jolt's own SetIsLastPart.
  const uint32_t state = zjolt::ReadLE32(contents.extra + kStateMaskOffset);
  const bool is_partial = contents.extra[kIsPartialOffset] != 0;
  if (!is_partial && (state & ZJOLT_STATE_RECORDER_STATE_BODIES) != 0) {
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

  zjolt::MemoryCursor cursor;
  cursor.in = contents.payload;
  cursor.size = contents.payload_size;
  zjolt::HostStream stream(zjolt::StreamOverMemory(&cursor));
  stream.SetIsLastPart(is_last_part);

  bool restored;
  if (filter != nullptr) {
    StateFilterAdapter adapter(*filter);
    restored = system->system.RestoreState(stream, &adapter);
  } else {
    restored = system->system.RestoreState(stream);
  }
  if (!restored) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "Jolt refused the saved state");
  }
  if (stream.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the state data ended before the state did");
  }
  if (!zjolt::MemoryCursorConsumedAll(cursor)) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "trailing bytes after the state");
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemCompareState(const void *state_a, size_t size_a,
                                           const void *state_b, size_t size_b,
                                           bool *out_diverged,
                                           size_t *out_offset) {
  ZJOLT_ENTER(out_diverged, out_offset);
  if (!zjolt::Present(state_a, state_b, out_diverged, out_offset)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  zjolt::ContainerContents a;
  ZJoltResult framed = zjolt::ReadContainer(kContainer, state_a, size_a, &a);
  if (framed != ZJOLT_RESULT_OK) return framed;

  zjolt::ContainerContents b;
  framed = zjolt::ReadContainer(kContainer, state_b, size_b, &b);
  if (framed != ZJOLT_RESULT_OK) return framed;

  const size_t shorter =
      a.payload_size < b.payload_size ? a.payload_size : b.payload_size;
  size_t i = 0;
  for (; i < shorter; ++i) {
    if (a.payload[i] != b.payload[i]) break;
  }

  if (i < shorter || a.payload_size != b.payload_size) {
    *out_diverged = true;
    *out_offset = i;
  } else {
    *out_diverged = false;
    *out_offset = 0;
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

  zjolt::MemoryCursor cursor;
  cursor.out = count_only ? nullptr : bytes + kBodyHeaderSize;
  cursor.capacity = count_only ? 0 : capacity - kBodyHeaderSize;
  zjolt::HostStream stream(zjolt::StreamOverMemory(&cursor));
  system->system.SaveBodyState(*jbody, stream);

  const size_t payload_size = cursor.written;
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

  zjolt::MemoryCursor cursor;
  cursor.in = contents.payload;
  cursor.size = contents.payload_size;
  zjolt::HostStream stream(zjolt::StreamOverMemory(&cursor));
  system->system.RestoreBodyState(*jbody, stream);
  if (stream.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the body state data ended before the state did");
  }
  if (!zjolt::MemoryCursorConsumedAll(cursor)) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "trailing bytes after the body state");
  }
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// The stream form
//
// Jolt's payload, unframed except for the twelve-byte header WriteStreamHeader/ReadStreamHeader always write — see ZJoltStream in zjolt_core.h for what that keeps and does not.
// The whole-system pair also keeps the body-set digest `restore` checks: a few bytes computed from the system already in hand, not needing the payload addressable, so no reason to drop it with the rest of the container.
//===----------------------------------------------------------------------===//

namespace {

constexpr uint8_t kStreamMagic[4] = {'Z', 'S', 'S', 'T'};
constexpr uint8_t kBodyStreamMagic[4] = {'Z', 'S', 'B', 'S'};

}  // namespace

ZJoltResult zjoltPhysicsSystemSaveStateStream(const ZJoltPhysicsSystem *system,
                                              uint32_t state,
                                              const ZJoltStateFilter *filter,
                                              const ZJoltStream *stream) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to save through");
  }
  if ((state & ~(uint32_t)ZJOLT_STATE_RECORDER_STATE_ALL) != 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "state carries bits outside ZJOLT_STATE_RECORDER_STATE_ALL");
  }

  zjolt::HostStream host(*stream);
  zjolt::WriteStreamHeader(host, kStreamMagic);

  uint32_t body_count = 0;
  const uint32_t body_digest = BodySetDigest(system->system, &body_count);
  const bool is_partial = IsPartialSave(filter);
  uint8_t extra[13];
  zjolt::WriteLE32(extra + 0, state);
  zjolt::WriteLE32(extra + 4, body_count);
  zjolt::WriteLE32(extra + 8, body_digest);
  extra[12] = is_partial ? 1 : 0;
  host.WriteBytes(extra, sizeof(extra));

  if (filter != nullptr) {
    StateFilterAdapter adapter(*filter);
    system->system.SaveState(host, static_cast<JPH::EStateRecorderState>(state),
                             &adapter);
  } else {
    system->system.SaveState(host, static_cast<JPH::EStateRecorderState>(state));
  }

  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while writing the state");
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemRestoreStateStream(ZJoltPhysicsSystem *system,
                                                 const ZJoltStream *stream,
                                                 const ZJoltStateFilter *filter,
                                                 bool is_last_part) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "stream needs read, is_eof and is_failed to restore through");
  }

  zjolt::HostStream host(*stream);
  const ZJoltResult header = zjolt::ReadStreamHeader(
      host, kStreamMagic, "not a state saved by zjoltPhysicsSystemSaveStateStream");
  if (header != ZJOLT_RESULT_OK) return header;

  uint8_t extra[13];
  host.ReadBytes(extra, sizeof(extra));
  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while reading the state header");
  }
  if (host.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the stream ended before the state header did");
  }
  const uint32_t state = zjolt::ReadLE32(extra + 0);
  const bool is_partial = extra[12] != 0;

  // The same check zjoltPhysicsSystemRestoreState makes, kept here because it
  // is cheap without a resident blob -- @see the file comment above.
  if (!is_partial && (state & ZJOLT_STATE_RECORDER_STATE_BODIES) != 0) {
    uint32_t body_count = 0;
    const uint32_t body_digest = BodySetDigest(system->system, &body_count);
    if (body_count != zjolt::ReadLE32(extra + 4) ||
        body_digest != zjolt::ReadLE32(extra + 8)) {
      return zjolt::SetError(
          ZJOLT_RESULT_BAD_FORMAT,
          "the system no longer holds the same bodies this state was saved "
          "from; a state restore puts motion back, it does not create or "
          "destroy bodies");
    }
  }

  host.SetIsLastPart(is_last_part);
  bool restored;
  if (filter != nullptr) {
    StateFilterAdapter adapter(*filter);
    restored = system->system.RestoreState(host, &adapter);
  } else {
    restored = system->system.RestoreState(host);
  }
  if (!restored) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "Jolt refused the saved state");
  }
  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while reading the state");
  }
  if (host.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the stream ended before the state did");
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemSaveBodyStateLockedStream(
    const ZJoltPhysicsSystem *system, const ZJoltBody *body,
    const ZJoltStream *stream) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, body, stream)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to save through");
  }

  const JPH::Body *jbody = zjolt::ToJolt(body);
  zjolt::HostStream host(*stream);
  zjolt::WriteStreamHeader(host, kBodyStreamMagic);

  uint8_t extra[4];
  zjolt::WriteLE32(extra, BodyCategory(*jbody));
  host.WriteBytes(extra, sizeof(extra));

  system->system.SaveBodyState(*jbody, host);
  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while writing the body state");
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemRestoreBodyStateLockedStream(
    ZJoltPhysicsSystem *system, ZJoltBody *body, const ZJoltStream *stream) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, body, stream)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "stream needs read, is_eof and is_failed to restore through");
  }

  zjolt::HostStream host(*stream);
  const ZJoltResult header = zjolt::ReadStreamHeader(
      host, kBodyStreamMagic,
      "not a body state saved by zjoltPhysicsSystemSaveBodyStateLockedStream");
  if (header != ZJOLT_RESULT_OK) return header;

  uint8_t extra[4];
  host.ReadBytes(extra, sizeof(extra));
  if (host.IsFailed()) {
    return zjolt::SetError(
        ZJOLT_RESULT_IO_ERROR,
        "the stream failed while reading the body state header");
  }
  if (host.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the stream ended before the body state header did");
  }

  JPH::Body *jbody = zjolt::ToJolt(body);
  if (zjolt::ReadLE32(extra) != BodyCategory(*jbody)) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "saved from a body of a different motion type; a body state restore "
        "puts motion back onto the same kind of body it was saved from");
  }

  system->system.RestoreBodyState(*jbody, host);
  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while reading the body state");
  }
  if (host.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the stream ended before the body state did");
  }
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
