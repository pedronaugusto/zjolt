//===----------------------------------------------------------------------===//
// zjolt — saving and restoring the state of a simulation.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// The state a step CHANGES, as a flat buffer: for rollback in a networked
// game, for a replay, or for a determinism check. Positions, rotations,
// velocities, sleep state, and the solver's contact cache.
//
// What it deliberately does not carry is everything the simulation only reads:
// shapes, layers, motion types, masses, friction, restitution, collision
// groups, the settings in zjolt_system.h. A restore expects to find the same
// bodies, in the same system, configured the same way, and puts their motion
// back. It is a snapshot of where the world is, not of what the world is.
//
// So a body created or destroyed since the save is not something a restore can
// paper over, and zjoltPhysicsSystemRestoreState reports failure instead of
// guessing. Save and restore the same set of bodies, or rebuild the set first.
//
// The buffer follows the same container convention as zjoltShapeSave — a magic
// tag, a container version, this library's config id, the Jolt version, the
// payload length and a CRC-32, all validated before Jolt reads a byte — and
// carries the same caveat: it rejects the wrong file, a truncated file and a
// damaged file, and it is not a defence against a crafted payload that carries
// a matching checksum.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_STATE_H_
#define ZJOLT_STATE_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Which parts of a simulation a save writes, as a bit mask.
///
/// The mask is recorded in the payload, so a restore is not told what it is
/// reading — it restores exactly the parts that were saved.
///
/// ZJOLT_STATE_RECORDER_STATE_ALL is what rollback wants. Anything less leaves
/// some part of the solver holding data from a different frame, and the step
/// after the restore is then merely close to the one that was saved rather
/// than identical to it.
typedef enum ZJoltStateRecorderState {
  ZJOLT_STATE_RECORDER_STATE_NONE = 0,
  /// The previous step's delta time, and the world gravity.
  ZJOLT_STATE_RECORDER_STATE_GLOBAL = 1 << 0,
  /// Position, rotation, velocity and sleep state of every body.
  ZJOLT_STATE_RECORDER_STATE_BODIES = 1 << 1,
  /// The contact cache, including the accumulated impulses the solver warm
  /// starts the next step from. Leaving this out is what makes a restored
  /// world settle differently from the one it was copied out of.
  ZJOLT_STATE_RECORDER_STATE_CONTACTS = 1 << 2,
  /// Constraint state. zjolt exposes no constraints yet, so this part is empty
  /// today and costs nothing to include.
  ZJOLT_STATE_RECORDER_STATE_CONSTRAINTS = 1 << 3,
  ZJOLT_STATE_RECORDER_STATE_ALL = 0b1111,
} ZJoltStateRecorderState;

/// Writes the state into `buffer`.
///
/// Two-call protocol: `buffer` NULL reports the size in `*out_size` and writes
/// nothing, and a `capacity` short of that reports
/// ZJOLT_RESULT_BUFFER_TOO_SMALL with the required size still written. The
/// size is not stable across steps — a world that gained contacts needs more
/// — so ask each time rather than caching it.
///
/// `state` is a mask of ZJoltStateRecorderState. Pass
/// ZJOLT_STATE_RECORDER_STATE_ALL unless you know why you are not; a mask of
/// ZJOLT_STATE_RECORDER_STATE_NONE is accepted and writes an empty payload
/// that restores nothing.
///
/// Do not call this during a step. It reads bodies without locking them,
/// because it is meant to run between steps, where nothing is moving.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSaveState(
    const ZJoltPhysicsSystem *system, uint32_t state, void *buffer,
    size_t capacity, size_t *out_size);

/// Puts the saved state back.
///
/// ZJOLT_RESULT_BAD_FORMAT covers five different kinds of wrong input: not a
/// zjolt state buffer at all, written by a different zjolt or a different
/// precision setting, truncated or carrying trailing bytes, damaged in
/// storage, and — the one that is not about the buffer — a payload Jolt
/// refused because the world no longer holds the bodies it names.
///
/// The first four leave the system untouched, because they are all decided
/// before Jolt sees a byte. The fifth does not: a payload Jolt started reading
/// and then rejected leaves the world partly restored, and the only sound
/// recovery from there is to restore a state that does apply.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRestoreState(
    ZJoltPhysicsSystem *system, const void *data, size_t size);

//===----------------------------------------------------------------------===//
// One body at a time
//
// PhysicsSystem::SaveBodyState / RestoreBodyState, for a caller that only
// wants to roll one body back rather than the whole simulation — a grabbed
// object dropped by a networking correction, say, without touching anything
// else's contacts. Position, rotation, velocity and sleep state; nothing a
// whole-system restore's body-set digest needs to guard, since there is only
// ever the one body named here.
//
// `body` must already be locked — see zjoltBodyLockRead / zjoltBodyLockWrite
// in zjolt_body.h. Jolt's own Body::RestoreState reads a different number of
// bytes for a body with motion properties than for one without, so restoring
// a dynamic body's saved state onto a static body (or the reverse) would
// misread the stream rather than fail cleanly; the container records the
// body's motion type and rejects that case before Jolt reads a byte.
//===----------------------------------------------------------------------===//

/// Two-call protocol, as zjoltPhysicsSystemSaveState.
///
/// Do not call this during a step, for the same reason as
/// zjoltPhysicsSystemSaveState: it reads the body without taking a fresh
/// lock, on the assumption that the caller's lock already has it still.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSaveBodyStateLocked(
    const ZJoltPhysicsSystem *system, const ZJoltBody *body, void *buffer,
    size_t capacity, size_t *out_size);

/// Puts one body's saved state back. See zjoltPhysicsSystemSaveBodyStateLocked
/// for the container, and zjoltBodyLockWrite for the lock this needs — a
/// write lock, since restoring can move the body between the active and
/// sleeping lists.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRestoreBodyStateLocked(
    ZJoltPhysicsSystem *system, ZJoltBody *body, const void *data,
    size_t size);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_STATE_H_
