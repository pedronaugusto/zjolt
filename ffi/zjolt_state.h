//===----------------------------------------------------------------------===//
// zjolt — save and restore simulation state.
//
// Captures what a step changes: position, rotation, velocity, sleep,
// contact cache. Restore requires matching bodies, system, and
// configuration, or fails. Buffer format matches zjoltShapeSave's
// container; it catches a wrong, truncated, or damaged file, not a
// crafted payload with a matching checksum.

#ifndef ZJOLT_STATE_H_
#define ZJOLT_STATE_H_

#include "zjolt_constraint.h"
#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Bit mask of what a save writes. Restore reads exactly what was saved.
/// ZJOLT_STATE_RECORDER_STATE_ALL for a rollback; less makes the restored
/// step merely close to the saved one, not identical.
typedef enum ZJoltStateRecorderState {
  ZJOLT_STATE_RECORDER_STATE_NONE = 0,
  /// Previous step's delta time and world gravity.
  ZJOLT_STATE_RECORDER_STATE_GLOBAL = 1 << 0,
  /// Position, rotation, velocity, sleep state of every body.
  ZJOLT_STATE_RECORDER_STATE_BODIES = 1 << 1,
  /// Contact cache, including warm-start impulses.
  ZJOLT_STATE_RECORDER_STATE_CONTACTS = 1 << 2,
  /// Constraint state, including vehicles. Required for a world with joints.
  ZJOLT_STATE_RECORDER_STATE_CONSTRAINTS = 1 << 3,
  ZJOLT_STATE_RECORDER_STATE_ALL = 0b1111,
} ZJoltStateRecorderState;

/// NULL callback accepts every item of that kind. A PARTIAL save (any
/// non-NULL should_save_*) carries no body-set digest; restoring
/// overlapping partial saves is undefined.
typedef struct ZJoltStateFilter {
  /// NULL saves every body.
  bool (*should_save_body)(void *user, ZJoltBodyId body);
  /// NULL saves every constraint.
  bool (*should_save_constraint)(void *user, ZJoltConstraint *constraint);
  /// NULL saves every contact.
  bool (*should_save_contact)(void *user, ZJoltBodyId body1, ZJoltBodyId body2);
  /// NULL restores every contact the payload carries.
  bool (*should_restore_contact)(void *user, ZJoltBodyId body1,
                                 ZJoltBodyId body2);
  void *user;
} ZJoltStateFilter;

/// Writes state into `buffer`. `buffer` NULL sizes into `*out_size`; a
/// short `capacity` returns ZJOLT_RESULT_BUFFER_TOO_SMALL with the size
/// still written. `filter` NULL saves everything `state` names. Not
/// callable during a step.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSaveState(
    const ZJoltPhysicsSystem *system, uint32_t state,
    const ZJoltStateFilter *filter, void *buffer, size_t capacity,
    size_t *out_size);

/// As zjoltPhysicsSystemSaveState, writing through `stream`. Omits length
/// and CRC-32. ZJOLT_RESULT_IO_ERROR if `stream` fails.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSaveStateStream(
    const ZJoltPhysicsSystem *system, uint32_t state,
    const ZJoltStateFilter *filter, const ZJoltStream *stream);

/// Restores saved state. `filter`'s should_restore_contact runs if
/// non-NULL. `is_last_part` false for every part but the last of a
/// partial sequence, true otherwise. ZJOLT_RESULT_BAD_FORMAT for a
/// malformed, mismatched, or unrecognized buffer; a failure partway
/// through can leave the system PARTLY RESTORED.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRestoreState(
    ZJoltPhysicsSystem *system, const void *data, size_t size,
    const ZJoltStateFilter *filter, bool is_last_part);

/// As zjoltPhysicsSystemRestoreState, reading through `stream`.
/// ZJOLT_RESULT_IO_ERROR or ZJOLT_RESULT_BAD_FORMAT on a bad stream.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRestoreStateStream(
    ZJoltPhysicsSystem *system, const ZJoltStream *stream,
    const ZJoltStateFilter *filter, bool is_last_part);

/// Reports the first payload byte where `state_a` and `state_b` differ.
/// A length mismatch diverges at the shorter length. Compares payload
/// only, not the container header. ZJOLT_RESULT_BAD_FORMAT if either
/// buffer fails to parse.
ZJOLT_API ZJoltResult zjoltPhysicsSystemCompareState(
    const void *state_a, size_t size_a, const void *state_b, size_t size_b,
    bool *out_diverged, size_t *out_offset);

/// As zjoltPhysicsSystemSaveState, for `body` alone. `body` must already
/// be locked (zjoltBodyLockRead/Write). Not callable during a step.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSaveBodyStateLocked(
    const ZJoltPhysicsSystem *system, const ZJoltBody *body, void *buffer,
    size_t capacity, size_t *out_size);

/// As zjoltPhysicsSystemSaveBodyStateLocked, writing through `stream`.
/// ZJOLT_RESULT_IO_ERROR if `stream` fails.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSaveBodyStateLockedStream(
    const ZJoltPhysicsSystem *system, const ZJoltBody *body,
    const ZJoltStream *stream);

/// Restores one body's saved state. Requires zjoltBodyLockWrite, not a
/// read lock: restoring can move the body between the active and
/// sleeping lists. Rejects a dynamic/static motion-type mismatch against
/// the saved data before reading the payload.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRestoreBodyStateLocked(
    ZJoltPhysicsSystem *system, ZJoltBody *body, const void *data,
    size_t size);

/// As zjoltPhysicsSystemRestoreBodyStateLocked, reading through `stream`.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRestoreBodyStateLockedStream(
    ZJoltPhysicsSystem *system, ZJoltBody *body, const ZJoltStream *stream);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_STATE_H_
