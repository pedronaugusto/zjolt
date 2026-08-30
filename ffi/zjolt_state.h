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

#include "zjolt_character.h"
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

//===----------------------------------------------------------------------===//
// Characters
//
// A JPH::PhysicsSystem does not save its characters: CharacterVirtual.h:266
// says so outright for the virtual kind, and the rigid kind's CharacterBase
// ground state is not in its body's state either. A whole-system save is
// handed the characters it must cover, and refuses with
// ZJOLT_RESULT_STATE_INCOMPLETE when the set is not every character the
// system holds — the alternative being a snapshot that restores the world
// without the player in it, and reports success.
//===----------------------------------------------------------------------===//

/// The characters a whole-system save or restore covers. NULL means none,
/// which only a system holding no characters accepts.
///
/// The order is part of the payload: a restore must pass the same characters
/// in the same order the save did. Both arrays are borrowed for the call.
typedef struct ZJoltStateCharacters {
  /// Virtual characters (zjoltCharacterCreate). May be NULL when `count` is 0.
  ZJoltCharacter *const *characters;
  uint32_t count;
  /// Rigid characters (zjoltRigidCharacterCreate). May be NULL when
  /// `rigid_count` is 0.
  ZJoltRigidCharacter *const *rigid_characters;
  uint32_t rigid_count;
} ZJoltStateCharacters;

/// Writes state into `buffer`, which NULL sizes into `*out_size`; a short
/// `capacity` returns ZJOLT_RESULT_BUFFER_TOO_SMALL with the size still
/// written. `filter` NULL saves everything `state` names. `characters` folds
/// the characters into the same payload, and is ZJOLT_RESULT_STATE_INCOMPLETE
/// unless it names every one `system` holds, each once — a PARTIAL save (@see
/// ZJoltStateFilter) exempted. Not callable during a step.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSaveState(
    const ZJoltPhysicsSystem *system, uint32_t state,
    const ZJoltStateFilter *filter, const ZJoltStateCharacters *characters,
    void *buffer, size_t capacity, size_t *out_size);

/// As zjoltPhysicsSystemSaveState, writing through `stream`. Omits length
/// and CRC-32. ZJOLT_RESULT_IO_ERROR if `stream` fails.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSaveStateStream(
    const ZJoltPhysicsSystem *system, uint32_t state,
    const ZJoltStateFilter *filter, const ZJoltStateCharacters *characters,
    const ZJoltStream *stream);

/// Restores saved state. `filter`'s should_restore_contact runs if
/// non-NULL. `is_last_part` false for every part but the last of a
/// partial sequence, true otherwise. `characters` must name the same
/// characters, in the same order, as the save did.
/// ZJOLT_RESULT_BAD_FORMAT for a malformed, mismatched, or unrecognized
/// buffer; a failure partway through can leave the system PARTLY RESTORED.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRestoreState(
    ZJoltPhysicsSystem *system, const void *data, size_t size,
    const ZJoltStateFilter *filter, const ZJoltStateCharacters *characters,
    bool is_last_part);

/// As zjoltPhysicsSystemRestoreState, reading through `stream`.
/// ZJOLT_RESULT_IO_ERROR or ZJOLT_RESULT_BAD_FORMAT on a bad stream.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRestoreStateStream(
    ZJoltPhysicsSystem *system, const ZJoltStream *stream,
    const ZJoltStateFilter *filter, const ZJoltStateCharacters *characters,
    bool is_last_part);

/// One virtual character's state, on its own — CharacterVirtual::SaveState.
/// Same buffer protocol as zjoltPhysicsSystemSaveState: NULL `buffer` sizes
/// into `*out_size`. For replicating ONE character; a rollback of the whole
/// simulation wants zjoltPhysicsSystemSaveState's `characters` instead, which
/// keeps system and characters in one payload that cannot be half-restored.
ZJOLT_API ZJoltResult zjoltCharacterSaveState(const ZJoltCharacter *character,
                                              void *buffer, size_t capacity,
                                              size_t *out_size);

/// As zjoltCharacterSaveState, writing through `stream`.
ZJOLT_API ZJoltResult zjoltCharacterSaveStateStream(
    const ZJoltCharacter *character, const ZJoltStream *stream);

/// Puts back what zjoltCharacterSaveState wrote —
/// CharacterVirtual::RestoreState. ZJOLT_RESULT_BAD_FORMAT for a buffer that
/// is not one of those, truncated, or damaged.
ZJOLT_API ZJoltResult zjoltCharacterRestoreState(ZJoltCharacter *character,
                                                 const void *data,
                                                 size_t size);

/// As zjoltCharacterRestoreState, reading through `stream`.
ZJOLT_API ZJoltResult zjoltCharacterRestoreStateStream(
    ZJoltCharacter *character, const ZJoltStream *stream);

/// One rigid character's CharacterBase state — ground state, ground body,
/// ground position/normal/velocity. Its POSITION and VELOCITY live in its
/// body and are saved by zjoltPhysicsSystemSaveState like any other body's;
/// this is the part that is not.
ZJOLT_API ZJoltResult zjoltRigidCharacterSaveState(
    const ZJoltRigidCharacter *character, void *buffer, size_t capacity,
    size_t *out_size);

/// As zjoltRigidCharacterSaveState, writing through `stream`.
ZJOLT_API ZJoltResult zjoltRigidCharacterSaveStateStream(
    const ZJoltRigidCharacter *character, const ZJoltStream *stream);

/// Puts back what zjoltRigidCharacterSaveState wrote.
ZJOLT_API ZJoltResult zjoltRigidCharacterRestoreState(
    ZJoltRigidCharacter *character, const void *data, size_t size);

/// As zjoltRigidCharacterRestoreState, reading through `stream`.
ZJOLT_API ZJoltResult zjoltRigidCharacterRestoreStateStream(
    ZJoltRigidCharacter *character, const ZJoltStream *stream);

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
