//===----------------------------------------------------------------------===//
// zjolt — character controllers.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_CHARACTER_H_
#define ZJOLT_CHARACTER_H_

#include "zjolt_core.h"

// A character is moved by casting it through the world, so it takes the same
// filters a query does.
#include "zjolt_query.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Character
//
// CharacterVirtual is not a rigid body: it is a shape that is swept through
// the world under the host's control, which is what makes it feel like a game
// character rather than a barrel. It can optionally carry an inner rigid body
// so that other bodies and queries can see it.
//===----------------------------------------------------------------------===//

typedef struct ZJoltCharacterDesc {
  /// Required. Its centre should sit at the character's centre; use a rotated-
  /// translated shape to put a capsule's base at the origin.
  const ZJoltShape *shape;
  ZJoltRVec3 position;
  ZJoltQuat rotation;
  /// Which way is up for this character. Need not be the world up.
  ZJoltVec3 up;
  ZJoltVec3 shape_offset;
  uint64_t user_data;
  /// Radians. Ground steeper than this reports ON_STEEP_GROUND.
  float max_slope_angle;
  float mass;
  /// Force the character can apply to push dynamic bodies.
  float max_strength;
  /// How far to look ahead for contacts. 0 will get the character stuck.
  float predictive_contact_distance;
  /// Gap kept between the shape and geometry, so sweeps hit less.
  float character_padding;
  float penetration_recovery_speed;
  float collision_tolerance;
  float hit_reduction_cos_max_angle;
  uint32_t max_collision_iterations;
  uint32_t max_constraint_iterations;
  uint32_t max_num_hits;
  ZJoltBackFaceMode back_face_mode;
  bool enhanced_internal_edge_removal;
  /// Optional rigid body that follows the character so the world can see it.
  /// It is what makes the character visible to ray casts and to other bodies,
  /// since CharacterVirtual itself is not in the broad phase. NULL for none.
  const ZJoltShape *inner_body_shape;
  ZJoltObjectLayer inner_body_layer;
} ZJoltCharacterDesc;

/// Fills `desc` with Jolt's defaults. `shape` is left NULL and is required.
ZJOLT_API void zjoltCharacterDescInit(ZJoltCharacterDesc *desc);

/// Extra behaviour layered on top of the plain move: sticking to the floor on
/// the way down a slope, and stepping up stairs.
typedef struct ZJoltCharacterUpdateSettings {
  /// Zero disables floor sticking.
  ZJoltVec3 stick_to_floor_step_down;
  /// Zero disables stair walking.
  ZJoltVec3 walk_stairs_step_up;
  float walk_stairs_min_step_forward;
  float walk_stairs_step_forward_test;
  float walk_stairs_cos_angle_forward_contact;
  ZJoltVec3 walk_stairs_step_down_extra;
} ZJoltCharacterUpdateSettings;

ZJOLT_API void zjoltCharacterUpdateSettingsInit(
    ZJoltCharacterUpdateSettings *settings);

/// The character borrows `system` for its lifetime and must be destroyed
/// before it.
ZJOLT_API ZJoltResult zjoltCharacterCreate(ZJoltPhysicsSystem *system,
                                           const ZJoltCharacterDesc *desc,
                                           ZJoltCharacter **out);
ZJOLT_API void zjoltCharacterDestroy(ZJoltCharacter *character);

/// Moves the character by its current velocity for `delta_time`, resolving
/// collision. Set the velocity first with zjoltCharacterSetLinearVelocity.
///
/// `settings` may be NULL for a plain move with no stair or floor handling.
/// `filters` may be NULL to collide with everything.
ZJOLT_API ZJoltResult zjoltCharacterUpdate(
    ZJoltCharacter *character, float delta_time, const ZJoltVec3 *gravity,
    const ZJoltCharacterUpdateSettings *settings,
    const ZJoltQueryFilters *filters);

ZJOLT_API void zjoltCharacterGetPosition(const ZJoltCharacter *character,
                                         ZJoltRVec3 *out);
ZJOLT_API void zjoltCharacterSetPosition(ZJoltCharacter *character,
                                         const ZJoltRVec3 *position);
ZJOLT_API void zjoltCharacterGetRotation(const ZJoltCharacter *character,
                                         ZJoltQuat *out);
ZJOLT_API void zjoltCharacterSetRotation(ZJoltCharacter *character,
                                         const ZJoltQuat *rotation);
ZJOLT_API void zjoltCharacterGetLinearVelocity(const ZJoltCharacter *character,
                                               ZJoltVec3 *out);
ZJOLT_API void zjoltCharacterSetLinearVelocity(ZJoltCharacter *character,
                                               const ZJoltVec3 *velocity);

ZJOLT_API ZJoltGroundState zjoltCharacterGetGroundState(
    const ZJoltCharacter *character);
/// True for ON_GROUND and ON_STEEP_GROUND.
ZJOLT_API bool zjoltCharacterIsSupported(const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterGetGroundNormal(const ZJoltCharacter *character,
                                             ZJoltVec3 *out);
ZJOLT_API void zjoltCharacterGetGroundVelocity(const ZJoltCharacter *character,
                                               ZJoltVec3 *out);
ZJOLT_API void zjoltCharacterGetGroundPosition(const ZJoltCharacter *character,
                                               ZJoltRVec3 *out);
ZJOLT_API ZJoltBodyId zjoltCharacterGetGroundBodyId(
    const ZJoltCharacter *character);
ZJOLT_API uint64_t zjoltCharacterGetGroundUserData(
    const ZJoltCharacter *character);

/// Recomputes the ground velocity, for reading after moving the ground body.
ZJOLT_API void zjoltCharacterUpdateGroundVelocity(ZJoltCharacter *character);

/// Swaps the shape — crouching. Fails without changing anything if the new
/// shape would be more than `max_penetration_depth` inside the world.
ZJOLT_API ZJoltResult zjoltCharacterSetShape(
    ZJoltCharacter *character, const ZJoltShape *shape,
    float max_penetration_depth, const ZJoltQueryFilters *filters,
    bool *out_changed);

ZJOLT_API const ZJoltShape *zjoltCharacterGetShape(
    const ZJoltCharacter *character);
ZJOLT_API ZJoltBodyId zjoltCharacterGetInnerBodyId(
    const ZJoltCharacter *character);

//===----------------------------------------------------------------------===//
// CharacterBase, on the virtual character
//
// Everything below was on CharacterBase or CharacterVirtual but missing above.
// The identifier of a character (as opposed to a body) is a CharacterID —
// used for deterministic sort order and to name a character in a contact
// callback after it may already be gone.
//===----------------------------------------------------------------------===//

/// The numeric value of a Jolt CharacterID. Two characters never share one,
/// and ZJOLT_CHARACTER_ID_INVALID is what a contact reports in the field that
/// does not apply — a body contact's character_id_b, a character contact's
/// body_b.
typedef uint32_t ZJoltCharacterId;
#define ZJOLT_CHARACTER_ID_INVALID ((ZJoltCharacterId)0xffffffffu)

/// A contact between a virtual character and a body, or another character.
/// Shared by zjoltCharacterGetActiveContacts and CharacterContactListener.
typedef struct ZJoltCharacterContact {
  /// Valid when colliding with an ordinary body; ZJOLT_BODY_ID_INVALID when
  /// colliding with another character (see character_id_b instead).
  ZJoltBodyId body_b;
  /// Valid when colliding with another character; ZJOLT_CHARACTER_ID_INVALID
  /// when colliding with an ordinary body.
  ZJoltCharacterId character_id_b;
  ZJoltSubShapeId sub_shape_id_b;
  /// Where the character makes contact.
  ZJoltRVec3 position;
  /// Velocity of the contact point (e.g. a moving platform).
  ZJoltVec3 linear_velocity;
  /// Points toward the character.
  ZJoltVec3 contact_normal;
  /// Flipped from contact_normal when the contact is back-facing.
  ZJoltVec3 surface_normal;
  /// <= 0 is an actual contact; > 0 is predictive.
  float distance;
  float fraction;
  ZJoltMotionType motion_type_b;
  bool is_sensor_b;
  uint64_t user_data;
  /// NULL when colliding with a character rather than a material-bearing
  /// body.
  const ZJoltPhysicsMaterial *material;
  /// False when a predictive contact never became a real one.
  bool had_collision;
  /// True when OnContactValidate/OnCharacterContactValidate rejected this
  /// contact, or the body is a sensor.
  bool was_discarded;
  bool can_push_character;
  bool is_back_facing_contact;
} ZJoltCharacterContact;

typedef struct ZJoltCharacterContactSettings {
  /// True when the other object can push the character.
  bool can_push_character;
  /// True when the character can push the other object. Only takes effect
  /// against a rigid body; another CharacterVirtual can only be moved by its
  /// own Update.
  bool can_receive_impulses;
} ZJoltCharacterContactSettings;

/// Uniquely identifies this character, assigned when it was created.
ZJOLT_API ZJoltCharacterId zjoltCharacterGetId(const ZJoltCharacter *character);

ZJOLT_API void zjoltCharacterGetUp(const ZJoltCharacter *character,
                                   ZJoltVec3 *out);
ZJOLT_API void zjoltCharacterSetUp(ZJoltCharacter *character,
                                   const ZJoltVec3 *up);

/// Radians. Ground steeper than this reports ON_STEEP_GROUND.
ZJOLT_API void zjoltCharacterSetMaxSlopeAngle(ZJoltCharacter *character,
                                              float radians);
/// Jolt stores the angle as its cosine; this returns that, not radians.
ZJOLT_API float zjoltCharacterGetCosMaxSlopeAngle(
    const ZJoltCharacter *character);
ZJOLT_API bool zjoltCharacterIsSlopeTooSteep(const ZJoltCharacter *character,
                                             const ZJoltVec3 *normal);

/// NULL when the character has never touched anything, or `character` is
/// NULL.
ZJOLT_API const ZJoltPhysicsMaterial *zjoltCharacterGetGroundMaterial(
    const ZJoltCharacter *character);
ZJOLT_API ZJoltSubShapeId zjoltCharacterGetGroundSubShapeId(
    const ZJoltCharacter *character);

ZJOLT_API float zjoltCharacterGetMass(const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterSetMass(ZJoltCharacter *character, float mass);
ZJOLT_API float zjoltCharacterGetMaxStrength(const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterSetMaxStrength(ZJoltCharacter *character,
                                            float max_strength);
ZJOLT_API float zjoltCharacterGetPenetrationRecoverySpeed(
    const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterSetPenetrationRecoverySpeed(
    ZJoltCharacter *character, float speed);
ZJOLT_API bool zjoltCharacterGetEnhancedInternalEdgeRemoval(
    const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterSetEnhancedInternalEdgeRemoval(
    ZJoltCharacter *character, bool apply);
ZJOLT_API float zjoltCharacterGetCharacterPadding(
    const ZJoltCharacter *character);
ZJOLT_API uint32_t zjoltCharacterGetMaxNumHits(const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterSetMaxNumHits(ZJoltCharacter *character,
                                           uint32_t max_hits);
ZJOLT_API float zjoltCharacterGetHitReductionCosMaxAngle(
    const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterSetHitReductionCosMaxAngle(
    ZJoltCharacter *character, float cos_max_angle);
/// True if the last Update/WalkStairs/ExtendedUpdate hit more contacts than
/// max_num_hits and had to discard some by distance. A character that trips
/// this often needs a higher max_num_hits or simpler nearby geometry.
ZJOLT_API bool zjoltCharacterGetMaxHitsExceeded(const ZJoltCharacter *character);

ZJOLT_API void zjoltCharacterGetShapeOffset(const ZJoltCharacter *character,
                                            ZJoltVec3 *out);
/// Setting this on the fly can teleport the shape into collision; prefer
/// setting it once at creation.
ZJOLT_API void zjoltCharacterSetShapeOffset(ZJoltCharacter *character,
                                            const ZJoltVec3 *offset);

ZJOLT_API uint64_t zjoltCharacterGetUserData(const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterSetUserData(ZJoltCharacter *character,
                                         uint64_t user_data);

/// Clamps `desired_velocity` so that it will not carry the character further
/// onto ground steeper than max_slope_angle. Call before Update and feed the
/// result to SetLinearVelocity; this does not read or write anything.
ZJOLT_API void zjoltCharacterCancelVelocityTowardsSteepSlopes(
    const ZJoltCharacter *character, const ZJoltVec3 *desired_velocity,
    ZJoltVec3 *out);

/// Groups a run of Update / WalkStairs / StickToFloor / ExtendedUpdate calls
/// so the contact listener sees one added/persisted/removed transition per
/// contact for the whole run, instead of one per call. Every
/// zjoltCharacterStartTrackingContactChanges needs exactly one matching
/// zjoltCharacterFinishTrackingContactChanges — Jolt asserts on an unbalanced
/// pair in a debug build, and an unfinished one leaks its bookkeeping into the
/// next frame.
ZJOLT_API void zjoltCharacterStartTrackingContactChanges(
    ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterFinishTrackingContactChanges(
    ZJoltCharacter *character);

//===----------------------------------------------------------------------===//
// Stair walking and floor sticking, standalone
//
// zjoltCharacterUpdate already runs both of these through
// ZJoltCharacterUpdateSettings when it is given one (that is Jolt's own
// ExtendedUpdate). These are the two pieces on their own, for a host that
// wants to run them at a different point in its frame than Update, or with
// different parameters per call.
//===----------------------------------------------------------------------===//

/// True if the character just moved into a slope steeper than it can climb,
/// which is the usual trigger for calling zjoltCharacterWalkStairs.
ZJOLT_API bool zjoltCharacterCanWalkStairs(const ZJoltCharacter *character,
                                           const ZJoltVec3 *linear_velocity);

/// Casts up, forward and back down to try to place the character one stair
/// higher. `out_stepped` reports whether a valid step was found; a refused
/// step changes nothing and is a normal outcome, not an error.
///
/// `filters` may be NULL to collide with everything.
ZJOLT_API ZJoltResult zjoltCharacterWalkStairs(
    ZJoltCharacter *character, float delta_time, const ZJoltVec3 *step_up,
    const ZJoltVec3 *step_forward, const ZJoltVec3 *step_forward_test,
    const ZJoltVec3 *step_down_extra, const ZJoltQueryFilters *filters,
    bool *out_stepped);

/// Projects the character down onto the floor by up to `step_down` when a
/// floor is found within that distance. Meant to run right after a horizontal
/// move that lost contact with a floor the character is still effectively
/// standing on. `out_stuck` is false when no floor was found within range,
/// which is a normal outcome (the character is airborne), not an error.
ZJOLT_API ZJoltResult zjoltCharacterStickToFloor(ZJoltCharacter *character,
                                                 const ZJoltVec3 *step_down,
                                                 const ZJoltQueryFilters *filters,
                                                 bool *out_stuck);

/// Recomputes contacts at the character's current position. Call after
/// teleporting a character so its ground state reflects where it landed
/// instead of where it was.
ZJOLT_API ZJoltResult zjoltCharacterRefreshContacts(
    ZJoltCharacter *character, const ZJoltQueryFilters *filters);

/// Every contact the last Update/WalkStairs/etc. call found, count-then-fill.
/// `out_contacts` NULL reports the count in `*out_count` and writes nothing; a
/// `capacity` short of that reports ZJOLT_RESULT_BUFFER_TOO_SMALL with the
/// true count still written. Only contacts with had_collision set are actual
/// touches — the rest were predictive and never became real.
ZJOLT_API ZJoltResult zjoltCharacterGetActiveContacts(
    const ZJoltCharacter *character, ZJoltCharacterContact *out_contacts,
    uint32_t capacity, uint32_t *out_count);

ZJOLT_API bool zjoltCharacterHasCollidedWithBody(
    const ZJoltCharacter *character, ZJoltBodyId body);
ZJOLT_API bool zjoltCharacterHasCollidedWithCharacter(
    const ZJoltCharacter *character, ZJoltCharacterId other_character_id);

//===----------------------------------------------------------------------===//
// CharacterContactListener
//
// Fires as a virtual character finds, keeps and loses contacts. Crosses as
// function pointers plus a `void *user`, never a mirrored C++ vtable — any
// field left NULL behaves as Jolt's own default override (accept every
// contact, change nothing).
//
// A callback names the character it was called about by ZJoltCharacterId, not
// by handle: Jolt hands this code a raw pointer to its own internal
// CharacterVirtual, which is not the ZJoltCharacter this API handed out and
// cannot be turned back into one. Match the id against whichever
// ZJoltCharacter(s) you attached this listener to with zjoltCharacterGetId.
//
// NOTHING MAY UNWIND OUT OF ONE OF THESE. Jolt is built with exceptions off,
// so an exception crossing a callback is std::terminate — and from a Zig host,
// a panic crossing one skips lock destructors and can wedge the next Update.
//===----------------------------------------------------------------------===//

typedef void (*ZJoltCharacterOnAdjustBodyVelocityFn)(
    void *user, ZJoltCharacterId character, ZJoltBodyId body2,
    ZJoltVec3 *io_linear_velocity, ZJoltVec3 *io_angular_velocity);
typedef bool (*ZJoltCharacterOnContactValidateFn)(
    void *user, ZJoltCharacterId character,
    const ZJoltCharacterContact *contact);
typedef void (*ZJoltCharacterOnContactAddedFn)(
    void *user, ZJoltCharacterId character,
    const ZJoltCharacterContact *contact,
    ZJoltCharacterContactSettings *io_settings);
typedef void (*ZJoltCharacterOnContactPersistedFn)(
    void *user, ZJoltCharacterId character,
    const ZJoltCharacterContact *contact,
    ZJoltCharacterContactSettings *io_settings);
typedef void (*ZJoltCharacterOnContactRemovedFn)(void *user,
                                                  ZJoltCharacterId character,
                                                  ZJoltBodyId body_id2,
                                                  ZJoltSubShapeId sub_shape_id2);
typedef bool (*ZJoltCharacterOnCharacterContactValidateFn)(
    void *user, ZJoltCharacterId character,
    const ZJoltCharacterContact *contact);
typedef void (*ZJoltCharacterOnCharacterContactAddedFn)(
    void *user, ZJoltCharacterId character,
    const ZJoltCharacterContact *contact,
    ZJoltCharacterContactSettings *io_settings);
typedef void (*ZJoltCharacterOnCharacterContactPersistedFn)(
    void *user, ZJoltCharacterId character,
    const ZJoltCharacterContact *contact,
    ZJoltCharacterContactSettings *io_settings);
typedef void (*ZJoltCharacterOnCharacterContactRemovedFn)(
    void *user, ZJoltCharacterId character, ZJoltCharacterId other_character_id,
    ZJoltSubShapeId sub_shape_id2);
typedef void (*ZJoltCharacterOnContactSolveFn)(
    void *user, ZJoltCharacterId character, ZJoltBodyId body_id2,
    ZJoltSubShapeId sub_shape_id2, const ZJoltRVec3 *contact_position,
    const ZJoltVec3 *contact_normal, const ZJoltVec3 *contact_velocity,
    const ZJoltPhysicsMaterial *contact_material,
    const ZJoltVec3 *character_velocity, ZJoltVec3 *io_new_character_velocity);
typedef void (*ZJoltCharacterOnCharacterContactSolveFn)(
    void *user, ZJoltCharacterId character, ZJoltCharacterId other_character,
    ZJoltSubShapeId sub_shape_id2, const ZJoltRVec3 *contact_position,
    const ZJoltVec3 *contact_normal, const ZJoltVec3 *contact_velocity,
    const ZJoltPhysicsMaterial *contact_material,
    const ZJoltVec3 *character_velocity, ZJoltVec3 *io_new_character_velocity);

typedef struct ZJoltCharacterContactListenerCallbacks {
  ZJoltCharacterOnAdjustBodyVelocityFn on_adjust_body_velocity;
  ZJoltCharacterOnContactValidateFn on_contact_validate;
  ZJoltCharacterOnContactAddedFn on_contact_added;
  ZJoltCharacterOnContactPersistedFn on_contact_persisted;
  ZJoltCharacterOnContactRemovedFn on_contact_removed;
  ZJoltCharacterOnCharacterContactValidateFn on_character_contact_validate;
  ZJoltCharacterOnCharacterContactAddedFn on_character_contact_added;
  ZJoltCharacterOnCharacterContactPersistedFn on_character_contact_persisted;
  ZJoltCharacterOnCharacterContactRemovedFn on_character_contact_removed;
  ZJoltCharacterOnContactSolveFn on_contact_solve;
  ZJoltCharacterOnCharacterContactSolveFn on_character_contact_solve;
  void *user;
} ZJoltCharacterContactListenerCallbacks;

typedef struct ZJoltCharacterContactListener ZJoltCharacterContactListener;

/// `callbacks` is copied; only its `user` pointer needs to outlive this call.
ZJOLT_API ZJoltResult zjoltCharacterContactListenerCreate(
    const ZJoltCharacterContactListenerCallbacks *callbacks,
    ZJoltCharacterContactListener **out);
/// Detach with zjoltCharacterSetListener(character, NULL) first if the
/// character this is attached to is still alive.
ZJOLT_API void zjoltCharacterContactListenerDestroy(
    ZJoltCharacterContactListener *listener);

/// NULL detaches whatever listener is currently attached.
///
/// Returns a result rather than nothing for the same reason
/// zjoltPhysicsSystemSetContactListener does: a listener that failed to
/// install is a character that silently stops reporting its contacts, and
/// nothing later says so.
ZJOLT_API ZJoltResult zjoltCharacterSetListener(
    ZJoltCharacter *character, ZJoltCharacterContactListener *listener);

//===----------------------------------------------------------------------===//
// Character-vs-character collision
//
// CharacterVirtual is not in the broad phase, so nothing sees it unless it is
// told to look. This is Jolt's CharacterVsCharacterCollisionSimple: a plain
// list of characters, checked by brute force. It is not thread-safe — only
// one CharacterVirtual may be checking collision against it at a time.
//===----------------------------------------------------------------------===//

typedef struct ZJoltCharacterVsCharacterCollision ZJoltCharacterVsCharacterCollision;

ZJOLT_API ZJoltResult zjoltCharacterVsCharacterCollisionCreate(
    ZJoltCharacterVsCharacterCollision **out);
ZJOLT_API void zjoltCharacterVsCharacterCollisionDestroy(
    ZJoltCharacterVsCharacterCollision *collision);

ZJOLT_API void zjoltCharacterVsCharacterCollisionAdd(
    ZJoltCharacterVsCharacterCollision *collision, ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterVsCharacterCollisionRemove(
    ZJoltCharacterVsCharacterCollision *collision,
    const ZJoltCharacter *character);

/// NULL detaches: the character then collides with no other character.
ZJOLT_API void zjoltCharacterSetCharacterVsCharacterCollision(
    ZJoltCharacter *character, ZJoltCharacterVsCharacterCollision *collision);

//===----------------------------------------------------------------------===//
// RigidCharacter
//
// Jolt's own name for this is "Character" — spent above on the swept virtual
// one, which this binding settled on first. This is the other character base
// class, Jolt/Physics/Character/Character.h: a real dynamic rigid body that
// the host drives by setting its velocity every frame, same as the virtual
// one, but collision response, sleeping and being pushed by other dynamics
// fall out of the ordinary rigid-body solver instead of a hand-rolled sweep.
// Prefer this one when the world needs to see the character as an ordinary
// body — felt by a trigger volume, knocked back by an explosion — and can
// live with the solver's collision response instead of a per-frame sweep.
//===----------------------------------------------------------------------===//

typedef struct ZJoltRigidCharacter ZJoltRigidCharacter;

typedef struct ZJoltRigidCharacterDesc {
  /// Required. Its centre should sit at the character's centre; use a
  /// rotated-translated shape to put a capsule's base at the origin.
  const ZJoltShape *shape;
  ZJoltRVec3 position;
  ZJoltQuat rotation;
  uint64_t user_data;
  /// Which way is up for this character. Need not be the world up.
  ZJoltVec3 up;
  /// Radians. Ground steeper than this reports ON_STEEP_GROUND.
  float max_slope_angle;
  bool enhanced_internal_edge_removal;
  ZJoltObjectLayer layer;
  float mass;
  float friction;
  /// Multiplies the system's gravity for this character alone.
  float gravity_factor;
  /// A mask of ZJoltAllowedDofs, not a single enumerator.
  uint32_t allowed_dofs;
} ZJoltRigidCharacterDesc;

/// Fills `desc` with Jolt's defaults. `shape` is left NULL and is required.
ZJOLT_API void zjoltRigidCharacterDescInit(ZJoltRigidCharacterDesc *desc);

/// Builds the character's rigid body but does not add it to the system yet —
/// call zjoltRigidCharacterAddToPhysicsSystem to make it move and collide.
///
/// Fails with ZJOLT_RESULT_OUT_OF_MEMORY, and creates nothing, if `system` is
/// already holding max_bodies bodies.
///
/// The character borrows `system` for its lifetime and must be destroyed
/// before it.
ZJOLT_API ZJoltResult zjoltRigidCharacterCreate(
    ZJoltPhysicsSystem *system, const ZJoltRigidCharacterDesc *desc,
    ZJoltRigidCharacter **out);
/// Destroys the character's rigid body along with the character. Safe to call
/// whether or not the character was ever added to the physics system.
ZJOLT_API void zjoltRigidCharacterDestroy(ZJoltRigidCharacter *character);

ZJOLT_API void zjoltRigidCharacterAddToPhysicsSystem(
    ZJoltRigidCharacter *character, ZJoltActivation activation);
ZJOLT_API void zjoltRigidCharacterRemoveFromPhysicsSystem(
    ZJoltRigidCharacter *character);
ZJOLT_API void zjoltRigidCharacterActivate(ZJoltRigidCharacter *character);

/// Call once after every ZJoltPhysicsSystem step so ground state reflects
/// where the character ended up.
ZJOLT_API void zjoltRigidCharacterPostSimulation(
    ZJoltRigidCharacter *character, float max_separation_distance);

ZJOLT_API void zjoltRigidCharacterSetLinearAndAngularVelocity(
    ZJoltRigidCharacter *character, const ZJoltVec3 *linear_velocity,
    const ZJoltVec3 *angular_velocity);
ZJOLT_API void zjoltRigidCharacterGetLinearVelocity(
    const ZJoltRigidCharacter *character, ZJoltVec3 *out);
ZJOLT_API void zjoltRigidCharacterSetLinearVelocity(
    ZJoltRigidCharacter *character, const ZJoltVec3 *velocity);
ZJOLT_API void zjoltRigidCharacterAddLinearVelocity(
    ZJoltRigidCharacter *character, const ZJoltVec3 *velocity);
ZJOLT_API void zjoltRigidCharacterAddImpulse(ZJoltRigidCharacter *character,
                                             const ZJoltVec3 *impulse);

ZJOLT_API ZJoltBodyId zjoltRigidCharacterGetBodyId(
    const ZJoltRigidCharacter *character);

ZJOLT_API void zjoltRigidCharacterGetPositionAndRotation(
    const ZJoltRigidCharacter *character, ZJoltRVec3 *out_position,
    ZJoltQuat *out_rotation);
ZJOLT_API void zjoltRigidCharacterSetPositionAndRotation(
    ZJoltRigidCharacter *character, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, ZJoltActivation activation);
ZJOLT_API void zjoltRigidCharacterGetPosition(
    const ZJoltRigidCharacter *character, ZJoltRVec3 *out);
ZJOLT_API void zjoltRigidCharacterSetPosition(ZJoltRigidCharacter *character,
                                              const ZJoltRVec3 *position,
                                              ZJoltActivation activation);
ZJOLT_API void zjoltRigidCharacterGetRotation(
    const ZJoltRigidCharacter *character, ZJoltQuat *out);
ZJOLT_API void zjoltRigidCharacterSetRotation(ZJoltRigidCharacter *character,
                                              const ZJoltQuat *rotation,
                                              ZJoltActivation activation);
ZJOLT_API void zjoltRigidCharacterGetCenterOfMassPosition(
    const ZJoltRigidCharacter *character, ZJoltRVec3 *out);

ZJOLT_API ZJoltObjectLayer zjoltRigidCharacterGetLayer(
    const ZJoltRigidCharacter *character);
ZJOLT_API void zjoltRigidCharacterSetLayer(ZJoltRigidCharacter *character,
                                           ZJoltObjectLayer layer);

/// Swaps the shape — crouching. Fails without changing anything if the new
/// shape would be more than `max_penetration_depth` inside the world.
ZJOLT_API ZJoltResult zjoltRigidCharacterSetShape(
    ZJoltRigidCharacter *character, const ZJoltShape *shape,
    float max_penetration_depth, bool *out_changed);
ZJOLT_API const ZJoltShape *zjoltRigidCharacterGetShape(
    const ZJoltRigidCharacter *character);

/// Uniquely identifies this character, assigned when it was created.
ZJOLT_API ZJoltCharacterId zjoltRigidCharacterGetId(
    const ZJoltRigidCharacter *character);

ZJOLT_API void zjoltRigidCharacterGetUp(const ZJoltRigidCharacter *character,
                                        ZJoltVec3 *out);
ZJOLT_API void zjoltRigidCharacterSetUp(ZJoltRigidCharacter *character,
                                        const ZJoltVec3 *up);
ZJOLT_API void zjoltRigidCharacterSetMaxSlopeAngle(
    ZJoltRigidCharacter *character, float radians);
ZJOLT_API float zjoltRigidCharacterGetCosMaxSlopeAngle(
    const ZJoltRigidCharacter *character);
ZJOLT_API bool zjoltRigidCharacterIsSlopeTooSteep(
    const ZJoltRigidCharacter *character, const ZJoltVec3 *normal);

ZJOLT_API ZJoltGroundState zjoltRigidCharacterGetGroundState(
    const ZJoltRigidCharacter *character);
/// True for ON_GROUND and ON_STEEP_GROUND.
ZJOLT_API bool zjoltRigidCharacterIsSupported(
    const ZJoltRigidCharacter *character);
ZJOLT_API void zjoltRigidCharacterGetGroundPosition(
    const ZJoltRigidCharacter *character, ZJoltRVec3 *out);
ZJOLT_API void zjoltRigidCharacterGetGroundNormal(
    const ZJoltRigidCharacter *character, ZJoltVec3 *out);
ZJOLT_API void zjoltRigidCharacterGetGroundVelocity(
    const ZJoltRigidCharacter *character, ZJoltVec3 *out);
ZJOLT_API const ZJoltPhysicsMaterial *zjoltRigidCharacterGetGroundMaterial(
    const ZJoltRigidCharacter *character);
ZJOLT_API ZJoltBodyId zjoltRigidCharacterGetGroundBodyId(
    const ZJoltRigidCharacter *character);
ZJOLT_API ZJoltSubShapeId zjoltRigidCharacterGetGroundSubShapeId(
    const ZJoltRigidCharacter *character);
ZJOLT_API uint64_t zjoltRigidCharacterGetGroundUserData(
    const ZJoltRigidCharacter *character);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_CHARACTER_H_
