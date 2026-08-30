//===----------------------------------------------------------------------===//
// zjolt — character controllers.
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
// Character: CharacterVirtual is not a rigid body — a shape swept through
// the world under the host's control, which is what makes it feel like a
// game character rather than a barrel. Optionally carries an inner rigid
// body so other bodies and queries can see it.
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
  /// The solver stops once this little of the step is left to simulate. Jolt's
  /// early-out; smaller costs iterations, larger leaves motion unsimulated.
  float min_time_remaining;
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
  /// The id the inner body is created with, instead of a generated one — what
  /// makes a rebuilt world hand the same character the same id, which is what
  /// a replay or a rollback compares against. ZJOLT_BODY_ID_INVALID for a
  /// generated one. Ignored without `inner_body_shape`; the same rules as
  /// zjoltBodyCreateWithId otherwise.
  ZJoltBodyId inner_body_id_override;
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

/// The velocity `body_b` counts as having for ground-velocity purposes: its
/// own linear/angular velocity (zero if STATIC), adjusted by this
/// character's listener if one is installed via zjoltCharacterSetListener —
/// CharacterVirtual::GetAdjustedBodyVelocity, the building block
/// zjoltCharacterUpdateGroundVelocity itself uses. ZJOLT_RESULT_BODY_NOT_FOUND
/// for a stale `body_b`.
ZJOLT_API ZJoltResult zjoltCharacterGetAdjustedBodyVelocity(
    const ZJoltCharacter *character, ZJoltBodyId body_b,
    ZJoltVec3 *out_linear_velocity, ZJoltVec3 *out_angular_velocity);

/// What this character's own velocity would be if it stood on an object at
/// `center_of_mass` moving with `linear_velocity`/`angular_velocity` —
/// CharacterVirtual::CalculateCharacterGroundVelocity, the other building
/// block zjoltCharacterUpdateGroundVelocity uses, exposed standalone for a
/// hypothetical ground body rather than the character's actual one. Reads
/// only the character's current position; writes nothing.
ZJOLT_API void zjoltCharacterCalculateGroundVelocity(
    const ZJoltCharacter *character, const ZJoltRVec3 *center_of_mass,
    const ZJoltVec3 *linear_velocity, const ZJoltVec3 *angular_velocity,
    float delta_time, ZJoltVec3 *out);

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

/// Gives the inner rigid body a shape of its own, independent of the one the
/// character sweeps. zjoltCharacterSetShape already keeps the inner body in
/// step with the new swept shape (what a crouch wants); this is for undoing
/// that — putting back an `inner_body_shape` DIFFERENT from `shape` (a cheap
/// cast proxy, say) after a swap overwrote it. ZJOLT_RESULT_INVALID_ARGUMENT
/// for a character with no inner body, rather than silently doing nothing.
ZJOLT_API ZJoltResult zjoltCharacterSetInnerBodyShape(
    ZJoltCharacter *character, const ZJoltShape *shape);

//===----------------------------------------------------------------------===//
// CharacterBase, on the virtual character. A character's identifier (as
// opposed to a body's) is a CharacterID — used for deterministic sort order
// and to name a character in a contact callback after it may already be gone.
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

//===----------------------------------------------------------------------===//
// The supporting volume: a plane in the character's LOCAL space, (normal,
// distance) with signed distance `dot(normal, p) + distance`. A contact
// behind it may support the character; one in front only collides — the
// second half of the ground test (max slope angle rejects by normal, this
// rejects by where on the shape it landed), keeping a wall the character
// leans on from reading as floor.
//
// zjoltCharacterCreate installs one an inner radius above the shape's
// lowest point. Putting it AT the lowest point is the tempting mistake that
// breaks every slope: a capsule on a ramp touches the plane on the SIDE of
// its bottom cap, above the lowest point.
//===----------------------------------------------------------------------===//

ZJOLT_API void zjoltCharacterGetSupportingVolume(
    const ZJoltCharacter *character, ZJoltVec3 *out_normal,
    float *out_distance);
/// A zero-length `normal` is refused rather than installed: the plane it
/// describes has no side, and a character with one never reports ground again.
ZJOLT_API ZJoltResult zjoltCharacterSetSupportingVolume(
    ZJoltCharacter *character, const ZJoltVec3 *normal, float distance);

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

/// Groups a run of Update/WalkStairs/StickToFloor/ExtendedUpdate calls so
/// the contact listener sees one added/persisted/removed transition per
/// contact for the run, not one per call. Every Start needs exactly one
/// matching Finish — an unbalanced pair asserts in debug and leaks
/// bookkeeping into the next frame otherwise.
ZJOLT_API void zjoltCharacterStartTrackingContactChanges(
    ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterFinishTrackingContactChanges(
    ZJoltCharacter *character);

//===----------------------------------------------------------------------===//
// Stair walking and floor sticking, standalone: zjoltCharacterUpdate already
// runs both through ZJoltCharacterUpdateSettings (Jolt's ExtendedUpdate).
// These are the two pieces alone, for a host that wants to run them at a
// different point in its frame, or with different parameters per call.
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
// Asking about a placement the character is not at: "what would happen if"
// — whether a stance fits, whether a spawn point is clear, what a shot
// would hit — without moving it. zjolt_transformed.h owns the handle the
// first call hands back; declared here too since naming an opaque type
// doesn't require its defining header.
//
// zjoltCharacterCheckCollision is not zjoltCollideShapeAll with the
// character's shape: it applies the character's own back-face mode,
// active-edge handling, enhanced internal edge removal and padding, skips
// its inner body, and also tests the character-vs-character list —
// unreachable from zjolt_query.h, since those characters are not in the
// broad phase.
//===----------------------------------------------------------------------===//

typedef struct ZJoltTransformedShape ZJoltTransformedShape;

/// The character's current volume, placed where the character is, as a
/// standalone queryable shape — release with zjoltTransformedShapeDestroy.
/// The only way to hit a virtual character with a cast (Jolt never puts one
/// in the broad phase, so zjoltCastRay* misses it without an inner body). A
/// SNAPSHOT: reposition with zjoltTransformedShapeSetWorldTransform, whose
/// position is centre-of-mass, NOT what zjoltCharacterGetPosition returns.
ZJOLT_API ZJoltResult zjoltCharacterGetTransformedShape(
    const ZJoltCharacter *character, ZJoltTransformedShape **out);

/// One overlap reported by zjoltCharacterCheckCollision.
///
/// Deliberately not ZJoltCollideShapeHit: a virtual character is not a body,
/// so an overlap with another one has no body id to report and would come
/// back indistinguishable from an overlap with nothing.
typedef struct ZJoltCharacterCollisionHit {
  /// Set when the overlap is with an ordinary body; ZJOLT_BODY_ID_INVALID
  /// when it is with another virtual character — see `character_id`.
  ZJoltBodyId body;
  /// Set when the overlap is with another virtual character;
  /// ZJOLT_CHARACTER_ID_INVALID when it is with an ordinary body.
  ZJoltCharacterId character_id;
  ZJoltSubShapeId sub_shape_id;
  /// Both relative to the `position` the test was run at, not to the world
  /// origin — the same convention every hit in zjolt_query.h uses, and for
  /// the same reason: it is the only one that keeps precision far from the
  /// origin.
  ZJoltVec3 contact_point_on_1;
  ZJoltVec3 contact_point_on_2;
  /// Direction to separate the shapes; its magnitude is meaningless.
  /// `-normalize(penetration_axis)` is the contact normal.
  ZJoltVec3 penetration_axis;
  float penetration_depth;
  /// NULL for an overlap with another virtual character, which carries no
  /// material. @see ZJoltRayCastHit::material for the borrowing rule.
  const ZJoltPhysicsMaterial *material;
} ZJoltCharacterCollisionHit;

/// Everything `shape` would overlap if the character stood at
/// `position`/`rotation`, without moving it or touching its contacts.
/// Count-then-fill: `out_hits` NULL reports the count; a short `capacity`
/// reports ZJOLT_RESULT_BUFFER_TOO_SMALL with the true count. `shape`/
/// `rotation` NULL use the character's current shape/rotation;
/// `movement_direction`/`filters` NULL mean no hint / collide with everything.
ZJOLT_API ZJoltResult zjoltCharacterCheckCollision(
    const ZJoltCharacter *character, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *movement_direction,
    float max_separation_distance, const ZJoltShape *shape,
    const ZJoltQueryFilters *filters, ZJoltCharacterCollisionHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// CharacterContactListener: fires as a virtual character finds, keeps and
// loses contacts. A NULL field behaves as Jolt's own default (accept every
// contact, change nothing). A callback names the character by
// ZJoltCharacterId, not handle — match it against zjoltCharacterGetId,
// since Jolt hands this a raw internal CharacterVirtual pointer that cannot
// be turned back into a ZJoltCharacter.
//
// NOTHING MAY UNWIND OUT OF ONE OF THESE — Jolt is built with exceptions off.
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

/// Whatever zjoltCharacterSetListener last installed, or NULL for none. The
/// character does not own it and destroying the character does not destroy
/// it — this reports who is attached, it does not transfer anything.
ZJOLT_API ZJoltCharacterContactListener *zjoltCharacterGetListener(
    const ZJoltCharacter *character);

//===----------------------------------------------------------------------===//
// Character-vs-character collision: CharacterVirtual is not in the broad
// phase, so nothing sees it unless told to look. This is Jolt's
// CharacterVsCharacterCollisionSimple — a plain list, checked by brute
// force, NOT thread-safe (one CharacterVirtual may check it at a time).
// zjoltCharacterVsCharacterCollisionCreateCustom below builds the same kind
// of handle from a host's own callbacks instead, for a spatial structure or
// anything else the brute-force scan cannot do.
//===----------------------------------------------------------------------===//

typedef struct ZJoltCharacterVsCharacterCollision ZJoltCharacterVsCharacterCollision;

ZJOLT_API ZJoltResult zjoltCharacterVsCharacterCollisionCreate(
    ZJoltCharacterVsCharacterCollision **out);
ZJOLT_API void zjoltCharacterVsCharacterCollisionDestroy(
    ZJoltCharacterVsCharacterCollision *collision);

/// Meaningful only on a handle from zjoltCharacterVsCharacterCollisionCreate;
/// on one from zjoltCharacterVsCharacterCollisionCreateCustom, which has no
/// list of its own, these do nothing.
ZJOLT_API void zjoltCharacterVsCharacterCollisionAdd(
    ZJoltCharacterVsCharacterCollision *collision, ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterVsCharacterCollisionRemove(
    ZJoltCharacterVsCharacterCollision *collision,
    const ZJoltCharacter *character);

/// NULL detaches: the character then collides with no other character.
ZJOLT_API void zjoltCharacterSetCharacterVsCharacterCollision(
    ZJoltCharacter *character, ZJoltCharacterVsCharacterCollision *collision);

//===----------------------------------------------------------------------===//
// A custom character-vs-character broad phase: CharacterVsCharacterCollision
// is the interface CharacterVirtual::Update asks two questions through while
// moving — CollideCharacter ("who overlaps this character") and CastCharacter
// ("who is in the way of this sweep") — handed to a host as callbacks instead
// of CharacterVsCharacterCollisionSimple's brute-force scan.
// Each callback gets a VISITOR in place of Jolt's own collector: call it once
// per OTHER character to test `character` against; zjolt runs the real test
// on every candidate the visitor accepts. Return false once you have seen
// enough (calling again after is harmless). `character` is named by id —
// match zjoltCharacterGetId on whichever ZJoltCharacter this was installed
// on; `candidate` is one of the host's own handles, `visit`/`visit_user`
// valid only for the call. NOTHING MAY UNWIND OUT OF ONE OF THESE.
//===----------------------------------------------------------------------===//

/// `candidate` is a character the host itself created and still holds.
/// Returns whether to keep visiting: false once the underlying test has seen
/// enough, true to be offered another.
typedef bool (*ZJoltCharacterVsCharacterVisitFn)(void *visit_user,
                                                 ZJoltCharacter *candidate);

/// `center_of_mass_transform` is `character`'s placement for this test, world
/// space, valid only for the duration of the call.
typedef void (*ZJoltCollideCharacterFn)(
    void *user, ZJoltCharacterId character,
    const ZJoltRMat44 *center_of_mass_transform,
    ZJoltCharacterVsCharacterVisitFn visit, void *visit_user);

/// As ZJoltCollideCharacterFn, but `character`'s shape is being swept along
/// `direction` from `center_of_mass_transform` rather than tested in place.
typedef void (*ZJoltCastCharacterFn)(
    void *user, ZJoltCharacterId character,
    const ZJoltRMat44 *center_of_mass_transform, const ZJoltVec3 *direction,
    ZJoltCharacterVsCharacterVisitFn visit, void *visit_user);

typedef struct ZJoltCharacterVsCharacterCollisionCallbacks {
  ZJoltCollideCharacterFn collide_character;
  ZJoltCastCharacterFn cast_character;
  void *user;
} ZJoltCharacterVsCharacterCollisionCallbacks;

/// Builds a collision object from `callbacks` in place of the brute-force
/// list; install through zjoltCharacterSetCharacterVsCharacterCollision and
/// release with zjoltCharacterVsCharacterCollisionDestroy. `callbacks` is
/// copied; only its `user` pointer needs to outlive this call. A NULL field
/// behaves as Jolt's own default override would if it had one: that
/// question is never asked, so `character` collides with nothing through it.
ZJOLT_API ZJoltResult zjoltCharacterVsCharacterCollisionCreateCustom(
    const ZJoltCharacterVsCharacterCollisionCallbacks *callbacks,
    ZJoltCharacterVsCharacterCollision **out);

//===----------------------------------------------------------------------===//
// RigidCharacter (Jolt's Character.h): a real dynamic rigid body the host
// drives by setting its velocity each frame, like the virtual character,
// but collision response, sleeping and being pushed fall out of the
// ordinary rigid-body solver instead of a hand-rolled sweep. Prefer this
// when the world needs to see the character as an ordinary body — felt by
// a trigger, knocked back by an explosion.
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

/// Builds the character's rigid body but does not add it to the system yet
/// — call zjoltRigidCharacterAddToPhysicsSystem to make it move and collide.
/// Fails with ZJOLT_RESULT_OUT_OF_MEMORY, and creates nothing, if `system`
/// is already holding max_bodies bodies. The character borrows `system` for
/// its lifetime and must be destroyed before it.
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

/// @see zjoltCharacterGetSupportingVolume for what the plane is and how it is
/// spelled. The rigid character checks it in zjoltRigidCharacterPostSimulation
/// rather than during a sweep, so a change takes effect at the next one.
ZJOLT_API void zjoltRigidCharacterGetSupportingVolume(
    const ZJoltRigidCharacter *character, ZJoltVec3 *out_normal,
    float *out_distance);
ZJOLT_API ZJoltResult zjoltRigidCharacterSetSupportingVolume(
    ZJoltRigidCharacter *character, const ZJoltVec3 *normal, float distance);

/// The character's body and shape as a standalone queryable handle — release
/// it with zjoltTransformedShapeDestroy. Less essential than the virtual
/// character's: this one has a real body, so zjoltCastRay* against the system
/// already finds it. @see zjoltCharacterGetTransformedShape for the snapshot
/// rule, which applies here too.
ZJOLT_API ZJoltResult zjoltRigidCharacterGetTransformedShape(
    const ZJoltRigidCharacter *character, ZJoltTransformedShape **out);

/// Everything `shape` would overlap if the character stood at
/// `position`/`rotation`. @see zjoltCharacterCheckCollision for the protocol
/// and the NULL arguments. There are no filters: Jolt builds them from the
/// character's own object layer, skipping its own body and every sensor.
/// `character_id` on every hit is ZJOLT_CHARACTER_ID_INVALID: a rigid
/// character collides through the ordinary broad phase, no character list.
ZJOLT_API ZJoltResult zjoltRigidCharacterCheckCollision(
    const ZJoltRigidCharacter *character, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *movement_direction,
    float max_separation_distance, const ZJoltShape *shape,
    ZJoltCharacterCollisionHit *out_hits, uint32_t capacity,
    uint32_t *out_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_CHARACTER_H_
