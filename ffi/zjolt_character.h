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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_CHARACTER_H_
