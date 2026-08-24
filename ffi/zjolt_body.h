//===----------------------------------------------------------------------===//
// zjolt — bodies, bulk read-back and body locks.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_BODY_H_
#define ZJOLT_BODY_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Bodies
//===----------------------------------------------------------------------===//

typedef struct ZJoltBodyDesc {
  ZJoltRVec3 position;
  ZJoltQuat rotation;
  ZJoltVec3 linear_velocity;
  ZJoltVec3 angular_velocity;
  /// Required. A reference is taken for the lifetime of the body.
  const ZJoltShape *shape;
  uint64_t user_data;
  ZJoltObjectLayer object_layer;
  ZJoltMotionType motion_type;
  ZJoltMotionQuality motion_quality;
  ZJoltAllowedDofs allowed_dofs;
  ZJoltOverrideMassProperties override_mass_properties;
  /// Used when override_mass_properties is CALCULATE_INERTIA.
  float mass;
  /// Lets a static body be switched to kinematic or dynamic later.
  bool allow_dynamic_or_kinematic;
  bool is_sensor;
  bool allow_sleeping;
  /// Extra work to suppress ghost collisions on internal mesh edges.
  bool enhanced_internal_edge_removal;
  float friction;
  float restitution;
  float linear_damping;
  float angular_damping;
  float max_linear_velocity;
  float max_angular_velocity;
  float gravity_factor;
} ZJoltBodyDesc;

/// Fills `desc` with Jolt's own defaults. Call this first and then overwrite;
/// the defaults are not all zero and are not all obvious.
ZJOLT_API void zjoltBodyDescInit(ZJoltBodyDesc *desc);

/// Creates a body without adding it to the simulation.
ZJOLT_API ZJoltResult zjoltBodyCreate(ZJoltPhysicsSystem *system,
                                      const ZJoltBodyDesc *desc,
                                      ZJoltBodyId *out);

ZJOLT_API ZJoltResult zjoltBodyCreateAndAdd(ZJoltPhysicsSystem *system,
                                            const ZJoltBodyDesc *desc,
                                            ZJoltActivation activation,
                                            ZJoltBodyId *out);

/// Removes the body if it is still added, then destroys it. The id becomes
/// stale; further calls with it report ZJOLT_RESULT_BODY_NOT_FOUND rather than
/// touching whatever body was created next.
ZJOLT_API void zjoltBodyDestroy(ZJoltPhysicsSystem *system, ZJoltBodyId body);

ZJOLT_API void zjoltBodyAdd(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                            ZJoltActivation activation);
ZJOLT_API void zjoltBodyRemove(ZJoltPhysicsSystem *system, ZJoltBodyId body);
ZJOLT_API bool zjoltBodyIsAdded(const ZJoltPhysicsSystem *system,
                                ZJoltBodyId body);
ZJOLT_API bool zjoltBodyIsActive(const ZJoltPhysicsSystem *system,
                                 ZJoltBodyId body);
ZJOLT_API void zjoltBodyActivate(ZJoltPhysicsSystem *system, ZJoltBodyId body);
ZJOLT_API void zjoltBodyDeactivate(ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body);

ZJOLT_API void zjoltBodySetMotionType(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body, ZJoltMotionType type,
                                      ZJoltActivation activation);
ZJOLT_API ZJoltMotionType zjoltBodyGetMotionType(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// Teleport: places the body immediately, ignoring the collision that puts it
/// there. `rotation` may be NULL to keep the current orientation.
ZJOLT_API void zjoltBodySetPositionAndRotation(ZJoltPhysicsSystem *system,
                                               ZJoltBodyId body,
                                               const ZJoltRVec3 *position,
                                               const ZJoltQuat *rotation,
                                               ZJoltActivation activation);
/// Either out-parameter may be NULL.
ZJOLT_API void zjoltBodyGetPositionAndRotation(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltRVec3 *out_position,
    ZJoltQuat *out_rotation);
ZJOLT_API void zjoltBodyGetCenterOfMassPosition(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltRVec3 *out);

/// Drives a kinematic body toward a target over `delta_time`, so it pushes
/// dynamic bodies out of the way instead of teleporting through them. This is
/// how a moving platform or an animated character should move.
ZJOLT_API void zjoltBodyMoveKinematic(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body,
                                      const ZJoltRVec3 *target_position,
                                      const ZJoltQuat *target_rotation,
                                      float delta_time);

ZJOLT_API void zjoltBodySetLinearVelocity(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodyGetLinearVelocity(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body, ZJoltVec3 *out);
ZJOLT_API void zjoltBodySetAngularVelocity(ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body,
                                           const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodyGetAngularVelocity(const ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body, ZJoltVec3 *out);

/// Forces and torques accumulate until the next step consumes them; impulses
/// change velocity immediately. `point` is in world space.
ZJOLT_API void zjoltBodyAddForce(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                 const ZJoltVec3 *force);
ZJOLT_API void zjoltBodyAddForceAtPoint(ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body,
                                        const ZJoltVec3 *force,
                                        const ZJoltRVec3 *point);
ZJOLT_API void zjoltBodyAddTorque(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                  const ZJoltVec3 *torque);
ZJOLT_API void zjoltBodyAddImpulse(ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body,
                                   const ZJoltVec3 *impulse);
ZJOLT_API void zjoltBodyAddImpulseAtPoint(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *impulse,
                                          const ZJoltRVec3 *point);
ZJOLT_API void zjoltBodyAddAngularImpulse(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *angular_impulse);

/// Replaces the shape. With `update_mass_properties` the body's mass and
/// inertia are recomputed from the new shape.
ZJOLT_API void zjoltBodySetShape(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                 const ZJoltShape *shape,
                                 bool update_mass_properties,
                                 ZJoltActivation activation);

ZJOLT_API void zjoltBodySetObjectLayer(ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body,
                                       ZJoltObjectLayer layer);
ZJOLT_API ZJoltObjectLayer zjoltBodyGetObjectLayer(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

ZJOLT_API void zjoltBodySetUserData(ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body, uint64_t user_data);
ZJOLT_API uint64_t zjoltBodyGetUserData(const ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body);

ZJOLT_API void zjoltBodySetFriction(ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body, float friction);
ZJOLT_API float zjoltBodyGetFriction(const ZJoltPhysicsSystem *system,
                                     ZJoltBodyId body);
ZJOLT_API void zjoltBodySetRestitution(ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body, float restitution);
ZJOLT_API float zjoltBodyGetRestitution(const ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body);
ZJOLT_API void zjoltBodySetGravityFactor(ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body, float factor);
ZJOLT_API float zjoltBodyGetGravityFactor(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body);

/// The material of one leaf of the body's shape. @see zjoltShapeGetMaterial
/// for what a material is and for the sub-shape id rules.
///
/// THIS IS NOT A WAY TO TEST WHETHER A BODY EXISTS. Jolt takes a body lock and
/// returns the shared default material when it fails, so a body that was
/// destroyed and a body whose shape has no materials of its own answer
/// identically. That answer is forwarded rather than replaced with an invented
/// NULL, because reporting a failure Jolt did not report would be a different
/// contract, not a stricter one. Use zjoltBodyIsAdded when the question is
/// about the body, or take the lock yourself and read the shape through
/// zjoltBodyGetShapeLocked.
///
/// NULL only when `system` is NULL or the library is not initialised.
ZJOLT_API const ZJoltPhysicsMaterial *zjoltBodyGetMaterial(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltSubShapeId sub_shape_id);

/// Tells the system a body's shape changed underneath it — which is what the
/// zjoltShapeMutableCompound* calls do.
///
/// `previous_center_of_mass` is the shape's centre of mass BEFORE the change,
/// read with zjoltShapeGetCenterOfMass; the body is moved so that its geometry
/// stays where it was. With `update_mass_properties` the mass and inertia are
/// recomputed from the new shape. Without this call the broad phase and the
/// contact cache go on describing the shape as it used to be.
ZJOLT_API void zjoltBodyNotifyShapeChanged(
    ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltVec3 *previous_center_of_mass, bool update_mass_properties,
    ZJoltActivation activation);

//===----------------------------------------------------------------------===//
// Bulk read-back
//
// The accessors above are one ABI crossing and one body lock each. That is the
// right shape for the occasional query and the wrong shape for the thing a
// renderer does every frame: read the transform of every body that moved.
//
// These two calls exist so that per-frame read-back is two crossings and one
// lock acquisition rather than 2N of each. Reach for them in the frame loop
// and for the single-body accessors everywhere else.
//===----------------------------------------------------------------------===//

/// Ids of the bodies that are awake, which is the set whose transforms can
/// have changed since the last step.
///
/// Two-call protocol: `*out_count` always receives the true number, so a
/// capacity of 0 with `out_ids` NULL is a size query.
ZJOLT_API ZJoltResult zjoltPhysicsSystemGetActiveBodies(
    const ZJoltPhysicsSystem *system, ZJoltBodyId *out_ids, uint32_t capacity,
    uint32_t *out_count);

/// Every body in the system, awake or not, in no particular order.
ZJOLT_API ZJoltResult zjoltPhysicsSystemGetBodies(
    const ZJoltPhysicsSystem *system, ZJoltBodyId *out_ids, uint32_t capacity,
    uint32_t *out_count);

/// Reads `count` body transforms under a single lock.
///
/// `out_positions` and `out_rotations` are parallel arrays of `count` entries;
/// either may be NULL if that half is not wanted. An id that no longer names a
/// live body writes the identity transform and is reported through
/// `out_missing` (which may be NULL) rather than failing the whole batch —
/// a body destroyed between the step and the read is normal, not an error.
ZJOLT_API ZJoltResult zjoltBodyGetTransforms(const ZJoltPhysicsSystem *system,
                                             const ZJoltBodyId *ids,
                                             uint32_t count,
                                             ZJoltRVec3 *out_positions,
                                             ZJoltQuat *out_rotations,
                                             uint32_t *out_missing);

/// As zjoltBodyGetTransforms, but reads centre-of-mass positions and linear
/// velocities — what an interpolating or audio-driven host wants.
ZJOLT_API ZJoltResult zjoltBodyGetMotions(const ZJoltPhysicsSystem *system,
                                          const ZJoltBodyId *ids,
                                          uint32_t count,
                                          ZJoltRVec3 *out_center_of_mass,
                                          ZJoltVec3 *out_linear_velocities,
                                          uint32_t *out_missing);

//===----------------------------------------------------------------------===//
// Body locks
//
// The accessors above take a lock per call, which is the right default and the
// wrong tool for reading six properties of the same body. A lock holds the
// body still for a scope, and hands out a borrowed ZJoltBody that must not
// outlive it.
//
// This maps Jolt's RAII lock one-to-one, which means the release call is the
// caller's responsibility. `body` is NULL when the id was stale — check it,
// and release either way.
//===----------------------------------------------------------------------===//

typedef struct ZJoltBodyLock {
  /// NULL when the body id does not name a live body.
  ZJoltBody *body;
  /// Implementation detail. Do not read, do not copy this struct while held.
  void *_reserved[2];
} ZJoltBodyLock;

/// Takes a shared lock. Several readers may hold one at once.
ZJOLT_API void zjoltBodyLockRead(const ZJoltPhysicsSystem *system,
                                 ZJoltBodyId body, ZJoltBodyLock *out_lock);
ZJOLT_API void zjoltBodyLockReadRelease(ZJoltBodyLock *lock);

/// Takes an exclusive lock. Required before any zjoltBodyMut* call.
ZJOLT_API void zjoltBodyLockWrite(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                  ZJoltBodyLock *out_lock);
ZJOLT_API void zjoltBodyLockWriteRelease(ZJoltBodyLock *lock);

ZJOLT_API ZJoltBodyId zjoltBodyGetId(const ZJoltBody *body);
ZJOLT_API void zjoltBodyGetPosition(const ZJoltBody *body, ZJoltRVec3 *out);
ZJOLT_API void zjoltBodyGetRotation(const ZJoltBody *body, ZJoltQuat *out);
ZJOLT_API void zjoltBodyGetCenterOfMassPositionLocked(const ZJoltBody *body,
                                                      ZJoltRVec3 *out);
ZJOLT_API void zjoltBodyGetLinearVelocityLocked(const ZJoltBody *body,
                                                ZJoltVec3 *out);
ZJOLT_API void zjoltBodyGetAngularVelocityLocked(const ZJoltBody *body,
                                                 ZJoltVec3 *out);
ZJOLT_API uint64_t zjoltBodyGetUserDataLocked(const ZJoltBody *body);
ZJOLT_API ZJoltObjectLayer zjoltBodyGetObjectLayerLocked(const ZJoltBody *body);
ZJOLT_API ZJoltMotionType zjoltBodyGetMotionTypeLocked(const ZJoltBody *body);
ZJOLT_API bool zjoltBodyIsActiveLocked(const ZJoltBody *body);
ZJOLT_API bool zjoltBodyIsSensorLocked(const ZJoltBody *body);
/// Borrowed; valid while the body is alive. Takes no reference.
ZJOLT_API const ZJoltShape *zjoltBodyGetShapeLocked(const ZJoltBody *body);
ZJOLT_API void zjoltBodyGetWorldBounds(const ZJoltBody *body, ZJoltAABox *out);

/// Requires a write lock. Velocity changes made here bypass activation, so
/// activate the body separately if it may be asleep.
ZJOLT_API void zjoltBodyMutSetLinearVelocity(ZJoltBody *body,
                                             const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodyMutSetAngularVelocity(ZJoltBody *body,
                                              const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodyMutSetUserData(ZJoltBody *body, uint64_t user_data);
ZJOLT_API void zjoltBodyMutSetFriction(ZJoltBody *body, float friction);
ZJOLT_API void zjoltBodyMutSetRestitution(ZJoltBody *body, float restitution);
ZJOLT_API void zjoltBodyMutAddImpulse(ZJoltBody *body,
                                      const ZJoltVec3 *impulse);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_BODY_H_
