//===----------------------------------------------------------------------===//
// zjolt — bodies, bulk read-back and body locks.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_BODY_H_
#define ZJOLT_BODY_H_

#include "zjolt_core.h"

// For ZJoltCollisionGroup, which ZJoltBodyDesc carries.
#include "zjolt_group.h"

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
  /// Which exceptions this body makes to layer-based collision. The default
  /// zjoltBodyDescInit writes is "no group, no filter", which collides with
  /// everything its object layer allows. A reference is taken on
  /// `collision_group.filter` for the lifetime of the body, exactly as one is
  /// on `shape`; the desc itself takes none.
  ZJoltCollisionGroup collision_group;
  uint64_t user_data;
  ZJoltObjectLayer object_layer;
  ZJoltMotionType motion_type;
  ZJoltMotionQuality motion_quality;
  /// A mask of ZJoltAllowedDofs, not a single enumerator.
  uint32_t allowed_dofs;
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

/// Restarts the clock `time_before_sleep` counts down before an active body is
/// allowed to sleep. A no-op on a body that is already asleep or not added.
ZJOLT_API void zjoltBodyResetSleepTimer(ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body);

/// Which kind of body this is: rigid, or a soft body created through
/// zjoltSoftBodyCreate in zjolt_softbody.h. Worth asking before reaching for
/// anything in zjolt_softbody.h with a body id from a query or a contact.
///
/// ZJOLT_BODY_TYPE_RIGID_BODY when `system` is NULL or the body lock fails,
/// which is Jolt's own default rather than one this binding invented.
typedef enum ZJoltBodyType {
  ZJOLT_BODY_TYPE_RIGID_BODY = 0,
  ZJOLT_BODY_TYPE_SOFT_BODY = 1,
} ZJoltBodyType;

ZJOLT_API ZJoltBodyType zjoltBodyGetBodyType(const ZJoltPhysicsSystem *system,
                                             ZJoltBodyId body);

ZJOLT_API void zjoltBodySetMotionType(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body, ZJoltMotionType type,
                                      ZJoltActivation activation);
ZJOLT_API ZJoltMotionType zjoltBodyGetMotionType(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// How well this body detects collisions when it moves fast. NULL system or a
/// failed body lock reads back ZJOLT_MOTION_QUALITY_DISCRETE.
ZJOLT_API void zjoltBodySetMotionQuality(ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body,
                                         ZJoltMotionQuality quality);
ZJOLT_API ZJoltMotionQuality zjoltBodyGetMotionQuality(
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

/// The body's world transform: the rotation and translation that place the
/// SHAPE's origin, column-major with the translation in the fourth column.
///
/// Identity when `system` is NULL or the body lock fails. That is Jolt's own
/// answer rather than one this binding invented — BodyInterface::
/// GetWorldTransform returns RMat44::sIdentity() on a failed lock
/// (BodyInterface.cpp:535) — so a stale id and a body sitting unrotated at the
/// origin read the same. Ask zjoltBodyIsAdded if the difference matters.
ZJOLT_API void zjoltBodyGetWorldTransform(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body, ZJoltRMat44 *out);

/// As zjoltBodyGetWorldTransform, but placing the body's CENTRE OF MASS rather
/// than the shape's origin. This is the space Jolt simulates in, and the two
/// differ by the shape's centre of mass offset — which is not zero for a
/// capsule, a compound, or anything wrapped in an offset-centre-of-mass shape.
///
/// Identity on a NULL system or a failed body lock, again Jolt's own default
/// (BodyInterface.cpp:544).
ZJOLT_API void zjoltBodyGetCenterOfMassTransform(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltRMat44 *out);

/// The body's inverse inertia tensor, rotated into world space, as a 3x3
/// matrix padded out to 4x4 the way Jolt stores one.
///
/// Only a DYNAMIC body has one, and asking a body that is not is refused with
/// ZJOLT_RESULT_INVALID_ARGUMENT rather than forwarded. Jolt's own
/// BodyInterface::GetInverseInertia checks only whether the body lock
/// succeeded (BodyInterface.cpp:917) and then calls Body::GetInverseInertia,
/// which asserts IsDynamic (Body.inl:122) and, in a build without asserts,
/// dereferences the motion properties a static body does not have. So the
/// identity-on-failed-lock default that call documents is not reachable
/// through here: a stale id is ZJOLT_RESULT_BODY_NOT_FOUND instead.
ZJOLT_API ZJoltResult zjoltBodyGetInverseInertia(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltMat44 *out);

/// As zjoltBodySetPositionAndRotation, but skips the broad-phase update and
/// the activation check entirely when the new pose is close enough to the old
/// one. Worth using in place of the plain teleport when the caller cannot
/// already tell "did this actually move" — a networked snapshot applied every
/// tick, for instance — since that is what would otherwise wake a resting
/// body and dirty its broad-phase node for nothing. `rotation` may be NULL to
/// keep the current orientation.
ZJOLT_API void zjoltBodySetPositionAndRotationWhenChanged(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, ZJoltActivation activation);

/// Drives a kinematic body toward a target over `delta_time`, so it pushes
/// dynamic bodies out of the way instead of teleporting through them. This is
/// how a moving platform or an animated character should move.
ZJOLT_API void zjoltBodyMoveKinematic(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body,
                                      const ZJoltRVec3 *target_position,
                                      const ZJoltQuat *target_rotation,
                                      float delta_time);

/// Sets position, rotation and both velocities in one lock. Cheaper than the
/// equivalent separate calls when restoring a full motion state — a network
/// snapshot, a save-state restore of a single body. Static bodies keep
/// whatever velocity they had (Jolt has nowhere to store one). `rotation` may
/// be NULL to keep the current orientation.
ZJOLT_API void zjoltBodySetPositionRotationAndVelocity(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *linear_velocity,
    const ZJoltVec3 *angular_velocity);

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

/// Sets both in one lock rather than two.
ZJOLT_API void zjoltBodySetLinearAndAngularVelocity(
    ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltVec3 *linear_velocity, const ZJoltVec3 *angular_velocity);
/// Reads both in one lock rather than two. Either out-parameter may be NULL.
ZJOLT_API void zjoltBodyGetLinearAndAngularVelocity(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltVec3 *out_linear_velocity, ZJoltVec3 *out_angular_velocity);

/// Adds to the current linear velocity, clamped the same way
/// zjoltBodySetLinearVelocity is.
ZJOLT_API void zjoltBodyAddLinearVelocity(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *linear_velocity);
ZJOLT_API void zjoltBodyAddLinearAndAngularVelocity(
    ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltVec3 *linear_velocity, const ZJoltVec3 *angular_velocity);

/// Velocity of the point on this body that is currently at `point` (world
/// space), including the contribution from spin. Zero for a static body, and
/// zero when the body lock fails — the same answer, which is why
/// zjoltBodyIsAdded is the way to tell them apart if that matters.
ZJOLT_API void zjoltBodyGetPointVelocity(const ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body,
                                         const ZJoltRVec3 *point,
                                         ZJoltVec3 *out);

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

/// Applies drag and buoyancy for a body partly submerged at
/// `surface_position`/`surface_normal` (surface plane in world space, normal
/// pointing out of the fluid) and activates it on success.
///
/// Returns false, and does nothing, for a body that is not dynamic or whose
/// lock fails — the same false Jolt returns, forwarded rather than replaced
/// with an invented error.
ZJOLT_API bool zjoltBodyApplyBuoyancyImpulse(
    ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltRVec3 *surface_position, const ZJoltVec3 *surface_normal,
    float buoyancy, float linear_drag, float angular_drag,
    const ZJoltVec3 *fluid_velocity, const ZJoltVec3 *gravity,
    float delta_time);

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

/// A static or kinematic body has no motion properties to hold this, so these
/// two are no-ops on one. `velocity` must not be negative — Jolt asserts that
/// in a build with asserts enabled and reads it as-is otherwise, so passing a
/// negative one is undefined rather than refused.
///
/// The getters answer 500 and 15*pi (Jolt's own construction-time defaults,
/// `BodyCreationSettings::mMaxLinearVelocity`/`mMaxAngularVelocity`) when
/// `system` is NULL or the body lock fails — Jolt's own fallback, forwarded
/// rather than replaced with zero.
ZJOLT_API void zjoltBodySetMaxLinearVelocity(ZJoltPhysicsSystem *system,
                                             ZJoltBodyId body, float velocity);
ZJOLT_API float zjoltBodyGetMaxLinearVelocity(const ZJoltPhysicsSystem *system,
                                              ZJoltBodyId body);
ZJOLT_API void zjoltBodySetMaxAngularVelocity(ZJoltPhysicsSystem *system,
                                              ZJoltBodyId body,
                                              float velocity);
ZJOLT_API float zjoltBodyGetMaxAngularVelocity(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// Merging nearby contact manifolds into one, on by default. Turning it off
/// for one body invalidates that body's contact cache, so a pair already
/// resting picks up the change on the next step rather than the one after.
///
/// The getter answers true — Jolt's own default — when `system` is NULL or the
/// body lock fails.
ZJOLT_API void zjoltBodySetUseManifoldReduction(ZJoltPhysicsSystem *system,
                                                ZJoltBodyId body,
                                                bool use_reduction);
ZJOLT_API bool zjoltBodyGetUseManifoldReduction(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// Whether this body reports contacts without responding to them — a trigger
/// volume. Unlocked counterpart of zjoltBodyIsSensorLocked; NULL system or a
/// failed body lock answers false.
ZJOLT_API void zjoltBodySetIsSensor(ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body, bool is_sensor);
ZJOLT_API bool zjoltBodyIsSensor(const ZJoltPhysicsSystem *system,
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

/// Takes an exclusive lock. Required before any zjoltBody*Locked writer.
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
ZJOLT_API void zjoltBodySetLinearVelocityLocked(ZJoltBody *body,
                                             const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodySetAngularVelocityLocked(ZJoltBody *body,
                                              const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodySetUserDataLocked(ZJoltBody *body, uint64_t user_data);
ZJOLT_API void zjoltBodySetFrictionLocked(ZJoltBody *body, float friction);
ZJOLT_API void zjoltBodySetRestitutionLocked(ZJoltBody *body, float restitution);
ZJOLT_API void zjoltBodyAddImpulseLocked(ZJoltBody *body,
                                      const ZJoltVec3 *impulse);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_BODY_H_
