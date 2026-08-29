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
  /// Used when override_mass_properties is MASS_AND_INERTIA_PROVIDED, in
  /// place of both `mass` and the shape's own computed inertia. `inertia` is
  /// a direct (not inverse) tensor, the same convention
  /// zjoltShapeGetMassProperties reads out in — decomposed into principal
  /// axes at construction, so a non-diagonal tensor round-trips to an
  /// equivalent one rather than byte-for-byte.
  ZJoltMassProperties mass_properties_override;
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

/// As zjoltBodyCreate, but the body takes the id the caller names instead of
/// the next one Jolt would assign — for deterministic lockstep networking. `id`
/// must not be ZJOLT_BODY_ID_INVALID, must not already name a live body, and
/// must not set bit 31 (reserved for the broad phase). An id round-tripped from
/// zjoltBodyGetId or zjoltBodyCreate's own `out` always satisfies this; a
/// violation is ZJOLT_RESULT_INVALID_ARGUMENT.
ZJOLT_API ZJoltResult zjoltBodyCreateWithId(ZJoltPhysicsSystem *system,
                                            const ZJoltBodyDesc *desc,
                                            ZJoltBodyId id, ZJoltBodyId *out);

/// As zjoltBodyCreateWithId, followed immediately by zjoltBodyAdd.
ZJOLT_API ZJoltResult zjoltBodyCreateAndAddWithId(ZJoltPhysicsSystem *system,
                                                  const ZJoltBodyDesc *desc,
                                                  ZJoltBodyId id,
                                                  ZJoltActivation activation,
                                                  ZJoltBodyId *out);

/// Overwrites `body`'s state from `desc`, as though it had just been created
/// with it — Body::ApplyBodyCreationSettings. `body` must not currently be
/// added to `system`: ZJOLT_RESULT_INVALID_ARGUMENT if it is, or if `desc`
/// implies motion properties this body was created without (STATIC with
/// allow_dynamic_or_kinematic false cannot gain any here).
ZJOLT_API ZJoltResult zjoltBodyApplyBodyCreationSettings(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, const ZJoltBodyDesc *desc);

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

/// Isolates the 8-bit generation counter a ZJoltBodyId packs into bits
/// [23:30] (bit 31 is the broad phase's reserved bit; bits [0:22] are the
/// body index). A slot reused after its occupant was destroyed gets the
/// next sequence number, distinguishing two ids that share an index.
///
/// Pure bit extraction: no ZJoltPhysicsSystem needed, live or not.
ZJOLT_API uint8_t zjoltBodyIdGetSequenceNumber(ZJoltBodyId id);

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
/// Identity when `system` is NULL or the body lock fails (Jolt's own
/// default) — a stale id and a body sitting unrotated at the origin read
/// the same. Ask zjoltBodyIsAdded if the difference matters.
ZJOLT_API void zjoltBodyGetWorldTransform(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body, ZJoltRMat44 *out);

/// As zjoltBodyGetWorldTransform, but placing the body's CENTRE OF MASS
/// rather than the shape's origin — the space Jolt simulates in. The two
/// differ by the shape's centre of mass offset (nonzero for a capsule, a
/// compound, or an offset-centre-of-mass shape).
///
/// Identity on a NULL system or a failed body lock, Jolt's own default.
ZJOLT_API void zjoltBodyGetCenterOfMassTransform(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltRMat44 *out);

/// The body's inverse inertia tensor, rotated into world space, as a 3x3
/// matrix padded out to 4x4 the way Jolt stores one.
///
/// Only a DYNAMIC body has one; asking of a body that is not is
/// ZJOLT_RESULT_INVALID_ARGUMENT rather than forwarded. A stale id is
/// ZJOLT_RESULT_BODY_NOT_FOUND.
ZJOLT_API ZJoltResult zjoltBodyGetInverseInertia(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltMat44 *out);

//===----------------------------------------------------------------------===//
// Mass and inertia
//
// Runtime changes to mass/inertia set once from the shape at creation. Live
// on MotionProperties rather than BodyInterface, so each takes its own lock.
//===----------------------------------------------------------------------===//

/// As zjoltBodyGetInverseInertia, but in LOCAL (body) space rather than
/// rotated into world space — MotionProperties::GetLocalSpaceInverseInertia.
/// Same DYNAMIC-only requirement and the same refusal on a body that is not.
ZJOLT_API ZJoltResult zjoltBodyGetLocalSpaceInverseInertia(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltMat44 *out);

/// As zjoltBodyGetInverseInertia, but for the hypothetical orientation
/// `rotation` instead of the body's actual current one —
/// MotionProperties::GetInverseInertiaForRotation. Only the rotation part of
/// `rotation` is used; translation is ignored. Same DYNAMIC-only requirement.
ZJOLT_API ZJoltResult zjoltBodyGetInverseInertiaForRotation(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltMat44 *rotation, ZJoltMat44 *out);

/// Inverse mass (1/kg), without the DYNAMIC check zjoltBodyGetInverseInertia
/// makes — MotionProperties::GetInverseMassUnchecked. Meaningful on a
/// currently-KINEMATIC body too, if it was created with
/// allow_dynamic_or_kinematic and therefore has a real mass ready for the
/// moment it becomes dynamic. 0 on a STATIC body, which has no motion
/// properties to hold this, exactly as for a NULL system or a stale id.
ZJOLT_API float zjoltBodyGetInverseMassUnchecked(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// Which axes this body is allowed to move along — the same bit mask
/// ZJoltBodyDesc::allowed_dofs sets at creation, read back via a body lock
/// like the accessors above. ZJOLT_ALLOWED_DOFS_ALL, Jolt's own
/// MotionProperties default, on a STATIC body, a NULL system, or a stale id.
ZJOLT_API uint32_t zjoltBodyGetAllowedDOFs(const ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body);

/// Whether this body currently has motion properties allocated at all. A
/// STATIC body has none; a KINEMATIC or DYNAMIC one always does. A body
/// created STATIC with allow_dynamic_or_kinematic true is the one case
/// this distinguishes from an ordinary static body (both report
/// ZJOLT_MOTION_TYPE_STATIC). False for a NULL system or a stale id.
ZJOLT_API bool zjoltBodyHasMotionProperties(const ZJoltPhysicsSystem *system,
                                            ZJoltBodyId body);

/// Sets the inverse mass (1 / mass) directly, with no validation and no
/// derived recomputation. Read back via zjoltBodyGetInverseMassUnchecked.
///
/// Unlike zjoltBodySetMassProperties (which asserts mass > 0), this can
/// express inverse_mass == 0 on a still-translating body. No-op on a
/// STATIC body.
ZJOLT_API ZJoltResult zjoltBodySetInverseMass(ZJoltPhysicsSystem *system,
                                              ZJoltBodyId body,
                                              float inverse_mass);

/// Sets the already-diagonalised inverse inertia tensor directly: a
/// diagonal and the rotation carrying it into local space. Unlike
/// zjoltBodySetMassProperties — whose eigen-solver CAN FAIL and silently
/// substitutes a unit-sphere inverse inertia on failure — this sets exactly
/// what is given. `rotation` is renormalised rather than asserting on a
/// non-unit input. No-op on a STATIC body.
ZJOLT_API ZJoltResult zjoltBodySetInverseInertia(ZJoltPhysicsSystem *system,
                                                 ZJoltBodyId body,
                                                 const ZJoltVec3 *diagonal,
                                                 const ZJoltQuat *rotation);

/// Rescales an already-live body's mass and inertia together, keeping their
/// ratio. `mass` must be positive.
///
/// A no-op on a STATIC body, or one whose current inverse mass is zero
/// (every translation DOF locked, or a never-massed KINEMATIC body). Read
/// zjoltBodyGetInverseMassUnchecked back to confirm it changed anything.
ZJOLT_API ZJoltResult zjoltBodyScaleToMass(ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body, float mass);

/// Gives a body a fully custom mass and inertia tensor, and sets its
/// allowed degrees of freedom at once. Pass zjoltBodyGetAllowedDOFs back to
/// leave the existing ones alone. `mass_properties->inertia` is direct
/// (not inverse). No-op on a STATIC body. `allowed_dofs` of 0, or
/// `mass_properties->mass` <= 0 with any translation axis allowed, is
/// ZJOLT_RESULT_INVALID_ARGUMENT.
ZJOLT_API ZJoltResult zjoltBodySetMassProperties(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t allowed_dofs,
    const ZJoltMassProperties *mass_properties);

/// `v` with the translation axes zjoltBodyGetAllowedDOFs excludes zeroed out
/// — MotionProperties::LockTranslation. `v` unchanged on a STATIC body, a
/// NULL system, or a stale id, matching zjoltBodyGetAllowedDOFs's ALL
/// default. `v` and `out` are required; a NULL either is a no-op.
ZJOLT_API void zjoltBodyMaskTranslationDOFs(const ZJoltPhysicsSystem *system,
                                            ZJoltBodyId body,
                                            const ZJoltVec3 *v, ZJoltVec3 *out);
/// As zjoltBodyMaskTranslationDOFs, but for the rotation axes —
/// MotionProperties::LockAngular.
ZJOLT_API void zjoltBodyMaskAngularDOFs(const ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body, const ZJoltVec3 *v,
                                        ZJoltVec3 *out);

/// Rescales the body's current linear velocity down to
/// zjoltBodyGetMaxLinearVelocity if it exceeds it, in place —
/// MotionProperties::ClampLinearVelocity. No-op below the limit or on a
/// STATIC body.
ZJOLT_API ZJoltResult zjoltBodyClampLinearVelocity(ZJoltPhysicsSystem *system,
                                                   ZJoltBodyId body);
/// As zjoltBodyClampLinearVelocity, for angular velocity and
/// zjoltBodyGetMaxAngularVelocity — MotionProperties::ClampAngularVelocity.
ZJOLT_API ZJoltResult zjoltBodyClampAngularVelocity(ZJoltPhysicsSystem *system,
                                                    ZJoltBodyId body);

/// I_world^-1 * v, using the body's current rotation —
/// MotionProperties::MultiplyWorldSpaceInverseInertiaByVector. Same
/// DYNAMIC-only requirement as zjoltBodyGetInverseInertia.
ZJOLT_API ZJoltResult zjoltBodyMultiplyWorldSpaceInverseInertiaByVector(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, const ZJoltVec3 *v,
    ZJoltVec3 *out);

/// As zjoltBodyGetLocalSpaceInverseInertia, but answers for a KINEMATIC body
/// too instead of refusing —
/// MotionProperties::GetLocalSpaceInverseInertiaUnchecked. Only a STATIC body
/// (no motion properties) is ZJOLT_RESULT_INVALID_ARGUMENT.
ZJOLT_API ZJoltResult zjoltBodyGetLocalSpaceInverseInertiaUnchecked(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltMat44 *out);

/// As zjoltBodySetPositionAndRotation, but skips the broad-phase update and
/// activation check when the new pose is close enough to the old one — for
/// a caller that cannot already tell "did this move" (a networked snapshot
/// applied every tick), avoiding waking a resting body for nothing.
/// `rotation` may be NULL to keep the current orientation.
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
/// space), including the contribution from spin. Zero for a static body,
/// and zero when the body lock fails; ask zjoltBodyIsAdded to tell them
/// apart.
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
/// As calling zjoltBodyAddForce then zjoltBodyAddTorque, but under the SAME
/// body lock and the SAME activation check — the two separate calls can
/// straddle a concurrent step and leave it seeing only the force applied
/// this sub-step; this cannot.
ZJOLT_API void zjoltBodyAddForceAndTorque(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *force,
                                          const ZJoltVec3 *torque);

/// What zjoltBodyAddForce/AddForceAtPoint/AddTorque/AddForceAndTorque have
/// accumulated on this body since the last step consumed it. Zero for a
/// body that is not DYNAMIC or whose lock fails.
///
/// A step clears both automatically; zjoltBodyResetForce/ResetTorque cancel
/// one early instead.
ZJOLT_API void zjoltBodyGetAccumulatedForce(const ZJoltPhysicsSystem *system,
                                            ZJoltBodyId body, ZJoltVec3 *out);
ZJOLT_API void zjoltBodyGetAccumulatedTorque(const ZJoltPhysicsSystem *system,
                                             ZJoltBodyId body, ZJoltVec3 *out);

/// A no-op on a body that is not DYNAMIC or whose lock fails, same as the
/// getters above.
ZJOLT_API void zjoltBodyResetForce(ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body);
ZJOLT_API void zjoltBodyResetTorque(ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body);

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
/// `surface_position`/`surface_normal` (surface plane in world space,
/// normal pointing out of the fluid) and activates it on success.
///
/// Returns false, and does nothing, for a body that is not dynamic or
/// whose lock fails.
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

/// No-op on a static or kinematic body (no motion properties). `velocity`
/// must not be negative — a negative one is undefined, not refused.
///
/// Getters answer 500 / 15*pi (Jolt's construction-time defaults) when
/// `system` is NULL or the body lock fails.
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

/// Whether this body is allowed to settle and go to sleep. Disabling it
/// does not wake an already-sleeping body — pair with zjoltBodyActivate.
///
/// Both are a no-op on a STATIC body (no motion properties); this checks
/// the motion type first rather than forwarding Jolt's unconditional
/// dereference. Getter answers true (Jolt's default) for a NULL system too.
ZJOLT_API void zjoltBodySetAllowSleeping(ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body, bool allow);
ZJOLT_API bool zjoltBodyGetAllowSleeping(const ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body);

/// Runtime linear and angular damping: dv/dt = -c * v. ZJoltBodyDesc sets
/// the starting value; these change drag after the fact (a mud patch, an
/// underwater volume, a speed limiter).
///
/// `damping` must not be negative (ZJOLT_RESULT_INVALID_ARGUMENT). No-op
/// (setters) / answers 0.05 (getters) on a STATIC body or a stale id.
ZJOLT_API ZJoltResult zjoltBodySetLinearDamping(ZJoltPhysicsSystem *system,
                                                ZJoltBodyId body,
                                                float damping);
ZJOLT_API float zjoltBodyGetLinearDamping(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body);
ZJOLT_API ZJoltResult zjoltBodySetAngularDamping(ZJoltPhysicsSystem *system,
                                                 ZJoltBodyId body,
                                                 float damping);
ZJOLT_API float zjoltBodyGetAngularDamping(const ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body);

/// Whether this body reports contacts without responding to them — a trigger
/// volume. Unlocked counterpart of zjoltBodyIsSensorLocked; NULL system or a
/// failed body lock answers false.
ZJOLT_API void zjoltBodySetIsSensor(ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body, bool is_sensor);
ZJOLT_API bool zjoltBodyIsSensor(const ZJoltPhysicsSystem *system,
                                 ZJoltBodyId body);

//===----------------------------------------------------------------------===//
// Flags with no BodyInterface wrapper
//
// The three below live on Body itself (Body::mFlags), so they read
// correctly even for a body with no motion properties; a lock is still taken.
//===----------------------------------------------------------------------===//

/// Whether this body applies the gyroscopic force (the Dzhanibekov "tennis
/// racket" effect) as part of the step — Body::GetApplyGyroscopicForce.
/// false, Jolt's own construction-time default, on a NULL system or a stale
/// id.
ZJOLT_API bool zjoltBodyGetApplyGyroscopicForce(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// Whether a KINEMATIC body generates contacts against other kinematic or
/// static bodies — Body::GetCollideKinematicVsNonDynamic. Meaningless for a
/// dynamic body, which already collides with everything its object layer
/// allows regardless of this flag. false, Jolt's own construction-time
/// default, on a NULL system or a stale id.
ZJOLT_API bool zjoltBodyGetCollideKinematicVsNonDynamic(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// Whether this body gets the extra ghost-contact suppression a convex shape
/// sliding over a mesh's internal edges needs —
/// Body::GetEnhancedInternalEdgeRemoval. false, Jolt's own construction-time
/// default, on a NULL system or a stale id.
ZJOLT_API bool zjoltBodyGetEnhancedInternalEdgeRemoval(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// Whether this body changed in a way that invalidates cached contact
/// results for every pair touching it — Body::IsCollisionCacheInvalid, set
/// by zjoltBodyInvalidateContactCache and cleared once the next step
/// reprocesses those pairs. false for a NULL system or a stale id.
ZJOLT_API bool zjoltBodyIsCollisionCacheInvalid(const ZJoltPhysicsSystem *system,
                                                ZJoltBodyId body);

/// Whether a contact between `body1` and `body2` uses manifold reduction —
/// Body::GetUseManifoldReductionWithBody, true only when BOTH allow it. true,
/// matching zjoltBodyGetUseManifoldReduction's own default, if either body's
/// lock fails.
ZJOLT_API bool zjoltBodyGetUseManifoldReductionWithBody(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2);
/// Whether a contact between `body1` and `body2` gets enhanced internal-edge
/// removal — Body::GetEnhancedInternalEdgeRemovalWithBody, true if EITHER
/// body requests it. false, matching zjoltBodyGetEnhancedInternalEdgeRemoval's
/// own default, if either body's lock fails.
ZJOLT_API bool zjoltBodyGetEnhancedInternalEdgeRemovalWithBody(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2);

/// The material of one leaf of the body's shape. @see zjoltShapeGetMaterial
/// for what a material is and the sub-shape id rules.
///
/// NOT A WAY TO TEST WHETHER A BODY EXISTS: a destroyed body and a body
/// whose shape has no materials of its own both answer with the shared
/// default. Use zjoltBodyIsAdded instead. NULL only when `system` is NULL.
ZJOLT_API const ZJoltPhysicsMaterial *zjoltBodyGetMaterial(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltSubShapeId sub_shape_id);

/// Tells the system a body's shape changed underneath it — required after
/// any zjoltShapeMutableCompound* call.
///
/// `previous_center_of_mass` is the shape's centre of mass BEFORE the
/// change; the body is moved so its geometry stays in place. Without this
/// call the broad phase and contact cache keep describing the old shape.
ZJOLT_API void zjoltBodyNotifyShapeChanged(
    ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltVec3 *previous_center_of_mass, bool update_mass_properties,
    ZJoltActivation activation);

//===----------------------------------------------------------------------===//
// Bulk read-back
//
// Wrong to pay one ABI crossing and one body lock per body per frame. These
// two do every body in two crossings and one lock acquisition instead.
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
/// `out_positions` and `out_rotations` are parallel arrays of `count`
/// entries; either may be NULL if that half is not wanted. An id that no
/// longer names a live body writes the identity transform and is reported
/// through `out_missing` (may be NULL) rather than failing the whole batch.
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
// A shape query, not a body one
//
// Jolt never checks Shape::MustBeStatic when attaching a shape to a body,
// so nothing refuses a mesh or height field handed to a dynamic body.
//===----------------------------------------------------------------------===//

/// Whether a body wearing `shape` is only ever allowed to be STATIC —
/// Shape::MustBeStatic. true for a height field, a plane, a mesh, and any
/// compound or decorated shape with one of those somewhere inside it; false
/// for every other kind, and for NULL.
ZJOLT_API bool zjoltShapeMustBeStatic(const ZJoltShape *shape);

//===----------------------------------------------------------------------===//
// Body locks
//
// Holds the body for a scope, handing out a borrowed ZJoltBody (must not
// outlive it); release is the caller's job. `body` is NULL for a stale id.
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

/// Position, rotation and velocity below assume ordinary caller code,
/// outside PhysicsSystem::Update. Jolt narrows this window itself during
/// its own step — a contact listener, a custom constraint's
/// SetupVelocityConstraint — and asserts a violation in a debug build;
/// mind the same split (BodyAccess::Grant) from inside one of those.
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

/// Timing for the simulation island this body was part of during its last
/// step. Requires a lock, same as the accessors above.
///
/// ZJOLT_RESULT_UNSUPPORTED (leaving `*out` zeroed) unless built with
/// JPH_TRACK_SIMULATION_STATS. A STATIC body reads back all zeroes with
/// ZJOLT_RESULT_OK instead — nothing tracked, not an unsupported build.
typedef struct ZJoltSimulationStats {
  uint64_t broad_phase_ticks;
  uint64_t narrow_phase_ticks;
  uint64_t velocity_constraint_ticks;
  uint64_t position_constraint_ticks;
  uint64_t update_bounds_ticks;
  uint64_t ccd_ticks;
  uint32_t num_contact_constraints;
  uint8_t num_collision_steps;
  uint8_t num_velocity_steps;
  uint8_t num_position_steps;
  bool is_large_island;
} ZJoltSimulationStats;

ZJOLT_API ZJoltResult zjoltBodyGetSimulationStatsLocked(
    const ZJoltBody *body, ZJoltSimulationStats *out);

/// Debug check: the body's cached broad-phase bounds still match its
/// shape's actual bounds — Body::ValidateCachedBounds, asserting on a
/// mismatch (a mutable compound edited without zjoltBodyNotifyShapeChanged).
///
/// ZJOLT_RESULT_UNSUPPORTED (no-op) when Jolt is built without asserts,
/// where the check does not exist to run.
ZJOLT_API ZJoltResult zjoltBodyValidateCachedBoundsLocked(const ZJoltBody *body);

/// Debug check: a sleeping body has zero velocity — Body::ValidateMotion.
/// Catches a velocity set directly on MotionProperties, bypassing
/// zjoltBodySetLinearVelocity/AngularVelocity, which wake the body first.
///
/// ZJOLT_RESULT_UNSUPPORTED (no-op) without asserts, same as
/// zjoltBodyValidateCachedBoundsLocked.
ZJOLT_API ZJoltResult zjoltBodyValidateMotionLocked(const ZJoltBody *body);

//===----------------------------------------------------------------------===//
// Multi-body locks
//
// Reads several bodies CONSISTENTLY under one mutex mask, unlike
// zjoltBodyLockRead/Write one at a time. What GetTransforms/GetMotions use.
//===----------------------------------------------------------------------===//

typedef struct ZJoltBodyLockMulti {
  /// Implementation detail. Do not read, do not copy this struct while held.
  const ZJoltBodyId *_reserved_ids;
  uint32_t _reserved_count;
  uint64_t _reserved_mask;
  void *_reserved_interface;
} ZJoltBodyLockMulti;

/// Takes a shared lock over every body in `ids` at once. `ids` must stay
/// valid until the matching release — Jolt's own BodyLockMultiRead holds the
/// same borrowed pointer, not a copy, and zjoltBodyLockMultiGet reads it
/// again on every call.
ZJOLT_API void zjoltBodyLockMultiRead(const ZJoltPhysicsSystem *system,
                                      const ZJoltBodyId *ids, uint32_t count,
                                      ZJoltBodyLockMulti *out_lock);
ZJOLT_API void zjoltBodyLockMultiReadRelease(ZJoltBodyLockMulti *lock);

/// Takes an exclusive lock over every body in `ids` at once. Same borrowing
/// rule as zjoltBodyLockMultiRead.
ZJOLT_API void zjoltBodyLockMultiWrite(ZJoltPhysicsSystem *system,
                                       const ZJoltBodyId *ids, uint32_t count,
                                       ZJoltBodyLockMulti *out_lock);
ZJOLT_API void zjoltBodyLockMultiWriteRelease(ZJoltBodyLockMulti *lock);

/// The body at `index` in the set the lock was taken over, or NULL if that id
/// no longer names a live body — BodyLockMultiBase::GetBody. `index` must be
/// < the `count` passed to zjoltBodyLockMultiRead/Write; Jolt's own bound is
/// only an assert, so this checks it rather than reading past what was
/// locked when asserts are compiled out.
ZJOLT_API ZJoltBody *zjoltBodyLockMultiGet(const ZJoltBodyLockMulti *lock,
                                           uint32_t index);

//===----------------------------------------------------------------------===//
// Detaching a body from its id
//
// Unlike zjoltBodyDestroy, frees a live body's id for reuse while it
// survives, in its own owning handle rather than ZJoltBody.
//===----------------------------------------------------------------------===//

/// A body that was added and then had zjoltBodyUnassignId(s) called on it: it
/// keeps its shape, transform, velocity and every other property, but no
/// longer has an id and cannot be queried, stepped, or found by any
/// zjoltBody* call until it is given one back. Owned outright — release it
/// with zjoltUnassignedBodyAssignId or zjoltUnassignedBodyDestroy, exactly
/// one of the two.
typedef struct ZJoltUnassignedBody ZJoltUnassignedBody;

/// Removes `body`'s id and hands back the still-alive object, added or
/// not — zjoltBodyRemove (if added) followed by BodyInterface::UnassignBodyID.
///
/// `body` must currently name a live body in `system`; a stale id is
/// ZJOLT_RESULT_BODY_NOT_FOUND. Liveness is checked under a body lock
/// first — the same hazard the batch calls in zjolt_batch.h document.
ZJOLT_API ZJoltResult zjoltBodyUnassignId(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          ZJoltUnassignedBody **out);

/// Gives `unassigned` the id `id`, consuming the handle. `id` is checked
/// like zjoltBodyCreateWithId: not ZJOLT_BODY_ID_INVALID, bit 31 clear, and
/// not already naming a live body.
///
/// `unassigned` must have come from `system`; from another system's is
/// ZJOLT_RESULT_INVALID_ARGUMENT, and the handle is left untouched.
ZJOLT_API ZJoltResult zjoltUnassignedBodyAssignId(ZJoltPhysicsSystem *system,
                                                  ZJoltUnassignedBody *unassigned,
                                                  ZJoltBodyId id,
                                                  ZJoltBodyId *out);

/// Destroys the object outright instead of giving it back an id. `system`
/// must be present; `unassigned` may be NULL, in which case this does
/// nothing.
ZJOLT_API ZJoltResult zjoltUnassignedBodyDestroy(
    ZJoltPhysicsSystem *system, ZJoltUnassignedBody *unassigned);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_BODY_H_
