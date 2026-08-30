//! ZJolt C declarations for bodies, the body interface, and body locks.
//!
//! Mirrors `ffi/zjolt_body.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");
const group = @import("group.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const AABox = core.AABox;
pub const Activation = core.Activation;
pub const AllowedDofs = core.AllowedDofs;
pub const Body = core.Body;
pub const BodyId = core.BodyId;
pub const Mat44 = core.Mat44;
pub const MassProperties = core.MassProperties;
pub const MotionQuality = core.MotionQuality;
pub const MotionType = core.MotionType;
pub const ObjectLayer = core.ObjectLayer;
pub const OverrideMassProperties = core.OverrideMassProperties;
pub const PhysicsMaterial = core.PhysicsMaterial;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Quat = core.Quat;
pub const RMat44 = core.RMat44;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Shape = core.Shape;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
pub const body_id_invalid = core.body_id_invalid;
pub const CollisionGroup = group.CollisionGroup;
pub const zjoltBodyGetCollisionGroup = group.zjoltBodyGetCollisionGroup;
pub const zjoltBodyInvalidateContactCache = group.zjoltBodyInvalidateContactCache;
pub const zjoltBodySetCollisionGroup = group.zjoltBodySetCollisionGroup;

pub const BodyType = enum(c_int) {
    rigid_body = 0,
    soft_body = 1,
};

pub const BodyDesc = extern struct {
    position: RVec3,
    rotation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    shape: ?*const Shape,
    collision_group: CollisionGroup,
    user_data: u64,
    object_layer: ObjectLayer,
    motion_type: MotionType,
    motion_quality: MotionQuality,
    allowed_dofs: AllowedDofs,
    override_mass_properties: OverrideMassProperties,
    mass: f32,
    mass_properties_override: MassProperties,
    allow_dynamic_or_kinematic: bool,
    is_sensor: bool,
    allow_sleeping: bool,
    enhanced_internal_edge_removal: bool,
    collide_kinematic_vs_non_dynamic: bool,
    use_manifold_reduction: bool,
    apply_gyroscopic_force: bool,
    friction: f32,
    restitution: f32,
    linear_damping: f32,
    angular_damping: f32,
    max_linear_velocity: f32,
    max_angular_velocity: f32,
    gravity_factor: f32,
    num_velocity_steps_override: u32,
    num_position_steps_override: u32,
    inertia_multiplier: f32,
};

pub const BodyLock = extern struct {
    body: ?*Body,
    _reserved: [2]?*anyopaque,
};

pub const BodyLockMulti = extern struct {
    _reserved_ids: ?[*]const BodyId,
    _reserved_count: u32,
    _reserved_mask: u64,
    _reserved_interface: ?*anyopaque,
};

pub const SimulationStats = extern struct {
    broad_phase_ticks: u64,
    narrow_phase_ticks: u64,
    velocity_constraint_ticks: u64,
    position_constraint_ticks: u64,
    update_bounds_ticks: u64,
    ccd_ticks: u64,
    num_contact_constraints: u32,
    num_collision_steps: u8,
    num_velocity_steps: u8,
    num_position_steps: u8,
    is_large_island: bool,
};

pub extern fn zjoltPhysicsSystemGetActiveBodies(system: *const PhysicsSystem, body_type: BodyType, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltPhysicsSystemGetBodies(system: *const PhysicsSystem, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltBodyDescInit(desc: *BodyDesc) void;

pub extern fn zjoltBodyCreate(system: *PhysicsSystem, desc: *const BodyDesc, out: *BodyId) Result;

pub extern fn zjoltBodyCreateAndAdd(system: *PhysicsSystem, desc: *const BodyDesc, activation: Activation, out: *BodyId) Result;

pub extern fn zjoltBodyCreateWithId(system: *PhysicsSystem, desc: *const BodyDesc, id: BodyId, out: *BodyId) Result;

pub extern fn zjoltBodyCreateAndAddWithId(system: *PhysicsSystem, desc: *const BodyDesc, id: BodyId, activation: Activation, out: *BodyId) Result;

pub extern fn zjoltBodyApplyBodyCreationSettings(system: *PhysicsSystem, body: BodyId, desc: *const BodyDesc) Result;

pub extern fn zjoltBodyDestroy(system: *PhysicsSystem, body: BodyId) void;

pub extern fn zjoltBodyAdd(system: *PhysicsSystem, body: BodyId, activation: Activation) void;

pub extern fn zjoltBodyRemove(system: *PhysicsSystem, body: BodyId) void;

pub extern fn zjoltBodyIsAdded(system: *const PhysicsSystem, body: BodyId) bool;

pub extern fn zjoltBodyIsActive(system: *const PhysicsSystem, body: BodyId) bool;

pub extern fn zjoltBodyActivate(system: *PhysicsSystem, body: BodyId) void;

pub extern fn zjoltBodyDeactivate(system: *PhysicsSystem, body: BodyId) void;

pub extern fn zjoltBodyResetSleepTimer(system: *PhysicsSystem, body: BodyId) void;

pub extern fn zjoltBodyGetBodyType(system: *const PhysicsSystem, body: BodyId) BodyType;

pub extern fn zjoltBodyIdGetSequenceNumber(id: BodyId) u8;

pub extern fn zjoltBodySetMotionType(system: *PhysicsSystem, body: BodyId, motion_type: MotionType, activation: Activation) void;

pub extern fn zjoltBodyGetMotionType(system: *const PhysicsSystem, body: BodyId) MotionType;

pub extern fn zjoltBodySetMotionQuality(system: *PhysicsSystem, body: BodyId, quality: MotionQuality) void;

pub extern fn zjoltBodyGetMotionQuality(system: *const PhysicsSystem, body: BodyId) MotionQuality;

pub extern fn zjoltBodySetPositionAndRotation(system: *PhysicsSystem, body: BodyId, position: *const RVec3, rotation: ?*const Quat, activation: Activation) void;

pub extern fn zjoltBodyGetPositionAndRotation(system: *const PhysicsSystem, body: BodyId, out_position: ?*RVec3, out_rotation: ?*Quat) void;

pub extern fn zjoltBodyGetCenterOfMassPosition(system: *const PhysicsSystem, body: BodyId, out: *RVec3) void;

pub extern fn zjoltBodyGetWorldTransform(system: *const PhysicsSystem, body: BodyId, out: *RMat44) void;

pub extern fn zjoltBodyGetCenterOfMassTransform(system: *const PhysicsSystem, body: BodyId, out: *RMat44) void;

pub extern fn zjoltBodyGetInverseInertia(system: *const PhysicsSystem, body: BodyId, out: *Mat44) Result;

pub extern fn zjoltBodyGetLocalSpaceInverseInertia(system: *const PhysicsSystem, body: BodyId, out: *Mat44) Result;

pub extern fn zjoltBodyGetInverseInertiaForRotation(system: *const PhysicsSystem, body: BodyId, rotation: *const Mat44, out: *Mat44) Result;

pub extern fn zjoltBodyGetInverseMassUnchecked(system: *const PhysicsSystem, body: BodyId) f32;

pub extern fn zjoltBodyGetAllowedDOFs(system: *const PhysicsSystem, body: BodyId) AllowedDofs;

pub extern fn zjoltBodyHasMotionProperties(system: *const PhysicsSystem, body: BodyId) bool;

pub extern fn zjoltBodySetInverseMass(system: *PhysicsSystem, body: BodyId, inverse_mass: f32) Result;

pub extern fn zjoltBodySetInverseInertia(system: *PhysicsSystem, body: BodyId, diagonal: *const Vec3, rotation: *const Quat) Result;

pub extern fn zjoltBodyScaleToMass(system: *PhysicsSystem, body: BodyId, mass: f32) Result;

pub extern fn zjoltBodySetMassProperties(system: *PhysicsSystem, body: BodyId, allowed_dofs: AllowedDofs, mass_properties: *const MassProperties) Result;

pub extern fn zjoltBodyMaskTranslationDOFs(system: *const PhysicsSystem, body: BodyId, v: *const Vec3, out: *Vec3) void;

pub extern fn zjoltBodyMaskAngularDOFs(system: *const PhysicsSystem, body: BodyId, v: *const Vec3, out: *Vec3) void;

pub extern fn zjoltBodyClampLinearVelocity(system: *PhysicsSystem, body: BodyId) Result;

pub extern fn zjoltBodyClampAngularVelocity(system: *PhysicsSystem, body: BodyId) Result;

pub extern fn zjoltBodyMultiplyWorldSpaceInverseInertiaByVector(system: *const PhysicsSystem, body: BodyId, v: *const Vec3, out: *Vec3) Result;

pub extern fn zjoltBodyGetLocalSpaceInverseInertiaUnchecked(system: *const PhysicsSystem, body: BodyId, out: *Mat44) Result;

pub extern fn zjoltBodySetPositionAndRotationWhenChanged(system: *PhysicsSystem, body: BodyId, position: *const RVec3, rotation: ?*const Quat, activation: Activation) void;

pub extern fn zjoltBodyMoveKinematic(system: *PhysicsSystem, body: BodyId, target_position: *const RVec3, target_rotation: ?*const Quat, delta_time: f32) void;

pub extern fn zjoltBodySetPositionRotationAndVelocity(system: *PhysicsSystem, body: BodyId, position: *const RVec3, rotation: ?*const Quat, linear_velocity: *const Vec3, angular_velocity: *const Vec3) void;

pub extern fn zjoltBodySetLinearVelocity(system: *PhysicsSystem, body: BodyId, velocity: *const Vec3) void;

pub extern fn zjoltBodyGetLinearVelocity(system: *const PhysicsSystem, body: BodyId, out: *Vec3) void;

pub extern fn zjoltBodySetAngularVelocity(system: *PhysicsSystem, body: BodyId, velocity: *const Vec3) void;

pub extern fn zjoltBodyGetAngularVelocity(system: *const PhysicsSystem, body: BodyId, out: *Vec3) void;

pub extern fn zjoltBodySetLinearAndAngularVelocity(system: *PhysicsSystem, body: BodyId, linear_velocity: *const Vec3, angular_velocity: *const Vec3) void;

pub extern fn zjoltBodyGetLinearAndAngularVelocity(system: *const PhysicsSystem, body: BodyId, out_linear_velocity: ?*Vec3, out_angular_velocity: ?*Vec3) void;

pub extern fn zjoltBodyAddLinearVelocity(system: *PhysicsSystem, body: BodyId, linear_velocity: *const Vec3) void;

pub extern fn zjoltBodyAddLinearAndAngularVelocity(system: *PhysicsSystem, body: BodyId, linear_velocity: *const Vec3, angular_velocity: *const Vec3) void;

pub extern fn zjoltBodyGetPointVelocity(system: *const PhysicsSystem, body: BodyId, point: *const RVec3, out: *Vec3) void;

pub extern fn zjoltBodyAddForce(system: *PhysicsSystem, body: BodyId, force: *const Vec3) void;

pub extern fn zjoltBodyAddForceAtPoint(system: *PhysicsSystem, body: BodyId, force: *const Vec3, point: *const RVec3) void;

pub extern fn zjoltBodyAddTorque(system: *PhysicsSystem, body: BodyId, torque: *const Vec3) void;

pub extern fn zjoltBodyAddForceAndTorque(system: *PhysicsSystem, body: BodyId, force: *const Vec3, torque: *const Vec3) void;

pub extern fn zjoltBodyGetAccumulatedForce(system: *const PhysicsSystem, body: BodyId, out: *Vec3) void;

pub extern fn zjoltBodyGetAccumulatedTorque(system: *const PhysicsSystem, body: BodyId, out: *Vec3) void;

pub extern fn zjoltBodyResetForce(system: *PhysicsSystem, body: BodyId) void;

pub extern fn zjoltBodyResetTorque(system: *PhysicsSystem, body: BodyId) void;

pub extern fn zjoltBodyAddImpulse(system: *PhysicsSystem, body: BodyId, impulse: *const Vec3) void;

pub extern fn zjoltBodyAddImpulseAtPoint(system: *PhysicsSystem, body: BodyId, impulse: *const Vec3, point: *const RVec3) void;

pub extern fn zjoltBodyAddAngularImpulse(system: *PhysicsSystem, body: BodyId, angular_impulse: *const Vec3) void;

pub extern fn zjoltBodyApplyBuoyancyImpulse(system: *PhysicsSystem, body: BodyId, surface_position: *const RVec3, surface_normal: *const Vec3, buoyancy: f32, linear_drag: f32, angular_drag: f32, fluid_velocity: *const Vec3, gravity: *const Vec3, delta_time: f32) bool;

pub extern fn zjoltBodySetShape(system: *PhysicsSystem, body: BodyId, shape: *const Shape, update_mass_properties: bool, activation: Activation) void;

pub extern fn zjoltBodySetObjectLayer(system: *PhysicsSystem, body: BodyId, layer: ObjectLayer) void;

pub extern fn zjoltBodyGetObjectLayer(system: *const PhysicsSystem, body: BodyId) ObjectLayer;

pub extern fn zjoltBodySetUserData(system: *PhysicsSystem, body: BodyId, user_data: u64) void;

pub extern fn zjoltBodyGetUserData(system: *const PhysicsSystem, body: BodyId) u64;

pub extern fn zjoltBodySetFriction(system: *PhysicsSystem, body: BodyId, friction: f32) void;

pub extern fn zjoltBodyGetFriction(system: *const PhysicsSystem, body: BodyId) f32;

pub extern fn zjoltBodySetRestitution(system: *PhysicsSystem, body: BodyId, restitution: f32) void;

pub extern fn zjoltBodyGetRestitution(system: *const PhysicsSystem, body: BodyId) f32;

pub extern fn zjoltBodySetGravityFactor(system: *PhysicsSystem, body: BodyId, factor: f32) void;

pub extern fn zjoltBodyGetGravityFactor(system: *const PhysicsSystem, body: BodyId) f32;

pub extern fn zjoltBodySetMaxLinearVelocity(system: *PhysicsSystem, body: BodyId, velocity: f32) void;

pub extern fn zjoltBodyGetMaxLinearVelocity(system: *const PhysicsSystem, body: BodyId) f32;

pub extern fn zjoltBodySetMaxAngularVelocity(system: *PhysicsSystem, body: BodyId, velocity: f32) void;

pub extern fn zjoltBodyGetMaxAngularVelocity(system: *const PhysicsSystem, body: BodyId) f32;

pub extern fn zjoltBodySetUseManifoldReduction(system: *PhysicsSystem, body: BodyId, use_reduction: bool) void;

pub extern fn zjoltBodyGetUseManifoldReduction(system: *const PhysicsSystem, body: BodyId) bool;

pub extern fn zjoltBodySetAllowSleeping(system: *PhysicsSystem, body: BodyId, allow: bool) void;

pub extern fn zjoltBodyGetAllowSleeping(system: *const PhysicsSystem, body: BodyId) bool;

pub extern fn zjoltBodySetLinearDamping(system: *PhysicsSystem, body: BodyId, damping: f32) Result;

pub extern fn zjoltBodyGetLinearDamping(system: *const PhysicsSystem, body: BodyId) f32;

pub extern fn zjoltBodySetAngularDamping(system: *PhysicsSystem, body: BodyId, damping: f32) Result;

pub extern fn zjoltBodyGetAngularDamping(system: *const PhysicsSystem, body: BodyId) f32;

pub extern fn zjoltBodySetIsSensor(system: *PhysicsSystem, body: BodyId, is_sensor: bool) void;

pub extern fn zjoltBodyIsSensor(system: *const PhysicsSystem, body: BodyId) bool;

pub extern fn zjoltBodySetNumVelocityStepsOverride(system: *PhysicsSystem, body: BodyId, steps: u32) Result;
pub extern fn zjoltBodyGetNumVelocityStepsOverride(system: *const PhysicsSystem, body: BodyId) u32;
pub extern fn zjoltBodySetNumPositionStepsOverride(system: *PhysicsSystem, body: BodyId, steps: u32) Result;
pub extern fn zjoltBodyGetNumPositionStepsOverride(system: *const PhysicsSystem, body: BodyId) u32;

pub extern fn zjoltBodyGetApplyGyroscopicForce(system: *const PhysicsSystem, body: BodyId) bool;
pub extern fn zjoltBodySetApplyGyroscopicForce(system: *PhysicsSystem, body: BodyId, apply: bool) void;

pub extern fn zjoltBodyGetCollideKinematicVsNonDynamic(system: *const PhysicsSystem, body: BodyId) bool;
pub extern fn zjoltBodySetCollideKinematicVsNonDynamic(system: *PhysicsSystem, body: BodyId, collide: bool) void;

pub extern fn zjoltBodyGetEnhancedInternalEdgeRemoval(system: *const PhysicsSystem, body: BodyId) bool;
pub extern fn zjoltBodySetEnhancedInternalEdgeRemoval(system: *PhysicsSystem, body: BodyId, remove: bool) void;

pub extern fn zjoltBodyIsCollisionCacheInvalid(system: *const PhysicsSystem, body: BodyId) bool;

pub extern fn zjoltBodyGetUseManifoldReductionWithBody(system: *const PhysicsSystem, body1: BodyId, body2: BodyId) bool;

pub extern fn zjoltBodyGetEnhancedInternalEdgeRemovalWithBody(system: *const PhysicsSystem, body1: BodyId, body2: BodyId) bool;

pub extern fn zjoltBodyGetMaterial(system: *const PhysicsSystem, body: BodyId, sub_shape_id: SubShapeId) ?*const PhysicsMaterial;

pub extern fn zjoltBodyNotifyShapeChanged(system: *PhysicsSystem, body: BodyId, previous_center_of_mass: *const Vec3, update_mass_properties: bool, activation: Activation) void;

pub extern fn zjoltBodyGetTransforms(system: *const PhysicsSystem, ids: [*]const BodyId, count: u32, out_positions: ?[*]RVec3, out_rotations: ?[*]Quat, out_missing: ?*u32) Result;

pub extern fn zjoltBodyGetMotions(system: *const PhysicsSystem, ids: [*]const BodyId, count: u32, out_center_of_mass: ?[*]RVec3, out_linear_velocities: ?[*]Vec3, out_missing: ?*u32) Result;

pub extern fn zjoltShapeMustBeStatic(shape: ?*const Shape) bool;

pub extern fn zjoltBodyLockRead(system: *const PhysicsSystem, body: BodyId, out_lock: *BodyLock) void;

pub extern fn zjoltBodyLockReadRelease(lock: *BodyLock) void;

pub extern fn zjoltBodyLockWrite(system: *PhysicsSystem, body: BodyId, out_lock: *BodyLock) void;

pub extern fn zjoltBodyLockWriteRelease(lock: *BodyLock) void;

pub extern fn zjoltBodyGetId(body: *const Body) BodyId;

pub extern fn zjoltBodyGetPosition(body: *const Body, out: *RVec3) void;

pub extern fn zjoltBodyGetRotation(body: *const Body, out: *Quat) void;

pub extern fn zjoltBodyGetCenterOfMassPositionLocked(body: *const Body, out: *RVec3) void;

pub extern fn zjoltBodyGetLinearVelocityLocked(body: *const Body, out: *Vec3) void;

pub extern fn zjoltBodyGetAngularVelocityLocked(body: *const Body, out: *Vec3) void;

pub extern fn zjoltBodyGetUserDataLocked(body: *const Body) u64;

pub extern fn zjoltBodyGetObjectLayerLocked(body: *const Body) ObjectLayer;

pub extern fn zjoltBodyGetMotionTypeLocked(body: *const Body) MotionType;

pub extern fn zjoltBodyIsActiveLocked(body: *const Body) bool;

pub extern fn zjoltBodyIsSensorLocked(body: *const Body) bool;

pub extern fn zjoltBodyGetShapeLocked(body: *const Body) ?*const Shape;

pub extern fn zjoltBodyGetWorldBounds(body: *const Body, out: *AABox) void;

pub extern fn zjoltBodySetLinearVelocityLocked(body: *Body, velocity: *const Vec3) void;

pub extern fn zjoltBodySetAngularVelocityLocked(body: *Body, velocity: *const Vec3) void;

pub extern fn zjoltBodySetUserDataLocked(body: *Body, user_data: u64) void;

pub extern fn zjoltBodySetFrictionLocked(body: *Body, friction: f32) void;

pub extern fn zjoltBodySetRestitutionLocked(body: *Body, restitution: f32) void;

pub extern fn zjoltBodyAddImpulseLocked(body: *Body, impulse: *const Vec3) void;

pub extern fn zjoltBodyGetSimulationStatsLocked(body: *const Body, out: *SimulationStats) Result;

pub extern fn zjoltBodyValidateCachedBoundsLocked(body: *const Body) Result;

pub extern fn zjoltBodyValidateMotionLocked(body: *const Body) Result;

pub extern fn zjoltBodyLockMultiRead(system: *const PhysicsSystem, ids: [*]const BodyId, count: u32, out_lock: *BodyLockMulti) void;

pub extern fn zjoltBodyLockMultiReadRelease(lock: *BodyLockMulti) void;

pub extern fn zjoltBodyLockMultiWrite(system: *PhysicsSystem, ids: [*]const BodyId, count: u32, out_lock: *BodyLockMulti) void;

pub extern fn zjoltBodyLockMultiWriteRelease(lock: *BodyLockMulti) void;

pub extern fn zjoltBodyLockMultiGet(lock: *const BodyLockMulti, index: u32) ?*Body;

pub const UnassignedBody = opaque {};

pub extern fn zjoltBodyUnassignId(system: *PhysicsSystem, body: BodyId, out: **UnassignedBody) Result;

pub extern fn zjoltUnassignedBodyAssignId(system: *PhysicsSystem, unassigned: *UnassignedBody, id: BodyId, out: *BodyId) Result;

pub extern fn zjoltUnassignedBodyDestroy(system: *PhysicsSystem, unassigned: ?*UnassignedBody) Result;
