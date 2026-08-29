//! ZJolt C declarations for every constraint kind, its motors, limits and
//! springs.
//!
//! Mirrors `ffi/zjolt_constraint.h` exactly: a declaration belongs to the
//! module named after the header that declares it, so there is nothing to
//! decide and nothing to drift. `src/c.zig` lists every one of these and is
//! what the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");
const debug = @import("debug.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const BodyId = core.BodyId;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Mat44 = core.Mat44;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Vec3 = core.Vec3;
pub const max_physics_jobs = core.max_physics_jobs;

pub const Constraint = opaque {};

pub const PathConstraintPath = opaque {};

/// What a custom constraint's save_state / restore_state callbacks talk to.
/// @see ZJoltCustomConstraintCallbacks.
pub const StateRecorder = opaque {};

/// The body id that stands for "the world" when creating a constraint. Spelled
/// as the invalid id because that is what Jolt's own fixed-to-world body
/// carries, and because no real body can collide with the value.
pub const body_id_world: BodyId = 0xffff_ffff;

pub const six_dof_axis_count: u32 = 6;

pub const six_dof_translation_axis_count: u32 = 3;

pub const ConstraintSubType = enum(c_int) {
    other = 0,
    fixed = 1,
    point = 2,
    hinge = 3,
    slider = 4,
    distance = 5,
    cone = 6,
    swing_twist = 7,
    six_dof = 8,
    path = 9,
    gear = 10,
    rack_and_pinion = 11,
    pulley = 12,
    custom = 13,
};

pub const ConstraintSpace = enum(c_int) {
    local_to_body_com = 0,
    world = 1,
};

pub const MotorState = enum(c_int) {
    off = 0,
    velocity = 1,
    position = 2,
    position_and_velocity = 3,
};

pub const SpringMode = enum(c_int) {
    frequency_and_damping = 0,
    stiffness_and_damping = 1,
    mass_normalized_stiffness_and_damping = 2,
};

pub const SwingType = enum(c_int) {
    cone = 0,
    pyramid = 1,
};

pub const SixDofAxis = enum(c_int) {
    translation_x = 0,
    translation_y = 1,
    translation_z = 2,
    rotation_x = 3,
    rotation_y = 4,
    rotation_z = 5,
};

pub const PathRotationConstraintType = enum(c_int) {
    free = 0,
    constrain_around_tangent = 1,
    constrain_around_normal = 2,
    constrain_around_binormal = 3,
    constrain_to_path = 4,
    fully_constrained = 5,
};

pub const SpringSettings = extern struct {
    mode: SpringMode = .frequency_and_damping,
    frequency_or_stiffness: f32 = 0,
    damping: f32 = 0,
};

pub const MotorSettings = extern struct {
    spring: SpringSettings = .{},
    min_force_limit: f32 = -std.math.floatMax(f32),
    max_force_limit: f32 = std.math.floatMax(f32),
    min_torque_limit: f32 = -std.math.floatMax(f32),
    max_torque_limit: f32 = std.math.floatMax(f32),
};

pub const FixedConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    auto_detect_point: bool = false,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    axis_x1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    axis_y1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    axis_x2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    axis_y2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
};

pub const PointConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
};

pub const HingeConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    hinge_axis1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    normal_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    hinge_axis2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    normal_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    limits_min: f32 = -std.math.pi,
    limits_max: f32 = std.math.pi,
    limits_spring: SpringSettings = .{},
    max_friction_torque: f32 = 0,
    motor: MotorSettings = .{},
};

pub const SliderConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    auto_detect_point: bool = false,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    slider_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    normal_axis1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    slider_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    normal_axis2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    limits_min: f32 = -std.math.floatMax(f32),
    limits_max: f32 = std.math.floatMax(f32),
    limits_spring: SpringSettings = .{},
    max_friction_force: f32 = 0,
    motor: MotorSettings = .{},
};

pub const DistanceConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    min_distance: f32 = -1,
    max_distance: f32 = -1,
    limits_spring: SpringSettings = .{},
};

pub const ConeConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    twist_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    twist_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    half_cone_angle: f32 = 0,
};

pub const SwingTwistConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    position1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    twist_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    plane_axis1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    position2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    twist_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    plane_axis2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    swing_type: SwingType = .cone,
    normal_half_cone_angle: f32 = 0,
    plane_half_cone_angle: f32 = 0,
    twist_min_angle: f32 = 0,
    twist_max_angle: f32 = 0,
    max_friction_torque: f32 = 0,
    swing_motor: MotorSettings = .{},
    twist_motor: MotorSettings = .{},
};

pub const SixDofConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    position1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    axis_x1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    axis_y1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    position2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    axis_x2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    axis_y2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    swing_type: SwingType = .cone,
    max_friction: [six_dof_axis_count]f32 = @splat(0),
    limit_min: [six_dof_axis_count]f32 = @splat(-std.math.floatMax(f32)),
    limit_max: [six_dof_axis_count]f32 = @splat(std.math.floatMax(f32)),
    limits_spring: [six_dof_translation_axis_count]SpringSettings = @splat(.{}),
    motor: [six_dof_axis_count]MotorSettings = @splat(.{}),
};

pub const GearConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    hinge_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    hinge_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    ratio: f32 = 1,
};

pub const RackAndPinionConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    hinge_axis: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    slider_axis: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    ratio: f32 = 1,
};

pub const PulleyConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    body_point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    fixed_point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    body_point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    fixed_point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    ratio: f32 = 1,
    min_length: f32 = 0,
    max_length: f32 = -1,
};

pub const PathPoint = extern struct {
    position: Vec3,
    tangent: Vec3,
    normal: Vec3,
};

pub const PathConstraintDesc = extern struct {
    path: ?*const PathConstraintPath = null,
    path_position: Vec3 = .{ .x = 0, .y = 0, .z = 0 },
    path_rotation: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 },
    path_fraction: f32 = 0,
    max_friction_force: f32 = 0,
    motor: MotorSettings = .{},
    rotation_constraint_type: PathRotationConstraintType = .free,
};

pub extern fn zjoltPathConstraintPathCreateHermite(points: [*]const PathPoint, count: u32, is_looping: bool, out: **PathConstraintPath) Result;

pub extern fn zjoltPathConstraintPathAddRef(path: *const PathConstraintPath) void;

pub extern fn zjoltPathConstraintPathRelease(path: *const PathConstraintPath) void;

pub extern fn zjoltPathConstraintPathGetRefCount(path: *const PathConstraintPath) u32;

pub extern fn zjoltPathConstraintPathIsLooping(path: *const PathConstraintPath) bool;

pub extern fn zjoltPathConstraintPathGetMaxFraction(path: *const PathConstraintPath) f32;

pub extern fn zjoltPathConstraintPathGetClosestPoint(path: *const PathConstraintPath, position: *const Vec3, fraction_hint: f32, out_fraction: *f32) Result;

pub extern fn zjoltPathConstraintPathGetPointOnPath(path: *const PathConstraintPath, fraction: f32, out_position: ?*Vec3, out_tangent: ?*Vec3, out_normal: ?*Vec3, out_binormal: ?*Vec3) Result;

pub extern fn zjoltConstraintCreateFixed(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const FixedConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreatePoint(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const PointConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreateHinge(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const HingeConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreateSlider(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const SliderConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreateDistance(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const DistanceConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreateCone(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const ConeConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreateSwingTwist(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const SwingTwistConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreateSixDof(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const SixDofConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreateGear(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const GearConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreateRackAndPinion(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const RackAndPinionConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreatePulley(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const PulleyConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintCreatePath(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const PathConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintAddRef(constraint: *const Constraint) void;

pub extern fn zjoltConstraintRelease(constraint: *const Constraint) void;

pub extern fn zjoltConstraintGetRefCount(constraint: *const Constraint) u32;

pub extern fn zjoltConstraintAdd(system: *PhysicsSystem, constraint: *Constraint) Result;

pub extern fn zjoltConstraintRemove(system: *PhysicsSystem, constraint: *Constraint) Result;

pub extern fn zjoltConstraintIsAdded(system: *const PhysicsSystem, constraint: *const Constraint) bool;

pub extern fn zjoltPhysicsSystemGetNumConstraints(system: *const PhysicsSystem) u32;

pub extern fn zjoltConstraintGetSubType(constraint: *const Constraint) ConstraintSubType;

pub extern fn zjoltConstraintSetEnabled(constraint: *Constraint, enabled: bool) void;

pub extern fn zjoltConstraintIsEnabled(constraint: *const Constraint) bool;

pub extern fn zjoltConstraintIsActive(constraint: *const Constraint) bool;

pub extern fn zjoltConstraintActivate(system: *PhysicsSystem, constraint: *Constraint) Result;

pub extern fn zjoltConstraintSetUserData(constraint: *Constraint, user_data: u64) void;

pub extern fn zjoltConstraintGetUserData(constraint: *const Constraint) u64;

pub extern fn zjoltConstraintSetPriority(constraint: *Constraint, priority: u32) void;

pub extern fn zjoltConstraintGetPriority(constraint: *const Constraint) u32;

pub extern fn zjoltConstraintSetNumVelocityStepsOverride(constraint: *Constraint, steps: u32) Result;

pub extern fn zjoltConstraintGetNumVelocityStepsOverride(constraint: *const Constraint) u32;

pub extern fn zjoltConstraintSetNumPositionStepsOverride(constraint: *Constraint, steps: u32) Result;

pub extern fn zjoltConstraintGetNumPositionStepsOverride(constraint: *const Constraint) u32;

pub extern fn zjoltConstraintGetBodies(constraint: *const Constraint, out_body1: ?*BodyId, out_body2: ?*BodyId) Result;

pub extern fn zjoltConstraintGetConstraintToBody1Matrix(constraint: *const Constraint, out: *Mat44) Result;

pub extern fn zjoltConstraintGetConstraintToBody2Matrix(constraint: *const Constraint, out: *Mat44) Result;

pub extern fn zjoltConstraintSetDrawSize(constraint: *Constraint, size: f32) Result;

pub extern fn zjoltConstraintGetDrawSize(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltConstraintResetWarmStart(constraint: *Constraint) void;

//=============================================================================
// Constraint settings
//=============================================================================

pub const ConstraintSettings = opaque {};

pub extern fn zjoltConstraintGetConstraintSettings(constraint: *const Constraint, out: **ConstraintSettings) Result;

pub extern fn zjoltConstraintSettingsAddRef(settings: *const ConstraintSettings) void;
pub extern fn zjoltConstraintSettingsRelease(settings: *const ConstraintSettings) void;
pub extern fn zjoltConstraintSettingsGetRefCount(settings: *const ConstraintSettings) u32;

pub extern fn zjoltConstraintSettingsGetEnabled(settings: *const ConstraintSettings) bool;
pub extern fn zjoltConstraintSettingsGetConstraintPriority(settings: *const ConstraintSettings) u32;
pub extern fn zjoltConstraintSettingsGetNumVelocityStepsOverride(settings: *const ConstraintSettings) u32;
pub extern fn zjoltConstraintSettingsGetNumPositionStepsOverride(settings: *const ConstraintSettings) u32;
pub extern fn zjoltConstraintSettingsGetDrawConstraintSize(settings: *const ConstraintSettings) f32;
pub extern fn zjoltConstraintSettingsGetUserData(settings: *const ConstraintSettings) u64;

pub extern fn zjoltConstraintSettingsSaveBinaryState(settings: *const ConstraintSettings, stream: *const core.Stream) Result;
pub extern fn zjoltConstraintSettingsRestoreBinaryState(stream: *const core.Stream, out: **ConstraintSettings) Result;

pub extern fn zjoltFixedConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltFixedConstraintGetTotalLambdaRotation(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltPointConstraintSetPoint1(constraint: *Constraint, space: ConstraintSpace, point: *const RVec3) Result;

pub extern fn zjoltPointConstraintSetPoint2(constraint: *Constraint, space: ConstraintSpace, point: *const RVec3) Result;

pub extern fn zjoltPointConstraintGetLocalSpacePoint1(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltPointConstraintGetLocalSpacePoint2(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltPointConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetLocalSpacePoint1(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetLocalSpacePoint2(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetLocalSpaceHingeAxis1(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetLocalSpaceHingeAxis2(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetLocalSpaceNormalAxis1(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetLocalSpaceNormalAxis2(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetCurrentAngle(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltHingeConstraintSetLimits(constraint: *Constraint, min: f32, max: f32) Result;

pub extern fn zjoltHingeConstraintGetLimits(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;

pub extern fn zjoltHingeConstraintHasLimits(constraint: *const Constraint, out: *bool) Result;

pub extern fn zjoltHingeConstraintSetLimitsSpringSettings(constraint: *Constraint, spring: *const SpringSettings) Result;

pub extern fn zjoltHingeConstraintGetLimitsSpringSettings(constraint: *const Constraint, out: *SpringSettings) Result;

pub extern fn zjoltHingeConstraintSetMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;

pub extern fn zjoltHingeConstraintGetMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;

pub extern fn zjoltHingeConstraintSetMotorState(constraint: *Constraint, state: MotorState) Result;

pub extern fn zjoltHingeConstraintGetMotorState(constraint: *const Constraint, out: *MotorState) Result;

pub extern fn zjoltHingeConstraintSetTargetAngularVelocity(constraint: *Constraint, angular_velocity: f32) Result;

pub extern fn zjoltHingeConstraintGetTargetAngularVelocity(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltHingeConstraintSetTargetAngle(constraint: *Constraint, angle: f32) Result;

pub extern fn zjoltHingeConstraintGetTargetAngle(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltHingeConstraintSetTargetOrientation(constraint: *Constraint, orientation: *const Quat) Result;

pub extern fn zjoltHingeConstraintSetMaxFrictionTorque(constraint: *Constraint, torque: f32) Result;

pub extern fn zjoltHingeConstraintGetMaxFrictionTorque(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltHingeConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetTotalLambdaRotation(constraint: *const Constraint, out_x: ?*f32, out_y: ?*f32) Result;

pub extern fn zjoltHingeConstraintGetTotalLambdaRotationLimits(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltHingeConstraintGetTotalLambdaMotor(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSliderConstraintGetCurrentPosition(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSliderConstraintSetLimits(constraint: *Constraint, min: f32, max: f32) Result;

pub extern fn zjoltSliderConstraintGetLimits(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;

pub extern fn zjoltSliderConstraintHasLimits(constraint: *const Constraint, out: *bool) Result;

pub extern fn zjoltSliderConstraintSetLimitsSpringSettings(constraint: *Constraint, spring: *const SpringSettings) Result;

pub extern fn zjoltSliderConstraintGetLimitsSpringSettings(constraint: *const Constraint, out: *SpringSettings) Result;

pub extern fn zjoltSliderConstraintSetMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;

pub extern fn zjoltSliderConstraintGetMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;

pub extern fn zjoltSliderConstraintSetMotorState(constraint: *Constraint, state: MotorState) Result;

pub extern fn zjoltSliderConstraintGetMotorState(constraint: *const Constraint, out: *MotorState) Result;

pub extern fn zjoltSliderConstraintSetTargetVelocity(constraint: *Constraint, velocity: f32) Result;

pub extern fn zjoltSliderConstraintGetTargetVelocity(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSliderConstraintSetTargetPosition(constraint: *Constraint, position: f32) Result;

pub extern fn zjoltSliderConstraintGetTargetPosition(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSliderConstraintSetMaxFrictionForce(constraint: *Constraint, force: f32) Result;

pub extern fn zjoltSliderConstraintGetMaxFrictionForce(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSliderConstraintGetTotalLambdaPosition(constraint: *const Constraint, out_x: ?*f32, out_y: ?*f32) Result;

pub extern fn zjoltSliderConstraintGetTotalLambdaPositionLimits(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSliderConstraintGetTotalLambdaRotation(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSliderConstraintGetTotalLambdaMotor(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltDistanceConstraintSetDistance(constraint: *Constraint, min: f32, max: f32) Result;

pub extern fn zjoltDistanceConstraintGetDistance(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;

pub extern fn zjoltDistanceConstraintSetLimitsSpringSettings(constraint: *Constraint, spring: *const SpringSettings) Result;

pub extern fn zjoltDistanceConstraintGetLimitsSpringSettings(constraint: *const Constraint, out: *SpringSettings) Result;

pub extern fn zjoltDistanceConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltConeConstraintSetHalfConeAngle(constraint: *Constraint, half_cone_angle: f32) Result;

pub extern fn zjoltConeConstraintGetCosHalfConeAngle(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltConeConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltConeConstraintGetTotalLambdaRotation(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSwingTwistConstraintGetLocalSpacePosition1(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSwingTwistConstraintGetLocalSpacePosition2(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSwingTwistConstraintGetConstraintToBody1(constraint: *const Constraint, out: *Quat) Result;

pub extern fn zjoltSwingTwistConstraintGetConstraintToBody2(constraint: *const Constraint, out: *Quat) Result;

pub extern fn zjoltSwingTwistConstraintSetNormalHalfConeAngle(constraint: *Constraint, angle: f32) Result;

pub extern fn zjoltSwingTwistConstraintGetNormalHalfConeAngle(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSwingTwistConstraintSetPlaneHalfConeAngle(constraint: *Constraint, angle: f32) Result;

pub extern fn zjoltSwingTwistConstraintGetPlaneHalfConeAngle(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSwingTwistConstraintSetTwistLimits(constraint: *Constraint, min: f32, max: f32) Result;

pub extern fn zjoltSwingTwistConstraintGetTwistLimits(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;

pub extern fn zjoltSwingTwistConstraintSetSwingMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;

pub extern fn zjoltSwingTwistConstraintGetSwingMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;

pub extern fn zjoltSwingTwistConstraintSetSwingMotorState(constraint: *Constraint, state: MotorState) Result;

pub extern fn zjoltSwingTwistConstraintGetSwingMotorState(constraint: *const Constraint, out: *MotorState) Result;

pub extern fn zjoltSwingTwistConstraintSetTwistMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;

pub extern fn zjoltSwingTwistConstraintGetTwistMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;

pub extern fn zjoltSwingTwistConstraintSetTwistMotorState(constraint: *Constraint, state: MotorState) Result;

pub extern fn zjoltSwingTwistConstraintGetTwistMotorState(constraint: *const Constraint, out: *MotorState) Result;

pub extern fn zjoltSwingTwistConstraintSetMaxFrictionTorque(constraint: *Constraint, torque: f32) Result;

pub extern fn zjoltSwingTwistConstraintGetMaxFrictionTorque(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSwingTwistConstraintSetTargetAngularVelocity(constraint: *Constraint, angular_velocity: *const Vec3) Result;

pub extern fn zjoltSwingTwistConstraintGetTargetAngularVelocity(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSwingTwistConstraintSetTargetAngularVelocityBodySpace(constraint: *Constraint, angular_velocity: *const Vec3) Result;

pub extern fn zjoltSwingTwistConstraintSetTargetOrientation(constraint: *Constraint, orientation: *const Quat) Result;

pub extern fn zjoltSwingTwistConstraintSetTargetOrientationBodySpace(constraint: *Constraint, orientation: *const Quat) Result;

pub extern fn zjoltSwingTwistConstraintGetTargetOrientation(constraint: *const Constraint, out: *Quat) Result;

pub extern fn zjoltSwingTwistConstraintGetRotationInConstraintSpace(constraint: *const Constraint, out: *Quat) Result;

pub extern fn zjoltSwingTwistConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSwingTwistConstraintGetTotalLambdaTwist(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSwingTwistConstraintGetTotalLambdaSwingY(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSwingTwistConstraintGetTotalLambdaSwingZ(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSwingTwistConstraintGetTotalLambdaMotor(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSixDofConstraintSetTranslationLimits(constraint: *Constraint, min: *const Vec3, max: *const Vec3) Result;

pub extern fn zjoltSixDofConstraintSetRotationLimits(constraint: *Constraint, min: *const Vec3, max: *const Vec3) Result;

pub extern fn zjoltSixDofConstraintGetLimits(constraint: *const Constraint, axis: SixDofAxis, out_min: ?*f32, out_max: ?*f32) Result;

pub extern fn zjoltSixDofConstraintIsFixedAxis(constraint: *const Constraint, axis: SixDofAxis, out: *bool) Result;

pub extern fn zjoltSixDofConstraintIsFreeAxis(constraint: *const Constraint, axis: SixDofAxis, out: *bool) Result;

pub extern fn zjoltSixDofConstraintSetMaxFriction(constraint: *Constraint, axis: SixDofAxis, friction: f32) Result;

pub extern fn zjoltSixDofConstraintGetMaxFriction(constraint: *const Constraint, axis: SixDofAxis, out: *f32) Result;

pub extern fn zjoltSixDofConstraintSetMotorSettings(constraint: *Constraint, axis: SixDofAxis, motor: *const MotorSettings) Result;

pub extern fn zjoltSixDofConstraintGetMotorSettings(constraint: *const Constraint, axis: SixDofAxis, out: *MotorSettings) Result;

pub extern fn zjoltSixDofConstraintSetMotorState(constraint: *Constraint, axis: SixDofAxis, state: MotorState) Result;

pub extern fn zjoltSixDofConstraintGetMotorState(constraint: *const Constraint, axis: SixDofAxis, out: *MotorState) Result;

pub extern fn zjoltSixDofConstraintSetLimitsSpringSettings(constraint: *Constraint, axis: SixDofAxis, spring: *const SpringSettings) Result;

pub extern fn zjoltSixDofConstraintGetLimitsSpringSettings(constraint: *const Constraint, axis: SixDofAxis, out: *SpringSettings) Result;

pub extern fn zjoltSixDofConstraintSetTargetVelocity(constraint: *Constraint, velocity: *const Vec3) Result;

pub extern fn zjoltSixDofConstraintGetTargetVelocity(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSixDofConstraintSetTargetAngularVelocity(constraint: *Constraint, angular_velocity: *const Vec3) Result;

pub extern fn zjoltSixDofConstraintGetTargetAngularVelocity(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSixDofConstraintSetTargetPosition(constraint: *Constraint, position: *const Vec3) Result;

pub extern fn zjoltSixDofConstraintGetTargetPosition(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSixDofConstraintSetTargetOrientation(constraint: *Constraint, orientation: *const Quat) Result;

pub extern fn zjoltSixDofConstraintGetTargetOrientation(constraint: *const Constraint, out: *Quat) Result;

pub extern fn zjoltSixDofConstraintSetTargetOrientationBodySpace(constraint: *Constraint, orientation: *const Quat) Result;

pub extern fn zjoltSixDofConstraintGetRotationInConstraintSpace(constraint: *const Constraint, out: *Quat) Result;

pub extern fn zjoltSixDofConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSixDofConstraintGetTotalLambdaRotation(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSixDofConstraintGetTotalLambdaMotorTranslation(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSixDofConstraintGetTotalLambdaMotorRotation(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltGearConstraintSetConstraints(constraint: *Constraint, gear1: ?*Constraint, gear2: ?*Constraint) Result;

pub extern fn zjoltGearConstraintGetTotalLambda(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltRackAndPinionConstraintSetConstraints(constraint: *Constraint, pinion: ?*Constraint, rack: ?*Constraint) Result;

pub extern fn zjoltRackAndPinionConstraintGetTotalLambda(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPulleyConstraintSetLength(constraint: *Constraint, min: f32, max: f32) Result;

pub extern fn zjoltPulleyConstraintGetLength(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;

pub extern fn zjoltPulleyConstraintGetCurrentLength(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPulleyConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPathConstraintSetPath(constraint: *Constraint, path: ?*const PathConstraintPath, fraction: f32) Result;

pub extern fn zjoltPathConstraintGetPath(constraint: *const Constraint, out: *?*const PathConstraintPath) Result;

pub extern fn zjoltPathConstraintGetPathFraction(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPathConstraintSetMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;

pub extern fn zjoltPathConstraintGetMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;

pub extern fn zjoltPathConstraintSetMotorState(constraint: *Constraint, state: MotorState) Result;

pub extern fn zjoltPathConstraintGetMotorState(constraint: *const Constraint, out: *MotorState) Result;

pub extern fn zjoltPathConstraintSetTargetVelocity(constraint: *Constraint, velocity: f32) Result;

pub extern fn zjoltPathConstraintGetTargetVelocity(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPathConstraintSetTargetPathFraction(constraint: *Constraint, fraction: f32) Result;

pub extern fn zjoltPathConstraintGetTargetPathFraction(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPathConstraintSetMaxFrictionForce(constraint: *Constraint, force: f32) Result;

pub extern fn zjoltPathConstraintGetMaxFrictionForce(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPathConstraintGetTotalLambdaPosition(constraint: *const Constraint, out_x: ?*f32, out_y: ?*f32) Result;

pub extern fn zjoltPathConstraintGetTotalLambdaPositionLimits(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPathConstraintGetTotalLambdaRotationHinge(constraint: *const Constraint, out_x: ?*f32, out_y: ?*f32) Result;

pub extern fn zjoltPathConstraintGetTotalLambdaRotation(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltPathConstraintGetTotalLambdaMotor(constraint: *const Constraint, out: *f32) Result;

//=============================================================================
// Custom constraints
//=============================================================================

/// Body state a custom constraint's solver callbacks read and write. @see
/// ffi/zjolt_constraint.h's ZJoltSolverBody for what each field means and the
/// "world space, already rotated" contract on `inverse_inertia`.
pub const SolverBody = extern struct {
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    position_delta: Vec3,
    rotation_delta: Vec3,
    inverse_mass: f32,
    inverse_inertia: [9]f32,
    center_of_mass: Vec3,
    rotation: Quat,
    is_dynamic: bool,
};

pub const SolverBodyPair = extern struct {
    body1: SolverBody,
    body2: SolverBody,
};

/// One function pointer per Jolt solver virtual. Every one but `draw` and
/// `destroy` is required — @see zjoltConstraintCreateCustom.
pub const CustomConstraintCallbacks = extern struct {
    setup_velocity: ?*const fn (user: ?*anyopaque, bodies: *SolverBodyPair, delta_time: f32) callconv(.c) void = null,
    warm_start_velocity: ?*const fn (user: ?*anyopaque, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) callconv(.c) void = null,
    solve_velocity: ?*const fn (user: ?*anyopaque, bodies: *SolverBodyPair, delta_time: f32) callconv(.c) bool = null,
    solve_position: ?*const fn (user: ?*anyopaque, bodies: *SolverBodyPair, delta_time: f32, baumgarte: f32) callconv(.c) bool = null,
    reset_warm_start: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
    is_active: ?*const fn (user: ?*anyopaque) callconv(.c) bool = null,
    notify_shape_changed: ?*const fn (user: ?*anyopaque, body: BodyId, delta_center_of_mass: Vec3) callconv(.c) void = null,
    save_state: ?*const fn (user: ?*anyopaque, recorder: *StateRecorder) callconv(.c) void = null,
    restore_state: ?*const fn (user: ?*anyopaque, recorder: *StateRecorder) callconv(.c) void = null,
    draw: ?*const fn (user: ?*anyopaque, renderer: *debug.DebugRenderer) callconv(.c) void = null,
    destroy: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
};

pub const CustomConstraintDesc = extern struct {
    body1: BodyId,
    body2: BodyId,
    constraint_to_body1: Mat44,
    constraint_to_body2: Mat44,
    enabled: bool = true,
    num_velocity_steps_override: u32 = 0,
    num_position_steps_override: u32 = 0,
    draw_constraint_size: f32 = 1,
    callbacks: CustomConstraintCallbacks = .{},
    user: ?*anyopaque = null,
};

pub extern fn zjoltStateRecorderWriteBytes(recorder: *StateRecorder, data: ?*const anyopaque, size: usize) void;

pub extern fn zjoltStateRecorderReadBytes(recorder: *StateRecorder, data: ?*anyopaque, size: usize) void;

pub extern fn zjoltConstraintCreateCustom(system: *PhysicsSystem, desc: *const CustomConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintGetCustomUserData(constraint: *const Constraint, out: *?*anyopaque) Result;
