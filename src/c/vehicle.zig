//! ZJolt C declarations for wheeled and tracked vehicles and their drivetrains.
//!
//! Mirrors `ffi/zjolt_vehicle.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const constraint = @import("constraint.zig");
const core = @import("core.zig");
const debug = @import("debug.zig");
const query = @import("query.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Constraint = constraint.Constraint;
pub const BodyId = core.BodyId;
pub const ObjectLayer = core.ObjectLayer;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
// For zjoltVehicleConstraintEngineDrawRPM, which zjolt_vehicle.h reuses
// zjolt_debug.h's opaque handle for rather than respelling it.
pub const DebugRenderer = debug.DebugRenderer;

// A wheel's ground test filters like any other cast, so `zjolt_vehicle.h`
// reuses the query header's three filter tables rather than respelling them.
pub const BroadPhaseLayerFilter = query.BroadPhaseLayerFilter;
pub const ObjectLayerFilter = query.ObjectLayerFilter;
pub const BodyFilter = query.BodyFilter;

pub const VehicleControllerKind = enum(c_int) {
    wheeled = 0,
    tracked = 1,
    motorcycle = 2,
};

pub const VehicleCollisionTesterKind = enum(c_int) {
    ray = 0,
    cast_sphere = 1,
    cast_cylinder = 2,
    /// Set by zjoltVehicleConstraintSetCollisionTesterCallback rather than
    /// zjoltVehicleConstraintSetCollisionTester.
    callback = 3,
};

pub const VehicleTransmissionMode = enum(c_int) {
    auto = 0,
    manual = 1,
};

pub const vehicle_curve_max_points: usize = 8;

pub const VehicleCurvePoint = extern struct {
    x: f32,
    y: f32,
};

pub const VehicleCurveDesc = extern struct {
    points: [8]VehicleCurvePoint,
    count: u32,
};

pub const VehicleWheelDesc = extern struct {
    position: Vec3,
    suspension_force_point: Vec3,
    enable_suspension_force_point: bool,
    suspension_direction: Vec3,
    steering_axis: Vec3,
    wheel_up: Vec3,
    wheel_forward: Vec3,
    suspension_min_length: f32,
    suspension_max_length: f32,
    suspension_preload_length: f32,
    suspension_spring_frequency: f32,
    suspension_spring_damping: f32,
    radius: f32,
    width: f32,
    inertia: f32,
    angular_damping: f32,
    max_steer_angle: f32,
    longitudinal_friction: VehicleCurveDesc,
    lateral_friction: VehicleCurveDesc,
    max_brake_torque: f32,
    max_hand_brake_torque: f32,
    tracked_longitudinal_friction: f32,
    tracked_lateral_friction: f32,
};

pub const VehicleEngineDesc = extern struct {
    max_torque: f32,
    min_rpm: f32,
    max_rpm: f32,
    normalized_torque: VehicleCurveDesc,
    inertia: f32,
    angular_damping: f32,
};

pub const VehicleTransmissionDesc = extern struct {
    mode: VehicleTransmissionMode,
    forward_gear_ratios: ?[*]const f32,
    forward_gear_ratio_count: u32,
    reverse_gear_ratios: ?[*]const f32,
    reverse_gear_ratio_count: u32,
    switch_time: f32,
    clutch_release_time: f32,
    switch_latency: f32,
    shift_up_rpm: f32,
    shift_down_rpm: f32,
    clutch_strength: f32,
};

pub const VehicleDifferentialDesc = extern struct {
    left_wheel: i32,
    right_wheel: i32,
    differential_ratio: f32,
    left_right_split: f32,
    limited_slip_ratio: f32,
    engine_torque_ratio: f32,
};

pub const VehicleAntiRollBarDesc = extern struct {
    left_wheel: i32,
    right_wheel: i32,
    stiffness: f32,
};

pub const VehicleCollisionTesterDesc = extern struct {
    kind: VehicleCollisionTesterKind,
    object_layer: ObjectLayer,
    up: Vec3,
    max_slope_angle: f32,
    radius: f32,
    convex_radius_fraction: f32,
};

/// What VehicleConstraint::OnStep already knows before it asks a custom
/// tester: which wheel, and the suspension ray to test.
pub const VehicleGroundTestInput = extern struct {
    wheel_index: u32,
    origin: RVec3,
    direction: Vec3,
    /// Excluded from a test of your own the way the three built-in testers
    /// exclude it — this ABI does not do that exclusion for you.
    vehicle_body: BodyId,
};

/// What a collision test reports, or — for
/// VehicleCollisionTesterCallback.predict_contact_properties — the previous
/// step's contact going in and the extrapolated one coming out.
pub const VehicleGroundContact = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    position: RVec3,
    normal: Vec3,
    /// Metres, clamped into [0, suspension_max_length] the way Jolt's own
    /// testers clamp it.
    suspension_length: f32,
};

/// A custom wheel-ground collision tester, reached as a callback pair rather
/// than a mirrored C++ vtable. Both fields run once per wheel from inside
/// VehicleConstraint::OnStep — job thread, every body and constraint locked,
/// nothing may unwind — under the same rules as the step callbacks below, and
/// neither is handed a system to query with; see `ffi/zjolt_vehicle.h` for
/// why.
pub const VehicleCollisionTesterCallback = extern struct {
    collide: ?*const fn (
        user: ?*anyopaque,
        input: *const VehicleGroundTestInput,
        out_contact: *VehicleGroundContact,
    ) callconv(.c) bool = null,
    predict_contact_properties: ?*const fn (
        user: ?*anyopaque,
        input: *const VehicleGroundTestInput,
        contact: *VehicleGroundContact,
    ) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub const VehicleMotorcycleDesc = extern struct {
    max_lean_angle: f32,
    lean_spring_constant: f32,
    lean_spring_damping: f32,
    lean_spring_integration_coefficient: f32,
    lean_spring_integration_coefficient_decay: f32,
    lean_smoothing_factor: f32,
};

pub const VehicleTrackSide = enum(c_int) {
    left = 0,
    right = 1,
};

pub const VehicleTrackDesc = extern struct {
    inertia: f32,
    angular_damping: f32,
    max_brake_torque: f32,
    differential_ratio: f32,
};

pub const VehicleMotorcycleLeanSpring = extern struct {
    spring_constant: f32,
    spring_damping: f32,
    spring_integration_coefficient: f32,
    spring_integration_coefficient_decay: f32,
    lean_smoothing_factor: f32,
};

pub const VehicleWheelFilters = extern struct {
    broad_phase_layer: BroadPhaseLayerFilter = .{},
    object_layer: ObjectLayerFilter = .{},
    body: BodyFilter = .{},
};

pub const VehicleStepContext = extern struct {
    delta_time: f32,
    is_first_step: bool,
    is_last_step: bool,
};

pub const VehicleConstraint = opaque {};

pub const VehicleStepCallback = extern struct {
    on_step: ?*const fn (
        user: ?*anyopaque,
        constraint_handle: *VehicleConstraint,
        context: *const VehicleStepContext,
    ) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub const VehicleCombineFrictionCallback = extern struct {
    combine: ?*const fn (
        user: ?*anyopaque,
        wheel_index: u32,
        body: BodyId,
        sub_shape_id: SubShapeId,
        longitudinal_friction: *f32,
        lateral_friction: *f32,
    ) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub const VehicleTireImpulseInputs = extern struct {
    suspension_impulse: f32,
    longitudinal_friction: f32,
    lateral_friction: f32,
    longitudinal_slip: f32,
    lateral_slip: f32,
    delta_time: f32,
};

pub const VehicleTireMaxImpulseCallback = extern struct {
    compute: ?*const fn (
        user: ?*anyopaque,
        wheel_index: u32,
        inputs: *const VehicleTireImpulseInputs,
        out_longitudinal_impulse: *f32,
        out_lateral_impulse: *f32,
    ) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub const VehicleConstraintDesc = extern struct {
    up: Vec3,
    forward: Vec3,
    max_pitch_roll_angle: f32,
    wheels: ?[*]const VehicleWheelDesc,
    wheel_count: u32,
    anti_roll_bars: ?[*]const VehicleAntiRollBarDesc,
    anti_roll_bar_count: u32,
    controller_kind: VehicleControllerKind,
    engine: VehicleEngineDesc,
    transmission: VehicleTransmissionDesc,
    differentials: ?[*]const VehicleDifferentialDesc,
    differential_count: u32,
    differential_limited_slip_ratio: f32,
    left_track_wheels: ?[*]const u32,
    left_track_wheel_count: u32,
    left_track_driven_wheel: u32,
    right_track_wheels: ?[*]const u32,
    right_track_wheel_count: u32,
    right_track_driven_wheel: u32,
    track_inertia: f32,
    track_angular_damping: f32,
    track_max_brake_torque: f32,
    track_differential_ratio: f32,
    motorcycle: VehicleMotorcycleDesc,
    collision_tester: VehicleCollisionTesterDesc,
};

pub extern fn zjoltVehicleWheelDescInit(desc: *VehicleWheelDesc) void;

pub extern fn zjoltVehicleEngineDescInit(desc: *VehicleEngineDesc) void;

pub extern fn zjoltVehicleTransmissionDescInit(desc: *VehicleTransmissionDesc) void;

pub extern fn zjoltVehicleDifferentialDescInit(desc: *VehicleDifferentialDesc) void;

pub extern fn zjoltVehicleAntiRollBarDescInit(desc: *VehicleAntiRollBarDesc) void;

pub extern fn zjoltVehicleCollisionTesterDescInit(desc: *VehicleCollisionTesterDesc) void;

pub extern fn zjoltVehicleMotorcycleDescInit(desc: *VehicleMotorcycleDesc) void;

pub extern fn zjoltVehicleConstraintDescInit(desc: *VehicleConstraintDesc) void;

pub extern fn zjoltVehicleConstraintCreate(system: *PhysicsSystem, body: BodyId, desc: *const VehicleConstraintDesc, out: **VehicleConstraint) Result;

pub extern fn zjoltVehicleConstraintDestroy(constraint: *VehicleConstraint) void;

pub extern fn zjoltVehicleConstraintGetControllerKind(constraint: *const VehicleConstraint) VehicleControllerKind;

pub extern fn zjoltVehicleConstraintGetCollisionTesterKind(constraint: *const VehicleConstraint) VehicleCollisionTesterKind;

pub extern fn zjoltVehicleConstraintGetBodyId(constraint: *const VehicleConstraint) BodyId;

pub extern fn zjoltVehicleConstraintIsActive(constraint: *const VehicleConstraint) bool;

pub extern fn zjoltVehicleConstraintGetMaxPitchRollAngle(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintSetMaxPitchRollAngle(constraint: *VehicleConstraint, max_pitch_roll_angle: f32) void;

pub extern fn zjoltVehicleConstraintGetLocalUp(constraint: *const VehicleConstraint, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetLocalForward(constraint: *const VehicleConstraint, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetWorldUp(constraint: *const VehicleConstraint, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetWheelCount(constraint: *const VehicleConstraint) u32;

pub extern fn zjoltVehicleConstraintGetWheelRotationAngle(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintSetWheelRotationAngle(constraint: *VehicleConstraint, wheel_index: u32, angle: f32) void;

pub extern fn zjoltVehicleConstraintGetWheelSteerAngle(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintSetWheelSteerAngle(constraint: *VehicleConstraint, wheel_index: u32, angle: f32) void;

pub extern fn zjoltVehicleConstraintGetWheelAngularVelocity(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintSetWheelAngularVelocity(constraint: *VehicleConstraint, wheel_index: u32, velocity: f32) void;

pub extern fn zjoltVehicleConstraintGetWheelSuspensionLength(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintHasWheelContact(constraint: *const VehicleConstraint, wheel_index: u32) bool;

pub extern fn zjoltVehicleConstraintGetWheelContactBodyId(constraint: *const VehicleConstraint, wheel_index: u32) BodyId;

pub extern fn zjoltVehicleConstraintGetWheelContactPosition(constraint: *const VehicleConstraint, wheel_index: u32, out: *RVec3) void;

pub extern fn zjoltVehicleConstraintGetWheelContactNormal(constraint: *const VehicleConstraint, wheel_index: u32, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetWheelContactLongitudinal(constraint: *const VehicleConstraint, wheel_index: u32, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetWheelContactLateral(constraint: *const VehicleConstraint, wheel_index: u32, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetWheelContactPointVelocity(constraint: *const VehicleConstraint, wheel_index: u32, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetForwardInput(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintGetBrakeInput(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintGetEngineRpm(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintGetCurrentGear(constraint: *const VehicleConstraint) i32;

pub extern fn zjoltVehicleConstraintIsSwitchingGear(constraint: *const VehicleConstraint) bool;

pub extern fn zjoltVehicleConstraintGetClutchFriction(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintGetGearRatio(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintSetGear(constraint: *VehicleConstraint, gear: i32, clutch_friction: f32) Result;

pub extern fn zjoltVehicleConstraintEngineApplyTorque(constraint: *VehicleConstraint, torque: f32, delta_time: f32) Result;

pub extern fn zjoltVehicleConstraintEngineApplyDamping(constraint: *VehicleConstraint, delta_time: f32) Result;

pub extern fn zjoltVehicleConstraintEngineClampRPM(constraint: *VehicleConstraint) Result;

pub extern fn zjoltVehicleConstraintEngineAllowSleep(constraint: *const VehicleConstraint) bool;

pub extern fn zjoltVehicleConstraintTransmissionAllowSleep(constraint: *const VehicleConstraint) bool;

pub extern fn zjoltVehicleConstraintEngineConvertRPMToAngle(constraint: *const VehicleConstraint, rpm: f32, out_angle: *f32) Result;

pub extern fn zjoltVehicleConstraintEngineDrawRPM(constraint: *VehicleConstraint, renderer: *DebugRenderer, position: *const RVec3, forward: *const Vec3, up: *const Vec3, size: f32, shift_down_rpm: f32, shift_up_rpm: f32) Result;

pub extern fn zjoltVehicleConstraintGetEngineDesc(constraint: *const VehicleConstraint, out: *VehicleEngineDesc) Result;

pub extern fn zjoltVehicleConstraintGetTransmissionDesc(constraint: *const VehicleConstraint, out: *VehicleTransmissionDesc) Result;

pub extern fn zjoltVehicleConstraintGetGearRatios(constraint: *const VehicleConstraint, out_forward: ?[*]f32, forward_capacity: u32, out_forward_count: *u32, out_reverse: ?[*]f32, reverse_capacity: u32, out_reverse_count: *u32) Result;

pub extern fn zjoltVehicleConstraintCalculateDifferentialTorqueRatio(constraint: *const VehicleConstraint, differential_index: u32, left_angular_velocity: f32, right_angular_velocity: f32, out_left_torque_fraction: *f32, out_right_torque_fraction: *f32) Result;

pub extern fn zjoltVehicleConstraintCalculateTrackedWheelAngularVelocity(constraint: *VehicleConstraint, wheel_index: u32) Result;

pub extern fn zjoltVehicleConstraintSetWheeledDriverInput(constraint: *VehicleConstraint, forward: f32, right: f32, brake: f32, hand_brake: f32) Result;

pub extern fn zjoltVehicleConstraintGetWheeledRightInput(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintGetWheeledHandBrakeInput(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintSetTrackedDriverInput(constraint: *VehicleConstraint, forward: f32, left_ratio: f32, right_ratio: f32, brake: f32) Result;

pub extern fn zjoltVehicleConstraintGetTrackedLeftRatio(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintGetTrackedRightRatio(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintSetMotorcycleLeanControllerEnabled(constraint: *VehicleConstraint, enabled: bool) Result;

pub extern fn zjoltVehicleConstraintIsMotorcycleLeanControllerEnabled(constraint: *const VehicleConstraint) bool;

pub extern fn zjoltVehicleConstraintSetMotorcycleLeanSteeringLimitEnabled(constraint: *VehicleConstraint, enabled: bool) Result;

pub extern fn zjoltVehicleConstraintIsMotorcycleLeanSteeringLimitEnabled(constraint: *const VehicleConstraint) bool;

pub extern fn zjoltVehicleConstraintGetMotorcycleWheelBase(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintOverrideGravity(constraint: *VehicleConstraint, gravity: *const Vec3) void;

pub extern fn zjoltVehicleConstraintResetGravityOverride(constraint: *VehicleConstraint) void;

pub extern fn zjoltVehicleConstraintIsGravityOverridden(constraint: *const VehicleConstraint) bool;

pub extern fn zjoltVehicleConstraintGetGravityOverride(constraint: *const VehicleConstraint, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetNumStepsBetweenCollisionTestActive(constraint: *const VehicleConstraint) u32;

pub extern fn zjoltVehicleConstraintSetNumStepsBetweenCollisionTestActive(constraint: *VehicleConstraint, steps: u32) void;

pub extern fn zjoltVehicleConstraintGetNumStepsBetweenCollisionTestInactive(constraint: *const VehicleConstraint) u32;

pub extern fn zjoltVehicleConstraintSetNumStepsBetweenCollisionTestInactive(constraint: *VehicleConstraint, steps: u32) void;

pub extern fn zjoltVehicleConstraintGetWheelContactSubShapeId(constraint: *const VehicleConstraint, wheel_index: u32) SubShapeId;

pub extern fn zjoltVehicleConstraintHasWheelHitHardPoint(constraint: *const VehicleConstraint, wheel_index: u32) bool;

pub extern fn zjoltVehicleConstraintGetWheelSuspensionLambda(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintGetWheelLongitudinalLambda(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintGetWheelLateralLambda(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintGetWheelBrakeImpulse(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintGetWheelLocalBasis(constraint: *const VehicleConstraint, wheel_index: u32, out_forward: *Vec3, out_up: *Vec3, out_right: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetWheelLocalTransform(constraint: *const VehicleConstraint, wheel_index: u32, wheel_right: *const Vec3, wheel_up: *const Vec3, out_position: *Vec3, out_rotation: *Quat) void;

pub extern fn zjoltVehicleConstraintGetWheelWorldTransform(constraint: *const VehicleConstraint, wheel_index: u32, wheel_right: *const Vec3, wheel_up: *const Vec3, out_position: *RVec3, out_rotation: *Quat) void;

pub extern fn zjoltVehicleConstraintGetTrackedWheelBrakeImpulse(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintGetWheelCombinedLongitudinalFriction(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintGetWheelCombinedLateralFriction(constraint: *const VehicleConstraint, wheel_index: u32) f32;

pub extern fn zjoltVehicleConstraintGetWheelTrackIndex(constraint: *const VehicleConstraint, wheel_index: u32) i32;

pub extern fn zjoltVehicleConstraintGetWheelLongitudinalFrictionCurve(constraint: *const VehicleConstraint, wheel_index: u32, out: *VehicleCurveDesc) bool;

pub extern fn zjoltVehicleConstraintGetWheelLateralFrictionCurve(constraint: *const VehicleConstraint, wheel_index: u32, out: *VehicleCurveDesc) bool;

pub extern fn zjoltVehicleConstraintGetEngineNormalizedTorqueCurve(constraint: *const VehicleConstraint, out: *VehicleCurveDesc) bool;

pub extern fn zjoltVehicleConstraintGetAntiRollBarCount(constraint: *const VehicleConstraint) u32;

pub extern fn zjoltVehicleConstraintGetAntiRollBar(constraint: *const VehicleConstraint, index: u32, out: *VehicleAntiRollBarDesc) bool;

pub extern fn zjoltVehicleConstraintSetAntiRollBarStiffness(constraint: *VehicleConstraint, index: u32, stiffness: f32) Result;

pub extern fn zjoltVehicleConstraintGetDifferentialCount(constraint: *const VehicleConstraint) u32;

pub extern fn zjoltVehicleConstraintGetDifferential(constraint: *const VehicleConstraint, index: u32, out: *VehicleDifferentialDesc) bool;

pub extern fn zjoltVehicleConstraintSetDifferential(constraint: *VehicleConstraint, index: u32, desc: *const VehicleDifferentialDesc) Result;

pub extern fn zjoltVehicleConstraintGetDifferentialLimitedSlipRatio(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintSetDifferentialLimitedSlipRatio(constraint: *VehicleConstraint, ratio: f32) Result;

pub extern fn zjoltVehicleConstraintGetTrackAngularVelocity(constraint: *const VehicleConstraint, side: VehicleTrackSide) f32;

pub extern fn zjoltVehicleConstraintSetTrackAngularVelocity(constraint: *VehicleConstraint, side: VehicleTrackSide, angular_velocity: f32) Result;

pub extern fn zjoltVehicleConstraintGetTrack(constraint: *const VehicleConstraint, side: VehicleTrackSide, out: *VehicleTrackDesc) bool;

pub extern fn zjoltVehicleConstraintSetTrack(constraint: *VehicleConstraint, side: VehicleTrackSide, desc: *const VehicleTrackDesc) Result;

pub extern fn zjoltVehicleConstraintSetEngineRpm(constraint: *VehicleConstraint, rpm: f32) Result;

pub extern fn zjoltVehicleConstraintGetEngineTorque(constraint: *const VehicleConstraint, acceleration: f32) f32;

pub extern fn zjoltVehicleConstraintGetWheelSpeedAtClutch(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintSetRpmMeter(constraint: *VehicleConstraint, position: *const Vec3, size: f32) Result;

pub extern fn zjoltVehicleConstraintGetMotorcycleLeanSpring(constraint: *const VehicleConstraint, out: *VehicleMotorcycleLeanSpring) bool;

pub extern fn zjoltVehicleConstraintSetMotorcycleLeanSpring(constraint: *VehicleConstraint, spring: *const VehicleMotorcycleLeanSpring) Result;

pub extern fn zjoltVehicleConstraintSetWheelFilters(constraint: *VehicleConstraint, filters: ?*const VehicleWheelFilters) Result;

pub extern fn zjoltVehicleConstraintGetWheelFilters(constraint: *const VehicleConstraint, out: *VehicleWheelFilters) void;

pub extern fn zjoltVehicleConstraintSetCollisionTester(constraint: *VehicleConstraint, desc: *const VehicleCollisionTesterDesc) Result;

pub extern fn zjoltVehicleConstraintSetCollisionTesterCallback(constraint: *VehicleConstraint, callback: *const VehicleCollisionTesterCallback) Result;

pub extern fn zjoltVehicleConstraintGetCollisionTesterCallback(constraint: *const VehicleConstraint, out: *VehicleCollisionTesterCallback) void;

pub extern fn zjoltVehicleConstraintSetPreStepCallback(constraint: *VehicleConstraint, callback: ?*const VehicleStepCallback) Result;

pub extern fn zjoltVehicleConstraintGetPreStepCallback(constraint: *const VehicleConstraint, out: *VehicleStepCallback) void;

pub extern fn zjoltVehicleConstraintSetPostCollideCallback(constraint: *VehicleConstraint, callback: ?*const VehicleStepCallback) Result;

pub extern fn zjoltVehicleConstraintGetPostCollideCallback(constraint: *const VehicleConstraint, out: *VehicleStepCallback) void;

pub extern fn zjoltVehicleConstraintSetPostStepCallback(constraint: *VehicleConstraint, callback: ?*const VehicleStepCallback) Result;

pub extern fn zjoltVehicleConstraintGetPostStepCallback(constraint: *const VehicleConstraint, out: *VehicleStepCallback) void;

pub extern fn zjoltVehicleConstraintSetCombineFrictionCallback(constraint: *VehicleConstraint, callback: ?*const VehicleCombineFrictionCallback) Result;

pub extern fn zjoltVehicleConstraintGetCombineFrictionCallback(constraint: *const VehicleConstraint, out: *VehicleCombineFrictionCallback) void;

pub extern fn zjoltVehicleConstraintSetTireMaxImpulseCallback(constraint: *VehicleConstraint, callback: ?*const VehicleTireMaxImpulseCallback) Result;

pub extern fn zjoltVehicleConstraintGetTireMaxImpulseCallback(constraint: *const VehicleConstraint, out: *VehicleTireMaxImpulseCallback) void;

pub extern fn zjoltVehicleConstraintAsConstraint(constraint: *const VehicleConstraint) ?*Constraint;
