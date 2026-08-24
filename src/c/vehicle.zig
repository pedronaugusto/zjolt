//! ZJolt C declarations for wheeled and tracked vehicles and their drivetrains.
//!
//! Mirrors `ffi/zjolt_vehicle.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const constraint = @import("constraint.zig");
const core = @import("core.zig");

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

pub const VehicleControllerKind = enum(c_int) {
    wheeled = 0,
    tracked = 1,
    motorcycle = 2,
};

pub const VehicleCollisionTesterKind = enum(c_int) {
    ray = 0,
    cast_sphere = 1,
    cast_cylinder = 2,
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

pub const VehicleMotorcycleDesc = extern struct {
    max_lean_angle: f32,
    lean_spring_constant: f32,
    lean_spring_damping: f32,
    lean_spring_integration_coefficient: f32,
    lean_spring_integration_coefficient_decay: f32,
    lean_smoothing_factor: f32,
};

pub const VehicleConstraint = opaque {};

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

pub extern fn zjoltVehicleConstraintGetWheelLocalBasis(constraint: *const VehicleConstraint, wheel_index: u32, out_forward: *Vec3, out_up: *Vec3, out_right: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetWheelLocalTransform(constraint: *const VehicleConstraint, wheel_index: u32, wheel_right: *const Vec3, wheel_up: *const Vec3, out_position: *Vec3, out_rotation: *Quat) void;

pub extern fn zjoltVehicleConstraintGetWheelWorldTransform(constraint: *const VehicleConstraint, wheel_index: u32, wheel_right: *const Vec3, wheel_up: *const Vec3, out_position: *RVec3, out_rotation: *Quat) void;

pub extern fn zjoltVehicleConstraintAsConstraint(constraint: *const VehicleConstraint) ?*Constraint;
