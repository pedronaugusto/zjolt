//! Wheeled and tracked vehicles.
//!
//! A vehicle is one constraint tying a body to a set of wheels and one
//! controller. Which controller it has is fixed at `init` and reported by
//! `controllerKind`; the wheeled/motorcycle-only and tracked-only methods
//! below return `error.InvalidArgument` against the wrong kind rather than
//! reinterpreting memory. A motorcycle IS a wheeled controller plus a lean
//! spring, so every wheeled method also accepts `.motorcycle`.
//!
//! As everywhere in this wrapper, `WheelDesc`/`EngineDesc`/... are the C
//! descriptors re-exported, not copies — build one from `defaultWheelDesc()`
//! and friends, then override the fields that matter.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const system_mod = @import("system.zig");
const constraint_mod = @import("constraint.zig");

pub const ControllerKind = c.VehicleControllerKind;
pub const CollisionTesterKind = c.VehicleCollisionTesterKind;
pub const TransmissionMode = c.VehicleTransmissionMode;

pub const curve_max_points = c.vehicle_curve_max_points;
pub const CurvePoint = c.VehicleCurvePoint;
pub const CurveDesc = c.VehicleCurveDesc;

/// A curve with no points: leaves whichever Jolt default the owning settings
/// object already carries in place. See `ZJoltVehicleCurveDesc` in the header.
pub const default_curve: CurveDesc = .{ .points = undefined, .count = 0 };

pub const WheelDesc = c.VehicleWheelDesc;
pub const EngineDesc = c.VehicleEngineDesc;
pub const TransmissionDesc = c.VehicleTransmissionDesc;
pub const DifferentialDesc = c.VehicleDifferentialDesc;
pub const AntiRollBarDesc = c.VehicleAntiRollBarDesc;
pub const CollisionTesterDesc = c.VehicleCollisionTesterDesc;
pub const MotorcycleDesc = c.VehicleMotorcycleDesc;

pub fn defaultWheelDesc() WheelDesc {
    var desc: WheelDesc = undefined;
    c.zjoltVehicleWheelDescInit(&desc);
    return desc;
}

pub fn defaultEngineDesc() EngineDesc {
    var desc: EngineDesc = undefined;
    c.zjoltVehicleEngineDescInit(&desc);
    return desc;
}

pub fn defaultTransmissionDesc() TransmissionDesc {
    var desc: TransmissionDesc = undefined;
    c.zjoltVehicleTransmissionDescInit(&desc);
    return desc;
}

pub fn defaultDifferentialDesc() DifferentialDesc {
    var desc: DifferentialDesc = undefined;
    c.zjoltVehicleDifferentialDescInit(&desc);
    return desc;
}

pub fn defaultAntiRollBarDesc() AntiRollBarDesc {
    var desc: AntiRollBarDesc = undefined;
    c.zjoltVehicleAntiRollBarDescInit(&desc);
    return desc;
}

pub fn defaultCollisionTesterDesc() CollisionTesterDesc {
    var desc: CollisionTesterDesc = undefined;
    c.zjoltVehicleCollisionTesterDescInit(&desc);
    return desc;
}

pub fn defaultMotorcycleDesc() MotorcycleDesc {
    var desc: MotorcycleDesc = undefined;
    c.zjoltVehicleMotorcycleDescInit(&desc);
    return desc;
}

pub const VehicleConstraint = struct {
    handle: *c.VehicleConstraint,

    pub const Options = struct {
        /// Local-space up/forward for the vehicle body. Normalised by the C
        /// side if not already unit length.
        up: math.Vec3 = .{ .x = 0, .y = 1, .z = 0 },
        forward: math.Vec3 = .{ .x = 0, .y = 0, .z = 1 },
        /// Radians; pi (the default) disables the limit.
        max_pitch_roll_angle: f32 = std.math.pi,

        /// Required: at least one wheel. Borrowed for the duration of `init`.
        wheels: []const WheelDesc,
        /// Borrowed for the duration of `init`.
        anti_roll_bars: []const AntiRollBarDesc = &.{},

        controller_kind: ControllerKind = .wheeled,
        engine: EngineDesc = .{
            .max_torque = 500,
            .min_rpm = 1000,
            .max_rpm = 6000,
            .normalized_torque = default_curve,
            .inertia = 0.5,
            .angular_damping = 0.2,
        },
        transmission: TransmissionDesc = .{
            .mode = .auto,
            .forward_gear_ratios = null,
            .forward_gear_ratio_count = 0,
            .reverse_gear_ratios = null,
            .reverse_gear_ratio_count = 0,
            .switch_time = 0.5,
            .clutch_release_time = 0.3,
            .switch_latency = 0.5,
            .shift_up_rpm = 4000,
            .shift_down_rpm = 2000,
            .clutch_strength = 10,
        },

        /// Wheeled and motorcycle only. Borrowed for the duration of `init`.
        differentials: []const DifferentialDesc = &.{},
        /// Wheeled and motorcycle only.
        differential_limited_slip_ratio: f32 = 1.4,

        /// Tracked only. Indices into `wheels`, borrowed for `init`.
        left_track_wheels: []const u32 = &.{},
        left_track_driven_wheel: u32 = 0,
        right_track_wheels: []const u32 = &.{},
        right_track_driven_wheel: u32 = 0,
        track_inertia: f32 = 10,
        track_angular_damping: f32 = 0.5,
        track_max_brake_torque: f32 = 15000,
        track_differential_ratio: f32 = 6,

        /// Motorcycle only (`controller_kind == .motorcycle`).
        motorcycle: MotorcycleDesc = .{
            .max_lean_angle = std.math.degreesToRadians(45.0),
            .lean_spring_constant = 5000,
            .lean_spring_damping = 1000,
            .lean_spring_integration_coefficient = 0,
            .lean_spring_integration_coefficient_decay = 4,
            .lean_smoothing_factor = 0.8,
        },

        collision_tester: CollisionTesterDesc,
    };

    /// Attaches a vehicle constraint to `body`, which must already be added
    /// to `system`. The constraint keeps a raw pointer to that body for its
    /// own lifetime — `deinit` the vehicle before destroying the body.
    pub fn init(
        system: system_mod.PhysicsSystem,
        body: body_mod.BodyId,
        opts: Options,
    ) err.Error!VehicleConstraint {
        var desc: c.VehicleConstraintDesc = undefined;
        c.zjoltVehicleConstraintDescInit(&desc);
        desc.up = opts.up;
        desc.forward = opts.forward;
        desc.max_pitch_roll_angle = opts.max_pitch_roll_angle;
        desc.wheels = opts.wheels.ptr;
        desc.wheel_count = @intCast(opts.wheels.len);
        desc.anti_roll_bars = if (opts.anti_roll_bars.len > 0) opts.anti_roll_bars.ptr else null;
        desc.anti_roll_bar_count = @intCast(opts.anti_roll_bars.len);
        desc.controller_kind = opts.controller_kind;
        desc.engine = opts.engine;
        desc.transmission = opts.transmission;
        desc.differentials = if (opts.differentials.len > 0) opts.differentials.ptr else null;
        desc.differential_count = @intCast(opts.differentials.len);
        desc.differential_limited_slip_ratio = opts.differential_limited_slip_ratio;
        desc.left_track_wheels = if (opts.left_track_wheels.len > 0) opts.left_track_wheels.ptr else null;
        desc.left_track_wheel_count = @intCast(opts.left_track_wheels.len);
        desc.left_track_driven_wheel = opts.left_track_driven_wheel;
        desc.right_track_wheels = if (opts.right_track_wheels.len > 0) opts.right_track_wheels.ptr else null;
        desc.right_track_wheel_count = @intCast(opts.right_track_wheels.len);
        desc.right_track_driven_wheel = opts.right_track_driven_wheel;
        desc.track_inertia = opts.track_inertia;
        desc.track_angular_damping = opts.track_angular_damping;
        desc.track_max_brake_torque = opts.track_max_brake_torque;
        desc.track_differential_ratio = opts.track_differential_ratio;
        desc.motorcycle = opts.motorcycle;
        desc.collision_tester = opts.collision_tester;

        var handle: *c.VehicleConstraint = undefined;
        try err.check(c.zjoltVehicleConstraintCreate(system.handle, body, &desc, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: VehicleConstraint) void {
        c.zjoltVehicleConstraintDestroy(self.handle);
    }

    //=========================================================================
    // Vehicle-level state
    //=========================================================================

    pub fn controllerKind(self: VehicleConstraint) ControllerKind {
        return c.zjoltVehicleConstraintGetControllerKind(self.handle);
    }

    pub fn collisionTesterKind(self: VehicleConstraint) CollisionTesterKind {
        return c.zjoltVehicleConstraintGetCollisionTesterKind(self.handle);
    }

    pub fn bodyId(self: VehicleConstraint) body_mod.BodyId {
        return c.zjoltVehicleConstraintGetBodyId(self.handle);
    }

    /// True when the underlying body is in the broad phase and the
    /// constraint has not idled out.
    pub fn isActive(self: VehicleConstraint) bool {
        return c.zjoltVehicleConstraintIsActive(self.handle);
    }

    pub fn maxPitchRollAngle(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetMaxPitchRollAngle(self.handle);
    }

    pub fn setMaxPitchRollAngle(self: VehicleConstraint, radians: f32) void {
        c.zjoltVehicleConstraintSetMaxPitchRollAngle(self.handle, radians);
    }

    pub fn localUp(self: VehicleConstraint) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltVehicleConstraintGetLocalUp(self.handle, &out);
        return out;
    }

    pub fn localForward(self: VehicleConstraint) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltVehicleConstraintGetLocalForward(self.handle, &out);
        return out;
    }

    /// World-space up used this step to judge pitch/roll; tracks inverted
    /// gravity.
    pub fn worldUp(self: VehicleConstraint) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltVehicleConstraintGetWorldUp(self.handle, &out);
        return out;
    }

    //=========================================================================
    // Wheels — runtime state
    //
    // Wheel settings (radius, suspension range, ...) are creation-time only,
    // via WheelDesc; there is no live getter for them. Every accessor below
    // returns a zero/false default for a wheel_index at or past wheelCount().
    //=========================================================================

    pub fn wheelCount(self: VehicleConstraint) u32 {
        return c.zjoltVehicleConstraintGetWheelCount(self.handle);
    }

    pub fn wheelRotationAngle(self: VehicleConstraint, wheel_index: u32) f32 {
        return c.zjoltVehicleConstraintGetWheelRotationAngle(self.handle, wheel_index);
    }

    pub fn setWheelRotationAngle(self: VehicleConstraint, wheel_index: u32, angle: f32) void {
        c.zjoltVehicleConstraintSetWheelRotationAngle(self.handle, wheel_index, angle);
    }

    pub fn wheelSteerAngle(self: VehicleConstraint, wheel_index: u32) f32 {
        return c.zjoltVehicleConstraintGetWheelSteerAngle(self.handle, wheel_index);
    }

    pub fn setWheelSteerAngle(self: VehicleConstraint, wheel_index: u32, angle: f32) void {
        c.zjoltVehicleConstraintSetWheelSteerAngle(self.handle, wheel_index, angle);
    }

    pub fn wheelAngularVelocity(self: VehicleConstraint, wheel_index: u32) f32 {
        return c.zjoltVehicleConstraintGetWheelAngularVelocity(self.handle, wheel_index);
    }

    pub fn setWheelAngularVelocity(self: VehicleConstraint, wheel_index: u32, velocity: f32) void {
        c.zjoltVehicleConstraintSetWheelAngularVelocity(self.handle, wheel_index, velocity);
    }

    pub fn wheelSuspensionLength(self: VehicleConstraint, wheel_index: u32) f32 {
        return c.zjoltVehicleConstraintGetWheelSuspensionLength(self.handle, wheel_index);
    }

    pub fn wheelHasContact(self: VehicleConstraint, wheel_index: u32) bool {
        return c.zjoltVehicleConstraintHasWheelContact(self.handle, wheel_index);
    }

    /// The body the wheel is touching, or `invalid_body_id` without contact.
    pub fn wheelContactBodyId(self: VehicleConstraint, wheel_index: u32) body_mod.BodyId {
        return c.zjoltVehicleConstraintGetWheelContactBodyId(self.handle, wheel_index);
    }

    pub fn wheelContactPosition(self: VehicleConstraint, wheel_index: u32) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltVehicleConstraintGetWheelContactPosition(self.handle, wheel_index, &out);
        return out;
    }

    pub fn wheelContactNormal(self: VehicleConstraint, wheel_index: u32) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltVehicleConstraintGetWheelContactNormal(self.handle, wheel_index, &out);
        return out;
    }

    pub fn wheelContactLongitudinal(self: VehicleConstraint, wheel_index: u32) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltVehicleConstraintGetWheelContactLongitudinal(self.handle, wheel_index, &out);
        return out;
    }

    pub fn wheelContactLateral(self: VehicleConstraint, wheel_index: u32) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltVehicleConstraintGetWheelContactLateral(self.handle, wheel_index, &out);
        return out;
    }

    pub fn wheelContactPointVelocity(self: VehicleConstraint, wheel_index: u32) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltVehicleConstraintGetWheelContactPointVelocity(self.handle, wheel_index, &out);
        return out;
    }

    //=========================================================================
    // Driver input and drivetrain readback — every controller kind
    //
    // forward/brake, engine RPM, current gear, clutch friction and gear
    // ratio mean the same thing whichever controller this constraint has,
    // so each of these is one method rather than a wheeled/tracked pair a
    // caller could pick the wrong one of for the kind it actually has.
    //=========================================================================

    pub fn forwardInput(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetForwardInput(self.handle);
    }

    pub fn brakeInput(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetBrakeInput(self.handle);
    }

    pub fn engineRpm(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetEngineRpm(self.handle);
    }

    /// -1 = reverse, 0 = neutral, 1 = 1st gear, etc.
    pub fn currentGear(self: VehicleConstraint) i32 {
        return c.zjoltVehicleConstraintGetCurrentGear(self.handle);
    }

    pub fn isSwitchingGear(self: VehicleConstraint) bool {
        return c.zjoltVehicleConstraintIsSwitchingGear(self.handle);
    }

    pub fn clutchFriction(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetClutchFriction(self.handle);
    }

    /// Current gear ratio times the differential ratio; 0 in neutral.
    pub fn gearRatio(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetGearRatio(self.handle);
    }

    /// Drives a manual transmission (`.transmission.mode = .manual`): sets
    /// the current gear and the clutch's friction fraction directly,
    /// bypassing auto mode's shift points and timers entirely.
    /// @param gear -1 = reverse, 0 = neutral, 1 = 1st gear, etc.
    /// @param clutch_friction [0, 1]: 0 = fully disengaged, 1 = fully engaged.
    pub fn setGear(self: VehicleConstraint, gear: i32, clutch_friction: f32) err.Error!void {
        try err.check(c.zjoltVehicleConstraintSetGear(self.handle, gear, clutch_friction));
    }

    //=========================================================================
    // Wheeled and motorcycle controller
    //
    // Also valid against `.motorcycle` — see the module doc comment.
    //=========================================================================

    /// @param forward [-1, 1] auto, [0, 1] manual: desired direction and throttle.
    /// @param right [-1, 1]: desired steering angle, 1 = right.
    /// @param brake [0, 1].
    /// @param hand_brake [0, 1]; usually only the rear wheels honour it.
    pub fn setWheeledDriverInput(
        self: VehicleConstraint,
        forward: f32,
        right: f32,
        brake: f32,
        hand_brake: f32,
    ) err.Error!void {
        try err.check(c.zjoltVehicleConstraintSetWheeledDriverInput(
            self.handle,
            forward,
            right,
            brake,
            hand_brake,
        ));
    }

    pub fn wheeledRightInput(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetWheeledRightInput(self.handle);
    }

    pub fn wheeledHandBrakeInput(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetWheeledHandBrakeInput(self.handle);
    }

    //=========================================================================
    // Tracked controller
    //=========================================================================

    /// @param forward [-1, 1] auto, [0, 1] manual.
    /// @param left_ratio Nonzero multiplier on the left track's rate; steers.
    /// @param right_ratio Nonzero multiplier on the right track's rate; steers.
    /// @param brake [0, 1].
    pub fn setTrackedDriverInput(
        self: VehicleConstraint,
        forward: f32,
        left_ratio: f32,
        right_ratio: f32,
        brake: f32,
    ) err.Error!void {
        try err.check(c.zjoltVehicleConstraintSetTrackedDriverInput(
            self.handle,
            forward,
            left_ratio,
            right_ratio,
            brake,
        ));
    }

    pub fn trackedLeftRatio(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetTrackedLeftRatio(self.handle);
    }

    pub fn trackedRightRatio(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetTrackedRightRatio(self.handle);
    }

    //=========================================================================
    // Motorcycle controller
    //=========================================================================

    /// Disabling lets the motorcycle fall over instead of self-balancing.
    pub fn setMotorcycleLeanControllerEnabled(
        self: VehicleConstraint,
        enabled: bool,
    ) err.Error!void {
        try err.check(c.zjoltVehicleConstraintSetMotorcycleLeanControllerEnabled(
            self.handle,
            enabled,
        ));
    }

    pub fn isMotorcycleLeanControllerEnabled(self: VehicleConstraint) bool {
        return c.zjoltVehicleConstraintIsMotorcycleLeanControllerEnabled(self.handle);
    }

    /// Caps how far the steering angle can go as the motorcycle leans, so it
    /// cannot steer into itself. Disabling makes it steer like a car with
    /// two close-together front wheels instead of leaning into turns.
    pub fn setMotorcycleLeanSteeringLimitEnabled(
        self: VehicleConstraint,
        enabled: bool,
    ) err.Error!void {
        try err.check(c.zjoltVehicleConstraintSetMotorcycleLeanSteeringLimitEnabled(
            self.handle,
            enabled,
        ));
    }

    pub fn isMotorcycleLeanSteeringLimitEnabled(self: VehicleConstraint) bool {
        return c.zjoltVehicleConstraintIsMotorcycleLeanSteeringLimitEnabled(self.handle);
    }

    /// Distance between the front and rear wheel's ground contact; 0
    /// against a non-motorcycle constraint.
    pub fn motorcycleWheelBase(self: VehicleConstraint) f32 {
        return c.zjoltVehicleConstraintGetMotorcycleWheelBase(self.handle);
    }

    //=========================================================================
    // Gravity override
    //
    // Replaces the physics system's own gravity for this vehicle only.
    //=========================================================================

    pub fn overrideGravity(self: VehicleConstraint, gravity: math.Vec3) void {
        c.zjoltVehicleConstraintOverrideGravity(self.handle, &gravity);
    }

    /// Also restores the vehicle body's gravity factor to 1.
    pub fn resetGravityOverride(self: VehicleConstraint) void {
        c.zjoltVehicleConstraintResetGravityOverride(self.handle);
    }

    pub fn isGravityOverridden(self: VehicleConstraint) bool {
        return c.zjoltVehicleConstraintIsGravityOverridden(self.handle);
    }

    pub fn gravityOverride(self: VehicleConstraint) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltVehicleConstraintGetGravityOverride(self.handle, &out);
        return out;
    }

    //=========================================================================
    // Wheel-ground collision test frequency
    //
    // Skipping steps between wheel-ground collision tests is a cheap way to
    // spend less time on vehicles far from the camera or barely moving; the
    // wheels keep their last contact between tests.
    //=========================================================================

    pub fn numStepsBetweenCollisionTestActive(self: VehicleConstraint) u32 {
        return c.zjoltVehicleConstraintGetNumStepsBetweenCollisionTestActive(self.handle);
    }

    pub fn setNumStepsBetweenCollisionTestActive(self: VehicleConstraint, steps: u32) void {
        c.zjoltVehicleConstraintSetNumStepsBetweenCollisionTestActive(self.handle, steps);
    }

    pub fn numStepsBetweenCollisionTestInactive(self: VehicleConstraint) u32 {
        return c.zjoltVehicleConstraintGetNumStepsBetweenCollisionTestInactive(self.handle);
    }

    pub fn setNumStepsBetweenCollisionTestInactive(self: VehicleConstraint, steps: u32) void {
        c.zjoltVehicleConstraintSetNumStepsBetweenCollisionTestInactive(self.handle, steps);
    }

    //=========================================================================
    // Wheel force and pose readback
    //
    // The lambdas are the solver's own accumulated impulses for the wheel
    // this step, divided by delta time to read as a force.
    //=========================================================================

    pub fn wheelContactSubShapeId(self: VehicleConstraint, wheel_index: u32) c.SubShapeId {
        return c.zjoltVehicleConstraintGetWheelContactSubShapeId(self.handle, wheel_index);
    }

    /// True when the suspension has bottomed out against its hard limit
    /// this step.
    pub fn wheelHasHitHardPoint(self: VehicleConstraint, wheel_index: u32) bool {
        return c.zjoltVehicleConstraintHasWheelHitHardPoint(self.handle, wheel_index);
    }

    pub fn wheelSuspensionLambda(self: VehicleConstraint, wheel_index: u32) f32 {
        return c.zjoltVehicleConstraintGetWheelSuspensionLambda(self.handle, wheel_index);
    }

    pub fn wheelLongitudinalLambda(self: VehicleConstraint, wheel_index: u32) f32 {
        return c.zjoltVehicleConstraintGetWheelLongitudinalLambda(self.handle, wheel_index);
    }

    pub fn wheelLateralLambda(self: VehicleConstraint, wheel_index: u32) f32 {
        return c.zjoltVehicleConstraintGetWheelLateralLambda(self.handle, wheel_index);
    }

    pub const WheelBasis = struct {
        forward: math.Vec3,
        up: math.Vec3,
        right: math.Vec3,
    };

    /// The wheel's forward/up/right axes in the vehicle body's local space,
    /// after steering — feed `.up`/`.right` into the two poses below to
    /// place a wheel mesh whose own local axes do not match theirs.
    pub fn wheelLocalBasis(self: VehicleConstraint, wheel_index: u32) WheelBasis {
        var out: WheelBasis = .{ .forward = math.vec3_zero, .up = math.vec3_zero, .right = math.vec3_zero };
        c.zjoltVehicleConstraintGetWheelLocalBasis(self.handle, wheel_index, &out.forward, &out.up, &out.right);
        return out;
    }

    pub const WheelPose = struct {
        position: math.Vec3,
        rotation: math.Quat,
    };

    /// The wheel's pose in the vehicle body's local space — suspension
    /// travel, steering and spin all folded in. `wheel_right`/`wheel_up`
    /// are the wheel mesh's own local axes, typically a prior
    /// `wheelLocalBasis` call's `.right`/`.up`.
    pub fn wheelLocalTransform(
        self: VehicleConstraint,
        wheel_index: u32,
        wheel_right: math.Vec3,
        wheel_up: math.Vec3,
    ) WheelPose {
        var out: WheelPose = .{ .position = math.vec3_zero, .rotation = math.quat_identity };
        c.zjoltVehicleConstraintGetWheelLocalTransform(
            self.handle,
            wheel_index,
            &wheel_right,
            &wheel_up,
            &out.position,
            &out.rotation,
        );
        return out;
    }

    pub const WheelWorldPose = struct {
        position: math.RVec3,
        rotation: math.Quat,
    };

    /// The wheel's pose in world space — what a wheel mesh should be drawn
    /// at each frame.
    pub fn wheelWorldTransform(
        self: VehicleConstraint,
        wheel_index: u32,
        wheel_right: math.Vec3,
        wheel_up: math.Vec3,
    ) WheelWorldPose {
        var out: WheelWorldPose = .{ .position = math.rvec3_zero, .rotation = math.quat_identity };
        c.zjoltVehicleConstraintGetWheelWorldTransform(
            self.handle,
            wheel_index,
            &wheel_right,
            &wheel_up,
            &out.position,
            &out.rotation,
        );
        return out;
    }

    //=========================================================================
    // Viewed as a plain constraint
    //=========================================================================

    /// A borrowed view through the generic constraint API — `setEnabled`,
    /// `subType`, `isAdded`, `setPriority` and friends on
    /// `zjolt.Constraint` all work through it. Borrowed: never `.release()`
    /// or `.deinit()` it, and it is valid only until this vehicle's own
    /// `deinit`. Null for a destroyed/never-created constraint.
    pub fn asConstraint(self: VehicleConstraint) ?constraint_mod.Constraint {
        const handle = c.zjoltVehicleConstraintAsConstraint(self.handle) orelse return null;
        return .{ .handle = handle };
    }
};
