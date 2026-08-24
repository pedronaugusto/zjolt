//! Constraints: the joints between two bodies.
//!
//! Twelve kinds, one handle. `Constraint` is a thin owner over the C handle;
//! which kind it is comes from `subType`, and every kind-specific accessor
//! checks it before doing anything, so calling `hingeGetCurrentAngle` on a
//! slider is `error.InvalidArgument` rather than a reinterpreted object.
//!
//! ## Ownership
//!
//! A constraint is reference counted. `init*` hands back one reference that is
//! yours — `deinit` (or `release`, which is the same thing spelled the way
//! `Shape` spells it) drops it. Adding one to a system takes a second
//! reference that the system owns, so these two are independent:
//!
//! ```zig
//! var hinge = try zjolt.Constraint.initHinge(system, door, frame, .{
//!     .point1 = zjolt.rvec3(0, 1, 0),
//!     .hinge_axis1 = zjolt.vec3(0, 1, 0),
//!     .normal_axis1 = zjolt.vec3(1, 0, 0),
//!     .hinge_axis2 = zjolt.vec3(0, 1, 0),
//!     .normal_axis2 = zjolt.vec3(1, 0, 0),
//!     .limits_min = -std.math.pi / 2.0,
//!     .limits_max = 0,
//! });
//! defer hinge.deinit();
//! try hinge.addTo(system);
//! ```
//!
//! ## Against the bodies
//!
//! A constraint holds raw pointers to its bodies, as Jolt does. Destroying a
//! body a live constraint still names leaves it pointing at freed memory, and
//! nothing detects it. Remove and release a body's constraints first.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const system_mod = @import("system.zig");

/// The body id that stands for "the world": an implicit, infinitely heavy
/// static body at the origin. Pass it as either body to bolt the other one
/// down — but not as both.
pub const world_body: body_mod.BodyId = c.body_id_world;

pub const SubType = c.ConstraintSubType;
pub const Space = c.ConstraintSpace;
pub const MotorState = c.MotorState;
pub const SpringMode = c.SpringMode;
pub const SwingType = c.SwingType;
pub const SixDofAxis = c.SixDofAxis;
pub const PathRotationConstraintType = c.PathRotationConstraintType;

/// A linear or angular spring. All-zero is a HARD limit, which is the default
/// and usually what is wanted.
pub const SpringSettings = c.SpringSettings;

/// What a powered joint may apply. The default is unlimited in both
/// directions; a zeroed one can apply nothing at all.
pub const MotorSettings = c.MotorSettings;

pub const FixedDesc = c.FixedConstraintDesc;
pub const PointDesc = c.PointConstraintDesc;
pub const HingeDesc = c.HingeConstraintDesc;
pub const SliderDesc = c.SliderConstraintDesc;
pub const DistanceDesc = c.DistanceConstraintDesc;
pub const ConeDesc = c.ConeConstraintDesc;
pub const SwingTwistDesc = c.SwingTwistConstraintDesc;
pub const SixDofDesc = c.SixDofConstraintDesc;
pub const GearDesc = c.GearConstraintDesc;
pub const RackAndPinionDesc = c.RackAndPinionConstraintDesc;
pub const PulleyDesc = c.PulleyConstraintDesc;
pub const PathDesc = c.PathConstraintDesc;

/// One control point of a cubic Hermite spline.
pub const PathPoint = c.PathPoint;

/// The frame at one point along a path.
pub const PathFrame = struct {
    position: math.Vec3,
    tangent: math.Vec3,
    normal: math.Vec3,
    binormal: math.Vec3,
};

//=============================================================================
// Paths
//=============================================================================

/// A curve a path constraint follows. Reference counted and shareable — one
/// track, many carts.
pub const Path = struct {
    handle: *c.PathConstraintPath,

    /// Builds a cubic Hermite spline through `points`.
    ///
    /// At least two points, and for a looping path the first and last must
    /// differ: the loop is closed by the implied final segment, not by
    /// repeating the point. Each point's tangent and normal must be non-zero
    /// and not parallel to each other — they are what the frame is built from.
    pub fn initHermite(points: []const PathPoint, is_looping: bool) err.Error!Path {
        var handle: *c.PathConstraintPath = undefined;
        try err.check(c.zjoltPathConstraintPathCreateHermite(
            points.ptr,
            @intCast(points.len),
            is_looping,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn addRef(self: Path) void {
        c.zjoltPathConstraintPathAddRef(self.handle);
    }

    pub fn release(self: Path) void {
        c.zjoltPathConstraintPathRelease(self.handle);
    }

    /// Alias for `release`, so a path reads like every other handle at a
    /// `defer`.
    pub fn deinit(self: Path) void {
        self.release();
    }

    pub fn refCount(self: Path) u32 {
        return c.zjoltPathConstraintPathGetRefCount(self.handle);
    }

    pub fn isLooping(self: Path) bool {
        return c.zjoltPathConstraintPathIsLooping(self.handle);
    }

    /// The largest fraction naming a point on this path. A fraction is a
    /// control-point index plus a remainder, not a 0-to-1 parameter.
    pub fn maxFraction(self: Path) f32 {
        return c.zjoltPathConstraintPathGetMaxFraction(self.handle);
    }

    /// The fraction of the point closest to `position`. `hint` seeds the
    /// search — the current fraction when tracking, 0 when placing.
    pub fn closestPoint(self: Path, position: math.Vec3, hint: f32) err.Error!f32 {
        var fraction: f32 = 0;
        try err.check(c.zjoltPathConstraintPathGetClosestPoint(
            self.handle,
            &position,
            hint,
            &fraction,
        ));
        return fraction;
    }

    pub fn pointOnPath(self: Path, fraction: f32) err.Error!PathFrame {
        var frame: PathFrame = undefined;
        try err.check(c.zjoltPathConstraintPathGetPointOnPath(
            self.handle,
            fraction,
            &frame.position,
            &frame.tangent,
            &frame.normal,
            &frame.binormal,
        ));
        return frame;
    }
};

//=============================================================================
// The constraint
//=============================================================================

pub const Constraint = struct {
    handle: *c.Constraint,

    //-------------------------------------------------------------------------
    // Construction
    //
    // Each takes the system the bodies live in, because that is where a body
    // id resolves. Creating does NOT add: `addTo` is what starts it
    // simulating.
    //-------------------------------------------------------------------------

    /// Welds two bodies together, removing all six degrees of freedom.
    pub fn initFixed(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: FixedDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateFixed, system, body1, body2, desc);
    }

    /// Pins two bodies at one point, leaving rotation free. A ball joint.
    pub fn initPoint(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: PointDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreatePoint, system, body1, body2, desc);
    }

    /// One point and one axis of rotation. A door, an elbow, a wheel.
    pub fn initHinge(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: HingeDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateHinge, system, body1, body2, desc);
    }

    /// Movement along one axis and nothing else. A piston.
    ///
    /// Jolt solves the rotation part from body 1, so where the two differ in
    /// mass, body 1 should be the heavier.
    pub fn initSlider(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: SliderDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateSlider, system, body1, body2, desc);
    }

    /// Keeps two points a fixed distance — or a range — apart. A rope.
    pub fn initDistance(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: DistanceDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateDistance, system, body1, body2, desc);
    }

    /// A point, plus a limit on how far the two twist axes may open apart.
    pub fn initCone(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: ConeDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateCone, system, body1, body2, desc);
    }

    /// A ball joint with separate swing and twist limits and motors. What a
    /// ragdoll's shoulders are made of.
    pub fn initSwingTwist(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: SwingTwistDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateSwingTwist, system, body1, body2, desc);
    }

    /// Every degree of freedom controlled separately. The general joint.
    pub fn initSixDof(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: SixDofDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateSixDof, system, body1, body2, desc);
    }

    /// Ties the rotation of two bodies together as gear teeth do.
    pub fn initGear(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: GearDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateGear, system, body1, body2, desc);
    }

    /// Ties the rotation of body 1 (the pinion) to the translation of body 2
    /// (the rack).
    pub fn initRackAndPinion(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: RackAndPinionDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreateRackAndPinion, system, body1, body2, desc);
    }

    /// Two bodies on a rope over two fixed points.
    pub fn initPulley(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: PulleyDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreatePulley, system, body1, body2, desc);
    }

    /// Constrains body 2 to slide along a path attached to body 1.
    pub fn initPath(
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: PathDesc,
    ) err.Error!Constraint {
        return build(c.zjoltConstraintCreatePath, system, body1, body2, desc);
    }

    fn build(
        comptime create: anytype,
        system: system_mod.PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        desc: anytype,
    ) err.Error!Constraint {
        var handle: *c.Constraint = undefined;
        var raw = desc;
        try err.check(create(system.handle, body1, body2, &raw, &handle));
        return .{ .handle = handle };
    }

    //-------------------------------------------------------------------------
    // Reference counting
    //-------------------------------------------------------------------------

    pub fn addRef(self: Constraint) void {
        c.zjoltConstraintAddRef(self.handle);
    }

    pub fn release(self: Constraint) void {
        c.zjoltConstraintRelease(self.handle);
    }

    /// Alias for `release`. A constraint still in a system survives this: the
    /// system holds its own reference.
    pub fn deinit(self: Constraint) void {
        self.release();
    }

    pub fn refCount(self: Constraint) u32 {
        return c.zjoltConstraintGetRefCount(self.handle);
    }

    //-------------------------------------------------------------------------
    // Membership of a system
    //-------------------------------------------------------------------------

    /// Starts simulating this constraint, and takes a reference of its own.
    ///
    /// `error.InvalidArgument` when it is already in a system, or when its
    /// bodies belong to a different one. Both would be aborts inside Jolt.
    pub fn addTo(self: Constraint, system: system_mod.PhysicsSystem) err.Error!void {
        try err.check(c.zjoltConstraintAdd(system.handle, self.handle));
    }

    /// Stops simulating it and drops the system's reference.
    pub fn removeFrom(self: Constraint, system: system_mod.PhysicsSystem) err.Error!void {
        try err.check(c.zjoltConstraintRemove(system.handle, self.handle));
    }

    /// Whether it is currently in `system`. O(number of constraints): Jolt
    /// exposes membership only as a copy of the whole list.
    pub fn isAddedTo(self: Constraint, system: system_mod.PhysicsSystem) bool {
        return c.zjoltConstraintIsAdded(system.handle, self.handle);
    }

    //-------------------------------------------------------------------------
    // Common state
    //-------------------------------------------------------------------------

    pub fn subType(self: Constraint) SubType {
        return c.zjoltConstraintGetSubType(self.handle);
    }

    /// A disabled constraint stays in the system and applies nothing. This is
    /// how a joint is broken, and how it is repaired.
    pub fn setEnabled(self: Constraint, enabled: bool) void {
        c.zjoltConstraintSetEnabled(self.handle, enabled);
    }

    pub fn isEnabled(self: Constraint) bool {
        return c.zjoltConstraintIsEnabled(self.handle);
    }

    /// Whether the solver will touch it this step: enabled, with at least one
    /// body awake and dynamic.
    pub fn isActive(self: Constraint) bool {
        return c.zjoltConstraintIsActive(self.handle);
    }

    pub fn setUserData(self: Constraint, user_data: u64) void {
        c.zjoltConstraintSetUserData(self.handle, user_data);
    }

    pub fn getUserData(self: Constraint) u64 {
        return c.zjoltConstraintGetUserData(self.handle);
    }

    /// Higher priority is solved later, which makes it win.
    pub fn setPriority(self: Constraint, priority: u32) void {
        c.zjoltConstraintSetPriority(self.handle, priority);
    }

    pub fn getPriority(self: Constraint) u32 {
        return c.zjoltConstraintGetPriority(self.handle);
    }

    /// Solver iterations for this constraint alone; 0 uses the system's. Must
    /// be under 256 — Jolt keeps it in a byte and asserts.
    pub fn setNumVelocityStepsOverride(self: Constraint, steps: u32) err.Error!void {
        try err.check(c.zjoltConstraintSetNumVelocityStepsOverride(self.handle, steps));
    }

    pub fn getNumVelocityStepsOverride(self: Constraint) u32 {
        return c.zjoltConstraintGetNumVelocityStepsOverride(self.handle);
    }

    pub fn setNumPositionStepsOverride(self: Constraint, steps: u32) err.Error!void {
        try err.check(c.zjoltConstraintSetNumPositionStepsOverride(self.handle, steps));
    }

    pub fn getNumPositionStepsOverride(self: Constraint) u32 {
        return c.zjoltConstraintGetNumPositionStepsOverride(self.handle);
    }

    /// The two bodies, in the order they were given. `world_body` comes back
    /// for the world.
    pub fn bodies(self: Constraint) err.Error![2]body_mod.BodyId {
        var out: [2]body_mod.BodyId = undefined;
        try err.check(c.zjoltConstraintGetBodies(self.handle, &out[0], &out[1]));
        return out;
    }

    /// Throws away the impulse carried over from last step. Do this after
    /// teleporting a body, so the solver does not push against a change it did
    /// not make.
    pub fn resetWarmStart(self: Constraint) void {
        c.zjoltConstraintResetWarmStart(self.handle);
    }
};

/// How many constraints are currently added to `system`.
pub fn count(system: system_mod.PhysicsSystem) u32 {
    return c.zjoltPhysicsSystemGetNumConstraints(system.handle);
}
