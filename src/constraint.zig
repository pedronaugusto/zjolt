//! Constraints: the joints between two bodies.
//!
//! Twelve kinds, one handle. `Constraint` is a thin owner over the C
//! handle; which kind it is comes from `subType`, and every kind-specific
//! accessor checks it first — `hingeGetCurrentAngle` on a slider is
//! `error.InvalidArgument`, not a reinterpreted object.
//!
//! Reference counted: `init*` hands back one reference (`deinit`/`release`
//! drops it); adding one to a system takes a second, independent
//! reference the system owns.
//!
//! A constraint holds raw pointers to its bodies, as Jolt does. Destroying
//! a body a live constraint still names leaves it pointing at freed
//! memory, undetected — remove and release a body's constraints first.

const std = @import("std");
const c = @import("c/constraint.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const system_mod = @import("system.zig");
const stream_mod = @import("stream.zig");
const tree_mod = @import("tree.zig");

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

//=============================================================================
// Custom constraints
//
// The callback seam a Zig host builds its own solver on. @see
// `src/constraint_part.zig`, a port of the fourteen `ConstraintPart` types Jolt's own constraints use, meant to run in these callbacks without a crossing per Jacobian operation.
//=============================================================================

/// Body state a custom constraint's callbacks read and write. @see
/// `ffi/zjolt_constraint.h`'s `ZJoltSolverBody`.
pub const SolverBody = c.SolverBody;
pub const SolverBodyPair = c.SolverBodyPair;

/// One function pointer per Jolt solver virtual. Every one but `draw` and
/// `destroy` must be set — @see `Constraint.initCustom`.
pub const CustomCallbacks = c.CustomConstraintCallbacks;
pub const CustomDesc = c.CustomConstraintDesc;

/// What a custom constraint's `save_state` / `restore_state` callbacks talk
/// to — a live `JPH::StateRecorder&` Jolt itself is mid-call on, not a stream
/// this ABI built. Valid only for the length of that one callback.
pub const StateRecorder = c.StateRecorder;

/// Writes `data` through `recorder`. Called from inside a `save_state`
/// callback; @see `src/constraint_part.zig`'s `ConstraintPart`s for what a
/// port of Jolt's own `mTotalLambda` writes here.
pub fn stateRecorderWriteBytes(recorder: *StateRecorder, data: []const u8) void {
    if (data.len == 0) return;
    c.zjoltStateRecorderWriteBytes(recorder, data.ptr, data.len);
}

/// Reads `data.len` bytes through `recorder`. Called from inside a
/// `restore_state` callback.
pub fn stateRecorderReadBytes(recorder: *StateRecorder, data: []u8) void {
    if (data.len == 0) return;
    c.zjoltStateRecorderReadBytes(recorder, data.ptr, data.len);
}

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
// Constraint settings
//=============================================================================

/// A constraint's configuration, snapshotted from a live one
/// (`Constraint.constraintSettings`) or restored from a stream. Reference
/// counted, like `Path`. Does not record which bodies the constraint it
/// came from joins — Jolt's own note on `GetConstraintSettings`;
/// recreating an equivalent constraint from `restoreBinaryState` still
/// needs body ids from elsewhere.
pub const ConstraintSettings = struct {
    handle: *c.ConstraintSettings,

    pub fn deinit(self: ConstraintSettings) void {
        c.zjoltConstraintSettingsRelease(self.handle);
    }

    pub fn addRef(self: ConstraintSettings) void {
        c.zjoltConstraintSettingsAddRef(self.handle);
    }

    pub fn refCount(self: ConstraintSettings) u32 {
        return c.zjoltConstraintSettingsGetRefCount(self.handle);
    }

    /// The five base fields `Constraint.enabled`, `.constraintPriority`,
    /// `.numVelocityStepsOverride`, `.numPositionStepsOverride` and
    /// `.drawSize` read off a live constraint, plus `.userData` — read here
    /// too since a settings object restored from a stream has no live
    /// constraint to ask.
    pub fn enabled(self: ConstraintSettings) bool {
        return c.zjoltConstraintSettingsGetEnabled(self.handle);
    }

    pub fn constraintPriority(self: ConstraintSettings) u32 {
        return c.zjoltConstraintSettingsGetConstraintPriority(self.handle);
    }

    pub fn numVelocityStepsOverride(self: ConstraintSettings) u32 {
        return c.zjoltConstraintSettingsGetNumVelocityStepsOverride(self.handle);
    }

    pub fn numPositionStepsOverride(self: ConstraintSettings) u32 {
        return c.zjoltConstraintSettingsGetNumPositionStepsOverride(self.handle);
    }

    pub fn drawConstraintSize(self: ConstraintSettings) f32 {
        return c.zjoltConstraintSettingsGetDrawConstraintSize(self.handle);
    }

    pub fn userData(self: ConstraintSettings) u64 {
        return c.zjoltConstraintSettingsGetUserData(self.handle);
    }

    /// Writes `self` through `stream` in Jolt's own binary form. Does not
    /// write `userData`; Jolt's own `ConstraintSettings::SaveBinaryState`
    /// does not either.
    pub fn saveBinaryState(self: ConstraintSettings, stream: stream_mod.Stream) err.Error!void {
        try err.check(c.zjoltConstraintSettingsSaveBinaryState(self.handle, &stream));
    }

    /// Reads settings written by `saveBinaryState`, whichever concrete kind
    /// was saved. Release the result with `deinit`.
    pub fn restoreBinaryState(stream: stream_mod.Stream) err.Error!ConstraintSettings {
        var handle: *c.ConstraintSettings = undefined;
        try err.check(c.zjoltConstraintSettingsRestoreBinaryState(&stream, &handle));
        return .{ .handle = handle };
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
    // Each takes the system the bodies live in, where a body id resolves.
    // Creating does NOT add: `addTo` starts it simulating.
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

    /// A constraint whose solver virtuals forward to `desc.callbacks`,
    /// one crossing per virtual rather than one per Jacobian operation.
    /// `desc` carries its own two bodies, unlike every other `init*`
    /// above. `error.InvalidArgument` for a NULL required callback
    /// (anything but `draw`/`destroy`), or a step override of 256+.
    pub fn initCustom(system: system_mod.PhysicsSystem, desc: CustomDesc) err.Error!Constraint {
        var handle: *c.Constraint = undefined;
        var raw = desc;
        try err.check(c.zjoltConstraintCreateCustom(system.handle, &raw, &handle));
        return .{ .handle = handle };
    }

    //-------------------------------------------------------------------------
    // Reference counting
    //-------------------------------------------------------------------------

    pub fn addRef(self: Constraint) void {
        c.zjoltConstraintAddRef(self.handle);
    }

    /// Drops one reference. A constraint still in a system survives this —
    /// the system holds its own — so this is the right call at a `defer`
    /// beside the `init` that made it, whether or not `addTo` was called.
    pub fn release(self: Constraint) void {
        c.zjoltConstraintRelease(self.handle);
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

    /// Wakes both of its bodies — the pairwise counterpart of
    /// `BodyInterface.activate`, for when what a caller has in hand is the
    /// joint rather than either body it names.
    ///
    /// Same two refusals as `addTo`: not a two-body constraint, or its bodies
    /// do not belong to `system`.
    pub fn activate(self: Constraint, system: system_mod.PhysicsSystem) err.Error!void {
        try err.check(c.zjoltConstraintActivate(system.handle, self.handle));
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

    /// The constraint's frame: a transform from constraint space into
    /// body 1's CENTRE-OF-MASS space — compose with the body's transform
    /// to reach the world; what the `*CS` accessors (swing-twist/six-DOF
    /// motor targets) are measured against. A kind that does not
    /// constrain rotation (point, distance, pulley) has an arbitrary
    /// rotation part; gear/rack-and-pinion has a zero translation column. `error.InvalidArgument` for a vehicle constraint.
    pub fn constraintToBody1Matrix(self: Constraint) err.Error!math.Mat44 {
        var out: math.Mat44 = math.mat44_identity;
        try err.check(c.zjoltConstraintGetConstraintToBody1Matrix(self.handle, &out));
        return out;
    }

    /// As `constraintToBody1Matrix`, for body 2.
    pub fn constraintToBody2Matrix(self: Constraint) err.Error!math.Mat44 {
        var out: math.Mat44 = math.mat44_identity;
        try err.check(c.zjoltConstraintGetConstraintToBody2Matrix(self.handle, &out));
        return out;
    }

    /// How large Jolt draws this constraint through
    /// `PhysicsSystem.drawConstraints` and friends. Metres; Jolt's
    /// default is 1.
    ///
    /// `error.Unsupported` without `-Ddebug_renderer=true` — Jolt keeps the field behind that flag.
    pub fn setDrawSize(self: Constraint, size: f32) err.Error!void {
        try err.check(c.zjoltConstraintSetDrawSize(self.handle, size));
    }

    /// `error.Unsupported` without `-Ddebug_renderer=true`, as `setDrawSize`
    /// is.
    pub fn drawSize(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltConstraintGetDrawSize(self.handle, &out));
        return out;
    }

    /// Throws away the impulse carried over from last step. Do this after
    /// teleporting a body, so the solver does not push against a change it did
    /// not make.
    pub fn resetWarmStart(self: Constraint) void {
        c.zjoltConstraintResetWarmStart(self.handle);
    }

    /// Snapshots this constraint's current settings — the read-back half of
    /// authoring: create a constraint, then recover the settings object
    /// that describes it, for a level editor or a save format. Works on any
    /// kind, vehicle included (@see `vehicle.zig`'s `asConstraint`).
    /// `error.OutOfMemory` if Jolt could not build it. Release the result
    /// with `ConstraintSettings.deinit`.
    pub fn constraintSettings(self: Constraint) err.Error!ConstraintSettings {
        var handle: *c.ConstraintSettings = undefined;
        try err.check(c.zjoltConstraintGetConstraintSettings(self.handle, &handle));
        return .{ .handle = handle };
    }

    /// `Constraint::BuildIslands`, adapted for a standalone
    /// `tree.IslandBuilder`: `body1_index`/`body2_index` are the host's own
    /// active-body-list indices in place of a live `BodyManager` lookup,
    /// and activating an inactive body is the caller's job first, if at
    /// all. Links both bodies only if both are dynamic, then links
    /// `constraint_index` to whichever is — once per pair, for more than two.
    pub fn buildIslands(
        self: Constraint,
        constraint_index: u32,
        builder: tree_mod.IslandBuilder,
        body1_index: u32,
        body1_dynamic: bool,
        body2_index: u32,
        body2_dynamic: bool,
    ) err.Error!void {
        _ = self;
        if (body1_dynamic and body2_dynamic) {
            try builder.linkBodies(body1_index, body2_index);
        }
        if (body1_dynamic) {
            try builder.linkConstraint(constraint_index, body1_index);
        } else if (body2_dynamic) {
            try builder.linkConstraint(constraint_index, body2_index);
        }
    }

    /// The `user` pointer a custom constraint was created with.
    /// `error.InvalidArgument` on a constraint that is not one.
    pub fn customUserData(self: Constraint) err.Error!?*anyopaque {
        var out: ?*anyopaque = null;
        try err.check(c.zjoltConstraintGetCustomUserData(self.handle, &out));
        return out;
    }

    //-------------------------------------------------------------------------
    // Per-kind state
    //
    // Each is `error.InvalidArgument` on a constraint of a different kind
    // (`subType` says which). `totalLambda*` readings are the impulse the solver applied last step, in the constraint's own frame — what a breakable joint is built from.
    //-------------------------------------------------------------------------

    pub fn fixedTotalLambdaPosition(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltFixedConstraintGetTotalLambdaPosition(self.handle, &out));
        return out;
    }

    pub fn fixedTotalLambdaRotation(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltFixedConstraintGetTotalLambdaRotation(self.handle, &out));
        return out;
    }

    /// Moves the attachment point on body 1. The constraint keeps no record of
    /// the rotation between the bodies, so this disturbs nothing else.
    pub fn pointSetPoint1(self: Constraint, space: Space, point: math.RVec3) err.Error!void {
        try err.check(c.zjoltPointConstraintSetPoint1(self.handle, space, &point));
    }

    pub fn pointSetPoint2(self: Constraint, space: Space, point: math.RVec3) err.Error!void {
        try err.check(c.zjoltPointConstraintSetPoint2(self.handle, space, &point));
    }

    /// The attachment point in the body's centre-of-mass space, whichever
    /// space it was given in.
    pub fn pointLocalSpacePoint1(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltPointConstraintGetLocalSpacePoint1(self.handle, &out));
        return out;
    }

    pub fn pointLocalSpacePoint2(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltPointConstraintGetLocalSpacePoint2(self.handle, &out));
        return out;
    }

    pub fn pointTotalLambdaPosition(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltPointConstraintGetTotalLambdaPosition(self.handle, &out));
        return out;
    }

    /// The hinge's frame, read back in each body's CENTRE-OF-MASS space —
    /// what Jolt resolved the descriptor to, not a `.world` descriptor's
    /// original value.
    ///
    /// No setter for any of these: a hinge derives its rest orientation
    /// from the frame once, at construction, and never revisits it — rebuild the constraint to move it.
    pub fn hingeLocalSpacePoint1(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltHingeConstraintGetLocalSpacePoint1(self.handle, &out));
        return out;
    }

    pub fn hingeLocalSpacePoint2(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltHingeConstraintGetLocalSpacePoint2(self.handle, &out));
        return out;
    }

    /// The axis rotation is allowed about, on body 1.
    pub fn hingeLocalSpaceHingeAxis1(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltHingeConstraintGetLocalSpaceHingeAxis1(self.handle, &out));
        return out;
    }

    pub fn hingeLocalSpaceHingeAxis2(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltHingeConstraintGetLocalSpaceHingeAxis2(self.handle, &out));
        return out;
    }

    /// The reference `hingeCurrentAngle` is measured from, on body 1.
    pub fn hingeLocalSpaceNormalAxis1(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltHingeConstraintGetLocalSpaceNormalAxis1(self.handle, &out));
        return out;
    }

    pub fn hingeLocalSpaceNormalAxis2(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltHingeConstraintGetLocalSpaceNormalAxis2(self.handle, &out));
        return out;
    }

    /// Radians, measured from where the two normal axes align.
    pub fn hingeCurrentAngle(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltHingeConstraintGetCurrentAngle(self.handle, &out));
        return out;
    }

    /// `min` must be in [-pi, 0] and `max` in [0, pi] — Jolt asserts both.
    pub fn hingeSetLimits(self: Constraint, min: f32, max: f32) err.Error!void {
        try err.check(c.zjoltHingeConstraintSetLimits(self.handle, min, max));
    }

    pub fn hingeLimits(self: Constraint) err.Error!struct { min: f32, max: f32 } {
        var min: f32 = 0;
        var max: f32 = 0;
        try err.check(c.zjoltHingeConstraintGetLimits(self.handle, &min, &max));
        return .{ .min = min, .max = max };
    }

    /// False means the hinge spins freely and the limit part is not solved.
    pub fn hingeHasLimits(self: Constraint) err.Error!bool {
        var out: bool = false;
        try err.check(c.zjoltHingeConstraintHasLimits(self.handle, &out));
        return out;
    }

    pub fn hingeSetLimitsSpring(self: Constraint, spring: SpringSettings) err.Error!void {
        try err.check(c.zjoltHingeConstraintSetLimitsSpringSettings(self.handle, &spring));
    }

    pub fn hingeLimitsSpring(self: Constraint) err.Error!SpringSettings {
        var out: SpringSettings = .{};
        try err.check(c.zjoltHingeConstraintGetLimitsSpringSettings(self.handle, &out));
        return out;
    }

    /// Refused when the settings are not valid, because the next
    /// `hingeSetMotorState` would assert on exactly that.
    pub fn hingeSetMotorSettings(self: Constraint, motor: MotorSettings) err.Error!void {
        try err.check(c.zjoltHingeConstraintSetMotorSettings(self.handle, &motor));
    }

    pub fn hingeMotorSettings(self: Constraint) err.Error!MotorSettings {
        var out: MotorSettings = .{};
        try err.check(c.zjoltHingeConstraintGetMotorSettings(self.handle, &out));
        return out;
    }

    pub fn hingeSetMotorState(self: Constraint, state: MotorState) err.Error!void {
        try err.check(c.zjoltHingeConstraintSetMotorState(self.handle, state));
    }

    pub fn hingeMotorState(self: Constraint) err.Error!MotorState {
        var out: MotorState = .off;
        try err.check(c.zjoltHingeConstraintGetMotorState(self.handle, &out));
        return out;
    }

    /// Radians per second, for a velocity motor.
    pub fn hingeSetTargetAngularVelocity(self: Constraint, velocity: f32) err.Error!void {
        try err.check(c.zjoltHingeConstraintSetTargetAngularVelocity(self.handle, velocity));
    }

    pub fn hingeTargetAngularVelocity(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltHingeConstraintGetTargetAngularVelocity(self.handle, &out));
        return out;
    }

    /// Radians, for a position motor. Jolt CLAMPS this to the current limits,
    /// so reading it back may give a different number.
    pub fn hingeSetTargetAngle(self: Constraint, angle: f32) err.Error!void {
        try err.check(c.zjoltHingeConstraintSetTargetAngle(self.handle, angle));
    }

    pub fn hingeTargetAngle(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltHingeConstraintGetTargetAngle(self.handle, &out));
        return out;
    }

    /// The same target as an orientation of body 2 relative to body 1,
    /// projected onto the hinge axis and clamped as above.
    ///
    /// No getter, deliberately: Jolt reduces `orientation` to an angle
    /// and keeps only that (same state `hingeSetTargetAngle` leaves), so
    /// no quaternion survives to read back. Use `hingeTargetAngle`.
    pub fn hingeSetTargetOrientation(self: Constraint, orientation: math.Quat) err.Error!void {
        try err.check(c.zjoltHingeConstraintSetTargetOrientation(self.handle, &orientation));
    }

    pub fn hingeSetMaxFrictionTorque(self: Constraint, torque: f32) err.Error!void {
        try err.check(c.zjoltHingeConstraintSetMaxFrictionTorque(self.handle, torque));
    }

    pub fn hingeMaxFrictionTorque(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltHingeConstraintGetMaxFrictionTorque(self.handle, &out));
        return out;
    }

    pub fn hingeTotalLambdaPosition(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltHingeConstraintGetTotalLambdaPosition(self.handle, &out));
        return out;
    }

    /// The impulse that held the two bodies to the hinge AXIS — two
    /// numbers, since a hinge removes exactly two rotational DOF: what a
    /// hinge twisted out of plane is spending. Distinct from
    /// `hingeTotalLambdaRotationLimits` (a door forced past its stop) and `hingeTotalLambdaMotor` (the motor holding its target).
    pub fn hingeTotalLambdaRotation(self: Constraint) err.Error![2]f32 {
        var out: [2]f32 = .{ 0, 0 };
        try err.check(c.zjoltHingeConstraintGetTotalLambdaRotation(self.handle, &out[0], &out[1]));
        return out;
    }

    pub fn hingeTotalLambdaRotationLimits(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltHingeConstraintGetTotalLambdaRotationLimits(self.handle, &out));
        return out;
    }

    pub fn hingeTotalLambdaMotor(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltHingeConstraintGetTotalLambdaMotor(self.handle, &out));
        return out;
    }

    /// Metres along the slider axis, from where the two points coincide.
    pub fn sliderCurrentPosition(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSliderConstraintGetCurrentPosition(self.handle, &out));
        return out;
    }

    /// `min` must be <= 0 and `max` >= 0 — Jolt asserts both.
    pub fn sliderSetLimits(self: Constraint, min: f32, max: f32) err.Error!void {
        try err.check(c.zjoltSliderConstraintSetLimits(self.handle, min, max));
    }

    pub fn sliderLimits(self: Constraint) err.Error!struct { min: f32, max: f32 } {
        var min: f32 = 0;
        var max: f32 = 0;
        try err.check(c.zjoltSliderConstraintGetLimits(self.handle, &min, &max));
        return .{ .min = min, .max = max };
    }

    pub fn sliderHasLimits(self: Constraint) err.Error!bool {
        var out: bool = false;
        try err.check(c.zjoltSliderConstraintHasLimits(self.handle, &out));
        return out;
    }

    pub fn sliderSetLimitsSpring(self: Constraint, spring: SpringSettings) err.Error!void {
        try err.check(c.zjoltSliderConstraintSetLimitsSpringSettings(self.handle, &spring));
    }

    pub fn sliderLimitsSpring(self: Constraint) err.Error!SpringSettings {
        var out: SpringSettings = .{};
        try err.check(c.zjoltSliderConstraintGetLimitsSpringSettings(self.handle, &out));
        return out;
    }

    pub fn sliderSetMotorSettings(self: Constraint, motor: MotorSettings) err.Error!void {
        try err.check(c.zjoltSliderConstraintSetMotorSettings(self.handle, &motor));
    }

    pub fn sliderMotorSettings(self: Constraint) err.Error!MotorSettings {
        var out: MotorSettings = .{};
        try err.check(c.zjoltSliderConstraintGetMotorSettings(self.handle, &out));
        return out;
    }

    pub fn sliderSetMotorState(self: Constraint, state: MotorState) err.Error!void {
        try err.check(c.zjoltSliderConstraintSetMotorState(self.handle, state));
    }

    pub fn sliderMotorState(self: Constraint) err.Error!MotorState {
        var out: MotorState = .off;
        try err.check(c.zjoltSliderConstraintGetMotorState(self.handle, &out));
        return out;
    }

    /// Metres per second, for a velocity motor.
    pub fn sliderSetTargetVelocity(self: Constraint, velocity: f32) err.Error!void {
        try err.check(c.zjoltSliderConstraintSetTargetVelocity(self.handle, velocity));
    }

    pub fn sliderTargetVelocity(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSliderConstraintGetTargetVelocity(self.handle, &out));
        return out;
    }

    /// Metres, for a position motor. Clamped to the current limits.
    pub fn sliderSetTargetPosition(self: Constraint, position: f32) err.Error!void {
        try err.check(c.zjoltSliderConstraintSetTargetPosition(self.handle, position));
    }

    pub fn sliderTargetPosition(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSliderConstraintGetTargetPosition(self.handle, &out));
        return out;
    }

    pub fn sliderSetMaxFrictionForce(self: Constraint, force: f32) err.Error!void {
        try err.check(c.zjoltSliderConstraintSetMaxFrictionForce(self.handle, force));
    }

    pub fn sliderMaxFrictionForce(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSliderConstraintGetMaxFrictionForce(self.handle, &out));
        return out;
    }

    /// The impulse that held the body onto the slider AXIS, one number per
    /// perpendicular direction. Travel past a limit is
    /// `sliderTotalLambdaPositionLimits` instead — this is what a slider being
    /// bent sideways is spending.
    pub fn sliderTotalLambdaPosition(self: Constraint) err.Error![2]f32 {
        var out: [2]f32 = .{ 0, 0 };
        try err.check(c.zjoltSliderConstraintGetTotalLambdaPosition(self.handle, &out[0], &out[1]));
        return out;
    }

    pub fn sliderTotalLambdaPositionLimits(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSliderConstraintGetTotalLambdaPositionLimits(self.handle, &out));
        return out;
    }

    /// The torque that kept the two bodies from rotating relative to each
    /// other.
    pub fn sliderTotalLambdaRotation(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSliderConstraintGetTotalLambdaRotation(self.handle, &out));
        return out;
    }

    pub fn sliderTotalLambdaMotor(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSliderConstraintGetTotalLambdaMotor(self.handle, &out));
        return out;
    }

    /// `min` must be <= `max` — Jolt asserts it. Unlike the descriptor, a
    /// negative value here is not a sentinel: this is the raw setter.
    pub fn distanceSetDistance(self: Constraint, min: f32, max: f32) err.Error!void {
        try err.check(c.zjoltDistanceConstraintSetDistance(self.handle, min, max));
    }

    pub fn distanceDistance(self: Constraint) err.Error!struct { min: f32, max: f32 } {
        var min: f32 = 0;
        var max: f32 = 0;
        try err.check(c.zjoltDistanceConstraintGetDistance(self.handle, &min, &max));
        return .{ .min = min, .max = max };
    }

    pub fn distanceSetLimitsSpring(self: Constraint, spring: SpringSettings) err.Error!void {
        try err.check(c.zjoltDistanceConstraintSetLimitsSpringSettings(self.handle, &spring));
    }

    pub fn distanceLimitsSpring(self: Constraint) err.Error!SpringSettings {
        var out: SpringSettings = .{};
        try err.check(c.zjoltDistanceConstraintGetLimitsSpringSettings(self.handle, &out));
        return out;
    }

    pub fn distanceTotalLambdaPosition(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltDistanceConstraintGetTotalLambdaPosition(self.handle, &out));
        return out;
    }

    //-------------------------------------------------------------------------
    // Cone
    //-------------------------------------------------------------------------

    /// Radians from the principal axis to the cone's edge, in [0, pi].
    pub fn coneSetHalfConeAngle(self: Constraint, angle: f32) err.Error!void {
        try err.check(c.zjoltConeConstraintSetHalfConeAngle(self.handle, angle));
    }

    /// The COSINE of the half angle, which is what the constraint keeps. Jolt
    /// has no getter for the angle itself.
    pub fn coneCosHalfConeAngle(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltConeConstraintGetCosHalfConeAngle(self.handle, &out));
        return out;
    }

    pub fn coneTotalLambdaPosition(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltConeConstraintGetTotalLambdaPosition(self.handle, &out));
        return out;
    }

    pub fn coneTotalLambdaRotation(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltConeConstraintGetTotalLambdaRotation(self.handle, &out));
        return out;
    }

    //-------------------------------------------------------------------------
    // Swing-twist
    //-------------------------------------------------------------------------

    /// The attachment point on each body, in that body's centre-of-mass space.
    pub fn swingTwistLocalSpacePosition1(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetLocalSpacePosition1(self.handle, &out));
        return out;
    }

    pub fn swingTwistLocalSpacePosition2(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetLocalSpacePosition2(self.handle, &out));
        return out;
    }

    /// The rotation from constraint space into body 1's centre-of-mass space.
    ///
    /// This is the conversion the constraint-space targets are named after:
    /// `swingTwistTargetOrientation` reads back in CONSTRAINT space, and
    /// composing it with these two puts it into a body's own frame.
    pub fn swingTwistConstraintToBody1(self: Constraint) err.Error!math.Quat {
        var out: math.Quat = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetConstraintToBody1(self.handle, &out));
        return out;
    }

    pub fn swingTwistConstraintToBody2(self: Constraint) err.Error!math.Quat {
        var out: math.Quat = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetConstraintToBody2(self.handle, &out));
        return out;
    }

    pub fn swingTwistSetNormalHalfConeAngle(self: Constraint, angle: f32) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetNormalHalfConeAngle(self.handle, angle));
    }

    pub fn swingTwistNormalHalfConeAngle(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSwingTwistConstraintGetNormalHalfConeAngle(self.handle, &out));
        return out;
    }

    pub fn swingTwistSetPlaneHalfConeAngle(self: Constraint, angle: f32) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetPlaneHalfConeAngle(self.handle, angle));
    }

    pub fn swingTwistPlaneHalfConeAngle(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSwingTwistConstraintGetPlaneHalfConeAngle(self.handle, &out));
        return out;
    }

    /// Both twist limits at once, in radians, both in [-pi, pi] and ordered.
    ///
    /// One call rather than two setters on purpose: Jolt recomputes the
    /// constraint part inside each individual setter and asserts on the
    /// ordering, so moving a range through a single setter can pass through an
    /// invalid intermediate state.
    pub fn swingTwistSetTwistLimits(self: Constraint, min: f32, max: f32) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetTwistLimits(self.handle, min, max));
    }

    pub fn swingTwistTwistLimits(self: Constraint) err.Error!struct { min: f32, max: f32 } {
        var min: f32 = 0;
        var max: f32 = 0;
        try err.check(c.zjoltSwingTwistConstraintGetTwistLimits(self.handle, &min, &max));
        return .{ .min = min, .max = max };
    }

    pub fn swingTwistSetSwingMotorSettings(self: Constraint, motor: MotorSettings) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetSwingMotorSettings(self.handle, &motor));
    }

    pub fn swingTwistSwingMotorSettings(self: Constraint) err.Error!MotorSettings {
        var out: MotorSettings = .{};
        try err.check(c.zjoltSwingTwistConstraintGetSwingMotorSettings(self.handle, &out));
        return out;
    }

    pub fn swingTwistSetSwingMotorState(self: Constraint, state: MotorState) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetSwingMotorState(self.handle, state));
    }

    pub fn swingTwistSwingMotorState(self: Constraint) err.Error!MotorState {
        var out: MotorState = .off;
        try err.check(c.zjoltSwingTwistConstraintGetSwingMotorState(self.handle, &out));
        return out;
    }

    pub fn swingTwistSetTwistMotorSettings(self: Constraint, motor: MotorSettings) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetTwistMotorSettings(self.handle, &motor));
    }

    pub fn swingTwistTwistMotorSettings(self: Constraint) err.Error!MotorSettings {
        var out: MotorSettings = .{};
        try err.check(c.zjoltSwingTwistConstraintGetTwistMotorSettings(self.handle, &out));
        return out;
    }

    pub fn swingTwistSetTwistMotorState(self: Constraint, state: MotorState) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetTwistMotorState(self.handle, state));
    }

    pub fn swingTwistTwistMotorState(self: Constraint) err.Error!MotorState {
        var out: MotorState = .off;
        try err.check(c.zjoltSwingTwistConstraintGetTwistMotorState(self.handle, &out));
        return out;
    }

    pub fn swingTwistSetMaxFrictionTorque(self: Constraint, torque: f32) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetMaxFrictionTorque(self.handle, torque));
    }

    pub fn swingTwistMaxFrictionTorque(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSwingTwistConstraintGetMaxFrictionTorque(self.handle, &out));
        return out;
    }

    /// Radians per second, in CONSTRAINT space: x is twist, y/z are swing.
    ///
    /// Setting a target while the motor is off is not an error: the
    /// value is kept and starts driving once `swingTwistSetSwingMotorState`
    /// / `swingTwistSetTwistMotorState` turns the motor on — the usual reason a joint seems to "ignore" a target.
    pub fn swingTwistSetTargetAngularVelocity(self: Constraint, velocity: math.Vec3) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetTargetAngularVelocity(self.handle, &velocity));
    }

    /// The same target expressed in BODY 2's space; stored as the
    /// constraint-space one, so `swingTwistTargetAngularVelocity` returns
    /// the CONVERTED value, not what was passed here. An angular
    /// velocity taken from a body's own motion is in body space —
    /// handing it to the constraint-space setter instead rotates it by the constraint frame.
    pub fn swingTwistSetTargetAngularVelocityBodySpace(self: Constraint, velocity: math.Vec3) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetTargetAngularVelocityBodySpace(self.handle, &velocity));
    }

    pub fn swingTwistTargetAngularVelocity(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetTargetAngularVelocity(self.handle, &out));
        return out;
    }

    /// Clamped to the current swing and twist limits, so reading it back may
    /// differ from what was written.
    pub fn swingTwistSetTargetOrientation(self: Constraint, orientation: math.Quat) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetTargetOrientation(self.handle, &orientation));
    }

    /// The same target as the rotation of body 2 RELATIVE TO BODY 1 —
    /// the `q` in `world_rotation2 = world_rotation1 * q`, the shape a
    /// pose already comes in (the call an animation-driven ragdoll
    /// wants). Clamped identically; reads back through
    /// `swingTwistTargetOrientation` in constraint space.
    pub fn swingTwistSetTargetOrientationBodySpace(self: Constraint, orientation: math.Quat) err.Error!void {
        try err.check(c.zjoltSwingTwistConstraintSetTargetOrientationBodySpace(self.handle, &orientation));
    }

    pub fn swingTwistTargetOrientation(self: Constraint) err.Error!math.Quat {
        var out: math.Quat = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetTargetOrientation(self.handle, &out));
        return out;
    }

    /// Where the joint actually is, as a rotation in constraint space.
    pub fn swingTwistRotationInConstraintSpace(self: Constraint) err.Error!math.Quat {
        var out: math.Quat = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetRotationInConstraintSpace(self.handle, &out));
        return out;
    }

    pub fn swingTwistTotalLambdaPosition(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetTotalLambdaPosition(self.handle, &out));
        return out;
    }

    /// The torque each LIMIT applied over the last step. Zero inside the
    /// cone and twist range; non-zero signals a joint forced past a
    /// limit — what a ragdoll bone that should break or go limp watches.
    /// Distinct from `swingTwistTotalLambdaMotor` (what the motors spend holding their own targets).
    pub fn swingTwistTotalLambdaTwist(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSwingTwistConstraintGetTotalLambdaTwist(self.handle, &out));
        return out;
    }

    pub fn swingTwistTotalLambdaSwingY(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSwingTwistConstraintGetTotalLambdaSwingY(self.handle, &out));
        return out;
    }

    pub fn swingTwistTotalLambdaSwingZ(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSwingTwistConstraintGetTotalLambdaSwingZ(self.handle, &out));
        return out;
    }

    pub fn swingTwistTotalLambdaMotor(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSwingTwistConstraintGetTotalLambdaMotor(self.handle, &out));
        return out;
    }

    //-------------------------------------------------------------------------
    // Six degrees of freedom
    //
    // Every axis argument is checked against `SixDofAxis`: Jolt indexes plain
    // arrays of six with it and has no bounds check of its own.
    //-------------------------------------------------------------------------

    pub fn sixDofSetTranslationLimits(self: Constraint, min: math.Vec3, max: math.Vec3) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetTranslationLimits(self.handle, &min, &max));
    }

    /// Radians. Clamped to [-pi, pi], and forced symmetric for a cone swing,
    /// so what comes back out may not be what went in.
    pub fn sixDofSetRotationLimits(self: Constraint, min: math.Vec3, max: math.Vec3) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetRotationLimits(self.handle, &min, &max));
    }

    /// The reader for both `sixDofSetTranslationLimits` and
    /// `sixDofSetRotationLimits`: whichever last touched `axis`, this returns
    /// exactly what Jolt kept for it, sanitising included.
    pub fn sixDofLimits(self: Constraint, axis: SixDofAxis) err.Error!struct { min: f32, max: f32 } {
        var min: f32 = 0;
        var max: f32 = 0;
        try err.check(c.zjoltSixDofConstraintGetLimits(self.handle, axis, &min, &max));
        return .{ .min = min, .max = max };
    }

    pub fn sixDofIsFixedAxis(self: Constraint, axis: SixDofAxis) err.Error!bool {
        var out: bool = false;
        try err.check(c.zjoltSixDofConstraintIsFixedAxis(self.handle, axis, &out));
        return out;
    }

    pub fn sixDofIsFreeAxis(self: Constraint, axis: SixDofAxis) err.Error!bool {
        var out: bool = false;
        try err.check(c.zjoltSixDofConstraintIsFreeAxis(self.handle, axis, &out));
        return out;
    }

    pub fn sixDofSetMaxFriction(self: Constraint, axis: SixDofAxis, friction: f32) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetMaxFriction(self.handle, axis, friction));
    }

    pub fn sixDofMaxFriction(self: Constraint, axis: SixDofAxis) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltSixDofConstraintGetMaxFriction(self.handle, axis, &out));
        return out;
    }

    pub fn sixDofSetMotorSettings(self: Constraint, axis: SixDofAxis, motor: MotorSettings) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetMotorSettings(self.handle, axis, &motor));
    }

    pub fn sixDofMotorSettings(self: Constraint, axis: SixDofAxis) err.Error!MotorSettings {
        var out: MotorSettings = .{};
        try err.check(c.zjoltSixDofConstraintGetMotorSettings(self.handle, axis, &out));
        return out;
    }

    pub fn sixDofSetMotorState(self: Constraint, axis: SixDofAxis, state: MotorState) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetMotorState(self.handle, axis, state));
    }

    pub fn sixDofMotorState(self: Constraint, axis: SixDofAxis) err.Error!MotorState {
        var out: MotorState = .off;
        try err.check(c.zjoltSixDofConstraintGetMotorState(self.handle, axis, &out));
        return out;
    }

    /// Translation axes only — Jolt has no soft rotation limits and asserts on
    /// a rotation axis here.
    pub fn sixDofSetLimitsSpring(self: Constraint, axis: SixDofAxis, spring: SpringSettings) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetLimitsSpringSettings(self.handle, axis, &spring));
    }

    pub fn sixDofLimitsSpring(self: Constraint, axis: SixDofAxis) err.Error!SpringSettings {
        var out: SpringSettings = .{};
        try err.check(c.zjoltSixDofConstraintGetLimitsSpringSettings(self.handle, axis, &out));
        return out;
    }

    pub fn sixDofSetTargetVelocity(self: Constraint, velocity: math.Vec3) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetTargetVelocity(self.handle, &velocity));
    }

    pub fn sixDofTargetVelocity(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSixDofConstraintGetTargetVelocity(self.handle, &out));
        return out;
    }

    pub fn sixDofSetTargetAngularVelocity(self: Constraint, velocity: math.Vec3) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetTargetAngularVelocity(self.handle, &velocity));
    }

    pub fn sixDofTargetAngularVelocity(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSixDofConstraintGetTargetAngularVelocity(self.handle, &out));
        return out;
    }

    pub fn sixDofSetTargetPosition(self: Constraint, position: math.Vec3) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetTargetPosition(self.handle, &position));
    }

    pub fn sixDofTargetPosition(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSixDofConstraintGetTargetPosition(self.handle, &out));
        return out;
    }

    pub fn sixDofSetTargetOrientation(self: Constraint, orientation: math.Quat) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetTargetOrientation(self.handle, &orientation));
    }

    /// The same target as the rotation of body 2 relative to body 1, as
    /// `swingTwistSetTargetOrientationBodySpace` is; reads back through
    /// `sixDofTargetOrientation` in constraint space.
    ///
    /// This constraint's TRANSLATION motors work in body 1's constraint
    /// space, ROTATION motors in body 2's — that asymmetry applies to the constraint-space calls; this one takes body space and converts.
    pub fn sixDofSetTargetOrientationBodySpace(self: Constraint, orientation: math.Quat) err.Error!void {
        try err.check(c.zjoltSixDofConstraintSetTargetOrientationBodySpace(self.handle, &orientation));
    }

    pub fn sixDofTargetOrientation(self: Constraint) err.Error!math.Quat {
        var out: math.Quat = undefined;
        try err.check(c.zjoltSixDofConstraintGetTargetOrientation(self.handle, &out));
        return out;
    }

    pub fn sixDofRotationInConstraintSpace(self: Constraint) err.Error!math.Quat {
        var out: math.Quat = undefined;
        try err.check(c.zjoltSixDofConstraintGetRotationInConstraintSpace(self.handle, &out));
        return out;
    }

    pub fn sixDofTotalLambdaPosition(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSixDofConstraintGetTotalLambdaPosition(self.handle, &out));
        return out;
    }

    pub fn sixDofTotalLambdaRotation(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSixDofConstraintGetTotalLambdaRotation(self.handle, &out));
        return out;
    }

    pub fn sixDofTotalLambdaMotorTranslation(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSixDofConstraintGetTotalLambdaMotorTranslation(self.handle, &out));
        return out;
    }

    pub fn sixDofTotalLambdaMotorRotation(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltSixDofConstraintGetTotalLambdaMotorRotation(self.handle, &out));
        return out;
    }

    //-------------------------------------------------------------------------
    // Gear, rack and pinion
    //-------------------------------------------------------------------------

    /// Hands the gear the two HINGES its bodies are mounted on, so it can
    /// correct positional drift rather than matching velocities only. Null for
    /// both clears them.
    ///
    /// Both must be hinges: Jolt reads them during position solving and aborts
    /// on anything else, a frame later and with nothing naming this call.
    pub fn gearSetConstraints(self: Constraint, gear1: ?Constraint, gear2: ?Constraint) err.Error!void {
        try err.check(c.zjoltGearConstraintSetConstraints(
            self.handle,
            if (gear1) |g| g.handle else null,
            if (gear2) |g| g.handle else null,
        ));
    }

    pub fn gearTotalLambda(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltGearConstraintGetTotalLambda(self.handle, &out));
        return out;
    }

    /// As `gearSetConstraints`, and with the same abort behind it: `pinion`
    /// must be a hinge and `rack` must be a slider.
    pub fn rackAndPinionSetConstraints(self: Constraint, pinion: ?Constraint, rack: ?Constraint) err.Error!void {
        try err.check(c.zjoltRackAndPinionConstraintSetConstraints(
            self.handle,
            if (pinion) |p| p.handle else null,
            if (rack) |r| r.handle else null,
        ));
    }

    pub fn rackAndPinionTotalLambda(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltRackAndPinionConstraintGetTotalLambda(self.handle, &out));
        return out;
    }

    //-------------------------------------------------------------------------
    // Pulley
    //-------------------------------------------------------------------------

    /// Metres of rope. `min` must be non-negative and no greater than `max`.
    /// The descriptor's negative sentinel does not apply here.
    pub fn pulleySetLength(self: Constraint, min: f32, max: f32) err.Error!void {
        try err.check(c.zjoltPulleyConstraintSetLength(self.handle, min, max));
    }

    pub fn pulleyLength(self: Constraint) err.Error!struct { min: f32, max: f32 } {
        var min: f32 = 0;
        var max: f32 = 0;
        try err.check(c.zjoltPulleyConstraintGetLength(self.handle, &min, &max));
        return .{ .min = min, .max = max };
    }

    /// Both segments summed with the ratio applied. Computed from the
    /// positions the last step cached, so before the first step it reports the
    /// length at creation.
    pub fn pulleyCurrentLength(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltPulleyConstraintGetCurrentLength(self.handle, &out));
        return out;
    }

    pub fn pulleyTotalLambdaPosition(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltPulleyConstraintGetTotalLambdaPosition(self.handle, &out));
        return out;
    }

    //-------------------------------------------------------------------------
    // Path
    //-------------------------------------------------------------------------

    /// Replaces the path and the point along it that body 2 sits at. Null makes
    /// the constraint inactive: it stays in the system and applies nothing.
    pub fn pathSetPath(self: Constraint, path: ?Path, fraction: f32) err.Error!void {
        try err.check(c.zjoltPathConstraintSetPath(
            self.handle,
            if (path) |p| p.handle else null,
            fraction,
        ));
    }

    /// BORROWED — valid only while the constraint holds it. Call `addRef` on
    /// the result to keep it.
    ///
    /// Null means the constraint has no path set, which is a normal state.
    /// A constraint that is not a path constraint is `error.InvalidArgument`
    /// instead — the two would otherwise be the same answer.
    pub fn pathPath(self: Constraint) err.Error!?Path {
        var raw: ?*const c.PathConstraintPath = null;
        try err.check(c.zjoltPathConstraintGetPath(self.handle, &raw));
        return if (raw) |p| Path{ .handle = @constCast(p) } else null;
    }

    pub fn pathFraction(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltPathConstraintGetPathFraction(self.handle, &out));
        return out;
    }

    pub fn pathSetMotorSettings(self: Constraint, motor: MotorSettings) err.Error!void {
        try err.check(c.zjoltPathConstraintSetMotorSettings(self.handle, &motor));
    }

    pub fn pathMotorSettings(self: Constraint) err.Error!MotorSettings {
        var out: MotorSettings = .{};
        try err.check(c.zjoltPathConstraintGetMotorSettings(self.handle, &out));
        return out;
    }

    pub fn pathSetMotorState(self: Constraint, state: MotorState) err.Error!void {
        try err.check(c.zjoltPathConstraintSetMotorState(self.handle, state));
    }

    pub fn pathMotorState(self: Constraint) err.Error!MotorState {
        var out: MotorState = .off;
        try err.check(c.zjoltPathConstraintGetMotorState(self.handle, &out));
        return out;
    }

    /// Metres per second along the path, for a velocity motor.
    pub fn pathSetTargetVelocity(self: Constraint, velocity: f32) err.Error!void {
        try err.check(c.zjoltPathConstraintSetTargetVelocity(self.handle, velocity));
    }

    pub fn pathTargetVelocity(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltPathConstraintGetTargetVelocity(self.handle, &out));
        return out;
    }

    /// On a non-looping path this must lie between 0 and the path's max
    /// fraction — Jolt asserts it. A looping path accepts anything and wraps.
    pub fn pathSetTargetFraction(self: Constraint, fraction: f32) err.Error!void {
        try err.check(c.zjoltPathConstraintSetTargetPathFraction(self.handle, fraction));
    }

    pub fn pathTargetFraction(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltPathConstraintGetTargetPathFraction(self.handle, &out));
        return out;
    }

    pub fn pathSetMaxFrictionForce(self: Constraint, force: f32) err.Error!void {
        try err.check(c.zjoltPathConstraintSetMaxFrictionForce(self.handle, force));
    }

    pub fn pathMaxFrictionForce(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltPathConstraintGetMaxFrictionForce(self.handle, &out));
        return out;
    }

    /// The impulse that held body 2 ON the path, one number per direction
    /// perpendicular to the tangent.
    pub fn pathTotalLambdaPosition(self: Constraint) err.Error![2]f32 {
        var out: [2]f32 = .{ 0, 0 };
        try err.check(c.zjoltPathConstraintGetTotalLambdaPosition(self.handle, &out[0], &out[1]));
        return out;
    }

    /// The impulse that stopped body 2 running off the end of a non-looping
    /// path. Always zero on a looping one, which has no end to stop at.
    pub fn pathTotalLambdaPositionLimits(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltPathConstraintGetTotalLambdaPositionLimits(self.handle, &out));
        return out;
    }

    /// The torque the rotation constraint applied, in the two shapes it takes.
    /// The hinge pair is what the `constrain_around_*` rotation types use and
    /// the three-component one is what `constrain_to_path` and
    /// `fully_constrained` use; whichever the constraint is not using reads
    /// zero, so both are safe to sample without branching on the type.
    pub fn pathTotalLambdaRotationHinge(self: Constraint) err.Error![2]f32 {
        var out: [2]f32 = .{ 0, 0 };
        try err.check(c.zjoltPathConstraintGetTotalLambdaRotationHinge(self.handle, &out[0], &out[1]));
        return out;
    }

    pub fn pathTotalLambdaRotation(self: Constraint) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltPathConstraintGetTotalLambdaRotation(self.handle, &out));
        return out;
    }

    pub fn pathTotalLambdaMotor(self: Constraint) err.Error!f32 {
        var out: f32 = 0;
        try err.check(c.zjoltPathConstraintGetTotalLambdaMotor(self.handle, &out));
        return out;
    }
};

/// How many constraints are currently added to `system`.
pub fn count(system: system_mod.PhysicsSystem) u32 {
    return c.zjoltPhysicsSystemGetNumConstraints(system.handle);
}

//=============================================================================
// The behavioural tests
//
// A hinge takes five DOF away and leaves one; a six-DOF joint reports
// back, axis by axis, exactly the limits it was told to keep. Not one test per entry point — `misuse_sweep_test.zig`/`abi_check.zig` cover the rest mechanically.
//=============================================================================

const zjolt = @import("zjolt.zig");

const TestLayers = struct {
    pub const static: system_mod.ObjectLayer = 0;
    pub const moving: system_mod.ObjectLayer = 1;

    pub const bp_static: system_mod.BroadPhaseLayer = 0;
    pub const bp_moving: system_mod.BroadPhaseLayer = 1;

    pub fn broadPhaseLayerCount() u32 {
        return 2;
    }

    pub fn broadPhaseLayerFor(layer: system_mod.ObjectLayer) system_mod.BroadPhaseLayer {
        return if (layer == static) bp_static else bp_moving;
    }

    pub fn objectCanCollideWithBroadPhase(
        object: system_mod.ObjectLayer,
        broad: system_mod.BroadPhaseLayer,
    ) bool {
        return if (object == static) broad == bp_moving else true;
    }

    pub fn objectsCanCollide(a: system_mod.ObjectLayer, b: system_mod.ObjectLayer) bool {
        return if (a == static) b == moving else true;
    }
};

test "a hinge constrains a body to rotation about its axis" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const jobs = try zjolt.JobSystem.initSingleThreaded(c.max_physics_jobs);
    defer jobs.deinit();

    const system = try zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(TestLayers),
        .max_bodies = 64,
    });
    defer system.deinit();

    // No gravity: the only thing acting on the body is the impulse below and
    // the hinge's reaction to it, which is what the assertions are about.
    system.setGravity(zjolt.vec3(0, 0, 0));

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.5, 0.5), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const door = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = TestLayers.moving,
        .motion_type = .dynamic,
        .position = anchor,
    }, .activate);

    // Hinged to the world about Y, at the body's own position. Body 1 is the
    // world, so the hinge frame is the world frame.
    var hinge = try Constraint.initHinge(system, world_body, door, .{
        .point1 = anchor,
        .point2 = anchor,
        .hinge_axis1 = zjolt.vec3(0, 1, 0),
        .normal_axis1 = zjolt.vec3(1, 0, 0),
        .hinge_axis2 = zjolt.vec3(0, 1, 0),
        .normal_axis2 = zjolt.vec3(1, 0, 0),
    });
    defer hinge.release();

    try std.testing.expectEqual(SubType.hinge, hinge.subType());
    try std.testing.expectEqual(@as(u32, 0), count(system));

    try hinge.addTo(system);
    try std.testing.expect(hinge.isAddedTo(system));
    try std.testing.expectEqual(@as(u32, 1), count(system));

    // Twist about all three axes at once. Only the Y component may survive.
    system.bodies().addAngularImpulse(door, zjolt.vec3(1000, 1000, 1000));

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 0.5) : (elapsed += dt) {
        _ = try system.step(dt, 1, jobs);
    }

    const angular = system.bodies().getAngularVelocity(door);
    try std.testing.expect(angular.y > 1.0);
    try std.testing.expect(@abs(angular.x) < 5.0e-2);
    try std.testing.expect(@abs(angular.z) < 5.0e-2);

    // And the point stays where it was pinned: a hinge removes all three
    // translations too.
    const transform = system.bodies().getTransform(door);
    try std.testing.expect(@abs(transform.position.x - anchor.x) < 1.0e-2);
    try std.testing.expect(@abs(transform.position.y - anchor.y) < 1.0e-2);
    try std.testing.expect(@abs(transform.position.z - anchor.z) < 1.0e-2);

    // The hinge angle is a real reading, not a stub: the body has turned.
    try std.testing.expect(@abs(try hinge.hingeCurrentAngle()) > 0.1);

    // Removing gives the system's reference back and stops the solving.
    try hinge.removeFrom(system);
    try std.testing.expect(!hinge.isAddedTo(system));
    try std.testing.expectEqual(@as(u32, 0), count(system));
}

test "a six-DOF constraint reports back the limits it was told to keep" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(TestLayers),
        .max_bodies = 64,
    });
    defer system.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.5, 0.5), .{});
    defer shape.release();

    const body = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = TestLayers.moving,
        .motion_type = .dynamic,
        .position = zjolt.rvec3(0, 0, 0),
    }, .activate);

    // Never added to a system: GetLimitsMin/GetLimitsMax read stored state,
    // not solved state, so nothing here depends on stepping.
    var joint = try Constraint.initSixDof(system, world_body, body, .{});
    defer joint.release();

    try joint.sixDofSetTranslationLimits(zjolt.vec3(-1.0, -2.0, -3.0), zjolt.vec3(1.0, 2.0, 3.0));

    // Comfortably inside [-pi, pi] and, for the default CONE swing, already
    // symmetric on Y and Z — chosen so Jolt's own sanitising in
    // UpdateRotationLimits leaves them untouched, which is the point: what
    // comes back should be what was asked for, not merely "something".
    try joint.sixDofSetRotationLimits(zjolt.vec3(-0.5, -0.3, -0.3), zjolt.vec3(0.5, 0.3, 0.3));

    const expected = [_]struct { axis: SixDofAxis, min: f32, max: f32 }{
        .{ .axis = .translation_x, .min = -1.0, .max = 1.0 },
        .{ .axis = .translation_y, .min = -2.0, .max = 2.0 },
        .{ .axis = .translation_z, .min = -3.0, .max = 3.0 },
        .{ .axis = .rotation_x, .min = -0.5, .max = 0.5 },
        .{ .axis = .rotation_y, .min = -0.3, .max = 0.3 },
        .{ .axis = .rotation_z, .min = -0.3, .max = 0.3 },
    };
    for (expected) |case| {
        const limits = try joint.sixDofLimits(case.axis);
        try std.testing.expectApproxEqAbs(case.min, limits.min, 1.0e-5);
        try std.testing.expectApproxEqAbs(case.max, limits.max, 1.0e-5);
    }
}

test "activating a constraint wakes both bodies it joins, not a third" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(TestLayers),
        .max_bodies = 64,
    });
    defer system.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.5, 0.5), .{});
    defer shape.release();

    const bodies = system.bodies();
    const a = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = TestLayers.moving,
        .motion_type = .dynamic,
        .position = zjolt.rvec3(0, 0, 0),
    }, .dont_activate);
    const b = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = TestLayers.moving,
        .motion_type = .dynamic,
        .position = zjolt.rvec3(2, 0, 0),
    }, .dont_activate);
    // Bystander: joined to nothing, must stay asleep throughout.
    const bystander = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = TestLayers.moving,
        .motion_type = .dynamic,
        .position = zjolt.rvec3(4, 0, 0),
    }, .dont_activate);

    try std.testing.expect(!bodies.isActive(a));
    try std.testing.expect(!bodies.isActive(b));

    var point = try Constraint.initPoint(system, a, b, .{
        .point1 = zjolt.rvec3(1, 0, 0),
        .point2 = zjolt.rvec3(1, 0, 0),
    });
    defer point.release();

    try point.addTo(system);

    try point.activate(system);
    try std.testing.expect(bodies.isActive(a));
    try std.testing.expect(bodies.isActive(b));
    try std.testing.expect(!bodies.isActive(bystander));
}
