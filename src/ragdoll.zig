//! Ragdolls: a skeleton, per-part bodies and constraints, driven or read
//! back as a pose.
//!
//! Three pieces, built up in order:
//!
//!   * `Skeleton` — the joint hierarchy: names and parent indices only, no
//!     transforms. Reference counted.
//!   * `RagdollSettings` — one body-and-constraint part per joint. This is
//!     the reusable template; `createRagdoll` may be called more than once
//!     to spawn more than one ragdoll from it. Reference counted, and NOT
//!     flattened away on the C side the way a body's or a shape's settings
//!     are: a live `Ragdoll` keeps a reference to the settings that created
//!     it and consults it on every `driveToPoseUsingMotors` call.
//!   * `SkeletonPose` — one instance of the hierarchy in a particular pose: a
//!     root offset plus, per joint, a local rotation/translation relative to
//!     its parent. NOT reference counted — a plain owning handle.
//!
//! A pose never carries a 4x4 matrix across the C boundary, because the ABI
//! has none. `Ragdoll.setPose`/`getPose`/`driveToPoseUsingKinematics` work
//! through the pose's internal flattened joint matrices; the caller only
//! ever reads or writes local joint rotations/translations
//! (`SkeletonPose.setJoints`/`getJoints`) plus the root offset, converting
//! with `SkeletonPose.calculateJointMatrices` (before driving) and
//! `calculateJointStates` (after reading back).

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const system_mod = @import("system.zig");

//=============================================================================
// Skeleton
//=============================================================================

pub const Skeleton = struct {
    handle: *c.Skeleton,

    pub fn init() err.Error!Skeleton {
        var handle: *c.Skeleton = undefined;
        try err.check(c.zjoltSkeletonCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn addRef(self: Skeleton) void {
        c.zjoltSkeletonAddRef(self.handle);
    }

    pub fn release(self: Skeleton) void {
        c.zjoltSkeletonRelease(self.handle);
    }

    pub fn refCount(self: Skeleton) u32 {
        return c.zjoltSkeletonGetRefCount(self.handle);
    }

    /// Appends one joint and returns its index. `parent_index` is `null` for
    /// a root joint, or otherwise the index of a joint ADDED EARLIER —
    /// required for `areJointsCorrectlyOrdered` to hold, and checked here
    /// rather than left to be discovered later.
    pub fn addJoint(self: Skeleton, name: ?[:0]const u8, parent_index: ?u32) err.Error!u32 {
        var index: u32 = undefined;
        try err.check(c.zjoltSkeletonAddJoint(
            self.handle,
            if (name) |n| n.ptr else null,
            if (parent_index) |p| @intCast(p) else -1,
            &index,
        ));
        return index;
    }

    pub fn jointCount(self: Skeleton) u32 {
        return c.zjoltSkeletonGetJointCount(self.handle);
    }

    /// `null` if no joint has that name.
    pub fn jointIndex(self: Skeleton, name: [:0]const u8) ?u32 {
        const index = c.zjoltSkeletonGetJointIndex(self.handle, name.ptr);
        return if (index < 0) null else @intCast(index);
    }

    /// `null` for a root joint, or if `index` is out of range.
    pub fn jointParentIndex(self: Skeleton, index: u32) ?u32 {
        const parent = c.zjoltSkeletonGetJointParentIndex(self.handle, index);
        return if (parent < 0) null else @intCast(parent);
    }

    /// Borrowed; valid until the skeleton is destroyed. Empty if `index` is
    /// out of range.
    pub fn jointName(self: Skeleton, index: u32) [:0]const u8 {
        return std.mem.span(c.zjoltSkeletonGetJointName(self.handle, index));
    }

    /// True if every joint's parent index is below its own — the
    /// precondition every ragdoll and pose operation has on a skeleton.
    /// `addJoint` cannot build one that fails this.
    pub fn areJointsCorrectlyOrdered(self: Skeleton) bool {
        return c.zjoltSkeletonAreJointsCorrectlyOrdered(self.handle);
    }
};

//=============================================================================
// SkeletonPose
//=============================================================================

pub const SkeletonPose = struct {
    handle: *c.SkeletonPose,

    /// A pose with no skeleton assigned (zero joints) — call `setSkeleton`
    /// before anything else.
    pub fn init() err.Error!SkeletonPose {
        var handle: *c.SkeletonPose = undefined;
        try err.check(c.zjoltSkeletonPoseCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: SkeletonPose) void {
        c.zjoltSkeletonPoseDestroy(self.handle);
    }

    /// Resizes the pose to `skeleton`'s joint count, discarding any joint
    /// data it held. The pose keeps its own reference to `skeleton`.
    pub fn setSkeleton(self: SkeletonPose, new_skeleton: Skeleton) err.Error!void {
        try err.check(c.zjoltSkeletonPoseSetSkeleton(self.handle, new_skeleton.handle));
    }

    /// The one `@constCast` in this module, and it is here on purpose: Jolt
    /// stores a pose's skeleton as a `RefConst`, so the C getter hands back a
    /// `const` pointer, but `Skeleton` is a mutable handle — its own methods
    /// mutate the joint list. Mutating a skeleton you got out of a pose is
    /// legal; it is the same object the owner of the reference holds.
    ///
    /// `null` if none is assigned.
    pub fn skeleton(self: SkeletonPose) ?Skeleton {
        const handle = c.zjoltSkeletonPoseGetSkeleton(self.handle) orelse return null;
        return .{ .handle = @constCast(handle) };
    }

    pub fn jointCount(self: SkeletonPose) u32 {
        return c.zjoltSkeletonPoseGetJointCount(self.handle);
    }

    /// Extra offset applied to the root (and therefore to all of its
    /// children).
    pub fn setRootOffset(self: SkeletonPose, offset: math.RVec3) void {
        c.zjoltSkeletonPoseSetRootOffset(self.handle, &offset);
    }

    pub fn rootOffset(self: SkeletonPose) math.RVec3 {
        var out: math.RVec3 = undefined;
        c.zjoltSkeletonPoseGetRootOffset(self.handle, &out);
        return out;
    }

    /// Bulk-sets every joint's LOCAL rotation/translation, relative to its
    /// parent (identity/zero meaning "at the parent"). Both slices must have
    /// exactly `jointCount()` entries.
    pub fn setJoints(
        self: SkeletonPose,
        rotations: []const math.Quat,
        translations: []const math.Vec3,
    ) err.Error!void {
        std.debug.assert(rotations.len == translations.len);
        try err.check(c.zjoltSkeletonPoseSetJoints(
            self.handle,
            rotations.ptr,
            translations.ptr,
            @intCast(rotations.len),
        ));
    }

    /// Bulk-reads every joint's local rotation/translation into caller-owned
    /// buffers, each exactly `jointCount()` entries.
    pub fn getJoints(
        self: SkeletonPose,
        out_rotations: []math.Quat,
        out_translations: []math.Vec3,
    ) err.Error!void {
        std.debug.assert(out_rotations.len == out_translations.len);
        var count: u32 = undefined;
        try err.check(c.zjoltSkeletonPoseGetJoints(
            self.handle,
            out_rotations.ptr,
            out_translations.ptr,
            @intCast(out_rotations.len),
            &count,
        ));
    }

    /// Converts the per-joint local rotations/translations set by
    /// `setJoints` into the flattened, root-relative form that
    /// `Ragdoll.setPose`, `getPose` and `driveToPoseUsingKinematics`
    /// consume — call this before any of them. Requires the assigned
    /// skeleton's joints to be correctly ordered, and requires a skeleton to
    /// be assigned at all.
    pub fn calculateJointMatrices(self: SkeletonPose) err.Error!void {
        try err.check(c.zjoltSkeletonPoseCalculateJointMatrices(self.handle));
    }

    /// The reverse conversion: what `Ragdoll.getPose` leaves the caller
    /// needing before `getJoints` returns anything meaningful.
    pub fn calculateJointStates(self: SkeletonPose) err.Error!void {
        try err.check(c.zjoltSkeletonPoseCalculateJointStates(self.handle));
    }
};

//=============================================================================
// RagdollSettings
//=============================================================================

/// Which swing shape a `RagdollConstraintDesc` uses. Cone limits are
/// symmetric around zero; a pyramid supports asymmetric ones.
///
/// The same type `Constraint` uses — there is one swing type, not one per
/// subsystem.
pub const SwingType = c.SwingType;

/// The swing-twist constraint attaching one part to its parent, in WORLD
/// space — the same space as the parts' `body.position`/`body.rotation`.
/// This is Jolt's own constraint for humanoid ragdolls, and the only
/// non-hinge type `Ragdoll.driveToPoseUsingMotors` knows how to drive.
///
/// Motor tuning is not exposed: driving uses Jolt's own default
/// `MotorSettings` (a critically-damped position spring, unlimited torque),
/// which is enough to reach a target pose.
pub const RagdollConstraintDesc = struct {
    position1: math.RVec3 = math.rvec3_zero,
    twist_axis1: math.Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    plane_axis1: math.Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    position2: math.RVec3 = math.rvec3_zero,
    twist_axis2: math.Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    plane_axis2: math.Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    swing_type: SwingType = .cone,
    /// Angle in radians.
    normal_half_cone_angle: f32 = 0,
    plane_half_cone_angle: f32 = 0,
    /// Angle in radians, should be within [-pi, pi].
    twist_min_angle: f32 = 0,
    twist_max_angle: f32 = 0,
    /// Torque (N m) applied as friction when no motor is driving this joint.
    max_friction_torque: f32 = 0,

    fn toC(self: RagdollConstraintDesc) c.RagdollConstraintDesc {
        return .{
            .position1 = self.position1,
            .twist_axis1 = self.twist_axis1,
            .plane_axis1 = self.plane_axis1,
            .position2 = self.position2,
            .twist_axis2 = self.twist_axis2,
            .plane_axis2 = self.plane_axis2,
            .swing_type = self.swing_type,
            .normal_half_cone_angle = self.normal_half_cone_angle,
            .plane_half_cone_angle = self.plane_half_cone_angle,
            .twist_min_angle = self.twist_min_angle,
            .twist_max_angle = self.twist_max_angle,
            .max_friction_torque = self.max_friction_torque,
        };
    }
};

/// One rigid body of a ragdoll, and the constraint attaching it to its
/// parent. `to_parent` must be `null` for the root part (the joint whose
/// parent index is `null`) and non-null for every other part —
/// `RagdollSettings.build` refuses the mismatch.
pub const RagdollPartDesc = struct {
    body: body_mod.BodyDesc,
    to_parent: ?RagdollConstraintDesc = null,
};

/// Duplicated from `body.zig`'s private `toC`: that conversion is not
/// `pub`, and this subsystem adds files and appends registrations rather
/// than editing the existing ones that would let it be shared. See the
/// comment at the top of `ffi/zjolt_ragdoll.cpp` for the same trade on the
/// C++ side.
fn bodyDescToC(desc: body_mod.BodyDesc) c.BodyDesc {
    var out: c.BodyDesc = undefined;
    c.zjoltBodyDescInit(&out);
    out.shape = desc.shape.handle;
    out.object_layer = desc.object_layer;
    out.position = desc.position;
    out.rotation = desc.rotation;
    out.linear_velocity = desc.linear_velocity;
    out.angular_velocity = desc.angular_velocity;
    out.user_data = desc.user_data;
    out.motion_type = desc.motion_type;
    out.motion_quality = desc.motion_quality;
    out.allowed_dofs = desc.allowed_dofs;
    out.override_mass_properties = desc.override_mass_properties;
    out.mass = desc.mass;
    out.allow_dynamic_or_kinematic = desc.allow_dynamic_or_kinematic;
    out.is_sensor = desc.is_sensor;
    out.allow_sleeping = desc.allow_sleeping;
    out.enhanced_internal_edge_removal = desc.enhanced_internal_edge_removal;
    out.friction = desc.friction;
    out.restitution = desc.restitution;
    out.linear_damping = desc.linear_damping;
    out.angular_damping = desc.angular_damping;
    out.max_linear_velocity = desc.max_linear_velocity;
    out.max_angular_velocity = desc.max_angular_velocity;
    out.gravity_factor = desc.gravity_factor;
    return out;
}

pub const RagdollSettings = struct {
    handle: *c.RagdollSettings,

    /// Settings with no skeleton and no parts — call `build` before
    /// anything else.
    pub fn init() err.Error!RagdollSettings {
        var handle: *c.RagdollSettings = undefined;
        try err.check(c.zjoltRagdollSettingsCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn addRef(self: RagdollSettings) void {
        c.zjoltRagdollSettingsAddRef(self.handle);
    }

    pub fn release(self: RagdollSettings) void {
        c.zjoltRagdollSettingsRelease(self.handle);
    }

    pub fn refCount(self: RagdollSettings) u32 {
        return c.zjoltRagdollSettingsGetRefCount(self.handle);
    }

    /// Assigns `skeleton` and builds one part per joint from `parts`, which
    /// must have exactly `skeleton.jointCount()` entries in joint-index
    /// order. Overwrites whatever `self` held before.
    ///
    /// `allocator` is used only for the duration of this call, to build the
    /// flat C descriptors `to_parent` points into; nothing it allocates is
    /// kept afterwards.
    pub fn build(
        self: RagdollSettings,
        allocator: std.mem.Allocator,
        skeleton: Skeleton,
        parts: []const RagdollPartDesc,
    ) (err.Error || std.mem.Allocator.Error)!void {
        const c_parts = try allocator.alloc(c.RagdollPartDesc, parts.len);
        defer allocator.free(c_parts);
        const c_constraints = try allocator.alloc(c.RagdollConstraintDesc, parts.len);
        defer allocator.free(c_constraints);

        for (parts, 0..) |part, i| {
            c_parts[i] = .{ .body = bodyDescToC(part.body), .to_parent = null };
            if (part.to_parent) |to_parent| {
                c_constraints[i] = to_parent.toC();
                c_parts[i].to_parent = &c_constraints[i];
            }
        }

        try err.check(c.zjoltRagdollSettingsBuild(
            self.handle,
            skeleton.handle,
            c_parts.ptr,
            @intCast(c_parts.len),
        ));
    }

    /// Rebalances part mass and inertia along parent-child chains so the
    /// solver stays stable. Returns false if a chain's inertia tensor could
    /// not be decomposed (rare).
    pub fn stabilize(self: RagdollSettings) bool {
        return c.zjoltRagdollSettingsStabilize(self.handle);
    }

    /// Builds a shared collision-group filter that disables collisions
    /// between every part and its parent, and assigns it (with a per-joint
    /// sub-group) to every part. Pass the resulting collision group's id to
    /// every ragdoll spawned from these settings — see `createRagdoll`.
    pub fn disableParentChildCollisions(self: RagdollSettings) void {
        c.zjoltRagdollSettingsDisableParentChildCollisions(self.handle);
    }

    /// Builds the body-index -> constraint-index table
    /// `driveToPoseUsingMotors` needs to find each body's constraint. Call
    /// this — after `build`, before `createRagdoll` — for every settings
    /// object a ragdoll driven with motors will be spawned from;
    /// `createRagdoll` does not call it for you, matching Jolt's own
    /// `RagdollSettings`.
    pub fn calculateBodyIndexToConstraintIndex(self: RagdollSettings) void {
        c.zjoltRagdollSettingsCalculateBodyIndexToConstraintIndex(self.handle);
    }

    /// Spawns one body per part and one constraint per non-root part, in
    /// `system` but NOT YET added to its simulation — see
    /// `Ragdoll.addToPhysicsSystem`. `self` is left untouched and may be
    /// used again to spawn more ragdolls from the same template.
    ///
    /// `collision_group` is the id every spawned body's collision group is
    /// set to; give ragdolls sharing a `disableParentChildCollisions` filter
    /// DIFFERENT ones — a filter is only ever consulted within one group id.
    pub fn createRagdoll(
        self: RagdollSettings,
        system: system_mod.PhysicsSystem,
        collision_group: u32,
        user_data: u64,
    ) err.Error!Ragdoll {
        var handle: *c.Ragdoll = undefined;
        try err.check(c.zjoltRagdollSettingsCreateRagdoll(
            self.handle,
            system.handle,
            collision_group,
            user_data,
            &handle,
        ));
        return .{ .handle = handle };
    }
};

//=============================================================================
// Ragdoll
//=============================================================================

pub const Ragdoll = struct {
    handle: *c.Ragdoll,

    pub fn addRef(self: Ragdoll) void {
        c.zjoltRagdollAddRef(self.handle);
    }

    /// Drops one reference. The last one takes the ragdoll back out of its
    /// physics system first if it is still in it — constraints and bodies
    /// both, with body locking — and then destroys every body it owns.
    ///
    /// So `defer ragdoll.release()` is the whole teardown, whether or not
    /// `addToPhysicsSystem` was ever called. That is this package's doing
    /// rather than Jolt's: `~Ragdoll` destroys the bodies where they stand,
    /// and a `JPH::Ragdoll` does not expose the system it was spawned in for
    /// anyone to remove them from. The handle keeps it, and asks whether the
    /// ragdoll's first body is still added before removing anything —
    /// removing one that was never added is an error in Jolt in its own
    /// right.
    ///
    /// `removeFromPhysicsSystem` remains the call for taking a ragdoll out of
    /// the simulation while keeping it alive.
    pub fn release(self: Ragdoll) void {
        c.zjoltRagdollRelease(self.handle);
    }

    /// References outstanding: one from `RagdollSettings.createRagdoll`, plus
    /// one per `addRef`, less one per `release`.
    pub fn refCount(self: Ragdoll) u32 {
        return c.zjoltRagdollGetRefCount(self.handle);
    }

    /// Adds every body and constraint to the system passed to
    /// `RagdollSettings.createRagdoll`.
    pub fn addToPhysicsSystem(self: Ragdoll, activation: body_mod.Activation, lock_bodies: bool) void {
        c.zjoltRagdollAddToPhysicsSystem(self.handle, activation, lock_bodies);
    }

    /// Takes them back out again, leaving the ragdoll alive and addable
    /// again. Not idempotent — remove at most once per `addToPhysicsSystem` —
    /// and not something `release` needs done for it.
    pub fn removeFromPhysicsSystem(self: Ragdoll, lock_bodies: bool) void {
        c.zjoltRagdollRemoveFromPhysicsSystem(self.handle, lock_bodies);
    }

    /// How many bodies the ragdoll owns — one per skeleton joint.
    pub fn bodyCount(self: Ragdoll) u32 {
        var count: u32 = undefined;
        // Cannot fail: a size query on a non-null ragdoll.
        err.check(c.zjoltRagdollGetBodyIds(self.handle, null, 0, &count)) catch unreachable;
        return count;
    }

    /// Writes the ragdoll's body ids into `out`, indexed by joint — so
    /// `out[i]` is the body built from skeleton joint `i`.
    ///
    /// This is how a contact or a ray hit gets mapped back to a limb, and how
    /// `BodyInterface` reaches one part on its own. `out` must have at least
    /// `bodyCount()` entries; a shorter one reports `error.BufferTooSmall`
    /// and writes nothing.
    ///
    /// The ids stay valid until the ragdoll's last reference drops. They name
    /// bodies the ragdoll owns — do not destroy one through `BodyInterface`.
    pub fn getBodyIds(self: Ragdoll, out: []body_mod.BodyId) err.Error![]body_mod.BodyId {
        var count: u32 = undefined;
        try err.check(c.zjoltRagdollGetBodyIds(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return out[0..count];
    }

    pub fn activate(self: Ragdoll, lock_bodies: bool) void {
        c.zjoltRagdollActivate(self.handle, lock_bodies);
    }

    pub fn isActive(self: Ragdoll, lock_bodies: bool) bool {
        return c.zjoltRagdollIsActive(self.handle, lock_bodies);
    }

    /// Clears warm-start impulses on every constraint. Call after `setPose`
    /// so the next step does not solve toward where the bodies used to be.
    pub fn resetWarmStart(self: Ragdoll) void {
        c.zjoltRagdollResetWarmStart(self.handle);
    }

    /// Teleports every body to `pose` (instantly, not activating). `pose`
    /// must have been through `SkeletonPose.calculateJointMatrices` since
    /// its joints were last set, and its skeleton must be the one `self`'s
    /// settings were built with.
    pub fn setPose(self: Ragdoll, pose: SkeletonPose, lock_bodies: bool) err.Error!void {
        try err.check(c.zjoltRagdollSetPose(self.handle, pose.handle, lock_bodies));
    }

    /// Fills `pose`'s joint matrices from the ragdoll's current body
    /// transforms. `pose`'s skeleton must be the ragdoll's. This does NOT
    /// call `SkeletonPose.calculateJointStates` — call that afterwards
    /// before reading `pose` back with `getJoints`.
    pub fn getPose(self: Ragdoll, pose: SkeletonPose, lock_bodies: bool) err.Error!void {
        try err.check(c.zjoltRagdollGetPose(self.handle, pose.handle, lock_bodies));
    }

    /// Drives every body toward `pose` over `delta_time` by setting
    /// velocities rather than teleporting — the right choice for a
    /// kinematic ragdoll being animated rather than simulated. Same
    /// `calculateJointMatrices` and matching-skeleton requirement as
    /// `setPose`.
    pub fn driveToPoseUsingKinematics(
        self: Ragdoll,
        pose: SkeletonPose,
        delta_time: f32,
        lock_bodies: bool,
    ) err.Error!void {
        try err.check(c.zjoltRagdollDriveToPoseUsingKinematics(
            self.handle,
            pose.handle,
            delta_time,
            lock_bodies,
        ));
    }

    /// Drives every constrained body toward `pose` by activating that
    /// joint's constraint motor in position mode. Reads `pose`'s LOCAL joint
    /// rotations directly (`SkeletonPose.setJoints` is enough; unlike
    /// `setPose` this does not need `calculateJointMatrices` first). `pose`'s
    /// skeleton must be the ragdoll's.
    pub fn driveToPoseUsingMotors(self: Ragdoll, pose: SkeletonPose) err.Error!void {
        try err.check(c.zjoltRagdollDriveToPoseUsingMotors(self.handle, pose.handle));
    }

    /// As `driveToPoseUsingMotors`, but also drives each motor's target
    /// velocity — computed from how `pose` differs from `prev_pose` over
    /// `delta_time` — for a smoother follow. Both poses' skeletons must be
    /// the ragdoll's; `delta_time` must be positive.
    pub fn driveToPoseUsingMotorsWithVelocity(
        self: Ragdoll,
        prev_pose: SkeletonPose,
        pose: SkeletonPose,
        delta_time: f32,
    ) err.Error!void {
        try err.check(c.zjoltRagdollDriveToPoseUsingMotorsWithVelocity(
            self.handle,
            prev_pose.handle,
            pose.handle,
            delta_time,
        ));
    }
};
