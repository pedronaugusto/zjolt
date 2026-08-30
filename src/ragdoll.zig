//! Ragdolls: a skeleton, per-part bodies and constraints, driven or read
//! back as a pose.
//!
//! Built up in order: `Skeleton` is the joint hierarchy (names/parent
//! indices only; reference counted). `RagdollSettings` is one
//! body-and-constraint part per joint — a reusable template a live
//! `Ragdoll` keeps referencing and consults on every
//! `driveToPoseUsingMotors` call. `SkeletonPose` is one pose instance
//! (root offset plus per-joint local rotation/translation; NOT reference
//! counted) — convert to/from its internal matrices with
//! `calculateJointMatrices`/`calculateJointStates`.
//!
//! `SkeletonMapper`, the fourth piece, maps a low-joint ragdoll pose onto
//! a higher-joint mesh skeleton and back. Reference counted.

const std = @import("std");
const c = @import("c/ragdoll.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const constraint_mod = @import("constraint.zig");
const system_mod = @import("system.zig");
const stream_mod = @import("stream.zig");

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

    /// The skeleton assigned via `setSkeleton`. Borrowed as a mutable
    /// handle — legal to mutate; it is the same object the owner's
    /// reference points at, since `Skeleton`'s own methods mutate the
    /// joint list. `null` if none is assigned.
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

    /// The joint matrices `calculateJointMatrices` computes: one MODEL-space
    /// transform per joint, sixteen floats each, column-major — column c's
    /// row r is `out[16 * joint + 4 * c + r]`. `out` holds
    /// `16 * jointCount()` floats, or `error.BufferTooSmall`.
    pub fn getJointMatrices(self: SkeletonPose, out: []f32) err.Error![]f32 {
        var count: u32 = 0;
        try err.check(c.zjoltSkeletonPoseGetJointMatrices(
            self.handle,
            out.ptr,
            @intCast(out.len / 16),
            &count,
        ));
        return out[0 .. 16 * count];
    }

    /// The same array, written. `matrices` holds exactly `16 * jointCount()`
    /// floats. The way in for a pose produced outside Jolt — a skinning
    /// solver, an animation runtime — without per-joint local transforms;
    /// `calculateJointStates` then derives those from these.
    pub fn setJointMatrices(self: SkeletonPose, matrices: []const f32) err.Error!void {
        try err.check(c.zjoltSkeletonPoseSetJointMatrices(
            self.handle,
            matrices.ptr,
            @intCast(matrices.len / 16),
        ));
    }

    /// The reverse conversion: what `Ragdoll.getPose` leaves the caller
    /// needing before `getJoints` returns anything meaningful.
    pub fn calculateJointStates(self: SkeletonPose) err.Error!void {
        try err.check(c.zjoltSkeletonPoseCalculateJointStates(self.handle));
    }
};

//=============================================================================
// SkeletalAnimation
//=============================================================================

/// A skinned animation: keyframed local-space rotation/translation per
/// animated joint, sampled onto a `SkeletonPose`. Reference counted.
pub const SkeletalAnimation = struct {
    handle: *c.SkeletalAnimation,

    /// An empty animation (no animated joints).
    pub fn init() err.Error!SkeletalAnimation {
        var handle: *c.SkeletalAnimation = undefined;
        try err.check(c.zjoltSkeletalAnimationCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn addRef(self: SkeletalAnimation) void {
        c.zjoltSkeletalAnimationAddRef(self.handle);
    }

    pub fn release(self: SkeletalAnimation) void {
        c.zjoltSkeletalAnimationRelease(self.handle);
    }

    pub fn refCount(self: SkeletalAnimation) u32 {
        return c.zjoltSkeletalAnimationGetRefCount(self.handle);
    }

    /// Appends an animated joint, initially with no keyframes, and returns
    /// its index. `name` is what `sample` looks up in the pose's skeleton —
    /// it need not match one yet.
    pub fn addAnimatedJoint(self: SkeletalAnimation, name: ?[:0]const u8) err.Error!u32 {
        var index: u32 = undefined;
        try err.check(c.zjoltSkeletalAnimationAddAnimatedJoint(
            self.handle,
            if (name) |n| n.ptr else null,
            &index,
        ));
        return index;
    }

    pub fn animatedJointCount(self: SkeletalAnimation) u32 {
        return c.zjoltSkeletalAnimationGetAnimatedJointCount(self.handle);
    }

    /// Borrowed; valid until the animation is destroyed. Empty if
    /// `joint_index` is out of range.
    pub fn animatedJointName(self: SkeletalAnimation, joint_index: u32) [:0]const u8 {
        return std.mem.span(c.zjoltSkeletalAnimationGetAnimatedJointName(self.handle, joint_index));
    }

    /// Appends a keyframe to animated joint `joint_index`. Keyframes must be
    /// added in non-decreasing `time` order: `sample`'s binary search over
    /// them assumes it, and Jolt does not check.
    pub fn addKeyframe(
        self: SkeletalAnimation,
        joint_index: u32,
        time: f32,
        rotation: math.Quat,
        translation: math.Vec3,
    ) err.Error!void {
        try err.check(c.zjoltSkeletalAnimationAddKeyframe(
            self.handle,
            joint_index,
            time,
            &rotation,
            &translation,
        ));
    }

    pub fn keyframeCount(self: SkeletalAnimation, joint_index: u32) u32 {
        return c.zjoltSkeletalAnimationGetKeyframeCount(self.handle, joint_index);
    }

    pub const Keyframe = struct {
        time: f32,
        rotation: math.Quat,
        translation: math.Vec3,
    };

    /// Fails if `joint_index` or `keyframe_index` is out of range.
    pub fn keyframe(self: SkeletalAnimation, joint_index: u32, keyframe_index: u32) err.Error!Keyframe {
        var out: Keyframe = undefined;
        try err.check(c.zjoltSkeletalAnimationGetKeyframe(
            self.handle,
            joint_index,
            keyframe_index,
            &out.time,
            &out.rotation,
            &out.translation,
        ));
        return out;
    }

    /// The time (seconds) of the last keyframe of the FIRST animated joint.
    /// 0 if there are no animated joints, or that joint has no keyframes.
    pub fn duration(self: SkeletalAnimation) f32 {
        return c.zjoltSkeletalAnimationGetDuration(self.handle);
    }

    /// Scales every keyframe's translation by `scale`.
    pub fn scaleJoints(self: SkeletalAnimation, scale: f32) void {
        c.zjoltSkeletalAnimationScaleJoints(self.handle, scale);
    }

    /// If the animation loops, `sample` wraps `time` modulo `duration`
    /// instead of clamping to the last keyframe.
    pub fn setIsLooping(self: SkeletalAnimation, is_looping: bool) void {
        c.zjoltSkeletalAnimationSetIsLooping(self.handle, is_looping);
    }

    pub fn isLooping(self: SkeletalAnimation) bool {
        return c.zjoltSkeletalAnimationIsLooping(self.handle);
    }

    /// Samples the (interpolated) joint transforms at `time` onto `pose`'s
    /// LOCAL joint rotations/translations — `SkeletonPose.setJoints`'s form;
    /// no `calculateJointMatrices` needed first.
    ///
    /// `time` must not be negative. Every animated joint's name must be a
    /// joint of `pose`'s assigned skeleton.
    pub fn sample(self: SkeletalAnimation, time: f32, pose: SkeletonPose) err.Error!void {
        try err.check(c.zjoltSkeletalAnimationSample(self.handle, time, pose.handle));
    }

    /// Writes `self` through `stream` in Jolt's own binary form.
    pub fn saveBinaryState(self: SkeletalAnimation, stream: stream_mod.Stream) err.Error!void {
        try err.check(c.zjoltSkeletalAnimationSaveBinaryState(self.handle, &stream));
    }

    /// Reads an animation written by `saveBinaryState`.
    pub fn restoreBinaryState(stream: stream_mod.Stream) err.Error!SkeletalAnimation {
        var handle: *c.SkeletalAnimation = undefined;
        try err.check(c.zjoltSkeletalAnimationRestoreBinaryState(&stream, &handle));
        return .{ .handle = handle };
    }

    /// The rotation/translation pair a keyframe carries, and the matrix
    /// conversion `sample` uses internally.
    pub const JointState = struct {
        rotation: math.Quat,
        translation: math.Vec3,

        /// Decomposes a local-space matrix into a rotation and translation.
        pub fn fromMatrix(matrix: math.Mat44) JointState {
            var out: JointState = undefined;
            c.zjoltSkeletalAnimationJointStateFromMatrix(&matrix, &out.rotation, &out.translation);
            return out;
        }

        /// The inverse.
        pub fn toMatrix(self: JointState) math.Mat44 {
            var out: math.Mat44 = math.mat44_identity;
            c.zjoltSkeletalAnimationJointStateToMatrix(&self.rotation, &self.translation, &out);
            return out;
        }
    };
};

//=============================================================================
// SkeletonMapper
//=============================================================================

/// One skeleton drives another: the low-detail skeleton a ragdoll simulates,
/// onto the high-detail skeleton a mesh is skinned to, and back.
///
/// Order matters — wrong order maps to nonsense, not an error: pose both
/// skeletons NEUTRAL and `calculateJointMatrices`, THEN `initialize`
/// (low-detail first). Reference counted, with no reference to either skeleton.
pub const SkeletonMapper = struct {
    handle: *c.SkeletonMapper,

    /// Whether joint `index1` of `skeleton1` is the same joint as `index2` of
    /// `skeleton2`. See `initialize`.
    pub const CanMapJointFn = c.SkeletonMapperCanMapJointFn;

    /// An uninitialised mapper — call `initialize` next.
    pub fn init() err.Error!SkeletonMapper {
        var handle: *c.SkeletonMapper = undefined;
        try err.check(c.zjoltSkeletonMapperCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn addRef(self: SkeletonMapper) void {
        c.zjoltSkeletonMapperAddRef(self.handle);
    }

    pub fn release(self: SkeletonMapper) void {
        c.zjoltSkeletonMapperRelease(self.handle);
    }

    pub fn refCount(self: SkeletonMapper) u32 {
        return c.zjoltSkeletonMapperGetRefCount(self.handle);
    }

    /// Works out which joints of the two skeletons correspond, from two
    /// neutral poses (MODEL space, via `calculateJointMatrices`). Every
    /// other method depends on this having run. `neutral1` must be the
    /// LOW-detail skeleton's pose — reversed with equal joint counts is
    /// not refused, and maps wrong. `can_map_joint` (`null` for
    /// name-equality) must not panic: it runs where Jolt cannot unwind.
    pub fn initialize(
        self: SkeletonMapper,
        neutral1: SkeletonPose,
        neutral2: SkeletonPose,
        can_map_joint: ?CanMapJointFn,
        user: ?*anyopaque,
    ) err.Error!void {
        try err.check(c.zjoltSkeletonMapperInitialize(
            self.handle,
            neutral1.handle,
            neutral2.handle,
            can_map_joint,
            user,
        ));
    }

    /// How many joints of skeleton 1 matched a joint of skeleton 2. 0 before
    /// `initialize`, and 0 after it if nothing matched — worth checking,
    /// because that is not an error and `map` will simply do nothing.
    pub fn mappingCount(self: SkeletonMapper) u32 {
        return c.zjoltSkeletonMapperGetMappingCount(self.handle);
    }

    /// The joint of skeleton 2 that joint `joint1_index` of skeleton 1 drives,
    /// or `null` if it was not matched.
    pub fn mappedJointIndex(self: SkeletonMapper, joint1_index: u32) ?u32 {
        const index = c.zjoltSkeletonMapperGetMappedJointIndex(self.handle, joint1_index);
        return if (index < 0) null else @intCast(index);
    }

    /// How many joint chains the mapper found between two 1-on-1 mapped
    /// joints — runs of joints in skeleton 2 that skeleton 1 has no
    /// equivalent for.
    pub fn chainCount(self: SkeletonMapper) u32 {
        return c.zjoltSkeletonMapperGetChainCount(self.handle);
    }

    /// The length of chain `chain_index`'s two joint-index runs: skeleton
    /// 1's and skeleton 2's. Both 0 if `chain_index` is past the end.
    pub fn chainJointCounts(self: SkeletonMapper, chain_index: u32) struct { count1: u32, count2: u32 } {
        var count1: u32 = undefined;
        var count2: u32 = undefined;
        c.zjoltSkeletonMapperGetChainJointCounts(self.handle, chain_index, &count1, &count2);
        return .{ .count1 = count1, .count2 = count2 };
    }

    /// One joint index of chain `chain_index`'s skeleton-1 run. `null` if
    /// `chain_index` or `index` is out of range.
    pub fn chainJointIndex1(self: SkeletonMapper, chain_index: u32, index: u32) ?u32 {
        const i = c.zjoltSkeletonMapperGetChainJointIndex1(self.handle, chain_index, index);
        return if (i < 0) null else @intCast(i);
    }

    /// One joint index of chain `chain_index`'s skeleton-2 run. `null` if
    /// `chain_index` or `index` is out of range.
    pub fn chainJointIndex2(self: SkeletonMapper, chain_index: u32, index: u32) ?u32 {
        const i = c.zjoltSkeletonMapperGetChainJointIndex2(self.handle, chain_index, index);
        return if (i < 0) null else @intCast(i);
    }

    /// How many joints of skeleton 2 could not be mapped to skeleton 1 at
    /// all.
    pub fn unmappedCount(self: SkeletonMapper) u32 {
        return c.zjoltSkeletonMapperGetUnmappedCount(self.handle);
    }

    pub const Unmapped = struct {
        /// Its own index, in skeleton 2.
        joint_index: u32,
        /// Its parent's index, in skeleton 2. `null` if it has none.
        parent_joint_index: ?u32,
    };

    /// One unmapped joint of skeleton 2. Fails if `index` is out of range.
    pub fn unmapped(self: SkeletonMapper, index: u32) err.Error!Unmapped {
        var raw: c.SkeletonMapperUnmapped = undefined;
        try err.check(c.zjoltSkeletonMapperGetUnmapped(self.handle, index, &raw));
        return .{
            .joint_index = @intCast(raw.joint_index),
            .parent_joint_index = if (raw.parent_joint_index < 0) null else @intCast(raw.parent_joint_index),
        };
    }

    /// Pins the translation of the named joints of skeleton 2 to their
    /// neutral pose, so only their rotation follows the ragdoll — removes
    /// mesh stretch at the cost of the drawn skeleton not exactly matching
    /// the simulated one. `locked` has one entry per joint of `neutral2`
    /// (the same pose `initialize` used); a joint with NO PARENT is
    /// refused. Calls accumulate; there is no unlock.
    pub fn lockTranslations(
        self: SkeletonMapper,
        neutral2: SkeletonPose,
        locked: []const bool,
    ) err.Error!void {
        try err.check(c.zjoltSkeletonMapperLockTranslations(
            self.handle,
            neutral2.handle,
            locked.ptr,
            @intCast(locked.len),
        ));
    }

    /// `lockTranslations` for every joint below the topmost mapped joint,
    /// that joint itself excluded. The usual choice. Requires `initialize`
    /// to have run.
    pub fn lockAllTranslations(self: SkeletonMapper, neutral2: SkeletonPose) err.Error!void {
        try err.check(c.zjoltSkeletonMapperLockAllTranslations(self.handle, neutral2.handle));
    }

    /// Whether `lockTranslations` or `lockAllTranslations` locked joint
    /// `joint2_index` of skeleton 2.
    pub fn isJointTranslationLocked(self: SkeletonMapper, joint2_index: u32) bool {
        return c.zjoltSkeletonMapperIsJointTranslationLocked(self.handle, joint2_index);
    }

    /// Pushes `pose1`'s MODEL-space joint matrices onto `pose2` (combined
    /// with `pose2`'s own LOCAL joints for anything the ragdoll does not
    /// drive) and writes `pose2`'s MODEL-space matrices — read back via
    /// `calculateJointStates`/`getJoints`. Unlike Jolt's own Map, `pose1`'s
    /// ROOT OFFSET is copied onto `pose2` too (a pose's world position
    /// lives there, not its matrices).
    pub fn map(self: SkeletonMapper, pose1: SkeletonPose, pose2: SkeletonPose) err.Error!void {
        try err.check(c.zjoltSkeletonMapperMap(self.handle, pose1.handle, pose2.handle));
    }

    /// The other direction: reads `pose2`'s MODEL-space matrices, writes
    /// `pose1`'s — sample animation into `pose2`, `calculateJointMatrices`,
    /// map back, then `Ragdoll.setPose`/`driveToPoseUsingKinematics` with
    /// `pose1` (`pose2`'s root offset travels onto `pose1` too). Only
    /// one-to-one mapped joints are written; an unmapped `pose1` joint
    /// keeps whatever (possibly uninitialised) matrix it already held.
    pub fn mapReverse(self: SkeletonMapper, pose2: SkeletonPose, pose1: SkeletonPose) err.Error!void {
        try err.check(c.zjoltSkeletonMapperMapReverse(self.handle, pose2.handle, pose1.handle));
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

/// The swing-twist constraint linking a part to its parent, in WORLD space —
/// matching `body.position`/`body.rotation`. Jolt's constraint for humanoid
/// ragdolls, the only non-hinge type `Ragdoll.driveToPoseUsingMotors` drives.
///
/// Motor tuning is not exposed: driving uses Jolt's default `MotorSettings`
/// (critically-damped spring, unlimited torque).
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

/// One of `RagdollSettings.addAdditionalConstraint`'s entries, read back.
pub const AdditionalConstraint = struct {
    part_index1: u32,
    part_index2: u32,
    settings: ?constraint_mod.ConstraintSettings,
};

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

    //=========================================================================
    // Jolt's own object stream
    //
    // Same feature as `Scene.saveObjectStream`, on a different registered
    // type, including the same loss (a part's SHAPE does not survive). `error.Unsupported` unless built with `-Dobject_stream=true`.
    //=========================================================================

    /// Writes `self` through `stream` in Jolt's own object-stream format.
    /// `error.IoError` if `stream` reports failure while this runs. @see the
    /// section comment above for what a part loses on the way through.
    pub fn saveObjectStream(
        self: RagdollSettings,
        format: stream_mod.ObjectStreamFormat,
        stream: stream_mod.Stream,
    ) err.Error!void {
        try err.check(c.zjoltRagdollSettingsSaveObjectStream(self.handle, format, &stream));
    }

    /// Reads settings written by `saveObjectStream`, or by a C++ host of
    /// vanilla Jolt through `ObjectStreamOut::sWriteObject<RagdollSettings>`
    /// — either form, sniffed from the stream. Not run through `build`,
    /// but already carries the described skeleton and parts the same way
    /// `build` would have; no part has a shape. @see the section comment above.
    pub fn restoreObjectStream(stream: stream_mod.Stream) err.Error!RagdollSettings {
        var handle: *c.RagdollSettings = undefined;
        try err.check(c.zjoltRagdollSettingsRestoreObjectStream(&stream, &handle));
        return .{ .handle = handle };
    }

    /// Assigns `new_skeleton` and builds one part per joint from `parts`
    /// (exactly `new_skeleton.jointCount()` entries, in joint-index order).
    /// Overwrites whatever `self` held before. `allocator` is used only
    /// for the duration of this call; nothing it allocates is kept after.
    pub fn build(
        self: RagdollSettings,
        allocator: std.mem.Allocator,
        new_skeleton: Skeleton,
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
            new_skeleton.handle,
            c_parts.ptr,
            @intCast(c_parts.len),
        ));
    }

    /// Replaces part `part_index`'s joint to its parent with ANY two-body
    /// constraint, not just the swing-twist `PartDesc.to_parent` describes.
    /// `null` removes it, which is what the root part has.
    /// `error.InvalidArgument` for a settings object that is not a two-body
    /// kind. A reference is taken; the caller keeps its own. Call after
    /// `build`, which is what sizes the part list.
    pub fn setPartConstraint(
        self: RagdollSettings,
        part_index: u32,
        constraint_settings: ?constraint_mod.ConstraintSettings,
    ) err.Error!void {
        try err.check(c.zjoltRagdollSettingsSetPartConstraint(
            self.handle,
            part_index,
            if (constraint_settings) |cs| cs.handle else null,
        ));
    }

    /// Part `part_index`'s joint to its parent, with a reference taken —
    /// `deinit` it. `null` for a part with no joint, which is not an error.
    pub fn partConstraint(
        self: RagdollSettings,
        part_index: u32,
    ) err.Error!?constraint_mod.ConstraintSettings {
        var handle: ?*c.ConstraintSettings = null;
        try err.check(c.zjoltRagdollSettingsGetPartConstraint(
            self.handle,
            part_index,
            &handle,
        ));
        return if (handle) |h| .{ .handle = h } else null;
    }

    /// A constraint between two parts that are NOT parent and child — Jolt's
    /// `mAdditionalConstraints`. Same two-body requirement as
    /// `setPartConstraint`, and a reference is taken.
    pub fn addAdditionalConstraint(
        self: RagdollSettings,
        part_index1: u32,
        part_index2: u32,
        constraint_settings: constraint_mod.ConstraintSettings,
    ) err.Error!void {
        try err.check(c.zjoltRagdollSettingsAddAdditionalConstraint(
            self.handle,
            part_index1,
            part_index2,
            constraint_settings.handle,
        ));
    }

    pub fn additionalConstraintCount(self: RagdollSettings) u32 {
        return c.zjoltRagdollSettingsGetNumAdditionalConstraints(self.handle);
    }

    /// One of them. `.settings` carries a reference — `deinit` it.
    pub fn additionalConstraint(
        self: RagdollSettings,
        index: u32,
    ) err.Error!AdditionalConstraint {
        var part1: u32 = 0;
        var part2: u32 = 0;
        var handle: ?*c.ConstraintSettings = null;
        try err.check(c.zjoltRagdollSettingsGetAdditionalConstraint(
            self.handle,
            index,
            &part1,
            &part2,
            &handle,
        ));
        return .{
            .part_index1 = part1,
            .part_index2 = part2,
            .settings = if (handle) |h| .{ .handle = h } else null,
        };
    }

    /// The skeleton `build` was given. Borrowed — the settings' own reference
    /// keeps it alive, so this does not need releasing. `null` before `build`.
    ///
    /// With `Ragdoll.settings`, the route from a live ragdoll to the joint
    /// names behind the ids `Ragdoll.getBodyIds` hands back.
    pub fn skeleton(self: RagdollSettings) ?Skeleton {
        const handle = c.zjoltRagdollSettingsGetSkeleton(self.handle) orelse return null;
        // The same `@constCast` as `SkeletonPose.skeleton`, for the same
        // reason: Jolt hands back a const pointer to an object a caller is
        // entitled to mutate.
        return .{ .handle = @constCast(handle) };
    }

    /// Gives each part's constraint a solver priority counting UP toward
    /// the root — a joint nearer the root solves before one nearer a
    /// leaf, so a shoulder stops fighting the wrist hanging off it. Call
    /// after `build`, like `stabilize`: only a ragdoll spawned afterwards
    /// carries the priorities. `base_priority` is the LOWEST (leaf-most)
    /// priority; 0 unless sorting against other constraints in the system.
    pub fn calculateConstraintPriorities(
        self: RagdollSettings,
        base_priority: u32,
    ) err.Error!void {
        try err.check(c.zjoltRagdollSettingsCalculateConstraintPriorities(
            self.handle,
            base_priority,
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

    /// The constraint index (into `Ragdoll.getConstraint`) attaching body
    /// index `body_index` (into `Ragdoll.getBodyIds`) to its parent. `null`
    /// if it has none — the root, or the table has not been built.
    pub fn constraintIndexForBodyIndex(self: RagdollSettings, body_index: u32) ?u32 {
        const index = c.zjoltRagdollSettingsGetConstraintIndexForBodyIndex(self.handle, body_index);
        return if (index < 0) null else @intCast(index);
    }

    /// Every body index's answer to `constraintIndexForBodyIndex`: -1 where
    /// that would answer `null`. `out` must have at least the part count
    /// entries; a shorter one reports `error.BufferTooSmall`.
    pub fn bodyIndexToConstraintIndex(self: RagdollSettings, out: []i32) err.Error![]i32 {
        var count: u32 = undefined;
        try err.check(c.zjoltRagdollSettingsGetBodyIndexToConstraintIndex(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return out[0..count];
    }

    /// Builds the constraint-index -> body-index-pair table
    /// `bodyIndicesForConstraintIndex` and `constraintIndexToBodyIdxPair`
    /// need. Call after `build`, like `calculateBodyIndexToConstraintIndex` —
    /// the two tables are independent, so building one does not build the
    /// other.
    pub fn calculateConstraintIndexToBodyIdxPair(self: RagdollSettings) void {
        c.zjoltRagdollSettingsCalculateConstraintIndexToBodyIdxPair(self.handle);
    }

    pub const BodyIndexPair = c.RagdollBodyIndexPair;

    /// A constraint's two body indices, in the same order
    /// `Ragdoll.getBodyIds` reports them. Fails if `constraint_index` is out
    /// of range, including before `calculateConstraintIndexToBodyIdxPair`
    /// has run.
    pub fn bodyIndicesForConstraintIndex(self: RagdollSettings, constraint_index: u32) err.Error!BodyIndexPair {
        var out: BodyIndexPair = undefined;
        try err.check(c.zjoltRagdollSettingsGetBodyIndicesForConstraintIndex(
            self.handle,
            constraint_index,
            &out,
        ));
        return out;
    }

    /// Every constraint index's answer to `bodyIndicesForConstraintIndex`.
    /// `out` must have at least the constraint count entries; a shorter one
    /// reports `error.BufferTooSmall`.
    pub fn constraintIndexToBodyIdxPair(self: RagdollSettings, out: []BodyIndexPair) err.Error![]BodyIndexPair {
        var count: u32 = undefined;
        try err.check(c.zjoltRagdollSettingsGetConstraintIndexToBodyIdxPair(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return out[0..count];
    }

    /// Spawns one body per part and one constraint per non-root part, in
    /// `system` but NOT YET added to its simulation (see
    /// `Ragdoll.addToPhysicsSystem`). `self` is untouched — reusable for more.
    ///
    /// `collision_group`: ragdolls sharing a `disableParentChildCollisions`
    /// filter need DIFFERENT ids — a filter is consulted within one id only.
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
    /// physics system first if still in it (constraints and bodies both,
    /// with body locking), then destroys every body it owns — `defer
    /// ragdoll.release()` is the whole teardown either way.
    ///
    /// `removeFromPhysicsSystem` removes it while keeping it alive.
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

    /// Writes the ragdoll's body ids into `out`, indexed by joint: `out[i]` is
    /// the body from skeleton joint `i`. `out` needs at least `bodyCount()`
    /// entries; a shorter one reports `error.BufferTooSmall`, writing nothing.
    ///
    /// Ids stay valid until the ragdoll's last reference drops — do not destroy
    /// one through `BodyInterface`.
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

    /// The settings this ragdoll was spawned from. Borrowed — the ragdoll's
    /// own reference keeps them alive for as long as it does, so this does not
    /// need releasing.
    pub fn settings(self: Ragdoll) RagdollSettings {
        // Cannot be null: a Ragdoll handle only ever comes from
        // `RagdollSettings.createRagdoll`, which sets the settings before it
        // hands one back.
        const handle = c.zjoltRagdollGetRagdollSettings(self.handle).?;
        return .{ .handle = @constCast(handle) };
    }

    /// How many constraints the ragdoll owns: one per part with a parent, so
    /// one fewer than `bodyCount()` for a single-rooted skeleton.
    pub fn constraintCount(self: Ragdoll) u32 {
        return c.zjoltRagdollGetConstraintCount(self.handle);
    }

    /// One of them, `null` past the end. Borrowed, owned by the ragdoll:
    /// do not `release` or remove it by hand (`removeFromPhysicsSystem`
    /// moves the whole ragdoll as one batch).
    ///
    /// Joint-index order, skipping the root: constraint `i` attaches the
    /// `i`-th non-root joint to its parent. Always a swing-twist constraint.
    pub fn getConstraint(self: Ragdoll, index: u32) ?constraint_mod.Constraint {
        const handle = c.zjoltRagdollGetConstraint(self.handle, index) orelse return null;
        return .{ .handle = handle };
    }

    /// Where the ragdoll is: the world transform of the body built from
    /// the skeleton's first joint, without reading every part back.
    ///
    /// `error.InvalidArgument` for a ragdoll with no parts. If the body
    /// lock fails, outputs are zeroed/identity but the call still succeeds.
    pub fn rootTransform(self: Ragdoll, lock_bodies: bool) err.Error!struct {
        position: math.RVec3,
        rotation: math.Quat,
    } {
        var position: math.RVec3 = undefined;
        var rotation: math.Quat = undefined;
        try err.check(c.zjoltRagdollGetRootTransform(
            self.handle,
            &position,
            &rotation,
            lock_bodies,
        ));
        return .{ .position = position, .rotation = rotation };
    }

    /// Moves every part into collision group `group_id` (keeping sub-group
    /// id and filter), changing what `createRagdoll` was given.
    ///
    /// Ragdolls sharing a `disableParentChildCollisions` filter must not
    /// hold the same id: a filter is consulted only within one group id,
    /// so sharing lets every part of one ragdoll pass through the other's.
    pub fn setGroupId(self: Ragdoll, group_id: u32, lock_bodies: bool) void {
        c.zjoltRagdollSetGroupId(self.handle, group_id, lock_bodies);
    }

    pub fn activate(self: Ragdoll, lock_bodies: bool) void {
        c.zjoltRagdollActivate(self.handle, lock_bodies);
    }

    /// Every body of the ragdoll at once — what makes it launch, carry a
    /// runner's momentum into the fall, or take a hit as one thing rather
    /// than as a list of parts the caller walks.
    pub fn setLinearAndAngularVelocity(
        self: Ragdoll,
        linear: math.Vec3,
        angular: math.Vec3,
        lock_bodies: bool,
    ) err.Error!void {
        try err.check(c.zjoltRagdollSetLinearAndAngularVelocity(
            self.handle,
            &linear,
            &angular,
            lock_bodies,
        ));
    }

    pub fn setLinearVelocity(
        self: Ragdoll,
        linear: math.Vec3,
        lock_bodies: bool,
    ) err.Error!void {
        try err.check(c.zjoltRagdollSetLinearVelocity(self.handle, &linear, lock_bodies));
    }

    pub fn addLinearVelocity(
        self: Ragdoll,
        linear: math.Vec3,
        lock_bodies: bool,
    ) err.Error!void {
        try err.check(c.zjoltRagdollAddLinearVelocity(self.handle, &linear, lock_bodies));
    }

    /// Adds `impulse` at the centre of mass of EVERY body — the SAME
    /// impulse once per body, not one shared across the ragdoll, so a
    /// heavier part gains less velocity. Jolt's own AddImpulse.
    pub fn addImpulse(
        self: Ragdoll,
        impulse: math.Vec3,
        lock_bodies: bool,
    ) err.Error!void {
        try err.check(c.zjoltRagdollAddImpulse(self.handle, &impulse, lock_bodies));
    }

    pub fn isActive(self: Ragdoll, lock_bodies: bool) bool {
        return c.zjoltRagdollIsActive(self.handle, lock_bodies);
    }

    /// Clears warm-start impulses on every constraint. Call after `setPose`
    /// so the next step does not solve toward the bodies' prior position.
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
