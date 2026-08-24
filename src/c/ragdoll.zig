//! ZJolt C declarations for skeletons, poses, and ragdoll instances.
//!
//! Mirrors `ffi/zjolt_ragdoll.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const body = @import("body.zig");
const constraint = @import("constraint.zig");
const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const BodyDesc = body.BodyDesc;
pub const zjoltBodyDescInit = body.zjoltBodyDescInit;
pub const SwingType = constraint.SwingType;
pub const Activation = core.Activation;
pub const BodyId = core.BodyId;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Vec3 = core.Vec3;

pub const Skeleton = opaque {};

pub const SkeletonPose = opaque {};

pub const RagdollSettings = opaque {};

pub const Ragdoll = opaque {};

pub const RagdollConstraintDesc = extern struct {
    position1: RVec3,
    twist_axis1: Vec3,
    plane_axis1: Vec3,
    position2: RVec3,
    twist_axis2: Vec3,
    plane_axis2: Vec3,
    swing_type: SwingType,
    normal_half_cone_angle: f32,
    plane_half_cone_angle: f32,
    twist_min_angle: f32,
    twist_max_angle: f32,
    max_friction_torque: f32,
};

pub const RagdollPartDesc = extern struct {
    body: BodyDesc,
    to_parent: ?*const RagdollConstraintDesc,
};

pub extern fn zjoltSkeletonCreate(out: **Skeleton) Result;

pub extern fn zjoltSkeletonAddRef(skeleton: *const Skeleton) void;

pub extern fn zjoltSkeletonRelease(skeleton: *const Skeleton) void;

pub extern fn zjoltSkeletonGetRefCount(skeleton: *const Skeleton) u32;

pub extern fn zjoltSkeletonAddJoint(skeleton: *Skeleton, name: ?[*:0]const u8, parent_index: i32, out_index: *u32) Result;

pub extern fn zjoltSkeletonGetJointCount(skeleton: *const Skeleton) u32;

pub extern fn zjoltSkeletonGetJointIndex(skeleton: *const Skeleton, name: ?[*:0]const u8) i32;

pub extern fn zjoltSkeletonGetJointParentIndex(skeleton: *const Skeleton, index: u32) i32;

pub extern fn zjoltSkeletonGetJointName(skeleton: *const Skeleton, index: u32) [*:0]const u8;

pub extern fn zjoltSkeletonAreJointsCorrectlyOrdered(skeleton: *const Skeleton) bool;

pub extern fn zjoltSkeletonPoseCreate(out: **SkeletonPose) Result;

pub extern fn zjoltSkeletonPoseDestroy(pose: *SkeletonPose) void;

pub extern fn zjoltSkeletonPoseSetSkeleton(pose: *SkeletonPose, skeleton: *const Skeleton) Result;

pub extern fn zjoltSkeletonPoseGetSkeleton(pose: *const SkeletonPose) ?*const Skeleton;

pub extern fn zjoltSkeletonPoseGetJointCount(pose: *const SkeletonPose) u32;

pub extern fn zjoltSkeletonPoseSetRootOffset(pose: *SkeletonPose, offset: *const RVec3) void;

pub extern fn zjoltSkeletonPoseGetRootOffset(pose: *const SkeletonPose, out: *RVec3) void;

pub extern fn zjoltSkeletonPoseSetJoints(pose: *SkeletonPose, rotations: ?[*]const Quat, translations: ?[*]const Vec3, count: u32) Result;

pub extern fn zjoltSkeletonPoseGetJoints(pose: *const SkeletonPose, out_rotations: ?[*]Quat, out_translations: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltSkeletonPoseCalculateJointMatrices(pose: *SkeletonPose) Result;

pub extern fn zjoltSkeletonPoseCalculateJointStates(pose: *SkeletonPose) Result;

pub extern fn zjoltRagdollSettingsCreate(out: **RagdollSettings) Result;

pub extern fn zjoltRagdollSettingsAddRef(settings: *const RagdollSettings) void;

pub extern fn zjoltRagdollSettingsRelease(settings: *const RagdollSettings) void;

pub extern fn zjoltRagdollSettingsGetRefCount(settings: *const RagdollSettings) u32;

pub extern fn zjoltRagdollSettingsBuild(settings: *RagdollSettings, skeleton: *const Skeleton, parts: [*]const RagdollPartDesc, part_count: u32) Result;

pub extern fn zjoltRagdollSettingsStabilize(settings: *RagdollSettings) bool;

pub extern fn zjoltRagdollSettingsDisableParentChildCollisions(settings: *RagdollSettings) void;

pub extern fn zjoltRagdollSettingsCalculateBodyIndexToConstraintIndex(settings: *RagdollSettings) void;

pub extern fn zjoltRagdollSettingsCreateRagdoll(settings: *const RagdollSettings, system: *PhysicsSystem, collision_group: u32, user_data: u64, out: **Ragdoll) Result;

pub extern fn zjoltRagdollAddRef(ragdoll: *const Ragdoll) void;

pub extern fn zjoltRagdollRelease(ragdoll: *const Ragdoll) void;

pub extern fn zjoltRagdollGetRefCount(ragdoll: *const Ragdoll) u32;

pub extern fn zjoltRagdollAddToPhysicsSystem(ragdoll: *Ragdoll, activation: Activation, lock_bodies: bool) void;

pub extern fn zjoltRagdollRemoveFromPhysicsSystem(ragdoll: *Ragdoll, lock_bodies: bool) void;

pub extern fn zjoltRagdollGetBodyIds(ragdoll: *const Ragdoll, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltRagdollActivate(ragdoll: *Ragdoll, lock_bodies: bool) void;

pub extern fn zjoltRagdollIsActive(ragdoll: *const Ragdoll, lock_bodies: bool) bool;

pub extern fn zjoltRagdollResetWarmStart(ragdoll: *Ragdoll) void;

pub extern fn zjoltRagdollSetPose(ragdoll: *Ragdoll, pose: *const SkeletonPose, lock_bodies: bool) Result;

pub extern fn zjoltRagdollGetPose(ragdoll: *Ragdoll, pose: *SkeletonPose, lock_bodies: bool) Result;

pub extern fn zjoltRagdollDriveToPoseUsingKinematics(ragdoll: *Ragdoll, pose: *const SkeletonPose, delta_time: f32, lock_bodies: bool) Result;

pub extern fn zjoltRagdollDriveToPoseUsingMotors(ragdoll: *Ragdoll, pose: *const SkeletonPose) Result;

pub extern fn zjoltRagdollDriveToPoseUsingMotorsWithVelocity(ragdoll: *Ragdoll, prev_pose: *const SkeletonPose, pose: *const SkeletonPose, delta_time: f32) Result;
