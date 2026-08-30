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
pub const Constraint = constraint.Constraint;
pub const ConstraintSettings = constraint.ConstraintSettings;
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

pub const SkeletalAnimation = opaque {};

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

pub extern fn zjoltSkeletonPoseGetJointMatrices(pose: *const SkeletonPose, out_matrices: ?[*]f32, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltSkeletonPoseSetJointMatrices(pose: *SkeletonPose, matrices: [*]const f32, count: u32) Result;

pub extern fn zjoltSkeletonPoseCalculateJointStates(pose: *SkeletonPose) Result;

pub extern fn zjoltSkeletalAnimationCreate(out: **SkeletalAnimation) Result;

pub extern fn zjoltSkeletalAnimationAddRef(animation: *const SkeletalAnimation) void;

pub extern fn zjoltSkeletalAnimationRelease(animation: *const SkeletalAnimation) void;

pub extern fn zjoltSkeletalAnimationGetRefCount(animation: *const SkeletalAnimation) u32;

pub extern fn zjoltSkeletalAnimationAddAnimatedJoint(animation: *SkeletalAnimation, name: ?[*:0]const u8, out_index: *u32) Result;

pub extern fn zjoltSkeletalAnimationGetAnimatedJointCount(animation: *const SkeletalAnimation) u32;

pub extern fn zjoltSkeletalAnimationGetAnimatedJointName(animation: *const SkeletalAnimation, joint_index: u32) [*:0]const u8;

pub extern fn zjoltSkeletalAnimationAddKeyframe(animation: *SkeletalAnimation, joint_index: u32, time: f32, rotation: *const Quat, translation: *const Vec3) Result;

pub extern fn zjoltSkeletalAnimationGetKeyframeCount(animation: *const SkeletalAnimation, joint_index: u32) u32;

pub extern fn zjoltSkeletalAnimationGetKeyframe(animation: *const SkeletalAnimation, joint_index: u32, keyframe_index: u32, out_time: *f32, out_rotation: *Quat, out_translation: *Vec3) Result;

pub extern fn zjoltSkeletalAnimationGetDuration(animation: *const SkeletalAnimation) f32;

pub extern fn zjoltSkeletalAnimationScaleJoints(animation: *SkeletalAnimation, scale: f32) void;

pub extern fn zjoltSkeletalAnimationSetIsLooping(animation: *SkeletalAnimation, is_looping: bool) void;

pub extern fn zjoltSkeletalAnimationIsLooping(animation: *const SkeletalAnimation) bool;

pub extern fn zjoltSkeletalAnimationSample(animation: *const SkeletalAnimation, time: f32, pose: *SkeletonPose) Result;

pub extern fn zjoltSkeletalAnimationSaveBinaryState(animation: *const SkeletalAnimation, stream: *const core.Stream) Result;

pub extern fn zjoltSkeletalAnimationRestoreBinaryState(stream: *const core.Stream, out: **SkeletalAnimation) Result;

pub extern fn zjoltSkeletalAnimationJointStateFromMatrix(matrix: *const core.Mat44, out_rotation: *Quat, out_translation: *Vec3) void;

pub extern fn zjoltSkeletalAnimationJointStateToMatrix(rotation: *const Quat, translation: *const Vec3, out: *core.Mat44) void;

pub const SkeletonMapper = opaque {};

pub const SkeletonMapperCanMapJointFn = *const fn (user: ?*anyopaque, skeleton1: *const Skeleton, index1: u32, skeleton2: *const Skeleton, index2: u32) callconv(.c) bool;

pub extern fn zjoltSkeletonMapperCreate(out: **SkeletonMapper) Result;

pub extern fn zjoltSkeletonMapperAddRef(mapper: *const SkeletonMapper) void;

pub extern fn zjoltSkeletonMapperRelease(mapper: *const SkeletonMapper) void;

pub extern fn zjoltSkeletonMapperGetRefCount(mapper: *const SkeletonMapper) u32;

pub extern fn zjoltSkeletonMapperInitialize(mapper: *SkeletonMapper, neutral1: *const SkeletonPose, neutral2: *const SkeletonPose, can_map_joint: ?SkeletonMapperCanMapJointFn, user: ?*anyopaque) Result;

pub extern fn zjoltSkeletonMapperGetMappingCount(mapper: *const SkeletonMapper) u32;

pub extern fn zjoltSkeletonMapperGetMappedJointIndex(mapper: *const SkeletonMapper, joint1_index: u32) i32;

pub extern fn zjoltSkeletonMapperGetChainCount(mapper: *const SkeletonMapper) u32;

pub extern fn zjoltSkeletonMapperGetChainJointCounts(mapper: *const SkeletonMapper, chain_index: u32, out_count1: *u32, out_count2: *u32) void;

pub extern fn zjoltSkeletonMapperGetChainJointIndex1(mapper: *const SkeletonMapper, chain_index: u32, index: u32) i32;

pub extern fn zjoltSkeletonMapperGetChainJointIndex2(mapper: *const SkeletonMapper, chain_index: u32, index: u32) i32;

pub extern fn zjoltSkeletonMapperGetUnmappedCount(mapper: *const SkeletonMapper) u32;

pub const SkeletonMapperUnmapped = extern struct {
    joint_index: i32,
    parent_joint_index: i32,
};

pub extern fn zjoltSkeletonMapperGetUnmapped(mapper: *const SkeletonMapper, index: u32, out: *SkeletonMapperUnmapped) Result;

pub extern fn zjoltSkeletonMapperLockTranslations(mapper: *SkeletonMapper, neutral2: *const SkeletonPose, locked: [*]const bool, count: u32) Result;

pub extern fn zjoltSkeletonMapperLockAllTranslations(mapper: *SkeletonMapper, neutral2: *const SkeletonPose) Result;

pub extern fn zjoltSkeletonMapperIsJointTranslationLocked(mapper: *const SkeletonMapper, joint2_index: u32) bool;

pub extern fn zjoltSkeletonMapperMap(mapper: *const SkeletonMapper, pose1: *const SkeletonPose, pose2: *SkeletonPose) Result;

pub extern fn zjoltSkeletonMapperMapReverse(mapper: *const SkeletonMapper, pose2: *const SkeletonPose, pose1: *SkeletonPose) Result;

pub extern fn zjoltRagdollSettingsCreate(out: **RagdollSettings) Result;

pub extern fn zjoltRagdollSettingsAddRef(settings: *const RagdollSettings) void;

pub extern fn zjoltRagdollSettingsRelease(settings: *const RagdollSettings) void;

pub extern fn zjoltRagdollSettingsGetRefCount(settings: *const RagdollSettings) u32;

pub extern fn zjoltRagdollSettingsSaveObjectStream(settings: *const RagdollSettings, format: core.ObjectStreamFormat, stream: *const core.Stream) Result;

pub extern fn zjoltRagdollSettingsRestoreObjectStream(stream: *const core.Stream, out: **RagdollSettings) Result;

pub extern fn zjoltRagdollSettingsBuild(settings: *RagdollSettings, skeleton: *const Skeleton, parts: [*]const RagdollPartDesc, part_count: u32) Result;

pub extern fn zjoltRagdollSettingsGetSkeleton(settings: *const RagdollSettings) ?*const Skeleton;

pub extern fn zjoltRagdollSettingsSetPartConstraint(settings: *RagdollSettings, part_index: u32, constraint_settings: ?*ConstraintSettings) Result;

pub extern fn zjoltRagdollSettingsGetPartConstraint(settings: *const RagdollSettings, part_index: u32, out: *?*ConstraintSettings) Result;

pub extern fn zjoltRagdollSettingsAddAdditionalConstraint(settings: *RagdollSettings, part_index1: u32, part_index2: u32, constraint_settings: *ConstraintSettings) Result;

pub extern fn zjoltRagdollSettingsGetNumAdditionalConstraints(settings: *const RagdollSettings) u32;

pub extern fn zjoltRagdollSettingsGetAdditionalConstraint(settings: *const RagdollSettings, index: u32, out_part_index1: ?*u32, out_part_index2: ?*u32, out_constraint: ?*?*ConstraintSettings) Result;

pub extern fn zjoltRagdollSettingsCalculateConstraintPriorities(settings: *RagdollSettings, base_priority: u32) Result;

pub extern fn zjoltRagdollSettingsStabilize(settings: *RagdollSettings) bool;

pub extern fn zjoltRagdollSettingsDisableParentChildCollisions(settings: *RagdollSettings) void;

pub extern fn zjoltRagdollSettingsCalculateBodyIndexToConstraintIndex(settings: *RagdollSettings) void;

pub extern fn zjoltRagdollSettingsGetConstraintIndexForBodyIndex(settings: *const RagdollSettings, body_index: u32) i32;

pub extern fn zjoltRagdollSettingsGetBodyIndexToConstraintIndex(settings: *const RagdollSettings, out_indices: ?[*]i32, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltRagdollSettingsCalculateConstraintIndexToBodyIdxPair(settings: *RagdollSettings) void;

pub const RagdollBodyIndexPair = extern struct {
    body_index1: i32,
    body_index2: i32,
};

pub extern fn zjoltRagdollSettingsGetBodyIndicesForConstraintIndex(settings: *const RagdollSettings, constraint_index: u32, out: *RagdollBodyIndexPair) Result;

pub extern fn zjoltRagdollSettingsGetConstraintIndexToBodyIdxPair(settings: *const RagdollSettings, out_pairs: ?[*]RagdollBodyIndexPair, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltRagdollSettingsCreateRagdoll(settings: *const RagdollSettings, system: *PhysicsSystem, collision_group: u32, user_data: u64, out: **Ragdoll) Result;

pub extern fn zjoltRagdollAddRef(ragdoll: *const Ragdoll) void;

pub extern fn zjoltRagdollRelease(ragdoll: *const Ragdoll) void;

pub extern fn zjoltRagdollGetRefCount(ragdoll: *const Ragdoll) u32;

pub extern fn zjoltRagdollAddToPhysicsSystem(ragdoll: *Ragdoll, activation: Activation, lock_bodies: bool) void;

pub extern fn zjoltRagdollRemoveFromPhysicsSystem(ragdoll: *Ragdoll, lock_bodies: bool) void;

pub extern fn zjoltRagdollGetBodyIds(ragdoll: *const Ragdoll, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltRagdollGetRagdollSettings(ragdoll: *const Ragdoll) ?*const RagdollSettings;

pub extern fn zjoltRagdollGetConstraintCount(ragdoll: *const Ragdoll) u32;

pub extern fn zjoltRagdollGetConstraint(ragdoll: *Ragdoll, index: u32) ?*Constraint;

pub extern fn zjoltRagdollGetRootTransform(ragdoll: *const Ragdoll, out_position: *RVec3, out_rotation: *Quat, lock_bodies: bool) Result;

pub extern fn zjoltRagdollSetGroupId(ragdoll: *Ragdoll, group_id: u32, lock_bodies: bool) void;

pub extern fn zjoltRagdollActivate(ragdoll: *Ragdoll, lock_bodies: bool) void;

pub extern fn zjoltRagdollSetLinearAndAngularVelocity(ragdoll: *Ragdoll, linear_velocity: *const Vec3, angular_velocity: *const Vec3, lock_bodies: bool) Result;

pub extern fn zjoltRagdollSetLinearVelocity(ragdoll: *Ragdoll, linear_velocity: *const Vec3, lock_bodies: bool) Result;

pub extern fn zjoltRagdollAddLinearVelocity(ragdoll: *Ragdoll, linear_velocity: *const Vec3, lock_bodies: bool) Result;

pub extern fn zjoltRagdollAddImpulse(ragdoll: *Ragdoll, impulse: *const Vec3, lock_bodies: bool) Result;

pub extern fn zjoltRagdollIsActive(ragdoll: *const Ragdoll, lock_bodies: bool) bool;

pub extern fn zjoltRagdollResetWarmStart(ragdoll: *Ragdoll) void;

pub extern fn zjoltRagdollSetPose(ragdoll: *Ragdoll, pose: *const SkeletonPose, lock_bodies: bool) Result;

pub extern fn zjoltRagdollGetPose(ragdoll: *Ragdoll, pose: *SkeletonPose, lock_bodies: bool) Result;

pub extern fn zjoltRagdollDriveToPoseUsingKinematics(ragdoll: *Ragdoll, pose: *const SkeletonPose, delta_time: f32, lock_bodies: bool) Result;

pub extern fn zjoltRagdollDriveToPoseUsingMotors(ragdoll: *Ragdoll, pose: *const SkeletonPose) Result;

pub extern fn zjoltRagdollDriveToPoseUsingMotorsWithVelocity(ragdoll: *Ragdoll, prev_pose: *const SkeletonPose, pose: *const SkeletonPose, delta_time: f32) Result;
