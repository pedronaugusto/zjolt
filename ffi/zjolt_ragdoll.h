//===----------------------------------------------------------------------===//
// zjolt — ragdolls: a skeleton, per-part bodies and constraints, driven or
// read back as a pose.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part.
//
// Three pieces: ZJoltSkeleton (joint hierarchy), ZJoltSkeletonPose (one
// pose of it), and ZJoltRagdollSettings (one body part per joint,
// optionally swing-twist constrained to its parent) — the reusable,
// reference-counted template zjoltRagdollSettingsCreateRagdoll spawns from.
// A pose crosses as local (SetJoints/GetJoints) or flattened
// (CalculateJointMatrices/CalculateJointStates) form; convert explicitly.
// ZJoltSkeletonMapper maps a pose between a low- and high-detail skeleton.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_RAGDOLL_H_
#define ZJOLT_RAGDOLL_H_

#include "zjolt_body.h"
#include "zjolt_constraint.h"
#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Opaque handles
//===----------------------------------------------------------------------===//

/// A joint hierarchy: names and parent indices, no transforms. Reference
/// counted, and shareable between poses and between ragdoll settings.
typedef struct ZJoltSkeleton ZJoltSkeleton;

/// One instance of a skeleton in a particular pose. NOT reference counted —
/// a plain owning handle, created and destroyed like a job system.
typedef struct ZJoltSkeletonPose ZJoltSkeletonPose;

/// A skinned animation: keyframed local-space rotation/translation per
/// animated joint, sampled onto a ZJoltSkeletonPose. Reference counted.
typedef struct ZJoltSkeletalAnimation ZJoltSkeletalAnimation;

/// A correspondence between the joints of two skeletons, and the poses that
/// can be pushed across it. Reference counted. Built once with
/// zjoltSkeletonMapperInitialize and then read-only on every frame it maps.
typedef struct ZJoltSkeletonMapper ZJoltSkeletonMapper;

/// The reusable template a ragdoll is spawned from: a skeleton plus one body
/// part per joint. Reference counted; see the file comment above for why this
/// crosses the boundary as a handle rather than being built on the stack.
typedef struct ZJoltRagdollSettings ZJoltRagdollSettings;

/// A spawned ragdoll: real bodies and constraints in one PhysicsSystem.
/// Reference counted, like the settings that created it — but unlike the
/// three handles above it owns storage of its own beside the Jolt object, so
/// it is also counted by zjoltLiveHandleCount and zjoltDeinit refuses while
/// one is still alive.
typedef struct ZJoltRagdoll ZJoltRagdoll;

//===----------------------------------------------------------------------===//
// Skeleton
//===----------------------------------------------------------------------===//

/// Creates an empty skeleton (no joints).
ZJOLT_API ZJoltResult zjoltSkeletonCreate(ZJoltSkeleton **out);

ZJOLT_API void zjoltSkeletonAddRef(const ZJoltSkeleton *skeleton);
ZJOLT_API void zjoltSkeletonRelease(const ZJoltSkeleton *skeleton);
ZJOLT_API uint32_t zjoltSkeletonGetRefCount(const ZJoltSkeleton *skeleton);

/// Appends one joint and reports its index through `out_index`. `name` is
/// copied and may be NULL for an unnamed joint.
///
/// `parent_index` is -1 for a root joint, or the index of a joint ADDED
/// EARLIER in the same skeleton (required by every hierarchy-walking
/// ragdoll operation) — refused if out of range or not yet added.
ZJOLT_API ZJoltResult zjoltSkeletonAddJoint(ZJoltSkeleton *skeleton,
                                            const char *name,
                                            int32_t parent_index,
                                            uint32_t *out_index);

/// Appends one joint whose parent is NAMED, for a host importing a skeleton
/// that stores parents that way. The index stays -1 until
/// zjoltSkeletonCalculateParentJointIndices resolves it; `parent_name` NULL
/// or empty means a root joint.
ZJOLT_API ZJoltResult zjoltSkeletonAddJointWithParentName(
    ZJoltSkeleton *skeleton, const char *name, const char *parent_name,
    uint32_t *out_index);

/// Resolves every joint's parent NAME to its index —
/// Skeleton::CalculateParentJointIndices. A joint added by index already
/// carries its answer; one added by name has -1 until this runs. A name no
/// joint has leaves -1, which is a root.
ZJOLT_API ZJoltResult zjoltSkeletonCalculateParentJointIndices(
    ZJoltSkeleton *skeleton);

/// 0 if `skeleton` is NULL.
ZJOLT_API uint32_t zjoltSkeletonGetJointCount(const ZJoltSkeleton *skeleton);

/// -1 if `skeleton` or `name` is NULL, or no joint has that name.
ZJOLT_API int32_t zjoltSkeletonGetJointIndex(const ZJoltSkeleton *skeleton,
                                             const char *name);

/// -1 for a root joint, or if `skeleton` is NULL or `index` is out of range.
ZJOLT_API int32_t zjoltSkeletonGetJointParentIndex(
    const ZJoltSkeleton *skeleton, uint32_t index);

/// Borrowed; valid until the skeleton is destroyed. "" if `skeleton` is NULL
/// or `index` is out of range.
ZJOLT_API const char *zjoltSkeletonGetJointName(const ZJoltSkeleton *skeleton,
                                                uint32_t index);

/// True if every joint's parent index is below its own — the precondition
/// every ragdoll and pose operation below has on the skeleton it is given.
/// `zjoltSkeletonAddJoint` cannot build a skeleton that fails this;
/// `zjoltSkeletonAddJointWithParentName` can, and so can a restore.
/// False if `skeleton` is NULL.
ZJOLT_API bool zjoltSkeletonAreJointsCorrectlyOrdered(
    const ZJoltSkeleton *skeleton);

//===----------------------------------------------------------------------===//
// SkeletonPose
//===----------------------------------------------------------------------===//

/// Creates a pose with no skeleton assigned (zero joints) — call
/// zjoltSkeletonPoseSetSkeleton before anything else.
ZJOLT_API ZJoltResult zjoltSkeletonPoseCreate(ZJoltSkeletonPose **out);

ZJOLT_API void zjoltSkeletonPoseDestroy(ZJoltSkeletonPose *pose);

/// Resizes the pose to `skeleton`'s joint count, discarding any joint data it
/// held. The pose keeps its own reference to `skeleton`.
ZJOLT_API ZJoltResult zjoltSkeletonPoseSetSkeleton(ZJoltSkeletonPose *pose,
                                                   const ZJoltSkeleton *skeleton);

/// Borrowed. NULL if `pose` is NULL or has none assigned.
ZJOLT_API const ZJoltSkeleton *zjoltSkeletonPoseGetSkeleton(
    const ZJoltSkeletonPose *pose);

/// 0 if `pose` is NULL or has no skeleton assigned.
ZJOLT_API uint32_t zjoltSkeletonPoseGetJointCount(
    const ZJoltSkeletonPose *pose);

ZJOLT_API void zjoltSkeletonPoseSetRootOffset(ZJoltSkeletonPose *pose,
                                              const ZJoltRVec3 *offset);
ZJOLT_API void zjoltSkeletonPoseGetRootOffset(const ZJoltSkeletonPose *pose,
                                              ZJoltRVec3 *out);

/// Bulk-sets every joint's LOCAL rotation/translation (relative to its
/// parent joint, identity/zero meaning "at the parent"). `count` must equal
/// zjoltSkeletonPoseGetJointCount exactly. Either array may be NULL to leave
/// that half of every joint unchanged.
ZJOLT_API ZJoltResult zjoltSkeletonPoseSetJoints(ZJoltSkeletonPose *pose,
                                                 const ZJoltQuat *rotations,
                                                 const ZJoltVec3 *translations,
                                                 uint32_t count);

/// Two-call protocol, as zjoltPhysicsSystemGetBodies: `*out_count` always
/// receives the joint count, and a NULL array with any capacity is a size
/// query. Either array may be NULL independently to read only one half.
ZJOLT_API ZJoltResult zjoltSkeletonPoseGetJoints(const ZJoltSkeletonPose *pose,
                                                 ZJoltQuat *out_rotations,
                                                 ZJoltVec3 *out_translations,
                                                 uint32_t capacity,
                                                 uint32_t *out_count);

/// The joint matrices zjoltSkeletonPoseCalculateJointMatrices computes: one
/// MODEL-space transform per joint, sixteen floats each, column-major —
/// column c's row r is `out_matrices[16 * joint + 4 * c + r]`, the layout
/// zjoltHairSetPose uses. Two-call protocol: `*out_count` receives the joint
/// count and a NULL array is a size query, so `capacity` counts JOINTS.
ZJOLT_API ZJoltResult zjoltSkeletonPoseGetJointMatrices(
    const ZJoltSkeletonPose *pose, float *out_matrices, uint32_t capacity,
    uint32_t *out_count);

/// The same array, written. `count` must equal the joint count exactly. This
/// is the way in for a pose produced outside Jolt — a skinning solver, an
/// animation runtime — without going through per-joint local transforms;
/// zjoltSkeletonPoseCalculateJointStates then derives those from these.
ZJOLT_API ZJoltResult zjoltSkeletonPoseSetJointMatrices(
    ZJoltSkeletonPose *pose, const float *matrices, uint32_t count);

/// Converts the per-joint LOCAL rotations/translations from
/// zjoltSkeletonPoseSetJoints into the flattened form zjoltRagdollSetPose,
/// GetPose, and DriveToPoseUsingKinematics consume — call before any of
/// them. Requires the skeleton's joints correctly ordered
/// (zjoltSkeletonAreJointsCorrectlyOrdered) and assigned;
/// ZJOLT_RESULT_INVALID_ARGUMENT otherwise.
ZJOLT_API ZJoltResult zjoltSkeletonPoseCalculateJointMatrices(
    ZJoltSkeletonPose *pose);

/// The reverse conversion: what zjoltRagdollGetPose leaves the caller
/// needing before zjoltSkeletonPoseGetJoints returns anything meaningful.
/// Fails with ZJOLT_RESULT_INVALID_ARGUMENT if no skeleton is assigned.
ZJOLT_API ZJoltResult zjoltSkeletonPoseCalculateJointStates(
    ZJoltSkeletonPose *pose);

//===----------------------------------------------------------------------===//
// SkeletalAnimation
//===----------------------------------------------------------------------===//

/// Creates an empty animation (no animated joints).
ZJOLT_API ZJoltResult zjoltSkeletalAnimationCreate(ZJoltSkeletalAnimation **out);

ZJOLT_API void zjoltSkeletalAnimationAddRef(const ZJoltSkeletalAnimation *animation);
ZJOLT_API void zjoltSkeletalAnimationRelease(const ZJoltSkeletalAnimation *animation);
ZJOLT_API uint32_t zjoltSkeletalAnimationGetRefCount(
    const ZJoltSkeletalAnimation *animation);

/// Appends an animated joint, initially with no keyframes, and reports its
/// index. `name` is copied, and is what zjoltSkeletalAnimationSample looks up
/// in the pose's skeleton — it need not match one yet.
ZJOLT_API ZJoltResult zjoltSkeletalAnimationAddAnimatedJoint(
    ZJoltSkeletalAnimation *animation, const char *name, uint32_t *out_index);

/// 0 if `animation` is NULL.
ZJOLT_API uint32_t zjoltSkeletalAnimationGetAnimatedJointCount(
    const ZJoltSkeletalAnimation *animation);

/// Borrowed; valid until the animation is destroyed. "" if `animation` is
/// NULL or `joint_index` is out of range.
ZJOLT_API const char *zjoltSkeletalAnimationGetAnimatedJointName(
    const ZJoltSkeletalAnimation *animation, uint32_t joint_index);

/// Appends a keyframe to animated joint `joint_index`. Keyframes must be
/// added in non-decreasing `time` order: zjoltSkeletalAnimationSample does a
/// binary search over them that assumes it, and Jolt does not check.
ZJOLT_API ZJoltResult zjoltSkeletalAnimationAddKeyframe(
    ZJoltSkeletalAnimation *animation, uint32_t joint_index, float time,
    const ZJoltQuat *rotation, const ZJoltVec3 *translation);

/// 0 if `animation` is NULL or `joint_index` is out of range.
ZJOLT_API uint32_t zjoltSkeletalAnimationGetKeyframeCount(
    const ZJoltSkeletalAnimation *animation, uint32_t joint_index);

/// Fails with ZJOLT_RESULT_INVALID_ARGUMENT if `joint_index` or
/// `keyframe_index` is out of range.
ZJOLT_API ZJoltResult zjoltSkeletalAnimationGetKeyframe(
    const ZJoltSkeletalAnimation *animation, uint32_t joint_index,
    uint32_t keyframe_index, float *out_time, ZJoltQuat *out_rotation,
    ZJoltVec3 *out_translation);

/// The time (seconds) of the last keyframe of the FIRST animated joint. 0 if
/// `animation` is NULL, has no animated joints, or that joint has none —
/// Jolt's own fallback.
ZJOLT_API float zjoltSkeletalAnimationGetDuration(
    const ZJoltSkeletalAnimation *animation);

/// Scales every keyframe's translation by `scale`.
ZJOLT_API void zjoltSkeletalAnimationScaleJoints(
    ZJoltSkeletalAnimation *animation, float scale);

/// If the animation loops, zjoltSkeletalAnimationSample wraps `time` modulo
/// zjoltSkeletalAnimationGetDuration instead of clamping to the last
/// keyframe.
ZJOLT_API void zjoltSkeletalAnimationSetIsLooping(
    ZJoltSkeletalAnimation *animation, bool is_looping);
ZJOLT_API bool zjoltSkeletalAnimationIsLooping(
    const ZJoltSkeletalAnimation *animation);

/// Samples the (interpolated) joint transforms at `time` onto `pose`'s
/// LOCAL joint rotations/translations; no CalculateJointMatrices needed first.
///
/// `time` must not be negative. Every animated joint's name must be a
/// joint of `pose`'s assigned skeleton, or the call is refused — Jolt
/// indexes by name with no bounds check.
ZJOLT_API ZJoltResult zjoltSkeletalAnimationSample(
    const ZJoltSkeletalAnimation *animation, float time,
    ZJoltSkeletonPose *pose);

//===----------------------------------------------------------------------===//
// Jolt's own binary stream, over the same ZJoltStream seam as
// zjoltRagdollSettingsSaveObjectStream below — but Jolt's older hand-rolled
// binary form rather than the reflective object stream. An animation has no
// shape to lose on the way through.
//===----------------------------------------------------------------------===//

ZJOLT_API ZJoltResult zjoltSkeletalAnimationSaveBinaryState(
    const ZJoltSkeletalAnimation *animation, const ZJoltStream *stream);

/// Rebuilds an animation written by zjoltSkeletalAnimationSaveBinaryState.
/// Release the result with zjoltSkeletalAnimationRelease.
ZJOLT_API ZJoltResult zjoltSkeletalAnimationRestoreBinaryState(
    const ZJoltStream *stream, ZJoltSkeletalAnimation **out);

//===----------------------------------------------------------------------===//
// JointState: the rotation/translation pair a keyframe carries.
//===----------------------------------------------------------------------===//

/// Decomposes a local-space matrix into a rotation and translation.
ZJOLT_API void zjoltSkeletalAnimationJointStateFromMatrix(
    const ZJoltMat44 *matrix, ZJoltQuat *out_rotation,
    ZJoltVec3 *out_translation);

/// The inverse.
ZJOLT_API void zjoltSkeletalAnimationJointStateToMatrix(
    const ZJoltQuat *rotation, const ZJoltVec3 *translation, ZJoltMat44 *out);

//===----------------------------------------------------------------------===//
// SkeletonMapper
//
// Maps a pose between two skeletons: Initialize finds the correspondence
// from neutral poses; Map/MapReverse push a pose across it, in model space.
//===----------------------------------------------------------------------===//

/// Whether joint `index1` of `skeleton1` is the same joint as `index2` of
/// `skeleton2`. Called during zjoltSkeletonMapperInitialize only. Pass
/// NULL for Jolt's default (joint NAMES equal); supply one when skeletons
/// name joints differently, or a name appears twice.
///
/// Nothing may unwind out of this — see the callback note in BINDING.md.
typedef bool (*ZJoltSkeletonMapperCanMapJointFn)(void *user,
                                                 const ZJoltSkeleton *skeleton1,
                                                 uint32_t index1,
                                                 const ZJoltSkeleton *skeleton2,
                                                 uint32_t index2);

/// Creates an uninitialised mapper — call zjoltSkeletonMapperInitialize next.
ZJOLT_API ZJoltResult zjoltSkeletonMapperCreate(ZJoltSkeletonMapper **out);

ZJOLT_API void zjoltSkeletonMapperAddRef(const ZJoltSkeletonMapper *mapper);
ZJOLT_API void zjoltSkeletonMapperRelease(const ZJoltSkeletonMapper *mapper);
ZJOLT_API uint32_t zjoltSkeletonMapperGetRefCount(
    const ZJoltSkeletonMapper *mapper);

/// Works out the correspondence between the two skeletons from their neutral
/// poses; every other call below depends on this. `neutral1` is LOW-detail
/// (simulated), `neutral2` HIGH-detail (Jolt requires that order), and both
/// poses must be in MODEL space (unchecked — wrong space maps to nonsense, not
/// an error). ZJOLT_RESULT_INVALID_ARGUMENT if already initialised, unassigned,
/// misordered, or `neutral1` outsizes `neutral2`.
ZJOLT_API ZJoltResult zjoltSkeletonMapperInitialize(
    ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *neutral1,
    const ZJoltSkeletonPose *neutral2,
    ZJoltSkeletonMapperCanMapJointFn can_map_joint, void *user);

/// How many joints of skeleton 1 were matched one-to-one with a joint of
/// skeleton 2. 0 before zjoltSkeletonMapperInitialize, and 0 after it if
/// nothing matched — which is the failure worth checking for, because a
/// mapper that matched nothing is not an error, it just maps nothing.
ZJOLT_API uint32_t zjoltSkeletonMapperGetMappingCount(
    const ZJoltSkeletonMapper *mapper);

/// The joint of skeleton 2 that joint `joint1_index` of skeleton 1 drives, or
/// -1 if that joint was not matched (or `mapper` is NULL).
ZJOLT_API int32_t zjoltSkeletonMapperGetMappedJointIndex(
    const ZJoltSkeletonMapper *mapper, uint32_t joint1_index);

/// How many joint chains the mapper found between two 1-on-1 mapped joints —
/// runs of joints in skeleton 2 that skeleton 1 has no equivalent for. 0
/// before zjoltSkeletonMapperInitialize.
ZJOLT_API uint32_t zjoltSkeletonMapperGetChainCount(
    const ZJoltSkeletonMapper *mapper);

/// The length of chain `chain_index`'s two joint-index runs: skeleton 1's and
/// skeleton 2's. Both 0 if `mapper` is NULL or `chain_index` is past the end.
ZJOLT_API void zjoltSkeletonMapperGetChainJointCounts(
    const ZJoltSkeletonMapper *mapper, uint32_t chain_index,
    uint32_t *out_count1, uint32_t *out_count2);

/// One joint index of chain `chain_index`'s skeleton-1 run. -1 if `mapper` is
/// NULL or either index is out of range.
ZJOLT_API int32_t zjoltSkeletonMapperGetChainJointIndex1(
    const ZJoltSkeletonMapper *mapper, uint32_t chain_index, uint32_t index);

/// One joint index of chain `chain_index`'s skeleton-2 run. -1 if `mapper` is
/// NULL or either index is out of range.
ZJOLT_API int32_t zjoltSkeletonMapperGetChainJointIndex2(
    const ZJoltSkeletonMapper *mapper, uint32_t chain_index, uint32_t index);

/// How many joints of skeleton 2 could not be mapped to skeleton 1 at all. 0
/// before zjoltSkeletonMapperInitialize.
ZJOLT_API uint32_t zjoltSkeletonMapperGetUnmappedCount(
    const ZJoltSkeletonMapper *mapper);

/// One unmapped joint of skeleton 2: its own index and its parent's, both in
/// skeleton 2. `parent_joint_index` is -1 for one with no parent.
typedef struct ZJoltSkeletonMapperUnmapped {
  int32_t joint_index;
  int32_t parent_joint_index;
} ZJoltSkeletonMapperUnmapped;

/// Fails with ZJOLT_RESULT_INVALID_ARGUMENT if `mapper` is NULL or `index` is
/// past the end.
ZJOLT_API ZJoltResult zjoltSkeletonMapperGetUnmapped(
    const ZJoltSkeletonMapper *mapper, uint32_t index,
    ZJoltSkeletonMapperUnmapped *out);

/// Pins the translation of the named joints of skeleton 2 to their
/// neutral pose, so only their rotation follows the ragdoll — removes
/// stretch a non-rigid constraint causes, at the cost of drawn and
/// simulated skeletons no longer matching exactly. `locked` has one bool
/// per joint of `neutral2` (`count` must match); a joint with NO PARENT
/// is refused, not locked. Calls accumulate; no unlock.
ZJOLT_API ZJoltResult zjoltSkeletonMapperLockTranslations(
    ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *neutral2,
    const bool *locked, uint32_t count);

/// zjoltSkeletonMapperLockTranslations for every joint of skeleton 2 below
/// the topmost mapped joint, excluded — everything the ragdoll drives
/// stops stretching, while the ragdoll stays free to move as a whole.
///
/// Refuses a mapper not yet through zjoltSkeletonMapperInitialize.
ZJOLT_API ZJoltResult zjoltSkeletonMapperLockAllTranslations(
    ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *neutral2);

/// Whether zjoltSkeletonMapperLockTranslations or LockAllTranslations locked
/// joint `joint2_index` of skeleton 2. False if `mapper` is NULL.
ZJOLT_API bool zjoltSkeletonMapperIsJointTranslationLocked(
    const ZJoltSkeletonMapper *mapper, uint32_t joint2_index);

/// Pushes `pose1`'s MODEL-space joint matrices onto `pose2` (combined
/// with `pose2`'s own LOCAL joints for anything the ragdoll does not
/// drive) and writes `pose2`'s MODEL-space matrices — read back via
/// CalculateJointStates + GetJoints. Unlike Jolt's own Map, `pose1`'s
/// ROOT OFFSET is copied onto `pose2` too (a pose's world position lives
/// there, not its matrices).
ZJOLT_API ZJoltResult zjoltSkeletonMapperMap(const ZJoltSkeletonMapper *mapper,
                                             const ZJoltSkeletonPose *pose1,
                                             ZJoltSkeletonPose *pose2);

/// The other direction: reads `pose2`'s MODEL-space matrices, writes
/// `pose1`'s — sample animation into `pose2`, CalculateJointMatrices, map
/// back, then DriveToPoseUsingKinematics/SetPose with `pose1` (`pose2`'s
/// root offset travels onto `pose1` too). Only one-to-one mapped joints
/// are written; an unmapped `pose1` joint keeps whatever (possibly
/// uninitialised) matrix it already held.
ZJOLT_API ZJoltResult zjoltSkeletonMapperMapReverse(
    const ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *pose2,
    ZJoltSkeletonPose *pose1);

//===----------------------------------------------------------------------===//
// RagdollSettings
//===----------------------------------------------------------------------===//

/// The swing-twist constraint attaching one part to its parent, in WORLD
/// space — the same space as the parts' ZJoltBodyDesc::position/rotation.
/// Motor tuning is not exposed: driving through
/// zjoltRagdollDriveToPoseUsingMotors uses Jolt's own default MotorSettings
/// (a critically-damped position spring, unlimited torque), which is enough
/// to reach a target pose.
typedef struct ZJoltRagdollConstraintDesc {
  ZJoltRVec3 position1;
  ZJoltVec3 twist_axis1;
  ZJoltVec3 plane_axis1;
  ZJoltRVec3 position2;
  ZJoltVec3 twist_axis2;
  ZJoltVec3 plane_axis2;
  /// The shape of this joint's swing limit. Jolt's SwingTwistConstraint is
  /// what it documents as built for humanoid ragdolls, and the only non-hinge
  /// constraint zjoltRagdollDriveToPoseUsingMotors knows how to drive; see it
  /// for the geometry these limits describe.
  ZJoltSwingType swing_type;
  /// Angle in radians. See SwingTwistConstraintSettings::mNormalHalfConeAngle.
  float normal_half_cone_angle;
  float plane_half_cone_angle;
  /// Angle in radians, should be within [-pi, pi].
  float twist_min_angle;
  float twist_max_angle;
  /// Torque (N m) applied as friction when no motor is driving this joint.
  float max_friction_torque;
} ZJoltRagdollConstraintDesc;

/// One rigid body of a ragdoll, and the constraint attaching it to its
/// parent. `to_parent` must be NULL for the root part (the joint whose
/// parent index is -1) and non-NULL for every other part —
/// zjoltRagdollSettingsBuild refuses the mismatch rather than letting
/// zjoltRagdollSettingsCreateRagdoll index a parent that does not exist.
typedef struct ZJoltRagdollPartDesc {
  ZJoltBodyDesc body;
  const ZJoltRagdollConstraintDesc *to_parent;
} ZJoltRagdollPartDesc;

/// Creates settings with no skeleton and no parts — call
/// zjoltRagdollSettingsBuild before anything else.
ZJOLT_API ZJoltResult zjoltRagdollSettingsCreate(ZJoltRagdollSettings **out);

ZJOLT_API void zjoltRagdollSettingsAddRef(const ZJoltRagdollSettings *settings);
ZJOLT_API void zjoltRagdollSettingsRelease(const ZJoltRagdollSettings *settings);
ZJOLT_API uint32_t zjoltRagdollSettingsGetRefCount(
    const ZJoltRagdollSettings *settings);

//===----------------------------------------------------------------------===//
// Jolt's own object stream
//
// RagdollSettings is one of Jolt's reflective-object-stream types, for editor authoring — @see zjoltSceneSaveObjectStream.
// A part's SHAPE does not survive the round trip; re-attach after loading. ZJOLT_RESULT_UNSUPPORTED without -Dobject_stream=true.
//===----------------------------------------------------------------------===//

/// Writes `settings` through `stream` in Jolt's own object-stream format.
/// ZJOLT_RESULT_IO_ERROR if `stream` reports failure while this runs. @see
/// the section comment above for what a part loses on the way through.
ZJOLT_API ZJoltResult zjoltRagdollSettingsSaveObjectStream(
    const ZJoltRagdollSettings *settings, ZJoltObjectStreamFormat format,
    const ZJoltStream *stream);

/// Reads settings written by zjoltRagdollSettingsSaveObjectStream, or by
/// a C++ host of vanilla Jolt — sniffed from the stream. Release with
/// zjoltRagdollSettingsRelease.
///
/// Not run through zjoltRagdollSettingsBuild — already carries whatever
/// skeleton and parts the stream described. No part has a shape.
ZJOLT_API ZJoltResult zjoltRagdollSettingsRestoreObjectStream(
    const ZJoltStream *stream, ZJoltRagdollSettings **out);

/// Assigns `skeleton` and builds one part per joint from `parts`, which must
/// have exactly `skeleton`'s joint count entries in joint-index order.
/// Overwrites whatever the settings held before. Fails with
/// ZJOLT_RESULT_INVALID_ARGUMENT if `part_count` disagrees with the
/// skeleton's joint count, if any part is missing a shape, or if
/// `to_parent`'s NULL-ness disagrees with whether that joint has a parent.
ZJOLT_API ZJoltResult zjoltRagdollSettingsBuild(ZJoltRagdollSettings *settings,
                                                const ZJoltSkeleton *skeleton,
                                                const ZJoltRagdollPartDesc *parts,
                                                uint32_t part_count);

/// The skeleton zjoltRagdollSettingsBuild was given. Borrowed, and NOT
/// AddRef'd — the settings' own reference keeps it alive. NULL if `settings`
/// is NULL or has not been built.
///
/// With zjoltRagdollGetRagdollSettings, this is the route from a live ragdoll
/// to the joint names behind the ids zjoltRagdollGetBodyIds hands back.
ZJOLT_API const ZJoltSkeleton *zjoltRagdollSettingsGetSkeleton(
    const ZJoltRagdollSettings *settings);

/// Replaces part `part_index`'s joint to its parent with any two-body
/// constraint, not just the swing-twist ZJoltRagdollConstraintDesc describes;
/// NULL removes it, which is what the root part has. Refuses a kind that is
/// not two-body: Jolt's field takes TwoBodyConstraintSettings and no other.
/// Call after zjoltRagdollSettingsBuild, which sizes the part list; a
/// reference is taken and the caller keeps its own.
ZJOLT_API ZJoltResult zjoltRagdollSettingsSetPartConstraint(
    ZJoltRagdollSettings *settings, uint32_t part_index,
    ZJoltConstraintSettings *constraint);

/// Part `part_index`'s joint to its parent, with a reference taken — release
/// it. `*out` is NULL for a part with no joint, which is not an error.
ZJOLT_API ZJoltResult zjoltRagdollSettingsGetPartConstraint(
    const ZJoltRagdollSettings *settings, uint32_t part_index,
    ZJoltConstraintSettings **out);

/// A constraint between two parts that are NOT parent and child — Jolt's
/// RagdollSettings::mAdditionalConstraints. The two indices are part indices,
/// the same ones zjoltRagdollSettingsSetPartConstraint takes. Same two-body
/// requirement, and a reference is taken.
ZJOLT_API ZJoltResult zjoltRagdollSettingsAddAdditionalConstraint(
    ZJoltRagdollSettings *settings, uint32_t part_index1, uint32_t part_index2,
    ZJoltConstraintSettings *constraint);

ZJOLT_API uint32_t zjoltRagdollSettingsGetNumAdditionalConstraints(
    const ZJoltRagdollSettings *settings);

/// One of them, by index. `out_constraint` receives a reference — release it.
/// Any of the three out pointers may be NULL.
ZJOLT_API ZJoltResult zjoltRagdollSettingsGetAdditionalConstraint(
    const ZJoltRagdollSettings *settings, uint32_t index,
    uint32_t *out_part_index1, uint32_t *out_part_index2,
    ZJoltConstraintSettings **out_constraint);

/// Gives each part's constraint a solver priority, counting UP toward the
/// root, so a shoulder under load stops fighting the wrist hanging off
/// it. Call after zjoltRagdollSettingsBuild. `base_priority` is the
/// LOWEST priority (leaf-most constraints); 0 unless sorting against
/// other constraints in the same system. ZJOLT_RESULT_INVALID_ARGUMENT
/// if unbuilt, misordered, or overflowing.
ZJOLT_API ZJoltResult zjoltRagdollSettingsCalculateConstraintPriorities(
    ZJoltRagdollSettings *settings, uint32_t base_priority);

/// Rebalances part mass and inertia along parent-child chains so the solver
/// stays stable. Returns false if a chain's inertia tensor could not be
/// decomposed (rare); false if `settings` is NULL.
ZJOLT_API bool zjoltRagdollSettingsStabilize(ZJoltRagdollSettings *settings);

/// Builds a shared collision-group filter that disables collisions
/// between every part and its parent, assigned (per-joint sub-group) to
/// every part. Pass the resulting group id to every ragdoll spawned from
/// these settings. Unlike Jolt's own DisableParentChildCollisions, never
/// runs the optional bind-pose collision pass — only hierarchy-adjacent
/// joints are disabled.
ZJOLT_API void zjoltRagdollSettingsDisableParentChildCollisions(
    ZJoltRagdollSettings *settings);

/// Builds the body-index -> constraint-index table
/// zjoltRagdollDriveToPoseUsingMotors needs to find each body's constraint.
/// Call this — after zjoltRagdollSettingsBuild, before
/// zjoltRagdollSettingsCreateRagdoll — for every settings object a ragdoll
/// driven with motors will be spawned from; CreateRagdoll does not call it
/// for you, matching Jolt's own RagdollSettings.
ZJOLT_API void zjoltRagdollSettingsCalculateBodyIndexToConstraintIndex(
    ZJoltRagdollSettings *settings);

/// Maps a body index (index into zjoltRagdollGetBodyIds) to the constraint
/// index (into zjoltRagdollGetConstraint) attaching it to its parent. -1 if
/// it has none — the root, or the table has not been built — or if
/// `settings` is NULL or `body_index` is out of range.
ZJOLT_API int32_t zjoltRagdollSettingsGetConstraintIndexForBodyIndex(
    const ZJoltRagdollSettings *settings, uint32_t body_index);

/// Two-call protocol, as zjoltRagdollGetBodyIds: `*out_count` always receives
/// the part count, and a NULL array with any capacity is a size query. Every
/// entry is zjoltRagdollSettingsGetConstraintIndexForBodyIndex's answer for
/// that body index, in order.
ZJOLT_API ZJoltResult zjoltRagdollSettingsGetBodyIndexToConstraintIndex(
    const ZJoltRagdollSettings *settings, int32_t *out_indices,
    uint32_t capacity, uint32_t *out_count);

/// Builds the constraint-index -> body-index-pair table
/// zjoltRagdollSettingsGetBodyIndicesForConstraintIndex and
/// zjoltRagdollSettingsGetConstraintIndexToBodyIdxPair need. Call after
/// zjoltRagdollSettingsBuild, the same as
/// zjoltRagdollSettingsCalculateBodyIndexToConstraintIndex above — the two
/// tables are independent, so building one does not build the other.
ZJOLT_API void zjoltRagdollSettingsCalculateConstraintIndexToBodyIdxPair(
    ZJoltRagdollSettings *settings);

/// A constraint's two body indices, in the same order zjoltRagdollGetBodyIds
/// reports them.
typedef struct ZJoltRagdollBodyIndexPair {
  int32_t body_index1;
  int32_t body_index2;
} ZJoltRagdollBodyIndexPair;

/// Fails with ZJOLT_RESULT_INVALID_ARGUMENT if `settings` is NULL or
/// `constraint_index` is out of range, including before
/// zjoltRagdollSettingsCalculateConstraintIndexToBodyIdxPair has run.
ZJOLT_API ZJoltResult zjoltRagdollSettingsGetBodyIndicesForConstraintIndex(
    const ZJoltRagdollSettings *settings, uint32_t constraint_index,
    ZJoltRagdollBodyIndexPair *out);

/// Two-call protocol, as zjoltRagdollGetBodyIds: `*out_count` always receives
/// the constraint count, and a NULL array with any capacity is a size query.
ZJOLT_API ZJoltResult zjoltRagdollSettingsGetConstraintIndexToBodyIdxPair(
    const ZJoltRagdollSettings *settings, ZJoltRagdollBodyIndexPair *out_pairs,
    uint32_t capacity, uint32_t *out_count);

/// Spawns one body per part and one constraint per non-root part, in
/// `system` but NOT YET added — see zjoltRagdollAddToPhysicsSystem.
///
/// `collision_group` is set on every spawned body; ragdolls sharing a
/// DisableParentChildCollisions filter need DIFFERENT ids.
/// ZJOLT_RESULT_OUT_OF_MEMORY if there is no room.
ZJOLT_API ZJoltResult zjoltRagdollSettingsCreateRagdoll(
    const ZJoltRagdollSettings *settings, ZJoltPhysicsSystem *system,
    uint32_t collision_group, uint64_t user_data, ZJoltRagdoll **out);

//===----------------------------------------------------------------------===//
// Ragdoll
//===----------------------------------------------------------------------===//

ZJOLT_API void zjoltRagdollAddRef(const ZJoltRagdoll *ragdoll);

/// Drops one reference. The last one takes the ragdoll out of its
/// physics system first (if still in it) and destroys every body it owns.
/// Call order is free: releasing an added ragdoll works the same as
/// removing it first. zjoltRagdollRemoveFromPhysicsSystem takes a
/// ragdoll out while keeping it alive.
ZJOLT_API void zjoltRagdollRelease(const ZJoltRagdoll *ragdoll);

/// References outstanding: one from zjoltRagdollSettingsCreateRagdoll, plus
/// one per zjoltRagdollAddRef, less one per zjoltRagdollRelease. 0 if
/// `ragdoll` is NULL.
ZJOLT_API uint32_t zjoltRagdollGetRefCount(const ZJoltRagdoll *ragdoll);

/// Adds every body and constraint to the system passed to
/// zjoltRagdollSettingsCreateRagdoll.
ZJOLT_API void zjoltRagdollAddToPhysicsSystem(ZJoltRagdoll *ragdoll,
                                              ZJoltActivation activation,
                                              bool lock_bodies);

/// Takes them back out again, leaving the ragdoll alive and addable again.
/// Not idempotent — remove at most once per zjoltRagdollAddToPhysicsSystem —
/// and not something zjoltRagdollRelease needs done for it.
ZJOLT_API void zjoltRagdollRemoveFromPhysicsSystem(ZJoltRagdoll *ragdoll,
                                                   bool lock_bodies);

/// The body ids of the ragdoll's parts, indexed by joint — index `i` is
/// the body built from skeleton joint `i`. Maps a hit's ZJoltBodyId back
/// to a limb, and lets zjoltBodyInterface* reach one part directly.
///
/// Two-call protocol: NULL array is a size query. Ids stay valid until
/// the last reference drops — do not destroy one directly.
ZJOLT_API ZJoltResult zjoltRagdollGetBodyIds(const ZJoltRagdoll *ragdoll,
                                             ZJoltBodyId *out_ids,
                                             uint32_t capacity,
                                             uint32_t *out_count);

/// The settings this ragdoll was spawned from. Borrowed, and NOT AddRef'd —
/// the ragdoll's own reference keeps them alive for as long as it does. NULL
/// if `ragdoll` is NULL.
ZJOLT_API const ZJoltRagdollSettings *zjoltRagdollGetRagdollSettings(
    const ZJoltRagdoll *ragdoll);

/// How many constraints the ragdoll owns: one per part that has a parent, so
/// one fewer than zjoltRagdollGetBodyIds reports for a single-rooted
/// skeleton. 0 if `ragdoll` is NULL.
ZJOLT_API uint32_t zjoltRagdollGetConstraintCount(const ZJoltRagdoll *ragdoll);

/// One of them, NULL if `index` is past the end or `ragdoll` is NULL.
/// Borrowed, owned by the ragdoll: do not Release or Remove it by hand
/// (RemoveFromPhysicsSystem moves the whole ragdoll as one batch).
///
/// Joint-index order, skipping the root: constraint `i` attaches the
/// `i`-th non-root joint to its parent. Always a swing-twist constraint.
ZJOLT_API ZJoltConstraint *zjoltRagdollGetConstraint(ZJoltRagdoll *ragdoll,
                                                     uint32_t index);

/// Where the ragdoll is: the world transform of the body built from the
/// skeleton's first joint — what a camera, footstep or distance cull
/// wants without reading every part back.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT for a ragdoll with no parts. If the
/// body lock fails, outputs are zeroed/identity but the call still succeeds.
ZJOLT_API ZJoltResult zjoltRagdollGetRootTransform(const ZJoltRagdoll *ragdoll,
                                                   ZJoltRVec3 *out_position,
                                                   ZJoltQuat *out_rotation,
                                                   bool lock_bodies);

/// Moves every part into collision group `group_id` (keeping sub-group
/// id and filter), changing what zjoltRagdollSettingsCreateRagdoll was given.
///
/// Two ragdolls sharing a DisableParentChildCollisions filter must not
/// hold the same id: a filter is consulted only within one group id, so
/// sharing lets every part of one ragdoll pass through the other's.
ZJOLT_API void zjoltRagdollSetGroupId(ZJoltRagdoll *ragdoll, uint32_t group_id,
                                      bool lock_bodies);

ZJOLT_API void zjoltRagdollActivate(ZJoltRagdoll *ragdoll, bool lock_bodies);

/// Every body of the ragdoll at once, which is what makes a ragdoll launch,
/// carry a dead runner's momentum, or take a hit as one thing rather than as
/// a list of parts a caller has to walk. Jolt's Ragdoll::SetLinearVelocity
/// and its three siblings; NULL vector is ZJOLT_RESULT_INVALID_ARGUMENT.
ZJOLT_API ZJoltResult zjoltRagdollSetLinearAndAngularVelocity(
    ZJoltRagdoll *ragdoll, const ZJoltVec3 *linear_velocity,
    const ZJoltVec3 *angular_velocity, bool lock_bodies);
ZJOLT_API ZJoltResult zjoltRagdollSetLinearVelocity(
    ZJoltRagdoll *ragdoll, const ZJoltVec3 *linear_velocity, bool lock_bodies);
ZJOLT_API ZJoltResult zjoltRagdollAddLinearVelocity(
    ZJoltRagdoll *ragdoll, const ZJoltVec3 *linear_velocity, bool lock_bodies);

/// Adds `impulse` at the centre of mass of EVERY body — the SAME impulse
/// once per body, not one impulse shared out across the ragdoll, so a heavier
/// part gains less velocity than a lighter one. Jolt's own AddImpulse.
ZJOLT_API ZJoltResult zjoltRagdollAddImpulse(
    ZJoltRagdoll *ragdoll, const ZJoltVec3 *impulse, bool lock_bodies);
ZJOLT_API bool zjoltRagdollIsActive(const ZJoltRagdoll *ragdoll,
                                    bool lock_bodies);

/// Clears warm-start impulses on every constraint. Call after
/// zjoltRagdollSetPose so the next step does not solve toward the
/// bodies' previous positions.
ZJOLT_API void zjoltRagdollResetWarmStart(ZJoltRagdoll *ragdoll);

/// Teleports every body to `pose` (SetPositionAndRotation, not activating).
/// `pose` must have been through zjoltSkeletonPoseCalculateJointMatrices
/// since its joints were last set, and its skeleton must be the same one
/// `pose`'s owning settings were built with — fails with
/// ZJOLT_RESULT_INVALID_ARGUMENT otherwise.
ZJOLT_API ZJoltResult zjoltRagdollSetPose(ZJoltRagdoll *ragdoll,
                                          const ZJoltSkeletonPose *pose,
                                          bool lock_bodies);

/// Fills `pose`'s joint matrices from the ragdoll's current body transforms.
/// `pose`'s skeleton must be the ragdoll's. This does NOT call
/// zjoltSkeletonPoseCalculateJointStates — call that afterwards before
/// reading `pose` back with zjoltSkeletonPoseGetJoints.
ZJOLT_API ZJoltResult zjoltRagdollGetPose(ZJoltRagdoll *ragdoll,
                                          ZJoltSkeletonPose *pose,
                                          bool lock_bodies);

/// Drives every body toward `pose` over `delta_time` by setting velocities
/// (BodyInterface::MoveKinematic) rather than teleporting — the right choice
/// for a kinematic ragdoll being animated rather than simulated. Same
/// zjoltSkeletonPoseCalculateJointMatrices and matching-skeleton requirement
/// as zjoltRagdollSetPose.
ZJOLT_API ZJoltResult zjoltRagdollDriveToPoseUsingKinematics(
    ZJoltRagdoll *ragdoll, const ZJoltSkeletonPose *pose, float delta_time,
    bool lock_bodies);

/// Drives every constrained body toward `pose` by activating that joint's
/// constraint motor in position mode. Reads `pose`'s LOCAL joint rotations
/// directly (zjoltSkeletonPoseSetJoints is enough; unlike zjoltRagdollSetPose
/// this does not need zjoltSkeletonPoseCalculateJointMatrices first). `pose`'s
/// skeleton must be the ragdoll's.
ZJOLT_API ZJoltResult zjoltRagdollDriveToPoseUsingMotors(
    ZJoltRagdoll *ragdoll, const ZJoltSkeletonPose *pose);

/// As zjoltRagdollDriveToPoseUsingMotors, but also drives each motor's target
/// velocity — computed from how `pose` differs from `prev_pose` over
/// `delta_time` — for a smoother follow. Both poses' skeletons must be the
/// ragdoll's; `delta_time` must be positive.
ZJOLT_API ZJoltResult zjoltRagdollDriveToPoseUsingMotorsWithVelocity(
    ZJoltRagdoll *ragdoll, const ZJoltSkeletonPose *prev_pose,
    const ZJoltSkeletonPose *pose, float delta_time);

#ifdef __cplusplus
}
#endif

#endif  // ZJOLT_RAGDOLL_H_
