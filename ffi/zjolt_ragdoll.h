//===----------------------------------------------------------------------===//
// zjolt — ragdolls: a skeleton, per-part bodies and constraints, driven or
// read back as a pose.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// A ragdoll is built from three pieces:
//
//   * ZJoltSkeleton    — the joint hierarchy: names and parent indices only,
//                        no transforms.
//   * ZJoltSkeletonPose — one instance of that hierarchy in a particular
//                        pose: a root offset plus, per joint, a local
//                        rotation/translation relative to its parent.
//   * ZJoltRagdollSettings — one JPH::BodyDesc-shaped part per joint, each
//                        optionally attached to its parent by a swing-twist
//                        constraint. This is the reusable template;
//                        zjoltRagdollSettingsCreateRagdoll spawns bodies and
//                        constraints from it, and may be called more than
//                        once to spawn more than one ragdoll.
//
// Unlike a shape's or a body's settings, RagdollSettings does not get built on
// the stack and discarded: a live ZJoltRagdoll keeps a reference to the
// settings that created it and consults it on every DriveToPoseUsingMotors
// call (to find which constraint belongs to which body), so it is a real,
// reference-counted, crossing handle like ZJoltSkeleton — not flattened away.
//
// A pose never carries a JPH::Mat44 across this boundary: the ABI has no
// matrix type, so zjoltRagdollGetPose/SetPose and DriveToPoseUsingKinematics
// work through the pose's flattened joint matrices internally, and the
// caller only ever reads or writes the pose's per-joint rotation/translation
// (zjoltSkeletonPoseGetJoints/SetJoints) plus its root offset. Convert
// between the two with zjoltSkeletonPoseCalculateJointMatrices (before
// driving) and zjoltSkeletonPoseCalculateJointStates (after reading back).
//
// ZJoltSkeletonMapper is the fourth piece, and it is what a ragdoll is
// usually for: it maps a pose between the low-detail skeleton the ragdoll
// simulates and the high-detail skeleton something is drawn from. It is the
// one type here whose Jolt signatures are all Mat44 arrays, and for the same
// reason as above it takes ZJoltSkeletonPose handles instead — a pose's
// joint matrices ARE the array Jolt asked for.
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

/// Appends one joint and reports its index through `out_index`.
///
/// `parent_index` is -1 for a root joint, or otherwise the index of a joint
/// ADDED EARLIER in the same skeleton. That is what every ragdoll operation
/// that walks the hierarchy requires (zjoltSkeletonAreJointsCorrectlyOrdered:
/// a parent's index below its child's), and it is enforced here rather than
/// left to be discovered later — an out-of-range or not-yet-added
/// `parent_index` is refused rather than indexing past what has been built so
/// far. `name` is copied and may be NULL for an unnamed joint.
ZJOLT_API ZJoltResult zjoltSkeletonAddJoint(ZJoltSkeleton *skeleton,
                                            const char *name,
                                            int32_t parent_index,
                                            uint32_t *out_index);

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
/// `zjoltSkeletonAddJoint` cannot build a skeleton that fails this, so the
/// only way to reach false is a skeleton restored or built some other way.
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

/// Converts the per-joint LOCAL rotations/translations set by
/// zjoltSkeletonPoseSetJoints into the flattened, root-relative form that
/// zjoltRagdollSetPose, zjoltRagdollGetPose and
/// zjoltRagdollDriveToPoseUsingKinematics consume — call this before any of
/// them. Requires the assigned skeleton's joints to be correctly ordered
/// (zjoltSkeletonAreJointsCorrectlyOrdered); fails with
/// ZJOLT_RESULT_INVALID_ARGUMENT rather than the assertion Jolt itself would
/// hit. Fails the same way if no skeleton is assigned.
ZJOLT_API ZJoltResult zjoltSkeletonPoseCalculateJointMatrices(
    ZJoltSkeletonPose *pose);

/// The reverse conversion: what zjoltRagdollGetPose leaves the caller
/// needing before zjoltSkeletonPoseGetJoints returns anything meaningful.
/// Fails with ZJOLT_RESULT_INVALID_ARGUMENT if no skeleton is assigned.
ZJOLT_API ZJoltResult zjoltSkeletonPoseCalculateJointStates(
    ZJoltSkeletonPose *pose);

//===----------------------------------------------------------------------===//
// SkeletonMapper
//
// One skeleton drives another. The ragdoll simulates a dozen-odd joints; the
// mesh is skinned to a hundred. zjoltSkeletonMapperInitialize works out which
// joints of the two correspond, from their NEUTRAL poses, and then
// zjoltSkeletonMapperMap pushes a simulated pose onto the render skeleton
// every frame — and zjoltSkeletonMapperMapReverse pushes an animated pose the
// other way, which is what drives a ragdoll from animation before it is let
// go.
//
// Every pose here is a ZJoltSkeletonPose, and WHICH of a pose's two forms is
// read differs per call, so it is spelled out on each one below. Briefly: a
// pose carries per-joint local rotation/translation (zjoltSkeletonPoseSetJoints
// and GetJoints) and, beside it, a flattened model-space form
// (zjoltSkeletonPoseCalculateJointMatrices builds it from the first,
// CalculateJointStates rebuilds the first from it). The mapper works in the
// model-space form, because that is the only space in which two skeletons of
// different shape mean the same thing.
//===----------------------------------------------------------------------===//

/// Whether joint `index1` of `skeleton1` is the same joint as `index2` of
/// `skeleton2`. Called during zjoltSkeletonMapperInitialize only, once per
/// candidate pair, with no lock held and nothing else in flight.
///
/// Pass NULL for Jolt's own default, which is that the two joint NAMES are
/// equal. Supply one when the two skeletons name their joints differently, or
/// when a name appears twice and the first match is the wrong one.
///
/// Nothing may unwind out of this — see the note on callbacks in BINDING.md.
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
/// poses, and is the call every other one below depends on.
///
/// `neutral1` is the LOW-detail skeleton — the one the ragdoll simulates —
/// and `neutral2` the HIGH-detail one. Jolt requires that ordering (it
/// assumes every joint of skeleton 1 maps to one of skeleton 2, and that
/// skeleton 2 is skeleton 1's hierarchy with extra joints between, above or
/// below); a `neutral1` with more joints than `neutral2` is refused rather
/// than left to map to nothing.
///
/// Both poses must be in MODEL space, meaning
/// zjoltSkeletonPoseCalculateJointMatrices has been called on each since its
/// joints were last set. That cannot be checked — a pose's matrices are
/// allocated by zjoltSkeletonPoseSetSkeleton and are uninitialised until
/// something writes them — and getting it wrong produces a mapper that maps
/// to nonsense rather than an error. The two neutral poses should also agree
/// as closely as possible: every mapped joint's transform is stored as the
/// difference between them, and a difference that is really a mistake is
/// applied to every frame afterwards.
///
/// The poses are read during this call only; neither is retained.
///
/// Refuses (ZJOLT_RESULT_INVALID_ARGUMENT) a mapper that has already been
/// initialised, a pose with no skeleton assigned, a skeleton whose joints are
/// not correctly ordered — zjoltSkeletonMapperMap requires a parent to come
/// before its children — and `neutral1` having more joints than `neutral2`.
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

/// Pins the translation of the named joints of skeleton 2 to their neutral
/// pose, so that only their rotation follows the ragdoll.
///
/// A constraint is never perfectly rigid, so a ragdoll under load stretches
/// at the joints; mapping that stretch onto the render skeleton stretches the
/// mesh with it. Locking removes the stretch at the cost of the drawn
/// skeleton no longer being exactly where the simulated one is.
///
/// `locked` has one bool per joint of `neutral2`, and `count` must equal its
/// joint count exactly. `neutral2` is the same MODEL-space neutral pose
/// zjoltSkeletonMapperInitialize was given, and is read during this call
/// only.
///
/// A joint with NO PARENT is refused rather than locked: its translation is
/// what positions the whole ragdoll, and Jolt's mapping step would resolve
/// its lock against parent joint -1. Jolt's own LockAllTranslations excludes
/// the root for the first of those reasons; this refuses it for the second.
///
/// Calls accumulate — locking is additive, and there is no unlock.
ZJOLT_API ZJoltResult zjoltSkeletonMapperLockTranslations(
    ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *neutral2,
    const bool *locked, uint32_t count);

/// zjoltSkeletonMapperLockTranslations for every joint of skeleton 2 below
/// the topmost mapped joint, that joint itself excluded. The usual choice:
/// everything the ragdoll drives stops stretching, and the ragdoll is still
/// free to move as a whole.
///
/// Refuses a mapper that has not been through zjoltSkeletonMapperInitialize —
/// there is no topmost mapped joint before that, which Jolt only asserts.
ZJOLT_API ZJoltResult zjoltSkeletonMapperLockAllTranslations(
    ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *neutral2);

/// Whether zjoltSkeletonMapperLockTranslations or LockAllTranslations locked
/// joint `joint2_index` of skeleton 2. False if `mapper` is NULL.
ZJOLT_API bool zjoltSkeletonMapperIsJointTranslationLocked(
    const ZJoltSkeletonMapper *mapper, uint32_t joint2_index);

/// Pushes `pose1` onto `pose2`: the frame call.
///
/// Reads `pose1`'s MODEL-space joint matrices — so the pose
/// zjoltRagdollGetPose just filled in, with no conversion in between — and
/// `pose2`'s LOCAL joint rotations/translations, the ones
/// zjoltSkeletonPoseSetJoints writes. Writes `pose2`'s MODEL-space joint
/// matrices. Read them back with zjoltSkeletonPoseCalculateJointStates
/// followed by zjoltSkeletonPoseGetJoints.
///
/// `pose2`'s local joints are read because joints of skeleton 2 that the
/// ragdoll does not drive have to come from somewhere: they keep their own
/// local transform relative to whatever their parent ended up at. So `pose2`
/// carries the animation, and this overwrites the part of it the ragdoll owns.
///
/// `pose1`'s ROOT OFFSET is copied onto `pose2` as well. Jolt's own Map does
/// not do that, and leaving it out is the trap it looks like: a pose's joint
/// matrices are relative to its root offset, and zjoltRagdollGetPose puts the
/// ragdoll's world position in the offset rather than in the first matrix, so
/// that a 32-bit matrix never has to hold a double-precision world position.
/// A target pose left at its own offset draws the character correctly posed,
/// at the origin. Set a different offset afterwards if that is really wanted.
///
/// Both poses must belong to the skeletons zjoltSkeletonMapperInitialize was
/// given. What is actually checked is that every joint index the mapper holds
/// is inside both poses, which is what stops this writing past the end of one;
/// a pose of the wrong skeleton with enough joints maps to nonsense rather
/// than being caught.
ZJOLT_API ZJoltResult zjoltSkeletonMapperMap(const ZJoltSkeletonMapper *mapper,
                                             const ZJoltSkeletonPose *pose1,
                                             ZJoltSkeletonPose *pose2);

/// The other direction: reads `pose2`'s MODEL-space joint matrices and writes
/// `pose1`'s. What drives a ragdoll from animation — sample the animation
/// into `pose2`, run zjoltSkeletonPoseCalculateJointMatrices on it, map back,
/// then zjoltRagdollDriveToPoseUsingKinematics or SetPose with `pose1`.
///
/// `pose2`'s root offset travels onto `pose1` too, as in zjoltSkeletonMapperMap.
///
/// Only the joints that mapped one-to-one are written; chains and unmapped
/// joints are not, because the correspondence they describe does not run
/// backwards. Every joint of skeleton 1 normally maps, which is the case Jolt
/// assumes — but a joint of `pose1` that did NOT map keeps whatever matrix it
/// already held, which is uninitialised memory in a pose nothing has written.
/// Check zjoltSkeletonMapperGetMappingCount against `pose1`'s joint count if
/// that matters.
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

/// Gives each part's constraint a solver priority, counting UP toward the
/// root: a joint nearer the root is solved before one nearer a leaf, so a
/// shoulder under load stops fighting the wrist hanging off it. Call it after
/// zjoltRagdollSettingsBuild, like Stabilize; a ragdoll spawned afterwards
/// carries the priorities, and one spawned before does not.
///
/// `base_priority` is the LOWEST priority used, given to the leaf-most
/// constraints; everything else counts up from there. 0 unless the ragdoll's
/// constraints have to sort against other constraints in the same system,
/// which is the only reason to pass anything else — see
/// zjoltConstraintSetPriority for what the number means.
///
/// Fails with ZJOLT_RESULT_INVALID_ARGUMENT if the settings have not been
/// built, if the skeleton's joints are not correctly ordered, or if
/// `base_priority` plus the part count would overflow.
ZJOLT_API ZJoltResult zjoltRagdollSettingsCalculateConstraintPriorities(
    ZJoltRagdollSettings *settings, uint32_t base_priority);

/// Rebalances part mass and inertia along parent-child chains so the solver
/// stays stable. Returns false if a chain's inertia tensor could not be
/// decomposed (rare); false if `settings` is NULL.
ZJOLT_API bool zjoltRagdollSettingsStabilize(ZJoltRagdollSettings *settings);

/// Builds a shared collision-group filter that disables collisions between
/// every part and its parent, and assigns it (with a per-joint sub-group) to
/// every part. Pass the resulting collision group's id to every ragdoll
/// spawned from these settings — see zjoltRagdollSettingsCreateRagdoll.
///
/// Unlike Jolt's own DisableParentChildCollisions, this never runs the
/// optional bind-pose collision pass: only joints adjacent in the hierarchy
/// are disabled, not joints that happen to overlap in the rest pose. That
/// pass wants the bind pose as an array of matrices, and nothing here takes
/// one yet.
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

/// Spawns one body per part and one constraint per non-root part, in
/// `system` but NOT YET added to its simulation — see
/// zjoltRagdollAddToPhysicsSystem. `settings` is left untouched and may be
/// used again to spawn more ragdolls from the same template.
///
/// `collision_group` is the id every spawned body's collision group is set
/// to; give each ragdoll spawned from settings sharing a
/// zjoltRagdollSettingsDisableParentChildCollisions filter a DIFFERENT one; a
/// filter is only ever consulted within one group id.
///
/// Fails with ZJOLT_RESULT_OUT_OF_MEMORY if `system` has no room left for
/// the new bodies, or if the handle itself could not be allocated.
ZJOLT_API ZJoltResult zjoltRagdollSettingsCreateRagdoll(
    const ZJoltRagdollSettings *settings, ZJoltPhysicsSystem *system,
    uint32_t collision_group, uint64_t user_data, ZJoltRagdoll **out);

//===----------------------------------------------------------------------===//
// Ragdoll
//===----------------------------------------------------------------------===//

ZJOLT_API void zjoltRagdollAddRef(const ZJoltRagdoll *ragdoll);

/// Drops one reference. The last one takes the ragdoll back out of its
/// physics system first if it is still in it — constraints and bodies both,
/// with body locking — and then destroys every body it owns.
///
/// Call order is therefore free: releasing a ragdoll that is still added is
/// as correct as removing it first, and having removed it first does not
/// make the release remove it twice. That is this library's doing rather
/// than Jolt's. `~Ragdoll` destroys the bodies where they stand and cannot
/// remove them itself, because a JPH::Ragdoll does not expose the system it
/// was spawned in; ZJoltRagdoll keeps it, and asks whether the ragdoll's
/// first body is added — the question zjoltBodyIsAdded answers — before
/// removing anything. That check is not a nicety: removing a ragdoll that
/// was never added is itself an error in Jolt, since the constraints go
/// first and Jolt asserts each one was added.
///
/// zjoltRagdollRemoveFromPhysicsSystem remains the call for taking a ragdoll
/// OUT of the simulation while keeping it alive.
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

/// The body ids of the ragdoll's parts, indexed by joint — so index `i` is
/// the body built from skeleton joint `i`, and
/// zjoltSkeletonGetJoints/zjoltSkeletonGetJointName name it.
///
/// This is how a contact, a ray hit or a constraint gets mapped back to a
/// limb: shoot a ragdoll and the hit reports a ZJoltBodyId, and without this
/// there is no way to say which part of it was hit. It is also what
/// zjoltBodyInterface* needs to reach one part on its own — to attach a rope
/// to a hand, or to make one limb heavier.
///
/// Two-call protocol, as zjoltSkeletonPoseGetJoints: `*out_count` always
/// receives the body count, and a NULL array with any capacity is a size
/// query. A capacity below the count reports
/// ZJOLT_RESULT_BUFFER_TOO_SMALL and writes nothing.
///
/// The ids stay valid until the ragdoll's last reference drops; they name
/// bodies the ragdoll owns, so do not destroy one through
/// zjoltBodyInterfaceDestroyBody.
///
/// These ids are also how a ragdoll gets a constraint that is NOT one of the
/// parent-child joints — Jolt calls those additional constraints, and they are
/// what closes a kinematic loop: two hands tied together, a strap across a
/// chest. Build one with any zjoltConstraintCreate* between two of these ids
/// and zjoltConstraintAdd it, which offers the whole constraint zoo rather
/// than only the swing-twist a ragdoll part joint is. It is yours, though, not
/// the ragdoll's: zjoltRagdollAddToPhysicsSystem and
/// RemoveFromPhysicsSystem do not move it, zjoltRagdollGetConstraintCount does
/// not count it, and it must be removed before the ragdoll's last release
/// destroys the bodies underneath it.
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
/// Borrowed, and NOT AddRef'd: it is valid until the ragdoll's last reference
/// drops, and the ragdoll owns it — do not zjoltConstraintRelease it, and do
/// not zjoltConstraintRemove it from the system by hand
/// (zjoltRagdollRemoveFromPhysicsSystem moves the whole ragdoll as one batch,
/// and a constraint already removed is removed twice). Call zjoltConstraintAddRef
/// if it has to outlive your reference to the ragdoll.
///
/// Constraints are in joint-index order, skipping the root — so for a
/// skeleton whose joints are correctly ordered, constraint `i` is the one
/// attaching the `i`-th non-root joint to its parent.
///
/// This is what makes the zjoltConstraint* surface reachable on a ragdoll:
/// retuning one joint's motor (zjoltSwingTwistConstraintSetSwingMotorSettings),
/// disabling a joint (zjoltConstraintSetEnabled), or reading how much force a
/// joint is taking (zjoltSwingTwistConstraintGetTotalLambdaPosition) to break
/// a limb off. Every ragdoll part joint this ABI builds is a swing-twist
/// constraint.
ZJOLT_API ZJoltConstraint *zjoltRagdollGetConstraint(ZJoltRagdoll *ragdoll,
                                                     uint32_t index);

/// Where the ragdoll is: the world transform of the body built from the
/// skeleton's first joint. What a camera, a footstep or a distance cull wants
/// without reading every part back.
///
/// Fails with ZJOLT_RESULT_INVALID_ARGUMENT for a ragdoll with no parts (a
/// skeleton with no joints), which Jolt itself would index past. When the
/// body lock fails instead — the one silent path Jolt keeps here — the
/// outputs are a zero position and an identity rotation, and the call still
/// reports success, because Jolt does not say which happened.
ZJOLT_API ZJoltResult zjoltRagdollGetRootTransform(const ZJoltRagdoll *ragdoll,
                                                   ZJoltRVec3 *out_position,
                                                   ZJoltQuat *out_rotation,
                                                   bool lock_bodies);

/// Moves every part into collision group `group_id`, keeping each part's
/// sub-group id and group filter.
///
/// The id zjoltRagdollSettingsCreateRagdoll was given, changed afterwards.
/// Two ragdolls spawned from settings that share a
/// zjoltRagdollSettingsDisableParentChildCollisions filter must not hold the
/// same id — a filter is only ever consulted within one group id, so sharing
/// one makes every part of one ragdoll pass through every part of the other.
/// So this is the call for reusing a pooled ragdoll under a fresh id, and the
/// call that silently breaks a pool if the fresh id is not fresh.
ZJOLT_API void zjoltRagdollSetGroupId(ZJoltRagdoll *ragdoll, uint32_t group_id,
                                      bool lock_bodies);

ZJOLT_API void zjoltRagdollActivate(ZJoltRagdoll *ragdoll, bool lock_bodies);
ZJOLT_API bool zjoltRagdollIsActive(const ZJoltRagdoll *ragdoll,
                                    bool lock_bodies);

/// Clears warm-start impulses on every constraint. Call after
/// zjoltRagdollSetPose so the next step does not solve toward where the
/// bodies used to be.
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
