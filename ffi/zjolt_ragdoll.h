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
// RagdollSettings
//===----------------------------------------------------------------------===//

/// Deprecated spellings of ZJoltSwingType and its enumerators, which
/// zjolt_constraint.h declares.
///
/// There was one enum per subsystem mirroring JPH::ESwingType, added by
/// separate changes, and a caller should not have to know which one an entry
/// point wants. These aliases keep existing code building; write ZJoltSwingType
/// in anything new.
#define ZJoltRagdollSwingType ZJoltSwingType
#define ZJOLT_RAGDOLL_SWING_TYPE_CONE ZJOLT_SWING_TYPE_CONE
#define ZJOLT_RAGDOLL_SWING_TYPE_PYRAMID ZJOLT_SWING_TYPE_PYRAMID

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
ZJOLT_API ZJoltResult zjoltRagdollGetBodyIds(const ZJoltRagdoll *ragdoll,
                                             ZJoltBodyId *out_ids,
                                             uint32_t capacity,
                                             uint32_t *out_count);

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
