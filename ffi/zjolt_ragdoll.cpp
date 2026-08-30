//===----------------------------------------------------------------------===//
// zjolt — ragdolls.
//
// A few conversions are duplicated from zjolt_body.cpp/zjolt_internal.h
// rather than shared, since this subsystem is scoped to add files and append registrations only — not to edit the existing translation units they would otherwise join.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/ObjectStream/ObjectStreamOut.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonMapper.h>
#include <Jolt/Skeleton/SkeletonPose.h>

#include <atomic>

//===----------------------------------------------------------------------===//
// Opaque handle mapping
//
// Three of the five are reinterpret_cast tags directly onto the Jolt type, like ZJoltShape/ZJoltPhysicsMaterial in zjolt_internal.h. Skeleton/RagdollSettings are reference counted; SkeletonPose is a plain value type, allocated/freed like a job system.
// The other two are not tags — see "The handle" and "The mapper handle" below for why.
//===----------------------------------------------------------------------===//

namespace zjolt {

inline const JPH::Skeleton *ToJolt(const ZJoltSkeleton *s) {
  return reinterpret_cast<const JPH::Skeleton *>(s);
}
inline JPH::Skeleton *ToJolt(ZJoltSkeleton *s) {
  return reinterpret_cast<JPH::Skeleton *>(s);
}
inline ZJoltSkeleton *ToC(JPH::Skeleton *s) {
  return reinterpret_cast<ZJoltSkeleton *>(s);
}
inline const ZJoltSkeleton *ToC(const JPH::Skeleton *s) {
  return reinterpret_cast<const ZJoltSkeleton *>(s);
}

inline const JPH::SkeletonPose *ToJolt(const ZJoltSkeletonPose *p) {
  return reinterpret_cast<const JPH::SkeletonPose *>(p);
}
inline JPH::SkeletonPose *ToJolt(ZJoltSkeletonPose *p) {
  return reinterpret_cast<JPH::SkeletonPose *>(p);
}
inline ZJoltSkeletonPose *ToC(JPH::SkeletonPose *p) {
  return reinterpret_cast<ZJoltSkeletonPose *>(p);
}

inline const JPH::SkeletalAnimation *ToJolt(const ZJoltSkeletalAnimation *a) {
  return reinterpret_cast<const JPH::SkeletalAnimation *>(a);
}
inline JPH::SkeletalAnimation *ToJolt(ZJoltSkeletalAnimation *a) {
  return reinterpret_cast<JPH::SkeletalAnimation *>(a);
}
inline ZJoltSkeletalAnimation *ToC(JPH::SkeletalAnimation *a) {
  return reinterpret_cast<ZJoltSkeletalAnimation *>(a);
}
inline const ZJoltSkeletalAnimation *ToC(const JPH::SkeletalAnimation *a) {
  return reinterpret_cast<const ZJoltSkeletalAnimation *>(a);
}

inline const JPH::RagdollSettings *ToJolt(const ZJoltRagdollSettings *s) {
  return reinterpret_cast<const JPH::RagdollSettings *>(s);
}
inline JPH::RagdollSettings *ToJolt(ZJoltRagdollSettings *s) {
  return reinterpret_cast<JPH::RagdollSettings *>(s);
}
inline ZJoltRagdollSettings *ToC(JPH::RagdollSettings *s) {
  return reinterpret_cast<ZJoltRagdollSettings *>(s);
}
inline const ZJoltRagdollSettings *ToC(const JPH::RagdollSettings *s) {
  return reinterpret_cast<const ZJoltRagdollSettings *>(s);
}

// ZJoltSkeletonMapper is deliberately absent here — it is not a tag; see The
// mapper handle, below.

// The one conversion in this file that is not for a handle this file
// introduces: zjoltRagdollGetConstraint hands back a constraint of
// zjolt_constraint.cpp's, and this is that file's ToC, character for
// character. Same reason as the file comment above — this subsystem adds
// files rather than editing the shared header the pair belongs in.
inline ZJoltConstraint *ToC(JPH::Constraint *constraint) {
  return reinterpret_cast<ZJoltConstraint *>(constraint);
}

}  // namespace zjolt

//===----------------------------------------------------------------------===//
// The handle
//
// Not a tag like the others: releasing the last reference must take its bodies out of the broad phase first, needing the PhysicsSystem — `owner` holds it (`Ragdoll::mSystem` has no getter).
// `refs` counts the HANDLE's references, not JPH::RefTarget's: two concurrent Release calls must resolve race-free, which RefTarget's own count cannot (`impl` pins Jolt's reference at one for the handle's lifetime).
//===----------------------------------------------------------------------===//

struct ZJoltRagdoll {
  JPH::Ref<JPH::Ragdoll> impl;
  ZJoltPhysicsSystem *owner;

  /// Mutable for the same reason JPH::RefTarget's own count is: AddRef and
  /// Release take a `const ZJoltRagdoll *`, because holding a reference to a
  /// ragdoll is not modifying it.
  mutable std::atomic<uint32_t> refs{1};
};

//===----------------------------------------------------------------------===//
// The mapper handle
//
// Not a tag either, for a sharper reason: JPH::SkeletonMapper's Release ends in a GLOBAL `delete this` on a block zjolt::New took from the HOST allocator — heap corruption a test might not catch. So the count is the handle's; this file constructs/destroys the member directly and never AddRefs it, keeping Jolt's own count at zero.
// NOT counted by zjoltLiveHandleCount, like ZJoltSkeletonPose.
//===----------------------------------------------------------------------===//

struct ZJoltSkeletonMapper {
  JPH::SkeletonMapper impl;

  /// Mutable for the same reason JPH::RefTarget's own count is: AddRef and
  /// Release take a `const ZJoltSkeletonMapper *`.
  mutable std::atomic<uint32_t> refs{1};
};

namespace {

//===----------------------------------------------------------------------===//
// Unwrapping
//===----------------------------------------------------------------------===//

/// The Jolt ragdoll behind a handle, or NULL for a NULL handle. `impl` is
/// cleared only between the last two statements of zjoltRagdollRelease, so
/// for any handle a caller can still hold this is NULL exactly when
/// `ragdoll` is.
JPH::Ragdoll *Impl(ZJoltRagdoll *ragdoll) {
  return ragdoll != nullptr ? ragdoll->impl.GetPtr() : nullptr;
}
const JPH::Ragdoll *Impl(const ZJoltRagdoll *ragdoll) {
  return ragdoll != nullptr ? ragdoll->impl.GetPtr() : nullptr;
}

/// The Jolt mapper inside a handle, or NULL for a NULL handle. A member
/// rather than a pointer, so this is an address rather than a load.
JPH::SkeletonMapper *Impl(ZJoltSkeletonMapper *mapper) {
  return mapper != nullptr ? &mapper->impl : nullptr;
}
const JPH::SkeletonMapper *Impl(const ZJoltSkeletonMapper *mapper) {
  return mapper != nullptr ? &mapper->impl : nullptr;
}

//===----------------------------------------------------------------------===//
// Small conversions duplicated from zjolt_body.cpp — see the file comment.
//===----------------------------------------------------------------------===//

// Takes the raw integer, not the enum — see zjolt::RawEnum in
// zjolt_internal.h.
JPH::EActivation ToJoltActivation(int32_t activation) {
  return activation == ZJOLT_ACTIVATION_DONT_ACTIVATE
             ? JPH::EActivation::DontActivate
             : JPH::EActivation::Activate;
}

// Takes the raw integer, not the enum — see zjolt::RawEnum in
// zjolt_internal.h. Its one caller reads this straight out of a host-supplied
// ZJoltRagdollConstraintDesc, which is the entry point this value arrives
// from.
JPH::ESwingType ToJoltSwingType(int32_t type) {
  return type == ZJOLT_SWING_TYPE_PYRAMID ? JPH::ESwingType::Pyramid
                                          : JPH::ESwingType::Cone;
}

/// Builds a swing-twist constraint settings object on the heap — must outlive
/// this call, unlike the parts' BodyCreationSettings above, since
/// RagdollSettings::Part::mToParent keeps a reference to it. Never crosses the
/// ABI: assigned straight into mToParent, which takes its own reference on
/// assignment. Returns NULL only on allocation failure.
JPH::SwingTwistConstraintSettings *BuildConstraint(
    const ZJoltRagdollConstraintDesc &desc) {
  JPH::SwingTwistConstraintSettings *settings =
      zjolt::New<JPH::SwingTwistConstraintSettings>();
  if (settings == nullptr) return nullptr;

  settings->mSpace = JPH::EConstraintSpace::WorldSpace;
  settings->mPosition1 = zjolt::ToJoltR(desc.position1);
  settings->mTwistAxis1 = zjolt::ToJolt(desc.twist_axis1);
  settings->mPlaneAxis1 = zjolt::ToJolt(desc.plane_axis1);
  settings->mPosition2 = zjolt::ToJoltR(desc.position2);
  settings->mTwistAxis2 = zjolt::ToJolt(desc.twist_axis2);
  settings->mPlaneAxis2 = zjolt::ToJolt(desc.plane_axis2);
  settings->mSwingType = ToJoltSwingType(zjolt::RawEnum(desc.swing_type));
  settings->mNormalHalfConeAngle = desc.normal_half_cone_angle;
  settings->mPlaneHalfConeAngle = desc.plane_half_cone_angle;
  settings->mTwistMinAngle = desc.twist_min_angle;
  settings->mTwistMaxAngle = desc.twist_max_angle;
  settings->mMaxFrictionTorque = desc.max_friction_torque;
  return settings;
}

/// Everything zjoltRagdollSettingsBuild must reject BEFORE mutating
/// `settings`, so a rejected call leaves it unchanged. The to_parent check
/// is the one CreateRagdoll skips: it indexes
/// `bodies[...mParentJointIndex]` for every part with a non-null mToParent,
/// so a root part (index -1) carrying one reads `bodies[-1]` — out of
/// bounds, per Ragdoll.cpp's CreateRagdoll, not any JPH_ASSERT.
ZJoltResult ValidatePart(const JPH::Skeleton &skeleton,
                         const ZJoltRagdollPartDesc &part,
                         uint32_t joint_index) {
  if (part.body.shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a ragdoll part needs a shape");
  }
  if (zjolt::RawEnum(part.body.override_mass_properties) ==
          ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_INERTIA &&
      !(part.body.mass > 0.0f)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "override_mass_properties asks for an explicit mass, so mass must "
        "be positive");
  }

  const bool is_root =
      skeleton.GetJoint(static_cast<int>(joint_index)).mParentJointIndex < 0;
  const bool has_to_parent = part.to_parent != nullptr;
  if (is_root == has_to_parent) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "to_parent must be NULL exactly for the root joint");
  }
  return ZJOLT_RESULT_OK;
}

/// How many joints a pose of skeleton 1 and skeleton 2 must have for the
/// mapper to stay inside both of them. SkeletonMapper keeps only joint
/// indices after Initialize, not the skeletons it was built from — this
/// stands in for "is this the right pose". Every index it can reach: a
/// mapping's pair, a chain's two index runs, an unmapped joint, or a locked
/// joint and its parent.
void MapperExtent(const JPH::SkeletonMapper &mapper, uint32_t *out_needed1,
                  uint32_t *out_needed2) {
  int needed1 = 0;
  int needed2 = 0;
  auto note1 = [&needed1](int index) {
    if (index + 1 > needed1) needed1 = index + 1;
  };
  auto note2 = [&needed2](int index) {
    if (index + 1 > needed2) needed2 = index + 1;
  };

  for (const JPH::SkeletonMapper::Mapping &m : mapper.GetMappings()) {
    note1(m.mJointIdx1);
    note2(m.mJointIdx2);
  }
  for (const JPH::SkeletonMapper::Chain &c : mapper.GetChains()) {
    for (int index : c.mJointIndices1) note1(index);
    for (int index : c.mJointIndices2) note2(index);
  }
  for (const JPH::SkeletonMapper::Unmapped &u : mapper.GetUnmapped()) {
    note2(u.mJointIdx);
  }
  for (const JPH::SkeletonMapper::Locked &l : mapper.GetLockedTranslations()) {
    note2(l.mJointIdx);
    note2(l.mParentJointIdx);
  }

  *out_needed1 = static_cast<uint32_t>(needed1);
  *out_needed2 = static_cast<uint32_t>(needed2);
}

/// Everything zjoltSkeletonMapperMap and MapReverse check before letting Jolt
/// index either pose.
ZJoltResult PosesFitMapper(const JPH::SkeletonMapper &mapper,
                           const JPH::SkeletonPose &pose1,
                           const JPH::SkeletonPose &pose2) {
  if (mapper.GetMappings().empty()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "the mapper has no mapped joints; call zjoltSkeletonMapperInitialize "
        "first, and check zjoltSkeletonMapperGetMappingCount if it has run");
  }

  uint32_t needed1 = 0;
  uint32_t needed2 = 0;
  MapperExtent(mapper, &needed1, &needed2);
  if (pose1.GetJointCount() < needed1 || pose2.GetJointCount() < needed2) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a pose has fewer joints than the mapper indexes, so it is not a pose "
        "of the skeleton zjoltSkeletonMapperInitialize was given");
  }
  return ZJOLT_RESULT_OK;
}

/// The neutral pose every SkeletonMapper setup call takes, checked once.
/// `out_skeleton` receives its skeleton.
ZJoltResult NeutralPoseIsUsable(const JPH::SkeletonPose &pose,
                                const JPH::Skeleton **out_skeleton) {
  const JPH::Skeleton *skeleton = pose.GetSkeleton();
  if (skeleton == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a neutral pose has no skeleton assigned");
  }
  if (!skeleton->AreJointsCorrectlyOrdered()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a neutral pose's skeleton has joints that are not correctly ordered, "
        "and mapping requires a parent to come before its children");
  }
  *out_skeleton = skeleton;
  return ZJOLT_RESULT_OK;
}

/// True when `pose`'s skeleton is the same object `ragdoll` was built from —
/// the precondition every SetPose/GetPose/DriveToPose* overload below
/// JPH_ASSERTs on rather than checking.
bool SkeletonMatches(const JPH::Ragdoll &ragdoll,
                     const JPH::SkeletonPose &pose) {
  return pose.GetSkeleton() == ragdoll.GetRagdollSettings()->GetSkeleton();
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Skeleton
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSkeletonCreate(ZJoltSkeleton **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Skeleton *fresh = zjolt::New<JPH::Skeleton>();
  if (fresh == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate a skeleton");
  }
  *out = zjolt::ToC(zjolt::Own(fresh));
  return ZJOLT_RESULT_OK;
}

void zjoltSkeletonAddRef(const ZJoltSkeleton *skeleton) {
  if (skeleton == nullptr) return;
  zjolt::ToJolt(skeleton)->AddRef();
}

void zjoltSkeletonRelease(const ZJoltSkeleton *skeleton) {
  if (skeleton == nullptr) return;
  zjolt::ToJolt(skeleton)->Release();
}

uint32_t zjoltSkeletonGetRefCount(const ZJoltSkeleton *skeleton) {
  if (skeleton == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(skeleton)->GetRefCount());
}

ZJoltResult zjoltSkeletonAddJoint(ZJoltSkeleton *skeleton, const char *name,
                                  int32_t parent_index, uint32_t *out_index) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out_index, (uint32_t)0xffffffffu));
  if (!zjolt::Present(skeleton, out_index)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Skeleton *s = zjolt::ToJolt(skeleton);
  const int32_t joint_count = static_cast<int32_t>(s->GetJointCount());
  if (parent_index < -1 || parent_index >= joint_count) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "parent_index must be -1 or the index of an already-added joint");
  }

  *out_index =
      s->AddJoint(JPH::string_view(name != nullptr ? name : ""),
                 static_cast<int>(parent_index));
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltSkeletonGetJointCount(const ZJoltSkeleton *skeleton) {
  if (skeleton == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(skeleton)->GetJointCount());
}

int32_t zjoltSkeletonGetJointIndex(const ZJoltSkeleton *skeleton,
                                   const char *name) {
  if (skeleton == nullptr || name == nullptr) return -1;
  return zjolt::ToJolt(skeleton)->GetJointIndex(JPH::string_view(name));
}

int32_t zjoltSkeletonGetJointParentIndex(const ZJoltSkeleton *skeleton,
                                         uint32_t index) {
  if (skeleton == nullptr) return -1;
  const JPH::Skeleton *s = zjolt::ToJolt(skeleton);
  if (index >= static_cast<uint32_t>(s->GetJointCount())) return -1;
  return s->GetJoint(static_cast<int>(index)).mParentJointIndex;
}

const char *zjoltSkeletonGetJointName(const ZJoltSkeleton *skeleton,
                                      uint32_t index) {
  if (skeleton == nullptr) return "";
  const JPH::Skeleton *s = zjolt::ToJolt(skeleton);
  if (index >= static_cast<uint32_t>(s->GetJointCount())) return "";
  return s->GetJoint(static_cast<int>(index)).mName.c_str();
}

bool zjoltSkeletonAreJointsCorrectlyOrdered(const ZJoltSkeleton *skeleton) {
  if (skeleton == nullptr) return false;
  return zjolt::ToJolt(skeleton)->AreJointsCorrectlyOrdered();
}

//===----------------------------------------------------------------------===//
// SkeletonPose
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSkeletonPoseCreate(ZJoltSkeletonPose **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SkeletonPose *fresh = zjolt::New<JPH::SkeletonPose>();
  if (fresh == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate a skeleton pose");
  }
  *out = zjolt::ToC(fresh);
  return ZJOLT_RESULT_OK;
}

void zjoltSkeletonPoseDestroy(ZJoltSkeletonPose *pose) {
  zjolt::Delete(zjolt::ToJolt(pose));
}

ZJoltResult zjoltSkeletonPoseSetSkeleton(ZJoltSkeletonPose *pose,
                                         const ZJoltSkeleton *skeleton) {
  ZJOLT_ENTER();
  if (!zjolt::Present(pose, skeleton)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::ToJolt(pose)->SetSkeleton(zjolt::ToJolt(skeleton));
  return ZJOLT_RESULT_OK;
}

const ZJoltSkeleton *zjoltSkeletonPoseGetSkeleton(
    const ZJoltSkeletonPose *pose) {
  if (pose == nullptr) return nullptr;
  return zjolt::ToC(zjolt::ToJolt(pose)->GetSkeleton());
}

uint32_t zjoltSkeletonPoseGetJointCount(const ZJoltSkeletonPose *pose) {
  if (pose == nullptr) return 0;
  return zjolt::ToJolt(pose)->GetJointCount();
}

void zjoltSkeletonPoseSetRootOffset(ZJoltSkeletonPose *pose,
                                    const ZJoltRVec3 *offset) {
  if (pose == nullptr || offset == nullptr) return;
  zjolt::ToJolt(pose)->SetRootOffset(zjolt::ToJoltR(*offset));
}

void zjoltSkeletonPoseGetRootOffset(const ZJoltSkeletonPose *pose,
                                    ZJoltRVec3 *out) {
  if (out == nullptr) return;
  if (pose == nullptr) {
    *out = ZJoltRVec3{0, 0, 0};
    return;
  }
  *out = zjolt::ToCR(zjolt::ToJolt(pose)->GetRootOffset());
}

ZJoltResult zjoltSkeletonPoseSetJoints(ZJoltSkeletonPose *pose,
                                       const ZJoltQuat *rotations,
                                       const ZJoltVec3 *translations,
                                       uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(pose)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  if (count != p->GetJointCount()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "count must equal the pose's joint count");
  }

  JPH::SkeletonPose::JointStateVector &joints = p->GetJoints();
  for (uint32_t i = 0; i < count; ++i) {
    if (rotations != nullptr) joints[i].mRotation = zjolt::ToJoltRotation(rotations[i]);
    if (translations != nullptr) joints[i].mTranslation = zjolt::ToJolt(translations[i]);
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSkeletonPoseGetJoints(const ZJoltSkeletonPose *pose,
                                       ZJoltQuat *out_rotations,
                                       ZJoltVec3 *out_translations,
                                       uint32_t capacity,
                                       uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(pose, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  const uint32_t count = p->GetJointCount();
  *out_count = count;
  if (out_rotations == nullptr && out_translations == nullptr) {
    return ZJOLT_RESULT_OK;
  }
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  const JPH::SkeletonPose::JointStateVector &joints = p->GetJoints();
  for (uint32_t i = 0; i < count; ++i) {
    if (out_rotations != nullptr) out_rotations[i] = zjolt::ToC(joints[i].mRotation);
    if (out_translations != nullptr) out_translations[i] = zjolt::ToC(joints[i].mTranslation);
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSkeletonPoseCalculateJointMatrices(ZJoltSkeletonPose *pose) {
  ZJOLT_ENTER();
  if (!zjolt::Present(pose)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  const JPH::Skeleton *skeleton = p->GetSkeleton();
  if (skeleton == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the pose has no skeleton assigned");
  }
  if (!skeleton->AreJointsCorrectlyOrdered()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "the pose's skeleton has joints that are not correctly ordered");
  }
  p->CalculateJointMatrices();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSkeletonPoseCalculateJointStates(ZJoltSkeletonPose *pose) {
  ZJOLT_ENTER();
  if (!zjolt::Present(pose)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  if (p->GetSkeleton() == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the pose has no skeleton assigned");
  }
  p->CalculateJointStates();
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// SkeletalAnimation
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSkeletalAnimationCreate(ZJoltSkeletalAnimation **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SkeletalAnimation *fresh = zjolt::New<JPH::SkeletalAnimation>();
  if (fresh == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate a skeletal animation");
  }
  *out = zjolt::ToC(zjolt::Own(fresh));
  return ZJOLT_RESULT_OK;
}

void zjoltSkeletalAnimationAddRef(const ZJoltSkeletalAnimation *animation) {
  if (animation == nullptr) return;
  zjolt::ToJolt(animation)->AddRef();
}

void zjoltSkeletalAnimationRelease(const ZJoltSkeletalAnimation *animation) {
  if (animation == nullptr) return;
  zjolt::ToJolt(animation)->Release();
}

uint32_t zjoltSkeletalAnimationGetRefCount(
    const ZJoltSkeletalAnimation *animation) {
  if (animation == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(animation)->GetRefCount());
}

ZJoltResult zjoltSkeletalAnimationAddAnimatedJoint(
    ZJoltSkeletalAnimation *animation, const char *name, uint32_t *out_index) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out_index, (uint32_t)0xffffffffu));
  if (!zjolt::Present(animation, out_index)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SkeletalAnimation::AnimatedJointVector &joints =
      zjolt::ToJolt(animation)->GetAnimatedJoints();
  JPH::SkeletalAnimation::AnimatedJoint fresh;
  fresh.mJointName = name != nullptr ? name : "";
  joints.push_back(fresh);
  *out_index = static_cast<uint32_t>(joints.size() - 1);
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltSkeletalAnimationGetAnimatedJointCount(
    const ZJoltSkeletalAnimation *animation) {
  if (animation == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(animation)->GetAnimatedJoints().size());
}

const char *zjoltSkeletalAnimationGetAnimatedJointName(
    const ZJoltSkeletalAnimation *animation, uint32_t joint_index) {
  if (animation == nullptr) return "";
  const JPH::SkeletalAnimation::AnimatedJointVector &joints =
      zjolt::ToJolt(animation)->GetAnimatedJoints();
  if (joint_index >= static_cast<uint32_t>(joints.size())) return "";
  return joints[joint_index].mJointName.c_str();
}

ZJoltResult zjoltSkeletalAnimationAddKeyframe(
    ZJoltSkeletalAnimation *animation, uint32_t joint_index, float time,
    const ZJoltQuat *rotation, const ZJoltVec3 *translation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(animation, rotation, translation)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  JPH::SkeletalAnimation::AnimatedJointVector &joints =
      zjolt::ToJolt(animation)->GetAnimatedJoints();
  if (joint_index >= static_cast<uint32_t>(joints.size())) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "joint_index is out of range");
  }

  JPH::SkeletalAnimation::KeyframeVector &keyframes = joints[joint_index].mKeyframes;
  // Sample's binary search over a joint's keyframes assumes ascending mTime;
  // Jolt does not check this itself.
  if (!keyframes.empty() && time < keyframes.back().mTime) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "keyframes must be added in non-decreasing time order");
  }

  JPH::SkeletalAnimation::Keyframe fresh;
  fresh.mTime = time;
  fresh.mRotation = zjolt::ToJoltRotation(*rotation);
  fresh.mTranslation = zjolt::ToJolt(*translation);
  keyframes.push_back(fresh);
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltSkeletalAnimationGetKeyframeCount(
    const ZJoltSkeletalAnimation *animation, uint32_t joint_index) {
  if (animation == nullptr) return 0;
  const JPH::SkeletalAnimation::AnimatedJointVector &joints =
      zjolt::ToJolt(animation)->GetAnimatedJoints();
  if (joint_index >= static_cast<uint32_t>(joints.size())) return 0;
  return static_cast<uint32_t>(joints[joint_index].mKeyframes.size());
}

ZJoltResult zjoltSkeletalAnimationGetKeyframe(
    const ZJoltSkeletalAnimation *animation, uint32_t joint_index,
    uint32_t keyframe_index, float *out_time, ZJoltQuat *out_rotation,
    ZJoltVec3 *out_translation) {
  ZJOLT_ENTER(out_time, zjolt::OutIsEmptyAs(out_rotation, ZJoltQuat{0, 0, 0, 1}),
              out_translation);
  if (!zjolt::Present(animation, out_time, out_rotation, out_translation)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::SkeletalAnimation::AnimatedJointVector &joints =
      zjolt::ToJolt(animation)->GetAnimatedJoints();
  if (joint_index >= static_cast<uint32_t>(joints.size())) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "joint_index is out of range");
  }
  const JPH::SkeletalAnimation::KeyframeVector &keyframes =
      joints[joint_index].mKeyframes;
  if (keyframe_index >= static_cast<uint32_t>(keyframes.size())) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "keyframe_index is out of range");
  }

  const JPH::SkeletalAnimation::Keyframe &k = keyframes[keyframe_index];
  *out_time = k.mTime;
  *out_rotation = zjolt::ToC(k.mRotation);
  *out_translation = zjolt::ToC(k.mTranslation);
  return ZJOLT_RESULT_OK;
}

float zjoltSkeletalAnimationGetDuration(const ZJoltSkeletalAnimation *animation) {
  if (animation == nullptr) return 0.0f;
  return zjolt::ToJolt(animation)->GetDuration();
}

void zjoltSkeletalAnimationScaleJoints(ZJoltSkeletalAnimation *animation,
                                       float scale) {
  if (animation == nullptr) return;
  zjolt::ToJolt(animation)->ScaleJoints(scale);
}

void zjoltSkeletalAnimationSetIsLooping(ZJoltSkeletalAnimation *animation,
                                        bool is_looping) {
  if (animation == nullptr) return;
  zjolt::ToJolt(animation)->SetIsLooping(is_looping);
}

bool zjoltSkeletalAnimationIsLooping(const ZJoltSkeletalAnimation *animation) {
  if (animation == nullptr) return false;
  return zjolt::ToJolt(animation)->IsLooping();
}

ZJoltResult zjoltSkeletalAnimationSample(const ZJoltSkeletalAnimation *animation,
                                         float time, ZJoltSkeletonPose *pose) {
  ZJOLT_ENTER();
  if (!zjolt::Present(animation, pose)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!(time >= 0.0f)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "time must not be negative");
  }

  JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  const JPH::Skeleton *skeleton = p->GetSkeleton();
  if (skeleton == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the pose has no skeleton assigned");
  }

  const JPH::SkeletalAnimation *a = zjolt::ToJolt(animation);
  // Jolt looks up each animated joint by name and indexes the pose with no
  // bounds check; a name that is not a joint of the pose's skeleton would
  // read out of bounds.
  for (const JPH::SkeletalAnimation::AnimatedJoint &aj : a->GetAnimatedJoints()) {
    if (skeleton->GetJointIndex(aj.mJointName) < 0) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "an animated joint's name is not a joint of the pose's skeleton");
    }
  }

  a->Sample(time, *p);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// SkeletalAnimation — Jolt's own binary stream
//===----------------------------------------------------------------------===//

namespace {
constexpr uint8_t kAnimationStreamMagic[4] = {'Z', 'S', 'A', 'N'};
}  // namespace

ZJoltResult zjoltSkeletalAnimationSaveBinaryState(
    const ZJoltSkeletalAnimation *animation, const ZJoltStream *stream) {
  ZJOLT_ENTER();
  if (!zjolt::Present(animation, stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to save through");
  }

  zjolt::HostStream host(*stream);
  zjolt::WriteStreamHeader(host, kAnimationStreamMagic);
  zjolt::ToJolt(animation)->SaveBinaryState(host);

  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while writing the animation");
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSkeletalAnimationRestoreBinaryState(
    const ZJoltStream *stream, ZJoltSkeletalAnimation **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(stream, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "stream needs read, is_eof and is_failed to restore through");
  }

  zjolt::HostStream host(*stream);
  const ZJoltResult header = zjolt::ReadStreamHeader(
      host, kAnimationStreamMagic,
      "not an animation saved by zjoltSkeletalAnimationSaveBinaryState");
  if (header != ZJOLT_RESULT_OK) return header;

  JPH::SkeletalAnimation::AnimationResult result =
      JPH::SkeletalAnimation::sRestoreFromBinaryState(host);

  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT, result.GetError().c_str());
  }
  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while reading the animation");
  }
  if (!result.IsValid() || host.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the stream ended before the animation did");
  }

  // Same arithmetic as zjoltSceneRestoreStream: the Result's own Ref drops
  // when it goes out of scope below, so the handle needs its own reference
  // added before that happens.
  JPH::SkeletalAnimation *restored = result.Get();
  restored->AddRef();
  *out = zjolt::ToC(restored);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// SkeletalAnimation — JointState
//===----------------------------------------------------------------------===//

void zjoltSkeletalAnimationJointStateFromMatrix(const ZJoltMat44 *matrix,
                                                ZJoltQuat *out_rotation,
                                                ZJoltVec3 *out_translation) {
  if (matrix == nullptr) return;
  JPH::SkeletalAnimation::JointState state;
  state.FromMatrix(zjolt::ToJolt(*matrix));
  zjolt::WriteQuat(out_rotation, state.mRotation);
  zjolt::WriteVec3(out_translation, state.mTranslation);
}

void zjoltSkeletalAnimationJointStateToMatrix(const ZJoltQuat *rotation,
                                              const ZJoltVec3 *translation,
                                              ZJoltMat44 *out) {
  if (rotation == nullptr || translation == nullptr) return;
  JPH::SkeletalAnimation::JointState state;
  state.mRotation = zjolt::ToJoltRotation(*rotation);
  state.mTranslation = zjolt::ToJolt(*translation);
  zjolt::WriteMat44(out, state.ToMatrix());
}

//===----------------------------------------------------------------------===//
// SkeletonMapper
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSkeletonMapperCreate(ZJoltSkeletonMapper **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Not zjolt::Own: the count this hands the caller is the handle's `refs`,
  // which starts at one, and Jolt's own count on the member stays at zero for
  // as long as the handle lives. See The mapper handle, above.
  ZJoltSkeletonMapper *fresh = zjolt::New<ZJoltSkeletonMapper>();
  if (fresh == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate a skeleton mapper");
  }
  *out = fresh;
  return ZJOLT_RESULT_OK;
}

void zjoltSkeletonMapperAddRef(const ZJoltSkeletonMapper *mapper) {
  if (mapper == nullptr) return;
  mapper->refs.fetch_add(1, std::memory_order_relaxed);
}

void zjoltSkeletonMapperRelease(const ZJoltSkeletonMapper *mapper) {
  if (mapper == nullptr) return;

  // The same arithmetic and the same fence as zjoltRagdollRelease, for the
  // same reason: whichever thread reaches zero must see every other holder's
  // writes before it runs the destructor.
  if (mapper->refs.fetch_sub(1, std::memory_order_release) != 1) return;
  std::atomic_thread_fence(std::memory_order_acquire);
  zjolt::Delete(const_cast<ZJoltSkeletonMapper *>(mapper));
}

uint32_t zjoltSkeletonMapperGetRefCount(const ZJoltSkeletonMapper *mapper) {
  if (mapper == nullptr) return 0;
  return mapper->refs.load(std::memory_order_relaxed);
}

ZJoltResult zjoltSkeletonMapperInitialize(
    ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *neutral1,
    const ZJoltSkeletonPose *neutral2,
    ZJoltSkeletonMapperCanMapJointFn can_map_joint, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(mapper, neutral1, neutral2)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  JPH::SkeletonMapper *m = Impl(mapper);
  // Jolt asserts this rather than clearing: initialising twice appends a
  // second set of mappings to the first, and Map would then apply both.
  if (!m->GetMappings().empty() || !m->GetChains().empty() ||
      !m->GetUnmapped().empty()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the mapper has already been initialised");
  }

  const JPH::SkeletonPose *p1 = zjolt::ToJolt(neutral1);
  const JPH::SkeletonPose *p2 = zjolt::ToJolt(neutral2);
  const JPH::Skeleton *s1 = nullptr;
  const JPH::Skeleton *s2 = nullptr;
  ZJoltResult usable = NeutralPoseIsUsable(*p1, &s1);
  if (usable != ZJOLT_RESULT_OK) return usable;
  usable = NeutralPoseIsUsable(*p2, &s2);
  if (usable != ZJOLT_RESULT_OK) return usable;

  if (s1->GetJointCount() > s2->GetJointCount()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "neutral1 must be the pose of the LOW-detail skeleton, so it cannot "
        "have more joints than neutral2");
  }

  // A null callback is the default, not an error, so a caller who does not
  // need one never writes a shim comparing names. Nothing may unwind out of
  // this one: it runs inside Jolt's own loop, compiled without exceptions.
  // The C signature has no way to report a failure, so a predicate that
  // cannot answer just says "these two joints are not the same joint".
  JPH::SkeletonMapper::CanMapJoint can_map =
      &JPH::SkeletonMapper::sDefaultCanMapJoint;
  if (can_map_joint != nullptr) {
    can_map = [can_map_joint, user](const JPH::Skeleton *skeleton1, int index1,
                                    const JPH::Skeleton *skeleton2,
                                    int index2) {
      return can_map_joint(user, zjolt::ToC(skeleton1),
                           static_cast<uint32_t>(index1),
                           zjolt::ToC(skeleton2),
                           static_cast<uint32_t>(index2));
    };
  }

  m->Initialize(s1, p1->GetJointMatrices().data(), s2,
                p2->GetJointMatrices().data(), can_map);
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltSkeletonMapperGetMappingCount(const ZJoltSkeletonMapper *mapper) {
  if (mapper == nullptr) return 0;
  return static_cast<uint32_t>(Impl(mapper)->GetMappings().size());
}

int32_t zjoltSkeletonMapperGetMappedJointIndex(
    const ZJoltSkeletonMapper *mapper, uint32_t joint1_index) {
  if (mapper == nullptr) return -1;
  // A scan for a match, so an index past the end simply matches nothing.
  return Impl(mapper)->GetMappedJointIdx(
      static_cast<int>(joint1_index));
}

uint32_t zjoltSkeletonMapperGetChainCount(const ZJoltSkeletonMapper *mapper) {
  if (mapper == nullptr) return 0;
  return static_cast<uint32_t>(Impl(mapper)->GetChains().size());
}

void zjoltSkeletonMapperGetChainJointCounts(const ZJoltSkeletonMapper *mapper,
                                            uint32_t chain_index,
                                            uint32_t *out_count1,
                                            uint32_t *out_count2) {
  if (out_count1 != nullptr) *out_count1 = 0;
  if (out_count2 != nullptr) *out_count2 = 0;
  if (mapper == nullptr) return;

  const JPH::SkeletonMapper::ChainVector &chains = Impl(mapper)->GetChains();
  if (chain_index >= chains.size()) return;
  if (out_count1 != nullptr) {
    *out_count1 = static_cast<uint32_t>(chains[chain_index].mJointIndices1.size());
  }
  if (out_count2 != nullptr) {
    *out_count2 = static_cast<uint32_t>(chains[chain_index].mJointIndices2.size());
  }
}

int32_t zjoltSkeletonMapperGetChainJointIndex1(const ZJoltSkeletonMapper *mapper,
                                               uint32_t chain_index,
                                               uint32_t index) {
  if (mapper == nullptr) return -1;
  const JPH::SkeletonMapper::ChainVector &chains = Impl(mapper)->GetChains();
  if (chain_index >= chains.size()) return -1;
  const JPH::Array<int> &indices = chains[chain_index].mJointIndices1;
  if (index >= indices.size()) return -1;
  return indices[index];
}

int32_t zjoltSkeletonMapperGetChainJointIndex2(const ZJoltSkeletonMapper *mapper,
                                               uint32_t chain_index,
                                               uint32_t index) {
  if (mapper == nullptr) return -1;
  const JPH::SkeletonMapper::ChainVector &chains = Impl(mapper)->GetChains();
  if (chain_index >= chains.size()) return -1;
  const JPH::Array<int> &indices = chains[chain_index].mJointIndices2;
  if (index >= indices.size()) return -1;
  return indices[index];
}

uint32_t zjoltSkeletonMapperGetUnmappedCount(const ZJoltSkeletonMapper *mapper) {
  if (mapper == nullptr) return 0;
  return static_cast<uint32_t>(Impl(mapper)->GetUnmapped().size());
}

ZJoltResult zjoltSkeletonMapperGetUnmapped(const ZJoltSkeletonMapper *mapper,
                                           uint32_t index,
                                           ZJoltSkeletonMapperUnmapped *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(mapper, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::SkeletonMapper::UnmappedVector &unmapped = Impl(mapper)->GetUnmapped();
  if (index >= unmapped.size()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "index is out of range");
  }
  out->joint_index = unmapped[index].mJointIdx;
  out->parent_joint_index = unmapped[index].mParentJointIdx;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSkeletonMapperLockTranslations(
    ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *neutral2,
    const bool *locked, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(mapper, neutral2, locked)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::SkeletonPose *p2 = zjolt::ToJolt(neutral2);
  const JPH::Skeleton *s2 = nullptr;
  const ZJoltResult usable = NeutralPoseIsUsable(*p2, &s2);
  if (usable != ZJOLT_RESULT_OK) return usable;

  if (count != p2->GetJointCount()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "count must equal the neutral pose's joint count");
  }

  // Rejected before anything is recorded, so a refused call leaves the
  // mapper's locks exactly as they were. A root joint's lock would be
  // resolved by SkeletonMapper::Map against parent joint -1
  // (SkeletonMapper.cpp:209), which reads before the start of the pose.
  for (uint32_t i = 0; i < count; ++i) {
    if (locked[i] && s2->GetJoint(static_cast<int>(i)).mParentJointIndex < 0) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a joint with no parent cannot have its translation locked: its "
          "translation is what positions the whole skeleton");
    }
  }

  Impl(mapper)->LockTranslations(s2, locked,
                                          p2->GetJointMatrices().data());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSkeletonMapperLockAllTranslations(
    ZJoltSkeletonMapper *mapper, const ZJoltSkeletonPose *neutral2) {
  ZJOLT_ENTER();
  if (!zjolt::Present(mapper, neutral2)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SkeletonMapper *m = Impl(mapper);
  // Jolt reads mMappings[0] to find the joint to lock everything below.
  if (m->GetMappings().empty()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "the mapper has no mapped joints, so there is no joint to lock below; "
        "call zjoltSkeletonMapperInitialize first");
  }

  const JPH::SkeletonPose *p2 = zjolt::ToJolt(neutral2);
  const JPH::Skeleton *s2 = nullptr;
  const ZJoltResult usable = NeutralPoseIsUsable(*p2, &s2);
  if (usable != ZJOLT_RESULT_OK) return usable;

  m->LockAllTranslations(s2, p2->GetJointMatrices().data());
  return ZJOLT_RESULT_OK;
}

bool zjoltSkeletonMapperIsJointTranslationLocked(
    const ZJoltSkeletonMapper *mapper, uint32_t joint2_index) {
  if (mapper == nullptr) return false;
  return Impl(mapper)->IsJointTranslationLocked(
      static_cast<int>(joint2_index));
}

ZJoltResult zjoltSkeletonMapperMap(const ZJoltSkeletonMapper *mapper,
                                   const ZJoltSkeletonPose *pose1,
                                   ZJoltSkeletonPose *pose2) {
  ZJOLT_ENTER();
  if (!zjolt::Present(mapper, pose1, pose2)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::SkeletonMapper *m = Impl(mapper);
  const JPH::SkeletonPose *p1 = zjolt::ToJolt(pose1);
  JPH::SkeletonPose *p2 = zjolt::ToJolt(pose2);

  const ZJoltResult fits = PosesFitMapper(*m, *p1, *p2);
  if (fits != ZJOLT_RESULT_OK) return fits;

  // Map wants pose 2's LOCAL joint matrices beside the model-space ones it
  // writes, and a SkeletonPose stores neither: it keeps the local form as
  // rotation/translation pairs and the model form as matrices. Building them
  // here is what keeps a matrix array out of the ABI — the caller would
  // otherwise have to hold one, in a type this header does not name.
  JPH::Array<JPH::Mat44> local(p2->GetJointCount());
  p2->CalculateLocalSpaceJointMatrices(local.data());

  m->Map(p1->GetJointMatrices().data(), local.data(),
         p2->GetJointMatrices().data());

  // Jolt's Map does not do this, and leaving it out is the trap: a pose's
  // joint matrices are relative to its root offset, and zjoltRagdollGetPose
  // puts the ragdoll's WORLD position in the offset rather than in matrix 0
  // (Ragdoll.cpp:599-613, so a float Mat44 never has to hold a double-precision
  // world position). A target pose left at its own offset therefore draws the
  // character at the origin, correctly posed.
  p2->SetRootOffset(p1->GetRootOffset());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSkeletonMapperMapReverse(const ZJoltSkeletonMapper *mapper,
                                          const ZJoltSkeletonPose *pose2,
                                          ZJoltSkeletonPose *pose1) {
  ZJOLT_ENTER();
  if (!zjolt::Present(mapper, pose2, pose1)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::SkeletonMapper *m = Impl(mapper);
  JPH::SkeletonPose *p1 = zjolt::ToJolt(pose1);
  const JPH::SkeletonPose *p2 = zjolt::ToJolt(pose2);

  const ZJoltResult fits = PosesFitMapper(*m, *p1, *p2);
  if (fits != ZJOLT_RESULT_OK) return fits;

  m->MapReverse(p2->GetJointMatrices().data(),
                p1->GetJointMatrices().data());
  // The root offset travels with the pose, as in zjoltSkeletonMapperMap.
  p1->SetRootOffset(p2->GetRootOffset());
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// RagdollSettings
//===----------------------------------------------------------------------===//

ZJoltResult zjoltRagdollSettingsCreate(ZJoltRagdollSettings **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::RagdollSettings *fresh = zjolt::New<JPH::RagdollSettings>();
  if (fresh == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate ragdoll settings");
  }
  *out = zjolt::ToC(zjolt::Own(fresh));
  return ZJOLT_RESULT_OK;
}

void zjoltRagdollSettingsAddRef(const ZJoltRagdollSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->AddRef();
}

void zjoltRagdollSettingsRelease(const ZJoltRagdollSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->Release();
}

uint32_t zjoltRagdollSettingsGetRefCount(const ZJoltRagdollSettings *settings) {
  if (settings == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(settings)->GetRefCount());
}

//===----------------------------------------------------------------------===//
// Jolt's own object stream
//
// Same feature as zjoltSceneSaveObjectStream/RestoreObjectStream (zjolt_scene.cpp), on RagdollSettings instead of PhysicsScene — see that file for what the two entry points do and do not check.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltRagdollSettingsSaveObjectStream(
    const ZJoltRagdollSettings *settings, ZJoltObjectStreamFormat format,
    const ZJoltStream *stream) {
  ZJOLT_ENTER();
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_format = zjolt::RawEnum(format);
  if (!zjolt::Present(settings, stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to save through");
  }
#ifdef JPH_OBJECT_STREAM
  zjolt::ObjectStreamOStream out(*stream);
  const JPH::ObjectStream::EStreamType type =
      raw_format == ZJOLT_OBJECT_STREAM_FORMAT_TEXT
          ? JPH::ObjectStream::EStreamType::Text
          : JPH::ObjectStream::EStreamType::Binary;
  const bool ok =
      JPH::ObjectStreamOut::sWriteObject(out, type, *zjolt::ToJolt(settings));

  if (out.StreamFailed()) {
    return zjolt::SetError(
        ZJOLT_RESULT_IO_ERROR,
        "the stream failed while writing the ragdoll settings");
  }
  if (!ok) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "Jolt's object stream could not write these ragdoll settings");
  }
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltRagdollSettingsRestoreObjectStream(const ZJoltStream *stream,
                                                    ZJoltRagdollSettings **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(stream, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "stream needs read, is_eof and is_failed to restore through");
  }
#ifdef JPH_OBJECT_STREAM
  zjolt::ObjectStreamIStream in(*stream);
  JPH::RagdollSettings *settings = nullptr;
  const bool ok = JPH::ObjectStreamIn::sReadObject(in, settings);

  if (in.StreamFailed()) {
    delete settings;
    return zjolt::SetError(
        ZJOLT_RESULT_IO_ERROR,
        "the stream failed while reading the ragdoll settings");
  }
  if (!ok || settings == nullptr) {
    delete settings;
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "not ragdoll settings Jolt's object stream can read");
  }

  // Same arithmetic as zjoltSceneRestoreObjectStream: a fresh object nobody
  // has referenced yet, not the AddRef-once compensation a dropped Ref<>
  // elsewhere in this file needs.
  *out = zjolt::ToC(zjolt::Own(settings));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltRagdollSettingsBuild(ZJoltRagdollSettings *settings,
                                      const ZJoltSkeleton *skeleton,
                                      const ZJoltRagdollPartDesc *parts,
                                      uint32_t part_count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings, skeleton, parts)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  JPH::RagdollSettings *s = zjolt::ToJolt(settings);
  const JPH::Skeleton *sk = zjolt::ToJolt(skeleton);

  if (part_count != static_cast<uint32_t>(sk->GetJointCount())) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "part_count must equal the skeleton's joint count");
  }

  // Reject everything first, so a rejected call leaves `settings` untouched.
  for (uint32_t i = 0; i < part_count; ++i) {
    const ZJoltResult validated = ValidatePart(*sk, parts[i], i);
    if (validated != ZJOLT_RESULT_OK) return validated;
  }

  // mSkeleton is a plain Ref<Skeleton>, not RefConst — Jolt's own field is
  // not const-correct here even though every setter that reaches it
  // (SkeletonPose::SetSkeleton included) takes `const Skeleton *`.
  s->mSkeleton = const_cast<JPH::Skeleton *>(sk);
  s->mParts.resize(part_count);

  for (uint32_t i = 0; i < part_count; ++i) {
    // Already validated above, so the shape check inside cannot fire; the
    // only way left to fail is allocation. RagdollSettings::Part IS a
    // BodyCreationSettings plus mToParent, filled in just below.
    zjolt::ToJolt(parts[i].body, "a ragdoll part needs a shape",
                  &s->mParts[i]);

    if (parts[i].to_parent != nullptr) {
      JPH::SwingTwistConstraintSettings *cs = BuildConstraint(*parts[i].to_parent);
      if (cs == nullptr) {
        return zjolt::SetError(
            ZJOLT_RESULT_OUT_OF_MEMORY,
            "could not allocate a ragdoll part's constraint");
      }
      s->mParts[i].mToParent = cs;
    }
  }

  return ZJOLT_RESULT_OK;
}

const ZJoltSkeleton *zjoltRagdollSettingsGetSkeleton(
    const ZJoltRagdollSettings *settings) {
  if (settings == nullptr) return nullptr;
  return zjolt::ToC(zjolt::ToJolt(settings)->GetSkeleton());
}

ZJoltResult zjoltRagdollSettingsCalculateConstraintPriorities(
    ZJoltRagdollSettings *settings, uint32_t base_priority) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::RagdollSettings *s = zjolt::ToJolt(settings);
  // Jolt walks mSkeleton to find each part's parent, dereferencing it
  // unconditionally — the same unguarded read Stabilize makes.
  const JPH::Skeleton *skeleton = s->GetSkeleton();
  if (skeleton == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "the settings have not been through zjoltRagdollSettingsBuild");
  }
  if (!skeleton->AreJointsCorrectlyOrdered()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "the settings' skeleton has joints that are not correctly ordered");
  }

  const uint32_t parts = static_cast<uint32_t>(s->mParts.size());
  if (parts == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the settings have no parts to prioritise");
  }
  // Jolt's own assertion, as a check: the walk hands the root a priority of
  // base_priority plus the length of the longest chain below it, which is at
  // most the part count.
  if (base_priority + parts <= base_priority) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "base_priority plus the part count overflows a 32-bit priority");
  }

  s->CalculateConstraintPriorities(base_priority);
  return ZJOLT_RESULT_OK;
}

bool zjoltRagdollSettingsStabilize(ZJoltRagdollSettings *settings) {
  if (settings == nullptr) return false;
  JPH::RagdollSettings *s = zjolt::ToJolt(settings);
  // Jolt's own Stabilize dereferences mSkeleton unconditionally; settings
  // that have not been through zjoltRagdollSettingsBuild have none.
  if (s->GetSkeleton() == nullptr) return true;
  return s->Stabilize();
}

void zjoltRagdollSettingsDisableParentChildCollisions(
    ZJoltRagdollSettings *settings) {
  if (settings == nullptr) return;
  JPH::RagdollSettings *s = zjolt::ToJolt(settings);
  // Same unconditional mSkeleton dereference as Stabilize, above.
  if (s->GetSkeleton() == nullptr) return;
  s->DisableParentChildCollisions(nullptr, 0.0f);
}

void zjoltRagdollSettingsCalculateBodyIndexToConstraintIndex(
    ZJoltRagdollSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->CalculateBodyIndexToConstraintIndex();
}

int32_t zjoltRagdollSettingsGetConstraintIndexForBodyIndex(
    const ZJoltRagdollSettings *settings, uint32_t body_index) {
  if (settings == nullptr) return -1;
  const JPH::Array<int> &table =
      zjolt::ToJolt(settings)->GetBodyIndexToConstraintIndex();
  if (body_index >= table.size()) return -1;
  return table[body_index];
}

ZJoltResult zjoltRagdollSettingsGetBodyIndexToConstraintIndex(
    const ZJoltRagdollSettings *settings, int32_t *out_indices,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(settings, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Array<int> &table =
      zjolt::ToJolt(settings)->GetBodyIndexToConstraintIndex();
  const uint32_t count = static_cast<uint32_t>(table.size());
  *out_count = count;
  if (out_indices == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  for (uint32_t i = 0; i < count; ++i) out_indices[i] = table[i];
  return ZJOLT_RESULT_OK;
}

void zjoltRagdollSettingsCalculateConstraintIndexToBodyIdxPair(
    ZJoltRagdollSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->CalculateConstraintIndexToBodyIdxPair();
}

ZJoltResult zjoltRagdollSettingsGetBodyIndicesForConstraintIndex(
    const ZJoltRagdollSettings *settings, uint32_t constraint_index,
    ZJoltRagdollBodyIndexPair *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(settings, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Array<JPH::RagdollSettings::BodyIdxPair> &table =
      zjolt::ToJolt(settings)->GetConstraintIndexToBodyIdxPair();
  if (constraint_index >= table.size()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "constraint_index is out of range, or the table has not been built "
        "by zjoltRagdollSettingsCalculateConstraintIndexToBodyIdxPair");
  }
  const JPH::RagdollSettings::BodyIdxPair &pair = table[constraint_index];
  out->body_index1 = pair.first;
  out->body_index2 = pair.second;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRagdollSettingsGetConstraintIndexToBodyIdxPair(
    const ZJoltRagdollSettings *settings, ZJoltRagdollBodyIndexPair *out_pairs,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(settings, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Array<JPH::RagdollSettings::BodyIdxPair> &table =
      zjolt::ToJolt(settings)->GetConstraintIndexToBodyIdxPair();
  const uint32_t count = static_cast<uint32_t>(table.size());
  *out_count = count;
  if (out_pairs == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  for (uint32_t i = 0; i < count; ++i) {
    out_pairs[i] = ZJoltRagdollBodyIndexPair{table[i].first, table[i].second};
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRagdollSettingsCreateRagdoll(
    const ZJoltRagdollSettings *settings, ZJoltPhysicsSystem *system,
    uint32_t collision_group, uint64_t user_data, ZJoltRagdoll **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(settings, system, out)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::RagdollSettings *s = zjolt::ToJolt(settings);
  JPH::Ragdoll *created = s->CreateRagdoll(
      static_cast<JPH::CollisionGroup::GroupID>(collision_group), user_data,
      &system->system);
  if (created == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "the physics system has run out of bodies");
  }

  // Held from here, so failing to allocate the handle destroys the bodies
  // CreateRagdoll just made rather than stranding them in the system. None
  // of them are added yet, which is the one state ~Ragdoll can clean up on
  // its own.
  JPH::Ref<JPH::Ragdoll> fresh = created;

  ZJoltRagdoll *handle = zjolt::New<ZJoltRagdoll>();
  if (handle == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate a ragdoll handle");
  }

  // Not zjolt::Own: the handle's JPH::Ref is what owns the ragdoll now, and
  // the reference this call hands the caller is `refs`, which starts at one.
  handle->impl = fresh;
  handle->owner = system;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Ragdoll
//===----------------------------------------------------------------------===//

void zjoltRagdollAddRef(const ZJoltRagdoll *ragdoll) {
  if (ragdoll == nullptr) return;
  ragdoll->refs.fetch_add(1, std::memory_order_relaxed);
}

void zjoltRagdollRelease(const ZJoltRagdoll *ragdoll) {
  if (ragdoll == nullptr) return;

  // Release ordering on the way down and an acquire fence once the count
  // reaches zero, so whichever thread runs the teardown sees every other
  // holder's writes. Exactly the arithmetic JPH::RefTarget::Release does
  // (Core/Reference.h:60), for exactly the same reason.
  if (ragdoll->refs.fetch_sub(1, std::memory_order_release) != 1) return;
  std::atomic_thread_fence(std::memory_order_acquire);

  ZJoltRagdoll *handle = const_cast<ZJoltRagdoll *>(ragdoll);
  JPH::Ragdoll *impl = handle->impl.GetPtr();

  // Dropping `impl` runs ~Ragdoll, destroying the bodies in place, so the broad
  // phase must clear them first. Checked, not assumed: RemoveFromPhysicsSystem
  // is unsafe on a ragdoll never added (it removes constraints before bodies,
  // and ConstraintManager::Remove asserts each was added).
  // AddToPhysicsSystem/RemoveFromPhysicsSystem move every part as one batch, so
  // the first body answers for all; empty parts need no check.
  const JPH::Array<JPH::BodyID> &ids = impl->GetBodyIDs();
  if (!ids.empty() &&
      handle->owner->system.GetBodyInterface().IsAdded(ids.front())) {
    // Locking: a release comes from outside the simulation, and there is no
    // narrower promise about where the bodies are.
    impl->RemoveFromPhysicsSystem(true);
  }

  handle->impl = nullptr;
  zjolt::Delete(handle);
  zjolt::HandleDestroyed();
}

uint32_t zjoltRagdollGetRefCount(const ZJoltRagdoll *ragdoll) {
  if (ragdoll == nullptr) return 0;
  return ragdoll->refs.load(std::memory_order_relaxed);
}

void zjoltRagdollAddToPhysicsSystem(ZJoltRagdoll *ragdoll,
                                    ZJoltActivation activation,
                                    bool lock_bodies) {
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_activation = zjolt::RawEnum(activation);
  JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return;
  impl->AddToPhysicsSystem(ToJoltActivation(raw_activation), lock_bodies);
}

void zjoltRagdollRemoveFromPhysicsSystem(ZJoltRagdoll *ragdoll,
                                         bool lock_bodies) {
  JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return;
  impl->RemoveFromPhysicsSystem(lock_bodies);
}

ZJoltResult zjoltRagdollGetBodyIds(const ZJoltRagdoll *ragdoll,
                                   ZJoltBodyId *out_ids, uint32_t capacity,
                                   uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(ragdoll, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Ragdoll *impl = Impl(ragdoll);
  const JPH::Array<JPH::BodyID> &ids = impl->GetBodyIDs();
  const uint32_t count = static_cast<uint32_t>(ids.size());
  *out_count = count;
  if (out_ids == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  for (uint32_t i = 0; i < count; ++i) out_ids[i] = zjolt::ToC(ids[i]);
  return ZJOLT_RESULT_OK;
}

const ZJoltRagdollSettings *zjoltRagdollGetRagdollSettings(
    const ZJoltRagdoll *ragdoll) {
  const JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return nullptr;
  return zjolt::ToC(impl->GetRagdollSettings());
}

uint32_t zjoltRagdollGetConstraintCount(const ZJoltRagdoll *ragdoll) {
  const JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return 0;
  return static_cast<uint32_t>(impl->GetConstraintCount());
}

ZJoltConstraint *zjoltRagdollGetConstraint(ZJoltRagdoll *ragdoll,
                                           uint32_t index) {
  JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return nullptr;
  // Jolt indexes mConstraints without a bounds check.
  if (index >= static_cast<uint32_t>(impl->GetConstraintCount())) {
    return nullptr;
  }
  return zjolt::ToC(impl->GetConstraint(static_cast<int>(index)));
}

ZJoltResult zjoltRagdollGetRootTransform(const ZJoltRagdoll *ragdoll,
                                         ZJoltRVec3 *out_position,
                                         ZJoltQuat *out_rotation,
                                         bool lock_bodies) {
  ZJOLT_ENTER(out_position,
              zjolt::OutIsEmptyAs(out_rotation, ZJoltQuat{0, 0, 0, 1}));
  if (!zjolt::Present(ragdoll, out_position, out_rotation)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::Ragdoll *impl = Impl(ragdoll);
  // Jolt reads mBodyIDs[0] with no bounds check. A ragdoll spawned from
  // settings built on a skeleton with no joints has none.
  if (impl->GetBodyIDs().empty()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the ragdoll has no parts, so it has no root");
  }

  JPH::RVec3 position = JPH::RVec3::sZero();
  JPH::Quat rotation = JPH::Quat::sIdentity();
  impl->GetRootTransform(position, rotation, lock_bodies);
  *out_position = zjolt::ToCR(position);
  *out_rotation = zjolt::ToC(rotation);
  return ZJOLT_RESULT_OK;
}

void zjoltRagdollSetGroupId(ZJoltRagdoll *ragdoll, uint32_t group_id,
                            bool lock_bodies) {
  JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return;
  // Jolt takes a multi-body write lock over mBodyIDs.data(), which is null
  // for a ragdoll with no parts.
  if (impl->GetBodyIDs().empty()) return;
  impl->SetGroupID(static_cast<JPH::CollisionGroup::GroupID>(group_id),
                   lock_bodies);
}

void zjoltRagdollActivate(ZJoltRagdoll *ragdoll, bool lock_bodies) {
  JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return;
  impl->Activate(lock_bodies);
}

bool zjoltRagdollIsActive(const ZJoltRagdoll *ragdoll, bool lock_bodies) {
  const JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return false;
  return impl->IsActive(lock_bodies);
}

void zjoltRagdollResetWarmStart(ZJoltRagdoll *ragdoll) {
  JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return;
  impl->ResetWarmStart();
}

ZJoltResult zjoltRagdollSetPose(ZJoltRagdoll *ragdoll,
                                const ZJoltSkeletonPose *pose,
                                bool lock_bodies) {
  ZJOLT_ENTER();
  if (!zjolt::Present(ragdoll, pose)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Ragdoll *r = Impl(ragdoll);
  const JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  if (!SkeletonMatches(*r, *p)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the pose's skeleton is not the ragdoll's");
  }
  r->SetPose(*p, lock_bodies);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRagdollGetPose(ZJoltRagdoll *ragdoll, ZJoltSkeletonPose *pose,
                                bool lock_bodies) {
  ZJOLT_ENTER();
  if (!zjolt::Present(ragdoll, pose)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Ragdoll *r = Impl(ragdoll);
  JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  if (!SkeletonMatches(*r, *p)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the pose's skeleton is not the ragdoll's");
  }
  r->GetPose(*p, lock_bodies);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRagdollDriveToPoseUsingKinematics(
    ZJoltRagdoll *ragdoll, const ZJoltSkeletonPose *pose, float delta_time,
    bool lock_bodies) {
  ZJOLT_ENTER();
  if (!zjolt::Present(ragdoll, pose)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Ragdoll *r = Impl(ragdoll);
  const JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  if (!SkeletonMatches(*r, *p)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the pose's skeleton is not the ragdoll's");
  }
  r->DriveToPoseUsingKinematics(*p, delta_time, lock_bodies);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRagdollDriveToPoseUsingMotors(ZJoltRagdoll *ragdoll,
                                               const ZJoltSkeletonPose *pose) {
  ZJOLT_ENTER();
  if (!zjolt::Present(ragdoll, pose)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Ragdoll *r = Impl(ragdoll);
  const JPH::SkeletonPose *p = zjolt::ToJolt(pose);
  if (!SkeletonMatches(*r, *p)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the pose's skeleton is not the ragdoll's");
  }
  // Every constraint a ragdoll built through this ABI owns is the
  // SwingTwistConstraint BuildConstraint makes, so Jolt's own
  // "Constraint type not implemented!" assert (for anything besides that and
  // a hinge, which this ABI never constructs) can never fire here.
  r->DriveToPoseUsingMotors(*p);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRagdollDriveToPoseUsingMotorsWithVelocity(
    ZJoltRagdoll *ragdoll, const ZJoltSkeletonPose *prev_pose,
    const ZJoltSkeletonPose *pose, float delta_time) {
  ZJOLT_ENTER();
  if (!zjolt::Present(ragdoll, prev_pose, pose)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  if (!(delta_time > 0.0f)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "delta_time must be positive");
  }

  JPH::Ragdoll *r = Impl(ragdoll);
  const JPH::SkeletonPose *prev = zjolt::ToJolt(prev_pose);
  const JPH::SkeletonPose *cur = zjolt::ToJolt(pose);
  if (!SkeletonMatches(*r, *prev) || !SkeletonMatches(*r, *cur)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a pose's skeleton is not the ragdoll's");
  }
  r->DriveToPoseUsingMotors(*prev, *cur, delta_time);
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
