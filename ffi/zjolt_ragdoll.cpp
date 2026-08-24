//===----------------------------------------------------------------------===//
// zjolt — ragdolls.
//
// A few conversions are duplicated from zjolt_body.cpp (ToJoltMotionType,
// the ZJoltBodyDesc -> JPH::BodyCreationSettings fields) and zjolt_internal.h
// (ToJolt/ToC for the tag handles this file introduces). Both would
// normally live in one shared place, but this subsystem is scoped to add
// files and append registrations only — not to edit the existing
// translation units they would otherwise join — so each stays local to this
// file instead.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>

#include <atomic>

//===----------------------------------------------------------------------===//
// Opaque handle mapping
//
// Three of the four are reinterpret_cast tags directly onto the Jolt type,
// exactly like ZJoltShape/ZJoltPhysicsMaterial in zjolt_internal.h — never
// completed, never dereferenced except after converting back. Skeleton and
// RagdollSettings are reference counted (JPH::RefTarget); SkeletonPose is a
// plain value type with no reference count of its own, allocated and freed
// like a job system.
//
// ZJoltRagdoll is the fourth, and it is not a tag — see The handle, below.
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

inline const JPH::RagdollSettings *ToJolt(const ZJoltRagdollSettings *s) {
  return reinterpret_cast<const JPH::RagdollSettings *>(s);
}
inline JPH::RagdollSettings *ToJolt(ZJoltRagdollSettings *s) {
  return reinterpret_cast<JPH::RagdollSettings *>(s);
}
inline ZJoltRagdollSettings *ToC(JPH::RagdollSettings *s) {
  return reinterpret_cast<ZJoltRagdollSettings *>(s);
}

}  // namespace zjolt

//===----------------------------------------------------------------------===//
// The handle
//
// A ragdoll cannot be a tag on JPH::Ragdoll the way the three types above
// are, because releasing the last reference to one has to be able to take
// its bodies out of the broad phase first. `~Ragdoll` calls
// `BodyInterface::DestroyBodies` where the bodies stand, and
// `BodyManager::RemoveBodyInternal` asserts `!IsInBroadPhase()` before it
// frees one (BodyManager.cpp:354) — so releasing a still-added ragdoll aborts
// a build with assertions and corrupts the broad phase in one without.
//
// Removing it needs the PhysicsSystem, and `Ragdoll::mSystem` is private
// with no getter (Ragdoll.h:248). So the handle keeps it: `owner`, the same
// shape ZJoltCharacter (zjolt_internal.h) and ZJoltVehicleConstraint
// (zjolt_vehicle.cpp) already have. Defined here rather than in the shared
// header for the same reason the vehicle one is — nothing outside this file
// ever sees a ZJoltRagdoll *.
//
// `refs` is the third member, and it is not a duplicate of the ragdoll's own
// JPH::RefTarget count. BINDING.md's rule for the Release verb is that there
// may be other holders, so two of them releasing at once must both be
// correct and exactly one must run the teardown below. RefTarget cannot say
// which: its Release destroys the object on the way to telling you, and
// reading GetRefCount() before releasing leaves a gap between the read and
// the drop that another thread fits into. Counting the handle's own
// references is what makes that answer atomic. `impl` therefore holds Jolt's
// only reference, pinned at one for as long as the handle lives, and
// zjoltRagdollGetRefCount reports `refs` — which is the number the caller was
// moving anyway.
//===----------------------------------------------------------------------===//

struct ZJoltRagdoll {
  JPH::Ref<JPH::Ragdoll> impl;
  ZJoltPhysicsSystem *owner;

  /// Mutable for the same reason JPH::RefTarget's own count is: AddRef and
  /// Release take a `const ZJoltRagdoll *`, because holding a reference to a
  /// ragdoll is not modifying it.
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

//===----------------------------------------------------------------------===//
// Small conversions duplicated from zjolt_body.cpp — see the file comment.
//===----------------------------------------------------------------------===//

JPH::EActivation ToJoltActivation(ZJoltActivation activation) {
  return activation == ZJOLT_ACTIVATION_DONT_ACTIVATE
             ? JPH::EActivation::DontActivate
             : JPH::EActivation::Activate;
}

JPH::EMotionType ToJoltMotionType(ZJoltMotionType type) {
  switch (type) {
    case ZJOLT_MOTION_TYPE_STATIC:
      return JPH::EMotionType::Static;
    case ZJOLT_MOTION_TYPE_KINEMATIC:
      return JPH::EMotionType::Kinematic;
    case ZJOLT_MOTION_TYPE_DYNAMIC:
      break;
  }
  return JPH::EMotionType::Dynamic;
}

/// Fills the BodyCreationSettings half of a ragdoll part from a flat
/// ZJoltBodyDesc. Mirrors zjolt_body.cpp's BuildCreationSettings field for
/// field; RagdollSettings::Part is a BodyCreationSettings plus mToParent,
/// which the caller fills in separately.
///
/// Called only after ValidatePart has already accepted `desc`, so the checks
/// it repeats (shape present, mass positive when required) cannot actually
/// fail here — this always returns ZJOLT_RESULT_OK. It still reports them
/// rather than asserting, so it stays correct if ever called on its own.
ZJoltResult BuildPartBody(const ZJoltBodyDesc &desc,
                          JPH::BodyCreationSettings *out) {
  if (desc.shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a ragdoll part needs a shape");
  }

  out->mPosition = zjolt::ToJoltR(desc.position);
  out->mRotation = zjolt::ToJoltRotation(desc.rotation);
  out->mLinearVelocity = zjolt::ToJolt(desc.linear_velocity);
  out->mAngularVelocity = zjolt::ToJolt(desc.angular_velocity);
  out->SetShape(zjolt::ToJolt(desc.shape));
  // Overwritten for every part by zjoltRagdollSettingsDisableParentChildCollisions
  // if that is called, and by zjoltRagdollSettingsCreateRagdoll's group id in
  // any case — a per-part group here is what those two build on top of.
  out->mCollisionGroup = zjolt::ToJolt(&desc.collision_group);
  out->mUserData = desc.user_data;
  out->mObjectLayer = static_cast<JPH::ObjectLayer>(desc.object_layer);
  out->mMotionType = ToJoltMotionType(desc.motion_type);
  out->mMotionQuality = desc.motion_quality == ZJOLT_MOTION_QUALITY_LINEAR_CAST
                            ? JPH::EMotionQuality::LinearCast
                            : JPH::EMotionQuality::Discrete;
  out->mAllowedDOFs = static_cast<JPH::EAllowedDOFs>(desc.allowed_dofs);
  out->mAllowDynamicOrKinematic = desc.allow_dynamic_or_kinematic;
  out->mIsSensor = desc.is_sensor;
  out->mAllowSleeping = desc.allow_sleeping;
  out->mEnhancedInternalEdgeRemoval = desc.enhanced_internal_edge_removal;
  out->mFriction = desc.friction;
  out->mRestitution = desc.restitution;
  out->mLinearDamping = desc.linear_damping;
  out->mAngularDamping = desc.angular_damping;
  out->mMaxLinearVelocity = desc.max_linear_velocity;
  out->mMaxAngularVelocity = desc.max_angular_velocity;
  out->mGravityFactor = desc.gravity_factor;

  if (desc.override_mass_properties ==
      ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_INERTIA) {
    if (!(desc.mass > 0.0f)) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "override_mass_properties asks for an explicit mass, so mass must "
          "be positive");
    }
    out->mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    out->mMassPropertiesOverride.mMass = desc.mass;
  } else {
    out->mOverrideMassProperties =
        JPH::EOverrideMassProperties::CalculateMassAndInertia;
  }
  return ZJOLT_RESULT_OK;
}

JPH::ESwingType ToJoltSwingType(ZJoltSwingType type) {
  return type == ZJOLT_SWING_TYPE_PYRAMID ? JPH::ESwingType::Pyramid
                                          : JPH::ESwingType::Cone;
}

/// Builds a swing-twist constraint settings object on the heap (it must
/// outlive this call, unlike the parts' BodyCreationSettings above, because
/// RagdollSettings::Part::mToParent keeps a reference to it) from a flat
/// descriptor. Never crosses the ABI: it is assigned straight into
/// mToParent, which takes its own reference on assignment.
///
/// Returns NULL only on allocation failure.
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
  settings->mSwingType = ToJoltSwingType(desc.swing_type);
  settings->mNormalHalfConeAngle = desc.normal_half_cone_angle;
  settings->mPlaneHalfConeAngle = desc.plane_half_cone_angle;
  settings->mTwistMinAngle = desc.twist_min_angle;
  settings->mTwistMaxAngle = desc.twist_max_angle;
  settings->mMaxFrictionTorque = desc.max_friction_torque;
  return settings;
}

/// Everything zjoltRagdollSettingsBuild must reject BEFORE it starts
/// mutating `settings`, so a rejected call leaves it exactly as it was.
///
/// The to_parent check is the one CreateRagdoll itself does not make: it
/// indexes `bodies[mSkeleton->GetJoint(joint_idx).mParentJointIndex]` for
/// every part with a non-null mToParent, so a root part (parent index -1)
/// carrying one would read `bodies[-1]` — out of bounds, found by reading
/// Ragdoll.cpp's CreateRagdoll rather than by any JPH_ASSERT.
ZJoltResult ValidatePart(const JPH::Skeleton &skeleton,
                         const ZJoltRagdollPartDesc &part,
                         uint32_t joint_index) {
  if (part.body.shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a ragdoll part needs a shape");
  }
  if (part.body.override_mass_properties ==
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
    // Already validated above; the only way left to fail is allocation.
    BuildPartBody(parts[i].body, &s->mParts[i]);

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

  // Dropping `impl` below runs ~Ragdoll, which destroys the bodies where
  // they stand — so anything the broad phase still holds has to come out
  // first. Asked rather than assumed, because RemoveFromPhysicsSystem is not
  // safe on a ragdoll that was never added: it removes the constraints
  // before the bodies, and ConstraintManager::Remove asserts that each one
  // was added (ConstraintManager.cpp:47).
  //
  // AddToPhysicsSystem and RemoveFromPhysicsSystem move every part as one
  // batch and this ABI offers no way to split them, so the first body
  // answers for all of them. A ragdoll with no parts — settings never built,
  // or built from a skeleton with no joints — has nothing to ask and nothing
  // to remove.
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
  JPH::Ragdoll *impl = Impl(ragdoll);
  if (impl == nullptr) return;
  impl->AddToPhysicsSystem(ToJoltActivation(activation), lock_bodies);
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
