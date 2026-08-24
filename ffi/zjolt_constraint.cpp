//===----------------------------------------------------------------------===//
// zjolt — constraints.
//
// Almost all of this file is precondition checking, and that is the point.
// Jolt's constraints carry more caller-trippable assertions than anything else
// in the library — a hinge whose limits have the wrong sign, a motor switched
// on before its settings are valid, a gear handed a slider where it wanted a
// hinge — and every one of them is an abort in an asserts-on build, at a
// stack that names the solver rather than the call that was wrong. The rule
// here is the one in BINDING.md: never forward a call that would abort.
//
// The rest is the shape of the boundary. Descriptors are flat plain data; the
// Jolt `*Settings` object is built on the stack from one, used once, and
// dropped. Nothing of Jolt's crosses.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>

#include <cfloat>

//===----------------------------------------------------------------------===//
// Handle mapping
//
// Two more of the incomplete tags described at the top of zjolt_internal.h.
// Both name objects Jolt constructs and reference counts itself, so neither
// can be a struct of ours without a second identity, and both are converted
// through exactly as ZJoltShape is: never completed, never dereferenced as the
// tag, every use going back through the Jolt type first.
//
// They live here rather than in zjolt_internal.h because they are the only two
// handles this subsystem introduces and nothing outside it converts one. The
// rule the internal header states — one conversion, in one place — is about
// there being a single definition, not about which file it sits in.
//===----------------------------------------------------------------------===//

namespace zjolt {

inline JPH::Constraint *ToJolt(ZJoltConstraint *constraint) {
  return reinterpret_cast<JPH::Constraint *>(constraint);
}
inline const JPH::Constraint *ToJolt(const ZJoltConstraint *constraint) {
  return reinterpret_cast<const JPH::Constraint *>(constraint);
}
inline ZJoltConstraint *ToC(JPH::Constraint *constraint) {
  return reinterpret_cast<ZJoltConstraint *>(constraint);
}

inline const JPH::PathConstraintPath *ToJolt(const ZJoltPathConstraintPath *path) {
  return reinterpret_cast<const JPH::PathConstraintPath *>(path);
}
inline ZJoltPathConstraintPath *ToC(JPH::PathConstraintPath *path) {
  return reinterpret_cast<ZJoltPathConstraintPath *>(path);
}

}  // namespace zjolt

namespace {

//===----------------------------------------------------------------------===//
// Scalar and vector validation
//
// A NaN reaching Jolt does not abort; it spreads. Every float a descriptor
// carries is checked for finiteness before it is used, because the failure
// that follows one otherwise surfaces as a body that has vanished, several
// frames later, in a different subsystem.
//===----------------------------------------------------------------------===//

bool IsFinite(float v) { return std::isfinite(v); }

bool IsFinite(const ZJoltVec3 &v) {
  return IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
}

bool IsFinite(const ZJoltRVec3 &v) {
  return std::isfinite(static_cast<double>(v.x)) &&
         std::isfinite(static_cast<double>(v.y)) &&
         std::isfinite(static_cast<double>(v.z));
}

ZJoltResult CheckPoint(const ZJoltRVec3 &v, const char *what) {
  if (!IsFinite(v)) return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
  return ZJOLT_RESULT_OK;
}

ZJoltResult CheckFloat(float v, const char *what) {
  if (!IsFinite(v)) return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
  return ZJOLT_RESULT_OK;
}

/// An axis a constraint frame is built from.
///
/// Jolt does not assert on a degenerate one — it feeds it to
/// `Mat44::GetQuaternion`, which produces a quaternion that is not a rotation,
/// and the joint then holds a frame that is not a frame. That is worse than an
/// abort, not better, so the check is here: finite, and long enough to
/// normalise.
ZJoltResult CheckAxis(const ZJoltVec3 &v, const char *what) {
  if (!IsFinite(v)) return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
  const float length_sq = v.x * v.x + v.y * v.y + v.z * v.z;
  if (length_sq < 1.0e-12f)
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
  return ZJOLT_RESULT_OK;
}

/// Two axes that have to span a plane.
///
/// Tolerant rather than exact: a caller's axes have usually been through a
/// transform, and demanding an exact right angle would refuse frames that are
/// perfectly usable. What is refused is a pair so close to parallel that their
/// cross product carries no direction — which is the case Jolt cannot recover
/// from.
ZJoltResult CheckPerpendicular(const ZJoltVec3 &a, const ZJoltVec3 &b,
                               const char *what) {
  const JPH::Vec3 ja = zjolt::ToJolt(a).NormalizedOr(JPH::Vec3::sZero());
  const JPH::Vec3 jb = zjolt::ToJolt(b).NormalizedOr(JPH::Vec3::sZero());
  if (std::abs(ja.Dot(jb)) > 0.999f)
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Enumeration conversion
//
// A C caller can pass any integer for an enum parameter, so every one of these
// refuses a value it does not know rather than falling through to a default.
// A silently substituted default here is a joint that does not do what the
// call said, which is exactly the class of bug an ABI should not invent.
//===----------------------------------------------------------------------===//

ZJoltResult ToJoltSpace(ZJoltConstraintSpace space, JPH::EConstraintSpace *out) {
  switch (space) {
    case ZJOLT_CONSTRAINT_SPACE_LOCAL_TO_BODY_COM:
      *out = JPH::EConstraintSpace::LocalToBodyCOM;
      return ZJOLT_RESULT_OK;
    case ZJOLT_CONSTRAINT_SPACE_WORLD:
      *out = JPH::EConstraintSpace::WorldSpace;
      return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                         "space is not a ZJoltConstraintSpace");
}

ZJoltResult ToJoltMotorState(ZJoltMotorState state, JPH::EMotorState *out) {
  switch (state) {
    case ZJOLT_MOTOR_STATE_OFF:
      *out = JPH::EMotorState::Off;
      return ZJOLT_RESULT_OK;
    case ZJOLT_MOTOR_STATE_VELOCITY:
      *out = JPH::EMotorState::Velocity;
      return ZJOLT_RESULT_OK;
    case ZJOLT_MOTOR_STATE_POSITION:
      *out = JPH::EMotorState::Position;
      return ZJOLT_RESULT_OK;
    case ZJOLT_MOTOR_STATE_POSITION_AND_VELOCITY:
      *out = JPH::EMotorState::PositionAndVelocity;
      return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                         "state is not a ZJoltMotorState");
}

ZJoltMotorState ToCMotorState(JPH::EMotorState state) {
  switch (state) {
    case JPH::EMotorState::Off:
      return ZJOLT_MOTOR_STATE_OFF;
    case JPH::EMotorState::Velocity:
      return ZJOLT_MOTOR_STATE_VELOCITY;
    case JPH::EMotorState::Position:
      return ZJOLT_MOTOR_STATE_POSITION;
    case JPH::EMotorState::PositionAndVelocity:
      break;
  }
  return ZJOLT_MOTOR_STATE_POSITION_AND_VELOCITY;
}

ZJoltResult ToJoltSwingType(ZJoltSwingType type, JPH::ESwingType *out) {
  switch (type) {
    case ZJOLT_SWING_TYPE_CONE:
      *out = JPH::ESwingType::Cone;
      return ZJOLT_RESULT_OK;
    case ZJOLT_SWING_TYPE_PYRAMID:
      *out = JPH::ESwingType::Pyramid;
      return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                         "swing_type is not a ZJoltSwingType");
}

ZJoltResult ToJoltPathRotation(ZJoltPathRotationConstraintType type,
                               JPH::EPathRotationConstraintType *out) {
  switch (type) {
    case ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_FREE:
      *out = JPH::EPathRotationConstraintType::Free;
      return ZJOLT_RESULT_OK;
    case ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_AROUND_TANGENT:
      *out = JPH::EPathRotationConstraintType::ConstrainAroundTangent;
      return ZJOLT_RESULT_OK;
    case ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_AROUND_NORMAL:
      *out = JPH::EPathRotationConstraintType::ConstrainAroundNormal;
      return ZJOLT_RESULT_OK;
    case ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_AROUND_BINORMAL:
      *out = JPH::EPathRotationConstraintType::ConstrainAroundBinormal;
      return ZJOLT_RESULT_OK;
    case ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_TO_PATH:
      *out = JPH::EPathRotationConstraintType::ConstrainToPath;
      return ZJOLT_RESULT_OK;
    case ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_FULLY_CONSTRAINED:
      *out = JPH::EPathRotationConstraintType::FullyConstrained;
      return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "rotation_constraint_type is not a ZJoltPathRotationConstraintType");
}

ZJoltConstraintSubType ToCSubType(JPH::EConstraintSubType sub_type) {
  switch (sub_type) {
    case JPH::EConstraintSubType::Fixed:
      return ZJOLT_CONSTRAINT_SUB_TYPE_FIXED;
    case JPH::EConstraintSubType::Point:
      return ZJOLT_CONSTRAINT_SUB_TYPE_POINT;
    case JPH::EConstraintSubType::Hinge:
      return ZJOLT_CONSTRAINT_SUB_TYPE_HINGE;
    case JPH::EConstraintSubType::Slider:
      return ZJOLT_CONSTRAINT_SUB_TYPE_SLIDER;
    case JPH::EConstraintSubType::Distance:
      return ZJOLT_CONSTRAINT_SUB_TYPE_DISTANCE;
    case JPH::EConstraintSubType::Cone:
      return ZJOLT_CONSTRAINT_SUB_TYPE_CONE;
    case JPH::EConstraintSubType::SwingTwist:
      return ZJOLT_CONSTRAINT_SUB_TYPE_SWING_TWIST;
    case JPH::EConstraintSubType::SixDOF:
      return ZJOLT_CONSTRAINT_SUB_TYPE_SIX_DOF;
    case JPH::EConstraintSubType::Path:
      return ZJOLT_CONSTRAINT_SUB_TYPE_PATH;
    case JPH::EConstraintSubType::Gear:
      return ZJOLT_CONSTRAINT_SUB_TYPE_GEAR;
    case JPH::EConstraintSubType::RackAndPinion:
      return ZJOLT_CONSTRAINT_SUB_TYPE_RACK_AND_PINION;
    case JPH::EConstraintSubType::Pulley:
      return ZJOLT_CONSTRAINT_SUB_TYPE_PULLEY;
    default:
      break;
  }
  // Vehicle, and the four User* slots. Named rather than guessed at.
  return ZJOLT_CONSTRAINT_SUB_TYPE_OTHER;
}

//===----------------------------------------------------------------------===//
// Springs and motors
//===----------------------------------------------------------------------===//

/// A spring the constraint parts will accept.
///
/// `MotorSettings::IsValid` — the thing `SetMotorState` asserts on — requires
/// a non-negative frequency and damping, so a spring is validated wherever it
/// crosses rather than only where it is attached to a motor. The same struct
/// also drives limit springs, which have no IsValid of their own but produce
/// the same nonsense from a negative stiffness.
ZJoltResult ToJoltSpring(const ZJoltSpringSettings &spring,
                         JPH::SpringSettings *out) {
  if (!IsFinite(spring.frequency_or_stiffness) || !IsFinite(spring.damping)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "spring frequency/stiffness and damping must be finite");
  }
  if (spring.frequency_or_stiffness < 0.0f || spring.damping < 0.0f) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "spring frequency/stiffness and damping must not be negative; 0 "
        "frequency is how a hard limit is spelled");
  }
  switch (spring.mode) {
    case ZJOLT_SPRING_MODE_FREQUENCY_AND_DAMPING:
      out->mMode = JPH::ESpringMode::FrequencyAndDamping;
      break;
    case ZJOLT_SPRING_MODE_STIFFNESS_AND_DAMPING:
      out->mMode = JPH::ESpringMode::StiffnessAndDamping;
      break;
    case ZJOLT_SPRING_MODE_MASS_NORMALIZED_STIFFNESS_AND_DAMPING:
      out->mMode = JPH::ESpringMode::MassNormalizedStiffnessAndDamping;
      break;
    default:
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "spring mode is not a ZJoltSpringMode");
  }
  out->mFrequency = spring.frequency_or_stiffness;
  out->mDamping = spring.damping;
  return ZJOLT_RESULT_OK;
}

ZJoltSpringSettings ToCSpring(const JPH::SpringSettings &spring) {
  ZJoltSpringSettings out{};
  switch (spring.mMode) {
    case JPH::ESpringMode::StiffnessAndDamping:
      out.mode = ZJOLT_SPRING_MODE_STIFFNESS_AND_DAMPING;
      break;
    case JPH::ESpringMode::MassNormalizedStiffnessAndDamping:
      out.mode = ZJOLT_SPRING_MODE_MASS_NORMALIZED_STIFFNESS_AND_DAMPING;
      break;
    case JPH::ESpringMode::FrequencyAndDamping:
    default:
      out.mode = ZJOLT_SPRING_MODE_FREQUENCY_AND_DAMPING;
      break;
  }
  out.frequency_or_stiffness = spring.mFrequency;
  out.damping = spring.mDamping;
  return out;
}

/// A motor that `SetMotorState` will accept.
///
/// This is the precondition that costs the most to get wrong. Every joint with
/// a motor asserts `mMotorSettings.IsValid()` before switching one on, so a
/// motor built from a descriptor with crossed limits does not fail at the
/// descriptor — it fails, in an asserts-on build, at whatever later frame
/// first turns the motor on. Validating here means the assertion is
/// unreachable rather than merely unlikely.
///
/// The fields are assigned rather than passed to a MotorSettings constructor
/// on purpose: every one of those constructors asserts IsValid() itself, so
/// building an invalid one to inspect it would abort before it could be
/// refused.
ZJoltResult ToJoltMotor(const ZJoltMotorSettings &motor,
                        JPH::MotorSettings *out) {
  const ZJoltResult spring = ToJoltSpring(motor.spring, &out->mSpringSettings);
  if (spring != ZJOLT_RESULT_OK) return spring;

  if (std::isnan(motor.min_force_limit) || std::isnan(motor.max_force_limit) ||
      std::isnan(motor.min_torque_limit) || std::isnan(motor.max_torque_limit)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "motor force and torque limits must not be NaN");
  }
  if (motor.min_force_limit > motor.max_force_limit) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "motor min_force_limit exceeds max_force_limit");
  }
  if (motor.min_torque_limit > motor.max_torque_limit) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "motor min_torque_limit exceeds max_torque_limit");
  }

  out->mMinForceLimit = motor.min_force_limit;
  out->mMaxForceLimit = motor.max_force_limit;
  out->mMinTorqueLimit = motor.min_torque_limit;
  out->mMaxTorqueLimit = motor.max_torque_limit;
  return ZJOLT_RESULT_OK;
}

ZJoltMotorSettings ToCMotor(const JPH::MotorSettings &motor) {
  ZJoltMotorSettings out{};
  out.spring = ToCSpring(motor.mSpringSettings);
  out.min_force_limit = motor.mMinForceLimit;
  out.max_force_limit = motor.mMaxForceLimit;
  out.min_torque_limit = motor.mMinTorqueLimit;
  out.max_torque_limit = motor.mMaxTorqueLimit;
  return out;
}

//===----------------------------------------------------------------------===//
// Limit validation, shared by the joints that spell limits the same way
//===----------------------------------------------------------------------===//

/// A hinge's limits: `HingeConstraint::SetLimits` asserts min is in [-pi, 0]
/// and max is in [0, pi], and the constructor reaches it too.
ZJoltResult CheckHingeLimits(float min, float max) {
  if (!IsFinite(min) || !IsFinite(max)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "hinge limits must be finite");
  }
  if (min > 0.0f || min < -JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "hinge limits_min must be in [-pi, 0]");
  }
  if (max < 0.0f || max > JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "hinge limits_max must be in [0, pi]");
  }
  return ZJOLT_RESULT_OK;
}

/// A slider's limits: `SliderConstraint::SetLimits` asserts min <= 0 <= max.
ZJoltResult CheckSliderLimits(float min, float max) {
  if (std::isnan(min) || std::isnan(max)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "slider limits must not be NaN");
  }
  if (min > 0.0f) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "slider limits_min must be <= 0");
  }
  if (max < 0.0f) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "slider limits_max must be >= 0");
  }
  return ZJOLT_RESULT_OK;
}

/// Both a hinge and a slider assert that their limits are not EQUAL unless the
/// limit spring is soft, with the message "better use a fixed constraint". It
/// is a real constraint on the caller — the solver has no range to work in —
/// so it is reported rather than worked around.
ZJoltResult CheckLimitsNotDegenerate(float min, float max,
                                     const ZJoltSpringSettings &spring,
                                     const char *what) {
  if (min == max && !(spring.frequency_or_stiffness > 0.0f)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
  }
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Construction
//===----------------------------------------------------------------------===//

/// Resolves two body ids and hands the bodies to `build`.
///
/// The bodies are held under one multi-body WRITE lock for the duration:
/// a constraint reads both transforms and then stores raw pointers to them,
/// and a single multi-lock is what makes taking two of them at once free of a
/// lock-ordering question. ZJOLT_BODY_ID_WORLD is not a body and is not
/// locked; Jolt's own `Body::sFixedToWorld` stands in for it, which is exactly
/// what that object exists for.
template <typename Build>
ZJoltResult BuildConstraint(ZJoltPhysicsSystem *system, ZJoltBodyId body1,
                            ZJoltBodyId body2, ZJoltConstraint **out,
                            Build build) {
  const bool world1 = body1 == ZJOLT_BODY_ID_WORLD;
  const bool world2 = body2 == ZJOLT_BODY_ID_WORLD;
  if (world1 && world2) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "both bodies are ZJOLT_BODY_ID_WORLD, so the constraint would join "
        "nothing to nothing");
  }
  if (body1 == body2) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a constraint needs two different bodies");
  }

  const JPH::BodyID ids[2] = {zjolt::ToJolt(body1), zjolt::ToJolt(body2)};
  JPH::BodyLockMultiWrite lock(system->system.GetBodyLockInterface(), ids, 2);

  JPH::Body *jolt1 = world1 ? &JPH::Body::sFixedToWorld : lock.GetBody(0);
  JPH::Body *jolt2 = world2 ? &JPH::Body::sFixedToWorld : lock.GetBody(1);
  if (jolt1 == nullptr || jolt2 == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_BODY_NOT_FOUND,
                           "a body id does not name a body in this system");
  }

  JPH::TwoBodyConstraint *fresh = build(*jolt1, *jolt2);
  if (fresh == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  // Own() is spelled on the RefTarget base: the reference count lives in
  // RefTarget<Constraint>, and Own's static_assert is what says so. A fresh
  // one starts at zero, so this is the caller's first reference, not a second.
  *out = zjolt::ToC(zjolt::Own<JPH::Constraint>(fresh));
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Narrowing a handle
//===----------------------------------------------------------------------===//

/// The handle, checked to actually be the kind the caller is asking about.
///
/// Without this, `zjoltHingeConstraintSetLimits` on a slider is a static_cast
/// to the wrong type and then a write through it. The sub-type is Jolt's own
/// answer, not a tag kept here, so it cannot drift.
template <typename T>
ZJoltResult Narrow(const ZJoltConstraint *constraint,
                   JPH::EConstraintSubType sub_type, const char *what, T **out) {
  if (constraint == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "constraint is NULL");
  }
  JPH::Constraint *base =
      zjolt::ToJolt(const_cast<ZJoltConstraint *>(constraint));
  if (base->GetSubType() != sub_type) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
  }
  *out = static_cast<T *>(base);
  return ZJOLT_RESULT_OK;
}

/// Whether `constraint` is in `system`'s constraint list.
///
/// Jolt exposes membership only as a copy of the whole list — the index it
/// keeps per constraint is private, with ConstraintManager as its only friend
/// — so this is a scan. It is exact, which a cached flag would not stay:
/// destroying a physics system drops its constraints without telling anyone
/// holding one, and a flag set at add time would then be a lie about a system
/// that no longer exists.
bool IsInSystem(const ZJoltPhysicsSystem *system,
                const JPH::Constraint *constraint) {
  const JPH::Constraints constraints = system->system.GetConstraints();
  for (const JPH::Ref<JPH::Constraint> &held : constraints)
    if (held.GetPtr() == constraint) return true;
  return false;
}

/// Whether both of a constraint's bodies live in `system`.
///
/// Adding a constraint whose bodies belong to a different system hands Jolt's
/// island builder body indices from a body manager it is not looking at —
/// out-of-bounds, and silent in a build without asserts. Comparing the
/// POINTER the id resolves to in this system against the one the constraint
/// holds is what makes that exact rather than approximate.
bool BodiesBelongTo(const ZJoltPhysicsSystem *system,
                    const JPH::TwoBodyConstraint *constraint) {
  const JPH::Body *bodies[2] = {constraint->GetBody1(), constraint->GetBody2()};
  JPH::BodyID ids[2] = {bodies[0]->GetID(), bodies[1]->GetID()};

  JPH::BodyLockMultiRead lock(system->system.GetBodyLockInterface(), ids, 2);
  for (int i = 0; i < 2; ++i) {
    // The world body is nobody's, and belongs to every system equally.
    if (bodies[i] == &JPH::Body::sFixedToWorld) continue;
    if (lock.GetBody(i) != bodies[i]) return false;
  }
  return true;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Reference counting
//===----------------------------------------------------------------------===//

void zjoltConstraintAddRef(const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return;
  zjolt::ToJolt(constraint)->AddRef();
}

void zjoltConstraintRelease(const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return;
  zjolt::ToJolt(constraint)->Release();
}

uint32_t zjoltConstraintGetRefCount(const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return 0;
  return zjolt::ToJolt(constraint)->GetRefCount();
}

//===----------------------------------------------------------------------===//
// Membership of a system
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintAdd(ZJoltPhysicsSystem *system,
                               ZJoltConstraint *constraint) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Constraint *base = zjolt::ToJolt(constraint);
  if (base->GetType() != JPH::EConstraintType::TwoBodyConstraint) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint is not a two-body constraint");
  }
  const JPH::TwoBodyConstraint *two =
      static_cast<const JPH::TwoBodyConstraint *>(base);

  if (!BodiesBelongTo(system, two)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "the constraint's bodies do not belong to this physics system");
  }

  // ConstraintManager::Add asserts the constraint is not already in a list,
  // and in a build without asserts it overwrites the index that says where it
  // is — losing the first entry and corrupting the second removal.
  if (IsInSystem(system, base)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint is already added to this system");
  }

  system->system.AddConstraint(base);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConstraintRemove(ZJoltPhysicsSystem *system,
                                  ZJoltConstraint *constraint) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Constraint *base = zjolt::ToJolt(constraint);
  if (!IsInSystem(system, base)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint is not added to this system");
  }

  system->system.RemoveConstraint(base);
  return ZJOLT_RESULT_OK;
}

bool zjoltConstraintIsAdded(const ZJoltPhysicsSystem *system,
                            const ZJoltConstraint *constraint) {
  if (system == nullptr || constraint == nullptr) return false;
  return IsInSystem(system, zjolt::ToJolt(constraint));
}

uint32_t zjoltPhysicsSystemGetNumConstraints(const ZJoltPhysicsSystem *system) {
  if (system == nullptr) return 0;
  return static_cast<uint32_t>(system->system.GetConstraints().size());
}

//===----------------------------------------------------------------------===//
// Common state
//===----------------------------------------------------------------------===//

ZJoltConstraintSubType zjoltConstraintGetSubType(
    const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return ZJOLT_CONSTRAINT_SUB_TYPE_OTHER;
  return ToCSubType(zjolt::ToJolt(constraint)->GetSubType());
}

void zjoltConstraintSetEnabled(ZJoltConstraint *constraint, bool enabled) {
  if (constraint == nullptr) return;
  zjolt::ToJolt(constraint)->SetEnabled(enabled);
}

bool zjoltConstraintIsEnabled(const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return false;
  return zjolt::ToJolt(constraint)->GetEnabled();
}

bool zjoltConstraintIsActive(const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return false;
  return zjolt::ToJolt(constraint)->IsActive();
}

void zjoltConstraintSetUserData(ZJoltConstraint *constraint,
                                uint64_t user_data) {
  if (constraint == nullptr) return;
  zjolt::ToJolt(constraint)->SetUserData(user_data);
}

uint64_t zjoltConstraintGetUserData(const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return 0;
  return zjolt::ToJolt(constraint)->GetUserData();
}

void zjoltConstraintSetPriority(ZJoltConstraint *constraint,
                                uint32_t priority) {
  if (constraint == nullptr) return;
  zjolt::ToJolt(constraint)->SetConstraintPriority(priority);
}

uint32_t zjoltConstraintGetPriority(const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return 0;
  return zjolt::ToJolt(constraint)->GetConstraintPriority();
}

ZJoltResult zjoltConstraintSetNumVelocityStepsOverride(
    ZJoltConstraint *constraint, uint32_t steps) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (steps >= 256) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "velocity step override must be under 256; Jolt stores it in a byte");
  }
  zjolt::ToJolt(constraint)->SetNumVelocityStepsOverride(steps);
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltConstraintGetNumVelocityStepsOverride(
    const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return 0;
  return zjolt::ToJolt(constraint)->GetNumVelocityStepsOverride();
}

ZJoltResult zjoltConstraintSetNumPositionStepsOverride(
    ZJoltConstraint *constraint, uint32_t steps) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (steps >= 256) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "position step override must be under 256; Jolt stores it in a byte");
  }
  zjolt::ToJolt(constraint)->SetNumPositionStepsOverride(steps);
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltConstraintGetNumPositionStepsOverride(
    const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return 0;
  return zjolt::ToJolt(constraint)->GetNumPositionStepsOverride();
}

ZJoltResult zjoltConstraintGetBodies(const ZJoltConstraint *constraint,
                                     ZJoltBodyId *out_body1,
                                     ZJoltBodyId *out_body2) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out_body1, (ZJoltBodyId)ZJOLT_BODY_ID_INVALID),
              zjolt::OutIsEmptyAs(out_body2, (ZJoltBodyId)ZJOLT_BODY_ID_INVALID));
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Constraint *base = zjolt::ToJolt(constraint);
  if (base->GetType() != JPH::EConstraintType::TwoBodyConstraint) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint is not a two-body constraint");
  }
  const JPH::TwoBodyConstraint *two =
      static_cast<const JPH::TwoBodyConstraint *>(base);

  // The world body's id IS the invalid one, which is what
  // ZJOLT_BODY_ID_WORLD is defined as, so no special case is needed here.
  if (out_body1 != nullptr) *out_body1 = zjolt::ToC(two->GetBody1()->GetID());
  if (out_body2 != nullptr) *out_body2 = zjolt::ToC(two->GetBody2()->GetID());
  return ZJOLT_RESULT_OK;
}

void zjoltConstraintResetWarmStart(ZJoltConstraint *constraint) {
  if (constraint == nullptr) return;
  zjolt::ToJolt(constraint)->ResetWarmStart();
}

//===----------------------------------------------------------------------===//
// Paths
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPathConstraintPathCreateHermite(const ZJoltPathPoint *points,
                                                 uint32_t count, bool is_looping,
                                                 ZJoltPathConstraintPath **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(points, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Fewer than two points is not a curve. A non-looping path indexes
  // `points[count - 2]` for its last segment, which at count = 1 reads before
  // the array; a looping one compares its first and last point, which at
  // count = 1 are the same point and trip the assertion below.
  if (count < 2) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a Hermite path needs at least two points");
  }

  for (uint32_t i = 0; i < count; ++i) {
    const ZJoltPathPoint &p = points[i];
    if (!IsFinite(p.position)) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "path point position must be finite");
    }
    const ZJoltResult tangent =
        CheckAxis(p.tangent, "path point tangent must be finite and non-zero");
    if (tangent != ZJOLT_RESULT_OK) return tangent;
    const ZJoltResult normal =
        CheckAxis(p.normal, "path point normal must be finite and non-zero");
    if (normal != ZJOLT_RESULT_OK) return normal;

    // GetPointOnPath builds the frame as normal x tangent and then asserts the
    // result is normalised. A normal parallel to the tangent makes that cross
    // product zero, the normalise a NaN, and the assertion a process abort at
    // the first step rather than at this call.
    const ZJoltResult frame = CheckPerpendicular(
        p.tangent, p.normal,
        "path point normal is parallel to its tangent, so the two do not "
        "define a frame");
    if (frame != ZJOLT_RESULT_OK) return frame;
  }

  if (is_looping) {
    const JPH::Vec3 first = zjolt::ToJolt(points[0].position);
    const JPH::Vec3 last = zjolt::ToJolt(points[count - 1].position);
    if (first.IsClose(last)) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a looping path's first and last point must differ; the loop is "
          "closed by the implied last segment, not by repeating the point");
    }
  }

  JPH::PathConstraintPathHermite *path =
      zjolt::New<JPH::PathConstraintPathHermite>();
  if (path == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  path->SetIsLooping(is_looping);
  for (uint32_t i = 0; i < count; ++i) {
    path->AddPoint(zjolt::ToJolt(points[i].position),
                   zjolt::ToJolt(points[i].tangent),
                   zjolt::ToJolt(points[i].normal));
  }

  *out = zjolt::ToC(zjolt::Own<JPH::PathConstraintPath>(path));
  return ZJOLT_RESULT_OK;
}

void zjoltPathConstraintPathAddRef(const ZJoltPathConstraintPath *path) {
  if (path == nullptr) return;
  zjolt::ToJolt(path)->AddRef();
}

void zjoltPathConstraintPathRelease(const ZJoltPathConstraintPath *path) {
  if (path == nullptr) return;
  zjolt::ToJolt(path)->Release();
}

uint32_t zjoltPathConstraintPathGetRefCount(const ZJoltPathConstraintPath *path) {
  if (path == nullptr) return 0;
  return zjolt::ToJolt(path)->GetRefCount();
}

bool zjoltPathConstraintPathIsLooping(const ZJoltPathConstraintPath *path) {
  if (path == nullptr) return false;
  return zjolt::ToJolt(path)->IsLooping();
}

float zjoltPathConstraintPathGetMaxFraction(const ZJoltPathConstraintPath *path) {
  if (path == nullptr) return 0.0f;
  return zjolt::ToJolt(path)->GetPathMaxFraction();
}

ZJoltResult zjoltPathConstraintPathGetClosestPoint(
    const ZJoltPathConstraintPath *path, const ZJoltVec3 *position,
    float fraction_hint, float *out_fraction) {
  ZJOLT_ENTER(out_fraction);
  if (!zjolt::Present(path, position, out_fraction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!IsFinite(*position) || !IsFinite(fraction_hint)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "position and fraction_hint must be finite");
  }

  *out_fraction =
      zjolt::ToJolt(path)->GetClosestPoint(zjolt::ToJolt(*position), fraction_hint);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintPathGetPointOnPath(
    const ZJoltPathConstraintPath *path, float fraction, ZJoltVec3 *out_position,
    ZJoltVec3 *out_tangent, ZJoltVec3 *out_normal, ZJoltVec3 *out_binormal) {
  ZJOLT_ENTER(out_position, out_tangent, out_normal, out_binormal);
  if (!zjolt::Present(path)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!IsFinite(fraction)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "fraction must be finite");
  }

  JPH::Vec3 position, tangent, normal, binormal;
  zjolt::ToJolt(path)->GetPointOnPath(fraction, position, tangent, normal,
                                      binormal);
  zjolt::WriteVec3(out_position, position);
  zjolt::WriteVec3(out_tangent, tangent);
  zjolt::WriteVec3(out_normal, normal);
  zjolt::WriteVec3(out_binormal, binormal);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Fixed
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateFixed(ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body1, ZJoltBodyId body2,
                                       const ZJoltFixedConstraintDesc *desc,
                                       ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::FixedConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  const ZJoltResult checks[] = {
      CheckPoint(desc->point1, "point1 must be finite"),
      CheckPoint(desc->point2, "point2 must be finite"),
      CheckAxis(desc->axis_x1, "axis_x1 must be finite and non-zero"),
      CheckAxis(desc->axis_y1, "axis_y1 must be finite and non-zero"),
      CheckAxis(desc->axis_x2, "axis_x2 must be finite and non-zero"),
      CheckAxis(desc->axis_y2, "axis_y2 must be finite and non-zero"),
      CheckPerpendicular(desc->axis_x1, desc->axis_y1,
                         "axis_x1 and axis_y1 are parallel, so they do not "
                         "define a constraint frame"),
      CheckPerpendicular(desc->axis_x2, desc->axis_y2,
                         "axis_x2 and axis_y2 are parallel, so they do not "
                         "define a constraint frame"),
  };
  for (ZJoltResult check : checks)
    if (check != ZJOLT_RESULT_OK) return check;

  settings.mAutoDetectPoint = desc->auto_detect_point;
  settings.mPoint1 = zjolt::ToJoltR(desc->point1);
  settings.mAxisX1 = zjolt::ToJolt(desc->axis_x1).Normalized();
  settings.mAxisY1 = zjolt::ToJolt(desc->axis_y1).Normalized();
  settings.mPoint2 = zjolt::ToJoltR(desc->point2);
  settings.mAxisX2 = zjolt::ToJolt(desc->axis_x2).Normalized();
  settings.mAxisY2 = zjolt::ToJolt(desc->axis_y2).Normalized();

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::FixedConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Point
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreatePoint(ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body1, ZJoltBodyId body2,
                                       const ZJoltPointConstraintDesc *desc,
                                       ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::PointConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  ZJoltResult check = CheckPoint(desc->point1, "point1 must be finite");
  if (check != ZJOLT_RESULT_OK) return check;
  check = CheckPoint(desc->point2, "point2 must be finite");
  if (check != ZJOLT_RESULT_OK) return check;

  settings.mPoint1 = zjolt::ToJoltR(desc->point1);
  settings.mPoint2 = zjolt::ToJoltR(desc->point2);

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::PointConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Hinge
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateHinge(ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body1, ZJoltBodyId body2,
                                       const ZJoltHingeConstraintDesc *desc,
                                       ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::HingeConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  const ZJoltResult checks[] = {
      CheckPoint(desc->point1, "point1 must be finite"),
      CheckPoint(desc->point2, "point2 must be finite"),
      CheckAxis(desc->hinge_axis1, "hinge_axis1 must be finite and non-zero"),
      CheckAxis(desc->normal_axis1, "normal_axis1 must be finite and non-zero"),
      CheckAxis(desc->hinge_axis2, "hinge_axis2 must be finite and non-zero"),
      CheckAxis(desc->normal_axis2, "normal_axis2 must be finite and non-zero"),
      CheckPerpendicular(desc->hinge_axis1, desc->normal_axis1,
                         "hinge_axis1 and normal_axis1 are parallel; the "
                         "normal axis is what the hinge angle is measured "
                         "from and must be perpendicular to the hinge"),
      CheckPerpendicular(desc->hinge_axis2, desc->normal_axis2,
                         "hinge_axis2 and normal_axis2 are parallel; the "
                         "normal axis is what the hinge angle is measured "
                         "from and must be perpendicular to the hinge"),
      CheckHingeLimits(desc->limits_min, desc->limits_max),
      CheckLimitsNotDegenerate(desc->limits_min, desc->limits_max,
                               desc->limits_spring,
                               "hinge limits_min equals limits_max with a hard "
                               "limit spring, which locks the hinge; use a "
                               "fixed constraint, or make the spring soft"),
      CheckFloat(desc->max_friction_torque, "max_friction_torque must be finite"),
  };
  for (ZJoltResult check : checks)
    if (check != ZJOLT_RESULT_OK) return check;

  ZJoltResult sub = ToJoltSpring(desc->limits_spring, &settings.mLimitsSpringSettings);
  if (sub != ZJOLT_RESULT_OK) return sub;
  sub = ToJoltMotor(desc->motor, &settings.mMotorSettings);
  if (sub != ZJOLT_RESULT_OK) return sub;

  settings.mPoint1 = zjolt::ToJoltR(desc->point1);
  settings.mHingeAxis1 = zjolt::ToJolt(desc->hinge_axis1).Normalized();
  settings.mNormalAxis1 = zjolt::ToJolt(desc->normal_axis1).Normalized();
  settings.mPoint2 = zjolt::ToJoltR(desc->point2);
  settings.mHingeAxis2 = zjolt::ToJolt(desc->hinge_axis2).Normalized();
  settings.mNormalAxis2 = zjolt::ToJolt(desc->normal_axis2).Normalized();
  settings.mLimitsMin = desc->limits_min;
  settings.mLimitsMax = desc->limits_max;
  settings.mMaxFrictionTorque = desc->max_friction_torque;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::HingeConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Slider
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateSlider(ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body1, ZJoltBodyId body2,
                                        const ZJoltSliderConstraintDesc *desc,
                                        ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SliderConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  const ZJoltResult checks[] = {
      CheckPoint(desc->point1, "point1 must be finite"),
      CheckPoint(desc->point2, "point2 must be finite"),
      CheckAxis(desc->slider_axis1, "slider_axis1 must be finite and non-zero"),
      CheckAxis(desc->normal_axis1, "normal_axis1 must be finite and non-zero"),
      CheckAxis(desc->slider_axis2, "slider_axis2 must be finite and non-zero"),
      CheckAxis(desc->normal_axis2, "normal_axis2 must be finite and non-zero"),
      CheckPerpendicular(desc->slider_axis1, desc->normal_axis1,
                         "slider_axis1 and normal_axis1 are parallel, so they "
                         "do not define a constraint frame"),
      CheckPerpendicular(desc->slider_axis2, desc->normal_axis2,
                         "slider_axis2 and normal_axis2 are parallel, so they "
                         "do not define a constraint frame"),
      CheckSliderLimits(desc->limits_min, desc->limits_max),
      CheckLimitsNotDegenerate(desc->limits_min, desc->limits_max,
                               desc->limits_spring,
                               "slider limits_min equals limits_max with a "
                               "hard limit spring, which locks the slider; use "
                               "a fixed constraint, or make the spring soft"),
      CheckFloat(desc->max_friction_force, "max_friction_force must be finite"),
  };
  for (ZJoltResult check : checks)
    if (check != ZJOLT_RESULT_OK) return check;

  ZJoltResult sub = ToJoltSpring(desc->limits_spring, &settings.mLimitsSpringSettings);
  if (sub != ZJOLT_RESULT_OK) return sub;
  sub = ToJoltMotor(desc->motor, &settings.mMotorSettings);
  if (sub != ZJOLT_RESULT_OK) return sub;

  settings.mAutoDetectPoint = desc->auto_detect_point;
  settings.mPoint1 = zjolt::ToJoltR(desc->point1);
  settings.mSliderAxis1 = zjolt::ToJolt(desc->slider_axis1).Normalized();
  settings.mNormalAxis1 = zjolt::ToJolt(desc->normal_axis1).Normalized();
  settings.mPoint2 = zjolt::ToJoltR(desc->point2);
  settings.mSliderAxis2 = zjolt::ToJolt(desc->slider_axis2).Normalized();
  settings.mNormalAxis2 = zjolt::ToJolt(desc->normal_axis2).Normalized();
  settings.mLimitsMin = desc->limits_min;
  settings.mLimitsMax = desc->limits_max;
  settings.mMaxFrictionForce = desc->max_friction_force;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::SliderConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Distance
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateDistance(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body1, ZJoltBodyId body2,
                                          const ZJoltDistanceConstraintDesc *desc,
                                          ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::DistanceConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  ZJoltResult check = CheckPoint(desc->point1, "point1 must be finite");
  if (check != ZJOLT_RESULT_OK) return check;
  check = CheckPoint(desc->point2, "point2 must be finite");
  if (check != ZJOLT_RESULT_OK) return check;
  check = CheckFloat(desc->min_distance, "min_distance must be finite");
  if (check != ZJOLT_RESULT_OK) return check;
  check = CheckFloat(desc->max_distance, "max_distance must be finite");
  if (check != ZJOLT_RESULT_OK) return check;

  // The constructor resolves the negative sentinels and then calls SetDistance,
  // which asserts min <= max. Once a sentinel is involved the resolved pair is
  // ordered by construction, so the only way to trip it is to give two real
  // distances the wrong way round.
  if (desc->min_distance >= 0.0f && desc->max_distance >= 0.0f &&
      desc->min_distance > desc->max_distance) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "min_distance exceeds max_distance");
  }

  const ZJoltResult spring =
      ToJoltSpring(desc->limits_spring, &settings.mLimitsSpringSettings);
  if (spring != ZJOLT_RESULT_OK) return spring;

  settings.mPoint1 = zjolt::ToJoltR(desc->point1);
  settings.mPoint2 = zjolt::ToJoltR(desc->point2);
  settings.mMinDistance = desc->min_distance;
  settings.mMaxDistance = desc->max_distance;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::DistanceConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Cone
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateCone(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body1, ZJoltBodyId body2,
                                      const ZJoltConeConstraintDesc *desc,
                                      ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::ConeConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  const ZJoltResult checks[] = {
      CheckPoint(desc->point1, "point1 must be finite"),
      CheckPoint(desc->point2, "point2 must be finite"),
      CheckAxis(desc->twist_axis1, "twist_axis1 must be finite and non-zero"),
      CheckAxis(desc->twist_axis2, "twist_axis2 must be finite and non-zero"),
  };
  for (ZJoltResult check : checks)
    if (check != ZJOLT_RESULT_OK) return check;

  // ConeConstraint::SetHalfConeAngle asserts the angle is in [0, pi], and the
  // constructor goes straight through it.
  if (!IsFinite(desc->half_cone_angle) || desc->half_cone_angle < 0.0f ||
      desc->half_cone_angle > JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "half_cone_angle must be in [0, pi]");
  }

  settings.mPoint1 = zjolt::ToJoltR(desc->point1);
  settings.mTwistAxis1 = zjolt::ToJolt(desc->twist_axis1).Normalized();
  settings.mPoint2 = zjolt::ToJoltR(desc->point2);
  settings.mTwistAxis2 = zjolt::ToJolt(desc->twist_axis2).Normalized();
  settings.mHalfConeAngle = desc->half_cone_angle;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::ConeConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Swing-twist
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateSwingTwist(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltSwingTwistConstraintDesc *desc, ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SwingTwistConstraintSettings settings;
  ZJoltResult sub = ToJoltSpace(desc->space, &settings.mSpace);
  if (sub != ZJOLT_RESULT_OK) return sub;
  sub = ToJoltSwingType(desc->swing_type, &settings.mSwingType);
  if (sub != ZJOLT_RESULT_OK) return sub;

  const ZJoltResult checks[] = {
      CheckPoint(desc->position1, "position1 must be finite"),
      CheckPoint(desc->position2, "position2 must be finite"),
      CheckAxis(desc->twist_axis1, "twist_axis1 must be finite and non-zero"),
      CheckAxis(desc->plane_axis1, "plane_axis1 must be finite and non-zero"),
      CheckAxis(desc->twist_axis2, "twist_axis2 must be finite and non-zero"),
      CheckAxis(desc->plane_axis2, "plane_axis2 must be finite and non-zero"),
      CheckPerpendicular(desc->twist_axis1, desc->plane_axis1,
                         "twist_axis1 and plane_axis1 are parallel, so they do "
                         "not define a constraint frame"),
      CheckPerpendicular(desc->twist_axis2, desc->plane_axis2,
                         "twist_axis2 and plane_axis2 are parallel, so they do "
                         "not define a constraint frame"),
      CheckFloat(desc->max_friction_torque, "max_friction_torque must be finite"),
  };
  for (ZJoltResult check : checks)
    if (check != ZJOLT_RESULT_OK) return check;

  // SwingTwistConstraintPart::SetLimits asserts all of these. The half cone
  // angles reach it negated as the minimum of a symmetric range, so a negative
  // one inverts the range and a value past pi leaves it.
  if (!IsFinite(desc->normal_half_cone_angle) ||
      desc->normal_half_cone_angle < 0.0f ||
      desc->normal_half_cone_angle > JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "normal_half_cone_angle must be in [0, pi]");
  }
  if (!IsFinite(desc->plane_half_cone_angle) ||
      desc->plane_half_cone_angle < 0.0f ||
      desc->plane_half_cone_angle > JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "plane_half_cone_angle must be in [0, pi]");
  }
  if (!IsFinite(desc->twist_min_angle) || !IsFinite(desc->twist_max_angle)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "twist angles must be finite");
  }
  if (desc->twist_min_angle > desc->twist_max_angle) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "twist_min_angle exceeds twist_max_angle");
  }

  sub = ToJoltMotor(desc->swing_motor, &settings.mSwingMotorSettings);
  if (sub != ZJOLT_RESULT_OK) return sub;
  sub = ToJoltMotor(desc->twist_motor, &settings.mTwistMotorSettings);
  if (sub != ZJOLT_RESULT_OK) return sub;

  settings.mPosition1 = zjolt::ToJoltR(desc->position1);
  settings.mTwistAxis1 = zjolt::ToJolt(desc->twist_axis1).Normalized();
  settings.mPlaneAxis1 = zjolt::ToJolt(desc->plane_axis1).Normalized();
  settings.mPosition2 = zjolt::ToJoltR(desc->position2);
  settings.mTwistAxis2 = zjolt::ToJolt(desc->twist_axis2).Normalized();
  settings.mPlaneAxis2 = zjolt::ToJolt(desc->plane_axis2).Normalized();
  settings.mNormalHalfConeAngle = desc->normal_half_cone_angle;
  settings.mPlaneHalfConeAngle = desc->plane_half_cone_angle;
  settings.mTwistMinAngle = desc->twist_min_angle;
  settings.mTwistMaxAngle = desc->twist_max_angle;
  settings.mMaxFrictionTorque = desc->max_friction_torque;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::SwingTwistConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Six degrees of freedom
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateSixDof(ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body1, ZJoltBodyId body2,
                                        const ZJoltSixDofConstraintDesc *desc,
                                        ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SixDOFConstraintSettings settings;
  ZJoltResult sub = ToJoltSpace(desc->space, &settings.mSpace);
  if (sub != ZJOLT_RESULT_OK) return sub;
  sub = ToJoltSwingType(desc->swing_type, &settings.mSwingType);
  if (sub != ZJOLT_RESULT_OK) return sub;

  const ZJoltResult checks[] = {
      CheckPoint(desc->position1, "position1 must be finite"),
      CheckPoint(desc->position2, "position2 must be finite"),
      CheckAxis(desc->axis_x1, "axis_x1 must be finite and non-zero"),
      CheckAxis(desc->axis_y1, "axis_y1 must be finite and non-zero"),
      CheckAxis(desc->axis_x2, "axis_x2 must be finite and non-zero"),
      CheckAxis(desc->axis_y2, "axis_y2 must be finite and non-zero"),
      CheckPerpendicular(desc->axis_x1, desc->axis_y1,
                         "axis_x1 and axis_y1 are parallel, so they do not "
                         "define a constraint frame"),
      CheckPerpendicular(desc->axis_x2, desc->axis_y2,
                         "axis_x2 and axis_y2 are parallel, so they do not "
                         "define a constraint frame"),
  };
  for (ZJoltResult check : checks)
    if (check != ZJOLT_RESULT_OK) return check;

  for (int axis = 0; axis < ZJOLT_SIX_DOF_AXIS_COUNT; ++axis) {
    // A NaN limit is the one value SixDOFConstraint's own sanitising cannot
    // fix: every comparison it makes against one is false, so the axis is
    // neither clamped nor zeroed and the NaN reaches the solver.
    if (std::isnan(desc->limit_min[axis]) || std::isnan(desc->limit_max[axis])) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "six-DOF limits must not be NaN");
    }
    const ZJoltResult friction =
        CheckFloat(desc->max_friction[axis], "six-DOF friction must be finite");
    if (friction != ZJOLT_RESULT_OK) return friction;

    sub = ToJoltMotor(desc->motor[axis], &settings.mMotorSettings[axis]);
    if (sub != ZJOLT_RESULT_OK) return sub;

    settings.mMaxFriction[axis] = desc->max_friction[axis];
    settings.mLimitMin[axis] = desc->limit_min[axis];
    settings.mLimitMax[axis] = desc->limit_max[axis];
  }

  for (int axis = 0; axis < ZJOLT_SIX_DOF_TRANSLATION_AXIS_COUNT; ++axis) {
    sub = ToJoltSpring(desc->limits_spring[axis],
                       &settings.mLimitsSpringSettings[axis]);
    if (sub != ZJOLT_RESULT_OK) return sub;
  }

  settings.mPosition1 = zjolt::ToJoltR(desc->position1);
  settings.mAxisX1 = zjolt::ToJolt(desc->axis_x1).Normalized();
  settings.mAxisY1 = zjolt::ToJolt(desc->axis_y1).Normalized();
  settings.mPosition2 = zjolt::ToJoltR(desc->position2);
  settings.mAxisX2 = zjolt::ToJolt(desc->axis_x2).Normalized();
  settings.mAxisY2 = zjolt::ToJolt(desc->axis_y2).Normalized();

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::SixDOFConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Gear
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateGear(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body1, ZJoltBodyId body2,
                                      const ZJoltGearConstraintDesc *desc,
                                      ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::GearConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  ZJoltResult check =
      CheckAxis(desc->hinge_axis1, "hinge_axis1 must be finite and non-zero");
  if (check != ZJOLT_RESULT_OK) return check;
  check = CheckAxis(desc->hinge_axis2, "hinge_axis2 must be finite and non-zero");
  if (check != ZJOLT_RESULT_OK) return check;

  // The ratio divides the two rotations into each other. Zero is a gear that
  // holds body 1 still whatever body 2 does, which is not a gear; an infinity
  // or a NaN spreads into both bodies' velocities on the first step.
  if (!IsFinite(desc->ratio) || desc->ratio == 0.0f) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "gear ratio must be finite and non-zero");
  }

  settings.mHingeAxis1 = zjolt::ToJolt(desc->hinge_axis1).Normalized();
  settings.mHingeAxis2 = zjolt::ToJolt(desc->hinge_axis2).Normalized();
  settings.mRatio = desc->ratio;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::GearConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Rack and pinion
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreateRackAndPinion(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltRackAndPinionConstraintDesc *desc, ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::RackAndPinionConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  ZJoltResult check =
      CheckAxis(desc->hinge_axis, "hinge_axis must be finite and non-zero");
  if (check != ZJOLT_RESULT_OK) return check;
  check = CheckAxis(desc->slider_axis, "slider_axis must be finite and non-zero");
  if (check != ZJOLT_RESULT_OK) return check;

  if (!IsFinite(desc->ratio) || desc->ratio == 0.0f) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "rack and pinion ratio must be finite and non-zero");
  }

  settings.mHingeAxis = zjolt::ToJolt(desc->hinge_axis).Normalized();
  settings.mSliderAxis = zjolt::ToJolt(desc->slider_axis).Normalized();
  settings.mRatio = desc->ratio;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::RackAndPinionConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Pulley
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreatePulley(ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body1, ZJoltBodyId body2,
                                        const ZJoltPulleyConstraintDesc *desc,
                                        ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::PulleyConstraintSettings settings;
  const ZJoltResult space = ToJoltSpace(desc->space, &settings.mSpace);
  if (space != ZJOLT_RESULT_OK) return space;

  const ZJoltResult checks[] = {
      CheckPoint(desc->body_point1, "body_point1 must be finite"),
      CheckPoint(desc->fixed_point1, "fixed_point1 must be finite"),
      CheckPoint(desc->body_point2, "body_point2 must be finite"),
      CheckPoint(desc->fixed_point2, "fixed_point2 must be finite"),
      CheckFloat(desc->min_length, "min_length must be finite"),
      CheckFloat(desc->max_length, "max_length must be finite"),
  };
  for (ZJoltResult check : checks)
    if (check != ZJOLT_RESULT_OK) return check;

  if (!IsFinite(desc->ratio) || desc->ratio <= 0.0f) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "pulley ratio must be finite and greater than zero");
  }

  // The constructor replaces a negative length with the current one and then
  // leaves both in place without checking. Every later SetLength asserts
  // `min >= 0 && min <= max`, so a pair the constructor accepted can be one
  // the setter would refuse; the same rule is applied here so the two agree.
  if (desc->min_length >= 0.0f && desc->max_length >= 0.0f &&
      desc->min_length > desc->max_length) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "min_length exceeds max_length");
  }

  settings.mBodyPoint1 = zjolt::ToJoltR(desc->body_point1);
  settings.mFixedPoint1 = zjolt::ToJoltR(desc->fixed_point1);
  settings.mBodyPoint2 = zjolt::ToJoltR(desc->body_point2);
  settings.mFixedPoint2 = zjolt::ToJoltR(desc->fixed_point2);
  settings.mRatio = desc->ratio;
  settings.mMinLength = desc->min_length;
  settings.mMaxLength = desc->max_length;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::PulleyConstraint>(jolt1, jolt2, settings);
      });
}

//===----------------------------------------------------------------------===//
// Path
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintCreatePath(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body1, ZJoltBodyId body2,
                                      const ZJoltPathConstraintDesc *desc,
                                      ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::Present(desc->path)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::PathConstraintSettings settings;
  ZJoltResult sub = ToJoltPathRotation(desc->rotation_constraint_type,
                                       &settings.mRotationConstraintType);
  if (sub != ZJOLT_RESULT_OK) return sub;
  sub = ToJoltMotor(desc->motor, &settings.mPositionMotorSettings);
  if (sub != ZJOLT_RESULT_OK) return sub;

  if (!IsFinite(desc->path_position)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "path_position must be finite");
  }
  const ZJoltResult fraction =
      CheckFloat(desc->path_fraction, "path_fraction must be finite");
  if (fraction != ZJOLT_RESULT_OK) return fraction;
  const ZJoltResult friction =
      CheckFloat(desc->max_friction_force, "max_friction_force must be finite");
  if (friction != ZJOLT_RESULT_OK) return friction;

  settings.mPath = zjolt::ToJolt(desc->path);
  settings.mPathPosition = zjolt::ToJolt(desc->path_position);
  settings.mPathRotation = zjolt::ToJoltRotation(desc->path_rotation);
  settings.mPathFraction = desc->path_fraction;
  settings.mMaxFrictionForce = desc->max_friction_force;

  return BuildConstraint(
      system, body1, body2, out,
      [&settings](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<JPH::PathConstraint>(jolt1, jolt2, settings);
      });
}

}  // extern "C"
