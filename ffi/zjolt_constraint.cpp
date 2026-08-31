//===----------------------------------------------------------------------===//
// zjolt — constraints.
//
// Almost all of this file is precondition checking, and that is the
// point: Jolt's constraints carry more caller-trippable assertions than
// anything else in the library — a hinge with backwards limits, a motor
// enabled before its settings are valid — and each is an abort in an
// asserts-on build, at a stack naming the solver, not the bad call. The
// rule here is BINDING.md's: never forward a call that would abort.
//
// The rest is the shape of the boundary: descriptors are flat plain
// data; the Jolt `*Settings` object is built on the stack from one, used
// once, and dropped. Nothing of Jolt's crosses.
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
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/StateRecorder.h>

#include <cfloat>

//===----------------------------------------------------------------------===//
// Handle mapping
//
// Two more of the incomplete tags from zjolt_internal.h, converted
// exactly as ZJoltShape is. Live here (not zjolt_internal.h) since only this subsystem introduces or converts them — the "one conversion, one place" rule is about a single definition, not which file it sits in.
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

/// `ZJoltStateRecorder` names a live `JPH::StateRecorder&` Jolt hands the
/// shim during a step, not an object either side owns — nothing here
/// constructs or destroys one, only converts the reference for the length of
/// one SaveState/RestoreState call.
inline JPH::StateRecorder *ToJolt(ZJoltStateRecorder *recorder) {
  return reinterpret_cast<JPH::StateRecorder *>(recorder);
}
inline ZJoltStateRecorder *ToC(JPH::StateRecorder *recorder) {
  return reinterpret_cast<ZJoltStateRecorder *>(recorder);
}

}  // namespace zjolt

namespace {

//===----------------------------------------------------------------------===//
// Scalar and vector validation
//
// A NaN reaching Jolt does not abort; it spreads. Every float a
// descriptor carries is checked for finiteness before use — otherwise the failure surfaces as a body that has vanished, several frames later, in a different subsystem.
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
/// Jolt does not assert on a degenerate one — it feeds `Mat44::GetQuaternion`,
/// producing a non-rotation quaternion and a joint whose frame is not a frame.
/// Worse than an abort, so checked here: finite, and long enough to normalise.
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
/// transform, so an exact right angle is not demanded — only a pair so close to
/// parallel that their cross product carries no direction, the case Jolt cannot
/// recover from.
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
// A C caller can pass any integer for an enum parameter, so every one refuses an unknown value rather than falling through to a silently wrong default.
// Callers take int32_t, not the enum, and convert via zjolt::RawEnum (zjolt_internal.h) — ToC* converts what zjolt itself computed and needs none of this.
//===----------------------------------------------------------------------===//

ZJoltResult ToJoltSpace(int32_t space, JPH::EConstraintSpace *out) {
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

ZJoltResult ToJoltMotorState(int32_t state, JPH::EMotorState *out) {
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

ZJoltResult ToJoltSwingType(int32_t type, JPH::ESwingType *out) {
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

ZJoltResult ToJoltPathRotation(int32_t type,
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
    case JPH::EConstraintSubType::Vehicle:
      return ZJOLT_CONSTRAINT_SUB_TYPE_VEHICLE;
    case JPH::EConstraintSubType::User1:
      // The only C++ type in this library that claims a User* slot — see
      // ZJoltCustomConstraint below.
      return ZJOLT_CONSTRAINT_SUB_TYPE_CUSTOM;
    default:
      break;
  }
  // The three User* slots this library does not claim. A real constraint of
  // a type registered outside it, which is a different fact from NONE.
  return ZJOLT_CONSTRAINT_SUB_TYPE_USER_DEFINED;
}

//===----------------------------------------------------------------------===//
// Springs and motors
//===----------------------------------------------------------------------===//

/// A spring the constraint parts will accept.
///
/// `MotorSettings::IsValid` requires non-negative frequency and damping, so a
/// spring is validated wherever it crosses, not only when attached to a motor —
/// the same struct also drives limit springs, which have no IsValid of their
/// own but produce the same nonsense from a negative stiffness.
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
  // `mode` is a field of a struct the host filled in, so it is read as a
  // plain integer here: switching on it as its enum type would itself be the
  // load. See zjolt::RawEnum in zjolt_internal.h.
  switch (zjolt::RawEnum(spring.mode)) {
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
/// Every joint with a motor asserts `mMotorSettings.IsValid()` before switching
/// one on, so an invalid descriptor fails several frames later, not here.
/// Fields are assigned directly, not passed to a MotorSettings constructor:
/// that asserts IsValid() itself, so building one to inspect would abort.
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
/// Held under one multi-body WRITE lock for the duration — a constraint
/// reads both transforms and stores raw pointers, and one multi-lock
/// avoids a lock-ordering question. ZJOLT_BODY_ID_WORLD is not a body
/// and is not locked; Jolt's own `Body::sFixedToWorld` stands in for it.
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

/// The handle as a TwoBodyConstraint: the two bodies and the constraint frame.
///
/// Every kind this ABI can create is one, so this cannot fail for a
/// zjoltConstraintCreate* handle — but it is checked, not assumed:
/// zjoltVehicleConstraintAsConstraint hands back a JPH::VehicleConstraint,
/// which derives from Constraint DIRECTLY, not a TwoBodyConstraint.
ZJoltResult NarrowTwoBody(const ZJoltConstraint *constraint,
                          const JPH::TwoBodyConstraint **out) {
  if (constraint == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "constraint is NULL");
  }
  const JPH::Constraint *base = zjolt::ToJolt(constraint);
  if (base->GetType() != JPH::EConstraintType::TwoBodyConstraint) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint is not a two-body constraint");
  }
  *out = static_cast<const JPH::TwoBodyConstraint *>(base);
  return ZJOLT_RESULT_OK;
}

/// Whether `constraint` is in `system`'s constraint list.
///
/// Jolt exposes membership only as a copy of the whole list — its
/// per-constraint index is private to ConstraintManager — so this is a
/// scan. Exact, unlike a cached flag: destroying a system drops its
/// constraints without telling anyone still holding one.
bool IsInSystem(const ZJoltPhysicsSystem *system,
                const JPH::Constraint *constraint) {
  const JPH::Constraints constraints = system->system.GetConstraints();
  for (const JPH::Ref<JPH::Constraint> &held : constraints)
    if (held.GetPtr() == constraint) return true;
  return false;
}

/// Whether both of a constraint's bodies live in `system`.
///
/// Adding one whose bodies belong to a different system hands Jolt's island
/// builder body indices from a body manager it is not looking at —
/// out-of-bounds, silent without asserts. Compares the POINTER the id resolves
/// to here against the one the constraint holds, for an exact answer.
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


/// A six-DOF axis, checked to be one.
///
/// SixDOFConstraint indexes `mLimitMin`/`mMaxFriction`/`mMotorSettings` by
/// enumerator, unchecked: out-of-range is an out-of-bounds access, not an
/// assertion. `translations_only` limits to translation-axis soft limits. Takes
/// the raw integer, like the conversions above (zjolt::RawEnum).
ZJoltResult CheckSixDofAxis(int32_t axis, bool translations_only,
                            JPH::SixDOFConstraintSettings::EAxis *out) {
  const int value = axis;
  const int limit = translations_only
                        ? ZJOLT_SIX_DOF_TRANSLATION_AXIS_COUNT
                        : ZJOLT_SIX_DOF_AXIS_COUNT;
  if (value < 0 || value >= limit) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        translations_only
            ? "axis must be one of the three translation axes; Jolt has no "
              "soft rotation limits"
            : "axis is not a ZJoltSixDofAxis");
  }
  *out = static_cast<JPH::SixDOFConstraintSettings::EAxis>(value);
  return ZJOLT_RESULT_OK;
}

/// One of the auxiliary constraints a gear or rack and pinion is told about.
///
/// NULL means "none": both joints fall back to matching velocities only. A
/// constraint of the wrong kind is refused here, since
/// SolvePositionConstraint's own cast asserts `false, "Unsupported"` during a
/// step, not at the call that was wrong.
ZJoltResult CheckAuxiliary(const ZJoltConstraint *auxiliary,
                           JPH::EConstraintSubType expected, const char *what,
                           const JPH::Constraint **out) {
  if (auxiliary == nullptr) {
    *out = nullptr;
    return ZJOLT_RESULT_OK;
  }
  const JPH::Constraint *jolt = zjolt::ToJolt(auxiliary);
  if (jolt->GetSubType() != expected)
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
  *out = jolt;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Custom constraints
//
// ZJoltCustomConstraint forwards every solver virtual to a host's function pointers, once per virtual rather than once per Jacobian operation.
// What crosses is ZJoltSolverBodyPair, a POD snapshot built from the live bodies before the callback and written back after.
//===----------------------------------------------------------------------===//

/// A settings object with nowhere to round-trip a host's callbacks or user
/// pointer through: those live only on the constraint itself. `Create` is
/// therefore unreachable in practice — nothing in this library calls it, and
/// a host cannot either, since ZJoltCustomConstraintSettings is not a type
/// this ABI exposes — but TwoBodyConstraintSettings declares it pure virtual,
/// so a concrete override is what the type system asks for.
class ZJoltCustomConstraintSettings final : public JPH::TwoBodyConstraintSettings {
 public:
  JPH::TwoBodyConstraint *Create(JPH::Body &, JPH::Body &) const override {
    return nullptr;
  }
};

class ZJoltCustomConstraint final : public JPH::TwoBodyConstraint {
 public:
  ZJoltCustomConstraint(JPH::Body &inBody1, JPH::Body &inBody2,
                        const ZJoltCustomConstraintSettings &inSettings,
                        const ZJoltCustomConstraintCallbacks &inCallbacks,
                        void *inUser, JPH::Mat44Arg inToBody1,
                        JPH::Mat44Arg inToBody2)
      : JPH::TwoBodyConstraint(inBody1, inBody2, inSettings),
        callbacks_(inCallbacks),
        user_(inUser),
        to_body1_(inToBody1),
        to_body2_(inToBody2) {}

  ~ZJoltCustomConstraint() override {
    if (callbacks_.destroy != nullptr) callbacks_.destroy(user_);
  }

  void *UserData() const { return user_; }

  JPH::EConstraintSubType GetSubType() const override {
    return JPH::EConstraintSubType::User1;
  }

  void NotifyShapeChanged(const JPH::BodyID &inBodyID,
                          JPH::Vec3Arg inDeltaCOM) override {
    callbacks_.notify_shape_changed(user_, zjolt::ToC(inBodyID),
                                    zjolt::ToC(inDeltaCOM));
  }

  void ResetWarmStart() override { callbacks_.reset_warm_start(user_); }

  bool IsActive() const override {
    return JPH::TwoBodyConstraint::IsActive() && callbacks_.is_active(user_);
  }

  // Jolt's own solver does not grant every phase the same body access:
  // JobSetupVelocityConstraints and JobSolvePositionConstraints both take
  // BodyAccess::Grant(velocity = None, ...) (PhysicsSystem.cpp), the same
  // grant every built-in constraint's own SetupVelocityConstraint /
  // SolvePositionConstraint honours by never touching velocity. Snapshot
  // and WriteBackVelocity below follow that split — see Snapshot's `bool`.

  void SetupVelocityConstraint(float inDeltaTime) override {
    ZJoltSolverBodyPair pair = Snapshot(/*include_velocity=*/false);
    callbacks_.setup_velocity(user_, &pair, inDeltaTime);
  }

  void WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override {
    ZJoltSolverBodyPair pair = Snapshot(/*include_velocity=*/true);
    callbacks_.warm_start_velocity(user_, &pair, inWarmStartImpulseRatio);
    WriteBackVelocity(pair);
  }

  bool SolveVelocityConstraint(float inDeltaTime) override {
    ZJoltSolverBodyPair pair = Snapshot(/*include_velocity=*/true);
    const bool changed = callbacks_.solve_velocity(user_, &pair, inDeltaTime);
    WriteBackVelocity(pair);
    return changed;
  }

  bool SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override {
    ZJoltSolverBodyPair pair = Snapshot(/*include_velocity=*/false);
    const bool changed =
        callbacks_.solve_position(user_, &pair, inDeltaTime, inBaumgarte);
    ApplyPositionDeltas(pair);
    return changed;
  }

  JPH::Mat44 GetConstraintToBody1Matrix() const override { return to_body1_; }
  JPH::Mat44 GetConstraintToBody2Matrix() const override { return to_body2_; }

#ifdef JPH_DEBUG_RENDERER
  void DrawConstraint(JPH::DebugRenderer *inRenderer) const override {
    // Deliberately unwired. `inRenderer` is a live JPH::DebugRenderer* and
    // the callback wants a ZJoltDebugRenderer*, and the only conversion
    // between the two lives inside zjolt_debug.cpp, out of reach here — see
    // this file's implementer's report for the rest of that note.
    (void)inRenderer;
  }
#endif  // JPH_DEBUG_RENDERER

  void SaveState(JPH::StateRecorder &inStream) const override {
    JPH::TwoBodyConstraint::SaveState(inStream);
    callbacks_.save_state(user_, zjolt::ToC(&inStream));
  }

  void RestoreState(JPH::StateRecorder &inStream) override {
    JPH::TwoBodyConstraint::RestoreState(inStream);
    callbacks_.restore_state(user_, zjolt::ToC(&inStream));
  }

  JPH::Ref<JPH::ConstraintSettings> GetConstraintSettings() const override {
    ZJoltCustomConstraintSettings *settings =
        zjolt::New<ZJoltCustomConstraintSettings>();
    if (settings == nullptr) return nullptr;
    ToConstraintSettings(*settings);
    return settings;
  }

 private:
  // `include_velocity` is false from SetupVelocityConstraint and
  // SolvePositionConstraint: Jolt's own solver grants no velocity access at
  // all during those two phases (see the note above), and
  // `Body::GetLinearVelocity`/`GetAngularVelocity` assert exactly that.
  // Reading them there — even a value the callback goes on to ignore — is
  // the abort this shim exists to never forward.
  static void FillSolverBody(ZJoltSolverBody &out, const JPH::Body &body,
                             bool include_velocity) {
    out.is_dynamic = body.IsDynamic();
    out.center_of_mass = zjolt::ToC(JPH::Vec3(body.GetCenterOfMassPosition()));
    out.rotation = zjolt::ToC(body.GetRotation());
    if (include_velocity) {
      out.linear_velocity = zjolt::ToC(body.GetLinearVelocity());
      out.angular_velocity = zjolt::ToC(body.GetAngularVelocity());
    } else {
      out.linear_velocity = ZJoltVec3{0, 0, 0};
      out.angular_velocity = ZJoltVec3{0, 0, 0};
    }
    out.position_delta = ZJoltVec3{0, 0, 0};
    out.rotation_delta = ZJoltVec3{0, 0, 0};
    if (out.is_dynamic) {
      const JPH::MotionProperties *mp = body.GetMotionProperties();
      out.inverse_mass = mp->GetInverseMass();
      // World space, computed once here rather than once per Jacobian
      // operation on the Zig side — the reason this whole seam exists.
      const JPH::Mat44 inv_i =
          mp->GetInverseInertiaForRotation(JPH::Mat44::sRotation(body.GetRotation()));
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
          out.inverse_inertia[row * 3 + col] = inv_i(row, col);
    } else {
      out.inverse_mass = 0.0f;
      std::memset(out.inverse_inertia, 0, sizeof(out.inverse_inertia));
    }
  }

  ZJoltSolverBodyPair Snapshot(bool include_velocity) const {
    ZJoltSolverBodyPair pair{};
    FillSolverBody(pair.body1, *mBody1, include_velocity);
    FillSolverBody(pair.body2, *mBody2, include_velocity);
    return pair;
  }

  void WriteBackVelocity(const ZJoltSolverBodyPair &pair) {
    if (mBody1->IsDynamic()) {
      mBody1->SetLinearVelocity(zjolt::ToJolt(pair.body1.linear_velocity));
      mBody1->SetAngularVelocity(zjolt::ToJolt(pair.body1.angular_velocity));
    }
    if (mBody2->IsDynamic()) {
      mBody2->SetLinearVelocity(zjolt::ToJolt(pair.body2.linear_velocity));
      mBody2->SetAngularVelocity(zjolt::ToJolt(pair.body2.angular_velocity));
    }
  }

  void ApplyPositionDeltas(const ZJoltSolverBodyPair &pair) {
    if (mBody1->IsDynamic()) {
      mBody1->AddPositionStep(zjolt::ToJolt(pair.body1.position_delta));
      mBody1->AddRotationStep(zjolt::ToJolt(pair.body1.rotation_delta));
    }
    if (mBody2->IsDynamic()) {
      mBody2->AddPositionStep(zjolt::ToJolt(pair.body2.position_delta));
      mBody2->AddRotationStep(zjolt::ToJolt(pair.body2.rotation_delta));
    }
  }

  ZJoltCustomConstraintCallbacks callbacks_;
  void *user_;
  JPH::Mat44 to_body1_;
  JPH::Mat44 to_body2_;
};

}  // namespace

//===----------------------------------------------------------------------===//
// Narrowing, as macros
//
// Both have to return from the CALLER, which a function cannot do, and
// between them open nearly every entry point below — a per-kind check that can be half-written is one that eventually will be.
//===----------------------------------------------------------------------===//

/// Refuses a NULL handle and a handle of the wrong kind in one step, and binds
/// the narrowed pointer to VAR.
#define ZJOLT_NARROW(TYPE, SUBTYPE, VAR)                                \
  JPH::TYPE *VAR = nullptr;                                             \
  do {                                                                  \
    const ZJoltResult zjolt_narrowed_ =                                 \
        Narrow<JPH::TYPE>(constraint, JPH::EConstraintSubType::SUBTYPE, \
                          "constraint is not a " #TYPE, &VAR);          \
    if (zjolt_narrowed_ != ZJOLT_RESULT_OK) return zjolt_narrowed_;     \
  } while (false)

/// The same, plus the axis argument every six-DOF accessor carries. That one
/// is converted here, at the entry point that receives it from the host — see
/// zjolt::RawEnum in zjolt_internal.h.
#define ZJOLT_SIX_DOF(VAR, AXIS_VAR, TRANSLATIONS_ONLY)                 \
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, VAR);                          \
  JPH::SixDOFConstraintSettings::EAxis AXIS_VAR;                        \
  const int32_t zjolt_raw_axis_ = zjolt::RawEnum(axis);                 \
  do {                                                                  \
    const ZJoltResult zjolt_axis_ =                                     \
        CheckSixDofAxis(zjolt_raw_axis_, TRANSLATIONS_ONLY, &AXIS_VAR); \
    if (zjolt_axis_ != ZJOLT_RESULT_OK) return zjolt_axis_;             \
  } while (false)

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

ZJoltResult zjoltPhysicsSystemGetConstraints(const ZJoltPhysicsSystem *system,
                                             ZJoltConstraint **out_constraints,
                                             uint32_t capacity,
                                             uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Constraints constraints = system->system.GetConstraints();
  const uint32_t count = static_cast<uint32_t>(constraints.size());
  *out_count = count;
  if (out_constraints == nullptr) return ZJOLT_RESULT_OK;

  const uint32_t to_copy = count < capacity ? count : capacity;
  for (uint32_t i = 0; i < to_copy; ++i) {
    JPH::Constraint *held = constraints[i].GetPtr();
    held->AddRef();
    out_constraints[i] = zjolt::ToC(held);
  }
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Common state
//===----------------------------------------------------------------------===//

ZJoltConstraintSubType zjoltConstraintGetSubType(
    const ZJoltConstraint *constraint) {
  if (constraint == nullptr) return ZJOLT_CONSTRAINT_SUB_TYPE_NONE;
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

ZJoltResult zjoltConstraintActivate(ZJoltPhysicsSystem *system,
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

  system->system.GetBodyInterface().ActivateConstraint(two);
  return ZJOLT_RESULT_OK;
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

  const JPH::TwoBodyConstraint *two = nullptr;
  const ZJoltResult narrowed = NarrowTwoBody(constraint, &two);
  if (narrowed != ZJOLT_RESULT_OK) return narrowed;

  // The world body's id IS the invalid one, which is what
  // ZJOLT_BODY_ID_WORLD is defined as, so no special case is needed here.
  if (out_body1 != nullptr) *out_body1 = zjolt::ToC(two->GetBody1()->GetID());
  if (out_body2 != nullptr) *out_body2 = zjolt::ToC(two->GetBody2()->GetID());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConstraintGetConstraintToBody1Matrix(
    const ZJoltConstraint *constraint, ZJoltMat44 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::TwoBodyConstraint *two = nullptr;
  const ZJoltResult narrowed = NarrowTwoBody(constraint, &two);
  if (narrowed != ZJOLT_RESULT_OK) return narrowed;

  *out = zjolt::ToC(two->GetConstraintToBody1Matrix());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConstraintGetConstraintToBody2Matrix(
    const ZJoltConstraint *constraint, ZJoltMat44 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::TwoBodyConstraint *two = nullptr;
  const ZJoltResult narrowed = NarrowTwoBody(constraint, &two);
  if (narrowed != ZJOLT_RESULT_OK) return narrowed;

  *out = zjolt::ToC(two->GetConstraintToBody2Matrix());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConstraintSetDrawSize(ZJoltConstraint *constraint, float size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  // Jolt does not check this and would draw a NaN-sized frame, which in a
  // renderer that culls by bounds is a frame that silently never appears.
  const ZJoltResult checked = CheckFloat(size, "size must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  zjolt::ToJolt(constraint)->SetDrawConstraintSize(size);
  return ZJOLT_RESULT_OK;
#else
  (void)size;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltConstraintGetDrawSize(const ZJoltConstraint *constraint,
                                       float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(constraint, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  *out = zjolt::ToJolt(constraint)->GetDrawConstraintSize();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

void zjoltConstraintResetWarmStart(ZJoltConstraint *constraint) {
  if (constraint == nullptr) return;
  zjolt::ToJolt(constraint)->ResetWarmStart();
}

//===----------------------------------------------------------------------===//
// Constraint settings
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConstraintGetConstraintSettings(const ZJoltConstraint *constraint,
                                                 ZJoltConstraintSettings **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(constraint, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Ref<JPH::ConstraintSettings> settings =
      zjolt::ToJolt(constraint)->GetConstraintSettings();
  if (settings == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not build settings for this constraint");
  }

  // `settings` hands back one reference; keep it alive past this Ref's own
  // destructor by taking a second one, the same way every other *Create
  // entry point in this file hands a caller-owned reference out.
  JPH::ConstraintSettings *raw = settings.GetPtr();
  raw->AddRef();
  zjolt::HandleCreated();
  *out = zjolt::ToC(raw);
  return ZJOLT_RESULT_OK;
}

void zjoltConstraintSettingsAddRef(const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->AddRef();
}

void zjoltConstraintSettingsRelease(const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->Release();
  zjolt::HandleDestroyed();
}

uint32_t zjoltConstraintSettingsGetRefCount(const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(settings)->GetRefCount());
}

bool zjoltConstraintSettingsGetEnabled(const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return false;
  return zjolt::ToJolt(settings)->mEnabled;
}

uint32_t zjoltConstraintSettingsGetConstraintPriority(
    const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return 0;
  return zjolt::ToJolt(settings)->mConstraintPriority;
}

uint32_t zjoltConstraintSettingsGetNumVelocityStepsOverride(
    const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return 0;
  return zjolt::ToJolt(settings)->mNumVelocityStepsOverride;
}

uint32_t zjoltConstraintSettingsGetNumPositionStepsOverride(
    const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return 0;
  return zjolt::ToJolt(settings)->mNumPositionStepsOverride;
}

float zjoltConstraintSettingsGetDrawConstraintSize(
    const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return 0.0f;
  return zjolt::ToJolt(settings)->mDrawConstraintSize;
}

uint64_t zjoltConstraintSettingsGetUserData(const ZJoltConstraintSettings *settings) {
  if (settings == nullptr) return 0;
  return zjolt::ToJolt(settings)->mUserData;
}

namespace {
constexpr uint8_t kConstraintSettingsStreamMagic[4] = {'Z', 'C', 'S', 'T'};
}  // namespace

ZJoltResult zjoltConstraintSettingsSaveBinaryState(
    const ZJoltConstraintSettings *settings, const ZJoltStream *stream) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings, stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to save through");
  }

  zjolt::HostStream host(*stream);
  zjolt::WriteStreamHeader(host, kConstraintSettingsStreamMagic);
  zjolt::ToJolt(settings)->SaveBinaryState(host);

  if (host.IsFailed()) {
    return zjolt::SetError(
        ZJOLT_RESULT_IO_ERROR,
        "the stream failed while writing the constraint settings");
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConstraintSettingsRestoreBinaryState(
    const ZJoltStream *stream, ZJoltConstraintSettings **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(stream, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "stream needs read, is_eof and is_failed to restore through");
  }

  zjolt::HostStream host(*stream);
  const ZJoltResult header = zjolt::ReadStreamHeader(
      host, kConstraintSettingsStreamMagic,
      "not constraint settings saved by zjoltConstraintSettingsSaveBinaryState");
  if (header != ZJOLT_RESULT_OK) return header;

  JPH::ConstraintSettings::ConstraintResult result =
      JPH::ConstraintSettings::sRestoreFromBinaryState(host);

  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT, result.GetError().c_str());
  }
  if (host.IsFailed()) {
    return zjolt::SetError(
        ZJOLT_RESULT_IO_ERROR,
        "the stream failed while reading the constraint settings");
  }

  JPH::ConstraintSettings *raw = result.Get().GetPtr();
  raw->AddRef();
  zjolt::HandleCreated();
  *out = zjolt::ToC(raw);
  return ZJOLT_RESULT_OK;
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
  // The descriptor is the host's, so its space is converted here, where it is
  // read — see zjolt::RawEnum in zjolt_internal.h. Every Create below does the
  // same with its own.
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  ZJoltResult sub = ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
  if (sub != ZJOLT_RESULT_OK) return sub;
  sub = ToJoltSwingType(zjolt::RawEnum(desc->swing_type),
                        &settings.mSwingType);
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
  ZJoltResult sub = ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
  if (sub != ZJOLT_RESULT_OK) return sub;
  sub = ToJoltSwingType(zjolt::RawEnum(desc->swing_type),
                        &settings.mSwingType);
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
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  const ZJoltResult space =
      ToJoltSpace(zjolt::RawEnum(desc->space), &settings.mSpace);
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
  ZJoltResult sub =
      ToJoltPathRotation(zjolt::RawEnum(desc->rotation_constraint_type),
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

//===----------------------------------------------------------------------===//
// Per-kind state
//
// Every accessor below opens the same way: ZJOLT_ENTER with its outputs, then
// ZJOLT_NARROW, or ZJOLT_SIX_DOF where an axis argument needs checking too.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Fixed
//===----------------------------------------------------------------------===//

ZJoltResult zjoltFixedConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(FixedConstraint, Fixed, fixed);
  *out = zjolt::ToC(fixed->GetTotalLambdaPosition());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltFixedConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(FixedConstraint, Fixed, fixed);
  *out = zjolt::ToC(fixed->GetTotalLambdaRotation());
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Point
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPointConstraintSetPoint1(ZJoltConstraint *constraint,
                                          ZJoltConstraintSpace space,
                                          const ZJoltRVec3 *point) {
  ZJOLT_ENTER();
  if (!zjolt::Present(point)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PointConstraint, Point, joint);

  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  JPH::EConstraintSpace jolt_space;
  const ZJoltResult converted = ToJoltSpace(zjolt::RawEnum(space), &jolt_space);
  if (converted != ZJOLT_RESULT_OK) return converted;
  const ZJoltResult checked = CheckPoint(*point, "point must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;

  joint->SetPoint1(jolt_space, zjolt::ToJoltR(*point));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPointConstraintSetPoint2(ZJoltConstraint *constraint,
                                          ZJoltConstraintSpace space,
                                          const ZJoltRVec3 *point) {
  ZJOLT_ENTER();
  if (!zjolt::Present(point)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PointConstraint, Point, joint);

  JPH::EConstraintSpace jolt_space;
  const ZJoltResult converted = ToJoltSpace(zjolt::RawEnum(space), &jolt_space);
  if (converted != ZJOLT_RESULT_OK) return converted;
  const ZJoltResult checked = CheckPoint(*point, "point must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;

  joint->SetPoint2(jolt_space, zjolt::ToJoltR(*point));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPointConstraintGetLocalSpacePoint1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PointConstraint, Point, joint);
  *out = zjolt::ToC(joint->GetLocalSpacePoint1());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPointConstraintGetLocalSpacePoint2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PointConstraint, Point, joint);
  *out = zjolt::ToC(joint->GetLocalSpacePoint2());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPointConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PointConstraint, Point, joint);
  *out = zjolt::ToC(joint->GetTotalLambdaPosition());
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Hinge
//===----------------------------------------------------------------------===//

ZJoltResult zjoltHingeConstraintGetLocalSpacePoint1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = zjolt::ToC(hinge->GetLocalSpacePoint1());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetLocalSpacePoint2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = zjolt::ToC(hinge->GetLocalSpacePoint2());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetLocalSpaceHingeAxis1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = zjolt::ToC(hinge->GetLocalSpaceHingeAxis1());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetLocalSpaceHingeAxis2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = zjolt::ToC(hinge->GetLocalSpaceHingeAxis2());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetLocalSpaceNormalAxis1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = zjolt::ToC(hinge->GetLocalSpaceNormalAxis1());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetLocalSpaceNormalAxis2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = zjolt::ToC(hinge->GetLocalSpaceNormalAxis2());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetCurrentAngle(const ZJoltConstraint *constraint,
                                                float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = hinge->GetCurrentAngle();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintSetLimits(ZJoltConstraint *constraint, float min,
                                          float max) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  const ZJoltResult checked = CheckHingeLimits(min, max);
  if (checked != ZJOLT_RESULT_OK) return checked;
  hinge->SetLimits(min, max);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetLimits(const ZJoltConstraint *constraint,
                                          float *out_min, float *out_max) {
  ZJOLT_ENTER(out_min, out_max);
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  if (out_min != nullptr) *out_min = hinge->GetLimitsMin();
  if (out_max != nullptr) *out_max = hinge->GetLimitsMax();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintHasLimits(const ZJoltConstraint *constraint,
                                          bool *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = hinge->HasLimits();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintSetLimitsSpringSettings(
    ZJoltConstraint *constraint, const ZJoltSpringSettings *spring) {
  ZJOLT_ENTER();
  if (!zjolt::Present(spring)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);

  JPH::SpringSettings settings;
  const ZJoltResult converted = ToJoltSpring(*spring, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  hinge->SetLimitsSpringSettings(settings);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetLimitsSpringSettings(
    const ZJoltConstraint *constraint, ZJoltSpringSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = ToCSpring(hinge->GetLimitsSpringSettings());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintSetMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor) {
  ZJOLT_ENTER();
  if (!zjolt::Present(motor)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);

  JPH::MotorSettings settings;
  const ZJoltResult converted = ToJoltMotor(*motor, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  hinge->GetMotorSettings() = settings;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = ToCMotor(hinge->GetMotorSettings());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintSetMotorState(ZJoltConstraint *constraint,
                                              ZJoltMotorState state) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);

  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h. Every SetMotorState below does the
  // same.
  JPH::EMotorState jolt_state;
  const ZJoltResult converted =
      ToJoltMotorState(zjolt::RawEnum(state), &jolt_state);
  if (converted != ZJOLT_RESULT_OK) return converted;

  // The assertion this stands in for. Every route to these settings validates
  // them, so reaching this is a bug here rather than in the call — but a
  // returned error is still the right way to say so.
  if (jolt_state != JPH::EMotorState::Off && !hinge->GetMotorSettings().IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the hinge's motor settings are not valid, so the "
                           "motor cannot be switched on");
  }

  hinge->SetMotorState(jolt_state);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetMotorState(const ZJoltConstraint *constraint,
                                              ZJoltMotorState *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = ToCMotorState(hinge->GetMotorState());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintSetTargetAngularVelocity(
    ZJoltConstraint *constraint, float angular_velocity) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  const ZJoltResult checked =
      CheckFloat(angular_velocity, "angular_velocity must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  hinge->SetTargetAngularVelocity(angular_velocity);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetTargetAngularVelocity(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = hinge->GetTargetAngularVelocity();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintSetTargetAngle(ZJoltConstraint *constraint,
                                               float angle) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  const ZJoltResult checked = CheckFloat(angle, "angle must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  hinge->SetTargetAngle(angle);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetTargetAngle(const ZJoltConstraint *constraint,
                                               float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = hinge->GetTargetAngle();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintSetTargetOrientation(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(orientation)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  hinge->SetTargetOrientationBS(zjolt::ToJoltRotation(*orientation));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintSetMaxFrictionTorque(ZJoltConstraint *constraint,
                                                     float torque) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  const ZJoltResult checked = CheckFloat(torque, "torque must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  hinge->SetMaxFrictionTorque(torque);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetMaxFrictionTorque(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = hinge->GetMaxFrictionTorque();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = zjolt::ToC(hinge->GetTotalLambdaPosition());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, float *out_x, float *out_y) {
  ZJOLT_ENTER(out_x, out_y);
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  const JPH::Vector<2> lambda = hinge->GetTotalLambdaRotation();
  if (out_x != nullptr) *out_x = lambda[0];
  if (out_y != nullptr) *out_y = lambda[1];
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetTotalLambdaRotationLimits(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = hinge->GetTotalLambdaRotationLimits();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHingeConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(HingeConstraint, Hinge, hinge);
  *out = hinge->GetTotalLambdaMotor();
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Slider
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSliderConstraintGetCurrentPosition(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = slider->GetCurrentPosition();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintSetLimits(ZJoltConstraint *constraint,
                                           float min, float max) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  const ZJoltResult checked = CheckSliderLimits(min, max);
  if (checked != ZJOLT_RESULT_OK) return checked;
  slider->SetLimits(min, max);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetLimits(const ZJoltConstraint *constraint,
                                           float *out_min, float *out_max) {
  ZJOLT_ENTER(out_min, out_max);
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  if (out_min != nullptr) *out_min = slider->GetLimitsMin();
  if (out_max != nullptr) *out_max = slider->GetLimitsMax();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintHasLimits(const ZJoltConstraint *constraint,
                                           bool *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = slider->HasLimits();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintSetLimitsSpringSettings(
    ZJoltConstraint *constraint, const ZJoltSpringSettings *spring) {
  ZJOLT_ENTER();
  if (!zjolt::Present(spring)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);

  JPH::SpringSettings settings;
  const ZJoltResult converted = ToJoltSpring(*spring, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  slider->SetLimitsSpringSettings(settings);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetLimitsSpringSettings(
    const ZJoltConstraint *constraint, ZJoltSpringSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = ToCSpring(slider->GetLimitsSpringSettings());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintSetMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor) {
  ZJOLT_ENTER();
  if (!zjolt::Present(motor)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);

  JPH::MotorSettings settings;
  const ZJoltResult converted = ToJoltMotor(*motor, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  slider->GetMotorSettings() = settings;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = ToCMotor(slider->GetMotorSettings());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintSetMotorState(ZJoltConstraint *constraint,
                                               ZJoltMotorState state) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SliderConstraint, Slider, slider);

  JPH::EMotorState jolt_state;
  const ZJoltResult converted =
      ToJoltMotorState(zjolt::RawEnum(state), &jolt_state);
  if (converted != ZJOLT_RESULT_OK) return converted;

  if (jolt_state != JPH::EMotorState::Off &&
      !slider->GetMotorSettings().IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the slider's motor settings are not valid, so the "
                           "motor cannot be switched on");
  }

  slider->SetMotorState(jolt_state);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetMotorState(const ZJoltConstraint *constraint,
                                               ZJoltMotorState *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = ToCMotorState(slider->GetMotorState());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintSetTargetVelocity(ZJoltConstraint *constraint,
                                                   float velocity) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  const ZJoltResult checked = CheckFloat(velocity, "velocity must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  slider->SetTargetVelocity(velocity);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetTargetVelocity(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = slider->GetTargetVelocity();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintSetTargetPosition(ZJoltConstraint *constraint,
                                                   float position) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  const ZJoltResult checked = CheckFloat(position, "position must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  slider->SetTargetPosition(position);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetTargetPosition(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = slider->GetTargetPosition();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintSetMaxFrictionForce(ZJoltConstraint *constraint,
                                                     float force) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  const ZJoltResult checked = CheckFloat(force, "force must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  slider->SetMaxFrictionForce(force);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetMaxFrictionForce(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = slider->GetMaxFrictionForce();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, float *out_x, float *out_y) {
  ZJOLT_ENTER(out_x, out_y);
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  const JPH::Vector<2> lambda = slider->GetTotalLambdaPosition();
  if (out_x != nullptr) *out_x = lambda[0];
  if (out_y != nullptr) *out_y = lambda[1];
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetTotalLambdaPositionLimits(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = slider->GetTotalLambdaPositionLimits();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = zjolt::ToC(slider->GetTotalLambdaRotation());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSliderConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SliderConstraint, Slider, slider);
  *out = slider->GetTotalLambdaMotor();
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Distance
//===----------------------------------------------------------------------===//

ZJoltResult zjoltDistanceConstraintSetDistance(ZJoltConstraint *constraint,
                                               float min, float max) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(DistanceConstraint, Distance, distance);

  if (!IsFinite(min) || !IsFinite(max)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "distance limits must be finite");
  }
  if (min > max) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "min distance exceeds max distance");
  }

  distance->SetDistance(min, max);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltDistanceConstraintGetDistance(const ZJoltConstraint *constraint,
                                               float *out_min, float *out_max) {
  ZJOLT_ENTER(out_min, out_max);
  ZJOLT_NARROW(DistanceConstraint, Distance, distance);
  if (out_min != nullptr) *out_min = distance->GetMinDistance();
  if (out_max != nullptr) *out_max = distance->GetMaxDistance();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltDistanceConstraintSetLimitsSpringSettings(
    ZJoltConstraint *constraint, const ZJoltSpringSettings *spring) {
  ZJOLT_ENTER();
  if (!zjolt::Present(spring)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(DistanceConstraint, Distance, distance);

  JPH::SpringSettings settings;
  const ZJoltResult converted = ToJoltSpring(*spring, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  distance->SetLimitsSpringSettings(settings);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltDistanceConstraintGetLimitsSpringSettings(
    const ZJoltConstraint *constraint, ZJoltSpringSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(DistanceConstraint, Distance, distance);
  *out = ToCSpring(distance->GetLimitsSpringSettings());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltDistanceConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(DistanceConstraint, Distance, distance);
  *out = distance->GetTotalLambdaPosition();
  return ZJOLT_RESULT_OK;
}


//===----------------------------------------------------------------------===//
// Cone
//===----------------------------------------------------------------------===//

ZJoltResult zjoltConeConstraintSetHalfConeAngle(ZJoltConstraint *constraint,
                                                float half_cone_angle) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(ConeConstraint, Cone, cone);
  if (!IsFinite(half_cone_angle) || half_cone_angle < 0.0f ||
      half_cone_angle > JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "half_cone_angle must be in [0, pi]");
  }
  cone->SetHalfConeAngle(half_cone_angle);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConeConstraintGetCosHalfConeAngle(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(ConeConstraint, Cone, cone);
  *out = cone->GetCosHalfConeAngle();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConeConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(ConeConstraint, Cone, cone);
  *out = zjolt::ToC(cone->GetTotalLambdaPosition());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConeConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(ConeConstraint, Cone, cone);
  *out = cone->GetTotalLambdaRotation();
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Swing-twist
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSwingTwistConstraintGetLocalSpacePosition1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetLocalSpacePosition1());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetLocalSpacePosition2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetLocalSpacePosition2());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetConstraintToBody1(
    const ZJoltConstraint *constraint, ZJoltQuat *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetConstraintToBody1());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetConstraintToBody2(
    const ZJoltConstraint *constraint, ZJoltQuat *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetConstraintToBody2());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetNormalHalfConeAngle(
    ZJoltConstraint *constraint, float angle) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  if (!IsFinite(angle) || angle < 0.0f || angle > JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "normal half cone angle must be in [0, pi]");
  }
  joint->SetNormalHalfConeAngle(angle);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetNormalHalfConeAngle(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = joint->GetNormalHalfConeAngle();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetPlaneHalfConeAngle(
    ZJoltConstraint *constraint, float angle) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  if (!IsFinite(angle) || angle < 0.0f || angle > JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "plane half cone angle must be in [0, pi]");
  }
  joint->SetPlaneHalfConeAngle(angle);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetPlaneHalfConeAngle(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = joint->GetPlaneHalfConeAngle();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetTwistLimits(ZJoltConstraint *constraint,
                                                    float min, float max) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);

  if (!IsFinite(min) || !IsFinite(max)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "twist limits must be finite");
  }
  if (min < -JPH::JPH_PI || max > JPH::JPH_PI) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "twist limits must be in [-pi, pi]");
  }
  if (min > max) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "twist minimum exceeds twist maximum");
  }

  // Jolt recomputes the constraint part inside each setter, asserting
  // min <= max — one order passes through an invalid intermediate state,
  // the other does not. At least one is always safe: if new_min >
  // old_max and new_max < old_min, then new_min > old_max >= old_min >
  // new_max >= new_min, which cannot hold.
  if (min <= joint->GetTwistMaxAngle()) {
    joint->SetTwistMinAngle(min);
    joint->SetTwistMaxAngle(max);
  } else {
    joint->SetTwistMaxAngle(max);
    joint->SetTwistMinAngle(min);
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTwistLimits(
    const ZJoltConstraint *constraint, float *out_min, float *out_max) {
  ZJOLT_ENTER(out_min, out_max);
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  if (out_min != nullptr) *out_min = joint->GetTwistMinAngle();
  if (out_max != nullptr) *out_max = joint->GetTwistMaxAngle();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetSwingMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor) {
  ZJOLT_ENTER();
  if (!zjolt::Present(motor)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);

  JPH::MotorSettings settings;
  const ZJoltResult converted = ToJoltMotor(*motor, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  joint->GetSwingMotorSettings() = settings;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetSwingMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = ToCMotor(joint->GetSwingMotorSettings());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetSwingMotorState(
    ZJoltConstraint *constraint, ZJoltMotorState state) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);

  JPH::EMotorState jolt_state;
  const ZJoltResult converted =
      ToJoltMotorState(zjolt::RawEnum(state), &jolt_state);
  if (converted != ZJOLT_RESULT_OK) return converted;

  if (jolt_state != JPH::EMotorState::Off &&
      !joint->GetSwingMotorSettings().IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the swing motor settings are not valid, so the "
                           "motor cannot be switched on");
  }

  joint->SetSwingMotorState(jolt_state);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetSwingMotorState(
    const ZJoltConstraint *constraint, ZJoltMotorState *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = ToCMotorState(joint->GetSwingMotorState());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetTwistMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor) {
  ZJOLT_ENTER();
  if (!zjolt::Present(motor)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);

  JPH::MotorSettings settings;
  const ZJoltResult converted = ToJoltMotor(*motor, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  joint->GetTwistMotorSettings() = settings;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTwistMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = ToCMotor(joint->GetTwistMotorSettings());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetTwistMotorState(
    ZJoltConstraint *constraint, ZJoltMotorState state) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);

  JPH::EMotorState jolt_state;
  const ZJoltResult converted =
      ToJoltMotorState(zjolt::RawEnum(state), &jolt_state);
  if (converted != ZJOLT_RESULT_OK) return converted;

  if (jolt_state != JPH::EMotorState::Off &&
      !joint->GetTwistMotorSettings().IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the twist motor settings are not valid, so the "
                           "motor cannot be switched on");
  }

  joint->SetTwistMotorState(jolt_state);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTwistMotorState(
    const ZJoltConstraint *constraint, ZJoltMotorState *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = ToCMotorState(joint->GetTwistMotorState());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetMaxFrictionTorque(
    ZJoltConstraint *constraint, float torque) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  const ZJoltResult checked = CheckFloat(torque, "torque must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  joint->SetMaxFrictionTorque(torque);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetMaxFrictionTorque(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = joint->GetMaxFrictionTorque();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetTargetAngularVelocity(
    ZJoltConstraint *constraint, const ZJoltVec3 *angular_velocity) {
  ZJOLT_ENTER();
  if (!zjolt::Present(angular_velocity)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  if (!IsFinite(*angular_velocity)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "angular_velocity must be finite");
  }
  joint->SetTargetAngularVelocityCS(zjolt::ToJolt(*angular_velocity));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTargetAngularVelocity(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetTargetAngularVelocityCS());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetTargetAngularVelocityBodySpace(
    ZJoltConstraint *constraint, const ZJoltVec3 *angular_velocity) {
  ZJOLT_ENTER();
  if (!zjolt::Present(angular_velocity)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  if (!IsFinite(*angular_velocity)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "angular_velocity must be finite");
  }
  joint->SetTargetAngularVelocityBS(zjolt::ToJolt(*angular_velocity));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetTargetOrientation(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(orientation)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  joint->SetTargetOrientationCS(zjolt::ToJoltRotation(*orientation));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintSetTargetOrientationBodySpace(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(orientation)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  joint->SetTargetOrientationBS(zjolt::ToJoltRotation(*orientation));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTargetOrientation(
    const ZJoltConstraint *constraint, ZJoltQuat *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetTargetOrientationCS());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetRotationInConstraintSpace(
    const ZJoltConstraint *constraint, ZJoltQuat *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetRotationInConstraintSpace());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetTotalLambdaPosition());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaTwist(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = joint->GetTotalLambdaTwist();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaSwingY(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = joint->GetTotalLambdaSwingY();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaSwingZ(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = joint->GetTotalLambdaSwingZ();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SwingTwistConstraint, SwingTwist, joint);
  *out = zjolt::ToC(joint->GetTotalLambdaMotor());
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Six degrees of freedom
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSixDofConstraintSetTranslationLimits(
    ZJoltConstraint *constraint, const ZJoltVec3 *min, const ZJoltVec3 *max) {
  ZJOLT_ENTER();
  if (!zjolt::Present(min, max)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  if (!IsFinite(*min) || !IsFinite(*max)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "translation limits must be finite; a NaN is the one value the "
        "constraint's own sanitising cannot fix");
  }
  joint->SetTranslationLimits(zjolt::ToJolt(*min), zjolt::ToJolt(*max));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetRotationLimits(ZJoltConstraint *constraint,
                                                   const ZJoltVec3 *min,
                                                   const ZJoltVec3 *max) {
  ZJOLT_ENTER();
  if (!zjolt::Present(min, max)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  if (!IsFinite(*min) || !IsFinite(*max)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "rotation limits must be finite");
  }
  joint->SetRotationLimits(zjolt::ToJolt(*min), zjolt::ToJolt(*max));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetLimits(const ZJoltConstraint *constraint,
                                           ZJoltSixDofAxis axis, float *out_min,
                                           float *out_max) {
  ZJOLT_ENTER(out_min, out_max);
  ZJOLT_SIX_DOF(joint, jolt_axis, false);
  if (out_min != nullptr) *out_min = joint->GetLimitsMin(jolt_axis);
  if (out_max != nullptr) *out_max = joint->GetLimitsMax(jolt_axis);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintIsFixedAxis(const ZJoltConstraint *constraint,
                                             ZJoltSixDofAxis axis, bool *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_SIX_DOF(joint, jolt_axis, false);
  *out = joint->IsFixedAxis(jolt_axis);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintIsFreeAxis(const ZJoltConstraint *constraint,
                                            ZJoltSixDofAxis axis, bool *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_SIX_DOF(joint, jolt_axis, false);
  *out = joint->IsFreeAxis(jolt_axis);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetMaxFriction(ZJoltConstraint *constraint,
                                                ZJoltSixDofAxis axis,
                                                float friction) {
  ZJOLT_ENTER();
  ZJOLT_SIX_DOF(joint, jolt_axis, false);
  const ZJoltResult checked = CheckFloat(friction, "friction must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  joint->SetMaxFriction(jolt_axis, friction);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetMaxFriction(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_SIX_DOF(joint, jolt_axis, false);
  *out = joint->GetMaxFriction(jolt_axis);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetMotorSettings(
    ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    const ZJoltMotorSettings *motor) {
  ZJOLT_ENTER();
  if (!zjolt::Present(motor)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_SIX_DOF(joint, jolt_axis, false);

  JPH::MotorSettings settings;
  const ZJoltResult converted = ToJoltMotor(*motor, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  joint->GetMotorSettings(jolt_axis) = settings;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetMotorSettings(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    ZJoltMotorSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_SIX_DOF(joint, jolt_axis, false);
  *out = ToCMotor(joint->GetMotorSettings(jolt_axis));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetMotorState(ZJoltConstraint *constraint,
                                               ZJoltSixDofAxis axis,
                                               ZJoltMotorState state) {
  ZJOLT_ENTER();
  ZJOLT_SIX_DOF(joint, jolt_axis, false);

  JPH::EMotorState jolt_state;
  const ZJoltResult converted =
      ToJoltMotorState(zjolt::RawEnum(state), &jolt_state);
  if (converted != ZJOLT_RESULT_OK) return converted;

  if (jolt_state != JPH::EMotorState::Off &&
      !joint->GetMotorSettings(jolt_axis).IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this axis' motor settings are not valid, so the "
                           "motor cannot be switched on");
  }

  joint->SetMotorState(jolt_axis, jolt_state);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetMotorState(const ZJoltConstraint *constraint,
                                               ZJoltSixDofAxis axis,
                                               ZJoltMotorState *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_SIX_DOF(joint, jolt_axis, false);
  *out = ToCMotorState(joint->GetMotorState(jolt_axis));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetLimitsSpringSettings(
    ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    const ZJoltSpringSettings *spring) {
  ZJOLT_ENTER();
  if (!zjolt::Present(spring)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_SIX_DOF(joint, jolt_axis, true);

  JPH::SpringSettings settings;
  const ZJoltResult converted = ToJoltSpring(*spring, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  joint->SetLimitsSpringSettings(jolt_axis, settings);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetLimitsSpringSettings(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    ZJoltSpringSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_SIX_DOF(joint, jolt_axis, true);
  *out = ToCSpring(joint->GetLimitsSpringSettings(jolt_axis));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetTargetVelocity(ZJoltConstraint *constraint,
                                                   const ZJoltVec3 *velocity) {
  ZJOLT_ENTER();
  if (!zjolt::Present(velocity)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  if (!IsFinite(*velocity)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "velocity must be finite");
  }
  joint->SetTargetVelocityCS(zjolt::ToJolt(*velocity));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetTargetVelocity(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetTargetVelocityCS());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetTargetAngularVelocity(
    ZJoltConstraint *constraint, const ZJoltVec3 *angular_velocity) {
  ZJOLT_ENTER();
  if (!zjolt::Present(angular_velocity)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  if (!IsFinite(*angular_velocity)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "angular_velocity must be finite");
  }
  joint->SetTargetAngularVelocityCS(zjolt::ToJolt(*angular_velocity));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetTargetAngularVelocity(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetTargetAngularVelocityCS());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetTargetPosition(ZJoltConstraint *constraint,
                                                   const ZJoltVec3 *position) {
  ZJOLT_ENTER();
  if (!zjolt::Present(position)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  if (!IsFinite(*position)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "position must be finite");
  }
  joint->SetTargetPositionCS(zjolt::ToJolt(*position));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetTargetPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetTargetPositionCS());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetTargetOrientation(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(orientation)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  joint->SetTargetOrientationCS(zjolt::ToJoltRotation(*orientation));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetTargetOrientation(
    const ZJoltConstraint *constraint, ZJoltQuat *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetTargetOrientationCS());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintSetTargetOrientationBodySpace(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(orientation)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  joint->SetTargetOrientationBS(zjolt::ToJoltRotation(*orientation));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetRotationInConstraintSpace(
    const ZJoltConstraint *constraint, ZJoltQuat *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetRotationInConstraintSpace());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetTotalLambdaPosition());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetTotalLambdaRotation());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetTotalLambdaMotorTranslation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetTotalLambdaMotorTranslation());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSixDofConstraintGetTotalLambdaMotorRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(SixDOFConstraint, SixDOF, joint);
  *out = zjolt::ToC(joint->GetTotalLambdaMotorRotation());
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Gear and rack and pinion
//===----------------------------------------------------------------------===//

ZJoltResult zjoltGearConstraintSetConstraints(ZJoltConstraint *constraint,
                                              ZJoltConstraint *gear1,
                                              ZJoltConstraint *gear2) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(GearConstraint, Gear, gear);

  const JPH::Constraint *jolt1 = nullptr;
  const JPH::Constraint *jolt2 = nullptr;
  ZJoltResult checked = CheckAuxiliary(
      gear1, JPH::EConstraintSubType::Hinge,
      "gear1 must be a hinge constraint or NULL", &jolt1);
  if (checked != ZJOLT_RESULT_OK) return checked;
  checked = CheckAuxiliary(gear2, JPH::EConstraintSubType::Hinge,
                           "gear2 must be a hinge constraint or NULL", &jolt2);
  if (checked != ZJOLT_RESULT_OK) return checked;

  gear->SetConstraints(jolt1, jolt2);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltGearConstraintGetTotalLambda(const ZJoltConstraint *constraint,
                                              float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(GearConstraint, Gear, gear);
  *out = gear->GetTotalLambda();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRackAndPinionConstraintSetConstraints(
    ZJoltConstraint *constraint, ZJoltConstraint *pinion,
    ZJoltConstraint *rack) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(RackAndPinionConstraint, RackAndPinion, joint);

  const JPH::Constraint *jolt_pinion = nullptr;
  const JPH::Constraint *jolt_rack = nullptr;
  ZJoltResult checked = CheckAuxiliary(
      pinion, JPH::EConstraintSubType::Hinge,
      "pinion must be a hinge constraint or NULL", &jolt_pinion);
  if (checked != ZJOLT_RESULT_OK) return checked;
  checked = CheckAuxiliary(rack, JPH::EConstraintSubType::Slider,
                           "rack must be a slider constraint or NULL",
                           &jolt_rack);
  if (checked != ZJOLT_RESULT_OK) return checked;

  joint->SetConstraints(jolt_pinion, jolt_rack);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRackAndPinionConstraintGetTotalLambda(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(RackAndPinionConstraint, RackAndPinion, joint);
  *out = joint->GetTotalLambda();
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Pulley
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPulleyConstraintSetLength(ZJoltConstraint *constraint,
                                           float min, float max) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(PulleyConstraint, Pulley, pulley);

  if (!IsFinite(min) || !IsFinite(max)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "pulley lengths must be finite");
  }
  if (min < 0.0f) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "pulley min length must not be negative");
  }
  if (min > max) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "pulley min length exceeds max length");
  }

  pulley->SetLength(min, max);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPulleyConstraintGetLength(const ZJoltConstraint *constraint,
                                           float *out_min, float *out_max) {
  ZJOLT_ENTER(out_min, out_max);
  ZJOLT_NARROW(PulleyConstraint, Pulley, pulley);
  if (out_min != nullptr) *out_min = pulley->GetMinLength();
  if (out_max != nullptr) *out_max = pulley->GetMaxLength();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPulleyConstraintGetCurrentLength(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PulleyConstraint, Pulley, pulley);
  *out = pulley->GetCurrentLength();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPulleyConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PulleyConstraint, Pulley, pulley);
  *out = pulley->GetTotalLambdaPosition();
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Path
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPathConstraintSetPath(ZJoltConstraint *constraint,
                                       const ZJoltPathConstraintPath *path,
                                       float fraction) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(PathConstraint, Path, joint);
  const ZJoltResult checked = CheckFloat(fraction, "fraction must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;

  // A NULL path is legal and makes the constraint inactive; SetPath checks for
  // it, and IsActive reports false so the solver never dereferences it.
  joint->SetPath(path == nullptr ? nullptr : zjolt::ToJolt(path), fraction);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetPath(const ZJoltConstraint *constraint,
                                       const ZJoltPathConstraintPath **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  *out = nullptr;
  ZJOLT_NARROW(PathConstraint, Path, joint);

  // Borrowed, and const_cast only to reach the ToC overload: the handle the
  // caller gets back is a `const` one, and nothing here mutates the path.
  *out = zjolt::ToC(const_cast<JPH::PathConstraintPath *>(joint->GetPath()));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetPathFraction(const ZJoltConstraint *constraint,
                                               float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = joint->GetPathFraction();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintSetMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor) {
  ZJOLT_ENTER();
  if (!zjolt::Present(motor)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);

  JPH::MotorSettings settings;
  const ZJoltResult converted = ToJoltMotor(*motor, &settings);
  if (converted != ZJOLT_RESULT_OK) return converted;

  joint->GetPositionMotorSettings() = settings;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = ToCMotor(joint->GetPositionMotorSettings());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintSetMotorState(ZJoltConstraint *constraint,
                                             ZJoltMotorState state) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(PathConstraint, Path, joint);

  JPH::EMotorState jolt_state;
  const ZJoltResult converted =
      ToJoltMotorState(zjolt::RawEnum(state), &jolt_state);
  if (converted != ZJOLT_RESULT_OK) return converted;

  if (jolt_state != JPH::EMotorState::Off &&
      !joint->GetPositionMotorSettings().IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the path motor settings are not valid, so the "
                           "motor cannot be switched on");
  }

  joint->SetPositionMotorState(jolt_state);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetMotorState(const ZJoltConstraint *constraint,
                                             ZJoltMotorState *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = ToCMotorState(joint->GetPositionMotorState());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintSetTargetVelocity(ZJoltConstraint *constraint,
                                                 float velocity) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(PathConstraint, Path, joint);
  const ZJoltResult checked = CheckFloat(velocity, "velocity must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  joint->SetTargetVelocity(velocity);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetTargetVelocity(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = joint->GetTargetVelocity();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintSetTargetPathFraction(
    ZJoltConstraint *constraint, float fraction) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(PathConstraint, Path, joint);
  const ZJoltResult checked = CheckFloat(fraction, "fraction must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;

  // `JPH_ASSERT(mPath->IsLooping() || (inFraction >= 0 && inFraction <=
  // mPath->GetPathMaxFraction()))`. Note the dereference: the assertion reads
  // the path unconditionally, so a constraint with no path at all cannot take
  // this call either.
  const JPH::PathConstraintPath *path = joint->GetPath();
  if (path == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the constraint has no path, so no fraction along "
                           "one can be targeted");
  }
  if (!path->IsLooping() &&
      (fraction < 0.0f || fraction > path->GetPathMaxFraction())) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "target fraction is off the end of a non-looping path; only a looping "
        "path wraps");
  }

  joint->SetTargetPathFraction(fraction);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetTargetPathFraction(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = joint->GetTargetPathFraction();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintSetMaxFrictionForce(ZJoltConstraint *constraint,
                                                   float force) {
  ZJOLT_ENTER();
  ZJOLT_NARROW(PathConstraint, Path, joint);
  const ZJoltResult checked = CheckFloat(force, "force must be finite");
  if (checked != ZJOLT_RESULT_OK) return checked;
  joint->SetMaxFrictionForce(force);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetMaxFrictionForce(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = joint->GetMaxFrictionForce();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, float *out_x, float *out_y) {
  ZJOLT_ENTER(out_x, out_y);
  ZJOLT_NARROW(PathConstraint, Path, joint);
  const JPH::Vector<2> lambda = joint->GetTotalLambdaPosition();
  if (out_x != nullptr) *out_x = lambda[0];
  if (out_y != nullptr) *out_y = lambda[1];
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetTotalLambdaPositionLimits(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = joint->GetTotalLambdaPositionLimits();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetTotalLambdaRotationHinge(
    const ZJoltConstraint *constraint, float *out_x, float *out_y) {
  ZJOLT_ENTER(out_x, out_y);
  ZJOLT_NARROW(PathConstraint, Path, joint);
  const JPH::Vector<2> lambda = joint->GetTotalLambdaRotationHinge();
  if (out_x != nullptr) *out_x = lambda[0];
  if (out_y != nullptr) *out_y = lambda[1];
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = zjolt::ToC(joint->GetTotalLambdaRotation());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPathConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJOLT_NARROW(PathConstraint, Path, joint);
  *out = joint->GetTotalLambdaMotor();
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Custom constraints
//===----------------------------------------------------------------------===//

void zjoltStateRecorderWriteBytes(ZJoltStateRecorder *recorder,
                                  const void *data, size_t size) {
  if (recorder == nullptr || data == nullptr || size == 0) return;
  zjolt::ToJolt(recorder)->WriteBytes(data, size);
}

void zjoltStateRecorderReadBytes(ZJoltStateRecorder *recorder, void *data,
                                 size_t size) {
  if (recorder == nullptr || data == nullptr || size == 0) return;
  zjolt::ToJolt(recorder)->ReadBytes(data, size);
}

ZJoltResult zjoltConstraintCreateCustom(ZJoltPhysicsSystem *system,
                                        const ZJoltCustomConstraintDesc *desc,
                                        ZJoltConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const ZJoltCustomConstraintCallbacks &cb = desc->callbacks;
  if (cb.setup_velocity == nullptr || cb.warm_start_velocity == nullptr ||
      cb.solve_velocity == nullptr || cb.solve_position == nullptr ||
      cb.reset_warm_start == nullptr || cb.is_active == nullptr ||
      cb.notify_shape_changed == nullptr || cb.save_state == nullptr ||
      cb.restore_state == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a required custom constraint callback is NULL; only draw and "
        "destroy may be");
  }

  if (desc->num_velocity_steps_override >= 256 ||
      desc->num_position_steps_override >= 256) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "step override must be under 256; Jolt stores it in a byte");
  }

  for (int i = 0; i < 16; ++i) {
    if (!IsFinite(desc->constraint_to_body1.m[i]) ||
        !IsFinite(desc->constraint_to_body2.m[i])) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "constraint_to_body1 and constraint_to_body2 must be finite");
    }
  }

  const JPH::Mat44 to_body1 = zjolt::ToJolt(desc->constraint_to_body1);
  const JPH::Mat44 to_body2 = zjolt::ToJolt(desc->constraint_to_body2);

  ZJoltCustomConstraintSettings settings;
  settings.mEnabled = desc->enabled;
  settings.mNumVelocityStepsOverride = desc->num_velocity_steps_override;
  settings.mNumPositionStepsOverride = desc->num_position_steps_override;
  settings.mDrawConstraintSize = desc->draw_constraint_size;

  return BuildConstraint(
      system, desc->body1, desc->body2, out,
      [&](JPH::Body &jolt1, JPH::Body &jolt2) -> JPH::TwoBodyConstraint * {
        return zjolt::New<ZJoltCustomConstraint>(jolt1, jolt2, settings, cb,
                                                 desc->user, to_body1,
                                                 to_body2);
      });
}

ZJoltResult zjoltConstraintGetCustomUserData(const ZJoltConstraint *constraint,
                                             void **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(constraint, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Constraint *base = zjolt::ToJolt(constraint);
  if (base->GetSubType() != JPH::EConstraintSubType::User1) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint is not a custom constraint");
  }
  *out = static_cast<const ZJoltCustomConstraint *>(base)->UserData();
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
