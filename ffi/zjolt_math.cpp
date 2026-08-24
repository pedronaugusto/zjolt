//===----------------------------------------------------------------------===//
// zjolt — quaternion and matrix algebra.
//
// Every function here is a thin call into a `JPH::Quat`, `JPH::Mat44` or
// `JPH::RMat44` method, converted at the boundary through the same
// zjolt::ToJolt / zjolt::ToC family every other subsystem uses. What makes
// this file worth having rather than leaving a C consumer to write the
// arithmetic themselves is not the arithmetic — it is the handful of
// JPH_ASSERT preconditions Jolt guards these operations with, which this
// checks and reports as ZJOLT_RESULT_INVALID_ARGUMENT instead of leaving to
// abort, and the sign and axis-order conventions (Hamilton product order,
// X-then-Y-then-Z Euler angles, SLERP's shortest-path correction) that are
// exactly the things a caller gets backwards.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"
#include "zjolt_math.h"

namespace {

/// The tolerance Jolt's own `JPH_ASSERT(IsNormalized())` calls use when they
/// call it with no argument — `Quat::IsNormalized`'s default (Quat.h:60).
/// Named here so every check below and zjoltQuatIsNormalized's documentation
/// agree with one source rather than a repeated literal.
constexpr float kQuatNormalizedTolerance = 1.0e-5f;

/// Refuses `q` unless it is unit length at Jolt's own default tolerance,
/// naming the assert this stands in for in the reported message. Shared by
/// every entry point that calls through a Jolt method asserting
/// `IsNormalized()` on its own `*this` or on a `Quat` parameter.
ZJoltResult CheckQuatNormalized(const JPH::Quat &q, const char *why) {
  if (q.IsNormalized(kQuatNormalizedTolerance)) return ZJOLT_RESULT_OK;
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, why);
}

}  // namespace

//===----------------------------------------------------------------------===//
// Quaternions
//===----------------------------------------------------------------------===//

void zjoltQuatMultiply(const ZJoltQuat *lhs, const ZJoltQuat *rhs,
                        ZJoltQuat *out) {
  if (lhs == nullptr || rhs == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*lhs) * zjolt::ToJolt(*rhs));
}

ZJoltResult zjoltQuatRotateVector(const ZJoltQuat *q, const ZJoltVec3 *v,
                                   ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(q, v, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Quat jolt_q = zjolt::ToJolt(*q);
  const ZJoltResult normalized = CheckQuatNormalized(
      jolt_q,
      "q is not unit length; Jolt's operator*(Vec3) asserts IsNormalized() "
      "rather than tolerating drift, so renormalise with zjoltQuatNormalize "
      "first if q merely drifted after being integrated over many frames");
  if (normalized != ZJOLT_RESULT_OK) return normalized;

  zjolt::WriteVec3(out, jolt_q * zjolt::ToJolt(*v));
  return ZJOLT_RESULT_OK;
}

void zjoltQuatInverse(const ZJoltQuat *q, ZJoltQuat *out) {
  if (q == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*q).Inversed());
}

void zjoltQuatConjugate(const ZJoltQuat *q, ZJoltQuat *out) {
  if (q == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*q).Conjugated());
}

float zjoltQuatDot(const ZJoltQuat *a, const ZJoltQuat *b) {
  if (a == nullptr || b == nullptr) return 0.0f;
  return zjolt::ToJolt(*a).Dot(zjolt::ToJolt(*b));
}

bool zjoltQuatIsNormalized(const ZJoltQuat *q, float tolerance) {
  if (q == nullptr) return false;
  return zjolt::ToJolt(*q).IsNormalized(tolerance);
}

void zjoltQuatNormalize(const ZJoltQuat *q, ZJoltQuat *out) {
  if (q == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*q).Normalized());
}

ZJoltResult zjoltQuatFromAxisAngle(const ZJoltVec3 *axis, float radians,
                                    ZJoltQuat *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(axis, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Vec3 jolt_axis = zjolt::ToJolt(*axis);
  if (!jolt_axis.IsNormalized()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "axis is not unit length; Jolt's Quat::sRotation asserts "
        "inAxis.IsNormalized()");
  }

  zjolt::WriteQuat(out, JPH::Quat::sRotation(jolt_axis, radians));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltQuatGetAxisAngle(const ZJoltQuat *q, ZJoltVec3 *out_axis,
                                   float *out_angle) {
  ZJOLT_ENTER(out_axis, out_angle);
  if (!zjolt::Present(q, out_axis, out_angle))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Quat jolt_q = zjolt::ToJolt(*q);
  const ZJoltResult normalized = CheckQuatNormalized(
      jolt_q, "q is not unit length; Jolt's GetAxisAngle asserts "
              "IsNormalized()");
  if (normalized != ZJOLT_RESULT_OK) return normalized;

  JPH::Vec3 axis;
  float angle;
  jolt_q.GetAxisAngle(axis, angle);
  zjolt::WriteVec3(out_axis, axis);
  *out_angle = angle;
  return ZJOLT_RESULT_OK;
}

void zjoltQuatFromTo(const ZJoltVec3 *from, const ZJoltVec3 *to,
                      ZJoltQuat *out) {
  if (from == nullptr || to == nullptr) return;
  zjolt::WriteQuat(out,
                   JPH::Quat::sFromTo(zjolt::ToJolt(*from), zjolt::ToJolt(*to)));
}

void zjoltQuatFromEulerAngles(const ZJoltVec3 *angles_radians,
                               ZJoltQuat *out) {
  if (angles_radians == nullptr) return;
  zjolt::WriteQuat(out, JPH::Quat::sEulerAngles(zjolt::ToJolt(*angles_radians)));
}

void zjoltQuatGetEulerAngles(const ZJoltQuat *q, ZJoltVec3 *out) {
  if (q == nullptr) return;
  zjolt::WriteVec3(out, zjolt::ToJolt(*q).GetEulerAngles());
}

void zjoltQuatGetPerpendicular(const ZJoltQuat *q, ZJoltQuat *out) {
  if (q == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*q).GetPerpendicular());
}

float zjoltQuatGetRotationAngle(const ZJoltQuat *q, const ZJoltVec3 *axis) {
  if (q == nullptr || axis == nullptr) return 0.0f;
  return zjolt::ToJolt(*q).GetRotationAngle(zjolt::ToJolt(*axis));
}

void zjoltQuatGetTwist(const ZJoltQuat *q, const ZJoltVec3 *axis,
                        ZJoltQuat *out) {
  if (q == nullptr || axis == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*q).GetTwist(zjolt::ToJolt(*axis)));
}

void zjoltQuatGetSwingTwist(const ZJoltQuat *q, ZJoltQuat *out_swing,
                             ZJoltQuat *out_twist) {
  if (q == nullptr) return;
  JPH::Quat swing;
  JPH::Quat twist;
  zjolt::ToJolt(*q).GetSwingTwist(swing, twist);
  zjolt::WriteQuat(out_swing, swing);
  zjolt::WriteQuat(out_twist, twist);
}

void zjoltQuatLerp(const ZJoltQuat *a, const ZJoltQuat *b, float t,
                    ZJoltQuat *out) {
  if (a == nullptr || b == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*a).LERP(zjolt::ToJolt(*b), t));
}

void zjoltQuatSlerp(const ZJoltQuat *a, const ZJoltQuat *b, float t,
                     ZJoltQuat *out) {
  if (a == nullptr || b == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*a).SLERP(zjolt::ToJolt(*b), t));
}

//===----------------------------------------------------------------------===//
// Vector interpolation
//===----------------------------------------------------------------------===//

void zjoltVec3Lerp(const ZJoltVec3 *a, const ZJoltVec3 *b, float t,
                    ZJoltVec3 *out) {
  if (out == nullptr) return;
  if (a == nullptr || b == nullptr) return;
  out->x = a->x + (b->x - a->x) * t;
  out->y = a->y + (b->y - a->y) * t;
  out->z = a->z + (b->z - a->z) * t;
}

void zjoltRVec3Lerp(const ZJoltRVec3 *a, const ZJoltRVec3 *b, float t,
                     ZJoltRVec3 *out) {
  if (out == nullptr) return;
  if (a == nullptr || b == nullptr) return;
  const ZJoltReal ft = static_cast<ZJoltReal>(t);
  out->x = a->x + (b->x - a->x) * ft;
  out->y = a->y + (b->y - a->y) * ft;
  out->z = a->z + (b->z - a->z) * ft;
}

//===----------------------------------------------------------------------===//
// ZJoltMat44
//===----------------------------------------------------------------------===//

ZJoltResult zjoltMat44FromRotationTranslation(const ZJoltQuat *rotation,
                                               const ZJoltVec3 *translation,
                                               ZJoltMat44 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(rotation, translation, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Quat jolt_rotation = zjolt::ToJolt(*rotation);
  const ZJoltResult normalized = CheckQuatNormalized(
      jolt_rotation,
      "rotation is not unit length; Jolt's Mat44::sRotation asserts "
      "IsNormalized()");
  if (normalized != ZJOLT_RESULT_OK) return normalized;

  zjolt::WriteMat44(out, JPH::Mat44::sRotationTranslation(
                             jolt_rotation, zjolt::ToJolt(*translation)));
  return ZJOLT_RESULT_OK;
}

void zjoltMat44Multiply(const ZJoltMat44 *a, const ZJoltMat44 *b,
                         ZJoltMat44 *out) {
  if (a == nullptr || b == nullptr) return;
  zjolt::WriteMat44(out, zjolt::ToJolt(*a) * zjolt::ToJolt(*b));
}

void zjoltMat44Inverse(const ZJoltMat44 *m, ZJoltMat44 *out) {
  if (m == nullptr) return;
  zjolt::WriteMat44(out, zjolt::ToJolt(*m).Inversed());
}

void zjoltMat44InverseRotationTranslation(const ZJoltMat44 *m,
                                           ZJoltMat44 *out) {
  if (m == nullptr) return;
  zjolt::WriteMat44(out, zjolt::ToJolt(*m).InversedRotationTranslation());
}

void zjoltMat44TransformPoint(const ZJoltMat44 *m, const ZJoltVec3 *point,
                               ZJoltVec3 *out) {
  if (m == nullptr || point == nullptr) return;
  zjolt::WriteVec3(out, zjolt::ToJolt(*m) * zjolt::ToJolt(*point));
}

void zjoltMat44TransformDirection(const ZJoltMat44 *m,
                                   const ZJoltVec3 *direction,
                                   ZJoltVec3 *out) {
  if (m == nullptr || direction == nullptr) return;
  zjolt::WriteVec3(out,
                   zjolt::ToJolt(*m).Multiply3x3(zjolt::ToJolt(*direction)));
}

//===----------------------------------------------------------------------===//
// ZJoltRMat44
//===----------------------------------------------------------------------===//

ZJoltResult zjoltRMat44FromRotationTranslation(const ZJoltQuat *rotation,
                                                const ZJoltRVec3 *translation,
                                                ZJoltRMat44 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(rotation, translation, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Quat jolt_rotation = zjolt::ToJolt(*rotation);
  const ZJoltResult normalized = CheckQuatNormalized(
      jolt_rotation,
      "rotation is not unit length; Jolt's Mat44::sRotation asserts "
      "IsNormalized()");
  if (normalized != ZJOLT_RESULT_OK) return normalized;

  zjolt::WriteRMat44(out, JPH::RMat44::sRotationTranslation(
                              jolt_rotation, zjolt::ToJoltR(*translation)));
  return ZJOLT_RESULT_OK;
}

void zjoltRMat44Multiply(const ZJoltRMat44 *a, const ZJoltRMat44 *b,
                          ZJoltRMat44 *out) {
  if (a == nullptr || b == nullptr) return;
  zjolt::WriteRMat44(out, zjolt::ToJoltR(*a) * zjolt::ToJoltR(*b));
}

void zjoltRMat44Inverse(const ZJoltRMat44 *m, ZJoltRMat44 *out) {
  if (m == nullptr) return;
  zjolt::WriteRMat44(out, zjolt::ToJoltR(*m).Inversed());
}

void zjoltRMat44InverseRotationTranslation(const ZJoltRMat44 *m,
                                            ZJoltRMat44 *out) {
  if (m == nullptr) return;
  zjolt::WriteRMat44(out, zjolt::ToJoltR(*m).InversedRotationTranslation());
}

void zjoltRMat44TransformPoint(const ZJoltRMat44 *m, const ZJoltRVec3 *point,
                                ZJoltRVec3 *out) {
  if (m == nullptr || point == nullptr) return;
  zjolt::WriteRVec3(out, zjolt::ToJoltR(*m) * zjolt::ToJoltR(*point));
}

void zjoltRMat44TransformDirection(const ZJoltRMat44 *m,
                                    const ZJoltVec3 *direction,
                                    ZJoltVec3 *out) {
  if (m == nullptr || direction == nullptr) return;
  zjolt::WriteVec3(out,
                   zjolt::ToJoltR(*m).Multiply3x3(zjolt::ToJolt(*direction)));
}
