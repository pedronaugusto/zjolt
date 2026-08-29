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

#include <Jolt/Geometry/AABox.h>
#include <Jolt/Geometry/ClosestPoint.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Geometry/RayCapsule.h>
#include <Jolt/Geometry/RayCylinder.h>
#include <Jolt/Geometry/RaySphere.h>
#include <Jolt/Geometry/RayTriangle.h>
#include <Jolt/Math/DVec3.h>
#include <Jolt/Math/FindRoot.h>
#include <Jolt/Math/GaussianElimination.h>
#include <Jolt/Math/HalfFloat.h>
#include <Jolt/Math/Math.h>
#include <Jolt/Math/Matrix.h>
#include <Jolt/Math/Trigonometry.h>
#include <Jolt/Physics/Body/MassProperties.h>

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

/// `min`/`max` straight across: zjolt_shape.cpp's zjoltShapeGetLocalBounds
/// crosses a JPH::AABox the same way in the other direction.
JPH::AABox ToJoltAABox(const ZJoltAABox &box) {
  return JPH::AABox(zjolt::ToJolt(box.min), zjolt::ToJolt(box.max));
}

/// The reverse of zjoltShapeGetMassProperties's row-major unpacking
/// (zjolt_shape.cpp): row r, column c of `mp.inertia` becomes Mat44 element
/// (r, c). Duplicated from zjolt_body.cpp's own local ToJoltMassProperties
/// rather than shared, since that one has internal linkage there too — see
/// this file's own note on the layout, not that one's.
JPH::MassProperties ToJoltMassProperties(const ZJoltMassProperties &mp) {
  JPH::MassProperties out;
  out.mMass = mp.mass;
  out.mInertia = JPH::Mat44::sZero();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out.mInertia(row, col) = mp.inertia[row * 3 + col];
    }
  }
  out.mInertia(3, 3) = 1.0f;
  return out;
}

JPH::Vector<2> Vector2ToJolt(const ZJoltVector2 &v) {
  JPH::Vector<2> out;
  out[0] = v.x;
  out[1] = v.y;
  return out;
}

ZJoltVector2 JoltToVector2(const JPH::Vector<2> &v) {
  return ZJoltVector2{v[0], v[1]};
}

void WriteVector2(ZJoltVector2 *out, const JPH::Vector<2> &v) {
  if (out != nullptr) *out = JoltToVector2(v);
}

JPH::Vector<3> Vec3ToJoltVector3(const ZJoltVec3 &v) {
  JPH::Vector<3> out;
  out[0] = v.x;
  out[1] = v.y;
  out[2] = v.z;
  return out;
}

ZJoltVec3 JoltVector3ToVec3(const JPH::Vector<3> &v) {
  ZJoltVec3 out;
  out.x = v[0];
  out.y = v[1];
  out.z = v[2];
  return out;
}

void WriteVec3FromVector3(ZJoltVec3 *out, const JPH::Vector<3> &v) {
  if (out != nullptr) *out = JoltVector3ToVec3(v);
}

/// Column-major unpack/pack: ZJoltMatrix2::m[2*c+r] <-> Matrix<2,2>(r, c).
JPH::Matrix<2, 2> Matrix2ToJolt(const ZJoltMatrix2 &m) {
  JPH::Matrix<2, 2> out;
  for (uint32_t c = 0; c < 2; ++c)
    for (uint32_t r = 0; r < 2; ++r) out(r, c) = m.m[2 * c + r];
  return out;
}

ZJoltMatrix2 JoltToMatrix2(const JPH::Matrix<2, 2> &m) {
  ZJoltMatrix2 out;
  for (uint32_t c = 0; c < 2; ++c)
    for (uint32_t r = 0; r < 2; ++r) out.m[2 * c + r] = m(r, c);
  return out;
}

void WriteMatrix2(ZJoltMatrix2 *out, const JPH::Matrix<2, 2> &m) {
  if (out != nullptr) *out = JoltToMatrix2(m);
}

/// As Matrix2ToJolt/JoltToMatrix2, over the 3x3 instantiation.
JPH::Matrix<3, 3> Matrix3ToJolt(const ZJoltMatrix3 &m) {
  JPH::Matrix<3, 3> out;
  for (uint32_t c = 0; c < 3; ++c)
    for (uint32_t r = 0; r < 3; ++r) out(r, c) = m.m[3 * c + r];
  return out;
}

ZJoltMatrix3 JoltToMatrix3(const JPH::Matrix<3, 3> &m) {
  ZJoltMatrix3 out;
  for (uint32_t c = 0; c < 3; ++c)
    for (uint32_t r = 0; r < 3; ++r) out.m[3 * c + r] = m(r, c);
  return out;
}

void WriteMatrix3(ZJoltMatrix3 *out, const JPH::Matrix<3, 3> &m) {
  if (out != nullptr) *out = JoltToMatrix3(m);
}

/// The `UVec4` layout `HalfFloatConversion::ToFloat(Fallback)` expects: 4
/// packed uint16 half floats in lanes 0-1 (2 per lane), lanes 2-3 unused —
/// see `Expand4Uint16Lo`'s bit-shift math (UVec4.inl).
JPH::UVec4 PackHalfFloats(const ZJoltHalfFloat *in) {
  const uint32_t lane0 =
      static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 16);
  const uint32_t lane1 =
      static_cast<uint32_t>(in[2]) | (static_cast<uint32_t>(in[3]) << 16);
  return JPH::UVec4(lane0, lane1, 0, 0);
}

void UnpackFloats(const JPH::Vec4 &v, float *out) {
  out[0] = v.GetX();
  out[1] = v.GetY();
  out[2] = v.GetZ();
  out[3] = v.GetW();
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

//===----------------------------------------------------------------------===//
// Decomposition
//===----------------------------------------------------------------------===//

void zjoltMat44Decompose(const ZJoltMat44 *m,
                          ZJoltMat44 *out_rotation_translation,
                          ZJoltVec3 *out_scale) {
  if (m == nullptr) return;
  JPH::Vec3 scale;
  const JPH::Mat44 rotation_translation = zjolt::ToJolt(*m).Decompose(scale);
  zjolt::WriteMat44(out_rotation_translation, rotation_translation);
  zjolt::WriteVec3(out_scale, scale);
}

void zjoltRMat44Decompose(const ZJoltRMat44 *m,
                           ZJoltRMat44 *out_rotation_translation,
                           ZJoltVec3 *out_scale) {
  if (m == nullptr) return;
  JPH::Vec3 scale;
  const JPH::RMat44 rotation_translation = zjolt::ToJoltR(*m).Decompose(scale);
  zjolt::WriteRMat44(out_rotation_translation, rotation_translation);
  zjolt::WriteVec3(out_scale, scale);
}

void zjoltMat44GetQuaternion(const ZJoltMat44 *m, ZJoltQuat *out) {
  if (m == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(*m).GetQuaternion());
}

void zjoltRMat44GetQuaternion(const ZJoltRMat44 *m, ZJoltQuat *out) {
  if (m == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJoltR(*m).GetQuaternion());
}

//===----------------------------------------------------------------------===//
// Directed rounding
//===----------------------------------------------------------------------===//

void zjoltRVec3ToVec3RoundDown(const ZJoltRVec3 *v, ZJoltVec3 *out) {
  if (v == nullptr) return;
  const JPH::DVec3 widened(static_cast<double>(v->x), static_cast<double>(v->y),
                           static_cast<double>(v->z));
  zjolt::WriteVec3(out, widened.ToVec3RoundDown());
}

void zjoltRVec3ToVec3RoundUp(const ZJoltRVec3 *v, ZJoltVec3 *out) {
  if (v == nullptr) return;
  const JPH::DVec3 widened(static_cast<double>(v->x), static_cast<double>(v->y),
                           static_cast<double>(v->z));
  zjolt::WriteVec3(out, widened.ToVec3RoundUp());
}

//===----------------------------------------------------------------------===//
// Mass properties
//===----------------------------------------------------------------------===//

ZJoltResult zjoltMassPropertiesDecomposePrincipalMomentsOfInertia(
    const ZJoltMassProperties *properties, ZJoltMat44 *out_rotation,
    ZJoltVec3 *out_diagonal) {
  ZJOLT_ENTER(out_rotation, out_diagonal);
  if (!zjolt::Present(properties, out_rotation, out_diagonal))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::MassProperties jolt_properties = ToJoltMassProperties(*properties);
  JPH::Mat44 rotation;
  JPH::Vec3 diagonal;
  if (!jolt_properties.DecomposePrincipalMomentsOfInertia(rotation, diagonal)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "properties.inertia did not converge under Jolt's Jacobi-sweep "
        "eigensolver (50-sweep budget); the tensor is not symmetric "
        "positive-semi-definite");
  }

  zjolt::WriteMat44(out_rotation, rotation);
  zjolt::WriteVec3(out_diagonal, diagonal);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Barycentric coordinates and closest points
//===----------------------------------------------------------------------===//

bool zjoltGetBaryCentricCoordinatesLine(const ZJoltVec3 *a, const ZJoltVec3 *b,
                                        float *out_u, float *out_v) {
  if (a == nullptr || b == nullptr || out_u == nullptr || out_v == nullptr)
    return false;
  return JPH::ClosestPoint::GetBaryCentricCoordinates(
      zjolt::ToJolt(*a), zjolt::ToJolt(*b), *out_u, *out_v);
}

bool zjoltGetBaryCentricCoordinatesTriangle(const ZJoltVec3 *a,
                                            const ZJoltVec3 *b,
                                            const ZJoltVec3 *c, float *out_u,
                                            float *out_v, float *out_w) {
  if (a == nullptr || b == nullptr || c == nullptr || out_u == nullptr ||
      out_v == nullptr || out_w == nullptr)
    return false;
  return JPH::ClosestPoint::GetBaryCentricCoordinates(
      zjolt::ToJolt(*a), zjolt::ToJolt(*b), zjolt::ToJolt(*c), *out_u, *out_v,
      *out_w);
}

void zjoltGetClosestPointOnLine(const ZJoltVec3 *a, const ZJoltVec3 *b,
                                 ZJoltVec3 *out_point, uint32_t *out_set) {
  if (a == nullptr || b == nullptr) return;
  JPH::uint32 set = 0;
  const JPH::Vec3 point = JPH::ClosestPoint::GetClosestPointOnLine(
      zjolt::ToJolt(*a), zjolt::ToJolt(*b), set);
  zjolt::WriteVec3(out_point, point);
  if (out_set != nullptr) *out_set = set;
}

void zjoltGetClosestPointOnTriangle(const ZJoltVec3 *a, const ZJoltVec3 *b,
                                    const ZJoltVec3 *c, ZJoltVec3 *out_point,
                                    uint32_t *out_set) {
  if (a == nullptr || b == nullptr || c == nullptr) return;
  JPH::uint32 set = 0;
  const JPH::Vec3 point = JPH::ClosestPoint::GetClosestPointOnTriangle(
      zjolt::ToJolt(*a), zjolt::ToJolt(*b), zjolt::ToJolt(*c), set);
  zjolt::WriteVec3(out_point, point);
  if (out_set != nullptr) *out_set = set;
}

void zjoltGetClosestPointOnTetrahedron(const ZJoltVec3 *a, const ZJoltVec3 *b,
                                       const ZJoltVec3 *c, const ZJoltVec3 *d,
                                       ZJoltVec3 *out_point,
                                       uint32_t *out_set) {
  if (a == nullptr || b == nullptr || c == nullptr || d == nullptr) return;
  JPH::uint32 set = 0;
  const JPH::Vec3 point = JPH::ClosestPoint::GetClosestPointOnTetrahedron(
      zjolt::ToJolt(*a), zjolt::ToJolt(*b), zjolt::ToJolt(*c),
      zjolt::ToJolt(*d), set);
  zjolt::WriteVec3(out_point, point);
  if (out_set != nullptr) *out_set = set;
}

//===----------------------------------------------------------------------===//
// Ray intersection primitives
//===----------------------------------------------------------------------===//

float zjoltRayTriangle(const ZJoltVec3 *origin, const ZJoltVec3 *direction,
                        const ZJoltVec3 *v0, const ZJoltVec3 *v1,
                        const ZJoltVec3 *v2) {
  if (origin == nullptr || direction == nullptr || v0 == nullptr ||
      v1 == nullptr || v2 == nullptr)
    return FLT_MAX;
  return JPH::RayTriangle(zjolt::ToJolt(*origin), zjolt::ToJolt(*direction),
                          zjolt::ToJolt(*v0), zjolt::ToJolt(*v1),
                          zjolt::ToJolt(*v2));
}

float zjoltRaySphere(const ZJoltVec3 *origin, const ZJoltVec3 *direction,
                     const ZJoltVec3 *center, float radius) {
  if (origin == nullptr || direction == nullptr || center == nullptr)
    return FLT_MAX;
  return JPH::RaySphere(zjolt::ToJolt(*origin), zjolt::ToJolt(*direction),
                        zjolt::ToJolt(*center), radius);
}

uint32_t zjoltRaySphereMinMax(const ZJoltVec3 *origin,
                              const ZJoltVec3 *direction,
                              const ZJoltVec3 *center, float radius,
                              float *out_min_fraction,
                              float *out_max_fraction) {
  if (origin == nullptr || direction == nullptr || center == nullptr)
    return 0;
  float min_fraction = 0.0f;
  float max_fraction = 0.0f;
  const int count =
      JPH::RaySphere(zjolt::ToJolt(*origin), zjolt::ToJolt(*direction),
                     zjolt::ToJolt(*center), radius, min_fraction,
                     max_fraction);
  if (count > 0) {
    if (out_min_fraction != nullptr) *out_min_fraction = min_fraction;
    if (out_max_fraction != nullptr) *out_max_fraction = max_fraction;
  }
  return static_cast<uint32_t>(count);
}

float zjoltRayCylinder(const ZJoltVec3 *origin, const ZJoltVec3 *direction,
                       float half_height, float radius) {
  if (origin == nullptr || direction == nullptr) return FLT_MAX;
  return JPH::RayCylinder(zjolt::ToJolt(*origin), zjolt::ToJolt(*direction),
                          half_height, radius);
}

float zjoltRayCapsule(const ZJoltVec3 *origin, const ZJoltVec3 *direction,
                      float half_height, float radius) {
  if (origin == nullptr || direction == nullptr) return FLT_MAX;
  return JPH::RayCapsule(zjolt::ToJolt(*origin), zjolt::ToJolt(*direction),
                         half_height, radius);
}

float zjoltRayAABox(const ZJoltVec3 *origin, const ZJoltVec3 *direction,
                    const ZJoltAABox *box) {
  if (origin == nullptr || direction == nullptr || box == nullptr)
    return FLT_MAX;
  const JPH::Vec3 jolt_direction = zjolt::ToJolt(*direction);
  const JPH::RayInvDirection inv_direction(jolt_direction);
  const JPH::AABox jolt_box = ToJoltAABox(*box);
  return JPH::RayAABox(zjolt::ToJolt(*origin), inv_direction, jolt_box.mMin,
                       jolt_box.mMax);
}

void zjoltRayAABoxMinMax(const ZJoltVec3 *origin, const ZJoltVec3 *direction,
                         const ZJoltAABox *box, float *out_min,
                         float *out_max) {
  if (origin == nullptr || direction == nullptr || box == nullptr) return;
  const JPH::Vec3 jolt_direction = zjolt::ToJolt(*direction);
  const JPH::RayInvDirection inv_direction(jolt_direction);
  const JPH::AABox jolt_box = ToJoltAABox(*box);
  float min_fraction = 0.0f;
  float max_fraction = 0.0f;
  JPH::RayAABox(zjolt::ToJolt(*origin), inv_direction, jolt_box.mMin,
               jolt_box.mMax, min_fraction, max_fraction);
  if (out_min != nullptr) *out_min = min_fraction;
  if (out_max != nullptr) *out_max = max_fraction;
}

bool zjoltRayAABoxHits(const ZJoltVec3 *origin, const ZJoltVec3 *direction,
                       const ZJoltAABox *box) {
  if (origin == nullptr || direction == nullptr || box == nullptr)
    return false;
  const JPH::AABox jolt_box = ToJoltAABox(*box);
  return JPH::RayAABoxHits(zjolt::ToJolt(*origin), zjolt::ToJolt(*direction),
                           jolt_box.mMin, jolt_box.mMax);
}

//===----------------------------------------------------------------------===//
// Oriented box overlap
//===----------------------------------------------------------------------===//

bool zjoltOrientedBoxOverlapsAABox(const ZJoltMat44 *orientation,
                                   const ZJoltVec3 *half_extents,
                                   const ZJoltAABox *box, float epsilon) {
  if (orientation == nullptr || half_extents == nullptr || box == nullptr)
    return false;
  const JPH::OrientedBox jolt_box(zjolt::ToJolt(*orientation),
                                  zjolt::ToJolt(*half_extents));
  return jolt_box.Overlaps(ToJoltAABox(*box), epsilon);
}

bool zjoltOrientedBoxOverlapsOrientedBox(const ZJoltMat44 *orientation_a,
                                         const ZJoltVec3 *half_extents_a,
                                         const ZJoltMat44 *orientation_b,
                                         const ZJoltVec3 *half_extents_b,
                                         float epsilon) {
  if (orientation_a == nullptr || half_extents_a == nullptr ||
      orientation_b == nullptr || half_extents_b == nullptr)
    return false;
  const JPH::OrientedBox jolt_a(zjolt::ToJolt(*orientation_a),
                                zjolt::ToJolt(*half_extents_a));
  const JPH::OrientedBox jolt_b(zjolt::ToJolt(*orientation_b),
                                zjolt::ToJolt(*half_extents_b));
  return jolt_a.Overlaps(jolt_b, epsilon);
}

//===----------------------------------------------------------------------===//
// Deterministic trigonometry
//===----------------------------------------------------------------------===//

float zjoltSin(float radians) { return JPH::Sin(radians); }

float zjoltCos(float radians) { return JPH::Cos(radians); }

float zjoltTan(float radians) { return JPH::Tan(radians); }

float zjoltASin(float x) { return JPH::ASin(x); }

float zjoltACos(float x) { return JPH::ACos(x); }

float zjoltATan(float x) { return JPH::ATan(x); }

float zjoltATan2(float y, float x) { return JPH::ATan2(y, x); }

void zjoltSinCos(float radians, float *out_sin, float *out_cos) {
  JPH::Vec4 s, c;
  JPH::Vec4::sReplicate(radians).SinCos(s, c);
  if (out_sin != nullptr) *out_sin = s.GetX();
  if (out_cos != nullptr) *out_cos = c.GetX();
}

float zjoltACosApproximate(float x) { return JPH::ACosApproximate(x); }

void zjoltSinCos4(const float *in, float *out_sin, float *out_cos) {
  if (in == nullptr) return;
  JPH::Vec4 s, c;
  JPH::Vec4(in[0], in[1], in[2], in[3]).SinCos(s, c);
  if (out_sin != nullptr) UnpackFloats(s, out_sin);
  if (out_cos != nullptr) UnpackFloats(c, out_cos);
}

void zjoltTan4(const float *in, float *out) {
  if (in == nullptr || out == nullptr) return;
  UnpackFloats(JPH::Vec4(in[0], in[1], in[2], in[3]).Tan(), out);
}

void zjoltASin4(const float *in, float *out) {
  if (in == nullptr || out == nullptr) return;
  UnpackFloats(JPH::Vec4(in[0], in[1], in[2], in[3]).ASin(), out);
}

void zjoltACos4(const float *in, float *out) {
  if (in == nullptr || out == nullptr) return;
  UnpackFloats(JPH::Vec4(in[0], in[1], in[2], in[3]).ACos(), out);
}

void zjoltATan4(const float *in, float *out) {
  if (in == nullptr || out == nullptr) return;
  UnpackFloats(JPH::Vec4(in[0], in[1], in[2], in[3]).ATan(), out);
}

void zjoltATan24(const float *in_y, const float *in_x, float *out) {
  if (in_y == nullptr || in_x == nullptr || out == nullptr) return;
  const JPH::Vec4 y(in_y[0], in_y[1], in_y[2], in_y[3]);
  const JPH::Vec4 x(in_x[0], in_x[1], in_x[2], in_x[3]);
  UnpackFloats(JPH::Vec4::sATan2(y, x), out);
}

//===----------------------------------------------------------------------===//
// Dense linear algebra
//===----------------------------------------------------------------------===//

void zjoltMatrix2Zero(ZJoltMatrix2 *out) {
  WriteMatrix2(out, JPH::Matrix<2, 2>::sZero());
}

void zjoltMatrix2Identity(ZJoltMatrix2 *out) {
  WriteMatrix2(out, JPH::Matrix<2, 2>::sIdentity());
}

void zjoltMatrix2Diagonal(const ZJoltVector2 *v, ZJoltMatrix2 *out) {
  if (v == nullptr) return;
  WriteMatrix2(out, JPH::Matrix<2, 2>::sDiagonal(Vector2ToJolt(*v)));
}

uint32_t zjoltMatrix2GetRows(void) { return JPH::Matrix<2, 2>().GetRows(); }

uint32_t zjoltMatrix2GetCols(void) { return JPH::Matrix<2, 2>().GetCols(); }

bool zjoltMatrix2IsIdentity(const ZJoltMatrix2 *m) {
  if (m == nullptr) return false;
  return Matrix2ToJolt(*m).IsIdentity();
}

ZJoltResult zjoltMatrix2GetColumn(const ZJoltMatrix2 *m, uint32_t column,
                                  ZJoltVector2 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(m, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (column >= 2)
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                            "column must be < 2");
  WriteVector2(out, Matrix2ToJolt(*m).GetColumn(static_cast<int>(column)));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltMatrix2WithColumn(const ZJoltMatrix2 *m, uint32_t column,
                                   const ZJoltVector2 *v, ZJoltMatrix2 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(m, v, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (column >= 2)
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                            "column must be < 2");
  JPH::Matrix<2, 2> jolt_m = Matrix2ToJolt(*m);
  jolt_m.GetColumn(static_cast<int>(column)) = Vector2ToJolt(*v);
  WriteMatrix2(out, jolt_m);
  return ZJOLT_RESULT_OK;
}

void zjoltMatrix2GetDiagonal(const ZJoltMatrix2 *m, ZJoltVector2 *out) {
  if (m == nullptr) return;
  const JPH::Matrix<2, 2> jolt_m = Matrix2ToJolt(*m);
  JPH::Vector<2> diag;
  for (uint32_t i = 0; i < 2; ++i) diag[i] = jolt_m(i, i);
  WriteVector2(out, diag);
}

bool zjoltMatrix2IsClose(const ZJoltMatrix2 *a, const ZJoltMatrix2 *b,
                         float max_dist_sq) {
  if (a == nullptr || b == nullptr) return false;
  const JPH::Matrix<2, 2> jolt_a = Matrix2ToJolt(*a);
  const JPH::Matrix<2, 2> jolt_b = Matrix2ToJolt(*b);
  for (int c = 0; c < 2; ++c)
    if (!jolt_a.GetColumn(c).IsClose(jolt_b.GetColumn(c), max_dist_sq))
      return false;
  return true;
}

void zjoltMatrix2Transposed(const ZJoltMatrix2 *m, ZJoltMatrix2 *out) {
  if (m == nullptr) return;
  WriteMatrix2(out, Matrix2ToJolt(*m).Transposed());
}

ZJoltResult zjoltMatrix2Inversed(const ZJoltMatrix2 *m, ZJoltMatrix2 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(m, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::Matrix<2, 2> inv;
  if (!inv.SetInversed(Matrix2ToJolt(*m))) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "m is singular; Jolt's Matrix<2,2>::SetInversed could not invert it");
  }
  WriteMatrix2(out, inv);
  return ZJOLT_RESULT_OK;
}

void zjoltMatrix2Multiply(const ZJoltMatrix2 *a, const ZJoltMatrix2 *b,
                          ZJoltMatrix2 *out) {
  if (a == nullptr || b == nullptr) return;
  WriteMatrix2(out, Matrix2ToJolt(*a) * Matrix2ToJolt(*b));
}

void zjoltMatrix2MultiplyVector(const ZJoltMatrix2 *m, const ZJoltVector2 *v,
                                ZJoltVector2 *out) {
  if (m == nullptr || v == nullptr) return;
  WriteVector2(out, Matrix2ToJolt(*m) * Vector2ToJolt(*v));
}

void zjoltMatrix2MultiplyScalar(const ZJoltMatrix2 *m, float s,
                                ZJoltMatrix2 *out) {
  if (m == nullptr) return;
  WriteMatrix2(out, Matrix2ToJolt(*m) * s);
}

void zjoltMatrix2Add(const ZJoltMatrix2 *a, const ZJoltMatrix2 *b,
                     ZJoltMatrix2 *out) {
  if (a == nullptr || b == nullptr) return;
  WriteMatrix2(out, Matrix2ToJolt(*a) + Matrix2ToJolt(*b));
}

void zjoltMatrix2Subtract(const ZJoltMatrix2 *a, const ZJoltMatrix2 *b,
                          ZJoltMatrix2 *out) {
  if (a == nullptr || b == nullptr) return;
  WriteMatrix2(out, Matrix2ToJolt(*a) - Matrix2ToJolt(*b));
}

ZJoltResult zjoltMatrix2Solve(const ZJoltMatrix2 *a, const ZJoltVector2 *b,
                              float tolerance, ZJoltVector2 *out_x) {
  ZJOLT_ENTER(out_x);
  if (!zjolt::Present(a, b, out_x)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Matrix<2, 2> jolt_a = Matrix2ToJolt(*a);
  JPH::Matrix<2, 1> jolt_b;
  jolt_b(0, 0) = b->x;
  jolt_b(1, 0) = b->y;

  if (!JPH::GaussianElimination(jolt_a, jolt_b, tolerance)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                            "a is singular within tolerance; Jolt's "
                            "GaussianElimination found no usable pivot");
  }

  out_x->x = jolt_b(0, 0);
  out_x->y = jolt_b(1, 0);
  return ZJOLT_RESULT_OK;
}

void zjoltMatrix3Zero(ZJoltMatrix3 *out) {
  WriteMatrix3(out, JPH::Matrix<3, 3>::sZero());
}

void zjoltMatrix3Identity(ZJoltMatrix3 *out) {
  WriteMatrix3(out, JPH::Matrix<3, 3>::sIdentity());
}

void zjoltMatrix3Diagonal(const ZJoltVec3 *v, ZJoltMatrix3 *out) {
  if (v == nullptr) return;
  WriteMatrix3(out, JPH::Matrix<3, 3>::sDiagonal(Vec3ToJoltVector3(*v)));
}

uint32_t zjoltMatrix3GetRows(void) { return JPH::Matrix<3, 3>().GetRows(); }

uint32_t zjoltMatrix3GetCols(void) { return JPH::Matrix<3, 3>().GetCols(); }

bool zjoltMatrix3IsIdentity(const ZJoltMatrix3 *m) {
  if (m == nullptr) return false;
  return Matrix3ToJolt(*m).IsIdentity();
}

ZJoltResult zjoltMatrix3GetColumn(const ZJoltMatrix3 *m, uint32_t column,
                                  ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(m, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (column >= 3)
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                            "column must be < 3");
  WriteVec3FromVector3(out,
                       Matrix3ToJolt(*m).GetColumn(static_cast<int>(column)));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltMatrix3WithColumn(const ZJoltMatrix3 *m, uint32_t column,
                                   const ZJoltVec3 *v, ZJoltMatrix3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(m, v, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (column >= 3)
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                            "column must be < 3");
  JPH::Matrix<3, 3> jolt_m = Matrix3ToJolt(*m);
  jolt_m.GetColumn(static_cast<int>(column)) = Vec3ToJoltVector3(*v);
  WriteMatrix3(out, jolt_m);
  return ZJOLT_RESULT_OK;
}

void zjoltMatrix3GetDiagonal(const ZJoltMatrix3 *m, ZJoltVec3 *out) {
  if (m == nullptr) return;
  const JPH::Matrix<3, 3> jolt_m = Matrix3ToJolt(*m);
  JPH::Vector<3> diag;
  for (uint32_t i = 0; i < 3; ++i) diag[i] = jolt_m(i, i);
  WriteVec3FromVector3(out, diag);
}

bool zjoltMatrix3IsClose(const ZJoltMatrix3 *a, const ZJoltMatrix3 *b,
                         float max_dist_sq) {
  if (a == nullptr || b == nullptr) return false;
  const JPH::Matrix<3, 3> jolt_a = Matrix3ToJolt(*a);
  const JPH::Matrix<3, 3> jolt_b = Matrix3ToJolt(*b);
  for (int c = 0; c < 3; ++c)
    if (!jolt_a.GetColumn(c).IsClose(jolt_b.GetColumn(c), max_dist_sq))
      return false;
  return true;
}

void zjoltMatrix3Transposed(const ZJoltMatrix3 *m, ZJoltMatrix3 *out) {
  if (m == nullptr) return;
  WriteMatrix3(out, Matrix3ToJolt(*m).Transposed());
}

ZJoltResult zjoltMatrix3Inversed(const ZJoltMatrix3 *m, ZJoltMatrix3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(m, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::Matrix<3, 3> inv;
  if (!inv.SetInversed(Matrix3ToJolt(*m))) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "m is singular; Jolt's Matrix<3,3>::SetInversed (GaussianElimination)"
        " could not invert it");
  }
  WriteMatrix3(out, inv);
  return ZJOLT_RESULT_OK;
}

void zjoltMatrix3Multiply(const ZJoltMatrix3 *a, const ZJoltMatrix3 *b,
                          ZJoltMatrix3 *out) {
  if (a == nullptr || b == nullptr) return;
  WriteMatrix3(out, Matrix3ToJolt(*a) * Matrix3ToJolt(*b));
}

void zjoltMatrix3MultiplyVector(const ZJoltMatrix3 *m, const ZJoltVec3 *v,
                                ZJoltVec3 *out) {
  if (m == nullptr || v == nullptr) return;
  WriteVec3FromVector3(out, Matrix3ToJolt(*m) * Vec3ToJoltVector3(*v));
}

void zjoltMatrix3MultiplyScalar(const ZJoltMatrix3 *m, float s,
                                ZJoltMatrix3 *out) {
  if (m == nullptr) return;
  WriteMatrix3(out, Matrix3ToJolt(*m) * s);
}

void zjoltMatrix3Add(const ZJoltMatrix3 *a, const ZJoltMatrix3 *b,
                     ZJoltMatrix3 *out) {
  if (a == nullptr || b == nullptr) return;
  WriteMatrix3(out, Matrix3ToJolt(*a) + Matrix3ToJolt(*b));
}

void zjoltMatrix3Subtract(const ZJoltMatrix3 *a, const ZJoltMatrix3 *b,
                          ZJoltMatrix3 *out) {
  if (a == nullptr || b == nullptr) return;
  WriteMatrix3(out, Matrix3ToJolt(*a) - Matrix3ToJolt(*b));
}

ZJoltResult zjoltMatrix3Solve(const ZJoltMatrix3 *a, const ZJoltVec3 *b,
                              float tolerance, ZJoltVec3 *out_x) {
  ZJOLT_ENTER(out_x);
  if (!zjolt::Present(a, b, out_x)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Matrix<3, 3> jolt_a = Matrix3ToJolt(*a);
  JPH::Matrix<3, 1> jolt_b;
  jolt_b(0, 0) = b->x;
  jolt_b(1, 0) = b->y;
  jolt_b(2, 0) = b->z;

  if (!JPH::GaussianElimination(jolt_a, jolt_b, tolerance)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                            "a is singular within tolerance; Jolt's "
                            "GaussianElimination found no usable pivot");
  }

  out_x->x = jolt_b(0, 0);
  out_x->y = jolt_b(1, 0);
  out_x->z = jolt_b(2, 0);
  return ZJOLT_RESULT_OK;
}

void zjoltRMat44ToMat44(const ZJoltRMat44 *m, ZJoltMat44 *out) {
  if (m == nullptr) return;
  // JPH::RMat44::ToMat44 exists under both precision modes: DMat44's does
  // the real narrowing, Mat44's is `return *this;`.
  zjolt::WriteMat44(out, zjolt::ToJoltR(*m).ToMat44());
}

//===----------------------------------------------------------------------===//
// Half floats
//===----------------------------------------------------------------------===//

ZJoltHalfFloat zjoltHalfFloatFromFloat(float value,
                                       ZJoltHalfFloatRoundingMode mode) {
  switch (mode) {
    case ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_NEG_INF:
      return JPH::HalfFloatConversion::FromFloat<
          JPH::HalfFloatConversion::ROUND_TO_NEG_INF>(value);
    case ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_POS_INF:
      return JPH::HalfFloatConversion::FromFloat<
          JPH::HalfFloatConversion::ROUND_TO_POS_INF>(value);
    case ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_NEAREST:
    default:
      return JPH::HalfFloatConversion::FromFloat<
          JPH::HalfFloatConversion::ROUND_TO_NEAREST>(value);
  }
}

ZJoltHalfFloat zjoltHalfFloatFromFloatFallback(
    float value, ZJoltHalfFloatRoundingMode mode) {
  switch (mode) {
    case ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_NEG_INF:
      return JPH::HalfFloatConversion::FromFloatFallback<
          JPH::HalfFloatConversion::ROUND_TO_NEG_INF>(value);
    case ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_POS_INF:
      return JPH::HalfFloatConversion::FromFloatFallback<
          JPH::HalfFloatConversion::ROUND_TO_POS_INF>(value);
    case ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_NEAREST:
    default:
      return JPH::HalfFloatConversion::FromFloatFallback<
          JPH::HalfFloatConversion::ROUND_TO_NEAREST>(value);
  }
}

void zjoltHalfFloatToFloat4(const ZJoltHalfFloat *in, float *out) {
  if (in == nullptr || out == nullptr) return;
  UnpackFloats(JPH::HalfFloatConversion::ToFloat(PackHalfFloats(in)), out);
}

void zjoltHalfFloatToFloatFallback4(const ZJoltHalfFloat *in, float *out) {
  if (in == nullptr || out == nullptr) return;
  UnpackFloats(JPH::HalfFloatConversion::ToFloatFallback(PackHalfFloats(in)),
              out);
}

//===----------------------------------------------------------------------===//
// Root finding and angle wrapping
//===----------------------------------------------------------------------===//

uint32_t zjoltFindRoot(float a, float b, float c, float *out_x1,
                       float *out_x2) {
  float x1 = 0.0f, x2 = 0.0f;
  const int count = JPH::FindRoot<float>(a, b, c, x1, x2);
  if (count > 0) {
    if (out_x1 != nullptr) *out_x1 = x1;
    if (out_x2 != nullptr) *out_x2 = x2;
  }
  return static_cast<uint32_t>(count);
}

float zjoltCenterAngleAroundZero(float radians) {
  return JPH::CenterAngleAroundZero(radians);
}
