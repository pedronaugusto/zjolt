//===----------------------------------------------------------------------===//
// zjolt — quaternion and matrix algebra.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_MATH_H_
#define ZJOLT_MATH_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Quaternions: ZJoltQuat is (x, y, z, w) — w LAST, matching Jolt.
//
// A function Jolt itself guards with JPH_ASSERT(IsNormalized()) checks the
// same condition here and reports ZJOLT_RESULT_INVALID_ARGUMENT instead of
// asserting. A function with no such assert is INFALLIBLE and computes
// whatever the arithmetic gives for nonsense input — same as Jolt's C++.
//===----------------------------------------------------------------------===//

/// Composes two rotations: `out = lhs * rhs`. Hamilton product order —
/// rotating by `out` matches rotating by `rhs` FIRST, then `lhs`; get this
/// backwards and only a SECOND composed rotation exposes it. Infallible:
/// no normalisation requirement, as Jolt's own `operator*`.
ZJOLT_API void zjoltQuatMultiply(const ZJoltQuat *lhs, const ZJoltQuat *rhs,
                                  ZJoltQuat *out);

/// Rotates `v` by `q`. ZJOLT_RESULT_INVALID_ARGUMENT when `q` is not unit
/// length (checked at the same 1e-5 tolerance as zjoltQuatIsNormalized).
/// Renormalise with zjoltQuatNormalize first if `q` merely drifted from
/// integrating a rotation over many frames.
ZJOLT_API ZJoltResult zjoltQuatRotateVector(const ZJoltQuat *q,
                                             const ZJoltVec3 *v,
                                             ZJoltVec3 *out);

/// The inverse rotation: `zjoltQuatMultiply(q, out, ..)` and `(out, q, ..)`
/// both give the identity for a unit `q`. Infallible: `Conjugated() /
/// Length()`; a zero-length `q` produces NaN, as Jolt's own `Inversed()`.
ZJOLT_API void zjoltQuatInverse(const ZJoltQuat *q, ZJoltQuat *out);

/// The conjugate `(-x, -y, -z, w)`. Equal to the inverse for a unit
/// quaternion, and cheaper to compute: no length, no division. Infallible.
ZJOLT_API void zjoltQuatConjugate(const ZJoltQuat *q, ZJoltQuat *out);

/// Four-component dot product. For two unit quaternions this is the cosine
/// of half the angle between the rotations — 1.0 for equal, -1.0 for `q`
/// against `-q` (same rotation, opposite point on the SLERP space; see
/// zjoltQuatSlerp). 0.0, not a refusal, for a null pointer.
ZJOLT_API float zjoltQuatDot(const ZJoltQuat *a, const ZJoltQuat *b);

/// Whether `q`'s SQUARED length is within `tolerance` of 1 (stricter than a
/// length-based tolerance). Pass 1e-5 to match Jolt's own
/// JPH_ASSERT(IsNormalized()) default, including the checks
/// zjoltQuatRotateVector and zjoltQuatGetAxisAngle run. False, not a
/// refusal, for a null `q`.
ZJOLT_API bool zjoltQuatIsNormalized(const ZJoltQuat *q, float tolerance);

/// `q` rescaled to unit length. Infallible: a zero/NaN `q` produces NaN, as
/// Jolt's own `Normalized()` — no identity fallback (unlike this library's
/// internal per-step renormalisation). Check zjoltQuatIsNormalized, or the
/// result for NaN, first if a hard failure is not acceptable.
ZJOLT_API void zjoltQuatNormalize(const ZJoltQuat *q, ZJoltQuat *out);

/// A right-handed rotation of `radians` about `axis`.
/// ZJOLT_RESULT_INVALID_ARGUMENT when `axis` is not unit length, checked at
/// Vec3's default 1e-6 tolerance (tighter than the quaternion functions'
/// 1e-5, since this checks a three- not four-component vector).
ZJOLT_API ZJoltResult zjoltQuatFromAxisAngle(const ZJoltVec3 *axis,
                                              float radians, ZJoltQuat *out);

/// The axis/angle zjoltQuatFromAxisAngle would need to reconstruct `q`.
/// `*out_angle` is always in [0, pi] — a rotation past pi is reported as
/// the shorter rotation around the opposite axis, never a reflex angle.
/// ZJOLT_RESULT_INVALID_ARGUMENT when `q` is not unit length.
ZJOLT_API ZJoltResult zjoltQuatGetAxisAngle(const ZJoltQuat *q,
                                             ZJoltVec3 *out_axis,
                                             float *out_angle);

/// The rotation taking the DIRECTION of `from` to the DIRECTION of `to`,
/// along the shorter arc. Infallible: a zero-length `from`/`to` gives the
/// identity; opposite directions give SOME 180-degree rotation about a
/// perpendicular axis (not necessarily the one zjoltQuatFromAxisAngle would
/// build). Neither input need be unit length.
ZJOLT_API void zjoltQuatFromTo(const ZJoltVec3 *from, const ZJoltVec3 *to,
                                ZJoltQuat *out);

/// A rotation from Euler angles `(x, y, z)` radians. Jolt's fixed axis
/// order is `RotZ * RotY * RotX` (X first, then Y, then Z) — one convention
/// among several (Unity/Unreal differ), so check the source's order before
/// reusing a triple from elsewhere. Infallible.
ZJOLT_API void zjoltQuatFromEulerAngles(const ZJoltVec3 *angles_radians,
                                         ZJoltQuat *out);

/// The Euler angles (X-then-Y-then-Z order) to reconstruct `q`. Not a true
/// inverse near gimbal lock (Y at +-pi/2): the middle angle clamps and the
/// X/Z split becomes arbitrary, as for any Euler representation. Never
/// aborts, but also never checks `q` is unit length — a non-unit `q` gives
/// a plausible but meaningless triple.
ZJOLT_API void zjoltQuatGetEulerAngles(const ZJoltQuat *q, ZJoltVec3 *out);

/// A quaternion perpendicular to `q` in the four-dimensional sense Jolt's own
/// swing-twist machinery uses internally — NOT "perpendicular" in the sense
/// of a 3-D rotation axis. Infallible.
ZJOLT_API void zjoltQuatGetPerpendicular(const ZJoltQuat *q, ZJoltQuat *out);

/// The signed angle `q` rotates around `axis` (Swing-Twist decomposition,
/// as zjoltQuatGetTwist). `axis` MUST be unit length — unlike
/// zjoltQuatFromAxisAngle, NOT checked or normalised here (nor by Jolt's
/// own `GetRotationAngle`); a non-unit axis silently skews the result. 0.0,
/// not a refusal, for a null pointer.
ZJOLT_API float zjoltQuatGetRotationAngle(const ZJoltQuat *q,
                                           const ZJoltVec3 *axis);

/// The component of `q` that rotates ONLY around `axis`. `axis` MUST be
/// unit length (same caveat as zjoltQuatGetRotationAngle — unchecked).
/// Infallible: the identity comes back if `q` has no component around
/// `axis`, not a divide-by-zero NaN.
ZJOLT_API void zjoltQuatGetTwist(const ZJoltQuat *q, const ZJoltVec3 *axis,
                                  ZJoltQuat *out);

/// Splits `q` into `q = swing * twist` (recompose with
/// `zjoltQuatMultiply(out_swing, out_twist, &q)`). `twist` rotates ONLY
/// around `q`'s own LOCAL X AXIS, `swing` around local Y/Z — the split
/// zjoltConstraintCreateSwingTwist's limits use; agrees with
/// zjoltQuatGetTwist only when its axis is local X. Infallible, including
/// a 180-degree Y/Z rotation (identity twist, all rotation in swing).
ZJOLT_API void zjoltQuatGetSwingTwist(const ZJoltQuat *q, ZJoltQuat *out_swing,
                                       ZJoltQuat *out_twist);

/// Componentwise `(1 - t) * a + t * b`. NOT renormalised, NOT shortest-path
/// — extrapolates outside [0, 1], and even inside it is not unit length
/// unless `a`/`b` already are and nearly equal. Prefer zjoltQuatSlerp; this
/// is only cheaper/close-to-correct when `a` and `b` are already close.
ZJOLT_API void zjoltQuatLerp(const ZJoltQuat *a, const ZJoltQuat *b, float t,
                              ZJoltQuat *out);

/// Spherical interpolation from `a` to `b`, taking the SHORTER of the two
/// arcs around the rotation sphere — `a` and `-a` are the same rotation but
/// sweep different arcs to `b` unless corrected, done here by negating one
/// side when their dot product is negative. Skip this and an interpolated
/// body spins the LONG way round once per revolution, visible only in
/// motion. `t` outside [0, 1] extrapolates; neither input is renormalised.
ZJOLT_API void zjoltQuatSlerp(const ZJoltQuat *a, const ZJoltQuat *b, float t,
                               ZJoltQuat *out);

//===----------------------------------------------------------------------===//
// Vector interpolation: componentwise `a + (b - a) * t` — no "spherical"
// version exists for a plain linear quantity; bound here since a
// fixed-timestep loop needs it every frame alongside zjoltQuatSlerp.
//===----------------------------------------------------------------------===//

/// `(1 - t) * a + t * b`, componentwise. `t` outside [0, 1] extrapolates.
ZJOLT_API void zjoltVec3Lerp(const ZJoltVec3 *a, const ZJoltVec3 *b, float t,
                              ZJoltVec3 *out);

/// As zjoltVec3Lerp, over ZJoltRVec3 — the type a WORLD-SPACE position
/// actually has (`double` under -Ddouble_precision). Interpolating a
/// position with zjoltVec3Lerp instead narrows to float first, losing
/// exactly the precision double-precision mode exists to keep.
ZJOLT_API void zjoltRVec3Lerp(const ZJoltRVec3 *a, const ZJoltRVec3 *b,
                               float t, ZJoltRVec3 *out);

//===----------------------------------------------------------------------===//
// ZJoltMat44 — a plain 4x4, float even under -Ddouble_precision (what
// zjoltBodyGetInverseInertia and soft-body skinning matrices carry, never a
// world position). Column-major: column c's row r is `m[4 * c + r]`.
//===----------------------------------------------------------------------===//

/// Builds a transform: `rotation` in the upper 3x3, `translation` in the
/// fourth column. ZJOLT_RESULT_INVALID_ARGUMENT when `rotation` is not unit
/// length (same tolerance as zjoltQuatRotateVector).
ZJOLT_API ZJoltResult zjoltMat44FromRotationTranslation(
    const ZJoltQuat *rotation, const ZJoltVec3 *translation, ZJoltMat44 *out);

/// Composes two transforms: `out = a * b` — "apply b first, then a", as
/// zjoltQuatMultiply. Infallible.
ZJOLT_API void zjoltMat44Multiply(const ZJoltMat44 *a, const ZJoltMat44 *b,
                                   ZJoltMat44 *out);

/// The general inverse: `out` composed with `m` (either order) is the
/// identity for any invertible `m`, including scale/shear. Infallible but
/// unchecked — a singular `m` divides by zero, giving NaN/Inf, as Jolt's
/// own `Inversed()`. Prefer zjoltMat44InverseRotationTranslation when `m`
/// carries no scale/shear: cheaper and exact rather than merely accurate.
ZJOLT_API void zjoltMat44Inverse(const ZJoltMat44 *m, ZJoltMat44 *out);

/// The inverse of `m`, ASSUMING it is a rigid transform (orthonormal upper
/// 3x3, translation in the fourth column) — a transpose and dot product,
/// cheaper and exact where zjoltMat44Inverse merely converges. Holds for
/// every transform this ABI hands OUT, but not one carrying scale —
/// unchecked, so a scaled `m` silently returns the wrong answer.
/// Infallible.
ZJOLT_API void zjoltMat44InverseRotationTranslation(const ZJoltMat44 *m,
                                                      ZJoltMat44 *out);

/// Transforms `point` as a POSITION: the fourth column (translation) is
/// added. Infallible.
ZJOLT_API void zjoltMat44TransformPoint(const ZJoltMat44 *m,
                                         const ZJoltVec3 *point,
                                         ZJoltVec3 *out);

/// Transforms `direction` as a DIRECTION: only the upper 3x3 applies, the
/// fourth column (translation) is ignored — unlike
/// zjoltMat44TransformPoint, whose mistaken use on a normal or velocity
/// picks up a spurious position offset. Infallible.
ZJOLT_API void zjoltMat44TransformDirection(const ZJoltMat44 *m,
                                             const ZJoltVec3 *direction,
                                             ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// ZJoltRMat44 — the WORLD-SPACE transform type (ZJoltMat44 with ZJoltReal
// elements, `double` under -Ddouble_precision); zjoltBodyGetWorldTransform
// and zjoltBodyGetCenterOfMassTransform return this, not ZJoltMat44, to
// avoid narrowing every position through float. Same five operations below.
//===----------------------------------------------------------------------===//

/// As zjoltMat44FromRotationTranslation, `translation` widened to ZJoltReal.
/// Same refusal, for the same reason: `rotation` must be unit length.
ZJOLT_API ZJoltResult zjoltRMat44FromRotationTranslation(
    const ZJoltQuat *rotation, const ZJoltRVec3 *translation,
    ZJoltRMat44 *out);

/// As zjoltMat44Multiply, over ZJoltRMat44. Infallible.
ZJOLT_API void zjoltRMat44Multiply(const ZJoltRMat44 *a, const ZJoltRMat44 *b,
                                    ZJoltRMat44 *out);

/// As zjoltMat44Inverse, over ZJoltRMat44. Infallible; singular in, NaN out.
ZJOLT_API void zjoltRMat44Inverse(const ZJoltRMat44 *m, ZJoltRMat44 *out);

/// As zjoltMat44InverseRotationTranslation, over ZJoltRMat44. Same rigid-
/// transform assumption, unchecked the same way. Infallible.
ZJOLT_API void zjoltRMat44InverseRotationTranslation(const ZJoltRMat44 *m,
                                                       ZJoltRMat44 *out);

/// Transforms `point` — a WORLD-SPACE position, ZJoltReal precision in and
/// out — as a POSITION. Infallible.
ZJOLT_API void zjoltRMat44TransformPoint(const ZJoltRMat44 *m,
                                          const ZJoltRVec3 *point,
                                          ZJoltRVec3 *out);

/// Transforms `direction` as a DIRECTION. `direction` and `out` are
/// ZJoltVec3, not ZJoltRVec3: a direction — a normal, a velocity — never
/// needs the extended range a world position does, in a transform's rotation
/// part no less than anywhere else this ABI draws that line. Infallible.
ZJOLT_API void zjoltRMat44TransformDirection(const ZJoltRMat44 *m,
                                              const ZJoltVec3 *direction,
                                              ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Decomposition: pulling rotation/translation/scale out of a general
// Mat44/RMat44, and a quaternion out of a rotation matrix — the reverse of
// zjoltMat44FromRotationTranslation, for a matrix that may carry scale.
//===----------------------------------------------------------------------===//

/// Splits `m` into `out_scale` and the rigid rotation+translation left over
/// once that scale divides back out (axis columns unit/perpendicular,
/// translation column `m`'s unchanged); multiplying each axis column by
/// the matching `out_scale` component reconstructs `m`. Always a proper
/// rotation, never a mirror — a mirrored `m` pushes the sign into
/// `out_scale`'s Z. Infallible: a singular `m` gives NaN in both outputs.
ZJOLT_API void zjoltMat44Decompose(const ZJoltMat44 *m,
                                    ZJoltMat44 *out_rotation_translation,
                                    ZJoltVec3 *out_scale);

/// As zjoltMat44Decompose, over ZJoltRMat44. `out_rotation_translation`'s
/// translation carries `m`'s ZJoltReal precision through unchanged — only
/// the upper 3x3 goes through scale extraction.
ZJOLT_API void zjoltRMat44Decompose(const ZJoltRMat44 *m,
                                     ZJoltRMat44 *out_rotation_translation,
                                     ZJoltVec3 *out_scale);

/// The rotation `m`'s upper 3x3 represents, as a quaternion. Trace-sign
/// branching keeps this accurate even near a 180-degree rotation.
/// Meaningful only when the upper 3x3 is actually a rotation (orthonormal,
/// no scale/shear) — unchecked; a scaled matrix hands back a
/// plausible-looking but wrong quaternion. Infallible.
ZJOLT_API void zjoltMat44GetQuaternion(const ZJoltMat44 *m, ZJoltQuat *out);

/// As zjoltMat44GetQuaternion, over ZJoltRMat44's rotation part. `m`'s
/// translation is ignored entirely, the same way
/// zjoltRMat44TransformDirection ignores it.
ZJOLT_API void zjoltRMat44GetQuaternion(const ZJoltRMat44 *m, ZJoltQuat *out);

//===----------------------------------------------------------------------===//
// Directed rounding — narrowing a world-space position to float without
// shrinking a bound built from it.
//===----------------------------------------------------------------------===//

/// `v` narrowed to float, rounding every component TOWARD NEGATIVE
/// INFINITY — unlike an ordinary round-to-nearest cast, this never moves a
/// float bound above the true double-precision value. Pair with
/// zjoltRVec3ToVec3RoundUp on a box's opposite corner. An exact copy (no
/// rounding) when ZJoltReal is already float.
ZJOLT_API void zjoltRVec3ToVec3RoundDown(const ZJoltRVec3 *v, ZJoltVec3 *out);

/// As zjoltRVec3ToVec3RoundDown, rounding every component TOWARD POSITIVE
/// INFINITY instead.
ZJOLT_API void zjoltRVec3ToVec3RoundUp(const ZJoltRVec3 *v, ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Mass properties — principal axes of an inertia tensor.
//===----------------------------------------------------------------------===//

/// Eigendecomposes `properties->inertia` into a diagonal D and rotation R
/// (tensor = R*D*R^-1): `out_rotation`'s columns are the principal axes (no
/// cross-coupling torque), `out_diagonal` the moments about them, largest
/// first. ZJOLT_RESULT_INVALID_ARGUMENT if Jolt's 50-sweep Jacobi solver
/// doesn't converge — in practice only from a non-SPD tensor, which one
/// this library computed never is. Detail in zjoltLastError.
ZJOLT_API ZJoltResult zjoltMassPropertiesDecomposePrincipalMomentsOfInertia(
    const ZJoltMassProperties *properties, ZJoltMat44 *out_rotation,
    ZJoltVec3 *out_diagonal);

//===----------------------------------------------------------------------===//
// Barycentric coordinates and closest points — line segments, triangles,
// tetrahedra, relative to the ORIGIN (Jolt's own convention). To test
// against point `p`, subtract it from every vertex first and add it back
// to the result.
//===----------------------------------------------------------------------===//

/// Barycentric weights `(u, v)` of the point on the INFINITE line through
/// `a`/`b` closest to the origin (`u*a + v*b`, `u + v == 1`) — for
/// interpolating a per-vertex attribute at a hit on an edge. False when
/// `a`/`b` coincide (no line to project onto): outputs still come back
/// set, picking whichever is nearer the origin, as Jolt's own fallback.
ZJOLT_API bool zjoltGetBaryCentricCoordinatesLine(const ZJoltVec3 *a,
                                                   const ZJoltVec3 *b,
                                                   float *out_u, float *out_v);

/// As zjoltGetBaryCentricCoordinatesLine, for the plane through triangle
/// `(a, b, c)`: `(u, v, w)`, `u + v + w == 1`.
///
/// False when `a, b, c` are collinear/coincident: fallback coordinates run
/// along the longest edge, as Jolt's own does, still fully set.
ZJOLT_API bool zjoltGetBaryCentricCoordinatesTriangle(
    const ZJoltVec3 *a, const ZJoltVec3 *b, const ZJoltVec3 *c, float *out_u,
    float *out_v, float *out_w);

/// The point on segment `(a, b)` closest to the origin. `*out_set`: bit 0
/// is `a`, bit 1 is `b` — one bit for an endpoint, both (0b11) for a point
/// strictly between. Infallible: `a == b` just reports both bits set.
ZJOLT_API void zjoltGetClosestPointOnLine(const ZJoltVec3 *a,
                                           const ZJoltVec3 *b,
                                           ZJoltVec3 *out_point,
                                           uint32_t *out_set);

/// The point on triangle `(a, b, c)` (interior included) closest to the
/// origin. `*out_set`: bit 0 `a`, bit 1 `b`, bit 2 `c` — one bit a vertex,
/// two an edge, all three the interior. Infallible, including degenerate
/// (collinear/zero-area) triangles: the fallback walks vertices/edges
/// directly rather than dividing by near-zero area.
ZJOLT_API void zjoltGetClosestPointOnTriangle(const ZJoltVec3 *a,
                                               const ZJoltVec3 *b,
                                               const ZJoltVec3 *c,
                                               ZJoltVec3 *out_point,
                                               uint32_t *out_set);

/// The point on tetrahedron `(a, b, c, d)` (interior included) closest to
/// the origin. `*out_set`: one bit per vertex in `a,b,c,d` order — one bit
/// a vertex, two an edge, three a face, four the interior. Infallible.
ZJOLT_API void zjoltGetClosestPointOnTetrahedron(
    const ZJoltVec3 *a, const ZJoltVec3 *b, const ZJoltVec3 *c,
    const ZJoltVec3 *d, ZJoltVec3 *out_point, uint32_t *out_set);

//===----------------------------------------------------------------------===//
// Ray intersection primitives — the exact tests Jolt's shapes run
// internally, usable standalone. Fraction is `origin + fraction *
// direction`; `direction` need not be normalised.
//===----------------------------------------------------------------------===//

/// Intersects a ray with the triangle `(v0, v1, v2)`: the entry fraction, or
/// FLT_MAX for no hit. A hit behind the ray origin (a negative fraction) is
/// reported as no hit, matching Jolt's own triangle shape.
ZJOLT_API float zjoltRayTriangle(const ZJoltVec3 *origin,
                                  const ZJoltVec3 *direction,
                                  const ZJoltVec3 *v0, const ZJoltVec3 *v1,
                                  const ZJoltVec3 *v2);

/// Intersects a ray with a sphere: the entry fraction, or FLT_MAX for no
/// hit. 0.0 when the ray starts inside the sphere.
ZJOLT_API float zjoltRaySphere(const ZJoltVec3 *origin,
                                const ZJoltVec3 *direction,
                                const ZJoltVec3 *center, float radius);

/// As zjoltRaySphere, but reports BOTH crossings: `*out_min_fraction`/
/// `*out_max_fraction` (negative = behind origin; equal if grazing).
/// Returns crossing count (0, 1, 2); out-parameters unwritten for 0.
ZJOLT_API uint32_t zjoltRaySphereMinMax(const ZJoltVec3 *origin,
                                        const ZJoltVec3 *direction,
                                        const ZJoltVec3 *center, float radius,
                                        float *out_min_fraction,
                                        float *out_max_fraction);

/// Intersects a ray with a FINITE cylinder centred at the origin, axis
/// along Y, `half_height` centre-to-cap: the entry fraction, or FLT_MAX.
ZJOLT_API float zjoltRayCylinder(const ZJoltVec3 *origin,
                                  const ZJoltVec3 *direction,
                                  float half_height, float radius);

/// Intersects a ray with a capsule centred at the origin, axis along Y —
/// `half_height` from the centre to the centre of each end cap, `radius` the
/// cap and barrel radius: the entry fraction, or FLT_MAX.
ZJOLT_API float zjoltRayCapsule(const ZJoltVec3 *origin,
                                 const ZJoltVec3 *direction, float half_height,
                                 float radius);

/// Intersects a ray with an axis-aligned box: the entry fraction, or FLT_MAX
/// for no hit. Negative when the ray starts inside `box`.
ZJOLT_API float zjoltRayAABox(const ZJoltVec3 *origin,
                               const ZJoltVec3 *direction,
                               const ZJoltAABox *box);

/// As zjoltRayAABox, but reports both the entry and exit fraction rather
/// than only the entry one. `*out_min`/`*out_max` come back
/// FLT_MAX/-FLT_MAX (not left unwritten) when there is no intersection,
/// matching Jolt's own slab test.
ZJOLT_API void zjoltRayAABoxMinMax(const ZJoltVec3 *origin,
                                    const ZJoltVec3 *direction,
                                    const ZJoltAABox *box, float *out_min,
                                    float *out_max);

/// Whether a ray hits `box` at all, without computing a fraction — the
/// separating-axis form of the same test zjoltRayAABox runs, cheaper when
/// only a yes/no answer is needed.
ZJOLT_API bool zjoltRayAABoxHits(const ZJoltVec3 *origin,
                                  const ZJoltVec3 *direction,
                                  const ZJoltAABox *box);

//===----------------------------------------------------------------------===//
// Oriented box overlap — the one overlap test not reducible to one-line
// arithmetic elsewhere: an arbitrarily-rotated box needs the full
// separating-axis test.
//===----------------------------------------------------------------------===//

/// Whether an oriented box (`orientation`'s upper 3x3 rotates it, fourth
/// column positions its centre, `half_extents` its local half size)
/// overlaps `box`. `epsilon` slackens the test for near-parallel edges;
/// 1e-6 matches Jolt's default.
ZJOLT_API bool zjoltOrientedBoxOverlapsAABox(const ZJoltMat44 *orientation,
                                              const ZJoltVec3 *half_extents,
                                              const ZJoltAABox *box,
                                              float epsilon);

/// As zjoltOrientedBoxOverlapsAABox, between two oriented boxes.
ZJOLT_API bool zjoltOrientedBoxOverlapsOrientedBox(
    const ZJoltMat44 *orientation_a, const ZJoltVec3 *half_extents_a,
    const ZJoltMat44 *orientation_b, const ZJoltVec3 *half_extents_b,
    float epsilon);

//===----------------------------------------------------------------------===//
// Deterministic trigonometry — `Jolt/Math/Trigonometry.h`. Jolt hand-rolls
// these instead of calling libm because libm's sin/cos/etc. are not
// bit-identical across platforms, which a `-Dcross_platform_deterministic`
// build depends on. The four-lane forms are four INDEPENDENT angles packed
// into one SIMD call (`Jolt::Vec4`'s own trig methods), not four lanes of
// one angle.
//===----------------------------------------------------------------------===//

/// Sine of `radians`.
ZJOLT_API float zjoltSin(float radians);

/// Cosine of `radians`.
ZJOLT_API float zjoltCos(float radians);

/// Tangent of `radians`.
ZJOLT_API float zjoltTan(float radians);

/// Arc sine, range `[-pi/2, pi/2]`. `x` outside `[-1, 1]` is clamped first —
/// never NaN, unlike `std::asin`.
ZJOLT_API float zjoltASin(float x);

/// Arc cosine, range `[0, pi]`. `x` outside `[-1, 1]` is clamped first —
/// never NaN, unlike `std::acos`.
ZJOLT_API float zjoltACos(float x);

/// Arc tangent, range `[-pi/2, pi/2]`.
ZJOLT_API float zjoltATan(float x);

/// Arc tangent of `y / x`, using both signs to pick the quadrant. Range
/// `[-pi, pi]`. `(0, 0)` is NaN, not a refusal: Jolt's own `sATan2` divides
/// 0/0 internally in that case rather than special-casing it.
ZJOLT_API float zjoltATan2(float y, float x);

/// `*out_sin`/`*out_cos` together, cheaper than calling zjoltSin and
/// zjoltCos separately — both come from the same underlying computation.
ZJOLT_API void zjoltSinCos(float radians, float *out_sin, float *out_cos);

/// ACos, approximated: max error 4.2e-3 over `[-1, 1]`, about 2.5x faster.
ZJOLT_API float zjoltACosApproximate(float x);

/// zjoltSinCos over 4 independent angles at once. `in`, `out_sin`, `out_cos`
/// each point to exactly 4 elements.
ZJOLT_API void zjoltSinCos4(const float *in, float *out_sin, float *out_cos);

/// zjoltTan over 4 independent angles at once. `in`/`out` each point to
/// exactly 4 elements.
ZJOLT_API void zjoltTan4(const float *in, float *out);

/// zjoltASin over 4 independent values at once. `in`/`out` each point to
/// exactly 4 elements.
ZJOLT_API void zjoltASin4(const float *in, float *out);

/// zjoltACos over 4 independent values at once. `in`/`out` each point to
/// exactly 4 elements.
ZJOLT_API void zjoltACos4(const float *in, float *out);

/// zjoltATan over 4 independent values at once. `in`/`out` each point to
/// exactly 4 elements.
ZJOLT_API void zjoltATan4(const float *in, float *out);

/// zjoltATan2 over 4 independent (y, x) pairs at once. `in_y`, `in_x`, `out`
/// each point to exactly 4 elements.
ZJOLT_API void zjoltATan24(const float *in_y, const float *in_x, float *out);

//===----------------------------------------------------------------------===//
// Dense linear algebra — `Jolt/Math/Matrix.h`, `Jolt/Math/
// GaussianElimination.h`. Two square instantiations, both the sizes Jolt
// itself instantiates: 2x2 (`Mat22` in its hinge and dual-axis constraint
// parts, a 2-DOF effective mass) and 3x3 (MassProperties.cpp, ahead of its
// principal-moments eigensolve). Column-major: column c's row r is
// `m[Rows * c + r]`, same convention as ZJoltMat44.
//===----------------------------------------------------------------------===//

/// A dense 2-component column vector — `JPH::Vector<2>`, Matrix2's column,
/// diagonal, and zjoltMatrix2Solve's right-hand side and solution.
typedef struct ZJoltVector2 {
  float x, y;
} ZJoltVector2;

/// A dense 2x2 matrix — `JPH::Matrix<2, 2>`.
typedef struct ZJoltMatrix2 {
  float m[4];
} ZJoltMatrix2;

/// A dense 3x3 matrix — `JPH::Matrix<3, 3>`. Distinct from ZJoltMat44's
/// upper-left 3x3: no SIMD padding lane, and its inverse runs
/// GaussianElimination rather than ZJoltMat44's closed-form adjoint. Column,
/// diagonal and solve vectors reuse ZJoltVec3 (three bare floats, same
/// layout) rather than a second three-float type.
typedef struct ZJoltMatrix3 {
  float m[9];
} ZJoltMatrix3;

/// The all-zeros 2x2 matrix.
ZJOLT_API void zjoltMatrix2Zero(ZJoltMatrix2 *out);

/// The 2x2 identity matrix.
ZJOLT_API void zjoltMatrix2Identity(ZJoltMatrix2 *out);

/// The 2x2 diagonal matrix with `v` on the diagonal and zero elsewhere —
/// replaces any prior content, it does not merge with an existing matrix.
ZJOLT_API void zjoltMatrix2Diagonal(const ZJoltVector2 *v, ZJoltMatrix2 *out);

/// Row count of a Matrix2: always 2.
ZJOLT_API uint32_t zjoltMatrix2GetRows(void);

/// Column count of a Matrix2: always 2.
ZJOLT_API uint32_t zjoltMatrix2GetCols(void);

/// Whether `m` equals zjoltMatrix2Identity's result exactly.
ZJOLT_API bool zjoltMatrix2IsIdentity(const ZJoltMatrix2 *m);

/// Column `column` of `m`. ZJOLT_RESULT_INVALID_ARGUMENT when `column >= 2`
/// (Jolt's own GetColumn has no such check and reads out of bounds).
ZJOLT_API ZJoltResult zjoltMatrix2GetColumn(const ZJoltMatrix2 *m,
                                            uint32_t column,
                                            ZJoltVector2 *out);

/// `m` with column `column` replaced by `v`, every other column unchanged.
/// Same bounds check as zjoltMatrix2GetColumn.
ZJOLT_API ZJoltResult zjoltMatrix2WithColumn(const ZJoltMatrix2 *m,
                                             uint32_t column,
                                             const ZJoltVector2 *v,
                                             ZJoltMatrix2 *out);

/// The diagonal of `m` (row i, column i for each i), NOT restricted to a
/// matrix zjoltMatrix2Diagonal built.
ZJOLT_API void zjoltMatrix2GetDiagonal(const ZJoltMatrix2 *m,
                                        ZJoltVector2 *out);

/// Squared-distance closeness: true when every column of `a` is within
/// `max_dist_sq` (by Vector::IsClose) of the matching column of `b`.
ZJOLT_API bool zjoltMatrix2IsClose(const ZJoltMatrix2 *a, const ZJoltMatrix2 *b,
                                    float max_dist_sq);

/// The transpose of `m`.
ZJOLT_API void zjoltMatrix2Transposed(const ZJoltMatrix2 *m, ZJoltMatrix2 *out);

/// The inverse of `m`. ZJOLT_RESULT_INVALID_ARGUMENT when `m` is singular
/// (or within its solver's tolerance of singular) — Jolt's own
/// `Matrix<2,2>::Inversed` uses a closed-form determinant, not
/// GaussianElimination; zjoltMatrix2Solve exercises that path directly.
ZJOLT_API ZJoltResult zjoltMatrix2Inversed(const ZJoltMatrix2 *m,
                                           ZJoltMatrix2 *out);

/// Matrix product `a * b`.
ZJOLT_API void zjoltMatrix2Multiply(const ZJoltMatrix2 *a,
                                     const ZJoltMatrix2 *b, ZJoltMatrix2 *out);

/// `m * v`.
ZJOLT_API void zjoltMatrix2MultiplyVector(const ZJoltMatrix2 *m,
                                          const ZJoltVector2 *v,
                                          ZJoltVector2 *out);

/// `m` scaled by `s`.
ZJOLT_API void zjoltMatrix2MultiplyScalar(const ZJoltMatrix2 *m, float s,
                                          ZJoltMatrix2 *out);

/// Elementwise `a + b`.
ZJOLT_API void zjoltMatrix2Add(const ZJoltMatrix2 *a, const ZJoltMatrix2 *b,
                                ZJoltMatrix2 *out);

/// Elementwise `a - b`.
ZJOLT_API void zjoltMatrix2Subtract(const ZJoltMatrix2 *a,
                                     const ZJoltMatrix2 *b, ZJoltMatrix2 *out);

/// Solves `a * x = b` by Gauss-Jordan elimination with partial pivoting —
/// `JPH::GaussianElimination`, the algorithm Jolt's constraint machinery
/// uses for its small dense systems. `tolerance` refuses a pivot smaller
/// than this (1.0e-16 matches Jolt's own default). Returns
/// ZJOLT_RESULT_INVALID_ARGUMENT, not a wrong answer, when `a` is singular
/// within `tolerance` — `*out_x` is left unwritten in that case.
ZJOLT_API ZJoltResult zjoltMatrix2Solve(const ZJoltMatrix2 *a,
                                        const ZJoltVector2 *b, float tolerance,
                                        ZJoltVector2 *out_x);

/// As zjoltMatrix2Zero, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3Zero(ZJoltMatrix3 *out);

/// As zjoltMatrix2Identity, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3Identity(ZJoltMatrix3 *out);

/// As zjoltMatrix2Diagonal, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3Diagonal(const ZJoltVec3 *v, ZJoltMatrix3 *out);

/// Row count of a Matrix3: always 3.
ZJOLT_API uint32_t zjoltMatrix3GetRows(void);

/// Column count of a Matrix3: always 3.
ZJOLT_API uint32_t zjoltMatrix3GetCols(void);

/// As zjoltMatrix2IsIdentity, over ZJoltMatrix3.
ZJOLT_API bool zjoltMatrix3IsIdentity(const ZJoltMatrix3 *m);

/// As zjoltMatrix2GetColumn, over ZJoltMatrix3; `column` must be `< 3`.
ZJOLT_API ZJoltResult zjoltMatrix3GetColumn(const ZJoltMatrix3 *m,
                                            uint32_t column, ZJoltVec3 *out);

/// As zjoltMatrix2WithColumn, over ZJoltMatrix3; `column` must be `< 3`.
ZJOLT_API ZJoltResult zjoltMatrix3WithColumn(const ZJoltMatrix3 *m,
                                             uint32_t column,
                                             const ZJoltVec3 *v,
                                             ZJoltMatrix3 *out);

/// As zjoltMatrix2GetDiagonal, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3GetDiagonal(const ZJoltMatrix3 *m, ZJoltVec3 *out);

/// As zjoltMatrix2IsClose, over ZJoltMatrix3.
ZJOLT_API bool zjoltMatrix3IsClose(const ZJoltMatrix3 *a, const ZJoltMatrix3 *b,
                                    float max_dist_sq);

/// As zjoltMatrix2Transposed, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3Transposed(const ZJoltMatrix3 *m, ZJoltMatrix3 *out);

/// As zjoltMatrix2Inversed, over ZJoltMatrix3 — here `Matrix<3,3>::Inversed`
/// DOES run GaussianElimination internally (no 3x3 closed-form
/// specialization exists), unlike the 2x2 case.
ZJOLT_API ZJoltResult zjoltMatrix3Inversed(const ZJoltMatrix3 *m,
                                           ZJoltMatrix3 *out);

/// As zjoltMatrix2Multiply, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3Multiply(const ZJoltMatrix3 *a,
                                     const ZJoltMatrix3 *b, ZJoltMatrix3 *out);

/// As zjoltMatrix2MultiplyVector, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3MultiplyVector(const ZJoltMatrix3 *m,
                                          const ZJoltVec3 *v, ZJoltVec3 *out);

/// As zjoltMatrix2MultiplyScalar, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3MultiplyScalar(const ZJoltMatrix3 *m, float s,
                                          ZJoltMatrix3 *out);

/// As zjoltMatrix2Add, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3Add(const ZJoltMatrix3 *a, const ZJoltMatrix3 *b,
                                ZJoltMatrix3 *out);

/// As zjoltMatrix2Subtract, over ZJoltMatrix3.
ZJOLT_API void zjoltMatrix3Subtract(const ZJoltMatrix3 *a,
                                     const ZJoltMatrix3 *b, ZJoltMatrix3 *out);

/// As zjoltMatrix2Solve, over ZJoltMatrix3.
ZJOLT_API ZJoltResult zjoltMatrix3Solve(const ZJoltMatrix3 *a,
                                        const ZJoltVec3 *b, float tolerance,
                                        ZJoltVec3 *out_x);

/// `m` narrowed to ZJoltMat44 — `DMat44::ToMat44` under `-Ddouble_precision`
/// (an ordinary round-to-nearest narrow, NOT the directed rounding
/// zjoltRVec3ToVec3RoundDown/Up use), an exact copy otherwise.
ZJOLT_API void zjoltRMat44ToMat44(const ZJoltRMat44 *m, ZJoltMat44 *out);

//===----------------------------------------------------------------------===//
// Half floats — `Jolt/Math/HalfFloat.h`. Bound rather than ported because
// the rounding and the Inf/NaN handling are specific to Jolt's own bit
// manipulation, not IEEE-754 binary16 conversion in general.
//===----------------------------------------------------------------------===//

/// A 16-bit IEEE-754 binary16 value — Jolt's own `HalfFloat`.
typedef uint16_t ZJoltHalfFloat;

/// Rounding mode for zjoltHalfFloatFromFloat(Fallback) — Jolt's own
/// `HalfFloatConversion::ERoundingMode`, same numeric values.
typedef enum ZJoltHalfFloatRoundingMode {
  ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_NEG_INF = 0,
  ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_POS_INF = 1,
  ZJOLT_HALF_FLOAT_ROUNDING_MODE_ROUND_TO_NEAREST = 2,
} ZJoltHalfFloatRoundingMode;

/// `value` converted to half float on the fastest path this build has
/// (F16C/NEON, or the portable fallback). Overflow saturates to +-infinity
/// or +-HALF_FLT_MAX depending on `mode`; any NaN maps to a quiet NaN.
ZJOLT_API ZJoltHalfFloat zjoltHalfFloatFromFloat(float value,
                                                 ZJoltHalfFloatRoundingMode mode);

/// As zjoltHalfFloatFromFloat, but always the portable bit-manipulation
/// path — what a cross_platform_deterministic build needs, since the F16C/
/// NEON paths are bit-identical to THIS but not vouched for against each
/// other across every microarchitecture this library targets.
ZJOLT_API ZJoltHalfFloat zjoltHalfFloatFromFloatFallback(
    float value, ZJoltHalfFloatRoundingMode mode);

/// 4 half floats converted to float, fastest available path. `in`/`out`
/// each point to exactly 4 elements.
ZJOLT_API void zjoltHalfFloatToFloat4(const ZJoltHalfFloat *in, float *out);

/// As zjoltHalfFloatToFloat4, but always the portable path.
ZJOLT_API void zjoltHalfFloatToFloatFallback4(const ZJoltHalfFloat *in,
                                              float *out);

//===----------------------------------------------------------------------===//
// The rest of `Jolt/Math/`: root finding and angle wrapping.
//===----------------------------------------------------------------------===//

/// Roots of `a*x^2 + b*x + c = 0`, including the degenerate linear
/// (`a == 0`) and constant cases. Returns the count: 0, 1 (`*out_x1 ==
/// *out_x2`), or 2. `a == b == c == 0` reports count 1 with `*out_x1 == 0`,
/// one of Jolt's own infinitely-many solutions, not a refusal.
ZJOLT_API uint32_t zjoltFindRoot(float a, float b, float c, float *out_x1,
                                  float *out_x2);

/// `radians` shifted by whole turns of 2*pi into `[-pi, pi]`.
ZJOLT_API float zjoltCenterAngleAroundZero(float radians);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_MATH_H_
