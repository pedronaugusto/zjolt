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
// Quaternions
//
// ZJoltQuat is (x, y, z, w) — w LAST, matching Jolt (zjolt_core.h). A function
// here that treats one as a ROTATION and that Jolt itself guards with
// JPH_ASSERT(IsNormalized()) — RotateVector, GetAxisAngle, and the axis
// argument to FromAxisAngle — checks the same condition itself and reports
// ZJOLT_RESULT_INVALID_ARGUMENT rather than reaching that assert. A function
// with no such assert (Multiply, Inverse, Conjugate, LERP, SLERP, ...) is
// infallible and computes whatever the arithmetic gives, which is a
// plausible-looking wrong answer — often silently, sometimes NaN — for
// nonsense input. That is what Jolt's own C++ would do with the same input,
// and this does not add a safety net Jolt itself does not have.
//===----------------------------------------------------------------------===//

/// Composes two rotations: `out = lhs * rhs`.
///
/// Hamilton product order, not "apply lhs then rhs": rotating a vector by
/// `out` matches rotating it by `rhs` FIRST and then by `lhs`. Get the operand
/// order backwards and a single composed rotation can still look plausible —
/// only chaining a second one exposes it, mirrored around the wrong axis.
///
/// Infallible: Jolt's own `operator*` carries no normalisation requirement.
/// Both operands are ordinarily unit quaternions, but nothing here checks —
/// composing two non-unit quaternions is well-defined arithmetic, it is just
/// not a rotation.
ZJOLT_API void zjoltQuatMultiply(const ZJoltQuat *lhs, const ZJoltQuat *rhs,
                                  ZJoltQuat *out);

/// Rotates `v` by `q`.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT when `q` is not unit length — Jolt's
/// `operator*(Vec3)` asserts `IsNormalized()`, checked here first at the same
/// 1e-5 tolerance zjoltQuatIsNormalized uses, so the two never disagree about
/// what counts as normalised. Renormalise with zjoltQuatNormalize first if
/// `q` merely drifted, which is the ordinary case after integrating a
/// rotation over many frames.
ZJOLT_API ZJoltResult zjoltQuatRotateVector(const ZJoltQuat *q,
                                             const ZJoltVec3 *v,
                                             ZJoltVec3 *out);

/// The inverse rotation: for a unit `q`, both `zjoltQuatMultiply(q, out, ..)`
/// and `zjoltQuatMultiply(out, q, ..)` give the identity.
///
/// Infallible: this is `Conjugated() / Length()`, exactly as Jolt's own
/// `Inversed()` is, and a zero-length `q` produces NaN the same way Jolt's
/// would rather than being refused.
ZJOLT_API void zjoltQuatInverse(const ZJoltQuat *q, ZJoltQuat *out);

/// The conjugate `(-x, -y, -z, w)`. Equal to the inverse for a unit
/// quaternion, and cheaper to compute: no length, no division. Infallible.
ZJOLT_API void zjoltQuatConjugate(const ZJoltQuat *q, ZJoltQuat *out);

/// The four-component dot product `lhs.x*rhs.x + ... + lhs.w*rhs.w`. For two
/// unit quaternions this is the cosine of half the angle between the
/// rotations they represent — 1.0 for equal rotations, -1.0 for `q` against
/// `-q`, which represent the SAME rotation but sit at opposite points on the
/// space SLERP interpolates over (see zjoltQuatSlerp). 0.0, not a refusal,
/// when either pointer is null.
ZJOLT_API float zjoltQuatDot(const ZJoltQuat *a, const ZJoltQuat *b);

/// Whether `q`'s squared length is within `tolerance` of 1.
///
/// This compares the SQUARED length to 1, not the length itself (Jolt's own
/// `Quat::IsNormalized`, via `Vec4::IsNormalized`), so a tolerance sized for a
/// length-based check is stricter than it looks here. Pass 1e-5 to match
/// exactly the default Jolt's own `JPH_ASSERT(IsNormalized())` calls use —
/// including the ones zjoltQuatRotateVector and zjoltQuatGetAxisAngle check on
/// this ABI's behalf. False, not a refusal, when `q` is null.
ZJOLT_API bool zjoltQuatIsNormalized(const ZJoltQuat *q, float tolerance);

/// `q` rescaled to unit length.
///
/// Infallible: a zero (or NaN-carrying) `q` produces NaN, exactly as Jolt's
/// own `Normalized()` would. There is no identity fallback here the way
/// there is inside this library's OWN internal renormalisation of a body's
/// rotation on every step — check zjoltQuatIsNormalized, or the result for a
/// NaN component, first if a hard failure is not acceptable.
ZJOLT_API void zjoltQuatNormalize(const ZJoltQuat *q, ZJoltQuat *out);

/// A right-handed rotation of `radians` about `axis`.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT when `axis` is not unit length — Jolt's
/// `Quat::sRotation` asserts `inAxis.IsNormalized()`, checked here first, at
/// Vec3's own default tolerance of 1e-6 (tighter than the 1e-5 the quaternion
/// functions above use: this checks a three-component axis, not a
/// four-component quaternion).
ZJOLT_API ZJoltResult zjoltQuatFromAxisAngle(const ZJoltVec3 *axis,
                                              float radians, ZJoltQuat *out);

/// The axis and angle zjoltQuatFromAxisAngle would have to be given to
/// reconstruct `q`. `*out_angle` always comes back in [0, pi]: a rotation
/// past pi around some axis is reported as the shorter, positive rotation
/// around the opposite axis, never as a reflex angle.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT when `q` is not unit length (`GetAxisAngle`
/// asserts `IsNormalized()`, checked here first).
ZJOLT_API ZJoltResult zjoltQuatGetAxisAngle(const ZJoltQuat *q,
                                             ZJoltVec3 *out_axis,
                                             float *out_angle);

/// The rotation that takes the DIRECTION of `from` to the DIRECTION of `to`,
/// along the shorter great-circle arc.
///
/// Infallible, and every degenerate case has a defined answer rather than a
/// NaN: a zero-length `from` or `to` gives the identity, and two exactly
/// opposite directions give SOME 180-degree rotation about a perpendicular
/// axis — there is no unique one to prefer, so this is not necessarily the
/// same 180-degree rotation zjoltQuatFromAxisAngle would build from a
/// specific axis. Neither input needs to be unit length; only its direction
/// is used.
ZJOLT_API void zjoltQuatFromTo(const ZJoltVec3 *from, const ZJoltVec3 *to,
                                ZJoltQuat *out);

/// A rotation from Euler angles `(x, y, z)`, in radians.
///
/// Jolt fixes the axis order: the rotation is `RotZ * RotY * RotX`, i.e. a
/// vector is rotated about the ORIGINAL X axis first, then Y, then Z. This is
/// one convention among several in general use — Unity and Unreal both
/// default to a different order — so a triple imported from elsewhere is not
/// interchangeable with this call without checking its axis order first.
/// Infallible.
ZJOLT_API void zjoltQuatFromEulerAngles(const ZJoltVec3 *angles_radians,
                                         ZJoltQuat *out);

/// The Euler angles zjoltQuatFromEulerAngles would have to be given to
/// reconstruct `q`, in the same X-then-Y-then-Z order.
///
/// Not a true inverse near the gimbal-lock singularity (the Y angle at
/// +-pi/2): the middle angle is clamped there and the X/Z split becomes
/// arbitrary, the way it does for every Euler-angle representation, not a
/// defect specific to this one. Infallible in the sense that it never
/// aborts — but it also never checks that `q` is unit length, so a non-unit
/// `q` produces a plausible-looking but meaningless triple rather than an
/// error.
ZJOLT_API void zjoltQuatGetEulerAngles(const ZJoltQuat *q, ZJoltVec3 *out);

/// A quaternion perpendicular to `q` in the four-dimensional sense Jolt's own
/// swing-twist machinery uses internally — NOT "perpendicular" in the sense
/// of a 3-D rotation axis. Infallible.
ZJOLT_API void zjoltQuatGetPerpendicular(const ZJoltQuat *q, ZJoltQuat *out);

/// The signed angle `q` rotates around `axis`, computed the same way
/// zjoltQuatGetTwist derives its twist quaternion (Jolt: "uses Swing Twist
/// Decomposition to get the twist quaternion").
///
/// `axis` MUST be unit length. Unlike zjoltQuatFromAxisAngle's axis, this is
/// NOT checked or normalised for the caller — Jolt's own `GetRotationAngle`
/// neither asserts nor normalises, and the formula it uses is only
/// scale-invariant in `axis` by accident of algebra that does not actually
/// hold: a non-unit axis silently changes the reported angle by a nonlinear
/// amount rather than being cancelled out. 0.0, not a refusal, when either
/// pointer is null.
ZJOLT_API float zjoltQuatGetRotationAngle(const ZJoltQuat *q,
                                           const ZJoltVec3 *axis);

/// The component of `q` that rotates ONLY around `axis`.
///
/// `axis` MUST be unit length, for the same reason as zjoltQuatGetRotationAngle:
/// Jolt's own `GetTwist` neither asserts nor normalises it, and a non-unit
/// axis changes the result rather than being harmlessly cancelled.
///
/// Infallible otherwise: the identity comes back if `q` has no component
/// around `axis` at all, rather than a divide-by-zero NaN.
ZJOLT_API void zjoltQuatGetTwist(const ZJoltQuat *q, const ZJoltVec3 *axis,
                                  ZJoltQuat *out);

/// Splits `q` into `q = swing * twist` (recompose with
/// `zjoltQuatMultiply(out_swing, out_twist, &q)`), where `twist` rotates ONLY
/// around the quaternion's OWN LOCAL X AXIS and `swing` rotates only around
/// its local Y and Z axes.
///
/// The fixed axis is the whole difference from zjoltQuatGetTwist, which takes
/// the axis as a parameter: that call and this one agree only when the axis
/// passed to it happens to be local X, and are different decompositions for
/// any other axis. This fixed-X split is the one zjoltSwingTwistConstraintCreate's
/// limits are expressed against (constraint.zig), because a constraint's
/// twist axis is always its local X by construction.
///
/// Infallible: a 180-degree rotation about Y or Z — the one case with no
/// unique twist — reports the identity twist and puts the whole rotation into
/// swing, which is what Jolt's own `GetSwingTwist` does rather than picking
/// an arbitrary twist.
ZJOLT_API void zjoltQuatGetSwingTwist(const ZJoltQuat *q, ZJoltQuat *out_swing,
                                       ZJoltQuat *out_twist);

/// Componentwise interpolation `(1 - t) * a + t * b`. NOT renormalised and
/// NOT taking the shorter path around the rotation space — for `t` outside
/// [0, 1] this extrapolates, and even inside it the result is not unit length
/// unless `a` and `b` already are and happen to average out close to one.
/// Prefer zjoltQuatSlerp to interpolate a rotation across a frame; this is
/// only cheaper, and only close to correct, when `a` and `b` are already
/// nearly equal — which is the case Jolt's own `LERP` documents itself for.
ZJOLT_API void zjoltQuatLerp(const ZJoltQuat *a, const ZJoltQuat *b, float t,
                              ZJoltQuat *out);

/// Spherical interpolation from `a` to `b`, taking whichever of the two arcs
/// around the four-dimensional rotation sphere is SHORTER.
///
/// `a` and `-a` represent the same rotation, but SLERPing from `a` to some
/// `b` and from `-a` to that same `b` sweep different arcs unless this
/// correction is applied — and it is, automatically, by negating one side
/// when their dot product is negative. That sign flip is the entire reason to
/// call this instead of writing the interpolation formula by hand: get it
/// wrong, or fetch a formula that omits it, and a body interpolated every
/// frame between two keyframes spins the LONG way round once per revolution —
/// silently, and only visible in motion, never in a single frame's numbers.
///
/// `t` outside [0, 1] extrapolates. The result is unit length whenever `a`
/// and `b` are; neither is checked or renormalised first.
ZJOLT_API void zjoltQuatSlerp(const ZJoltQuat *a, const ZJoltQuat *b, float t,
                               ZJoltQuat *out);

//===----------------------------------------------------------------------===//
// Vector interpolation
//
// Componentwise, unlike SLERP above — there is no "spherical" version of a
// plain linear quantity, and Jolt itself has no Vec3::Lerp to bind: this is
// `a + (b - a) * t`, written out here rather than left to a caller because
// the standard fixed-timestep game loop needs it every frame (interpolating a
// rendered body between the previous and current physics step, alongside
// zjoltQuatSlerp for its rotation) and because it is one call over this ABI's
// own struct layout rather than three multiplies a caller has to get right
// over someone else's.
//===----------------------------------------------------------------------===//

/// `(1 - t) * a + t * b`, componentwise. `t` outside [0, 1] extrapolates.
ZJOLT_API void zjoltVec3Lerp(const ZJoltVec3 *a, const ZJoltVec3 *b, float t,
                              ZJoltVec3 *out);

/// As zjoltVec3Lerp, but over ZJoltRVec3 — the type a WORLD-SPACE position
/// actually has, `double` under -Ddouble_precision. Interpolating a body's
/// position with zjoltVec3Lerp instead would mean narrowing it to `float`
/// first, which throws away exactly the precision double-precision mode
/// exists to keep, at exactly the position furthest from the origin where it
/// matters most.
ZJOLT_API void zjoltRVec3Lerp(const ZJoltRVec3 *a, const ZJoltRVec3 *b,
                               float t, ZJoltRVec3 *out);

//===----------------------------------------------------------------------===//
// ZJoltMat44 — a plain 4x4, float even under -Ddouble_precision.
//
// This is the type zjoltBodyGetInverseInertia and a soft body's skinning
// joint matrices (zjoltSoftBodySkinVertices) carry: never a world-space
// position, so never widened. Column-major, per zjolt_core.h: column c's row
// r is `m[4 * c + r]`.
//===----------------------------------------------------------------------===//

/// Builds a transform: `rotation` in the upper 3x3, `translation` in the
/// fourth column.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT when `rotation` is not unit length — this
/// calls through Jolt's `Mat44::sRotation(Quat)`, which asserts
/// `IsNormalized()`, checked here first at the same tolerance
/// zjoltQuatRotateVector uses.
ZJOLT_API ZJoltResult zjoltMat44FromRotationTranslation(
    const ZJoltQuat *rotation, const ZJoltVec3 *translation, ZJoltMat44 *out);

/// Composes two transforms: `out = a * b`. As with zjoltQuatMultiply, this is
/// "apply b first, then a" — transforming a point by `out` matches
/// transforming it by `b` and then by `a`, not the other order. Infallible.
ZJOLT_API void zjoltMat44Multiply(const ZJoltMat44 *a, const ZJoltMat44 *b,
                                   ZJoltMat44 *out);

/// The general inverse: `out` composed with `m` (either order) is the
/// identity for any invertible `m`, including one carrying scale or shear.
///
/// Infallible in the sense that nothing aborts, but a singular `m` (zero
/// determinant) divides by zero and `*out` comes back full of NaN or Inf,
/// exactly as Jolt's own `Inversed()` would — there is no determinant check.
/// Prefer zjoltMat44InverseRotationTranslation when `m` is known to carry no
/// scale or shear: it is both cheaper and exact where this is merely
/// accurate.
ZJOLT_API void zjoltMat44Inverse(const ZJoltMat44 *m, ZJoltMat44 *out);

/// The inverse of `m`, ASSUMING `m` is a rigid transform — its upper 3x3 is
/// exactly a rotation (orthonormal, no scale or shear) and its fourth column
/// is a translation. Computed as a transpose and a dot product rather than a
/// general 4x4 inverse, which is both cheaper and exact where
/// zjoltMat44Inverse merely converges.
///
/// This assumption holds for every transform this ABI hands OUT — a body's
/// world transform, its centre-of-mass transform — but not for one carrying
/// scale, and nothing here can tell the difference: Jolt's own
/// `InversedRotationTranslation` does not check either, and this silently
/// returns the wrong answer for a scaled `m` rather than detecting the
/// mismatch. Infallible.
ZJOLT_API void zjoltMat44InverseRotationTranslation(const ZJoltMat44 *m,
                                                      ZJoltMat44 *out);

/// Transforms `point` as a POSITION: the fourth column (translation) is
/// added. Infallible.
ZJOLT_API void zjoltMat44TransformPoint(const ZJoltMat44 *m,
                                         const ZJoltVec3 *point,
                                         ZJoltVec3 *out);

/// Transforms `direction` as a DIRECTION: only the upper 3x3 is applied, and
/// the fourth column (translation) is ignored entirely — the whole
/// difference from zjoltMat44TransformPoint, and what a caller who reaches
/// for TransformPoint on a normal or a velocity gets wrong, picking up an
/// offset by the transform's position that a direction should never have.
/// Infallible.
ZJOLT_API void zjoltMat44TransformDirection(const ZJoltMat44 *m,
                                             const ZJoltVec3 *direction,
                                             ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// ZJoltRMat44 — the WORLD-SPACE transform type: ZJoltMat44 with ZJoltReal
// elements, `double` under -Ddouble_precision. zjoltBodyGetWorldTransform and
// zjoltBodyGetCenterOfMassTransform return this, not ZJoltMat44 — composing
// or inverting one of THOSE through the ZJoltMat44 functions above would mean
// narrowing every position through `float` on the way in, which is exactly
// the precision loss -Ddouble_precision exists to avoid. Same five
// operations as above, over ZJoltReal, so a -Ddouble_precision build is not a
// second-class citizen of this file.
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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_MATH_H_
