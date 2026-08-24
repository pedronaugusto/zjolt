//! The plain-data types that cross the boundary.
//!
//! These are the C types re-exported, not copies: a wrapper struct would mean
//! a conversion per body per frame for no benefit.

const std = @import("std");
const c = @import("c/core.zig");
const c_math = @import("c/math.zig");
const err = @import("error.zig");

/// A direction, velocity or extent. Three floats even in a double-precision
/// build — only positions get the extra range, which is the same split Jolt
/// makes internally.
pub const Vec3 = c.Vec3;

/// A world-space position. `f64` when the library was built with
/// `-Ddouble_precision`, otherwise `f32`.
pub const RVec3 = c.RVec3;

/// The scalar type of a world-space position.
pub const Real = c.Real;

/// A quaternion in (x, y, z, w) order — w LAST, matching Jolt. Consumers whose
/// own quaternion type is w-first must reorder; there is no silent conversion
/// here to get wrong.
pub const Quat = c.Quat;

/// A 4x4 matrix: four COLUMNS of four floats, so column `c`'s row `r` is
/// `m[4 * c + r]`. Column-major, matching Jolt — not row-major.
pub const Mat44 = c.Mat44;

/// A world-space transform. `Mat44` with `Real` elements, exactly as `RVec3`
/// is `Vec3` with `Real` elements, so its width follows
/// `-Ddouble_precision`.
pub const RMat44 = c.RMat44;

pub const AABox = c.AABox;
pub const MassProperties = c.MassProperties;
pub const ShapeStats = c.ShapeStats;

pub const vec3_zero: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
pub const rvec3_zero: RVec3 = .{ .x = 0, .y = 0, .z = 0 };
pub const quat_identity: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };

pub const mat44_identity: Mat44 = .{ .m = .{
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
} };
pub const rmat44_identity: RMat44 = .{ .m = .{
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
} };

/// Earth gravity along -Y, the axis Jolt treats as up by default.
pub const gravity_earth: Vec3 = .{ .x = 0, .y = -9.81, .z = 0 };

pub fn vec3(x: f32, y: f32, z: f32) Vec3 {
    return .{ .x = x, .y = y, .z = z };
}

pub fn rvec3(x: Real, y: Real, z: Real) RVec3 {
    return .{ .x = x, .y = y, .z = z };
}

pub fn quat(x: f32, y: f32, z: f32, w: f32) Quat {
    return .{ .x = x, .y = y, .z = z, .w = w };
}

/// A rotation of `radians` about a unit axis.
///
/// `axis` is trusted, not checked: this is plain arithmetic with no call into
/// the C library, so it works before `zjolt.init` and never fails, but a
/// non-unit `axis` silently produces a non-unit `Quat` rather than being
/// refused. `quatRotation` is the checked equivalent — it calls through
/// Jolt's own `Quat::sRotation`, which this reimplements, and reports
/// `error.InvalidArgument` for a non-unit axis instead.
pub fn quatFromAxisAngle(axis: Vec3, radians: f32) Quat {
    const half = radians * 0.5;
    const s = @sin(half);
    return .{ .x = axis.x * s, .y = axis.y * s, .z = axis.z * s, .w = @cos(half) };
}

/// Widens a direction to a position. Lossless in a float build, and an
/// explicit widening in a double-precision one.
pub fn toRVec3(v: Vec3) RVec3 {
    return .{ .x = @floatCast(v.x), .y = @floatCast(v.y), .z = @floatCast(v.z) };
}

//=============================================================================
// Quaternion algebra
//
// Everything below calls through `ffi/zjolt_math.h`, which is the C ABI's
// own quaternion and matrix surface — see that header for the full
// documentation of each operation's conventions and preconditions; this only
// restates what is Zig-specific: `err.Error!T` for what the C side reports
// through `ZJoltResult`, a plain `T` for what it reports by writing an
// out-parameter unconditionally.
//=============================================================================

/// Composes two rotations: `lhs * rhs`, Hamilton product order — rotating a
/// vector by the result matches rotating it by `rhs` first, then `lhs`.
pub fn quatMultiply(lhs: Quat, rhs: Quat) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatMultiply(&lhs, &rhs, &out);
    return out;
}

/// Rotates `v` by `q`. `error.InvalidArgument` when `q` is not unit length;
/// see `quatNormalize`.
pub fn quatRotateVector(q: Quat, v: Vec3) err.Error!Vec3 {
    var out: Vec3 = vec3_zero;
    try err.check(c_math.zjoltQuatRotateVector(&q, &v, &out));
    return out;
}

/// The inverse rotation. NaN on a zero-length `q`, exactly as Jolt's own
/// `Inversed()` — infallible, not refused.
pub fn quatInverse(q: Quat) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatInverse(&q, &out);
    return out;
}

/// `(-x, -y, -z, w)`. Equal to `quatInverse` for a unit `q`, and cheaper.
pub fn quatConjugate(q: Quat) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatConjugate(&q, &out);
    return out;
}

/// The four-component dot product.
pub fn quatDot(a: Quat, b: Quat) f32 {
    return c_math.zjoltQuatDot(&a, &b);
}

/// Whether `q`'s SQUARED length is within `tolerance` of 1 — see
/// `zjoltQuatIsNormalized` in `ffi/zjolt_math.h` for why that is stricter than
/// it looks. Pass `1e-5` to match what Jolt's own asserts require.
pub fn quatIsNormalized(q: Quat, tolerance: f32) bool {
    return c_math.zjoltQuatIsNormalized(&q, tolerance);
}

/// `q` rescaled to unit length. NaN on a zero-length `q`, not refused.
pub fn quatNormalize(q: Quat) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatNormalize(&q, &out);
    return out;
}

/// A right-handed rotation of `radians` about `axis`, CHECKED against Jolt's
/// own `Quat::sRotation` precondition: `error.InvalidArgument` when `axis` is
/// not unit length, rather than `quatFromAxisAngle`'s silent wrong answer.
/// Calls through the C library, so requires `zjolt.init` first.
pub fn quatRotation(axis: Vec3, radians: f32) err.Error!Quat {
    var out: Quat = quat_identity;
    try err.check(c_math.zjoltQuatFromAxisAngle(&axis, radians, &out));
    return out;
}

/// The axis and angle `quatRotation` would have to be given to reconstruct
/// `q`. `angle` is always in `[0, pi]`. `error.InvalidArgument` when `q` is
/// not unit length.
pub fn quatGetAxisAngle(q: Quat) err.Error!struct { axis: Vec3, angle: f32 } {
    var axis: Vec3 = vec3_zero;
    var angle: f32 = 0;
    try err.check(c_math.zjoltQuatGetAxisAngle(&q, &axis, &angle));
    return .{ .axis = axis, .angle = angle };
}

/// The rotation from the DIRECTION of `from` to the DIRECTION of `to`, along
/// the shorter arc. Neither needs to be unit length. Infallible: a
/// zero-length input gives the identity rather than NaN.
pub fn quatFromTo(from: Vec3, to: Vec3) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatFromTo(&from, &to, &out);
    return out;
}

/// A rotation from Euler angles `(x, y, z)` radians, Jolt's fixed
/// X-then-Y-then-Z order — see `ffi/zjolt_math.h` for what that means and
/// what it does not match.
pub fn quatFromEulerAngles(angles_radians: Vec3) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatFromEulerAngles(&angles_radians, &out);
    return out;
}

/// The Euler angles `quatFromEulerAngles` would have to be given to
/// reconstruct `q`, same axis order. Not exact through gimbal lock.
pub fn quatGetEulerAngles(q: Quat) Vec3 {
    var out: Vec3 = vec3_zero;
    c_math.zjoltQuatGetEulerAngles(&q, &out);
    return out;
}

/// A quaternion perpendicular to `q` in the four-dimensional sense Jolt's
/// swing-twist machinery uses — not a 3-D rotation axis.
pub fn quatGetPerpendicular(q: Quat) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatGetPerpendicular(&q, &out);
    return out;
}

/// The signed angle `q` rotates around `axis`, which MUST be unit length —
/// see `ffi/zjolt_math.h`; unlike `quatRotation`'s axis this is not checked.
pub fn quatGetRotationAngle(q: Quat, axis: Vec3) f32 {
    return c_math.zjoltQuatGetRotationAngle(&q, &axis);
}

/// The component of `q` that rotates only around `axis`, which MUST be unit
/// length for the same reason as `quatGetRotationAngle`.
pub fn quatGetTwist(q: Quat, axis: Vec3) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatGetTwist(&q, &axis, &out);
    return out;
}

/// Splits `q` into `q = swing * twist`, where `twist` rotates only around
/// `q`'s own LOCAL X AXIS (fixed, unlike `quatGetTwist`'s axis parameter) and
/// `swing` only around its local Y and Z. Recompose with
/// `quatMultiply(result.swing, result.twist)`. This is the decomposition
/// `Constraint.initSwingTwist`'s limits are expressed against.
pub fn quatGetSwingTwist(q: Quat) struct { swing: Quat, twist: Quat } {
    var swing: Quat = quat_identity;
    var twist: Quat = quat_identity;
    c_math.zjoltQuatGetSwingTwist(&q, &swing, &twist);
    return .{ .swing = swing, .twist = twist };
}

/// `(1 - t) * a + t * b`, componentwise — NOT renormalised, NOT shortest-path.
/// Prefer `quatSlerp` to interpolate a rotation across a frame.
pub fn quatLerp(a: Quat, b: Quat, t: f32) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatLerp(&a, &b, t, &out);
    return out;
}

/// Spherical interpolation from `a` to `b`, taking the shorter of the two
/// arcs — the sign correction that makes this safe to call every frame
/// between two keyframes; see `ffi/zjolt_math.h`.
pub fn quatSlerp(a: Quat, b: Quat, t: f32) Quat {
    var out: Quat = quat_identity;
    c_math.zjoltQuatSlerp(&a, &b, t, &out);
    return out;
}

/// `(1 - t) * a + t * b`, componentwise.
pub fn vec3Lerp(a: Vec3, b: Vec3, t: f32) Vec3 {
    var out: Vec3 = vec3_zero;
    c_math.zjoltVec3Lerp(&a, &b, t, &out);
    return out;
}

/// As `vec3Lerp`, over `RVec3` — the type a world-space position actually
/// has. Interpolating a position through `vec3Lerp` instead narrows it to
/// `f32` first, losing exactly the range `-Ddouble_precision` exists to keep.
pub fn rvec3Lerp(a: RVec3, b: RVec3, t: f32) RVec3 {
    var out: RVec3 = rvec3_zero;
    c_math.zjoltRVec3Lerp(&a, &b, t, &out);
    return out;
}

//=============================================================================
// Matrix algebra
//
// `Mat44` and `RMat44` (see `ffi/zjolt_math.h` for the ZJoltMat44 versus
// ZJoltRMat44 distinction this mirrors exactly). Every "Rotation" function
// checks its quaternion the same way `quatRotation` does.
//=============================================================================

/// Builds a transform: `rotation` in the upper 3x3, `translation` in the
/// fourth column. `error.InvalidArgument` when `rotation` is not unit length.
pub fn mat44FromRotationTranslation(rotation: Quat, translation: Vec3) err.Error!Mat44 {
    var out: Mat44 = mat44_identity;
    try err.check(c_math.zjoltMat44FromRotationTranslation(&rotation, &translation, &out));
    return out;
}

/// Composes two transforms: `a * b` — apply `b` first, then `a`.
pub fn mat44Multiply(a: Mat44, b: Mat44) Mat44 {
    var out: Mat44 = mat44_identity;
    c_math.zjoltMat44Multiply(&a, &b, &out);
    return out;
}

/// The general inverse. NaN/Inf on a singular `m`, not refused. Prefer
/// `mat44InverseRotationTranslation` when `m` carries no scale or shear.
pub fn mat44Inverse(m: Mat44) Mat44 {
    var out: Mat44 = mat44_identity;
    c_math.zjoltMat44Inverse(&m, &out);
    return out;
}

/// The inverse of `m`, ASSUMING it is a rigid transform (orthonormal
/// rotation, no scale). Cheaper and exact where `mat44Inverse` merely
/// converges; wrong, silently, if that assumption does not hold.
pub fn mat44InverseRotationTranslation(m: Mat44) Mat44 {
    var out: Mat44 = mat44_identity;
    c_math.zjoltMat44InverseRotationTranslation(&m, &out);
    return out;
}

/// Transforms `point` as a POSITION: the translation column is added.
pub fn mat44TransformPoint(m: Mat44, point: Vec3) Vec3 {
    var out: Vec3 = vec3_zero;
    c_math.zjoltMat44TransformPoint(&m, &point, &out);
    return out;
}

/// Transforms `direction` as a DIRECTION: the translation column is ignored.
pub fn mat44TransformDirection(m: Mat44, direction: Vec3) Vec3 {
    var out: Vec3 = vec3_zero;
    c_math.zjoltMat44TransformDirection(&m, &direction, &out);
    return out;
}

/// As `mat44FromRotationTranslation`, `translation` widened to `RVec3` — the
/// type `zjolt.BodyInterface.getWorldTransform` actually returns.
pub fn rmat44FromRotationTranslation(rotation: Quat, translation: RVec3) err.Error!RMat44 {
    var out: RMat44 = rmat44_identity;
    try err.check(c_math.zjoltRMat44FromRotationTranslation(&rotation, &translation, &out));
    return out;
}

/// As `mat44Multiply`, over `RMat44`.
pub fn rmat44Multiply(a: RMat44, b: RMat44) RMat44 {
    var out: RMat44 = rmat44_identity;
    c_math.zjoltRMat44Multiply(&a, &b, &out);
    return out;
}

/// As `mat44Inverse`, over `RMat44`.
pub fn rmat44Inverse(m: RMat44) RMat44 {
    var out: RMat44 = rmat44_identity;
    c_math.zjoltRMat44Inverse(&m, &out);
    return out;
}

/// As `mat44InverseRotationTranslation`, over `RMat44`.
pub fn rmat44InverseRotationTranslation(m: RMat44) RMat44 {
    var out: RMat44 = rmat44_identity;
    c_math.zjoltRMat44InverseRotationTranslation(&m, &out);
    return out;
}

/// Transforms `point` — a world-space position, `RVec3` precision in and
/// out — as a POSITION.
pub fn rmat44TransformPoint(m: RMat44, point: RVec3) RVec3 {
    var out: RVec3 = rvec3_zero;
    c_math.zjoltRMat44TransformPoint(&m, &point, &out);
    return out;
}

/// Transforms `direction` as a DIRECTION. `Vec3`, not `RVec3`: a direction
/// never needs the extended range a world position does.
pub fn rmat44TransformDirection(m: RMat44, direction: Vec3) Vec3 {
    var out: Vec3 = vec3_zero;
    c_math.zjoltRMat44TransformDirection(&m, &direction, &out);
    return out;
}

test "identity constants are what their names claim" {
    try std.testing.expectEqual(@as(f32, 1), quat_identity.w);
    try std.testing.expectEqual(@as(f32, 0), vec3_zero.x);
    try std.testing.expectEqual(@as(f32, -9.81), gravity_earth.y);
}

test "an axis-angle rotation is unit length" {
    const q = quatFromAxisAngle(vec3(0, 1, 0), std.math.pi / 2.0);
    const length = @sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    try std.testing.expectApproxEqAbs(@as(f32, 1), length, 1e-6);
}

test "a swing-twist decomposition recomposes to the original rotation" {
    // An axis with no zero component and an angle nowhere near a multiple of
    // pi, so neither GetSwingTwist's degenerate branch (a 180-degree turn
    // about Y or Z) nor a trivially-identity twist is exercised by accident.
    const raw = vec3(0.3, 0.7, -0.2);
    const len = @sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z);
    const axis = vec3(raw.x / len, raw.y / len, raw.z / len);
    const q = quatFromAxisAngle(axis, 1.1);

    const parts = quatGetSwingTwist(q);
    const recomposed = quatMultiply(parts.swing, parts.twist);

    try std.testing.expectApproxEqAbs(q.x, recomposed.x, 1e-5);
    try std.testing.expectApproxEqAbs(q.y, recomposed.y, 1e-5);
    try std.testing.expectApproxEqAbs(q.z, recomposed.z, 1e-5);
    try std.testing.expectApproxEqAbs(q.w, recomposed.w, 1e-5);
}

test "quaternion multiply matches applying two rotations in sequence to a vector" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // rhs applied first, then lhs: the Hamilton product order this module
    // documents on quatMultiply.
    const rhs = quatFromAxisAngle(vec3(0, 0, 1), std.math.pi / 2.0);
    const lhs = quatFromAxisAngle(vec3(0, 1, 0), std.math.pi / 2.0);
    const composed = quatMultiply(lhs, rhs);

    const v = vec3(1, 0, 0);
    const sequential = try quatRotateVector(lhs, try quatRotateVector(rhs, v));
    const direct = try quatRotateVector(composed, v);

    try std.testing.expectApproxEqAbs(sequential.x, direct.x, 1e-5);
    try std.testing.expectApproxEqAbs(sequential.y, direct.y, 1e-5);
    try std.testing.expectApproxEqAbs(sequential.z, direct.z, 1e-5);
}

test "euler angles round-trip through quatFromEulerAngles and quatGetEulerAngles" {
    // Well away from the Y = +-pi/2 gimbal lock singularity, where the X/Z
    // split becomes arbitrary and a round trip is not expected to hold.
    const angles = vec3(0.2, -0.5, 0.9);
    const q = quatFromEulerAngles(angles);
    const back = quatGetEulerAngles(q);

    try std.testing.expectApproxEqAbs(angles.x, back.x, 1e-4);
    try std.testing.expectApproxEqAbs(angles.y, back.y, 1e-4);
    try std.testing.expectApproxEqAbs(angles.z, back.z, 1e-4);
}

test "quatRotation refuses a non-unit axis and rotates correctly with a unit one" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    try std.testing.expectError(err.Error.InvalidArgument, quatRotation(vec3(2, 0, 0), 1.0));

    const q = try quatRotation(vec3(0, 1, 0), std.math.pi / 2.0);
    const rotated = try quatRotateVector(q, vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotated.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotated.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, -1), rotated.z, 1e-5);
}

test "mat44 composes with its rigid inverse to the identity and carries its translation" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rotation = try quatRotation(vec3(0, 1, 0), 0.6);
    const translation = vec3(1, 2, 3);
    const m = try mat44FromRotationTranslation(rotation, translation);

    const inv = mat44InverseRotationTranslation(m);
    const identity_ish = mat44Multiply(inv, m);
    for (0..16) |i| {
        const expected: f32 = if (i % 5 == 0) 1 else 0; // diagonal: 0, 5, 10, 15
        try std.testing.expectApproxEqAbs(expected, identity_ish.m[i], 1e-4);
    }

    const transformed = mat44TransformPoint(m, vec3_zero);
    try std.testing.expectApproxEqAbs(translation.x, transformed.x, 1e-5);
    try std.testing.expectApproxEqAbs(translation.y, transformed.y, 1e-5);
    try std.testing.expectApproxEqAbs(translation.z, transformed.z, 1e-5);
}

test "rmat44 transforms a point and its rigid inverse undoes it" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rotation = try quatRotation(vec3(1, 0, 0), 0.4);
    const translation = rvec3(5, -2, 100);
    const m = try rmat44FromRotationTranslation(rotation, translation);
    const inv = rmat44InverseRotationTranslation(m);

    const original = rvec3(1, 1, 1);
    const world = rmat44TransformPoint(m, original);
    const back = rmat44TransformPoint(inv, world);

    try std.testing.expectApproxEqAbs(original.x, back.x, 1e-3);
    try std.testing.expectApproxEqAbs(original.y, back.y, 1e-3);
    try std.testing.expectApproxEqAbs(original.z, back.z, 1e-3);
}

test "lerp matches its endpoints and slerp stays unit length" {
    const a = quat_identity;
    const b = quatFromAxisAngle(vec3(0, 0, 1), std.math.pi / 2.0);

    const lerp_start = quatLerp(a, b, 0.0);
    const lerp_end = quatLerp(a, b, 1.0);
    try std.testing.expectApproxEqAbs(a.w, lerp_start.w, 1e-6);
    try std.testing.expectApproxEqAbs(b.w, lerp_end.w, 1e-6);

    const slerp_mid = quatSlerp(a, b, 0.5);
    const length = @sqrt(slerp_mid.x * slerp_mid.x + slerp_mid.y * slerp_mid.y +
        slerp_mid.z * slerp_mid.z + slerp_mid.w * slerp_mid.w);
    try std.testing.expectApproxEqAbs(@as(f32, 1), length, 1e-6);
}

test "vec3Lerp and rvec3Lerp interpolate componentwise" {
    const mid = vec3Lerp(vec3(0, 0, 0), vec3(10, -4, 2), 0.5);
    try std.testing.expectApproxEqAbs(@as(f32, 5), mid.x, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, -2), mid.y, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), mid.z, 1e-6);

    const rmid = rvec3Lerp(rvec3(0, 0, 0), rvec3(10, -4, 2), 0.5);
    try std.testing.expectApproxEqAbs(@as(Real, 5), rmid.x, 1e-6);
}
