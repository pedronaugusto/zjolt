//! Behavioural tests for the value-type helpers and free functions added to
//! `src/math.zig`. Expected values are hand-derived from Jolt's own source
//! (`Jolt/Math/{Mat44,Vec3,Vec4,Quat,Trigonometry,Matrix,GaussianElimination,
//! HalfFloat,FindRoot,Math}.{h,inl}`, `Jolt/Geometry/{AABox,AABox4,RayAABox,
//! RayTriangle,Triangle,IndexedTriangle,Plane}.h`), not by re-deriving them
//! from this file's own implementation or from `std.math`. The trig/matrix/
//! half-float sections below cross the ABI and, where fallible, need
//! `zjolt.init`; everything above them does not.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const math_mod = @import("math.zig");
const err = @import("error.zig");

const vec3 = zjolt.vec3;
const rvec3 = zjolt.rvec3;
const quat = zjolt.quat;
const Vec3 = zjolt.Vec3;
const RVec3 = zjolt.RVec3;
const Real = zjolt.Real;
const Mat44 = zjolt.Mat44;
const RMat44 = zjolt.RMat44;
const AABox = zjolt.AABox;

fn expectV3(expected: Vec3, actual: Vec3, tol: f32) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, tol);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, tol);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, tol);
}

//=============================================================================
// Vec3 / Quat
//=============================================================================

test "Vec3.normalizedPerpendicular is unit length and perpendicular to its input" {
    const a = vec3(5, 0, 0).normalizedPerpendicular();
    try expectV3(vec3(0, 0, -1), a, 1e-6);

    const b = vec3(0, 5, 0).normalizedPerpendicular();
    // xx (0) is not > yy (25), so the y-branch fires: (0, z, -y) = (0,0,-5), normalized.
    try expectV3(vec3(0, 0, -1), b, 1e-6);

    const c = vec3(3, 4, 0).normalizedPerpendicular();
    try std.testing.expectApproxEqAbs(@as(f32, 0), c.x * 3 + c.y * 4 + c.z * 0, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), @sqrt(c.x * c.x + c.y * c.y + c.z * c.z), 1e-6);
}

test "Vec3.isClose, isNearZero and isNormalized match hand-picked distances" {
    try std.testing.expect(vec3(1, 2, 3).isClose(vec3(1, 2, 3.0001), 1e-6));
    try std.testing.expect(!vec3(1, 2, 3).isClose(vec3(1, 2, 3.1), 1e-6));

    try std.testing.expect(vec3(0.0001, 0, 0).isNearZero(1e-6));
    try std.testing.expect(!vec3(0.1, 0, 0).isNearZero(1e-6));

    try std.testing.expect(vec3(1, 0, 0).isNormalized(1e-6));
    try std.testing.expect(!vec3(1.1, 0, 0).isNormalized(1e-6));
}

test "Quat.rotateAxisX/Y/Z match a hand-verified 90-degree rotation about Z" {
    // (0, 0, sin(45deg), cos(45deg)): Quat::sRotation(Vec3::sAxisZ(), pi/2).
    const half_sqrt2: f32 = 0.70710678;
    const q = quat(0, 0, half_sqrt2, half_sqrt2);

    try expectV3(vec3(0, 1, 0), q.rotateAxisX(), 1e-6);
    try expectV3(vec3(-1, 0, 0), q.rotateAxisY(), 1e-6);
    try expectV3(vec3(0, 0, 1), q.rotateAxisZ(), 1e-6);
}

//=============================================================================
// Compress / decompress: bit-exact against values ported to Python from the
// same source and hand-run there, not against this file's own Zig code.
//=============================================================================

test "Vec3.compressUnitVector encodes axis-aligned vectors to Jolt's exact bits" {
    try std.testing.expectEqual(@as(u32, 0x07FFDFFF), vec3(1, 0, 0).compressUnitVector());
    try std.testing.expectEqual(@as(u32, 0x27FFDFFF), vec3(0, 1, 0).compressUnitVector());
    try std.testing.expectEqual(@as(u32, 0xC7FFDFFF), vec3(0, 0, -1).compressUnitVector());
}

test "Vec3.decompressUnitVector inverts compressUnitVector within quantization error" {
    try expectV3(vec3(1, 0, 0), Vec3.decompressUnitVector(0x07FFDFFF), 1e-6);
    try expectV3(vec3(0, 1, 0), Vec3.decompressUnitVector(0x27FFDFFF), 1e-6);
    try expectV3(vec3(0, 0, -1), Vec3.decompressUnitVector(0xC7FFDFFF), 1e-6);

    const v = vec3(0.2672612, 0.5345225, 0.8017837); // (1,2,3) normalized
    const round_tripped = Vec3.decompressUnitVector(v.compressUnitVector());
    try expectV3(v, round_tripped, 1e-3);
}

test "Quat.compressUnitQuat encodes hand-picked quaternions to Jolt's exact bits" {
    try std.testing.expectEqual(@as(u32, 0x63FDFEFF), quat(0, 0, 0, 1).compressUnitQuat());
    // Quat::sRotation(Vec3::sAxisY(), 1.0): (0, sin(0.5), 0, cos(0.5)).
    try std.testing.expectEqual(@as(u32, 0x63FF58FF), quat(0, 0.4794255495071411, 0, 0.8775825500488281).compressUnitQuat());
    try std.testing.expectEqual(@as(u32, 0x06CF664B), quat(0.5, -0.5, 0.5, 0.5).compressUnitQuat());
}

test "Quat.decompressUnitQuat inverts compressUnitQuat within quantization error" {
    const q = quat(0.5, -0.5, 0.5, 0.5);
    const round_tripped = zjolt.Quat.decompressUnitQuat(q.compressUnitQuat());
    try std.testing.expectApproxEqAbs(q.x, round_tripped.x, 5e-3);
    try std.testing.expectApproxEqAbs(q.y, round_tripped.y, 5e-3);
    try std.testing.expectApproxEqAbs(q.z, round_tripped.z, 5e-3);
    try std.testing.expectApproxEqAbs(q.w, round_tripped.w, 5e-3);
}

//=============================================================================
// Mat44 / RMat44 — a 90-degree-about-Z rotation with translation (5,6,7),
// same fixture the existing `Mat44.decompose` tests in math.zig use.
//=============================================================================

const rot90z: Mat44 = .{ .m = .{
    0,  1, 0, 0,
    -1, 0, 0, 0,
    0,  0, 1, 0,
    5,  6, 7, 1,
} };

test "mat44 axis and column accessors read the fixture's known layout" {
    try expectV3(vec3(0, 1, 0), rot90z.axisX(), 1e-6);
    try expectV3(vec3(-1, 0, 0), rot90z.axisY(), 1e-6);
    try expectV3(vec3(0, 0, 1), rot90z.axisZ(), 1e-6);
    try expectV3(vec3(5, 6, 7), rot90z.column3(3), 1e-6);
    try std.testing.expectEqualSlices(f32, &.{ 5, 6, 7, 1 }, &rot90z.column4(3));
    try expectV3(vec3(0, 0, 1), rot90z.diagonal3(), 1e-6);
    try std.testing.expectEqualSlices(f32, &.{ 0, 0, 1, 1 }, &rot90z.diagonal4());
}

test "Mat44.withAxisX/Y/Z and withColumn3 round trip, withColumn3 forces w by column" {
    const with_axis = rot90z.withAxisX(vec3(9, 9, 9));
    try expectV3(vec3(9, 9, 9), with_axis.axisX(), 1e-6);
    try std.testing.expectEqual(@as(f32, 0), with_axis.m[3]);

    const translated = rot90z.withColumn3(3, vec3(1, 2, 3));
    try std.testing.expectEqual(@as(f32, 1), translated.m[15]); // col 3 -> w forced to 1
    const rotated_col = rot90z.withColumn3(0, vec3(1, 2, 3));
    try std.testing.expectEqual(@as(f32, 0), rotated_col.m[3]); // col 0 -> w forced to 0
}

test "Mat44.withDiagonal3/4 round trip through the getters" {
    const m = rot90z.withDiagonal4(.{ 2, 3, 4, 5 });
    try std.testing.expectEqualSlices(f32, &.{ 2, 3, 4, 5 }, &m.diagonal4());
    const m2 = rot90z.withDiagonal3(vec3(7, 8, 9));
    try expectV3(vec3(7, 8, 9), m2.diagonal3(), 1e-6);
}

test "Mat44.isClose matches a hand-picked tolerance boundary" {
    var nudged = rot90z;
    nudged.m[12] += 0.1; // translation.x off by 0.1 -> squared distance 0.01 in that column
    try std.testing.expect(rot90z.isClose(nudged, 0.02));
    try std.testing.expect(!rot90z.isClose(nudged, 0.005));
    try std.testing.expect(rot90z.isClose(rot90z, 1e-12));
}

test "Mat44.determinant3x3 is 1 for a pure rotation, matching CrossPrecise's dot(X, Y x Z)" {
    try std.testing.expectApproxEqAbs(@as(f32, 1), rot90z.determinant3x3(), 1e-6);

    const scaled: Mat44 = .{ .m = .{
        2, 0, 0, 0,
        0, 3, 0, 0,
        0, 0, 4, 0,
        0, 0, 0, 1,
    } };
    try std.testing.expectApproxEqAbs(@as(f32, 24), scaled.determinant3x3(), 1e-5);
}

test "Mat44.multiply3x3 composes two 90-degree Z rotations into 180 degrees" {
    const composed = rot90z.multiply3x3(rot90z);
    try expectV3(vec3(-1, 0, 0), composed.axisX(), 1e-6);
    try expectV3(vec3(0, -1, 0), composed.axisY(), 1e-6);
    try expectV3(vec3(0, 0, 1), composed.axisZ(), 1e-6);
    // Translation columns are ignored by multiply3x3, and the result's is forced to identity.
    try expectV3(vec3(0, 0, 0), composed.column3(3), 1e-6);
}

test "Mat44.multiply3x3RightTransposed of an orthonormal matrix with itself is the identity 3x3" {
    const identity_3x3 = rot90z.multiply3x3RightTransposed(rot90z);
    try expectV3(vec3(1, 0, 0), identity_3x3.axisX(), 1e-6);
    try expectV3(vec3(0, 1, 0), identity_3x3.axisY(), 1e-6);
    try expectV3(vec3(0, 0, 1), identity_3x3.axisZ(), 1e-6);
}

test "Mat44.multiply3x3Transposed transforms by the transpose of the rotation" {
    // Transposed3x3(rot90z)'s column 0 is rot90z's row 0: (0, -1, 0).
    try expectV3(vec3(0, -1, 0), rot90z.multiply3x3Transposed(vec3(1, 0, 0)), 1e-6);
}

test "Mat44.transposed swaps rows and columns of the full 4x4, including the translation row" {
    const t = rot90z.transposed();
    const expected = [16]f32{
        0, -1, 0, 5,
        1, 0,  0, 6,
        0, 0,  1, 7,
        0, 0,  0, 1,
    };
    for (0..16) |i| try std.testing.expectApproxEqAbs(expected[i], t.m[i], 1e-6);
}

test "Mat44.transposed3x3 transposes only the rotation and clears the rest" {
    const t3 = rot90z.transposed3x3();
    try expectV3(vec3(0, -1, 0), t3.axisX(), 1e-6);
    try expectV3(vec3(1, 0, 0), t3.axisY(), 1e-6);
    try expectV3(vec3(0, 0, 1), t3.axisZ(), 1e-6);
    try expectV3(vec3(0, 0, 0), t3.column3(3), 1e-6);
}

test "Mat44.inversed3x3 equals the transpose for an orthonormal rotation" {
    const inv = rot90z.inversed3x3();
    const t3 = rot90z.transposed3x3();
    for (0..16) |i| try std.testing.expectApproxEqAbs(t3.m[i], inv.m[i], 1e-5);
}

test "Mat44.rotation retains row 3, rotationSafe clears it" {
    const m2: Mat44 = .{ .m = .{
        1, 0, 0, 9,
        0, 1, 0, 8,
        0, 0, 1, 7,
        2, 3, 4, 1,
    } };
    const rotation = m2.rotation();
    const expected_rotation = [16]f32{ 1, 0, 0, 9, 0, 1, 0, 8, 0, 0, 1, 7, 0, 0, 0, 1 };
    for (0..16) |i| try std.testing.expectApproxEqAbs(expected_rotation[i], rotation.m[i], 1e-6);

    const safe = m2.rotationSafe();
    for (0..16) |i| {
        const expected: f32 = if (i % 5 == 0) 1 else 0;
        try std.testing.expectApproxEqAbs(expected, safe.m[i], 1e-6);
    }
}

//=============================================================================
// RMat44 twins: same fixture, `Real`-typed. Compiles and passes identically
// under `-Ddouble_precision=true` since every literal here is untyped and
// adapts to whatever `Real` resolves to.
//=============================================================================

const rrot90z: RMat44 = .{ .m = .{
    0,  1, 0, 0,
    -1, 0, 0, 0,
    0,  0, 1, 0,
    5,  6, 7, 1,
} };

test "rmat44 axis, diagonal and column accessors narrow to f32 like DMat44's do" {
    try expectV3(vec3(0, 1, 0), rrot90z.axisX(), 1e-6);
    try expectV3(vec3(-1, 0, 0), rrot90z.axisY(), 1e-6);
    try expectV3(vec3(0, 0, 1), rrot90z.axisZ(), 1e-6);
    try expectV3(vec3(0, 0, 1), rrot90z.diagonal3(), 1e-6);
    try expectV3(vec3(0, 1, 0), rrot90z.column3(0), 1e-6);

    const with_axis = rrot90z.withAxisZ(vec3(3, 4, 5));
    try expectV3(vec3(3, 4, 5), with_axis.axisZ(), 1e-6);
}

test "RMat44.determinant3x3 matches its mat44 twin for the same rotation" {
    try std.testing.expectApproxEqAbs(@as(f32, 1), rrot90z.determinant3x3(), 1e-6);
}

test "RMat44.multiply3x3 and multiply3x3RightTransposed narrow to Mat44 like RMat44.rotation" {
    const composed = rrot90z.multiply3x3(rrot90z);
    try expectV3(vec3(-1, 0, 0), composed.axisX(), 1e-6);
    try expectV3(vec3(0, -1, 0), composed.axisY(), 1e-6);

    const identity_3x3 = rrot90z.multiply3x3RightTransposed(rrot90z);
    try expectV3(vec3(1, 0, 0), identity_3x3.axisX(), 1e-6);

    try expectV3(vec3(0, -1, 0), rrot90z.multiply3x3Transposed(vec3(1, 0, 0)), 1e-6);
}

test "RMat44.transposed3x3 and inversed3x3 agree, transposed stays wide" {
    const t3 = rrot90z.transposed3x3();
    const inv = rrot90z.inversed3x3();
    for (0..16) |i| try std.testing.expectApproxEqAbs(t3.m[i], inv.m[i], 1e-5);

    const full_t = rrot90z.transposed();
    const expected = [16]f32{
        0, -1, 0, 5,
        1, 0,  0, 6,
        0, 0,  1, 7,
        0, 0,  0, 1,
    };
    for (0..16) |i| try std.testing.expectApproxEqAbs(expected[i], @as(f32, @floatCast(full_t.m[i])), 1e-6);
}

test "RMat44.rotation/rotationSafe and isClose match their mat44 twins' semantics" {
    const rm2: RMat44 = .{ .m = .{
        1, 0, 0, 9,
        0, 1, 0, 8,
        0, 0, 1, 7,
        2, 3, 4, 1,
    } };
    const safe = rm2.rotationSafe();
    for (0..16) |i| {
        const expected: f32 = if (i % 5 == 0) 1 else 0;
        try std.testing.expectApproxEqAbs(expected, safe.m[i], 1e-6);
    }

    var nudged = rrot90z;
    nudged.m[12] += 0.1;
    try std.testing.expect(rrot90z.isClose(nudged, 0.02));
    try std.testing.expect(!rrot90z.isClose(nudged, 0.005));
}

test "RMat44.withDiagonal3/4 and withColumn3/4 round trip through the getters" {
    const m = rrot90z.withDiagonal4(.{ 2, 3, 4, 5 });
    const diag = m.diagonal4();
    try std.testing.expectApproxEqAbs(@as(f32, 2), diag[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 5), diag[3], 1e-6);

    const m2 = rrot90z.withColumn3(1, vec3(11, 12, 13));
    try expectV3(vec3(11, 12, 13), m2.column3(1), 1e-6);
}

//=============================================================================
// AABox
//=============================================================================

const test_box: AABox = .{ .min = vec3(-1, -2, -3), .max = vec3(4, 5, 6) };

test "AABox.encapsulatePoint and encapsulateBox grow the box to fit" {
    const grown = test_box.encapsulatePoint(vec3(10, -10, 0));
    try expectV3(vec3(-1, -10, -3), grown.min, 1e-6);
    try expectV3(vec3(10, 5, 6), grown.max, 1e-6);

    const other: AABox = .{ .min = vec3(-5, 0, 0), .max = vec3(0, 10, 10) };
    const merged = test_box.encapsulateBox(other);
    try expectV3(vec3(-5, -2, -3), merged.min, 1e-6);
    try expectV3(vec3(4, 10, 10), merged.max, 1e-6);
}

test "AABox.intersect, overlaps agree on an overlapping and a disjoint pair" {
    const other: AABox = .{ .min = vec3(0, 0, 0), .max = vec3(10, 10, 10) };
    const overlap = test_box.intersect(other);
    try expectV3(vec3(0, 0, 0), overlap.min, 1e-6);
    try expectV3(vec3(4, 5, 6), overlap.max, 1e-6);
    try std.testing.expect(test_box.overlaps(other));

    const far: AABox = .{ .min = vec3(100, 100, 100), .max = vec3(200, 200, 200) };
    try std.testing.expect(!test_box.overlaps(far));
}

test "AABox.support, extent, closestPoint and sqDistanceTo match hand computation" {
    try expectV3(vec3(4, -2, 6), test_box.support(vec3(1, -1, 1)), 1e-6);
    try expectV3(vec3(2.5, 3.5, 4.5), test_box.extent(), 1e-6);

    const closest = test_box.closestPoint(vec3(100, -100, 2));
    try expectV3(vec3(4, -2, 2), closest, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 18820), test_box.sqDistanceTo(vec3(100, -100, 2)), 1e-2);
}

//=============================================================================
// AABox4 / RayAABox4 / RayTriangle4 — four boxes: [0,1]^3, [10,11]^3,
// [-5,-4]^3, [2,3]^3.
//=============================================================================

fn fourBoxes() zjolt.AABox4 {
    return .{
        .min = .{ .x = .{ 0, 10, -5, 2 }, .y = .{ 0, 10, -5, 2 }, .z = .{ 0, 10, -5, 2 } },
        .max = .{ .x = .{ 1, 11, -4, 3 }, .y = .{ 1, 11, -4, 3 }, .z = .{ 1, 11, -4, 3 } },
    };
}

test "AABox4.vsBox and vsPoint pick out only the matching lane" {
    const boxes = fourBoxes();
    const small: AABox = .{ .min = vec3(0.4, 0.4, 0.4), .max = vec3(0.6, 0.6, 0.6) };
    const vs_box = zjolt.AABox4.vsBox(small, boxes);
    try std.testing.expectEqual([4]bool{ true, false, false, false }, @as([4]bool, vs_box));

    const vs_point = zjolt.AABox4.vsPoint(vec3(-4.5, -4.5, -4.5), boxes);
    try std.testing.expectEqual([4]bool{ false, false, true, false }, @as([4]bool, vs_point));
}

test "AABox4.scale scales positively and swaps min/max under a negative scale" {
    const boxes = fourBoxes();
    const scaled = zjolt.AABox4.scale(vec3(2, 2, 2), boxes);
    try std.testing.expectApproxEqAbs(@as(f32, 0), scaled.min.x[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 2), scaled.max.x[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 20), scaled.min.x[1], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 22), scaled.max.x[1], 1e-6);

    const flipped = zjolt.AABox4.scale(vec3(-1, -1, -1), boxes);
    try std.testing.expectApproxEqAbs(@as(f32, -1), flipped.min.x[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), flipped.max.x[0], 1e-6);
}

test "AABox4.enlargeWithExtent widens every lane on both sides" {
    const boxes = fourBoxes();
    const enlarged = zjolt.AABox4.enlargeWithExtent(vec3(1, 1, 1), boxes);
    try std.testing.expectApproxEqAbs(@as(f32, -1), enlarged.min.x[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 2), enlarged.max.x[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 9), enlarged.min.x[1], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 12), enlarged.max.x[1], 1e-6);
}

test "RayInvDirection.set treats a near-zero component as parallel" {
    const inv = zjolt.RayInvDirection.set(vec3(1, 0, 0));
    try std.testing.expect(!inv.is_parallel[0]);
    try std.testing.expect(inv.is_parallel[1]);
    try std.testing.expect(inv.is_parallel[2]);
    try std.testing.expectApproxEqAbs(@as(f32, 1), inv.inv_direction.x, 1e-6);
}

test "ray.aabox4 hits the in-range lane and misses the lanes the ray's y misses entirely" {
    const boxes = fourBoxes();
    const origin = vec3(-5, 0.5, 0.5);
    const inv = zjolt.RayInvDirection.set(vec3(1, 0, 0));
    const result = zjolt.ray.aabox4(origin, inv, boxes);

    const no_hit = std.math.floatMax(f32);
    try std.testing.expectApproxEqAbs(@as(f32, 5), result[0], 1e-5);
    try std.testing.expectEqual(no_hit, result[1]);
    try std.testing.expectEqual(no_hit, result[2]);
    try std.testing.expectEqual(no_hit, result[3]);
}

test "ray.triangle4 hits the two in-range lanes at the right fraction and misses the rest" {
    const v0: zjolt.Vec3x4 = .{ .x = .{ 0, 10, 0, 0 }, .y = .{ 0, 10, 0, 5 }, .z = .{ 1, 1, 2, 1 } };
    const v1: zjolt.Vec3x4 = .{ .x = .{ 1, 11, 1, 1 }, .y = .{ 0, 10, 0, 5 }, .z = .{ 1, 1, 2, 1 } };
    const v2: zjolt.Vec3x4 = .{ .x = .{ 0, 10, 0, 0 }, .y = .{ 1, 11, 1, 6 }, .z = .{ 1, 1, 2, 1 } };

    const result = zjolt.ray.triangle4(vec3(0.2, 0.2, 0), vec3(0, 0, 1), v0, v1, v2);
    const no_hit = std.math.floatMax(f32);
    try std.testing.expectApproxEqAbs(@as(f32, 1), result[0], 1e-5);
    try std.testing.expectEqual(no_hit, result[1]);
    try std.testing.expectApproxEqAbs(@as(f32, 2), result[2], 1e-5);
    try std.testing.expectEqual(no_hit, result[3]);
}

//=============================================================================
// pointOnRay / Plane / Triangle / IndexedTriangle
//=============================================================================

test "ray.pointOnRay and rpointOnRay are origin + fraction * direction" {
    try expectV3(vec3(2, 4, 6), zjolt.ray.pointOnRay(vec3(0, 0, 0), vec3(2, 4, 6), 1), 1e-6);
    try expectV3(vec3(1, 2, 3), zjolt.ray.pointOnRay(vec3(0, 0, 0), vec3(2, 4, 6), 0.5), 1e-6);

    const rp = zjolt.ray.rpointOnRay(rvec3(10, 10, 10), vec3(2, 4, 6), 0.5);
    try std.testing.expectApproxEqAbs(@as(Real, 11), rp.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(Real, 12), rp.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(Real, 13), rp.z, 1e-5);
}

test "planeSignedDistance matches point.Dot(normal) + constant" {
    const p: zjolt.Plane = .{ .normal = vec3(0, 1, 0), .constant = -5 }; // plane y = 5
    try std.testing.expectApproxEqAbs(@as(f32, 5), zjolt.planeSignedDistance(p, vec3(0, 10, 0)), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, -5), zjolt.planeSignedDistance(p, vec3(0, 0, 0)), 1e-6);
}

test "triangleCentroid and IndexedTriangle.centroid agree on the same triangle" {
    try expectV3(vec3(1, 1, 0), zjolt.triangleCentroid(vec3(0, 0, 0), vec3(3, 0, 0), vec3(0, 3, 0)), 1e-6);

    const vertices = [_]Vec3{ vec3(0, 0, 0), vec3(3, 0, 0), vec3(0, 3, 0) };
    const t: zjolt.IndexedTriangle = .{ .idx = .{ 0, 1, 2 } };
    try expectV3(vec3(1, 1, 0), t.centroid(&vertices), 1e-6);
}

test "IndexedTriangle.isDegenerate detects a collinear triangle and accepts a proper one" {
    const collinear = [_]Vec3{ vec3(0, 0, 0), vec3(1, 0, 0), vec3(2, 0, 0) };
    const collinear_t: zjolt.IndexedTriangle = .{ .idx = .{ 0, 1, 2 } };
    try std.testing.expect(collinear_t.isDegenerate(&collinear));

    const proper = [_]Vec3{ vec3(0, 0, 0), vec3(1, 0, 0), vec3(0, 1, 0) };
    const proper_t: zjolt.IndexedTriangle = .{ .idx = .{ 0, 1, 2 } };
    try std.testing.expect(!proper_t.isDegenerate(&proper));
}

test "IndexedTriangle.isEquivalent accepts a rotation of vertex order and rejects a reversal" {
    const a: zjolt.IndexedTriangle = .{ .idx = .{ 0, 1, 2 } };
    const rotated: zjolt.IndexedTriangle = .{ .idx = .{ 1, 2, 0 } };
    const reversed: zjolt.IndexedTriangle = .{ .idx = .{ 2, 1, 0 } };
    try std.testing.expect(a.isEquivalent(rotated));
    try std.testing.expect(!a.isEquivalent(reversed));
}

test "IndexedTriangle.lowestIndexFirst rotates to whichever index is smallest, preserving material and user data" {
    const a: zjolt.IndexedTriangle = .{ .idx = .{ 5, 2, 8 }, .material_index = 3, .user_data = 7 };
    const a_rotated = a.lowestIndexFirst();
    try std.testing.expectEqual([3]u32{ 2, 8, 5 }, a_rotated.idx);
    try std.testing.expectEqual(@as(u32, 3), a_rotated.material_index);
    try std.testing.expectEqual(@as(u32, 7), a_rotated.user_data);

    const b: zjolt.IndexedTriangle = .{ .idx = .{ 1, 5, 3 } };
    try std.testing.expectEqual([3]u32{ 1, 5, 3 }, b.lowestIndexFirst().idx);

    const c: zjolt.IndexedTriangle = .{ .idx = .{ 5, 8, 2 } };
    try std.testing.expectEqual([3]u32{ 2, 5, 8 }, c.lowestIndexFirst().idx);

    const d: zjolt.IndexedTriangle = .{ .idx = .{ 9, 7, 3 } };
    try std.testing.expectEqual([3]u32{ 3, 9, 7 }, d.lowestIndexFirst().idx);
}

//=============================================================================
// Deterministic trigonometry — hand-derived reference values (standard
// trig identities, not `std.math`). `trig_half_sqrt2`/`trig_half_sqrt3` etc. are the
// same style of literal `Quat.rotateAxisX` above uses.
//=============================================================================

const trig = math_mod.trig;
const trig_half_sqrt2: f32 = 0.70710678;
const trig_half_sqrt3: f32 = 0.86602540;
const sqrt3: f32 = 1.73205081;
const pi: f32 = std.math.pi;

test "trig.sin/cos/tan match standard angles" {
    try std.testing.expectApproxEqAbs(@as(f32, 0), trig.sin(0), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), trig.sin(pi / 6.0), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), trig.sin(pi / 2.0), 1e-6);

    try std.testing.expectApproxEqAbs(@as(f32, 1), trig.cos(0), 1e-6);
    try std.testing.expectApproxEqAbs(trig_half_sqrt3, trig.cos(pi / 6.0), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, -1), trig.cos(pi), 1e-5);

    try std.testing.expectApproxEqAbs(@as(f32, 0), trig.tan(0), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), trig.tan(pi / 4.0), 1e-6);
    try std.testing.expectApproxEqAbs(sqrt3, trig.tan(pi / 3.0), 1e-5);
}

test "trig.asin/acos hit their domain edges exactly, matching Vec4::ASin's exact-at-1 branch" {
    try std.testing.expectApproxEqAbs(-pi / 2.0, trig.asin(-1), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), trig.asin(0), 1e-6);
    try std.testing.expectApproxEqAbs(pi / 6.0, trig.asin(0.5), 1e-5);
    try std.testing.expectApproxEqAbs(pi / 2.0, trig.asin(1), 1e-6);

    try std.testing.expectApproxEqAbs(pi, trig.acos(-1), 1e-6);
    try std.testing.expectApproxEqAbs(pi / 2.0, trig.acos(0), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), trig.acos(1), 1e-6);
}

test "trig.atan/atan2 match standard angles in every quadrant" {
    try std.testing.expectApproxEqAbs(@as(f32, 0), trig.atan(0), 1e-6);
    try std.testing.expectApproxEqAbs(pi / 4.0, trig.atan(1), 1e-6);
    try std.testing.expectApproxEqAbs(pi / 3.0, trig.atan(sqrt3), 1e-5);

    try std.testing.expectApproxEqAbs(pi / 4.0, trig.atan2(1, 1), 1e-6);
    try std.testing.expectApproxEqAbs(3.0 * pi / 4.0, trig.atan2(1, -1), 1e-6);
    try std.testing.expectApproxEqAbs(-3.0 * pi / 4.0, trig.atan2(-1, -1), 1e-6);
    try std.testing.expectApproxEqAbs(-pi / 4.0, trig.atan2(-1, 1), 1e-6);

    // (0, 0): both the numerator and denominator Jolt's own sATan2 divides
    // are zero, so this is NaN — not a domain refusal, Jolt's C++ itself
    // produces NaN here (see Jolt/Math/Vec4.inl's sATan2).
    try std.testing.expect(std.math.isNan(trig.atan2(0, 0)));
}

test "trig.sinCos matches sin/cos individually and stays exact at pi/2" {
    const sc = trig.sinCos(pi / 3.0);
    try std.testing.expectApproxEqAbs(trig_half_sqrt3, sc.sin, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), sc.cos, 1e-6);

    const sc2 = trig.sinCos(pi / 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1), sc2.sin, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), sc2.cos, 1e-6);
}

test "trig.acosApproximate stays within its documented 4.2e-3 error bound" {
    // True acos values, not std.math.acos and not zjolt's own precise ACos.
    const cases = [_]struct { x: f32, acos_x: f32 }{
        .{ .x = 1, .acos_x = 0 },
        .{ .x = 0.5, .acos_x = pi / 3.0 },
        .{ .x = 0, .acos_x = pi / 2.0 },
        .{ .x = -0.5, .acos_x = 2.0 * pi / 3.0 },
        .{ .x = -1, .acos_x = pi },
    };
    for (cases) |c| {
        try std.testing.expectApproxEqAbs(c.acos_x, trig.acosApproximate(c.x), 4.2e-3);
    }
}

test "trig.sinCos4/tan4/asin4/acos4/atan4/atan24 agree with the scalar forms lane by lane" {
    const angles = [4]f32{ 0, pi / 6.0, pi / 4.0, pi / 2.0 };
    const sc4 = trig.sinCos4(angles);
    const expected_sin = [4]f32{ 0, 0.5, trig_half_sqrt2, 1 };
    const expected_cos = [4]f32{ 1, trig_half_sqrt3, trig_half_sqrt2, 0 };
    for (0..4) |i| {
        try std.testing.expectApproxEqAbs(expected_sin[i], sc4.sin[i], 1e-5);
        try std.testing.expectApproxEqAbs(expected_cos[i], sc4.cos[i], 1e-5);
    }

    const tan4 = trig.tan4([4]f32{ 0, pi / 4.0, pi / 3.0, -pi / 4.0 });
    const expected_tan = [4]f32{ 0, 1, sqrt3, -1 };
    for (0..4) |i| try std.testing.expectApproxEqAbs(expected_tan[i], tan4[i], 1e-4);

    const asin4 = trig.asin4([4]f32{ 0, 0.5, 1, -1 });
    const expected_asin = [4]f32{ 0, pi / 6.0, pi / 2.0, -pi / 2.0 };
    for (0..4) |i| try std.testing.expectApproxEqAbs(expected_asin[i], asin4[i], 1e-5);

    const acos4 = trig.acos4([4]f32{ 1, 0.5, 0, -1 });
    const expected_acos = [4]f32{ 0, pi / 3.0, pi / 2.0, pi };
    for (0..4) |i| try std.testing.expectApproxEqAbs(expected_acos[i], acos4[i], 1e-5);

    const atan4 = trig.atan4([4]f32{ 0, 1, -1, sqrt3 });
    const expected_atan = [4]f32{ 0, pi / 4.0, -pi / 4.0, pi / 3.0 };
    for (0..4) |i| try std.testing.expectApproxEqAbs(expected_atan[i], atan4[i], 1e-5);

    const atan24 = trig.atan24([4]f32{ 0, 1, 1, -1 }, [4]f32{ 1, 1, -1, -1 });
    const expected_atan2 = [4]f32{ 0, pi / 4.0, 3.0 * pi / 4.0, -3.0 * pi / 4.0 };
    for (0..4) |i| try std.testing.expectApproxEqAbs(expected_atan2[i], atan24[i], 1e-5);
}

//=============================================================================
// Half floats — bit patterns pinned against IEEE-754 binary16 facts and
// Jolt's own named constants (HalfFloat.h: HALF_FLT_MAX = 0x7bff,
// HALF_FLT_INF = 0x7c00, HALF_FLT_NANQ = 0x7e00), not re-derived from this
// file's own conversion.
//=============================================================================

const half_float = math_mod.half_float;

test "half_float.fromFloat pins zero, the smallest subnormal, the largest finite value, infinity and NaN" {
    const modes = [_]half_float.RoundingMode{ .round_to_neg_inf, .round_to_pos_inf, .round_to_nearest };

    // Zero: underflows unconditionally regardless of rounding mode.
    for (modes) |mode| {
        try std.testing.expectEqual(@as(u16, 0x0000), half_float.fromFloat(0.0, mode));
        try std.testing.expectEqual(@as(u16, 0x8000), half_float.fromFloat(-0.0, mode));
    }

    // Smallest positive subnormal: half bit pattern 0x0001 represents
    // exactly 2^-24, which round-trips exactly at ROUND_TO_NEAREST.
    try std.testing.expectEqual(@as(u16, 0x0001), half_float.fromFloat(0x1p-24, .round_to_nearest));

    // Largest finite value: HALF_FLT_MAX, 65504.0, exactly representable in
    // both float and half.
    try std.testing.expectEqual(@as(u16, 0x7bff), half_float.fromFloat(65504.0, .round_to_nearest));
    try std.testing.expectEqual(@as(u16, 0xfbff), half_float.fromFloat(-65504.0, .round_to_nearest));

    // Infinity: HALF_FLT_INF, independent of rounding mode (exponent all
    // ones, mantissa zero is the unconditional-infinity branch).
    for (modes) |mode| {
        try std.testing.expectEqual(@as(u16, 0x7c00), half_float.fromFloat(std.math.inf(f32), mode));
        try std.testing.expectEqual(@as(u16, 0xfc00), half_float.fromFloat(-std.math.inf(f32), mode));
    }

    // NaN: HALF_FLT_NANQ, independent of rounding mode and of the input
    // NaN's own payload (only "mantissa != 0" is checked).
    for (modes) |mode| {
        try std.testing.expectEqual(@as(u16, 0x7e00), half_float.fromFloat(std.math.nan(f32), mode));
    }
}

test "half_float.fromFloatFallback matches fromFloat's pinned bits" {
    try std.testing.expectEqual(@as(u16, 0x0000), half_float.fromFloatFallback(0.0, .round_to_nearest));
    try std.testing.expectEqual(@as(u16, 0x0001), half_float.fromFloatFallback(0x1p-24, .round_to_nearest));
    try std.testing.expectEqual(@as(u16, 0x7bff), half_float.fromFloatFallback(65504.0, .round_to_nearest));
    try std.testing.expectEqual(@as(u16, 0x7c00), half_float.fromFloatFallback(std.math.inf(f32), .round_to_nearest));
    try std.testing.expectEqual(@as(u16, 0x7e00), half_float.fromFloatFallback(std.math.nan(f32), .round_to_nearest));
}

test "half_float.toFloat4/toFloatFallback4 invert the same five pinned bit patterns" {
    // 0x3c00 is IEEE-754 binary16's exact bit pattern for 1.0 — a
    // well-known constant, not derived from this conversion.
    const in = [4]u16{ 0x0000, 0x0001, 0x3c00, 0x7bff };
    const out = half_float.toFloat4(in);
    try std.testing.expectApproxEqAbs(@as(f32, 0), out[0], 0);
    try std.testing.expectApproxEqAbs(@as(f32, 0x1p-24), out[1], 0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), out[2], 0);
    try std.testing.expectApproxEqAbs(@as(f32, 65504.0), out[3], 0);

    const inf_nan = [4]u16{ 0x7c00, 0xfc00, 0x7e00, 0x8000 };
    const out2 = half_float.toFloat4(inf_nan);
    try std.testing.expect(std.math.isPositiveInf(out2[0]));
    try std.testing.expect(std.math.isNegativeInf(out2[1]));
    try std.testing.expect(std.math.isNan(out2[2]));
    try std.testing.expectEqual(@as(f32, 0), out2[3]);

    const out_fallback = half_float.toFloatFallback4(in);
    for (0..4) |i| try std.testing.expectEqual(out[i], out_fallback[i]);
}

//=============================================================================
// FindRoot / CenterAngleAroundZero
//=============================================================================

test "findRoot solves a quadratic with two known roots, matching Jolt's own q-substitution order" {
    // x^2 - 5x + 6 = 0: det = 1, q = (b + sign(b)*sqrt(det)) / -2 = 3,
    // x1 = q/a = 3, x2 = c/q = 2 — Jolt's own order, not sorted.
    const r = math_mod.findRoot(1, -5, 6);
    try std.testing.expectEqual(@as(u32, 2), r.count);
    try std.testing.expectApproxEqAbs(@as(f32, 3), r.x1, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 2), r.x2, 1e-5);
}

test "findRoot reports zero roots for a quadratic with a negative discriminant" {
    // x^2 + 1 = 0: det = 0 - 4 = -4 < 0.
    const r = math_mod.findRoot(1, 0, 1);
    try std.testing.expectEqual(@as(u32, 0), r.count);
}

test "findRoot solves the degenerate linear case" {
    // 2x - 4 = 0 -> x = 2, reported as a single root (x1 == x2).
    const r = math_mod.findRoot(0, 2, -4);
    try std.testing.expectEqual(@as(u32, 1), r.count);
    try std.testing.expectApproxEqAbs(@as(f32, 2), r.x1, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 2), r.x2, 1e-5);
}

test "centerAngleAroundZero wraps whole turns into [-pi, pi] and leaves an in-range angle alone" {
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), math_mod.centerAngleAroundZero(0.5), 1e-6);
    try std.testing.expectApproxEqAbs(pi, math_mod.centerAngleAroundZero(3.0 * pi), 1e-5);
    try std.testing.expectApproxEqAbs(-pi, math_mod.centerAngleAroundZero(-3.0 * pi), 1e-5);
}

//=============================================================================
// Dense linear algebra — Matrix2. Fixtures: A rows [1,2;3,4] (flat
// column-major m = [1,3,2,4]), B rows [5,6;7,8] (m = [5,7,6,8]).
//=============================================================================

const Matrix2 = math_mod.Matrix2;
const Vector2 = math_mod.Vector2;

const mat2_a: Matrix2 = .{ .m = .{ 1, 3, 2, 4 } };
const mat2_b: Matrix2 = .{ .m = .{ 5, 7, 6, 8 } };

fn expectM2(expected: [4]f32, actual: Matrix2, tol: f32) !void {
    for (0..4) |i| try std.testing.expectApproxEqAbs(expected[i], actual.m[i], tol);
}

test "Matrix2.zero/identity/diagonal/rows/cols/isIdentity match their construction" {
    try expectM2(.{ 0, 0, 0, 0 }, Matrix2.zero(), 0);
    try expectM2(.{ 1, 0, 0, 1 }, Matrix2.identity(), 0);
    try expectM2(.{ 2, 0, 0, 3 }, Matrix2.diagonal(.{ .x = 2, .y = 3 }), 0);

    try std.testing.expectEqual(@as(u32, 2), Matrix2.rows());
    try std.testing.expectEqual(@as(u32, 2), Matrix2.cols());

    try std.testing.expect(Matrix2.identity().isIdentity());
    try std.testing.expect(!mat2_a.isIdentity());
}

test "Matrix2.diagonalOf/isClose/transposed read the A fixture correctly" {
    const diag = mat2_a.diagonalOf();
    try std.testing.expectApproxEqAbs(@as(f32, 1), diag.x, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 4), diag.y, 1e-6);

    try std.testing.expect(Matrix2.isClose(mat2_a, mat2_a, 1e-12));
    try std.testing.expect(!Matrix2.isClose(mat2_a, mat2_b, 1e-12));

    // A = [[1,2],[3,4]]; transposed = [[1,3],[2,4]] (row i <-> column i),
    // whose column-major flat form (m[2*c+r] = transposed(r,c)) is [1,2,3,4].
    try expectM2(.{ 1, 2, 3, 4 }, mat2_a.transposed(), 1e-6);
}

test "Matrix2.multiply/multiplyVector/multiplyScalar/add/subtract match hand-computed A, B" {
    // A*B: row0=[1,2]·[5,7]/[6,8], row1=[3,4]·[5,7]/[6,8].
    try expectM2(.{ 19, 43, 22, 50 }, mat2_a.multiply(mat2_b), 1e-4);
    try expectM2(.{ 6, 10, 8, 12 }, mat2_a.add(mat2_b), 1e-6);
    try expectM2(.{ -4, -4, -4, -4 }, mat2_a.subtract(mat2_b), 1e-6);
    try expectM2(.{ 2, 6, 4, 8 }, mat2_a.multiplyScalar(2), 1e-6);

    const v = mat2_a.multiplyVector(.{ .x = 1, .y = 1 });
    try std.testing.expectApproxEqAbs(@as(f32, 3), v.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 7), v.y, 1e-5);
}

test "Matrix2.column/withColumn/inversed/solve succeed on well-posed input and fail on ill-posed input" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const col1 = try mat2_a.column(1);
    try std.testing.expectApproxEqAbs(@as(f32, 2), col1.x, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 4), col1.y, 1e-6);
    try std.testing.expectError(err.Error.InvalidArgument, mat2_a.column(2));

    const replaced = try Matrix2.zero().withColumn(0, .{ .x = 5, .y = 6 });
    try expectM2(.{ 5, 6, 0, 0 }, replaced, 1e-6);
    try std.testing.expectError(err.Error.InvalidArgument, Matrix2.zero().withColumn(2, .{ .x = 0, .y = 0 }));

    // diag(2,4)^-1 = diag(0.5,0.25) — a diagonal matrix, but 2x2 still runs
    // through Matrix<2,2>'s own closed-form SetInversed, not GaussianElimination.
    const inv = try Matrix2.diagonal(.{ .x = 2, .y = 4 }).inversed();
    try expectM2(.{ 0.5, 0, 0, 0.25 }, inv, 1e-5);

    const singular: Matrix2 = .{ .m = .{ 1, 2, 2, 4 } }; // rows [1,2],[2,4]: det 0
    try std.testing.expectError(err.Error.InvalidArgument, singular.inversed());

    // 2x + y = 5, x + 3y = 10 -> x = 1, y = 3.
    const system: Matrix2 = .{ .m = .{ 2, 1, 1, 3 } };
    const x = try Matrix2.solve(system, .{ .x = 5, .y = 10 }, 1.0e-16);
    try std.testing.expectApproxEqAbs(@as(f32, 1), x.x, 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 3), x.y, 1e-4);

    try std.testing.expectError(err.Error.InvalidArgument, Matrix2.solve(singular, .{ .x = 1, .y = 1 }, 1.0e-16));
}

//=============================================================================
// Dense linear algebra — Matrix3. Fixture M: flat column-major
// m = [1,2,3,4,5,6,7,8,9] (columns (1,2,3), (4,5,6), (7,8,9)).
//=============================================================================

const Matrix3 = math_mod.Matrix3;

const mat3_m: Matrix3 = .{ .m = .{ 1, 2, 3, 4, 5, 6, 7, 8, 9 } };

fn expectM3(expected: [9]f32, actual: Matrix3, tol: f32) !void {
    for (0..9) |i| try std.testing.expectApproxEqAbs(expected[i], actual.m[i], tol);
}

test "Matrix3.zero/identity/diagonal/rows/cols/isIdentity match their construction" {
    try expectM3(.{ 0, 0, 0, 0, 0, 0, 0, 0, 0 }, Matrix3.zero(), 0);
    try expectM3(.{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }, Matrix3.identity(), 0);
    try expectM3(.{ 2, 0, 0, 0, 3, 0, 0, 0, 4 }, Matrix3.diagonal(vec3(2, 3, 4)), 0);

    try std.testing.expectEqual(@as(u32, 3), Matrix3.rows());
    try std.testing.expectEqual(@as(u32, 3), Matrix3.cols());
    try std.testing.expect(Matrix3.identity().isIdentity());
    try std.testing.expect(!mat3_m.isIdentity());
}

test "Matrix3.diagonalOf/isClose/transposed read the M fixture correctly" {
    try expectV3(vec3(1, 5, 9), mat3_m.diagonalOf(), 1e-6);

    try std.testing.expect(Matrix3.isClose(mat3_m, mat3_m, 1e-12));
    var nudged = mat3_m;
    nudged.m[0] += 1;
    try std.testing.expect(!Matrix3.isClose(mat3_m, nudged, 1e-12));

    // M's flat storage already IS its own transpose's mirror image: swapping
    // (m[1],m[3]), (m[2],m[6]) and (m[5],m[7]) while the diagonal stays put.
    try expectM3(.{ 1, 4, 7, 2, 5, 8, 3, 6, 9 }, mat3_m.transposed(), 1e-6);
}

test "Matrix3.multiply/multiplyVector/multiplyScalar/add/subtract match hand-computed diagonals" {
    const d1 = Matrix3.diagonal(vec3(2, 3, 4));
    const d2 = Matrix3.diagonal(vec3(5, 6, 7));

    try expectM3(.{ 10, 0, 0, 0, 18, 0, 0, 0, 28 }, d1.multiply(d2), 1e-4);
    try expectM3(.{ 3, 0, 0, 0, 4, 0, 0, 0, 5 }, d1.add(Matrix3.identity()), 1e-6);
    try expectM3(.{ 1, 0, 0, 0, 2, 0, 0, 0, 3 }, d1.subtract(Matrix3.identity()), 1e-6);
    try expectM3(.{ 4, 0, 0, 0, 6, 0, 0, 0, 8 }, d1.multiplyScalar(2), 1e-6);

    try expectV3(vec3(2, 3, 4), d1.multiplyVector(vec3(1, 1, 1)), 1e-5);
}

test "Matrix3.column/withColumn/inversed/solve succeed on well-posed input and fail on ill-posed input" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    try expectV3(vec3(1, 2, 3), try mat3_m.column(0), 1e-6);
    try expectV3(vec3(4, 5, 6), try mat3_m.column(1), 1e-6);
    try std.testing.expectError(err.Error.InvalidArgument, mat3_m.column(3));

    const replaced = try Matrix3.zero().withColumn(1, vec3(9, 8, 7));
    try expectM3(.{ 0, 0, 0, 9, 8, 7, 0, 0, 0 }, replaced, 1e-6);
    try std.testing.expectError(err.Error.InvalidArgument, Matrix3.zero().withColumn(3, vec3(0, 0, 0)));

    const inv = try Matrix3.diagonal(vec3(2, 4, 5)).inversed();
    try expectM3(.{ 0.5, 0, 0, 0, 0.25, 0, 0, 0, 0.2 }, inv, 1e-5);

    // A diagonal matrix with one zero entry: every elimination step here
    // divides by an exact power of two, so the zero pivot stays exactly
    // 0.0 rather than a nonzero float rounding residual — unlike a rank-
    // deficient matrix needing a 1/3-style division, which can round past
    // Jolt's permissive 1.0e-16 default tolerance and "succeed" wrongly.
    const singular = Matrix3.diagonal(vec3(2, 4, 0));
    try std.testing.expectError(err.Error.InvalidArgument, singular.inversed());

    // The textbook Gaussian-elimination example (Wikipedia): x+y+z=6,
    // 2y+5z=-4, 2x+5y-z=27 -> (x,y,z) = (5,3,-2).
    const system: Matrix3 = .{ .m = .{ 1, 0, 2, 1, 2, 5, 1, 5, -1 } };
    const x = try Matrix3.solve(system, vec3(6, -4, 27), 1.0e-16);
    try std.testing.expectApproxEqAbs(@as(f32, 5), x.x, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 3), x.y, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, -2), x.z, 1e-3);

    try std.testing.expectError(err.Error.InvalidArgument, Matrix3.solve(singular, vec3(1, 1, 1), 1.0e-16));
}

test "rmat44ToMat44 narrows RMat44's fixture to the equal-valued Mat44" {
    const narrowed = math_mod.rmat44ToMat44(rrot90z);
    for (0..16) |i| try std.testing.expectApproxEqAbs(rot90z.m[i], narrowed.m[i], 1e-6);
}
