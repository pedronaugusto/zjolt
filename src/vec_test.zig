//! The gate under `src/vec.zig`: every operation zjolt computes in Zig,
//! compared against the C entry point that reaches Jolt's own. Two
//! implementations of one formula is an arrangement this package accepts only
//! because of this file, and `tools/zig_native.txt` pairs them.
//!
//! The tolerance is RELATIVE. Bit equality is not the bar and could not be:
//! the C++ side may contract a multiply and an add into one FMA at the
//! compiler's discretion, a legal difference no shipping engine forbids.
//!
//! Measured cost of that: of five planted defects this file catches four.
//! Writing `Vec3.lerp` as `(1-t)*a + t*b` rather than `a + (b-a)*t` passes,
//! the two being the same function to well inside the tolerance. A change to
//! the ANSWER is caught; one to the last bits is not, and is not meant to be.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const c = @import("c/core.zig");
const c_math = @import("c/math.zig");
const err = @import("error.zig");

const tolerance = 1e-6;

fn expectClose(want: f32, got: f32) !void {
    const scale = @max(@abs(want), 1.0);
    try std.testing.expectApproxEqAbs(want, got, tolerance * scale);
}

fn expectVec3(want: c.Vec3, got: c.Vec3) !void {
    try expectClose(want.x, got.x);
    try expectClose(want.y, got.y);
    try expectClose(want.z, got.z);
}

fn expectQuat(want: c.Quat, got: c.Quat) !void {
    try expectClose(want.x, got.x);
    try expectClose(want.y, got.y);
    try expectClose(want.z, got.z);
    try expectClose(want.w, got.w);
}

fn expectMat44(want: c.Mat44, got: c.Mat44) !void {
    for (want.m, got.m) |w, g| try expectClose(w, g);
}

const Sample = struct {
    rng: std.Random.DefaultPrng,

    fn init(seed: u64) Sample {
        return .{ .rng = std.Random.DefaultPrng.init(seed) };
    }

    fn scalar(self: *Sample) f32 {
        return self.rng.random().float(f32) * 4 - 2;
    }

    fn vec3(self: *Sample) c.Vec3 {
        return .{ .x = self.scalar(), .y = self.scalar(), .z = self.scalar() };
    }

    fn quat(self: *Sample) c.Quat {
        return .{ .x = self.scalar(), .y = self.scalar(), .z = self.scalar(), .w = self.scalar() };
    }

    fn unitQuat(self: *Sample) c.Quat {
        const q = self.quat();
        const len = @sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (len < 1e-3) return .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        return .{ .x = q.x / len, .y = q.y / len, .z = q.z / len, .w = q.w / len };
    }

    /// A rotation and a translation and nothing else, which is the only kind
    /// of matrix `inverseRotationTranslation` claims to invert.
    fn rigid(self: *Sample) !c.Mat44 {
        return c.Mat44.fromRotationTranslation(self.unitQuat(), self.vec3());
    }

    fn mat44(self: *Sample) c.Mat44 {
        var m: c.Mat44 = undefined;
        for (&m.m) |*e| e.* = self.scalar();
        return m;
    }
};

const rounds = 2000;

test "Vec3.lerp agrees with zjoltVec3Lerp" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var s = Sample.init(0x11);
    for (0..rounds) |_| {
        const a = s.vec3();
        const b = s.vec3();
        const t = s.scalar();
        var want: c.Vec3 = undefined;
        c_math.zjoltVec3Lerp(&a, &b, t, &want);
        try expectVec3(want, c.Vec3.lerp(a, b, t));
    }
}

test "RVec3.lerp agrees with zjoltRVec3Lerp" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var s = Sample.init(0x12);
    for (0..rounds) |_| {
        const a = c.RVec3{ .x = s.scalar(), .y = s.scalar(), .z = s.scalar() };
        const b = c.RVec3{ .x = s.scalar(), .y = s.scalar(), .z = s.scalar() };
        const t = s.scalar();
        var want: c.RVec3 = undefined;
        c_math.zjoltRVec3Lerp(&a, &b, t, &want);
        const got = c.RVec3.lerp(a, b, t);
        try expectClose(@floatCast(want.x), @floatCast(got.x));
        try expectClose(@floatCast(want.y), @floatCast(got.y));
        try expectClose(@floatCast(want.z), @floatCast(got.z));
    }
}

test "the Quat formulas agree with their entry points" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var s = Sample.init(0x13);
    for (0..rounds) |_| {
        const a = s.quat();
        const b = s.quat();
        const t = s.scalar();

        var want: c.Quat = undefined;
        c_math.zjoltQuatMultiply(&a, &b, &want);
        try expectQuat(want, a.multiply(b));

        c_math.zjoltQuatConjugate(&a, &want);
        try expectQuat(want, a.conjugate());

        c_math.zjoltQuatInverse(&a, &want);
        try expectQuat(want, a.inverse());

        c_math.zjoltQuatNormalize(&a, &want);
        try expectQuat(want, a.normalize());

        c_math.zjoltQuatLerp(&a, &b, t, &want);
        try expectQuat(want, c.Quat.lerp(a, b, t));

        try expectClose(c_math.zjoltQuatDot(&a, &b), a.dot(b));

        // Blind spot, stated rather than gated: the two sides sum a squared
        // length in different orders, so a quaternion sitting within a few
        // ULP of the tolerance could be judged either way. Sampled inputs are
        // never that close; the boundary itself is what the sweep below is
        // for.
        try std.testing.expectEqual(
            c_math.zjoltQuatIsNormalized(&a, 1.0),
            a.isNormalized(1.0),
        );
    }
}

test "Quat.rotateVector agrees with zjoltQuatRotateVector for unit quaternions" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var s = Sample.init(0x14);
    for (0..rounds) |_| {
        const q = s.unitQuat();
        const v = s.vec3();
        var want: c.Vec3 = undefined;
        try err.check(c_math.zjoltQuatRotateVector(&q, &v, &want));
        try expectVec3(want, try q.rotateVector(v));
    }
}

test "the Mat44 formulas agree with their entry points" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var s = Sample.init(0x15);
    for (0..rounds) |_| {
        const a = s.mat44();
        const b = s.mat44();
        const v = s.vec3();

        var want_m: c.Mat44 = undefined;
        c_math.zjoltMat44Multiply(&a, &b, &want_m);
        try expectMat44(want_m, a.multiply(b));

        var want_v: c.Vec3 = undefined;
        c_math.zjoltMat44TransformPoint(&a, &v, &want_v);
        try expectVec3(want_v, a.transformPoint(v));

        c_math.zjoltMat44TransformDirection(&a, &v, &want_v);
        try expectVec3(want_v, a.transformDirection(v));

        const rigid = try s.rigid();
        c_math.zjoltMat44InverseRotationTranslation(&rigid, &want_m);
        try expectMat44(want_m, rigid.inverseRotationTranslation());
    }
}

test "isNormalized draws the line in the same place as the entry point" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // `Quat.rotateVector` computes in Zig only inside a margin and hands
    // everything near the line to the C entry point, so the two can never
    // disagree about a refusal. That is the claim this sweeps: a quaternion
    // the Zig side answers and the C side would have refused is a fast path
    // taking a case Jolt asserts on.
    var s = Sample.init(0x16);
    var accepted_by_c: usize = 0;
    var refused_by_c: usize = 0;
    for (0..rounds) |i| {
        const unit = s.unitQuat();
        // Scales that straddle the tolerance: at i = 0 exactly unit, then
        // outward in steps far finer than 1e-5.
        const drift = @as(f32, @floatFromInt(i)) * 1.0e-6;
        const scale = 1.0 + drift * (if (i % 2 == 0) @as(f32, 1) else -1);
        const q = c.Quat{
            .x = unit.x * scale,
            .y = unit.y * scale,
            .z = unit.z * scale,
            .w = unit.w * scale,
        };
        const v = s.vec3();

        var ignored: c.Vec3 = undefined;
        const c_result = c_math.zjoltQuatRotateVector(&q, &v, &ignored);
        const c_ok = c_result == .ok;
        if (c_ok) accepted_by_c += 1 else refused_by_c += 1;

        const zig_ok = if (q.rotateVector(v)) |_| true else |_| false;
        try std.testing.expectEqual(c_ok, zig_ok);
    }
    // Both arms were actually exercised; a sweep that only ever accepted
    // would pass while proving nothing about the boundary.
    try std.testing.expect(accepted_by_c > 0);
    try std.testing.expect(refused_by_c > 0);
}

test "a refused rotateVector still leaves the C library's own message behind" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const q = c.Quat{ .x = 0, .y = 0, .z = 0, .w = 4 };
    try std.testing.expectError(
        error.InvalidArgument,
        q.rotateVector(.{ .x = 1, .y = 0, .z = 0 }),
    );
    // The refusal went through the entry point rather than being invented in
    // Zig, which is the only reason there is a sentence here at all.
    try std.testing.expect(std.mem.indexOf(u8, zjolt.lastError(), "unit length") != null);
}
