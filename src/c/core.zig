//! ZJolt C declarations for the primitives every other module here is built
//! from: the scalar widths this build chose, results, vectors and matrices, the
//! opaque handles, and library setup.
//!
//! Mirrors `ffi/zjolt_core.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide and
//! nothing to drift. `src/c.zig` lists every one of these and is what the ABI
//! cross-check and the misuse sweep walk.
//!
//! `Vec3`, `Quat`, `RVec3`, `Mat44`, `RMat44`, `AABox` and `MassProperties`
//! carry their own math API as methods, declared directly on these structs so
//! the ABI type and the ergonomic one are the same type. Every method backed by
//! `ffi/zjolt_math.h` documents that there; this file restates only what is
//! Zig-specific.

const std = @import("std");
const options = @import("zjolt_options");
const shape = @import("shape.zig");
const c_math = @import("math.zig");
const err = @import("../error.zig");
const vec = @import("../vec.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const ShapeStats = shape.ShapeStats;

pub const Real = if (options.double_precision) f64 else f32;

pub const ObjectLayer = if (options.object_layer_bits == 32) u32 else u16;

pub const BroadPhaseLayer = u8;

pub const BodyId = u32;

pub const SubShapeId = u32;

pub const body_id_invalid: BodyId = 0xffff_ffff;

pub const sub_shape_id_empty: SubShapeId = 0xffff_ffff;

pub const version_major: u32 = 0;

pub const version_minor: u32 = 2;

pub const version_patch: u32 = 0;

/// The Zig side's view of ZJOLT_CONFIG_ID. Must be computed the same way the
/// header computes it.
pub const config_id: u32 = (version_major << 16) | (version_minor << 8) |
    version_patch |
    (@as(u32, if (options.double_precision) 1 else 0) << 24) |
    (@as(u32, if (options.object_layer_bits == 32) 1 else 0) << 25);

pub const max_physics_jobs: u32 = 2048;

pub const max_physics_barriers: u32 = 8;

pub const build_flag_double_precision: u32 = 1 << 0;

pub const build_flag_object_layer_32: u32 = 1 << 1;

pub const build_flag_asserts_enabled: u32 = 1 << 2;

pub const build_flag_cross_platform_deterministic: u32 = 1 << 3;

pub const Result = enum(c_int) {
    ok = 0,
    not_initialized = 1,
    already_initialized = 2,
    config_mismatch = 3,
    out_of_memory = 4,
    invalid_argument = 5,
    buffer_too_small = 6,
    shape_invalid = 7,
    bad_format = 8,
    body_not_found = 9,
    unsupported = 10,
    io_error = 11,
    /// A save would have written less than the caller asked for — today, a
    /// whole-system state save made while the system holds characters that
    /// the save was not given.
    state_incomplete = 12,
};

/// A host-supplied byte stream. @see ffi/zjolt_core.h's ZJoltStream for what
/// each field means and which may be null.
pub const Stream = extern struct {
    read: ?*const fn (user: ?*anyopaque, data: ?*anyopaque, size: usize) callconv(.c) void = null,
    write: ?*const fn (user: ?*anyopaque, data: ?*const anyopaque, size: usize) callconv(.c) void = null,
    is_eof: ?*const fn (user: ?*anyopaque) callconv(.c) bool = null,
    is_failed: ?*const fn (user: ?*anyopaque) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

/// Which of Jolt's own object-stream formats a save writes. @see
/// ffi/zjolt_core.h's ZJoltObjectStreamFormat.
pub const ObjectStreamFormat = enum(c_int) {
    text = 0,
    binary = 1,
};

//=============================================================================
// The value algebra these methods are built from lives in `src/vec.zig`, one
// home shared with `src/math.zig`. Each method below is either that algebra
// applied here, or a call into the C library — never both, and the choice is
// stated per method.
//
// The rule: an operation that is ONE FORMULA over its inputs is computed
// here, because a cross-TU call for it cannot be inlined and costs more than
// the arithmetic. An operation that follows a Jolt CONVENTION — a
// decomposition, a shortest-arc choice, a transcendental, a narrowing rule —
// stays a call, because agreeing with Jolt is the whole point of it.
// `src/vec_test.zig` proves every formula here against the entry point that
// reaches Jolt, and `tools/zig_native.txt` records the pairing.
//=============================================================================

/// HALF the length tolerance `zjoltQuatRotateVector` applies (`1.0e-5`), and
/// half on purpose: the two sides sum a squared length in different orders,
/// so on the exact line they disagree in the last bits. The fast path takes
/// only what is clearly inside; everything near the line goes to the C entry
/// point, which decides. `src/vec_test.zig` sweeps across it.
const quat_fast_path_tolerance: f32 = 0.5e-5;

/// A direction, velocity or extent. Three floats even in a double-precision
/// build — only positions get the extra range, which is the same split Jolt
/// makes internally.
pub const Vec3 = extern struct {
    x: f32,
    y: f32,
    z: f32,

    /// `a + (b - a) * t`, componentwise.
    pub fn lerp(a: Vec3, b: Vec3, t: f32) Vec3 {
        return vec.to(Vec3, vec.lerp(vec.from(a), vec.from(b), t));
    }

    /// A unit vector perpendicular to `v`, choosing the more numerically
    /// stable of two candidate axes the way Jolt's scalar fallback does.
    pub fn normalizedPerpendicular(v: Vec3) Vec3 {
        const xx = v.x * v.x;
        const yy = v.y * v.y;
        const chosen = if (xx > yy) Vec3{ .x = v.z, .y = 0, .z = -v.x } else Vec3{ .x = 0, .y = v.z, .z = -v.y };
        const len = @sqrt(@max(xx, yy) + v.z * v.z);
        return .{ .x = chosen.x / len, .y = chosen.y / len, .z = chosen.z / len };
    }

    /// Whether `a` and `b` are within `max_dist_sq` of each other. Pass
    /// `1e-12` to match Jolt's own default.
    pub fn isClose(a: Vec3, b: Vec3, max_dist_sq: f32) bool {
        return vec.lengthSq3(vec.from(b) - vec.from(a)) <= max_dist_sq;
    }

    /// Whether `v`'s squared length is within `max_dist_sq` of zero. Pass
    /// `1e-12` to match Jolt's own default.
    pub fn isNearZero(v: Vec3, max_dist_sq: f32) bool {
        return vec.lengthSq3(vec.from(v)) <= max_dist_sq;
    }

    /// Whether `v`'s squared length is within `tolerance` of 1. Pass `1e-6`
    /// to match Jolt's own default.
    pub fn isNormalized(v: Vec3, tolerance: f32) bool {
        return @abs(vec.lengthSq3(vec.from(v)) - 1.0) <= tolerance;
    }

    /// Packs a unit `Vec3` into 32 bits: 1 sign bit, 2 bits for which axis had
    /// the largest magnitude, 14 bits each for the other two (reconstructed
    /// via the unit-length constraint on decompress). ~10^-4 precision,
    /// matching Jolt's own doc comment.
    pub fn compressUnitVector(v: Vec3) u32 {
        const one_over_sqrt2: f32 = 0.70710678;
        const max_value: f32 = 16382; // (1 << 14) - 1 - 1
        const scale: f32 = max_value / (2.0 * one_over_sqrt2);

        const ax = @abs(v.x);
        const ay = @abs(v.y);
        const az = @abs(v.z);
        // Literal port of Vec3::GetHighestComponentIndex's ternary chain: on a
        // tie it does NOT always pick the lowest index (see e.g. x==y==z), so
        // this cannot be simplified to a generic "first max" loop.
        const max_element: u32 = if (ax > ay) (if (az > ax) 2 else 0) else (if (az > ay) 2 else 1);

        const comps = [3]f32{ v.x, v.y, v.z };
        var value: u32 = 0;
        var signed_comps = comps;
        if (comps[max_element] < 0.0) {
            value = 0x8000_0000;
            signed_comps = .{ -comps[0], -comps[1], -comps[2] };
        }
        value |= max_element << 29;

        var packed_comps: [3]u32 = undefined;
        for (0..3) |i| {
            const clamped = std.math.clamp((signed_comps[i] + one_over_sqrt2) * scale + 0.5, 0.0, max_value);
            packed_comps[i] = @intFromFloat(clamped);
        }
        const lo_hi: [2]u32 = switch (max_element) {
            0 => .{ packed_comps[1], packed_comps[2] },
            1 => .{ packed_comps[0], packed_comps[2] },
            else => .{ packed_comps[0], packed_comps[1] },
        };
        value |= lo_hi[0];
        value |= lo_hi[1] << 14;
        return value;
    }

    /// The inverse of `Vec3.compressUnitVector`.
    pub fn decompressUnitVector(value: u32) Vec3 {
        const one_over_sqrt2: f32 = 0.70710678;
        const mask: u32 = (1 << 14) - 1;
        const max_value: f32 = 16382;
        const half_max_value: i32 = 8191;
        const scale: f32 = 2.0 * one_over_sqrt2 / max_value;

        const raw0: i32 = @as(i32, @intCast(value & mask)) - half_max_value;
        const raw1: i32 = @as(i32, @intCast((value >> 14) & mask)) - half_max_value;
        const a: f32 = @as(f32, @floatFromInt(raw0)) * scale;
        const b: f32 = @as(f32, @floatFromInt(raw1)) * scale;
        const recon: f32 = @sqrt(@max(1.0 - (a * a + b * b), 0.0));

        var v = [3]f32{ a, b, recon };
        if ((value & 0x8000_0000) != 0) v = .{ -v[0], -v[1], -v[2] };

        const out: [3]f32 = switch ((value >> 29) & 3) {
            0 => .{ v[2], v[0], v[1] },
            1 => .{ v[0], v[2], v[1] },
            else => v,
        };
        return .{ .x = out[0], .y = out[1], .z = out[2] };
    }

    /// Widens a direction to a position. Lossless in a float build, and an
    /// explicit widening in a double-precision one.
    pub fn toRVec3(v: Vec3) RVec3 {
        return .{ .x = @floatCast(v.x), .y = @floatCast(v.y), .z = @floatCast(v.z) };
    }
};

/// A quaternion in (x, y, z, w) order — w LAST, matching Jolt. Consumers whose
/// own quaternion type is w-first must reorder; there is no silent conversion
/// here to get wrong.
pub const Quat = extern struct {
    x: f32,
    y: f32,
    z: f32,
    w: f32,

    /// Composes two rotations: `lhs * rhs`, Hamilton product order —
    /// rotating a vector by the result matches rotating it by `rhs` first,
    /// then `lhs`.
    pub fn multiply(lhs: Quat, rhs: Quat) Quat {
        return vec.to(Quat, vec.quatMul(vec.from(lhs), vec.from(rhs)));
    }

    /// Rotates `v` by `q`. `error.InvalidArgument` when `q` is not unit
    /// length; see `Quat.normalize`.
    pub fn rotateVector(q: Quat, v: Vec3) err.Error!Vec3 {
        var out: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        if (vec.quatIsNormalized(vec.from(q), quat_fast_path_tolerance)) {
            return vec.to(Vec3, vec.quatRotate(vec.from(q), vec.from(v)));
        }
        // The refusal goes back through the C entry point, which owns the
        // sentence `zjolt.lastError` returns. Writing that sentence again
        // here would be a second home for it, free to disagree.
        try err.check(c_math.zjoltQuatRotateVector(&q, &v, &out));
        return out;
    }

    /// The inverse rotation. NaN on a zero-length `q`, exactly as Jolt's own
    /// `Inversed()` — infallible, not refused.
    pub fn inverse(q: Quat) Quat {
        return vec.to(Quat, vec.quatInverse(vec.from(q)));
    }

    /// `(-x, -y, -z, w)`. Equal to `Quat.inverse` for a unit `q`, and cheaper.
    pub fn conjugate(q: Quat) Quat {
        return vec.to(Quat, vec.quatConjugate(vec.from(q)));
    }

    /// The four-component dot product.
    pub fn dot(a: Quat, b: Quat) f32 {
        return vec.dot4(vec.from(a), vec.from(b));
    }

    /// Whether `q`'s SQUARED length is within `tolerance` of 1 — see
    /// `zjoltQuatIsNormalized` in `ffi/zjolt_math.h` for why that is stricter
    /// than it looks. Pass `1e-5` to match what Jolt's own asserts require.
    pub fn isNormalized(q: Quat, tolerance: f32) bool {
        return vec.quatIsNormalized(vec.from(q), tolerance);
    }

    /// `q` rescaled to unit length. NaN on a zero-length `q`, not refused.
    pub fn normalize(q: Quat) Quat {
        return vec.to(Quat, vec.quatNormalize(vec.from(q)));
    }

    /// A right-handed rotation of `radians` about `axis`.
    ///
    /// Returns `error.InvalidArgument` if `axis` is not unit length — an
    /// un-normalised axis produces an un-normalised quaternion that can trip
    /// an assertion far from this call. Goes through the C library, so
    /// `zjolt.init` must have run first.
    pub fn fromAxisAngle(axis: Vec3, radians: f32) err.Error!Quat {
        var out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        try err.check(c_math.zjoltQuatFromAxisAngle(&axis, radians, &out));
        return out;
    }

    /// The axis and angle `Quat.fromAxisAngle` would have to be given to
    /// reconstruct `q`. `angle` is always in `[0, pi]`. `error.InvalidArgument`
    /// when `q` is not unit length.
    pub fn axisAngle(q: Quat) err.Error!struct { axis: Vec3, angle: f32 } {
        var axis: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        var angle: f32 = 0;
        try err.check(c_math.zjoltQuatGetAxisAngle(&q, &axis, &angle));
        return .{ .axis = axis, .angle = angle };
    }

    /// The rotation from the DIRECTION of `from` to the DIRECTION of `to`,
    /// along the shorter arc. Neither needs to be unit length. Infallible: a
    /// zero-length input gives the identity rather than NaN.
    pub fn fromTo(from: Vec3, to: Vec3) Quat {
        var out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        c_math.zjoltQuatFromTo(&from, &to, &out);
        return out;
    }

    /// A rotation from Euler angles `(x, y, z)` radians, Jolt's fixed
    /// X-then-Y-then-Z order — see `ffi/zjolt_math.h` for what that means and
    /// what it does not match.
    pub fn fromEulerAngles(angles_radians: Vec3) Quat {
        var out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        c_math.zjoltQuatFromEulerAngles(&angles_radians, &out);
        return out;
    }

    /// The Euler angles `Quat.fromEulerAngles` would have to be given to
    /// reconstruct `q`, same axis order. Not exact through gimbal lock.
    pub fn eulerAngles(q: Quat) Vec3 {
        var out: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        c_math.zjoltQuatGetEulerAngles(&q, &out);
        return out;
    }

    /// A quaternion perpendicular to `q` in the four-dimensional sense Jolt's
    /// swing-twist machinery uses — not a 3-D rotation axis.
    pub fn perpendicular(q: Quat) Quat {
        var out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        c_math.zjoltQuatGetPerpendicular(&q, &out);
        return out;
    }

    /// The signed angle `q` rotates around `axis`, which MUST be unit length
    /// — see `ffi/zjolt_math.h`; unlike `Quat.fromAxisAngle`'s axis this is
    /// not checked.
    pub fn rotationAngle(q: Quat, axis: Vec3) f32 {
        return c_math.zjoltQuatGetRotationAngle(&q, &axis);
    }

    /// The component of `q` that rotates only around `axis`, which MUST be
    /// unit length for the same reason as `Quat.rotationAngle`.
    pub fn twist(q: Quat, axis: Vec3) Quat {
        var out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        c_math.zjoltQuatGetTwist(&q, &axis, &out);
        return out;
    }

    /// Splits `q` into `q = swing * twist`, where `twist` rotates only around
    /// `q`'s own LOCAL X AXIS (fixed, unlike `Quat.twist`'s axis parameter)
    /// and `swing` only around its local Y and Z. Recompose with
    /// `result.swing.multiply(result.twist)`. This is the decomposition
    /// `Constraint.initSwingTwist`'s limits are expressed against.
    pub fn swingTwist(q: Quat) struct { swing: Quat, twist: Quat } {
        var swing: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        var twist_out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        c_math.zjoltQuatGetSwingTwist(&q, &swing, &twist_out);
        return .{ .swing = swing, .twist = twist_out };
    }

    /// `(1 - t) * a + t * b`, componentwise — NOT renormalised, NOT
    /// shortest-path. Prefer `Quat.slerp` to interpolate a rotation across a
    /// frame.
    pub fn lerp(a: Quat, b: Quat, t: f32) Quat {
        return vec.to(Quat, vec.quatLerp(vec.from(a), vec.from(b), t));
    }

    /// Spherical interpolation from `a` to `b`, taking the shorter of the two
    /// arcs — the sign correction that makes this safe to call every frame
    /// between two keyframes; see `ffi/zjolt_math.h`.
    pub fn slerp(a: Quat, b: Quat, t: f32) Quat {
        var out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        c_math.zjoltQuatSlerp(&a, &b, t, &out);
        return out;
    }

    /// The local X axis of the rotation `q` represents — `Quat::RotateAxisX`,
    /// which is `q.RotateVector(Vec3(1,0,0))` written out without the general
    /// rotation path. `q` MUST be unit length.
    pub fn rotateAxisX(q: Quat) Vec3 {
        return .{
            .x = 2 * (q.x * q.x + q.w * q.w) - 1,
            .y = 2 * (q.x * q.y + q.w * q.z),
            .z = 2 * (q.x * q.z - q.w * q.y),
        };
    }

    /// As `Quat.rotateAxisX`, for the local Y axis.
    pub fn rotateAxisY(q: Quat) Vec3 {
        return .{
            .x = 2 * (q.x * q.y - q.w * q.z),
            .y = 2 * (q.y * q.y + q.w * q.w) - 1,
            .z = 2 * (q.y * q.z + q.w * q.x),
        };
    }

    /// As `Quat.rotateAxisX`, for the local Z axis.
    pub fn rotateAxisZ(q: Quat) Vec3 {
        return .{
            .x = 2 * (q.z * q.x + q.w * q.y),
            .y = 2 * (q.z * q.y - q.w * q.x),
            .z = 2 * (q.z * q.z + q.w * q.w) - 1,
        };
    }

    /// As `Vec3.compressUnitVector`, but for a unit `Quat` —
    /// `Quat::CompressUnitQuat` forwards to `Vec4::CompressUnitVector`, whose
    /// layout uses 9 bits (not 14) per stored component, since it has to fit
    /// three of them, not two.
    pub fn compressUnitQuat(q: Quat) u32 {
        const one_over_sqrt2: f32 = 0.70710678;
        const max_value: f32 = 510; // (1 << 9) - 1 - 1
        const scale: f32 = max_value / (2.0 * one_over_sqrt2);

        const comps = [4]f32{ q.x, q.y, q.z, q.w };
        var max_element: u32 = 0;
        var max_abs = @abs(comps[0]);
        for (1..4) |i| {
            const a = @abs(comps[i]);
            if (a > max_abs) {
                max_abs = a;
                max_element = @intCast(i);
            }
        }

        var value: u32 = 0;
        var signed_comps = comps;
        if (comps[max_element] < 0.0) {
            value = 0x8000_0000;
            signed_comps = .{ -comps[0], -comps[1], -comps[2], -comps[3] };
        }
        value |= max_element << 29;

        var packed_comps: [4]u32 = undefined;
        for (0..4) |i| {
            const clamped = std.math.clamp((signed_comps[i] + one_over_sqrt2) * scale + 0.5, 0.0, max_value);
            packed_comps[i] = @intFromFloat(clamped);
        }
        const three: [3]u32 = switch (max_element) {
            0 => .{ packed_comps[1], packed_comps[2], packed_comps[3] },
            1 => .{ packed_comps[0], packed_comps[2], packed_comps[3] },
            2 => .{ packed_comps[0], packed_comps[1], packed_comps[3] },
            else => .{ packed_comps[0], packed_comps[1], packed_comps[2] },
        };
        value |= three[0];
        value |= three[1] << 9;
        value |= three[2] << 18;
        return value;
    }

    /// The inverse of `Quat.compressUnitQuat`.
    pub fn decompressUnitQuat(value: u32) Quat {
        const one_over_sqrt2: f32 = 0.70710678;
        const mask: u32 = (1 << 9) - 1;
        const max_value: f32 = 510;
        const half_max_value: i32 = 255;
        const scale: f32 = 2.0 * one_over_sqrt2 / max_value;

        const raw0: i32 = @as(i32, @intCast(value & mask)) - half_max_value;
        const raw1: i32 = @as(i32, @intCast((value >> 9) & mask)) - half_max_value;
        const raw2: i32 = @as(i32, @intCast((value >> 18) & mask)) - half_max_value;
        const a: f32 = @as(f32, @floatFromInt(raw0)) * scale;
        const b: f32 = @as(f32, @floatFromInt(raw1)) * scale;
        const cc: f32 = @as(f32, @floatFromInt(raw2)) * scale;
        const d: f32 = @sqrt(@max(1.0 - (a * a + b * b + cc * cc), 0.0));

        var v = [4]f32{ a, b, cc, d };
        if ((value & 0x8000_0000) != 0) v = .{ -v[0], -v[1], -v[2], -v[3] };

        const out: [4]f32 = switch ((value >> 29) & 3) {
            0 => .{ v[3], v[0], v[1], v[2] },
            1 => .{ v[0], v[3], v[1], v[2] },
            2 => .{ v[0], v[1], v[3], v[2] },
            else => v,
        };
        return .{ .x = out[0], .y = out[1], .z = out[2], .w = out[3] };
    }
};

/// A world-space position. `f64` when the library was built with
/// `-Ddouble_precision`, otherwise `f32`.
pub const RVec3 = extern struct {
    x: Real,
    y: Real,
    z: Real,

    /// As `Vec3.lerp`, over `RVec3` — the type a world-space position
    /// actually has. Interpolating a position through `Vec3.lerp` instead
    /// narrows it to `f32` first, losing exactly the range
    /// `-Ddouble_precision` exists to keep.
    pub fn lerp(a: RVec3, b: RVec3, t: f32) RVec3 {
        const ft: Real = @floatCast(t);
        return .{
            .x = a.x + (b.x - a.x) * ft,
            .y = a.y + (b.y - a.y) * ft,
            .z = a.z + (b.z - a.z) * ft,
        };
    }

    /// `v` narrowed to `f32`, rounding every component TOWARD NEGATIVE
    /// INFINITY. Pair with `RVec3.toVec3RoundUp` on the opposite corner of a
    /// box being narrowed, so the result is never smaller than the true
    /// value. An exact copy under a build without `-Ddouble_precision`,
    /// where `Real` already is `f32`.
    pub fn toVec3RoundDown(v: RVec3) Vec3 {
        var out: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        c_math.zjoltRVec3ToVec3RoundDown(&v, &out);
        return out;
    }

    /// As `RVec3.toVec3RoundDown`, rounding every component TOWARD POSITIVE
    /// INFINITY instead.
    pub fn toVec3RoundUp(v: RVec3) Vec3 {
        var out: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        c_math.zjoltRVec3ToVec3RoundUp(&v, &out);
        return out;
    }
};

/// A 4x4 matrix: four COLUMNS of four floats, so column `c`'s row `r` is
/// `m[4 * c + r]`. Column-major, matching Jolt — not row-major.
///
/// 16-byte aligned, matching `JPH::Mat44` and zozz's `ZozzFloat4x4`, so a host
/// can share one matrix buffer between the two packages.
pub const Mat44 = extern struct {
    m: [16]f32 align(16),

    /// Builds a transform: `rot` in the upper 3x3, `translation` in the
    /// fourth column. `error.InvalidArgument` when `rot` is not unit
    /// length.
    pub fn fromRotationTranslation(rot: Quat, translation: Vec3) err.Error!Mat44 {
        var out: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        try err.check(c_math.zjoltMat44FromRotationTranslation(&rot, &translation, &out));
        return out;
    }

    /// Composes two transforms: `a * b` — apply `b` first, then `a`.
    pub fn multiply(a: Mat44, b: Mat44) Mat44 {
        const cols = vec.mat44Mul(vec.loadCols(a.m), vec.loadCols(b.m));
        return .{ .m = vec.storeCols(cols) };
    }

    /// The general inverse. NaN/Inf on a singular `m`, not refused. Prefer
    /// `Mat44.inverseRotationTranslation` when `m` carries no scale or shear.
    pub fn inverse(m: Mat44) Mat44 {
        var out: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        c_math.zjoltMat44Inverse(&m, &out);
        return out;
    }

    /// The inverse of `m`, ASSUMING it is a rigid transform (orthonormal
    /// rotation, no scale). Cheaper and exact where `Mat44.inverse` merely
    /// converges; wrong, silently, if that assumption does not hold.
    pub fn inverseRotationTranslation(m: Mat44) Mat44 {
        const cols = vec.mat44InverseRotationTranslation(vec.loadCols(m.m));
        return .{ .m = vec.storeCols(cols) };
    }

    /// Transforms `point` as a POSITION: the translation column is added.
    pub fn transformPoint(m: Mat44, point: Vec3) Vec3 {
        return vec.to(Vec3, vec.mat44Point(vec.loadCols(m.m), vec.from(point)));
    }

    /// Transforms `direction` as a DIRECTION: the translation column is
    /// ignored.
    pub fn transformDirection(m: Mat44, direction: Vec3) Vec3 {
        return vec.to(Vec3, vec.mat44Direction(vec.loadCols(m.m), vec.from(direction)));
    }

    /// Splits `m` into a scale and the rigid rotation-and-translation left
    /// over once that scale is divided back out. Multiplying
    /// `result.rotation_translation`'s axis columns back by `result.scale`'s
    /// components reconstructs `m`, mirror image included: a
    /// negative-determinant `m` comes back as a proper (non-mirrored)
    /// rotation with the mirroring folded into `scale`'s Z sign.
    pub fn decompose(m: Mat44) struct { rotation_translation: Mat44, scale: Vec3 } {
        var rotation_translation: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        var scale: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        c_math.zjoltMat44Decompose(&m, &rotation_translation, &scale);
        return .{ .rotation_translation = rotation_translation, .scale = scale };
    }

    /// The rotation `m`'s upper 3x3 represents. Meaningful only when that 3x3
    /// is actually orthonormal (no scale or shear) — see `Mat44.decompose`
    /// for a matrix that might carry either first.
    pub fn quaternion(m: Mat44) Quat {
        var out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        c_math.zjoltMat44GetQuaternion(&m, &out);
        return out;
    }

    // Where Jolt's own `DMat44` narrows a rotation-only result to
    // `Mat44`/`Vec3` (`GetRotation`, `GetAxisX`), the `RMat44` twins below do
    // too; where `DMat44` has no such method, the twin stays `Real`-typed,
    // like `RMat44.multiply` above.

    pub fn axisX(m: Mat44) Vec3 {
        return .{ .x = m.m[0], .y = m.m[1], .z = m.m[2] };
    }
    pub fn axisY(m: Mat44) Vec3 {
        return .{ .x = m.m[4], .y = m.m[5], .z = m.m[6] };
    }
    pub fn axisZ(m: Mat44) Vec3 {
        return .{ .x = m.m[8], .y = m.m[9], .z = m.m[10] };
    }
    pub fn withAxisX(m: Mat44, v: Vec3) Mat44 {
        var out = m;
        out.m[0] = v.x;
        out.m[1] = v.y;
        out.m[2] = v.z;
        out.m[3] = 0;
        return out;
    }
    pub fn withAxisY(m: Mat44, v: Vec3) Mat44 {
        var out = m;
        out.m[4] = v.x;
        out.m[5] = v.y;
        out.m[6] = v.z;
        out.m[7] = 0;
        return out;
    }
    pub fn withAxisZ(m: Mat44, v: Vec3) Mat44 {
        var out = m;
        out.m[8] = v.x;
        out.m[9] = v.y;
        out.m[10] = v.z;
        out.m[11] = 0;
        return out;
    }

    /// Column `col`'s upper 3 rows. `col == 3` reads the translation —
    /// matches `Mat44::GetColumn3`, which (unlike `DMat44::GetColumn3`)
    /// allows all four.
    pub fn column3(m: Mat44, col: usize) Vec3 {
        const base = 4 * col;
        return .{ .x = m.m[base], .y = m.m[base + 1], .z = m.m[base + 2] };
    }
    pub fn column4(m: Mat44, col: usize) [4]f32 {
        const base = 4 * col;
        return .{ m.m[base], m.m[base + 1], m.m[base + 2], m.m[base + 3] };
    }
    /// `m` with column `col`'s upper 3 rows set to `v`; the 4th is forced to
    /// 1 for `col == 3` (translation) and 0 otherwise, matching
    /// `Mat44::SetColumn3`.
    pub fn withColumn3(m: Mat44, col: usize, v: Vec3) Mat44 {
        var out = m;
        const base = 4 * col;
        out.m[base] = v.x;
        out.m[base + 1] = v.y;
        out.m[base + 2] = v.z;
        out.m[base + 3] = if (col == 3) 1 else 0;
        return out;
    }
    pub fn withColumn4(m: Mat44, col: usize, v: [4]f32) Mat44 {
        var out = m;
        const base = 4 * col;
        out.m[base] = v[0];
        out.m[base + 1] = v[1];
        out.m[base + 2] = v[2];
        out.m[base + 3] = v[3];
        return out;
    }

    pub fn diagonal3(m: Mat44) Vec3 {
        return .{ .x = m.m[0], .y = m.m[5], .z = m.m[10] };
    }
    pub fn withDiagonal3(m: Mat44, v: Vec3) Mat44 {
        var out = m;
        out.m[0] = v.x;
        out.m[5] = v.y;
        out.m[10] = v.z;
        return out;
    }
    pub fn diagonal4(m: Mat44) [4]f32 {
        return .{ m.m[0], m.m[5], m.m[10], m.m[15] };
    }
    pub fn withDiagonal4(m: Mat44, v: [4]f32) Mat44 {
        var out = m;
        out.m[0] = v[0];
        out.m[5] = v[1];
        out.m[10] = v[2];
        out.m[15] = v[3];
        return out;
    }

    /// Whether every column of `a` and `b` is within `max_dist_sq` (squared
    /// distance) of the matching column of the other. `1e-12` matches Jolt's
    /// own default.
    pub fn isClose(a: Mat44, b: Mat44, max_dist_sq: f32) bool {
        for (0..4) |col| {
            var sum: f32 = 0;
            for (0..4) |row| {
                const d = a.m[4 * col + row] - b.m[4 * col + row];
                sum += d * d;
            }
            if (sum > max_dist_sq) return false;
        }
        return true;
    }

    /// The determinant of `m`'s upper 3x3, via `vec.crossPrecise` — see
    /// `vec.differenceOfProducts` for why this is not the plain triple
    /// product.
    pub fn determinant3x3(m: Mat44) f32 {
        return vec.dot3(
            vec.from(m.axisX()),
            vec.crossPrecise(vec.from(m.axisY()), vec.from(m.axisZ())),
        );
    }

    /// `m`'s upper 3x3 times `n`'s, translation columns ignored and forced to
    /// `(0,0,0,1)` in the result — `Mat44::Multiply3x3(Mat44Arg)`. The Vec3
    /// overload of the same Jolt method is `Mat44.transformDirection`.
    pub fn multiply3x3(m: Mat44, n: Mat44) Mat44 {
        var out: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        for (0..3) |i| {
            for (0..4) |r| {
                out.m[4 * i + r] = m.m[r] * n.m[4 * i] + m.m[4 + r] * n.m[4 * i + 1] + m.m[8 + r] * n.m[4 * i + 2];
            }
        }
        out.m[12] = 0;
        out.m[13] = 0;
        out.m[14] = 0;
        out.m[15] = 1;
        return out;
    }

    /// `m`'s upper 3x3 times the TRANSPOSE of `n`'s —
    /// `Mat44::Multiply3x3RightTransposed`.
    pub fn multiply3x3RightTransposed(m: Mat44, n: Mat44) Mat44 {
        var out: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        for (0..3) |j| {
            for (0..4) |r| {
                out.m[4 * j + r] = m.m[r] * n.m[j] + m.m[4 + r] * n.m[4 + j] + m.m[8 + r] * n.m[8 + j];
            }
        }
        out.m[12] = 0;
        out.m[13] = 0;
        out.m[14] = 0;
        out.m[15] = 1;
        return out;
    }

    /// `v` transformed by the TRANSPOSE of `m`'s upper 3x3 —
    /// `Mat44::Multiply3x3Transposed`, Jolt's own
    /// `Transposed3x3().Multiply3x3(v)` written directly in terms of the pieces
    /// already here.
    pub fn multiply3x3Transposed(m: Mat44, v: Vec3) Vec3 {
        return m.transposed3x3().transformDirection(v);
    }

    /// The full 4x4 transpose.
    pub fn transposed(m: Mat44) Mat44 {
        var out: Mat44 = undefined;
        for (0..4) |col| {
            for (0..4) |r| out.m[4 * col + r] = m.m[4 * r + col];
        }
        return out;
    }

    /// Transposes the upper 3x3, zeroing everything else —
    /// `Mat44::Transposed3x3`.
    pub fn transposed3x3(m: Mat44) Mat44 {
        var out: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        for (0..3) |col| {
            for (0..3) |r| out.m[4 * col + r] = m.m[4 * r + col];
            out.m[4 * col + 3] = 0;
        }
        return out;
    }

    /// The inverse of `m`'s upper 3x3 (via the adjugate, divided by
    /// `Mat44.determinant3x3`), everything else forced to `(0,0,0,1)` —
    /// `Mat44::Inversed3x3`. NaN/Inf on a singular 3x3, not refused.
    pub fn inversed3x3(m: Mat44) Mat44 {
        const det = m.determinant3x3();
        const e = struct {
            fn at(mm: Mat44, r: usize, col: usize) f32 {
                return mm.m[4 * col + r];
            }
        }.at;

        var out: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        out.m[0] = (e(m, 1, 1) * e(m, 2, 2) - e(m, 1, 2) * e(m, 2, 1)) / det;
        out.m[1] = (e(m, 1, 2) * e(m, 2, 0) - e(m, 1, 0) * e(m, 2, 2)) / det;
        out.m[2] = (e(m, 1, 0) * e(m, 2, 1) - e(m, 1, 1) * e(m, 2, 0)) / det;
        out.m[3] = 0;
        out.m[4] = (e(m, 0, 2) * e(m, 2, 1) - e(m, 0, 1) * e(m, 2, 2)) / det;
        out.m[5] = (e(m, 0, 0) * e(m, 2, 2) - e(m, 0, 2) * e(m, 2, 0)) / det;
        out.m[6] = (e(m, 0, 1) * e(m, 2, 0) - e(m, 0, 0) * e(m, 2, 1)) / det;
        out.m[7] = 0;
        out.m[8] = (e(m, 0, 1) * e(m, 1, 2) - e(m, 0, 2) * e(m, 1, 1)) / det;
        out.m[9] = (e(m, 0, 2) * e(m, 1, 0) - e(m, 0, 0) * e(m, 1, 2)) / det;
        out.m[10] = (e(m, 0, 0) * e(m, 1, 1) - e(m, 0, 1) * e(m, 1, 0)) / det;
        out.m[11] = 0;
        return out;
    }

    /// `m` with its translation column replaced by `(0,0,0,1)`, the upper
    /// 3x3 otherwise untouched — including row 3, which `Mat44::GetRotation`'s
    /// own doc comment calls out as deliberately NOT cleared. See
    /// `Mat44.rotationSafe` for the version that does clear it.
    pub fn rotation(m: Mat44) Mat44 {
        var out = m;
        out.m[12] = 0;
        out.m[13] = 0;
        out.m[14] = 0;
        out.m[15] = 1;
        return out;
    }

    /// As `Mat44.rotation`, but also zeroing row 3 of the first three columns
    /// — `Mat44::GetRotationSafe`.
    pub fn rotationSafe(m: Mat44) Mat44 {
        var out = m.rotation();
        out.m[3] = 0;
        out.m[7] = 0;
        out.m[11] = 0;
        return out;
    }
};

/// A world-space transform. `Mat44` with `Real` elements, exactly as `RVec3`
/// is `Vec3` with `Real` elements, so its width follows
/// `-Ddouble_precision`.
///
/// 16-byte aligned, matching `Mat44` and zozz's `ZozzFloat4x4`, so a host can
/// share one matrix buffer between the two packages.
pub const RMat44 = extern struct {
    m: [16]Real align(16),

    /// `m`'s upper 3x3, widened to a plain `Mat44` by narrowing each axis
    /// through `RMat44.axisX/Y/Z` — the shared piece every `Real`-narrowing
    /// method below reuses rather than re-deriving.
    fn rotation3x3AsMat44(m: RMat44) Mat44 {
        var out: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        out = out.withAxisX(m.axisX());
        out = out.withAxisY(m.axisY());
        out = out.withAxisZ(m.axisZ());
        return out;
    }

    /// As `Mat44.fromRotationTranslation`, `translation` widened to `RVec3`
    /// — the type `zjolt.BodyInterface.getWorldTransform` actually returns.
    pub fn fromRotationTranslation(rot: Quat, translation: RVec3) err.Error!RMat44 {
        var out: RMat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        try err.check(c_math.zjoltRMat44FromRotationTranslation(&rot, &translation, &out));
        return out;
    }

    /// As `Mat44.multiply`, over `RMat44`.
    pub fn multiply(a: RMat44, b: RMat44) RMat44 {
        var out: RMat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        c_math.zjoltRMat44Multiply(&a, &b, &out);
        return out;
    }

    /// As `Mat44.inverse`, over `RMat44`.
    pub fn inverse(m: RMat44) RMat44 {
        var out: RMat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        c_math.zjoltRMat44Inverse(&m, &out);
        return out;
    }

    /// As `Mat44.inverseRotationTranslation`, over `RMat44`.
    pub fn inverseRotationTranslation(m: RMat44) RMat44 {
        var out: RMat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        c_math.zjoltRMat44InverseRotationTranslation(&m, &out);
        return out;
    }

    /// Transforms `point` — a world-space position, `RVec3` precision in and
    /// out — as a POSITION.
    pub fn transformPoint(m: RMat44, point: RVec3) RVec3 {
        var out: RVec3 = .{ .x = 0, .y = 0, .z = 0 };
        c_math.zjoltRMat44TransformPoint(&m, &point, &out);
        return out;
    }

    /// Transforms `direction` as a DIRECTION. `Vec3`, not `RVec3`: a
    /// direction never needs the extended range a world position does.
    pub fn transformDirection(m: RMat44, direction: Vec3) Vec3 {
        var out: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        c_math.zjoltRMat44TransformDirection(&m, &direction, &out);
        return out;
    }

    /// As `Mat44.decompose`, over `RMat44`. The translation column keeps
    /// `m`'s own `Real` precision; only the upper 3x3 goes through the scale
    /// extraction.
    pub fn decompose(m: RMat44) struct { rotation_translation: RMat44, scale: Vec3 } {
        var rotation_translation: RMat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        var scale: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        c_math.zjoltRMat44Decompose(&m, &rotation_translation, &scale);
        return .{ .rotation_translation = rotation_translation, .scale = scale };
    }

    /// As `Mat44.quaternion`, over `RMat44`'s rotation part. `m`'s
    /// translation is ignored entirely.
    pub fn quaternion(m: RMat44) Quat {
        var out: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };
        c_math.zjoltRMat44GetQuaternion(&m, &out);
        return out;
    }

    /// As `Mat44.axisX`, over `RMat44`. Narrows to `Vec3` because `DMat44`'s
    /// own `GetAxisX` does too — an axis is a direction, never in need of
    /// `Real` range, regardless of what width the matrix that holds it
    /// stores it at.
    pub fn axisX(m: RMat44) Vec3 {
        return .{ .x = @floatCast(m.m[0]), .y = @floatCast(m.m[1]), .z = @floatCast(m.m[2]) };
    }
    pub fn axisY(m: RMat44) Vec3 {
        return .{ .x = @floatCast(m.m[4]), .y = @floatCast(m.m[5]), .z = @floatCast(m.m[6]) };
    }
    pub fn axisZ(m: RMat44) Vec3 {
        return .{ .x = @floatCast(m.m[8]), .y = @floatCast(m.m[9]), .z = @floatCast(m.m[10]) };
    }
    pub fn withAxisX(m: RMat44, v: Vec3) RMat44 {
        var out = m;
        out.m[0] = @floatCast(v.x);
        out.m[1] = @floatCast(v.y);
        out.m[2] = @floatCast(v.z);
        out.m[3] = 0;
        return out;
    }
    pub fn withAxisY(m: RMat44, v: Vec3) RMat44 {
        var out = m;
        out.m[4] = @floatCast(v.x);
        out.m[5] = @floatCast(v.y);
        out.m[6] = @floatCast(v.z);
        out.m[7] = 0;
        return out;
    }
    pub fn withAxisZ(m: RMat44, v: Vec3) RMat44 {
        var out = m;
        out.m[8] = @floatCast(v.x);
        out.m[9] = @floatCast(v.y);
        out.m[10] = @floatCast(v.z);
        out.m[11] = 0;
        return out;
    }

    /// As `Mat44.column3`, over `RMat44`. `col` MUST be < 3:
    /// `DMat44::GetColumn3` itself only reaches the rotation columns this way —
    /// reading the translation column at `Real` precision is what
    /// `RMat44.transformPoint`'s matrix already carries, not this accessor.
    pub fn column3(m: RMat44, col: usize) Vec3 {
        std.debug.assert(col < 3);
        const base = 4 * col;
        return .{ .x = @floatCast(m.m[base]), .y = @floatCast(m.m[base + 1]), .z = @floatCast(m.m[base + 2]) };
    }
    pub fn column4(m: RMat44, col: usize) [4]f32 {
        std.debug.assert(col < 3);
        const base = 4 * col;
        return .{ @floatCast(m.m[base]), @floatCast(m.m[base + 1]), @floatCast(m.m[base + 2]), @floatCast(m.m[base + 3]) };
    }
    pub fn withColumn3(m: RMat44, col: usize, v: Vec3) RMat44 {
        std.debug.assert(col < 3);
        var out = m;
        const base = 4 * col;
        out.m[base] = @floatCast(v.x);
        out.m[base + 1] = @floatCast(v.y);
        out.m[base + 2] = @floatCast(v.z);
        out.m[base + 3] = 0;
        return out;
    }
    pub fn withColumn4(m: RMat44, col: usize, v: [4]f32) RMat44 {
        std.debug.assert(col < 3);
        var out = m;
        const base = 4 * col;
        out.m[base] = @floatCast(v[0]);
        out.m[base + 1] = @floatCast(v[1]);
        out.m[base + 2] = @floatCast(v[2]);
        out.m[base + 3] = @floatCast(v[3]);
        return out;
    }

    /// As `Mat44.diagonal3`, over `RMat44`. Narrowed like the axis
    /// accessors: the diagonal of the upper 3x3 is rotation/scale data,
    /// never translation.
    pub fn diagonal3(m: RMat44) Vec3 {
        return .{ .x = @floatCast(m.m[0]), .y = @floatCast(m.m[5]), .z = @floatCast(m.m[10]) };
    }
    pub fn withDiagonal3(m: RMat44, v: Vec3) RMat44 {
        var out = m;
        out.m[0] = @floatCast(v.x);
        out.m[5] = @floatCast(v.y);
        out.m[10] = @floatCast(v.z);
        return out;
    }
    pub fn diagonal4(m: RMat44) [4]f32 {
        return .{ @floatCast(m.m[0]), @floatCast(m.m[5]), @floatCast(m.m[10]), @floatCast(m.m[15]) };
    }
    pub fn withDiagonal4(m: RMat44, v: [4]f32) RMat44 {
        var out = m;
        out.m[0] = @floatCast(v[0]);
        out.m[5] = @floatCast(v[1]);
        out.m[10] = @floatCast(v[2]);
        out.m[15] = @floatCast(v[3]);
        return out;
    }

    /// As `Mat44.isClose`, over `RMat44` — including the translation column,
    /// compared at `Real` precision, unlike the narrowed accessors above.
    pub fn isClose(a: RMat44, b: RMat44, max_dist_sq: f32) bool {
        const tol: Real = @floatCast(max_dist_sq);
        for (0..4) |col| {
            var sum: Real = 0;
            for (0..4) |row| {
                const d = a.m[4 * col + row] - b.m[4 * col + row];
                sum += d * d;
            }
            if (sum > tol) return false;
        }
        return true;
    }

    /// As `Mat44.determinant3x3`, over `RMat44`'s rotation part. `DMat44`
    /// has no such method (rotation is always plain `Mat44` there too), so
    /// this narrows via `RMat44.axisX/Y/Z` and reuses the `Mat44` formula.
    pub fn determinant3x3(m: RMat44) f32 {
        return m.rotation3x3AsMat44().determinant3x3();
    }

    /// As `Mat44.multiply3x3`, over `RMat44`. Not on `DMat44` (nor is any 3x3
    /// matrix-times-matrix there); narrows like `RMat44.determinant3x3`.
    pub fn multiply3x3(m: RMat44, n: RMat44) Mat44 {
        return m.rotation3x3AsMat44().multiply3x3(n.rotation3x3AsMat44());
    }

    /// As `Mat44.multiply3x3RightTransposed`, over `RMat44`.
    pub fn multiply3x3RightTransposed(m: RMat44, n: RMat44) Mat44 {
        return m.rotation3x3AsMat44().multiply3x3RightTransposed(n.rotation3x3AsMat44());
    }

    /// As `Mat44.multiply3x3Transposed`, over `RMat44`.
    pub fn multiply3x3Transposed(m: RMat44, v: Vec3) Vec3 {
        return m.transposed3x3().transformDirection(v);
    }

    /// As `Mat44.transposed`, over `RMat44`. `DMat44` has no full-4x4
    /// transpose (its translation column isn't the same type as its
    /// rotation columns); `RMat44` stores all 16 as `Real` uniformly, so
    /// this one stays wide.
    pub fn transposed(m: RMat44) RMat44 {
        var out: RMat44 = undefined;
        for (0..4) |col| {
            for (0..4) |r| out.m[4 * col + r] = m.m[4 * r + col];
        }
        return out;
    }

    /// As `Mat44.transposed3x3`, over `RMat44`'s rotation part. Matches
    /// `DMat44::Transposed3x3`, which itself returns a plain `Mat44`.
    pub fn transposed3x3(m: RMat44) Mat44 {
        return m.rotation3x3AsMat44().transposed3x3();
    }

    /// As `Mat44.inversed3x3`, over `RMat44`'s rotation part.
    pub fn inversed3x3(m: RMat44) Mat44 {
        return m.rotation3x3AsMat44().inversed3x3();
    }

    /// As `Mat44.rotation`, over `RMat44` — narrowed, matching
    /// `DMat44::GetRotation` exactly (it returns `Mat44`, not `DMat44`).
    pub fn rotation(m: RMat44) Mat44 {
        var out: Mat44 = undefined;
        for (0..12) |i| out.m[i] = @floatCast(m.m[i]);
        out.m[12] = 0;
        out.m[13] = 0;
        out.m[14] = 0;
        out.m[15] = 1;
        return out;
    }

    /// As `Mat44.rotationSafe`, over `RMat44`.
    pub fn rotationSafe(m: RMat44) Mat44 {
        var out = m.rotation();
        out.m[3] = 0;
        out.m[7] = 0;
        out.m[11] = 0;
        return out;
    }
};

// AABox value-type helpers — Jolt/Geometry/AABox.h.
pub const AABox = extern struct {
    min: Vec3,
    max: Vec3,

    pub fn encapsulatePoint(box: AABox, point: Vec3) AABox {
        return .{
            .min = vec.to(Vec3, @min(vec.from(box.min), vec.from(point))),
            .max = vec.to(Vec3, @max(vec.from(box.max), vec.from(point))),
        };
    }

    pub fn encapsulateBox(a: AABox, b: AABox) AABox {
        return .{
            .min = vec.to(Vec3, @min(vec.from(a.min), vec.from(b.min))),
            .max = vec.to(Vec3, @max(vec.from(a.max), vec.from(b.max))),
        };
    }

    /// The overlapping region of `a` and `b`. Not itself a valid box (`min`
    /// may end up past `max` on some axis) when `a` and `b` don't actually
    /// overlap — check with `AABox.overlaps` first if that matters.
    pub fn intersect(a: AABox, b: AABox) AABox {
        return .{
            .min = vec.to(Vec3, @max(vec.from(a.min), vec.from(b.min))),
            .max = vec.to(Vec3, @min(vec.from(a.max), vec.from(b.max))),
        };
    }

    pub fn overlaps(a: AABox, b: AABox) bool {
        const separated = a.min.x > b.max.x or a.min.y > b.max.y or a.min.z > b.max.z or
            a.max.x < b.min.x or a.max.y < b.min.y or a.max.z < b.min.z;
        return !separated;
    }

    /// The corner of `box` farthest along `direction` — the support function
    /// a GJK/SAT test calls.
    pub fn support(box: AABox, direction: Vec3) Vec3 {
        return .{
            .x = if (direction.x < 0) box.min.x else box.max.x,
            .y = if (direction.y < 0) box.min.y else box.max.y,
            .z = if (direction.z < 0) box.min.z else box.max.z,
        };
    }

    pub fn extent(box: AABox) Vec3 {
        return .{ .x = 0.5 * (box.max.x - box.min.x), .y = 0.5 * (box.max.y - box.min.y), .z = 0.5 * (box.max.z - box.min.z) };
    }

    pub fn closestPoint(box: AABox, point: Vec3) Vec3 {
        const clamped = @min(@max(vec.from(point), vec.from(box.min)), vec.from(box.max));
        return vec.to(Vec3, clamped);
    }

    pub fn sqDistanceTo(box: AABox, point: Vec3) f32 {
        return vec.lengthSq3(vec.from(box.closestPoint(point)) - vec.from(point));
    }
};

pub const MassProperties = extern struct {
    mass: f32,
    inertia: [9]f32,

    /// Decomposes `properties.inertia` into principal axes
    /// (`result.rotation`'s columns) and the moments of inertia about them
    /// (`result.diagonal`'s components, largest first), by
    /// eigendecomposition. `error.InvalidArgument` if Jolt's eigensolver
    /// does not converge — in practice reachable only from a tensor that is
    /// not symmetric positive-semi-definite.
    pub fn decomposePrincipalMomentsOfInertia(properties: MassProperties) err.Error!struct { rotation: Mat44, diagonal: Vec3 } {
        var rotation_out: Mat44 = .{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        var diagonal: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
        try err.check(c_math.zjoltMassPropertiesDecomposePrincipalMomentsOfInertia(&properties, &rotation_out, &diagonal));
        return .{ .rotation = rotation_out, .diagonal = diagonal };
    }
};

pub const Color = extern struct {
    r: u8,
    g: u8,
    b: u8,
    a: u8,

    /// A dense 4-component vector. Here only as `toVec4`'s return type;
    /// nothing else in this package's surface needs a fourth lane.
    pub const Vec4 = extern struct { x: f32, y: f32, z: f32, w: f32 };

    /// Each channel divided by 255, in range [0, 1] — `JPH::Color::ToVec4`.
    pub fn toVec4(self: Color) Vec4 {
        return .{
            .x = @as(f32, @floatFromInt(self.r)) / 255.0,
            .y = @as(f32, @floatFromInt(self.g)) / 255.0,
            .z = @as(f32, @floatFromInt(self.b)) / 255.0,
            .w = @as(f32, @floatFromInt(self.a)) / 255.0,
        };
    }
};

pub const MotionType = enum(c_int) {
    static = 0,
    kinematic = 1,
    dynamic = 2,
};

pub const MotionQuality = enum(c_int) {
    discrete = 0,
    linear_cast = 1,
};

pub const Activation = enum(c_int) {
    activate = 0,
    dont_activate = 1,
};

/// Which degrees of freedom a body may use, as a bit mask.
///
/// A packed struct, layout-identical to the C header's six-bit `c_int` — the
/// header uses enumerators only because C has no better way to name bits.
/// Write `.{ .translation_z = false }` rather than `@enumFromInt` of an OR.
pub const AllowedDofs = packed struct(u32) {
    translation_x: bool = true,
    translation_y: bool = true,
    translation_z: bool = true,
    rotation_x: bool = true,
    rotation_y: bool = true,
    rotation_z: bool = true,
    _reserved: u26 = 0,

    pub const all: AllowedDofs = .{};

    /// The translation axes, X/Y/Z in that order — what
    /// `MotionProperties::GetLinearDOFsMask` computes as a SIMD lane mask for
    /// `LockTranslation`, laid out as plain fields since nothing here masks
    /// vectors that way.
    pub fn linearMask(dofs: AllowedDofs) [3]bool {
        return .{ dofs.translation_x, dofs.translation_y, dofs.translation_z };
    }

    /// As `linearMask`, for the rotation axes — `GetAngularDOFsMask`.
    pub fn angularMask(dofs: AllowedDofs) [3]bool {
        return .{ dofs.rotation_x, dofs.rotation_y, dofs.rotation_z };
    }

    /// Movement in X and Y and rotation about Z — a body confined to a plane,
    /// which is Jolt's own `Plane2D`.
    pub const plane_2d: AllowedDofs = .{
        .translation_z = false,
        .rotation_x = false,
        .rotation_y = false,
    };
};

pub const OverrideMassProperties = enum(c_int) {
    calculate_mass_and_inertia = 0,
    calculate_inertia = 1,
    mass_and_inertia_provided = 2,
};

pub const ShapeSubType = enum(c_int) {
    /// Not a shape at all — what `zjoltShapeGetSubType` reports for a null
    /// handle. Distinct from `user_defined`, which is a shape of a kind this
    /// binding has no name for.
    none = 0,
    sphere = 1,
    box = 2,
    capsule = 3,
    convex_hull = 4,
    mesh = 5,
    scaled = 6,
    rotated_translated = 7,
    offset_center_of_mass = 8,
    triangle = 9,
    cylinder = 10,
    tapered_capsule = 11,
    tapered_cylinder = 12,
    static_compound = 13,
    mutable_compound = 14,
    height_field = 15,
    plane = 16,
    empty = 17,
    soft_body = 18,
    /// One of the sixteen `User*` slots Jolt reserves for shape types
    /// registered outside this library. Nothing here can construct one.
    user_defined = 19,
};

pub const BackFaceMode = enum(c_int) {
    ignore = 0,
    collide = 1,
};

pub const GroundState = enum(c_int) {
    on_ground = 0,
    on_steep_ground = 1,
    not_supported = 2,
    in_air = 3,
};

pub const ValidateResult = enum(c_int) {
    accept_all_contacts_for_this_body_pair = 0,
    accept_contact = 1,
    reject_contact = 2,
    reject_all_contacts_for_this_body_pair = 3,
};

/// What a step silently dropped, if anything. Non-zero does not mean the step
/// failed — it means a limit in `PhysicsSystem.Options` is too low.
///
/// A packed struct for the same reason as `AllowedDofs`: it is a bit mask, and
/// `if (update_error.contact_constraints_full)` reads better than masking.
pub const UpdateError = packed struct(u32) {
    manifold_cache_full: bool = false,
    body_pair_cache_full: bool = false,
    contact_constraints_full: bool = false,
    _reserved: u29 = 0,

    pub const none: UpdateError = .{};

    pub fn any(self: UpdateError) bool {
        return @as(u32, @bitCast(self)) != 0;
    }
};

pub const Shape = opaque {};

pub const PhysicsMaterial = opaque {};

pub const PhysicsSystem = opaque {};

pub const JobSystem = opaque {};

pub const Body = opaque {};

pub const Character = opaque {};

pub const GroupFilter = opaque {};

pub const StepListener = opaque {};

pub const BodyAddBatch = opaque {};

pub const Allocator = extern struct {
    allocate: ?*const fn (user: ?*anyopaque, size: usize) callconv(.c) ?*anyopaque,
    reallocate: ?*const fn (user: ?*anyopaque, block: ?*anyopaque, old_size: usize, new_size: usize) callconv(.c) ?*anyopaque,
    free: ?*const fn (user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void,
    aligned_allocate: ?*const fn (user: ?*anyopaque, size: usize, alignment: usize) callconv(.c) ?*anyopaque,
    aligned_free: ?*const fn (user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void,
    user: ?*anyopaque,
};

pub const TraceFn = *const fn (user: ?*anyopaque, message: [*:0]const u8) callconv(.c) void;

pub const AssertFailedFn = *const fn (
    user: ?*anyopaque,
    expression: [*:0]const u8,
    message: ?[*:0]const u8,
    file: [*:0]const u8,
    line: u32,
) callconv(.c) bool;

pub const InitDesc = extern struct {
    allocator: ?*const Allocator = null,
    trace: ?TraceFn = null,
    assert_failed: ?AssertFailedFn = null,
    hooks_user: ?*anyopaque = null,
};

pub const AbiLayout = extern struct {
    layout_size: u32,
    config_id: u32,
    build_flags: u32,
    real_size: u32,
    object_layer_size: u32,
    default_allocate_alignment: u32,
};

pub extern fn zjoltVersion() u32;

pub extern fn zjoltJoltVersion() u32;

pub extern fn zjoltConfigId() u32;

pub extern fn zjoltResultName(result: Result) [*:0]const u8;

pub extern fn zjoltLastError() [*:0]const u8;

pub extern fn zjoltDefaultAllocateAlignment() usize;

pub extern fn zjoltInitWithConfig(desc: ?*const InitDesc, config_id: u32) Result;

pub extern fn zjoltDeinit() void;

pub extern fn zjoltIsInitialized() bool;

pub extern fn zjoltLiveHandleCount() u32;

pub extern fn zjoltAbiLayout(out: *AbiLayout) void;

pub extern fn zjoltJobSystemCreateThreadPool(max_jobs: u32, max_barriers: u32, num_threads: i32, out: **JobSystem) Result;

pub const ThreadHookFn = *const fn (user: ?*anyopaque, thread_index: i32) callconv(.c) void;

pub extern fn zjoltJobSystemCreateThreadPoolWithHooks(max_jobs: u32, max_barriers: u32, num_threads: i32, thread_init: ?ThreadHookFn, thread_exit: ?ThreadHookFn, user: ?*anyopaque, out: **JobSystem) Result;

pub extern fn zjoltJobSystemSetNumThreads(job_system: *JobSystem, num_threads: i32) Result;

pub extern fn zjoltJobSystemCreateSingleThreaded(max_jobs: u32, out: **JobSystem) Result;

/// One unit of the step's work, handed to a host scheduler by
/// `HostJobSystem.queue_job(s)` and run back on the library through
/// `zjoltJobRun`.
pub const Job = opaque {};

pub const HostJobSystem = extern struct {
    get_max_concurrency: ?*const fn (user: ?*anyopaque) callconv(.c) u32 = null,
    queue_job: ?*const fn (user: ?*anyopaque, job: *Job) callconv(.c) void = null,
    queue_jobs: ?*const fn (user: ?*anyopaque, jobs: [*]const *Job, count: u32) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub extern fn zjoltJobSystemCreateHost(host: *const HostJobSystem, max_barriers: u32, out: **JobSystem) Result;

pub extern fn zjoltJobRun(job: ?*Job) void;

pub extern fn zjoltJobAddRef(job: ?*Job) void;

pub extern fn zjoltJobRelease(job: ?*Job) void;

pub extern fn zjoltJobSystemDestroy(job_system: ?*JobSystem) void;

pub extern fn zjoltJobSystemGetMaxConcurrency(job_system: *const JobSystem) u32;

//=============================================================================
// Factory
//=============================================================================

pub const RttiInfo = extern struct {
    name: ?[*:0]const u8,
    hash: u32,
    size: i32,
    is_abstract: bool,
};

pub extern fn zjoltFactoryFind(name: [*:0]const u8, out: *RttiInfo) void;

pub extern fn zjoltFactoryFindByHash(hash: u32, out: *RttiInfo) void;

pub extern fn zjoltFactoryGetAllClasses(out: ?[*]RttiInfo, capacity: u32, out_count: *u32) Result;

//=============================================================================
// Floating-point control word
//=============================================================================

pub const FPControlWordState = extern struct {
    reserved: [1]u64,
};

pub extern fn zjoltFPFlushDenormalsEnter(out: *FPControlWordState) void;

pub extern fn zjoltFPFlushDenormalsLeave(state: *FPControlWordState) void;

//=============================================================================
// External profiler bridge
//=============================================================================

pub const ExternalProfilerStartFn = *const fn (user: ?*anyopaque, name: [*:0]const u8, color: u32, user_data: [*]u8) callconv(.c) void;
pub const ExternalProfilerEndFn = *const fn (user: ?*anyopaque, user_data: [*]u8) callconv(.c) void;

pub extern fn zjoltExternalProfilerSetHooks(start: ?ExternalProfilerStartFn, end: ?ExternalProfilerEndFn, user: ?*anyopaque) Result;

pub extern fn zjoltExternalProfilerClearHooks() void;
