//! The plain-data types that cross the boundary.
//!
//! These are the C types re-exported, not copies: a wrapper struct would mean
//! a conversion per body per frame for no benefit.

const std = @import("std");
const c = @import("c.zig");

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

pub const AABox = c.AABox;
pub const MassProperties = c.MassProperties;
pub const ShapeStats = c.ShapeStats;

pub const vec3_zero: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
pub const rvec3_zero: RVec3 = .{ .x = 0, .y = 0, .z = 0 };
pub const quat_identity: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };

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
