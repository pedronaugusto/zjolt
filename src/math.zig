//! The plain-data types that cross the boundary, and the pure-Zig math API
//! with no natural receiver type: barycentric coordinates and closest
//! points (`closest_point`), ray intersection primitives (`ray`), oriented
//! boxes, and the raw value types `Vec3x4`/`AABox4`/`RayInvDirection`/
//! `IndexedTriangle`. Everything with a receiver — `Vec3`, `Quat`, `Mat44`,
//! `RMat44`, `AABox`, `MassProperties` — is a method on that struct,
//! declared in `src/c/core.zig`, the file that owns the struct.
//!
//! These are the C types re-exported, not copies: a wrapper struct would mean
//! a conversion per body per frame for no benefit.

const std = @import("std");
const c = @import("c/core.zig");
const c_math = @import("c/math.zig");
const c_shape = @import("c/shape.zig");
const vec = @import("vec.zig");
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

/// A dense 2-component column vector. @see `src/c/math.zig`.
pub const Vector2 = c_math.Vector2;

/// A dense 2x2 matrix — `Jolt::Matrix<2, 2>`. @see `src/c/math.zig`.
pub const Matrix2 = c_math.Matrix2;

/// A dense 3x3 matrix — `Jolt::Matrix<3, 3>`. @see `src/c/math.zig`.
pub const Matrix3 = c_math.Matrix3;

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

//=============================================================================
// Barycentric coordinates and closest points — line segments, triangles and
// tetrahedra, all relative to the ORIGIN, matching Jolt's own convention: to
// test against an arbitrary point `p` rather than the origin, subtract `p`
// from every vertex first, and add it back to whatever point one of these
// hands back. See `ffi/zjolt_math.h`. No receiver type unifies these, so
// they are namespaced here rather than left flat.
//=============================================================================

pub const closest_point = struct {
    /// Barycentric weights of the point on the infinite line through `a` and
    /// `b` closest to the origin — the pair to interpolate a per-vertex
    /// attribute at a hit on an edge. `well_defined` is false when `a` and
    /// `b` coincide; `u`/`v` still come back set, picking whichever is
    /// nearer the origin.
    pub fn baryCentricCoordinatesLine(a: Vec3, b: Vec3) struct { u: f32, v: f32, well_defined: bool } {
        var u: f32 = 0;
        var v: f32 = 0;
        const well_defined = c_math.zjoltGetBaryCentricCoordinatesLine(&a, &b, &u, &v);
        return .{ .u = u, .v = v, .well_defined = well_defined };
    }

    /// As `closest_point.baryCentricCoordinatesLine`, for the plane through
    /// triangle `(a, b, c)`. `well_defined` is false when the three are
    /// collinear or coincident; the fallback weights still come back set,
    /// running along whichever edge is longest.
    pub fn baryCentricCoordinatesTriangle(a: Vec3, b: Vec3, vc: Vec3) struct { u: f32, v: f32, w: f32, well_defined: bool } {
        var u: f32 = 0;
        var v: f32 = 0;
        var w: f32 = 0;
        const well_defined = c_math.zjoltGetBaryCentricCoordinatesTriangle(&a, &b, &vc, &u, &v, &w);
        return .{ .u = u, .v = v, .w = w, .well_defined = well_defined };
    }

    /// The point on line segment `(a, b)` closest to the origin. `set`'s bit
    /// 0 is `a`, bit 1 is `b`: one bit means that endpoint is closest, both
    /// bits mean the closest point lies strictly between them (or that `a`
    /// and `b` coincide).
    pub fn line(a: Vec3, b: Vec3) struct { point: Vec3, set: u32 } {
        var point: Vec3 = vec3_zero;
        var set: u32 = 0;
        c_math.zjoltGetClosestPointOnLine(&a, &b, &point, &set);
        return .{ .point = point, .set = set };
    }

    /// The point on triangle `(a, b, c)` — its interior included — closest
    /// to the origin. `set`'s bits are `a`, `b`, `c` in that order: one bit
    /// means a vertex, two an edge, three the interior.
    pub fn triangle(a: Vec3, b: Vec3, vc: Vec3) struct { point: Vec3, set: u32 } {
        var point: Vec3 = vec3_zero;
        var set: u32 = 0;
        c_math.zjoltGetClosestPointOnTriangle(&a, &b, &vc, &point, &set);
        return .{ .point = point, .set = set };
    }

    /// The point on tetrahedron `(a, b, c, d)` — its interior included —
    /// closest to the origin. `set`'s bits are `a`, `b`, `c`, `d` in that
    /// order.
    pub fn tetrahedron(a: Vec3, b: Vec3, vc: Vec3, d: Vec3) struct { point: Vec3, set: u32 } {
        var point: Vec3 = vec3_zero;
        var set: u32 = 0;
        c_math.zjoltGetClosestPointOnTetrahedron(&a, &b, &vc, &d, &point, &set);
        return .{ .point = point, .set = set };
    }
};

//=============================================================================
// Ray intersection primitives — the exact tests Jolt's own shapes run
// internally, usable standalone without building a Shape or running a query.
// A fraction is measured in units of `direction`, which need not be
// normalised: the hit point is `origin + fraction * direction`. No `Ray`
// type exists to hang these off, so they are namespaced here.
//=============================================================================

pub const ray = struct {
    /// Intersects a ray with triangle `(v0, v1, v2)`: the entry fraction, or
    /// `std.math.floatMax(f32)` for no hit (a hit behind the ray origin
    /// included).
    pub fn triangle(origin: Vec3, direction: Vec3, v0: Vec3, v1: Vec3, v2: Vec3) f32 {
        return c_math.zjoltRayTriangle(&origin, &direction, &v0, &v1, &v2);
    }

    /// Intersects a ray with a sphere: the entry fraction, or no-hit
    /// sentinel. 0.0 when the ray starts inside the sphere.
    pub fn sphere(origin: Vec3, direction: Vec3, center: Vec3, radius: f32) f32 {
        return c_math.zjoltRaySphere(&origin, &direction, &center, radius);
    }

    /// As `ray.sphere`, but reports both crossings. Returns how many there
    /// were — 0, 1 or 2 — with `min_fraction`/`max_fraction` meaningful only
    /// when it is at least 1 (equal to each other for exactly 1, a graze).
    pub fn sphereMinMax(origin: Vec3, direction: Vec3, center: Vec3, radius: f32) struct { count: u32, min_fraction: f32, max_fraction: f32 } {
        var min_fraction: f32 = 0;
        var max_fraction: f32 = 0;
        const count = c_math.zjoltRaySphereMinMax(&origin, &direction, &center, radius, &min_fraction, &max_fraction);
        return .{ .count = count, .min_fraction = min_fraction, .max_fraction = max_fraction };
    }

    /// Intersects a ray with a finite cylinder centred at the origin, axis
    /// along Y, `half_height` from the centre to each cap: the entry
    /// fraction, or no-hit sentinel.
    pub fn cylinder(origin: Vec3, direction: Vec3, half_height: f32, radius: f32) f32 {
        return c_math.zjoltRayCylinder(&origin, &direction, half_height, radius);
    }

    /// Intersects a ray with a capsule centred at the origin, axis along Y:
    /// the entry fraction, or no-hit sentinel.
    pub fn capsule(origin: Vec3, direction: Vec3, half_height: f32, radius: f32) f32 {
        return c_math.zjoltRayCapsule(&origin, &direction, half_height, radius);
    }

    /// Intersects a ray with an axis-aligned box: the entry fraction, or
    /// no-hit sentinel. Negative when the ray starts inside `box`.
    pub fn aabox(origin: Vec3, direction: Vec3, box: AABox) f32 {
        return c_math.zjoltRayAABox(&origin, &direction, &box);
    }

    /// As `ray.aabox`, but reports both the entry and exit fraction. Comes
    /// back as the no-hit sentinel pair (positive/negative) rather than
    /// left meaningless when there is no intersection.
    pub fn aaboxMinMax(origin: Vec3, direction: Vec3, box: AABox) struct { min: f32, max: f32 } {
        var min: f32 = 0;
        var max: f32 = 0;
        c_math.zjoltRayAABoxMinMax(&origin, &direction, &box, &min, &max);
        return .{ .min = min, .max = max };
    }

    /// Whether a ray hits `box` at all, without computing a fraction.
    pub fn aaboxHits(origin: Vec3, direction: Vec3, box: AABox) bool {
        return c_math.zjoltRayAABoxHits(&origin, &direction, &box);
    }

    /// `origin + fraction * direction` — `RayCast::GetPointOnRay`.
    pub fn pointOnRay(origin: Vec3, direction: Vec3, fraction: f32) Vec3 {
        return .{ .x = origin.x + fraction * direction.x, .y = origin.y + fraction * direction.y, .z = origin.z + fraction * direction.z };
    }

    /// As `ray.pointOnRay`, over an `RVec3` origin — `RRayCast::GetPointOnRay`.
    /// `direction` stays `Vec3`: `RRayCast` itself never widens it.
    pub fn rpointOnRay(origin: RVec3, direction: Vec3, fraction: f32) RVec3 {
        const f: Real = @floatCast(fraction);
        return .{
            .x = origin.x + f * @as(Real, @floatCast(direction.x)),
            .y = origin.y + f * @as(Real, @floatCast(direction.y)),
            .z = origin.z + f * @as(Real, @floatCast(direction.z)),
        };
    }

    /// Ray-vs-4-boxes: the entry fraction per lane, or `floatMax(f32)` for a
    /// miss on that lane — `RayAABox4`. Unlike `ray.aabox`, takes a
    /// precomputed `RayInvDirection` since batching is the point.
    pub fn aabox4(origin: Vec3, inv_dir: RayInvDirection, boxes: AABox4) @Vector(4, f32) {
        const flt_min: @Vector(4, f32) = @splat(-std.math.floatMax(f32));
        const flt_max: @Vector(4, f32) = @splat(std.math.floatMax(f32));
        const zero: @Vector(4, f32) = @splat(0);

        const origin_x: @Vector(4, f32) = @splat(origin.x);
        const origin_y: @Vector(4, f32) = @splat(origin.y);
        const origin_z: @Vector(4, f32) = @splat(origin.z);
        const parallel_x: @Vector(4, bool) = @splat(inv_dir.is_parallel[0]);
        const parallel_y: @Vector(4, bool) = @splat(inv_dir.is_parallel[1]);
        const parallel_z: @Vector(4, bool) = @splat(inv_dir.is_parallel[2]);
        const invdir_x: @Vector(4, f32) = @splat(inv_dir.inv_direction.x);
        const invdir_y: @Vector(4, f32) = @splat(inv_dir.inv_direction.y);
        const invdir_z: @Vector(4, f32) = @splat(inv_dir.inv_direction.z);

        const t1x = (boxes.min.x - origin_x) * invdir_x;
        const t1y = (boxes.min.y - origin_y) * invdir_y;
        const t1z = (boxes.min.z - origin_z) * invdir_z;
        const t2x = (boxes.max.x - origin_x) * invdir_x;
        const t2y = (boxes.max.y - origin_y) * invdir_y;
        const t2z = (boxes.max.z - origin_z) * invdir_z;

        const tmin_x = @select(f32, parallel_x, flt_min, @min(t1x, t2x));
        const tmin_y = @select(f32, parallel_y, flt_min, @min(t1y, t2y));
        const tmin_z = @select(f32, parallel_z, flt_min, @min(t1z, t2z));
        const tmax_x = @select(f32, parallel_x, flt_max, @max(t1x, t2x));
        const tmax_y = @select(f32, parallel_y, flt_max, @max(t1y, t2y));
        const tmax_z = @select(f32, parallel_z, flt_max, @max(t1z, t2z));

        const tmin = @max(@max(tmin_x, tmin_y), tmin_z);
        const tmax = @min(@min(tmax_x, tmax_y), tmax_z);

        var no_hit = (tmin > tmax) | (tmax < zero);
        no_hit = no_hit | (boxes.min.x > boxes.max.x) | (boxes.min.y > boxes.max.y) | (boxes.min.z > boxes.max.z);
        no_hit = no_hit | (parallel_x & ((origin_x < boxes.min.x) | (origin_x > boxes.max.x)));
        no_hit = no_hit | (parallel_y & ((origin_y < boxes.min.y) | (origin_y > boxes.max.y)));
        no_hit = no_hit | (parallel_z & ((origin_z < boxes.min.z) | (origin_z > boxes.max.z)));

        return @select(f32, no_hit, flt_max, tmin);
    }

    /// Ray-vs-4-triangles (SOA): the entry fraction per lane, or
    /// `floatMax(f32)` for a miss — `RayTriangle4`. Unlike Jolt's own
    /// version this divides directly instead of deferring one division via
    /// a sign-bit trick; same results, simpler to read.
    pub fn triangle4(origin: Vec3, direction: Vec3, v0: Vec3x4, v1: Vec3x4, v2: Vec3x4) @Vector(4, f32) {
        const epsilon: @Vector(4, f32) = @splat(1.0e-12);
        const zero: @Vector(4, f32) = @splat(0);
        const one: @Vector(4, f32) = @splat(1);
        const flt_max: @Vector(4, f32) = @splat(std.math.floatMax(f32));

        const e1x = v1.x - v0.x;
        const e1y = v1.y - v0.y;
        const e1z = v1.z - v0.z;
        const e2x = v2.x - v0.x;
        const e2y = v2.y - v0.y;
        const e2z = v2.z - v0.z;

        const dx: @Vector(4, f32) = @splat(direction.x);
        const dy: @Vector(4, f32) = @splat(direction.y);
        const dz: @Vector(4, f32) = @splat(direction.z);

        const px = v4differenceOfProducts(dy, e2z, dz, e2y);
        const py = v4differenceOfProducts(dz, e2x, dx, e2z);
        const pz = v4differenceOfProducts(dx, e2y, dy, e2x);

        const det = e1x * px + e1y * py + e1z * pz;
        const det_near_zero = @abs(det) < epsilon;
        const det_safe = @select(f32, det_near_zero, one, det);

        const ox: @Vector(4, f32) = @splat(origin.x);
        const oy: @Vector(4, f32) = @splat(origin.y);
        const oz: @Vector(4, f32) = @splat(origin.z);
        const sx = ox - v0.x;
        const sy = oy - v0.y;
        const sz = oz - v0.z;

        const u = (sx * px + sy * py + sz * pz) / det_safe;

        const qx = v4differenceOfProducts(sy, e1z, sz, e1y);
        const qy = v4differenceOfProducts(sz, e1x, sx, e1z);
        const qz = v4differenceOfProducts(sx, e1y, sy, e1x);

        const v = (dx * qx + dy * qy + dz * qz) / det_safe;
        const t = (e2x * qx + e2y * qy + e2z * qz) / det_safe;

        var no_hit = det_near_zero | (u < zero);
        no_hit = no_hit | (v < zero) | ((u + v) > one);
        no_hit = no_hit | (t < zero);

        return @select(f32, no_hit, flt_max, t);
    }
};

fn v4differenceOfProducts(a: @Vector(4, f32), b: @Vector(4, f32), cc: @Vector(4, f32), d: @Vector(4, f32)) @Vector(4, f32) {
    const cd = cc * d;
    const compensation = @mulAdd(@Vector(4, f32), -cc, d, cd);
    const dop = @mulAdd(@Vector(4, f32), a, b, -cd);
    return dop + compensation;
}

//=============================================================================
// Oriented box overlap — the one overlap test that is not one-line arithmetic
// on already-crossed fields (see `tools/verdicts_geometry.txt`'s `Overlaps`
// line): an arbitrarily-rotated box has no such shortcut. No `OrientedBox`
// type exists in this file (`orientation`'s upper 3x3 plus `half_extents` is
// passed loose) and the receiver rule needs the checked TYPE as the first
// parameter, which neither function has, so both stay flat.
//=============================================================================

/// Whether an oriented box — `orientation`'s upper 3x3 rotates it, its fourth
/// column positions its centre, `half_extents` its local-space half size on
/// each axis — overlaps `box`. `epsilon` slackens the separating-axis test
/// for near-parallel edges; 1e-6 matches Jolt's own default.
pub fn orientedBoxOverlapsAABox(orientation: Mat44, half_extents: Vec3, box: AABox, epsilon: f32) bool {
    return c_math.zjoltOrientedBoxOverlapsAABox(&orientation, &half_extents, &box, epsilon);
}

/// As `orientedBoxOverlapsAABox`, between two oriented boxes.
pub fn orientedBoxOverlapsOrientedBox(orientation_a: Mat44, half_extents_a: Vec3, orientation_b: Mat44, half_extents_b: Vec3, epsilon: f32) bool {
    return c_math.zjoltOrientedBoxOverlapsOrientedBox(&orientation_a, &half_extents_a, &orientation_b, &half_extents_b, epsilon);
}

//=============================================================================
// Deterministic trigonometry — `Jolt/Math/Trigonometry.h`. Bound rather than
// computed via `std.math`: libm's sin/cos/etc. are not bit-identical across
// platforms, which `-Dcross_platform_deterministic` depends on being. The
// `*4` functions are four INDEPENDENT angles packed into one SIMD call
// (`Jolt::Vec4`'s own trig methods), not four lanes of the same angle.
//=============================================================================

pub const trig = struct {
    pub fn sin(radians: f32) f32 {
        return c_math.zjoltSin(radians);
    }
    pub fn cos(radians: f32) f32 {
        return c_math.zjoltCos(radians);
    }
    pub fn tan(radians: f32) f32 {
        return c_math.zjoltTan(radians);
    }
    /// Range `[-pi/2, pi/2]`. `x` outside `[-1, 1]` is clamped first, never
    /// NaN.
    pub fn asin(x: f32) f32 {
        return c_math.zjoltASin(x);
    }
    /// Range `[0, pi]`. `x` outside `[-1, 1]` is clamped first, never NaN.
    pub fn acos(x: f32) f32 {
        return c_math.zjoltACos(x);
    }
    /// Range `[-pi/2, pi/2]`.
    pub fn atan(x: f32) f32 {
        return c_math.zjoltATan(x);
    }
    /// Range `[-pi, pi]`. `(0, 0)` is NaN, not a refusal — Jolt's own
    /// `sATan2` divides 0/0 internally rather than special-casing it.
    pub fn atan2(y: f32, x: f32) f32 {
        return c_math.zjoltATan2(y, x);
    }
    /// Cheaper than calling `sin` and `cos` separately: both come from the
    /// same underlying computation.
    pub fn sinCos(radians: f32) struct { sin: f32, cos: f32 } {
        var s: f32 = 0;
        var co: f32 = 0;
        c_math.zjoltSinCos(radians, &s, &co);
        return .{ .sin = s, .cos = co };
    }
    /// `acos`, approximated: max error 4.2e-3 over `[-1, 1]`, ~2.5x faster.
    pub fn acosApproximate(x: f32) f32 {
        return c_math.zjoltACosApproximate(x);
    }

    pub fn sinCos4(in: [4]f32) struct { sin: [4]f32, cos: [4]f32 } {
        var s: [4]f32 = undefined;
        var co: [4]f32 = undefined;
        c_math.zjoltSinCos4(&in, &s, &co);
        return .{ .sin = s, .cos = co };
    }
    pub fn tan4(in: [4]f32) [4]f32 {
        var out: [4]f32 = undefined;
        c_math.zjoltTan4(&in, &out);
        return out;
    }
    pub fn asin4(in: [4]f32) [4]f32 {
        var out: [4]f32 = undefined;
        c_math.zjoltASin4(&in, &out);
        return out;
    }
    pub fn acos4(in: [4]f32) [4]f32 {
        var out: [4]f32 = undefined;
        c_math.zjoltACos4(&in, &out);
        return out;
    }
    pub fn atan4(in: [4]f32) [4]f32 {
        var out: [4]f32 = undefined;
        c_math.zjoltATan4(&in, &out);
        return out;
    }
    pub fn atan24(in_y: [4]f32, in_x: [4]f32) [4]f32 {
        var out: [4]f32 = undefined;
        c_math.zjoltATan24(&in_y, &in_x, &out);
        return out;
    }
};

//=============================================================================
// Half floats — `Jolt/Math/HalfFloat.h`. Bound rather than substituting Zig's
// own `f16`: Jolt's rounding and Inf/NaN handling are its own bit
// manipulation, not vouched for against Zig's f16 conversion.
//=============================================================================

pub const half_float = struct {
    pub const RoundingMode = c_math.HalfFloatRoundingMode;
    pub const HalfFloat = c_math.HalfFloat;

    /// `value` converted to half float on the fastest path this build has
    /// (F16C/NEON, or the portable fallback).
    pub fn fromFloat(value: f32, mode: RoundingMode) HalfFloat {
        return c_math.zjoltHalfFloatFromFloat(value, mode);
    }
    /// As `fromFloat`, but always the portable bit-manipulation path —
    /// what a cross_platform_deterministic build needs.
    pub fn fromFloatFallback(value: f32, mode: RoundingMode) HalfFloat {
        return c_math.zjoltHalfFloatFromFloatFallback(value, mode);
    }
    /// 4 half floats converted to float, fastest available path.
    pub fn toFloat4(in: [4]HalfFloat) [4]f32 {
        var out: [4]f32 = undefined;
        c_math.zjoltHalfFloatToFloat4(&in, &out);
        return out;
    }
    /// As `toFloat4`, but always the portable path.
    pub fn toFloatFallback4(in: [4]HalfFloat) [4]f32 {
        var out: [4]f32 = undefined;
        c_math.zjoltHalfFloatToFloatFallback4(&in, &out);
        return out;
    }
};

/// Roots of `a*x^2 + b*x + c = 0`, including the degenerate linear/constant
/// cases — `Jolt::FindRoot`. `count` is 0, 1 (`x1 == x2`) or 2. `a == b ==
/// c == 0` reports 1 root at `x1 == 0`, one of Jolt's own infinitely-many
/// solutions, not a refusal.
pub fn findRoot(a: f32, b: f32, coef_c: f32) struct { count: u32, x1: f32, x2: f32 } {
    var x1: f32 = 0;
    var x2: f32 = 0;
    const count = c_math.zjoltFindRoot(a, b, coef_c, &x1, &x2);
    return .{ .count = count, .x1 = x1, .x2 = x2 };
}

/// `radians` shifted by whole turns of 2*pi into `[-pi, pi]` —
/// `Jolt::CenterAngleAroundZero`.
pub fn centerAngleAroundZero(radians: f32) f32 {
    return c_math.zjoltCenterAngleAroundZero(radians);
}

/// `m` narrowed to `Mat44` — `DMat44::ToMat44` under `-Ddouble_precision`
/// (ordinary round-to-nearest, NOT `RVec3.toVec3RoundDown/Up`'s directed
/// rounding), an exact copy otherwise. A free function, not an `RMat44`
/// method: `RMat44` is declared in `c/core.zig`, not owned by this file.
pub fn rmat44ToMat44(m: RMat44) Mat44 {
    var out: Mat44 = mat44_identity;
    c_math.zjoltRMat44ToMat44(&m, &out);
    return out;
}

// Everything below is pure Zig: no `ffi/` entry point backs it, so none of it
// needs `zjolt.init`.

/// A `Vec3` with each component held as a 4-wide SIMD lane, for the 4-boxes-
/// at-once helpers below — `Jolt/Geometry/AABox4.h`'s SOA layout.
pub const Vec3x4 = struct {
    x: @Vector(4, f32),
    y: @Vector(4, f32),
    z: @Vector(4, f32),
};

/// Four axis-aligned boxes, SOA: `min`/`max` each hold one lane per box. The
/// per-lane operations below take the SOA "boxes" argument last: it has no
/// natural position as a Zig method receiver (`box1`/`point`/`scale` there
/// are the single, non-SOA operand), so they are namespaced on the type they
/// operate on instead of being dot-callable.
pub const AABox4 = struct {
    min: Vec3x4,
    max: Vec3x4,

    /// Which of `boxes`' 4 lanes overlap the single box `box1` —
    /// `AABox4VsBox`.
    pub fn vsBox(box1: AABox, boxes: AABox4) @Vector(4, bool) {
        const min_x: @Vector(4, f32) = @splat(box1.min.x);
        const min_y: @Vector(4, f32) = @splat(box1.min.y);
        const min_z: @Vector(4, f32) = @splat(box1.min.z);
        const max_x: @Vector(4, f32) = @splat(box1.max.x);
        const max_y: @Vector(4, f32) = @splat(box1.max.y);
        const max_z: @Vector(4, f32) = @splat(box1.max.z);

        const no_overlap_x = (min_x > boxes.max.x) | (boxes.min.x > max_x);
        const no_overlap_y = (min_y > boxes.max.y) | (boxes.min.y > max_y);
        const no_overlap_z = (min_z > boxes.max.z) | (boxes.min.z > max_z);
        return ~(no_overlap_x | no_overlap_y | no_overlap_z);
    }

    /// Which of `boxes`' 4 lanes contain `point` — `AABox4VsPoint`.
    pub fn vsPoint(point: Vec3, boxes: AABox4) @Vector(4, bool) {
        const px: @Vector(4, f32) = @splat(point.x);
        const py: @Vector(4, f32) = @splat(point.y);
        const pz: @Vector(4, f32) = @splat(point.z);

        const overlap_x = (px >= boxes.min.x) & (px <= boxes.max.x);
        const overlap_y = (py >= boxes.min.y) & (py <= boxes.max.y);
        const overlap_z = (pz >= boxes.min.z) & (pz <= boxes.max.z);
        return overlap_x & overlap_y & overlap_z;
    }

    /// Scales all 4 of `boxes`' lanes by `scale`, handling a negative
    /// component the way `AABox.Scaled` does (min/max can swap per axis) —
    /// `AABox4Scale`.
    pub fn scale(s: Vec3, boxes: AABox4) AABox4 {
        const sx: @Vector(4, f32) = @splat(s.x);
        const sy: @Vector(4, f32) = @splat(s.y);
        const sz: @Vector(4, f32) = @splat(s.z);

        const scaled_min_x = sx * boxes.min.x;
        const scaled_max_x = sx * boxes.max.x;
        const scaled_min_y = sy * boxes.min.y;
        const scaled_max_y = sy * boxes.max.y;
        const scaled_min_z = sz * boxes.min.z;
        const scaled_max_z = sz * boxes.max.z;

        return .{
            .min = .{ .x = @min(scaled_min_x, scaled_max_x), .y = @min(scaled_min_y, scaled_max_y), .z = @min(scaled_min_z, scaled_max_z) },
            .max = .{ .x = @max(scaled_min_x, scaled_max_x), .y = @max(scaled_min_y, scaled_max_y), .z = @max(scaled_min_z, scaled_max_z) },
        };
    }

    /// Widens all 4 of `boxes`' lanes by `extent` on both sides —
    /// `AABox4EnlargeWithExtent`.
    pub fn enlargeWithExtent(extent: Vec3, boxes: AABox4) AABox4 {
        const ex: @Vector(4, f32) = @splat(extent.x);
        const ey: @Vector(4, f32) = @splat(extent.y);
        const ez: @Vector(4, f32) = @splat(extent.z);
        return .{
            .min = .{ .x = boxes.min.x - ex, .y = boxes.min.y - ey, .z = boxes.min.z - ez },
            .max = .{ .x = boxes.max.x + ex, .y = boxes.max.y + ey, .z = boxes.max.z + ez },
        };
    }

    /// Squared distance from `point` to each of `boxes`' 4 lanes —
    /// `AABox4DistanceSqToPoint`.
    pub fn distanceSqToPoint(point: Vec3, boxes: AABox4) @Vector(4, f32) {
        const px: @Vector(4, f32) = @splat(point.x);
        const py: @Vector(4, f32) = @splat(point.y);
        const pz: @Vector(4, f32) = @splat(point.z);

        const closest_x = @min(@max(px, boxes.min.x), boxes.max.x);
        const closest_y = @min(@max(py, boxes.min.y), boxes.max.y);
        const closest_z = @min(@max(pz, boxes.min.z), boxes.max.z);

        const dx = closest_x - px;
        const dy = closest_y - py;
        const dz = closest_z - pz;
        return dx * dx + dy * dy + dz * dz;
    }

    /// Which of `boxes`' 4 lanes overlap the sphere at `center` with squared
    /// radius `radius_sq` — `AABox4VsSphere`.
    pub fn vsSphere(center: Vec3, radius_sq: f32, boxes: AABox4) @Vector(4, bool) {
        const dist_sq = AABox4.distanceSqToPoint(center, boxes);
        const r: @Vector(4, f32) = @splat(radius_sq);
        return dist_sq <= r;
    }
};

// Rays, triangles and planes as raw value types — `Jolt/Geometry/
// {RayAABox,RayTriangle,Plane,Triangle,IndexedTriangle}.h`.

/// The reciprocal of a ray direction, precomputed once for repeated
/// ray/box tests — `Jolt/Geometry/RayAABox.h`'s `RayInvDirection`. A
/// component within `1e-20` of zero is treated as parallel to that axis
/// rather than divided by, exactly as Jolt's `Set` does.
pub const RayInvDirection = struct {
    inv_direction: Vec3,
    is_parallel: [3]bool,

    pub fn set(direction: Vec3) RayInvDirection {
        const epsilon = 1.0e-20;
        const parallel = [3]bool{ @abs(direction.x) <= epsilon, @abs(direction.y) <= epsilon, @abs(direction.z) <= epsilon };
        return .{
            .inv_direction = .{
                .x = 1.0 / (if (parallel[0]) @as(f32, 1.0) else direction.x),
                .y = 1.0 / (if (parallel[1]) @as(f32, 1.0) else direction.y),
                .z = 1.0 / (if (parallel[2]) @as(f32, 1.0) else direction.z),
            },
            .is_parallel = parallel,
        };
    }
};

/// `X . normal + constant = 0` — `Jolt/Geometry/Plane.h`. Reuses
/// `shape.zig`'s `Plane` (normal + constant, the Plane shape's own
/// on-the-wire type) rather than a second, layout-identical struct.
pub const Plane = c_shape.Plane;

/// `Plane` is declared in `shape.zig`, so this cannot be one of its methods —
/// Zig has no mechanism to add a method to a type from outside the file that
/// declares it. Stays a free function for that reason alone.
pub fn planeSignedDistance(p: Plane, point: Vec3) f32 {
    return vec.dot3(vec.from(point), vec.from(p.normal)) + p.constant;
}

/// A triangle by position, no vertex list attached — `Jolt::Triangle`. No
/// dedicated Zig type backs a bare triangle (only `IndexedTriangle`, which
/// carries a vertex list), so this has no receiver and stays flat.
pub fn triangleCentroid(v0: Vec3, v1: Vec3, v2: Vec3) Vec3 {
    const sum = vec.to(Vec3, vec.from(v0) + vec.from(v1) + vec.from(v2));
    return .{ .x = sum.x * (1.0 / 3.0), .y = sum.y * (1.0 / 3.0), .z = sum.z * (1.0 / 3.0) };
}

/// A triangle by vertex index, with the material/user-data Jolt's
/// `IndexedTriangle` (not `IndexedTriangleNoMaterial`) carries.
pub const IndexedTriangle = struct {
    idx: [3]u32,
    material_index: u32 = 0,
    user_data: u32 = 0,

    /// Whether `t`'s vertices in `vertices` are collinear or coincident —
    /// `IndexedTriangleNoMaterial::IsDegenerate`. Uses the plain (not
    /// FMA-precise) cross product, matching that method's own `.Cross()`
    /// call.
    pub fn isDegenerate(t: IndexedTriangle, vertices: []const Vec3) bool {
        const v0 = vertices[t.idx[0]];
        const v1 = vertices[t.idx[1]];
        const v2 = vertices[t.idx[2]];
        const cross = vec.to(Vec3, vec.cross3(
            vec.from(v1) - vec.from(v0),
            vec.from(v2) - vec.from(v0),
        ));
        return cross.isNearZero(1.0e-12);
    }

    /// Whether `a` and `b` name the same triangle, in any rotation of vertex
    /// order — `IndexedTriangleNoMaterial::IsEquivalent`. Material/user-data
    /// are not compared, matching that method exactly (`operator==` is the
    /// one that compares them).
    pub fn isEquivalent(a: IndexedTriangle, b: IndexedTriangle) bool {
        return (a.idx[0] == b.idx[0] and a.idx[1] == b.idx[1] and a.idx[2] == b.idx[2]) or
            (a.idx[0] == b.idx[1] and a.idx[1] == b.idx[2] and a.idx[2] == b.idx[0]) or
            (a.idx[0] == b.idx[2] and a.idx[1] == b.idx[0] and a.idx[2] == b.idx[1]);
    }

    /// `t` rotated so its lowest index comes first —
    /// `IndexedTriangle::GetLowestIndexFirst`. Represents the same triangle;
    /// material/user-data carry through unchanged.
    pub fn lowestIndexFirst(t: IndexedTriangle) IndexedTriangle {
        const idx: [3]u32 = if (t.idx[0] < t.idx[1])
            (if (t.idx[0] < t.idx[2]) .{ t.idx[0], t.idx[1], t.idx[2] } else .{ t.idx[2], t.idx[0], t.idx[1] })
        else if (t.idx[1] < t.idx[2])
            .{ t.idx[1], t.idx[2], t.idx[0] }
        else
            .{ t.idx[2], t.idx[0], t.idx[1] };
        return .{ .idx = idx, .material_index = t.material_index, .user_data = t.user_data };
    }

    /// `IndexedTriangleNoMaterial::GetCentroid` — note this divides by 3
    /// where `triangleCentroid` multiplies by `1/3`, matching a genuine (if
    /// inconsequential) difference between the two methods in Jolt itself.
    pub fn centroid(t: IndexedTriangle, vertices: []const Vec3) Vec3 {
        const sum = vec.to(Vec3, vec.from(vertices[t.idx[0]]) +
            vec.from(vertices[t.idx[1]]) + vec.from(vertices[t.idx[2]]));
        return .{ .x = sum.x / 3.0, .y = sum.y / 3.0, .z = sum.z / 3.0 };
    }
};

test "identity constants are what their names claim" {
    try std.testing.expectEqual(@as(f32, 1), quat_identity.w);
    try std.testing.expectEqual(@as(f32, 0), vec3_zero.x);
    try std.testing.expectEqual(@as(f32, -9.81), gravity_earth.y);
}

test "an axis-angle rotation is unit length" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const q = try Quat.fromAxisAngle(vec3(0, 1, 0), std.math.pi / 2.0);
    const length = @sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    try std.testing.expectApproxEqAbs(@as(f32, 1), length, 1e-6);
}

test "a swing-twist decomposition recomposes to the original rotation" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // An axis with no zero component and an angle nowhere near a multiple of
    // pi, so neither GetSwingTwist's degenerate branch (a 180-degree turn
    // about Y or Z) nor a trivially-identity twist is exercised by accident.
    const raw = vec3(0.3, 0.7, -0.2);
    const len = @sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z);
    const axis = vec3(raw.x / len, raw.y / len, raw.z / len);
    const q = try Quat.fromAxisAngle(axis, 1.1);

    const parts = q.swingTwist();
    const recomposed = parts.swing.multiply(parts.twist);

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
    // documents on Quat.multiply.
    const rhs = try Quat.fromAxisAngle(vec3(0, 0, 1), std.math.pi / 2.0);
    const lhs = try Quat.fromAxisAngle(vec3(0, 1, 0), std.math.pi / 2.0);
    const composed = lhs.multiply(rhs);

    const v = vec3(1, 0, 0);
    const sequential = try lhs.rotateVector(try rhs.rotateVector(v));
    const direct = try composed.rotateVector(v);

    try std.testing.expectApproxEqAbs(sequential.x, direct.x, 1e-5);
    try std.testing.expectApproxEqAbs(sequential.y, direct.y, 1e-5);
    try std.testing.expectApproxEqAbs(sequential.z, direct.z, 1e-5);
}

test "euler angles round-trip through Quat.fromEulerAngles and Quat.eulerAngles" {
    // Well away from the Y = +-pi/2 gimbal lock singularity, where the X/Z
    // split becomes arbitrary and a round trip is not expected to hold.
    const angles = vec3(0.2, -0.5, 0.9);
    const q = Quat.fromEulerAngles(angles);
    const back = q.eulerAngles();

    try std.testing.expectApproxEqAbs(angles.x, back.x, 1e-4);
    try std.testing.expectApproxEqAbs(angles.y, back.y, 1e-4);
    try std.testing.expectApproxEqAbs(angles.z, back.z, 1e-4);
}

test "Quat.fromAxisAngle refuses a non-unit axis and rotates correctly with a unit one" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    try std.testing.expectError(err.Error.InvalidArgument, Quat.fromAxisAngle(vec3(2, 0, 0), 1.0));

    const q = try Quat.fromAxisAngle(vec3(0, 1, 0), std.math.pi / 2.0);
    const rotated = try q.rotateVector(vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotated.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotated.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, -1), rotated.z, 1e-5);
}

test "mat44 composes with its rigid inverse to the identity and carries its translation" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rotation = try Quat.fromAxisAngle(vec3(0, 1, 0), 0.6);
    const translation = vec3(1, 2, 3);
    const m = try Mat44.fromRotationTranslation(rotation, translation);

    const inv = m.inverseRotationTranslation();
    const identity_ish = inv.multiply(m);
    for (0..16) |i| {
        const expected: f32 = if (i % 5 == 0) 1 else 0; // diagonal: 0, 5, 10, 15
        try std.testing.expectApproxEqAbs(expected, identity_ish.m[i], 1e-4);
    }

    const transformed = m.transformPoint(vec3_zero);
    try std.testing.expectApproxEqAbs(translation.x, transformed.x, 1e-5);
    try std.testing.expectApproxEqAbs(translation.y, transformed.y, 1e-5);
    try std.testing.expectApproxEqAbs(translation.z, transformed.z, 1e-5);
}

test "rmat44 transforms a point and its rigid inverse undoes it" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rotation = try Quat.fromAxisAngle(vec3(1, 0, 0), 0.4);
    const translation = rvec3(5, -2, 100);
    const m = try RMat44.fromRotationTranslation(rotation, translation);
    const inv = m.inverseRotationTranslation();

    const original = rvec3(1, 1, 1);
    const world = m.transformPoint(original);
    const back = inv.transformPoint(world);

    try std.testing.expectApproxEqAbs(original.x, back.x, 1e-3);
    try std.testing.expectApproxEqAbs(original.y, back.y, 1e-3);
    try std.testing.expectApproxEqAbs(original.z, back.z, 1e-3);
}

test "lerp matches its endpoints and slerp stays unit length" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const a = quat_identity;
    const b = try Quat.fromAxisAngle(vec3(0, 0, 1), std.math.pi / 2.0);

    const lerp_start = a.lerp(b, 0.0);
    const lerp_end = a.lerp(b, 1.0);
    try std.testing.expectApproxEqAbs(a.w, lerp_start.w, 1e-6);
    try std.testing.expectApproxEqAbs(b.w, lerp_end.w, 1e-6);

    const slerp_mid = a.slerp(b, 0.5);
    const length = @sqrt(slerp_mid.x * slerp_mid.x + slerp_mid.y * slerp_mid.y +
        slerp_mid.z * slerp_mid.z + slerp_mid.w * slerp_mid.w);
    try std.testing.expectApproxEqAbs(@as(f32, 1), length, 1e-6);
}

test "Vec3.lerp and RVec3.lerp interpolate componentwise" {
    const mid = vec3(0, 0, 0).lerp(vec3(10, -4, 2), 0.5);
    try std.testing.expectApproxEqAbs(@as(f32, 5), mid.x, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, -2), mid.y, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), mid.z, 1e-6);

    const rmid = rvec3(0, 0, 0).lerp(rvec3(10, -4, 2), 0.5);
    try std.testing.expectApproxEqAbs(@as(Real, 5), rmid.x, 1e-6);
}

test "Mat44.decompose recovers a hand-built scale and unscaled rotation" {
    // A 90-degree rotation about Z (x axis -> y axis) with axis columns
    // scaled by (2, 3, 4) and a translation of (5, 6, 7). Built directly as a
    // struct literal, not through Mat44.fromRotationTranslation, so this does
    // not depend on that call or on zjolt.init.
    const m: Mat44 = .{ .m = .{
        0,  2, 0, 0,
        -3, 0, 0, 0,
        0,  0, 4, 0,
        5,  6, 7, 1,
    } };

    const result = m.decompose();
    try std.testing.expectApproxEqAbs(@as(f32, 2), result.scale.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 3), result.scale.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 4), result.scale.z, 1e-5);

    const expected_rt = [16]f32{
        0,  1, 0, 0,
        -1, 0, 0, 0,
        0,  0, 1, 0,
        5,  6, 7, 1,
    };
    for (0..16) |i| {
        try std.testing.expectApproxEqAbs(expected_rt[i], result.rotation_translation.m[i], 1e-5);
    }
}

test "Mat44.decompose flips the sign into scale, not the rotation, for a mirrored input" {
    // The same rotation as above, but with the Z axis column negated: a
    // mirror (negative-determinant) matrix rather than a proper scale.
    const m: Mat44 = .{ .m = .{
        0,  1, 0,  0,
        -1, 0, 0,  0,
        0,  0, -1, 0,
        0,  0, 0,  1,
    } };

    const result = m.decompose();
    // The rotation stays proper (determinant +1): X cross Y should equal Z,
    // not -Z, so the mirroring must have gone into scale.z's sign instead.
    try std.testing.expectApproxEqAbs(@as(f32, -1), result.scale.z, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), result.rotation_translation.m[10], 1e-5);
}

test "Mat44.quaternion matches a hand-computed 90-degree rotation about Z" {
    const m: Mat44 = .{ .m = .{
        0,  1, 0, 0,
        -1, 0, 0, 0,
        0,  0, 1, 0,
        0,  0, 0, 1,
    } };

    const q = m.quaternion();
    const half_sqrt2: f32 = 0.70710678;
    try std.testing.expectApproxEqAbs(@as(f32, 0), q.x, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), q.y, 1e-6);
    try std.testing.expectApproxEqAbs(half_sqrt2, q.z, 1e-6);
    try std.testing.expectApproxEqAbs(half_sqrt2, q.w, 1e-6);
}

test "RVec3.toVec3RoundDown and RoundUp bracket the true value and are exact under double precision" {
    const v = rvec3(1.0 / 3.0, -7.5, 1000000.25);
    const down = v.toVec3RoundDown();
    const up = v.toVec3RoundUp();

    // True regardless of build precision: a directed rounding must never
    // overshoot the direction it is named for, compared at full precision.
    try std.testing.expect(@as(f64, down.x) <= @as(f64, v.x));
    try std.testing.expect(@as(f64, down.y) <= @as(f64, v.y));
    try std.testing.expect(@as(f64, down.z) <= @as(f64, v.z));
    try std.testing.expect(@as(f64, up.x) >= @as(f64, v.x));
    try std.testing.expect(@as(f64, up.y) >= @as(f64, v.y));
    try std.testing.expect(@as(f64, up.z) >= @as(f64, v.z));

    // Under -Ddouble_precision, Real is f64 and there is real rounding to get
    // right — an ordinary nearest-rounding cast would fail this. Halfway
    // between 1.0f and the next float up is exactly representable as a
    // double, so both bounds have an exact expected value.
    if (Real == f64) {
        const eps: f64 = @floatCast(std.math.floatEps(f32));
        const halfway = rvec3(1.0 + eps / 2.0, 0, 0);
        const d = halfway.toVec3RoundDown();
        const u = halfway.toVec3RoundUp();
        try std.testing.expectEqual(@as(f32, 1.0), d.x);
        try std.testing.expectEqual(@as(f32, 1.0) + std.math.floatEps(f32), u.x);
    }
}

test "MassProperties.decomposePrincipalMomentsOfInertia sorts an already-diagonal tensor" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A diagonal inertia tensor: its own eigenvectors are the standard basis,
    // sorted here into descending eigenvalue order (4, 3, 2), which reorders
    // the axes to (Z, Y, X) — and that reordering is left-handed, so the
    // handedness fix-up must flip the reported X axis's sign.
    const properties: MassProperties = .{ .mass = 1, .inertia = .{
        2, 0, 0,
        0, 3, 0,
        0, 0, 4,
    } };

    const result = try properties.decomposePrincipalMomentsOfInertia();
    try std.testing.expectApproxEqAbs(@as(f32, 4), result.diagonal.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 3), result.diagonal.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 2), result.diagonal.z, 1e-5);

    const expected_rotation = [16]f32{
        0,  0, 1, 0,
        0,  1, 0, 0,
        -1, 0, 0, 0,
        0,  0, 0, 1,
    };
    for (0..16) |i| {
        try std.testing.expectApproxEqAbs(expected_rotation[i], result.rotation.m[i], 1e-5);
    }
}

test "closest_point.baryCentricCoordinatesTriangle finds the centroid for a symmetric triangle facing the origin" {
    // The plane x + y + z = 1 through the standard basis vectors: by
    // symmetry its closest point to the origin is its own centroid,
    // (1/3, 1/3, 1/3), which is exactly u = v = w = 1/3.
    const result = closest_point.baryCentricCoordinatesTriangle(vec3(1, 0, 0), vec3(0, 1, 0), vec3(0, 0, 1));
    try std.testing.expect(result.well_defined);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 3.0), result.u, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 3.0), result.v, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 3.0), result.w, 1e-5);
}

test "closest_point.baryCentricCoordinatesLine finds the midpoint for a symmetric segment" {
    const result = closest_point.baryCentricCoordinatesLine(vec3(1, 0, 0), vec3(0, 1, 0));
    try std.testing.expect(result.well_defined);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), result.u, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), result.v, 1e-5);
}

test "closest_point.line lands strictly between two symmetric endpoints" {
    const result = closest_point.line(vec3(1, 0, 0), vec3(0, 1, 0));
    try std.testing.expectEqual(@as(u32, 0b11), result.set);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), result.point.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), result.point.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), result.point.z, 1e-5);
}

test "closest_point.triangle finds the interior centroid of a symmetric triangle" {
    const result = closest_point.triangle(vec3(1, 0, 0), vec3(0, 1, 0), vec3(0, 0, 1));
    try std.testing.expectEqual(@as(u32, 0b111), result.set);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 3.0), result.point.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 3.0), result.point.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 3.0), result.point.z, 1e-5);
}

test "closest_point.tetrahedron reports the origin itself when it is the centroid" {
    // Alternating corners of a cube: a regular tetrahedron centred exactly on
    // the origin, which therefore lies in its interior.
    const result = closest_point.tetrahedron(
        vec3(1, 1, 1),
        vec3(-1, -1, 1),
        vec3(-1, 1, -1),
        vec3(1, -1, -1),
    );
    try std.testing.expectEqual(@as(u32, 0b1111), result.set);
    try std.testing.expectApproxEqAbs(@as(f32, 0), result.point.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), result.point.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), result.point.z, 1e-5);
}

test "ray intersection primitives match hand-computed fractions" {
    const no_hit = std.math.floatMax(f32);

    // Triangle in the z = 1 plane; a ray straight up through its interior
    // hits it at fraction 1, straight up outside it misses entirely.
    const tri_v0 = vec3(0, 0, 1);
    const tri_v1 = vec3(1, 0, 1);
    const tri_v2 = vec3(0, 1, 1);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), ray.triangle(vec3(0.2, 0.2, 0), vec3(0, 0, 1), tri_v0, tri_v1, tri_v2), 1e-5);
    try std.testing.expectEqual(no_hit, ray.triangle(vec3(5, 5, 0), vec3(0, 0, 1), tri_v0, tri_v1, tri_v2));

    // Unit sphere at the origin, ray along +X starting 5 units to the left:
    // enters at x = -1 (fraction 4), exits at x = 1 (fraction 6).
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), ray.sphere(vec3(-5, 0, 0), vec3(1, 0, 0), vec3_zero, 1.0), 1e-5);
    const sphere_range = ray.sphereMinMax(vec3(-5, 0, 0), vec3(1, 0, 0), vec3_zero, 1.0);
    try std.testing.expectEqual(@as(u32, 2), sphere_range.count);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), sphere_range.min_fraction, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), sphere_range.max_fraction, 1e-5);

    // Same ray against a cylinder/capsule of radius 1 whose finite extent
    // (half-height 2 or 1) comfortably contains y = 0: both reduce to the
    // same infinite-cylinder crossing as the sphere case's equatorial ring.
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), ray.cylinder(vec3(-5, 0, 0), vec3(1, 0, 0), 2.0, 1.0), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), ray.capsule(vec3(-5, 0, 0), vec3(1, 0, 0), 1.0, 1.0), 1e-5);

    // A [-1,1]^3 box, same ray: enters at x = -1 (fraction 4), exits at
    // x = 1 (fraction 6).
    const box: AABox = .{ .min = vec3(-1, -1, -1), .max = vec3(1, 1, 1) };
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), ray.aabox(vec3(-5, 0, 0), vec3(1, 0, 0), box), 1e-5);
    const box_range = ray.aaboxMinMax(vec3(-5, 0, 0), vec3(1, 0, 0), box);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), box_range.min, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), box_range.max, 1e-5);
    try std.testing.expect(ray.aaboxHits(vec3(-5, 0, 0), vec3(1, 0, 0), box));
    try std.testing.expect(!ray.aaboxHits(vec3(-5, 5, 5), vec3(1, 0, 0), box));
}

test "orientedBoxOverlapsAABox and orientedBoxOverlapsOrientedBox agree with an obvious overlap and an obvious miss" {
    const identity_orientation = mat44_identity;
    const unit_half_extents = vec3(1, 1, 1);
    const unit_box: AABox = .{ .min = vec3(-1, -1, -1), .max = vec3(1, 1, 1) };
    const far_box: AABox = .{ .min = vec3(10, 10, 10), .max = vec3(12, 12, 12) };

    try std.testing.expect(orientedBoxOverlapsAABox(identity_orientation, unit_half_extents, unit_box, 1e-6));
    try std.testing.expect(!orientedBoxOverlapsAABox(identity_orientation, unit_half_extents, far_box, 1e-6));

    var far_orientation = mat44_identity;
    far_orientation.m[12] = 20;
    try std.testing.expect(orientedBoxOverlapsOrientedBox(identity_orientation, unit_half_extents, identity_orientation, unit_half_extents, 1e-6));
    try std.testing.expect(!orientedBoxOverlapsOrientedBox(identity_orientation, unit_half_extents, far_orientation, unit_half_extents, 1e-6));
}
