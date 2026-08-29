//! GJK, EPA, convex hull building, polygon clipping and triangle indexing.
//!
//! GJK and EPA work against any convex object that can answer "the point of
//! me furthest in this direction" -- `ConvexSupport` is that question as a
//! callback, and `ConvexSupportAdapter` builds the common compositions
//! (transformed, rounded, a Minkowski difference, a bare polygon or
//! triangle) without a caller writing its own.

const std = @import("std");
const c = @import("c/geometry.zig");
const err = @import("error.zig");
const math = @import("math.zig");

//=============================================================================
// Vec3 arithmetic, file-private
//
// `math.Vec3` carries no operators of its own; the plane/tetrahedron
// predicates and the hull normal/centroid math below need them locally.
//=============================================================================

fn vAdd(a: math.Vec3, b: math.Vec3) math.Vec3 {
    return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
}
fn vSub(a: math.Vec3, b: math.Vec3) math.Vec3 {
    return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
}
fn vScale(a: math.Vec3, s: f32) math.Vec3 {
    return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s };
}
fn vDot(a: math.Vec3, b: math.Vec3) f32 {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
fn vCross(a: math.Vec3, b: math.Vec3) math.Vec3 {
    return .{ .x = a.y * b.z - a.z * b.y, .y = a.z * b.x - a.x * b.z, .z = a.x * b.y - a.y * b.x };
}

//=============================================================================
// The support-function seam
//=============================================================================

/// A convex object, given by the support point it returns for a direction.
/// Required wherever this module takes one: a null `support` is refused at
/// the C entry point that receives it, never dereferenced.
pub const ConvexSupport = c.ConvexSupport;

/// One of the five adapters below, wrapping an inner support (or raw
/// vertices) into a fresh `ConvexSupport`.
///
/// `inner`'s `user` context must stay valid for as long as the adapter is
/// used, not just one call: unlike a plain `ConvexSupport` handed straight
/// to `GJK`/`EPA`, an adapter is a handle invoked many times after creation.
pub const ConvexSupportAdapter = struct {
    handle: *c.ConvexSupportAdapter,

    /// `inner`, placed by `transform` (uniform scale only).
    pub fn initTransformed(transform: math.Mat44, inner: ConvexSupport) err.Error!ConvexSupportAdapter {
        var handle: *c.ConvexSupportAdapter = undefined;
        try err.check(c.zjoltConvexSupportCreateTransformed(&transform, &inner, &handle));
        return .{ .handle = handle };
    }

    /// `inner`, rounded by `radius` in every direction.
    pub fn initAddConvexRadius(inner: ConvexSupport, radius: f32) err.Error!ConvexSupportAdapter {
        var handle: *c.ConvexSupportAdapter = undefined;
        try err.check(c.zjoltConvexSupportCreateAddConvexRadius(&inner, radius, &handle));
        return .{ .handle = handle };
    }

    /// The Minkowski difference `a` - `b`, the shape GJK itself searches.
    pub fn initMinkowskiDifference(a: ConvexSupport, b: ConvexSupport) err.Error!ConvexSupportAdapter {
        var handle: *c.ConvexSupportAdapter = undefined;
        try err.check(c.zjoltConvexSupportCreateMinkowskiDifference(&a, &b, &handle));
        return .{ .handle = handle };
    }

    /// `points` copied in; at least one point is required.
    pub fn initPolygon(points: []const math.Vec3) err.Error!ConvexSupportAdapter {
        var handle: *c.ConvexSupportAdapter = undefined;
        try err.check(c.zjoltConvexSupportCreatePolygon(points.ptr, @intCast(points.len), &handle));
        return .{ .handle = handle };
    }

    pub fn initTriangle(v1: math.Vec3, v2: math.Vec3, v3: math.Vec3) err.Error!ConvexSupportAdapter {
        var handle: *c.ConvexSupportAdapter = undefined;
        try err.check(c.zjoltConvexSupportCreateTriangle(&v1, &v2, &v3, &handle));
        return .{ .handle = handle };
    }

    /// This adapter as something `GJK`/`EPA` can take. Valid exactly as long
    /// as `self` is.
    pub fn asSupport(self: ConvexSupportAdapter) ConvexSupport {
        var out: ConvexSupport = undefined;
        c.zjoltConvexSupportAdapterAsSupport(self.handle, &out);
        return out;
    }

    pub fn deinit(self: ConvexSupportAdapter) void {
        c.zjoltConvexSupportAdapterDestroy(self.handle);
    }
};

//=============================================================================
// GJK
//=============================================================================

/// Vertices `GJK.closestPointsSimplex` reports.
pub const max_simplex_points: u32 = c.gjk_max_simplex_points;

/// The simplex a `GJK` holds after its last call: `y` on the Minkowski
/// difference, `p`/`q` the support points on A and B it came from.
pub const Simplex = struct {
    y: [max_simplex_points]math.Vec3 = undefined,
    p: [max_simplex_points]math.Vec3 = undefined,
    q: [max_simplex_points]math.Vec3 = undefined,
    num_points: u32 = 0,

    /// Keeps `y[i]` where bit `i` of `set` is set, compacted in order, and
    /// sets `num_points` to the kept count. `p`/`q` are untouched --
    /// `GJKClosestPoint::UpdatePointSetY`.
    pub fn updatePointSetY(self: *Simplex, set: u32) void {
        var n: u32 = 0;
        for (0..self.num_points) |i| {
            if ((set & (@as(u32, 1) << @intCast(i))) != 0) {
                self.y[n] = self.y[i];
                n += 1;
            }
        }
        self.num_points = n;
    }

    /// As `updatePointSetY`, but compacts `p` instead of `y`; `y`/`q` are
    /// untouched -- `GJKClosestPoint::UpdatePointSetP`.
    pub fn updatePointSetP(self: *Simplex, set: u32) void {
        var n: u32 = 0;
        for (0..self.num_points) |i| {
            if ((set & (@as(u32, 1) << @intCast(i))) != 0) {
                self.p[n] = self.p[i];
                n += 1;
            }
        }
        self.num_points = n;
    }

    /// As `updatePointSetY`, but compacts `p` and `q` together; `y` is
    /// untouched -- `GJKClosestPoint::UpdatePointSetPQ`.
    pub fn updatePointSetPQ(self: *Simplex, set: u32) void {
        var n: u32 = 0;
        for (0..self.num_points) |i| {
            if ((set & (@as(u32, 1) << @intCast(i))) != 0) {
                self.p[n] = self.p[i];
                self.q[n] = self.q[i];
                n += 1;
            }
        }
        self.num_points = n;
    }

    /// As `updatePointSetY`, but compacts `y`, `p` and `q` together --
    /// `GJKClosestPoint::UpdatePointSetYPQ`.
    pub fn updatePointSetYPQ(self: *Simplex, set: u32) void {
        var n: u32 = 0;
        for (0..self.num_points) |i| {
            if ((set & (@as(u32, 1) << @intCast(i))) != 0) {
                self.y[n] = self.y[i];
                self.p[n] = self.p[i];
                self.q[n] = self.q[i];
                n += 1;
            }
        }
        self.num_points = n;
    }
};

//=============================================================================
// Plane and tetrahedron predicates -- Jolt/Geometry/ClosestPoint.h
//
// Used internally by GJK's own closest-point-on-tetrahedron search; pure
// functions over vectors and bits, so they cross no ABI of their own.
//=============================================================================

/// True if the origin and `d` lie on opposite sides of the plane through (`a`,
/// `b`, `vc`); `d` marks the plane's front side -- `OriginOutsideOfPlane`.
pub fn originOutsideOfPlane(a: math.Vec3, b: math.Vec3, vc: math.Vec3, d: math.Vec3) bool {
    const n = vCross(vSub(b, a), vSub(vc, a));
    const signp = vDot(a, n);
    const signd = vDot(vSub(d, a), n);
    return signp * signd > -std.math.floatEps(f32);
}

/// Whether the origin is outside each of tetrahedron (`a`,`b`,`vc`,`d`)'s four
/// faces, indices 0..3 for planes ABC, ACD, ADB, BDC. All four come back
/// `true` if the tetrahedron is degenerate --
/// `OriginOutsideOfTetrahedronPlanes`.
pub fn originOutsideOfTetrahedronPlanes(a: math.Vec3, b: math.Vec3, vc: math.Vec3, d: math.Vec3) [4]bool {
    const ab = vSub(b, a);
    const ac = vSub(vc, a);
    const ad = vSub(d, a);
    const bd = vSub(d, b);
    const bc = vSub(vc, b);

    const ab_cross_ac = vCross(ab, ac);
    const ac_cross_ad = vCross(ac, ad);
    const ad_cross_ab = vCross(ad, ab);
    const bd_cross_bc = vCross(bd, bc);

    const signp = [4]f32{
        vDot(a, ab_cross_ac),
        vDot(a, ac_cross_ad),
        vDot(a, ad_cross_ab),
        vDot(b, bd_cross_bc),
    };
    const signd = [4]f32{
        vDot(ad, ab_cross_ac),
        vDot(ab, ac_cross_ad),
        vDot(ac, ad_cross_ab),
        -vDot(ab, bd_cross_bc),
    };

    var all_pos = true;
    var all_neg = true;
    for (signd) |s| {
        if (std.math.signbit(s)) all_pos = false else all_neg = false;
    }

    const eps = std.math.floatEps(f32);
    var out: [4]bool = undefined;
    for (0..4) |i| {
        out[i] = if (all_pos) signp[i] >= -eps else if (all_neg) signp[i] <= eps else true;
    }
    return out;
}

/// Recomputes the closest points on A and B from a simplex -- typically one
/// read back with `GJK.closestPointsSimplex` -- without needing a `GJK` of
/// its own. `simplex.num_points` must be in [1, max_simplex_points]; at
/// `max_simplex_points` the origin is inside the simplex and there is no
/// well-defined closest pair, so both points come back as the zero vector.
pub fn calculatePointAAndB(simplex: Simplex) err.Error!struct { point_a: math.Vec3, point_b: math.Vec3 } {
    var point_a: math.Vec3 = undefined;
    var point_b: math.Vec3 = undefined;
    try err.check(c.zjoltGJKCalculatePointAAndB(
        &simplex.y,
        &simplex.p,
        &simplex.q,
        simplex.num_points,
        &point_a,
        &point_b,
    ));
    return .{ .point_a = point_a, .point_b = point_b };
}

pub const GJK = struct {
    handle: *c.GJK,

    pub fn init() err.Error!GJK {
        var handle: *c.GJK = undefined;
        try err.check(c.zjoltGJKCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: GJK) void {
        c.zjoltGJKDestroy(self.handle);
    }

    /// True if `a` and `b` overlap. `v` is the initial separating-axis guess
    /// (zero if unknown); the returned `v` is a separating axis on a miss.
    pub fn intersects(
        self: GJK,
        a: ConvexSupport,
        b: ConvexSupport,
        tolerance: f32,
        v: math.Vec3,
    ) err.Error!struct { intersects: bool, v: math.Vec3 } {
        var io_v = v;
        var hit: bool = undefined;
        try err.check(c.zjoltGJKIntersects(self.handle, &a, &b, tolerance, &io_v, &hit));
        return .{ .intersects = hit, .v = io_v };
    }

    /// The squared distance between `a` and `b`, or `FLT_MAX` if it exceeds
    /// `max_dist_sq`. `point_a`/`point_b` are meaningful only when the
    /// returned `dist_sq` is strictly between 0 and `FLT_MAX`; both come back
    /// as the zero vector otherwise.
    pub fn closestPoints(
        self: GJK,
        a: ConvexSupport,
        b: ConvexSupport,
        tolerance: f32,
        max_dist_sq: f32,
        v: math.Vec3,
    ) err.Error!struct { v: math.Vec3, point_a: math.Vec3, point_b: math.Vec3, dist_sq: f32 } {
        var io_v = v;
        var point_a: math.Vec3 = undefined;
        var point_b: math.Vec3 = undefined;
        var dist_sq: f32 = undefined;
        try err.check(c.zjoltGJKGetClosestPoints(
            self.handle,
            &a,
            &b,
            tolerance,
            max_dist_sq,
            &io_v,
            &point_a,
            &point_b,
            &dist_sq,
        ));
        return .{ .v = io_v, .point_a = point_a, .point_b = point_b, .dist_sq = dist_sq };
    }

    /// The simplex left behind by the last call, typically `closestPoints`.
    pub fn closestPointsSimplex(self: GJK) Simplex {
        var s: Simplex = .{};
        c.zjoltGJKGetClosestPointsSimplex(self.handle, &s.y, &s.p, &s.q, &s.num_points);
        return s;
    }

    /// Casts a ray against `a`. `max_lambda` is the max fraction along the
    /// ray; the returned `lambda` is the hit fraction on a hit.
    pub fn castRay(
        self: GJK,
        ray_origin: math.Vec3,
        ray_direction: math.Vec3,
        tolerance: f32,
        a: ConvexSupport,
        max_lambda: f32,
    ) err.Error!struct { hit: bool, lambda: f32 } {
        var lambda = max_lambda;
        var hit: bool = undefined;
        try err.check(c.zjoltGJKCastRay(
            self.handle,
            &ray_origin,
            &ray_direction,
            tolerance,
            &a,
            &lambda,
            &hit,
        ));
        return .{ .hit = hit, .lambda = lambda };
    }

    /// Whether `a`, swept from `start` along `direction`, hits `b` -- no
    /// convex radius, no contact points. `max_lambda` is the max fraction of
    /// the sweep; the returned `lambda` is the hit fraction on a hit.
    pub fn intersectsSweep(
        self: GJK,
        start: math.Mat44,
        direction: math.Vec3,
        tolerance: f32,
        a: ConvexSupport,
        b: ConvexSupport,
        max_lambda: f32,
    ) err.Error!struct { hit: bool, lambda: f32 } {
        var lambda = max_lambda;
        var hit: bool = undefined;
        try err.check(c.zjoltGJKIntersectsSweep(
            self.handle,
            &start,
            &direction,
            tolerance,
            &a,
            &b,
            &lambda,
            &hit,
        ));
        return .{ .hit = hit, .lambda = lambda };
    }

    /// As `intersectsSweep`, but with convex radii and a contact point pair
    /// plus separating axis on a hit.
    pub fn castShape(
        self: GJK,
        start: math.Mat44,
        direction: math.Vec3,
        tolerance: f32,
        a: ConvexSupport,
        b: ConvexSupport,
        convex_radius_a: f32,
        convex_radius_b: f32,
        max_lambda: f32,
    ) err.Error!struct {
        hit: bool,
        lambda: f32,
        point_a: math.Vec3,
        point_b: math.Vec3,
        separating_axis: math.Vec3,
    } {
        var lambda = max_lambda;
        var point_a: math.Vec3 = undefined;
        var point_b: math.Vec3 = undefined;
        var separating_axis: math.Vec3 = undefined;
        var hit: bool = undefined;
        try err.check(c.zjoltGJKCastShape(
            self.handle,
            &start,
            &direction,
            tolerance,
            &a,
            &b,
            convex_radius_a,
            convex_radius_b,
            &lambda,
            &point_a,
            &point_b,
            &separating_axis,
            &hit,
        ));
        return .{
            .hit = hit,
            .lambda = lambda,
            .point_a = point_a,
            .point_b = point_b,
            .separating_axis = separating_axis,
        };
    }
};

//=============================================================================
// EPA
//=============================================================================

pub const EPA = struct {
    handle: *c.EPA,

    pub const Status = c.EpaStatus;

    pub fn init() err.Error!EPA {
        var handle: *c.EPA = undefined;
        try err.check(c.zjoltEPACreate(&handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: EPA) void {
        c.zjoltEPADestroy(self.handle);
    }

    /// First step: GJK against the objects without their convex radius. `v`
    /// must not be near zero (pass a previous result, or (1, 0, 0)).
    pub fn penetrationDepthStepGJK(
        self: EPA,
        a_excluding_radius: ConvexSupport,
        convex_radius_a: f32,
        b_excluding_radius: ConvexSupport,
        convex_radius_b: f32,
        tolerance: f32,
        v: math.Vec3,
    ) err.Error!struct { status: Status, v: math.Vec3, point_a: math.Vec3, point_b: math.Vec3 } {
        var io_v = v;
        var point_a: math.Vec3 = undefined;
        var point_b: math.Vec3 = undefined;
        var status: Status = undefined;
        try err.check(c.zjoltEPAGetPenetrationDepthStepGJK(
            self.handle,
            &a_excluding_radius,
            convex_radius_a,
            &b_excluding_radius,
            convex_radius_b,
            tolerance,
            &io_v,
            &point_a,
            &point_b,
            &status,
        ));
        return .{ .status = status, .v = io_v, .point_a = point_a, .point_b = point_b };
    }

    /// Second step, only needed after `.indeterminate`: the objects WITH
    /// their convex radius. `tolerance` must be at least `FLT_EPSILON`.
    pub fn penetrationDepthStepEPA(
        self: EPA,
        a_including_radius: ConvexSupport,
        b_including_radius: ConvexSupport,
        tolerance: f32,
    ) err.Error!struct { collided: bool, v: math.Vec3, point_a: math.Vec3, point_b: math.Vec3 } {
        var v: math.Vec3 = undefined;
        var point_a: math.Vec3 = undefined;
        var point_b: math.Vec3 = undefined;
        var collided: bool = undefined;
        try err.check(c.zjoltEPAGetPenetrationDepthStepEPA(
            self.handle,
            &a_including_radius,
            &b_including_radius,
            tolerance,
            &v,
            &point_a,
            &point_b,
            &collided,
        ));
        return .{ .collided = collided, .v = v, .point_a = point_a, .point_b = point_b };
    }

    /// Both steps in one call. `v` must not be near zero.
    /// `penetration_tolerance` must be at least `FLT_EPSILON`.
    pub fn penetrationDepth(
        self: EPA,
        a_excluding_radius: ConvexSupport,
        a_including_radius: ConvexSupport,
        convex_radius_a: f32,
        b_excluding_radius: ConvexSupport,
        b_including_radius: ConvexSupport,
        convex_radius_b: f32,
        collision_tolerance_sq: f32,
        penetration_tolerance: f32,
        v: math.Vec3,
    ) err.Error!struct { collided: bool, v: math.Vec3, point_a: math.Vec3, point_b: math.Vec3 } {
        var io_v = v;
        var point_a: math.Vec3 = undefined;
        var point_b: math.Vec3 = undefined;
        var collided: bool = undefined;
        try err.check(c.zjoltEPAGetPenetrationDepth(
            self.handle,
            &a_excluding_radius,
            &a_including_radius,
            convex_radius_a,
            &b_excluding_radius,
            &b_including_radius,
            convex_radius_b,
            collision_tolerance_sq,
            penetration_tolerance,
            &io_v,
            &point_a,
            &point_b,
            &collided,
        ));
        return .{ .collided = collided, .v = io_v, .point_a = point_a, .point_b = point_b };
    }

    /// Casts `a` from `start` along `direction` against `b`, resolving a
    /// starting overlap into a deepest contact when `return_deepest_point` is
    /// true. `max_lambda` is the max fraction of the sweep; the returned
    /// `lambda` is the hit fraction on a hit. `penetration_tolerance` must be
    /// at least `FLT_EPSILON`.
    pub fn castShape(
        self: EPA,
        start: math.Mat44,
        direction: math.Vec3,
        collision_tolerance: f32,
        penetration_tolerance: f32,
        a: ConvexSupport,
        b: ConvexSupport,
        convex_radius_a: f32,
        convex_radius_b: f32,
        return_deepest_point: bool,
        max_lambda: f32,
    ) err.Error!struct {
        hit: bool,
        lambda: f32,
        point_a: math.Vec3,
        point_b: math.Vec3,
        contact_normal: math.Vec3,
    } {
        var lambda = max_lambda;
        var point_a: math.Vec3 = undefined;
        var point_b: math.Vec3 = undefined;
        var contact_normal: math.Vec3 = undefined;
        var hit: bool = undefined;
        try err.check(c.zjoltEPACastShape(
            self.handle,
            &start,
            &direction,
            collision_tolerance,
            penetration_tolerance,
            &a,
            &b,
            convex_radius_a,
            convex_radius_b,
            return_deepest_point,
            &lambda,
            &point_a,
            &point_b,
            &contact_normal,
            &hit,
        ));
        return .{
            .hit = hit,
            .lambda = lambda,
            .point_a = point_a,
            .point_b = point_b,
            .contact_normal = contact_normal,
        };
    }
};

//=============================================================================
// EPA's own hull builder -- Jolt/Geometry/EPAConvexHullBuilder.h
//
// The incremental hull EPA grows around the Minkowski difference while it
// searches for the origin. Adding a point to the hull itself is not exposed
// here; EPA's own bound entry points already run that step internally.
//=============================================================================

/// One edge of an `EPATriangle`, read out by value.
pub const EPAEdge = struct {
    /// `null` if this edge has no neighbour linked yet.
    neighbour_triangle: ?EPATriangle,
    /// Edge index on `neighbour_triangle`; meaningless if it is `null`.
    neighbour_edge: i32,
    /// Vertex index, into the owning builder's positions, at this edge's start.
    start_idx: u32,
};

/// One triangle of an `EPAConvexHullBuilder`'s hull. Borrowed: valid until
/// freed with `EPAConvexHullBuilder.freeTriangle` or its builder is
/// destroyed, whichever comes first.
pub const EPATriangle = struct {
    handle: *c.EPATriangle,

    pub fn isFacing(self: EPATriangle, position: math.Vec3) bool {
        return c.zjoltEPATriangleIsFacing(self.handle, &position);
    }

    pub fn isFacingOrigin(self: EPATriangle) bool {
        return c.zjoltEPATriangleIsFacingOrigin(self.handle);
    }

    /// This triangle's edge that follows edge `index` around it, (`index` +
    /// 1) % 3. `index` must be < 3.
    pub fn nextEdge(self: EPATriangle, index: u32) err.Error!EPAEdge {
        var out: c.EPAEdge = undefined;
        try err.check(c.zjoltEPATriangleGetNextEdge(self.handle, index, &out));
        return .{
            .neighbour_triangle = if (out.neighbour_triangle) |t| EPATriangle{ .handle = t } else null,
            .neighbour_edge = out.neighbour_edge,
            .start_idx = out.start_idx,
        };
    }
};

pub const EPAConvexHullBuilder = struct {
    handle: *c.EPAConvexHullBuilder,

    /// `positions` is copied in; at most
    /// `c.epa_convex_hull_builder_max_points`.
    pub fn init(positions: []const math.Vec3) err.Error!EPAConvexHullBuilder {
        var handle: *c.EPAConvexHullBuilder = undefined;
        try err.check(c.zjoltEPAConvexHullBuilderCreate(positions.ptr, @intCast(positions.len), &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: EPAConvexHullBuilder) void {
        c.zjoltEPAConvexHullBuilderDestroy(self.handle);
    }

    /// Starts the hull as two triangles, back to back, over three distinct
    /// indices into the positions this builder was created with.
    pub fn initialize(self: EPAConvexHullBuilder, idx1: u32, idx2: u32, idx3: u32) err.Error!void {
        try err.check(c.zjoltEPAConvexHullBuilderInitialize(self.handle, idx1, idx2, idx3));
    }

    pub fn hasNextTriangle(self: EPAConvexHullBuilder) bool {
        return c.zjoltEPAConvexHullBuilderHasNextTriangle(self.handle);
    }

    /// The next closest triangle to the origin, without removing it from the
    /// queue. `null` if the queue is empty.
    pub fn peekClosestTriangle(self: EPAConvexHullBuilder) ?EPATriangle {
        const t = c.zjoltEPAConvexHullBuilderPeekClosestTriangle(self.handle) orelse return null;
        return .{ .handle = t };
    }

    /// As `peekClosestTriangle`, but removes it from the queue.
    pub fn popClosestTriangle(self: EPAConvexHullBuilder) ?EPATriangle {
        const t = c.zjoltEPAConvexHullBuilderPopClosestTriangle(self.handle) orelse return null;
        return .{ .handle = t };
    }

    /// The triangle `position` is furthest in front of, or `null` if
    /// `position` is behind every triangle still in the queue.
    pub fn findFacingTriangle(self: EPAConvexHullBuilder, position: math.Vec3) struct { triangle: ?EPATriangle, best_dist_sq: f32 } {
        var best_dist_sq: f32 = undefined;
        const t = c.zjoltEPAConvexHullBuilderFindFacingTriangle(self.handle, &position, &best_dist_sq);
        return .{ .triangle = if (t) |tt| EPATriangle{ .handle = tt } else null, .best_dist_sq = best_dist_sq };
    }

    /// Unlinks `triangle` from its neighbours and returns it to this
    /// builder's free list; `triangle` must not be used afterwards.
    pub fn freeTriangle(self: EPAConvexHullBuilder, triangle: EPATriangle) void {
        c.zjoltEPAConvexHullBuilderFreeTriangle(self.handle, triangle.handle);
    }
};

//=============================================================================
// Convex hull building
//=============================================================================

pub const ConvexHullBuilder = struct {
    handle: *c.ConvexHullBuilder,

    pub const Result = c.ConvexHullResult;
    pub const Face = c.ConvexHullFace;

    /// `positions` is copied in; the builder considers all of them,
    /// discarding interior points as it goes.
    pub fn init(positions: []const math.Vec3) err.Error!ConvexHullBuilder {
        var handle: *c.ConvexHullBuilder = undefined;
        try err.check(c.zjoltConvexHullBuilderCreate(positions.ptr, @intCast(positions.len), &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: ConvexHullBuilder) void {
        c.zjoltConvexHullBuilderDestroy(self.handle);
    }

    /// Builds the hull. `max_vertices` of 0 means no limit.
    /// `error.ShapeInvalid` for `.too_few_points`, `.too_few_faces` or
    /// `.degenerate`; `.success` and `.max_vertices_reached` both leave a
    /// usable hull.
    pub fn initialize(self: ConvexHullBuilder, max_vertices: u32, tolerance: f32) err.Error!Result {
        var result: Result = undefined;
        try err.check(c.zjoltConvexHullBuilderInitialize(self.handle, max_vertices, tolerance, &result));
        return result;
    }

    pub fn numVerticesUsed(self: ConvexHullBuilder) u32 {
        return c.zjoltConvexHullBuilderGetNumVerticesUsed(self.handle);
    }

    /// Whether the hull has a face with exactly these vertices (counter
    /// clockwise, indices into the positions this builder was created with).
    pub fn containsFace(self: ConvexHullBuilder, indices: []const u32) bool {
        return c.zjoltConvexHullBuilderContainsFace(self.handle, indices.ptr, @intCast(indices.len));
    }

    pub fn centerOfMassAndVolume(self: ConvexHullBuilder) err.Error!struct { center_of_mass: math.Vec3, volume: f32 } {
        var center_of_mass: math.Vec3 = undefined;
        var volume: f32 = undefined;
        try err.check(c.zjoltConvexHullBuilderGetCenterOfMassAndVolume(self.handle, &center_of_mass, &volume));
        return .{ .center_of_mass = center_of_mass, .volume = volume };
    }

    /// The point furthest outside the hull and how far, 0 for a perfectly
    /// built hull. `face_index`/`max_error_position_index` are -1 when there
    /// is no such face/point.
    pub fn determineMaxError(self: ConvexHullBuilder) err.Error!struct {
        face_index: i32,
        max_error: f32,
        max_error_position_index: i32,
        coplanar_distance: f32,
    } {
        var face_index: i32 = undefined;
        var max_error: f32 = undefined;
        var max_error_position_index: i32 = undefined;
        var coplanar_distance: f32 = undefined;
        try err.check(c.zjoltConvexHullBuilderDetermineMaxError(
            self.handle,
            &face_index,
            &max_error,
            &max_error_position_index,
            &coplanar_distance,
        ));
        return .{
            .face_index = face_index,
            .max_error = max_error,
            .max_error_position_index = max_error_position_index,
            .coplanar_distance = coplanar_distance,
        };
    }

    pub fn numFaces(self: ConvexHullBuilder) err.Error!u32 {
        var n: u32 = undefined;
        try err.check(c.zjoltConvexHullBuilderGetNumFaces(self.handle, &n));
        return n;
    }

    pub fn face(self: ConvexHullBuilder, face_index: u32) err.Error!Face {
        var f: Face = undefined;
        try err.check(c.zjoltConvexHullBuilderGetFace(self.handle, face_index, &f));
        return f;
    }

    /// `face_index`'s vertices, counter clockwise, as indices into the
    /// positions this builder was created with. `error.BufferTooSmall` if
    /// `buffer` does not fit -- size it with `face(face_index).num_vertices`
    /// first.
    pub fn faceVertices(self: ConvexHullBuilder, face_index: u32, buffer: []u32) err.Error![]u32 {
        var out_count: u32 = undefined;
        try err.check(c.zjoltConvexHullBuilderGetFaceVertices(
            self.handle,
            face_index,
            buffer.ptr,
            @intCast(buffer.len),
            &out_count,
        ));
        return buffer[0..out_count];
    }
};

/// The vertex before `loop[index]` in a counter-clockwise face loop --
/// `ConvexHullBuilder::Edge::GetPreviousEdge`, computed from the vertex
/// array a forward walk (`ConvexHullBuilder.faceVertices`) already gives,
/// rather than walking the edge list itself. `loop` must not be empty.
pub fn previousVertexInLoop(loop: []const u32, index: usize) u32 {
    return loop[(index + loop.len - 1) % loop.len];
}

/// Face normal (twice the face's area, unnormalized) and centroid over an
/// ordered loop of at least 3 vertices -- usable with the loop
/// `ConvexHullBuilder.faceVertices` returns, mapped back through the
/// original positions -- `ConvexHullBuilder::Face::CalculateNormalAndCentroid`.
pub fn calculateNormalAndCentroid(loop: []const math.Vec3) struct { normal: math.Vec3, centroid: math.Vec3 } {
    const y0 = loop[0];
    var y1 = loop[1];
    var centroid = vAdd(y0, y1);
    var normal = math.Vec3{ .x = 0, .y = 0, .z = 0 };
    var n: f32 = 2;

    for (loop[2..]) |y2| {
        const e0 = vSub(y1, y0);
        const e1 = vSub(y2, y1);
        const e2 = vSub(y0, y2);
        const shorter_e1 = vDot(e1, e1) < vDot(e2, e2);
        normal = vAdd(normal, if (shorter_e1) vCross(e0, e1) else vCross(e2, e0));
        centroid = vAdd(centroid, y2);
        n += 1;
        y1 = y2;
    }

    return .{ .normal = normal, .centroid = vScale(centroid, 1.0 / n) };
}

//=============================================================================
// 2D convex hull building
//=============================================================================

pub const ConvexHullBuilder2D = struct {
    handle: *c.ConvexHullBuilder2D,

    /// Uses only the X and Y of each position, matching the upstream type.
    pub fn init(positions: []const math.Vec3) err.Error!ConvexHullBuilder2D {
        var handle: *c.ConvexHullBuilder2D = undefined;
        try err.check(c.zjoltConvexHullBuilder2DCreate(positions.ptr, @intCast(positions.len), &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: ConvexHullBuilder2D) void {
        c.zjoltConvexHullBuilder2DDestroy(self.handle);
    }

    /// Builds the hull edge loop starting from `idx1`/`idx2`/`idx3` (in any
    /// order, indices into the positions this builder was created with).
    /// `max_vertices` of 0 means no limit. `buffer` receives the loop, counter
    /// clockwise, as indices into the same positions -- `error.BufferTooSmall`
    /// if it does not fit (a loop never has more edges than this builder has
    /// positions). `result` is `.success` or `.max_vertices_reached`.
    pub fn initialize(
        self: ConvexHullBuilder2D,
        idx1: u32,
        idx2: u32,
        idx3: u32,
        max_vertices: u32,
        tolerance: f32,
        buffer: []u32,
    ) err.Error!struct { edges: []u32, result: ConvexHullBuilder.Result } {
        var out_count: u32 = undefined;
        var result: ConvexHullBuilder.Result = undefined;
        try err.check(c.zjoltConvexHullBuilder2DInitialize(
            self.handle,
            idx1,
            idx2,
            idx3,
            max_vertices,
            tolerance,
            buffer.ptr,
            @intCast(buffer.len),
            &out_count,
            &result,
        ));
        return .{ .edges = buffer[0..out_count], .result = result };
    }
};

/// Edge normal (unnormalized, in the XY plane) and center point of the edge
/// from `edge_start` to `edge_end` -- two positions consecutive in a loop
/// `ConvexHullBuilder2D.initialize` returns --
/// `ConvexHullBuilder2D::Edge::CalculateNormalAndCenter`.
pub fn calculateNormalAndCenter2D(edge_start: math.Vec3, edge_end: math.Vec3) struct { normal: math.Vec3, center: math.Vec3 } {
    const edge = vSub(edge_end, edge_start);
    return .{
        .normal = .{ .x = edge.y, .y = -edge.x, .z = 0 },
        .center = vScale(vAdd(edge_start, edge_end), 0.5),
    };
}

//=============================================================================
// Polygon clipping
//=============================================================================

/// Clips `polygon` against the positive half space of the plane through
/// `plane_origin` with `plane_normal` (need not be unit length). `polygon`
/// needs at least 2 vertices. `error.BufferTooSmall` if `buffer` does not fit.
pub fn clipPolyVsPlane(
    polygon: []const math.Vec3,
    plane_origin: math.Vec3,
    plane_normal: math.Vec3,
    buffer: []math.Vec3,
) err.Error![]math.Vec3 {
    var out_count: u32 = undefined;
    try err.check(c.zjoltClipPolyVsPlane(
        polygon.ptr,
        @intCast(polygon.len),
        &plane_origin,
        &plane_normal,
        buffer.ptr,
        @intCast(buffer.len),
        &out_count,
    ));
    return buffer[0..out_count];
}

/// Clips `polygon` against `clipping_polygon`, both counter clockwise.
/// `polygon` needs at least 2 vertices, `clipping_polygon` at least 3.
pub fn clipPolyVsPoly(
    polygon: []const math.Vec3,
    clipping_polygon: []const math.Vec3,
    clipping_polygon_normal: math.Vec3,
    buffer: []math.Vec3,
) err.Error![]math.Vec3 {
    var out_count: u32 = undefined;
    try err.check(c.zjoltClipPolyVsPoly(
        polygon.ptr,
        @intCast(polygon.len),
        clipping_polygon.ptr,
        @intCast(clipping_polygon.len),
        &clipping_polygon_normal,
        buffer.ptr,
        @intCast(buffer.len),
        &out_count,
    ));
    return buffer[0..out_count];
}

/// Clips `polygon` (a planar polygon with at least 3 vertices) against an
/// edge projected onto its plane using `clipping_edge_normal`; the half
/// space in that normal's direction is cut away.
pub fn clipPolyVsEdge(
    polygon: []const math.Vec3,
    edge_vertex1: math.Vec3,
    edge_vertex2: math.Vec3,
    clipping_edge_normal: math.Vec3,
    buffer: []math.Vec3,
) err.Error![]math.Vec3 {
    var out_count: u32 = undefined;
    try err.check(c.zjoltClipPolyVsEdge(
        polygon.ptr,
        @intCast(polygon.len),
        &edge_vertex1,
        &edge_vertex2,
        &clipping_edge_normal,
        buffer.ptr,
        @intCast(buffer.len),
        &out_count,
    ));
    return buffer[0..out_count];
}

/// Clips `polygon` (counter clockwise, at least 2 vertices) against `box`,
/// keeping what is inside.
pub fn clipPolyVsAABox(
    polygon: []const math.Vec3,
    box: math.AABox,
    buffer: []math.Vec3,
) err.Error![]math.Vec3 {
    var out_count: u32 = undefined;
    try err.check(c.zjoltClipPolyVsAABox(
        polygon.ptr,
        @intCast(polygon.len),
        &box,
        buffer.ptr,
        @intCast(buffer.len),
        &out_count,
    ));
    return buffer[0..out_count];
}

//=============================================================================
// Triangle indexing
//=============================================================================

pub const IndexifyTriangle = c.IndexifyTriangle;
pub const IndexedTriangle = c.IndexedTriangle;

/// Jolt's own default weld distance. Lives here rather than in
/// `c/geometry.zig` because the ABI cross-check pairs constants by integer
/// value and cannot compare a float; pinned behaviourally instead, by a test
/// against `Jolt/Geometry/Indexify.h`'s own documented default.
pub const default_vertex_weld_distance: f32 = 1.0e-4;

/// Welds vertices within `vertex_weld_distance` of each other and builds an
/// indexed mesh from `triangles`; a triangle that degenerates once its
/// vertices are welded is dropped, so the result may have fewer triangles
/// than `triangles`. `error.BufferTooSmall` if either output does not fit.
pub fn indexify(
    triangles: []const IndexifyTriangle,
    vertex_weld_distance: f32,
    out_vertices: []math.Vec3,
    out_triangles: []IndexedTriangle,
) err.Error!struct { vertices: []math.Vec3, triangles: []IndexedTriangle } {
    var num_vertices: u32 = undefined;
    var num_triangles: u32 = undefined;
    try err.check(c.zjoltIndexify(
        triangles.ptr,
        @intCast(triangles.len),
        vertex_weld_distance,
        out_vertices.ptr,
        @intCast(out_vertices.len),
        &num_vertices,
        out_triangles.ptr,
        @intCast(out_triangles.len),
        &num_triangles,
    ));
    return .{ .vertices = out_vertices[0..num_vertices], .triangles = out_triangles[0..num_triangles] };
}

/// The inverse: unpacks `triangles` (indices into `vertices`) into a soup.
/// One output triangle per input triangle.
pub fn deindexify(
    vertices: []const math.Vec3,
    triangles: []const IndexedTriangle,
    out_triangles: []IndexifyTriangle,
) err.Error![]IndexifyTriangle {
    var out_count: u32 = undefined;
    try err.check(c.zjoltDeindexify(
        vertices.ptr,
        @intCast(vertices.len),
        triangles.ptr,
        @intCast(triangles.len),
        out_triangles.ptr,
        @intCast(out_triangles.len),
        &out_count,
    ));
    return out_triangles[0..out_count];
}

//=============================================================================
// IndexedTriangleNoMaterial -- Jolt/Geometry/IndexedTriangle.h
//=============================================================================

/// Three vertex indices, no material -- Jolt's `IndexedTriangleNoMaterial`.
pub const IndexedTriangleNoMaterial = extern struct {
    idx: [3]u32,

    /// True if `self` and `other` name the same three vertices but in
    /// reversed winding order.
    pub fn isOpposite(self: IndexedTriangleNoMaterial, other: IndexedTriangleNoMaterial) bool {
        return (self.idx[0] == other.idx[0] and self.idx[1] == other.idx[2] and self.idx[2] == other.idx[1]) or
            (self.idx[0] == other.idx[1] and self.idx[1] == other.idx[0] and self.idx[2] == other.idx[2]) or
            (self.idx[0] == other.idx[2] and self.idx[1] == other.idx[1] and self.idx[2] == other.idx[0]);
    }
};

//=============================================================================
// Ellipse -- Jolt/Geometry/Ellipse.h
//=============================================================================

/// An ellipse centered on the origin, radius `a` along X and `b` along Y.
pub const Ellipse = struct {
    a: f32,
    b: f32,

    /// True if (`x`, `y`) lies inside or on the ellipse -- `Ellipse::IsInside`.
    pub fn isInside(self: Ellipse, x: f32, y: f32) bool {
        const nx = x / self.a;
        const ny = y / self.b;
        return nx * nx + ny * ny <= 1.0;
    }
};
