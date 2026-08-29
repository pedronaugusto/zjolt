//! Behavioural tests for GJK, EPA, convex hull building, polygon clipping and
//! triangle indexing.
//!
//! `geometry.zig` is not reachable through `zjolt.zig` (see the module's own
//! report), so this imports it directly rather than through the umbrella the
//! rest of the suite uses.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const geometry = @import("geometry.zig");

//=============================================================================
// Support-function fixtures
//=============================================================================

const Sphere = struct { center: zjolt.Vec3, radius: f32 };

fn sphereSupport(user: ?*anyopaque, direction: zjolt.Vec3) callconv(.c) zjolt.Vec3 {
    const self: *const Sphere = @ptrCast(@alignCast(user.?));
    const len = @sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    if (len == 0) return self.center;
    const s = self.radius / len;
    return .{
        .x = self.center.x + direction.x * s,
        .y = self.center.y + direction.y * s,
        .z = self.center.z + direction.z * s,
    };
}

fn sphere(s: *const Sphere) geometry.ConvexSupport {
    return .{ .support = sphereSupport, .user = @constCast(s) };
}

const Box = struct { center: zjolt.Vec3, half_extent: zjolt.Vec3 };

fn boxSupport(user: ?*anyopaque, direction: zjolt.Vec3) callconv(.c) zjolt.Vec3 {
    const self: *const Box = @ptrCast(@alignCast(user.?));
    return .{
        .x = self.center.x + (if (direction.x >= 0) self.half_extent.x else -self.half_extent.x),
        .y = self.center.y + (if (direction.y >= 0) self.half_extent.y else -self.half_extent.y),
        .z = self.center.z + (if (direction.z >= 0) self.half_extent.z else -self.half_extent.z),
    };
}

fn box(b: *const Box) geometry.ConvexSupport {
    return .{ .support = boxSupport, .user = @constCast(b) };
}

fn vlen(v: zjolt.Vec3) f32 {
    return @sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

fn vsub(a: zjolt.Vec3, b: zjolt.Vec3) zjolt.Vec3 {
    return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
}

//=============================================================================
// GJK
//=============================================================================

test "GJK finds the analytic closest points and separation between two spheres" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Centres 5 apart, radii 1 and 2: surfaces are exactly 2 apart along X.
    const a = Sphere{ .center = zjolt.vec3(0, 0, 0), .radius = 1 };
    const b = Sphere{ .center = zjolt.vec3(5, 0, 0), .radius = 2 };

    const gjk = try geometry.GJK.init();
    defer gjk.deinit();

    const result = try gjk.closestPoints(sphere(&a), sphere(&b), 1.0e-4, std.math.floatMax(f32), zjolt.vec3(1, 0, 0));

    try std.testing.expectApproxEqAbs(@as(f32, 4.0), result.dist_sq, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), result.point_a.x, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), result.point_a.y, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), result.point_a.z, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), result.point_b.x, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), result.point_b.y, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), result.point_b.z, 1.0e-4);

    // The simplex GJK left behind reconstructs the same pair of points
    // independently of the session that computed them.
    const simplex = gjk.closestPointsSimplex();
    try std.testing.expect(simplex.num_points >= 1 and simplex.num_points <= 3);
    const recomputed = try geometry.calculatePointAAndB(simplex);
    try std.testing.expectApproxEqAbs(result.point_a.x, recomputed.point_a.x, 1.0e-4);
    try std.testing.expectApproxEqAbs(result.point_b.x, recomputed.point_b.x, 1.0e-4);
}

test "GJK reports intersection for two overlapping spheres" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const a = Sphere{ .center = zjolt.vec3(0, 0, 0), .radius = 1 };
    const b = Sphere{ .center = zjolt.vec3(1, 0, 0), .radius = 1 };

    const gjk = try geometry.GJK.init();
    defer gjk.deinit();

    const result = try gjk.intersects(sphere(&a), sphere(&b), 1.0e-4, zjolt.vec3(0, 0, 0));
    try std.testing.expect(result.intersects);

    const closest = try gjk.closestPoints(sphere(&a), sphere(&b), 1.0e-4, std.math.floatMax(f32), zjolt.vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), closest.dist_sq, 1.0e-4);
}

test "GJK ray cast, sweep test and shape cast run against a sphere" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const a = Sphere{ .center = zjolt.vec3(5, 0, 0), .radius = 1 };
    const gjk = try geometry.GJK.init();
    defer gjk.deinit();

    // A ray along +X from the origin hits the sphere's near surface at x = 4.
    const ray = try gjk.castRay(zjolt.vec3(0, 0, 0), zjolt.vec3(1, 0, 0), 1.0e-4, sphere(&a), 100.0);
    try std.testing.expect(ray.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), ray.lambda, 1.0e-3);

    const start = zjolt.mat44_identity;
    const other = Sphere{ .center = zjolt.vec3(0, 0, 0), .radius = 1 };
    const sweep = try gjk.intersectsSweep(start, zjolt.vec3(1, 0, 0), 1.0e-4, sphere(&other), sphere(&a), 100.0);
    try std.testing.expect(sweep.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), sweep.lambda, 1.0e-3);

    const cast = try gjk.castShape(start, zjolt.vec3(1, 0, 0), 1.0e-4, sphere(&other), sphere(&a), 0, 0, 100.0);
    try std.testing.expect(cast.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), cast.lambda, 1.0e-3);
}

//=============================================================================
// GJK's plane and tetrahedron predicates, and its point-set updates
//=============================================================================

test "originOutsideOfPlane classifies the origin against the plane x + y + z = 1" {
    const a = zjolt.vec3(1, 0, 0);
    const b = zjolt.vec3(0, 1, 0);
    const c = zjolt.vec3(0, 0, 1);

    // D on the far side (x+y+z=6 > 1): the origin (x+y+z=0) is on the
    // opposite side from D, so it is outside relative to D.
    try std.testing.expect(geometry.originOutsideOfPlane(a, b, c, zjolt.vec3(2, 2, 2)));

    // D on the near side (x+y+z=-3 < 1): the origin is on the same side as
    // D, so it is not outside relative to D.
    try std.testing.expect(!geometry.originOutsideOfPlane(a, b, c, zjolt.vec3(-1, -1, -1)));
}

test "originOutsideOfTetrahedronPlanes agrees with originOutsideOfPlane, all false for a tetrahedron centered on the origin" {
    // A regular tetrahedron whose centroid is exactly the origin.
    const a = zjolt.vec3(1, 1, 1);
    const b = zjolt.vec3(-1, -1, 1);
    const c = zjolt.vec3(-1, 1, -1);
    const d = zjolt.vec3(1, -1, -1);

    try std.testing.expect(!geometry.originOutsideOfPlane(a, b, c, d));

    const planes = geometry.originOutsideOfTetrahedronPlanes(a, b, c, d);
    try std.testing.expectEqual(geometry.originOutsideOfPlane(a, b, c, d), planes[0]);
    for (planes) |outside| try std.testing.expect(!outside);
}

test "Simplex.updatePointSetY/P/PQ/YPQ compact a real GJK simplex the same way" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Two overlapping spheres: GJK's closest-points search leaves behind a
    // simplex with at least 2 points to compact.
    const a = Sphere{ .center = zjolt.vec3(0, 0, 0), .radius = 1 };
    const b = Sphere{ .center = zjolt.vec3(0.5, 0, 0), .radius = 1 };
    const gjk = try geometry.GJK.init();
    defer gjk.deinit();
    _ = try gjk.closestPoints(sphere(&a), sphere(&b), 1.0e-4, std.math.floatMax(f32), zjolt.vec3(1, 0, 0));

    const original = gjk.closestPointsSimplex();
    try std.testing.expect(original.num_points >= 1);

    // Keeping every point (all bits set) must not change the count or any
    // kept value.
    var kept_all = original;
    kept_all.updatePointSetYPQ((@as(u32, 1) << @intCast(original.num_points)) - 1);
    try std.testing.expectEqual(original.num_points, kept_all.num_points);
    for (0..original.num_points) |i| {
        try std.testing.expectEqual(original.y[i].x, kept_all.y[i].x);
        try std.testing.expectEqual(original.p[i].x, kept_all.p[i].x);
        try std.testing.expectEqual(original.q[i].x, kept_all.q[i].x);
    }

    // Keeping only point 0 must leave exactly one point, equal to the
    // original's point 0, for every one of the four variants.
    var only_y = original;
    only_y.updatePointSetY(0b1);
    try std.testing.expectEqual(@as(u32, 1), only_y.num_points);
    try std.testing.expectEqual(original.y[0].x, only_y.y[0].x);

    var only_p = original;
    only_p.updatePointSetP(0b1);
    try std.testing.expectEqual(@as(u32, 1), only_p.num_points);
    try std.testing.expectEqual(original.p[0].x, only_p.p[0].x);

    var only_pq = original;
    only_pq.updatePointSetPQ(0b1);
    try std.testing.expectEqual(@as(u32, 1), only_pq.num_points);
    try std.testing.expectEqual(original.p[0].x, only_pq.p[0].x);
    try std.testing.expectEqual(original.q[0].x, only_pq.q[0].x);

    var only_ypq = original;
    only_ypq.updatePointSetYPQ(0b1);
    try std.testing.expectEqual(@as(u32, 1), only_ypq.num_points);
    try std.testing.expectEqual(original.y[0].x, only_ypq.y[0].x);
    try std.testing.expectEqual(original.p[0].x, only_ypq.p[0].x);
    try std.testing.expectEqual(original.q[0].x, only_ypq.q[0].x);
}

//=============================================================================
// EPA
//=============================================================================

test "EPA finds the analytic penetration axis and depth between two overlapping boxes" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A spans x in [-1, 1]; B spans x in [0.5, 2.5]. Y and Z fully overlap
    // (depth 2 each), so X is the axis of minimum penetration, depth 0.5.
    const a = Box{ .center = zjolt.vec3(0, 0, 0), .half_extent = zjolt.vec3(1, 1, 1) };
    const b = Box{ .center = zjolt.vec3(1.5, 0, 0), .half_extent = zjolt.vec3(1, 1, 1) };

    const epa = try geometry.EPA.init();
    defer epa.deinit();

    const result = try epa.penetrationDepth(
        box(&a),
        box(&a),
        0,
        box(&b),
        box(&b),
        0,
        1.0e-4,
        1.0e-4,
        zjolt.vec3(1, 0, 0),
    );
    try std.testing.expect(result.collided);

    const depth = vlen(vsub(result.point_b, result.point_a));
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), depth, 1.0e-3);

    const axis = vsub(result.point_b, result.point_a);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), axis.y, 1.0e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), axis.z, 1.0e-3);
    try std.testing.expectApproxEqAbs(depth, @abs(axis.x), 1.0e-3);
}

test "EPA's two-step form agrees with the one-call convenience function" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const a = Box{ .center = zjolt.vec3(0, 0, 0), .half_extent = zjolt.vec3(1, 1, 1) };
    const b = Box{ .center = zjolt.vec3(1.5, 0, 0), .half_extent = zjolt.vec3(1, 1, 1) };

    const epa = try geometry.EPA.init();
    defer epa.deinit();

    const step_gjk = try epa.penetrationDepthStepGJK(box(&a), 0, box(&b), 0, 1.0e-4, zjolt.vec3(1, 0, 0));
    try std.testing.expectEqual(geometry.EPA.Status.indeterminate, step_gjk.status);

    const step_epa = try epa.penetrationDepthStepEPA(box(&a), box(&b), 1.0e-4);
    try std.testing.expect(step_epa.collided);
    const depth = vlen(vsub(step_epa.point_b, step_epa.point_a));
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), depth, 1.0e-3);

    const start = zjolt.mat44_identity;
    const cast = try epa.castShape(start, zjolt.vec3(0, 1, 0), 1.0e-4, 1.0e-4, box(&a), box(&b), 0, 0, true, 0.0);
    try std.testing.expect(cast.hit);
}

//=============================================================================
// EPA's own hull builder
//=============================================================================

test "EPAConvexHullBuilder.initialize starts two fully-linked triangles, exactly one facing the origin" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Plane x + y + z = 1 does not pass through the origin, so the two
    // back-to-back triangles built over it -- opposite in winding, so
    // opposite in normal direction -- disagree about facing the origin.
    const points = [_]zjolt.Vec3{ zjolt.vec3(1, 0, 0), zjolt.vec3(0, 1, 0), zjolt.vec3(0, 0, 1) };

    const builder = try geometry.EPAConvexHullBuilder.init(&points);
    defer builder.deinit();
    try builder.initialize(0, 1, 2);

    try std.testing.expect(builder.hasNextTriangle());
    const peeked = builder.peekClosestTriangle().?;
    _ = peeked.isFacing(zjolt.vec3(10, 10, 10)); // must not crash on a live triangle

    // A point far outside the hull is in front of one of the two triangles
    // still in the queue -- both are found while both remain in it.
    const facing = builder.findFacingTriangle(zjolt.vec3(10, 10, 10));
    try std.testing.expect(facing.triangle != null);
    try std.testing.expect(facing.best_dist_sq > 0);

    const t1 = builder.popClosestTriangle().?;
    try std.testing.expect(builder.hasNextTriangle());
    const t2 = builder.popClosestTriangle().?;
    try std.testing.expect(!builder.hasNextTriangle());
    try std.testing.expectEqual(@as(?geometry.EPATriangle, null), builder.popClosestTriangle());

    try std.testing.expect(t1.isFacingOrigin() != t2.isFacingOrigin());

    // Every edge of the initial hull is linked to the other triangle, and
    // the three start indices form a permutation of {0, 1, 2}.
    var seen = [_]bool{ false, false, false };
    for (0..3) |i| {
        const edge = try t1.nextEdge(@intCast(i));
        try std.testing.expect(edge.neighbour_triangle != null);
        try std.testing.expect(edge.start_idx < 3);
        seen[edge.start_idx] = true;
    }
    for (seen) |s| try std.testing.expect(s);

    builder.freeTriangle(t1);
    builder.freeTriangle(t2);
}

//=============================================================================
// Convex hull building
//=============================================================================

test "the hull of a unit cube's corners has 6 faces, volume 1, and centroid at its centre" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const corners = [_]zjolt.Vec3{
        zjolt.vec3(-0.5, -0.5, -0.5), zjolt.vec3(0.5, -0.5, -0.5),
        zjolt.vec3(-0.5, 0.5, -0.5),  zjolt.vec3(0.5, 0.5, -0.5),
        zjolt.vec3(-0.5, -0.5, 0.5),  zjolt.vec3(0.5, -0.5, 0.5),
        zjolt.vec3(-0.5, 0.5, 0.5),   zjolt.vec3(0.5, 0.5, 0.5),
    };

    const builder = try geometry.ConvexHullBuilder.init(&corners);
    defer builder.deinit();

    const result = try builder.initialize(0, 1.0e-4);
    try std.testing.expectEqual(geometry.ConvexHullBuilder.Result.success, result);

    try std.testing.expectEqual(@as(u32, 8), builder.numVerticesUsed());

    const num_faces = try builder.numFaces();
    try std.testing.expectEqual(@as(u32, 6), num_faces);

    const cm_vol = try builder.centerOfMassAndVolume();
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), cm_vol.volume, 1.0e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), cm_vol.center_of_mass.x, 1.0e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), cm_vol.center_of_mass.y, 1.0e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), cm_vol.center_of_mass.z, 1.0e-3);

    const max_error = try builder.determineMaxError();
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), max_error.max_error, 1.0e-3);

    var indices_buf: [16]u32 = undefined;
    var f: u32 = 0;
    var total_face_vertices: usize = 0;
    while (f < num_faces) : (f += 1) {
        const face_info = try builder.face(f);
        try std.testing.expect(face_info.num_vertices >= 3);
        const verts = try builder.faceVertices(f, &indices_buf);
        try std.testing.expectEqual(face_info.num_vertices, @as(u32, @intCast(verts.len)));
        total_face_vertices += verts.len;
        try std.testing.expect(builder.containsFace(verts));
    }
    // A cube's hull is 6 quads (or triangulated equivalently) -- at least
    // 4 vertices per face, so at least 24 total.
    try std.testing.expect(total_face_vertices >= 24);
}

test "calculateNormalAndCentroid and previousVertexInLoop agree with the cube hull's own faces" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const corners = [_]zjolt.Vec3{
        zjolt.vec3(-0.5, -0.5, -0.5), zjolt.vec3(0.5, -0.5, -0.5),
        zjolt.vec3(-0.5, 0.5, -0.5),  zjolt.vec3(0.5, 0.5, -0.5),
        zjolt.vec3(-0.5, -0.5, 0.5),  zjolt.vec3(0.5, -0.5, 0.5),
        zjolt.vec3(-0.5, 0.5, 0.5),   zjolt.vec3(0.5, 0.5, 0.5),
    };

    const builder = try geometry.ConvexHullBuilder.init(&corners);
    defer builder.deinit();
    _ = try builder.initialize(0, 1.0e-4);

    const num_faces = try builder.numFaces();
    var indices_buf: [16]u32 = undefined;
    var loop_buf: [16]zjolt.Vec3 = undefined;

    var f: u32 = 0;
    while (f < num_faces) : (f += 1) {
        const face_info = try builder.face(f);
        const verts = try builder.faceVertices(f, &indices_buf);
        for (verts, 0..) |idx, i| loop_buf[i] = corners[idx];

        const recomputed = geometry.calculateNormalAndCentroid(loop_buf[0..verts.len]);
        try std.testing.expectApproxEqAbs(face_info.normal.x, recomputed.normal.x, 1.0e-3);
        try std.testing.expectApproxEqAbs(face_info.normal.y, recomputed.normal.y, 1.0e-3);
        try std.testing.expectApproxEqAbs(face_info.normal.z, recomputed.normal.z, 1.0e-3);
        try std.testing.expectApproxEqAbs(face_info.centroid.x, recomputed.centroid.x, 1.0e-3);
        try std.testing.expectApproxEqAbs(face_info.centroid.y, recomputed.centroid.y, 1.0e-3);
        try std.testing.expectApproxEqAbs(face_info.centroid.z, recomputed.centroid.z, 1.0e-3);

        // The vertex before index 0 in the loop is the loop's own last
        // vertex; the vertex before any other index i is index i - 1.
        try std.testing.expectEqual(verts[verts.len - 1], geometry.previousVertexInLoop(verts, 0));
        for (1..verts.len) |i| {
            try std.testing.expectEqual(verts[i - 1], geometry.previousVertexInLoop(verts, i));
        }
    }
}

test "the 2D hull builder produces a counter clockwise loop over a square" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A unit square in the XY plane (Z ignored by the 2D builder) plus an
    // interior point that must not survive into the hull.
    const points = [_]zjolt.Vec3{
        zjolt.vec3(0, 0, 0),
        zjolt.vec3(1, 0, 0),
        zjolt.vec3(1, 1, 0),
        zjolt.vec3(0, 1, 0),
        zjolt.vec3(0.5, 0.5, 0),
    };

    const builder2d = try geometry.ConvexHullBuilder2D.init(&points);
    defer builder2d.deinit();

    var edge_buf: [8]u32 = undefined;
    const result = try builder2d.initialize(0, 1, 2, 0, 1.0e-4, &edge_buf);
    try std.testing.expectEqual(geometry.ConvexHullBuilder.Result.success, result.result);
    try std.testing.expectEqual(@as(usize, 4), result.edges.len);
    for (result.edges) |idx| try std.testing.expect(idx != 4);
}

test "calculateNormalAndCenter2D produces outward-pointing normals for the 2D hull's own loop" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const square = [_]zjolt.Vec3{
        zjolt.vec3(0, 0, 0),
        zjolt.vec3(1, 0, 0),
        zjolt.vec3(1, 1, 0),
        zjolt.vec3(0, 1, 0),
    };

    const builder2d = try geometry.ConvexHullBuilder2D.init(&square);
    defer builder2d.deinit();

    var edge_buf: [8]u32 = undefined;
    const result = try builder2d.initialize(0, 1, 2, 0, 1.0e-4, &edge_buf);
    try std.testing.expectEqual(@as(usize, 4), result.edges.len);

    const center = zjolt.vec3(0.5, 0.5, 0);
    for (result.edges, 0..) |idx, i| {
        const start = square[idx];
        const end = square[result.edges[(i + 1) % result.edges.len]];
        const info = geometry.calculateNormalAndCenter2D(start, end);

        try std.testing.expectApproxEqAbs(0.5 * (start.x + end.x), info.center.x, 1.0e-4);
        try std.testing.expectApproxEqAbs(0.5 * (start.y + end.y), info.center.y, 1.0e-4);

        // The normal points away from the square's own center.
        const to_edge_x = info.center.x - center.x;
        const to_edge_y = info.center.y - center.y;
        try std.testing.expect(info.normal.x * to_edge_x + info.normal.y * to_edge_y > 0);
    }
}

//=============================================================================
// Polygon clipping
//=============================================================================

test "ClipPolyVsPlane clips a straddling triangle and reports BufferTooSmall for a short buffer" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A triangle in the Z = 0 plane straddling the plane x = 0: one vertex on
    // the negative side, two on the positive side that is kept.
    const triangle = [_]zjolt.Vec3{
        zjolt.vec3(-1, 0, 0),
        zjolt.vec3(1, -1, 0),
        zjolt.vec3(1, 1, 0),
    };
    // A point is kept when dot(normal, point - origin) > 0, so a normal of
    // +X with the origin at x = 0 keeps x >= 0.
    const plane_origin = zjolt.vec3(0, 0, 0);
    const plane_normal = zjolt.vec3(1, 0, 0);

    var buffer: [8]zjolt.Vec3 = undefined;
    const clipped = try geometry.clipPolyVsPlane(&triangle, plane_origin, plane_normal, &buffer);

    // The two kept vertices plus two new ones where the removed edges cross
    // x = 0: a quad.
    try std.testing.expectEqual(@as(usize, 4), clipped.len);
    for (clipped) |v| try std.testing.expect(v.x >= -1.0e-4);

    var short_buffer: [3]zjolt.Vec3 = undefined;
    try std.testing.expectError(error.BufferTooSmall, geometry.clipPolyVsPlane(&triangle, plane_origin, plane_normal, &short_buffer));
}

test "ClipPolyVsPoly, ClipPolyVsEdge and ClipPolyVsAABox run over a square" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const square = [_]zjolt.Vec3{
        zjolt.vec3(-1, -1, 0), zjolt.vec3(1, -1, 0), zjolt.vec3(1, 1, 0), zjolt.vec3(-1, 1, 0),
    };
    const clip_square = [_]zjolt.Vec3{
        zjolt.vec3(-0.5, -0.5, 0), zjolt.vec3(0.5, -0.5, 0), zjolt.vec3(0.5, 0.5, 0), zjolt.vec3(-0.5, 0.5, 0),
    };

    var buffer: [16]zjolt.Vec3 = undefined;
    const vs_poly = try geometry.clipPolyVsPoly(&square, &clip_square, zjolt.vec3(0, 0, 1), &buffer);
    try std.testing.expect(vs_poly.len >= 3);
    for (vs_poly) |v| try std.testing.expect(@abs(v.x) <= 0.5 + 1.0e-3 and @abs(v.y) <= 0.5 + 1.0e-3);

    // The edge (0, -10, 0)-(0, 10, 0) lies in the polygon's own plane, so
    // its projection is a no-op; crossed with a (0, 0, 1) clipping normal
    // it gives an effective cut plane through x = 0 with normal (-20, 0, 0).
    // Unlike ClipPolyVsPlane, ClipPolyVsEdge reports only the two points
    // where the polygon's boundary crosses that plane, not vertices
    // already on the kept side (no "point already inside, keep it" step).
    const vs_edge = try geometry.clipPolyVsEdge(&square, zjolt.vec3(0, -10, 0), zjolt.vec3(0, 10, 0), zjolt.vec3(0, 0, 1), &buffer);
    try std.testing.expectEqual(@as(usize, 2), vs_edge.len);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), vs_edge[0].x, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), vs_edge[0].y, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), vs_edge[1].x, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), vs_edge[1].y, 1.0e-4);

    const box_bounds = zjolt.AABox{ .min = zjolt.vec3(-0.5, -0.5, -1), .max = zjolt.vec3(0.5, 0.5, 1) };
    const vs_box = try geometry.clipPolyVsAABox(&square, box_bounds, &buffer);
    try std.testing.expect(vs_box.len >= 3);
    for (vs_box) |v| try std.testing.expect(@abs(v.x) <= 0.5 + 1.0e-3 and @abs(v.y) <= 0.5 + 1.0e-3);
}

//=============================================================================
// Triangle indexing
//=============================================================================

test "Indexify then Deindexify round-trips a triangle soup" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Two triangles sharing an edge exactly: welding merges the shared
    // vertices without moving them, so Deindexify recovers the same
    // positions the soup started with.
    const soup = [_]geometry.IndexifyTriangle{
        .{
            .v = .{ zjolt.vec3(0, 0, 0), zjolt.vec3(1, 0, 0), zjolt.vec3(0, 1, 0) },
            .material_index = 1,
            .user_data = 11,
        },
        .{
            .v = .{ zjolt.vec3(1, 0, 0), zjolt.vec3(1, 1, 0), zjolt.vec3(0, 1, 0) },
            .material_index = 2,
            .user_data = 22,
        },
    };

    var vertex_buf: [8]zjolt.Vec3 = undefined;
    var tri_buf: [8]geometry.IndexedTriangle = undefined;
    const indexed = try geometry.indexify(&soup, geometry.default_vertex_weld_distance, &vertex_buf, &tri_buf);

    // Four distinct corners shared by two triangles that weld exactly.
    try std.testing.expectEqual(@as(usize, 4), indexed.vertices.len);
    try std.testing.expectEqual(@as(usize, 2), indexed.triangles.len);

    var out_buf: [8]geometry.IndexifyTriangle = undefined;
    const roundtrip = try geometry.deindexify(indexed.vertices, indexed.triangles, &out_buf);
    try std.testing.expectEqual(@as(usize, 2), roundtrip.len);

    for (roundtrip, 0..) |tri, t| {
        try std.testing.expectEqual(soup[t].material_index, tri.material_index);
        try std.testing.expectEqual(soup[t].user_data, tri.user_data);
        for (0..3) |i| {
            try std.testing.expectApproxEqAbs(soup[t].v[i].x, tri.v[i].x, 1.0e-6);
            try std.testing.expectApproxEqAbs(soup[t].v[i].y, tri.v[i].y, 1.0e-6);
            try std.testing.expectApproxEqAbs(soup[t].v[i].z, tri.v[i].z, 1.0e-6);
        }
    }
}

test "IndexedTriangleNoMaterial.isOpposite recognizes reversed winding, not any permutation" {
    const t = geometry.IndexedTriangleNoMaterial{ .idx = .{ 1, 2, 3 } };

    // Every cyclic rotation of the reversed winding is opposite.
    try std.testing.expect(t.isOpposite(.{ .idx = .{ 1, 3, 2 } }));
    try std.testing.expect(t.isOpposite(.{ .idx = .{ 3, 2, 1 } }));
    try std.testing.expect(t.isOpposite(.{ .idx = .{ 2, 1, 3 } }));

    // The same winding, even rotated, is not opposite.
    try std.testing.expect(!t.isOpposite(.{ .idx = .{ 1, 2, 3 } }));
    try std.testing.expect(!t.isOpposite(.{ .idx = .{ 2, 3, 1 } }));

    // An unrelated triangle is neither.
    try std.testing.expect(!t.isOpposite(.{ .idx = .{ 4, 5, 6 } }));
}

//=============================================================================
// Ellipse -- Jolt/Geometry/Ellipse.h
//=============================================================================

test "Ellipse.isInside matches the analytic (x/a)^2 + (y/b)^2 <= 1" {
    const e = geometry.Ellipse{ .a = 2, .b = 1 };

    try std.testing.expect(e.isInside(0, 0)); // center
    try std.testing.expect(e.isInside(2, 0)); // on the boundary, +X
    try std.testing.expect(e.isInside(0, 1)); // on the boundary, +Y
    try std.testing.expect(!e.isInside(2.1, 0));
    try std.testing.expect(!e.isInside(0, 1.1));
    try std.testing.expect(!e.isInside(2, 1)); // outside both axes at once
}

//=============================================================================
// The support-function seam's own guard, and its adapters
//=============================================================================

test "a NULL support callback is refused at the entry point" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const good = Sphere{ .center = zjolt.vec3(0, 0, 0), .radius = 1 };
    const missing = geometry.ConvexSupport{ .support = null, .user = null };

    const gjk = try geometry.GJK.init();
    defer gjk.deinit();
    try std.testing.expectError(error.InvalidArgument, gjk.intersects(sphere(&good), missing, 1.0e-4, zjolt.vec3(0, 0, 0)));

    const epa = try geometry.EPA.init();
    defer epa.deinit();
    try std.testing.expectError(
        error.InvalidArgument,
        epa.penetrationDepthStepGJK(sphere(&good), 0, missing, 0, 1.0e-4, zjolt.vec3(1, 0, 0)),
    );

    try std.testing.expectError(
        error.InvalidArgument,
        geometry.ConvexSupportAdapter.initAddConvexRadius(missing, 1.0),
    );
}

test "the convex-support adapters reproduce their Jolt originals' support point" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const inner = Sphere{ .center = zjolt.vec3(0, 0, 0), .radius = 1 };

    // Translated by (10, 0, 0): support in +X moves from (1, 0, 0) to (11, 0,
    // 0).
    var transform = zjolt.mat44_identity;
    transform.m[12] = 10;
    const transformed = try geometry.ConvexSupportAdapter.initTransformed(transform, sphere(&inner));
    defer transformed.deinit();
    const transformed_support = transformed.asSupport();
    const p1 = transformed_support.support.?(transformed_support.user, zjolt.vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 11.0), p1.x, 1.0e-4);

    // Rounded by 0.5: support in +X moves from (1, 0, 0) to (1.5, 0, 0).
    const rounded = try geometry.ConvexSupportAdapter.initAddConvexRadius(sphere(&inner), 0.5);
    defer rounded.deinit();
    const rounded_support = rounded.asSupport();
    const p2 = rounded_support.support.?(rounded_support.user, zjolt.vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), p2.x, 1.0e-4);

    // Minkowski difference of a sphere with itself in +X: 1 - (-1) = 2.
    const diff = try geometry.ConvexSupportAdapter.initMinkowskiDifference(sphere(&inner), sphere(&inner));
    defer diff.deinit();
    const diff_support = diff.asSupport();
    const p3 = diff_support.support.?(diff_support.user, zjolt.vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), p3.x, 1.0e-4);

    const poly_points = [_]zjolt.Vec3{ zjolt.vec3(0, 0, 0), zjolt.vec3(3, 0, 0), zjolt.vec3(0, 3, 0) };
    const poly = try geometry.ConvexSupportAdapter.initPolygon(&poly_points);
    defer poly.deinit();
    const poly_support = poly.asSupport();
    const p4 = poly_support.support.?(poly_support.user, zjolt.vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), p4.x, 1.0e-4);

    const tri = try geometry.ConvexSupportAdapter.initTriangle(zjolt.vec3(0, 0, 0), zjolt.vec3(3, 0, 0), zjolt.vec3(0, 3, 0));
    defer tri.deinit();
    const tri_support = tri.asSupport();
    const p5 = tri_support.support.?(tri_support.user, zjolt.vec3(0, 1, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), p5.y, 1.0e-4);
}
