//! Behavioural tests for scale helpers, triangle collision and ghost-collision
//! removal — three surfaces with no coverage until now.
//!
//! The scale tests need no physics system, no shapes, not even `zjolt.init`:
//! `scale.zig` is pure `Vec3`/`Quat` arithmetic, ported rather than bound. The
//! triangle-collision and internal-edge-removal tests do need `zjolt.init`,
//! since they cross the C ABI, but none of them need a `PhysicsSystem` —
//! every entry point under test here takes two placed shapes (or a shape and
//! a triangle a test hands it directly), never a body.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const scale = @import("scale.zig");
const collision = @import("collision.zig");
const query = @import("query.zig");
const err = @import("error.zig");
const c_collision = @import("c/collision.zig");

fn normalize(v: zjolt.Vec3) zjolt.Vec3 {
    const len = @sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return .{ .x = v.x / len, .y = v.y / len, .z = v.z / len };
}

fn quatAboutZ(radians: f32) zjolt.Quat {
    const half = radians / 2.0;
    return .{ .x = 0, .y = 0, .z = @sin(half), .w = @cos(half) };
}

//=============================================================================
// Scale helpers
//=============================================================================

test "isNotScaled, isUniformScale and isUniformScaleXZ read pairwise equality by hand, including a negative uniform scale" {
    try std.testing.expect(scale.isNotScaled(.{ .x = 1, .y = 1, .z = 1 }));
    try std.testing.expect(!scale.isNotScaled(.{ .x = 1.01, .y = 1, .z = 1 }));

    try std.testing.expect(scale.isUniformScale(.{ .x = 2, .y = 2, .z = 2 }));
    // A negative uniform scale is still uniform: every component agrees with
    // its neighbour, sign included.
    try std.testing.expect(scale.isUniformScale(.{ .x = -2, .y = -2, .z = -2 }));
    try std.testing.expect(!scale.isUniformScale(.{ .x = 2, .y = 3, .z = 2 }));

    // Uniform in X/Z only: Y is free to disagree.
    try std.testing.expect(scale.isUniformScaleXZ(.{ .x = 2, .y = 5, .z = 2 }));
    try std.testing.expect(!scale.isUniformScaleXZ(.{ .x = 2, .y = 5, .z = 3 }));
}

test "isZeroScale, makeUniformScale, makeUniformScaleXZ and convexRadius match hand computation, including a zero and a negative scale" {
    // Zero component.
    try std.testing.expect(scale.isZeroScale(.{ .x = 0, .y = 1, .z = 1 }));
    // Below min_scale (1e-6), positive and negative.
    try std.testing.expect(scale.isZeroScale(.{ .x = 1.0e-7, .y = 1, .z = 1 }));
    try std.testing.expect(scale.isZeroScale(.{ .x = -1.0e-7, .y = 1, .z = 1 }));
    try std.testing.expect(!scale.isZeroScale(.{ .x = 1, .y = 1, .z = 1 }));

    // (3 + 6 + 9) / 3 = 6, replicated.
    const uniform = scale.makeUniformScale(.{ .x = 3, .y = 6, .z = 9 });
    try std.testing.expectApproxEqAbs(@as(f32, 6), uniform.x, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 6), uniform.y, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 6), uniform.z, 1.0e-6);

    // (2 + 4) / 2 = 3 for X and Z; Y passes through untouched.
    const uniform_xz = scale.makeUniformScaleXZ(.{ .x = 2, .y = 10, .z = 4 });
    try std.testing.expectApproxEqAbs(@as(f32, 3), uniform_xz.x, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 10), uniform_xz.y, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 3), uniform_xz.z, 1.0e-6);

    // min(|2|,|3|,|4|) = 2: 0.1 * 2 = 0.2, capped at default_convex_radius (0.05).
    try std.testing.expectApproxEqAbs(@as(f32, 0.05), scale.convexRadius(0.1, .{ .x = 2, .y = 3, .z = 4 }), 1.0e-6);
    // 0.01 * 2 = 0.02, under the cap.
    try std.testing.expectApproxEqAbs(@as(f32, 0.02), scale.convexRadius(0.01, .{ .x = 2, .y = 3, .z = 4 }), 1.0e-6);
    // A negative component: min(|-2|,3,4) = 2, 0.05 * 2 = 0.1, still capped at 0.05.
    try std.testing.expectApproxEqAbs(@as(f32, 0.05), scale.convexRadius(0.05, .{ .x = -2, .y = 3, .z = 4 }), 1.0e-6);
}

test "rotateScale and canScaleBeRotated agree for identity and an axis-permuting rotation, and refuse a scale that would shear" {
    const identity: zjolt.Quat = zjolt.quat_identity;
    try std.testing.expect(scale.canScaleBeRotated(identity, .{ .x = 2, .y = 3, .z = 4 }));
    const via_identity = scale.rotateScale(identity, .{ .x = 2, .y = 3, .z = 4 });
    try std.testing.expectApproxEqAbs(@as(f32, 2), via_identity.x, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 3), via_identity.y, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 4), via_identity.z, 1.0e-6);

    // A 90-degree rotation about Z maps local X -> world Y and local Y ->
    // world -X, so it permutes the scale's X and Y components without
    // shearing: R^T diag(5,7,9) R is still diagonal, (7, 5, 9).
    const q90 = quatAboutZ(std.math.pi / 2.0);
    try std.testing.expect(scale.canScaleBeRotated(q90, .{ .x = 5, .y = 7, .z = 9 }));
    const via_90 = scale.rotateScale(q90, .{ .x = 5, .y = 7, .z = 9 });
    try std.testing.expectApproxEqAbs(@as(f32, 7), via_90.x, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 5), via_90.y, 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 9), via_90.z, 1.0e-4);

    // A 45-degree rotation about Z, with a non-uniform X/Y scale, mixes the
    // two axes instead of permuting them: by hand, the off-diagonal term of
    // R^T diag(1,4,1) R between the rotated X and Y axes works out to
    // -0.5*1 + 0.5*4 = 1.5, far past the 1e-6 tolerance a rotatable scale
    // needs.
    const q45 = quatAboutZ(std.math.pi / 4.0);
    try std.testing.expect(!scale.canScaleBeRotated(q45, .{ .x = 1, .y = 4, .z = 1 }));
}

test "transformScale short-circuits a uniform scale, and otherwise matches rotateScale" {
    const q45 = quatAboutZ(std.math.pi / 4.0);
    // Uniform: unchanged regardless of rotation, even one that would shear a
    // non-uniform scale.
    const uniform = scale.transformScale(q45, .{ .x = 5, .y = 5, .z = 5 });
    try std.testing.expectApproxEqAbs(@as(f32, 5), uniform.x, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 5), uniform.y, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 5), uniform.z, 1.0e-6);

    // Non-uniform, identity rotation: passes through unchanged, the same way
    // rotateScale does.
    const identity_case = scale.transformScale(zjolt.quat_identity, .{ .x = 2, .y = 3, .z = 4 });
    try std.testing.expectApproxEqAbs(@as(f32, 2), identity_case.x, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 3), identity_case.y, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 4), identity_case.z, 1.0e-6);
}

//=============================================================================
// Triangle collision
//=============================================================================

test "CollideConvexVsTriangles reports the expected penetration axis for an overlapping triangle, and no hit for one it misses" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A 2x2x2 box, convex radius zero for a crisp, exactly computable
    // silhouette.
    const box = try zjolt.Shape.initBox(zjolt.vec3(1, 1, 1), .{ .convex_radius = 0 });
    defer box.release();

    // A single huge triangle in the XZ plane at y = 0 -- far bigger than the
    // box's footprint, so the box's corners and edges never come near the
    // triangle's own boundary and this is a clean face contact.
    const v0 = zjolt.vec3(-50, 0, -50);
    const v1 = zjolt.vec3(50, 0, -50);
    const v2 = zjolt.vec3(0, 0, 50);

    const Recorder = struct {
        count: u32 = 0,
        axis: zjolt.Vec3 = undefined,
        depth: f32 = undefined,

        fn onHit(user: ?*anyopaque, hit: *const query.CollideShapeHit) callconv(.c) query.HitAction {
            const self: *@This() = @ptrCast(@alignCast(user.?));
            self.count += 1;
            self.axis = hit.penetration_axis;
            self.depth = hit.penetration_depth;
            return .@"continue";
        }
    };

    // Forced to collide with back faces so the test does not also have to
    // hand-verify the triangle's winding.
    const settings = query.CollideShapeSettings{ .back_face_mode = .collide };

    // Box centred at y = -0.9: it spans y in [-1.9, 0.1], so its top face
    // protrudes 0.1 above the triangle's plane and its bottom sits 1.9
    // below it. The shorter escape -- and so the reported penetration
    // depth -- is the smaller of the two: push it out by 0.1, straight
    // along the triangle's normal.
    var hit: Recorder = .{};
    var collider = try collision.CollideConvexVsTriangles.init(.{
        .shape1 = box,
        .position1 = zjolt.rvec3(0, -0.9, 0),
        .position2 = zjolt.rvec3(0, 0, 0),
    }, collision.empty_sub_shape_id, settings, Recorder.onHit, &hit);
    defer collider.deinit();

    const stopped = try collider.collide(v0, v1, v2, 0b111, collision.empty_sub_shape_id);
    try std.testing.expect(!stopped);
    try std.testing.expectEqual(@as(u32, 1), hit.count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), hit.depth, 5.0e-3);

    const n = normalize(hit.axis);
    try std.testing.expect(@abs(n.y) > 0.999);
    try std.testing.expect(@abs(n.x) < 0.05);
    try std.testing.expect(@abs(n.z) < 0.05);

    // Moved far above the triangle: no overlap, so on_hit never runs.
    var miss: Recorder = .{};
    var far_collider = try collision.CollideConvexVsTriangles.init(.{
        .shape1 = box,
        .position1 = zjolt.rvec3(0, 10, 0),
        .position2 = zjolt.rvec3(0, 0, 0),
    }, collision.empty_sub_shape_id, settings, Recorder.onHit, &miss);
    defer far_collider.deinit();

    const far_stopped = try far_collider.collide(v0, v1, v2, 0b111, collision.empty_sub_shape_id);
    try std.testing.expect(!far_stopped);
    try std.testing.expectEqual(@as(u32, 0), miss.count);
}

test "CollideConvexVsTriangles hands its callback the two faces it was asked for" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(1, 1, 1), .{ .convex_radius = 0 });
    defer box.release();

    // The same clean face contact as the test above: a box's flat bottom
    // against one big triangle, so both supporting faces are exact.
    const v0 = zjolt.vec3(-50, 0, -50);
    const v1 = zjolt.vec3(50, 0, -50);
    const v2 = zjolt.vec3(0, 0, 50);

    const Recorder = struct {
        count: u32 = 0,
        on_1: usize = 0,
        on_2: usize = 0,

        fn onHit(user: ?*anyopaque, hit: *const query.CollideShapeHit) callconv(.c) query.HitAction {
            const self: *@This() = @ptrCast(@alignCast(user.?));
            self.count += 1;
            self.on_1 = hit.faceOn1().len;
            self.on_2 = hit.faceOn2().len;
            return .@"continue";
        }
    };

    const settings = query.CollideShapeSettings{
        .back_face_mode = .collide,
        .collect_faces_mode = .collect_faces,
    };

    var hit: Recorder = .{};
    var collider = try collision.CollideConvexVsTriangles.init(.{
        .shape1 = box,
        .position1 = zjolt.rvec3(0, -0.9, 0),
        .position2 = zjolt.rvec3(0, 0, 0),
    }, collision.empty_sub_shape_id, settings, Recorder.onHit, &hit);
    defer collider.deinit();

    _ = try collider.collide(v0, v1, v2, 0b111, collision.empty_sub_shape_id);
    try std.testing.expectEqual(@as(u32, 1), hit.count);

    // A box face is a quad and a triangle is a triangle. Both are borrowed
    // for the callback only, so the lengths are recorded and the vertices are
    // not.
    try std.testing.expectEqual(@as(usize, 4), hit.on_1);
    try std.testing.expectEqual(@as(usize, 3), hit.on_2);
}

test "CastConvexVsTriangles hands its callback two faces, and none when it did not ask" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(1, 1, 1), .{ .convex_radius = 0 });
    defer box.release();

    const v0 = zjolt.vec3(-50, 0, -50);
    const v1 = zjolt.vec3(50, 0, -50);
    const v2 = zjolt.vec3(0, 0, 50);

    const Recorder = struct {
        count: u32 = 0,
        fraction: f32 = undefined,
        on_1: usize = 0,
        on_2: usize = 0,

        fn onHit(user: ?*anyopaque, hit: *const query.ShapeCastHit) callconv(.c) query.HitAction {
            const self: *@This() = @ptrCast(@alignCast(user.?));
            self.count += 1;
            self.fraction = hit.fraction;
            self.on_1 = hit.faceOn1().len;
            self.on_2 = hit.faceOn2().len;
            return .@"continue";
        }
    };

    // The box's flat bottom starts at y = 2 and the sweep is 4 long, so the
    // triangle at y = 0 is met halfway.
    const params = collision.TrianglesCast{
        .shape1 = box,
        .position1 = zjolt.rvec3(0, 3, 0),
        .direction = zjolt.vec3(0, -4, 0),
        .position2 = zjolt.rvec3(0, 0, 0),
    };

    // The triangle's winding puts its normal along -Y, so a sweep from above
    // arrives at its back face, exactly as in the overlap test above.
    const base = query.ShapeCastSettings{
        .back_face_mode_triangles = .collide,
        .back_face_mode_convex = .collide,
    };
    var with_faces = base;
    with_faces.collect_faces_mode = .collect_faces;

    var asked: Recorder = .{};
    var caster = try collision.CastConvexVsTriangles.init(params, with_faces, Recorder.onHit, &asked);
    defer caster.deinit();

    _ = try caster.cast(v0, v1, v2, 0b111, collision.empty_sub_shape_id);
    try std.testing.expectEqual(@as(u32, 1), asked.count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), asked.fraction, 1.0e-4);
    try std.testing.expectEqual(@as(usize, 4), asked.on_1);
    try std.testing.expectEqual(@as(usize, 3), asked.on_2);

    // The default, which is `no_faces`: the same hit arrives carrying nothing.
    var unasked: Recorder = .{};
    var plain = try collision.CastConvexVsTriangles.init(params, base, Recorder.onHit, &unasked);
    defer plain.deinit();

    _ = try plain.cast(v0, v1, v2, 0b111, collision.empty_sub_shape_id);
    try std.testing.expectEqual(@as(u32, 1), unasked.count);
    try std.testing.expectEqual(@as(usize, 0), unasked.on_1);
    try std.testing.expectEqual(@as(usize, 0), unasked.on_2);
}

test "CastSphereVsTriangles grazing a triangle edge reports the fraction computed by hand" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(0.5, .{});
    defer sphere.release();

    // A triangle in the XZ plane whose v0-v1 edge runs along the X axis at
    // y = 0, z = 0, from x = -1 to x = 1.
    const v0 = zjolt.vec3(-1, 0, 0);
    const v1 = zjolt.vec3(1, 0, 0);
    const v2 = zjolt.vec3(0, 0, 2);

    const Recorder = struct {
        count: u32 = 0,
        fraction: f32 = undefined,

        fn onHit(user: ?*anyopaque, hit: *const query.ShapeCastHit) callconv(.c) query.HitAction {
            const self: *@This() = @ptrCast(@alignCast(user.?));
            self.count += 1;
            self.fraction = hit.fraction;
            return .@"continue";
        }
    };

    // The sphere's (x, z) stays fixed at (0, -0.3); direction is pure -Y,
    // so the closest point on the v0-v1 edge to the centre is always
    // (0, 0, 0). At entry fraction t, the centre sits at y = 2.4 - 4t, and
    // the sphere's surface (radius 0.5) first reaches that edge point when
    // (2.4 - 4t)^2 + 0.3^2 = 0.5^2, i.e. 2.4 - 4t = sqrt(0.16) = 0.4
    // (positive: still approaching), giving t = (2.4 - 0.4) / 4 = 0.5.
    const settings = query.ShapeCastSettings{
        .back_face_mode_triangles = .collide,
        .back_face_mode_convex = .collide,
    };
    var rec: Recorder = .{};
    var caster = try collision.CastSphereVsTriangles.init(.{
        .shape1 = sphere,
        .position1 = zjolt.rvec3(0, 2.4, -0.3),
        .direction = zjolt.vec3(0, -4, 0),
        .position2 = zjolt.rvec3(0, 0, 0),
    }, settings, Recorder.onHit, &rec);
    defer caster.deinit();

    const stopped = try caster.cast(v0, v1, v2, 0b111, collision.empty_sub_shape_id);
    try std.testing.expect(!stopped);
    try std.testing.expectEqual(@as(u32, 1), rec.count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), rec.fraction, 1.0e-4);
}

//=============================================================================
// Ghost-collision removal
//
// The test that proves the feature: a flat floor of two coplanar triangles reports the SAME face contact for a box resting flat, whichever triangle is asked, so that configuration cannot demonstrate a fix. Jolt's InternalEdgeRemovingCollector exists for a contact whose closest feature, against ONE triangle, is that triangle's own edge or corner rather than its face — exactly what a box balanced corner-first produces here.
//=============================================================================

test "collideShapeWithInternalEdgeRemoval drops the ghost contact a naive shape-versus-shape query reports at a seam" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A flat 10x10 floor, split into two coplanar triangles along the
    // diagonal z = x.
    const verts = [_]zjolt.Vec3{
        zjolt.vec3(-5, 0, -5), zjolt.vec3(5, 0, -5),
        zjolt.vec3(5, 0, 5),   zjolt.vec3(-5, 0, 5),
    };
    const indices = [_]u32{ 0, 1, 2, 0, 2, 3 };
    const floor = try zjolt.Shape.initMesh(&verts, &indices, .{});
    defer floor.release();

    // A cube, corner down: rotated so its own (1,1,1) corner points along
    // world -Y. That corner sits `half * sqrt(3)` from the box's centre.
    const half: f32 = 0.3;
    const box = try zjolt.Shape.initBox(zjolt.vec3(half, half, half), .{ .convex_radius = 0 });
    defer box.release();
    const corner_down = zjolt.Quat.fromTo(zjolt.vec3(1, 1, 1), zjolt.vec3(0, -1, 0));
    const corner_reach = half * @sqrt(@as(f32, 3.0));

    // Positioned 0.15 off the diagonal seam in X, deep enough (centre 0.1
    // above where the corner would just touch y = 0) that the box's cross
    // section overlaps both triangles substantially rather than grazing
    // one. Empirically (EPA/GJK's own numerical answer, not closed-form)
    // this lands one triangle a clean face contact and the other a
    // shallow, off-axis one -- exactly the seam artifact edge removal drops.
    const d: f32 = 0.1;
    const position: zjolt.RVec3 = zjolt.rvec3(0.15, -corner_reach + d, 0);

    // active_edge_mode/collect_faces_mode set exactly as
    // collideShapeWithInternalEdgeRemoval forces them internally, so the
    // naive call below reaches the identical dispatch path and the only
    // difference is whether the ghost gets removed.
    const settings = query.CollideShapeSettings{
        .active_edge_mode = .collide_with_all,
        .collect_faces_mode = .collect_faces,
    };
    const pair: query.ShapePair = .{
        .first = .{ .shape = box, .position = position, .rotation = corner_down },
        .second = .{ .shape = floor, .position = zjolt.rvec3_zero },
    };

    var naive_buf: [8]query.CollideShapeHit = undefined;
    const naive = try query.collideShapeVsShapeAll(pair, settings, null, &naive_buf);
    try std.testing.expectEqual(@as(usize, 2), naive.len);

    // Sort the two naive hits into the real face contact (the deeper one)
    // and the ghost (the shallow, off-axis one) rather than assuming order,
    // which zjoltCollideShapeVsShapeAll does not promise.
    var face: ?query.CollideShapeHit = null;
    var ghost: ?query.CollideShapeHit = null;
    for (naive) |h| {
        if (h.penetration_depth > 0.05) face = h else ghost = h;
    }
    const face_hit = face orelse return error.TestUnexpectedResult;
    const ghost_hit = ghost orelse return error.TestUnexpectedResult;

    const face_normal = normalize(face_hit.penetration_axis);
    try std.testing.expect(@abs(face_normal.y) > 0.99);
    try std.testing.expect(@abs(face_normal.x) < 0.05);
    try std.testing.expect(@abs(face_normal.z) < 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), face_hit.penetration_depth, 5.0e-3);

    // The ghost is clearly NOT vertical -- its closest feature was the
    // seam's edge, not either triangle's face -- and it is shallower than
    // the real contact.
    const ghost_normal = normalize(ghost_hit.penetration_axis);
    try std.testing.expect(@abs(ghost_normal.x) > 0.3);
    try std.testing.expect(@abs(ghost_normal.z) > 0.3);
    try std.testing.expect(ghost_hit.penetration_depth < face_hit.penetration_depth);

    // The ghost-free call: settings is null here on purpose, proving it
    // forces active_edge_mode/collect_faces_mode itself rather than relying
    // on a caller who remembered to.
    const Recorder = struct {
        count: u32 = 0,
        hit: query.CollideShapeHit = undefined,

        fn onHit(user: ?*anyopaque, hit: *const query.CollideShapeHit) callconv(.c) query.HitAction {
            const self: *@This() = @ptrCast(@alignCast(user.?));
            self.count += 1;
            self.hit = hit.*;
            return .@"continue";
        }
    };
    var rec: Recorder = .{};
    try collision.collideShapeWithInternalEdgeRemoval(pair, null, null, Recorder.onHit, &rec);

    // One contact, not two -- the ghost is gone -- and it is the real face
    // contact, unchanged.
    try std.testing.expectEqual(@as(u32, 1), rec.count);
    try std.testing.expectApproxEqAbs(face_hit.penetration_depth, rec.hit.penetration_depth, 1.0e-4);
    const fixed_normal = normalize(rec.hit.penetration_axis);
    try std.testing.expectApproxEqAbs(face_normal.x, fixed_normal.x, 1.0e-4);
    try std.testing.expectApproxEqAbs(face_normal.y, fixed_normal.y, 1.0e-4);
    try std.testing.expectApproxEqAbs(face_normal.z, fixed_normal.z, 1.0e-4);
}

//=============================================================================
// Boundary validation
//=============================================================================

test "a NULL hit callback is refused at zjoltCollideConvexVsTrianglesCreate" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(1, 1, 1), .{});
    defer box.release();

    // Zig's own type system will not let a caller write `on_hit = null` --
    // CollideShapeHitFn is a bare function pointer, the same way every
    // callback parameter in this ABI is -- so this goes through the raw
    // extern directly and forges the null a C caller could actually pass,
    // the same technique misuse_sweep_test.zig's `hostile` uses.
    const position = zjolt.rvec3(0, 0, 0);
    const rotation = zjolt.quat_identity;

    @setRuntimeSafety(false);
    var zero: usize = 0;
    _ = &zero;
    const null_callback: c_collision.CollideShapeHitFn = @ptrFromInt(zero);

    var out: *c_collision.CollideConvexVsTriangles = undefined;
    const result = c_collision.zjoltCollideConvexVsTrianglesCreate(
        box.handle,
        null,
        null,
        &position,
        &rotation,
        &position,
        &rotation,
        null,
        collision.empty_sub_shape_id,
        null,
        null_callback,
        null,
        &out,
    );
    try std.testing.expectError(error.InvalidArgument, err.check(result));
}
