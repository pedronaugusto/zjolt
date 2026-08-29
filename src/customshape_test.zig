//! Tests for host-defined shapes (`ffi/zjolt_customshape.h`).
//!
//! Layer A (a custom convex shape) is the priority: a host describing a unit
//! sphere through nothing but a support function has to match Jolt's own
//! `SphereShape` in every geometric respect a real host would rely on —
//! mass, bounds, inner radius, a ray cast, a drop-to-rest simulation, and
//! life inside a compound. Layer B (a general custom shape) gets one test
//! proving its own batched, non-derived path: `GetTrianglesStart`/`Next` and
//! a ray through the geometry they describe.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const customshape = @import("customshape.zig");
const softbody = @import("softbody.zig");
const integration = @import("integration_test.zig");

const World = integration.World;
const Layers = integration.Layers;

fn expectVec3Near(expected: zjolt.Vec3, actual: zjolt.Vec3, tolerance: f32) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, tolerance);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, tolerance);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, tolerance);
}

//=============================================================================
// Layer A fixture — a custom convex shape whose support function describes a
// sphere, the plainest possible convex shape and the one every geometric
// comparison below has a ready-made answer for.
//=============================================================================

const Sphere = struct {
    radius: f32,
    destroy_count: u32 = 0,

    fn support(user: ?*anyopaque, direction: zjolt.Vec3) callconv(.c) zjolt.Vec3 {
        const self: *const Sphere = @ptrCast(@alignCast(user.?));
        const len_sq = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (len_sq == 0) return zjolt.vec3_zero;
        const scale = self.radius / @sqrt(len_sq);
        return .{ .x = direction.x * scale, .y = direction.y * scale, .z = direction.z * scale };
    }

    fn innerRadius(user: ?*const anyopaque) callconv(.c) f32 {
        const self: *const Sphere = @ptrCast(@alignCast(user.?));
        return self.radius;
    }

    fn localBounds(user: ?*const anyopaque, out: *zjolt.AABox) callconv(.c) void {
        const self: *const Sphere = @ptrCast(@alignCast(user.?));
        out.* = .{
            .min = .{ .x = -self.radius, .y = -self.radius, .z = -self.radius },
            .max = .{ .x = self.radius, .y = self.radius, .z = self.radius },
        };
    }

    fn massProperties(user: ?*const anyopaque, out: *zjolt.MassProperties) callconv(.c) void {
        const self: *const Sphere = @ptrCast(@alignCast(user.?));
        // Solid sphere, density 1000 kg/m^3 -- Shape.initSphere's own
        // default with no `density` given -- so the two are directly
        // comparable.
        const density: f32 = 1000.0;
        const r = self.radius;
        const mass = density * (4.0 / 3.0) * std.math.pi * r * r * r;
        const i = 0.4 * mass * r * r;
        out.* = .{ .mass = mass, .inertia = .{ i, 0, 0, 0, i, 0, 0, 0, i } };
    }

    fn volume(user: ?*const anyopaque) callconv(.c) f32 {
        const self: *const Sphere = @ptrCast(@alignCast(user.?));
        return (4.0 / 3.0) * std.math.pi * self.radius * self.radius * self.radius;
    }

    fn destroy(user: ?*anyopaque) callconv(.c) void {
        const self: *Sphere = @ptrCast(@alignCast(user.?));
        self.destroy_count += 1;
    }

    fn callbacks() customshape.ConvexShapeCallbacks {
        return .{
            .support = support,
            .inner_radius = innerRadius,
            .local_bounds = localBounds,
            .mass_properties = massProperties,
            .volume = volume,
            .destroy = destroy,
        };
    }
};

//=============================================================================
// Layer A — geometry matches Shape.initSphere exactly
//=============================================================================

test "a custom convex shape describing a unit sphere matches Shape.initSphere in mass, bounds, inner radius and a ray cast" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var host = Sphere{ .radius = 1.0 };
    const custom = try customshape.initCustomConvex(Sphere.callbacks(), &host, null);
    defer custom.release();

    const reference = try zjolt.Shape.initSphere(1.0, .{});
    defer reference.release();

    const custom_mp = custom.massProperties();
    const reference_mp = reference.massProperties();
    try std.testing.expectApproxEqAbs(reference_mp.mass, custom_mp.mass, 1e-3);
    for (0..9) |i| {
        try std.testing.expectApproxEqAbs(reference_mp.inertia[i], custom_mp.inertia[i], 1e-3);
    }

    const custom_bounds = custom.localBounds();
    const reference_bounds = reference.localBounds();
    try expectVec3Near(reference_bounds.min, custom_bounds.min, 1e-5);
    try expectVec3Near(reference_bounds.max, custom_bounds.max, 1e-5);

    try std.testing.expectApproxEqAbs(reference.innerRadius(), custom.innerRadius(), 1e-5);

    // A ray straight down from (0, 5, 0), length 5: it meets the sphere's
    // top surface at y = radius = 1, i.e. at fraction (5 - 1) / 5 = 0.8.
    const placed = try zjolt.TransformedShape.init(custom, zjolt.rvec3(0, 0, 0), .{});
    defer placed.deinit();
    const hit = try placed.castRayClosest(zjolt.rvec3(0, 5, 0), zjolt.vec3(0, -5, 0), null, null);
    try std.testing.expect(hit != null);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), hit.?.fraction, 1e-4);
}

//=============================================================================
// Layer A — drops onto a static floor and settles at the analytic height
//=============================================================================

test "a custom convex sphere dropped onto a static floor comes to rest at the analytically correct height" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // World's floor is a box of half-extent (200, 0.5, 200) centred at
    // y = -0.5, so its top surface is at y = 0. A sphere of radius 0.5
    // settles with its centre at y = 0.5.
    var host = Sphere{ .radius = 0.5 };
    const shape = try customshape.initCustomConvex(Sphere.callbacks(), &host, null);
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 3, 0),
    }, .activate);

    try world.stepFor(3.0);

    // Jolt's default penetration slop (cDefaultPenetrationSlop, 0.02 m) is
    // exactly that: a real SphereShape settles with the same small gap, so
    // the tolerance below accounts for it rather than for anything specific
    // to a custom shape.
    const position = bodies.getPosition(ball);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0.0), position.x, 1e-2);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0.5), position.y, 0.03);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0.0), position.z, 1e-2);

    const velocity = bodies.getLinearVelocity(ball);
    try std.testing.expect(@abs(velocity.y) < 0.05);
}

//=============================================================================
// Layer A — inside a StaticCompoundShape, cast against
//=============================================================================

test "a custom convex shape inside a StaticCompoundShape can be cast against" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var host = Sphere{ .radius = 1.0 };
    const custom = try customshape.initCustomConvex(Sphere.callbacks(), &host, null);
    defer custom.release();

    const compound = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(custom, .{ .position = zjolt.vec3(2, 0, 0) }),
    });
    defer compound.release();

    const placed = try zjolt.TransformedShape.init(compound, zjolt.rvec3(0, 0, 0), .{});
    defer placed.deinit();

    // The child sits at local (2, 0, 0), the compound is unrotated at the
    // world origin, so a ray straight down through x = 2 meets the sphere's
    // top at y = 1: the same fraction 0.8 as the standalone case above.
    const hit = try placed.castRayClosest(zjolt.rvec3(2, 5, 0), zjolt.vec3(0, -5, 0), null, null);
    try std.testing.expect(hit != null);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), hit.?.fraction, 1e-4);

    // A ray that misses the child entirely finds nothing.
    const miss = try placed.castRayClosest(zjolt.rvec3(10, 5, 0), zjolt.vec3(0, -5, 0), null, null);
    try std.testing.expect(miss == null);
}

//=============================================================================
// Layer A — a create call missing a required callback
//=============================================================================

test "a custom convex shape create call with a required callback left null is refused" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var callbacks = Sphere.callbacks();
    callbacks.support = null;
    try std.testing.expectError(zjolt.Error.InvalidArgument, customshape.initCustomConvex(callbacks, null, null));
    try std.testing.expect(zjolt.lastError().len > 0);
}

//=============================================================================
// Layer A — destroy runs exactly once, when the last reference goes
//=============================================================================

test "destroy runs exactly once when the last reference to a custom convex shape goes" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var host = Sphere{ .radius = 1.0 };
    const shape = try customshape.initCustomConvex(Sphere.callbacks(), &host, null);
    shape.addRef();
    try std.testing.expectEqual(@as(u32, 2), shape.refCount());

    shape.release();
    try std.testing.expectEqual(@as(u32, 0), host.destroy_count);

    shape.release();
    try std.testing.expectEqual(@as(u32, 1), host.destroy_count);
}

//=============================================================================
// Layer B fixture — a general custom shape describing two triangles forming
// a unit square in the XZ plane, y = 0.
//=============================================================================

const TwoTriangles = struct {
    const v0 = zjolt.vec3(-1, 0, -1);
    const v1 = zjolt.vec3(1, 0, -1);
    const v2 = zjolt.vec3(1, 0, 1);
    const v3 = zjolt.vec3(-1, 0, 1);

    const WalkState = extern struct {
        done: bool = false,
    };

    destroy_count: u32 = 0,
    soft_count: u32 = 0,
    soft_inv_mass: [4]f32 = @splat(-1),

    fn localBounds(user: ?*const anyopaque, out: *zjolt.AABox) callconv(.c) void {
        _ = user;
        out.* = .{ .min = .{ .x = -1, .y = -0.01, .z = -1 }, .max = .{ .x = 1, .y = 0.01, .z = 1 } };
    }

    fn bits(user: ?*const anyopaque) callconv(.c) u32 {
        _ = user;
        return 1;
    }

    fn innerRadius(user: ?*const anyopaque) callconv(.c) f32 {
        _ = user;
        return 0.0;
    }

    fn massProperties(user: ?*const anyopaque, out: *zjolt.MassProperties) callconv(.c) void {
        _ = user;
        out.* = .{ .mass = 1.0, .inertia = .{ 1, 0, 0, 0, 1, 0, 0, 0, 1 } };
    }

    fn getMaterial(user: ?*const anyopaque, sub_shape_id: zjolt.SubShapeId) callconv(.c) ?*const customshape.c_decls.PhysicsMaterial {
        _ = user;
        _ = sub_shape_id;
        return null;
    }

    fn surfaceNormal(user: ?*const anyopaque, sub_shape_id: zjolt.SubShapeId, position: zjolt.Vec3, out_normal: *zjolt.Vec3) callconv(.c) void {
        _ = user;
        _ = sub_shape_id;
        _ = position;
        out_normal.* = zjolt.vec3(0, 1, 0);
    }

    fn submergedVolume(
        user: ?*const anyopaque,
        transform: *const zjolt.Mat44,
        scale: zjolt.Vec3,
        surface: *const zjolt.Plane,
        out_total: *f32,
        out_submerged: *f32,
        out_center: *zjolt.Vec3,
    ) callconv(.c) void {
        _ = user;
        _ = transform;
        _ = scale;
        _ = surface;
        out_total.* = 0;
        out_submerged.* = 0;
        out_center.* = zjolt.vec3_zero;
    }

    fn castRayClosest(
        user: ?*const anyopaque,
        origin: zjolt.Vec3,
        direction: zjolt.Vec3,
        max_fraction: f32,
        out_fraction: *f32,
        out_sub_shape_id: *zjolt.SubShapeId,
    ) callconv(.c) bool {
        _ = user;
        const f0 = zjolt.ray.triangle(origin, direction, v0, v1, v2);
        const f1 = zjolt.ray.triangle(origin, direction, v0, v2, v3);
        var best = max_fraction;
        var found = false;
        var which: zjolt.SubShapeId = 0;
        if (f0 <= 1.0 and f0 < best) {
            best = f0;
            found = true;
            which = 0;
        }
        if (f1 <= 1.0 and f1 < best) {
            best = f1;
            found = true;
            which = 1;
        }
        if (!found) return false;
        out_fraction.* = best;
        out_sub_shape_id.* = which;
        return true;
    }

    fn castRayAll(
        user: ?*const anyopaque,
        origin: zjolt.Vec3,
        direction: zjolt.Vec3,
        settings: *const customshape.RayCastSettings,
        out_hits: [*]customshape.CustomShapeRayHit,
        max_hits: u32,
    ) callconv(.c) u32 {
        _ = user;
        _ = settings;
        var count: u32 = 0;
        const f0 = zjolt.ray.triangle(origin, direction, v0, v1, v2);
        if (f0 <= 1.0 and count < max_hits) {
            out_hits[count] = .{ .fraction = f0, .sub_shape_id = 0 };
            count += 1;
        }
        const f1 = zjolt.ray.triangle(origin, direction, v0, v2, v3);
        if (f1 <= 1.0 and count < max_hits) {
            out_hits[count] = .{ .fraction = f1, .sub_shape_id = 1 };
            count += 1;
        }
        return count;
    }

    fn collidePoint(user: ?*const anyopaque, point: zjolt.Vec3, out_sub_shape_ids: [*]zjolt.SubShapeId, max_hits: u32) callconv(.c) u32 {
        _ = user;
        _ = point;
        _ = out_sub_shape_ids;
        _ = max_hits;
        return 0;
    }

    fn collideSoftBody(
        user: ?*const anyopaque,
        scale: zjolt.Vec3,
        positions: [*]const zjolt.Vec3,
        inv_mass: [*]const f32,
        count: u32,
        out_penetration: [*]f32,
        out_normal: [*]zjolt.Vec3,
    ) callconv(.c) void {
        _ = scale;
        const self: *TwoTriangles = @ptrCast(@alignCast(@constCast(user.?)));
        self.soft_count = count;
        var i: u32 = 0;
        while (i < count) : (i += 1) {
            if (i < self.soft_inv_mass.len) self.soft_inv_mass[i] = inv_mass[i];
            // A pinned vertex cannot move, so there is nothing to report for it.
            if (inv_mass[i] <= 0) {
                out_penetration[i] = -std.math.floatMax(f32);
                out_normal[i] = zjolt.vec3(0, 1, 0);
                continue;
            }
            out_penetration[i] = -positions[i].y;
            out_normal[i] = zjolt.vec3(0, 1, 0);
        }
    }

    fn trianglesStart(
        user: ?*const anyopaque,
        context: *customshape.ShapeTrianglesContext,
        box: *const zjolt.AABox,
        position: ?*const zjolt.Vec3,
        rotation: ?*const zjolt.Quat,
        scale: ?*const zjolt.Vec3,
    ) callconv(.c) customshape.Result {
        _ = user;
        _ = box;
        _ = position;
        _ = rotation;
        _ = scale;
        const state: *WalkState = @ptrCast(@alignCast(context));
        state.* = .{ .done = false };
        return .ok;
    }

    fn trianglesNext(
        user: ?*const anyopaque,
        context: *customshape.ShapeTrianglesContext,
        max_triangles: u32,
        out_vertices: [*]zjolt.Vec3,
        out_materials: ?[*]?*const customshape.c_decls.PhysicsMaterial,
        out_count: *u32,
    ) callconv(.c) customshape.Result {
        _ = user;
        _ = max_triangles;
        const state: *WalkState = @ptrCast(@alignCast(context));
        if (state.done) {
            out_count.* = 0;
            return .ok;
        }
        out_vertices[0] = v0;
        out_vertices[1] = v1;
        out_vertices[2] = v2;
        out_vertices[3] = v0;
        out_vertices[4] = v2;
        out_vertices[5] = v3;
        if (out_materials) |m| {
            m[0] = null;
            m[1] = null;
        }
        out_count.* = 2;
        state.done = true;
        return .ok;
    }

    fn getStats(user: ?*const anyopaque, out: *zjolt.ShapeStats) callconv(.c) void {
        _ = user;
        out.* = .{ .size_bytes = 0, .num_triangles = 2 };
    }

    fn volume(user: ?*const anyopaque) callconv(.c) f32 {
        _ = user;
        return 0.0;
    }

    fn destroy(user: ?*anyopaque) callconv(.c) void {
        const self: *TwoTriangles = @ptrCast(@alignCast(user.?));
        self.destroy_count += 1;
    }

    fn callbacks() customshape.ShapeCallbacks {
        return .{
            .local_bounds = localBounds,
            .sub_shape_id_bits_recursive = bits,
            .inner_radius = innerRadius,
            .mass_properties = massProperties,
            .get_material = getMaterial,
            .surface_normal = surfaceNormal,
            .submerged_volume = submergedVolume,
            .cast_ray_closest = castRayClosest,
            .cast_ray_all = castRayAll,
            .collide_point = collidePoint,
            .collide_soft_body_vertices = collideSoftBody,
            .get_triangles_start = trianglesStart,
            .get_triangles_next = trianglesNext,
            .get_stats = getStats,
            .volume = volume,
            .destroy = destroy,
        };
    }
};

test "a custom shape's soft-body callback sees each vertex's inverse mass" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var host = TwoTriangles{};
    const shape = try customshape.initCustom(TwoTriangles.callbacks(), &host);
    defer shape.release();

    // Vertex 0 is pinned, vertex 1 is free; both sit above the shape's plane.
    var positions = [_]zjolt.Vec3{ zjolt.vec3(0, 1, 0), zjolt.vec3(0.5, 2, 0.5) };
    var inv_mass = [_]f32{ 0, 1 };
    var planes = [_]zjolt.Plane{ .{ .normal = zjolt.vec3(0, 1, 0), .constant = 0 }, .{ .normal = zjolt.vec3(0, 1, 0), .constant = 0 } };
    var penetration = [_]f32{ -std.math.floatMax(f32), -std.math.floatMax(f32) };
    var shape_index = [_]i32{ -1, -1 };

    const it: softbody.CollideSoftBodyVertexIterator = .{
        .position = @ptrCast(&positions),
        .position_stride = @sizeOf(zjolt.Vec3),
        .inv_mass = @ptrCast(&inv_mass),
        .inv_mass_stride = @sizeOf(f32),
        .collision_plane = @ptrCast(&planes),
        .collision_plane_stride = @sizeOf(zjolt.Plane),
        .largest_penetration = @ptrCast(&penetration),
        .largest_penetration_stride = @sizeOf(f32),
        .colliding_shape_index = @ptrCast(&shape_index),
        .colliding_shape_index_stride = @sizeOf(i32),
    };

    try softbody.collideSoftBodyVertices(shape, zjolt.mat44_identity, zjolt.vec3(1, 1, 1), &it, 2, 7);

    try std.testing.expectEqual(@as(u32, 2), host.soft_count);
    try std.testing.expectEqual(@as(f32, 0), host.soft_inv_mass[0]);
    try std.testing.expectEqual(@as(f32, 1), host.soft_inv_mass[1]);

    // The pinned vertex is left alone; the free one is claimed by this shape.
    try std.testing.expectEqual(@as(i32, -1), shape_index[0]);
    try std.testing.expectEqual(@as(i32, 7), shape_index[1]);
}

test "a general custom shape returning two triangles reports both through the triangle walk, and a ray through one hits" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var host = TwoTriangles{};
    const shape = try customshape.initCustom(TwoTriangles.callbacks(), &host);
    defer shape.release();

    var walk = try shape.triangleWalk(
        .{ .min = zjolt.vec3(-10, -10, -10), .max = zjolt.vec3(10, 10, 10) },
        .{},
    );
    var vertex_buffer: [96]zjolt.Vec3 = undefined;
    const first = try walk.next(&vertex_buffer, null);
    try std.testing.expectEqual(@as(usize, 6), first.len);
    const second = try walk.next(&vertex_buffer, null);
    try std.testing.expectEqual(@as(usize, 0), second.len);

    // A ray straight down through the middle of the second triangle
    // (x = 0.5, z = 0.5, inside v0-v2-v3) meets the plane y = 0 at fraction
    // (3 - 0) / 3 = 1, the far end of a ray of length 3 from y = 3.
    const placed = try zjolt.TransformedShape.init(shape, zjolt.rvec3(0, 0, 0), .{});
    defer placed.deinit();
    const hit = try placed.castRayClosest(zjolt.rvec3(0.5, 3, 0.5), zjolt.vec3(0, -3, 0), null, null);
    try std.testing.expect(hit != null);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), hit.?.fraction, 1e-4);
    // Not asserting the exact sub_shape_id: SubShapeID starts all-ones and
    // composes by clearing and setting bits within it, so with a 1-bit
    // budget the local value 1 (binary 1) composed at the root reproduces
    // the all-ones (empty) pattern exactly -- correct behaviour of Jolt's
    // own id scheme, not something a host decodes by comparing raw integers.

    try std.testing.expectEqual(@as(u32, 0), host.destroy_count);
}

test "a general custom shape create call with a required callback left null is refused" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var callbacks = TwoTriangles.callbacks();
    callbacks.get_material = null;
    try std.testing.expectError(zjolt.Error.InvalidArgument, customshape.initCustom(callbacks, null));
    try std.testing.expect(zjolt.lastError().len > 0);
}
