//! Behavioural tests for shapes, materials and collision groups: three
//! subsystems with a large surface area and, until now, no test that drove
//! them end to end and checked the result of the computation rather than the
//! return code.
//!
//! Reuses `integration_test.zig`'s layer map and floor fixture rather than
//! building its own — see `BINDING.md`.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;
const World = fixture.World;

//=============================================================================
// Shapes
//=============================================================================

test "a box's mass properties match its dimensions and density" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Half extent (0.5, 1, 1.5) -> a 1 x 2 x 3 box. Every axis a different
    // length, so a swapped pair of dimensions in the inertia formula would
    // show up as a wrong axis rather than cancelling out.
    const box = try zjolt.Shape.initBox(zjolt.vec3(0.5, 1.0, 1.5), .{ .density = 500 });
    defer box.release();

    const props = box.massProperties();

    // mass = volume * density = (1 * 2 * 3) * 500.
    try std.testing.expectApproxEqAbs(@as(f32, 3000), props.mass, 1.0);

    // A solid box's inertia about its own centre: I_xx = m/12 * (dy^2 + dz^2),
    // and cyclically for the other two axes, where dx,dy,dz are the FULL
    // extents (2, 4, 6 here) not the half extents passed to initBox — an easy
    // pair to mix up. inertia[9] is a row-major 3x3, so the diagonal is
    // indices 0, 4, 8.
    try std.testing.expectApproxEqAbs(@as(f32, 3250), props.inertia[0], 2.0); // Ixx
    try std.testing.expectApproxEqAbs(@as(f32, 2500), props.inertia[4], 2.0); // Iyy
    try std.testing.expectApproxEqAbs(@as(f32, 1250), props.inertia[8], 2.0); // Izz

    // A box's inertia tensor about its own centre has no off-diagonal terms.
    try std.testing.expectApproxEqAbs(@as(f32, 0), props.inertia[1], 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0), props.inertia[2], 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0), props.inertia[5], 1e-3);
}

test "a compound's centre of mass sits where its children put it, not at its origin" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Two spheres of equal size and density, so their combined centre of
    // mass is the plain average of their positions — deliberately NOT
    // symmetric about the origin, so a compound that quietly reported (0,0,0)
    // regardless of its children could not pass this by accident.
    const left = try zjolt.Shape.initSphere(0.5, .{});
    defer left.release();
    const right = try zjolt.Shape.initSphere(0.5, .{});
    defer right.release();

    const compound = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(left, .{ .position = zjolt.vec3(-1, 0, 0) }),
        zjolt.compoundChild(right, .{ .position = zjolt.vec3(3, 0, 0) }),
    });
    defer compound.release();

    const com = compound.centerOfMass();
    // Average of -1 and 3.
    try std.testing.expectApproxEqAbs(@as(f32, 1), com.x, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, 0), com.y, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, 0), com.z, 0.05);
}

test "Shape.initOffsetCenterOfMass makes a body's world and centre-of-mass transforms genuinely differ" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer box.release();

    // Without an offset, the two transforms agree — the baseline this test
    // needs to show the offset actually changes something.
    const bodies = world.system.bodies();
    const plain = try bodies.createAndAdd(.{
        .shape = box,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    const plain_world = bodies.getWorldTransform(plain);
    const plain_com = bodies.getCenterOfMassTransform(plain);
    try std.testing.expectApproxEqAbs(plain_world.m[13], plain_com.m[13], 1e-4);

    // With a two-metre offset along the shape's own +Y, the shape reports
    // where its centre of mass sits...
    const offset = try zjolt.Shape.initOffsetCenterOfMass(box, zjolt.vec3(0, 2, 0));
    defer offset.release();
    try std.testing.expectApproxEqAbs(@as(f32, 2), offset.centerOfMass().y, 1e-5);

    const shifted = try bodies.createAndAdd(.{
        .shape = offset,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    const world_transform = bodies.getWorldTransform(shifted);
    const com_transform = bodies.getCenterOfMassTransform(shifted);

    // ...and that offset is exactly what separates the two transforms: the
    // world transform still places the body's origin at y = 5, while the
    // centre-of-mass transform sits two metres above it. Getting the two
    // backwards — reporting the shape's placement from the centre of mass, or
    // vice versa — would make this assertion fail with the sign flipped
    // rather than simply pass.
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 5), world_transform.m[13], 1e-4);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 7), com_transform.m[13], 1e-4);
    try std.testing.expectApproxEqAbs(
        @as(zjolt.Real, 2),
        com_transform.m[13] - world_transform.m[13],
        1e-4,
    );
}

test "a mutable compound whose child moves reports a changed bounding box" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const child_shape = try zjolt.Shape.initBox(zjolt.vec3(0.2, 0.2, 0.2), .{});
    defer child_shape.release();

    // Two children, the floor MutableCompound.init enforces.
    var compound = try zjolt.MutableCompound.init(&.{
        zjolt.compoundChild(child_shape, .{ .position = zjolt.vec3(-1, 0, 0) }),
        zjolt.compoundChild(child_shape, .{ .position = zjolt.vec3(1, 0, 0) }),
    });
    defer compound.release();

    const before = compound.asShape().localBounds();
    // Both children currently fit within +/- 1.2.
    try std.testing.expect(before.max.x < 1.5);

    // Move the second child far out along +X.
    try compound.moveChild(1, zjolt.vec3(10, 0, 0), zjolt.quat_identity);

    const after = compound.asShape().localBounds();
    // The bounding box grew to cover the child's new position — a compound
    // whose bounds were cached at construction and never refreshed would
    // leave `after` equal to `before`.
    try std.testing.expect(after.max.x > before.max.x + 5.0);
    try std.testing.expectApproxEqAbs(@as(f32, 10.2), after.max.x, 0.05);
}

//=============================================================================
// Materials
//=============================================================================

test "a material attached to a mesh triangle comes back through a ray-cast hit as the same identity" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const mat_a = try zjolt.PhysicsMaterial.init(.{ .debug_name = "gravel" });
    defer mat_a.release();
    const mat_b = try zjolt.PhysicsMaterial.init(.{ .debug_name = "metal" });
    defer mat_b.release();

    // A flat two-triangle square in the XZ plane, wound so both triangles
    // face +Y. Triangle A is (v0, v2, v1); triangle B is (v1, v2, v3).
    const vertices = [_]zjolt.Vec3{
        zjolt.vec3(-1, 0, -1), // v0
        zjolt.vec3(1, 0, -1), // v1
        zjolt.vec3(-1, 0, 1), // v2
        zjolt.vec3(1, 0, 1), // v3
    };
    const indices = [_]u32{ 0, 2, 1, 1, 2, 3 };

    const mesh = try zjolt.Shape.initMesh(&vertices, &indices, .{
        .materials = &.{ mat_a, mat_b },
        .triangle_materials = &.{ 0, 1 },
    });
    defer mesh.release();

    // Placed well above the fixture's own floor, so the two flat surfaces
    // are never coincident and a ray only ever has one thing to hit.
    const bodies = world.system.bodies();
    const mesh_body = try bodies.createAndAdd(.{
        .shape = mesh,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();

    // Centroid of triangle A (v0, v2, v1) is (-1/3, 0, -1/3); of triangle B
    // (v1, v2, v3) is (1/3, 0, 1/3). Both well clear of the shared diagonal.
    const hit_a = (try queries.castRayClosest(
        zjolt.rvec3(-1.0 / 3.0, 15, -1.0 / 3.0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    const hit_b = (try queries.castRayClosest(
        zjolt.rvec3(1.0 / 3.0, 15, 1.0 / 3.0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;

    try std.testing.expectEqual(mesh_body, hit_a.body);
    try std.testing.expectEqual(mesh_body, hit_b.body);

    const material_a = mesh.material(hit_a.sub_shape_id) orelse return error.TestUnexpectedResult;
    const material_b = mesh.material(hit_b.sub_shape_id) orelse return error.TestUnexpectedResult;

    // The same identity as what the triangle was built with...
    try std.testing.expect(material_a.eql(mat_a));
    try std.testing.expect(material_b.eql(mat_b));
    // ...and the two triangles really do report different materials, not the
    // same one twice.
    try std.testing.expect(!material_a.eql(material_b));
}

test "a height field reports the material of the quad that was hit" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const special = try zjolt.PhysicsMaterial.init(.{ .debug_name = "special" });
    defer special.release();
    const default_mat = try zjolt.PhysicsMaterial.init(.{ .debug_name = "default" });
    defer default_mat.release();

    // A flat 4x4-sample field: the smallest useful size at the default block
    // size of 2. Quad index 0 is the (0,0) corner quad and the last index is
    // the far (2,2) corner quad in EITHER a row-major or column-major quad
    // encoding, since a square grid's first and last indices land on the same
    // corners either way — so this test does not depend on which one Jolt
    // uses.
    const sample_count = 4;
    const samples = [_]f32{0} ** (sample_count * sample_count);
    var quad_materials = [_]u8{1} ** ((sample_count - 1) * (sample_count - 1));
    quad_materials[0] = 0; // the (0,0) corner quad gets `special`.

    const field = try zjolt.Shape.initHeightField(&samples, sample_count, .{
        .materials = &.{ special, default_mat },
        .quad_materials = &quad_materials,
    });
    defer field.release();

    const bodies = world.system.bodies();
    const field_body = try bodies.createAndAdd(.{
        .shape = field,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();

    // Centre of the (0,0) quad, and centre of the (2,2) quad.
    const hit_special = (try queries.castRayClosest(
        zjolt.rvec3(0.5, 15, 0.5),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    const hit_default = (try queries.castRayClosest(
        zjolt.rvec3(2.5, 15, 2.5),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;

    try std.testing.expectEqual(field_body, hit_special.body);
    try std.testing.expectEqual(field_body, hit_default.body);

    const material_special = field.material(hit_special.sub_shape_id) orelse
        return error.TestUnexpectedResult;
    const material_default = field.material(hit_default.sub_shape_id) orelse
        return error.TestUnexpectedResult;

    try std.testing.expect(material_special.eql(special));
    try std.testing.expect(material_default.eql(default_mat));
}

test "a shape built without a material reports the shared default, not null" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const plain = try zjolt.Shape.initSphere(0.5, .{});
    defer plain.release();

    const default_material = zjolt.PhysicsMaterial.default() orelse
        return error.TestUnexpectedResult;
    // A sphere has no leaves, so its own material is addressed with the
    // empty sub-shape id — Jolt asserts on any other value here.
    const reported = plain.material(zjolt.sub_shape_id_empty) orelse
        return error.TestUnexpectedResult;
    try std.testing.expect(reported.eql(default_material));

    const custom = try zjolt.PhysicsMaterial.init(.{
        .debug_name = "rubber",
        .debug_color = zjolt.color(200, 50, 25),
    });
    defer custom.release();

    const branded = try zjolt.Shape.initSphere(0.5, .{ .material = custom });
    defer branded.release();

    const branded_material = branded.material(zjolt.sub_shape_id_empty) orelse
        return error.TestUnexpectedResult;
    try std.testing.expect(branded_material.eql(custom));
    try std.testing.expect(!branded_material.eql(default_material));

    try std.testing.expectEqualStrings("rubber", custom.debugName());
    const c = custom.debugColor();
    try std.testing.expectEqual(@as(u8, 200), c.r);
    try std.testing.expectEqual(@as(u8, 50), c.g);
    try std.testing.expectEqual(@as(u8, 25), c.b);
}

//=============================================================================
// Collision groups
//=============================================================================

/// Fires two equal spheres at each other from opposite sides with no gravity,
/// and returns the FINAL x of the left one minus the x of the right one.
///
/// If they collide, restitution 0 and equal-and-opposite closing velocities
/// bring them to rest at contact — separated by about the sum of their radii
/// (0.8), still in their original left/right order, so the result stays
/// negative and close to -0.8.
///
/// If they do not collide, nothing stops them: each carries on past the
/// other's starting position, so by the time this returns they have swapped
/// places and the result is strongly positive. Checked on position after
/// stepping, not on whether a contact was ever reported.
fn finalXSeparation(group_a: zjolt.CollisionGroup, group_b: zjolt.CollisionGroup) !f64 {
    var world = try World.init();
    defer world.deinit();
    world.system.setGravity(zjolt.vec3_zero);

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const left = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(-2, 5, 0),
        .linear_velocity = zjolt.vec3(2, 0, 0),
        .restitution = 0,
        .collision_group = group_a,
    }, .activate);
    const right = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(2, 5, 0),
        .linear_velocity = zjolt.vec3(-2, 0, 0),
        .restitution = 0,
        .collision_group = group_b,
    }, .activate);

    try world.stepFor(2.0);

    const left_x: f64 = @floatCast(bodies.getPosition(left).x);
    const right_x: f64 = @floatCast(bodies.getPosition(right).x);
    return left_x - right_x;
}

test "a group filter's disabled sub-group pair blocks collision, but a different group id overrides it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Same group, and the filter disables exactly the (0, 1) pair the two
    // bodies below sit in -> no collision, so they pass straight through.
    {
        const filter = try zjolt.GroupFilter.initTable(2);
        defer filter.release();
        try filter.disableCollision(0, 1);

        const separation = try finalXSeparation(
            .{ .filter = filter, .group_id = 1, .sub_group_id = 0 },
            .{ .filter = filter, .group_id = 1, .sub_group_id = 1 },
        );
        try std.testing.expect(separation > 1.0);
    }

    // The SAME filter and sub-groups, but different group ids. "Bodies with
    // different group ids always collide" overrides the filter table
    // entirely -> they collide and stop at contact.
    {
        const filter = try zjolt.GroupFilter.initTable(2);
        defer filter.release();
        try filter.disableCollision(0, 1);

        const separation = try finalXSeparation(
            .{ .filter = filter, .group_id = 1, .sub_group_id = 0 },
            .{ .filter = filter, .group_id = 2, .sub_group_id = 1 },
        );
        try std.testing.expectApproxEqAbs(@as(f64, -0.8), separation, 0.2);
    }
}

test "bodies in the same sub-group never collide, even though the filter table allows every pair by default" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A fresh table disables nothing - every DIFFERENT pair is enabled. But
    // these two bodies share one sub-group, which the table is never even
    // consulted for: a sub-group never collides with itself.
    const filter = try zjolt.GroupFilter.initTable(2);
    defer filter.release();

    const separation = try finalXSeparation(
        .{ .filter = filter, .group_id = 1, .sub_group_id = 0 },
        .{ .filter = filter, .group_id = 1, .sub_group_id = 0 },
    );
    try std.testing.expect(separation > 1.0);
}

test "bodies in the same group with different filter objects never collide" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // The documented trap: same group id, but each body carries its OWN
    // filter object rather than sharing one. That is enough to suppress
    // collision even though nothing in either table forbids the pair.
    const filter_a = try zjolt.GroupFilter.initTable(2);
    defer filter_a.release();
    const filter_b = try zjolt.GroupFilter.initTable(2);
    defer filter_b.release();

    const separation = try finalXSeparation(
        .{ .filter = filter_a, .group_id = 1, .sub_group_id = 0 },
        .{ .filter = filter_b, .group_id = 1, .sub_group_id = 1 },
    );
    try std.testing.expect(separation > 1.0);
}

//=============================================================================
// Shape introspection
//
// A caller holding an opaque `Shape` could not previously ask it what it was
// built from. `subType` says which kind it is; these say with what.
//=============================================================================

test "every convex shape reports back the dimensions it was built with" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(0.75, .{});
    defer sphere.release();
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), try sphere.sphereRadius(), 1e-5);

    // Deliberately three different half extents, so a getter that returned the
    // wrong axis — or the full extent instead of the half — shows up as a
    // wrong number rather than coincidentally matching.
    const box = try zjolt.Shape.initBox(zjolt.vec3(0.5, 1.0, 1.5), .{});
    defer box.release();
    const half = try box.halfExtent();
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), half.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), half.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), half.z, 1e-5);

    // A capsule's two numbers are the half height of its CYLINDER and its
    // radius, not the half height of the whole shape — the pair most easily
    // conflated, which is why both are asserted.
    const capsule = try zjolt.Shape.initCapsule(0.4, 0.3, .{});
    defer capsule.release();
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), try capsule.halfHeightOfCylinder(), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.3), try capsule.sphereRadius(), 1e-5);

    const cylinder = try zjolt.Shape.initCylinder(0.6, 0.2, .{});
    defer cylinder.release();
    try std.testing.expectApproxEqAbs(@as(f32, 0.6), try cylinder.halfHeight(), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), try cylinder.sphereRadius(), 1e-5);

    // Tapered ends differ from each other, so a getter wired to the wrong end
    // cannot pass.
    const tapered = try zjolt.Shape.initTaperedCylinder(0.5, 0.9, 0.3, .{});
    defer tapered.release();
    try std.testing.expectApproxEqAbs(@as(f32, 0.9), try tapered.topRadius(), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.3), try tapered.bottomRadius(), 1e-5);
}

test "an introspection getter refuses a shape of the wrong kind" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(1, 1, 1), .{});
    defer box.release();
    const sphere = try zjolt.Shape.initSphere(1.0, .{});
    defer sphere.release();

    // The whole point of narrowing rather than casting: asking a box for a
    // radius reads whatever sits at a SphereShape's radius offset if the cast
    // is blind, which is a plausible float and therefore a silent wrong
    // answer rather than a crash.
    try std.testing.expectError(error.InvalidArgument, box.sphereRadius());
    try std.testing.expectError(error.InvalidArgument, sphere.halfExtent());
    try std.testing.expectError(error.InvalidArgument, sphere.topRadius());
    try std.testing.expectError(error.InvalidArgument, box.halfHeightOfCylinder());
}

test "a convex hull reports its points back" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const corners = [_]zjolt.Vec3{
        zjolt.vec3(0, 0, 0),
        zjolt.vec3(1, 0, 0),
        zjolt.vec3(0, 1, 0),
        zjolt.vec3(0, 0, 1),
    };
    const hull = try zjolt.Shape.initConvexHull(&corners, .{});
    defer hull.release();

    // A tetrahedron: every input point is a vertex of the hull, so the count
    // round-trips exactly. A hull that dropped or duplicated one would differ.
    const count = try hull.numPoints();
    try std.testing.expectEqual(@as(u32, 4), count);

    var buffer: [8]zjolt.Vec3 = undefined;
    const got = try hull.hullPoints(&buffer);
    try std.testing.expectEqual(@as(usize, 4), got.len);

    // The points come back RELATIVE TO THE CENTRE OF MASS, not in the space
    // they were handed in — for this tetrahedron that is a shift of exactly
    // its centroid, (0.25, 0.25, 0.25). A caller who plots them raw draws the
    // hull in the wrong place. Adding the centre of mass back is what makes
    // this an assertion about the documented behaviour rather than a
    // tolerance loosened until it passed. Order is Jolt's business.
    const com = hull.centerOfMass();
    for (got) |p| {
        var matched = false;
        for (corners) |q| {
            if (@abs(p.x + com.x - q.x) < 1e-3 and @abs(p.y + com.y - q.y) < 1e-3 and
                @abs(p.z + com.z - q.z) < 1e-3) matched = true;
        }
        try std.testing.expect(matched);
    }

    try std.testing.expect(try hull.numFaces() >= 4);
}
