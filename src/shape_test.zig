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
const shape_mod = @import("shape.zig");

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

test "innerShape unwraps exactly one level of decoration, not all the way to a leaf" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer box.release();

    // Two layers of decoration: scaled wraps rotated-translated, which
    // wraps the box.
    const rt = try zjolt.Shape.initRotatedTranslated(box, zjolt.vec3(1, 0, 0), zjolt.quat_identity);
    defer rt.release();
    const scaled = try zjolt.Shape.initScaled(rt, zjolt.vec3(2, 2, 2));
    defer scaled.release();

    // One level from `scaled` is `rt`, NOT `box` — drilling past the first
    // wrapper is exactly what `leafShape` is for, and this is not that.
    const one_level = try scaled.innerShape();
    try std.testing.expectEqual(zjolt.ShapeSubType.rotated_translated, one_level.subType());

    const two_levels = try one_level.innerShape();
    try std.testing.expectEqual(zjolt.ShapeSubType.box, two_levels.subType());

    // A shape that is not decorated has no inner shape to unwrap.
    try std.testing.expectError(error.InvalidArgument, box.innerShape());
}

test "a compound child's user data can be changed after the compound is built, even a static one" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(0.5, .{});
    defer sphere.release();
    const box = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer box.release();

    // Two children — a single one at the origin with no rotation is a case
    // Jolt SIMPLIFIES away into the child shape itself, which has no
    // compound child user data at all.
    const compound = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(sphere, .{ .position = zjolt.vec3(-1, 0, 0), .user_data = 1 }),
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(1, 0, 0) }),
    });
    defer compound.release();
    try std.testing.expectEqual(@as(u32, 1), compound.compoundChildUserData(0));

    try compound.setCompoundChildUserData(0, 99);
    try std.testing.expectEqual(@as(u32, 99), compound.compoundChildUserData(0));

    try std.testing.expectError(
        error.InvalidArgument,
        compound.setCompoundChildUserData(5, 1),
    );
}

test "a convex hull's max_error_convex_radius bounds how far shrinking the hull for its convex radius may go" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A thin, sharply pointed tetrahedron: the sharper a vertex, the more
    // Jolt has to shrink the hull to keep the convex-radius error within
    // budget, so this geometry is what makes the two settings visibly
    // disagree rather than both silently keeping the full 0.3 requested.
    const corners = [_]zjolt.Vec3{
        zjolt.vec3(0, 0, 0),
        zjolt.vec3(1, 0, 0),
        zjolt.vec3(0, 1, 0),
        zjolt.vec3(0, 0, 3),
    };

    // Jolt's own default (0.05) leaves little error budget, so the radius
    // this sharp a hull can support is shrunk far below the 0.3 requested.
    const tight = try zjolt.Shape.initConvexHull(&corners, .{
        .max_convex_radius = 0.3,
        .max_error_convex_radius = 0,
    });
    defer tight.release();
    try std.testing.expect(try tight.convexRadius() < 0.05);

    // A generous error budget removes that constraint, so the full
    // requested radius survives. Same points, same max_convex_radius —
    // max_error_convex_radius is the only thing that differs, and it is
    // what accounts for the radius coming back an order of magnitude larger.
    const loose = try zjolt.Shape.initConvexHull(&corners, .{
        .max_convex_radius = 0.3,
        .max_error_convex_radius = 10.0,
    });
    defer loose.release();
    try std.testing.expectApproxEqAbs(@as(f32, 0.3), try loose.convexRadius(), 1e-4);
}

test "a shape's user data defaults to zero and is reachable after creation, for every shape family" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // One representative from each family the reviewer's gap list spanned:
    // a convex primitive, a compound, a decorated shape, and a mesh. All
    // four route through the same two generic accessors — proving it works
    // for this spread is what proves it works for the other ten kinds too,
    // since nothing in the implementation branches on shape kind.
    const sphere = try zjolt.Shape.initSphere(0.5, .{});
    defer sphere.release();
    try std.testing.expectEqual(@as(u64, 0), sphere.userData());
    sphere.setUserData(0xdead_beef);
    try std.testing.expectEqual(@as(u64, 0xdead_beef), sphere.userData());

    // TWO children, at different positions: a single child at the origin
    // with no rotation is a case Jolt SIMPLIFIES away into the child shape
    // itself, which would make this accidentally test `sphere` a second
    // time rather than a real compound.
    const box = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer box.release();
    const compound = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(sphere, .{ .position = zjolt.vec3(-1, 0, 0) }),
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(1, 0, 0) }),
    });
    defer compound.release();
    try std.testing.expectEqual(@as(u64, 0), compound.userData());
    compound.setUserData(111);

    const scaled = try zjolt.Shape.initScaled(sphere, zjolt.vec3(2, 2, 2));
    defer scaled.release();
    try std.testing.expectEqual(@as(u64, 0), scaled.userData());
    scaled.setUserData(222);

    const vertices = [_]zjolt.Vec3{
        zjolt.vec3(0, 0, 0), zjolt.vec3(1, 0, 0), zjolt.vec3(0, 0, 1),
    };
    const indices = [_]u32{ 0, 1, 2 };
    const mesh = try zjolt.Shape.initMesh(&vertices, &indices, .{});
    defer mesh.release();
    try std.testing.expectEqual(@as(u64, 0), mesh.userData());
    mesh.setUserData(333);

    // Each one kept its OWN value rather than sharing state through some
    // process-wide slot — the failure mode a naive "one global" shim would
    // have.
    try std.testing.expectEqual(@as(u64, 0xdead_beef), sphere.userData());
    try std.testing.expectEqual(@as(u64, 111), compound.userData());
    try std.testing.expectEqual(@as(u64, 222), scaled.userData());
    try std.testing.expectEqual(@as(u64, 333), mesh.userData());
}

test "a convex shape's density can be read and changed after creation, and the change reaches later mass properties" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(0.5, .{ .density = 500 });
    defer sphere.release();
    try std.testing.expectApproxEqAbs(@as(f32, 500), try sphere.density(), 1e-3);

    const mass_before = sphere.massProperties().mass;

    try sphere.setDensity(1500);
    try std.testing.expectApproxEqAbs(@as(f32, 1500), try sphere.density(), 1e-3);

    // Tripling the density triples the mass computed from this shape's
    // volume — the setter reaching a live field, not a creation-time-only
    // copy nothing downstream ever reads again.
    const mass_after = sphere.massProperties().mass;
    try std.testing.expectApproxEqAbs(mass_before * 3.0, mass_after, mass_before * 0.01);

    // A mesh has no density of its own.
    const vertices = [_]zjolt.Vec3{
        zjolt.vec3(0, 0, 0), zjolt.vec3(1, 0, 0), zjolt.vec3(0, 0, 1),
    };
    const indices = [_]u32{ 0, 1, 2 };
    const mesh = try zjolt.Shape.initMesh(&vertices, &indices, .{});
    defer mesh.release();
    try std.testing.expectError(error.InvalidArgument, mesh.density());
    try std.testing.expectError(error.InvalidArgument, mesh.setDensity(1.0));
}

test "a mesh's per-triangle user data is the host's own value when supplied at construction" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // Same flat two-triangle square the material test above uses, so the
    // ray placements that land cleanly on one triangle or the other are
    // already known good.
    const vertices = [_]zjolt.Vec3{
        zjolt.vec3(-1, 0, -1), zjolt.vec3(1, 0, -1),
        zjolt.vec3(-1, 0, 1),  zjolt.vec3(1, 0, 1),
    };
    const indices = [_]u32{ 0, 2, 1, 1, 2, 3 };
    const triangle_user_data = [_]u32{ 111, 222 };

    const mesh = try zjolt.Shape.initMesh(&vertices, &indices, .{
        .triangle_user_data = &triangle_user_data,
    });
    defer mesh.release();

    const bodies = world.system.bodies();
    const mesh_body = try bodies.createAndAdd(.{
        .shape = mesh,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();
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

    // The host's own values come back exactly, not Jolt's pre-reorder
    // fallback index (which for this mesh would be 0 and 1) — the whole
    // point of supplying triangle_user_data at all.
    try std.testing.expectEqual(@as(u32, 111), mesh.meshTriangleUserData(hit_a.sub_shape_id));
    try std.testing.expectEqual(@as(u32, 222), mesh.meshTriangleUserData(hit_b.sub_shape_id));
}

test "sub-shape index and id convert into each other, and drill down to the same leaf" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(0.2, 0.2, 0.2), .{});
    defer box.release();
    const sphere = try zjolt.Shape.initSphere(0.3, .{});
    defer sphere.release();
    const capsule = try zjolt.Shape.initCapsule(0.2, 0.1, .{});
    defer capsule.release();

    // THREE children rather than two: with exactly two, the compound needs
    // only 1 sub-shape id bit, and ZJOLT_SUB_SHAPE_ID_EMPTY's low bit (all
    // sub-shape ids are all-ones) happens to decode to a valid index by
    // coincidence. Three children need 2 bits, where the same coincidence
    // does not land on a valid index — which is what lets the "empty is not
    // valid for a compound" assertion below mean something.
    const compound = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(-2, 0, 0) }),
        zjolt.compoundChild(sphere, .{ .position = zjolt.vec3(2, 0, 0) }),
        zjolt.compoundChild(capsule, .{ .position = zjolt.vec3(0, 5, 0) }),
    });
    defer compound.release();

    // Index -> id -> index round trips, and the remainder is empty because
    // each child here is a leaf convex shape with no sub-shape id space of
    // its own.
    const sphere_id = try compound.subShapeIDFromIndex(1);
    const back = try compound.subShapeIndexFromID(sphere_id);
    try std.testing.expectEqual(@as(u32, 1), back.index);
    try std.testing.expectEqual(zjolt.sub_shape_id_empty, back.remainder);

    // The id drills down to the actual sphere, not the box or the compound
    // itself — a wrong index-to-id mapping would land on the wrong child.
    const leaf = compound.leafShape(sphere_id) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(zjolt.ShapeSubType.sphere, leaf.shape.subType());
    try std.testing.expectEqual(zjolt.sub_shape_id_empty, leaf.remainder);

    // A box's own empty id is valid for the box in isolation, but is not a
    // valid id for the compound that wraps it — the compound has leaves, so
    // an id addressing it has to name one of them.
    try std.testing.expect(box.isSubShapeIDValid(zjolt.sub_shape_id_empty));
    try std.testing.expect(!compound.isSubShapeIDValid(zjolt.sub_shape_id_empty));
    try std.testing.expect(compound.isSubShapeIDValid(sphere_id));

    // A box around only the sphere's position intersects exactly child 1.
    const box_around_sphere = zjolt.AABox{
        .min = zjolt.vec3(1, -1, -1),
        .max = zjolt.vec3(3, 1, 1),
    };
    const hits = try compound.intersectingSubShapes(box_around_sphere, std.testing.allocator);
    defer std.testing.allocator.free(hits);
    try std.testing.expectEqual(@as(usize, 1), hits.len);
    try std.testing.expectEqual(@as(u32, 1), hits[0]);
}

test "SubShapeIdCreator addresses a grandchild in a nested compound, one level at a time" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(0.2, .{});
    defer sphere.release();
    const box = try zjolt.Shape.initBox(zjolt.vec3(0.1, 0.1, 0.1), .{});
    defer box.release();
    const capsule = try zjolt.Shape.initCapsule(0.2, 0.1, .{});
    defer capsule.release();

    // inner's two children sit close together, so a bounding-box query
    // around the pair would hit both. The level-by-level path below
    // addresses one exactly.
    const inner = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(sphere, .{ .position = zjolt.vec3(0, 0, 0) }),
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(0.3, 0, 0) }),
    });
    defer inner.release();
    const outer = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(inner, .{ .position = zjolt.vec3(10, 0, 0) }),
        zjolt.compoundChild(capsule, .{ .position = zjolt.vec3(-10, 0, 0) }),
    });
    defer outer.release();

    const creator = try shape_mod.SubShapeIdCreator.init();
    defer creator.deinit();
    try outer.subShapeIDFromIndexInto(creator, 0); // outer's child 0 = inner
    try inner.subShapeIDFromIndexInto(creator, 1); // inner's child 1 = box
    const composed = creator.id();

    // Round trip: decode two levels and confirm each index.
    const level1 = try outer.subShapeIndexFromID(composed);
    try std.testing.expectEqual(@as(u32, 0), level1.index);
    const level2 = try inner.subShapeIndexFromID(level1.remainder);
    try std.testing.expectEqual(@as(u32, 1), level2.index);
    try std.testing.expectEqual(zjolt.sub_shape_id_empty, level2.remainder);

    // Drills to the actual box, not the sphere or either compound.
    const leaf = outer.leafShape(composed) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(zjolt.ShapeSubType.box, leaf.shape.subType());

    // A creator started fresh and pushed through
    // zjoltShapeGetSubShapeIDFromIndex (the pre-existing, direct-child-only
    // entry point) for just the first level should equal the id
    // subShapeIDFromIndex(outer, 0) reports -- confirms the chained path and
    // the plain path agree at the root.
    const direct_child_id = try outer.subShapeIDFromIndex(0);
    const creator2 = try shape_mod.SubShapeIdCreator.init();
    defer creator2.deinit();
    try outer.subShapeIDFromIndexInto(creator2, 0);
    try std.testing.expectEqual(direct_child_id, creator2.id());
}

test "MutableCompound.replaceChild swaps the shape in place without shifting other children's ids" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const small = try zjolt.Shape.initBox(zjolt.vec3(0.2, 0.2, 0.2), .{});
    defer small.release();
    const big = try zjolt.Shape.initBox(zjolt.vec3(3, 3, 3), .{});
    defer big.release();

    var compound = try zjolt.MutableCompound.init(&.{
        zjolt.compoundChild(small, .{ .position = zjolt.vec3(-1, 0, 0) }),
        zjolt.compoundChild(small, .{ .position = zjolt.vec3(1, 0, 0) }),
    });
    defer compound.release();

    const id_before = try compound.asShape().subShapeIDFromIndex(1);

    const before = compound.asShape().localBounds();
    try std.testing.expect(before.max.x < 1.5);

    // Same position, but child 0 becomes the much bigger box.
    try compound.replaceChild(0, big, zjolt.vec3(-1, 0, 0), zjolt.quat_identity);

    const after = compound.asShape().localBounds();
    // The bounds grew to cover the NEW shape's own extent — moveChild alone
    // could never do this, since the position did not change.
    try std.testing.expect(after.min.x < before.min.x - 1.0);

    // Child 1's id is exactly what it was before child 0 changed — a
    // remove-then-add of child 0 would instead have shifted it.
    const id_after = try compound.asShape().subShapeIDFromIndex(1);
    try std.testing.expectEqual(id_before, id_after);
}

test "HeightFieldShape.setHeights repaints samples in place" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A gradient, not a flat field: Jolt fixes the quantised height RANGE at
    // construction from the samples it is given, so a perfectly flat field
    // (every sample 0) locks the encodable range to zero width and no later
    // SetHeights call could ever move a sample away from 0 again. Values 0
    // through 15 give the encoding real range to work with.
    const sample_count = 4;
    var samples: [sample_count * sample_count]f32 = undefined;
    for (&samples, 0..) |*s, i| s.* = @floatFromInt(i);
    const field = try zjolt.Shape.initHeightField(&samples, sample_count, .{});
    defer field.release();

    try std.testing.expectApproxEqAbs(@as(f32, 0), field.heightFieldPosition(0, 0).y, 0.2);
    const far_corner_before = field.heightFieldPosition(3, 3).y;

    // A 2x2 block starting at (0, 0) — block-aligned at the default block
    // size of 2 — raised to height 8, still within the range the field was
    // built to encode.
    const new_heights = [_]f32{ 8, 8, 8, 8 };
    try field.heightFieldSetHeights(
        0,
        0,
        2,
        2,
        &new_heights,
        zjolt.default_active_edge_cos_threshold_angle,
    );

    // The repainted corner reflects the new height...
    try std.testing.expectApproxEqAbs(@as(f32, 8), field.heightFieldPosition(0, 0).y, 0.2);
    // ...and a sample outside the touched block was left alone.
    try std.testing.expectApproxEqAbs(far_corner_before, field.heightFieldPosition(3, 3).y, 0.2);
}

test "min_height_value/max_height_value reserve headroom a flat field's own samples would not" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A perfectly flat field: every sample the same height. This is exactly
    // the case the test above avoids on purpose — with no reserved range,
    // Jolt derives the tightest one that fits, which for a flat field is a
    // single point, and quantisation has nowhere else to put anything.
    const sample_count = 4;
    var flat: [sample_count * sample_count]f32 = undefined;
    for (&flat) |*s| s.* = 5;
    const far_heights = [_]f32{ 500, 500, 500, 500 };

    // Built with no explicit range (the default): the auto-derived range
    // pins to the samples given here, [5, 5].
    const narrow = try zjolt.Shape.initHeightField(&flat, sample_count, .{});
    defer narrow.release();
    try std.testing.expectApproxEqAbs(@as(f32, 5), narrow.heightFieldMinHeightValue(), 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 5), narrow.heightFieldMaxHeightValue(), 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 5), narrow.heightFieldPosition(0, 0).y, 0.01);

    try narrow.heightFieldSetHeights(
        0,
        0,
        2,
        2,
        &far_heights,
        zjolt.default_active_edge_cos_threshold_angle,
    );
    // 500 has nowhere to go in a [5, 5] range: it is clamped straight back to
    // (approximately) where it started, nowhere close to what was asked for.
    try std.testing.expect(@abs(narrow.heightFieldPosition(0, 0).y - 500) > 1.0);

    // The SAME flat samples, but with an explicit, wider range reserved up
    // front. The field still reports the samples it was actually given...
    const wide = try zjolt.Shape.initHeightField(&flat, sample_count, .{
        .min_height_value = 0,
        .max_height_value = 1000,
    });
    defer wide.release();
    try std.testing.expectApproxEqAbs(@as(f32, 0), wide.heightFieldMinHeightValue(), 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 1000), wide.heightFieldMaxHeightValue(), 0.01);
    // Generous tolerance: an 8-bit field with 1000 units of headroom to
    // resolve is coarser than the earlier gradient test's, and precision here
    // is not the point — landing anywhere near 5 rather than being clamped
    // to it exactly is.
    try std.testing.expectApproxEqAbs(@as(f32, 5), wide.heightFieldPosition(0, 0).y, 15.0);

    // ...and now the SAME later setHeights call lands close to where it was
    // asked to, because the range it needs was reserved from the start.
    try wide.heightFieldSetHeights(
        0,
        0,
        2,
        2,
        &far_heights,
        zjolt.default_active_edge_cos_threshold_angle,
    );
    try std.testing.expectApproxEqAbs(@as(f32, 500), wide.heightFieldPosition(0, 0).y, 15.0);
}

test "Shape.submergedVolume computes buoyancy for an ordinary shape, and refuses one with a mesh, height field or plane in its tree" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const radius = 1.0;
    const sphere = try zjolt.Shape.initSphere(radius, .{});
    defer sphere.release();

    // The water line passes exactly through the sphere's own center, so
    // half its volume is submerged — a formula simple enough to check
    // independently of whatever the sphere's own volume getter reports.
    const surface = zjolt.Plane{
        .normal = zjolt.vec3(0, 1, 0),
        .constant = 0,
    };
    const result = try sphere.submergedVolume(zjolt.mat44_identity, null, surface);
    const expected_total = (4.0 / 3.0) * std.math.pi * radius * radius * radius;
    try std.testing.expectApproxEqAbs(@as(f32, expected_total), result.total_volume, 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, expected_total / 2.0), result.submerged_volume, 0.01);

    // A compound with a plane leaf inherits the hazard from its child, not
    // just from being a plane itself.
    const plane = try zjolt.Shape.initPlane(zjolt.vec3(0, 1, 0), 0, .{});
    defer plane.release();
    const compound = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(sphere, .{ .position = zjolt.vec3(-5, 0, 0) }),
        zjolt.compoundChild(plane, .{ .position = zjolt.vec3(5, 0, 0) }),
    });
    defer compound.release();
    try std.testing.expectError(
        error.InvalidArgument,
        compound.submergedVolume(zjolt.mat44_identity, null, surface),
    );
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

test "setMaterial swaps a shape's material live, and every body sharing that shape sees it through a query hit" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const ice = try zjolt.PhysicsMaterial.init(.{ .debug_name = "ice" });
    defer ice.release();
    const asphalt = try zjolt.PhysicsMaterial.init(.{ .debug_name = "asphalt" });
    defer asphalt.release();

    const sphere = try zjolt.Shape.initSphere(0.5, .{ .material = ice });
    defer sphere.release();

    // Two bodies built from the SAME shape handle, well separated and well
    // above the fixture's floor so each ray only ever meets its own sphere.
    const bodies = world.system.bodies();
    const left = try bodies.createAndAdd(.{
        .shape = sphere,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(-5, 5, 0),
    }, .dont_activate);
    const right = try bodies.createAndAdd(.{
        .shape = sphere,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(5, 5, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();

    const rayAt = struct {
        fn at(q: zjolt.Queries, x: f32) !zjolt.RayCastHit {
            return (try q.castRayClosest(
                zjolt.rvec3(x, 15, 0),
                zjolt.vec3(0, -20, 0),
                null,
                null,
            )) orelse return error.TestUnexpectedResult;
        }
    }.at;

    const before_left = try rayAt(queries, -5);
    const before_right = try rayAt(queries, 5);
    try std.testing.expectEqual(left, before_left.body);
    try std.testing.expectEqual(right, before_right.body);
    // Resolved by the query pipeline itself, not re-read from the shape: this
    // is what proves the change is visible to collision, not just to the
    // getter that mirrors the same field.
    try std.testing.expectEqual(ice.handle, before_left.material.?);
    try std.testing.expectEqual(ice.handle, before_right.material.?);

    try sphere.setMaterial(asphalt);

    const after_left = try rayAt(queries, -5);
    const after_right = try rayAt(queries, 5);
    // One shape object, shared by both bodies: the swap on `sphere` reaches
    // both without touching either body.
    try std.testing.expectEqual(asphalt.handle, after_left.material.?);
    try std.testing.expectEqual(asphalt.handle, after_right.material.?);

    // NULL puts back Jolt's own shared default, the same one an unset
    // `material` at creation installs.
    try sphere.setMaterial(null);
    const default_material = zjolt.PhysicsMaterial.default() orelse
        return error.TestUnexpectedResult;
    const after_default = try rayAt(queries, -5);
    try std.testing.expectEqual(default_material.handle, after_default.material.?);

    // A mesh has no single material of its own to replace.
    const vertices = [_]zjolt.Vec3{
        zjolt.vec3(0, 0, 0), zjolt.vec3(1, 0, 0), zjolt.vec3(0, 0, 1),
    };
    const indices = [_]u32{ 0, 1, 2 };
    const mesh = try zjolt.Shape.initMesh(&vertices, &indices, .{});
    defer mesh.release();
    try std.testing.expectError(error.InvalidArgument, mesh.setMaterial(asphalt));
}

//=============================================================================
// Collision groups
//=============================================================================

/// Fires two equal spheres at each other from opposite sides with no gravity,
/// and returns the FINAL x of the left one minus the x of the right one.
/// Checked on position, not whether a contact was reported. If they collide,
/// they rest at contact (restitution 0), still in their original order, so the
/// result stays negative near -0.8 (their combined radii). If they do not, they
/// swap places and the result is strongly positive.
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

/// Collide only when the two sub-group ids share a parity — a rule over an
/// unbounded id space, which a fixed table cannot hold at any size.
///
/// Not synchronised: `World` runs a single-threaded job system, so the whole
/// step is on the calling thread. A callback filter in a threaded system
/// would need its own synchronisation.
const SameParityOnly = struct {
    calls: usize = 0,

    fn canCollide(
        user: ?*anyopaque,
        _: u32,
        sub_group_id1: u32,
        _: u32,
        sub_group_id2: u32,
    ) callconv(.c) bool {
        const self: *SameParityOnly = @ptrCast(@alignCast(user.?));
        self.calls += 1;
        return sub_group_id1 % 2 == sub_group_id2 % 2;
    }
};

test "a callback group filter decides pairs no bit table could hold" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var counter: SameParityOnly = .{};
    const filter = try zjolt.GroupFilter.initCustom(.{
        .can_collide = SameParityOnly.canCollide,
        .user = @ptrCast(&counter),
    });
    defer filter.release();

    try std.testing.expect(!filter.isTable());
    try std.testing.expectEqual(@as(u32, 0), filter.numSubGroups());

    // Sub-groups 0 and 2 share a parity, so they collide and rest at contact.
    // Both are past what a two-sub-group table could even index.
    const collided = try finalXSeparation(
        .{ .filter = filter, .group_id = 1, .sub_group_id = 0 },
        .{ .filter = filter, .group_id = 1, .sub_group_id = 2 },
    );
    try std.testing.expectApproxEqAbs(@as(f64, -0.8), collided, 0.2);
    try std.testing.expect(counter.calls > 0);

    // 0 and 1 do not, so they pass through each other.
    const passed = try finalXSeparation(
        .{ .filter = filter, .group_id = 1, .sub_group_id = 0 },
        .{ .filter = filter, .group_id = 1, .sub_group_id = 1 },
    );
    try std.testing.expect(passed > 1.0);
}

fn alwaysCollide(_: ?*anyopaque, _: u32, _: u32, _: u32, _: u32) callconv(.c) bool {
    return true;
}

test "the table methods refuse a callback filter rather than casting it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const custom = try zjolt.GroupFilter.initCustom(.{ .can_collide = alwaysCollide });
    defer custom.release();

    // A callback filter has no bit table, so there is nothing for these
    // three to read or write. The refusal names the entry point that made
    // it, rather than a downcast reading another object's memory.
    try std.testing.expectError(error.InvalidArgument, custom.disableCollision(0, 1));
    try std.testing.expect(
        std.mem.indexOf(u8, zjolt.lastError(), "zjoltGroupFilterCustomCreate") != null,
    );
    try std.testing.expectError(error.InvalidArgument, custom.enableCollision(0, 1));
    try std.testing.expectError(error.InvalidArgument, custom.isCollisionEnabled(0, 1));

    // A table filter answers all three, and says which kind it is.
    const table = try zjolt.GroupFilter.initTable(2);
    defer table.release();
    try std.testing.expect(table.isTable());
    try std.testing.expectEqual(@as(u32, 2), table.numSubGroups());
    try table.disableCollision(0, 1);
    try std.testing.expect(!try table.isCollisionEnabled(0, 1));
}

test "a callback filter without its callback is refused at creation" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Nothing sensible to answer with, so the refusal is here rather than on
    // the first pair, inside a step, on a job thread.
    try std.testing.expectError(
        error.InvalidArgument,
        zjolt.GroupFilter.initCustom(.{}),
    );
}

//=============================================================================
// Shape introspection
//
// An opaque `Shape` alone does not say what it was built from — `subType`
// says which kind it is; these say with what.
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
    // radius, not the half height of the whole shape. Both are asserted
    // because the two are easily conflated.
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

//=============================================================================
// PolyhedronSubmergedVolumeCalculator
//=============================================================================

test "PolyhedronSubmergedVolumeCalculator: a unit box half submerged has half its volume, centred a quarter deep" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A unit cube (extent 1 on every axis) centred on the origin: corners at
    // (+-0.5, +-0.5, +-0.5). Points 0, 1, 4, 5 tie for deepest once the
    // surface (below) bisects the box; the calculator keeps the first one
    // seen, so index 0 is the reference point and every face below skips it.
    const points = [_]zjolt.Vec3{
        zjolt.vec3(-0.5, -0.5, -0.5), // 0
        zjolt.vec3(0.5, -0.5, -0.5), // 1
        zjolt.vec3(0.5, 0.5, -0.5), // 2
        zjolt.vec3(-0.5, 0.5, -0.5), // 3
        zjolt.vec3(-0.5, -0.5, 0.5), // 4
        zjolt.vec3(0.5, -0.5, 0.5), // 5
        zjolt.vec3(0.5, 0.5, 0.5), // 6
        zjolt.vec3(-0.5, 0.5, 0.5), // 7
    };
    // Surface at y = 0, normal up: the box's lower half (y in [-0.5, 0]) is
    // submerged.
    const surface = zjolt.Plane{ .normal = zjolt.vec3(0, 1, 0), .constant = 0 };

    const calc = try shape_mod.PolyhedronSubmergedVolumeCalculator.init(
        zjolt.mat44_identity,
        &points,
        surface,
    );
    defer calc.deinit();

    try std.testing.expect(!calc.areAllAbove());
    try std.testing.expect(!calc.areAllBelow());
    try std.testing.expectEqual(@as(u32, 0), calc.referencePointIdx());

    // The three faces not touching the reference point (+X, +Y, +Z), fan
    // triangulated and wound counter-clockwise as seen from outside.
    try calc.addFace(4, 5, 6);
    try calc.addFace(4, 6, 7);
    try calc.addFace(1, 2, 6);
    try calc.addFace(1, 6, 5);
    try calc.addFace(3, 7, 6);
    try calc.addFace(3, 6, 2);

    // A face using the reference point is refused, not silently a no-op.
    try std.testing.expectError(error.InvalidArgument, calc.addFace(0, 1, 2));

    const res = calc.result();
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), res.submerged_volume, 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0), res.center_of_buoyancy.x, 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, -0.25), res.center_of_buoyancy.y, 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0), res.center_of_buoyancy.z, 1e-4);
}

test "PolyhedronSubmergedVolumeCalculator: AreAllAbove and AreAllBelow short-circuit without faces" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const points = [_]zjolt.Vec3{
        zjolt.vec3(-0.5, -0.5, -0.5),
        zjolt.vec3(0.5, 0.5, 0.5),
    };

    {
        // Surface far below both points: everything is above.
        const surface = zjolt.Plane{ .normal = zjolt.vec3(0, 1, 0), .constant = 100 };
        const calc = try shape_mod.PolyhedronSubmergedVolumeCalculator.init(
            zjolt.mat44_identity,
            &points,
            surface,
        );
        defer calc.deinit();
        try std.testing.expect(calc.areAllAbove());
        try std.testing.expect(!calc.areAllBelow());
    }
    {
        // Surface far above both points: everything is below.
        const surface = zjolt.Plane{ .normal = zjolt.vec3(0, 1, 0), .constant = -100 };
        const calc = try shape_mod.PolyhedronSubmergedVolumeCalculator.init(
            zjolt.mat44_identity,
            &points,
            surface,
        );
        defer calc.deinit();
        try std.testing.expect(!calc.areAllAbove());
        try std.testing.expect(calc.areAllBelow());
    }
}

//=============================================================================
// Support function
//=============================================================================

test "Shape.supportFunction reaches a box's own support, with and without its convex radius" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A convex radius of 0.3, well above Jolt's own 0.05 cap on how much
    // ExcludeConvexRadius mode will actually remove (@see `SupportMode`).
    const box = try zjolt.Shape.initBox(zjolt.vec3(1, 2, 3), .{ .convex_radius = 0.3 });
    defer box.release();

    // Excluding the convex radius shrinks the support shape by the CAPPED
    // radius (0.05, not the shape's own 0.3), and GetConvexRadius reports
    // that same capped value back for the caller to add separately.
    var buffer: shape_mod.SupportBuffer = undefined;
    const excluded = try box.supportFunction(.exclude_convex_radius, &buffer, null);
    const p = excluded.support(zjolt.vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 0.95), p.x, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 1.95), p.y, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 2.95), p.z, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0.05), excluded.convexRadius(), 1e-6);

    // Including it, the support shape is the box's own full extent, and
    // GetConvexRadius reports 0 -- there is nothing left to add.
    var buffer2: shape_mod.SupportBuffer = undefined;
    const included = try box.supportFunction(.include_convex_radius, &buffer2, null);
    const q = included.support(zjolt.vec3(1, 0, 0));
    try std.testing.expectApproxEqAbs(@as(f32, 1), q.x, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 2), q.y, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 3), q.z, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0), included.convexRadius(), 1e-6);

    // A non-convex shape (a compound) has no support function at all. Two
    // children, so Jolt does not simplify this back down to a plain box.
    const compound = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(-5, 0, 0) }),
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(5, 0, 0) }),
    });
    defer compound.release();
    var buffer3: shape_mod.SupportBuffer = undefined;
    try std.testing.expectError(
        error.InvalidArgument,
        compound.supportFunction(.default, &buffer3, null),
    );
}

//=============================================================================
// Sub-shape id decode
//=============================================================================

test "popSubShapeId decodes what SubShapeIdCreator.push encoded, parents before children" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const creator = try zjolt.SubShapeIdCreator.init();
    defer creator.deinit();
    try creator.push(3, 4);
    try creator.push(5, 4);
    const id = creator.id();

    const first = try shape_mod.popSubShapeId(id, 4);
    try std.testing.expectEqual(@as(u32, 3), first.value);

    const second = try shape_mod.popSubShapeId(first.remainder, 4);
    try std.testing.expectEqual(@as(u32, 5), second.value);

    // Bits above 32 (JPH::SubShapeID::MaxBits) are refused rather than
    // reaching Jolt's own shift-by-more-than-width undefined behaviour.
    try std.testing.expectError(error.InvalidArgument, shape_mod.popSubShapeId(id, 33));
}

//=============================================================================
// SIMD batch helpers
//=============================================================================

test "sortReverseAndStore sorts descending and keeps the entries below the threshold" {
    var identifiers: [4]u32 = .{ 10, 20, 30, 40 };
    var out_values: [4]f32 = undefined;
    const count = shape_mod.sortReverseAndStore(
        .{ 3, 1, 4, 2 },
        3.5,
        &identifiers,
        &out_values,
    );

    // Sorted descending: (4,30) (3,10) (2,40) (1,20). Below 3.5: the last
    // three, moved to the front in the same order.
    try std.testing.expectEqual(@as(usize, 3), count);
    try std.testing.expectApproxEqAbs(@as(f32, 3), out_values[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 2), out_values[1], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), out_values[2], 1e-6);
    try std.testing.expectEqual(@as(u32, 10), identifiers[0]);
    try std.testing.expectEqual(@as(u32, 40), identifiers[1]);
    try std.testing.expectEqual(@as(u32, 20), identifiers[2]);
}

test "countAndSortTrues moves the flagged identifiers to the front, in order" {
    var identifiers: [4]u32 = .{ 1, 2, 3, 4 };
    const count = shape_mod.countAndSortTrues(.{ true, false, true, false }, &identifiers);

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(u32, 1), identifiers[0]);
    try std.testing.expectEqual(@as(u32, 3), identifiers[1]);
}
