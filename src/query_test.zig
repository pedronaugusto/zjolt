//! Behavioural tests for queries, transformed shapes, the broad phase, and
//! batched body add/remove — four surfaces with plenty of entry points and,
//! until now, almost no behavioural coverage.
//!
//! Reuses `integration_test.zig`'s `World` fixture and `Layers` rather than
//! building its own — see `BINDING.md`. `integration_test.zig` already covers
//! the basic ray cast, the closest-vs-all agreement for rays, the streaming
//! `castRayEach` forms, and the back-face `settings` sweep at BOTH the system
//! and `TransformedShape` level; none of that is repeated here.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;
const World = fixture.World;

//=============================================================================
// Queries
//=============================================================================

/// Four static spheres in a column above the fixture's floor, spaced two
/// metres apart on Y. A shape swept straight down through them, plus the
/// floor, gives five distinct hits at five distinct fractions — the shape-cast
/// analogue of `integration_test.zig`'s own `Stack`, which this file cannot
/// reuse: it is private to that one.
fn sphereColumn(system: zjolt.PhysicsSystem, shape: zjolt.Shape) ![4]zjolt.BodyId {
    var ids: [4]zjolt.BodyId = undefined;
    for (&ids, 0..) |*id, i| {
        id.* = try system.bodies().createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.static,
            .motion_type = .static,
            .position = zjolt.rvec3(0, @floatFromInt(2 * (i + 1)), 0),
        }, .dont_activate);
    }
    system.optimizeBroadPhase();
    return ids;
}

test "castShapeClosest and castShapeAll agree about the nearest hit" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const target_shape = try zjolt.Shape.initSphere(0.5, .{});
    defer target_shape.release();
    const spheres = try sphereColumn(world.system, target_shape);

    const probe = try zjolt.Shape.initSphere(0.2, .{});
    defer probe.release();

    const cast: zjolt.Queries.ShapeCast = .{
        .shape = probe,
        .position = zjolt.rvec3(0, 10, 0),
        .direction = zjolt.vec3(0, -20, 0),
    };
    const queries = world.system.queries();

    var buffer: [8]zjolt.ShapeCastHit = undefined;
    const all = try queries.castShapeAll(cast, null, &buffer);
    // The four spheres, plus the floor underneath them.
    try std.testing.expectEqual(@as(usize, 5), all.len);

    var nearest = all[0];
    for (all[1..]) |hit| {
        if (hit.fraction < nearest.fraction) nearest = hit;
    }

    const closest = (try queries.castShapeClosest(cast, null)) orelse
        return error.TestUnexpectedResult;
    try std.testing.expectEqual(nearest.body, closest.body);
    try std.testing.expectApproxEqAbs(nearest.fraction, closest.fraction, 1.0e-4);
    // And it really is the topmost sphere the sweep meets first — y = 8,
    // the last one built — not the floor further down or an arbitrary one
    // of the four.
    try std.testing.expectEqual(spheres[3], closest.body);
}

test "collideShapeClosest returns the deepest overlap, not an arbitrary one" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const radius = 1.0;
    const target_shape = try zjolt.Shape.initSphere(radius, .{});
    defer target_shape.release();

    // Three spheres overlapping a probe centred at the origin, high above
    // the fixture's floor so it plays no part. Built shallowest first and
    // middling last, so a "closest" that secretly meant "first found" or
    // "last found" would answer with the wrong one either way — only the
    // deepest, built second, is correct.
    const bodies = world.system.bodies();
    const shallow = try bodies.createAndAdd(.{
        .shape = target_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(1.9, 20, 0), // depth = 2r - 1.9 = 0.1
    }, .dont_activate);
    const deep = try bodies.createAndAdd(.{
        .shape = target_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(-0.4, 20, 0), // depth = 2r - 0.4 = 1.6
    }, .dont_activate);
    const medium = try bodies.createAndAdd(.{
        .shape = target_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(-1.0, 20, 0), // depth = 2r - 1.0 = 1.0
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const probe = try zjolt.Shape.initSphere(radius, .{});
    defer probe.release();
    const overlap: zjolt.Queries.Overlap = .{
        .shape = probe,
        .position = zjolt.rvec3(0, 20, 0),
    };

    const queries = world.system.queries();
    const hit = (try queries.collideShapeClosest(overlap, null)) orelse
        return error.TestUnexpectedResult;

    try std.testing.expectEqual(deep, hit.body);
    try std.testing.expect(hit.body != shallow);
    try std.testing.expect(hit.body != medium);
    try std.testing.expectApproxEqAbs(@as(f32, 1.6), hit.penetration_depth, 0.05);

    // Cross-checked against the exhaustive form, the way
    // `integration_test.zig` checks `castRayClosest` against `castRayAll`:
    // the deepest of everything `collideShape` reports is this same hit.
    var buffer: [8]zjolt.CollideShapeHit = undefined;
    const all = try queries.collideShape(overlap, null, &buffer);
    try std.testing.expectEqual(@as(usize, 3), all.len);
    var deepest = all[0];
    for (all[1..]) |candidate| {
        if (candidate.penetration_depth > deepest.penetration_depth) deepest = candidate;
    }
    try std.testing.expectEqual(deepest.body, hit.body);
}

/// Records every shape-cast hit it is shown, and asks for nothing unless
/// `action` says otherwise. A shape-cast counterpart of
/// `integration_test.zig`'s own `CollectHits` — a local copy rather than an
/// import, since that one is not `pub` and, per `constraint_test.zig`'s
/// `AssertSink`, every subsystem file here stands on its own.
const CollectShapeHits = struct {
    bodies: [16]zjolt.BodyId = undefined,
    count: usize = 0,
    action: zjolt.HitAction = .@"continue",

    pub fn onHit(self: *CollectShapeHits, hit: zjolt.ShapeCastHit) zjolt.HitAction {
        if (self.count < self.bodies.len) self.bodies[self.count] = hit.body;
        self.count += 1;
        return self.action;
    }
};

fn containsBody(haystack: []const zjolt.BodyId, needle: zjolt.BodyId) bool {
    for (haystack) |id| {
        if (id == needle) return true;
    }
    return false;
}

test "castShapeEach visits exactly the hits castShapeAll collects, and stops on request" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const target_shape = try zjolt.Shape.initSphere(0.5, .{});
    defer target_shape.release();
    _ = try sphereColumn(world.system, target_shape);

    const probe = try zjolt.Shape.initSphere(0.2, .{});
    defer probe.release();

    const cast: zjolt.Queries.ShapeCast = .{
        .shape = probe,
        .position = zjolt.rvec3(0, 10, 0),
        .direction = zjolt.vec3(0, -20, 0),
    };
    const queries = world.system.queries();

    var buffer: [8]zjolt.ShapeCastHit = undefined;
    const all = try queries.castShapeAll(cast, null, &buffer);
    try std.testing.expectEqual(@as(usize, 5), all.len);

    var streamed: CollectShapeHits = .{};
    try queries.castShapeEach(cast, null, &streamed);

    // Same hits, as a set — neither form promises an order.
    try std.testing.expectEqual(all.len, streamed.count);
    for (all) |hit| {
        try std.testing.expect(containsBody(streamed.bodies[0..streamed.count], hit.body));
    }

    // And a callback that says stop is honoured after the very first hit,
    // rather than being treated as advice the traversal finishes anyway.
    var once: CollectShapeHits = .{ .action = .stop };
    try queries.castShapeEach(cast, null, &once);
    try std.testing.expectEqual(@as(usize, 1), once.count);
}

test "a query filtered to one object layer misses a body in another" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    // A body on the MOVING layer, directly above the fixture's STATIC floor,
    // so one ray straight down passes through a body of each layer. Static
    // motion type keeps it in place without a step — the same recipe
    // `integration_test.zig`'s own query test uses.
    const ball = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 2, 0),
        .motion_type = .static,
        .allow_dynamic_or_kinematic = true,
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();
    const origin = zjolt.rvec3(0, 10, 0);
    const direction = zjolt.vec3(0, -20, 0);

    // Unfiltered: both.
    try std.testing.expectEqual(
        @as(u32, 2),
        try queries.countRayHits(origin, direction, null, null),
    );

    // Filtered to the layer the ball is on: the floor is missing entirely,
    // not merely demoted to second place.
    const moving_only: zjolt.OnlyObjectLayer = .{ .layer = Layers.moving };
    const moving_filters = moving_only.filters();
    try std.testing.expectEqual(
        @as(u32, 1),
        try queries.countRayHits(origin, direction, null, &moving_filters),
    );
    const moving_hit = (try queries.castRayClosest(origin, direction, null, &moving_filters)) orelse
        return error.TestUnexpectedResult;
    try std.testing.expectEqual(ball, moving_hit.body);

    // And filtered the other way, it is the ball that goes missing.
    const static_only: zjolt.OnlyObjectLayer = .{ .layer = Layers.static };
    const static_filters = static_only.filters();
    try std.testing.expectEqual(
        @as(u32, 1),
        try queries.countRayHits(origin, direction, null, &static_filters),
    );
    const static_hit = (try queries.castRayClosest(origin, direction, null, &static_filters)) orelse
        return error.TestUnexpectedResult;
    try std.testing.expectEqual(world.floor, static_hit.body);
}

//=============================================================================
// TransformedShape
//=============================================================================

test "a ray against a transformed shape agrees with the same ray against the body it came from" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.5, 0.5), .{});
    defer shape.release();

    const position = zjolt.rvec3(3, 4, -2);
    // Rotated off-axis, so a bug that only shows up once rotation enters the
    // picture cannot hide behind a lucky symmetry.
    const rotation = try zjolt.quatFromAxisAngle(zjolt.vec3(0, 1, 0), 0.4);

    const body = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = position,
        .rotation = rotation,
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const ts = try zjolt.TransformedShape.init(shape, position, .{
        .rotation = rotation,
        .body = body,
    });
    defer ts.deinit();

    // A ray straight through the box's centre in X, so it hits regardless of
    // the box's own rotation about Y.
    const origin = zjolt.rvec3(0, 4, -2);
    const direction = zjolt.vec3(6, 0, 0);

    const from_system = (try world.system.queries().castRayClosest(origin, direction, null, null)) orelse
        return error.TestUnexpectedResult;
    const from_shape = (try ts.castRayClosest(origin, direction, null, null)) orelse
        return error.TestUnexpectedResult;

    try std.testing.expectEqual(body, from_system.body);
    try std.testing.expectEqual(body, from_shape.body);
    try std.testing.expectApproxEqAbs(from_system.fraction, from_shape.fraction, 1.0e-4);
    try std.testing.expectApproxEqAbs(from_system.normal.x, from_shape.normal.x, 1.0e-4);
    try std.testing.expectApproxEqAbs(from_system.normal.y, from_shape.normal.y, 1.0e-4);
    try std.testing.expectApproxEqAbs(from_system.normal.z, from_shape.normal.z, 1.0e-4);
}

test "moving a transformed shape moves the hit" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const ts = try zjolt.TransformedShape.init(shape, zjolt.rvec3(0, 0, 0), .{});
    defer ts.deinit();

    // A ray spanning x = -10 .. 10, long enough to still reach the sphere
    // after it moves.
    const origin = zjolt.rvec3(-10, 0, 0);
    const direction = zjolt.vec3(20, 0, 0);

    const first = (try ts.castRayClosest(origin, direction, null, null)) orelse
        return error.TestUnexpectedResult;
    // Hits the near face at x = -0.5: fraction (−0.5 − −10) / 20.
    try std.testing.expectApproxEqAbs(@as(f32, 0.475), first.fraction, 0.01);

    // Pulled six metres toward the ray's own origin.
    ts.setWorldTransform(zjolt.rvec3(-6, 0, 0), zjolt.quat_identity, null);

    const moved = (try ts.castRayClosest(origin, direction, null, null)) orelse
        return error.TestUnexpectedResult;
    // Now hits at x = -6.5: fraction (−6.5 − −10) / 20.
    try std.testing.expectApproxEqAbs(@as(f32, 0.175), moved.fraction, 0.01);
    try std.testing.expect(moved.fraction < first.fraction);
}

test "worldSpaceBounds tracks the placement" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(1, 2, 3), .{});
    defer shape.release();

    const start = zjolt.rvec3(5, -1, 2);
    const ts = try zjolt.TransformedShape.init(shape, start, .{});
    defer ts.deinit();

    const initial = ts.worldSpaceBounds();
    try std.testing.expectApproxEqAbs(@as(f32, 4), initial.min.x, 1.0e-3); // 5 - 1
    try std.testing.expectApproxEqAbs(@as(f32, 6), initial.max.x, 1.0e-3); // 5 + 1
    try std.testing.expectApproxEqAbs(@as(f32, -3), initial.min.y, 1.0e-3); // -1 - 2
    try std.testing.expectApproxEqAbs(@as(f32, 1), initial.max.y, 1.0e-3); // -1 + 2

    const shift = zjolt.rvec3(15, 9, -8);
    ts.setWorldTransform(shift, zjolt.quat_identity, null);
    const moved = ts.worldSpaceBounds();

    // The box only translates, so the bounds keep their size and shift by
    // exactly the displacement between the two placements.
    const dx: f32 = @floatCast(shift.x - start.x);
    const dy: f32 = @floatCast(shift.y - start.y);
    const dz: f32 = @floatCast(shift.z - start.z);
    try std.testing.expectApproxEqAbs(initial.min.x + dx, moved.min.x, 1.0e-3);
    try std.testing.expectApproxEqAbs(initial.max.x + dx, moved.max.x, 1.0e-3);
    try std.testing.expectApproxEqAbs(initial.min.y + dy, moved.min.y, 1.0e-3);
    try std.testing.expectApproxEqAbs(initial.max.y + dy, moved.max.y, 1.0e-3);
    try std.testing.expectApproxEqAbs(initial.min.z + dz, moved.min.z, 1.0e-3);
    try std.testing.expectApproxEqAbs(initial.max.z + dz, moved.max.z, 1.0e-3);
}

//=============================================================================
// Broad phase
//
// `BroadPhase.bounds()` turns out to track every add and remove immediately
// — verified against this binding directly, rather than assumed: even a
// system on which `optimizeBroadPhase` has never once been called reports
// correct bounds for what it holds, and a query finds a body added to it with
// no optimize call at all. `optimizeBroadPhase` is a tree-balance operation,
// not a step this correctness depends on — which is worth pinning down
// precisely because every OTHER test in this package calls it out of habit
// right after adding static geometry. What these two tests check is the
// thing that IS true unconditionally: the bounds match what is actually in
// the system, growing and shrinking with it.
//=============================================================================

test "the broad phase's bounds cover every body added" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    // Two bodies placed far apart and far from the fixture's floor (which
    // spans -50..50 in x and z), so the floor's own extent cannot be
    // mistaken for covering them.
    _ = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(-40, 30, 0),
    }, .dont_activate);
    _ = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(60, -20, 15),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const bounds = world.system.broadPhase().bounds();
    try std.testing.expect(bounds.min.x <= -40.5);
    try std.testing.expect(bounds.max.x >= 60.5);
    try std.testing.expect(bounds.min.y <= -20.5);
    try std.testing.expect(bounds.max.y >= 30.5);
    try std.testing.expect(bounds.min.z <= -0.5);
    try std.testing.expect(bounds.max.z >= 15.5);
}

test "the broad phase's bounds shrink again once the body stretching them is gone" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const before = world.system.broadPhase().bounds();
    try std.testing.expect(before.max.x < 100);

    const far = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(500, 500, 500),
    }, .dont_activate);

    const grown = world.system.broadPhase().bounds();
    try std.testing.expect(grown.max.x >= 499.5);

    // Removed, not merely deactivated — and the bounds pull back in with it,
    // with no `optimizeBroadPhase` call anywhere in this test.
    world.system.bodies().remove(far);
    world.system.bodies().destroy(far);

    const shrunk = world.system.broadPhase().bounds();
    try std.testing.expect(shrunk.max.x < 100);
    try std.testing.expect(shrunk.max.x < grown.max.x);
}

//=============================================================================
// Batch add / remove
//=============================================================================

test "a batched add puts every body in the system, and a batched remove takes them all out" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    var ids: [20]zjolt.BodyId = undefined;
    for (&ids, 0..) |*id, i| {
        const f: f32 = @floatFromInt(i);
        // Created, not added: exactly the state a batched add is for.
        id.* = try bodies.create(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(f * 2 - 20, 5, 0),
        });
    }
    for (ids) |id| try std.testing.expect(!bodies.isAdded(id));

    try world.system.batch().add(&ids, .dont_activate);
    for (ids) |id| try std.testing.expect(bodies.isAdded(id));

    // Findable, not just flagged: a ray through the middle of the row finds
    // one of them, which it could not if the batch had updated the body's
    // own state without ever inserting it into the broad phase.
    world.system.optimizeBroadPhase();
    const hit = try world.system.queries().castRayClosest(
        zjolt.rvec3(0, 5, 0),
        zjolt.vec3(0, -1, 0),
        null,
        null,
    );
    try std.testing.expect(hit != null);

    try world.system.batch().remove(&ids);
    for (ids) |id| try std.testing.expect(!bodies.isAdded(id));
}

test "a staged batch, once finalized, is added exactly like a direct add" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    var ids: [12]zjolt.BodyId = undefined;
    for (&ids, 0..) |*id, i| {
        const f: f32 = @floatFromInt(i);
        id.* = try bodies.create(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(f * 3 - 18, 5, 0),
        });
    }

    const staged = try world.system.batch().prepare(&ids);
    // Staged is not yet added: `prepare` sorts and stages without touching
    // the system's own state, per its own doc comment.
    for (ids) |id| try std.testing.expect(!bodies.isAdded(id));

    try world.system.batch().finalize(staged, .activate);
    for (ids) |id| {
        try std.testing.expect(bodies.isAdded(id));
        try std.testing.expect(bodies.isActive(id));
    }
}

test "aborting a staged batch leaves the system exactly as it was" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    var ids: [8]zjolt.BodyId = undefined;
    for (&ids, 0..) |*id, i| {
        const f: f32 = @floatFromInt(i);
        id.* = try bodies.create(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(f * 2, 5, 0),
        });
    }

    const staged = try world.system.batch().prepare(&ids);
    try world.system.batch().abort(staged);

    // Not half-applied: every one of them is exactly where `prepare` found
    // it — created, but not added. This is the case that matters: an abort
    // that half-applied would corrupt the broad phase silently, with no
    // error anywhere to catch it.
    for (ids) |id| try std.testing.expect(!bodies.isAdded(id));

    // And the system is not merely quiet about it — it still works: the
    // same ids can go through prepare and finalize afterwards as if the
    // abort had never happened.
    const staged_again = try world.system.batch().prepare(&ids);
    try world.system.batch().finalize(staged_again, .dont_activate);
    for (ids) |id| try std.testing.expect(bodies.isAdded(id));
}
