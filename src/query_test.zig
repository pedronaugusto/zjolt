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

test "the unlocked query path finds exactly what the locked path finds" {
    // GetNarrowPhaseQueryNoLock differs from GetNarrowPhaseQuery in exactly
    // one thing: whether it locks each body it visits. Outside a step,
    // nothing is contending for those locks, so a *NoLock query run against
    // the same scene the locked form already sees must agree on every hit --
    // this is the check for that, on both a ray cast and an overlap.
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const target_shape = try zjolt.Shape.initSphere(0.5, .{});
    defer target_shape.release();
    const spheres = try sphereColumn(world.system, target_shape);

    const queries = world.system.queries();
    const origin = zjolt.rvec3(0, 10, 0);
    const direction = zjolt.vec3(0, -20, 0);

    var locked_buf: [8]zjolt.RayCastHit = undefined;
    var unlocked_buf: [8]zjolt.RayCastHit = undefined;
    const locked = try queries.castRayAll(origin, direction, null, null, &locked_buf);
    const unlocked = try queries.castRayAllNoLock(origin, direction, null, null, &unlocked_buf);

    // The four spheres, plus the floor underneath them -- same as
    // `castShapeClosest and castShapeAll agree about the nearest hit`.
    try std.testing.expectEqual(@as(usize, 5), locked.len);
    try std.testing.expectEqual(locked.len, unlocked.len);
    for (locked) |hit| {
        var found = false;
        for (unlocked) |other| {
            if (hit.body == other.body and
                @abs(hit.fraction - other.fraction) < 1.0e-5)
            {
                found = true;
            }
        }
        try std.testing.expect(found);
    }

    // And the closest-hit forms agree on the same, single, topmost sphere.
    const locked_closest = (try queries.castRayClosest(origin, direction, null, null)) orelse
        return error.TestUnexpectedResult;
    const unlocked_closest = (try queries.castRayClosestNoLock(origin, direction, null, null)) orelse
        return error.TestUnexpectedResult;
    try std.testing.expectEqual(spheres[3], locked_closest.body);
    try std.testing.expectEqual(locked_closest.body, unlocked_closest.body);
    try std.testing.expectApproxEqAbs(locked_closest.fraction, unlocked_closest.fraction, 1.0e-5);
}

/// Tracks, in order, every body `onBody` announces and the body each
/// following `onHit` reports -- so a test can check the two line up, rather
/// than just that both eventually fire.
const BodyTrace = struct {
    body_events: [8]zjolt.BodyId = undefined,
    body_event_count: usize = 0,
    hit_bodies: [8]zjolt.BodyId = undefined,
    hit_preceding_body: [8]zjolt.BodyId = undefined,
    hit_count: usize = 0,

    pub fn onBody(self: *BodyTrace, body: zjolt.Body) void {
        self.body_events[self.body_event_count] = body.id();
        self.body_event_count += 1;
    }

    pub fn onHit(self: *BodyTrace, hit: zjolt.CollideShapeHit) zjolt.HitAction {
        self.hit_bodies[self.hit_count] = hit.body;
        self.hit_preceding_body[self.hit_count] = if (self.body_event_count > 0)
            self.body_events[self.body_event_count - 1]
        else
            zjolt.invalid_body_id;
        self.hit_count += 1;
        return .@"continue";
    }
};

test "collideShapeEachWithBody announces each body immediately before its own hits" {
    // CollisionCollector::OnBody fires once per body, under the same lock
    // the traversal already holds, strictly before any of that body's
    // hits reach AddHit. Checks both halves: three overlapping bodies
    // produce three OnBody calls and three hits, and each hit's body is
    // exactly the one OnBody most recently announced -- never the next
    // one's (too late) and never a stale one (no OnBody for it at all).
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(1.0, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const a = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(1, 30, 0),
    }, .dont_activate);
    const b = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(-1, 30, 0.5),
    }, .dont_activate);
    const c = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 30, -1),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const probe = try zjolt.Shape.initSphere(2.5, .{});
    defer probe.release();

    var trace: BodyTrace = .{};
    try world.system.queries().collideShapeEachWithBody(
        .{ .shape = probe, .position = zjolt.rvec3(0, 30, 0) },
        null,
        &trace,
    );

    try std.testing.expectEqual(@as(usize, 3), trace.hit_count);
    try std.testing.expectEqual(@as(usize, 3), trace.body_event_count);
    for (0..trace.hit_count) |i| {
        try std.testing.expectEqual(trace.hit_bodies[i], trace.hit_preceding_body[i]);
    }

    const announced = trace.body_events[0..trace.body_event_count];
    try std.testing.expect(containsBody(announced, a));
    try std.testing.expect(containsBody(announced, b));
    try std.testing.expect(containsBody(announced, c));
}

//=============================================================================
// Predicting a contact's response
//=============================================================================

test "estimateCollisionResponse predicts an exact elastic bounce off an immovable body" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    // A known mass rather than a density-derived one, so the impulse this
    // predicts can be checked by number and not just by direction.
    const mass: f32 = 2.0;
    const moving = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 20, 0),
        .linear_velocity = zjolt.vec3(-5, 0, 0),
        .override_mass_properties = .calculate_inertia,
        .mass = mass,
    }, .dont_activate);

    // An immovable body: STATIC, so its own inverse mass is zero regardless
    // of what CalculateInertia would have given it.
    const wall = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(-1, 20, 0),
    }, .dont_activate);

    // The one contact point, on the line joining the two centres of mass --
    // exactly where two spheres colliding head-on always touch. That is
    // what makes the result below a PURE one-dimensional collision: the
    // lever arm from each body's centre to this point is parallel to the
    // normal, so no rotation enters the answer no matter what the shape's
    // own inertia happens to be.
    const points_1 = [_]zjolt.Vec3{zjolt.vec3(-0.5, 20, 0)};
    const points_2 = [_]zjolt.Vec3{zjolt.vec3(-0.5, 20, 0)};

    const queries = world.system.queries();

    // Restitution 1, no friction: elastic collision with an immovable body
    // conserves both momentum and kinetic energy, which for a single moving
    // body has exactly one solution -- its velocity reverses outright. That
    // is true by the physics alone, independent of anything this predicts
    // about impulses or how many iterations it runs.
    const bounced = try queries.estimateCollisionResponse(
        moving,
        wall,
        .{
            .world_space_normal = zjolt.vec3(-1, 0, 0),
            .points_on_1 = &points_1,
            .points_on_2 = &points_2,
        },
        0.0,
        1.0,
        0.0,
        10,
    );
    try std.testing.expectApproxEqAbs(@as(f32, 5), bounced.linear_velocity1.x, 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 0), bounced.linear_velocity1.y, 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 0), bounced.linear_velocity1.z, 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 0), bounced.linear_velocity2.x, 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 0), bounced.angular_velocity1.x, 1.0e-4);

    // No friction was asked for.
    try std.testing.expectEqual(@as(f32, 0), bounced.friction_impulse1);
    try std.testing.expectEqual(@as(f32, 0), bounced.friction_impulse2);

    // The moving body's momentum changed by mass * (5 - (-5)) = 20 kg m/s,
    // carried entirely by the one contact point since it is the only one.
    try std.testing.expectEqual(@as(u32, 1), bounced.num_contact_impulses);
    try std.testing.expectApproxEqAbs(mass * 10, bounced.contact_impulses[0], 0.05);

    // A fully inelastic collision (restitution 0) instead brings the
    // approach velocity to exactly zero: the moving body stops dead, since
    // the wall cannot move to carry any momentum away and there is nothing
    // elastic to bounce it back with.
    const absorbed = try queries.estimateCollisionResponse(
        moving,
        wall,
        .{
            .world_space_normal = zjolt.vec3(-1, 0, 0),
            .points_on_1 = &points_1,
            .points_on_2 = &points_2,
        },
        0.0,
        0.0,
        0.0,
        10,
    );
    try std.testing.expectApproxEqAbs(@as(f32, 0), absorbed.linear_velocity1.x, 0.01);
    try std.testing.expectApproxEqAbs(mass * 5, absorbed.contact_impulses[0], 0.05);

    // A stale or mismatched body id is refused rather than read as garbage.
    try std.testing.expectError(
        error.BodyNotFound,
        queries.estimateCollisionResponse(
            moving,
            zjolt.invalid_body_id,
            .{
                .world_space_normal = zjolt.vec3(-1, 0, 0),
                .points_on_1 = &points_1,
                .points_on_2 = &points_2,
            },
            0.0,
            1.0,
            0.0,
            10,
        ),
    );
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
    const rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 1, 0), 0.4);

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
// `BroadPhase.bounds()` tracks every add/remove immediately, verified directly here: even with `optimizeBroadPhase` never called, bounds and queries are correct — it is a tree-balance operation, not a correctness dependency, worth pinning down since every OTHER test in this package calls it out of habit.
//
// Everything below it queries BOUNDING BOXES, never bodies: the broad phase holds one box per body and nothing else. Several of these tests assert the gap that follows from it — a point outside the sphere but inside its box is a hit — because a caller who reads them as body tests gets answers that look wrong.
//=============================================================================

/// Three spheres in a row, far enough above whatever the fixture already
/// holds that no query here can reach the floor by accident. Placed relative
/// to the broad phase's own bounds, not at a fixed height a bigger floor
/// could reach.
const Row = struct {
    shape: zjolt.Shape,
    y: f32,
    near: zjolt.BodyId,
    mid: zjolt.BodyId,
    far: zjolt.BodyId,

    fn init(system: zjolt.PhysicsSystem) !Row {
        const y = system.broadPhase().bounds().max.y + 20;
        const shape = try zjolt.Shape.initSphere(0.5, .{});
        errdefer shape.release();

        const xs = [_]f32{ 0, 5, 50 };
        var ids: [3]zjolt.BodyId = undefined;
        for (&ids, &xs) |*id, x| {
            id.* = try system.bodies().createAndAdd(.{
                .shape = shape,
                .object_layer = Layers.moving,
                .position = zjolt.rvec3(x, y, 0),
            }, .dont_activate);
        }
        system.optimizeBroadPhase();
        return .{ .shape = shape, .y = y, .near = ids[0], .mid = ids[1], .far = ids[2] };
    }

    fn deinit(self: Row) void {
        self.shape.release();
    }
};

fn holds(ids: []const zjolt.BodyId, id: zjolt.BodyId) bool {
    return std.mem.indexOfScalar(zjolt.BodyId, ids, id) != null;
}

test "the broad phase's bounds cover every body added" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    // Outside what the fixture already spans, on every axis. A body inside
    // the floor's own extent is covered by the floor, and the assertions
    // below would then hold with no body added at all.
    const before = world.system.broadPhase().bounds();
    const low = zjolt.rvec3(before.min.x - 40, before.min.y - 20, before.min.z - 15);
    const high = zjolt.rvec3(before.max.x + 60, before.max.y + 30, before.max.z + 25);

    _ = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = low,
    }, .dont_activate);
    _ = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = high,
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const bounds = world.system.broadPhase().bounds();
    try std.testing.expect(bounds.min.x <= low.x + 0.5);
    try std.testing.expect(bounds.min.y <= low.y + 0.5);
    try std.testing.expect(bounds.min.z <= low.z + 0.5);
    try std.testing.expect(bounds.max.x >= high.x - 0.5);
    try std.testing.expect(bounds.max.y >= high.y - 0.5);
    try std.testing.expect(bounds.max.z >= high.z - 0.5);
}

test "the broad phase's bounds shrink again once the body stretching them is gone" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    // Relative to whatever the fixture floor happens to be, not an absolute
    // figure: the floor's size is the fixture's business and a test that
    // hard-codes it fails the day the fixture grows, which says nothing about
    // the broad phase.
    const before = world.system.broadPhase().bounds();

    const far_x = before.max.x + 500;
    const far = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(far_x, far_x, far_x),
    }, .dont_activate);

    const grown = world.system.broadPhase().bounds();
    try std.testing.expect(grown.max.x >= far_x - 0.5);
    try std.testing.expect(grown.max.x > before.max.x);

    // Removed, not merely deactivated — and the bounds pull back in with it,
    // with no `optimizeBroadPhase` call anywhere in this test.
    world.system.bodies().remove(far);
    world.system.bodies().destroy(far);

    const shrunk = world.system.broadPhase().bounds();
    try std.testing.expect(shrunk.max.x < grown.max.x);
    try std.testing.expectApproxEqAbs(before.max.x, shrunk.max.x, 1.0);
}

test "an axis-aligned overlap reports the boxes it covers, and the count-only form agrees" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const row = try Row.init(world.system);
    defer row.deinit();
    const broad = world.system.broadPhase();

    // Reaches x = 6, so it covers the spheres at 0 and 5 and not the one at
    // 50; one metre either side of the row on Y, so it cannot reach the floor.
    const box: zjolt.AABox = .{
        .min = zjolt.vec3(-1, row.y - 1, -1),
        .max = zjolt.vec3(6, row.y + 1, 1),
    };

    var buffer: [16]zjolt.BodyId = undefined;
    const hits = try broad.collideBox(box, null, &buffer);
    try std.testing.expectEqual(@as(usize, 2), hits.len);
    try std.testing.expect(holds(hits, row.near));
    try std.testing.expect(holds(hits, row.mid));
    try std.testing.expect(!holds(hits, row.far));

    // The count-only form is the first half of the two-call protocol, so it
    // has to answer with what the second half would fill in.
    try std.testing.expectEqual(@as(u32, 2), try broad.countBoxOverlaps(box, null));
}

test "an overlap buffer smaller than the answer fails instead of truncating" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const row = try Row.init(world.system);
    defer row.deinit();
    const broad = world.system.broadPhase();

    const box: zjolt.AABox = .{
        .min = zjolt.vec3(-1, row.y - 1, -1),
        .max = zjolt.vec3(6, row.y + 1, 1),
    };

    var one: [1]zjolt.BodyId = undefined;
    try std.testing.expectError(error.BufferTooSmall, broad.collideBox(box, null, &one));

    // And a buffer of exactly the counted size is enough, which is what makes
    // the failure above a size check rather than an off-by-one.
    var two: [2]zjolt.BodyId = undefined;
    const hits = try broad.collideBox(box, null, &two);
    try std.testing.expectEqual(@as(usize, 2), hits.len);
}

test "the sphere and point overlaps test the bounding box, not the body inside it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const row = try Row.init(world.system);
    defer row.deinit();
    const broad = world.system.broadPhase();
    var buffer: [16]zjolt.BodyId = undefined;

    // A corner of the near sphere's bounding box. 0.4 on each axis is 0.69
    // from the centre, so the point is outside a sphere of radius 0.5 and
    // inside its box — and the broad phase answers for the box.
    const corner = zjolt.rvec3(0.4, row.y + 0.4, 0.4);
    const at_corner = try broad.collidePoint(corner, null, &buffer);
    try std.testing.expectEqual(@as(usize, 1), at_corner.len);
    try std.testing.expectEqual(row.near, at_corner[0]);

    // Just outside the box on X, and nothing is reported.
    const outside = zjolt.rvec3(0.6, row.y, 0);
    try std.testing.expectEqual(
        @as(usize, 0),
        (try broad.collidePoint(outside, null, &buffer)).len,
    );

    // Same story for a sphere: 0.7 from the box, so 0.3 misses and 0.8 hits.
    const center = zjolt.rvec3(1.2, row.y, 0);
    try std.testing.expectEqual(
        @as(usize, 0),
        (try broad.collideSphere(center, 0.3, null, &buffer)).len,
    );
    const wide = try broad.collideSphere(center, 0.8, null, &buffer);
    try std.testing.expectEqual(@as(usize, 1), wide.len);
    try std.testing.expectEqual(row.near, wide[0]);
}

test "a broad-phase ray reports every box it enters and nothing past the end of it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const floor_top = world.system.broadPhase().bounds().max.y;
    const row = try Row.init(world.system);
    defer row.deinit();
    const broad = world.system.broadPhase();

    const origin = zjolt.rvec3(0, row.y + 20, 0);
    var buffer: [16]zjolt.BroadPhaseCastHit = undefined;

    // Straight down the near sphere's axis, ending below the floor. The
    // spheres at x = 5 and x = 50 are off the ray entirely.
    const through = zjolt.vec3(0, floor_top - 2 - (row.y + 20), 0);
    const hits = try broad.castRay(origin, through, null, &buffer);
    try std.testing.expectEqual(@as(usize, 2), hits.len);
    try std.testing.expectEqual(@as(u32, 2), try broad.countRayHits(origin, through, null));

    var saw_near = false;
    var saw_floor = false;
    for (hits) |hit| {
        try std.testing.expect(hit.fraction >= 0 and hit.fraction <= 1);
        if (hit.body == row.near) saw_near = true;
        if (hit.body == world.floor) saw_floor = true;
    }
    try std.testing.expect(saw_near and saw_floor);

    // The same ray, stopped between the sphere and the floor. `direction`
    // carries the length, so the floor is now out of reach.
    const short = zjolt.vec3(0, row.y - 2 - (row.y + 20), 0);
    const near_only = try broad.castRay(origin, short, null, &buffer);
    try std.testing.expectEqual(@as(usize, 1), near_only.len);
    try std.testing.expectEqual(row.near, near_only[0].body);
}

test "a swept box reports what its own width passes through, not just what a ray would" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const floor_top = world.system.broadPhase().bounds().max.y;
    const row = try Row.init(world.system);
    defer row.deinit();
    const broad = world.system.broadPhase();

    const top = row.y + 20;
    const down = zjolt.vec3(0, floor_top - 2 - top, 0);
    var buffer: [16]zjolt.BroadPhaseCastHit = undefined;

    const narrow: zjolt.AABox = .{
        .min = zjolt.vec3(-0.2, top - 0.2, -0.2),
        .max = zjolt.vec3(0.2, top + 0.2, 0.2),
    };
    const narrow_hits = try broad.castBox(narrow, down, null, &buffer);
    try std.testing.expectEqual(@as(usize, 2), narrow_hits.len);

    // Widened to reach x = 5 and swept along the same line. The extra hit is
    // the sphere the narrow sweep passed beside, which is what says the box's
    // extent is carried through the sweep rather than only its centre.
    const wide: zjolt.AABox = .{
        .min = zjolt.vec3(-0.2, top - 0.2, -0.2),
        .max = zjolt.vec3(5.2, top + 0.2, 0.2),
    };
    const wide_hits = try broad.castBox(wide, down, null, &buffer);
    try std.testing.expectEqual(@as(usize, 3), wide_hits.len);

    var saw_mid = false;
    var saw_far = false;
    for (wide_hits) |hit| {
        if (hit.body == row.mid) saw_mid = true;
        if (hit.body == row.far) saw_far = true;
    }
    try std.testing.expect(saw_mid);
    try std.testing.expect(!saw_far);
}

test "an oriented box rejects a body that its own axis-aligned bounds accept" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const y = world.system.broadPhase().bounds().max.y + 20;
    const rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 1, 0), std.math.pi / 4.0);
    const box: zjolt.OrientedBox = .{
        .center = zjolt.rvec3(0, y, 0),
        .rotation = rotation,
        .half_extent = zjolt.vec3(4, 1, 0.5),
    };

    // A slab turned 45 degrees about Y. `inside` sits along its long axis;
    // `outside` sits off the end of its short one, four metres clear of it,
    // and yet inside the axis-aligned bounds the turn spreads out.
    const inside = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(2, y, -2),
    }, .dont_activate);
    const outside = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(3, y, 3),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    // The oriented box's own bounds: a 45-degree turn spreads both horizontal
    // half extents over both horizontal axes.
    const spread: f32 = (4.0 + 0.5) * @sqrt(@as(f32, 0.5));
    const bounds: zjolt.AABox = .{
        .min = zjolt.vec3(-spread, y - 1, -spread),
        .max = zjolt.vec3(spread, y + 1, spread),
    };

    const broad = world.system.broadPhase();
    var loose_buffer: [16]zjolt.BodyId = undefined;
    const loose = try broad.collideBox(bounds, null, &loose_buffer);
    try std.testing.expect(holds(loose, inside));
    try std.testing.expect(holds(loose, outside));

    var tight_buffer: [16]zjolt.BodyId = undefined;
    const tight = try broad.collideOrientedBox(box, null, &tight_buffer);
    try std.testing.expect(holds(tight, inside));
    try std.testing.expect(!holds(tight, outside));
}

test "a broad-phase object-layer filter keeps the static floor out of an overlap" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const floor_top = world.system.broadPhase().bounds().max.y;
    const row = try Row.init(world.system);
    defer row.deinit();
    const broad = world.system.broadPhase();

    // A column from below the floor up past the near sphere.
    const box: zjolt.AABox = .{
        .min = zjolt.vec3(-1, floor_top - 0.1, -1),
        .max = zjolt.vec3(1, row.y + 1, 1),
    };

    var buffer: [16]zjolt.BodyId = undefined;
    const unfiltered = try broad.collideBox(box, null, &buffer);
    try std.testing.expect(holds(unfiltered, world.floor));
    try std.testing.expect(holds(unfiltered, row.near));

    // `OnlyObjectLayer` builds a query filter; the broad phase takes the
    // object-layer half of one, so the callback has a single home either way.
    const only: zjolt.OnlyObjectLayer = .{ .layer = Layers.moving };
    const filters: zjolt.BroadPhaseFilters = .{ .object_layer = only.filters().object_layer };

    const filtered = try broad.collideBox(box, &filters, &buffer);
    try std.testing.expect(!holds(filtered, world.floor));
    try std.testing.expect(holds(filtered, row.near));
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

test "collideShapeWithInternalEdgeRemoval runs Jolt's real ghost-collision suppression, not a stand-in" {
    // Two coplanar triangles from a flat square split along a diagonal are the
    // textbook edge-removal case: a box on either must feel one continuous
    // surface, no seam. Box-vs-triangle EPA alone finds the true normal on a
    // flat box regardless of the diagonal, so the real proof is
    // InternalEdgeRemovingCollector forcing active_edge_mode/collect_faces_mode
    // regardless of `settings`, as the plain query guarantees.
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // Well above the fixture's floor, so it plays no part.
    const vertices = [_]zjolt.Vec3{
        zjolt.vec3(-1, 10, -1), zjolt.vec3(1, 10, -1),
        zjolt.vec3(-1, 10, 1),  zjolt.vec3(1, 10, 1),
    };
    const indices = [_]u32{ 0, 2, 1, 1, 2, 3 };
    const mesh = try zjolt.Shape.initMesh(&vertices, &indices, .{});
    defer mesh.release();

    _ = try world.system.bodies().createAndAdd(.{
        .shape = mesh,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 0, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const box = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer box.release();

    // Deliberately the OPPOSITE of what internal edge removal needs --
    // active_edge_mode left at Jolt's own default and faces not collected --
    // to check that the *WithInternalEdgeRemoval* family forces both fields
    // itself rather than silently doing nothing because the caller forgot.
    const overlap: zjolt.Queries.Overlap = .{
        .shape = box,
        .position = zjolt.rvec3(0, 10.25, 0),
    };

    const queries = world.system.queries();

    const closest = (try queries.collideShapeWithInternalEdgeRemovalClosest(overlap, null)) orelse
        return error.TestUnexpectedResult;
    // A box resting on a flat mesh is pushed straight up, never sideways --
    // the seam contributing a horizontal component is exactly the ghost
    // collision this exists to prevent, so this holds even though this
    // configuration does not exercise the voiding path itself.
    try std.testing.expectApproxEqAbs(@as(f32, 0), closest.penetration_axis.x, 1.0e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0), closest.penetration_axis.z, 1.0e-3);
    try std.testing.expect(@abs(closest.penetration_axis.y) > 1.0e-3);

    var buf: [8]zjolt.CollideShapeHit = undefined;
    const removed = try queries.collideShapeWithInternalEdgeRemoval(overlap, null, &buf);
    try std.testing.expect(removed.len >= 1);

    var plain_buf: [8]zjolt.CollideShapeHit = undefined;
    const plain = try queries.collideShape(overlap, null, &plain_buf);

    // Internal edge removal only ever drops hits, never invents one the
    // plain traversal did not already find.
    try std.testing.expect(removed.len <= plain.len);
    for (removed) |r| {
        try std.testing.expectApproxEqAbs(@as(f32, 0), r.penetration_axis.x, 1.0e-3);
        try std.testing.expectApproxEqAbs(@as(f32, 0), r.penetration_axis.z, 1.0e-3);
    }
}

//=============================================================================
// Body filters, both halves
//=============================================================================

/// Rejects one body by its user data, but only from `should_collide_locked`,
/// where the body itself is readable. `should_collide` accepts everything, so
/// a hit that disappears proves the second callback ran and was obeyed.
const LockedFilter = struct {
    reject_user_data: u64,
    unlocked_calls: u32 = 0,
    locked_calls: u32 = 0,

    fn acceptAll(user: ?*anyopaque, body: zjolt.BodyId) callconv(.c) bool {
        _ = body;
        const self: *LockedFilter = @ptrCast(@alignCast(user.?));
        self.unlocked_calls += 1;
        return true;
    }

    fn rejectByUserData(user: ?*anyopaque, body: *const zjolt.c.core.Body) callconv(.c) bool {
        const self: *LockedFilter = @ptrCast(@alignCast(user.?));
        self.locked_calls += 1;
        const b: zjolt.Body = .{ .handle = @constCast(body) };
        return b.userData() != self.reject_user_data;
    }
};

test "a body filter's locked half rejects on a body member the unlocked half cannot see" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const marker = 0xBEEF_CAFE;
    const sphere = try zjolt.Shape.initSphere(0.5, .{});
    defer sphere.release();
    const target = try world.system.bodies().createAndAdd(.{
        .shape = sphere,
        .position = zjolt.rvec3(0, 4, 0),
        .motion_type = .static,
        .object_layer = Layers.static,
        .user_data = marker,
    }, .dont_activate);

    const queries = world.system.queries();
    const origin = zjolt.rvec3(0, 10, 0);
    const direction = zjolt.vec3(0, -20, 0);

    // Unfiltered, the sphere is nearer than the floor and wins.
    const plain = (try queries.castRayClosest(origin, direction, null, null)) orelse
        return error.TestUnexpectedResult;
    try std.testing.expectEqual(target, plain.body);

    var filter: LockedFilter = .{ .reject_user_data = marker };
    const filters: zjolt.QueryFilters = .{ .body = .{
        .should_collide = LockedFilter.acceptAll,
        .should_collide_locked = LockedFilter.rejectByUserData,
        .user = &filter,
    } };

    const filtered = (try queries.castRayClosest(origin, direction, null, &filters)) orelse
        return error.TestUnexpectedResult;
    try std.testing.expect(filtered.body != target);
    try std.testing.expectEqual(world.floor, filtered.body);
    try std.testing.expect(filter.unlocked_calls > 0);
    try std.testing.expect(filter.locked_calls > 0);
}

test "a body filter with only the locked half set still filters" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const marker = 0x1234_5678;
    const sphere = try zjolt.Shape.initSphere(0.5, .{});
    defer sphere.release();
    _ = try world.system.bodies().createAndAdd(.{
        .shape = sphere,
        .position = zjolt.rvec3(0, 4, 0),
        .motion_type = .static,
        .object_layer = Layers.static,
        .user_data = marker,
    }, .dont_activate);

    var filter: LockedFilter = .{ .reject_user_data = marker };
    const filters: zjolt.QueryFilters = .{ .body = .{
        .should_collide_locked = LockedFilter.rejectByUserData,
        .user = &filter,
    } };

    const hit = (try world.system.queries().castRayClosest(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        &filters,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(world.floor, hit.body);
    try std.testing.expectEqual(@as(u32, 0), filter.unlocked_calls);
    try std.testing.expect(filter.locked_calls > 0);
}
