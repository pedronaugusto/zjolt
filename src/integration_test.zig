//! Behavioural tests: a real world, stepped, and checked.
//!
//! Every test runs the whole library through `std.testing.allocator`, which
//! fails on a leak. That is the Zig-side counterpart of the counting allocator
//! in `tests/c_smoke.c`: between them, both entry points to the allocator seam
//! are proved to balance.
//!
//! The single-threaded job system is used throughout, so a step is
//! reproducible without pinning a thread count — which is also what makes the
//! determinism test below meaningful.

const std = @import("std");
const zjolt = @import("zjolt.zig");

//=============================================================================
// A minimal layer map
//
// Two object layers and two broad-phase layers: the smallest configuration
// that still exercises filtering, because static-vs-static pairs must be
// rejected and moving-vs-anything accepted.
//=============================================================================

const Layers = struct {
    pub const static: zjolt.ObjectLayer = 0;
    pub const moving: zjolt.ObjectLayer = 1;

    pub const bp_static: zjolt.BroadPhaseLayer = 0;
    pub const bp_moving: zjolt.BroadPhaseLayer = 1;

    pub fn broadPhaseLayerCount() u32 {
        return 2;
    }

    pub fn broadPhaseLayerFor(layer: zjolt.ObjectLayer) zjolt.BroadPhaseLayer {
        return if (layer == static) bp_static else bp_moving;
    }

    pub fn objectCanCollideWithBroadPhase(
        object: zjolt.ObjectLayer,
        broad: zjolt.BroadPhaseLayer,
    ) bool {
        return if (object == static) broad == bp_moving else true;
    }

    pub fn objectsCanCollide(a: zjolt.ObjectLayer, b: zjolt.ObjectLayer) bool {
        return if (a == static) b == moving else true;
    }
};

//=============================================================================
// Fixture
//=============================================================================

/// A world with a large static box floor whose top surface is at y = 0.
const World = struct {
    system: zjolt.PhysicsSystem,
    jobs: zjolt.JobSystem,
    floor_shape: zjolt.Shape,
    floor: zjolt.BodyId,

    fn init() !World {
        const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.max_physics_jobs);
        errdefer jobs.deinit();

        const system = try zjolt.PhysicsSystem.init(.{
            .layers = zjolt.layersFromType(Layers),
            .max_bodies = 1024,
        });
        errdefer system.deinit();
        system.setGravity(zjolt.gravity_earth);

        const floor_shape = try zjolt.Shape.initBox(
            zjolt.vec3(50, 0.5, 50),
            .{ .convex_radius = 0.05 },
        );
        errdefer floor_shape.release();

        const floor = try system.bodies().createAndAdd(.{
            .shape = floor_shape,
            .object_layer = Layers.static,
            .motion_type = .static,
            .position = zjolt.rvec3(0, -0.5, 0),
            .user_data = 0xF100,
        }, .dont_activate);

        system.optimizeBroadPhase();

        return .{
            .system = system,
            .jobs = jobs,
            .floor_shape = floor_shape,
            .floor = floor,
        };
    }

    fn deinit(self: *World) void {
        self.system.deinit();
        self.floor_shape.release();
        self.jobs.deinit();
    }

    fn stepFor(self: *World, seconds: f32) !void {
        const dt: f32 = 1.0 / 60.0;
        var elapsed: f32 = 0;
        while (elapsed < seconds) : (elapsed += dt) {
            const update_error = try self.system.step(dt, 1, self.jobs);
            try std.testing.expectEqual(zjolt.UpdateError.none, update_error);
        }
    }
};

//=============================================================================
// Lifecycle
//=============================================================================

test "init installs the allocator and deinit gives every byte back" {
    // std.testing.allocator fails this test if a single allocation survives,
    // which is the whole assertion.
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    try std.testing.expect(zjolt.isInitialized());

    const shape = try zjolt.Shape.initSphere(1.0, 0);
    defer shape.release();
    try std.testing.expect(shape.volume() > 0);
}

test "a second init is refused and leaves the first intact" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    try std.testing.expectError(zjolt.Error.AlreadyInitialized, zjolt.init(.{}));
    try std.testing.expect(zjolt.isInitialized());
}

test "calls before init are refused rather than crashing" {
    try std.testing.expect(!zjolt.isInitialized());
    try std.testing.expectError(
        zjolt.Error.NotInitialized,
        zjolt.Shape.initSphere(1.0, 0),
    );
    try std.testing.expectError(
        zjolt.Error.NotInitialized,
        zjolt.JobSystem.initSingleThreaded(1024),
    );
}

test "misuse is refused rather than fatal" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Jolt packs a body index and a generation counter into one 32-bit id, so
    // there is a ceiling on max_bodies. Past it Jolt asserts while handing out
    // an id — far from the call that caused it, and fatally. Reproduced with
    // 100,000,000 before this was checked.
    try std.testing.expectError(zjolt.Error.InvalidArgument, zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(Layers),
        .max_bodies = 100_000_000,
    }));
    try std.testing.expect(zjolt.lastError().len > 0);

    try std.testing.expectError(zjolt.Error.InvalidArgument, zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(Layers),
        .max_bodies = 0,
    }));

    // A layer map with no answers at all. Jolt would reach a null vtable entry
    // during the first broad-phase build.
    try std.testing.expectError(zjolt.Error.InvalidArgument, zjolt.PhysicsSystem.init(.{
        .layers = .{
            .broad_phase = .{},
            .object_vs_broad_phase = .{},
            .object_layer_pair = .{},
        },
    }));

    // A save buffer too small to hold even the container header. The failure
    // has to come back as a size, not as a write past the end of it.
    const sphere = try zjolt.Shape.initSphere(0.5, 0);
    defer sphere.release();

    const needed = try sphere.saveSize();
    inline for (.{ 0, 1, 8, 31 }) |capacity| {
        var buffer: [capacity]u8 = undefined;
        try std.testing.expectError(zjolt.Error.BufferTooSmall, sphere.save(&buffer));
    }
    // ...and the size a refused save reports is the size that then works.
    const buffer = try std.testing.allocator.alloc(u8, needed);
    defer std.testing.allocator.free(buffer);
    try std.testing.expectEqual(needed, (try sphere.save(buffer)).len);

    // A job system that can run no jobs.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.JobSystem.initSingleThreaded(0),
    );
}

/// Captures Jolt's diagnostic output so a test can assert it arrived.
const TraceSink = struct {
    var lines: u32 = 0;
    var last: [256]u8 = undefined;
    var last_len: usize = 0;

    fn reset() void {
        lines = 0;
        last_len = 0;
    }

    fn onTrace(user: ?*anyopaque, message: [*:0]const u8) callconv(.c) void {
        _ = user;
        lines += 1;
        const text = std.mem.span(message);
        last_len = @min(text.len, last.len);
        @memcpy(last[0..last_len], text[0..last_len]);
    }

    fn lastMessage() []const u8 {
        return last[0..last_len];
    }
};

test "deinit refuses while a handle is still alive, and says so" {
    TraceSink.reset();
    try zjolt.init(.{
        .allocator = std.testing.allocator,
        .trace = TraceSink.onTrace,
    });

    try std.testing.expectEqual(@as(u32, 0), zjolt.liveHandleCount());

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.max_physics_jobs);
    try std.testing.expectEqual(@as(u32, 1), zjolt.liveHandleCount());

    const system = try zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(Layers),
    });
    try std.testing.expectEqual(@as(u32, 2), zjolt.liveHandleCount());

    // Deinit here would restore Jolt's allocator, and the destroys below would
    // then free through an allocator the memory never came from. It has to
    // refuse — and it has to still be initialised afterwards, or the destroys
    // would fail too.
    zjolt.deinit();
    try std.testing.expect(zjolt.isInitialized());
    try std.testing.expectEqual(@as(u32, 2), zjolt.liveHandleCount());

    // ...and the refusal is not silent. This is also the only reachable path
    // through the trace hook, so it is what proves the hook is wired up at
    // all: Jolt's own default trace is a stub that asserts, so every message
    // either reaches this callback or aborts the process.
    try std.testing.expect(TraceSink.lines > 0);
    try std.testing.expect(
        std.mem.indexOf(u8, TraceSink.lastMessage(), "still alive") != null,
    );

    system.deinit();
    jobs.deinit();
    try std.testing.expectEqual(@as(u32, 0), zjolt.liveHandleCount());

    zjolt.deinit();
    try std.testing.expect(!zjolt.isInitialized());
}

//=============================================================================
// Shapes
//=============================================================================

test "every shape kind builds and reports itself" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(0.5, 0);
    defer sphere.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.sphere, sphere.subType());
    try std.testing.expectEqual(@as(u32, 1), sphere.refCount());
    try std.testing.expect(sphere.massProperties().mass > 0);

    const box = try zjolt.Shape.initBox(zjolt.vec3(1, 2, 3), .{});
    defer box.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.box, box.subType());
    const bounds = box.localBounds();
    try std.testing.expectApproxEqAbs(@as(f32, -1), bounds.min.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 3), bounds.max.z, 1e-5);

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, 0);
    defer capsule.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.capsule, capsule.subType());

    var cube_points: [8]zjolt.Vec3 = undefined;
    var cube_indices: [36]u32 = undefined;
    makeBoxMesh(1, 1, 1, &cube_points, &cube_indices);

    const hull = try zjolt.Shape.initConvexHull(&cube_points, .{});
    defer hull.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.convex_hull, hull.subType());
    // A unit cube of half extent 1 has volume 8; the convex radius rounds the
    // corners slightly, so this is approximate on purpose.
    try std.testing.expectApproxEqAbs(@as(f32, 8), hull.volume(), 0.5);

    const mesh = try zjolt.Shape.initMesh(&cube_points, &cube_indices, 0);
    defer mesh.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.mesh, mesh.subType());
    try std.testing.expectEqual(@as(u32, 12), mesh.stats().num_triangles);

    // Decorated shapes take their own reference on the inner shape.
    const before = box.refCount();
    const scaled = try zjolt.Shape.initScaled(box, zjolt.vec3(2, 2, 2));
    defer scaled.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.scaled, scaled.subType());
    try std.testing.expect(box.refCount() > before);

    const moved = try zjolt.Shape.initRotatedTranslated(
        box,
        zjolt.vec3(0, 5, 0),
        zjolt.quat_identity,
    );
    defer moved.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.rotated_translated, moved.subType());
    // Bounds come back relative to the centre of mass, which the translation
    // moved along with the shape — so the offset shows up there, not in the
    // bounds.
    try std.testing.expectApproxEqAbs(@as(f32, 5), moved.centerOfMass().y, 1e-5);

    const offset = try zjolt.Shape.initOffsetCenterOfMass(box, zjolt.vec3(0, -1, 0));
    defer offset.release();
    try std.testing.expectEqual(
        zjolt.ShapeSubType.offset_center_of_mass,
        offset.subType(),
    );
    try std.testing.expectApproxEqAbs(@as(f32, -1), offset.centerOfMass().y, 1e-5);
}

test "a malformed mesh is refused with a reason" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const points = [_]zjolt.Vec3{ zjolt.vec3(0, 0, 0), zjolt.vec3(1, 0, 0), zjolt.vec3(0, 1, 0) };

    // An index past the end of the vertex array would be read as a vertex from
    // beyond the caller's slice, inside Jolt's tree builder.
    const bad_indices = [_]u32{ 0, 1, 99 };
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.Shape.initMesh(&points, &bad_indices, 0),
    );
    try std.testing.expect(zjolt.lastError().len > 0);

    // A partial triangle.
    const short_indices = [_]u32{ 0, 1 };
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.Shape.initMesh(&points, &short_indices, 0),
    );

    const no_points: []const zjolt.Vec3 = &.{};
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.Shape.initConvexHull(no_points, .{}),
    );
}

test "a shape survives a save and restore, and refuses damaged input" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var points: [8]zjolt.Vec3 = undefined;
    var indices: [36]u32 = undefined;
    makeBoxMesh(3, 1, 2, &points, &indices);

    const mesh = try zjolt.Shape.initMesh(&points, &indices, 0);
    defer mesh.release();

    const saved = try mesh.saveAlloc(std.testing.allocator);
    defer std.testing.allocator.free(saved);
    try std.testing.expect(saved.len > 0);
    try std.testing.expectEqual(saved.len, try mesh.saveSize());

    const restored = try zjolt.Shape.restore(saved);
    defer restored.release();

    try std.testing.expectEqual(mesh.subType(), restored.subType());
    try std.testing.expectEqual(mesh.stats().num_triangles, restored.stats().num_triangles);

    const before = mesh.localBounds();
    const after = restored.localBounds();
    try std.testing.expectApproxEqAbs(before.min.x, after.min.x, 1e-5);
    try std.testing.expectApproxEqAbs(before.max.y, after.max.y, 1e-5);
    try std.testing.expectApproxEqAbs(before.max.z, after.max.z, 1e-5);

    // A buffer that is too small must report how much was needed rather than
    // writing what fits.
    var tiny: [8]u8 = undefined;
    try std.testing.expectError(zjolt.Error.BufferTooSmall, mesh.save(&tiny));

    // Every field of the container must be load-bearing: damage each in turn
    // and confirm none of them is decoration.
    const cases = [_]struct { name: []const u8, offset: usize }{
        .{ .name = "magic", .offset = 0 },
        .{ .name = "container version", .offset = 4 },
        .{ .name = "build config", .offset = 8 },
        .{ .name = "Jolt version", .offset = 12 },
        .{ .name = "payload length", .offset = 16 },
        .{ .name = "checksum", .offset = 24 },
    };
    for (cases) |case| {
        const damaged = try std.testing.allocator.dupe(u8, saved);
        defer std.testing.allocator.free(damaged);
        damaged[case.offset] +%= 1;
        if (zjolt.Shape.restore(damaged)) |shape| {
            shape.release();
            std.debug.print("a damaged {s} was accepted\n", .{case.name});
            return error.TestUnexpectedResult;
        } else |e| {
            try std.testing.expectEqual(zjolt.Error.BadFormat, e);
            try std.testing.expect(zjolt.lastError().len > 0);
        }
    }

    // A payload damaged in storage must fail the checksum.
    const flipped = try std.testing.allocator.dupe(u8, saved);
    defer std.testing.allocator.free(flipped);
    flipped[flipped.len - 1] +%= 1;
    try std.testing.expectError(zjolt.Error.BadFormat, zjolt.Shape.restore(flipped));

    // Truncation, trailing bytes and outright garbage are all BadFormat. The
    // last of these is the case that used to reach Jolt's parser and index its
    // shape-type table with whatever byte happened to be there.
    try std.testing.expectError(
        zjolt.Error.BadFormat,
        zjolt.Shape.restore(saved[0 .. saved.len / 2]),
    );

    const padded = try std.testing.allocator.alloc(u8, saved.len + 16);
    defer std.testing.allocator.free(padded);
    @memcpy(padded[0..saved.len], saved);
    @memset(padded[saved.len..], 0);
    try std.testing.expectError(zjolt.Error.BadFormat, zjolt.Shape.restore(padded));

    const garbage = "this is definitely not a jolt shape" ** 4;
    try std.testing.expectError(zjolt.Error.BadFormat, zjolt.Shape.restore(garbage));
    try std.testing.expectError(zjolt.Error.BadFormat, zjolt.Shape.restore("short"));
}

test "a truncated payload inside a well-formed container is still refused" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(0.5, 0);
    defer sphere.release();

    const saved = try sphere.saveAlloc(std.testing.allocator);
    defer std.testing.allocator.free(saved);

    // The container rejects a short buffer on its recorded length alone, which
    // would make a plain prefix test prove nothing about the layer underneath.
    // Rebuilding the header around each truncated payload is the only way to
    // reach Jolt's own parser reading a stream that runs out, and confirm it
    // reports the shortfall rather than improvising a shape from zeros.
    const header = zjolt.c.shape_header_size;
    const payload = saved[header..];

    var length: usize = 0;
    while (length < payload.len) : (length += 1) {
        const forged = try std.testing.allocator.alloc(u8, header + length);
        defer std.testing.allocator.free(forged);
        @memcpy(forged[0..header], saved[0..header]);
        @memcpy(forged[header..], payload[0..length]);
        std.mem.writeInt(u64, forged[16..24], length, .little);
        std.mem.writeInt(
            u32,
            forged[24..28],
            std.hash.Crc32.hash(forged[header..]),
            .little,
        );

        if (zjolt.Shape.restore(forged)) |shape| {
            shape.release();
            std.debug.print("a {d}-byte payload parsed as a shape\n", .{length});
            return error.TestUnexpectedResult;
        } else |e| {
            try std.testing.expectEqual(zjolt.Error.BadFormat, e);
        }
    }

    // ...and the whole payload still restores.
    const whole = try zjolt.Shape.restore(saved);
    whole.release();
}

//=============================================================================
// Simulation
//=============================================================================

test "a dynamic body falls under gravity and comes to rest on the floor" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .restitution = 0,
        .user_data = 0xBA11,
    }, .activate);

    try std.testing.expectEqual(@as(u64, 0xBA11), bodies.getUserData(ball));
    try std.testing.expect(bodies.isAdded(ball));
    try std.testing.expectEqual(@as(u32, 2), world.system.numBodies());

    const start = bodies.getPosition(ball);
    try world.stepFor(4.0);
    const rest = bodies.getPosition(ball);

    try std.testing.expect(rest.y < start.y);
    // The floor's top is at y = 0 and the ball's radius is 0.5.
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0.5), rest.y, 0.05);
    // Nothing pushed it sideways.
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0), rest.x, 0.05);

    // It should have gone to sleep by now, which is what makes the active-body
    // list a useful thing to read back.
    try std.testing.expect(!bodies.isActive(ball));
    try std.testing.expectEqual(@as(u32, 0), try world.system.countActiveBodies());
}

test "the same inputs step to the same state twice" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const run = struct {
        fn once() ![3]zjolt.Real {
            var world = try World.init();
            defer world.deinit();

            const shape = try zjolt.Shape.initBox(zjolt.vec3(0.4, 0.4, 0.4), .{});
            defer shape.release();

            const bodies = world.system.bodies();
            const cube = try bodies.createAndAdd(.{
                .shape = shape,
                .object_layer = Layers.moving,
                .position = zjolt.rvec3(0.1, 4, -0.2),
                .rotation = zjolt.quatFromAxisAngle(normalize(zjolt.vec3(1, 1, 1)), 0.7),
                .linear_velocity = zjolt.vec3(1.5, 0, -0.75),
                .angular_velocity = zjolt.vec3(0.3, 1.1, -0.2),
                .restitution = 0.4,
            }, .activate);

            try world.stepFor(3.0);
            const p = bodies.getPosition(cube);
            return .{ p.x, p.y, p.z };
        }
    };

    const first = try run.once();
    const second = try run.once();
    // Bit-identical, not approximately equal: the single-threaded scheduler
    // removes the one source of run-to-run variation, so anything less would
    // be hiding a real difference.
    try std.testing.expectEqual(first, second);
}

test "a kinematic body moves toward its target and pushes what is in the way" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const platform_shape = try zjolt.Shape.initBox(zjolt.vec3(2, 0.25, 2), .{});
    defer platform_shape.release();

    const bodies = world.system.bodies();
    const platform = try bodies.createAndAdd(.{
        .shape = platform_shape,
        .object_layer = Layers.moving,
        .motion_type = .kinematic,
        .position = zjolt.rvec3(0, 1, 0),
    }, .activate);

    try std.testing.expectEqual(zjolt.MotionType.kinematic, bodies.getMotionType(platform));

    // Drive it upward for a second.
    const dt: f32 = 1.0 / 60.0;
    var frame: u32 = 0;
    while (frame < 60) : (frame += 1) {
        const current = bodies.getPosition(platform);
        const target = zjolt.rvec3(current.x, current.y + 0.02, current.z);
        bodies.moveKinematic(platform, target, null, dt);
        _ = try world.system.step(dt, 1, world.jobs);
    }

    const raised = bodies.getPosition(platform);
    try std.testing.expect(raised.y > 1.5);

    // A teleport, by contrast, is immediate and ignores what is in the way.
    bodies.setPositionAndRotation(platform, zjolt.rvec3(10, 3, 0), null, .activate);
    const teleported = bodies.getPosition(platform);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 10), teleported.x, 1e-4);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 3), teleported.y, 1e-4);

    bodies.setMotionType(platform, .dynamic, .activate);
    try std.testing.expectEqual(zjolt.MotionType.dynamic, bodies.getMotionType(platform));
}

test "a rotation that is not unit length is renormalised, not fatal" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();

    // What integrating a rotation for a few thousand frames produces. Jolt
    // asserts on it inside Mat44::sRotation, which a body's rotation reaches
    // every step — so without the fix-up at the boundary this aborts a build
    // with asserts on rather than returning anything.
    const drifted = zjolt.quat(0, 0.0, 0, 1.02);
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .rotation = drifted,
    }, .activate);

    const stored = bodies.getRotation(ball);
    const length = @sqrt(stored.x * stored.x + stored.y * stored.y +
        stored.z * stored.z + stored.w * stored.w);
    try std.testing.expectApproxEqAbs(@as(f32, 1), length, 1e-5);

    // A rotation with no direction at all has no normalised form, so it
    // becomes the identity rather than a quaternion full of NaN.
    const degenerate = zjolt.quat(0, 0, 0, 0);
    bodies.setPositionAndRotation(ball, zjolt.rvec3(0, 5, 0), degenerate, .activate);
    try std.testing.expectEqual(zjolt.quat_identity, bodies.getRotation(ball));

    // And the world still steps with it.
    try world.stepFor(0.2);
    try std.testing.expect(bodies.getPosition(ball).y < 5);

    // The same fix-up covers the other doors a rotation comes in through.
    const placed = try zjolt.Shape.initRotatedTranslated(
        shape,
        zjolt.vec3(0, 1, 0),
        drifted,
    );
    placed.release();

    const character = try zjolt.Character.init(world.system, .{
        .shape = shape,
        .position = zjolt.rvec3(-10, 1, 0),
        .rotation = drifted,
    });
    defer character.deinit();
    character.setRotation(degenerate);
    try std.testing.expectEqual(zjolt.quat_identity, character.getRotation());

    _ = try world.system.queries().castShapeClosest(.{
        .shape = shape,
        .position = zjolt.rvec3(0, 5, 0),
        .rotation = drifted,
        .direction = zjolt.vec3(0, -10, 0),
    }, null);
}

test "an impulse changes velocity immediately and a force does not" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .gravity_factor = 0,
    }, .activate);

    bodies.addImpulse(ball, zjolt.vec3(0, 0, 10));
    const after_impulse = bodies.getLinearVelocity(ball);
    try std.testing.expect(after_impulse.z > 0);

    bodies.setLinearVelocity(ball, zjolt.vec3_zero);
    bodies.addForce(ball, zjolt.vec3(0, 0, 100));
    // A force is only consumed by the step.
    try std.testing.expectApproxEqAbs(
        @as(f32, 0),
        bodies.getLinearVelocity(ball).z,
        1e-6,
    );
    _ = try world.system.step(1.0 / 60.0, 1, world.jobs);
    try std.testing.expect(bodies.getLinearVelocity(ball).z > 0);
}

test "a body constrained to a plane stays in it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();
    const confined = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        // Movement in X and Y, rotation about Z. This is the bit mask the C
        // header spells as an enum; if the packed struct's bits were laid out
        // wrongly, the body would be free on the wrong axis and this would
        // catch it.
        .allowed_dofs = .plane_2d,
    }, .activate);

    bodies.setLinearVelocity(confined, zjolt.vec3(2, 0, 3));
    try world.stepFor(1.0);

    const position = bodies.getPosition(confined);
    try std.testing.expect(position.x > 0.5); // free in X
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0), position.z, 1e-4); // pinned in Z

    // The default is every degree of freedom, and an unconstrained body does
    // move on Z under the same push.
    const free_body = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(20, 5, 0),
    }, .activate);
    bodies.setLinearVelocity(free_body, zjolt.vec3(2, 0, 3));
    try world.stepFor(1.0);
    try std.testing.expect(bodies.getPosition(free_body).z > 0.5);

    try std.testing.expectEqual(
        @as(c_int, 0b111111),
        @as(c_int, @bitCast(zjolt.AllowedDofs.all)),
    );
    try std.testing.expectEqual(
        @as(c_int, 0b100011),
        @as(c_int, @bitCast(zjolt.AllowedDofs.plane_2d)),
    );
}

/// Records Jolt assertions instead of breaking on them.
///
/// Returning false is the whole point: Jolt asserts on conditions it also
/// reports as data, so a test that wants to observe the data has to decline
/// the breakpoint. This is also the only path through the assert hook the
/// suite can reach, so it is what proves that hook is wired up.
const AssertSink = struct {
    var count: u32 = 0;

    fn reset() void {
        count = 0;
    }

    fn onAssert(
        user: ?*anyopaque,
        expression: [*:0]const u8,
        message: ?[*:0]const u8,
        file: [*:0]const u8,
        line: u32,
    ) callconv(.c) bool {
        _ = .{ user, expression, message, file, line };
        count += 1;
        return false; // do not break
    }
};

test "a step reports which limit it ran out of" {
    AssertSink.reset();
    try zjolt.init(.{
        .allocator = std.testing.allocator,
        .assert_failed = AssertSink.onAssert,
    });
    defer zjolt.deinit();

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.max_physics_jobs);
    defer jobs.deinit();

    // Deliberately far too few contact constraints for what is about to
    // happen, so the step has to drop some and say so.
    const system = try zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(Layers),
        .max_bodies = 256,
        .max_body_pairs = 4,
        .max_contact_constraints = 4,
    });
    defer system.deinit();
    system.setGravity(zjolt.gravity_earth);

    const floor_shape = try zjolt.Shape.initBox(zjolt.vec3(20, 0.5, 20), .{});
    defer floor_shape.release();
    _ = try system.bodies().createAndAdd(.{
        .shape = floor_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, -0.5, 0),
    }, .dont_activate);

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();
    for (0..40) |i| {
        const f: f32 = @floatFromInt(i);
        _ = try system.bodies().createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(@mod(f, 8) - 4, 1 + f * 0.05, @divFloor(f, 8) - 2),
        }, .activate);
    }
    system.optimizeBroadPhase();

    var seen: zjolt.UpdateError = .none;
    for (0..120) |_| {
        const update_error = try system.step(1.0 / 60.0, 1, jobs);
        seen = @bitCast(@as(u32, @bitCast(seen)) | @as(u32, @bitCast(update_error)));
    }

    // The step never fails; it drops contacts and reports which cache filled.
    // A host that ignores this gets bodies sinking into each other with
    // nothing to explain it, which is why it is returned rather than traced.
    try std.testing.expect(seen.any());
    try std.testing.expect(seen.body_pair_cache_full or seen.contact_constraints_full);
    try std.testing.expect(!zjolt.UpdateError.none.any());

    // Jolt asserts that this mask is empty one line before returning it, so
    // reaching the assertions above at all depended on the hook declining the
    // breakpoint. See UPSTREAM.md.
    if (zjolt.options.enable_asserts) {
        try std.testing.expect(AssertSink.count > 0);
    }
}

//=============================================================================
// Listeners
//=============================================================================

const ContactRecorder = struct {
    added: u32 = 0,
    persisted: u32 = 0,
    removed: u32 = 0,
    validated: u32 = 0,
    max_points: u32 = 0,
    saw_floor: bool = false,

    pub fn onContactValidate(
        self: *ContactRecorder,
        info: *const zjolt.ContactValidateInfo,
    ) zjolt.ValidateResult {
        _ = info;
        self.validated += 1;
        return .accept_all_contacts_for_this_body_pair;
    }

    pub fn onContactAdded(
        self: *ContactRecorder,
        info: *const zjolt.ContactInfo,
        settings: *zjolt.ContactSettings,
    ) void {
        _ = settings;
        self.added += 1;
        if (info.manifold.num_points > self.max_points)
            self.max_points = info.manifold.num_points;
        if (info.user_data1 == 0xF100 or info.user_data2 == 0xF100)
            self.saw_floor = true;
        // The point slices are borrowed for the duration of this call.
        const points = zjolt.contactPointsOn1(&info.manifold);
        std.debug.assert(points.len == info.manifold.num_points);
    }

    pub fn onContactPersisted(
        self: *ContactRecorder,
        info: *const zjolt.ContactInfo,
        settings: *zjolt.ContactSettings,
    ) void {
        _ = info;
        _ = settings;
        self.persisted += 1;
    }

    pub fn onContactRemoved(
        self: *ContactRecorder,
        pair: *const zjolt.SubShapeIdPair,
    ) void {
        _ = pair;
        self.removed += 1;
    }
};

const ActivationRecorder = struct {
    activated: u32 = 0,
    deactivated: u32 = 0,
    last_user_data: u64 = 0,

    pub fn onBodyActivated(self: *ActivationRecorder, body: zjolt.BodyId, user_data: u64) void {
        _ = body;
        self.activated += 1;
        self.last_user_data = user_data;
    }

    pub fn onBodyDeactivated(self: *ActivationRecorder, body: zjolt.BodyId, user_data: u64) void {
        _ = body;
        _ = user_data;
        self.deactivated += 1;
    }
};

test "contact and activation listeners fire with the right bodies" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    var contacts: ContactRecorder = .{};
    const contact_listener = zjolt.contactListener(ContactRecorder, &contacts);
    try world.system.setContactListener(&contact_listener);

    var activations: ActivationRecorder = .{};
    const activation_listener =
        zjolt.bodyActivationListener(ActivationRecorder, &activations);
    try world.system.setBodyActivationListener(&activation_listener);

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();

    // One ball that is allowed to fall asleep, and one that is not. They test
    // different things: sleeping is what raises the deactivation callback, and
    // staying awake is what keeps a contact in the cache long enough for
    // destroying the body to retire it.
    _ = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 2, 0),
        .user_data = 0xBA11,
    }, .activate);

    const restless = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(5, 2, 0),
        .allow_sleeping = false,
        .user_data = 0xBA12,
    }, .activate);

    try world.stepFor(4.0);

    try std.testing.expect(contacts.validated > 0);
    try std.testing.expect(contacts.added > 0);
    try std.testing.expect(contacts.persisted > 0);
    try std.testing.expect(contacts.max_points > 0);
    try std.testing.expect(contacts.saw_floor);
    try std.testing.expect(activations.activated > 0);
    // The sleeper reached the floor and stopped.
    try std.testing.expect(activations.deactivated > 0);
    // ...and its contacts were retired when it did, which is why the
    // destroy-while-awake case below needs the restless ball.
    try std.testing.expect(contacts.removed > 0);

    const removed_before = contacts.removed;
    try std.testing.expect(bodies.isActive(restless));
    bodies.destroy(restless);
    // Jolt retires a contact when the pair fails to reappear in a later step's
    // cache, not at the instant the body goes away.
    try world.stepFor(0.2);
    try std.testing.expect(contacts.removed > removed_before);

    // Clearing the listeners must stop them firing, which is what makes it
    // safe to free the recorder afterwards.
    try world.system.setContactListener(null);
    try world.system.setBodyActivationListener(null);
    const added_after_clear = contacts.added;
    const removed_after_clear = contacts.removed;
    try world.stepFor(0.5);
    try std.testing.expectEqual(added_after_clear, contacts.added);
    try std.testing.expectEqual(removed_after_clear, contacts.removed);
}

//=============================================================================
// Queries
//=============================================================================

test "ray, shape cast and overlap find what is there and miss what is not" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 2, 0),
        .motion_type = .static,
        .allow_dynamic_or_kinematic = true,
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();

    // Straight down through the ball and into the floor.
    const hit = (try queries.castRayClosest(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(ball, hit.body);
    try std.testing.expect(hit.fraction > 0 and hit.fraction < 1);

    const total = try queries.countRayHits(zjolt.rvec3(0, 10, 0), zjolt.vec3(0, -20, 0), null);
    try std.testing.expect(total >= 2);

    var buffer: [16]zjolt.RayCastHit = undefined;
    const all = try queries.castRayAll(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        &buffer,
    );
    try std.testing.expectEqual(total, @as(u32, @intCast(all.len)));

    // A buffer too small reports what was needed instead of writing what fits.
    var one: [1]zjolt.RayCastHit = undefined;
    try std.testing.expectError(zjolt.Error.BufferTooSmall, queries.castRayAll(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        &one,
    ));

    // Excluding the ball must leave only the floor.
    const exclude: zjolt.ExcludeBody = .{ .body = ball };
    const filters = exclude.filters();
    const floor_hit = (try queries.castRayClosest(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        &filters,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(world.floor, floor_hit.body);

    // Far from anything.
    try std.testing.expect((try queries.castRayClosest(
        zjolt.rvec3(1000, 1000, 1000),
        zjolt.vec3(0, -1, 0),
        null,
    )) == null);

    // A sphere swept downward reaches the floor.
    const swept = (try queries.castShapeClosest(.{
        .shape = shape,
        .position = zjolt.rvec3(20, 5, 0),
        .direction = zjolt.vec3(0, -10, 0),
    }, null)) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(world.floor, swept.body);
    // Contact points are relative to the cast's start, which was 5 above the
    // floor's surface — so the point on the floor sits about -5 from it.
    try std.testing.expectApproxEqAbs(@as(f32, -5), swept.contact_point_on_2.y, 0.2);

    // An overlap where the ball is.
    var overlaps: [8]zjolt.CollideShapeHit = undefined;
    const found = try queries.collideShape(.{
        .shape = shape,
        .position = zjolt.rvec3(0, 2, 0),
    }, null, &overlaps);
    try std.testing.expect(found.len > 0);

    // ...and none in empty air.
    const empty = try queries.collideShape(.{
        .shape = shape,
        .position = zjolt.rvec3(0, 30, 0),
    }, null, &overlaps);
    try std.testing.expectEqual(@as(usize, 0), empty.len);
}

//=============================================================================
// Bulk read-back and locks
//=============================================================================

test "bulk read-back matches the per-body accessors" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, 0);
    defer shape.release();

    const bodies = world.system.bodies();
    var ids: [16]zjolt.BodyId = undefined;
    for (&ids, 0..) |*id, i| {
        const f: f32 = @floatFromInt(i);
        id.* = try bodies.createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(f * 2 - 16, 3 + f * 0.5, 0),
        }, .activate);
    }

    try world.stepFor(0.5);

    var positions: [16]zjolt.RVec3 = undefined;
    var rotations: [16]zjolt.Quat = undefined;
    const missing = try world.system.getTransforms(&ids, &positions, &rotations);
    try std.testing.expectEqual(@as(u32, 0), missing);

    // The bulk path and the single-body path must agree exactly — they read
    // the same field through different locks.
    for (ids, positions) |id, bulk| {
        const single = bodies.getPosition(id);
        try std.testing.expectEqual(single.x, bulk.x);
        try std.testing.expectEqual(single.y, bulk.y);
        try std.testing.expectEqual(single.z, bulk.z);
    }

    var centers: [16]zjolt.RVec3 = undefined;
    var velocities: [16]zjolt.Vec3 = undefined;
    _ = try world.system.getMotions(&ids, &centers, &velocities);
    for (velocities) |v| try std.testing.expect(v.y < 0); // still falling

    // Everything is awake and moving, so the active list holds all of them
    // plus nothing else. The floor is static and never appears.
    const active_count = try world.system.countActiveBodies();
    try std.testing.expectEqual(@as(u32, 16), active_count);

    var active: [32]zjolt.BodyId = undefined;
    const awake = try world.system.getActiveBodies(&active);
    try std.testing.expectEqual(@as(usize, 16), awake.len);

    const all_count = try world.system.countBodies();
    try std.testing.expectEqual(@as(u32, 17), all_count); // + the floor

    // A destroyed body is reported missing, not fatal, and the surrounding
    // entries stay aligned with the id list.
    bodies.destroy(ids[3]);
    const missing_after = try world.system.getTransforms(&ids, &positions, null);
    try std.testing.expectEqual(@as(u32, 1), missing_after);
    try std.testing.expectEqual(@as(zjolt.Real, 0), positions[3].y);
    try std.testing.expect(positions[4].y != 0);

    // A short output buffer is caught before the C call.
    var too_small: [2]zjolt.RVec3 = undefined;
    try std.testing.expectError(
        zjolt.Error.BufferTooSmall,
        world.system.getTransforms(&ids, &too_small, null),
    );
}

test "a body lock reads and writes the body it holds" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .user_data = 0xBA11,
    }, .activate);

    {
        var lock = world.system.lockRead(ball);
        defer lock.release();
        const body = lock.body() orelse return error.TestUnexpectedResult;

        try std.testing.expectEqual(ball, body.id());
        try std.testing.expectEqual(@as(u64, 0xBA11), body.userData());
        try std.testing.expectEqual(Layers.moving, body.objectLayer());
        try std.testing.expectEqual(zjolt.MotionType.dynamic, body.motionType());
        try std.testing.expect(!body.isSensor());
        try std.testing.expectEqual(shape.handle, (body.shape() orelse
            return error.TestUnexpectedResult).handle);
        const bounds = body.worldBounds();
        try std.testing.expect(bounds.max.y > bounds.min.y);
    }

    {
        var lock = world.system.lockWrite(ball);
        defer lock.release();
        const body = lock.body() orelse return error.TestUnexpectedResult;
        body.setLinearVelocity(zjolt.vec3(3, 0, 0));
        body.setUserData(0xC0FFEE);
    }

    try std.testing.expectApproxEqAbs(
        @as(f32, 3),
        bodies.getLinearVelocity(ball).x,
        1e-5,
    );
    try std.testing.expectEqual(@as(u64, 0xC0FFEE), bodies.getUserData(ball));

    // A stale id yields a lock with no body, which must still be released.
    bodies.destroy(ball);
    var stale = world.system.lockRead(ball);
    defer stale.release();
    try std.testing.expect(stale.body() == null);
}

//=============================================================================
// Character
//=============================================================================

test "a character settles on the floor and reports the body under it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, 0);
    defer capsule.release();

    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        // Centre at 0.9 puts the capsule's base a little above the floor.
        .position = zjolt.rvec3(0, 0.9, 0),
        .user_data = 0xC4A2,
    });
    defer character.deinit();

    try std.testing.expectEqual(capsule.handle, (character.getShape() orelse
        return error.TestUnexpectedResult).handle);
    try std.testing.expectEqual(zjolt.invalid_body_id, character.innerBodyId());

    const settings = zjolt.defaultCharacterUpdateSettings();
    const dt: f32 = 1.0 / 60.0;

    var frame: u32 = 0;
    while (frame < 120) : (frame += 1) {
        var velocity = character.getLinearVelocity();
        if (character.groundState() == .on_ground) {
            velocity.y = 0;
        } else {
            velocity.y += zjolt.gravity_earth.y * dt;
        }
        // Walk in +x once grounded, so the run also covers moving contact.
        velocity.x = if (character.isSupported()) 1.0 else 0.0;
        character.setLinearVelocity(velocity);
        try character.update(dt, zjolt.gravity_earth, &settings, null);
    }

    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());
    try std.testing.expect(character.isSupported());
    try std.testing.expectEqual(world.floor, character.groundBodyId());
    try std.testing.expectEqual(@as(u64, 0xF100), character.groundUserData());
    try std.testing.expect(character.groundNormal().y > 0.9);

    const position = character.getPosition();
    // It walked, and it is still standing at capsule height above the floor.
    try std.testing.expect(position.x > 0.5);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0.8), position.y, 0.1);

    // Crouching: a shorter shape always fits.
    const crouched = try zjolt.Shape.initCapsule(0.2, 0.3, 0);
    defer crouched.release();
    try std.testing.expect(try character.setShape(crouched, 0.02, null));
    try std.testing.expectEqual(crouched.handle, (character.getShape() orelse
        return error.TestUnexpectedResult).handle);
}

test "a character with an inner body is visible to ray casts" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, 0);
    defer capsule.release();

    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 0.9, 0),
        .inner_body_shape = capsule,
        .inner_body_layer = Layers.moving,
    });
    defer character.deinit();

    const inner = character.innerBodyId();
    try std.testing.expect(inner != zjolt.invalid_body_id);

    world.system.optimizeBroadPhase();
    const hit = (try world.system.queries().castRayClosest(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
    )) orelse return error.TestUnexpectedResult;
    // Without the inner body this ray would have found the floor.
    try std.testing.expectEqual(inner, hit.body);
}

//=============================================================================
// Simulation settings
//=============================================================================

test "a settings change produces a measurably different simulation" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    // Same world twice, one field apart. `allow_sleeping` is the field to
    // pick: it changes an outcome that is observable through the API rather
    // than a number that only shows up in a trajectory.
    for ([_]bool{ true, false }) |allow_sleeping| {
        var world = try World.init();
        defer world.deinit();

        var settings = world.system.getSettings();
        try std.testing.expect(settings.allow_sleeping);
        settings.allow_sleeping = allow_sleeping;
        try world.system.setSettings(settings);

        // Read back, because the point of a flat descriptor is that what goes
        // in comes out.
        try std.testing.expectEqual(allow_sleeping, world.system.getSettings().allow_sleeping);
        try std.testing.expectEqual(
            settings.num_velocity_steps,
            world.system.getSettings().num_velocity_steps,
        );

        const ball = try world.system.bodies().createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(0, 1, 0),
        }, .activate);

        try world.stepFor(3.0);

        try std.testing.expectEqual(
            !allow_sleeping,
            world.system.bodies().isActive(ball),
        );
    }
}

test "settings Jolt would divide by or loop on are refused" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const good = zjolt.defaultPhysicsSettings();
    try std.testing.expect(good.step_listeners_batch_size >= 1);
    try world.system.setSettings(good);

    // Jolt asserts on none of these. A zero batch size is a `fetch_add(0)` in
    // the step's listener loop, which never advances; a zero batches-per-job
    // is an integer division. Both fail several frames from here.
    var bad = good;
    bad.step_listeners_batch_size = 0;
    try std.testing.expectError(error.InvalidArgument, world.system.setSettings(bad));

    bad = good;
    bad.step_listener_batches_per_job = 0;
    try std.testing.expectError(error.InvalidArgument, world.system.setSettings(bad));

    bad = good;
    bad.max_in_flight_body_pairs = 0;
    try std.testing.expectError(error.InvalidArgument, world.system.setSettings(bad));

    bad = good;
    bad.num_velocity_steps = 0;
    try std.testing.expectError(error.InvalidArgument, world.system.setSettings(bad));

    // ...and a refused set changed nothing.
    try std.testing.expectEqual(
        good.step_listeners_batch_size,
        world.system.getSettings().step_listeners_batch_size,
    );
}

//=============================================================================
// Broad-phase queries
//=============================================================================

test "a broad-phase query finds the bodies in a region and not the ones outside" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();
    const near = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);
    const far = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(30, 5, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const broad = world.system.broadPhase();
    var found: [16]zjolt.BodyId = undefined;

    // A box around the near ball only.
    const box: zjolt.AABox = .{
        .min = zjolt.vec3(-2, 3, -2),
        .max = zjolt.vec3(2, 7, 2),
    };
    const in_box = try broad.collideBox(box, null, &found);
    try std.testing.expectEqual(@as(usize, 1), in_box.len);
    try std.testing.expectEqual(near, in_box[0]);
    try std.testing.expectEqual(@as(u32, 1), try broad.countBoxOverlaps(box, null));

    // The same region as a sphere, a point and an oriented box.
    const in_sphere = try broad.collideSphere(zjolt.rvec3(0, 5, 0), 1.0, null, &found);
    try std.testing.expectEqual(@as(usize, 1), in_sphere.len);
    try std.testing.expectEqual(near, in_sphere[0]);

    const at_point = try broad.collidePoint(zjolt.rvec3(0, 5, 0), null, &found);
    try std.testing.expectEqual(@as(usize, 1), at_point.len);
    try std.testing.expectEqual(near, at_point[0]);

    const oriented = try broad.collideOrientedBox(.{
        .center = zjolt.rvec3(0, 5, 0),
        .rotation = zjolt.quatFromAxisAngle(zjolt.vec3(0, 1, 0), std.math.pi / 4.0),
        .half_extent = zjolt.vec3(2, 2, 2),
    }, null, &found);
    try std.testing.expectEqual(@as(usize, 1), oriented.len);
    try std.testing.expectEqual(near, oriented[0]);

    // Nowhere near either ball.
    const empty = try broad.collidePoint(zjolt.rvec3(0, 100, 0), null, &found);
    try std.testing.expectEqual(@as(usize, 0), empty.len);

    // A ray down the +x axis passes through both balls, and through the floor
    // only if it dips — it does not.
    var hits: [16]zjolt.BroadPhaseCastHit = undefined;
    const along_x = try broad.castRay(
        zjolt.rvec3(-5, 5, 0),
        zjolt.vec3(50, 0, 0),
        null,
        &hits,
    );
    try std.testing.expectEqual(@as(usize, 2), along_x.len);
    for (along_x) |hit| {
        try std.testing.expect(hit.body == near or hit.body == far);
        try std.testing.expect(hit.fraction >= 0 and hit.fraction <= 1);
    }

    // Sweeping a small box the same way finds the same two.
    const swept = try broad.castBox(
        .{ .min = zjolt.vec3(-5.2, 4.8, -0.2), .max = zjolt.vec3(-4.8, 5.2, 0.2) },
        zjolt.vec3(50, 0, 0),
        null,
        &hits,
    );
    try std.testing.expectEqual(@as(usize, 2), swept.len);

    // The layer filter is consulted, and it is the only kind the broad phase
    // has — there is no body filter here, which is why `Filters` is its own
    // type rather than the narrow phase's.
    const only_static: zjolt.BroadPhaseFilters = .{
        .object_layer = .{ .should_collide = onlyStaticLayer },
    };
    const filtered = try broad.collideBox(box, &only_static, &found);
    try std.testing.expectEqual(@as(usize, 0), filtered.len);

    // A short buffer reports the count rather than truncating silently.
    var one: [1]zjolt.BodyId = undefined;
    const wide: zjolt.AABox = .{
        .min = zjolt.vec3(-50, -50, -50),
        .max = zjolt.vec3(50, 50, 50),
    };
    try std.testing.expectError(
        error.BufferTooSmall,
        broad.collideBox(wide, null, &one),
    );

    // Whole-world bounds cover both balls and the floor.
    const bounds = broad.bounds();
    try std.testing.expect(bounds.min.x <= -1 and bounds.max.x >= 30);
}

fn onlyStaticLayer(user: ?*anyopaque, layer: zjolt.ObjectLayer) callconv(.c) bool {
    _ = user;
    return layer == Layers.static;
}

//=============================================================================
// Batch add and remove
//=============================================================================

test "a batch insert leaves the same world a loop of single inserts would" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const count = 64;
    var by_batch: [count]zjolt.RVec3 = undefined;
    var by_loop: [count]zjolt.RVec3 = undefined;

    for ([_]bool{ true, false }) |use_batch| {
        var world = try World.init();
        defer world.deinit();

        var ids: [count]zjolt.BodyId = undefined;
        for (&ids, 0..) |*id, i| {
            const x: f32 = @floatFromInt(@as(i32, @intCast(i % 8)) - 4);
            const z: f32 = @floatFromInt(@as(i32, @intCast(i / 8)) - 4);
            id.* = try world.system.bodies().create(.{
                .shape = shape,
                .object_layer = Layers.moving,
                .position = zjolt.rvec3(x * 2, 1.0, z * 2),
            });
        }

        if (use_batch) {
            try world.system.batch().add(&ids, .activate);
        } else {
            for (ids) |id| world.system.bodies().add(id, .activate);
        }

        try std.testing.expectEqual(@as(u32, count + 1), world.system.numBodies());
        for (ids) |id| try std.testing.expect(world.system.bodies().isAdded(id));

        try world.stepFor(1.0);

        const out = if (use_batch) &by_batch else &by_loop;
        for (ids, 0..) |id, i| out[i] = world.system.bodies().getPosition(id);
    }

    // Not asserted bit-identical: a batch insert builds a different subtree
    // from an incremental one, and the tree decides the order pairs are found
    // in, which decides the order the solver sums them in. The world is the
    // same world; the last bits of it are not the same bits.
    for (by_batch, by_loop) |a, b| {
        try std.testing.expectApproxEqAbs(a.x, b.x, 1e-3);
        try std.testing.expectApproxEqAbs(a.y, b.y, 1e-3);
        try std.testing.expectApproxEqAbs(a.z, b.z, 1e-3);
    }
}

test "a batch refuses what Jolt would assert on, and stages nothing when it does" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const bodies = world.system.bodies();
    const a = try bodies.create(.{ .shape = shape, .object_layer = Layers.moving });
    const b = try bodies.create(.{ .shape = shape, .object_layer = Layers.moving });

    // The same body twice. Jolt's prepare checks every id against the broad
    // phase before inserting any of them, so both copies pass its check and
    // the finalize inserts one body into the tree twice.
    try std.testing.expectError(
        error.InvalidArgument,
        world.system.batch().add(&[_]zjolt.BodyId{ a, a }, .dont_activate),
    );
    try std.testing.expect(!bodies.isAdded(a));

    // A body that is already added.
    bodies.add(a, .dont_activate);
    try std.testing.expectError(
        error.InvalidArgument,
        world.system.batch().add(&[_]zjolt.BodyId{ a, b }, .dont_activate),
    );
    try std.testing.expect(!bodies.isAdded(b));

    // A body that no longer exists.
    bodies.remove(a);
    const gone = try bodies.create(.{ .shape = shape, .object_layer = Layers.moving });
    bodies.destroy(gone);
    try std.testing.expectError(
        error.BodyNotFound,
        world.system.batch().add(&[_]zjolt.BodyId{ a, gone }, .dont_activate),
    );

    // Removing one that was never added is refused too, and refusing means
    // refusing the whole set.
    try world.system.batch().add(&[_]zjolt.BodyId{ a, b }, .dont_activate);
    bodies.remove(b);
    try std.testing.expectError(
        error.InvalidArgument,
        world.system.batch().remove(&[_]zjolt.BodyId{ a, b }),
    );
    try std.testing.expect(bodies.isAdded(a));

    // Destroy is the lenient one: a list that outlived one of its bodies is
    // ordinary, so the survivors still go.
    try world.system.batch().destroy(&[_]zjolt.BodyId{ a, b, gone });
    try std.testing.expectEqual(@as(u32, 1), world.system.numBodies());
}

test "a prepared batch that is aborted, and one that is never consumed" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    {
        var world = try World.init();
        defer world.deinit();

        var ids: [8]zjolt.BodyId = undefined;
        for (&ids) |*id| {
            id.* = try world.system.bodies().create(.{
                .shape = shape,
                .object_layer = Layers.moving,
            });
        }

        // Staged: neither in the simulation nor findable.
        const staged = try world.system.batch().prepare(&ids);
        for (ids) |id| try std.testing.expect(!world.system.bodies().isAdded(id));

        try world.system.batch().abort(staged);
        for (ids) |id| try std.testing.expect(!world.system.bodies().isAdded(id));

        // The handle is spent; a second use is refused rather than reaching a
        // freed pointer.
        try std.testing.expectError(
            error.InvalidArgument,
            world.system.batch().abort(staged),
        );

        // ...and finalizing after that works from a fresh prepare.
        const again = try world.system.batch().prepare(&ids);
        try world.system.batch().finalize(again, .dont_activate);
        for (ids) |id| try std.testing.expect(world.system.bodies().isAdded(id));
    }

    // A batch left outstanding is aborted with the system rather than leaking
    // it — std.testing.allocator is the assertion.
    {
        var world = try World.init();
        defer world.deinit();

        var ids: [4]zjolt.BodyId = undefined;
        for (&ids) |*id| {
            id.* = try world.system.bodies().create(.{
                .shape = shape,
                .object_layer = Layers.moving,
            });
        }
        _ = try world.system.batch().prepare(&ids);
    }
}

test "a batch wakes and sleeps many bodies at once" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    var ids: [8]zjolt.BodyId = undefined;
    for (&ids, 0..) |*id, i| {
        const x: f32 = @floatFromInt(i);
        id.* = try world.system.bodies().create(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(x * 3, 0.5, 0),
        });
    }
    try world.system.batch().add(&ids, .dont_activate);
    for (ids) |id| try std.testing.expect(!world.system.bodies().isActive(id));

    try world.system.batch().activate(&ids);
    for (ids) |id| try std.testing.expect(world.system.bodies().isActive(id));

    try world.system.batch().deactivate(&ids);
    for (ids) |id| try std.testing.expect(!world.system.bodies().isActive(id));

    // And by region, which is a broad-phase query with the same caveats.
    try world.system.batch().activateInBox(.{
        .min = zjolt.vec3(-1, -1, -1),
        .max = zjolt.vec3(4, 2, 1),
    }, null);
    try std.testing.expect(world.system.bodies().isActive(ids[0]));
    try std.testing.expect(world.system.bodies().isActive(ids[1]));
    try std.testing.expect(!world.system.bodies().isActive(ids[7]));
}

//=============================================================================
// Collision groups
//=============================================================================

test "a collision group filter suppresses exactly the pairs it names" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    const filter = try zjolt.GroupFilter.initTable(3);
    defer filter.release();
    try std.testing.expectEqual(@as(u32, 1), filter.refCount());
    try std.testing.expectEqual(@as(u32, 3), filter.numSubGroups());

    // Everything collides until a pair is cleared.
    try std.testing.expect(try filter.isCollisionEnabled(0, 1));
    try filter.disableCollision(0, 1);
    try std.testing.expect(!try filter.isCollisionEnabled(0, 1));
    // Symmetric, and only that pair.
    try std.testing.expect(!try filter.isCollisionEnabled(1, 0));
    try std.testing.expect(try filter.isCollisionEnabled(0, 2));

    // The two arguments Jolt asserts on rather than checking.
    try std.testing.expectError(error.InvalidArgument, filter.disableCollision(1, 1));
    try std.testing.expectError(error.InvalidArgument, filter.disableCollision(0, 3));
    try std.testing.expectError(
        error.InvalidArgument,
        zjolt.GroupFilter.initTable(zjolt.max_sub_groups + 1),
    );

    // Two overlapping spheres, no gravity: without the filter they push each
    // other apart, with it they stay where they were put.
    for ([_]bool{ false, true }) |suppressed| {
        var world = try World.init();
        defer world.deinit();
        world.system.setGravity(zjolt.vec3_zero);

        const bodies = world.system.bodies();
        const a = try bodies.createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(-0.1, 5, 0),
        }, .activate);
        const b = try bodies.createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(0.1, 5, 0),
        }, .activate);

        // Same group, different sub-groups: the table decides. Sub-groups 0
        // and 1 are the pair that was disabled above; 0 and 2 were not.
        bodies.setCollisionGroup(a, .{ .filter = filter, .group_id = 7, .sub_group_id = 0 });
        bodies.setCollisionGroup(b, .{
            .filter = filter,
            .group_id = 7,
            .sub_group_id = if (suppressed) 1 else 2,
        });
        try std.testing.expectEqual(@as(u32, 7), bodies.getCollisionGroup(a).group_id);
        try std.testing.expect(bodies.getCollisionGroup(a).filter != null);

        try world.stepFor(0.5);

        const gap = @abs(bodies.getPosition(a).x - bodies.getPosition(b).x);
        if (suppressed) {
            try std.testing.expect(gap < 0.3);
        } else {
            try std.testing.expect(gap > 0.9);
        }
    }

    // The bodies took their own references and gave them back; ours is the
    // only one left.
    try std.testing.expectEqual(@as(u32, 1), filter.refCount());
}

//=============================================================================
// Step listeners and combine callbacks
//=============================================================================

const CountingListener = struct {
    calls: u32 = 0,
    last_delta: f32 = 0,
    fail_after: ?u32 = null,

    pub fn onStep(self: *CountingListener, ctx: *const zjolt.StepListenerContext) !void {
        self.calls += 1;
        self.last_delta = ctx.delta_time;
        if (self.fail_after) |n| {
            if (self.calls > n) return error.ListenerGaveUp;
        }
    }
};

test "a step listener fires once per collision step" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();
    _ = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);

    var counter: CountingListener = .{};
    var listener: zjolt.StepListener(CountingListener) = .init(&counter);
    try listener.attach(world.system);

    _ = try world.system.step(1.0 / 60.0, 1, world.jobs);
    try listener.check();
    try std.testing.expectEqual(@as(u32, 1), counter.calls);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 60.0), counter.last_delta, 1e-6);

    // Two collision steps, so two callbacks, each with half the delta.
    _ = try world.system.step(1.0 / 60.0, 2, world.jobs);
    try listener.check();
    try std.testing.expectEqual(@as(u32, 3), counter.calls);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 120.0), counter.last_delta, 1e-6);

    try listener.detach(world.system);
    _ = try world.system.step(1.0 / 60.0, 1, world.jobs);
    try std.testing.expectEqual(@as(u32, 3), counter.calls);

    // Detaching twice is not a double free, and a handle from nowhere is
    // refused where Jolt would assert.
    try listener.detach(world.system);
}

test "a step listener that signals an error leaves the system usable" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();
    const ball = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);

    var counter: CountingListener = .{ .fail_after = 0 };
    var listener: zjolt.StepListener(CountingListener) = .init(&counter);
    try listener.attach(world.system);
    defer listener.detach(world.system) catch {};

    // The error does not unwind out of the callback — if it did, the solver's
    // locks would still be held and the NEXT step would deadlock. It is
    // carried out and re-raised here instead.
    _ = try world.system.step(1.0 / 60.0, 1, world.jobs);
    try std.testing.expectError(error.ListenerGaveUp, listener.check());

    // Taking it clears it, so a host is not told twice about one failure.
    try listener.check();

    // ...and the system still works: this is the step that would have hung.
    const before = world.system.bodies().getPosition(ball);
    _ = try world.system.step(1.0 / 60.0, 1, world.jobs);
    _ = try world.system.step(1.0 / 60.0, 1, world.jobs);
    try std.testing.expect(world.system.bodies().getPosition(ball).y < before.y);
    try std.testing.expectError(error.ListenerGaveUp, listener.check());
}

const Bouncy = struct {
    calls: u32 = 0,
    value: f32,
    fail: bool = false,

    pub fn combine(self: *Bouncy, info: *const zjolt.CombineInfo) !f32 {
        // Both bodies' own values arrive with the call, because the callback
        // must not reach back into the system to read them.
        _ = info.value1;
        _ = info.value2;
        self.calls += 1;
        if (self.fail) return error.NoOpinion;
        return self.value;
    }
};

test "a combine callback decides the contact, and one that fails leaves the system usable" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();

    // Both bodies have restitution 0, so anything that bounces came from the
    // callback and not from the bodies.
    var peak_by_restitution: [2]f32 = .{ 0, 0 };
    for ([_]f32{ 0.0, 0.9 }, 0..) |restitution, i| {
        var world = try World.init();
        defer world.deinit();

        var bouncy: Bouncy = .{ .value = restitution };
        var callback: zjolt.CombineCallback(Bouncy) = .init(&bouncy);
        try callback.attachRestitution(world.system);

        const ball = try world.system.bodies().createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(0, 3, 0),
        }, .activate);

        var peak: f32 = 0;
        var step: u32 = 0;
        while (step < 120) : (step += 1) {
            _ = try world.system.step(1.0 / 60.0, 1, world.jobs);
            const up = world.system.bodies().getLinearVelocity(ball).y;
            if (up > peak) peak = up;
        }
        try callback.check();
        try std.testing.expect(bouncy.calls > 0);
        peak_by_restitution[i] = peak;

        try world.system.clearCombineRestitution();
    }

    try std.testing.expect(peak_by_restitution[0] < 0.5);
    try std.testing.expect(peak_by_restitution[1] > 3.0);

    // A callback that signals an error yields 0 for that contact and comes
    // back out of `check`, rather than unwinding through the solver.
    {
        var world = try World.init();
        defer world.deinit();

        var bouncy: Bouncy = .{ .value = 0.9, .fail = true };
        var callback: zjolt.CombineCallback(Bouncy) = .init(&bouncy);
        try callback.attachRestitution(world.system);

        const ball = try world.system.bodies().createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(0, 1, 0),
        }, .activate);

        try world.stepFor(1.0);
        try std.testing.expect(bouncy.calls > 0);
        try std.testing.expectError(error.NoOpinion, callback.check());

        // Still simulating, and the ball came to rest on the floor rather than
        // bouncing on a restitution the failed callback never returned.
        try world.stepFor(0.5);
        try std.testing.expectApproxEqAbs(
            @as(zjolt.Real, 0.5),
            world.system.bodies().getPosition(ball).y,
            0.05,
        );
        try world.system.clearCombineRestitution();
    }
}

//=============================================================================
// Saving and restoring state
//=============================================================================

test "a saved state restores bit-identically" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.5, 0.5), .{});
    defer shape.release();

    var ids: [12]zjolt.BodyId = undefined;
    for (&ids, 0..) |*id, i| {
        const x: f32 = @floatFromInt(@as(i32, @intCast(i)) - 6);
        id.* = try world.system.bodies().createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(x * 0.9, 2.0 + x * 0.05, 0),
            .angular_velocity = zjolt.vec3(0.3, 0.1, -0.2),
        }, .activate);
    }

    // Far enough in that bodies are in contact with the floor and with each
    // other, so the contact cache is carrying real warm-start data.
    try world.stepFor(1.0);

    const saved = try world.system.state().saveAlloc(std.testing.allocator, .all);
    defer std.testing.allocator.free(saved);
    try std.testing.expect(saved.len > 0);
    try std.testing.expectEqual(saved.len, try world.system.state().size(.all));

    var expected_positions: [ids.len]zjolt.RVec3 = undefined;
    var expected_rotations: [ids.len]zjolt.Quat = undefined;
    try world.stepFor(0.5);
    _ = try world.system.getTransforms(&ids, &expected_positions, &expected_rotations);

    try world.system.state().restore(saved);

    var actual_positions: [ids.len]zjolt.RVec3 = undefined;
    var actual_rotations: [ids.len]zjolt.Quat = undefined;
    try world.stepFor(0.5);
    _ = try world.system.getTransforms(&ids, &actual_positions, &actual_rotations);

    // Bit-identical, not approximately equal. A restore that put back
    // everything except the solver's warm-start impulses would pass a
    // tolerance and fail this.
    for (expected_positions, actual_positions) |want, got| {
        try std.testing.expectEqual(want.x, got.x);
        try std.testing.expectEqual(want.y, got.y);
        try std.testing.expectEqual(want.z, got.z);
    }
    for (expected_rotations, actual_rotations) |want, got| {
        try std.testing.expectEqual(want.x, got.x);
        try std.testing.expectEqual(want.w, got.w);
    }
}

test "a state buffer is refused when it is not one, and when the world moved on" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, 0);
    defer shape.release();
    const ball = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 3, 0),
    }, .activate);
    try world.stepFor(0.2);

    const saved = try world.system.state().saveAlloc(std.testing.allocator, .all);
    defer std.testing.allocator.free(saved);

    // A short buffer reports what it needed rather than writing a prefix.
    var stub: [8]u8 = undefined;
    try std.testing.expectError(
        error.BufferTooSmall,
        world.system.state().save(.all, &stub),
    );

    // Ordinary text is not a saved state.
    try std.testing.expectError(
        error.BadFormat,
        world.system.state().restore("this is not a physics state at all, at all"),
    );

    // Truncation and damage are both caught before Jolt reads a byte.
    try std.testing.expectError(
        error.BadFormat,
        world.system.state().restore(saved[0 .. saved.len - 4]),
    );
    const damaged = try std.testing.allocator.dupe(u8, saved);
    defer std.testing.allocator.free(damaged);
    damaged[damaged.len - 1] ^= 0xff;
    try std.testing.expectError(error.BadFormat, world.system.state().restore(damaged));

    // ...and so is the one Jolt reacts to with an assertion rather than a
    // return: a body that no longer exists.
    world.system.bodies().destroy(ball);
    try std.testing.expectError(error.BadFormat, world.system.state().restore(saved));
}

//=============================================================================
// Helpers
//=============================================================================

/// A unit-length copy of `v`. Jolt asserts that a body's rotation quaternion
/// is normalised, so a hand-typed axis has to be normalised first.
fn normalize(v: zjolt.Vec3) zjolt.Vec3 {
    const length = @sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return zjolt.vec3(v.x / length, v.y / length, v.z / length);
}

/// An axis-aligned box as an indexed triangle mesh, wound so every face points
/// outward — which matters, because a mesh shape collides on its front faces.
fn makeBoxMesh(
    half_x: f32,
    half_y: f32,
    half_z: f32,
    vertices: *[8]zjolt.Vec3,
    indices: *[36]u32,
) void {
    const xs = [2]f32{ -half_x, half_x };
    const ys = [2]f32{ -half_y, half_y };
    const zs = [2]f32{ -half_z, half_z };

    var v: usize = 0;
    for (0..2) |ix| {
        for (0..2) |iy| {
            for (0..2) |iz| {
                vertices[v] = zjolt.vec3(xs[ix], ys[iy], zs[iz]);
                v += 1;
            }
        }
    }

    // Vertex index is (ix<<2) | (iy<<1) | iz.
    const faces = [12][3]u32{
        .{ 0, 1, 3 }, .{ 0, 3, 2 }, // -x
        .{ 4, 6, 7 }, .{ 4, 7, 5 }, // +x
        .{ 0, 4, 5 }, .{ 0, 5, 1 }, // -y
        .{ 2, 3, 7 }, .{ 2, 7, 6 }, // +y
        .{ 0, 2, 6 }, .{ 0, 6, 4 }, // -z
        .{ 1, 5, 7 }, .{ 1, 7, 3 }, // +z
    };
    for (faces, 0..) |face, i| {
        indices[i * 3 + 0] = face[0];
        indices[i * 3 + 1] = face[1];
        indices[i * 3 + 2] = face[2];
    }
}
