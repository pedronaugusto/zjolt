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
// Two object layers and two broad-phase layers: the smallest configuration that still exercises filtering — static-vs-static must be rejected, moving-vs-anything accepted.
//=============================================================================

pub const Layers = struct {
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
pub const World = struct {
    system: zjolt.PhysicsSystem,
    jobs: zjolt.JobSystem,
    floor_shape: zjolt.Shape,
    floor: zjolt.BodyId,

    pub fn init() !World {
        const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
        errdefer jobs.deinit();

        const system = try zjolt.PhysicsSystem.init(.{
            .layers = zjolt.layersFromType(Layers),
            .max_bodies = 1024,
        });
        errdefer system.deinit();
        system.setGravity(zjolt.gravity_earth);

        // 400 m square, not 100. A vehicle test accelerates for a few seconds
        // and then needs room to brake, and a floor that runs out mid-test
        // reads as a physics failure — the car simply drives off the end and
        // falls, with every wheel reporting no contact and nothing to brake
        // against. That is what a 100 m floor did here: one build stopped a
        // metre inside the edge and another crossed it.
        const floor_shape = try zjolt.Shape.initBox(
            zjolt.vec3(200, 0.5, 200),
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

    pub fn deinit(self: *World) void {
        self.system.deinit();
        self.floor_shape.release();
        self.jobs.deinit();
    }

    pub fn stepFor(self: *World, seconds: f32) !void {
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

    const shape = try zjolt.Shape.initSphere(1.0, .{});
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
        zjolt.Shape.initSphere(1.0, .{}),
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
    const sphere = try zjolt.Shape.initSphere(0.5, .{});
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

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
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

    const sphere = try zjolt.Shape.initSphere(0.5, .{});
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

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
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

    const mesh = try zjolt.Shape.initMesh(&cube_points, &cube_indices, .{});
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
        zjolt.Shape.initMesh(&points, &bad_indices, .{}),
    );
    try std.testing.expect(zjolt.lastError().len > 0);

    // A partial triangle.
    const short_indices = [_]u32{ 0, 1 };
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.Shape.initMesh(&points, &short_indices, .{}),
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

    const mesh = try zjolt.Shape.initMesh(&points, &indices, .{});
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
    // last of these is the case that would otherwise reach Jolt's parser and
    // index its shape-type table with whatever byte happened to be there.
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

test "a shape saved through a host stream round-trips, and matches the buffer form's payload byte for byte" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(1.25, .{});
    defer sphere.release();

    const via_buffer = try sphere.saveAlloc(std.testing.allocator);
    defer std.testing.allocator.free(via_buffer);

    var stream_buffer: [4096]u8 = undefined;
    var writer: zjolt.StreamBufferWriter = .{ .buffer = &stream_buffer };
    try sphere.saveStream(zjolt.hostStream(zjolt.StreamBufferWriter, &writer));
    const via_stream = writer.slice();

    // The two forms carry different framing on purpose — @see zjolt.Stream —
    // but underneath, Jolt's own payload has to be the exact same bytes.
    const buffer_payload = via_buffer[zjolt.c.shape.shape_header_size..];
    const stream_payload = via_stream[zjolt.c.shape.shape_stream_header_size..];
    try std.testing.expectEqualSlices(u8, buffer_payload, stream_payload);

    var reader: zjolt.StreamBufferReader = .{ .buffer = via_stream };
    const restored = try zjolt.Shape.restoreStream(zjolt.hostStream(zjolt.StreamBufferReader, &reader));
    defer restored.release();

    try std.testing.expectEqual(sphere.subType(), restored.subType());
    const before = sphere.localBounds();
    const after = restored.localBounds();
    try std.testing.expectApproxEqAbs(before.min.x, after.min.x, 1e-5);
    try std.testing.expectApproxEqAbs(before.max.x, after.max.x, 1e-5);
}

test "a truncated payload inside a well-formed container is still refused" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try zjolt.Shape.initSphere(0.5, .{});
    defer sphere.release();

    const saved = try sphere.saveAlloc(std.testing.allocator);
    defer std.testing.allocator.free(saved);

    // The container rejects a short buffer on its recorded length alone, which
    // would make a plain prefix test prove nothing about the layer underneath.
    // Rebuilding the header around each truncated payload is the only way to
    // reach Jolt's own parser reading a stream that runs out, and confirm it
    // reports the shortfall rather than improvising a shape from zeros.
    const header = zjolt.c.shape.shape_header_size;
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

/// Asserts the last failure was the one the container reports for a wrong
/// magic tag, and not one of the other checks in the same chain. The message
/// is the only thing that separates them: every one of them is `BadFormat`.
fn expectRefusedOnTag(expected: []const u8) !void {
    try std.testing.expectEqualStrings(expected, zjolt.lastError());
}

test "one save/load pair's buffer is refused by another's, on the tag" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Four save/load pairs share one framing, each with its own magic tag: the
    // fields behind it line up between pairs, so without it a scene buffer
    // would clear a shape's header checks and hand Jolt a payload it has no
    // business reading — a parse of the wrong thing, not a refusal. Asserted on
    // the MESSAGE, not just `BadFormat`: the length/checksum checks also report
    // `BadFormat`, which would not generalise as a reason to refuse.
    const box = try zjolt.Shape.initBox(zjolt.vec3(0.25, 0.5, 0.75), .{});
    defer box.release();

    var world = try World.init();
    defer world.deinit();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = box,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);
    try world.stepFor(0.25);

    const shape_blob = try box.saveAlloc(std.testing.allocator);
    defer std.testing.allocator.free(shape_blob);

    const scene_blob = blk: {
        const scene = try zjolt.Scene.init();
        defer scene.release();
        _ = try scene.addBody(.{
            .shape = box,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(1, 2, 3),
        });
        break :blk try scene.saveAlloc(std.testing.allocator);
    };
    defer std.testing.allocator.free(scene_blob);

    const state = world.system.state();
    const state_blob = try state.saveAlloc(std.testing.allocator, .{});
    defer std.testing.allocator.free(state_blob);

    const body_blob = blk: {
        var lock = world.system.lockRead(ball);
        defer lock.release();
        const body = lock.body() orelse return error.TestUnexpectedResult;
        break :blk try state.saveBodyStateAlloc(std.testing.allocator, body);
    };
    defer std.testing.allocator.free(body_blob);

    // A shape is not a scene, and a scene is not a shape.
    try std.testing.expectError(
        zjolt.Error.BadFormat,
        zjolt.Scene.restore(shape_blob),
    );
    try expectRefusedOnTag("not a scene saved by zjoltSceneSave");

    try std.testing.expectError(
        zjolt.Error.BadFormat,
        zjolt.Shape.restore(scene_blob),
    );
    try expectRefusedOnTag("not a shape saved by zjoltShapeSave");

    // A whole-world state is not one body's, and the position afterwards is
    // what says Jolt never started reading: a payload it half-consumed would
    // leave the ball somewhere else.
    const before = bodies.getPosition(ball);
    {
        var lock = world.system.lockWrite(ball);
        defer lock.release();
        const body = lock.body() orelse return error.TestUnexpectedResult;
        try std.testing.expectError(
            zjolt.Error.BadFormat,
            state.restoreBodyState(body, state_blob),
        );
    }
    try expectRefusedOnTag(
        "not a body state saved by zjoltPhysicsSystemSaveBodyStateLocked",
    );

    try std.testing.expectError(
        zjolt.Error.BadFormat,
        state.restore(body_blob, .{}),
    );
    try expectRefusedOnTag("not a state saved by zjoltPhysicsSystemSaveState");

    const after = bodies.getPosition(ball);
    try std.testing.expectEqual(before.x, after.x);
    try std.testing.expectEqual(before.y, after.y);
    try std.testing.expectEqual(before.z, after.z);

    // ...and each buffer still loads through the pair that wrote it.
    const shape_again = try zjolt.Shape.restore(shape_blob);
    shape_again.release();
    const scene_again = try zjolt.Scene.restore(scene_blob);
    scene_again.release();
    try state.restore(state_blob, .{});
    {
        var lock = world.system.lockWrite(ball);
        defer lock.release();
        const body = lock.body() orelse return error.TestUnexpectedResult;
        try state.restoreBodyState(body, body_blob);
    }
}

//=============================================================================
// Simulation
//=============================================================================

test "a dynamic body falls under gravity and comes to rest on the floor" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
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

test "the transform matrices place the body, its centre of mass, and its inertia" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const sphere = try zjolt.Shape.initSphere(0.5, .{});
    defer sphere.release();

    // A centre of mass a metre up the shape's own +Y, so the two transforms
    // below cannot come out equal by accident.
    const offset = try zjolt.Shape.initOffsetCenterOfMass(sphere, zjolt.vec3(0, 1, 0));
    defer offset.release();

    const bodies = world.system.bodies();
    const quarter_turn = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 1, 0), std.math.pi / 2.0);
    const ball = try bodies.createAndAdd(.{
        .shape = offset,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(2, 6, -3),
        .rotation = quarter_turn,
    }, .dont_activate);

    // Column-major: the translation is the fourth column, elements 12..15.
    const transform = bodies.getWorldTransform(ball);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 2), transform.m[12], 1e-4);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 6), transform.m[13], 1e-4);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, -3), transform.m[14], 1e-4);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 1), transform.m[15], 1e-4);

    // A quarter turn about +Y sends the x axis to -z, so column 0 — elements
    // 0..3 — is (0, 0, -1, 0). Reading it row-major would find 0 here and the
    // translation somewhere else entirely.
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0), transform.m[0], 1e-4);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0), transform.m[1], 1e-4);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, -1), transform.m[2], 1e-4);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 0), transform.m[3], 1e-4);

    // The centre-of-mass transform agrees with the accessor that reports the
    // same point as a position, and sits the offset above the shape's origin.
    const com_transform = bodies.getCenterOfMassTransform(ball);
    const com = bodies.getCenterOfMassPosition(ball);
    try std.testing.expectApproxEqAbs(com.x, com_transform.m[12], 1e-4);
    try std.testing.expectApproxEqAbs(com.y, com_transform.m[13], 1e-4);
    try std.testing.expectApproxEqAbs(com.z, com_transform.m[14], 1e-4);
    try std.testing.expectApproxEqAbs(
        transform.m[13] + 1,
        com_transform.m[13],
        1e-4,
    );

    // A plain sphere, whose inertia is the same about every axis, so its
    // inverse is a scaled identity whatever the body's rotation. The offset
    // centre of mass above would not be: displacing the mass adds a
    // parallel-axis term to two of the three axes.
    const plain = try bodies.createAndAdd(.{
        .shape = sphere,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(-4, 6, 0),
        .rotation = quarter_turn,
    }, .dont_activate);

    const inverse_inertia = try bodies.getInverseInertia(plain);
    try std.testing.expect(inverse_inertia.m[0] > 0);
    try std.testing.expectApproxEqRel(inverse_inertia.m[0], inverse_inertia.m[5], 1e-3);
    try std.testing.expectApproxEqRel(inverse_inertia.m[0], inverse_inertia.m[10], 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0), inverse_inertia.m[1], 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0), inverse_inertia.m[4], 1e-4);

    // Only a dynamic body has one, and only a live id names a body.
    try std.testing.expectError(
        error.InvalidArgument,
        bodies.getInverseInertia(world.floor),
    );
    try std.testing.expectError(
        error.BodyNotFound,
        bodies.getInverseInertia(zjolt.invalid_body_id),
    );
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
                .rotation = try zjolt.Quat.fromAxisAngle(normalize(zjolt.vec3(1, 1, 1)), 0.7),
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

    const shape = try zjolt.Shape.initSphere(0.5, .{});
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

test "a body created with a custom id carries exactly that id" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const desc = zjolt.BodyDesc{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    };

    // The id deterministic lockstep networking wants a body to carry
    // identically on every peer, not the next one Jolt happens to pick.
    const chosen: zjolt.BodyId = 42;
    const id = try bodies.createWithId(desc, chosen);
    try std.testing.expectEqual(chosen, id);
    try std.testing.expect(!bodies.isAdded(id));

    // The same id a second time is refused rather than silently colliding
    // with the first body's slot.
    try std.testing.expectError(zjolt.Error.InvalidArgument, bodies.createWithId(desc, chosen));

    // Bit 31 is reserved for the broad phase; Jolt's own BodyID constructor
    // asserts on it rather than reporting it, so this is the one input this
    // entry point has to catch itself before ever building one.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.createWithId(desc, chosen | 0x8000_0000),
    );
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.createWithId(desc, zjolt.invalid_body_id),
    );

    // createAndAddWithId does the same and also adds it.
    const second = try bodies.createAndAddWithId(desc, 43, .activate);
    try std.testing.expectEqual(@as(zjolt.BodyId, 43), second);
    try std.testing.expect(bodies.isAdded(second));

    bodies.add(id, .activate);
    try world.stepFor(0.5);
    try std.testing.expect(bodies.getPosition(id).y < 5);
}

test "allow_sleeping can be changed after creation, and a static body ignores it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 1, 0),
        .allow_sleeping = true,
    }, .activate);
    try std.testing.expect(bodies.getAllowSleeping(ball));

    bodies.setAllowSleeping(ball, false);
    try std.testing.expect(!bodies.getAllowSleeping(ball));

    bodies.setAllowSleeping(ball, true);
    try std.testing.expect(bodies.getAllowSleeping(ball));

    // A static body has no motion properties to hold this. Jolt's own
    // Body::GetAllowSleeping would dereference one unconditionally; this
    // reports Jolt's construction-time default instead, and the setter is a
    // quiet no-op rather than a crash.
    bodies.setAllowSleeping(world.floor, false);
    try std.testing.expect(bodies.getAllowSleeping(world.floor));
}

test "linear and angular damping read back what was set, and refuse a negative value" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .gravity_factor = 0,
        .linear_damping = 0.05,
        .angular_damping = 0.05,
    }, .activate);

    try std.testing.expectApproxEqAbs(@as(f32, 0.05), bodies.getLinearDamping(ball), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.05), bodies.getAngularDamping(ball), 1e-6);

    try bodies.setLinearDamping(ball, 2.0);
    try bodies.setAngularDamping(ball, 3.0);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), bodies.getLinearDamping(ball), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), bodies.getAngularDamping(ball), 1e-6);

    try std.testing.expectError(zjolt.Error.InvalidArgument, bodies.setLinearDamping(ball, -1.0));
    try std.testing.expectError(zjolt.Error.InvalidArgument, bodies.setAngularDamping(ball, -1.0));

    // Heavier damping (c = 2.0) bleeds off a push far faster than the
    // default (c = 0.05) would: dv/dt = -c * v decays to under a fifth of
    // its start within one second at this rate, well past what rounding
    // could explain.
    bodies.setLinearVelocity(ball, zjolt.vec3(0, 0, 10));
    try world.stepFor(1.0);
    try std.testing.expect(bodies.getLinearVelocity(ball).z < 2.0);
}

test "an impulse changes velocity immediately and a force does not" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
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

test "a force and a torque applied together both survive to the next step" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .gravity_factor = 0,
    }, .activate);

    bodies.addForceAndTorque(ball, zjolt.vec3(0, 0, 100), zjolt.vec3(0, 50, 0));
    // Both accumulate until the step, exactly as the separate calls do.
    try std.testing.expectApproxEqAbs(@as(f32, 0), bodies.getLinearVelocity(ball).z, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), bodies.getAngularVelocity(ball).y, 1e-6);

    _ = try world.system.step(1.0 / 60.0, 1, world.jobs);

    // Both took effect from the ONE call — not just whichever a caller would
    // notice first if the two had silently been dropped to one.
    try std.testing.expect(bodies.getLinearVelocity(ball).z > 0);
    try std.testing.expect(bodies.getAngularVelocity(ball).y > 0);
}

test "a body constrained to a plane stays in it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
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

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
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

    const shape = try zjolt.Shape.initSphere(0.5, .{});
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
    // A host that ignores the returned flags gets bodies sinking into each
    // other with no diagnostic.
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

    const shape = try zjolt.Shape.initSphere(0.5, .{});
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
    // Its contacts were retired when it slept, so the destroy-while-awake
    // case below needs the restless ball instead.
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

    const shape = try zjolt.Shape.initSphere(0.5, .{});
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
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(ball, hit.body);
    try std.testing.expect(hit.fraction > 0 and hit.fraction < 1);
    // The normal comes back resolved: the top of a sphere hit from above
    // points back up the ray.
    try std.testing.expect(hit.normal.y > 0.9);

    const total = try queries.countRayHits(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    );
    try std.testing.expect(total >= 2);

    var buffer: [16]zjolt.RayCastHit = undefined;
    const all = try queries.castRayAll(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
        &buffer,
    );
    try std.testing.expectEqual(total, @as(u32, @intCast(all.len)));

    // A buffer too small reports what was needed.
    var one: [1]zjolt.RayCastHit = undefined;
    try std.testing.expectError(zjolt.Error.BufferTooSmall, queries.castRayAll(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
        &one,
    ));

    // Excluding the ball must leave only the floor.
    const exclude: zjolt.ExcludeBody = .{ .body = ball };
    const filters = exclude.filters();
    const floor_hit = (try queries.castRayClosest(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        &filters,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(world.floor, floor_hit.body);

    // Far from anything.
    try std.testing.expect((try queries.castRayClosest(zjolt.rvec3(1000, 1000, 1000), zjolt.vec3(0, -1, 0), null, null)) == null);

    // The point tests: inside the ball, and in open air above it.
    try std.testing.expect((try queries.countPointHits(zjolt.rvec3(0, 2, 0), null)) >= 1);
    try std.testing.expectEqual(
        @as(u32, 0),
        try queries.countPointHits(zjolt.rvec3(0, 30, 0), null),
    );

    var inside: [4]zjolt.CollidePointHit = undefined;
    const containing = try queries.collidePoint(zjolt.rvec3(0, 2, 0), null, &inside);
    try std.testing.expectEqual(@as(usize, 1), containing.len);
    try std.testing.expectEqual(ball, containing[0].body);

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
// Streaming queries
//
// A fixture with several things in a line: everything below is about what happens BETWEEN hits — the order they arrive in, stopping part way, what a failing callback leaves behind.
//=============================================================================

/// Four static spheres up the Y axis at y = 2, 4, 6, 8, above the fixture's
/// floor. One ray straight down passes through all of them and then the floor,
/// so a query has five hits at five distinct distances.
const Stack = struct {
    world: World,
    shape: zjolt.Shape,
    spheres: [4]zjolt.BodyId,

    const ray_origin = zjolt.rvec3(0, 10, 0);
    const ray_direction = zjolt.vec3(0, -20, 0);
    const total_hits = 5;

    fn init() !Stack {
        var world = try World.init();
        errdefer world.deinit();

        const shape = try zjolt.Shape.initSphere(0.5, .{});
        errdefer shape.release();

        var spheres: [4]zjolt.BodyId = undefined;
        for (&spheres, 0..) |*id, i| {
            id.* = try world.system.bodies().createAndAdd(.{
                .shape = shape,
                .object_layer = Layers.static,
                .motion_type = .static,
                .position = zjolt.rvec3(0, @floatFromInt(2 * (i + 1)), 0),
            }, .dont_activate);
        }
        world.system.optimizeBroadPhase();

        return .{ .world = world, .shape = shape, .spheres = spheres };
    }

    fn deinit(self: *Stack) void {
        self.world.deinit();
        self.shape.release();
    }

    fn queries(self: *const Stack) zjolt.Queries {
        return self.world.system.queries();
    }
};

/// Records everything it is shown and asks for nothing.
const CollectHits = struct {
    bodies: [16]zjolt.BodyId = undefined,
    fractions: [16]f32 = undefined,
    count: usize = 0,
    action: zjolt.HitAction = .@"continue",

    pub fn onHit(self: *CollectHits, hit: zjolt.RayCastHit) zjolt.HitAction {
        if (self.count < self.bodies.len) {
            self.bodies[self.count] = hit.body;
            self.fractions[self.count] = hit.fraction;
        }
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

test "a sweep starting inside a mesh reports a hit only when back faces count" {
    // What the settings parameter is for. A mesh triangle met from the inside
    // is a back face, and Jolt's default ignores those, so a sweep beginning
    // inside geometry reports NOTHING, reading exactly like a clear placement —
    // the opposite of one. The two casts below differ in exactly one field —
    // the whole answer. Both the system-level and per-shape queries are
    // checked: each translates the same settings struct through separate code.
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // A closed box of triangles, wound so its normals face outward, centred
    // well clear of the fixture's floor.
    const box_centre = zjolt.rvec3(0, 10, 0);
    var points: [8]zjolt.Vec3 = undefined;
    var indices: [36]u32 = undefined;
    makeBoxMesh(1, 1, 1, &points, &indices);

    const mesh = try zjolt.Shape.initMesh(&points, &indices, .{});
    defer mesh.release();

    _ = try world.system.bodies().createAndAdd(.{
        .shape = mesh,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = box_centre,
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const probe_radius = 0.1;
    const probe = try zjolt.Shape.initSphere(probe_radius, .{});
    defer probe.release();

    // From the dead centre of the box, straight out through the +z wall.
    const travel = 4.0;
    const from_inside: zjolt.Queries.ShapeCast = .{
        .shape = probe,
        .position = box_centre,
        .direction = zjolt.vec3(0, 0, travel),
    };

    // Jolt's defaults: the only triangles on the way out are back faces, so
    // there is nothing to report.
    try std.testing.expectEqual(
        @as(?zjolt.ShapeCastHit, null),
        try world.system.queries().castShapeClosest(from_inside, null),
    );

    // The same sweep, with back-facing triangles collided with.
    var with_back_faces = from_inside;
    with_back_faces.settings = .{ .back_face_mode_triangles = .collide };

    const hit = (try world.system.queries().castShapeClosest(
        with_back_faces,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expect(hit.is_back_face_hit);

    // The sphere's centre travels one metre to the wall less its own radius,
    // out of the four the sweep was given.
    try std.testing.expectApproxEqAbs(
        @as(f32, (1.0 - probe_radius) / travel),
        hit.fraction,
        1.0e-2,
    );

    // And the per-shape queries, which reach Jolt by a different route with a
    // second copy of the same translation.
    const placed = try zjolt.TransformedShape.init(mesh, box_centre, .{});
    defer placed.deinit();

    const alone: zjolt.TransformedShape.ShapeCast = .{
        .shape = probe,
        .position = box_centre,
        .direction = zjolt.vec3(0, 0, travel),
    };
    try std.testing.expectEqual(
        @as(?zjolt.ShapeCastHit, null),
        try placed.castShapeClosest(alone, null),
    );

    var alone_with_back_faces = alone;
    alone_with_back_faces.settings = .{ .back_face_mode_triangles = .collide };

    const alone_hit = (try placed.castShapeClosest(
        alone_with_back_faces,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectApproxEqAbs(
        hit.fraction,
        alone_hit.fraction,
        1.0e-3,
    );
}

test "a streaming query visits exactly the hits a fill query collects" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var stack = try Stack.init();
    defer stack.deinit();

    var buffer: [16]zjolt.RayCastHit = undefined;
    const filled = try stack.queries().castRayAll(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
        &buffer,
    );
    try std.testing.expectEqual(@as(usize, Stack.total_hits), filled.len);

    var streamed: CollectHits = .{};
    try stack.queries().castRayEach(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
        &streamed,
    );

    // The same hits, as a set. Not as a sequence: neither form promises an
    // order unless the callback narrows, and asserting one would be asserting
    // something Jolt does not owe.
    try std.testing.expectEqual(filled.len, streamed.count);
    for (filled) |hit| {
        try std.testing.expect(containsBody(streamed.bodies[0..streamed.count], hit.body));
    }
    for (streamed.bodies[0..streamed.count]) |id| {
        var found = false;
        for (filled) |hit| {
            if (hit.body == id) found = true;
        }
        try std.testing.expect(found);
    }
}

test "stopping from a hit callback ends the traversal" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var stack = try Stack.init();
    defer stack.deinit();

    var once: CollectHits = .{ .action = .stop };
    try stack.queries().castRayEach(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
        &once,
    );
    try std.testing.expectEqual(@as(usize, 1), once.count);
}

test "narrowing on every hit puts the hits in order, best last" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var stack = try Stack.init();
    defer stack.deinit();

    var narrowed: CollectHits = .{ .action = .narrow };
    try stack.queries().castRayEach(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
        &narrowed,
    );

    // Narrowing tells Jolt not to bother with anything worse than the hit it
    // has just been given: every further hit is strictly nearer than the last,
    // and uninteresting ones are never computed at all. The pruning effect
    // mostly swallows the ordering one, worth knowing before reaching for this
    // — the broad phase already walks roughly front to back, so this fixture's
    // five unnarrowed hits become ONE narrowed.
    try std.testing.expect(narrowed.count >= 1);
    try std.testing.expect(narrowed.count < Stack.total_hits);
    for (1..narrowed.count) |i| {
        try std.testing.expect(narrowed.fractions[i] < narrowed.fractions[i - 1]);
    }

    const closest = (try stack.queries().castRayClosest(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(
        narrowed.fractions[narrowed.count - 1],
        closest.fraction,
    );
}

test "the closest hit is the nearest of all of them" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var stack = try Stack.init();
    defer stack.deinit();

    // Closest is a sink over the same traversal as the other two forms, so
    // this is really asking whether that reimplementation still agrees with
    // the exhaustive answer.
    var buffer: [16]zjolt.RayCastHit = undefined;
    const all = try stack.queries().castRayAll(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
        &buffer,
    );
    var nearest = all[0];
    for (all[1..]) |hit| {
        if (hit.fraction < nearest.fraction) nearest = hit;
    }

    const closest = (try stack.queries().castRayClosest(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(nearest.body, closest.body);
    try std.testing.expectEqual(nearest.fraction, closest.fraction);
    try std.testing.expectEqual(stack.spheres[3], closest.body);
}

/// Rejects one body at the shape level, and records what it was asked.
const RejectShapesOf = struct {
    body: zjolt.BodyId,
    calls: usize = 0,
    all_sub_shapes_empty: bool = true,

    fn shouldCollide(
        user: ?*anyopaque,
        body: zjolt.BodyId,
        sub_shape_id: zjolt.SubShapeId,
        query_sub_shape_id: zjolt.SubShapeId,
    ) callconv(.c) bool {
        const self: *RejectShapesOf = @ptrCast(@alignCast(user.?));
        self.calls += 1;
        if (sub_shape_id != zjolt.empty_sub_shape_id) self.all_sub_shapes_empty = false;
        if (query_sub_shape_id != zjolt.empty_sub_shape_id) self.all_sub_shapes_empty = false;
        return body != self.body;
    }

    fn filters(self: *RejectShapesOf) zjolt.QueryFilters {
        return .{ .shape = .{ .should_collide = shouldCollide, .user = @ptrCast(self) } };
    }
};

test "a shape filter suppresses exactly the shapes it rejects" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var stack = try Stack.init();
    defer stack.deinit();

    var reject: RejectShapesOf = .{ .body = stack.spheres[3] };
    const filters = reject.filters();

    var buffer: [16]zjolt.RayCastHit = undefined;
    const kept = try stack.queries().castRayAll(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        &filters,
        &buffer,
    );

    try std.testing.expect(reject.calls > 0);

    // Exactly the one shape named is gone, and everything else survived.
    try std.testing.expectEqual(@as(usize, Stack.total_hits - 1), kept.len);
    var kept_bodies: [16]zjolt.BodyId = undefined;
    for (kept, 0..) |hit, i| kept_bodies[i] = hit.body;
    try std.testing.expect(!containsBody(kept_bodies[0..kept.len], stack.spheres[3]));
    for (stack.spheres[0..3]) |id| {
        try std.testing.expect(containsBody(kept_bodies[0..kept.len], id));
    }
    try std.testing.expect(containsBody(kept_bodies[0..kept.len], stack.world.floor));

    // Jolt asks this filter once per shape, and none of the shapes here has
    // children — so both ids are the empty one every time. That is worth
    // pinning down: it is the difference between this filter and a body
    // filter, and it only appears when compound shapes do.
    try std.testing.expect(reject.all_sub_shapes_empty);
}

/// Fails part way through, which is the thing a Zig callback may not simply
/// do: it cannot return an error into Jolt's traversal. The wrapper stashes it
/// and re-raises after every lock has been released.
const FailOnSecondHit = struct {
    seen: usize = 0,

    pub fn onHit(self: *FailOnSecondHit, _: zjolt.RayCastHit) !zjolt.HitAction {
        self.seen += 1;
        if (self.seen == 2) return error.HitRejected;
        return .@"continue";
    }
};

const FailOnFirstOverlap = struct {
    seen: usize = 0,

    pub fn onHit(self: *FailOnFirstOverlap, _: zjolt.CollideShapeHit) !zjolt.HitAction {
        self.seen += 1;
        return error.OverlapRejected;
    }
};

test "an error out of a hit callback leaves the system usable" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var stack = try Stack.init();
    defer stack.deinit();

    // A failure inside the traversal. What must NOT happen is the error
    // travelling out through Jolt: the callback runs under the broad phase's
    // read lock, and a lock leaked there does not fail anything — it hangs the
    // next step, minutes later and nowhere near the cause. So if this
    // regresses the symptom is a hung test run, not a red assertion, which is
    // exactly why the step below is part of the test.
    var failing: FailOnSecondHit = .{};
    try std.testing.expectError(error.HitRejected, stack.queries().castRayEach(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
        &failing,
    ));
    try std.testing.expectEqual(@as(usize, 2), failing.seen);

    // The same through an overlap query, which takes the same locks by a
    // different route.
    var overlap_failure: FailOnFirstOverlap = .{};
    try std.testing.expectError(
        error.OverlapRejected,
        stack.queries().collideShapeEach(.{
            .shape = stack.shape,
            .position = zjolt.rvec3(0, 4, 0),
            .settings = .{ .max_separation_distance = 4 },
        }, null, &overlap_failure),
    );
    try std.testing.expect(overlap_failure.seen > 0);

    // The assertion: everything still works. The step is what would hang on a
    // leaked broad-phase lock, and the queries are what would be wrong if the
    // collector had been left in a half-finished state.
    try stack.world.stepFor(0.2);

    var buffer: [16]zjolt.RayCastHit = undefined;
    const all = try stack.queries().castRayAll(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
        &buffer,
    );
    try std.testing.expectEqual(@as(usize, Stack.total_hits), all.len);

    var again: CollectHits = .{};
    try stack.queries().castRayEach(
        Stack.ray_origin,
        Stack.ray_direction,
        null,
        null,
        &again,
    );
    try std.testing.expectEqual(@as(usize, Stack.total_hits), again.count);
}

//=============================================================================
// Bulk read-back and locks
//=============================================================================

test "bulk read-back matches the per-body accessors" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
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

    const shape = try zjolt.Shape.initSphere(0.5, .{});
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

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
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
    const crouched = try zjolt.Shape.initCapsule(0.2, 0.3, .{});
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

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
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
        null,
    )) orelse return error.TestUnexpectedResult;
    // Without the inner body this ray would have found the floor.
    try std.testing.expectEqual(inner, hit.body);
}

//=============================================================================
// The rest of the shape catalogue
//=============================================================================

test "the remaining shape kinds build and report themselves" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const triangle = try zjolt.Shape.initTriangle(
        zjolt.vec3(0, 0, 0),
        zjolt.vec3(0, 0, 1),
        zjolt.vec3(1, 0, 1),
        .{},
    );
    defer triangle.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.triangle, triangle.subType());

    const cylinder = try zjolt.Shape.initCylinder(1.0, 0.5, .{});
    defer cylinder.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.cylinder, cylinder.subType());
    // pi * r^2 * h, with the convex radius rounding the rim slightly.
    try std.testing.expectApproxEqAbs(
        @as(f32, std.math.pi * 0.25 * 2.0),
        cylinder.volume(),
        0.05,
    );

    const tapered_capsule = try zjolt.Shape.initTaperedCapsule(0.5, 0.2, 0.4, .{});
    defer tapered_capsule.release();
    try std.testing.expectEqual(
        zjolt.ShapeSubType.tapered_capsule,
        tapered_capsule.subType(),
    );

    // Jolt simplifies as it builds, and the surprise is worth pinning rather
    // than merely documenting: a tapered capsule whose larger sphere swallows
    // the smaller one is not a tapered capsule at all. Equal radii do the same
    // to a tapered cylinder.
    const swallowed = try zjolt.Shape.initTaperedCapsule(0.05, 0.1, 1.0, .{});
    defer swallowed.release();
    try std.testing.expect(swallowed.subType() != .tapered_capsule);

    const tapered_cylinder = try zjolt.Shape.initTaperedCylinder(0.5, 0.2, 0.4, .{});
    defer tapered_cylinder.release();
    try std.testing.expectEqual(
        zjolt.ShapeSubType.tapered_cylinder,
        tapered_cylinder.subType(),
    );

    const untapered = try zjolt.Shape.initTaperedCylinder(0.5, 0.4, 0.4, .{});
    defer untapered.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.cylinder, untapered.subType());

    const plane = try zjolt.Shape.initPlane(zjolt.vec3(0, 1, 0), 0, .{ .half_extent = 20 });
    defer plane.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.plane, plane.subType());

    // Authored data, so a normal that is not unit length is refused rather
    // than normalised — the opposite of what happens to a body's rotation.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.Shape.initPlane(zjolt.vec3(0, 3, 0), 0, .{}),
    );

    const empty = try zjolt.Shape.initEmpty(zjolt.vec3(0, 1, 0));
    defer empty.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.empty, empty.subType());
    try std.testing.expectEqual(@as(f32, 0), empty.volume());
    try std.testing.expectApproxEqAbs(@as(f32, 1), empty.centerOfMass().y, 1e-5);

    var samples = [_]f32{0} ** 16;
    const field = try zjolt.Shape.initHeightField(&samples, 4, .{});
    defer field.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.height_field, field.subType());

    const box = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.5, 0.5), .{});
    defer box.release();

    const children = [_]zjolt.CompoundChild{
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(-1, 0, 0), .user_data = 11 }),
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(1, 0, 0), .user_data = 22 }),
    };

    const static_compound = try zjolt.Shape.initStaticCompound(&children);
    defer static_compound.release();
    try std.testing.expectEqual(
        zjolt.ShapeSubType.static_compound,
        static_compound.subType(),
    );
    try std.testing.expectEqual(@as(u32, 2), static_compound.compoundChildCount());
    try std.testing.expectEqual(@as(u32, 22), static_compound.compoundChildUserData(1));

    // The other simplification: a static compound of exactly one unmoved child
    // is that child, and one that is moved is a rotated-translated shape.
    const one_child = try zjolt.Shape.initStaticCompound(&.{zjolt.compoundChild(box, .{})});
    defer one_child.release();
    try std.testing.expectEqual(zjolt.ShapeSubType.box, one_child.subType());

    const moved_child = try zjolt.Shape.initStaticCompound(
        &.{zjolt.compoundChild(box, .{ .position = zjolt.vec3(0, 2, 0) })},
    );
    defer moved_child.release();
    try std.testing.expectEqual(
        zjolt.ShapeSubType.rotated_translated,
        moved_child.subType(),
    );

    // A mutable compound never simplifies, because the child count is not
    // final.
    const mutable = try zjolt.MutableCompound.init(&children);
    defer mutable.release();
    try std.testing.expectEqual(
        zjolt.ShapeSubType.mutable_compound,
        mutable.asShape().subType(),
    );

    // A compound cannot encode "no children", so no constructor accepts it.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.Shape.initStaticCompound(&.{}),
    );
    // And a MUTABLE compound cannot go to one either: it does not simplify,
    // so a single child would leave Jolt computing CountLeadingZeros(0) — a
    // bare __builtin_clz on ARM, where zero is undefined. Refused at both
    // ends, so the shape can never be in that state.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.MutableCompound.init(&.{zjolt.compoundChild(box, .{})}),
    );
    try std.testing.expectError(zjolt.Error.InvalidArgument, mutable.removeChild(0));

    // And the mutators are for mutable compounds only, which is a returned
    // error rather than a bad cast because there is no RTTI to ask.
    const not_a_compound = zjolt.MutableCompound{ .handle = @constCast(box.handle) };
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        not_a_compound.adjustCenterOfMass(),
    );
    // Jolt's own RemoveShape and ModifyShape index their array with neither a
    // bounds check nor an assertion, so the range check is this side's job.
    try std.testing.expectError(zjolt.Error.InvalidArgument, mutable.removeChild(7));
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        mutable.moveChild(7, zjolt.vec3_zero, zjolt.quat_identity),
    );
}

test "a plane holds a body up" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // A half space five metres above the fixture's floor. If it does not
    // collide, the ball carries on down to the floor at y = 0.5 and the
    // difference is unmistakable.
    const plane = try zjolt.Shape.initPlane(zjolt.vec3(0, 1, 0), 0, .{ .half_extent = 20 });
    defer plane.release();

    const bodies = world.system.bodies();
    _ = try bodies.createAndAdd(.{
        .shape = plane,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    const ball_shape = try zjolt.Shape.initSphere(0.5, .{});
    defer ball_shape.release();
    const ball = try bodies.createAndAdd(.{
        .shape = ball_shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 8, 0),
    }, .activate);

    world.system.optimizeBroadPhase();
    try world.stepFor(2.0);

    const y = bodies.getTransform(ball).position.y;
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 5.5), y, 0.05);
}

test "a height field's surface is where its samples say, and a hole is a hole" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // 4x4 samples is the smallest field the default block size allows: Jolt
    // wants at least two blocks along each axis.
    var samples = [_]f32{0} ** 16;
    // One sample of the sentinel removes the one quad it is a corner of.
    samples[0] = zjolt.height_field_no_collision;

    const field = try zjolt.Shape.initHeightField(&samples, 4, .{});
    defer field.release();

    const bodies = world.system.bodies();
    const terrain = try bodies.createAndAdd(.{
        .shape = field,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 4, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();

    // Middle of a solid quad: the surface is exactly where the samples put it.
    const solid = (try queries.castRayClosest(
        zjolt.rvec3(2.5, 10, 2.5),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(terrain, solid.body);
    try std.testing.expectApproxEqAbs(
        @as(f32, 4.0),
        10.0 - 20.0 * solid.fraction,
        0.05,
    );

    // Middle of the hole: the ray goes straight through to the fixture's
    // floor, five metres below.
    const through = (try queries.castRayClosest(
        zjolt.rvec3(0.5, 10, 0.5),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(world.floor, through.body);
}

test "a static compound collides at each of its children and nowhere between" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(0.4, 0.4, 0.4), .{});
    defer box.release();

    const compound = try zjolt.Shape.initStaticCompound(&.{
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(-2, 0, 0) }),
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(2, 0, 0) }),
    });
    defer compound.release();

    const bodies = world.system.bodies();
    const body = try bodies.createAndAdd(.{
        .shape = compound,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 4, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();

    // Above each child, and above the gap in the middle. A single box of the
    // same bounds would answer the same for all three.
    for ([_]f32{ -2, 2 }) |x| {
        const hit = (try queries.castRayClosest(
            zjolt.rvec3(x, 10, 0),
            zjolt.vec3(0, -20, 0),
            null,
            null,
        )) orelse return error.TestUnexpectedResult;
        try std.testing.expectEqual(body, hit.body);
    }

    const between = (try queries.castRayClosest(
        zjolt.rvec3(0, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(world.floor, between.body);

    // The two children were hit at different sub-shape ids, which is the whole
    // reason a compound is not one box.
    const left = (try queries.castRayClosest(
        zjolt.rvec3(-2, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )).?;
    const right = (try queries.castRayClosest(
        zjolt.rvec3(2, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )).?;
    try std.testing.expect(left.sub_shape_id != right.sub_shape_id);
}

test "a mutable compound reflects a runtime edit on the next step" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const box = try zjolt.Shape.initBox(zjolt.vec3(1, 0.25, 1), .{});
    defer box.release();

    // Three shelves, because taking one away has to leave two: a mutable
    // compound cannot go below that. @see MutableCompound.init.
    const shelf = try zjolt.MutableCompound.init(&.{
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(-4, 0, 0) }),
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(4, 0, 0) }),
        zjolt.compoundChild(box, .{ .position = zjolt.vec3(0, 0, 8) }),
    });
    defer shelf.release();

    const bodies = world.system.bodies();
    const platform = try bodies.createAndAdd(.{
        .shape = shelf.asShape(),
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 4, 0),
    }, .dont_activate);

    const ball_shape = try zjolt.Shape.initSphere(0.5, .{});
    defer ball_shape.release();
    const ball = try bodies.createAndAdd(.{
        .shape = ball_shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(4, 6, 0),
    }, .activate);

    world.system.optimizeBroadPhase();
    try world.stepFor(2.0);

    // Resting on the right-hand shelf, four and a bit metres up.
    try std.testing.expectApproxEqAbs(
        @as(zjolt.Real, 4.75),
        bodies.getTransform(ball).position.y,
        0.05,
    );

    // Take the shelf away underneath it. The centre of mass moves, so it is
    // read before the edit and handed to notifyShapeChanged, which is also
    // what invalidates the broad phase and the contact cache — without it the
    // step would go on colliding against geometry that is no longer there.
    const previous_center_of_mass = shelf.asShape().centerOfMass();
    try shelf.removeChild(1);
    try shelf.adjustCenterOfMass();
    try std.testing.expectEqual(@as(u32, 2), shelf.childCount());
    bodies.notifyShapeChanged(platform, previous_center_of_mass, false, .activate);

    bodies.activate(ball);
    try world.stepFor(2.0);

    // Down to the floor, because there is nothing above it any more.
    try std.testing.expectApproxEqAbs(
        @as(zjolt.Real, 0.5),
        bodies.getTransform(ball).position.y,
        0.05,
    );
}

//=============================================================================
// Materials
//
// A material is an identity and nothing else — no friction, no restitution, no user data. What these assert is the only thing it is for: telling one surface apart from another inside a single shape, through the sub-shape id a hit carries.
//=============================================================================

test "a convex shape reports the material it was built with, at the empty sub-shape id" {
    AssertSink.reset();
    try zjolt.init(.{
        .allocator = std.testing.allocator,
        .assert_failed = AssertSink.onAssert,
    });
    defer zjolt.deinit();

    const gravel = try zjolt.PhysicsMaterial.init(.{
        .debug_name = "gravel",
        .debug_color = zjolt.color(160, 150, 140),
    });
    defer gravel.release();

    try std.testing.expectEqual(@as(u32, 1), gravel.refCount());
    try std.testing.expectEqualStrings("gravel", gravel.debugName());
    try std.testing.expectEqual(@as(u8, 160), gravel.debugColor().r);
    try std.testing.expectEqual(@as(u8, 255), gravel.debugColor().a);

    const sphere = try zjolt.Shape.initSphere(0.5, .{ .material = gravel });
    defer sphere.release();

    // The shape holds a reference of its own, which is what makes "create the
    // material, build the shapes, release" correct.
    try std.testing.expectEqual(@as(u32, 2), gravel.refCount());

    const found = sphere.material(zjolt.sub_shape_id_empty) orelse
        return error.TestUnexpectedResult;
    try std.testing.expect(found.eql(gravel));

    // The constant, pinned against what Jolt itself treats as empty rather
    // than against a number typed twice. ConvexShape::GetMaterial asserts the
    // id is empty, so the right value fires no assertion...
    try std.testing.expectEqual(@as(u32, 0), AssertSink.count);
    // ...and any other value does. Only observable in a build that kept Jolt's
    // assertions; without them there is no assertion to count, and without the
    // hook installed above this line would take the process down rather than
    // return.
    if (zjolt.options.enable_asserts) {
        _ = sphere.material(0);
        try std.testing.expect(AssertSink.count > 0);
        AssertSink.reset();
    }

    // A shape built without one is not a shape with no material: it reports
    // the shared default, which is a real material and not a null.
    const bare = try zjolt.Shape.initBox(zjolt.vec3(1, 1, 1), .{});
    defer bare.release();
    const default = zjolt.PhysicsMaterial.default() orelse
        return error.TestUnexpectedResult;
    const bare_material = bare.material(zjolt.sub_shape_id_empty) orelse
        return error.TestUnexpectedResult;
    try std.testing.expect(bare_material.eql(default));
    try std.testing.expect(!bare_material.eql(gravel));
    try std.testing.expectEqualStrings("Default", default.debugName());
}

test "a mesh reports the material of the triangle that was hit" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const gravel = try zjolt.PhysicsMaterial.init(.{ .debug_name = "gravel" });
    defer gravel.release();
    const metal = try zjolt.PhysicsMaterial.init(.{ .debug_name = "metal" });
    defer metal.release();

    // A flat two-triangle quad in the XZ plane, wound so both faces point up.
    // Triangle 0 covers the half where x < z, triangle 1 the half where x > z.
    const vertices = [_]zjolt.Vec3{
        zjolt.vec3(0, 0, 0),
        zjolt.vec3(2, 0, 0),
        zjolt.vec3(2, 0, 2),
        zjolt.vec3(0, 0, 2),
    };
    const indices = [_]u32{ 0, 3, 2, 0, 2, 1 };

    const mesh = try zjolt.Shape.initMesh(&vertices, &indices, .{
        .materials = &.{ gravel, metal },
        .triangle_materials = &.{ 0, 1 },
    });
    defer mesh.release();

    // Both materials are kept alive by the mesh.
    try std.testing.expectEqual(@as(u32, 2), gravel.refCount());
    try std.testing.expectEqual(@as(u32, 2), metal.refCount());

    const bodies = world.system.bodies();
    const ground = try bodies.createAndAdd(.{
        .shape = mesh,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 3, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();

    const gravel_hit = (try queries.castRayClosest(
        zjolt.rvec3(0.5, 10, 1.5),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(ground, gravel_hit.body);

    const metal_hit = (try queries.castRayClosest(
        zjolt.rvec3(1.5, 10, 0.5),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(ground, metal_hit.body);

    // The two triangles are different leaves of one shape, and the sub-shape
    // id is what tells them apart. Jolt reorders triangles spatially while it
    // builds the tree, so this is the only thing that can map a hit back to
    // the material the triangle was authored with.
    try std.testing.expect(gravel_hit.sub_shape_id != metal_hit.sub_shape_id);
    try std.testing.expect(
        (mesh.material(gravel_hit.sub_shape_id) orelse
            return error.TestUnexpectedResult).eql(gravel),
    );
    try std.testing.expect(
        (mesh.material(metal_hit.sub_shape_id) orelse
            return error.TestUnexpectedResult).eql(metal),
    );

    // The same answer through the body, which is where a contact callback
    // starts from.
    try std.testing.expect(
        (bodies.getMaterial(ground, metal_hit.sub_shape_id) orelse
            return error.TestUnexpectedResult).eql(metal),
    );

    // And the caveat the header spells out: a body id that names nothing is
    // NOT reported as a failure. Jolt's body lock fails and it answers with
    // the shared default, so this reads exactly like a shape with no materials
    // of its own.
    const default = zjolt.PhysicsMaterial.default() orelse
        return error.TestUnexpectedResult;
    const stale = bodies.getMaterial(zjolt.invalid_body_id, zjolt.sub_shape_id_empty) orelse
        return error.TestUnexpectedResult;
    try std.testing.expect(stale.eql(default));

    // Out-of-range indices are refused before Jolt sees them, with a reason.
    try std.testing.expectError(zjolt.Error.InvalidArgument, zjolt.Shape.initMesh(
        &vertices,
        &indices,
        .{ .materials = &.{gravel}, .triangle_materials = &.{ 0, 1 } },
    ));
}

test "a height field reports the material of the quad that was hit" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const grass = try zjolt.PhysicsMaterial.init(.{ .debug_name = "grass" });
    defer grass.release();
    const rock = try zjolt.PhysicsMaterial.init(.{ .debug_name = "rock" });
    defer rock.release();

    const samples = [_]f32{0} ** 16;
    // One index per quad — (4 - 1)^2 of them, not one per sample. Quad (2, 2)
    // is the odd one out.
    var quad_materials = [_]u8{0} ** 9;
    quad_materials[2 + 2 * 3] = 1;

    const field = try zjolt.Shape.initHeightField(&samples, 4, .{
        .materials = &.{ grass, rock },
        .quad_materials = &quad_materials,
    });
    defer field.release();

    const bodies = world.system.bodies();
    _ = try bodies.createAndAdd(.{
        .shape = field,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 4, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();

    const on_rock = (try queries.castRayClosest(
        zjolt.rvec3(2.5, 10, 2.5),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expect(
        (field.material(on_rock.sub_shape_id) orelse
            return error.TestUnexpectedResult).eql(rock),
    );

    const on_grass = (try queries.castRayClosest(
        zjolt.rvec3(1.5, 10, 1.5),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expect(
        (field.material(on_grass.sub_shape_id) orelse
            return error.TestUnexpectedResult).eql(grass),
    );
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

test "a compound reports which child was hit" {
    // The regression this pins: `sub_shape_id` was documented as *always*
    // empty, which was true only while no compound shape could be built. That
    // stopped being true the moment compound constructors landed, and nothing
    // failed — no test queried against a compound, so a doc comment quietly
    // became false. Adding a capability re-opens every assumption that was
    // resting on its absence.
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const left = try zjolt.Shape.initSphere(0.5, .{});
    defer left.release();
    const right = try zjolt.Shape.initSphere(0.5, .{});
    defer right.release();

    const compound = try zjolt.Shape.initStaticCompound(&.{
        .{ .shape = left.handle, .position = zjolt.vec3(-1, 0, 0) },
        .{ .shape = right.handle, .position = zjolt.vec3(1, 0, 0) },
    });
    defer compound.release();

    _ = try world.system.bodies().createAndAdd(.{
        .shape = compound,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 1, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const queries = world.system.queries();
    const hit_left = (try queries.castRayClosest(
        zjolt.rvec3(-1, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    const hit_right = (try queries.castRayClosest(
        zjolt.rvec3(1, 10, 0),
        zjolt.vec3(0, -20, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;

    // Both rays hit the same body, and the sub-shape id is what tells them
    // apart. If either were empty, the child would be unidentifiable.
    try std.testing.expectEqual(hit_left.body, hit_right.body);
    try std.testing.expect(hit_left.sub_shape_id != zjolt.c.core.sub_shape_id_empty);
    try std.testing.expect(hit_left.sub_shape_id != hit_right.sub_shape_id);
}
