//! Behavioural tests for the capabilities this file's group of gaps closed:
//! a host-supplied job system, a host-supplied step scratch allocator, and
//! the simulation shape filter. `state_test.zig` covers the selective
//! save/restore and divergence-locating pieces of the same group.
//!
//! Also covers a later pair closed the same way: `onContactValidate` now
//! carries the colliding face Jolt already collects for it, and
//! `onContactValidate`/`onContactAdded`/`onContactPersisted` all carry the
//! live body C++ gets, not a bare id.
//!
//! Reuses `integration_test.zig`'s layer map rather than building its own —
//! see `BINDING.md` — but not its `World` fixture, since that hardcodes the
//! single-threaded job system these tests replace.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const system_mod = @import("system.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;

/// A floor plus one falling sphere, without any particular job system or
/// scratch allocator — those are what each test below supplies itself.
const Rig = struct {
    system: zjolt.PhysicsSystem,
    floor_shape: zjolt.Shape,
    ball_shape: zjolt.Shape,

    fn init(opts: zjolt.PhysicsSystem.Options) !Rig {
        var full_opts = opts;
        full_opts.layers = zjolt.layersFromType(Layers);
        full_opts.max_bodies = 8;
        const system = try zjolt.PhysicsSystem.init(full_opts);
        errdefer system.deinit();
        system.setGravity(zjolt.gravity_earth);

        const floor_shape = try zjolt.Shape.initBox(zjolt.vec3(50, 0.5, 50), .{});
        errdefer floor_shape.release();
        _ = try system.bodies().createAndAdd(.{
            .shape = floor_shape,
            .object_layer = Layers.static,
            .motion_type = .static,
            .position = zjolt.rvec3(0, -0.5, 0),
        }, .dont_activate);
        system.optimizeBroadPhase();

        const ball_shape = try zjolt.Shape.initSphere(0.5, .{});
        errdefer ball_shape.release();

        return .{ .system = system, .floor_shape = floor_shape, .ball_shape = ball_shape };
    }

    fn deinit(self: *Rig) void {
        self.system.deinit();
        self.floor_shape.release();
        self.ball_shape.release();
    }

    fn dropBall(self: *Rig, start_height: f32) !zjolt.BodyId {
        return self.system.bodies().createAndAdd(.{
            .shape = self.ball_shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(0, start_height, 0),
        }, .activate);
    }
};

fn floatY(v: zjolt.RVec3) f32 {
    return @floatCast(v.y);
}

//=============================================================================
// A host job system
//=============================================================================

/// The simplest possible host scheduler: run every job synchronously, on
/// whatever thread queues it, the instant it is queued. Proves the seam end
/// to end — GetMaxConcurrency, CreateJob (exercised indirectly, since Jolt
/// calls it) and QueueJob/QueueJobs all have to cooperate correctly for a
/// step built entirely on this to produce the same physics the built-in
/// schedulers do.
const SyncJobs = struct {
    ran: usize = 0,

    pub fn getMaxConcurrency(self: *SyncJobs) u32 {
        _ = self;
        return 1;
    }
    pub fn queueJob(self: *SyncJobs, job: system_mod.JobSystem.Job) void {
        self.ran += 1;
        job.run();
        job.release();
    }
    pub fn queueJobs(self: *SyncJobs, jobs: []const system_mod.JobSystem.Job) void {
        for (jobs) |job| self.queueJob(job);
    }
};

test "a host job system settles a dropped ball exactly as the built-in schedulers do" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var rig = try Rig.init(.{ .layers = undefined });
    defer rig.deinit();

    var sync: SyncJobs = .{};
    const jobs = try zjolt.JobSystem.initHost(SyncJobs, &sync, 4);
    defer jobs.deinit();

    const ball = try rig.dropBall(5);

    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    while (elapsed < 3.0) : (elapsed += dt) {
        const update_error = try rig.system.step(dt, 1, jobs);
        try std.testing.expectEqual(zjolt.UpdateError.none, update_error);
    }

    // The queue actually ran real work — a job system that silently never
    // scheduled anything would leave the ball exactly where it started, which
    // the position check below would also catch, but this pins down why.
    try std.testing.expect(sync.ran > 0);

    const pos = rig.system.bodies().getPosition(ball);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), floatY(pos), 0.05);
}

test "JobSystem.setNumThreads resizes a thread pool created with hooks" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var starts = std.atomic.Value(u32).init(0);

    const Hooks = struct {
        fn onInit(user: ?*anyopaque, thread_index: i32) callconv(.c) void {
            _ = thread_index;
            const counter: *std.atomic.Value(u32) = @ptrCast(@alignCast(user.?));
            _ = counter.fetchAdd(1, .monotonic);
        }
    };

    const jobs = try zjolt.JobSystem.initThreadPool(.{
        .num_threads = 2,
        .thread_init = Hooks.onInit,
        .thread_hooks_user = &starts,
    });
    defer jobs.deinit();

    // `setNumThreads` stops (joining) the original threads before starting
    // the replacement, so by the time it returns every original worker has
    // certainly run its init hook to completion — asserting before this
    // point would race against thread startup instead of proving anything.
    try jobs.setNumThreads(1);
    try std.testing.expectEqual(@as(u32, 2), starts.load(.monotonic));
    try std.testing.expectEqual(@as(u32, 2), jobs.maxConcurrency());
}

//=============================================================================
// A host scratch allocator
//=============================================================================

/// Tracks every allocate/free the step makes, so the test can assert the
/// seam is actually exercised (not just accepted at create) and that usage
/// returns to zero afterwards — Jolt's own documented stack discipline.
const CountingTempAllocator = struct {
    live: usize = 0,
    peak: usize = 0,
    allocations: usize = 0,

    pub fn allocate(self: *CountingTempAllocator, size: u32) ?*anyopaque {
        // Page-aligned, well past JPH_RVECTOR_ALIGNMENT, so the test does not
        // have to track the exact SIMD alignment Jolt actually needs.
        const slice = std.heap.page_allocator.alloc(u8, size) catch return null;
        self.allocations += 1;
        self.live += size;
        if (self.live > self.peak) self.peak = self.live;
        return slice.ptr;
    }
    pub fn free(self: *CountingTempAllocator, address: ?*anyopaque, size: u32) void {
        const ptr = address orelse return;
        self.live -= size;
        const bytes: [*]u8 = @ptrCast(ptr);
        std.heap.page_allocator.free(bytes[0..size]);
    }
    pub fn getSize(self: *CountingTempAllocator) usize {
        return self.peak;
    }
    pub fn getUsage(self: *CountingTempAllocator) usize {
        return self.live;
    }
};

test "a host temp allocator backs the step, and usage returns to zero after it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var counting: CountingTempAllocator = .{};
    const host = system_mod.hostTempAllocator(CountingTempAllocator, &counting);

    var rig = try Rig.init(.{
        .layers = undefined,
        .temp_allocator_kind = .host,
        .temp_allocator = &host,
    });
    defer rig.deinit();

    _ = try rig.dropBall(3);

    const dt: f32 = 1.0 / 60.0;
    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
    defer jobs.deinit();
    var elapsed: f32 = 0;
    while (elapsed < 0.5) : (elapsed += dt) {
        _ = try rig.system.step(dt, 1, jobs);
    }

    try std.testing.expect(counting.allocations > 0);
    // Stack discipline: every allocate this step made was freed by the time
    // it returned. A leak here would mean the adapter is not matching Jolt's
    // own allocate/free pairing.
    try std.testing.expectEqual(@as(usize, 0), counting.live);

    const stats = rig.system.getTempAllocatorStats();
    try std.testing.expectEqual(counting.peak, stats.capacity);
    try std.testing.expectEqual(@as(usize, 0), stats.usage);
}

//=============================================================================
// Simulation shape filter
//=============================================================================

const BlockOne = struct {
    blocked: zjolt.BodyId,

    pub fn shouldCollide(
        self: *BlockOne,
        body1: zjolt.BodyId,
        shape1: ?*const zjolt.c.system.Shape,
        sub1: zjolt.SubShapeId,
        body2: zjolt.BodyId,
        shape2: ?*const zjolt.c.system.Shape,
        sub2: zjolt.SubShapeId,
    ) bool {
        _ = shape1;
        _ = shape2;
        _ = sub1;
        _ = sub2;
        return body1 != self.blocked and body2 != self.blocked;
    }
};

test "SimShapeFilter excludes exactly the body it targets, and nothing else" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var rig = try Rig.init(.{ .layers = undefined });
    defer rig.deinit();

    // A platform between the start height and the floor. Without the filter
    // the ball rests on it; with the filter naming it, the ball falls right
    // through to the floor beneath.
    const platform_shape = try zjolt.Shape.initBox(zjolt.vec3(2, 0.25, 2), .{});
    defer platform_shape.release();
    const platform = try rig.system.bodies().createAndAdd(.{
        .shape = platform_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 2, 0),
    }, .dont_activate);
    rig.system.optimizeBroadPhase();

    var ctx = BlockOne{ .blocked = platform };
    const filter = system_mod.simShapeFilter(BlockOne, &ctx);
    try rig.system.setSimShapeFilter(&filter);

    const installed = rig.system.getSimShapeFilter();
    try std.testing.expect(installed.should_collide != null);
    try std.testing.expect(installed.user == @as(?*anyopaque, @ptrCast(&ctx)));

    const ball = try rig.dropBall(5);

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
    defer jobs.deinit();
    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    while (elapsed < 3.0) : (elapsed += dt) {
        _ = try rig.system.step(dt, 1, jobs);
    }

    // Landed on the FLOOR (y ~= 0.5), not the platform (y ~= 2.75) — the
    // filter really did exclude the platform pair during the step, not just
    // report as installed.
    const pos = rig.system.bodies().getPosition(ball);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), floatY(pos), 0.05);

    try rig.system.setSimShapeFilter(null);
    const cleared = rig.system.getSimShapeFilter();
    try std.testing.expect(cleared.should_collide == null);
}

//=============================================================================
// Reading back what init and the setters were given
//=============================================================================

test "the layer tables, listeners, combine callback and sim shape filter all read back what was installed" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var rig = try Rig.init(.{ .layers = undefined });
    defer rig.deinit();

    // Layer tables: copied in at create, and read back byte for byte.
    const broad_phase = rig.system.getBroadPhaseLayerInterface();
    try std.testing.expect(broad_phase.num_broad_phase_layers != null);
    try std.testing.expectEqual(@as(u32, 2), broad_phase.num_broad_phase_layers.?(null));

    // No listener installed yet.
    const empty_listener = rig.system.getContactListener();
    try std.testing.expect(empty_listener.on_contact_added == null);

    const Listener = struct {
        pub fn onContactAdded(
            self: *@This(),
            info: *const system_mod.ContactInfo,
            settings: *system_mod.ContactSettings,
        ) void {
            _ = self;
            _ = info;
            _ = settings;
        }
    };
    var listener_ctx: Listener = .{};
    const listener = system_mod.contactListener(Listener, &listener_ctx);
    try rig.system.setContactListener(&listener);

    const installed_listener = rig.system.getContactListener();
    try std.testing.expect(installed_listener.on_contact_added != null);
    try std.testing.expect(installed_listener.user == @as(?*anyopaque, @ptrCast(&listener_ctx)));

    // Combine restitution: nothing installed reads back as null/null.
    const default_combine = rig.system.getCombineRestitution();
    try std.testing.expect(default_combine.combine == null);

    const Combiner = struct {
        pub fn combine(self: *@This(), info: *const system_mod.CombineInfo) f32 {
            _ = self;
            return info.value1;
        }
    };
    var combiner_ctx: Combiner = .{};
    var combiner: system_mod.CombineCallback(Combiner) = .init(&combiner_ctx);
    try combiner.attachRestitution(rig.system);

    const installed_combine = rig.system.getCombineRestitution();
    try std.testing.expect(installed_combine.combine != null);
}

//=============================================================================
// The unlocked path
//=============================================================================

test "tryGetBodyNoLock and getActiveBodiesUnsafe agree with the locked accessors" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var rig = try Rig.init(.{ .layers = undefined });
    defer rig.deinit();

    // Not a live body: nothing was created with this id.
    try std.testing.expect(rig.system.tryGetBodyNoLock(zjolt.invalid_body_id) == null);

    const ball = try rig.dropBall(4);

    const unlocked = rig.system.tryGetBodyNoLock(ball) orelse
        return error.TestUnexpectedResult;
    try std.testing.expectEqual(ball, unlocked.id());

    const locked_pos = rig.system.bodies().getPosition(ball);
    const unlocked_pos = unlocked.position();
    try std.testing.expectEqual(floatY(locked_pos), floatY(unlocked_pos));

    const active = rig.system.getActiveBodiesUnsafe();
    try std.testing.expectEqual(@as(usize, 1), active.len);
    try std.testing.expectEqual(ball, active[0]);
}

//=============================================================================
// Contact validate: the colliding face
//=============================================================================

const ValidateFaceRecorder = struct {
    validated: u32 = 0,
    face1_len: u32 = 0,
    face2_len: u32 = 0,

    pub fn onContactValidate(
        self: *@This(),
        info: *const system_mod.ContactValidateInfo,
    ) system_mod.ValidateResult {
        self.validated += 1;
        // Only the first call is pinned down: once the ball has sunk in and
        // is being pushed back out, later calls are free to see a different
        // shaped overlap. The first one is guaranteed sphere vs. box.
        if (self.validated == 1) {
            self.face1_len = @intCast(system_mod.contactValidateFace1(info).len);
            self.face2_len = @intCast(system_mod.contactValidateFace2(info).len);
        }
        return .accept_all_contacts_for_this_body_pair;
    }
};

test "OnContactValidate sees the colliding face Jolt already collected for it, not just the pair" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var rig = try Rig.init(.{ .layers = undefined });
    defer rig.deinit();

    var recorder: ValidateFaceRecorder = .{};
    const listener = system_mod.contactListener(ValidateFaceRecorder, &recorder);
    try rig.system.setContactListener(&listener);

    _ = try rig.dropBall(2);

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
    defer jobs.deinit();
    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    while (elapsed < 1.0) : (elapsed += dt) {
        _ = try rig.system.step(dt, 1, jobs);
    }

    try std.testing.expect(recorder.validated > 0);
    // The ball is a sphere: SphereShape::GetSupportingFace is a deliberate
    // no-op, so it has no face of its own to report.
    try std.testing.expectEqual(@as(u32, 0), recorder.face1_len);
    // The floor is a box: AABox::GetSupportingFace always returns exactly 4
    // vertices for the face it picked -- a wrong vertex count (0, because
    // the field was never carried across, or something other than 4) is
    // exactly what a broken carry-through would produce.
    try std.testing.expectEqual(@as(u32, 4), recorder.face2_len);
}

//=============================================================================
// Contact added/persisted: the live body
//=============================================================================

const PersistedVelocityRecorder = struct {
    ball: zjolt.BodyId,
    persisted_calls: u32 = 0,
    min_seen_vy: f32 = 0,

    pub fn onContactPersisted(
        self: *@This(),
        info: *const system_mod.ContactInfo,
        settings: *system_mod.ContactSettings,
    ) void {
        _ = settings;
        self.persisted_calls += 1;
        const ball_body = if (info.body1 == self.ball)
            system_mod.contactBody1(info)
        else
            system_mod.contactBody2(info);
        const vy = ball_body.linearVelocity().y;
        if (vy < self.min_seen_vy) self.min_seen_vy = vy;
    }
};

test "OnContactPersisted's live body reads the velocity the body has mid-step, not a stale zero" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var rig = try Rig.init(.{ .layers = undefined });
    defer rig.deinit();

    // A short drop: touching down quickly leaves most of the run settled,
    // which is the part this test cares about.
    const ball = try rig.dropBall(0.6);

    var recorder: PersistedVelocityRecorder = .{ .ball = ball };
    const listener = system_mod.contactListener(PersistedVelocityRecorder, &recorder);
    try rig.system.setContactListener(&listener);

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
    defer jobs.deinit();
    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    while (elapsed < 1.0) : (elapsed += dt) {
        _ = try rig.system.step(dt, 1, jobs);
    }

    try std.testing.expect(recorder.persisted_calls > 0);

    // JobApplyGravity hard-depends on JobFindCollisions, so this step's gravity
    // is already applied to the body by the time OnContactPersisted runs — but
    // the velocity solve that cancels it out for a resting contact hasn't run
    // yet. So a resting ball's persisted callback sees a small downward
    // velocity each step, not the ~0 it settles to. Reading a default or wrong
    // body would read back exactly 0 instead.
    try std.testing.expect(recorder.min_seen_vy < -0.01);

    const settled = rig.system.bodies().getLinearVelocity(ball);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), settled.y, 0.05);
}

//=============================================================================
// Diagnostic output
//=============================================================================

test "reportBroadphaseStats is UNSUPPORTED without -Dtrack_broadphase_stats, or succeeds when it is on" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var rig = try Rig.init(.{ .layers = undefined });
    defer rig.deinit();

    rig.system.reportBroadphaseStats() catch |e| {
        try std.testing.expectEqual(zjolt.Error.Unsupported, e);
        return;
    };
    // Reached only when built with -Dtrack_broadphase_stats=true: the call
    // above must have returned OK without erroring, which is itself the
    // assertion (nothing structured to inspect further).
}

test "reportNarrowPhaseStats is UNSUPPORTED without -Dtrack_narrowphase_stats, or succeeds when it is on" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    system_mod.reportNarrowPhaseStats() catch |e| {
        try std.testing.expectEqual(zjolt.Error.Unsupported, e);
        return;
    };
    // Reached only when built with -Dtrack_narrowphase_stats=true, same as
    // reportBroadphaseStats above.
}

//=============================================================================
// Body-vs-body narrow-phase collide hook
//=============================================================================

/// One-way-platform shape: skips the pair naming `skipped` entirely (no hit
/// added, no delegation), delegates every other pair to Jolt's own default.
const SkipOne = struct {
    skipped: zjolt.BodyId,
    delegated: u32 = 0,

    pub fn collide(
        self: *SkipOne,
        body1: zjolt.Body,
        body2: zjolt.Body,
        transform1: zjolt.Mat44,
        transform2: zjolt.Mat44,
        settings: *zjolt.CollideShapeSettings,
        shape_filter: *const system_mod.SimCollideShapeFilter,
        collector: *system_mod.SimCollideCollector,
    ) void {
        const id1 = body1.id();
        const id2 = body2.id();
        if (id1 == self.skipped or id2 == self.skipped) return;

        self.delegated += 1;
        system_mod.simCollideDefault(
            body1,
            body2,
            transform1,
            transform2,
            settings,
            shape_filter,
            collector,
        );
    }
};

test "simCollideBodyVsBody suppresses one pair and simCollideDefault runs every other unchanged" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var rig = try Rig.init(.{ .layers = undefined });
    defer rig.deinit();

    // A platform between the start height and the floor, exactly as the
    // SimShapeFilter test above -- but suppressed through the collide hook
    // this time, not the shape filter.
    const platform_shape = try zjolt.Shape.initBox(zjolt.vec3(2, 0.25, 2), .{});
    defer platform_shape.release();
    const platform = try rig.system.bodies().createAndAdd(.{
        .shape = platform_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 2, 0),
    }, .dont_activate);
    rig.system.optimizeBroadPhase();

    var ctx = SkipOne{ .skipped = platform };
    const hook = system_mod.simCollideBodyVsBody(SkipOne, &ctx);
    try rig.system.setSimCollideBodyVsBody(&hook);

    const installed = rig.system.getSimCollideBodyVsBody();
    try std.testing.expect(installed.collide != null);
    try std.testing.expect(installed.user == @as(?*anyopaque, @ptrCast(&ctx)));

    const ball = try rig.dropBall(5);

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
    defer jobs.deinit();
    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    while (elapsed < 3.0) : (elapsed += dt) {
        _ = try rig.system.step(dt, 1, jobs);
    }

    // The hook ran for real (not just installed): every other pair this run
    // touched, including ball-vs-floor, went through it.
    try std.testing.expect(ctx.delegated > 0);

    // Landed on the FLOOR (y ~= 0.5), not the platform (y ~= 2.75) -- the
    // suppressed pair really produced no collision, and the delegated pair
    // really reproduced Jolt's own default collision.
    const pos = rig.system.bodies().getPosition(ball);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), floatY(pos), 0.05);

    try rig.system.setSimCollideBodyVsBody(null);
    const cleared = rig.system.getSimCollideBodyVsBody();
    try std.testing.expect(cleared.collide == null);
}

/// Reports one hand-built hit for the ball-vs-floor pair, regardless of
/// whether the shapes actually overlap yet -- proof that addSimCollideHit's
/// hit reaches the real contact pipeline (manifold, contact cache), not just
/// that it does not crash.
const SyntheticContact = struct {
    ball: zjolt.BodyId,
    floor: zjolt.BodyId,
    hits_added: u32 = 0,

    pub fn collide(
        self: *SyntheticContact,
        body1: zjolt.Body,
        body2: zjolt.Body,
        transform1: zjolt.Mat44,
        transform2: zjolt.Mat44,
        settings: *zjolt.CollideShapeSettings,
        shape_filter: *const system_mod.SimCollideShapeFilter,
        collector: *system_mod.SimCollideCollector,
    ) void {
        _ = transform1;
        _ = transform2;
        _ = settings;
        _ = shape_filter;
        const id1 = body1.id();
        const id2 = body2.id();
        const is_ball_floor = (id1 == self.ball and id2 == self.floor) or
            (id1 == self.floor and id2 == self.ball);
        if (!is_ball_floor) return;

        self.hits_added += 1;
        system_mod.addSimCollideHit(collector, id2, .{
            .sub_shape_id1 = zjolt.sub_shape_id_empty,
            .sub_shape_id2 = zjolt.sub_shape_id_empty,
            .contact_point_on_1 = zjolt.vec3(0, 0, 0),
            .contact_point_on_2 = zjolt.vec3(0, 0, 0),
            .penetration_axis = zjolt.vec3(0, 1, 0),
            .penetration_depth = 0.01,
        });
    }
};

test "addSimCollideHit's synthetic hit reaches the real contact pipeline" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Built by hand rather than through Rig: this test needs the floor's own
    // id, which Rig.init discards.
    const opts: zjolt.PhysicsSystem.Options = .{ .layers = zjolt.layersFromType(Layers), .max_bodies = 8 };
    const system = try zjolt.PhysicsSystem.init(opts);
    defer system.deinit();
    system.setGravity(zjolt.gravity_earth);

    const floor_shape = try zjolt.Shape.initBox(zjolt.vec3(50, 0.5, 50), .{});
    defer floor_shape.release();
    const floor = try system.bodies().createAndAdd(.{
        .shape = floor_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, -0.5, 0),
    }, .dont_activate);
    system.optimizeBroadPhase();

    const ball_shape = try zjolt.Shape.initSphere(0.5, .{});
    defer ball_shape.release();
    const ball = try system.bodies().createAndAdd(.{
        .shape = ball_shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);

    var ctx = SyntheticContact{ .ball = ball, .floor = floor };
    const hook = system_mod.simCollideBodyVsBody(SyntheticContact, &ctx);
    try system.setSimCollideBodyVsBody(&hook);

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
    defer jobs.deinit();
    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    while (elapsed < 2.0 and ctx.hits_added == 0) : (elapsed += dt) {
        _ = try system.step(dt, 1, jobs);
    }

    try std.testing.expect(ctx.hits_added > 0);
    try std.testing.expect(system.wereBodiesInContact(ball, floor));
}
