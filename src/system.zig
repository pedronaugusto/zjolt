//! The physics system: collision layers, listeners, the step, and bulk
//! read-back.

const std = @import("std");
const c = @import("c/system.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const query_mod = @import("query.zig");
const broadphase_mod = @import("broadphase.zig");
const batch_mod = @import("batch.zig");
const state_mod = @import("state.zig");

pub const ObjectLayer = c.ObjectLayer;
pub const BroadPhaseLayer = c.BroadPhaseLayer;

//=============================================================================
// Job system
//
// Which threads the step runs on is a host decision. v0.1 ships Jolt's
// own thread pool and single-threaded scheduler; a host scheduler plugs in later without any call taking a `JobSystem` changing shape.
//=============================================================================

pub const JobSystem = struct {
    handle: *c.JobSystem,

    pub const Options = struct {
        max_jobs: u32 = c.max_physics_jobs,
        max_barriers: u32 = c.max_physics_barriers,
        /// -1 means one worker per hardware thread minus the calling thread.
        num_threads: i32 = -1,
        /// Called on the STARTING thread when a thread-pool worker starts or
        /// exits, with its index. Supplying either routes `initThreadPool`
        /// through a different C constructor than leaving both null: Jolt's
        /// own SetThreadInitFunction/SetThreadExitFunction only take effect
        /// if they are set before the pool's threads start, which the
        /// single-call constructor never gives a chance to do.
        thread_init: ?c.ThreadHookFn = null,
        thread_exit: ?c.ThreadHookFn = null,
        thread_hooks_user: ?*anyopaque = null,
    };

    pub fn initThreadPool(opts: Options) err.Error!JobSystem {
        var handle: *c.JobSystem = undefined;
        if (opts.thread_init == null and opts.thread_exit == null) {
            try err.check(c.zjoltJobSystemCreateThreadPool(
                opts.max_jobs,
                opts.max_barriers,
                opts.num_threads,
                &handle,
            ));
        } else {
            try err.check(c.zjoltJobSystemCreateThreadPoolWithHooks(
                opts.max_jobs,
                opts.max_barriers,
                opts.num_threads,
                opts.thread_init,
                opts.thread_exit,
                opts.thread_hooks_user,
                &handle,
            ));
        }
        return .{ .handle = handle };
    }

    /// Resizes a thread-pool job system's worker count, stopping and
    /// restarting its threads. `error.InvalidArgument` if this job system was
    /// not created by `initThreadPool` — the single-threaded and host-backed
    /// kinds have no thread count of their own to resize.
    pub fn setNumThreads(self: JobSystem, num_threads: i32) err.Error!void {
        try err.check(c.zjoltJobSystemSetNumThreads(self.handle, num_threads));
    }

    /// Runs every job on the calling thread. Slower, but it makes a step
    /// reproducible without pinning a thread count — which is what the tests
    /// use, and what a deterministic replay would want.
    pub fn initSingleThreaded(max_jobs: u32) err.Error!JobSystem {
        var handle: *c.JobSystem = undefined;
        try err.check(c.zjoltJobSystemCreateSingleThreaded(max_jobs, &handle));
        return .{ .handle = handle };
    }

    /// One unit of the step's work, handed to a host scheduler's `queueJob(s)`
    /// and run back on the library through `run`. `handle` is guaranteed
    /// alive only for the duration of the callback that receives it — call
    /// `addRef` first if you need it longer.
    pub const Job = extern struct {
        handle: *c.Job,

        /// Runs the job's function on the calling thread. May itself queue
        /// further jobs before returning, on THIS thread — see `initHost`.
        pub fn run(self: Job) void {
            c.zjoltJobRun(self.handle);
        }

        /// Takes a reference, keeping the job alive past the `queueJob(s)`
        /// call that handed it to you.
        pub fn addRef(self: Job) void {
            c.zjoltJobAddRef(self.handle);
        }

        /// Releases a reference taken by `queueJob(s)` or `addRef`. Call this
        /// exactly once for each — including once for the reference
        /// `queueJob(s)` itself took, after `run` returns.
        pub fn release(self: Job) void {
            c.zjoltJobRelease(self.handle);
        }
    };

    /// Wraps a host's own task graph in a job system Jolt's step can run
    /// on, instead of spawning threads of its own. `T` declares
    /// `getMaxConcurrency`, `queueJob`, `queueJobs`. Every job handed to
    /// either callback is alive for the DURATION of the call only —
    /// `job.addRef()` before returning if it runs later or elsewhere,
    /// `job.run()` there, `job.release()` once done. `context` must outlive the job system; no-unwinding applies as for a contact listener.
    pub fn initHost(comptime T: type, context: *T, max_barriers: u32) err.Error!JobSystem {
        const Thunks = struct {
            fn selfOf(user: ?*anyopaque) *T {
                return @ptrCast(@alignCast(user.?));
            }
            fn maxConcurrency(user: ?*anyopaque) callconv(.c) u32 {
                return T.getMaxConcurrency(selfOf(user));
            }
            fn queueJob(user: ?*anyopaque, job: *c.Job) callconv(.c) void {
                T.queueJob(selfOf(user), .{ .handle = job });
            }
            fn queueJobs(user: ?*anyopaque, jobs: [*]const *c.Job, count: u32) callconv(.c) void {
                const handles: [*]const Job = @ptrCast(jobs);
                T.queueJobs(selfOf(user), handles[0..count]);
            }
        };

        const host: c.HostJobSystem = .{
            .get_max_concurrency = Thunks.maxConcurrency,
            .queue_job = Thunks.queueJob,
            .queue_jobs = Thunks.queueJobs,
            .user = @ptrCast(context),
        };
        var handle: *c.JobSystem = undefined;
        try err.check(c.zjoltJobSystemCreateHost(&host, max_barriers, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: JobSystem) void {
        c.zjoltJobSystemDestroy(self.handle);
    }

    pub fn maxConcurrency(self: JobSystem) u32 {
        return c.zjoltJobSystemGetMaxConcurrency(self.handle);
    }
};

//=============================================================================
// Collision layers
//
// Jolt asks the host three questions during the broad phase; this turns
// a plain Zig type answering them into the three C tables the library wants, so a host writes no `callconv(.c)` or vtable of its own.
//=============================================================================

pub const Layers = struct {
    broad_phase: c.BroadPhaseLayerInterface,
    object_vs_broad_phase: c.ObjectVsBroadPhaseLayerFilter,
    object_layer_pair: c.ObjectLayerPairFilter,
};

/// Compile error unless `T` declares at least one of `names` — every
/// optional listener callback is found by `@hasDecl`, which only sees
/// `pub` declarations, so a misspelled or non-`pub` method would
/// otherwise install an all-null vtable and silently never fire.
pub fn requireAnyDecl(comptime T: type, comptime names: []const []const u8) void {
    comptime {
        var found = false;
        for (names) |name| {
            if (@hasDecl(T, name)) found = true;
        }
        if (!found) {
            var list: []const u8 = "";
            for (names) |name| list = list ++ "\n  pub fn " ++ name;
            @compileError(@typeName(T) ++ " declares none of the callbacks this listener" ++
                " looks for, so it would be installed and never called. `@hasDecl` only" ++
                " sees `pub` declarations across files — check that yours are `pub`, and" ++
                " that they are spelled as one of:" ++ list);
        }
    }
}

/// Builds the three layer tables from a type declaring
/// `broadPhaseLayerCount`, `broadPhaseLayerFor`,
/// `objectCanCollideWithBroadPhase`, `objectsCanCollide`, and optionally
/// `broadPhaseLayerName` (silently never called if misspelled or non-`pub`
/// — `@hasDecl` only sees `pub`). Its returned string is cached for the
/// system's whole life; return static storage, never a stack buffer.
pub fn layersFromType(comptime T: type) Layers {
    comptime {
        for ([_][]const u8{
            "broadPhaseLayerCount",
            "broadPhaseLayerFor",
            "objectCanCollideWithBroadPhase",
            "objectsCanCollide",
        }) |required| {
            if (!@hasDecl(T, required)) {
                @compileError(@typeName(T) ++ " is missing `pub fn " ++ required ++
                    "`, which zjolt needs to answer Jolt's broad-phase questions");
            }
        }
    }

    const Thunks = struct {
        fn count(user: ?*anyopaque) callconv(.c) u32 {
            _ = user;
            return T.broadPhaseLayerCount();
        }
        fn layerFor(user: ?*anyopaque, layer: ObjectLayer) callconv(.c) BroadPhaseLayer {
            _ = user;
            return T.broadPhaseLayerFor(layer);
        }
        fn objectVsBroad(
            user: ?*anyopaque,
            object: ObjectLayer,
            broad: BroadPhaseLayer,
        ) callconv(.c) bool {
            _ = user;
            return T.objectCanCollideWithBroadPhase(object, broad);
        }
        fn objectPair(user: ?*anyopaque, a: ObjectLayer, b: ObjectLayer) callconv(.c) bool {
            _ = user;
            return T.objectsCanCollide(a, b);
        }
        fn layerName(user: ?*anyopaque, layer: BroadPhaseLayer) callconv(.c) ?[*:0]const u8 {
            _ = user;
            return T.broadPhaseLayerName(layer);
        }
    };

    return .{
        .broad_phase = .{
            .num_broad_phase_layers = Thunks.count,
            .broad_phase_layer_for_object_layer = Thunks.layerFor,
            .broad_phase_layer_name = if (@hasDecl(T, "broadPhaseLayerName"))
                Thunks.layerName
            else
                null,
        },
        .object_vs_broad_phase = .{ .should_collide = Thunks.objectVsBroad },
        .object_layer_pair = .{ .should_collide = Thunks.objectPair },
    };
}

//=============================================================================
// Listeners
//
// IMPORTANT: contact callbacks run on Jolt's job threads, in parallel,
// inside `step` — re-entrant, no calling back in (queue and drain after). Activation callbacks run serialised, under the body manager's lock.
//=============================================================================

pub const ContactInfo = c.ContactInfo;
pub const ContactManifold = c.ContactManifold;
pub const ContactSettings = c.ContactSettings;
pub const ContactValidateInfo = c.ContactValidateInfo;
pub const SubShapeIdPair = c.SubShapeIdPair;
pub const ValidateResult = c.ValidateResult;

/// The contact points of a manifold, as a slice. Valid only for the duration
/// of the callback that was handed the manifold.
pub fn contactPointsOn1(manifold: *const ContactManifold) []const math.Vec3 {
    const ptr = manifold.points_on_1 orelse return &.{};
    return ptr[0..manifold.num_points];
}

pub fn contactPointsOn2(manifold: *const ContactManifold) []const math.Vec3 {
    const ptr = manifold.points_on_2 orelse return &.{};
    return ptr[0..manifold.num_points];
}

/// The colliding face `onContactValidate` was shown for shape 1, as a slice —
/// empty exactly when Jolt's own CollideShapeResult reports no face for that
/// shape (a sphere, for instance). Relative to `info.base_offset`, like the
/// contact points next to it, and valid only for the duration of the call.
pub fn contactValidateFace1(info: *const ContactValidateInfo) []const math.Vec3 {
    const ptr = info.shape1_face orelse return &.{};
    return ptr[0..info.num_shape1_face_vertices];
}

pub fn contactValidateFace2(info: *const ContactValidateInfo) []const math.Vec3 {
    const ptr = info.shape2_face orelse return &.{};
    return ptr[0..info.num_shape2_face_vertices];
}

/// The live body Jolt handed this callback in place of `body1`. Read
/// real state (velocity, shape, motion type, layer) with `body.zig`'s
/// `Body` accessors — the zjoltBody*Locked family underneath. No lock of
/// your own is needed or allowed: taking one here would deadlock the
/// job thread that already excludes it from that concern. Valid only
/// for the callback's duration; takes `*const ContactValidateInfo` or `*const ContactInfo`.
pub fn contactBody1(info: anytype) body_mod.Body {
    return .{ .handle = @constCast(info.live_body1.?) };
}

pub fn contactBody2(info: anytype) body_mod.Body {
    return .{ .handle = @constCast(info.live_body2.?) };
}

/// Builds a contact listener from `context` and whichever of
/// `onContactValidate`, `onContactAdded`, `onContactPersisted`,
/// `onContactRemoved` `T` declares — any it omits simply does not fire.
/// `context` must outlive the system it is installed on.
pub fn contactListener(comptime T: type, context: *T) c.ContactListener {
    requireAnyDecl(T, &.{
        "onContactValidate", "onContactAdded", "onContactPersisted", "onContactRemoved",
    });

    const Thunks = struct {
        fn selfOf(user: ?*anyopaque) *T {
            return @ptrCast(@alignCast(user.?));
        }
        fn validate(user: ?*anyopaque, info: *const ContactValidateInfo) callconv(.c) ValidateResult {
            return T.onContactValidate(selfOf(user), info);
        }
        fn added(user: ?*anyopaque, info: *const ContactInfo, settings: *ContactSettings) callconv(.c) void {
            T.onContactAdded(selfOf(user), info, settings);
        }
        fn persisted(user: ?*anyopaque, info: *const ContactInfo, settings: *ContactSettings) callconv(.c) void {
            T.onContactPersisted(selfOf(user), info, settings);
        }
        fn removed(user: ?*anyopaque, pair: *const SubShapeIdPair) callconv(.c) void {
            T.onContactRemoved(selfOf(user), pair);
        }
    };

    return .{
        .on_contact_validate = if (@hasDecl(T, "onContactValidate")) Thunks.validate else null,
        .on_contact_added = if (@hasDecl(T, "onContactAdded")) Thunks.added else null,
        .on_contact_persisted = if (@hasDecl(T, "onContactPersisted")) Thunks.persisted else null,
        .on_contact_removed = if (@hasDecl(T, "onContactRemoved")) Thunks.removed else null,
        .user = @ptrCast(context),
    };
}

/// Builds an activation listener from `context` and whichever of
/// `onBodyActivated(self, BodyId, u64)` / `onBodyDeactivated(self, BodyId, u64)`
/// `T` declares.
pub fn bodyActivationListener(comptime T: type, context: *T) c.BodyActivationListener {
    requireAnyDecl(T, &.{ "onBodyActivated", "onBodyDeactivated" });

    const Thunks = struct {
        fn selfOf(user: ?*anyopaque) *T {
            return @ptrCast(@alignCast(user.?));
        }
        fn activated(user: ?*anyopaque, body: c.BodyId, user_data: u64) callconv(.c) void {
            T.onBodyActivated(selfOf(user), body, user_data);
        }
        fn deactivated(user: ?*anyopaque, body: c.BodyId, user_data: u64) callconv(.c) void {
            T.onBodyDeactivated(selfOf(user), body, user_data);
        }
    };

    return .{
        .on_body_activated = if (@hasDecl(T, "onBodyActivated")) Thunks.activated else null,
        .on_body_deactivated = if (@hasDecl(T, "onBodyDeactivated")) Thunks.deactivated else null,
        .user = @ptrCast(context),
    };
}

//=============================================================================
// Callbacks that can fail
//
// Nothing may unwind out of a Jolt callback: exceptions are compiled off,
// and a Zig panic out of a step listener or combine callback PERMANENTLY DEADLOCKS the next `step`. The shims below catch instead, re-raised by `check` after the step.
//=============================================================================

const ErrorInt = std.meta.Int(.unsigned, @bitSizeOf(anyerror));

/// The first error a callback signalled, whichever job thread it ran on.
///
/// Atomic because a combine callback is called concurrently for different
/// contacts, and because step listeners for one system can be spread across
/// jobs. First wins: the one that started the trouble is more useful than the
/// last one to notice it.
const Failure = struct {
    code: std.atomic.Value(ErrorInt) = std.atomic.Value(ErrorInt).init(0),

    fn record(self: *Failure, e: anyerror) void {
        _ = self.code.cmpxchgStrong(0, @intFromError(e), .monotonic, .monotonic);
    }

    fn take(self: *Failure) ?anyerror {
        const code = self.code.swap(0, .monotonic);
        return if (code == 0) null else @errorFromInt(code);
    }
};

fn returnsError(comptime Fn: type) bool {
    const ret = @typeInfo(Fn).@"fn".return_type orelse return false;
    return @typeInfo(ret) == .error_union;
}

pub const StepListenerContext = c.StepListenerContext;
pub const CombineInfo = c.CombineInfo;

/// A host callback run once per collision step, before that step's
/// collision detection — for forces applied inside the sub-step loop
/// (buoyancy, thrusters, wind), so each of several sub-steps gets one.
/// `T` declares `onStep(*T, *const StepListenerContext)` (or `!void`).
/// Every body/constraint mutex is held: READ/WRITE bodies, but no
/// add/remove or calling back in. Not called with no active bodies or a zero dt. MUST NOT MOVE once attached.
pub fn StepListener(comptime T: type) type {
    return struct {
        const Self = @This();

        context: *T,
        failure: Failure = .{},
        handle: ?*c.StepListener = null,

        pub fn init(context: *T) Self {
            return .{ .context = context };
        }

        fn thunk(user: ?*anyopaque, ctx: *const StepListenerContext) callconv(.c) void {
            const self: *Self = @ptrCast(@alignCast(user.?));
            if (comptime returnsError(@TypeOf(T.onStep))) {
                T.onStep(self.context, ctx) catch |e| self.failure.record(e);
            } else {
                T.onStep(self.context, ctx);
            }
        }

        pub fn attach(self: *Self, system: PhysicsSystem) err.Error!void {
            var handle: *c.StepListener = undefined;
            try err.check(c.zjoltPhysicsSystemAddStepListener(
                system.handle,
                thunk,
                @ptrCast(self),
                &handle,
            ));
            self.handle = handle;
        }

        /// Removes the listener. Attaching twice yields two listeners, so a
        /// detach is per attach; detaching one that is not attached does
        /// nothing.
        pub fn detach(self: *Self, system: PhysicsSystem) err.Error!void {
            const handle = self.handle orelse return;
            self.handle = null;
            try err.check(c.zjoltPhysicsSystemRemoveStepListener(system.handle, handle));
        }

        /// Re-raises the first error the callback signalled since the last
        /// call, and clears it. A callback that failed does NOT stop the step
        /// or leave the system unusable — it ran, it returned, and this is
        /// where you find out.
        pub fn check(self: *Self) anyerror!void {
            if (self.failure.take()) |e| return e;
        }
    };
}

/// A host answer to "what friction, or restitution, does this contact
/// gets". Jolt's defaults are `sqrt(f1 * f2)` and `max(r1, r2)`; a host
/// with a material table usually wants its own. `T` declares
/// `combine(*T, *const CombineInfo) f32` (or `!f32`), run on Jolt's job
/// threads DURING a step: re-entrant, no calls back into the system. An
/// error yields 0 for that contact and surfaces from `check`. MUST NOT MOVE once attached.
pub fn CombineCallback(comptime T: type) type {
    return struct {
        const Self = @This();

        context: *T,
        failure: Failure = .{},

        pub fn init(context: *T) Self {
            return .{ .context = context };
        }

        fn thunk(user: ?*anyopaque, info: *const CombineInfo) callconv(.c) f32 {
            const self: *Self = @ptrCast(@alignCast(user.?));
            if (comptime returnsError(@TypeOf(T.combine))) {
                return T.combine(self.context, info) catch |e| {
                    self.failure.record(e);
                    return 0;
                };
            }
            return T.combine(self.context, info);
        }

        pub fn attachFriction(self: *Self, system: PhysicsSystem) err.Error!void {
            try err.check(c.zjoltPhysicsSystemSetCombineFriction(
                system.handle,
                thunk,
                @ptrCast(self),
            ));
        }

        pub fn attachRestitution(self: *Self, system: PhysicsSystem) err.Error!void {
            try err.check(c.zjoltPhysicsSystemSetCombineRestitution(
                system.handle,
                thunk,
                @ptrCast(self),
            ));
        }

        /// @see StepListener.check
        pub fn check(self: *Self) anyerror!void {
            if (self.failure.take()) |e| return e;
        }
    };
}

//=============================================================================
// The step's scratch allocator
//=============================================================================

pub const TempAllocatorKind = c.TempAllocatorKind;
pub const TempAllocator = c.TempAllocator;
pub const TempAllocatorStats = c.TempAllocatorStats;

/// Builds a `TempAllocator` table from `context` and `T`, which declares
/// `allocate`/`free`, and optionally `canAllocate`, `getSize`, `getUsage`
/// — each omitted one answers as documented on `ZJoltTempAllocator` in
/// `ffi/zjolt_system.h`.
///
/// Runs on Jolt's job threads during a step; nothing may propagate out of `allocate`/`free`.
pub fn hostTempAllocator(comptime T: type, context: *T) TempAllocator {
    const Thunks = struct {
        fn selfOf(user: ?*anyopaque) *T {
            return @ptrCast(@alignCast(user.?));
        }
        fn allocate(user: ?*anyopaque, size: u32) callconv(.c) ?*anyopaque {
            return T.allocate(selfOf(user), size);
        }
        fn free(user: ?*anyopaque, address: ?*anyopaque, size: u32) callconv(.c) void {
            T.free(selfOf(user), address, size);
        }
        fn canAllocate(user: ?*anyopaque, size: u32) callconv(.c) bool {
            return T.canAllocate(selfOf(user), size);
        }
        fn getSize(user: ?*anyopaque) callconv(.c) usize {
            return T.getSize(selfOf(user));
        }
        fn getUsage(user: ?*anyopaque) callconv(.c) usize {
            return T.getUsage(selfOf(user));
        }
    };

    return .{
        .allocate = Thunks.allocate,
        .free = Thunks.free,
        .can_allocate = if (@hasDecl(T, "canAllocate")) Thunks.canAllocate else null,
        .get_size = if (@hasDecl(T, "getSize")) Thunks.getSize else null,
        .get_usage = if (@hasDecl(T, "getUsage")) Thunks.getUsage else null,
        .user = @ptrCast(context),
    };
}

//=============================================================================
// Simulation shape filter
//
// Consulted by the step itself, at every compound level, on Jolt's job
// threads, concurrently — distinct from query.zig's shape filter, which applies only to an explicit query.
//=============================================================================

pub const SimShapeFilter = c.SimShapeFilter;

/// Builds a `SimShapeFilter` from `context` and `T`, which declares:
/// `pub fn shouldCollide(self: *T, body1: BodyId, shape1: ?*const c.Shape, sub1: SubShapeId, body2: BodyId, shape2: ?*const c.Shape, sub2: SubShapeId) bool`.
///
/// `sub1`/`sub2` lead from the whole body's own shape down to `shape1`/
/// `shape2`; each is `zjolt.sub_shape_id_empty` for a body whose shape is not
/// compound. Must not call back into the system; nothing may propagate out.
pub fn simShapeFilter(comptime T: type, context: *T) SimShapeFilter {
    const Thunks = struct {
        fn shouldCollide(
            user: ?*anyopaque,
            body1: c.BodyId,
            shape1: ?*const c.Shape,
            sub_shape_id1: c.SubShapeId,
            body2: c.BodyId,
            shape2: ?*const c.Shape,
            sub_shape_id2: c.SubShapeId,
        ) callconv(.c) bool {
            const self: *T = @ptrCast(@alignCast(user.?));
            return T.shouldCollide(
                self,
                body1,
                shape1,
                sub_shape_id1,
                body2,
                shape2,
                sub_shape_id2,
            );
        }
    };
    return .{ .should_collide = Thunks.shouldCollide, .user = @ptrCast(context) };
}

//=============================================================================
// Body-vs-body narrow-phase collide hook
//
// PhysicsSystem::SetSimCollideBodyVsBody replaces the shape-vs-shape collide
// the step itself uses for every candidate body pair -- one-way platforms,
// per-pair overrides. Runs on Jolt's job threads during the step, once per
// candidate pair; no calls back into the system, nothing may propagate out.
//=============================================================================

pub const SimCollideHit = c.SimCollideHit;
pub const SimCollideCollector = c.SimCollideCollector;
pub const SimCollideShapeFilter = c.SimCollideShapeFilter;
pub const SimCollideBodyVsBody = c.SimCollideBodyVsBody;

/// Reports one hit from inside the `collide` callback `simCollideBodyVsBody`
/// built. May be called any number of times, including zero -- a suppressed
/// pair adds none. Call before, not after, `simCollideDefault` if you need
/// both: a pair a contact-validate listener already rejected silently
/// accepts no further hit.
pub fn addSimCollideHit(
    collector: *SimCollideCollector,
    body2: body_mod.BodyId,
    hit: SimCollideHit,
) void {
    c.zjoltSimCollideAddHit(collector, body2, &hit);
}

/// Runs Jolt's own default body-vs-body collide for this pair, reporting
/// through the same `collector` the `collide` callback was given. Null
/// `settings` takes Jolt's own defaults.
pub fn simCollideDefault(
    live_body1: body_mod.Body,
    live_body2: body_mod.Body,
    center_of_mass_transform1: math.Mat44,
    center_of_mass_transform2: math.Mat44,
    settings: ?*const query_mod.CollideShapeSettings,
    shape_filter: ?*const SimCollideShapeFilter,
    collector: *SimCollideCollector,
) void {
    c.zjoltSimCollideDefault(
        live_body1.handle,
        live_body2.handle,
        &center_of_mass_transform1,
        &center_of_mass_transform2,
        settings,
        shape_filter,
        collector,
    );
}

/// Builds a `SimCollideBodyVsBody` from `context` and `T`, which declares
/// `pub fn collide(self: *T, body1: Body, body2: Body, transform1: Mat44,
/// transform2: Mat44, settings: *CollideShapeSettings, shapeFilter: *const
/// SimCollideShapeFilter, collector: *SimCollideCollector) void`. Add hits
/// with `addSimCollideHit`, delegate with `simCollideDefault`, or both.
/// `context` must outlive the system it is installed on.
pub fn simCollideBodyVsBody(comptime T: type, context: *T) SimCollideBodyVsBody {
    const Thunks = struct {
        fn collide(
            user: ?*anyopaque,
            live_body1: *const c.Body,
            live_body2: *const c.Body,
            transform1: *const c.Mat44,
            transform2: *const c.Mat44,
            settings: *c.CollideShapeSettings,
            shape_filter: *const c.SimCollideShapeFilter,
            collector: *c.SimCollideCollector,
        ) callconv(.c) void {
            const self: *T = @ptrCast(@alignCast(user.?));
            T.collide(
                self,
                .{ .handle = @constCast(live_body1) },
                .{ .handle = @constCast(live_body2) },
                transform1.*,
                transform2.*,
                settings,
                shape_filter,
                collector,
            );
        }
    };
    return .{ .collide = Thunks.collide, .user = @ptrCast(context) };
}

//=============================================================================
// Physics system
//=============================================================================

/// The constants of the solver. Jolt's `PhysicsSettings`, field for field;
/// what each one means is documented in `ffi/zjolt_system.h`.
///
/// Read the current ones, change what you mean, write them back — there is no
/// per-field setter because several of these interact.
pub const PhysicsSettings = c.PhysicsSettings;

/// Jolt's own defaults, read out of a default-constructed `PhysicsSettings`
/// rather than transcribed, so they cannot drift from the vendored library.
pub fn defaultPhysicsSettings() PhysicsSettings {
    var settings: PhysicsSettings = undefined;
    c.zjoltPhysicsSettingsInit(&settings);
    return settings;
}

/// What a step silently dropped, if anything. A non-zero value means contacts
/// were ignored and the matching limit in `Options` should be raised — the
/// step itself did not fail.
pub const UpdateError = c.UpdateError;

/// Traces the narrow phase's own accumulated per-shape-pair timing stats to
/// the TTY as CSV. Process-wide, not per system: Jolt keeps one set of
/// tables regardless of how many physics systems exist. Diagnostic output
/// only, nothing returned. `error.Unsupported` without
/// `-Dtrack_narrowphase_stats`.
pub fn reportNarrowPhaseStats() err.Error!void {
    try err.check(c.zjoltReportNarrowPhaseStats());
}

pub const PhysicsSystem = struct {
    handle: *c.PhysicsSystem,

    pub const Options = struct {
        layers: Layers,
        /// Hard ceiling on bodies. Cannot grow.
        /// Cannot exceed 8388608 — Jolt packs a body's index and generation
        /// counter into one 32-bit id.
        max_bodies: u32 = 10240,
        /// Body mutexes for parallel access; 0 picks Jolt's default.
        num_body_mutexes: u32 = 0,
        max_body_pairs: u32 = 65536,
        max_contact_constraints: u32 = 65536,
        /// Scratch arena for one step. 0 picks 10 MiB. Used by
        /// `.malloc_fallback` (its fallback path aside) and `.fixed`; ignored
        /// for `.host`. A `.malloc_fallback` step needing more falls back to
        /// the host allocator rather than failing; a `.fixed` one aborts.
        temp_allocator_size: usize = 0,
        /// Which strategy backs the step's scratch allocator.
        temp_allocator_kind: TempAllocatorKind = .malloc_fallback,
        /// Required, and read, only when `temp_allocator_kind` is `.host` —
        /// build one with `hostTempAllocator`. Copied at create, so it need
        /// not outlive the call — but its `user` must outlive the system.
        temp_allocator: ?*const TempAllocator = null,
    };

    pub fn init(opts: Options) err.Error!PhysicsSystem {
        var desc: c.PhysicsSystemDesc = undefined;
        c.zjoltPhysicsSystemDescInit(&desc);
        desc.max_bodies = opts.max_bodies;
        desc.num_body_mutexes = opts.num_body_mutexes;
        desc.max_body_pairs = opts.max_body_pairs;
        desc.max_contact_constraints = opts.max_contact_constraints;
        if (opts.temp_allocator_size != 0) desc.temp_allocator_size = opts.temp_allocator_size;
        desc.temp_allocator_kind = opts.temp_allocator_kind;
        desc.temp_allocator = opts.temp_allocator;
        desc.broad_phase_layers = opts.layers.broad_phase;
        desc.object_vs_broad_phase_filter = opts.layers.object_vs_broad_phase;
        desc.object_layer_pair_filter = opts.layers.object_layer_pair;

        var handle: *c.PhysicsSystem = undefined;
        try err.check(c.zjoltPhysicsSystemCreate(&desc, &handle));
        return .{ .handle = handle };
    }

    /// Destroys the system and every body still in it. Shapes are released,
    /// not destroyed — one outlives the system if the host still holds a
    /// reference.
    pub fn deinit(self: PhysicsSystem) void {
        c.zjoltPhysicsSystemDestroy(self.handle);
    }

    //-------------------------------------------------------------------------
    // Reading back what init was given
    //
    // The three tables in Layers, and this system's scratch-allocator stats,
    // are copied in at create and Jolt itself never hands them back.
    //-------------------------------------------------------------------------

    pub fn getBroadPhaseLayerInterface(self: PhysicsSystem) c.BroadPhaseLayerInterface {
        var out: c.BroadPhaseLayerInterface = undefined;
        c.zjoltPhysicsSystemGetBroadPhaseLayerInterface(self.handle, &out);
        return out;
    }

    pub fn getObjectVsBroadPhaseLayerFilter(self: PhysicsSystem) c.ObjectVsBroadPhaseLayerFilter {
        var out: c.ObjectVsBroadPhaseLayerFilter = undefined;
        c.zjoltPhysicsSystemGetObjectVsBroadPhaseLayerFilter(self.handle, &out);
        return out;
    }

    pub fn getObjectLayerPairFilter(self: PhysicsSystem) c.ObjectLayerPairFilter {
        var out: c.ObjectLayerPairFilter = undefined;
        c.zjoltPhysicsSystemGetObjectLayerPairFilter(self.handle, &out);
        return out;
    }

    pub fn getTempAllocatorStats(self: PhysicsSystem) TempAllocatorStats {
        var out: TempAllocatorStats = undefined;
        c.zjoltPhysicsSystemGetTempAllocatorStats(self.handle, &out);
        return out;
    }

    /// Whether `size` more bytes could be allocated right now without
    /// spilling into a slower fallback (`.malloc_fallback`) or aborting
    /// (`.fixed`). @see ZJoltTempAllocator.can_allocate for `.host`.
    pub fn tempAllocatorCanAllocate(self: PhysicsSystem, size: u32) bool {
        return c.zjoltPhysicsSystemTempAllocatorCanAllocate(self.handle, size);
    }

    /// Body creation and per-body access.
    pub fn bodies(self: PhysicsSystem) body_mod.BodyInterface {
        return .{ .handle = self.handle };
    }

    /// Ray casts, shape casts and overlap tests.
    pub fn queries(self: PhysicsSystem) query_mod.Queries {
        return .{ .handle = self.handle };
    }

    /// Bounding-box queries: which bodies are roughly there. Cheaper than
    /// `queries()` and coarser — body ids only, no contact points.
    pub fn broadPhase(self: PhysicsSystem) broadphase_mod.BroadPhase {
        return .{ .handle = self.handle };
    }

    /// Adding, removing and waking many bodies at once.
    pub fn batch(self: PhysicsSystem) batch_mod.Batch {
        return .{ .handle = self.handle };
    }

    /// Saving and restoring the state a step changes.
    pub fn state(self: PhysicsSystem) state_mod.State {
        return .{ .handle = self.handle };
    }

    //-------------------------------------------------------------------------
    // World properties
    //-------------------------------------------------------------------------

    pub fn setGravity(self: PhysicsSystem, gravity: math.Vec3) void {
        c.zjoltPhysicsSystemSetGravity(self.handle, &gravity);
    }

    pub fn getGravity(self: PhysicsSystem) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltPhysicsSystemGetGravity(self.handle, &out);
        return out;
    }

    /// Rebuilds the broad phase for the bodies added so far. Worth calling
    /// once after bulk-loading static geometry, and never per frame.
    pub fn optimizeBroadPhase(self: PhysicsSystem) void {
        c.zjoltPhysicsSystemOptimizeBroadPhase(self.handle);
    }

    pub fn numBodies(self: PhysicsSystem) u32 {
        return c.zjoltPhysicsSystemGetNumBodies(self.handle);
    }

    pub fn numActiveBodies(self: PhysicsSystem) u32 {
        return c.zjoltPhysicsSystemGetNumActiveBodies(self.handle);
    }

    /// The ceiling this system was created with, which cannot grow.
    pub fn maxBodies(self: PhysicsSystem) u32 {
        return c.zjoltPhysicsSystemGetMaxBodies(self.handle);
    }

    pub const BodyStats = c.BodyStats;

    /// A census of the bodies in this system, broken down by motion type.
    /// Slow — it iterates every body — so this is for an occasional debug
    /// overlay, not a per-frame call.
    pub fn getBodyStats(self: PhysicsSystem) BodyStats {
        var out: BodyStats = undefined;
        c.zjoltPhysicsSystemGetBodyStats(self.handle, &out);
        return out;
    }

    /// Traces the broad phase's own accumulated per-layer query stats to the
    /// TTY as CSV -- diagnostic output only, nothing structured is returned.
    /// `error.Unsupported` without `-Dtrack_broadphase_stats`.
    pub fn reportBroadphaseStats(self: PhysicsSystem) err.Error!void {
        try err.check(c.zjoltPhysicsSystemReportBroadphaseStats(self.handle));
    }

    /// True if the two bodies were touching during the LAST step.
    ///
    /// This reads the contact cache, so it answers a question about the step
    /// that has already run rather than about where the bodies are now — which
    /// is what a gameplay rule usually wants, and far cheaper than an overlap
    /// query. False before the first step, and not to be called during one.
    pub fn wereBodiesInContact(
        self: PhysicsSystem,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
    ) bool {
        return c.zjoltPhysicsSystemWereBodiesInContact(self.handle, body1, body2);
    }

    //-------------------------------------------------------------------------
    // Simulation settings
    //-------------------------------------------------------------------------

    pub fn getSettings(self: PhysicsSystem) PhysicsSettings {
        var settings: PhysicsSettings = undefined;
        c.zjoltPhysicsSystemGetSettings(self.handle, &settings);
        return settings;
    }

    /// Applies every field at once, from the next step onwards.
    ///
    /// `error.InvalidArgument` for a non-positive batch size, batches per
    /// job or in-flight pair count, or a zero velocity iteration count —
    /// Jolt only asserts these, and a zero stride/divisor otherwise fails
    /// several frames later, as a step that never finishes or a division by zero.
    pub fn setSettings(self: PhysicsSystem, settings: PhysicsSettings) err.Error!void {
        try err.check(c.zjoltPhysicsSystemSetSettings(self.handle, &settings));
    }

    //-------------------------------------------------------------------------
    // Combine callbacks
    //
    // Installed through `CombineCallback`, which is what carries the error out
    // of a callback rather than letting it unwind. These two clear them.
    //-------------------------------------------------------------------------

    /// Puts Jolt's default back: the geometric mean `sqrt(f1 * f2)`.
    pub fn clearCombineFriction(self: PhysicsSystem) err.Error!void {
        try err.check(c.zjoltPhysicsSystemSetCombineFriction(self.handle, null, null));
    }

    /// Puts Jolt's default back: `max(r1, r2)`.
    pub fn clearCombineRestitution(self: PhysicsSystem) err.Error!void {
        try err.check(c.zjoltPhysicsSystemSetCombineRestitution(self.handle, null, null));
    }

    /// What `CombineCallback.attachRestitution` installed: null for both if
    /// Jolt's own default (`max(r1, r2)`) is active.
    pub const CombineHandle = struct {
        combine: ?c.CombineFn,
        user: ?*anyopaque,
    };

    pub fn getCombineRestitution(self: PhysicsSystem) CombineHandle {
        var combine: ?c.CombineFn = null;
        var user: ?*anyopaque = null;
        c.zjoltPhysicsSystemGetCombineRestitution(self.handle, &combine, &user);
        return .{ .combine = combine, .user = user };
    }

    //-------------------------------------------------------------------------
    // Listeners
    //-------------------------------------------------------------------------

    /// The struct is copied, so it need not outlive the call — but its `user`
    /// pointer must outlive the system. Pass null to clear.
    pub fn setContactListener(
        self: PhysicsSystem,
        listener: ?*const c.ContactListener,
    ) err.Error!void {
        try err.check(c.zjoltPhysicsSystemSetContactListener(self.handle, listener));
    }

    pub fn setBodyActivationListener(
        self: PhysicsSystem,
        listener: ?*const c.BodyActivationListener,
    ) err.Error!void {
        try err.check(
            c.zjoltPhysicsSystemSetBodyActivationListener(self.handle, listener),
        );
    }

    /// All-zero (every field null) if none is installed.
    pub fn getContactListener(self: PhysicsSystem) c.ContactListener {
        var out: c.ContactListener = undefined;
        c.zjoltPhysicsSystemGetContactListener(self.handle, &out);
        return out;
    }

    /// All-zero if none is installed.
    pub fn getBodyActivationListener(self: PhysicsSystem) c.BodyActivationListener {
        var out: c.BodyActivationListener = undefined;
        c.zjoltPhysicsSystemGetBodyActivationListener(self.handle, &out);
        return out;
    }

    /// What `zjoltSoftBodySetContactListener` (softbody.zig) installed, or
    /// all-zero if none is.
    pub fn getSoftBodyContactListener(self: PhysicsSystem) c.SoftBodyContactListener {
        var out: c.SoftBodyContactListener = undefined;
        c.zjoltPhysicsSystemGetSoftBodyContactListener(self.handle, &out);
        return out;
    }

    /// NULL restores Jolt's default (every shape pair may collide). The
    /// struct is copied — but `user` must outlive the system. @see
    /// `simShapeFilter` to build one from a Zig type.
    pub fn setSimShapeFilter(self: PhysicsSystem, filter: ?*const SimShapeFilter) err.Error!void {
        try err.check(c.zjoltPhysicsSystemSetSimShapeFilter(self.handle, filter));
    }

    /// All-zero if none is installed.
    pub fn getSimShapeFilter(self: PhysicsSystem) SimShapeFilter {
        var out: SimShapeFilter = undefined;
        c.zjoltPhysicsSystemGetSimShapeFilter(self.handle, &out);
        return out;
    }

    /// NULL, or a NULL `collide`, restores Jolt's default. The struct is
    /// copied — but `user` must outlive the system. @see
    /// `simCollideBodyVsBody` to build one from a Zig type.
    pub fn setSimCollideBodyVsBody(
        self: PhysicsSystem,
        hook: ?*const SimCollideBodyVsBody,
    ) err.Error!void {
        try err.check(c.zjoltPhysicsSystemSetSimCollideBodyVsBody(self.handle, hook));
    }

    /// All-zero if none is installed (Jolt's default is active).
    pub fn getSimCollideBodyVsBody(self: PhysicsSystem) SimCollideBodyVsBody {
        var out: SimCollideBodyVsBody = undefined;
        c.zjoltPhysicsSystemGetSimCollideBodyVsBody(self.handle, &out);
        return out;
    }

    //-------------------------------------------------------------------------
    // The unlocked path
    //
    // For inside a callback where locks are already held (contact/combine
    // listener, step listener); USE WITH GREAT CARE — nothing stops another thread mutating the body you get back while you read it.
    //-------------------------------------------------------------------------

    /// The body named by `id`, with NO lock taken. Null if `id` does not name
    /// a live body in this system.
    pub fn tryGetBodyNoLock(self: PhysicsSystem, id: body_mod.BodyId) ?body_mod.Body {
        const handle = c.zjoltPhysicsSystemTryGetBodyNoLock(self.handle, id) orelse
            return null;
        return .{ .handle = @constCast(handle) };
    }

    /// A live, zero-copy view of the same set `getActiveBodies` copies —
    /// PhysicsSystem::GetActiveBodiesUnsafe, for a caller unwilling to
    /// pay that copy every frame.
    ///
    /// Not thread safe, and invalidated by the next call that adds,
    /// removes, activates or deactivates a body — read what you need before doing anything else.
    pub fn getActiveBodiesUnsafe(self: PhysicsSystem) []const body_mod.BodyId {
        var ids: ?[*]const body_mod.BodyId = null;
        var count: u32 = 0;
        c.zjoltPhysicsSystemGetActiveBodiesUnsafe(self.handle, &ids, &count);
        const ptr = ids orelse return &.{};
        return ptr[0..count];
    }

    //-------------------------------------------------------------------------
    // The step
    //-------------------------------------------------------------------------

    /// Advances the simulation, returning the `UpdateError` mask.
    /// `collision_steps` splits the interval into sub-steps; 1 is normal
    /// — raise it for fast bodies rather than shortening the frame.
    ///
    /// A non-empty mask means contacts were dropped, not that the step
    /// failed. With asserts enabled, Jolt breaks into the debugger before returning a non-empty one — see UPSTREAM.md.
    pub fn step(
        self: PhysicsSystem,
        delta_time: f32,
        collision_steps: i32,
        job_system: JobSystem,
    ) err.Error!UpdateError {
        var update_error: UpdateError = .none;
        try err.check(c.zjoltPhysicsSystemStep(
            self.handle,
            delta_time,
            collision_steps,
            job_system.handle,
            &update_error,
        ));
        return update_error;
    }

    //-------------------------------------------------------------------------
    // Bulk read-back
    //
    // The per-body accessors are one ABI crossing and one lock each —
    // right for occasional use, wrong for a renderer's every frame. These are the frame-loop path: two crossings and one lock per batch, not 2N.
    //-------------------------------------------------------------------------

    pub fn countActiveBodies(self: PhysicsSystem) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltPhysicsSystemGetActiveBodies(self.handle, null, 0, &count));
        return count;
    }

    /// Ids of the bodies that are awake — the set whose transforms can have
    /// changed since the last step.
    pub fn getActiveBodies(
        self: PhysicsSystem,
        buffer: []body_mod.BodyId,
    ) err.Error![]body_mod.BodyId {
        var count: u32 = 0;
        try err.check(c.zjoltPhysicsSystemGetActiveBodies(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    pub fn countBodies(self: PhysicsSystem) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltPhysicsSystemGetBodies(self.handle, null, 0, &count));
        return count;
    }

    pub fn getBodies(
        self: PhysicsSystem,
        buffer: []body_mod.BodyId,
    ) err.Error![]body_mod.BodyId {
        var count: u32 = 0;
        try err.check(c.zjoltPhysicsSystemGetBodies(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// Reads many body transforms under a single lock.
    ///
    /// `positions` and `rotations` are parallel to `ids`; either may be null.
    /// Returns how many ids named a body that no longer exists — normal
    /// between a step and a read, not an error. Those entries receive the
    /// identity transform so the arrays stay aligned with `ids`.
    pub fn getTransforms(
        self: PhysicsSystem,
        ids: []const body_mod.BodyId,
        positions: ?[]math.RVec3,
        rotations: ?[]math.Quat,
    ) err.Error!u32 {
        if (positions) |p| if (p.len < ids.len) return err.Error.BufferTooSmall;
        if (rotations) |r| if (r.len < ids.len) return err.Error.BufferTooSmall;

        var missing: u32 = 0;
        try err.check(c.zjoltBodyGetTransforms(
            self.handle,
            ids.ptr,
            @intCast(ids.len),
            if (positions) |p| p.ptr else null,
            if (rotations) |r| r.ptr else null,
            &missing,
        ));
        return missing;
    }

    /// As `getTransforms`, but centre-of-mass positions and linear velocities
    /// — what an interpolating or audio-driven host wants.
    pub fn getMotions(
        self: PhysicsSystem,
        ids: []const body_mod.BodyId,
        center_of_mass: ?[]math.RVec3,
        linear_velocities: ?[]math.Vec3,
    ) err.Error!u32 {
        if (center_of_mass) |p| if (p.len < ids.len) return err.Error.BufferTooSmall;
        if (linear_velocities) |v| if (v.len < ids.len) return err.Error.BufferTooSmall;

        var missing: u32 = 0;
        try err.check(c.zjoltBodyGetMotions(
            self.handle,
            ids.ptr,
            @intCast(ids.len),
            if (center_of_mass) |p| p.ptr else null,
            if (linear_velocities) |v| v.ptr else null,
            &missing,
        ));
        return missing;
    }

    //-------------------------------------------------------------------------
    // Locks
    //
    // Jolt's own SharedLock/UniqueLock (PhysicsLock.h), reached only through BodyLockInterface here.
    // Do not call these from inside a contact, combine, or step listener — they already hold every body lock, and this is the reentry Jolt's own lock-order assert exists to catch; use the unlocked path above instead.
    //-------------------------------------------------------------------------

    /// Takes a shared lock on one body. Several readers may hold one at once.
    /// Release it exactly once, whether or not it found a body.
    pub fn lockRead(self: PhysicsSystem, id: body_mod.BodyId) body_mod.Lock {
        var lock: body_mod.Lock = .{ .raw = undefined, .write = false };
        c.zjoltBodyLockRead(self.handle, id, &lock.raw);
        return lock;
    }

    /// Takes an exclusive lock, required before any `Body` mutator.
    pub fn lockWrite(self: PhysicsSystem, id: body_mod.BodyId) body_mod.Lock {
        var lock: body_mod.Lock = .{ .raw = undefined, .write = true };
        c.zjoltBodyLockWrite(self.handle, id, &lock.raw);
        return lock;
    }

    /// Takes a shared lock over every body in `ids` at once — one mutex mask
    /// for the whole set, computed the way `BodyLockMultiRead` does, rather
    /// than one lock per id. `ids` must stay valid until `release`.
    pub fn lockMultiRead(self: PhysicsSystem, ids: []const body_mod.BodyId) body_mod.MultiLock {
        var lock: body_mod.MultiLock = .{ .raw = undefined, .write = false };
        c.zjoltBodyLockMultiRead(self.handle, ids.ptr, @intCast(ids.len), &lock.raw);
        return lock;
    }

    /// Takes an exclusive lock over every body in `ids` at once. Same
    /// borrowing rule as `lockMultiRead`.
    pub fn lockMultiWrite(self: PhysicsSystem, ids: []const body_mod.BodyId) body_mod.MultiLock {
        var lock: body_mod.MultiLock = .{ .raw = undefined, .write = true };
        c.zjoltBodyLockMultiWrite(self.handle, ids.ptr, @intCast(ids.len), &lock.raw);
        return lock;
    }
};
