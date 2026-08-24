//! The physics system: collision layers, listeners, the step, and bulk
//! read-back.

const std = @import("std");
const c = @import("c.zig");
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
// Which threads the step runs on is a host decision. v0.1 ships Jolt's own
// thread pool and its single-threaded scheduler; a host scheduler plugs in
// later through an additional constructor, without any call that takes a
// `JobSystem` changing shape.
//=============================================================================

pub const JobSystem = struct {
    handle: *c.JobSystem,

    pub const Options = struct {
        max_jobs: u32 = c.max_physics_jobs,
        max_barriers: u32 = c.max_physics_barriers,
        /// -1 means one worker per hardware thread minus the calling thread.
        num_threads: i32 = -1,
    };

    pub fn initThreadPool(opts: Options) err.Error!JobSystem {
        var handle: *c.JobSystem = undefined;
        try err.check(c.zjoltJobSystemCreateThreadPool(
            opts.max_jobs,
            opts.max_barriers,
            opts.num_threads,
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// Runs every job on the calling thread. Slower, but it makes a step
    /// reproducible without pinning a thread count — which is what the tests
    /// use, and what a deterministic replay would want.
    pub fn initSingleThreaded(max_jobs: u32) err.Error!JobSystem {
        var handle: *c.JobSystem = undefined;
        try err.check(c.zjoltJobSystemCreateSingleThreaded(max_jobs, &handle));
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
// Jolt asks the host three questions during the broad phase. This turns a
// plain Zig type whose declarations answer them into the three C tables the
// library wants — so a host writes no `callconv(.c)`, takes no address of a
// vtable, and needs no per-ABI branch. That last point is the whole reason the
// C boundary passes function pointers rather than C++ objects.
//=============================================================================

pub const Layers = struct {
    broad_phase: c.BroadPhaseLayerInterface,
    object_vs_broad_phase: c.ObjectVsBroadPhaseLayerFilter,
    object_layer_pair: c.ObjectLayerPairFilter,
};

/// Builds the three layer tables from a type declaring:
///
/// ```zig
/// pub fn broadPhaseLayerCount() u32
/// pub fn broadPhaseLayerFor(layer: ObjectLayer) BroadPhaseLayer
/// pub fn objectCanCollideWithBroadPhase(object: ObjectLayer, broad: BroadPhaseLayer) bool
/// pub fn objectsCanCollide(a: ObjectLayer, b: ObjectLayer) bool
/// ```
///
/// plus optionally `pub fn broadPhaseLayerName(layer: BroadPhaseLayer) [*:0]const u8`.
///
/// The functions are stateless, which is what a layer map almost always is: a
/// compile-time table. For a layer map that has to be built at run time, fill
/// in `Layers` by hand with a `user` pointer.
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
// IMPORTANT: contact callbacks run on Jolt's job threads, in parallel, inside
// `step`. They must be re-entrant and must not call back into the system. The
// usual shape is to append to a per-thread queue and drain it after the step
// returns.
//
// Activation callbacks are also raised during the step, but under the body
// manager's lock, so they are serialised.
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

/// Builds a contact listener from `context` and whichever of these `T`
/// declares — any it omits simply does not fire:
///
/// ```zig
/// pub fn onContactValidate(self: *T, info: *const ContactValidateInfo) ValidateResult
/// pub fn onContactAdded(self: *T, info: *const ContactInfo, settings: *ContactSettings) void
/// pub fn onContactPersisted(self: *T, info: *const ContactInfo, settings: *ContactSettings) void
/// pub fn onContactRemoved(self: *T, pair: *const SubShapeIdPair) void
/// ```
///
/// `context` must outlive the system it is installed on.
pub fn contactListener(comptime T: type, context: *T) c.ContactListener {
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
// Nothing may unwind out of a Jolt callback. Jolt compiles with exceptions
// off, so an exception crossing one is `std::terminate` — and a Zig panic
// unwinding out of a step listener or a combine callback skips the solver's
// lock destructors and PERMANENTLY DEADLOCKS the next `step`, silently and
// much later, with nothing to connect it to its cause.
//
// So the shims below catch. The error goes into the wrapper the host owns and
// is re-raised by `check` after the step returns, which is the point in the
// frame where a host can actually do something about it.
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

/// A host callback run once per collision step, before that step's collision
/// detection.
///
/// This is where to apply your own forces — buoyancy, thrusters, wind —
/// because it happens inside the sub-step loop, so a frame split into several
/// collision steps applies the force at each one rather than once for the
/// whole frame.
///
/// `T` declares `pub fn onStep(self: *T, ctx: *const StepListenerContext) void`
/// or the same returning `!void`. Every body and constraint mutex is held for
/// the duration, so it may READ and WRITE bodies but must not add or remove
/// them, and must not call back into the system.
///
/// Jolt does not call step listeners when there are no active bodies, or when
/// the step's delta time is zero. This is not a frame tick.
///
/// ```zig
/// var thrusters: Thrusters = .{};
/// var listener: zjolt.StepListener(Thrusters) = .init(&thrusters);
/// try listener.attach(system);
/// defer listener.detach(system) catch {};
///
/// _ = try system.step(dt, 1, jobs);
/// try listener.check();   // re-raises whatever onStep signalled
/// ```
///
/// The value must not move once attached: the C side holds a pointer to it.
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

/// A host answer to "what friction, or restitution, does this contact get".
///
/// Jolt's defaults are `sqrt(f1 * f2)` and `max(r1, r2)`; a host with a
/// material table usually wants its own. `T` declares
/// `pub fn combine(self: *T, info: *const CombineInfo) f32`, or the same
/// returning `!f32`.
///
/// This runs on Jolt's job threads DURING a step, once per contact, and is
/// subject to the same rules as a contact listener: re-entrant, no calls back
/// into the system. `info` carries both bodies' own values, so it needs
/// nothing else.
///
/// A callback that signals an error yields 0 for that contact — visibly wrong
/// rather than plausibly wrong — and the error comes back out of `check`.
///
/// The value must not move once attached.
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
        /// Scratch arena for one step. 0 picks 10 MiB. A step needing more
        /// falls back to the host allocator rather than failing.
        temp_allocator_size: usize = 0,
    };

    pub fn init(opts: Options) err.Error!PhysicsSystem {
        var desc: c.PhysicsSystemDesc = undefined;
        c.zjoltPhysicsSystemDescInit(&desc);
        desc.max_bodies = opts.max_bodies;
        desc.num_body_mutexes = opts.num_body_mutexes;
        desc.max_body_pairs = opts.max_body_pairs;
        desc.max_contact_constraints = opts.max_contact_constraints;
        if (opts.temp_allocator_size != 0) desc.temp_allocator_size = opts.temp_allocator_size;
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
    /// `error.InvalidArgument` for a non-positive batch size, batches per job
    /// or in-flight pair count, and for a zero velocity iteration count. Jolt
    /// asserts none of those — the first two are a loop stride and a divisor
    /// inside the step, so a zero there is a step that never finishes or a
    /// division by zero, several frames from the call that caused it.
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

    //-------------------------------------------------------------------------
    // The step
    //-------------------------------------------------------------------------

    /// Advances the simulation, returning the `UpdateError` mask.
    ///
    /// `collision_steps` splits the interval into sub-steps; 1 is normal, and
    /// raising it is the answer to fast bodies rather than shortening the
    /// frame.
    ///
    /// A non-empty `UpdateError` means contacts were dropped, not that the
    /// step failed. Jolt asserts that the mask is empty before returning it,
    /// so with asserts enabled the condition breaks into the debugger before
    /// you can read it — see UPSTREAM.md.
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
    // The per-body accessors are one ABI crossing and one lock each — right
    // for the occasional query, wrong for what a renderer does every frame.
    // These are the frame-loop path: two crossings and one lock acquisition
    // per batch instead of 2N of each.
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
};
