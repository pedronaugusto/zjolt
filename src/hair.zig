//! Hair: strand simulation, and the compute backend it runs on.
//!
//! Jolt's hair solver is written as compute shaders and driven through an
//! abstract `ComputeSystem`. zjolt compiles none of Jolt's three GPU
//! implementations, because a physics package that cannot build without a
//! graphics SDK is a graphics package. There are therefore two ways to get a
//! backend, and hair cannot tell them apart:
//!
//! ```zig
//! // No SDK, no device, no renderer. On by default; `options.cpu_compute`.
//! const compute = try zjolt.ComputeSystem.initCpu();
//! defer compute.deinit();
//!
//! const hair = try zjolt.Hair.init(compute, .{
//!     .vertices = &vertices,
//!     .strands = &strands,
//!     .object_layer = Layers.moving,
//! });
//! defer hair.deinit();
//!
//! // Per frame, after stepping the physics system:
//! try hair.followBody(system, head);
//! try hair.update(system, 1.0 / 60.0);
//! _ = try hair.readBackRenderPositions(&positions);
//! ```
//!
//! A host that already owns a device implements `ComputeBackend` instead and
//! passes its table to `ComputeSystem.init`. Everything below that point is
//! identical.
//!
//! Upstream calls the hair system "still in development" and lists what it does
//! not have yet — level of detail, wind, collision against anything but convex
//! hulls. That is Jolt's own assessment and it applies here unchanged.

const std = @import("std");
const c = @import("c/hair.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const system_mod = @import("system.zig");

//=============================================================================
// Plain types, shared with the C boundary
//=============================================================================

pub const BufferType = c.ComputeBufferType;
pub const MapMode = c.ComputeMapMode;
pub const Barrier = c.ComputeBarrier;

/// The raw function-pointer table. Build one with `ComputeBackend` rather than
/// filling it in by hand: every field is a callback, and a Zig error escaping
/// one of them across the C boundary is undefined behaviour, not an error.
pub const Interface = c.ComputeInterface;

pub const Vertex = c.HairVertex;
pub const Strand = c.HairStrand;
pub const Gradient = c.HairGradient;
pub const SkinWeight = c.HairSkinWeight;
pub const Material = c.HairMaterial;

/// Whether this build compiled Jolt's CPU compute path. False means
/// `ComputeSystem.initCpu` returns `error.Unsupported` and an injected backend
/// is the only way to run hair.
pub fn isCpuSupported() bool {
    return c.zjoltComputeIsCpuSupported();
}

/// Jolt's defaults for a hair material. Start here and change what you mean: a
/// zeroed `Material` is not a soft one, it is an inert one.
pub fn defaultMaterial() Material {
    var material: Material = undefined;
    c.zjoltHairMaterialInit(&material);
    return material;
}

//=============================================================================
// Injecting a backend
//
// The whole table is callbacks, which makes this the one place in the wrapper
// where the no-unwinding rule is not a formality. Jolt compiles without
// exceptions and calls these from inside its own solver; a Zig error returned
// across the boundary has nowhere to go, and a panic unwinding out of one
// leaves Jolt's compute state half-recorded.
//
// So the thunks below CATCH. The error is stashed in the wrapper the host owns
// and re-raised by `check`, which is the point in the frame where a host can
// actually do something about it. The callback itself reports failure to Jolt
// the only way the C ABI can: a non-OK result, or a null handle.
//=============================================================================

const ErrorInt = std.meta.Int(.unsigned, @bitSizeOf(anyerror));

/// The first error a callback signalled. First wins: the one that started the
/// trouble says more than the one that noticed it.
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

/// Wraps a host compute device as something Jolt can drive.
///
/// `T` declares these, and all of them are required:
///
/// ```zig
/// pub fn createShader(self: *T, name: []const u8, group_size: [3]u32) !*anyopaque
/// pub fn createBuffer(self: *T, kind: BufferType, count: u64, stride: u32, data: ?[]const u8) !*anyopaque
/// pub fn createQueue(self: *T) !*anyopaque
/// pub fn destroyShader(self: *T, shader: *anyopaque) void
/// pub fn destroyBuffer(self: *T, buffer: *anyopaque) void
/// pub fn destroyQueue(self: *T, queue: *anyopaque) void
/// pub fn mapBuffer(self: *T, buffer: *anyopaque, mode: MapMode) ![*]u8
/// pub fn unmapBuffer(self: *T, buffer: *anyopaque) void
/// pub fn queueSetShader(self: *T, queue: *anyopaque, shader: ?*anyopaque) void
/// pub fn queueSetConstantBuffer(self: *T, queue: *anyopaque, name: []const u8, buffer: ?*anyopaque) void
/// pub fn queueSetBuffer(self: *T, queue: *anyopaque, name: []const u8, buffer: ?*anyopaque) void
/// pub fn queueSetRwBuffer(self: *T, queue: *anyopaque, name: []const u8, buffer: ?*anyopaque, barrier: Barrier) void
/// pub fn queueDispatch(self: *T, queue: *anyopaque, groups: [3]u32) void
/// pub fn queueScheduleReadback(self: *T, queue: *anyopaque, dst: ?*anyopaque, src: ?*anyopaque) void
/// pub fn queueExecute(self: *T, queue: *anyopaque) void
/// pub fn queueWait(self: *T, queue: *anyopaque) void
/// ```
///
/// and may declare these two, each of which has a documented fallback:
///
/// ```zig
/// pub fn createReadbackBuffer(self: *T, buffer: *anyopaque) !*anyopaque
/// pub fn destroy(self: *T) void
/// ```
///
/// `name` and `data` are borrowed for the duration of the call. The value must
/// not move once its `interface()` has been handed to `ComputeSystem.init`: the
/// C side holds a pointer to it.
pub fn ComputeBackend(comptime T: type) type {
    return struct {
        const Self = @This();

        context: *T,
        failure: Failure = .{},

        pub fn init(context: *T) Self {
            return .{ .context = context };
        }

        /// The table to pass to `ComputeSystem.init`.
        pub fn interface(self: *Self) Interface {
            return .{
                .create_shader = Thunks.createShader,
                .create_buffer = Thunks.createBuffer,
                .create_readback_buffer = if (@hasDecl(T, "createReadbackBuffer"))
                    Thunks.createReadbackBuffer
                else
                    null,
                .create_queue = Thunks.createQueue,
                .destroy_shader = Thunks.destroyShader,
                .destroy_buffer = Thunks.destroyBuffer,
                .destroy_queue = Thunks.destroyQueue,
                .map_buffer = Thunks.mapBuffer,
                .unmap_buffer = Thunks.unmapBuffer,
                .queue_set_shader = Thunks.queueSetShader,
                .queue_set_constant_buffer = Thunks.queueSetConstantBuffer,
                .queue_set_buffer = Thunks.queueSetBuffer,
                .queue_set_rw_buffer = Thunks.queueSetRwBuffer,
                .queue_dispatch = Thunks.queueDispatch,
                .queue_schedule_readback = Thunks.queueScheduleReadback,
                .queue_execute = Thunks.queueExecute,
                .queue_wait = Thunks.queueWait,
                .destroy = if (@hasDecl(T, "destroy")) Thunks.destroy else null,
                .user = @ptrCast(self),
            };
        }

        /// Re-raises the first error a callback signalled since the last call,
        /// and clears it.
        ///
        /// A callback that failed did not stop Jolt — it returned, and Jolt
        /// carried on with whatever the failure left behind. Call this after
        /// `Hair.init` and after `Hair.update`, which are the two calls that
        /// reach the backend.
        pub fn check(self: *Self) anyerror!void {
            if (self.failure.take()) |e| return e;
        }

        const Thunks = struct {
            fn selfOf(user: ?*anyopaque) *Self {
                return @ptrCast(@alignCast(user.?));
            }

            fn createShader(
                user: ?*anyopaque,
                name: [*:0]const u8,
                gx: u32,
                gy: u32,
                gz: u32,
                out: *?*anyopaque,
            ) callconv(.c) c.Result {
                const self = selfOf(user);
                const handle = T.createShader(
                    self.context,
                    std.mem.span(name),
                    .{ gx, gy, gz },
                ) catch |e| {
                    self.failure.record(e);
                    return .invalid_argument;
                };
                out.* = handle;
                return .ok;
            }

            fn createBuffer(
                user: ?*anyopaque,
                kind: BufferType,
                count: u64,
                stride: u32,
                data: ?*const anyopaque,
                out: *?*anyopaque,
            ) callconv(.c) c.Result {
                const self = selfOf(user);
                const bytes: ?[]const u8 = if (data) |ptr|
                    @as([*]const u8, @ptrCast(ptr))[0..@intCast(count * stride)]
                else
                    null;
                const handle = T.createBuffer(self.context, kind, count, stride, bytes) catch |e| {
                    self.failure.record(e);
                    return .invalid_argument;
                };
                out.* = handle;
                return .ok;
            }

            fn createReadbackBuffer(
                user: ?*anyopaque,
                buffer: ?*anyopaque,
                out: *?*anyopaque,
            ) callconv(.c) c.Result {
                const self = selfOf(user);
                const handle = T.createReadbackBuffer(self.context, buffer.?) catch |e| {
                    self.failure.record(e);
                    return .invalid_argument;
                };
                out.* = handle;
                return .ok;
            }

            fn createQueue(user: ?*anyopaque, out: *?*anyopaque) callconv(.c) c.Result {
                const self = selfOf(user);
                const handle = T.createQueue(self.context) catch |e| {
                    self.failure.record(e);
                    return .invalid_argument;
                };
                out.* = handle;
                return .ok;
            }

            fn destroyShader(user: ?*anyopaque, shader: ?*anyopaque) callconv(.c) void {
                T.destroyShader(selfOf(user).context, shader orelse return);
            }

            fn destroyBuffer(user: ?*anyopaque, buffer: ?*anyopaque) callconv(.c) void {
                T.destroyBuffer(selfOf(user).context, buffer orelse return);
            }

            fn destroyQueue(user: ?*anyopaque, queue: ?*anyopaque) callconv(.c) void {
                T.destroyQueue(selfOf(user).context, queue orelse return);
            }

            /// Failure here is a null return, which is what the C ABI has —
            /// and which upstream will dereference. A backend that can fail to
            /// map should refuse at creation instead.
            fn mapBuffer(
                user: ?*anyopaque,
                buffer: ?*anyopaque,
                mode: MapMode,
            ) callconv(.c) ?*anyopaque {
                const self = selfOf(user);
                const mapped = T.mapBuffer(self.context, buffer orelse return null, mode) catch |e| {
                    self.failure.record(e);
                    return null;
                };
                return @ptrCast(mapped);
            }

            fn unmapBuffer(user: ?*anyopaque, buffer: ?*anyopaque) callconv(.c) void {
                T.unmapBuffer(selfOf(user).context, buffer orelse return);
            }

            fn queueSetShader(
                user: ?*anyopaque,
                queue: ?*anyopaque,
                shader: ?*anyopaque,
            ) callconv(.c) void {
                T.queueSetShader(selfOf(user).context, queue orelse return, shader);
            }

            fn queueSetConstantBuffer(
                user: ?*anyopaque,
                queue: ?*anyopaque,
                name: [*:0]const u8,
                buffer: ?*anyopaque,
            ) callconv(.c) void {
                T.queueSetConstantBuffer(
                    selfOf(user).context,
                    queue orelse return,
                    std.mem.span(name),
                    buffer,
                );
            }

            fn queueSetBuffer(
                user: ?*anyopaque,
                queue: ?*anyopaque,
                name: [*:0]const u8,
                buffer: ?*anyopaque,
            ) callconv(.c) void {
                T.queueSetBuffer(
                    selfOf(user).context,
                    queue orelse return,
                    std.mem.span(name),
                    buffer,
                );
            }

            fn queueSetRwBuffer(
                user: ?*anyopaque,
                queue: ?*anyopaque,
                name: [*:0]const u8,
                buffer: ?*anyopaque,
                barrier: Barrier,
            ) callconv(.c) void {
                T.queueSetRwBuffer(
                    selfOf(user).context,
                    queue orelse return,
                    std.mem.span(name),
                    buffer,
                    barrier,
                );
            }

            fn queueDispatch(
                user: ?*anyopaque,
                queue: ?*anyopaque,
                gx: u32,
                gy: u32,
                gz: u32,
            ) callconv(.c) void {
                T.queueDispatch(selfOf(user).context, queue orelse return, .{ gx, gy, gz });
            }

            fn queueScheduleReadback(
                user: ?*anyopaque,
                queue: ?*anyopaque,
                dst: ?*anyopaque,
                src: ?*anyopaque,
            ) callconv(.c) void {
                T.queueScheduleReadback(selfOf(user).context, queue orelse return, dst, src);
            }

            fn queueExecute(user: ?*anyopaque, queue: ?*anyopaque) callconv(.c) void {
                T.queueExecute(selfOf(user).context, queue orelse return);
            }

            fn queueWait(user: ?*anyopaque, queue: ?*anyopaque) callconv(.c) void {
                T.queueWait(selfOf(user).context, queue orelse return);
            }

            fn destroy(user: ?*anyopaque) callconv(.c) void {
                T.destroy(selfOf(user).context);
            }
        };
    };
}

//=============================================================================
// The backend handle
//=============================================================================

/// A compute backend, its one queue, and the fifteen hair shaders loaded on it.
/// Share one between every hair in a scene.
///
/// A hair holds its own reference on the backend, so `deinit` here and
/// `Hair.deinit` may happen in either order.
pub const ComputeSystem = struct {
    handle: *c.ComputeSystem,

    /// Jolt's own CPU implementation. `error.Unsupported` when this build was
    /// made with `-Dcpu_compute=false`; ask `isCpuSupported` first if that is a
    /// configuration you ship.
    ///
    /// Upstream describes it as a debugging aid and explicitly not optimised.
    /// It is what makes hair work with no graphics SDK in the build at all.
    pub fn initCpu() err.Error!ComputeSystem {
        var handle: *c.ComputeSystem = undefined;
        try err.check(c.zjoltComputeSystemCreateCpu(&handle));
        return .{ .handle = handle };
    }

    /// A backend the host implements. See `ComputeBackend`, which is what
    /// builds `iface` without hand-writing eighteen `callconv(.c)` shims.
    ///
    /// The fifteen hair shaders are loaded here, so a backend that cannot
    /// produce one of them fails at this call — with its name in `lastError` —
    /// rather than as a null dereference during the first step.
    pub fn init(iface: *const Interface) err.Error!ComputeSystem {
        var handle: *c.ComputeSystem = undefined;
        try err.check(c.zjoltComputeSystemCreate(iface, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: ComputeSystem) void {
        c.zjoltComputeSystemDestroy(self.handle);
    }
};

//=============================================================================
// Describing a groom
//=============================================================================

/// The mesh the roots of the strands are attached to, and the skinning that
/// moves it. Optional, and all-or-nothing: without it the roots are fixed in
/// the hair's local space and the whole groom moves with `setTransform`.
pub const Scalp = struct {
    vertices: []const math.Vec3,
    triangles: []const [3]u32,
    /// `vertices.len * weights_per_vertex` entries, in vertex order.
    skin_weights: []const SkinWeight,
    weights_per_vertex: u32,
    /// One column-major 4x4 matrix per joint.
    inverse_bind_pose: []const [16]f32,
};

/// A groom plus where to put it. Everything is copied by `Hair.init`; none of
/// these slices needs to outlive that call.
pub const Groom = struct {
    vertices: []const Vertex,
    strands: []const Strand,
    /// Empty uses one `defaultMaterial()`.
    materials: []const Material = &.{},
    scalp: ?Scalp = null,

    /// Gravity in the hair's local space, used once to work out the unloaded
    /// rest pose. Not the gravity the simulation runs under — that is the
    /// physics system's.
    initial_gravity: math.Vec3 = math.vec3_zero,
    /// Added on all sides of the neutral pose's bounds. All zero means Jolt's
    /// 0.1 rather than a literal zero: the velocity grid is scaled by
    /// `size / extent`, and a groom that is flat on one axis has a zero extent
    /// on it.
    simulation_bounds_padding: math.Vec3 = math.vec3_zero,
    /// Cells in the velocity/density grid. A zero uses Jolt's 32.
    grid_size: [3]u32 = .{ 0, 0, 0 },
    /// Solver iterations per second of simulated time. 0 uses Jolt's 360.
    iterations_per_second: u32 = 0,
    /// Longest step the solver takes; a longer one is clamped, which slows the
    /// hair down rather than letting it explode. 0 uses Jolt's 1/30.
    max_delta_time: f32 = 0,

    position: math.RVec3 = math.rvec3_zero,
    rotation: math.Quat = math.quat_identity,
    object_layer: system_mod.ObjectLayer = 0,

    fn toC(self: Groom) c.HairDesc {
        var desc: c.HairDesc = .{
            .vertices = self.vertices.ptr,
            .strands = self.strands.ptr,
            .materials = if (self.materials.len == 0) null else self.materials.ptr,
            .vertex_count = @intCast(self.vertices.len),
            .strand_count = @intCast(self.strands.len),
            .material_count = @intCast(self.materials.len),
            .scalp_vertices = null,
            .scalp_triangles = null,
            .scalp_skin_weights = null,
            .scalp_inverse_bind_pose = null,
            .scalp_vertex_count = 0,
            .scalp_triangle_count = 0,
            .skin_weights_per_vertex = 0,
            .joint_count = 0,
            .initial_gravity = self.initial_gravity,
            .simulation_bounds_padding = self.simulation_bounds_padding,
            .grid_size_x = self.grid_size[0],
            .grid_size_y = self.grid_size[1],
            .grid_size_z = self.grid_size[2],
            .iterations_per_second = self.iterations_per_second,
            .max_delta_time = self.max_delta_time,
            .position = self.position,
            .rotation = self.rotation,
            .object_layer = self.object_layer,
        };
        if (self.scalp) |scalp| {
            desc.scalp_vertices = scalp.vertices.ptr;
            desc.scalp_triangles = @ptrCast(scalp.triangles.ptr);
            desc.scalp_skin_weights = scalp.skin_weights.ptr;
            desc.scalp_inverse_bind_pose = @ptrCast(scalp.inverse_bind_pose.ptr);
            desc.scalp_vertex_count = @intCast(scalp.vertices.len);
            desc.scalp_triangle_count = @intCast(scalp.triangles.len);
            desc.skin_weights_per_vertex = scalp.weights_per_vertex;
            desc.joint_count = @intCast(scalp.inverse_bind_pose.len);
        }
        return desc;
    }
};

//=============================================================================
// A hair instance
//=============================================================================

pub const Hair = struct {
    handle: *c.Hair,

    /// Builds the groom and uploads it. This is the expensive call — it splits
    /// the strands into simulated and interpolated sets, computes rest frames,
    /// matches roots to the scalp and allocates every compute buffer. Do it at
    /// load time.
    pub fn init(compute: ComputeSystem, groom: Groom) err.Error!Hair {
        const desc = groom.toC();
        var handle: *c.Hair = undefined;
        try err.check(c.zjoltHairCreate(compute.handle, &desc, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: Hair) void {
        c.zjoltHairDestroy(self.handle);
    }

    /// Where the hair is in the world. The difference from the previous step is
    /// what drives the hair's inertia, so moving it is a physical act — use
    /// `teleport` when it is not meant to be.
    pub fn setTransform(self: Hair, position: math.RVec3, rotation: math.Quat) err.Error!void {
        try err.check(c.zjoltHairSetTransform(self.handle, &position, &rotation));
    }

    /// Where the hair is, as last set. This is the transform that takes the
    /// LOCAL-space vertices `getVertices` reports into world space — without
    /// it those vertices cannot be drawn, because Jolt's own `Hair` takes a
    /// transform and never gives it back.
    pub fn getTransform(self: Hair) err.Error!struct { position: math.RVec3, rotation: math.Quat } {
        var position: math.RVec3 = undefined;
        var rotation: math.Quat = undefined;
        try err.check(c.zjoltHairGetTransform(self.handle, &position, &rotation));
        return .{ .position = position, .rotation = rotation };
    }

    /// Takes the transform from a body, which is the usual way to hang hair on
    /// a head. `error.BodyNotFound` if the id names no body, rather than
    /// silently placing the hair at the origin.
    pub fn followBody(
        self: Hair,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
    ) err.Error!void {
        try err.check(c.zjoltHairFollowBody(self.handle, system.handle, body));
    }

    /// The skeleton pose that skins the scalp, and therefore the roots.
    ///
    /// `joint_to_hair` takes model space to the hair's local space; `joints`
    /// are the model-space joints, column-major, and must be exactly
    /// `jointCount()` of them — Jolt indexes its inverse bind pose by that with
    /// no bound of its own. A groom with no scalp has no joints and refuses.
    pub fn setPose(
        self: Hair,
        joint_to_hair: *const [16]f32,
        joints: []const [16]f32,
    ) err.Error!void {
        try err.check(c.zjoltHairSetPose(
            self.handle,
            joint_to_hair,
            @ptrCast(joints.ptr),
            @intCast(joints.len),
        ));
    }

    /// How many matrices `setPose` expects. 0 for a groom with no scalp.
    pub fn jointCount(self: Hair) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairGetJointCount(self.handle, &count));
        return count;
    }

    /// The next step puts the hair back in its default pose at rest, instead of
    /// treating the change in transform since the last step as motion.
    pub fn teleport(self: Hair) err.Error!void {
        try err.check(c.zjoltHairOnTeleported(self.handle));
    }

    /// Steps the hair and waits for the work to finish.
    ///
    /// `system` is read for gravity and for the shapes the strands collide
    /// with, so this belongs after `system.step` in a frame, not before. Jolt
    /// clamps `delta_time` to the groom's `max_delta_time` and divides what is
    /// left into whole solver iterations, so a very small step may do nothing.
    pub fn update(
        self: Hair,
        system: system_mod.PhysicsSystem,
        delta_time: f32,
    ) err.Error!void {
        try err.check(c.zjoltHairUpdate(self.handle, system.handle, delta_time));
    }

    /// How many simulated vertices `readBackPositions` would write.
    pub fn positionCount(self: Hair) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairReadBackPositions(self.handle, null, 0, &count));
        return count;
    }

    /// How many render vertices `readBackRenderPositions` would write.
    pub fn renderPositionCount(self: Hair) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairReadBackRenderPositions(self.handle, null, 0, &count));
        return count;
    }

    /// Copies the simulated vertex positions, in the hair's LOCAL space, into
    /// `out`, and returns how many were written.
    ///
    /// These are the vertices of the simulated strands only — the subset chosen
    /// by each material's `simulation_strands_fraction` — in the order Jolt
    /// assigned them, which is not the order they went in. For rendering, use
    /// `readBackRenderPositions`.
    ///
    /// `error.BufferTooSmall` when `out` is shorter than `positionCount()`; the
    /// prefix is still written. Upstream is blunt about the cost of this: it
    /// copies the whole simulation state back from the device and is "for
    /// debugging purposes only, this is slow!". On a GPU backend it stalls.
    pub fn readBackPositions(self: Hair, out: []math.Vec3) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairReadBackPositions(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return count;
    }

    /// The same, for the render vertices — every strand, including the ones
    /// that were interpolated rather than simulated.
    pub fn readBackRenderPositions(self: Hair, out: []math.Vec3) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairReadBackRenderPositions(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return count;
    }
};

//=============================================================================
// Tests
//=============================================================================

const TestLayers = struct {
    pub const moving: system_mod.ObjectLayer = 0;
    pub const bp_moving: system_mod.BroadPhaseLayer = 0;

    pub fn broadPhaseLayerCount() u32 {
        return 1;
    }
    pub fn broadPhaseLayerFor(_: system_mod.ObjectLayer) system_mod.BroadPhaseLayer {
        return bp_moving;
    }
    pub fn objectCanCollideWithBroadPhase(
        _: system_mod.ObjectLayer,
        _: system_mod.BroadPhaseLayer,
    ) bool {
        return true;
    }
    pub fn objectsCanCollide(_: system_mod.ObjectLayer, _: system_mod.ObjectLayer) bool {
        return true;
    }
};

test "hair stepped on the CPU compute backend moves" {
    if (!isCpuSupported()) return error.SkipZigTest;

    var gpa = std.heap.DebugAllocator(.{}){};
    defer std.debug.assert(gpa.deinit() == .ok);

    var bridged = @import("memory.zig").bridge(gpa.allocator());
    var init_desc: c.InitDesc = .{ .allocator = &bridged };
    try std.testing.expectEqual(
        c.Result.ok,
        c.zjoltInitWithConfig(&init_desc, c.config_id),
    );
    defer c.zjoltDeinit();

    const system = try system_mod.PhysicsSystem.init(.{
        .layers = system_mod.layersFromType(TestLayers),
        .max_bodies = 16,
    });
    defer system.deinit();
    system.setGravity(math.gravity_earth);

    const compute = try ComputeSystem.initCpu();
    defer compute.deinit();

    // One strand lying along +X with its root pinned, so gravity acts across it
    // rather than along it. A strand pointing the way gravity already pulls
    // would only stretch, and stretch compliance is 1e-8.
    const vertices = [_]Vertex{
        .{ .position = math.vec3(0.0, 0, 0), .inv_mass = 0 },
        .{ .position = math.vec3(0.1, 0, 0), .inv_mass = 1 },
        .{ .position = math.vec3(0.2, 0, 0), .inv_mass = 1 },
        .{ .position = math.vec3(0.3, 0, 0), .inv_mass = 1 },
    };
    const strands = [_]Strand{
        .{ .start_vertex = 0, .end_vertex = vertices.len, .material_index = 0 },
    };

    // Softened, and with everything that is not the rod solver switched off:
    // this test is about whether the CPU backend runs the solver at all, so
    // collision against an empty world and the velocity grid are noise.
    var material = defaultMaterial();
    material.enable_collision = false;
    material.bend_compliance = 1.0e-3;
    material.stretch_compliance = 1.0e-5;
    material.gravity_factor = .{ .min = 1, .max = 1, .min_fraction = 0, .max_fraction = 1 };
    material.global_pose = .{ .min = 0, .max = 0, .min_fraction = 0, .max_fraction = 1 };
    material.grid_velocity_factor = .{ .min = 0, .max = 0, .min_fraction = 0, .max_fraction = 1 };
    material.grid_density_force_factor = 0;
    material.simulation_strands_fraction = 1;

    const hair = try Hair.init(compute, .{
        .vertices = &vertices,
        .strands = &strands,
        .materials = &.{material},
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 4, 4 },
        .object_layer = TestLayers.moving,
    });
    defer hair.deinit();

    try std.testing.expectEqual(@as(u32, vertices.len), try hair.positionCount());

    // The first update is what teleports the hair into its default pose; the
    // position buffer holds nothing meaningful before it.
    try hair.update(system, 1.0 / 60.0);
    var before: [vertices.len]math.Vec3 = undefined;
    try std.testing.expectEqual(
        @as(u32, vertices.len),
        try hair.readBackPositions(&before),
    );

    var step: u32 = 0;
    while (step < 30) : (step += 1) try hair.update(system, 1.0 / 60.0);

    var after: [vertices.len]math.Vec3 = undefined;
    _ = try hair.readBackPositions(&after);

    for (after) |p| {
        try std.testing.expect(std.math.isFinite(p.x));
        try std.testing.expect(std.math.isFinite(p.y));
        try std.testing.expect(std.math.isFinite(p.z));
    }

    // The pinned root has not gone anywhere, and the free tip has fallen.
    try std.testing.expectApproxEqAbs(before[0].y, after[0].y, 1.0e-4);
    const tip = vertices.len - 1;
    try std.testing.expect(after[tip].y < before[tip].y - 1.0e-4);
    try std.testing.expect(after[tip].y < -1.0e-3);
}
