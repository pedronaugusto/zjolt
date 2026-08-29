//! Hair: strand simulation, and the compute backend it runs on.
//!
//! Jolt's hair solver is compute shaders driven through an abstract
//! `ComputeSystem`. zjolt compiles none of Jolt's three GPU backends (a physics
//! package that cannot build without a graphics SDK is a graphics package), so
//! there are two ways to get one: `initCpu()` (no SDK, no device, on by
//! default) or a host-owned device via `ComputeBackend`/`ComputeSystem.init` —
//! everything past that point is identical. Per frame, after stepping:
//! `followBody`, `update`, `readBackRenderPositions`.
//!
//! Upstream calls the hair system "still in development" — no level of detail,
//! wind, or collision against anything but convex hulls. That is Jolt's own
//! assessment and it applies here unchanged.

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
pub const VertexState = c.HairVertexState;
pub const Info = c.HairInfo;

/// One velocity/density grid cell, as `Hair.lockReadBack` reports it: velocity
/// in the hair's local space per second, density relative to the rest pose.
pub const GridCell = c.HairGridCell;

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

/// What the solver reads from `gradient` at `strand_fraction` along a strand
/// (0 at the root, 1 at the tip). Outside the gradient's own range the value
/// clamps, not extrapolates; `strand_fraction` may be any finite value.
///
/// `min_fraction == max_fraction` is `error.InvalidArgument` — Jolt divides by
/// that difference unguarded; `Hair.init` refuses one too.
pub fn sampleGradient(gradient: Gradient, strand_fraction: f32) err.Error!f32 {
    var value: f32 = 0;
    try err.check(c.zjoltHairGradientSample(&gradient, strand_fraction, &value));
    return value;
}

/// The bend compliance the solver will use at `strand_fraction` of the way
/// along a strand: `bend_compliance` scaled by the four-entry multiplier curve,
/// interpolated across the thirds of the strand it names.
///
/// `strand_fraction` must be in [0, 1] — Jolt truncates it to an unsigned
/// index, so anything below 0 is undefined behaviour there rather than a clamp.
pub fn bendComplianceAt(material: Material, strand_fraction: f32) err.Error!f32 {
    var value: f32 = 0;
    try err.check(c.zjoltHairMaterialGetBendCompliance(
        &material,
        strand_fraction,
        &value,
    ));
    return value;
}

//=============================================================================
// Injecting a backend
//
// The whole table is callbacks, so the no-unwinding rule is not a formality: Jolt calls these from inside its own solver with exceptions off, and a panic escaping one leaves compute state half-recorded.
// The thunks below CATCH instead — stashed and re-raised by `check`; the callback reports failure to Jolt the only way it can: a non-OK result or a null handle.
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
/// `T` needs 14 create/destroy/map/queue methods (`interface()` lists which),
/// plus optional `createReadbackBuffer`/`destroy` with a fallback.
/// `name`/`data` are borrowed for the call; the value must not move once
/// `interface()` reaches `ComputeSystem.init` — the C side keeps a pointer.
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
        /// and clears it. A callback that failed did not stop Jolt — it
        /// returned, and Jolt carried on with whatever the failure left behind.
        /// Call after `Hair.init` and `Hair.update`, the two calls that reach
        /// the backend.
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

/// A grid cell index and its blend fraction toward the next cell on each
/// axis, as `Hair.positionToGridIndex` computes it.
pub const GridLocation = struct {
    index: [3]u32,
    fraction: math.Vec3,
};

/// A zero-copy window into a hair's last simulated state, opened by
/// `Hair.lockReadBack`. Every slice is valid only until `deinit`, and the
/// value itself must not outlive that call — it carries slices into
/// device-mapped memory, not memory of its own.
pub const ReadBack = struct {
    hair: *c.Hair,
    /// Empty for a scalp-less groom.
    scalp_vertices: []const math.Vec3,
    /// `grid_size_x * grid_size_y * grid_size_z` cells (`Hair.info`),
    /// x fastest, then y, then z.
    grid_velocity_and_density: []const GridCell,
    render_positions: []const math.Vec3,

    pub fn deinit(self: ReadBack) void {
        _ = c.zjoltHairUnlockReadBackBuffers(self.hair);
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

    /// Copies simulated vertex positions (hair LOCAL space) into `out`: the
    /// `simulation_strands_fraction` subset, in Jolt's assigned order, not
    /// insertion order. Use `readBackRenderPositions` for rendering.
    /// `error.BufferTooSmall` if `out` is shorter than `positionCount()`
    /// (prefix still written). SLOW: copies the whole simulation state from the
    /// device — Jolt calls this debugging-only; stalls on a GPU backend.
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

    /// How many scalp vertices `readBackScalpVertices` would write. 0 for a
    /// groom with no scalp.
    pub fn scalpVertexCount(self: Hair) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairReadBackScalpVertices(self.handle, null, 0, &count));
        return count;
    }

    /// The scalp mesh as the solver skinned it, in the hair's LOCAL space.
    ///
    /// This is what `setPose` actually did. Worth looking at when a groom sits
    /// in the wrong place and nothing else says why: the roots are barycentric
    /// points on these triangles, so a scalp that is not where the head is has
    /// no other symptom than hair that is not where the head is.
    pub fn readBackScalpVertices(self: Hair, out: []math.Vec3) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairReadBackScalpVertices(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return count;
    }

    /// Position, orientation, velocity and angular velocity of every simulated
    /// vertex, in ONE device stall. `out` wants `positionCount()` entries.
    /// Reach for this rather than `readBackPositions` plus anything else: each
    /// readback stalls on the whole simulation state. `rotation` is the
    /// vertex's Bishop frame, orienting a strand's cross-section — positions
    /// alone give a polyline with no ribbon-facing.
    pub fn readBackVertexState(self: Hair, out: []VertexState) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairReadBackVertexState(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return count;
    }

    /// How many strands `simulatedStrands` would write.
    pub fn simulatedStrandCount(self: Hair) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairGetSimulatedStrands(self.handle, null, 0, &count));
        return count;
    }

    /// Where each simulated strand starts and ends in what
    /// `readBackPositions`/`readBackVertexState` return, and which material it
    /// uses — without this those are a flat vertex run with no boundaries,
    /// since Jolt keeps only a fraction of authored strands, grouped by
    /// material. `readBackRenderPositions` needs no such call (already in the
    /// caller's own indexing). Cheap: reads the groom, not the device.
    pub fn simulatedStrands(self: Hair, out: []Strand) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairGetSimulatedStrands(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return count;
    }

    /// What `init` made of the groom — the counts, the grid, and the bounds
    /// Jolt derived, not the ones that went in. `max_root_distance_to_scalp` is
    /// the one to check first on an authoring-tool groom: Jolt projects every
    /// root onto the nearest scalp triangle instead of complaining, so a
    /// mismatched head silently attaches anyway.
    pub fn info(self: Hair) err.Error!Info {
        var out: Info = undefined;
        try err.check(c.zjoltHairGetInfo(self.handle, &out));
        return out;
    }

    /// Reads the device state back and locks it for zero-copy access —
    /// the velocity/density grid, which no copy-out call exposes, plus the
    /// scalp and render positions those calls copy out of anyway. Call
    /// `ReadBack.deinit` before doing anything else with this hair; a
    /// second `lockReadBack` before that is `error.InvalidArgument`, not a
    /// silent double-lock. Slow: a full device stall.
    pub fn lockReadBack(self: Hair) err.Error!ReadBack {
        const groom_info = try self.info();
        const scalp_count = try self.scalpVertexCount();

        var view: c.HairReadBackView = undefined;
        try err.check(c.zjoltHairLockReadBackBuffers(self.handle, &view));

        const grid_count = groom_info.grid_size_x * groom_info.grid_size_y * groom_info.grid_size_z;
        return .{
            .hair = self.handle,
            .scalp_vertices = if (view.scalp_vertices) |p| p[0..scalp_count] else &.{},
            .grid_velocity_and_density = if (view.grid_velocity_and_density) |p| p[0..grid_count] else &.{},
            .render_positions = if (view.render_positions) |p| p[0..groom_info.render_vertex_count] else &.{},
        };
    }

    /// The rest-pose density at grid cell `(x, y, z)` — how much of a
    /// strand's own weight already sits there before simulation runs. Same
    /// grid as `info().grid_size_*`; no device access.
    pub fn neutralDensity(self: Hair, x: u32, y: u32, z: u32) err.Error!f32 {
        var value: f32 = 0;
        try err.check(c.zjoltHairGetNeutralDensity(self.handle, x, y, z, &value));
        return value;
    }

    /// The velocity/density grid cell containing `position` (hair LOCAL
    /// space), and its blend fraction toward the next cell on each axis —
    /// index a `lockReadBack` view's `grid_velocity_and_density` with this
    /// to trilinearly sample it. `position` outside `info()`'s simulation
    /// bounds clamps to the nearest edge cell rather than failing.
    pub fn positionToGridIndex(self: Hair, position: math.Vec3) err.Error!GridLocation {
        var loc: GridLocation = undefined;
        try err.check(c.zjoltHairPositionToGridIndex(
            self.handle,
            &position,
            &loc.index[0],
            &loc.index[1],
            &loc.index[2],
            &loc.fraction,
        ));
        return loc;
    }

    /// Skins the scalp mesh to `joints` on the CPU, without touching the
    /// solver or device — the same math `update` runs after `setPose`,
    /// usable standalone to preview a pose. Same convention as `setPose`:
    /// `joints.len` must equal `jointCount()`. Refused for a scalp-less
    /// groom.
    pub fn skinScalpVertices(
        self: Hair,
        joint_to_hair: *const [16]f32,
        joints: []const [16]f32,
        out: []math.Vec3,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltHairSkinScalpVertices(
            self.handle,
            joint_to_hair,
            @ptrCast(joints.ptr),
            @intCast(joints.len),
            out.ptr,
            @intCast(out.len),
            &count,
        ));
        return count;
    }

    /// Bytes `saveGroom` needs.
    pub fn saveGroomSize(self: Hair) err.Error!usize {
        var size: usize = 0;
        try err.check(c.zjoltHairSaveGroom(self.handle, null, 0, &size));
        return size;
    }

    /// Writes the built groom into `buffer`, returning the slice written.
    /// `error.BufferTooSmall` if it does not fit; use `saveGroomSize` first.
    ///
    /// The groom, not the simulation: the strands, materials, rest frames, skin
    /// points and density grid as they were when the hair was built. Not where
    /// the hair is, not how it is moving, not the pose.
    pub fn saveGroom(self: Hair, buffer: []u8) err.Error![]u8 {
        var size: usize = 0;
        try err.check(c.zjoltHairSaveGroom(
            self.handle,
            buffer.ptr,
            buffer.len,
            &size,
        ));
        return buffer[0..size];
    }

    /// Allocates and bakes in one step.
    pub fn saveGroomAlloc(self: Hair, gpa: std.mem.Allocator) ![]u8 {
        const size = try self.saveGroomSize();
        const buffer = try gpa.alloc(u8, size);
        errdefer gpa.free(buffer);
        return try self.saveGroom(buffer);
    }

    /// Rebuilds a hair from `saveGroom` output, skipping the strand split, root
    /// matching, and density grid — everything `init` spends its time on. Only
    /// the compute buffers are allocated.
    ///
    /// `error.BadFormat`, creating nothing, for a blob from a different zjolt
    /// build, Jolt, or precision setting, or one truncated/damaged in storage.
    pub fn initFromGroom(
        compute: ComputeSystem,
        data: []const u8,
        placement: Placement,
    ) err.Error!Hair {
        var handle: *c.Hair = undefined;
        try err.check(c.zjoltHairCreateFromGroom(
            compute.handle,
            data.ptr,
            data.len,
            &placement.position,
            &placement.rotation,
            placement.object_layer,
            &handle,
        ));
        return .{ .handle = handle };
    }
};

/// Where a restored groom goes and what it collides with. `Groom` carries the
/// same three for a fresh one; they are not in the baked blob because they
/// describe the instance rather than the asset.
pub const Placement = struct {
    position: math.RVec3 = math.rvec3_zero,
    rotation: math.Quat = math.quat_identity,
    object_layer: system_mod.ObjectLayer = 0,
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

/// A groom of two strands of DIFFERENT lengths, which is what makes the padded
/// vertex count differ from the simulated one and the strand ranges worth
/// asking for. Everything but the rod solver is switched off, as above.
fn testGroom() struct { vertices: [7]Vertex, strands: [2]Strand, material: Material } {
    var material = defaultMaterial();
    material.enable_collision = false;
    material.bend_compliance = 1.0e-3;
    material.stretch_compliance = 1.0e-5;
    material.gravity_factor = .{ .min = 1, .max = 1, .min_fraction = 0, .max_fraction = 1 };
    material.global_pose = .{ .min = 0, .max = 0, .min_fraction = 0, .max_fraction = 1 };
    material.grid_velocity_factor = .{ .min = 0, .max = 0, .min_fraction = 0, .max_fraction = 1 };
    material.grid_density_force_factor = 0;
    material.simulation_strands_fraction = 1;

    return .{
        .vertices = .{
            .{ .position = math.vec3(0.0, 0, 0.0), .inv_mass = 0 },
            .{ .position = math.vec3(0.1, 0, 0.0), .inv_mass = 1 },
            .{ .position = math.vec3(0.2, 0, 0.0), .inv_mass = 1 },
            .{ .position = math.vec3(0.3, 0, 0.0), .inv_mass = 1 },
            .{ .position = math.vec3(0.0, 0, 0.1), .inv_mass = 0 },
            .{ .position = math.vec3(0.1, 0, 0.1), .inv_mass = 1 },
            .{ .position = math.vec3(0.2, 0, 0.1), .inv_mass = 1 },
        },
        .strands = .{
            .{ .start_vertex = 0, .end_vertex = 4, .material_index = 0 },
            .{ .start_vertex = 4, .end_vertex = 7, .material_index = 0 },
        },
        .material = material,
    };
}

test "the simulated set describes itself: strand ranges, padding and per-vertex state" {
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

    const groom = testGroom();
    const hair = try Hair.init(compute, .{
        .vertices = &groom.vertices,
        .strands = &groom.strands,
        .materials = &.{groom.material},
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 5, 6 },
        .object_layer = TestLayers.moving,
    });
    defer hair.deinit();

    const groom_info = try hair.info();
    try std.testing.expectEqual(@as(u32, 7), groom_info.simulated_vertex_count);
    try std.testing.expectEqual(@as(u32, 2), groom_info.simulated_strand_count);
    try std.testing.expectEqual(@as(u32, 7), groom_info.render_vertex_count);
    try std.testing.expectEqual(@as(u32, 2), groom_info.render_strand_count);
    try std.testing.expectEqual(@as(u32, 1), groom_info.material_count);
    try std.testing.expectEqual(@as(u32, 0), groom_info.joint_count);
    try std.testing.expectEqual(@as(u32, 4), groom_info.max_vertices_per_strand);

    // The device buffers are a rectangle of strands by longest strand, so the
    // three-vertex strand is padded out to four. Reading them without knowing
    // that reads one vertex of the next strand as the tail of this one.
    try std.testing.expectEqual(@as(u32, 8), groom_info.padded_vertex_count);
    try std.testing.expect(groom_info.padded_vertex_count > groom_info.simulated_vertex_count);

    try std.testing.expectEqual(@as(u32, 4), groom_info.grid_size_x);
    try std.testing.expectEqual(@as(u32, 5), groom_info.grid_size_y);
    try std.testing.expectEqual(@as(u32, 6), groom_info.grid_size_z);
    try std.testing.expect(groom_info.simulation_bounds_max.x > groom_info.simulation_bounds_min.x);
    try std.testing.expect(groom_info.simulation_bounds_max.y > groom_info.simulation_bounds_min.y);
    // No scalp, so nothing was matched to one.
    try std.testing.expectEqual(@as(f32, 0), groom_info.max_root_distance_to_scalp);

    // The strand ranges tile the simulated vertex array exactly: no gap for the
    // padding to show through, and no overlap.
    var strands: [2]Strand = undefined;
    try std.testing.expectEqual(@as(u32, 2), try hair.simulatedStrandCount());
    try std.testing.expectEqual(@as(u32, 2), try hair.simulatedStrands(&strands));
    try std.testing.expectEqual(@as(u32, 0), strands[0].start_vertex);
    try std.testing.expectEqual(strands[0].end_vertex, strands[1].start_vertex);
    try std.testing.expectEqual(
        groom_info.simulated_vertex_count,
        strands[1].end_vertex,
    );
    for (strands) |strand| {
        try std.testing.expectEqual(@as(u32, 0), strand.material_index);
        try std.testing.expect(strand.end_vertex - strand.start_vertex >= 2);
    }

    var step: u32 = 0;
    while (step < 30) : (step += 1) try hair.update(system, 1.0 / 60.0);

    var state: [7]VertexState = undefined;
    try std.testing.expectEqual(@as(u32, 7), try hair.readBackVertexState(&state));

    // One readback, four quantities, and the positions in it are the same ones
    // the positions-only readback reports.
    var positions: [7]math.Vec3 = undefined;
    _ = try hair.readBackPositions(&positions);
    for (state, positions) |s, p| {
        try std.testing.expectApproxEqAbs(p.x, s.position.x, 1.0e-5);
        try std.testing.expectApproxEqAbs(p.y, s.position.y, 1.0e-5);
        try std.testing.expectApproxEqAbs(p.z, s.position.z, 1.0e-5);
    }

    // Every rotation is a usable frame rather than whatever the buffer held.
    for (state) |s| {
        const length_sq = s.rotation.x * s.rotation.x + s.rotation.y * s.rotation.y +
            s.rotation.z * s.rotation.z + s.rotation.w * s.rotation.w;
        try std.testing.expectApproxEqAbs(@as(f32, 1), length_sq, 1.0e-3);
    }

    // A pinned root is not moving, and a tip swinging under gravity is — in
    // the direction it is about to go. Correlating the velocity with the next
    // step's displacement is what says these are velocities of these vertices,
    // rather than a plausible-looking read of some other buffer: a pendulum's
    // tip is somewhere different every step, so no fixed sign would hold.
    try hair.update(system, 1.0 / 60.0);
    var next: [7]math.Vec3 = undefined;
    _ = try hair.readBackPositions(&next);

    for (strands) |strand| {
        const root = state[strand.start_vertex];
        try std.testing.expectApproxEqAbs(@as(f32, 0), root.velocity.x, 1.0e-4);
        try std.testing.expectApproxEqAbs(@as(f32, 0), root.velocity.y, 1.0e-4);
        try std.testing.expectApproxEqAbs(@as(f32, 0), root.velocity.z, 1.0e-4);

        const t = strand.end_vertex - 1;
        const tip = state[t];
        const speed = @abs(tip.velocity.x) + @abs(tip.velocity.y) + @abs(tip.velocity.z);
        try std.testing.expect(speed > 1.0e-3);

        const dx = next[t].x - positions[t].x;
        const dy = next[t].y - positions[t].y;
        const dz = next[t].z - positions[t].z;
        const along = dx * tip.velocity.x + dy * tip.velocity.y + dz * tip.velocity.z;
        try std.testing.expect(along > 0);

        // And it is turning, because a rod that swings rotates its frame with
        // it — the quantity nothing but this readback reports.
        const spin = @abs(tip.angular_velocity.x) + @abs(tip.angular_velocity.y) +
            @abs(tip.angular_velocity.z);
        try std.testing.expect(spin > 1.0e-4);
    }
}

test "a baked groom rebuilds into the same hair without redoing the build" {
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

    const groom = testGroom();
    const baked = blk: {
        const built = try Hair.init(compute, .{
            .vertices = &groom.vertices,
            .strands = &groom.strands,
            .materials = &.{groom.material},
            .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
            .grid_size = .{ 4, 4, 4 },
            .object_layer = TestLayers.moving,
        });
        defer built.deinit();
        break :blk try built.saveGroomAlloc(std.testing.allocator);
    };
    defer std.testing.allocator.free(baked);
    try std.testing.expect(baked.len > 0);

    // The cook's hair is gone; this one never sees the authored strands.
    const restored = try Hair.initFromGroom(compute, baked, .{
        .object_layer = TestLayers.moving,
    });
    defer restored.deinit();

    const fresh = try Hair.init(compute, .{
        .vertices = &groom.vertices,
        .strands = &groom.strands,
        .materials = &.{groom.material},
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 4, 4 },
        .object_layer = TestLayers.moving,
    });
    defer fresh.deinit();

    // Same groom, field for field — the split, the padding and the grid all
    // came out of the blob rather than being recomputed.
    try std.testing.expectEqual(try fresh.info(), try restored.info());

    // And it simulates, which is the part a restored settings object could be
    // structurally right about and still get wrong: the rest frames and the
    // density grid are in the blob too.
    var step: u32 = 0;
    while (step < 30) : (step += 1) {
        try fresh.update(system, 1.0 / 60.0);
        try restored.update(system, 1.0 / 60.0);
    }

    var fresh_positions: [7]math.Vec3 = undefined;
    var restored_positions: [7]math.Vec3 = undefined;
    _ = try fresh.readBackPositions(&fresh_positions);
    _ = try restored.readBackPositions(&restored_positions);
    for (fresh_positions, restored_positions) |a, b| {
        try std.testing.expectApproxEqAbs(a.x, b.x, 1.0e-5);
        try std.testing.expectApproxEqAbs(a.y, b.y, 1.0e-5);
        try std.testing.expectApproxEqAbs(a.z, b.z, 1.0e-5);
    }

    // A blob that lost bytes, and one that kept its length and lost a byte's
    // worth of meaning, are both refused before Jolt reads any of it.
    try std.testing.expectError(
        error.BadFormat,
        Hair.initFromGroom(compute, baked[0 .. baked.len - 1], .{}),
    );
    const damaged = try std.testing.allocator.dupe(u8, baked);
    defer std.testing.allocator.free(damaged);
    damaged[damaged.len - 1] ^= 0xff;
    try std.testing.expectError(
        error.BadFormat,
        Hair.initFromGroom(compute, damaged, .{}),
    );
}

test "a scalp carries the roots, and the skinned scalp is what says so" {
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

    // One triangle in the XZ plane, one joint, one weight per vertex. The
    // strand roots sit on it, which is what Jolt matches them against.
    const scalp_vertices = [_]math.Vec3{
        math.vec3(-0.2, 0, -0.2),
        math.vec3(0.4, 0, -0.2),
        math.vec3(-0.2, 0, 0.4),
    };
    const scalp_triangles = [_][3]u32{.{ 0, 1, 2 }};
    const scalp_weights = [_]SkinWeight{
        .{ .joint_index = 0, .weight = 1 },
        .{ .joint_index = 0, .weight = 1 },
        .{ .joint_index = 0, .weight = 1 },
    };
    const identity = [16]f32{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

    const groom = testGroom();
    const hair = try Hair.init(compute, .{
        .vertices = &groom.vertices,
        .strands = &groom.strands,
        .materials = &.{groom.material},
        .scalp = .{
            .vertices = &scalp_vertices,
            .triangles = &scalp_triangles,
            .skin_weights = &scalp_weights,
            .weights_per_vertex = 1,
            .inverse_bind_pose = &.{identity},
        },
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 4, 4 },
        .object_layer = TestLayers.moving,
    });
    defer hair.deinit();

    try std.testing.expectEqual(@as(u32, 1), try hair.jointCount());
    const groom_info = try hair.info();
    try std.testing.expectEqual(@as(u32, 1), groom_info.joint_count);
    // Every root was already on the triangle, so Jolt moved none of them. A
    // groom authored against a different head reports the distance here instead
    // of failing anywhere.
    try std.testing.expect(groom_info.max_root_distance_to_scalp < 1.0e-3);

    try std.testing.expectEqual(@as(u32, 3), try hair.scalpVertexCount());

    // The rest pose skins the scalp to where it was modelled.
    try hair.setPose(&identity, &.{identity});
    try hair.update(system, 1.0 / 60.0);

    var skinned: [3]math.Vec3 = undefined;
    try std.testing.expectEqual(@as(u32, 3), try hair.readBackScalpVertices(&skinned));
    for (skinned, scalp_vertices) |got, want| {
        try std.testing.expectApproxEqAbs(want.x, got.x, 1.0e-4);
        try std.testing.expectApproxEqAbs(want.y, got.y, 1.0e-4);
        try std.testing.expectApproxEqAbs(want.z, got.z, 1.0e-4);
    }

    // Raise the one joint half a metre and the scalp goes with it — this is the
    // step between a pose and hair that moves, and the only place it is visible
    // before the strands have had time to react.
    var raised = identity;
    raised[13] = 0.5;
    try hair.setPose(&identity, &.{raised});
    try hair.update(system, 1.0 / 60.0);

    _ = try hair.readBackScalpVertices(&skinned);
    for (skinned, scalp_vertices) |got, want| {
        try std.testing.expectApproxEqAbs(want.y + 0.5, got.y, 1.0e-4);
    }

    // A groom with no scalp has none of this rather than failing for it.
    const bald = try Hair.init(compute, .{
        .vertices = &groom.vertices,
        .strands = &groom.strands,
        .materials = &.{groom.material},
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 4, 4 },
        .object_layer = TestLayers.moving,
    });
    defer bald.deinit();
    try std.testing.expectEqual(@as(u32, 0), try bald.scalpVertexCount());
}

test "the authored gradients and compliance curve evaluate the way the solver reads them" {
    var gpa = std.heap.DebugAllocator(.{}){};
    defer std.debug.assert(gpa.deinit() == .ok);

    var bridged = @import("memory.zig").bridge(gpa.allocator());
    var init_desc: c.InitDesc = .{ .allocator = &bridged };
    try std.testing.expectEqual(
        c.Result.ok,
        c.zjoltInitWithConfig(&init_desc, c.config_id),
    );
    defer c.zjoltDeinit();

    // Ramps between its two fractions and CLAMPS outside them, rather than
    // running on past the ends the way a plain lerp would.
    const ramp: Gradient = .{ .min = 0, .max = 1, .min_fraction = 0.2, .max_fraction = 0.8 };
    try std.testing.expectApproxEqAbs(@as(f32, 0), try sampleGradient(ramp, 0), 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), try sampleGradient(ramp, 0.2), 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), try sampleGradient(ramp, 0.5), 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), try sampleGradient(ramp, 0.8), 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), try sampleGradient(ramp, 1), 1.0e-6);

    // A descending gradient clamps to the same two values, not to min-then-max.
    const falling: Gradient = .{ .min = 1, .max = 0, .min_fraction = 0, .max_fraction = 1 };
    try std.testing.expectApproxEqAbs(@as(f32, 1), try sampleGradient(falling, -5), 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), try sampleGradient(falling, 5), 1.0e-6);

    // An empty fraction range is a division by zero inside Jolt, so it is
    // refused here and refused again by the groom that would carry it.
    const degenerate: Gradient = .{ .min = 0, .max = 1, .min_fraction = 0.5, .max_fraction = 0.5 };
    try std.testing.expectError(error.InvalidArgument, sampleGradient(degenerate, 0.5));
    try std.testing.expectError(
        error.InvalidArgument,
        sampleGradient(ramp, std.math.nan(f32)),
    );

    // bend_compliance_multiplier names the value at 0%, 33%, 66% and 100%, and
    // the ends of the strand are the ends of that array.
    var material = defaultMaterial();
    material.bend_compliance = 1.0e-3;
    material.bend_compliance_multiplier = .{ 1, 10, 10, 1 };
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0e-3),
        try bendComplianceAt(material, 0),
        1.0e-9,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0e-2),
        try bendComplianceAt(material, 1.0 / 3.0),
        1.0e-8,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0e-3),
        try bendComplianceAt(material, 1),
        1.0e-9,
    );
    // The middle third is flat between two equal multipliers.
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0e-2),
        try bendComplianceAt(material, 0.5),
        1.0e-8,
    );
    // Jolt truncates the fraction to an unsigned index, so out of range is
    // undefined behaviour there and refused here.
    try std.testing.expectError(error.InvalidArgument, bendComplianceAt(material, -0.1));
    try std.testing.expectError(error.InvalidArgument, bendComplianceAt(material, 1.1));

    if (!isCpuSupported()) return;

    const compute = try ComputeSystem.initCpu();
    defer compute.deinit();

    const groom = testGroom();
    var broken = groom.material;
    broken.gravity_factor = degenerate;
    try std.testing.expectError(error.InvalidArgument, Hair.init(compute, .{
        .vertices = &groom.vertices,
        .strands = &groom.strands,
        .materials = &.{broken},
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 4, 4 },
        .object_layer = TestLayers.moving,
    }));
}

test "Gradient.makeStepDependent reproduces Jolt's step reparameterization" {
    const g: Gradient = .{ .min = 0.5, .max = 0.8, .min_fraction = 0.1, .max_fraction = 0.9 };

    // One substep per default step changes nothing.
    const same = g.makeStepDependent(1.0);
    try std.testing.expectApproxEqAbs(g.min, same.min, 1.0e-6);
    try std.testing.expectApproxEqAbs(g.max, same.max, 1.0e-6);
    try std.testing.expectEqual(g.min_fraction, same.min_fraction);
    try std.testing.expectEqual(g.max_fraction, same.max_fraction);

    // A shorter substep decays less: 1 - (1-k)^0.5 < k for k in (0, 1).
    const half = g.makeStepDependent(0.5);
    try std.testing.expect(half.min < g.min);
    try std.testing.expect(half.max < g.max);
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0 - std.math.pow(f32, 1.0 - 0.5, 0.5)),
        half.min,
        1.0e-6,
    );
}

test "Strand.measureLength sums consecutive vertex distances" {
    const vertices = [_]Vertex{
        .{ .position = math.vec3(0, 0, 0), .inv_mass = 0 },
        .{ .position = math.vec3(3, 0, 0), .inv_mass = 1 },
        .{ .position = math.vec3(3, 4, 0), .inv_mass = 1 },
    };
    const strand: Strand = .{ .start_vertex = 0, .end_vertex = 3, .material_index = 0 };
    try std.testing.expectApproxEqAbs(@as(f32, 7), strand.measureLength(&vertices), 1.0e-5);
}

test "Material.needsGrid follows the grid-affecting fields" {
    var material = defaultMaterial();
    material.grid_velocity_factor = .{ .min = 0, .max = 0, .min_fraction = 0, .max_fraction = 1 };
    material.grid_density_force_factor = 0;
    try std.testing.expect(!material.needsGrid());

    material.grid_density_force_factor = 0.1;
    try std.testing.expect(material.needsGrid());

    material.grid_density_force_factor = 0;
    material.grid_velocity_factor.max = 0.2;
    try std.testing.expect(material.needsGrid());
}

test "the grid locates a position, and locking exposes its cells zero-copy" {
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

    const groom = testGroom();
    const hair = try Hair.init(compute, .{
        .vertices = &groom.vertices,
        .strands = &groom.strands,
        .materials = &.{groom.material},
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 4, 4 },
        .object_layer = TestLayers.moving,
    });
    defer hair.deinit();

    const groom_info = try hair.info();

    // The pinned root of a strand sits inside the simulation bounds, so it
    // lands on a real cell rather than clamping to an edge.
    const root = groom.vertices[0].position;
    const loc = try hair.positionToGridIndex(root);
    try std.testing.expect(loc.index[0] < groom_info.grid_size_x);
    try std.testing.expect(loc.index[1] < groom_info.grid_size_y);
    try std.testing.expect(loc.index[2] < groom_info.grid_size_z);
    try std.testing.expect(loc.fraction.x >= 0 and loc.fraction.x <= 1);
    try std.testing.expect(loc.fraction.y >= 0 and loc.fraction.y <= 1);
    try std.testing.expect(loc.fraction.z >= 0 and loc.fraction.z <= 1);

    // A strand's own weight is what fills the neutral density grid, so the
    // total across every cell is not zero.
    var total_density: f32 = 0;
    var x: u32 = 0;
    while (x < groom_info.grid_size_x) : (x += 1) {
        var y: u32 = 0;
        while (y < groom_info.grid_size_y) : (y += 1) {
            var z: u32 = 0;
            while (z < groom_info.grid_size_z) : (z += 1) {
                total_density += try hair.neutralDensity(x, y, z);
            }
        }
    }
    try std.testing.expect(total_density > 0);

    // A coordinate past the grid is refused rather than read out of bounds.
    try std.testing.expectError(
        error.InvalidArgument,
        hair.neutralDensity(groom_info.grid_size_x, 0, 0),
    );

    try hair.update(system, 1.0 / 60.0);

    {
        const readback = try hair.lockReadBack();
        defer readback.deinit();

        const grid_count = groom_info.grid_size_x * groom_info.grid_size_y * groom_info.grid_size_z;
        try std.testing.expectEqual(@as(usize, grid_count), readback.grid_velocity_and_density.len);
        try std.testing.expectEqual(
            @as(usize, groom_info.render_vertex_count),
            readback.render_positions.len,
        );
        // No scalp on this groom.
        try std.testing.expectEqual(@as(usize, 0), readback.scalp_vertices.len);

        for (readback.grid_velocity_and_density) |cell| {
            try std.testing.expect(std.math.isFinite(cell.velocity.x));
            try std.testing.expect(std.math.isFinite(cell.velocity.y));
            try std.testing.expect(std.math.isFinite(cell.velocity.z));
            try std.testing.expect(std.math.isFinite(cell.density));
            try std.testing.expect(cell.density >= 0);
        }

        // Locking again before unlocking is refused, not a silent double-lock.
        try std.testing.expectError(error.InvalidArgument, hair.lockReadBack());
    }

    // The window really did close: a second lock/unlock cycle works.
    const second = try hair.lockReadBack();
    second.deinit();
}

test "skinScalpVertices matches the solver's own skinning, off the device" {
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

    const scalp_vertices = [_]math.Vec3{
        math.vec3(-0.2, 0, -0.2),
        math.vec3(0.4, 0, -0.2),
        math.vec3(-0.2, 0, 0.4),
    };
    const scalp_triangles = [_][3]u32{.{ 0, 1, 2 }};
    const scalp_weights = [_]SkinWeight{
        .{ .joint_index = 0, .weight = 1 },
        .{ .joint_index = 0, .weight = 1 },
        .{ .joint_index = 0, .weight = 1 },
    };
    const identity = [16]f32{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

    const groom = testGroom();
    const hair = try Hair.init(compute, .{
        .vertices = &groom.vertices,
        .strands = &groom.strands,
        .materials = &.{groom.material},
        .scalp = .{
            .vertices = &scalp_vertices,
            .triangles = &scalp_triangles,
            .skin_weights = &scalp_weights,
            .weights_per_vertex = 1,
            .inverse_bind_pose = &.{identity},
        },
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 4, 4 },
        .object_layer = TestLayers.moving,
    });
    defer hair.deinit();

    var raised = identity;
    raised[13] = 0.5;

    // Skinned directly, with no update and no compute backend touched.
    var skinned_directly: [3]math.Vec3 = undefined;
    try std.testing.expectEqual(
        @as(u32, 3),
        try hair.skinScalpVertices(&identity, &.{raised}, &skinned_directly),
    );
    for (skinned_directly, scalp_vertices) |got, want| {
        try std.testing.expectApproxEqAbs(want.y + 0.5, got.y, 1.0e-4);
    }

    // The same pose, driven through the solver instead, agrees.
    try hair.setPose(&identity, &.{raised});
    try hair.update(system, 1.0 / 60.0);
    var skinned_by_solver: [3]math.Vec3 = undefined;
    _ = try hair.readBackScalpVertices(&skinned_by_solver);
    for (skinned_directly, skinned_by_solver) |direct, solver| {
        try std.testing.expectApproxEqAbs(direct.x, solver.x, 1.0e-4);
        try std.testing.expectApproxEqAbs(direct.y, solver.y, 1.0e-4);
        try std.testing.expectApproxEqAbs(direct.z, solver.z, 1.0e-4);
    }

    // A joint count that does not match the groom's inverse bind pose is
    // refused rather than indexed out of bounds.
    try std.testing.expectError(
        error.InvalidArgument,
        hair.skinScalpVertices(&identity, &.{ raised, raised }, &skinned_directly),
    );

    // A groom with no scalp has no skeleton to skin.
    const bald = try Hair.init(compute, .{
        .vertices = &groom.vertices,
        .strands = &groom.strands,
        .materials = &.{groom.material},
        .simulation_bounds_padding = math.vec3(0.5, 0.5, 0.5),
        .grid_size = .{ 4, 4, 4 },
        .object_layer = TestLayers.moving,
    });
    defer bald.deinit();
    try std.testing.expectError(
        error.InvalidArgument,
        bald.skinScalpVertices(&identity, &.{raised}, &skinned_directly),
    );
}
