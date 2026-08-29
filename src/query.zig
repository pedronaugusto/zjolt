//! Ray casts, shape casts, overlap and point tests.
//!
//! Every query takes an optional `Filters`; null (or unset) accepts everything.
//!
//! Each query comes in three forms, all sharing one traversal:
//! `...Closest` (one hit); `count...`/`...All` (two-call protocol —
//! allocation-free from the caller's side, for a frame loop); `...Each`
//! (a callback per hit, nothing accumulated — cheapest for overlap
//! queries, where a hit is about a kilobyte inside Jolt).
//!
//! A hit callback runs *inside* Jolt's traversal, behind a C function
//! pointer, so it MUST NOT PANIC (no unwinding across that boundary). A
//! fallible `onHit`'s error is stashed and re-raised once the query
//! returns and every lock has dropped — write `try` and otherwise ignore it.

const std = @import("std");
const c = @import("c/query.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const shape_mod = @import("shape.zig");

/// Names a shape within a body's shape. `empty_sub_shape_id` is what a query
/// reports for a shape with no children, which is every shape this package can
/// build today.
pub const SubShapeId = c.SubShapeId;
pub const empty_sub_shape_id = c.sub_shape_id_empty;

pub const RayCastHit = c.RayCastHit;
pub const ShapeCastHit = c.ShapeCastHit;
pub const CollideShapeHit = c.CollideShapeHit;
pub const CollidePointHit = c.CollidePointHit;

pub const BroadPhaseLayerFilter = c.BroadPhaseLayerFilter;
pub const ObjectLayerFilter = c.ObjectLayerFilter;
pub const BodyFilter = c.BodyFilter;
pub const ShapeFilter = c.ShapeFilter;
pub const Filters = c.QueryFilters;

pub const HitAction = c.HitAction;
pub const RayCastSettings = c.RayCastSettings;

/// Jolt's collision settings, taken by every shape cast and every overlap in
/// this file. Every field defaults to Jolt's own value, so
/// `.{ .max_separation_distance = 1 }` changes one thing and leaves the rest.
pub const CollideShapeSettings = c.CollideShapeSettings;
pub const ShapeCastSettings = c.ShapeCastSettings;
pub const ActiveEdgeMode = c.ActiveEdgeMode;
pub const CollectFacesMode = c.CollectFacesMode;

pub const CollisionEstimationResult = c.CollisionEstimationResult;
pub const contact_points_capacity = c.contact_points_capacity;

//=============================================================================
// Ready-made filters
//=============================================================================

/// Rejects one body — the common case: a character that must not cast
/// against itself.
///
/// Borrows `self` (via `filters()`) rather than returning something
/// self-contained — do not return a `Filters` built from an argument
/// that then goes out of scope; keep `self` alive at the call site.
pub const ExcludeBody = struct {
    body: body_mod.BodyId,

    fn shouldCollide(user: ?*anyopaque, id: c.BodyId) callconv(.c) bool {
        const self: *const ExcludeBody = @ptrCast(@alignCast(user.?));
        return id != self.body;
    }

    pub fn filters(self: *const ExcludeBody) Filters {
        return .{ .body = .{
            .should_collide = shouldCollide,
            .user = @ptrCast(@constCast(self)),
        } };
    }
};

/// Accepts only one object layer. Same borrowing rule as `ExcludeBody`.
pub const OnlyObjectLayer = struct {
    layer: c.ObjectLayer,

    fn shouldCollide(user: ?*anyopaque, candidate: c.ObjectLayer) callconv(.c) bool {
        const self: *const OnlyObjectLayer = @ptrCast(@alignCast(user.?));
        return candidate == self.layer;
    }

    pub fn filters(self: *const OnlyObjectLayer) Filters {
        return .{ .object_layer = .{
            .should_collide = shouldCollide,
            .user = @ptrCast(@constCast(self)),
        } };
    }
};

//=============================================================================
// The callback bridge
//
// One generic shim between a Zig `onHit` and Jolt's C function pointer —
// it exists to carry a Zig error back out, which a plain wrapper cannot.
//=============================================================================

fn ContextOf(comptime Ptr: type) type {
    return @typeInfo(Ptr).pointer.child;
}

/// The C-side callback for a `Context` declaring `onHit(*Context, Hit)`.
///
/// `onHit` may return `HitAction` or `E!HitAction`. When it can fail, the
/// error is stashed in the shim and the traversal is stopped; `raise` hands it
/// back afterwards. @see the module comment for why it cannot simply be
/// returned.
fn Visitor(comptime Hit: type, comptime Context: type) type {
    const Returns = @typeInfo(@TypeOf(Context.onHit)).@"fn".return_type.?;
    const fallible = @typeInfo(Returns) == .error_union;

    return struct {
        const Self = @This();

        /// Empty when `onHit` cannot fail, which makes `Error!void` collapse
        /// to `void` in the query's own return type.
        pub const Error = if (fallible)
            @typeInfo(Returns).error_union.error_set
        else
            error{};

        context: *Context,
        failed: ?Error = null,

        pub fn onHit(user: ?*anyopaque, hit: *const Hit) callconv(.c) c.HitAction {
            const self: *Self = @ptrCast(@alignCast(user.?));
            if (comptime !fallible) return self.context.onHit(hit.*);
            // Jolt stops as soon as it can see the answer rather than
            // instantly, so guard against being asked again: a callback that
            // has already failed is not asked a second time, and the first
            // error is the one that survives.
            if (self.failed != null) return .stop;
            return self.context.onHit(hit.*) catch |e| {
                self.failed = e;
                return .stop;
            };
        }

        pub fn raise(self: *const Self) Error!void {
            if (comptime !fallible) return;
            if (self.failed) |e| return e;
        }

        /// The C-side callback for a `Context` additionally declaring
        /// `onBody`, wired up only by the `*WithBody` methods below.
        ///
        /// Called once per body, strictly before that body's hits reach
        /// `onHit`. Infallible: a context that needs to fail from here
        /// stashes it and returns an error from the next `onHit` instead.
        pub fn onBody(user: ?*anyopaque, body: *const c.Body) callconv(.c) void {
            const self: *Self = @ptrCast(@alignCast(user.?));
            self.context.onBody(.{ .handle = @constCast(body) });
        }
    };
}

/// What a streaming query returns: this package's errors, plus whatever the
/// caller's `onHit` can raise.
fn StreamError(comptime Hit: type, comptime ContextPtr: type) type {
    return err.Error || Visitor(Hit, ContextOf(ContextPtr)).Error;
}

/// A pointer to an optional's payload, or null. Written once because every
/// nullable-by-pointer argument in this file needs it, and writing it inline
/// invites taking the address of a temporary.
fn optionalPtr(comptime T: type, value: *const ?T) ?*const T {
    return if (value.*) |*payload| payload else null;
}

pub const Queries = struct {
    handle: *const c.PhysicsSystem,

    //=========================================================================
    // Ray casts
    //=========================================================================

    /// `direction` carries the ray's length: nothing beyond it is reported.
    /// The hit point is `origin + hit.fraction * direction`.
    ///
    /// `settings` may be null for Jolt's defaults, which ignore back faces and
    /// treat a convex shape as solid.
    pub fn castRayClosest(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
    ) err.Error!?RayCastHit {
        var hit: RayCastHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCastRayClosest(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    /// Number of hits along the ray, without collecting them.
    pub fn countRayHits(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCastRayAll(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Every hit along the ray, unsorted, written into `buffer`.
    /// `error.BufferTooSmall` if it does not fit; use `countRayHits` first, or
    /// `castRayEach` and skip the buffer entirely.
    pub fn castRayAll(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
        buffer: []RayCastHit,
    ) err.Error![]RayCastHit {
        var count: u32 = 0;
        try err.check(c.zjoltCastRayAll(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// Visits every hit along the ray as it is found, in one traversal
    /// and with no buffer. `context` declares `onHit(*T, RayCastHit)
    /// HitAction` (or `!HitAction`). `.stop` ends the query; `.narrow`
    /// asks for only nearer hits from here on — expect FEWER hits, not
    /// sorted ones (the broad phase already runs roughly front to back).
    pub fn castRayEach(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(RayCastHit, @TypeOf(context))!void {
        const V = Visitor(RayCastHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCastRayEach(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    //=========================================================================
    // Shape casts
    //=========================================================================

    pub const ShapeCast = struct {
        shape: shape_mod.Shape,
        position: math.RVec3,
        rotation: math.Quat = math.quat_identity,
        direction: math.Vec3,
        /// Null means (1, 1, 1).
        scale: ?math.Vec3 = null,
        /// Null takes Jolt's defaults, which IGNORE back faces — so a sweep
        /// beginning inside geometry finds nothing there at all. Set
        /// `back_face_mode_convex` or `back_face_mode_triangles` to `.collide`
        /// when the question is "is this placement clear", rather than
        /// concluding from an empty result that it is.
        settings: ?ShapeCastSettings = null,
    };

    /// Sweeps a shape along `direction`. The centre of mass at the hit is
    /// `position + hit.fraction * direction`.
    ///
    /// Contact points come back RELATIVE TO `position` — they are floats, and
    /// in a double-precision world an absolute contact point would not survive
    /// the conversion. Add `position` back if world space is what you want.
    pub fn castShapeClosest(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
    ) err.Error!?ShapeCastHit {
        var hit: ShapeCastHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCastShapeClosest(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    pub fn countShapeCastHits(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCastShapeAll(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    pub fn castShapeAll(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
        buffer: []ShapeCastHit,
    ) err.Error![]ShapeCastHit {
        var count: u32 = 0;
        try err.check(c.zjoltCastShapeAll(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// Every hit along the sweep, streamed. @see `castRayEach`.
    pub fn castShapeEach(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(ShapeCastHit, @TypeOf(context))!void {
        const V = Visitor(ShapeCastHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCastShapeEach(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    //=========================================================================
    // Overlap
    //=========================================================================

    pub const Overlap = struct {
        shape: shape_mod.Shape,
        position: math.RVec3,
        rotation: math.Quat = math.quat_identity,
        scale: ?math.Vec3 = null,
        /// Null takes Jolt's defaults. `max_separation_distance` lives here,
        /// not beside `scale`, since it is also a field of
        /// `CollideShapeSettings`, and an overlap cannot be told two
        /// different things about how far a near miss counts.
        settings: ?CollideShapeSettings = null,
    };

    /// The single deepest overlap — the hit with the largest
    /// `penetration_depth`. As well-defined as `castShapeClosest`'s nearest
    /// hit: Jolt orders overlaps by depth the same way it orders casts by
    /// distance. There is no equivalent for `collidePoint`, because every
    /// point hit ties (`GetEarlyOutFraction` is a constant there) and
    /// "closest" would just mean "whichever the traversal visited first".
    pub fn collideShapeClosest(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!?CollideShapeHit {
        var hit: CollideShapeHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCollideShapeClosest(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    pub fn countCollideShapeHits(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCollideShapeAll(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Everything overlapping the shape at the given placement.
    pub fn collideShape(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        buffer: []CollideShapeHit,
    ) err.Error![]CollideShapeHit {
        var count: u32 = 0;
        try err.check(c.zjoltCollideShapeAll(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// Everything overlapping the shape, streamed. This is the form worth
    /// reaching for: an overlap hit carries two 32-vertex face arrays inside
    /// Jolt, and the streaming path never accumulates them.
    /// @see `castRayEach`.
    pub fn collideShapeEach(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollideShapeHit, @TypeOf(context))!void {
        const V = Visitor(CollideShapeHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollideShapeEach(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    //=========================================================================
    // Overlap, with internal edge removal
    //
    // Same four forms as the plain overlap family, over the same `Overlap` — but forces `active_edge_mode`/`collect_faces_mode` regardless of what `overlap.settings` carries, so a slide across a mesh does not catch on seams.
    // Hits can arrive later, and out of the order a plain query finds them in: Jolt buffers every hit on one body's mesh, sorts deepest first.
    //=========================================================================

    pub fn collideShapeWithInternalEdgeRemovalClosest(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!?CollideShapeHit {
        var hit: CollideShapeHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalClosest(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    pub fn countCollideShapeWithInternalEdgeRemovalHits(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalAll(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    pub fn collideShapeWithInternalEdgeRemoval(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        buffer: []CollideShapeHit,
    ) err.Error![]CollideShapeHit {
        var count: u32 = 0;
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalAll(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    pub fn collideShapeWithInternalEdgeRemovalEach(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollideShapeHit, @TypeOf(context))!void {
        const V = Visitor(CollideShapeHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalEach(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    //=========================================================================
    // Predicting a contact's response, without running a step
    //=========================================================================

    /// The manifold input `estimateCollisionResponse` needs, cut down to
    /// exactly what Jolt's own algorithm reads from one: not a penetration
    /// depth, not a sub-shape id. `points_on_1` and `points_on_2` must be the
    /// same length -- at least 1 and at most `contact_points_capacity` --
    /// and are relative to `base_offset` the same way a `ShapeCastHit`'s
    /// contact points are.
    pub const EstimateManifold = struct {
        base_offset: math.RVec3 = math.rvec3_zero,
        world_space_normal: math.Vec3,
        points_on_1: []const math.Vec3,
        points_on_2: []const math.Vec3,
    };

    /// Predicts the impulses and post-collision velocities of a contact
    /// WITHOUT running a step — sizes an impact response (sound, debris)
    /// from inside a contact callback. `body1`/`body2` must already be
    /// safely readable without a lock — true inside a `ContactListener`
    /// callback; unsafe elsewhere is on the caller. `error.InvalidArgument`/`error.BodyNotFound` for bad slices/a dead id.
    pub fn estimateCollisionResponse(
        self: Queries,
        body1: body_mod.BodyId,
        body2: body_mod.BodyId,
        manifold: EstimateManifold,
        combined_friction: f32,
        combined_restitution: f32,
        min_velocity_for_restitution: f32,
        num_iterations: u32,
    ) err.Error!CollisionEstimationResult {
        std.debug.assert(manifold.points_on_1.len == manifold.points_on_2.len);
        const c_manifold: c.CollisionEstimationManifold = .{
            .base_offset = manifold.base_offset,
            .world_space_normal = manifold.world_space_normal,
            .num_points = @intCast(manifold.points_on_1.len),
            .points_on_1 = manifold.points_on_1.ptr,
            .points_on_2 = manifold.points_on_2.ptr,
        };
        var result: CollisionEstimationResult = undefined;
        try err.check(c.zjoltEstimateCollisionResponse(
            self.handle,
            body1,
            body2,
            &c_manifold,
            combined_friction,
            combined_restitution,
            min_velocity_for_restitution,
            num_iterations,
            &result,
        ));
        return result;
    }

    //=========================================================================
    // Point
    //
    // Every shape the point is inside, treated as solid. A mesh answers
    // usefully only if it is a closed manifold — an open mesh has no inside.
    //=========================================================================

    pub fn countPointHits(
        self: Queries,
        point: math.RVec3,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCollidePointAll(
            self.handle,
            &point,
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    pub fn collidePoint(
        self: Queries,
        point: math.RVec3,
        filters: ?*const Filters,
        buffer: []CollidePointHit,
    ) err.Error![]CollidePointHit {
        var count: u32 = 0;
        try err.check(c.zjoltCollidePointAll(
            self.handle,
            &point,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// Every shape containing the point, streamed. `.narrow` does nothing
    /// here — point hits have no distance to be nearer by — but `.stop`
    /// answers "is this point inside anything" without visiting the rest.
    /// @see `castRayEach`.
    pub fn collidePointEach(
        self: Queries,
        point: math.RVec3,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollidePointHit, @TypeOf(context))!void {
        const V = Visitor(CollidePointHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollidePointEach(
            self.handle,
            &point,
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    //=========================================================================
    // Observing a hit's body before the hit
    //
    // Stops at *Each — *Closest/*All run no host code during the
    // traversal for a per-body notification to reach. @see ffi/zjolt_query.h.
    //=========================================================================

    /// `castRayEach`, with `context` additionally declaring
    /// `onBody(*T, zjolt.Body) void` — called once per body, strictly
    /// before any of that body's hits reach `onHit`. `body` is a
    /// borrowed view: it lives no longer than the call — a `Body` kept
    /// past `onBody` returning is a dangling handle. Copy out what's needed.
    pub fn castRayEachWithBody(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(RayCastHit, @TypeOf(context))!void {
        const V = Visitor(RayCastHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCastRayEachWithBody(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            V.onHit,
            &visitor,
            V.onBody,
            &visitor,
        ));
        try visitor.raise();
    }

    /// @see `castRayEachWithBody`.
    pub fn castShapeEachWithBody(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(ShapeCastHit, @TypeOf(context))!void {
        const V = Visitor(ShapeCastHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCastShapeEachWithBody(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            V.onHit,
            &visitor,
            V.onBody,
            &visitor,
        ));
        try visitor.raise();
    }

    /// @see `castRayEachWithBody`.
    pub fn collideShapeEachWithBody(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollideShapeHit, @TypeOf(context))!void {
        const V = Visitor(CollideShapeHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollideShapeEachWithBody(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            V.onHit,
            &visitor,
            V.onBody,
            &visitor,
        ));
        try visitor.raise();
    }

    /// @see `castRayEachWithBody`.
    pub fn collideShapeWithInternalEdgeRemovalEachWithBody(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollideShapeHit, @TypeOf(context))!void {
        const V = Visitor(CollideShapeHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalEachWithBody(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            V.onHit,
            &visitor,
            V.onBody,
            &visitor,
        ));
        try visitor.raise();
    }

    /// @see `castRayEachWithBody`.
    pub fn collidePointEachWithBody(
        self: Queries,
        point: math.RVec3,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollidePointHit, @TypeOf(context))!void {
        const V = Visitor(CollidePointHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollidePointEachWithBody(
            self.handle,
            &point,
            filters,
            V.onHit,
            &visitor,
            V.onBody,
            &visitor,
        ));
        try visitor.raise();
    }

    //=========================================================================
    // The unlocked path
    //
    // For inside a callback where locks are already held (contact/combine
    // listener, step listener); USE WITH GREAT CARE — nothing stops another thread mutating a body one of these reads while it reads it.
    //=========================================================================

    /// @see `castRayClosest`.
    pub fn castRayClosestNoLock(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
    ) err.Error!?RayCastHit {
        var hit: RayCastHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCastRayClosestNoLock(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    /// @see `countRayHits`.
    pub fn countRayHitsNoLock(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCastRayAllNoLock(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// @see `castRayAll`.
    pub fn castRayAllNoLock(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
        buffer: []RayCastHit,
    ) err.Error![]RayCastHit {
        var count: u32 = 0;
        try err.check(c.zjoltCastRayAllNoLock(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// @see `castRayEach`.
    pub fn castRayEachNoLock(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(RayCastHit, @TypeOf(context))!void {
        const V = Visitor(RayCastHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCastRayEachNoLock(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    /// @see `castShapeClosest`.
    pub fn castShapeClosestNoLock(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
    ) err.Error!?ShapeCastHit {
        var hit: ShapeCastHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCastShapeClosestNoLock(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    /// @see `countShapeCastHits`.
    pub fn countShapeCastHitsNoLock(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCastShapeAllNoLock(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// @see `castShapeAll`.
    pub fn castShapeAllNoLock(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
        buffer: []ShapeCastHit,
    ) err.Error![]ShapeCastHit {
        var count: u32 = 0;
        try err.check(c.zjoltCastShapeAllNoLock(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// @see `castShapeEach`.
    pub fn castShapeEachNoLock(
        self: Queries,
        cast: ShapeCast,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(ShapeCastHit, @TypeOf(context))!void {
        const V = Visitor(ShapeCastHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCastShapeEachNoLock(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            optionalPtr(ShapeCastSettings, &cast.settings),
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    /// @see `collideShapeClosest`.
    pub fn collideShapeClosestNoLock(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!?CollideShapeHit {
        var hit: CollideShapeHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCollideShapeClosestNoLock(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    /// @see `countCollideShapeHits`.
    pub fn countCollideShapeHitsNoLock(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCollideShapeAllNoLock(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// @see `collideShape`.
    pub fn collideShapeNoLock(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        buffer: []CollideShapeHit,
    ) err.Error![]CollideShapeHit {
        var count: u32 = 0;
        try err.check(c.zjoltCollideShapeAllNoLock(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// @see `collideShapeEach`.
    pub fn collideShapeEachNoLock(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollideShapeHit, @TypeOf(context))!void {
        const V = Visitor(CollideShapeHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollideShapeEachNoLock(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    /// @see `collideShapeWithInternalEdgeRemovalClosest`.
    pub fn collideShapeWithInternalEdgeRemovalClosestNoLock(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!?CollideShapeHit {
        var hit: CollideShapeHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalClosestNoLock(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    /// @see `countCollideShapeWithInternalEdgeRemovalHits`.
    pub fn countCollideShapeWithInternalEdgeRemovalHitsNoLock(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalAllNoLock(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// @see `collideShapeWithInternalEdgeRemoval`.
    pub fn collideShapeWithInternalEdgeRemovalNoLock(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        buffer: []CollideShapeHit,
    ) err.Error![]CollideShapeHit {
        var count: u32 = 0;
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalAllNoLock(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// @see `collideShapeWithInternalEdgeRemovalEach`.
    pub fn collideShapeWithInternalEdgeRemovalEachNoLock(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollideShapeHit, @TypeOf(context))!void {
        const V = Visitor(CollideShapeHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollideShapeWithInternalEdgeRemovalEachNoLock(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            optionalPtr(CollideShapeSettings, &overlap.settings),
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    /// @see `countPointHits`.
    pub fn countPointHitsNoLock(
        self: Queries,
        point: math.RVec3,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCollidePointAllNoLock(
            self.handle,
            &point,
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// @see `collidePoint`.
    pub fn collidePointNoLock(
        self: Queries,
        point: math.RVec3,
        filters: ?*const Filters,
        buffer: []CollidePointHit,
    ) err.Error![]CollidePointHit {
        var count: u32 = 0;
        try err.check(c.zjoltCollidePointAllNoLock(
            self.handle,
            &point,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// @see `collidePointEach`.
    pub fn collidePointEachNoLock(
        self: Queries,
        point: math.RVec3,
        filters: ?*const Filters,
        context: anytype,
    ) StreamError(CollidePointHit, @TypeOf(context))!void {
        const V = Visitor(CollidePointHit, ContextOf(@TypeOf(context)));
        var visitor: V = .{ .context = context };
        try err.check(c.zjoltCollidePointEachNoLock(
            self.handle,
            &point,
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }
};

//=============================================================================
// Shape versus shape
//
// Two shapes, two placements, no `Queries`/physics system — placement
// validation, an editor's drag handle, a sweep against non-world geometry. `hit.body` is `invalid_body_id`; no `...Each` form (leaf-bounded, not broad-phase-bounded).
//=============================================================================

/// A shape and where it sits, for the shape-versus-shape queries.
pub const PlacedShape = struct {
    /// Borrowed for the call; nothing here takes a reference on it.
    shape: shape_mod.Shape,
    /// The shape's own world placement, NOT its centre of mass — Jolt collides
    /// in centre-of-mass space and the conversion happens on the C side, so a
    /// shape built with `Shape.initOffsetCenterOfMass` sits where it is put.
    position: math.RVec3,
    rotation: math.Quat = math.quat_identity,
    /// Null means (1, 1, 1). A scale the shape cannot take — a zero component,
    /// or a non-uniform one on a sphere — is `error.InvalidArgument` rather
    /// than an abort inside Jolt. `Shape.isValidScale` asks the same question
    /// ahead of time.
    scale: ?math.Vec3 = null,
};

/// Two placed shapes, and the frame their contact points come back in.
pub const ShapePair = struct {
    first: PlacedShape,
    second: PlacedShape,
    /// Contact points come back RELATIVE TO this; zero gives world space.
    /// They are floats, so in a double-precision build a contact point far
    /// from the origin does not survive the conversion — passing one of the
    /// two positions here is what keeps the precision.
    base_offset: math.RVec3 = math.rvec3_zero,
};

/// A shape swept against another, both placed, neither in a world.
pub const ShapeCastPair = struct {
    /// The shape that moves.
    cast: PlacedShape,
    /// Carries the sweep's length: nothing beyond it is reported.
    direction: math.Vec3,
    /// The shape that does not.
    target: PlacedShape,
    /// @see `ShapePair.base_offset`.
    base_offset: math.RVec3 = math.rvec3_zero,
};

/// The deepest overlap between the two shapes, or null.
///
/// `error.Unsupported` when Jolt has no collision function for the pair
/// (two surfaces — mesh, height field, plane, soft body — have no inside
/// to separate; `lastError` names both sub-types). Looks THROUGH a
/// compound or decorated shape, same as Jolt: one side must be convex all the way down.
pub fn collideShapeVsShapeClosest(
    pair: ShapePair,
    settings: ?CollideShapeSettings,
    filter: ?*const ShapeFilter,
) err.Error!?CollideShapeHit {
    var hit: CollideShapeHit = undefined;
    var did_hit: bool = false;
    try err.check(c.zjoltCollideShapeVsShapeClosest(
        pair.first.shape.handle,
        optionalPtr(math.Vec3, &pair.first.scale),
        &pair.first.position,
        &pair.first.rotation,
        pair.second.shape.handle,
        optionalPtr(math.Vec3, &pair.second.scale),
        &pair.second.position,
        &pair.second.rotation,
        &pair.base_offset,
        optionalPtr(CollideShapeSettings, &settings),
        filter,
        &hit,
        &did_hit,
    ));
    return if (did_hit) hit else null;
}

/// How many overlaps the two shapes have, without collecting them.
pub fn countShapeVsShapeOverlaps(
    pair: ShapePair,
    settings: ?CollideShapeSettings,
    filter: ?*const ShapeFilter,
) err.Error!u32 {
    var count: u32 = 0;
    try err.check(c.zjoltCollideShapeVsShapeAll(
        pair.first.shape.handle,
        optionalPtr(math.Vec3, &pair.first.scale),
        &pair.first.position,
        &pair.first.rotation,
        pair.second.shape.handle,
        optionalPtr(math.Vec3, &pair.second.scale),
        &pair.second.position,
        &pair.second.rotation,
        &pair.base_offset,
        optionalPtr(CollideShapeSettings, &settings),
        filter,
        null,
        0,
        &count,
    ));
    return count;
}

/// Every overlap between the two shapes — one per pair of leaves that touch,
/// so a compound or a mesh reports many. `error.BufferTooSmall` if they do not
/// fit; use `countShapeVsShapeOverlaps` first.
pub fn collideShapeVsShapeAll(
    pair: ShapePair,
    settings: ?CollideShapeSettings,
    filter: ?*const ShapeFilter,
    buffer: []CollideShapeHit,
) err.Error![]CollideShapeHit {
    var count: u32 = 0;
    try err.check(c.zjoltCollideShapeVsShapeAll(
        pair.first.shape.handle,
        optionalPtr(math.Vec3, &pair.first.scale),
        &pair.first.position,
        &pair.first.rotation,
        pair.second.shape.handle,
        optionalPtr(math.Vec3, &pair.second.scale),
        &pair.second.position,
        &pair.second.rotation,
        &pair.base_offset,
        optionalPtr(CollideShapeSettings, &settings),
        filter,
        buffer.ptr,
        @intCast(buffer.len),
        &count,
    ));
    return buffer[0..count];
}

/// Like `collideShapeVsShapeAll`, but AT MOST ONE (the DEEPEST) hit per
/// pair of LEAF shapes, not one per pair of triangles — a compound of two
/// meshes against a compound of two spheres reports at most four hits.
///
/// Same dispatch refusal as `collideShapeVsShapeAll`. `error.BufferTooSmall`
/// if leaf pairs don't fit; `countShapeVsShapeOverlaps` is an upper bound.
pub fn collideShapeVsShapePerLeafAll(
    pair: ShapePair,
    settings: ?CollideShapeSettings,
    filter: ?*const ShapeFilter,
    buffer: []CollideShapeHit,
) err.Error![]CollideShapeHit {
    var count: u32 = 0;
    try err.check(c.zjoltCollideShapeVsShapePerLeafAll(
        pair.first.shape.handle,
        optionalPtr(math.Vec3, &pair.first.scale),
        &pair.first.position,
        &pair.first.rotation,
        pair.second.shape.handle,
        optionalPtr(math.Vec3, &pair.second.scale),
        &pair.second.position,
        &pair.second.rotation,
        &pair.base_offset,
        optionalPtr(CollideShapeSettings, &settings),
        filter,
        buffer.ptr,
        @intCast(buffer.len),
        &count,
    ));
    return buffer[0..count];
}

/// The nearest hit as `pair.cast` sweeps along `pair.direction` into
/// `pair.target` (centre of mass at the hit = start + `hit.fraction *
/// pair.direction`). A sweep starting already overlapping reports
/// fraction 0 only if `settings.back_face_mode_convex` is `.collide`;
/// Jolt's default ignores back faces, so such a sweep reports nothing.
pub fn castShapeVsShapeClosest(
    pair: ShapeCastPair,
    settings: ?ShapeCastSettings,
    filter: ?*const ShapeFilter,
) err.Error!?ShapeCastHit {
    var hit: ShapeCastHit = undefined;
    var did_hit: bool = false;
    try err.check(c.zjoltCastShapeVsShapeClosest(
        pair.cast.shape.handle,
        optionalPtr(math.Vec3, &pair.cast.scale),
        &pair.cast.position,
        &pair.cast.rotation,
        &pair.direction,
        pair.target.shape.handle,
        optionalPtr(math.Vec3, &pair.target.scale),
        &pair.target.position,
        &pair.target.rotation,
        &pair.base_offset,
        optionalPtr(ShapeCastSettings, &settings),
        filter,
        &hit,
        &did_hit,
    ));
    return if (did_hit) hit else null;
}

pub fn countShapeVsShapeCastHits(
    pair: ShapeCastPair,
    settings: ?ShapeCastSettings,
    filter: ?*const ShapeFilter,
) err.Error!u32 {
    var count: u32 = 0;
    try err.check(c.zjoltCastShapeVsShapeAll(
        pair.cast.shape.handle,
        optionalPtr(math.Vec3, &pair.cast.scale),
        &pair.cast.position,
        &pair.cast.rotation,
        &pair.direction,
        pair.target.shape.handle,
        optionalPtr(math.Vec3, &pair.target.scale),
        &pair.target.position,
        &pair.target.rotation,
        &pair.base_offset,
        optionalPtr(ShapeCastSettings, &settings),
        filter,
        null,
        0,
        &count,
    ));
    return count;
}

/// Every hit along the sweep, unsorted. `error.BufferTooSmall` if they do not
/// fit; use `countShapeVsShapeCastHits` first.
pub fn castShapeVsShapeAll(
    pair: ShapeCastPair,
    settings: ?ShapeCastSettings,
    filter: ?*const ShapeFilter,
    buffer: []ShapeCastHit,
) err.Error![]ShapeCastHit {
    var count: u32 = 0;
    try err.check(c.zjoltCastShapeVsShapeAll(
        pair.cast.shape.handle,
        optionalPtr(math.Vec3, &pair.cast.scale),
        &pair.cast.position,
        &pair.cast.rotation,
        &pair.direction,
        pair.target.shape.handle,
        optionalPtr(math.Vec3, &pair.target.scale),
        &pair.target.position,
        &pair.target.rotation,
        &pair.base_offset,
        optionalPtr(ShapeCastSettings, &settings),
        filter,
        buffer.ptr,
        @intCast(buffer.len),
        &count,
    ));
    return buffer[0..count];
}

//=============================================================================
// Manifold reduction
//
// A standalone utility, like the pair above: no shape, no system, just two
// matched point arrays a host's own contact processing already has.
//=============================================================================

/// Reduces two matched contact-point arrays (up to `contact_points_capacity`
/// points) down to 4 or fewer, in place, choosing the four that best
/// preserve the manifold's torque. `points_on_1`/`points_on_2` must be
/// the same length; the returned slices alias the same (caller-owned)
/// memory, narrowed to what survived. `error.InvalidArgument` for
/// 4-or-fewer or more-than-capacity arrays, or a non-unit-vector-able `penetration_axis`.
pub fn pruneContactPoints(
    penetration_axis: math.Vec3,
    points_on_1: []math.Vec3,
    points_on_2: []math.Vec3,
) err.Error!struct { on_1: []math.Vec3, on_2: []math.Vec3 } {
    std.debug.assert(points_on_1.len == points_on_2.len);
    var count: u32 = @intCast(points_on_1.len);
    try err.check(c.zjoltPruneContactPoints(
        &penetration_axis,
        points_on_1.ptr,
        points_on_2.ptr,
        &count,
    ));
    return .{ .on_1 = points_on_1[0..count], .on_2 = points_on_2[0..count] };
}

test "pruneContactPoints keeps the four points that form the widest polygon" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A "plus" of six points in the z = 0 plane: four arm tips ten units
    // out, plus the origin and a point just off it (both inside the
    // quadrilateral the four arms form, so reduction must drop them).
    // Equal penetration depth on every point makes distance from the
    // origin the only tiebreaker, so every step is traceable by hand
    // from ManifoldBetweenTwoFaces.cpp's own algorithm.
    var points_1 = [_]math.Vec3{
        math.vec3(0, 0, 0),
        math.vec3(10, 0, 0),
        math.vec3(-10, 0, 0),
        math.vec3(0, 10, 0),
        math.vec3(0, -10, 0),
        math.vec3(1, 1, 0),
    };
    var points_2 = points_1;

    const reduced = try pruneContactPoints(math.vec3(0, 0, 1), &points_1, &points_2);

    try std.testing.expectEqual(@as(usize, 4), reduced.on_1.len);
    try std.testing.expectEqual(@as(usize, 4), reduced.on_2.len);

    // The exact order Jolt's own algorithm builds it in: the point furthest
    // from the origin first (an arbitrary pick among the four tied arms,
    // broken by array order), then the arm opposite it, then the two
    // remaining arms in the order that maximises the polygon's area.
    const expected = [_]math.Vec3{
        math.vec3(10, 0, 0),
        math.vec3(0, -10, 0),
        math.vec3(-10, 0, 0),
        math.vec3(0, 10, 0),
    };
    for (expected, reduced.on_1, reduced.on_2) |want, got_1, got_2| {
        try std.testing.expectApproxEqAbs(want.x, got_1.x, 1.0e-4);
        try std.testing.expectApproxEqAbs(want.y, got_1.y, 1.0e-4);
        try std.testing.expectApproxEqAbs(want.x, got_2.x, 1.0e-4);
        try std.testing.expectApproxEqAbs(want.y, got_2.y, 1.0e-4);
    }

    // Nothing left to reduce: Jolt's own precondition is that this makes no
    // sense for four points or fewer, so it is refused rather than answered
    // with a no-op.
    var four = [_]math.Vec3{
        math.vec3(10, 0, 0),
        math.vec3(-10, 0, 0),
        math.vec3(0, 10, 0),
        math.vec3(0, -10, 0),
    };
    try std.testing.expectError(
        error.InvalidArgument,
        pruneContactPoints(math.vec3(0, 0, 1), &four, &four),
    );
}

test "the mirrored ray cast defaults are the library's" {
    // `RayCastSettings` carries Zig field defaults so a caller can override
    // one thing and leave the rest; that is a second copy of numbers the C++
    // side owns. This is what stops the copy from drifting.
    var from_library: RayCastSettings = undefined;
    c.zjoltRayCastSettingsInit(&from_library);
    try std.testing.expectEqual(RayCastSettings{}, from_library);
}

test "the mirrored shape query settings defaults are the library's" {
    // Same reason as the ray cast defaults above: `CollideShapeSettings` and
    // `ShapeCastSettings` carry Zig field defaults so a caller can override
    // one thing, and that is a second copy of numbers the C++ side owns.
    var collide: CollideShapeSettings = undefined;
    c.zjoltCollideShapeSettingsInit(&collide);
    try std.testing.expectEqual(CollideShapeSettings{}, collide);

    var cast: ShapeCastSettings = undefined;
    c.zjoltShapeCastSettingsInit(&cast);
    try std.testing.expectEqual(ShapeCastSettings{}, cast);
}

test "two boxes overlap by exactly as much as they are pushed together" {
    // The characteristic behaviour of the shape-versus-shape surface: no
    // system, no body, no step — two shapes and two placements, and a
    // penetration depth that is the overlap and not something near it.
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const half = 0.5;
    const box = try shape_mod.Shape.initBox(math.vec3(half, half, half), .{});
    defer box.release();

    // Overlapping along x by 0.1: the gap between centres is 0.9 and the two
    // half extents sum to 1.
    const overlap = 0.1;
    const together: ShapePair = .{
        .first = .{ .shape = box, .position = math.rvec3_zero },
        .second = .{
            .shape = box,
            .position = math.rvec3(2 * half - overlap, 0, 0),
        },
    };

    const hit = try collideShapeVsShapeClosest(together, null, null);
    try std.testing.expect(hit != null);
    try std.testing.expectApproxEqAbs(
        @as(f32, overlap),
        hit.?.penetration_depth,
        1.0e-3,
    );

    // Nothing found them, so nothing names a body.
    try std.testing.expectEqual(body_mod.invalid_body_id, hit.?.body);

    // And the count-then-fill pair agrees with the closest form.
    try std.testing.expectEqual(
        @as(u32, 1),
        try countShapeVsShapeOverlaps(together, null, null),
    );

    // Pulled apart by half a metre, with no separation distance asked for,
    // there is nothing to report.
    var apart = together;
    apart.second.position = math.rvec3(2 * half + 0.5, 0, 0);
    try std.testing.expectEqual(
        @as(?CollideShapeHit, null),
        try collideShapeVsShapeClosest(apart, null, null),
    );

    // Unless it is asked for: a metre of separation distance turns the same
    // miss into a hit whose depth is the NEGATIVE gap.
    const near_miss = try collideShapeVsShapeClosest(
        apart,
        .{ .max_separation_distance = 1 },
        null,
    );
    try std.testing.expect(near_miss != null);
    try std.testing.expectApproxEqAbs(
        @as(f32, -0.5),
        near_miss.?.penetration_depth,
        1.0e-3,
    );
}

test "collideShapeVsShapePerLeafAll reports one hit where collideShapeVsShapeAll floods one per triangle" {
    // Jolt's own motivating case for CollideShapeVsShapePerLeaf: a
    // triangle mesh overlapped by a convex shape touching several
    // triangles at once. Neither shape is a compound, so the whole
    // mesh-versus-box pair is exactly one leaf pair — this reports one
    // hit for it however many triangles the box touches, while the
    // exhaustive form reports one per triangle.
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // A 4x4x4 box built as a mesh — eight corners, twelve outward-facing
    // triangles, the same construction a mesh floor uses anywhere else in
    // this package's tests.
    const half = 2.0;
    const vertices = [_]math.Vec3{
        math.vec3(-half, -half, -half), math.vec3(-half, -half, half),
        math.vec3(-half, half, -half),  math.vec3(-half, half, half),
        math.vec3(half, -half, -half),  math.vec3(half, -half, half),
        math.vec3(half, half, -half),   math.vec3(half, half, half),
    };
    const indices = [_]u32{
        0, 1, 3, 0, 3, 2, // -x
        4, 6, 7, 4, 7, 5, // +x
        0, 4, 5, 0, 5, 1, // -y
        2, 3, 7, 2, 7, 6, // +y
        0, 2, 6, 0, 6, 4, // -z
        1, 5, 7, 1, 7, 3, // +z
    };
    const mesh = try shape_mod.Shape.initMesh(&vertices, &indices, .{});
    defer mesh.release();

    // Sunk half a metre into the mesh's +y face, straddling the diagonal
    // seam between that face's two triangles — the box's 1.5-unit half
    // extent easily reaches both sides of it.
    const box = try shape_mod.Shape.initBox(math.vec3(1.5, 1.5, 1.5), .{});
    defer box.release();

    const pair: ShapePair = .{
        .first = .{ .shape = mesh, .position = math.rvec3_zero },
        .second = .{ .shape = box, .position = math.rvec3(0, 3, 0) },
    };

    var exhaustive_buf: [8]CollideShapeHit = undefined;
    const exhaustive = try collideShapeVsShapeAll(pair, null, null, &exhaustive_buf);
    try std.testing.expect(exhaustive.len >= 2);

    var per_leaf_buf: [8]CollideShapeHit = undefined;
    const per_leaf = try collideShapeVsShapePerLeafAll(pair, null, null, &per_leaf_buf);
    try std.testing.expectEqual(@as(usize, 1), per_leaf.len);
    try std.testing.expect(per_leaf.len < exhaustive.len);

    // The exhaustive count-then-fill form agrees with the buffer just
    // filled -- the two are the same traversal, one buffered and one not.
    try std.testing.expectEqual(
        @as(u32, @intCast(exhaustive.len)),
        try countShapeVsShapeOverlaps(pair, null, null),
    );
}
