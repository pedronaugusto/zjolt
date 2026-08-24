//! Ray casts, shape casts, overlap and point tests.
//!
//! Every query takes an optional `Filters`. A null one accepts everything, and
//! so does any member left unset, so a caller fills in only the question they
//! care about.
//!
//! Each query comes in three forms, and all three run the same traversal
//! underneath:
//!
//!   * `...Closest` — one hit, the nearest or deepest.
//!   * `count...` / `...All` — the two-call protocol the rest of this package
//!     uses: ask for the count, then hand over a buffer. That keeps the query
//!     path allocation-free from the caller's side, which matters because
//!     these are called from a frame loop.
//!   * `...Each` — a callback per hit, as the traversal finds it. Nothing is
//!     accumulated anywhere, which is what makes it the cheap one for overlap
//!     queries: an overlap hit is about a kilobyte inside Jolt.
//!
//! ## A callback may not return an error to Jolt
//!
//! The hit callbacks here run *inside* Jolt's traversal, with a broad-phase
//! read lock held, behind a C function pointer. A Zig error has no way across
//! that boundary — and a language that unwound instead would skip the lock's
//! destructor and deadlock the next `step` rather than fail here.
//!
//! So a fallible `onHit` is bridged: the shim keeps the error, tells Jolt to
//! stop, and this module returns it once the query is over and every lock has
//! been dropped. A caller writes `try` and does not think about it; what the
//! caller must not do is panic, which aborts rather than returning.

const std = @import("std");
const c = @import("c.zig");
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

/// Jolt's collision settings, reachable only through the shape-versus-shape
/// queries below. Every field defaults to Jolt's own value, so
/// `.{ .max_separation_distance = 1 }` changes one thing and leaves the rest.
pub const CollideShapeSettings = c.CollideShapeSettings;
pub const ShapeCastSettings = c.ShapeCastSettings;
pub const ActiveEdgeMode = c.ActiveEdgeMode;
pub const CollectFacesMode = c.CollectFacesMode;

//=============================================================================
// Ready-made filters
//=============================================================================

/// Rejects one body — the most common filter there is, for a character that
/// must not cast against itself.
///
/// The id has to live somewhere the query can reach, so `filters()` borrows
/// `self` rather than returning something self-contained. Making that a method
/// on a value the caller holds is deliberate: a free function returning a
/// `Filters` that points at its own argument would compile, read fine, and
/// dangle.
///
/// ```zig
/// const exclude: zjolt.ExcludeBody = .{ .body = self_id };
/// const f = exclude.filters();
/// const hit = try queries.castRayClosest(origin, direction, null, &f);
/// ```
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
// One generic shim stands between a Zig `onHit` and the C function pointer
// Jolt ends up calling. It exists to do exactly one thing that a plain
// `callconv(.c)` wrapper could not: carry a Zig error back out.
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

    /// Visits every hit along the ray as it is found, in one traversal and
    /// with no buffer at all.
    ///
    /// `context` points at something declaring
    /// `pub fn onHit(self: *T, hit: RayCastHit) HitAction`, or the same
    /// returning `!HitAction`. Returning `.stop` ends the query; `.narrow`
    /// asks for only nearer hits from here on, so each subsequent hit is
    /// strictly better than the last. Expect narrowing to yield FEWER hits
    /// rather than sorted ones: pruning is what it is for, and the broad phase
    /// already walks roughly front to back.
    ///
    /// ```zig
    /// var seen: struct {
    ///     count: usize = 0,
    ///     pub fn onHit(s: *@This(), _: zjolt.RayCastHit) zjolt.HitAction {
    ///         s.count += 1;
    ///         return .@"continue";
    ///     }
    /// } = .{};
    /// try queries.castRayEach(origin, direction, null, null, &seen);
    /// ```
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
        /// Reports near misses too, with a negative penetration depth. Useful
        /// for "is there anything within a metre of here".
        max_separation_distance: f32 = 0,
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
            overlap.max_separation_distance,
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
            overlap.max_separation_distance,
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
            overlap.max_separation_distance,
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
            overlap.max_separation_distance,
            filters,
            V.onHit,
            &visitor,
        ));
        try visitor.raise();
    }

    //=========================================================================
    // Point
    //
    // Every shape the point is inside, all of them treated as solid. A mesh
    // answers this usefully only if it is a closed manifold — an open mesh has
    // no inside for a point to be in.
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
};

//=============================================================================
// Shape versus shape
//
// Two shapes, two placements, and no `Queries` — no physics system is involved
// at all. "Would these overlap if I put them here", asked before a body
// exists: placement validation, an editor's drag handle, a sweep against
// geometry that is not in the world.
//
// These are also the only queries in this package that take Jolt's collision
// settings. Everything else uses its defaults.
//
// No hit names a body, because there is none: `hit.body` is `invalid_body_id`
// and a `ShapeFilter` is handed that same invalid id, leaving it the two
// sub-shape ids to decide on. `hit.material` still resolves, off the second
// shape.
//
// There is no `...Each` form here, for the reason `transformed.zig` gives: the
// result set behind two already-named shapes is bounded by their own leaf
// counts rather than by however much of a world a broad phase hands back.
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
/// `error.Unsupported` when Jolt has no collision function for the pair — two
/// surfaces (mesh, height field, plane, soft body) have no inside to separate,
/// so nothing is registered for them. `lastError` names both sub-types.
/// The check looks THROUGH a compound or a decorated shape, because Jolt
/// does: those re-dispatch what they hold, so wrapping a mesh does not rescue
/// a mesh-versus-mesh pair. One side has to be convex all the way down.
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

/// The nearest hit as `pair.cast` sweeps along `pair.direction` into
/// `pair.target`. The swept shape's centre of mass at the hit is where it
/// started plus `hit.fraction * pair.direction`.
///
/// A sweep that starts already overlapping reports fraction 0 only if
/// `settings.back_face_mode_convex` is `.collide`; Jolt's default ignores back
/// faces, and such a sweep then reports nothing at all.
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
