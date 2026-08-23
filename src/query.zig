//! Ray casts, shape casts and overlap tests.
//!
//! Every query takes an optional `Filters`. A null one accepts everything, and
//! so does any member left unset, so a caller fills in only the question they
//! care about.
//!
//! The "all hits" variants use the two-call protocol the rest of this package
//! uses: pass a null slice to learn the count, then a buffer. That keeps the
//! query path allocation-free from the caller's side, which matters because
//! these are called from a frame loop.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const shape_mod = @import("shape.zig");

pub const RayCastHit = c.RayCastHit;
pub const ShapeCastHit = c.ShapeCastHit;
pub const CollideShapeHit = c.CollideShapeHit;

pub const BroadPhaseLayerFilter = c.BroadPhaseLayerFilter;
pub const ObjectLayerFilter = c.ObjectLayerFilter;
pub const BodyFilter = c.BodyFilter;
pub const Filters = c.QueryFilters;

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
/// const hit = try queries.castRayClosest(origin, direction, &f);
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

pub const Queries = struct {
    handle: *const c.PhysicsSystem,

    //=========================================================================
    // Ray casts
    //=========================================================================

    /// `direction` carries the ray's length: nothing beyond it is reported.
    /// The hit point is `origin + hit.fraction * direction`.
    pub fn castRayClosest(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        filters: ?*const Filters,
    ) err.Error!?RayCastHit {
        var hit: RayCastHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltCastRayClosest(
            self.handle,
            &origin,
            &direction,
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
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCastRayAll(
            self.handle,
            &origin,
            &direction,
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Every hit along the ray, unsorted, written into `buffer`.
    /// `error.BufferTooSmall` if it does not fit; use `countRayHits` first.
    pub fn castRayAll(
        self: Queries,
        origin: math.RVec3,
        direction: math.Vec3,
        filters: ?*const Filters,
        buffer: []RayCastHit,
    ) err.Error![]RayCastHit {
        var count: u32 = 0;
        try err.check(c.zjoltCastRayAll(
            self.handle,
            &origin,
            &direction,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
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
        const scale = cast.scale;
        try err.check(c.zjoltCastShapeClosest(
            self.handle,
            cast.shape.handle,
            if (scale) |*s| s else null,
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
        const scale = cast.scale;
        try err.check(c.zjoltCastShapeAll(
            self.handle,
            cast.shape.handle,
            if (scale) |*s| s else null,
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
        const scale = cast.scale;
        try err.check(c.zjoltCastShapeAll(
            self.handle,
            cast.shape.handle,
            if (scale) |*s| s else null,
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

    pub fn countCollideShapeHits(
        self: Queries,
        overlap: Overlap,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        const scale = overlap.scale;
        try err.check(c.zjoltCollideShape(
            self.handle,
            overlap.shape.handle,
            if (scale) |*s| s else null,
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
        const scale = overlap.scale;
        try err.check(c.zjoltCollideShape(
            self.handle,
            overlap.shape.handle,
            if (scale) |*s| s else null,
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
};
