//! Triangle collision, active-edge normals, and internal-edge removal.
//!
//! The triangle-collision families (`CollideConvexVsTriangles`,
//! `CastConvexVsTriangles`, `CollideSphereVsTriangles`,
//! `CastSphereVsTriangles`) let a custom shape report collisions against its
//! own triangles, and let a host collide a convex shape or a sphere against
//! a triangle soup it owns without wrapping it in a mesh shape first. Create
//! one per query, feed it triangles as your broad phase finds them, deinit.
//!
//! Their hit callback is the plain C function pointer `CollideShapeHitFn` /
//! `ShapeCastHitFn`, not the `context: anytype` bridge `Queries.*Each`
//! takes — these objects outlive a single call, so there is nowhere for
//! that bridge's stack-lived state to go. Write a small `callconv(.c)` shim
//! for closure-like ergonomics.

const err = @import("error.zig");
const math = @import("math.zig");
const shape_mod = @import("shape.zig");
const query_mod = @import("query.zig");
const c = @import("c/collision.zig");

pub const SubShapeId = query_mod.SubShapeId;
pub const empty_sub_shape_id = query_mod.empty_sub_shape_id;
pub const CollideShapeHitFn = c.CollideShapeHitFn;
pub const ShapeCastHitFn = c.ShapeCastHitFn;
pub const CollideShapeSettings = query_mod.CollideShapeSettings;
pub const ShapeCastSettings = query_mod.ShapeCastSettings;
pub const ShapeFilter = query_mod.ShapeFilter;
pub const PlacedShape = query_mod.PlacedShape;
pub const ShapePair = query_mod.ShapePair;

/// A pointer to an optional's payload, or null. @see the identical helper in
/// `query.zig`.
fn optionalPtr(comptime T: type, value: *const ?T) ?*const T {
    return if (value.*) |*payload| payload else null;
}

//=============================================================================
// Active edges
//
// An edge is "active" when nothing sits on the other side of it at a shallow
// enough angle — the same notion `query.zig`'s `ActiveEdgeMode` names for a
// whole query. These two are the building blocks a custom triangle source
// needs to supply active-edge bits and a corrected normal itself.
//=============================================================================

/// Whether the edge between two triangles sharing `normal1`/`normal2` counts
/// as active. `edge_direction` is the vector along the edge.
pub fn isEdgeActive(normal1: math.Vec3, normal2: math.Vec3, edge_direction: math.Vec3, cos_threshold_angle: f32) bool {
    return c.zjoltActiveEdgesIsEdgeActive(&normal1, &normal2, &edge_direction, cos_threshold_angle);
}

/// `normal` if the hit landed on an active edge or interior, `triangle_normal`
/// otherwise. `active_edges`: bit 0 = edge v0..v1, bit 1 = edge v1..v2, bit 2
/// = edge v2..v0. `movement_direction` may be zero.
pub fn fixNormal(
    v0: math.Vec3,
    v1: math.Vec3,
    v2: math.Vec3,
    triangle_normal: math.Vec3,
    active_edges: u8,
    point: math.Vec3,
    normal: math.Vec3,
    movement_direction: math.Vec3,
) math.Vec3 {
    var out: math.Vec3 = undefined;
    c.zjoltActiveEdgesFixNormal(&v0, &v1, &v2, &triangle_normal, active_edges, &point, &normal, &movement_direction, &out);
    return out;
}

//=============================================================================
// Triangle collision
//
// `shape1` sits at `position1`/`rotation1` with local-space scale `scale1`
// (null for (1,1,1)); its centre of mass is folded in, so it sits where
// placed regardless of offset. `scale2` is the triangle SOURCE's local-space
// scale, applied to every vertex handed to `collide`/`cast`; `position2`/
// `rotation2` place that source with no centre-of-mass adjustment — a
// triangle soup is data, not a `Shape`. `base_offset` is what every returned
// contact point is relative to, as in `Queries.collideShapeClosest`.
//
// A hit's `material` is always null (no `Shape` on the triangle side); its
// `body` is always `invalid_body_id`, as in `collideShapeVsShapeClosest`.
//=============================================================================

/// Shared by every family below: `shape1`'s placement and scale, the
/// triangle source's own placement and scale, and the frame contact points
/// come back in.
pub const TrianglesQuery = struct {
    shape1: shape_mod.Shape,
    scale1: ?math.Vec3 = null,
    position1: math.RVec3,
    rotation1: math.Quat = math.quat_identity,
    scale2: ?math.Vec3 = null,
    position2: math.RVec3,
    rotation2: math.Quat = math.quat_identity,
    base_offset: math.RVec3 = math.rvec3_zero,
};

/// As `TrianglesQuery`, plus the sweep's direction — carries the cast's
/// length, exactly as it does for `Queries.castShapeClosest`.
pub const TrianglesCast = struct {
    shape1: shape_mod.Shape,
    scale1: ?math.Vec3 = null,
    position1: math.RVec3,
    rotation1: math.Quat = math.quat_identity,
    direction: math.Vec3,
    scale2: ?math.Vec3 = null,
    position2: math.RVec3,
    rotation2: math.Quat = math.quat_identity,
    base_offset: math.RVec3 = math.rvec3_zero,
};

/// `shape1` must be a convex shape.
pub const CollideConvexVsTriangles = struct {
    handle: *c.CollideConvexVsTriangles,

    pub fn init(
        query: TrianglesQuery,
        sub_shape_id1: SubShapeId,
        settings: ?CollideShapeSettings,
        on_hit: CollideShapeHitFn,
        user: ?*anyopaque,
    ) err.Error!CollideConvexVsTriangles {
        var handle: *c.CollideConvexVsTriangles = undefined;
        try err.check(c.zjoltCollideConvexVsTrianglesCreate(
            query.shape1.handle,
            optionalPtr(math.Vec3, &query.scale1),
            optionalPtr(math.Vec3, &query.scale2),
            &query.position1,
            &query.rotation1,
            &query.position2,
            &query.rotation2,
            &query.base_offset,
            sub_shape_id1,
            optionalPtr(CollideShapeSettings, &settings),
            on_hit,
            user,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: CollideConvexVsTriangles) void {
        c.zjoltCollideConvexVsTrianglesDestroy(self.handle);
    }

    /// Collides with one CCW triangle; `v0`/`v1`/`v2` are in the triangle
    /// source's own local space, before `scale2`. Returns whether `on_hit`
    /// has asked to stop — a caller's own triangle loop should break rather
    /// than keep paying for calls with no possible result.
    pub fn collide(self: CollideConvexVsTriangles, v0: math.Vec3, v1: math.Vec3, v2: math.Vec3, active_edges: u8, sub_shape_id2: SubShapeId) err.Error!bool {
        var should_stop: bool = false;
        try err.check(c.zjoltCollideConvexVsTrianglesCollide(self.handle, &v0, &v1, &v2, active_edges, sub_shape_id2, &should_stop));
        return should_stop;
    }
};

/// `shape1` must be a convex shape.
pub const CastConvexVsTriangles = struct {
    handle: *c.CastConvexVsTriangles,

    pub fn init(
        params: TrianglesCast,
        settings: ?ShapeCastSettings,
        on_hit: ShapeCastHitFn,
        user: ?*anyopaque,
    ) err.Error!CastConvexVsTriangles {
        var handle: *c.CastConvexVsTriangles = undefined;
        try err.check(c.zjoltCastConvexVsTrianglesCreate(
            params.shape1.handle,
            optionalPtr(math.Vec3, &params.scale1),
            &params.position1,
            &params.rotation1,
            &params.direction,
            optionalPtr(math.Vec3, &params.scale2),
            &params.position2,
            &params.rotation2,
            &params.base_offset,
            optionalPtr(ShapeCastSettings, &settings),
            on_hit,
            user,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: CastConvexVsTriangles) void {
        c.zjoltCastConvexVsTrianglesDestroy(self.handle);
    }

    /// @see CollideConvexVsTriangles.collide.
    pub fn cast(self: CastConvexVsTriangles, v0: math.Vec3, v1: math.Vec3, v2: math.Vec3, active_edges: u8, sub_shape_id2: SubShapeId) err.Error!bool {
        var should_stop: bool = false;
        try err.check(c.zjoltCastConvexVsTrianglesCast(self.handle, &v0, &v1, &v2, active_edges, sub_shape_id2, &should_stop));
        return should_stop;
    }
};

/// `shape1` must be a sphere.
pub const CollideSphereVsTriangles = struct {
    handle: *c.CollideSphereVsTriangles,

    pub fn init(
        query: TrianglesQuery,
        sub_shape_id1: SubShapeId,
        settings: ?CollideShapeSettings,
        on_hit: CollideShapeHitFn,
        user: ?*anyopaque,
    ) err.Error!CollideSphereVsTriangles {
        var handle: *c.CollideSphereVsTriangles = undefined;
        try err.check(c.zjoltCollideSphereVsTrianglesCreate(
            query.shape1.handle,
            optionalPtr(math.Vec3, &query.scale1),
            optionalPtr(math.Vec3, &query.scale2),
            &query.position1,
            &query.rotation1,
            &query.position2,
            &query.rotation2,
            &query.base_offset,
            sub_shape_id1,
            optionalPtr(CollideShapeSettings, &settings),
            on_hit,
            user,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: CollideSphereVsTriangles) void {
        c.zjoltCollideSphereVsTrianglesDestroy(self.handle);
    }

    /// @see CollideConvexVsTriangles.collide.
    pub fn collide(self: CollideSphereVsTriangles, v0: math.Vec3, v1: math.Vec3, v2: math.Vec3, active_edges: u8, sub_shape_id2: SubShapeId) err.Error!bool {
        var should_stop: bool = false;
        try err.check(c.zjoltCollideSphereVsTrianglesCollide(self.handle, &v0, &v1, &v2, active_edges, sub_shape_id2, &should_stop));
        return should_stop;
    }
};

/// `shape1` must be a sphere.
pub const CastSphereVsTriangles = struct {
    handle: *c.CastSphereVsTriangles,

    pub fn init(
        params: TrianglesCast,
        settings: ?ShapeCastSettings,
        on_hit: ShapeCastHitFn,
        user: ?*anyopaque,
    ) err.Error!CastSphereVsTriangles {
        var handle: *c.CastSphereVsTriangles = undefined;
        try err.check(c.zjoltCastSphereVsTrianglesCreate(
            params.shape1.handle,
            optionalPtr(math.Vec3, &params.scale1),
            &params.position1,
            &params.rotation1,
            &params.direction,
            optionalPtr(math.Vec3, &params.scale2),
            &params.position2,
            &params.rotation2,
            &params.base_offset,
            optionalPtr(ShapeCastSettings, &settings),
            on_hit,
            user,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: CastSphereVsTriangles) void {
        c.zjoltCastSphereVsTrianglesDestroy(self.handle);
    }

    /// @see CollideConvexVsTriangles.collide.
    pub fn cast(self: CastSphereVsTriangles, v0: math.Vec3, v1: math.Vec3, v2: math.Vec3, active_edges: u8, sub_shape_id2: SubShapeId) err.Error!bool {
        var should_stop: bool = false;
        try err.check(c.zjoltCastSphereVsTrianglesCast(self.handle, &v0, &v1, &v2, active_edges, sub_shape_id2, &should_stop));
        return should_stop;
    }
};

//=============================================================================
// Ghost-collision removal
//
// A body sliding across a mesh should feel one continuous surface, not catch
// on every seam between triangles. `query.zig`'s
// `Queries.collideShapeWithInternalEdgeRemoval*` offers this against a whole
// `PhysicsSystem`; this is the same correction for two placed shapes with no
// physics system, the way `collideShapeVsShapeAll` is to
// `Queries.collideShapeAll` — the one-call swap that makes a manual
// shape-versus-mesh query ghost-free.
//
// `settings.active_edge_mode` and `.collect_faces_mode` are forced: the
// removal algorithm cannot work without both.
//=============================================================================

/// @see `query.zig`'s `collideShapeVsShapeClosest` for `pair` and the
/// `error.Unsupported` a pair with no convex side raises.
pub fn collideShapeWithInternalEdgeRemoval(
    pair: ShapePair,
    settings: ?CollideShapeSettings,
    filter: ?*const ShapeFilter,
    on_hit: CollideShapeHitFn,
    user: ?*anyopaque,
) err.Error!void {
    try err.check(c.zjoltCollideShapeWithInternalEdgeRemoval(
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
        on_hit,
        user,
    ));
}
