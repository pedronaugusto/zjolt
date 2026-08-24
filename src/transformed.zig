//! A shape, placed in the world, queried on its own.
//!
//! `Queries` answers "what does this system's geometry look like from here",
//! and every call on it takes a `PhysicsSystem`. This is the other half: a
//! shape the caller already holds, a world transform for it, and the same
//! kinds of query run against that alone. There is no broad phase to walk,
//! because there is nothing to find but the one shape already named.
//!
//! Two things produce one. A broad-phase hit read under a body lock and then
//! carried past the point where holding that lock is safe — which is what
//! Jolt's own `TransformedShape` exists FOR — and
//! `Shape.subShapeTransformedShape`, drilling into a compound's child.
//!
//! Queries here come in the `...Closest` and `count.../...All` forms
//! `query.zig` describes, and in neither of its streaming `...Each` forms.
//! That is deliberate rather than unfinished: the result set behind one
//! already-resolved shape is bounded by its own leaf count, not by however
//! much of a world a broad phase might hand back, so the allocation-avoidance
//! the streaming form buys matters far less on this side.
//!
//! Unlike a `Shape`, one of these is a plain owning handle rather than a
//! reference-counted object — `deinit` it, and it counts towards
//! `liveHandleCount` until you do.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const material_mod = @import("material.zig");
const shape_mod = @import("shape.zig");
const body_mod = @import("body.zig");
const query_mod = @import("query.zig");

pub const PhysicsMaterial = material_mod.PhysicsMaterial;

const RayCastHit = query_mod.RayCastHit;
const ShapeCastHit = query_mod.ShapeCastHit;
const CollideShapeHit = query_mod.CollideShapeHit;
const CollidePointHit = query_mod.CollidePointHit;
const RayCastSettings = query_mod.RayCastSettings;

/// The one filter a query against a single shape can still take. There is no
/// body filter, no layer filter and no broad-phase filter here, because there
/// is no broad phase and no body to reject — only the leaves of the one shape
/// already chosen. `should_collide` is handed the body id this
/// `TransformedShape` was created with, which is `invalid_body_id` unless the
/// caller supplied one.
const ShapeFilter = query_mod.ShapeFilter;

/// The placement a `TransformedShape` carries: where the shape sits, how it is
/// turned, and how it is scaled.
pub const Transform = c.TransformedShapeTransform;

/// A pointer to an optional's payload, or null. Written once because every
/// nullable-by-pointer argument in this file needs it, and writing it inline
/// invites taking the address of a temporary.
fn optionalPtr(comptime T: type, value: *const ?T) ?*const T {
    return if (value.*) |*payload| payload else null;
}

pub const TransformedShape = struct {
    handle: *c.TransformedShape,

    //=========================================================================
    // Lifetime
    //=========================================================================

    pub const InitOptions = struct {
        rotation: math.Quat = math.quat_identity,
        /// Null keeps (1, 1, 1).
        scale: ?math.Vec3 = null,
        /// Reported by every hit this shape produces, and handed to a
        /// `ShapeFilter` alongside the sub-shape id. The default says there is
        /// no body behind this shape at all — a filter that inspects the
        /// reported body id then sees the invalid id, the same as for any
        /// other query result naming a body-less shape.
        body: body_mod.BodyId = body_mod.invalid_body_id,
    };

    /// Wraps `shape` at the given world placement.
    ///
    /// Takes its own reference on `shape`, so the caller may release theirs
    /// once this returns. The result is owned outright: `deinit` it.
    pub fn init(
        shape: shape_mod.Shape,
        position: math.RVec3,
        opts: InitOptions,
    ) err.Error!TransformedShape {
        var handle: *c.TransformedShape = undefined;
        try err.check(c.zjoltTransformedShapeCreate(
            shape.handle,
            &position,
            &opts.rotation,
            optionalPtr(math.Vec3, &opts.scale),
            opts.body,
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// Destroys the wrapper and drops its reference on the wrapped shape.
    ///
    /// `deinit` rather than `release` because this handle is not reference
    /// counted — the first one is the last one, and a second is a double free.
    pub fn deinit(self: TransformedShape) void {
        c.zjoltTransformedShapeDestroy(self.handle);
    }

    //=========================================================================
    // Placement and geometry
    //=========================================================================

    /// The world placement this was given — NOT its centre-of-mass transform,
    /// which sits elsewhere whenever the wrapped shape's own centre of mass is
    /// offset from where it was authored. That is the same distinction
    /// `BodyInterface.getTransform` and `getCenterOfMassTransform` draw, and
    /// it is the one worth getting wrong here: a shape built with
    /// `Shape.initOffsetCenterOfMass`, or any compound, has the two apart.
    pub fn worldTransform(self: TransformedShape) Transform {
        var out: Transform = undefined;
        c.zjoltTransformedShapeGetWorldTransform(self.handle, &out);
        return out;
    }

    /// Repositions this in place, in the same sense `init` took its placement
    /// in. `scale` null keeps (1, 1, 1) — it does not keep the scale already
    /// set, so a caller moving a scaled shape must pass the scale again.
    pub fn setWorldTransform(
        self: TransformedShape,
        position: math.RVec3,
        rotation: math.Quat,
        scale: ?math.Vec3,
    ) void {
        c.zjoltTransformedShapeSetWorldTransform(
            self.handle,
            &position,
            &rotation,
            optionalPtr(math.Vec3, &scale),
        );
    }

    /// World-space bounds of the wrapped shape at its current placement.
    pub fn worldSpaceBounds(self: TransformedShape) math.AABox {
        var out: math.AABox = undefined;
        c.zjoltTransformedShapeGetWorldSpaceBounds(self.handle, &out);
        return out;
    }

    /// The FACE normal at world-space `position` on the leaf named by
    /// `sub_shape_id`. @see `Shape.surfaceNormal` for what this is — and is
    /// not — a substitute for: for a hit's contact normal use
    /// `-penetration_axis` from the hit itself.
    ///
    /// `sub_shape_id` follows `material`'s rule: `sub_shape_id_empty` for a
    /// shape with no leaves.
    pub fn worldSpaceSurfaceNormal(
        self: TransformedShape,
        sub_shape_id: c.SubShapeId,
        position: math.RVec3,
    ) math.Vec3 {
        var out: math.Vec3 = undefined;
        c.zjoltTransformedShapeGetWorldSpaceSurfaceNormal(
            self.handle,
            sub_shape_id,
            &position,
            &out,
        );
        return out;
    }

    /// The material of the leaf named by `sub_shape_id`. Never null for a
    /// valid handle. @see `Shape.material` for the whole of it, including why
    /// a shape with no leaves must be asked with `sub_shape_id_empty` — Jolt
    /// asserts otherwise — and why a shape built without a material answers
    /// with the shared default rather than null.
    ///
    /// The material is borrowed from the wrapped shape; `addRef` it to outlive
    /// one.
    pub fn material(self: TransformedShape, sub_shape_id: c.SubShapeId) ?PhysicsMaterial {
        const handle = c.zjoltTransformedShapeGetMaterial(self.handle, sub_shape_id) orelse
            return null;
        return .{ .handle = handle };
    }

    /// The user data Jolt keeps on the leaf named by `sub_shape_id` — a mesh's
    /// per-triangle value widened to 64 bits, or 0 for a leaf that has none.
    pub fn subShapeUserData(self: TransformedShape, sub_shape_id: c.SubShapeId) u64 {
        return c.zjoltTransformedShapeGetSubShapeUserData(self.handle, sub_shape_id);
    }

    /// The face of the leaf named by `sub_shape_id` that faces `direction` the
    /// most. `out_vertices` must hold `max_supporting_face_vertices` entries;
    /// the returned slice is a view into it. Only convex shapes and triangles
    /// have one — a sphere, an empty shape and so on report an empty slice,
    /// which is Jolt's own answer and not a failure of this call.
    ///
    /// `direction` is in WORLD space here, unlike `Shape.supportingFace`,
    /// which takes it in the shape's own. `base_offset` is subtracted from
    /// each vertex before it is returned: pass `rvec3_zero` for world-space
    /// vertices, or a point near them — this shape's own position is usually
    /// right — for the precision a double-precision world needs far from the
    /// origin.
    pub fn supportingFace(
        self: TransformedShape,
        sub_shape_id: c.SubShapeId,
        direction: math.Vec3,
        base_offset: math.RVec3,
        out_vertices: *[shape_mod.max_supporting_face_vertices]math.Vec3,
    ) err.Error![]math.Vec3 {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeGetSupportingFace(
            self.handle,
            sub_shape_id,
            &direction,
            &base_offset,
            out_vertices,
            &count,
        ));
        return out_vertices[0..count];
    }

    //=========================================================================
    // Triangle read-back
    //=========================================================================

    /// Starts a triangle walk over this shape, restricted to `box` in world
    /// space. Call `next` repeatedly until it returns an empty slice.
    ///
    /// `base_offset` shifts the returned vertices exactly the way
    /// `supportingFace`'s does.
    pub fn triangleWalk(
        self: TransformedShape,
        box: math.AABox,
        base_offset: math.RVec3,
    ) err.Error!TriangleWalk {
        var walk: TriangleWalk = .{ .shape = self, .context = undefined };
        try err.check(c.zjoltTransformedShapeGetTrianglesStart(
            self.handle,
            &walk.context,
            &box,
            &base_offset,
        ));
        return walk;
    }

    //=========================================================================
    // Ray casts
    //=========================================================================

    /// `direction` carries the ray's length: nothing beyond it is reported.
    /// The hit point is `origin + hit.fraction * direction`.
    ///
    /// `settings` may be null for Jolt's defaults, which ignore back faces and
    /// treat a convex shape as solid. `filter` may be null to accept every
    /// sub-shape.
    pub fn castRayClosest(
        self: TransformedShape,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filter: ?*const ShapeFilter,
    ) err.Error!?RayCastHit {
        var hit: RayCastHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltTransformedShapeCastRayClosest(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filter,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    /// Number of hits along the ray, without collecting them.
    pub fn countRayHits(
        self: TransformedShape,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filter: ?*const ShapeFilter,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCastRayAll(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filter,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Every hit along the ray, unsorted, written into `buffer`. One shape
    /// still yields many hits: a compound has a leaf per child and a mesh one
    /// per triangle crossed. `error.BufferTooSmall` if they do not fit, with
    /// the required count still reported — use `countRayHits` first.
    pub fn castRayAll(
        self: TransformedShape,
        origin: math.RVec3,
        direction: math.Vec3,
        settings: ?RayCastSettings,
        filter: ?*const ShapeFilter,
        buffer: []RayCastHit,
    ) err.Error![]RayCastHit {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCastRayAll(
            self.handle,
            &origin,
            &direction,
            optionalPtr(RayCastSettings, &settings),
            filter,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    //=========================================================================
    // Point
    //
    // Every leaf the point is inside, all of them treated as solid. A mesh
    // answers this usefully only if it is a closed manifold — an open mesh has
    // no inside for a point to be in.
    //=========================================================================

    pub fn countPointHits(
        self: TransformedShape,
        point: math.RVec3,
        filter: ?*const ShapeFilter,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCollidePointAll(
            self.handle,
            &point,
            filter,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// `error.BufferTooSmall` if the hits do not fit; use `countPointHits`
    /// first.
    pub fn collidePoint(
        self: TransformedShape,
        point: math.RVec3,
        filter: ?*const ShapeFilter,
        buffer: []CollidePointHit,
    ) err.Error![]CollidePointHit {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCollidePointAll(
            self.handle,
            &point,
            filter,
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
        /// Borrowed for the call only; this takes no reference on it.
        shape: shape_mod.Shape,
        position: math.RVec3,
        rotation: math.Quat = math.quat_identity,
        /// Null means (1, 1, 1).
        scale: ?math.Vec3 = null,
        /// Reports near misses too, with a negative penetration depth. Useful
        /// for "is there anything within a metre of this".
        max_separation_distance: f32 = 0,
        /// Contact points come back RELATIVE TO this — they are floats, and in
        /// a double-precision world an absolute contact point would not
        /// survive the conversion. Add it back if world space is what you
        /// want; keeping the default of zero means the two are the same
        /// thing.
        base_offset: math.RVec3 = math.rvec3_zero,
    };

    pub fn countCollideShapeHits(
        self: TransformedShape,
        overlap: Overlap,
        filter: ?*const ShapeFilter,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCollideShapeAll(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            overlap.max_separation_distance,
            &overlap.base_offset,
            filter,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Everything in `overlap.shape`, at its given placement, that overlaps
    /// this one. `error.BufferTooSmall` if the hits do not fit; use
    /// `countCollideShapeHits` first.
    pub fn collideShape(
        self: TransformedShape,
        overlap: Overlap,
        filter: ?*const ShapeFilter,
        buffer: []CollideShapeHit,
    ) err.Error![]CollideShapeHit {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCollideShapeAll(
            self.handle,
            overlap.shape.handle,
            optionalPtr(math.Vec3, &overlap.scale),
            &overlap.position,
            &overlap.rotation,
            overlap.max_separation_distance,
            &overlap.base_offset,
            filter,
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
        /// Borrowed for the call only; this takes no reference on it.
        shape: shape_mod.Shape,
        /// Where the sweep starts. This is the shape's own transform, not its
        /// centre of mass; Jolt derives the latter.
        position: math.RVec3,
        rotation: math.Quat = math.quat_identity,
        /// Carries the sweep's length: nothing beyond it is reported.
        direction: math.Vec3,
        /// Null means (1, 1, 1).
        scale: ?math.Vec3 = null,
        /// Contact points come back relative to this, for the reason
        /// `Overlap.base_offset` gives.
        base_offset: math.RVec3 = math.rvec3_zero,
    };

    /// Sweeps `cast.shape` along `cast.direction` and reports the nearest hit
    /// against this one. The centre of mass at the hit is the swept shape's
    /// starting centre of mass plus `hit.fraction * cast.direction`.
    ///
    /// Back faces are collided with or ignored by Jolt's own
    /// `ShapeCastSettings` defaults; there is no settings parameter on this
    /// side to override them with.
    pub fn castShapeClosest(
        self: TransformedShape,
        cast: ShapeCast,
        filter: ?*const ShapeFilter,
    ) err.Error!?ShapeCastHit {
        var hit: ShapeCastHit = undefined;
        var did_hit: bool = false;
        try err.check(c.zjoltTransformedShapeCastShapeClosest(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            &cast.base_offset,
            filter,
            &hit,
            &did_hit,
        ));
        return if (did_hit) hit else null;
    }

    pub fn countShapeCastHits(
        self: TransformedShape,
        cast: ShapeCast,
        filter: ?*const ShapeFilter,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCastShapeAll(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            &cast.base_offset,
            filter,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Every hit along the sweep, unsorted. `error.BufferTooSmall` if they do
    /// not fit; use `countShapeCastHits` first.
    pub fn castShapeAll(
        self: TransformedShape,
        cast: ShapeCast,
        filter: ?*const ShapeFilter,
        buffer: []ShapeCastHit,
    ) err.Error![]ShapeCastHit {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCastShapeAll(
            self.handle,
            cast.shape.handle,
            optionalPtr(math.Vec3, &cast.scale),
            &cast.position,
            &cast.rotation,
            &cast.direction,
            &cast.base_offset,
            filter,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }
};

/// One `TransformedShape.triangleWalk` in progress. `shape` must outlive it,
/// and it must not be moved or reused for a second walk once started.
///
/// A separate type from `shape.zig`'s `TriangleWalk` even though the protocol
/// is identical, because the scratch buffer is: the two query surfaces are
/// independent on the C side, and nothing here assumes they share a layout
/// merely because they happen to today.
pub const TriangleWalk = struct {
    shape: TransformedShape,
    context: c.TransformedShapeTrianglesContext,

    /// Continues the walk into `out_vertices` (three consecutive vertices per
    /// triangle) and, if not null, `out_materials` (one per triangle, borrowed
    /// from the wrapped shape). Both must hold at least `out_vertices.len / 3`
    /// triangles' worth — the request is taken from `out_vertices.len / 3`,
    /// and it must be at least `min_triangles_requested`, because Jolt asserts
    /// on a smaller one rather than honouring it. Returns the vertices
    /// actually written; an empty result means the walk is over, not "call
    /// again with a bigger buffer".
    pub fn next(
        self: *TriangleWalk,
        out_vertices: []math.Vec3,
        out_materials: ?[]?PhysicsMaterial,
    ) err.Error![]math.Vec3 {
        var max_triangles: u32 = @intCast(out_vertices.len / 3);

        var raw_materials: [256]?*const c.PhysicsMaterial = undefined;
        var materials_ptr: ?[*]?*const c.PhysicsMaterial = null;
        if (out_materials) |materials| {
            if (materials.len < max_triangles) return err.Error.InvalidArgument;
            // One call only fills as many materials as this scratch buffer
            // holds. That costs nothing but an extra `next()` when a caller
            // asks for more than this many at once — the protocol already
            // allows a call to return fewer triangles than requested with more
            // still to come, and this is exactly that.
            if (max_triangles > raw_materials.len)
                max_triangles = @intCast(raw_materials.len);
            materials_ptr = &raw_materials;
        }

        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeGetTrianglesNext(
            self.shape.handle,
            &self.context,
            max_triangles,
            out_vertices.ptr,
            materials_ptr,
            &count,
        ));

        if (out_materials) |materials| {
            for (raw_materials[0..count], 0..) |m, i| {
                materials[i] = if (m) |handle| .{ .handle = handle } else null;
            }
        }
        return out_vertices[0 .. @as(usize, count) * 3];
    }
};

test "a transformed shape reports the placement it was given" {
    // The one thing that cannot be checked by reflection: `worldTransform`
    // must report the placement `init` took, not the centre-of-mass transform
    // Jolt keeps beside it.
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try shape_mod.Shape.initSphere(0.5, .{});
    defer sphere.release();

    const ts = try TransformedShape.init(sphere, math.rvec3(1, 2, 3), .{});
    defer ts.deinit();

    const placement = ts.worldTransform();
    try std.testing.expectEqual(@as(math.Real, 2), placement.position.y);
    try std.testing.expectEqual(@as(f32, 1), placement.scale.x);
    try std.testing.expectEqual(@as(f32, 1), placement.rotation.w);

    // And the wrapper is genuinely queryable on its own: a ray straight at it
    // hits, and the hit names no body because none was supplied.
    const hit = try ts.castRayClosest(
        math.rvec3(1, 2, 0),
        math.vec3(0, 0, 6),
        null,
        null,
    );
    try std.testing.expect(hit != null);
    try std.testing.expectEqual(body_mod.invalid_body_id, hit.?.body);
}
