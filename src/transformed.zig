//! A shape, placed in the world, queried on its own.
//!
//! `Queries` answers "what does this system's geometry look like from here",
//! against a `PhysicsSystem`. This is the other half: a shape the caller
//! already holds, a world transform for it, the same query kinds run against
//! that alone — no broad phase to walk, since nothing to find but the one shape
//! already named. Produced from a broad-phase hit carried past its body lock,
//! or `Shape.subShapeTransformedShape` drilling into a compound's child.
//!
//! `...Closest`/`count.../...All` only, no streaming `...Each`: deliberate —
//! one shape's result set is leaf-bounded, not broad-phase-bounded, so
//! streaming buys much less. Unlike a `Shape`, a plain owning handle, not
//! reference-counted — `deinit` it; it counts towards `liveHandleCount` until
//! you do.

const std = @import("std");
const c = @import("c/transformed.zig");
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
const ShapeCastSettings = query_mod.ShapeCastSettings;
const CollideShapeSettings = query_mod.CollideShapeSettings;

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
    /// `sub_shape_id`. @see `Shape.surfaceNormal` for what this is — and is not
    /// — a substitute for (a hit's contact normal is `-penetration_axis` from
    /// the hit itself). `sub_shape_id` follows `material`'s rule:
    /// `sub_shape_id_empty` for a shape with no leaves.
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

    /// The material of the leaf named by `sub_shape_id`. Never null for
    /// a valid handle. @see `Shape.material` for the whole of it,
    /// including why a leafless shape must use `sub_shape_id_empty`
    /// (Jolt asserts otherwise) and the no-material default.
    ///
    /// Borrowed from the wrapped shape; `addRef` it to outlive one.
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

    /// Leaf `sub_shape_id`'s face most facing `direction`. `out_vertices` must
    /// hold `max_supporting_face_vertices` entries; the return slice views it.
    /// Convex shapes/triangles alone have one; others get an empty slice, no
    /// failure. `direction` is WORLD space, unlike `Shape.supportingFace`.
    /// `base_offset` is subtracted per vertex: zero for world-space, or a
    /// nearby point (often the shape's position) for precision far from origin.
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
    // Decomposing a compound
    //
    // `Shape.subShapeTransformedShape` addresses a DIRECT child only; a
    // `SubShapeId` always "comes from a hit", with no way to synthesize one for an arbitrary compound child. This is the other way in: hand Jolt a box, walk the tree to any depth, and report each LEAF overlapping it.
    //=========================================================================

    /// Every leaf of this shape whose world-space bounds overlap `box`, each a
    /// fresh, owned handle — `deinit` every one, free the slice with
    /// `allocator`. A childless shape reports one handle: itself. Each carries
    /// its own root path, baked in at collection, so querying one resolves
    /// sub-shape ids/materials/normals without rebuilding it. `filter` may be
    /// null; its body id is always `invalid_body_id`.
    pub fn collectTransformedShapes(
        self: TransformedShape,
        allocator: std.mem.Allocator,
        box: math.AABox,
        filter: ?*const ShapeFilter,
    ) (err.Error || std.mem.Allocator.Error)![]TransformedShape {
        var count: u32 = 0;
        try err.check(c.zjoltTransformedShapeCollectTransformedShapes(
            self.handle,
            &box,
            filter,
            null,
            0,
            &count,
        ));
        if (count == 0) return &.{};

        const raw = try allocator.alloc(?*c.TransformedShape, count);
        defer allocator.free(raw);

        var written: u32 = 0;
        try err.check(c.zjoltTransformedShapeCollectTransformedShapes(
            self.handle,
            &box,
            filter,
            raw.ptr,
            count,
            &written,
        ));

        // Each entry in raw[0..written] is a handle THIS call produced and
        // this function now owns. If wrapping them fails partway, every one
        // of them is released here rather than leaked — the same "leave
        // nothing behind" rule the C entry point itself follows on its own
        // out-of-memory path.
        const out = allocator.alloc(TransformedShape, written) catch |alloc_err| {
            for (raw[0..written]) |handle| c.zjoltTransformedShapeDestroy(handle);
            return alloc_err;
        };
        for (out, raw[0..written]) |*shape, handle|
            shape.* = .{ .handle = handle.? };
        return out;
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
    // Every leaf the point is inside, treated as solid. A mesh answers usefully only if it is a closed manifold — an open mesh has no inside.
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
        /// Null takes Jolt's defaults. `max_separation_distance` is one of its
        /// fields — above zero it reports near misses too, with a negative
        /// penetration depth, for "is there anything within a metre of this".
        settings: ?CollideShapeSettings = null,
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
            &overlap.base_offset,
            optionalPtr(CollideShapeSettings, &overlap.settings),
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
            &overlap.base_offset,
            optionalPtr(CollideShapeSettings, &overlap.settings),
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
        /// Null takes Jolt's defaults, which IGNORE back faces — so a sweep
        /// starting inside this shape reports nothing at all. Set
        /// `back_face_mode_triangles` to `.collide` against a mesh, or
        /// `back_face_mode_convex` against anything else, when the question is
        /// whether the placement is clear.
        settings: ?ShapeCastSettings = null,
    };

    /// Sweeps `cast.shape` along `cast.direction` and reports the nearest hit
    /// against this one. The centre of mass at the hit is the swept shape's
    /// starting centre of mass plus `hit.fraction * cast.direction`.
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
            optionalPtr(ShapeCastSettings, &cast.settings),
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
            optionalPtr(ShapeCastSettings, &cast.settings),
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
            optionalPtr(ShapeCastSettings, &cast.settings),
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
/// A separate type from `shape.zig`'s `TriangleWalk`, same protocol but a
/// different scratch buffer — the two query surfaces are independent on the C
/// side, sharing no assumed layout.
pub const TriangleWalk = struct {
    shape: TransformedShape,
    context: c.TransformedShapeTrianglesContext,

    /// Continues the walk into `out_vertices` (three consecutive vertices per
    /// triangle) and, if not null, `out_materials` (one per triangle, borrowed
    /// from the wrapped shape). The request comes from `out_vertices.len / 3`,
    /// which must be at least `min_triangles_requested` (Jolt asserts on a
    /// smaller one). An empty result means the walk is over, not "call again
    /// with a bigger buffer".
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

test "collectTransformedShapes decomposes a compound into its known leaves" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const sphere = try shape_mod.Shape.initSphere(0.5, .{});
    defer sphere.release();
    const box = try shape_mod.Shape.initBox(math.vec3(0.5, 0.5, 0.5), .{});
    defer box.release();

    // Two children ten metres apart, each at a position this test already
    // knows by construction -- the thing a whole-body handle cannot report
    // and a per-child sub-shape id cannot be synthesized to ask for.
    const compound = try shape_mod.Shape.initStaticCompound(&.{
        shape_mod.compoundChild(sphere, .{ .position = math.vec3(-5, 0, 0) }),
        shape_mod.compoundChild(box, .{ .position = math.vec3(5, 0, 0) }),
    });
    defer compound.release();

    const ts = try TransformedShape.init(compound, math.rvec3(0, 0, 0), .{});
    defer ts.deinit();

    // A box spanning both children's known positions finds exactly two
    // leaves -- the compound's two actual children, each reported at the
    // position it was built with, not the whole compound reported twice.
    const covers_both: math.AABox = .{
        .min = math.vec3(-6, -1, -1),
        .max = math.vec3(6, 1, 1),
    };
    const leaves = try ts.collectTransformedShapes(std.testing.allocator, covers_both, null);
    defer {
        for (leaves) |leaf| leaf.deinit();
        std.testing.allocator.free(leaves);
    }
    try std.testing.expectEqual(@as(usize, 2), leaves.len);

    var saw_sphere_side = false;
    var saw_box_side = false;
    for (leaves) |leaf| {
        const placement = leaf.worldTransform();
        if (std.math.approxEqAbs(math.Real, placement.position.x, -5, 1.0e-3))
            saw_sphere_side = true;
        if (std.math.approxEqAbs(math.Real, placement.position.x, 5, 1.0e-3))
            saw_box_side = true;

        // Each leaf is queryable on its own, at the path Jolt baked into it
        // during collection -- not the empty sub-shape id a whole-body
        // handle would report for a shape it cannot decompose.
        const leaf_hit = try leaf.castRayClosest(
            math.rvec3(placement.position.x, placement.position.y, -10),
            math.vec3(0, 0, 20),
            null,
            null,
        );
        try std.testing.expect(leaf_hit != null);
    }
    try std.testing.expect(saw_sphere_side);
    try std.testing.expect(saw_box_side);

    // A box overlapping only one child's known position finds exactly that
    // one, not both.
    const covers_sphere_only: math.AABox = .{
        .min = math.vec3(-6, -1, -1),
        .max = math.vec3(-4, 1, 1),
    };
    const one = try ts.collectTransformedShapes(std.testing.allocator, covers_sphere_only, null);
    defer {
        for (one) |leaf| leaf.deinit();
        std.testing.allocator.free(one);
    }
    try std.testing.expectEqual(@as(usize, 1), one.len);
    try std.testing.expectApproxEqAbs(
        @as(math.Real, -5),
        one[0].worldTransform().position.x,
        1.0e-3,
    );

    // A box nowhere near either child finds nothing, and the empty result
    // needs no handle destroyed and no allocation freed.
    const covers_neither: math.AABox = .{
        .min = math.vec3(90, 90, 90),
        .max = math.vec3(91, 91, 91),
    };
    const none = try ts.collectTransformedShapes(std.testing.allocator, covers_neither, null);
    try std.testing.expectEqual(@as(usize, 0), none.len);
}
