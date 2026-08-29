//! Collision shapes.
//!
//! A shape is immutable, reference counted, and shareable between bodies and
//! between systems. Every constructor here returns one reference, which the
//! caller owns; adding a shape to a body takes its own, so the usual pattern
//! is create, create the body, `release`.

const std = @import("std");
const c = @import("c/shape.zig");
const err = @import("error.zig");
const material_mod = @import("material.zig");
const math = @import("math.zig");
const transformed_mod = @import("transformed.zig");
const stream_mod = @import("stream.zig");

pub const SubType = c.ShapeSubType;
pub const PhysicsMaterial = material_mod.PhysicsMaterial;
pub const TransformedShape = transformed_mod.TransformedShape;
pub const MeshBuildQuality = c.MeshBuildQuality;

/// Vertices `Shape.supportingFace` can report in one call. Jolt's own
/// `Shape::SupportingFace` capacity.
pub const max_supporting_face_vertices: u32 = c.shape_max_supporting_face_vertices;

/// Fewest triangles `TriangleWalk.next` accepts a request for.
pub const min_triangles_requested: u32 = c.shape_min_triangles_requested;

/// Cosine of Jolt's default active-edge threshold angle (5 degrees),
/// shared by `initMesh`, `initHeightField` and `Shape.heightFieldSetHeights`.
/// Not a "leave at zero" default: zero and negative values are themselves
/// meaningful settings, so matching Jolt's default means naming this
/// constant explicitly. Pinned behaviourally, not by the ABI cross-check
/// (which cannot compare floats).
pub const default_active_edge_cos_threshold_angle: f32 = 0.996195;

fn optionalPtr(comptime T: type, value: *const ?T) ?*const T {
    return if (value.*) |*payload| payload else null;
}

/// The height sample that punches a hole in a height field.
///
/// Jolt's own `FLT_MAX` sentinel. It lives here rather than in `c.zig`
/// because the ABI cross-check pairs constants by integer value and this one
/// is a float; the suite pins it behaviourally instead, by casting a ray at a
/// hole and missing.
pub const height_field_no_collision: f32 = std.math.floatMax(f32);

/// Passed as `min_height_value`/`max_height_value` together (the default)
/// to derive `initHeightField`'s quantisation range from the samples
/// rather than reserving a wider one. Jolt's own sentinel
/// (`HeightFieldShape.h`'s `cLargeFloat`), pinned behaviourally: a field
/// built with these accepts a `heightFieldSetHeights` only within the
/// samples' own spread; an explicit wider pair accepts one further out.
pub const auto_min_height_value: f32 = 1.0e15;
pub const auto_max_height_value: f32 = -1.0e15;

/// The sub-shape id meaning "the shape itself, no leaf below it".
///
/// Required rather than convenient: Jolt asserts on any other value for a
/// shape that has no leaves, so `sphere.material(0)` aborts a build with
/// asserts on. @see `Shape.material`.
pub const sub_shape_id_empty: c.SubShapeId = c.sub_shape_id_empty;

/// Pops `bits` bits off the front of `id`, parents before children -- the
/// decode direction of `SubShapeIdCreator.push`. Returns the popped value
/// and what remains of `id`; feed the remainder back in to decode the next
/// level of a nested compound, mesh, or height field by hand.
///
/// `error.InvalidArgument` for `bits` above 32.
pub fn popSubShapeId(
    id: c.SubShapeId,
    bits: u32,
) err.Error!struct { value: u32, remainder: c.SubShapeId } {
    var value: u32 = 0;
    var remainder: c.SubShapeId = 0;
    try err.check(c.zjoltSubShapeIdPopID(id, bits, &value, &remainder));
    return .{ .value = value, .remainder = remainder };
}

/// One child of a compound shape, in the parent's local space.
///
/// Layout-identical to the C struct, so an array of these crosses the boundary
/// with no copy and a compound constructor needs no allocator. Build one with
/// `compoundChild`.
pub const CompoundChild = c.CompoundChild;

/// A plane, as a unit normal and the signed distance from the origin along
/// it: `dot(x, normal) + constant = 0` on the plane.
///
/// Layout-identical to the C struct. Returned by `Shape.plane` (a plane
/// shape's own equation) and `Shape.planes` (a convex hull's face planes) —
/// distinct queries, `Shape.subType` says which applies.
pub const Plane = c.Plane;

/// How `Shape.supportFunction` folds in a convex primitive's rounding
/// radius. `.exclude_convex_radius` caps the radius it removes at Jolt's
/// own 0.05, regardless of the shape's own convex radius.
pub const SupportMode = c.ShapeSupportMode;

/// Scratch space `Shape.supportFunction` places its support object into.
/// Must stay alive and untouched for as long as the result is used --
/// there is nothing to release.
pub const SupportBuffer = c.ShapeSupportBuffer;

/// A convex shape's support function, for GJK/EPA (`geometry.zig`).
/// Borrowed from the `SupportBuffer` it was placed in by
/// `Shape.supportFunction`.
pub const SupportFunction = struct {
    handle: *const c.ShapeSupportFunction,

    /// The support point along `direction`, relative to the shape's own
    /// centre of mass.
    pub fn support(self: SupportFunction, direction: math.Vec3) math.Vec3 {
        var out: math.Vec3 = undefined;
        c.zjoltShapeSupportFunctionGetSupport(self.handle, &direction, &out);
        return out;
    }

    /// The convex radius this support function folds in -- 0 in
    /// `.include_convex_radius` mode.
    pub fn convexRadius(self: SupportFunction) f32 {
        return c.zjoltShapeSupportFunctionGetConvexRadius(self.handle);
    }
};

/// A compound child, positioned and oriented in its parent.
///
/// `shape` is borrowed for the duration of the construction call; the compound
/// takes its own reference, so the caller may release theirs afterwards.
pub fn compoundChild(shape: Shape, opts: struct {
    position: math.Vec3 = math.vec3_zero,
    rotation: math.Quat = math.quat_identity,
    /// Opaque to the library; read back with `Shape.compoundChildUserData`.
    user_data: u32 = 0,
}) CompoundChild {
    return .{
        .shape = shape.handle,
        .position = opts.position,
        .rotation = opts.rotation,
        .user_data = opts.user_data,
    };
}

pub const Shape = struct {
    /// Const because every C entry point that READS a shape, and every
    /// one that mutates a COMPOUND's structure, takes it as `const
    /// ZJoltShape *` or reaches it through `MutableCompound` below.
    /// `setUserData`/`setDensity` are the two exceptions — plain
    /// non-const setters on every shape kind — so those two `@constCast`
    /// at the call site rather than widen this field.
    handle: *const c.Shape,

    //=========================================================================
    // Convex primitives
    //=========================================================================

    /// What every convex primitive takes beyond its own dimensions.
    ///
    /// A null `material` means Jolt's shared default — which is what
    /// `Shape.material` then reports, rather than null.
    pub const ConvexOptions = struct {
        /// Rounds the shape for cheaper, more stable collision. Must not
        /// exceed the smallest half extent.
        convex_radius: f32 = 0.05,
        /// kg/m^3. Zero keeps Jolt's default of 1000.
        density: f32 = 0,
        material: ?PhysicsMaterial = null,
    };

    fn materialHandle(m: ?PhysicsMaterial) ?*const c.PhysicsMaterial {
        return if (m) |value| value.handle else null;
    }

    pub fn initBox(half_extent: math.Vec3, opts: ConvexOptions) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateBox(
            &half_extent,
            opts.convex_radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn initSphere(radius: f32, opts: ConvexOptions) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateSphere(
            radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// A capsule along the Y axis: a cylinder of `half_height_of_cylinder`
    /// capped by hemispheres of `radius`. Total height is
    /// `2 * (half_height_of_cylinder + radius)`.
    pub fn initCapsule(
        half_height_of_cylinder: f32,
        radius: f32,
        opts: ConvexOptions,
    ) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateCapsule(
            half_height_of_cylinder,
            radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// A cylinder along the Y axis, from `(0, -half_height, 0)` to
    /// `(0, half_height, 0)`.
    pub fn initCylinder(half_height: f32, radius: f32, opts: ConvexOptions) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateCylinder(
            half_height,
            radius,
            opts.convex_radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// A single triangle, wound counter-clockwise.
    ///
    /// Infinitely thin except in shape-versus-shape collision, where
    /// `opts.convex_radius` gives it thickness. More a query shape than a body
    /// shape: a world made of triangles wants `initMesh`.
    pub fn initTriangle(
        v1: math.Vec3,
        v2: math.Vec3,
        v3: math.Vec3,
        opts: ConvexOptions,
    ) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateTriangle(
            &v1,
            &v2,
            &v3,
            opts.convex_radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// A capsule whose caps have different radii, with the `top_radius`
    /// cap at `(0, half_height_of_tapered_cylinder, 0)`.
    ///
    /// Jolt simplifies a degenerate case (one cap fully containing the
    /// other) to a sphere or rotated-translated sphere: `subType()` may
    /// then not be `.tapered_capsule` for a shape built here.
    pub fn initTaperedCapsule(
        half_height_of_tapered_cylinder: f32,
        top_radius: f32,
        bottom_radius: f32,
        opts: ConvexOptions,
    ) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateTaperedCapsule(
            half_height_of_tapered_cylinder,
            top_radius,
            bottom_radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// A cylinder whose ends have different radii, with the `top_radius` end
    /// at `(0, half_height, 0)`. Simplified to a plain cylinder when the two
    /// radii are equal.
    pub fn initTaperedCylinder(
        half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
        opts: ConvexOptions,
    ) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateTaperedCylinder(
            half_height,
            top_radius,
            bottom_radius,
            opts.convex_radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub const ConvexHullOptions = struct {
        max_convex_radius: f32 = 0.05,
        /// How far a point may sit outside the hull; larger yields fewer
        /// vertices. Zero keeps Jolt's default.
        hull_tolerance: f32 = 0,
        /// How far the shrunk hull plus `max_convex_radius` may sit from the
        /// actual hull; the radius is lowered automatically if it would
        /// exceed this. Zero keeps Jolt's default of 0.05.
        max_error_convex_radius: f32 = 0,
        density: f32 = 0,
        material: ?PhysicsMaterial = null,
    };

    /// Builds the convex hull OF `points`. Interior points are allowed and are
    /// discarded, so a whole mesh's vertex list is a valid input.
    pub fn initConvexHull(points: []const math.Vec3, opts: ConvexHullOptions) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateConvexHull(
            points.ptr,
            @intCast(points.len),
            opts.max_convex_radius,
            opts.hull_tolerance,
            opts.max_error_convex_radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Shapes without a volume
    //=========================================================================

    pub const PlaneOptions = struct {
        /// Bounds the plane for the broad phase. Inside that box it behaves as
        /// an infinite plane, at the edge inconsistently, and outside not at
        /// all. Zero keeps Jolt's default of 1000.
        half_extent: f32 = 0,
        material: ?PhysicsMaterial = null,
    };

    /// A half space: the negative side of `dot(x, normal) + constant = 0` is
    /// solid. `normal` must be unit length; else `error.InvalidArgument`, not
    /// silent normalisation (that would shift the surface `constant` sets).
    ///
    /// Static or kinematic only — a half space has no volume for dynamic-body
    /// mass.
    pub fn initPlane(normal: math.Vec3, constant: f32, opts: PlaneOptions) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreatePlane(
            &normal,
            constant,
            opts.half_extent,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// A shape with no volume that collides with nothing.
    ///
    /// For a body that must exist before its geometry is known, or one that is
    /// only there to be attached to. Put it in an object layer that collides
    /// with nothing as well, so the broad phase rejects it before the narrow
    /// phase has to.
    pub fn initEmpty(center_of_mass: math.Vec3) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateEmpty(&center_of_mass, &handle));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Mesh
    //=========================================================================

    pub const MeshOptions = struct {
        /// Balances memory against query speed. Zero keeps Jolt's default of
        /// 8; 4 is faster and larger.
        max_triangles_per_leaf: u32 = 0,
        /// The materials this mesh's triangles choose between. At most 32: a
        /// mesh stores the index in five bits of a per-triangle flag byte.
        materials: []const PhysicsMaterial = &.{},
        /// One index into `materials` per triangle, in the order `indices`
        /// gives them. Empty means the mesh has no materials of its own and
        /// every triangle reports the shared default.
        triangle_materials: []const u32 = &.{},
        /// One host-chosen value per triangle, read back later with
        /// `Shape.meshTriangleUserData`. Empty keeps Jolt's default (the
        /// triangle's own pre-reorder index) and costs roughly 25% less
        /// memory.
        triangle_user_data: []const u32 = &.{},
        active_edge_cos_threshold_angle: f32 = default_active_edge_cos_threshold_angle,
        build_quality: MeshBuildQuality = .favor_runtime_performance,
    };

    /// A static triangle mesh, usable only by a static or kinematic body.
    /// `indices` must be a multiple of three, in range for `vertices` —
    /// checked here, not left to fault inside Jolt's tree builder. A
    /// hit's sub-shape id names the triangle AFTER Jolt's own reordering,
    /// meaningful to `material` only, not an index into `indices`.
    pub fn initMesh(
        vertices: []const math.Vec3,
        indices: []const u32,
        opts: MeshOptions,
    ) err.Error!Shape {
        if (indices.len % 3 != 0) return err.Error.InvalidArgument;
        const num_triangles = indices.len / 3;
        if (opts.triangle_materials.len != 0 and
            opts.triangle_materials.len != num_triangles)
        {
            return err.Error.InvalidArgument;
        }
        if (opts.triangle_user_data.len != 0 and
            opts.triangle_user_data.len != num_triangles)
        {
            return err.Error.InvalidArgument;
        }

        // `PhysicsMaterial` is a struct wrapping one pointer, so a slice of
        // them is already the array of pointers the C side wants — but that is
        // a layout coincidence, not a promise, and reinterpreting the slice
        // would make it one. Copied instead, into a buffer sized by the limit
        // Jolt itself enforces.
        var handles: [32]*const c.PhysicsMaterial = undefined;
        if (opts.materials.len > handles.len) return err.Error.InvalidArgument;
        for (opts.materials, 0..) |m, i| handles[i] = m.handle;

        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateMesh(
            vertices.ptr,
            @intCast(vertices.len),
            indices.ptr,
            @intCast(num_triangles),
            if (opts.triangle_materials.len != 0) opts.triangle_materials.ptr else null,
            if (opts.triangle_user_data.len != 0) opts.triangle_user_data.ptr else null,
            if (opts.materials.len != 0) &handles else null,
            @intCast(opts.materials.len),
            opts.max_triangles_per_leaf,
            opts.active_edge_cos_threshold_angle,
            opts.build_quality,
            &handle,
        ));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Height field
    //=========================================================================

    pub const HeightFieldOptions = struct {
        /// The surface is `offset + scale * (x, samples[..], y)`.
        offset: math.Vec3 = math.vec3_zero,
        scale: math.Vec3 = .{ .x = 1, .y = 1, .z = 1 },
        /// Jolt's default of 2 when zero, otherwise in `[2, 8]`. Bigger blocks
        /// cost less memory and more query time.
        block_size: u32 = 0,
        /// Jolt's default of 8 when zero, otherwise in `[1, 16]`.
        bits_per_sample: u32 = 0,
        /// At most 256.
        materials: []const PhysicsMaterial = &.{},
        /// One index into `materials` per QUAD — `(sample_count - 1)^2` of
        /// them, not one per sample.
        quad_materials: []const u8 = &.{},
        /// The range `samples` is quantised into, to 16 bits, fixed for
        /// the shape's life. Defaults derive the tightest range that fits
        /// `samples`. Widen the pair to reserve headroom
        /// `heightFieldSetHeights` can move a sample into later without
        /// being clamped back; narrower than `samples` needs has no effect.
        min_height_value: f32 = auto_min_height_value,
        max_height_value: f32 = auto_max_height_value,
        active_edge_cos_threshold_angle: f32 = default_active_edge_cos_threshold_angle,
    };

    /// A static height field of `sample_count` x `sample_count` samples,
    /// row major: `(x, y)` is `samples[y * sample_count + x]`. A sample
    /// of `height_field_no_collision` punches a hole. `sample_count` need
    /// not be a multiple of `opts.block_size` (Jolt rounds up and pads
    /// with holes), but the rounded count / block size must be at least 2.
    pub fn initHeightField(
        samples: []const f32,
        sample_count: u32,
        opts: HeightFieldOptions,
    ) err.Error!Shape {
        if (sample_count == 0) return err.Error.InvalidArgument;
        if (samples.len != @as(usize, sample_count) * sample_count)
            return err.Error.InvalidArgument;

        const quads = @as(usize, sample_count - 1) * (sample_count - 1);
        if (opts.quad_materials.len != 0 and opts.quad_materials.len != quads)
            return err.Error.InvalidArgument;

        var handles: [256]*const c.PhysicsMaterial = undefined;
        if (opts.materials.len > handles.len) return err.Error.InvalidArgument;
        for (opts.materials, 0..) |m, i| handles[i] = m.handle;

        var handle: *c.Shape = undefined;
        var offset = opts.offset;
        var scale = opts.scale;
        try err.check(c.zjoltShapeCreateHeightField(
            samples.ptr,
            sample_count,
            &offset,
            &scale,
            if (opts.quad_materials.len != 0) opts.quad_materials.ptr else null,
            if (opts.materials.len != 0) &handles else null,
            @intCast(opts.materials.len),
            opts.block_size,
            opts.bits_per_sample,
            opts.min_height_value,
            opts.max_height_value,
            opts.active_edge_cos_threshold_angle,
            &handle,
        ));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Compounds
    //=========================================================================

    /// A compound whose children are fixed once built, stored in a tree.
    /// An empty `children` is `error.InvalidArgument`.
    ///
    /// Jolt simplifies a single unmoved, unrotated child to that child
    /// itself, and a single moved/rotated one to a rotated-translated
    /// shape: `subType()` may then not be `.static_compound`.
    pub fn initStaticCompound(children: []const CompoundChild) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateStaticCompound(
            children.ptr,
            @intCast(children.len),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// Children of a compound shape, or 0 for any other kind of shape.
    pub fn compoundChildCount(self: Shape) u32 {
        return c.zjoltShapeCompoundGetNumChildren(self.handle);
    }

    /// The user data a child was added with, or 0 if `index` is out of range.
    pub fn compoundChildUserData(self: Shape, index: u32) u32 {
        return c.zjoltShapeCompoundGetChildUserData(self.handle, index);
    }

    /// Changes the user data a child was added with — a plain data field,
    /// independent of the structural obligations `MutableCompound`'s
    /// mutating methods carry. Works on a STATIC compound too. `error.
    /// InvalidArgument` if this is not a compound, or `index` is at or
    /// beyond `compoundChildCount`.
    pub fn setCompoundChildUserData(self: Shape, index: u32, user_data: u32) err.Error!void {
        try err.check(
            c.zjoltShapeCompoundSetChildUserData(@constCast(self.handle), index, user_data),
        );
    }

    /// The sub-shape id addressing this compound's direct child `index`,
    /// from this shape's own root. Inverse of `subShapeIndexFromID`; for
    /// a grandchild (or deeper), chain `subShapeIDFromIndexInto` with a
    /// `SubShapeIdCreator` instead. `error.InvalidArgument` if this is
    /// not a compound, or `index` is at or beyond `compoundChildCount`.
    pub fn subShapeIDFromIndex(self: Shape, index: u32) err.Error!c.SubShapeId {
        var out: c.SubShapeId = 0;
        try err.check(c.zjoltShapeGetSubShapeIDFromIndex(self.handle, index, &out));
        return out;
    }

    /// As `subShapeIDFromIndex`, but composing onto `creator` in place instead
    /// of always starting at the root -- the level-by-level way to address a
    /// grandchild (or deeper): call once per level, from the outermost
    /// compound down, passing the same `creator` through each call.
    pub fn subShapeIDFromIndexInto(self: Shape, creator: SubShapeIdCreator, index: u32) err.Error!void {
        try err.check(c.zjoltShapeGetSubShapeIDFromIndexInto(self.handle, index, creator.handle));
    }

    /// The inverse of `subShapeIDFromIndex`: which direct child
    /// `sub_shape_id` names, and what is left of the id after removing the
    /// path to it — meaningful when that child is itself a compound or a
    /// mesh. `error.InvalidArgument` if this is not a compound, or
    /// `sub_shape_id` does not name one of its direct children.
    pub fn subShapeIndexFromID(
        self: Shape,
        sub_shape_id: c.SubShapeId,
    ) err.Error!struct { index: u32, remainder: c.SubShapeId } {
        var index: u32 = 0;
        var remainder: c.SubShapeId = 0;
        try err.check(c.zjoltShapeGetSubShapeIndexFromID(
            self.handle,
            sub_shape_id,
            &index,
            &remainder,
        ));
        return .{ .index = index, .remainder = remainder };
    }

    /// Which of this compound's direct children have a bounding box
    /// overlapping `box`, both in this shape's own local space.
    /// `error.InvalidArgument` if this is not a compound.
    pub fn intersectingSubShapes(
        self: Shape,
        box: math.AABox,
        allocator: std.mem.Allocator,
    ) err.Error![]u32 {
        var count: u32 = 0;
        try err.check(
            c.zjoltShapeGetIntersectingSubShapes(self.handle, &box, null, 0, &count),
        );

        // `count` is the number of CHILDREN, an upper bound on how many
        // actually intersect — not the exact answer the way most two-call
        // protocols in this package report. So the exact result is copied
        // into its own exactly-sized allocation rather than handed back as
        // a sub-slice of the scratch buffer: a slice shorter than what it
        // was allocated with is not a valid argument to `allocator.free`.
        const scratch = try allocator.alloc(u32, count);
        defer allocator.free(scratch);
        var actual: u32 = 0;
        try err.check(c.zjoltShapeGetIntersectingSubShapes(
            self.handle,
            &box,
            scratch.ptr,
            @intCast(scratch.len),
            &actual,
        ));

        const result = try allocator.alloc(u32, actual);
        @memcpy(result, scratch[0..actual]);
        return result;
    }

    //=========================================================================
    // Decorated shapes
    //=========================================================================

    /// Non-uniformly scales an existing shape. Takes its own reference on the
    /// inner shape, so the caller may release theirs.
    pub fn initScaled(inner: Shape, scale: math.Vec3) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateScaled(inner.handle, &scale, &handle));
        return .{ .handle = handle };
    }

    /// Places a shape at an offset and orientation inside its parent. This is
    /// how a capsule gets its base at the origin rather than its centre.
    pub fn initRotatedTranslated(
        inner: Shape,
        translation: math.Vec3,
        rotation: math.Quat,
    ) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateRotatedTranslated(
            inner.handle,
            &translation,
            &rotation,
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// Shifts a shape's centre of mass without moving its geometry — a
    /// weeble, or a vehicle with a low centre of gravity.
    pub fn initOffsetCenterOfMass(inner: Shape, offset: math.Vec3) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateOffsetCenterOfMass(inner.handle, &offset, &handle));
        return .{ .handle = handle };
    }

    /// The shape immediately inside a scaled, rotated-translated, or
    /// offset-center-of-mass wrapper, exactly one level — unlike `leafShape`,
    /// which keeps drilling until it reaches a non-decorated leaf. Borrowed:
    /// owned by this shape, valid exactly as long as it is. `error.
    /// InvalidArgument` if this is not decorated.
    pub fn innerShape(self: Shape) err.Error!Shape {
        var handle: *const c.Shape = undefined;
        try err.check(c.zjoltShapeGetInnerShape(self.handle, &handle));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Lifetime
    //=========================================================================

    pub fn addRef(self: Shape) void {
        c.zjoltShapeAddRef(self.handle);
    }

    /// Drops one reference. The shape is destroyed when the last one goes.
    pub fn release(self: Shape) void {
        c.zjoltShapeRelease(self.handle);
    }

    pub fn refCount(self: Shape) u32 {
        return c.zjoltShapeGetRefCount(self.handle);
    }

    /// Opaque to the library, and 0 until set with `setUserData`. Every
    /// `init*` here builds its settings on the stack without crossing the
    /// settings object itself, so this is the only way to reach the user
    /// data every Jolt `*ShapeSettings` inherits, uniformly across shape kinds.
    pub fn userData(self: Shape) u64 {
        return c.zjoltShapeGetUserData(self.handle);
    }

    pub fn setUserData(self: Shape, user_data: u64) void {
        c.zjoltShapeSetUserData(@constCast(self.handle), user_data);
    }

    //=========================================================================
    // Introspection
    //=========================================================================

    pub fn subType(self: Shape) SubType {
        return c.zjoltShapeGetSubType(self.handle);
    }

    /// The material of one leaf of this shape. Never null for a valid
    /// shape; one built without a material answers with the shared
    /// default — compare `PhysicsMaterial.default()` to tell apart.
    /// `sub_shape_id` comes from a hit or contact manifold, not one
    /// composed by hand; pass `sub_shape_id_empty` for a shape with no
    /// leaves. Borrowed, `addRef` to outlive this shape.
    pub fn material(self: Shape, sub_shape_id: c.SubShapeId) ?PhysicsMaterial {
        const handle = c.zjoltShapeGetMaterial(self.handle, sub_shape_id) orelse
            return null;
        return .{ .handle = handle };
    }

    /// Replaces a convex primitive's material. `null` installs Jolt's
    /// shared default. Live edit to shared state: every body using
    /// `self` sees the change immediately (a shape is shared, not
    /// copied). Drops the old material reference and adds one to
    /// `new_material`; `error.InvalidArgument` for a non-convex shape.
    pub fn setMaterial(self: Shape, new_material: ?PhysicsMaterial) err.Error!void {
        try err.check(c.zjoltShapeSetMaterial(
            @constCast(self.handle),
            if (new_material) |m| m.handle else null,
        ));
    }

    pub fn volume(self: Shape) f32 {
        return c.zjoltShapeGetVolume(self.handle);
    }

    pub fn centerOfMass(self: Shape) math.Vec3 {
        var out: math.Vec3 = undefined;
        c.zjoltShapeGetCenterOfMass(self.handle, &out);
        return out;
    }

    pub fn localBounds(self: Shape) math.AABox {
        var out: math.AABox = undefined;
        c.zjoltShapeGetLocalBounds(self.handle, &out);
        return out;
    }

    pub fn massProperties(self: Shape) math.MassProperties {
        var out: math.MassProperties = undefined;
        c.zjoltShapeGetMassProperties(self.handle, &out);
        return out;
    }

    /// Memory footprint and triangle count, this shape and everything under
    /// it. Useful for budgeting, and the cheapest way to confirm a save/restore
    /// round trip kept its children.
    pub fn stats(self: Shape) math.ShapeStats {
        var out: math.ShapeStats = undefined;
        c.zjoltShapeGetStats(self.handle, &out);
        return out;
    }

    //=========================================================================
    // Convex-primitive dimension introspection
    //
    // Each getter below returns `error.InvalidArgument` for a shape kind
    // it does not apply to, except `innerRadius` (implemented by every kind).
    //=========================================================================

    /// The radius of the largest sphere that fits inside this shape, Jolt's
    /// own convex-collision margin. 0 for a shape with no meaningful
    /// interior, such as a plane or an empty shape — defined for every kind,
    /// so unlike the rest of this section it cannot fail.
    pub fn innerRadius(self: Shape) f32 {
        return c.zjoltShapeGetInnerRadius(self.handle);
    }

    /// A convex shape's density in kg/m^3, as it stands right now — distinct
    /// from `ConvexOptions.density`'s creation-time role. `error.
    /// InvalidArgument` for a shape that is not a convex primitive (sphere,
    /// box, capsule, cylinder, convex hull, triangle, tapered capsule,
    /// tapered cylinder) — a mesh, height field, plane, compound or
    /// decorated shape has no density of its own.
    pub fn density(self: Shape) err.Error!f32 {
        var out: f32 = undefined;
        try err.check(c.zjoltShapeGetDensity(self.handle, &out));
        return out;
    }

    /// Changing this does not retroactively rescale a body already built
    /// from this shape: Jolt bakes mass properties in at body creation, so a
    /// later `massProperties` reflects the new density but an existing body
    /// does not until it is rebuilt or given a mass override. Same
    /// `error.InvalidArgument` restriction as `density`.
    pub fn setDensity(self: Shape, new_density: f32) err.Error!void {
        try err.check(c.zjoltShapeSetDensity(@constCast(self.handle), new_density));
    }

    /// This shape's support function for GJK/EPA (`geometry.zig`), placed
    /// inside `buffer`. `buffer` must stay alive and untouched for as long
    /// as the result is used. `scale` null keeps (1, 1, 1).
    /// `error.InvalidArgument` for a shape that is not a convex primitive.
    pub fn supportFunction(
        self: Shape,
        mode: SupportMode,
        buffer: *SupportBuffer,
        scale: ?math.Vec3,
    ) err.Error!SupportFunction {
        var handle: *const c.ShapeSupportFunction = undefined;
        try err.check(c.zjoltShapeGetSupportFunction(
            self.handle,
            mode,
            buffer,
            optionalPtr(math.Vec3, &scale),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// Sphere, capsule or cylinder radius. `error.InvalidArgument` for any
    /// other kind, including a box or a tapered capsule/cylinder — those
    /// round a different dimension, @see `convexRadius` and
    /// `topRadius`/`bottomRadius`.
    pub fn sphereRadius(self: Shape) err.Error!f32 {
        var out: f32 = undefined;
        try err.check(c.zjoltShapeGetRadius(self.handle, &out));
        return out;
    }

    /// A box's half extent, the same value `initBox` took.
    /// `error.InvalidArgument` for any other kind.
    pub fn halfExtent(self: Shape) err.Error!math.Vec3 {
        var out: math.Vec3 = undefined;
        try err.check(c.zjoltShapeGetHalfExtent(self.handle, &out));
        return out;
    }

    /// Cylinder, tapered capsule or tapered cylinder half height.
    /// `error.InvalidArgument` for any other kind — a plain capsule's is
    /// `halfHeightOfCylinder` instead, because Jolt gives that one a distinct
    /// name (it still excludes the hemispherical caps `initCapsule`'s
    /// `radius` adds).
    pub fn halfHeight(self: Shape) err.Error!f32 {
        var out: f32 = undefined;
        try err.check(c.zjoltShapeGetHalfHeight(self.handle, &out));
        return out;
    }

    /// A capsule's half height of cylinder, the same value `initCapsule`
    /// took. `error.InvalidArgument` for any other kind.
    pub fn halfHeightOfCylinder(self: Shape) err.Error!f32 {
        var out: f32 = undefined;
        try err.check(c.zjoltShapeGetHalfHeightOfCylinder(self.handle, &out));
        return out;
    }

    /// A tapered capsule or tapered cylinder's radius at its `+half_height`
    /// end. `error.InvalidArgument` for any other kind — including a shape
    /// that was SIMPLIFIED into a sphere or a plain cylinder, which no longer
    /// has one.
    pub fn topRadius(self: Shape) err.Error!f32 {
        var out: f32 = undefined;
        try err.check(c.zjoltShapeGetTopRadius(self.handle, &out));
        return out;
    }

    /// The radius at the `-half_height` end. @see `topRadius`.
    pub fn bottomRadius(self: Shape) err.Error!f32 {
        var out: f32 = undefined;
        try err.check(c.zjoltShapeGetBottomRadius(self.handle, &out));
        return out;
    }

    /// The convex radius a box, cylinder, convex hull or tapered cylinder
    /// rounds its edges by. `error.InvalidArgument` for any other kind — a
    /// sphere or capsule has no separate convex radius, because its own
    /// radius already plays that role, and a tapered capsule has none at
    /// all.
    pub fn convexRadius(self: Shape) err.Error!f32 {
        var out: f32 = undefined;
        try err.check(c.zjoltShapeGetConvexRadius(self.handle, &out));
        return out;
    }

    /// A convex hull's face count. `error.InvalidArgument` for any other
    /// kind.
    pub fn numFaces(self: Shape) err.Error!u32 {
        var out: u32 = undefined;
        try err.check(c.zjoltShapeGetNumFaces(self.handle, &out));
        return out;
    }

    /// The number of vertices in convex hull face `face_index`.
    /// `error.InvalidArgument` for any other kind, or for a `face_index` at
    /// or beyond `numFaces` — Jolt indexes its own face array with no bounds
    /// check at all, so out of range there is not an assert but a read past
    /// the end.
    pub fn numVerticesInFace(self: Shape, face_index: u32) err.Error!u32 {
        var out: u32 = undefined;
        try err.check(
            c.zjoltShapeGetNumVerticesInFace(self.handle, face_index, &out),
        );
        return out;
    }

    /// How many points `points` will need for this shape, or
    /// `error.InvalidArgument` if it is not a convex hull.
    pub fn numPoints(self: Shape) err.Error!u32 {
        var out: u32 = undefined;
        try err.check(c.zjoltShapeGetPoints(self.handle, null, 0, &out));
        return out;
    }

    /// A convex hull's own vertices, relative to its centre of mass, written
    /// into `buffer`. `error.BufferTooSmall` if it does not fit — size with
    /// `numPoints` first; `error.InvalidArgument` if this is not a convex
    /// hull.
    pub fn hullPoints(self: Shape, buffer: []math.Vec3) err.Error![]math.Vec3 {
        var out_count: u32 = undefined;
        try err.check(c.zjoltShapeGetPoints(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &out_count,
        ));
        return buffer[0..out_count];
    }

    /// How many planes `planes` will need for this shape, or
    /// `error.InvalidArgument` if it is not a convex hull.
    pub fn numPlanes(self: Shape) err.Error!u32 {
        var out: u32 = undefined;
        try err.check(c.zjoltShapeGetPlanes(self.handle, null, 0, &out));
        return out;
    }

    /// A convex hull's own face planes, relative to its centre of mass,
    /// written into `buffer`. `error.BufferTooSmall` if it does not fit —
    /// size with `numPlanes` first; `error.InvalidArgument` if this is not a
    /// convex hull — in particular NOT a plane shape's plane, which is
    /// `plane` (singular).
    pub fn planes(self: Shape, buffer: []Plane) err.Error![]Plane {
        var out_count: u32 = undefined;
        try err.check(c.zjoltShapeGetPlanes(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &out_count,
        ));
        return buffer[0..out_count];
    }

    /// A plane shape's own plane equation and bounding half extent — both
    /// values `initPlane` took. `error.InvalidArgument` for any other kind.
    ///
    /// No separate vertex read-back: Jolt keeps the four corners as a
    /// PRIVATE helper with no accessor — reconstruct them from the
    /// returned plane and half extent directly.
    pub fn plane(self: Shape) err.Error!struct { plane: Plane, half_extent: f32 } {
        var out_plane: Plane = undefined;
        var out_half_extent: f32 = undefined;
        try err.check(
            c.zjoltShapeGetPlane(self.handle, &out_plane, &out_half_extent),
        );
        return .{ .plane = out_plane, .half_extent = out_half_extent };
    }

    /// The total and submerged volume of this shape — placed by
    /// `transform` and `scale` (null for (1, 1, 1)) — below `surface`,
    /// and the world-space centre of mass of the submerged part.
    ///
    /// `error.InvalidArgument` if this shape or anything beneath it is a
    /// mesh, height field, or plane — Jolt 5.6.0 leaves those uninitialised.
    pub fn submergedVolume(
        self: Shape,
        transform: math.Mat44,
        scale: ?math.Vec3,
        surface: Plane,
    ) err.Error!struct {
        total_volume: f32,
        submerged_volume: f32,
        center_of_buoyancy: math.Vec3,
    } {
        var total_volume: f32 = undefined;
        var submerged_volume: f32 = undefined;
        var center_of_buoyancy: math.Vec3 = undefined;
        try err.check(c.zjoltShapeGetSubmergedVolume(
            self.handle,
            &transform,
            optionalPtr(math.Vec3, &scale),
            &surface,
            &total_volume,
            &submerged_volume,
            &center_of_buoyancy,
        ));
        return .{
            .total_volume = total_volume,
            .submerged_volume = submerged_volume,
            .center_of_buoyancy = center_of_buoyancy,
        };
    }

    //=========================================================================
    // Serialisation
    //
    // Jolt's own binary shape state, tied to the vendored Jolt version and double-precision setting — a
    // cooking cache, not an interchange format. See UPSTREAM.md.
    //=========================================================================

    /// Bytes `save` will need. Cheap enough to call before every save; it runs
    /// the same traversal, so the two can never disagree.
    pub fn saveSize(self: Shape) err.Error!usize {
        var size: usize = 0;
        try err.check(c.zjoltShapeSave(self.handle, null, 0, &size));
        return size;
    }

    /// Serialises into `buffer`, returning the slice actually written.
    /// `error.BufferTooSmall` if it does not fit; use `saveSize` first.
    pub fn save(self: Shape, buffer: []u8) err.Error![]u8 {
        var size: usize = 0;
        try err.check(c.zjoltShapeSave(self.handle, buffer.ptr, buffer.len, &size));
        return buffer[0..size];
    }

    /// Allocates and serialises in one step.
    pub fn saveAlloc(self: Shape, gpa: std.mem.Allocator) ![]u8 {
        const size = try self.saveSize();
        const buffer = try gpa.alloc(u8, size);
        errdefer gpa.free(buffer);
        return try self.save(buffer);
    }

    /// `save`, through `stream` instead of a resident buffer — for a large
    /// cook a caller would rather stream than size and hold whole. @see
    /// `zjolt.hostStream`. Less corruption margin than `restore`: the
    /// header carries a magic tag and build identity, but no length or
    /// checksum. `error.IoError` if `stream` reports failure.
    pub fn saveStream(self: Shape, stream: stream_mod.Stream) err.Error!void {
        try err.check(c.zjoltShapeSaveStream(self.handle, &stream));
    }

    /// Rebuilds a shape from `save` output. A truncated buffer, or one with
    /// trailing bytes, is `error.BadFormat` rather than a partially parsed
    /// shape.
    pub fn restore(data: []const u8) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeRestore(data.ptr, data.len, &handle));
        return .{ .handle = handle };
    }

    /// Rebuilds a shape written by `saveStream`. @see `restore` for what
    /// `error.BadFormat` covers; a stream form has no length or checksum to
    /// check first.
    pub fn restoreStream(stream: stream_mod.Stream) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeRestoreStream(&stream, &handle));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Introspection Jolt puts on every leaf shape
    //=========================================================================

    /// Sub-shape id bits this shape needs to address any leaf beneath it.
    pub fn subShapeIDBits(self: Shape) u32 {
        return c.zjoltShapeGetSubShapeIDBits(self.handle);
    }

    /// Whether `sub_shape_id` names something in this shape that
    /// `material` and friends could safely be given — every shape kind
    /// but a compound just asserts on a bad id instead of reporting it.
    /// For a shape with no leaves, valid means exactly
    /// `sub_shape_id_empty`. A mesh/height field checks only structural
    /// well-formedness, not that the id names a real triangle or quad.
    pub fn isSubShapeIDValid(self: Shape, sub_shape_id: c.SubShapeId) bool {
        return c.zjoltShapeIsSubShapeIDValid(self.handle, sub_shape_id);
    }

    /// The FACE normal at `local_surface_position` on the leaf named by
    /// `sub_shape_id`, both relative to this shape's own center of mass.
    /// A face normal, not a vertex or edge one — for a hit's contact
    /// normal use `-penetration_axis` from the hit itself. `sub_shape_id`
    /// follows `material`'s rule: pass `sub_shape_id_empty` for a shape
    /// with no leaves.
    pub fn surfaceNormal(
        self: Shape,
        sub_shape_id: c.SubShapeId,
        local_surface_position: math.Vec3,
    ) math.Vec3 {
        var out: math.Vec3 = undefined;
        c.zjoltShapeGetSurfaceNormal(
            self.handle,
            sub_shape_id,
            &local_surface_position,
            &out,
        );
        return out;
    }

    pub const SupportingFaceOptions = struct {
        /// Local to this shape. Null keeps (1, 1, 1).
        scale: ?math.Vec3 = null,
        /// Placement of this shape's center of mass in the space the face
        /// should come back in.
        position: math.Vec3 = math.vec3_zero,
        rotation: math.Quat = math.quat_identity,
    };

    /// The face of the leaf named by `sub_shape_id` that faces `direction`
    /// (in this shape's own local space) the most. `out_vertices` must hold
    /// `max_supporting_face_vertices` entries; the returned slice is a view
    /// into it. Only convex shapes and triangles have one — a sphere, an
    /// empty shape, and so on report an empty slice, which is Jolt's own
    /// answer and not a failure of this call.
    pub fn supportingFace(
        self: Shape,
        sub_shape_id: c.SubShapeId,
        direction: math.Vec3,
        opts: SupportingFaceOptions,
        out_vertices: *[max_supporting_face_vertices]math.Vec3,
    ) err.Error![]math.Vec3 {
        var count: u32 = 0;
        try err.check(c.zjoltShapeGetSupportingFace(
            self.handle,
            sub_shape_id,
            &direction,
            optionalPtr(math.Vec3, &opts.scale),
            &opts.position,
            &opts.rotation,
            out_vertices,
            &count,
        ));
        return out_vertices[0..count];
    }

    pub const SubShapeTransformedShapeOptions = struct {
        /// The child's transform relative to this shape's own center of
        /// mass. Null keeps the origin / no rotation — the usual choice when
        /// this shape is not itself placed in the world yet.
        position: ?math.Vec3 = null,
        rotation: ?math.Quat = null,
        /// Null keeps (1, 1, 1).
        scale: ?math.Vec3 = null,
    };

    /// The direct child at `sub_shape_id`, as a fresh `TransformedShape` —
    /// `deinit` it when done. For a shape with no children this is the shape
    /// itself, wrapped at the given placement.
    ///
    /// The returned value's body id is always `invalid_body_id`: this
    /// relates two shapes, and there is no body on either side of it.
    pub fn subShapeTransformedShape(
        self: Shape,
        sub_shape_id: c.SubShapeId,
        opts: SubShapeTransformedShapeOptions,
    ) err.Error!struct { shape: TransformedShape, remainder: c.SubShapeId } {
        var handle: *c.TransformedShape = undefined;
        var remainder: c.SubShapeId = 0;
        try err.check(c.zjoltShapeGetSubShapeTransformedShape(
            self.handle,
            sub_shape_id,
            optionalPtr(math.Vec3, &opts.position),
            optionalPtr(math.Quat, &opts.rotation),
            optionalPtr(math.Vec3, &opts.scale),
            &handle,
            &remainder,
        ));
        return .{ .shape = .{ .handle = handle }, .remainder = remainder };
    }

    /// The innermost real shape at `sub_shape_id`, drilling through every
    /// compound and decoration — the identity-transform counterpart of
    /// `subShapeTransformedShape`.
    ///
    /// Borrowed: valid as long as this shape is — `addRef` to outlive it.
    /// Null if `sub_shape_id` did not resolve to a leaf, rather than an assert.
    pub fn leafShape(
        self: Shape,
        sub_shape_id: c.SubShapeId,
    ) ?struct { shape: Shape, remainder: c.SubShapeId } {
        var remainder: c.SubShapeId = 0;
        const handle = c.zjoltShapeGetLeafShape(self.handle, sub_shape_id, &remainder) orelse
            return null;
        return .{ .shape = .{ .handle = handle }, .remainder = remainder };
    }

    /// A copy of this shape, scaled by `scale` IN THE SPACE IT WAS CREATED —
    /// not the space of its leaves, which is what every other `scale` in
    /// this package means. Not every shape supports every scale; see
    /// `isValidScale`. Jolt matches the request as closely as it can rather
    /// than refusing an unsupported one.
    pub fn scaleShape(self: Shape, scale: math.Vec3) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeScaleShape(self.handle, &scale, &handle));
        return .{ .handle = handle };
    }

    /// Whether `scale` can be used directly for this shape — wrapped with
    /// `initScaled`, for instance — without Jolt substituting something else
    /// for it. A sphere or a capsule need a uniform scale; a compound
    /// refuses one that would shear a child.
    pub fn isValidScale(self: Shape, scale: math.Vec3) bool {
        return c.zjoltShapeIsValidScale(self.handle, &scale);
    }

    /// The scale nearest `scale` that `isValidScale` would accept for this
    /// shape. Compare the result against `scale` to detect a caller mistake
    /// worth a warning.
    pub fn makeScaleValid(self: Shape, scale: math.Vec3) math.Vec3 {
        var out: math.Vec3 = undefined;
        c.zjoltShapeMakeScaleValid(self.handle, &scale, &out);
        return out;
    }

    //=========================================================================
    // Triangle read-back
    //
    // The mesh-cooking path in reverse: read the triangles a shape is built
    // from — or approximates itself with, for a convex primitive — back out.
    //=========================================================================

    pub const TriangleWalkOptions = struct {
        position: math.Vec3 = math.vec3_zero,
        rotation: math.Quat = math.quat_identity,
        /// Null keeps (1, 1, 1).
        scale: ?math.Vec3 = null,
    };

    /// Starts a triangle walk over this shape, restricted to `box`. Call
    /// `next` repeatedly until it returns an empty slice.
    pub fn triangleWalk(
        self: Shape,
        box: math.AABox,
        opts: TriangleWalkOptions,
    ) err.Error!TriangleWalk {
        var walk: TriangleWalk = .{ .shape = self, .context = undefined };
        try err.check(c.zjoltShapeGetTrianglesStart(
            self.handle,
            &walk.context,
            &box,
            &opts.position,
            &opts.rotation,
            optionalPtr(math.Vec3, &opts.scale),
        ));
        return walk;
    }

    /// The materials this shape was built with, in the order `material` and
    /// `TriangleWalk.next` index them. Only a mesh or a height field has
    /// one; `error.InvalidArgument` for any other shape kind, including a
    /// compound of meshes — ask each leaf individually instead.
    pub fn materialList(
        self: Shape,
        buffer: []?PhysicsMaterial,
    ) err.Error![]?PhysicsMaterial {
        // `?PhysicsMaterial` and `?*const c.PhysicsMaterial` share a layout —
        // both are a single optional pointer — but that is a coincidence of
        // representation, not a promise, so the C call fills a raw buffer
        // and this converts it rather than reinterpreting the slice.
        var raw: [256]?*const c.PhysicsMaterial = undefined;
        var count: u32 = 0;
        try err.check(c.zjoltShapeGetMaterialList(
            self.handle,
            &raw,
            raw.len,
            &count,
        ));
        if (count > buffer.len) return err.Error.BufferTooSmall;
        for (raw[0..count], 0..) |m, i| {
            buffer[i] = if (m) |handle| .{ .handle = handle } else null;
        }
        return buffer[0..count];
    }

    //=========================================================================
    // Mesh specifics
    //=========================================================================

    /// The index into `materialList` that `sub_shape_id` names, for a mesh
    /// built with `triangle_materials`. 0 for any other shape kind, which is
    /// indistinguishable from a real index 0 — check `subType` first if the
    /// difference matters.
    pub fn meshMaterialIndex(self: Shape, sub_shape_id: c.SubShapeId) u32 {
        return c.zjoltShapeMeshGetMaterialIndex(self.handle, sub_shape_id);
    }

    /// The per-triangle user data a mesh keeps even though `initMesh` has no
    /// way to set it — it defaults to the triangle's own index in `indices`
    /// before Jolt's internal reordering. 0 for any other shape kind.
    pub fn meshTriangleUserData(self: Shape, sub_shape_id: c.SubShapeId) u32 {
        return c.zjoltShapeMeshGetTriangleUserData(self.handle, sub_shape_id);
    }

    //=========================================================================
    // Height field specifics
    //
    // Addressed by (x, y) sample coordinates, except
    // `heightFieldSubShapeCoordinates` (a hit's sub-shape id -> (x, y)).
    //=========================================================================

    /// Samples per side, after Jolt rounds the construction-time count up to
    /// a multiple of the block size. 0 for any other shape kind.
    pub fn heightFieldSampleCount(self: Shape) u32 {
        return c.zjoltShapeHeightFieldGetSampleCount(self.handle);
    }

    /// 0 for any other shape kind, which is never a real block size — Jolt's
    /// smallest is 2.
    pub fn heightFieldBlockSize(self: Shape) u32 {
        return c.zjoltShapeHeightFieldGetBlockSize(self.handle);
    }

    /// The range of height values this shape can encode. Both 0 for any
    /// other shape kind.
    pub fn heightFieldMinHeightValue(self: Shape) f32 {
        return c.zjoltShapeHeightFieldGetMinHeightValue(self.handle);
    }
    pub fn heightFieldMaxHeightValue(self: Shape) f32 {
        return c.zjoltShapeHeightFieldGetMaxHeightValue(self.handle);
    }

    /// The local-space position of sample (x, y), including its height.
    /// Zeroed for any other shape kind, or for (x, y) outside the grid.
    pub fn heightFieldPosition(self: Shape, x: u32, y: u32) math.Vec3 {
        var out: math.Vec3 = undefined;
        c.zjoltShapeHeightFieldGetPosition(self.handle, x, y, &out);
        return out;
    }

    /// Whether sample (x, y) is a hole. True (there is nothing to collide
    /// with) for any other shape kind or for (x, y) outside the grid.
    pub fn heightFieldIsNoCollision(self: Shape, x: u32, y: u32) bool {
        return c.zjoltShapeHeightFieldIsNoCollision(self.handle, x, y);
    }

    /// Drops `local_position` onto the surface. Null when `local_position`
    /// is outside the field's footprint or over a hole.
    pub fn heightFieldProjectOntoSurface(
        self: Shape,
        local_position: math.Vec3,
    ) err.Error!?struct { position: math.Vec3, sub_shape_id: c.SubShapeId } {
        var position: math.Vec3 = undefined;
        var sub_shape_id: c.SubShapeId = 0;
        var found: bool = false;
        try err.check(c.zjoltShapeHeightFieldProjectOntoSurface(
            self.handle,
            &local_position,
            &position,
            &sub_shape_id,
            &found,
        ));
        if (!found) return null;
        return .{ .position = position, .sub_shape_id = sub_shape_id };
    }

    /// The grid cell and which of its two triangles `sub_shape_id` names —
    /// the inverse of the encoding `heightFieldProjectOntoSurface` and a
    /// hit's sub-shape id both use.
    pub fn heightFieldSubShapeCoordinates(
        self: Shape,
        sub_shape_id: c.SubShapeId,
    ) err.Error!struct { x: u32, y: u32, triangle_index: u32 } {
        var x: u32 = 0;
        var y: u32 = 0;
        var triangle_index: u32 = 0;
        try err.check(c.zjoltShapeHeightFieldGetSubShapeCoordinates(
            self.handle,
            sub_shape_id,
            &x,
            &y,
            &triangle_index,
        ));
        return .{ .x = x, .y = y, .triangle_index = triangle_index };
    }

    /// Reads a `size_x` by `size_y` block of height samples starting at
    /// (x, y), row major into `out_heights[row * size_x + col]` (must hold
    /// `size_x * size_y` entries); a hole reads back as
    /// `height_field_no_collision`. `x`/`y` must be multiples of
    /// `heightFieldBlockSize` and the block must fit the grid, or
    /// `error.InvalidArgument`.
    pub fn heightFieldHeights(
        self: Shape,
        x: u32,
        y: u32,
        size_x: u32,
        size_y: u32,
        out_heights: []f32,
    ) err.Error!void {
        if (out_heights.len < @as(usize, size_x) * size_y)
            return err.Error.InvalidArgument;
        try err.check(c.zjoltShapeHeightFieldGetHeights(
            self.handle,
            x,
            y,
            size_x,
            size_y,
            out_heights.ptr,
            size_x,
        ));
    }

    /// Repaints height-field samples in place — craters, terrain deformation,
    /// without a rebuild. Same bounds as `heightFieldHeights`; values
    /// clamp into [`heightFieldMinHeightValue`, `heightFieldMaxHeightValue`].
    ///
    /// NOT thread safe against a query or step; needs `notifyShapeChanged`
    /// after.
    pub fn heightFieldSetHeights(
        self: Shape,
        x: u32,
        y: u32,
        size_x: u32,
        size_y: u32,
        heights: []const f32,
        active_edge_cos_threshold_angle: f32,
    ) err.Error!void {
        if (heights.len < @as(usize, size_x) * size_y)
            return err.Error.InvalidArgument;
        try err.check(c.zjoltShapeHeightFieldSetHeights(
            @constCast(self.handle),
            x,
            y,
            size_x,
            size_y,
            heights.ptr,
            size_x,
            active_edge_cos_threshold_angle,
        ));
    }
};

/// A running SubShapeIDCreator -- composes a multi-level sub-shape id one
/// level at a time, for addressing a nested compound's grandchild (or
/// deeper). Owned outright, not reference counted: `deinit` it.
pub const SubShapeIdCreator = struct {
    handle: *c.SubShapeIdCreator,

    /// The root of the chain -- no bits written yet.
    pub fn init() err.Error!SubShapeIdCreator {
        var handle: *c.SubShapeIdCreator = undefined;
        try err.check(c.zjoltSubShapeIdCreatorCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: SubShapeIdCreator) void {
        c.zjoltSubShapeIdCreatorDestroy(self.handle);
    }

    /// Direct bind of SubShapeIDCreator::PushID -- advances `self` in place.
    /// `value` must fit in `bits` bits, and the running total must not
    /// exceed 32; both are `error.InvalidArgument` rather than Jolt's
    /// asserts.
    pub fn push(self: SubShapeIdCreator, value: u32, bits: u32) err.Error!void {
        try err.check(c.zjoltSubShapeIdCreatorPushID(self.handle, value, bits));
    }

    pub fn id(self: SubShapeIdCreator) c.SubShapeId {
        return c.zjoltSubShapeIdCreatorGetID(self.handle);
    }

    pub fn numBitsWritten(self: SubShapeIdCreator) u32 {
        return c.zjoltSubShapeIdCreatorGetNumBitsWritten(self.handle);
    }
};

/// Submerged-volume accumulator for an arbitrary convex polyhedron -- what
/// `Shape.submergedVolume` runs internally for a shape already built,
/// exposed over a caller's own point cloud and face list. Owned outright,
/// not reference counted: `deinit`.
pub const PolyhedronSubmergedVolumeCalculator = struct {
    handle: *c.PolyhedronSubmergedVolumeCalculator,

    /// Transforms `points` by `transform` and classifies each against
    /// `surface` (normal pointing up).
    pub fn init(
        transform: math.Mat44,
        points: []const math.Vec3,
        surface: Plane,
    ) err.Error!PolyhedronSubmergedVolumeCalculator {
        var handle: *c.PolyhedronSubmergedVolumeCalculator = undefined;
        try err.check(c.zjoltPolyhedronSubmergedVolumeCalculatorCreate(
            &transform,
            points.ptr,
            @intCast(points.len),
            &surface,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: PolyhedronSubmergedVolumeCalculator) void {
        c.zjoltPolyhedronSubmergedVolumeCalculatorDestroy(self.handle);
    }

    /// True once every point sits above `surface` -- the submerged volume
    /// is zero without adding any faces.
    pub fn areAllAbove(self: PolyhedronSubmergedVolumeCalculator) bool {
        return c.zjoltPolyhedronSubmergedVolumeCalculatorAreAllAbove(self.handle);
    }

    /// True once every point sits below `surface` -- the submerged volume
    /// is the whole polyhedron's.
    pub fn areAllBelow(self: PolyhedronSubmergedVolumeCalculator) bool {
        return c.zjoltPolyhedronSubmergedVolumeCalculatorAreAllBelow(self.handle);
    }

    /// Index into `points` (as given to `init`) of the point deepest below
    /// `surface`. `addFace` refuses a face that uses it.
    pub fn referencePointIdx(self: PolyhedronSubmergedVolumeCalculator) u32 {
        return c.zjoltPolyhedronSubmergedVolumeCalculatorGetReferencePointIdx(self.handle);
    }

    /// Accumulates one triangular face, wound counter-clockwise, naming
    /// indices into `points` (as given to `init`). `error.InvalidArgument`
    /// for an out-of-range index or one equal to `referencePointIdx` --
    /// skip that face instead, its contribution is always zero.
    pub fn addFace(
        self: PolyhedronSubmergedVolumeCalculator,
        idx1: u32,
        idx2: u32,
        idx3: u32,
    ) err.Error!void {
        try err.check(c.zjoltPolyhedronSubmergedVolumeCalculatorAddFace(
            self.handle,
            idx1,
            idx2,
            idx3,
        ));
    }

    /// The accumulated submerged volume and its centre, after every face
    /// has been added.
    pub fn result(self: PolyhedronSubmergedVolumeCalculator) struct {
        submerged_volume: f32,
        center_of_buoyancy: math.Vec3,
    } {
        var submerged_volume: f32 = undefined;
        var center_of_buoyancy: math.Vec3 = undefined;
        c.zjoltPolyhedronSubmergedVolumeCalculatorGetResult(
            self.handle,
            &submerged_volume,
            &center_of_buoyancy,
        );
        return .{
            .submerged_volume = submerged_volume,
            .center_of_buoyancy = center_of_buoyancy,
        };
    }
};

//=============================================================================
// SIMD batch helpers Jolt's own compound and tree traversal build on
//
// Pure value math over a fixed 4-wide batch -- ports of
// Jolt/Physics/Collision/SortReverseAndStore.h, needing no shape or Jolt
// object to call.
//=============================================================================

/// Sorts `values` descending and keeps the entries below `max_value`,
/// permuting `identifiers` the same way -- `JPH::SortReverseAndStore` for a
/// 4-wide batch. `out_values` holds the kept entries at `[0, count)`, high
/// to low; trailing entries of both `out_values` and `identifiers` are
/// zeroed, not meaningful. Returns `count`.
pub fn sortReverseAndStore(
    in_values: [4]f32,
    max_value: f32,
    identifiers: *[4]u32,
    out_values: *[4]f32,
) usize {
    var values = in_values;
    var i: usize = 0;
    while (i < 3) : (i += 1) {
        var j: usize = 0;
        while (j < 3 - i) : (j += 1) {
            if (values[j] < values[j + 1]) {
                std.mem.swap(f32, &values[j], &values[j + 1]);
                std.mem.swap(u32, &identifiers[j], &identifiers[j + 1]);
            }
        }
    }

    // Sorted descending, so the entries below `max_value` form a suffix:
    // count the trailing run rather than a full scan.
    var count: usize = 0;
    while (count < 4 and values[3 - count] < max_value) : (count += 1) {}

    var shifted_values: [4]f32 = .{ 0, 0, 0, 0 };
    var shifted_ids: [4]u32 = .{ 0, 0, 0, 0 };
    var k: usize = 0;
    while (k < count) : (k += 1) {
        shifted_values[k] = values[k + 4 - count];
        shifted_ids[k] = identifiers[k + 4 - count];
    }
    out_values.* = shifted_values;
    identifiers.* = shifted_ids;
    return count;
}

/// Shifts `identifiers` so the ones whose flag in `mask` is true come
/// first, in their original relative order -- `JPH::CountAndSortTrues` for
/// a 4-wide batch. Trailing entries past the returned count are
/// unspecified. Returns how many were true.
pub fn countAndSortTrues(mask: [4]bool, identifiers: *[4]u32) usize {
    var shifted: [4]u32 = .{ 0, 0, 0, 0 };
    var count: usize = 0;
    for (mask, 0..) |m, idx| {
        if (m) {
            shifted[count] = identifiers[idx];
            count += 1;
        }
    }
    identifiers.* = shifted;
    return count;
}

/// One `Shape.triangleWalk` in progress. `shape` must outlive it, and it
/// must not be moved or reused for a second walk once started.
pub const TriangleWalk = struct {
    shape: Shape,
    context: c.ShapeTrianglesContext,

    /// Continues the walk into `out_vertices` (three consecutive vertices
    /// per triangle) and, if not null, `out_materials` (one per triangle,
    /// borrowed from `shape`). `max_triangles` comes from
    /// `out_vertices.len / 3`, which must be at least
    /// `min_triangles_requested`. An empty result means the walk is over,
    /// not "call again with a bigger buffer".
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
            // allows a call to return fewer triangles than requested with
            // more still to come, and this is exactly that.
            if (max_triangles > raw_materials.len)
                max_triangles = @intCast(raw_materials.len);
            materials_ptr = &raw_materials;
        }

        var count: u32 = 0;
        try err.check(c.zjoltShapeGetTrianglesNext(
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

/// A compound whose children can be added, removed, and moved after the fact
/// — a MUTABLE handle, cheaper to edit but costlier to query than static.
///
/// Every mutating method: NOT thread safe against a query, step, or each
/// other (hold `lockWrite`, or swap in a fresh compound via `setShape`);
/// invalidates every sub-shape id; needs `notifyShapeChanged` after.
pub const MutableCompound = struct {
    handle: *c.Shape,

    /// AT LEAST TWO CHILDREN, enforced rather than a preference — one child
    /// hits undefined behaviour in Jolt's sub-shape id sizing on ARM
    /// (`CountLeadingZeros(0)`). See UPSTREAM.md.
    pub fn init(children: []const CompoundChild) err.Error!MutableCompound {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateMutableCompound(
            children.ptr,
            @intCast(children.len),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// The same shape, seen as an ordinary immutable one — which is what a
    /// body, a query and a save all take.
    pub fn asShape(self: MutableCompound) Shape {
        return .{ .handle = self.handle };
    }

    pub fn addRef(self: MutableCompound) void {
        c.zjoltShapeAddRef(self.handle);
    }

    pub fn release(self: MutableCompound) void {
        c.zjoltShapeRelease(self.handle);
    }

    pub fn childCount(self: MutableCompound) u32 {
        return c.zjoltShapeCompoundGetNumChildren(self.handle);
    }

    /// Appends a child and returns its index.
    pub fn addChild(self: MutableCompound, child: CompoundChild) err.Error!u32 {
        var index: u32 = 0;
        try err.check(c.zjoltShapeMutableCompoundAddChild(self.handle, &child, &index));
        return index;
    }

    /// Removes the child at `index`, shifting the ones after it down.
    ///
    /// Refuses to go below two children, for the reason `init` gives: at one,
    /// Jolt's sub-shape id bit count reaches `CountLeadingZeros(0)`, which is
    /// undefined on ARM.
    pub fn removeChild(self: MutableCompound, index: u32) err.Error!void {
        try err.check(c.zjoltShapeMutableCompoundRemoveChild(self.handle, index));
    }

    /// Moves and reorients the child at `index`.
    pub fn moveChild(
        self: MutableCompound,
        index: u32,
        position: math.Vec3,
        rotation: math.Quat,
    ) err.Error!void {
        try err.check(c.zjoltShapeMutableCompoundMoveChild(
            self.handle,
            index,
            &position,
            &rotation,
        ));
    }

    /// Moves, reorients, AND swaps the shape occupying `index` for
    /// `new_shape` — the one thing `moveChild` cannot do. Prefer this over
    /// `removeChild` + `addChild`, which shifts every later child's
    /// sub-shape id; this does not. `new_shape` is borrowed for the call;
    /// the compound takes its own reference.
    pub fn replaceChild(
        self: MutableCompound,
        index: u32,
        new_shape: Shape,
        position: math.Vec3,
        rotation: math.Quat,
    ) err.Error!void {
        try err.check(c.zjoltShapeMutableCompoundReplaceChild(
            self.handle,
            index,
            new_shape.handle,
            &position,
            &rotation,
        ));
    }

    /// Recomputes the centre of mass and shifts the children around it.
    ///
    /// Needed after changing the children of a shape a DYNAMIC body uses, or
    /// the body spins about a point that is no longer its balance point. The
    /// centre of mass moves, so read `asShape().centerOfMass()` BEFORE calling
    /// this and hand the old value to `notifyShapeChanged`.
    pub fn adjustCenterOfMass(self: MutableCompound) err.Error!void {
        try err.check(c.zjoltShapeMutableCompoundAdjustCenterOfMass(self.handle));
    }
};
