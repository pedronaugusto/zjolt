//! Collision shapes.
//!
//! A shape is immutable, reference counted, and shareable between bodies and
//! between systems. Every constructor here returns one reference, which the
//! caller owns; adding a shape to a body takes its own, so the usual pattern
//! is create, create the body, `release`.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const material_mod = @import("material.zig");
const math = @import("math.zig");

pub const SubType = c.ShapeSubType;
pub const PhysicsMaterial = material_mod.PhysicsMaterial;

/// The height sample that punches a hole in a height field.
///
/// Jolt's own `FLT_MAX` sentinel. It lives here rather than in `c.zig`
/// because the ABI cross-check pairs constants by integer value and this one
/// is a float; the suite pins it behaviourally instead, by casting a ray at a
/// hole and missing.
pub const height_field_no_collision: f32 = std.math.floatMax(f32);

/// The sub-shape id meaning "the shape itself, no leaf below it".
///
/// Required rather than convenient: Jolt asserts on any other value for a
/// shape that has no leaves, so `sphere.material(0)` aborts a build with
/// asserts on. @see `Shape.material`.
pub const sub_shape_id_empty: c.SubShapeId = c.sub_shape_id_empty;

/// One child of a compound shape, in the parent's local space.
///
/// Layout-identical to the C struct, so an array of these crosses the boundary
/// with no copy and a compound constructor needs no allocator. Build one with
/// `compoundChild`.
pub const CompoundChild = c.CompoundChild;

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
    /// Const because every C entry point that READS a shape takes it as
    /// `const ZJoltShape *`. The one exception is a mutable compound, and that
    /// is reached through `MutableCompound` below, which holds a mutable
    /// handle of its own — so this stays const and no cast appears anywhere.
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

    /// A capsule whose caps have different radii, with the `top_radius` cap at
    /// `(0, half_height_of_tapered_cylinder, 0)`.
    ///
    /// Jolt SIMPLIFIES this one as it builds: when either sphere fully
    /// contains the other the result is a sphere, or a rotated-translated
    /// sphere. So `subType()` is not necessarily `.tapered_capsule` for a
    /// shape built here.
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

    /// A half space: everything on the negative side of
    /// `dot(x, normal) + constant = 0` is solid.
    ///
    /// Static or kinematic only — a half space has no volume to give a dynamic
    /// body mass. `normal` must be unit length; one that is not is
    /// `error.InvalidArgument` rather than quietly normalised, because a
    /// rescaled normal moves the surface away from where `constant` says it
    /// is.
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
    };

    /// A static triangle mesh. `indices` must be a multiple of three, and
    /// every index must be in range for `vertices` — both are checked here
    /// rather than left to fault inside Jolt's tree builder.
    ///
    /// A mesh shape may only be used by a static or kinematic body. Building
    /// it is the expensive part of collision cooking, which is why `save` and
    /// `restore` exist.
    ///
    /// A sub-shape id from a hit on this mesh names the triangle AFTER Jolt's
    /// own spatial reordering, so it is meaningful to `material` and to
    /// nothing else — in particular it is not an index into `indices`.
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
            if (opts.materials.len != 0) &handles else null,
            @intCast(opts.materials.len),
            opts.max_triangles_per_leaf,
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
    };

    /// A static height field of `sample_count` x `sample_count` samples, laid
    /// out row major so `(x, y)` is `samples[y * sample_count + x]`.
    ///
    /// A sample of `height_field_no_collision` punches a hole. `sample_count`
    /// need not be a multiple of `opts.block_size` — Jolt rounds it up and
    /// pads the difference with holes — but the rounded count divided by the
    /// block size must be at least 2, which makes 4 the smallest useful
    /// `sample_count` at the default block size.
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
            &handle,
        ));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Compounds
    //=========================================================================

    /// A compound whose children are fixed once built, stored in a tree.
    ///
    /// Jolt SIMPLIFIES the one-child case: a single child at the origin with
    /// no rotation comes back as that child itself, and one that is moved or
    /// rotated comes back as a rotated-translated shape. So `subType()` is not
    /// necessarily `.static_compound` for a shape built here. An empty
    /// `children` is `error.InvalidArgument`.
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

    //=========================================================================
    // Introspection
    //=========================================================================

    pub fn subType(self: Shape) SubType {
        return c.zjoltShapeGetSubType(self.handle);
    }

    /// The material of one leaf of this shape. Never null for a valid shape.
    ///
    /// `sub_shape_id` comes from a hit — `RayCastHit.sub_shape_id` and its
    /// friends — or from a contact manifold. For a shape with no leaves, which
    /// is every convex primitive and a plane, pass `sub_shape_id_empty`: Jolt
    /// ASSERTS the id is empty there, so passing 0 aborts a build with asserts
    /// on rather than returning anything. An id naming a child a compound does
    /// not have asserts the same way, so this is for ids Jolt handed you, not
    /// ids you composed.
    ///
    /// A shape built without a material answers with the shared default rather
    /// than null; compare against `PhysicsMaterial.default()` to tell them
    /// apart. The material is borrowed from the shape — `addRef` it to outlive
    /// one.
    pub fn material(self: Shape, sub_shape_id: c.SubShapeId) ?PhysicsMaterial {
        const handle = c.zjoltShapeGetMaterial(self.handle, sub_shape_id) orelse
            return null;
        return .{ .handle = handle };
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
    // Serialisation
    //
    // The format is Jolt's own binary shape state. It is tied to the vendored
    // Jolt version and to the double-precision setting — it is a cooking
    // cache, not an interchange format. See UPSTREAM.md.
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

    /// Rebuilds a shape from `save` output. A truncated buffer, or one with
    /// trailing bytes, is `error.BadFormat` rather than a partially parsed
    /// shape.
    pub fn restore(data: []const u8) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeRestore(data.ptr, data.len, &handle));
        return .{ .handle = handle };
    }
};

/// A compound whose children can be added, removed and moved after the fact.
///
/// A separate type rather than more methods on `Shape`, because it is the one
/// shape that is not immutable and the distinction is worth carrying in the
/// type. It holds a MUTABLE handle, which is what lets `Shape` keep a const
/// one and lets neither side need a cast: `asShape` widens, and nothing
/// narrows.
///
/// Cheaper to modify and more expensive to query than a static compound; reach
/// for it when the shape genuinely changes, not merely because it is built at
/// run time.
///
/// **Every mutating method below carries three obligations.** They are not
/// thread safe against a query, a step, or each other — hold `lockWrite` on
/// any body using the shape, or better, follow Jolt's own advice and build a
/// fresh compound to swap in with `setShape`, so a query already running keeps
/// the old one alive through its own reference. They invalidate every
/// sub-shape id into the shape, because indices shift. And each body using the
/// shape needs `BodyInterface.notifyShapeChanged` afterwards, or the broad
/// phase and the contact cache go on describing the old geometry.
pub const MutableCompound = struct {
    handle: *c.Shape,

    /// AT LEAST TWO CHILDREN, which is upstream's constraint rather than a
    /// preference. A compound sizes the index field of its sub-shape ids as
    /// `32 - CountLeadingZeros(count - 1)`; Jolt's `CountLeadingZeros` guards
    /// a zero argument on x86 but not on ARM, where it is a bare
    /// `__builtin_clz` and zero is undefined. One child makes that argument
    /// zero, and none underflows the subtraction before it. A static compound
    /// never runs into this because Jolt simplifies one child away; this one
    /// does not simplify, so the floor is enforced at the boundary. See
    /// UPSTREAM.md.
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
