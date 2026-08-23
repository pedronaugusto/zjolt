//! Collision shapes.
//!
//! A shape is immutable, reference counted, and shareable between bodies and
//! between systems. Every constructor here returns one reference, which the
//! caller owns; adding a shape to a body takes its own, so the usual pattern
//! is create, create the body, `release`.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");

pub const SubType = c.ShapeSubType;

pub const Shape = struct {
    /// Const because every C entry point that takes a shape takes it as
    /// `const ZJoltShape *` — a shape is immutable once built. Holding a
    /// mutable pointer would buy nothing and cost a cast everywhere a shape
    /// comes back out of the library.
    handle: *const c.Shape,

    //=========================================================================
    // Convex primitives
    //=========================================================================

    pub const BoxOptions = struct {
        /// Rounds the corners for cheaper, more stable collision. Must not
        /// exceed the smallest half extent.
        convex_radius: f32 = 0.05,
        /// kg/m^3. Zero keeps Jolt's default of 1000.
        density: f32 = 0,
    };

    pub fn initBox(half_extent: math.Vec3, opts: BoxOptions) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateBox(
            &half_extent,
            opts.convex_radius,
            opts.density,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn initSphere(radius: f32, density: f32) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateSphere(radius, density, &handle));
        return .{ .handle = handle };
    }

    /// A capsule along the Y axis: a cylinder of `half_height_of_cylinder`
    /// capped by hemispheres of `radius`. Total height is
    /// `2 * (half_height_of_cylinder + radius)`.
    pub fn initCapsule(half_height_of_cylinder: f32, radius: f32, density: f32) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateCapsule(
            half_height_of_cylinder,
            radius,
            density,
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
            &handle,
        ));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Mesh
    //=========================================================================

    /// A static triangle mesh. `indices` must be a multiple of three, and
    /// every index must be in range for `vertices` — both are checked here
    /// rather than left to fault inside Jolt's tree builder.
    ///
    /// A mesh shape may only be used by a static or kinematic body. Building
    /// it is the expensive part of collision cooking, which is why `save` and
    /// `restore` exist.
    pub fn initMesh(
        vertices: []const math.Vec3,
        indices: []const u32,
        max_triangles_per_leaf: u32,
    ) err.Error!Shape {
        if (indices.len % 3 != 0) return err.Error.InvalidArgument;
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateMesh(
            vertices.ptr,
            @intCast(vertices.len),
            indices.ptr,
            @intCast(indices.len / 3),
            max_triangles_per_leaf,
            &handle,
        ));
        return .{ .handle = handle };
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
