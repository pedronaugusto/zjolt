//! ZJolt C declarations for host-defined shapes.
//!
//! Mirrors `ffi/zjolt_customshape.h` exactly: a declaration belongs to the
//! module named after the header that declares it, so there is nothing to
//! decide and nothing to drift. `src/c.zig` lists every one of these and is
//! what the ABI cross-check and the misuse sweep walk.

const core = @import("core.zig");
const query = @import("query.zig");
const shape_mod = @import("shape.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const AABox = core.AABox;
pub const Mat44 = core.Mat44;
pub const MassProperties = core.MassProperties;
pub const PhysicsMaterial = core.PhysicsMaterial;
pub const Quat = core.Quat;
pub const Result = core.Result;
pub const Shape = core.Shape;
pub const ShapeStats = shape_mod.ShapeStats;
pub const ShapeTrianglesContext = shape_mod.ShapeTrianglesContext;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
pub const Plane = shape_mod.Plane;
pub const RayCastSettings = query.RayCastSettings;

//===----------------------------------------------------------------------===//
// Layer A — custom convex shape
//===----------------------------------------------------------------------===//

pub const ConvexShapeCallbacks = extern struct {
    support: ?*const fn (user: ?*anyopaque, direction: Vec3) callconv(.c) Vec3 = null,
    supporting_face: ?*const fn (
        user: ?*anyopaque,
        direction: Vec3,
        scale: Vec3,
        out_vertices: [*]Vec3,
        max_vertices: u32,
    ) callconv(.c) u32 = null,
    inner_radius: ?*const fn (user: ?*const anyopaque) callconv(.c) f32 = null,
    local_bounds: ?*const fn (user: ?*const anyopaque, out: *AABox) callconv(.c) void = null,
    mass_properties: ?*const fn (user: ?*const anyopaque, out: *MassProperties) callconv(.c) void = null,
    volume: ?*const fn (user: ?*const anyopaque) callconv(.c) f32 = null,
    destroy: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
};

pub extern fn zjoltShapeCreateCustomConvex(callbacks: *const ConvexShapeCallbacks, user: ?*anyopaque, material: ?*const PhysicsMaterial, out: **Shape) Result;

//===----------------------------------------------------------------------===//
// Layer B — general custom shape
//===----------------------------------------------------------------------===//

pub const custom_shape_max_batch: u32 = 32;

pub const CustomShapeRayHit = extern struct {
    fraction: f32,
    sub_shape_id: SubShapeId,
};

pub const CustomShapeChild = extern struct {
    shape: ?*const Shape,
    position: Vec3,
    rotation: Quat,
    scale: Vec3,
    sub_shape_id: SubShapeId,
};

pub const ShapeCallbacks = extern struct {
    // Required — Jolt declares each of these pure virtual.
    local_bounds: ?*const fn (user: ?*const anyopaque, out: *AABox) callconv(.c) void = null,
    sub_shape_id_bits_recursive: ?*const fn (user: ?*const anyopaque) callconv(.c) u32 = null,
    inner_radius: ?*const fn (user: ?*const anyopaque) callconv(.c) f32 = null,
    mass_properties: ?*const fn (user: ?*const anyopaque, out: *MassProperties) callconv(.c) void = null,
    get_material: ?*const fn (user: ?*const anyopaque, sub_shape_id: SubShapeId) callconv(.c) ?*const PhysicsMaterial = null,
    surface_normal: ?*const fn (user: ?*const anyopaque, sub_shape_id: SubShapeId, local_surface_position: Vec3, out_normal: *Vec3) callconv(.c) void = null,
    submerged_volume: ?*const fn (
        user: ?*const anyopaque,
        center_of_mass_transform: *const Mat44,
        scale: Vec3,
        surface: *const Plane,
        out_total_volume: *f32,
        out_submerged_volume: *f32,
        out_center_of_buoyancy: *Vec3,
    ) callconv(.c) void = null,
    cast_ray_closest: ?*const fn (
        user: ?*const anyopaque,
        origin: Vec3,
        direction: Vec3,
        max_fraction: f32,
        out_fraction: *f32,
        out_sub_shape_id: *SubShapeId,
    ) callconv(.c) bool = null,
    cast_ray_all: ?*const fn (
        user: ?*const anyopaque,
        origin: Vec3,
        direction: Vec3,
        settings: *const RayCastSettings,
        out_hits: [*]CustomShapeRayHit,
        max_hits: u32,
    ) callconv(.c) u32 = null,
    collide_point: ?*const fn (
        user: ?*const anyopaque,
        point: Vec3,
        out_sub_shape_ids: [*]SubShapeId,
        max_hits: u32,
    ) callconv(.c) u32 = null,
    collide_soft_body_vertices: ?*const fn (
        user: ?*const anyopaque,
        scale: Vec3,
        local_positions: [*]const Vec3,
        inv_mass: [*]const f32,
        count: u32,
        out_penetration: [*]f32,
        out_normal: [*]Vec3,
    ) callconv(.c) void = null,
    get_triangles_start: ?*const fn (
        user: ?*const anyopaque,
        context: *ShapeTrianglesContext,
        box: *const AABox,
        position: ?*const Vec3,
        rotation: ?*const Quat,
        scale: ?*const Vec3,
    ) callconv(.c) Result = null,
    get_triangles_next: ?*const fn (
        user: ?*const anyopaque,
        context: *ShapeTrianglesContext,
        max_triangles: u32,
        out_vertices: [*]Vec3,
        out_materials: ?[*]?*const PhysicsMaterial,
        out_count: *u32,
    ) callconv(.c) Result = null,
    get_stats: ?*const fn (user: ?*const anyopaque, out: *ShapeStats) callconv(.c) void = null,
    volume: ?*const fn (user: ?*const anyopaque) callconv(.c) f32 = null,

    // Optional — null uses Jolt's own default for the virtual it stands in for.
    must_be_static: ?*const fn (user: ?*const anyopaque) callconv(.c) bool = null,
    center_of_mass: ?*const fn (user: ?*const anyopaque) callconv(.c) Vec3 = null,
    world_space_bounds: ?*const fn (
        user: ?*const anyopaque,
        center_of_mass_transform: *const Mat44,
        scale: Vec3,
        out: *AABox,
    ) callconv(.c) void = null,
    supporting_face: ?*const fn (
        user: ?*const anyopaque,
        sub_shape_id: SubShapeId,
        direction: Vec3,
        scale: Vec3,
        out_vertices: [*]Vec3,
        max_vertices: u32,
    ) callconv(.c) u32 = null,
    sub_shape_user_data: ?*const fn (user: ?*const anyopaque, sub_shape_id: SubShapeId) callconv(.c) u64 = null,
    collect_transformed_shapes: ?*const fn (
        user: ?*const anyopaque,
        box: *const AABox,
        position: *const Vec3,
        rotation: *const Quat,
        scale: Vec3,
        out_children: [*]CustomShapeChild,
        max_children: u32,
    ) callconv(.c) u32 = null,
    transform_shape: ?*const fn (
        user: ?*const anyopaque,
        center_of_mass_transform: *const Mat44,
        out_children: [*]CustomShapeChild,
        max_children: u32,
    ) callconv(.c) u32 = null,
    is_valid_scale: ?*const fn (user: ?*const anyopaque, scale: Vec3) callconv(.c) bool = null,
    make_scale_valid: ?*const fn (user: ?*const anyopaque, scale: Vec3, out_scale: *Vec3) callconv(.c) void = null,
    save_binary_state: ?*const fn (user: ?*const anyopaque, stream: *const core.Stream) callconv(.c) void = null,
    save_material_state: ?*const fn (
        user: ?*const anyopaque,
        out_materials: [*]?*const PhysicsMaterial,
        max_materials: u32,
    ) callconv(.c) u32 = null,
    save_sub_shape_state: ?*const fn (
        user: ?*const anyopaque,
        out_shapes: [*]?*const Shape,
        max_shapes: u32,
    ) callconv(.c) u32 = null,
    destroy: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
};

pub extern fn zjoltShapeCreateCustom(callbacks: *const ShapeCallbacks, user: ?*anyopaque, out: **Shape) Result;
