//! ZJolt C declarations for ray casts, shape casts, overlap and point tests against a physics system.
//!
//! Mirrors `ffi/zjolt_query.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const BackFaceMode = core.BackFaceMode;
pub const BodyId = core.BodyId;
pub const BroadPhaseLayer = core.BroadPhaseLayer;
pub const ObjectLayer = core.ObjectLayer;
pub const PhysicsMaterial = core.PhysicsMaterial;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Shape = core.Shape;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
pub const sub_shape_id_empty = core.sub_shape_id_empty;

pub const BroadPhaseLayerFilter = extern struct {
    should_collide: ?*const fn (user: ?*anyopaque, layer: BroadPhaseLayer) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const ObjectLayerFilter = extern struct {
    should_collide: ?*const fn (user: ?*anyopaque, layer: ObjectLayer) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const BodyFilter = extern struct {
    should_collide: ?*const fn (user: ?*anyopaque, body: BodyId) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const ShapeFilter = extern struct {
    should_collide: ?*const fn (
        user: ?*anyopaque,
        body: BodyId,
        sub_shape_id: SubShapeId,
        query_sub_shape_id: SubShapeId,
    ) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const QueryFilters = extern struct {
    broad_phase_layer: BroadPhaseLayerFilter = .{},
    object_layer: ObjectLayerFilter = .{},
    body: BodyFilter = .{},
    shape: ShapeFilter = .{},
};

pub const HitAction = enum(c_int) {
    @"continue" = 0,
    narrow = 1,
    stop = 2,
};

/// The defaults are Jolt's own, restated here so a Zig caller can write
/// `.{ .back_face_mode_convex = .collide }` and leave the rest alone. A test in
/// `query.zig` checks them against `zjoltRayCastSettingsInit`, so this copy
/// cannot quietly drift from the library's.
pub const RayCastSettings = extern struct {
    back_face_mode_triangles: BackFaceMode = .ignore,
    back_face_mode_convex: BackFaceMode = .ignore,
    treat_convex_as_solid: bool = true,
};

/// @see `ZJoltActiveEdgeMode` in the header for what an active edge is.
pub const ActiveEdgeMode = enum(c_int) {
    collide_only_with_active = 0,
    collide_with_all = 1,
};

/// Note the numbering: `collect_faces` is 0, so a zeroed settings struct asks
/// for faces. That is Jolt's own numbering, mirrored rather than tidied.
pub const CollectFacesMode = enum(c_int) {
    collect_faces = 0,
    no_faces = 1,
};

/// Jolt's own defaults, restated so a caller can set one field and leave the
/// rest. The test in `query.zig` checks them against
/// `zjoltCollideShapeSettingsInit`, so this copy cannot quietly drift.
pub const CollideShapeSettings = extern struct {
    active_edge_mode: ActiveEdgeMode = .collide_only_with_active,
    collect_faces_mode: CollectFacesMode = .no_faces,
    collision_tolerance: f32 = 1.0e-4,
    penetration_tolerance: f32 = 1.0e-4,
    active_edge_movement_direction: Vec3 = .{ .x = 0, .y = 0, .z = 0 },
    max_separation_distance: f32 = 0,
    back_face_mode: BackFaceMode = .ignore,
};

/// @see `CollideShapeSettings` for the drift check these defaults are under.
pub const ShapeCastSettings = extern struct {
    active_edge_mode: ActiveEdgeMode = .collide_only_with_active,
    collect_faces_mode: CollectFacesMode = .no_faces,
    collision_tolerance: f32 = 1.0e-4,
    penetration_tolerance: f32 = 1.0e-4,
    active_edge_movement_direction: Vec3 = .{ .x = 0, .y = 0, .z = 0 },
    extra_convex_radius: f32 = 0,
    back_face_mode_triangles: BackFaceMode = .ignore,
    back_face_mode_convex: BackFaceMode = .ignore,
    use_shrunken_shape_and_convex_radius: bool = false,
    return_deepest_point: bool = false,
};

pub const RayCastHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    fraction: f32,
    normal: Vec3,
    material: ?*const PhysicsMaterial,
};

pub const ShapeCastHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    fraction: f32,
    contact_point_on_1: Vec3,
    contact_point_on_2: Vec3,
    penetration_axis: Vec3,
    penetration_depth: f32,
    is_back_face_hit: bool,
    material: ?*const PhysicsMaterial,
};

pub const CollideShapeHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    contact_point_on_1: Vec3,
    contact_point_on_2: Vec3,
    penetration_axis: Vec3,
    penetration_depth: f32,
    material: ?*const PhysicsMaterial,
};

pub const CollidePointHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    material: ?*const PhysicsMaterial,
};

pub const RayCastHitFn = *const fn (user: ?*anyopaque, hit: *const RayCastHit) callconv(.c) HitAction;

pub const ShapeCastHitFn = *const fn (user: ?*anyopaque, hit: *const ShapeCastHit) callconv(.c) HitAction;

pub const CollideShapeHitFn = *const fn (user: ?*anyopaque, hit: *const CollideShapeHit) callconv(.c) HitAction;

pub const CollidePointHitFn = *const fn (user: ?*anyopaque, hit: *const CollidePointHit) callconv(.c) HitAction;

pub extern fn zjoltRayCastSettingsInit(settings: *RayCastSettings) void;

pub extern fn zjoltCastRayClosest(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filters: ?*const QueryFilters, out_hit: *RayCastHit, out_hit_any: *bool) Result;

pub extern fn zjoltCastRayAll(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filters: ?*const QueryFilters, out_hits: ?[*]RayCastHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltCastRayEach(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filters: ?*const QueryFilters, on_hit: RayCastHitFn, user: ?*anyopaque) Result;

pub extern fn zjoltCastShapeClosest(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, settings: ?*const ShapeCastSettings, filters: ?*const QueryFilters, out_hit: *ShapeCastHit, out_hit_any: *bool) Result;

pub extern fn zjoltCastShapeAll(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, settings: ?*const ShapeCastSettings, filters: ?*const QueryFilters, out_hits: ?[*]ShapeCastHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltCastShapeEach(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, settings: ?*const ShapeCastSettings, filters: ?*const QueryFilters, on_hit: ShapeCastHitFn, user: ?*anyopaque) Result;

pub extern fn zjoltCollideShapeClosest(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, settings: ?*const CollideShapeSettings, filters: ?*const QueryFilters, out_hit: *CollideShapeHit, out_hit_any: *bool) Result;

pub extern fn zjoltCollideShapeAll(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, settings: ?*const CollideShapeSettings, filters: ?*const QueryFilters, out_hits: ?[*]CollideShapeHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltCollideShapeEach(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, settings: ?*const CollideShapeSettings, filters: ?*const QueryFilters, on_hit: CollideShapeHitFn, user: ?*anyopaque) Result;

pub extern fn zjoltCollidePointAll(system: *const PhysicsSystem, point: *const RVec3, filters: ?*const QueryFilters, out_hits: ?[*]CollidePointHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltCollidePointEach(system: *const PhysicsSystem, point: *const RVec3, filters: ?*const QueryFilters, on_hit: CollidePointHitFn, user: ?*anyopaque) Result;

pub extern fn zjoltCollideShapeSettingsInit(settings: *CollideShapeSettings) void;

pub extern fn zjoltShapeCastSettingsInit(settings: *ShapeCastSettings) void;

pub extern fn zjoltCollideShapeVsShapeClosest(shape1: *const Shape, scale1: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, shape2: *const Shape, scale2: ?*const Vec3, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, settings: ?*const CollideShapeSettings, filter: ?*const ShapeFilter, out_hit: *CollideShapeHit, out_hit_any: *bool) Result;

pub extern fn zjoltCollideShapeVsShapeAll(shape1: *const Shape, scale1: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, shape2: *const Shape, scale2: ?*const Vec3, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, settings: ?*const CollideShapeSettings, filter: ?*const ShapeFilter, out_hits: ?[*]CollideShapeHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltCastShapeVsShapeClosest(shape1: *const Shape, scale1: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, direction: *const Vec3, shape2: *const Shape, scale2: ?*const Vec3, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, settings: ?*const ShapeCastSettings, filter: ?*const ShapeFilter, out_hit: *ShapeCastHit, out_hit_any: *bool) Result;

pub extern fn zjoltCastShapeVsShapeAll(shape1: *const Shape, scale1: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, direction: *const Vec3, shape2: *const Shape, scale2: ?*const Vec3, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, settings: ?*const ShapeCastSettings, filter: ?*const ShapeFilter, out_hits: ?[*]ShapeCastHit, capacity: u32, out_count: *u32) Result;
