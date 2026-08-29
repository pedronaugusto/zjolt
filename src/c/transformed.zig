//! ZJolt C declarations for a shape with a placement, queried on its own with no physics system.
//!
//! Mirrors `ffi/zjolt_transformed.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");
const query = @import("query.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const AABox = core.AABox;
pub const BodyId = core.BodyId;
pub const PhysicsMaterial = core.PhysicsMaterial;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Shape = core.Shape;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
pub const CollidePointHit = query.CollidePointHit;
pub const CollideShapeHit = query.CollideShapeHit;
pub const CollideShapeSettings = query.CollideShapeSettings;
pub const RayCastHit = query.RayCastHit;
pub const RayCastSettings = query.RayCastSettings;
pub const ShapeCastHit = query.ShapeCastHit;
pub const ShapeCastSettings = query.ShapeCastSettings;
pub const ShapeFilter = query.ShapeFilter;

/// The same scratch space, for a walk over a `TransformedShape` instead. A
/// second type rather than the one above reused, matching the C header: the
/// two query surfaces are independent, so nothing here assumes they share a
/// layout even though they happen to today.
pub const TransformedShapeTrianglesContext = extern struct {
    data: [4288]u8 align(16) = undefined,
};

pub const TransformedShape = opaque {};

pub const TransformedShapeTransform = extern struct {
    position: RVec3,
    rotation: Quat,
    scale: Vec3,
};

pub extern fn zjoltTransformedShapeCreate(shape: *const Shape, position: *const RVec3, rotation: *const Quat, scale: ?*const Vec3, body: BodyId, out: **TransformedShape) Result;

pub extern fn zjoltTransformedShapeDestroy(ts: ?*TransformedShape) void;

pub extern fn zjoltTransformedShapeGetWorldTransform(ts: *const TransformedShape, out: *TransformedShapeTransform) void;

pub extern fn zjoltTransformedShapeSetWorldTransform(ts: *TransformedShape, position: *const RVec3, rotation: *const Quat, scale: ?*const Vec3) void;

pub extern fn zjoltTransformedShapeGetWorldSpaceBounds(ts: *const TransformedShape, out: *AABox) void;

pub extern fn zjoltTransformedShapeGetWorldSpaceSurfaceNormal(ts: *const TransformedShape, sub_shape_id: SubShapeId, position: *const RVec3, out_normal: *Vec3) void;

pub extern fn zjoltTransformedShapeGetMaterial(ts: *const TransformedShape, sub_shape_id: SubShapeId) ?*const PhysicsMaterial;

pub extern fn zjoltTransformedShapeGetSubShapeUserData(ts: *const TransformedShape, sub_shape_id: SubShapeId) u64;

pub extern fn zjoltTransformedShapeGetSupportingFace(ts: *const TransformedShape, sub_shape_id: SubShapeId, direction: *const Vec3, base_offset: *const RVec3, out_vertices: [*]Vec3, out_count: *u32) Result;

pub extern fn zjoltTransformedShapeCollectTransformedShapes(ts: *const TransformedShape, box: *const AABox, filter: ?*const ShapeFilter, out_shapes: ?[*]?*TransformedShape, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltTransformedShapeGetTrianglesStart(ts: *const TransformedShape, context: *TransformedShapeTrianglesContext, box: *const AABox, base_offset: *const RVec3) Result;

pub extern fn zjoltTransformedShapeGetTrianglesNext(ts: *const TransformedShape, context: *TransformedShapeTrianglesContext, max_triangles: u32, out_vertices: [*]Vec3, out_materials: ?[*]?*const PhysicsMaterial, out_count: *u32) Result;

pub extern fn zjoltTransformedShapeCastRayClosest(ts: *const TransformedShape, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filter: ?*const ShapeFilter, out_hit: *RayCastHit, out_hit_any: *bool) Result;

pub extern fn zjoltTransformedShapeCastRayAll(ts: *const TransformedShape, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filter: ?*const ShapeFilter, out_hits: ?[*]RayCastHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltTransformedShapeCollidePointAll(ts: *const TransformedShape, point: *const RVec3, filter: ?*const ShapeFilter, out_hits: ?[*]CollidePointHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltTransformedShapeCollideShapeAll(ts: *const TransformedShape, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, base_offset: *const RVec3, settings: ?*const CollideShapeSettings, filter: ?*const ShapeFilter, out_hits: ?[*]CollideShapeHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltTransformedShapeCastShapeClosest(ts: *const TransformedShape, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, base_offset: *const RVec3, settings: ?*const ShapeCastSettings, filter: ?*const ShapeFilter, out_hit: *ShapeCastHit, out_hit_any: *bool) Result;

pub extern fn zjoltTransformedShapeCastShapeAll(ts: *const TransformedShape, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, base_offset: *const RVec3, settings: ?*const ShapeCastSettings, filter: ?*const ShapeFilter, out_hits: ?[*]ShapeCastHit, capacity: u32, out_count: *u32) Result;
