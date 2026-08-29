//! ZJolt C declarations for triangle collision, active-edge normals, and internal-edge removal.
//!
//! Mirrors `ffi/zjolt_collision.h` exactly: a declaration belongs to the
//! module named after the header that declares it, so there is nothing to
//! decide and nothing to drift. `src/c.zig` lists every one of these and is
//! what the ABI cross-check and the misuse sweep walk.

const core = @import("core.zig");
const query = @import("query.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Quat = core.Quat;
pub const Result = core.Result;
pub const RVec3 = core.RVec3;
pub const Shape = core.Shape;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
pub const CollideShapeHitFn = query.CollideShapeHitFn;
pub const CollideShapeSettings = query.CollideShapeSettings;
pub const ShapeCastHitFn = query.ShapeCastHitFn;
pub const ShapeCastSettings = query.ShapeCastSettings;
pub const ShapeFilter = query.ShapeFilter;

pub const CollideConvexVsTriangles = opaque {};
pub const CastConvexVsTriangles = opaque {};
pub const CollideSphereVsTriangles = opaque {};
pub const CastSphereVsTriangles = opaque {};

//=============================================================================
// Active edges
//=============================================================================

pub extern fn zjoltActiveEdgesIsEdgeActive(normal1: *const Vec3, normal2: *const Vec3, edge_direction: *const Vec3, cos_threshold_angle: f32) bool;

pub extern fn zjoltActiveEdgesFixNormal(v0: *const Vec3, v1: *const Vec3, v2: *const Vec3, triangle_normal: *const Vec3, active_edges: u8, point: *const Vec3, normal: *const Vec3, movement_direction: *const Vec3, out_normal: *Vec3) void;

//=============================================================================
// Triangle collision
//=============================================================================

pub extern fn zjoltCollideConvexVsTrianglesCreate(shape1: *const Shape, scale1: ?*const Vec3, scale2: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, sub_shape_id1: SubShapeId, settings: ?*const CollideShapeSettings, on_hit: CollideShapeHitFn, user: ?*anyopaque, out: **CollideConvexVsTriangles) Result;

pub extern fn zjoltCollideConvexVsTrianglesCollide(collider: *CollideConvexVsTriangles, v0: *const Vec3, v1: *const Vec3, v2: *const Vec3, active_edges: u8, sub_shape_id2: SubShapeId, out_should_stop: *bool) Result;

pub extern fn zjoltCollideConvexVsTrianglesDestroy(collider: ?*CollideConvexVsTriangles) void;

pub extern fn zjoltCastConvexVsTrianglesCreate(shape1: *const Shape, scale1: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, direction: *const Vec3, scale2: ?*const Vec3, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, settings: ?*const ShapeCastSettings, on_hit: ShapeCastHitFn, user: ?*anyopaque, out: **CastConvexVsTriangles) Result;

pub extern fn zjoltCastConvexVsTrianglesCast(caster: *CastConvexVsTriangles, v0: *const Vec3, v1: *const Vec3, v2: *const Vec3, active_edges: u8, sub_shape_id2: SubShapeId, out_should_stop: *bool) Result;

pub extern fn zjoltCastConvexVsTrianglesDestroy(caster: ?*CastConvexVsTriangles) void;

pub extern fn zjoltCollideSphereVsTrianglesCreate(shape1: *const Shape, scale1: ?*const Vec3, scale2: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, sub_shape_id1: SubShapeId, settings: ?*const CollideShapeSettings, on_hit: CollideShapeHitFn, user: ?*anyopaque, out: **CollideSphereVsTriangles) Result;

pub extern fn zjoltCollideSphereVsTrianglesCollide(collider: *CollideSphereVsTriangles, v0: *const Vec3, v1: *const Vec3, v2: *const Vec3, active_edges: u8, sub_shape_id2: SubShapeId, out_should_stop: *bool) Result;

pub extern fn zjoltCollideSphereVsTrianglesDestroy(collider: ?*CollideSphereVsTriangles) void;

pub extern fn zjoltCastSphereVsTrianglesCreate(shape1: *const Shape, scale1: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, direction: *const Vec3, scale2: ?*const Vec3, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, settings: ?*const ShapeCastSettings, on_hit: ShapeCastHitFn, user: ?*anyopaque, out: **CastSphereVsTriangles) Result;

pub extern fn zjoltCastSphereVsTrianglesCast(caster: *CastSphereVsTriangles, v0: *const Vec3, v1: *const Vec3, v2: *const Vec3, active_edges: u8, sub_shape_id2: SubShapeId, out_should_stop: *bool) Result;

pub extern fn zjoltCastSphereVsTrianglesDestroy(caster: ?*CastSphereVsTriangles) void;

//=============================================================================
// Ghost-collision removal
//=============================================================================

pub extern fn zjoltCollideShapeWithInternalEdgeRemoval(shape1: *const Shape, scale1: ?*const Vec3, position1: *const RVec3, rotation1: *const Quat, shape2: *const Shape, scale2: ?*const Vec3, position2: *const RVec3, rotation2: *const Quat, base_offset: ?*const RVec3, settings: ?*const CollideShapeSettings, filter: ?*const ShapeFilter, on_hit: CollideShapeHitFn, user: ?*anyopaque) Result;
