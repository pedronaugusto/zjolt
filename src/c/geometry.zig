//! ZJolt C declarations for GJK, EPA, convex hull building, polygon clipping
//! and triangle indexing.
//!
//! Mirrors `ffi/zjolt_geometry.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const core = @import("core.zig");

pub const AABox = core.AABox;
pub const Mat44 = core.Mat44;
pub const Result = core.Result;
pub const Vec3 = core.Vec3;

//===----------------------------------------------------------------------===//
// The support-function seam
//===----------------------------------------------------------------------===//

pub const ConvexSupport = extern struct {
    support: ?*const fn (user: ?*anyopaque, direction: Vec3) callconv(.c) Vec3 = null,
    user: ?*anyopaque = null,
};

pub const ConvexSupportAdapter = opaque {};

pub extern fn zjoltConvexSupportCreateTransformed(transform: *const Mat44, inner: *const ConvexSupport, out: **ConvexSupportAdapter) Result;

pub extern fn zjoltConvexSupportCreateAddConvexRadius(inner: *const ConvexSupport, radius: f32, out: **ConvexSupportAdapter) Result;

pub extern fn zjoltConvexSupportCreateMinkowskiDifference(a: *const ConvexSupport, b: *const ConvexSupport, out: **ConvexSupportAdapter) Result;

pub extern fn zjoltConvexSupportCreatePolygon(points: [*]const Vec3, num_points: u32, out: **ConvexSupportAdapter) Result;

pub extern fn zjoltConvexSupportCreateTriangle(v1: *const Vec3, v2: *const Vec3, v3: *const Vec3, out: **ConvexSupportAdapter) Result;

pub extern fn zjoltConvexSupportAdapterAsSupport(adapter: *const ConvexSupportAdapter, out: *ConvexSupport) void;

pub extern fn zjoltConvexSupportAdapterDestroy(adapter: ?*ConvexSupportAdapter) void;

//===----------------------------------------------------------------------===//
// GJK
//===----------------------------------------------------------------------===//

pub const GJK = opaque {};

pub extern fn zjoltGJKCreate(out: **GJK) Result;

pub extern fn zjoltGJKDestroy(gjk: ?*GJK) void;

pub extern fn zjoltGJKIntersects(gjk: *GJK, a: *const ConvexSupport, b: *const ConvexSupport, tolerance: f32, io_v: *Vec3, out_intersects: *bool) Result;

pub extern fn zjoltGJKGetClosestPoints(gjk: *GJK, a: *const ConvexSupport, b: *const ConvexSupport, tolerance: f32, max_dist_sq: f32, io_v: *Vec3, out_point_a: *Vec3, out_point_b: *Vec3, out_dist_sq: *f32) Result;

/// Vertices `zjoltGJKGetClosestPointsSimplex`'s three output arrays must hold.
pub const gjk_max_simplex_points: usize = 4;

pub extern fn zjoltGJKGetClosestPointsSimplex(gjk: *const GJK, out_y: [*]Vec3, out_p: [*]Vec3, out_q: [*]Vec3, out_num_points: *u32) void;

pub extern fn zjoltGJKCalculatePointAAndB(y: [*]const Vec3, p: [*]const Vec3, q: [*]const Vec3, num_points: u32, out_point_a: *Vec3, out_point_b: *Vec3) Result;

pub extern fn zjoltGJKCastRay(gjk: *GJK, ray_origin: *const Vec3, ray_direction: *const Vec3, tolerance: f32, a: *const ConvexSupport, io_lambda: *f32, out_hit: *bool) Result;

pub extern fn zjoltGJKIntersectsSweep(gjk: *GJK, start: *const Mat44, direction: *const Vec3, tolerance: f32, a: *const ConvexSupport, b: *const ConvexSupport, io_lambda: *f32, out_hit: *bool) Result;

pub extern fn zjoltGJKCastShape(gjk: *GJK, start: *const Mat44, direction: *const Vec3, tolerance: f32, a: *const ConvexSupport, b: *const ConvexSupport, convex_radius_a: f32, convex_radius_b: f32, io_lambda: *f32, out_point_a: *Vec3, out_point_b: *Vec3, out_separating_axis: *Vec3, out_hit: *bool) Result;

//===----------------------------------------------------------------------===//
// EPA
//===----------------------------------------------------------------------===//

pub const EPA = opaque {};

pub extern fn zjoltEPACreate(out: **EPA) Result;

pub extern fn zjoltEPADestroy(epa: ?*EPA) void;

pub const EpaStatus = enum(c_int) {
    not_colliding = 0,
    colliding = 1,
    indeterminate = 2,
};

pub extern fn zjoltEPAGetPenetrationDepthStepGJK(epa: *EPA, a_excluding_radius: *const ConvexSupport, convex_radius_a: f32, b_excluding_radius: *const ConvexSupport, convex_radius_b: f32, tolerance: f32, io_v: *Vec3, out_point_a: *Vec3, out_point_b: *Vec3, out_status: *EpaStatus) Result;

pub extern fn zjoltEPAGetPenetrationDepthStepEPA(epa: *EPA, a_including_radius: *const ConvexSupport, b_including_radius: *const ConvexSupport, tolerance: f32, out_v: *Vec3, out_point_a: *Vec3, out_point_b: *Vec3, out_collided: *bool) Result;

pub extern fn zjoltEPAGetPenetrationDepth(epa: *EPA, a_excluding_radius: *const ConvexSupport, a_including_radius: *const ConvexSupport, convex_radius_a: f32, b_excluding_radius: *const ConvexSupport, b_including_radius: *const ConvexSupport, convex_radius_b: f32, collision_tolerance_sq: f32, penetration_tolerance: f32, io_v: *Vec3, out_point_a: *Vec3, out_point_b: *Vec3, out_collided: *bool) Result;

pub extern fn zjoltEPACastShape(epa: *EPA, start: *const Mat44, direction: *const Vec3, collision_tolerance: f32, penetration_tolerance: f32, a: *const ConvexSupport, b: *const ConvexSupport, convex_radius_a: f32, convex_radius_b: f32, return_deepest_point: bool, io_lambda: *f32, out_point_a: *Vec3, out_point_b: *Vec3, out_contact_normal: *Vec3, out_hit: *bool) Result;

//===----------------------------------------------------------------------===//
// EPA's own hull builder
//===----------------------------------------------------------------------===//

pub const EPAConvexHullBuilder = opaque {};
pub const EPATriangle = opaque {};

/// `Points`' fixed capacity -- `EPAConvexHullBuilder::cMaxPoints`.
pub const epa_convex_hull_builder_max_points: u32 = 128;

pub extern fn zjoltEPAConvexHullBuilderCreate(positions: [*]const Vec3, num_positions: u32, out: **EPAConvexHullBuilder) Result;

pub extern fn zjoltEPAConvexHullBuilderDestroy(builder: ?*EPAConvexHullBuilder) void;

pub extern fn zjoltEPAConvexHullBuilderInitialize(builder: *EPAConvexHullBuilder, idx1: u32, idx2: u32, idx3: u32) Result;

pub extern fn zjoltEPAConvexHullBuilderHasNextTriangle(builder: *const EPAConvexHullBuilder) bool;

pub extern fn zjoltEPAConvexHullBuilderPeekClosestTriangle(builder: *EPAConvexHullBuilder) ?*EPATriangle;

pub extern fn zjoltEPAConvexHullBuilderPopClosestTriangle(builder: *EPAConvexHullBuilder) ?*EPATriangle;

pub extern fn zjoltEPAConvexHullBuilderFindFacingTriangle(builder: *EPAConvexHullBuilder, position: *const Vec3, out_best_dist_sq: *f32) ?*EPATriangle;

pub extern fn zjoltEPAConvexHullBuilderFreeTriangle(builder: *EPAConvexHullBuilder, triangle: *EPATriangle) void;

pub extern fn zjoltEPATriangleIsFacing(triangle: *const EPATriangle, position: *const Vec3) bool;

pub extern fn zjoltEPATriangleIsFacingOrigin(triangle: *const EPATriangle) bool;

pub const EPAEdge = extern struct {
    neighbour_triangle: ?*EPATriangle,
    neighbour_edge: i32,
    start_idx: u32,
};

pub extern fn zjoltEPATriangleGetNextEdge(triangle: *const EPATriangle, index: u32, out_edge: *EPAEdge) Result;

//===----------------------------------------------------------------------===//
// Convex hull building
//===----------------------------------------------------------------------===//

pub const ConvexHullBuilder = opaque {};

pub extern fn zjoltConvexHullBuilderCreate(positions: [*]const Vec3, num_positions: u32, out: **ConvexHullBuilder) Result;

pub extern fn zjoltConvexHullBuilderDestroy(builder: ?*ConvexHullBuilder) void;

pub const ConvexHullResult = enum(c_int) {
    success = 0,
    max_vertices_reached = 1,
    too_few_points = 2,
    too_few_faces = 3,
    degenerate = 4,
};

pub extern fn zjoltConvexHullBuilderInitialize(builder: *ConvexHullBuilder, max_vertices: u32, tolerance: f32, out_hull_result: *ConvexHullResult) Result;

pub extern fn zjoltConvexHullBuilderGetNumVerticesUsed(builder: *const ConvexHullBuilder) u32;

pub extern fn zjoltConvexHullBuilderContainsFace(builder: *const ConvexHullBuilder, indices: [*]const u32, num_indices: u32) bool;

pub extern fn zjoltConvexHullBuilderGetCenterOfMassAndVolume(builder: *const ConvexHullBuilder, out_center_of_mass: *Vec3, out_volume: *f32) Result;

pub extern fn zjoltConvexHullBuilderDetermineMaxError(builder: *const ConvexHullBuilder, out_face_index: *i32, out_max_error: *f32, out_max_error_position_index: *i32, out_coplanar_distance: *f32) Result;

pub extern fn zjoltConvexHullBuilderGetNumFaces(builder: *const ConvexHullBuilder, out_num_faces: *u32) Result;

pub const ConvexHullFace = extern struct {
    normal: Vec3,
    centroid: Vec3,
    num_vertices: u32,
};

pub extern fn zjoltConvexHullBuilderGetFace(builder: *const ConvexHullBuilder, face_index: u32, out_face: *ConvexHullFace) Result;

pub extern fn zjoltConvexHullBuilderGetFaceVertices(builder: *const ConvexHullBuilder, face_index: u32, out_indices: ?[*]u32, capacity: u32, out_count: *u32) Result;

//===----------------------------------------------------------------------===//
// 2D convex hull building
//===----------------------------------------------------------------------===//

pub const ConvexHullBuilder2D = opaque {};

pub extern fn zjoltConvexHullBuilder2DCreate(positions: [*]const Vec3, num_positions: u32, out: **ConvexHullBuilder2D) Result;

pub extern fn zjoltConvexHullBuilder2DDestroy(builder: ?*ConvexHullBuilder2D) void;

/// Always SUCCESS or MAX_VERTICES_REACHED for this builder -- it shares
/// ConvexHullResult rather than needing its own enum.
pub extern fn zjoltConvexHullBuilder2DInitialize(builder: *ConvexHullBuilder2D, idx1: u32, idx2: u32, idx3: u32, max_vertices: u32, tolerance: f32, out_edges: ?[*]u32, capacity: u32, out_count: *u32, out_hull_result: *ConvexHullResult) Result;

//===----------------------------------------------------------------------===//
// Polygon clipping
//===----------------------------------------------------------------------===//

pub extern fn zjoltClipPolyVsPlane(polygon: [*]const Vec3, num_vertices: u32, plane_origin: *const Vec3, plane_normal: *const Vec3, out_polygon: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltClipPolyVsPoly(polygon: [*]const Vec3, num_vertices: u32, clipping_polygon: [*]const Vec3, num_clipping_vertices: u32, clipping_polygon_normal: *const Vec3, out_polygon: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltClipPolyVsEdge(polygon: [*]const Vec3, num_vertices: u32, edge_vertex1: *const Vec3, edge_vertex2: *const Vec3, clipping_edge_normal: *const Vec3, out_polygon: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltClipPolyVsAABox(polygon: [*]const Vec3, num_vertices: u32, box: *const AABox, out_polygon: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

//===----------------------------------------------------------------------===//
// Triangle indexing
//===----------------------------------------------------------------------===//

pub const IndexifyTriangle = extern struct {
    v: [3]Vec3,
    material_index: u32,
    user_data: u32,
};

pub const IndexedTriangle = extern struct {
    indices: [3]u32,
    material_index: u32,
    user_data: u32,
};

pub extern fn zjoltIndexify(triangles: [*]const IndexifyTriangle, num_triangles: u32, vertex_weld_distance: f32, out_vertices: ?[*]Vec3, vertex_capacity: u32, out_num_vertices: *u32, out_triangles: ?[*]IndexedTriangle, triangle_capacity: u32, out_num_triangles: *u32) Result;

pub extern fn zjoltDeindexify(vertices: [*]const Vec3, num_vertices: u32, triangles: [*]const IndexedTriangle, num_triangles: u32, out_triangles: ?[*]IndexifyTriangle, capacity: u32, out_count: *u32) Result;
