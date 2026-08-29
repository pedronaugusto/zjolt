//===----------------------------------------------------------------------===//
// zjolt — GJK, EPA, convex hull building, polygon clipping, triangle indexing.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_GEOMETRY_H_
#define ZJOLT_GEOMETRY_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// The support-function seam
//
// GJK and EPA work against any convex object answering "the point of me
// furthest in this direction" — a callback standing in for a C++ host's GetSupport(Vec3) method.
//===----------------------------------------------------------------------===//

/// A convex object, given by the support point it returns for a direction.
/// Required wherever this header takes one: a NULL `support` is refused at
/// the entry point that receives it, never dereferenced.
typedef struct ZJoltConvexSupport {
  ZJoltVec3 (*support)(void *user, ZJoltVec3 direction);
  void *user;
} ZJoltConvexSupport;

/// One of the five adapters below, wrapping an inner support (or raw
/// vertices) into a fresh `ZJoltConvexSupport` -- `Jolt/Geometry/
/// ConvexSupport.h`'s `TransformedConvexObject`, `AddConvexRadius`,
/// `MinkowskiDifference`, `PolygonConvexSupport` and `TriangleConvexSupport`.
typedef struct ZJoltConvexSupportAdapter ZJoltConvexSupportAdapter;

/// `inner`'s `user` context must stay valid for as long as the resulting
/// adapter is used, not just for one call: an adapter is a handle invoked
/// many times after creation, unlike a plain `ZJoltConvexSupport` handed
/// straight to GJK/EPA.
///
/// `inner`, placed by `transform` (uniform scale only).
ZJOLT_API ZJoltResult zjoltConvexSupportCreateTransformed(
    const ZJoltMat44 *transform, const ZJoltConvexSupport *inner,
    ZJoltConvexSupportAdapter **out);

/// `inner`, rounded by `radius` in every direction.
ZJOLT_API ZJoltResult zjoltConvexSupportCreateAddConvexRadius(
    const ZJoltConvexSupport *inner, float radius,
    ZJoltConvexSupportAdapter **out);

/// The Minkowski difference `a` - `b`, the shape GJK itself searches.
ZJOLT_API ZJoltResult zjoltConvexSupportCreateMinkowskiDifference(
    const ZJoltConvexSupport *a, const ZJoltConvexSupport *b,
    ZJoltConvexSupportAdapter **out);

/// `points` copied in; at least one point is required.
ZJOLT_API ZJoltResult zjoltConvexSupportCreatePolygon(
    const ZJoltVec3 *points, uint32_t num_points,
    ZJoltConvexSupportAdapter **out);

ZJOLT_API ZJoltResult zjoltConvexSupportCreateTriangle(
    const ZJoltVec3 *v1, const ZJoltVec3 *v2, const ZJoltVec3 *v3,
    ZJoltConvexSupportAdapter **out);

/// `adapter` as something GJK/EPA can take: `out->user` is `adapter` itself,
/// so `out` is valid exactly as long as `adapter` is.
ZJOLT_API void zjoltConvexSupportAdapterAsSupport(
    const ZJoltConvexSupportAdapter *adapter, ZJoltConvexSupport *out);

ZJOLT_API void zjoltConvexSupportAdapterDestroy(ZJoltConvexSupportAdapter *adapter);

//===----------------------------------------------------------------------===//
// GJK -- Jolt/Geometry/GJKClosestPoint.h
//
// A `ZJoltGJK` is JPH::GJKClosestPoint itself: cheap to create, and required across zjoltGJKGetClosestPoints
// then zjoltGJKGetClosestPointsSimplex, so it outlives any single call.
//===----------------------------------------------------------------------===//

typedef struct ZJoltGJK ZJoltGJK;

ZJOLT_API ZJoltResult zjoltGJKCreate(ZJoltGJK **out);
ZJOLT_API void zjoltGJKDestroy(ZJoltGJK *gjk);

/// True if `a` and `b` overlap. `io_v` is the initial separating-axis guess
/// on entry (zero if unknown) and, on a miss, a separating axis on exit.
ZJOLT_API ZJoltResult zjoltGJKIntersects(
    ZJoltGJK *gjk, const ZJoltConvexSupport *a, const ZJoltConvexSupport *b,
    float tolerance, ZJoltVec3 *io_v, bool *out_intersects);

/// The squared distance between `a` and `b`, or `FLT_MAX` if it exceeds
/// `max_dist_sq`. `out_point_a`/`out_point_b` are meaningful only when the
/// returned `*out_dist_sq` is strictly between 0 and `FLT_MAX`; both are
/// zeroed otherwise, matching Jolt's own "the points are invalid" rule.
ZJOLT_API ZJoltResult zjoltGJKGetClosestPoints(
    ZJoltGJK *gjk, const ZJoltConvexSupport *a, const ZJoltConvexSupport *b,
    float tolerance, float max_dist_sq, ZJoltVec3 *io_v,
    ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b, float *out_dist_sq);

/// Vertices `zjoltGJKGetClosestPointsSimplex` can report, and the fixed
/// capacity its three output arrays must have.
#define ZJOLT_GJK_MAX_SIMPLEX_POINTS 4

/// The simplex `gjk` holds after its last call, as three parallel arrays of
/// `ZJOLT_GJK_MAX_SIMPLEX_POINTS` capacity: `out_y` on the Minkowski
/// difference, `out_p`/`out_q` the support points on A and B it came from.
/// Zeroes `*out_num_points` for a NULL `gjk`.
ZJOLT_API void zjoltGJKGetClosestPointsSimplex(
    const ZJoltGJK *gjk, ZJoltVec3 *out_y, ZJoltVec3 *out_p, ZJoltVec3 *out_q,
    uint32_t *out_num_points);

/// Recomputes the closest points on A and B from a simplex — typically
/// one read back with zjoltGJKGetClosestPointsSimplex — without a
/// `ZJoltGJK` of its own. `num_points` must be in [1, 4]; at 4 the
/// origin is inside the simplex with no well-defined closest pair, so
/// both outputs are left zeroed.
ZJOLT_API ZJoltResult zjoltGJKCalculatePointAAndB(
    const ZJoltVec3 *y, const ZJoltVec3 *p, const ZJoltVec3 *q,
    uint32_t num_points, ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b);

/// Casts a ray against `a`. `io_lambda` is the max fraction along the ray on
/// entry and the hit fraction on a hit.
ZJOLT_API ZJoltResult zjoltGJKCastRay(
    ZJoltGJK *gjk, const ZJoltVec3 *ray_origin, const ZJoltVec3 *ray_direction,
    float tolerance, const ZJoltConvexSupport *a, float *io_lambda,
    bool *out_hit);

/// Whether `a`, swept from `start` along `direction`, hits `b` -- GJK's own
/// `CastShape` overload with no convex radius and no contact points. `io_lambda`
/// is the max fraction of the sweep on entry and the hit fraction on a hit.
ZJOLT_API ZJoltResult zjoltGJKIntersectsSweep(
    ZJoltGJK *gjk, const ZJoltMat44 *start, const ZJoltVec3 *direction,
    float tolerance, const ZJoltConvexSupport *a, const ZJoltConvexSupport *b,
    float *io_lambda, bool *out_hit);

/// As zjoltGJKIntersectsSweep, but with convex radii and a contact point pair
/// plus separating axis on a hit -- GJK's other `CastShape` overload.
ZJOLT_API ZJoltResult zjoltGJKCastShape(
    ZJoltGJK *gjk, const ZJoltMat44 *start, const ZJoltVec3 *direction,
    float tolerance, const ZJoltConvexSupport *a, const ZJoltConvexSupport *b,
    float convex_radius_a, float convex_radius_b, float *io_lambda,
    ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
    ZJoltVec3 *out_separating_axis, bool *out_hit);

//===----------------------------------------------------------------------===//
// EPA -- Jolt/Geometry/EPAPenetrationDepth.h
//
// `GetPenetrationDepth` is the headline: how deep two convex shapes overlap, and along which axis. The two-step
// form shares a `ZJoltEPA`'s internal GJK simplex between calls, like `ZJoltGJK` does.
//===----------------------------------------------------------------------===//

typedef struct ZJoltEPA ZJoltEPA;

ZJOLT_API ZJoltResult zjoltEPACreate(ZJoltEPA **out);
ZJOLT_API void zjoltEPADestroy(ZJoltEPA *epa);

typedef enum ZJoltEpaStatus {
  ZJOLT_EPA_STATUS_NOT_COLLIDING = 0,
  ZJOLT_EPA_STATUS_COLLIDING = 1,
  /// Penetrating deeper than the convex radius: call
  /// zjoltEPAGetPenetrationDepthStepEPA next.
  ZJOLT_EPA_STATUS_INDETERMINATE = 2,
} ZJoltEpaStatus;

/// First step: GJK against the objects without their convex radius. `io_v`
/// must not be near zero on entry (pass a previous result, or (1, 0, 0)).
ZJOLT_API ZJoltResult zjoltEPAGetPenetrationDepthStepGJK(
    ZJoltEPA *epa, const ZJoltConvexSupport *a_excluding_radius,
    float convex_radius_a, const ZJoltConvexSupport *b_excluding_radius,
    float convex_radius_b, float tolerance, ZJoltVec3 *io_v,
    ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
    ZJoltEpaStatus *out_status);

/// Second step, only needed after ZJOLT_EPA_STATUS_INDETERMINATE: the objects
/// WITH their convex radius. `tolerance` must be at least FLT_EPSILON.
ZJOLT_API ZJoltResult zjoltEPAGetPenetrationDepthStepEPA(
    ZJoltEPA *epa, const ZJoltConvexSupport *a_including_radius,
    const ZJoltConvexSupport *b_including_radius, float tolerance,
    ZJoltVec3 *out_v, ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
    bool *out_collided);

/// Both steps in one call. `io_v` must not be near zero on entry, the same
/// requirement as the GJK step alone. `penetration_tolerance` must be at
/// least FLT_EPSILON.
ZJOLT_API ZJoltResult zjoltEPAGetPenetrationDepth(
    ZJoltEPA *epa, const ZJoltConvexSupport *a_excluding_radius,
    const ZJoltConvexSupport *a_including_radius, float convex_radius_a,
    const ZJoltConvexSupport *b_excluding_radius,
    const ZJoltConvexSupport *b_including_radius, float convex_radius_b,
    float collision_tolerance_sq, float penetration_tolerance,
    ZJoltVec3 *io_v, ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
    bool *out_collided);

/// Casts `a` from `start` along `direction` against `b`, resolving a
/// starting overlap into a deepest contact when `return_deepest_point` is
/// true. `io_lambda` is the max fraction of the sweep on entry and the hit
/// fraction on a hit. `penetration_tolerance` must be at least FLT_EPSILON.
ZJOLT_API ZJoltResult zjoltEPACastShape(
    ZJoltEPA *epa, const ZJoltMat44 *start, const ZJoltVec3 *direction,
    float collision_tolerance, float penetration_tolerance,
    const ZJoltConvexSupport *a, const ZJoltConvexSupport *b,
    float convex_radius_a, float convex_radius_b, bool return_deepest_point,
    float *io_lambda, ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
    ZJoltVec3 *out_contact_normal, bool *out_hit);

//===----------------------------------------------------------------------===//
// EPA's own hull builder -- Jolt/Geometry/EPAConvexHullBuilder.h
//
// The incremental hull EPA grows around the Minkowski difference while it
// searches for the origin. `positions` is fixed at creation; adding a point
// to the hull itself is not exposed here.
//===----------------------------------------------------------------------===//

typedef struct ZJoltEPAConvexHullBuilder ZJoltEPAConvexHullBuilder;

/// One triangle of a `ZJoltEPAConvexHullBuilder`'s hull. Borrowed: valid
/// until freed with zjoltEPAConvexHullBuilderFreeTriangle or its builder is
/// destroyed, whichever comes first.
typedef struct ZJoltEPATriangle ZJoltEPATriangle;

/// `Points`' fixed capacity -- `EPAConvexHullBuilder::cMaxPoints`.
#define ZJOLT_EPA_CONVEX_HULL_BUILDER_MAX_POINTS 128

/// `positions` is copied in. `num_positions` must not exceed
/// ZJOLT_EPA_CONVEX_HULL_BUILDER_MAX_POINTS.
ZJOLT_API ZJoltResult zjoltEPAConvexHullBuilderCreate(
    const ZJoltVec3 *positions, uint32_t num_positions,
    ZJoltEPAConvexHullBuilder **out);
ZJOLT_API void zjoltEPAConvexHullBuilderDestroy(
    ZJoltEPAConvexHullBuilder *builder);

/// Starts the hull as two triangles, back to back, over three distinct
/// indices into `builder`'s positions. ZJOLT_RESULT_INVALID_ARGUMENT if any
/// index is out of range or repeated.
ZJOLT_API ZJoltResult zjoltEPAConvexHullBuilderInitialize(
    ZJoltEPAConvexHullBuilder *builder, uint32_t idx1, uint32_t idx2,
    uint32_t idx3);

ZJOLT_API bool zjoltEPAConvexHullBuilderHasNextTriangle(
    const ZJoltEPAConvexHullBuilder *builder);

/// The next closest triangle to the origin, without removing it from the
/// queue. NULL if the queue is empty.
ZJOLT_API ZJoltEPATriangle *zjoltEPAConvexHullBuilderPeekClosestTriangle(
    ZJoltEPAConvexHullBuilder *builder);

/// As above, but removes it from the queue.
ZJOLT_API ZJoltEPATriangle *zjoltEPAConvexHullBuilderPopClosestTriangle(
    ZJoltEPAConvexHullBuilder *builder);

/// The triangle `position` is furthest in front of, or NULL if `position` is
/// behind every triangle still in the queue. `*out_best_dist_sq` is the
/// squared distance to that triangle's plane, 0 when NULL is returned.
ZJOLT_API ZJoltEPATriangle *zjoltEPAConvexHullBuilderFindFacingTriangle(
    ZJoltEPAConvexHullBuilder *builder, const ZJoltVec3 *position,
    float *out_best_dist_sq);

/// Unlinks `triangle` from its neighbours and returns it to `builder`'s free
/// list; `triangle` must not be used afterwards.
ZJOLT_API void zjoltEPAConvexHullBuilderFreeTriangle(
    ZJoltEPAConvexHullBuilder *builder, ZJoltEPATriangle *triangle);

ZJOLT_API bool zjoltEPATriangleIsFacing(const ZJoltEPATriangle *triangle,
                                        const ZJoltVec3 *position);
ZJOLT_API bool zjoltEPATriangleIsFacingOrigin(
    const ZJoltEPATriangle *triangle);

/// One edge of a `ZJoltEPATriangle`, read out by value.
typedef struct ZJoltEPAEdge {
  /// NULL if this edge has no neighbour linked yet.
  ZJoltEPATriangle *neighbour_triangle;
  /// Edge index on `neighbour_triangle`; meaningless if it is NULL.
  int32_t neighbour_edge;
  /// Vertex index, into the owning builder's positions, at this edge's start.
  uint32_t start_idx;
} ZJoltEPAEdge;

/// `triangle`'s edge that follows edge `index` around it, (`index` + 1) % 3.
/// ZJOLT_RESULT_INVALID_ARGUMENT if `index` is not in [0, 3).
ZJOLT_API ZJoltResult zjoltEPATriangleGetNextEdge(const ZJoltEPATriangle *triangle,
                                                  uint32_t index,
                                                  ZJoltEPAEdge *out_edge);

//===----------------------------------------------------------------------===//
// Convex hull building -- Jolt/Geometry/ConvexHullBuilder.h
//
// Offline construction: given a point cloud, the minimal hull that contains it. Faces are a count plus an indexed
// getter, not Jolt's own `Face *`, which the builder owns and frees out from under a caller.
//===----------------------------------------------------------------------===//

typedef struct ZJoltConvexHullBuilder ZJoltConvexHullBuilder;

/// `positions` is copied in; the builder considers all of them, discarding
/// interior points as it goes.
ZJOLT_API ZJoltResult zjoltConvexHullBuilderCreate(
    const ZJoltVec3 *positions, uint32_t num_positions,
    ZJoltConvexHullBuilder **out);
ZJOLT_API void zjoltConvexHullBuilderDestroy(ZJoltConvexHullBuilder *builder);

typedef enum ZJoltConvexHullResult {
  ZJOLT_CONVEX_HULL_RESULT_SUCCESS = 0,
  /// Still a usable hull; `max_vertices` cut it off before full accuracy.
  ZJOLT_CONVEX_HULL_RESULT_MAX_VERTICES_REACHED = 1,
  ZJOLT_CONVEX_HULL_RESULT_TOO_FEW_POINTS = 2,
  ZJOLT_CONVEX_HULL_RESULT_TOO_FEW_FACES = 3,
  ZJOLT_CONVEX_HULL_RESULT_DEGENERATE = 4,
} ZJoltConvexHullResult;

/// Builds the hull. `max_vertices` of 0 means no limit. `*out_hull_result`
/// is always written; ZJOLT_RESULT_SHAPE_INVALID (with zjoltLastError set)
/// for TOO_FEW_POINTS, TOO_FEW_FACES or DEGENERATE, ZJOLT_RESULT_OK for the
/// other two, which both leave a usable hull.
ZJOLT_API ZJoltResult zjoltConvexHullBuilderInitialize(
    ZJoltConvexHullBuilder *builder, uint32_t max_vertices, float tolerance,
    ZJoltConvexHullResult *out_hull_result);

ZJOLT_API uint32_t zjoltConvexHullBuilderGetNumVerticesUsed(
    const ZJoltConvexHullBuilder *builder);

/// Whether the hull has a face with exactly these vertices (counter
/// clockwise, indices into the positions `builder` was created with). False
/// for a NULL `builder` or `indices`.
ZJOLT_API bool zjoltConvexHullBuilderContainsFace(
    const ZJoltConvexHullBuilder *builder, const uint32_t *indices,
    uint32_t num_indices);

ZJOLT_API ZJoltResult zjoltConvexHullBuilderGetCenterOfMassAndVolume(
    const ZJoltConvexHullBuilder *builder, ZJoltVec3 *out_center_of_mass,
    float *out_volume);

/// The point furthest outside the hull and how far, which is 0 for a
/// perfectly built hull and otherwise a measure of how much
/// `zjoltConvexHullBuilderInitialize`'s `tolerance` was exceeded.
/// `*out_face_index` and `*out_max_error_position_index` are -1 when there is
/// no such face/point -- the hull has no faces, or no point exceeds it.
ZJOLT_API ZJoltResult zjoltConvexHullBuilderDetermineMaxError(
    const ZJoltConvexHullBuilder *builder, int32_t *out_face_index,
    float *out_max_error, int32_t *out_max_error_position_index,
    float *out_coplanar_distance);

ZJOLT_API ZJoltResult zjoltConvexHullBuilderGetNumFaces(
    const ZJoltConvexHullBuilder *builder, uint32_t *out_num_faces);

typedef struct ZJoltConvexHullFace {
  ZJoltVec3 normal;
  ZJoltVec3 centroid;
  uint32_t num_vertices;
} ZJoltConvexHullFace;

ZJOLT_API ZJoltResult zjoltConvexHullBuilderGetFace(
    const ZJoltConvexHullBuilder *builder, uint32_t face_index,
    ZJoltConvexHullFace *out_face);

/// `face_index`'s vertices, counter clockwise, as indices into the positions
/// `builder` was created with. Two-call protocol: pass `out_indices` = NULL
/// to learn the count.
ZJOLT_API ZJoltResult zjoltConvexHullBuilderGetFaceVertices(
    const ZJoltConvexHullBuilder *builder, uint32_t face_index,
    uint32_t *out_indices, uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// 2D convex hull building -- Jolt/Geometry/ConvexHullBuilder2D.h
//
// Uses only the X and Y of each position, matching the upstream type.
//===----------------------------------------------------------------------===//

typedef struct ZJoltConvexHullBuilder2D ZJoltConvexHullBuilder2D;

ZJOLT_API ZJoltResult zjoltConvexHullBuilder2DCreate(
    const ZJoltVec3 *positions, uint32_t num_positions,
    ZJoltConvexHullBuilder2D **out);
ZJOLT_API void zjoltConvexHullBuilder2DDestroy(
    ZJoltConvexHullBuilder2D *builder);

/// Builds the hull edge loop starting from `idx1`/`idx2`/`idx3` (any
/// order, indices into the positions `builder` was created with).
/// `max_vertices` of 0 means no limit. `out_edges` is the loop, counter
/// clockwise; two-call protocol, NULL to learn the count.
/// `*out_hull_result` is always SUCCESS or MAX_VERTICES_REACHED.
ZJOLT_API ZJoltResult zjoltConvexHullBuilder2DInitialize(
    ZJoltConvexHullBuilder2D *builder, uint32_t idx1, uint32_t idx2,
    uint32_t idx3, uint32_t max_vertices, float tolerance,
    uint32_t *out_edges, uint32_t capacity, uint32_t *out_count,
    ZJoltConvexHullResult *out_hull_result);

//===----------------------------------------------------------------------===//
// Polygon clipping -- Jolt/Geometry/ClipPoly.h
//
// Free functions over vertex arrays. Unlike Jolt's own growing container, these take a fixed output buffer and
// report ZJOLT_RESULT_BUFFER_TOO_SMALL (true count still written) if short.
//===----------------------------------------------------------------------===//

/// Clips `polygon` against the positive half space of the plane through
/// `plane_origin` with `plane_normal` (need not be unit length). `polygon`
/// needs at least 2 vertices.
ZJOLT_API ZJoltResult zjoltClipPolyVsPlane(
    const ZJoltVec3 *polygon, uint32_t num_vertices,
    const ZJoltVec3 *plane_origin, const ZJoltVec3 *plane_normal,
    ZJoltVec3 *out_polygon, uint32_t capacity, uint32_t *out_count);

/// Clips `polygon` against `clipping_polygon`, both counter clockwise.
/// `polygon` needs at least 2 vertices, `clipping_polygon` at least 3.
ZJOLT_API ZJoltResult zjoltClipPolyVsPoly(
    const ZJoltVec3 *polygon, uint32_t num_vertices,
    const ZJoltVec3 *clipping_polygon, uint32_t num_clipping_vertices,
    const ZJoltVec3 *clipping_polygon_normal, ZJoltVec3 *out_polygon,
    uint32_t capacity, uint32_t *out_count);

/// Clips `polygon` (a planar polygon with at least 3 vertices) against an
/// edge projected onto its plane using `clipping_edge_normal`; the half
/// space in that normal's direction is cut away.
ZJOLT_API ZJoltResult zjoltClipPolyVsEdge(
    const ZJoltVec3 *polygon, uint32_t num_vertices,
    const ZJoltVec3 *edge_vertex1, const ZJoltVec3 *edge_vertex2,
    const ZJoltVec3 *clipping_edge_normal, ZJoltVec3 *out_polygon,
    uint32_t capacity, uint32_t *out_count);

/// Clips `polygon` (counter clockwise, at least 2 vertices) against `box`,
/// keeping what is inside.
ZJOLT_API ZJoltResult zjoltClipPolyVsAABox(
    const ZJoltVec3 *polygon, uint32_t num_vertices, const ZJoltAABox *box,
    ZJoltVec3 *out_polygon, uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Triangle indexing -- Jolt/Geometry/Indexify.h
//===----------------------------------------------------------------------===//

/// One triangle in a soup: three vertices, a material index and a
/// host-chosen user value. Layout mirrors `Jolt::Triangle`.
typedef struct ZJoltIndexifyTriangle {
  ZJoltVec3 v[3];
  uint32_t material_index;
  uint32_t user_data;
} ZJoltIndexifyTriangle;

/// One triangle addressed by vertex index. Layout mirrors
/// `Jolt::IndexedTriangle`.
typedef struct ZJoltIndexedTriangle {
  uint32_t indices[3];
  uint32_t material_index;
  uint32_t user_data;
} ZJoltIndexedTriangle;

/// Jolt's own default weld distance, for a caller that wants it without
/// spelling out the constant.
#define ZJOLT_INDEXIFY_DEFAULT_VERTEX_WELD_DISTANCE 1.0e-4f

/// Welds vertices within `vertex_weld_distance` of each other and builds an
/// indexed mesh from `triangles`; a triangle that degenerates once its
/// vertices are welded is dropped, so `*out_num_triangles` may be less than
/// `num_triangles`. Two-call protocol, independently for each output: pass
/// `out_vertices`/`out_triangles` = NULL to learn their counts.
ZJOLT_API ZJoltResult zjoltIndexify(
    const ZJoltIndexifyTriangle *triangles, uint32_t num_triangles,
    float vertex_weld_distance, ZJoltVec3 *out_vertices,
    uint32_t vertex_capacity, uint32_t *out_num_vertices,
    ZJoltIndexedTriangle *out_triangles, uint32_t triangle_capacity,
    uint32_t *out_num_triangles);

/// The inverse: unpacks `triangles` (indices into `vertices`) into a soup.
/// One output triangle per input triangle.
ZJOLT_API ZJoltResult zjoltDeindexify(
    const ZJoltVec3 *vertices, uint32_t num_vertices,
    const ZJoltIndexedTriangle *triangles, uint32_t num_triangles,
    ZJoltIndexifyTriangle *out_triangles, uint32_t capacity,
    uint32_t *out_count);

#ifdef __cplusplus
}
#endif

#endif  // ZJOLT_GEOMETRY_H_
