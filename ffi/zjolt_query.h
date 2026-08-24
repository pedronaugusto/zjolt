//===----------------------------------------------------------------------===//
// zjolt — ray casts, shape casts and overlap tests.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_QUERY_H_
#define ZJOLT_QUERY_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Queries
//
// Filters are optional at every level: a NULL ZJoltQueryFilters accepts
// everything, and so does a member whose function pointer is NULL.
//===----------------------------------------------------------------------===//

typedef struct ZJoltBroadPhaseLayerFilter {
  bool (*should_collide)(void *user, ZJoltBroadPhaseLayer layer);
  void *user;
} ZJoltBroadPhaseLayerFilter;

typedef struct ZJoltObjectLayerFilter {
  bool (*should_collide)(void *user, ZJoltObjectLayer layer);
  void *user;
} ZJoltObjectLayerFilter;

typedef struct ZJoltBodyFilter {
  bool (*should_collide)(void *user, ZJoltBodyId body);
  void *user;
} ZJoltBodyFilter;

typedef struct ZJoltQueryFilters {
  ZJoltBroadPhaseLayerFilter broad_phase_layer;
  ZJoltObjectLayerFilter object_layer;
  ZJoltBodyFilter body;
} ZJoltQueryFilters;

typedef struct ZJoltRayCastHit {
  ZJoltBodyId body;
  ZJoltSubShapeId sub_shape_id;
  /// Hit point is origin + fraction * direction, with fraction in [0, 1].
  float fraction;
} ZJoltRayCastHit;

/// Contact points in a shape query are RELATIVE TO the `position` the query
/// was given; add it back for world space.
///
/// They are relative rather than absolute because they are floats. In a
/// double-precision world an absolute contact point does not survive the
/// conversion, and quietly losing that precision on every hit would defeat the
/// reason to build with double precision at all.
typedef struct ZJoltShapeCastHit {
  ZJoltBodyId body;
  ZJoltSubShapeId sub_shape_id;
  /// Centre of mass at the hit is start + fraction * direction.
  float fraction;
  /// Relative to the query's `position`. @see ZJoltShapeCastHit
  ZJoltVec3 contact_point_on_1;
  ZJoltVec3 contact_point_on_2;
  /// Direction to separate the shapes; its magnitude is meaningless.
  ZJoltVec3 penetration_axis;
  float penetration_depth;
  bool is_back_face_hit;
} ZJoltShapeCastHit;

typedef struct ZJoltCollideShapeHit {
  ZJoltBodyId body;
  ZJoltSubShapeId sub_shape_id;
  /// Relative to the query's `position`. @see ZJoltShapeCastHit
  ZJoltVec3 contact_point_on_1;
  ZJoltVec3 contact_point_on_2;
  ZJoltVec3 penetration_axis;
  float penetration_depth;
} ZJoltCollideShapeHit;

/// `direction` carries the ray's length: nothing beyond it is reported.
/// `*out_hit_any` says whether `out_hit` was written.
ZJOLT_API ZJoltResult zjoltCastRayClosest(const ZJoltPhysicsSystem *system,
                                          const ZJoltRVec3 *origin,
                                          const ZJoltVec3 *direction,
                                          const ZJoltQueryFilters *filters,
                                          ZJoltRayCastHit *out_hit,
                                          bool *out_hit_any);

/// Collects every hit along the ray, unsorted.
///
/// Two-call protocol, as everywhere here: `*out_count` always receives the
/// true number of hits, so a capacity of 0 (with hits = NULL) is a size query
/// and a short buffer reports ZJOLT_ERR_BUFFER_TOO_SMALL with the count.
ZJOLT_API ZJoltResult zjoltCastRayAll(const ZJoltPhysicsSystem *system,
                                      const ZJoltRVec3 *origin,
                                      const ZJoltVec3 *direction,
                                      const ZJoltQueryFilters *filters,
                                      ZJoltRayCastHit *out_hits,
                                      uint32_t capacity, uint32_t *out_count);

/// Sweeps `shape` from `position`/`rotation` along `direction`. `scale` may be
/// NULL for (1,1,1).
ZJOLT_API ZJoltResult zjoltCastShapeClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltQueryFilters *filters, ZJoltShapeCastHit *out_hit,
    bool *out_hit_any);

ZJOLT_API ZJoltResult zjoltCastShapeAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltQueryFilters *filters, ZJoltShapeCastHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// Everything overlapping `shape` placed at `position`/`rotation`.
ZJOLT_API ZJoltResult zjoltCollideShape(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, float max_separation_distance,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_QUERY_H_
