//===----------------------------------------------------------------------===//
// zjolt — broad-phase queries: which bodies are roughly there.
//
// Return BODY IDS ONLY — no contact points, normals or penetration depth;
// for those use zjoltCastRay*/zjoltCastShape*/zjoltCollideShape in
// zjolt_query.h. A hit is an AXIS-ALIGNED BOUNDING BOX overlap, not the
// shape itself. Single precision even with double-precision positions.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_BROADPHASE_H_
#define ZJOLT_BROADPHASE_H_

#include "zjolt_core.h"
// Pulls in the narrow-phase queries' broad/object-layer filter types.
#include "zjolt_query.h"

#ifdef __cplusplus
extern "C" {
#endif

/// The two filters a query can consult — NULL, or any NULL member,
/// accepts everything. Not ZJoltQueryFilters: no body filter to give one to.
typedef struct ZJoltBroadPhaseFilters {
  ZJoltBroadPhaseLayerFilter broad_phase_layer;
  ZJoltObjectLayerFilter object_layer;
} ZJoltBroadPhaseFilters;

/// A body whose bounding box the ray or swept box entered, and how far along.
/// No sub-shape id or normal — neither exists at this stage.
typedef struct ZJoltBroadPhaseCastHit {
  ZJoltBodyId body;
  /// In [0, 1] along the cast: entry point is origin + fraction * direction,
  /// of the BOUNDING BOX, not the shape.
  float fraction;
} ZJoltBroadPhaseCastHit;

/// A non-axis-aligned box (centre, rotation, half extents), for
/// zjoltBroadPhaseCollideOrientedBox — a host already has a rotation.
typedef struct ZJoltOrientedBox {
  ZJoltRVec3 center;
  ZJoltQuat rotation;
  ZJoltVec3 half_extent;
} ZJoltOrientedBox;

//===----------------------------------------------------------------------===//
// The queries: two-call protocol (`*out_count` true count; short buffer ->
// ZJOLT_RESULT_BUFFER_TOO_SMALL). Results are UNSORTED, including both casts.
//===----------------------------------------------------------------------===//

/// Bodies whose bounding box the ray enters. `direction` carries the ray's
/// length; nothing beyond it is reported.
ZJOLT_API ZJoltResult zjoltBroadPhaseCastRay(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltBroadPhaseFilters *filters,
    ZJoltBroadPhaseCastHit *out_hits, uint32_t capacity, uint32_t *out_count);

/// Bodies whose bounding box overlaps `box`.
ZJOLT_API ZJoltResult zjoltBroadPhaseCollideAABox(
    const ZJoltPhysicsSystem *system, const ZJoltAABox *box,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count);

/// Bodies whose bounding box overlaps the sphere — NOT a sphere overlap
/// test: a body whose box merely clips the sphere is reported too.
ZJOLT_API ZJoltResult zjoltBroadPhaseCollideSphere(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *center, float radius,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count);

/// Bodies whose bounding box contains the point — the cheapest broad-phase
/// query, a reasonable first pass for a picking ray's origin.
ZJOLT_API ZJoltResult zjoltBroadPhaseCollidePoint(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *point,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count);

/// Bodies whose box overlaps an oriented box — more precise (and pricier:
/// a separating-axis test per candidate) than AABox's own AA bounds.
ZJOLT_API ZJoltResult zjoltBroadPhaseCollideOrientedBox(
    const ZJoltPhysicsSystem *system, const ZJoltOrientedBox *box,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count);

/// Bodies whose box `box` sweeps through along `direction` — stays axis
/// aligned for the sweep; for a rotating/non-box sweep use zjoltCastShapeAll.
ZJOLT_API ZJoltResult zjoltBroadPhaseCastAABox(
    const ZJoltPhysicsSystem *system, const ZJoltAABox *box,
    const ZJoltVec3 *direction, const ZJoltBroadPhaseFilters *filters,
    ZJoltBroadPhaseCastHit *out_hits, uint32_t capacity, uint32_t *out_count);

/// Bounding box of every body. An EMPTY world reports Jolt's inside-out
/// box (min +FLT_MAX, max -FLT_MAX), not an empty box — check
/// zjoltPhysicsSystemGetNumBodies first. NULL `system` zeroes `out` instead.
ZJOLT_API void zjoltBroadPhaseGetBounds(const ZJoltPhysicsSystem *system,
                                        ZJoltAABox *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_BROADPHASE_H_
