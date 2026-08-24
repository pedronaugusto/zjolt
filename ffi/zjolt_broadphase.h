//===----------------------------------------------------------------------===//
// zjolt — broad-phase queries: which bodies are roughly there.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// THESE RETURN BODY IDS AND NOTHING ELSE. That is the point of them, and it is
// the thing to understand before reaching for one: the broad phase tests only
// the bounding box of each body, so it answers "which bodies could possibly be
// involved" and never "where did they touch". There are no contact points, no
// normals and no penetration depths to be had here, and a caller who wants
// those wants zjoltCastRay*, zjoltCastShape* or zjoltCollideShape in
// zjolt_query.h instead — those run the broad phase first and then the narrow
// phase, which is where a contact comes from.
//
// What this is for: culling. A blast radius that needs candidates before it
// does its own falloff, an editor selection box, a trigger volume that only
// cares which bodies are near, a level streamer asking what is in a region.
// Work that would otherwise pay for narrow-phase collision it is going to
// throw away.
//
// A hit here means the body's AXIS-ALIGNED BOUNDING BOX overlaps, which for a
// long thin body at forty-five degrees is a box several times its own volume.
// Filtering the result is the caller's job.
//
// PRECISION. Jolt's broad phase is single precision in every build: its trees
// store `AABox`, which is float, and Jolt itself narrows an `RVec3` to a
// `Vec3` on the way in (`NarrowPhaseQuery.cpp:216`). The positions below are
// ZJoltRVec3 for consistency with the rest of this ABI and are narrowed the
// same way. In a double-precision world far from the origin that costs real
// precision — but it costs exactly what Jolt's own narrow-phase queries cost,
// because they take the same path.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_BROADPHASE_H_
#define ZJOLT_BROADPHASE_H_

#include "zjolt_core.h"
// For ZJoltBroadPhaseLayerFilter and ZJoltObjectLayerFilter, which a
// narrow-phase query takes too and which are therefore declared with those.
#include "zjolt_query.h"

#ifdef __cplusplus
extern "C" {
#endif

/// The two filters a broad-phase query can consult.
///
/// Deliberately not ZJoltQueryFilters: that carries a body filter as well, and
/// the broad phase has no body filter to give it to. Accepting one here and
/// ignoring it would be a filter that silently does nothing.
///
/// A NULL pointer accepts everything, and so does any member whose function
/// pointer is NULL.
typedef struct ZJoltBroadPhaseFilters {
  ZJoltBroadPhaseLayerFilter broad_phase_layer;
  ZJoltObjectLayerFilter object_layer;
} ZJoltBroadPhaseFilters;

/// A body whose bounding box the ray or the swept box entered, and how far
/// along. There is no sub-shape id and no normal: neither exists yet at this
/// stage.
typedef struct ZJoltBroadPhaseCastHit {
  ZJoltBodyId body;
  /// In [0, 1] along the cast. The entry point is origin + fraction *
  /// direction — of the BOUNDING BOX, not of the shape.
  float fraction;
} ZJoltBroadPhaseCastHit;

/// A box that is not axis aligned, for zjoltBroadPhaseCollideOrientedBox.
///
/// Given as centre, rotation and half extents rather than as Jolt's matrix and
/// half extents, because a matrix does not cross this boundary and a host
/// already has a rotation for everything it draws.
typedef struct ZJoltOrientedBox {
  ZJoltRVec3 center;
  ZJoltQuat rotation;
  ZJoltVec3 half_extent;
} ZJoltOrientedBox;

//===----------------------------------------------------------------------===//
// The queries
//
// Every one uses the two-call protocol the rest of this ABI uses: `*out_count`
// always receives the true number of results, so a capacity of 0 with a NULL
// buffer is a size query, and a short buffer reports
// ZJOLT_RESULT_BUFFER_TOO_SMALL with the count still written.
//
// Results are UNSORTED in every case, including the two casts. Jolt's
// broad-phase collectors append in tree-traversal order and nothing here
// reorders them, because sorting a candidate list the caller is about to
// filter anyway would be work done twice.
//===----------------------------------------------------------------------===//

/// Bodies whose bounding box the ray enters. `direction` carries the ray's
/// length; nothing beyond it is reported.
ZJOLT_API ZJoltResult zjoltBroadPhaseCastRay(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltBroadPhaseFilters *filters,
    ZJoltBroadPhaseCastHit *out_hits, uint32_t capacity, uint32_t *out_count);

/// Bodies whose bounding box overlaps `box`. World space, and float in every
/// build — see the precision note at the top of this header.
ZJOLT_API ZJoltResult zjoltBroadPhaseCollideAABox(
    const ZJoltPhysicsSystem *system, const ZJoltAABox *box,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count);

/// Bodies whose bounding box overlaps the sphere.
///
/// Note what this is NOT: a sphere overlap test. It is the bounding boxes that
/// are tested against the sphere, so a body whose box the sphere clips is
/// reported even when the body itself is nowhere near it.
ZJOLT_API ZJoltResult zjoltBroadPhaseCollideSphere(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *center, float radius,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count);

/// Bodies whose bounding box contains the point. The cheapest question the
/// broad phase answers, and a reasonable first pass for a picking ray's
/// origin.
ZJOLT_API ZJoltResult zjoltBroadPhaseCollidePoint(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *point,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count);

/// Bodies whose bounding box overlaps an oriented box.
///
/// More precise than passing the oriented box's own axis-aligned bounds to
/// zjoltBroadPhaseCollideAABox, and more expensive: each candidate gets a
/// separating-axis test instead of an interval overlap.
ZJOLT_API ZJoltResult zjoltBroadPhaseCollideOrientedBox(
    const ZJoltPhysicsSystem *system, const ZJoltOrientedBox *box,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count);

/// Bodies whose bounding box `box` sweeps through along `direction`.
///
/// The box stays axis aligned for the whole sweep — this is a moving AABB, not
/// a shape cast. For a rotating or non-box swept volume, use
/// zjoltCastShapeAll.
ZJOLT_API ZJoltResult zjoltBroadPhaseCastAABox(
    const ZJoltPhysicsSystem *system, const ZJoltAABox *box,
    const ZJoltVec3 *direction, const ZJoltBroadPhaseFilters *filters,
    ZJoltBroadPhaseCastHit *out_hits, uint32_t capacity, uint32_t *out_count);

/// Bounding box of every body in the broad phase.
///
/// An EMPTY world does not report an empty box: Jolt's quad tree returns its
/// inside-out initial box, whose min is +FLT_MAX and whose max is -FLT_MAX. So
/// check zjoltPhysicsSystemGetNumBodies before reading this as a world extent.
/// A NULL `system` zeroes `out`, which is a degenerate box at the origin and
/// not a world either.
ZJOLT_API void zjoltBroadPhaseGetBounds(const ZJoltPhysicsSystem *system,
                                        ZJoltAABox *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_BROADPHASE_H_
