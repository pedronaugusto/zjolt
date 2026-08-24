//===----------------------------------------------------------------------===//
// zjolt — a shape, placed in the world, queried on its own.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_TRANSFORMED_H_
#define ZJOLT_TRANSFORMED_H_

#include "zjolt_core.h"
#include "zjolt_query.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// A shape and its placement, with no body and no physics system
//
// zjolt_query.h answers "what does this system's geometry look like from
// here" — every query in it takes a ZJoltPhysicsSystem. This is the other
// half: given a shape a caller already has in hand, and a world transform for
// it, run the same kinds of query against it alone. There is no broad phase
// to walk, because there is nothing to find but the one shape already named.
//
// Typical sources of one of these: a broad-phase hit read under a body lock,
// carried past the point where the lock is safe to hold (that is what the
// upstream type this wraps is FOR); or a compound's child, drilled into with
// zjoltShapeGetSubShapeTransformedShape.
//===----------------------------------------------------------------------===//

typedef struct ZJoltTransformedShape ZJoltTransformedShape;

/// Wraps `shape` at the given world placement. Takes a reference on `shape`,
/// so the caller may release theirs once this returns; release the result
/// with zjoltTransformedShapeDestroy.
///
/// `scale` may be NULL for (1, 1, 1). `body` may be ZJOLT_BODY_ID_INVALID
/// when there is no body behind this shape — a filter that inspects the
/// reported body id then sees the invalid id, same as any other query result
/// naming a body-less shape.
ZJOLT_API ZJoltResult zjoltTransformedShapeCreate(
    const ZJoltShape *shape, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *scale, ZJoltBodyId body,
    ZJoltTransformedShape **out);

ZJOLT_API void zjoltTransformedShapeDestroy(ZJoltTransformedShape *ts);

//===----------------------------------------------------------------------===//
// Placement and geometry
//===----------------------------------------------------------------------===//

typedef struct ZJoltTransformedShapeTransform {
  ZJoltRVec3 position;
  ZJoltQuat rotation;
  ZJoltVec3 scale;
} ZJoltTransformedShapeTransform;

/// The world placement `ts` was given — NOT its center-of-mass transform,
/// which sits elsewhere whenever the wrapped shape's own center of mass is
/// offset from where it was authored. Zeroed (scale included) for a NULL
/// `ts`.
ZJOLT_API void zjoltTransformedShapeGetWorldTransform(
    const ZJoltTransformedShape *ts, ZJoltTransformedShapeTransform *out);

/// Repositions `ts` in place, in the same sense zjoltTransformedShapeCreate
/// took its placement in.
///
/// A NULL `scale` SETS the scale to (1, 1, 1); it does not leave the current
/// one alone. Moving an already-scaled shape means passing its scale again —
/// read it back with zjoltTransformedShapeGetWorldTransform first if the
/// caller does not still hold it.
ZJOLT_API void zjoltTransformedShapeSetWorldTransform(
    ZJoltTransformedShape *ts, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *scale);

/// World-space bounds of the wrapped shape at its current placement. Zeroed
/// for a NULL `ts`.
ZJOLT_API void zjoltTransformedShapeGetWorldSpaceBounds(
    const ZJoltTransformedShape *ts, ZJoltAABox *out);

/// Surface normal at world-space `position` on the leaf named by
/// `sub_shape_id`. @see zjoltShapeGetSurfaceNormal for what this is — and is
/// not — a substitute for. Zeroed for a NULL `ts` or `position`.
ZJOLT_API void zjoltTransformedShapeGetWorldSpaceSurfaceNormal(
    const ZJoltTransformedShape *ts, ZJoltSubShapeId sub_shape_id,
    const ZJoltRVec3 *position, ZJoltVec3 *out_normal);

/// The material of the leaf named by `sub_shape_id`. Never NULL for a valid
/// `ts`. @see zjoltShapeGetMaterial.
ZJOLT_API const ZJoltPhysicsMaterial *zjoltTransformedShapeGetMaterial(
    const ZJoltTransformedShape *ts, ZJoltSubShapeId sub_shape_id);

/// @see Shape::GetSubShapeUserData. 0 for a NULL `ts`.
ZJOLT_API uint64_t zjoltTransformedShapeGetSubShapeUserData(
    const ZJoltTransformedShape *ts, ZJoltSubShapeId sub_shape_id);

/// @see zjoltShapeGetSupportingFace; `out_vertices` must hold
/// ZJOLT_SHAPE_MAX_SUPPORTING_FACE_VERTICES entries. `direction` is in world
/// space here, unlike the Shape-level call. `base_offset` is subtracted from
/// the result before it is returned: pass zero for world-space vertices, or a
/// point close to them (`ts`'s own position is usually right) for the
/// precision a double-precision world needs far from the origin.
ZJOLT_API ZJoltResult zjoltTransformedShapeGetSupportingFace(
    const ZJoltTransformedShape *ts, ZJoltSubShapeId sub_shape_id,
    const ZJoltVec3 *direction, const ZJoltRVec3 *base_offset,
    ZJoltVec3 *out_vertices, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Triangle read-back
//
// A second ZJoltShapeTrianglesContext-shaped type rather than that one
// reused: sharing it would mean this header including zjolt_shape.h for a
// scratch buffer's layout alone, the one dependency zjolt_shape.h's own
// comment on the forward-declared ZJoltTransformedShape tag was written to
// avoid taking on in the other direction. @see zjoltShapeGetTrianglesNext for
// the protocol, which is identical.
//===----------------------------------------------------------------------===//

#if defined(__cplusplus)
#define ZJOLT_TRANSFORMED_ALIGNAS_16 alignas(16)
#else
#define ZJOLT_TRANSFORMED_ALIGNAS_16 _Alignas(16)
#endif
typedef struct ZJoltTransformedShapeTrianglesContext {
  ZJOLT_TRANSFORMED_ALIGNAS_16 uint8_t data[4288];
} ZJoltTransformedShapeTrianglesContext;
#undef ZJOLT_TRANSFORMED_ALIGNAS_16

/// Starts a triangle walk over `ts`, restricted to `box` (world space).
/// `base_offset` shifts the returned vertices the same way
/// zjoltTransformedShapeGetSupportingFace's does.
ZJOLT_API ZJoltResult zjoltTransformedShapeGetTrianglesStart(
    const ZJoltTransformedShape *ts,
    ZJoltTransformedShapeTrianglesContext *context, const ZJoltAABox *box,
    const ZJoltRVec3 *base_offset);

/// @see zjoltShapeGetTrianglesNext; identical protocol and the same
/// ZJOLT_SHAPE_MIN_TRIANGLES_REQUESTED floor.
ZJOLT_API ZJoltResult zjoltTransformedShapeGetTrianglesNext(
    const ZJoltTransformedShape *ts,
    ZJoltTransformedShapeTrianglesContext *context, uint32_t max_triangles,
    ZJoltVec3 *out_vertices, const ZJoltPhysicsMaterial **out_materials,
    uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Queries against this one shape
//
// Same three forms as zjolt_query.h where more than one hit is possible — a
// compound wrapped in a single ZJoltTransformedShape still has many leaves —
// except the streaming *Each form, left out here: the result set behind one
// already-resolved shape is bounded by its own leaf count rather than by
// however much of a world a broad phase might otherwise hand back, so the
// allocation avoidance that streaming exists for in zjolt_query.h matters far
// less on this side. Add it if that stops being true for some caller; nothing
// here forecloses it.
//===----------------------------------------------------------------------===//

/// @see zjoltCastRayClosest. `filter` may be NULL to accept every sub-shape.
ZJOLT_API ZJoltResult zjoltTransformedShapeCastRayClosest(
    const ZJoltTransformedShape *ts, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltRayCastSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltRayCastHit *out_hit,
    bool *out_hit_any);

/// @see zjoltCastRayAll for the two-call protocol.
ZJOLT_API ZJoltResult zjoltTransformedShapeCastRayAll(
    const ZJoltTransformedShape *ts, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltRayCastSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltRayCastHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// @see zjoltCollidePointAll.
ZJOLT_API ZJoltResult zjoltTransformedShapeCollidePointAll(
    const ZJoltTransformedShape *ts, const ZJoltRVec3 *point,
    const ZJoltShapeFilter *filter, ZJoltCollidePointHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// Everything in `shape` (at `scale`/`position`/`rotation`) overlapping `ts`.
/// @see zjoltCollideShapeAll; contact points come back relative to
/// `base_offset` for the same precision reason.
///
/// `settings` may be NULL for Jolt's defaults. The separation distance that
/// used to be a parameter of its own here is `settings->max_separation_
/// distance`, for the reason zjolt_query.h's Overlap section gives.
ZJOLT_API ZJoltResult zjoltTransformedShapeCollideShapeAll(
    const ZJoltTransformedShape *ts, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltRVec3 *base_offset,
    const ZJoltCollideShapeSettings *settings, const ZJoltShapeFilter *filter,
    ZJoltCollideShapeHit *out_hits, uint32_t capacity, uint32_t *out_count);

/// Sweeps `shape` from `position`/`rotation` along `direction` and reports
/// the nearest hit against `ts`. @see zjoltCastShapeClosest, including for
/// what a NULL `settings` costs a sweep that starts inside `ts`.
ZJOLT_API ZJoltResult zjoltTransformedShapeCastShapeClosest(
    const ZJoltTransformedShape *ts, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltRVec3 *base_offset, const ZJoltShapeCastSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltShapeCastHit *out_hit,
    bool *out_hit_any);

/// @see zjoltCastShapeAll for the two-call protocol.
ZJOLT_API ZJoltResult zjoltTransformedShapeCastShapeAll(
    const ZJoltTransformedShape *ts, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltRVec3 *base_offset, const ZJoltShapeCastSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltShapeCastHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_TRANSFORMED_H_
