//===----------------------------------------------------------------------===//
// zjolt — triangle collision, active-edge normals, and internal-edge removal.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_COLLISION_H_
#define ZJOLT_COLLISION_H_

#include "zjolt_core.h"
#include "zjolt_query.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Active edges: an edge is "active" when nothing sits on the other side of
// it at a shallow enough angle (see `ZJoltActiveEdgeMode`). These two free
// functions let a custom triangle source supply active-edge bits and a
// corrected normal itself, the way `zjolt_query.h`'s queries do internally.
//===----------------------------------------------------------------------===//

/// Whether the edge between two triangles sharing `normal1`/`normal2` counts
/// as active. `edge_direction` is the vector along the edge; the triangles
/// need not agree on which end is which. False for a NULL `normal1`,
/// `normal2` or `edge_direction`, rather than a crash.
ZJOLT_API bool zjoltActiveEdgesIsEdgeActive(const ZJoltVec3 *normal1,
                                            const ZJoltVec3 *normal2,
                                            const ZJoltVec3 *edge_direction,
                                            float cos_threshold_angle);

/// `normal` if the hit landed on an active edge or interior, `triangle_normal`
/// otherwise — avoids catching on seams between triangles on a sliding body.
///
/// `active_edges`: bit 0 = edge v0..v1, bit 1 = v1..v2, bit 2 = v2..v0.
/// `movement_direction` may be zero. Writes zero to `*out_normal` on a NULL
/// required pointer.
ZJOLT_API void zjoltActiveEdgesFixNormal(
    const ZJoltVec3 *v0, const ZJoltVec3 *v1, const ZJoltVec3 *v2,
    const ZJoltVec3 *triangle_normal, uint8_t active_edges,
    const ZJoltVec3 *point, const ZJoltVec3 *normal,
    const ZJoltVec3 *movement_direction, ZJoltVec3 *out_normal);

//===----------------------------------------------------------------------===//
// Triangle collision: collide/cast a shape against triangles fed in one at
// a time. `scale1`/`position1`/`rotation1` place `shape1`, centre of mass
// folded in; `scale2` scales the triangle SOURCE only (data, not a
// ZJoltShape). Hits report `hit->material` NULL, `hit->body` INVALID.
//
// `base_offset` may be NULL, meaning world space; otherwise hit positions come
// back relative to it. Every Collide/Cast call writes `*out_should_stop`: true
// means the host asked to stop and the caller must not feed further triangles.
//===----------------------------------------------------------------------===//

typedef struct ZJoltCollideConvexVsTriangles ZJoltCollideConvexVsTriangles;
typedef struct ZJoltCastConvexVsTriangles ZJoltCastConvexVsTriangles;
typedef struct ZJoltCollideSphereVsTriangles ZJoltCollideSphereVsTriangles;
typedef struct ZJoltCastSphereVsTriangles ZJoltCastSphereVsTriangles;

/// `shape1` must be a convex shape (`ZJOLT_RESULT_INVALID_ARGUMENT`
/// otherwise — a mesh or a compound has no single support function for this
/// to collide against). `sub_shape_id1` is reported nowhere in
/// `ZJoltCollideShapeHit` today; it exists because Jolt's own constructor
/// takes one.
ZJOLT_API ZJoltResult zjoltCollideConvexVsTrianglesCreate(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, ZJoltSubShapeId sub_shape_id1,
    const ZJoltCollideShapeSettings *settings, ZJoltCollideShapeHitFn on_hit,
    void *user, ZJoltCollideConvexVsTriangles **out);

/// Collides `shape1` with one CCW triangle. `v0`/`v1`/`v2` are in the
/// triangle source's own local space, before `scale2`.
ZJOLT_API ZJoltResult zjoltCollideConvexVsTrianglesCollide(
    ZJoltCollideConvexVsTriangles *collider, const ZJoltVec3 *v0,
    const ZJoltVec3 *v1, const ZJoltVec3 *v2, uint8_t active_edges,
    ZJoltSubShapeId sub_shape_id2, bool *out_should_stop);

ZJOLT_API void zjoltCollideConvexVsTrianglesDestroy(
    ZJoltCollideConvexVsTriangles *collider);

/// `shape1` must be a convex shape, swept from `position1`/`rotation1` along
/// `direction`. @see zjoltCollideConvexVsTrianglesCreate for the rest.
ZJOLT_API ZJoltResult zjoltCastConvexVsTrianglesCreate(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltVec3 *direction, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltShapeCastSettings *settings,
    ZJoltShapeCastHitFn on_hit, void *user, ZJoltCastConvexVsTriangles **out);

/// @see zjoltCollideConvexVsTrianglesCollide.
ZJOLT_API ZJoltResult zjoltCastConvexVsTrianglesCast(
    ZJoltCastConvexVsTriangles *caster, const ZJoltVec3 *v0,
    const ZJoltVec3 *v1, const ZJoltVec3 *v2, uint8_t active_edges,
    ZJoltSubShapeId sub_shape_id2, bool *out_should_stop);

ZJOLT_API void zjoltCastConvexVsTrianglesDestroy(
    ZJoltCastConvexVsTriangles *caster);

/// `shape1` must be a sphere (`ZJOLT_RESULT_INVALID_ARGUMENT` for any other
/// convex shape — Jolt's own constructor takes a `SphereShape*`, not a
/// general `ConvexShape*`). @see zjoltCollideConvexVsTrianglesCreate.
ZJOLT_API ZJoltResult zjoltCollideSphereVsTrianglesCreate(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, ZJoltSubShapeId sub_shape_id1,
    const ZJoltCollideShapeSettings *settings, ZJoltCollideShapeHitFn on_hit,
    void *user, ZJoltCollideSphereVsTriangles **out);

/// @see zjoltCollideConvexVsTrianglesCollide.
ZJOLT_API ZJoltResult zjoltCollideSphereVsTrianglesCollide(
    ZJoltCollideSphereVsTriangles *collider, const ZJoltVec3 *v0,
    const ZJoltVec3 *v1, const ZJoltVec3 *v2, uint8_t active_edges,
    ZJoltSubShapeId sub_shape_id2, bool *out_should_stop);

ZJOLT_API void zjoltCollideSphereVsTrianglesDestroy(
    ZJoltCollideSphereVsTriangles *collider);

/// `shape1` must be a sphere. @see zjoltCastConvexVsTrianglesCreate and
/// zjoltCollideSphereVsTrianglesCreate.
ZJOLT_API ZJoltResult zjoltCastSphereVsTrianglesCreate(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltVec3 *direction, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltShapeCastSettings *settings,
    ZJoltShapeCastHitFn on_hit, void *user, ZJoltCastSphereVsTriangles **out);

/// @see zjoltCollideConvexVsTrianglesCollide.
ZJOLT_API ZJoltResult zjoltCastSphereVsTrianglesCast(
    ZJoltCastSphereVsTriangles *caster, const ZJoltVec3 *v0,
    const ZJoltVec3 *v1, const ZJoltVec3 *v2, uint8_t active_edges,
    ZJoltSubShapeId sub_shape_id2, bool *out_should_stop);

ZJOLT_API void zjoltCastSphereVsTrianglesDestroy(
    ZJoltCastSphereVsTriangles *caster);

//===----------------------------------------------------------------------===//
// Ghost-collision removal: zjoltCollideShapeWithInternalEdgeRemoval* applies
// this against a whole ZJoltPhysicsSystem; for two placed shapes with none.
// `settings->active_edge_mode`/`.collect_faces_mode` are forced to
// COLLIDE_WITH_ALL/COLLECT_FACES regardless of what `settings` carries.
//===----------------------------------------------------------------------===//

/// @see zjoltCollideShapeVsShapeAll for `shape1`/`shape2` placement,
/// `base_offset` and the dispatch refusal (`ZJOLT_RESULT_UNSUPPORTED` for a
/// pair with no convex side), and zjoltCollideShapeEach for `on_hit`.
ZJOLT_API ZJoltResult zjoltCollideShapeWithInternalEdgeRemoval(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHitFn on_hit, void *user);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_COLLISION_H_
