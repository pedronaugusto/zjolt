//===----------------------------------------------------------------------===//
// zjolt — ray casts, shape casts, overlap and point tests.
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
// Filters
//
// Filters are optional at every level: a NULL ZJoltQueryFilters accepts
// everything, and so does a member whose function pointer is NULL.
//
// Jolt consults them cheapest first — broad-phase layer, object layer, body,
// then shape — so a rejection costs only the tests above it.
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

/// Rejects individual shapes, after the body carrying them has been accepted.
///
/// Jolt asks this once per SHAPE, never once per triangle: a mesh is accepted
/// or rejected whole, and a compound is asked about each of its children. None
/// of the shapes this ABI can build today is a compound, so `sub_shape_id` is
/// always ZJOLT_SUB_SHAPE_ID_EMPTY and the answer is one a ZJoltBodyFilter
/// could have given. It is here anyway for two reasons: that stops being true
/// the moment compound shapes are exposed, and Jolt takes a shape filter on
/// every narrow-phase entry point, so leaving it out made those calls
/// unrepresentable from C. To drop individual triangles of a mesh, reject them
/// in the hit callback by `sub_shape_id`, which is where Jolt reports them.
///
/// `query_sub_shape_id` names the sub-shape of the shape being cast or
/// overlapped. A ray and a point have no shape of their own, so they pass
/// ZJOLT_SUB_SHAPE_ID_EMPTY.
typedef struct ZJoltShapeFilter {
  bool (*should_collide)(void *user, ZJoltBodyId body,
                         ZJoltSubShapeId sub_shape_id,
                         ZJoltSubShapeId query_sub_shape_id);
  void *user;
} ZJoltShapeFilter;

typedef struct ZJoltQueryFilters {
  ZJoltBroadPhaseLayerFilter broad_phase_layer;
  ZJoltObjectLayerFilter object_layer;
  ZJoltBodyFilter body;
  ZJoltShapeFilter shape;
} ZJoltQueryFilters;

//===----------------------------------------------------------------------===//
// Hits
//===----------------------------------------------------------------------===//

typedef struct ZJoltRayCastHit {
  ZJoltBodyId body;
  ZJoltSubShapeId sub_shape_id;
  /// Hit point is origin + fraction * direction, with fraction in [0, 1].
  float fraction;
  /// World-space surface normal at the hit, already resolved.
  ///
  /// Jolt does not put it in its own result. Reaching it needs the
  /// TransformedShape the collector holds while it reports a hit, and that
  /// pointer is live for exactly one call — Jolt does not clear it afterwards,
  /// so a kept copy points at freed memory. Handing it across the ABI would
  /// export that trap. Resolving it here does not.
  ZJoltVec3 normal;
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
  /// `-normalize(penetration_axis)` is the contact normal, and Jolt recommends
  /// it over a face normal because it is defined at vertices and edges too.
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
  /// @see ZJoltShapeCastHit::penetration_axis
  ZJoltVec3 penetration_axis;
  float penetration_depth;
} ZJoltCollideShapeHit;

/// A shape a point was found inside. There is no contact geometry to report:
/// the point is either in or out.
typedef struct ZJoltCollidePointHit {
  ZJoltBodyId body;
  ZJoltSubShapeId sub_shape_id;
} ZJoltCollidePointHit;

//===----------------------------------------------------------------------===//
// The three forms, and the one traversal underneath them
//
// Jolt finds hits with a collector — an object it calls once per hit, as it
// finds them. All three shapes a caller wants out of that are built on one
// streaming collector here, so there is a single traversal path rather than
// three that can drift apart:
//
//   *Closest  the best hit, and whether there was one
//   *All      every hit, into the caller's buffer, with the true count
//             reported even when the buffer was too small
//   *Each     every hit, handed to a callback as it is found
//
// The streaming form is the one that materialises nothing. It matters most for
// overlap queries: Jolt's own AllHitCollisionCollector accumulates into an
// unbounded array, and a CollideShapeResult is about a kilobyte because it
// embeds two 32-vertex face arrays — sized that way whether or not faces were
// asked for. No form here builds that array any more, and the count-then-fill
// pair is one traversal per call rather than one per call twice over.
//
// NOTHING MAY UNWIND OUT OF A HIT CALLBACK. This is the rule with teeth. Jolt
// is compiled with exceptions off, so an exception crossing a collector is
// std::terminate; worse, a stack unwind through one skips the broad phase's
// read-lock destructor and PERMANENTLY DEADLOCKS the next step, with nothing
// at the point of failure to say why. A callback that has failed must record
// that where it can reach it, return ZJOLT_HIT_ACTION_STOP, and raise the
// failure after the query has returned.
//
// Two things a callback does NOT get, both decisions rather than oversights:
//
//   * The TransformedShape Jolt is holding. It is live for exactly one call
//     and Jolt does not clear it afterwards, so it cannot be handed out
//     safely. What a caller wants from it — the surface normal — is resolved
//     into the hit instead. The other thing it carries is the surface's
//     physics material, which is not resolved because no entry point in this
//     ABI attaches a material to a shape yet: every hit would report Jolt's
//     default one. It goes in when materials do.
//   * A per-body bracket. Jolt calls OnBody / OnBodyEnd around the hits
//     belonging to one body, and that is the only point in a query where a
//     JPH::Body is readable. Its base-class versions do nothing, so leaving
//     them out costs nothing today and adding them later is purely additive.
//===----------------------------------------------------------------------===//

/// What a streaming query does after a hit.
///
/// The narrowed fraction is computed from the hit rather than taken from the
/// caller, and that is the whole reason this is an enum. Jolt's collectors
/// assert that the early-out fraction only ever decreases, and with assertions
/// compiled out an increasing one violates preconditions the narrow phase
/// asserts on — which surfaces as a crash inside Jolt with the caller's
/// mistake nowhere in sight. A float parameter would put that in every
/// caller's hands. An enum cannot be got wrong.
///
/// A value outside this enum is treated as ZJOLT_HIT_ACTION_CONTINUE, that
/// being the one answer which can neither lose a hit nor break a precondition.
typedef enum ZJoltHitAction {
  /// Keep going, accepting hits at any distance. Hit order is UNSPECIFIED.
  ZJOLT_HIT_ACTION_CONTINUE = 0,
  /// Keep going, but only accept hits better than this one: nearer for a ray
  /// or a shape cast, deeper for an overlap.
  ///
  /// Narrowing on every hit is what gives the traversal an order: each hit is
  /// then strictly better than the one before it, so hits arrive WORST FIRST
  /// and the last one is the best. Narrowing on some hits and not others
  /// orders nothing.
  ///
  /// Expect FEWER hits, not sorted ones. Pruning is the point and ordering is
  /// the side effect — the broad phase already walks roughly front to back, so
  /// a ray that reports five hits without narrowing may well report one with
  /// it. Reach for this to avoid work, not to sort.
  ///
  /// For a ray this can end the query outright: a hit at fraction 0 leaves
  /// nothing that could be nearer, so Jolt stops. That is correct rather than
  /// a surprise, and it is only reachable because narrowing is opt-in.
  ZJOLT_HIT_ACTION_NARROW = 1,
  /// End the query now. Hits already reported stand.
  ZJOLT_HIT_ACTION_STOP = 2,
} ZJoltHitAction;

/// Called once per hit, from inside the traversal that found it.
///
/// `hit` is borrowed for the duration of the call only. @see ZJoltHitAction,
/// and the note above it about unwinding.
typedef ZJoltHitAction (*ZJoltRayCastHitFn)(void *user,
                                            const ZJoltRayCastHit *hit);
typedef ZJoltHitAction (*ZJoltShapeCastHitFn)(void *user,
                                              const ZJoltShapeCastHit *hit);
typedef ZJoltHitAction (*ZJoltCollideShapeHitFn)(
    void *user, const ZJoltCollideShapeHit *hit);
typedef ZJoltHitAction (*ZJoltCollidePointHitFn)(
    void *user, const ZJoltCollidePointHit *hit);

//===----------------------------------------------------------------------===//
// Ray casts
//===----------------------------------------------------------------------===//

/// How a ray treats surfaces it meets from behind, and shapes it starts
/// inside. NULL takes these defaults, which are Jolt's.
///
/// This is exposed because the three ray forms share one traversal and so one
/// set of settings. Jolt's single-hit CastRay overload — which the closest-hit
/// form used to be — bakes in a different answer for triangles than its
/// collector overload does, and burying that would have left the closest hit
/// and the full list disagreeing about whether the underside of a mesh is a
/// surface.
typedef struct ZJoltRayCastSettings {
  /// Whether a triangle hit from behind counts. Applies to mesh shapes.
  ZJoltBackFaceMode back_face_mode_triangles;
  /// Whether the far side of a convex shape counts, reported as a second hit.
  ZJoltBackFaceMode back_face_mode_convex;
  /// When true, a ray starting inside a convex shape hits it at fraction 0.
  /// When false, such a ray passes through it.
  bool treat_convex_as_solid;
} ZJoltRayCastSettings;

/// Fills `settings` with the defaults described above.
ZJOLT_API void zjoltRayCastSettingsInit(ZJoltRayCastSettings *settings);

/// The nearest hit along the ray. `direction` carries the ray's length:
/// nothing beyond it is reported. `*out_hit_any` says whether `out_hit` was
/// written.
ZJOLT_API ZJoltResult zjoltCastRayClosest(const ZJoltPhysicsSystem *system,
                                          const ZJoltRVec3 *origin,
                                          const ZJoltVec3 *direction,
                                          const ZJoltRayCastSettings *settings,
                                          const ZJoltQueryFilters *filters,
                                          ZJoltRayCastHit *out_hit,
                                          bool *out_hit_any);

/// Every hit along the ray, in an unspecified order, written into `out_hits`.
///
/// Two-call protocol, as everywhere here: `*out_count` always receives the
/// true number of hits, so a capacity of 0 (with out_hits = NULL) is a size
/// query. A short buffer receives the first `capacity` hits and reports
/// ZJOLT_RESULT_BUFFER_TOO_SMALL with the true count; WHICH hits those are is
/// unspecified, for the same reason the order is.
ZJOLT_API ZJoltResult zjoltCastRayAll(const ZJoltPhysicsSystem *system,
                                      const ZJoltRVec3 *origin,
                                      const ZJoltVec3 *direction,
                                      const ZJoltRayCastSettings *settings,
                                      const ZJoltQueryFilters *filters,
                                      ZJoltRayCastHit *out_hits,
                                      uint32_t capacity, uint32_t *out_count);

/// Every hit along the ray, handed to `on_hit` as it is found: one traversal,
/// no intermediate storage. @see ZJoltHitAction for what `on_hit` returns and
/// what it must never do.
ZJOLT_API ZJoltResult zjoltCastRayEach(const ZJoltPhysicsSystem *system,
                                       const ZJoltRVec3 *origin,
                                       const ZJoltVec3 *direction,
                                       const ZJoltRayCastSettings *settings,
                                       const ZJoltQueryFilters *filters,
                                       ZJoltRayCastHitFn on_hit, void *user);

//===----------------------------------------------------------------------===//
// Shape casts
//===----------------------------------------------------------------------===//

/// Sweeps `shape` from `position`/`rotation` along `direction` and reports the
/// nearest hit. `scale` may be NULL for (1,1,1).
ZJOLT_API ZJoltResult zjoltCastShapeClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltQueryFilters *filters, ZJoltShapeCastHit *out_hit,
    bool *out_hit_any);

/// Every hit along the sweep. @see zjoltCastRayAll for the protocol.
ZJOLT_API ZJoltResult zjoltCastShapeAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltQueryFilters *filters, ZJoltShapeCastHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// Every hit along the sweep, streamed. @see zjoltCastRayEach.
ZJOLT_API ZJoltResult zjoltCastShapeEach(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltQueryFilters *filters, ZJoltShapeCastHitFn on_hit, void *user);

//===----------------------------------------------------------------------===//
// Overlap
//
// A `max_separation_distance` above zero reports near misses too, with a
// negative penetration depth — "is there anything within a metre of here".
//===----------------------------------------------------------------------===//

/// Everything overlapping `shape` placed at `position`/`rotation`.
/// @see zjoltCastRayAll for the protocol.
ZJOLT_API ZJoltResult zjoltCollideShapeAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, float max_separation_distance,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// Everything overlapping the shape, streamed. This is the form the streaming
/// path was built for: an overlap hit is the expensive one to accumulate.
/// @see zjoltCastRayEach.
ZJOLT_API ZJoltResult zjoltCollideShapeEach(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, float max_separation_distance,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user);

//===----------------------------------------------------------------------===//
// Point
//
// Every shape containing the point, all of them treated as solid. A mesh
// answers this usefully only if it is a closed manifold — an open mesh has no
// inside for a point to be in.
//===----------------------------------------------------------------------===//

/// @see zjoltCastRayAll for the protocol.
ZJOLT_API ZJoltResult zjoltCollidePointAll(const ZJoltPhysicsSystem *system,
                                           const ZJoltRVec3 *point,
                                           const ZJoltQueryFilters *filters,
                                           ZJoltCollidePointHit *out_hits,
                                           uint32_t capacity,
                                           uint32_t *out_count);

/// @see zjoltCastRayEach. ZJOLT_HIT_ACTION_NARROW does nothing useful here:
/// point hits have no distance to be nearer by, so there is no order to
/// impose. ZJOLT_HIT_ACTION_STOP answers "is this point inside anything"
/// without visiting the rest.
ZJOLT_API ZJoltResult zjoltCollidePointEach(const ZJoltPhysicsSystem *system,
                                            const ZJoltRVec3 *point,
                                            const ZJoltQueryFilters *filters,
                                            ZJoltCollidePointHitFn on_hit,
                                            void *user);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_QUERY_H_
