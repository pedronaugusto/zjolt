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
/// or rejected whole, and a compound is asked about each of its children, so
/// `sub_shape_id` names the child. Against a shape with no children it is
/// ZJOLT_SUB_SHAPE_ID_EMPTY and the answer is one a ZJoltBodyFilter could have
/// given; against a compound it is the only way to reject one child and keep
/// the rest. To drop individual triangles of a mesh, reject them in the hit
/// callback by `sub_shape_id`, which is where Jolt reports them.
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
  /// The material of the surface at `sub_shape_id`, resolved the same way
  /// `normal` is. Borrowed from the shape the hit body owns; it outlives this
  /// hit but not necessarily the body — take a reference with
  /// zjoltPhysicsMaterialAddRef to keep it past a body lock. Never NULL for a
  /// real hit; NULL only if the shape was released between the hit and the
  /// read, which a caller holding a body lock cannot cause.
  const ZJoltPhysicsMaterial *material;
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
  /// The material at `sub_shape_id` on the shape that was hit (not the shape
  /// that was cast). @see ZJoltRayCastHit::material for the borrowing rule.
  const ZJoltPhysicsMaterial *material;
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
  /// The material at `sub_shape_id` on the shape already in the system (not
  /// the shape the query passed in). @see ZJoltRayCastHit::material.
  const ZJoltPhysicsMaterial *material;
} ZJoltCollideShapeHit;

/// A shape a point was found inside. There is no contact geometry to report:
/// the point is either in or out.
typedef struct ZJoltCollidePointHit {
  ZJoltBodyId body;
  ZJoltSubShapeId sub_shape_id;
  /// The material at `sub_shape_id`. @see ZJoltRayCastHit::material.
  const ZJoltPhysicsMaterial *material;
} ZJoltCollidePointHit;

//===----------------------------------------------------------------------===//
// The forms, and the one traversal underneath them
//
// Jolt finds hits with a collector — an object it calls once per hit, as it
// finds them. The shapes a caller wants out of that are built on one
// streaming collector here, so there is a single traversal path rather than
// several that can drift apart:
//
//   *Closest  the best hit, and whether there was one
//   *All      every hit, into the caller's buffer, with the true count
//             reported even when the buffer was too small
//   *Each     every hit, handed to a callback as it is found
//
// Not every query family has all three. A ray cast and a shape cast both have
// a real distance to travel, so "closest" is unambiguous: the smallest
// fraction, which is also what ZJOLT_HIT_ACTION_NARROW already prunes by. An
// overlap test (zjoltCollideShape*) has no travel distance, but it does have
// a comparable depth — Jolt's own CollideShapeResult::GetEarlyOutFraction is
// `-penetration_depth` — so "closest" there means DEEPEST, and
// zjoltCollideShapeClosest is exactly as well-defined as the other two.
//
// A point test (zjoltCollidePoint*) has neither. CollidePointResult::
// GetEarlyOutFraction is a constant 0.0f for every hit: a point is either
// inside a shape or it is not, with no scalar that makes one "closer" than
// another. A zjoltCollidePointClosest built on this collector would silently
// return whichever hit the traversal happened to visit first — an accident
// of internal order dressed up as an answer — so it is deliberately not
// offered. Use zjoltCollidePointEach with ZJOLT_HIT_ACTION_STOP if "is this
// point inside anything at all" is the actual question.
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
//     safely. The two things a caller actually wants from it — the surface
//     normal and the surface material — are resolved into the hit instead,
//     the same way and for the same reason.
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

/// The single deepest overlap of `shape` placed at `position`/`rotation` —
/// the hit with the largest `penetration_depth`. @see the note above on why
/// this exists for an overlap test and not for a point test.
ZJOLT_API ZJoltResult zjoltCollideShapeClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, float max_separation_distance,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any);

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

//===----------------------------------------------------------------------===//
// Shape versus shape
//
// Two shapes, two placements, and no physics system anywhere: "would these
// overlap if I put them here", answered without creating a body, adding it to
// a world or stepping anything. Placement validation, an editor's drag handle,
// a sweep against geometry that is not in the world.
//
// Every other query in this file walks a broad phase and reports the BODIES it
// found. These have no broad phase and no body, so every hit reports
// ZJOLT_BODY_ID_INVALID and a ZJoltShapeFilter is handed that same invalid id
// — the two sub-shape ids are all it has to decide on, and against shapes with
// no children both are ZJOLT_SUB_SHAPE_ID_EMPTY. `material` still resolves; it
// comes off the second shape, as it does everywhere else here.
//
// These are also the only entry points in this file that take Jolt's collision
// settings. The rest use its defaults with no way to say otherwise.
//
// WHAT JOLT ASSERTS, AND WHAT IS REFUSED INSTEAD
//
// Jolt dispatches a pair through a table indexed by both sub-types, and the
// cells nothing ever registered assert "Unsupported shape pair" — a mesh
// against a mesh, a height field against a plane, any two surfaces with no
// convex shape between them. That is a real gap rather than an oversight:
// neither surface has an inside, so there is nothing for the narrow phase to
// separate. With assertions compiled out the same cell is a function that
// returns without reporting anything, so the answer would be a confident "no
// overlap" for a query that never ran. These refuse it up front with
// ZJOLT_RESULT_UNSUPPORTED, in every build.
//
// The check looks THROUGH a compound or a decorated shape rather than at it,
// because Jolt does: those re-dispatch whatever they hold, so a compound of
// meshes against a mesh reaches the same unregistered cell a bare mesh would.
// Wrapping does not rescue a pair. What decides the answer is the leaves, and
// the rule is that one side has to be convex all the way down.
//
// A scale the shape cannot take is the other assert: a sphere insists on a
// uniform one, every shape refuses a zero one. zjoltShapeIsValidScale asks the
// same question ahead of time and zjoltShapeMakeScaleValid answers with the
// nearest scale that passes; here it is ZJOLT_RESULT_INVALID_ARGUMENT.
//===----------------------------------------------------------------------===//

/// Whether a triangle edge that a neighbouring triangle already covers can
/// produce a contact.
///
/// An edge is "active" when nothing sits on the other side of it at a shallow
/// enough angle. Colliding with the inactive ones is what makes a box slid
/// across a flat mesh catch on the seams between its triangles, so simulation
/// wants only the active ones. A query asking what geometry is really there
/// usually wants all of them.
typedef enum ZJoltActiveEdgeMode {
  /// Jolt's default. A contact on an edge a neighbour covers is dropped.
  ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_ONLY_WITH_ACTIVE = 0,
  ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL = 1,
} ZJoltActiveEdgeMode;

/// Whether Jolt works out the colliding FACE as well as the contact point.
///
/// Note the numbering: collecting is 0 and not collecting is 1, so a
/// zero-initialised settings struct asks for faces. Fill one through
/// zjoltCollideShapeSettingsInit rather than with memset and the question does
/// not come up.
typedef enum ZJoltCollectFacesMode {
  ZJOLT_COLLECT_FACES_MODE_COLLECT_FACES = 0,
  /// Jolt's default.
  ZJOLT_COLLECT_FACES_MODE_NO_FACES = 1,
} ZJoltCollectFacesMode;

/// Options for a shape-versus-shape overlap. NULL takes Jolt's defaults, which
/// are what zjoltCollideShapeSettingsInit writes.
typedef struct ZJoltCollideShapeSettings {
  /// @see ZJoltActiveEdgeMode.
  ZJoltActiveEdgeMode active_edge_mode;
  /// Faces are computed into Jolt's own result and ZJoltCollideShapeHit does
  /// not carry them, so asking for them here buys nothing today and costs the
  /// work of finding them. It is exposed because it is the other half of what
  /// `active_edge_movement_direction` below is for, and because a settings
  /// struct that quietly drops a field is worse than one that explains it.
  /// Reach for zjoltTransformedShapeGetSupportingFace when the face itself is
  /// the answer wanted.
  ZJoltCollectFacesMode collect_faces_mode;
  /// How close counts as touching inside GJK, in metres. Jolt's default is
  /// 1e-4.
  float collision_tolerance;
  /// How precisely the penetration depth is resolved, as a dimensionless
  /// factor: EPA stops once the squared distance moves by less than this times
  /// the current depth squared. Jolt's default is 1e-4, and Jolt ASSERTS that
  /// it is at least FLT_EPSILON — anything smaller only buys iterations — so
  /// a smaller one is refused here with ZJOLT_RESULT_INVALID_ARGUMENT.
  float penetration_tolerance;
  /// Which way the first shape is moving, for the inactive edges
  /// `active_edge_mode` kept. Jolt takes the triangle's own normal over the
  /// calculated penetration axis when doing so impedes this direction less.
  /// Zero, the default, leaves the calculated axis alone.
  ZJoltVec3 active_edge_movement_direction;
  /// Above zero, near misses are reported too, with a NEGATIVE
  /// penetration_depth giving the gap — "is anything within a metre of here".
  /// The contact points are then the closest points rather than touching ones.
  float max_separation_distance;
  /// Whether a triangle met from behind counts. Applies to mesh and height
  /// field shapes; an overlap test has no direction of travel, which is why
  /// this is one mode where a shape cast below has two.
  ZJoltBackFaceMode back_face_mode;
} ZJoltCollideShapeSettings;

/// Options for a shape-versus-shape sweep. NULL takes Jolt's defaults, which
/// are what zjoltShapeCastSettingsInit writes.
typedef struct ZJoltShapeCastSettings {
  /// @see ZJoltCollideShapeSettings for these five. They are the same fields
  /// on the same Jolt base class, and `penetration_tolerance` carries the same
  /// refusal.
  ZJoltActiveEdgeMode active_edge_mode;
  ZJoltCollectFacesMode collect_faces_mode;
  float collision_tolerance;
  float penetration_tolerance;
  ZJoltVec3 active_edge_movement_direction;
  /// Grows the swept shape by this margin in every direction, in metres. A
  /// character's skin width, without building a second shape to hold it.
  float extra_convex_radius;
  /// Whether the sweep reports passing through a triangle from behind. Mesh
  /// and height field shapes.
  ZJoltBackFaceMode back_face_mode_triangles;
  /// Whether a sweep that STARTS inside a convex shape reports coming back out
  /// of it. Ignoring back faces, which is Jolt's default, means such a sweep
  /// reports nothing at all for that shape — rarely what a placement test
  /// wants, and the usual reason a sweep from inside geometry comes back
  /// empty.
  ZJoltBackFaceMode back_face_mode_convex;
  /// Shrink the swept shape by its convex radius and expand the result again.
  /// Faster, and gives a more accurate normal, at the cost of rounding the
  /// shape's corners.
  bool use_shrunken_shape_and_convex_radius;
  /// When the two already overlap where the sweep starts (fraction 0), work
  /// out the deepest point rather than reporting the first one found. Costs an
  /// extra EPA pass, and without it a hit at fraction 0 carries a penetration
  /// depth that is not the deepest available.
  bool return_deepest_point;
} ZJoltShapeCastSettings;

/// Fills `settings` with Jolt's defaults, which is the thing to change one
/// field of. A zeroed struct is NOT the same — @see ZJoltCollectFacesMode.
ZJOLT_API void zjoltCollideShapeSettingsInit(
    ZJoltCollideShapeSettings *settings);

/// @see zjoltCollideShapeSettingsInit.
ZJOLT_API void zjoltShapeCastSettingsInit(ZJoltShapeCastSettings *settings);

/// The deepest overlap between two placed shapes, or nothing.
///
/// `scale1` and `scale2` may be NULL for (1, 1, 1). `position` and `rotation`
/// are each shape's own world placement, NOT its centre of mass; Jolt collides
/// in centre-of-mass space and the conversion happens here, so a shape built
/// with an offset centre of mass sits where it was asked to.
///
/// Contact points come back RELATIVE TO `base_offset`, which may be NULL for
/// world space. They are floats: in a double-precision build a contact point
/// far from the origin does not survive the conversion, and passing one of the
/// two positions here is what keeps the precision.
///
/// `*out_hit_any` says whether `out_hit` was written.
ZJOLT_API ZJoltResult zjoltCollideShapeVsShapeClosest(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any);

/// Every overlap between two placed shapes — one per pair of leaves that
/// touch, so a compound or a mesh reports many. @see zjoltCastRayAll for the
/// two-call protocol and zjoltCollideShapeVsShapeClosest for the arguments.
ZJOLT_API ZJoltResult zjoltCollideShapeVsShapeAll(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// Sweeps `shape1` from its placement along `direction` and reports the
/// nearest hit against `shape2`, which does not move.
///
/// `direction` carries the sweep's length: nothing beyond it is reported, and
/// the swept shape's centre of mass at the hit is where it started plus
/// `fraction * direction`. A sweep that starts already overlapping reports
/// fraction 0 — but only if `settings->back_face_mode_convex` says to collide
/// with back faces, which Jolt's default does not.
///
/// @see zjoltCollideShapeVsShapeClosest for scale, placement and base offset.
ZJOLT_API ZJoltResult zjoltCastShapeVsShapeClosest(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltVec3 *direction, const ZJoltShape *shape2,
    const ZJoltVec3 *scale2, const ZJoltRVec3 *position2,
    const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltShapeCastSettings *settings, const ZJoltShapeFilter *filter,
    ZJoltShapeCastHit *out_hit, bool *out_hit_any);

/// Every hit along the sweep. @see zjoltCastRayAll for the two-call protocol
/// and zjoltCastShapeVsShapeClosest for the arguments.
ZJOLT_API ZJoltResult zjoltCastShapeVsShapeAll(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltVec3 *direction, const ZJoltShape *shape2,
    const ZJoltVec3 *scale2, const ZJoltRVec3 *position2,
    const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltShapeCastSettings *settings, const ZJoltShapeFilter *filter,
    ZJoltShapeCastHit *out_hits, uint32_t capacity, uint32_t *out_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_QUERY_H_
