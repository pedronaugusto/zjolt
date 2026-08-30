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

/// Rejects whole bodies. Jolt asks twice: `should_collide` by id with no
/// lock held, then `should_collide_locked` with the body once it is locked.
/// A NULL callback accepts. `body` is borrowed for the call only: read it
/// with the `zjoltBody*Locked` accessors, do not store it, and do not call
/// anything that takes a body lock from inside the callback.
typedef struct ZJoltBodyFilter {
  bool (*should_collide)(void *user, ZJoltBodyId body);
  bool (*should_collide_locked)(void *user, const ZJoltBody *body);
  void *user;
} ZJoltBodyFilter;

/// Rejects individual shapes, once the shape's body has been accepted, once
/// per SHAPE and never per triangle — reject mesh triangles by
/// `sub_shape_id` in the hit callback instead. `shape`/`sub_shape_id` name
/// what is collided against (EMPTY id when childless); `query_shape` and
/// `query_sub_shape_id` name what is cast, NULL/EMPTY for a ray or a point.
/// Both shapes are borrowed: read them, never store or release them.
typedef struct ZJoltShapeFilter {
  bool (*should_collide)(void *user, ZJoltBodyId body,
                         const ZJoltShape *shape,
                         ZJoltSubShapeId sub_shape_id,
                         const ZJoltShape *query_shape,
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
  /// World-space surface normal at the hit, already resolved — Jolt's own
  /// result does not carry one.
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
// The forms, and the one traversal underneath them:
//
//   *Closest  the best hit, and whether there was one
//   *All      every hit, into the caller's buffer, true count always reported
//   *Each     every hit, handed to a callback as found
//
// Point tests have no *Closest: every hit's fraction is a constant 0.0. Use
// zjoltCollidePointEach with ZJOLT_HIT_ACTION_STOP instead.
//
// NOTHING MAY UNWIND OUT OF A HIT CALLBACK. Exceptions are off; a stack
// unwind through a collector PERMANENTLY DEADLOCKS the next step. Record a
// failure elsewhere, return ZJOLT_HIT_ACTION_STOP, and raise it afterward.
//===----------------------------------------------------------------------===//

/// What a streaming query does after a hit, returned by the query's hit
/// callback. A value outside this enum is treated as
/// ZJOLT_HIT_ACTION_CONTINUE.
typedef enum ZJoltHitAction {
  /// Keep going, accepting hits at any distance. Hit order is UNSPECIFIED.
  ZJOLT_HIT_ACTION_CONTINUE = 0,
  /// Keep going, but only accept hits better than this one: nearer for a ray
  /// or shape cast, deeper for an overlap. Narrowing on EVERY hit makes hits
  /// arrive WORST FIRST; narrowing on only some orders nothing. For a ray, a
  /// hit at fraction 0 ends the query outright — nothing can be nearer.
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
/// inside. NULL takes these defaults, which are Jolt's. Shared by
/// zjoltCastRayClosest/All/Each, so all three agree on what counts as a
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

/// Every hit along the ray, in an unspecified order, written into
/// `out_hits`. Two-call protocol, as everywhere here: `*out_count` always
/// receives the true number of hits, so capacity 0 (out_hits = NULL) sizes
/// the buffer. A short buffer gets the first `capacity` hits and
/// ZJOLT_RESULT_BUFFER_TOO_SMALL; WHICH hits those are is unspecified.
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
// Collision settings for shape casts and overlaps
//
// Declared here rather than beside the queries that were once the only ones to
// take them: every shape cast and every overlap in this file takes one now, as
// do zjolt_transformed.h's. A ray cast has its own, smaller, above.
//===----------------------------------------------------------------------===//

/// Whether a triangle edge that a neighbouring triangle already covers can
/// produce a contact. An edge is "active" when nothing covers it at a
/// shallow enough angle. Simulation wants only active edges (avoids
/// catching on mesh seams); a query asking what geometry is really there
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

/// Options for an overlap test, taken by every zjoltCollideShape* entry point
/// here and by zjoltTransformedShapeCollideShapeAll. NULL takes Jolt's
/// defaults, which are what zjoltCollideShapeSettingsInit writes.
typedef struct ZJoltCollideShapeSettings {
  /// @see ZJoltActiveEdgeMode.
  ZJoltActiveEdgeMode active_edge_mode;
  /// Computed into Jolt's own result, but ZJoltCollideShapeHit does not carry
  /// faces — setting this buys nothing today, only costs the work. Reach for
  /// zjoltTransformedShapeGetSupportingFace when the face itself is wanted.
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
  /// Whether a triangle met from behind counts. Applies to mesh and
  /// height field shapes. The matching shape-cast setting below has two
  /// modes (triangle and convex) since a sweep has a direction; this has
  /// only one.
  ZJoltBackFaceMode back_face_mode;
  /// How close two vertices must be (max squared distance, in metres
  /// squared) to count as the same one when deciding whether an edge is
  /// shared between triangles. Read only by the
  /// zjoltCollideShapeWithInternalEdgeRemoval* family below; every other
  /// query in this header ignores it. Jolt's default is 1e-8 (a 1e-4 metre
  /// tolerance, squared).
  float internal_edge_removal_vertex_tolerance_sq;
} ZJoltCollideShapeSettings;

/// Options for a sweep, taken by every zjoltCastShape* entry point here and by
/// the zjoltTransformedShapeCastShape* pair. NULL takes Jolt's defaults, which
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

//===----------------------------------------------------------------------===//
// Shape casts
//===----------------------------------------------------------------------===//

/// Sweeps `shape` from `position`/`rotation` along `direction` and reports
/// the nearest hit. `scale` may be NULL for (1,1,1).
///
/// `settings` NULL takes Jolt's defaults, which ignore back faces — a sweep
/// that STARTS inside geometry then reports nothing until
/// `back_face_mode_convex` or `back_face_mode_triangles` says otherwise.
ZJOLT_API ZJoltResult zjoltCastShapeClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHit *out_hit, bool *out_hit_any);

/// Every hit along the sweep. @see zjoltCastRayAll for the protocol and
/// zjoltCastShapeClosest for `settings`.
ZJOLT_API ZJoltResult zjoltCastShapeAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHit *out_hits, uint32_t capacity, uint32_t *out_count);

/// Every hit along the sweep, streamed. @see zjoltCastRayEach.
ZJOLT_API ZJoltResult zjoltCastShapeEach(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHitFn on_hit, void *user);

//===----------------------------------------------------------------------===//
// Overlap. `settings->max_separation_distance` above zero reports near
// misses too, as a negative penetration depth: "anything within a metre
// of here".
//===----------------------------------------------------------------------===//

/// The single deepest overlap of `shape` placed at `position`/`rotation` —
/// the hit with the largest `penetration_depth`. @see the note above on why
/// this exists for an overlap test and not for a point test.
ZJOLT_API ZJoltResult zjoltCollideShapeClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any);

/// Everything overlapping `shape` placed at `position`/`rotation`.
/// @see zjoltCastRayAll for the protocol.
ZJOLT_API ZJoltResult zjoltCollideShapeAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// Everything overlapping the shape, streamed. This is the form the streaming
/// path was built for: an overlap hit is the expensive one to accumulate.
/// @see zjoltCastRayEach.
ZJOLT_API ZJoltResult zjoltCollideShapeEach(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user);

//===----------------------------------------------------------------------===//
// Overlap, with internal edge removal — the fix
// zjoltCharacterSetEnhancedInternalEdgeRemoval applies automatically for a
// character, run here by hand: drops a contact on a triangle edge that is
// really another triangle's interior, not a real seam.
//
// Same forms/settings/filters as zjoltCollideShape* above, except
// `active_edge_mode`/`collect_faces_mode` are forced to COLLIDE_WITH_ALL/
// COLLECT_FACES regardless of `settings`, and
// `internal_edge_removal_vertex_tolerance_sq` is read (ignored elsewhere).
// Versus zjoltCollideShapeEach, hits can arrive later and out of order:
// Jolt buffers and sorts them deepest-first per body before deciding which
// survive.
//===----------------------------------------------------------------------===//

/// @see zjoltCollideShapeClosest.
ZJOLT_API ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any);

/// @see zjoltCollideShapeAll for the two-call protocol.
ZJOLT_API ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// @see zjoltCollideShapeEach.
ZJOLT_API ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalEach(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
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
// Observing a hit's body before the hit: CollisionCollector::OnBody fires
// once per body, under the traversal's own lock, strictly before any of
// that body's hits reach on_hit — for filtering/annotating by user data,
// layer or motion type without a second lookup inside on_hit.
//
// Not offered on *Closest/*All (neither runs host code during traversal);
// only alongside on_hit, as *EachWithBody. OnBodyEnd is deliberately not
// offered either — the per-hit callback has nothing left to close out once
// a body's last hit has arrived.
//===----------------------------------------------------------------------===//

/// Called once per body, from inside the traversal, before any of that
/// body's hits reach the query's on_hit callback.
///
/// `body` is valid only for this call — copy out what is needed before
/// returning; do not keep the pointer past it. Under the traversal's own
/// lock, every zjoltBody*Locked accessor (zjolt_body.h) may be called on it.
typedef void (*ZJoltOnBodyFn)(void *user, const ZJoltBody *body);

/// @see zjoltCastRayEach; the same ray and the same traversal, with `on_body`
/// additionally called once per body before its hits reach `on_hit`.
ZJOLT_API ZJoltResult zjoltCastRayEachWithBody(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltRayCastSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltRayCastHitFn on_hit, void *user,
    ZJoltOnBodyFn on_body, void *on_body_user);

/// @see zjoltCastShapeEach and zjoltCastRayEachWithBody.
ZJOLT_API ZJoltResult zjoltCastShapeEachWithBody(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHitFn on_hit, void *user, ZJoltOnBodyFn on_body,
    void *on_body_user);

/// @see zjoltCollideShapeEach and zjoltCastRayEachWithBody.
ZJOLT_API ZJoltResult zjoltCollideShapeEachWithBody(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user, ZJoltOnBodyFn on_body, void *on_body_user);

/// @see zjoltCollideShapeWithInternalEdgeRemovalEach and
/// zjoltCastRayEachWithBody.
ZJOLT_API ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalEachWithBody(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user, ZJoltOnBodyFn on_body, void *on_body_user);

/// @see zjoltCollidePointEach and zjoltCastRayEachWithBody.
ZJOLT_API ZJoltResult zjoltCollidePointEachWithBody(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *point,
    const ZJoltQueryFilters *filters, ZJoltCollidePointHitFn on_hit,
    void *user, ZJoltOnBodyFn on_body, void *on_body_user);

//===----------------------------------------------------------------------===//
// The unlocked path: the locked queries above take PhysicsSystem's body
// locks — correct outside a step, but deadlocks inside one where a contact
// listener, combine callback or step listener already holds them. These
// *NoLock twins skip that lock, the same trade
// zjoltPhysicsSystemTryGetBodyNoLock makes for a single body.
//
// Same settings/filters/hits as the locked entry point of the same name
// minus the suffix. USE WITH GREAT CARE (Jolt's own phrase): nothing stops
// another thread mutating a body this reads while it reads it — calling one
// outside a step that already excludes that race IS a data race, not a
// slower path with the same answer.
//===----------------------------------------------------------------------===//

/// @see zjoltCastRayClosest.
ZJOLT_API ZJoltResult zjoltCastRayClosestNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltRayCastSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltRayCastHit *out_hit,
    bool *out_hit_any);

/// @see zjoltCastRayAll.
ZJOLT_API ZJoltResult zjoltCastRayAllNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltRayCastSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltRayCastHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// @see zjoltCastRayEach.
ZJOLT_API ZJoltResult zjoltCastRayEachNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltRayCastSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltRayCastHitFn on_hit, void *user);

/// @see zjoltCastShapeClosest.
ZJOLT_API ZJoltResult zjoltCastShapeClosestNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHit *out_hit, bool *out_hit_any);

/// @see zjoltCastShapeAll.
ZJOLT_API ZJoltResult zjoltCastShapeAllNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHit *out_hits, uint32_t capacity, uint32_t *out_count);

/// @see zjoltCastShapeEach.
ZJOLT_API ZJoltResult zjoltCastShapeEachNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHitFn on_hit, void *user);

/// @see zjoltCollideShapeClosest.
ZJOLT_API ZJoltResult zjoltCollideShapeClosestNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any);

/// @see zjoltCollideShapeAll.
ZJOLT_API ZJoltResult zjoltCollideShapeAllNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// @see zjoltCollideShapeEach.
ZJOLT_API ZJoltResult zjoltCollideShapeEachNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user);

/// @see zjoltCollideShapeWithInternalEdgeRemovalClosest.
ZJOLT_API ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalClosestNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any);

/// @see zjoltCollideShapeWithInternalEdgeRemovalAll.
ZJOLT_API ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalAllNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// @see zjoltCollideShapeWithInternalEdgeRemovalEach.
ZJOLT_API ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalEachNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user);

/// @see zjoltCollidePointAll.
ZJOLT_API ZJoltResult zjoltCollidePointAllNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *point,
    const ZJoltQueryFilters *filters, ZJoltCollidePointHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// @see zjoltCollidePointEach.
ZJOLT_API ZJoltResult zjoltCollidePointEachNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltRVec3 *point,
    const ZJoltQueryFilters *filters, ZJoltCollidePointHitFn on_hit,
    void *user);

//===----------------------------------------------------------------------===//
// Shape versus shape: two placed shapes, no physics system. Every hit
// reports ZJOLT_BODY_ID_INVALID. ZJOLT_RESULT_UNSUPPORTED for a pair with
// no convex side (one side must be convex all the way down, checked
// through any compound/decorated shape). ZJOLT_RESULT_INVALID_ARGUMENT
// for a scale a shape cannot take.
//===----------------------------------------------------------------------===//

/// The deepest overlap between two placed shapes, or nothing. `scale1`/
/// `scale2` may be NULL for (1,1,1). `position`/`rotation` are each shape's
/// own world placement (not centre of mass); the conversion happens here.
///
/// Contact points are RELATIVE TO `base_offset` (NULL for world space).
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

/// Like zjoltCollideShapeVsShapeAll, but AT MOST ONE hit per pair of LEAF
/// shapes that overlap, not one per pair of intersecting triangles. The hit
/// kept per leaf pair is the DEEPEST found, as with
/// zjoltCollideShapeClosest's "closest".
///
/// @see zjoltCollideShapeVsShapeAll for the two-call protocol and arguments.
ZJOLT_API ZJoltResult zjoltCollideShapeVsShapePerLeafAll(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// Sweeps `shape1` along `direction` and reports the nearest hit against
/// `shape2`, which does not move. The hit's centre of mass is the start
/// plus `fraction * direction`; nothing beyond it is reported. A sweep
/// starting already overlapping reports fraction 0 only if
/// `settings->back_face_mode_convex` allows back faces. @see
/// zjoltCollideShapeVsShapeClosest for scale, placement and base offset.
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

//===----------------------------------------------------------------------===//
// Predicting a contact's response, without running a step: standalone Jolt
// utilities over plain contact data, not a shape or system — the second one
// does not even take a body.
//
// zjoltEstimateCollisionResponse reads `body1`/`body2` through Jolt's own
// no-lock body interface — safe only where they are already guaranteed
// readable without a lock of this call's own, as inside ZJoltContactListener
// (zjolt_system.h), which this exists for. Elsewhere, without that
// guarantee, it is a data race. `combined_friction`/`combined_restitution`
// are the values ZJoltContactSettings carries from OnContactAdded;
// `num_iterations` of 1 computes no friction at all.
//===----------------------------------------------------------------------===//

/// The size of `ZJoltCollisionEstimationResult::contact_impulses`, and the
/// bound zjoltPruneContactPoints reduces toward from the other direction --
/// JPH::ContactPoints::Capacity, the width every one of Jolt's own contact
/// manifolds is built around.
#define ZJOLT_CONTACT_POINTS_CAPACITY 64

/// Input for zjoltEstimateCollisionResponse: a contact manifold reduced to
/// what its algorithm reads. `points_on_1`/`points_on_2` each hold
/// `num_points` entries, relative to `base_offset` (add it back for world
/// space). `num_points` must be at least 1 and at most
/// ZJOLT_CONTACT_POINTS_CAPACITY.
typedef struct ZJoltCollisionEstimationManifold {
  ZJoltRVec3 base_offset;
  /// Direction to move body 2 out of collision along the shortest path.
  ZJoltVec3 world_space_normal;
  uint32_t num_points;
  const ZJoltVec3 *points_on_1;
  const ZJoltVec3 *points_on_2;
} ZJoltCollisionEstimationManifold;

/// @see JPH::CollisionEstimationResult. Every velocity and impulse here is a
/// PREDICTION: accurate for two bodies colliding, not for three or more
/// resolving at the same time, because this knows nothing of any other
/// contact either body is in.
typedef struct ZJoltCollisionEstimationResult {
  ZJoltVec3 linear_velocity1;
  ZJoltVec3 angular_velocity1;
  ZJoltVec3 linear_velocity2;
  ZJoltVec3 angular_velocity2;
  /// Point at which friction was applied, relative to the manifold's
  /// base_offset -- the average of its contact points.
  ZJoltVec3 friction_point;
  /// Normalized tangent of the contact normal.
  ZJoltVec3 tangent1;
  /// Second normalized tangent; forms a basis with tangent1 and the
  /// manifold's world_space_normal.
  ZJoltVec3 tangent2;
  float friction_impulse1;
  float friction_impulse2;
  float angular_friction_impulse;
  /// Entries [0, num_contact_impulses) are valid, one per point in the
  /// manifold passed in, same order. Always equal to that manifold's
  /// num_points.
  uint32_t num_contact_impulses;
  float contact_impulses[ZJOLT_CONTACT_POINTS_CAPACITY];
} ZJoltCollisionEstimationResult;

/// Predicts impulses and post-collision velocities of a contact, for sizing
/// an impact response from inside a contact callback (see the note above).
///
/// ZJOLT_RESULT_BODY_NOT_FOUND if either id names no live body.
/// ZJOLT_RESULT_INVALID_ARGUMENT if manifold->num_points is 0 or exceeds
/// ZJOLT_CONTACT_POINTS_CAPACITY.
ZJOLT_API ZJoltResult zjoltEstimateCollisionResponse(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltCollisionEstimationManifold *manifold, float combined_friction,
    float combined_restitution, float min_velocity_for_restitution,
    uint32_t num_iterations, ZJoltCollisionEstimationResult *out_result);

//===----------------------------------------------------------------------===//
// zjoltPruneContactPoints: reduces a manifold to 4 or fewer points in place,
// the same reduction Jolt's own solver applies before a manifold becomes a
// contact constraint — for a host running identical contact processing.
//
// `*count` is read as the input count and written as the surviving count
// (always <= 4); refused (ZJOLT_RESULT_INVALID_ARGUMENT) if above
// ZJOLT_CONTACT_POINTS_CAPACITY or already at 4 or below.
//===----------------------------------------------------------------------===//

/// `points_on_1`/`points_on_2` are overwritten IN PLACE, relative to the
/// caller's existing offset (a choice of points, not a coordinate change).
///
/// `penetration_axis` must be normalizable: zero, non-finite or otherwise
/// degenerate is ZJOLT_RESULT_INVALID_ARGUMENT; near-unit vectors are
/// renormalized instead of refused.
ZJOLT_API ZJoltResult zjoltPruneContactPoints(
    const ZJoltVec3 *penetration_axis, ZJoltVec3 *points_on_1,
    ZJoltVec3 *points_on_2, uint32_t *count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_QUERY_H_
