//===----------------------------------------------------------------------===//
// zjolt — the physics system: layers, listeners and the step.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_SYSTEM_H_
#define ZJOLT_SYSTEM_H_

#include "zjolt_core.h"
#include "zjolt_query.h"
#include "zjolt_softbody.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Collision layers
//
// Jolt's three broad-phase questions cross as plain function-pointer tables, keeping C++ vtable layout out of the ABI.
// Copied by value at creation; `user` must outlive the system.
//===----------------------------------------------------------------------===//

typedef struct ZJoltBroadPhaseLayerInterface {
  /// How many broad-phase layers exist. Must be > 0 and constant.
  uint32_t (*num_broad_phase_layers)(void *user);
  /// Which broad-phase layer an object layer lives in. Must be < the count
  /// above, for every object layer the host uses.
  ZJoltBroadPhaseLayer (*broad_phase_layer_for_object_layer)(
      void *user, ZJoltObjectLayer layer);
  /// Optional; may be NULL. Only consulted by Jolt's profiler builds, once
  /// per layer at create. The returned pointer is cached and read again for
  /// as long as the system lives -- return a string literal or otherwise
  /// static storage, never a stack buffer or a slice this call frees.
  const char *(*broad_phase_layer_name)(void *user,
                                        ZJoltBroadPhaseLayer layer);
  void *user;
} ZJoltBroadPhaseLayerInterface;

typedef struct ZJoltObjectVsBroadPhaseLayerFilter {
  /// True if an object in `layer1` could collide with anything in `layer2`.
  bool (*should_collide)(void *user, ZJoltObjectLayer layer1,
                         ZJoltBroadPhaseLayer layer2);
  void *user;
} ZJoltObjectVsBroadPhaseLayerFilter;

typedef struct ZJoltObjectLayerPairFilter {
  /// True if two object layers collide. Must be symmetric.
  bool (*should_collide)(void *user, ZJoltObjectLayer layer1,
                         ZJoltObjectLayer layer2);
  void *user;
} ZJoltObjectLayerPairFilter;

//===----------------------------------------------------------------------===//
// Listeners
//
// IMPORTANT: contact callbacks run on Jolt's job threads, in parallel, inside zjoltPhysicsSystemStep — re-entrant, no calls back into the system.
// Activation callbacks run under the body manager's lock, serialised.
//===----------------------------------------------------------------------===//

/// One contact manifold, projected into plain data.
///
/// `points_on_1`/`points_on_2` point at storage owned by the call, valid
/// only until the callback returns, relative to `base_offset` (add it for
/// world space) — split out because absolute points would lose precision
/// in a double-precision world.
typedef struct ZJoltContactManifold {
  ZJoltRVec3 base_offset;
  /// Direction to move body 2 out of collision by the shortest path.
  ZJoltVec3 world_space_normal;
  /// Negative for a speculative contact that may not resolve into a response.
  float penetration_depth;
  ZJoltSubShapeId sub_shape_id1;
  ZJoltSubShapeId sub_shape_id2;
  uint32_t num_points;
  const ZJoltVec3 *points_on_1;
  const ZJoltVec3 *points_on_2;
} ZJoltContactManifold;

typedef struct ZJoltContactInfo {
  ZJoltBodyId body1;
  ZJoltBodyId body2;
  uint64_t user_data1;
  uint64_t user_data2;
  /// The live body, exactly as `OnContactAdded`/`OnContactPersisted` receive
  /// it in C++ — read velocity, shape, motion type or layer off it with
  /// the zjoltBody*Locked family. No lock of your own is needed or may be
  /// taken: this runs already excluded from what a lock protects, so a
  /// fresh lock request here is the deadlock this path avoids. Borrowed;
  /// valid only for the duration of the callback.
  const ZJoltBody *live_body1;
  const ZJoltBody *live_body2;
  ZJoltContactManifold manifold;
} ZJoltContactInfo;

/// Response properties a listener may adjust for one contact.
typedef struct ZJoltContactSettings {
  float combined_friction;
  float combined_restitution;
  /// 0 = infinite mass, 1 = as-is, 2 = half mass.
  float inv_mass_scale1;
  float inv_inertia_scale1;
  float inv_mass_scale2;
  float inv_inertia_scale2;
  /// Treat as a sensor contact: reported, but with no collision response.
  bool is_sensor;
  /// Surface velocity of body 2 relative to body 1 — a conveyor belt.
  ZJoltVec3 relative_linear_surface_velocity;
  ZJoltVec3 relative_angular_surface_velocity;
} ZJoltContactSettings;

/// What a validate callback is shown: the pair and the deepest point
/// found so far, before any manifold has been built.
///
/// `shape1_face`/`shape2_face` are the colliding face the narrow phase
/// collects; a convex shape or triangle reports its face, anything else
/// (a sphere) reports zero vertices. Relative to `base_offset`, borrowed.
typedef struct ZJoltContactValidateInfo {
  ZJoltBodyId body1;
  ZJoltBodyId body2;
  uint64_t user_data1;
  uint64_t user_data2;
  /// The live body, exactly as `OnContactValidate` receives it in C++ — see
  /// ZJoltContactInfo::live_body1 for what it is safe to do with one.
  const ZJoltBody *live_body1;
  const ZJoltBody *live_body2;
  ZJoltRVec3 base_offset;
  ZJoltVec3 contact_point_on_1;
  ZJoltVec3 contact_point_on_2;
  /// Direction to separate the shapes; its magnitude is meaningless.
  ZJoltVec3 penetration_axis;
  float penetration_depth;
  ZJoltSubShapeId sub_shape_id1;
  ZJoltSubShapeId sub_shape_id2;
  uint32_t num_shape1_face_vertices;
  uint32_t num_shape2_face_vertices;
  const ZJoltVec3 *shape1_face;
  const ZJoltVec3 *shape2_face;
} ZJoltContactValidateInfo;

/// Identifies the contact that was removed. The bodies may already be
/// gone, so this carries sub-shape ids rather than a manifold.
typedef struct ZJoltSubShapeIdPair {
  ZJoltBodyId body1;
  ZJoltSubShapeId sub_shape_id1;
  ZJoltBodyId body2;
  ZJoltSubShapeId sub_shape_id2;
} ZJoltSubShapeIdPair;

/// Every field may be NULL; a NULL callback takes Jolt's default behaviour.
typedef struct ZJoltContactListener {
  ZJoltValidateResult (*on_contact_validate)(
      void *user, const ZJoltContactValidateInfo *info);
  void (*on_contact_added)(void *user, const ZJoltContactInfo *info,
                           ZJoltContactSettings *settings);
  void (*on_contact_persisted)(void *user, const ZJoltContactInfo *info,
                               ZJoltContactSettings *settings);
  void (*on_contact_removed)(void *user, const ZJoltSubShapeIdPair *pair);
  void *user;
} ZJoltContactListener;

typedef struct ZJoltBodyActivationListener {
  void (*on_body_activated)(void *user, ZJoltBodyId body, uint64_t user_data);
  void (*on_body_deactivated)(void *user, ZJoltBodyId body, uint64_t user_data);
  void *user;
} ZJoltBodyActivationListener;

//===----------------------------------------------------------------------===//
// The step's scratch allocator
//
// Jolt asks for a TempAllocator at every step. This library's default is a fixed block (temp_allocator_size)
// falling back to malloc; supply ZJoltTempAllocator for a frame arena or a hard ceiling instead.
//===----------------------------------------------------------------------===//

typedef enum ZJoltTempAllocatorKind {
  /// A fixed block sized from ZJoltPhysicsSystemDesc::temp_allocator_size,
  /// falling back to malloc once it is exhausted. Jolt's own
  /// TempAllocatorImplWithMallocFallback strategy, and what a zeroed
  /// ZJoltPhysicsSystemDesc still gets — the byte-count knob keeps working
  /// unchanged for a host that does not care which of these it is.
  ZJOLT_TEMP_ALLOCATOR_KIND_MALLOC_FALLBACK = 0,
  /// The same fixed block with no fallback: an allocation past it aborts the
  /// process, the way Jolt's own TempAllocatorImpl does. Useful for proving a
  /// scratch budget rather than silently spilling into malloc mid-step.
  ZJOLT_TEMP_ALLOCATOR_KIND_FIXED = 1,
  /// ZJoltPhysicsSystemDesc::temp_allocator supplies the whole strategy.
  ZJOLT_TEMP_ALLOCATOR_KIND_HOST = 2,
} ZJoltTempAllocatorKind;

/// A host-supplied scratch allocator, shaped like ZJoltAllocator but scoped to
/// one physics system's step.
///
/// `allocate`/`free` are what Jolt calls during a step; `can_allocate`,
/// `get_size`/`get_usage` back the introspection Jolt's own concrete allocators
/// offer inconsistently — @see zjoltPhysicsSystemGetTempAllocatorStats.
typedef struct ZJoltTempAllocator {
  /// Returns at least `size` bytes. `size` is never 0. Jolt's stack discipline
  /// means the matching `free` sees the same size and frees in reverse order —
  /// see the section comment above.
  void *(*allocate)(void *user, uint32_t size);
  /// Frees a block from `allocate`. `address` is never NULL; `size` is the
  /// size that block was allocated with.
  void (*free)(void *user, void *address, uint32_t size);
  /// Optional; NULL answers true always. Whether `size` more bytes could be
  /// allocated without whatever this host's fallback path is.
  bool (*can_allocate)(void *user, uint32_t size);
  /// Optional; NULL reports 0. Total capacity of the preferred (non-fallback)
  /// path, for zjoltPhysicsSystemGetTempAllocatorStats.
  size_t (*get_size)(void *user);
  /// Optional; NULL reports 0. Bytes currently allocated, for the same.
  size_t (*get_usage)(void *user);
  void *user;
} ZJoltTempAllocator;

/// zjoltPhysicsSystemGetTempAllocatorStats's answer. All-zero for a HOST
/// allocator that supplied neither get_size nor get_usage.
typedef struct ZJoltTempAllocatorStats {
  /// Bytes in the fixed block before it falls back or aborts. 0 for a HOST
  /// allocator with no get_size.
  size_t capacity;
  /// Bytes allocated right now. Read this from a step listener or after the
  /// step returns — mid-step, from another thread, it is a moving target.
  size_t usage;
} ZJoltTempAllocatorStats;

//===----------------------------------------------------------------------===//
// Physics system
//===----------------------------------------------------------------------===//

typedef struct ZJoltPhysicsSystemDesc {
  /// Hard ceiling on bodies. Cannot grow, and cannot exceed 8388608 — Jolt
  /// packs a body's index and generation counter into one 32-bit id.
  uint32_t max_bodies;
  /// Body mutexes for parallel access; 0 picks Jolt's default.
  uint32_t num_body_mutexes;
  /// Ceiling on broad-phase pairs found in one step.
  uint32_t max_body_pairs;
  /// Ceiling on contact constraints solved in one step.
  uint32_t max_contact_constraints;
  /// Scratch arena for one step, in bytes. 0 picks 10 MiB. Used by
  /// temp_allocator_kind ZJOLT_TEMP_ALLOCATOR_KIND_MALLOC_FALLBACK (its
  /// fallback path aside) and ZJOLT_TEMP_ALLOCATOR_KIND_FIXED; ignored for
  /// ZJOLT_TEMP_ALLOCATOR_KIND_HOST.
  size_t temp_allocator_size;
  /// Which strategy backs the step's scratch allocator. Defaults (0) to
  /// ZJOLT_TEMP_ALLOCATOR_KIND_MALLOC_FALLBACK.
  ZJoltTempAllocatorKind temp_allocator_kind;
  /// Required, and read, only when temp_allocator_kind is
  /// ZJOLT_TEMP_ALLOCATOR_KIND_HOST. Copied by value at create, so it need not
  /// outlive the call — but `user` must outlive the system, the same promise
  /// ZJoltAllocator makes about its own `user`.
  const ZJoltTempAllocator *temp_allocator;
  ZJoltBroadPhaseLayerInterface broad_phase_layers;
  ZJoltObjectVsBroadPhaseLayerFilter object_vs_broad_phase_filter;
  ZJoltObjectLayerPairFilter object_layer_pair_filter;
} ZJoltPhysicsSystemDesc;

/// Fills `desc` with defaults: 10240 bodies, 65536 pairs and constraints, and
/// no filters (which must be supplied before create).
ZJOLT_API void zjoltPhysicsSystemDescInit(ZJoltPhysicsSystemDesc *desc);

ZJOLT_API ZJoltResult zjoltPhysicsSystemCreate(
    const ZJoltPhysicsSystemDesc *desc, ZJoltPhysicsSystem **out);

//===----------------------------------------------------------------------===//
// Reading back what create was given
//
// The three tables above are copied in at zjoltPhysicsSystemCreate; Jolt never hands them back, so a host that
// kept no copy of its own had no way to ask again. All-zero if `system` is NULL.
//===----------------------------------------------------------------------===//

ZJOLT_API void zjoltPhysicsSystemGetBroadPhaseLayerInterface(
    const ZJoltPhysicsSystem *system, ZJoltBroadPhaseLayerInterface *out);
ZJOLT_API void zjoltPhysicsSystemGetObjectVsBroadPhaseLayerFilter(
    const ZJoltPhysicsSystem *system, ZJoltObjectVsBroadPhaseLayerFilter *out);
ZJOLT_API void zjoltPhysicsSystemGetObjectLayerPairFilter(
    const ZJoltPhysicsSystem *system, ZJoltObjectLayerPairFilter *out);

/// Destroys the system and every body still in it. Shapes are released,
/// not destroyed — a shape outlives the system if the host still holds a
/// reference.
///
/// Every ZJoltCharacter created against this system must be destroyed
/// FIRST: it holds a pointer to its system, and outliving it dangles.
ZJOLT_API void zjoltPhysicsSystemDestroy(ZJoltPhysicsSystem *system);

/// All-zero if `system` is NULL. @see ZJoltTempAllocatorStats.
ZJOLT_API void zjoltPhysicsSystemGetTempAllocatorStats(
    const ZJoltPhysicsSystem *system, ZJoltTempAllocatorStats *out);

/// False if `system` is NULL. @see ZJoltTempAllocator::can_allocate.
ZJOLT_API bool zjoltPhysicsSystemTempAllocatorCanAllocate(
    const ZJoltPhysicsSystem *system, uint32_t size);

ZJOLT_API void zjoltPhysicsSystemSetGravity(ZJoltPhysicsSystem *system,
                                            const ZJoltVec3 *gravity);
ZJOLT_API void zjoltPhysicsSystemGetGravity(const ZJoltPhysicsSystem *system,
                                            ZJoltVec3 *out);

/// Rebuilds the broad phase for the bodies added so far. Worth calling once
/// after bulk-loading static geometry, and never per frame.
ZJOLT_API void zjoltPhysicsSystemOptimizeBroadPhase(ZJoltPhysicsSystem *system);

ZJOLT_API uint32_t zjoltPhysicsSystemGetNumBodies(
    const ZJoltPhysicsSystem *system);
ZJOLT_API uint32_t zjoltPhysicsSystemGetNumActiveBodies(
    const ZJoltPhysicsSystem *system);

/// NULL clears the listener. The struct is copied, so it need not outlive the
/// call — but its `user` pointer must outlive the system.
///
/// Returns a result rather than nothing because a listener that failed to
/// install is a world that silently stops reporting collisions.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetContactListener(
    ZJoltPhysicsSystem *system, const ZJoltContactListener *listener);
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetBodyActivationListener(
    ZJoltPhysicsSystem *system, const ZJoltBodyActivationListener *listener);

/// All-zero (every field NULL) if no listener is installed, or if `system`
/// is NULL.
ZJOLT_API void zjoltPhysicsSystemGetContactListener(
    const ZJoltPhysicsSystem *system, ZJoltContactListener *out);
/// All-zero if no listener is installed, or if `system` is NULL.
ZJOLT_API void zjoltPhysicsSystemGetBodyActivationListener(
    const ZJoltPhysicsSystem *system, ZJoltBodyActivationListener *out);

/// The listener zjoltSoftBodySetContactListener (zjolt_softbody.h) installed,
/// or all-zero if none is, or if `system` is NULL. Declared here rather than
/// in zjolt_softbody.h because the listener is stored on ZJoltPhysicsSystem
/// itself, alongside the other two PhysicsSystem-level listener getters above.
ZJOLT_API void zjoltPhysicsSystemGetSoftBodyContactListener(
    const ZJoltPhysicsSystem *system, ZJoltSoftBodyContactListener *out);

//===----------------------------------------------------------------------===//
// Simulation shape filter
//
// PhysicsSystem::SetSimShapeFilter is consulted by the step itself, on Jolt's job threads, to decide whether two
// shapes may collide — distinct from ZJoltShapeFilter in zjolt_query.h, which only applies to an explicit query.
//===----------------------------------------------------------------------===//

typedef struct ZJoltSimShapeFilter {
  /// True if the two shapes may collide. `sub_shape_id1`/`sub_shape_id2` lead
  /// from the whole body's own shape down to `shape1`/`shape2`; each is
  /// ZJOLT_SUB_SHAPE_ID_EMPTY for a body whose shape is not compound. Read
  /// only — must not call back into the system, and the no-unwinding rule
  /// that applies to every other callback in this header applies here too.
  /// NULL accepts everything, Jolt's own default.
  bool (*should_collide)(void *user, ZJoltBodyId body1,
                         const ZJoltShape *shape1,
                         ZJoltSubShapeId sub_shape_id1, ZJoltBodyId body2,
                         const ZJoltShape *shape2,
                         ZJoltSubShapeId sub_shape_id2);
  void *user;
} ZJoltSimShapeFilter;

/// NULL restores Jolt's default (every shape pair may collide). The struct is
/// copied, so it need not outlive the call — but `user` must outlive the
/// system.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetSimShapeFilter(
    ZJoltPhysicsSystem *system, const ZJoltSimShapeFilter *filter);

/// All-zero if none is installed, or if `system` is NULL.
ZJOLT_API void zjoltPhysicsSystemGetSimShapeFilter(
    const ZJoltPhysicsSystem *system, ZJoltSimShapeFilter *out);

//===----------------------------------------------------------------------===//
// Body-vs-body narrow-phase collide hook
//
// PhysicsSystem::SetSimCollideBodyVsBody replaces the shape-vs-shape collide
// the step itself uses for every candidate body pair -- one-way platforms,
// per-pair overrides. Called from Jolt's job threads during the step, once
// per candidate pair; no calls back into the system, nothing may unwind out,
// same rule as a contact listener. NULL restores Jolt's own default
// (CollisionDispatch, with enhanced internal edge removal when either body
// opts in).
//===----------------------------------------------------------------------===//

/// One shape-pair hit, reported through zjoltSimCollideAddHit. Carries both
/// sub-shape ids, unlike ZJoltCollideShapeHit: either body's own shape may be
/// compound here, not just the one found by a query. Relative to the same
/// base as `center_of_mass_transform1`/`center_of_mass_transform2`, single
/// precision.
typedef struct ZJoltSimCollideHit {
  ZJoltSubShapeId sub_shape_id1;
  ZJoltSubShapeId sub_shape_id2;
  ZJoltVec3 contact_point_on_1;
  ZJoltVec3 contact_point_on_2;
  /// Direction to move body 2 out of collision along the shortest path;
  /// magnitude is meaningless.
  ZJoltVec3 penetration_axis;
  float penetration_depth;
} ZJoltSimCollideHit;

/// The in-flight result collector a ZJoltSimCollideFn was given -- opaque,
/// borrowed for the call only. Feed it to zjoltSimCollideAddHit and/or
/// zjoltSimCollideDefault.
typedef struct ZJoltSimCollideCollector ZJoltSimCollideCollector;

/// The shape filter active for this pair -- opaque, borrowed for the call
/// only, never NULL here. Forward it to zjoltSimCollideDefault to preserve
/// whatever zjoltPhysicsSystemSetSimShapeFilter installed; a callback that
/// neither reads it nor delegates loses no filtering of its own.
typedef struct ZJoltSimCollideShapeFilter ZJoltSimCollideShapeFilter;

/// Replaces Jolt's narrow-phase collide for one candidate body pair. The two
/// transforms are relative to a shared base, not world space -- see
/// ZJoltSimCollideHit. `live_body1`/`live_body2`, `settings`, `shape_filter`
/// and `collector` are borrowed for the call only; `settings` may be edited
/// before adding hits (zjoltSimCollideAddHit) or delegating
/// (zjoltSimCollideDefault) to Jolt's own collision for this pair, unchanged.
typedef void (*ZJoltSimCollideFn)(
    void *user, const ZJoltBody *live_body1, const ZJoltBody *live_body2,
    const ZJoltMat44 *center_of_mass_transform1,
    const ZJoltMat44 *center_of_mass_transform2,
    ZJoltCollideShapeSettings *settings,
    const ZJoltSimCollideShapeFilter *shape_filter,
    ZJoltSimCollideCollector *collector);

typedef struct ZJoltSimCollideBodyVsBody {
  ZJoltSimCollideFn collide;
  void *user;
} ZJoltSimCollideBodyVsBody;

/// NULL, or a NULL `collide`, restores Jolt's default. The struct is copied,
/// so it need not outlive the call -- but `user` must outlive the system.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetSimCollideBodyVsBody(
    ZJoltPhysicsSystem *system, const ZJoltSimCollideBodyVsBody *hook);

/// All-zero if none is installed (Jolt's default is active), or if `system`
/// is NULL.
ZJOLT_API void zjoltPhysicsSystemGetSimCollideBodyVsBody(
    const ZJoltPhysicsSystem *system, ZJoltSimCollideBodyVsBody *out);

/// Reports one hit from inside a ZJoltSimCollideFn. May be called any number
/// of times, including zero -- a suppressed pair adds none. A pair a contact
/// validate listener has already rejected outright silently accepts no
/// further hit; call this before, not after, zjoltSimCollideDefault if you
/// need both.
ZJOLT_API void zjoltSimCollideAddHit(ZJoltSimCollideCollector *collector,
                                     ZJoltBodyId body2,
                                     const ZJoltSimCollideHit *hit);

/// Runs Jolt's own default body-vs-body collide for this pair, reporting
/// through the same `collector` a ZJoltSimCollideFn was given. NULL
/// `settings` takes Jolt's own defaults, same as every zjoltCollideShape*
/// query.
ZJOLT_API void zjoltSimCollideDefault(
    const ZJoltBody *live_body1, const ZJoltBody *live_body2,
    const ZJoltMat44 *center_of_mass_transform1,
    const ZJoltMat44 *center_of_mass_transform2,
    const ZJoltCollideShapeSettings *settings,
    const ZJoltSimCollideShapeFilter *shape_filter,
    ZJoltSimCollideCollector *collector);

/// Advances the simulation by `delta_time`, split into `collision_steps`
/// sub-steps (1 is normal; raise for fast bodies, not by shortening the frame).
///
/// `out_error` receives a ZJoltUpdateError mask (may be NULL); non-zero means a
/// limit in ZJoltPhysicsSystemDesc is too low, not that the step failed.
ZJOLT_API ZJoltResult zjoltPhysicsSystemStep(ZJoltPhysicsSystem *system,
                                             float delta_time,
                                             int32_t collision_steps,
                                             ZJoltJobSystem *job_system,
                                             uint32_t *out_error);

/// The ceiling this system was created with, which cannot grow. 0 if `system`
/// is NULL.
ZJOLT_API uint32_t zjoltPhysicsSystemGetMaxBodies(
    const ZJoltPhysicsSystem *system);

/// A census of the bodies in a system, broken down by motion type.
typedef struct ZJoltBodyStats {
  uint32_t num_bodies;
  /// The ceiling this system was created with; same as
  /// zjoltPhysicsSystemGetMaxBodies.
  uint32_t max_bodies;
  uint32_t num_bodies_static;
  uint32_t num_bodies_dynamic;
  uint32_t num_active_bodies_dynamic;
  uint32_t num_bodies_kinematic;
  uint32_t num_active_bodies_kinematic;
  uint32_t num_soft_bodies;
  uint32_t num_active_soft_bodies;
} ZJoltBodyStats;

/// Slow: iterates every body in the system. Meant for an occasional debug
/// overlay, not a per-frame call. All-zero if `system` is NULL.
ZJOLT_API void zjoltPhysicsSystemGetBodyStats(const ZJoltPhysicsSystem *system,
                                              ZJoltBodyStats *out);

/// Traces the broad phase's own accumulated per-layer query stats to the
/// TTY as CSV. Diagnostic output only, nothing returned — @see
/// zjoltBodyGetSimulationStatsLocked for numbers a caller can read
/// programmatically.
///
/// ZJOLT_RESULT_UNSUPPORTED unless built with -Dtrack_broadphase_stats.
ZJOLT_API ZJoltResult zjoltPhysicsSystemReportBroadphaseStats(
    ZJoltPhysicsSystem *system);

/// Traces the narrow phase's own accumulated per-shape-pair timing stats to
/// the TTY as CSV. Process-wide, not per system: Jolt keeps one set of
/// tables regardless of how many physics systems exist. Diagnostic output
/// only, nothing returned.
///
/// ZJOLT_RESULT_UNSUPPORTED unless built with -Dtrack_narrowphase_stats.
ZJOLT_API ZJoltResult zjoltReportNarrowPhaseStats(void);

/// True if the two bodies were touching during the LAST step.
///
/// This reads the contact cache, so it answers a question about the step that
/// has already run rather than about where the bodies are now — which is what
/// a gameplay rule usually wants, and is far cheaper than an overlap query.
/// Order does not matter. Do not call it during a step.
ZJOLT_API bool zjoltPhysicsSystemWereBodiesInContact(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2);

//===----------------------------------------------------------------------===//
// The unlocked path
//
// Every zjoltBody* accessor locks unless it is a *Locked one, wrong inside a step where a listener already holds
// the relevant locks. This is the unlocked way to get a ZJoltBody* for that case — "use with great care".
//===----------------------------------------------------------------------===//

/// The body named by `body`, with NO lock taken — @see the section comment
/// above. NULL if `body` does not name a live body in `system`, exactly as
/// Jolt's own BodyLockInterfaceNoLock::TryGetBody answers "not found" rather
/// than failing.
ZJOLT_API const ZJoltBody *zjoltPhysicsSystemTryGetBodyNoLock(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// A live, zero-copy view of the same array zjoltPhysicsSystemGetActiveBodies
/// copies from. NOT thread safe, can change at any moment (Jolt's phrase).
///
/// `*out_ids` is borrowed and invalidated by the next add/remove/
/// activate/deactivate on `system` — read `*out_count` entries before
/// anything else. Both cleared if `system` is NULL.
ZJOLT_API void zjoltPhysicsSystemGetActiveBodiesUnsafe(
    const ZJoltPhysicsSystem *system, const ZJoltBodyId **out_ids,
    uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Simulation settings
//
// Jolt's PhysicsSettings verbatim, one flat struct with no per-field setter (several fields interact). Takes effect
// from the next step; not saved by zjoltPhysicsSystemSaveState (configuration, not state).
//===----------------------------------------------------------------------===//

typedef struct ZJoltPhysicsSettings {
  /// Body pairs that may be in flight at once. Lower uses less memory and
  /// stalls the narrow phase when a worker runs out of pairs to chew on. Must
  /// be at least 1: it is a buffer length.
  int32_t max_in_flight_body_pairs;
  /// How many step listeners one job notifies before taking the next batch.
  /// Must be at least 1 — a zero batch never advances the read index, and the
  /// step never ends.
  int32_t step_listeners_batch_size;
  /// How many of those batches justify spawning another job. Must be at least
  /// 1: it is a divisor. Raise it, or set it to INT32_MAX, to keep every
  /// listener on one job when the listeners are not safe to run concurrently.
  int32_t step_listener_batches_per_job;
  /// Baumgarte stabilisation: the fraction of a position error corrected in
  /// one step. 0 corrects none of it, 1 corrects all of it and adds energy.
  float baumgarte;
  /// Radius around a body within which speculative contacts are created (m).
  /// Raising it catches faster bodies and starts producing ghost collisions,
  /// because a speculative contact is placed from the closest points at
  /// detection time and those need not be the closest points at impact.
  float speculative_contact_distance;
  /// How far bodies may sink into each other before the solver objects (m).
  /// Zero is not the right answer: a little slop is what stops a resting stack
  /// from vibrating.
  float penetration_slop;
  /// Fraction of its inner radius a body must move in one step before the
  /// ZJOLT_MOTION_QUALITY_LINEAR_CAST quality actually sweeps.
  float linear_cast_threshold;
  /// Fraction of its inner radius a linear-cast body is still allowed to
  /// penetrate another.
  float linear_cast_max_penetration;
  /// How far apart two points may be and still count as lying on the same
  /// plane when a contact manifold is built (m).
  float manifold_tolerance;
  /// The most penetration one position-solver iteration will correct (m). A
  /// cap, so a deeply overlapping pair is eased apart over several steps
  /// instead of being launched.
  float max_penetration_distance;
  /// How far a body pair may move and still reuse the previous step's
  /// collision result (m^2). @see use_body_pair_contact_cache
  float body_pair_cache_max_delta_position_sq;
  /// The rotation half of that same test, stored as cos(max angle / 2). Jolt's
  /// default is two degrees.
  float body_pair_cache_cos_max_delta_rotation_div2;
  /// Largest angle between the normals of two sub-shape manifolds that still
  /// lets them be merged into one, as a cosine.
  float contact_normal_cos_max_delta_rotation;
  /// How far a contact point may move between steps and still carry its
  /// accumulated impulse into the next warm start (m^2).
  float contact_point_preserve_lambda_max_dist_sq;
  /// How close two vertices must be to count as the same one when the
  /// internal-edge remover decides whether two triangles share an edge (m^2).
  /// @see ZJoltBodyDesc::enhanced_internal_edge_removal
  float internal_edge_removal_vertex_tolerance_sq;
  /// Velocity solver iterations. Must be at least 2 for friction to work at
  /// all: friction is applied from the previous iteration's non-penetration
  /// impulse, and with one iteration there is no previous one. This is a
  /// floor rather than a count — a constraint may ask for more.
  uint32_t num_velocity_steps;
  /// Position solver iterations. Zero is legal and means penetration is
  /// resolved by the velocity solver alone.
  uint32_t num_position_steps;
  /// Below this closing speed a collision is inelastic whatever the bodies'
  /// restitution says (m/s). Must not be negative; it is what lets a bouncy
  /// ball eventually stop bouncing.
  float min_velocity_for_restitution;
  /// How long a body must hold still before it may sleep (s).
  float time_before_sleep;
  /// How still "still" is (m/s). Jolt tracks three points — the centre of mass
  /// and the two bounding-box face centres furthest from it — and all three
  /// must be slower than this. Must not be negative.
  float point_velocity_sleep_threshold;
  /// Turning this off makes the simulation faster and its results no longer
  /// reproducible. zjoltPhysicsSystemSaveState, and any replay built on it,
  /// assume it is on.
  bool deterministic_simulation;
  /// Warm start contacts and constraints from the previous step's impulses.
  bool constraint_warm_start;
  /// Skip the narrow phase for a body pair whose relative transform has not
  /// moved past the two tolerances above. Worth knowing when a body's
  /// collision group changes: the cached answer outlives the change until the
  /// pair moves. @see zjoltBodyInvalidateContactCache
  bool use_body_pair_contact_cache;
  /// Merge manifolds with similar normals into one.
  bool use_manifold_reduction;
  /// Split large islands into batches that can be solved in parallel.
  bool use_large_island_splitter;
  /// Whether bodies may sleep at all. A body's own allow_sleeping is ANDed
  /// with this.
  bool allow_sleeping;
  /// Collide only with the active edges of a triangle mesh. On is correct; off
  /// collides with every edge, which is a debugging aid whose whole point is
  /// that it produces the ghost collisions active edges exist to prevent.
  bool check_active_edges;
} ZJoltPhysicsSettings;

/// Fills `settings` with Jolt's own defaults.
///
/// They are read out of a default-constructed PhysicsSettings rather than
/// transcribed, so they cannot drift from the vendored library.
ZJOLT_API void zjoltPhysicsSettingsInit(ZJoltPhysicsSettings *settings);

/// Leaves `out` all-zero if `system` is NULL, which is not a valid settings
/// struct — check the handle, do not check the result.
ZJOLT_API void zjoltPhysicsSystemGetSettings(const ZJoltPhysicsSystem *system,
                                             ZJoltPhysicsSettings *out);

/// Applies every field at once, from the next step onwards.
///
/// Rejected rather than applied: a non-positive batch size, batches per
/// job, or in-flight body pair count, a zero velocity iteration count,
/// or a negative sleep threshold — Jolt asserts none of those, and the
/// first two are a divisor/loop stride the step would hang or crash on.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetSettings(
    ZJoltPhysicsSystem *system, const ZJoltPhysicsSettings *settings);

//===----------------------------------------------------------------------===//
// Combine callbacks
//
// What friction/restitution a contact gets when bodies disagree (Jolt's defaults: sqrt(f1*f2), max(r1,r2)). Run on
// job threads during a step, once per contact — re-entrant; nothing may unwind out of one.
//===----------------------------------------------------------------------===//

/// The pair being combined, and the two values to combine.
///
/// Both values are read from the bodies before the call, so the callback needs
/// nothing from the system — which matters, because it must not touch it.
typedef struct ZJoltCombineInfo {
  ZJoltBodyId body1;
  ZJoltSubShapeId sub_shape_id1;
  uint64_t user_data1;
  /// Body 1's own friction, or its restitution, depending on which callback
  /// this is.
  float value1;
  ZJoltBodyId body2;
  ZJoltSubShapeId sub_shape_id2;
  uint64_t user_data2;
  float value2;
} ZJoltCombineInfo;

/// Returns the combined value for one contact.
typedef float (*ZJoltCombineFn)(void *user, const ZJoltCombineInfo *info);

/// NULL restores Jolt's default, the geometric mean sqrt(f1 * f2).
///
/// Jolt's own hook takes no user parameter, so `user` is carried in a
/// fixed table of ZJOLT_COMBINE_SLOT_COUNT slots, one per system with a
/// callback installed; past that, ZJOLT_RESULT_OUT_OF_MEMORY rather than
/// silently not installing.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetCombineFriction(
    ZJoltPhysicsSystem *system, ZJoltCombineFn combine, void *user);

/// NULL restores Jolt's default, max(r1, r2). @see
/// zjoltPhysicsSystemSetCombineFriction for the slot rule.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetCombineRestitution(
    ZJoltPhysicsSystem *system, ZJoltCombineFn combine, void *user);

/// Reads back the callback and user pointer
/// zjoltPhysicsSystemSetCombineRestitution installed. `*out_combine` and
/// `*out_user` are both NULL if none is installed (Jolt's default, max(r1, r2),
/// is active) or if `system` is NULL.
ZJOLT_API void zjoltPhysicsSystemGetCombineRestitution(
    const ZJoltPhysicsSystem *system, ZJoltCombineFn *out_combine,
    void **out_user);

/// How many physics systems may have a combine callback installed at once.
#define ZJOLT_COMBINE_SLOT_COUNT 64

//===----------------------------------------------------------------------===//
// Step listeners
//
// Called once per collision step, before collision detection, with every body/constraint mutex held: read/write
// bodies, but do not add or remove them. Nothing may unwind out of one, same as ZJoltCombineFn.
//===----------------------------------------------------------------------===//

typedef struct ZJoltStepListenerContext {
  /// Delta time of THIS collision step: the frame's delta time divided by the
  /// collision step count.
  float delta_time;
  bool is_first_step;
  bool is_last_step;
} ZJoltStepListenerContext;

typedef void (*ZJoltStepListenerFn)(void *user,
                                    const ZJoltStepListenerContext *context);

/// Adds a listener and yields a handle that identifies it for removal.
/// The handle belongs to the system: destroying the system destroys any
/// attached listener. `user` must outlive the listener. The same
/// function pointer may be added more than once; each add yields its
/// own handle, removed separately — the handle is the identity here,
/// not the pointer.
ZJOLT_API ZJoltResult zjoltPhysicsSystemAddStepListener(
    ZJoltPhysicsSystem *system, ZJoltStepListenerFn listener, void *user,
    ZJoltStepListener **out);

/// Removes and destroys a listener. Reports ZJOLT_RESULT_INVALID_ARGUMENT if
/// the handle does not belong to this system, where Jolt would assert.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRemoveStepListener(
    ZJoltPhysicsSystem *system, ZJoltStepListener *listener);

//===----------------------------------------------------------------------===//
// Islands
//
// Groups linked bodies, constraints and contacts for the solver. A standalone
// instance a host builds and drives itself — PhysicsSystem keeps its own
// privately, with no accessor, so there is no way to reach that one.
//===----------------------------------------------------------------------===//

typedef struct ZJoltIslandBuilder ZJoltIslandBuilder;

ZJOLT_API ZJoltResult zjoltIslandBuilderCreate(ZJoltIslandBuilder **out);

/// Frees the builder. Islands still built (Finalize, or a Prepare*, without
/// a matching ResetIslands) are freed with it, through the temp allocator
/// most recently given to Prepare*/Finalize — @see the note on those.
ZJOLT_API void zjoltIslandBuilderDestroy(ZJoltIslandBuilder *builder);

/// Allocates for up to `max_active_bodies` bodies, identified in every call
/// below by their index into the host's own active-body list. Required
/// before any other call; ZJOLT_RESULT_INVALID_ARGUMENT if already called.
ZJOLT_API ZJoltResult zjoltIslandBuilderInit(ZJoltIslandBuilder *builder,
                                             uint32_t max_active_bodies);

/// Merges the islands `first` and `second` belong to. Silently ignores an
/// index at or past `max_active_bodies` (Jolt's own behaviour: a static
/// body has no island to link into). ZJOLT_RESULT_INVALID_ARGUMENT if
/// zjoltIslandBuilderInit has not run.
ZJOLT_API ZJoltResult zjoltIslandBuilderLinkBodies(ZJoltIslandBuilder *builder,
                                                   uint32_t first,
                                                   uint32_t second);

/// Allocates the contact-link table. Required before
/// zjoltIslandBuilderLinkContact, once per Init/ResetIslands cycle.
/// `temp_allocator`'s `user` context must stay valid until the builder is
/// destroyed or a later Prepare*/Finalize call supersedes it: Destroy frees
/// through the most recent one if the host never calls ResetIslands.
ZJOLT_API ZJoltResult zjoltIslandBuilderPrepareContactConstraints(
    ZJoltIslandBuilder *builder, uint32_t max_contacts,
    const ZJoltTempAllocator *temp_allocator);

/// Allocates the constraint-link table. Required before
/// zjoltIslandBuilderLinkConstraint, once per Init/ResetIslands cycle.
/// Same `temp_allocator` lifetime note as PrepareContactConstraints.
ZJOLT_API ZJoltResult zjoltIslandBuilderPrepareNonContactConstraints(
    ZJoltIslandBuilder *builder, uint32_t num_constraints,
    const ZJoltTempAllocator *temp_allocator);

/// Records that constraint `constraint_index` (below the count given to
/// PrepareNonContactConstraints) touches the active body at
/// `index_in_active_body_list`.
ZJOLT_API ZJoltResult zjoltIslandBuilderLinkConstraint(
    ZJoltIslandBuilder *builder, uint32_t constraint_index,
    uint32_t index_in_active_body_list);

/// As LinkConstraint, for contact `contact_index` (below the count given to
/// PrepareContactConstraints).
ZJOLT_API ZJoltResult zjoltIslandBuilderLinkContact(
    ZJoltIslandBuilder *builder, uint32_t contact_index,
    uint32_t index_in_active_body_list);

/// Closes linking and sorts islands largest-first. `active_bodies` is
/// `num_active_bodies` ids, in the same order LinkBodies/LinkConstraint/
/// LinkContact indexed into; `num_contacts` must match what
/// PrepareContactConstraints (if called) was given. Required before every
/// getter below. Same `temp_allocator` lifetime note as
/// PrepareContactConstraints.
ZJOLT_API ZJoltResult zjoltIslandBuilderFinalize(
    ZJoltIslandBuilder *builder, const ZJoltBodyId *active_bodies,
    uint32_t num_active_bodies, uint32_t num_contacts,
    const ZJoltTempAllocator *temp_allocator);

/// 0 if `builder` is NULL or has not been finalized.
ZJOLT_API uint32_t zjoltIslandBuilderGetNumIslands(
    const ZJoltIslandBuilder *builder);

/// Two-call protocol, as zjoltPhysicsSystemGetBodies: `*out_count` always
/// receives island `island_index`'s body count, and a NULL `out_bodies`
/// with any capacity is a size query. ZJOLT_RESULT_INVALID_ARGUMENT if
/// `island_index` is at or past zjoltIslandBuilderGetNumIslands.
ZJOLT_API ZJoltResult zjoltIslandBuilderGetBodiesInIsland(
    const ZJoltIslandBuilder *builder, uint32_t island_index,
    ZJoltBodyId *out_bodies, uint32_t capacity, uint32_t *out_count);

/// As GetBodiesInIsland, for the constraint indices LinkConstraint recorded
/// against this island. `*out_count` is 0 when PrepareNonContactConstraints
/// was never called.
ZJOLT_API ZJoltResult zjoltIslandBuilderGetConstraintsInIsland(
    const ZJoltIslandBuilder *builder, uint32_t island_index,
    uint32_t *out_constraints, uint32_t capacity, uint32_t *out_count);

/// As GetBodiesInIsland, for the contact indices LinkContact recorded
/// against this island. `*out_count` is 0 when PrepareContactConstraints
/// was never called.
ZJOLT_API ZJoltResult zjoltIslandBuilderGetContactsInIsland(
    const ZJoltIslandBuilder *builder, uint32_t island_index,
    uint32_t *out_contacts, uint32_t capacity, uint32_t *out_count);

ZJOLT_API ZJoltResult zjoltIslandBuilderGetNumPositionSteps(
    const ZJoltIslandBuilder *builder, uint32_t island_index,
    uint32_t *out_num_position_steps);

/// Per-island solver timings, as IslandBuilder::GetIslandStats reports them.
/// Ticks are the platform's own cycle counter, the unit Jolt records; they
/// are comparable to each other, not to wall-clock, and reset every step.
typedef struct ZJoltIslandStats {
  uint64_t velocity_constraint_ticks;
  uint64_t position_constraint_ticks;
  uint64_t update_bounds_ticks;
  uint8_t num_velocity_steps;
  uint8_t num_position_steps;
  bool is_large_island;
} ZJoltIslandStats;

/// ZJOLT_RESULT_UNSUPPORTED unless built with -Dtrack_simulation_stats,
/// which is what fills these in. `num_position_steps` is tracked separately
/// from zjoltIslandBuilderGetNumPositionSteps: Jolt leaves that one unset
/// for a large island, or for one with no constraints.
ZJOLT_API ZJoltResult zjoltIslandBuilderGetStats(
    const ZJoltIslandBuilder *builder, uint32_t island_index,
    ZJoltIslandStats *out_stats);

/// `num_position_steps` must be below 256, where Jolt asserts.
ZJOLT_API ZJoltResult zjoltIslandBuilderSetNumPositionSteps(
    ZJoltIslandBuilder *builder, uint32_t island_index,
    uint32_t num_position_steps);

/// Frees everything Finalize (and Prepare*, if called) allocated, so the
/// builder is ready for the next Init-less cycle: PrepareContactConstraints,
/// link calls, Finalize. `temp_allocator` need not outlive the call.
ZJOLT_API ZJoltResult zjoltIslandBuilderResetIslands(
    ZJoltIslandBuilder *builder, const ZJoltTempAllocator *temp_allocator);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SYSTEM_H_
