//===----------------------------------------------------------------------===//
// zjolt — the physics system: layers, listeners and the step.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_SYSTEM_H_
#define ZJOLT_SYSTEM_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Collision layers
//
// Jolt asks the host three questions during the broad phase, through three
// abstract C++ classes. Here they are plain function-pointer tables: the C++
// side implements the real interfaces and forwards. That keeps C++ vtable
// layout out of the ABI entirely, which is what makes this header portable
// across the SysV and Microsoft conventions without per-ABI branches on the
// consumer's side.
//
// All three are copied by value into the system at creation, so the structs
// themselves need not outlive the call — but `user` must outlive the system.
//===----------------------------------------------------------------------===//

typedef struct ZJoltBroadPhaseLayerInterface {
  /// How many broad-phase layers exist. Must be > 0 and constant.
  uint32_t (*num_broad_phase_layers)(void *user);
  /// Which broad-phase layer an object layer lives in. Must be < the count
  /// above, for every object layer the host uses.
  ZJoltBroadPhaseLayer (*broad_phase_layer_for_object_layer)(
      void *user, ZJoltObjectLayer layer);
  /// Optional; may be NULL. Only consulted by Jolt's profiler builds.
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
// IMPORTANT: contact callbacks run on Jolt's job threads, in parallel, inside
// zjoltPhysicsSystemStep. They must be re-entrant and must not call back into
// the physics system. The usual shape is to append to a per-thread queue and
// drain it after the step returns.
//
// Activation callbacks are also raised during the step, but under the body
// manager's lock, so they are serialised.
//===----------------------------------------------------------------------===//

/// One contact manifold, projected into plain data.
///
/// `points_on_1` and `points_on_2` point at storage owned by the call and are
/// valid only until the callback returns. They are relative to `base_offset`;
/// add it to get world space. In a float build `base_offset` is exact, but the
/// split exists because in a double-precision world absolute contact points
/// would lose precision as floats.
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

/// What a validate callback is shown: the pair and the deepest point found so
/// far, before any manifold has been built.
typedef struct ZJoltContactValidateInfo {
  ZJoltBodyId body1;
  ZJoltBodyId body2;
  uint64_t user_data1;
  uint64_t user_data2;
  ZJoltRVec3 base_offset;
  ZJoltVec3 contact_point_on_1;
  ZJoltVec3 contact_point_on_2;
  /// Direction to separate the shapes; its magnitude is meaningless.
  ZJoltVec3 penetration_axis;
  float penetration_depth;
  ZJoltSubShapeId sub_shape_id1;
  ZJoltSubShapeId sub_shape_id2;
} ZJoltContactValidateInfo;

/// Identifies the contact that was removed. The bodies may already be gone,
/// which is why this carries sub-shape ids and not a manifold.
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
  /// Scratch arena for one step, in bytes. 0 picks 10 MiB. A step that needs
  /// more falls back to the host allocator rather than failing.
  size_t temp_allocator_size;
  ZJoltBroadPhaseLayerInterface broad_phase_layers;
  ZJoltObjectVsBroadPhaseLayerFilter object_vs_broad_phase_filter;
  ZJoltObjectLayerPairFilter object_layer_pair_filter;
} ZJoltPhysicsSystemDesc;

/// Fills `desc` with defaults: 10240 bodies, 65536 pairs and constraints, and
/// no filters (which must be supplied before create).
ZJOLT_API void zjoltPhysicsSystemDescInit(ZJoltPhysicsSystemDesc *desc);

ZJOLT_API ZJoltResult zjoltPhysicsSystemCreate(
    const ZJoltPhysicsSystemDesc *desc, ZJoltPhysicsSystem **out);

/// Destroys the system and every body still in it. Shapes are released, not
/// destroyed — a shape outlives the system if the host still holds a
/// reference.
///
/// Every ZJoltCharacter created against this system must be destroyed FIRST. A
/// character holds a pointer to its system, as Jolt's own CharacterVirtual
/// does, and outliving it is a dangling one.
ZJOLT_API void zjoltPhysicsSystemDestroy(ZJoltPhysicsSystem *system);

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

/// Advances the simulation by `delta_time`, split into `collision_steps`
/// sub-steps (1 is normal; raise it for fast bodies rather than shortening
/// the frame).
///
/// `out_error` receives a ZJoltUpdateError mask; it may be NULL. A non-zero
/// mask means contacts were dropped, not that the step failed.
///
/// Note what Jolt does about that mask: `PhysicsSystem::Update` ASSERTS that
/// it is empty before returning it (`PhysicsSystem.cpp:679`). So in a build
/// with asserts enabled, the condition this mask exists to report breaks into
/// the debugger first — which is defensible during development, and means the
/// mask is what you actually read in a build with asserts off. Either way it
/// says the same thing: a limit in ZJoltPhysicsSystemDesc is too low.
ZJOLT_API ZJoltResult zjoltPhysicsSystemStep(ZJoltPhysicsSystem *system,
                                             float delta_time,
                                             int32_t collision_steps,
                                             ZJoltJobSystem *job_system,
                                             uint32_t *out_error);

/// The ceiling this system was created with, which cannot grow. 0 if `system`
/// is NULL.
ZJOLT_API uint32_t zjoltPhysicsSystemGetMaxBodies(
    const ZJoltPhysicsSystem *system);

/// True if the two bodies were touching during the LAST step.
///
/// This reads the contact cache, so it answers a question about the step that
/// has already run rather than about where the bodies are now — which is what
/// a gameplay rule usually wants, and is far cheaper than an overlap query.
/// Order does not matter. Do not call it during a step.
ZJOLT_API bool zjoltPhysicsSystemWereBodiesInContact(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2);

//===----------------------------------------------------------------------===//
// Simulation settings
//
// The constants of the solver, as one flat struct. This is Jolt's
// PhysicsSettings verbatim — every field, spelled in this header's convention.
//
// There is no per-field setter, on purpose. Several of these interact (the two
// iteration counts, the three sleep thresholds, the two body-pair cache
// tolerances), and a caller holding the whole struct can see what else is in
// play. Read, change what you mean, write it back.
//
// Everything here takes effect from the next step. None of it is saved by
// zjoltPhysicsSystemSaveState: this is configuration, not state, and restoring
// a world does not restore how it was tuned.
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
/// Rejected rather than applied: a non-positive batch size, batches per job,
/// or in-flight body pair count, a zero velocity iteration count, and a
/// negative sleep threshold. Jolt asserts none of those. The first two are a
/// divisor and a loop stride inside the step, so a zero there is a division by
/// zero or a step that never finishes, discovered several frames from the call
/// that caused it.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetSettings(
    ZJoltPhysicsSystem *system, const ZJoltPhysicsSettings *settings);

//===----------------------------------------------------------------------===//
// Combine callbacks
//
// What friction and restitution a contact gets when the two bodies disagree.
// Jolt's defaults are sqrt(f1 * f2) and max(r1, r2); a host with a material
// table usually wants its own answer.
//
// These run on Jolt's job threads DURING a step, once per contact, under the
// same rules as ZJoltContactListener: re-entrant, no calls back into the
// system, and nothing may propagate out of them. A callback that unwinds — a
// C++ exception, a Zig panic — leaves the solver's locks held and the NEXT
// step deadlocks, with nothing to connect it to its cause.
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
/// Jolt's own hook is a bare function pointer with no user parameter, so the
/// `user` pointer is carried in a fixed table of slots on this side. A system
/// takes a slot the first time either combine callback is installed and gives
/// it back when it is destroyed; there are ZJOLT_COMBINE_SLOT_COUNT of them,
/// so that many systems may have one installed at a time. Past that this
/// returns ZJOLT_RESULT_OUT_OF_MEMORY with a message saying so, rather than
/// silently not installing.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetCombineFriction(
    ZJoltPhysicsSystem *system, ZJoltCombineFn combine, void *user);

/// NULL restores Jolt's default, max(r1, r2). @see
/// zjoltPhysicsSystemSetCombineFriction for the slot rule.
ZJOLT_API ZJoltResult zjoltPhysicsSystemSetCombineRestitution(
    ZJoltPhysicsSystem *system, ZJoltCombineFn combine, void *user);

/// How many physics systems may have a combine callback installed at once.
#define ZJOLT_COMBINE_SLOT_COUNT 64

//===----------------------------------------------------------------------===//
// Step listeners
//
// Called once per collision step, before that step's collision detection. This
// is where a host applies its own forces — buoyancy, thrusters, wind — because
// doing it here happens inside the sub-step loop, so a frame split into
// several collision steps sees the force applied at each one rather than once
// for the whole frame.
//
// Every body and constraint mutex is held for the duration, so a listener may
// READ and WRITE bodies but must not add or remove them. Listeners may run
// concurrently with each other; ZJoltPhysicsSettings::
// step_listener_batches_per_job is how to stop that. Nothing may propagate out
// of one — the note above ZJoltCombineFn applies here word for word.
//
// Jolt does NOT call step listeners when there are no active bodies, or when
// the step's delta time is zero. A listener is not a frame tick.
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
///
/// The handle belongs to the system: destroying the system destroys any
/// listener still attached. That is why it is not counted by
/// zjoltLiveHandleCount and cannot outlive the allocator it came from — the
/// system is counted, and it has to go first. `user` must outlive the
/// listener.
///
/// The same function pointer may be added more than once; each add yields its
/// own handle and each must be removed separately. Jolt itself asserts on a
/// listener added twice, which is why the handle is the identity here and the
/// function pointer is not.
ZJOLT_API ZJoltResult zjoltPhysicsSystemAddStepListener(
    ZJoltPhysicsSystem *system, ZJoltStepListenerFn listener, void *user,
    ZJoltStepListener **out);

/// Removes and destroys a listener. Reports ZJOLT_RESULT_INVALID_ARGUMENT if
/// the handle does not belong to this system, where Jolt would assert.
ZJOLT_API ZJoltResult zjoltPhysicsSystemRemoveStepListener(
    ZJoltPhysicsSystem *system, ZJoltStepListener *listener);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SYSTEM_H_
