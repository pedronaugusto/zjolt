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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SYSTEM_H_
