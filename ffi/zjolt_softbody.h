//===----------------------------------------------------------------------===//
// zjolt — soft bodies: shared topology, creation, and per-step read-back.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// A soft body is built in two stages, same as Jolt's own:
//
//   1. A ZJoltSoftBodySharedSettings carries the topology — vertices, faces,
//      and the constraints that hold them together. It is reference counted
//      and shareable: build one cloth or one blob of jelly and stamp out many
//      soft bodies from it. Vertices and constraint arrays are appended in
//      bulk, and CreateConstraints derives edge/shear/bend constraints from
//      the faces automatically rather than requiring every edge by hand.
//   2. A ZJoltSoftBodyDesc (Jolt's SoftBodyCreationSettings, flattened) places
//      one instance of that topology in the world. It is NOT the same call as
//      a rigid body: soft bodies go through
//      BodyInterface::CreateSoftBody, a distinct entry point from CreateBody,
//      because a soft body has no shape, no motion type and no mass override
//      of its own — its "shape" IS its simulated vertices, and its dynamics
//      come from SoftBodyCreationSettings's own fields (iteration count,
//      pressure, per-vertex radius) instead. The body id it returns is an
//      ordinary ZJoltBodyId: add it, remove it, and put it in a collision
//      group through the existing zjoltBody* calls exactly like a rigid body.
//
// Per step, a soft body's vertices move on their own — there is no transform
// to read back the way a rigid body has one. zjoltSoftBodyGetVertexStates is
// the bulk read-back for that: one lock, one crossing, every vertex.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_SOFTBODY_H_
#define ZJOLT_SOFTBODY_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Shared topology for one or more soft bodies: vertices, faces, and the
/// constraints Jolt's soft-body solver enforces between them. Reference
/// counted like a shape — build once, create as many soft bodies from it as
/// you like, release when done creating.
typedef struct ZJoltSoftBodySharedSettings ZJoltSoftBodySharedSettings;

//===----------------------------------------------------------------------===//
// Geometry and constraints
//===----------------------------------------------------------------------===//

/// A particle. Position and velocity are in the settings' own local space —
/// the space CreateConstraints and CalculateEdgeLengths measure distances in
/// — not world space.
typedef struct ZJoltSoftBodyVertex {
  ZJoltVec3 position;
  ZJoltVec3 velocity;
  /// 0 pins the vertex: it never moves under simulation.
  float inv_mass;
} ZJoltSoftBodyVertex;

/// A triangle of the body's surface. `vertex[0..2]` must be pairwise
/// distinct — a face that repeats a vertex has no area and Jolt's own
/// AddFace asserts on it (SoftBodySharedSettings.h:345); this is checked and
/// reported as ZJOLT_RESULT_INVALID_ARGUMENT instead.
typedef struct ZJoltSoftBodyFace {
  uint32_t vertex[3];
  /// Index into the settings' material list. There is no call to set that
  /// list through this ABI yet, so every face uses the shared default
  /// material.
  uint32_t material_index;
} ZJoltSoftBodyFace;

/// A spring holding two vertices at a fixed rest length. The rest length
/// itself is not a field here: it is measured from the vertices' positions by
/// zjoltSoftBodySharedSettingsCalculateEdgeLengths (or by CreateConstraints,
/// which calls it for you), not supplied up front.
///
/// `vertex[0]` and `vertex[1]` must differ — Jolt asserts on an edge that
/// connects a vertex to itself (SoftBodySharedSettings.cpp:375).
typedef struct ZJoltSoftBodyEdge {
  uint32_t vertex[2];
  /// Inverse of the spring's stiffness. 0 is rigid.
  float compliance;
} ZJoltSoftBodyEdge;

/// Keeps the volume of a tetrahedron of four vertices constant. The four
/// vertices must be pairwise distinct, the same way a face's three must be.
typedef struct ZJoltSoftBodyVolumeConstraint {
  uint32_t vertex[4];
  float compliance;
} ZJoltSoftBodyVolumeConstraint;

/// One joint's inverse bind matrix, for skinning a soft body to an animated
/// skeleton. `matrix` is Jolt's own Mat44 layout: four columns of four
/// floats each (column-major), so a column c's row r is `matrix[4*c + r]` —
/// NOT row-major.
///
/// Referenced by index from ZJoltSoftBodySkinned::weights, so add every
/// joint's inverse bind matrix a Skinned constraint will use before or
/// alongside it; nothing here checks that a weight's index is in range at
/// add time, because the final count is not known until you stop adding.
typedef struct ZJoltSoftBodyInvBind {
  uint32_t joint_index;
  float matrix[16];
} ZJoltSoftBodyInvBind;

/// One joint's contribution to a skinned vertex. `inv_bind_index` indexes the
/// settings' inverse-bind-matrix list (the order zjoltSoftBodySharedSettings-
/// AddInvBindMatrices calls built it in, not `joint_index`).
typedef struct ZJoltSoftBodySkinWeight {
  uint32_t inv_bind_index;
  float weight;
} ZJoltSoftBodySkinWeight;

/// Skins one simulated vertex to up to four joints and limits how far the
/// simulated vertex may stray from its skinned position. The bind pose is
/// the vertex's own ZJoltSoftBodyVertex::position.
///
/// `weights` holds up to four entries; the first with `weight == 0` ends the
/// list, matching Jolt's own convention (SoftBodySharedSettings.h:280) —
/// pad unused trailing entries with a zero weight rather than zero-length
/// the list some other way.
typedef struct ZJoltSoftBodySkinned {
  uint32_t vertex;
  ZJoltSoftBodySkinWeight weights[4];
  /// FLT_MAX disables the limit entirely; 0 hard-skins the vertex.
  float max_distance;
  /// Disabled when this is >= max_distance, which is also its default.
  float back_stop_distance;
  float back_stop_radius;
} ZJoltSoftBodySkinned;

/// The type of bend constraint zjoltSoftBodySharedSettingsCreateConstraints
/// builds between adjacent faces.
typedef enum ZJoltSoftBodyBendType {
  ZJOLT_SOFT_BODY_BEND_TYPE_NONE = 0,
  ZJOLT_SOFT_BODY_BEND_TYPE_DISTANCE = 1,
  /// Most expensive; the only one that still bends correctly when the two
  /// triangles do not start out coplanar.
  ZJOLT_SOFT_BODY_BEND_TYPE_DIHEDRAL = 2,
} ZJoltSoftBodyBendType;

/// How CreateConstraints measures the long-range-attachment distance it
/// bakes into a vertex's LRA constraint, if any.
typedef enum ZJoltSoftBodyLraType {
  ZJOLT_SOFT_BODY_LRA_TYPE_NONE = 0,
  ZJOLT_SOFT_BODY_LRA_TYPE_EUCLIDEAN_DISTANCE = 1,
  /// Follows edge constraints rather than cutting straight through the body.
  ZJOLT_SOFT_BODY_LRA_TYPE_GEODESIC_DISTANCE = 2,
} ZJoltSoftBodyLraType;

/// Per-vertex knobs for zjoltSoftBodySharedSettingsCreateConstraints. One
/// list, indexed in parallel with the vertices already added — see that
/// function for what happens when the list is shorter.
typedef struct ZJoltSoftBodyVertexAttributes {
  float compliance;
  float shear_compliance;
  float bend_compliance;
  ZJoltSoftBodyLraType lra_type;
  float lra_max_distance_multiplier;
} ZJoltSoftBodyVertexAttributes;

/// Fills `out` with Jolt's own defaults.
ZJOLT_API void zjoltSoftBodyVertexAttributesInit(
    ZJoltSoftBodyVertexAttributes *out);

//===----------------------------------------------------------------------===//
// Shared settings
//===----------------------------------------------------------------------===//

/// Creates empty shared settings, returned with a reference count of one.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsCreate(
    ZJoltSoftBodySharedSettings **out);

ZJOLT_API void zjoltSoftBodySharedSettingsAddRef(
    const ZJoltSoftBodySharedSettings *settings);
ZJOLT_API void zjoltSoftBodySharedSettingsRelease(
    const ZJoltSoftBodySharedSettings *settings);
ZJOLT_API uint32_t zjoltSoftBodySharedSettingsGetRefCount(
    const ZJoltSoftBodySharedSettings *settings);

/// Appends `count` vertices. Vertex indices used by every other Add* call
/// below refer to the order vertices were appended in, across every call
/// made so far — so add all of a body's vertices before the faces and
/// constraints that reference them.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddVertices(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyVertex *vertices,
    uint32_t count);

/// Appends `count` faces. Fails without adding any of them if one is
/// degenerate (repeats a vertex) — see ZJoltSoftBodyFace.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddFaces(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyFace *faces,
    uint32_t count);

/// Appends `count` edge constraints directly, bypassing CreateConstraints.
/// Fails without adding any of them if one connects a vertex to itself.
/// Rest lengths are left at Jolt's placeholder (1.0) until
/// zjoltSoftBodySharedSettingsCalculateEdgeLengths runs.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddEdges(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyEdge *edges,
    uint32_t count);

/// Appends `count` volume constraints. Fails without adding any of them if
/// one repeats a vertex among its four. Rest volumes are left at Jolt's
/// placeholder until settings->CalculateVolumeConstraintVolumes would run —
/// which is not exposed here; supply the constraints through
/// CreateConstraints's automatic path if you need that calculated.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddVolumeConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyVolumeConstraint *constraints, uint32_t count);

/// Appends `count` inverse bind matrices, for ZJoltSoftBodySkinned::weights
/// to reference by index.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddInvBindMatrices(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyInvBind *inv_binds, uint32_t count);

/// Appends `count` skinned constraints.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddSkinnedConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodySkinned *constraints, uint32_t count);

/// Builds edge, shear and bend constraints from the faces already added,
/// instead of listing every edge by hand. Also derives an LRA constraint per
/// vertex when `vertex_attributes[i].lra_type` asks for one, and calls
/// CalculateEdgeLengths for you.
///
/// `vertex_attributes` is indexed in parallel with the vertices added so
/// far; if it is shorter than that, Jolt repeats its last element for the
/// remaining vertices (SoftBodySharedSettings.h:53) — which means
/// `vertex_attributes_count` must be at least 1, since repeating the last
/// element of an empty list has no element to repeat. That is checked here
/// and reported as ZJOLT_RESULT_INVALID_ARGUMENT rather than left to read
/// off the end of the caller's array.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsCreateConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyVertexAttributes *vertex_attributes,
    uint32_t vertex_attributes_count, ZJoltSoftBodyBendType bend_type,
    float angle_tolerance_radians);

/// Measures every edge constraint's rest length from its two vertices'
/// current positions. CreateConstraints already does this for the edges it
/// builds; call this too if you added edges directly through
/// zjoltSoftBodySharedSettingsAddEdges.
ZJOLT_API void zjoltSoftBodySharedSettingsCalculateEdgeLengths(
    ZJoltSoftBodySharedSettings *settings);

/// Reorders constraints so the solver can run groups of them in parallel.
/// Call once after every vertex, face and constraint has been added, and
/// before creating any soft body from these settings — Jolt asserts if a
/// body is simulated without this having run
/// (SoftBodyMotionProperties.cpp:1060).
ZJOLT_API void zjoltSoftBodySharedSettingsOptimize(
    ZJoltSoftBodySharedSettings *settings);

//===----------------------------------------------------------------------===//
// Creating a soft body
//
// Goes through BodyInterface::CreateSoftBody, a distinct entry point from
// CreateBody — see the note at the top of this file for why. The body id
// that comes back is ordinary: zjoltBodyAdd/Remove/Destroy, collision groups,
// and every other zjoltBody* call in zjolt_body.h and zjolt_group.h work on
// it exactly as they would on a rigid body.
//===----------------------------------------------------------------------===//

typedef struct ZJoltSoftBodyDesc {
  /// Required. A reference is taken for the lifetime of the body — call
  /// zjoltSoftBodySharedSettingsOptimize on it first.
  const ZJoltSoftBodySharedSettings *shared_settings;
  ZJoltRVec3 position;
  ZJoltQuat rotation;
  uint64_t user_data;
  ZJoltObjectLayer object_layer;
  uint32_t num_iterations;
  float linear_damping;
  float max_linear_velocity;
  float restitution;
  float friction;
  /// n * R * T: amount of substance times the ideal gas constant times
  /// absolute temperature. 0 disables internal pressure entirely.
  float pressure;
  float gravity_factor;
  /// Pushes vertices this far off the surface of whatever they collide
  /// with, to reduce z-fighting.
  float vertex_radius;
  bool update_position;
  /// Bakes `rotation` into the vertices and gives the body an identity
  /// rotation instead, which is slightly more accurate to simulate.
  bool make_rotation_identity;
  bool allow_sleeping;
  bool faces_double_sided;
} ZJoltSoftBodyDesc;

/// Fills `desc` with Jolt's own defaults. Call this first and then overwrite;
/// the defaults are not all zero and are not all obvious.
ZJOLT_API void zjoltSoftBodyDescInit(ZJoltSoftBodyDesc *desc);

/// Creates a soft body without adding it to the simulation.
ZJOLT_API ZJoltResult zjoltSoftBodyCreate(ZJoltPhysicsSystem *system,
                                          const ZJoltSoftBodyDesc *desc,
                                          ZJoltBodyId *out);

ZJOLT_API ZJoltResult zjoltSoftBodyCreateAndAdd(ZJoltPhysicsSystem *system,
                                                const ZJoltSoftBodyDesc *desc,
                                                ZJoltActivation activation,
                                                ZJoltBodyId *out);

//===----------------------------------------------------------------------===//
// Per-step read-back
//===----------------------------------------------------------------------===//

/// One simulated vertex's current state. Both fields are relative to the
/// soft body's CENTER OF MASS, not world space and not the settings' local
/// space vertices were authored in — the same frame
/// zjoltBodyGetCenterOfMassPositionLocked and zjoltBodyGetRotation report.
/// Add the body's center-of-mass position (rotated by its rotation, if the
/// body has moved) to place a vertex in the world.
typedef struct ZJoltSoftBodyVertexState {
  ZJoltVec3 position;
  ZJoltVec3 velocity;
} ZJoltSoftBodyVertexState;

/// Reads every simulated vertex of one soft body under a single lock.
///
/// Two-call protocol: `*out_count` always receives the vertex count, so a
/// capacity of 0 with `out_states` NULL is a size query. `body` naming a
/// rigid body (or no body at all) is ZJOLT_RESULT_INVALID_ARGUMENT and
/// ZJOLT_RESULT_BODY_NOT_FOUND respectively; `*out_count` is 0 either way.
ZJOLT_API ZJoltResult zjoltSoftBodyGetVertexStates(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltSoftBodyVertexState *out_states, uint32_t capacity,
    uint32_t *out_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SOFTBODY_H_
