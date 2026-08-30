//===----------------------------------------------------------------------===//
// zjolt — soft bodies: shared topology, creation, per-step read-back, and the
// per-instance properties a host drives while the simulation runs.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part.
//
// Built in two stages: ZJoltSoftBodySharedSettings carries the reference-
// counted, shareable topology (vertices, faces, constraints); ZJoltSoftBodyDesc
// places one instance in the world via BodyInterface::CreateSoftBody, a
// distinct entry point from CreateBody. The returned ZJoltBodyId is ordinary.
// zjoltSoftBodyGetVertexStates is the per-step bulk read-back; there is no
// transform to read the way a rigid body has one.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_SOFTBODY_H_
#define ZJOLT_SOFTBODY_H_

#include "zjolt_core.h"

// For ZJoltCollisionGroup, which ZJoltSoftBodyDesc carries.
#include "zjolt_group.h"
// For ZJoltPlane, which the vertex-collision iterator below reads and writes.
#include "zjolt_shape.h"

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
  /// Index into the settings' material list, as set by
  /// zjoltSoftBodySharedSettingsSetMaterials. Settings that never had that
  /// called carry exactly one material — Jolt's shared default — so 0 is the
  /// only index they have.
  uint32_t material_index;
} ZJoltSoftBodyFace;

/// A spring holding two vertices at a fixed rest length. The rest length is
/// not a field here: it is measured by
/// zjoltSoftBodySharedSettingsCalculateEdgeLengths (or by CreateConstraints).
///
/// `vertex[0]` and `vertex[1]` must differ.
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

/// A discrete Cosserat rod: two vertices at a fixed length, like an edge,
/// but with an ORIENTATION too, read back via zjoltSoftBodyGetRodStates.
///
/// Length, inverse mass and rest frame are derived by
/// zjoltSoftBodySharedSettingsCalculateRodProperties — NOT optional, unlike
/// CalculateEdgeLengths. Needs a ZJoltSoftBodyRodBendTwist to stop spinning.
typedef struct ZJoltSoftBodyRodStretchShear {
  /// Must differ, and must be at two DISTINCT positions — a rod of zero
  /// length is refused rather than left to Jolt's assert.
  uint32_t vertex[2];
  /// Inverse of the rod's stiffness. 0 is rigid.
  float compliance;
} ZJoltSoftBodyRodStretchShear;

/// Limits the bend and twist between two rods, indexed into the rods added
/// so far. Stops a rod spinning freely and chains rods for Bishop frames.
///
/// zjoltSoftBodySharedSettingsOptimize REORDERS rods and remaps these
/// indices; author every bend-twist constraint before optimising.
typedef struct ZJoltSoftBodyRodBendTwist {
  /// Must differ, and each must name a rod already added.
  uint32_t rod[2];
  float compliance;
} ZJoltSoftBodyRodBendTwist;

/// One joint's inverse bind matrix, for skinning to an animated skeleton.
/// `matrix` is column-major: column c's row r is `matrix[4*c + r]`.
///
/// Referenced by index from ZJoltSoftBodySkinned::weights; add every joint
/// used before or alongside it. Not range-checked at add time — the final
/// count is unknown until you stop adding.
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

/// Skins one simulated vertex to up to four joints and limits how far it may
/// stray from its skinned position. The bind pose is the vertex's own
/// ZJoltSoftBodyVertex::position.
///
/// `weights` holds up to four entries; the first with `weight == 0` ends the
/// list — pad unused trailing entries with a zero weight.
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

/// A deep copy of `settings`, returned with a reference count of one,
/// sharing nothing but material references. The only way to vary a
/// finished topology, since every other call only appends.
///
/// Carries the original's optimisation state; adding to the clone
/// afterward requires re-optimising, same as the original would.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsClone(
    const ZJoltSoftBodySharedSettings *settings,
    ZJoltSoftBodySharedSettings **out);

/// Replaces the settings' material list, indexed by
/// ZJoltSoftBodyFace::material_index. A reference is taken on each; NULL
/// entries are refused. Read by queries, not the solver.
///
/// `count` of 0 is refused (Jolt indexes it unguarded). A face already
/// added whose material_index is past the new list refuses the whole call.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsSetMaterials(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltPhysicsMaterial *const *materials, uint32_t count);

/// The settings' material list, in ZJoltSoftBodyFace::material_index order.
/// Two-call protocol: out_materials = NULL learns the count. Borrowed
/// pointers, valid as long as the settings are; zjoltPhysicsMaterialAddRef
/// to outlive them. Settings that never had SetMaterials called report one
/// material: zjoltPhysicsMaterialDefault().
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsGetMaterials(
    const ZJoltSoftBodySharedSettings *settings,
    const ZJoltPhysicsMaterial **out_materials, uint32_t capacity,
    uint32_t *out_count);

/// Jolt's own binary form of the settings — every vertex, face, constraint
/// and inverse bind matrix — over the same ZJoltStream seam as
/// zjoltShapeSaveStream. Does NOT write the material list, which is Jolt's
/// own split: zjoltSoftBodySharedSettingsSaveWithMaterials writes both.
///
/// A body's runtime state is not here. This is the asset, not the simulation.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsSaveBinaryState(
    const ZJoltSoftBodySharedSettings *settings, const ZJoltStream *stream);

/// Rebuilds what zjoltSoftBodySharedSettingsSaveBinaryState wrote, with the
/// default material on every face. Release the result with
/// zjoltSoftBodySharedSettingsRelease.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsRestoreBinaryState(
    const ZJoltStream *stream, ZJoltSoftBodySharedSettings **out);

/// The settings AND the material list they index, in one stream — what
/// zjoltSceneSaveStream writes per soft body, reachable for a single asset.
/// Every material must be one this ABI can rebuild by RTTI name.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsSaveWithMaterials(
    const ZJoltSoftBodySharedSettings *settings, const ZJoltStream *stream);

/// Rebuilds what zjoltSoftBodySharedSettingsSaveWithMaterials wrote,
/// materials included. Release the result with
/// zjoltSoftBodySharedSettingsRelease.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsRestoreWithMaterials(
    const ZJoltStream *stream, ZJoltSoftBodySharedSettings **out);

/// Appends `count` vertices. Vertex indices in every other Add* call below
/// refer to the append order across every call so far — add all vertices
/// before the faces and constraints that reference them.
///
/// Enforced, not advised: every Add* below refuses an out-of-range index,
/// since Jolt indexes its vertex array unguarded.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddVertices(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyVertex *vertices,
    uint32_t count);

/// Appends `count` faces. Fails without adding any of them if one is
/// degenerate (repeats a vertex) — see ZJoltSoftBodyFace — or names a vertex
/// index past the vertices added so far.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddFaces(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyFace *faces,
    uint32_t count);

/// Appends `count` edge constraints directly, bypassing CreateConstraints.
/// Fails without adding any of them if one connects a vertex to itself, or
/// names a vertex index past the vertices added so far.
/// Rest lengths are left at Jolt's placeholder (1.0) until
/// zjoltSoftBodySharedSettingsCalculateEdgeLengths runs.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddEdges(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyEdge *edges,
    uint32_t count);

/// Appends `count` volume constraints. Fails without adding any of them if
/// one repeats a vertex among its four, or names a vertex index past the
/// vertices added so far. Rest volumes are left at Jolt's
/// placeholder until settings->CalculateVolumeConstraintVolumes would run —
/// which is not exposed here; supply the constraints through
/// CreateConstraints's automatic path if you need that calculated.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddVolumeConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyVolumeConstraint *constraints, uint32_t count);

/// Appends `count` Cosserat rods. Fails without adding any if one connects
/// a vertex to itself, or names an out-of-range vertex index.
///
/// Not usable until zjoltSoftBodySharedSettingsCalculateRodProperties has
/// run. A rod's index (for bend-twist constraints) is its append position,
/// like a vertex's.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddRodStretchShearConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyRodStretchShear *rods, uint32_t count);

/// Appends `count` rod bend-twist constraints. Fails without adding any of
/// them if one names the same rod twice, or names a rod index past the rods
/// added so far — Jolt indexes its rod array with these while calculating
/// rod properties and while solving, unguarded.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddRodBendTwistConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyRodBendTwist *constraints, uint32_t count);

/// Appends `count` inverse bind matrices, for ZJoltSoftBodySkinned::weights
/// to reference by index.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddInvBindMatrices(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyInvBind *inv_binds, uint32_t count);

/// Appends `count` skinned constraints. Fails without adding any if one
/// names an out-of-range vertex index — indexed, not looked up, every solve.
///
/// A weight's `inv_bind_index` is NOT checked here, since the inverse
/// bind matrix list may still grow; zjoltSoftBodySkinVertices checks it
/// once the final count is known.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddSkinnedConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodySkinned *constraints, uint32_t count);

/// Builds edge, shear and bend constraints from the faces already added. Also
/// derives an LRA constraint per vertex when `vertex_attributes[i].lra_type`
/// asks for one, and calls CalculateEdgeLengths.
///
/// `vertex_attributes` is indexed in parallel with the vertices, repeating its
/// last element if shorter; `vertex_attributes_count` must be >= 1.
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

/// Measures every volume constraint's rest volume from its four vertices'
/// positions, like CalculateEdgeLengths measures an edge. Unlike edges,
/// NOTHING calls this for you: tetrahedra keep Jolt's placeholder rest
/// volume of 1 until this runs, so a jelly inflates or collapses on
/// creation without it. A coplanar tetrahedron gets a legal rest volume
/// of zero ("keep it flat"), not a refusal.
ZJOLT_API void zjoltSoftBodySharedSettingsCalculateVolumeConstraintVolumes(
    ZJoltSoftBodySharedSettings *settings);

/// Derives every rod's length, inverse mass and rest orientation, and
/// every bend-twist constraint's rest rotation. Run once, after every rod
/// and bend-twist constraint, before zjoltSoftBodySharedSettingsOptimize —
/// NOT optional: skipping it seeds every simulation at a zero quaternion.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT if both vertices sit at the same position.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsCalculateRodProperties(
    ZJoltSoftBodySharedSettings *settings);

/// Works out which faces meet at each skinned vertex, giving it a normal —
/// the direction ZJoltSoftBodySkinned::back_stop_distance is measured
/// along. Not called by Optimize or CreateConstraints; skipping it leaves
/// every skinned vertex with a zero normal (back stop then does nothing).
/// ZJOLT_RESULT_INVALID_ARGUMENT past 255 qualifying faces at one vertex.
ZJOLT_API ZJoltResult
zjoltSoftBodySharedSettingsCalculateSkinnedConstraintNormals(
    ZJoltSoftBodySharedSettings *settings);

/// Reorders constraints so the solver can run groups of them in parallel.
/// Call once after every vertex, face and constraint has been added, and
/// before creating any soft body from these settings — Jolt asserts if a
/// body is simulated without this having run
/// (SoftBodyMotionProperties.cpp:1060).
ZJOLT_API void zjoltSoftBodySharedSettingsOptimize(
    ZJoltSoftBodySharedSettings *settings);

/// How long each of zjoltSoftBodySharedSettingsOptimizeWithRemap's seven
/// remaps is: the length of the matching constraint list BEFORE optimising,
/// which is what makes a capacity checkable in advance.
typedef struct ZJoltSoftBodyRemapCounts {
  uint32_t edges;
  uint32_t lra;
  uint32_t rod_stretch_shear;
  uint32_t rod_bend_twist;
  uint32_t dihedral_bend;
  uint32_t volume;
  uint32_t skinned;
} ZJoltSoftBodyRemapCounts;

/// Where to write each remap. Any member may be NULL to skip that one.
/// Field for field the same seven as ZJoltSoftBodyRemapCounts, in the same
/// order; zjolt_softbody.cpp static_asserts that neither grows alone.
typedef struct ZJoltSoftBodyRemapBuffers {
  uint32_t *edges;
  uint32_t *lra;
  uint32_t *rod_stretch_shear;
  uint32_t *rod_bend_twist;
  uint32_t *dihedral_bend;
  uint32_t *volume;
  uint32_t *skinned;
} ZJoltSoftBodyRemapBuffers;

/// Fills `out` with the seven lengths above. Cheap; no allocation.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsGetRemapCounts(
    const ZJoltSoftBodySharedSettings *settings,
    ZJoltSoftBodyRemapCounts *out);

/// zjoltSoftBodySharedSettingsOptimize, keeping the seven index maps Jolt
/// builds while reordering: `out_remap->edges[old] == new`, and so on. Any
/// index a caller recorded before optimising names a different constraint
/// afterwards, and without these nothing says so. `capacity` is checked
/// against zjoltSoftBodySharedSettingsGetRemapCounts BEFORE anything is
/// reordered; a NULL buffer skips its remap and its capacity is not read.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsOptimizeWithRemap(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyRemapBuffers *out_remap,
    const ZJoltSoftBodyRemapCounts *capacity);

//===----------------------------------------------------------------------===//
// Creating a soft body
//
// Goes through BodyInterface::CreateSoftBody, distinct from CreateBody. The
// returned ZJoltBodyId is ordinary: every zjoltBody* call works on it too.
//===----------------------------------------------------------------------===//

typedef struct ZJoltSoftBodyDesc {
  /// Required. A reference is taken for the lifetime of the body — call
  /// zjoltSoftBodySharedSettingsOptimize on it first.
  const ZJoltSoftBodySharedSettings *shared_settings;
  /// Which exceptions this body makes to layer-based collision, on the same
  /// terms as ZJoltBodyDesc::collision_group: zjoltSoftBodyDescInit writes
  /// "no group, no filter", and the body takes its own reference on the
  /// filter when it is created.
  ZJoltCollisionGroup collision_group;
  ZJoltRVec3 position;
  ZJoltQuat rotation;
  uint64_t user_data;
  ZJoltObjectLayer object_layer;
  /// Solver iterations per step. Must be at least 1, and 0 is refused rather
  /// than forwarded: Jolt sizes its sub-step as the step's delta time divided
  /// by this (SoftBodyMotionProperties.cpp:953), so a zero makes every
  /// sub-step infinite. Jolt does not assert on it.
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
  /// with, to reduce z-fighting. Negative is refused — see
  /// zjoltSoftBodySetVertexRadius for what Jolt's own assert on this does
  /// and does not catch.
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

/// As zjoltSoftBodyCreate, but the body takes the id the caller names instead
/// of the next one Jolt would assign — see zjoltBodyCreateWithId in
/// zjolt_body.h, which this mirrors exactly, `id` requirements included.
ZJOLT_API ZJoltResult zjoltSoftBodyCreateWithId(ZJoltPhysicsSystem *system,
                                                const ZJoltSoftBodyDesc *desc,
                                                ZJoltBodyId id,
                                                ZJoltBodyId *out);

/// As zjoltSoftBodyCreateWithId, followed immediately by zjoltBodyAdd.
ZJOLT_API ZJoltResult zjoltSoftBodyCreateAndAddWithId(
    ZJoltPhysicsSystem *system, const ZJoltSoftBodyDesc *desc, ZJoltBodyId id,
    ZJoltActivation activation, ZJoltBodyId *out);

/// The shared topology a live soft body was built from, the reference
/// `shared_settings` took at creation. Borrowed; takes no reference, not
/// valid past the body's own destruction.
///
/// NULL for a rigid body, and for a stale id — not distinguishable through
/// this call alone; ask zjoltBodyGetBodyType first if that matters.
ZJOLT_API const ZJoltSoftBodySharedSettings *zjoltSoftBodyGetSharedSettings(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

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

//===----------------------------------------------------------------------===//
// Live properties of one soft body
//
// Runtime knobs on SoftBodyMotionProperties. A body id naming nothing is
// ZJOLT_RESULT_BODY_NOT_FOUND, a RIGID body ZJOLT_RESULT_INVALID_ARGUMENT.
//===----------------------------------------------------------------------===//

/// Solver iterations this body runs per step. @see ZJoltSoftBodyDesc for why
/// 0 is refused.
ZJOLT_API ZJoltResult zjoltSoftBodyGetNumIterations(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetNumIterations(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t num_iterations);

/// Internal gas pressure: n * R * T. 0 disables it entirely.
///
/// Computed from the volume the faces enclose: means nothing on an open
/// surface (a flat sheet of cloth), and pulls INWARD if faces wind inside
/// out — zjoltSoftBodyGetVolume tells the two apart.
ZJOLT_API ZJoltResult zjoltSoftBodyGetPressure(const ZJoltPhysicsSystem *system,
                                               ZJoltBodyId body, float *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetPressure(ZJoltPhysicsSystem *system,
                                               ZJoltBodyId body,
                                               float pressure);

/// Whether the body's own position follows its vertices as they move. Turn it
/// off for a soft body pinned to the static world, whose vertices move but
/// whose origin should not.
ZJOLT_API ZJoltResult zjoltSoftBodyGetUpdatePosition(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, bool *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetUpdatePosition(ZJoltPhysicsSystem *system,
                                                     ZJoltBodyId body,
                                                     bool update_position);

/// Whether ray casts, collide-shape and cast-shape against this body hit its
/// faces from behind as well as from in front. Affects queries only — the
/// solver's own collision handling does not read it.
ZJOLT_API ZJoltResult zjoltSoftBodyGetFacesDoubleSided(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, bool *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetFacesDoubleSided(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, bool double_sided);

/// How far this body's vertices are held off the surface of whatever they
/// touch. Negative is refused.
///
/// Jolt's own assert checks the value being OVERWRITTEN, not the incoming
/// one — so it can miss a bad radius for a call or forever. This checks
/// the incoming value instead.
ZJOLT_API ZJoltResult zjoltSoftBodyGetVertexRadius(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, float *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetVertexRadius(ZJoltPhysicsSystem *system,
                                                   ZJoltBodyId body,
                                                   float vertex_radius);

//===----------------------------------------------------------------------===//
// One vertex at a time
//
// Only velocity and inverse mass may be written while running (no position
// setter, to avoid skipping collisions); `index` is bounds-checked.
//===----------------------------------------------------------------------===//

/// One vertex's velocity, RELATIVE TO THE BODY'S CENTRE OF MASS — the frame
/// ZJoltSoftBodyVertexState uses, not world space.
ZJOLT_API ZJoltResult zjoltSoftBodyGetVertexVelocity(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t index,
    ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetVertexVelocity(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t index,
    const ZJoltVec3 *velocity);

/// One vertex's inverse mass. 0 pins it: it still takes part in every
/// constraint, but nothing moves it.
///
/// Does NOT recompute the body's own mass and inertia, derived once at
/// creation. @see zjoltSoftBodyCalculateMassAndInertia.
ZJOLT_API ZJoltResult zjoltSoftBodyGetVertexInvMass(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t index,
    float *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetVertexInvMass(ZJoltPhysicsSystem *system,
                                                    ZJoltBodyId body,
                                                    uint32_t index,
                                                    float inv_mass);

/// Recomputes the body's total mass and inertia from its vertices' current
/// inverse masses and positions. Jolt does this once, at creation only —
/// a body whose per-vertex masses changed keeps its original total until
/// this runs. A single vertex with an inverse mass of 0 gives the WHOLE
/// body infinite mass and inertia.
ZJOLT_API ZJoltResult zjoltSoftBodyCalculateMassAndInertia(
    ZJoltPhysicsSystem *system, ZJoltBodyId body);

//===----------------------------------------------------------------------===//
// Cheap measurements
//===----------------------------------------------------------------------===//

/// The volume the body's faces enclose, in local space. One pass, no
/// allocation.
///
/// NEGATIVE when faces wind inside out; a nonzero number even for an open
/// surface (a sheet of cloth). Pressure force is computed from exactly
/// this, so negative means it squeezes rather than inflates.
ZJOLT_API ZJoltResult zjoltSoftBodyGetVolume(const ZJoltPhysicsSystem *system,
                                             ZJoltBodyId body, float *out);

/// The box around every vertex, in the body's own local space rather than the
/// world's. Maintained by the solver as it steps, so this reads a cached value
/// instead of walking the vertices.
ZJOLT_API ZJoltResult zjoltSoftBodyGetLocalBounds(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltAABox *out);

//===----------------------------------------------------------------------===//
// Skinning to an animated skeleton
//
// Per-frame order: animate the skeleton, zjoltSoftBodySkinVertices, then
// zjoltPhysicsSystemStep. A missed frame is not an error — positions stay.
//===----------------------------------------------------------------------===//

/// Skins every vertex carrying a Skinned constraint to `joint_matrices`
/// (indexed by ZJoltSoftBodyInvBind::joint_index, `joint_count` entries).
/// Matrices must be RELATIVE TO THE BODY'S CENTRE-OF-MASS TRANSFORM, not
/// world space — a world-space matrix is not detectably wrong, just
/// silently offset. `hard_skin_all` snaps every vertex to its skinned
/// position, a reset for a spawn or teleport.
ZJOLT_API ZJoltResult zjoltSoftBodySkinVertices(
    ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltMat44 *joint_matrices, uint32_t joint_count,
    bool hard_skin_all);

/// Whether the solver enforces this body's skin constraints at all.
///
/// Switching it off does not stop zjoltSoftBodySkinVertices working: with
/// constraints off, that call hard-skins vertices whose max_distance is 0
/// and leaves the rest free — "follows the character" vs "pinned at
/// kinematic vertices, otherwise loose".
ZJOLT_API ZJoltResult zjoltSoftBodyGetEnableSkinConstraints(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, bool *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetEnableSkinConstraints(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, bool enable);

/// Scales every skin constraint's max_distance at once, so a whole garment can
/// be tightened or loosened without touching the shared settings every
/// instance is stamped from. 1 is the default; 0 hard-skins every vertex.
ZJOLT_API ZJoltResult zjoltSoftBodyGetSkinnedMaxDistanceMultiplier(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, float *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetSkinnedMaxDistanceMultiplier(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, float multiplier);

//===----------------------------------------------------------------------===//
// Rod state
//
// The read-back that makes rods worth having: a rod carries an orientation,
// and this is where a host collects it to place the geometry riding on it.
//===----------------------------------------------------------------------===//

/// One rod's current state, both relative to the body's CENTRE OF MASS — the
/// frame ZJoltSoftBodyVertexState uses, not world space.
typedef struct ZJoltSoftBodyRodState {
  /// The rod's two vertices, which is how a host recognises WHICH rod this
  /// is: zjoltSoftBodySharedSettingsOptimize reorders rods, so the position
  /// in this array is not the order rods were added in. The pair is
  /// unordered for that purpose — CalculateRodProperties may have swapped the
  /// two while orienting a chain.
  uint32_t vertex[2];
  ZJoltQuat rotation;
  ZJoltVec3 angular_velocity;
} ZJoltSoftBodyRodState;

/// Reads every rod of one soft body under a single lock. Two-call
/// protocol: `*out_count` always receives the rod count, so capacity 0
/// with out_states NULL is a size query.
///
/// `angular_velocity` is meaningful only BETWEEN steps — mid-step (e.g. a
/// contact callback) it reads a quaternion's components, not a velocity.
ZJOLT_API ZJoltResult zjoltSoftBodyGetRodStates(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltSoftBodyRodState *out_states, uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Hit read-back and manual update
//===----------------------------------------------------------------------===//

/// Which face of `body` a sub-shape id names — turns a hit's ZJoltSubShapeId
/// into an index into the SHARED SETTINGS face list, in
/// zjoltSoftBodySharedSettingsAddFaces order (never reordered).
/// ZJOLT_RESULT_INVALID_ARGUMENT for a sub-shape id with bits left over after
/// the face index.
ZJOLT_API ZJoltResult zjoltSoftBodyGetFaceIndex(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltSubShapeId sub_shape_id, uint32_t *out);

/// Runs one soft-body update immediately, without zjoltPhysicsSystemStep —
/// for a body just teleported, or kept out of the simulation (one IS in
/// the system gets updated twice).
///
/// THREADING: takes locks on this body and everything it collides with.
/// Must not be called during a step, a contact callback, or a step listener.
ZJOLT_API ZJoltResult zjoltSoftBodyCustomUpdate(ZJoltPhysicsSystem *system,
                                                ZJoltBodyId body,
                                                float delta_time);

//===----------------------------------------------------------------------===//
// Vertex-vs-shape collision
//
// What Shape::CollideSoftBodyVertices runs on: for each vertex, keep
// whichever call reports the deepest penetration. Exposed standalone so a
// caller can drive it with no PhysicsSystem and no soft body involved, and
// so a host-implemented shape (zjolt_customshape.h) can build its own
// collide_soft_body_vertices callback from the same pieces Jolt's built-in
// shapes use internally.
//===----------------------------------------------------------------------===//

/// Strided cursor over one soft-body vertex array's collision-relevant fields,
/// mirroring Jolt's CollideSoftBodyVertexIterator: `position` and `inv_mass`
/// read-only, the rest written as vertices are found to collide. `*_stride` is
/// the byte distance between elements, not necessarily sizeof the element.
/// Every pointer is borrowed, must stay valid and unmoved while the iterator is
/// used, and may NOT be NULL: every field is dereferenced unconditionally.
typedef struct ZJoltCollideSoftBodyVertexIterator {
  const uint8_t *position;
  uint32_t position_stride;
  const uint8_t *inv_mass;
  uint32_t inv_mass_stride;
  uint8_t *collision_plane;
  uint32_t collision_plane_stride;
  uint8_t *largest_penetration;
  uint32_t largest_penetration_stride;
  uint8_t *colliding_shape_index;
  uint32_t colliding_shape_index_stride;
} ZJoltCollideSoftBodyVertexIterator;

/// Runs `shape`'s own CollideSoftBodyVertices against `vertices`, standalone —
/// no PhysicsSystem or soft body needed. Dispatches to whatever concrete shape
/// `shape` is, a soft body's own SoftBodyShape included, which is how two soft
/// bodies collide with each other. `colliding_shape_index` marks any vertex
/// this call wins, uninterpreted otherwise. ZJOLT_RESULT_INVALID_ARGUMENT for a
/// NULL argument, or any NULL field of `vertices`.
ZJOLT_API ZJoltResult zjoltShapeCollideSoftBodyVertices(
    const ZJoltShape *shape, const ZJoltMat44 *center_of_mass_transform,
    ZJoltVec3 scale, const ZJoltCollideSoftBodyVertexIterator *vertices,
    uint32_t count, int32_t colliding_shape_index);

/// Collides soft-body vertices against a caller-supplied set of triangles —
/// Jolt's CollideSoftBodyVerticesVsTriangles, the closest-point search
/// TriangleShape, MeshShape and HeightFieldShape all build their own
/// CollideSoftBodyVertices on. Useful directly for a host-implemented shape
/// backed by its own triangle soup.
typedef struct ZJoltCollideSoftBodyVerticesVsTriangles
    ZJoltCollideSoftBodyVerticesVsTriangles;

/// `center_of_mass_transform` and `scale` are fixed for this collider's
/// whole lifetime, matching Jolt's own object — built fresh per
/// CollideSoftBodyVertices call. Destroy with
/// zjoltCollideSoftBodyVerticesVsTrianglesDestroy; accepts NULL.
ZJOLT_API ZJoltResult zjoltCollideSoftBodyVerticesVsTrianglesCreate(
    const ZJoltMat44 *center_of_mass_transform, ZJoltVec3 scale,
    ZJoltCollideSoftBodyVerticesVsTriangles **out);
ZJOLT_API void zjoltCollideSoftBodyVerticesVsTrianglesDestroy(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider);

/// Begins vertex `index`'s closest-triangle search, discarding whatever
/// ProcessTriangle found for whichever vertex StartVertex last began.
/// `vertex->position` must be non-NULL; no other field is read here.
ZJOLT_API void zjoltCollideSoftBodyVerticesVsTrianglesStartVertex(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider,
    const ZJoltCollideSoftBodyVertexIterator *vertex, uint32_t index);

/// Considers one candidate triangle, LOCAL space and unscaled — `collider`
/// applies its own scale — against the vertex StartVertex began; kept only
/// if closer than every triangle already seen since.
ZJOLT_API void zjoltCollideSoftBodyVerticesVsTrianglesProcessTriangle(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider, ZJoltVec3 v0,
    ZJoltVec3 v1, ZJoltVec3 v2);

/// Commits the closest triangle found since StartVertex into vertex
/// `index`: calls UpdatePenetration and, if it wins, writes
/// collision_plane and colliding_shape_index — a no-op if no
/// ProcessTriangle call beat what was already there. `vertex->position`,
/// `collision_plane`, `largest_penetration` and `colliding_shape_index`
/// must all be non-NULL.
ZJOLT_API void zjoltCollideSoftBodyVerticesVsTrianglesFinishVertex(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider,
    const ZJoltCollideSoftBodyVertexIterator *vertex, uint32_t index,
    int32_t colliding_shape_index);

/// Marks one vertex as touching `body` via continuous collision detection —
/// SoftBodyVertex::MarkCCDContact. `*out_colliding_shape_index` is stamped
/// from `body`'s own id, NOT an index into a per-step colliding-shape list
/// the way a discrete contact's is. Does not touch largest_penetration. A
/// no-op if any output pointer is NULL.
ZJOLT_API void zjoltSoftBodyVertexMarkCcdContact(
    ZJoltBodyId body, const ZJoltPlane *contact_plane,
    ZJoltPlane *out_collision_plane, int32_t *out_colliding_shape_index,
    bool *out_has_contact);

//===----------------------------------------------------------------------===//
// SoftBodyContactListener
//
// Separate from ZJoltContactListener: a rigid-only listener hears nothing about soft bodies. Exceptions are off — NOTHING may unwind out of a callback.
// Both run WITH ALL BODIES LOCKED. Do not call back into the system from inside one.
//===----------------------------------------------------------------------===//

/// Whether a soft body's collision with one other body is processed at all.
typedef enum ZJoltSoftBodyValidateResult {
  ZJOLT_SOFT_BODY_VALIDATE_RESULT_ACCEPT_CONTACT = 0,
  ZJOLT_SOFT_BODY_VALIDATE_RESULT_REJECT_CONTACT = 1,
} ZJoltSoftBodyValidateResult;

/// Filled in with its defaults before the validate callback is called, so a
/// callback that only wants to accept or reject need not touch it. Every
/// field applies to ALL of the contact points that collision produces, not to
/// one of them: the soft body has already been decided against the other body
/// as a whole by the time this is asked.
typedef struct ZJoltSoftBodyContactSettings {
  /// Scales the soft body's inverse mass for this collision. 0 makes it
  /// infinitely heavy, 1 leaves it alone, 2 halves its mass. Keep it steady
  /// across steps for the same pair — the solver has no memory of what it was
  /// last frame, and a value that jitters reads as a mass that jitters.
  float inv_mass_scale1;
  /// The same, for the other body.
  float inv_mass_scale2;
  /// The same again for the other body's inverse inertia; usually set to
  /// whatever inv_mass_scale2 is.
  float inv_inertia_scale2;
  /// Treat the touch as a sensor overlap: reported, with no collision
  /// response at all.
  bool is_sensor;
} ZJoltSoftBodyContactSettings;

/// Which vertices of a soft body touched something during one step. Valid
/// ONLY for the duration of the on_contact_added callback it is handed to —
/// it is a view over the solver's own arrays, not a copy, and it does not
/// outlive the call.
typedef struct ZJoltSoftBodyManifold ZJoltSoftBodyManifold;

/// One vertex's contact, as reported through a manifold.
typedef struct ZJoltSoftBodyVertexContact {
  /// Index into the same array zjoltSoftBodyGetVertexStates reads, in the
  /// same order.
  uint32_t vertex;
  /// The body this vertex touched.
  ZJoltBodyId body;
  /// Relative to the soft body's CENTRE OF MASS, like every other position
  /// this header reports for a vertex.
  ZJoltVec3 local_contact_point;
  /// Points from the soft body INTO the body it touched — the direction the
  /// soft body pushes, not the direction it is pushed. A cloth resting on a
  /// floor reports a normal pointing down. It is the negation of the plane
  /// the solver pushes the vertex out along, and it is the same sense as a
  /// rigid contact manifold's normal.
  ZJoltVec3 normal;
} ZJoltSoftBodyVertexContact;

/// The vertices that touched something, and what each touched. Two-call
/// protocol: `*out_count` always receives the touching-vertex count;
/// capacity 0 with out_contacts NULL is a size query.
///
/// `manifold` may only be the one the current callback was handed — kept
/// past it, this reads freed solver state.
ZJOLT_API ZJoltResult zjoltSoftBodyManifoldGetVertexContacts(
    const ZJoltSoftBodyManifold *manifold,
    ZJoltSoftBodyVertexContact *out_contacts, uint32_t capacity,
    uint32_t *out_count);

/// The sensors this soft body is overlapping. Same two-call protocol and
/// lifetime rule as zjoltSoftBodyManifoldGetVertexContacts.
///
/// Installing this listener is what makes soft-body sensor overlaps
/// happen at all: Jolt skips them entirely otherwise, even for a world
/// with zjoltPhysicsSystemSetContactListener installed.
ZJOLT_API ZJoltResult zjoltSoftBodyManifoldGetSensorContacts(
    const ZJoltSoftBodyManifold *manifold, ZJoltBodyId *out_bodies,
    uint32_t capacity, uint32_t *out_count);

/// Called when a soft body's bounding box overlaps another body's, BEFORE
/// any vertex is tested — so receiving it does not mean anything touched.
/// Returning REJECT_CONTACT drops the whole pair for this step.
typedef ZJoltSoftBodyValidateResult (*ZJoltSoftBodyOnContactValidateFn)(
    void *user, ZJoltBodyId soft_body, ZJoltBodyId other_body,
    ZJoltSoftBodyContactSettings *io_settings);

/// Called once per soft body per step, after every one of its contacts has
/// been handled — not once per contact. `manifold` carries all of them.
typedef void (*ZJoltSoftBodyOnContactAddedFn)(
    void *user, ZJoltBodyId soft_body, const ZJoltSoftBodyManifold *manifold);

typedef struct ZJoltSoftBodyContactListener {
  ZJoltSoftBodyOnContactValidateFn on_contact_validate;
  ZJoltSoftBodyOnContactAddedFn on_contact_added;
  void *user;
} ZJoltSoftBodyContactListener;

/// NULL clears the listener. The struct is copied, so it need not outlive the
/// call — but its `user` pointer must outlive the system.
///
/// Returns a result rather than nothing for the reason
/// zjoltPhysicsSystemSetContactListener does: a listener that failed to
/// install is a world that silently stops reporting collisions.
ZJOLT_API ZJoltResult zjoltSoftBodySetContactListener(
    ZJoltPhysicsSystem *system, const ZJoltSoftBodyContactListener *listener);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SOFTBODY_H_
