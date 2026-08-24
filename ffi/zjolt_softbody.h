//===----------------------------------------------------------------------===//
// zjolt — soft bodies: shared topology, creation, per-step read-back, and the
// per-instance properties a host drives while the simulation runs.
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
//
// The two stages above describe a body being born. The second half of this
// header is what a host does with one afterwards: the live properties of the
// body's SoftBodyMotionProperties, one-vertex-at-a-time control over velocity
// and inverse mass, and the per-frame skinning call that makes a cloth follow
// an animated skeleton.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_SOFTBODY_H_
#define ZJOLT_SOFTBODY_H_

#include "zjolt_core.h"

// For ZJoltCollisionGroup, which ZJoltSoftBodyDesc carries.
#include "zjolt_group.h"

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
///
/// That ordering is enforced rather than merely advised: every Add* below
/// refuses a vertex index that is not already in range. Jolt indexes its
/// vertex array with those indices unguarded, both while solving and while
/// measuring — `Array::operator[]` asserts (Array.h:566) and, in a build with
/// asserts off, reads past the end of the array instead.
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

/// Appends `count` inverse bind matrices, for ZJoltSoftBodySkinned::weights
/// to reference by index.
ZJOLT_API ZJoltResult zjoltSoftBodySharedSettingsAddInvBindMatrices(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyInvBind *inv_binds, uint32_t count);

/// Appends `count` skinned constraints. Fails without adding any of them if
/// one names a vertex index past the vertices added so far: Jolt reaches for
/// that vertex on every zjoltSoftBodySkinVertices call and on every step the
/// skin constraints are solved, and it is indexed rather than looked up.
///
/// A weight's `inv_bind_index` is NOT checked here, for the reason
/// ZJoltSoftBodyInvBind gives — the inverse bind matrix list is still allowed
/// to grow. zjoltSoftBodySkinVertices checks it instead, at the one point the
/// final count is known.
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

/// Works out which faces meet at each skinned vertex, which is what gives
/// that vertex a normal — and the normal is the direction
/// ZJoltSoftBodySkinned::back_stop_distance is measured along.
///
/// Neither zjoltSoftBodySharedSettingsOptimize nor CreateConstraints calls
/// this. Settings that never had it run still simulate: every skinned vertex
/// simply has a zero normal, so the back stop does nothing at all and only
/// max_distance still bites. Call it once, after every face and every skinned
/// constraint has been added.
///
/// Only a face whose three vertices are ALL skinned contributes a normal, so a
/// vertex on the boundary between a skinned region and a free one is given
/// fewer faces than meet at it, and one in a wholly unskinned neighbourhood is
/// given none.
///
/// Refused rather than left to Jolt's assert: more than 255 qualifying faces
/// meeting at one skinned vertex, which does not fit the 8-bit count Jolt
/// packs alongside the index (SoftBodySharedSettings.cpp:602).
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
// Everything above configures a soft body before it exists. This is the other
// half: the knobs Jolt keeps on the body's own SoftBodyMotionProperties, which
// a host changes while the simulation runs — stiffen the cloth this frame,
// inflate the balloon, unpin the corner the character just let go of.
//
// Every call below takes the system and a body id and resolves those motion
// properties under a lock, exactly as zjoltSoftBodyGetVertexStates does. A
// body id naming nothing is ZJOLT_RESULT_BODY_NOT_FOUND; a body id naming a
// RIGID body is ZJOLT_RESULT_INVALID_ARGUMENT. That second one is the reason
// these are entry points rather than a struct a caller fills in: a rigid
// body's MotionProperties is a sibling of a soft body's, not a parent, so
// casting one to the other is a mistake no compiler can see and no crash
// points back to.
//
// None of these wake a sleeping body. Change one on a body that has gone to
// sleep and it takes effect when something else wakes it.
//===----------------------------------------------------------------------===//

/// Solver iterations this body runs per step. @see ZJoltSoftBodyDesc for why
/// 0 is refused.
ZJOLT_API ZJoltResult zjoltSoftBodyGetNumIterations(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetNumIterations(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t num_iterations);

/// Internal gas pressure: n * R * T, amount of substance times the ideal gas
/// constant times absolute temperature. 0 disables it entirely.
///
/// The force it produces is computed from the volume the faces enclose, so it
/// means nothing on a body whose faces do not close a surface — a flat sheet
/// of cloth — and it pulls INWARD on one whose faces wind inside out.
/// zjoltSoftBodyGetVolume is what tells the two apart.
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
/// Jolt's own setter asserts on this — but on the value it is about to
/// OVERWRITE rather than the one being set (SoftBodyMotionProperties.h:102),
/// so upstream notices a bad radius one call late, or never, if the caller
/// sets it only once. The check here is on the incoming value, which is what
/// that assert was written to mean.
ZJOLT_API ZJoltResult zjoltSoftBodyGetVertexRadius(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, float *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetVertexRadius(ZJoltPhysicsSystem *system,
                                                   ZJoltBodyId body,
                                                   float vertex_radius);

//===----------------------------------------------------------------------===//
// One vertex at a time
//
// Velocity and inverse mass are the only two parts of a simulated vertex Jolt
// sanctions writing while the body runs (SoftBodyVertex.h:12). There is
// deliberately no setter for a position: writing one moves the vertex without
// moving the previous position the solver integrates from, and the step that
// follows misses every collision along the way. Move a soft body by moving
// the body, or by hard-skinning it.
//
// `index` is into the same array zjoltSoftBodyGetVertexStates reads, in the
// same order, and is checked against its length — Jolt indexes it unguarded.
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
/// constraint, but nothing moves it — which is how a cloth is nailed to a
/// flagpole, or to a hand.
///
/// This does NOT recompute the body's own mass and inertia, which Jolt derives
/// from the vertices once at creation. @see
/// zjoltSoftBodyCalculateMassAndInertia.
ZJOLT_API ZJoltResult zjoltSoftBodyGetVertexInvMass(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, uint32_t index,
    float *out);
ZJOLT_API ZJoltResult zjoltSoftBodySetVertexInvMass(ZJoltPhysicsSystem *system,
                                                    ZJoltBodyId body,
                                                    uint32_t index,
                                                    float inv_mass);

/// Recomputes the body's total mass and inertia from its vertices' current
/// inverse masses and positions. Jolt does this once, at creation, and never
/// again — so a body whose per-vertex inverse masses were changed keeps the
/// mass it was born with until this runs.
///
/// A single vertex with an inverse mass of 0 gives the WHOLE body infinite
/// mass and inertia (SoftBodyMotionProperties.cpp:48). That is upstream's
/// rule, not this binding's, and it is why pinning one corner of a cloth stops
/// the body as a whole from responding to an impulse.
ZJOLT_API ZJoltResult zjoltSoftBodyCalculateMassAndInertia(
    ZJoltPhysicsSystem *system, ZJoltBodyId body);

//===----------------------------------------------------------------------===//
// Cheap measurements
//===----------------------------------------------------------------------===//

/// The volume the body's faces enclose, in its own local space. One pass over
/// the faces, no allocation.
///
/// NEGATIVE when the faces wind inside out, and merely a number — not zero —
/// for a surface that does not close, such as a sheet of cloth. Jolt computes
/// its pressure force from exactly this quantity, so a body reporting a
/// negative volume is a body whose pressure is squeezing it rather than
/// inflating it.
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
// A cloth that follows a character is not simulated out of nothing. Each
// frame the host hands Jolt the skeleton's joint matrices; Jolt skins every
// vertex carrying a Skinned constraint the way a renderer would, and the
// solver is then allowed to pull the simulated vertex away from that skinned
// position by at most ZJoltSoftBodySkinned::max_distance.
//
// The authoring half is already above — AddInvBindMatrices,
// AddSkinnedConstraints and CalculateSkinnedConstraintNormals build the bind
// pose. This is the per-frame half, and the order within a frame is:
//
//   1. animate the skeleton;
//   2. zjoltSoftBodySkinVertices with this frame's joint matrices;
//   3. zjoltPhysicsSystemStep.
//
// Missing a frame is not an error. The skinned positions simply stay where the
// last call left them, and the cloth spends that step constrained to a pose
// the character has already left.
//===----------------------------------------------------------------------===//

/// Skins every vertex carrying a Skinned constraint to `joint_matrices`, and
/// records the result for the solver to constrain against on the next step.
///
/// `joint_matrices` is indexed by ZJoltSoftBodyInvBind::joint_index and holds
/// `joint_count` entries. Each matrix must be expressed RELATIVE TO THE BODY'S
/// CENTRE-OF-MASS TRANSFORM, not in world space: take the world joint matrix
/// and pre-multiply it by the inverse of zjoltBodyGetCenterOfMassTransform.
/// World-space matrices are not detectably wrong and nothing here fails — the
/// cloth simply skins to wherever the body's centre of mass sits relative to
/// the origin, which is the single most common way to get this call wrong.
///
/// `hard_skin_all` puts every skinned vertex exactly on its skinned position
/// and zeroes its velocity, ignoring max_distance entirely. That is a reset,
/// for the frame a character spawns or is teleported — not something to do
/// every frame, which would leave nothing for the solver to do.
///
/// `joint_matrices` may be NULL only when `joint_count` is 0, which in turn is
/// only usable on settings carrying no inverse bind matrices.
///
/// Refused rather than left to Jolt's asserts:
///
///   * an inverse bind matrix whose joint_index is >= `joint_count`
///     (SoftBodyMotionProperties.cpp:1150) — the one thing `joint_count` is
///     passed for;
///   * a skin weight whose inv_bind_index is past the settings' inverse bind
///     matrix list (:1173), which is the check
///     zjoltSoftBodySharedSettingsAddSkinnedConstraints could not make;
///   * shared settings carrying no skinned constraints at all, or carrying
///     vertices appended AFTER this body was created (:1157). Jolt sizes the
///     per-instance skinning state once, at creation, and only when there is
///     at least one skinned constraint to size it for.
///
/// The skinned positions and normals themselves stay inside Jolt: they are
/// private to SoftBodyMotionProperties with no accessor, so there is nothing
/// to read back here. What the skinning did is visible in the vertices
/// zjoltSoftBodyGetVertexStates reports after the next step.
ZJOLT_API ZJoltResult zjoltSoftBodySkinVertices(
    ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltMat44 *joint_matrices, uint32_t joint_count,
    bool hard_skin_all);

/// Whether the solver enforces this body's skin constraints at all.
///
/// Switching it off does not stop zjoltSoftBodySkinVertices from doing its
/// work: with the constraints off, that call hard-skins every vertex whose
/// max_distance is 0 and leaves the rest to simulate freely. So this is the
/// difference between "cloth that follows the character" and "cloth pinned at
/// its kinematic vertices and otherwise loose".
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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SOFTBODY_H_
