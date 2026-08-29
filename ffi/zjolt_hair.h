//===----------------------------------------------------------------------===//
// zjolt — hair: strand simulation, and the compute backend it runs on.
//
// Hair runs on JPH::ComputeSystem (compute shaders), not the CPU, and zjolt
// links no graphics SDK — so the interface is injectable
// (ZJoltComputeInterface) instead. No host backend? Use
// zjoltComputeSystemCreateCpu, Jolt's own CPU fallback (build option
// `cpu_compute`, on by default; off, it returns ZJOLT_RESULT_UNSUPPORTED —
// check zjoltComputeIsCpuSupported first). Declarations do not move with
// build options. Upstream calls this subsystem "still in development": no
// LOD, no wind, convex hulls only for collision.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_HAIR_H_
#define ZJOLT_HAIR_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Opaque handles
//===----------------------------------------------------------------------===//

/// A compute backend, plus the one queue and hair shader set that run on
/// it. Share one between every hair instance in a scene. The queue is part
/// of the handle because Jolt documents a ComputeQueue as usable from one
/// thread at a time — a second handle to hand to the wrong thread is a
/// hazard this ABI has no reason to offer.
typedef struct ZJoltComputeSystem ZJoltComputeSystem;

/// One head of hair: a groom, a world transform, and the buffers its solver
/// steps. Holds its own reference on the compute backend it was created with,
/// so the two can be destroyed in either order.
typedef struct ZJoltHair ZJoltHair;

//===----------------------------------------------------------------------===//
// The compute interface: what a host implements, taken from
// JPH::ComputeSystem. Every callback runs on whichever thread called into
// zjolt; nothing may unwind out of one — this library compiles without
// exceptions.
//===----------------------------------------------------------------------===//

/// How a buffer is used, mirroring JPH::ComputeBuffer::EType.
///
/// `size` in `create_buffer` is an ELEMENT COUNT, not a byte count — the
/// buffer holds `size * stride` bytes. Upstream itself over-allocates one
/// buffer by 64x with this convention (`Hair.cpp:52`); wasteful, not wrong,
/// but don't be surprised sizing device memory from these numbers.
typedef enum ZJoltComputeBufferType {
  /// Written on the CPU, then uploaded. Mapped for writing every frame.
  ZJOLT_COMPUTE_BUFFER_TYPE_UPLOAD = 0,
  /// Copied back from the device so the CPU can read it.
  ZJOLT_COMPUTE_BUFFER_TYPE_READBACK = 1,
  /// A small block of shader constants.
  ZJOLT_COMPUTE_BUFFER_TYPE_CONSTANT = 2,
  /// Read-only to a shader. Initialised at creation and immutable after.
  ZJOLT_COMPUTE_BUFFER_TYPE_READ_ONLY = 3,
  /// Read and written by a shader.
  ZJOLT_COMPUTE_BUFFER_TYPE_READ_WRITE = 4,
} ZJoltComputeBufferType;

/// Mirrors JPH::ComputeBuffer::EMode. WRITE discards whatever the buffer held.
typedef enum ZJoltComputeMapMode {
  ZJOLT_COMPUTE_MAP_MODE_READ = 0,
  ZJOLT_COMPUTE_MAP_MODE_WRITE = 1,
} ZJoltComputeMapMode;

/// Whether a barrier is needed before a read/write binding, mirroring
/// JPH::ComputeQueue::EBarrier. INSERT means earlier writes to that buffer must
/// be made visible first; SKIP means the caller knows they already are.
typedef enum ZJoltComputeBarrier {
  ZJOLT_COMPUTE_BARRIER_INSERT = 0,
  ZJOLT_COMPUTE_BARRIER_SKIP = 1,
} ZJoltComputeBarrier;

/// A host compute backend. Every field except `create_readback_buffer`,
/// `destroy` and `user` is REQUIRED — refused at zjoltComputeSystemCreate
/// rather than discovered mid-frame. The `void *` handles are the host's
/// own: zjolt never dereferences one, only stores and hands it back.
typedef struct ZJoltComputeInterface {
  /// Compiles or looks up a shader by name. The hair solver asks for all
  /// fifteen names in `HairShaders.cpp`; a table that cannot produce one is
  /// refused at creation, since Jolt's own loader would dereference a null
  /// shader later. Group sizes are the shader's own; `queue_dispatch`
  /// counts are in units of them.
  ZJoltResult (*create_shader)(void *user, const char *name,
                               uint32_t group_size_x, uint32_t group_size_y,
                               uint32_t group_size_z, void **out_shader);

  /// `size` is an element count; the buffer is `size * stride` bytes. `data`
  /// is NULL or `size * stride` bytes of initial contents, borrowed for the
  /// call. Report a failure honestly — upstream's own `Hair::Init` asserts
  /// rather than checking, so a failure here surfaces as a refused
  /// zjoltHairCreate.
  ZJoltResult (*create_buffer)(void *user, ZJoltComputeBufferType type,
                               uint64_t size, uint32_t stride, const void *data,
                               void **out_buffer);

  /// Optional. A CPU-readable twin of `buffer`, same size/stride, as the
  /// destination of `queue_schedule_readback`. NULL falls back to
  /// `create_buffer` with ZJOLT_COMPUTE_BUFFER_TYPE_READBACK. Must NOT
  /// return `buffer` itself — the two handles are destroyed independently,
  /// so aliasing them double-frees one allocation.
  ZJoltResult (*create_readback_buffer)(void *user, void *buffer,
                                        void **out_buffer);

  ZJoltResult (*create_queue)(void *user, void **out_queue);

  /// Called when the last reference goes away — not necessarily inside a
  /// zjolt call the host made. Never called twice for one handle, never NULL.
  void (*destroy_shader)(void *user, void *shader);
  void (*destroy_buffer)(void *user, void *buffer);
  void (*destroy_queue)(void *user, void *queue);

  /// Pointer to the buffer's `size * stride` bytes, or NULL on failure —
  /// which upstream WILL dereference, so a device that can fail here should
  /// refuse at creation instead. Unmap only ever pairs with a successful map.
  void *(*map_buffer)(void *user, void *buffer, ZJoltComputeMapMode mode);
  void (*unmap_buffer)(void *user, void *buffer);

  /// Recording. `name` is the buffer's name in the shader, null-terminated and
  /// borrowed for the call. A shader is bound before its buffers, and both must
  /// be bound again after every dispatch.
  void (*queue_set_shader)(void *user, void *queue, void *shader);
  void (*queue_set_constant_buffer)(void *user, void *queue, const char *name,
                                    void *buffer);
  void (*queue_set_buffer)(void *user, void *queue, const char *name,
                           void *buffer);
  void (*queue_set_rw_buffer)(void *user, void *queue, const char *name,
                              void *buffer, ZJoltComputeBarrier barrier);
  /// Thread-group counts, in units of the bound shader's group size.
  void (*queue_dispatch)(void *user, void *queue, uint32_t groups_x,
                         uint32_t groups_y, uint32_t groups_z);
  /// Copies `src` into `dst` once the work recorded before it has run.
  void (*queue_schedule_readback)(void *user, void *queue, void *dst,
                                  void *src);

  /// Submits everything recorded so far. Nothing more may be recorded until
  /// `queue_wait` has returned.
  void (*queue_execute)(void *user, void *queue);
  /// Blocks until the submitted work has finished AND every scheduled
  /// readback is visible to the CPU.
  void (*queue_wait)(void *user, void *queue);

  /// Optional. Called once, when the last reference to the backend goes away —
  /// after every shader, buffer and queue it created has been destroyed.
  void (*destroy)(void *user);

  /// Opaque host pointer, passed back to every callback above unmodified. It
  /// must outlive the ZJoltComputeSystem and every hair created on it.
  void *user;
} ZJoltComputeInterface;

//===----------------------------------------------------------------------===//
// Creating a compute backend
//===----------------------------------------------------------------------===//

/// Whether this build compiled Jolt's CPU compute path. False means
/// zjoltComputeSystemCreateCpu returns ZJOLT_RESULT_UNSUPPORTED and an injected
/// table is the only way to run hair.
ZJOLT_API bool zjoltComputeIsCpuSupported(void);

/// Jolt's own CPU implementation of the compute interface, with the hair
/// shaders registered. Upstream marks it for debugging, not optimised —
/// what makes hair testable with no graphics SDK in the build, not what to
/// ship a crowd on.
ZJOLT_API ZJoltResult zjoltComputeSystemCreateCpu(ZJoltComputeSystem **out);

/// A backend the host implements. `iface` is copied by value (need not
/// outlive the call), but `iface->user` and everything it names must
/// outlive the returned handle and every hair created on it. A table
/// missing a required shader fails here, with the name in zjoltLastError.
ZJOLT_API ZJoltResult zjoltComputeSystemCreate(
    const ZJoltComputeInterface *iface, ZJoltComputeSystem **out);

/// Drops this handle's references. The backend itself survives as long as any
/// hair created on it, so the destroy order of the two does not matter.
ZJOLT_API void zjoltComputeSystemDestroy(ZJoltComputeSystem *compute);

//===----------------------------------------------------------------------===//
// Describing a groom: Jolt's HairSettings (a reference-counted asset with
// compute buffers) does not cross this boundary — these parameters do, and
// zjolt builds the settings behind the handle.
//===----------------------------------------------------------------------===//

/// One vertex of one strand, in the hair's local space.
typedef struct ZJoltHairVertex {
  ZJoltVec3 position;
  /// 1 / mass. 0 pins the vertex; the root of a strand is usually held by the
  /// scalp rather than by this.
  float inv_mass;
} ZJoltHairVertex;

/// A run of vertices, half-open. `material_index` indexes ZJoltHairDesc's
/// materials. Refused: fewer than 2 vertices, or two consecutive vertices
/// coinciding (Jolt asserts building a Bishop frame from a zero-length
/// rod). Also refused: more than 255 vertices in a strand, or more than
/// 256 materials in a groom — Jolt packs both into a byte and silently
/// truncates past that, turning an overlong strand into an empty one.
typedef struct ZJoltHairStrand {
  uint32_t start_vertex;
  uint32_t end_vertex;
  uint32_t material_index;
} ZJoltHairStrand;

/// A value that varies along a strand: `min` at `min_fraction` of its length,
/// `max` at `max_fraction`, clamped outside that range.
typedef struct ZJoltHairGradient {
  float min;
  float max;
  float min_fraction;
  float max_fraction;
} ZJoltHairGradient;

/// How one joint influences one scalp vertex. There are exactly
/// `skin_weights_per_vertex` of these per scalp vertex, in vertex order.
typedef struct ZJoltHairSkinWeight {
  uint32_t joint_index;
  float weight;
} ZJoltHairSkinWeight;

/// The simulation parameters of a strand. Call zjoltHairMaterialInit first:
/// every field has a considered default, and a zeroed material is not a soft
/// one, it is an inert one.
typedef struct ZJoltHairMaterial {
  /// Collide the strands with the world. Only convex hulls are tested;
  /// upstream lists the other shape types as missing (`Hair.h:26`).
  bool enable_collision;
  /// Long-range attachments, which stop a strand stretching when the head moves
  /// quickly.
  bool enable_lra;
  float linear_damping;
  float angular_damping;
  float max_linear_velocity;
  float max_angular_velocity;
  /// 0 = weightless, 1 = full gravity, along the strand.
  ZJoltHairGradient gravity_factor;
  float friction;
  /// Compliance is 1 / stiffness, so smaller is stiffer.
  float bend_compliance;
  /// Multiplies `bend_compliance` at 0%, 33%, 66% and 100% of the strand.
  float bend_compliance_multiplier[4];
  float stretch_compliance;
  float inertia_multiplier;
  /// Strand radius along its length, used for collision.
  ZJoltHairGradient hair_radius;
  /// 0 = the hair moves rigidly with the head, 1 = it is fully simulated.
  ZJoltHairGradient world_transform_influence;
  /// Fraction of the grid velocity applied to a vertex each iteration.
  ZJoltHairGradient grid_velocity_factor;
  /// Pushes the hair back towards its neutral density. Defaults to 0: upstream
  /// notes it can produce artefacts.
  float grid_density_force_factor;
  /// Fraction of the neutral pose applied each iteration.
  ZJoltHairGradient global_pose;
  /// How closely the neutral pose follows the skin of the scalp.
  ZJoltHairGradient skin_global_pose;
  /// Fraction of this material's strands actually simulated; the rest
  /// interpolate from the nearest simulated one. At least one is always
  /// simulated.
  float simulation_strands_fraction;
  /// How much gravity to remove from the modelled pose so it does not sag on
  /// the first step. Upstream marks this as not fully functional.
  float gravity_preload_factor;
} ZJoltHairMaterial;

/// Fills `out` with Jolt's defaults. Does nothing if `out` is NULL.
ZJOLT_API void zjoltHairMaterialInit(ZJoltHairMaterial *out);

/// A groom plus where to put it. Everything is copied during
/// zjoltHairCreate; nothing here needs to outlive that call.
///
/// The scalp is optional and all-or-nothing: supply all four `scalp_*`/
/// `scalp_skin_weights`/`scalp_inverse_bind_pose` fields and roots skin to
/// it (hair follows a head), or none and roots stay fixed in local space.
typedef struct ZJoltHairDesc {
  const ZJoltHairVertex *vertices;
  const ZJoltHairStrand *strands;
  /// NULL uses one default material, as zjoltHairMaterialInit would fill it.
  const ZJoltHairMaterial *materials;
  uint32_t vertex_count;
  uint32_t strand_count;
  uint32_t material_count;

  const ZJoltVec3 *scalp_vertices;
  /// Three vertex indices per triangle, so `3 * scalp_triangle_count` of them.
  const uint32_t *scalp_triangles;
  /// `scalp_vertex_count * skin_weights_per_vertex` entries, in vertex order.
  const ZJoltHairSkinWeight *scalp_skin_weights;
  /// Sixteen floats per joint, column-major, matching zjoltHairSetPose.
  const float *scalp_inverse_bind_pose;
  uint32_t scalp_vertex_count;
  uint32_t scalp_triangle_count;
  uint32_t skin_weights_per_vertex;
  uint32_t joint_count;

  /// Gravity in the hair's local space, used once for the unloaded rest pose
  /// — not the simulation's gravity, which is the physics system's.
  ZJoltVec3 initial_gravity;
  /// Added on all sides of the neutral-pose bounds, sizing the velocity
  /// grid. All zero uses Jolt's 0.1 rather than literal zero — a flat
  /// neutral pose (e.g. one straight strand) would otherwise divide by
  /// zero into an all-NaN groom.
  ZJoltVec3 simulation_bounds_padding;
  /// Cells in the velocity/density grid. 0 in any axis uses Jolt's 32.
  uint32_t grid_size_x;
  uint32_t grid_size_y;
  uint32_t grid_size_z;
  /// Solver iterations per second of simulated time. 0 uses Jolt's 360.
  uint32_t iterations_per_second;
  /// The longest step the solver will take; a longer one is clamped, which
  /// slows the hair down rather than letting it explode. 0 uses Jolt's 1/30.
  float max_delta_time;

  ZJoltRVec3 position;
  ZJoltQuat rotation;
  /// The layer the strands collide against, read through the same object-layer
  /// filter the rest of the system uses.
  ZJoltObjectLayer object_layer;
} ZJoltHairDesc;

//===----------------------------------------------------------------------===//
// A hair instance
//===----------------------------------------------------------------------===//

/// Builds the groom, uploads it to `compute`, and places it at
/// `desc->position`/`desc->rotation`. Expensive: splits strands into
/// simulated/interpolated sets, computes rest frames, matches roots to the
/// scalp, allocates every compute buffer. Do it at load time.
ZJOLT_API ZJoltResult zjoltHairCreate(ZJoltComputeSystem *compute,
                                      const ZJoltHairDesc *desc,
                                      ZJoltHair **out);

ZJOLT_API void zjoltHairDestroy(ZJoltHair *hair);

/// Where the hair is in the world. The difference from the previous step is
/// what drives the hair's inertia, so moving it is a physical act — use
/// zjoltHairOnTeleported when it is not meant to be.
ZJOLT_API ZJoltResult zjoltHairSetTransform(ZJoltHair *hair,
                                            const ZJoltRVec3 *position,
                                            const ZJoltQuat *rotation);

/// Where the hair is, as last set — the matrix zjoltHairGetVertices'
/// LOCAL-space vertices are placed into world space by. Either out pointer
/// may be NULL. Cached on this side (JPH::Hair takes a transform but never
/// gives it back), so this reports what zjoltHairSetTransform or
/// zjoltHairFollowBody last set.
ZJOLT_API ZJoltResult zjoltHairGetTransform(const ZJoltHair *hair,
                                            ZJoltRVec3 *out_position,
                                            ZJoltQuat *out_rotation);

/// Takes the transform from a body, which is the usual way to hang hair on a
/// head. Equivalent to reading the body's position and rotation and calling
/// zjoltHairSetTransform, and returns ZJOLT_RESULT_BODY_NOT_FOUND when the id
/// names no body rather than silently placing the hair at the origin.
ZJOLT_API ZJoltResult zjoltHairFollowBody(ZJoltHair *hair,
                                          const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body);

/// The skeleton pose that skins the scalp, and therefore the roots.
/// `joint_to_hair` is one 4x4 column-major matrix (model space to hair
/// local space); `joint_matrices` is `joint_count` more, in model space —
/// `joint_count` must equal zjoltHairGetJointCount's report, since Jolt
/// indexes its inverse bind pose by it unchecked. Refused for a
/// scalp-less groom. Both arrays are copied.
ZJOLT_API ZJoltResult zjoltHairSetPose(ZJoltHair *hair,
                                       const float *joint_to_hair,
                                       const float *joint_matrices,
                                       uint32_t joint_count);

/// How many joints zjoltHairSetPose expects. 0 for a groom with no scalp.
ZJOLT_API ZJoltResult zjoltHairGetJointCount(const ZJoltHair *hair,
                                             uint32_t *out_count);

/// The next step puts the hair back in its default pose at rest, instead of
/// treating the change in transform since the last step as motion.
ZJOLT_API ZJoltResult zjoltHairOnTeleported(ZJoltHair *hair);

/// Steps the hair by `delta_time` and waits for the work to finish.
/// `system` supplies gravity and collision shapes, so call after
/// zjoltPhysicsSystemStep, not before. Synchronous — submits and blocks
/// until the device is done. Jolt clamps `delta_time` to the groom's
/// `max_delta_time` and divides the rest into whole iterations, so a very
/// small step may do nothing.
ZJOLT_API ZJoltResult zjoltHairUpdate(ZJoltHair *hair,
                                      ZJoltPhysicsSystem *system,
                                      float delta_time);

/// Copies simulated vertex positions, in the hair's LOCAL space, into
/// `out_positions` — the simulated subset only, in Jolt's own order, not
/// input order. For rendering, use zjoltHairReadBackRenderPositions.
///
/// Two-call protocol: NULL sizes; a short buffer fills as far as it goes,
/// reporting ZJOLT_RESULT_BUFFER_TOO_SMALL. Slow: a full device stall.
ZJOLT_API ZJoltResult zjoltHairReadBackPositions(ZJoltHair *hair,
                                                 ZJoltVec3 *out_positions,
                                                 uint32_t capacity,
                                                 uint32_t *out_count);

/// The same, for the render vertices — every strand, including the ones that
/// were interpolated rather than simulated. Same cost, same buffer protocol.
ZJOLT_API ZJoltResult zjoltHairReadBackRenderPositions(ZJoltHair *hair,
                                                       ZJoltVec3 *out_positions,
                                                       uint32_t capacity,
                                                       uint32_t *out_count);

/// The scalp mesh as the solver skinned it, in the hair's LOCAL space.
/// Same cost/protocol as the two calls above — the only way to check
/// whether roots are being carried where expected, since a root is a
/// barycentric point on one of these triangles. A scalp-less groom reports
/// a count of zero rather than failing.
ZJOLT_API ZJoltResult zjoltHairReadBackScalpVertices(ZJoltHair *hair,
                                                     ZJoltVec3 *out_positions,
                                                     uint32_t capacity,
                                                     uint32_t *out_count);

/// Everything the solver knows about one simulated vertex. `rotation` is
/// the vertex's Bishop frame (the rod starting there) — what orients a
/// strand's cross-section, since positions alone give a polyline with no
/// "which way a ribbon faces".
typedef struct ZJoltHairVertexState {
  /// In the hair's LOCAL space, as zjoltHairReadBackPositions reports it.
  ZJoltVec3 position;
  ZJoltQuat rotation;
  /// Local space per second.
  ZJoltVec3 velocity;
  ZJoltVec3 angular_velocity;
} ZJoltHairVertexState;

/// Position, orientation, velocity and angular velocity of every simulated
/// vertex, in ONE device stall. Same ordering/protocol as
/// zjoltHairReadBackPositions. Prefer this over that call plus wanting
/// velocities too — every readback copies the whole simulation state back,
/// so two calls cost two stalls for the same bytes.
ZJOLT_API ZJoltResult zjoltHairReadBackVertexState(
    ZJoltHair *hair, ZJoltHairVertexState *out_state, uint32_t capacity,
    uint32_t *out_count);

/// Where each simulated strand starts/ends in the arrays
/// zjoltHairReadBackPositions/VertexState fill, and which material it uses
/// — those arrays are otherwise a flat run with no strand boundaries.
/// RENDER positions need no such call: they stay in the caller's own
/// indexing. Two-call protocol; cheap — reads the groom, not the device.
ZJOLT_API ZJoltResult zjoltHairGetSimulatedStrands(const ZJoltHair *hair,
                                                   ZJoltHairStrand *out_strands,
                                                   uint32_t capacity,
                                                   uint32_t *out_count);

/// What zjoltHairCreate made of a groom — the parts Jolt derived that a
/// caller cannot recompute from what it passed in (Jolt's HairSettings
/// itself does not cross this boundary).
typedef struct ZJoltHairInfo {
  /// Vertices and strands the solver simulates — a subset, regrouped.
  uint32_t simulated_vertex_count;
  uint32_t simulated_strand_count;
  /// Every authored vertex and strand, in the order they were passed in.
  uint32_t render_vertex_count;
  uint32_t render_strand_count;
  uint32_t material_count;
  /// As zjoltHairGetJointCount reports it; 0 for a groom with no scalp.
  uint32_t joint_count;
  /// Vertices in the longest simulated strand.
  uint32_t max_vertices_per_strand;
  /// `simulated_strand_count * max_vertices_per_strand` — the device
  /// buffers' row stride, for a host reading them directly. Larger than
  /// `simulated_vertex_count` unless all strands are one length.
  uint32_t padded_vertex_count;
  /// The velocity grid's box, in local space — the neutral pose plus
  /// `simulation_bounds_padding`; an escaping vertex clamps to the edge cell.
  ZJoltVec3 simulation_bounds_min;
  ZJoltVec3 simulation_bounds_max;
  /// Cells in that box, after the zeroes in ZJoltHairDesc became Jolt's 32.
  uint32_t grid_size_x;
  uint32_t grid_size_y;
  uint32_t grid_size_z;
  /// Worst-matched strand root's distance from the scalp mesh, in local
  /// units, at build time — Jolt projects every root onto the closest
  /// triangle, so a mismatch silently relocates a root rather than
  /// failing. 0 for a scalp-less groom, or one restored from a blob
  /// predating this field.
  float max_root_distance_to_scalp;
} ZJoltHairInfo;

/// Fills `out` with what the groom turned into. Cheap; nothing here touches the
/// device.
ZJOLT_API ZJoltResult zjoltHairGetInfo(const ZJoltHair *hair,
                                       ZJoltHairInfo *out);

//===----------------------------------------------------------------------===//
// The raw buffer window: what Lock/UnlockReadBackBuffers guard, zero-copy.
// The copy-out calls above already cover the vertex data; reach for this for
// the velocity/density grid, which they do not expose at all.
//===----------------------------------------------------------------------===//

/// The grid zjoltHairUpdate fills for materials with a nonzero
/// grid_velocity_factor or grid_density_force_factor: velocity in the
/// hair's LOCAL space per second, and how crowded that cell is relative to
/// the rest pose. Bit-identical layout to Jolt's own Float4.
typedef struct ZJoltHairGridCell {
  ZJoltVec3 velocity;
  float density;
} ZJoltHairGridCell;

/// Pointers into `hair`'s locked buffers, valid only until
/// zjoltHairUnlockReadBackBuffers. `scalp_vertices` is NULL for a
/// scalp-less groom. Counts: zjoltHairGetInfo's render_vertex_count and
/// grid_size_x * grid_size_y * grid_size_z (x fastest, then y, then z);
/// zjoltHairReadBackScalpVertices's count for the scalp.
typedef struct ZJoltHairReadBackView {
  const ZJoltVec3 *scalp_vertices;
  const ZJoltHairGridCell *grid_velocity_and_density;
  const ZJoltVec3 *render_positions;
} ZJoltHairReadBackView;

/// Reads the device state back and locks the buffers `out` points into.
/// The window stays open until zjoltHairUnlockReadBackBuffers: every
/// pointer in `out` is invalid the moment that returns, and locking again
/// before unlocking is refused. Slow: a full device stall.
ZJOLT_API ZJoltResult zjoltHairLockReadBackBuffers(
    ZJoltHair *hair, ZJoltHairReadBackView *out);

/// Ends the window zjoltHairLockReadBackBuffers opened. Refused without a
/// matching lock still open.
ZJOLT_API ZJoltResult zjoltHairUnlockReadBackBuffers(ZJoltHair *hair);

/// The rest-pose density HairSettings::Init computed at grid cell (x, y,
/// z) — how much of a strand's own weight already sits there before
/// simulation runs. Same grid as zjoltHairGetInfo's grid_size_*; no device
/// access. Refused if any coordinate is out of range.
ZJOLT_API ZJoltResult zjoltHairGetNeutralDensity(const ZJoltHair *hair,
                                                 uint32_t x, uint32_t y,
                                                 uint32_t z,
                                                 float *out_density);

/// The velocity/density grid cell containing `position` (hair LOCAL
/// space), and its blend fraction toward the next cell on each axis — what
/// trilinearly interpolating zjoltHairLockReadBackBuffers' grid needs.
/// `position` outside zjoltHairGetInfo's simulation_bounds clamps to the
/// nearest edge cell rather than failing.
ZJOLT_API ZJoltResult zjoltHairPositionToGridIndex(
    const ZJoltHair *hair, const ZJoltVec3 *position, uint32_t *out_index_x,
    uint32_t *out_index_y, uint32_t *out_index_z, ZJoltVec3 *out_fraction);

//===----------------------------------------------------------------------===//
// Skinning the scalp off the solver: the same skeleton math zjoltHairUpdate
// runs on the device, run once on the CPU with no compute backend touched —
// for previewing a pose, or animating a scalp that never simulates.
//===----------------------------------------------------------------------===//

/// Skins the scalp mesh to `joint_matrices`. Same convention as
/// zjoltHairSetPose: `joint_to_hair` model space to hair local space,
/// `joint_count` must equal zjoltHairGetJointCount. Refused for a
/// scalp-less groom. Two-call protocol, as zjoltHairReadBackPositions; no
/// device access.
ZJOLT_API ZJoltResult zjoltHairSkinScalpVertices(
    const ZJoltHair *hair, const float *joint_to_hair,
    const float *joint_matrices, uint32_t joint_count,
    ZJoltVec3 *out_vertices, uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Baking a groom: zjoltHairCreate's expensive work (strand split, scalp
// matching, Bishop frames, voxelisation) does not depend on the compute
// backend, so a cook can do it once and ship the result — treat the result
// as something your own cook wrote, not untrusted input.
//===----------------------------------------------------------------------===//

/// Writes the built groom into `buffer`. Two-call protocol: `buffer` NULL
/// sizes into `*out_size`; a short `capacity` reports
/// ZJOLT_RESULT_BUFFER_TOO_SMALL with the size still written.
///
/// Saves the GROOM, not the simulation: strands, materials, rest frames,
/// density grid. Not position or pose — a restored groom starts at rest.
ZJOLT_API ZJoltResult zjoltHairSaveGroom(const ZJoltHair *hair, void *buffer,
                                         size_t capacity, size_t *out_size);

/// Builds a hair from zjoltHairSaveGroom output, skipping the strand
/// split, root matching and density grid — only compute buffers are
/// allocated, the cheap way in. `position`/`rotation` may each be NULL
/// (origin/identity). ZJOLT_RESULT_BAD_FORMAT for a mismatched, truncated,
/// or damaged blob.
ZJOLT_API ZJoltResult zjoltHairCreateFromGroom(
    ZJoltComputeSystem *compute, const void *data, size_t size,
    const ZJoltRVec3 *position, const ZJoltQuat *rotation,
    ZJoltObjectLayer object_layer, ZJoltHair **out);

//===----------------------------------------------------------------------===//
// Evaluating the authored parameters: two pure functions doing what the
// solver does to a gradient/compliance curve, since re-deriving either by
// hand gets the clamping wrong.
//===----------------------------------------------------------------------===//

/// The gradient's value at `strand_fraction` along a strand (0 root, 1
/// tip). Clamped, not extrapolated, outside [`min_fraction`,
/// `max_fraction`]; NaN in `strand_fraction` is refused.
///
/// Refused if `min_fraction == max_fraction` (Jolt divides by zero,
/// corrupting the shader constants); zjoltHairCreate refuses one too.
ZJOLT_API ZJoltResult zjoltHairGradientSample(const ZJoltHairGradient *gradient,
                                              float strand_fraction,
                                              float *out_value);

/// The material's bend compliance at `strand_fraction` along a strand:
/// `bend_compliance` scaled by `bend_compliance_multiplier` interpolated
/// across its four entries. `strand_fraction` must be in [0, 1] — Jolt's
/// own version truncates a negative to an unsigned index (UB) and asserts
/// above 1.
ZJOLT_API ZJoltResult zjoltHairMaterialGetBendCompliance(
    const ZJoltHairMaterial *material, float strand_fraction,
    float *out_value);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_HAIR_H_
