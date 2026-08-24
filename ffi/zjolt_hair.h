//===----------------------------------------------------------------------===//
// zjolt — hair: strand simulation, and the compute backend it runs on.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// Hair is the one Jolt subsystem that does not run on the CPU by default. Its
// solver is written as compute shaders and driven through JPH::ComputeSystem,
// an abstract interface Jolt implements over Direct3D 12, Vulkan and Metal.
// zjolt compiles none of those: a physics package that needs a graphics SDK to
// build is a graphics package, and this one promises no renderer.
//
// So that interface is bound as an injectable table instead —
// ZJoltComputeInterface below. A host that already owns a device fills it in
// and gets hair on the GPU with zjolt still linking against nothing; a host
// that does not calls zjoltComputeSystemCreateCpu and gets Jolt's own CPU
// fallback, which zjolt DOES compile (build option `cpu_compute`, on by
// default). Both arrive as the same ZJoltComputeSystem handle, and nothing
// downstream of it can tell the difference.
//
// As everywhere in this ABI, the declarations below do not move with the build
// options. With `cpu_compute` off, zjoltComputeSystemCreateCpu still exists and
// returns ZJOLT_RESULT_UNSUPPORTED; zjoltComputeIsCpuSupported is how to ask
// before calling. An injected table works in every build.
//
// Upstream calls the hair system "still in development" and lists what it is
// missing (`Hair.h:20`) — no level of detail, no wind, convex hulls only for
// collision. That is Jolt's assessment of its own subsystem and it is repeated
// here rather than smoothed over.
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

/// A compute backend, plus the one queue and the hair shader set that run on
/// it. Share one between every hair instance in a scene.
///
/// The queue is part of the handle rather than a thing of its own because Jolt
/// documents a ComputeQueue as usable from one thread at a time
/// (`ComputeQueue.h:21`), and a second handle to hand to the wrong thread is a
/// hazard this ABI has no reason to offer.
typedef struct ZJoltComputeSystem ZJoltComputeSystem;

/// One head of hair: a groom, a world transform, and the buffers its solver
/// steps. Holds its own reference on the compute backend it was created with,
/// so the two can be destroyed in either order.
typedef struct ZJoltHair ZJoltHair;

//===----------------------------------------------------------------------===//
// The compute interface
//
// What a host has to be able to express, taken from JPH::ComputeSystem and the
// three interfaces it hands back. A table that cannot express one of them is
// worse than no table, so all of them are here:
//
//   creation        shaders, buffers and a queue, each yielding an opaque host
//                   handle that this ABI stores and hands straight back;
//   data            map/unmap, which is how the solver's input reaches the
//                   device and its output comes home;
//   dispatch        bind a shader, bind buffers by shader-side name, dispatch,
//                   schedule a readback;
//   synchronisation `queue_execute` submits, `queue_wait` blocks until the
//                   submitted work — readbacks included — has landed;
//   lifetime        one destroy per create, called when Jolt drops the last
//                   reference. zjolt holds the buffers a queue was given alive
//                   until `queue_wait` returns, which is the contract Jolt's
//                   own backends implement, so a host need not.
//
// Every callback is invoked from whichever thread called into zjolt. NOTHING
// MAY UNWIND OUT OF ONE: this library compiles without exceptions, so a C++
// exception or a Zig panic crossing one is std::terminate at best.
//===----------------------------------------------------------------------===//

/// How a buffer is used, mirroring JPH::ComputeBuffer::EType.
///
/// `size` in `create_buffer` is an ELEMENT COUNT, not a byte count: the buffer
/// holds `size * stride` bytes. Worth stating because upstream does not always
/// obey it — `Hair.cpp:52` asks for `joints * sizeof(Mat44)` elements of stride
/// `sizeof(Mat44)`, over-allocating that one buffer by a factor of 64. Wasteful
/// rather than wrong, but a host sizing device memory from these numbers should
/// not be surprised by it.
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
/// `destroy` and `user` is REQUIRED; zjoltComputeSystemCreate refuses a table
/// with a hole in it rather than discovering it mid-frame.
///
/// The `void *` handles are the host's own. zjolt never dereferences one — it
/// stores it, hands it back to the callbacks that take one, and passes it to
/// the matching destroy exactly once.
typedef struct ZJoltComputeInterface {
  /// Compiles or looks up a shader by name. The hair solver asks for fifteen,
  /// by the names in `HairShaders.cpp`: "HairTeleport",
  /// "HairApplyDeltaTransform", "HairSkinVertices", "HairSkinRoots",
  /// "HairApplyGlobalPose", "HairCalculateCollisionPlanes", "HairGridClear",
  /// "HairGridAccumulate", "HairGridNormalize", "HairIntegrate",
  /// "HairUpdateRoots", "HairUpdateStrands", "HairUpdateVelocity",
  /// "HairUpdateVelocityIntegrate" and "HairCalculateRenderPositions". A table
  /// that cannot produce all fifteen is refused at creation, because Jolt's own
  /// loader keeps a null shader and dereferences it later.
  ///
  /// The group sizes are the shader's own, and the counts passed to
  /// `queue_dispatch` are in units of them.
  ZJoltResult (*create_shader)(void *user, const char *name,
                               uint32_t group_size_x, uint32_t group_size_y,
                               uint32_t group_size_z, void **out_shader);

  /// `size` is an element count; the buffer is `size * stride` bytes. `data` is
  /// NULL or `size * stride` bytes of initial contents, borrowed for the
  /// duration of the call.
  ///
  /// Upstream does not check this for failure — `Hair::Init` calls `.Get()` on
  /// the result, which asserts rather than returning. Report a failure honestly
  /// anyway; it surfaces as a refused zjoltHairCreate.
  ZJoltResult (*create_buffer)(void *user, ZJoltComputeBufferType type,
                               uint64_t size, uint32_t stride, const void *data,
                               void **out_buffer);

  /// Optional. A CPU-readable twin of `buffer`, same size and stride, to be the
  /// destination of `queue_schedule_readback`. NULL falls back to
  /// `create_buffer` with ZJOLT_COMPUTE_BUFFER_TYPE_READBACK and no data.
  ///
  /// It must NOT return `buffer` itself, however tempting on a unified-memory
  /// device: the two handles are destroyed independently, so aliasing them
  /// means two destroys of one allocation.
  ZJoltResult (*create_readback_buffer)(void *user, void *buffer,
                                        void **out_buffer);

  ZJoltResult (*create_queue)(void *user, void **out_queue);

  /// Called when the last reference to the object goes away, which is not
  /// necessarily inside a zjolt call the host made. Never called twice for one
  /// handle, and never with NULL.
  void (*destroy_shader)(void *user, void *shader);
  void (*destroy_buffer)(void *user, void *buffer);
  void (*destroy_queue)(void *user, void *queue);

  /// Returns a pointer to the buffer's `size * stride` bytes, or NULL on
  /// failure — which upstream will dereference, so a device that can fail here
  /// should refuse at creation instead. Unmap is only ever paired with a
  /// successful map.
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
  /// Blocks until the submitted work has finished AND every scheduled readback
  /// is visible to the CPU.
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
/// shaders registered on it.
///
/// Upstream describes it as being for debugging and explicitly not optimised
/// (`ComputeSystem.h:47`). It is what makes hair testable and usable with no
/// graphics SDK anywhere in the build; it is not what to ship a crowd on.
ZJOLT_API ZJoltResult zjoltComputeSystemCreateCpu(ZJoltComputeSystem **out);

/// A backend the host implements. `iface` is copied by value, so it need not
/// outlive the call — but `iface->user` and everything it names must outlive
/// the returned handle and every hair created on it.
///
/// The fifteen hair shaders are loaded here, so a table that cannot produce one
/// of them fails at this call, with the name in zjoltLastError, rather than as
/// a null dereference during the first step.
ZJOLT_API ZJoltResult zjoltComputeSystemCreate(
    const ZJoltComputeInterface *iface, ZJoltComputeSystem **out);

/// Drops this handle's references. The backend itself survives as long as any
/// hair created on it, so the destroy order of the two does not matter.
ZJOLT_API void zjoltComputeSystemDestroy(ZJoltComputeSystem *compute);

//===----------------------------------------------------------------------===//
// Describing a groom
//
// Jolt's HairSettings is a reference-counted asset with compute buffers in it,
// not a serialisation record, and it does not cross this boundary — the
// parameters do, and zjolt builds the settings behind the handle. A host has
// its own groom format; this is the shape it converts into.
//===----------------------------------------------------------------------===//

/// One vertex of one strand, in the hair's local space.
typedef struct ZJoltHairVertex {
  ZJoltVec3 position;
  /// 1 / mass. 0 pins the vertex; the root of a strand is usually held by the
  /// scalp rather than by this.
  float inv_mass;
} ZJoltHairVertex;

/// A run of vertices, half-open. `material_index` indexes ZJoltHairDesc's
/// materials.
///
/// A strand needs at least two vertices, and no two consecutive vertices may
/// coincide: Jolt builds a Bishop frame from each rod's direction and asserts
/// "Rods of zero length are not supported!" (`HairSettings.cpp:510,532`),
/// dividing by zero in a build without asserts. Both are refused here.
///
/// A strand may have at most 255 vertices, and a groom at most 256 materials.
/// Jolt packs both a strand's vertex count and its material index into a byte
/// (`HairSettings.cpp:666,676`); past those limits it truncates rather than
/// failing, which turns a 256-vertex strand into an empty one. Refused here too.
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
  /// Fraction of this material's strands that are actually simulated; the rest
  /// are interpolated from the nearest simulated one. At least one strand per
  /// material is always simulated.
  float simulation_strands_fraction;
  /// How much gravity to remove from the modelled pose so it does not sag on
  /// the first step. Upstream marks this as not fully functional.
  float gravity_preload_factor;
} ZJoltHairMaterial;

/// Fills `out` with Jolt's defaults. Does nothing if `out` is NULL.
ZJOLT_API void zjoltHairMaterialInit(ZJoltHairMaterial *out);

/// A groom plus where to put it. Everything is copied during zjoltHairCreate;
/// nothing here needs to outlive that call.
///
/// The scalp is optional and all-or-nothing. Supply `scalp_vertices`,
/// `scalp_triangles`, `scalp_skin_weights` and `scalp_inverse_bind_pose`
/// together and the roots of the strands are skinned to it, which is what makes
/// hair follow a head; supply none of them and the roots are fixed in the
/// hair's local space and the whole groom moves with zjoltHairSetTransform.
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

  /// Gravity in the hair's local space, used once to work out the unloaded rest
  /// pose. Not the gravity the simulation runs under — that is the physics
  /// system's.
  ZJoltVec3 initial_gravity;
  /// Added on all sides of the bounds computed from the neutral pose, so the
  /// velocity grid is large enough to hold the hair once it moves. All zero
  /// uses Jolt's 0.1 rather than being taken literally: the grid is scaled by
  /// `grid size / bounds extent`, and a groom whose neutral pose is flat on one
  /// axis — a single straight strand is — would otherwise divide by zero and
  /// produce a groom made entirely of NaN.
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
/// `desc->position` / `desc->rotation`.
///
/// This is the expensive call: it splits the strands into simulated and
/// interpolated sets, computes rest frames, matches roots to the scalp and
/// allocates every compute buffer. Do it at load time.
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

/// Takes the transform from a body, which is the usual way to hang hair on a
/// head. Equivalent to reading the body's position and rotation and calling
/// zjoltHairSetTransform, and returns ZJOLT_RESULT_BODY_NOT_FOUND when the id
/// names no body rather than silently placing the hair at the origin.
ZJOLT_API ZJoltResult zjoltHairFollowBody(ZJoltHair *hair,
                                          const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body);

/// The skeleton pose that skins the scalp, and therefore the roots.
///
/// `joint_to_hair` is one 4x4 matrix, column-major, taking model space to the
/// hair's local space. `joint_matrices` is `joint_count` more of the same, the
/// joints in model space, and `joint_count` must equal the count reported by
/// zjoltHairGetJointCount — Jolt indexes its inverse bind pose by it with no
/// bound of its own.
///
/// A groom with no scalp has no joints, and calling this on one is refused.
/// Both arrays are copied, so neither needs to outlive the call.
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
///
/// `system` is read for gravity and for the shapes the strands collide with, so
/// this belongs after zjoltPhysicsSystemStep in a frame, not before. The step
/// is synchronous: it submits the recorded work and blocks until the device is
/// done with it, because the alternative is a fence in the ABI that nothing
/// else here needs.
///
/// Jolt clamps `delta_time` to the groom's `max_delta_time` and divides what is
/// left into whole solver iterations, so a very small step may do nothing.
ZJOLT_API ZJoltResult zjoltHairUpdate(ZJoltHair *hair,
                                      ZJoltPhysicsSystem *system,
                                      float delta_time);

/// Copies the simulated vertex positions, in the hair's LOCAL space, into
/// `out_positions`.
///
/// These are the vertices of the simulated strands only — the subset chosen by
/// each material's `simulation_strands_fraction` — in the order Jolt assigned
/// them, which is not the order they were passed in. For rendering, use
/// zjoltHairReadBackRenderPositions.
///
/// Pass NULL for `out_positions` to learn the count. A buffer smaller than the
/// count is filled as far as it goes and reports ZJOLT_RESULT_BUFFER_TOO_SMALL
/// with the required count in `*out_count`.
///
/// Upstream is blunt about the cost: this copies the whole simulation state
/// back from the device and is "for debugging purposes only, this is slow!"
/// (`Hair.h:98`). On a GPU backend it is a full stall.
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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_HAIR_H_
