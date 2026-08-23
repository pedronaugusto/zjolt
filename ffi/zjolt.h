//===----------------------------------------------------------------------===//
// zjolt — a C ABI over Jolt Physics.
//
// This header is the ONLY contract between the C++ implementation and any
// consumer (the Zig wrapper in ../src, or a plain C host). It is deliberately
// free of C++: opaque handles, POD structs with fixed layout, plain function
// pointers, and a flat error enum. No exceptions cross this boundary.
//
// Ownership rules, uniformly:
//   *Create        allocate through the installed allocator and yield a handle
//                  the caller owns.
//   *Destroy       accept NULL and are safe to call once.
//   Shapes         are reference counted: zjoltShapeAddRef / zjoltShapeRelease.
//                  Everything else is a plain owning handle.
//   Query results  borrow nothing; every out-parameter is caller-owned storage.
//   Callbacks      receive pointers valid only for the duration of the call.
//
// Aggregates are always passed and returned through pointers, never by value.
// That is deliberate: struct-return conventions differ between the SysV and
// Microsoft ABIs, and a binding that gets it wrong is silently corrupt rather
// than broken at the link step. Scalars are passed by value.
//
// Thread safety: a ZJoltPhysicsSystem may be stepped from one thread at a
// time. Contact and activation callbacks are invoked from Jolt's job threads,
// concurrently, DURING zjoltPhysicsSystemStep — see the note above
// ZJoltContactListener. Body reads outside a step should go through a body
// lock.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_H_
#define ZJOLT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(_MSC_VER) && defined(ZJOLT_SHARED)
#ifdef ZJOLT_BUILD
#define ZJOLT_API __declspec(dllexport)
#else
#define ZJOLT_API __declspec(dllimport)
#endif
#else
#define ZJOLT_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Version and build configuration
//
// Two build options change the LAYOUT of types in this header, so a consumer
// that compiles against different settings than the library was built with
// would misread every position it is handed. Rather than trust that not to
// happen, the settings are folded into ZJOLT_CONFIG_ID, which zjoltInit
// compares against the library's own — a skew is a clean error at start-up
// instead of silent corruption. This mirrors the trick Jolt plays on its own
// clients with JPH_VERSION_ID, for the same reason.
//===----------------------------------------------------------------------===//

#define ZJOLT_VERSION_MAJOR 0
#define ZJOLT_VERSION_MINOR 1
#define ZJOLT_VERSION_PATCH 0

/// Defined by the build when world positions are doubles. Must match the
/// setting the library was compiled with.
#ifdef ZJOLT_DOUBLE_PRECISION
#define ZJOLT_CONFIG_BIT_DOUBLE_PRECISION 1u
#else
#define ZJOLT_CONFIG_BIT_DOUBLE_PRECISION 0u
#endif

/// Width of ZJoltObjectLayer, 16 or 32. Must match the library.
#ifndef ZJOLT_OBJECT_LAYER_BITS
#define ZJOLT_OBJECT_LAYER_BITS 16
#endif
#if ZJOLT_OBJECT_LAYER_BITS == 32
#define ZJOLT_CONFIG_BIT_OBJECT_LAYER_32 1u
#elif ZJOLT_OBJECT_LAYER_BITS == 16
#define ZJOLT_CONFIG_BIT_OBJECT_LAYER_32 0u
#else
#error "ZJOLT_OBJECT_LAYER_BITS must be 16 or 32"
#endif

/// Version and layout-affecting build settings, packed. Passed to the library
/// by the zjoltInit wrapper below.
#define ZJOLT_CONFIG_ID                                       \
  (((uint32_t)ZJOLT_VERSION_MAJOR << 16) |                    \
   ((uint32_t)ZJOLT_VERSION_MINOR << 8) |                     \
   ((uint32_t)ZJOLT_VERSION_PATCH) |                          \
   ((uint32_t)ZJOLT_CONFIG_BIT_DOUBLE_PRECISION << 24) |      \
   ((uint32_t)ZJOLT_CONFIG_BIT_OBJECT_LAYER_32 << 25))

/// Version of the zjolt binding, packed as (major<<16)|(minor<<8)|patch.
ZJOLT_API uint32_t zjoltVersion(void);

/// Version of the vendored Jolt Physics library, same packing.
ZJOLT_API uint32_t zjoltJoltVersion(void);

/// The library's own ZJOLT_CONFIG_ID. A consumer whose header disagrees is
/// reading a different set of types than the library writes.
ZJOLT_API uint32_t zjoltConfigId(void);

//===----------------------------------------------------------------------===//
// Results
//===----------------------------------------------------------------------===//

typedef enum ZJoltResult {
  ZJOLT_OK = 0,
  /// A call was made before zjoltInit, or after zjoltDeinit.
  ZJOLT_ERR_NOT_INITIALIZED = 1,
  /// zjoltInit was called twice without an intervening zjoltDeinit.
  ZJOLT_ERR_ALREADY_INITIALIZED = 2,
  /// The consumer's ZJOLT_CONFIG_ID does not match the library's.
  ZJOLT_ERR_CONFIG_MISMATCH = 3,
  /// The installed allocator returned NULL.
  ZJOLT_ERR_OUT_OF_MEMORY = 4,
  /// A NULL handle, a NULL required callback, or an out-of-domain scalar.
  ZJOLT_ERR_INVALID_ARGUMENT = 5,
  /// A caller-provided output buffer was too small; the required count is
  /// still written to the out-count parameter.
  ZJOLT_ERR_BUFFER_TOO_SMALL = 6,
  /// Jolt refused to build the shape. zjoltLastError has its message.
  ZJOLT_ERR_SHAPE_INVALID = 7,
  /// A serialised shape could not be restored: truncated, or not a shape.
  ZJOLT_ERR_BAD_FORMAT = 8,
  /// The body id does not name a body in this system (it may have been
  /// destroyed).
  ZJOLT_ERR_BODY_NOT_FOUND = 9,
} ZJoltResult;

/// Static, never-NULL description of a result code. Borrowed; do not free.
ZJOLT_API const char *zjoltResultName(ZJoltResult result);

/// Detail for the most recent failure ON THIS THREAD, or "" if there is none.
///
/// Jolt reports shape-construction and deserialisation problems as strings
/// ("Failed to create hull", "Trying to restore a shape of unknown type"), and
/// a flat enum would throw them away. The buffer is thread-local, borrowed,
/// and overwritten by the next failing call on the same thread.
ZJOLT_API const char *zjoltLastError(void);

//===----------------------------------------------------------------------===//
// Initialisation
//===----------------------------------------------------------------------===//

/// Host allocator, installed process-wide.
///
/// Jolt routes every allocation through five global function pointers, and
/// this exposes that seam verbatim rather than hiding it, so a host can put
/// physics memory in its own budget.
///
/// Note the asymmetry inherited from Jolt: `free` and `aligned_free` receive
/// only the block pointer — no size, no alignment — while `reallocate` does
/// get the old size. An allocator that needs the missing information (Zig's
/// std.mem.Allocator does) must record it in a header of its own; the Zig
/// wrapper in ../src/memory.zig does exactly that.
typedef struct ZJoltAllocator {
  /// Returns at least `size` bytes, aligned to at least
  /// zjoltDefaultAllocateAlignment(), or NULL. `size` is never 0.
  void *(*allocate)(void *user, size_t size);
  /// Grows or shrinks a block from `allocate`. `block` may be NULL, in which
  /// case this behaves as `allocate`. `new_size` is never 0.
  void *(*reallocate)(void *user, void *block, size_t old_size, size_t new_size);
  /// Frees a block from `allocate` or `reallocate`. Must tolerate NULL.
  void (*free)(void *user, void *block);
  /// Returns at least `size` bytes aligned to `alignment` (a power of two),
  /// or NULL.
  void *(*aligned_allocate)(void *user, size_t size, size_t alignment);
  /// Frees a block from `aligned_allocate`. Must tolerate NULL.
  void (*aligned_free)(void *user, void *block);
  /// Opaque host pointer, passed back unmodified.
  void *user;
} ZJoltAllocator;

/// Minimum alignment `ZJoltAllocator::allocate` must satisfy. Jolt places
/// SIMD types in unaligned-allocated memory and relies on this, so it is not
/// advisory. Read it rather than assuming 16.
ZJOLT_API size_t zjoltDefaultAllocateAlignment(void);

/// Receives Jolt's diagnostic output, already formatted. Jolt's own hook is a
/// varargs function; the formatting is done on the C++ side so that no host
/// has to model a C varargs callback.
///
/// Supplying one is optional but recommended. Without it the messages go to
/// stderr — which is zjolt's fallback, not Jolt's: Jolt's own default trace
/// function is a stub whose body is an assertion, so an unhooked Jolt aborts
/// the first time it has something to report.
typedef void (*ZJoltTraceFn)(void *user, const char *message);

/// Called when a Jolt assertion fails. Return true to trigger a debugger
/// breakpoint. `message` may be NULL. Only reachable when the library was
/// built with asserts enabled.
typedef bool (*ZJoltAssertFailedFn)(void *user, const char *expression,
                                    const char *message, const char *file,
                                    uint32_t line);

/// Copied by value during zjoltInit, so neither this struct nor the
/// ZJoltAllocator it points at needs to outlive the call. The allocator's
/// `user` pointer, and anything `hooks_user` refers to, must outlive
/// zjoltDeinit.
typedef struct ZJoltInitDesc {
  /// NULL installs Jolt's default malloc/free allocator.
  const ZJoltAllocator *allocator;
  /// NULL sends Jolt's diagnostics to stderr. @see ZJoltTraceFn.
  ZJoltTraceFn trace;
  /// NULL leaves Jolt's default, which triggers a debugger breakpoint — the
  /// right behaviour for a game, and fatal in a build without a debugger
  /// attached. Every assertion zjolt found to be reachable through this ABI
  /// has been turned into a returned error instead; see UPSTREAM.md.
  ZJoltAssertFailedFn assert_failed;
  /// Passed back to `trace` and `assert_failed`. Must outlive zjoltDeinit.
  void *hooks_user;
} ZJoltInitDesc;

/// Do not call directly; use zjoltInit, which supplies the config id.
ZJOLT_API ZJoltResult zjoltInitWithConfig(const ZJoltInitDesc *desc,
                                          uint32_t config_id);

/// Installs the allocator and hooks, creates Jolt's factory and registers its
/// types. Must be called before anything else, and is process-wide.
///
/// `desc` may be NULL for all defaults. Returns ZJOLT_ERR_CONFIG_MISMATCH if
/// this header was compiled with different layout-affecting settings than the
/// library.
static inline ZJoltResult zjoltInit(const ZJoltInitDesc *desc) {
  return zjoltInitWithConfig(desc, (uint32_t)ZJOLT_CONFIG_ID);
}

/// Unregisters Jolt's types, destroys the factory, and gives Jolt its own
/// allocator back. Safe to call when not initialised.
///
/// REFUSES, and does nothing but trace, while any handle is still alive. That
/// is deliberate: the last thing it does is restore the allocator, and a
/// handle destroyed after that is freed through an allocator it was never
/// allocated from — heap corruption with no symptom anywhere near the mistake.
/// Leaving the library up instead keeps the eventual destroy correct. Check
/// zjoltIsInitialized or zjoltLiveHandleCount if you need to know it happened.
ZJOLT_API void zjoltDeinit(void);

ZJOLT_API bool zjoltIsInitialized(void);

/// Physics systems, job systems and characters currently alive.
///
/// Shapes are not counted: Jolt reference counts them, and their count changes
/// inside calls zjolt does not mediate, so a number kept here would be wrong
/// rather than merely absent. Release your shapes anyway — they are freed
/// through the installed allocator too.
ZJOLT_API uint32_t zjoltLiveHandleCount(void);

//===----------------------------------------------------------------------===//
// Plain-data types
//
// ZJoltVec3 is three floats, not four. Jolt's own Vec3 is a 16-byte SIMD
// register with a padding lane; projecting it to 12 bytes at the boundary
// costs one shuffle per crossing and saves every consumer from modelling a
// lane it must never read.
//===----------------------------------------------------------------------===//

typedef struct ZJoltVec3 {
  float x, y, z;
} ZJoltVec3;

/// A quaternion in (x, y, z, w) order — w LAST, matching Jolt.
typedef struct ZJoltQuat {
  float x, y, z, w;
} ZJoltQuat;

/// Scalar type of a world-space position. `double` when the library was built
/// with double precision, otherwise `float`.
#ifdef ZJOLT_DOUBLE_PRECISION
typedef double ZJoltReal;
#else
typedef float ZJoltReal;
#endif

/// A world-space position. Distinct from ZJoltVec3, which stays float even in
/// a double-precision build: directions, velocities and extents do not need
/// the range, and Jolt keeps that distinction internally too.
typedef struct ZJoltRVec3 {
  ZJoltReal x, y, z;
} ZJoltRVec3;

typedef struct ZJoltAABox {
  ZJoltVec3 min;
  ZJoltVec3 max;
} ZJoltAABox;

/// Mass and inertia of a shape. `inertia` is a row-major 3x3 matrix.
typedef struct ZJoltMassProperties {
  float mass;
  float inertia[9];
} ZJoltMassProperties;

/// Identifies a body within one physics system. Stable until the body is
/// destroyed; ids are recycled with a generation counter, so a stale id is
/// detected rather than aliasing a new body.
typedef uint32_t ZJoltBodyId;
#define ZJOLT_BODY_ID_INVALID ((ZJoltBodyId)0xffffffffu)

/// Identifies one leaf of a compound or mesh shape within a hit.
typedef uint32_t ZJoltSubShapeId;

#if ZJOLT_OBJECT_LAYER_BITS == 32
typedef uint32_t ZJoltObjectLayer;
#else
typedef uint16_t ZJoltObjectLayer;
#endif

typedef uint8_t ZJoltBroadPhaseLayer;

//===----------------------------------------------------------------------===//
// Enumerations
//===----------------------------------------------------------------------===//

typedef enum ZJoltMotionType {
  ZJOLT_MOTION_TYPE_STATIC = 0,
  ZJOLT_MOTION_TYPE_KINEMATIC = 1,
  ZJOLT_MOTION_TYPE_DYNAMIC = 2,
} ZJoltMotionType;

typedef enum ZJoltMotionQuality {
  /// Cheapest. A fast body can tunnel through thin geometry.
  ZJOLT_MOTION_QUALITY_DISCRETE = 0,
  /// Sweeps the shape from start to end each step.
  ZJOLT_MOTION_QUALITY_LINEAR_CAST = 1,
} ZJoltMotionQuality;

typedef enum ZJoltActivation {
  ZJOLT_ACTIVATION_ACTIVATE = 0,
  ZJOLT_ACTIVATION_DONT_ACTIVATE = 1,
} ZJoltActivation;

/// Degrees of freedom a body is allowed, as a bit mask. Use
/// ZJOLT_ALLOWED_DOFS_ALL unless constraining to a plane.
typedef enum ZJoltAllowedDofs {
  ZJOLT_ALLOWED_DOFS_TRANSLATION_X = 1 << 0,
  ZJOLT_ALLOWED_DOFS_TRANSLATION_Y = 1 << 1,
  ZJOLT_ALLOWED_DOFS_TRANSLATION_Z = 1 << 2,
  ZJOLT_ALLOWED_DOFS_ROTATION_X = 1 << 3,
  ZJOLT_ALLOWED_DOFS_ROTATION_Y = 1 << 4,
  ZJOLT_ALLOWED_DOFS_ROTATION_Z = 1 << 5,
  ZJOLT_ALLOWED_DOFS_ALL = 0b111111,
} ZJoltAllowedDofs;

typedef enum ZJoltOverrideMassProperties {
  /// Use the shape's mass and inertia.
  ZJOLT_OVERRIDE_MASS_CALCULATE_MASS_AND_INERTIA = 0,
  /// Use ZJoltBodyDesc::mass, scale the shape's inertia to match.
  ZJOLT_OVERRIDE_MASS_CALCULATE_INERTIA = 1,
} ZJoltOverrideMassProperties;

/// Shape kinds this ABI can produce. Reported by zjoltShapeGetSubType, and
/// meaningful after a restore to confirm what came back.
typedef enum ZJoltShapeSubType {
  ZJOLT_SHAPE_SUB_TYPE_OTHER = 0,
  ZJOLT_SHAPE_SUB_TYPE_SPHERE = 1,
  ZJOLT_SHAPE_SUB_TYPE_BOX = 2,
  ZJOLT_SHAPE_SUB_TYPE_CAPSULE = 3,
  ZJOLT_SHAPE_SUB_TYPE_CONVEX_HULL = 4,
  ZJOLT_SHAPE_SUB_TYPE_MESH = 5,
  ZJOLT_SHAPE_SUB_TYPE_SCALED = 6,
  ZJOLT_SHAPE_SUB_TYPE_ROTATED_TRANSLATED = 7,
  ZJOLT_SHAPE_SUB_TYPE_OFFSET_CENTER_OF_MASS = 8,
} ZJoltShapeSubType;

typedef enum ZJoltBackFaceMode {
  ZJOLT_BACK_FACE_IGNORE = 0,
  ZJOLT_BACK_FACE_COLLIDE = 1,
} ZJoltBackFaceMode;

typedef enum ZJoltGroundState {
  /// On the ground and free to move.
  ZJOLT_GROUND_STATE_ON_GROUND = 0,
  /// On a slope too steep to climb. The caller should start sliding.
  ZJOLT_GROUND_STATE_ON_STEEP_GROUND = 1,
  /// Touching something that does not support the character; it should fall.
  ZJOLT_GROUND_STATE_NOT_SUPPORTED = 2,
  /// Touching nothing.
  ZJOLT_GROUND_STATE_IN_AIR = 3,
} ZJoltGroundState;

/// What a contact-validate callback decides about a body pair.
typedef enum ZJoltValidateResult {
  ZJOLT_VALIDATE_ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR = 0,
  ZJOLT_VALIDATE_ACCEPT_CONTACT = 1,
  ZJOLT_VALIDATE_REJECT_CONTACT = 2,
  ZJOLT_VALIDATE_REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR = 3,
} ZJoltValidateResult;

/// Bit mask reported by zjoltPhysicsSystemStep. Non-zero means the step
/// silently dropped contacts and the corresponding limit in
/// ZJoltPhysicsSystemDesc should be raised.
typedef enum ZJoltUpdateError {
  ZJOLT_UPDATE_ERROR_NONE = 0,
  ZJOLT_UPDATE_ERROR_MANIFOLD_CACHE_FULL = 1 << 0,
  ZJOLT_UPDATE_ERROR_BODY_PAIR_CACHE_FULL = 1 << 1,
  ZJOLT_UPDATE_ERROR_CONTACT_CONSTRAINTS_FULL = 1 << 2,
} ZJoltUpdateError;

//===----------------------------------------------------------------------===//
// Opaque handles
//===----------------------------------------------------------------------===//

/// A collision shape. Immutable, reference counted, and shareable between
/// bodies and between systems.
typedef struct ZJoltShape ZJoltShape;

/// A world: bodies, a broad phase, and the step.
typedef struct ZJoltPhysicsSystem ZJoltPhysicsSystem;

/// The scheduler a step runs its jobs on. See the JobSystem section.
typedef struct ZJoltJobSystem ZJoltJobSystem;

/// A body, borrowed for the duration of a body lock. Never outlives its lock.
typedef struct ZJoltBody ZJoltBody;

/// A shape-based character controller that is not a rigid body.
typedef struct ZJoltCharacter ZJoltCharacter;

//===----------------------------------------------------------------------===//
// Job system
//
// The step is parallel, and which threads it runs on is a host decision. v0.1
// ships Jolt's own thread pool and its single-threaded scheduler; a host
// scheduler plugs in later through an ADDITIONAL constructor, which is why
// this is an opaque handle rather than a struct the caller fills in — adding
// a way to make one is not an ABI change.
//===----------------------------------------------------------------------===//

/// Jolt's own recommended sizing for a physics job system.
#define ZJOLT_MAX_PHYSICS_JOBS 2048
#define ZJOLT_MAX_PHYSICS_BARRIERS 8

/// `num_threads` of -1 means one worker per hardware thread minus the calling
/// thread.
ZJOLT_API ZJoltResult zjoltJobSystemCreateThreadPool(uint32_t max_jobs,
                                                     uint32_t max_barriers,
                                                     int32_t num_threads,
                                                     ZJoltJobSystem **out);

/// Runs every job on the calling thread. Slower, but makes a step reproducible
/// without pinning a thread count — which is what the tests use.
ZJOLT_API ZJoltResult zjoltJobSystemCreateSingleThreaded(uint32_t max_jobs,
                                                         ZJoltJobSystem **out);

ZJOLT_API void zjoltJobSystemDestroy(ZJoltJobSystem *job_system);

/// Number of jobs this scheduler can run at once. 1 for the single-threaded
/// scheduler.
ZJOLT_API uint32_t zjoltJobSystemGetMaxConcurrency(const ZJoltJobSystem *job_system);

//===----------------------------------------------------------------------===//
// Shapes
//
// Every constructor returns a shape with a reference count of one. Adding a
// shape to a body takes its own reference, so the usual pattern is to create,
// create the body, and release.
//===----------------------------------------------------------------------===//

/// `convex_radius` rounds the box corners for cheaper, more stable collision;
/// it must be at most the smallest half extent. `density` is in kg/m^3.
ZJOLT_API ZJoltResult zjoltShapeCreateBox(const ZJoltVec3 *half_extent,
                                          float convex_radius, float density,
                                          ZJoltShape **out);

ZJOLT_API ZJoltResult zjoltShapeCreateSphere(float radius, float density,
                                             ZJoltShape **out);

/// A capsule along the Y axis: a cylinder of `half_height_of_cylinder` capped
/// by hemispheres of `radius`. Total height is 2*(half_height + radius).
ZJOLT_API ZJoltResult zjoltShapeCreateCapsule(float half_height_of_cylinder,
                                              float radius, float density,
                                              ZJoltShape **out);

/// Builds the convex hull OF the given points; interior points are allowed and
/// discarded. `hull_tolerance` is how far a point may sit outside the hull
/// (larger yields fewer vertices); pass 0 for Jolt's default.
ZJOLT_API ZJoltResult zjoltShapeCreateConvexHull(const ZJoltVec3 *points,
                                                 uint32_t num_points,
                                                 float max_convex_radius,
                                                 float hull_tolerance,
                                                 float density,
                                                 ZJoltShape **out);

/// A static triangle mesh. `indices` holds 3*num_triangles vertex indices.
/// Duplicate and degenerate triangles are removed by Jolt during the build.
///
/// A mesh shape may only be used by a static or kinematic body — Jolt has no
/// inertia for one. Building it is the expensive part of collision cooking, so
/// this is the shape most worth saving with zjoltShapeSave.
ZJOLT_API ZJoltResult zjoltShapeCreateMesh(const ZJoltVec3 *vertices,
                                           uint32_t num_vertices,
                                           const uint32_t *indices,
                                           uint32_t num_triangles,
                                           uint32_t max_triangles_per_leaf,
                                           ZJoltShape **out);

/// Non-uniformly scales an existing shape. Takes a reference on `inner`.
ZJOLT_API ZJoltResult zjoltShapeCreateScaled(const ZJoltShape *inner,
                                             const ZJoltVec3 *scale,
                                             ZJoltShape **out);

/// Places an existing shape at an offset and orientation inside its parent.
ZJOLT_API ZJoltResult zjoltShapeCreateRotatedTranslated(
    const ZJoltShape *inner, const ZJoltVec3 *translation,
    const ZJoltQuat *rotation, ZJoltShape **out);

/// Shifts a shape's centre of mass without moving its geometry.
ZJOLT_API ZJoltResult zjoltShapeCreateOffsetCenterOfMass(
    const ZJoltShape *inner, const ZJoltVec3 *offset, ZJoltShape **out);

ZJOLT_API void zjoltShapeAddRef(const ZJoltShape *shape);
ZJOLT_API void zjoltShapeRelease(const ZJoltShape *shape);
ZJOLT_API uint32_t zjoltShapeGetRefCount(const ZJoltShape *shape);

ZJOLT_API ZJoltShapeSubType zjoltShapeGetSubType(const ZJoltShape *shape);
ZJOLT_API float zjoltShapeGetVolume(const ZJoltShape *shape);
ZJOLT_API void zjoltShapeGetCenterOfMass(const ZJoltShape *shape,
                                         ZJoltVec3 *out);
ZJOLT_API void zjoltShapeGetLocalBounds(const ZJoltShape *shape,
                                        ZJoltAABox *out);
ZJOLT_API void zjoltShapeGetMassProperties(const ZJoltShape *shape,
                                           ZJoltMassProperties *out);
/// Memory footprint and triangle count of a shape and everything under it.
///
/// Jolt exposes this for logging and budgeting, and it is the cheapest way to
/// confirm a shape survived a save/restore round trip with its children
/// intact.
typedef struct ZJoltShapeStats {
  uint64_t size_bytes;
  uint32_t num_triangles;
} ZJoltShapeStats;

ZJOLT_API void zjoltShapeGetStats(const ZJoltShape *shape,
                                  ZJoltShapeStats *out);

/// Serialises `shape` and everything under it into `buffer`.
///
/// Two-call protocol: pass buffer = NULL to learn the size, then call again
/// with storage. `*out_size` is always written, so a too-small buffer reports
/// ZJOLT_ERR_BUFFER_TOO_SMALL along with what was needed.
///
/// The payload is Jolt's own binary shape state, behind a 32-byte header
/// carrying a magic tag, the container version, this library's config id, the
/// Jolt version, the payload length and a CRC-32. The header is not
/// decoration: Jolt reads the shape type out of the stream and uses it to
/// index a table BEFORE it checks whether the read succeeded, so a buffer that
/// is not a shape has to be rejected before Jolt sees it.
///
/// It remains a cooking cache, not an interchange format — a shape saved by a
/// different Jolt version, or a different precision setting, is refused rather
/// than reinterpreted. And it is not a defence against a crafted payload with
/// a matching checksum; treat a cache as something your own tools wrote.
ZJOLT_API ZJoltResult zjoltShapeSave(const ZJoltShape *shape, void *buffer,
                                     size_t capacity, size_t *out_size);

/// Rebuilds a shape from zjoltShapeSave output. The buffer is read during the
/// call only. A wrong tag, a wrong build, a length that disagrees with the
/// buffer, or a failed checksum is ZJOLT_ERR_BAD_FORMAT; zjoltLastError says
/// which.
ZJOLT_API ZJoltResult zjoltShapeRestore(const void *data, size_t size,
                                        ZJoltShape **out);

/// Bytes zjoltShapeSave prepends to Jolt's payload. Exposed so a test can
/// reach the payload itself, not so callers can build one by hand.
#define ZJOLT_SHAPE_HEADER_SIZE 32

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

//===----------------------------------------------------------------------===//
// Bodies
//===----------------------------------------------------------------------===//

typedef struct ZJoltBodyDesc {
  ZJoltRVec3 position;
  ZJoltQuat rotation;
  ZJoltVec3 linear_velocity;
  ZJoltVec3 angular_velocity;
  /// Required. A reference is taken for the lifetime of the body.
  const ZJoltShape *shape;
  uint64_t user_data;
  ZJoltObjectLayer object_layer;
  ZJoltMotionType motion_type;
  ZJoltMotionQuality motion_quality;
  ZJoltAllowedDofs allowed_dofs;
  ZJoltOverrideMassProperties override_mass_properties;
  /// Used when override_mass_properties is CALCULATE_INERTIA.
  float mass;
  /// Lets a static body be switched to kinematic or dynamic later.
  bool allow_dynamic_or_kinematic;
  bool is_sensor;
  bool allow_sleeping;
  /// Extra work to suppress ghost collisions on internal mesh edges.
  bool enhanced_internal_edge_removal;
  float friction;
  float restitution;
  float linear_damping;
  float angular_damping;
  float max_linear_velocity;
  float max_angular_velocity;
  float gravity_factor;
} ZJoltBodyDesc;

/// Fills `desc` with Jolt's own defaults. Call this first and then overwrite;
/// the defaults are not all zero and are not all obvious.
ZJOLT_API void zjoltBodyDescInit(ZJoltBodyDesc *desc);

/// Creates a body without adding it to the simulation.
ZJOLT_API ZJoltResult zjoltBodyCreate(ZJoltPhysicsSystem *system,
                                      const ZJoltBodyDesc *desc,
                                      ZJoltBodyId *out);

ZJOLT_API ZJoltResult zjoltBodyCreateAndAdd(ZJoltPhysicsSystem *system,
                                            const ZJoltBodyDesc *desc,
                                            ZJoltActivation activation,
                                            ZJoltBodyId *out);

/// Removes the body if it is still added, then destroys it. The id becomes
/// stale; further calls with it report ZJOLT_ERR_BODY_NOT_FOUND rather than
/// touching whatever body was created next.
ZJOLT_API void zjoltBodyDestroy(ZJoltPhysicsSystem *system, ZJoltBodyId body);

ZJOLT_API void zjoltBodyAdd(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                            ZJoltActivation activation);
ZJOLT_API void zjoltBodyRemove(ZJoltPhysicsSystem *system, ZJoltBodyId body);
ZJOLT_API bool zjoltBodyIsAdded(const ZJoltPhysicsSystem *system,
                                ZJoltBodyId body);
ZJOLT_API bool zjoltBodyIsActive(const ZJoltPhysicsSystem *system,
                                 ZJoltBodyId body);
ZJOLT_API void zjoltBodyActivate(ZJoltPhysicsSystem *system, ZJoltBodyId body);
ZJOLT_API void zjoltBodyDeactivate(ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body);

ZJOLT_API void zjoltBodySetMotionType(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body, ZJoltMotionType type,
                                      ZJoltActivation activation);
ZJOLT_API ZJoltMotionType zjoltBodyGetMotionType(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

/// Teleport: places the body immediately, ignoring the collision that puts it
/// there. `rotation` may be NULL to keep the current orientation.
ZJOLT_API void zjoltBodySetPositionAndRotation(ZJoltPhysicsSystem *system,
                                               ZJoltBodyId body,
                                               const ZJoltRVec3 *position,
                                               const ZJoltQuat *rotation,
                                               ZJoltActivation activation);
/// Either out-parameter may be NULL.
ZJOLT_API void zjoltBodyGetPositionAndRotation(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltRVec3 *out_position,
    ZJoltQuat *out_rotation);
ZJOLT_API void zjoltBodyGetCenterOfMassPosition(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, ZJoltRVec3 *out);

/// Drives a kinematic body toward a target over `delta_time`, so it pushes
/// dynamic bodies out of the way instead of teleporting through them. This is
/// how a moving platform or an animated character should move.
ZJOLT_API void zjoltBodyMoveKinematic(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body,
                                      const ZJoltRVec3 *target_position,
                                      const ZJoltQuat *target_rotation,
                                      float delta_time);

ZJOLT_API void zjoltBodySetLinearVelocity(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodyGetLinearVelocity(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body, ZJoltVec3 *out);
ZJOLT_API void zjoltBodySetAngularVelocity(ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body,
                                           const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodyGetAngularVelocity(const ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body, ZJoltVec3 *out);

/// Forces and torques accumulate until the next step consumes them; impulses
/// change velocity immediately. `point` is in world space.
ZJOLT_API void zjoltBodyAddForce(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                 const ZJoltVec3 *force);
ZJOLT_API void zjoltBodyAddForceAtPoint(ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body,
                                        const ZJoltVec3 *force,
                                        const ZJoltRVec3 *point);
ZJOLT_API void zjoltBodyAddTorque(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                  const ZJoltVec3 *torque);
ZJOLT_API void zjoltBodyAddImpulse(ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body,
                                   const ZJoltVec3 *impulse);
ZJOLT_API void zjoltBodyAddImpulseAtPoint(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *impulse,
                                          const ZJoltRVec3 *point);
ZJOLT_API void zjoltBodyAddAngularImpulse(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *angular_impulse);

/// Replaces the shape. With `update_mass_properties` the body's mass and
/// inertia are recomputed from the new shape.
ZJOLT_API void zjoltBodySetShape(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                 const ZJoltShape *shape,
                                 bool update_mass_properties,
                                 ZJoltActivation activation);

ZJOLT_API void zjoltBodySetObjectLayer(ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body,
                                       ZJoltObjectLayer layer);
ZJOLT_API ZJoltObjectLayer zjoltBodyGetObjectLayer(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body);

ZJOLT_API void zjoltBodySetUserData(ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body, uint64_t user_data);
ZJOLT_API uint64_t zjoltBodyGetUserData(const ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body);

ZJOLT_API void zjoltBodySetFriction(ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body, float friction);
ZJOLT_API float zjoltBodyGetFriction(const ZJoltPhysicsSystem *system,
                                     ZJoltBodyId body);
ZJOLT_API void zjoltBodySetRestitution(ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body, float restitution);
ZJOLT_API float zjoltBodyGetRestitution(const ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body);
ZJOLT_API void zjoltBodySetGravityFactor(ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body, float factor);
ZJOLT_API float zjoltBodyGetGravityFactor(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body);

//===----------------------------------------------------------------------===//
// Bulk read-back
//
// The accessors above are one ABI crossing and one body lock each. That is the
// right shape for the occasional query and the wrong shape for the thing a
// renderer does every frame: read the transform of every body that moved.
//
// These two calls exist so that per-frame read-back is two crossings and one
// lock acquisition rather than 2N of each. Reach for them in the frame loop
// and for the single-body accessors everywhere else.
//===----------------------------------------------------------------------===//

/// Ids of the bodies that are awake, which is the set whose transforms can
/// have changed since the last step.
///
/// Two-call protocol: `*out_count` always receives the true number, so a
/// capacity of 0 with `out_ids` NULL is a size query.
ZJOLT_API ZJoltResult zjoltPhysicsSystemGetActiveBodies(
    const ZJoltPhysicsSystem *system, ZJoltBodyId *out_ids, uint32_t capacity,
    uint32_t *out_count);

/// Every body in the system, awake or not, in no particular order.
ZJOLT_API ZJoltResult zjoltPhysicsSystemGetBodies(
    const ZJoltPhysicsSystem *system, ZJoltBodyId *out_ids, uint32_t capacity,
    uint32_t *out_count);

/// Reads `count` body transforms under a single lock.
///
/// `out_positions` and `out_rotations` are parallel arrays of `count` entries;
/// either may be NULL if that half is not wanted. An id that no longer names a
/// live body writes the identity transform and is reported through
/// `out_missing` (which may be NULL) rather than failing the whole batch —
/// a body destroyed between the step and the read is normal, not an error.
ZJOLT_API ZJoltResult zjoltBodyGetTransforms(const ZJoltPhysicsSystem *system,
                                             const ZJoltBodyId *ids,
                                             uint32_t count,
                                             ZJoltRVec3 *out_positions,
                                             ZJoltQuat *out_rotations,
                                             uint32_t *out_missing);

/// As zjoltBodyGetTransforms, but reads centre-of-mass positions and linear
/// velocities — what an interpolating or audio-driven host wants.
ZJOLT_API ZJoltResult zjoltBodyGetMotions(const ZJoltPhysicsSystem *system,
                                          const ZJoltBodyId *ids,
                                          uint32_t count,
                                          ZJoltRVec3 *out_center_of_mass,
                                          ZJoltVec3 *out_linear_velocities,
                                          uint32_t *out_missing);

//===----------------------------------------------------------------------===//
// Body locks
//
// The accessors above take a lock per call, which is the right default and the
// wrong tool for reading six properties of the same body. A lock holds the
// body still for a scope, and hands out a borrowed ZJoltBody that must not
// outlive it.
//
// This maps Jolt's RAII lock one-to-one, which means the release call is the
// caller's responsibility. `body` is NULL when the id was stale — check it,
// and release either way.
//===----------------------------------------------------------------------===//

typedef struct ZJoltBodyLock {
  /// NULL when the body id does not name a live body.
  ZJoltBody *body;
  /// Implementation detail. Do not read, do not copy this struct while held.
  void *_reserved[2];
} ZJoltBodyLock;

/// Takes a shared lock. Several readers may hold one at once.
ZJOLT_API void zjoltBodyLockRead(const ZJoltPhysicsSystem *system,
                                 ZJoltBodyId body, ZJoltBodyLock *out_lock);
ZJOLT_API void zjoltBodyLockReadRelease(ZJoltBodyLock *lock);

/// Takes an exclusive lock. Required before any zjoltBodyMut* call.
ZJOLT_API void zjoltBodyLockWrite(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                  ZJoltBodyLock *out_lock);
ZJOLT_API void zjoltBodyLockWriteRelease(ZJoltBodyLock *lock);

ZJOLT_API ZJoltBodyId zjoltBodyGetId(const ZJoltBody *body);
ZJOLT_API void zjoltBodyGetPosition(const ZJoltBody *body, ZJoltRVec3 *out);
ZJOLT_API void zjoltBodyGetRotation(const ZJoltBody *body, ZJoltQuat *out);
ZJOLT_API void zjoltBodyGetCenterOfMassPositionLocked(const ZJoltBody *body,
                                                      ZJoltRVec3 *out);
ZJOLT_API void zjoltBodyGetLinearVelocityLocked(const ZJoltBody *body,
                                                ZJoltVec3 *out);
ZJOLT_API void zjoltBodyGetAngularVelocityLocked(const ZJoltBody *body,
                                                 ZJoltVec3 *out);
ZJOLT_API uint64_t zjoltBodyGetUserDataLocked(const ZJoltBody *body);
ZJOLT_API ZJoltObjectLayer zjoltBodyGetObjectLayerLocked(const ZJoltBody *body);
ZJOLT_API ZJoltMotionType zjoltBodyGetMotionTypeLocked(const ZJoltBody *body);
ZJOLT_API bool zjoltBodyIsActiveLocked(const ZJoltBody *body);
ZJOLT_API bool zjoltBodyIsSensorLocked(const ZJoltBody *body);
/// Borrowed; valid while the body is alive. Takes no reference.
ZJOLT_API const ZJoltShape *zjoltBodyGetShapeLocked(const ZJoltBody *body);
ZJOLT_API void zjoltBodyGetWorldBounds(const ZJoltBody *body, ZJoltAABox *out);

/// Requires a write lock. Velocity changes made here bypass activation, so
/// activate the body separately if it may be asleep.
ZJOLT_API void zjoltBodyMutSetLinearVelocity(ZJoltBody *body,
                                             const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodyMutSetAngularVelocity(ZJoltBody *body,
                                              const ZJoltVec3 *velocity);
ZJOLT_API void zjoltBodyMutSetUserData(ZJoltBody *body, uint64_t user_data);
ZJOLT_API void zjoltBodyMutSetFriction(ZJoltBody *body, float friction);
ZJOLT_API void zjoltBodyMutSetRestitution(ZJoltBody *body, float restitution);
ZJOLT_API void zjoltBodyMutAddImpulse(ZJoltBody *body,
                                      const ZJoltVec3 *impulse);

//===----------------------------------------------------------------------===//
// Queries
//
// Filters are optional at every level: a NULL ZJoltQueryFilters accepts
// everything, and so does a member whose function pointer is NULL.
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

typedef struct ZJoltQueryFilters {
  ZJoltBroadPhaseLayerFilter broad_phase_layer;
  ZJoltObjectLayerFilter object_layer;
  ZJoltBodyFilter body;
} ZJoltQueryFilters;

typedef struct ZJoltRayCastHit {
  ZJoltBodyId body;
  ZJoltSubShapeId sub_shape_id;
  /// Hit point is origin + fraction * direction, with fraction in [0, 1].
  float fraction;
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
  ZJoltVec3 penetration_axis;
  float penetration_depth;
  bool is_back_face_hit;
} ZJoltShapeCastHit;

typedef struct ZJoltCollideShapeHit {
  ZJoltBodyId body;
  ZJoltSubShapeId sub_shape_id;
  /// Relative to the query's `position`. @see ZJoltShapeCastHit
  ZJoltVec3 contact_point_on_1;
  ZJoltVec3 contact_point_on_2;
  ZJoltVec3 penetration_axis;
  float penetration_depth;
} ZJoltCollideShapeHit;

/// `direction` carries the ray's length: nothing beyond it is reported.
/// `*out_hit_any` says whether `out_hit` was written.
ZJOLT_API ZJoltResult zjoltCastRayClosest(const ZJoltPhysicsSystem *system,
                                          const ZJoltRVec3 *origin,
                                          const ZJoltVec3 *direction,
                                          const ZJoltQueryFilters *filters,
                                          ZJoltRayCastHit *out_hit,
                                          bool *out_hit_any);

/// Collects every hit along the ray, unsorted.
///
/// Two-call protocol, as everywhere here: `*out_count` always receives the
/// true number of hits, so a capacity of 0 (with hits = NULL) is a size query
/// and a short buffer reports ZJOLT_ERR_BUFFER_TOO_SMALL with the count.
ZJOLT_API ZJoltResult zjoltCastRayAll(const ZJoltPhysicsSystem *system,
                                      const ZJoltRVec3 *origin,
                                      const ZJoltVec3 *direction,
                                      const ZJoltQueryFilters *filters,
                                      ZJoltRayCastHit *out_hits,
                                      uint32_t capacity, uint32_t *out_count);

/// Sweeps `shape` from `position`/`rotation` along `direction`. `scale` may be
/// NULL for (1,1,1).
ZJOLT_API ZJoltResult zjoltCastShapeClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltQueryFilters *filters, ZJoltShapeCastHit *out_hit,
    bool *out_hit_any);

ZJOLT_API ZJoltResult zjoltCastShapeAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltQueryFilters *filters, ZJoltShapeCastHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

/// Everything overlapping `shape` placed at `position`/`rotation`.
ZJOLT_API ZJoltResult zjoltCollideShape(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, float max_separation_distance,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Character
//
// CharacterVirtual is not a rigid body: it is a shape that is swept through
// the world under the host's control, which is what makes it feel like a game
// character rather than a barrel. It can optionally carry an inner rigid body
// so that other bodies and queries can see it.
//===----------------------------------------------------------------------===//

typedef struct ZJoltCharacterDesc {
  /// Required. Its centre should sit at the character's centre; use a rotated-
  /// translated shape to put a capsule's base at the origin.
  const ZJoltShape *shape;
  ZJoltRVec3 position;
  ZJoltQuat rotation;
  /// Which way is up for this character. Need not be the world up.
  ZJoltVec3 up;
  ZJoltVec3 shape_offset;
  uint64_t user_data;
  /// Radians. Ground steeper than this reports ON_STEEP_GROUND.
  float max_slope_angle;
  float mass;
  /// Force the character can apply to push dynamic bodies.
  float max_strength;
  /// How far to look ahead for contacts. 0 will get the character stuck.
  float predictive_contact_distance;
  /// Gap kept between the shape and geometry, so sweeps hit less.
  float character_padding;
  float penetration_recovery_speed;
  float collision_tolerance;
  float hit_reduction_cos_max_angle;
  uint32_t max_collision_iterations;
  uint32_t max_constraint_iterations;
  uint32_t max_num_hits;
  ZJoltBackFaceMode back_face_mode;
  bool enhanced_internal_edge_removal;
  /// Optional rigid body that follows the character so the world can see it.
  /// It is what makes the character visible to ray casts and to other bodies,
  /// since CharacterVirtual itself is not in the broad phase. NULL for none.
  const ZJoltShape *inner_body_shape;
  ZJoltObjectLayer inner_body_layer;
} ZJoltCharacterDesc;

/// Fills `desc` with Jolt's defaults. `shape` is left NULL and is required.
ZJOLT_API void zjoltCharacterDescInit(ZJoltCharacterDesc *desc);

/// Extra behaviour layered on top of the plain move: sticking to the floor on
/// the way down a slope, and stepping up stairs.
typedef struct ZJoltCharacterUpdateSettings {
  /// Zero disables floor sticking.
  ZJoltVec3 stick_to_floor_step_down;
  /// Zero disables stair walking.
  ZJoltVec3 walk_stairs_step_up;
  float walk_stairs_min_step_forward;
  float walk_stairs_step_forward_test;
  float walk_stairs_cos_angle_forward_contact;
  ZJoltVec3 walk_stairs_step_down_extra;
} ZJoltCharacterUpdateSettings;

ZJOLT_API void zjoltCharacterUpdateSettingsInit(
    ZJoltCharacterUpdateSettings *settings);

/// The character borrows `system` for its lifetime and must be destroyed
/// before it.
ZJOLT_API ZJoltResult zjoltCharacterCreate(ZJoltPhysicsSystem *system,
                                           const ZJoltCharacterDesc *desc,
                                           ZJoltCharacter **out);
ZJOLT_API void zjoltCharacterDestroy(ZJoltCharacter *character);

/// Moves the character by its current velocity for `delta_time`, resolving
/// collision. Set the velocity first with zjoltCharacterSetLinearVelocity.
///
/// `settings` may be NULL for a plain move with no stair or floor handling.
/// `filters` may be NULL to collide with everything.
ZJOLT_API ZJoltResult zjoltCharacterUpdate(
    ZJoltCharacter *character, float delta_time, const ZJoltVec3 *gravity,
    const ZJoltCharacterUpdateSettings *settings,
    const ZJoltQueryFilters *filters);

ZJOLT_API void zjoltCharacterGetPosition(const ZJoltCharacter *character,
                                         ZJoltRVec3 *out);
ZJOLT_API void zjoltCharacterSetPosition(ZJoltCharacter *character,
                                         const ZJoltRVec3 *position);
ZJOLT_API void zjoltCharacterGetRotation(const ZJoltCharacter *character,
                                         ZJoltQuat *out);
ZJOLT_API void zjoltCharacterSetRotation(ZJoltCharacter *character,
                                         const ZJoltQuat *rotation);
ZJOLT_API void zjoltCharacterGetLinearVelocity(const ZJoltCharacter *character,
                                               ZJoltVec3 *out);
ZJOLT_API void zjoltCharacterSetLinearVelocity(ZJoltCharacter *character,
                                               const ZJoltVec3 *velocity);

ZJOLT_API ZJoltGroundState zjoltCharacterGetGroundState(
    const ZJoltCharacter *character);
/// True for ON_GROUND and ON_STEEP_GROUND.
ZJOLT_API bool zjoltCharacterIsSupported(const ZJoltCharacter *character);
ZJOLT_API void zjoltCharacterGetGroundNormal(const ZJoltCharacter *character,
                                             ZJoltVec3 *out);
ZJOLT_API void zjoltCharacterGetGroundVelocity(const ZJoltCharacter *character,
                                               ZJoltVec3 *out);
ZJOLT_API void zjoltCharacterGetGroundPosition(const ZJoltCharacter *character,
                                               ZJoltRVec3 *out);
ZJOLT_API ZJoltBodyId zjoltCharacterGetGroundBodyId(
    const ZJoltCharacter *character);
ZJOLT_API uint64_t zjoltCharacterGetGroundUserData(
    const ZJoltCharacter *character);

/// Recomputes the ground velocity, for reading after moving the ground body.
ZJOLT_API void zjoltCharacterUpdateGroundVelocity(ZJoltCharacter *character);

/// Swaps the shape — crouching. Fails without changing anything if the new
/// shape would be more than `max_penetration_depth` inside the world.
ZJOLT_API ZJoltResult zjoltCharacterSetShape(
    ZJoltCharacter *character, const ZJoltShape *shape,
    float max_penetration_depth, const ZJoltQueryFilters *filters,
    bool *out_changed);

ZJOLT_API const ZJoltShape *zjoltCharacterGetShape(
    const ZJoltCharacter *character);
ZJOLT_API ZJoltBodyId zjoltCharacterGetInnerBodyId(
    const ZJoltCharacter *character);

//===----------------------------------------------------------------------===//
// ABI layout guard
//
// The Zig wrapper hand-declares `extern struct`s mirroring the types above.
// Nothing in either compiler checks that those two declarations agree — a
// field reordered here and not there is silent memory corruption, not a build
// error. zjoltAbiLayout reports what the C++ side actually compiled to, so the
// other side can assert against it in a test.
//
// In the other direction, static_asserts in zjolt_abi.cpp fail the BUILD if a
// vendored Jolt upgrade changes a type this package converts to or from.
//===----------------------------------------------------------------------===//

/// Bits of ZJoltAbiLayout::build_flags. These do not change any layout — they
/// are here so a consumer can report, or refuse, a build it did not expect.
#define ZJOLT_BUILD_FLAG_DOUBLE_PRECISION (1u << 0)
#define ZJOLT_BUILD_FLAG_OBJECT_LAYER_32 (1u << 1)
#define ZJOLT_BUILD_FLAG_ASSERTS_ENABLED (1u << 2)
#define ZJOLT_BUILD_FLAG_CROSS_PLATFORM_DETERMINISTIC (1u << 3)

typedef struct ZJoltAbiLayout {
  /// sizeof(ZJoltAbiLayout). Read this first: if it disagrees with the
  /// consumer's own sizeof, the struct itself has changed and no field below
  /// can be trusted.
  uint32_t layout_size;
  uint32_t config_id;
  uint32_t build_flags;

  /// A hash folded over the size, alignment and EVERY field offset of every
  /// type in this header, in declaration order.
  ///
  /// The individual sizes and offsets below are the diagnosis; this is the
  /// detection. They are listed by hand, so they can only catch a change to a
  /// field somebody remembered to list — two adjacent floats swapped in
  /// ZJoltBodyDesc move no reported offset and change no size, and would pass
  /// every one of them. A consumer that can enumerate its own fields (Zig can)
  /// computes this the same way and compares one number, which cannot miss a
  /// field because it never had to know their names.
  uint32_t layout_digest;

  uint32_t real_size;
  uint32_t object_layer_size;
  uint32_t default_allocate_alignment;

  uint32_t vec3_size, vec3_align;
  uint32_t rvec3_size, rvec3_align;
  uint32_t quat_size, quat_align;
  uint32_t aabox_size, aabox_align;
  uint32_t mass_properties_size, mass_properties_align;
  uint32_t shape_stats_size, shape_stats_align;

  uint32_t allocator_size, allocator_align;
  uint32_t allocator_offset_allocate;
  uint32_t allocator_offset_reallocate;
  uint32_t allocator_offset_free;
  uint32_t allocator_offset_aligned_allocate;
  uint32_t allocator_offset_aligned_free;
  uint32_t allocator_offset_user;

  uint32_t init_desc_size, init_desc_align;

  uint32_t broad_phase_layer_interface_size;
  uint32_t broad_phase_layer_interface_align;
  uint32_t object_vs_broad_phase_filter_size;
  uint32_t object_layer_pair_filter_size;

  uint32_t system_desc_size, system_desc_align;
  uint32_t system_desc_offset_broad_phase_layers;
  uint32_t system_desc_offset_object_vs_broad_phase_filter;
  uint32_t system_desc_offset_object_layer_pair_filter;

  uint32_t contact_manifold_size, contact_manifold_align;
  uint32_t contact_manifold_offset_points_on_1;
  uint32_t contact_manifold_offset_points_on_2;
  uint32_t contact_info_size, contact_info_align;
  uint32_t contact_info_offset_manifold;
  uint32_t contact_settings_size, contact_settings_align;
  uint32_t contact_validate_info_size, contact_validate_info_align;
  uint32_t sub_shape_id_pair_size;
  uint32_t contact_listener_size;
  uint32_t body_activation_listener_size;

  uint32_t body_desc_size, body_desc_align;
  uint32_t body_desc_offset_position;
  uint32_t body_desc_offset_rotation;
  uint32_t body_desc_offset_shape;
  uint32_t body_desc_offset_user_data;
  uint32_t body_desc_offset_object_layer;
  uint32_t body_desc_offset_motion_type;
  uint32_t body_desc_offset_gravity_factor;

  uint32_t body_lock_size, body_lock_align;
  uint32_t body_lock_offset_body;

  uint32_t query_filters_size, query_filters_align;
  uint32_t ray_cast_hit_size, ray_cast_hit_align;
  uint32_t shape_cast_hit_size, shape_cast_hit_align;
  uint32_t collide_shape_hit_size, collide_shape_hit_align;

  uint32_t character_desc_size, character_desc_align;
  uint32_t character_desc_offset_shape;
  uint32_t character_desc_offset_position;
  uint32_t character_desc_offset_up;
  uint32_t character_update_settings_size;
  uint32_t character_update_settings_align;

  /// Number of enumerators, so a consumer can assert its own mapping is
  /// exhaustive.
  uint32_t result_count;
  uint32_t motion_type_count;
  uint32_t ground_state_count;
  uint32_t shape_sub_type_count;
  uint32_t validate_result_count;
} ZJoltAbiLayout;

/// Fills `out` with the layout the library was compiled with. Never fails.
ZJOLT_API void zjoltAbiLayout(ZJoltAbiLayout *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_H_
