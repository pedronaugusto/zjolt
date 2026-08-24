//===----------------------------------------------------------------------===//
// zjolt — a C ABI over Jolt Physics.
//
// These headers are the ONLY contract between the C++ implementation and any
// consumer (the Zig wrapper in ../src, or a plain C host). They are deliberately
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

#ifndef ZJOLT_CORE_H_
#define ZJOLT_CORE_H_

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

#endif  // ZJOLT_CORE_H_
