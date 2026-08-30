//===----------------------------------------------------------------------===//
// zjolt — a C ABI over Jolt Physics. Opaque handles, POD structs, function
// pointers, a flat error enum. No exceptions cross this boundary.
//
// Ownership: *Create yields an owned handle; *Destroy accepts NULL, safe to
// call once. Other handles are ref-counted (*AddRef/*Release) or owning
// (*Destroy), per type. Query results borrow nothing; callback pointers are
// valid only for the call. Aggregates pass by pointer; scalars by value.
//
// Thread safety: a ZJoltPhysicsSystem steps from one thread at a time.
// Contact/activation callbacks run on Jolt's job threads, concurrently,
// during zjoltPhysicsSystemStep (see ZJoltContactListener). Body reads
// outside a step need a body lock.
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
// Version and build configuration: two options change type LAYOUT, so
// they're folded into ZJOLT_CONFIG_ID, which zjoltInit compares against the
// library's own — a skew is a clean error at start-up, not silent
// corruption (mirrors Jolt's own JPH_VERSION_ID trick).
//===----------------------------------------------------------------------===//

// Unreleased. These stay 0.0.0 until the surface is complete and a version is
// actually cut; they are not decoration, they are folded into ZJOLT_CONFIG_ID
// below, so a consumer built against a different header is refused at init.
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
// Results. The declared surface does not depend on build options: a
// compiled-out feature keeps its function, returning
// ZJOLT_RESULT_UNSUPPORTED instead of failing to link. What is enabled is
// reported by zjoltConfigId/zjoltAbiLayout, not by which functions exist.
//===----------------------------------------------------------------------===//

typedef enum ZJoltResult {
  ZJOLT_RESULT_OK = 0,
  /// A call was made before zjoltInit, or after zjoltDeinit.
  ZJOLT_RESULT_NOT_INITIALIZED = 1,
  /// zjoltInit was called twice without an intervening zjoltDeinit.
  ZJOLT_RESULT_ALREADY_INITIALIZED = 2,
  /// The consumer's ZJOLT_CONFIG_ID does not match the library's.
  ZJOLT_RESULT_CONFIG_MISMATCH = 3,
  /// The installed allocator returned NULL.
  ZJOLT_RESULT_OUT_OF_MEMORY = 4,
  /// A NULL handle, a NULL required callback, or an out-of-domain scalar.
  ZJOLT_RESULT_INVALID_ARGUMENT = 5,
  /// A caller-provided output buffer was too small; the required count is
  /// still written to the out-count parameter.
  ZJOLT_RESULT_BUFFER_TOO_SMALL = 6,
  /// Jolt could not build the shape. zjoltLastError has its message.
  ZJOLT_RESULT_SHAPE_INVALID = 7,
  /// A serialised shape could not be restored: truncated, or not a shape.
  ZJOLT_RESULT_BAD_FORMAT = 8,
  /// The body id does not name a body in this system (it may have been
  /// destroyed).
  ZJOLT_RESULT_BODY_NOT_FOUND = 9,
  /// The call names a feature this library was not built with. The function
  /// exists in every build; whether it does anything is a build question.
  ZJOLT_RESULT_UNSUPPORTED = 10,
  /// A ZJoltStream reported failure — a write it could not complete, or a
  /// read from a stream already in an error state — during a save or
  /// restore. Distinct from ZJOLT_RESULT_BAD_FORMAT, which is about the
  /// shape of what was read rather than whether the transport delivered it.
  ZJOLT_RESULT_IO_ERROR = 11,
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
// Host streams: Jolt asks a save/restore to talk to StreamIn/StreamOut/
// StateRecorder (abstract C++ classes) — this ABI's standard answer to that
// shape of question: a plain function-pointer table, not a mirrored vtable.
//===----------------------------------------------------------------------===//

/// A host-supplied byte stream: read, write, end-of-file, failure. A save
/// entry point needs `write`/`is_failed`; a restore needs
/// `read`/`is_eof`/`is_failed` — the unused pair may be NULL. An entry
/// point taking this refuses with ZJOLT_RESULT_INVALID_ARGUMENT rather than
/// calling a missing callback.
typedef struct ZJoltStream {
  /// Fills `data` with exactly `size` bytes. A stream that cannot honour that
  /// reports it through `is_failed`, or `is_eof` when it simply ran out —
  /// zero-filling what it could not supply either way, so a caller that
  /// forgets to check reads defined bytes, not stale memory.
  void (*read)(void *user, void *data, size_t size);
  /// Writes exactly `size` bytes from `data`, or arranges for a later
  /// `is_failed` to report that it could not.
  void (*write)(void *user, const void *data, size_t size);
  /// True once a read has run out of input. Follows StreamIn::IsEOF's own
  /// convention: true only once a read asked for more than was left, not
  /// merely once the read position reaches the end.
  bool (*is_eof)(void *user);
  /// True once this stream has failed in either direction.
  bool (*is_failed)(void *user);
  void *user;
} ZJoltStream;

/// Which of Jolt's own object-stream formats a save writes — see
/// zjoltSceneSaveObjectStream / zjoltRagdollSettingsSaveObjectStream. A
/// restore has no counterpart: the format is sniffed from the stream's own
/// header, as Jolt's ObjectStreamIn::Open does.
typedef enum ZJoltObjectStreamFormat {
  /// Human-readable and diffable — every field written by name. The point
  /// of the feature; costs more bytes than the binary form.
  ZJOLT_OBJECT_STREAM_FORMAT_TEXT = 0,
  /// Packed, and closer in size to zjoltSceneSave's own binary form, at the
  /// cost of not being something a person can read or diff.
  ZJOLT_OBJECT_STREAM_FORMAT_BINARY = 1,
} ZJoltObjectStreamFormat;

//===----------------------------------------------------------------------===//
// Initialisation
//===----------------------------------------------------------------------===//

/// Host allocator, installed process-wide. Mirrors Jolt's five allocation
/// function pointers verbatim, so a host can budget physics memory itself.
///
/// `free`/`aligned_free` get only the block pointer (no size or alignment);
/// `reallocate` gets the old size. Record missing info in your own header.
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

/// Receives Jolt's diagnostic output, already formatted (formatting is done
/// on the C++ side so no host models a C varargs callback).
///
/// Optional but recommended: without it, messages fall back to stderr —
/// zjolt's fallback, not Jolt's own (Jolt's own default trace is a stub
/// that asserts, so an unhooked Jolt aborts on its first message).
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

/// Installs the allocator and hooks, creates Jolt's factory and registers
/// its types. Must be called before anything else; process-wide.
///
/// `desc` may be NULL for all defaults. ZJOLT_RESULT_CONFIG_MISMATCH if
/// this header was compiled with different layout-affecting settings than
/// the library.
static inline ZJoltResult zjoltInit(const ZJoltInitDesc *desc) {
  return zjoltInitWithConfig(desc, (uint32_t)ZJOLT_CONFIG_ID);
}

/// Unregisters Jolt's types, destroys the factory, restores Jolt's own
/// allocator. Safe when not initialised.
///
/// Refuses (tracing only) while any handle is alive: destroying one after
/// would free through an allocator never allocated from, corrupting the
/// heap with no nearby symptom. Check zjoltIsInitialized/zjoltLiveHandleCount.
ZJOLT_API void zjoltDeinit(void);

ZJOLT_API bool zjoltIsInitialized(void);

/// Every owning handle currently alive. zjoltDeinit refuses while non-zero.
///
/// Excludes Jolt-refcounted objects (shapes, materials, group filters,
/// skeletons, ragdoll settings) — release them anyway; Jolt owns their
/// counts. A ragdoll IS counted, moving only on create and final release,
/// not on every AddRef.
ZJOLT_API uint32_t zjoltLiveHandleCount(void);

//===----------------------------------------------------------------------===//
// Plain-data types. ZJoltVec3 is three floats, not four: Jolt's own Vec3 is
// a 16-byte SIMD register with a padding lane; projecting to 12 bytes costs
// one shuffle per crossing and saves every consumer modelling a lane it
// must never read.
//===----------------------------------------------------------------------===//

typedef struct ZJoltVec3 {
  float x, y, z;
} ZJoltVec3;

/// A quaternion in (x, y, z, w) order — w LAST, matching Jolt.
typedef struct ZJoltQuat {
  float x, y, z, w;
} ZJoltQuat;

//===----------------------------------------------------------------------===//
// 16-byte alignment, spelled for the STRUCT and written between `struct` and
// the tag, so one spelling serves all three compilers.
//
// It has to be the struct: Zig 0.16's translate-c drops `#pragma pack(pop)` on
// non-MSVC targets, so under mingw a member declared `_Alignas(16)` reads back
// as align 8 through `@cImport` while the compiled ABI is 16. Struct-level
// alignment survives that on all four targets [measured 2026-08-30], and
// `src/abi_check.zig` is the gate. C11's `_Alignas` cannot be attached to a
// struct definition at all, which is what put the member spelling here.
//===----------------------------------------------------------------------===//
#if defined(__cplusplus)
#define ZJOLT_ALIGN16 alignas(16)
#elif defined(_MSC_VER)
#define ZJOLT_ALIGN16 __declspec(align(16))
#elif defined(__GNUC__) || defined(__clang__)
#define ZJOLT_ALIGN16 __attribute__((aligned(16)))
#else
#error "zjolt needs a struct-level 16-byte alignment attribute for this compiler"
#endif

/// A 4x4 matrix in Jolt's own layout: four COLUMNS of four floats, so
/// column c's row r is `m[4 * c + r]`. Carries a transform (rotation in the
/// first three columns, translation in the fourth) or a 3x3 inertia tensor
/// padded to 4x4. 16-byte aligned, matching `JPH::Mat44` and `ZozzFloat4x4`,
/// so one buffer serves both packages; a struct embedding this one inherits
/// that alignment.
typedef struct ZJOLT_ALIGN16 ZJoltMat44 {
  float m[16];
} ZJoltMat44;

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

/// A world-space transform: ZJoltMat44 with ZJoltReal elements. Width
/// follows ZJOLT_DOUBLE_PRECISION (folded into ZJOLT_CONFIG_ID); a
/// consumer that disagrees about that setting disagrees about this
/// struct's size, and zjoltInit refuses it.
///
/// 16-byte aligned, matching `ZJoltMat44`.
typedef struct ZJOLT_ALIGN16 ZJoltRMat44 {
  ZJoltReal m[16];
} ZJoltRMat44;

typedef struct ZJoltAABox {
  ZJoltVec3 min;
  ZJoltVec3 max;
} ZJoltAABox;

/// Mass and inertia of a shape. `inertia` is a row-major 3x3 matrix.
typedef struct ZJoltMassProperties {
  float mass;
  float inertia[9];
} ZJoltMassProperties;

/// An 8-bit-per-channel colour. Used only to tint debug drawing — nothing in
/// the simulation reads one.
typedef struct ZJoltColor {
  uint8_t r, g, b, a;
} ZJoltColor;

/// Identifies a body within one physics system. Stable until the body is
/// destroyed; ids are recycled with a generation counter, so a stale id is
/// detected rather than aliasing a new body.
typedef uint32_t ZJoltBodyId;
#define ZJOLT_BODY_ID_INVALID ((ZJoltBodyId)0xffffffffu)

/// Identifies one leaf of a compound or mesh shape within a hit.
typedef uint32_t ZJoltSubShapeId;

/// The sub-shape id that means "the shape itself, no leaf". Required (not
/// optional) for any entry point taking a sub-shape id on a non-compound
/// shape — several of Jolt's accessors assert on an arbitrary value.
///
/// Written out, not reported by zjoltAbiLayout, because
/// `JPH::SubShapeID::cEmpty` is private; a behavioural test pins this value.
#define ZJOLT_SUB_SHAPE_ID_EMPTY ((ZJoltSubShapeId)0xffffffffu)

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
/// ZJOLT_ALLOWED_DOFS_ALL unless constraining to a plane. Crosses as a
/// uint32_t — this enum only names the bits, since a field typed as the
/// enum itself could not hold two bits combined.
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
  ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_MASS_AND_INERTIA = 0,
  /// Use ZJoltBodyDesc::mass, scale the shape's inertia to match.
  ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_INERTIA = 1,
  /// Use ZJoltBodyDesc::mass_properties_override outright; the shape's own
  /// mass and inertia are never computed.
  ZJOLT_OVERRIDE_MASS_PROPERTIES_MASS_AND_INERTIA_PROVIDED = 2,
} ZJoltOverrideMassProperties;

/// Shape kinds this ABI can produce, reported by zjoltShapeGetSubType and
/// meaningful after a restore. Two values are not one Jolt itself defines:
/// NONE (zero; not a shape, e.g. from a NULL handle) and USER_DEFINED (one
/// of Jolt's sixteen User1..UserConvex8 slots for a C++-registered type
/// this ABI cannot construct).
typedef enum ZJoltShapeSubType {
  ZJOLT_SHAPE_SUB_TYPE_NONE = 0,
  ZJOLT_SHAPE_SUB_TYPE_SPHERE = 1,
  ZJOLT_SHAPE_SUB_TYPE_BOX = 2,
  ZJOLT_SHAPE_SUB_TYPE_CAPSULE = 3,
  ZJOLT_SHAPE_SUB_TYPE_CONVEX_HULL = 4,
  ZJOLT_SHAPE_SUB_TYPE_MESH = 5,
  ZJOLT_SHAPE_SUB_TYPE_SCALED = 6,
  ZJOLT_SHAPE_SUB_TYPE_ROTATED_TRANSLATED = 7,
  ZJOLT_SHAPE_SUB_TYPE_OFFSET_CENTER_OF_MASS = 8,
  ZJOLT_SHAPE_SUB_TYPE_TRIANGLE = 9,
  ZJOLT_SHAPE_SUB_TYPE_CYLINDER = 10,
  ZJOLT_SHAPE_SUB_TYPE_TAPERED_CAPSULE = 11,
  ZJOLT_SHAPE_SUB_TYPE_TAPERED_CYLINDER = 12,
  ZJOLT_SHAPE_SUB_TYPE_STATIC_COMPOUND = 13,
  ZJOLT_SHAPE_SUB_TYPE_MUTABLE_COMPOUND = 14,
  ZJOLT_SHAPE_SUB_TYPE_HEIGHT_FIELD = 15,
  ZJOLT_SHAPE_SUB_TYPE_PLANE = 16,
  ZJOLT_SHAPE_SUB_TYPE_EMPTY = 17,
  /// Jolt's soft-body shape, which Jolt builds itself when a soft body is
  /// created. There is no shape constructor for one here, but it is named so
  /// that a shape read back off a body always has a name.
  ZJOLT_SHAPE_SUB_TYPE_SOFT_BODY = 18,
  ZJOLT_SHAPE_SUB_TYPE_USER_DEFINED = 19,
} ZJoltShapeSubType;

typedef enum ZJoltBackFaceMode {
  ZJOLT_BACK_FACE_MODE_IGNORE = 0,
  ZJOLT_BACK_FACE_MODE_COLLIDE = 1,
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
  ZJOLT_VALIDATE_RESULT_ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR = 0,
  ZJOLT_VALIDATE_RESULT_ACCEPT_CONTACT = 1,
  ZJOLT_VALIDATE_RESULT_REJECT_CONTACT = 2,
  ZJOLT_VALIDATE_RESULT_REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR = 3,
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

/// A collision shape. Reference counted, and shareable between bodies and
/// between systems. Immutable once built, with one deliberate exception: a
/// mutable compound, whose children move at run time through the
/// zjoltShapeMutableCompound* calls.
typedef struct ZJoltShape ZJoltShape;

/// The identity of a surface. Reference counted. See zjolt_material.h for what
/// it is and — more usefully — what it is not.
typedef struct ZJoltPhysicsMaterial ZJoltPhysicsMaterial;

/// A world: bodies, a broad phase, and the step.
typedef struct ZJoltPhysicsSystem ZJoltPhysicsSystem;

/// The scheduler a step runs its jobs on. See the JobSystem section.
typedef struct ZJoltJobSystem ZJoltJobSystem;

/// A body, borrowed for the duration of a body lock. Never outlives its lock.
typedef struct ZJoltBody ZJoltBody;

/// A shape-based character controller that is not a rigid body.
typedef struct ZJoltCharacter ZJoltCharacter;

/// Decides whether two collision groups collide. Reference counted like a
/// shape, and shareable between bodies and between systems.
typedef struct ZJoltGroupFilter ZJoltGroupFilter;

/// A host callback invoked before every collision step. Owned by the system it
/// was added to, and destroyed with it.
typedef struct ZJoltStepListener ZJoltStepListener;

/// Bodies sorted and staged for insertion, between zjoltBodyAddBatchPrepare
/// and the call that consumes it. Owned by the system.
typedef struct ZJoltBodyAddBatch ZJoltBodyAddBatch;

//===----------------------------------------------------------------------===//
// Job system: the step is parallel, and which threads it runs on is a host
// decision. v0.1 ships Jolt's own thread pool and a single-threaded
// scheduler; zjoltJobSystemCreateHost below adds a host scheduler as an
// ADDITIONAL constructor — why ZJoltJobSystem stayed opaque.
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

/// Called once when a worker thread starts or exits, with its index in
/// [0, num_threads). `user` is whatever zjoltJobSystemCreateThreadPoolWithHooks
/// was given.
typedef void (*ZJoltThreadHookFn)(void *user, int32_t thread_index);

/// Same pool as zjoltJobSystemCreateThreadPool, with per-thread init/exit
/// hooks reachable. Neither hook can be a setter added afterward: Jolt's
/// SetThreadInitFunction/SetThreadExitFunction only take effect before a
/// pool's threads start, so this constructs the pool, installs whichever
/// hook is non-NULL, then starts the threads. Either hook may be NULL;
/// `user` is passed to both.
ZJOLT_API ZJoltResult zjoltJobSystemCreateThreadPoolWithHooks(
    uint32_t max_jobs, uint32_t max_barriers, int32_t num_threads,
    ZJoltThreadHookFn thread_init, ZJoltThreadHookFn thread_exit, void *user,
    ZJoltJobSystem **out);

/// Resizes a thread-pool job system's worker count, stopping and restarting
/// its threads. ZJOLT_RESULT_INVALID_ARGUMENT if `job_system` was not created
/// by zjoltJobSystemCreateThreadPool or zjoltJobSystemCreateThreadPoolWithHooks
/// — the single-threaded and host-backed kinds have no thread count of their
/// own to resize.
ZJOLT_API ZJoltResult zjoltJobSystemSetNumThreads(ZJoltJobSystem *job_system,
                                                  int32_t num_threads);

/// Runs every job on the calling thread. Slower, but makes a step reproducible
/// without pinning a thread count — which is what the tests use.
ZJOLT_API ZJoltResult zjoltJobSystemCreateSingleThreaded(uint32_t max_jobs,
                                                         ZJoltJobSystem **out);

/// One unit of the step's work, handed to a host scheduler by
/// ZJoltHostJobSystem::queue_job(s) and run back on the library through
/// zjoltJobRun. Reference counted like a shape, but by Jolt's own job system
/// rather than this library's: see the lifetime note on ZJoltHostJobSystem.
typedef struct ZJoltJob ZJoltJob;

/// A host's own task graph: Jolt's step jobs run on it instead of Jolt's
/// own pool (see zjoltJobSystemCreateHost).
///
/// `job` is alive only for the callback's duration — AddRef before running
/// it later or elsewhere, release after. May run on one of Jolt's own
/// workers; nothing may unwind out of either callback.
typedef struct ZJoltHostJobSystem {
  /// Maximum jobs the host's scheduler can actually run at once; Jolt sizes
  /// its own step graph from this.
  uint32_t (*get_max_concurrency)(void *user);
  /// Schedule `job` to run. May run it inline before returning, or queue it
  /// for another thread — either is a valid scheduler.
  void (*queue_job)(void *user, ZJoltJob *job);
  /// The batch form of queue_job, for jobs whose dependency counts cleared
  /// together. Equivalent to calling queue_job once per element in order, but
  /// lets a scheduler that batches its own submissions do that instead.
  void (*queue_jobs)(void *user, ZJoltJob *const *jobs, uint32_t count);
  void *user;
} ZJoltHostJobSystem;

/// Wraps `host` in a job system Jolt's step can run on. `max_barriers`
/// sizes Jolt's OWN barrier bookkeeping (@see ZJOLT_MAX_PHYSICS_BARRIERS);
/// nothing about a barrier is implemented by the host. Every field of
/// `host` is required.
ZJOLT_API ZJoltResult zjoltJobSystemCreateHost(const ZJoltHostJobSystem *host,
                                               uint32_t max_barriers,
                                               ZJoltJobSystem **out);

/// Runs `job`'s function on the calling thread. Tolerates NULL. May call
/// back into ZJoltHostJobSystem::queue_job(s) before returning (running
/// this job can clear another one's last dependency, synchronously, on
/// THIS thread) — the no-unwinding rule applies here too.
ZJOLT_API void zjoltJobRun(ZJoltJob *job);

/// Takes a reference, keeping `job` alive past the queue_job(s) call that
/// handed it to the host. Tolerates NULL.
ZJOLT_API void zjoltJobAddRef(ZJoltJob *job);

/// Releases a reference taken by queue_job(s) or zjoltJobAddRef. Call this
/// exactly once for each — including once for the reference queue_job(s)
/// itself took, after zjoltJobRun returns. Tolerates NULL.
ZJOLT_API void zjoltJobRelease(ZJoltJob *job);

ZJOLT_API void zjoltJobSystemDestroy(ZJoltJobSystem *job_system);

/// Number of jobs this scheduler can run at once. 1 for the single-threaded
/// scheduler.
ZJOLT_API uint32_t zjoltJobSystemGetMaxConcurrency(const ZJoltJobSystem *job_system);

//===----------------------------------------------------------------------===//
// Factory: Jolt's process-wide type registry, keyed by name and hash.
// zjoltInit registers every Jolt-defined type; hosts cannot register their
// own. zjoltFactoryFind* and zjoltFactoryGetAllClasses read a type's name,
// hash, size, and whether it is abstract.
//===----------------------------------------------------------------------===//

/// What Factory::Find or GetAllClasses reports about one registered type.
/// `name` is borrowed, never NULL for a real entry (a string literal alive
/// for the process). All-zero (name NULL) means nothing is registered
/// under that name/hash.
typedef struct ZJoltRttiInfo {
  const char *name;
  uint32_t hash;
  int32_t size;
  /// True if RTTI::CreateObject would return NULL — not reachable from
  /// here either way.
  bool is_abstract;
} ZJoltRttiInfo;

/// Looks a registered type up by name. All-zero if nothing is registered
/// under it, or before zjoltInit.
ZJOLT_API void zjoltFactoryFind(const char *name, ZJoltRttiInfo *out);

/// Same lookup, by the hash zjoltFactoryFind's own result carries — the
/// cheaper comparison a saved stream's type tag actually uses.
ZJOLT_API void zjoltFactoryFindByHash(uint32_t hash, ZJoltRttiInfo *out);

/// Every type this library's zjoltInit registered with Jolt's Factory.
/// Two-call protocol: `out` NULL reports the count in `*out_count`.
ZJOLT_API ZJoltResult zjoltFactoryGetAllClasses(ZJoltRttiInfo *out,
                                                uint32_t capacity,
                                                uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Build report: covers what the macros below cannot show from the header
// alone — ZJoltReal/ZJoltObjectLayer widths a mismatched build presents
// with an identical header but different structs. zjoltAbiLayout reports
// them; zjoltInit separately refuses a ZJOLT_CONFIG_ID mismatch outright.
//===----------------------------------------------------------------------===//

/// Bits of ZJoltAbiLayout::build_flags. These do not change any layout — they
/// are here so a consumer can report, or refuse, a build it did not expect.
#define ZJOLT_BUILD_FLAG_DOUBLE_PRECISION (1u << 0)
#define ZJOLT_BUILD_FLAG_OBJECT_LAYER_32 (1u << 1)
#define ZJOLT_BUILD_FLAG_ASSERTS_ENABLED (1u << 2)
#define ZJOLT_BUILD_FLAG_CROSS_PLATFORM_DETERMINISTIC (1u << 3)

/// What this library was built as, for a consumer that cannot compare the
/// header against its own declarations by reflection. `config_id` is the
/// value zjoltInit itself refuses a mismatch on, so a host can check it
/// up front and report a useful message instead of an error code.
typedef struct ZJoltAbiLayout {
  /// sizeof(ZJoltAbiLayout). Read this first: if it disagrees with the
  /// consumer's own sizeof, the struct itself has changed and no field below
  /// can be trusted.
  uint32_t layout_size;
  uint32_t config_id;
  uint32_t build_flags;

  /// Widths the build options decide. Everything that carries a position or an
  /// object layer changes shape with these, so a consumer that disagrees here
  /// disagrees about most of the ABI.
  uint32_t real_size;
  uint32_t object_layer_size;

  /// Alignment the default allocator guarantees, for a host bridging its own.
  uint32_t default_allocate_alignment;
} ZJoltAbiLayout;

/// Fills `out` with the layout the library was compiled with. Never fails.
ZJOLT_API void zjoltAbiLayout(ZJoltAbiLayout *out);

//===----------------------------------------------------------------------===//
// Floating-point control word: forces denormal results on the calling
// thread to flush to zero, so a deterministic build stays deterministic
// across a call into third-party code that leaves the rounding/flush mode
// changed. Pair every Enter with a Leave, LIFO, on the same thread; a no-op
// on a target with no floating-point control word (WASM, RISC-V, PPC,
// LoongArch).
//===----------------------------------------------------------------------===//

/// Opaque: the calling thread's control word before
/// zjoltFPFlushDenormalsEnter ran. Meaningful only to
/// zjoltFPFlushDenormalsLeave.
typedef struct ZJoltFPControlWordState {
  uint64_t reserved[1];
} ZJoltFPControlWordState;

/// Tolerates NULL.
ZJOLT_API void zjoltFPFlushDenormalsEnter(ZJoltFPControlWordState *out);

/// Restores the state `state` recorded. Tolerates NULL.
ZJOLT_API void zjoltFPFlushDenormalsLeave(ZJoltFPControlWordState *state);

//===----------------------------------------------------------------------===//
// External profiler bridge: gated behind -Dexternal_profile
// (JPH_EXTERNAL_PROFILE). Every entry point exists regardless, reporting
// ZJOLT_RESULT_UNSUPPORTED when off. Once installed, `start`/`end` are
// called at the start and end of every JPH_PROFILE scope Jolt's own source
// (and this library's) places, until zjoltExternalProfilerClearHooks runs.
//===----------------------------------------------------------------------===//

/// Called when a profiled scope starts. `user_data` is 64 bytes, zeroed by
/// the caller before this runs, for the host to carry state to the matching
/// zjoltExternalProfilerEndFn call.
typedef void (*ZJoltExternalProfilerStartFn)(void *user, const char *name,
                                             uint32_t color,
                                             uint8_t *user_data);

/// Called when the scope that started with the same `user_data` ends.
typedef void (*ZJoltExternalProfilerEndFn)(void *user, uint8_t *user_data);

/// Installs `start`/`end`; both required. ZJOLT_RESULT_INVALID_ARGUMENT if
/// either is NULL. ZJOLT_RESULT_UNSUPPORTED unless built with
/// -Dexternal_profile.
ZJOLT_API ZJoltResult zjoltExternalProfilerSetHooks(
    ZJoltExternalProfilerStartFn start, ZJoltExternalProfilerEndFn end,
    void *user);

/// Uninstalls the hooks; a scope measured after this call is a no-op.
ZJOLT_API void zjoltExternalProfilerClearHooks(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_CORE_H_
