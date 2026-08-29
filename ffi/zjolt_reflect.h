//===----------------------------------------------------------------------===//
// zjolt — RTTI over Jolt's own registered types, ObjectStream for any of
// them, and the profiler.
//
// RTTI: opaque `ZJoltRtti` tags, walked and cast without touching a vtable
// Jolt has not registered.
//
// ObjectStream: read/write a registered settings object, text or binary,
// through the same ZJoltStream seam zjoltSceneSaveObjectStream uses.
//
// Profiler: gated behind -Dprofile; every entry point exists regardless and
// returns ZJOLT_RESULT_UNSUPPORTED when it is off.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_REFLECT_H_
#define ZJOLT_REFLECT_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// RTTI
//===----------------------------------------------------------------------===//

/// A registered type's runtime type information. Opaque; never NULL for a
/// successful lookup. Borrowed — Jolt owns every RTTI it registers for the
/// life of the process, so nothing here releases one.
typedef struct ZJoltRtti ZJoltRtti;

/// Looks a registered type up by name, as zjoltFactoryFind does, but returns
/// a handle a later zjoltReflect* call can walk rather than a snapshot.
/// ZJOLT_RESULT_INVALID_ARGUMENT if nothing is registered under `name`.
ZJOLT_API ZJoltResult zjoltReflectFind(const char *name, const ZJoltRtti **out);

/// As zjoltReflectFind, by the hash zjoltFactoryFind's result carries.
ZJOLT_API ZJoltResult zjoltReflectFindByHash(uint32_t hash, const ZJoltRtti **out);

/// Fills `out` as zjoltFactoryFind would. All-zero for a NULL `rtti`.
ZJOLT_API void zjoltReflectGetInfo(const ZJoltRtti *rtti, ZJoltRttiInfo *out);

/// Direct base classes only — not flattened through the base's own bases.
/// 0 for a NULL `rtti`.
ZJOLT_API uint32_t zjoltReflectGetBaseClassCount(const ZJoltRtti *rtti);

/// One direct base class and the byte offset of its subobject within the
/// derived type.
typedef struct ZJoltReflectBaseClass {
  ZJoltRttiInfo rtti;
  int32_t offset;
} ZJoltReflectBaseClass;

/// ZJOLT_RESULT_INVALID_ARGUMENT past zjoltReflectGetBaseClassCount.
ZJOLT_API ZJoltResult zjoltReflectGetBaseClass(const ZJoltRtti *rtti,
                                               uint32_t index,
                                               ZJoltReflectBaseClass *out);

/// True if `rtti` is `base`, or derived from it, transitively. False for a
/// NULL argument on either side.
ZJOLT_API bool zjoltReflectIsKindOf(const ZJoltRtti *rtti, const ZJoltRtti *base);

/// Builds an instance of `rtti` through Jolt's own factory function.
/// ZJOLT_RESULT_INVALID_ARGUMENT if `rtti` is abstract, or is not a
/// JPH::SerializableObject descendant (the only kind zjoltReflectCastTo and
/// zjoltReflectObjectGetRtti can safely call a virtual through).
ZJOLT_API ZJoltResult zjoltReflectCreateObject(const ZJoltRtti *rtti, void **out);

/// Destroys an object zjoltReflectCreateObject or zjoltReflectObjectRestoreStream
/// returned. Owned like any other *Destroy in this ABI, not reference
/// counted, even when the underlying type also happens to be. Tolerates
/// NULL either argument.
ZJOLT_API void zjoltReflectDestroyObject(const ZJoltRtti *rtti, void *object);

/// `object`'s own dynamic RTTI. `object` must be a JPH::SerializableObject
/// descendant, as zjoltReflectCreateObject enforces at construction. NULL
/// for a NULL `object`.
ZJOLT_API const ZJoltRtti *zjoltReflectObjectGetRtti(const void *object);

/// The one checked-cast entry point standing in for CastTo/StaticCast/
/// DynamicCast: succeeds when `object`'s real type is `target` or is
/// transitively derived from it, and fails otherwise — a failure is always
/// ZJOLT_RESULT_INVALID_ARGUMENT with `*out` left NULL, never a pointer of
/// the wrong type.
ZJOLT_API ZJoltResult zjoltReflectCastTo(void *object, const ZJoltRtti *target,
                                         void **out);

//===----------------------------------------------------------------------===//
// ObjectStream — member access
//
// Requires -Dobject_stream=true (on by default); ZJOLT_RESULT_UNSUPPORTED
// otherwise, the same contract debug_renderer's entry points use.
//===----------------------------------------------------------------------===//

/// The primitive kinds ObjectStream itself distinguishes (mirrors
/// Jolt/ObjectStream/ObjectStreamTypes.h), plus OBJECT for a member that is
/// a nested class instance or pointer rather than one of these.
typedef enum ZJoltPrimitiveType {
  ZJOLT_PRIMITIVE_TYPE_UINT8 = 0,
  ZJOLT_PRIMITIVE_TYPE_UINT16 = 1,
  ZJOLT_PRIMITIVE_TYPE_INT32 = 2,
  ZJOLT_PRIMITIVE_TYPE_UINT32 = 3,
  ZJOLT_PRIMITIVE_TYPE_UINT64 = 4,
  ZJOLT_PRIMITIVE_TYPE_FLOAT = 5,
  ZJOLT_PRIMITIVE_TYPE_BOOL = 6,
  ZJOLT_PRIMITIVE_TYPE_STRING = 7,
  ZJOLT_PRIMITIVE_TYPE_FLOAT3 = 8,
  ZJOLT_PRIMITIVE_TYPE_VEC3 = 9,
  ZJOLT_PRIMITIVE_TYPE_VEC4 = 10,
  ZJOLT_PRIMITIVE_TYPE_QUAT = 11,
  ZJOLT_PRIMITIVE_TYPE_MAT44 = 12,
  ZJOLT_PRIMITIVE_TYPE_DOUBLE = 13,
  ZJOLT_PRIMITIVE_TYPE_DVEC3 = 14,
  ZJOLT_PRIMITIVE_TYPE_DMAT44 = 15,
  ZJOLT_PRIMITIVE_TYPE_DOUBLE3 = 16,
  ZJOLT_PRIMITIVE_TYPE_FLOAT4 = 17,
  ZJOLT_PRIMITIVE_TYPE_UVEC4 = 18,
  /// A nested object, by value or by (ref-counted) pointer. No typed
  /// accessor below reads or writes one; use zjoltReflectObjectSaveStream/
  /// RestoreStream on the whole owning object instead.
  ZJOLT_PRIMITIVE_TYPE_OBJECT = 19,
} ZJoltPrimitiveType;

/// One serialized member: GetMemberPointer's offset plus GetMemberPrimitiveType's
/// kind, so a typed accessor below can validate before it touches memory.
typedef struct ZJoltReflectAttribute {
  /// Borrowed; alive for the process.
  const char *name;
  ZJoltPrimitiveType primitive_type;
  uint32_t offset;
} ZJoltReflectAttribute;

/// Flattened across every base class, in the order Jolt itself walks them.
ZJOLT_API ZJoltResult zjoltReflectGetAttributeCount(const ZJoltRtti *rtti,
                                                    uint32_t *out_count);

/// ZJOLT_RESULT_INVALID_ARGUMENT past zjoltReflectGetAttributeCount.
ZJOLT_API ZJoltResult zjoltReflectGetAttribute(const ZJoltRtti *rtti,
                                               uint32_t index,
                                               ZJoltReflectAttribute *out);

//===----------------------------------------------------------------------===//
// Plain-data types for the typed accessors below that ZJoltVec3/ZJoltQuat/
// ZJoltMat44 do not already cover.
//===----------------------------------------------------------------------===//

/// A four-component float vector, no padding lane. Wire type for both
/// ZJOLT_PRIMITIVE_TYPE_VEC4 (JPH::Vec4) and _FLOAT4 (JPH::Float4)
/// attributes; the attribute's primitive_type says which one it is.
typedef struct ZJoltVec4 {
  float x, y, z, w;
} ZJoltVec4;

/// A four-component unsigned-integer vector, no padding lane. Wire type for
/// ZJOLT_PRIMITIVE_TYPE_UVEC4 (JPH::UVec4) attributes.
typedef struct ZJoltUVec4 {
  uint32_t x, y, z, w;
} ZJoltUVec4;

/// A three-component double-precision vector. Always double, unlike
/// ZJoltRVec3 whose width follows ZJOLT_DOUBLE_PRECISION. Wire type for both
/// ZJOLT_PRIMITIVE_TYPE_DVEC3 (JPH::DVec3) and _DOUBLE3 (JPH::Double3).
typedef struct ZJoltDVec3 {
  double x, y, z;
} ZJoltDVec3;

/// A 4x4 matrix, sixteen doubles in ZJoltMat44's column-major layout.
/// Always double, unlike ZJoltRMat44 whose width follows
/// ZJOLT_DOUBLE_PRECISION. Wire type for ZJOLT_PRIMITIVE_TYPE_DMAT44
/// (JPH::DMat44) attributes.
typedef struct ZJoltDMat44 {
  double m[16];
} ZJoltDMat44;

//===----------------------------------------------------------------------===//
// Typed member access
//
// Each pair reads or writes exactly `attr`'s member on `object`, refusing
// with ZJOLT_RESULT_INVALID_ARGUMENT when `attr->primitive_type` names a
// different kind — the check that keeps a host from writing four bytes into
// a one-byte member through a stale or mismatched ZJoltReflectAttribute.
//===----------------------------------------------------------------------===//

ZJOLT_API ZJoltResult zjoltReflectAttributeReadUint8(const ZJoltReflectAttribute *attr, const void *object, uint8_t *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteUint8(const ZJoltReflectAttribute *attr, void *object, uint8_t value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadUint16(const ZJoltReflectAttribute *attr, const void *object, uint16_t *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteUint16(const ZJoltReflectAttribute *attr, void *object, uint16_t value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadInt32(const ZJoltReflectAttribute *attr, const void *object, int32_t *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteInt32(const ZJoltReflectAttribute *attr, void *object, int32_t value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadUint32(const ZJoltReflectAttribute *attr, const void *object, uint32_t *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteUint32(const ZJoltReflectAttribute *attr, void *object, uint32_t value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadUint64(const ZJoltReflectAttribute *attr, const void *object, uint64_t *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteUint64(const ZJoltReflectAttribute *attr, void *object, uint64_t value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadFloat(const ZJoltReflectAttribute *attr, const void *object, float *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteFloat(const ZJoltReflectAttribute *attr, void *object, float value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadDouble(const ZJoltReflectAttribute *attr, const void *object, double *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteDouble(const ZJoltReflectAttribute *attr, void *object, double value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadBool(const ZJoltReflectAttribute *attr, const void *object, bool *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteBool(const ZJoltReflectAttribute *attr, void *object, bool value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadVec3(const ZJoltReflectAttribute *attr, const void *object, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteVec3(const ZJoltReflectAttribute *attr, void *object, ZJoltVec3 value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadQuat(const ZJoltReflectAttribute *attr, const void *object, ZJoltQuat *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteQuat(const ZJoltReflectAttribute *attr, void *object, ZJoltQuat value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadMat44(const ZJoltReflectAttribute *attr, const void *object, ZJoltMat44 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteMat44(const ZJoltReflectAttribute *attr, void *object, ZJoltMat44 value);

/// Borrowed, null-terminated; valid until this attribute is next written on
/// `object` or `object` is destroyed. ZJOLT_RESULT_INVALID_ARGUMENT if
/// `attr` names a different kind.
ZJOLT_API ZJoltResult zjoltReflectAttributeReadString(const ZJoltReflectAttribute *attr, const void *object, const char **out);
/// Copies `value` in; `value` is read, not kept. ZJOLT_RESULT_INVALID_ARGUMENT
/// if `attr` names a different kind, or if `value` is NULL.
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteString(const ZJoltReflectAttribute *attr, void *object, const char *value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadFloat3(const ZJoltReflectAttribute *attr, const void *object, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteFloat3(const ZJoltReflectAttribute *attr, void *object, ZJoltVec3 value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadFloat4(const ZJoltReflectAttribute *attr, const void *object, ZJoltVec4 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteFloat4(const ZJoltReflectAttribute *attr, void *object, ZJoltVec4 value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadVec4(const ZJoltReflectAttribute *attr, const void *object, ZJoltVec4 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteVec4(const ZJoltReflectAttribute *attr, void *object, ZJoltVec4 value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadUVec4(const ZJoltReflectAttribute *attr, const void *object, ZJoltUVec4 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteUVec4(const ZJoltReflectAttribute *attr, void *object, ZJoltUVec4 value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadDouble3(const ZJoltReflectAttribute *attr, const void *object, ZJoltDVec3 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteDouble3(const ZJoltReflectAttribute *attr, void *object, ZJoltDVec3 value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadDVec3(const ZJoltReflectAttribute *attr, const void *object, ZJoltDVec3 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteDVec3(const ZJoltReflectAttribute *attr, void *object, ZJoltDVec3 value);

ZJOLT_API ZJoltResult zjoltReflectAttributeReadDMat44(const ZJoltReflectAttribute *attr, const void *object, ZJoltDMat44 *out);
ZJOLT_API ZJoltResult zjoltReflectAttributeWriteDMat44(const ZJoltReflectAttribute *attr, void *object, ZJoltDMat44 value);

//===----------------------------------------------------------------------===//
// ObjectStream — whole objects
//===----------------------------------------------------------------------===//

/// Writes `object` (an instance of `rtti`) through `stream` in Jolt's
/// object-stream format. `object` is read, not kept. ZJOLT_RESULT_IO_ERROR on
/// a stream failure, ZJOLT_RESULT_BAD_FORMAT if Jolt's writer refuses the
/// object.
ZJOLT_API ZJoltResult zjoltReflectObjectSaveStream(const ZJoltRtti *rtti,
                                                   const void *object,
                                                   ZJoltObjectStreamFormat format,
                                                   const ZJoltStream *stream);

/// Reads an object of `rtti`'s type (or a type derived from it) back from
/// `stream`; text or binary is sniffed from the stream's own header, as
/// Jolt's own reader does. Release with zjoltReflectDestroyObject.
/// ZJOLT_RESULT_BAD_FORMAT if the header or the object is not recognised.
ZJOLT_API ZJoltResult zjoltReflectObjectRestoreStream(const ZJoltRtti *rtti,
                                                      const ZJoltStream *stream,
                                                      void **out);

//===----------------------------------------------------------------------===//
// Profiler
//
// Gated behind -Dprofile (JPH_PROFILE_ENABLED). Every entry point below is
// declared regardless and returns ZJOLT_RESULT_UNSUPPORTED when the option
// is off, the contract ffi/zjolt_debug.cpp uses for -Ddebug_renderer.
//
// Jolt's own Profiler::Dump/DumpChart write "profile_chart_<tag>.html" to
// disk with no accessible seam to redirect that write, so this binding never
// calls either. zjoltProfilerDumpStream reads the calling thread's own
// recorded samples directly and hands them to the host through `stream` —
// one entry point standing in for both, immediately rather than deferred.
//===----------------------------------------------------------------------===//

/// Ends the current profiling frame: rotates every profiling thread's sample
/// buffer, as JPH_PROFILE_NEXTFRAME() would. Creates the process-wide
/// profiler on first call.
ZJOLT_API ZJoltResult zjoltProfilerNextFrame(void);

/// Tears down everything a lazy call above created: the calling thread's
/// zjoltProfileThreadBegin, if any, and the process-wide profiler singleton.
///
/// A ProfileThread or ZJoltProfileMeasurement still active on ANOTHER thread
/// is left DANGLING. Safe to call when nothing was started, and independent
/// of zjoltInit/zjoltDeinit either way.
ZJOLT_API void zjoltProfilerShutdown(void);

/// Writes the calling thread's recorded samples through `stream`: `tag`,
/// then a count, then each sample's name, color and start/end tick. See the
/// section comment for why this replaces Dump and DumpChart together.
/// ZJOLT_RESULT_INVALID_ARGUMENT if the calling thread has no zjoltProfileThreadBegin
/// in effect. `tag` may be NULL.
ZJOLT_API ZJoltResult zjoltProfilerDumpStream(const char *tag,
                                              const ZJoltStream *stream);

/// Ticks of the host's processor clock, of no defined epoch — only
/// differences between two calls are meaningful.
ZJOLT_API ZJoltResult zjoltProfilerGetProcessorTickCount(uint64_t *out);

/// An estimate of ticks per second, accurate only since the profiler's
/// current reference point (the last zjoltProfilerNextFrame, or the profiler's
/// creation). Creates the process-wide profiler on first call.
ZJOLT_API ZJoltResult zjoltProfilerGetProcessorTicksPerSecond(uint64_t *out);

/// Perceived brightness of `color`, 0-255. Mirrors JPH::Color::GetIntensity;
/// useful for choosing readable text over a chart bar tinted by a sample's
/// color. Always available — Color carries no profiler dependency.
ZJOLT_API uint8_t zjoltProfilerColorGetIntensity(ZJoltColor color);

/// Registers the calling thread for instrumentation, as
/// JPH_PROFILE_THREAD_START(name) would. Every zjoltProfileMeasurementBegin
/// on this thread needs one in effect; refused
/// (ZJOLT_RESULT_ALREADY_INITIALIZED) while one is already active on this
/// thread. Creates the process-wide profiler on first call.
ZJOLT_API ZJoltResult zjoltProfileThreadBegin(const char *name);

/// Unregisters the calling thread. Every zjoltProfileMeasurementBegin/End
/// pair on this thread must be closed first. A no-op with none active.
ZJOLT_API void zjoltProfileThreadEnd(void);

/// One scoped timing sample, begun here and ended by zjoltProfileMeasurementEnd
/// rather than by a destructor crossing the ABI.
typedef struct ZJoltProfileMeasurement ZJoltProfileMeasurement;

/// Starts timing. Silently records nothing (still returns a valid handle) if
/// the calling thread has no zjoltProfileThreadBegin in effect, matching
/// JPH::ProfileMeasurement's own "thread not instrumented" behaviour.
ZJOLT_API ZJoltResult zjoltProfileMeasurementBegin(const char *name,
                                                   uint32_t color,
                                                   ZJoltProfileMeasurement **out);

/// Stops timing and records the sample. Tolerates NULL.
ZJOLT_API void zjoltProfileMeasurementEnd(ZJoltProfileMeasurement *measurement);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_REFLECT_H_
