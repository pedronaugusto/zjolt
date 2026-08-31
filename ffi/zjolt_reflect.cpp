//===----------------------------------------------------------------------===//
// zjolt — RTTI, ObjectStream and the profiler.
//
// ZJoltRtti and the objects zjoltReflectCreateObject/RestoreStream hand back
// are reinterpret_cast tags onto JPH::RTTI and JPH::SerializableObject, on
// the same terms as ZJoltShape in zjolt_internal.h: never completed, never
// dereferenced as the tag. RTTI::CastTo never dereferences its object
// argument either, which is what lets BaseClassOffset below recover a byte
// offset RTTI has no public accessor for.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include "zjolt_reflect.h"

#include <Jolt/Core/Color.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/RTTI.h>
#include <Jolt/Core/TickCounter.h>
#include <Jolt/Math/UVec4.h>
#include <Jolt/ObjectStream/ObjectStreamBinaryIn.h>
#include <Jolt/ObjectStream/ObjectStreamBinaryOut.h>
#include <Jolt/ObjectStream/ObjectStreamTextIn.h>
#include <Jolt/ObjectStream/ObjectStreamTextOut.h>
#include <Jolt/ObjectStream/SerializableObject.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>

#ifdef JPH_PROFILE_ENABLED
#include <Jolt/Core/Profiler.h>
#endif

#include <cstring>

namespace zjolt {

inline const JPH::RTTI *ToJolt(const ZJoltRtti *rtti) {
  return reinterpret_cast<const JPH::RTTI *>(rtti);
}
inline const ZJoltRtti *ToC(const JPH::RTTI *rtti) {
  return reinterpret_cast<const ZJoltRtti *>(rtti);
}

}  // namespace zjolt

namespace {

//===----------------------------------------------------------------------===//
// RTTI
//===----------------------------------------------------------------------===//

void FillRttiInfo(const JPH::RTTI *rtti, ZJoltRttiInfo *out) {
  if (out == nullptr) return;
  if (rtti == nullptr) {
    *out = ZJoltRttiInfo{};
    return;
  }
  out->name = rtti->GetName();
  out->hash = rtti->GetHash();
  out->size = static_cast<int32_t>(rtti->GetSize());
  out->is_abstract = rtti->IsAbstract();
}

/// RTTI has no public accessor for a base class's byte offset — only the
/// private BaseClass::mOffset AddBaseClass records. CastTo walks the same
/// offsets without ever dereferencing its object argument (see RTTI.cpp), so
/// a fake non-null address recovers one exactly the way JPH_BASE_CLASS_OFFSET
/// (Core/RTTI.h) recovers a compile-time one.
int32_t BaseClassOffset(const JPH::RTTI *derived, const JPH::RTTI *base) {
  constexpr uintptr_t kFakeBase = 0x10000;
  const void *casted = derived->CastTo(reinterpret_cast<const void *>(kFakeBase), base);
  return casted != nullptr
             ? static_cast<int32_t>(reinterpret_cast<uintptr_t>(casted) - kFakeBase)
             : 0;
}

/// Whether zjoltReflectCreateObject/CastTo/ObjectGetRtti may call a virtual
/// through `rtti`: only sound for a JPH::SerializableObject descendant, the
/// base every one of Jolt's settings classes declares its RTTI virtual
/// through. Safe to ask regardless of -Dobject_stream — SerializableObject's
/// class and RTTI exist either way; only its ObjectStream I/O methods do not.
bool IsSerializableObjectKind(const JPH::RTTI *rtti) {
  // Unqualified: JPH_DECLARE_RTTI_ABSTRACT_BASE declares SerializableObject's
  // GetRTTIOfType as a hidden friend, findable only through argument-dependent
  // lookup — JPH::GetRTTIOfType(...) does not see it.
  return rtti->IsKindOf(GetRTTIOfType(static_cast<JPH::SerializableObject *>(nullptr)));
}

//===----------------------------------------------------------------------===//
// Conversions for the ObjectStream primitive kinds ZJoltVec3/ZJoltQuat/
// ZJoltMat44 and zjolt_internal.h's ToC/ToJolt do not already cover: four
// SIMD types crossed the same way zjolt_internal.h crosses Vec3/Mat44 —
// through JPH's own accessors, never a raw memcpy of the register.
//===----------------------------------------------------------------------===//

ZJoltVec4 ToC(JPH::Vec4Arg v) { return ZJoltVec4{v.GetX(), v.GetY(), v.GetZ(), v.GetW()}; }
JPH::Vec4 ToJolt(const ZJoltVec4 &v) { return JPH::Vec4(v.x, v.y, v.z, v.w); }

ZJoltUVec4 ToC(JPH::UVec4Arg v) {
  return ZJoltUVec4{v.GetX(), v.GetY(), v.GetZ(), v.GetW()};
}
JPH::UVec4 ToJolt(const ZJoltUVec4 &v) {
  return JPH::UVec4(v.x, v.y, v.z, v.w);
}

/// DVec3's 4th lane mirrors its 3rd (see JPH::DVec3), so only x/y/z round trip.
ZJoltDVec3 ToC(JPH::DVec3Arg v) { return ZJoltDVec3{v.GetX(), v.GetY(), v.GetZ()}; }
JPH::DVec3 ToJolt(const ZJoltDVec3 &v) { return JPH::DVec3(v.x, v.y, v.z); }

/// JPH::DMat44 is three float columns plus a double-precision translation —
/// the same shape zjolt_internal.h's ToCR/ToJoltR cross for ZJoltRMat44, but
/// DMat44 is double width regardless of ZJOLT_DOUBLE_PRECISION.
ZJoltDMat44 ToC(JPH::DMat44Arg m) {
  ZJoltDMat44 out{};
  for (JPH::uint col = 0; col < 3; ++col) {
    const JPH::Vec4 column = m.GetColumn4(col);
    out.m[4 * col + 0] = column.GetX();
    out.m[4 * col + 1] = column.GetY();
    out.m[4 * col + 2] = column.GetZ();
    out.m[4 * col + 3] = column.GetW();
  }
  const JPH::DVec3 translation = m.GetTranslation();
  out.m[12] = translation.GetX();
  out.m[13] = translation.GetY();
  out.m[14] = translation.GetZ();
  out.m[15] = 1.0;
  return out;
}
JPH::DMat44 ToJolt(const ZJoltDMat44 &m) {
  const JPH::Vec4 col0(static_cast<float>(m.m[0]), static_cast<float>(m.m[1]),
                       static_cast<float>(m.m[2]), static_cast<float>(m.m[3]));
  const JPH::Vec4 col1(static_cast<float>(m.m[4]), static_cast<float>(m.m[5]),
                       static_cast<float>(m.m[6]), static_cast<float>(m.m[7]));
  const JPH::Vec4 col2(static_cast<float>(m.m[8]), static_cast<float>(m.m[9]),
                       static_cast<float>(m.m[10]), static_cast<float>(m.m[11]));
  const JPH::DVec3 translation(m.m[12], m.m[13], m.m[14]);
  return JPH::DMat44(col0, col1, col2, translation);
}

#ifdef JPH_OBJECT_STREAM

//===----------------------------------------------------------------------===//
// ObjectStream — member access
//===----------------------------------------------------------------------===//

/// The RTTI Jolt registers for one of ZJoltPrimitiveType's own kinds, fully
/// qualified rather than through the JPH_RTTI(...) macro: that macro relies
/// on argument-dependent lookup finding JPH::GetRTTIOfType, which does not
/// happen for a fundamental type such as int or float — it has no associated
/// namespace to look in.
const JPH::RTTI *PrimitiveRtti(ZJoltPrimitiveType type) {
  switch (type) {
    case ZJOLT_PRIMITIVE_TYPE_UINT8:
      return JPH::GetRTTIOfType(static_cast<JPH::uint8 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_UINT16:
      return JPH::GetRTTIOfType(static_cast<JPH::uint16 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_INT32:
      return JPH::GetRTTIOfType(static_cast<int *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_UINT32:
      return JPH::GetRTTIOfType(static_cast<JPH::uint32 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_UINT64:
      return JPH::GetRTTIOfType(static_cast<JPH::uint64 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_FLOAT:
      return JPH::GetRTTIOfType(static_cast<float *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_BOOL:
      return JPH::GetRTTIOfType(static_cast<bool *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_STRING:
      return JPH::GetRTTIOfType(static_cast<JPH::String *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_FLOAT3:
      return JPH::GetRTTIOfType(static_cast<JPH::Float3 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_VEC3:
      return JPH::GetRTTIOfType(static_cast<JPH::Vec3 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_VEC4:
      return JPH::GetRTTIOfType(static_cast<JPH::Vec4 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_QUAT:
      return JPH::GetRTTIOfType(static_cast<JPH::Quat *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_MAT44:
      return JPH::GetRTTIOfType(static_cast<JPH::Mat44 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_DOUBLE:
      return JPH::GetRTTIOfType(static_cast<double *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_DVEC3:
      return JPH::GetRTTIOfType(static_cast<JPH::DVec3 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_DMAT44:
      return JPH::GetRTTIOfType(static_cast<JPH::DMat44 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_DOUBLE3:
      return JPH::GetRTTIOfType(static_cast<JPH::Double3 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_FLOAT4:
      return JPH::GetRTTIOfType(static_cast<JPH::Float4 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_UVEC4:
      return JPH::GetRTTIOfType(static_cast<JPH::UVec4 *>(nullptr));
    case ZJOLT_PRIMITIVE_TYPE_OBJECT:
      break;
  }
  return nullptr;
}

/// An enum-typed attribute reports no member RTTI at all —
/// AddSerializableAttributeEnum's GetMemberPrimitiveType always returns
/// null — but always serializes as uint32 (SerializableAttributeEnum.h), so
/// that is what a null classifies as here.
ZJoltPrimitiveType ClassifyPrimitive(const JPH::SerializableAttribute &attr) {
  const JPH::RTTI *prim = attr.GetMemberPrimitiveType();
  if (prim == nullptr) return ZJOLT_PRIMITIVE_TYPE_UINT32;
  for (int32_t i = ZJOLT_PRIMITIVE_TYPE_UINT8; i <= ZJOLT_PRIMITIVE_TYPE_UVEC4; ++i) {
    const auto candidate = static_cast<ZJoltPrimitiveType>(i);
    if (PrimitiveRtti(candidate) == prim) return candidate;
  }
  return ZJOLT_PRIMITIVE_TYPE_OBJECT;
}

/// SerializableAttribute has no public accessor for its member offset
/// either — only GetMemberPointer, which needs a real object to offset from.
/// The same fake-non-null-base trick as BaseClassOffset applies: nothing
/// downstream of a plain pointer add dereferences it.
uint32_t OffsetOfAttribute(const JPH::SerializableAttribute &attr) {
  constexpr uintptr_t kFakeBase = 0x10000;
  const uint8_t *member =
      attr.GetMemberPointer<uint8_t>(reinterpret_cast<const void *>(kFakeBase));
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(member) - kFakeBase);
}

//===----------------------------------------------------------------------===//
// ObjectStream — whole objects
//===----------------------------------------------------------------------===//

/// Mirrors JPH::ObjectStreamIn::GetInfo, which is protected and so cannot be
/// called from here: the 8-byte ASCII header ("TOS 1.00" or "BOS 1.00")
/// every object stream starts with. Consumed from `in` so the reader
/// constructed afterward starts exactly where Jolt's own Open() would leave
/// it. The version/revision digits are read past, not parsed — this ABI and
/// Jolt itself have only ever written the one version.
bool SniffObjectStreamHeader(std::istream &in, JPH::ObjectStream::EStreamType *out_type) {
  char header[8] = {};
  in.read(header, sizeof(header));
  if (in.gcount() != static_cast<std::streamsize>(sizeof(header))) return false;
  in.clear();

  const auto is_digit = [](char c) { return c >= '0' && c <= '9'; };
  const bool ok = (header[0] == 'B' || header[0] == 'T') && header[1] == 'O' &&
                  header[2] == 'S' && (header[3] == ' ' || is_digit(header[3])) &&
                  is_digit(header[4]) && header[5] == '.' && is_digit(header[6]) &&
                  is_digit(header[7]);
  if (!ok) return false;

  *out_type = header[0] == 'T' ? JPH::ObjectStream::EStreamType::Text
                                : JPH::ObjectStream::EStreamType::Binary;
  return true;
}

#endif  // JPH_OBJECT_STREAM

#ifdef JPH_PROFILE_ENABLED

//===----------------------------------------------------------------------===//
// Profiler
//===----------------------------------------------------------------------===//

/// Jolt's own JPH_PROFILE_START does exactly this before constructing the
/// first ProfileThread — Profiler::sInstance is dereferenced unconditionally
/// by ProfileThread's constructor and destructor, so every path that can
/// reach one creates the singleton first.
JPH::Profiler *EnsureProfiler() {
  if (JPH::Profiler::sInstance == nullptr) {
    JPH::Profiler::sInstance = zjolt::New<JPH::Profiler>();
  }
  return JPH::Profiler::sInstance;
}

void WriteLE32Stream(zjolt::HostStream &out, uint32_t value) {
  uint8_t bytes[4];
  zjolt::WriteLE32(bytes, value);
  out.WriteBytes(bytes, sizeof(bytes));
}

void WriteLE64Stream(zjolt::HostStream &out, uint64_t value) {
  uint8_t bytes[8];
  zjolt::WriteLE64(bytes, value);
  out.WriteBytes(bytes, sizeof(bytes));
}

/// A length-prefixed byte run, the framing zjoltProfilerDumpStream uses for
/// `tag` and each sample's name.
void WriteBytesWithLength(zjolt::HostStream &out, const char *data, uint32_t length) {
  WriteLE32Stream(out, length);
  if (length != 0) out.WriteBytes(data, length);
}

#endif  // JPH_PROFILE_ENABLED

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// RTTI
//===----------------------------------------------------------------------===//

ZJoltResult zjoltReflectFind(const char *name, const ZJoltRtti **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(name, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::RTTI *rtti =
      JPH::Factory::sInstance != nullptr ? JPH::Factory::sInstance->Find(name) : nullptr;
  if (rtti == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "no type registered under that name");
  }
  *out = zjolt::ToC(rtti);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectFindByHash(uint32_t hash, const ZJoltRtti **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::RTTI *rtti = JPH::Factory::sInstance != nullptr
                              ? JPH::Factory::sInstance->Find(static_cast<JPH::uint32>(hash))
                              : nullptr;
  if (rtti == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "no type registered under that hash");
  }
  *out = zjolt::ToC(rtti);
  return ZJOLT_RESULT_OK;
}

void zjoltReflectGetInfo(const ZJoltRtti *rtti, ZJoltRttiInfo *out) {
  FillRttiInfo(zjolt::ToJolt(rtti), out);
}

uint32_t zjoltReflectGetBaseClassCount(const ZJoltRtti *rtti) {
  if (rtti == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(rtti)->GetBaseClassCount());
}

ZJoltResult zjoltReflectGetBaseClass(const ZJoltRtti *rtti, uint32_t index,
                                     ZJoltReflectBaseClass *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(rtti, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::RTTI *jolt_rtti = zjolt::ToJolt(rtti);
  if (index >= static_cast<uint32_t>(jolt_rtti->GetBaseClassCount())) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "index is past the last base class on this type");
  }
  const JPH::RTTI *base = jolt_rtti->GetBaseClass(static_cast<int>(index));
  FillRttiInfo(base, &out->rtti);
  out->offset = BaseClassOffset(jolt_rtti, base);
  return ZJOLT_RESULT_OK;
}

bool zjoltReflectIsKindOf(const ZJoltRtti *rtti, const ZJoltRtti *base) {
  if (rtti == nullptr || base == nullptr) return false;
  return zjolt::ToJolt(rtti)->IsKindOf(zjolt::ToJolt(base));
}

ZJoltResult zjoltReflectCreateObject(const ZJoltRtti *rtti, void **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(rtti, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::RTTI *jolt_rtti = zjolt::ToJolt(rtti);
  if (!IsSerializableObjectKind(jolt_rtti)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "zjoltReflectCreateObject only instantiates JPH::SerializableObject-derived "
        "types (Jolt's settings objects)");
  }
  if (jolt_rtti->IsAbstract()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "rtti names an abstract type and cannot be instantiated");
  }
  void *object = jolt_rtti->CreateObject();
  if (object == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  *out = object;
  return ZJOLT_RESULT_OK;
}

void zjoltReflectDestroyObject(const ZJoltRtti *rtti, void *object) {
  if (rtti == nullptr || object == nullptr) return;
  zjolt::ToJolt(rtti)->DestructObject(object);
}

const ZJoltRtti *zjoltReflectObjectGetRtti(const void *object) {
  if (object == nullptr) return nullptr;
  const auto *typed = static_cast<const JPH::SerializableObject *>(object);
  return zjolt::ToC(typed->GetRTTI());
}

ZJoltResult zjoltReflectCastTo(void *object, const ZJoltRtti *target, void **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(object, target, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  auto *typed = static_cast<JPH::SerializableObject *>(object);
  const void *casted = typed->CastTo(zjolt::ToJolt(target));
  if (casted == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "object is not the target type, and the target type is not one of its base classes");
  }
  *out = const_cast<void *>(casted);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// ObjectStream — member access
//===----------------------------------------------------------------------===//

ZJoltResult zjoltReflectGetAttributeCount(const ZJoltRtti *rtti, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(rtti, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_OBJECT_STREAM
  *out_count = static_cast<uint32_t>(zjolt::ToJolt(rtti)->GetAttributeCount());
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltReflectGetAttribute(const ZJoltRtti *rtti, uint32_t index,
                                     ZJoltReflectAttribute *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(rtti, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_OBJECT_STREAM
  const JPH::RTTI *jolt_rtti = zjolt::ToJolt(rtti);
  if (index >= static_cast<uint32_t>(jolt_rtti->GetAttributeCount())) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "index is past the last attribute on this type");
  }
  const JPH::SerializableAttribute &attr = jolt_rtti->GetAttribute(static_cast<int>(index));
  out->name = attr.GetName();
  out->primitive_type = ClassifyPrimitive(attr);
  out->offset = OffsetOfAttribute(attr);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

//===----------------------------------------------------------------------===//
// Typed member access
//
// One macro instantiation per scalar kind: both directions refuse
// (ZJOLT_RESULT_INVALID_ARGUMENT) unless `attr->primitive_type` already names
// this kind, so a stale or mismatched ZJoltReflectAttribute cannot write past
// the member it names. Read through zjolt::RawEnum, since a host-supplied
// struct's enum field can hold any integer.
//===----------------------------------------------------------------------===//

#define ZJOLT_REFLECT_SCALAR_ACCESSOR(Suffix, CType, PrimType)                                \
  ZJoltResult zjoltReflectAttributeRead##Suffix(const ZJoltReflectAttribute *attr,             \
                                                const void *object, CType *out) {               \
    ZJOLT_ENTER(out);                                                                          \
    if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;              \
    if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(PrimType)) {              \
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a " #Suffix);    \
    }                                                                                          \
    std::memcpy(out, static_cast<const uint8_t *>(object) + attr->offset, sizeof(CType));      \
    return ZJOLT_RESULT_OK;                                                                    \
  }                                                                                             \
  ZJoltResult zjoltReflectAttributeWrite##Suffix(const ZJoltReflectAttribute *attr,            \
                                                 void *object, CType value) {                   \
    ZJOLT_ENTER();                                                                             \
    if (!zjolt::Present(attr, object)) return ZJOLT_RESULT_INVALID_ARGUMENT;                   \
    if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(PrimType)) {              \
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a " #Suffix);    \
    }                                                                                           \
    std::memcpy(static_cast<uint8_t *>(object) + attr->offset, &value, sizeof(CType));         \
    return ZJOLT_RESULT_OK;                                                                    \
  }

ZJOLT_REFLECT_SCALAR_ACCESSOR(Uint8, uint8_t, ZJOLT_PRIMITIVE_TYPE_UINT8)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Uint16, uint16_t, ZJOLT_PRIMITIVE_TYPE_UINT16)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Int32, int32_t, ZJOLT_PRIMITIVE_TYPE_INT32)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Uint32, uint32_t, ZJOLT_PRIMITIVE_TYPE_UINT32)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Uint64, uint64_t, ZJOLT_PRIMITIVE_TYPE_UINT64)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Float, float, ZJOLT_PRIMITIVE_TYPE_FLOAT)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Double, double, ZJOLT_PRIMITIVE_TYPE_DOUBLE)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Bool, bool, ZJOLT_PRIMITIVE_TYPE_BOOL)

// JPH::Float3/Float4/Double3 are plain trivially-copyable structs with no
// SIMD register and no padding lane — the same byte layout as
// ZJoltVec3/ZJoltVec4/ZJoltDVec3, so the scalar macro's memcpy applies as-is.
ZJOLT_REFLECT_SCALAR_ACCESSOR(Float3, ZJoltVec3, ZJOLT_PRIMITIVE_TYPE_FLOAT3)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Float4, ZJoltVec4, ZJOLT_PRIMITIVE_TYPE_FLOAT4)
ZJOLT_REFLECT_SCALAR_ACCESSOR(Double3, ZJoltDVec3, ZJOLT_PRIMITIVE_TYPE_DOUBLE3)

#undef ZJOLT_REFLECT_SCALAR_ACCESSOR

/// JPH::Vec3 is a 16-byte SIMD register; ZJoltVec3 is three floats. The
/// scalar macro's memcpy would be wrong here, so this and the two vector/
/// matrix accessors below go through the existing zjolt::ToC/ToJolt
/// conversions instead.
ZJoltResult zjoltReflectAttributeReadVec3(const ZJoltReflectAttribute *attr, const void *object,
                                          ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_VEC3)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a Vec3");
  }
  const auto *member =
      reinterpret_cast<const JPH::Vec3 *>(static_cast<const uint8_t *>(object) + attr->offset);
  *out = zjolt::ToC(*member);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeWriteVec3(const ZJoltReflectAttribute *attr, void *object,
                                           ZJoltVec3 value) {
  ZJOLT_ENTER();
  if (!zjolt::Present(attr, object)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_VEC3)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a Vec3");
  }
  auto *member = reinterpret_cast<JPH::Vec3 *>(static_cast<uint8_t *>(object) + attr->offset);
  *member = zjolt::ToJolt(value);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeReadQuat(const ZJoltReflectAttribute *attr, const void *object,
                                          ZJoltQuat *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_QUAT)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a Quat");
  }
  const auto *member =
      reinterpret_cast<const JPH::Quat *>(static_cast<const uint8_t *>(object) + attr->offset);
  *out = zjolt::ToC(*member);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeWriteQuat(const ZJoltReflectAttribute *attr, void *object,
                                           ZJoltQuat value) {
  ZJOLT_ENTER();
  if (!zjolt::Present(attr, object)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_QUAT)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a Quat");
  }
  auto *member = reinterpret_cast<JPH::Quat *>(static_cast<uint8_t *>(object) + attr->offset);
  *member = zjolt::ToJolt(value);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeReadMat44(const ZJoltReflectAttribute *attr, const void *object,
                                           ZJoltMat44 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_MAT44)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a Mat44");
  }
  const auto *member =
      reinterpret_cast<const JPH::Mat44 *>(static_cast<const uint8_t *>(object) + attr->offset);
  *out = zjolt::ToC(*member);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeWriteMat44(const ZJoltReflectAttribute *attr, void *object,
                                            ZJoltMat44 value) {
  ZJOLT_ENTER();
  if (!zjolt::Present(attr, object)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_MAT44)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a Mat44");
  }
  auto *member = reinterpret_cast<JPH::Mat44 *>(static_cast<uint8_t *>(object) + attr->offset);
  *member = zjolt::ToJolt(value);
  return ZJOLT_RESULT_OK;
}

/// A member typed JPH::String cannot cross this ABI by memcpy — it owns a
/// heap allocation through Jolt's own STL allocator. Read/write instead go
/// through the live std::basic_string object at `attr->offset`.
ZJoltResult zjoltReflectAttributeReadString(const ZJoltReflectAttribute *attr, const void *object,
                                            const char **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_STRING)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a String");
  }
  const auto *member =
      reinterpret_cast<const JPH::String *>(static_cast<const uint8_t *>(object) + attr->offset);
  *out = member->c_str();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeWriteString(const ZJoltReflectAttribute *attr, void *object,
                                             const char *value) {
  ZJOLT_ENTER();
  if (!zjolt::Present(attr, object, value)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_STRING)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a String");
  }
  auto *member = reinterpret_cast<JPH::String *>(static_cast<uint8_t *>(object) + attr->offset);
  *member = value;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeReadVec4(const ZJoltReflectAttribute *attr, const void *object,
                                          ZJoltVec4 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_VEC4)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a Vec4");
  }
  const auto *member =
      reinterpret_cast<const JPH::Vec4 *>(static_cast<const uint8_t *>(object) + attr->offset);
  *out = ToC(*member);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeWriteVec4(const ZJoltReflectAttribute *attr, void *object,
                                           ZJoltVec4 value) {
  ZJOLT_ENTER();
  if (!zjolt::Present(attr, object)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_VEC4)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a Vec4");
  }
  auto *member = reinterpret_cast<JPH::Vec4 *>(static_cast<uint8_t *>(object) + attr->offset);
  *member = ToJolt(value);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeReadUVec4(const ZJoltReflectAttribute *attr, const void *object,
                                           ZJoltUVec4 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_UVEC4)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a UVec4");
  }
  const auto *member =
      reinterpret_cast<const JPH::UVec4 *>(static_cast<const uint8_t *>(object) + attr->offset);
  *out = ToC(*member);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeWriteUVec4(const ZJoltReflectAttribute *attr, void *object,
                                            ZJoltUVec4 value) {
  ZJOLT_ENTER();
  if (!zjolt::Present(attr, object)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_UVEC4)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a UVec4");
  }
  auto *member = reinterpret_cast<JPH::UVec4 *>(static_cast<uint8_t *>(object) + attr->offset);
  *member = ToJolt(value);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeReadDVec3(const ZJoltReflectAttribute *attr, const void *object,
                                           ZJoltDVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_DVEC3)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a DVec3");
  }
  const auto *member =
      reinterpret_cast<const JPH::DVec3 *>(static_cast<const uint8_t *>(object) + attr->offset);
  *out = ToC(*member);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeWriteDVec3(const ZJoltReflectAttribute *attr, void *object,
                                            ZJoltDVec3 value) {
  ZJOLT_ENTER();
  if (!zjolt::Present(attr, object)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_DVEC3)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a DVec3");
  }
  auto *member = reinterpret_cast<JPH::DVec3 *>(static_cast<uint8_t *>(object) + attr->offset);
  *member = ToJolt(value);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeReadDMat44(const ZJoltReflectAttribute *attr, const void *object,
                                            ZJoltDMat44 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(attr, object, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_DMAT44)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a DMat44");
  }
  const auto *member =
      reinterpret_cast<const JPH::DMat44 *>(static_cast<const uint8_t *>(object) + attr->offset);
  *out = ToC(*member);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltReflectAttributeWriteDMat44(const ZJoltReflectAttribute *attr, void *object,
                                             ZJoltDMat44 value) {
  ZJOLT_ENTER();
  if (!zjolt::Present(attr, object)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (zjolt::RawEnum(attr->primitive_type) != static_cast<int32_t>(ZJOLT_PRIMITIVE_TYPE_DMAT44)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "attribute is not a DMat44");
  }
  auto *member = reinterpret_cast<JPH::DMat44 *>(static_cast<uint8_t *>(object) + attr->offset);
  *member = ToJolt(value);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// ObjectStream — whole objects
//===----------------------------------------------------------------------===//

ZJoltResult zjoltReflectObjectSaveStream(const ZJoltRtti *rtti, const void *object,
                                         ZJoltObjectStreamFormat format,
                                         const ZJoltStream *stream) {
  ZJOLT_ENTER();
  const int32_t raw_format = zjolt::RawEnum(format);
  if (!zjolt::Present(rtti, object, stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to save through");
  }
#ifdef JPH_OBJECT_STREAM
  zjolt::ObjectStreamOStream out(*stream);
  bool ok;
  if (raw_format == ZJOLT_OBJECT_STREAM_FORMAT_TEXT) {
    JPH::ObjectStreamTextOut writer(out);
    ok = writer.Write(object, zjolt::ToJolt(rtti));
  } else {
    JPH::ObjectStreamBinaryOut writer(out);
    ok = writer.Write(object, zjolt::ToJolt(rtti));
  }
  if (out.StreamFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR, "the stream failed while writing the object");
  }
  if (!ok) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "Jolt's object stream could not write this object");
  }
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltReflectObjectRestoreStream(const ZJoltRtti *rtti, const ZJoltStream *stream,
                                            void **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(rtti, stream, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs read, is_eof and is_failed to restore through");
  }
#ifdef JPH_OBJECT_STREAM
  zjolt::ObjectStreamIStream in(*stream);
  JPH::ObjectStream::EStreamType type;
  if (!SniffObjectStreamHeader(in, &type)) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT, "not a valid Jolt object stream header");
  }

  void *object = nullptr;
  if (type == JPH::ObjectStream::EStreamType::Text) {
    JPH::ObjectStreamTextIn reader(in);
    object = reader.Read(zjolt::ToJolt(rtti));
  } else {
    JPH::ObjectStreamBinaryIn reader(in);
    object = reader.Read(zjolt::ToJolt(rtti));
  }
  if (in.StreamFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR, "the stream failed while reading the object");
  }
  if (object == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "Jolt's object stream could not read an object of this type");
  }
  *out = object;
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

//===----------------------------------------------------------------------===//
// Profiler
//===----------------------------------------------------------------------===//

ZJoltResult zjoltProfilerNextFrame(void) {
  ZJOLT_ENTER();
#ifdef JPH_PROFILE_ENABLED
  JPH::Profiler *profiler = EnsureProfiler();
  if (profiler == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  profiler->NextFrame();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

void zjoltProfilerShutdown(void) {
#ifdef JPH_PROFILE_ENABLED
  zjoltProfileThreadEnd();
  if (JPH::Profiler::sInstance != nullptr) {
    zjolt::Delete(JPH::Profiler::sInstance);
    JPH::Profiler::sInstance = nullptr;
  }
#endif
}

ZJoltResult zjoltProfilerDumpStream(const char *tag, const ZJoltStream *stream) {
  ZJOLT_ENTER();
  if (!zjolt::Present(stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to dump through");
  }
#ifdef JPH_PROFILE_ENABLED
  const JPH::ProfileThread *thread = JPH::ProfileThread::sGetInstance();
  if (thread == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the calling thread has no zjoltProfileThreadBegin in effect");
  }

  zjolt::HostStream out(*stream);
  const char *tag_text = tag != nullptr ? tag : "";
  WriteBytesWithLength(out, tag_text, static_cast<uint32_t>(std::strlen(tag_text)));
  WriteLE32Stream(out, static_cast<uint32_t>(thread->mCurrentSample));
  for (JPH::uint i = 0; i < thread->mCurrentSample; ++i) {
    const JPH::ProfileSample &sample = thread->mSamples[i];
    const char *sample_name = sample.mName != nullptr ? sample.mName : "";
    WriteBytesWithLength(out, sample_name, static_cast<uint32_t>(std::strlen(sample_name)));
    WriteLE32Stream(out, sample.mColor);
    WriteLE64Stream(out, sample.mStartCycle);
    WriteLE64Stream(out, sample.mEndCycle);
  }

  if (out.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while writing the profiler dump");
  }
  return ZJOLT_RESULT_OK;
#else
  (void)tag;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltProfilerGetProcessorTickCount(uint64_t *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_PROFILE_ENABLED
  *out = JPH::GetProcessorTickCount();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltProfilerGetProcessorTicksPerSecond(uint64_t *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_PROFILE_ENABLED
  JPH::Profiler *profiler = EnsureProfiler();
  if (profiler == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  *out = profiler->GetProcessorTicksPerSecond();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

uint8_t zjoltProfilerColorGetIntensity(ZJoltColor color) {
  return zjolt::ToJolt(color).GetIntensity();
}

ZJoltResult zjoltProfileThreadBegin(const char *name) {
  ZJOLT_ENTER();
  if (!zjolt::Present(name)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_PROFILE_ENABLED
  if (JPH::ProfileThread::sGetInstance() != nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_ALREADY_INITIALIZED,
        "a profile thread is already active on the calling thread; call "
        "zjoltProfileThreadEnd first");
  }
  JPH::Profiler *profiler = EnsureProfiler();
  if (profiler == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  JPH::ProfileThread *thread = zjolt::New<JPH::ProfileThread>(name);
  if (thread == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  JPH::ProfileThread::sSetInstance(thread);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

void zjoltProfileThreadEnd(void) {
#ifdef JPH_PROFILE_ENABLED
  JPH::ProfileThread *thread = JPH::ProfileThread::sGetInstance();
  if (thread == nullptr) return;
  JPH::ProfileThread::sSetInstance(nullptr);
  zjolt::Delete(thread);
#endif
}

ZJoltResult zjoltProfileMeasurementBegin(const char *name, uint32_t color,
                                         ZJoltProfileMeasurement **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(name, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_PROFILE_ENABLED
  JPH::ProfileMeasurement *measurement =
      zjolt::New<JPH::ProfileMeasurement>(name, static_cast<JPH::uint32>(color));
  if (measurement == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  *out = reinterpret_cast<ZJoltProfileMeasurement *>(measurement);
  return ZJOLT_RESULT_OK;
#else
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

void zjoltProfileMeasurementEnd(ZJoltProfileMeasurement *measurement) {
  if (measurement == nullptr) return;
#ifdef JPH_PROFILE_ENABLED
  zjolt::Delete(reinterpret_cast<JPH::ProfileMeasurement *>(measurement));
#endif
}

}  // extern "C"
