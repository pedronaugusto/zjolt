//===----------------------------------------------------------------------===//
// zjolt — implementation-private declarations shared by the ffi/*.cpp units.
//
// Not installed and not part of the ABI. Nothing here may appear in zjolt.h.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_INTERNAL_H_
#define ZJOLT_INTERNAL_H_

#include <Jolt/Jolt.h>

#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>

#include <cfloat>
#include <cmath>
#include <cstring>
#include <type_traits>
#include <new>
#include <utility>

#ifdef JPH_OBJECT_STREAM
#include <istream>
#include <ostream>
#include <streambuf>
#endif  // JPH_OBJECT_STREAM

#include "zjolt.h"

//===----------------------------------------------------------------------===//
// Opaque handle mapping
//
// Shape and Body are Jolt-owned, wrapped as incomplete tags a pointer
// converts through instead of a dedicated struct. zjolt's own handles are structs.
//===----------------------------------------------------------------------===//

namespace zjolt {

inline const JPH::Shape *ToJolt(const ZJoltShape *shape) {
  return reinterpret_cast<const JPH::Shape *>(shape);
}
inline const ZJoltShape *ToC(const JPH::Shape *shape) {
  return reinterpret_cast<const ZJoltShape *>(shape);
}
inline const JPH::PhysicsMaterial *ToJolt(const ZJoltPhysicsMaterial *material) {
  return reinterpret_cast<const JPH::PhysicsMaterial *>(material);
}
inline const ZJoltPhysicsMaterial *ToC(const JPH::PhysicsMaterial *material) {
  return reinterpret_cast<const ZJoltPhysicsMaterial *>(material);
}
inline const JPH::Body *ToJolt(const ZJoltBody *body) {
  return reinterpret_cast<const JPH::Body *>(body);
}
inline JPH::Body *ToJolt(ZJoltBody *body) {
  return reinterpret_cast<JPH::Body *>(body);
}
inline ZJoltBody *ToC(JPH::Body *body) {
  return reinterpret_cast<ZJoltBody *>(body);
}
/// The const-body overload zjoltPhysicsSystemTryGetBodyNoLock needs:
/// BodyLockInterfaceNoLock::TryGetBody hands back a `const Body *`, since the
/// whole point of the no-lock path is that nothing here may assume exclusive
/// access to mutate it.
inline const ZJoltBody *ToC(const JPH::Body *body) {
  return reinterpret_cast<const ZJoltBody *>(body);
}

//===----------------------------------------------------------------------===//
// Allocation
//
// Everything zjolt allocates goes through Jolt's allocator. Types needing
// more than default alignment take the aligned path, chosen at compile time.
//===----------------------------------------------------------------------===//

template <typename T>
inline void *AllocateFor() {
  if constexpr (alignof(T) > JPH_DEFAULT_ALLOCATE_ALIGNMENT)
    return JPH::AlignedAllocate(sizeof(T), alignof(T));
  else
    return JPH::Allocate(sizeof(T));
}

template <typename T>
inline void FreeFor(void *block) {
  if constexpr (alignof(T) > JPH_DEFAULT_ALLOCATE_ALIGNMENT)
    JPH::AlignedFree(block);
  else
    JPH::Free(block);
}

/// Allocation-checked replacement for a bare `new`. Jolt's own containers abort
/// on allocation failure; the objects zjolt creates report
/// ZJOLT_RESULT_OUT_OF_MEMORY instead, which is what makes that result
/// reachable rather than decorative.
template <typename T, typename... Args>
T *New(Args &&...args) {
  void *block = AllocateFor<T>();
  if (block == nullptr) return nullptr;
  return new (block) T(std::forward<Args>(args)...);
}

template <typename T>
void Delete(T *object) {
  if (object == nullptr) return;
  object->~T();
  FreeFor<T>(object);
}

/// Hands a freshly constructed reference-counted object to the caller. A
/// fresh `JPH::RefTarget` starts at refcount ZERO, not one: under-count
/// and the next Release wraps to 0xFFFFFFFF; over-count and it never dies.
///
/// Not the same arithmetic as `Finish` in zjolt_shape.cpp, which also
/// calls AddRef once but to compensate for a JPH::Ref dropping at scope exit.
template <typename T>
[[nodiscard]] inline T *Own(T *fresh) {
  static_assert(std::is_base_of_v<JPH::RefTargetVirtual, T> ||
                    std::is_convertible_v<T *, const JPH::RefTarget<T> *>,
                "Own() is for reference-counted objects; a plain one is Delete()'d");
  if (fresh == nullptr) return nullptr;
  fresh->AddRef();
  return fresh;
}

/// `Release` on a Jolt object routes `delete` through `JPH::Free`, picking
/// the overload by `alignof(T) > __STDCPP_DEFAULT_NEW_ALIGNMENT__`.
/// `AllocateFor<T>` above branches on `JPH_DEFAULT_ALLOCATE_ALIGNMENT` instead.
///
/// Same number today. If they ever disagree, an aligned block reaches the
/// unaligned free and reads a header that is not there, on a live heap.
static_assert(JPH_DEFAULT_ALLOCATE_ALIGNMENT == __STDCPP_DEFAULT_NEW_ALIGNMENT__,
              "Jolt's default allocation alignment no longer matches the one "
              "operator new/delete uses, so AllocateFor<T> and Jolt's own "
              "delete can pick different halves of the allocator");

//===----------------------------------------------------------------------===//
// Errors
//===----------------------------------------------------------------------===//

/// Records detail for zjoltLastError on the calling thread and returns
/// `result`, so a failing path is one `return SetError(...)`.
ZJoltResult SetError(ZJoltResult result, const char *message);

/// Clears this thread's error detail. Called at the head of every entry point
/// that can fail, so a stale message never accompanies a fresh failure.
void ClearError();

/// True once zjoltInit has completed and zjoltDeinit has not run.
bool IsInitialized();

/// Refuses a penetration tolerance Jolt would assert on rather than honour.
///
/// EPA asserts its tolerance is at least FLT_EPSILON; the collide-shape
/// and shape-cast settings structs are the only place a caller sets it.
/// Shared here so every entry point taking either struct owes the same check.
inline ZJoltResult CheckPenetrationTolerance(float tolerance) {
  // Written as an accept rather than a reject, which also rejects a NaN.
  if (tolerance >= FLT_EPSILON) return ZJOLT_RESULT_OK;
  return SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "penetration_tolerance is below FLT_EPSILON; Jolt asserts on that "
      "rather than honouring it, and a smaller one only buys iterations");
}

//===----------------------------------------------------------------------===//
// Live-handle accounting
//
// zjoltDeinit restores Jolt's allocator; a handle freed after that uses
// the wrong one. Counting handles zjolt owns (shapes excepted) lets deinit refuse instead.
//===----------------------------------------------------------------------===//

void HandleCreated();
void HandleDestroyed();

//===----------------------------------------------------------------------===//
// Entry-point guards
//
// Every failable entry point clears the thread's error, then out-parameters,
// then refuses if the library is not up. Required args: `Present(a, b, out)`.
//===----------------------------------------------------------------------===//

/// Zeroes one out-parameter, ignoring a NULL. A pointer becomes NULL, a struct
/// becomes all-zero, a scalar becomes 0.
template <typename T>
inline void ClearOut(T *out) {
  if (out != nullptr) *out = T{};
}

/// An out-parameter paired with the value that means "nothing was
/// written", for the cases where that value is not zero.
///
/// One such case today: a body id's empty value is ZJOLT_BODY_ID_INVALID,
/// and 0 is a valid body — zeroing on failure would name whichever body
/// was created first.
template <typename T>
struct EmptyOut {
  T *out;
  T empty;
};

template <typename T>
inline EmptyOut<T> OutIsEmptyAs(T *out, T empty) {
  return EmptyOut<T>{out, empty};
}

template <typename T>
inline void ClearOut(EmptyOut<T> out) {
  if (out.out != nullptr) *out.out = out.empty;
}

/// Steps 1 to 3 above. Pass every out-parameter the entry point writes.
template <typename... Outs>
[[nodiscard]] inline ZJoltResult Enter(Outs... outs) {
  ClearError();
  (ClearOut(outs), ...);
  if (!IsInitialized()) return ZJOLT_RESULT_NOT_INITIALIZED;
  return ZJOLT_RESULT_OK;
}

/// True when every pointer given is non-NULL, so the required arguments of an
/// entry point read as a list rather than as a chain of `|| == nullptr`.
template <typename... Ptrs>
[[nodiscard]] inline bool Present(const Ptrs *...ptrs) {
  return ((ptrs != nullptr) && ...);
}

//===----------------------------------------------------------------------===//
// An enum parameter's bytes, rather than its value.
//
// Reading an enum object holding a value no enumerator names is undefined
// behaviour (UBSan aborts). Taken by reference and copied as raw bytes to avoid that load.
//===----------------------------------------------------------------------===//

template <typename E>
int32_t RawEnum(const E &value) {
  static_assert(sizeof(E) == sizeof(int32_t),
                "an enum crossing this ABI must be int-sized: no enumerator is "
                "negative, so every compiler this ABI targets gives it int as "
                "its underlying type, and the byte copy below depends on it.");
  int32_t raw;
  std::memcpy(&raw, &value, sizeof raw);
  return raw;
}

//===----------------------------------------------------------------------===//
// Scalar and vector conversion
//
// Jolt's Vec3 is a 16-byte SIMD register with a padding lane; the ABI's
// is three floats. Every crossing goes through these, in one place.
//===----------------------------------------------------------------------===//

inline JPH::Vec3 ToJolt(const ZJoltVec3 &v) { return JPH::Vec3(v.x, v.y, v.z); }

inline ZJoltVec3 ToC(JPH::Vec3Arg v) {
  return ZJoltVec3{v.GetX(), v.GetY(), v.GetZ()};
}

inline JPH::RVec3 ToJoltR(const ZJoltRVec3 &v) {
  return JPH::RVec3(static_cast<JPH::Real>(v.x), static_cast<JPH::Real>(v.y),
                    static_cast<JPH::Real>(v.z));
}

/// Named apart from ToC(Vec3Arg) rather than overloaded: in a float build
/// JPH::RVec3 and JPH::Vec3 are the SAME type, so the two would collide on
/// return type alone. The asymmetry is upstream's, and hiding it behind an
/// overload set would make this header compile in one configuration only.
inline ZJoltRVec3 ToCR(JPH::RVec3Arg v) {
  return ZJoltRVec3{static_cast<ZJoltReal>(v.GetX()),
                    static_cast<ZJoltReal>(v.GetY()),
                    static_cast<ZJoltReal>(v.GetZ())};
}

inline JPH::Quat ToJolt(const ZJoltQuat &q) {
  return JPH::Quat(q.x, q.y, q.z, q.w);
}

inline ZJoltQuat ToC(JPH::QuatArg q) {
  return ZJoltQuat{q.GetX(), q.GetY(), q.GetZ(), q.GetW()};
}

/// A caller's rotation, made into one Jolt will accept. `Mat44::sRotation`
/// asserts its quaternion is unit length; a merely drifted rotation is
/// renormalised here instead of aborting.
///
/// One that cannot be normalised at all (zero, or carrying a NaN) becomes
/// the identity — a visible failure to rotate, not a silent NaN rotation.
inline JPH::Quat ToJoltRotation(const ZJoltQuat &q) {
  const JPH::Quat rotation = ToJolt(q);
  if (rotation.IsNormalized()) return rotation;
  const float length_sq = rotation.LengthSq();
  if (!std::isfinite(length_sq) || length_sq < 1.0e-12f)
    return JPH::Quat::sIdentity();
  return rotation.Normalized();
}

inline JPH::BodyID ToJolt(ZJoltBodyId id) { return JPH::BodyID(id); }

inline ZJoltBodyId ToC(const JPH::BodyID &id) {
  return static_cast<ZJoltBodyId>(id.GetIndexAndSequenceNumber());
}

/// A sub-shape id, built through SetValue rather than the value constructor.
///
/// `JPH::SubShapeID`'s value constructor is private and so is `cEmpty`, and
/// zjolt deliberately does not pass `-fno-access-control`. `SetValue` is the
/// public way in, and it is the only way in from here.
inline JPH::SubShapeID ToJoltSubShapeId(ZJoltSubShapeId id) {
  JPH::SubShapeID result;
  result.SetValue(static_cast<JPH::SubShapeID::Type>(id));
  return result;
}

inline ZJoltSubShapeId ToC(const JPH::SubShapeID &id) {
  return static_cast<ZJoltSubShapeId>(id.GetValue());
}

inline JPH::Color ToJolt(const ZJoltColor &c) {
  return JPH::Color(c.r, c.g, c.b, c.a);
}

inline ZJoltColor ToC(JPH::ColorArg c) {
  return ZJoltColor{c.r, c.g, c.b, c.a};
}

inline void WriteVec3(ZJoltVec3 *out, JPH::Vec3Arg v) {
  if (out != nullptr) *out = ToC(v);
}
inline void WriteRVec3(ZJoltRVec3 *out, JPH::RVec3Arg v) {
  if (out != nullptr) *out = ToCR(v);
}
inline void WriteQuat(ZJoltQuat *out, JPH::QuatArg q) {
  if (out != nullptr) *out = ToC(q);
}

/// Jolt's Mat44 is four SIMD columns; the ABI's is sixteen floats in the same
/// column-major order, so the crossing is a transpose-free copy.
inline ZJoltMat44 ToC(JPH::Mat44Arg m) {
  ZJoltMat44 out{};
  for (JPH::uint col = 0; col < 4; ++col) {
    const JPH::Vec4 column = m.GetColumn4(col);
    out.m[4 * col + 0] = column.GetX();
    out.m[4 * col + 1] = column.GetY();
    out.m[4 * col + 2] = column.GetZ();
    out.m[4 * col + 3] = column.GetW();
  }
  return out;
}

/// Named apart from ToC(Mat44Arg): in a float build JPH::RMat44 and
/// JPH::Mat44 are the SAME type, colliding on return type alone.
///
/// The translation column reads through GetTranslation, not GetColumn4(3)
/// (JPH::DMat44 asserts on that); its fourth element is written as 1
/// (every RMat44 crossing this boundary is a rigid transform).
inline ZJoltRMat44 ToCR(JPH::RMat44Arg m) {
  ZJoltRMat44 out{};
  for (JPH::uint col = 0; col < 3; ++col) {
    const JPH::Vec4 column = m.GetColumn4(col);
    out.m[4 * col + 0] = static_cast<ZJoltReal>(column.GetX());
    out.m[4 * col + 1] = static_cast<ZJoltReal>(column.GetY());
    out.m[4 * col + 2] = static_cast<ZJoltReal>(column.GetZ());
    out.m[4 * col + 3] = static_cast<ZJoltReal>(column.GetW());
  }
  const JPH::RVec3 translation = m.GetTranslation();
  out.m[12] = static_cast<ZJoltReal>(translation.GetX());
  out.m[13] = static_cast<ZJoltReal>(translation.GetY());
  out.m[14] = static_cast<ZJoltReal>(translation.GetZ());
  out.m[15] = static_cast<ZJoltReal>(1);
  return out;
}

inline void WriteMat44(ZJoltMat44 *out, JPH::Mat44Arg m) {
  if (out != nullptr) *out = ToC(m);
}
inline void WriteRMat44(ZJoltRMat44 *out, JPH::RMat44Arg m) {
  if (out != nullptr) *out = ToCR(m);
}

/// The other direction of ToC(Mat44Arg): sixteen column-major floats back
/// into Jolt's four SIMD columns. `sLoadFloat4x4` reads four consecutive
/// `Float4`s as four columns, which is exactly this layout — the same
/// reinterpretation `ffi/zjolt_softbody.cpp`'s local `ToJoltMat44` uses for
/// the identical reason, kept here too since zjolt_math.cpp is the other
/// translation unit that needs a Mat44 crossing INTO Jolt rather than out.
inline JPH::Mat44 ToJolt(const ZJoltMat44 &m) {
  return JPH::Mat44::sLoadFloat4x4(reinterpret_cast<const JPH::Float4 *>(m.m));
}

/// The other direction of ToCR(RMat44Arg). Built column by column rather
/// than through sLoadFloat4x4, which reads plain `float`s while
/// ZJoltRMat44's elements are `ZJoltReal` (`double` under -Ddouble_precision).
///
/// The translation column goes through ToJoltR(ZJoltRVec3) rather than a
/// bare cast, keeping one conversion per scalar type.
inline JPH::RMat44 ToJoltR(const ZJoltRMat44 &m) {
  const JPH::Vec4 col0(static_cast<float>(m.m[0]), static_cast<float>(m.m[1]),
                       static_cast<float>(m.m[2]), static_cast<float>(m.m[3]));
  const JPH::Vec4 col1(static_cast<float>(m.m[4]), static_cast<float>(m.m[5]),
                       static_cast<float>(m.m[6]), static_cast<float>(m.m[7]));
  const JPH::Vec4 col2(static_cast<float>(m.m[8]), static_cast<float>(m.m[9]),
                       static_cast<float>(m.m[10]),
                       static_cast<float>(m.m[11]));
  const ZJoltRVec3 translation{m.m[12], m.m[13], m.m[14]};
  return JPH::RMat44(col0, col1, col2, ToJoltR(translation));
}

//===----------------------------------------------------------------------===//
// Query filter adapters
//
// Jolt takes its query filters as abstract classes; the ABI passes plain
// function-pointer tables, forwarded here. NULL means "accept everything".
//===----------------------------------------------------------------------===//

class BroadPhaseLayerFilterAdapter final : public JPH::BroadPhaseLayerFilter {
 public:
  explicit BroadPhaseLayerFilterAdapter(const ZJoltBroadPhaseLayerFilter &f)
      : filter_(f) {}

  bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override {
    if (filter_.should_collide == nullptr) return true;
    return filter_.should_collide(
        filter_.user, static_cast<ZJoltBroadPhaseLayer>(inLayer.GetValue()));
  }

 private:
  ZJoltBroadPhaseLayerFilter filter_;
};

class ObjectLayerFilterAdapter final : public JPH::ObjectLayerFilter {
 public:
  explicit ObjectLayerFilterAdapter(const ZJoltObjectLayerFilter &f)
      : filter_(f) {}

  bool ShouldCollide(JPH::ObjectLayer inLayer) const override {
    if (filter_.should_collide == nullptr) return true;
    return filter_.should_collide(filter_.user,
                                  static_cast<ZJoltObjectLayer>(inLayer));
  }

 private:
  ZJoltObjectLayerFilter filter_;
};

class BodyFilterAdapter final : public JPH::BodyFilter {
 public:
  explicit BodyFilterAdapter(const ZJoltBodyFilter &f) : filter_(f) {}

  bool ShouldCollide(const JPH::BodyID &inBodyID) const override {
    if (filter_.should_collide == nullptr) return true;
    return filter_.should_collide(filter_.user, ToC(inBodyID));
  }

  bool ShouldCollideLocked(const JPH::Body &inBody) const override {
    if (filter_.should_collide_locked == nullptr) return true;
    return filter_.should_collide_locked(
        filter_.user, reinterpret_cast<const ZJoltBody *>(&inBody));
  }

 private:
  ZJoltBodyFilter filter_;
};

/// The narrowest of the four, and the only one Jolt asks twice: one
/// overload for queries with no shape of their own (a ray, a point), the
/// other for shape-versus-shape — both answered by the same C callback,
/// the empty sub-shape id standing in for the missing source shape.
///
/// `mBodyID2` is Jolt's: the base class carries it, set before every call.
class ShapeFilterAdapter final : public JPH::ShapeFilter {
 public:
  explicit ShapeFilterAdapter(const ZJoltShapeFilter &f) : filter_(f) {}

  bool ShouldCollide(const JPH::Shape *,
                     const JPH::SubShapeID &inSubShapeIDOfShape2) const override {
    if (filter_.should_collide == nullptr) return true;
    return filter_.should_collide(filter_.user, ToC(mBodyID2),
                                  ToC(inSubShapeIDOfShape2),
                                  ZJOLT_SUB_SHAPE_ID_EMPTY);
  }

  bool ShouldCollide(const JPH::Shape *,
                     const JPH::SubShapeID &inSubShapeIDOfShape1,
                     const JPH::Shape *,
                     const JPH::SubShapeID &inSubShapeIDOfShape2) const override {
    if (filter_.should_collide == nullptr) return true;
    return filter_.should_collide(filter_.user, ToC(mBodyID2),
                                  ToC(inSubShapeIDOfShape2),
                                  ToC(inSubShapeIDOfShape1));
  }

 private:
  ZJoltShapeFilter filter_;
};

/// The four adapters a query needs, built once from an optional
/// ZJoltQueryFilters and passed to Jolt by reference.
struct QueryFilters {
  explicit QueryFilters(const ZJoltQueryFilters *filters)
      : broad_phase(filters != nullptr ? filters->broad_phase_layer
                                       : ZJoltBroadPhaseLayerFilter{}),
        object_layer(filters != nullptr ? filters->object_layer
                                        : ZJoltObjectLayerFilter{}),
        body(filters != nullptr ? filters->body : ZJoltBodyFilter{}),
        shape(filters != nullptr ? filters->shape : ZJoltShapeFilter{}) {}

  BroadPhaseLayerFilterAdapter broad_phase;
  ObjectLayerFilterAdapter object_layer;
  BodyFilterAdapter body;
  ShapeFilterAdapter shape;
};

/// The two a broad-phase query needs.
///
/// Separate from QueryFilters rather than a subset of it because
/// BroadPhaseQuery has nowhere to put a body filter, and accepting one there
/// would be a filter that silently does nothing.
struct BroadPhaseFilters {
  explicit BroadPhaseFilters(const ZJoltBroadPhaseFilters *filters)
      : broad_phase(filters != nullptr ? filters->broad_phase_layer
                                       : ZJoltBroadPhaseLayerFilter{}),
        object_layer(filters != nullptr ? filters->object_layer
                                        : ZJoltObjectLayerFilter{}) {}

  BroadPhaseLayerFilterAdapter broad_phase;
  ObjectLayerFilterAdapter object_layer;
};

//===----------------------------------------------------------------------===//
// Byte-buffer streams, for shape serialisation
//
// Jolt's own StreamInWrapper/StreamOutWrapper wrap std::istream/ostream,
// dragging in <iostream>. A shape saves to and restores from a flat buffer here instead.
//===----------------------------------------------------------------------===//

/// Counts bytes when `buffer` is null, writes them when it is not. That is
/// what makes the size query and the write the same code path, so the two can
/// never disagree about how large a shape is.
class CountingStreamOut final : public JPH::StreamOut {
 public:
  CountingStreamOut(void *buffer, size_t capacity)
      : buffer_(static_cast<JPH::uint8 *>(buffer)), capacity_(capacity) {}

  void WriteBytes(const void *inData, size_t inNumBytes) override {
    if (buffer_ != nullptr) {
      if (written_ + inNumBytes > capacity_) {
        overflowed_ = true;
      } else {
        std::memcpy(buffer_ + written_, inData, inNumBytes);
      }
    }
    written_ += inNumBytes;
  }

  bool IsFailed() const override { return overflowed_; }

  size_t Size() const { return written_; }

 private:
  JPH::uint8 *buffer_;
  size_t capacity_;
  size_t written_ = 0;
  bool overflowed_ = false;
};

/// Reads from a borrowed buffer, reporting end-of-file rather than
/// handing back uninitialised bytes.
///
/// Jolt checks IsEOF()/IsFailed() after every read stage, so this only
/// needs to report truncation faithfully; zero-filling past the end keeps
/// an already-read count from looking like garbage before that check.
class ConstStreamIn final : public JPH::StreamIn {
 public:
  ConstStreamIn(const void *data, size_t size)
      : data_(static_cast<const JPH::uint8 *>(data)), size_(size) {}

  void ReadBytes(void *outData, size_t inNumBytes) override {
    JPH::uint8 *out = static_cast<JPH::uint8 *>(outData);
    const size_t available = read_ < size_ ? size_ - read_ : 0;
    const size_t n = inNumBytes < available ? inNumBytes : available;
    if (n != 0) std::memcpy(out, data_ + read_, n);
    if (n < inNumBytes) {
      std::memset(out + n, 0, inNumBytes - n);
      eof_ = true;
    }
    read_ += inNumBytes;
  }

  bool IsEOF() const override { return eof_; }
  bool IsFailed() const override { return false; }

  /// True when the whole buffer was consumed. A shape that restores from a
  /// prefix of the input means the input was not what it claimed to be.
  bool ConsumedAll() const { return !eof_ && read_ == size_; }

 private:
  const JPH::uint8 *data_;
  size_t size_;
  size_t read_ = 0;
  bool eof_ = false;
};

//===----------------------------------------------------------------------===//
// The save/load container
//
// Shared framing (magic tag per pair, version, config id, Jolt version,
// length, CRC-32), validated before Jolt reads a byte. Not adversarial-safe.
//===----------------------------------------------------------------------===//

/// Little-endian on every host, because the bytes are a file format rather
/// than a memory image: a blob written on one machine has to load on another.
inline void WriteLE32(uint8_t *out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

inline uint32_t ReadLE32(const uint8_t *in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

inline void WriteLE64(uint8_t *out, uint64_t value) {
  WriteLE32(out, static_cast<uint32_t>(value));
  WriteLE32(out + 4, static_cast<uint32_t>(value >> 32));
}

inline uint64_t ReadLE64(const uint8_t *in) {
  return static_cast<uint64_t>(ReadLE32(in)) |
         (static_cast<uint64_t>(ReadLE32(in + 4)) << 32);
}

/// CRC-32, the usual reflected polynomial, computed without a table. A blob is
/// written once and read once; the table is not worth the cache line.
inline uint32_t Crc32(const uint8_t *data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xedb88320u & mask);
    }
  }
  return ~crc;
}

/// The Jolt a blob was written against. A cache from a different Jolt would
/// otherwise deserialise into a plausible-looking wrong object.
inline uint32_t JoltVersionStamp() {
  return (static_cast<uint32_t>(JPH_VERSION_MAJOR) << 16) |
         (static_cast<uint32_t>(JPH_VERSION_MINOR) << 8) |
         static_cast<uint32_t>(JPH_VERSION_PATCH);
}

/// Bytes of header every container writes before its site-specific fields:
/// magic at 0, container version at 4, config id at 8, Jolt version at 12,
/// payload length at 16, payload CRC-32 at 24.
constexpr size_t kContainerCommonSize = 28;

/// What one save/load pair's container differs in, and the words it fails in.
///
/// Declare one of these next to the pair that uses it. The three messages are
/// the ones that name the thing being loaded; the checks that read the same
/// either way — version, config id, Jolt version, recorded length — word
/// themselves.
struct ContainerFormat {
  /// This pair's own four bytes. No two pairs may share them: it is the tag,
  /// not the layout, that refuses one pair's blob to another's loader.
  uint8_t magic[4];
  uint32_t version;
  /// Fixed-size fields this pair appends after the common ones, at
  /// `kContainerCommonSize`. Zero-filled on write when the pair has none.
  size_t extra_size;

  /// "too short to be a saved shape"
  const char *too_short;
  /// "not a shape saved by zjoltShapeSave"
  const char *wrong_magic;
  /// "the shape payload failed its checksum"
  const char *bad_checksum;

  constexpr size_t HeaderSize() const {
    return kContainerCommonSize + extra_size;
  }
};

/// Writes the header for a payload already sitting at `bytes + HeaderSize()`.
///
/// `extra` supplies `extra_size` bytes of the pair's own fields, or is null to
/// zero them — which is what a pair with nothing to add wants, since the space
/// is reserved either way.
inline void WriteContainerHeader(const ContainerFormat &format, uint8_t *bytes,
                                 size_t payload_size, const uint8_t *extra) {
  const size_t header_size = format.HeaderSize();
  std::memcpy(bytes, format.magic, sizeof(format.magic));
  WriteLE32(bytes + 4, format.version);
  WriteLE32(bytes + 8, static_cast<uint32_t>(ZJOLT_CONFIG_ID));
  WriteLE32(bytes + 12, JoltVersionStamp());
  WriteLE64(bytes + 16, static_cast<uint64_t>(payload_size));
  WriteLE32(bytes + 24, Crc32(bytes + header_size, payload_size));
  if (extra == nullptr) {
    std::memset(bytes + kContainerCommonSize, 0, format.extra_size);
  } else {
    std::memcpy(bytes + kContainerCommonSize, extra, format.extra_size);
  }
}

/// What a validated container was carrying. Both spans point into the caller's
/// buffer and live exactly as long as it does.
struct ContainerContents {
  const uint8_t *payload;
  size_t payload_size;
  /// `extra_size` bytes of the pair's own fields, to read with ReadLE32.
  const uint8_t *extra;
};

/// Validates the framing and reports what is inside, without letting Jolt
/// see a byte of a blob that failed. Anything wrong is
/// ZJOLT_RESULT_BAD_FORMAT with the detail recorded for zjoltLastError.
///
/// A pair's own fields are handed back rather than judged here: only the
/// pair knows what to check them against, and does so after this returns.
inline ZJoltResult ReadContainer(const ContainerFormat &format,
                                 const void *data, size_t size,
                                 ContainerContents *out) {
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  const size_t header_size = format.HeaderSize();

  if (size < header_size) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT, format.too_short);
  }
  if (std::memcmp(bytes, format.magic, sizeof(format.magic)) != 0) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT, format.wrong_magic);
  }
  if (ReadLE32(bytes + 4) != format.version) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT,
                    "saved by a different zjolt container version");
  }
  if (ReadLE32(bytes + 8) != static_cast<uint32_t>(ZJOLT_CONFIG_ID)) {
    return SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "saved by a zjolt built with different layout-affecting settings");
  }
  if (ReadLE32(bytes + 12) != JoltVersionStamp()) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT,
                    "saved against a different Jolt version");
  }

  const uint64_t payload_size = ReadLE64(bytes + 16);
  if (payload_size != static_cast<uint64_t>(size - header_size)) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT,
                    "the recorded payload length does not match the buffer");
  }
  if (ReadLE32(bytes + 24) !=
      Crc32(bytes + header_size, static_cast<size_t>(payload_size))) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT, format.bad_checksum);
  }

  out->payload = bytes + header_size;
  out->payload_size = static_cast<size_t>(payload_size);
  out->extra = bytes + kContainerCommonSize;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// The host-stream seam
//
// One adapter forwards Jolt's StreamIn/StreamOut/StateRecorder virtuals to a
// ZJoltStream, shared by *Stream entry points and the buffer form of SaveState/RestoreState.
//===----------------------------------------------------------------------===//

/// StreamIn, StreamOut and StateRecorder each declare IsFailed with the same
/// signature and are otherwise unrelated, so one override here answers all
/// three regardless of which base a particular call needs — exactly how
/// zjolt_state.cpp's own recorder classes did before this replaced them.
class HostStream final : public JPH::StateRecorder {
 public:
  explicit HostStream(const ZJoltStream &stream) : stream_(stream) {}

  void WriteBytes(const void *inData, size_t inNumBytes) override {
    if (stream_.write != nullptr) {
      stream_.write(stream_.user, inData, inNumBytes);
    } else {
      misused_ = true;
    }
  }

  void ReadBytes(void *outData, size_t inNumBytes) override {
    if (stream_.read != nullptr) {
      stream_.read(stream_.user, outData, inNumBytes);
    } else {
      std::memset(outData, 0, inNumBytes);
      misused_ = true;
    }
  }

  bool IsEOF() const override {
    return stream_.is_eof != nullptr && stream_.is_eof(stream_.user);
  }

  bool IsFailed() const override {
    return misused_ ||
           (stream_.is_failed != nullptr && stream_.is_failed(stream_.user));
  }

 private:
  ZJoltStream stream_;
  bool misused_ = false;
};

/// Whether `stream` is complete enough to write through: `write` is what
/// carries every byte, and `is_failed` is the only way a short write is ever
/// reported back, so neither has a sensible "accept everything" default the
/// way an optional filter callback does. Checked at every *Stream save entry
/// point instead of leaving a missing one to read as "never fails."
inline bool StreamCanWrite(const ZJoltStream *stream) {
  return stream->write != nullptr && stream->is_failed != nullptr;
}

/// Whether `stream` is complete enough to read through: `read` carries every
/// byte, `is_eof` is the only way running out of input is ever reported, and
/// `is_failed` the only way a read error is. Checked at every *Stream restore
/// entry point for the same reason as StreamCanWrite.
inline bool StreamCanRead(const ZJoltStream *stream) {
  return stream->read != nullptr && stream->is_eof != nullptr &&
         stream->is_failed != nullptr;
}

/// The bookkeeping a caller's plain buffer needs to stand in as a
/// ZJoltStream: a write cursor with overflow detection, a read cursor with
/// end-of-file detection, and the byte count a two-call size query reports.
/// Free functions rather than methods, because that is the shape
/// `ZJoltStream::read`/`write`/`is_eof`/`is_failed` themselves declare.
struct MemoryCursor {
  uint8_t *out = nullptr;
  size_t capacity = 0;
  size_t written = 0;
  bool overflowed = false;

  const uint8_t *in = nullptr;
  size_t size = 0;
  size_t consumed = 0;
  bool eof = false;
};

inline void MemoryCursorWrite(void *user, const void *data, size_t n) {
  MemoryCursor *cursor = static_cast<MemoryCursor *>(user);
  if (cursor->out != nullptr) {
    if (cursor->written + n > cursor->capacity) {
      cursor->overflowed = true;
    } else {
      std::memcpy(cursor->out + cursor->written, data, n);
    }
  }
  cursor->written += n;
}

inline void MemoryCursorRead(void *user, void *data, size_t n) {
  MemoryCursor *cursor = static_cast<MemoryCursor *>(user);
  const size_t available =
      cursor->consumed < cursor->size ? cursor->size - cursor->consumed : 0;
  const size_t taken = n < available ? n : available;
  if (taken != 0) {
    std::memcpy(data, cursor->in + cursor->consumed, taken);
  }
  if (taken < n) {
    std::memset(static_cast<uint8_t *>(data) + taken, 0, n - taken);
    cursor->eof = true;
  }
  cursor->consumed += n;
}

inline bool MemoryCursorIsEof(void *user) {
  return static_cast<const MemoryCursor *>(user)->eof;
}

inline bool MemoryCursorIsFailed(void *user) {
  return static_cast<const MemoryCursor *>(user)->overflowed;
}

/// True once a read cursor has consumed exactly the bytes it was given — a
/// restore that stopped short read something other than what was written.
inline bool MemoryCursorConsumedAll(const MemoryCursor &cursor) {
  return !cursor.eof && cursor.consumed == cursor.size;
}

/// A ZJoltStream view over `cursor`, for a buffer-based entry point to hand
/// `HostStream` in place of a real host stream.
inline ZJoltStream StreamOverMemory(MemoryCursor *cursor) {
  return ZJoltStream{MemoryCursorRead, MemoryCursorWrite, MemoryCursorIsEof,
                     MemoryCursorIsFailed, cursor};
}

/// The small fixed prefix a *Stream save writes ahead of Jolt's own
/// payload: magic tag (4), ZJOLT_CONFIG_ID (4), Jolt version stamp (4) —
/// guards against a mismatched build misreading the payload, not against
/// a crafted one. No length or CRC-32: a single-pass stream cannot
/// compute either without buffering the whole payload first.
constexpr size_t kStreamHeaderSize = 12;

inline void WriteStreamHeader(JPH::StreamOut &out, const uint8_t magic[4]) {
  uint8_t bytes[kStreamHeaderSize];
  std::memcpy(bytes, magic, 4);
  WriteLE32(bytes + 4, static_cast<uint32_t>(ZJOLT_CONFIG_ID));
  WriteLE32(bytes + 8, JoltVersionStamp());
  out.WriteBytes(bytes, sizeof(bytes));
}

/// Reads and checks WriteStreamHeader's twelve bytes. `wrong_magic` is the
/// message for a stream that is not this pair's kind at all.
inline ZJoltResult ReadStreamHeader(JPH::StreamIn &in, const uint8_t magic[4],
                                    const char *wrong_magic) {
  uint8_t bytes[kStreamHeaderSize];
  in.ReadBytes(bytes, sizeof(bytes));
  if (in.IsFailed()) {
    return SetError(ZJOLT_RESULT_IO_ERROR,
                    "the stream failed while reading its header");
  }
  if (in.IsEOF()) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT,
                    "the stream ended before its header did");
  }
  if (std::memcmp(bytes, magic, 4) != 0) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT, wrong_magic);
  }
  if (ReadLE32(bytes + 4) != static_cast<uint32_t>(ZJOLT_CONFIG_ID)) {
    return SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "saved by a zjolt built with different layout-affecting settings");
  }
  if (ReadLE32(bytes + 8) != JoltVersionStamp()) {
    return SetError(ZJOLT_RESULT_BAD_FORMAT,
                    "saved against a different Jolt version");
  }
  return ZJOLT_RESULT_OK;
}

#ifdef JPH_OBJECT_STREAM

//===----------------------------------------------------------------------===//
// Bridging a ZJoltStream to std::istream/std::ostream
//
// ObjectStreamIn/Out template over istream&/ostream&, predating Jolt's own
// StreamIn/StreamOut. Single-byte reads only: a buffered underflow could not tell truncation from a late arrival.
//===----------------------------------------------------------------------===//

class ZJoltStreamBuf : public std::streambuf {
 public:
  explicit ZJoltStreamBuf(const ZJoltStream &stream) : stream_(stream) {}

  bool Failed() const { return failed_; }

 protected:
  int_type underflow() override {
    if (stream_.read == nullptr) {
      failed_ = true;
      return traits_type::eof();
    }
    stream_.read(stream_.user, &get_byte_, 1);
    const bool failed =
        stream_.is_failed != nullptr && stream_.is_failed(stream_.user);
    const bool eof = stream_.is_eof != nullptr && stream_.is_eof(stream_.user);
    if (failed) failed_ = true;
    if (failed || eof) return traits_type::eof();
    setg(&get_byte_, &get_byte_, &get_byte_ + 1);
    return traits_type::to_int_type(get_byte_);
  }

  std::streamsize xsputn(const char *s, std::streamsize n) override {
    if (stream_.write == nullptr) {
      failed_ = true;
      return 0;
    }
    stream_.write(stream_.user, s, static_cast<size_t>(n));
    if (stream_.is_failed != nullptr && stream_.is_failed(stream_.user)) {
      failed_ = true;
      return 0;
    }
    return n;
  }

  int_type overflow(int_type ch) override {
    if (traits_type::eq_int_type(ch, traits_type::eof()))
      return traits_type::not_eof(ch);
    const char c = traits_type::to_char_type(ch);
    return xsputn(&c, 1) == 1 ? ch : traits_type::eof();
  }

 private:
  ZJoltStream stream_;
  char get_byte_ = 0;
  bool failed_ = false;
};

/// istream over a ZJoltStream. Private inheritance of the buffer, ahead of
/// the public stream base, is what makes the buffer exist before the stream
/// constructor runs and needs it — the usual idiom for pairing a standard
/// stream with a custom streambuf, the same one std::stringstream itself
/// uses internally.
class ObjectStreamIStream final : private ZJoltStreamBuf, public std::istream {
 public:
  explicit ObjectStreamIStream(const ZJoltStream &stream)
      : ZJoltStreamBuf(stream), std::istream(this) {}

  bool StreamFailed() const { return Failed(); }
};

class ObjectStreamOStream final : private ZJoltStreamBuf, public std::ostream {
 public:
  explicit ObjectStreamOStream(const ZJoltStream &stream)
      : ZJoltStreamBuf(stream), std::ostream(this) {}

  bool StreamFailed() const { return Failed(); }
};

#endif  // JPH_OBJECT_STREAM

}  // namespace zjolt

/// Opens a result-returning entry point. Returns FROM THE CALLER when the
/// library is not up, so it must be a macro rather than a plain call.
///
/// Its arguments are the entry point's out-parameters, cleared before the
/// check that can fail. Wrap one in zjolt::OutIsEmptyAs when its empty value
/// is not zero.
#define ZJOLT_ENTER(...)                                          \
  do {                                                            \
    const ZJoltResult zjolt_entered_ = zjolt::Enter(__VA_ARGS__); \
    if (zjolt_entered_ != ZJOLT_RESULT_OK) return zjolt_entered_; \
  } while (false)

//===----------------------------------------------------------------------===//
// Handle types zjolt owns (global namespace — they must match the C tag names)
//===----------------------------------------------------------------------===//

/// A collision group filter. HOLDS a GroupFilterTable rather than being
/// one: JPH::GroupFilterTable is `final` and keeps its sub-group count
/// private, so returning errors instead of Jolt's out-of-bounds indexing
/// requires tracking that count here. CanCollide forwards exactly: the
/// table compares filter POINTERS, and both bodies point at this wrapper.
struct ZJoltGroupFilter final : public JPH::GroupFilter {
  explicit ZJoltGroupFilter(JPH::uint sub_groups)
      : table(sub_groups), num_sub_groups(sub_groups) {}

  bool CanCollide(const JPH::CollisionGroup &group1,
                  const JPH::CollisionGroup &group2) const override {
    return table.CanCollide(group1, group2);
  }

  JPH::GroupFilterTable table;
  JPH::uint num_sub_groups;
};

namespace zjolt {

/// A body's collision group, made into Jolt's.
///
/// A NULL `group` — and a NULL filter inside one — is the default-constructed
/// CollisionGroup, which already means "no group, no filter". One place rather
/// than one per translation unit: a body desc, a soft-body desc, a ragdoll
/// part and zjoltBodySetCollisionGroup all hand Jolt the same three fields.
inline JPH::CollisionGroup ToJolt(const ZJoltCollisionGroup *group) {
  JPH::CollisionGroup out;
  if (group == nullptr) return out;
  out.SetGroupFilter(group->filter);
  out.SetGroupID(group->group_id);
  out.SetSubGroupID(group->sub_group_id);
  return out;
}

}  // namespace zjolt

/// Which concrete JPH::JobSystem a ZJoltJobSystem wraps.
///
/// Only exists so that zjoltJobSystemSetNumThreads can refuse a job system
/// that has no thread count of its own to resize, rather than reinterpreting
/// a JobSystemSingleThreaded or a host adapter as a JobSystemThreadPool and
/// corrupting whatever actually sits at that address.
enum class ZJoltJobSystemKind : uint8_t {
  ThreadPool,
  SingleThreaded,
  Host,
};

/// Wraps whichever JPH::JobSystem implementation the host asked for.
///
/// The indirection is the seam: a host scheduler is the third subclass,
/// reached through zjoltJobSystemCreateHost, and neither this struct nor any
/// call that takes a ZJoltJobSystem* changed shape to add it.
struct ZJoltJobSystem {
  JPH::JobSystem *impl;
  void (*destroy)(JPH::JobSystem *impl);
  ZJoltJobSystemKind kind;
};

/// Forwards Jolt's three broad-phase questions to the host's C callbacks.
class ZJoltBroadPhaseLayerInterfaceAdapter final
    : public JPH::BroadPhaseLayerInterface {
 public:
  explicit ZJoltBroadPhaseLayerInterfaceAdapter(
      const ZJoltBroadPhaseLayerInterface &iface)
      : iface_(iface) {}

  JPH::uint GetNumBroadPhaseLayers() const override {
    return iface_.num_broad_phase_layers(iface_.user);
  }

  JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
    return JPH::BroadPhaseLayer(iface_.broad_phase_layer_for_object_layer(
        iface_.user, static_cast<ZJoltObjectLayer>(inLayer)));
  }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
    if (iface_.broad_phase_layer_name == nullptr) return "layer";
    const char *name = iface_.broad_phase_layer_name(
        iface_.user, static_cast<ZJoltBroadPhaseLayer>(inLayer.GetValue()));
    return name != nullptr ? name : "layer";
  }
#endif

  /// What zjoltPhysicsSystemGetBroadPhaseLayerInterface hands back.
  const ZJoltBroadPhaseLayerInterface &Raw() const { return iface_; }

 private:
  ZJoltBroadPhaseLayerInterface iface_;
};

class ZJoltObjectVsBroadPhaseLayerFilterAdapter final
    : public JPH::ObjectVsBroadPhaseLayerFilter {
 public:
  explicit ZJoltObjectVsBroadPhaseLayerFilterAdapter(
      const ZJoltObjectVsBroadPhaseLayerFilter &filter)
      : filter_(filter) {}

  bool ShouldCollide(JPH::ObjectLayer inLayer1,
                     JPH::BroadPhaseLayer inLayer2) const override {
    if (filter_.should_collide == nullptr) return true;
    return filter_.should_collide(
        filter_.user, static_cast<ZJoltObjectLayer>(inLayer1),
        static_cast<ZJoltBroadPhaseLayer>(inLayer2.GetValue()));
  }

  /// What zjoltPhysicsSystemGetObjectVsBroadPhaseLayerFilter hands back.
  const ZJoltObjectVsBroadPhaseLayerFilter &Raw() const { return filter_; }

 private:
  ZJoltObjectVsBroadPhaseLayerFilter filter_;
};

class ZJoltObjectLayerPairFilterAdapter final
    : public JPH::ObjectLayerPairFilter {
 public:
  explicit ZJoltObjectLayerPairFilterAdapter(
      const ZJoltObjectLayerPairFilter &filter)
      : filter_(filter) {}

  bool ShouldCollide(JPH::ObjectLayer inLayer1,
                     JPH::ObjectLayer inLayer2) const override {
    if (filter_.should_collide == nullptr) return true;
    return filter_.should_collide(filter_.user,
                                  static_cast<ZJoltObjectLayer>(inLayer1),
                                  static_cast<ZJoltObjectLayer>(inLayer2));
  }

  /// What zjoltPhysicsSystemGetObjectLayerPairFilter hands back.
  const ZJoltObjectLayerPairFilter &Raw() const { return filter_; }

 private:
  ZJoltObjectLayerPairFilter filter_;
};

/// Projects Jolt's contact callbacks into plain data and forwards them.
class ZJoltContactListenerAdapter final : public JPH::ContactListener {
 public:
  explicit ZJoltContactListenerAdapter(const ZJoltContactListener &listener)
      : listener_(listener) {}

  JPH::ValidateResult OnContactValidate(
      const JPH::Body &inBody1, const JPH::Body &inBody2,
      JPH::RVec3Arg inBaseOffset,
      const JPH::CollideShapeResult &inCollisionResult) override;

  void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2,
                      const JPH::ContactManifold &inManifold,
                      JPH::ContactSettings &ioSettings) override;

  void OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2,
                          const JPH::ContactManifold &inManifold,
                          JPH::ContactSettings &ioSettings) override;

  void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override;

  /// What zjoltPhysicsSystemGetContactListener hands back.
  const ZJoltContactListener &Raw() const { return listener_; }

 private:
  ZJoltContactListener listener_;
};

class ZJoltBodyActivationListenerAdapter final
    : public JPH::BodyActivationListener {
 public:
  explicit ZJoltBodyActivationListenerAdapter(
      const ZJoltBodyActivationListener &listener)
      : listener_(listener) {}

  void OnBodyActivated(const JPH::BodyID &inBodyID,
                       JPH::uint64 inBodyUserData) override {
    if (listener_.on_body_activated != nullptr)
      listener_.on_body_activated(listener_.user, zjolt::ToC(inBodyID),
                                  inBodyUserData);
  }

  void OnBodyDeactivated(const JPH::BodyID &inBodyID,
                         JPH::uint64 inBodyUserData) override {
    if (listener_.on_body_deactivated != nullptr)
      listener_.on_body_deactivated(listener_.user, zjolt::ToC(inBodyID),
                                    inBodyUserData);
  }

  /// What zjoltPhysicsSystemGetBodyActivationListener hands back.
  const ZJoltBodyActivationListener &Raw() const { return listener_; }

 private:
  ZJoltBodyActivationListener listener_;
};

/// Projects Jolt's soft-body contact callbacks into plain data.
///
/// Separate from ZJoltContactListenerAdapter: Jolt keeps the two listeners
/// in separate slots and never calls one for the other's collisions.
/// Methods are in zjolt_softbody.cpp, with the manifold projection they share.
class ZJoltSoftBodyContactListenerAdapter final
    : public JPH::SoftBodyContactListener {
 public:
  explicit ZJoltSoftBodyContactListenerAdapter(
      const ZJoltSoftBodyContactListener &listener)
      : listener_(listener) {}

  JPH::SoftBodyValidateResult OnSoftBodyContactValidate(
      const JPH::Body &inSoftBody, const JPH::Body &inOtherBody,
      JPH::SoftBodyContactSettings &ioSettings) override;

  void OnSoftBodyContactAdded(const JPH::Body &inSoftBody,
                              const JPH::SoftBodyManifold &inManifold) override;

  /// What zjoltPhysicsSystemGetSoftBodyContactListener hands back.
  const ZJoltSoftBodyContactListener &Raw() const { return listener_; }

 private:
  ZJoltSoftBodyContactListener listener_;
};

/// Forwards Jolt's per-shape-pair simulation filter to the host's callback.
///
/// Declared here rather than built inline in zjolt_system.cpp for the same
/// reason as ZJoltContactListenerAdapter: PhysicsSystem::SetSimShapeFilter
/// stores a raw pointer, not a copy, and it must outlive every call
/// PhysicsSystem::Update makes into it during the step.
class ZJoltSimShapeFilterAdapter final : public JPH::SimShapeFilter {
 public:
  explicit ZJoltSimShapeFilterAdapter(const ZJoltSimShapeFilter &filter)
      : filter_(filter) {}

  bool ShouldCollide(const JPH::Body &inBody1, const JPH::Shape *inShape1,
                     const JPH::SubShapeID &inSubShapeIDOfShape1,
                     const JPH::Body &inBody2, const JPH::Shape *inShape2,
                     const JPH::SubShapeID &inSubShapeIDOfShape2) const override {
    if (filter_.should_collide == nullptr) return true;
    return filter_.should_collide(
        filter_.user, zjolt::ToC(inBody1.GetID()), zjolt::ToC(inShape1),
        zjolt::ToC(inSubShapeIDOfShape1), zjolt::ToC(inBody2.GetID()),
        zjolt::ToC(inShape2), zjolt::ToC(inSubShapeIDOfShape2));
  }

  /// What zjoltPhysicsSystemGetSimShapeFilter hands back.
  const ZJoltSimShapeFilter &Raw() const { return filter_; }

 private:
  ZJoltSimShapeFilter filter_;
};

/// A body detached from its id by zjoltBodyUnassignId(s) -- see the note
/// in zjolt_body.h. Defined here since zjolt_batch.cpp constructs one too.
///
/// `body` is a fully independent Jolt object here: freeing it does not
/// touch `owner`'s body table. `owner` is kept only to check a handle is
/// handed back to the system it came from.
struct ZJoltUnassignedBody {
  JPH::Body *body = nullptr;
  ZJoltPhysicsSystem *owner = nullptr;
};

/// Defined below; a system holds the characters created against it.
struct ZJoltCharacter;
struct ZJoltRigidCharacter;

/// One world.
///
/// The layer adapters live here, by value, because PhysicsSystem::Init stores
/// references to them for the lifetime of the system. Owning them alongside
/// the system is what makes the C API's "the structs are copied, you need not
/// keep them alive" promise true.
struct ZJoltPhysicsSystem {
  JPH::PhysicsSystem system;

  /// The step's scratch allocator. Always the base class: which concrete type
  /// actually sits here depends on ZJoltPhysicsSystemDesc::temp_allocator_kind,
  /// so destroying it correctly, or reading its stats back, goes through
  /// `destroy_temp_allocator` / `temp_allocator_kind` rather than a fixed
  /// static_cast. Defined in zjolt_system.cpp.
  JPH::TempAllocator *temp_allocator = nullptr;
  void (*destroy_temp_allocator)(JPH::TempAllocator *) = nullptr;
  ZJoltTempAllocatorKind temp_allocator_kind = ZJOLT_TEMP_ALLOCATOR_KIND_MALLOC_FALLBACK;

  ZJoltBroadPhaseLayerInterfaceAdapter broad_phase_layers;
  ZJoltObjectVsBroadPhaseLayerFilterAdapter object_vs_broad_phase_filter;
  ZJoltObjectLayerPairFilterAdapter object_layer_pair_filter;

  /// Null until a listener is installed; owned when not.
  ZJoltContactListenerAdapter *contact_listener = nullptr;
  ZJoltBodyActivationListenerAdapter *activation_listener = nullptr;
  ZJoltSoftBodyContactListenerAdapter *soft_body_contact_listener = nullptr;

  /// Null until zjoltPhysicsSystemSetSimShapeFilter installs one; owned when
  /// not, for the same reason contact_listener is: PhysicsSystem keeps only a
  /// raw pointer, so the pointee has to outlive it.
  ZJoltSimShapeFilterAdapter *sim_shape_filter = nullptr;

  /// Step listeners still attached, owned. Jolt keeps its own list and asserts
  /// on a double add or a stranger's remove, so this is the list that answers
  /// "is this handle mine" before Jolt is asked. Defined in zjolt_system.cpp.
  JPH::Array<ZJoltStepListener *> step_listeners;

  /// Batches prepared but neither finalized nor aborted, owned. Each one holds
  /// bodies that Jolt has marked as belonging to a broad-phase layer without
  /// having put them in the tree, so destroying the system has to unwind them.
  /// Defined in zjolt_batch.cpp.
  JPH::Array<ZJoltBodyAddBatch *> pending_batches;

  /// Every character created against this system and not yet destroyed,
  /// borrowed. Neither kind is saved by JPH::PhysicsSystem::SaveState, so
  /// zjoltPhysicsSystemSaveState reads these lists to refuse a save it was
  /// not handed them for. Maintained in zjolt_character.cpp.
  JPH::Array<ZJoltCharacter *> characters;
  JPH::Array<ZJoltRigidCharacter *> rigid_characters;

  /// Whether Update has ever run on this system.
  ///
  /// `WereBodiesInContact` reads the contact cache's back buffer, which
  /// `ManifoldCache::Find` asserts is finalised — not true until the first
  /// step ends. Asking before then answers "no", not an abort.
  bool has_stepped = false;

  /// Index into the combine-callback slot table in zjolt_system.cpp, or -1
  /// when this system has never installed one. Jolt's combine hook is a bare
  /// function pointer with no user parameter, and the slot is what carries the
  /// host's.
  int combine_slot = -1;

  /// Jolt's own combine functions, captured at creation.
  ///
  /// Clearing a combine callback must put something back:
  /// `SetCombineFriction(nullptr)` is not it — the solver calls the pointer
  /// unconditionally. Spelled out rather than named via
  /// `ContactConstraintManager::CombineFunction`, avoiding that include.
  using CombineFunction = float (*)(const JPH::Body &, const JPH::SubShapeID &,
                                    const JPH::Body &, const JPH::SubShapeID &);
  CombineFunction default_combine_friction = nullptr;
  CombineFunction default_combine_restitution = nullptr;

  ZJoltPhysicsSystem(const ZJoltPhysicsSystemDesc &desc)
      : broad_phase_layers(desc.broad_phase_layers),
        object_vs_broad_phase_filter(desc.object_vs_broad_phase_filter),
        object_layer_pair_filter(desc.object_layer_pair_filter) {}
};

namespace zjolt {

/// Aborts and frees every batch still staged on `system`.
///
/// Defined in zjolt_batch.cpp, called from zjoltPhysicsSystemDestroy, and
/// declared here because those are different translation units. A batch that
/// outlived its system would leave bodies marked as being in a broad-phase
/// layer they were never inserted into.
void AbortPendingBatches(ZJoltPhysicsSystem *system);

}  // namespace zjolt

namespace zjolt {

/// The plane below which a contact may count as ground, for a character of
/// the given shape: one inner radius above the shape's lowest point (for
/// a capsule or sphere, the centre of its bottom cap). Shared by both
/// character kinds, so swapping one for the other keeps "on the ground"
/// meaning the same thing.
inline JPH::Plane SupportingVolumeFor(const JPH::Shape *shape,
                                      JPH::Vec3Arg up_hint) {
  const JPH::Vec3 up = up_hint.NormalizedOr(JPH::Vec3::sAxisY());
  const JPH::AABox local_bounds = shape->GetLocalBounds();
  const float lowest = up.Dot(local_bounds.mMin);
  return JPH::Plane(up, -(lowest + shape->GetInnerRadius()));
}

}  // namespace zjolt

/// A character controller, plus the system it queries against. `owner` is
/// null once that system has been destroyed.
struct ZJoltCharacter {
  JPH::Ref<JPH::CharacterVirtual> impl;
  ZJoltPhysicsSystem *owner;
};

/// Jolt's rigid-body-backed character, plus the system its body lives in.
/// Beside ZJoltCharacter rather than inside zjolt_character.cpp because
/// zjolt_state.cpp saves and restores both kinds.
struct ZJoltRigidCharacter {
  JPH::Ref<JPH::Character> impl;
  ZJoltPhysicsSystem *owner;
};

namespace zjolt {

/// Clears `system` out of every character still registered against it, and
/// empties both registries. Called by zjoltPhysicsSystemDestroy; defined in
/// zjolt_character.cpp. A character that outlives its system is already a
/// misuse, and this is what keeps it from becoming a write through a freed
/// pointer when the character is destroyed afterwards.
void ForgetCharacters(ZJoltPhysicsSystem *system);

}  // namespace zjolt

#endif  // ZJOLT_INTERNAL_H_
