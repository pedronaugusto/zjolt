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
#include <Jolt/Physics/PhysicsSystem.h>

#include <cfloat>
#include <cmath>
#include <cstring>
#include <type_traits>
#include <new>
#include <utility>

#include "zjolt.h"

//===----------------------------------------------------------------------===//
// Opaque handle mapping
//
// Two of the handles in zjolt.h name objects Jolt owns and constructs itself
// (Shape is reference counted and comes back out of a deserialise; Body lives
// in the body manager), so they cannot be wrapped in a struct of ours without
// an extra allocation and a second identity. They are instead incomplete tags
// that a pointer to the Jolt type is converted through.
//
// That conversion is sound rather than merely conventional: converting an
// object pointer to a pointer to an unrelated type and back again yields the
// original value, and these tags are NEVER completed and NEVER dereferenced —
// every dereference happens after converting back to the Jolt type. This is
// the same guarantee any opaque `typedef struct Foo Foo;` C API relies on.
//
// The handles zjolt allocates itself (physics system, job system, character,
// group filter) are real structs, defined at the bottom of this header.
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

//===----------------------------------------------------------------------===//
// Allocation
//
// Everything zjolt allocates goes through Jolt's allocator, so a host that
// installed one sees zjolt's own bookkeeping in its budget too, not just
// Jolt's. Types with SIMD members need more than the default alignment, which
// Jolt's plain Allocate does not promise, so the aligned path is chosen at
// compile time per type.
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

/// Allocation-checked replacement for a bare `new`. Jolt's own containers
/// abort on allocation failure; the objects zjolt creates report
/// ZJOLT_RESULT_OUT_OF_MEMORY instead, which is what makes that result reachable
/// rather than decorative.
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

/// Hands a freshly constructed reference-counted object to the caller.
///
/// This exists because the arithmetic is not what anyone expects. A fresh
/// `JPH::RefTarget` starts at refcount **zero** (`Core/Reference.h:20,86`), not
/// one, so the object a constructor just built is owned by nobody and the
/// caller's reference is the first. Under-count and the next `Release` wraps
/// the counter to `0xFFFFFFFF` and the object outlives its own frees;
/// over-count and it never dies at all.
///
/// It is also not the same arithmetic as `Finish` in `zjolt_shape.cpp`, which
/// also calls `AddRef` exactly once — there the count is already 1 and the
/// call compensates for a `JPH::Ref` dropping at scope exit. Same call, two
/// different reasons, and mixing them up is silent. Use this and the question
/// does not come up.
template <typename T>
[[nodiscard]] inline T *Own(T *fresh) {
  static_assert(std::is_base_of_v<JPH::RefTargetVirtual, T> ||
                    std::is_convertible_v<T *, const JPH::RefTarget<T> *>,
                "Own() is for reference-counted objects; a plain one is Delete()'d");
  if (fresh == nullptr) return nullptr;
  fresh->AddRef();
  return fresh;
}

/// `Release` on a Jolt object routes `delete` through `JPH::Free` via
/// `JPH_OVERRIDE_NEW_DELETE`, and which overload runs depends on
/// `alignof(T) > __STDCPP_DEFAULT_NEW_ALIGNMENT__`. `AllocateFor<T>` above
/// branches on `alignof(T) > JPH_DEFAULT_ALLOCATE_ALIGNMENT` instead.
///
/// They are the same number today, and Jolt explicitly invites overriding its
/// half. If the two ever disagree, a block from the aligned path reaches the
/// unaligned free — which reads a header that is not there, on a heap that is
/// still live. Nothing about that failure points back here, so it is pinned
/// where the reason is written rather than left to be rediscovered.
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
/// EPA asserts its tolerance is at least FLT_EPSILON
/// (EPAPenetrationDepth.h:154), and the collide-shape and shape-cast settings
/// structs are the only place in this ABI where a caller can set it at all.
/// Shared rather than per-file because every entry point taking either struct
/// owes the same refusal, and a second copy is a copy that can drift.
inline ZJoltResult CheckPenetrationTolerance(float tolerance) {
  // Written as an accept rather than a reject so that a NaN is refused too.
  if (tolerance >= FLT_EPSILON) return ZJOLT_RESULT_OK;
  return SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "penetration_tolerance is below FLT_EPSILON; Jolt asserts on that "
      "rather than honouring it, and a smaller one only buys iterations");
}

//===----------------------------------------------------------------------===//
// Live-handle accounting
//
// zjoltDeinit restores Jolt's own allocator. A handle destroyed after that
// point is freed through an allocator it was not allocated from, which is heap
// corruption with no symptom at the site of the mistake. Counting the handles
// zjolt owns is what lets deinit refuse instead.
//
// Shapes are not counted: they are reference counted by Jolt and their count
// changes inside calls zjolt does not mediate, so a number kept here would be
// wrong rather than merely absent.
//===----------------------------------------------------------------------===//

void HandleCreated();
void HandleDestroyed();

//===----------------------------------------------------------------------===//
// Entry-point guards
//
// Every entry point that can fail opens the same way, and the order is not
// arbitrary:
//
//   1. clear this thread's error detail, so a stale message never accompanies
//      a fresh failure;
//   2. clear the out-parameters, BEFORE anything can fail, so a caller that
//      ignores the result never reads uninitialised storage;
//   3. refuse the call if the library is not up.
//
// Which arguments are REQUIRED is the one part that is not uniform, so that
// check stays at the entry point, written as a list — `Present(a, b, out)`.
//
// This is one thing rather than a habit repeated per translation unit because
// the surface is growing to several hundred entry points, and a preamble that
// can be half-written is a preamble that eventually will be.
//===----------------------------------------------------------------------===//

/// Zeroes one out-parameter, ignoring a NULL. A pointer becomes NULL, a struct
/// becomes all-zero, a scalar becomes 0.
template <typename T>
inline void ClearOut(T *out) {
  if (out != nullptr) *out = T{};
}

/// An out-parameter paired with the value that means "nothing was written",
/// for the cases where that value is not zero.
///
/// There is exactly one such case today and it is a trap worth naming: a body
/// id's empty value is ZJOLT_BODY_ID_INVALID, and 0 is a perfectly good body.
/// Zeroing one on failure would hand back a reference to whichever body was
/// created first.
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
// Scalar and vector conversion
//
// Jolt's Vec3 is a 16-byte SIMD register with a padding lane; the ABI's is
// three floats. Every crossing goes through these, in one place, so no
// translation unit invents its own.
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

/// A caller's rotation, made into one Jolt will accept.
///
/// `Mat44::sRotation` asserts that its quaternion is unit length, and a body's
/// rotation reaches it on every step and every query placement. So a rotation
/// that has merely drifted — the usual result of integrating one over a few
/// thousand frames — would abort the process in a build with asserts on. An
/// ABI should not do that to a caller over their arithmetic, so it is
/// renormalised here instead.
///
/// A rotation that cannot be normalised at all — all zeroes, or carrying a NaN
/// — becomes the identity. There is no other answer, and a visible failure to
/// rotate is easier to find than a silent rotation by NaN.
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

/// Named apart from ToC(Mat44Arg) for the same reason ToCR(RVec3Arg) is named
/// apart from ToC(Vec3Arg): in a float build JPH::RMat44 and JPH::Mat44 are
/// the SAME type, so the two would collide on return type alone.
///
/// The translation column is read through GetTranslation rather than
/// GetColumn4(3), because JPH::DMat44 keeps it as a DVec3 and asserts on
/// GetColumn4(3) (DMat44.h:115). Its fourth element is therefore written as 1
/// rather than read back — which is what a rigid transform carries, and every
/// RMat44 that crosses this boundary is one.
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

/// The other direction of ToCR(RMat44Arg). Built column by column rather than
/// through sLoadFloat4x4: that loader reads plain `float`s, and ZJoltRMat44's
/// elements are `ZJoltReal` — `double` under -Ddouble_precision, when they do
/// not fit a Float4 at all. The upper 3x3 is exact float precision either
/// way (see zjolt_core.h's note on ZJoltRMat44), so only the cast is doing
/// anything in a float build; the translation column goes through
/// ToJoltR(ZJoltRVec3) rather than a bare cast so it keeps this file's usual
/// one conversion per scalar type.
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
// Jolt takes its query filters as abstract classes. The ABI passes plain
// function-pointer tables, so these forward. A NULL function pointer means
// "accept everything", which lets a host fill in only the filter it cares
// about — and lets zjoltCastRay* take a NULL ZJoltQueryFilters entirely.
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

 private:
  ZJoltBodyFilter filter_;
};

/// The narrowest of the four, and the only one Jolt asks twice.
///
/// One overload is for queries with no shape of their own (a ray, a point) and
/// the other for shape-versus-shape. Both are answered by the same C callback,
/// with the empty sub-shape id standing in for the source shape the first
/// overload does not have — one question is easier to answer correctly than
/// two, and no caller has yet wanted to distinguish them.
///
/// `mBodyID2` is Jolt's, not ours: the base class carries it and the narrow
/// phase sets it before every call, which is the only reason this filter can
/// say WHICH body it is being asked about.
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
// Jolt ships StreamInWrapper/StreamOutWrapper over std::istream/std::ostream,
// which would drag <iostream> in and force a copy through a stringstream. A
// shape is saved to and restored from a flat buffer here, so these are the
// two implementations that need to exist.
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

/// Reads from a borrowed buffer, reporting end-of-file rather than handing
/// back uninitialised bytes.
///
/// Jolt does check its own reads — Shape::sRestoreFromBinaryState tests
/// IsEOF() and IsFailed() after every stage — so this stream's job is only to
/// report truncation faithfully. Zero-filling past the end keeps any count
/// Jolt has already read from being interpreted as garbage in the window
/// before the check.
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
// Every save/load pair in this library wraps Jolt's payload in the same
// framing — a magic tag, a container version, this library's config id, the
// Jolt version, the payload length and a CRC-32, all validated before Jolt
// reads a byte. Shapes, scenes, whole-system state and per-body state each
// used to carry their own copy of it, which meant four places to fix a
// framing bug and nothing that made them agree. They share this one instead.
//
// What still differs per site is deliberate and is what `ContainerFormat`
// carries: the magic tag is a DIFFERENT four bytes for each pair, so a shape
// blob handed to a scene restore is refused on the tag rather than parsed
// into something plausible and wrong; and a site may append fixed-size fields
// of its own after the common ones (the state pair records which parts were
// saved and which bodies they came from, the per-body pair the body's motion
// type).
//
// What this does NOT claim: it is not a defence against a deliberately
// crafted payload carrying a matching checksum. Treat a saved blob as
// something your own cook wrote, not as untrusted input.
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

/// Validates the framing and reports what is inside, without letting Jolt see
/// a byte of a blob that failed. Anything wrong is ZJOLT_RESULT_BAD_FORMAT
/// with the detail already recorded for zjoltLastError.
///
/// A pair's own fields are handed back rather than judged here: only the pair
/// knows what its motion type or state mask has to agree with, and it checks
/// them after this returns — still before Jolt reads anything.
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

}  // namespace zjolt

/// Opens a result-returning entry point. Returns FROM THE CALLER when the
/// library is not up — which is the reason it is a macro and not just a call.
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

/// A collision group filter.
///
/// A thin subclass of JPH::GroupFilter that HOLDS a GroupFilterTable rather
/// than being one, for a reason worth writing down: JPH::GroupFilterTable is
/// `final`, and it keeps its sub-group count private with no accessor. Every
/// one of its mutators indexes a bit table through GetBit, which asserts
/// `sub_group1 != sub_group2` and `sub_group2 < mNumSubGroups`
/// (GroupFilterTable.h:46,52) and, in a build without asserts, indexes out of
/// bounds instead. Turning those into returned errors means knowing the count,
/// and the only way to know it is to keep it.
///
/// Forwarding CanCollide to the contained table is exact rather than
/// approximate. The table's rules compare the two bodies' filter POINTERS, and
/// both bodies point at this wrapper, so the comparison means what it meant.
///
/// Declared here rather than in zjolt_group.cpp because a body desc carries a
/// ZJoltCollisionGroup, so the translation units that build a body out of one
/// need the complete type to hand Jolt.
///
/// Reference counted by Jolt: GroupFilter derives from RefTarget<GroupFilter>,
/// so AddRef and Release are its own, and Release is what destroys it —
/// through the operator delete GroupFilter inherits from
/// JPH_DECLARE_RTTI_HELPER, which routes to Jolt's allocator and therefore to
/// the host's.
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

/// Wraps whichever JPH::JobSystem implementation the host asked for.
///
/// The indirection is the seam: a host scheduler becomes a third subclass
/// reached through a new constructor function, and neither this struct nor any
/// call that takes a ZJoltJobSystem* changes shape.
struct ZJoltJobSystem {
  JPH::JobSystem *impl;
  void (*destroy)(JPH::JobSystem *impl);
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

 private:
  ZJoltBodyActivationListener listener_;
};

/// One world.
///
/// The layer adapters live here, by value, because PhysicsSystem::Init stores
/// references to them for the lifetime of the system. Owning them alongside
/// the system is what makes the C API's "the structs are copied, you need not
/// keep them alive" promise true.
struct ZJoltPhysicsSystem {
  JPH::PhysicsSystem system;
  JPH::TempAllocatorImplWithMallocFallback *temp_allocator = nullptr;

  ZJoltBroadPhaseLayerInterfaceAdapter broad_phase_layers;
  ZJoltObjectVsBroadPhaseLayerFilterAdapter object_vs_broad_phase_filter;
  ZJoltObjectLayerPairFilterAdapter object_layer_pair_filter;

  /// Null until a listener is installed; owned when not.
  ZJoltContactListenerAdapter *contact_listener = nullptr;
  ZJoltBodyActivationListenerAdapter *activation_listener = nullptr;

  /// Step listeners still attached, owned. Jolt keeps its own list and asserts
  /// on a double add or a stranger's remove, so this is the list that answers
  /// "is this handle mine" before Jolt is asked. Defined in zjolt_system.cpp.
  JPH::Array<ZJoltStepListener *> step_listeners;

  /// Batches prepared but neither finalized nor aborted, owned. Each one holds
  /// bodies that Jolt has marked as belonging to a broad-phase layer without
  /// having put them in the tree, so destroying the system has to unwind them.
  /// Defined in zjolt_batch.cpp.
  JPH::Array<ZJoltBodyAddBatch *> pending_batches;

  /// Whether Update has ever run on this system.
  ///
  /// `WereBodiesInContact` reads the contact cache's back buffer, and
  /// `ManifoldCache::Find` asserts that buffer has been finalised
  /// (`ContactConstraintManager.cpp:380`) — which it is not until the first
  /// step ends. Asking before then is an ordinary thing for a host to do, and
  /// the honest answer is "no", not an abort.
  bool has_stepped = false;

  /// Index into the combine-callback slot table in zjolt_system.cpp, or -1
  /// when this system has never installed one. Jolt's combine hook is a bare
  /// function pointer with no user parameter, and the slot is what carries the
  /// host's.
  int combine_slot = -1;

  /// Jolt's own combine functions, captured at creation.
  ///
  /// Clearing a combine callback has to put something back, and
  /// `SetCombineFriction(nullptr)` is not it — the solver calls the pointer
  /// unconditionally. Spelled out rather than named through
  /// `ContactConstraintManager::CombineFunction` so this header does not have
  /// to include the constraint manager for one typedef.
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
/// the given shape.
///
/// One inner radius above the shape's lowest point: for a capsule or a sphere
/// that is the centre of its bottom cap, which is the height Jolt's own
/// samples use (spelled there as the standing radius). Both character kinds
/// share it, because both answer the same question with it and a host that
/// swaps one for the other should not find "on the ground" means something
/// different afterwards.
inline JPH::Plane SupportingVolumeFor(const JPH::Shape *shape,
                                      JPH::Vec3Arg up_hint) {
  const JPH::Vec3 up = up_hint.NormalizedOr(JPH::Vec3::sAxisY());
  const JPH::AABox local_bounds = shape->GetLocalBounds();
  const float lowest = up.Dot(local_bounds.mMin);
  return JPH::Plane(up, -(lowest + shape->GetInnerRadius()));
}

}  // namespace zjolt

/// A character controller, plus the system it queries against.
struct ZJoltCharacter {
  JPH::Ref<JPH::CharacterVirtual> impl;
  ZJoltPhysicsSystem *owner;
};

#endif  // ZJOLT_INTERNAL_H_
