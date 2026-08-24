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
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/PhysicsSystem.h>

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
// The handles zjolt allocates itself (physics system, job system, character)
// are real structs, defined at the bottom of this header.
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

/// A character controller, plus the system it queries against.
struct ZJoltCharacter {
  JPH::Ref<JPH::CharacterVirtual> impl;
  ZJoltPhysicsSystem *owner;
};

#endif  // ZJOLT_INTERNAL_H_
