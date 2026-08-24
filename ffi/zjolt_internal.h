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
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <cmath>
#include <cstring>
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

/// The three adapters a query needs, built once from an optional
/// ZJoltQueryFilters and passed to Jolt by reference.
struct QueryFilters {
  explicit QueryFilters(const ZJoltQueryFilters *filters)
      : broad_phase(filters != nullptr ? filters->broad_phase_layer
                                       : ZJoltBroadPhaseLayerFilter{}),
        object_layer(filters != nullptr ? filters->object_layer
                                        : ZJoltObjectLayerFilter{}),
        body(filters != nullptr ? filters->body : ZJoltBodyFilter{}) {}

  BroadPhaseLayerFilterAdapter broad_phase;
  ObjectLayerFilterAdapter object_layer;
  BodyFilterAdapter body;
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

  ZJoltPhysicsSystem(const ZJoltPhysicsSystemDesc &desc)
      : broad_phase_layers(desc.broad_phase_layers),
        object_vs_broad_phase_filter(desc.object_vs_broad_phase_filter),
        object_layer_pair_filter(desc.object_layer_pair_filter) {}
};

/// A character controller, plus the system it queries against.
struct ZJoltCharacter {
  JPH::Ref<JPH::CharacterVirtual> impl;
  ZJoltPhysicsSystem *owner;
};

#endif  // ZJOLT_INTERNAL_H_
