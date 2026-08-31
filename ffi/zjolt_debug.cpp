//===----------------------------------------------------------------------===//
// zjolt — debug-draw geometry, collected into arrays.
//
// Every entry point here is declared unconditionally in zjolt_debug.h and
// returns ZJOLT_RESULT_UNSUPPORTED when this library was not built with
// -Ddebug_renderer=true. That is what lets the header stay fixed regardless
// of the build: everything JPH_DEBUG_RENDERER-shaped — the subclass, the
// conversions, Jolt's own DrawSettings type — lives inside the #ifdef below,
// and only the plain, always-available C structs cross it.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include "zjolt_debug.h"

#ifdef JPH_DEBUG_RENDERER

#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Constraints/ConstraintManager.h>
#include <Jolt/Renderer/DebugRendererPlayback.h>
#include <Jolt/Renderer/DebugRendererRecorder.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Renderer/DebugRendererSimple.h>

#include <atomic>
#include <cstring>
#include <vector>

namespace {

//===----------------------------------------------------------------------===//
// The sink
//
// DebugRendererSimple already turns Jolt's batched-geometry machinery into
// calls to DrawLine / DrawTriangle / DrawText3D (see its class comment) —
// overriding those three is the entire subclass; nothing here mirrors Jolt's
// vtable.
//===----------------------------------------------------------------------===//

struct RecordedLine {
  JPH::RVec3 from;
  JPH::RVec3 to;
  JPH::Color color;
};

struct RecordedTriangle {
  JPH::RVec3 v1;
  JPH::RVec3 v2;
  JPH::RVec3 v3;
  JPH::Color color;
  bool cast_shadow;
};

struct RecordedText {
  JPH::RVec3 position;
  JPH::Color color;
  float height;
  char text[ZJOLT_DEBUG_TEXT_MAX_LENGTH];
  uint32_t length;
};

//===----------------------------------------------------------------------===//
// Batched geometry — the host-owned side
//
// HostBatch is the RefTargetVirtual DebugRenderer::CreateTriangleBatch
// returns once zjoltDebugRendererSetBatchCallbacks is active: a thin
// ref-counted wrapper around the host's create-callback return value,
// routing Ref<>/AddRef/Release into destroy_batch. A batch built by
// DebugRendererSimple's own fallback — always true of Jolt's shared
// box/sphere/capsule/cylinder/cone primitives, built once from its
// constructor before any callback exists — has no HostBatch behind it.
// This library builds with -fno-rtti, so DrawGeometry below tells the two
// apart with the registry that follows, not a dynamic_cast.
//===----------------------------------------------------------------------===//

ZJoltCullMode ToCCullMode(JPH::DebugRenderer::ECullMode cull_mode) {
  return static_cast<ZJoltCullMode>(cull_mode);
}

ZJoltDebugVertex ToCVertex(const JPH::DebugRenderer::Vertex &v) {
  return ZJoltDebugVertex{
      {v.mPosition.x, v.mPosition.y, v.mPosition.z},
      {v.mNormal.x, v.mNormal.y, v.mNormal.z},
      v.mUV.x,
      v.mUV.y,
      zjolt::ToC(v.mColor),
  };
}

/// Every live HostBatch, by address — process-wide because a Shape's cached
/// Geometry (and the HostBatch objects its LODs reference) outlives any one
/// ZJoltDebugRendererImpl. std::vector, not JPH::Array: its storage outlives
/// any one zjoltInit/zjoltDeinit cycle, so it cannot route through the
/// allocator installed for the duration of one. Not thread-safe.
std::vector<const void *> &HostBatchRegistry() {
  static std::vector<const void *> registry;
  return registry;
}

void RegisterHostBatch(const void *batch) { HostBatchRegistry().push_back(batch); }

/// Swap-erase: registry order carries no meaning.
void UnregisterHostBatch(const void *batch) {
  std::vector<const void *> &registry = HostBatchRegistry();
  for (size_t i = 0; i < registry.size(); ++i) {
    if (registry[i] == batch) {
      registry[i] = registry.back();
      registry.pop_back();
      return;
    }
  }
}

bool IsHostBatch(const void *batch) {
  for (const void *b : HostBatchRegistry())
    if (b == batch) return true;
  return false;
}

class HostBatch final : public JPH::RefTargetVirtual {
 public:
  HostBatch(const ZJoltDebugBatchCallbacks &callbacks, void *handle)
      : callbacks_(callbacks), handle_(handle) {
    RegisterHostBatch(this);
  }

  void AddRef() override { ref_count_.fetch_add(1, std::memory_order_relaxed); }

  /// Runs destroy_batch exactly once, on the transition to zero — see
  /// ZJoltDebugBatchCallbacks in zjolt_debug.h for the contract this hands
  /// to the host.
  void Release() override {
    if (ref_count_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      UnregisterHostBatch(this);
      if (callbacks_.destroy_batch != nullptr)
        callbacks_.destroy_batch(callbacks_.user, handle_);
      zjolt::Delete(this);
    }
  }

  void *Handle() const { return handle_; }

 private:
  ZJoltDebugBatchCallbacks callbacks_;
  void *handle_;
  std::atomic<uint32_t> ref_count_{0};
};

class ZJoltDebugRendererImpl final : public JPH::DebugRendererSimple {
 public:
  void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo,
                JPH::ColorArg inColor) override {
    lines.push_back(RecordedLine{inFrom, inTo, inColor});
  }

  void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3,
                    JPH::ColorArg inColor,
                    ECastShadow inCastShadow) override {
    triangles.push_back(RecordedTriangle{
        inV1, inV2, inV3, inColor, inCastShadow == ECastShadow::On});
  }

  void DrawText3D(JPH::RVec3Arg inPosition, const JPH::string_view &inString,
                  JPH::ColorArg inColor, float inHeight) override {
    // Value-initialized so the tail of `text` past the terminator is a
    // deterministic zero rather than whatever this stack frame held before —
    // that tail is copied whole into the caller's buffer below.
    RecordedText entry{};
    entry.position = inPosition;
    entry.color = inColor;
    entry.height = inHeight;
    const size_t n = inString.size();
    const size_t copy = n < sizeof(entry.text) - 1 ? n : sizeof(entry.text) - 1;
    std::memcpy(entry.text, inString.data(), copy);
    entry.text[copy] = '\0';
    entry.length = static_cast<uint32_t>(n);
    texts.push_back(entry);
  }

  /// Empties every buffer and tells the base class a new frame started, which
  /// resets the level-of-detail timing DrawGeometry reads.
  void ClearFrame() {
    lines.clear();
    triangles.clear();
    texts.clear();
    NextFrame();
  }

  /// SetCameraPos plus a shadow copy DrawGeometry below reads: the base
  /// class keeps its own camera fields private to itself, out of an
  /// override's reach.
  void TrackCameraPosition(JPH::RVec3Arg position) {
    SetCameraPos(position);
    camera_pos_ = position;
    camera_pos_set_ = true;
  }

  /// NULL clears back to the immediate-mode fallback for both overrides
  /// below; a non-NULL table is copied whole, replacing whichever was set.
  void SetBatchCallbacks(const ZJoltDebugBatchCallbacks *callbacks) {
    batch_callbacks_ = callbacks == nullptr ? ZJoltDebugBatchCallbacks{} : *callbacks;
  }

  /// NULL create_triangle_batch, a NULL host return, or a wrapper allocation
  /// failure all fall back to the base class's own BatchImpl path — the
  /// last case only after telling the host to undo what it already built,
  /// so a batch it made is never silently orphaned.
  Batch CreateTriangleBatch(const Triangle *inTriangles, int inTriangleCount) override {
    if (batch_callbacks_.create_triangle_batch == nullptr)
      return DebugRendererSimple::CreateTriangleBatch(inTriangles, inTriangleCount);

    JPH::Array<ZJoltDebugVertex> flat;
    flat.reserve(static_cast<size_t>(inTriangleCount) * 3);
    for (int t = 0; t < inTriangleCount; ++t)
      for (int v = 0; v < 3; ++v) flat.push_back(ToCVertex(inTriangles[t].mV[v]));

    void *handle = batch_callbacks_.create_triangle_batch(
        batch_callbacks_.user, flat.empty() ? nullptr : flat.data(),
        static_cast<uint32_t>(flat.size()));
    return WrapOrFallBack(handle, [&] {
      return DebugRendererSimple::CreateTriangleBatch(inTriangles, inTriangleCount);
    });
  }

  /// Same fallback rules as the Triangle overload above.
  Batch CreateTriangleBatch(const Vertex *inVertices, int inVertexCount,
                            const uint32_t *inIndices, int inIndexCount) override {
    if (batch_callbacks_.create_triangle_batch_indexed == nullptr) {
      return DebugRendererSimple::CreateTriangleBatch(inVertices, inVertexCount,
                                                       inIndices, inIndexCount);
    }

    JPH::Array<ZJoltDebugVertex> c_vertices;
    c_vertices.reserve(static_cast<size_t>(inVertexCount));
    for (int i = 0; i < inVertexCount; ++i) c_vertices.push_back(ToCVertex(inVertices[i]));

    void *handle = batch_callbacks_.create_triangle_batch_indexed(
        batch_callbacks_.user, c_vertices.empty() ? nullptr : c_vertices.data(),
        static_cast<uint32_t>(c_vertices.size()), inIndices,
        static_cast<uint32_t>(inIndexCount));
    return WrapOrFallBack(handle, [&] {
      return DebugRendererSimple::CreateTriangleBatch(inVertices, inVertexCount,
                                                       inIndices, inIndexCount);
    });
  }

  /// Builds the LOD chain and selected index CreateTriangleBatch's caller
  /// sees as ZJoltDebugGeometry, resolving the selection with Jolt's own
  /// Geometry::GetLOD exactly as the base class's own fallback would.
  void DrawGeometry(JPH::RMat44Arg inModelMatrix, const JPH::AABox &inWorldSpaceBounds,
                    float inLODScaleSq, JPH::ColorArg inModelColor,
                    const GeometryRef &inGeometry, ECullMode inCullMode,
                    ECastShadow inCastShadow, EDrawMode inDrawMode) override {
    if (batch_callbacks_.draw_geometry == nullptr) {
      DebugRendererSimple::DrawGeometry(inModelMatrix, inWorldSpaceBounds, inLODScaleSq,
                                        inModelColor, inGeometry, inCullMode, inCastShadow,
                                        inDrawMode);
      return;
    }

    JPH::Array<ZJoltDebugLOD> c_lods;
    c_lods.reserve(inGeometry->mLODs.size());
    for (const LOD &lod : inGeometry->mLODs) {
      void *handle = nullptr;
      const JPH::RefTargetVirtual *raw = lod.mTriangleBatch.GetPtr();
      if (raw != nullptr && IsHostBatch(raw))
        handle = static_cast<const HostBatch *>(raw)->Handle();
      c_lods.push_back(ZJoltDebugLOD{handle, lod.mDistance});
    }

    uint32_t selected = 0;
    if (camera_pos_set_) {
      const LOD &chosen =
          inGeometry->GetLOD(JPH::Vec3(camera_pos_), inWorldSpaceBounds, inLODScaleSq);
      selected = static_cast<uint32_t>(&chosen - inGeometry->mLODs.data());
    }

    const ZJoltRMat44 c_matrix = zjolt::ToCR(inModelMatrix);
    const ZJoltAABox c_world_bounds = zjolt::ToC(inWorldSpaceBounds);
    const ZJoltDebugGeometry c_geometry{c_lods.data(), static_cast<uint32_t>(c_lods.size()),
                                        selected, zjolt::ToC(inGeometry->mBounds)};
    batch_callbacks_.draw_geometry(batch_callbacks_.user, &c_matrix, &c_world_bounds,
                                   inLODScaleSq, zjolt::ToC(inModelColor), &c_geometry,
                                   ToCCullMode(inCullMode),
                                   static_cast<ZJoltCastShadow>(inCastShadow),
                                   static_cast<ZJoltDrawMode>(inDrawMode));
  }

  JPH::Array<RecordedLine> lines;
  JPH::Array<RecordedTriangle> triangles;
  JPH::Array<RecordedText> texts;

 private:
  /// NULL `handle` (host declined) or a failed wrapper allocation both defer
  /// to `fallback`; a failed allocation first tells the host to destroy the
  /// handle it already built, so nothing it made is leaked.
  template <typename Fallback>
  Batch WrapOrFallBack(void *handle, Fallback &&fallback) {
    if (handle == nullptr) return fallback();
    HostBatch *wrapper = zjolt::New<HostBatch>(batch_callbacks_, handle);
    if (wrapper == nullptr) {
      if (batch_callbacks_.destroy_batch != nullptr)
        batch_callbacks_.destroy_batch(batch_callbacks_.user, handle);
      return fallback();
    }
    return wrapper;
  }

  ZJoltDebugBatchCallbacks batch_callbacks_{};
  JPH::RVec3 camera_pos_ = JPH::RVec3(0, 0, 0);
  bool camera_pos_set_ = false;
};

//===----------------------------------------------------------------------===//
// ZJoltDebugRenderer is reinterpreted as JPH::DebugRenderer, not the
// concrete Impl: a handle may be zjoltDebugRecorderAsRenderer's view of an
// unrelated DebugRendererRecorder (both share DebugRenderer's base address).
// ToImpl narrows to the concrete sink ONLY where needed (Clear,
// SetCameraPosition, GetX read-backs) — never otherwise.
//===----------------------------------------------------------------------===//

inline JPH::DebugRenderer *ToBase(ZJoltDebugRenderer *renderer) {
  return reinterpret_cast<JPH::DebugRenderer *>(renderer);
}

inline const JPH::DebugRenderer *ToBase(const ZJoltDebugRenderer *renderer) {
  return reinterpret_cast<const JPH::DebugRenderer *>(renderer);
}

inline ZJoltDebugRenderer *FromBase(JPH::DebugRenderer *renderer) {
  return reinterpret_cast<ZJoltDebugRenderer *>(renderer);
}

inline ZJoltDebugRendererImpl *ToImpl(ZJoltDebugRenderer *renderer) {
  return static_cast<ZJoltDebugRendererImpl *>(ToBase(renderer));
}

inline const ZJoltDebugRendererImpl *ToImpl(const ZJoltDebugRenderer *renderer) {
  return static_cast<const ZJoltDebugRendererImpl *>(ToBase(renderer));
}

inline ZJoltDebugRenderer *FromImpl(ZJoltDebugRendererImpl *impl) {
  return FromBase(static_cast<JPH::DebugRenderer *>(impl));
}

ZJoltShapeColor ToCShapeColor(JPH::BodyManager::EShapeColor color) {
  return static_cast<ZJoltShapeColor>(color);
}

// These three take the raw integer, not the enum: every value they convert
// arrives from a host, as a draw-entry parameter or a
// ZJoltDebugDrawBodiesSettings field, and reading it as its enum type loads
// a value this ABI never validated (see zjolt::RawEnum, zjolt_internal.h).
// The last two are named apart rather than overloaded on ToJolt: once both
// parameters are int32_t, overload resolution has nothing left to tell them by.
JPH::BodyManager::EShapeColor ToJoltShapeColor(int32_t color) {
  return static_cast<JPH::BodyManager::EShapeColor>(color);
}

JPH::DebugRenderer::ECastShadow ToJoltCastShadow(int32_t cast_shadow) {
  return static_cast<JPH::DebugRenderer::ECastShadow>(cast_shadow);
}

JPH::DebugRenderer::EDrawMode ToJoltDrawMode(int32_t draw_mode) {
  return static_cast<JPH::DebugRenderer::EDrawMode>(draw_mode);
}

//===----------------------------------------------------------------------===//
// Body-draw filtering
//===----------------------------------------------------------------------===//

class ZJoltBodyDrawFilterImpl final : public JPH::BodyDrawFilter {
 public:
  ZJoltBodyDrawFilter table{};

  bool ShouldDraw(const JPH::Body &inBody) const override {
    if (table.should_draw == nullptr) return true;
    return table.should_draw(table.user, zjolt::ToC(inBody.GetID()));
  }
};

//===----------------------------------------------------------------------===//
// Recording
//
// Captured into a flat, growable buffer rather than Jolt's
// StreamInWrapper/StreamOutWrapper over std::istream/std::ostream, for the same
// reason zjolt_internal.h's CountingStreamOut/ConstStreamIn exist. A
// recording's final size is not known up front, so unlike a shape save this
// cannot be two-pass count-then-write: it appends every write and hands the
// accumulated bytes to the same container-framing helpers when a host finally
// asks for them.
//===----------------------------------------------------------------------===//

class GrowingStreamOut final : public JPH::StreamOut {
 public:
  void WriteBytes(const void *inData, size_t inNumBytes) override {
    const uint8_t *bytes = static_cast<const uint8_t *>(inData);
    data.insert(data.end(), bytes, bytes + inNumBytes);
  }

  bool IsFailed() const override { return false; }

  JPH::Array<uint8_t> data;
};

/// The shared container from zjolt_internal.h, with this pair's own tag and
/// no extra fields of its own.
constexpr zjolt::ContainerFormat kRecordingContainer = {
    /*magic=*/{'Z', 'J', 'D', 'R'},
    /*version=*/1,
    /*extra_size=*/0,
    /*too_short=*/"too short to be a saved debug recording",
    /*wrong_magic=*/"not a recording saved by zjoltDebugRecorderGetData",
    /*bad_checksum=*/"the recording payload failed its checksum",
};

/// Runs `fn` against a locked soft body and the transform to draw it with.
/// A NULL `com_transform` means the body's own.
template <typename Fn>
ZJoltResult WithSoftBodyDraw(const ZJoltPhysicsSystem *system, ZJoltBodyId body,
                             const ZJoltRMat44 *com_transform, Fn &&fn) {
  const JPH::BodyLockRead lock(system->system.GetBodyLockInterface(),
                               zjolt::ToJolt(body));
  if (!lock.Succeeded())
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "no body with that id is in this physics system");
  const JPH::Body &jolt_body = lock.GetBody();
  if (!jolt_body.IsSoftBody())
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "that body is not a soft body");
  const JPH::RMat44 transform = com_transform == nullptr
                                    ? jolt_body.GetCenterOfMassTransform()
                                    : zjolt::ToJoltR(*com_transform);
  return fn(*static_cast<const JPH::SoftBodyMotionProperties *>(
                jolt_body.GetMotionPropertiesUnchecked()),
            transform);
}

/// The host populated this, so it is read as raw bits — see zjolt::RawEnum.
JPH::ESoftBodyConstraintColor ToJoltConstraintColor(int32_t raw) {
  switch (raw) {
    case ZJOLT_SOFT_BODY_CONSTRAINT_COLOR_GROUP:
      return JPH::ESoftBodyConstraintColor::ConstraintGroup;
    case ZJOLT_SOFT_BODY_CONSTRAINT_COLOR_ORDER:
      return JPH::ESoftBodyConstraintColor::ConstraintOrder;
    default:
      return JPH::ESoftBodyConstraintColor::ConstraintType;
  }
}

}  // namespace

//===----------------------------------------------------------------------===//
// The recorder and playback handles
//
// Defined here rather than in zjolt_internal.h, matching ZJoltVehicleConstraint
// in zjolt_vehicle.cpp: nothing outside this file ever sees either pointer.
//===----------------------------------------------------------------------===//

struct ZJoltDebugRecorder {
  GrowingStreamOut stream;
  JPH::DebugRendererRecorder recorder{stream};
};

struct ZJoltDebugPlayback {
  JPH::DebugRendererPlayback playback;

  explicit ZJoltDebugPlayback(JPH::DebugRenderer &target) : playback(target) {}
};

#endif  // JPH_DEBUG_RENDERER

extern "C" {

ZJoltResult zjoltDebugRendererCreate(ZJoltDebugRenderer **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  // Jolt's DebugRenderer constructor already asserts this in a build with
  // asserts enabled (DebugRenderer.cpp: `JPH_ASSERT(sInstance == nullptr)`).
  // Checking it here as well means a release build without asserts refuses
  // cleanly instead of silently repointing the singleton every deep call site
  // that reaches for `DebugRenderer::sInstance` directly reads from.
  if (JPH::DebugRenderer::sInstance != nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_ALREADY_INITIALIZED,
        "a ZJoltDebugRenderer already exists; Jolt's DebugRenderer is a "
        "process-wide singleton and only one may be alive at a time");
  }
  ZJoltDebugRendererImpl *renderer = zjolt::New<ZJoltDebugRendererImpl>();
  if (renderer == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  *out = FromImpl(renderer);
  zjolt::HandleCreated();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

void zjoltDebugRendererDestroy(ZJoltDebugRenderer *renderer) {
  if (renderer == nullptr) return;
#ifdef JPH_DEBUG_RENDERER
  zjolt::Delete(ToImpl(renderer));
  zjolt::HandleDestroyed();
#else
  // Unreachable: zjoltDebugRendererCreate reports ZJOLT_RESULT_UNSUPPORTED in
  // this build and writes no handle, so there is nothing here to have been
  // handed one.
  (void)renderer;
#endif
}

ZJoltResult zjoltDebugRendererClear(ZJoltDebugRenderer *renderer) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToImpl(renderer)->ClearFrame();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererSetCameraPosition(ZJoltDebugRenderer *renderer,
                                                const ZJoltRVec3 *position) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, position)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToImpl(renderer)->TrackCameraPosition(zjolt::ToJoltR(*position));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererSetBatchCallbacks(
    ZJoltDebugRenderer *renderer, const ZJoltDebugBatchCallbacks *callbacks) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  if (callbacks != nullptr && callbacks->draw_geometry == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "callbacks is non-NULL but its draw_geometry field is NULL");
  }
  ToImpl(renderer)->SetBatchCallbacks(callbacks);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererGetLines(const ZJoltDebugRenderer *renderer,
                                       ZJoltDebugLine *lines, uint32_t capacity,
                                       uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(renderer, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const auto *impl = ToImpl(renderer);
  const uint32_t count = static_cast<uint32_t>(impl->lines.size());
  *out_count = count;
  if (lines == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) {
    const RecordedLine &line = impl->lines[i];
    lines[i].from = zjolt::ToCR(line.from);
    lines[i].to = zjolt::ToCR(line.to);
    lines[i].color = zjolt::ToC(line.color);
  }
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererGetTriangles(const ZJoltDebugRenderer *renderer,
                                           ZJoltDebugTriangle *triangles,
                                           uint32_t capacity,
                                           uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(renderer, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const auto *impl = ToImpl(renderer);
  const uint32_t count = static_cast<uint32_t>(impl->triangles.size());
  *out_count = count;
  if (triangles == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) {
    const RecordedTriangle &tri = impl->triangles[i];
    triangles[i].v1 = zjolt::ToCR(tri.v1);
    triangles[i].v2 = zjolt::ToCR(tri.v2);
    triangles[i].v3 = zjolt::ToCR(tri.v3);
    triangles[i].color = zjolt::ToC(tri.color);
    triangles[i].cast_shadow = tri.cast_shadow;
  }
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererGetTexts(const ZJoltDebugRenderer *renderer,
                                       ZJoltDebugText *texts, uint32_t capacity,
                                       uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(renderer, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const auto *impl = ToImpl(renderer);
  const uint32_t count = static_cast<uint32_t>(impl->texts.size());
  *out_count = count;
  if (texts == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) {
    const RecordedText &text = impl->texts[i];
    texts[i].position = zjolt::ToCR(text.position);
    texts[i].color = zjolt::ToC(text.color);
    texts[i].height = text.height;
    std::memcpy(texts[i].text, text.text, sizeof(texts[i].text));
    texts[i].text_length = text.length;
  }
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

//===----------------------------------------------------------------------===//
// Host-issued primitives
//
// Each forwards straight into the matching JPH::DebugRenderer method on
// `ToBase(renderer)` — DrawLine/DrawTriangle dispatch through the vtable into
// whichever concrete renderer it actually is; the rest are DebugRenderer's own
// non-virtual helpers, decomposing into those two exactly as for a C++ caller
// holding the same pointer.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltDebugRendererDrawLine(ZJoltDebugRenderer *renderer,
                                       const ZJoltRVec3 *from,
                                       const ZJoltRVec3 *to, ZJoltColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, from, to)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToBase(renderer)->DrawLine(zjolt::ToJoltR(*from), zjolt::ToJoltR(*to),
                             zjolt::ToJolt(color));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawTriangle(ZJoltDebugRenderer *renderer,
                                           const ZJoltRVec3 *v1,
                                           const ZJoltRVec3 *v2,
                                           const ZJoltRVec3 *v3,
                                           ZJoltColor color,
                                           ZJoltCastShadow cast_shadow) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, v1, v2, v3)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_cast_shadow = zjolt::RawEnum(cast_shadow);
  ToBase(renderer)->DrawTriangle(zjolt::ToJoltR(*v1), zjolt::ToJoltR(*v2),
                                 zjolt::ToJoltR(*v3), zjolt::ToJolt(color),
                                 ToJoltCastShadow(raw_cast_shadow));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawText3D(ZJoltDebugRenderer *renderer,
                                         const ZJoltRVec3 *position,
                                         const char *text, uint32_t text_length,
                                         ZJoltColor color, float height) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, position)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  if (text == nullptr && text_length != 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "text is NULL but text_length is not 0");
  }
  const JPH::string_view view(text == nullptr ? "" : text, text_length);
  ToBase(renderer)->DrawText3D(zjolt::ToJoltR(*position), view,
                               zjolt::ToJolt(color), height);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawWireBox(ZJoltDebugRenderer *renderer,
                                          const ZJoltAABox *box,
                                          ZJoltColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, box)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::AABox jolt_box(zjolt::ToJolt(box->min), zjolt::ToJolt(box->max));
  ToBase(renderer)->DrawWireBox(jolt_box, zjolt::ToJolt(color));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawBox(ZJoltDebugRenderer *renderer,
                                      const ZJoltAABox *box, ZJoltColor color,
                                      ZJoltCastShadow cast_shadow,
                                      ZJoltDrawMode draw_mode) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, box)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const int32_t raw_cast_shadow = zjolt::RawEnum(cast_shadow);
  const int32_t raw_draw_mode = zjolt::RawEnum(draw_mode);
  const JPH::AABox jolt_box(zjolt::ToJolt(box->min), zjolt::ToJolt(box->max));
  ToBase(renderer)->DrawBox(jolt_box, zjolt::ToJolt(color),
                            ToJoltCastShadow(raw_cast_shadow),
                            ToJoltDrawMode(raw_draw_mode));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawSphere(ZJoltDebugRenderer *renderer,
                                         const ZJoltRVec3 *center, float radius,
                                         ZJoltColor color,
                                         ZJoltCastShadow cast_shadow,
                                         ZJoltDrawMode draw_mode) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, center)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const int32_t raw_cast_shadow = zjolt::RawEnum(cast_shadow);
  const int32_t raw_draw_mode = zjolt::RawEnum(draw_mode);
  ToBase(renderer)->DrawSphere(zjolt::ToJoltR(*center), radius,
                               zjolt::ToJolt(color),
                               ToJoltCastShadow(raw_cast_shadow),
                               ToJoltDrawMode(raw_draw_mode));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawWireSphere(ZJoltDebugRenderer *renderer,
                                             const ZJoltRVec3 *center,
                                             float radius, ZJoltColor color,
                                             int32_t level) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, center)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToBase(renderer)->DrawWireSphere(zjolt::ToJoltR(*center), radius,
                                   zjolt::ToJolt(color), level);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawUnitSphere(ZJoltDebugRenderer *renderer,
                                             const ZJoltRMat44 *matrix,
                                             ZJoltColor color,
                                             ZJoltCastShadow cast_shadow,
                                             ZJoltDrawMode draw_mode) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, matrix)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const int32_t raw_cast_shadow = zjolt::RawEnum(cast_shadow);
  const int32_t raw_draw_mode = zjolt::RawEnum(draw_mode);
  ToBase(renderer)->DrawUnitSphere(zjolt::ToJoltR(*matrix), zjolt::ToJolt(color),
                                   ToJoltCastShadow(raw_cast_shadow),
                                   ToJoltDrawMode(raw_draw_mode));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawWireUnitSphere(ZJoltDebugRenderer *renderer,
                                                 const ZJoltRMat44 *matrix,
                                                 ZJoltColor color,
                                                 int32_t level) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, matrix)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToBase(renderer)->DrawWireUnitSphere(zjolt::ToJoltR(*matrix),
                                       zjolt::ToJolt(color), level);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawCapsule(ZJoltDebugRenderer *renderer,
                                          const ZJoltRMat44 *matrix,
                                          float half_height_of_cylinder,
                                          float radius, ZJoltColor color,
                                          ZJoltCastShadow cast_shadow,
                                          ZJoltDrawMode draw_mode) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, matrix)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const int32_t raw_cast_shadow = zjolt::RawEnum(cast_shadow);
  const int32_t raw_draw_mode = zjolt::RawEnum(draw_mode);
  ToBase(renderer)->DrawCapsule(zjolt::ToJoltR(*matrix), half_height_of_cylinder,
                                radius, zjolt::ToJolt(color),
                                ToJoltCastShadow(raw_cast_shadow),
                                ToJoltDrawMode(raw_draw_mode));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawCylinder(ZJoltDebugRenderer *renderer,
                                           const ZJoltRMat44 *matrix,
                                           float half_height, float radius,
                                           ZJoltColor color,
                                           ZJoltCastShadow cast_shadow,
                                           ZJoltDrawMode draw_mode) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, matrix)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const int32_t raw_cast_shadow = zjolt::RawEnum(cast_shadow);
  const int32_t raw_draw_mode = zjolt::RawEnum(draw_mode);
  ToBase(renderer)->DrawCylinder(zjolt::ToJoltR(*matrix), half_height, radius,
                                 zjolt::ToJolt(color),
                                 ToJoltCastShadow(raw_cast_shadow),
                                 ToJoltDrawMode(raw_draw_mode));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawTaperedCylinder(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *matrix, float top,
    float bottom, float top_radius, float bottom_radius, ZJoltColor color,
    ZJoltCastShadow cast_shadow, ZJoltDrawMode draw_mode) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, matrix)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const int32_t raw_cast_shadow = zjolt::RawEnum(cast_shadow);
  const int32_t raw_draw_mode = zjolt::RawEnum(draw_mode);
  ToBase(renderer)->DrawTaperedCylinder(
      zjolt::ToJoltR(*matrix), top, bottom, top_radius, bottom_radius,
      zjolt::ToJolt(color), ToJoltCastShadow(raw_cast_shadow),
      ToJoltDrawMode(raw_draw_mode));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawPie(ZJoltDebugRenderer *renderer,
                                      const ZJoltRVec3 *center, float radius,
                                      const ZJoltVec3 *normal,
                                      const ZJoltVec3 *axis, float min_angle,
                                      float max_angle, ZJoltColor color,
                                      ZJoltCastShadow cast_shadow,
                                      ZJoltDrawMode draw_mode) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, center, normal, axis))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const int32_t raw_cast_shadow = zjolt::RawEnum(cast_shadow);
  const int32_t raw_draw_mode = zjolt::RawEnum(draw_mode);
  ToBase(renderer)->DrawPie(zjolt::ToJoltR(*center), radius,
                            zjolt::ToJolt(*normal), zjolt::ToJolt(*axis),
                            min_angle, max_angle, zjolt::ToJolt(color),
                            ToJoltCastShadow(raw_cast_shadow),
                            ToJoltDrawMode(raw_draw_mode));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawArrow(ZJoltDebugRenderer *renderer,
                                        const ZJoltRVec3 *from,
                                        const ZJoltRVec3 *to, ZJoltColor color,
                                        float size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, from, to)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToBase(renderer)->DrawArrow(zjolt::ToJoltR(*from), zjolt::ToJoltR(*to),
                              zjolt::ToJolt(color), size);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawMarker(ZJoltDebugRenderer *renderer,
                                         const ZJoltRVec3 *position,
                                         ZJoltColor color, float size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, position)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToBase(renderer)->DrawMarker(zjolt::ToJoltR(*position), zjolt::ToJolt(color),
                               size);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawCoordinateSystem(ZJoltDebugRenderer *renderer,
                                                   const ZJoltRMat44 *transform,
                                                   float size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, transform)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToBase(renderer)->DrawCoordinateSystem(zjolt::ToJoltR(*transform), size);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawPlane(ZJoltDebugRenderer *renderer,
                                        const ZJoltRVec3 *point,
                                        const ZJoltVec3 *normal,
                                        ZJoltColor color, float size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, point, normal))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToBase(renderer)->DrawPlane(zjolt::ToJoltR(*point), zjolt::ToJolt(*normal),
                              zjolt::ToJolt(color), size);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawWireTriangle(ZJoltDebugRenderer *renderer,
                                               const ZJoltRVec3 *v1,
                                               const ZJoltRVec3 *v2,
                                               const ZJoltRVec3 *v3,
                                               ZJoltColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, v1, v2, v3)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ToBase(renderer)->DrawWireTriangle(zjolt::ToJoltR(*v1), zjolt::ToJoltR(*v2),
                                     zjolt::ToJoltR(*v3), zjolt::ToJolt(color));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRendererDrawWirePolygon(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *transform,
    const ZJoltVec3 *vertices, uint32_t vertex_count, ZJoltColor color,
    float arrow_size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer, transform))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  // Kept inside the #ifdef, unlike zjolt::Present above: every entry point in
  // this file reports ZJOLT_RESULT_UNSUPPORTED unconditionally when the
  // library was not built with the flag, regardless of what else is wrong
  // with the call, and a validation check placed ahead of the #ifdef would
  // quietly carve out an exception to that for this one function.
  if (vertices == nullptr || vertex_count < 2) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a wire polygon needs at least 2 vertices");
  }
  const JPH::RMat44 jolt_transform = zjolt::ToJoltR(*transform);
  const JPH::Color jolt_color = zjolt::ToJolt(color);
  JPH::DebugRenderer *target = ToBase(renderer);
  for (uint32_t i = 0; i < vertex_count; ++i) {
    const JPH::Vec3 a = zjolt::ToJolt(vertices[i]);
    const JPH::Vec3 b = zjolt::ToJolt(vertices[(i + 1) % vertex_count]);
    target->DrawArrow(jolt_transform * a, jolt_transform * b, jolt_color,
                      arrow_size);
  }
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugDrawBodiesSettingsInit(
    ZJoltDebugDrawBodiesSettings *settings) {
  ZJOLT_ENTER(settings);
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::BodyManager::DrawSettings defaults;
  settings->draw_get_support_function = defaults.mDrawGetSupportFunction;
  settings->draw_support_direction = defaults.mDrawSupportDirection;
  settings->draw_get_supporting_face = defaults.mDrawGetSupportingFace;
  settings->draw_shape = defaults.mDrawShape;
  settings->draw_shape_wireframe = defaults.mDrawShapeWireframe;
  settings->shape_color = ToCShapeColor(defaults.mDrawShapeColor);
  settings->draw_bounding_box = defaults.mDrawBoundingBox;
  settings->draw_center_of_mass_transform = defaults.mDrawCenterOfMassTransform;
  settings->draw_world_transform = defaults.mDrawWorldTransform;
  settings->draw_velocity = defaults.mDrawVelocity;
  settings->draw_mass_and_inertia = defaults.mDrawMassAndInertia;
  settings->draw_sleep_stats = defaults.mDrawSleepStats;
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltPhysicsSystemDrawBodies(
    ZJoltPhysicsSystem *system, const ZJoltDebugDrawBodiesSettings *settings,
    ZJoltDebugRenderer *renderer, const ZJoltBodyDrawFilter *filter) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, settings, renderer))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  JPH::BodyManager::DrawSettings draw_settings;
  draw_settings.mDrawGetSupportFunction = settings->draw_get_support_function;
  draw_settings.mDrawSupportDirection = settings->draw_support_direction;
  draw_settings.mDrawGetSupportingFace = settings->draw_get_supporting_face;
  draw_settings.mDrawShape = settings->draw_shape;
  draw_settings.mDrawShapeWireframe = settings->draw_shape_wireframe;
  // Read out as raw bits, not as its enum type: the host populated this
  // field — see zjolt::RawEnum in zjolt_internal.h.
  draw_settings.mDrawShapeColor =
      ToJoltShapeColor(zjolt::RawEnum(settings->shape_color));
  draw_settings.mDrawBoundingBox = settings->draw_bounding_box;
  draw_settings.mDrawCenterOfMassTransform =
      settings->draw_center_of_mass_transform;
  draw_settings.mDrawWorldTransform = settings->draw_world_transform;
  draw_settings.mDrawVelocity = settings->draw_velocity;
  draw_settings.mDrawMassAndInertia = settings->draw_mass_and_inertia;
  draw_settings.mDrawSleepStats = settings->draw_sleep_stats;

  ZJoltBodyDrawFilterImpl filter_impl;
  const JPH::BodyDrawFilter *jolt_filter = nullptr;
  if (filter != nullptr) {
    filter_impl.table = *filter;
    jolt_filter = &filter_impl;
  }

  system->system.DrawBodies(draw_settings, ToBase(renderer), jolt_filter);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltPhysicsSystemDrawConstraints(ZJoltPhysicsSystem *system,
                                              ZJoltDebugRenderer *renderer) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  system->system.DrawConstraints(ToBase(renderer));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltPhysicsSystemDrawConstraintLimits(
    ZJoltPhysicsSystem *system, ZJoltDebugRenderer *renderer) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  system->system.DrawConstraintLimits(ToBase(renderer));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltPhysicsSystemDrawConstraintReferenceFrame(
    ZJoltPhysicsSystem *system, ZJoltDebugRenderer *renderer) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  system->system.DrawConstraintReferenceFrame(ToBase(renderer));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

//===----------------------------------------------------------------------===//
// A shape's own debugging helper
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeDrawShrunkShape(const ZJoltShape *shape,
                                      const ZJoltRMat44 *center_of_mass_transform,
                                      ZJoltVec3 scale,
                                      ZJoltDebugRenderer *renderer) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, center_of_mass_transform, renderer))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::Shape *jolt_shape = zjolt::ToJolt(shape);
  if (jolt_shape->GetSubType() != JPH::EShapeSubType::ConvexHull) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex hull");
  }
  const auto *hull = static_cast<const JPH::ConvexHullShape *>(jolt_shape);
  hull->DrawShrunkShape(ToBase(renderer), zjolt::ToJoltR(*center_of_mass_transform),
                        zjolt::ToJolt(scale));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

//===----------------------------------------------------------------------===//
// Record and playback
//===----------------------------------------------------------------------===//

ZJoltResult zjoltDebugRecorderCreate(ZJoltDebugRecorder **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  // Same singleton the note atop this file describes: DebugRendererRecorder
  // IS a DebugRenderer, and its base constructor asserts sInstance was NULL.
  if (JPH::DebugRenderer::sInstance != nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_ALREADY_INITIALIZED,
        "a ZJoltDebugRenderer or ZJoltDebugRecorder already exists; Jolt's "
        "DebugRenderer is a process-wide singleton and only one may be "
        "alive at a time");
  }
  ZJoltDebugRecorder *recorder = zjolt::New<ZJoltDebugRecorder>();
  if (recorder == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  *out = recorder;
  zjolt::HandleCreated();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

void zjoltDebugRecorderDestroy(ZJoltDebugRecorder *recorder) {
  if (recorder == nullptr) return;
#ifdef JPH_DEBUG_RENDERER
  zjolt::Delete(recorder);
  zjolt::HandleDestroyed();
#else
  (void)recorder;
#endif
}

ZJoltDebugRenderer *zjoltDebugRecorderAsRenderer(ZJoltDebugRecorder *recorder) {
#ifdef JPH_DEBUG_RENDERER
  if (recorder == nullptr) return nullptr;
  return FromBase(static_cast<JPH::DebugRenderer *>(&recorder->recorder));
#else
  (void)recorder;
  return nullptr;
#endif
}

ZJoltResult zjoltDebugRecorderEndFrame(ZJoltDebugRecorder *recorder) {
  ZJOLT_ENTER();
  if (!zjolt::Present(recorder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  recorder->recorder.EndFrame();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugRecorderGetData(const ZJoltDebugRecorder *recorder,
                                      void *buffer, size_t capacity,
                                      size_t *out_size) {
  ZJOLT_ENTER(out_size);
  if (!zjolt::Present(recorder, out_size)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const size_t payload_size = recorder->stream.data.size();
  const size_t header_size = kRecordingContainer.HeaderSize();
  *out_size = header_size + payload_size;
  if (buffer == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < *out_size) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  uint8_t *bytes = static_cast<uint8_t *>(buffer);
  if (payload_size > 0) {
    std::memcpy(bytes + header_size, recorder->stream.data.data(),
               payload_size);
  }
  zjolt::WriteContainerHeader(kRecordingContainer, bytes, payload_size,
                             /*extra=*/nullptr);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugPlaybackCreate(ZJoltDebugRenderer *renderer,
                                     ZJoltDebugPlayback **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(renderer, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  ZJoltDebugPlayback *playback = zjolt::New<ZJoltDebugPlayback>(*ToBase(renderer));
  if (playback == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  *out = playback;
  zjolt::HandleCreated();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

void zjoltDebugPlaybackDestroy(ZJoltDebugPlayback *playback) {
  if (playback == nullptr) return;
#ifdef JPH_DEBUG_RENDERER
  zjolt::Delete(playback);
  zjolt::HandleDestroyed();
#else
  (void)playback;
#endif
}

ZJoltResult zjoltDebugPlaybackParse(ZJoltDebugPlayback *playback,
                                    const void *data, size_t size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(playback)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  if (data == nullptr || size == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "no data to parse a debug recording from");
  }
  zjolt::ContainerContents contents;
  const ZJoltResult framed =
      zjolt::ReadContainer(kRecordingContainer, data, size, &contents);
  if (framed != ZJOLT_RESULT_OK) return framed;

  // DebugRendererPlayback::Parse has no success/failure signal of its own: it
  // reads commands until the stream it is given runs out, and running out is
  // how a well-formed recording always ends (there is no terminator command).
  // The container check above is what stands between Jolt and a buffer that
  // is not one of its own recordings — same trust boundary zjoltShapeRestore
  // documents, not a defence against a crafted payload with a valid checksum.
  zjolt::ConstStreamIn stream(contents.payload, contents.payload_size);
  playback->playback.Parse(stream);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugPlaybackGetNumFrames(const ZJoltDebugPlayback *playback,
                                           uint32_t *out_num_frames) {
  ZJOLT_ENTER(out_num_frames);
  if (!zjolt::Present(playback, out_num_frames))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  *out_num_frames = playback->playback.GetNumFrames();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltDebugPlaybackDrawFrame(ZJoltDebugPlayback *playback,
                                        uint32_t frame_number) {
  ZJOLT_ENTER();
  if (!zjolt::Present(playback)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  // DebugRendererPlayback::DrawFrame indexes its frame array with operator[],
  // which is JPH_ASSERT-only bounds checking — compiled out entirely in a
  // release build, so an out-of-range frame_number would read past the array
  // rather than fail.
  if (frame_number >= playback->playback.GetNumFrames()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "frame_number is not below GetNumFrames() for this playback");
  }
  playback->playback.DrawFrame(frame_number);
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawVertices(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawVertices(ToBase(renderer), transform);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawVertexVelocities(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawVertexVelocities(ToBase(renderer), transform);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawPredictedBounds(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawPredictedBounds(ToBase(renderer), transform);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawEdgeConstraints(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform,
                                    ZJoltSoftBodyConstraintColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::ESoftBodyConstraintColor jolt_color =
      ToJoltConstraintColor(zjolt::RawEnum(color));
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawEdgeConstraints(ToBase(renderer), transform, jolt_color);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawRods(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform,
                                    ZJoltSoftBodyConstraintColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::ESoftBodyConstraintColor jolt_color =
      ToJoltConstraintColor(zjolt::RawEnum(color));
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawRods(ToBase(renderer), transform, jolt_color);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawRodStates(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform,
                                    ZJoltSoftBodyConstraintColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::ESoftBodyConstraintColor jolt_color =
      ToJoltConstraintColor(zjolt::RawEnum(color));
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawRodStates(ToBase(renderer), transform, jolt_color);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawRodBendTwistConstraints(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform,
                                    ZJoltSoftBodyConstraintColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::ESoftBodyConstraintColor jolt_color =
      ToJoltConstraintColor(zjolt::RawEnum(color));
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawRodBendTwistConstraints(ToBase(renderer), transform, jolt_color);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawBendConstraints(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform,
                                    ZJoltSoftBodyConstraintColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::ESoftBodyConstraintColor jolt_color =
      ToJoltConstraintColor(zjolt::RawEnum(color));
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawBendConstraints(ToBase(renderer), transform, jolt_color);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawVolumeConstraints(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform,
                                    ZJoltSoftBodyConstraintColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::ESoftBodyConstraintColor jolt_color =
      ToJoltConstraintColor(zjolt::RawEnum(color));
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawVolumeConstraints(ToBase(renderer), transform, jolt_color);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawSkinConstraints(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform,
                                    ZJoltSoftBodyConstraintColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::ESoftBodyConstraintColor jolt_color =
      ToJoltConstraintColor(zjolt::RawEnum(color));
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawSkinConstraints(ToBase(renderer), transform, jolt_color);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltSoftBodyDrawLRAConstraints(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body,
                                    ZJoltDebugRenderer *renderer,
                                    const ZJoltRMat44 *com_transform,
                                    ZJoltSoftBodyConstraintColor color) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  const JPH::ESoftBodyConstraintColor jolt_color =
      ToJoltConstraintColor(zjolt::RawEnum(color));
  return WithSoftBodyDraw(
      system, body, com_transform,
      [&](const JPH::SoftBodyMotionProperties &motion, JPH::RMat44Arg transform) {
        motion.DrawLRAConstraints(ToBase(renderer), transform, jolt_color);
        return ZJOLT_RESULT_OK;
      });
#else
  (void)body;
  (void)com_transform;
  (void)color;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}
}  // extern "C"
