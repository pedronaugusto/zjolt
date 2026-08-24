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

#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Constraints/ConstraintManager.h>
#include <Jolt/Renderer/DebugRendererSimple.h>

#include <cstring>

namespace {

//===----------------------------------------------------------------------===//
// The sink
//
// DebugRendererSimple already turns Jolt's batched-geometry machinery
// (CreateTriangleBatch, DrawGeometry, level-of-detail selection) into calls to
// DrawLine / DrawTriangle / DrawText3D — see its class comment. Overriding
// those three is the entire subclass; nothing here mirrors Jolt's vtable.
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

  JPH::Array<RecordedLine> lines;
  JPH::Array<RecordedTriangle> triangles;
  JPH::Array<RecordedText> texts;
};

inline ZJoltDebugRendererImpl *ToImpl(ZJoltDebugRenderer *renderer) {
  return reinterpret_cast<ZJoltDebugRendererImpl *>(renderer);
}

inline const ZJoltDebugRendererImpl *ToImpl(const ZJoltDebugRenderer *renderer) {
  return reinterpret_cast<const ZJoltDebugRendererImpl *>(renderer);
}

inline ZJoltDebugRenderer *FromImpl(ZJoltDebugRendererImpl *impl) {
  return reinterpret_cast<ZJoltDebugRenderer *>(impl);
}

ZJoltShapeColor ToCShapeColor(JPH::BodyManager::EShapeColor color) {
  return static_cast<ZJoltShapeColor>(color);
}

JPH::BodyManager::EShapeColor ToJoltShapeColor(ZJoltShapeColor color) {
  return static_cast<JPH::BodyManager::EShapeColor>(color);
}

}  // namespace

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

ZJoltResult zjoltDebugRendererDestroy(ZJoltDebugRenderer *renderer) {
  ZJOLT_ENTER();
  if (!zjolt::Present(renderer)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  zjolt::Delete(ToImpl(renderer));
  zjolt::HandleDestroyed();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
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
  ToImpl(renderer)->SetCameraPos(zjolt::ToJoltR(*position));
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
    ZJoltDebugRenderer *renderer) {
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
  draw_settings.mDrawShapeColor = ToJoltShapeColor(settings->shape_color);
  draw_settings.mDrawBoundingBox = settings->draw_bounding_box;
  draw_settings.mDrawCenterOfMassTransform =
      settings->draw_center_of_mass_transform;
  draw_settings.mDrawWorldTransform = settings->draw_world_transform;
  draw_settings.mDrawVelocity = settings->draw_velocity;
  draw_settings.mDrawMassAndInertia = settings->draw_mass_and_inertia;
  draw_settings.mDrawSleepStats = settings->draw_sleep_stats;

  system->system.DrawBodies(draw_settings, ToImpl(renderer));
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
  system->system.DrawConstraints(ToImpl(renderer));
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
  system->system.DrawConstraintLimits(ToImpl(renderer));
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
  system->system.DrawConstraintReferenceFrame(ToImpl(renderer));
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

}  // extern "C"
