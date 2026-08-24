//===----------------------------------------------------------------------===//
// zjolt — debug-draw geometry, collected into arrays.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// Jolt's own DebugRenderer is an abstract C++ class a host would otherwise
// have to subclass across the language boundary. This does that subclassing
// once, inside the library, and exposes the result as three arrays — lines,
// triangles, text — read back through the same two-call protocol used
// everywhere else in this ABI. A host renders with whatever graphics API it
// already has; zjolt needs none.
//
// Everything here is declared whether or not this library was built with
// `-Ddebug_renderer=true`, and every entry point returns
// ZJOLT_RESULT_UNSUPPORTED when it was not. A header whose contents moved
// with a build flag could not be checked against the library that shipped it,
// which is what a fixed declared surface with a runtime answer avoids.
//
// Jolt's DebugRenderer is also a process-wide singleton: constructing one
// while another is still alive is refused here
// (ZJOLT_RESULT_ALREADY_INITIALIZED) rather than left to Jolt's own assertion,
// because several call sites deep inside Jolt (shape submerged-volume
// drawing, continuous-collision drawing, constraint-contact drawing) reach
// for that singleton directly instead of the renderer passed as a parameter —
// so a second live renderer would not just be redundant, it would silently
// steal those particular draws from the first one.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_DEBUG_H_
#define ZJOLT_DEBUG_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Opaque. Declared here rather than in zjolt_core.h because this is the only
/// file that needs it.
typedef struct ZJoltDebugRenderer ZJoltDebugRenderer;

//===----------------------------------------------------------------------===//
// Geometry
//===----------------------------------------------------------------------===//

typedef struct ZJoltDebugLine {
  ZJoltRVec3 from;
  ZJoltRVec3 to;
  ZJoltColor color;
} ZJoltDebugLine;

typedef struct ZJoltDebugTriangle {
  ZJoltRVec3 v1;
  ZJoltRVec3 v2;
  ZJoltRVec3 v3;
  ZJoltColor color;
  /// Whether this triangle should cast a shadow, for a renderer that does
  /// shadows. Jolt sets this to false for things like sensors and debug
  /// overlays that should never occlude light.
  bool cast_shadow;
} ZJoltDebugTriangle;

/// Bytes of `ZJoltDebugText::text`, including the terminator.
#define ZJOLT_DEBUG_TEXT_MAX_LENGTH 64

typedef struct ZJoltDebugText {
  ZJoltRVec3 position;
  ZJoltColor color;
  float height;
  /// Always NUL-terminated. Text longer than ZJOLT_DEBUG_TEXT_MAX_LENGTH - 1
  /// bytes is truncated to fit; `text_length` below still carries the
  /// original length, so truncation is detectable.
  uint8_t text[ZJOLT_DEBUG_TEXT_MAX_LENGTH];
  /// The untruncated length of the string Jolt drew. Longer than
  /// ZJOLT_DEBUG_TEXT_MAX_LENGTH - 1 means `text` was cut short.
  uint32_t text_length;
} ZJoltDebugText;

//===----------------------------------------------------------------------===//
// Renderer lifecycle
//===----------------------------------------------------------------------===//

/// Creates an empty sink for lines, triangles and text.
///
/// ZJOLT_RESULT_ALREADY_INITIALIZED while another ZJoltDebugRenderer is still
/// alive — see the note at the top of this file. Destroy it first.
ZJOLT_API ZJoltResult zjoltDebugRendererCreate(ZJoltDebugRenderer **out);

/// Frees a renderer and everything buffered in it.
ZJOLT_API ZJoltResult zjoltDebugRendererDestroy(ZJoltDebugRenderer *renderer);

/// Empties the buffered lines, triangles and text. Call once per frame before
/// drawing into a renderer again — the buffers only grow otherwise, since
/// nothing here knows when a frame ends except being told.
ZJOLT_API ZJoltResult zjoltDebugRendererClear(ZJoltDebugRenderer *renderer);

/// Records the camera position used to pick a level of detail for the solid
/// shapes zjoltPhysicsSystemDrawBodies emits. Optional; a renderer that never
/// calls this draws every shape at its highest level of detail.
ZJOLT_API ZJoltResult zjoltDebugRendererSetCameraPosition(
    ZJoltDebugRenderer *renderer, const ZJoltRVec3 *position);

//===----------------------------------------------------------------------===//
// Read-back
//
// The same two-call protocol as everywhere else in this ABI: the array
// pointer NULL reports the count in `*out_count` and writes nothing, and a
// `capacity` short of that reports ZJOLT_RESULT_BUFFER_TOO_SMALL with the true
// count still written. The count can grow between two calls if something
// draws into the renderer in between; it does not shrink on its own.
//===----------------------------------------------------------------------===//

ZJOLT_API ZJoltResult zjoltDebugRendererGetLines(const ZJoltDebugRenderer *renderer,
                                                  ZJoltDebugLine *lines,
                                                  uint32_t capacity,
                                                  uint32_t *out_count);

ZJOLT_API ZJoltResult zjoltDebugRendererGetTriangles(
    const ZJoltDebugRenderer *renderer, ZJoltDebugTriangle *triangles,
    uint32_t capacity, uint32_t *out_count);

ZJOLT_API ZJoltResult zjoltDebugRendererGetTexts(const ZJoltDebugRenderer *renderer,
                                                  ZJoltDebugText *texts,
                                                  uint32_t capacity,
                                                  uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Drawing the world
//
// These forward straight into Jolt's own PhysicsSystem::DrawBodies /
// DrawConstraints* — the settings and behaviour are Jolt's, not zjolt's.
//===----------------------------------------------------------------------===//

/// Coloring scheme for a drawn shape. Mirrors
/// JPH::BodyManager::EShapeColor.
typedef enum ZJoltShapeColor {
  /// Random color per body instance.
  ZJOLT_SHAPE_COLOR_INSTANCE_COLOR = 0,
  /// Convex = green, scaled = yellow, compound = orange, mesh = red.
  ZJOLT_SHAPE_COLOR_SHAPE_TYPE_COLOR = 1,
  /// Static = grey, keyframed = green, dynamic = random color per instance.
  ZJOLT_SHAPE_COLOR_MOTION_TYPE_COLOR = 2,
  /// Static = grey, keyframed = green, dynamic = yellow, sleeping = red.
  ZJOLT_SHAPE_COLOR_SLEEP_COLOR = 3,
  /// Static = grey, active = random color per island, sleeping = light grey.
  ZJOLT_SHAPE_COLOR_ISLAND_COLOR = 4,
  /// Color as defined by the body's PhysicsMaterial.
  ZJOLT_SHAPE_COLOR_MATERIAL_COLOR = 5,
} ZJoltShapeColor;

/// Subset of JPH::BodyManager::DrawSettings: what to draw and how to color
/// it. The soft-body-specific fields are omitted — this ABI has no soft-body
/// creation entry points yet, so there is nothing they could apply to.
typedef struct ZJoltDebugDrawBodiesSettings {
  /// Draw the GetSupport() function used for convex collision detection.
  bool draw_get_support_function;
  /// When drawing the support function, also draw which direction mapped to
  /// a specific support point.
  bool draw_support_direction;
  /// Draw the faces found colliding during collision detection.
  bool draw_get_supporting_face;
  /// Draw the shape of every body.
  bool draw_shape;
  /// When draw_shape is set, draw wireframe instead of solid.
  bool draw_shape_wireframe;
  ZJoltShapeColor shape_color;
  /// Draw a bounding box per body.
  bool draw_bounding_box;
  /// Draw the center of mass for each body.
  bool draw_center_of_mass_transform;
  /// Draw the world transform (can differ from the center of mass) for each
  /// body.
  bool draw_world_transform;
  /// Draw the velocity vector for each body.
  bool draw_velocity;
  /// Draw the mass and inertia (as the box equivalent) for each body.
  bool draw_mass_and_inertia;
  /// Draw stats about each body's sleeping algorithm.
  bool draw_sleep_stats;
} ZJoltDebugDrawBodiesSettings;

/// Fills `settings` with Jolt's own defaults, read out of a
/// default-constructed BodyManager::DrawSettings rather than transcribed, so
/// a re-vendor that retunes one of these carries it across correctly.
///
/// Unlike every zjoltDebugRenderer* entry point, this can be called without
/// ever having created a ZJoltDebugRenderer — it still reports
/// ZJOLT_RESULT_UNSUPPORTED when this library was not built with
/// -Ddebug_renderer=true, since Jolt's DrawSettings type does not exist in
/// that build to read defaults from.
ZJOLT_API ZJoltResult zjoltDebugDrawBodiesSettingsInit(
    ZJoltDebugDrawBodiesSettings *settings);

/// Draws every body's shape, plus whichever extras `settings` asks for, into
/// `renderer`.
ZJOLT_API ZJoltResult zjoltPhysicsSystemDrawBodies(
    ZJoltPhysicsSystem *system, const ZJoltDebugDrawBodiesSettings *settings,
    ZJoltDebugRenderer *renderer);

/// Draws every constraint currently in the system.
ZJOLT_API ZJoltResult zjoltPhysicsSystemDrawConstraints(
    ZJoltPhysicsSystem *system, ZJoltDebugRenderer *renderer);

/// Draws the swing/twist limits of every constraint that has them.
ZJOLT_API ZJoltResult zjoltPhysicsSystemDrawConstraintLimits(
    ZJoltPhysicsSystem *system, ZJoltDebugRenderer *renderer);

/// Draws the reference frame of every constraint.
ZJOLT_API ZJoltResult zjoltPhysicsSystemDrawConstraintReferenceFrame(
    ZJoltPhysicsSystem *system, ZJoltDebugRenderer *renderer);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_DEBUG_H_
