//===----------------------------------------------------------------------===//
// zjolt — debug-draw geometry, collected into arrays.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part.
//
// Wraps Jolt's DebugRenderer as three read-back arrays (lines,
// triangles, text). Every entry point is declared regardless of build,
// returning ZJOLT_RESULT_UNSUPPORTED without -Ddebug_renderer=true.
//
// Jolt's DebugRenderer is a process-wide singleton: constructing a
// second one (ZJoltDebugRenderer or ZJoltDebugRecorder) while one is
// alive is ZJOLT_RESULT_ALREADY_INITIALIZED — several call sites deep
// in Jolt reach for the singleton directly, not the parameter passed in.
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

/// Frees a renderer and everything buffered in it. NULL is a no-op, as for
/// every other zjolt*Destroy.
ZJOLT_API void zjoltDebugRendererDestroy(ZJoltDebugRenderer *renderer);

/// Empties the buffered lines, triangles and text. Call once per frame before
/// drawing into a renderer again — the buffers only grow otherwise, since
/// nothing here knows when a frame ends except being told.
ZJOLT_API ZJoltResult zjoltDebugRendererClear(ZJoltDebugRenderer *renderer);

/// Records the camera position for choosing a level of detail on the
/// solid shapes zjoltPhysicsSystemDrawBodies emits. Optional; a renderer
/// that never calls this draws every shape at its highest level of detail.
ZJOLT_API ZJoltResult zjoltDebugRendererSetCameraPosition(
    ZJoltDebugRenderer *renderer, const ZJoltRVec3 *position);

//===----------------------------------------------------------------------===//
// Read-back
//
// Two-call protocol: array NULL reports the count in `*out_count`; a short `capacity` is
// ZJOLT_RESULT_BUFFER_TOO_SMALL with the true count written. Can grow between calls.
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
// Host-issued primitives
//
// Direct forwards to a DebugRenderer method, writing into `renderer`'s buffers alongside whatever Jolt drew.
// Also accepts the recorder view zjoltDebugRecorderAsRenderer returns, writing into a recorded stream instead.
//===----------------------------------------------------------------------===//

/// Mirrors JPH::DebugRenderer::ECastShadow.
typedef enum ZJoltCastShadow {
  ZJOLT_CAST_SHADOW_ON = 0,
  ZJOLT_CAST_SHADOW_OFF = 1,
} ZJoltCastShadow;

/// Mirrors JPH::DebugRenderer::EDrawMode.
typedef enum ZJoltDrawMode {
  ZJOLT_DRAW_MODE_SOLID = 0,
  ZJOLT_DRAW_MODE_WIREFRAME = 1,
} ZJoltDrawMode;

/// The base primitive. DrawTriangle below is the other one.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawLine(ZJoltDebugRenderer *renderer,
                                                  const ZJoltRVec3 *from,
                                                  const ZJoltRVec3 *to,
                                                  ZJoltColor color);

/// The second base primitive. Back-face culled, like Jolt's own.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawTriangle(
    ZJoltDebugRenderer *renderer, const ZJoltRVec3 *v1, const ZJoltRVec3 *v2,
    const ZJoltRVec3 *v3, ZJoltColor color, ZJoltCastShadow cast_shadow);

/// A label at a world position. `text` need not be NUL-terminated;
/// `text_length` is authoritative. Text longer than
/// ZJOLT_DEBUG_TEXT_MAX_LENGTH - 1 bytes still draws in full here — the
/// truncation zjoltDebugRendererGetTexts documents happens only on the way
/// back out of a ZJoltDebugRenderer sink, not on the way in.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawText3D(ZJoltDebugRenderer *renderer,
                                                    const ZJoltRVec3 *position,
                                                    const char *text,
                                                    uint32_t text_length,
                                                    ZJoltColor color,
                                                    float height);

/// An axis-aligned box already in world space — the shape a query region or
/// a bounding volume already is. For a box a host wants oriented instead,
/// transform the corners and use zjoltDebugRendererDrawWirePolygon or build
/// the twelve edges from zjoltDebugRendererDrawLine directly.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawWireBox(ZJoltDebugRenderer *renderer,
                                                     const ZJoltAABox *box,
                                                     ZJoltColor color);

ZJOLT_API ZJoltResult zjoltDebugRendererDrawBox(ZJoltDebugRenderer *renderer,
                                                 const ZJoltAABox *box,
                                                 ZJoltColor color,
                                                 ZJoltCastShadow cast_shadow,
                                                 ZJoltDrawMode draw_mode);

ZJOLT_API ZJoltResult zjoltDebugRendererDrawSphere(
    ZJoltDebugRenderer *renderer, const ZJoltRVec3 *center, float radius,
    ZJoltColor color, ZJoltCastShadow cast_shadow, ZJoltDrawMode draw_mode);

ZJOLT_API ZJoltResult zjoltDebugRendererDrawWireSphere(
    ZJoltDebugRenderer *renderer, const ZJoltRVec3 *center, float radius,
    ZJoltColor color, int32_t level);

/// A sphere placed by a full matrix rather than a center and radius, so it
/// may come out non-uniformly scaled. `matrix`'s scale IS the sphere's shape;
/// there is no separate radius.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawUnitSphere(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *matrix, ZJoltColor color,
    ZJoltCastShadow cast_shadow, ZJoltDrawMode draw_mode);

ZJOLT_API ZJoltResult zjoltDebugRendererDrawWireUnitSphere(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *matrix, ZJoltColor color,
    int32_t level);

/// A capsule with one half-sphere at local (0, -half_height, 0), the other at
/// local (0, half_height, 0), transformed into world space by `matrix`.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawCapsule(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *matrix,
    float half_height_of_cylinder, float radius, ZJoltColor color,
    ZJoltCastShadow cast_shadow, ZJoltDrawMode draw_mode);

/// A cylinder with top at local (0, half_height, 0) and bottom at local
/// (0, -half_height, 0), transformed into world space by `matrix`.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawCylinder(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *matrix, float half_height,
    float radius, ZJoltColor color, ZJoltCastShadow cast_shadow,
    ZJoltDrawMode draw_mode);

ZJOLT_API ZJoltResult zjoltDebugRendererDrawTaperedCylinder(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *matrix, float top,
    float bottom, float top_radius, float bottom_radius, ZJoltColor color,
    ZJoltCastShadow cast_shadow, ZJoltDrawMode draw_mode);

/// A wedge of a circle in the plane through `center` with normal `normal`,
/// between `min_angle` and `max_angle` radians measured from `axis`.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawPie(
    ZJoltDebugRenderer *renderer, const ZJoltRVec3 *center, float radius,
    const ZJoltVec3 *normal, const ZJoltVec3 *axis, float min_angle,
    float max_angle, ZJoltColor color, ZJoltCastShadow cast_shadow,
    ZJoltDrawMode draw_mode);

ZJOLT_API ZJoltResult zjoltDebugRendererDrawArrow(ZJoltDebugRenderer *renderer,
                                                   const ZJoltRVec3 *from,
                                                   const ZJoltRVec3 *to,
                                                   ZJoltColor color,
                                                   float size);

/// Three short lines through `position` along the coordinate axes, `size`
/// long each way.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawMarker(ZJoltDebugRenderer *renderer,
                                                    const ZJoltRVec3 *position,
                                                    ZJoltColor color,
                                                    float size);

/// Three arrows out of `transform`'s origin along its own axes: x red, y
/// green, z blue, `size` long.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawCoordinateSystem(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *transform, float size);

/// A `size`-wide square through `point`, normal to `normal`.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawPlane(ZJoltDebugRenderer *renderer,
                                                   const ZJoltRVec3 *point,
                                                   const ZJoltVec3 *normal,
                                                   ZJoltColor color,
                                                   float size);

ZJOLT_API ZJoltResult zjoltDebugRendererDrawWireTriangle(
    ZJoltDebugRenderer *renderer, const ZJoltRVec3 *v1, const ZJoltRVec3 *v2,
    const ZJoltRVec3 *v3, ZJoltColor color);

/// `vertices` are in `transform`'s local space, drawn as arrows tip-to-tail
/// around the polygon they form — `arrow_size` 0 draws plain edges. Mirrors
/// the DebugRenderer::DrawWirePolygon template, which this ABI cannot
/// instantiate directly since C has no templates.
ZJOLT_API ZJoltResult zjoltDebugRendererDrawWirePolygon(
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *transform,
    const ZJoltVec3 *vertices, uint32_t vertex_count, ZJoltColor color,
    float arrow_size);

//===----------------------------------------------------------------------===//
// Batched geometry — the fast path
//
// Jolt tessellates a shape once into a Batch it holds for the shape's
// lifetime, replayed every frame through DrawGeometry instead of walking
// triangles through zjoltDebugRendererDrawTriangle each draw. Unset,
// zjoltDebugRendererSetBatchCallbacks leaves batches flattened into the
// immediate-mode stream zjoltDebugRendererGetTriangles reads back.
// ZJoltDebugGeometry.selected_lod mirrors Geometry::GetLOD: the first
// `lods` entry, nearest-detail first, whose squared, lod_scale_sq-scaled
// distance is >= the squared distance from the last
// zjoltDebugRendererSetCameraPosition to world_space_bounds; else the
// last entry, or 0 with no camera position ever set.
//===----------------------------------------------------------------------===//

/// One vertex of a batched triangle. Mirrors JPH::DebugRenderer::Vertex.
/// `position`/`normal` are in the batch's own model space, not world space —
/// a zjoltDebugBatchCallbacks draw_geometry call's model_matrix places it.
typedef struct ZJoltDebugVertex {
  ZJoltVec3 position;
  ZJoltVec3 normal;
  float u, v;
  ZJoltColor color;
} ZJoltDebugVertex;

/// Mirrors JPH::DebugRenderer::ECullMode.
typedef enum ZJoltCullMode {
  ZJOLT_CULL_MODE_BACK_FACE = 0,
  ZJOLT_CULL_MODE_FRONT_FACE = 1,
  ZJOLT_CULL_MODE_OFF = 2,
} ZJoltCullMode;

/// One level of detail. `batch` is a create-callback's return value,
/// unchanged, or NULL for a LOD Jolt built before any callbacks were
/// registered (its shared box/sphere/capsule/cylinder/cone primitives,
/// always built at zjoltDebugRendererCreate time). `distance`: see the
/// section comment above.
typedef struct ZJoltDebugLOD {
  void *batch;
  float distance;
} ZJoltDebugLOD;

/// A shape's whole level-of-detail chain, as draw_geometry receives it.
/// `lods` is valid only for the call. `selected_lod`: see the section
/// comment above. `bounds` is the model-space box enclosing every LOD.
typedef struct ZJoltDebugGeometry {
  const ZJoltDebugLOD *lods;
  uint32_t lod_count;
  uint32_t selected_lod;
  ZJoltAABox bounds;
} ZJoltDebugGeometry;

/// Delivers CreateTriangleBatch/DrawGeometry to a host, set with
/// zjoltDebugRendererSetBatchCallbacks (a NULL field falls back to the
/// immediate-mode path for that one call). Ownership: a returned batch
/// handle is a Ref<> this library now holds; destroy_batch runs once, when
/// the last reference drops, and only then may the host free it.
typedef struct ZJoltDebugBatchCallbacks {
  /// Builds a batch from `vertex_count` vertices forming unindexed
  /// triangles: `vertex_count` is always a multiple of 3, and every
  /// consecutive triple is one triangle. Returns NULL on failure, which
  /// falls back to the immediate-mode path for this batch only.
  void *(*create_triangle_batch)(void *user, const ZJoltDebugVertex *vertices,
                                  uint32_t vertex_count);
  /// Builds a batch from an indexed vertex list: `indices[0..index_count)`
  /// group into triangles of 3. Returns NULL on failure, same fallback as
  /// create_triangle_batch.
  void *(*create_triangle_batch_indexed)(void *user,
                                          const ZJoltDebugVertex *vertices,
                                          uint32_t vertex_count,
                                          const uint32_t *indices,
                                          uint32_t index_count);
  /// Called exactly once per batch, when this library's last reference to it
  /// drops. `batch` is a value a create function above returned; never NULL.
  void (*destroy_batch)(void *user, void *batch);
  /// Draws `geometry`'s selected LOD, transformed by `model_matrix` into
  /// `world_space_bounds`, tinted by `model_color`. Required; a batch
  /// callbacks table with this NULL is refused with
  /// ZJOLT_RESULT_INVALID_ARGUMENT.
  void (*draw_geometry)(void *user, const ZJoltRMat44 *model_matrix,
                        const ZJoltAABox *world_space_bounds, float lod_scale_sq,
                        ZJoltColor model_color, const ZJoltDebugGeometry *geometry,
                        ZJoltCullMode cull_mode, ZJoltCastShadow cast_shadow,
                        ZJoltDrawMode draw_mode);
  void *user;
} ZJoltDebugBatchCallbacks;

/// Routes CreateTriangleBatch/DrawGeometry to `callbacks` from now on,
/// replacing whichever were set before; NULL restores the immediate-mode
/// fallback for both. `callbacks` is copied, but its function pointers and
/// `user` must stay valid for every draw through `renderer` until this
/// runs again or `renderer` is destroyed. ZJOLT_RESULT_INVALID_ARGUMENT if
/// `callbacks` is non-NULL with a NULL draw_geometry.
ZJOLT_API ZJoltResult zjoltDebugRendererSetBatchCallbacks(
    ZJoltDebugRenderer *renderer, const ZJoltDebugBatchCallbacks *callbacks);

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
/// default-constructed BodyManager::DrawSettings rather than transcribed.
/// Unlike every zjoltDebugRenderer* entry point, callable without ever
/// creating a ZJoltDebugRenderer — still ZJOLT_RESULT_UNSUPPORTED
/// without -Ddebug_renderer=true, since Jolt's DrawSettings type does
/// not exist to read defaults from in that build.
ZJOLT_API ZJoltResult zjoltDebugDrawBodiesSettingsInit(
    ZJoltDebugDrawBodiesSettings *settings);

/// Excludes chosen bodies from a zjoltPhysicsSystemDrawBodies pass. Mirrors
/// JPH::BodyDrawFilter::ShouldDraw, which Jolt calls with the body already
/// locked — `should_draw` may read anything on `body` a locked body permits.
/// NULL `should_draw` draws every body, matching a NULL filter passed to
/// Jolt's own DrawBodies.
typedef struct ZJoltBodyDrawFilter {
  bool (*should_draw)(void *user, ZJoltBodyId body);
  void *user;
} ZJoltBodyDrawFilter;

/// Draws every body's shape, plus whichever extras `settings` asks for, into
/// `renderer`. `filter` may be NULL to draw every body.
ZJOLT_API ZJoltResult zjoltPhysicsSystemDrawBodies(
    ZJoltPhysicsSystem *system, const ZJoltDebugDrawBodiesSettings *settings,
    ZJoltDebugRenderer *renderer, const ZJoltBodyDrawFilter *filter);

/// Draws every constraint currently in the system.
ZJOLT_API ZJoltResult zjoltPhysicsSystemDrawConstraints(
    ZJoltPhysicsSystem *system, ZJoltDebugRenderer *renderer);

/// Draws the swing/twist limits of every constraint that has them.
ZJOLT_API ZJoltResult zjoltPhysicsSystemDrawConstraintLimits(
    ZJoltPhysicsSystem *system, ZJoltDebugRenderer *renderer);

/// Draws the reference frame of every constraint.
ZJOLT_API ZJoltResult zjoltPhysicsSystemDrawConstraintReferenceFrame(
    ZJoltPhysicsSystem *system, ZJoltDebugRenderer *renderer);

//===----------------------------------------------------------------------===//
// A shape's own debugging helper
//===----------------------------------------------------------------------===//

/// ConvexHullShape::DrawShrunkShape: the hull shrunk inward by its
/// convex radius — the surface Jolt's collision detection actually
/// uses. The plain shape draw shows the hull before shrinking.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT if `shape`'s subtype is not ConvexHull.
ZJOLT_API ZJoltResult zjoltShapeDrawShrunkShape(
    const ZJoltShape *shape, const ZJoltRMat44 *center_of_mass_transform,
    ZJoltVec3 scale, ZJoltDebugRenderer *renderer);

//===----------------------------------------------------------------------===//
// Record and playback
//
// A ZJoltDebugRecorder serialises every draw call into a stream instead of buffering the current frame, so a
// session replays after the fact. zjoltDebugRecorderGetData uses zjoltShapeSave's container framing; not adversarial-safe.
//===----------------------------------------------------------------------===//

typedef struct ZJoltDebugRecorder ZJoltDebugRecorder;
typedef struct ZJoltDebugPlayback ZJoltDebugPlayback;

/// A recorder is also a process-wide DebugRenderer underneath (see the note
/// above ZJoltDebugRenderer), so creating one while any ZJoltDebugRenderer or
/// ZJoltDebugRecorder is alive is ZJOLT_RESULT_ALREADY_INITIALIZED.
ZJOLT_API ZJoltResult zjoltDebugRecorderCreate(ZJoltDebugRecorder **out);

/// Frees a recorder and discards whatever it had buffered. NULL is a no-op.
ZJOLT_API void zjoltDebugRecorderDestroy(ZJoltDebugRecorder *recorder);

/// A recorder IS a JPH::DebugRenderer underneath — every entry point
/// above that takes a ZJoltDebugRenderer works on it through this
/// borrowed view, writing into the recorded stream instead. Borrowed:
/// never zjoltDebugRendererDestroy/Clear/GetLines/GetTriangles/GetTexts
/// it — those assume the other kind. Valid only as long as `recorder` is.
ZJOLT_API ZJoltDebugRenderer *zjoltDebugRecorderAsRenderer(
    ZJoltDebugRecorder *recorder);

/// Flushes everything drawn into this recorder's view since the last
/// zjoltDebugRecorderEndFrame (or since creation) into the stream as one
/// frame. Nothing drawn is visible to zjoltDebugPlaybackParse until this
/// runs — mirrors DebugRendererRecorder::EndFrame exactly: a recording with
/// no EndFrame calls holds zero frames no matter how much was drawn.
ZJOLT_API ZJoltResult zjoltDebugRecorderEndFrame(ZJoltDebugRecorder *recorder);

/// The recorded stream so far, framed as described above. Two-call protocol:
/// buffer NULL reports the size in `*out_size`; a `capacity` short of that is
/// ZJOLT_RESULT_BUFFER_TOO_SMALL with the true size still written.
ZJOLT_API ZJoltResult zjoltDebugRecorderGetData(const ZJoltDebugRecorder *recorder,
                                                void *buffer, size_t capacity,
                                                size_t *out_size);

/// Parses zjoltDebugRecorderGetData output and replays it, frame by frame,
/// into `renderer` — which, unlike the ZJoltDebugRenderer a recorder view
/// wraps, is meant to be an ordinary read-back renderer here: this is how a
/// recorded session gets back onto screen. `renderer` must outlive the
/// playback.
ZJOLT_API ZJoltResult zjoltDebugPlaybackCreate(ZJoltDebugRenderer *renderer,
                                               ZJoltDebugPlayback **out);

/// Frees a playback. Does not touch the renderer it draws into. NULL is a
/// no-op.
ZJOLT_API void zjoltDebugPlaybackDestroy(ZJoltDebugPlayback *playback);

/// Adds the frames in `data` (zjoltDebugRecorderGetData output) to this
/// playback's timeline. A wrong tag, a wrong build, a length that disagrees
/// with the buffer, or a failed checksum is ZJOLT_RESULT_BAD_FORMAT. Calling
/// this more than once appends further frames rather than replacing the
/// first batch, matching DebugRendererPlayback::Parse, which never clears
/// what it already parsed.
ZJOLT_API ZJoltResult zjoltDebugPlaybackParse(ZJoltDebugPlayback *playback,
                                              const void *data, size_t size);

/// How many frames zjoltDebugPlaybackParse has added so far.
ZJOLT_API ZJoltResult zjoltDebugPlaybackGetNumFrames(
    const ZJoltDebugPlayback *playback, uint32_t *out_num_frames);

/// Replays frame `frame_number` into the renderer this playback was created
/// with. ZJOLT_RESULT_INVALID_ARGUMENT if `frame_number` is not below
/// zjoltDebugPlaybackGetNumFrames — Jolt's own DrawFrame indexes its frame
/// array with no bounds check at all.
ZJOLT_API ZJoltResult zjoltDebugPlaybackDrawFrame(ZJoltDebugPlayback *playback,
                                                  uint32_t frame_number);

//===----------------------------------------------------------------------===//
// Soft bodies
//
// Separate from zjoltPhysicsSystemDrawBodies: Jolt draws a soft body's state through SoftBodyMotionProperties,
// not the shape. ZJOLT_RESULT_UNSUPPORTED without -Ddebug_renderer=true, INVALID_ARGUMENT for a non-soft body.
//===----------------------------------------------------------------------===//

/// How constraints are coloured. Mirrors JPH::ESoftBodyConstraintColor.
typedef enum ZJoltSoftBodyConstraintColor {
  /// A colour per constraint type.
  ZJOLT_SOFT_BODY_CONSTRAINT_COLOR_TYPE = 0,
  /// A colour per parallel group; the non-parallel group is red.
  ZJOLT_SOFT_BODY_CONSTRAINT_COLOR_GROUP = 1,
  /// As GROUP, with a gradient showing solve order inside each group.
  ZJOLT_SOFT_BODY_CONSTRAINT_COLOR_ORDER = 2,
} ZJoltSoftBodyConstraintColor;

/// Draws the simulated vertices. `com_transform` may be NULL for the body’s own
/// centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawVertices(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform);

/// Draws a velocity line per vertex. `com_transform` may be NULL for the body’s
/// own centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawVertexVelocities(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform);

/// Draws the bounds the solver predicted for this step. `com_transform` may be
/// NULL for the body’s own centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawPredictedBounds(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform);

/// Draws the edge constraints. `com_transform` may be NULL for the body’s own
/// centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawEdgeConstraints(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform,
    ZJoltSoftBodyConstraintColor color);

/// Draws the stretch-shear rods. `com_transform` may be NULL for the body’s own
/// centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawRods(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform,
    ZJoltSoftBodyConstraintColor color);

/// Draws each rod’s orientation frame. `com_transform` may be NULL for the
/// body’s own centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawRodStates(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform,
    ZJoltSoftBodyConstraintColor color);

/// Draws the rod bend-twist constraints. `com_transform` may be NULL for the
/// body’s own centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawRodBendTwistConstraints(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform,
    ZJoltSoftBodyConstraintColor color);

/// Draws the dihedral bend constraints. `com_transform` may be NULL for the
/// body’s own centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawBendConstraints(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform,
    ZJoltSoftBodyConstraintColor color);

/// Draws the volume constraints. `com_transform` may be NULL for the body’s own
/// centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawVolumeConstraints(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform,
    ZJoltSoftBodyConstraintColor color);

/// Draws the skin constraints. `com_transform` may be NULL for the body’s own
/// centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawSkinConstraints(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform,
    ZJoltSoftBodyConstraintColor color);

/// Draws the long-range-attachment constraints. `com_transform` may be NULL for
/// the body’s own centre-of-mass transform.
ZJOLT_API ZJoltResult zjoltSoftBodyDrawLRAConstraints(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltDebugRenderer *renderer, const ZJoltRMat44 *com_transform,
    ZJoltSoftBodyConstraintColor color);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_DEBUG_H_
