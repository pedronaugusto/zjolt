//! ZJolt C declarations for debug geometry, as arrays for a host to draw.
//!
//! Mirrors `ffi/zjolt_debug.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const AABox = core.AABox;
pub const BodyId = core.BodyId;
pub const Color = core.Color;
pub const Mat44 = core.Mat44;
pub const PhysicsSystem = core.PhysicsSystem;
pub const RMat44 = core.RMat44;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Shape = core.Shape;
pub const Vec3 = core.Vec3;

/// Bytes of `DebugText.text`, including the terminator.
pub const debug_text_max_length: u32 = 64;

pub const DebugRenderer = opaque {};

pub const DebugLine = extern struct {
    from: RVec3,
    to: RVec3,
    color: Color,
};

pub const DebugTriangle = extern struct {
    v1: RVec3,
    v2: RVec3,
    v3: RVec3,
    color: Color,
    cast_shadow: bool,
};

pub const DebugText = extern struct {
    position: RVec3,
    color: Color,
    height: f32,
    text: [debug_text_max_length]u8,
    text_length: u32,
};

/// Mirrors JPH::BodyManager::EShapeColor.
pub const ShapeColor = enum(c_int) {
    instance_color = 0,
    shape_type_color = 1,
    motion_type_color = 2,
    sleep_color = 3,
    island_color = 4,
    material_color = 5,
};

/// Subset of JPH::BodyManager::DrawSettings — see ffi/zjolt_debug.h for what
/// was left out and why.
pub const DebugDrawBodiesSettings = extern struct {
    draw_get_support_function: bool,
    draw_support_direction: bool,
    draw_get_supporting_face: bool,
    draw_shape: bool,
    draw_shape_wireframe: bool,
    shape_color: ShapeColor,
    draw_bounding_box: bool,
    draw_center_of_mass_transform: bool,
    draw_world_transform: bool,
    draw_velocity: bool,
    draw_mass_and_inertia: bool,
    draw_sleep_stats: bool,
};

pub extern fn zjoltDebugRendererCreate(out: **DebugRenderer) Result;

pub extern fn zjoltDebugRendererDestroy(renderer: ?*DebugRenderer) void;

pub extern fn zjoltDebugRendererClear(renderer: *DebugRenderer) Result;

pub extern fn zjoltDebugRendererSetCameraPosition(renderer: *DebugRenderer, position: *const RVec3) Result;

//=============================================================================
// Batched geometry — the fast path
//=============================================================================

/// One vertex of a batched triangle. Mirrors JPH::DebugRenderer::Vertex.
pub const DebugVertex = extern struct {
    position: Vec3,
    normal: Vec3,
    u: f32,
    v: f32,
    color: Color,
};

/// Mirrors JPH::DebugRenderer::ECullMode.
pub const CullMode = enum(c_int) {
    back_face = 0,
    front_face = 1,
    off = 2,
};

pub const DebugLOD = extern struct {
    batch: ?*anyopaque,
    distance: f32,
};

pub const DebugGeometry = extern struct {
    lods: [*]const DebugLOD,
    lod_count: u32,
    selected_lod: u32,
    bounds: AABox,
};

/// See ffi/zjolt_debug.h for the ownership contract: a batch handle a create
/// function returns is one this library now holds a reference to, and
/// destroy_batch runs exactly once, when the last reference drops.
pub const DebugBatchCallbacks = extern struct {
    create_triangle_batch: ?*const fn (user: ?*anyopaque, vertices: ?[*]const DebugVertex, vertex_count: u32) callconv(.c) ?*anyopaque = null,
    create_triangle_batch_indexed: ?*const fn (user: ?*anyopaque, vertices: ?[*]const DebugVertex, vertex_count: u32, indices: ?[*]const u32, index_count: u32) callconv(.c) ?*anyopaque = null,
    destroy_batch: ?*const fn (user: ?*anyopaque, batch: ?*anyopaque) callconv(.c) void = null,
    draw_geometry: ?*const fn (
        user: ?*anyopaque,
        model_matrix: *const RMat44,
        world_space_bounds: *const AABox,
        lod_scale_sq: f32,
        model_color: Color,
        geometry: *const DebugGeometry,
        cull_mode: CullMode,
        cast_shadow: CastShadow,
        draw_mode: DrawMode,
    ) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub extern fn zjoltDebugRendererSetBatchCallbacks(renderer: *DebugRenderer, callbacks: ?*const DebugBatchCallbacks) Result;

pub extern fn zjoltDebugRendererGetLines(renderer: *const DebugRenderer, lines: ?[*]DebugLine, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltDebugRendererGetTriangles(renderer: *const DebugRenderer, triangles: ?[*]DebugTriangle, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltDebugRendererGetTexts(renderer: *const DebugRenderer, texts: ?[*]DebugText, capacity: u32, out_count: *u32) Result;

//=============================================================================
// Host-issued primitives
//=============================================================================

/// Mirrors JPH::DebugRenderer::ECastShadow.
pub const CastShadow = enum(c_int) {
    on = 0,
    off = 1,
};

/// Mirrors JPH::DebugRenderer::EDrawMode.
pub const DrawMode = enum(c_int) {
    solid = 0,
    wireframe = 1,
};

pub extern fn zjoltDebugRendererDrawLine(renderer: *DebugRenderer, from: *const RVec3, to: *const RVec3, color: Color) Result;

pub extern fn zjoltDebugRendererDrawTriangle(renderer: *DebugRenderer, v1: *const RVec3, v2: *const RVec3, v3: *const RVec3, color: Color, cast_shadow: CastShadow) Result;

pub extern fn zjoltDebugRendererDrawText3D(renderer: *DebugRenderer, position: *const RVec3, text: ?[*]const u8, text_length: u32, color: Color, height: f32) Result;

pub extern fn zjoltDebugRendererDrawWireBox(renderer: *DebugRenderer, box: *const AABox, color: Color) Result;

pub extern fn zjoltDebugRendererDrawBox(renderer: *DebugRenderer, box: *const AABox, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) Result;

pub extern fn zjoltDebugRendererDrawSphere(renderer: *DebugRenderer, center: *const RVec3, radius: f32, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) Result;

pub extern fn zjoltDebugRendererDrawWireSphere(renderer: *DebugRenderer, center: *const RVec3, radius: f32, color: Color, level: i32) Result;

pub extern fn zjoltDebugRendererDrawUnitSphere(renderer: *DebugRenderer, matrix: *const RMat44, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) Result;

pub extern fn zjoltDebugRendererDrawWireUnitSphere(renderer: *DebugRenderer, matrix: *const RMat44, color: Color, level: i32) Result;

pub extern fn zjoltDebugRendererDrawCapsule(renderer: *DebugRenderer, matrix: *const RMat44, half_height_of_cylinder: f32, radius: f32, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) Result;

pub extern fn zjoltDebugRendererDrawCylinder(renderer: *DebugRenderer, matrix: *const RMat44, half_height: f32, radius: f32, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) Result;

pub extern fn zjoltDebugRendererDrawTaperedCylinder(renderer: *DebugRenderer, matrix: *const RMat44, top: f32, bottom: f32, top_radius: f32, bottom_radius: f32, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) Result;

pub extern fn zjoltDebugRendererDrawPie(renderer: *DebugRenderer, center: *const RVec3, radius: f32, normal: *const Vec3, axis: *const Vec3, min_angle: f32, max_angle: f32, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) Result;

pub extern fn zjoltDebugRendererDrawArrow(renderer: *DebugRenderer, from: *const RVec3, to: *const RVec3, color: Color, size: f32) Result;

pub extern fn zjoltDebugRendererDrawMarker(renderer: *DebugRenderer, position: *const RVec3, color: Color, size: f32) Result;

pub extern fn zjoltDebugRendererDrawCoordinateSystem(renderer: *DebugRenderer, transform: *const RMat44, size: f32) Result;

pub extern fn zjoltDebugRendererDrawPlane(renderer: *DebugRenderer, point: *const RVec3, normal: *const Vec3, color: Color, size: f32) Result;

pub extern fn zjoltDebugRendererDrawWireTriangle(renderer: *DebugRenderer, v1: *const RVec3, v2: *const RVec3, v3: *const RVec3, color: Color) Result;

pub extern fn zjoltDebugRendererDrawWirePolygon(renderer: *DebugRenderer, transform: *const RMat44, vertices: [*]const Vec3, vertex_count: u32, color: Color, arrow_size: f32) Result;

//=============================================================================
// Drawing the world
//=============================================================================

pub extern fn zjoltDebugDrawBodiesSettingsInit(settings: *DebugDrawBodiesSettings) Result;

/// A filter a host installs to exclude specific bodies from a debug-draw
/// pass. NULL `should_draw` draws every body.
pub const BodyDrawFilter = extern struct {
    should_draw: ?*const fn (user: ?*anyopaque, body: BodyId) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub extern fn zjoltPhysicsSystemDrawBodies(system: *PhysicsSystem, settings: *const DebugDrawBodiesSettings, renderer: *DebugRenderer, filter: ?*const BodyDrawFilter) Result;

pub extern fn zjoltPhysicsSystemDrawConstraints(system: *PhysicsSystem, renderer: *DebugRenderer) Result;

pub extern fn zjoltPhysicsSystemDrawConstraintLimits(system: *PhysicsSystem, renderer: *DebugRenderer) Result;

pub extern fn zjoltPhysicsSystemDrawConstraintReferenceFrame(system: *PhysicsSystem, renderer: *DebugRenderer) Result;

//=============================================================================
// Soft bodies
//=============================================================================

/// Non-exhaustive: a host can pass any integer, and the C side falls back to
/// TYPE rather than reading an out-of-range enum.
pub const SoftBodyConstraintColor = enum(c_int) { type = 0, group = 1, order = 2, _ };

pub extern fn zjoltSoftBodyDrawVertices(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44) Result;
pub extern fn zjoltSoftBodyDrawVertexVelocities(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44) Result;
pub extern fn zjoltSoftBodyDrawPredictedBounds(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44) Result;
pub extern fn zjoltSoftBodyDrawEdgeConstraints(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44, color: SoftBodyConstraintColor) Result;
pub extern fn zjoltSoftBodyDrawRods(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44, color: SoftBodyConstraintColor) Result;
pub extern fn zjoltSoftBodyDrawRodStates(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44, color: SoftBodyConstraintColor) Result;
pub extern fn zjoltSoftBodyDrawRodBendTwistConstraints(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44, color: SoftBodyConstraintColor) Result;
pub extern fn zjoltSoftBodyDrawBendConstraints(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44, color: SoftBodyConstraintColor) Result;
pub extern fn zjoltSoftBodyDrawVolumeConstraints(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44, color: SoftBodyConstraintColor) Result;
pub extern fn zjoltSoftBodyDrawSkinConstraints(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44, color: SoftBodyConstraintColor) Result;
pub extern fn zjoltSoftBodyDrawLRAConstraints(system: *const PhysicsSystem, body: BodyId, renderer: *DebugRenderer, com_transform: ?*const RMat44, color: SoftBodyConstraintColor) Result;

//=============================================================================
// A shape's own debugging helper
//=============================================================================

pub extern fn zjoltShapeDrawShrunkShape(shape: *const Shape, center_of_mass_transform: *const RMat44, scale: Vec3, renderer: *DebugRenderer) Result;

//=============================================================================
// Record and playback
//=============================================================================

pub const DebugRecorder = opaque {};
pub const DebugPlayback = opaque {};

pub extern fn zjoltDebugRecorderCreate(out: **DebugRecorder) Result;

pub extern fn zjoltDebugRecorderDestroy(recorder: ?*DebugRecorder) void;

pub extern fn zjoltDebugRecorderAsRenderer(recorder: *DebugRecorder) ?*DebugRenderer;

pub extern fn zjoltDebugRecorderEndFrame(recorder: *DebugRecorder) Result;

pub extern fn zjoltDebugRecorderGetData(recorder: *const DebugRecorder, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;

pub extern fn zjoltDebugPlaybackCreate(renderer: *DebugRenderer, out: **DebugPlayback) Result;

pub extern fn zjoltDebugPlaybackDestroy(playback: ?*DebugPlayback) void;

pub extern fn zjoltDebugPlaybackParse(playback: *DebugPlayback, data: ?[*]const u8, size: usize) Result;

pub extern fn zjoltDebugPlaybackGetNumFrames(playback: *const DebugPlayback, out_num_frames: *u32) Result;

pub extern fn zjoltDebugPlaybackDrawFrame(playback: *DebugPlayback, frame_number: u32) Result;
