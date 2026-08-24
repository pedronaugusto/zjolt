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
pub const Color = core.Color;
pub const PhysicsSystem = core.PhysicsSystem;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;

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

pub extern fn zjoltDebugRendererGetLines(renderer: *const DebugRenderer, lines: ?[*]DebugLine, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltDebugRendererGetTriangles(renderer: *const DebugRenderer, triangles: ?[*]DebugTriangle, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltDebugRendererGetTexts(renderer: *const DebugRenderer, texts: ?[*]DebugText, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltDebugDrawBodiesSettingsInit(settings: *DebugDrawBodiesSettings) Result;

pub extern fn zjoltPhysicsSystemDrawBodies(system: *PhysicsSystem, settings: *const DebugDrawBodiesSettings, renderer: *DebugRenderer) Result;

pub extern fn zjoltPhysicsSystemDrawConstraints(system: *PhysicsSystem, renderer: *DebugRenderer) Result;

pub extern fn zjoltPhysicsSystemDrawConstraintLimits(system: *PhysicsSystem, renderer: *DebugRenderer) Result;

pub extern fn zjoltPhysicsSystemDrawConstraintReferenceFrame(system: *PhysicsSystem, renderer: *DebugRenderer) Result;
