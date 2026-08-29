//! Debug-draw geometry, collected into arrays.
//!
//! Jolt's `DebugRenderer` is an abstract C++ class; nothing here mirrors
//! its vtable. `Renderer` is a sink the library fills in once, and this
//! wrapper reads back what accumulated — lines, triangles, text — as a
//! count then a fill into a caller-owned buffer. Every function on
//! `Renderer`, and `defaultDrawBodiesSettings`, returns
//! `error.Unsupported` unless built with `-Ddebug_renderer=true`
//! (`zjolt.options.debug_renderer` says which, without failing first).
//!
//! `renderer.target()` is also where a host's own geometry goes —
//! drawLine/drawBox/drawSphere and "Host-issued primitives"
//! (`ffi/zjolt_debug.h`) land in the same buffers `drawBodies` fills.
//! `Recorder` is the other `DrawTarget`, serialising calls to a stream `Playback` can replay instead of buffering one frame.

const std = @import("std");
const c = @import("c/debug.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const shape_mod = @import("shape.zig");
const system_mod = @import("system.zig");

pub const Line = c.DebugLine;
pub const Triangle = c.DebugTriangle;
pub const Text = c.DebugText;
pub const DrawBodiesSettings = c.DebugDrawBodiesSettings;
pub const ShapeColor = c.ShapeColor;
pub const Color = c.Color;
pub const CastShadow = c.CastShadow;
pub const DrawMode = c.DrawMode;
pub const BodyDrawFilter = c.BodyDrawFilter;

/// See `Renderer.setBatchCallbacks` and `ffi/zjolt_debug.h` for the batched
/// geometry fast path these five make up.
pub const BatchVertex = c.DebugVertex;
pub const CullMode = c.CullMode;
pub const BatchLOD = c.DebugLOD;
pub const BatchGeometry = c.DebugGeometry;
pub const BatchCallbacks = c.DebugBatchCallbacks;

/// `entry.text`, trimmed at its NUL terminator.
///
/// Truncated to `text_max_length - 1` bytes if the string Jolt drew was
/// longer; `entry.text_length` still carries the true length, so truncation
/// is detectable rather than silent.
pub fn textSlice(entry: *const Text) [:0]const u8 {
    // Through a sentinel-typed pointer, so the result carries the sentinel.
    // `sliceTo` on the array itself yields a plain slice, and the buffer is
    // always terminated: Jolt's text is truncated to text_max_length - 1.
    const text: [*:0]const u8 = @ptrCast(&entry.text);
    return std.mem.sliceTo(text, 0);
}

pub const text_max_length = c.debug_text_max_length;

/// Fills in `DrawBodiesSettings` with Jolt's own defaults.
///
/// Unlike every method on `Renderer`, this can be called without ever having
/// created one — and still fails with `error.Unsupported` when the library
/// was not built with `-Ddebug_renderer=true`, since Jolt's own settings type
/// does not exist in that build to read defaults from.
pub fn defaultDrawBodiesSettings() err.Error!DrawBodiesSettings {
    var settings: DrawBodiesSettings = undefined;
    try err.check(c.zjoltDebugDrawBodiesSettingsInit(&settings));
    return settings;
}

/// A sink for lines, triangles and text, filled in by drawing a system into
/// it and read back as arrays.
///
/// Jolt's `DebugRenderer` is a process-wide singleton — only one `Renderer`
/// may be alive at a time. `init` fails with `error.AlreadyInitialized`
/// while another is still alive; `deinit` the one you have before the next.
pub const Renderer = struct {
    handle: *c.DebugRenderer,

    /// Fails with `error.Unsupported` unless this library was built with
    /// `-Ddebug_renderer=true`.
    pub fn init() err.Error!Renderer {
        var handle: *c.DebugRenderer = undefined;
        try err.check(c.zjoltDebugRendererCreate(&handle));
        return .{ .handle = handle };
    }

    /// The only failure the C entry point reports (`error.Unsupported`)
    /// cannot happen to a handle `init` already succeeded in producing, so
    /// this discards the result rather than asking the caller to handle it —
    /// matching every other `deinit` in this package.
    pub fn deinit(self: Renderer) void {
        c.zjoltDebugRendererDestroy(self.handle);
    }

    /// Empties the buffered lines, triangles and text. Call once per frame
    /// before drawing into a renderer again — the buffers only grow
    /// otherwise.
    pub fn clear(self: Renderer) void {
        _ = c.zjoltDebugRendererClear(self.handle);
    }

    /// Picks the level of detail for the solid shapes `target().drawBodies`
    /// emits. Optional; skipping it draws every shape at its highest detail.
    pub fn setCameraPosition(self: Renderer, position: math.RVec3) void {
        _ = c.zjoltDebugRendererSetCameraPosition(self.handle, &position);
    }

    /// Routes CreateTriangleBatch/DrawGeometry to `callbacks`, replacing
    /// whichever were set before; `null` restores the immediate-mode
    /// fallback for both. `callbacks`' function pointers and `user` must
    /// stay valid for every draw through this renderer until this runs
    /// again or `deinit`. `error.InvalidArgument` if `callbacks` is
    /// non-null with a null `draw_geometry`.
    pub fn setBatchCallbacks(self: Renderer, callbacks: ?*const BatchCallbacks) err.Error!void {
        try err.check(c.zjoltDebugRendererSetBatchCallbacks(self.handle, callbacks));
    }

    /// Where to draw: every `DrawTarget` method — `drawBodies`, `drawLine`,
    /// the rest of "Host-issued primitives" in `ffi/zjolt_debug.h` — writes
    /// into this renderer's buffers, read back below.
    pub fn target(self: Renderer) DrawTarget {
        return .{ .handle = self.handle };
    }

    //-------------------------------------------------------------------------
    // Read-back
    //-------------------------------------------------------------------------

    pub fn lineCount(self: Renderer) err.Error!usize {
        var needed: u32 = 0;
        try err.check(c.zjoltDebugRendererGetLines(self.handle, null, 0, &needed));
        return needed;
    }

    /// Fills `buffer`, returning the part that was used.
    /// `error.BufferTooSmall` if it does not fit; ask `lineCount` first.
    pub fn lines(self: Renderer, buffer: []Line) err.Error![]Line {
        var written: u32 = 0;
        try err.check(c.zjoltDebugRendererGetLines(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &written,
        ));
        return buffer[0..written];
    }

    /// `lines` into memory from `allocator`. The caller owns the slice.
    pub fn linesAlloc(
        self: Renderer,
        allocator: std.mem.Allocator,
    ) (err.Error || std.mem.Allocator.Error)![]Line {
        const needed = try self.lineCount();
        const buffer = try allocator.alloc(Line, needed);
        errdefer allocator.free(buffer);
        return try self.lines(buffer);
    }

    pub fn triangleCount(self: Renderer) err.Error!usize {
        var needed: u32 = 0;
        try err.check(c.zjoltDebugRendererGetTriangles(self.handle, null, 0, &needed));
        return needed;
    }

    /// Fills `buffer`, returning the part that was used.
    /// `error.BufferTooSmall` if it does not fit; ask `triangleCount` first.
    pub fn triangles(self: Renderer, buffer: []Triangle) err.Error![]Triangle {
        var written: u32 = 0;
        try err.check(c.zjoltDebugRendererGetTriangles(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &written,
        ));
        return buffer[0..written];
    }

    /// `triangles` into memory from `allocator`. The caller owns the slice.
    pub fn trianglesAlloc(
        self: Renderer,
        allocator: std.mem.Allocator,
    ) (err.Error || std.mem.Allocator.Error)![]Triangle {
        const needed = try self.triangleCount();
        const buffer = try allocator.alloc(Triangle, needed);
        errdefer allocator.free(buffer);
        return try self.triangles(buffer);
    }

    pub fn textCount(self: Renderer) err.Error!usize {
        var needed: u32 = 0;
        try err.check(c.zjoltDebugRendererGetTexts(self.handle, null, 0, &needed));
        return needed;
    }

    /// Fills `buffer`, returning the part that was used.
    /// `error.BufferTooSmall` if it does not fit; ask `textCount` first.
    pub fn texts(self: Renderer, buffer: []Text) err.Error![]Text {
        var written: u32 = 0;
        try err.check(c.zjoltDebugRendererGetTexts(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &written,
        ));
        return buffer[0..written];
    }

    /// `texts` into memory from `allocator`. The caller owns the slice.
    pub fn textsAlloc(
        self: Renderer,
        allocator: std.mem.Allocator,
    ) (err.Error || std.mem.Allocator.Error)![]Text {
        const needed = try self.textCount();
        const buffer = try allocator.alloc(Text, needed);
        errdefer allocator.free(buffer);
        return try self.texts(buffer);
    }
};

//=============================================================================
// Host-issued primitives
//=============================================================================

/// Where debug geometry actually goes — a `Renderer`'s buffers via
/// `Renderer.target()`, or a `Recorder`'s stream via `Recorder.target()`.
/// A host's own lines, boxes, spheres and labels land in the same place
/// `drawBodies` does.
///
/// Kept distinct from `Renderer`: a `Recorder`'s view has no `clear` or read-back, and must never be `deinit`'d (`ffi/zjolt_debug.h`, `zjoltDebugRecorderAsRenderer`) — only `Renderer` itself owns geometry.
pub const DrawTarget = struct {
    handle: *c.DebugRenderer,

    pub fn drawLine(self: DrawTarget, from: math.RVec3, to: math.RVec3, color: Color) void {
        _ = c.zjoltDebugRendererDrawLine(self.handle, &from, &to, color);
    }

    pub fn drawTriangle(
        self: DrawTarget,
        v1: math.RVec3,
        v2: math.RVec3,
        v3: math.RVec3,
        color: Color,
        cast_shadow: CastShadow,
    ) void {
        _ = c.zjoltDebugRendererDrawTriangle(self.handle, &v1, &v2, &v3, color, cast_shadow);
    }

    /// `text` need not be NUL-terminated and may contain any bytes.
    pub fn drawText3D(self: DrawTarget, position: math.RVec3, text: []const u8, color: Color, height: f32) void {
        _ = c.zjoltDebugRendererDrawText3D(self.handle, &position, text.ptr, @intCast(text.len), color, height);
    }

    /// `box` is already in world space — see `ffi/zjolt_debug.h` for why
    /// there is no separate oriented-box overload.
    pub fn drawWireBox(self: DrawTarget, box: math.AABox, color: Color) void {
        _ = c.zjoltDebugRendererDrawWireBox(self.handle, &box, color);
    }

    pub fn drawBox(self: DrawTarget, box: math.AABox, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) void {
        _ = c.zjoltDebugRendererDrawBox(self.handle, &box, color, cast_shadow, draw_mode);
    }

    pub fn drawSphere(
        self: DrawTarget,
        center: math.RVec3,
        radius: f32,
        color: Color,
        cast_shadow: CastShadow,
        draw_mode: DrawMode,
    ) void {
        _ = c.zjoltDebugRendererDrawSphere(self.handle, &center, radius, color, cast_shadow, draw_mode);
    }

    pub fn drawWireSphere(self: DrawTarget, center: math.RVec3, radius: f32, color: Color, level: i32) void {
        _ = c.zjoltDebugRendererDrawWireSphere(self.handle, &center, radius, color, level);
    }

    /// `matrix`'s scale IS the sphere's shape, which may come out
    /// non-uniform; there is no separate radius.
    pub fn drawUnitSphere(self: DrawTarget, matrix: math.RMat44, color: Color, cast_shadow: CastShadow, draw_mode: DrawMode) void {
        _ = c.zjoltDebugRendererDrawUnitSphere(self.handle, &matrix, color, cast_shadow, draw_mode);
    }

    pub fn drawWireUnitSphere(self: DrawTarget, matrix: math.RMat44, color: Color, level: i32) void {
        _ = c.zjoltDebugRendererDrawWireUnitSphere(self.handle, &matrix, color, level);
    }

    pub fn drawCapsule(
        self: DrawTarget,
        matrix: math.RMat44,
        half_height_of_cylinder: f32,
        radius: f32,
        color: Color,
        cast_shadow: CastShadow,
        draw_mode: DrawMode,
    ) void {
        _ = c.zjoltDebugRendererDrawCapsule(
            self.handle,
            &matrix,
            half_height_of_cylinder,
            radius,
            color,
            cast_shadow,
            draw_mode,
        );
    }

    pub fn drawCylinder(
        self: DrawTarget,
        matrix: math.RMat44,
        half_height: f32,
        radius: f32,
        color: Color,
        cast_shadow: CastShadow,
        draw_mode: DrawMode,
    ) void {
        _ = c.zjoltDebugRendererDrawCylinder(self.handle, &matrix, half_height, radius, color, cast_shadow, draw_mode);
    }

    pub fn drawTaperedCylinder(
        self: DrawTarget,
        matrix: math.RMat44,
        top: f32,
        bottom: f32,
        top_radius: f32,
        bottom_radius: f32,
        color: Color,
        cast_shadow: CastShadow,
        draw_mode: DrawMode,
    ) void {
        _ = c.zjoltDebugRendererDrawTaperedCylinder(
            self.handle,
            &matrix,
            top,
            bottom,
            top_radius,
            bottom_radius,
            color,
            cast_shadow,
            draw_mode,
        );
    }

    /// A wedge of a circle between `min_angle` and `max_angle` radians,
    /// measured from `axis`, in the plane through `center` with `normal`.
    pub fn drawPie(
        self: DrawTarget,
        center: math.RVec3,
        radius: f32,
        normal: math.Vec3,
        axis: math.Vec3,
        min_angle: f32,
        max_angle: f32,
        color: Color,
        cast_shadow: CastShadow,
        draw_mode: DrawMode,
    ) void {
        _ = c.zjoltDebugRendererDrawPie(
            self.handle,
            &center,
            radius,
            &normal,
            &axis,
            min_angle,
            max_angle,
            color,
            cast_shadow,
            draw_mode,
        );
    }

    pub fn drawArrow(self: DrawTarget, from: math.RVec3, to: math.RVec3, color: Color, size: f32) void {
        _ = c.zjoltDebugRendererDrawArrow(self.handle, &from, &to, color, size);
    }

    /// Three short lines through `position` along the coordinate axes.
    pub fn drawMarker(self: DrawTarget, position: math.RVec3, color: Color, size: f32) void {
        _ = c.zjoltDebugRendererDrawMarker(self.handle, &position, color, size);
    }

    /// Three arrows out of `transform`'s origin along its own axes: x red, y
    /// green, z blue.
    pub fn drawCoordinateSystem(self: DrawTarget, transform: math.RMat44, size: f32) void {
        _ = c.zjoltDebugRendererDrawCoordinateSystem(self.handle, &transform, size);
    }

    pub fn drawPlane(self: DrawTarget, point: math.RVec3, normal: math.Vec3, color: Color, size: f32) void {
        _ = c.zjoltDebugRendererDrawPlane(self.handle, &point, &normal, color, size);
    }

    pub fn drawWireTriangle(self: DrawTarget, v1: math.RVec3, v2: math.RVec3, v3: math.RVec3, color: Color) void {
        _ = c.zjoltDebugRendererDrawWireTriangle(self.handle, &v1, &v2, &v3, color);
    }

    /// `vertices` are in `transform`'s local space, drawn as arrows
    /// tip-to-tail around the polygon they form. `error.InvalidArgument` if
    /// fewer than 2 vertices are given.
    pub fn drawWirePolygon(
        self: DrawTarget,
        transform: math.RMat44,
        vertices: []const math.Vec3,
        color: Color,
        arrow_size: f32,
    ) err.Error!void {
        try err.check(c.zjoltDebugRendererDrawWirePolygon(
            self.handle,
            &transform,
            vertices.ptr,
            @intCast(vertices.len),
            color,
            arrow_size,
        ));
    }

    /// Draws every body's shape, plus whichever extras `settings` asks for.
    /// `zjolt.defaultDrawBodiesSettings()` is a reasonable starting point.
    /// `filter` may be null to draw every body.
    pub fn drawBodies(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        settings: DrawBodiesSettings,
        filter: ?*const BodyDrawFilter,
    ) void {
        _ = c.zjoltPhysicsSystemDrawBodies(system.handle, &settings, self.handle, filter);
    }

    pub fn drawConstraints(self: DrawTarget, system: system_mod.PhysicsSystem) void {
        _ = c.zjoltPhysicsSystemDrawConstraints(system.handle, self.handle);
    }

    pub fn drawConstraintLimits(self: DrawTarget, system: system_mod.PhysicsSystem) void {
        _ = c.zjoltPhysicsSystemDrawConstraintLimits(system.handle, self.handle);
    }

    pub fn drawConstraintReferenceFrame(self: DrawTarget, system: system_mod.PhysicsSystem) void {
        _ = c.zjoltPhysicsSystemDrawConstraintReferenceFrame(system.handle, self.handle);
    }

    /// How a soft body's constraints are coloured.
    pub const SoftBodyConstraintColor = c.SoftBodyConstraintColor;

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyVertices(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawVertices(system.handle, body, self.handle, com_transform));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyVertexVelocities(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawVertexVelocities(system.handle, body, self.handle, com_transform));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyPredictedBounds(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawPredictedBounds(system.handle, body, self.handle, com_transform));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyEdgeConstraints(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
        color: SoftBodyConstraintColor,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawEdgeConstraints(system.handle, body, self.handle, com_transform, color));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyRods(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
        color: SoftBodyConstraintColor,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawRods(system.handle, body, self.handle, com_transform, color));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyRodStates(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
        color: SoftBodyConstraintColor,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawRodStates(system.handle, body, self.handle, com_transform, color));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyRodBendTwistConstraints(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
        color: SoftBodyConstraintColor,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawRodBendTwistConstraints(system.handle, body, self.handle, com_transform, color));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyBendConstraints(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
        color: SoftBodyConstraintColor,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawBendConstraints(system.handle, body, self.handle, com_transform, color));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyVolumeConstraints(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
        color: SoftBodyConstraintColor,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawVolumeConstraints(system.handle, body, self.handle, com_transform, color));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodySkinConstraints(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
        color: SoftBodyConstraintColor,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawSkinConstraints(system.handle, body, self.handle, com_transform, color));
    }

    /// `com_transform` may be null for the body's own centre-of-mass transform.
    pub fn drawSoftBodyLRAConstraints(
        self: DrawTarget,
        system: system_mod.PhysicsSystem,
        body: c.BodyId,
        com_transform: ?*const math.RMat44,
        color: SoftBodyConstraintColor,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodyDrawLRAConstraints(system.handle, body, self.handle, com_transform, color));
    }

    /// `ConvexHullShape::DrawShrunkShape`: the hull shrunk inward by its
    /// convex radius, which is the surface collision detection actually
    /// uses — `drawBodies` always shows the hull before shrinking.
    /// `error.InvalidArgument` if `shape`'s subtype is not a convex hull.
    pub fn drawShrunkConvexHull(
        self: DrawTarget,
        shape: shape_mod.Shape,
        center_of_mass_transform: math.RMat44,
        scale: math.Vec3,
    ) err.Error!void {
        try err.check(c.zjoltShapeDrawShrunkShape(shape.handle, &center_of_mass_transform, scale, self.handle));
    }
};

//=============================================================================
// Record and playback
//=============================================================================

/// A second kind of draw destination: instead of buffering the current
/// frame for read-back like `Renderer`, it serialises every call into a
/// stream, so a session can be replayed and inspected after the fact.
///
/// Shares `Renderer`'s process-wide singleton slot: `init` fails with
/// `error.AlreadyInitialized` while a `Renderer` or another `Recorder` is still alive.
pub const Recorder = struct {
    handle: *c.DebugRecorder,

    pub fn init() err.Error!Recorder {
        var handle: *c.DebugRecorder = undefined;
        try err.check(c.zjoltDebugRecorderCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: Recorder) void {
        c.zjoltDebugRecorderDestroy(self.handle);
    }

    /// Where to draw to record. Every `DrawTarget` method writes into this
    /// recorder's stream instead of a `Renderer`'s read-back arrays. Valid
    /// only as long as `self` is.
    pub fn target(self: Recorder) ?DrawTarget {
        const handle = c.zjoltDebugRecorderAsRenderer(self.handle) orelse return null;
        return .{ .handle = handle };
    }

    /// Flushes everything drawn into `target()` since the last `endFrame`
    /// (or since `init`) into the stream as one frame. Nothing drawn is
    /// visible to `Playback.parse` until this runs.
    pub fn endFrame(self: Recorder) void {
        _ = c.zjoltDebugRecorderEndFrame(self.handle);
    }

    pub fn dataSize(self: Recorder) err.Error!usize {
        var needed: usize = 0;
        try err.check(c.zjoltDebugRecorderGetData(self.handle, null, 0, &needed));
        return needed;
    }

    /// Fills `buffer`, returning the part that was used.
    /// `error.BufferTooSmall` if it does not fit; ask `dataSize` first.
    pub fn data(self: Recorder, buffer: []u8) err.Error![]u8 {
        var written: usize = 0;
        try err.check(c.zjoltDebugRecorderGetData(self.handle, buffer.ptr, buffer.len, &written));
        return buffer[0..written];
    }

    /// `data` into memory from `allocator`. The caller owns the slice.
    pub fn dataAlloc(self: Recorder, allocator: std.mem.Allocator) (err.Error || std.mem.Allocator.Error)![]u8 {
        const needed = try self.dataSize();
        const buffer = try allocator.alloc(u8, needed);
        errdefer allocator.free(buffer);
        return try self.data(buffer);
    }
};

/// Replays a `Recorder.data`/`dataAlloc` recording into a `Renderer`, frame
/// by frame.
pub const Playback = struct {
    handle: *c.DebugPlayback,

    /// `renderer` must outlive this playback.
    pub fn init(renderer: Renderer) err.Error!Playback {
        var handle: *c.DebugPlayback = undefined;
        try err.check(c.zjoltDebugPlaybackCreate(renderer.handle, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: Playback) void {
        c.zjoltDebugPlaybackDestroy(self.handle);
    }

    /// Adds the frames in `data` to this playback's timeline. Calling this
    /// more than once appends further frames rather than replacing the first
    /// batch.
    pub fn parse(self: Playback, frames: []const u8) err.Error!void {
        try err.check(c.zjoltDebugPlaybackParse(self.handle, frames.ptr, frames.len));
    }

    pub fn numFrames(self: Playback) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltDebugPlaybackGetNumFrames(self.handle, &count));
        return count;
    }

    /// Replays `frame_number` into the renderer this playback was created
    /// with. `error.InvalidArgument` if `frame_number` is not below
    /// `numFrames()`.
    pub fn drawFrame(self: Playback, frame_number: u32) err.Error!void {
        try err.check(c.zjoltDebugPlaybackDrawFrame(self.handle, frame_number));
    }
};
