//! Debug-draw geometry, collected into arrays.
//!
//! Jolt's `DebugRenderer` is an abstract C++ class; nothing here mirrors its
//! vtable. `Renderer` is a sink the C library fills in by subclassing it once,
//! internally, and this wrapper only reads back what accumulated — lines,
//! triangles, text — the same way any other bulk result in this package
//! reads back: a count, then a fill into a caller-owned buffer.
//!
//! Every function on `Renderer`, and `defaultDrawBodiesSettings` below,
//! returns `error.Unsupported` unless the library was built with
//! `-Ddebug_renderer=true`. `zjolt.options.debug_renderer` says which,
//! without needing to fail a call first to find out.
//!
//! ```zig
//! var renderer = try zjolt.DebugRenderer.init();
//! defer renderer.deinit();
//!
//! // Per frame:
//! renderer.clear();
//! renderer.drawBodies(system, try zjolt.defaultDrawBodiesSettings());
//! const lines = try renderer.linesAlloc(gpa);
//! defer gpa.free(lines);
//! // hand `lines` to whatever renders them
//! ```

const std = @import("std");
const c = @import("c/debug.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const system_mod = @import("system.zig");

pub const Line = c.DebugLine;
pub const Triangle = c.DebugTriangle;
pub const Text = c.DebugText;
pub const DrawBodiesSettings = c.DebugDrawBodiesSettings;
pub const ShapeColor = c.ShapeColor;

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
/// may be alive at a time, process-wide, not just per `PhysicsSystem`. `init`
/// fails with `error.AlreadyInitialized` while another is still alive;
/// `deinit` the one you have before creating the next.
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

    /// Picks the level of detail for the solid shapes `drawBodies` emits.
    /// Optional; skipping it draws every shape at its highest detail.
    pub fn setCameraPosition(self: Renderer, position: math.RVec3) void {
        _ = c.zjoltDebugRendererSetCameraPosition(self.handle, &position);
    }

    /// Draws every body's shape, plus whichever extras `settings` asks for,
    /// into this renderer. `zjolt.defaultDrawBodiesSettings()` is a
    /// reasonable starting point.
    pub fn drawBodies(
        self: Renderer,
        system: system_mod.PhysicsSystem,
        settings: DrawBodiesSettings,
    ) void {
        _ = c.zjoltPhysicsSystemDrawBodies(system.handle, &settings, self.handle);
    }

    pub fn drawConstraints(self: Renderer, system: system_mod.PhysicsSystem) void {
        _ = c.zjoltPhysicsSystemDrawConstraints(system.handle, self.handle);
    }

    pub fn drawConstraintLimits(self: Renderer, system: system_mod.PhysicsSystem) void {
        _ = c.zjoltPhysicsSystemDrawConstraintLimits(system.handle, self.handle);
    }

    pub fn drawConstraintReferenceFrame(self: Renderer, system: system_mod.PhysicsSystem) void {
        _ = c.zjoltPhysicsSystemDrawConstraintReferenceFrame(system.handle, self.handle);
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
