//! Behavioural tests for debug-draw geometry: a body's shape produces real
//! triangles bounded around where the body actually is, `clear` empties the
//! buffers, a constraint draws its own geometry independent of any body's
//! shape, and the honest refusal without `-Ddebug_renderer=true`.
//!
//! Every entry point is declared unconditionally but returns
//! `error.Unsupported` without the flag, so the first test runs
//! unconditionally; the rest skip (`SkipZigTest`) when it is off, like
//! `hair.zig`'s CPU-compute test.
//!
//! Deliberately does NOT reuse `integration_test.zig`'s `World` fixture for the
//! shape-bounds test: its 50x50 m floor would swallow the bounds under test.
//! Later tests cover host-issued primitives round-tripping, BodyDrawFilter
//! exclusion, a shrunk convex hull, and a recording played back bit-for-bit.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const debug = @import("debug.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;
const World = fixture.World;

//=============================================================================
// Shared helpers for the bounds-based checks below.
//=============================================================================

fn expectNear(v: zjolt.RVec3, center: zjolt.RVec3, radius: zjolt.Real) !void {
    try std.testing.expect(v.x >= center.x - radius and v.x <= center.x + radius);
    try std.testing.expect(v.y >= center.y - radius and v.y <= center.y + radius);
    try std.testing.expect(v.z >= center.z - radius and v.z <= center.z + radius);
}

/// Reads back every line and triangle now in `renderer`, asserts the ones
/// added since `before_lines`/`before_triangles` all sit within `radius` of
/// `center`, and that at least one was added — then advances the counters so
/// the next call only looks at what is new again.
fn assertGeometryGrewNear(
    allocator: std.mem.Allocator,
    renderer: zjolt.DebugRenderer,
    before_lines: *usize,
    before_triangles: *usize,
    center: zjolt.RVec3,
    radius: zjolt.Real,
) !void {
    const lines = try renderer.linesAlloc(allocator);
    defer allocator.free(lines);
    const triangles = try renderer.trianglesAlloc(allocator);
    defer allocator.free(triangles);

    try std.testing.expect(lines.len > before_lines.* or triangles.len > before_triangles.*);

    for (lines[before_lines.*..]) |line| {
        try expectNear(line.from, center, radius);
        try expectNear(line.to, center, radius);
    }
    for (triangles[before_triangles.*..]) |tri| {
        try expectNear(tri.v1, center, radius);
        try expectNear(tri.v2, center, radius);
        try expectNear(tri.v3, center, radius);
    }

    before_lines.* = lines.len;
    before_triangles.* = triangles.len;
}

//=============================================================================
// A minimal one-body, no-floor world, for the shape-bounds test only.
//=============================================================================

const SoloLayers = struct {
    pub const moving: zjolt.ObjectLayer = 0;
    pub const bp_moving: zjolt.BroadPhaseLayer = 0;

    pub fn broadPhaseLayerCount() u32 {
        return 1;
    }
    pub fn broadPhaseLayerFor(_: zjolt.ObjectLayer) zjolt.BroadPhaseLayer {
        return bp_moving;
    }
    pub fn objectCanCollideWithBroadPhase(_: zjolt.ObjectLayer, _: zjolt.BroadPhaseLayer) bool {
        return true;
    }
    pub fn objectsCanCollide(_: zjolt.ObjectLayer, _: zjolt.ObjectLayer) bool {
        return true;
    }
};

//=============================================================================
// Unsupported without the build flag
//=============================================================================

test "debug draw reports Unsupported without -Ddebug_renderer=true, not silently doing nothing" {
    if (zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Fails even though no renderer was ever created — Jolt's own settings
    // type does not exist in this build to read defaults from.
    try std.testing.expectError(zjolt.Error.Unsupported, zjolt.defaultDrawBodiesSettings());
    try std.testing.expectError(zjolt.Error.Unsupported, zjolt.DebugRenderer.init());
    // The recorder shares the same declared-but-inert contract.
    try std.testing.expectError(zjolt.Error.Unsupported, debug.Recorder.init());
}

//=============================================================================
// A body's shape
//=============================================================================

test "drawing a body's shape produces triangles bounded around it, with text for the extras, and clearing empties everything" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try zjolt.PhysicsSystem.init(.{ .layers = zjolt.layersFromType(SoloLayers) });
    defer system.deinit();

    const shape = try zjolt.Shape.initSphere(1.0, .{});
    defer shape.release();

    const center = zjolt.rvec3(2, 6, -3);
    _ = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = SoloLayers.moving,
        .position = center,
    }, .dont_activate);

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();

    // Exercised for coverage: a single sphere draws the same geometry at
    // any level of detail, so nothing below depends on where this puts it.
    renderer.setCameraPosition(zjolt.rvec3(0, 0, 20));

    var settings = try zjolt.defaultDrawBodiesSettings();
    settings.draw_mass_and_inertia = true; // also exercises GetTexts below.
    renderer.target().drawBodies(system, settings, null);

    const triangles = try renderer.trianglesAlloc(std.testing.allocator);
    defer std.testing.allocator.free(triangles);
    // Solid shapes (Jolt's default: draw_shape = true, draw_shape_wireframe
    // = false) come out as triangles, not lines.
    try std.testing.expect(triangles.len > 0);

    var min_x: zjolt.Real = std.math.inf(zjolt.Real);
    var min_y: zjolt.Real = std.math.inf(zjolt.Real);
    var min_z: zjolt.Real = std.math.inf(zjolt.Real);
    var max_x: zjolt.Real = -std.math.inf(zjolt.Real);
    var max_y: zjolt.Real = -std.math.inf(zjolt.Real);
    var max_z: zjolt.Real = -std.math.inf(zjolt.Real);
    for (triangles) |tri| {
        for ([_]zjolt.RVec3{ tri.v1, tri.v2, tri.v3 }) |v| {
            min_x = @min(min_x, v.x);
            min_y = @min(min_y, v.y);
            min_z = @min(min_z, v.z);
            max_x = @max(max_x, v.x);
            max_y = @max(max_y, v.y);
            max_z = @max(max_z, v.z);
        }
    }

    // This world holds exactly one body, so the whole triangle set has to
    // sit within its radius of its center on every axis...
    try std.testing.expect(min_x >= center.x - 1.05 and max_x <= center.x + 1.05);
    try std.testing.expect(min_y >= center.y - 1.05 and max_y <= center.y + 1.05);
    try std.testing.expect(min_z >= center.z - 1.05 and max_z <= center.z + 1.05);
    // ...and span close to its full diameter, rather than being a
    // degenerate sliver or a single point.
    try std.testing.expect(max_x - min_x > 1.5);

    // draw_mass_and_inertia asked for text alongside the shape.
    const texts = try renderer.textsAlloc(std.testing.allocator);
    defer std.testing.allocator.free(texts);
    try std.testing.expect(texts.len > 0);
    try std.testing.expect(zjolt.debugTextSlice(&texts[0]).len > 0);

    renderer.clear();
    try std.testing.expectEqual(@as(usize, 0), try renderer.triangleCount());
    try std.testing.expectEqual(@as(usize, 0), try renderer.lineCount());
    try std.testing.expectEqual(@as(usize, 0), try renderer.textCount());
}

//=============================================================================
// A constraint's own geometry
//=============================================================================

test "drawConstraints draws a constraint's own geometry, independent of any body's shape" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.3, .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 10, 0);
    const ball = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .dont_activate);

    var rod = try zjolt.Constraint.initDistance(world.system, zjolt.world_body, ball, .{
        .point1 = anchor,
        .point2 = anchor,
        .min_distance = 0,
        .max_distance = 3,
    });
    defer rod.release();
    try rod.addTo(world.system);

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();

    // Nothing drawn into a fresh renderer.
    try std.testing.expectEqual(@as(usize, 0), try renderer.lineCount());
    try std.testing.expectEqual(@as(usize, 0), try renderer.triangleCount());

    renderer.target().drawConstraints(world.system);

    // A distance constraint draws as a line between its two anchor points —
    // nothing to do with either body's shape, which drawBodies (never
    // called in this test) is what would emit.
    const after_constraints = try renderer.lineCount();
    try std.testing.expect(after_constraints > 0);

    renderer.target().drawConstraintLimits(world.system);
    renderer.target().drawConstraintReferenceFrame(world.system);
    // Both draw additively into the same buffer, never fewer lines than
    // drawConstraints alone left behind.
    try std.testing.expect(try renderer.lineCount() >= after_constraints);

    renderer.clear();
    try std.testing.expectEqual(@as(usize, 0), try renderer.lineCount());
}

//=============================================================================
// Host-issued primitives
//=============================================================================

test "DrawLine, DrawTriangle and DrawText3D land in the renderer's own buffers with the exact geometry given" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();
    const target = renderer.target();

    const from = zjolt.rvec3(1, 2, 3);
    const to = zjolt.rvec3(4, 5, 6);
    const line_color: zjolt.Color = .{ .r = 10, .g = 20, .b = 30, .a = 255 };
    target.drawLine(from, to, line_color);

    const lines = try renderer.linesAlloc(std.testing.allocator);
    defer std.testing.allocator.free(lines);
    try std.testing.expectEqual(@as(usize, 1), lines.len);
    try std.testing.expectEqual(from, lines[0].from);
    try std.testing.expectEqual(to, lines[0].to);
    try std.testing.expectEqual(line_color, lines[0].color);

    const v1 = zjolt.rvec3(0, 0, 0);
    const v2 = zjolt.rvec3(1, 0, 0);
    const v3 = zjolt.rvec3(0, 1, 0);
    const tri_color: zjolt.Color = .{ .r = 40, .g = 50, .b = 60, .a = 255 };
    target.drawTriangle(v1, v2, v3, tri_color, .on);

    const triangles = try renderer.trianglesAlloc(std.testing.allocator);
    defer std.testing.allocator.free(triangles);
    try std.testing.expectEqual(@as(usize, 1), triangles.len);
    try std.testing.expectEqual(v1, triangles[0].v1);
    try std.testing.expectEqual(v2, triangles[0].v2);
    try std.testing.expectEqual(v3, triangles[0].v3);
    try std.testing.expectEqual(tri_color, triangles[0].color);
    try std.testing.expect(triangles[0].cast_shadow);

    const label = "host label";
    target.drawText3D(zjolt.rvec3(7, 8, 9), label, tri_color, 0.75);

    const texts = try renderer.textsAlloc(std.testing.allocator);
    defer std.testing.allocator.free(texts);
    try std.testing.expectEqual(@as(usize, 1), texts.len);
    try std.testing.expectEqualStrings(label, zjolt.debugTextSlice(&texts[0]));
    try std.testing.expectEqual(@as(f32, 0.75), texts[0].height);
}

test "every other host-issued primitive decomposes into geometry bounded around the parameters given" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();
    const target = renderer.target();
    const color: zjolt.Color = .{ .r = 200, .g = 100, .b = 50, .a = 255 };

    const center = zjolt.rvec3(10, -20, 30);
    const matrix = try zjolt.RMat44.fromRotationTranslation(zjolt.quat_identity, center);
    // Generously covers every radius/half-extent/size used below, so a
    // primitive landing near `center` at all is what is under test here —
    // exact extents are Jolt's own geometry-building code, not this ABI's.
    const margin: zjolt.Real = 3.5;

    var before_lines: usize = 0;
    var before_triangles: usize = 0;
    const alloc = std.testing.allocator;

    const box: zjolt.AABox = .{
        .min = .{ .x = center.x - 1, .y = center.y - 1, .z = center.z - 1 },
        .max = .{ .x = center.x + 1, .y = center.y + 1, .z = center.z + 1 },
    };

    target.drawWireBox(box, color);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawBox(box, color, .on, .solid);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawSphere(center, 1.0, color, .on, .solid);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawWireSphere(center, 1.0, color, 1);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawUnitSphere(matrix, color, .on, .solid);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawWireUnitSphere(matrix, color, 1);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawCapsule(matrix, 1.0, 0.5, color, .on, .solid);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawCylinder(matrix, 1.0, 0.5, color, .on, .solid);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawTaperedCylinder(matrix, 1.0, -1.0, 0.5, 0.25, color, .on, .solid);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawPie(center, 1.0, zjolt.vec3(0, 1, 0), zjolt.vec3(1, 0, 0), 0, std.math.pi / 2.0, color, .on, .solid);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawArrow(center, zjolt.rvec3(center.x + 1, center.y, center.z), color, 0.2);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawMarker(center, color, 1.0);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawCoordinateSystem(matrix, 1.0);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawPlane(center, zjolt.vec3(0, 1, 0), color, 1.0);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    target.drawWireTriangle(
        zjolt.rvec3(center.x - 1, center.y, center.z),
        zjolt.rvec3(center.x + 1, center.y, center.z),
        zjolt.rvec3(center.x, center.y + 1, center.z),
        color,
    );
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);

    const polygon = [_]zjolt.Vec3{
        .{ .x = -1, .y = 0, .z = -1 },
        .{ .x = 1, .y = 0, .z = -1 },
        .{ .x = 1, .y = 0, .z = 1 },
        .{ .x = -1, .y = 0, .z = 1 },
    };
    try target.drawWirePolygon(matrix, &polygon, color, 0);
    try assertGeometryGrewNear(alloc, renderer, &before_lines, &before_triangles, center, margin);
}

test "drawWirePolygon refuses fewer than 2 vertices" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();

    const one = [_]zjolt.Vec3{.{ .x = 0, .y = 0, .z = 0 }};
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        renderer.target().drawWirePolygon(zjolt.rmat44_identity, &one, .{ .r = 1, .g = 1, .b = 1, .a = 1 }, 0),
    );
}

//=============================================================================
// Body-draw filtering
//=============================================================================

test "a BodyDrawFilter excludes the body it names, leaving every other body's shape drawn" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try zjolt.PhysicsSystem.init(.{ .layers = zjolt.layersFromType(SoloLayers) });
    defer system.deinit();

    const shape = try zjolt.Shape.initSphere(1.0, .{});
    defer shape.release();

    const kept_center = zjolt.rvec3(0, 0, 0);
    const excluded_center = zjolt.rvec3(100, 0, 0);

    _ = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = SoloLayers.moving,
        .position = kept_center,
    }, .dont_activate);
    const excluded = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = SoloLayers.moving,
        .position = excluded_center,
    }, .dont_activate);

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();

    const Context = struct {
        excluded_body: zjolt.BodyId,

        fn shouldDraw(user: ?*anyopaque, body: zjolt.BodyId) callconv(.c) bool {
            const self: *const @This() = @ptrCast(@alignCast(user.?));
            return body != self.excluded_body;
        }
    };
    const context = Context{ .excluded_body = excluded };
    const filter = debug.BodyDrawFilter{
        .should_draw = Context.shouldDraw,
        .user = @ptrCast(@constCast(&context)),
    };

    const settings = try zjolt.defaultDrawBodiesSettings();
    renderer.target().drawBodies(system, settings, &filter);

    const triangles = try renderer.trianglesAlloc(std.testing.allocator);
    defer std.testing.allocator.free(triangles);
    const lines = try renderer.linesAlloc(std.testing.allocator);
    defer std.testing.allocator.free(lines);

    // The kept body's shape drew something, and none of it reaches anywhere
    // near the excluded body's position — the only way that is true is if
    // the excluded body's own sphere, centered 100 units away, never drew.
    try std.testing.expect(triangles.len > 0);
    for (triangles) |tri| {
        try expectNear(tri.v1, kept_center, 1.05);
        try expectNear(tri.v2, kept_center, 1.05);
        try expectNear(tri.v3, kept_center, 1.05);
    }
    for (lines) |line| {
        try expectNear(line.from, kept_center, 1.05);
        try expectNear(line.to, kept_center, 1.05);
    }
}

//=============================================================================
// A shape's own debugging helper
//=============================================================================

test "drawShrunkConvexHull draws a convex hull's shrunk surface and refuses a non-hull shape" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const points = [_]zjolt.Vec3{
        .{ .x = -1, .y = -1, .z = -1 },
        .{ .x = 1, .y = -1, .z = -1 },
        .{ .x = -1, .y = 1, .z = -1 },
        .{ .x = -1, .y = -1, .z = 1 },
        .{ .x = 1, .y = 1, .z = 1 },
    };
    const hull = try zjolt.Shape.initConvexHull(&points, .{});
    defer hull.release();

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();

    const center = zjolt.rvec3(5, 5, 5);
    const matrix = try zjolt.RMat44.fromRotationTranslation(zjolt.quat_identity, center);
    try renderer.target().drawShrunkConvexHull(hull, matrix, zjolt.vec3(1, 1, 1));

    // ConvexHullShape::DrawShrunkShape draws only lines (position-to-shrunk-
    // position, plus a short one per contributing face normal) and one
    // DrawText3D point-index label per point — never a triangle.
    const lines = try renderer.linesAlloc(std.testing.allocator);
    defer std.testing.allocator.free(lines);
    try std.testing.expect(lines.len >= points.len);
    for (lines) |line| {
        try expectNear(line.from, center, 3);
        try expectNear(line.to, center, 3);
    }

    const texts = try renderer.textsAlloc(std.testing.allocator);
    defer std.testing.allocator.free(texts);
    try std.testing.expectEqual(points.len, texts.len);

    const sphere = try zjolt.Shape.initSphere(1.0, .{});
    defer sphere.release();
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        renderer.target().drawShrunkConvexHull(sphere, matrix, zjolt.vec3(1, 1, 1)),
    );
}

//=============================================================================
// Record and playback
//=============================================================================

test "a recording played back into a fresh renderer reproduces exactly what was drawn, and DrawFrame refuses an out-of-range frame" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var recorder = try debug.Recorder.init();

    const from = zjolt.rvec3(1, 2, 3);
    const to = zjolt.rvec3(4, 5, 6);
    const line_color: zjolt.Color = .{ .r = 77, .g = 88, .b = 99, .a = 255 };
    recorder.target().?.drawLine(from, to, line_color);
    recorder.endFrame();

    const bytes = try recorder.dataAlloc(std.testing.allocator);
    defer std.testing.allocator.free(bytes);
    try std.testing.expect(bytes.len > 0);

    // The recorder is the only live debug renderer right now, and Jolt's own
    // is a process-wide singleton — it has to go before a playback target
    // can be created.
    recorder.deinit();

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();

    var playback = try debug.Playback.init(renderer);
    defer playback.deinit();

    try std.testing.expectEqual(@as(u32, 0), try playback.numFrames());
    try playback.parse(bytes);
    try std.testing.expectEqual(@as(u32, 1), try playback.numFrames());

    // DebugRendererPlayback::DrawFrame indexes its frame array with no bounds
    // check of its own; this is the guard that stands in for it.
    try std.testing.expectError(zjolt.Error.InvalidArgument, playback.drawFrame(1));

    try playback.drawFrame(0);

    const lines = try renderer.linesAlloc(std.testing.allocator);
    defer std.testing.allocator.free(lines);
    try std.testing.expectEqual(@as(usize, 1), lines.len);
    try std.testing.expectEqual(from, lines[0].from);
    try std.testing.expectEqual(to, lines[0].to);
    try std.testing.expectEqual(line_color, lines[0].color);
}

test "a soft body's vertices, edges and predicted bounds draw, and a rigid body is refused" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try zjolt.PhysicsSystem.init(.{ .layers = zjolt.layersFromType(SoloLayers) });
    defer system.deinit();

    const settings = try zjolt.SoftBodySharedSettings.create();
    defer settings.release();
    try settings.addVertices(&.{
        .{ .position = .{ .x = 0, .y = 0, .z = 0 }, .velocity = zjolt.vec3_zero, .inv_mass = 1 },
        .{ .position = .{ .x = 1, .y = 0, .z = 0 }, .velocity = zjolt.vec3_zero, .inv_mass = 1 },
        .{ .position = .{ .x = 0, .y = 0, .z = 1 }, .velocity = zjolt.vec3_zero, .inv_mass = 1 },
    });
    try settings.addFaces(&.{.{ .vertex = .{ 0, 1, 2 }, .material_index = 0 }});
    try settings.addEdges(&.{
        .{ .vertex = .{ 0, 1 }, .compliance = 0 },
        .{ .vertex = .{ 1, 2 }, .compliance = 0 },
        .{ .vertex = .{ 2, 0 }, .compliance = 0 },
    });
    settings.calculateEdgeLengths();
    settings.optimize();

    const soft = try zjolt.createAndAddSoftBody(system, .{
        .shared_settings = settings,
        .object_layer = SoloLayers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);

    const shape = try zjolt.Shape.initSphere(1.0, .{});
    defer shape.release();
    const rigid = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = SoloLayers.moving,
        .position = zjolt.rvec3(50, 0, 0),
    }, .dont_activate);

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();
    const target = renderer.target();

    // Each of the three streams emits its own primitives, so the counts have
    // to grow between them rather than only at the end.
    try target.drawSoftBodyVertices(system, soft, null);
    const after_vertices = try renderer.triangleCount() + try renderer.lineCount();
    try std.testing.expect(after_vertices > 0);

    try target.drawSoftBodyEdgeConstraints(system, soft, null, .type);
    const after_edges = try renderer.triangleCount() + try renderer.lineCount();
    try std.testing.expect(after_edges > after_vertices);

    try target.drawSoftBodyPredictedBounds(system, soft, null);
    try std.testing.expect((try renderer.triangleCount() + try renderer.lineCount()) > after_edges);

    // A colour value the host made up falls back to TYPE rather than reading
    // an out-of-range enum.
    try target.drawSoftBodyEdgeConstraints(system, soft, null, @enumFromInt(99));

    // A rigid body has no soft-body state to draw, and says so.
    try std.testing.expectError(
        error.InvalidArgument,
        target.drawSoftBodyVertices(system, rigid, null),
    );
}

//=============================================================================
// Batched geometry — the fast path
//=============================================================================

/// Shared by both tests below. `tokens` stands in for GPU resources a real
/// host would allocate per batch; a create callback hands back one entry's
/// address as its opaque handle, distinct from every other live batch.
const BatchContext = struct {
    create_calls: usize = 0,
    last_vertex_count: u32 = 0,
    last_index_count: u32 = 0,
    last_handle: ?*anyopaque = null,
    destroy_calls: usize = 0,
    destroyed_handle: ?*anyopaque = null,
    draw_calls: usize = 0,
    last_lod_count: u32 = 0,
    last_selected_lod: u32 = 0,
    last_translation: zjolt.RVec3 = undefined,
    tokens: [8]u8 = undefined,

    fn nextToken(self: *BatchContext) ?*anyopaque {
        self.last_handle = &self.tokens[self.create_calls % self.tokens.len];
        return self.last_handle;
    }

    fn createTriangleBatch(
        user: ?*anyopaque,
        vertices: ?[*]const debug.BatchVertex,
        vertex_count: u32,
    ) callconv(.c) ?*anyopaque {
        _ = vertices;
        const self: *BatchContext = @ptrCast(@alignCast(user.?));
        self.create_calls += 1;
        self.last_vertex_count = vertex_count;
        return self.nextToken();
    }

    fn createTriangleBatchIndexed(
        user: ?*anyopaque,
        vertices: ?[*]const debug.BatchVertex,
        vertex_count: u32,
        indices: ?[*]const u32,
        index_count: u32,
    ) callconv(.c) ?*anyopaque {
        _ = vertices;
        _ = indices;
        const self: *BatchContext = @ptrCast(@alignCast(user.?));
        self.create_calls += 1;
        self.last_vertex_count = vertex_count;
        self.last_index_count = index_count;
        return self.nextToken();
    }

    fn destroyBatch(user: ?*anyopaque, batch: ?*anyopaque) callconv(.c) void {
        const self: *BatchContext = @ptrCast(@alignCast(user.?));
        self.destroy_calls += 1;
        self.destroyed_handle = batch;
    }

    fn drawGeometry(
        user: ?*anyopaque,
        model_matrix: *const zjolt.RMat44,
        world_space_bounds: *const zjolt.AABox,
        lod_scale_sq: f32,
        model_color: debug.Color,
        geometry: *const debug.BatchGeometry,
        cull_mode: debug.CullMode,
        cast_shadow: debug.CastShadow,
        draw_mode: debug.DrawMode,
    ) callconv(.c) void {
        _ = world_space_bounds;
        _ = lod_scale_sq;
        _ = model_color;
        _ = cull_mode;
        _ = cast_shadow;
        _ = draw_mode;
        const self: *BatchContext = @ptrCast(@alignCast(user.?));
        self.draw_calls += 1;
        self.last_lod_count = geometry.lod_count;
        self.last_selected_lod = geometry.selected_lod;
        self.last_translation = .{
            .x = model_matrix.m[12],
            .y = model_matrix.m[13],
            .z = model_matrix.m[14],
        };
    }
};

test "batch callbacks reroute CreateTriangleBatch/DrawGeometry off the immediate-mode path, and destroy_batch runs once the last reference drops" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try zjolt.PhysicsSystem.init(.{ .layers = zjolt.layersFromType(SoloLayers) });
    defer system.deinit();

    // ConvexHullShape::Draw builds its (single-LOD) geometry from an
    // unindexed triangle list the first time it draws, and caches it on the
    // shape from then on — exercises CreateTriangleBatch's Triangle overload.
    const points = [_]zjolt.Vec3{
        .{ .x = -1, .y = -1, .z = -1 },
        .{ .x = 1, .y = -1, .z = -1 },
        .{ .x = -1, .y = 1, .z = -1 },
        .{ .x = -1, .y = -1, .z = 1 },
        .{ .x = 1, .y = 1, .z = 1 },
    };
    const hull = try zjolt.Shape.initConvexHull(&points, .{});

    const center = zjolt.rvec3(20, 5, -8);
    const body = try system.bodies().createAndAdd(.{
        .shape = hull,
        .object_layer = SoloLayers.moving,
        .position = center,
    }, .dont_activate);

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();

    var ctx = BatchContext{};
    const callbacks = debug.BatchCallbacks{
        .create_triangle_batch = BatchContext.createTriangleBatch,
        .destroy_batch = BatchContext.destroyBatch,
        .draw_geometry = BatchContext.drawGeometry,
        .user = &ctx,
    };
    try renderer.setBatchCallbacks(&callbacks);

    const settings = try zjolt.defaultDrawBodiesSettings();
    renderer.target().drawBodies(system, settings, null);

    try std.testing.expectEqual(@as(usize, 1), ctx.create_calls);
    try std.testing.expect(ctx.last_vertex_count > 0);
    try std.testing.expectEqual(@as(u32, 0), ctx.last_vertex_count % 3);
    try std.testing.expectEqual(@as(usize, 1), ctx.draw_calls);
    try std.testing.expectEqual(@as(u32, 1), ctx.last_lod_count);
    try std.testing.expectEqual(@as(u32, 0), ctx.last_selected_lod);
    // Within the hull's own extent of `center`, not exactly on it: the
    // model matrix DrawGeometry receives is the center-of-MASS transform,
    // offset from the body's position by wherever this asymmetric hull's
    // mass center falls.
    try expectNear(ctx.last_translation, center, 2.0);

    // The batched path claimed the triangles the immediate-mode fallback
    // would otherwise have produced.
    try std.testing.expectEqual(@as(usize, 0), try renderer.triangleCount());

    // Still referenced by the shape's cached geometry.
    try std.testing.expectEqual(@as(usize, 0), ctx.destroy_calls);

    // Dropping every reference — the body's and this test's own — releases
    // the shape, which releases its cached Geometry, which releases the
    // batch: destroy_batch runs exactly once, with the handle create
    // returned.
    system.bodies().destroy(body);
    hull.release();
    try std.testing.expectEqual(@as(usize, 1), ctx.destroy_calls);
    try std.testing.expectEqual(ctx.last_handle, ctx.destroyed_handle);

    // Clearing the callbacks restores the immediate-mode fallback for a
    // shape drawn from here on; no further batch call fires for it.
    try renderer.setBatchCallbacks(null);
    const sphere = try zjolt.Shape.initSphere(1.0, .{});
    defer sphere.release();
    _ = try system.bodies().createAndAdd(.{
        .shape = sphere,
        .object_layer = SoloLayers.moving,
        .position = zjolt.rvec3(0, 0, 0),
    }, .dont_activate);
    renderer.target().drawBodies(system, settings, null);
    try std.testing.expect(try renderer.triangleCount() > 0);
    try std.testing.expectEqual(@as(usize, 1), ctx.create_calls);
    try std.testing.expectEqual(@as(usize, 1), ctx.draw_calls);
}

test "a convex shape's multi-level geometry is created once and redrawn many times, and GetLOD picks a coarser level for a farther camera" {
    if (!zjolt.options.debug_renderer) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try zjolt.PhysicsSystem.init(.{ .layers = zjolt.layersFromType(SoloLayers) });
    defer system.deinit();

    // TaperedCapsuleShape::Draw builds its geometry, on first draw, through
    // CreateTriangleGeometryForConvex — one CreateTriangleBatchForConvex
    // (and so one create_triangle_batch_indexed call) per level of detail.
    const shape = try zjolt.Shape.initTaperedCapsule(1.0, 0.5, 0.3, .{});
    defer shape.release();

    const center = zjolt.rvec3(0, 0, 0);
    const body = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = SoloLayers.moving,
        .position = center,
    }, .dont_activate);
    defer system.bodies().destroy(body);

    var renderer = try zjolt.DebugRenderer.init();
    defer renderer.deinit();

    var ctx = BatchContext{};
    const callbacks = debug.BatchCallbacks{
        .create_triangle_batch_indexed = BatchContext.createTriangleBatchIndexed,
        .destroy_batch = BatchContext.destroyBatch,
        .draw_geometry = BatchContext.drawGeometry,
        .user = &ctx,
    };
    try renderer.setBatchCallbacks(&callbacks);
    const settings = try zjolt.defaultDrawBodiesSettings();

    // No camera position set: GetLOD is never consulted, so the finest LOD
    // (index 0) is what draws.
    renderer.target().drawBodies(system, settings, null);
    try std.testing.expectEqual(@as(usize, 1), ctx.draw_calls);
    try std.testing.expect(ctx.last_lod_count > 1);
    try std.testing.expectEqual(ctx.last_lod_count, @as(u32, @intCast(ctx.create_calls)));
    try std.testing.expectEqual(@as(u32, 0), ctx.last_selected_lod);
    const lod_count = ctx.last_lod_count;
    const created_after_first_draw = ctx.create_calls;

    // A camera far from the body selects the coarsest, last LOD instead —
    // and the geometry, cached on the shape, is not rebuilt to get there.
    renderer.setCameraPosition(zjolt.rvec3(0, 0, 1000));
    renderer.target().drawBodies(system, settings, null);
    try std.testing.expectEqual(@as(usize, 2), ctx.draw_calls);
    try std.testing.expectEqual(created_after_first_draw, ctx.create_calls);
    try std.testing.expectEqual(lod_count - 1, ctx.last_selected_lod);

    try std.testing.expectEqual(@as(usize, 0), try renderer.triangleCount());
}
