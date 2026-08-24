//! Behavioural tests for debug-draw geometry: a body's shape produces real
//! triangles bounded around where the body actually is, `clear` empties the
//! buffers, a constraint draws its own geometry independent of any body's
//! shape, and — since none of that exists unless this library was built with
//! `-Ddebug_renderer=true` — the honest refusal when it was not.
//!
//! Every entry point here is declared unconditionally (see `ffi/zjolt_debug.h`)
//! but returns `error.Unsupported` in a build without the flag, so the first
//! test below runs unconditionally too: it is the behaviour of a default
//! build. The rest need real geometry to assert on and skip (`SkipZigTest`)
//! when the flag is off — the same pattern `hair.zig`'s CPU-compute test
//! uses for its own build-time-optional feature.
//!
//! Deliberately does NOT reuse `integration_test.zig`'s `World` fixture for
//! the shape-bounds test: that fixture's floor is a 50x50 m box, and drawing
//! it alongside the sphere under test would swallow the very bounds this
//! test is checking. The constraint-drawing test has no such problem — a
//! constraint draws its own geometry, not a body's shape — so it reuses the
//! fixture as everywhere else in this suite does.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;
const World = fixture.World;

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
    renderer.drawBodies(system, settings);

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

    renderer.drawConstraints(world.system);

    // A distance constraint draws as a line between its two anchor points —
    // nothing to do with either body's shape, which drawBodies (never
    // called in this test) is what would emit.
    const after_constraints = try renderer.lineCount();
    try std.testing.expect(after_constraints > 0);

    renderer.drawConstraintLimits(world.system);
    renderer.drawConstraintReferenceFrame(world.system);
    // Both draw additively into the same buffer, never fewer lines than
    // drawConstraints alone left behind.
    try std.testing.expect(try renderer.lineCount() >= after_constraints);

    renderer.clear();
    try std.testing.expectEqual(@as(usize, 0), try renderer.lineCount());
}
