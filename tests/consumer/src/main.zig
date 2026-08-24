//! A downstream consumer of the `zjolt` Zig module.
//!
//! Deliberately the README's own example rather than anything clever: what is
//! under test is that a consumer can reach the module at all, and that the
//! build options it was compiled with are visible on the other side.
const std = @import("std");
const zjolt = @import("zjolt");

const Layers = struct {
    pub const static: zjolt.ObjectLayer = 0;
    pub const moving: zjolt.ObjectLayer = 1;

    pub const bp_static: zjolt.BroadPhaseLayer = 0;
    pub const bp_moving: zjolt.BroadPhaseLayer = 1;

    pub fn broadPhaseLayerCount() u32 {
        return 2;
    }

    pub fn broadPhaseLayerFor(layer: zjolt.ObjectLayer) zjolt.BroadPhaseLayer {
        return if (layer == static) bp_static else bp_moving;
    }

    pub fn objectCanCollideWithBroadPhase(
        object: zjolt.ObjectLayer,
        broad: zjolt.BroadPhaseLayer,
    ) bool {
        return if (object == static) broad == bp_moving else true;
    }

    pub fn objectsCanCollide(a: zjolt.ObjectLayer, b: zjolt.ObjectLayer) bool {
        return if (a == static) b == moving else true;
    }
};

pub fn main() !void {
    var gpa_state = std.heap.DebugAllocator(.{}){};
    defer std.debug.assert(gpa_state.deinit() == .ok);
    const gpa = gpa_state.allocator();

    try zjolt.init(.{ .allocator = gpa });
    defer zjolt.deinit();

    const jobs = try zjolt.JobSystem.initSingleThreaded(zjolt.c.core.max_physics_jobs);
    defer jobs.deinit();

    const system = try zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(Layers),
        .max_bodies = 1024,
    });
    defer system.deinit();
    system.setGravity(zjolt.gravity_earth);

    const shape = try zjolt.Shape.initSphere(0.5, .{ .density = 1000 });
    defer shape.release();

    const ball = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 10, 0),
    }, .activate);

    var i: usize = 0;
    while (i < 60) : (i += 1) _ = try system.step(1.0 / 60.0, 1, jobs);

    // Gravity is the whole point: a ball left alone for a second must be
    // lower than it started, or the library linked here is not stepping.
    const transform = system.bodies().getTransform(ball);
    if (!(transform.position.y < 10)) return error.BallDidNotFall;

    // `options` is a separate module the dependency has to export alongside
    // `zjolt` itself, and it is what a consumer branches on to know whether
    // `Real` is 4 or 8 bytes wide. Reaching it from out here is the test.
    if (zjolt.options.double_precision != (@sizeOf(zjolt.Real) == 8)) {
        return error.OptionsDisagreeWithLayout;
    }

    std.debug.print(
        "zig consumer ok: zjolt {f}, jolt {f}, real {d} bytes\n",
        .{ zjolt.version(), zjolt.joltVersion(), @sizeOf(zjolt.Real) },
    );
}
