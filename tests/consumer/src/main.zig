//! A downstream consumer of the `zjolt` Zig module.
//!
//! It is the README's quick start, and the README QUOTES THIS FILE:
//! `ci/check-examples.sh` fails the build if the two drift apart. What is
//! under test is that a consumer can reach the module through `b.dependency`
//! at all, that the example in the README compiles and runs, and that the
//! build options zjolt was compiled with are visible from out here.
const std = @import("std");
const zjolt = @import("zjolt");

// Which layers exist, and what collides with what. Plain Zig functions.
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

    const jobs = try zjolt.JobSystem.initThreadPool(.{});
    defer jobs.deinit();

    const system = try zjolt.PhysicsSystem.init(.{ .layers = zjolt.layersFromType(Layers) });
    defer system.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const ball = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 10, 0),
    }, .activate);

    // Per frame:
    var frame: usize = 0;
    while (frame < 60) : (frame += 1) {
        const update_error = try system.step(1.0 / 60.0, 1, jobs);
        if (update_error.contact_constraints_full) return error.ContactConstraintsFull;
    }

    // The ball began at y = 10 and gravity is the whole point, so this is
    // what says the library linked here really stepped.
    const transform = system.bodies().getTransform(ball);
    if (!(transform.position.y < 10)) return error.BallDidNotFall;

    try reportBuild();
}

/// The half a real program would not have: what a consumer can learn about
/// the library it linked, checked here because nothing else can check it.
fn reportBuild() !void {
    // `options` is a separate module the dependency has to export alongside
    // `zjolt` itself, and it is what a consumer branches on to know whether
    // `Real` is 4 or 8 bytes wide. Reaching it from out here is the test.
    if (zjolt.options.double_precision != (@sizeOf(zjolt.Real) == 8)) {
        return error.OptionsDisagreeWithLayout;
    }

    const cpu = zjolt.cpuFeatures();
    std.debug.print(
        "zig consumer ok: zjolt {f}, jolt {f}, real {d} bytes, avx2 {}\n",
        .{ zjolt.version(), zjolt.joltVersion(), @sizeOf(zjolt.Real), cpu.avx2 },
    );
}
