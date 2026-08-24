//! Behavioural tests for saving and restoring simulation state: the
//! determinism property the subsystem exists for, the refusal when the body
//! set has changed since the save, and why `CONSTRAINTS` is its own mask bit
//! rather than folded into `BODIES`.
//!
//! Reuses `integration_test.zig`'s layer map and floor fixture rather than
//! building its own — see `BINDING.md`. `integration_test.zig` already
//! covers the container format (bad magic, truncation, checksum, the
//! save/load pairs refusing each other's buffers); this file is everything
//! past that: what a *valid* save/restore actually does to a running world.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;
const World = fixture.World;

//=============================================================================
// Determinism
//=============================================================================

test "save, step further, then restore reproduces the exact saved positions and velocities" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    var balls: [3]zjolt.BodyId = undefined;
    for (&balls, 0..) |*id, i| {
        const f: f32 = @floatFromInt(i);
        id.* = try bodies.createAndAdd(.{
            .shape = shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(f * 1.2 - 1.2, 5 + f * 0.3, 0),
            .restitution = 0.6,
        }, .activate);
    }

    // Mid-fall/bounce, not yet settled — there is real position and velocity
    // to lose if the restore below turns out to do nothing.
    try world.stepFor(0.8);

    const state = world.system.state();
    const saved = try state.saveAlloc(std.testing.allocator, .all);
    defer std.testing.allocator.free(saved);

    const Sample = struct { pos: zjolt.RVec3, vel: zjolt.Vec3 };
    var reference: [3]Sample = undefined;
    for (balls, 0..) |id, i| {
        reference[i] = .{ .pos = bodies.getPosition(id), .vel = bodies.getLinearVelocity(id) };
    }

    // The world moves on from the save point.
    try world.stepFor(1.5);

    var moved = false;
    for (balls, 0..) |id, i| {
        if (@abs(bodies.getPosition(id).y - reference[i].pos.y) > 0.05) moved = true;
    }
    // It really did move on — otherwise the restore below would be
    // restoring a state indistinguishable from where it already was.
    try std.testing.expect(moved);

    try state.restore(saved);

    for (balls, 0..) |id, i| {
        const pos = bodies.getPosition(id);
        const vel = bodies.getLinearVelocity(id);
        try std.testing.expectApproxEqAbs(reference[i].pos.x, pos.x, 1e-4);
        try std.testing.expectApproxEqAbs(reference[i].pos.y, pos.y, 1e-4);
        try std.testing.expectApproxEqAbs(reference[i].pos.z, pos.z, 1e-4);
        try std.testing.expectApproxEqAbs(reference[i].vel.x, vel.x, 1e-4);
        try std.testing.expectApproxEqAbs(reference[i].vel.y, vel.y, 1e-4);
        try std.testing.expectApproxEqAbs(reference[i].vel.z, vel.z, 1e-4);
    }
}

//=============================================================================
// Refusal when the body set has changed
//=============================================================================

test "restoring into a world whose body set changed is refused, not half-applied" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const keeper = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);
    const doomed = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(3, 5, 0),
    }, .activate);

    try world.stepFor(0.5);

    const state = world.system.state();
    const saved = try state.saveAlloc(std.testing.allocator, .all);
    defer std.testing.allocator.free(saved);

    // The set of bodies the save was taken from no longer exists.
    bodies.destroy(doomed);

    const before = bodies.getPosition(keeper);
    try std.testing.expectError(zjolt.Error.BadFormat, state.restore(saved));
    try std.testing.expect(zjolt.lastError().len > 0);

    // Refused, not half-applied: the survivor was not nudged even part way
    // toward the saved state. The body-set digest is checked before Jolt
    // reads a single byte of the payload — see zjolt_state.cpp — precisely
    // so this comparison can be exact rather than approximate.
    const after = bodies.getPosition(keeper);
    try std.testing.expectEqual(before.x, after.x);
    try std.testing.expectEqual(before.y, after.y);
    try std.testing.expectEqual(before.z, after.z);

    // The world still knows what it actually holds, not what the refused
    // restore might have half-believed: the floor plus the one survivor.
    try std.testing.expectEqual(@as(u32, 2), world.system.numBodies());

    // And a save taken now, of the world as it actually is, restores fine —
    // the refusal above did not wound the system for future saves either.
    const now = try state.saveAlloc(std.testing.allocator, .all);
    defer std.testing.allocator.free(now);
    try state.restore(now);
}

//=============================================================================
// Why CONSTRAINTS is its own bit
//=============================================================================

/// A single-wheel vehicle constraint's own engine RPM is exactly the kind of
/// state `ZJOLT_STATE_RECORDER_STATE_CONSTRAINTS` is for and `BODIES` is
/// not: it belongs to the constraint, not to any body, so nothing about the
/// wheel spinning up shows up in a body's position or velocity at all — a
/// save that leaves CONSTRAINTS out has no way to carry it, and a restore
/// from that save cannot put it back. `ffi/zjolt_vehicle.h` needs nothing
/// beyond `zjoltVehicleConstraintCreate` to make the engine step; see
/// `vehicle_test.zig`'s module doc comment for why the chassis is created
/// with `allow_sleeping = false` regardless (not load-bearing here, since
/// this test never lets the body sit idle long enough to matter, but it is
/// the standing rule for any vehicle body this suite drives).
const VehicleConstraintRun = struct {
    const Result = struct {
        rpm_at_save: f32,
        rpm_after_restore: f32,
    };

    fn once(save_mask: zjolt.StateRecorderState) !Result {
        try zjolt.init(.{ .allocator = std.testing.allocator });
        defer zjolt.deinit();

        var world = try World.init();
        defer world.deinit();

        const chassis_shape = try zjolt.Shape.initBox(zjolt.vec3(0.75, 0.25, 1.75), .{ .density = 300 });
        defer chassis_shape.release();

        const chassis = try world.system.bodies().createAndAdd(.{
            .shape = chassis_shape,
            .object_layer = Layers.moving,
            .position = zjolt.rvec3(0, 1, 0),
            .allow_sleeping = false,
        }, .activate);

        var wheel = zjolt.defaultVehicleWheelDesc();
        wheel.position = zjolt.vec3(0, -0.25, 0);
        const wheels = [_]zjolt.VehicleWheelDesc{wheel};

        var diff = zjolt.defaultVehicleDifferentialDesc();
        diff.left_wheel = 0;
        diff.right_wheel = -1; // no wheel on the right side of this one

        var tester = zjolt.defaultVehicleCollisionTesterDesc();
        tester.object_layer = Layers.moving;

        const vehicle = try zjolt.VehicleConstraint.init(world.system, chassis, .{
            .wheels = &wheels,
            .differentials = &.{diff},
            .collision_tester = tester,
        });
        defer vehicle.deinit();

        try vehicle.setWheeledDriverInput(1.0, 0, 0, 0);
        try world.stepFor(0.5); // The engine has revved up some by here.

        const state = world.system.state();
        const saved = try state.saveAlloc(std.testing.allocator, save_mask);
        defer std.testing.allocator.free(saved);
        const rpm_at_save = vehicle.engineRpm();

        // Keep revving past the save point before restoring into it.
        try world.stepFor(0.5);
        try state.restore(saved);

        return .{ .rpm_at_save = rpm_at_save, .rpm_after_restore = vehicle.engineRpm() };
    }
};

test "excluding CONSTRAINTS from a save loses state BODIES never carried, and restoring with ALL gets it back" {
    const with_all = try VehicleConstraintRun.once(.all);
    // ALL puts the engine back exactly where the save found it.
    try std.testing.expectApproxEqAbs(with_all.rpm_at_save, with_all.rpm_after_restore, 1.0);

    const without_constraints = try VehicleConstraintRun.once(.{
        .global = true,
        .bodies = true,
        .contacts = true,
    });
    // Without CONSTRAINTS, the restore has nowhere to put the engine's RPM
    // back — so it stays at the later frame's value instead of the saved
    // one, mismatched against the chassis body that DID snap back. This is
    // the exact failure zjolt_state.h describes: "a restore without it
    // leaves every constraint warm-started from a different frame than the
    // bodies it acts on."
    try std.testing.expect(
        @abs(without_constraints.rpm_at_save - without_constraints.rpm_after_restore) > 50,
    );
}
