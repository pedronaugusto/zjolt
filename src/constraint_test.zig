//! Behavioural tests for constraints, beyond the hinge-axis test that lives
//! in `constraint.zig` itself: a distance constraint under gravity, a
//! slider's single free axis, a position motor driving a hinge to a target,
//! and the precondition that keeps a bad limit call from ever reaching
//! Jolt's own assert.
//!
//! Reuses `integration_test.zig`'s layer map and floor fixture rather than
//! building its own — see `BINDING.md`.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;
const World = fixture.World;

//=============================================================================
// Distance
//=============================================================================

test "a distance constraint holds two bodies at a fixed separation under gravity" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.3, .{});
    defer shape.release();

    // A pendulum: the ball starts level with the anchor and already three
    // metres away from it, so the rod is at its fixed length from the first
    // step and gravity is the only thing left to disturb it.
    const anchor = zjolt.rvec3(0, 10, 0);
    const start = zjolt.rvec3(3, 10, 0);
    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = start,
    }, .activate);

    var rod = try zjolt.Constraint.initDistance(world.system, zjolt.world_body, ball, .{
        .point1 = anchor,
        .point2 = start,
        .min_distance = 3,
        .max_distance = 3,
    });
    defer rod.release();
    try rod.addTo(world.system);

    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    var fell = false;
    while (elapsed < 2.0) : (elapsed += dt) {
        _ = try world.system.step(dt, 1, world.jobs);

        const position = bodies.getPosition(ball);
        const dx: f64 = @floatCast(position.x - anchor.x);
        const dy: f64 = @floatCast(position.y - anchor.y);
        const dz: f64 = @floatCast(position.z - anchor.z);
        const distance = @sqrt(dx * dx + dy * dy + dz * dz);

        // Gravity is trying to change this every single step; holding it is
        // the whole job of the constraint.
        try std.testing.expectApproxEqAbs(@as(f64, 3), distance, 0.15);
        if (@as(f64, @floatCast(position.y)) < @as(f64, @floatCast(start.y)) - 0.2) fell = true;
    }

    // It actually swung — a rod that never moved would pass the distance
    // check by doing nothing at all.
    try std.testing.expect(fell);
}

//=============================================================================
// Slider
//=============================================================================

test "a slider permits translation along one axis only" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();
    // No gravity: this is about what the slider allows, not about falling.
    world.system.setGravity(zjolt.vec3_zero);

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const bodies = world.system.bodies();
    const piston = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    var slider = try zjolt.Constraint.initSlider(world.system, zjolt.world_body, piston, .{
        .point1 = anchor,
        .slider_axis1 = zjolt.vec3(1, 0, 0),
        .normal_axis1 = zjolt.vec3(0, 1, 0),
        .point2 = anchor,
        .slider_axis2 = zjolt.vec3(1, 0, 0),
        .normal_axis2 = zjolt.vec3(0, 1, 0),
    });
    defer slider.release();
    try slider.addTo(world.system);
    try std.testing.expectEqual(zjolt.ConstraintSubType.slider, slider.subType());

    // Push it off in every direction at once, linearly and angularly, so a
    // leak on any one of the five locked degrees of freedom would show up.
    bodies.setLinearVelocity(piston, zjolt.vec3(2, 3, 5));
    bodies.setAngularVelocity(piston, zjolt.vec3(4, 4, 4));

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 1.0) : (elapsed += dt) {
        _ = try world.system.step(dt, 1, world.jobs);
    }

    const position = bodies.getPosition(piston);
    const dx: f64 = @floatCast(position.x - anchor.x);
    const dy: f64 = @floatCast(position.y - anchor.y);
    const dz: f64 = @floatCast(position.z - anchor.z);

    // Free along the slider axis...
    try std.testing.expect(@abs(dx) > 1.0);
    // ...and pinned on the other two translation axes.
    try std.testing.expectApproxEqAbs(@as(f64, 0), dy, 0.05);
    try std.testing.expectApproxEqAbs(@as(f64, 0), dz, 0.05);

    // A slider solves its rotation constraint from body 1 and removes all
    // three degrees of it, not just the two translations off the slider
    // axis — so the angular push above must have gone nowhere either.
    const rotation = bodies.getRotation(piston);
    try std.testing.expect(quatAngle(rotation, zjolt.quat_identity) < 0.05);
}

//=============================================================================
// Motor
//=============================================================================

test "a position motor drives a hinge toward its target angle" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();
    world.system.setGravity(zjolt.vec3_zero);

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.1, 0.1), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const bodies = world.system.bodies();
    const arm = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    var hinge = try zjolt.Constraint.initHinge(world.system, zjolt.world_body, arm, .{
        .point1 = anchor,
        .point2 = anchor,
        .hinge_axis1 = zjolt.vec3(0, 1, 0),
        .normal_axis1 = zjolt.vec3(1, 0, 0),
        .hinge_axis2 = zjolt.vec3(0, 1, 0),
        .normal_axis2 = zjolt.vec3(1, 0, 0),
    });
    defer hinge.release();
    try hinge.addTo(world.system);

    try std.testing.expectApproxEqAbs(@as(f32, 0), try hinge.hingeCurrentAngle(), 1e-4);

    const target: f32 = 0.6;
    // A position motor is a SPRING pulling toward the target, so a default
    // MotorSettings — frequency 0 — is a motor that applies nothing however
    // large its torque limits are. This is the setting the test is really
    // about; without it the arm sits at 0 and every other assertion below
    // still passes.
    try hinge.hingeSetMotorSettings(.{ .spring = .{
        .mode = .frequency_and_damping,
        .frequency_or_stiffness = 2,
        .damping = 1,
    } });
    try hinge.hingeSetMotorState(.position);
    try hinge.hingeSetTargetAngle(target);
    try std.testing.expectEqual(zjolt.MotorState.position, try hinge.hingeMotorState());
    try std.testing.expectApproxEqAbs(target, try hinge.hingeTargetAngle(), 1e-4);

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 1.5) : (elapsed += dt) {
        _ = try world.system.step(dt, 1, world.jobs);
    }

    // It moved toward the target and got close, rather than drifting away
    // from it or staying put.
    try std.testing.expectApproxEqAbs(target, try hinge.hingeCurrentAngle(), 0.1);
}

//=============================================================================
// Preconditions
//=============================================================================

/// Records Jolt assertions instead of breaking on them, so a refusal that
/// happens BEFORE reaching Jolt can be told apart from one that only avoided
/// crashing by the skin of its teeth. A local copy of the one in
/// `integration_test.zig` rather than an import — see `character.zig`'s
/// `Failure` for the same call: every subsystem file here stands on its own.
const AssertSink = struct {
    var count: u32 = 0;

    fn reset() void {
        count = 0;
    }

    fn onAssert(
        user: ?*anyopaque,
        expression: [*:0]const u8,
        message: ?[*:0]const u8,
        file: [*:0]const u8,
        line: u32,
    ) callconv(.c) bool {
        _ = .{ user, expression, message, file, line };
        count += 1;
        return false; // do not break
    }
};

test "a hinge's limits outside [-pi, 0] / [0, pi] are refused, not asserted" {
    AssertSink.reset();
    try zjolt.init(.{
        .allocator = std.testing.allocator,
        .assert_failed = AssertSink.onAssert,
    });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const door = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    var hinge = try zjolt.Constraint.initHinge(world.system, zjolt.world_body, door, .{
        .point1 = anchor,
        .point2 = anchor,
        .hinge_axis1 = zjolt.vec3(0, 1, 0),
        .normal_axis1 = zjolt.vec3(1, 0, 0),
        .hinge_axis2 = zjolt.vec3(0, 1, 0),
        .normal_axis2 = zjolt.vec3(1, 0, 0),
    });
    defer hinge.release();

    // min must be in [-pi, 0]; this min is positive.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        hinge.hingeSetLimits(0.1, 1.0),
    );
    try std.testing.expect(zjolt.lastError().len > 0);

    // max must be in [0, pi]; this max is past it.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        hinge.hingeSetLimits(-1.0, 4.0),
    );

    // Refused by this wrapper's own check before ever reaching
    // HingeConstraint::SetLimits — so Jolt's assert, which that call would
    // otherwise trip, was never in the picture.
    try std.testing.expectEqual(@as(u32, 0), AssertSink.count);

    // The same precondition holds at construction time, not just on the
    // setter.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        zjolt.Constraint.initHinge(world.system, zjolt.world_body, door, .{
            .point1 = anchor,
            .point2 = anchor,
            .hinge_axis1 = zjolt.vec3(0, 1, 0),
            .normal_axis1 = zjolt.vec3(1, 0, 0),
            .hinge_axis2 = zjolt.vec3(0, 1, 0),
            .normal_axis2 = zjolt.vec3(1, 0, 0),
            .limits_min = 0.1,
            .limits_max = 1.0,
        }),
    );
    try std.testing.expectEqual(@as(u32, 0), AssertSink.count);

    // A valid pair is not swept up in the refusal.
    try hinge.hingeSetLimits(-1.0, 1.0);
    const limits = try hinge.hingeLimits();
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), limits.min, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), limits.max, 1e-5);

    // Refused, not wounded: the constraint and the system both still work.
    try hinge.addTo(world.system);
    try world.stepFor(0.1);
}

//=============================================================================
// Helpers
//=============================================================================

/// The angle between two rotations, in radians. `std.math.acos` is only
/// defined on [-1, 1]; a dot product of two unit quaternions can land a hair
/// outside that from float error, hence the clamp.
fn quatAngle(a: zjolt.Quat, b: zjolt.Quat) f32 {
    const dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    const clamped = std.math.clamp(@abs(dot), 0.0, 1.0);
    return 2.0 * std.math.acos(clamped);
}
