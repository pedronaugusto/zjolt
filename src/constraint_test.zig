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
// Reading a constraint's frame back
//=============================================================================

test "a hinge reports back the frame it was built from, and its constraint-to-body matrix is that frame" {
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

    // A frame nobody defaults to: the hinge turns about +Z and measures its
    // angle from +Y. Jolt's own defaults are (0,1,0) and (1,0,0), so a getter
    // that quietly reported those instead would be caught here.
    const hinge_axis = zjolt.vec3(0, 0, 1);
    const normal_axis = zjolt.vec3(0, 1, 0);
    var hinge = try zjolt.Constraint.initHinge(world.system, zjolt.world_body, arm, .{
        .point1 = anchor,
        .point2 = anchor,
        .hinge_axis1 = hinge_axis,
        .normal_axis1 = normal_axis,
        .hinge_axis2 = hinge_axis,
        .normal_axis2 = normal_axis,
    });
    defer hinge.release();
    try hinge.addTo(world.system);

    // The descriptor was in WORLD space. What comes back is centre-of-mass
    // space, which is the point of the getters: body 2 is a box at `anchor`
    // with identity rotation, so its centre of mass IS `anchor` and the same
    // frame reads back sitting at the origin.
    try expectVec3(hinge_axis, try hinge.hingeLocalSpaceHingeAxis2(), 1e-5);
    try expectVec3(normal_axis, try hinge.hingeLocalSpaceNormalAxis2(), 1e-5);
    try expectVec3(zjolt.vec3(0, 0, 0), try hinge.hingeLocalSpacePoint2(), 1e-5);

    // Body 1 is the world body, whose centre of mass is the origin — so the
    // same frame reads back sitting at `anchor` instead. The two differing is
    // exactly what makes these worth having.
    try expectVec3(hinge_axis, try hinge.hingeLocalSpaceHingeAxis1(), 1e-5);
    try expectVec3(normal_axis, try hinge.hingeLocalSpaceNormalAxis1(), 1e-5);
    try expectVec3(zjolt.vec3(0, 4, 0), try hinge.hingeLocalSpacePoint1(), 1e-5);

    // The matrix is that frame, column by column, in the column-major layout
    // ZJoltMat44 documents: hinge axis, normal axis, their cross, then the
    // attachment point.
    const m = try hinge.constraintToBody2Matrix();
    try expectVec3(hinge_axis, zjolt.vec3(m.m[0], m.m[1], m.m[2]), 1e-5);
    try expectVec3(normal_axis, zjolt.vec3(m.m[4], m.m[5], m.m[6]), 1e-5);
    try expectVec3(zjolt.vec3(-1, 0, 0), zjolt.vec3(m.m[8], m.m[9], m.m[10]), 1e-5);
    try expectVec3(zjolt.vec3(0, 0, 0), zjolt.vec3(m.m[12], m.m[13], m.m[14]), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), m.m[15], 1e-5);

    // And on body 1 the translation column is the attachment point there.
    const m1 = try hinge.constraintToBody1Matrix();
    try expectVec3(zjolt.vec3(0, 4, 0), zjolt.vec3(m1.m[12], m1.m[13], m1.m[14]), 1e-5);
}

test "the constraint-to-body matrix is refused on a handle that is not a two-body constraint" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const box = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    var point = try zjolt.Constraint.initPoint(world.system, zjolt.world_body, box, .{
        .point1 = anchor,
        .point2 = anchor,
    });
    defer point.release();

    // A point constraint IS a two-body constraint, so this succeeds — the
    // narrowing is not a blanket refusal of the kinds that have no meaningful
    // rotation, only of handles that are not two-body at all.
    _ = try point.constraintToBody1Matrix();

    // ...but the per-kind accessors still refuse the wrong kind, which is what
    // keeps a hinge accessor off a point constraint.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        point.hingeLocalSpaceHingeAxis1(),
    );
}

//=============================================================================
// Draw size
//=============================================================================

test "a constraint's draw size round-trips, or refuses honestly without the debug renderer" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const box = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    var point = try zjolt.Constraint.initPoint(world.system, zjolt.world_body, box, .{
        .point1 = anchor,
        .point2 = anchor,
    });
    defer point.release();

    if (zjolt.options.debug_renderer) {
        // Jolt's own default, which this reads rather than transcribes.
        try std.testing.expectApproxEqAbs(@as(f32, 1), try point.drawSize(), 1e-5);
        try point.setDrawSize(2.5);
        try std.testing.expectApproxEqAbs(@as(f32, 2.5), try point.drawSize(), 1e-5);
        // A NaN size would draw a frame with no bounds, which a renderer that
        // culls by bounds silently never shows.
        try std.testing.expectError(
            zjolt.Error.InvalidArgument,
            point.setDrawSize(std.math.nan(f32)),
        );
        try std.testing.expectApproxEqAbs(@as(f32, 2.5), try point.drawSize(), 1e-5);
    } else {
        // Declared in every build, and honest about the one it cannot serve.
        try std.testing.expectError(zjolt.Error.Unsupported, point.setDrawSize(2.5));
        try std.testing.expectError(zjolt.Error.Unsupported, point.drawSize());
    }
}

//=============================================================================
// Limit impulses — what a breakable joint is built from
//=============================================================================

test "a slider's position limit clamps travel, and the limit impulse says so" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.3, 0.3), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const bodies = world.system.bodies();
    const carriage = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    const limit: f32 = 0.5;
    var slider = try zjolt.Constraint.initSlider(world.system, zjolt.world_body, carriage, .{
        .point1 = anchor,
        .slider_axis1 = zjolt.vec3(1, 0, 0),
        .normal_axis1 = zjolt.vec3(0, 1, 0),
        .point2 = anchor,
        .slider_axis2 = zjolt.vec3(1, 0, 0),
        .normal_axis2 = zjolt.vec3(0, 1, 0),
        .limits_min = -limit,
        .limits_max = limit,
    });
    defer slider.release();
    try slider.addTo(world.system);

    // Gravity is left on and pulls perpendicular to the slider axis, so the
    // AXIS impulse is carrying the carriage's weight from the first step.
    try world.stepFor(0.3);
    try std.testing.expect(@abs(try slider.sliderCurrentPosition()) < 0.05);

    const held = try slider.sliderTotalLambdaPosition();
    try std.testing.expect(@abs(held[0]) + @abs(held[1]) > 0);

    // Nothing is pushing along the axis yet, so the LIMIT part has applied
    // nothing — which is the half of this that a getter returning the wrong
    // constraint part would fail.
    try std.testing.expectApproxEqAbs(
        @as(f32, 0),
        try slider.sliderTotalLambdaPositionLimits(),
        1e-6,
    );

    // Now drive it hard into the stop.
    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 1.0) : (elapsed += dt) {
        bodies.setLinearVelocity(carriage, zjolt.vec3(4, 0, 0));
        _ = try world.system.step(dt, 1, world.jobs);
    }

    // Travel actually clamped: four metres per second for a second, stopped
    // half a metre out.
    const position = try slider.sliderCurrentPosition();
    try std.testing.expect(position > limit - 0.05);
    try std.testing.expect(position < limit + 0.05);

    // And the limit is what stopped it.
    try std.testing.expect(@abs(try slider.sliderTotalLambdaPositionLimits()) > 0);
}

test "a swing-twist's twist limit stops rotation past it, and only the twist impulse rises" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();
    world.system.setGravity(zjolt.vec3_zero);

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.4, 0.1, 0.1), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const bodies = world.system.bodies();
    const bone = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    const twist_limit: f32 = 0.3;
    var joint = try zjolt.Constraint.initSwingTwist(world.system, zjolt.world_body, bone, .{
        .position1 = anchor,
        .twist_axis1 = zjolt.vec3(1, 0, 0),
        .plane_axis1 = zjolt.vec3(0, 1, 0),
        .position2 = anchor,
        .twist_axis2 = zjolt.vec3(1, 0, 0),
        .plane_axis2 = zjolt.vec3(0, 1, 0),
        // A roomy swing cone, so the swing limits are nowhere near engaged and
        // their impulses stay at zero while the twist limit does all the work.
        .normal_half_cone_angle = 1.0,
        .plane_half_cone_angle = 1.0,
        .twist_min_angle = -twist_limit,
        .twist_max_angle = twist_limit,
    });
    defer joint.release();
    try joint.addTo(world.system);

    // Inside every limit and nothing pushing: no limit has applied anything.
    try world.stepFor(0.1);
    try std.testing.expectApproxEqAbs(@as(f32, 0), try joint.swingTwistTotalLambdaTwist(), 1e-6);

    // Spin it about the twist axis, hard and for long enough that an
    // unconstrained bone would have turned several times round.
    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 1.0) : (elapsed += dt) {
        bodies.setAngularVelocity(bone, zjolt.vec3(6, 0, 0));
        _ = try world.system.step(dt, 1, world.jobs);
    }

    // The limit held: body 1 is the world at identity, so the bone's world
    // rotation is the joint's rotation, and its total angle is the twist.
    const rotation = bodies.getRotation(bone);
    try std.testing.expect(quatAngle(rotation, zjolt.quat_identity) < twist_limit + 0.1);

    // The TWIST limit is what held it...
    try std.testing.expect(@abs(try joint.swingTwistTotalLambdaTwist()) > 0);
    // ...and the swing limits, well inside their cone, applied nothing. That
    // is what says these three read three different constraint parts rather
    // than the same one three times.
    try std.testing.expectApproxEqAbs(@as(f32, 0), try joint.swingTwistTotalLambdaSwingY(), 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), try joint.swingTwistTotalLambdaSwingZ(), 1e-6);
}

test "a hinge driven past its limit reports the limit impulse, not just the axis one" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();
    world.system.setGravity(zjolt.vec3_zero);

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.1, 0.1), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const bodies = world.system.bodies();
    const door = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    const limit: f32 = 0.4;
    var hinge = try zjolt.Constraint.initHinge(world.system, zjolt.world_body, door, .{
        .point1 = anchor,
        .point2 = anchor,
        .hinge_axis1 = zjolt.vec3(0, 1, 0),
        .normal_axis1 = zjolt.vec3(1, 0, 0),
        .hinge_axis2 = zjolt.vec3(0, 1, 0),
        .normal_axis2 = zjolt.vec3(1, 0, 0),
        .limits_min = -limit,
        .limits_max = limit,
    });
    defer hinge.release();
    try hinge.addTo(world.system);

    try world.stepFor(0.1);
    try std.testing.expectApproxEqAbs(
        @as(f32, 0),
        try hinge.hingeTotalLambdaRotationLimits(),
        1e-6,
    );

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 1.0) : (elapsed += dt) {
        // About the hinge axis, so only the LIMIT can stop it — the axis part
        // has nothing to resist.
        bodies.setAngularVelocity(door, zjolt.vec3(0, 5, 0));
        _ = try world.system.step(dt, 1, world.jobs);
    }

    try std.testing.expectApproxEqAbs(limit, try hinge.hingeCurrentAngle(), 0.1);
    try std.testing.expect(@abs(try hinge.hingeTotalLambdaRotationLimits()) > 0);
}

test "a path constraint's position impulse is what holds the cart on the track" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.2, 0.2, 0.2), .{});
    defer shape.release();

    // A straight run along +X, three control points so the middle one has a
    // segment either side.
    const tangent = zjolt.vec3(2, 0, 0);
    const normal = zjolt.vec3(0, 1, 0);
    const path = try zjolt.ConstraintPath.initHermite(&[_]zjolt.PathPoint{
        .{ .position = zjolt.vec3(-2, 0, 0), .tangent = tangent, .normal = normal },
        .{ .position = zjolt.vec3(0, 0, 0), .tangent = tangent, .normal = normal },
        .{ .position = zjolt.vec3(2, 0, 0), .tangent = tangent, .normal = normal },
    }, false);
    defer path.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const bodies = world.system.bodies();
    const cart = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    var track = try zjolt.Constraint.initPath(world.system, zjolt.world_body, cart, .{
        .path = path.handle,
        .path_position = zjolt.vec3(0, 4, 0),
        .path_rotation = zjolt.quat_identity,
        .path_fraction = 1,
        .rotation_constraint_type = .free,
    });
    defer track.release();
    try track.addTo(world.system);

    // Gravity pulls the cart off the track the whole time; the position part
    // is what keeps it on.
    try world.stepFor(0.5);

    const held = try track.pathTotalLambdaPosition();
    try std.testing.expect(@abs(held[0]) + @abs(held[1]) > 0);

    // The cart stayed on the line the path runs along.
    const position = bodies.getPosition(cart);
    try std.testing.expectApproxEqAbs(@as(f64, 4), @as(f64, @floatCast(position.y)), 0.05);
    try std.testing.expectApproxEqAbs(@as(f64, 0), @as(f64, @floatCast(position.z)), 0.05);

    // Rotation is free on this track, so its rotation parts applied nothing —
    // and both shapes of the rotation impulse are safe to sample regardless.
    const hinge_lambda = try track.pathTotalLambdaRotationHinge();
    try std.testing.expectApproxEqAbs(@as(f32, 0), hinge_lambda[0], 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0), hinge_lambda[1], 1e-6);
    const rotation_lambda = try track.pathTotalLambdaRotation();
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotation_lambda.x, 1e-6);
}

//=============================================================================
// Body space against constraint space
//=============================================================================

test "a body-space target orientation drives a swing-twist to that relative rotation" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();
    world.system.setGravity(zjolt.vec3_zero);

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.4, 0.1, 0.1), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const bodies = world.system.bodies();
    const bone = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    // The constraint frame is deliberately NOT the body frame — twist about
    // +Z, plane axis +X — so constraint space and body space differ and
    // passing the same quaternion to the two setters cannot give the same
    // result. With an identity frame this test would pass without the
    // conversion being done at all.
    var joint = try zjolt.Constraint.initSwingTwist(world.system, zjolt.world_body, bone, .{
        .position1 = anchor,
        .twist_axis1 = zjolt.vec3(0, 0, 1),
        .plane_axis1 = zjolt.vec3(1, 0, 0),
        .position2 = anchor,
        .twist_axis2 = zjolt.vec3(0, 0, 1),
        .plane_axis2 = zjolt.vec3(1, 0, 0),
        .normal_half_cone_angle = 1.0,
        .plane_half_cone_angle = 1.0,
        .twist_min_angle = -1.0,
        .twist_max_angle = 1.0,
    });
    defer joint.release();
    try joint.addTo(world.system);

    const to_body1 = try joint.swingTwistConstraintToBody1();
    const to_body2 = try joint.swingTwistConstraintToBody2();
    // The frame really is rotated, or the rest of this proves nothing.
    try std.testing.expect(quatAngle(to_body1, zjolt.quat_identity) > 0.5);

    // Body 1 is the world at identity, so this is where the bone should end up
    // in world space too.
    const target = try zjolt.quatFromAxisAngle(zjolt.vec3(0, 0, 1), 0.4);
    try joint.swingTwistSetTargetOrientationBodySpace(target);

    // Stored in CONSTRAINT space: conj(toBody1) * target * toBody2, which is
    // not the quaternion that went in.
    const stored = try joint.swingTwistTargetOrientation();
    const expected = zjolt.quatMultiply(
        zjolt.quatMultiply(zjolt.quatConjugate(to_body1), target),
        to_body2,
    );
    try std.testing.expect(quatAngle(stored, expected) < 1e-3);
    try std.testing.expect(quatAngle(stored, target) > 1e-3);

    // A position motor is a spring: the default MotorSettings has frequency 0
    // and therefore applies nothing however large its torque limits are.
    const motor: zjolt.MotorSettings = .{ .spring = .{
        .mode = .frequency_and_damping,
        .frequency_or_stiffness = 8,
        .damping = 1,
    } };
    try joint.swingTwistSetSwingMotorSettings(motor);
    try joint.swingTwistSetTwistMotorSettings(motor);
    try joint.swingTwistSetSwingMotorState(.position);
    try joint.swingTwistSetTwistMotorState(.position);

    try world.stepFor(2.0);

    // It arrived where BODY space said, not where treating that quaternion as
    // constraint space would have put it.
    try std.testing.expect(quatAngle(bodies.getRotation(bone), target) < 0.1);
}

test "a body-space angular velocity target is converted, not stored as given" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();
    world.system.setGravity(zjolt.vec3_zero);

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.4, 0.1, 0.1), .{});
    defer shape.release();

    const anchor = zjolt.rvec3(0, 4, 0);
    const bone = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = anchor,
    }, .activate);

    var joint = try zjolt.Constraint.initSwingTwist(world.system, zjolt.world_body, bone, .{
        .position1 = anchor,
        .twist_axis1 = zjolt.vec3(0, 0, 1),
        .plane_axis1 = zjolt.vec3(1, 0, 0),
        .position2 = anchor,
        .twist_axis2 = zjolt.vec3(0, 0, 1),
        .plane_axis2 = zjolt.vec3(1, 0, 0),
        .normal_half_cone_angle = 1.0,
        .plane_half_cone_angle = 1.0,
        .twist_min_angle = -1.0,
        .twist_max_angle = 1.0,
    });
    defer joint.release();
    try joint.addTo(world.system);

    // Body space: turn about the bone's own +Z, which the constraint frame
    // maps to its twist axis (+X in constraint space).
    const body_space = zjolt.vec3(0, 0, 2);
    try joint.swingTwistSetTargetAngularVelocityBodySpace(body_space);

    // What comes back is the CONVERTED value — there is one target and it is
    // held in constraint space, so the getter cannot echo what was passed.
    const stored = try joint.swingTwistTargetAngularVelocity();
    try std.testing.expectApproxEqAbs(@as(f32, 2), stored.x, 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0), stored.z, 1e-4);

    // The constraint-space setter with the same numbers stores them verbatim,
    // which is the difference the two calls exist for.
    try joint.swingTwistSetTargetAngularVelocity(body_space);
    const verbatim = try joint.swingTwistTargetAngularVelocity();
    try std.testing.expectApproxEqAbs(@as(f32, 0), verbatim.x, 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 2), verbatim.z, 1e-4);
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

fn expectVec3(expected: zjolt.Vec3, actual: zjolt.Vec3, tolerance: f32) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, tolerance);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, tolerance);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, tolerance);
}
