//! Behavioural tests for vehicles: a wheeled car that accelerates, spins its
//! wheels, brakes to a stop; an automatic transmission that upshifts as
//! engine RPM climbs; wheel transform read-back against the suspension
//! length that produced it; and a tracked vehicle that curves when its two
//! tracks are driven at different rates.
//!
//! `zjoltVehicleConstraintCreate` (see `ffi/zjolt_vehicle.h`) both adds the
//! constraint to the system and registers it as a physics step listener in
//! the same call — a vehicle constraint that is not stepped never moves its
//! wheels, but nothing extra is needed here to make that happen: stepping
//! `world.system` is enough.
//!
//! One thing that DOES need doing explicitly: a body that has settled and
//! sat still for `PhysicsSettings::mTimeBeforeSleep` (0.5 s by default) goes
//! to sleep, same as any other dynamic body, and a sleeping body's
//! constraints are not solved — so a vehicle that fell asleep during an
//! idling settle-down does not move no matter what driver input arrives
//! afterward, because nothing ever wakes it back up.
//! `WheeledVehicleController::AllowSleep()` stops an ALREADY-active vehicle
//! from dozing off while it has driver input, but that guard cannot rescue a
//! body that is already asleep before the input arrives. Every rig below
//! creates its chassis with `allow_sleeping = false` for exactly this
//! reason — the alternative, `bodies.activate(chassis)` right before the
//! first driver input, works just as well for a one-shot script but not for
//! the settle-then-drive shape every test here has.
//!
//! Reuses `integration_test.zig`'s layer map and floor fixture rather than
//! building its own — see `BINDING.md`.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;
const World = fixture.World;

//=============================================================================
// A small AWD car
//
// Four wheels at the corners of a box chassis, front and rear differentials
// each taking half the engine's torque so every wheel is driven — the
// simplest layout that cannot leave a wheel silent under forward input.
// suspension_min/max, radius and inertia are all Jolt's own WheelSettings
// defaults (see zjoltVehicleWheelDescInit); only each wheel's position is
// this test's own.
//=============================================================================

const chassis_half_extent = zjolt.vec3(0.75, 0.25, 1.75);
/// Local Y of every suspension attachment point — the bottom of the chassis.
const wheel_hardpoint_y: f32 = -0.25;

fn wheelAt(x: f32, z: f32) zjolt.VehicleWheelDesc {
    var wheel = zjolt.defaultVehicleWheelDesc();
    wheel.position = zjolt.vec3(x, wheel_hardpoint_y, z);
    return wheel;
}

fn differential(left_wheel: i32, right_wheel: i32, engine_torque_ratio: f32) zjolt.VehicleDifferentialDesc {
    var d = zjolt.defaultVehicleDifferentialDesc();
    d.left_wheel = left_wheel;
    d.right_wheel = right_wheel;
    d.engine_torque_ratio = engine_torque_ratio;
    return d;
}

fn collisionTester() zjolt.VehicleCollisionTesterDesc {
    var tester = zjolt.defaultVehicleCollisionTesterDesc();
    // The default (0) is Layers.static, which a wheel ray can never hit: our
    // Layers map only lets a static-layer query hit the moving broad-phase
    // layer (see integration_test.zig). Layers.moving collides with
    // everything, including the static floor, which is what the wheels
    // actually need to test against.
    tester.object_layer = Layers.moving;
    return tester;
}

/// Wheel indices, front-left/front-right/rear-left/rear-right, shared by
/// every rig below so a wheel index in one test means the same corner in
/// another.
const wheel_x: f32 = 0.7;
const wheel_z: f32 = 1.4;

/// A world plus one vehicle on a box chassis. `deinit` order matters: the
/// vehicle constraint holds a raw pointer to the chassis body, so it must be
/// torn down before `world.deinit()` frees that body out from under it.
const Rig = struct {
    world: World,
    chassis_shape: zjolt.Shape,
    chassis: zjolt.BodyId,
    vehicle: zjolt.VehicleConstraint,

    fn deinit(self: *Rig) void {
        self.vehicle.deinit();
        self.world.deinit();
        self.chassis_shape.release();
    }

    fn settle(self: *Rig, seconds: f32) !void {
        try self.world.stepFor(seconds);
    }
};

fn buildWheeledCar(start_y: zjolt.Real) !Rig {
    var world = try World.init();
    errdefer world.deinit();

    // A light-ish chassis (roughly 800 kg) so the engine's default 500 Nm
    // has something to visibly accelerate within a few seconds of test time,
    // rather than a water-density (1000 kg/m^3) block that barely notices it.
    const chassis_shape = try zjolt.Shape.initBox(chassis_half_extent, .{ .density = 300 });
    errdefer chassis_shape.release();

    const bodies = world.system.bodies();
    const chassis = try bodies.createAndAdd(.{
        .shape = chassis_shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, start_y, 0),
        // See the module doc comment: a settled, sleeping chassis ignores
        // driver input forever, because a sleeping body's constraints are
        // not solved. This is the vehicle equivalent of allow_sleeping =
        // false on any other object a test needs to keep responsive.
        .allow_sleeping = false,
    }, .activate);

    // The fixture floor's default friction (0.2, plain BodyDesc default) is
    // asphalt-in-the-rain low. Jolt's own wheel/ground combine is
    // sqrt(tire_friction * ground_friction) (VehicleConstraint.h), so at 0.2
    // even a tire curve peaking at 1.2 only ever combines to ~0.49. A
    // dry-tarmac 1.0 is what the rest of this test is actually about:
    // acceleration and braking, not how little grip the road has.
    bodies.setFriction(world.floor, 1.0);

    const wheels = [_]zjolt.VehicleWheelDesc{
        wheelAt(-wheel_x, wheel_z), // front-left  (0)
        wheelAt(wheel_x, wheel_z), // front-right (1)
        wheelAt(-wheel_x, -wheel_z), // rear-left   (2)
        wheelAt(wheel_x, -wheel_z), // rear-right  (3)
    };
    // Front and rear differentials, half the engine's torque each: an
    // all-wheel-drive layout, so a forward-input test can expect every wheel
    // to spin rather than only naming the two that happen to be driven.
    const differentials = [_]zjolt.VehicleDifferentialDesc{
        differential(0, 1, 0.5),
        differential(2, 3, 0.5),
    };

    const vehicle = try zjolt.VehicleConstraint.init(world.system, chassis, .{
        .wheels = &wheels,
        .differentials = &differentials,
        .collision_tester = collisionTester(),
    });

    raiseVelocitySteps(world.system);
    return .{ .world = world, .chassis_shape = chassis_shape, .chassis = chassis, .vehicle = vehicle };
}

/// Jolt's own default (`PhysicsSettings::mNumVelocitySteps = 10`) is tuned
/// for ordinary rigid-body contacts. A vehicle's wheel is a much stiffer
/// coupling — its own moment of inertia is tiny next to the chassis it is
/// meant to be dragging along — and ten iterations are not enough to fully
/// resolve that stiffness every step: a locked, sliding wheel under hard
/// braking decelerates the chassis correctly at first and then, well before
/// it should, the correction all but stops landing on the chassis at all —
/// not because grip ran out, but because the solver hasn't converged. Thirty
/// iterations is comfortably enough for one small car; a host with many
/// vehicles on screen would want this measured, not copied blind.
fn raiseVelocitySteps(system: zjolt.PhysicsSystem) void {
    var settings = system.getSettings();
    settings.num_velocity_steps = 30;
    system.setSettings(settings) catch unreachable; // only rejects a zero count
}

//=============================================================================
// Wheeled: forward, spin, brake
//=============================================================================

test "forward input accelerates the chassis along its forward axis and spins the wheels; braking brings it back to rest" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();

    // Let the suspension settle before anything is measured, so the
    // acceleration below is not partly the chassis still dropping onto its
    // springs.
    try car.settle(1.5);

    const bodies = car.world.system.bodies();
    const start = bodies.getPosition(car.chassis);

    try car.vehicle.setWheeledDriverInput(1.0, 0, 0, 0);
    try car.settle(3.0);

    const accelerated = bodies.getPosition(car.chassis);
    // Forward is +Z by default (VehicleConstraint.Options.forward), and
    // wheel_forward matches it — so real progress means +Z, not +X or -Z.
    try std.testing.expect(accelerated.z - start.z > 1.0);
    try std.testing.expect(@abs(accelerated.x - start.x) < 0.5);
    try std.testing.expect(bodies.getLinearVelocity(car.chassis).z > 0.5);

    // Every wheel is driven (two differentials, front and rear), so every
    // wheel — not just one corner — reports real rotation speed.
    var wheel_index: u32 = 0;
    while (wheel_index < 4) : (wheel_index += 1) {
        try std.testing.expect(car.vehicle.wheelAngularVelocity(wheel_index) > 1.0);
    }

    // Full brake, no throttle: it comes back to rest rather than coasting or
    // (a stuck brake circuit) staying at speed.
    try car.vehicle.setWheeledDriverInput(0, 0, 1.0, 0);
    try car.settle(5.0);

    try std.testing.expect(@abs(bodies.getLinearVelocity(car.chassis).z) < 0.3);
}

//=============================================================================
// Wheeled: automatic transmission
//=============================================================================

test "the automatic transmission shifts up as engine RPM climbs, and the reported gear follows" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try car.settle(1.5);

    try car.vehicle.setWheeledDriverInput(1.0, 0, 0, 0);

    // Default transmission: auto mode, shift_up_rpm = 4000, starting in
    // neutral (gear 0) with the engine at min_rpm = 1000.
    var peak_rpm_in_first_gear: f32 = 0;
    var reached_second_gear = false;
    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    while (elapsed < 6.0) : (elapsed += dt) {
        _ = try car.world.system.step(dt, 1, car.world.jobs);

        const gear = car.vehicle.currentGear();
        if (gear == 1) {
            peak_rpm_in_first_gear = @max(peak_rpm_in_first_gear, car.vehicle.engineRpm());
        }
        if (gear >= 2) reached_second_gear = true;
    }

    // It revved toward the shift point while still in 1st...
    try std.testing.expect(peak_rpm_in_first_gear > 3000);
    // ...and the transmission actually acted on that: the reported gear
    // changed rather than the engine just bouncing off its rev limiter in
    // 1st for the whole run.
    try std.testing.expect(reached_second_gear);
}

//=============================================================================
// Wheeled: wheel transform read-back
//=============================================================================

test "wheel transform read-back puts each wheel under the chassis at its suspension length" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try car.settle(2.0);

    const bodies = car.world.system.bodies();
    const chassis_position = bodies.getPosition(car.chassis);
    const chassis_rotation = bodies.getRotation(car.chassis);

    const local_x = [_]f32{ -wheel_x, wheel_x, -wheel_x, wheel_x };
    const local_z = [_]f32{ wheel_z, wheel_z, -wheel_z, -wheel_z };

    var wheel_index: u32 = 0;
    while (wheel_index < 4) : (wheel_index += 1) {
        // At rest on flat ground, every wheel should have found the floor.
        try std.testing.expect(car.vehicle.wheelHasContact(wheel_index));

        const length = car.vehicle.wheelSuspensionLength(wheel_index);
        // Jolt's own WheelSettings defaults: suspension_min/max_length.
        try std.testing.expect(length >= 0.3 - 1e-4 and length <= 0.5 + 1e-4);

        // The exact formula zjoltVehicleConstraintGetWheelWorldTransform
        // uses internally (VehicleConstraint::GetWheelLocalTransform): the
        // hardpoint plus the suspension direction (straight down) scaled by
        // the current suspension length, then carried into world space by
        // the chassis's own transform.
        const local = zjolt.vec3(local_x[wheel_index], wheel_hardpoint_y - length, local_z[wheel_index]);
        const rotated = try zjolt.quatRotateVector(chassis_rotation, local);
        const expected_x = chassis_position.x + rotated.x;
        const expected_y = chassis_position.y + rotated.y;
        const expected_z = chassis_position.z + rotated.z;

        const basis = car.vehicle.wheelLocalBasis(wheel_index);
        const pose = car.vehicle.wheelWorldTransform(wheel_index, basis.right, basis.up);

        try std.testing.expectApproxEqAbs(expected_x, pose.position.x, 1e-3);
        try std.testing.expectApproxEqAbs(expected_y, pose.position.y, 1e-3);
        try std.testing.expectApproxEqAbs(expected_z, pose.position.z, 1e-3);

        // Under the chassis, not through it or floating above it.
        try std.testing.expect(pose.position.y < chassis_position.y);
    }
}

//=============================================================================
// Tracked: turning
//=============================================================================

fn buildTrackedCar(start_y: zjolt.Real) !Rig {
    var world = try World.init();
    errdefer world.deinit();

    const chassis_shape = try zjolt.Shape.initBox(chassis_half_extent, .{ .density = 300 });
    errdefer chassis_shape.release();

    const bodies = world.system.bodies();
    const chassis = try bodies.createAndAdd(.{
        .shape = chassis_shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, start_y, 0),
        // See buildWheeledCar / the module doc comment.
        .allow_sleeping = false,
    }, .activate);

    bodies.setFriction(world.floor, 1.0);

    const wheels = [_]zjolt.VehicleWheelDesc{
        wheelAt(-wheel_x, wheel_z), // left track, front  (0)
        wheelAt(-wheel_x, -wheel_z), // left track, rear   (1)
        wheelAt(wheel_x, wheel_z), // right track, front (2)
        wheelAt(wheel_x, -wheel_z), // right track, rear  (3)
    };
    const left_track_wheels = [_]u32{ 0, 1 };
    const right_track_wheels = [_]u32{ 2, 3 };

    const vehicle = try zjolt.VehicleConstraint.init(world.system, chassis, .{
        .controller_kind = .tracked,
        .wheels = &wheels,
        .left_track_wheels = &left_track_wheels,
        .left_track_driven_wheel = 0,
        .right_track_wheels = &right_track_wheels,
        .right_track_driven_wheel = 2,
        .collision_tester = collisionTester(),
    });

    raiseVelocitySteps(world.system);
    return .{ .world = world, .chassis_shape = chassis_shape, .chassis = chassis, .vehicle = vehicle };
}

test "a tracked vehicle curves when its two tracks are driven at different rates" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var tank = try buildTrackedCar(1.0);
    defer tank.deinit();
    try tank.settle(1.5);

    const bodies = tank.world.system.bodies();
    const start = bodies.getPosition(tank.chassis);

    // Both tracks driven forward, but the right one at a third of the
    // left's rate — it should curve to the right rather than track straight
    // down +Z.
    try tank.vehicle.setTrackedDriverInput(1.0, 1.0, 0.35, 0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), tank.vehicle.trackedLeftRatio(), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.35), tank.vehicle.trackedRightRatio(), 1e-5);

    try tank.settle(3.0);

    const after = bodies.getPosition(tank.chassis);
    const dx = after.x - start.x;
    const dz = after.z - start.z;
    // It actually went somewhere, rather than spinning its tracks in place.
    try std.testing.expect(@sqrt(dx * dx + dz * dz) > 0.5);

    // A vehicle driven straight off +Z would show essentially no sideways
    // travel; one curving under differential track rates displaces
    // significantly in X. This is checked on the DISPLACEMENT rather than
    // the final heading angle on purpose: over a long enough curving run the
    // heading can wrap back around past "facing +Z again" while the vehicle
    // has plainly been turning the entire time, which a one-shot angle
    // check at the end would alias back down to near zero.
    try std.testing.expect(@abs(dx) > 1.5);
}
