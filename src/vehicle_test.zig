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
    return buildWheeledCarWithBars(start_y, &.{});
}

fn buildWheeledCarWithBars(
    start_y: zjolt.Real,
    anti_roll_bars: []const zjolt.VehicleAntiRollBarDesc,
) !Rig {
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
        .anti_roll_bars = anti_roll_bars,
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
    {
        const v = car.vehicle;
        var w: u32 = 0;
        while (w < 4) : (w += 1) {
            std.debug.print("PROBE w{d} contact={} susp={d:.4} n=({d:.3},{d:.3},{d:.3}) av={d:.2}\n", .{
                w,                         v.wheelHasContact(w),      v.wheelSuspensionLength(w),
                v.wheelContactNormal(w).x, v.wheelContactNormal(w).y, v.wheelContactNormal(w).z,
                v.wheelAngularVelocity(w),
            });
        }
    }
    try car.vehicle.setWheeledDriverInput(0, 0, 1.0, 0);
    try car.settle(5.0);
    const after_braking = @abs(bodies.getLinearVelocity(car.chassis).z);

    // A dead stop, in every build configuration. This asserted a loose
    // fraction once, because the double-precision build appeared to brake far
    // worse than the float one — which turned out to be the car driving off
    // the end of a floor that was only 50 m ahead of it, free-falling with
    // every wheel reporting no contact and nothing to brake against. The
    // floor is 200 m each way now; the brakes always worked.
    try std.testing.expect(after_braking < 0.05);
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

//=============================================================================
// Anti-roll bars
//
// A bar couples the suspensions of the two wheels it names: the further apart
// they are compressed, the harder it pushes to even them out. Under a rolling
// load that shows up as the two suspension lengths staying closer together,
// which is measured directly here rather than through the chassis angle it
// eventually produces.
//=============================================================================

fn antiRollBar(left_wheel: i32, right_wheel: i32, stiffness: f32) zjolt.VehicleAntiRollBarDesc {
    var bar = zjolt.defaultVehicleAntiRollBarDesc();
    bar.left_wheel = left_wheel;
    bar.right_wheel = right_wheel;
    bar.stiffness = stiffness;
    return bar;
}

/// Rolls the chassis with a constant torque about its forward axis and
/// returns the largest front-axle suspension-length difference it reached.
/// The torque has to be re-applied every step: Jolt clears accumulated forces
/// at the end of each one.
fn peakFrontSuspensionSplit(car: *Rig, roll_torque: f32, seconds: f32) !f32 {
    const bodies = car.world.system.bodies();
    const dt: f32 = 1.0 / 60.0;
    var elapsed: f32 = 0;
    var peak: f32 = 0;
    while (elapsed < seconds) : (elapsed += dt) {
        bodies.addTorque(car.chassis, zjolt.vec3(0, 0, roll_torque));
        _ = try car.world.system.step(dt, 1, car.world.jobs);
        const split = @abs(car.vehicle.wheelSuspensionLength(0) - car.vehicle.wheelSuspensionLength(1));
        peak = @max(peak, split);
    }
    return peak;
}

test "a stiffer anti-roll bar keeps the two suspensions on an axle closer together under roll" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const bars = [_]zjolt.VehicleAntiRollBarDesc{
        antiRollBar(0, 1, 0), // front
        antiRollBar(2, 3, 0), // rear
    };

    const roll_torque: f32 = 12_000;

    // Both runs start from the same rig and the same settle, and differ only
    // in what setAntiRollBarStiffness was called with — so the comparison is
    // of the bar, not of two slightly different cars.
    const limp = blk: {
        var car = try buildWheeledCarWithBars(1.0, &bars);
        defer car.deinit();
        try car.settle(1.5);
        break :blk try peakFrontSuspensionSplit(&car, roll_torque, 1.5);
    };

    const stiff = blk: {
        var car = try buildWheeledCarWithBars(1.0, &bars);
        defer car.deinit();
        try car.settle(1.5);
        try car.vehicle.setAntiRollBarStiffness(0, 60_000);
        try car.vehicle.setAntiRollBarStiffness(1, 60_000);
        break :blk try peakFrontSuspensionSplit(&car, roll_torque, 1.5);
    };

    // The limp bar has to let the axle twist at all, or there is nothing for
    // the stiff one to have prevented.
    try std.testing.expect(limp > 0.02);
    try std.testing.expect(stiff < limp);
}

test "anti-roll bars read back after creation, and stiffness is the one field that can be retuned" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const bars = [_]zjolt.VehicleAntiRollBarDesc{
        antiRollBar(0, 1, 1500),
        antiRollBar(2, 3, 900),
    };
    var car = try buildWheeledCarWithBars(1.0, &bars);
    defer car.deinit();

    try std.testing.expectEqual(@as(u32, 2), car.vehicle.antiRollBarCount());

    const front = car.vehicle.antiRollBar(0).?;
    try std.testing.expectEqual(@as(i32, 0), front.left_wheel);
    try std.testing.expectEqual(@as(i32, 1), front.right_wheel);
    try std.testing.expectApproxEqAbs(@as(f32, 1500), front.stiffness, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 900), car.vehicle.antiRollBar(1).?.stiffness, 1e-3);

    // Past the count is null rather than a zeroed descriptor, which would
    // read as a real bar joining wheel 0 to wheel 0.
    try std.testing.expect(car.vehicle.antiRollBar(2) == null);

    try car.vehicle.setAntiRollBarStiffness(1, 4000);
    try std.testing.expectApproxEqAbs(@as(f32, 4000), car.vehicle.antiRollBar(1).?.stiffness, 1e-3);

    // Jolt asserts stiffness >= 0 every step it runs; a negative bar would
    // push the two suspensions apart rather than together.
    try std.testing.expectError(error.InvalidArgument, car.vehicle.setAntiRollBarStiffness(0, -1));
    try std.testing.expectError(error.InvalidArgument, car.vehicle.setAntiRollBarStiffness(2, 100));
}

//=============================================================================
// Differentials
//
// Driven in the air, with gravity overridden away, so the only thing deciding
// which wheels turn is how the engine's torque was split — on the ground the
// road turns every wheel regardless.
//=============================================================================

test "retuning a differential's engine torque ratio moves the drive between axles" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();

    try std.testing.expectEqual(@as(u32, 2), car.vehicle.differentialCount());
    const front = car.vehicle.differential(0).?;
    try std.testing.expectEqual(@as(i32, 0), front.left_wheel);
    try std.testing.expectEqual(@as(i32, 1), front.right_wheel);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), front.engine_torque_ratio, 1e-5);
    try std.testing.expect(car.vehicle.differential(2) == null);

    // Hovering: no wheel touches anything, so a wheel that turns is a wheel
    // the drivetrain turned.
    car.vehicle.overrideGravity(zjolt.vec3(0, 0, 0));

    // The vehicle-level limited slip ratio couples the two DIFFERENTIALS to
    // each other, so at its default 1.4 a spinning rear axle drags the front
    // one round with it whatever the torque split says. Opening it is what
    // makes the split the only thing left deciding which wheels turn.
    try car.vehicle.setDifferentialLimitedSlipRatio(std.math.floatMax(f32));

    var front_off = front;
    front_off.engine_torque_ratio = 0;
    try car.vehicle.setDifferential(0, front_off);

    var rear = car.vehicle.differential(1).?;
    rear.engine_torque_ratio = 1.0;
    try car.vehicle.setDifferential(1, rear);

    try car.vehicle.setWheeledDriverInput(1.0, 0, 0, 0);
    try car.settle(1.5);

    try std.testing.expect(car.vehicle.wheelAngularVelocity(2) > 5.0);
    try std.testing.expect(car.vehicle.wheelAngularVelocity(3) > 5.0);
    try std.testing.expect(@abs(car.vehicle.wheelAngularVelocity(0)) < 1.0);
    try std.testing.expect(@abs(car.vehicle.wheelAngularVelocity(1)) < 1.0);
}

test "a differential is validated on the way in, so a bad one cannot reach the solver" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();

    const good = car.vehicle.differential(0).?;

    var open = good;
    open.limited_slip_ratio = 1.0; // must be > 1
    try std.testing.expectError(error.InvalidArgument, car.vehicle.setDifferential(0, open));

    var split = good;
    split.left_right_split = 1.5; // must be in [0, 1]
    try std.testing.expectError(error.InvalidArgument, car.vehicle.setDifferential(0, split));

    var wheel = good;
    wheel.right_wheel = 9; // the car has four wheels
    try std.testing.expectError(error.InvalidArgument, car.vehicle.setDifferential(0, wheel));

    try std.testing.expectError(error.InvalidArgument, car.vehicle.setDifferential(7, good));

    // Nothing partially applied: the differential is exactly as it was.
    try std.testing.expectApproxEqAbs(
        good.limited_slip_ratio,
        car.vehicle.differential(0).?.limited_slip_ratio,
        1e-5,
    );

    // The vehicle-level ratio, which limits two DIFFERENTIALS against each
    // other rather than one differential's own two wheels.
    try std.testing.expect(car.vehicle.differentialLimitedSlipRatio() > 1.0);
    try car.vehicle.setDifferentialLimitedSlipRatio(2.5);
    try std.testing.expectApproxEqAbs(
        @as(f32, 2.5),
        car.vehicle.differentialLimitedSlipRatio(),
        1e-5,
    );
    try std.testing.expectError(error.InvalidArgument, car.vehicle.setDifferentialLimitedSlipRatio(1.0));
}

//=============================================================================
// Engine and clutch readback
//=============================================================================

test "engine RPM is clamped into the engine's own range, and torque is the curve sampled where it sits" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try car.settle(0.5);

    // The rig's engine defaults (see VehicleConstraint.Options): 1000 to 6000.
    try car.vehicle.setEngineRpm(100_000);
    try std.testing.expectApproxEqAbs(@as(f32, 6000), car.vehicle.engineRpm(), 1e-2);
    try car.vehicle.setEngineRpm(-50);
    try std.testing.expectApproxEqAbs(@as(f32, 1000), car.vehicle.engineRpm(), 1e-2);

    // Torque is `acceleration * max_torque * curve(rpm / max_rpm)`, so it is
    // linear in the throttle fraction and zero at a closed throttle whatever
    // the engine is doing.
    try car.vehicle.setEngineRpm(3000);
    const full = car.vehicle.engineTorque(1.0);
    try std.testing.expect(full > 0);
    try std.testing.expectApproxEqAbs(full * 0.5, car.vehicle.engineTorque(0.5), 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0), car.vehicle.engineTorque(0), 1e-6);

    // Placing the debug RPM meter is a no-op that reports itself when this
    // library was built without Jolt's debug renderer, which is the default.
    car.vehicle.setRpmMeter(zjolt.vec3(0, 1, 0), 0.5) catch |e| {
        try std.testing.expectEqual(error.Unsupported, e);
    };
}

test "wheel speed at the clutch is zero at rest and follows the driven wheels once moving" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try car.settle(1.5);

    try std.testing.expectApproxEqAbs(@as(f32, 0), car.vehicle.wheelSpeedAtClutch(), 1.0);

    try car.vehicle.setWheeledDriverInput(1.0, 0, 0, 0);
    try car.settle(2.0);

    const at_clutch = car.vehicle.wheelSpeedAtClutch();
    try std.testing.expect(car.vehicle.wheelAngularVelocity(0) > 1.0);
    try std.testing.expect(at_clutch > 0);
    // Same order of magnitude as the engine it is measured against — this is
    // the number engine RPM is subtracted from to get clutch slip.
    try std.testing.expect(at_clutch < car.vehicle.engineRpm() * 4);
}

//=============================================================================
// Tracks
//=============================================================================

test "each track reports the angular velocity that drives it, and the steering ratios separate them" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var tank = try buildTrackedCar(1.0);
    defer tank.deinit();
    try tank.settle(1.5);

    try std.testing.expectApproxEqAbs(
        @as(f32, 0),
        tank.vehicle.trackAngularVelocity(.left),
        0.5,
    );

    try tank.vehicle.setTrackedDriverInput(1.0, 1.0, 0.35, 0);
    try tank.settle(2.0);

    const left = tank.vehicle.trackAngularVelocity(.left);
    const right = tank.vehicle.trackAngularVelocity(.right);
    try std.testing.expect(left > 1.0);
    try std.testing.expect(right > 0);
    // The right track was asked for a third of the left's rate, and the
    // per-side readback is what says it actually got it.
    try std.testing.expect(right < left);

    // Seeding a track directly is accepted; the controller owns it again from
    // the next step onwards.
    try tank.vehicle.setTrackAngularVelocity(.right, 0);
    try std.testing.expectApproxEqAbs(@as(f32, 0), tank.vehicle.trackAngularVelocity(.right), 1e-5);
}

test "tracks and wheeled drivetrains refuse each other's entry points rather than reinterpreting memory" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();

    try std.testing.expectApproxEqAbs(@as(f32, 0), car.vehicle.trackAngularVelocity(.left), 1e-6);
    try std.testing.expectError(
        error.InvalidArgument,
        car.vehicle.setTrackAngularVelocity(.left, 1.0),
    );

    var tank = try buildTrackedCar(1.0);
    defer tank.deinit();

    try std.testing.expectEqual(@as(u32, 0), tank.vehicle.differentialCount());
    try std.testing.expect(tank.vehicle.differential(0) == null);
    try std.testing.expectApproxEqAbs(@as(f32, 0), tank.vehicle.wheelSpeedAtClutch(), 1e-6);
}

//=============================================================================
// Motorcycle lean spring
//=============================================================================

fn buildMotorcycle() !Rig {
    var world = try World.init();
    errdefer world.deinit();

    const chassis_shape = try zjolt.Shape.initBox(zjolt.vec3(0.25, 0.3, 0.9), .{ .density = 300 });
    errdefer chassis_shape.release();

    const bodies = world.system.bodies();
    const chassis = try bodies.createAndAdd(.{
        .shape = chassis_shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 1.0, 0),
        .allow_sleeping = false,
    }, .activate);

    const wheels = [_]zjolt.VehicleWheelDesc{
        wheelAt(0, 0.75), // front (0), steers
        wheelAt(0, -0.75), // rear  (1), driven
    };
    // A motorcycle's "differential" has one side: the rear wheel takes all
    // the engine torque and there is no wheel opposite it.
    var drive = zjolt.defaultVehicleDifferentialDesc();
    drive.left_wheel = 1;
    drive.right_wheel = -1;
    drive.engine_torque_ratio = 1.0;
    const differentials = [_]zjolt.VehicleDifferentialDesc{drive};

    const vehicle = try zjolt.VehicleConstraint.init(world.system, chassis, .{
        .controller_kind = .motorcycle,
        .wheels = &wheels,
        .differentials = &differentials,
        .collision_tester = collisionTester(),
    });

    raiseVelocitySteps(world.system);
    return .{ .world = world, .chassis_shape = chassis_shape, .chassis = chassis, .vehicle = vehicle };
}

test "a motorcycle's lean spring is readable and retunable after creation; a car has none" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var bike = try buildMotorcycle();
    defer bike.deinit();

    // The defaults VehicleConstraint.Options.motorcycle carries.
    const initial = bike.vehicle.motorcycleLeanSpring().?;
    try std.testing.expectApproxEqAbs(@as(f32, 5000), initial.spring_constant, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 1000), initial.spring_damping, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 4), initial.spring_integration_coefficient_decay, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), initial.lean_smoothing_factor, 1e-3);

    var softer = initial;
    softer.spring_constant = 1200;
    softer.spring_damping = 250;
    softer.spring_integration_coefficient = 3;
    softer.lean_smoothing_factor = 0.5;
    try bike.vehicle.setMotorcycleLeanSpring(softer);

    const read_back = bike.vehicle.motorcycleLeanSpring().?;
    try std.testing.expectApproxEqAbs(@as(f32, 1200), read_back.spring_constant, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 250), read_back.spring_damping, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 3), read_back.spring_integration_coefficient, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), read_back.lean_smoothing_factor, 1e-3);

    // A motorcycle IS a wheeled vehicle plus a lean spring, so it takes the
    // wheeled entry points too — and its wheel base is the geometry the lean
    // controller works from.
    try bike.vehicle.setWheeledDriverInput(0, 0, 0, 0);
    try std.testing.expect(bike.vehicle.motorcycleWheelBase() > 1.0);

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try std.testing.expect(car.vehicle.motorcycleLeanSpring() == null);
    try std.testing.expectError(
        error.InvalidArgument,
        car.vehicle.setMotorcycleLeanSpring(softer),
    );
}

//=============================================================================
// Wheel-ground collision filtering
//=============================================================================

const FloorFilter = struct {
    floor: zjolt.BodyId,
    vehicle_body: zjolt.BodyId,
    rejected: u32 = 0,
    saw_vehicle_body: bool = false,

    fn rejectFloor(user: ?*anyopaque, body: zjolt.BodyId) callconv(.c) bool {
        const self: *FloorFilter = @ptrCast(@alignCast(user.?));
        if (body == self.vehicle_body) self.saw_vehicle_body = true;
        if (body == self.floor) {
            self.rejected += 1;
            return false;
        }
        return true;
    }
};

test "a body filter takes the floor away from the wheels, and never has to exclude the vehicle itself" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try car.settle(1.5);

    // Baseline: every wheel is on the floor.
    var wheel: u32 = 0;
    while (wheel < 4) : (wheel += 1) try std.testing.expect(car.vehicle.wheelHasContact(wheel));

    var filter: FloorFilter = .{ .floor = car.world.floor, .vehicle_body = car.chassis };
    try car.vehicle.setWheelFilters(.{
        .body = .{ .should_collide = FloorFilter.rejectFloor, .user = &filter },
    });

    // Exactly what was installed comes back out.
    try std.testing.expectEqual(
        @as(?*anyopaque, @ptrCast(&filter)),
        car.vehicle.wheelFilters().body.user,
    );

    try car.settle(0.5);

    wheel = 0;
    while (wheel < 4) : (wheel += 1) try std.testing.expect(!car.vehicle.wheelHasContact(wheel));
    try std.testing.expect(filter.rejected > 0);

    // The vehicle's own body is rejected before the callback is consulted, so
    // a filter cannot accidentally drive the wheels into the chassis they
    // hang off — the callback is never even asked about it.
    try std.testing.expect(!filter.saw_vehicle_body);

    // Clearing puts the floor back.
    try car.vehicle.setWheelFilters(null);
    try std.testing.expect(car.vehicle.wheelFilters().body.should_collide == null);
    try car.settle(0.5);
    wheel = 0;
    while (wheel < 4) : (wheel += 1) try std.testing.expect(car.vehicle.wheelHasContact(wheel));
}

test "the wheel collision tester can be swapped after creation, filters and all" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try car.settle(1.5);
    try std.testing.expectEqual(zjolt.VehicleCollisionTesterKind.ray, car.vehicle.collisionTesterKind());

    var sphere = collisionTester();
    sphere.kind = .cast_sphere;
    sphere.radius = 0.1;
    try car.vehicle.setCollisionTester(sphere);
    try std.testing.expectEqual(
        zjolt.VehicleCollisionTesterKind.cast_sphere,
        car.vehicle.collisionTesterKind(),
    );

    try car.settle(0.5);
    var wheel: u32 = 0;
    while (wheel < 4) : (wheel += 1) try std.testing.expect(car.vehicle.wheelHasContact(wheel));

    // Filters survive the swap: the new tester is handed the same ones.
    var filter: FloorFilter = .{ .floor = car.world.floor, .vehicle_body = car.chassis };
    try car.vehicle.setWheelFilters(.{
        .body = .{ .should_collide = FloorFilter.rejectFloor, .user = &filter },
    });
    var cylinder = collisionTester();
    cylinder.kind = .cast_cylinder;
    try car.vehicle.setCollisionTester(cylinder);
    try car.settle(0.5);
    try std.testing.expect(filter.rejected > 0);
    wheel = 0;
    while (wheel < 4) : (wheel += 1) try std.testing.expect(!car.vehicle.wheelHasContact(wheel));

    try car.vehicle.setWheelFilters(null);
}

//=============================================================================
// Step callbacks
//=============================================================================

const StepLog = struct {
    pre: u32 = 0,
    post_collide: u32 = 0,
    post_step: u32 = 0,
    /// Where each of the three landed in one running sequence, so their order
    /// within a step is checkable rather than assumed.
    order: [3]u32 = .{ 0, 0, 0 },
    next: u32 = 0,
    delta_time: f32 = 0,
    first_and_last: bool = false,
    contact_at_post_collide: bool = false,

    fn onPre(
        user: ?*anyopaque,
        constraint: *zjolt.c.vehicle.VehicleConstraint,
        context: *const zjolt.VehicleStepContext,
    ) callconv(.c) void {
        _ = constraint;
        const self: *StepLog = @ptrCast(@alignCast(user.?));
        self.pre += 1;
        self.order[0] = self.next;
        self.next += 1;
        self.delta_time = context.delta_time;
        self.first_and_last = context.is_first_step and context.is_last_step;
    }

    fn onPostCollide(
        user: ?*anyopaque,
        constraint: *zjolt.c.vehicle.VehicleConstraint,
        context: *const zjolt.VehicleStepContext,
    ) callconv(.c) void {
        _ = context;
        const self: *StepLog = @ptrCast(@alignCast(user.?));
        self.post_collide += 1;
        self.order[1] = self.next;
        self.next += 1;
        // The whole point of this callback: the contacts found this step are
        // already there, a full step before anything outside `update` sees
        // them.
        const vehicle: zjolt.VehicleConstraint = .{ .handle = constraint };
        self.contact_at_post_collide = vehicle.wheelHasContact(0);
    }

    fn onPostStep(
        user: ?*anyopaque,
        constraint: *zjolt.c.vehicle.VehicleConstraint,
        context: *const zjolt.VehicleStepContext,
    ) callconv(.c) void {
        _ = constraint;
        _ = context;
        const self: *StepLog = @ptrCast(@alignCast(user.?));
        self.post_step += 1;
        self.order[2] = self.next;
        self.next += 1;
    }
};

test "the three step callbacks fire in order, once per collision step, with this vehicle's contacts already found" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var log: StepLog = .{};

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try car.settle(1.5);

    try car.vehicle.setPreStepCallback(.{ .on_step = StepLog.onPre, .user = &log });
    try car.vehicle.setPostCollideCallback(.{ .on_step = StepLog.onPostCollide, .user = &log });
    try car.vehicle.setPostStepCallback(.{ .on_step = StepLog.onPostStep, .user = &log });

    try std.testing.expectEqual(
        @as(?*anyopaque, @ptrCast(&log)),
        car.vehicle.preStepCallback().user,
    );

    const dt: f32 = 1.0 / 60.0;
    _ = try car.world.system.step(dt, 1, car.world.jobs);

    try std.testing.expectEqual(@as(u32, 1), log.pre);
    try std.testing.expectEqual(@as(u32, 1), log.post_collide);
    try std.testing.expectEqual(@as(u32, 1), log.post_step);
    try std.testing.expect(log.order[0] < log.order[1]);
    try std.testing.expect(log.order[1] < log.order[2]);
    try std.testing.expectApproxEqAbs(dt, log.delta_time, 1e-6);
    try std.testing.expect(log.first_and_last);
    try std.testing.expect(log.contact_at_post_collide);

    // delta_time is the SUB-step's: four collision steps is four calls at a
    // quarter of the time each.
    log.pre = 0;
    _ = try car.world.system.step(dt, 4, car.world.jobs);
    try std.testing.expectEqual(@as(u32, 4), log.pre);
    try std.testing.expectApproxEqAbs(dt / 4.0, log.delta_time, 1e-6);

    // Clearing one leaves the others alone.
    try car.vehicle.setPreStepCallback(null);
    log.pre = 0;
    log.post_step = 0;
    _ = try car.world.system.step(dt, 1, car.world.jobs);
    try std.testing.expectEqual(@as(u32, 0), log.pre);
    try std.testing.expectEqual(@as(u32, 1), log.post_step);

    try car.vehicle.setPostCollideCallback(null);
    try car.vehicle.setPostStepCallback(null);
}

//=============================================================================
// Tire friction callbacks
//=============================================================================

const FrictionLog = struct {
    floor: zjolt.BodyId,
    calls: u32 = 0,
    saw_floor: bool = false,

    /// Ice: whatever the tire and the road think, there is no grip.
    fn zeroGrip(
        user: ?*anyopaque,
        wheel_index: u32,
        body: zjolt.BodyId,
        sub_shape_id: zjolt.SubShapeId,
        longitudinal_friction: *f32,
        lateral_friction: *f32,
    ) callconv(.c) void {
        _ = wheel_index;
        _ = sub_shape_id;
        const self: *FrictionLog = @ptrCast(@alignCast(user.?));
        self.calls += 1;
        if (body == self.floor) self.saw_floor = true;
        longitudinal_friction.* = 0;
        lateral_friction.* = 0;
    }
};

fn driveAndMeasure(car: *Rig, seconds: f32) !f32 {
    const bodies = car.world.system.bodies();
    const start = bodies.getPosition(car.chassis);
    try car.vehicle.setWheeledDriverInput(1.0, 0, 0, 0);
    try car.settle(seconds);
    return @floatCast(bodies.getPosition(car.chassis).z - start.z);
}

test "a combine-friction callback replaces the road's grip, and is told which body the wheel is on" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const on_tarmac = blk: {
        var car = try buildWheeledCar(1.0);
        defer car.deinit();
        try car.settle(1.5);
        break :blk try driveAndMeasure(&car, 2.0);
    };

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    var log: FrictionLog = .{ .floor = car.world.floor };
    try car.settle(1.5);
    try car.vehicle.setCombineFrictionCallback(.{
        .combine = FrictionLog.zeroGrip,
        .user = &log,
    });

    const on_ice = try driveAndMeasure(&car, 2.0);

    try std.testing.expect(log.calls > 0);
    try std.testing.expect(log.saw_floor);
    try std.testing.expect(on_tarmac > 1.0);
    // No grip, no progress — the wheels spin instead.
    try std.testing.expect(on_ice < on_tarmac * 0.25);
    try std.testing.expect(car.vehicle.wheelAngularVelocity(0) > 1.0);

    try car.vehicle.setCombineFrictionCallback(null);
    try std.testing.expect(car.vehicle.combineFrictionCallback().combine == null);
}

const TireLog = struct {
    calls: u32 = 0,
    peak_suspension_impulse: f32 = 0,
    saw_delta_time: f32 = 0,

    /// A tire allowed to hold the car sideways but given no forward or
    /// backward grip at all.
    fn noDrive(
        user: ?*anyopaque,
        wheel_index: u32,
        inputs: *const zjolt.VehicleTireImpulseInputs,
        out_longitudinal_impulse: *f32,
        out_lateral_impulse: *f32,
    ) callconv(.c) void {
        _ = wheel_index;
        const self: *TireLog = @ptrCast(@alignCast(user.?));
        self.calls += 1;
        self.peak_suspension_impulse = @max(self.peak_suspension_impulse, inputs.suspension_impulse);
        self.saw_delta_time = inputs.delta_time;
        out_longitudinal_impulse.* = 0;
        out_lateral_impulse.* = inputs.lateral_friction * inputs.suspension_impulse;
    }
};

test "a tire max impulse callback caps grip per wheel, and is handed the suspension load it is capping" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var log: TireLog = .{};

    var car = try buildWheeledCar(1.0);
    defer car.deinit();
    try car.settle(1.5);
    try car.vehicle.setTireMaxImpulseCallback(.{ .compute = TireLog.noDrive, .user = &log });

    const capped = try driveAndMeasure(&car, 2.0);

    try std.testing.expect(log.calls > 0);
    // The car is standing on its springs, so the suspension is pushing real
    // impulse into the ground — this is the load a friction circle is scaled
    // against.
    try std.testing.expect(log.peak_suspension_impulse > 0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 60.0), log.saw_delta_time, 1e-5);
    try std.testing.expect(@abs(capped) < 0.5);

    // Clearing restores Jolt's own friction * suspension_impulse, and the
    // same car drives away.
    try car.vehicle.setTireMaxImpulseCallback(null);
    try std.testing.expect(car.vehicle.tireMaxImpulseCallback().compute == null);
    const freed = try driveAndMeasure(&car, 2.0);
    try std.testing.expect(freed > 1.0);
}
