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
const character_mod = @import("character.zig");
const state_mod = @import("state.zig");
const stream_mod = @import("stream.zig");
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
    // to lose if the restore below does nothing.
    try world.stepFor(0.8);

    const state = world.system.state();
    const saved = try state.saveAlloc(std.testing.allocator, .{ .state = .all });
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

    try state.restore(saved, .{});

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
    const saved = try state.saveAlloc(std.testing.allocator, .{ .state = .all });
    defer std.testing.allocator.free(saved);

    // The set of bodies the save was taken from no longer exists.
    bodies.destroy(doomed);

    const before = bodies.getPosition(keeper);
    try std.testing.expectError(zjolt.Error.BadFormat, state.restore(saved, .{}));
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
    const now = try state.saveAlloc(std.testing.allocator, .{ .state = .all });
    defer std.testing.allocator.free(now);
    try state.restore(now, .{});
}

//=============================================================================
// Why CONSTRAINTS is its own bit
//=============================================================================

/// A single-wheel vehicle's engine RPM is exactly the kind of state
/// `ZJOLT_STATE_RECORDER_STATE_CONSTRAINTS` is for and `BODIES` is not: it
/// belongs to the constraint, not any body, so wheel spin-up never shows in a
/// body's position or velocity — a save omitting CONSTRAINTS can't carry it,
/// nor can a restore put it back. The chassis uses `allow_sleeping = false` per
/// `vehicle_test.zig`'s standing rule, though not load-bearing here.
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
        const saved = try state.saveAlloc(std.testing.allocator, .{ .state = save_mask });
        defer std.testing.allocator.free(saved);
        const rpm_at_save = vehicle.engineRpm();

        // Keep revving past the save point before restoring into it.
        try world.stepFor(0.5);
        try state.restore(saved, .{});

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

//=============================================================================
// Selective save and restore (ZJoltStateFilter)
//=============================================================================

test "a partial save filtered to one body restores only that body, leaving the other untouched" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const kept = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(-1, 5, 0),
        .restitution = 0.6,
    }, .activate);
    const excluded = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(1, 5, 0),
        .restitution = 0.6,
    }, .activate);

    try world.stepFor(0.8);

    const state = world.system.state();

    const Filter = struct {
        keep: zjolt.BodyId,

        pub fn shouldSaveBody(self: *@This(), body: zjolt.BodyId) bool {
            return body == self.keep;
        }
    };
    var filter_ctx = Filter{ .keep = kept };
    const filter = state_mod.stateFilter(Filter, &filter_ctx);

    const saved = try state.saveAlloc(std.testing.allocator, .{ .state = .all, .filter = &filter });
    defer std.testing.allocator.free(saved);
    const kept_at_save = bodies.getPosition(kept);

    // Both bodies keep moving after the partial save was taken.
    try world.stepFor(1.0);
    const excluded_before_restore = bodies.getPosition(excluded);

    try state.restore(saved, .{ .filter = null, .is_last_part = true });

    // `kept` snapped back to where the (partial) save found it...
    const kept_after = bodies.getPosition(kept);
    try std.testing.expectApproxEqAbs(kept_at_save.y, kept_after.y, 1e-4);

    // ...while `excluded` was never in the payload at all, so the restore
    // left it exactly where it already was — not reset, not disturbed.
    const excluded_after = bodies.getPosition(excluded);
    try std.testing.expectEqual(excluded_before_restore.x, excluded_after.x);
    try std.testing.expectEqual(excluded_before_restore.y, excluded_after.y);
    try std.testing.expectEqual(excluded_before_restore.z, excluded_after.z);
}

test "a partial save is exempt from the body-set digest check a full save is not" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const kept = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);
    const doomed = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(3, 5, 0),
    }, .activate);

    const Filter = struct {
        keep: zjolt.BodyId,

        pub fn shouldSaveBody(self: *@This(), body: zjolt.BodyId) bool {
            return body == self.keep;
        }
    };
    var filter_ctx = Filter{ .keep = kept };
    const filter = state_mod.stateFilter(Filter, &filter_ctx);

    const saved = try world.system.state().saveAlloc(std.testing.allocator, .{ .state = .all, .filter = &filter });
    defer std.testing.allocator.free(saved);

    // The body set changes after the save — ordinarily refused (see the test
    // above this file's determinism section), but this save never claimed to
    // cover the whole world in the first place.
    bodies.destroy(doomed);

    try world.system.state().restore(saved, .{ .filter = null, .is_last_part = true });
}

//=============================================================================
// Host streams (ZJoltStream)
//=============================================================================

test "a state save through a host stream that cannot accept the bytes fails cleanly, not silently truncated" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();
    _ = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);

    try world.stepFor(0.2);

    // One byte cannot hold even the stream's own header, let alone a body's
    // state. The writer reports the shortfall through `is_failed` rather than
    // accepting a prefix of the bytes and calling it done — @see
    // ZJoltStream.write's contract in zjolt_core.h.
    var tiny: [1]u8 = undefined;
    var writer: zjolt.StreamBufferWriter = .{ .buffer = &tiny };
    try std.testing.expectError(
        zjolt.Error.IoError,
        world.system.state().saveStream(zjolt.hostStream(zjolt.StreamBufferWriter, &writer), .{}),
    );

    // Nothing silently truncated: the writer's own bookkeeping shows exactly
    // why it failed, not a partial write mistaken for a complete one.
    try std.testing.expect(writer.overflowed);
}

//=============================================================================
// Locating a divergence (zjoltPhysicsSystemCompareState)
//=============================================================================

test "compareState reports no divergence for a state compared with itself, and finds one after a step" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();

    _ = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);

    try world.stepFor(0.5);

    const state = world.system.state();
    const reference = try state.saveAlloc(std.testing.allocator, .{ .state = .all });
    defer std.testing.allocator.free(reference);

    const same = try state_mod.compareState(reference, reference);
    try std.testing.expect(!same.diverged);
    try std.testing.expectEqual(@as(usize, 0), same.offset);

    try world.stepFor(0.5);
    const after = try state.saveAlloc(std.testing.allocator, .{ .state = .all });
    defer std.testing.allocator.free(after);

    const different = try state_mod.compareState(reference, after);
    try std.testing.expect(different.diverged);
}

//=============================================================================
// One buffer, written then rewound for reading (RewindableBuffer)
//=============================================================================

test "a RewindableBuffer round-trips a save/restore through one buffer object, not a matching pair" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.4, .{});
    defer shape.release();
    const ball = try world.system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);

    try world.stepFor(0.5);

    var buffer: [4096]u8 = undefined;
    var recorder: stream_mod.RewindableBuffer = .{ .buffer = &buffer };
    try world.system.state().saveStream(stream_mod.hostStream(stream_mod.RewindableBuffer, &recorder), .{});
    const reference = world.system.bodies().getPosition(ball);

    try world.stepFor(0.5);
    try std.testing.expect(world.system.bodies().getPosition(ball).y != reference.y);

    // Same object, same bytes, just pointed back at the start — not a
    // second buffer built from `recorder`'s written slice.
    recorder.rewind();
    try world.system.state().restoreStream(stream_mod.hostStream(stream_mod.RewindableBuffer, &recorder), .{});

    const restored = world.system.bodies().getPosition(ball);
    try std.testing.expectApproxEqAbs(reference.x, restored.x, 1e-4);
    try std.testing.expectApproxEqAbs(reference.y, restored.y, 1e-4);
    try std.testing.expectApproxEqAbs(reference.z, restored.z, 1e-4);
}

//=============================================================================
// Characters
//
// JPH::PhysicsSystem::SaveState does not save a character — CharacterVirtual.h
// says so — so a save that is not handed them writes a snapshot of the world
// without its player and reports success. These are the tests that the refusal
// fires, and that a character handed in really does come back.
//=============================================================================

/// Walks `character` forward for `seconds`, applying gravity while airborne,
/// the same shape of loop `character_test.zig` uses.
fn walkCharacter(character: zjolt.Character, seconds: f32) !void {
    const dt: f32 = 1.0 / 60.0;
    const settings = zjolt.defaultCharacterUpdateSettings();
    var elapsed: f32 = 0;
    while (elapsed < seconds) : (elapsed += dt) {
        var velocity = character.getLinearVelocity();
        if (character.groundState() == .on_ground) {
            velocity.y = 0;
        } else {
            velocity.y += zjolt.gravity_earth.y * dt;
        }
        velocity.x = 1.5;
        character.setLinearVelocity(velocity);
        try character.update(dt, zjolt.gravity_earth, &settings, null);
    }
}

test "a save that omits a live character is refused, not written" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const state = world.system.state();
    // The same system, with no character in it, saves fine — so what the
    // refusal below reacts to is the character, not the world.
    const before = try state.saveAlloc(std.testing.allocator, .{});
    std.testing.allocator.free(before);

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 0.9, 0),
    });
    defer character.deinit();

    try std.testing.expectError(error.StateIncomplete, state.size(.{}));
    try std.testing.expectError(
        error.StateIncomplete,
        state.saveAlloc(std.testing.allocator, .{}),
    );

    // And the message says which mistake it is, rather than leaving the
    // caller to guess at a bare error code.
    try std.testing.expect(std.mem.indexOf(
        u8,
        zjolt.lastError(),
        "character controllers that the save was not given",
    ) != null);

    // Handing it in is what makes the same save legal.
    const saved = try state.saveAlloc(
        std.testing.allocator,
        .{ .characters = .{ .characters = &.{character} } },
    );
    std.testing.allocator.free(saved);
}

test "a character handed to the save has its position and ground state restored" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(-2, 0.9, 0),
    });
    defer character.deinit();

    try walkCharacter(character, 0.5);

    const state = world.system.state();
    const characters: state_mod.Characters = .{ .characters = &.{character} };
    const saved = try state.saveAlloc(std.testing.allocator, .{ .characters = characters });
    defer std.testing.allocator.free(saved);

    const reference_position = character.getPosition();
    const reference_velocity = character.getLinearVelocity();
    const reference_ground = character.groundState();

    try walkCharacter(character, 1.0);
    // It really did move on, so the restore below has something to undo.
    try std.testing.expect(@abs(character.getPosition().x - reference_position.x) > 0.5);

    try state.restore(saved, .{ .characters = characters });

    const position = character.getPosition();
    try std.testing.expectApproxEqAbs(reference_position.x, position.x, 1e-4);
    try std.testing.expectApproxEqAbs(reference_position.y, position.y, 1e-4);
    try std.testing.expectApproxEqAbs(reference_position.z, position.z, 1e-4);
    const velocity = character.getLinearVelocity();
    try std.testing.expectApproxEqAbs(reference_velocity.x, velocity.x, 1e-4);
    try std.testing.expectApproxEqAbs(reference_velocity.y, velocity.y, 1e-4);
    try std.testing.expectEqual(reference_ground, character.groundState());
}

test "a rigid character counts toward the same completeness check" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const rigid = try zjolt.RigidCharacter.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(3, 3, 0),
        .layer = Layers.moving,
    });
    defer rigid.deinit();
    rigid.addToPhysicsSystem(.activate);
    defer rigid.removeFromPhysicsSystem();

    try world.stepFor(1.5);
    rigid.postSimulation(0.05);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, rigid.groundState());

    const state = world.system.state();
    try std.testing.expectError(error.StateIncomplete, state.size(.{}));

    const characters: state_mod.Characters = .{ .rigid = &.{rigid} };
    const saved = try state.saveAlloc(std.testing.allocator, .{ .characters = characters });
    defer std.testing.allocator.free(saved);
    try state.restore(saved, .{ .characters = characters });
    try std.testing.expectEqual(zjolt.GroundState.on_ground, rigid.groundState());
}

test "a restore given a different character set than the save is refused" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const a = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(-2, 0.9, 0),
    });
    defer a.deinit();
    const b = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(2, 0.9, 0),
    });
    defer b.deinit();

    const state = world.system.state();
    const saved = try state.saveAlloc(
        std.testing.allocator,
        .{ .characters = .{ .characters = &.{ a, b } } },
    );
    defer std.testing.allocator.free(saved);

    // Same characters, opposite order: the payload is a flat concatenation,
    // so this would read a's bytes into b.
    try std.testing.expectError(error.BadFormat, state.restore(saved, .{
        .characters = .{ .characters = &.{ b, a } },
    }));
    // And none at all.
    try std.testing.expectError(error.BadFormat, state.restore(saved, .{}));

    // The right set in the right order still works.
    try state.restore(saved, .{ .characters = .{ .characters = &.{ a, b } } });
}

test "the same character twice is refused rather than counted as two" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const a = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(-2, 0.9, 0),
    });
    defer a.deinit();
    const b = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(2, 0.9, 0),
    });
    defer b.deinit();

    const state = world.system.state();
    try std.testing.expectError(error.InvalidArgument, state.size(.{
        .characters = .{ .characters = &.{ a, a } },
    }));

    // Both, once each, is the set the system actually holds.
    const saved = try state.saveAlloc(
        std.testing.allocator,
        .{ .characters = .{ .characters = &.{ a, b } } },
    );
    std.testing.allocator.free(saved);
}

test "one character saves and restores on its own, without the world" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(-2, 0.9, 0),
    });
    defer character.deinit();

    try walkCharacter(character, 0.5);
    const reference = character.getPosition();

    const saved = try character.saveStateAlloc(std.testing.allocator);
    defer std.testing.allocator.free(saved);
    try std.testing.expectEqual(saved.len, try character.stateSize());

    try walkCharacter(character, 1.0);
    try std.testing.expect(@abs(character.getPosition().x - reference.x) > 0.5);

    try character.restoreState(saved);
    try std.testing.expectApproxEqAbs(reference.x, character.getPosition().x, 1e-4);

    // A whole-system save handed to a character restore is refused on the
    // container tag rather than parsed as a character.
    const world_state = try world.system.state().saveAlloc(
        std.testing.allocator,
        .{ .characters = .{ .characters = &.{character} } },
    );
    defer std.testing.allocator.free(world_state);
    try std.testing.expectError(error.BadFormat, character.restoreState(world_state));
}

test "a character's state also round-trips through a host stream" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(-2, 0.9, 0),
    });
    defer character.deinit();

    try walkCharacter(character, 0.5);
    const reference = character.getPosition();

    var buffer: [8192]u8 = undefined;
    var recorder: stream_mod.RewindableBuffer = .{ .buffer = &buffer };
    try character.saveStateStream(stream_mod.hostStream(stream_mod.RewindableBuffer, &recorder));

    try walkCharacter(character, 1.0);
    try std.testing.expect(@abs(character.getPosition().x - reference.x) > 0.5);

    recorder.rewind();
    try character.restoreStateStream(stream_mod.hostStream(stream_mod.RewindableBuffer, &recorder));
    try std.testing.expectApproxEqAbs(reference.x, character.getPosition().x, 1e-4);
}
