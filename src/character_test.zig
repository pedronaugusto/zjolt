//! Behavioural tests for the virtual character: what tells it apart from a
//! plain swept shape is stepping over a low ledge and refusing a tall one,
//! reporting which of those it is doing, and a contact listener whose
//! failure does not take the character down with it.
//!
//! Reuses `integration_test.zig`'s layer map and floor fixture rather than
//! building its own — see `BINDING.md`. `integration_test.zig` already has a
//! settling test for `Character`; this file is everything past that.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");

const Layers = fixture.Layers;
const World = fixture.World;

const dt: f32 = 1.0 / 60.0;

/// Sets a walking velocity and calls `update` once, matching the pattern
/// `integration_test.zig` uses for its own character test: fall while
/// airborne, stop falling once grounded, walk forward only once supported so
/// a character mid-step off a ledge does not drift sideways in mid-air.
fn walkOneFrame(character: zjolt.Character, settings: *const zjolt.CharacterUpdateSettings, speed: f32) !void {
    var velocity = character.getLinearVelocity();
    if (character.groundState() == .on_ground) {
        velocity.y = 0;
    } else {
        velocity.y += zjolt.gravity_earth.y * dt;
    }
    // Horizontal input is kept whether or not the character is supported.
    // Zeroing it in the air looks reasonable and is a trap: riding up over a
    // step's edge lifts the character onto a surface too steep to count as
    // ground, so a controller that only pushes while supported stops pushing
    // exactly when it needs to push, falls back down, and loops forever at
    // the foot of a step it can clear.
    velocity.x = speed;
    character.setLinearVelocity(velocity);
    try character.update(dt, zjolt.gravity_earth, settings, null);
}

fn walkForward(character: zjolt.Character, settings: *const zjolt.CharacterUpdateSettings, speed: f32, seconds: f32) !void {
    var elapsed: f32 = 0;
    while (elapsed < seconds) : (elapsed += dt) {
        try walkOneFrame(character, settings, speed);
    }
}

//=============================================================================
// Stepping and walls
//=============================================================================

test "a character walks up a step within its stride" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // A ledge 0.35 m high — inside the default 0.4 m stair-step allowance, and
    // taller than the capsule's 0.3 m radius. That second number is what makes
    // this a test of WalkStairs rather than of geometry: a step below the
    // radius is one the bottom sphere simply rolls over, and the automatic
    // stair walk never has to run.
    const step_shape = try zjolt.Shape.initBox(zjolt.vec3(2, 0.5, 2), .{});
    defer step_shape.release();
    _ = try world.system.bodies().createAndAdd(.{
        .shape = step_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        // Top at y = 0.35, and the bottom well below the fixture floor's top
        // (y = 0) rather than exactly coplanar with it — coincident static
        // surfaces are a degenerate case collision detection handles
        // inconsistently, and it is the step's top and leading edge this test
        // cares about, not its underside.
        .position = zjolt.rvec3(3, -0.15, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(-2, 0.9, 0),
    });
    defer character.deinit();

    const settings = zjolt.defaultCharacterUpdateSettings();
    // 3 s at 1.5 m/s covers the 2.7 m to the ledge's near face (x = 1, less
    // the capsule radius) with room for the climb and a stride along the top.
    try walkForward(character, &settings, 1.5, 3.0);

    const position = character.getPosition();
    // Past the ledge's near face with room to spare, and standing ON the
    // ledge rather than stopped at its foot: the top is at y = 0.35, so
    // resting height there is 0.35 higher than the 0.8 the floor gives.
    try std.testing.expect(position.x > 1.5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.15), position.y, 0.05);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());
}

test "a character on a ramp is supported by it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // A 30-degree ramp, well inside the 50-degree default slope limit.
    const ramp_shape = try zjolt.Shape.initBox(zjolt.vec3(3, 0.2, 3), .{});
    defer ramp_shape.release();
    const tilt = std.math.degreesToRadians(30.0);
    _ = try world.system.bodies().createAndAdd(.{
        .shape = ramp_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(0, 1, 0),
        .rotation = zjolt.quatFromAxisAngle(zjolt.vec3(0, 0, 1), tilt),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 3, 0),
    });
    defer character.deinit();

    const settings = zjolt.defaultCharacterUpdateSettings();
    try walkForward(character, &settings, 0, 1.5);

    // A capsule resting on a slope touches it on the SIDE of its bottom cap,
    // above the shape's lowest point. If the supporting volume sits at that
    // lowest point, this contact is discarded and the character reports
    // itself unsupported on ground it is plainly standing on. It is the one
    // assertion that tells a correct plane from a plausible one.
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());
    try std.testing.expect(character.isSupported());

    // And it is the ramp's face it reports, not the floor underneath.
    const normal = character.groundNormal();
    try std.testing.expectApproxEqAbs(@cos(tilt), normal.y, 0.05);
    try std.testing.expect(normal.x < -0.3);
}

test "a wall the character is pressed against is not reported as ground" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // A wall with no floor near it, so the only thing the character can
    // possibly touch is vertical. Jolt's own default supporting volume
    // accepts a contact anywhere on the shape, which reports this as ground;
    // the override zjoltCharacterCreate applies is what makes it not.
    const wall_shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 5, 5), .{});
    defer wall_shape.release();
    _ = try world.system.bodies().createAndAdd(.{
        .shape = wall_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(2, 8, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        // Level with the middle of the wall and far above the floor.
        .position = zjolt.rvec3(1.1, 8, 0),
    });
    defer character.deinit();

    const settings = zjolt.defaultCharacterUpdateSettings();
    // Push into the wall with gravity switched off, so nothing but the wall
    // is ever in reach.
    var frame: u32 = 0;
    while (frame < 10) : (frame += 1) {
        character.setLinearVelocity(zjolt.vec3(2, 0, 0));
        try character.update(dt, zjolt.vec3(0, 0, 0), &settings, null);
    }

    try std.testing.expect(!character.isSupported());
    try std.testing.expect(character.groundState() != .on_ground);
    // It really is touching the wall — the assertions above are about how
    // that contact is classified, not about having missed it.
    try std.testing.expect(character.getPosition().x < 1.5);
}

test "a character is stopped by a wall taller than it can step" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // Six metres of wall — nothing like the 0.4 m default stair allowance,
    // so WalkStairs has nothing to find here and the character simply stops.
    const wall_shape = try zjolt.Shape.initBox(zjolt.vec3(0.2, 3, 5), .{});
    defer wall_shape.release();
    _ = try world.system.bodies().createAndAdd(.{
        .shape = wall_shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .position = zjolt.rvec3(4, 3, 0),
    }, .dont_activate);
    world.system.optimizeBroadPhase();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(-2, 0.9, 0),
    });
    defer character.deinit();

    const settings = zjolt.defaultCharacterUpdateSettings();
    try walkForward(character, &settings, 1.5, 4.0);

    const position = character.getPosition();
    // Unobstructed, four seconds at 1.5 m/s from x = -2 reaches x = 4; the
    // wall's near face is at x = 3.8, so anything close to that would mean
    // the wall did nothing.
    try std.testing.expect(position.x < 3.6);
    // Blocked, not derailed: it made real progress before the wall, and it
    // is still standing rather than having climbed or fallen.
    try std.testing.expect(position.x > 0);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());
}

//=============================================================================
// Ground state
//=============================================================================

test "ground state reports supported on a floor and in-air off it" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 5, 0),
    });
    defer character.deinit();

    // Nothing has moved yet, but asking fresh at a position with nothing
    // beneath it already reports the airborne state rather than a stale
    // default.
    try character.refreshContacts(null);
    try std.testing.expectEqual(zjolt.GroundState.in_air, character.groundState());
    try std.testing.expect(!character.isSupported());

    const settings = zjolt.defaultCharacterUpdateSettings();
    try walkForward(character, &settings, 0, 4.0);

    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());
    try std.testing.expect(character.isSupported());

    // Lifted back into open air and asked again: it reports in-air again
    // rather than remembering the floor it just left.
    character.setPosition(zjolt.rvec3(20, 20, 0));
    try character.refreshContacts(null);
    try std.testing.expectEqual(zjolt.GroundState.in_air, character.groundState());
    try std.testing.expect(!character.isSupported());
}

//=============================================================================
// Contact listener
//=============================================================================

test "the contact listener fires, and a listener that signals an error leaves the system usable" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 3, 0),
    });
    defer character.deinit();

    const Recorder = struct {
        calls: u32 = 0,
        added: u32 = 0,

        // `pub`, and it has to be: `@hasDecl` only sees public declarations
        // across files, so a private callback installs a null thunk and
        // never fires. `ContactListener` refuses a type with none of them,
        // which is what turns that into a compile error.
        pub fn onContactValidate(
            self: *@This(),
            character_id: zjolt.CharacterId,
            contact: *const zjolt.CharacterContact,
        ) !bool {
            _ = .{ character_id, contact };
            self.calls += 1;
            // The very first contact this character ever finds signals an
            // error instead of answering. The point of the test is that
            // this one rejection is all that happens — not a wedged
            // character or a wedged world.
            if (self.calls == 1) return error.FirstContactRejected;
            return true;
        }

        pub fn onContactAdded(
            self: *@This(),
            character_id: zjolt.CharacterId,
            contact: *const zjolt.CharacterContact,
            settings: *zjolt.CharacterContactSettings,
        ) void {
            _ = .{ character_id, contact, settings };
            self.added += 1;
        }
    };

    var recorder: Recorder = .{};
    var listener = zjolt.CharacterContactListener(Recorder).init(&recorder);
    try listener.attach();
    defer listener.deinit();
    character.setListener(listener.handle);
    defer character.setListener(null);

    const settings = zjolt.defaultCharacterUpdateSettings();

    // The character starts in the air, so the first validation call does not
    // come until it has fallen far enough to touch the floor. Step until it
    // does rather than assuming frame one — a fixed frame count here is a
    // test that passes or fails on the drop height.
    var frames: u32 = 0;
    while (recorder.calls == 0) : (frames += 1) {
        try std.testing.expect(frames < 600); // 10 s: it fell 3 m or it never will
        try walkOneFrame(character, &settings, 0);
    }

    // That first validation callback signalled an error. `update` itself
    // still returned successfully — the failure is stashed, not unwound
    // across the callback, which is what `check` is for.
    try std.testing.expectError(error.FirstContactRejected, listener.check());

    // Not wounded: the character keeps falling and settling exactly as it
    // would have if the first validation had never failed.
    try walkForward(character, &settings, 0, 3.0);

    // The one stashed failure was consumed by the `check` above, and every
    // validation since returned normally — so there is nothing left to
    // report the second time.
    try listener.check();
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());
    try std.testing.expect(character.isSupported());
    try std.testing.expect(recorder.calls > 1);
    try std.testing.expect(recorder.added > 0);
}
