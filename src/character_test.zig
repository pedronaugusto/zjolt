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

// `CharacterBase` is new and not yet re-exported from `zjolt.zig` — the
// closing report names it. Imported directly here rather than waiting on
// that.
const character_mod = @import("character.zig");

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
        .rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 0, 1), tilt),
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
    try character.setListener(listener.handle);
    defer character.setListener(null) catch {};

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

test "a character reports which listener is attached to it" {
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

    try std.testing.expect(character.getListener() == null);

    const Counter = struct {
        added: u32 = 0,

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

    var counter: Counter = .{};
    var listener = zjolt.CharacterContactListener(Counter).init(&counter);
    try listener.attach();
    defer listener.deinit();

    try character.setListener(listener.handle);
    try std.testing.expectEqual(listener.handle, character.getListener().?);

    // Detaching shows through the same getter, and destroys nothing: the
    // character never owned the listener, so this reports an attachment
    // rather than transferring one.
    try character.setListener(null);
    try std.testing.expect(character.getListener() == null);
    try listener.check();
}

//=============================================================================
// The supporting volume
//=============================================================================

test "the supporting volume decides which contacts may count as ground" {
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

    const settings = zjolt.defaultCharacterUpdateSettings();
    try walkForward(character, &settings, 0, 4.0);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());

    // `init` installs a plane an inner radius above the shape's lowest point,
    // pointing up. Jolt's own bare default sits 1e10 below everything and so
    // accepts any contact at all, including a wall.
    const original = character.supportingVolume();
    try std.testing.expect(original.normal.y > 0.9);

    // Lift the plane far above the character: every contact is now in FRONT
    // of it, and none of them may support the character any more.
    try character.setSupportingVolume(.{ .normal = zjolt.vec3(0, 1, 0), .distance = 100 });
    try character.refreshContacts(null);
    try std.testing.expectEqual(zjolt.GroundState.not_supported, character.groundState());
    try std.testing.expect(!character.isSupported());

    // Which is the distinction worth having: the plane rules out SUPPORT, not
    // contact. The character still touches the floor and still reports which
    // body it is touching — it just may not stand on it.
    try std.testing.expectEqual(world.floor, character.groundBodyId());

    try character.setSupportingVolume(original);
    try character.refreshContacts(null);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());

    // A plane with no direction has no side, and a character given one would
    // never report ground again — so it is refused here rather than installed
    // and puzzled over several frames later.
    try std.testing.expectError(error.InvalidArgument, character.setSupportingVolume(.{
        .normal = zjolt.vec3(0, 0, 0),
        .distance = 0,
    }));
}

//=============================================================================
// Asking about a placement the character is not at
//=============================================================================

test "a virtual character is reachable by a cast only through its transformed shape" {
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

    // Nothing in the system finds it: a CharacterVirtual is never put in the
    // broad phase, and this one has no inner body standing in for it.
    const queries = world.system.queries();
    try std.testing.expect((try queries.castRayClosest(
        zjolt.rvec3(-5, 3, 0),
        zjolt.vec3(10, 0, 0),
        null,
        null,
    )) == null);

    // The same ray against the character's own volume hits the front of the
    // capsule: 5 m to the axis less the 0.3 m radius, over a 10 m ray.
    const volume = try character.getTransformedShape();
    defer volume.deinit();
    const hit = (try volume.castRayClosest(
        zjolt.rvec3(-5, 3, 0),
        zjolt.vec3(10, 0, 0),
        null,
        null,
    )) orelse return error.TestExpectedCharacterHit;
    try std.testing.expectApproxEqAbs(@as(f32, 0.47), hit.fraction, 0.02);

    // It is a snapshot, not a view. Teleporting the character leaves it
    // wrapping around the character's earlier position, which is exactly
    // why a caller takes a fresh one per frame.
    character.setPosition(zjolt.rvec3(500, 3, 0));
    try std.testing.expect((try volume.castRayClosest(
        zjolt.rvec3(-5, 3, 0),
        zjolt.vec3(10, 0, 0),
        null,
        null,
    )) != null);
}

test "checkCollision answers about a placement without moving the character" {
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

    const settings = zjolt.defaultCharacterUpdateSettings();
    try walkForward(character, &settings, 0, 4.0);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());

    const standing = character.getPosition();
    const collision_query: zjolt.CharacterCollisionQuery = .{ .max_separation_distance = 0.1 };

    var buffer: [16]zjolt.CharacterCollisionHit = undefined;
    const here = try character.checkCollision(standing, collision_query, null, &buffer);
    try std.testing.expect(here.len > 0);
    try std.testing.expectEqual(world.floor, here[0].body);
    // A body hit names no character, and that is how the two are told apart.
    try std.testing.expectEqual(zjolt.invalid_character_id, here[0].character_id);

    // Fifty metres up there is nothing to overlap — and asking neither moved
    // the character nor disturbed the contacts it already had.
    try std.testing.expectEqual(
        @as(u32, 0),
        try character.countCollisions(zjolt.rvec3(0, 50, 0), collision_query, null),
    );
    try std.testing.expectEqual(standing, character.getPosition());
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());
}

test "checkCollision finds another virtual character, which no system query can" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();

    const walker = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 1, 0),
    });
    defer walker.deinit();
    // Close enough that the two 0.3 m capsules overlap.
    const other = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0.2, 1, 0),
    });
    defer other.deinit();

    const crowd = try zjolt.CharacterVsCharacterCollision.init();
    defer crowd.deinit();
    crowd.add(walker);
    crowd.add(other);
    walker.setCharacterVsCharacterCollision(crowd.handle);
    defer walker.setCharacterVsCharacterCollision(null);

    var buffer: [16]zjolt.CharacterCollisionHit = undefined;
    const hits = try walker.checkCollision(
        walker.getPosition(),
        .{ .max_separation_distance = 0.1 },
        null,
        &buffer,
    );

    // This hit exists only because the check also walks the
    // character-vs-character list — the other character is in no broad phase
    // for a query to find. It is named by character id and its body id is the
    // invalid one, which is the whole reason these are not plain overlap hits.
    var found = false;
    for (hits) |hit| {
        if (hit.character_id == other.id()) {
            found = true;
            try std.testing.expectEqual(zjolt.invalid_body_id, hit.body);
        }
    }
    try std.testing.expect(found);
}

test "a custom character-vs-character callback excludes a pairing the built-in list would have collided" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();

    const walker = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 1, 0),
    });
    defer walker.deinit();
    // Both close enough to overlap the capsule at the origin — geometrically,
    // `walker` collides with either one on its own.
    const friend = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0.2, 1, 0),
    });
    defer friend.deinit();
    const foe = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(-0.2, 1, 0),
    });
    defer foe.deinit();

    var buffer: [16]zjolt.CharacterCollisionHit = undefined;

    // Control: the built-in brute-force list finds BOTH. This is the
    // baseline the custom callback below is proven to differ from — without
    // it, "excludes a pairing" would not mean anything.
    const crowd = try zjolt.CharacterVsCharacterCollision.init();
    defer crowd.deinit();
    crowd.add(walker);
    crowd.add(friend);
    crowd.add(foe);
    walker.setCharacterVsCharacterCollision(crowd.handle);

    const control_hits = try walker.checkCollision(
        walker.getPosition(),
        .{ .max_separation_distance = 0.1 },
        null,
        &buffer,
    );
    var control_saw_friend = false;
    var control_saw_foe = false;
    for (control_hits) |hit| {
        if (hit.character_id == friend.id()) control_saw_friend = true;
        if (hit.character_id == foe.id()) control_saw_foe = true;
    }
    try std.testing.expect(control_saw_friend);
    try std.testing.expect(control_saw_foe);

    // A stand-in for a team filter or a spatial structure: this offers
    // `friend` as a candidate and never offers `foe` at all — not "visit and
    // reject", genuinely never proposed, the way the real capability (a
    // spatial index that simply never returns a distant character) works.
    const TeamFilter = struct {
        friend_character: zjolt.Character,

        pub fn onCollideCharacter(
            self: *@This(),
            character: zjolt.CharacterId,
            transform: *const zjolt.RMat44,
            visitor: character_mod.CandidateVisitor,
        ) void {
            _ = character;
            _ = transform;
            _ = visitor.visit(self.friend_character);
        }
    };

    var team_filter: TeamFilter = .{ .friend_character = friend };
    var collider = character_mod.CharacterVsCharacterCollider(TeamFilter).init(&team_filter);
    try collider.attach();
    defer {
        walker.setCharacterVsCharacterCollision(null);
        collider.deinit();
    }
    walker.setCharacterVsCharacterCollision(collider.handle);

    const filtered_hits = try walker.checkCollision(
        walker.getPosition(),
        .{ .max_separation_distance = 0.1 },
        null,
        &buffer,
    );
    var filtered_saw_friend = false;
    var filtered_saw_foe = false;
    for (filtered_hits) |hit| {
        if (hit.character_id == friend.id()) filtered_saw_friend = true;
        if (hit.character_id == foe.id()) filtered_saw_foe = true;
    }
    try std.testing.expect(filtered_saw_friend);
    try std.testing.expect(!filtered_saw_foe);
    try collider.check();
}

//=============================================================================
// The inner body
//=============================================================================

test "the inner body can be given a shape of its own" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const pin = try zjolt.Shape.initSphere(0.1, .{});
    defer pin.release();
    const bubble = try zjolt.Shape.initSphere(0.9, .{});
    defer bubble.release();

    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 3, 0),
        // Deliberately NOT the swept shape: a cheap proxy for the world to
        // see, standing in for a detailed sweep volume.
        .inner_body_shape = pin,
        .inner_body_layer = Layers.moving,
    });
    defer character.deinit();
    try std.testing.expect(character.innerBodyId() != zjolt.invalid_body_id);

    // Half a metre off the character's axis: outside the 0.1 m proxy, inside
    // a 0.9 m one. This is the ray the two shapes disagree about.
    const queries = world.system.queries();
    const origin = zjolt.rvec3(-5, 3.5, 0);
    const along = zjolt.vec3(10, 0, 0);
    try std.testing.expect((try queries.castRayClosest(origin, along, null, null)) == null);

    try character.setInnerBodyShape(bubble);
    const hit = (try queries.castRayClosest(origin, along, null, null)) orelse
        return error.TestExpectedInnerBodyHit;
    try std.testing.expectEqual(character.innerBodyId(), hit.body);

    // A character with no inner body has nothing to give a shape to, and
    // saying so beats Jolt's own silent no-op.
    const bodyless = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(20, 3, 0),
    });
    defer bodyless.deinit();
    try std.testing.expectError(error.InvalidArgument, bodyless.setInnerBodyShape(bubble));
}

//=============================================================================
// The rigid character
//=============================================================================

test "a rigid character answers the same three questions about itself" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.RigidCharacter.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 3, 0),
        .layer = Layers.moving,
    });
    defer character.deinit();
    character.addToPhysicsSystem(.activate);
    defer character.removeFromPhysicsSystem();

    try world.stepFor(2.0);
    character.postSimulation(0.05);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());

    // This one has a body, so its transformed shape is a convenience rather
    // than the only way in — but it still has to report the placement the
    // body is actually at.
    const volume = try character.getTransformedShape();
    defer volume.deinit();
    const standing = character.getPosition();
    const hit = (try volume.castRayClosest(
        zjolt.rvec3(-5, standing.y, standing.z),
        zjolt.vec3(10, 0, 0),
        null,
        null,
    )) orelse return error.TestExpectedRigidCharacterHit;
    try std.testing.expectApproxEqAbs(@as(f32, 0.47), hit.fraction, 0.05);

    // The floor is under it, and asking does not move it.
    var buffer: [16]zjolt.CharacterCollisionHit = undefined;
    const here = try character.checkCollision(
        standing,
        .{ .max_separation_distance = 0.1 },
        &buffer,
    );
    try std.testing.expect(here.len > 0);
    try std.testing.expectEqual(world.floor, here[0].body);
    // Nothing here walks a character-vs-character list, so no hit names one.
    try std.testing.expectEqual(zjolt.invalid_character_id, here[0].character_id);
    try std.testing.expectEqual(standing, character.getPosition());

    // The supporting volume crosses in both directions here too. A rigid
    // character re-reads it in postSimulation rather than during a sweep, so
    // that is where a change takes effect.
    const original = character.supportingVolume();
    try std.testing.expect(original.normal.y > 0.9);
    try character.setSupportingVolume(.{ .normal = zjolt.vec3(0, 1, 0), .distance = 100 });
    character.postSimulation(0.05);
    try std.testing.expect(!character.isSupported());

    try character.setSupportingVolume(original);
    character.postSimulation(0.05);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());
}

//=============================================================================
// CharacterBase
//=============================================================================

/// One function, either concrete kind behind it: nothing inside can tell
/// which one it was handed, which is the point of `CharacterBase`.
fn settledOnGround(base: character_mod.CharacterBase) bool {
    return base.groundState() == .on_ground and base.isSupported();
}

test "a single function reads ground state and the supporting volume through either character kind's shared base" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();

    const virtual_character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(0, 3, 0),
    });
    defer virtual_character.deinit();

    const rigid_character = try zjolt.RigidCharacter.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(10, 3, 0),
        .layer = Layers.moving,
    });
    defer rigid_character.deinit();
    rigid_character.addToPhysicsSystem(.activate);
    defer rigid_character.removeFromPhysicsSystem();

    const settings = zjolt.defaultCharacterUpdateSettings();
    try walkForward(virtual_character, &settings, 0, 4.0);
    try world.stepFor(2.0);
    rigid_character.postSimulation(0.05);

    try std.testing.expect(settledOnGround(virtual_character.asBase()));
    try std.testing.expect(settledOnGround(rigid_character.asBase()));

    // The shared surface reaches past the ground getters: the
    // supporting-volume plane, the shape, and the up vector all read the
    // same through `asBase` as through the concrete type.
    try std.testing.expect(virtual_character.asBase().supportingVolume().normal.y > 0.9);
    try std.testing.expect(rigid_character.asBase().supportingVolume().normal.y > 0.9);
    try std.testing.expect(virtual_character.asBase().getShape() != null);
    try std.testing.expect(rigid_character.asBase().getShape() != null);
    try std.testing.expectApproxEqAbs(@as(f32, 1), virtual_character.asBase().up().y, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1), rigid_character.asBase().up().y, 1e-6);

    // A write through the shared surface is a write to the same character:
    // lifting the plane out of reach of every contact reads back through
    // both the base view and the concrete type.
    try virtual_character.asBase().setSupportingVolume(.{ .normal = zjolt.vec3(0, 1, 0), .distance = 100 });
    try virtual_character.refreshContacts(null);
    try std.testing.expect(!virtual_character.isSupported());
    try std.testing.expect(!settledOnGround(virtual_character.asBase()));
}

//=============================================================================
// Ground-velocity building blocks
//
// CalculateCharacterGroundVelocity and GetAdjustedBodyVelocity are private on
// JPH::CharacterVirtual (see the report) and reimplemented here from public
// state: the character's own position, a body's real velocity, and its
// installed listener's public OnAdjustBodyVelocity override.
//=============================================================================

test "getAdjustedBodyVelocity reads a body's real velocity, zero on STATIC, doubled once a listener overrides it, and BodyNotFound for a stale id" {
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

    const box_shape = try zjolt.Shape.initBox(zjolt.vec3(1, 1, 1), .{});
    defer box_shape.release();
    const bodies = world.system.bodies();
    const platform = try bodies.createAndAdd(.{
        .shape = box_shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(5, 0, 0),
    }, .dont_activate);
    bodies.setLinearAndAngularVelocity(platform, zjolt.vec3(1, 0, 2), zjolt.vec3(0, 3, 0));

    // No listener installed: adjusted equals the body's own velocity
    // exactly, the "no override" branch of GetAdjustedBodyVelocity.
    {
        const adjusted = try character.getAdjustedBodyVelocity(platform);
        try expectVec3Near(zjolt.vec3(1, 0, 2), adjusted.linear_velocity, 1e-5);
        try expectVec3Near(zjolt.vec3(0, 3, 0), adjusted.angular_velocity, 1e-5);
    }

    // A STATIC body has no velocity to read: zero, not whatever a stale
    // motion-properties read would answer.
    {
        const adjusted = try character.getAdjustedBodyVelocity(world.floor);
        try expectVec3Near(zjolt.vec3_zero, adjusted.linear_velocity, 1e-5);
        try expectVec3Near(zjolt.vec3_zero, adjusted.angular_velocity, 1e-5);
    }

    // A listener's OnAdjustBodyVelocity override is applied on top of the
    // body's real velocity -- doubling it here is the value a hand
    // computation from the callback's own contract predicts.
    const Doubler = struct {
        pub fn onAdjustBodyVelocity(
            self: *@This(),
            character_id: zjolt.CharacterId,
            body2: zjolt.BodyId,
            linear: *zjolt.Vec3,
            angular: *zjolt.Vec3,
        ) void {
            _ = .{ self, character_id, body2 };
            linear.* = zjolt.vec3(linear.x * 2, linear.y * 2, linear.z * 2);
            angular.* = zjolt.vec3(angular.x * 2, angular.y * 2, angular.z * 2);
        }
    };
    var doubler: Doubler = .{};
    var listener = zjolt.CharacterContactListener(Doubler).init(&doubler);
    try listener.attach();
    defer listener.deinit();
    try character.setListener(listener.handle);
    defer character.setListener(null) catch {};

    const adjusted = try character.getAdjustedBodyVelocity(platform);
    try expectVec3Near(zjolt.vec3(2, 0, 4), adjusted.linear_velocity, 1e-5);
    try expectVec3Near(zjolt.vec3(0, 6, 0), adjusted.angular_velocity, 1e-5);
    try listener.check();

    // A stale id is BodyNotFound, not a silent zero.
    bodies.destroy(platform);
    try std.testing.expectError(zjolt.Error.BodyNotFound, character.getAdjustedBodyVelocity(platform));
}

test "calculateGroundVelocity returns linear_velocity unchanged at zero angular velocity, and matches an independently rotated offset otherwise" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const capsule = try zjolt.Shape.initCapsule(0.5, 0.3, .{});
    defer capsule.release();
    const character = try zjolt.Character.init(world.system, .{
        .shape = capsule,
        .position = zjolt.rvec3(5, 3, 0),
    });
    defer character.deinit();

    // No rotation: CalculateCharacterGroundVelocity's early-return branch,
    // verbatim linear_velocity regardless of where the center of mass is.
    const straight = character.calculateGroundVelocity(
        zjolt.rvec3(0, 0, 0),
        zjolt.vec3(3, 0, 4),
        zjolt.vec3_zero,
        dt,
    );
    try expectVec3Near(zjolt.vec3(3, 0, 4), straight, 1e-5);

    // A platform spinning about Y with the character standing off its axis:
    // by hand, the ground velocity is linear_velocity plus the finite
    // difference of rotating (character_position - center_of_mass) by
    // angular_velocity * delta_time, divided by delta_time -- computed here
    // through Quat.fromAxisAngle/rotateVector, an independent code path from
    // calculateGroundVelocity's own internal Quat::sRotation.
    const com = zjolt.rvec3(0, 0, 0);
    const angular_velocity = zjolt.vec3(0, 1.5, 0);
    const linear_velocity = zjolt.vec3(2, 0, 0);
    const angle: f32 = 1.5 * dt;
    const rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 1, 0), angle);

    const position = character.getPosition();
    const offset = zjolt.vec3(
        @as(f32, @floatCast(position.x - com.x)),
        @as(f32, @floatCast(position.y - com.y)),
        @as(f32, @floatCast(position.z - com.z)),
    );
    const rotated_offset = try rotation.rotateVector(offset);
    const expected = zjolt.vec3(
        linear_velocity.x + (rotated_offset.x - offset.x) / dt,
        linear_velocity.y + (rotated_offset.y - offset.y) / dt,
        linear_velocity.z + (rotated_offset.z - offset.z) / dt,
    );

    const actual = character.calculateGroundVelocity(com, linear_velocity, angular_velocity, dt);
    try expectVec3Near(expected, actual, 1e-3);
}

fn expectVec3Near(expected: zjolt.Vec3, actual: zjolt.Vec3, tolerance: f32) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, tolerance);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, tolerance);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, tolerance);
}

//=============================================================================
// Active contacts
//=============================================================================

test "active contacts are countable, readable into a caller's buffer, and refuse a short one" {
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

    const settings = zjolt.defaultCharacterUpdateSettings();
    try walkForward(character, &settings, 0, 4.0);
    try std.testing.expectEqual(zjolt.GroundState.on_ground, character.groundState());

    // Standing on the floor is at least one contact, and the count is
    // readable without a buffer at all.
    const count = try character.activeContactCount();
    try std.testing.expect(count > 0);

    var buffer: [16]zjolt.CharacterContact = undefined;
    const contacts = try character.activeContacts(&buffer);
    try std.testing.expectEqual(count, @as(u32, @intCast(contacts.len)));
    try std.testing.expectEqual(@intFromPtr(&buffer), @intFromPtr(contacts.ptr));

    // A buffer that cannot hold them says so rather than truncating quietly.
    try std.testing.expectError(
        error.BufferTooSmall,
        character.activeContacts(buffer[0 .. count - 1]),
    );

    // The allocating form agrees with both, and owns what it returns.
    const owned = try character.getActiveContacts(std.testing.allocator);
    defer std.testing.allocator.free(owned);
    try std.testing.expectEqual(contacts.len, owned.len);
}
