//! A whole world, described rather than simulated.
//!
//! Elsewhere a world is built call by call: make a shape, make a body,
//! make a constraint, add each one. A scene is that same world held as
//! DATA, with nothing stepping and no `PhysicsSystem` involved. Two
//! things follow: it SERIALISES (`save` writes one buffer carrying
//! shapes and placements, loaded without re-running the code that built
//! it), and it INSTANTIATES repeatedly (`createBodies` stamps the same
//! scene into as many systems as wanted, sharing shapes by reference).
//!
//! A scene names its bodies by INDEX — ids exist only once
//! `createBodies` runs; `world_body_index` means "bolted to the world".
//! Reference counted like a `Shape`: `release` it, not `deinit`.

const std = @import("std");
const c = @import("c/scene.zig");
const err = @import("error.zig");
const body_mod = @import("body.zig");
const group_mod = @import("group.zig");
const softbody_mod = @import("softbody.zig");
const constraint_mod = @import("constraint.zig");
const system_mod = @import("system.zig");
const stream_mod = @import("stream.zig");

/// The body index that means "the world" — the same implicit, infinitely
/// heavy static body at the origin that `zjolt.world_body` names for a
/// constraint built against a live system.
///
/// It is also what `addBody` and `addSoftBody` leave in their result on
/// failure, rather than 0: index 0 is a perfectly good body.
pub const world_body_index: u32 = c.scene_body_world;

/// What a scene records about one joint. @see `Scene.constraint`.
///
/// Which KIND of joint it is is deliberately absent: a scene holds a settings
/// object, and asking one what it would build means building it. Instantiate
/// the scene and ask the `Constraint` with `subType`.
pub const SceneConstraint = c.SceneConstraint;

/// A body descriptor read back out of a scene. Every field crosses by NAME,
/// the exact inverse of `BodyDesc.toC`: a field on one side and not the other
/// is a compile error, not a value quietly dropped on the way out.
///
/// `error.BadFormat` when the entry carries no shape. Nothing this library
/// creates or restores is in that state, so that is a foreign scene's answer.
fn bodyDescFromC(raw: c.BodyDesc) err.Error!body_mod.BodyDesc {
    var out: body_mod.BodyDesc = .{
        .shape = .{ .handle = raw.shape orelse return err.Error.BadFormat },
        .object_layer = raw.object_layer,
        .collision_group = group_mod.fromC(raw.collision_group),
    };
    inline for (@typeInfo(body_mod.BodyDesc).@"struct".fields) |field| {
        if (comptime std.mem.eql(u8, field.name, "shape")) continue;
        if (comptime std.mem.eql(u8, field.name, "collision_group")) continue;
        @field(out, field.name) = @field(raw, field.name);
    }
    return out;
}

/// The `@constCast` matches the one in `group.zig`, and for the same reason:
/// Jolt stores a soft body's topology as a `RefConst`, so the C getter hands
/// back a `const` pointer, while `SharedSettings` is a mutable handle because
/// building one mutates it. Mutating settings read out of a scene is legal —
/// it is the same object the scene holds a reference to, and every soft body
/// stamped out of that scene afterwards sees the change.
fn softBodyDescFromC(raw: c.SoftBodyDesc) err.Error!softbody_mod.Desc {
    const settings = raw.shared_settings orelse return err.Error.BadFormat;
    return .{
        .shared_settings = .{ .handle = @constCast(settings) },
        .object_layer = raw.object_layer,
        .position = raw.position,
        .rotation = raw.rotation,
        .user_data = raw.user_data,
        .collision_group = group_mod.fromC(raw.collision_group),
        .num_iterations = raw.num_iterations,
        .linear_damping = raw.linear_damping,
        .max_linear_velocity = raw.max_linear_velocity,
        .restitution = raw.restitution,
        .friction = raw.friction,
        .pressure = raw.pressure,
        .gravity_factor = raw.gravity_factor,
        .vertex_radius = raw.vertex_radius,
        .update_position = raw.update_position,
        .make_rotation_identity = raw.make_rotation_identity,
        .allow_sleeping = raw.allow_sleeping,
        .faces_double_sided = raw.faces_double_sided,
    };
}

pub const Scene = struct {
    handle: *c.Scene,

    //=========================================================================
    // Lifetime
    //=========================================================================

    /// An empty scene.
    pub fn init() err.Error!Scene {
        var handle: *c.Scene = undefined;
        try err.check(c.zjoltSceneCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn addRef(self: Scene) void {
        c.zjoltSceneAddRef(self.handle);
    }

    /// Drops one reference. The scene, and its references to the shapes in
    /// it, go when the last one does.
    pub fn release(self: Scene) void {
        c.zjoltSceneRelease(self.handle);
    }

    pub fn refCount(self: Scene) u32 {
        return c.zjoltSceneGetRefCount(self.handle);
    }

    //=========================================================================
    // Filling one in
    //
    // Descriptors are the ones `BodyInterface.create`/`createSoftBody`
    // take, and are copied; the scene keeps only a reference on the shape (or shared settings), so the caller may release theirs right after the call.
    //=========================================================================

    /// Appends a rigid body and returns its index — what a constraint added
    /// afterwards refers to it by.
    pub fn addBody(self: Scene, desc: body_mod.BodyDesc) err.Error!u32 {
        const raw = body_mod.descToC(desc);
        var index: u32 = world_body_index;
        try err.check(c.zjoltSceneAddBody(self.handle, &raw, &index));
        return index;
    }

    /// Appends a soft body and returns its index. Soft bodies are numbered
    /// SEPARATELY from rigid ones, and a constraint cannot name one: Jolt's
    /// scene attaches joints to the rigid body list alone.
    pub fn addSoftBody(self: Scene, desc: softbody_mod.Desc) err.Error!u32 {
        const raw = softbody_mod.descToC(desc);
        var index: u32 = world_body_index;
        try err.check(c.zjoltSceneAddSoftBody(self.handle, &raw, &index));
        return index;
    }

    /// Appends the joint `joint` describes, between bodies `body1`/`body2`
    /// (indices into this scene, or `world_body_index`). Read, not kept:
    /// release `constraint` and its bodies right after — later changes to it do
    /// not reach this scene's copy. `error.Unsupported` for a joint not between
    /// two bodies (a vehicle constraint); `error.InvalidArgument` for equal
    /// ends, or a body index this scene lacks (bodies must be added first).
    pub fn addConstraint(
        self: Scene,
        joint: constraint_mod.Constraint,
        body1: u32,
        body2: u32,
    ) err.Error!void {
        try err.check(c.zjoltSceneAddConstraint(
            self.handle,
            joint.handle,
            body1,
            body2,
        ));
    }

    /// Appends everything in `system` — every body, and every two-body
    /// constraint between them — to what this scene already holds: the other
    /// direction, a world built call by call, captured to be saved. Jolt marks
    /// this a debugging aid; reads without stepping. `error.BodyNotFound` for a
    /// constraint attached to a body the system no longer holds; checked whole
    /// before appending, so a refusal leaves the scene as it was.
    pub fn captureFrom(self: Scene, system: system_mod.PhysicsSystem) err.Error!void {
        try err.check(c.zjoltSceneFromPhysicsSystem(self.handle, system.handle));
    }

    //=========================================================================
    // Reading one back
    //
    // Three separate counts and three separate index spaces: body 0, soft
    // body 0 and constraint 0 are three different things.
    //=========================================================================

    pub fn bodyCount(self: Scene) u32 {
        return c.zjoltSceneGetNumBodies(self.handle);
    }

    pub fn softBodyCount(self: Scene) u32 {
        return c.zjoltSceneGetNumSoftBodies(self.handle);
    }

    pub fn constraintCount(self: Scene) u32 {
        return c.zjoltSceneGetNumConstraints(self.handle);
    }

    /// The body at `index`. `error.InvalidArgument` past `bodyCount`. `shape`
    /// is BORROWED from the scene, no reference taken — `addRef` it to outlive
    /// this entry. Every other field round-trips through `addBody`, Jolt's
    /// provided-tensor mass mode included.
    pub fn body(self: Scene, index: u32) err.Error!body_mod.BodyDesc {
        var raw: c.BodyDesc = undefined;
        try err.check(c.zjoltSceneGetBody(self.handle, index, &raw));
        return bodyDescFromC(raw);
    }

    /// The soft body at `index`. Its `shared_settings` are borrowed on the
    /// same terms as `body`'s shape.
    pub fn softBody(self: Scene, index: u32) err.Error!softbody_mod.Desc {
        var raw: c.SoftBodyDesc = undefined;
        try err.check(c.zjoltSceneGetSoftBody(self.handle, index, &raw));
        return softBodyDescFromC(raw);
    }

    /// The joint at `index`: which two bodies it connects, and the settings
    /// every kind of joint has in common.
    pub fn constraint(self: Scene, index: u32) err.Error!SceneConstraint {
        var out: SceneConstraint = undefined;
        try err.check(c.zjoltSceneGetConstraint(self.handle, index, &out));
        return out;
    }

    //=========================================================================
    // Turning one into a world
    //=========================================================================

    /// Creates every body and soft body in this scene in `system`, adds
    /// them activated, then creates and adds every constraint. The
    /// scene is unchanged and reusable; bodies share its shapes.
    /// `error.OutOfMemory` mid-way is NOT clean: bodies created before
    /// it stay in the system and no constraint is created at all — give
    /// `system` a `max_bodies` well past `bodyCount` + `softBodyCount`.
    pub fn createBodies(self: Scene, system: system_mod.PhysicsSystem) err.Error!void {
        try err.check(c.zjoltSceneCreateBodies(self.handle, system.handle));
    }

    /// Rescales any shape in the scene that cannot legally be used at unit
    /// scale, replacing it in place — CHANGES the scene, so a shape borrowed
    /// from an earlier `body` call may no longer be the scene's. For a scaled
    /// shape baked into an asset (a scaled convex hull, say) that Jolt refuses
    /// to build a body from — common in a level loaded from a file.
    /// `error.ShapeInvalid` if some could not be fixed; the rest still were.
    pub fn fixInvalidScales(self: Scene) err.Error!void {
        try err.check(c.zjoltSceneFixInvalidScales(self.handle));
    }

    //=========================================================================
    // Serialisation
    //
    // Same container as `Shape.save`/`State.save` (checksummed, versioned) — same caveat: rejects a bad file, not a defence against a crafted payload with a matching checksum.
    // NOT written: rigid-body/constraint `user_data` (comes back 0, unlike a SOFT body's); collision group FILTERS (re-attach via `BodyInterface.setCollisionGroup` after instantiating).
    //=========================================================================

    /// Bytes `save` would write. Ask each time rather than caching it.
    pub fn saveSize(self: Scene) err.Error!usize {
        var size: usize = 0;
        try err.check(c.zjoltSceneSave(self.handle, null, 0, &size));
        return size;
    }

    /// Serialises into `buffer`, returning the slice actually written.
    /// `error.BufferTooSmall` if it does not fit; use `saveSize` first.
    pub fn save(self: Scene, buffer: []u8) err.Error![]u8 {
        var size: usize = 0;
        try err.check(c.zjoltSceneSave(self.handle, buffer.ptr, buffer.len, &size));
        return buffer[0..size];
    }

    /// Allocates and serialises in one step. The caller owns the slice.
    pub fn saveAlloc(self: Scene, gpa: std.mem.Allocator) ![]u8 {
        const size = try self.saveSize();
        const buffer = try gpa.alloc(u8, size);
        errdefer gpa.free(buffer);
        return try self.save(buffer);
    }

    /// Rebuilds a scene from `save` output; `release` it when done.
    /// `error.BadFormat` for anything the container refuses (wrong buffer,
    /// different build/Jolt, truncated, bad checksum) or a payload Jolt itself
    /// rejects (message in `lastError`). Shapes come back newly built, NOT
    /// shared with what was saved — two scenes from one buffer hold two sets of
    /// shapes.
    pub fn restore(data: []const u8) err.Error!Scene {
        var handle: *c.Scene = undefined;
        try err.check(c.zjoltSceneRestore(data.ptr, data.len, &handle));
        return .{ .handle = handle };
    }

    /// `save`, through `stream` instead of a resident buffer — for
    /// streaming a cooked level to a pack file, socket or compressor
    /// rather than sizing and holding it whole. @see `zjolt.hostStream`.
    /// No length or checksum ahead of the payload (only a magic tag and
    /// build identity) — the same reduced margin against a corrupted
    /// stream as `Shape.saveStream`. `error.IoError` if `stream` fails.
    pub fn saveStream(self: Scene, stream: stream_mod.Stream) err.Error!void {
        try err.check(c.zjoltSceneSaveStream(self.handle, &stream));
    }

    /// Rebuilds a scene written by `saveStream`. @see `restore` for what
    /// `error.BadFormat` covers; a stream form has no length or checksum to
    /// check first.
    pub fn restoreStream(stream: stream_mod.Stream) err.Error!Scene {
        var handle: *c.Scene = undefined;
        try err.check(c.zjoltSceneRestoreStream(&stream, &handle));
        return .{ .handle = handle };
    }

    //=========================================================================
    // Jolt's own object stream
    //
    // Older, human-readable format for a C++ host of vanilla Jolt (`error.Unsupported` without `-Dobject_stream=true`).
    // A RIGID body's shape does NOT survive (a SOFT body's does) — `body()` on such a scene refuses `error.BadFormat`; use `save`/`saveStream` when a shape must survive.
    //=========================================================================

    /// Writes this scene through `stream` in Jolt's own object-stream
    /// format — unlike `saveStream`, exactly what a C++ host of vanilla
    /// Jolt would write: no zjolt header, no build check on the way
    /// back in, since the point is interchange with a plain-Jolt editor.
    ///
    /// `error.IoError` if `stream` reports failure while this runs.
    pub fn saveObjectStream(
        self: Scene,
        format: stream_mod.ObjectStreamFormat,
        stream: stream_mod.Stream,
    ) err.Error!void {
        try err.check(c.zjoltSceneSaveObjectStream(self.handle, format, &stream));
    }

    /// Reads a scene written by `saveObjectStream`, or by a C++ host of vanilla
    /// Jolt — either form, sniffed the way Jolt's own reader does.
    ///
    /// `error.BadFormat` when Jolt's own reader refuses the stream; unlike
    /// `restore`, `lastError` carries no detail — the object stream reports
    /// only success or failure, not why.
    pub fn restoreObjectStream(stream: stream_mod.Stream) err.Error!Scene {
        var handle: *c.Scene = undefined;
        try err.check(c.zjoltSceneRestoreObjectStream(&stream, &handle));
        return .{ .handle = handle };
    }
};

//=============================================================================
// Tests
//=============================================================================

const zjolt = @import("zjolt.zig");

const TestLayers = struct {
    pub const static: body_mod.ObjectLayer = 0;
    pub const moving: body_mod.ObjectLayer = 1;

    pub const bp_static: system_mod.BroadPhaseLayer = 0;
    pub const bp_moving: system_mod.BroadPhaseLayer = 1;

    pub fn broadPhaseLayerCount() u32 {
        return 2;
    }

    pub fn broadPhaseLayerFor(layer: body_mod.ObjectLayer) system_mod.BroadPhaseLayer {
        return if (layer == static) bp_static else bp_moving;
    }

    pub fn objectCanCollideWithBroadPhase(
        object: body_mod.ObjectLayer,
        broad: system_mod.BroadPhaseLayer,
    ) bool {
        return if (object == static) broad == bp_moving else true;
    }

    pub fn objectsCanCollide(a: body_mod.ObjectLayer, b: body_mod.ObjectLayer) bool {
        return if (a == static) b == moving else true;
    }
};

test "a scene survives a round trip through a buffer and back into a world" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const left = zjolt.rvec3(-1, 4, 0);
    const right = zjolt.rvec3(2, 4, 0);

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.25, 0.25, 0.25), .{});
    defer shape.release();

    // The scene is authored against a system that is then thrown away: it is
    // only there because a joint reaches a scene through a live constraint.
    const buffer = blk: {
        const authoring = try zjolt.PhysicsSystem.init(.{
            .layers = zjolt.layersFromType(TestLayers),
            .max_bodies = 64,
        });
        defer authoring.deinit();

        const scene = try Scene.init();
        defer scene.release();

        const bodies = authoring.bodies();
        const a = try bodies.createAndAdd(.{
            .shape = shape,
            .object_layer = TestLayers.moving,
            .position = left,
            .user_data = 0xA1,
        }, .activate);
        const b = try bodies.createAndAdd(.{
            .shape = shape,
            .object_layer = TestLayers.moving,
            .position = right,
            .user_data = 0xB2,
        }, .activate);

        try std.testing.expectEqual(@as(u32, 0), try scene.addBody(.{
            .shape = shape,
            .object_layer = TestLayers.moving,
            .position = left,
            .user_data = 0xA1,
        }));
        try std.testing.expectEqual(@as(u32, 1), try scene.addBody(.{
            .shape = shape,
            .object_layer = TestLayers.moving,
            .position = right,
            .user_data = 0xB2,
        }));

        const rod = try zjolt.Constraint.initDistance(authoring, a, b, .{
            .point1 = left,
            .point2 = right,
            .min_distance = 3,
            .max_distance = 3,
        });
        defer rod.release();
        try scene.addConstraint(rod, 0, 1);

        // A joint to itself is the mistake Jolt asserts on rather than
        // refusing, so it is refused here.
        try std.testing.expectError(
            err.Error.InvalidArgument,
            scene.addConstraint(rod, 1, 1),
        );

        break :blk try scene.saveAlloc(std.testing.allocator);
    };
    defer std.testing.allocator.free(buffer);

    const loaded = try Scene.restore(buffer);
    defer loaded.release();

    try std.testing.expectEqual(@as(u32, 2), loaded.bodyCount());
    try std.testing.expectEqual(@as(u32, 0), loaded.softBodyCount());
    try std.testing.expectEqual(@as(u32, 1), loaded.constraintCount());

    const restored_desc = try loaded.body(1);
    // Pinned rather than assumed: Jolt does not write a rigid body's user data
    // (it does write a soft body's), so this is 0 and not 0xB2. A Jolt upgrade
    // that starts saving it should fail here and be documented, not silently
    // change what a level carries.
    try std.testing.expectEqual(@as(u64, 0), restored_desc.user_data);
    try std.testing.expectApproxEqAbs(
        @as(f64, @floatCast(right.x)),
        @as(f64, @floatCast(restored_desc.position.x)),
        1e-4,
    );

    const joint = try loaded.constraint(0);
    try std.testing.expectEqual(@as(u32, 0), joint.body1);
    try std.testing.expectEqual(@as(u32, 1), joint.body2);
    try std.testing.expect(joint.enabled);

    // And it becomes a world again, with the bodies where they were.
    const system = try zjolt.PhysicsSystem.init(.{
        .layers = zjolt.layersFromType(TestLayers),
        .max_bodies = 64,
    });
    defer system.deinit();

    try loaded.createBodies(system);
    try std.testing.expectEqual(@as(u32, 2), system.numBodies());
    try std.testing.expectEqual(@as(u32, 1), zjolt.constraintCount(system));

    var ids: [4]zjolt.BodyId = undefined;
    const created = try system.getBodies(&ids);
    try std.testing.expectEqual(@as(usize, 2), created.len);

    const bodies = system.bodies();
    const expected_positions = [_]zjolt.RVec3{ left, right };
    for (created, &expected_positions) |id, expected| {
        const position = bodies.getPosition(id);
        try std.testing.expectApproxEqAbs(
            @as(f64, @floatCast(expected.x)),
            @as(f64, @floatCast(position.x)),
            1e-3,
        );
        try std.testing.expectApproxEqAbs(
            @as(f64, @floatCast(expected.y)),
            @as(f64, @floatCast(position.y)),
            1e-3,
        );
    }

    // Capturing the world back into a scene agrees with what was stamped out
    // of it, which is the one thing that says the two directions match.
    const captured = try Scene.init();
    defer captured.release();
    try captured.captureFrom(system);
    try std.testing.expectEqual(@as(u32, 2), captured.bodyCount());
    try std.testing.expectEqual(@as(u32, 1), captured.constraintCount());
}

test "a scene's text object stream is diffable text, and its scalar fields round-trip through a host stream" {
    if (!zjolt.options.object_stream) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.25, 0.25, 0.25), .{});
    defer shape.release();

    const scene = try Scene.init();
    defer scene.release();
    _ = try scene.addBody(.{
        .shape = shape,
        .object_layer = TestLayers.moving,
        .position = zjolt.rvec3(1, 2, 3),
    });

    var stream_buffer: [1 << 16]u8 = undefined;
    var writer: stream_mod.BufferWriter = .{ .buffer = &stream_buffer };
    try scene.saveObjectStream(.text, stream_mod.hostStream(stream_mod.BufferWriter, &writer));
    const text = writer.slice();

    // The point of the text form: every field is written by name rather than
    // packed, which is what makes it something a person can open and diff —
    // `Scene.save`'s checksummed binary payload has no equivalent at any
    // setting. "mPosition" is the exact attribute name
    // `JPH_ADD_ATTRIBUTE(BodyCreationSettings, mPosition)` registers.
    try std.testing.expect(std.mem.indexOf(u8, text, "mPosition") != null);

    var reader: stream_mod.BufferReader = .{ .buffer = text };
    const loaded = try Scene.restoreObjectStream(stream_mod.hostStream(stream_mod.BufferReader, &reader));
    defer loaded.release();
    try std.testing.expectEqual(@as(u32, 1), loaded.bodyCount());

    // A rigid body's SHAPE does not survive this round trip — Jolt's own
    // registration, not a gap here: the object stream serialises only
    // `BodyCreationSettings::mShape` (unbuilt ShapeSettings), and this ABI
    // always builds from `mShapePtr`, documented upstream as "cannot be
    // serialized". `body()` itself refuses a shapeless entry, so this reads
    // the raw descriptor to show what actually comes back.
    var raw: c.BodyDesc = undefined;
    try err.check(c.zjoltSceneGetBody(loaded.handle, 0, &raw));
    try std.testing.expect(raw.shape == null);
    try std.testing.expectApproxEqAbs(@as(f64, 1), @as(f64, @floatCast(raw.position.x)), 1e-3);
    try std.testing.expectApproxEqAbs(@as(f64, 2), @as(f64, @floatCast(raw.position.y)), 1e-3);
    try std.testing.expectApproxEqAbs(@as(f64, 3), @as(f64, @floatCast(raw.position.z)), 1e-3);
}

test "the object stream reports Unsupported without -Dobject_stream=true, not silently doing nothing" {
    if (zjolt.options.object_stream) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const scene = try Scene.init();
    defer scene.release();

    var stream_buffer: [64]u8 = undefined;
    var writer: stream_mod.BufferWriter = .{ .buffer = &stream_buffer };
    try std.testing.expectError(
        err.Error.Unsupported,
        scene.saveObjectStream(.text, stream_mod.hostStream(stream_mod.BufferWriter, &writer)),
    );

    var reader: stream_mod.BufferReader = .{ .buffer = &stream_buffer };
    try std.testing.expectError(
        err.Error.Unsupported,
        Scene.restoreObjectStream(stream_mod.hostStream(stream_mod.BufferReader, &reader)),
    );
}
