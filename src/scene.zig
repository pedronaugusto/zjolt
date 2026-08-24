//! A whole world, described rather than simulated.
//!
//! Everywhere else here a world is built call by call: make a shape, make a
//! body, make a constraint, add each one. A scene is that same world held as
//! DATA — bodies, soft bodies, and the joints between them — with nothing
//! stepping and no `PhysicsSystem` involved. Two things follow, and they are
//! the whole reason it exists:
//!
//! * it serialises. `save` writes one buffer carrying the shapes as well as
//!   the placements, so a level can be cooked once and loaded without
//!   re-running the code that built it.
//! * it instantiates repeatedly. `createBodies` stamps the same scene into as
//!   many systems as you like, and the second one costs no shape
//!   construction: the bodies share the scene's shapes by reference.
//!
//! A scene names its bodies by INDEX, because nothing in it has a `BodyId`
//! yet — ids come into existence when `createBodies` runs. Constraints
//! therefore connect index to index, and `world_body_index` is the end that
//! means "bolted to the world".
//!
//! Reference counted like a `Shape`: `release` it, not `deinit`.

const std = @import("std");
const c = @import("c/scene.zig");
const err = @import("error.zig");
const body_mod = @import("body.zig");
const group_mod = @import("group.zig");
const softbody_mod = @import("softbody.zig");
const constraint_mod = @import("constraint.zig");
const system_mod = @import("system.zig");

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

/// A body descriptor read back out of a scene.
///
/// `error.BadFormat` when the entry carries no shape. Nothing this library
/// creates or restores is in that state — a buffer whose bodies lost their
/// shapes is refused by `Scene.restore` — so this is the honest answer for a
/// scene that came from somewhere else entirely.
fn bodyDescFromC(raw: c.BodyDesc) err.Error!body_mod.BodyDesc {
    return .{
        .shape = .{ .handle = raw.shape orelse return err.Error.BadFormat },
        .object_layer = raw.object_layer,
        .position = raw.position,
        .rotation = raw.rotation,
        .linear_velocity = raw.linear_velocity,
        .angular_velocity = raw.angular_velocity,
        .user_data = raw.user_data,
        .collision_group = group_mod.fromC(raw.collision_group),
        .motion_type = raw.motion_type,
        .motion_quality = raw.motion_quality,
        .allowed_dofs = raw.allowed_dofs,
        .override_mass_properties = raw.override_mass_properties,
        .mass = raw.mass,
        .allow_dynamic_or_kinematic = raw.allow_dynamic_or_kinematic,
        .is_sensor = raw.is_sensor,
        .allow_sleeping = raw.allow_sleeping,
        .enhanced_internal_edge_removal = raw.enhanced_internal_edge_removal,
        .friction = raw.friction,
        .restitution = raw.restitution,
        .linear_damping = raw.linear_damping,
        .angular_damping = raw.angular_damping,
        .max_linear_velocity = raw.max_linear_velocity,
        .max_angular_velocity = raw.max_angular_velocity,
        .gravity_factor = raw.gravity_factor,
    };
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
    // The descriptors are the ones `BodyInterface.create` and
    // `createSoftBody` take, and they are copied. What the scene keeps is a
    // reference on the shape (or the shared settings), so the caller may
    // release theirs as soon as the call returns.
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

    /// Appends the joint `joint` describes, between the bodies at `body1`
    /// and `body2` — indices into this scene, or `world_body_index`.
    ///
    /// The constraint is read, not kept: what the scene stores is the
    /// settings Jolt builds from it, so the caller may release `constraint`
    /// and destroy the bodies it was created against straight afterwards, and
    /// changing the live constraint later does not change the scene's copy.
    ///
    /// A live constraint is how a joint gets into a scene because this
    /// library has no settings object of its own to hand over. Create the
    /// joint against real bodies with any `Constraint.init*`, add it here,
    /// release it; there is no need to have added it to a system first.
    ///
    /// `error.Unsupported` for a joint that is not between two bodies (a
    /// vehicle constraint is the one this library builds that is not).
    /// `error.InvalidArgument` when the two ends are the same, or when either
    /// names a body this scene does not have yet — add the bodies first, a
    /// constraint cannot forward-reference one.
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
    /// constraint between them — to what this scene already holds. The other
    /// direction: a world built call by call, captured so it can be saved.
    ///
    /// Jolt marks this as a debugging aid, and it reads the system without
    /// stepping it, so call it between steps.
    ///
    /// `error.BodyNotFound` when a constraint in `system` is attached to a
    /// body the system no longer holds; the whole system is checked before
    /// anything is appended, so a refusal leaves this scene as it was.
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

    /// The body at `index`. `error.InvalidArgument` past `bodyCount`.
    ///
    /// The `shape` in the result is BORROWED from the scene: no reference is
    /// taken on the way out, and it dies with the scene entry unless you
    /// `addRef` it yourself.
    ///
    /// One field of this view is lossy. Jolt has a third mass mode — an
    /// inertia tensor supplied outright — that `OverrideMassProperties`
    /// cannot spell, and a scene loaded from a file may carry it. Such a body
    /// reports `.calculate_inertia` with the mass it was given, the closest
    /// this descriptor comes; the scene still instantiates the real tensor,
    /// so what `createBodies` builds is unaffected. Round-tripping a body
    /// through this call and `addBody` is what loses it.
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

    /// Creates every body and soft body in this scene in `system`, adds them
    /// all activated, then creates and adds every constraint.
    ///
    /// The scene is unchanged and can be instantiated again, into this system
    /// or another. The bodies share its shapes rather than copying them.
    ///
    /// `error.OutOfMemory` when `system` runs out of body slots part way
    /// through, and that failure is not clean: the bodies created before it
    /// stay in the system, and no constraint is created at all — Jolt stops
    /// before them, because indices into a half-created body list would join
    /// the wrong things. Give a system that is to hold a scene a `max_bodies`
    /// well past `bodyCount` plus `softBodyCount`.
    ///
    /// `error.InvalidArgument`, before anything is created, when the scene
    /// itself cannot be instantiated: a body with no shape, a soft body with
    /// no topology, a constraint with no settings or naming a body index that
    /// does not exist. None of those is reachable through the `add*` calls;
    /// all of them are reachable through a hand-made buffer.
    pub fn createBodies(self: Scene, system: system_mod.PhysicsSystem) err.Error!void {
        try err.check(c.zjoltSceneCreateBodies(self.handle, system.handle));
    }

    /// Rescales any shape in the scene that cannot legally be used at unit
    /// scale, replacing it in place — so this CHANGES the scene, and a shape
    /// borrowed from an earlier `body` call may no longer be the scene's.
    ///
    /// The case it exists for is a scaled shape baked into an asset: a scaled
    /// convex hull, or any shape scaled by a factor its kind forbids. Jolt
    /// refuses to build a body from one, and a level loaded from a file is
    /// exactly where one turns up.
    ///
    /// `error.ShapeInvalid` when some shape could not be fixed; the ones that
    /// could be still were.
    pub fn fixInvalidScales(self: Scene) err.Error!void {
        try err.check(c.zjoltSceneFixInvalidScales(self.handle));
    }

    //=========================================================================
    // Serialisation
    //
    // The same container `Shape.save` and `State.save` write — a magic tag of
    // its own, a container version, this build's config id, the Jolt version,
    // the payload length and a CRC-32, all checked before Jolt reads a byte —
    // and the same caveat: it rejects the wrong file, a truncated file and a
    // damaged file, and it is not a defence against a crafted payload that
    // carries a matching checksum.
    //
    // Two fields Jolt does not write, worth knowing before a level leans on
    // them: a rigid body's `user_data` and a constraint's `user_data`. Neither
    // is in Jolt's own binary state, so both come back as 0 — while a SOFT
    // body's user data IS written, which is what makes the asymmetry easy to
    // miss. Keep what a host needs after a load in the level's own data, keyed
    // by index, not in a body's user data.
    //
    // Collision group FILTERS are not written either. This library's filter is
    // not one of Jolt's serialisable types, and a payload naming it could not
    // be restored; a restored body keeps its group and sub-group ids and comes
    // back with no filter. Re-attach one with `BodyInterface.setCollisionGroup`
    // after instantiating.
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
    ///
    /// `error.BadFormat` for anything the container refuses — not a zjolt
    /// scene buffer, a different build, a different Jolt, truncated, trailing
    /// bytes, a failed checksum — and for a payload Jolt itself rejects, with
    /// its message in `lastError`.
    ///
    /// The shapes come back newly built and owned by the new scene, NOT
    /// shared with whatever was saved: two scenes restored from one buffer
    /// hold two sets of shapes.
    pub fn restore(data: []const u8) err.Error!Scene {
        var handle: *c.Scene = undefined;
        try err.check(c.zjoltSceneRestore(data.ptr, data.len, &handle));
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
