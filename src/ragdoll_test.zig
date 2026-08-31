//! Behavioural tests for ragdolls: a two-part chain (root and child, joined
//! by a swing-twist constraint) that falls under gravity, is driven toward a
//! target pose by its motors, or is released while still in the world.
//!
//! Reuses `integration_test.zig`'s layer map and floor fixture rather than
//! building its own — see `BINDING.md`.
//!
//! `RagdollSettings.createRagdoll` hands back a `Ragdoll`, not the body ids
//! it spawned, so every test below recovers them the same way: read every
//! body id in the system before spawning, read them again after, and the new
//! ones are the ragdoll's parts. `zjoltPhysicsSystemGetBodies` promises no
//! order, so this diff — not position in the list — is what ties an id back
//! to the part that owns it.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");
// `SkeletalAnimation` is not re-exported from `zjolt.zig` yet, so it is
// reached through `ragdoll.zig` directly. Named apart from the many local
// `ragdoll` variables (spawned `Ragdoll` instances) below.
const ragdoll_mod = @import("ragdoll.zig");

const Layers = fixture.Layers;
const World = fixture.World;

//=============================================================================
// Fixture
//=============================================================================

/// A root joint and one child, joined by a swing-twist constraint at the
/// midpoint between them. The smallest chain that has a parent-child
/// relationship at all.
const Chain = struct {
    skeleton: zjolt.Skeleton,
    settings: zjolt.RagdollSettings,
    root_shape: zjolt.Shape,
    child_shape: zjolt.Shape,

    const root_position = zjolt.rvec3(0, 6, 0);
    const child_position = zjolt.rvec3(0, 7, 0);
    const joint_position = zjolt.rvec3(0, 6.5, 0);

    fn build(root_motion: zjolt.MotionType) !Chain {
        var skeleton = try zjolt.Skeleton.init();
        errdefer skeleton.release();
        const root_index = try skeleton.addJoint("root", null);
        _ = try skeleton.addJoint("child", root_index);
        try std.testing.expect(skeleton.areJointsCorrectlyOrdered());

        const root_shape = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.4, 0.3), .{});
        errdefer root_shape.release();
        const child_shape = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.4, 0.3), .{});
        errdefer child_shape.release();

        var settings = try zjolt.RagdollSettings.init();
        errdefer settings.release();

        const parts = [_]zjolt.RagdollPartDesc{
            .{
                .body = .{
                    .shape = root_shape,
                    .object_layer = Layers.moving,
                    .position = root_position,
                    .motion_type = root_motion,
                },
                .to_parent = null,
            },
            .{
                .body = .{
                    .shape = child_shape,
                    .object_layer = Layers.moving,
                    .position = child_position,
                },
                .to_parent = .{
                    .position1 = joint_position,
                    .twist_axis1 = zjolt.vec3(0, 1, 0),
                    .plane_axis1 = zjolt.vec3(1, 0, 0),
                    .position2 = joint_position,
                    .twist_axis2 = zjolt.vec3(0, 1, 0),
                    .plane_axis2 = zjolt.vec3(1, 0, 0),
                    // Generous on purpose: wide enough that no test below
                    // is really exercising the limits, only the
                    // settle-or-drive behaviour the limits sit around.
                    .normal_half_cone_angle = 1.0,
                    .plane_half_cone_angle = 1.0,
                    .twist_min_angle = -1.0,
                    .twist_max_angle = 1.0,
                },
            },
        };

        try settings.build(std.testing.allocator, skeleton, &parts);
        _ = settings.stabilize();
        settings.disableParentChildCollisions();
        settings.calculateBodyIndexToConstraintIndex();

        return .{
            .skeleton = skeleton,
            .settings = settings,
            .root_shape = root_shape,
            .child_shape = child_shape,
        };
    }

    fn deinit(self: *Chain) void {
        self.settings.release();
        self.skeleton.release();
        self.root_shape.release();
        self.child_shape.release();
    }
};

/// The ids of the two bodies `createRagdoll` just spawned, found by diffing
/// the system's body list against what it held before.
fn newBodyIds(before: []const zjolt.BodyId, after: []const zjolt.BodyId, out: *[2]zjolt.BodyId) !void {
    var found: usize = 0;
    for (after) |id| {
        var seen = false;
        for (before) |old| {
            if (old == id) seen = true;
        }
        if (!seen) {
            try std.testing.expect(found < out.len);
            out[found] = id;
            found += 1;
        }
    }
    try std.testing.expectEqual(out.len, found);
}

//=============================================================================
// Settling
//=============================================================================

test "a ragdoll added to a world settles under gravity without exploding" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    var chain = try Chain.build(.dynamic);
    defer chain.deinit();

    var before_buf: [8]zjolt.BodyId = undefined;
    const before = try world.system.getBodies(&before_buf);

    var ragdoll = try chain.settings.createRagdoll(world.system, 1, 0);
    defer ragdoll.release();
    ragdoll.addToPhysicsSystem(.activate, true);

    var after_buf: [8]zjolt.BodyId = undefined;
    const after = try world.system.getBodies(&after_buf);

    var parts: [2]zjolt.BodyId = undefined;
    try newBodyIds(before, after, &parts);

    try world.stepFor(3.0);

    const bodies = world.system.bodies();
    var lowest: zjolt.Real = std.math.floatMax(zjolt.Real);
    for (parts) |id| {
        const position = bodies.getPosition(id);
        const rotation = bodies.getRotation(id);

        inline for (.{ position.x, position.y, position.z }) |component| {
            try std.testing.expect(std.math.isFinite(@as(f64, @floatCast(component))));
            // A sane bound, not a tight one: this is "did not explode", not
            // "landed exactly here".
            try std.testing.expect(@abs(@as(f64, @floatCast(component))) < 50.0);
        }
        inline for (.{ rotation.x, rotation.y, rotation.z, rotation.w }) |component| {
            try std.testing.expect(std.math.isFinite(component));
        }
        // A unit quaternion stays a unit quaternion; NaN aside, a magnitude
        // running away is the other shape "exploded" takes.
        const length = @sqrt(rotation.x * rotation.x + rotation.y * rotation.y +
            rotation.z * rotation.z + rotation.w * rotation.w);
        try std.testing.expectApproxEqAbs(@as(f32, 1), length, 0.05);

        if (position.y < lowest) lowest = position.y;
    }

    // And it did something: gravity pulled it down from where it started,
    // toward the fixture's floor at y = 0. A ragdoll that never moved would
    // pass the checks above by doing nothing at all.
    try std.testing.expect(lowest < Chain.root_position.y - 1.0);
}

//=============================================================================
// Driving
//=============================================================================

test "driveToPoseUsingMotors moves the pose toward the target rather than away" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();
    // Isolates the motor: nothing here is about gravity, and a kinematic
    // root that gravity cannot move anyway removes it as a variable on the
    // parent side of the joint too.
    world.system.setGravity(zjolt.vec3_zero);

    var chain = try Chain.build(.kinematic);
    defer chain.deinit();

    var before_buf: [8]zjolt.BodyId = undefined;
    const before = try world.system.getBodies(&before_buf);

    var ragdoll = try chain.settings.createRagdoll(world.system, 1, 0);
    defer ragdoll.release();
    ragdoll.addToPhysicsSystem(.activate, true);

    var after_buf: [8]zjolt.BodyId = undefined;
    const after = try world.system.getBodies(&after_buf);

    var parts: [2]zjolt.BodyId = undefined;
    try newBodyIds(before, after, &parts);

    const bodies = world.system.bodies();
    var child: zjolt.BodyId = zjolt.invalid_body_id;
    for (parts) |id| {
        if (bodies.getMotionType(id) == .dynamic) child = id;
    }
    try std.testing.expect(child != zjolt.invalid_body_id);

    // A 0.7 radian tilt off the chain's resting orientation — comfortably
    // inside the cone `Chain.build` set up.
    const target_rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 0, 1), 0.7);

    var pose = try zjolt.SkeletonPose.init();
    defer pose.deinit();
    try pose.setSkeleton(chain.skeleton);
    try std.testing.expectEqual(@as(u32, 2), pose.jointCount());

    const rotations = [_]zjolt.Quat{ zjolt.quat_identity, target_rotation };
    const translations = [_]zjolt.Vec3{ zjolt.vec3_zero, zjolt.vec3_zero };
    try pose.setJoints(&rotations, &translations);

    const before_angle = quatAngle(bodies.getRotation(child), target_rotation);
    // The child starts at the chain's resting orientation, not already at
    // the target — otherwise "moved toward it" would be trivially true.
    try std.testing.expect(before_angle > 0.3);

    try ragdoll.driveToPoseUsingMotors(pose);

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 1.5) : (elapsed += dt) {
        _ = try world.system.step(dt, 1, world.jobs);
    }

    const after_angle = quatAngle(bodies.getRotation(child), target_rotation);

    // Closer to the target than it started, by a wide margin rather than a
    // rounding-error's worth — and close to the target outright, not merely
    // headed in its direction.
    try std.testing.expect(after_angle < before_angle * 0.5);
    try std.testing.expect(after_angle < 0.15);
}

//=============================================================================
// Teardown
//=============================================================================

test "releasing a still-added ragdoll removes it before destroying its bodies" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    var chain = try Chain.build(.dynamic);
    defer chain.deinit();

    const handles_before = zjolt.liveHandleCount();
    var before_buf: [8]zjolt.BodyId = undefined;
    const before = try world.system.getBodies(&before_buf);
    const bodies_before = world.system.numBodies();

    // The other side of the same decision first, because it is what makes
    // the removal conditional rather than unconditional: a ragdoll that never
    // reached the simulation must not be removed on the way out. Jolt takes
    // the constraints out before the bodies and asserts each one was added,
    // so removing a never-added ragdoll is its own abort.
    var never_added = try chain.settings.createRagdoll(world.system, 2, 0);
    never_added.release();
    try std.testing.expectEqual(handles_before, zjolt.liveHandleCount());
    try std.testing.expectEqual(bodies_before, world.system.numBodies());

    var ragdoll = try chain.settings.createRagdoll(world.system, 1, 0);
    // A ragdoll is a counted live handle now, not only a Jolt reference
    // count, because the wrapper that carries its physics system is storage
    // zjolt owns. `deinit` refuses while one is outstanding.
    try std.testing.expectEqual(handles_before + 1, zjolt.liveHandleCount());

    ragdoll.addToPhysicsSystem(.activate, true);

    var after_buf: [8]zjolt.BodyId = undefined;
    const after = try world.system.getBodies(&after_buf);
    var parts: [2]zjolt.BodyId = undefined;
    try newBodyIds(before, after, &parts);

    const bodies = world.system.bodies();
    for (parts) |id| try std.testing.expect(bodies.isAdded(id));

    // Added, awake, and stepped, which is the state the old contract said to
    // release out of only after `removeFromPhysicsSystem`. No such call here:
    // this is the whole test.
    try world.stepFor(0.5);
    try std.testing.expect(ragdoll.isActive(true));
    ragdoll.release();

    try std.testing.expectEqual(handles_before, zjolt.liveHandleCount());

    // The parts are gone, not merely removed — the release destroyed them
    // after taking them out, so the system's body list is back to what it
    // held before.
    try std.testing.expectEqual(bodies_before, world.system.numBodies());
    var left_buf: [8]zjolt.BodyId = undefined;
    const left = try world.system.getBodies(&left_buf);
    try std.testing.expectEqual(before.len, left.len);
    for (parts) |part| {
        for (left) |id| try std.testing.expect(id != part);
    }

    // And the world is still a world. This is what says the broad phase is
    // intact: destroying the parts where they stood would have left it
    // holding freed bodies, which a build without assertions only ever shows
    // as a corrupt step some time later.
    try world.stepFor(0.5);
    try std.testing.expect(world.system.bodies().isAdded(world.floor));
}

//=============================================================================
// Introspection
//=============================================================================

test "a spawned ragdoll reports its parts, its template and where its root is" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    var chain = try Chain.build(.dynamic);
    defer chain.deinit();

    var ragdoll = try chain.settings.createRagdoll(world.system, 1, 0);
    defer ragdoll.release();
    ragdoll.addToPhysicsSystem(.activate, true);

    // Two parts, one constraint: the root joint has no parent to be attached
    // to, so it has no constraint of its own.
    try std.testing.expectEqual(@as(u32, 2), ragdoll.bodyCount());
    try std.testing.expectEqual(@as(u32, 1), ragdoll.constraintCount());
    try std.testing.expect(ragdoll.getConstraint(0) != null);
    try std.testing.expect(ragdoll.getConstraint(1) == null);

    // The route from a ragdoll back to the joint names behind its body ids,
    // which is otherwise only reachable by having kept them yourself.
    const settings = ragdoll.settings();
    try std.testing.expectEqual(chain.settings.handle, settings.handle);
    const skeleton = settings.skeleton().?;
    try std.testing.expectEqual(chain.skeleton.handle, skeleton.handle);
    try std.testing.expectEqualStrings("root", skeleton.jointName(0));

    // Where the ragdoll is, without reading every part back: the same
    // transform the body built from joint 0 carries.
    var ids: [2]zjolt.BodyId = undefined;
    const parts = try ragdoll.getBodyIds(&ids);
    const bodies = world.system.bodies();
    const root = try ragdoll.rootTransform(true);
    try std.testing.expectApproxEqAbs(bodies.getPosition(parts[0]).y, root.position.y, 1e-5);
    try std.testing.expectApproxEqAbs(
        @as(f32, 0),
        quatAngle(bodies.getRotation(parts[0]), root.rotation),
        1e-4,
    );

    // And it tracks the simulation rather than the settings it was built
    // from: gravity has moved it since.
    const started_at = root.position.y;
    try world.stepFor(1.0);
    const now = try ragdoll.rootTransform(true);
    try std.testing.expect(now.position.y < started_at - 0.5);
}

test "setGroupId moves every part into a new collision group" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    var chain = try Chain.build(.dynamic);
    defer chain.deinit();

    var ragdoll = try chain.settings.createRagdoll(world.system, 1, 0);
    defer ragdoll.release();
    ragdoll.addToPhysicsSystem(.activate, true);

    var ids: [2]zjolt.BodyId = undefined;
    const parts = try ragdoll.getBodyIds(&ids);
    const bodies = world.system.bodies();

    // `createRagdoll` set the group id once, and every part carries the
    // per-joint sub-group `disableParentChildCollisions` gave it.
    for (parts, 0..) |id, joint| {
        const group = bodies.getCollisionGroup(id);
        try std.testing.expectEqual(@as(u32, 1), group.group_id);
        try std.testing.expectEqual(@as(u32, @intCast(joint)), group.sub_group_id);
        try std.testing.expect(group.filter != null);
    }

    ragdoll.setGroupId(7, true);

    // The group id moved; the sub-group ids and the shared filter did not,
    // which is what keeps parent-child collisions disabled in the new group.
    for (parts, 0..) |id, joint| {
        const group = bodies.getCollisionGroup(id);
        try std.testing.expectEqual(@as(u32, 7), group.group_id);
        try std.testing.expectEqual(@as(u32, @intCast(joint)), group.sub_group_id);
        try std.testing.expect(group.filter != null);
    }
}

test "calculateConstraintPriorities counts up from the leaves toward the root" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    var limb = try Limb.build();
    defer limb.deinit();

    // A ragdoll spawned BEFORE the call carries Jolt's default priority of 0
    // on every constraint — the settings are the thing that changes, not the
    // ragdoll.
    {
        var plain = try limb.settings.createRagdoll(world.system, 1, 0);
        defer plain.release();
        try std.testing.expectEqual(@as(u32, 2), plain.constraintCount());
        try std.testing.expectEqual(@as(u32, 0), plain.getConstraint(0).?.getPriority());
        try std.testing.expectEqual(@as(u32, 0), plain.getConstraint(1).?.getPriority());
    }

    try limb.settings.calculateConstraintPriorities(5);

    var ragdoll = try limb.settings.createRagdoll(world.system, 2, 0);
    defer ragdoll.release();

    // Constraint 0 attaches joint 1 to the root, constraint 1 attaches the
    // leaf to joint 1. The one nearer the root sorts HIGHER, so the solver
    // reaches it first and the leaf is not left fighting a shoulder that has
    // not moved yet. `base_priority` is the floor, and the leaf-most
    // constraint is what gets it.
    const near_root = ragdoll.getConstraint(0).?.getPriority();
    const near_leaf = ragdoll.getConstraint(1).?.getPriority();
    try std.testing.expectEqual(@as(u32, 5), near_leaf);
    try std.testing.expect(near_root > near_leaf);

    // Settings that were never built have no skeleton to walk, which Jolt
    // dereferences without looking.
    const empty = try zjolt.RagdollSettings.init();
    defer empty.release();
    try std.testing.expectError(
        error.InvalidArgument,
        empty.calculateConstraintPriorities(0),
    );

    // And the priority a root would end up with has to fit in 32 bits.
    try std.testing.expectError(
        error.InvalidArgument,
        limb.settings.calculateConstraintPriorities(std.math.maxInt(u32)),
    );
}

test "RagdollSettings' body-index/constraint-index maps agree with joint parentage" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var limb = try Limb.build();
    defer limb.deinit();

    // Before either Calculate* call, both tables are empty, so every index
    // reads as out of range rather than as a zeroed entry.
    try std.testing.expectEqual(@as(?u32, null), limb.settings.constraintIndexForBodyIndex(1));
    try std.testing.expectError(
        error.InvalidArgument,
        limb.settings.bodyIndicesForConstraintIndex(0),
    );

    limb.settings.calculateBodyIndexToConstraintIndex();
    limb.settings.calculateConstraintIndexToBodyIdxPair();

    // Root (0) has no parent, so no constraint of its own; middle (1) and
    // leaf (2) each get the constraint attaching them to their parent,
    // numbered in the order their parts appear.
    try std.testing.expectEqual(@as(?u32, null), limb.settings.constraintIndexForBodyIndex(0));
    try std.testing.expectEqual(@as(?u32, 0), limb.settings.constraintIndexForBodyIndex(1));
    try std.testing.expectEqual(@as(?u32, 1), limb.settings.constraintIndexForBodyIndex(2));
    try std.testing.expectEqual(@as(?u32, null), limb.settings.constraintIndexForBodyIndex(99));

    var table_buf: [3]i32 = undefined;
    const table = try limb.settings.bodyIndexToConstraintIndex(&table_buf);
    try std.testing.expectEqualSlices(i32, &.{ -1, 0, 1 }, table);

    // Constraint 0 attaches body 1 (middle) to body 0 (root); constraint 1
    // attaches body 2 (leaf) to body 1 (middle) — parent first, then child.
    const middle_constraint = try limb.settings.bodyIndicesForConstraintIndex(0);
    try std.testing.expectEqual(@as(i32, 0), middle_constraint.body_index1);
    try std.testing.expectEqual(@as(i32, 1), middle_constraint.body_index2);

    const leaf_constraint = try limb.settings.bodyIndicesForConstraintIndex(1);
    try std.testing.expectEqual(@as(i32, 1), leaf_constraint.body_index1);
    try std.testing.expectEqual(@as(i32, 2), leaf_constraint.body_index2);

    try std.testing.expectError(
        error.InvalidArgument,
        limb.settings.bodyIndicesForConstraintIndex(2),
    );

    var pairs_buf: [2]zjolt.RagdollSettings.BodyIndexPair = undefined;
    const pairs = try limb.settings.constraintIndexToBodyIdxPair(&pairs_buf);
    try std.testing.expectEqual(@as(usize, 2), pairs.len);
    try std.testing.expectEqual(@as(i32, 0), pairs[0].body_index1);
    try std.testing.expectEqual(@as(i32, 1), pairs[0].body_index2);
    try std.testing.expectEqual(@as(i32, 1), pairs[1].body_index1);
    try std.testing.expectEqual(@as(i32, 2), pairs[1].body_index2);

    // A buffer shorter than the table reports BufferTooSmall rather than
    // writing a truncated table silently.
    var short_buf: [1]i32 = undefined;
    try std.testing.expectError(
        error.BufferTooSmall,
        limb.settings.bodyIndexToConstraintIndex(&short_buf),
    );
}

//=============================================================================
// The joints holding the parts together
//=============================================================================

/// A weld (`FixedConstraint`) between two bodies standing exactly where
/// `Chain`'s two parts stand, snapshotted as settings. The settings outlive
/// the world they came from: Jolt records the two local-to-centre-of-mass
/// frames, not the bodies.
fn weldSettingsForChain() !zjolt.ConstraintSettings {
    var mold = try World.init();
    defer mold.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.4, 0.3), .{});
    defer shape.release();

    const bodies = mold.system.bodies();
    const root = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = Chain.root_position,
    }, .dont_activate);
    const child = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = Chain.child_position,
    }, .dont_activate);

    const weld = try zjolt.Constraint.initFixed(mold.system, root, child, .{
        .point1 = Chain.joint_position,
        .point2 = Chain.joint_position,
    });
    defer weld.release();
    return weld.constraintSettings();
}

/// How far apart the two parts of a spawned `Chain` end up after `seconds`,
/// and how low the child got, with the joint rewritten first. The one
/// variable across the three arms below.
const ChainOutcome = struct { separation: f32, child_y: f32 };

fn runChain(
    install: ?zjolt.ConstraintSettings,
    remove: bool,
    seconds: f32,
) !ChainOutcome {
    var world = try World.init();
    defer world.deinit();

    var chain = try Chain.build(.kinematic);
    defer chain.deinit();

    if (remove) {
        try chain.settings.setPartConstraint(1, null);
    } else if (install) |settings| {
        try chain.settings.setPartConstraint(1, settings);
    }
    chain.settings.calculateBodyIndexToConstraintIndex();

    var ragdoll = try chain.settings.createRagdoll(world.system, 1, 0);
    defer ragdoll.release();
    ragdoll.addToPhysicsSystem(.activate, true);

    var id_buf: [2]zjolt.BodyId = undefined;
    const ids = try ragdoll.getBodyIds(&id_buf);
    try world.stepFor(seconds);

    const bodies = world.system.bodies();
    const root = bodies.getPosition(ids[0]);
    const child = bodies.getPosition(ids[1]);
    const dx: f32 = @floatCast(child.x - root.x);
    const dy: f32 = @floatCast(child.y - root.y);
    const dz: f32 = @floatCast(child.z - root.z);
    return .{
        .separation = @sqrt(dx * dx + dy * dy + dz * dz),
        .child_y = @floatCast(child.y),
    };
}

test "a part's joint is what holds it to its parent, and it takes any two-body kind or none" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Arm 1: the swing-twist `PartDesc.to_parent` describes. A kinematic root
    // does not fall, so the child only stays up if something holds it.
    const hinged = try runChain(null, false, 1.5);
    try std.testing.expect(hinged.child_y > 4.0);
    try std.testing.expect(hinged.separation < 1.6);

    // Arm 2: no joint at all. Same settings, same skeleton, same bodies —
    // the one difference is the field, and the child falls to the floor.
    const loose = try runChain(null, true, 1.5);
    try std.testing.expect(loose.child_y < 3.0);

    // Arm 3: a weld, which `RagdollConstraintDesc` cannot describe at all.
    // It holds the child at exactly the offset it was built at, where the
    // swing-twist's generous cone lets it swing.
    const weld = try weldSettingsForChain();
    defer weld.deinit();
    const welded = try runChain(weld, false, 1.5);
    try std.testing.expect(welded.child_y > 4.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), welded.separation, 0.05);
}

test "a part's joint reads back as the same object that was installed" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var chain = try Chain.build(.dynamic);
    defer chain.deinit();

    // The root has no parent, so no joint — not an error, just absent.
    try std.testing.expect((try chain.settings.partConstraint(0)) == null);

    const built = (try chain.settings.partConstraint(1)) orelse
        return error.TestUnexpectedResult;
    defer built.deinit();

    const weld = try weldSettingsForChain();
    defer weld.deinit();
    try chain.settings.setPartConstraint(1, weld);

    const read_back = (try chain.settings.partConstraint(1)) orelse
        return error.TestUnexpectedResult;
    defer read_back.deinit();
    try std.testing.expectEqual(weld.handle, read_back.handle);
    // And the swing-twist it replaced is gone from the part, not merely
    // shadowed: the two handles are different objects.
    try std.testing.expect(built.handle != read_back.handle);

    try chain.settings.setPartConstraint(1, null);
    try std.testing.expect((try chain.settings.partConstraint(1)) == null);

    // Past the end of the part list is an error, not a silent no-op.
    try std.testing.expectError(
        error.InvalidArgument,
        chain.settings.setPartConstraint(2, weld),
    );
    try std.testing.expectError(
        error.InvalidArgument,
        chain.settings.partConstraint(2),
    );
}

test "additional constraints join parts that are not parent and child, and read back" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var chain = try Chain.build(.dynamic);
    defer chain.deinit();
    try std.testing.expectEqual(@as(u32, 0), chain.settings.additionalConstraintCount());

    const weld = try weldSettingsForChain();
    defer weld.deinit();
    try chain.settings.addAdditionalConstraint(0, 1, weld);
    try std.testing.expectEqual(@as(u32, 1), chain.settings.additionalConstraintCount());

    const entry = try chain.settings.additionalConstraint(0);
    const settings = entry.settings orelse return error.TestUnexpectedResult;
    defer settings.deinit();
    try std.testing.expectEqual(@as(u32, 0), entry.part_index1);
    try std.testing.expectEqual(@as(u32, 1), entry.part_index2);
    try std.testing.expectEqual(weld.handle, settings.handle);

    try std.testing.expectError(
        error.InvalidArgument,
        chain.settings.additionalConstraint(1),
    );
    try std.testing.expectError(
        error.InvalidArgument,
        chain.settings.addAdditionalConstraint(0, 2, weld),
    );

    // The extra constraint reaches the solver: a spawned ragdoll has two
    // constraints where the part list alone accounts for one.
    var world = try World.init();
    defer world.deinit();
    var ragdoll = try chain.settings.createRagdoll(world.system, 1, 0);
    defer ragdoll.release();
    try std.testing.expect(ragdoll.getConstraint(0) != null);
    try std.testing.expect(ragdoll.getConstraint(1) != null);
    try std.testing.expect(ragdoll.getConstraint(2) == null);
}

test "a constraint kind that is not two-body is refused rather than badly cast" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const chassis_shape = try zjolt.Shape.initBox(zjolt.vec3(0.75, 0.25, 1.75), .{});
    defer chassis_shape.release();
    const chassis = try world.system.bodies().createAndAdd(.{
        .shape = chassis_shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 2, 0),
    }, .dont_activate);

    // A vehicle constraint is the one kind of Jolt constraint that derives
    // from Constraint without going through TwoBodyConstraint, which is what
    // makes it the honest negative arm here.
    var wheels = [_]zjolt.VehicleWheelDesc{
        zjolt.defaultVehicleWheelDesc(),
        zjolt.defaultVehicleWheelDesc(),
    };
    wheels[0].position = zjolt.vec3(-0.7, -0.25, 1.4);
    wheels[1].position = zjolt.vec3(0.7, -0.25, 1.4);
    var axle = zjolt.defaultVehicleDifferentialDesc();
    axle.left_wheel = 0;
    axle.right_wheel = 1;
    const differentials = [_]zjolt.VehicleDifferentialDesc{axle};

    const vehicle = try zjolt.VehicleConstraint.init(world.system, chassis, .{
        .wheels = &wheels,
        .differentials = &differentials,
        .collision_tester = zjolt.defaultVehicleCollisionTesterDesc(),
    });
    defer vehicle.deinit();

    const as_constraint = vehicle.asConstraint() orelse
        return error.TestUnexpectedResult;
    const vehicle_settings = try as_constraint.constraintSettings();
    defer vehicle_settings.deinit();

    var chain = try Chain.build(.dynamic);
    defer chain.deinit();
    try std.testing.expectError(
        error.InvalidArgument,
        chain.settings.setPartConstraint(1, vehicle_settings),
    );
    try std.testing.expectError(
        error.InvalidArgument,
        chain.settings.addAdditionalConstraint(0, 1, vehicle_settings),
    );
    // And refusing left the part's own joint alone.
    const still_there = (try chain.settings.partConstraint(1)) orelse
        return error.TestUnexpectedResult;
    still_there.deinit();
}

//=============================================================================
// Moving a whole ragdoll at once
//=============================================================================

/// How far a spawned `Chain` travelled along +x in `seconds`, with `launch`
/// given the ragdoll first. The arms below differ only in `launch`.
fn launchedDistanceX(
    launch: ?*const fn (zjolt.Ragdoll) anyerror!void,
    seconds: f32,
) !f32 {
    var world = try World.init();
    defer world.deinit();

    var chain = try Chain.build(.dynamic);
    defer chain.deinit();

    var ragdoll = try chain.settings.createRagdoll(world.system, 1, 0);
    defer ragdoll.release();
    ragdoll.addToPhysicsSystem(.activate, true);

    var id_buf: [2]zjolt.BodyId = undefined;
    const ids = try ragdoll.getBodyIds(&id_buf);
    const before: f32 = @floatCast(world.system.bodies().getPosition(ids[0]).x);

    if (launch) |f| try f(ragdoll);
    try world.stepFor(seconds);

    const after: f32 = @floatCast(world.system.bodies().getPosition(ids[0]).x);
    return after - before;
}

fn setLinear(ragdoll: zjolt.Ragdoll) anyerror!void {
    try ragdoll.setLinearVelocity(zjolt.vec3(6, 0, 0), true);
}

fn addLinear(ragdoll: zjolt.Ragdoll) anyerror!void {
    try ragdoll.addLinearVelocity(zjolt.vec3(6, 0, 0), true);
}

fn setLinearAndAngular(ragdoll: zjolt.Ragdoll) anyerror!void {
    try ragdoll.setLinearAndAngularVelocity(zjolt.vec3(6, 0, 0), zjolt.vec3(0, 0, 0), true);
}

fn addTheImpulse(ragdoll: zjolt.Ragdoll) anyerror!void {
    // The root box is 0.6 x 0.8 x 0.6 at Jolt's default density, so a few
    // hundred kilograms; 2000 N.s is several metres per second on each part.
    try ragdoll.addImpulse(zjolt.vec3(2000, 0, 0), true);
}

test "a ragdoll is launched as one thing by velocity or by impulse" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // The control: nothing but gravity, which is along -y, so +x stays put.
    const drifted = try launchedDistanceX(null, 0.5);
    try std.testing.expect(@abs(drifted) < 0.1);

    inline for (.{ setLinear, addLinear, setLinearAndAngular, addTheImpulse }) |launch| {
        const travelled = try launchedDistanceX(&launch, 0.5);
        try std.testing.expect(travelled > 0.5);
    }
}

//=============================================================================
// Joint matrices in and out of a pose
//=============================================================================

test "a skeleton parented by name resolves its indices, and says so when the order is wrong" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var skeleton = try zjolt.Skeleton.init();
    defer skeleton.release();

    // The order an importer reading a file may well hand them over in: a
    // joint's parent named before the parent itself exists.
    const pelvis = try skeleton.addJointWithParentName("pelvis", null);
    const hand = try skeleton.addJointWithParentName("hand", "arm");
    const arm = try skeleton.addJointWithParentName("arm", "pelvis");
    try std.testing.expectEqual(@as(u32, 0), pelvis);
    try std.testing.expectEqual(@as(u32, 1), hand);
    try std.testing.expectEqual(@as(u32, 2), arm);

    // A name is not an index until it is resolved.
    try std.testing.expectEqual(@as(?u32, null), skeleton.jointParentIndex(hand));
    try std.testing.expectEqual(@as(?u32, null), skeleton.jointParentIndex(arm));

    try skeleton.calculateParentJointIndices();
    try std.testing.expectEqual(@as(?u32, null), skeleton.jointParentIndex(pelvis));
    try std.testing.expectEqual(@as(?u32, arm), skeleton.jointParentIndex(hand));
    try std.testing.expectEqual(@as(?u32, pelvis), skeleton.jointParentIndex(arm));

    // The hand's parent is BELOW it in the array, which is the case the
    // indexed `addJoint` refuses outright and this one can produce.
    try std.testing.expect(!skeleton.areJointsCorrectlyOrdered());

    // And the same three joints added in order are what a ragdoll needs.
    var ordered = try zjolt.Skeleton.init();
    defer ordered.release();
    const root = try ordered.addJointWithParentName("pelvis", null);
    const upper = try ordered.addJointWithParentName("arm", "pelvis");
    _ = try ordered.addJointWithParentName("hand", "arm");
    try ordered.calculateParentJointIndices();
    try std.testing.expectEqual(@as(?u32, root), ordered.jointParentIndex(upper));
    try std.testing.expect(ordered.areJointsCorrectlyOrdered());
}

test "a pose's joint matrices round-trip, and a short buffer is refused" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var skeleton = try zjolt.Skeleton.init();
    defer skeleton.release();
    const root = try skeleton.addJoint("root", null);
    _ = try skeleton.addJoint("child", root);

    const pose = try zjolt.SkeletonPose.init();
    defer pose.deinit();
    try pose.setSkeleton(skeleton);

    const rotations = [_]zjolt.Quat{ zjolt.quat_identity, zjolt.quat_identity };
    const translations = [_]zjolt.Vec3{ zjolt.vec3(0, 0, 0), zjolt.vec3(0, 1, 0) };
    try pose.setJoints(&rotations, &translations);
    try pose.calculateJointMatrices();

    var matrices: [32]f32 = undefined;
    const read = try pose.getJointMatrices(&matrices);
    try std.testing.expectEqual(@as(usize, 32), read.len);
    // Column-major: the translation is column 3. The child sits one metre
    // above the root in MODEL space, which is what these matrices are in.
    try std.testing.expectApproxEqAbs(@as(f32, 0), read[13], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), read[16 + 13], 1e-5);

    // Written straight back in, without going through local joint states,
    // and then derived back out of them.
    var moved = matrices;
    moved[16 + 13] = 2.5;
    try pose.setJointMatrices(&moved);
    try pose.calculateJointStates();

    var out_rotations: [2]zjolt.Quat = undefined;
    var out_translations: [2]zjolt.Vec3 = undefined;
    try pose.getJoints(&out_rotations, &out_translations);
    try std.testing.expectApproxEqAbs(@as(f32, 2.5), out_translations[1].y, 1e-4);

    var short: [16]f32 = undefined;
    try std.testing.expectError(error.BufferTooSmall, pose.getJointMatrices(&short));
    try std.testing.expectError(error.InvalidArgument, pose.setJointMatrices(&short));
}

//=============================================================================
// SkeletalAnimation
//=============================================================================

test "sampling an animation interpolates between keyframes the way SLERP and lerp do" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var skeleton = try zjolt.Skeleton.init();
    defer skeleton.release();
    _ = try skeleton.addJoint("j0", null);

    var anim = try ragdoll_mod.SkeletalAnimation.init();
    defer anim.release();
    try std.testing.expectEqual(@as(u32, 0), anim.animatedJointCount());

    const joint = try anim.addAnimatedJoint("j0");
    try std.testing.expectEqual(@as(u32, 0), joint);
    try std.testing.expectEqual(@as(u32, 1), anim.animatedJointCount());
    try std.testing.expectEqualStrings("j0", anim.animatedJointName(joint));
    try std.testing.expectEqualStrings("", anim.animatedJointName(1));

    const end_rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 0, 1), std.math.pi / 2.0);
    try anim.addKeyframe(joint, 0.0, zjolt.quat_identity, zjolt.vec3_zero);
    try anim.addKeyframe(joint, 1.0, end_rotation, zjolt.vec3(0, 2, 0));
    try std.testing.expectEqual(@as(u32, 2), anim.keyframeCount(joint));
    try std.testing.expectEqual(@as(u32, 0), anim.keyframeCount(1));

    const first = try anim.keyframe(joint, 0);
    try std.testing.expectEqual(@as(f32, 0.0), first.time);
    try expectVec3Near(zjolt.vec3_zero, first.translation);
    const last = try anim.keyframe(joint, 1);
    try std.testing.expectEqual(@as(f32, 1.0), last.time);
    try expectVec3Near(zjolt.vec3(0, 2, 0), last.translation);

    try std.testing.expectEqual(@as(f32, 1.0), anim.duration());
    try std.testing.expect(anim.isLooping());
    anim.setIsLooping(false);
    try std.testing.expect(!anim.isLooping());

    var pose = try zjolt.SkeletonPose.init();
    defer pose.deinit();
    try pose.setSkeleton(skeleton);

    // Halfway between the two keyframes: linear interpolation of the
    // translation, and — since the two rotations share an axis — exactly
    // half the rotation's angle, which is what SLERP between them gives.
    try anim.sample(0.5, pose);

    var rotations: [1]zjolt.Quat = undefined;
    var translations: [1]zjolt.Vec3 = undefined;
    try pose.getJoints(&rotations, &translations);

    try expectVec3Near(zjolt.vec3(0, 1, 0), translations[0]);
    const expected_rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 0, 1), std.math.pi / 4.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0), quatAngle(expected_rotation, rotations[0]), 1e-4);

    // Before the first keyframe and after the last: clamped to the nearest
    // end (looping is off), not extrapolated.
    try anim.sample(0.0, pose);
    try pose.getJoints(&rotations, &translations);
    try expectVec3Near(zjolt.vec3_zero, translations[0]);

    try anim.sample(5.0, pose);
    try pose.getJoints(&rotations, &translations);
    try expectVec3Near(zjolt.vec3(0, 2, 0), translations[0]);
    try std.testing.expectApproxEqAbs(@as(f32, 0), quatAngle(end_rotation, rotations[0]), 1e-4);

    // ScaleJoints scales every keyframe's translation, not its rotation.
    anim.scaleJoints(2.0);
    const scaled = try anim.keyframe(joint, 1);
    try expectVec3Near(zjolt.vec3(0, 4, 0), scaled.translation);
}

test "invalid indices, times and poses into an animation return an error" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var anim = try ragdoll_mod.SkeletalAnimation.init();
    defer anim.release();
    const joint = try anim.addAnimatedJoint("only");
    try anim.addKeyframe(joint, 0.0, zjolt.quat_identity, zjolt.vec3_zero);

    try std.testing.expectError(
        error.InvalidArgument,
        anim.addKeyframe(joint + 1, 0.0, zjolt.quat_identity, zjolt.vec3_zero),
    );
    try std.testing.expectError(error.InvalidArgument, anim.keyframe(joint + 1, 0));
    try std.testing.expectError(error.InvalidArgument, anim.keyframe(joint, 1));

    // Keyframes must be added in non-decreasing time order — Sample's binary
    // search over them assumes it, and Jolt does not check.
    try anim.addKeyframe(joint, 2.0, zjolt.quat_identity, zjolt.vec3_zero);
    try std.testing.expectError(
        error.InvalidArgument,
        anim.addKeyframe(joint, 1.0, zjolt.quat_identity, zjolt.vec3_zero),
    );

    // Sampling onto a pose whose skeleton does not have the animated joint's
    // name would index the pose with no bounds check inside Jolt.
    var different = try zjolt.Skeleton.init();
    defer different.release();
    _ = try different.addJoint("not-only", null);
    var mismatched_pose = try zjolt.SkeletonPose.init();
    defer mismatched_pose.deinit();
    try mismatched_pose.setSkeleton(different);
    try std.testing.expectError(error.InvalidArgument, anim.sample(0.0, mismatched_pose));

    // A pose with no skeleton assigned at all, and a negative time, are
    // refused the same way.
    var bare_pose = try zjolt.SkeletonPose.init();
    defer bare_pose.deinit();
    try std.testing.expectError(error.InvalidArgument, anim.sample(0.0, bare_pose));

    var matching = try zjolt.Skeleton.init();
    defer matching.release();
    _ = try matching.addJoint("only", null);
    try bare_pose.setSkeleton(matching);
    try std.testing.expectError(error.InvalidArgument, anim.sample(-1.0, bare_pose));
}

test "SaveBinaryState / sRestoreFromBinaryState round-trips an animation" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var anim = try ragdoll_mod.SkeletalAnimation.init();
    defer anim.release();
    const j0 = try anim.addAnimatedJoint("root");
    const j1 = try anim.addAnimatedJoint("child");
    try anim.addKeyframe(j0, 0.0, zjolt.quat_identity, zjolt.vec3_zero);
    try anim.addKeyframe(
        j0,
        1.0,
        try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 1, 0), 1.0),
        zjolt.vec3(1, 0, 0),
    );
    try anim.addKeyframe(
        j1,
        0.5,
        try zjolt.Quat.fromAxisAngle(zjolt.vec3(1, 0, 0), 0.25),
        zjolt.vec3(0, 0, 3),
    );
    anim.setIsLooping(false);

    var stream_buffer: [4096]u8 = undefined;
    var writer: zjolt.StreamBufferWriter = .{ .buffer = &stream_buffer };
    try anim.saveBinaryState(zjolt.hostStream(zjolt.StreamBufferWriter, &writer));

    var reader: zjolt.StreamBufferReader = .{ .buffer = writer.slice() };
    var restored = try ragdoll_mod.SkeletalAnimation.restoreBinaryState(
        zjolt.hostStream(zjolt.StreamBufferReader, &reader),
    );
    defer restored.release();

    try std.testing.expectEqual(anim.animatedJointCount(), restored.animatedJointCount());
    try std.testing.expectEqual(anim.isLooping(), restored.isLooping());
    try std.testing.expectEqual(anim.duration(), restored.duration());

    var i: u32 = 0;
    while (i < restored.animatedJointCount()) : (i += 1) {
        try std.testing.expectEqualStrings(anim.animatedJointName(i), restored.animatedJointName(i));
        try std.testing.expectEqual(anim.keyframeCount(i), restored.keyframeCount(i));

        var k: u32 = 0;
        while (k < anim.keyframeCount(i)) : (k += 1) {
            const original = try anim.keyframe(i, k);
            const copy = try restored.keyframe(i, k);
            try std.testing.expectEqual(original.time, copy.time);
            try expectVec3Near(original.translation, copy.translation);
            try std.testing.expectApproxEqAbs(@as(f32, 0), quatAngle(original.rotation, copy.rotation), 1e-6);
        }
    }
}

test "JointState.toMatrix and fromMatrix invert each other" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 1, 0), 0.9);
    const translation = zjolt.vec3(1, 2, 3);
    const state: ragdoll_mod.SkeletalAnimation.JointState = .{ .rotation = rotation, .translation = translation };

    const matrix = state.toMatrix();
    const back = ragdoll_mod.SkeletalAnimation.JointState.fromMatrix(matrix);

    try expectVec3Near(translation, back.translation);
    try std.testing.expectApproxEqAbs(@as(f32, 0), quatAngle(rotation, back.rotation), 1e-5);
}

//=============================================================================
// SkeletonMapper
//=============================================================================

test "a skeleton mapper drives a high-detail skeleton from a low-detail one" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var pair = try SkeletonPair.build();
    defer pair.deinit();

    const mapper = try zjolt.SkeletonMapper.init();
    defer mapper.release();
    try mapper.initialize(pair.neutral1, pair.neutral2, null, null);

    // Both joints of the ragdoll skeleton found their namesake. The finger
    // has no counterpart, so it is not a mapping — it is a joint the mapper
    // rebuilds from the render skeleton's own local pose.
    try std.testing.expectEqual(@as(u32, 2), mapper.mappingCount());
    try std.testing.expectEqual(@as(?u32, 0), mapper.mappedJointIndex(0));
    try std.testing.expectEqual(@as(?u32, 1), mapper.mappedJointIndex(1));
    try std.testing.expectEqual(@as(?u32, null), mapper.mappedJointIndex(2));

    // The simulated pose: the whole ragdoll has moved two metres along x, and
    // the hand has rotated.
    const twist = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 0, 1), 0.5);
    const pose1 = try zjolt.SkeletonPose.init();
    defer pose1.deinit();
    try pose1.setSkeleton(pair.ragdoll_skeleton);
    try pose1.setJoints(
        &.{ zjolt.quat_identity, twist },
        &.{ zjolt.vec3(2, 0, 0), zjolt.vec3(0, 1, 0) },
    );
    try pose1.calculateJointMatrices();
    // Where the ragdoll is in the world. `Ragdoll.getPose` puts it here
    // rather than in the joint matrices, which are relative to it.
    pose1.setRootOffset(zjolt.rvec3(0, 30, 0));

    // The render pose carries the animation, and `map` overwrites the part of
    // it the ragdoll owns.
    const pose2 = try zjolt.SkeletonPose.init();
    defer pose2.deinit();
    try pose2.setSkeleton(pair.render_skeleton);
    try pose2.setJoints(
        &.{ zjolt.quat_identity, zjolt.quat_identity, zjolt.quat_identity },
        &.{ zjolt.vec3_zero, zjolt.vec3(0, 1, 0), zjolt.vec3(0, 0.5, 0) },
    );

    try mapper.map(pose1, pose2);
    try pose2.calculateJointStates();

    var rotations: [3]zjolt.Quat = undefined;
    var translations: [3]zjolt.Vec3 = undefined;
    try pose2.getJoints(&rotations, &translations);

    // The world position rode across in the root offset, which Jolt's own map
    // leaves behind — a render pose left at its own offset would draw a
    // correctly posed character thirty metres below the ragdoll.
    try std.testing.expectApproxEqAbs(
        @as(zjolt.Real, 30),
        pose2.rootOffset().y,
        1e-4,
    );

    // The root moved with the ragdoll — it started at the origin — and the
    // hand took the ragdoll's rotation.
    try expectVec3Near(zjolt.vec3(2, 0, 0), translations[0]);
    try expectVec3Near(zjolt.vec3(0, 1, 0), translations[1]);
    try std.testing.expectApproxEqAbs(@as(f32, 0), quatAngle(twist, rotations[1]), 1e-4);

    // The finger is not driven by the ragdoll, so it kept its own local
    // transform relative to the hand that just moved under it.
    try expectVec3Near(zjolt.vec3(0, 0.5, 0), translations[2]);
    try std.testing.expectApproxEqAbs(
        @as(f32, 0),
        quatAngle(zjolt.quat_identity, rotations[2]),
        1e-4,
    );

    // And back the other way, which is what drives a ragdoll from animation.
    const back = try zjolt.SkeletonPose.init();
    defer back.deinit();
    try back.setSkeleton(pair.ragdoll_skeleton);
    try mapper.mapReverse(pose2, back);
    try back.calculateJointStates();

    var back_rotations: [2]zjolt.Quat = undefined;
    var back_translations: [2]zjolt.Vec3 = undefined;
    try back.getJoints(&back_rotations, &back_translations);
    try std.testing.expectApproxEqAbs(@as(zjolt.Real, 30), back.rootOffset().y, 1e-4);
    try expectVec3Near(zjolt.vec3(2, 0, 0), back_translations[0]);
    try expectVec3Near(zjolt.vec3(0, 1, 0), back_translations[1]);
    try std.testing.expectApproxEqAbs(
        @as(f32, 0),
        quatAngle(twist, back_rotations[1]),
        1e-4,
    );

    // A second initialize would append a second set of mappings to the first.
    try std.testing.expectError(
        error.InvalidArgument,
        mapper.initialize(pair.neutral1, pair.neutral2, null, null),
    );
    // And the low-detail skeleton has to be the first one: Jolt assumes every
    // joint of skeleton 1 maps into skeleton 2.
    const backwards = try zjolt.SkeletonMapper.init();
    defer backwards.release();
    try std.testing.expectError(
        error.InvalidArgument,
        backwards.initialize(pair.neutral2, pair.neutral1, null, null),
    );
}

test "locking translations holds a stretched joint at its neutral offset" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var pair = try SkeletonPair.build();
    defer pair.deinit();

    // A ragdoll under load: the constraint between root and hand has pulled
    // apart to twice its rest length.
    const stretched = try zjolt.SkeletonPose.init();
    defer stretched.deinit();
    try stretched.setSkeleton(pair.ragdoll_skeleton);
    try stretched.setJoints(
        &.{ zjolt.quat_identity, zjolt.quat_identity },
        &.{ zjolt.vec3_zero, zjolt.vec3(0, 2, 0) },
    );
    try stretched.calculateJointMatrices();

    // Mapped without locking, the stretch reaches the render skeleton whole.
    {
        const plain = try zjolt.SkeletonMapper.init();
        defer plain.release();
        try plain.initialize(pair.neutral1, pair.neutral2, null, null);
        try std.testing.expect(!plain.isJointTranslationLocked(1));

        const out = try renderPose(pair);
        defer out.deinit();
        try plain.map(stretched, out);
        try out.calculateJointStates();

        var translations: [3]zjolt.Vec3 = undefined;
        var rotations: [3]zjolt.Quat = undefined;
        try out.getJoints(&rotations, &translations);
        try expectVec3Near(zjolt.vec3(0, 2, 0), translations[1]);
    }

    // Locked, it does not: the hand stays one metre from the root, where the
    // neutral pose put it, and only its rotation follows the ragdoll.
    {
        const locked = try zjolt.SkeletonMapper.init();
        defer locked.release();
        try locked.initialize(pair.neutral1, pair.neutral2, null, null);
        try locked.lockAllTranslations(pair.neutral2);

        // Everything below the topmost mapped joint, that joint excluded —
        // its translation is what positions the whole ragdoll.
        try std.testing.expect(!locked.isJointTranslationLocked(0));
        try std.testing.expect(locked.isJointTranslationLocked(1));
        try std.testing.expect(locked.isJointTranslationLocked(2));

        const out = try renderPose(pair);
        defer out.deinit();
        try locked.map(stretched, out);
        try out.calculateJointStates();

        var translations: [3]zjolt.Vec3 = undefined;
        var rotations: [3]zjolt.Quat = undefined;
        try out.getJoints(&rotations, &translations);
        try expectVec3Near(zjolt.vec3(0, 1, 0), translations[1]);
        try expectVec3Near(zjolt.vec3(0, 0.5, 0), translations[2]);
    }

    // The explicit form locks exactly what it is given.
    {
        const some = try zjolt.SkeletonMapper.init();
        defer some.release();
        try some.initialize(pair.neutral1, pair.neutral2, null, null);
        try some.lockTranslations(pair.neutral2, &.{ false, false, true });
        try std.testing.expect(!some.isJointTranslationLocked(1));
        try std.testing.expect(some.isJointTranslationLocked(2));

        // A joint with no parent cannot be locked: Jolt would resolve the
        // lock against parent joint -1, reading before the start of the pose.
        try std.testing.expectError(
            error.InvalidArgument,
            some.lockTranslations(pair.neutral2, &.{ true, false, false }),
        );
        // And nothing was recorded by the refused call.
        try std.testing.expect(!some.isJointTranslationLocked(0));

        // The array is one bool per joint, not per anything else.
        try std.testing.expectError(
            error.InvalidArgument,
            some.lockTranslations(pair.neutral2, &.{ false, false }),
        );
    }
}

test "GetChains reports the joints of skeleton 2 between two mapped joints" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // Skeleton 1: root -> hand, directly. Skeleton 2 has the same two named
    // joints, but with an extra "elbow" between them that skeleton 1 has no
    // equivalent for — exactly the shape a chain describes.
    var low = try zjolt.Skeleton.init();
    defer low.release();
    const low_root = try low.addJoint("root", null);
    _ = try low.addJoint("hand", low_root);

    var high = try zjolt.Skeleton.init();
    defer high.release();
    const high_root = try high.addJoint("root", null);
    const high_elbow = try high.addJoint("elbow", high_root);
    _ = try high.addJoint("hand", high_elbow);

    const neutral_low = try zjolt.SkeletonPose.init();
    defer neutral_low.deinit();
    try neutral_low.setSkeleton(low);
    try neutral_low.setJoints(
        &.{ zjolt.quat_identity, zjolt.quat_identity },
        &.{ zjolt.vec3_zero, zjolt.vec3(0, 2, 0) },
    );
    try neutral_low.calculateJointMatrices();

    const neutral_high = try zjolt.SkeletonPose.init();
    defer neutral_high.deinit();
    try neutral_high.setSkeleton(high);
    try neutral_high.setJoints(
        &.{ zjolt.quat_identity, zjolt.quat_identity, zjolt.quat_identity },
        &.{ zjolt.vec3_zero, zjolt.vec3(0, 1, 0), zjolt.vec3(0, 1, 0) },
    );
    try neutral_high.calculateJointMatrices();

    const mapper = try zjolt.SkeletonMapper.init();
    defer mapper.release();
    try mapper.initialize(neutral_low, neutral_high, null, null);

    try std.testing.expectEqual(@as(u32, 2), mapper.mappingCount());
    try std.testing.expectEqual(@as(u32, 0), mapper.unmappedCount());
    try std.testing.expectEqual(@as(u32, 1), mapper.chainCount());

    const counts = mapper.chainJointCounts(0);
    try std.testing.expectEqual(@as(u32, 2), counts.count1);
    try std.testing.expectEqual(@as(u32, 3), counts.count2);

    // Skeleton 1's run is just the two mapped ends, root then hand.
    try std.testing.expectEqual(@as(?u32, 0), mapper.chainJointIndex1(0, 0));
    try std.testing.expectEqual(@as(?u32, 1), mapper.chainJointIndex1(0, 1));
    try std.testing.expectEqual(@as(?u32, null), mapper.chainJointIndex1(0, 2));

    // Skeleton 2's run is the same two ends with elbow in between.
    try std.testing.expectEqual(@as(?u32, 0), mapper.chainJointIndex2(0, 0));
    try std.testing.expectEqual(@as(?u32, high_elbow), mapper.chainJointIndex2(0, 1));
    try std.testing.expectEqual(@as(?u32, 2), mapper.chainJointIndex2(0, 2));
    try std.testing.expectEqual(@as(?u32, null), mapper.chainJointIndex2(0, 3));

    // Past the chain count, both axes report the same as an empty chain.
    try std.testing.expectEqual(@as(?u32, null), mapper.chainJointIndex1(1, 0));
    try std.testing.expectEqual(@as(?u32, null), mapper.chainJointIndex2(1, 0));
    const empty_counts = mapper.chainJointCounts(1);
    try std.testing.expectEqual(@as(u32, 0), empty_counts.count1);
    try std.testing.expectEqual(@as(u32, 0), empty_counts.count2);
}

test "GetUnmapped reports a joint of skeleton 2 that has no counterpart at all" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    // The render skeleton's "finger" is a leaf beyond the last mapped joint,
    // not a joint between two mapped ones — so it is unmapped, not a chain.
    var pair = try SkeletonPair.build();
    defer pair.deinit();

    const mapper = try zjolt.SkeletonMapper.init();
    defer mapper.release();
    try mapper.initialize(pair.neutral1, pair.neutral2, null, null);

    try std.testing.expectEqual(@as(u32, 0), mapper.chainCount());
    try std.testing.expectEqual(@as(u32, 1), mapper.unmappedCount());

    const finger = try mapper.unmapped(0);
    try std.testing.expectEqual(@as(u32, 2), finger.joint_index);
    try std.testing.expectEqual(@as(?u32, 1), finger.parent_joint_index);

    try std.testing.expectError(error.InvalidArgument, mapper.unmapped(1));
}

//=============================================================================
// Helpers
//=============================================================================

/// The angle between two rotations, in radians.
fn quatAngle(a: zjolt.Quat, b: zjolt.Quat) f32 {
    const dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    const clamped = std.math.clamp(@abs(dot), 0.0, 1.0);
    return 2.0 * std.math.acos(clamped);
}

/// A three-link chain: root, one joint below it, one below that. The shortest
/// hierarchy in which "nearer the root" and "nearer the leaf" are different
/// constraints.
const Limb = struct {
    skeleton: zjolt.Skeleton,
    settings: zjolt.RagdollSettings,
    shapes: [3]zjolt.Shape,

    fn build() !Limb {
        var skeleton = try zjolt.Skeleton.init();
        errdefer skeleton.release();
        const root = try skeleton.addJoint("root", null);
        const middle = try skeleton.addJoint("middle", root);
        _ = try skeleton.addJoint("leaf", middle);

        var shapes: [3]zjolt.Shape = undefined;
        var made: usize = 0;
        errdefer for (shapes[0..made]) |shape| shape.release();
        while (made < shapes.len) : (made += 1) {
            shapes[made] = try zjolt.Shape.initBox(zjolt.vec3(0.3, 0.4, 0.3), .{});
        }

        var settings = try zjolt.RagdollSettings.init();
        errdefer settings.release();

        var parts: [3]zjolt.RagdollPartDesc = undefined;
        var joints: [3]zjolt.RagdollConstraintDesc = undefined;
        for (&parts, 0..) |*part, i| {
            const y: f32 = 6.0 + @as(f32, @floatFromInt(i));
            part.* = .{
                .body = .{
                    .shape = shapes[i],
                    .object_layer = Layers.moving,
                    .position = zjolt.rvec3(0, y, 0),
                },
                .to_parent = null,
            };
            if (i == 0) continue;
            joints[i] = .{
                .position1 = zjolt.rvec3(0, y - 0.5, 0),
                .twist_axis1 = zjolt.vec3(0, 1, 0),
                .plane_axis1 = zjolt.vec3(1, 0, 0),
                .position2 = zjolt.rvec3(0, y - 0.5, 0),
                .twist_axis2 = zjolt.vec3(0, 1, 0),
                .plane_axis2 = zjolt.vec3(1, 0, 0),
                .normal_half_cone_angle = 1.0,
                .plane_half_cone_angle = 1.0,
                .twist_min_angle = -1.0,
                .twist_max_angle = 1.0,
            };
            part.to_parent = joints[i];
        }

        try settings.build(std.testing.allocator, skeleton, &parts);
        return .{ .skeleton = skeleton, .settings = settings, .shapes = shapes };
    }

    fn deinit(self: *Limb) void {
        self.settings.release();
        self.skeleton.release();
        for (self.shapes) |shape| shape.release();
    }
};

/// The two skeletons a mapper joins, and a neutral pose of each in model
/// space — which is the form `SkeletonMapper.initialize` reads.
///
/// The render skeleton is the ragdoll's with one extra leaf: a joint no
/// ragdoll part drives, which is the case `map` has to rebuild from the
/// render pose's own local transforms.
const SkeletonPair = struct {
    ragdoll_skeleton: zjolt.Skeleton,
    render_skeleton: zjolt.Skeleton,
    neutral1: zjolt.SkeletonPose,
    neutral2: zjolt.SkeletonPose,

    fn build() !SkeletonPair {
        var ragdoll_skeleton = try zjolt.Skeleton.init();
        errdefer ragdoll_skeleton.release();
        const r_root = try ragdoll_skeleton.addJoint("root", null);
        _ = try ragdoll_skeleton.addJoint("hand", r_root);

        var render_skeleton = try zjolt.Skeleton.init();
        errdefer render_skeleton.release();
        const d_root = try render_skeleton.addJoint("root", null);
        const d_hand = try render_skeleton.addJoint("hand", d_root);
        _ = try render_skeleton.addJoint("finger", d_hand);

        const neutral1 = try zjolt.SkeletonPose.init();
        errdefer neutral1.deinit();
        try neutral1.setSkeleton(ragdoll_skeleton);
        try neutral1.setJoints(
            &.{ zjolt.quat_identity, zjolt.quat_identity },
            &.{ zjolt.vec3_zero, zjolt.vec3(0, 1, 0) },
        );
        try neutral1.calculateJointMatrices();

        const neutral2 = try zjolt.SkeletonPose.init();
        errdefer neutral2.deinit();
        try neutral2.setSkeleton(render_skeleton);
        try neutral2.setJoints(
            &.{ zjolt.quat_identity, zjolt.quat_identity, zjolt.quat_identity },
            &.{ zjolt.vec3_zero, zjolt.vec3(0, 1, 0), zjolt.vec3(0, 0.5, 0) },
        );
        try neutral2.calculateJointMatrices();

        return .{
            .ragdoll_skeleton = ragdoll_skeleton,
            .render_skeleton = render_skeleton,
            .neutral1 = neutral1,
            .neutral2 = neutral2,
        };
    }

    fn deinit(self: *SkeletonPair) void {
        self.neutral2.deinit();
        self.neutral1.deinit();
        self.render_skeleton.release();
        self.ragdoll_skeleton.release();
    }
};

/// A render pose in its rest animation — what `map` reads for the joints the
/// ragdoll does not drive, and writes the rest of.
fn renderPose(pair: SkeletonPair) !zjolt.SkeletonPose {
    const pose = try zjolt.SkeletonPose.init();
    errdefer pose.deinit();
    try pose.setSkeleton(pair.render_skeleton);
    try pose.setJoints(
        &.{ zjolt.quat_identity, zjolt.quat_identity, zjolt.quat_identity },
        &.{ zjolt.vec3_zero, zjolt.vec3(0, 1, 0), zjolt.vec3(0, 0.5, 0) },
    );
    return pose;
}

fn expectVec3Near(expected: zjolt.Vec3, actual: zjolt.Vec3) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, 1e-4);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, 1e-4);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, 1e-4);
}
