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
    const target_rotation = try zjolt.quatFromAxisAngle(zjolt.vec3(0, 0, 1), 0.7);

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
    const twist = try zjolt.quatFromAxisAngle(zjolt.vec3(0, 0, 1), 0.5);
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
