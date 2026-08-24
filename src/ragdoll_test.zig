//! Behavioural tests for ragdolls: a two-part chain (root and child, joined
//! by a swing-twist constraint) that either falls under gravity or is driven
//! toward a target pose by its motors.
//!
//! Reuses `integration_test.zig`'s layer map and floor fixture rather than
//! building its own — see `BINDING.md`.
//!
//! `RagdollSettings.createRagdoll` hands back a `Ragdoll`, not the body ids
//! it spawned, so both tests below recover them the same way: read every
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
                    // Generous on purpose: wide enough that neither test
                    // below is really exercising the limits, only the
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
    // NOT just `ragdoll.release()`: see the defect note by the same defer in
    // the driving test below. Removing first is what makes this safe.
    defer {
        ragdoll.removeFromPhysicsSystem(true);
        ragdoll.release();
    }
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
    // DEFECT: `Ragdoll.release` is documented as "removing [bodies] from
    // their system first if still added", but Jolt's `~Ragdoll()` just calls
    // `BodyInterface::DestroyBodies` directly — no removal step — and
    // `DestroyBodies` asserts every body is already inactive and out of the
    // broad phase. Releasing a still-added, still-awake ragdoll (as both
    // tests in this file do) trips that assert instead of behaving as
    // documented. Removing explicitly first, as done here, is the only way
    // that actually works.
    defer {
        ragdoll.removeFromPhysicsSystem(true);
        ragdoll.release();
    }
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
    const target_rotation = zjolt.quatFromAxisAngle(zjolt.vec3(0, 0, 1), 0.7);

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
// Helpers
//=============================================================================

/// The angle between two rotations, in radians.
fn quatAngle(a: zjolt.Quat, b: zjolt.Quat) f32 {
    const dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    const clamped = std.math.clamp(@abs(dot), 0.0, 1.0);
    return 2.0 * std.math.acos(clamped);
}
