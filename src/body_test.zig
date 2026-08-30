//! Behavioural tests for the body surface added past the original
//! create/destroy/velocity/force set: force and torque read-back, runtime
//! mass and inertia, the three flags with no `BodyDesc` field of their own,
//! a body id's own generation counter, and detaching a body from its id
//! without destroying it.
//!
//! Also covers a later pair: a general multi-body lock (BodyLockMultiRead/
//! Write, including Jolt's own early release), and
//! MotionProperties::GetSimulationStats.
//!
//! Reuses `integration_test.zig`'s fixture rather than building a new one —
//! see `state_test.zig` for the same reasoning.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const fixture = @import("integration_test.zig");
const body_mod = @import("body.zig");
const c = @import("c.zig");

const Layers = fixture.Layers;
const World = fixture.World;

//=============================================================================
// Force and torque read-back
//=============================================================================

test "an added force and torque read back exactly, and reset clears them" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .allow_sleeping = false,
    }, .dont_activate);

    // Nothing applied yet: the accumulator starts at zero.
    try expectVec3Near(zjolt.vec3_zero, bodies.getAccumulatedForce(ball), 1e-6);
    try expectVec3Near(zjolt.vec3_zero, bodies.getAccumulatedTorque(ball), 1e-6);

    const force = zjolt.vec3(0, 0, 100);
    const torque = zjolt.vec3(0, 50, 0);
    bodies.addForceAndTorque(ball, force, torque);

    // The step has not run, so both are exactly what was added -- this is
    // the fact a getter that silently read a default (rather than the real
    // accumulator) would fail to reproduce.
    try expectVec3Near(force, bodies.getAccumulatedForce(ball), 1e-6);
    try expectVec3Near(torque, bodies.getAccumulatedTorque(ball), 1e-6);

    bodies.resetForce(ball);
    try expectVec3Near(zjolt.vec3_zero, bodies.getAccumulatedForce(ball), 1e-6);
    // Torque is untouched by resetForce.
    try expectVec3Near(torque, bodies.getAccumulatedTorque(ball), 1e-6);

    bodies.resetTorque(ball);
    try expectVec3Near(zjolt.vec3_zero, bodies.getAccumulatedTorque(ball), 1e-6);

    // A STATIC body has nothing to accumulate: the getters read zero and the
    // resets are no-ops rather than reaching for motion properties it does
    // not have.
    try expectVec3Near(zjolt.vec3_zero, bodies.getAccumulatedForce(world.floor), 1e-6);
    bodies.addForceAndTorque(ball, force, torque);
    try world.stepFor(1.0 / 60.0);
    // The step itself clears what it consumed.
    try expectVec3Near(zjolt.vec3_zero, bodies.getAccumulatedForce(ball), 1e-6);
}

//=============================================================================
// Mass and inertia
//=============================================================================

test "GetInverseInertiaForRotation at the body's own rotation matches GetInverseInertia, and at identity matches GetLocalSpaceInverseInertia" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.7, 0.9), .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const box = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    const tilt = try zjolt.Quat.fromAxisAngle(zjolt.vec3(1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0), 0.9);
    bodies.setPositionAndRotation(box, bodies.getPosition(box), tilt, .dont_activate);

    // GetInverseInertiaForRotation at the body's ACTUAL current rotation is
    // exactly what Body::GetInverseInertia computes internally
    // (Body.inl:124) -- if the new entry point ignored `rotation` and always
    // used the body's own orientation, this would still pass, so the
    // identity check below is what actually pins the parameter down.
    const world_space_matrix = try mat44FromQuat(tilt);
    const inverse_inertia = try bodies.getInverseInertia(box);
    const for_own_rotation = try bodies.getInverseInertiaForRotation(box, world_space_matrix);
    try expectMat44Near(inverse_inertia, for_own_rotation, 1e-4);

    // At an IDENTITY hypothetical rotation, the same call has to match the
    // LOCAL-space inertia instead -- a different value, since the body is
    // tilted. Confusing the two (or ignoring the parameter) fails one of
    // these two checks.
    const local_space = try bodies.getLocalSpaceInverseInertia(box);
    const for_identity = try bodies.getInverseInertiaForRotation(box, zjolt.mat44_identity);
    try expectMat44Near(local_space, for_identity, 1e-4);
    try std.testing.expect(!mat44Near(inverse_inertia, local_space, 1e-4));
}

test "ScaleToMass rescales mass and inertia together, and SetMassProperties gives a custom tensor and DOF mask at once" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .override_mass_properties = .calculate_inertia,
        .mass = 2.0,
    }, .dont_activate);

    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 2.0), bodies.getInverseMassUnchecked(ball), 1e-5);
    const inv_inertia_before = try bodies.getLocalSpaceInverseInertia(ball);

    try bodies.scaleToMass(ball, 4.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 4.0), bodies.getInverseMassUnchecked(ball), 1e-5);
    // Doubling the mass halves the inverse inertia, in every entry of the
    // actual 3x3 tensor: the two are scaled by the same ratio, which is the
    // entire point of ScaleToMass over setting mass and inertia separately.
    // (Row 3 / column 3 is Jolt's own homogeneous padding on a Mat44 used as
    // a 3x3 -- 0 off the diagonal, 1 at (3, 3) -- and never scales.)
    const inv_inertia_after = try bodies.getLocalSpaceInverseInertia(ball);
    for (0..3) |col| {
        for (0..3) |row| {
            const index = 4 * col + row;
            try std.testing.expectApproxEqAbs(inv_inertia_before.m[index] * 0.5, inv_inertia_after.m[index], 1e-4);
        }
    }

    // A body with no finite mass to scale -- every translation DOF locked --
    // is refused rather than dividing by zero.
    const locked = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(3, 5, 0),
        .allowed_dofs = .{
            .translation_x = false,
            .translation_y = false,
            .translation_z = false,
        },
    }, .dont_activate);
    try std.testing.expectError(zjolt.Error.InvalidArgument, bodies.scaleToMass(locked, 1.0));
    // A no-op on a STATIC body, not an error.
    try bodies.scaleToMass(world.floor, 1.0);

    // SetMassProperties: a custom tensor AND a DOF mask in one call.
    try std.testing.expectEqual(zjolt.AllowedDofs.all, bodies.getAllowedDOFs(ball));
    const custom = zjolt.MassProperties{ .mass = 10.0, .inertia = .{
        1, 0, 0,
        0, 2, 0,
        0, 0, 3,
    } };
    try bodies.setMassProperties(ball, .plane_2d, custom);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 10.0), bodies.getInverseMassUnchecked(ball), 1e-5);
    try std.testing.expectEqual(zjolt.AllowedDofs.plane_2d, bodies.getAllowedDOFs(ball));

    // Jolt asserts on both of these rather than checking them; both come
    // back as ordinary errors instead.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.setMassProperties(ball, .{
            .translation_x = false,
            .translation_y = false,
            .translation_z = false,
            .rotation_x = false,
            .rotation_y = false,
            .rotation_z = false,
        }, custom),
    );
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.setMassProperties(ball, .all, .{ .mass = 0, .inertia = @splat(0) }),
    );
}

test "setInverseMass reads back exactly, including zero on a still-translating body" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .override_mass_properties = .calculate_inertia,
        .mass = 2.0,
    }, .dont_activate);

    try bodies.setInverseMass(ball, 0.3717);
    try std.testing.expectEqual(@as(f32, 0.3717), bodies.getInverseMassUnchecked(ball));

    // The case setMassProperties cannot express while any translation axis
    // is allowed: an infinitely heavy body (inverse mass exactly zero) that
    // still translates.
    try bodies.setInverseMass(ball, 0.0);
    try std.testing.expectEqual(@as(f32, 0.0), bodies.getInverseMassUnchecked(ball));
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.setMassProperties(ball, .all, .{ .mass = 0, .inertia = @splat(0) }),
    );

    // A no-op on a STATIC body, not an error.
    try bodies.setInverseMass(world.floor, 1.0);
}

test "setInverseInertia reads back exactly, not the lossy DecomposePrincipalMomentsOfInertia substitute" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .override_mass_properties = .calculate_inertia,
        .mass = 2.0,
    }, .dont_activate);

    // A zero inertia tensor makes DecomposePrincipalMomentsOfInertia return a
    // near-zero diagonal, which MotionProperties::SetMassProperties treats as
    // failure: it silently substitutes a unit-sphere-shaped inverse inertia
    // (2.5 * invMass on every axis, identity rotation) instead of anything
    // derived from the zero tensor that was actually given.
    try bodies.setMassProperties(ball, .all, .{ .mass = 2.0, .inertia = @splat(0) });
    const inv_mass = bodies.getInverseMassUnchecked(ball);
    const fallback = zjolt.Mat44{ .m = .{
        2.5 * inv_mass, 0,              0,              0,
        0,              2.5 * inv_mass, 0,              0,
        0,              0,              2.5 * inv_mass, 0,
        0,              0,              0,              1,
    } };
    try expectMat44Near(fallback, try bodies.getLocalSpaceInverseInertia(ball), 1e-5);

    // setInverseInertia bypasses that substitute entirely: what comes back
    // is exactly R * diag(diagonal) * R^-1 for the (diagonal, rotation) that
    // was set, not the fallback above.
    const diagonal = zjolt.vec3(0.2, 0.4, 0.6);
    const rotation = try zjolt.Quat.fromAxisAngle(zjolt.vec3(0, 1, 0), 0.5);
    try bodies.setInverseInertia(ball, diagonal, rotation);

    const r = try mat44FromQuat(rotation);
    const diag = zjolt.Mat44{ .m = .{
        diagonal.x, 0,          0,          0,
        0,          diagonal.y, 0,          0,
        0,          0,          diagonal.z, 0,
        0,          0,          0,          1,
    } };
    const expected = r.multiply(diag).multiply(r.inverse());
    const actual = try bodies.getLocalSpaceInverseInertia(ball);
    try expectMat44Near(expected, actual, 1e-4);
    try std.testing.expect(!mat44Near(fallback, actual, 1e-4));

    // A no-op on a STATIC body, not an error.
    try bodies.setInverseInertia(world.floor, diagonal, rotation);
}

test "maskTranslationDOFs and maskAngularDOFs zero exactly the axes SetMassProperties locks" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    // Every DOF allowed by default: neither mask changes anything.
    try expectVec3Near(zjolt.vec3(1, 2, 3), bodies.maskTranslationDOFs(ball, zjolt.vec3(1, 2, 3)), 1e-6);
    try expectVec3Near(zjolt.vec3(4, 5, 6), bodies.maskAngularDOFs(ball, zjolt.vec3(4, 5, 6)), 1e-6);

    // Lock Y translation and Y rotation; the other two axes of each survive.
    try bodies.setMassProperties(ball, .{
        .translation_y = false,
        .rotation_y = false,
    }, .{ .mass = 10.0, .inertia = .{
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    } });
    try expectVec3Near(zjolt.vec3(1, 0, 3), bodies.maskTranslationDOFs(ball, zjolt.vec3(1, 2, 3)), 1e-6);
    try expectVec3Near(zjolt.vec3(4, 0, 6), bodies.maskAngularDOFs(ball, zjolt.vec3(4, 5, 6)), 1e-6);

    // A STATIC body has no motion properties, which reads as "every DOF
    // allowed" -- the same default getAllowedDOFs answers -- not as zero.
    try expectVec3Near(zjolt.vec3(1, 2, 3), bodies.maskTranslationDOFs(world.floor, zjolt.vec3(1, 2, 3)), 1e-6);
}

test "AllowedDofs.linearMask and .angularMask split into translation and rotation triples" {
    try std.testing.expectEqual([3]bool{ true, true, true }, @as(zjolt.AllowedDofs, .all).linearMask());
    try std.testing.expectEqual([3]bool{ true, true, true }, @as(zjolt.AllowedDofs, .all).angularMask());

    // plane_2d allows X/Y translation and Z rotation only.
    try std.testing.expectEqual([3]bool{ true, true, false }, @as(zjolt.AllowedDofs, .plane_2d).linearMask());
    try std.testing.expectEqual([3]bool{ false, false, true }, @as(zjolt.AllowedDofs, .plane_2d).angularMask());
}

test "clampLinearVelocity and clampAngularVelocity clamp only past the limit, and are a no-op on a STATIC body" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    // SetLinearVelocity/SetAngularVelocity clamp on the way in, so an
    // over-limit velocity is reached by raising the ceiling, storing one
    // under it, and then lowering the ceiling again -- not by setting one
    // that is already too large (which the ordinary setters would just
    // clamp on entry). Jolt's own default max_angular_velocity (15*pi) is
    // itself below 100, hence raising it first.
    bodies.setMaxAngularVelocity(ball, 200.0);
    bodies.setLinearVelocity(ball, zjolt.vec3(100, 0, 0));
    bodies.setAngularVelocity(ball, zjolt.vec3(0, 100, 0));

    // SetMaxLinearVelocity/SetMaxAngularVelocity only change the ceiling
    // field; they do not reclamp what is already stored.
    bodies.setMaxLinearVelocity(ball, 10.0);
    bodies.setMaxAngularVelocity(ball, 20.0);
    try expectVec3Near(zjolt.vec3(100, 0, 0), bodies.getLinearVelocity(ball), 1e-4);
    try expectVec3Near(zjolt.vec3(0, 100, 0), bodies.getAngularVelocity(ball), 1e-4);

    // Past the limit: scaled down to exactly the ceiling, direction kept.
    try bodies.clampLinearVelocity(ball);
    try bodies.clampAngularVelocity(ball);
    try expectVec3Near(zjolt.vec3(10.0, 0, 0), bodies.getLinearVelocity(ball), 1e-4);
    try expectVec3Near(zjolt.vec3(0, 20.0, 0), bodies.getAngularVelocity(ball), 1e-4);

    // At the limit exactly: the condition is `>`, not `>=`, so calling
    // again now is a no-op.
    try bodies.clampLinearVelocity(ball);
    try bodies.clampAngularVelocity(ball);
    try expectVec3Near(zjolt.vec3(10.0, 0, 0), bodies.getLinearVelocity(ball), 1e-4);
    try expectVec3Near(zjolt.vec3(0, 20.0, 0), bodies.getAngularVelocity(ball), 1e-4);

    // A no-op on a STATIC body, not an error.
    try bodies.clampLinearVelocity(world.floor);
    try bodies.clampAngularVelocity(world.floor);

    // A stale id is BodyNotFound, not a silent no-op.
    bodies.destroy(ball);
    try std.testing.expectError(zjolt.Error.BodyNotFound, bodies.clampLinearVelocity(ball));
    try std.testing.expectError(zjolt.Error.BodyNotFound, bodies.clampAngularVelocity(ball));
}

test "multiplyWorldSpaceInverseInertiaByVector matches diag(diagonal) * v at identity rotation, and refuses a non-dynamic body" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .rotation = zjolt.quat_identity,
    }, .dont_activate);

    // At an identity body rotation and an identity inertia rotation, world
    // space and local space coincide: the product is just the diagonal
    // scaling each component of `v`, by hand.
    try bodies.setInverseInertia(ball, zjolt.vec3(2, 3, 4), zjolt.quat_identity);
    const result = try bodies.multiplyWorldSpaceInverseInertiaByVector(ball, zjolt.vec3(1, 1, 1));
    try expectVec3Near(zjolt.vec3(2, 3, 4), result, 1e-4);

    // Only a DYNAMIC body has an inverse inertia -- a STATIC one is refused
    // rather than dereferencing motion properties it does not have.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.multiplyWorldSpaceInverseInertiaByVector(world.floor, zjolt.vec3_zero),
    );

    // A stale id is BodyNotFound.
    bodies.destroy(ball);
    try std.testing.expectError(
        zjolt.Error.BodyNotFound,
        bodies.multiplyWorldSpaceInverseInertiaByVector(ball, zjolt.vec3_zero),
    );
}

test "getLocalSpaceInverseInertiaUnchecked matches getLocalSpaceInverseInertia on an otherwise-identical dynamic body, and refuses STATIC" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();

    // Both bodies compute the same shape-derived MotionProperties fields at
    // creation time regardless of motion type -- HasMassProperties is true
    // for KINEMATIC exactly as it is for DYNAMIC -- so the two have to agree.
    const dynamic_ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .override_mass_properties = .calculate_inertia,
        .mass = 3.0,
    }, .dont_activate);
    const kinematic_ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(3, 5, 0),
        .motion_type = .kinematic,
        .override_mass_properties = .calculate_inertia,
        .mass = 3.0,
    }, .dont_activate);

    const dynamic_inertia = try bodies.getLocalSpaceInverseInertia(dynamic_ball);
    const kinematic_inertia = try bodies.getLocalSpaceInverseInertiaUnchecked(kinematic_ball);
    try expectMat44Near(dynamic_inertia, kinematic_inertia, 1e-4);

    // Only a STATIC body (no motion properties at all) is refused.
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.getLocalSpaceInverseInertiaUnchecked(world.floor),
    );

    // A stale id is BodyNotFound.
    bodies.destroy(dynamic_ball);
    try std.testing.expectError(
        zjolt.Error.BodyNotFound,
        bodies.getLocalSpaceInverseInertiaUnchecked(dynamic_ball),
    );
}

test "getUseManifoldReductionWithBody is AND of both bodies, getEnhancedInternalEdgeRemovalWithBody is OR, and a missing body reads each accessor's own default" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const a = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);
    const b = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(3, 5, 0),
    }, .dont_activate);

    // Both true by default (manifold reduction) / both false by default
    // (enhanced internal edge removal).
    try std.testing.expect(bodies.getUseManifoldReductionWithBody(a, b));
    try std.testing.expect(!bodies.getEnhancedInternalEdgeRemovalWithBody(a, b));

    bodies.setUseManifoldReduction(a, false);
    try std.testing.expect(!bodies.getUseManifoldReductionWithBody(a, b));
    try std.testing.expect(bodies.getUseManifoldReduction(b));

    // GetEnhancedInternalEdgeRemoval has no BodyInterface setter; apply
    // through ApplyBodyCreationSettings on an unadded body instead.
    bodies.remove(a);
    try bodies.applyBodyCreationSettings(a, .{
        .shape = shape,
        .object_layer = Layers.moving,
        .enhanced_internal_edge_removal = true,
    });
    bodies.add(a, .dont_activate);
    try std.testing.expect(bodies.getEnhancedInternalEdgeRemoval(a));
    try std.testing.expect(!bodies.getEnhancedInternalEdgeRemoval(b));
    // OR: either body asking for it is enough.
    try std.testing.expect(bodies.getEnhancedInternalEdgeRemovalWithBody(a, b));

    // A stale id reads the same default the single-body getter would.
    bodies.destroy(b);
    try std.testing.expect(bodies.getUseManifoldReductionWithBody(a, b));
    try std.testing.expect(!bodies.getEnhancedInternalEdgeRemovalWithBody(a, b));
}

test "applyBodyCreationSettings overwrites an unadded body's state, and refuses an added body or a body missing motion properties" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();

    // Created but never added: ApplyBodyCreationSettings' own requirement.
    const ball = try bodies.create(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
        .friction = 0.2,
    });
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), bodies.getFriction(ball), 1e-5);

    try bodies.applyBodyCreationSettings(ball, .{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(1, 6, 0),
        .friction = 0.9,
        .restitution = 0.4,
    });
    try std.testing.expectApproxEqAbs(@as(f32, 0.9), bodies.getFriction(ball), 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), bodies.getRestitution(ball), 1e-5);
    try expectRVec3Near(zjolt.rvec3(1, 6, 0), bodies.getPosition(ball), 1e-4);

    // Refused once the body is added -- Jolt's own assert is
    // `IsRigidBody() && !IsInBroadPhase()`.
    bodies.add(ball, .dont_activate);
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.applyBodyCreationSettings(ball, .{ .shape = shape, .object_layer = Layers.moving }),
    );

    // A STATIC body created without allow_dynamic_or_kinematic has no
    // motion properties to receive a desc that implies any.
    const static_only = try bodies.create(.{
        .shape = shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .allow_dynamic_or_kinematic = false,
        .position = zjolt.rvec3(3, 5, 0),
    });
    try std.testing.expectError(
        zjolt.Error.InvalidArgument,
        bodies.applyBodyCreationSettings(static_only, .{
            .shape = shape,
            .object_layer = Layers.moving,
            .motion_type = .dynamic,
        }),
    );
}

test "a NULL shape fails through ConvertShapeSettings, not a hand-rolled null check" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    // Below the friendly BodyDesc, which requires a shape at the type level:
    // BuildCreationSettings resolves the C desc's shape by calling
    // BodyCreationSettings::ConvertShapeSettings rather than checking
    // `desc.shape == NULL` itself, and a NULL shape takes ConvertShapeSettings'
    // own "no shape present" branch, same as a ShapeSettings that fails to
    // build.
    var desc: c.body.BodyDesc = undefined;
    c.body.zjoltBodyDescInit(&desc);
    desc.object_layer = Layers.moving;
    desc.position = zjolt.rvec3(0, 5, 0);

    var out: zjolt.BodyId = zjolt.invalid_body_id;
    const result = c.body.zjoltBodyCreate(world.system.handle, &desc, &out);
    try std.testing.expectEqual(c.body.Result.invalid_argument, result);
    try std.testing.expectEqual(zjolt.invalid_body_id, out);
}

//=============================================================================
// Flags with no BodyDesc field, and the sequence number
//=============================================================================

test "the three flag getters read Jolt's own construction-time default" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    try std.testing.expect(!bodies.getApplyGyroscopicForce(ball));
    try std.testing.expect(!bodies.getCollideKinematicVsNonDynamic(ball));
    try std.testing.expect(!bodies.getEnhancedInternalEdgeRemoval(ball));

    // Same false for a stale id -- reading Body::mFlags through a failed
    // lock, not through whatever the next body created happens to hold.
    bodies.destroy(ball);
    try std.testing.expect(!bodies.getApplyGyroscopicForce(ball));
}

test "hasMotionProperties tells a body allowed to switch motion type apart from an ordinary static one" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();

    const dynamic_ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);
    try std.testing.expect(bodies.hasMotionProperties(dynamic_ball));

    // world.floor is an ordinary static body: no motion properties.
    try std.testing.expect(!bodies.hasMotionProperties(world.floor));

    // A body created STATIC but switchable reports the same motion type as
    // an ordinary static body -- getMotionType alone cannot tell them apart
    // -- yet it holds motion properties underneath, ready for the switch.
    const switchable = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.static,
        .motion_type = .static,
        .allow_dynamic_or_kinematic = true,
        .position = zjolt.rvec3(3, 5, 0),
    }, .dont_activate);
    try std.testing.expectEqual(zjolt.MotionType.static, bodies.getMotionType(switchable));
    try std.testing.expectEqual(zjolt.MotionType.static, bodies.getMotionType(world.floor));
    try std.testing.expect(bodies.hasMotionProperties(switchable));

    bodies.setMotionType(switchable, .dynamic, .dont_activate);
    try std.testing.expectEqual(zjolt.MotionType.dynamic, bodies.getMotionType(switchable));
}

test "isCollisionCacheInvalid reads back what invalidateContactCache sets, and a stale id reads false" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    try std.testing.expect(!bodies.isCollisionCacheInvalid(ball));
    bodies.invalidateContactCache(ball);
    try std.testing.expect(bodies.isCollisionCacheInvalid(ball));

    bodies.destroy(ball);
    try std.testing.expect(!bodies.isCollisionCacheInvalid(ball));
}

test "Body.isDynamic matches motionType == .dynamic, true for an ordinary body and false for the STATIC floor" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);

    {
        var lock = world.system.lockRead(ball);
        defer lock.release();
        const locked = lock.body() orelse return error.TestUnexpectedResult;
        try std.testing.expect(locked.isDynamic());
    }
    {
        var lock = world.system.lockRead(world.floor);
        defer lock.release();
        const locked = lock.body() orelse return error.TestUnexpectedResult;
        try std.testing.expect(!locked.isDynamic());
    }
}

test "a body's sequence number advances when its index is recycled, and MustBeStatic matches the shape kind" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const first = try bodies.create(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    });
    const first_index = first & 0x7fffff;
    const first_seq = zjolt.getBodySequenceNumber(first);
    bodies.destroy(first);

    // The only free index is the one just vacated, so an auto-id create
    // recycles it -- same index, sequence number advanced by one. That is
    // exactly what lets a host tell "the body I remember" apart from
    // "whatever was created at this index since."
    const second = try bodies.create(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    });
    try std.testing.expectEqual(first_index, second & 0x7fffff);
    try std.testing.expectEqual(first_seq +% 1, zjolt.getBodySequenceNumber(second));

    const box_shape = try zjolt.Shape.initBox(zjolt.vec3(1, 1, 1), .{});
    defer box_shape.release();
    try std.testing.expect(!zjolt.shapeMustBeStatic(box_shape));

    const plane_shape = try zjolt.Shape.initPlane(zjolt.vec3(0, 1, 0), 0, .{});
    defer plane_shape.release();
    try std.testing.expect(zjolt.shapeMustBeStatic(plane_shape));
}

//=============================================================================
// Detaching a body from its id
//=============================================================================

test "unassignId frees the id but keeps the body alive for assignId, and the bulk form skips a stale id" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const original_position = zjolt.rvec3(1.5, 5.0, -2.5);
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = original_position,
        .user_data = 0xC0FFEE,
    }, .dont_activate);

    const unassigned = try bodies.unassignId(ball);

    // The old id is genuinely free: reading through it now sees a stale
    // id's defaults, not the body it once named.
    try std.testing.expect(!bodies.isAdded(ball));
    try std.testing.expectEqual(@as(u32, 1), world.system.numBodies()); // the floor only

    // A chosen id well clear of anything auto-assignment would ever pick.
    const chosen: zjolt.BodyId = 500;
    const reassigned = try unassigned.assignId(chosen);
    try std.testing.expectEqual(chosen, reassigned);

    // Same object: its position survived detachment untouched.
    const position = bodies.getPosition(reassigned);
    try std.testing.expectApproxEqAbs(original_position.x, position.x, 1e-6);
    try std.testing.expectApproxEqAbs(original_position.y, position.y, 1e-6);
    try std.testing.expectApproxEqAbs(original_position.z, position.z, 1e-6);
    try std.testing.expectEqual(@as(u64, 0xC0FFEE), bodies.getUserData(reassigned));

    bodies.add(reassigned, .dont_activate);
    try std.testing.expect(bodies.isAdded(reassigned));

    // Bulk form: one live body and one already-destroyed id in the same call.
    const other = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .dont_activate);
    const doomed = try bodies.create(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    });
    bodies.destroy(doomed);

    const batch = world.system.batch();
    const results = try batch.unassignIds(std.testing.allocator, &.{ other, doomed });
    defer std.testing.allocator.free(results);
    try std.testing.expect(results[0] != null);
    try std.testing.expect(results[1] == null);
    try (results[0].?).destroy();
}

//=============================================================================
// Multi-body locks
//=============================================================================

test "a multi-body lock reads several bodies consistently, skips a destroyed one, and releases early" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const a = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(-2, 5, 0),
    }, .activate);
    const b = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(2, 8, 0),
    }, .activate);

    try world.stepFor(0.5);

    // What the ordinary per-body accessors say right now, read before the
    // multi-lock so nothing in between can move either body out from under
    // the comparison below.
    const expected_a = bodies.getPosition(a);
    const expected_b = bodies.getPosition(b);

    const doomed = try bodies.create(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    });
    bodies.destroy(doomed);

    var lock = world.system.lockMultiRead(&.{ a, doomed, b });

    // The destroyed id's slot reports no body without disturbing its
    // neighbours' -- BodyLockMultiRead::GetBody answers per index, not for
    // the batch as a whole.
    try std.testing.expect(lock.body(1) == null);
    const locked_a = lock.body(0) orelse return error.TestUnexpectedResult;
    const locked_b = lock.body(2) orelse return error.TestUnexpectedResult;

    try expectRVec3Near(expected_a, locked_a.position(), 1e-4);
    try expectRVec3Near(expected_b, locked_b.position(), 1e-4);

    lock.release();

    // The release genuinely dropped the mutex mask: an exclusive lock over
    // the same bodies right after does not wait on a lock this test is still
    // (logically) holding, exactly as the single-body lock's own
    // read-then-write test right above proves for one body at a time.
    var write_lock = world.system.lockMultiWrite(&.{ a, b });
    defer write_lock.release();
    const write_a = write_lock.body(0) orelse return error.TestUnexpectedResult;
    write_a.setLinearVelocity(zjolt.vec3(0, 0, 5));
    // Read back through the SAME lock still held, not bodies.getLinearVelocity
    // -- that takes its own single-body lock, and taking one while this
    // multi-lock still holds body `a` is exactly the reentrant same-priority
    // lock Jolt's own PhysicsLock::sCheckLock refuses in a build with asserts
    // on, real usage error rather than something this test should paper over.
    try expectVec3Near(zjolt.vec3(0, 0, 5), write_a.linearVelocity(), 1e-5);
}

//=============================================================================
// Simulation stats
//=============================================================================

test "SimulationStats is UNSUPPORTED without -Dtrack_simulation_stats, or advances a resting body's step count when it is on" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 2, 0),
        // GatherIslandStats only counts bodies that are in an island, and a
        // sleeping body is in none, so the counts below would be zero once
        // the ball settled.
        .allow_sleeping = false,
    }, .activate);

    try world.stepFor(0.2);

    const first: body_mod.SimulationStats = blk: {
        var lock = world.system.lockRead(ball);
        defer lock.release();
        const body = lock.body() orelse return error.TestUnexpectedResult;
        break :blk body.simulationStats() catch |e| {
            // This build was not compiled with -Dtrack_simulation_stats:
            // MotionProperties::GetSimulationStats does not exist to read,
            // and the entry point says so rather than fabricating a value.
            try std.testing.expectEqual(zjolt.Error.Unsupported, e);
            return;
        };
    };

    // PhysicsSystem::Update calls BodyManager::ResetSimulationStats before
    // anything else, so these counters describe ONE Update and never
    // accumulate across calls: one collision step per Update, one count.
    try std.testing.expectEqual(@as(u8, 1), first.num_collision_steps);

    _ = try world.system.step(1.0 / 60.0, 4, world.jobs);

    const second: body_mod.SimulationStats = blk: {
        var lock = world.system.lockRead(ball);
        defer lock.release();
        const body = lock.body() orelse return error.TestUnexpectedResult;
        break :blk try body.simulationStats();
    };

    // Four collision steps inside one Update, counted once each — and still
    // four, not five, because the previous Update's count was reset.
    try std.testing.expectEqual(@as(u8, 4), second.num_collision_steps);
}

test "validateCachedBounds and validateMotion pass on a freshly added, active body, or read Unsupported without asserts" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 5, 0),
    }, .activate);

    var lock = world.system.lockRead(ball);
    defer lock.release();
    const locked = lock.body() orelse return error.TestUnexpectedResult;

    // A body just added has cached broad-phase bounds matching its shape,
    // and being active skips ValidateMotion's sleeping-velocity check
    // entirely -- both pass rather than assert.
    locked.validateCachedBounds() catch |e| {
        // This build was not compiled with asserts: Body::ValidateCachedBounds
        // does not exist to run, and the entry point says so.
        try std.testing.expectEqual(zjolt.Error.Unsupported, e);
        return;
    };
    try locked.validateMotion();
}

//=============================================================================
// Helpers
//=============================================================================

fn mat44FromQuat(q: zjolt.Quat) !zjolt.Mat44 {
    return zjolt.Mat44.fromRotationTranslation(q, zjolt.vec3_zero);
}

fn expectVec3Near(expected: zjolt.Vec3, actual: zjolt.Vec3, tolerance: f32) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, tolerance);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, tolerance);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, tolerance);
}

// `RVec3`'s components are `f64` in a `-Ddouble_precision` build, so they are
// cast down rather than compared directly -- the same reason `floatY` exists
// in `system_test.zig`.
fn expectRVec3Near(expected: zjolt.RVec3, actual: zjolt.RVec3, tolerance: f32) !void {
    try std.testing.expectApproxEqAbs(@as(f32, @floatCast(expected.x)), @as(f32, @floatCast(actual.x)), tolerance);
    try std.testing.expectApproxEqAbs(@as(f32, @floatCast(expected.y)), @as(f32, @floatCast(actual.y)), tolerance);
    try std.testing.expectApproxEqAbs(@as(f32, @floatCast(expected.z)), @as(f32, @floatCast(actual.z)), tolerance);
}

fn mat44Near(a: zjolt.Mat44, b: zjolt.Mat44, tolerance: f32) bool {
    for (a.m, b.m) |x, y| {
        if (@abs(x - y) > tolerance) return false;
    }
    return true;
}

fn expectMat44Near(expected: zjolt.Mat44, actual: zjolt.Mat44, tolerance: f32) !void {
    for (expected.m, actual.m) |e, a| {
        try std.testing.expectApproxEqAbs(e, a, tolerance);
    }
}

//=============================================================================
// Every descriptor field reaches Jolt
//
// The plant below sets EVERY field of a BodyDesc away from its default and
// the comparison walks the struct's own field list, so a field added later
// and left unwired fails here without anyone remembering to extend the test.
// Blind spot, stated rather than papered over: a field the writer and the
// reader BOTH drop round-trips as whatever was planted in neither, so the
// system-side checks below name the flags Jolt itself has to act on.
//=============================================================================

test "every field of a body descriptor crosses into Jolt and back" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const shape = try zjolt.Shape.initBox(zjolt.vec3(0.5, 0.5, 0.5), .{});
    defer shape.release();

    // Nothing here is a default, and no two numeric fields share a value: a
    // conversion that reads the wrong member shows as a wrong number rather
    // than a plausible one.
    const planted: body_mod.BodyDesc = .{
        .shape = shape,
        .object_layer = Layers.static,
        .position = zjolt.rvec3(11, 12, 13),
        .rotation = zjolt.quat(0, 0.6, 0, 0.8),
        .linear_velocity = zjolt.vec3(21, 22, 23),
        .angular_velocity = zjolt.vec3(31, 32, 33),
        .user_data = 0xfeed_face_dead_beef,
        .motion_type = .dynamic,
        .motion_quality = .linear_cast,
        .allowed_dofs = .{
            .translation_x = true,
            .translation_y = true,
            .translation_z = false,
            .rotation_x = false,
            .rotation_y = false,
            .rotation_z = true,
        },
        .override_mass_properties = .calculate_inertia,
        .mass = 7.5,
        .mass_properties_override = .{
            .mass = 7.5,
            .inertia = .{ 41, 0, 0, 0, 42, 0, 0, 0, 43 },
        },
        .allow_dynamic_or_kinematic = true,
        .is_sensor = true,
        .allow_sleeping = false,
        .enhanced_internal_edge_removal = true,
        .collide_kinematic_vs_non_dynamic = true,
        .use_manifold_reduction = false,
        .apply_gyroscopic_force = true,
        .friction = 0.375,
        .restitution = 0.625,
        .linear_damping = 0.125,
        .angular_damping = 0.25,
        .max_linear_velocity = 111,
        .max_angular_velocity = 222,
        .gravity_factor = 3.5,
        .num_velocity_steps_override = 17,
        .num_position_steps_override = 19,
        .inertia_multiplier = 2.5,
    };

    const scene = try zjolt.Scene.init();
    defer scene.release();
    try std.testing.expectEqual(@as(u32, 0), try scene.addBody(planted));

    const read = try scene.body(0);
    inline for (@typeInfo(body_mod.BodyDesc).@"struct".fields) |field| {
        const name = field.name;
        if (comptime std.mem.eql(u8, name, "shape")) continue;
        if (comptime std.mem.eql(u8, name, "collision_group")) continue;
        if (comptime std.mem.eql(u8, name, "rotation")) continue;
        try std.testing.expectEqualDeep(
            @field(planted, name),
            @field(read, name),
        );
    }
    // Jolt renormalises a rotation on the way in, so this one is the same
    // rotation rather than the same bits.
    inline for (.{ "x", "y", "z", "w" }) |lane| {
        try std.testing.expectApproxEqAbs(
            @field(planted.rotation, lane),
            @field(read.rotation, lane),
            1.0e-6,
        );
    }

    // And the same descriptor through the OTHER writer -- a live body, not a
    // scene entry -- read back through the accessors Jolt answers from its
    // own state rather than from a stored copy of the desc.
    var world = try World.init();
    defer world.deinit();
    const bodies = world.system.bodies();
    const body = try bodies.createAndAdd(planted, .dont_activate);

    try std.testing.expect(bodies.getApplyGyroscopicForce(body));
    try std.testing.expect(bodies.getCollideKinematicVsNonDynamic(body));
    try std.testing.expect(bodies.getEnhancedInternalEdgeRemoval(body));
    try std.testing.expect(!bodies.getUseManifoldReduction(body));
    try std.testing.expectEqual(
        @as(u32, 17),
        bodies.getNumVelocityStepsOverride(body),
    );
    try std.testing.expectEqual(
        @as(u32, 19),
        bodies.getNumPositionStepsOverride(body),
    );
    // `mass` reached Jolt's own mass properties, not just the stored desc.
    // Approximate because it is read back as its reciprocal.
    try std.testing.expectApproxEqRel(
        @as(f32, 7.5),
        1.0 / bodies.getInverseMassUnchecked(body),
        1.0e-6,
    );
}

test "the three Body-flag setters change the flag they name and nothing else" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const bodies = world.system.bodies();
    const ball = try bodies.createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 4, 0),
    }, .activate);

    // All three start at Jolt's construction-time default.
    try std.testing.expect(!bodies.getApplyGyroscopicForce(ball));
    try std.testing.expect(!bodies.getCollideKinematicVsNonDynamic(ball));
    try std.testing.expect(!bodies.getEnhancedInternalEdgeRemoval(ball));

    bodies.setApplyGyroscopicForce(ball, true);
    try std.testing.expect(bodies.getApplyGyroscopicForce(ball));
    try std.testing.expect(!bodies.getCollideKinematicVsNonDynamic(ball));
    try std.testing.expect(!bodies.getEnhancedInternalEdgeRemoval(ball));

    bodies.setCollideKinematicVsNonDynamic(ball, true);
    bodies.setEnhancedInternalEdgeRemoval(ball, true);
    try std.testing.expect(bodies.getCollideKinematicVsNonDynamic(ball));
    try std.testing.expect(bodies.getEnhancedInternalEdgeRemoval(ball));

    // Setting one back leaves the others where they were.
    bodies.setApplyGyroscopicForce(ball, false);
    try std.testing.expect(!bodies.getApplyGyroscopicForce(ball));
    try std.testing.expect(bodies.getCollideKinematicVsNonDynamic(ball));
    try std.testing.expect(bodies.getEnhancedInternalEdgeRemoval(ball));

    // A stale id is not a crash, and the step overrides refuse Jolt's own
    // 256 bound rather than truncating into a uint8.
    bodies.setApplyGyroscopicForce(zjolt.invalid_body_id, true);
    try std.testing.expectError(
        error.InvalidArgument,
        bodies.setNumVelocityStepsOverride(ball, 256),
    );
    try std.testing.expectError(
        error.BodyNotFound,
        bodies.setNumPositionStepsOverride(zjolt.invalid_body_id, 4),
    );
    try std.testing.expectEqual(
        @as(u32, 0),
        bodies.getNumVelocityStepsOverride(ball),
    );
    try bodies.setNumVelocityStepsOverride(ball, 255);
    try std.testing.expectEqual(
        @as(u32, 255),
        bodies.getNumVelocityStepsOverride(ball),
    );
}
