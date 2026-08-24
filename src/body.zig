//! Bodies: creation, per-body access, and locks.
//!
//! A body is named by a `BodyId`, not by a pointer. That is Jolt's model and
//! it is the right one: ids stay valid to hold across a step, are four bytes
//! in an array of ten thousand, and carry a generation counter so a stale one
//! is detected rather than aliasing whatever body was created next.
//!
//! Operations live on `BodyInterface`, obtained from `PhysicsSystem.bodies()`,
//! mirroring Jolt's own `BodyInterface`. Each call takes a body lock, which is
//! right for the occasional query and wrong for reading every body every
//! frame — see `PhysicsSystem.getTransforms` for that.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const shape_mod = @import("shape.zig");
const group_mod = @import("group.zig");

pub const BodyId = c.BodyId;
pub const invalid_body_id = c.body_id_invalid;

pub const MotionType = c.MotionType;
pub const MotionQuality = c.MotionQuality;
pub const BodyType = c.BodyType;
pub const Activation = c.Activation;
pub const AllowedDofs = c.AllowedDofs;
pub const OverrideMassProperties = c.OverrideMassProperties;
pub const ObjectLayer = c.ObjectLayer;

/// How a body is born.
///
/// The defaults are Jolt's own, read out of a default-constructed
/// `BodyCreationSettings` at start-up rather than transcribed — see
/// `BodyDesc.default`. `shape` and `object_layer` are the two a caller must
/// always think about.
pub const BodyDesc = struct {
    shape: shape_mod.Shape,
    object_layer: ObjectLayer,

    position: math.RVec3 = math.rvec3_zero,
    rotation: math.Quat = math.quat_identity,
    linear_velocity: math.Vec3 = math.vec3_zero,
    angular_velocity: math.Vec3 = math.vec3_zero,
    user_data: u64 = 0,

    /// Exceptions to layer-based collision, for bodies of the same kind that
    /// must not fight each other — the limbs of one ragdoll, the wheels of
    /// one vehicle. The default makes none. The body takes its own reference
    /// on `collision_group.filter`, exactly as it does on `shape`.
    collision_group: group_mod.CollisionGroup = .{},

    motion_type: MotionType = .dynamic,
    motion_quality: MotionQuality = .discrete,
    allowed_dofs: AllowedDofs = .all,

    /// `.calculate_inertia` uses `mass` below and scales the shape's inertia
    /// to match; the default computes both from the shape and its density.
    override_mass_properties: OverrideMassProperties = .calculate_mass_and_inertia,
    mass: f32 = 0,

    /// Lets a static body be switched to kinematic or dynamic later.
    allow_dynamic_or_kinematic: bool = false,
    /// Reports contacts without responding to them: a trigger volume.
    is_sensor: bool = false,
    allow_sleeping: bool = true,
    /// Extra work to suppress ghost collisions on internal mesh edges.
    enhanced_internal_edge_removal: bool = false,

    friction: f32 = 0.2,
    restitution: f32 = 0,
    linear_damping: f32 = 0.05,
    angular_damping: f32 = 0.05,
    max_linear_velocity: f32 = 500,
    max_angular_velocity: f32 = 0.25 * std.math.pi * 60.0,
    gravity_factor: f32 = 1,

    fn toC(self: BodyDesc) c.BodyDesc {
        var out: c.BodyDesc = undefined;
        // Start from Jolt's defaults so a field this wrapper does not model
        // still gets a sensible value rather than whatever was on the stack.
        c.zjoltBodyDescInit(&out);
        out.shape = self.shape.handle;
        out.collision_group = group_mod.toC(self.collision_group);
        out.object_layer = self.object_layer;
        out.position = self.position;
        out.rotation = self.rotation;
        out.linear_velocity = self.linear_velocity;
        out.angular_velocity = self.angular_velocity;
        out.user_data = self.user_data;
        out.motion_type = self.motion_type;
        out.motion_quality = self.motion_quality;
        out.allowed_dofs = self.allowed_dofs;
        out.override_mass_properties = self.override_mass_properties;
        out.mass = self.mass;
        out.allow_dynamic_or_kinematic = self.allow_dynamic_or_kinematic;
        out.is_sensor = self.is_sensor;
        out.allow_sleeping = self.allow_sleeping;
        out.enhanced_internal_edge_removal = self.enhanced_internal_edge_removal;
        out.friction = self.friction;
        out.restitution = self.restitution;
        out.linear_damping = self.linear_damping;
        out.angular_damping = self.angular_damping;
        out.max_linear_velocity = self.max_linear_velocity;
        out.max_angular_velocity = self.max_angular_velocity;
        out.gravity_factor = self.gravity_factor;
        return out;
    }
};

/// The transform half of what a body is, for the common "put it here" call.
pub const Transform = struct {
    position: math.RVec3,
    rotation: math.Quat,
};

//=============================================================================
// Body interface
//=============================================================================

pub const BodyInterface = struct {
    handle: *c.PhysicsSystem,

    //-------------------------------------------------------------------------
    // Lifetime
    //-------------------------------------------------------------------------

    /// Creates a body without adding it to the simulation. Useful for
    /// preparing a batch and adding it in one go.
    pub fn create(self: BodyInterface, desc: BodyDesc) err.Error!BodyId {
        const c_desc = desc.toC();
        var id: BodyId = invalid_body_id;
        try err.check(c.zjoltBodyCreate(self.handle, &c_desc, &id));
        return id;
    }

    pub fn createAndAdd(
        self: BodyInterface,
        desc: BodyDesc,
        activation: Activation,
    ) err.Error!BodyId {
        const c_desc = desc.toC();
        var id: BodyId = invalid_body_id;
        try err.check(c.zjoltBodyCreateAndAdd(self.handle, &c_desc, activation, &id));
        return id;
    }

    /// Removes the body if it is still added, then destroys it. The id becomes
    /// stale; later calls with it are ignored rather than reaching whatever
    /// body was created next.
    pub fn destroy(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyDestroy(self.handle, body);
    }

    pub fn add(self: BodyInterface, body: BodyId, activation: Activation) void {
        c.zjoltBodyAdd(self.handle, body, activation);
    }

    pub fn remove(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyRemove(self.handle, body);
    }

    pub fn isAdded(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyIsAdded(self.handle, body);
    }

    pub fn isActive(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyIsActive(self.handle, body);
    }

    pub fn activate(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyActivate(self.handle, body);
    }

    pub fn deactivate(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyDeactivate(self.handle, body);
    }

    /// Restarts the clock `time_before_sleep` counts down before an active
    /// body is allowed to sleep.
    pub fn resetSleepTimer(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyResetSleepTimer(self.handle, body);
    }

    /// Which kind of body this is: rigid, or a soft body made through
    /// `SoftBody.init`. Worth asking before reaching for anything in
    /// `softbody.zig` with an id that came from a query or a contact.
    pub fn getBodyType(self: BodyInterface, body: BodyId) BodyType {
        return c.zjoltBodyGetBodyType(self.handle, body);
    }

    //-------------------------------------------------------------------------
    // Transform
    //-------------------------------------------------------------------------

    /// Teleport: places the body immediately, ignoring whatever it passes
    /// through on the way. For a body that should push things aside, use
    /// `moveKinematic`.
    pub fn setPositionAndRotation(
        self: BodyInterface,
        body: BodyId,
        position: math.RVec3,
        rotation: ?math.Quat,
        activation: Activation,
    ) void {
        if (rotation) |r| {
            c.zjoltBodySetPositionAndRotation(self.handle, body, &position, &r, activation);
        } else {
            c.zjoltBodySetPositionAndRotation(self.handle, body, &position, null, activation);
        }
    }

    pub fn getTransform(self: BodyInterface, body: BodyId) Transform {
        var position: math.RVec3 = math.rvec3_zero;
        var rotation: math.Quat = math.quat_identity;
        c.zjoltBodyGetPositionAndRotation(self.handle, body, &position, &rotation);
        return .{ .position = position, .rotation = rotation };
    }

    pub fn getPosition(self: BodyInterface, body: BodyId) math.RVec3 {
        var position: math.RVec3 = math.rvec3_zero;
        c.zjoltBodyGetPositionAndRotation(self.handle, body, &position, null);
        return position;
    }

    pub fn getRotation(self: BodyInterface, body: BodyId) math.Quat {
        var rotation: math.Quat = math.quat_identity;
        c.zjoltBodyGetPositionAndRotation(self.handle, body, null, &rotation);
        return rotation;
    }

    pub fn getCenterOfMassPosition(self: BodyInterface, body: BodyId) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltBodyGetCenterOfMassPosition(self.handle, body, &out);
        return out;
    }

    /// The rotation and translation that place the SHAPE's origin, as a
    /// column-major matrix with the translation in the fourth column.
    ///
    /// Identity when the body lock fails, which is Jolt's own answer rather
    /// than one this wrapper invented — so a stale id and a body sitting
    /// unrotated at the origin read the same. Ask `isAdded` if that matters.
    pub fn getWorldTransform(self: BodyInterface, body: BodyId) math.RMat44 {
        var out: math.RMat44 = math.rmat44_identity;
        c.zjoltBodyGetWorldTransform(self.handle, body, &out);
        return out;
    }

    /// As `getWorldTransform`, but placing the body's CENTRE OF MASS — the
    /// space Jolt simulates in. The two differ by the shape's centre-of-mass
    /// offset, which is not zero for a capsule, a compound, or anything
    /// wrapped in an offset-centre-of-mass shape. Identity on a failed lock,
    /// same as above.
    pub fn getCenterOfMassTransform(self: BodyInterface, body: BodyId) math.RMat44 {
        var out: math.RMat44 = math.rmat44_identity;
        c.zjoltBodyGetCenterOfMassTransform(self.handle, body, &out);
        return out;
    }

    /// The inverse inertia tensor rotated into world space, as a 3x3 matrix
    /// padded out to 4x4 the way Jolt stores one.
    ///
    /// Only a dynamic body has one. A static or kinematic body is
    /// `error.InvalidArgument` rather than a default, because Jolt's own
    /// accessor asserts on it instead of returning anything; a stale id is
    /// `error.BodyNotFound`.
    pub fn getInverseInertia(self: BodyInterface, body: BodyId) err.Error!math.Mat44 {
        var out: math.Mat44 = math.mat44_identity;
        try err.check(c.zjoltBodyGetInverseInertia(self.handle, body, &out));
        return out;
    }

    /// As `setPositionAndRotation`, but skips the broad-phase update and the
    /// activation check when the new pose is close enough to the old one —
    /// worth using for a snapshot applied every tick, where "did this
    /// actually move" is not already known at the call site.
    pub fn setPositionAndRotationWhenChanged(
        self: BodyInterface,
        body: BodyId,
        position: math.RVec3,
        rotation: ?math.Quat,
        activation: Activation,
    ) void {
        if (rotation) |r| {
            c.zjoltBodySetPositionAndRotationWhenChanged(self.handle, body, &position, &r, activation);
        } else {
            c.zjoltBodySetPositionAndRotationWhenChanged(self.handle, body, &position, null, activation);
        }
    }

    /// Drives a kinematic body toward a target over `delta_time`, so it pushes
    /// dynamic bodies out of the way instead of teleporting through them.
    /// This is how a moving platform or an animated character should move.
    pub fn moveKinematic(
        self: BodyInterface,
        body: BodyId,
        target_position: math.RVec3,
        target_rotation: ?math.Quat,
        delta_time: f32,
    ) void {
        if (target_rotation) |r| {
            c.zjoltBodyMoveKinematic(self.handle, body, &target_position, &r, delta_time);
        } else {
            c.zjoltBodyMoveKinematic(self.handle, body, &target_position, null, delta_time);
        }
    }

    /// Sets position, rotation and both velocities in one lock — cheaper than
    /// the equivalent separate calls when restoring a full motion state.
    pub fn setPositionRotationAndVelocity(
        self: BodyInterface,
        body: BodyId,
        position: math.RVec3,
        rotation: ?math.Quat,
        linear_velocity: math.Vec3,
        angular_velocity: math.Vec3,
    ) void {
        if (rotation) |r| {
            c.zjoltBodySetPositionRotationAndVelocity(self.handle, body, &position, &r, &linear_velocity, &angular_velocity);
        } else {
            c.zjoltBodySetPositionRotationAndVelocity(self.handle, body, &position, null, &linear_velocity, &angular_velocity);
        }
    }

    //-------------------------------------------------------------------------
    // Velocity and forces
    //-------------------------------------------------------------------------

    pub fn setLinearVelocity(self: BodyInterface, body: BodyId, velocity: math.Vec3) void {
        c.zjoltBodySetLinearVelocity(self.handle, body, &velocity);
    }

    pub fn getLinearVelocity(self: BodyInterface, body: BodyId) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyGetLinearVelocity(self.handle, body, &out);
        return out;
    }

    pub fn setAngularVelocity(self: BodyInterface, body: BodyId, velocity: math.Vec3) void {
        c.zjoltBodySetAngularVelocity(self.handle, body, &velocity);
    }

    pub fn getAngularVelocity(self: BodyInterface, body: BodyId) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyGetAngularVelocity(self.handle, body, &out);
        return out;
    }

    /// Sets both in one lock rather than two.
    pub fn setLinearAndAngularVelocity(
        self: BodyInterface,
        body: BodyId,
        linear_velocity: math.Vec3,
        angular_velocity: math.Vec3,
    ) void {
        c.zjoltBodySetLinearAndAngularVelocity(self.handle, body, &linear_velocity, &angular_velocity);
    }

    pub const Velocity = struct {
        linear: math.Vec3,
        angular: math.Vec3,
    };

    /// Reads both in one lock rather than two.
    pub fn getLinearAndAngularVelocity(self: BodyInterface, body: BodyId) Velocity {
        var linear: math.Vec3 = math.vec3_zero;
        var angular: math.Vec3 = math.vec3_zero;
        c.zjoltBodyGetLinearAndAngularVelocity(self.handle, body, &linear, &angular);
        return .{ .linear = linear, .angular = angular };
    }

    /// Adds to the current linear velocity, clamped the same way
    /// `setLinearVelocity` is.
    pub fn addLinearVelocity(self: BodyInterface, body: BodyId, linear_velocity: math.Vec3) void {
        c.zjoltBodyAddLinearVelocity(self.handle, body, &linear_velocity);
    }

    pub fn addLinearAndAngularVelocity(
        self: BodyInterface,
        body: BodyId,
        linear_velocity: math.Vec3,
        angular_velocity: math.Vec3,
    ) void {
        c.zjoltBodyAddLinearAndAngularVelocity(self.handle, body, &linear_velocity, &angular_velocity);
    }

    /// Velocity of the point on this body currently at `point` (world space),
    /// including the contribution from spin. Zero for a static body and zero
    /// when the body lock fails — the same answer, so `isAdded` is the way to
    /// tell them apart if that matters.
    pub fn getPointVelocity(self: BodyInterface, body: BodyId, point: math.RVec3) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyGetPointVelocity(self.handle, body, &point, &out);
        return out;
    }

    /// Accumulates until the next step consumes it.
    pub fn addForce(self: BodyInterface, body: BodyId, force: math.Vec3) void {
        c.zjoltBodyAddForce(self.handle, body, &force);
    }

    pub fn addForceAtPoint(
        self: BodyInterface,
        body: BodyId,
        force: math.Vec3,
        point: math.RVec3,
    ) void {
        c.zjoltBodyAddForceAtPoint(self.handle, body, &force, &point);
    }

    pub fn addTorque(self: BodyInterface, body: BodyId, torque: math.Vec3) void {
        c.zjoltBodyAddTorque(self.handle, body, &torque);
    }

    /// Changes velocity immediately, unlike a force.
    pub fn addImpulse(self: BodyInterface, body: BodyId, impulse: math.Vec3) void {
        c.zjoltBodyAddImpulse(self.handle, body, &impulse);
    }

    pub fn addImpulseAtPoint(
        self: BodyInterface,
        body: BodyId,
        impulse: math.Vec3,
        point: math.RVec3,
    ) void {
        c.zjoltBodyAddImpulseAtPoint(self.handle, body, &impulse, &point);
    }

    pub fn addAngularImpulse(self: BodyInterface, body: BodyId, impulse: math.Vec3) void {
        c.zjoltBodyAddAngularImpulse(self.handle, body, &impulse);
    }

    /// Applies drag and buoyancy for a body partly submerged at
    /// `surface_position`/`surface_normal` (surface plane in world space,
    /// normal pointing out of the fluid), activating it on success.
    ///
    /// False, and does nothing, for a body that is not dynamic or whose lock
    /// fails — the same false Jolt returns.
    pub fn applyBuoyancyImpulse(
        self: BodyInterface,
        body: BodyId,
        surface_position: math.RVec3,
        surface_normal: math.Vec3,
        buoyancy: f32,
        linear_drag: f32,
        angular_drag: f32,
        fluid_velocity: math.Vec3,
        gravity: math.Vec3,
        delta_time: f32,
    ) bool {
        return c.zjoltBodyApplyBuoyancyImpulse(
            self.handle,
            body,
            &surface_position,
            &surface_normal,
            buoyancy,
            linear_drag,
            angular_drag,
            &fluid_velocity,
            &gravity,
            delta_time,
        );
    }

    //-------------------------------------------------------------------------
    // Properties
    //-------------------------------------------------------------------------

    pub fn setMotionType(
        self: BodyInterface,
        body: BodyId,
        motion_type: MotionType,
        activation: Activation,
    ) void {
        c.zjoltBodySetMotionType(self.handle, body, motion_type, activation);
    }

    pub fn getMotionType(self: BodyInterface, body: BodyId) MotionType {
        return c.zjoltBodyGetMotionType(self.handle, body);
    }

    /// How well this body detects collisions when it moves fast.
    pub fn setMotionQuality(self: BodyInterface, body: BodyId, quality: MotionQuality) void {
        c.zjoltBodySetMotionQuality(self.handle, body, quality);
    }

    pub fn getMotionQuality(self: BodyInterface, body: BodyId) MotionQuality {
        return c.zjoltBodyGetMotionQuality(self.handle, body);
    }

    /// Replaces the shape. With `update_mass_properties` the body's mass and
    /// inertia are recomputed from the new shape.
    pub fn setShape(
        self: BodyInterface,
        body: BodyId,
        shape: shape_mod.Shape,
        update_mass_properties: bool,
        activation: Activation,
    ) void {
        c.zjoltBodySetShape(self.handle, body, shape.handle, update_mass_properties, activation);
    }

    pub fn setObjectLayer(self: BodyInterface, body: BodyId, layer: ObjectLayer) void {
        c.zjoltBodySetObjectLayer(self.handle, body, layer);
    }

    pub fn getObjectLayer(self: BodyInterface, body: BodyId) ObjectLayer {
        return c.zjoltBodyGetObjectLayer(self.handle, body);
    }

    pub fn setUserData(self: BodyInterface, body: BodyId, user_data: u64) void {
        c.zjoltBodySetUserData(self.handle, body, user_data);
    }

    pub fn getUserData(self: BodyInterface, body: BodyId) u64 {
        return c.zjoltBodyGetUserData(self.handle, body);
    }

    pub fn setFriction(self: BodyInterface, body: BodyId, friction: f32) void {
        c.zjoltBodySetFriction(self.handle, body, friction);
    }

    pub fn getFriction(self: BodyInterface, body: BodyId) f32 {
        return c.zjoltBodyGetFriction(self.handle, body);
    }

    pub fn setRestitution(self: BodyInterface, body: BodyId, restitution: f32) void {
        c.zjoltBodySetRestitution(self.handle, body, restitution);
    }

    pub fn getRestitution(self: BodyInterface, body: BodyId) f32 {
        return c.zjoltBodyGetRestitution(self.handle, body);
    }

    pub fn setGravityFactor(self: BodyInterface, body: BodyId, factor: f32) void {
        c.zjoltBodySetGravityFactor(self.handle, body, factor);
    }

    pub fn getGravityFactor(self: BodyInterface, body: BodyId) f32 {
        return c.zjoltBodyGetGravityFactor(self.handle, body);
    }

    /// `velocity` must not be negative — Jolt asserts that in a build with
    /// asserts enabled and reads it as-is otherwise. A static or kinematic
    /// body has no motion properties to hold this, so it is a no-op on one.
    pub fn setMaxLinearVelocity(self: BodyInterface, body: BodyId, velocity: f32) void {
        c.zjoltBodySetMaxLinearVelocity(self.handle, body, velocity);
    }

    /// Jolt's own construction-time default (500) when the body lock fails.
    pub fn getMaxLinearVelocity(self: BodyInterface, body: BodyId) f32 {
        return c.zjoltBodyGetMaxLinearVelocity(self.handle, body);
    }

    pub fn setMaxAngularVelocity(self: BodyInterface, body: BodyId, velocity: f32) void {
        c.zjoltBodySetMaxAngularVelocity(self.handle, body, velocity);
    }

    /// Jolt's own construction-time default (15*pi) when the body lock fails.
    pub fn getMaxAngularVelocity(self: BodyInterface, body: BodyId) f32 {
        return c.zjoltBodyGetMaxAngularVelocity(self.handle, body);
    }

    /// Merging nearby contact manifolds into one, on by default. Turning it
    /// off invalidates this body's contact cache, so a pair already resting
    /// picks up the change on the next step.
    pub fn setUseManifoldReduction(self: BodyInterface, body: BodyId, use_reduction: bool) void {
        c.zjoltBodySetUseManifoldReduction(self.handle, body, use_reduction);
    }

    pub fn getUseManifoldReduction(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyGetUseManifoldReduction(self.handle, body);
    }

    /// Whether this body reports contacts without responding to them — a
    /// trigger volume. Unlocked counterpart of `Body.isSensor`.
    pub fn setIsSensor(self: BodyInterface, body: BodyId, is_sensor: bool) void {
        c.zjoltBodySetIsSensor(self.handle, body, is_sensor);
    }

    pub fn isSensor(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyIsSensor(self.handle, body);
    }

    /// The material of one leaf of the body's shape. @see `Shape.material`
    /// for what a material is and for the sub-shape id rules.
    ///
    /// **This is not a way to test whether a body exists.** Jolt takes a body
    /// lock and answers with the shared default material when it fails, so a
    /// destroyed body and a body whose shape has no materials of its own read
    /// identically. That answer is forwarded rather than replaced with an
    /// invented null, because reporting a failure Jolt did not report would be
    /// a different contract, not a stricter one. Use `isAdded` when the
    /// question is really about the body.
    pub fn getMaterial(
        self: BodyInterface,
        body: BodyId,
        sub_shape_id: c.SubShapeId,
    ) ?shape_mod.PhysicsMaterial {
        const handle = c.zjoltBodyGetMaterial(self.handle, body, sub_shape_id) orelse
            return null;
        return .{ .handle = handle };
    }

    /// Tells the system a body's shape changed underneath it — which is what
    /// the `MutableCompound` methods do.
    ///
    /// `previous_center_of_mass` is the shape's centre of mass BEFORE the
    /// change; the body is moved so its geometry stays where it was. Without
    /// this call the broad phase and the contact cache go on describing the
    /// shape as it used to be.
    pub fn notifyShapeChanged(
        self: BodyInterface,
        body: BodyId,
        previous_center_of_mass: math.Vec3,
        update_mass_properties: bool,
        activation: Activation,
    ) void {
        c.zjoltBodyNotifyShapeChanged(
            self.handle,
            body,
            &previous_center_of_mass,
            update_mass_properties,
            activation,
        );
    }

    //-------------------------------------------------------------------------
    // Collision groups
    //
    // Exceptions between individual bodies, where object layers can only draw
    // a line between kinds of body. See `group.zig`.
    //-------------------------------------------------------------------------

    /// The body takes its own reference on `group.filter` and drops its
    /// previous one.
    ///
    /// Changing this on a body already resting on another does not take effect
    /// until the cached pair is dropped — see `invalidateContactCache`.
    pub fn setCollisionGroup(
        self: BodyInterface,
        body: BodyId,
        group: group_mod.CollisionGroup,
    ) void {
        const raw = group_mod.toC(group);
        c.zjoltBodySetCollisionGroup(self.handle, body, &raw);
    }

    /// A stale body id is NOT distinguishable from a body with no group: Jolt
    /// returns its "invalid" group when the body lock fails, which is exactly
    /// what a group-less body carries. Ask `isAdded` first if that matters.
    ///
    /// The filter that comes back is borrowed and holds no reference of its
    /// own. Call `addRef` on it if you mean to keep it.
    pub fn getCollisionGroup(
        self: BodyInterface,
        body: BodyId,
    ) group_mod.CollisionGroup {
        var raw: c.CollisionGroup = undefined;
        c.zjoltBodyGetCollisionGroup(self.handle, body, &raw);
        return group_mod.fromC(raw);
    }

    /// Drops the cached collision result for every pair involving this body,
    /// so the next step re-evaluates them.
    ///
    /// This is what makes a collision-group change take effect on a pair that
    /// is already resting. `PhysicsSettings.use_body_pair_contact_cache` is on
    /// by default, and it skips the narrow phase for a pair whose relative
    /// transform has not moved — which a group change does not.
    pub fn invalidateContactCache(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyInvalidateContactCache(self.handle, body);
    }
};

//=============================================================================
// Locks
//
// The accessors above take a lock per call. A lock holds one body still for a
// scope and hands out a borrowed `Body` — right when reading several of its
// properties at once, and the only way to reach the mutators that bypass
// activation.
//
// Jolt's locks are RAII; these are explicit, because a scope is not something
// a C ABI can express. `defer lock.release()` restores the shape.
//=============================================================================

/// A borrowed view of a locked body. Must not outlive its lock.
pub const Body = struct {
    handle: *c.Body,

    pub fn id(self: Body) BodyId {
        return c.zjoltBodyGetId(self.handle);
    }

    pub fn position(self: Body) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltBodyGetPosition(self.handle, &out);
        return out;
    }

    pub fn rotation(self: Body) math.Quat {
        var out: math.Quat = math.quat_identity;
        c.zjoltBodyGetRotation(self.handle, &out);
        return out;
    }

    pub fn centerOfMassPosition(self: Body) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltBodyGetCenterOfMassPositionLocked(self.handle, &out);
        return out;
    }

    pub fn linearVelocity(self: Body) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyGetLinearVelocityLocked(self.handle, &out);
        return out;
    }

    pub fn angularVelocity(self: Body) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyGetAngularVelocityLocked(self.handle, &out);
        return out;
    }

    pub fn userData(self: Body) u64 {
        return c.zjoltBodyGetUserDataLocked(self.handle);
    }

    pub fn objectLayer(self: Body) ObjectLayer {
        return c.zjoltBodyGetObjectLayerLocked(self.handle);
    }

    pub fn motionType(self: Body) MotionType {
        return c.zjoltBodyGetMotionTypeLocked(self.handle);
    }

    pub fn isActive(self: Body) bool {
        return c.zjoltBodyIsActiveLocked(self.handle);
    }

    pub fn isSensor(self: Body) bool {
        return c.zjoltBodyIsSensorLocked(self.handle);
    }

    /// Borrowed; takes no reference. Valid while the body is alive.
    pub fn shape(self: Body) ?shape_mod.Shape {
        return .{ .handle = c.zjoltBodyGetShapeLocked(self.handle) orelse return null };
    }

    pub fn worldBounds(self: Body) math.AABox {
        var out: math.AABox = undefined;
        c.zjoltBodyGetWorldBounds(self.handle, &out);
        return out;
    }

    //-------------------------------------------------------------------------
    // Mutators — require a write lock
    //
    // These bypass activation, which is the point: they are for adjusting a
    // body you are already holding still. A sleeping body stays asleep, so
    // activate it separately if it should wake.
    //-------------------------------------------------------------------------

    pub fn setLinearVelocity(self: Body, velocity: math.Vec3) void {
        c.zjoltBodyMutSetLinearVelocity(self.handle, &velocity);
    }

    pub fn setAngularVelocity(self: Body, velocity: math.Vec3) void {
        c.zjoltBodyMutSetAngularVelocity(self.handle, &velocity);
    }

    pub fn setUserData(self: Body, user_data: u64) void {
        c.zjoltBodyMutSetUserData(self.handle, user_data);
    }

    pub fn setFriction(self: Body, friction: f32) void {
        c.zjoltBodyMutSetFriction(self.handle, friction);
    }

    pub fn setRestitution(self: Body, restitution: f32) void {
        c.zjoltBodyMutSetRestitution(self.handle, restitution);
    }

    pub fn addImpulse(self: Body, impulse: math.Vec3) void {
        c.zjoltBodyMutAddImpulse(self.handle, &impulse);
    }
};

/// A held body lock. Release it exactly once, whether or not `body` was null.
pub const Lock = struct {
    raw: c.BodyLock,
    write: bool,

    /// Null when the id did not name a live body — which is normal, not an
    /// error: a body can be destroyed between two frames.
    pub fn body(self: *const Lock) ?Body {
        return .{ .handle = self.raw.body orelse return null };
    }

    pub fn release(self: *Lock) void {
        if (self.write) {
            c.zjoltBodyLockWriteRelease(&self.raw);
        } else {
            c.zjoltBodyLockReadRelease(&self.raw);
        }
    }
};
