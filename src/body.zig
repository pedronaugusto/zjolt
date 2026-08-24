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

pub const BodyId = c.BodyId;
pub const invalid_body_id = c.body_id_invalid;

pub const MotionType = c.MotionType;
pub const MotionQuality = c.MotionQuality;
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
