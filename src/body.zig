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
const c = @import("c/body.zig");
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
pub const MassProperties = c.MassProperties;
pub const SimulationStats = c.SimulationStats;

/// The 8-bit generation counter packed into `id`'s top byte — two ids that
/// share an index (one recycled from the other) never share this. Pure bit
/// extraction; unlike everything else in this file it needs no
/// `BodyInterface` and answers the same whether `id` still names a live body.
pub fn getSequenceNumber(id: BodyId) u8 {
    return c.zjoltBodyIdGetSequenceNumber(id);
}

/// Whether a body wearing `shape` may only ever be STATIC — true for a height
/// field, a plane, a mesh, and any compound or decorated shape with one of
/// those inside it.
pub fn shapeMustBeStatic(shape: shape_mod.Shape) bool {
    return c.zjoltShapeMustBeStatic(shape.handle);
}

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
    /// to match; `.mass_and_inertia_provided` uses `mass_properties_override`
    /// outright; the default computes both from the shape and its density.
    override_mass_properties: OverrideMassProperties = .calculate_mass_and_inertia,
    mass: f32 = 0,
    /// Used when override_mass_properties is `.mass_and_inertia_provided`.
    /// `inertia` is a direct (not inverse) tensor, decomposed into principal
    /// axes at construction — a non-diagonal one round-trips to an
    /// equivalent tensor rather than byte-for-byte.
    mass_properties_override: MassProperties = .{ .mass = 0, .inertia = @splat(0) },

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
        out.mass_properties_override = self.mass_properties_override;
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

/// Used by `scene.zig`, which builds the same descriptor into a scene rather
/// than straight into a system; here rather than there because the conversion
/// belongs with the type it converts.
pub fn descToC(desc: BodyDesc) c.BodyDesc {
    return desc.toC();
}

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

    /// As `create`, but the body takes `id` instead of the next one Jolt
    /// would assign — for lockstep networking, where every peer needs
    /// the identical id. `id` must not be `invalid_body_id`, name a live
    /// body, or set bit 31 (Jolt reserves it for the broad phase); an id
    /// round-tripped from a body this binding created always satisfies
    /// all three — `error.InvalidArgument` otherwise.
    pub fn createWithId(self: BodyInterface, desc: BodyDesc, id: BodyId) err.Error!BodyId {
        const c_desc = desc.toC();
        var out: BodyId = invalid_body_id;
        try err.check(c.zjoltBodyCreateWithId(self.handle, &c_desc, id, &out));
        return out;
    }

    /// As `createWithId`, followed immediately by `add`.
    pub fn createAndAddWithId(
        self: BodyInterface,
        desc: BodyDesc,
        id: BodyId,
        activation: Activation,
    ) err.Error!BodyId {
        const c_desc = desc.toC();
        var out: BodyId = invalid_body_id;
        try err.check(c.zjoltBodyCreateAndAddWithId(self.handle, &c_desc, id, activation, &out));
        return out;
    }

    /// Overwrites `body`'s state from `desc`, as though it had just been
    /// created with it. `body` must not currently be added to this system.
    ///
    /// `error.InvalidArgument` if `body` is added, or if `desc` implies
    /// motion properties this body was created without (STATIC with
    /// `allow_dynamic_or_kinematic = false` cannot gain any here).
    pub fn applyBodyCreationSettings(self: BodyInterface, body: BodyId, desc: BodyDesc) err.Error!void {
        const c_desc = desc.toC();
        try err.check(c.zjoltBodyApplyBodyCreationSettings(self.handle, body, &c_desc));
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

    /// The inverse inertia tensor rotated into world space, as a 3x3
    /// matrix padded out to 4x4 the way Jolt stores one.
    ///
    /// Only a dynamic body has one: `error.InvalidArgument` for a static
    /// or kinematic body (Jolt's own accessor asserts rather than
    /// returning a default); `error.BodyNotFound` for a stale id.
    pub fn getInverseInertia(self: BodyInterface, body: BodyId) err.Error!math.Mat44 {
        var out: math.Mat44 = math.mat44_identity;
        try err.check(c.zjoltBodyGetInverseInertia(self.handle, body, &out));
        return out;
    }

    /// As `getInverseInertia`, but in LOCAL (body) space rather than rotated
    /// into world space.
    pub fn getLocalSpaceInverseInertia(self: BodyInterface, body: BodyId) err.Error!math.Mat44 {
        var out: math.Mat44 = math.mat44_identity;
        try err.check(c.zjoltBodyGetLocalSpaceInverseInertia(self.handle, body, &out));
        return out;
    }

    /// As `getInverseInertia`, but for the hypothetical orientation
    /// `rotation` instead of the body's actual current one. Only the
    /// rotation part of `rotation` is used; translation is ignored.
    pub fn getInverseInertiaForRotation(
        self: BodyInterface,
        body: BodyId,
        rotation: math.Mat44,
    ) err.Error!math.Mat44 {
        var out: math.Mat44 = math.mat44_identity;
        try err.check(c.zjoltBodyGetInverseInertiaForRotation(self.handle, body, &rotation, &out));
        return out;
    }

    /// Inverse mass (1/kg), without the dynamic check `getInverseInertia`
    /// makes. Meaningful on a currently-kinematic body too, if it was
    /// created with `allow_dynamic_or_kinematic` and therefore has a real
    /// mass ready for the moment it becomes dynamic. 0 on a static body,
    /// which has no motion properties to hold this.
    pub fn getInverseMassUnchecked(self: BodyInterface, body: BodyId) f32 {
        return c.zjoltBodyGetInverseMassUnchecked(self.handle, body);
    }

    /// Which axes this body is allowed to move along — the same mask
    /// `BodyDesc.allowed_dofs` sets at creation. `.all`, Jolt's own
    /// `MotionProperties` default, on a static body or a stale id.
    pub fn getAllowedDOFs(self: BodyInterface, body: BodyId) AllowedDofs {
        return c.zjoltBodyGetAllowedDOFs(self.handle, body);
    }

    /// Whether this body currently has motion properties allocated at all.
    /// A static body has none, a kinematic or dynamic one always does, and a
    /// body created static with `allow_dynamic_or_kinematic = true` is the
    /// one case this actually distinguishes from an ordinary static body,
    /// since both currently report `getMotionType() == .static`.
    pub fn hasMotionProperties(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyHasMotionProperties(self.handle, body);
    }

    /// Sets the inverse mass (1 / mass) directly, no validation, no
    /// derived recomputation. Unlike `setMassProperties`, this can
    /// express `inverse_mass == 0` on a body that still translates (an
    /// infinitely heavy object pushed by nothing), since it never
    /// computes 1/mass or asserts mass is positive. A no-op on a static body.
    pub fn setInverseMass(self: BodyInterface, body: BodyId, inverse_mass: f32) err.Error!void {
        try err.check(c.zjoltBodySetInverseMass(self.handle, body, inverse_mass));
    }

    /// Sets the already-diagonalised inverse inertia tensor directly —
    /// the diagonal and the rotation that carries it into local space.
    /// Unlike `setMassProperties`, this never runs Jolt's
    /// principal-moments eigen-solver (which can fail and silently
    /// substitute a unit-sphere tensor). A no-op on a static body.
    pub fn setInverseInertia(self: BodyInterface, body: BodyId, diagonal: math.Vec3, rotation: math.Quat) err.Error!void {
        try err.check(c.zjoltBodySetInverseInertia(self.handle, body, &diagonal, &rotation));
    }

    /// Rescales an already-live body's mass and inertia together, keeping
    /// their ratio. `mass` must be positive.
    ///
    /// A no-op on a static body, or one whose current inverse mass is
    /// zero (every translation DOF locked, or a kinematic body never
    /// given a finite mass). Read `getInverseMassUnchecked` to confirm it changed anything.
    pub fn scaleToMass(self: BodyInterface, body: BodyId, mass: f32) err.Error!void {
        try err.check(c.zjoltBodyScaleToMass(self.handle, body, mass));
    }

    /// Gives a body a fully custom mass and inertia tensor, and sets its
    /// allowed degrees of freedom at the same time — Jolt couples the two
    /// since the inverse inertia is masked to the allowed rotation axes.
    /// Pass `getAllowedDOFs` back to leave the existing ones alone.
    ///
    /// A no-op on a static body. `allowed_dofs` with nothing set, or a non-positive mass with any translation allowed, is `error.InvalidArgument`.
    pub fn setMassProperties(
        self: BodyInterface,
        body: BodyId,
        allowed_dofs: AllowedDofs,
        mass_properties: MassProperties,
    ) err.Error!void {
        try err.check(c.zjoltBodySetMassProperties(self.handle, body, allowed_dofs, &mass_properties));
    }

    /// `v` with the translation axes `getAllowedDOFs` excludes zeroed out.
    /// `v` unchanged on a static body, matching `getAllowedDOFs`'s `.all` default.
    pub fn maskTranslationDOFs(self: BodyInterface, body: BodyId, v: math.Vec3) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyMaskTranslationDOFs(self.handle, body, &v, &out);
        return out;
    }

    /// As `maskTranslationDOFs`, but for the rotation axes.
    pub fn maskAngularDOFs(self: BodyInterface, body: BodyId, v: math.Vec3) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyMaskAngularDOFs(self.handle, body, &v, &out);
        return out;
    }

    /// Rescales the body's current linear velocity down to
    /// `getMaxLinearVelocity` if it exceeds it, in place. No-op below the
    /// limit or on a static body.
    pub fn clampLinearVelocity(self: BodyInterface, body: BodyId) err.Error!void {
        try err.check(c.zjoltBodyClampLinearVelocity(self.handle, body));
    }

    /// As `clampLinearVelocity`, for angular velocity and `getMaxAngularVelocity`.
    pub fn clampAngularVelocity(self: BodyInterface, body: BodyId) err.Error!void {
        try err.check(c.zjoltBodyClampAngularVelocity(self.handle, body));
    }

    /// `I_world^-1 * v`, using the body's current rotation. Same dynamic-only
    /// requirement as `getInverseInertia`.
    pub fn multiplyWorldSpaceInverseInertiaByVector(self: BodyInterface, body: BodyId, v: math.Vec3) err.Error!math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        try err.check(c.zjoltBodyMultiplyWorldSpaceInverseInertiaByVector(self.handle, body, &v, &out));
        return out;
    }

    /// As `getLocalSpaceInverseInertia`, but answers for a kinematic body too
    /// instead of refusing. Only a static body (no motion properties) is
    /// `error.InvalidArgument`.
    pub fn getLocalSpaceInverseInertiaUnchecked(self: BodyInterface, body: BodyId) err.Error!math.Mat44 {
        var out: math.Mat44 = math.mat44_identity;
        try err.check(c.zjoltBodyGetLocalSpaceInverseInertiaUnchecked(self.handle, body, &out));
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

    /// As calling `addForce` then `addTorque`, but under the same body lock
    /// and the same activation check — the two separate calls can straddle a
    /// concurrent step and leave it seeing only the force applied this
    /// sub-step; this cannot.
    pub fn addForceAndTorque(
        self: BodyInterface,
        body: BodyId,
        force: math.Vec3,
        torque: math.Vec3,
    ) void {
        c.zjoltBodyAddForceAndTorque(self.handle, body, &force, &torque);
    }

    /// What `addForce`/`addForceAtPoint`/`addTorque`/`addForceAndTorque`
    /// have accumulated since the last step consumed it. Zero for a body
    /// that is not dynamic or whose lock fails.
    ///
    /// A step clears both automatically; `resetForce`/`resetTorque` cancel one early.
    pub fn getAccumulatedForce(self: BodyInterface, body: BodyId) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyGetAccumulatedForce(self.handle, body, &out);
        return out;
    }

    pub fn getAccumulatedTorque(self: BodyInterface, body: BodyId) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltBodyGetAccumulatedTorque(self.handle, body, &out);
        return out;
    }

    /// A no-op on a body that is not dynamic or whose lock fails, same as
    /// the getters above.
    pub fn resetForce(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyResetForce(self.handle, body);
    }

    pub fn resetTorque(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyResetTorque(self.handle, body);
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

    /// Whether a contact between `body1` and `body2` uses manifold
    /// reduction — true only when BOTH allow it. `true`, matching
    /// `getUseManifoldReduction`'s own default, if either lock fails.
    pub fn getUseManifoldReductionWithBody(self: BodyInterface, body1: BodyId, body2: BodyId) bool {
        return c.zjoltBodyGetUseManifoldReductionWithBody(self.handle, body1, body2);
    }

    /// Whether this body is allowed to settle and go to sleep. Disabling
    /// it on a sleeping body does not wake it — pair with `activate`.
    ///
    /// Both are a no-op on a static body (no motion properties); the
    /// getter then answers `true`, Jolt's default, same as for a stale id.
    pub fn setAllowSleeping(self: BodyInterface, body: BodyId, allow: bool) void {
        c.zjoltBodySetAllowSleeping(self.handle, body, allow);
    }

    pub fn getAllowSleeping(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyGetAllowSleeping(self.handle, body);
    }

    /// Runtime linear damping: dv/dt = -c * v. `BodyDesc.linear_damping`
    /// sets the starting value at creation; this changes drag after the
    /// fact — a mud patch, an underwater volume.
    ///
    /// `damping` must not be negative. No-op (setter) or 0.05, Jolt's
    /// default (getter), on a static body (no motion properties).
    pub fn setLinearDamping(self: BodyInterface, body: BodyId, damping: f32) err.Error!void {
        try err.check(c.zjoltBodySetLinearDamping(self.handle, body, damping));
    }

    pub fn getLinearDamping(self: BodyInterface, body: BodyId) f32 {
        return c.zjoltBodyGetLinearDamping(self.handle, body);
    }

    /// As `setLinearDamping`/`getLinearDamping`, for angular damping.
    pub fn setAngularDamping(self: BodyInterface, body: BodyId, damping: f32) err.Error!void {
        try err.check(c.zjoltBodySetAngularDamping(self.handle, body, damping));
    }

    pub fn getAngularDamping(self: BodyInterface, body: BodyId) f32 {
        return c.zjoltBodyGetAngularDamping(self.handle, body);
    }

    /// Whether this body reports contacts without responding to them — a
    /// trigger volume. Unlocked counterpart of `Body.isSensor`.
    pub fn setIsSensor(self: BodyInterface, body: BodyId, is_sensor: bool) void {
        c.zjoltBodySetIsSensor(self.handle, body, is_sensor);
    }

    pub fn isSensor(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyIsSensor(self.handle, body);
    }

    /// Whether this body applies the gyroscopic force (the Dzhanibekov
    /// "tennis racket" effect) as part of the step. `false`, Jolt's own
    /// construction-time default, on a stale id.
    pub fn getApplyGyroscopicForce(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyGetApplyGyroscopicForce(self.handle, body);
    }

    /// Whether a kinematic body generates contacts against other kinematic
    /// or static bodies. Meaningless for a dynamic body, which already
    /// collides with everything its object layer allows regardless of this
    /// flag. `false`, Jolt's own construction-time default, on a stale id.
    pub fn getCollideKinematicVsNonDynamic(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyGetCollideKinematicVsNonDynamic(self.handle, body);
    }

    /// Whether this body gets the extra ghost-contact suppression a convex
    /// shape sliding over a mesh's internal edges needs. `false`, Jolt's own
    /// construction-time default, on a stale id.
    pub fn getEnhancedInternalEdgeRemoval(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyGetEnhancedInternalEdgeRemoval(self.handle, body);
    }

    /// Whether a contact between `body1` and `body2` gets enhanced
    /// internal-edge removal — true if EITHER body requests it. `false`,
    /// matching `getEnhancedInternalEdgeRemoval`'s own default, if either
    /// lock fails.
    pub fn getEnhancedInternalEdgeRemovalWithBody(self: BodyInterface, body1: BodyId, body2: BodyId) bool {
        return c.zjoltBodyGetEnhancedInternalEdgeRemovalWithBody(self.handle, body1, body2);
    }

    /// Whether this body changed in a way that invalidates cached contact
    /// results for every pair touching it. Set by `invalidateContactCache`
    /// and cleared once the next step reprocesses those pairs.
    pub fn isCollisionCacheInvalid(self: BodyInterface, body: BodyId) bool {
        return c.zjoltBodyIsCollisionCacheInvalid(self.handle, body);
    }

    /// The material of one leaf of the body's shape. @see `Shape.material`
    /// for what a material is and the sub-shape id rules.
    ///
    /// **Not a way to test whether a body exists**: a failed lock answers
    /// with the shared default material, same as a body with no materials
    /// of its own — Jolt's own answer, forwarded rather than replaced with a null. Use `isAdded` when the question is really about the body.
    pub fn getMaterial(
        self: BodyInterface,
        body: BodyId,
        sub_shape_id: c.SubShapeId,
    ) ?shape_mod.PhysicsMaterial {
        const handle = c.zjoltBodyGetMaterial(self.handle, body, sub_shape_id) orelse
            return null;
        return .{ .handle = handle };
    }

    /// Tells the system a body's shape changed underneath it — what the
    /// `MutableCompound` methods do.
    ///
    /// `previous_center_of_mass` is the shape's centre of mass BEFORE the
    /// change; the body is moved so its geometry stays where it was.
    /// Without this call the broad phase and contact cache keep describing the prior shape.
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

    /// Drops the cached collision result for every pair involving this
    /// body, so the next step re-evaluates them — what makes a
    /// collision-group change take effect on a pair already resting.
    /// `PhysicsSettings.use_body_pair_contact_cache` (on by default)
    /// skips the narrow phase for a pair whose transform has not moved, which a group change does not.
    pub fn invalidateContactCache(self: BodyInterface, body: BodyId) void {
        c.zjoltBodyInvalidateContactCache(self.handle, body);
    }

    //-------------------------------------------------------------------------
    // Detaching a body from its id
    //-------------------------------------------------------------------------

    /// Removes `body`'s id and hands back the still-alive object, added
    /// or not — freeing the id for reuse while the body survives for a
    /// new one (`UnassignedBody.assignId`) or a later
    /// `UnassignedBody.destroy`. `body` must currently name a live body;
    /// `error.BodyNotFound` otherwise.
    pub fn unassignId(self: BodyInterface, body: BodyId) err.Error!UnassignedBody {
        var handle: *c.UnassignedBody = undefined;
        try err.check(c.zjoltBodyUnassignId(self.handle, body, &handle));
        return .{ .handle = handle, .owner = self.handle };
    }
};

/// A body that had `BodyInterface.unassignId` called on it: it keeps its
/// shape, transform, velocity and every other property, but no longer has an
/// id and cannot be queried, stepped, or found by any `BodyInterface` call
/// until it is given one back. Owned outright — release it with `assignId`
/// or `destroy`, exactly one of the two.
pub const UnassignedBody = struct {
    handle: *c.UnassignedBody,
    owner: *c.PhysicsSystem,

    /// Gives this body the id `id`, consuming it. `id` is checked the same
    /// way `BodyInterface.createWithId` checks one: not `invalid_body_id`,
    /// bit 31 clear, and not already naming a live body.
    pub fn assignId(self: UnassignedBody, id: BodyId) err.Error!BodyId {
        var out: BodyId = invalid_body_id;
        try err.check(c.zjoltUnassignedBodyAssignId(self.owner, self.handle, id, &out));
        return out;
    }

    /// Destroys the object outright instead of giving it back an id.
    pub fn destroy(self: UnassignedBody) err.Error!void {
        try err.check(c.zjoltUnassignedBodyDestroy(self.owner, self.handle));
    }
};

//=============================================================================
// Locks
//
// The accessors above take a lock per call; a lock holds one body still
// for a scope — for reading several properties at once, or reaching the mutators that bypass activation. `defer lock.release()`.
//=============================================================================

/// A borrowed view of a locked body. Must not outlive its lock.
///
/// Composes into what Jolt's own `BodyFilter::ShouldCollideLocked` gives a
/// query filter for free — a body already locked, no second acquisition:
/// `PhysicsSystem.lockRead`/`lockWrite` plus these accessors filter by body
/// member exactly as that callback would, at the cost of one extra lock.
pub const Body = struct {
    handle: *c.Body,

    pub fn id(self: Body) BodyId {
        return c.zjoltBodyGetId(self.handle);
    }

    /// Position, rotation and velocity accessors below assume ordinary
    /// caller code, outside `PhysicsSystem.step`. Jolt narrows this window
    /// itself during its own step — a contact listener, a custom
    /// constraint's setup — and asserts a violation in a debug build; mind
    /// the same split from inside one of those.
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

    /// Whether this body's motion type is `.dynamic` — Body::IsDynamic.
    pub fn isDynamic(self: Body) bool {
        return self.motionType() == .dynamic;
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

    /// MotionProperties::GetSimulationStats, averaged over the simulation
    /// island this body was part of during its last step.
    ///
    /// `error.Unsupported` unless built with `-Dtrack_simulation_stats`
    /// (Jolt compiles the counters out without it). A STATIC body reads
    /// back all zeroes either way — an ordinary body with nothing tracked, not an unsupported build.
    pub fn simulationStats(self: Body) err.Error!SimulationStats {
        var out: SimulationStats = undefined;
        try err.check(c.zjoltBodyGetSimulationStatsLocked(self.handle, &out));
        return out;
    }

    /// Debug check: this body's cached broad-phase bounds still match its
    /// shape's actual bounds, asserting on a mismatch (a mutable compound
    /// edited without `BodyInterface.notifyShapeChanged`).
    ///
    /// `error.Unsupported` when Jolt is built without asserts, where the
    /// check does not exist to run.
    pub fn validateCachedBounds(self: Body) err.Error!void {
        try err.check(c.zjoltBodyValidateCachedBoundsLocked(self.handle));
    }

    /// Debug check: a sleeping body has zero velocity, catching a velocity
    /// set directly on motion properties rather than through
    /// `BodyInterface.setLinearVelocity`/`setAngularVelocity`, which wake
    /// the body first. `error.Unsupported` without asserts, same as `validateCachedBounds`.
    pub fn validateMotion(self: Body) err.Error!void {
        try err.check(c.zjoltBodyValidateMotionLocked(self.handle));
    }

    //-------------------------------------------------------------------------
    // Mutators — require a write lock
    //
    // These bypass activation on purpose: for adjusting a body already
    // held still. A sleeping body stays asleep — activate it separately.
    //-------------------------------------------------------------------------

    pub fn setLinearVelocity(self: Body, velocity: math.Vec3) void {
        c.zjoltBodySetLinearVelocityLocked(self.handle, &velocity);
    }

    pub fn setAngularVelocity(self: Body, velocity: math.Vec3) void {
        c.zjoltBodySetAngularVelocityLocked(self.handle, &velocity);
    }

    pub fn setUserData(self: Body, user_data: u64) void {
        c.zjoltBodySetUserDataLocked(self.handle, user_data);
    }

    pub fn setFriction(self: Body, friction: f32) void {
        c.zjoltBodySetFrictionLocked(self.handle, friction);
    }

    pub fn setRestitution(self: Body, restitution: f32) void {
        c.zjoltBodySetRestitutionLocked(self.handle, restitution);
    }

    pub fn addImpulse(self: Body, impulse: math.Vec3) void {
        c.zjoltBodyAddImpulseLocked(self.handle, &impulse);
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

/// A lock held over several bodies at once, under one mutex mask so
/// nothing else can touch any of them between the first read and the
/// last — what taking them one at a time with `Lock` cannot promise.
/// Release it exactly once.
///
/// `ids` (the slice `lockMultiRead`/`lockMultiWrite` was given) must stay valid until `release`: this borrows it, same as Jolt's own BodyLockMultiRead.
pub const MultiLock = struct {
    raw: c.BodyLockMulti,
    write: bool,

    /// The body at `index`, or null if that id no longer names a live body —
    /// which is normal, not an error. `index` must be less than the length of
    /// the slice this lock was taken over.
    pub fn body(self: *const MultiLock, index: u32) ?Body {
        return .{ .handle = c.zjoltBodyLockMultiGet(&self.raw, index) orelse return null };
    }

    pub fn release(self: *MultiLock) void {
        if (self.write) {
            c.zjoltBodyLockMultiWriteRelease(&self.raw);
        } else {
            c.zjoltBodyLockMultiReadRelease(&self.raw);
        }
    }
};
