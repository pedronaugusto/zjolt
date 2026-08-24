//! Soft bodies: cloth, rope, and jelly-like volumes simulated as a mesh of
//! particles rather than driven by a rigid shape.
//!
//! Building one is two stages, same as Jolt's own:
//!
//!   1. `SharedSettings` carries the topology — vertices, faces, and the
//!      constraints that hold them together. It is reference counted and
//!      shareable: build one cloth or one blob of jelly and stamp out many
//!      soft bodies from it. `createConstraints` derives edge, shear and
//!      bend constraints from the faces automatically, rather than
//!      requiring every edge spelled out by hand.
//!   2. `create`/`createAndAdd` place one instance of that topology in the
//!      world — NOT `PhysicsSystem.bodies().create`. A soft body has no
//!      shape, no motion type, and none of a rigid body's mass overrides;
//!      its "shape" is its own simulated vertices, and Jolt creates it
//!      through `BodyInterface::CreateSoftBody`, a distinct entry point
//!      from `CreateBody`. The `BodyId` that comes back is ordinary,
//!      though: add it, remove it, put it in a collision group, through
//!      `PhysicsSystem.bodies()` exactly like a rigid body.
//!
//! Per step, a soft body's vertices move on their own — there is no single
//! transform to read back the way a rigid body has one. `getVertexStates`
//! is the bulk read-back for that: one lock, one crossing, every vertex.
//!
//! A body that already exists is driven through the calls below
//! `getVertexStates`: the live properties Jolt keeps on its
//! `SoftBodyMotionProperties` (iteration count, pressure, vertex radius, the
//! skin flags), per-vertex velocity and inverse mass, and `skinVertices`,
//! which is what a host calls each frame to make a cloth follow an animated
//! skeleton.
//!
//! There is no `PhysicsSystem.softBodies()` accessor, unlike `.bodies()` or
//! `.broadPhase()`: every function below takes the `PhysicsSystem` it
//! operates on directly instead.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const group_mod = @import("group.zig");
const system_mod = @import("system.zig");

pub const BendType = c.SoftBodyBendType;
pub const LraType = c.SoftBodyLraType;

pub const Vertex = c.SoftBodyVertex;
pub const Face = c.SoftBodyFace;
pub const Edge = c.SoftBodyEdge;
pub const VolumeConstraint = c.SoftBodyVolumeConstraint;
pub const InvBind = c.SoftBodyInvBind;
pub const SkinWeight = c.SoftBodySkinWeight;
pub const Skinned = c.SoftBodySkinned;
pub const VertexAttributes = c.SoftBodyVertexAttributes;
pub const VertexState = c.SoftBodyVertexState;

/// Jolt's own defaults for `VertexAttributes`, read out of a
/// default-constructed one rather than transcribed.
pub fn defaultVertexAttributes() VertexAttributes {
    var out: VertexAttributes = undefined;
    c.zjoltSoftBodyVertexAttributesInit(&out);
    return out;
}

//=============================================================================
// Shared settings
//=============================================================================

/// Topology shared by one or more soft bodies. Reference counted like a
/// `Shape`: build once, create as many soft bodies from it as you like,
/// release when done creating.
pub const SharedSettings = struct {
    handle: *c.SoftBodySharedSettings,

    pub fn create() err.Error!SharedSettings {
        var handle: *c.SoftBodySharedSettings = undefined;
        try err.check(c.zjoltSoftBodySharedSettingsCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn addRef(self: SharedSettings) void {
        c.zjoltSoftBodySharedSettingsAddRef(self.handle);
    }

    pub fn release(self: SharedSettings) void {
        c.zjoltSoftBodySharedSettingsRelease(self.handle);
    }

    pub fn refCount(self: SharedSettings) u32 {
        return c.zjoltSoftBodySharedSettingsGetRefCount(self.handle);
    }

    /// Vertex indices used by every call below refer to the order vertices
    /// were appended in, across every `addVertices` call made on these
    /// settings so far — add all of a body's vertices before the faces and
    /// constraints that reference them.
    pub fn addVertices(self: SharedSettings, vertices: []const Vertex) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddVertices(
            self.handle,
            vertices.ptr,
            @intCast(vertices.len),
        ));
    }

    /// Fails without adding any of them if one repeats a vertex, or names a
    /// vertex index past the vertices added so far.
    pub fn addFaces(self: SharedSettings, faces: []const Face) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddFaces(
            self.handle,
            faces.ptr,
            @intCast(faces.len),
        ));
    }

    /// Fails without adding any of them if one connects a vertex to itself,
    /// or names a vertex index past the vertices added so far. Rest lengths
    /// stay at Jolt's placeholder until `calculateEdgeLengths` runs.
    pub fn addEdges(self: SharedSettings, edges: []const Edge) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddEdges(
            self.handle,
            edges.ptr,
            @intCast(edges.len),
        ));
    }

    /// Fails without adding any of them if one repeats a vertex among its
    /// four, or names a vertex index past the vertices added so far.
    pub fn addVolumeConstraints(self: SharedSettings, constraints: []const VolumeConstraint) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddVolumeConstraints(
            self.handle,
            constraints.ptr,
            @intCast(constraints.len),
        ));
    }

    /// For `Skinned.weights` to reference by index.
    pub fn addInvBindMatrices(self: SharedSettings, inv_binds: []const InvBind) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddInvBindMatrices(
            self.handle,
            inv_binds.ptr,
            @intCast(inv_binds.len),
        ));
    }

    /// Fails without adding any of them if one names a vertex index past the
    /// vertices added so far — Jolt indexes with it on every `skinVertices`
    /// call and on every step. A weight's `inv_bind_index` is not checked
    /// here, because the inverse bind matrix list is still allowed to grow;
    /// `skinVertices` checks it instead.
    pub fn addSkinnedConstraints(self: SharedSettings, constraints: []const Skinned) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddSkinnedConstraints(
            self.handle,
            constraints.ptr,
            @intCast(constraints.len),
        ));
    }

    /// Builds edge, shear and bend constraints from the faces already
    /// added, instead of listing every edge by hand, and calls
    /// `calculateEdgeLengths` for you.
    ///
    /// `vertex_attributes` is indexed in parallel with the vertices added
    /// so far; Jolt repeats its last element for any vertex beyond the
    /// list, which is why the list must have at least one entry.
    pub fn createConstraints(
        self: SharedSettings,
        vertex_attributes: []const VertexAttributes,
        bend_type: BendType,
        angle_tolerance_radians: f32,
    ) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsCreateConstraints(
            self.handle,
            vertex_attributes.ptr,
            @intCast(vertex_attributes.len),
            bend_type,
            angle_tolerance_radians,
        ));
    }

    /// Measures every edge constraint's rest length from its two vertices'
    /// current positions. `createConstraints` already does this for the
    /// edges it builds; call this too if you added edges directly through
    /// `addEdges`.
    pub fn calculateEdgeLengths(self: SharedSettings) void {
        c.zjoltSoftBodySharedSettingsCalculateEdgeLengths(self.handle);
    }

    /// Works out which faces meet at each skinned vertex, which is what
    /// gives that vertex a normal — and the normal is the direction
    /// `Skinned.back_stop_distance` is measured along.
    ///
    /// Neither `optimize` nor `createConstraints` does this. Settings that
    /// never had it run still simulate; every skinned vertex simply has a
    /// zero normal, so the back stop does nothing and only `max_distance`
    /// still bites. Only a face whose three vertices are ALL skinned
    /// contributes a normal.
    ///
    /// Refused rather than left to Jolt's assert: more than 255 fully
    /// skinned faces meeting at one skinned vertex, which does not fit the
    /// 8-bit count Jolt packs it into.
    pub fn calculateSkinnedConstraintNormals(self: SharedSettings) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsCalculateSkinnedConstraintNormals(self.handle));
    }

    /// Reorders constraints so the solver can run groups of them in
    /// parallel. Call once, after every vertex, face and constraint has
    /// been added, and before creating any soft body from these settings.
    pub fn optimize(self: SharedSettings) void {
        c.zjoltSoftBodySharedSettingsOptimize(self.handle);
    }
};

//=============================================================================
// Creating a soft body
//=============================================================================

/// How a soft body is born. The defaults are Jolt's own, read out of a
/// default-constructed `SoftBodyCreationSettings` rather than transcribed —
/// see `Desc.toC`.
pub const Desc = struct {
    /// Required. A reference is taken for the lifetime of the body — call
    /// `.optimize()` on it first.
    shared_settings: SharedSettings,
    object_layer: body_mod.ObjectLayer,

    position: math.RVec3 = math.rvec3_zero,
    rotation: math.Quat = math.quat_identity,
    user_data: u64 = 0,

    /// Exceptions to layer-based collision, on the same terms as
    /// `BodyDesc.collision_group`. The default makes none.
    collision_group: group_mod.CollisionGroup = .{},

    /// Solver iterations per step. Must be at least 1: Jolt sizes its
    /// sub-step as the step's delta time divided by this, so 0 makes every
    /// sub-step infinite. Refused rather than forwarded.
    num_iterations: u32 = 5,
    linear_damping: f32 = 0.1,
    max_linear_velocity: f32 = 500,
    restitution: f32 = 0,
    friction: f32 = 0.2,
    /// n * R * T: amount of substance times the ideal gas constant times
    /// absolute temperature. 0 disables internal pressure entirely.
    pressure: f32 = 0,
    gravity_factor: f32 = 1,
    /// Pushes vertices this far off the surface of whatever they collide
    /// with, to reduce z-fighting. Negative is refused — see
    /// `Body.setVertexRadius` for what Jolt's own assert does and does not
    /// catch.
    vertex_radius: f32 = 0,
    update_position: bool = true,
    /// Bakes `rotation` into the vertices and gives the body an identity
    /// rotation instead, which is slightly more accurate to simulate.
    make_rotation_identity: bool = true,
    allow_sleeping: bool = true,
    faces_double_sided: bool = false,

    fn toC(self: Desc) c.SoftBodyDesc {
        var out: c.SoftBodyDesc = undefined;
        // Start from Jolt's defaults so a field this wrapper does not model
        // still gets a sensible value rather than whatever was on the stack.
        c.zjoltSoftBodyDescInit(&out);
        out.shared_settings = self.shared_settings.handle;
        out.collision_group = group_mod.toC(self.collision_group);
        out.position = self.position;
        out.rotation = self.rotation;
        out.user_data = self.user_data;
        out.object_layer = self.object_layer;
        out.num_iterations = self.num_iterations;
        out.linear_damping = self.linear_damping;
        out.max_linear_velocity = self.max_linear_velocity;
        out.restitution = self.restitution;
        out.friction = self.friction;
        out.pressure = self.pressure;
        out.gravity_factor = self.gravity_factor;
        out.vertex_radius = self.vertex_radius;
        out.update_position = self.update_position;
        out.make_rotation_identity = self.make_rotation_identity;
        out.allow_sleeping = self.allow_sleeping;
        out.faces_double_sided = self.faces_double_sided;
        return out;
    }
};

/// Used by `scene.zig`, for the same reason `body.zig`'s `descToC` is.
pub fn descToC(desc: Desc) c.SoftBodyDesc {
    return desc.toC();
}

/// Creates a soft body without adding it to the simulation.
pub fn create(system: system_mod.PhysicsSystem, desc: Desc) err.Error!body_mod.BodyId {
    const c_desc = desc.toC();
    var id: body_mod.BodyId = body_mod.invalid_body_id;
    try err.check(c.zjoltSoftBodyCreate(system.handle, &c_desc, &id));
    return id;
}

pub fn createAndAdd(
    system: system_mod.PhysicsSystem,
    desc: Desc,
    activation: body_mod.Activation,
) err.Error!body_mod.BodyId {
    const c_desc = desc.toC();
    var id: body_mod.BodyId = body_mod.invalid_body_id;
    try err.check(c.zjoltSoftBodyCreateAndAdd(system.handle, &c_desc, activation, &id));
    return id;
}

//=============================================================================
// Per-step read-back
//=============================================================================

pub fn countVertexStates(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!u32 {
    var count: u32 = 0;
    try err.check(c.zjoltSoftBodyGetVertexStates(system.handle, body, null, 0, &count));
    return count;
}

/// Reads every simulated vertex of one soft body into `buffer`, under a
/// single lock. Size `buffer` with `countVertexStates` first.
///
/// Both `position` and `velocity` of each state are relative to the soft
/// body's CENTER OF MASS, not world space — the same frame
/// `BodyInterface.getCenterOfMassPosition` and `.getRotation` report. Add
/// the body's center-of-mass position, rotated by its rotation if the body
/// has moved, to place a vertex in the world.
pub fn getVertexStates(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    buffer: []VertexState,
) err.Error![]VertexState {
    var count: u32 = 0;
    try err.check(c.zjoltSoftBodyGetVertexStates(
        system.handle,
        body,
        buffer.ptr,
        @intCast(buffer.len),
        &count,
    ));
    return buffer[0..count];
}

//=============================================================================
// Live properties of one soft body
//
// The knobs Jolt keeps on a body's own `SoftBodyMotionProperties`, which a
// host changes while the simulation runs. Each takes the system and the body
// id and resolves those properties under a lock, exactly as `getVertexStates`
// does — a body id naming nothing is `error.BodyNotFound`, and one naming a
// RIGID body is `error.InvalidArgument` rather than a blind cast.
//
// None of these wake a sleeping body.
//=============================================================================

/// Solver iterations this body runs per step. @see `Desc.num_iterations` for
/// why 0 is refused.
pub fn getNumIterations(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!u32 {
    var out: u32 = 0;
    try err.check(c.zjoltSoftBodyGetNumIterations(system.handle, body, &out));
    return out;
}

pub fn setNumIterations(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    num_iterations: u32,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetNumIterations(system.handle, body, num_iterations));
}

/// Internal gas pressure: n * R * T. 0 disables it.
///
/// The force it produces is computed from the volume the faces enclose, so it
/// means nothing on a body whose faces do not close a surface, and it squeezes
/// rather than inflates one whose faces wind inside out. `getVolume` is what
/// tells the two apart.
pub fn getPressure(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!f32 {
    var out: f32 = 0;
    try err.check(c.zjoltSoftBodyGetPressure(system.handle, body, &out));
    return out;
}

pub fn setPressure(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    pressure: f32,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetPressure(system.handle, body, pressure));
}

/// Whether the body's own position follows its vertices as they move. Turn it
/// off for a soft body pinned to the static world, whose vertices move but
/// whose origin should not.
pub fn getUpdatePosition(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!bool {
    var out: bool = false;
    try err.check(c.zjoltSoftBodyGetUpdatePosition(system.handle, body, &out));
    return out;
}

pub fn setUpdatePosition(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    update_position: bool,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetUpdatePosition(system.handle, body, update_position));
}

/// Whether ray casts, collide-shape and cast-shape hit this body's faces from
/// behind as well as from in front. Affects queries only; the solver's own
/// collision handling does not read it.
pub fn getFacesDoubleSided(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!bool {
    var out: bool = false;
    try err.check(c.zjoltSoftBodyGetFacesDoubleSided(system.handle, body, &out));
    return out;
}

pub fn setFacesDoubleSided(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    double_sided: bool,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetFacesDoubleSided(system.handle, body, double_sided));
}

/// How far this body's vertices are held off the surface of whatever they
/// touch. Negative is `error.InvalidArgument`.
///
/// Jolt's own setter asserts on this, but on the value it is about to
/// OVERWRITE rather than the one being set, so upstream notices a bad radius
/// one call late — or never, if the caller sets it only once. The check here
/// is on the incoming value.
pub fn getVertexRadius(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!f32 {
    var out: f32 = 0;
    try err.check(c.zjoltSoftBodyGetVertexRadius(system.handle, body, &out));
    return out;
}

pub fn setVertexRadius(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    vertex_radius: f32,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetVertexRadius(system.handle, body, vertex_radius));
}

//=============================================================================
// One vertex at a time
//
// Velocity and inverse mass are the only two parts of a simulated vertex Jolt
// sanctions writing while the body runs. There is deliberately no setter for a
// position: writing one moves the vertex without moving the previous position
// the solver integrates from, and the step that follows misses every collision
// along the way.
//
// `index` is into the same array `getVertexStates` reads, in the same order,
// and past the end is `error.InvalidArgument` rather than an out-of-bounds
// read inside Jolt.
//=============================================================================

/// One vertex's velocity, relative to the body's CENTRE OF MASS — the frame
/// `VertexState` uses, not world space.
pub fn getVertexVelocity(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    index: u32,
) err.Error!math.Vec3 {
    var out: math.Vec3 = math.vec3_zero;
    try err.check(c.zjoltSoftBodyGetVertexVelocity(system.handle, body, index, &out));
    return out;
}

pub fn setVertexVelocity(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    index: u32,
    velocity: math.Vec3,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetVertexVelocity(system.handle, body, index, &velocity));
}

/// One vertex's inverse mass. 0 pins it: it still takes part in every
/// constraint, but nothing moves it — which is how a cloth is nailed to a
/// flagpole, or to a hand.
///
/// Changing this does NOT recompute the body's own mass and inertia. @see
/// `calculateMassAndInertia`.
pub fn getVertexInvMass(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    index: u32,
) err.Error!f32 {
    var out: f32 = 0;
    try err.check(c.zjoltSoftBodyGetVertexInvMass(system.handle, body, index, &out));
    return out;
}

pub fn setVertexInvMass(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    index: u32,
    inv_mass: f32,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetVertexInvMass(system.handle, body, index, inv_mass));
}

/// Recomputes the body's total mass and inertia from its vertices' current
/// inverse masses and positions. Jolt does this once, at creation, and never
/// again — so a body whose per-vertex inverse masses were changed keeps the
/// mass it was born with until this runs.
///
/// A single vertex with an inverse mass of 0 gives the WHOLE body infinite
/// mass and inertia. That is upstream's rule, and it is why pinning one corner
/// of a cloth stops the body as a whole from responding to an impulse.
pub fn calculateMassAndInertia(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
) err.Error!void {
    try err.check(c.zjoltSoftBodyCalculateMassAndInertia(system.handle, body));
}

//=============================================================================
// Cheap measurements
//=============================================================================

/// The volume the body's faces enclose, in its own local space. One pass over
/// the faces, no allocation.
///
/// NEGATIVE when the faces wind inside out, and merely a number — not zero —
/// for a surface that does not close, such as a sheet of cloth. Jolt computes
/// its pressure force from exactly this quantity.
pub fn getVolume(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!f32 {
    var out: f32 = 0;
    try err.check(c.zjoltSoftBodyGetVolume(system.handle, body, &out));
    return out;
}

/// The box around every vertex, in the body's own local space rather than the
/// world's. Maintained by the solver as it steps, so this reads a cached value
/// instead of walking the vertices.
pub fn getLocalBounds(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!math.AABox {
    var out: math.AABox = undefined;
    try err.check(c.zjoltSoftBodyGetLocalBounds(system.handle, body, &out));
    return out;
}

//=============================================================================
// Skinning to an animated skeleton
//
// A cloth that follows a character is not simulated out of nothing. Each frame
// the host hands Jolt the skeleton's joint matrices; Jolt skins every vertex
// carrying a `Skinned` constraint the way a renderer would, and the solver is
// then allowed to pull the simulated vertex away from that skinned position by
// at most `Skinned.max_distance`.
//
// The authoring half is on `SharedSettings` — `addInvBindMatrices`,
// `addSkinnedConstraints`, `calculateSkinnedConstraintNormals`. This is the
// per-frame half: animate the skeleton, call `skinVertices`, then step.
//=============================================================================

/// Skins every vertex carrying a `Skinned` constraint to `joint_matrices`, and
/// records the result for the solver to constrain against on the next step.
///
/// `joint_matrices` is indexed by `InvBind.joint_index`. Each matrix must be
/// expressed RELATIVE TO THE BODY'S CENTRE-OF-MASS TRANSFORM, not in world
/// space: take the world joint matrix and pre-multiply it by the inverse of
/// `PhysicsSystem.bodies().getCenterOfMassTransform`. World-space matrices are
/// not detectably wrong and nothing here fails — the cloth simply skins to
/// wherever the body's centre of mass sits relative to the origin, which is
/// the single most common way to get this call wrong.
///
/// `hard_skin_all` puts every skinned vertex exactly on its skinned position
/// and zeroes its velocity, ignoring `max_distance` entirely. That is a reset,
/// for the frame a character spawns or is teleported.
///
/// `error.InvalidArgument` rather than one of Jolt's asserts when an inverse
/// bind matrix names a joint past `joint_matrices`, when a skin weight names
/// an inverse bind matrix that was never added, or when the body's shared
/// settings carry no skinned constraints at all — Jolt sizes the per-instance
/// skinning state once, at creation, and only when there is at least one
/// skinned constraint to size it for.
pub fn skinVertices(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    joint_matrices: []const math.Mat44,
    hard_skin_all: bool,
) err.Error!void {
    try err.check(c.zjoltSoftBodySkinVertices(
        system.handle,
        body,
        joint_matrices.ptr,
        @intCast(joint_matrices.len),
        hard_skin_all,
    ));
}

/// Whether the solver enforces this body's skin constraints at all.
///
/// Switching it off does not stop `skinVertices` from doing its work: with the
/// constraints off, that call hard-skins every vertex whose `max_distance` is
/// 0 and leaves the rest to simulate freely. So this is the difference between
/// "cloth that follows the character" and "cloth pinned at its kinematic
/// vertices and otherwise loose".
pub fn getEnableSkinConstraints(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!bool {
    var out: bool = false;
    try err.check(c.zjoltSoftBodyGetEnableSkinConstraints(system.handle, body, &out));
    return out;
}

pub fn setEnableSkinConstraints(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    enable: bool,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetEnableSkinConstraints(system.handle, body, enable));
}

/// Scales every skin constraint's `max_distance` at once, so a whole garment
/// can be tightened or loosened without touching the shared settings every
/// instance is stamped from. 1 is the default; 0 hard-skins every vertex.
pub fn getSkinnedMaxDistanceMultiplier(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
) err.Error!f32 {
    var out: f32 = 0;
    try err.check(c.zjoltSoftBodyGetSkinnedMaxDistanceMultiplier(system.handle, body, &out));
    return out;
}

pub fn setSkinnedMaxDistanceMultiplier(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    multiplier: f32,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetSkinnedMaxDistanceMultiplier(system.handle, body, multiplier));
}

//=============================================================================
// Tests
//=============================================================================

/// Two object layers, so a soft body can be dropped on a static floor. The
/// smallest map that still rejects static-vs-static.
const TestLayers = struct {
    pub const static: system_mod.ObjectLayer = 0;
    pub const moving: system_mod.ObjectLayer = 1;

    pub const bp_static: system_mod.BroadPhaseLayer = 0;
    pub const bp_moving: system_mod.BroadPhaseLayer = 1;

    pub fn broadPhaseLayerCount() u32 {
        return 2;
    }

    pub fn broadPhaseLayerFor(layer: system_mod.ObjectLayer) system_mod.BroadPhaseLayer {
        return if (layer == static) bp_static else bp_moving;
    }

    pub fn objectCanCollideWithBroadPhase(
        object: system_mod.ObjectLayer,
        broad: system_mod.BroadPhaseLayer,
    ) bool {
        return if (object == static) broad == bp_moving else true;
    }

    pub fn objectsCanCollide(a: system_mod.ObjectLayer, b: system_mod.ObjectLayer) bool {
        return if (a == static) b == moving else true;
    }
};

/// A unit cube's eight corners, every one of them free to move.
const cube_vertices = [8]Vertex{
    .{ .position = math.vec3(-0.5, -0.5, -0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(0.5, -0.5, -0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(0.5, -0.5, 0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(-0.5, -0.5, 0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(-0.5, 0.5, -0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(0.5, 0.5, -0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(0.5, 0.5, 0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(-0.5, 0.5, 0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
};

/// The cube's twelve triangles, every one of them wound counter-clockwise seen
/// from OUTSIDE. That winding is what makes the enclosed volume come back
/// positive rather than negative, and it is the only thing standing between a
/// pressurised body and one that squeezes itself flat.
const cube_faces = [12]Face{
    .{ .vertex = .{ 0, 1, 2 }, .material_index = 0 },
    .{ .vertex = .{ 0, 2, 3 }, .material_index = 0 },
    .{ .vertex = .{ 4, 6, 5 }, .material_index = 0 },
    .{ .vertex = .{ 4, 7, 6 }, .material_index = 0 },
    .{ .vertex = .{ 3, 2, 6 }, .material_index = 0 },
    .{ .vertex = .{ 3, 6, 7 }, .material_index = 0 },
    .{ .vertex = .{ 0, 4, 5 }, .material_index = 0 },
    .{ .vertex = .{ 0, 5, 1 }, .material_index = 0 },
    .{ .vertex = .{ 0, 3, 7 }, .material_index = 0 },
    .{ .vertex = .{ 0, 7, 4 }, .material_index = 0 },
    .{ .vertex = .{ 1, 5, 6 }, .material_index = 0 },
    .{ .vertex = .{ 1, 6, 2 }, .material_index = 0 },
};

fn buildCube() err.Error!SharedSettings {
    const settings = try SharedSettings.create();
    errdefer settings.release();
    try settings.addVertices(&cube_vertices);
    try settings.addFaces(&cube_faces);
    const attributes = [_]VertexAttributes{defaultVertexAttributes()};
    try settings.createConstraints(&attributes, .dihedral, std.math.degreesToRadians(8.0));
    settings.optimize();
    return settings;
}

test "a pressurised soft body settles on the floor with its volume intact" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const jobs = try system_mod.JobSystem.initSingleThreaded(c.max_physics_jobs);
    defer jobs.deinit();

    const system = try system_mod.PhysicsSystem.init(.{
        .layers = system_mod.layersFromType(TestLayers),
        .max_bodies = 64,
    });
    defer system.deinit();
    system.setGravity(math.gravity_earth);

    const shape_mod = @import("shape.zig");
    const floor_shape = try shape_mod.Shape.initBox(math.vec3(20, 0.5, 20), .{});
    defer floor_shape.release();
    _ = try system.bodies().createAndAdd(.{
        .shape = floor_shape,
        .object_layer = TestLayers.static,
        .motion_type = .static,
        .position = math.rvec3(0, -0.5, 0),
    }, .dont_activate);

    const settings = try buildCube();
    defer settings.release();

    const cube = try createAndAdd(system, .{
        .shared_settings = settings,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 4, 0),
        .pressure = 20,
        .vertex_radius = 0.01,
        // Sleeping mid-fall would make "it fell" a race with the sleep timer.
        .allow_sleeping = false,
    }, .activate);

    // The cube is a cube before anything has stepped: a unit cube encloses a
    // unit volume, and the sign is what says the faces were wound the right
    // way round.
    const volume_before = try getVolume(system, cube);
    try std.testing.expectApproxEqAbs(@as(f32, 1), volume_before, 0.05);
    try std.testing.expectEqual(@as(u32, cube_vertices.len), try countVertexStates(system, cube));

    var before: [cube_vertices.len]VertexState = undefined;
    _ = try getVertexStates(system, cube, &before);
    const com_before = system.bodies().getCenterOfMassPosition(cube);

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 2.0) : (elapsed += dt) {
        _ = try system.step(dt, 1, jobs);
    }

    var after: [cube_vertices.len]VertexState = undefined;
    _ = try getVertexStates(system, cube, &after);
    const com_after = system.bodies().getCenterOfMassPosition(cube);

    // Every vertex is lower than it was: vertex states are relative to the
    // centre of mass, so a vertex's world height is that centre plus its own,
    // and it is the pair that says the body fell rather than merely deformed.
    for (before, after) |b, a| {
        const world_before = @as(f64, com_before.y) + b.position.y;
        const world_after = @as(f64, com_after.y) + a.position.y;
        try std.testing.expect(world_after < world_before - 1.0);
        try std.testing.expect(std.math.isFinite(a.position.x));
        try std.testing.expect(std.math.isFinite(a.velocity.y));
    }

    // And it stopped on the floor rather than falling through it: two seconds
    // of free fall from y = 4 would be far below it.
    try std.testing.expect(com_after.y > -0.5);
    try std.testing.expect(com_after.y < 3.0);

    // Pressure held the cube open. The exact volume depends on how far the
    // solver let the floor squash it, so this is a band, not a number: it did
    // not collapse and it did not explode.
    const volume_after = try getVolume(system, cube);
    try std.testing.expect(volume_after > 0.25);
    try std.testing.expect(volume_after < 8.0);

    // The bounds are local, so they stay around the origin no matter how far
    // the body has fallen.
    const bounds = try getLocalBounds(system, cube);
    try std.testing.expect(bounds.max.y > bounds.min.y);
    try std.testing.expect(bounds.min.y > -4.0);

    // The live properties read back what the body was born with, and take a
    // change while it is running.
    try std.testing.expectApproxEqAbs(@as(f32, 20), try getPressure(system, cube), 1.0e-3);
    try setPressure(system, cube, 0);
    try std.testing.expectEqual(@as(f32, 0), try getPressure(system, cube));

    try std.testing.expectEqual(@as(u32, 5), try getNumIterations(system, cube));
    try setNumIterations(system, cube, 3);
    try std.testing.expectEqual(@as(u32, 3), try getNumIterations(system, cube));

    try std.testing.expect(try getEnableSkinConstraints(system, cube));
    try setEnableSkinConstraints(system, cube, false);
    try std.testing.expect(!try getEnableSkinConstraints(system, cube));

    // Pinning a vertex is the one runtime write that changes what the solver
    // does, and it is only half of the change: the body keeps the mass it was
    // born with until the recalculation runs.
    try std.testing.expectEqual(@as(f32, 1), try getVertexInvMass(system, cube, 0));
    try setVertexInvMass(system, cube, 0, 0);
    try std.testing.expectEqual(@as(f32, 0), try getVertexInvMass(system, cube, 0));
    try calculateMassAndInertia(system, cube);

    try setVertexVelocity(system, cube, 1, math.vec3(0, 2, 0));
    const velocity = try getVertexVelocity(system, cube, 1);
    try std.testing.expectApproxEqAbs(@as(f32, 2), velocity.y, 1.0e-4);

    // Still steps, with a pinned vertex and a hand-written velocity in it.
    _ = try system.step(dt, 1, jobs);
    try std.testing.expect(std.math.isFinite(try getVolume(system, cube)));
}

test "a vertex index nothing has is refused, not asserted" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const E = err.Error;

    // Three vertices, so 0..2 exist and 3 does not. Jolt would index its
    // vertex array with a 3 unguarded — on every skinning call and on every
    // step — so this is the one place it can still be refused.
    const settings = try SharedSettings.create();
    defer settings.release();
    try settings.addVertices(cube_vertices[0..3]);

    const past_the_end = Skinned{
        .vertex = 3,
        .weights = @splat(SkinWeight{ .inv_bind_index = 0, .weight = 0 }),
        .max_distance = 1,
        .back_stop_distance = 1,
        .back_stop_radius = 0,
    };
    try std.testing.expectError(E.InvalidArgument, settings.addSkinnedConstraints(&.{past_the_end}));

    // Refused without adding any of the batch: an in-range constraint that
    // travelled with a bad one is not half-applied.
    var in_range = past_the_end;
    in_range.vertex = 2;
    try std.testing.expectError(
        E.InvalidArgument,
        settings.addSkinnedConstraints(&.{ in_range, past_the_end }),
    );
    try settings.addSkinnedConstraints(&.{in_range});

    // The same index rule holds for the three sibling Add* calls, which reach
    // Jolt's vertex array by exactly the same route.
    try std.testing.expectError(E.InvalidArgument, settings.addFaces(&.{
        .{ .vertex = .{ 0, 1, 3 }, .material_index = 0 },
    }));
    try std.testing.expectError(E.InvalidArgument, settings.addEdges(&.{
        .{ .vertex = .{ 0, 3 }, .compliance = 0 },
    }));
    try std.testing.expectError(E.InvalidArgument, settings.addVolumeConstraints(&.{
        .{ .vertex = .{ 0, 1, 2, 3 }, .compliance = 0 },
    }));

    // A negative vertex radius is refused at creation. Jolt's own assert is on
    // the value SetVertexRadius is about to overwrite, so upstream would take
    // this one and blame the next caller.
    const cube = try buildCube();
    defer cube.release();

    const system = try system_mod.PhysicsSystem.init(.{
        .layers = system_mod.layersFromType(TestLayers),
        .max_bodies = 8,
    });
    defer system.deinit();

    try std.testing.expectError(E.InvalidArgument, create(system, .{
        .shared_settings = cube,
        .object_layer = TestLayers.moving,
        .vertex_radius = -0.01,
    }));
    try std.testing.expectError(E.InvalidArgument, create(system, .{
        .shared_settings = cube,
        .object_layer = TestLayers.moving,
        .num_iterations = 0,
    }));

    const body = try createAndAdd(system, .{
        .shared_settings = cube,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 1, 0),
    }, .activate);

    // And on the live body, at both ends of the per-vertex surface.
    try std.testing.expectEqual(@as(u32, cube_vertices.len), try countVertexStates(system, body));
    try std.testing.expectError(E.InvalidArgument, getVertexInvMass(system, body, cube_vertices.len));
    try std.testing.expectError(
        E.InvalidArgument,
        setVertexVelocity(system, body, cube_vertices.len, math.vec3(0, 1, 0)),
    );
    try std.testing.expectError(E.InvalidArgument, setVertexRadius(system, body, -1));

    // Skinning a body whose settings carry no skinned constraints at all is
    // the other half of the same class of mistake: Jolt sizes the skinning
    // state only when there is one, then asserts that it fits.
    try std.testing.expectError(E.InvalidArgument, skinVertices(system, body, &.{}, false));

    // A rigid body is not a soft body, and is told so rather than reinterpreted.
    const shape_mod = @import("shape.zig");
    const box = try shape_mod.Shape.initBox(math.vec3(0.5, 0.5, 0.5), .{});
    defer box.release();
    const rigid = try system.bodies().createAndAdd(.{
        .shape = box,
        .object_layer = TestLayers.moving,
        .motion_type = .dynamic,
        .position = math.rvec3(4, 1, 0),
    }, .activate);
    try std.testing.expectError(E.InvalidArgument, getVolume(system, rigid));
    try std.testing.expectError(E.InvalidArgument, setPressure(system, rigid, 1));

    // A body id nothing owns is a different answer again.
    try std.testing.expectError(E.BodyNotFound, getPressure(system, body_mod.invalid_body_id));
}

test "a skinned cloth follows the joint matrices it is handed each frame" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const jobs = try system_mod.JobSystem.initSingleThreaded(c.max_physics_jobs);
    defer jobs.deinit();

    const system = try system_mod.PhysicsSystem.init(.{
        .layers = system_mod.layersFromType(TestLayers),
        .max_bodies = 8,
    });
    defer system.deinit();
    system.setGravity(math.gravity_earth);

    // A flat two-triangle patch, every corner skinned to one joint. A
    // max_distance of 0 makes each of them kinematic: the solver puts the
    // vertex exactly on its skinned position and nothing else gets a say,
    // which is what lets this assert on a position rather than on a band.
    const patch = [4]Vertex{
        .{ .position = math.vec3(-0.5, 0, -0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
        .{ .position = math.vec3(0.5, 0, -0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
        .{ .position = math.vec3(0.5, 0, 0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
        .{ .position = math.vec3(-0.5, 0, 0.5), .velocity = math.vec3_zero, .inv_mass = 1 },
    };
    const patch_faces = [2]Face{
        .{ .vertex = .{ 0, 1, 2 }, .material_index = 0 },
        .{ .vertex = .{ 0, 2, 3 }, .material_index = 0 },
    };

    const settings = try SharedSettings.create();
    defer settings.release();
    try settings.addVertices(&patch);
    try settings.addFaces(&patch_faces);
    try settings.addInvBindMatrices(&.{.{ .joint_index = 0, .matrix = math.mat44_identity.m }});

    var skinned: [patch.len]Skinned = undefined;
    for (&skinned, 0..) |*sk, i| {
        sk.* = .{
            .vertex = @intCast(i),
            .weights = @splat(SkinWeight{ .inv_bind_index = 0, .weight = 0 }),
            .max_distance = 0,
            .back_stop_distance = 0,
            .back_stop_radius = 0,
        };
        sk.weights[0] = .{ .inv_bind_index = 0, .weight = 1 };
    }
    try settings.addSkinnedConstraints(&skinned);

    const attributes = [_]VertexAttributes{defaultVertexAttributes()};
    try settings.createConstraints(&attributes, .distance, std.math.degreesToRadians(8.0));
    try settings.calculateSkinnedConstraintNormals();
    settings.optimize();

    const cloth = try createAndAdd(system, .{
        .shared_settings = settings,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 5, 0),
        // The body's own origin stays put, so a vertex state — which is
        // relative to the centre of mass — reads as the offset the joint
        // asked for rather than as the offset minus wherever the body drifted.
        .update_position = false,
        .allow_sleeping = false,
    }, .activate);

    // One joint, translated a metre along +Z, expressed relative to the body's
    // centre-of-mass transform. Column-major: the translation is the fourth
    // column, so index 14 is its z.
    var joint = math.mat44_identity;
    joint.m[14] = 1;

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    var first = true;
    while (elapsed < 0.5) : (elapsed += dt) {
        // The first call is the reset: without it the skinning state has no
        // previous position to interpolate from and the first step lurches.
        try skinVertices(system, cloth, &.{joint}, first);
        first = false;
        _ = try system.step(dt, 1, jobs);
    }

    var states: [patch.len]VertexState = undefined;
    _ = try getVertexStates(system, cloth, &states);
    for (patch, states) |bind, state| {
        try std.testing.expectApproxEqAbs(bind.position.x, state.position.x, 0.02);
        try std.testing.expectApproxEqAbs(bind.position.y, state.position.y, 0.02);
        try std.testing.expectApproxEqAbs(bind.position.z + 1.0, state.position.z, 0.02);
    }

    // Gravity was on the whole time and did not win: that is the skin
    // constraints doing the work, not an absence of forces.
    try std.testing.expect(try getEnableSkinConstraints(system, cloth));
    try setSkinnedMaxDistanceMultiplier(system, cloth, 0.5);
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.5),
        try getSkinnedMaxDistanceMultiplier(system, cloth),
        1.0e-4,
    );

    // A joint index past the matrices handed over is the one thing the count
    // argument exists to catch, and Jolt only catches it by asserting.
    try std.testing.expectError(err.Error.InvalidArgument, skinVertices(system, cloth, &.{}, false));
}
