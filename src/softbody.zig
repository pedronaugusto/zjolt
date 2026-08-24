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

    /// Fails without adding any of them if one repeats a vertex.
    pub fn addFaces(self: SharedSettings, faces: []const Face) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddFaces(
            self.handle,
            faces.ptr,
            @intCast(faces.len),
        ));
    }

    /// Fails without adding any of them if one connects a vertex to itself.
    /// Rest lengths stay at Jolt's placeholder until `calculateEdgeLengths`
    /// runs.
    pub fn addEdges(self: SharedSettings, edges: []const Edge) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddEdges(
            self.handle,
            edges.ptr,
            @intCast(edges.len),
        ));
    }

    /// Fails without adding any of them if one repeats a vertex among its
    /// four.
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
    /// with, to reduce z-fighting.
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
