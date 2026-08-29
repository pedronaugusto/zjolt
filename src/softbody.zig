//! Soft bodies: cloth, rope, and jelly-like volumes simulated as a mesh of
//! particles rather than driven by a rigid shape.
//!
//! Two stages, same as Jolt's own: `SharedSettings` carries the topology
//! (vertices, faces, constraints) and is reference counted — build once,
//! stamp out many soft bodies from it; `createConstraints` derives edge,
//! shear and bend constraints from the faces automatically. `create`/
//! `createAndAdd` then place one instance in the world through
//! `BodyInterface::CreateSoftBody`, NOT `PhysicsSystem.bodies().create` —
//! but the resulting `BodyId` is ordinary afterward.
//!
//! Per step, vertices move on their own; `getVertexStates` is the bulk
//! read-back. There is no `PhysicsSystem.softBodies()` accessor: every
//! function below takes the `PhysicsSystem` it operates on directly.

const std = @import("std");
const c = @import("c/softbody.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const group_mod = @import("group.zig");
const material_mod = @import("material.zig");
const shape_mod = @import("shape.zig");
const system_mod = @import("system.zig");

pub const BendType = c.SoftBodyBendType;
pub const LraType = c.SoftBodyLraType;

pub const Vertex = c.SoftBodyVertex;
pub const Face = c.SoftBodyFace;
pub const Edge = c.SoftBodyEdge;
pub const VolumeConstraint = c.SoftBodyVolumeConstraint;
pub const RodStretchShear = c.SoftBodyRodStretchShear;
pub const RodBendTwist = c.SoftBodyRodBendTwist;
pub const RodState = c.SoftBodyRodState;
pub const InvBind = c.SoftBodyInvBind;
pub const SkinWeight = c.SoftBodySkinWeight;
pub const Skinned = c.SoftBodySkinned;
pub const VertexAttributes = c.SoftBodyVertexAttributes;
pub const VertexState = c.SoftBodyVertexState;
pub const CollideSoftBodyVertexIterator = c.CollideSoftBodyVertexIterator;

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

    /// A deep copy, reference count one, sharing nothing but material
    /// references. The only way to vary a finished topology — every other
    /// call here appends, with no remove and no reset.
    ///
    /// Inherits the original's optimisation state; re-`optimize` after
    /// adding to an already-optimised clone, same as the original needs.
    pub fn clone(self: SharedSettings) err.Error!SharedSettings {
        var handle: *c.SoftBodySharedSettings = undefined;
        try err.check(c.zjoltSoftBodySharedSettingsClone(self.handle, &handle));
        return .{ .handle = handle };
    }

    /// Replaces the material list `Face.material_index` indexes, taking a
    /// reference on each. `allocator` is used only for the call's duration.
    ///
    /// An empty list is `error.InvalidArgument` (Jolt indexes this
    /// unguarded). Pass `PhysicsMaterial.default()` for an unused slot —
    /// an existing face indexing past the new list refuses the whole call.
    pub fn setMaterials(
        self: SharedSettings,
        allocator: std.mem.Allocator,
        materials: []const material_mod.PhysicsMaterial,
    ) err.Error!void {
        // `PhysicsMaterial` wraps one pointer, so a slice of them is already
        // the array the C side wants — but that is a layout coincidence, not
        // a promise, and reinterpreting the slice would make it one.
        const handles = allocator.alloc(*const c.PhysicsMaterial, materials.len) catch
            return err.Error.OutOfMemory;
        defer allocator.free(handles);
        for (materials, 0..) |m, i| handles[i] = m.handle;

        try err.check(c.zjoltSoftBodySharedSettingsSetMaterials(
            self.handle,
            handles.ptr,
            @intCast(handles.len),
        ));
    }

    pub fn countMaterials(self: SharedSettings) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltSoftBodySharedSettingsGetMaterials(self.handle, null, 0, &count));
        return count;
    }

    /// The material list, in the order `Face.material_index` indexes it.
    /// The returned slice belongs to the caller; the materials themselves
    /// are BORROWED from the settings and live exactly as long — `addRef`
    /// one to outlive them. Settings that never had `setMaterials` called
    /// report exactly one material, Jolt's shared default.
    pub fn getMaterials(
        self: SharedSettings,
        allocator: std.mem.Allocator,
    ) err.Error![]material_mod.PhysicsMaterial {
        const count = try self.countMaterials();

        // Read into storage of the C type and copied across, for the reason
        // `setMaterials` copies the other way: a one-pointer wrapper happens
        // to share a layout with the pointer it wraps and is not promised to.
        const raw = allocator.alloc(*const c.PhysicsMaterial, count) catch
            return err.Error.OutOfMemory;
        defer allocator.free(raw);
        var got: u32 = 0;
        try err.check(c.zjoltSoftBodySharedSettingsGetMaterials(
            self.handle,
            raw.ptr,
            count,
            &got,
        ));

        const out = allocator.alloc(material_mod.PhysicsMaterial, got) catch
            return err.Error.OutOfMemory;
        for (raw[0..got], 0..) |m, i| out[i] = .{ .handle = m };
        return out;
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

    /// Appends Cosserat rods — edges that also carry an ORIENTATION;
    /// `getRodStates` reads it back to place a leaf or cable on the rod.
    /// Fails, adding none, if one connects a vertex to itself or names a
    /// vertex index past those added so far. A rod's index (for
    /// `addRodBendTwistConstraints`) is its position across all calls so far.
    /// Inert until `calculateRodProperties` runs — required, not optional.
    pub fn addRodStretchShearConstraints(self: SharedSettings, rods: []const RodStretchShear) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddRodStretchShearConstraints(
            self.handle,
            rods.ptr,
            @intCast(rods.len),
        ));
    }

    /// Appends rod bend-twist constraints, limiting how far two rods may
    /// bend or twist relative to each other — without one, a rod spins
    /// freely about its own axis, so a chain wants one between each
    /// neighbouring pair. Fails, adding none, if one names the same rod
    /// twice or a rod index past those added so far.
    pub fn addRodBendTwistConstraints(self: SharedSettings, constraints: []const RodBendTwist) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsAddRodBendTwistConstraints(
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
    /// added instead of listing every edge by hand, and calls
    /// `calculateEdgeLengths` for you. `vertex_attributes` is indexed in
    /// parallel with the vertices added so far; Jolt repeats its last
    /// element for any vertex beyond the list, so it must have at least
    /// one entry.
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

    /// Measures every volume constraint's rest volume from its four
    /// vertices' current positions. NOTHING calls this for you — unlike
    /// `createConstraints`, which never authors a volume constraint, so a
    /// tetrahedron keeps Jolt's placeholder rest volume of 1 until this
    /// runs, inflating or collapsing on creation if missed. Four coplanar
    /// vertices get a legal rest volume of zero ("keep it flat").
    pub fn calculateVolumeConstraintVolumes(self: SharedSettings) void {
        c.zjoltSoftBodySharedSettingsCalculateVolumeConstraintVolumes(self.handle);
    }

    /// Derives every rod's length, inverse mass, rest orientation, and
    /// each bend-twist constraint's rest rotation. Run once, after every
    /// rod/bend-twist constraint is added and before `optimize` — REQUIRED,
    /// not optional like `calculateEdgeLengths`: a skipped rod starts at a
    /// zero quaternion. Add bend-twist constraints FIRST; it may SWAP a
    /// rod's two vertices. `error.InvalidArgument` for coincident vertices.
    pub fn calculateRodProperties(self: SharedSettings) err.Error!void {
        try err.check(c.zjoltSoftBodySharedSettingsCalculateRodProperties(self.handle));
    }

    /// Works out which faces meet at each skinned vertex, giving it a
    /// normal — the direction `Skinned.back_stop_distance` is measured
    /// along. Neither `optimize` nor `createConstraints` does this;
    /// skipped, every skinned vertex has a zero normal, so only
    /// `max_distance` bites. Only a face with all three vertices skinned
    /// contributes. `error.InvalidArgument` past 255 such faces at one vertex.
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

/// As `create`, but the body takes `id` instead of the next one Jolt would
/// assign — see `BodyInterface.createWithId`, which this mirrors exactly,
/// `id` requirements included.
pub fn createWithId(
    system: system_mod.PhysicsSystem,
    desc: Desc,
    id: body_mod.BodyId,
) err.Error!body_mod.BodyId {
    const c_desc = desc.toC();
    var out: body_mod.BodyId = body_mod.invalid_body_id;
    try err.check(c.zjoltSoftBodyCreateWithId(system.handle, &c_desc, id, &out));
    return out;
}

/// As `createWithId`, followed immediately by adding the body.
pub fn createAndAddWithId(
    system: system_mod.PhysicsSystem,
    desc: Desc,
    id: body_mod.BodyId,
    activation: body_mod.Activation,
) err.Error!body_mod.BodyId {
    const c_desc = desc.toC();
    var out: body_mod.BodyId = body_mod.invalid_body_id;
    try err.check(c.zjoltSoftBodyCreateAndAddWithId(system.handle, &c_desc, id, activation, &out));
    return out;
}

/// The shared topology a live soft body was built from — the reference
/// `Desc.shared_settings` took at creation. Borrowed: not valid past the
/// body's own destruction. `null` for a rigid body or a stale id
/// (indistinguishable here; ask `BodyInterface.getBodyType` first).
///
/// Mutable though borrowed: mutating it reaches every OTHER body sharing this topology.
pub fn getSharedSettings(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
) ?SharedSettings {
    const handle = c.zjoltSoftBodyGetSharedSettings(system.handle, body) orelse return null;
    return .{ .handle = @constCast(handle) };
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
/// `position`/`velocity` are relative to the body's CENTER OF MASS
/// (`BodyInterface.getCenterOfMassPosition`/`.getRotation`), not world
/// space — add that position, rotated, to place a vertex in the world.
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
// Each getter/setter resolves under a lock: `error.BodyNotFound` for a
// stale id, `error.InvalidArgument` for a RIGID body. None wake a sleeping body.
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
/// Jolt's own setter asserts on the value it is about to OVERWRITE, not
/// the one being set — so upstream notices a bad radius one call late, or
/// never. The check here is on the incoming value.
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
// Only velocity and inverse mass are writable mid-simulation (a written
// position skips the solver's sweep); index past count is `error.InvalidArgument`.
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
/// inverse masses and positions. Jolt does this once, at creation, and
/// never again — a body with changed per-vertex inverse masses keeps its
/// birth mass until this runs. A single vertex with inverse mass 0 gives
/// the WHOLE body infinite mass and inertia, stopping it responding to an impulse.
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
// Each frame, hand Jolt updated joint matrices; it skins every `Skinned`
// vertex, and the solver may pull it away by at most `Skinned.max_distance`.
//=============================================================================

/// Skins every vertex carrying a `Skinned` constraint to `joint_matrices`
/// (indexed by `InvBind.joint_index`). Matrices must be RELATIVE TO THE
/// BODY'S CENTRE-OF-MASS TRANSFORM, not world space — a world-space
/// matrix is not detectably wrong, just silently offset.
///
/// `hard_skin_all` snaps every vertex to its skinned position, a reset for spawn/teleport.
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
/// Off does not stop `skinVertices`: it still hard-skins any vertex whose
/// `max_distance` is 0 and leaves the rest to simulate freely — the
/// difference between "cloth that follows the character" and "cloth
/// pinned at its kinematic vertices and otherwise loose".
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
// Rod state
//
// The read-back that makes rods worth having over edges: where a host
// collects a rod's orientation to place geometry riding on it.
//=============================================================================

pub fn countRodStates(system: system_mod.PhysicsSystem, body: body_mod.BodyId) err.Error!u32 {
    var count: u32 = 0;
    try err.check(c.zjoltSoftBodyGetRodStates(system.handle, body, null, 0, &count));
    return count;
}

/// Reads every rod into `buffer` under a single lock (size with
/// `countRodStates`; no rods reports zero). `RodState.vertex` identifies
/// WHICH rod each entry is — unordered, since `optimize` and
/// `calculateRodProperties` may reorder or swap it. `rotation` and
/// `angular_velocity` are CENTRE-OF-MASS relative, and `angular_velocity`
/// is meaningful only BETWEEN steps, not inside a contact callback.
pub fn getRodStates(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    buffer: []RodState,
) err.Error![]RodState {
    var count: u32 = 0;
    try err.check(c.zjoltSoftBodyGetRodStates(
        system.handle,
        body,
        buffer.ptr,
        @intCast(buffer.len),
        &count,
    ));
    return buffer[0..count];
}

//=============================================================================
// Hit read-back and manual update
//=============================================================================

/// Which face of `body` a sub-shape id names — turns a hit's `SubShapeId`
/// into an index into the shared settings' face list, in the order
/// `addFaces` built it (faces are never reordered, unlike constraints).
///
/// `error.InvalidArgument`, not Jolt's assert, for an id with bits left
/// over once the face index is extracted — an id from a different body's shape.
pub fn getFaceIndex(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    sub_shape_id: c.SubShapeId,
) err.Error!u32 {
    var out: u32 = 0;
    try err.check(c.zjoltSoftBodyGetFaceIndex(system.handle, body, sub_shape_id, &out));
    return out;
}

/// Runs one soft-body update immediately, on the calling thread, without
/// going through `PhysicsSystem.step` — for a body kept out of the
/// simulation on purpose; already IN the system, this updates it twice.
///
/// THREADING: takes locks on this body and everything it collides with.
/// Never call from inside a contact callback, a step listener, or while `step` runs.
pub fn customUpdate(
    system: system_mod.PhysicsSystem,
    body: body_mod.BodyId,
    delta_time: f32,
) err.Error!void {
    try err.check(c.zjoltSoftBodyCustomUpdate(system.handle, body, delta_time));
}

//=============================================================================
// Vertex-vs-shape collision
//
// What Shape::CollideSoftBodyVertices runs on: for each vertex, keep
// whichever call reports the deepest penetration. `CollideSoftBodyVertexIterator`
// is a strided view over caller-owned arrays; every field is required
// non-NULL wherever real Jolt shape code reads it (`collideSoftBodyVertices`,
// `CollideVsTriangles`), the same as the C ABI documents.
//=============================================================================

/// Resets one vertex's collision bookkeeping before a new collision pass —
/// SoftBodyVertex::ResetCollision. Pure constant writes; no Jolt call needed.
pub fn resetVertexCollision(
    largest_penetration: *f32,
    colliding_shape_index: *i32,
    has_contact: *bool,
) void {
    largest_penetration.* = -std.math.floatMax(f32);
    colliding_shape_index.* = -1;
    has_contact.* = false;
}

/// Lowest index among a constraint's own vertices —
/// SoftBodySharedSettings::Edge/Volume/RodStretchShear's own
/// GetMinVertexIndex, used while sorting constraints by locality. `vertices`
/// must be non-empty.
pub fn minVertexIndex(vertices: []const u32) u32 {
    var lowest = vertices[0];
    for (vertices[1..]) |v| lowest = @min(lowest, v);
    return lowest;
}

/// Scales `weights` so they sum to 1 — SoftBodySharedSettings::Skinned::
/// NormalizeWeights. Left alone if they already sum to 0 or less, same as
/// Jolt: an unweighted vertex stays unweighted rather than dividing by zero.
pub fn normalizeSkinWeights(weights: []SkinWeight) void {
    var total: f32 = 0;
    for (weights) |w| total += w.weight;
    if (total > 0) for (weights) |*w| {
        w.weight /= total;
    };
}

/// Runs `shape`'s own CollideSoftBodyVertices against `vertices`,
/// standalone — Shape::CollideSoftBodyVertices, dispatched directly rather
/// than through a soft body's collision pass. A soft body's own shape
/// (`Body.shape`, when it names a soft body) is a valid `shape` here too —
/// how two soft bodies collide with each other.
pub fn collideSoftBodyVertices(
    shape: shape_mod.Shape,
    center_of_mass_transform: math.Mat44,
    scale: math.Vec3,
    vertices: *const CollideSoftBodyVertexIterator,
    count: u32,
    colliding_shape_index: i32,
) err.Error!void {
    try err.check(c.zjoltShapeCollideSoftBodyVertices(
        shape.handle,
        &center_of_mass_transform,
        scale,
        vertices,
        count,
        colliding_shape_index,
    ));
}

/// Collides soft-body vertices against caller-supplied triangles —
/// CollideSoftBodyVerticesVsTriangles, the closest-point search
/// TriangleShape/MeshShape/HeightFieldShape build their own
/// CollideSoftBodyVertices from. `center_of_mass_transform`/`scale` are
/// fixed for its lifetime; `deinit` releases it.
pub const CollideVsTriangles = struct {
    handle: *c.CollideSoftBodyVerticesVsTriangles,

    pub fn init(
        center_of_mass_transform: math.Mat44,
        scale: math.Vec3,
    ) err.Error!CollideVsTriangles {
        var handle: *c.CollideSoftBodyVerticesVsTriangles = undefined;
        try err.check(c.zjoltCollideSoftBodyVerticesVsTrianglesCreate(
            &center_of_mass_transform,
            scale,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: CollideVsTriangles) void {
        c.zjoltCollideSoftBodyVerticesVsTrianglesDestroy(self.handle);
    }

    /// Begins vertex `index`'s closest-triangle search, discarding whatever
    /// `processTriangle` found for whichever vertex this last began.
    pub fn startVertex(
        self: CollideVsTriangles,
        vertex: *const CollideSoftBodyVertexIterator,
        index: u32,
    ) void {
        c.zjoltCollideSoftBodyVerticesVsTrianglesStartVertex(self.handle, vertex, index);
    }

    /// Considers one candidate triangle, LOCAL space and unscaled — kept
    /// only if closer than every triangle already seen since `startVertex`.
    pub fn processTriangle(
        self: CollideVsTriangles,
        v0: math.Vec3,
        v1: math.Vec3,
        v2: math.Vec3,
    ) void {
        c.zjoltCollideSoftBodyVerticesVsTrianglesProcessTriangle(self.handle, v0, v1, v2);
    }

    /// Commits the closest triangle found since `startVertex` into vertex
    /// `index` — a no-op unless it beats what was already recorded there.
    pub fn finishVertex(
        self: CollideVsTriangles,
        vertex: *const CollideSoftBodyVertexIterator,
        index: u32,
        colliding_shape_index: i32,
    ) void {
        c.zjoltCollideSoftBodyVerticesVsTrianglesFinishVertex(
            self.handle,
            vertex,
            index,
            colliding_shape_index,
        );
    }
};

/// One vertex's state after `markCcdContact` marked it as touching `body`.
pub const CcdContact = struct {
    collision_plane: math.Plane,
    colliding_shape_index: i32,
    has_contact: bool,
};

/// Marks one vertex as touching `body` via continuous collision detection —
/// SoftBodyVertex::MarkCCDContact. The returned `colliding_shape_index` is
/// stamped from `body`'s own id, NOT an index into a per-step
/// colliding-shape list the way a discrete contact's is.
pub fn markCcdContact(body: body_mod.BodyId, contact_plane: math.Plane) CcdContact {
    var out: CcdContact = undefined;
    c.zjoltSoftBodyVertexMarkCcdContact(
        body,
        &contact_plane,
        &out.collision_plane,
        &out.colliding_shape_index,
        &out.has_contact,
    );
    return out;
}

//=============================================================================
// Contact listener
//
// Fires as soft bodies collide with rigid ones — SEPARATE from the rigid listener in `system.zig` (silent if unset).
// Both callbacks run WITH ALL BODIES LOCKED — never call back into the system; copy what's needed and act after the step.
//=============================================================================

pub const ValidateResult = c.SoftBodyValidateResult;
pub const ContactSettings = c.SoftBodyContactSettings;
pub const VertexContact = c.SoftBodyVertexContact;
pub const ContactListener = c.SoftBodyContactListener;

/// Which vertices of a soft body touched something during one step.
///
/// A view over the solver's own arrays, not a copy: it is valid ONLY for the
/// duration of the `onSoftBodyContactAdded` call that was handed it. Keeping
/// one and reading it later reads freed solver state — nothing here hands
/// out a pointer into it.
pub const Manifold = struct {
    handle: *const c.SoftBodyManifold,

    pub fn countVertexContacts(self: Manifold) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltSoftBodyManifoldGetVertexContacts(self.handle, null, 0, &count));
        return count;
    }

    /// The vertices that touched something, and what each touched (only
    /// colliding ones — size with `countVertexContacts`, not vertex count).
    ///
    /// `normal` points from the soft body INTO what it touched (the push
    /// direction, not the pushed direction — a cloth on a floor reports
    /// down). `local_contact_point` is relative to the body's centre of mass.
    pub fn vertexContacts(
        self: Manifold,
        buffer: []VertexContact,
    ) err.Error![]VertexContact {
        var count: u32 = 0;
        try err.check(c.zjoltSoftBodyManifoldGetVertexContacts(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    pub fn countSensorContacts(self: Manifold) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltSoftBodyManifoldGetSensorContacts(self.handle, null, 0, &count));
        return count;
    }

    /// The sensors this body is overlapping, reported once per body rather
    /// than once per vertex.
    ///
    /// Installing this listener is not merely how a host HEARS about these
    /// overlaps — it is what makes them happen: Jolt skips soft-body sensor
    /// checks entirely when no soft-body contact listener is installed.
    pub fn sensorContacts(
        self: Manifold,
        buffer: []body_mod.BodyId,
    ) err.Error![]body_mod.BodyId {
        var count: u32 = 0;
        try err.check(c.zjoltSoftBodyManifoldGetSensorContacts(
            self.handle,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }
};

/// Builds a soft-body contact listener from `context` and whichever of
/// `onSoftBodyContactValidate`/`onSoftBodyContactAdded` `T` declares (an
/// omitted one never fires). `context` must outlive the system.
///
/// Validate fires on bounding-box overlap, BEFORE any vertex test; Added
/// fires once per body per step, after every contact is handled — not once per contact.
pub fn contactListener(comptime T: type, context: *T) ContactListener {
    system_mod.requireAnyDecl(T, &.{
        "onSoftBodyContactValidate", "onSoftBodyContactAdded",
    });

    const Thunks = struct {
        fn selfOf(user: ?*anyopaque) *T {
            return @ptrCast(@alignCast(user.?));
        }
        fn validate(
            user: ?*anyopaque,
            soft_body: body_mod.BodyId,
            other_body: body_mod.BodyId,
            settings: *ContactSettings,
        ) callconv(.c) ValidateResult {
            return T.onSoftBodyContactValidate(selfOf(user), soft_body, other_body, settings);
        }
        fn added(
            user: ?*anyopaque,
            soft_body: body_mod.BodyId,
            manifold: *const c.SoftBodyManifold,
        ) callconv(.c) void {
            T.onSoftBodyContactAdded(selfOf(user), soft_body, .{ .handle = manifold });
        }
    };

    return .{
        .on_contact_validate = if (@hasDecl(T, "onSoftBodyContactValidate")) Thunks.validate else null,
        .on_contact_added = if (@hasDecl(T, "onSoftBodyContactAdded")) Thunks.added else null,
        .user = @ptrCast(context),
    };
}

/// `null` clears the listener. The struct is copied, so it need not outlive
/// the call — but its `user` pointer must outlive the system.
pub fn setContactListener(
    system: system_mod.PhysicsSystem,
    listener: ?*const ContactListener,
) err.Error!void {
    try err.check(c.zjoltSoftBodySetContactListener(system.handle, listener));
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
        .{ .vertex = .{ 0, 1, 2, 3 }, .compliance = 1.0e-5 },
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

/// Collects what a soft-body contact listener is told, so a test can assert on
/// it after the step rather than from inside a callback that runs on a job
/// thread with every body locked.
///
/// Nothing here allocates, calls back into the system, or can fail — which is
/// the shape every real listener wants, for the same reasons.
const RecordingListener = struct {
    validate_calls: u32 = 0,
    added_calls: u32 = 0,
    saw_floor: bool = false,
    saw_sensor: bool = false,
    touching_vertices: u32 = 0,
    contact_body: body_mod.BodyId = body_mod.invalid_body_id,
    contact_normal: math.Vec3 = math.vec3_zero,
    contact_point: math.Vec3 = math.vec3_zero,
    soft_body_seen: body_mod.BodyId = body_mod.invalid_body_id,

    floor: body_mod.BodyId,
    sensor: body_mod.BodyId,
    reject_everything: bool = false,

    pub fn onSoftBodyContactValidate(
        self: *RecordingListener,
        soft_body: body_mod.BodyId,
        other_body: body_mod.BodyId,
        settings: *ContactSettings,
    ) ValidateResult {
        self.validate_calls += 1;
        self.soft_body_seen = soft_body;
        if (other_body == self.floor) self.saw_floor = true;
        // Left exactly as Jolt filled it, which is what a listener that only
        // wants to accept or reject is meant to be able to do.
        _ = settings;
        return if (self.reject_everything) .reject_contact else .accept_contact;
    }

    pub fn onSoftBodyContactAdded(
        self: *RecordingListener,
        soft_body: body_mod.BodyId,
        manifold: Manifold,
    ) void {
        self.added_calls += 1;
        self.soft_body_seen = soft_body;

        var contacts: [64]VertexContact = undefined;
        const touching = manifold.vertexContacts(&contacts) catch return;
        if (touching.len > self.touching_vertices) {
            self.touching_vertices = @intCast(touching.len);
        }
        for (touching) |contact| {
            if (contact.body == self.floor) {
                self.contact_body = contact.body;
                self.contact_normal = contact.normal;
                self.contact_point = contact.local_contact_point;
            }
        }

        var sensors: [8]body_mod.BodyId = undefined;
        const overlapping = manifold.sensorContacts(&sensors) catch return;
        for (overlapping) |id| {
            if (id == self.sensor) self.saw_sensor = true;
        }
    }
};

test "a soft-body contact listener reports which vertices touched what" {
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

    const floor_shape = try shape_mod.Shape.initBox(math.vec3(20, 0.5, 20), .{});
    defer floor_shape.release();
    const floor = try system.bodies().createAndAdd(.{
        .shape = floor_shape,
        .object_layer = TestLayers.static,
        .motion_type = .static,
        .position = math.rvec3(0, -0.5, 0),
    }, .dont_activate);

    // A sensor slab the cube falls through on its way down. Jolt gathers
    // sensors for a soft body ONLY when a soft-body contact listener is
    // installed, so this half of the test is not merely observed through the
    // listener — it does not happen without one.
    const sensor_shape = try shape_mod.Shape.initBox(math.vec3(3, 0.1, 3), .{});
    defer sensor_shape.release();
    const sensor = try system.bodies().createAndAdd(.{
        .shape = sensor_shape,
        .object_layer = TestLayers.static,
        .motion_type = .static,
        .position = math.rvec3(0, 2, 0),
        .is_sensor = true,
    }, .dont_activate);

    var recorder = RecordingListener{ .floor = floor, .sensor = sensor };
    const listener = contactListener(RecordingListener, &recorder);
    try setContactListener(system, &listener);

    const settings = try buildCube();
    defer settings.release();

    const cube = try createAndAdd(system, .{
        .shared_settings = settings,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 4, 0),
        .pressure = 20,
        .vertex_radius = 0.01,
        .allow_sleeping = false,
    }, .activate);

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 2.0) : (elapsed += dt) {
        _ = try system.step(dt, 1, jobs);
    }

    // The validate half fires on a bounding-box overlap, before any vertex is
    // tested, and it names the pair.
    try std.testing.expect(recorder.validate_calls > 0);
    try std.testing.expect(recorder.saw_floor);
    try std.testing.expectEqual(cube, recorder.soft_body_seen);

    // The added half fires once per body per step, and its manifold is the
    // only way to learn which vertices touched and what they touched.
    try std.testing.expect(recorder.added_calls > 0);
    try std.testing.expect(recorder.touching_vertices > 0);
    try std.testing.expect(recorder.touching_vertices <= cube_vertices.len);
    try std.testing.expectEqual(floor, recorder.contact_body);

    // Resting on the floor, so the normal points up out of it, and the
    // contact point is somewhere on the cube rather than a stale zero.
    // Pointing DOWN, into the floor: a contact normal here is the direction
    // the soft body pushes on what it touched, not the direction it is pushed.
    try std.testing.expect(recorder.contact_normal.y < -0.5);
    try std.testing.expect(std.math.isFinite(recorder.contact_point.x));
    try std.testing.expect(std.math.isFinite(recorder.contact_point.y));

    try std.testing.expect(recorder.saw_sensor);

    // And the decision the validate callback returns is a decision: rejecting
    // every contact takes the floor away, and the cube falls through it.
    const resting_y = system.bodies().getCenterOfMassPosition(cube).y;
    try std.testing.expect(resting_y > -1.0);

    recorder.reject_everything = true;
    elapsed = 0;
    while (elapsed < 1.0) : (elapsed += dt) {
        _ = try system.step(dt, 1, jobs);
    }
    try std.testing.expect(system.bodies().getCenterOfMassPosition(cube).y < resting_y - 2.0);

    // Clearing the listener is what a host does before dropping the context
    // it points at; the system must not go on calling into it.
    try setContactListener(system, null);
    const calls_before = recorder.validate_calls;
    _ = try system.step(dt, 1, jobs);
    try std.testing.expectEqual(calls_before, recorder.validate_calls);
}

/// A tetrahedron whose four faces wind outwards, six times its volume being 2
/// — deliberately not 1, which is the placeholder rest volume Jolt leaves a
/// volume constraint at.
const tet_vertices = [4]Vertex{
    .{ .position = math.vec3(0, 0, 0), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(2, 0, 0), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(0, 1, 0), .velocity = math.vec3_zero, .inv_mass = 1 },
    .{ .position = math.vec3(0, 0, 1), .velocity = math.vec3_zero, .inv_mass = 1 },
};

const tet_faces = [4]Face{
    .{ .vertex = .{ 0, 2, 1 }, .material_index = 0 },
    .{ .vertex = .{ 0, 3, 2 }, .material_index = 0 },
    .{ .vertex = .{ 0, 1, 3 }, .material_index = 0 },
    .{ .vertex = .{ 1, 2, 3 }, .material_index = 0 },
};

fn buildTetrahedron(measure_rest_volume: bool) err.Error!SharedSettings {
    const settings = try SharedSettings.create();
    errdefer settings.release();
    try settings.addVertices(&tet_vertices);
    try settings.addFaces(&tet_faces);
    try settings.addVolumeConstraints(&.{
        .{ .vertex = .{ 0, 1, 2, 3 }, .compliance = 1.0e-5 },
    });
    if (measure_rest_volume) settings.calculateVolumeConstraintVolumes();
    settings.optimize();
    return settings;
}

test "a volume constraint holds the volume it was measured at, not Jolt's placeholder" {
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
    // No gravity: the only thing that can move a vertex here is the volume
    // constraint, which is what makes the two numbers below attributable.
    system.setGravity(math.vec3_zero);

    const measured = try buildTetrahedron(true);
    defer measured.release();
    const unmeasured = try buildTetrahedron(false);
    defer unmeasured.release();

    const with = try createAndAdd(system, .{
        .shared_settings = measured,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 0, 0),
        .allow_sleeping = false,
    }, .activate);
    const without = try createAndAdd(system, .{
        .shared_settings = unmeasured,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(20, 0, 0),
        .allow_sleeping = false,
    }, .activate);

    // Both start as the same tetrahedron: eight sixths of a unit volume.
    const rest_volume: f32 = 2.0 / 6.0;
    try std.testing.expectApproxEqAbs(rest_volume, try getVolume(system, with), 1.0e-3);
    try std.testing.expectApproxEqAbs(rest_volume, try getVolume(system, without), 1.0e-3);

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 1.0) : (elapsed += dt) {
        _ = try system.step(dt, 1, jobs);
    }

    // Measured, the constraint is already satisfied and nothing moves.
    try std.testing.expectApproxEqAbs(rest_volume, try getVolume(system, with), 0.05);

    // Unmeasured, it is not: Jolt's placeholder is a SIX-times-volume of 1, so
    // the solver spends every step crushing the tetrahedron until its volume
    // is a sixth of a unit — half what it was authored at, from a call that
    // was never made. That number is the placeholder itself, not a wobble.
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0 / 6.0),
        try getVolume(system, without),
        0.02,
    );
}

test "a rod's rotation tracks the direction between its two vertices" {
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

    // A rope laid out along +X with its first vertex pinned, so gravity has
    // to swing it: a rope that started vertical would never turn, and the
    // whole point here is that the rotations FOLLOW the vertices.
    const rope = [4]Vertex{
        .{ .position = math.vec3(0, 0, 0), .velocity = math.vec3_zero, .inv_mass = 0 },
        .{ .position = math.vec3(1, 0, 0), .velocity = math.vec3_zero, .inv_mass = 1 },
        .{ .position = math.vec3(2, 0, 0), .velocity = math.vec3_zero, .inv_mass = 1 },
        .{ .position = math.vec3(3, 0, 0), .velocity = math.vec3_zero, .inv_mass = 1 },
    };

    const settings = try SharedSettings.create();
    defer settings.release();
    try settings.addVertices(&rope);
    try settings.addRodStretchShearConstraints(&.{
        .{ .vertex = .{ 0, 1 }, .compliance = 0 },
        .{ .vertex = .{ 1, 2 }, .compliance = 0 },
        .{ .vertex = .{ 2, 3 }, .compliance = 0 },
    });
    // Without these, each rod is free to spin about its own axis at constant
    // velocity, and they are also what tells calculateRodProperties which
    // rods are neighbours.
    try settings.addRodBendTwistConstraints(&.{
        .{ .rod = .{ 0, 1 }, .compliance = 0.01 },
        .{ .rod = .{ 1, 2 }, .compliance = 0.01 },
    });
    try settings.calculateRodProperties();
    settings.optimize();

    const strand = try createAndAdd(system, .{
        .shared_settings = settings,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 5, 0),
        .update_position = false,
        .allow_sleeping = false,
    }, .activate);

    try std.testing.expectEqual(@as(u32, 3), try countRodStates(system, strand));

    var elapsed: f32 = 0;
    const dt: f32 = 1.0 / 60.0;
    while (elapsed < 1.0) : (elapsed += dt) {
        _ = try system.step(dt, 1, jobs);
    }

    var states: [3]RodState = undefined;
    const rods = try getRodStates(system, strand, &states);
    try std.testing.expectEqual(@as(usize, 3), rods.len);

    var vertices: [rope.len]VertexState = undefined;
    _ = try getVertexStates(system, strand, &vertices);

    var swung = false;
    for (rods) |rod| {
        const a = vertices[rod.vertex[0]].position;
        const b = vertices[rod.vertex[1]].position;
        const delta = math.vec3(b.x - a.x, b.y - a.y, b.z - a.z);
        const length = @sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        try std.testing.expect(length > 0.5);

        // The rod's rotation takes its local +Z onto the direction between
        // its two vertices — that is the whole contract of a Cosserat rod's
        // orientation, and it is what makes it usable for placing geometry.
        const tangent = try rod.rotation.rotateVector(math.vec3(0, 0, 1));
        try std.testing.expectApproxEqAbs(delta.x / length, tangent.x, 0.05);
        try std.testing.expectApproxEqAbs(delta.y / length, tangent.y, 0.05);
        try std.testing.expectApproxEqAbs(delta.z / length, tangent.z, 0.05);

        if (@abs(delta.y / length) > 0.3) swung = true;
    }

    // And the rope really did swing, so the rotations above tracked a change
    // rather than agreeing with a pose nothing disturbed.
    try std.testing.expect(swung);

    // The rope kept its length: rods are inextensible where a spring is not.
    const tip = vertices[3].position;
    const root = vertices[0].position;
    const span = @sqrt((tip.x - root.x) * (tip.x - root.x) +
        (tip.y - root.y) * (tip.y - root.y) +
        (tip.z - root.z) * (tip.z - root.z));
    try std.testing.expect(span > 1.5);
    try std.testing.expect(span < 3.2);

    // A body without rods answers zero rather than failing, which is what
    // lets a host ask unconditionally.
    const cube_settings = try buildCube();
    defer cube_settings.release();
    const cube = try createAndAdd(system, .{
        .shared_settings = cube_settings,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(20, 5, 0),
    }, .activate);
    try std.testing.expectEqual(@as(u32, 0), try countRodStates(system, cube));

    // A rod of zero length has no direction to build a frame from, and is
    // refused rather than left to produce a NaN quaternion that every rod
    // downstream of it in the chain would inherit.
    const degenerate = try SharedSettings.create();
    defer degenerate.release();
    try degenerate.addVertices(&.{
        .{ .position = math.vec3(0, 0, 0), .velocity = math.vec3_zero, .inv_mass = 1 },
        .{ .position = math.vec3(0, 0, 0), .velocity = math.vec3_zero, .inv_mass = 1 },
    });
    try degenerate.addRodStretchShearConstraints(&.{.{ .vertex = .{ 0, 1 }, .compliance = 0 }});
    try std.testing.expectError(err.Error.InvalidArgument, degenerate.calculateRodProperties());

    // And the index checks on the two authoring calls, which stand between a
    // caller and an unguarded index into Jolt's own arrays.
    try std.testing.expectError(
        err.Error.InvalidArgument,
        degenerate.addRodStretchShearConstraints(&.{.{ .vertex = .{ 0, 2 }, .compliance = 0 }}),
    );
    try std.testing.expectError(
        err.Error.InvalidArgument,
        degenerate.addRodStretchShearConstraints(&.{.{ .vertex = .{ 1, 1 }, .compliance = 0 }}),
    );
    try std.testing.expectError(
        err.Error.InvalidArgument,
        degenerate.addRodBendTwistConstraints(&.{.{ .rod = .{ 0, 1 }, .compliance = 0 }}),
    );
    try std.testing.expectError(
        err.Error.InvalidArgument,
        degenerate.addRodBendTwistConstraints(&.{.{ .rod = .{ 0, 0 }, .compliance = 0 }}),
    );
}

test "a cloned topology is independent of the one it was copied from" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const material_lib = @import("material.zig");

    const original = try buildCube();
    defer original.release();
    try std.testing.expectEqual(@as(u32, 1), original.refCount());

    const copy = try original.clone();
    defer copy.release();

    // A fresh reference count, not a share of the original's.
    try std.testing.expectEqual(@as(u32, 1), copy.refCount());
    try std.testing.expectEqual(@as(u32, 1), original.refCount());

    // Both carry Jolt's single shared default material until told otherwise.
    try std.testing.expectEqual(@as(u32, 1), try original.countMaterials());
    try std.testing.expectEqual(@as(u32, 1), try copy.countMaterials());

    const canvas = try material_lib.PhysicsMaterial.init(.{ .debug_name = "canvas" });
    defer canvas.release();
    const rope = try material_lib.PhysicsMaterial.init(.{ .debug_name = "rope" });
    defer rope.release();

    try copy.setMaterials(std.testing.allocator, &.{ canvas, rope });

    // Changing the copy left the original where it was, which is the whole
    // reason to clone rather than to keep adding to one settings object.
    try std.testing.expectEqual(@as(u32, 2), try copy.countMaterials());
    try std.testing.expectEqual(@as(u32, 1), try original.countMaterials());

    const listed = try copy.getMaterials(std.testing.allocator);
    defer std.testing.allocator.free(listed);
    try std.testing.expectEqual(@as(usize, 2), listed.len);
    try std.testing.expect(listed[0].eql(canvas));
    try std.testing.expect(listed[1].eql(rope));

    // And the copy is a whole topology, not a header: a body built from it
    // has the original's geometry.
    const system = try system_mod.PhysicsSystem.init(.{
        .layers = system_mod.layersFromType(TestLayers),
        .max_bodies = 8,
    });
    defer system.deinit();

    const body = try createAndAdd(system, .{
        .shared_settings = copy,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 1, 0),
    }, .dont_activate);
    try std.testing.expectEqual(@as(u32, cube_vertices.len), try countVertexStates(system, body));
    try std.testing.expectApproxEqAbs(@as(f32, 1), try getVolume(system, body), 0.05);

    // An empty material list is refused rather than accepted as a clear: it
    // would be an out-of-bounds read for every face already added.
    try std.testing.expectError(
        err.Error.InvalidArgument,
        copy.setMaterials(std.testing.allocator, &.{}),
    );
    try std.testing.expectEqual(@as(u32, 2), try copy.countMaterials());
}

test "a ray cast at a soft body names the face it hit and the material on it" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const material_lib = @import("material.zig");
    const query_mod = @import("query.zig");

    const system = try system_mod.PhysicsSystem.init(.{
        .layers = system_mod.layersFromType(TestLayers),
        .max_bodies = 8,
    });
    defer system.deinit();

    const hull = try material_lib.PhysicsMaterial.init(.{ .debug_name = "hull" });
    defer hull.release();
    const lid = try material_lib.PhysicsMaterial.init(.{ .debug_name = "lid" });
    defer lid.release();

    // The cube's faces 2 and 3 are its top, so those two get the second
    // material and everything else keeps the first.
    var faces = cube_faces;
    faces[2].material_index = 1;
    faces[3].material_index = 1;

    const settings = try SharedSettings.create();
    defer settings.release();
    try settings.addVertices(&cube_vertices);
    try settings.setMaterials(std.testing.allocator, &.{ hull, lid });
    try settings.addFaces(&faces);
    const attributes = [_]VertexAttributes{defaultVertexAttributes()};
    try settings.createConstraints(&attributes, .dihedral, std.math.degreesToRadians(8.0));
    settings.optimize();

    const cube = try createAndAdd(system, .{
        .shared_settings = settings,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 0, 0),
    }, .activate);

    // Straight down the middle from above, so the first thing in the way is
    // one of the two top faces.
    const hit = (try system.queries().castRayClosest(
        math.rvec3(0, 5, 0),
        math.vec3(0, -10, 0),
        null,
        null,
    )) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(cube, hit.body);

    // The sub shape id a hit carries is opaque until this turns it into the
    // index of a face the host itself added.
    const face = try getFaceIndex(system, cube, hit.sub_shape_id);
    try std.testing.expect(face == 2 or face == 3);

    // And the material that comes back is the one assigned to that face,
    // which is the only reason a soft body has a material list at all.
    const material = hit.material orelse return error.TestUnexpectedResult;
    try std.testing.expect(material_lib.PhysicsMaterial.eql(.{ .handle = material }, lid));

    // A sub shape id that names nothing in this body is refused rather than
    // decoded into a plausible-looking face index. A rigid body's shape hands
    // out the empty id, which is all ones.
    const box = try shape_mod.Shape.initBox(math.vec3(0.5, 0.5, 0.5), .{});
    defer box.release();
    const rigid = try system.bodies().createAndAdd(.{
        .shape = box,
        .object_layer = TestLayers.moving,
        .motion_type = .dynamic,
        .position = math.rvec3(10, 0, 0),
    }, .dont_activate);
    try std.testing.expectError(
        err.Error.InvalidArgument,
        getFaceIndex(system, cube, query_mod.empty_sub_shape_id),
    );
    try std.testing.expectError(
        err.Error.InvalidArgument,
        getFaceIndex(system, rigid, hit.sub_shape_id),
    );
}

test "customUpdate advances a soft body with no step at all" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try system_mod.PhysicsSystem.init(.{
        .layers = system_mod.layersFromType(TestLayers),
        .max_bodies = 8,
    });
    defer system.deinit();
    system.setGravity(math.gravity_earth);

    const settings = try buildCube();
    defer settings.release();

    const cube = try createAndAdd(system, .{
        .shared_settings = settings,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 10, 0),
        .allow_sleeping = false,
    }, .activate);

    const before = system.bodies().getCenterOfMassPosition(cube);

    // No job system, no PhysicsSystem.step: this is the whole point. A host
    // that has just teleported a cloth onto a character settles it like this,
    // before anything sees the pose it teleported into.
    var i: u32 = 0;
    while (i < 30) : (i += 1) {
        try customUpdate(system, cube, 1.0 / 60.0);
    }

    const after = system.bodies().getCenterOfMassPosition(cube);
    try std.testing.expect(after.y < before.y - 0.5);

    // Half a second of free fall is about 1.2 m, and nothing was in the way.
    try std.testing.expect(after.y > before.y - 2.0);

    // A delta of zero would make every sub-step infinite, and a negative one
    // would run the solver backwards. Both are refused.
    try std.testing.expectError(err.Error.InvalidArgument, customUpdate(system, cube, 0));
    try std.testing.expectError(err.Error.InvalidArgument, customUpdate(system, cube, -0.1));

    // And a rigid body has no soft-body update to run.
    const box = try shape_mod.Shape.initBox(math.vec3(0.5, 0.5, 0.5), .{});
    defer box.release();
    const rigid = try system.bodies().createAndAdd(.{
        .shape = box,
        .object_layer = TestLayers.moving,
        .motion_type = .dynamic,
        .position = math.rvec3(10, 1, 0),
    }, .activate);
    try std.testing.expectError(err.Error.InvalidArgument, customUpdate(system, rigid, 1.0 / 60.0));
    try std.testing.expectError(
        err.Error.BodyNotFound,
        customUpdate(system, body_mod.invalid_body_id, 1.0 / 60.0),
    );
}

test "updatePenetration records only the deepest value seen so far" {
    var largest_penetration = [1]f32{-std.math.floatMax(f32)};
    const it: CollideSoftBodyVertexIterator = .{
        .largest_penetration = @ptrCast(&largest_penetration),
        .largest_penetration_stride = @sizeOf(f32),
    };
    try std.testing.expect(it.updatePenetration(0, 0.1));
    try std.testing.expectEqual(@as(f32, 0.1), largest_penetration[0]);

    // A shallower write loses: the deepest one already recorded stands.
    try std.testing.expect(!it.updatePenetration(0, 0.05));
    try std.testing.expectEqual(@as(f32, 0.1), largest_penetration[0]);

    try std.testing.expect(it.updatePenetration(0, 0.2));
    try std.testing.expectEqual(@as(f32, 0.2), largest_penetration[0]);
}

test "updatePenetration is false with no write when largest_penetration is unset" {
    const it: CollideSoftBodyVertexIterator = .{};
    try std.testing.expect(!it.updatePenetration(0, 1.0));
}

test "resetVertexCollision resets to Jolt's own sentinel values" {
    var largest_penetration: f32 = 5.0;
    var colliding_shape_index: i32 = 3;
    var has_contact = true;
    resetVertexCollision(&largest_penetration, &colliding_shape_index, &has_contact);
    try std.testing.expectEqual(-std.math.floatMax(f32), largest_penetration);
    try std.testing.expectEqual(@as(i32, -1), colliding_shape_index);
    try std.testing.expect(!has_contact);
}

test "minVertexIndex matches what Edge/Volume/RodStretchShear compute by hand" {
    const edge = Edge{ .vertex = .{ 7, 2 }, .compliance = 0 };
    try std.testing.expectEqual(@as(u32, 2), minVertexIndex(&edge.vertex));

    const volume = VolumeConstraint{ .vertex = .{ 5, 1, 9, 3 }, .compliance = 0 };
    try std.testing.expectEqual(@as(u32, 1), minVertexIndex(&volume.vertex));

    const rod = RodStretchShear{ .vertex = .{ 4, 4 }, .compliance = 0 };
    try std.testing.expectEqual(@as(u32, 4), minVertexIndex(&rod.vertex));
}

test "normalizeSkinWeights sums to 1, and leaves an unweighted vertex at 0" {
    var weights = [4]SkinWeight{
        .{ .inv_bind_index = 0, .weight = 1 },
        .{ .inv_bind_index = 1, .weight = 3 },
        .{ .inv_bind_index = 2, .weight = 0 },
        .{ .inv_bind_index = 3, .weight = 0 },
    };
    normalizeSkinWeights(&weights);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), weights[0].weight, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), weights[1].weight, 1e-6);
    try std.testing.expectEqual(@as(f32, 0), weights[2].weight);

    var unweighted = [4]SkinWeight{
        .{ .inv_bind_index = 0, .weight = 0 },
        .{ .inv_bind_index = 0, .weight = 0 },
        .{ .inv_bind_index = 0, .weight = 0 },
        .{ .inv_bind_index = 0, .weight = 0 },
    };
    normalizeSkinWeights(&unweighted);
    for (unweighted) |w| try std.testing.expectEqual(@as(f32, 0), w.weight);
}

test "collideSoftBodyVertices against a box matches BoxShape's own math" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const box = try shape_mod.Shape.initBox(math.vec3(1, 1, 1), .{});
    defer box.release();

    // Inside the box, closest to the top face: half_extent.y (1) minus the
    // vertex's own height (0.4) is the expected penetration.
    var positions = [1]math.Vec3{math.vec3(0, 0.4, 0)};
    var inv_masses = [1]f32{1};
    var planes = [1]math.Plane{.{ .normal = math.vec3_zero, .constant = 0 }};
    var largest_penetration = [1]f32{-std.math.floatMax(f32)};
    var colliding_shape_index = [1]i32{-1};
    const vertices: CollideSoftBodyVertexIterator = .{
        .position = @ptrCast(&positions),
        .position_stride = @sizeOf(math.Vec3),
        .inv_mass = @ptrCast(&inv_masses),
        .inv_mass_stride = @sizeOf(f32),
        .collision_plane = @ptrCast(&planes),
        .collision_plane_stride = @sizeOf(math.Plane),
        .largest_penetration = @ptrCast(&largest_penetration),
        .largest_penetration_stride = @sizeOf(f32),
        .colliding_shape_index = @ptrCast(&colliding_shape_index),
        .colliding_shape_index_stride = @sizeOf(i32),
    };

    try collideSoftBodyVertices(box, math.mat44_identity, math.vec3(1, 1, 1), &vertices, 1, 42);
    try std.testing.expectApproxEqAbs(@as(f32, 0.6), largest_penetration[0], 1e-5);
    try std.testing.expectEqual(@as(i32, 42), colliding_shape_index[0]);
    try std.testing.expectApproxEqAbs(@as(f32, 0), planes[0].normal.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), planes[0].normal.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), planes[0].normal.z, 1e-5);

    // A vertex with no mass is skipped outright, the same as Jolt's own
    // built-in shapes skip a pinned vertex: nothing is written for it.
    inv_masses[0] = 0;
    largest_penetration[0] = -std.math.floatMax(f32);
    colliding_shape_index[0] = -1;
    try collideSoftBodyVertices(box, math.mat44_identity, math.vec3(1, 1, 1), &vertices, 1, 42);
    try std.testing.expectEqual(-std.math.floatMax(f32), largest_penetration[0]);
    try std.testing.expectEqual(@as(i32, -1), colliding_shape_index[0]);
}

test "collideSoftBodyVertices dispatches into a soft body's own SoftBodyShape" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const system = try system_mod.PhysicsSystem.init(.{
        .layers = system_mod.layersFromType(TestLayers),
        .max_bodies = 8,
    });
    defer system.deinit();

    const settings = try buildCube();
    defer settings.release();
    const cube = try createAndAdd(system, .{
        .shared_settings = settings,
        .object_layer = TestLayers.moving,
        .position = math.rvec3(0, 5, 0),
    }, .activate);

    var lock = system.lockRead(cube);
    defer lock.release();
    const cube_shape = lock.body().?.shape().?;

    // A soft body's own CollideSoftBodyVertices is Jolt's own no-op — soft
    // bodies do not collide against each other's vertices in this Jolt
    // version — so a real dispatch through a valid vtable leaves every
    // field exactly as it was, rather than crashing or writing garbage.
    var positions = [1]math.Vec3{math.vec3(0, 0, 0)};
    var inv_masses = [1]f32{1};
    var planes = [1]math.Plane{.{ .normal = math.vec3(0, 1, 0), .constant = 7 }};
    var largest_penetration = [1]f32{-1.0};
    var colliding_shape_index = [1]i32{-1};
    const vertices: CollideSoftBodyVertexIterator = .{
        .position = @ptrCast(&positions),
        .position_stride = @sizeOf(math.Vec3),
        .inv_mass = @ptrCast(&inv_masses),
        .inv_mass_stride = @sizeOf(f32),
        .collision_plane = @ptrCast(&planes),
        .collision_plane_stride = @sizeOf(math.Plane),
        .largest_penetration = @ptrCast(&largest_penetration),
        .largest_penetration_stride = @sizeOf(f32),
        .colliding_shape_index = @ptrCast(&colliding_shape_index),
        .colliding_shape_index_stride = @sizeOf(i32),
    };
    try collideSoftBodyVertices(cube_shape, math.mat44_identity, math.vec3(1, 1, 1), &vertices, 1, 9);
    try std.testing.expectEqual(@as(f32, -1.0), largest_penetration[0]);
    try std.testing.expectEqual(@as(i32, -1), colliding_shape_index[0]);
    try std.testing.expectEqual(@as(f32, 7), planes[0].constant);
}

test "CollideVsTriangles keeps the closest triangle a vertex was tested against" {
    const zjolt = @import("zjolt.zig");
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const collider = try CollideVsTriangles.init(math.mat44_identity, math.vec3(1, 1, 1));
    defer collider.deinit();

    var positions = [1]math.Vec3{math.vec3(0, 0.3, 0)};
    var planes = [1]math.Plane{.{ .normal = math.vec3_zero, .constant = 0 }};
    var largest_penetration = [1]f32{-std.math.floatMax(f32)};
    var colliding_shape_index = [1]i32{-1};
    const vertex: CollideSoftBodyVertexIterator = .{
        .position = @ptrCast(&positions),
        .position_stride = @sizeOf(math.Vec3),
        .collision_plane = @ptrCast(&planes),
        .collision_plane_stride = @sizeOf(math.Plane),
        .largest_penetration = @ptrCast(&largest_penetration),
        .largest_penetration_stride = @sizeOf(f32),
        .colliding_shape_index = @ptrCast(&colliding_shape_index),
        .colliding_shape_index_stride = @sizeOf(i32),
    };

    collider.startVertex(&vertex, 0);
    // A large CCW (seen from +Y) triangle in the y=0 plane, well under the
    // vertex: closest point is its interior, straight down.
    collider.processTriangle(math.vec3(-10, 0, -10), math.vec3(0, 0, 10), math.vec3(10, 0, -10));
    // A second, farther-away candidate must lose to the first.
    collider.processTriangle(math.vec3(-10, -5, -10), math.vec3(0, -5, 10), math.vec3(10, -5, -10));
    collider.finishVertex(&vertex, 0, 3);

    try std.testing.expectApproxEqAbs(@as(f32, -0.3), largest_penetration[0], 1e-5);
    try std.testing.expectEqual(@as(i32, 3), colliding_shape_index[0]);
    try std.testing.expectApproxEqAbs(@as(f32, 0), planes[0].normal.x, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), planes[0].normal.y, 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), planes[0].constant, 1e-5);
}

test "markCcdContact stamps the given plane and a body-derived shape index" {
    const plane = math.Plane{ .normal = math.vec3(0, 1, 0), .constant = -2 };
    const result = markCcdContact(77, plane);
    try std.testing.expectEqual(plane.normal.x, result.collision_plane.normal.x);
    try std.testing.expectEqual(plane.normal.y, result.collision_plane.normal.y);
    try std.testing.expectEqual(plane.normal.z, result.collision_plane.normal.z);
    try std.testing.expectEqual(plane.constant, result.collision_plane.constant);
    try std.testing.expect(result.has_contact);

    // Two different bodies stamp two different shape indices.
    const other = markCcdContact(78, plane);
    try std.testing.expect(other.colliding_shape_index != result.colliding_shape_index);
}
