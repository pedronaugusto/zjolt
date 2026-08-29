//! ZJolt C declarations for soft bodies, their shared settings and their
//! per-instance state.
//!
//! Mirrors `ffi/zjolt_softbody.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide and
//! nothing to drift. `src/c.zig` lists every one of these and is what the ABI
//! cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");
const group = @import("group.zig");
const shape = @import("shape.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const AABox = core.AABox;
pub const Activation = core.Activation;
pub const BodyId = core.BodyId;
pub const Mat44 = core.Mat44;
pub const ObjectLayer = core.ObjectLayer;
pub const PhysicsMaterial = core.PhysicsMaterial;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Plane = shape.Plane;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Shape = core.Shape;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
pub const max_physics_jobs = core.max_physics_jobs;
pub const CollisionGroup = group.CollisionGroup;

pub const SoftBodySharedSettings = opaque {};

pub const SoftBodyVertex = extern struct {
    position: Vec3,
    velocity: Vec3,
    inv_mass: f32,
};

pub const SoftBodyFace = extern struct {
    vertex: [3]u32,
    material_index: u32,
};

pub const SoftBodyEdge = extern struct {
    vertex: [2]u32,
    compliance: f32,
};

pub const SoftBodyVolumeConstraint = extern struct {
    vertex: [4]u32,
    compliance: f32,
};

pub const SoftBodyInvBind = extern struct {
    joint_index: u32,
    /// Jolt's own Mat44 layout: four columns of four floats each.
    matrix: [16]f32,
};

pub const SoftBodySkinWeight = extern struct {
    inv_bind_index: u32,
    weight: f32,
};

pub const SoftBodySkinned = extern struct {
    vertex: u32,
    weights: [4]SoftBodySkinWeight,
    max_distance: f32,
    back_stop_distance: f32,
    back_stop_radius: f32,
};

pub const SoftBodyBendType = enum(c_int) {
    none = 0,
    distance = 1,
    dihedral = 2,
};

pub const SoftBodyLraType = enum(c_int) {
    none = 0,
    euclidean_distance = 1,
    geodesic_distance = 2,
};

pub const SoftBodyVertexAttributes = extern struct {
    compliance: f32,
    shear_compliance: f32,
    bend_compliance: f32,
    lra_type: SoftBodyLraType,
    lra_max_distance_multiplier: f32,
};

pub const SoftBodyDesc = extern struct {
    shared_settings: ?*const SoftBodySharedSettings,
    collision_group: CollisionGroup,
    position: RVec3,
    rotation: Quat,
    user_data: u64,
    object_layer: ObjectLayer,
    num_iterations: u32,
    linear_damping: f32,
    max_linear_velocity: f32,
    restitution: f32,
    friction: f32,
    pressure: f32,
    gravity_factor: f32,
    vertex_radius: f32,
    update_position: bool,
    make_rotation_identity: bool,
    allow_sleeping: bool,
    faces_double_sided: bool,
};

pub const SoftBodyVertexState = extern struct {
    position: Vec3,
    velocity: Vec3,
};

pub extern fn zjoltSoftBodyVertexAttributesInit(out: *SoftBodyVertexAttributes) void;

pub extern fn zjoltSoftBodySharedSettingsCreate(out: **SoftBodySharedSettings) Result;

pub extern fn zjoltSoftBodySharedSettingsAddRef(settings: *const SoftBodySharedSettings) void;

pub extern fn zjoltSoftBodySharedSettingsRelease(settings: *const SoftBodySharedSettings) void;

pub extern fn zjoltSoftBodySharedSettingsGetRefCount(settings: *const SoftBodySharedSettings) u32;

pub extern fn zjoltSoftBodySharedSettingsAddVertices(settings: *SoftBodySharedSettings, vertices: ?[*]const SoftBodyVertex, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsAddFaces(settings: *SoftBodySharedSettings, faces: ?[*]const SoftBodyFace, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsAddEdges(settings: *SoftBodySharedSettings, edges: ?[*]const SoftBodyEdge, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsAddVolumeConstraints(settings: *SoftBodySharedSettings, constraints: ?[*]const SoftBodyVolumeConstraint, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsAddInvBindMatrices(settings: *SoftBodySharedSettings, inv_binds: ?[*]const SoftBodyInvBind, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsAddSkinnedConstraints(settings: *SoftBodySharedSettings, constraints: ?[*]const SoftBodySkinned, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsCreateConstraints(settings: *SoftBodySharedSettings, vertex_attributes: ?[*]const SoftBodyVertexAttributes, vertex_attributes_count: u32, bend_type: SoftBodyBendType, angle_tolerance_radians: f32) Result;

pub extern fn zjoltSoftBodySharedSettingsCalculateEdgeLengths(settings: *SoftBodySharedSettings) void;

pub extern fn zjoltSoftBodySharedSettingsCalculateSkinnedConstraintNormals(settings: *SoftBodySharedSettings) Result;

pub extern fn zjoltSoftBodySharedSettingsOptimize(settings: *SoftBodySharedSettings) void;

pub extern fn zjoltSoftBodyDescInit(desc: *SoftBodyDesc) void;

pub extern fn zjoltSoftBodyCreate(system: *PhysicsSystem, desc: *const SoftBodyDesc, out: *BodyId) Result;

pub extern fn zjoltSoftBodyCreateAndAdd(system: *PhysicsSystem, desc: *const SoftBodyDesc, activation: Activation, out: *BodyId) Result;

pub extern fn zjoltSoftBodyCreateWithId(system: *PhysicsSystem, desc: *const SoftBodyDesc, id: BodyId, out: *BodyId) Result;

pub extern fn zjoltSoftBodyCreateAndAddWithId(system: *PhysicsSystem, desc: *const SoftBodyDesc, id: BodyId, activation: Activation, out: *BodyId) Result;

pub extern fn zjoltSoftBodyGetSharedSettings(system: *const PhysicsSystem, body: BodyId) ?*const SoftBodySharedSettings;

pub extern fn zjoltSoftBodyGetVertexStates(system: *const PhysicsSystem, body: BodyId, out_states: ?[*]SoftBodyVertexState, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltSoftBodyGetNumIterations(system: *const PhysicsSystem, body: BodyId, out: *u32) Result;

pub extern fn zjoltSoftBodySetNumIterations(system: *PhysicsSystem, body: BodyId, num_iterations: u32) Result;

pub extern fn zjoltSoftBodyGetPressure(system: *const PhysicsSystem, body: BodyId, out: *f32) Result;

pub extern fn zjoltSoftBodySetPressure(system: *PhysicsSystem, body: BodyId, pressure: f32) Result;

pub extern fn zjoltSoftBodyGetUpdatePosition(system: *const PhysicsSystem, body: BodyId, out: *bool) Result;

pub extern fn zjoltSoftBodySetUpdatePosition(system: *PhysicsSystem, body: BodyId, update_position: bool) Result;

pub extern fn zjoltSoftBodyGetFacesDoubleSided(system: *const PhysicsSystem, body: BodyId, out: *bool) Result;

pub extern fn zjoltSoftBodySetFacesDoubleSided(system: *PhysicsSystem, body: BodyId, double_sided: bool) Result;

pub extern fn zjoltSoftBodyGetVertexRadius(system: *const PhysicsSystem, body: BodyId, out: *f32) Result;

pub extern fn zjoltSoftBodySetVertexRadius(system: *PhysicsSystem, body: BodyId, vertex_radius: f32) Result;

pub extern fn zjoltSoftBodyGetVertexVelocity(system: *const PhysicsSystem, body: BodyId, index: u32, out: *Vec3) Result;

pub extern fn zjoltSoftBodySetVertexVelocity(system: *PhysicsSystem, body: BodyId, index: u32, velocity: *const Vec3) Result;

pub extern fn zjoltSoftBodyGetVertexInvMass(system: *const PhysicsSystem, body: BodyId, index: u32, out: *f32) Result;

pub extern fn zjoltSoftBodySetVertexInvMass(system: *PhysicsSystem, body: BodyId, index: u32, inv_mass: f32) Result;

pub extern fn zjoltSoftBodyCalculateMassAndInertia(system: *PhysicsSystem, body: BodyId) Result;

pub extern fn zjoltSoftBodyGetVolume(system: *const PhysicsSystem, body: BodyId, out: *f32) Result;

pub extern fn zjoltSoftBodyGetLocalBounds(system: *const PhysicsSystem, body: BodyId, out: *AABox) Result;

pub extern fn zjoltSoftBodySkinVertices(system: *PhysicsSystem, body: BodyId, joint_matrices: ?[*]const Mat44, joint_count: u32, hard_skin_all: bool) Result;

pub extern fn zjoltSoftBodyGetEnableSkinConstraints(system: *const PhysicsSystem, body: BodyId, out: *bool) Result;

pub extern fn zjoltSoftBodySetEnableSkinConstraints(system: *PhysicsSystem, body: BodyId, enable: bool) Result;

pub extern fn zjoltSoftBodyGetSkinnedMaxDistanceMultiplier(system: *const PhysicsSystem, body: BodyId, out: *f32) Result;

pub extern fn zjoltSoftBodySetSkinnedMaxDistanceMultiplier(system: *PhysicsSystem, body: BodyId, multiplier: f32) Result;

pub const SoftBodyRodStretchShear = extern struct {
    vertex: [2]u32,
    compliance: f32,
};

pub const SoftBodyRodBendTwist = extern struct {
    rod: [2]u32,
    compliance: f32,
};

pub const SoftBodyRodState = extern struct {
    vertex: [2]u32,
    rotation: Quat,
    angular_velocity: Vec3,
};

pub const SoftBodyValidateResult = enum(c_int) {
    accept_contact = 0,
    reject_contact = 1,
};

pub const SoftBodyContactSettings = extern struct {
    inv_mass_scale1: f32,
    inv_mass_scale2: f32,
    inv_inertia_scale2: f32,
    is_sensor: bool,
};

pub const SoftBodyManifold = opaque {};

pub const SoftBodyVertexContact = extern struct {
    vertex: u32,
    body: BodyId,
    local_contact_point: Vec3,
    normal: Vec3,
};

pub const SoftBodyContactListener = extern struct {
    on_contact_validate: ?*const fn (
        user: ?*anyopaque,
        soft_body: BodyId,
        other_body: BodyId,
        io_settings: *SoftBodyContactSettings,
    ) callconv(.c) SoftBodyValidateResult = null,
    on_contact_added: ?*const fn (
        user: ?*anyopaque,
        soft_body: BodyId,
        manifold: *const SoftBodyManifold,
    ) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub extern fn zjoltSoftBodySharedSettingsClone(settings: *const SoftBodySharedSettings, out: **SoftBodySharedSettings) Result;

pub extern fn zjoltSoftBodySharedSettingsSetMaterials(settings: *SoftBodySharedSettings, materials: ?[*]const *const PhysicsMaterial, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsGetMaterials(settings: *const SoftBodySharedSettings, out_materials: ?[*]*const PhysicsMaterial, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltSoftBodySharedSettingsAddRodStretchShearConstraints(settings: *SoftBodySharedSettings, rods: ?[*]const SoftBodyRodStretchShear, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsAddRodBendTwistConstraints(settings: *SoftBodySharedSettings, constraints: ?[*]const SoftBodyRodBendTwist, count: u32) Result;

pub extern fn zjoltSoftBodySharedSettingsCalculateVolumeConstraintVolumes(settings: *SoftBodySharedSettings) void;

pub extern fn zjoltSoftBodySharedSettingsCalculateRodProperties(settings: *SoftBodySharedSettings) Result;

pub extern fn zjoltSoftBodyGetRodStates(system: *const PhysicsSystem, body: BodyId, out_states: ?[*]SoftBodyRodState, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltSoftBodyGetFaceIndex(system: *const PhysicsSystem, body: BodyId, sub_shape_id: SubShapeId, out: *u32) Result;

pub extern fn zjoltSoftBodyCustomUpdate(system: *PhysicsSystem, body: BodyId, delta_time: f32) Result;

pub extern fn zjoltSoftBodyManifoldGetVertexContacts(manifold: *const SoftBodyManifold, out_contacts: ?[*]SoftBodyVertexContact, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltSoftBodyManifoldGetSensorContacts(manifold: *const SoftBodyManifold, out_bodies: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltSoftBodySetContactListener(system: *PhysicsSystem, listener: ?*const SoftBodyContactListener) Result;

pub const CollideSoftBodyVertexIterator = extern struct {
    position: ?[*]const u8 = null,
    position_stride: u32 = 0,
    inv_mass: ?[*]const u8 = null,
    inv_mass_stride: u32 = 0,
    collision_plane: ?[*]u8 = null,
    collision_plane_stride: u32 = 0,
    largest_penetration: ?[*]u8 = null,
    largest_penetration_stride: u32 = 0,
    colliding_shape_index: ?[*]u8 = null,
    colliding_shape_index_stride: u32 = 0,

    /// Records `penetration` as vertex `index`'s collision depth if it
    /// exceeds what `largest_penetration` already holds — JPH's own
    /// `CollideSoftBodyVertexIterator::UpdatePenetration`, done directly on
    /// caller memory with no ABI crossing. True means the caller must also
    /// write `collision_plane`/`colliding_shape_index` for `index`.
    pub fn updatePenetration(
        it: CollideSoftBodyVertexIterator,
        index: u32,
        penetration: f32,
    ) bool {
        const base = it.largest_penetration orelse return false;
        const slot: *align(1) f32 = @ptrCast(base + index * it.largest_penetration_stride);
        if (slot.* >= penetration) return false;
        slot.* = penetration;
        return true;
    }
};

pub extern fn zjoltShapeCollideSoftBodyVertices(shape_: *const Shape, center_of_mass_transform: *const Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, count: u32, colliding_shape_index: i32) Result;

pub const CollideSoftBodyVerticesVsTriangles = opaque {};

pub extern fn zjoltCollideSoftBodyVerticesVsTrianglesCreate(center_of_mass_transform: *const Mat44, scale: Vec3, out: **CollideSoftBodyVerticesVsTriangles) Result;
pub extern fn zjoltCollideSoftBodyVerticesVsTrianglesDestroy(collider: ?*CollideSoftBodyVerticesVsTriangles) void;
pub extern fn zjoltCollideSoftBodyVerticesVsTrianglesStartVertex(collider: *CollideSoftBodyVerticesVsTriangles, vertex: *const CollideSoftBodyVertexIterator, index: u32) void;
pub extern fn zjoltCollideSoftBodyVerticesVsTrianglesProcessTriangle(collider: *CollideSoftBodyVerticesVsTriangles, v0: Vec3, v1: Vec3, v2: Vec3) void;
pub extern fn zjoltCollideSoftBodyVerticesVsTrianglesFinishVertex(collider: *CollideSoftBodyVerticesVsTriangles, vertex: *const CollideSoftBodyVertexIterator, index: u32, colliding_shape_index: i32) void;

pub extern fn zjoltSoftBodyVertexMarkCcdContact(body: BodyId, contact_plane: *const Plane, out_collision_plane: *Plane, out_colliding_shape_index: *i32, out_has_contact: *bool) void;
