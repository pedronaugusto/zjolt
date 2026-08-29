//! ZJolt C declarations for hair simulation and the compute backend it runs on.
//!
//! Mirrors `ffi/zjolt_hair.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const BodyId = core.BodyId;
pub const InitDesc = core.InitDesc;
pub const ObjectLayer = core.ObjectLayer;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Vec3 = core.Vec3;
pub const config_id = core.config_id;
pub const zjoltDeinit = core.zjoltDeinit;
pub const zjoltInitWithConfig = core.zjoltInitWithConfig;

pub const ComputeSystem = opaque {};

pub const Hair = opaque {};

pub const ComputeBufferType = enum(c_int) {
    upload = 0,
    readback = 1,
    constant = 2,
    read_only = 3,
    read_write = 4,
};

pub const ComputeMapMode = enum(c_int) {
    read = 0,
    write = 1,
};

pub const ComputeBarrier = enum(c_int) {
    insert = 0,
    skip = 1,
};

/// A host compute backend, as a flat table. The `?*anyopaque` handles are the
/// host's own; nothing on this side ever dereferences one.
pub const ComputeInterface = extern struct {
    create_shader: ?*const fn (user: ?*anyopaque, name: [*:0]const u8, group_size_x: u32, group_size_y: u32, group_size_z: u32, out_shader: *?*anyopaque) callconv(.c) Result = null,
    create_buffer: ?*const fn (user: ?*anyopaque, kind: ComputeBufferType, size: u64, stride: u32, data: ?*const anyopaque, out_buffer: *?*anyopaque) callconv(.c) Result = null,
    create_readback_buffer: ?*const fn (user: ?*anyopaque, buffer: ?*anyopaque, out_buffer: *?*anyopaque) callconv(.c) Result = null,
    create_queue: ?*const fn (user: ?*anyopaque, out_queue: *?*anyopaque) callconv(.c) Result = null,
    destroy_shader: ?*const fn (user: ?*anyopaque, shader: ?*anyopaque) callconv(.c) void = null,
    destroy_buffer: ?*const fn (user: ?*anyopaque, buffer: ?*anyopaque) callconv(.c) void = null,
    destroy_queue: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque) callconv(.c) void = null,
    map_buffer: ?*const fn (user: ?*anyopaque, buffer: ?*anyopaque, mode: ComputeMapMode) callconv(.c) ?*anyopaque = null,
    unmap_buffer: ?*const fn (user: ?*anyopaque, buffer: ?*anyopaque) callconv(.c) void = null,
    queue_set_shader: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque, shader: ?*anyopaque) callconv(.c) void = null,
    queue_set_constant_buffer: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque, name: [*:0]const u8, buffer: ?*anyopaque) callconv(.c) void = null,
    queue_set_buffer: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque, name: [*:0]const u8, buffer: ?*anyopaque) callconv(.c) void = null,
    queue_set_rw_buffer: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque, name: [*:0]const u8, buffer: ?*anyopaque, barrier: ComputeBarrier) callconv(.c) void = null,
    queue_dispatch: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque, groups_x: u32, groups_y: u32, groups_z: u32) callconv(.c) void = null,
    queue_schedule_readback: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque, dst: ?*anyopaque, src: ?*anyopaque) callconv(.c) void = null,
    queue_execute: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque) callconv(.c) void = null,
    queue_wait: ?*const fn (user: ?*anyopaque, queue: ?*anyopaque) callconv(.c) void = null,
    destroy: ?*const fn (user: ?*anyopaque) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub const HairVertex = extern struct {
    position: Vec3,
    inv_mass: f32,
};

pub const HairStrand = extern struct {
    start_vertex: u32,
    end_vertex: u32,
    material_index: u32,

    /// The strand's length in its rest pose: consecutive vertex distances
    /// summed the way `JPH::HairSettings::RStrand::MeasureLength` does — what
    /// hair creation uses internally to size long-range attachments.
    /// `vertices` is indexed by `start_vertex .. end_vertex`.
    pub fn measureLength(strand: HairStrand, vertices: []const HairVertex) f32 {
        var length: f32 = 0;
        var v = strand.start_vertex;
        while (v + 1 < strand.end_vertex) : (v += 1) {
            const a = vertices[v].position;
            const b = vertices[v + 1].position;
            const dx = b.x - a.x;
            const dy = b.y - a.y;
            const dz = b.z - a.z;
            length += @sqrt(dx * dx + dy * dy + dz * dz);
        }
        return length;
    }
};

pub const HairGradient = extern struct {
    min: f32,
    max: f32,
    min_fraction: f32,
    max_fraction: f32,

    /// The gradient reparameterized from Jolt's fixed authoring rate
    /// (`cDefaultIterationsPerSecond`) to one substep of `time_ratio` default
    /// substeps: `min`/`max` become `1 - (1 - value)^time_ratio`, the identity
    /// a hair update applies every substep, so a slower or faster solver
    /// converges at the same rate. `min_fraction`/`max_fraction` pass through.
    pub fn makeStepDependent(gradient: HairGradient, time_ratio: f32) HairGradient {
        const stepDependent = struct {
            fn f(value: f32, ratio: f32) f32 {
                return 1.0 - std.math.pow(f32, 1.0 - value, ratio);
            }
        }.f;
        return .{
            .min = stepDependent(gradient.min, time_ratio),
            .max = stepDependent(gradient.max, time_ratio),
            .min_fraction = gradient.min_fraction,
            .max_fraction = gradient.max_fraction,
        };
    }
};

pub const HairSkinWeight = extern struct {
    joint_index: u32,
    weight: f32,
};

pub const HairMaterial = extern struct {
    enable_collision: bool,
    enable_lra: bool,
    linear_damping: f32,
    angular_damping: f32,
    max_linear_velocity: f32,
    max_angular_velocity: f32,
    gravity_factor: HairGradient,
    friction: f32,
    bend_compliance: f32,
    bend_compliance_multiplier: [4]f32,
    stretch_compliance: f32,
    inertia_multiplier: f32,
    hair_radius: HairGradient,
    world_transform_influence: HairGradient,
    grid_velocity_factor: HairGradient,
    grid_density_force_factor: f32,
    global_pose: HairGradient,
    skin_global_pose: HairGradient,
    simulation_strands_fraction: f32,
    gravity_preload_factor: f32,

    /// Whether the material needs the velocity/density grid built and
    /// stepped: `JPH::HairSettings::Material::NeedsGrid`'s condition, a
    /// nonzero `grid_velocity_factor` or `grid_density_force_factor`.
    pub fn needsGrid(material: HairMaterial) bool {
        return material.grid_velocity_factor.min != 0 or
            material.grid_velocity_factor.max != 0 or
            material.grid_density_force_factor != 0;
    }
};

pub const HairVertexState = extern struct {
    position: Vec3,
    rotation: Quat,
    velocity: Vec3,
    angular_velocity: Vec3,
};

pub const HairInfo = extern struct {
    simulated_vertex_count: u32,
    simulated_strand_count: u32,
    render_vertex_count: u32,
    render_strand_count: u32,
    material_count: u32,
    joint_count: u32,
    max_vertices_per_strand: u32,
    padded_vertex_count: u32,
    simulation_bounds_min: Vec3,
    simulation_bounds_max: Vec3,
    grid_size_x: u32,
    grid_size_y: u32,
    grid_size_z: u32,
    max_root_distance_to_scalp: f32,
};

pub const HairGridCell = extern struct {
    velocity: Vec3,
    density: f32,
};

pub const HairReadBackView = extern struct {
    scalp_vertices: ?[*]const Vec3,
    grid_velocity_and_density: ?[*]const HairGridCell,
    render_positions: ?[*]const Vec3,
};

pub const HairDesc = extern struct {
    vertices: ?[*]const HairVertex,
    strands: ?[*]const HairStrand,
    materials: ?[*]const HairMaterial,
    vertex_count: u32,
    strand_count: u32,
    material_count: u32,
    scalp_vertices: ?[*]const Vec3,
    scalp_triangles: ?[*]const u32,
    scalp_skin_weights: ?[*]const HairSkinWeight,
    scalp_inverse_bind_pose: ?[*]const f32,
    scalp_vertex_count: u32,
    scalp_triangle_count: u32,
    skin_weights_per_vertex: u32,
    joint_count: u32,
    initial_gravity: Vec3,
    simulation_bounds_padding: Vec3,
    grid_size_x: u32,
    grid_size_y: u32,
    grid_size_z: u32,
    iterations_per_second: u32,
    max_delta_time: f32,
    position: RVec3,
    rotation: Quat,
    object_layer: ObjectLayer,
};

pub extern fn zjoltComputeIsCpuSupported() bool;

pub extern fn zjoltComputeSystemCreateCpu(out: **ComputeSystem) Result;

pub extern fn zjoltComputeSystemCreate(iface: *const ComputeInterface, out: **ComputeSystem) Result;

pub extern fn zjoltComputeSystemDestroy(compute: ?*ComputeSystem) void;

pub extern fn zjoltHairMaterialInit(out: *HairMaterial) void;

pub extern fn zjoltHairCreate(compute: *ComputeSystem, desc: *const HairDesc, out: **Hair) Result;

pub extern fn zjoltHairDestroy(hair: ?*Hair) void;

pub extern fn zjoltHairSetTransform(hair: *Hair, position: ?*const RVec3, rotation: ?*const Quat) Result;

pub extern fn zjoltHairGetTransform(hair: *const Hair, out_position: ?*RVec3, out_rotation: ?*Quat) Result;

pub extern fn zjoltHairFollowBody(hair: *Hair, system: *const PhysicsSystem, body: BodyId) Result;

pub extern fn zjoltHairSetPose(hair: *Hair, joint_to_hair: [*]const f32, joint_matrices: [*]const f32, joint_count: u32) Result;

pub extern fn zjoltHairGetJointCount(hair: *const Hair, out_count: *u32) Result;

pub extern fn zjoltHairOnTeleported(hair: *Hair) Result;

pub extern fn zjoltHairUpdate(hair: *Hair, system: *PhysicsSystem, delta_time: f32) Result;

pub extern fn zjoltHairReadBackPositions(hair: *Hair, out_positions: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltHairReadBackRenderPositions(hair: *Hair, out_positions: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltHairReadBackScalpVertices(hair: *Hair, out_positions: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltHairReadBackVertexState(hair: *Hair, out_state: ?[*]HairVertexState, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltHairGetSimulatedStrands(hair: *const Hair, out_strands: ?[*]HairStrand, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltHairGetInfo(hair: *const Hair, out: *HairInfo) Result;

pub extern fn zjoltHairLockReadBackBuffers(hair: *Hair, out: *HairReadBackView) Result;

pub extern fn zjoltHairUnlockReadBackBuffers(hair: *Hair) Result;

pub extern fn zjoltHairGetNeutralDensity(hair: *const Hair, x: u32, y: u32, z: u32, out_density: *f32) Result;

pub extern fn zjoltHairPositionToGridIndex(hair: *const Hair, position: *const Vec3, out_index_x: *u32, out_index_y: *u32, out_index_z: *u32, out_fraction: *Vec3) Result;

pub extern fn zjoltHairSkinScalpVertices(hair: *const Hair, joint_to_hair: [*]const f32, joint_matrices: [*]const f32, joint_count: u32, out_vertices: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltHairSaveGroom(hair: *const Hair, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;

pub extern fn zjoltHairCreateFromGroom(compute: *ComputeSystem, data: [*]const u8, size: usize, position: ?*const RVec3, rotation: ?*const Quat, object_layer: ObjectLayer, out: **Hair) Result;

pub extern fn zjoltHairGradientSample(gradient: *const HairGradient, strand_fraction: f32, out_value: *f32) Result;

pub extern fn zjoltHairMaterialGetBendCompliance(material: *const HairMaterial, strand_fraction: f32, out_value: *f32) Result;
