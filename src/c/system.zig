//! ZJolt C declarations for the physics system, its settings, and the listeners it calls.
//!
//! Mirrors `ffi/zjolt_system.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const body = @import("body.zig");
const core = @import("core.zig");
const query = @import("query.zig");
const softbody = @import("softbody.zig");

pub const SoftBodyContactListener = softbody.SoftBodyContactListener;
pub const CollideShapeSettings = query.CollideShapeSettings;
pub const Mat44 = core.Mat44;

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Body = body.Body;
pub const zjoltBodyGetMotions = body.zjoltBodyGetMotions;
pub const zjoltBodyGetTransforms = body.zjoltBodyGetTransforms;
pub const zjoltBodyLockRead = body.zjoltBodyLockRead;
pub const zjoltBodyLockWrite = body.zjoltBodyLockWrite;
pub const zjoltBodyLockMultiRead = body.zjoltBodyLockMultiRead;
pub const zjoltBodyLockMultiWrite = body.zjoltBodyLockMultiWrite;
pub const zjoltPhysicsSystemGetActiveBodies = body.zjoltPhysicsSystemGetActiveBodies;
pub const zjoltPhysicsSystemGetBodies = body.zjoltPhysicsSystemGetBodies;
pub const BodyId = core.BodyId;
pub const BroadPhaseLayer = core.BroadPhaseLayer;
pub const JobSystem = core.JobSystem;
pub const ObjectLayer = core.ObjectLayer;
pub const PhysicsSystem = core.PhysicsSystem;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const StepListener = core.StepListener;
pub const SubShapeId = core.SubShapeId;
pub const UpdateError = core.UpdateError;
pub const ValidateResult = core.ValidateResult;
pub const Vec3 = core.Vec3;
pub const max_physics_barriers = core.max_physics_barriers;
pub const max_physics_jobs = core.max_physics_jobs;
pub const zjoltJobSystemCreateSingleThreaded = core.zjoltJobSystemCreateSingleThreaded;
pub const zjoltJobSystemCreateThreadPool = core.zjoltJobSystemCreateThreadPool;
pub const zjoltJobSystemDestroy = core.zjoltJobSystemDestroy;
pub const zjoltJobSystemGetMaxConcurrency = core.zjoltJobSystemGetMaxConcurrency;
pub const ThreadHookFn = core.ThreadHookFn;
pub const zjoltJobSystemCreateThreadPoolWithHooks = core.zjoltJobSystemCreateThreadPoolWithHooks;
pub const zjoltJobSystemSetNumThreads = core.zjoltJobSystemSetNumThreads;
pub const Job = core.Job;
pub const HostJobSystem = core.HostJobSystem;
pub const zjoltJobSystemCreateHost = core.zjoltJobSystemCreateHost;
pub const zjoltJobRun = core.zjoltJobRun;
pub const zjoltJobAddRef = core.zjoltJobAddRef;
pub const zjoltJobRelease = core.zjoltJobRelease;

/// How many physics systems may have a combine callback installed at once.
/// Jolt's combine hook carries no user pointer, so one fixed slot table on the
/// C side is what carries it.
pub const combine_slot_count: u32 = 64;

pub const BroadPhaseLayerInterface = extern struct {
    num_broad_phase_layers: ?*const fn (user: ?*anyopaque) callconv(.c) u32 = null,
    broad_phase_layer_for_object_layer: ?*const fn (user: ?*anyopaque, layer: ObjectLayer) callconv(.c) BroadPhaseLayer = null,
    broad_phase_layer_name: ?*const fn (user: ?*anyopaque, layer: BroadPhaseLayer) callconv(.c) ?[*:0]const u8 = null,
    user: ?*anyopaque = null,
};

pub const ObjectVsBroadPhaseLayerFilter = extern struct {
    should_collide: ?*const fn (user: ?*anyopaque, layer1: ObjectLayer, layer2: BroadPhaseLayer) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const ObjectLayerPairFilter = extern struct {
    should_collide: ?*const fn (user: ?*anyopaque, layer1: ObjectLayer, layer2: ObjectLayer) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const Shape = core.Shape;

pub const SimShapeFilter = extern struct {
    should_collide: ?*const fn (
        user: ?*anyopaque,
        body1: BodyId,
        shape1: ?*const Shape,
        sub_shape_id1: SubShapeId,
        body2: BodyId,
        shape2: ?*const Shape,
        sub_shape_id2: SubShapeId,
    ) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const ContactManifold = extern struct {
    base_offset: RVec3,
    world_space_normal: Vec3,
    penetration_depth: f32,
    sub_shape_id1: SubShapeId,
    sub_shape_id2: SubShapeId,
    num_points: u32,
    points_on_1: ?[*]const Vec3,
    points_on_2: ?[*]const Vec3,
};

pub const ContactInfo = extern struct {
    body1: BodyId,
    body2: BodyId,
    user_data1: u64,
    user_data2: u64,
    live_body1: ?*const Body,
    live_body2: ?*const Body,
    manifold: ContactManifold,
};

pub const ContactSettings = extern struct {
    combined_friction: f32,
    combined_restitution: f32,
    inv_mass_scale1: f32,
    inv_inertia_scale1: f32,
    inv_mass_scale2: f32,
    inv_inertia_scale2: f32,
    is_sensor: bool,
    relative_linear_surface_velocity: Vec3,
    relative_angular_surface_velocity: Vec3,
};

pub const ContactValidateInfo = extern struct {
    body1: BodyId,
    body2: BodyId,
    user_data1: u64,
    user_data2: u64,
    live_body1: ?*const Body,
    live_body2: ?*const Body,
    base_offset: RVec3,
    contact_point_on_1: Vec3,
    contact_point_on_2: Vec3,
    penetration_axis: Vec3,
    penetration_depth: f32,
    sub_shape_id1: SubShapeId,
    sub_shape_id2: SubShapeId,
    num_shape1_face_vertices: u32,
    num_shape2_face_vertices: u32,
    shape1_face: ?[*]const Vec3,
    shape2_face: ?[*]const Vec3,
};

pub const SubShapeIdPair = extern struct {
    body1: BodyId,
    sub_shape_id1: SubShapeId,
    body2: BodyId,
    sub_shape_id2: SubShapeId,
};

pub const ContactListener = extern struct {
    on_contact_validate: ?*const fn (user: ?*anyopaque, info: *const ContactValidateInfo) callconv(.c) ValidateResult = null,
    on_contact_added: ?*const fn (user: ?*anyopaque, info: *const ContactInfo, settings: *ContactSettings) callconv(.c) void = null,
    on_contact_persisted: ?*const fn (user: ?*anyopaque, info: *const ContactInfo, settings: *ContactSettings) callconv(.c) void = null,
    on_contact_removed: ?*const fn (user: ?*anyopaque, pair: *const SubShapeIdPair) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub const BodyActivationListener = extern struct {
    on_body_activated: ?*const fn (user: ?*anyopaque, body: BodyId, user_data: u64) callconv(.c) void = null,
    on_body_deactivated: ?*const fn (user: ?*anyopaque, body: BodyId, user_data: u64) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub const TempAllocatorKind = enum(c_int) {
    malloc_fallback = 0,
    fixed = 1,
    host = 2,
};

pub const TempAllocator = extern struct {
    allocate: ?*const fn (user: ?*anyopaque, size: u32) callconv(.c) ?*anyopaque = null,
    free: ?*const fn (user: ?*anyopaque, address: ?*anyopaque, size: u32) callconv(.c) void = null,
    can_allocate: ?*const fn (user: ?*anyopaque, size: u32) callconv(.c) bool = null,
    get_size: ?*const fn (user: ?*anyopaque) callconv(.c) usize = null,
    get_usage: ?*const fn (user: ?*anyopaque) callconv(.c) usize = null,
    user: ?*anyopaque = null,
};

pub const TempAllocatorStats = extern struct {
    capacity: usize,
    usage: usize,
};

pub const PhysicsSystemDesc = extern struct {
    max_bodies: u32,
    num_body_mutexes: u32,
    max_body_pairs: u32,
    max_contact_constraints: u32,
    temp_allocator_size: usize,
    temp_allocator_kind: TempAllocatorKind,
    temp_allocator: ?*const TempAllocator,
    broad_phase_layers: BroadPhaseLayerInterface,
    object_vs_broad_phase_filter: ObjectVsBroadPhaseLayerFilter,
    object_layer_pair_filter: ObjectLayerPairFilter,
};

/// Jolt's `PhysicsSettings`, field for field. What each one means is
/// documented on the C side, in `ffi/zjolt_system.h`.
pub const PhysicsSettings = extern struct {
    max_in_flight_body_pairs: i32,
    step_listeners_batch_size: i32,
    step_listener_batches_per_job: i32,
    baumgarte: f32,
    speculative_contact_distance: f32,
    penetration_slop: f32,
    linear_cast_threshold: f32,
    linear_cast_max_penetration: f32,
    manifold_tolerance: f32,
    max_penetration_distance: f32,
    body_pair_cache_max_delta_position_sq: f32,
    body_pair_cache_cos_max_delta_rotation_div2: f32,
    contact_normal_cos_max_delta_rotation: f32,
    contact_point_preserve_lambda_max_dist_sq: f32,
    internal_edge_removal_vertex_tolerance_sq: f32,
    num_velocity_steps: u32,
    num_position_steps: u32,
    min_velocity_for_restitution: f32,
    time_before_sleep: f32,
    point_velocity_sleep_threshold: f32,
    deterministic_simulation: bool,
    constraint_warm_start: bool,
    use_body_pair_contact_cache: bool,
    use_manifold_reduction: bool,
    use_large_island_splitter: bool,
    allow_sleeping: bool,
    check_active_edges: bool,
};

pub const CombineInfo = extern struct {
    body1: BodyId,
    sub_shape_id1: SubShapeId,
    user_data1: u64,
    value1: f32,
    body2: BodyId,
    sub_shape_id2: SubShapeId,
    user_data2: u64,
    value2: f32,
};

pub const CombineFn = *const fn (user: ?*anyopaque, info: *const CombineInfo) callconv(.c) f32;

pub const StepListenerContext = extern struct {
    delta_time: f32,
    is_first_step: bool,
    is_last_step: bool,
};

pub const StepListenerFn = *const fn (user: ?*anyopaque, context: *const StepListenerContext) callconv(.c) void;

pub const BodyStats = extern struct {
    num_bodies: u32,
    max_bodies: u32,
    num_bodies_static: u32,
    num_bodies_dynamic: u32,
    num_active_bodies_dynamic: u32,
    num_bodies_kinematic: u32,
    num_active_bodies_kinematic: u32,
    num_soft_bodies: u32,
    num_active_soft_bodies: u32,
};

pub extern fn zjoltPhysicsSystemDescInit(desc: *PhysicsSystemDesc) void;

pub extern fn zjoltPhysicsSystemCreate(desc: *const PhysicsSystemDesc, out: **PhysicsSystem) Result;

pub extern fn zjoltPhysicsSystemGetBroadPhaseLayerInterface(system: *const PhysicsSystem, out: *BroadPhaseLayerInterface) void;

pub extern fn zjoltPhysicsSystemGetObjectVsBroadPhaseLayerFilter(system: *const PhysicsSystem, out: *ObjectVsBroadPhaseLayerFilter) void;

pub extern fn zjoltPhysicsSystemGetObjectLayerPairFilter(system: *const PhysicsSystem, out: *ObjectLayerPairFilter) void;

pub extern fn zjoltPhysicsSystemDestroy(system: ?*PhysicsSystem) void;

pub extern fn zjoltPhysicsSystemGetTempAllocatorStats(system: *const PhysicsSystem, out: *TempAllocatorStats) void;

pub extern fn zjoltPhysicsSystemTempAllocatorCanAllocate(system: *const PhysicsSystem, size: u32) bool;

pub extern fn zjoltPhysicsSystemSetGravity(system: *PhysicsSystem, gravity: *const Vec3) void;

pub extern fn zjoltPhysicsSystemGetGravity(system: *const PhysicsSystem, out: *Vec3) void;

pub extern fn zjoltPhysicsSystemOptimizeBroadPhase(system: *PhysicsSystem) void;

pub extern fn zjoltPhysicsSystemGetNumBodies(system: *const PhysicsSystem) u32;

pub extern fn zjoltPhysicsSystemGetNumActiveBodies(system: *const PhysicsSystem) u32;

pub extern fn zjoltPhysicsSystemSetContactListener(system: *PhysicsSystem, listener: ?*const ContactListener) Result;

pub extern fn zjoltPhysicsSystemSetBodyActivationListener(system: *PhysicsSystem, listener: ?*const BodyActivationListener) Result;

pub extern fn zjoltPhysicsSystemGetContactListener(system: *const PhysicsSystem, out: *ContactListener) void;

pub extern fn zjoltPhysicsSystemGetBodyActivationListener(system: *const PhysicsSystem, out: *BodyActivationListener) void;

pub extern fn zjoltPhysicsSystemGetSoftBodyContactListener(system: *const PhysicsSystem, out: *SoftBodyContactListener) void;

pub extern fn zjoltPhysicsSystemSetSimShapeFilter(system: *PhysicsSystem, filter: ?*const SimShapeFilter) Result;

pub extern fn zjoltPhysicsSystemGetSimShapeFilter(system: *const PhysicsSystem, out: *SimShapeFilter) void;

pub const SimCollideHit = extern struct {
    sub_shape_id1: SubShapeId,
    sub_shape_id2: SubShapeId,
    contact_point_on_1: Vec3,
    contact_point_on_2: Vec3,
    penetration_axis: Vec3,
    penetration_depth: f32,
};

pub const SimCollideCollector = opaque {};
pub const SimCollideShapeFilter = opaque {};

pub const SimCollideFn = *const fn (
    user: ?*anyopaque,
    live_body1: *const Body,
    live_body2: *const Body,
    center_of_mass_transform1: *const Mat44,
    center_of_mass_transform2: *const Mat44,
    settings: *CollideShapeSettings,
    shape_filter: *const SimCollideShapeFilter,
    collector: *SimCollideCollector,
) callconv(.c) void;

pub const SimCollideBodyVsBody = extern struct {
    collide: ?SimCollideFn = null,
    user: ?*anyopaque = null,
};

pub extern fn zjoltPhysicsSystemSetSimCollideBodyVsBody(system: *PhysicsSystem, hook: ?*const SimCollideBodyVsBody) Result;

pub extern fn zjoltPhysicsSystemGetSimCollideBodyVsBody(system: *const PhysicsSystem, out: *SimCollideBodyVsBody) void;

pub extern fn zjoltSimCollideAddHit(collector: *SimCollideCollector, body2: BodyId, hit: *const SimCollideHit) void;

pub extern fn zjoltSimCollideDefault(
    live_body1: *const Body,
    live_body2: *const Body,
    center_of_mass_transform1: *const Mat44,
    center_of_mass_transform2: *const Mat44,
    settings: ?*const CollideShapeSettings,
    shape_filter: ?*const SimCollideShapeFilter,
    collector: *SimCollideCollector,
) void;

pub extern fn zjoltReportNarrowPhaseStats() Result;

pub extern fn zjoltPhysicsSystemTryGetBodyNoLock(system: *const PhysicsSystem, body_id: BodyId) ?*const body.Body;

pub extern fn zjoltPhysicsSystemGetActiveBodiesUnsafe(system: *const PhysicsSystem, out_ids: *?[*]const BodyId, out_count: *u32) void;

pub extern fn zjoltPhysicsSystemStep(system: *PhysicsSystem, delta_time: f32, collision_steps: i32, job_system: *JobSystem, out_error: ?*UpdateError) Result;

pub extern fn zjoltPhysicsSystemGetMaxBodies(system: *const PhysicsSystem) u32;

pub extern fn zjoltPhysicsSystemGetBodyStats(system: *const PhysicsSystem, out: *BodyStats) void;

pub extern fn zjoltPhysicsSystemReportBroadphaseStats(system: *PhysicsSystem) Result;

pub extern fn zjoltPhysicsSystemWereBodiesInContact(system: *const PhysicsSystem, body1: BodyId, body2: BodyId) bool;

pub extern fn zjoltPhysicsSettingsInit(settings: *PhysicsSettings) void;

pub extern fn zjoltPhysicsSystemGetSettings(system: *const PhysicsSystem, out: *PhysicsSettings) void;

pub extern fn zjoltPhysicsSystemSetSettings(system: *PhysicsSystem, settings: *const PhysicsSettings) Result;

pub extern fn zjoltPhysicsSystemSetCombineFriction(system: *PhysicsSystem, combine: ?CombineFn, user: ?*anyopaque) Result;

pub extern fn zjoltPhysicsSystemSetCombineRestitution(system: *PhysicsSystem, combine: ?CombineFn, user: ?*anyopaque) Result;

pub extern fn zjoltPhysicsSystemGetCombineRestitution(system: *const PhysicsSystem, out_combine: *?CombineFn, out_user: *?*anyopaque) void;

pub extern fn zjoltPhysicsSystemAddStepListener(system: *PhysicsSystem, listener: ?StepListenerFn, user: ?*anyopaque, out: **StepListener) Result;

pub extern fn zjoltPhysicsSystemRemoveStepListener(system: *PhysicsSystem, listener: *StepListener) Result;

//=============================================================================
// Islands
//=============================================================================

pub const IslandBuilder = opaque {};

pub extern fn zjoltIslandBuilderCreate(out: **IslandBuilder) Result;

pub extern fn zjoltIslandBuilderDestroy(builder: ?*IslandBuilder) void;

pub extern fn zjoltIslandBuilderInit(builder: *IslandBuilder, max_active_bodies: u32) Result;

pub extern fn zjoltIslandBuilderLinkBodies(builder: *IslandBuilder, first: u32, second: u32) Result;

pub extern fn zjoltIslandBuilderPrepareContactConstraints(builder: *IslandBuilder, max_contacts: u32, temp_allocator: *const TempAllocator) Result;

pub extern fn zjoltIslandBuilderPrepareNonContactConstraints(builder: *IslandBuilder, num_constraints: u32, temp_allocator: *const TempAllocator) Result;

pub extern fn zjoltIslandBuilderLinkConstraint(builder: *IslandBuilder, constraint_index: u32, index_in_active_body_list: u32) Result;

pub extern fn zjoltIslandBuilderLinkContact(builder: *IslandBuilder, contact_index: u32, index_in_active_body_list: u32) Result;

pub extern fn zjoltIslandBuilderFinalize(builder: *IslandBuilder, active_bodies: ?[*]const BodyId, num_active_bodies: u32, num_contacts: u32, temp_allocator: *const TempAllocator) Result;

pub extern fn zjoltIslandBuilderGetNumIslands(builder: *const IslandBuilder) u32;

pub extern fn zjoltIslandBuilderGetBodiesInIsland(builder: *const IslandBuilder, island_index: u32, out_bodies: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltIslandBuilderGetConstraintsInIsland(builder: *const IslandBuilder, island_index: u32, out_constraints: ?[*]u32, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltIslandBuilderGetContactsInIsland(builder: *const IslandBuilder, island_index: u32, out_contacts: ?[*]u32, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltIslandBuilderGetNumPositionSteps(builder: *const IslandBuilder, island_index: u32, out_num_position_steps: *u32) Result;

pub const IslandStats = extern struct {
    velocity_constraint_ticks: u64,
    position_constraint_ticks: u64,
    update_bounds_ticks: u64,
    num_velocity_steps: u8,
    num_position_steps: u8,
    is_large_island: bool,
};

pub extern fn zjoltIslandBuilderGetStats(builder: *const IslandBuilder, island_index: u32, out_stats: *IslandStats) Result;

pub extern fn zjoltIslandBuilderSetNumPositionSteps(builder: *IslandBuilder, island_index: u32, num_position_steps: u32) Result;

pub extern fn zjoltIslandBuilderResetIslands(builder: *IslandBuilder, temp_allocator: *const TempAllocator) Result;
