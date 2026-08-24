//! Hand-written declarations mirroring `ffi/zjolt.h`.
//!
//! These are written by hand rather than produced by `@cImport` so the package
//! stays translate-c-free and every type is exactly the shape the rest of the
//! wrapper wants. The cost of hand-writing is drift: nothing in either
//! compiler checks that this file still agrees with the header.
//!
//! Two things close that gap, between them covering both halves of a
//! declaration:
//!
//!   * `abi_check.zig` compares this file against the real header at comptime
//!     — every struct field by name and offset, every function's arity and
//!     per-parameter size, every enumerator's value. It discovers what to
//!     check by reflection, so it cannot fall behind what is declared here.
//!   * `zjoltInitWithConfig` refuses a library whose layout-affecting build
//!     settings differ from the ones `options` reports here, which is the one
//!     thing comparing against the header cannot tell you.

const std = @import("std");
const options = @import("zjolt_options");

//=============================================================================
// Build-configuration mirroring
//
// `ZJoltReal` and `ZJoltObjectLayer` change width with the build. The C header
// picks them from preprocessor macros; this picks them from the options module
// that `build.zig` fills in from the same struct that defines those macros.
// The two are therefore derived from one source, and `config_id` below is
// checked against the library's at init so that a mistake here fails loudly
// rather than misreading every position.
//=============================================================================

pub const Real = if (options.double_precision) f64 else f32;
pub const ObjectLayer = if (options.object_layer_bits == 32) u32 else u16;
pub const BroadPhaseLayer = u8;
pub const BodyId = u32;
pub const SubShapeId = u32;

pub const body_id_invalid: BodyId = 0xffff_ffff;
pub const sub_shape_id_empty: SubShapeId = 0xffff_ffff;

pub const version_major: u32 = 0;
pub const version_minor: u32 = 0;
pub const version_patch: u32 = 0;

/// The Zig side's view of ZJOLT_CONFIG_ID. Must be computed the same way the
/// header computes it.
pub const config_id: u32 = (version_major << 16) | (version_minor << 8) |
    version_patch |
    (@as(u32, if (options.double_precision) 1 else 0) << 24) |
    (@as(u32, if (options.object_layer_bits == 32) 1 else 0) << 25);

/// Bytes `zjoltShapeSave` prepends to Jolt's own payload.
pub const shape_header_size: usize = 32;

pub const max_physics_jobs: u32 = 2048;
pub const max_physics_barriers: u32 = 8;

/// How many physics systems may have a combine callback installed at once.
/// Jolt's combine hook carries no user pointer, so one fixed slot table on the
/// C side is what carries it.
pub const combine_slot_count: u32 = 64;

/// The group and sub-group id that mean "no group": a body carrying it
/// collides with everything.
pub const collision_group_invalid: u32 = 0xffff_ffff;

/// Past this, Jolt's `(n * (n - 1)) / 2` table sizing overflows 32 bits.
pub const group_filter_max_sub_groups: u32 = 65535;

pub const build_flag_double_precision: u32 = 1 << 0;
pub const build_flag_object_layer_32: u32 = 1 << 1;
pub const build_flag_asserts_enabled: u32 = 1 << 2;
pub const build_flag_cross_platform_deterministic: u32 = 1 << 3;

//=============================================================================
// Results
//=============================================================================

pub const Result = enum(c_int) {
    ok = 0,
    not_initialized = 1,
    already_initialized = 2,
    config_mismatch = 3,
    out_of_memory = 4,
    invalid_argument = 5,
    buffer_too_small = 6,
    shape_invalid = 7,
    bad_format = 8,
    body_not_found = 9,
    unsupported = 10,
};

//=============================================================================
// Plain data
//=============================================================================

pub const Vec3 = extern struct {
    x: f32,
    y: f32,
    z: f32,
};

pub const Quat = extern struct {
    x: f32,
    y: f32,
    z: f32,
    w: f32,
};

pub const RVec3 = extern struct {
    x: Real,
    y: Real,
    z: Real,
};

pub const AABox = extern struct {
    min: Vec3,
    max: Vec3,
};

pub const MassProperties = extern struct {
    mass: f32,
    inertia: [9]f32,
};

pub const ShapeStats = extern struct {
    size_bytes: u64,
    num_triangles: u32,
};

pub const Color = extern struct {
    r: u8,
    g: u8,
    b: u8,
    a: u8,
};

/// One child of a compound shape, in the parent's local space.
pub const CompoundChild = extern struct {
    shape: ?*const Shape = null,
    position: Vec3 = .{ .x = 0, .y = 0, .z = 0 },
    rotation: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 },
    user_data: u32 = 0,
};

//=============================================================================
// Enumerations
//=============================================================================

pub const MotionType = enum(c_int) {
    static = 0,
    kinematic = 1,
    dynamic = 2,
};

pub const MotionQuality = enum(c_int) {
    discrete = 0,
    linear_cast = 1,
};

pub const Activation = enum(c_int) {
    activate = 0,
    dont_activate = 1,
};

/// Which degrees of freedom a body may use, as a bit mask.
///
/// A packed struct rather than an enum, because that is what it is: the C
/// header spells the bits as enumerators only because C has no better way to
/// name them. This is layout-identical — six bits at the bottom of a `c_int` —
/// and lets a caller write `.{ .translation_z = false }` instead of an
/// `@enumFromInt` of an OR.
pub const AllowedDofs = packed struct(c_int) {
    translation_x: bool = true,
    translation_y: bool = true,
    translation_z: bool = true,
    rotation_x: bool = true,
    rotation_y: bool = true,
    rotation_z: bool = true,
    _reserved: u26 = 0,

    pub const all: AllowedDofs = .{};

    /// Movement in X and Y and rotation about Z — a body confined to a plane,
    /// which is Jolt's own `Plane2D`.
    pub const plane_2d: AllowedDofs = .{
        .translation_z = false,
        .rotation_x = false,
        .rotation_y = false,
    };
};

pub const OverrideMassProperties = enum(c_int) {
    calculate_mass_and_inertia = 0,
    calculate_inertia = 1,
};

pub const ShapeSubType = enum(c_int) {
    other = 0,
    sphere = 1,
    box = 2,
    capsule = 3,
    convex_hull = 4,
    mesh = 5,
    scaled = 6,
    rotated_translated = 7,
    offset_center_of_mass = 8,
    triangle = 9,
    cylinder = 10,
    tapered_capsule = 11,
    tapered_cylinder = 12,
    static_compound = 13,
    mutable_compound = 14,
    height_field = 15,
    plane = 16,
    empty = 17,
    soft_body = 18,
};

pub const BackFaceMode = enum(c_int) {
    ignore = 0,
    collide = 1,
};

pub const GroundState = enum(c_int) {
    on_ground = 0,
    on_steep_ground = 1,
    not_supported = 2,
    in_air = 3,
};

pub const ValidateResult = enum(c_int) {
    accept_all_contacts_for_this_body_pair = 0,
    accept_contact = 1,
    reject_contact = 2,
    reject_all_contacts_for_this_body_pair = 3,
};

/// What a step silently dropped, if anything. Non-zero does not mean the step
/// failed — it means a limit in `PhysicsSystem.Options` is too low.
///
/// A packed struct for the same reason as `AllowedDofs`: it is a bit mask, and
/// `if (update_error.contact_constraints_full)` reads better than masking.
pub const UpdateError = packed struct(u32) {
    manifold_cache_full: bool = false,
    body_pair_cache_full: bool = false,
    contact_constraints_full: bool = false,
    _reserved: u29 = 0,

    pub const none: UpdateError = .{};

    pub fn any(self: UpdateError) bool {
        return @as(u32, @bitCast(self)) != 0;
    }
};

/// Which parts of a simulation `zjoltPhysicsSystemSaveState` writes.
///
/// A packed struct for the same reason as `UpdateError`: the C header spells
/// it as an enum of bits only because C has no better way to name them.
pub const StateRecorderState = packed struct(u32) {
    global: bool = false,
    bodies: bool = false,
    contacts: bool = false,
    constraints: bool = false,
    _reserved: u28 = 0,

    pub const none: StateRecorderState = .{};

    /// What rollback wants. Anything less leaves part of the solver holding
    /// data from a different frame.
    pub const all: StateRecorderState = .{
        .global = true,
        .bodies = true,
        .contacts = true,
        .constraints = true,
    };
};

//=============================================================================
// Opaque handles
//=============================================================================

pub const Shape = opaque {};
pub const PhysicsMaterial = opaque {};
pub const PhysicsSystem = opaque {};
pub const JobSystem = opaque {};
pub const Body = opaque {};
pub const Character = opaque {};
pub const GroupFilter = opaque {};
pub const StepListener = opaque {};
pub const BodyAddBatch = opaque {};

//=============================================================================
// Allocator and hooks
//=============================================================================

pub const Allocator = extern struct {
    allocate: ?*const fn (user: ?*anyopaque, size: usize) callconv(.c) ?*anyopaque,
    reallocate: ?*const fn (user: ?*anyopaque, block: ?*anyopaque, old_size: usize, new_size: usize) callconv(.c) ?*anyopaque,
    free: ?*const fn (user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void,
    aligned_allocate: ?*const fn (user: ?*anyopaque, size: usize, alignment: usize) callconv(.c) ?*anyopaque,
    aligned_free: ?*const fn (user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void,
    user: ?*anyopaque,
};

pub const TraceFn = *const fn (user: ?*anyopaque, message: [*:0]const u8) callconv(.c) void;

pub const AssertFailedFn = *const fn (
    user: ?*anyopaque,
    expression: [*:0]const u8,
    message: ?[*:0]const u8,
    file: [*:0]const u8,
    line: u32,
) callconv(.c) bool;

pub const InitDesc = extern struct {
    allocator: ?*const Allocator = null,
    trace: ?TraceFn = null,
    assert_failed: ?AssertFailedFn = null,
    hooks_user: ?*anyopaque = null,
};

//=============================================================================
// Collision layers
//=============================================================================

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

//=============================================================================
// Listeners
//=============================================================================

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
    base_offset: RVec3,
    contact_point_on_1: Vec3,
    contact_point_on_2: Vec3,
    penetration_axis: Vec3,
    penetration_depth: f32,
    sub_shape_id1: SubShapeId,
    sub_shape_id2: SubShapeId,
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

//=============================================================================
// Physics system
//=============================================================================

pub const PhysicsSystemDesc = extern struct {
    max_bodies: u32,
    num_body_mutexes: u32,
    max_body_pairs: u32,
    max_contact_constraints: u32,
    temp_allocator_size: usize,
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

//=============================================================================
// Broad-phase queries
//=============================================================================

pub const BroadPhaseFilters = extern struct {
    broad_phase_layer: BroadPhaseLayerFilter = .{},
    object_layer: ObjectLayerFilter = .{},
};

pub const BroadPhaseCastHit = extern struct {
    body: BodyId,
    fraction: f32,
};

pub const OrientedBox = extern struct {
    center: RVec3,
    rotation: Quat,
    half_extent: Vec3,
};

//=============================================================================
// Collision groups
//=============================================================================

pub const CollisionGroup = extern struct {
    filter: ?*const GroupFilter = null,
    group_id: u32 = collision_group_invalid,
    sub_group_id: u32 = collision_group_invalid,
};

//=============================================================================
// Bodies
//=============================================================================

pub const BodyDesc = extern struct {
    position: RVec3,
    rotation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    shape: ?*const Shape,
    user_data: u64,
    object_layer: ObjectLayer,
    motion_type: MotionType,
    motion_quality: MotionQuality,
    allowed_dofs: AllowedDofs,
    override_mass_properties: OverrideMassProperties,
    mass: f32,
    allow_dynamic_or_kinematic: bool,
    is_sensor: bool,
    allow_sleeping: bool,
    enhanced_internal_edge_removal: bool,
    friction: f32,
    restitution: f32,
    linear_damping: f32,
    angular_damping: f32,
    max_linear_velocity: f32,
    max_angular_velocity: f32,
    gravity_factor: f32,
};

pub const BodyLock = extern struct {
    body: ?*Body,
    _reserved: [2]?*anyopaque,
};

//=============================================================================
// Queries
//=============================================================================

pub const BroadPhaseLayerFilter = extern struct {
    should_collide: ?*const fn (user: ?*anyopaque, layer: BroadPhaseLayer) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const ObjectLayerFilter = extern struct {
    should_collide: ?*const fn (user: ?*anyopaque, layer: ObjectLayer) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const BodyFilter = extern struct {
    should_collide: ?*const fn (user: ?*anyopaque, body: BodyId) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const ShapeFilter = extern struct {
    should_collide: ?*const fn (
        user: ?*anyopaque,
        body: BodyId,
        sub_shape_id: SubShapeId,
        query_sub_shape_id: SubShapeId,
    ) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub const QueryFilters = extern struct {
    broad_phase_layer: BroadPhaseLayerFilter = .{},
    object_layer: ObjectLayerFilter = .{},
    body: BodyFilter = .{},
    shape: ShapeFilter = .{},
};

pub const HitAction = enum(c_int) {
    @"continue" = 0,
    narrow = 1,
    stop = 2,
};

/// The defaults are Jolt's own, restated here so a Zig caller can write
/// `.{ .back_face_mode_convex = .collide }` and leave the rest alone. A test in
/// `query.zig` checks them against `zjoltRayCastSettingsInit`, so this copy
/// cannot quietly drift from the library's.
pub const RayCastSettings = extern struct {
    back_face_mode_triangles: BackFaceMode = .ignore,
    back_face_mode_convex: BackFaceMode = .ignore,
    treat_convex_as_solid: bool = true,
};

pub const RayCastHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    fraction: f32,
    normal: Vec3,
};

pub const ShapeCastHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    fraction: f32,
    contact_point_on_1: Vec3,
    contact_point_on_2: Vec3,
    penetration_axis: Vec3,
    penetration_depth: f32,
    is_back_face_hit: bool,
};

pub const CollideShapeHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    contact_point_on_1: Vec3,
    contact_point_on_2: Vec3,
    penetration_axis: Vec3,
    penetration_depth: f32,
};

pub const CollidePointHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
};

pub const RayCastHitFn = *const fn (user: ?*anyopaque, hit: *const RayCastHit) callconv(.c) HitAction;
pub const ShapeCastHitFn = *const fn (user: ?*anyopaque, hit: *const ShapeCastHit) callconv(.c) HitAction;
pub const CollideShapeHitFn = *const fn (user: ?*anyopaque, hit: *const CollideShapeHit) callconv(.c) HitAction;
pub const CollidePointHitFn = *const fn (user: ?*anyopaque, hit: *const CollidePointHit) callconv(.c) HitAction;

//=============================================================================
// Character
//=============================================================================

pub const CharacterDesc = extern struct {
    shape: ?*const Shape,
    position: RVec3,
    rotation: Quat,
    up: Vec3,
    shape_offset: Vec3,
    user_data: u64,
    max_slope_angle: f32,
    mass: f32,
    max_strength: f32,
    predictive_contact_distance: f32,
    character_padding: f32,
    penetration_recovery_speed: f32,
    collision_tolerance: f32,
    hit_reduction_cos_max_angle: f32,
    max_collision_iterations: u32,
    max_constraint_iterations: u32,
    max_num_hits: u32,
    back_face_mode: BackFaceMode,
    enhanced_internal_edge_removal: bool,
    inner_body_shape: ?*const Shape,
    inner_body_layer: ObjectLayer,
};

pub const CharacterUpdateSettings = extern struct {
    stick_to_floor_step_down: Vec3,
    walk_stairs_step_up: Vec3,
    walk_stairs_min_step_forward: f32,
    walk_stairs_step_forward_test: f32,
    walk_stairs_cos_angle_forward_contact: f32,
    walk_stairs_step_down_extra: Vec3,
};

//=============================================================================
// ABI layout report
//=============================================================================

pub const AbiLayout = extern struct {
    layout_size: u32,
    config_id: u32,
    build_flags: u32,
    real_size: u32,
    object_layer_size: u32,
    default_allocate_alignment: u32,
};

//=============================================================================
// Entry points
//
// zjoltInit is absent on purpose: it is a `static inline` wrapper in the
// header that supplies ZJOLT_CONFIG_ID, and Zig cannot call an inline C
// function. The Zig side calls zjoltInitWithConfig with the `config_id`
// computed above, which is the same check by the same mechanism.
//=============================================================================

pub extern fn zjoltVersion() u32;
pub extern fn zjoltJoltVersion() u32;
pub extern fn zjoltConfigId() u32;
pub extern fn zjoltResultName(result: Result) [*:0]const u8;
pub extern fn zjoltLastError() [*:0]const u8;
pub extern fn zjoltDefaultAllocateAlignment() usize;
pub extern fn zjoltInitWithConfig(desc: ?*const InitDesc, config_id: u32) Result;
pub extern fn zjoltDeinit() void;
pub extern fn zjoltIsInitialized() bool;
pub extern fn zjoltLiveHandleCount() u32;
pub extern fn zjoltAbiLayout(out: *AbiLayout) void;

pub extern fn zjoltJobSystemCreateThreadPool(max_jobs: u32, max_barriers: u32, num_threads: i32, out: **JobSystem) Result;
pub extern fn zjoltJobSystemCreateSingleThreaded(max_jobs: u32, out: **JobSystem) Result;
pub extern fn zjoltJobSystemDestroy(job_system: ?*JobSystem) void;
pub extern fn zjoltJobSystemGetMaxConcurrency(job_system: *const JobSystem) u32;

pub extern fn zjoltShapeCreateBox(half_extent: *const Vec3, convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreateCapsule(half_height_of_cylinder: f32, radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreateConvexHull(points: [*]const Vec3, num_points: u32, max_convex_radius: f32, hull_tolerance: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreateMesh(vertices: [*]const Vec3, num_vertices: u32, indices: [*]const u32, num_triangles: u32, triangle_materials: ?[*]const u32, materials: ?[*]const *const PhysicsMaterial, num_materials: u32, max_triangles_per_leaf: u32, out: **Shape) Result;
pub extern fn zjoltShapeCreateScaled(inner: *const Shape, scale: *const Vec3, out: **Shape) Result;
pub extern fn zjoltShapeCreateRotatedTranslated(inner: *const Shape, translation: *const Vec3, rotation: *const Quat, out: **Shape) Result;
pub extern fn zjoltShapeCreateOffsetCenterOfMass(inner: *const Shape, offset: *const Vec3, out: **Shape) Result;
pub extern fn zjoltShapeAddRef(shape: *const Shape) void;
pub extern fn zjoltShapeRelease(shape: *const Shape) void;
pub extern fn zjoltShapeGetRefCount(shape: *const Shape) u32;
pub extern fn zjoltShapeGetSubType(shape: *const Shape) ShapeSubType;
pub extern fn zjoltShapeGetVolume(shape: *const Shape) f32;
pub extern fn zjoltShapeGetCenterOfMass(shape: *const Shape, out: *Vec3) void;
pub extern fn zjoltShapeGetLocalBounds(shape: *const Shape, out: *AABox) void;
pub extern fn zjoltShapeGetMassProperties(shape: *const Shape, out: *MassProperties) void;
pub extern fn zjoltShapeGetStats(shape: *const Shape, out: *ShapeStats) void;
pub extern fn zjoltShapeSave(shape: *const Shape, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;
pub extern fn zjoltShapeRestore(data: [*]const u8, size: usize, out: **Shape) Result;
pub extern fn zjoltShapeCreateCylinder(half_height: f32, radius: f32, convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreateTriangle(v1: *const Vec3, v2: *const Vec3, v3: *const Vec3, convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreateTaperedCapsule(half_height_of_tapered_cylinder: f32, top_radius: f32, bottom_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreateTaperedCylinder(half_height: f32, top_radius: f32, bottom_radius: f32, convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreatePlane(normal: *const Vec3, constant: f32, half_extent: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
pub extern fn zjoltShapeCreateEmpty(center_of_mass: ?*const Vec3, out: **Shape) Result;
pub extern fn zjoltShapeCreateHeightField(samples: [*]const f32, sample_count: u32, offset: ?*const Vec3, scale: ?*const Vec3, material_indices: ?[*]const u8, materials: ?[*]const *const PhysicsMaterial, num_materials: u32, block_size: u32, bits_per_sample: u32, out: **Shape) Result;
pub extern fn zjoltShapeCreateStaticCompound(children: [*]const CompoundChild, num_children: u32, out: **Shape) Result;
pub extern fn zjoltShapeCreateMutableCompound(children: [*]const CompoundChild, num_children: u32, out: **Shape) Result;
pub extern fn zjoltShapeCompoundGetNumChildren(shape: *const Shape) u32;
pub extern fn zjoltShapeCompoundGetChildUserData(shape: *const Shape, index: u32) u32;
pub extern fn zjoltShapeMutableCompoundAddChild(shape: *Shape, child: *const CompoundChild, out_index: *u32) Result;
pub extern fn zjoltShapeMutableCompoundRemoveChild(shape: *Shape, index: u32) Result;
pub extern fn zjoltShapeMutableCompoundMoveChild(shape: *Shape, index: u32, position: *const Vec3, rotation: ?*const Quat) Result;
pub extern fn zjoltShapeMutableCompoundAdjustCenterOfMass(shape: *Shape) Result;
pub extern fn zjoltShapeGetMaterial(shape: *const Shape, sub_shape_id: SubShapeId) ?*const PhysicsMaterial;

pub extern fn zjoltPhysicsMaterialCreate(debug_name: ?[*:0]const u8, debug_color: ?*const Color, out: **PhysicsMaterial) Result;
pub extern fn zjoltPhysicsMaterialDefault() ?*const PhysicsMaterial;
pub extern fn zjoltPhysicsMaterialAddRef(material: *const PhysicsMaterial) void;
pub extern fn zjoltPhysicsMaterialRelease(material: *const PhysicsMaterial) void;
pub extern fn zjoltPhysicsMaterialGetRefCount(material: *const PhysicsMaterial) u32;
pub extern fn zjoltPhysicsMaterialGetDebugName(material: *const PhysicsMaterial) [*:0]const u8;
pub extern fn zjoltPhysicsMaterialGetDebugColor(material: *const PhysicsMaterial, out: *Color) void;

pub extern fn zjoltPhysicsSystemDescInit(desc: *PhysicsSystemDesc) void;
pub extern fn zjoltPhysicsSystemCreate(desc: *const PhysicsSystemDesc, out: **PhysicsSystem) Result;
pub extern fn zjoltPhysicsSystemDestroy(system: ?*PhysicsSystem) void;
pub extern fn zjoltPhysicsSystemSetGravity(system: *PhysicsSystem, gravity: *const Vec3) void;
pub extern fn zjoltPhysicsSystemGetGravity(system: *const PhysicsSystem, out: *Vec3) void;
pub extern fn zjoltPhysicsSystemOptimizeBroadPhase(system: *PhysicsSystem) void;
pub extern fn zjoltPhysicsSystemGetNumBodies(system: *const PhysicsSystem) u32;
pub extern fn zjoltPhysicsSystemGetNumActiveBodies(system: *const PhysicsSystem) u32;
pub extern fn zjoltPhysicsSystemSetContactListener(system: *PhysicsSystem, listener: ?*const ContactListener) Result;
pub extern fn zjoltPhysicsSystemSetBodyActivationListener(system: *PhysicsSystem, listener: ?*const BodyActivationListener) Result;
pub extern fn zjoltPhysicsSystemStep(system: *PhysicsSystem, delta_time: f32, collision_steps: i32, job_system: *JobSystem, out_error: ?*UpdateError) Result;
pub extern fn zjoltPhysicsSystemGetActiveBodies(system: *const PhysicsSystem, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltPhysicsSystemGetBodies(system: *const PhysicsSystem, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltBodyDescInit(desc: *BodyDesc) void;
pub extern fn zjoltBodyCreate(system: *PhysicsSystem, desc: *const BodyDesc, out: *BodyId) Result;
pub extern fn zjoltBodyCreateAndAdd(system: *PhysicsSystem, desc: *const BodyDesc, activation: Activation, out: *BodyId) Result;
pub extern fn zjoltBodyDestroy(system: *PhysicsSystem, body: BodyId) void;
pub extern fn zjoltBodyAdd(system: *PhysicsSystem, body: BodyId, activation: Activation) void;
pub extern fn zjoltBodyRemove(system: *PhysicsSystem, body: BodyId) void;
pub extern fn zjoltBodyIsAdded(system: *const PhysicsSystem, body: BodyId) bool;
pub extern fn zjoltBodyIsActive(system: *const PhysicsSystem, body: BodyId) bool;
pub extern fn zjoltBodyActivate(system: *PhysicsSystem, body: BodyId) void;
pub extern fn zjoltBodyDeactivate(system: *PhysicsSystem, body: BodyId) void;
pub extern fn zjoltBodySetMotionType(system: *PhysicsSystem, body: BodyId, motion_type: MotionType, activation: Activation) void;
pub extern fn zjoltBodyGetMotionType(system: *const PhysicsSystem, body: BodyId) MotionType;
pub extern fn zjoltBodySetPositionAndRotation(system: *PhysicsSystem, body: BodyId, position: *const RVec3, rotation: ?*const Quat, activation: Activation) void;
pub extern fn zjoltBodyGetPositionAndRotation(system: *const PhysicsSystem, body: BodyId, out_position: ?*RVec3, out_rotation: ?*Quat) void;
pub extern fn zjoltBodyGetCenterOfMassPosition(system: *const PhysicsSystem, body: BodyId, out: *RVec3) void;
pub extern fn zjoltBodyMoveKinematic(system: *PhysicsSystem, body: BodyId, target_position: *const RVec3, target_rotation: ?*const Quat, delta_time: f32) void;
pub extern fn zjoltBodySetLinearVelocity(system: *PhysicsSystem, body: BodyId, velocity: *const Vec3) void;
pub extern fn zjoltBodyGetLinearVelocity(system: *const PhysicsSystem, body: BodyId, out: *Vec3) void;
pub extern fn zjoltBodySetAngularVelocity(system: *PhysicsSystem, body: BodyId, velocity: *const Vec3) void;
pub extern fn zjoltBodyGetAngularVelocity(system: *const PhysicsSystem, body: BodyId, out: *Vec3) void;
pub extern fn zjoltBodyAddForce(system: *PhysicsSystem, body: BodyId, force: *const Vec3) void;
pub extern fn zjoltBodyAddForceAtPoint(system: *PhysicsSystem, body: BodyId, force: *const Vec3, point: *const RVec3) void;
pub extern fn zjoltBodyAddTorque(system: *PhysicsSystem, body: BodyId, torque: *const Vec3) void;
pub extern fn zjoltBodyAddImpulse(system: *PhysicsSystem, body: BodyId, impulse: *const Vec3) void;
pub extern fn zjoltBodyAddImpulseAtPoint(system: *PhysicsSystem, body: BodyId, impulse: *const Vec3, point: *const RVec3) void;
pub extern fn zjoltBodyAddAngularImpulse(system: *PhysicsSystem, body: BodyId, angular_impulse: *const Vec3) void;
pub extern fn zjoltBodySetShape(system: *PhysicsSystem, body: BodyId, shape: *const Shape, update_mass_properties: bool, activation: Activation) void;
pub extern fn zjoltBodySetObjectLayer(system: *PhysicsSystem, body: BodyId, layer: ObjectLayer) void;
pub extern fn zjoltBodyGetObjectLayer(system: *const PhysicsSystem, body: BodyId) ObjectLayer;
pub extern fn zjoltBodySetUserData(system: *PhysicsSystem, body: BodyId, user_data: u64) void;
pub extern fn zjoltBodyGetUserData(system: *const PhysicsSystem, body: BodyId) u64;
pub extern fn zjoltBodySetFriction(system: *PhysicsSystem, body: BodyId, friction: f32) void;
pub extern fn zjoltBodyGetFriction(system: *const PhysicsSystem, body: BodyId) f32;
pub extern fn zjoltBodySetRestitution(system: *PhysicsSystem, body: BodyId, restitution: f32) void;
pub extern fn zjoltBodyGetRestitution(system: *const PhysicsSystem, body: BodyId) f32;
pub extern fn zjoltBodySetGravityFactor(system: *PhysicsSystem, body: BodyId, factor: f32) void;
pub extern fn zjoltBodyGetGravityFactor(system: *const PhysicsSystem, body: BodyId) f32;
pub extern fn zjoltBodyGetMaterial(system: *const PhysicsSystem, body: BodyId, sub_shape_id: SubShapeId) ?*const PhysicsMaterial;
pub extern fn zjoltBodyNotifyShapeChanged(system: *PhysicsSystem, body: BodyId, previous_center_of_mass: *const Vec3, update_mass_properties: bool, activation: Activation) void;

pub extern fn zjoltBodyGetTransforms(system: *const PhysicsSystem, ids: [*]const BodyId, count: u32, out_positions: ?[*]RVec3, out_rotations: ?[*]Quat, out_missing: ?*u32) Result;
pub extern fn zjoltBodyGetMotions(system: *const PhysicsSystem, ids: [*]const BodyId, count: u32, out_center_of_mass: ?[*]RVec3, out_linear_velocities: ?[*]Vec3, out_missing: ?*u32) Result;

pub extern fn zjoltBodyLockRead(system: *const PhysicsSystem, body: BodyId, out_lock: *BodyLock) void;
pub extern fn zjoltBodyLockReadRelease(lock: *BodyLock) void;
pub extern fn zjoltBodyLockWrite(system: *PhysicsSystem, body: BodyId, out_lock: *BodyLock) void;
pub extern fn zjoltBodyLockWriteRelease(lock: *BodyLock) void;

pub extern fn zjoltBodyGetId(body: *const Body) BodyId;
pub extern fn zjoltBodyGetPosition(body: *const Body, out: *RVec3) void;
pub extern fn zjoltBodyGetRotation(body: *const Body, out: *Quat) void;
pub extern fn zjoltBodyGetCenterOfMassPositionLocked(body: *const Body, out: *RVec3) void;
pub extern fn zjoltBodyGetLinearVelocityLocked(body: *const Body, out: *Vec3) void;
pub extern fn zjoltBodyGetAngularVelocityLocked(body: *const Body, out: *Vec3) void;
pub extern fn zjoltBodyGetUserDataLocked(body: *const Body) u64;
pub extern fn zjoltBodyGetObjectLayerLocked(body: *const Body) ObjectLayer;
pub extern fn zjoltBodyGetMotionTypeLocked(body: *const Body) MotionType;
pub extern fn zjoltBodyIsActiveLocked(body: *const Body) bool;
pub extern fn zjoltBodyIsSensorLocked(body: *const Body) bool;
pub extern fn zjoltBodyGetShapeLocked(body: *const Body) ?*const Shape;
pub extern fn zjoltBodyGetWorldBounds(body: *const Body, out: *AABox) void;
pub extern fn zjoltBodyMutSetLinearVelocity(body: *Body, velocity: *const Vec3) void;
pub extern fn zjoltBodyMutSetAngularVelocity(body: *Body, velocity: *const Vec3) void;
pub extern fn zjoltBodyMutSetUserData(body: *Body, user_data: u64) void;
pub extern fn zjoltBodyMutSetFriction(body: *Body, friction: f32) void;
pub extern fn zjoltBodyMutSetRestitution(body: *Body, restitution: f32) void;
pub extern fn zjoltBodyMutAddImpulse(body: *Body, impulse: *const Vec3) void;

pub extern fn zjoltRayCastSettingsInit(settings: *RayCastSettings) void;
pub extern fn zjoltCastRayClosest(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filters: ?*const QueryFilters, out_hit: *RayCastHit, out_hit_any: *bool) Result;
pub extern fn zjoltCastRayAll(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filters: ?*const QueryFilters, out_hits: ?[*]RayCastHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltCastRayEach(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filters: ?*const QueryFilters, on_hit: RayCastHitFn, user: ?*anyopaque) Result;
pub extern fn zjoltCastShapeClosest(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, filters: ?*const QueryFilters, out_hit: *ShapeCastHit, out_hit_any: *bool) Result;
pub extern fn zjoltCastShapeAll(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, filters: ?*const QueryFilters, out_hits: ?[*]ShapeCastHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltCastShapeEach(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, filters: ?*const QueryFilters, on_hit: ShapeCastHitFn, user: ?*anyopaque) Result;
pub extern fn zjoltCollideShapeAll(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, max_separation_distance: f32, filters: ?*const QueryFilters, out_hits: ?[*]CollideShapeHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltCollideShapeEach(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, max_separation_distance: f32, filters: ?*const QueryFilters, on_hit: CollideShapeHitFn, user: ?*anyopaque) Result;
pub extern fn zjoltCollidePointAll(system: *const PhysicsSystem, point: *const RVec3, filters: ?*const QueryFilters, out_hits: ?[*]CollidePointHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltCollidePointEach(system: *const PhysicsSystem, point: *const RVec3, filters: ?*const QueryFilters, on_hit: CollidePointHitFn, user: ?*anyopaque) Result;

pub extern fn zjoltCharacterDescInit(desc: *CharacterDesc) void;
pub extern fn zjoltCharacterUpdateSettingsInit(settings: *CharacterUpdateSettings) void;
pub extern fn zjoltCharacterCreate(system: *PhysicsSystem, desc: *const CharacterDesc, out: **Character) Result;
pub extern fn zjoltCharacterDestroy(character: ?*Character) void;
pub extern fn zjoltCharacterUpdate(character: *Character, delta_time: f32, gravity: *const Vec3, settings: ?*const CharacterUpdateSettings, filters: ?*const QueryFilters) Result;
pub extern fn zjoltCharacterGetPosition(character: *const Character, out: *RVec3) void;
pub extern fn zjoltCharacterSetPosition(character: *Character, position: *const RVec3) void;
pub extern fn zjoltCharacterGetRotation(character: *const Character, out: *Quat) void;
pub extern fn zjoltCharacterSetRotation(character: *Character, rotation: *const Quat) void;
pub extern fn zjoltCharacterGetLinearVelocity(character: *const Character, out: *Vec3) void;
pub extern fn zjoltCharacterSetLinearVelocity(character: *Character, velocity: *const Vec3) void;
pub extern fn zjoltCharacterGetGroundState(character: *const Character) GroundState;
pub extern fn zjoltCharacterIsSupported(character: *const Character) bool;
pub extern fn zjoltCharacterGetGroundNormal(character: *const Character, out: *Vec3) void;
pub extern fn zjoltCharacterGetGroundVelocity(character: *const Character, out: *Vec3) void;
pub extern fn zjoltCharacterGetGroundPosition(character: *const Character, out: *RVec3) void;
pub extern fn zjoltCharacterGetGroundBodyId(character: *const Character) BodyId;
pub extern fn zjoltCharacterGetGroundUserData(character: *const Character) u64;
pub extern fn zjoltCharacterUpdateGroundVelocity(character: *Character) void;
pub extern fn zjoltCharacterSetShape(character: *Character, shape: *const Shape, max_penetration_depth: f32, filters: ?*const QueryFilters, out_changed: ?*bool) Result;
pub extern fn zjoltCharacterGetShape(character: *const Character) ?*const Shape;
pub extern fn zjoltCharacterGetInnerBodyId(character: *const Character) BodyId;

pub extern fn zjoltPhysicsSystemGetMaxBodies(system: *const PhysicsSystem) u32;
pub extern fn zjoltPhysicsSystemWereBodiesInContact(system: *const PhysicsSystem, body1: BodyId, body2: BodyId) bool;
pub extern fn zjoltPhysicsSettingsInit(settings: *PhysicsSettings) void;
pub extern fn zjoltPhysicsSystemGetSettings(system: *const PhysicsSystem, out: *PhysicsSettings) void;
pub extern fn zjoltPhysicsSystemSetSettings(system: *PhysicsSystem, settings: *const PhysicsSettings) Result;
pub extern fn zjoltPhysicsSystemSetCombineFriction(system: *PhysicsSystem, combine: ?CombineFn, user: ?*anyopaque) Result;
pub extern fn zjoltPhysicsSystemSetCombineRestitution(system: *PhysicsSystem, combine: ?CombineFn, user: ?*anyopaque) Result;
pub extern fn zjoltPhysicsSystemAddStepListener(system: *PhysicsSystem, listener: ?StepListenerFn, user: ?*anyopaque, out: **StepListener) Result;
pub extern fn zjoltPhysicsSystemRemoveStepListener(system: *PhysicsSystem, listener: *StepListener) Result;

pub extern fn zjoltBroadPhaseCastRay(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, filters: ?*const BroadPhaseFilters, out_hits: ?[*]BroadPhaseCastHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltBroadPhaseCollideAABox(system: *const PhysicsSystem, box: *const AABox, filters: ?*const BroadPhaseFilters, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltBroadPhaseCollideSphere(system: *const PhysicsSystem, center: *const RVec3, radius: f32, filters: ?*const BroadPhaseFilters, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltBroadPhaseCollidePoint(system: *const PhysicsSystem, point: *const RVec3, filters: ?*const BroadPhaseFilters, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltBroadPhaseCollideOrientedBox(system: *const PhysicsSystem, box: *const OrientedBox, filters: ?*const BroadPhaseFilters, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltBroadPhaseCastAABox(system: *const PhysicsSystem, box: *const AABox, direction: *const Vec3, filters: ?*const BroadPhaseFilters, out_hits: ?[*]BroadPhaseCastHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltBroadPhaseGetBounds(system: *const PhysicsSystem, out: *AABox) void;

pub extern fn zjoltBodyAddBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32, activation: Activation) Result;
pub extern fn zjoltBodyAddBatchPrepare(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32, out: **BodyAddBatch) Result;
pub extern fn zjoltBodyAddBatchFinalize(system: *PhysicsSystem, batch: *BodyAddBatch, activation: Activation) Result;
pub extern fn zjoltBodyAddBatchAbort(system: *PhysicsSystem, batch: *BodyAddBatch) Result;
pub extern fn zjoltBodyRemoveBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32) Result;
pub extern fn zjoltBodyDestroyBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32) Result;
pub extern fn zjoltBodyActivateBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32) Result;
pub extern fn zjoltBodyDeactivateBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32) Result;
pub extern fn zjoltBodyActivateInBox(system: *PhysicsSystem, box: *const AABox, filters: ?*const BroadPhaseFilters) Result;

pub extern fn zjoltGroupFilterTableCreate(num_sub_groups: u32, out: **GroupFilter) Result;
pub extern fn zjoltGroupFilterAddRef(filter: *const GroupFilter) void;
pub extern fn zjoltGroupFilterRelease(filter: *const GroupFilter) void;
pub extern fn zjoltGroupFilterGetRefCount(filter: *const GroupFilter) u32;
pub extern fn zjoltGroupFilterGetNumSubGroups(filter: *const GroupFilter) u32;
pub extern fn zjoltGroupFilterTableDisableCollision(filter: *GroupFilter, sub_group1: u32, sub_group2: u32) Result;
pub extern fn zjoltGroupFilterTableEnableCollision(filter: *GroupFilter, sub_group1: u32, sub_group2: u32) Result;
pub extern fn zjoltGroupFilterTableIsCollisionEnabled(filter: *const GroupFilter, sub_group1: u32, sub_group2: u32, out_enabled: *bool) Result;
pub extern fn zjoltBodySetCollisionGroup(system: *PhysicsSystem, body: BodyId, group: ?*const CollisionGroup) void;
pub extern fn zjoltBodyGetCollisionGroup(system: *const PhysicsSystem, body: BodyId, out: *CollisionGroup) void;
pub extern fn zjoltBodyInvalidateContactCache(system: *PhysicsSystem, body: BodyId) void;

pub extern fn zjoltPhysicsSystemSaveState(system: *const PhysicsSystem, state: StateRecorderState, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;
pub extern fn zjoltPhysicsSystemRestoreState(system: *PhysicsSystem, data: [*]const u8, size: usize) Result;

//=============================================================================
// Vehicles
//=============================================================================

pub const VehicleControllerKind = enum(c_int) {
    wheeled = 0,
    tracked = 1,
    motorcycle = 2,
};

pub const VehicleCollisionTesterKind = enum(c_int) {
    ray = 0,
    cast_sphere = 1,
    cast_cylinder = 2,
};

pub const VehicleTransmissionMode = enum(c_int) {
    auto = 0,
    manual = 1,
};

pub const vehicle_curve_max_points: usize = 8;

pub const VehicleCurvePoint = extern struct {
    x: f32,
    y: f32,
};

pub const VehicleCurveDesc = extern struct {
    points: [8]VehicleCurvePoint,
    count: u32,
};

pub const VehicleWheelDesc = extern struct {
    position: Vec3,
    suspension_force_point: Vec3,
    enable_suspension_force_point: bool,
    suspension_direction: Vec3,
    steering_axis: Vec3,
    wheel_up: Vec3,
    wheel_forward: Vec3,
    suspension_min_length: f32,
    suspension_max_length: f32,
    suspension_preload_length: f32,
    suspension_spring_frequency: f32,
    suspension_spring_damping: f32,
    radius: f32,
    width: f32,
    inertia: f32,
    angular_damping: f32,
    max_steer_angle: f32,
    longitudinal_friction: VehicleCurveDesc,
    lateral_friction: VehicleCurveDesc,
    max_brake_torque: f32,
    max_hand_brake_torque: f32,
    tracked_longitudinal_friction: f32,
    tracked_lateral_friction: f32,
};

pub const VehicleEngineDesc = extern struct {
    max_torque: f32,
    min_rpm: f32,
    max_rpm: f32,
    normalized_torque: VehicleCurveDesc,
    inertia: f32,
    angular_damping: f32,
};

pub const VehicleTransmissionDesc = extern struct {
    mode: VehicleTransmissionMode,
    forward_gear_ratios: ?[*]const f32,
    forward_gear_ratio_count: u32,
    reverse_gear_ratios: ?[*]const f32,
    reverse_gear_ratio_count: u32,
    switch_time: f32,
    clutch_release_time: f32,
    switch_latency: f32,
    shift_up_rpm: f32,
    shift_down_rpm: f32,
    clutch_strength: f32,
};

pub const VehicleDifferentialDesc = extern struct {
    left_wheel: i32,
    right_wheel: i32,
    differential_ratio: f32,
    left_right_split: f32,
    limited_slip_ratio: f32,
    engine_torque_ratio: f32,
};

pub const VehicleAntiRollBarDesc = extern struct {
    left_wheel: i32,
    right_wheel: i32,
    stiffness: f32,
};

pub const VehicleCollisionTesterDesc = extern struct {
    kind: VehicleCollisionTesterKind,
    object_layer: ObjectLayer,
    up: Vec3,
    max_slope_angle: f32,
    radius: f32,
    convex_radius_fraction: f32,
};

pub const VehicleMotorcycleDesc = extern struct {
    max_lean_angle: f32,
    lean_spring_constant: f32,
    lean_spring_damping: f32,
    lean_spring_integration_coefficient: f32,
    lean_spring_integration_coefficient_decay: f32,
    lean_smoothing_factor: f32,
};

pub const VehicleConstraint = opaque {};

pub const VehicleConstraintDesc = extern struct {
    up: Vec3,
    forward: Vec3,
    max_pitch_roll_angle: f32,
    wheels: ?[*]const VehicleWheelDesc,
    wheel_count: u32,
    anti_roll_bars: ?[*]const VehicleAntiRollBarDesc,
    anti_roll_bar_count: u32,
    controller_kind: VehicleControllerKind,
    engine: VehicleEngineDesc,
    transmission: VehicleTransmissionDesc,
    differentials: ?[*]const VehicleDifferentialDesc,
    differential_count: u32,
    differential_limited_slip_ratio: f32,
    left_track_wheels: ?[*]const u32,
    left_track_wheel_count: u32,
    left_track_driven_wheel: u32,
    right_track_wheels: ?[*]const u32,
    right_track_wheel_count: u32,
    right_track_driven_wheel: u32,
    track_inertia: f32,
    track_angular_damping: f32,
    track_max_brake_torque: f32,
    track_differential_ratio: f32,
    motorcycle: VehicleMotorcycleDesc,
    collision_tester: VehicleCollisionTesterDesc,
};

pub extern fn zjoltVehicleWheelDescInit(desc: *VehicleWheelDesc) void;
pub extern fn zjoltVehicleEngineDescInit(desc: *VehicleEngineDesc) void;
pub extern fn zjoltVehicleTransmissionDescInit(desc: *VehicleTransmissionDesc) void;
pub extern fn zjoltVehicleDifferentialDescInit(desc: *VehicleDifferentialDesc) void;
pub extern fn zjoltVehicleAntiRollBarDescInit(desc: *VehicleAntiRollBarDesc) void;
pub extern fn zjoltVehicleCollisionTesterDescInit(desc: *VehicleCollisionTesterDesc) void;
pub extern fn zjoltVehicleMotorcycleDescInit(desc: *VehicleMotorcycleDesc) void;
pub extern fn zjoltVehicleConstraintDescInit(desc: *VehicleConstraintDesc) void;

pub extern fn zjoltVehicleConstraintCreate(system: *PhysicsSystem, body: BodyId, desc: *const VehicleConstraintDesc, out: **VehicleConstraint) Result;
pub extern fn zjoltVehicleConstraintDestroy(constraint: *VehicleConstraint) void;

pub extern fn zjoltVehicleConstraintGetControllerKind(constraint: *const VehicleConstraint) VehicleControllerKind;
pub extern fn zjoltVehicleConstraintGetCollisionTesterKind(constraint: *const VehicleConstraint) VehicleCollisionTesterKind;
pub extern fn zjoltVehicleConstraintGetBodyId(constraint: *const VehicleConstraint) BodyId;
pub extern fn zjoltVehicleConstraintIsActive(constraint: *const VehicleConstraint) bool;
pub extern fn zjoltVehicleConstraintGetMaxPitchRollAngle(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintSetMaxPitchRollAngle(constraint: *VehicleConstraint, max_pitch_roll_angle: f32) void;
pub extern fn zjoltVehicleConstraintGetLocalUp(constraint: *const VehicleConstraint, out: *Vec3) void;
pub extern fn zjoltVehicleConstraintGetLocalForward(constraint: *const VehicleConstraint, out: *Vec3) void;
pub extern fn zjoltVehicleConstraintGetWorldUp(constraint: *const VehicleConstraint, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetWheelCount(constraint: *const VehicleConstraint) u32;
pub extern fn zjoltVehicleConstraintGetWheelRotationAngle(constraint: *const VehicleConstraint, wheel_index: u32) f32;
pub extern fn zjoltVehicleConstraintSetWheelRotationAngle(constraint: *VehicleConstraint, wheel_index: u32, angle: f32) void;
pub extern fn zjoltVehicleConstraintGetWheelSteerAngle(constraint: *const VehicleConstraint, wheel_index: u32) f32;
pub extern fn zjoltVehicleConstraintSetWheelSteerAngle(constraint: *VehicleConstraint, wheel_index: u32, angle: f32) void;
pub extern fn zjoltVehicleConstraintGetWheelAngularVelocity(constraint: *const VehicleConstraint, wheel_index: u32) f32;
pub extern fn zjoltVehicleConstraintSetWheelAngularVelocity(constraint: *VehicleConstraint, wheel_index: u32, velocity: f32) void;
pub extern fn zjoltVehicleConstraintGetWheelSuspensionLength(constraint: *const VehicleConstraint, wheel_index: u32) f32;
pub extern fn zjoltVehicleConstraintHasWheelContact(constraint: *const VehicleConstraint, wheel_index: u32) bool;
pub extern fn zjoltVehicleConstraintGetWheelContactBodyId(constraint: *const VehicleConstraint, wheel_index: u32) BodyId;
pub extern fn zjoltVehicleConstraintGetWheelContactPosition(constraint: *const VehicleConstraint, wheel_index: u32, out: *RVec3) void;
pub extern fn zjoltVehicleConstraintGetWheelContactNormal(constraint: *const VehicleConstraint, wheel_index: u32, out: *Vec3) void;
pub extern fn zjoltVehicleConstraintGetWheelContactLongitudinal(constraint: *const VehicleConstraint, wheel_index: u32, out: *Vec3) void;
pub extern fn zjoltVehicleConstraintGetWheelContactLateral(constraint: *const VehicleConstraint, wheel_index: u32, out: *Vec3) void;
pub extern fn zjoltVehicleConstraintGetWheelContactPointVelocity(constraint: *const VehicleConstraint, wheel_index: u32, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintSetWheeledDriverInput(constraint: *VehicleConstraint, forward: f32, right: f32, brake: f32, hand_brake: f32) Result;
pub extern fn zjoltVehicleConstraintGetWheeledForwardInput(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetWheeledRightInput(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetWheeledBrakeInput(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetWheeledHandBrakeInput(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetWheeledEngineRpm(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetWheeledCurrentGear(constraint: *const VehicleConstraint) i32;
pub extern fn zjoltVehicleConstraintIsWheeledSwitchingGear(constraint: *const VehicleConstraint) bool;

pub extern fn zjoltVehicleConstraintSetTrackedDriverInput(constraint: *VehicleConstraint, forward: f32, left_ratio: f32, right_ratio: f32, brake: f32) Result;
pub extern fn zjoltVehicleConstraintGetTrackedEngineRpm(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetTrackedCurrentGear(constraint: *const VehicleConstraint) i32;

pub extern fn zjoltVehicleConstraintSetMotorcycleLeanControllerEnabled(constraint: *VehicleConstraint, enabled: bool) Result;
pub extern fn zjoltVehicleConstraintIsMotorcycleLeanControllerEnabled(constraint: *const VehicleConstraint) bool;
