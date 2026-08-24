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

/// Bytes of `DebugText.text`, including the terminator.
pub const debug_text_max_length: u32 = 64;

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

/// Four columns of four floats, column-major: column `c`'s row `r` is
/// `m[4 * c + r]`.
pub const Mat44 = extern struct {
    m: [16]f32,
};

/// `Mat44` with `Real` elements, the way `RVec3` is `Vec3` with `Real`
/// elements. Its width follows `-Ddouble_precision`.
pub const RMat44 = extern struct {
    m: [16]Real,
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

pub const BodyType = enum(c_int) {
    rigid_body = 0,
    soft_body = 1,
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
pub const AllowedDofs = packed struct(u32) {
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
    /// Not a shape at all — what `zjoltShapeGetSubType` reports for a null
    /// handle. Distinct from `user_defined`, which is a shape of a kind this
    /// binding has no name for.
    none = 0,
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
    /// One of the sixteen `User*` slots Jolt reserves for shape types
    /// registered outside this library. Nothing here can construct one.
    user_defined = 19,
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
pub const RigidCharacter = opaque {};
pub const CharacterContactListener = opaque {};
pub const CharacterVsCharacterCollision = opaque {};
pub const GroupFilter = opaque {};
pub const StepListener = opaque {};
pub const BodyAddBatch = opaque {};
pub const DebugRenderer = opaque {};

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
    collision_group: CollisionGroup,
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
    material: ?*const PhysicsMaterial,
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
    material: ?*const PhysicsMaterial,
};

pub const CollideShapeHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    contact_point_on_1: Vec3,
    contact_point_on_2: Vec3,
    penetration_axis: Vec3,
    penetration_depth: f32,
    material: ?*const PhysicsMaterial,
};

pub const CollidePointHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    material: ?*const PhysicsMaterial,
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

//=============================================================================
// Shape introspection, triangle read-back, mesh and height-field specifics
//=============================================================================

/// Vertices `zjoltShapeGetSupportingFace` can report in one call.
pub const shape_max_supporting_face_vertices: u32 = 32;

/// Fewest triangles `zjoltShapeGetTrianglesNext` accepts a request for.
pub const shape_min_triangles_requested: u32 = 32;

/// Opaque scratch space for one Shape-level triangle walk. Matches
/// `Shape::GetTrianglesContext` byte for byte; never read from Zig.
pub const ShapeTrianglesContext = extern struct {
    data: [4288]u8 align(16) = undefined,
};

/// The same scratch space, for a walk over a `TransformedShape` instead. A
/// second type rather than the one above reused, matching the C header: the
/// two query surfaces are independent, so nothing here assumes they share a
/// layout even though they happen to today.
pub const TransformedShapeTrianglesContext = extern struct {
    data: [4288]u8 align(16) = undefined,
};

pub extern fn zjoltShapeGetSubShapeIDBits(shape: *const Shape) u32;
pub extern fn zjoltShapeGetSurfaceNormal(shape: *const Shape, sub_shape_id: SubShapeId, local_surface_position: *const Vec3, out_normal: *Vec3) void;
pub extern fn zjoltShapeGetSupportingFace(shape: *const Shape, sub_shape_id: SubShapeId, direction: *const Vec3, scale: ?*const Vec3, position: *const Vec3, rotation: *const Quat, out_vertices: [*]Vec3, out_count: *u32) Result;
pub extern fn zjoltShapeGetSubShapeTransformedShape(shape: *const Shape, sub_shape_id: SubShapeId, position: ?*const Vec3, rotation: ?*const Quat, scale: ?*const Vec3, out: **TransformedShape, out_remainder: ?*SubShapeId) Result;
pub extern fn zjoltShapeScaleShape(shape: *const Shape, scale: *const Vec3, out: **Shape) Result;
pub extern fn zjoltShapeIsValidScale(shape: *const Shape, scale: *const Vec3) bool;
pub extern fn zjoltShapeMakeScaleValid(shape: *const Shape, scale: *const Vec3, out_scale: *Vec3) void;

pub extern fn zjoltShapeGetTrianglesStart(shape: *const Shape, context: *ShapeTrianglesContext, box: *const AABox, position: ?*const Vec3, rotation: ?*const Quat, scale: ?*const Vec3) Result;
pub extern fn zjoltShapeGetTrianglesNext(shape: *const Shape, context: *ShapeTrianglesContext, max_triangles: u32, out_vertices: [*]Vec3, out_materials: ?[*]?*const PhysicsMaterial, out_count: *u32) Result;
pub extern fn zjoltShapeGetMaterialList(shape: *const Shape, out_materials: ?[*]?*const PhysicsMaterial, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltShapeMeshGetMaterialIndex(shape: *const Shape, sub_shape_id: SubShapeId) u32;
pub extern fn zjoltShapeMeshGetTriangleUserData(shape: *const Shape, sub_shape_id: SubShapeId) u32;

pub extern fn zjoltShapeHeightFieldGetSampleCount(shape: *const Shape) u32;
pub extern fn zjoltShapeHeightFieldGetBlockSize(shape: *const Shape) u32;
pub extern fn zjoltShapeHeightFieldGetMinHeightValue(shape: *const Shape) f32;
pub extern fn zjoltShapeHeightFieldGetMaxHeightValue(shape: *const Shape) f32;
pub extern fn zjoltShapeHeightFieldGetPosition(shape: *const Shape, x: u32, y: u32, out_position: *Vec3) void;
pub extern fn zjoltShapeHeightFieldIsNoCollision(shape: *const Shape, x: u32, y: u32) bool;
pub extern fn zjoltShapeHeightFieldProjectOntoSurface(shape: *const Shape, local_position: *const Vec3, out_surface_position: *Vec3, out_sub_shape_id: *SubShapeId, out_found: *bool) Result;
pub extern fn zjoltShapeHeightFieldGetSubShapeCoordinates(shape: *const Shape, sub_shape_id: SubShapeId, out_x: *u32, out_y: *u32, out_triangle_index: *u32) Result;
pub extern fn zjoltShapeHeightFieldGetHeights(shape: *const Shape, x: u32, y: u32, size_x: u32, size_y: u32, out_heights: [*]f32, stride: u32) Result;

//=============================================================================
// TransformedShape — a shape, placed in the world, queried on its own
//=============================================================================

pub const TransformedShape = opaque {};

pub const TransformedShapeTransform = extern struct {
    position: RVec3,
    rotation: Quat,
    scale: Vec3,
};

pub extern fn zjoltTransformedShapeCreate(shape: *const Shape, position: *const RVec3, rotation: *const Quat, scale: ?*const Vec3, body: BodyId, out: **TransformedShape) Result;
pub extern fn zjoltTransformedShapeDestroy(ts: ?*TransformedShape) void;

pub extern fn zjoltTransformedShapeGetWorldTransform(ts: *const TransformedShape, out: *TransformedShapeTransform) void;
pub extern fn zjoltTransformedShapeSetWorldTransform(ts: *TransformedShape, position: *const RVec3, rotation: *const Quat, scale: ?*const Vec3) void;
pub extern fn zjoltTransformedShapeGetWorldSpaceBounds(ts: *const TransformedShape, out: *AABox) void;
pub extern fn zjoltTransformedShapeGetWorldSpaceSurfaceNormal(ts: *const TransformedShape, sub_shape_id: SubShapeId, position: *const RVec3, out_normal: *Vec3) void;
pub extern fn zjoltTransformedShapeGetMaterial(ts: *const TransformedShape, sub_shape_id: SubShapeId) ?*const PhysicsMaterial;
pub extern fn zjoltTransformedShapeGetSubShapeUserData(ts: *const TransformedShape, sub_shape_id: SubShapeId) u64;
pub extern fn zjoltTransformedShapeGetSupportingFace(ts: *const TransformedShape, sub_shape_id: SubShapeId, direction: *const Vec3, base_offset: *const RVec3, out_vertices: [*]Vec3, out_count: *u32) Result;

pub extern fn zjoltTransformedShapeGetTrianglesStart(ts: *const TransformedShape, context: *TransformedShapeTrianglesContext, box: *const AABox, base_offset: *const RVec3) Result;
pub extern fn zjoltTransformedShapeGetTrianglesNext(ts: *const TransformedShape, context: *TransformedShapeTrianglesContext, max_triangles: u32, out_vertices: [*]Vec3, out_materials: ?[*]?*const PhysicsMaterial, out_count: *u32) Result;

pub extern fn zjoltTransformedShapeCastRayClosest(ts: *const TransformedShape, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filter: ?*const ShapeFilter, out_hit: *RayCastHit, out_hit_any: *bool) Result;
pub extern fn zjoltTransformedShapeCastRayAll(ts: *const TransformedShape, origin: *const RVec3, direction: *const Vec3, settings: ?*const RayCastSettings, filter: ?*const ShapeFilter, out_hits: ?[*]RayCastHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltTransformedShapeCollidePointAll(ts: *const TransformedShape, point: *const RVec3, filter: ?*const ShapeFilter, out_hits: ?[*]CollidePointHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltTransformedShapeCollideShapeAll(ts: *const TransformedShape, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, max_separation_distance: f32, base_offset: *const RVec3, filter: ?*const ShapeFilter, out_hits: ?[*]CollideShapeHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltTransformedShapeCastShapeClosest(ts: *const TransformedShape, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, base_offset: *const RVec3, filter: ?*const ShapeFilter, out_hit: *ShapeCastHit, out_hit_any: *bool) Result;
pub extern fn zjoltTransformedShapeCastShapeAll(ts: *const TransformedShape, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, base_offset: *const RVec3, filter: ?*const ShapeFilter, out_hits: ?[*]ShapeCastHit, capacity: u32, out_count: *u32) Result;

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
pub extern fn zjoltBodyResetSleepTimer(system: *PhysicsSystem, body: BodyId) void;
pub extern fn zjoltBodyGetBodyType(system: *const PhysicsSystem, body: BodyId) BodyType;
pub extern fn zjoltBodySetMotionType(system: *PhysicsSystem, body: BodyId, motion_type: MotionType, activation: Activation) void;
pub extern fn zjoltBodyGetMotionType(system: *const PhysicsSystem, body: BodyId) MotionType;
pub extern fn zjoltBodySetMotionQuality(system: *PhysicsSystem, body: BodyId, quality: MotionQuality) void;
pub extern fn zjoltBodyGetMotionQuality(system: *const PhysicsSystem, body: BodyId) MotionQuality;
pub extern fn zjoltBodySetPositionAndRotation(system: *PhysicsSystem, body: BodyId, position: *const RVec3, rotation: ?*const Quat, activation: Activation) void;
pub extern fn zjoltBodyGetPositionAndRotation(system: *const PhysicsSystem, body: BodyId, out_position: ?*RVec3, out_rotation: ?*Quat) void;
pub extern fn zjoltBodyGetCenterOfMassPosition(system: *const PhysicsSystem, body: BodyId, out: *RVec3) void;
pub extern fn zjoltBodyGetWorldTransform(system: *const PhysicsSystem, body: BodyId, out: *RMat44) void;
pub extern fn zjoltBodyGetCenterOfMassTransform(system: *const PhysicsSystem, body: BodyId, out: *RMat44) void;
pub extern fn zjoltBodyGetInverseInertia(system: *const PhysicsSystem, body: BodyId, out: *Mat44) Result;
pub extern fn zjoltBodySetPositionAndRotationWhenChanged(system: *PhysicsSystem, body: BodyId, position: *const RVec3, rotation: ?*const Quat, activation: Activation) void;
pub extern fn zjoltBodyMoveKinematic(system: *PhysicsSystem, body: BodyId, target_position: *const RVec3, target_rotation: ?*const Quat, delta_time: f32) void;
pub extern fn zjoltBodySetPositionRotationAndVelocity(system: *PhysicsSystem, body: BodyId, position: *const RVec3, rotation: ?*const Quat, linear_velocity: *const Vec3, angular_velocity: *const Vec3) void;
pub extern fn zjoltBodySetLinearVelocity(system: *PhysicsSystem, body: BodyId, velocity: *const Vec3) void;
pub extern fn zjoltBodyGetLinearVelocity(system: *const PhysicsSystem, body: BodyId, out: *Vec3) void;
pub extern fn zjoltBodySetAngularVelocity(system: *PhysicsSystem, body: BodyId, velocity: *const Vec3) void;
pub extern fn zjoltBodyGetAngularVelocity(system: *const PhysicsSystem, body: BodyId, out: *Vec3) void;
pub extern fn zjoltBodySetLinearAndAngularVelocity(system: *PhysicsSystem, body: BodyId, linear_velocity: *const Vec3, angular_velocity: *const Vec3) void;
pub extern fn zjoltBodyGetLinearAndAngularVelocity(system: *const PhysicsSystem, body: BodyId, out_linear_velocity: ?*Vec3, out_angular_velocity: ?*Vec3) void;
pub extern fn zjoltBodyAddLinearVelocity(system: *PhysicsSystem, body: BodyId, linear_velocity: *const Vec3) void;
pub extern fn zjoltBodyAddLinearAndAngularVelocity(system: *PhysicsSystem, body: BodyId, linear_velocity: *const Vec3, angular_velocity: *const Vec3) void;
pub extern fn zjoltBodyGetPointVelocity(system: *const PhysicsSystem, body: BodyId, point: *const RVec3, out: *Vec3) void;
pub extern fn zjoltBodyAddForce(system: *PhysicsSystem, body: BodyId, force: *const Vec3) void;
pub extern fn zjoltBodyAddForceAtPoint(system: *PhysicsSystem, body: BodyId, force: *const Vec3, point: *const RVec3) void;
pub extern fn zjoltBodyAddTorque(system: *PhysicsSystem, body: BodyId, torque: *const Vec3) void;
pub extern fn zjoltBodyAddImpulse(system: *PhysicsSystem, body: BodyId, impulse: *const Vec3) void;
pub extern fn zjoltBodyAddImpulseAtPoint(system: *PhysicsSystem, body: BodyId, impulse: *const Vec3, point: *const RVec3) void;
pub extern fn zjoltBodyAddAngularImpulse(system: *PhysicsSystem, body: BodyId, angular_impulse: *const Vec3) void;
pub extern fn zjoltBodyApplyBuoyancyImpulse(system: *PhysicsSystem, body: BodyId, surface_position: *const RVec3, surface_normal: *const Vec3, buoyancy: f32, linear_drag: f32, angular_drag: f32, fluid_velocity: *const Vec3, gravity: *const Vec3, delta_time: f32) bool;
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
pub extern fn zjoltBodySetMaxLinearVelocity(system: *PhysicsSystem, body: BodyId, velocity: f32) void;
pub extern fn zjoltBodyGetMaxLinearVelocity(system: *const PhysicsSystem, body: BodyId) f32;
pub extern fn zjoltBodySetMaxAngularVelocity(system: *PhysicsSystem, body: BodyId, velocity: f32) void;
pub extern fn zjoltBodyGetMaxAngularVelocity(system: *const PhysicsSystem, body: BodyId) f32;
pub extern fn zjoltBodySetUseManifoldReduction(system: *PhysicsSystem, body: BodyId, use_reduction: bool) void;
pub extern fn zjoltBodyGetUseManifoldReduction(system: *const PhysicsSystem, body: BodyId) bool;
pub extern fn zjoltBodySetIsSensor(system: *PhysicsSystem, body: BodyId, is_sensor: bool) void;
pub extern fn zjoltBodyIsSensor(system: *const PhysicsSystem, body: BodyId) bool;
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
pub extern fn zjoltCollideShapeClosest(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, max_separation_distance: f32, filters: ?*const QueryFilters, out_hit: *CollideShapeHit, out_hit_any: *bool) Result;
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

//=============================================================================
// CharacterBase, on the virtual character
//=============================================================================

pub const CharacterId = u32;
pub const character_id_invalid: CharacterId = 0xffff_ffff;

pub extern fn zjoltCharacterGetId(character: *const Character) CharacterId;
pub extern fn zjoltCharacterGetUp(character: *const Character, out: *Vec3) void;
pub extern fn zjoltCharacterSetUp(character: *Character, up: *const Vec3) void;
pub extern fn zjoltCharacterSetMaxSlopeAngle(character: *Character, radians: f32) void;
pub extern fn zjoltCharacterGetCosMaxSlopeAngle(character: *const Character) f32;
pub extern fn zjoltCharacterIsSlopeTooSteep(character: *const Character, normal: *const Vec3) bool;
pub extern fn zjoltCharacterGetGroundMaterial(character: *const Character) ?*const PhysicsMaterial;
pub extern fn zjoltCharacterGetGroundSubShapeId(character: *const Character) SubShapeId;
pub extern fn zjoltCharacterGetMass(character: *const Character) f32;
pub extern fn zjoltCharacterSetMass(character: *Character, mass: f32) void;
pub extern fn zjoltCharacterGetMaxStrength(character: *const Character) f32;
pub extern fn zjoltCharacterSetMaxStrength(character: *Character, max_strength: f32) void;
pub extern fn zjoltCharacterGetPenetrationRecoverySpeed(character: *const Character) f32;
pub extern fn zjoltCharacterSetPenetrationRecoverySpeed(character: *Character, speed: f32) void;
pub extern fn zjoltCharacterGetEnhancedInternalEdgeRemoval(character: *const Character) bool;
pub extern fn zjoltCharacterSetEnhancedInternalEdgeRemoval(character: *Character, apply: bool) void;
pub extern fn zjoltCharacterGetCharacterPadding(character: *const Character) f32;
pub extern fn zjoltCharacterGetMaxNumHits(character: *const Character) u32;
pub extern fn zjoltCharacterSetMaxNumHits(character: *Character, max_hits: u32) void;
pub extern fn zjoltCharacterGetHitReductionCosMaxAngle(character: *const Character) f32;
pub extern fn zjoltCharacterSetHitReductionCosMaxAngle(character: *Character, cos_max_angle: f32) void;
pub extern fn zjoltCharacterGetMaxHitsExceeded(character: *const Character) bool;
pub extern fn zjoltCharacterGetShapeOffset(character: *const Character, out: *Vec3) void;
pub extern fn zjoltCharacterSetShapeOffset(character: *Character, offset: *const Vec3) void;
pub extern fn zjoltCharacterGetUserData(character: *const Character) u64;
pub extern fn zjoltCharacterSetUserData(character: *Character, user_data: u64) void;
pub extern fn zjoltCharacterCancelVelocityTowardsSteepSlopes(character: *const Character, desired_velocity: *const Vec3, out: *Vec3) void;
pub extern fn zjoltCharacterStartTrackingContactChanges(character: *Character) void;
pub extern fn zjoltCharacterFinishTrackingContactChanges(character: *Character) void;

//=============================================================================
// Stair walking and floor sticking, standalone
//=============================================================================

pub extern fn zjoltCharacterCanWalkStairs(character: *const Character, linear_velocity: *const Vec3) bool;
pub extern fn zjoltCharacterWalkStairs(character: *Character, delta_time: f32, step_up: *const Vec3, step_forward: *const Vec3, step_forward_test: *const Vec3, step_down_extra: *const Vec3, filters: ?*const QueryFilters, out_stepped: ?*bool) Result;
pub extern fn zjoltCharacterStickToFloor(character: *Character, step_down: *const Vec3, filters: ?*const QueryFilters, out_stuck: ?*bool) Result;
pub extern fn zjoltCharacterRefreshContacts(character: *Character, filters: ?*const QueryFilters) Result;

pub const CharacterContact = extern struct {
    body_b: BodyId,
    character_id_b: CharacterId,
    sub_shape_id_b: SubShapeId,
    position: RVec3,
    linear_velocity: Vec3,
    contact_normal: Vec3,
    surface_normal: Vec3,
    distance: f32,
    fraction: f32,
    motion_type_b: MotionType,
    is_sensor_b: bool,
    user_data: u64,
    material: ?*const PhysicsMaterial,
    had_collision: bool,
    was_discarded: bool,
    can_push_character: bool,
    is_back_facing_contact: bool,
};

pub extern fn zjoltCharacterGetActiveContacts(character: *const Character, out_contacts: ?[*]CharacterContact, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltCharacterHasCollidedWithBody(character: *const Character, body: BodyId) bool;
pub extern fn zjoltCharacterHasCollidedWithCharacter(character: *const Character, other_character_id: CharacterId) bool;

//=============================================================================
// CharacterContactListener
//=============================================================================

pub const CharacterContactSettings = extern struct {
    can_push_character: bool,
    can_receive_impulses: bool,
};

pub const CharacterContactListenerCallbacks = extern struct {
    on_adjust_body_velocity: ?*const fn (user: ?*anyopaque, character: CharacterId, body2: BodyId, io_linear_velocity: *Vec3, io_angular_velocity: *Vec3) callconv(.c) void = null,
    on_contact_validate: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact) callconv(.c) bool = null,
    on_contact_added: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact, io_settings: *CharacterContactSettings) callconv(.c) void = null,
    on_contact_persisted: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact, io_settings: *CharacterContactSettings) callconv(.c) void = null,
    on_contact_removed: ?*const fn (user: ?*anyopaque, character: CharacterId, body_id2: BodyId, sub_shape_id2: SubShapeId) callconv(.c) void = null,
    on_character_contact_validate: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact) callconv(.c) bool = null,
    on_character_contact_added: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact, io_settings: *CharacterContactSettings) callconv(.c) void = null,
    on_character_contact_persisted: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact, io_settings: *CharacterContactSettings) callconv(.c) void = null,
    on_character_contact_removed: ?*const fn (user: ?*anyopaque, character: CharacterId, other_character_id: CharacterId, sub_shape_id2: SubShapeId) callconv(.c) void = null,
    on_contact_solve: ?*const fn (user: ?*anyopaque, character: CharacterId, body_id2: BodyId, sub_shape_id2: SubShapeId, contact_position: *const RVec3, contact_normal: *const Vec3, contact_velocity: *const Vec3, contact_material: ?*const PhysicsMaterial, character_velocity: *const Vec3, io_new_character_velocity: *Vec3) callconv(.c) void = null,
    on_character_contact_solve: ?*const fn (user: ?*anyopaque, character: CharacterId, other_character: CharacterId, sub_shape_id2: SubShapeId, contact_position: *const RVec3, contact_normal: *const Vec3, contact_velocity: *const Vec3, contact_material: ?*const PhysicsMaterial, character_velocity: *const Vec3, io_new_character_velocity: *Vec3) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub extern fn zjoltCharacterContactListenerCreate(callbacks: *const CharacterContactListenerCallbacks, out: **CharacterContactListener) Result;
pub extern fn zjoltCharacterContactListenerDestroy(listener: ?*CharacterContactListener) void;
pub extern fn zjoltCharacterSetListener(character: *Character, listener: ?*CharacterContactListener) void;

//=============================================================================
// Character-vs-character collision
//=============================================================================

pub extern fn zjoltCharacterVsCharacterCollisionCreate(out: **CharacterVsCharacterCollision) Result;
pub extern fn zjoltCharacterVsCharacterCollisionDestroy(collision: ?*CharacterVsCharacterCollision) void;
pub extern fn zjoltCharacterVsCharacterCollisionAdd(collision: *CharacterVsCharacterCollision, character: *Character) void;
pub extern fn zjoltCharacterVsCharacterCollisionRemove(collision: *CharacterVsCharacterCollision, character: *const Character) void;
pub extern fn zjoltCharacterSetCharacterVsCharacterCollision(character: *Character, collision: ?*CharacterVsCharacterCollision) void;

//=============================================================================
// RigidCharacter
//=============================================================================

pub const RigidCharacterDesc = extern struct {
    shape: ?*const Shape,
    position: RVec3,
    rotation: Quat,
    user_data: u64,
    up: Vec3,
    max_slope_angle: f32,
    enhanced_internal_edge_removal: bool,
    layer: ObjectLayer,
    mass: f32,
    friction: f32,
    gravity_factor: f32,
    allowed_dofs: AllowedDofs,
};

pub extern fn zjoltRigidCharacterDescInit(desc: *RigidCharacterDesc) void;
pub extern fn zjoltRigidCharacterCreate(system: *PhysicsSystem, desc: *const RigidCharacterDesc, out: **RigidCharacter) Result;
pub extern fn zjoltRigidCharacterDestroy(character: ?*RigidCharacter) void;
pub extern fn zjoltRigidCharacterAddToPhysicsSystem(character: *RigidCharacter, activation: Activation) void;
pub extern fn zjoltRigidCharacterRemoveFromPhysicsSystem(character: *RigidCharacter) void;
pub extern fn zjoltRigidCharacterActivate(character: *RigidCharacter) void;
pub extern fn zjoltRigidCharacterPostSimulation(character: *RigidCharacter, max_separation_distance: f32) void;
pub extern fn zjoltRigidCharacterSetLinearAndAngularVelocity(character: *RigidCharacter, linear_velocity: *const Vec3, angular_velocity: *const Vec3) void;
pub extern fn zjoltRigidCharacterGetLinearVelocity(character: *const RigidCharacter, out: *Vec3) void;
pub extern fn zjoltRigidCharacterSetLinearVelocity(character: *RigidCharacter, velocity: *const Vec3) void;
pub extern fn zjoltRigidCharacterAddLinearVelocity(character: *RigidCharacter, velocity: *const Vec3) void;
pub extern fn zjoltRigidCharacterAddImpulse(character: *RigidCharacter, impulse: *const Vec3) void;
pub extern fn zjoltRigidCharacterGetBodyId(character: *const RigidCharacter) BodyId;
pub extern fn zjoltRigidCharacterGetPositionAndRotation(character: *const RigidCharacter, out_position: ?*RVec3, out_rotation: ?*Quat) void;
pub extern fn zjoltRigidCharacterSetPositionAndRotation(character: *RigidCharacter, position: *const RVec3, rotation: *const Quat, activation: Activation) void;
pub extern fn zjoltRigidCharacterGetPosition(character: *const RigidCharacter, out: *RVec3) void;
pub extern fn zjoltRigidCharacterSetPosition(character: *RigidCharacter, position: *const RVec3, activation: Activation) void;
pub extern fn zjoltRigidCharacterGetRotation(character: *const RigidCharacter, out: *Quat) void;
pub extern fn zjoltRigidCharacterSetRotation(character: *RigidCharacter, rotation: *const Quat, activation: Activation) void;
pub extern fn zjoltRigidCharacterGetCenterOfMassPosition(character: *const RigidCharacter, out: *RVec3) void;
pub extern fn zjoltRigidCharacterGetLayer(character: *const RigidCharacter) ObjectLayer;
pub extern fn zjoltRigidCharacterSetLayer(character: *RigidCharacter, layer: ObjectLayer) void;
pub extern fn zjoltRigidCharacterSetShape(character: *RigidCharacter, shape: *const Shape, max_penetration_depth: f32, out_changed: ?*bool) Result;
pub extern fn zjoltRigidCharacterGetShape(character: *const RigidCharacter) ?*const Shape;
pub extern fn zjoltRigidCharacterGetId(character: *const RigidCharacter) CharacterId;
pub extern fn zjoltRigidCharacterGetUp(character: *const RigidCharacter, out: *Vec3) void;
pub extern fn zjoltRigidCharacterSetUp(character: *RigidCharacter, up: *const Vec3) void;
pub extern fn zjoltRigidCharacterSetMaxSlopeAngle(character: *RigidCharacter, radians: f32) void;
pub extern fn zjoltRigidCharacterGetCosMaxSlopeAngle(character: *const RigidCharacter) f32;
pub extern fn zjoltRigidCharacterIsSlopeTooSteep(character: *const RigidCharacter, normal: *const Vec3) bool;
pub extern fn zjoltRigidCharacterGetGroundState(character: *const RigidCharacter) GroundState;
pub extern fn zjoltRigidCharacterIsSupported(character: *const RigidCharacter) bool;
pub extern fn zjoltRigidCharacterGetGroundPosition(character: *const RigidCharacter, out: *RVec3) void;
pub extern fn zjoltRigidCharacterGetGroundNormal(character: *const RigidCharacter, out: *Vec3) void;
pub extern fn zjoltRigidCharacterGetGroundVelocity(character: *const RigidCharacter, out: *Vec3) void;
pub extern fn zjoltRigidCharacterGetGroundMaterial(character: *const RigidCharacter) ?*const PhysicsMaterial;
pub extern fn zjoltRigidCharacterGetGroundBodyId(character: *const RigidCharacter) BodyId;
pub extern fn zjoltRigidCharacterGetGroundSubShapeId(character: *const RigidCharacter) SubShapeId;
pub extern fn zjoltRigidCharacterGetGroundUserData(character: *const RigidCharacter) u64;

pub extern fn zjoltPhysicsSystemGetMaxBodies(system: *const PhysicsSystem) u32;
pub extern fn zjoltPhysicsSystemGetBodyStats(system: *const PhysicsSystem, out: *BodyStats) void;
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
// Soft bodies
//=============================================================================

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
pub extern fn zjoltSoftBodySharedSettingsOptimize(settings: *SoftBodySharedSettings) void;

pub extern fn zjoltSoftBodyDescInit(desc: *SoftBodyDesc) void;
pub extern fn zjoltSoftBodyCreate(system: *PhysicsSystem, desc: *const SoftBodyDesc, out: *BodyId) Result;
pub extern fn zjoltSoftBodyCreateAndAdd(system: *PhysicsSystem, desc: *const SoftBodyDesc, activation: Activation, out: *BodyId) Result;

pub extern fn zjoltSoftBodyGetVertexStates(system: *const PhysicsSystem, body: BodyId, out_states: ?[*]SoftBodyVertexState, capacity: u32, out_count: *u32) Result;
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

pub extern fn zjoltVehicleConstraintGetForwardInput(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetBrakeInput(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetEngineRpm(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetCurrentGear(constraint: *const VehicleConstraint) i32;
pub extern fn zjoltVehicleConstraintIsSwitchingGear(constraint: *const VehicleConstraint) bool;
pub extern fn zjoltVehicleConstraintGetClutchFriction(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetGearRatio(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintSetGear(constraint: *VehicleConstraint, gear: i32, clutch_friction: f32) Result;

pub extern fn zjoltVehicleConstraintSetWheeledDriverInput(constraint: *VehicleConstraint, forward: f32, right: f32, brake: f32, hand_brake: f32) Result;
pub extern fn zjoltVehicleConstraintGetWheeledRightInput(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetWheeledHandBrakeInput(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintSetTrackedDriverInput(constraint: *VehicleConstraint, forward: f32, left_ratio: f32, right_ratio: f32, brake: f32) Result;
pub extern fn zjoltVehicleConstraintGetTrackedLeftRatio(constraint: *const VehicleConstraint) f32;
pub extern fn zjoltVehicleConstraintGetTrackedRightRatio(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintSetMotorcycleLeanControllerEnabled(constraint: *VehicleConstraint, enabled: bool) Result;
pub extern fn zjoltVehicleConstraintIsMotorcycleLeanControllerEnabled(constraint: *const VehicleConstraint) bool;
pub extern fn zjoltVehicleConstraintSetMotorcycleLeanSteeringLimitEnabled(constraint: *VehicleConstraint, enabled: bool) Result;
pub extern fn zjoltVehicleConstraintIsMotorcycleLeanSteeringLimitEnabled(constraint: *const VehicleConstraint) bool;
pub extern fn zjoltVehicleConstraintGetMotorcycleWheelBase(constraint: *const VehicleConstraint) f32;

pub extern fn zjoltVehicleConstraintOverrideGravity(constraint: *VehicleConstraint, gravity: *const Vec3) void;
pub extern fn zjoltVehicleConstraintResetGravityOverride(constraint: *VehicleConstraint) void;
pub extern fn zjoltVehicleConstraintIsGravityOverridden(constraint: *const VehicleConstraint) bool;
pub extern fn zjoltVehicleConstraintGetGravityOverride(constraint: *const VehicleConstraint, out: *Vec3) void;

pub extern fn zjoltVehicleConstraintGetNumStepsBetweenCollisionTestActive(constraint: *const VehicleConstraint) u32;
pub extern fn zjoltVehicleConstraintSetNumStepsBetweenCollisionTestActive(constraint: *VehicleConstraint, steps: u32) void;
pub extern fn zjoltVehicleConstraintGetNumStepsBetweenCollisionTestInactive(constraint: *const VehicleConstraint) u32;
pub extern fn zjoltVehicleConstraintSetNumStepsBetweenCollisionTestInactive(constraint: *VehicleConstraint, steps: u32) void;

pub extern fn zjoltVehicleConstraintGetWheelContactSubShapeId(constraint: *const VehicleConstraint, wheel_index: u32) SubShapeId;
pub extern fn zjoltVehicleConstraintHasWheelHitHardPoint(constraint: *const VehicleConstraint, wheel_index: u32) bool;
pub extern fn zjoltVehicleConstraintGetWheelSuspensionLambda(constraint: *const VehicleConstraint, wheel_index: u32) f32;
pub extern fn zjoltVehicleConstraintGetWheelLongitudinalLambda(constraint: *const VehicleConstraint, wheel_index: u32) f32;
pub extern fn zjoltVehicleConstraintGetWheelLateralLambda(constraint: *const VehicleConstraint, wheel_index: u32) f32;
pub extern fn zjoltVehicleConstraintGetWheelLocalBasis(constraint: *const VehicleConstraint, wheel_index: u32, out_forward: *Vec3, out_up: *Vec3, out_right: *Vec3) void;
pub extern fn zjoltVehicleConstraintGetWheelLocalTransform(constraint: *const VehicleConstraint, wheel_index: u32, wheel_right: *const Vec3, wheel_up: *const Vec3, out_position: *Vec3, out_rotation: *Quat) void;
pub extern fn zjoltVehicleConstraintGetWheelWorldTransform(constraint: *const VehicleConstraint, wheel_index: u32, wheel_right: *const Vec3, wheel_up: *const Vec3, out_position: *RVec3, out_rotation: *Quat) void;

pub extern fn zjoltVehicleConstraintAsConstraint(constraint: *const VehicleConstraint) ?*Constraint;

pub extern fn zjoltPhysicsSystemSaveBodyStateLocked(system: *const PhysicsSystem, body: *const Body, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;
pub extern fn zjoltPhysicsSystemRestoreBodyStateLocked(system: *PhysicsSystem, body: *Body, data: [*]const u8, size: usize) Result;

//=============================================================================
// Debug draw
//=============================================================================

pub const DebugLine = extern struct {
    from: RVec3,
    to: RVec3,
    color: Color,
};

pub const DebugTriangle = extern struct {
    v1: RVec3,
    v2: RVec3,
    v3: RVec3,
    color: Color,
    cast_shadow: bool,
};

pub const DebugText = extern struct {
    position: RVec3,
    color: Color,
    height: f32,
    text: [debug_text_max_length]u8,
    text_length: u32,
};

/// Mirrors JPH::BodyManager::EShapeColor.
pub const ShapeColor = enum(c_int) {
    instance_color = 0,
    shape_type_color = 1,
    motion_type_color = 2,
    sleep_color = 3,
    island_color = 4,
    material_color = 5,
};

/// Subset of JPH::BodyManager::DrawSettings — see ffi/zjolt_debug.h for what
/// was left out and why.
pub const DebugDrawBodiesSettings = extern struct {
    draw_get_support_function: bool,
    draw_support_direction: bool,
    draw_get_supporting_face: bool,
    draw_shape: bool,
    draw_shape_wireframe: bool,
    shape_color: ShapeColor,
    draw_bounding_box: bool,
    draw_center_of_mass_transform: bool,
    draw_world_transform: bool,
    draw_velocity: bool,
    draw_mass_and_inertia: bool,
    draw_sleep_stats: bool,
};

pub extern fn zjoltDebugRendererCreate(out: **DebugRenderer) Result;
pub extern fn zjoltDebugRendererDestroy(renderer: ?*DebugRenderer) Result;
pub extern fn zjoltDebugRendererClear(renderer: *DebugRenderer) Result;
pub extern fn zjoltDebugRendererSetCameraPosition(renderer: *DebugRenderer, position: *const RVec3) Result;
pub extern fn zjoltDebugRendererGetLines(renderer: *const DebugRenderer, lines: ?[*]DebugLine, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltDebugRendererGetTriangles(renderer: *const DebugRenderer, triangles: ?[*]DebugTriangle, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltDebugRendererGetTexts(renderer: *const DebugRenderer, texts: ?[*]DebugText, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltDebugDrawBodiesSettingsInit(settings: *DebugDrawBodiesSettings) Result;
pub extern fn zjoltPhysicsSystemDrawBodies(system: *PhysicsSystem, settings: *const DebugDrawBodiesSettings, renderer: *DebugRenderer) Result;
pub extern fn zjoltPhysicsSystemDrawConstraints(system: *PhysicsSystem, renderer: *DebugRenderer) Result;
pub extern fn zjoltPhysicsSystemDrawConstraintLimits(system: *PhysicsSystem, renderer: *DebugRenderer) Result;
pub extern fn zjoltPhysicsSystemDrawConstraintReferenceFrame(system: *PhysicsSystem, renderer: *DebugRenderer) Result;
// Ragdoll
//=============================================================================

pub const Skeleton = opaque {};
pub const SkeletonPose = opaque {};
pub const RagdollSettings = opaque {};
pub const Ragdoll = opaque {};

//=============================================================================
// Constraints
//=============================================================================

pub const Constraint = opaque {};
pub const PathConstraintPath = opaque {};

/// The body id that stands for "the world" when creating a constraint. Spelled
/// as the invalid id because that is what Jolt's own fixed-to-world body
/// carries, and because no real body can collide with the value.
pub const body_id_world: BodyId = 0xffff_ffff;

pub const six_dof_axis_count: u32 = 6;
pub const six_dof_translation_axis_count: u32 = 3;

pub const ConstraintSubType = enum(c_int) {
    other = 0,
    fixed = 1,
    point = 2,
    hinge = 3,
    slider = 4,
    distance = 5,
    cone = 6,
    swing_twist = 7,
    six_dof = 8,
    path = 9,
    gear = 10,
    rack_and_pinion = 11,
    pulley = 12,
};

pub const ConstraintSpace = enum(c_int) {
    local_to_body_com = 0,
    world = 1,
};

pub const MotorState = enum(c_int) {
    off = 0,
    velocity = 1,
    position = 2,
    position_and_velocity = 3,
};

pub const SpringMode = enum(c_int) {
    frequency_and_damping = 0,
    stiffness_and_damping = 1,
    mass_normalized_stiffness_and_damping = 2,
};

pub const SwingType = enum(c_int) {
    cone = 0,
    pyramid = 1,
};

pub const RagdollConstraintDesc = extern struct {
    position1: RVec3,
    twist_axis1: Vec3,
    plane_axis1: Vec3,
    position2: RVec3,
    twist_axis2: Vec3,
    plane_axis2: Vec3,
    swing_type: SwingType,
    normal_half_cone_angle: f32,
    plane_half_cone_angle: f32,
    twist_min_angle: f32,
    twist_max_angle: f32,
    max_friction_torque: f32,
};

pub const RagdollPartDesc = extern struct {
    body: BodyDesc,
    to_parent: ?*const RagdollConstraintDesc,
};

pub extern fn zjoltSkeletonCreate(out: **Skeleton) Result;
pub extern fn zjoltSkeletonAddRef(skeleton: *const Skeleton) void;
pub extern fn zjoltSkeletonRelease(skeleton: *const Skeleton) void;
pub extern fn zjoltSkeletonGetRefCount(skeleton: *const Skeleton) u32;
pub extern fn zjoltSkeletonAddJoint(skeleton: *Skeleton, name: ?[*:0]const u8, parent_index: i32, out_index: *u32) Result;
pub extern fn zjoltSkeletonGetJointCount(skeleton: *const Skeleton) u32;
pub extern fn zjoltSkeletonGetJointIndex(skeleton: *const Skeleton, name: ?[*:0]const u8) i32;
pub extern fn zjoltSkeletonGetJointParentIndex(skeleton: *const Skeleton, index: u32) i32;
pub extern fn zjoltSkeletonGetJointName(skeleton: *const Skeleton, index: u32) [*:0]const u8;
pub extern fn zjoltSkeletonAreJointsCorrectlyOrdered(skeleton: *const Skeleton) bool;

pub extern fn zjoltSkeletonPoseCreate(out: **SkeletonPose) Result;
pub extern fn zjoltSkeletonPoseDestroy(pose: *SkeletonPose) void;
pub extern fn zjoltSkeletonPoseSetSkeleton(pose: *SkeletonPose, skeleton: *const Skeleton) Result;
pub extern fn zjoltSkeletonPoseGetSkeleton(pose: *const SkeletonPose) ?*const Skeleton;
pub extern fn zjoltSkeletonPoseGetJointCount(pose: *const SkeletonPose) u32;
pub extern fn zjoltSkeletonPoseSetRootOffset(pose: *SkeletonPose, offset: *const RVec3) void;
pub extern fn zjoltSkeletonPoseGetRootOffset(pose: *const SkeletonPose, out: *RVec3) void;
pub extern fn zjoltSkeletonPoseSetJoints(pose: *SkeletonPose, rotations: ?[*]const Quat, translations: ?[*]const Vec3, count: u32) Result;
pub extern fn zjoltSkeletonPoseGetJoints(pose: *const SkeletonPose, out_rotations: ?[*]Quat, out_translations: ?[*]Vec3, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltSkeletonPoseCalculateJointMatrices(pose: *SkeletonPose) Result;
pub extern fn zjoltSkeletonPoseCalculateJointStates(pose: *SkeletonPose) Result;

pub extern fn zjoltRagdollSettingsCreate(out: **RagdollSettings) Result;
pub extern fn zjoltRagdollSettingsAddRef(settings: *const RagdollSettings) void;
pub extern fn zjoltRagdollSettingsRelease(settings: *const RagdollSettings) void;
pub extern fn zjoltRagdollSettingsGetRefCount(settings: *const RagdollSettings) u32;
pub extern fn zjoltRagdollSettingsBuild(settings: *RagdollSettings, skeleton: *const Skeleton, parts: [*]const RagdollPartDesc, part_count: u32) Result;
pub extern fn zjoltRagdollSettingsStabilize(settings: *RagdollSettings) bool;
pub extern fn zjoltRagdollSettingsDisableParentChildCollisions(settings: *RagdollSettings) void;
pub extern fn zjoltRagdollSettingsCalculateBodyIndexToConstraintIndex(settings: *RagdollSettings) void;
pub extern fn zjoltRagdollSettingsCreateRagdoll(settings: *const RagdollSettings, system: *PhysicsSystem, collision_group: u32, user_data: u64, out: **Ragdoll) Result;

pub extern fn zjoltRagdollAddRef(ragdoll: *const Ragdoll) void;
pub extern fn zjoltRagdollRelease(ragdoll: *const Ragdoll) void;
pub extern fn zjoltRagdollGetRefCount(ragdoll: *const Ragdoll) u32;
pub extern fn zjoltRagdollAddToPhysicsSystem(ragdoll: *Ragdoll, activation: Activation, lock_bodies: bool) void;
pub extern fn zjoltRagdollRemoveFromPhysicsSystem(ragdoll: *Ragdoll, lock_bodies: bool) void;
pub extern fn zjoltRagdollGetBodyIds(ragdoll: *const Ragdoll, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltRagdollActivate(ragdoll: *Ragdoll, lock_bodies: bool) void;
pub extern fn zjoltRagdollIsActive(ragdoll: *const Ragdoll, lock_bodies: bool) bool;
pub extern fn zjoltRagdollResetWarmStart(ragdoll: *Ragdoll) void;
pub extern fn zjoltRagdollSetPose(ragdoll: *Ragdoll, pose: *const SkeletonPose, lock_bodies: bool) Result;
pub extern fn zjoltRagdollGetPose(ragdoll: *Ragdoll, pose: *SkeletonPose, lock_bodies: bool) Result;
pub extern fn zjoltRagdollDriveToPoseUsingKinematics(ragdoll: *Ragdoll, pose: *const SkeletonPose, delta_time: f32, lock_bodies: bool) Result;
pub extern fn zjoltRagdollDriveToPoseUsingMotors(ragdoll: *Ragdoll, pose: *const SkeletonPose) Result;
pub extern fn zjoltRagdollDriveToPoseUsingMotorsWithVelocity(ragdoll: *Ragdoll, prev_pose: *const SkeletonPose, pose: *const SkeletonPose, delta_time: f32) Result;
// Hair, and the compute backend it runs on
//
// Declared unconditionally, like everything else here. `options.cpu_compute`
// says whether the CPU backend has any contents; it does not say whether these
// functions exist, because a header that changes shape with a `-D` flag cannot
// be checked against this file at all.
//=============================================================================

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
};

pub const HairGradient = extern struct {
    min: f32,
    max: f32,
    min_fraction: f32,
    max_fraction: f32,
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
pub const SixDofAxis = enum(c_int) {
    translation_x = 0,
    translation_y = 1,
    translation_z = 2,
    rotation_x = 3,
    rotation_y = 4,
    rotation_z = 5,
};

pub const PathRotationConstraintType = enum(c_int) {
    free = 0,
    constrain_around_tangent = 1,
    constrain_around_normal = 2,
    constrain_around_binormal = 3,
    constrain_to_path = 4,
    fully_constrained = 5,
};

pub const SpringSettings = extern struct {
    mode: SpringMode = .frequency_and_damping,
    frequency_or_stiffness: f32 = 0,
    damping: f32 = 0,
};

pub const MotorSettings = extern struct {
    spring: SpringSettings = .{},
    min_force_limit: f32 = -std.math.floatMax(f32),
    max_force_limit: f32 = std.math.floatMax(f32),
    min_torque_limit: f32 = -std.math.floatMax(f32),
    max_torque_limit: f32 = std.math.floatMax(f32),
};

pub const FixedConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    auto_detect_point: bool = false,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    axis_x1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    axis_y1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    axis_x2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    axis_y2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
};

pub const PointConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
};

pub const HingeConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    hinge_axis1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    normal_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    hinge_axis2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    normal_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    limits_min: f32 = -std.math.pi,
    limits_max: f32 = std.math.pi,
    limits_spring: SpringSettings = .{},
    max_friction_torque: f32 = 0,
    motor: MotorSettings = .{},
};

pub const SliderConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    auto_detect_point: bool = false,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    slider_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    normal_axis1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    slider_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    normal_axis2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    limits_min: f32 = -std.math.floatMax(f32),
    limits_max: f32 = std.math.floatMax(f32),
    limits_spring: SpringSettings = .{},
    max_friction_force: f32 = 0,
    motor: MotorSettings = .{},
};

pub const DistanceConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    min_distance: f32 = -1,
    max_distance: f32 = -1,
    limits_spring: SpringSettings = .{},
};

pub const ConeConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    twist_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    twist_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    half_cone_angle: f32 = 0,
};

pub const SwingTwistConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    position1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    twist_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    plane_axis1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    position2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    twist_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    plane_axis2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    swing_type: SwingType = .cone,
    normal_half_cone_angle: f32 = 0,
    plane_half_cone_angle: f32 = 0,
    twist_min_angle: f32 = 0,
    twist_max_angle: f32 = 0,
    max_friction_torque: f32 = 0,
    swing_motor: MotorSettings = .{},
    twist_motor: MotorSettings = .{},
};

pub const SixDofConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    position1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    axis_x1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    axis_y1: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    position2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    axis_x2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    axis_y2: Vec3 = .{ .x = 0, .y = 1, .z = 0 },
    swing_type: SwingType = .cone,
    max_friction: [six_dof_axis_count]f32 = @splat(0),
    limit_min: [six_dof_axis_count]f32 = @splat(-std.math.floatMax(f32)),
    limit_max: [six_dof_axis_count]f32 = @splat(std.math.floatMax(f32)),
    limits_spring: [six_dof_translation_axis_count]SpringSettings = @splat(.{}),
    motor: [six_dof_axis_count]MotorSettings = @splat(.{}),
};

pub const GearConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    hinge_axis1: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    hinge_axis2: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    ratio: f32 = 1,
};

pub const RackAndPinionConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    hinge_axis: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    slider_axis: Vec3 = .{ .x = 1, .y = 0, .z = 0 },
    ratio: f32 = 1,
};

pub const PulleyConstraintDesc = extern struct {
    space: ConstraintSpace = .world,
    body_point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    fixed_point1: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    body_point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    fixed_point2: RVec3 = .{ .x = 0, .y = 0, .z = 0 },
    ratio: f32 = 1,
    min_length: f32 = 0,
    max_length: f32 = -1,
};

pub const PathPoint = extern struct {
    position: Vec3,
    tangent: Vec3,
    normal: Vec3,
};

pub const PathConstraintDesc = extern struct {
    path: ?*const PathConstraintPath = null,
    path_position: Vec3 = .{ .x = 0, .y = 0, .z = 0 },
    path_rotation: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 },
    path_fraction: f32 = 0,
    max_friction_force: f32 = 0,
    motor: MotorSettings = .{},
    rotation_constraint_type: PathRotationConstraintType = .free,
};

pub extern fn zjoltPathConstraintPathCreateHermite(points: [*]const PathPoint, count: u32, is_looping: bool, out: **PathConstraintPath) Result;
pub extern fn zjoltPathConstraintPathAddRef(path: *const PathConstraintPath) void;
pub extern fn zjoltPathConstraintPathRelease(path: *const PathConstraintPath) void;
pub extern fn zjoltPathConstraintPathGetRefCount(path: *const PathConstraintPath) u32;
pub extern fn zjoltPathConstraintPathIsLooping(path: *const PathConstraintPath) bool;
pub extern fn zjoltPathConstraintPathGetMaxFraction(path: *const PathConstraintPath) f32;
pub extern fn zjoltPathConstraintPathGetClosestPoint(path: *const PathConstraintPath, position: *const Vec3, fraction_hint: f32, out_fraction: *f32) Result;
pub extern fn zjoltPathConstraintPathGetPointOnPath(path: *const PathConstraintPath, fraction: f32, out_position: ?*Vec3, out_tangent: ?*Vec3, out_normal: ?*Vec3, out_binormal: ?*Vec3) Result;

pub extern fn zjoltConstraintCreateFixed(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const FixedConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreatePoint(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const PointConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreateHinge(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const HingeConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreateSlider(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const SliderConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreateDistance(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const DistanceConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreateCone(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const ConeConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreateSwingTwist(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const SwingTwistConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreateSixDof(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const SixDofConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreateGear(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const GearConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreateRackAndPinion(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const RackAndPinionConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreatePulley(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const PulleyConstraintDesc, out: **Constraint) Result;
pub extern fn zjoltConstraintCreatePath(system: *PhysicsSystem, body1: BodyId, body2: BodyId, desc: *const PathConstraintDesc, out: **Constraint) Result;

pub extern fn zjoltConstraintAddRef(constraint: *const Constraint) void;
pub extern fn zjoltConstraintRelease(constraint: *const Constraint) void;
pub extern fn zjoltConstraintGetRefCount(constraint: *const Constraint) u32;

pub extern fn zjoltConstraintAdd(system: *PhysicsSystem, constraint: *Constraint) Result;
pub extern fn zjoltConstraintRemove(system: *PhysicsSystem, constraint: *Constraint) Result;
pub extern fn zjoltConstraintIsAdded(system: *const PhysicsSystem, constraint: *const Constraint) bool;
pub extern fn zjoltPhysicsSystemGetNumConstraints(system: *const PhysicsSystem) u32;

pub extern fn zjoltConstraintGetSubType(constraint: *const Constraint) ConstraintSubType;
pub extern fn zjoltConstraintSetEnabled(constraint: *Constraint, enabled: bool) void;
pub extern fn zjoltConstraintIsEnabled(constraint: *const Constraint) bool;
pub extern fn zjoltConstraintIsActive(constraint: *const Constraint) bool;
pub extern fn zjoltConstraintSetUserData(constraint: *Constraint, user_data: u64) void;
pub extern fn zjoltConstraintGetUserData(constraint: *const Constraint) u64;
pub extern fn zjoltConstraintSetPriority(constraint: *Constraint, priority: u32) void;
pub extern fn zjoltConstraintGetPriority(constraint: *const Constraint) u32;
pub extern fn zjoltConstraintSetNumVelocityStepsOverride(constraint: *Constraint, steps: u32) Result;
pub extern fn zjoltConstraintGetNumVelocityStepsOverride(constraint: *const Constraint) u32;
pub extern fn zjoltConstraintSetNumPositionStepsOverride(constraint: *Constraint, steps: u32) Result;
pub extern fn zjoltConstraintGetNumPositionStepsOverride(constraint: *const Constraint) u32;
pub extern fn zjoltConstraintGetBodies(constraint: *const Constraint, out_body1: ?*BodyId, out_body2: ?*BodyId) Result;
pub extern fn zjoltConstraintResetWarmStart(constraint: *Constraint) void;

pub extern fn zjoltFixedConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltFixedConstraintGetTotalLambdaRotation(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltPointConstraintSetPoint1(constraint: *Constraint, space: ConstraintSpace, point: *const RVec3) Result;
pub extern fn zjoltPointConstraintSetPoint2(constraint: *Constraint, space: ConstraintSpace, point: *const RVec3) Result;
pub extern fn zjoltPointConstraintGetLocalSpacePoint1(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltPointConstraintGetLocalSpacePoint2(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltPointConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltHingeConstraintGetCurrentAngle(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltHingeConstraintSetLimits(constraint: *Constraint, min: f32, max: f32) Result;
pub extern fn zjoltHingeConstraintGetLimits(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;
pub extern fn zjoltHingeConstraintHasLimits(constraint: *const Constraint, out: *bool) Result;
pub extern fn zjoltHingeConstraintSetLimitsSpringSettings(constraint: *Constraint, spring: *const SpringSettings) Result;
pub extern fn zjoltHingeConstraintGetLimitsSpringSettings(constraint: *const Constraint, out: *SpringSettings) Result;
pub extern fn zjoltHingeConstraintSetMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;
pub extern fn zjoltHingeConstraintGetMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;
pub extern fn zjoltHingeConstraintSetMotorState(constraint: *Constraint, state: MotorState) Result;
pub extern fn zjoltHingeConstraintGetMotorState(constraint: *const Constraint, out: *MotorState) Result;
pub extern fn zjoltHingeConstraintSetTargetAngularVelocity(constraint: *Constraint, angular_velocity: f32) Result;
pub extern fn zjoltHingeConstraintGetTargetAngularVelocity(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltHingeConstraintSetTargetAngle(constraint: *Constraint, angle: f32) Result;
pub extern fn zjoltHingeConstraintGetTargetAngle(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltHingeConstraintSetTargetOrientation(constraint: *Constraint, orientation: *const Quat) Result;
pub extern fn zjoltHingeConstraintSetMaxFrictionTorque(constraint: *Constraint, torque: f32) Result;
pub extern fn zjoltHingeConstraintGetMaxFrictionTorque(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltHingeConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltHingeConstraintGetTotalLambdaMotor(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSliderConstraintGetCurrentPosition(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltSliderConstraintSetLimits(constraint: *Constraint, min: f32, max: f32) Result;
pub extern fn zjoltSliderConstraintGetLimits(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;
pub extern fn zjoltSliderConstraintHasLimits(constraint: *const Constraint, out: *bool) Result;
pub extern fn zjoltSliderConstraintSetLimitsSpringSettings(constraint: *Constraint, spring: *const SpringSettings) Result;
pub extern fn zjoltSliderConstraintGetLimitsSpringSettings(constraint: *const Constraint, out: *SpringSettings) Result;
pub extern fn zjoltSliderConstraintSetMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;
pub extern fn zjoltSliderConstraintGetMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;
pub extern fn zjoltSliderConstraintSetMotorState(constraint: *Constraint, state: MotorState) Result;
pub extern fn zjoltSliderConstraintGetMotorState(constraint: *const Constraint, out: *MotorState) Result;
pub extern fn zjoltSliderConstraintSetTargetVelocity(constraint: *Constraint, velocity: f32) Result;
pub extern fn zjoltSliderConstraintGetTargetVelocity(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltSliderConstraintSetTargetPosition(constraint: *Constraint, position: f32) Result;
pub extern fn zjoltSliderConstraintGetTargetPosition(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltSliderConstraintSetMaxFrictionForce(constraint: *Constraint, force: f32) Result;
pub extern fn zjoltSliderConstraintGetMaxFrictionForce(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltSliderConstraintGetTotalLambdaMotor(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltDistanceConstraintSetDistance(constraint: *Constraint, min: f32, max: f32) Result;
pub extern fn zjoltDistanceConstraintGetDistance(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;
pub extern fn zjoltDistanceConstraintSetLimitsSpringSettings(constraint: *Constraint, spring: *const SpringSettings) Result;
pub extern fn zjoltDistanceConstraintGetLimitsSpringSettings(constraint: *const Constraint, out: *SpringSettings) Result;
pub extern fn zjoltDistanceConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltConeConstraintSetHalfConeAngle(constraint: *Constraint, half_cone_angle: f32) Result;
pub extern fn zjoltConeConstraintGetCosHalfConeAngle(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltConeConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltConeConstraintGetTotalLambdaRotation(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltSwingTwistConstraintSetNormalHalfConeAngle(constraint: *Constraint, angle: f32) Result;
pub extern fn zjoltSwingTwistConstraintGetNormalHalfConeAngle(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltSwingTwistConstraintSetPlaneHalfConeAngle(constraint: *Constraint, angle: f32) Result;
pub extern fn zjoltSwingTwistConstraintGetPlaneHalfConeAngle(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltSwingTwistConstraintSetTwistLimits(constraint: *Constraint, min: f32, max: f32) Result;
pub extern fn zjoltSwingTwistConstraintGetTwistLimits(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;
pub extern fn zjoltSwingTwistConstraintSetSwingMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;
pub extern fn zjoltSwingTwistConstraintGetSwingMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;
pub extern fn zjoltSwingTwistConstraintSetSwingMotorState(constraint: *Constraint, state: MotorState) Result;
pub extern fn zjoltSwingTwistConstraintGetSwingMotorState(constraint: *const Constraint, out: *MotorState) Result;
pub extern fn zjoltSwingTwistConstraintSetTwistMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;
pub extern fn zjoltSwingTwistConstraintGetTwistMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;
pub extern fn zjoltSwingTwistConstraintSetTwistMotorState(constraint: *Constraint, state: MotorState) Result;
pub extern fn zjoltSwingTwistConstraintGetTwistMotorState(constraint: *const Constraint, out: *MotorState) Result;
pub extern fn zjoltSwingTwistConstraintSetMaxFrictionTorque(constraint: *Constraint, torque: f32) Result;
pub extern fn zjoltSwingTwistConstraintGetMaxFrictionTorque(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltSwingTwistConstraintSetTargetAngularVelocity(constraint: *Constraint, angular_velocity: *const Vec3) Result;
pub extern fn zjoltSwingTwistConstraintGetTargetAngularVelocity(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltSwingTwistConstraintSetTargetOrientation(constraint: *Constraint, orientation: *const Quat) Result;
pub extern fn zjoltSwingTwistConstraintGetTargetOrientation(constraint: *const Constraint, out: *Quat) Result;
pub extern fn zjoltSwingTwistConstraintGetRotationInConstraintSpace(constraint: *const Constraint, out: *Quat) Result;
pub extern fn zjoltSwingTwistConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltSwingTwistConstraintGetTotalLambdaMotor(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltSixDofConstraintSetTranslationLimits(constraint: *Constraint, min: *const Vec3, max: *const Vec3) Result;
pub extern fn zjoltSixDofConstraintSetRotationLimits(constraint: *Constraint, min: *const Vec3, max: *const Vec3) Result;
pub extern fn zjoltSixDofConstraintGetLimits(constraint: *const Constraint, axis: SixDofAxis, out_min: ?*f32, out_max: ?*f32) Result;
pub extern fn zjoltSixDofConstraintIsFixedAxis(constraint: *const Constraint, axis: SixDofAxis, out: *bool) Result;
pub extern fn zjoltSixDofConstraintIsFreeAxis(constraint: *const Constraint, axis: SixDofAxis, out: *bool) Result;
pub extern fn zjoltSixDofConstraintSetMaxFriction(constraint: *Constraint, axis: SixDofAxis, friction: f32) Result;
pub extern fn zjoltSixDofConstraintGetMaxFriction(constraint: *const Constraint, axis: SixDofAxis, out: *f32) Result;
pub extern fn zjoltSixDofConstraintSetMotorSettings(constraint: *Constraint, axis: SixDofAxis, motor: *const MotorSettings) Result;
pub extern fn zjoltSixDofConstraintGetMotorSettings(constraint: *const Constraint, axis: SixDofAxis, out: *MotorSettings) Result;
pub extern fn zjoltSixDofConstraintSetMotorState(constraint: *Constraint, axis: SixDofAxis, state: MotorState) Result;
pub extern fn zjoltSixDofConstraintGetMotorState(constraint: *const Constraint, axis: SixDofAxis, out: *MotorState) Result;
pub extern fn zjoltSixDofConstraintSetLimitsSpringSettings(constraint: *Constraint, axis: SixDofAxis, spring: *const SpringSettings) Result;
pub extern fn zjoltSixDofConstraintGetLimitsSpringSettings(constraint: *const Constraint, axis: SixDofAxis, out: *SpringSettings) Result;
pub extern fn zjoltSixDofConstraintSetTargetVelocity(constraint: *Constraint, velocity: *const Vec3) Result;
pub extern fn zjoltSixDofConstraintGetTargetVelocity(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltSixDofConstraintSetTargetAngularVelocity(constraint: *Constraint, angular_velocity: *const Vec3) Result;
pub extern fn zjoltSixDofConstraintGetTargetAngularVelocity(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltSixDofConstraintSetTargetPosition(constraint: *Constraint, position: *const Vec3) Result;
pub extern fn zjoltSixDofConstraintGetTargetPosition(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltSixDofConstraintSetTargetOrientation(constraint: *Constraint, orientation: *const Quat) Result;
pub extern fn zjoltSixDofConstraintGetTargetOrientation(constraint: *const Constraint, out: *Quat) Result;
pub extern fn zjoltSixDofConstraintGetRotationInConstraintSpace(constraint: *const Constraint, out: *Quat) Result;
pub extern fn zjoltSixDofConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltSixDofConstraintGetTotalLambdaRotation(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltSixDofConstraintGetTotalLambdaMotorTranslation(constraint: *const Constraint, out: *Vec3) Result;
pub extern fn zjoltSixDofConstraintGetTotalLambdaMotorRotation(constraint: *const Constraint, out: *Vec3) Result;

pub extern fn zjoltGearConstraintSetConstraints(constraint: *Constraint, gear1: ?*Constraint, gear2: ?*Constraint) Result;
pub extern fn zjoltGearConstraintGetTotalLambda(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltRackAndPinionConstraintSetConstraints(constraint: *Constraint, pinion: ?*Constraint, rack: ?*Constraint) Result;
pub extern fn zjoltRackAndPinionConstraintGetTotalLambda(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPulleyConstraintSetLength(constraint: *Constraint, min: f32, max: f32) Result;
pub extern fn zjoltPulleyConstraintGetLength(constraint: *const Constraint, out_min: ?*f32, out_max: ?*f32) Result;
pub extern fn zjoltPulleyConstraintGetCurrentLength(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltPulleyConstraintGetTotalLambdaPosition(constraint: *const Constraint, out: *f32) Result;

pub extern fn zjoltPathConstraintSetPath(constraint: *Constraint, path: ?*const PathConstraintPath, fraction: f32) Result;
pub extern fn zjoltPathConstraintGetPath(constraint: *const Constraint) ?*const PathConstraintPath;
pub extern fn zjoltPathConstraintGetPathFraction(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltPathConstraintSetMotorSettings(constraint: *Constraint, motor: *const MotorSettings) Result;
pub extern fn zjoltPathConstraintGetMotorSettings(constraint: *const Constraint, out: *MotorSettings) Result;
pub extern fn zjoltPathConstraintSetMotorState(constraint: *Constraint, state: MotorState) Result;
pub extern fn zjoltPathConstraintGetMotorState(constraint: *const Constraint, out: *MotorState) Result;
pub extern fn zjoltPathConstraintSetTargetVelocity(constraint: *Constraint, velocity: f32) Result;
pub extern fn zjoltPathConstraintGetTargetVelocity(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltPathConstraintSetTargetPathFraction(constraint: *Constraint, fraction: f32) Result;
pub extern fn zjoltPathConstraintGetTargetPathFraction(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltPathConstraintSetMaxFrictionForce(constraint: *Constraint, force: f32) Result;
pub extern fn zjoltPathConstraintGetMaxFrictionForce(constraint: *const Constraint, out: *f32) Result;
pub extern fn zjoltPathConstraintGetTotalLambdaMotor(constraint: *const Constraint, out: *f32) Result;
