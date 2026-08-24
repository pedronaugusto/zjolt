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

//=============================================================================
// Opaque handles
//=============================================================================

pub const Shape = opaque {};
pub const PhysicsMaterial = opaque {};
pub const PhysicsSystem = opaque {};
pub const JobSystem = opaque {};
pub const Body = opaque {};
pub const Character = opaque {};

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

pub const QueryFilters = extern struct {
    broad_phase_layer: BroadPhaseLayerFilter = .{},
    object_layer: ObjectLayerFilter = .{},
    body: BodyFilter = .{},
};

pub const RayCastHit = extern struct {
    body: BodyId,
    sub_shape_id: SubShapeId,
    fraction: f32,
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

pub extern fn zjoltCastRayClosest(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, filters: ?*const QueryFilters, out_hit: *RayCastHit, out_hit_any: *bool) Result;
pub extern fn zjoltCastRayAll(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, filters: ?*const QueryFilters, out_hits: ?[*]RayCastHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltCastShapeClosest(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, filters: ?*const QueryFilters, out_hit: *ShapeCastHit, out_hit_any: *bool) Result;
pub extern fn zjoltCastShapeAll(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, direction: *const Vec3, filters: ?*const QueryFilters, out_hits: ?[*]ShapeCastHit, capacity: u32, out_count: *u32) Result;
pub extern fn zjoltCollideShape(system: *const PhysicsSystem, shape: *const Shape, scale: ?*const Vec3, position: *const RVec3, rotation: *const Quat, max_separation_distance: f32, filters: ?*const QueryFilters, out_hits: ?[*]CollideShapeHit, capacity: u32, out_count: *u32) Result;

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
