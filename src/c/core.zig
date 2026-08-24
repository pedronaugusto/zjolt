//! ZJolt C declarations for the primitives every other module here is built from: the scalar widths this build chose, results, vectors and matrices, the opaque handles, and library setup.
//!
//! Mirrors `ffi/zjolt_core.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const options = @import("zjolt_options");
const shape = @import("shape.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const ShapeStats = shape.ShapeStats;

pub const Real = if (options.double_precision) f64 else f32;

pub const ObjectLayer = if (options.object_layer_bits == 32) u32 else u16;

pub const BroadPhaseLayer = u8;

pub const BodyId = u32;

pub const SubShapeId = u32;

pub const body_id_invalid: BodyId = 0xffff_ffff;

pub const sub_shape_id_empty: SubShapeId = 0xffff_ffff;

pub const version_major: u32 = 0;

pub const version_minor: u32 = 1;

pub const version_patch: u32 = 0;

/// The Zig side's view of ZJOLT_CONFIG_ID. Must be computed the same way the
/// header computes it.
pub const config_id: u32 = (version_major << 16) | (version_minor << 8) |
    version_patch |
    (@as(u32, if (options.double_precision) 1 else 0) << 24) |
    (@as(u32, if (options.object_layer_bits == 32) 1 else 0) << 25);

pub const max_physics_jobs: u32 = 2048;

pub const max_physics_barriers: u32 = 8;

pub const build_flag_double_precision: u32 = 1 << 0;

pub const build_flag_object_layer_32: u32 = 1 << 1;

pub const build_flag_asserts_enabled: u32 = 1 << 2;

pub const build_flag_cross_platform_deterministic: u32 = 1 << 3;

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

pub const Color = extern struct {
    r: u8,
    g: u8,
    b: u8,
    a: u8,
};

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

pub const Shape = opaque {};

pub const PhysicsMaterial = opaque {};

pub const PhysicsSystem = opaque {};

pub const JobSystem = opaque {};

pub const Body = opaque {};

pub const Character = opaque {};

pub const GroupFilter = opaque {};

pub const StepListener = opaque {};

pub const BodyAddBatch = opaque {};

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

pub const AbiLayout = extern struct {
    layout_size: u32,
    config_id: u32,
    build_flags: u32,
    real_size: u32,
    object_layer_size: u32,
    default_allocate_alignment: u32,
};

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
