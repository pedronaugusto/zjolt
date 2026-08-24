//! zjolt — Zig bindings for Jolt Physics.
//!
//! The package owns no policy: no fixed timestep, no entity system, no
//! renderer. A host drives it as `build shapes -> create bodies -> step ->
//! read back`.
//!
//! ```zig
//! try zjolt.init(.{ .allocator = gpa });
//! defer zjolt.deinit();
//!
//! const jobs = try zjolt.JobSystem.initThreadPool(.{});
//! defer jobs.deinit();
//!
//! const system = try zjolt.PhysicsSystem.init(.{
//!     .layers = zjolt.layersFromType(MyLayers),
//! });
//! defer system.deinit();
//!
//! const shape = try zjolt.Shape.initSphere(0.5, .{});
//! defer shape.release();
//!
//! const ball = try system.bodies().createAndAdd(.{
//!     .shape = shape,
//!     .object_layer = MyLayers.moving,
//!     .position = zjolt.rvec3(0, 10, 0),
//! }, .activate);
//!
//! // Per frame:
//! _ = try system.step(1.0 / 60.0, 1, jobs);
//! const transform = system.bodies().getTransform(ball);
//! ```

const std = @import("std");

pub const c = @import("c.zig");

const error_mod = @import("error.zig");
const math_mod = @import("math.zig");
const memory_mod = @import("memory.zig");
const material_mod = @import("material.zig");
const shape_mod = @import("shape.zig");
const body_mod = @import("body.zig");
const query_mod = @import("query.zig");
const system_mod = @import("system.zig");
const character_mod = @import("character.zig");

//=============================================================================
// Public surface
//=============================================================================

pub const Error = error_mod.Error;
pub const resultName = error_mod.name;
pub const lastError = error_mod.lastError;

pub const Vec3 = math_mod.Vec3;
pub const RVec3 = math_mod.RVec3;
pub const Quat = math_mod.Quat;
pub const Real = math_mod.Real;
pub const AABox = math_mod.AABox;
pub const MassProperties = math_mod.MassProperties;
pub const ShapeStats = math_mod.ShapeStats;
pub const vec3 = math_mod.vec3;
pub const rvec3 = math_mod.rvec3;
pub const quat = math_mod.quat;
pub const quatFromAxisAngle = math_mod.quatFromAxisAngle;
pub const toRVec3 = math_mod.toRVec3;
pub const vec3_zero = math_mod.vec3_zero;
pub const rvec3_zero = math_mod.rvec3_zero;
pub const quat_identity = math_mod.quat_identity;
pub const gravity_earth = math_mod.gravity_earth;

pub const Shape = shape_mod.Shape;
pub const ShapeSubType = shape_mod.SubType;
pub const MutableCompound = shape_mod.MutableCompound;
pub const CompoundChild = shape_mod.CompoundChild;
pub const compoundChild = shape_mod.compoundChild;
pub const SubShapeId = c.SubShapeId;
pub const sub_shape_id_empty = shape_mod.sub_shape_id_empty;
pub const height_field_no_collision = shape_mod.height_field_no_collision;

pub const PhysicsMaterial = material_mod.PhysicsMaterial;
pub const Color = material_mod.Color;
pub const color = material_mod.color;

pub const BodyId = body_mod.BodyId;
pub const invalid_body_id = body_mod.invalid_body_id;
pub const BodyDesc = body_mod.BodyDesc;
pub const BodyInterface = body_mod.BodyInterface;
pub const Body = body_mod.Body;
pub const BodyLock = body_mod.Lock;
pub const Transform = body_mod.Transform;
pub const MotionType = body_mod.MotionType;
pub const MotionQuality = body_mod.MotionQuality;
pub const Activation = body_mod.Activation;
pub const AllowedDofs = body_mod.AllowedDofs;
pub const OverrideMassProperties = body_mod.OverrideMassProperties;

pub const ObjectLayer = system_mod.ObjectLayer;
pub const BroadPhaseLayer = system_mod.BroadPhaseLayer;
pub const Layers = system_mod.Layers;
pub const layersFromType = system_mod.layersFromType;
pub const PhysicsSystem = system_mod.PhysicsSystem;
pub const JobSystem = system_mod.JobSystem;
pub const UpdateError = system_mod.UpdateError;
pub const contactListener = system_mod.contactListener;
pub const bodyActivationListener = system_mod.bodyActivationListener;
pub const ContactInfo = system_mod.ContactInfo;
pub const ContactManifold = system_mod.ContactManifold;
pub const ContactSettings = system_mod.ContactSettings;
pub const ContactValidateInfo = system_mod.ContactValidateInfo;
pub const SubShapeIdPair = system_mod.SubShapeIdPair;
pub const ValidateResult = system_mod.ValidateResult;
pub const contactPointsOn1 = system_mod.contactPointsOn1;
pub const contactPointsOn2 = system_mod.contactPointsOn2;

pub const Queries = query_mod.Queries;
pub const QueryFilters = query_mod.Filters;
pub const RayCastHit = query_mod.RayCastHit;
pub const ShapeCastHit = query_mod.ShapeCastHit;
pub const CollideShapeHit = query_mod.CollideShapeHit;
pub const ExcludeBody = query_mod.ExcludeBody;
pub const OnlyObjectLayer = query_mod.OnlyObjectLayer;
pub const CollidePointHit = query_mod.CollidePointHit;
pub const QueryShapeFilter = query_mod.ShapeFilter;
pub const SubShapeId = query_mod.SubShapeId;
pub const empty_sub_shape_id = query_mod.empty_sub_shape_id;
pub const HitAction = query_mod.HitAction;
pub const RayCastSettings = query_mod.RayCastSettings;

pub const Character = character_mod.Character;
pub const GroundState = character_mod.GroundState;
pub const BackFaceMode = character_mod.BackFaceMode;
pub const CharacterUpdateSettings = character_mod.UpdateSettings;
pub const defaultCharacterUpdateSettings = character_mod.defaultUpdateSettings;

/// Build options the C library was actually compiled with, so a consumer can
/// branch on them instead of assuming.
pub const options = @import("zjolt_options");

//=============================================================================
// Initialisation
//=============================================================================

/// A formatted line of Jolt diagnostic output.
pub const TraceFn = c.TraceFn;
pub const AssertFailedFn = c.AssertFailedFn;

pub const InitOptions = struct {
    /// Routes every Jolt allocation through this. Null keeps Jolt's own
    /// malloc/free.
    ///
    /// Process-wide, because Jolt's allocator is. It must outlive `deinit`.
    allocator: ?std.mem.Allocator = null,
    /// Receives Jolt's log output, already formatted. Null sends it to
    /// stderr — which is zjolt's fallback, not Jolt's: Jolt's own default
    /// trace function is a stub whose body is an assertion, so an unhooked
    /// Jolt aborts the first time it has something to report.
    trace: ?TraceFn = null,
    /// Called when a Jolt assertion fails; return true to break. Only
    /// reachable when `options.enable_asserts` is set, and every assertion
    /// reachable through this API has been turned into a returned error
    /// instead — so in practice this fires only on a zjolt bug.
    assert_failed: ?AssertFailedFn = null,
    /// Passed back to `trace` and `assert_failed`.
    hooks_user: ?*anyopaque = null,
};

/// Installs the allocator and hooks, creates Jolt's factory and registers its
/// types. Must be called before anything else, and is process-wide.
///
/// Returns `error.ConfigMismatch` if this wrapper and the C library were built
/// with different layout-affecting settings. That cannot happen through
/// `build.zig`, which derives both from one option set — it is the guard for
/// a hand-assembled link.
pub fn init(opts: InitOptions) Error!void {
    var bridged: c.Allocator = undefined;
    var desc: c.InitDesc = .{
        .allocator = null,
        .trace = opts.trace,
        .assert_failed = opts.assert_failed,
        .hooks_user = opts.hooks_user,
    };
    if (opts.allocator) |gpa| {
        bridged = memory_mod.bridge(gpa);
        desc.allocator = &bridged;
    }
    try error_mod.check(c.zjoltInitWithConfig(&desc, c.config_id));
}

/// Unregisters Jolt's types, destroys the factory, and gives Jolt its own
/// allocator back. Safe to call when not initialised.
///
/// Refuses, and does nothing but trace, while any handle is still alive — see
/// `liveHandleCount`. Restoring the allocator with handles outstanding would
/// make destroying them free through an allocator they were never allocated
/// from, so leaving the library up is the safer of the two wrong situations.
pub fn deinit() void {
    c.zjoltDeinit();
}

pub fn isInitialized() bool {
    return c.zjoltIsInitialized();
}

/// Physics systems, job systems and characters currently alive.
///
/// `deinit` refuses while this is non-zero, so a shutdown path can assert on
/// it rather than discover the problem as heap corruption later. Shapes are
/// not counted; Jolt reference counts those.
pub fn liveHandleCount() u32 {
    return c.zjoltLiveHandleCount();
}

//=============================================================================
// Versions
//=============================================================================

pub const Version = struct {
    major: u8,
    minor: u8,
    patch: u8,

    fn unpack(packed_value: u32) Version {
        return .{
            .major = @truncate(packed_value >> 16),
            .minor = @truncate(packed_value >> 8),
            .patch = @truncate(packed_value),
        };
    }

    pub fn format(self: Version, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{d}.{d}.{d}", .{ self.major, self.minor, self.patch });
    }
};

/// Version of these bindings.
pub fn version() Version {
    return Version.unpack(c.zjoltVersion());
}

/// Version of the vendored Jolt Physics library.
pub fn joltVersion() Version {
    return Version.unpack(c.zjoltJoltVersion());
}

//=============================================================================
// Tests
//=============================================================================

test {
    // Pull every module in so its own tests are discovered and run.
    _ = error_mod;
    _ = math_mod;
    _ = memory_mod;
    _ = material_mod;
    _ = shape_mod;
    _ = body_mod;
    _ = query_mod;
    _ = system_mod;
    _ = character_mod;
    _ = @import("integration_test.zig");

    // Mechanical, and it has to be: it calls every entry point in the ABI
    // with nulls, discovered by reflection rather than listed.
    _ = @import("misuse_sweep_test.zig");

    // Test-only: this one @cImport-s the C header. Reached from a test block
    // and nowhere else, so a normal build never analyses it and the shipped
    // module stays translate-c-free.
    _ = @import("abi_check.zig");
}

test "the library reports the build the wrapper was compiled for" {
    // What abi_check.zig cannot see. It compares this wrapper against the
    // header as THIS build's preprocessor rendered it — which says nothing
    // about whether the library linked here was compiled with the same macros.
    // ZJoltReal and ZJoltObjectLayer change width with those macros, so a
    // library built with different ones describes different structs while
    // presenting an identical header. This is the check that catches it.
    var layout: c.AbiLayout = undefined;
    c.zjoltAbiLayout(&layout);

    try std.testing.expectEqual(@as(u32, @sizeOf(c.AbiLayout)), layout.layout_size);
    try std.testing.expectEqual(@as(u32, @sizeOf(c.Real)), layout.real_size);
    try std.testing.expectEqual(@as(u32, @sizeOf(c.ObjectLayer)), layout.object_layer_size);
    try std.testing.expectEqual(c.config_id, layout.config_id);
    try std.testing.expectEqual(c.zjoltConfigId(), layout.config_id);
    try std.testing.expectEqual(
        @as(u32, @intCast(c.zjoltDefaultAllocateAlignment())),
        layout.default_allocate_alignment,
    );

    // The options module and the C library must describe the same build.
    const double_bit = (layout.build_flags & c.build_flag_double_precision) != 0;
    try std.testing.expectEqual(options.double_precision, double_bit);
    const layer32_bit = (layout.build_flags & c.build_flag_object_layer_32) != 0;
    try std.testing.expectEqual(options.object_layer_bits == 32, layer32_bit);
    const asserts_bit = (layout.build_flags & c.build_flag_asserts_enabled) != 0;
    try std.testing.expectEqual(options.enable_asserts, asserts_bit);
    const deterministic_bit =
        (layout.build_flags & c.build_flag_cross_platform_deterministic) != 0;
    try std.testing.expectEqual(options.cross_platform_deterministic, deterministic_bit);
}

test "version reporting is wired up" {
    // Against the mirrored constants rather than literals: abi_check.zig
    // already ties those to the header's ZJOLT_VERSION_* macros, so this
    // completes the chain library -> Zig mirror -> header without a third
    // copy of the number that would need editing when a version is cut.
    const v = version();
    try std.testing.expectEqual(@as(u8, @intCast(c.version_major)), v.major);
    try std.testing.expectEqual(@as(u8, @intCast(c.version_minor)), v.minor);
    try std.testing.expectEqual(@as(u8, @intCast(c.version_patch)), v.patch);

    const jolt = joltVersion();
    try std.testing.expectEqual(@as(u8, 5), jolt.major);
    try std.testing.expectEqual(@as(u8, 6), jolt.minor);
}

test "result names are never empty" {
    inline for (@typeInfo(c.Result).@"enum".fields) |field| {
        const name = resultName(@enumFromInt(field.value));
        try std.testing.expect(name.len > 0);
    }
}
