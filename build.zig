const std = @import("std");

/// The translation units of the vendored Jolt library that zjolt compiles.
///
/// Taken from upstream's own `Jolt/Jolt.cmake` unconditional source list and
/// written out explicitly rather than globbed, for two reasons: a glob would
/// silently start compiling whatever a future re-vendor drops in, and several
/// of Jolt's directories need SDKs that have no place in a physics package.
///
/// Not compiled, deliberately (see UPSTREAM.md):
///
///   * `Compute/{DX12,VK,MTL,CPU}` — GPU compute backends, which need the
///     Direct3D 12, Vulkan or Metal SDKs.
///   * `Shaders/*.cpp` — the CPU-compute shader wrappers, live only under
///     `JPH_USE_CPU_COMPUTE`.
///   * `ObjectStream/*.cpp` — optional upstream (`ENABLE_OBJECT_STREAM`), and
///     unused here: shape serialisation goes through StreamIn/StreamOut.
///
/// Vehicles, ragdolls and soft bodies ARE compiled. They are part of the
/// library and `RegisterTypes.cpp` refers to them; they are out of scope for
/// the C ABI, not for the build.
const jolt_sources = [_][]const u8{
    "libs/JoltPhysics/Jolt/AABBTree/AABBTreeBuilder.cpp",
    "libs/JoltPhysics/Jolt/Compute/ComputeSystem.cpp",
    "libs/JoltPhysics/Jolt/Core/Color.cpp",
    "libs/JoltPhysics/Jolt/Core/Factory.cpp",
    "libs/JoltPhysics/Jolt/Core/IssueReporting.cpp",
    "libs/JoltPhysics/Jolt/Core/JobSystemSingleThreaded.cpp",
    "libs/JoltPhysics/Jolt/Core/JobSystemThreadPool.cpp",
    "libs/JoltPhysics/Jolt/Core/JobSystemWithBarrier.cpp",
    "libs/JoltPhysics/Jolt/Core/LinearCurve.cpp",
    "libs/JoltPhysics/Jolt/Core/Memory.cpp",
    "libs/JoltPhysics/Jolt/Core/Profiler.cpp",
    "libs/JoltPhysics/Jolt/Core/RTTI.cpp",
    "libs/JoltPhysics/Jolt/Core/Semaphore.cpp",
    "libs/JoltPhysics/Jolt/Core/StringTools.cpp",
    "libs/JoltPhysics/Jolt/Core/TickCounter.cpp",
    "libs/JoltPhysics/Jolt/Geometry/ConvexHullBuilder.cpp",
    "libs/JoltPhysics/Jolt/Geometry/ConvexHullBuilder2D.cpp",
    "libs/JoltPhysics/Jolt/Geometry/Indexify.cpp",
    "libs/JoltPhysics/Jolt/Geometry/OrientedBox.cpp",
    "libs/JoltPhysics/Jolt/Math/Vec3.cpp",
    "libs/JoltPhysics/Jolt/ObjectStream/SerializableObject.cpp",
    "libs/JoltPhysics/Jolt/Physics/Body/Body.cpp",
    "libs/JoltPhysics/Jolt/Physics/Body/BodyCreationSettings.cpp",
    "libs/JoltPhysics/Jolt/Physics/Body/BodyInterface.cpp",
    "libs/JoltPhysics/Jolt/Physics/Body/BodyManager.cpp",
    "libs/JoltPhysics/Jolt/Physics/Body/MassProperties.cpp",
    "libs/JoltPhysics/Jolt/Physics/Body/MotionProperties.cpp",
    "libs/JoltPhysics/Jolt/Physics/Character/Character.cpp",
    "libs/JoltPhysics/Jolt/Physics/Character/CharacterBase.cpp",
    "libs/JoltPhysics/Jolt/Physics/Character/CharacterVirtual.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/BroadPhase/BroadPhase.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/BroadPhase/BroadPhaseBruteForce.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/BroadPhase/BroadPhaseQuadTree.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/BroadPhase/QuadTree.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/CastConvexVsTriangles.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/CastSphereVsTriangles.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/CollideConvexVsTriangles.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/CollideSphereVsTriangles.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/CollisionDispatch.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/CollisionGroup.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/EstimateCollisionResponse.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/GroupFilter.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/GroupFilterTable.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/ManifoldBetweenTwoFaces.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/NarrowPhaseQuery.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/NarrowPhaseStats.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/PhysicsMaterial.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/PhysicsMaterialSimple.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/BoxShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/CapsuleShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/CompoundShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/ConvexHullShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/ConvexShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/CylinderShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/DecoratedShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/EmptyShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/HeightFieldShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/MeshShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/MutableCompoundShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/PlaneShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/RotatedTranslatedShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/ScaledShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/Shape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/SphereShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/StaticCompoundShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/TaperedCapsuleShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/TaperedCylinderShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/Shape/TriangleShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Collision/TransformedShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/ConeConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/Constraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/ConstraintManager.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/ContactConstraintManager.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/DistanceConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/FixedConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/GearConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/HingeConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/MotorSettings.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/PathConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/PathConstraintPath.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/PathConstraintPathHermite.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/PointConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/PulleyConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/RackAndPinionConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/SixDOFConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/SliderConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/SpringSettings.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/SwingTwistConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Constraints/TwoBodyConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/DeterminismLog.cpp",
    "libs/JoltPhysics/Jolt/Physics/Hair/Hair.cpp",
    "libs/JoltPhysics/Jolt/Physics/Hair/HairSettings.cpp",
    "libs/JoltPhysics/Jolt/Physics/Hair/HairShaders.cpp",
    "libs/JoltPhysics/Jolt/Physics/Hair/RegisterHair.cpp",
    "libs/JoltPhysics/Jolt/Physics/IslandBuilder.cpp",
    "libs/JoltPhysics/Jolt/Physics/LargeIslandSplitter.cpp",
    "libs/JoltPhysics/Jolt/Physics/PhysicsScene.cpp",
    "libs/JoltPhysics/Jolt/Physics/PhysicsSystem.cpp",
    "libs/JoltPhysics/Jolt/Physics/PhysicsUpdateContext.cpp",
    "libs/JoltPhysics/Jolt/Physics/Ragdoll/Ragdoll.cpp",
    "libs/JoltPhysics/Jolt/Physics/SoftBody/SoftBodyCreationSettings.cpp",
    "libs/JoltPhysics/Jolt/Physics/SoftBody/SoftBodyMotionProperties.cpp",
    "libs/JoltPhysics/Jolt/Physics/SoftBody/SoftBodyShape.cpp",
    "libs/JoltPhysics/Jolt/Physics/SoftBody/SoftBodySharedSettings.cpp",
    "libs/JoltPhysics/Jolt/Physics/StateRecorderImpl.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/MotorcycleController.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/TrackedVehicleController.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/VehicleAntiRollBar.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/VehicleCollisionTester.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/VehicleConstraint.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/VehicleController.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/VehicleDifferential.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/VehicleEngine.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/VehicleTrack.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/VehicleTransmission.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/Wheel.cpp",
    "libs/JoltPhysics/Jolt/Physics/Vehicle/WheeledVehicleController.cpp",
    "libs/JoltPhysics/Jolt/RegisterTypes.cpp",
    "libs/JoltPhysics/Jolt/Renderer/DebugRenderer.cpp",
    "libs/JoltPhysics/Jolt/Renderer/DebugRendererPlayback.cpp",
    "libs/JoltPhysics/Jolt/Renderer/DebugRendererRecorder.cpp",
    "libs/JoltPhysics/Jolt/Renderer/DebugRendererSimple.cpp",
    "libs/JoltPhysics/Jolt/Skeleton/SkeletalAnimation.cpp",
    "libs/JoltPhysics/Jolt/Skeleton/Skeleton.cpp",
    "libs/JoltPhysics/Jolt/Skeleton/SkeletonMapper.cpp",
    "libs/JoltPhysics/Jolt/Skeleton/SkeletonPose.cpp",
    "libs/JoltPhysics/Jolt/TriangleSplitter/TriangleSplitter.cpp",
    "libs/JoltPhysics/Jolt/TriangleSplitter/TriangleSplitterBinning.cpp",
    "libs/JoltPhysics/Jolt/TriangleSplitter/TriangleSplitterMean.cpp",
};

/// The public headers, installed for consumers. `ffi/zjolt.h` is the umbrella
/// that includes the rest; `ffi/zjolt_internal.h` is deliberately absent,
/// being implementation-private.
const public_headers = [_][]const u8{
    "ffi/zjolt.h",
    "ffi/zjolt_core.h",
    "ffi/zjolt_shape.h",
    "ffi/zjolt_material.h",
    "ffi/zjolt_system.h",
    "ffi/zjolt_body.h",
    "ffi/zjolt_query.h",
    "ffi/zjolt_character.h",
};

/// The zjolt C boundary. One translation unit per concern — deliberately not a
/// single monolithic binding file.
const zjolt_ffi_sources = [_][]const u8{
    "ffi/zjolt_core.cpp",
    "ffi/zjolt_shape.cpp",
    "ffi/zjolt_material.cpp",
    "ffi/zjolt_system.cpp",
    "ffi/zjolt_body.cpp",
    "ffi/zjolt_query.cpp",
    "ffi/zjolt_character.cpp",
    "ffi/zjolt_abi.cpp",
};

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const options = .{
        .shared = b.option(
            bool,
            "shared",
            "Build the C library as a shared object",
        ) orelse false,
        .enable_asserts = b.option(
            bool,
            "enable_asserts",
            "Keep Jolt's internal asserts (defaults to on in Debug)",
        ) orelse (optimize == .Debug),
        // Off by default, and deliberately NOT tied to `optimize`.
        //
        // Zig's full C sanitizer emits calls into a runtime that is linked
        // only into a compilation that is itself sanitized. Defaulting it on
        // in Debug means a consumer who writes `b.dependency("zjolt", .{})` —
        // forgetting to forward `optimize`, the most common Zig packaging
        // mistake — gets a Debug zjolt inside a release executable and a link
        // failure reading `undefined symbol: __ubsan_handle_shift_out_of_bounds`,
        // which names nothing they can act on.
        //
        // zjolt's own suite turns it on explicitly instead: `ci/run.sh` and CI
        // both pass `-Dsanitize_c=true` for the Debug runs. A library should
        // not decide that its consumers are running a sanitizer.
        .sanitize_c = b.option(
            bool,
            "sanitize_c",
            "Compile the C and C++ with Zig's undefined-behaviour sanitizer",
        ) orelse false,
        .double_precision = b.option(
            bool,
            "double_precision",
            "Store world positions as doubles (JPH_DOUBLE_PRECISION), for large worlds",
        ) orelse false,
        .object_layer_bits = b.option(
            u8,
            "object_layer_bits",
            "Width of an object layer, 16 or 32 (JPH_OBJECT_LAYER_BITS)",
        ) orelse 16,
        .cross_platform_deterministic = b.option(
            bool,
            "cross_platform_deterministic",
            "Trade speed for bit-identical results across platforms (JPH_CROSS_PLATFORM_DETERMINISTIC)",
        ) orelse false,
    };

    if (options.object_layer_bits != 16 and options.object_layer_bits != 32) {
        std.debug.panic(
            "-Dobject_layer_bits must be 16 or 32, got {d}",
            .{options.object_layer_bits},
        );
    }

    // Every ABI- or behaviour-affecting option is mirrored into a Zig module
    // so the wrapper can never disagree with how the C++ was compiled. The
    // single `options` struct above is the one source both sides read from,
    // and `zjoltInitWithConfig` checks the two at run time as well.
    const options_step = b.addOptions();
    inline for (std.meta.fields(@TypeOf(options))) |field| {
        options_step.addOption(field.type, field.name, @field(options, field.name));
    }
    const options_module = options_step.createModule();

    const lib = b.addLibrary(.{
        .name = "zjolt",
        .linkage = if (options.shared) .dynamic else .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });

    lib.root_module.link_libc = true;
    if (target.result.abi != .msvc) lib.root_module.link_libcpp = true;

    // Jolt includes itself as <Jolt/...>, so its repository root is the include
    // path — not the Jolt/ directory itself.
    lib.root_module.addIncludePath(b.path("libs/JoltPhysics"));
    lib.root_module.addIncludePath(b.path("ffi"));

    applyBuildMacros(lib.root_module, options);
    if (options.shared and target.result.abi == .msvc) {
        lib.root_module.addCMacro("ZJOLT_SHARED", "");
        lib.root_module.addCMacro("ZJOLT_BUILD", "");
    }

    // Jolt throws nothing and brings its own RTTI, so both are disabled where
    // doing so is reliable. Under the MSVC ABI they are not: the Microsoft
    // standard library headers Jolt pulls in are written assuming exceptions
    // are available, and disabling them through Clang flags is a well-known
    // source of header errors. The saving is a little code size; the cost
    // would be a toolchain-specific build failure, so the MSVC ABI keeps the
    // defaults.
    //
    // Note what is NOT here on any target: no -fno-access-control (the FFI
    // layer uses only Jolt's public API, so it has no reason to defeat access
    // checking) and no blanket -fno-sanitize=undefined (UBSan stays on in
    // Debug, controlled by the `sanitize_c` option, so that real undefined
    // behaviour surfaces instead of being suppressed).
    const cxx_flags: []const []const u8 = if (target.result.abi == .msvc)
        &.{"-std=c++17"}
    else
        &.{ "-std=c++17", "-fno-exceptions", "-fno-rtti" };

    lib.root_module.addCSourceFiles(.{
        .files = &jolt_sources,
        .flags = cxx_flags,
    });
    lib.root_module.addCSourceFiles(.{
        .files = &zjolt_ffi_sources,
        .flags = cxx_flags,
    });
    lib.root_module.sanitize_c = if (options.sanitize_c) .full else .off;

    // Consumers get the public headers without reaching into the source tree.
    // zjolt.h is the umbrella; the rest are its parts, split by concern because
    // one header carrying the whole surface stopped being readable.
    for (public_headers) |header| {
        lib.installHeader(b.path(header), std.fs.path.basename(header));
    }

    //=====================================================================
    // The Zig module.
    //=====================================================================

    const module = b.addModule("zjolt", .{
        .root_source_file = b.path("src/zjolt.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zjolt_options", .module = options_module },
        },
    });
    // No include path: the wrapper hand-writes its externs rather than
    // @cImport-ing the header, so nothing Zig-side compiles C.
    module.linkLibrary(lib);

    // Registered unconditionally, including when zjolt is consumed as a
    // dependency. `std.Build.Dependency.artifact` finds an artifact by
    // scanning the dependency's install step, so anything NOT installed here
    // is invisible to a consumer — `dep.artifact("zjolt")` panics rather than
    // failing gracefully, and the installed headers go with it.
    //
    // This does not put zjolt's library in a consumer's prefix: a dependency's
    // install step only runs when something the consumer builds actually
    // depends on it. `tests/consumer` is what keeps this honest.
    b.installArtifact(lib);

    //=====================================================================
    // Tests.
    //=====================================================================

    // A C-only smoke test proves the boundary stands on its own, independent
    // of anything Zig-side — the header is a real C contract, not a private
    // detail of the wrapper, and the allocator seam is genuinely in use (it
    // asserts every allocation is released).
    const c_smoke = b.addExecutable(.{
        .name = "zjolt-c-smoke",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });
    c_smoke.root_module.link_libc = true;
    c_smoke.root_module.addIncludePath(b.path("ffi"));
    applyBuildMacros(c_smoke.root_module, options);
    c_smoke.root_module.addCSourceFile(.{
        .file = b.path("tests/c_smoke.c"),
        .flags = &.{"-std=c11"},
    });
    c_smoke.root_module.linkLibrary(lib);
    c_smoke.root_module.sanitize_c = if (options.sanitize_c) .full else .off;

    const c_test_step = b.step("test-c", "Run the C-level smoke test");
    c_test_step.dependOn(&b.addRunArtifact(c_smoke).step);

    // Linking the smoke test is part of the default build, without installing
    // or running it. That makes `zig build -Dtarget=...` a link check rather
    // than only a compile check: a cross target where libc++ or the platform's
    // threading primitives fail to resolve produces a static library
    // regardless, and would otherwise pass.
    b.getInstallStep().dependOn(&c_smoke.step);

    const tests = b.addTest(.{
        .name = "zjolt-tests",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/zjolt.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zjolt_options", .module = options_module },
            },
        }),
    });
    tests.root_module.linkLibrary(lib);

    // The ABI cross-check @cImport-s ffi/zjolt.h. It is wired here, on the test
    // module, and deliberately not on the module above: the shipped module has
    // no include path and never runs translate-c.
    //
    // The macros matter as much as the include path. ZJoltReal and
    // ZJoltObjectLayer change width with the build, so a header preprocessed
    // with different macros than the library describes different structs — and
    // a guard that compared against that header would be checking something
    // nobody ships. Same applyBuildMacros, same source of truth.
    tests.root_module.addIncludePath(b.path("ffi"));
    applyBuildMacros(tests.root_module, options);

    const test_step = b.step("test", "Run zjolt tests");
    test_step.dependOn(&b.addRunArtifact(tests).step);
    test_step.dependOn(c_test_step);
}

/// Applies every macro that changes what Jolt, or the zjolt header, compiles
/// to.
///
/// Two independent mismatches are prevented here. Jolt bakes its own
/// configuration into `JPH_VERSION_ID` and `RegisterTypes` calls `std::abort()`
/// when a client disagrees with the library. And `ffi/zjolt.h` changes the
/// width of `ZJoltReal` and `ZJoltObjectLayer`, so a consumer compiled against
/// different settings would misread every position it is handed.
///
/// Defining the set once, in one function applied to every module that sees
/// either header, is what makes both mismatches unreachable rather than merely
/// unlikely. The same struct is mirrored into the Zig `zjolt_options` module,
/// which is how the Zig side stays in step without a second copy.
fn applyBuildMacros(module: *std.Build.Module, options: anytype) void {
    if (!options.enable_asserts) {
        module.addCMacro("NDEBUG", "");
    } else {
        module.addCMacro("JPH_ENABLE_ASSERTS", "");
    }
    if (options.double_precision) {
        module.addCMacro("JPH_DOUBLE_PRECISION", "");
        module.addCMacro("ZJOLT_DOUBLE_PRECISION", "");
    }
    if (options.cross_platform_deterministic) {
        module.addCMacro("JPH_CROSS_PLATFORM_DETERMINISTIC", "");
    }
    if (options.object_layer_bits == 32) {
        module.addCMacro("JPH_OBJECT_LAYER_BITS", "32");
        module.addCMacro("ZJOLT_OBJECT_LAYER_BITS", "32");
    }
}
