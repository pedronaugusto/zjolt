const std = @import("std");

/// Builds zjolt the way a real consumer does, which is not the way its own
/// test suite does.
///
/// The two are genuinely different code paths. An in-repo test reaches the
/// module and the library through the same `std.Build` graph that created
/// them; a consumer reaches them through `b.dependency`, which resolves
/// artifacts by scanning the dependency's *install step* and installed headers
/// by their spelling. Neither of those is exercised by anything in `src/` or
/// `tests/`, so both can break while the whole suite stays green. Registering
/// an artifact behind `if (b.pkg_hash.len == 0)` is exactly that failure: it
/// looks correct, it builds, and `dep.artifact("zjolt")` panics for everyone
/// downstream.
pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // Forwarded so this build can be run twice, once either way. The two
    // options that change zjolt's ABI are exactly the two a C host used to
    // have to repeat by hand; running with one of them set is what proves
    // the installed zjolt_config.h carries them instead.
    const double_precision = b.option(
        bool,
        "double_precision",
        "Build zjolt with JPH_DOUBLE_PRECISION",
    ) orelse false;
    const object_layer_bits = b.option(
        u8,
        "object_layer_bits",
        "Width of an object layer, 16 or 32",
    ) orelse 16;

    const zjolt = b.dependency("zjolt", .{
        .target = target,
        .optimize = optimize,
        .double_precision = double_precision,
        .object_layer_bits = object_layer_bits,
    });

    // 1. The Zig module, driven the way the README's example drives it.
    const zig_consumer = b.addExecutable(.{
        .name = "zig-consumer",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zjolt", .module = zjolt.module("zjolt") },
            },
        }),
    });

    // 2. The C library as an artifact, with its headers, which is what the
    //    README tells a C or C++ host it can do. `#include <zjolt.h>` here
    //    resolves only if build.zig installed the umbrella *and* the six
    //    headers it includes — a partial install compiles in-repo, where the
    //    whole `ffi/` directory is on the include path, and fails here.
    const c_consumer = b.addExecutable(.{
        .name = "c-consumer",
        .root_module = b.createModule(.{ .target = target, .optimize = optimize }),
    });
    c_consumer.root_module.link_libc = true;
    c_consumer.root_module.addCSourceFile(.{
        .file = b.path("src/main.c"),
        .flags = &.{"-std=c11"},
    });
    c_consumer.root_module.linkLibrary(zjolt.artifact("zjolt"));

    const step = b.step("run", "Build and run both consumers");
    step.dependOn(&b.addRunArtifact(zig_consumer).step);
    step.dependOn(&b.addRunArtifact(c_consumer).step);
    b.getInstallStep().dependOn(step);
}
