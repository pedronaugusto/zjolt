//! Soft body shared settings: the index remaps `optimize` produces, and the
//! two binary forms the settings themselves save into.
//!
//! Everything else about soft bodies is exercised where it is used — in
//! `system_test.zig` for the active lists, `debug_test.zig` for drawing.
//! These two are about the settings object as an ASSET: what survives being
//! reordered, and what survives being written and read back.

const std = @import("std");
const zjolt = @import("zjolt.zig");

/// A grid mesh with enough shared vertices that `optimize` has real work to
/// do: `size` by `size` vertices, an edge along every row and column, and a
/// tetrahedral volume constraint per interior cell. Small meshes are the
/// trap here — a handful of edges can come out of `optimize` in the order
/// they went in, and a remap that happened to be the identity would prove
/// nothing.
fn buildGrid(size: u32) !zjolt.SoftBodySharedSettings {
    const settings = try zjolt.SoftBodySharedSettings.create();
    errdefer settings.release();

    var vertices: [64]zjolt.SoftBodyVertex = undefined;
    var vertex_count: usize = 0;
    for (0..size) |y| {
        for (0..size) |x| {
            vertices[vertex_count] = .{
                .position = .{
                    .x = @floatFromInt(x),
                    .y = 0,
                    .z = @floatFromInt(y),
                },
                .velocity = zjolt.vec3_zero,
                .inv_mass = 1,
            };
            vertex_count += 1;
        }
    }
    try settings.addVertices(vertices[0..vertex_count]);

    var faces: [128]zjolt.SoftBodyFace = undefined;
    var edges: [256]zjolt.SoftBodyEdge = undefined;
    var volumes: [128]zjolt.SoftBodyVolumeConstraint = undefined;
    var face_count: usize = 0;
    var edge_count: usize = 0;
    var volume_count: usize = 0;
    for (0..size - 1) |y| {
        for (0..size - 1) |x| {
            const a: u32 = @intCast(y * size + x);
            const b: u32 = a + 1;
            const c: u32 = @intCast((y + 1) * size + x);
            const d: u32 = c + 1;
            faces[face_count] = .{ .vertex = .{ a, b, c }, .material_index = 0 };
            face_count += 1;
            faces[face_count] = .{ .vertex = .{ b, d, c }, .material_index = 0 };
            face_count += 1;
            edges[edge_count] = .{ .vertex = .{ a, b }, .compliance = 0 };
            edge_count += 1;
            edges[edge_count] = .{ .vertex = .{ a, c }, .compliance = 0 };
            edge_count += 1;
            volumes[volume_count] = .{ .vertex = .{ a, b, c, d }, .compliance = 0 };
            volume_count += 1;
        }
    }
    try settings.addFaces(faces[0..face_count]);
    // Reversed on the way in. Jolt's `Optimize` sorts each parallel group by
    // its lowest vertex index, so a list already in that order comes back
    // unchanged and a remap that is the identity would prove nothing.
    std.mem.reverse(zjolt.SoftBodyEdge, edges[0..edge_count]);
    try settings.addEdges(edges[0..edge_count]);
    try settings.addVolumeConstraints(volumes[0..volume_count]);
    settings.calculateEdgeLengths();
    settings.calculateVolumeConstraintVolumes();
    return settings;
}

const sentinel: u32 = 0xFFFF_FFFF;

/// Every index 0..len appears exactly once — the only thing a remap can be.
/// Also catches a buffer that was never written: the sentinel is out of
/// range, so it fails the bounds check before the duplicate check.
fn expectPermutation(remap: []const u32) !void {
    var seen: [256]bool = @splat(false);
    for (remap) |index| {
        try std.testing.expect(index < remap.len);
        try std.testing.expect(!seen[index]);
        seen[index] = true;
    }
}

fn isIdentity(remap: []const u32) bool {
    for (remap, 0..) |index, i| {
        if (index != i) return false;
    }
    return true;
}

test "optimize reports where every constraint went, for each of Jolt's seven lists" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const settings = try buildGrid(6);
    defer settings.release();

    const counts = try settings.remapCounts();
    try std.testing.expectEqual(@as(u32, 50), counts.edges);
    try std.testing.expectEqual(@as(u32, 25), counts.volume);
    // Nothing authored any of these, so their remaps are empty rather than
    // absent — the call still has to leave the buffers alone.
    try std.testing.expectEqual(@as(u32, 0), counts.lra);
    try std.testing.expectEqual(@as(u32, 0), counts.rod_stretch_shear);
    try std.testing.expectEqual(@as(u32, 0), counts.rod_bend_twist);
    try std.testing.expectEqual(@as(u32, 0), counts.dihedral_bend);
    try std.testing.expectEqual(@as(u32, 0), counts.skinned);

    var edge_remap: [64]u32 = @splat(sentinel);
    var volume_remap: [32]u32 = @splat(sentinel);
    try settings.optimizeWithRemap(.{
        .edges = edge_remap[0..counts.edges],
        .volume = volume_remap[0..counts.volume],
    });

    try expectPermutation(edge_remap[0..counts.edges]);
    try expectPermutation(volume_remap[0..counts.volume]);
    // Untouched past the length asked for: the copy is bounded by Jolt's own
    // array, not by the buffer handed in.
    try std.testing.expectEqual(sentinel, edge_remap[counts.edges]);

    // And the remap is not the identity, so it is reporting a reordering
    // that actually happened rather than describing a no-op.
    try std.testing.expect(!isIdentity(edge_remap[0..counts.edges]));
}

test "a short remap buffer is refused before anything is reordered" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const settings = try buildGrid(6);
    defer settings.release();

    const counts = try settings.remapCounts();
    var too_short: [8]u32 = @splat(sentinel);
    try std.testing.expectError(
        error.BufferTooSmall,
        settings.optimizeWithRemap(.{ .edges = &too_short }),
    );

    // Nothing was written into the short buffer, and — the point of checking
    // capacities first — the settings are still un-optimised, so the same
    // call with a long enough buffer still reports a full reordering.
    try std.testing.expectEqual(sentinel, too_short[0]);
    try std.testing.expectEqual(counts, try settings.remapCounts());

    var edge_remap: [64]u32 = @splat(sentinel);
    try settings.optimizeWithRemap(.{ .edges = edge_remap[0..counts.edges] });
    try expectPermutation(edge_remap[0..counts.edges]);
    try std.testing.expect(!isIdentity(edge_remap[0..counts.edges]));
}

test "shared settings round-trip through their own binary form" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const settings = try buildGrid(5);
    defer settings.release();
    settings.optimize();
    const before = try settings.remapCounts();

    var stream_buffer: [65536]u8 = undefined;
    var writer: zjolt.StreamBufferWriter = .{ .buffer = &stream_buffer };
    try settings.saveBinaryState(zjolt.hostStream(zjolt.StreamBufferWriter, &writer));

    var reader: zjolt.StreamBufferReader = .{ .buffer = writer.slice() };
    const restored = try zjolt.SoftBodySharedSettings.restoreBinaryState(
        zjolt.hostStream(zjolt.StreamBufferReader, &reader),
    );
    defer restored.release();
    try std.testing.expectEqual(before, try restored.remapCounts());

    // The two forms are not interchangeable, and the header says so rather
    // than the reader running off the end of a different layout.
    var wrong: zjolt.StreamBufferReader = .{ .buffer = writer.slice() };
    try std.testing.expectError(
        error.BadFormat,
        zjolt.SoftBodySharedSettings.restoreWithMaterials(
            zjolt.hostStream(zjolt.StreamBufferReader, &wrong),
        ),
    );
}

test "saveWithMaterials carries the material list that saveBinaryState leaves out" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const settings = try buildGrid(5);
    defer settings.release();

    const rubber = try zjolt.PhysicsMaterial.init(.{ .debug_name = "rubber" });
    defer rubber.release();
    const cloth = try zjolt.PhysicsMaterial.init(.{ .debug_name = "cloth" });
    defer cloth.release();
    try settings.setMaterials(std.testing.allocator, &.{ rubber, cloth });
    settings.optimize();

    var stream_buffer: [65536]u8 = undefined;
    var writer: zjolt.StreamBufferWriter = .{ .buffer = &stream_buffer };
    try settings.saveWithMaterials(zjolt.hostStream(zjolt.StreamBufferWriter, &writer));

    var reader: zjolt.StreamBufferReader = .{ .buffer = writer.slice() };
    const restored = try zjolt.SoftBodySharedSettings.restoreWithMaterials(
        zjolt.hostStream(zjolt.StreamBufferReader, &reader),
    );
    defer restored.release();

    const materials = try restored.getMaterials(std.testing.allocator);
    defer std.testing.allocator.free(materials);
    // Borrowed from the settings, not owned by this slice — releasing one
    // here would drop the settings' own reference.
    try std.testing.expectEqual(@as(usize, 2), materials.len);

    // The other arm: the same settings through the form that does NOT write
    // materials come back with the default list instead of these two.
    var plain_buffer: [65536]u8 = undefined;
    var plain_writer: zjolt.StreamBufferWriter = .{ .buffer = &plain_buffer };
    try settings.saveBinaryState(zjolt.hostStream(zjolt.StreamBufferWriter, &plain_writer));
    var plain_reader: zjolt.StreamBufferReader = .{ .buffer = plain_writer.slice() };
    const plain = try zjolt.SoftBodySharedSettings.restoreBinaryState(
        zjolt.hostStream(zjolt.StreamBufferReader, &plain_reader),
    );
    defer plain.release();
    try std.testing.expect((try plain.countMaterials()) != 2);
}
