//! Behavioural tests for `tree.zig`: triangle splitting and AABB tree
//! building/packing over a unit cube, and a standalone island builder over a
//! small known body graph.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const tree = @import("tree.zig");
const system_mod = @import("system.zig");
const math = @import("math.zig");

//=============================================================================
// A unit cube: 8 vertices, 12 triangles (2 per face, 6 faces)
//=============================================================================

const cube_vertices = [_]tree.Vec3{
    math.vec3(0, 0, 0), // 0
    math.vec3(1, 0, 0), // 1
    math.vec3(1, 1, 0), // 2
    math.vec3(0, 1, 0), // 3
    math.vec3(0, 0, 1), // 4
    math.vec3(1, 0, 1), // 5
    math.vec3(1, 1, 1), // 6
    math.vec3(0, 1, 1), // 7
};

const cube_indices = [_]u32{
    0, 1, 2, 0, 2, 3, // bottom (z=0)
    4, 6, 5, 4, 7, 6, // top (z=1)
    0, 5, 1, 0, 4, 5, // front (y=0)
    3, 2, 6, 3, 6, 7, // back (y=1)
    0, 3, 7, 0, 7, 4, // left (x=0)
    1, 5, 6, 1, 6, 2, // right (x=1)
};

const cube_triangle_count: u32 = cube_indices.len / 3;

fn cubeTriangles() tree.Triangles {
    return .{ .vertices = &cube_vertices, .indices = &cube_indices };
}

//=============================================================================
// Triangle splitting and tree building
//=============================================================================

/// Small enough relative to `cube_triangle_count` (12) to force an actual
/// multi-level tree, so the node/leaf/depth relationships below are
/// exercising real structure rather than a single trivial leaf.
const max_triangles_per_leaf: u32 = 2;

fn checkTreeSelfConsistent(splitter: tree.TriangleSplitter) !void {
    var builder = try tree.AABBTreeBuilder.init(splitter, max_triangles_per_leaf);
    defer builder.deinit();

    const stats = try builder.build();

    // Every triangle the splitter started with appears exactly once across
    // the tree's leaves: AABBTreeBuilder partitions the sorted index range
    // it is given, so the leaves' combined count is the whole soup, exactly.
    try std.testing.expectEqual(cube_triangle_count, builder.triangleCountInTree());

    // AABBTreeBuilder never emits a node with exactly one child: every
    // internal node splits its range into two. So the tree is always full,
    // and node_count and leaf_node_count satisfy this exactly.
    try std.testing.expectEqual(2 * stats.leaf_node_count - 1, stats.node_count);
    try std.testing.expect(stats.leaf_node_count >= 1);
    try std.testing.expect(stats.min_depth >= 1);
    try std.testing.expect(stats.max_depth >= stats.min_depth);
    try std.testing.expectEqual(builder.hasChildren(), stats.node_count > 1);

    // CalculateSAHCost, both as Build reports it (cost_traversal =
    // cost_leaf = 1) and as the standalone call with the same weights.
    try std.testing.expect(std.math.isFinite(stats.sah_cost) and stats.sah_cost > 0);
    const sah = builder.calculateSAHCost(1.0, 1.0);
    try std.testing.expect(std.math.isFinite(sah) and sah > 0);
}

test "a binning-split tree over the cube is self-consistent and covers every triangle" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var splitter = try tree.TriangleSplitter.initBinning(cubeTriangles(), .{});
    defer splitter.deinit();

    try checkTreeSelfConsistent(splitter);
}

test "a mean-split tree over the cube is self-consistent and covers every triangle" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var splitter = try tree.TriangleSplitter.initMean(cubeTriangles());
    defer splitter.deinit();

    try checkTreeSelfConsistent(splitter);
}

test "a splitter's initial range and split cover the whole soup" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var splitter = try tree.TriangleSplitter.initMean(cubeTriangles());
    defer splitter.deinit();

    const initial = splitter.initialRange();
    try std.testing.expectEqual(@as(u32, 0), initial.begin);
    try std.testing.expectEqual(cube_triangle_count, initial.end);

    const halves = splitter.split(initial) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(initial.begin, halves.left.begin);
    try std.testing.expectEqual(initial.end, halves.right.end);
    try std.testing.expectEqual(halves.left.end, halves.right.begin);
    try std.testing.expect(halves.left.end > halves.left.begin);
    try std.testing.expect(halves.right.end > halves.right.begin);

    // Out of range: refused rather than read out of bounds.
    try std.testing.expect(splitter.split(.{ .begin = 0, .end = cube_triangle_count + 1 }) == null);
}

test "converting a built tree to a buffer yields readable node and triangle headers" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var splitter = try tree.TriangleSplitter.initMean(cubeTriangles());
    defer splitter.deinit();
    var builder = try tree.AABBTreeBuilder.init(splitter, max_triangles_per_leaf);
    defer builder.deinit();
    _ = try builder.build();

    var buffer = try tree.AABBTreeBuffer.init(builder, false);
    defer buffer.deinit();

    try std.testing.expect(buffer.data().len > 0);

    const node_header = try buffer.nodeHeader();
    try std.testing.expect(std.math.isFinite(node_header.root_bounds_min.x));
    try std.testing.expect(std.math.isFinite(node_header.root_bounds_max.x));

    const triangle_header = try buffer.triangleHeader();
    try std.testing.expect(std.math.isFinite(triangle_header.offset.x));
    try std.testing.expect(std.math.isFinite(triangle_header.scale.x));
}

//=============================================================================
// Islands
//=============================================================================

/// A page-backed `TempAllocator`, the same shape `system_test.zig` uses for
/// the physics system's own step scratch allocator — `IslandBuilder`'s
/// temp-allocator seam is the identical `ZJoltTempAllocator` table.
const PageTempAllocator = struct {
    pub fn allocate(self: *PageTempAllocator, size: u32) ?*anyopaque {
        _ = self;
        const slice = std.heap.page_allocator.alloc(u8, size) catch return null;
        return slice.ptr;
    }
    pub fn free(self: *PageTempAllocator, address: ?*anyopaque, size: u32) void {
        _ = self;
        const ptr = address orelse return;
        const bytes: [*]u8 = @ptrCast(ptr);
        std.heap.page_allocator.free(bytes[0..size]);
    }
};

test "an island builder over three bodies linked 1-2 and 2-3 reports one island; unlinked, three" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var page: PageTempAllocator = .{};
    const temp_allocator = system_mod.hostTempAllocator(PageTempAllocator, &page);

    var islands = try tree.IslandBuilder.init();
    defer islands.deinit();
    try islands.reserve(3);

    const bodies = [_]tree.BodyId{ 100, 200, 300 };

    try islands.linkBodies(0, 1);
    try islands.linkBodies(1, 2);
    try islands.finalize(&bodies, 0, &temp_allocator);

    try std.testing.expectEqual(@as(u32, 1), islands.numIslands());

    var members: [3]tree.BodyId = undefined;
    const count = try islands.bodiesInIsland(0, &members);
    try std.testing.expectEqual(@as(u32, 3), count);
    for (bodies) |id| {
        try std.testing.expect(std.mem.indexOfScalar(tree.BodyId, &members, id) != null);
    }

    // A capacity one short of the true count: BufferTooSmall, not a
    // silently truncated copy.
    var short: [2]tree.BodyId = undefined;
    try std.testing.expectError(error.BufferTooSmall, islands.bodiesInIsland(0, &short));

    // No constraints/contacts were ever prepared for this island.
    try std.testing.expectEqual(@as(u32, 0), try islands.constraintsInIsland(0, null));
    try std.testing.expectEqual(@as(u32, 0), try islands.contactsInIsland(0, null));

    // Unlinked (Finalize itself resets the link state it consumed): the
    // same three bodies, with no new linkBodies calls, fall into three
    // separate islands.
    try islands.resetIslands(&temp_allocator);
    try islands.finalize(&bodies, 0, &temp_allocator);

    try std.testing.expectEqual(@as(u32, 3), islands.numIslands());
    for (0..3) |i| {
        const one = try islands.bodiesInIsland(@intCast(i), &members);
        try std.testing.expectEqual(@as(u32, 1), one);
    }
}

test "an island builder refuses calls before reserve, and a stale index" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var page: PageTempAllocator = .{};
    const temp_allocator = system_mod.hostTempAllocator(PageTempAllocator, &page);

    var islands = try tree.IslandBuilder.init();
    defer islands.deinit();

    try std.testing.expectError(error.InvalidArgument, islands.linkBodies(0, 1));
    try std.testing.expectError(
        error.InvalidArgument,
        islands.prepareContactConstraints(4, &temp_allocator),
    );

    try islands.reserve(2);
    try islands.finalize(&.{ 1, 2 }, 0, &temp_allocator);
    try std.testing.expectError(error.InvalidArgument, islands.numPositionSteps(5));
}

//=============================================================================
// Large island splitting
//=============================================================================

test "assignSplit gives two never-before-linked bodies split 0" {
    var masks = [_]u32{ 0, 0 };
    const splitter = tree.SplitMasks.init(&masks);

    const split = splitter.assignSplit(0, true, 1, true);
    try std.testing.expectEqual(@as(u32, 0), split);
    try std.testing.expectEqual(@as(u32, 1), masks[0]);
    try std.testing.expectEqual(@as(u32, 1), masks[1]);
}

test "assignSplit picks the lowest split neither body already occupies" {
    var masks = [_]u32{ 0b011, 0b001 };
    const splitter = tree.SplitMasks.init(&masks);

    // Body 0 is in splits 0 and 1; body 1 is only in split 0. The first
    // split free for both is 2.
    const split = splitter.assignSplit(0, true, 1, true);
    try std.testing.expectEqual(@as(u32, 2), split);
}

test "assignSplit only touches the dynamic body when the other is not" {
    var masks = [_]u32{ 0, 0 };
    const splitter = tree.SplitMasks.init(&masks);

    const split = splitter.assignSplit(0, true, 1, false);
    try std.testing.expectEqual(@as(u32, 0), split);
    try std.testing.expectEqual(@as(u32, 1), masks[0]);
    try std.testing.expectEqual(@as(u32, 0), masks[1]);
}

test "assignSplit treats an out-of-range index as not present" {
    var masks = [_]u32{0};
    const splitter = tree.SplitMasks.init(&masks);

    // Body 1 is past the end of this splitter's array (a static/kinematic
    // body a caller never gave an active-body-list slot) — only body 0 is
    // touched.
    const split = splitter.assignSplit(0, true, 1, true);
    try std.testing.expectEqual(@as(u32, 0), split);
    try std.testing.expectEqual(@as(u32, 1), masks[0]);
}

test "assignSplit saturates at the non-parallel split once all 31 are taken" {
    var masks = [_]u32{ 0x7fffffff, 0 }; // body 0 already in every split but the last
    const splitter = tree.SplitMasks.init(&masks);

    const split = splitter.assignSplit(0, true, 1, true);
    try std.testing.expectEqual(tree.non_parallel_split_index, split);
}

test "assignToNonParallelSplit pins a dynamic body to the reserved split" {
    var masks = [_]u32{0};
    const splitter = tree.SplitMasks.init(&masks);

    const split = splitter.assignToNonParallelSplit(0, true);
    try std.testing.expectEqual(tree.non_parallel_split_index, split);
    try std.testing.expectEqual(@as(u32, 1) << 31, masks[0]);
}

test "assignToNonParallelSplit leaves a non-dynamic body's mask untouched" {
    var masks = [_]u32{0};
    const splitter = tree.SplitMasks.init(&masks);

    _ = splitter.assignToNonParallelSplit(0, false);
    try std.testing.expectEqual(@as(u32, 0), masks[0]);
}

test "IslandBuilder stats are UNSUPPORTED without -Dtrack_simulation_stats, or read back per island when it is on" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var page: PageTempAllocator = .{};
    const temp_allocator = system_mod.hostTempAllocator(PageTempAllocator, &page);

    var islands = try tree.IslandBuilder.init();
    defer islands.deinit();
    try islands.reserve(2);

    const bodies = [_]tree.BodyId{ 10, 20 };
    try islands.linkBodies(0, 1);
    try islands.finalize(&bodies, 0, &temp_allocator);
    try std.testing.expectEqual(@as(u32, 1), islands.numIslands());

    const stats = islands.stats(0) catch |e| {
        // Not built with -Dtrack_simulation_stats: IslandBuilder::IslandStats
        // does not exist to read, and the entry point says so rather than
        // fabricating a value.
        try std.testing.expectEqual(zjolt.Error.Unsupported, e);

        // The index is still validated ahead of the flag, so a caller cannot
        // tell a bad index apart from a missing feature by accident.
        try std.testing.expectError(error.InvalidArgument, islands.stats(1));
        return;
    };

    // Nothing has solved this island, so every counter is still at its
    // freshly-constructed value.
    try std.testing.expectEqual(@as(u8, 0), stats.num_velocity_steps);
    try std.testing.expectEqual(@as(u8, 0), stats.num_position_steps);
    try std.testing.expect(!stats.is_large_island);

    try std.testing.expectError(error.InvalidArgument, islands.stats(1));
}
