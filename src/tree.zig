//! Triangle splitting, AABB tree building, and packing a tree into the
//! buffer `MeshShape` itself queries — plus a standalone island builder, the
//! grouping `PhysicsSystem` uses for the solver with no accessor of its own.
//!
//! Two independent capabilities sharing this file because they share one
//! test file (`tree_test.zig`); neither depends on the other.

const c = @import("c/tree.zig");
const system_c = @import("c/system.zig");
const err = @import("error.zig");
const math_mod = @import("math.zig");

pub const Vec3 = math_mod.Vec3;
pub const TempAllocator = system_c.TempAllocator;

//=============================================================================
// Triangle splitting
//=============================================================================

/// A half-open range of triangle indices, [begin, end), into a splitter's
/// own sorted order.
pub const TriangleRange = c.TriangleRange;

/// The triangle soup a splitter divides. `indices` holds 3*(triangle count)
/// vertex indices; `triangle_materials`/`triangle_user_data`, if given, are
/// one entry per triangle.
pub const Triangles = struct {
    vertices: []const Vec3,
    indices: []const u32,
    triangle_materials: ?[]const u32 = null,
    triangle_user_data: ?[]const u32 = null,
};

/// Divides a triangle soup for `AABBTreeBuilder`, the same way
/// `MeshBuildQuality` picks a strategy for a mesh shape. Borrow one into
/// exactly one builder (`AABBTreeBuilder.init`); the builder reads its
/// triangles during `build`, and `AABBTreeBuffer.init` reads its vertices
/// again afterwards, so it must outlive both.
pub const TriangleSplitter = struct {
    handle: *c.TriangleSplitter,

    /// Binning: slower to build, better-balanced trees — what
    /// `.favor_runtime_performance` uses. 0 for any bin option is Jolt's
    /// default (8, 128, 6); given non-zero, `min_num_bins` must not exceed
    /// `max_num_bins`.
    pub fn initBinning(triangles: Triangles, opts: struct {
        min_num_bins: u32 = 0,
        max_num_bins: u32 = 0,
        num_triangles_per_bin: u32 = 0,
    }) err.Error!TriangleSplitter {
        var handle: *c.TriangleSplitter = undefined;
        try err.check(c.zjoltTriangleSplitterCreateBinning(
            triangles.vertices.ptr,
            @intCast(triangles.vertices.len),
            triangles.indices.ptr,
            @intCast(triangles.indices.len / 3),
            if (triangles.triangle_materials) |m| m.ptr else null,
            if (triangles.triangle_user_data) |d| d.ptr else null,
            opts.min_num_bins,
            opts.max_num_bins,
            opts.num_triangles_per_bin,
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// Mean: faster to build, worse-balanced trees — what
    /// `.favor_build_speed` uses.
    pub fn initMean(triangles: Triangles) err.Error!TriangleSplitter {
        var handle: *c.TriangleSplitter = undefined;
        try err.check(c.zjoltTriangleSplitterCreateMean(
            triangles.vertices.ptr,
            @intCast(triangles.vertices.len),
            triangles.indices.ptr,
            @intCast(triangles.indices.len / 3),
            if (triangles.triangle_materials) |m| m.ptr else null,
            if (triangles.triangle_user_data) |d| d.ptr else null,
            &handle,
        ));
        return .{ .handle = handle };
    }

    pub fn deinit(self: TriangleSplitter) void {
        c.zjoltTriangleSplitterDestroy(self.handle);
    }

    /// The full range: every triangle this splitter was created with.
    pub fn initialRange(self: TriangleSplitter) TriangleRange {
        var out: TriangleRange = undefined;
        c.zjoltTriangleSplitterGetInitialRange(self.handle, &out);
        return out;
    }

    pub const Split = struct { left: TriangleRange, right: TriangleRange };

    /// `null` if no split could be made, or `range` is outside
    /// [0, this splitter's triangle count]. `AABBTreeBuilder.build` is the
    /// normal caller; call directly only to inspect the strategy's own
    /// decisions.
    pub fn split(self: TriangleSplitter, range: TriangleRange) ?Split {
        var left: TriangleRange = undefined;
        var right: TriangleRange = undefined;
        if (!c.zjoltTriangleSplitterSplit(self.handle, &range, &left, &right)) {
            return null;
        }
        return .{ .left = left, .right = right };
    }
};

//=============================================================================
// Tree building
//=============================================================================

/// What `AABBTreeBuilder.build` reports, read off the tree's root node.
/// Field for field, what `ZJoltAABBTreeBuilderStats` carries — documented on
/// the C side, in `ffi/zjolt_tree.h`.
pub const AABBTreeBuilderStats = c.AABBTreeBuilderStats;

/// Jolt's own optimised (SAH-guided) binary AABB tree, built over a
/// `TriangleSplitter` — the structure `MeshShape` itself is built from.
pub const AABBTreeBuilder = struct {
    handle: *c.AABBTreeBuilder,

    /// Borrows `splitter`, which must outlive this builder AND any
    /// `AABBTreeBuffer.init` built from it. `max_triangles_per_leaf` 0 is
    /// Jolt's default of 16.
    pub fn init(splitter: TriangleSplitter, max_triangles_per_leaf: u32) err.Error!AABBTreeBuilder {
        var handle: *c.AABBTreeBuilder = undefined;
        try err.check(c.zjoltAABBTreeBuilderCreate(splitter.handle, max_triangles_per_leaf, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: AABBTreeBuilder) void {
        c.zjoltAABBTreeBuilderDestroy(self.handle);
    }

    /// Recursively builds the tree. `error.InvalidArgument` if this builder
    /// has already built one.
    pub fn build(self: AABBTreeBuilder) err.Error!AABBTreeBuilderStats {
        var stats: AABBTreeBuilderStats = undefined;
        try err.check(c.zjoltAABBTreeBuilderBuild(self.handle, &stats));
        return stats;
    }

    /// The root node's own triangle count — non-zero only when the whole
    /// tree is one leaf. @see `triangleCountInTree` for the recursive total.
    pub fn triangleCount(self: AABBTreeBuilder) u32 {
        return c.zjoltAABBTreeBuilderGetTriangleCount(self.handle);
    }

    /// Recursive total across every leaf in the tree.
    pub fn triangleCountInTree(self: AABBTreeBuilder) u32 {
        return c.zjoltAABBTreeBuilderGetTriangleCountInTree(self.handle);
    }

    /// Surface Area Heuristic cost with caller-chosen weights, distinct from
    /// `AABBTreeBuilderStats.sah_cost` (fixed at 1, 1).
    pub fn calculateSAHCost(self: AABBTreeBuilder, cost_traversal: f32, cost_leaf: f32) f32 {
        return c.zjoltAABBTreeBuilderCalculateSAHCost(self.handle, cost_traversal, cost_leaf);
    }

    /// Whether the root node has children (false for a tree small enough to
    /// be one leaf).
    pub fn hasChildren(self: AABBTreeBuilder) bool {
        return c.zjoltAABBTreeBuilderHasChildren(self.handle);
    }

    /// Expands the root breadth-first to collect up to `requested`
    /// descendant nodes (the packed buffer's own node fan-out is 4),
    /// returning how many were found.
    pub fn numChildren(self: AABBTreeBuilder, requested: u32) u32 {
        return c.zjoltAABBTreeBuilderGetNChildren(self.handle, requested);
    }
};

//=============================================================================
// Packing into a queryable buffer
//=============================================================================

/// Byte-identical to `NodeCodecQuadTreeHalfFloat::Header`.
pub const AABBTreeNodeHeader = c.AABBTreeNodeHeader;

/// Byte-identical to `TriangleCodecIndexed8BitPackSOA4Flags::TriangleHeader`.
pub const AABBTreeTriangleHeader = c.AABBTreeTriangleHeader;

/// The packed buffer `MeshShape` itself is built from and queries against:
/// a quad-tree of half-float node bounds over 8-bit-packed indexed
/// triangles — the one codec pair Jolt instantiates.
pub const AABBTreeBuffer = struct {
    handle: *c.AABBTreeBuffer,

    /// Packs `builder`'s built tree, reading vertices back from the
    /// splitter it was built with. `store_user_data` mirrors
    /// `MeshShapeSettings`'s own per-triangle user data flag.
    pub fn init(builder: AABBTreeBuilder, store_user_data: bool) err.Error!AABBTreeBuffer {
        var handle: *c.AABBTreeBuffer = undefined;
        try err.check(c.zjoltAABBTreeBufferCreate(builder.handle, store_user_data, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: AABBTreeBuffer) void {
        c.zjoltAABBTreeBufferDestroy(self.handle);
    }

    /// Borrowed; valid until `deinit`.
    pub fn data(self: AABBTreeBuffer) []const u8 {
        var ptr: ?[*]const u8 = null;
        var size: usize = 0;
        c.zjoltAABBTreeBufferGetData(self.handle, &ptr, &size);
        return if (ptr) |p| p[0..size] else &.{};
    }

    pub fn nodeHeader(self: AABBTreeBuffer) err.Error!AABBTreeNodeHeader {
        var out: AABBTreeNodeHeader = undefined;
        try err.check(c.zjoltAABBTreeBufferGetNodeHeader(self.handle, &out));
        return out;
    }

    pub fn triangleHeader(self: AABBTreeBuffer) err.Error!AABBTreeTriangleHeader {
        var out: AABBTreeTriangleHeader = undefined;
        try err.check(c.zjoltAABBTreeBufferGetTriangleHeader(self.handle, &out));
        return out;
    }
};

//=============================================================================
// Islands
//=============================================================================

pub const BodyId = system_c.BodyId;

/// Groups linked bodies, constraints and contacts for the solver — what
/// `PhysicsSystem` keeps privately, with no accessor. A standalone instance
/// a host builds, drives and destroys itself. One call sequence per step:
/// `reserve` once; then, per step, any of `prepareContactConstraints`/
/// `prepareNonContactConstraints`, the `link*` calls, `finalize`, the
/// read-back calls, `resetIslands` — then repeat from `prepareContactConstraints`.
pub const IslandStats = system_c.IslandStats;

pub const IslandBuilder = struct {
    handle: *system_c.IslandBuilder,

    pub fn init() err.Error!IslandBuilder {
        var handle: *system_c.IslandBuilder = undefined;
        try err.check(system_c.zjoltIslandBuilderCreate(&handle));
        return .{ .handle = handle };
    }

    /// Frees the builder. Islands still built (`finalize` without a
    /// matching `resetIslands`) are freed with it.
    pub fn deinit(self: IslandBuilder) void {
        system_c.zjoltIslandBuilderDestroy(self.handle);
    }

    /// Allocates for up to `max_active_bodies` bodies, identified in every
    /// call below by index into the host's own active-body list. Required
    /// before any other call; `error.InvalidArgument` if already called.
    pub fn reserve(self: IslandBuilder, max_active_bodies: u32) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderInit(self.handle, max_active_bodies));
    }

    /// Merges the islands `first` and `second` belong to. Silently ignores
    /// an index at or past `max_active_bodies` — Jolt's own behaviour, since
    /// a static body has no island to link into.
    pub fn linkBodies(self: IslandBuilder, first: u32, second: u32) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderLinkBodies(self.handle, first, second));
    }

    /// Allocates the contact-link table. Required before `linkContact`,
    /// once per `reserve`/`resetIslands` cycle.
    pub fn prepareContactConstraints(
        self: IslandBuilder,
        max_contacts: u32,
        temp_allocator: *const TempAllocator,
    ) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderPrepareContactConstraints(
            self.handle,
            max_contacts,
            temp_allocator,
        ));
    }

    /// Allocates the constraint-link table. Required before
    /// `linkConstraint`, once per `reserve`/`resetIslands` cycle.
    pub fn prepareNonContactConstraints(
        self: IslandBuilder,
        num_constraints: u32,
        temp_allocator: *const TempAllocator,
    ) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderPrepareNonContactConstraints(
            self.handle,
            num_constraints,
            temp_allocator,
        ));
    }

    /// Records that constraint `constraint_index` (below the count given to
    /// `prepareNonContactConstraints`) touches the active body at
    /// `index_in_active_body_list`.
    pub fn linkConstraint(
        self: IslandBuilder,
        constraint_index: u32,
        index_in_active_body_list: u32,
    ) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderLinkConstraint(
            self.handle,
            constraint_index,
            index_in_active_body_list,
        ));
    }

    /// As `linkConstraint`, for contact `contact_index` (below the count
    /// given to `prepareContactConstraints`).
    pub fn linkContact(
        self: IslandBuilder,
        contact_index: u32,
        index_in_active_body_list: u32,
    ) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderLinkContact(
            self.handle,
            contact_index,
            index_in_active_body_list,
        ));
    }

    /// Closes linking and sorts islands largest-first. `active_bodies` is
    /// in the same order the `link*` calls indexed into; `num_contacts`
    /// must match what `prepareContactConstraints` (if called) was given.
    /// Required before every read-back call below.
    pub fn finalize(
        self: IslandBuilder,
        active_bodies: []const BodyId,
        num_contacts: u32,
        temp_allocator: *const TempAllocator,
    ) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderFinalize(
            self.handle,
            if (active_bodies.len != 0) active_bodies.ptr else null,
            @intCast(active_bodies.len),
            num_contacts,
            temp_allocator,
        ));
    }

    /// 0 before `finalize` has run.
    pub fn numIslands(self: IslandBuilder) u32 {
        return system_c.zjoltIslandBuilderGetNumIslands(self.handle);
    }

    /// Two-call protocol, as `PhysicsSystem.getBodies`: the returned count
    /// is always island `island_index`'s true body count; `out` null (or
    /// short) sizes without copying, and a short `out` is
    /// `error.BufferTooSmall`.
    pub fn bodiesInIsland(self: IslandBuilder, island_index: u32, out: ?[]BodyId) err.Error!u32 {
        var count: u32 = 0;
        try err.check(system_c.zjoltIslandBuilderGetBodiesInIsland(
            self.handle,
            island_index,
            if (out) |o| o.ptr else null,
            if (out) |o| @intCast(o.len) else 0,
            &count,
        ));
        return count;
    }

    /// As `bodiesInIsland`, for the constraint indices `linkConstraint`
    /// recorded against this island. 0 when `prepareNonContactConstraints`
    /// was never called.
    pub fn constraintsInIsland(self: IslandBuilder, island_index: u32, out: ?[]u32) err.Error!u32 {
        var count: u32 = 0;
        try err.check(system_c.zjoltIslandBuilderGetConstraintsInIsland(
            self.handle,
            island_index,
            if (out) |o| o.ptr else null,
            if (out) |o| @intCast(o.len) else 0,
            &count,
        ));
        return count;
    }

    /// As `bodiesInIsland`, for the contact indices `linkContact` recorded
    /// against this island. 0 when `prepareContactConstraints` was never
    /// called.
    pub fn contactsInIsland(self: IslandBuilder, island_index: u32, out: ?[]u32) err.Error!u32 {
        var count: u32 = 0;
        try err.check(system_c.zjoltIslandBuilderGetContactsInIsland(
            self.handle,
            island_index,
            if (out) |o| o.ptr else null,
            if (out) |o| @intCast(o.len) else 0,
            &count,
        ));
        return count;
    }

    pub fn numPositionSteps(self: IslandBuilder, island_index: u32) err.Error!u32 {
        var out: u32 = 0;
        try err.check(system_c.zjoltIslandBuilderGetNumPositionSteps(self.handle, island_index, &out));
        return out;
    }

    /// Per-island solver timings — IslandBuilder::GetIslandStats. Ticks are
    /// the platform's cycle counter, comparable to each other rather than to
    /// wall-clock, and reset every step. `error.Unsupported` unless built
    /// with `-Dtrack_simulation_stats`, which is what fills these in.
    pub fn stats(self: IslandBuilder, island_index: u32) err.Error!IslandStats {
        var out: IslandStats = undefined;
        try err.check(system_c.zjoltIslandBuilderGetStats(self.handle, island_index, &out));
        return out;
    }

    /// `num_position_steps` must be below 256, where Jolt asserts.
    pub fn setNumPositionSteps(self: IslandBuilder, island_index: u32, num_position_steps: u32) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderSetNumPositionSteps(
            self.handle,
            island_index,
            num_position_steps,
        ));
    }

    /// Frees everything `finalize` (and `prepare*`, if called) allocated,
    /// so the builder is ready for the next `prepareContactConstraints`/
    /// `link*`/`finalize` cycle. `error.InvalidArgument` if `finalize` has
    /// not run since the last `reserve`/`resetIslands`.
    pub fn resetIslands(self: IslandBuilder, temp_allocator: *const TempAllocator) err.Error!void {
        try err.check(system_c.zjoltIslandBuilderResetIslands(self.handle, temp_allocator));
    }
};

//=============================================================================
// Large island splitting
//
// `LargeIslandSplitter` assigns bodies in a large island to parallel-safe
// groups; upstream marks it "an internal part of PhysicsSystem, it has no
// functions that can be called by users of the library" because it reads a
// live `BodyManager`'s own active-body bookkeeping. This is that algorithm
// ported to the same host-supplied-index model `IslandBuilder` already
// uses, so a host driving one can drive this too.
//=============================================================================

/// `LargeIslandSplitter::cNumSplits` / `cNonParallelSplitIdx`: how many
/// parallel splits fit in one `u32` mask, and the reserved index for a body
/// that could not join any of the other 31 without colliding with itself.
pub const num_splits: u32 = 32;
pub const non_parallel_split_index: u32 = num_splits - 1;

/// The per-body bitmask state `LargeIslandSplitter::AssignSplit` and
/// `AssignToNonParallelSplit` maintain — which splits a body already
/// belongs to. A standalone counterpart to `IslandBuilder`: a host owns one
/// array, indexed the same way `IslandBuilder.linkBodies` indexes, sized to
/// its own active-body count and zeroed before the first `assignSplit` of
/// an island.
pub const SplitMasks = struct {
    masks: []u32,

    /// Borrows `masks`, which the caller sizes to its own active-body count
    /// and zeroes before use — `LargeIslandSplitter::Prepare`'s own
    /// precondition on `inNumActiveBodies`.
    pub fn init(masks: []u32) SplitMasks {
        return .{ .masks = masks };
    }

    fn assignOne(self: SplitMasks, index: u32) u32 {
        const trailing: u32 = @intCast(@ctz(~self.masks[index]));
        const split = @min(trailing, non_parallel_split_index);
        self.masks[index] |= @as(u32, 1) << @as(u5, @intCast(split));
        return split;
    }

    /// `Constraint::BuildIslandSplits` for a two-body constraint — what
    /// `TwoBodyConstraint`'s override calls `LargeIslandSplitter::AssignSplit`
    /// for. Assigns both bodies to the lowest split index neither already
    /// occupies, or just the one body if the other is not dynamic or out
    /// of range for this splitter. Returns the split index assigned.
    pub fn assignSplit(
        self: SplitMasks,
        body1_index: u32,
        body1_dynamic: bool,
        body2_index: u32,
        body2_dynamic: bool,
    ) u32 {
        const active1 = body1_dynamic and body1_index < self.masks.len;
        const active2 = body2_dynamic and body2_index < self.masks.len;
        if (!active1) return self.assignOne(body2_index);
        if (!active2) return self.assignOne(body1_index);

        const trailing: u32 = @intCast(@ctz(~(self.masks[body1_index] | self.masks[body2_index])));
        const split = @min(trailing, non_parallel_split_index);
        const bit = @as(u32, 1) << @as(u5, @intCast(split));
        self.masks[body1_index] |= bit;
        self.masks[body2_index] |= bit;
        return split;
    }

    /// `Constraint::BuildIslandSplits` for a single-body constraint — what
    /// `VehicleConstraint`'s override calls
    /// `LargeIslandSplitter::AssignToNonParallelSplit` for: pins
    /// `body_index` to the reserved non-parallel split. A no-op, still
    /// returning `non_parallel_split_index`, if `dynamic` is false or
    /// `body_index` is out of range for this splitter.
    pub fn assignToNonParallelSplit(self: SplitMasks, body_index: u32, dynamic: bool) u32 {
        if (dynamic and body_index < self.masks.len) {
            self.masks[body_index] |= @as(u32, 1) << @as(u5, @intCast(non_parallel_split_index));
        }
        return non_parallel_split_index;
    }
};
