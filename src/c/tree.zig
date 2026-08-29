//! ZJolt C declarations for triangle splitting, AABB tree building, and
//! packing a tree into the buffer MeshShape itself queries.
//!
//! Mirrors `ffi/zjolt_tree.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Result = core.Result;
pub const Vec3 = core.Vec3;

pub const TriangleSplitter = opaque {};

/// A half-open range of triangle indices, [begin, end), into a splitter's
/// own sorted order.
pub const TriangleRange = extern struct {
    begin: u32,
    end: u32,
};

pub extern fn zjoltTriangleSplitterCreateBinning(vertices: [*]const Vec3, num_vertices: u32, indices: [*]const u32, num_triangles: u32, triangle_materials: ?[*]const u32, triangle_user_data: ?[*]const u32, min_num_bins: u32, max_num_bins: u32, num_triangles_per_bin: u32, out: **TriangleSplitter) Result;

pub extern fn zjoltTriangleSplitterCreateMean(vertices: [*]const Vec3, num_vertices: u32, indices: [*]const u32, num_triangles: u32, triangle_materials: ?[*]const u32, triangle_user_data: ?[*]const u32, out: **TriangleSplitter) Result;

pub extern fn zjoltTriangleSplitterDestroy(splitter: ?*TriangleSplitter) void;

pub extern fn zjoltTriangleSplitterGetInitialRange(splitter: *const TriangleSplitter, out: *TriangleRange) void;

pub extern fn zjoltTriangleSplitterSplit(splitter: *TriangleSplitter, range: *const TriangleRange, out_left: *TriangleRange, out_right: *TriangleRange) bool;

pub const AABBTreeBuilder = opaque {};

/// What Build reports about the tree it just built, read off the root node.
/// Field for field, what `AABBTreeBuilderStats` carries — documented on the
/// C side, in `ffi/zjolt_tree.h`.
pub const AABBTreeBuilderStats = extern struct {
    splitter_name: ?[*:0]const u8,
    splitter_leaf_size: i32,
    sah_cost: f32,
    min_depth: i32,
    max_depth: i32,
    node_count: i32,
    leaf_node_count: i32,
    max_triangles_per_leaf: i32,
    tree_min_triangles_per_leaf: i32,
    tree_max_triangles_per_leaf: i32,
    tree_avg_triangles_per_leaf: f32,
};

pub extern fn zjoltAABBTreeBuilderCreate(splitter: *TriangleSplitter, max_triangles_per_leaf: u32, out: **AABBTreeBuilder) Result;

pub extern fn zjoltAABBTreeBuilderDestroy(builder: ?*AABBTreeBuilder) void;

pub extern fn zjoltAABBTreeBuilderBuild(builder: *AABBTreeBuilder, out_stats: ?*AABBTreeBuilderStats) Result;

pub extern fn zjoltAABBTreeBuilderGetTriangleCount(builder: *const AABBTreeBuilder) u32;

pub extern fn zjoltAABBTreeBuilderGetTriangleCountInTree(builder: *const AABBTreeBuilder) u32;

pub extern fn zjoltAABBTreeBuilderCalculateSAHCost(builder: *const AABBTreeBuilder, cost_traversal: f32, cost_leaf: f32) f32;

pub extern fn zjoltAABBTreeBuilderHasChildren(builder: *const AABBTreeBuilder) bool;

pub extern fn zjoltAABBTreeBuilderGetNChildren(builder: *const AABBTreeBuilder, requested: u32) u32;

pub const AABBTreeBuffer = opaque {};

/// Byte-identical to `NodeCodecQuadTreeHalfFloat::Header`.
pub const AABBTreeNodeHeader = extern struct {
    root_bounds_min: Vec3,
    root_bounds_max: Vec3,
    root_properties: u32,
    block_id_bits: u8,
    padding: [3]u8 = .{ 0, 0, 0 },
};

/// Byte-identical to `TriangleCodecIndexed8BitPackSOA4Flags::TriangleHeader`.
pub const AABBTreeTriangleHeader = extern struct {
    offset: Vec3,
    scale: Vec3,
};

pub extern fn zjoltAABBTreeBufferCreate(builder: *AABBTreeBuilder, store_user_data: bool, out: **AABBTreeBuffer) Result;

pub extern fn zjoltAABBTreeBufferDestroy(buffer: ?*AABBTreeBuffer) void;

pub extern fn zjoltAABBTreeBufferGetData(buffer: *const AABBTreeBuffer, out_data: *?[*]const u8, out_size: *usize) void;

pub extern fn zjoltAABBTreeBufferGetNodeHeader(buffer: *const AABBTreeBuffer, out: *AABBTreeNodeHeader) Result;

pub extern fn zjoltAABBTreeBufferGetTriangleHeader(buffer: *const AABBTreeBuffer, out: *AABBTreeTriangleHeader) Result;
