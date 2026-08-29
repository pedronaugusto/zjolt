//===----------------------------------------------------------------------===//
// zjolt — the AABB tree builder: splits a triangle soup, builds Jolt's own
// optimised tree over it, and packs the tree into the queryable buffer
// MeshShape itself is built from. Standalone from MeshShape; reusable by a
// host that wants the tree (or its stats) without a shape around it.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_TREE_H_
#define ZJOLT_TREE_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Triangle splitting
//
// A splitter owns a copy of the triangle soup it was created with and picks
// how AABBTreeBuilder divides it at each level. Feed the same splitter to
// exactly one builder; a strategy choice is FAVOR_RUNTIME_PERFORMANCE
// (binning) or FAVOR_BUILD_SPEED (mean) in MeshShapeSettings terms.
//===----------------------------------------------------------------------===//

typedef struct ZJoltTriangleSplitter ZJoltTriangleSplitter;

/// A half-open range of triangle indices, [begin, end), into a splitter's own
/// sorted order — not indices into the caller's original triangle array.
typedef struct ZJoltTriangleRange {
  uint32_t begin;
  uint32_t end;
} ZJoltTriangleRange;

/// Binning splitter: slower to build, better-balanced trees. What
/// MeshShapeSettings uses for FAVOR_RUNTIME_PERFORMANCE. `indices` holds
/// 3*num_triangles vertex indices. `min_num_bins`, `max_num_bins` and
/// `num_triangles_per_bin` are 0 individually for Jolt's defaults (8, 128,
/// 6); given non-zero, `min_num_bins` must not exceed `max_num_bins`.
ZJOLT_API ZJoltResult zjoltTriangleSplitterCreateBinning(
    const ZJoltVec3 *vertices, uint32_t num_vertices, const uint32_t *indices,
    uint32_t num_triangles, const uint32_t *triangle_materials,
    const uint32_t *triangle_user_data, uint32_t min_num_bins,
    uint32_t max_num_bins, uint32_t num_triangles_per_bin,
    ZJoltTriangleSplitter **out);

/// Mean splitter: splits on the mean of the axis with the biggest centroid
/// deviation. Faster to build, worse-balanced trees. What MeshShapeSettings
/// uses for FAVOR_BUILD_SPEED. Same triangle arguments as the binning form.
ZJOLT_API ZJoltResult zjoltTriangleSplitterCreateMean(
    const ZJoltVec3 *vertices, uint32_t num_vertices, const uint32_t *indices,
    uint32_t num_triangles, const uint32_t *triangle_materials,
    const uint32_t *triangle_user_data, ZJoltTriangleSplitter **out);

/// Destroys a splitter not yet consumed by an AABBTreeBuilder still in use —
/// the builder holds a borrowed reference, and a splitter destroyed while
/// its builder is still live dangles.
ZJOLT_API void zjoltTriangleSplitterDestroy(ZJoltTriangleSplitter *splitter);

/// The full range: every triangle the splitter was created with, in its own
/// sorted order. All-zero if `splitter` is NULL.
ZJOLT_API void zjoltTriangleSplitterGetInitialRange(
    const ZJoltTriangleSplitter *splitter, ZJoltTriangleRange *out);

/// Splits `range` into `out_left`/`out_right`, reordering the splitter's own
/// triangle order as it does; false if no split could be made, or if
/// `splitter`/`range`/`out_left`/`out_right` is NULL, or `range` is outside
/// [0, the splitter's triangle count]. AABBTreeBuilder is the normal caller;
/// call directly only to inspect the strategy's own decisions.
ZJOLT_API bool zjoltTriangleSplitterSplit(ZJoltTriangleSplitter *splitter,
                                          const ZJoltTriangleRange *range,
                                          ZJoltTriangleRange *out_left,
                                          ZJoltTriangleRange *out_right);

//===----------------------------------------------------------------------===//
// Tree building
//
// Consumes a splitter to build Jolt's own optimised (SAH-guided) binary AABB
// tree in memory. The stats below describe the RESULT of Build, read off its
// root node.
//===----------------------------------------------------------------------===//

typedef struct ZJoltAABBTreeBuilder ZJoltAABBTreeBuilder;

/// What Build reports about the tree it just built, read off the root node:
/// borrowed splitter name aside, every field is exactly what
/// AABBTreeBuilder::Node's own stat methods compute (node/leaf counts, min/max
/// depth, and SAH cost at cost_traversal = cost_leaf = 1, the weights Build
/// itself uses) or what the splitter and builder were configured with.
typedef struct ZJoltAABBTreeBuilderStats {
  /// Borrowed, process-lifetime static string: "TriangleSplitterBinning" or
  /// "TriangleSplitterMean".
  const char *splitter_name;
  int32_t splitter_leaf_size;
  float sah_cost;
  int32_t min_depth;
  int32_t max_depth;
  int32_t node_count;
  int32_t leaf_node_count;
  int32_t max_triangles_per_leaf;
  int32_t tree_min_triangles_per_leaf;
  int32_t tree_max_triangles_per_leaf;
  float tree_avg_triangles_per_leaf;
} ZJoltAABBTreeBuilderStats;

/// Borrows `splitter`, which must outlive this builder AND any
/// zjoltAABBTreeBufferCreate call made against it — Build reads the
/// splitter's triangles and Convert later reads its vertices.
/// `max_triangles_per_leaf` is 0 for Jolt's default of 16.
ZJOLT_API ZJoltResult zjoltAABBTreeBuilderCreate(
    ZJoltTriangleSplitter *splitter, uint32_t max_triangles_per_leaf,
    ZJoltAABBTreeBuilder **out);

ZJOLT_API void zjoltAABBTreeBuilderDestroy(ZJoltAABBTreeBuilder *builder);

/// Recursively builds the tree. Every stat getter below, and
/// zjoltAABBTreeBufferCreate, requires this to have run first;
/// ZJOLT_RESULT_INVALID_ARGUMENT if it already has.
ZJOLT_API ZJoltResult zjoltAABBTreeBuilderBuild(
    ZJoltAABBTreeBuilder *builder, ZJoltAABBTreeBuilderStats *out_stats);

/// The root node's own triangle count — non-zero only when the whole tree is
/// one leaf. @see zjoltAABBTreeBuilderGetTriangleCountInTree for the
/// recursive total. 0 if `builder` is NULL or not yet built.
ZJOLT_API uint32_t zjoltAABBTreeBuilderGetTriangleCount(
    const ZJoltAABBTreeBuilder *builder);

/// Recursive total across every leaf in the tree. 0 if `builder` is NULL or
/// not yet built.
ZJOLT_API uint32_t zjoltAABBTreeBuilderGetTriangleCountInTree(
    const ZJoltAABBTreeBuilder *builder);

/// Surface Area Heuristic cost with caller-chosen weights, distinct from
/// ZJoltAABBTreeBuilderStats::sah_cost (fixed at 1, 1). 0 if `builder` is
/// NULL, not yet built, or the root's bounds have zero surface area.
ZJOLT_API float zjoltAABBTreeBuilderCalculateSAHCost(
    const ZJoltAABBTreeBuilder *builder, float cost_traversal,
    float cost_leaf);

/// Whether the root node has children (false for a tree small enough to be
/// one leaf). False if `builder` is NULL or not yet built.
ZJOLT_API bool zjoltAABBTreeBuilderHasChildren(
    const ZJoltAABBTreeBuilder *builder);

/// Expands the root breadth-first to collect up to `requested` descendant
/// nodes (the packed buffer's own node fan-out is 4), returning how many
/// were found. 0 if `builder` is NULL, not yet built, or `requested` is 0.
ZJOLT_API uint32_t zjoltAABBTreeBuilderGetNChildren(
    const ZJoltAABBTreeBuilder *builder, uint32_t requested);

//===----------------------------------------------------------------------===//
// Packing into a queryable buffer
//
// The layout MeshShape itself stores and queries against: a quad-tree of
// half-float node bounds over 8-bit-packed indexed triangles. The one codec
// pair Jolt instantiates; @see UPSTREAM.md for the ones this does not expose.
//===----------------------------------------------------------------------===//

typedef struct ZJoltAABBTreeBuffer ZJoltAABBTreeBuffer;

/// Byte-identical to NodeCodecQuadTreeHalfFloat::Header.
typedef struct ZJoltAABBTreeNodeHeader {
  ZJoltVec3 root_bounds_min;
  ZJoltVec3 root_bounds_max;
  uint32_t root_properties;
  uint8_t block_id_bits;
  uint8_t padding[3];
} ZJoltAABBTreeNodeHeader;

/// Byte-identical to TriangleCodecIndexed8BitPackSOA4Flags::TriangleHeader:
/// vertex_position = offset + scale * compressed_vertex_position.
typedef struct ZJoltAABBTreeTriangleHeader {
  ZJoltVec3 offset;
  ZJoltVec3 scale;
} ZJoltAABBTreeTriangleHeader;

/// Packs `builder`'s built tree into the queryable buffer, reading vertices
/// back from the splitter it was built with (@see zjoltAABBTreeBuilderCreate).
/// `store_user_data` mirrors MeshShapeSettings::mPerTriangleUserData.
/// ZJOLT_RESULT_INVALID_ARGUMENT if `builder` has not been built, or the tree
/// does not fit the buffer format (detail in zjoltLastError) — most commonly
/// `max_triangles_per_leaf` past what one packed node can index (15).
ZJOLT_API ZJoltResult zjoltAABBTreeBufferCreate(ZJoltAABBTreeBuilder *builder,
                                                bool store_user_data,
                                                ZJoltAABBTreeBuffer **out);

ZJOLT_API void zjoltAABBTreeBufferDestroy(ZJoltAABBTreeBuffer *buffer);

/// The packed buffer's own bytes, borrowed and valid until `buffer` is
/// destroyed. `*out_data` is NULL only if `buffer` is NULL.
ZJOLT_API void zjoltAABBTreeBufferGetData(const ZJoltAABBTreeBuffer *buffer,
                                          const void **out_data,
                                          size_t *out_size);

ZJOLT_API ZJoltResult zjoltAABBTreeBufferGetNodeHeader(
    const ZJoltAABBTreeBuffer *buffer, ZJoltAABBTreeNodeHeader *out);

ZJOLT_API ZJoltResult zjoltAABBTreeBufferGetTriangleHeader(
    const ZJoltAABBTreeBuffer *buffer, ZJoltAABBTreeTriangleHeader *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_TREE_H_
