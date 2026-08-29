//===----------------------------------------------------------------------===//
// zjolt — triangle splitting, AABB tree building, and packing the tree into
// the buffer MeshShape itself queries. The node and triangle codecs are
// template policy parameters of AABBTreeToBuffer; this binds the one pair
// Jolt instantiates (MeshShape.cpp) rather than exposing them separately.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/AABBTree/AABBTreeBuilder.h>
#include <Jolt/AABBTree/AABBTreeToBuffer.h>
#include <Jolt/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.h>
#include <Jolt/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.h>
#include <Jolt/Core/ByteBuffer.h>
#include <Jolt/Geometry/IndexedTriangle.h>
#include <Jolt/TriangleSplitter/TriangleSplitterBinning.h>
#include <Jolt/TriangleSplitter/TriangleSplitterMean.h>

namespace {

//===----------------------------------------------------------------------===//
// Triangle splitting
//===----------------------------------------------------------------------===//

/// Owns the triangle soup a splitter references, so the splitter's base-class
/// `const VertexList &`/`const IndexedTriangleList &` bind to memory this
/// struct's own lifetime governs rather than the caller's arrays.
template <typename T>
void DestroySplitter(JPH::TriangleSplitter *impl) {
  zjolt::Delete(static_cast<T *>(impl));
}

/// Validates the triangle soup a splitter or a mesh needs and copies it into
/// Jolt's own containers. Shared by both splitter strategies.
ZJoltResult BuildTriangleArrays(const ZJoltVec3 *vertices,
                                uint32_t num_vertices, const uint32_t *indices,
                                uint32_t num_triangles,
                                const uint32_t *triangle_materials,
                                const uint32_t *triangle_user_data,
                                JPH::Array<JPH::Float3> *out_vertices,
                                JPH::Array<JPH::IndexedTriangle> *out_triangles) {
  if (vertices == nullptr || indices == nullptr || num_vertices == 0 ||
      num_triangles == 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a triangle splitter needs at least one vertex and one triangle");
  }
  for (uint32_t i = 0; i < num_triangles * 3; ++i) {
    if (indices[i] >= num_vertices) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a triangle index is out of range for the vertex array");
    }
  }

  out_vertices->reserve(num_vertices);
  for (uint32_t i = 0; i < num_vertices; ++i) {
    out_vertices->push_back(
        JPH::Float3(vertices[i].x, vertices[i].y, vertices[i].z));
  }

  out_triangles->reserve(num_triangles);
  for (uint32_t i = 0; i < num_triangles; ++i) {
    out_triangles->push_back(JPH::IndexedTriangle(
        indices[i * 3 + 0], indices[i * 3 + 1], indices[i * 3 + 2],
        triangle_materials != nullptr ? triangle_materials[i] : 0u,
        triangle_user_data != nullptr ? triangle_user_data[i] : 0u));
  }
  return ZJOLT_RESULT_OK;
}

}  // namespace

//===----------------------------------------------------------------------===//
// Handle types
//===----------------------------------------------------------------------===//

/// `vertices`/`triangles` back the base class's reference members, so they
/// are declared before `impl` and must outlive it — true for the lifetime of
/// this struct, since `impl` is destroyed first regardless.
struct ZJoltTriangleSplitter {
  JPH::Array<JPH::Float3> vertices;
  JPH::Array<JPH::IndexedTriangle> triangles;
  JPH::TriangleSplitter *impl = nullptr;
  void (*destroy)(JPH::TriangleSplitter *) = nullptr;
};

/// `splitter` is borrowed: Build reads its triangles, and a later
/// zjoltAABBTreeBufferCreate reads its vertices, so it must outlive both.
struct ZJoltAABBTreeBuilder {
  JPH::AABBTreeBuilder impl;
  ZJoltTriangleSplitter *splitter;
  const JPH::AABBTreeBuilder::Node *root = nullptr;
  JPH::AABBTreeBuilderStats stats{};
  bool built = false;

  ZJoltAABBTreeBuilder(ZJoltTriangleSplitter *s, JPH::uint max_triangles_per_leaf)
      : impl(*s->impl, max_triangles_per_leaf), splitter(s) {}
};

/// The one codec pair Jolt instantiates (MeshShape.cpp): a quad-tree of
/// half-float node bounds over 8-bit-packed indexed triangles.
using ZJoltTreeCodec =
    JPH::AABBTreeToBuffer<JPH::TriangleCodecIndexed8BitPackSOA4Flags,
                          JPH::NodeCodecQuadTreeHalfFloat>;

struct ZJoltAABBTreeBuffer {
  ZJoltTreeCodec impl;
};

static_assert(sizeof(ZJoltAABBTreeNodeHeader) == sizeof(ZJoltTreeCodec::NodeHeader),
              "ZJoltAABBTreeNodeHeader must mirror NodeCodecQuadTreeHalfFloat::Header");
static_assert(sizeof(ZJoltAABBTreeTriangleHeader) ==
                  sizeof(ZJoltTreeCodec::TriangleHeader),
              "ZJoltAABBTreeTriangleHeader must mirror "
              "TriangleCodecIndexed8BitPackSOA4Flags::TriangleHeader");

extern "C" {

//===----------------------------------------------------------------------===//
// Triangle splitting
//===----------------------------------------------------------------------===//

ZJoltResult zjoltTriangleSplitterCreateBinning(
    const ZJoltVec3 *vertices, uint32_t num_vertices, const uint32_t *indices,
    uint32_t num_triangles, const uint32_t *triangle_materials,
    const uint32_t *triangle_user_data, uint32_t min_num_bins,
    uint32_t max_num_bins, uint32_t num_triangles_per_bin,
    ZJoltTriangleSplitter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const uint32_t resolved_min = min_num_bins != 0 ? min_num_bins : 8;
  const uint32_t resolved_max = max_num_bins != 0 ? max_num_bins : 128;
  const uint32_t resolved_per_bin =
      num_triangles_per_bin != 0 ? num_triangles_per_bin : 6;
  if (resolved_min > resolved_max) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "min_num_bins must not exceed max_num_bins");
  }

  ZJoltTriangleSplitter *splitter = zjolt::New<ZJoltTriangleSplitter>();
  if (splitter == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  const ZJoltResult built = BuildTriangleArrays(
      vertices, num_vertices, indices, num_triangles, triangle_materials,
      triangle_user_data, &splitter->vertices, &splitter->triangles);
  if (built != ZJOLT_RESULT_OK) {
    zjolt::Delete(splitter);
    return built;
  }

  splitter->impl = zjolt::New<JPH::TriangleSplitterBinning>(
      splitter->vertices, splitter->triangles, resolved_min, resolved_max,
      resolved_per_bin);
  if (splitter->impl == nullptr) {
    zjolt::Delete(splitter);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }
  splitter->destroy = DestroySplitter<JPH::TriangleSplitterBinning>;

  zjolt::HandleCreated();
  *out = splitter;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltTriangleSplitterCreateMean(
    const ZJoltVec3 *vertices, uint32_t num_vertices, const uint32_t *indices,
    uint32_t num_triangles, const uint32_t *triangle_materials,
    const uint32_t *triangle_user_data, ZJoltTriangleSplitter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  ZJoltTriangleSplitter *splitter = zjolt::New<ZJoltTriangleSplitter>();
  if (splitter == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  const ZJoltResult built = BuildTriangleArrays(
      vertices, num_vertices, indices, num_triangles, triangle_materials,
      triangle_user_data, &splitter->vertices, &splitter->triangles);
  if (built != ZJOLT_RESULT_OK) {
    zjolt::Delete(splitter);
    return built;
  }

  splitter->impl = zjolt::New<JPH::TriangleSplitterMean>(splitter->vertices,
                                                          splitter->triangles);
  if (splitter->impl == nullptr) {
    zjolt::Delete(splitter);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }
  splitter->destroy = DestroySplitter<JPH::TriangleSplitterMean>;

  zjolt::HandleCreated();
  *out = splitter;
  return ZJOLT_RESULT_OK;
}

void zjoltTriangleSplitterDestroy(ZJoltTriangleSplitter *splitter) {
  if (splitter == nullptr) return;
  if (splitter->impl != nullptr) splitter->destroy(splitter->impl);
  zjolt::Delete(splitter);
  zjolt::HandleDestroyed();
}

void zjoltTriangleSplitterGetInitialRange(const ZJoltTriangleSplitter *splitter,
                                          ZJoltTriangleRange *out) {
  if (out == nullptr) return;
  *out = ZJoltTriangleRange{0, 0};
  if (splitter == nullptr || splitter->impl == nullptr) return;
  const JPH::TriangleSplitter::Range range = splitter->impl->GetInitialRange();
  *out = ZJoltTriangleRange{range.mBegin, range.mEnd};
}

bool zjoltTriangleSplitterSplit(ZJoltTriangleSplitter *splitter,
                                const ZJoltTriangleRange *range,
                                ZJoltTriangleRange *out_left,
                                ZJoltTriangleRange *out_right) {
  if (splitter == nullptr || splitter->impl == nullptr || range == nullptr ||
      out_left == nullptr || out_right == nullptr) {
    return false;
  }
  const uint32_t triangle_count =
      static_cast<uint32_t>(splitter->triangles.size());
  if (range->begin > range->end || range->end > triangle_count) return false;

  const JPH::TriangleSplitter::Range in(range->begin, range->end);
  JPH::TriangleSplitter::Range left, right;
  if (!splitter->impl->Split(in, left, right)) return false;

  *out_left = ZJoltTriangleRange{left.mBegin, left.mEnd};
  *out_right = ZJoltTriangleRange{right.mBegin, right.mEnd};
  return true;
}

//===----------------------------------------------------------------------===//
// Tree building
//===----------------------------------------------------------------------===//

ZJoltResult zjoltAABBTreeBuilderCreate(ZJoltTriangleSplitter *splitter,
                                       uint32_t max_triangles_per_leaf,
                                       ZJoltAABBTreeBuilder **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(splitter, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const uint32_t resolved =
      max_triangles_per_leaf != 0 ? max_triangles_per_leaf : 16;
  ZJoltAABBTreeBuilder *builder = zjolt::New<ZJoltAABBTreeBuilder>(
      splitter, static_cast<JPH::uint>(resolved));
  if (builder == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = builder;
  return ZJOLT_RESULT_OK;
}

void zjoltAABBTreeBuilderDestroy(ZJoltAABBTreeBuilder *builder) {
  if (builder == nullptr) return;
  zjolt::Delete(builder);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltAABBTreeBuilderBuild(ZJoltAABBTreeBuilder *builder,
                                      ZJoltAABBTreeBuilderStats *out_stats) {
  ZJOLT_ENTER(out_stats);
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (builder->built) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this builder has already built its tree");
  }

  builder->root = builder->impl.Build(builder->stats);
  builder->built = true;

  if (out_stats != nullptr) {
    const JPH::AABBTreeBuilderStats &s = builder->stats;
    *out_stats = ZJoltAABBTreeBuilderStats{
        s.mSplitterStats.mSplitterName,
        s.mSplitterStats.mLeafSize,
        s.mSAHCost,
        s.mMinDepth,
        s.mMaxDepth,
        s.mNodeCount,
        s.mLeafNodeCount,
        s.mMaxTrianglesPerLeaf,
        s.mTreeMinTrianglesPerLeaf,
        s.mTreeMaxTrianglesPerLeaf,
        s.mTreeAvgTrianglesPerLeaf,
    };
  }
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltAABBTreeBuilderGetTriangleCount(
    const ZJoltAABBTreeBuilder *builder) {
  if (builder == nullptr || !builder->built) return 0;
  return builder->root->GetTriangleCount();
}

uint32_t zjoltAABBTreeBuilderGetTriangleCountInTree(
    const ZJoltAABBTreeBuilder *builder) {
  if (builder == nullptr || !builder->built) return 0;
  return builder->root->GetTriangleCountInTree(builder->impl.GetNodes());
}

float zjoltAABBTreeBuilderCalculateSAHCost(const ZJoltAABBTreeBuilder *builder,
                                           float cost_traversal,
                                           float cost_leaf) {
  if (builder == nullptr || !builder->built) return 0.0f;
  return builder->root->CalculateSAHCost(builder->impl.GetNodes(),
                                         cost_traversal, cost_leaf);
}

bool zjoltAABBTreeBuilderHasChildren(const ZJoltAABBTreeBuilder *builder) {
  if (builder == nullptr || !builder->built) return false;
  return builder->root->HasChildren();
}

uint32_t zjoltAABBTreeBuilderGetNChildren(const ZJoltAABBTreeBuilder *builder,
                                          uint32_t requested) {
  if (builder == nullptr || !builder->built || requested == 0) return 0;
  JPH::Array<const JPH::AABBTreeBuilder::Node *> children;
  builder->root->GetNChildren(builder->impl.GetNodes(), requested, children);
  return static_cast<uint32_t>(children.size());
}

//===----------------------------------------------------------------------===//
// Packing into a queryable buffer
//===----------------------------------------------------------------------===//

ZJoltResult zjoltAABBTreeBufferCreate(ZJoltAABBTreeBuilder *builder,
                                      bool store_user_data,
                                      ZJoltAABBTreeBuffer **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(builder, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!builder->built) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "the builder has not built a tree yet");
  }

  ZJoltAABBTreeBuffer *buffer = zjolt::New<ZJoltAABBTreeBuffer>();
  if (buffer == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  const char *error = nullptr;
  const bool ok = buffer->impl.Convert(
      builder->impl.GetTriangles(), builder->impl.GetNodes(),
      builder->splitter->vertices, builder->root, store_user_data, error);
  if (!ok) {
    zjolt::Delete(buffer);
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           error != nullptr ? error : "AABBTreeToBuffer::Convert failed");
  }

  zjolt::HandleCreated();
  *out = buffer;
  return ZJOLT_RESULT_OK;
}

void zjoltAABBTreeBufferDestroy(ZJoltAABBTreeBuffer *buffer) {
  if (buffer == nullptr) return;
  zjolt::Delete(buffer);
  zjolt::HandleDestroyed();
}

void zjoltAABBTreeBufferGetData(const ZJoltAABBTreeBuffer *buffer,
                                const void **out_data, size_t *out_size) {
  if (out_data != nullptr) *out_data = nullptr;
  if (out_size != nullptr) *out_size = 0;
  if (buffer == nullptr) return;

  const JPH::ByteBuffer &bytes = buffer->impl.GetBuffer();
  if (out_data != nullptr) *out_data = bytes.data();
  if (out_size != nullptr) *out_size = bytes.size();
}

ZJoltResult zjoltAABBTreeBufferGetNodeHeader(const ZJoltAABBTreeBuffer *buffer,
                                             ZJoltAABBTreeNodeHeader *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(buffer, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  std::memcpy(out, buffer->impl.GetNodeHeader(), sizeof(*out));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltAABBTreeBufferGetTriangleHeader(
    const ZJoltAABBTreeBuffer *buffer, ZJoltAABBTreeTriangleHeader *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(buffer, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  std::memcpy(out, buffer->impl.GetTriangleHeader(), sizeof(*out));
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
