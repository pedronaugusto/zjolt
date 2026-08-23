//===----------------------------------------------------------------------===//
// zjolt — shape construction, introspection, and binary serialisation.
//
// Jolt builds shapes through *Settings objects that exist mainly so a scene
// can be serialised as data. A host driving this ABI already has its own asset
// format, so the settings objects never cross the boundary: each constructor
// takes the parameters directly, builds the settings on the stack, and hands
// back the finished shape.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

namespace {

/// Turns Jolt's ShapeResult into the ABI's convention: a reference handed to
/// the caller on success, or an error whose message survives in zjoltLastError.
///
/// The extra AddRef is deliberate. Jolt's ShapeResult holds a Ref that drops
/// its reference when the result goes out of scope at the end of the calling
/// function; taking one here is what makes "every constructor returns a shape
/// with a reference count of one" true.
ZJoltResult Finish(JPH::Shape::ShapeResult &result, ZJoltShape **out) {
  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_ERR_SHAPE_INVALID, result.GetError().c_str());
  }
  if (!result.IsValid()) {
    return zjolt::SetError(ZJOLT_ERR_SHAPE_INVALID,
                           "shape construction produced no result");
  }
  const JPH::Shape *shape = result.Get();
  shape->AddRef();
  *out = const_cast<ZJoltShape *>(zjolt::ToC(shape));
  return ZJOLT_OK;
}

/// Guard shared by every constructor. `out` is cleared first so a caller that
/// ignores the result never reads an uninitialised handle.
ZJoltResult Begin(ZJoltShape **out) {
  zjolt::ClearError();
  if (out == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (!zjolt::IsInitialized()) return ZJOLT_ERR_NOT_INITIALIZED;
  return ZJOLT_OK;
}

ZJoltShapeSubType ToCSubType(JPH::EShapeSubType sub_type) {
  switch (sub_type) {
    case JPH::EShapeSubType::Sphere:
      return ZJOLT_SHAPE_SUB_TYPE_SPHERE;
    case JPH::EShapeSubType::Box:
      return ZJOLT_SHAPE_SUB_TYPE_BOX;
    case JPH::EShapeSubType::Capsule:
      return ZJOLT_SHAPE_SUB_TYPE_CAPSULE;
    case JPH::EShapeSubType::ConvexHull:
      return ZJOLT_SHAPE_SUB_TYPE_CONVEX_HULL;
    case JPH::EShapeSubType::Mesh:
      return ZJOLT_SHAPE_SUB_TYPE_MESH;
    case JPH::EShapeSubType::Scaled:
      return ZJOLT_SHAPE_SUB_TYPE_SCALED;
    case JPH::EShapeSubType::RotatedTranslated:
      return ZJOLT_SHAPE_SUB_TYPE_ROTATED_TRANSLATED;
    case JPH::EShapeSubType::OffsetCenterOfMass:
      return ZJOLT_SHAPE_SUB_TYPE_OFFSET_CENTER_OF_MASS;
    default:
      // Jolt has more shape kinds than this ABI constructs. One can still
      // arrive here — through a body created elsewhere, or a restore — and
      // reporting OTHER is more useful than pretending it is a box.
      return ZJOLT_SHAPE_SUB_TYPE_OTHER;
  }
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Convex primitives
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreateBox(const ZJoltVec3 *half_extent,
                                float convex_radius, float density,
                                ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;
  if (half_extent == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;

  JPH::BoxShapeSettings settings(zjolt::ToJolt(*half_extent), convex_radius);
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateSphere(float radius, float density,
                                   ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;

  JPH::SphereShapeSettings settings(radius);
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateCapsule(float half_height_of_cylinder, float radius,
                                    float density, ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;

  JPH::CapsuleShapeSettings settings(half_height_of_cylinder, radius);
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateConvexHull(const ZJoltVec3 *points,
                                       uint32_t num_points,
                                       float max_convex_radius,
                                       float hull_tolerance, float density,
                                       ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;
  if (points == nullptr || num_points == 0) {
    return zjolt::SetError(ZJOLT_ERR_INVALID_ARGUMENT,
                           "a convex hull needs at least one point");
  }

  JPH::Array<JPH::Vec3> hull_points;
  hull_points.reserve(num_points);
  for (uint32_t i = 0; i < num_points; ++i)
    hull_points.push_back(zjolt::ToJolt(points[i]));

  JPH::ConvexHullShapeSettings settings(hull_points, max_convex_radius);
  if (hull_tolerance > 0.0f) settings.mHullTolerance = hull_tolerance;
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

//===----------------------------------------------------------------------===//
// Mesh
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreateMesh(const ZJoltVec3 *vertices,
                                 uint32_t num_vertices,
                                 const uint32_t *indices,
                                 uint32_t num_triangles,
                                 uint32_t max_triangles_per_leaf,
                                 ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;
  if (vertices == nullptr || indices == nullptr || num_vertices == 0 ||
      num_triangles == 0) {
    return zjolt::SetError(ZJOLT_ERR_INVALID_ARGUMENT,
                           "a mesh needs at least one vertex and one triangle");
  }

  // Checked here rather than left to Jolt: an out-of-range index would
  // otherwise be read as a vertex from beyond the caller's array, inside the
  // tree builder, with nothing to attribute the crash to.
  for (uint32_t i = 0; i < num_triangles * 3; ++i) {
    if (indices[i] >= num_vertices) {
      return zjolt::SetError(ZJOLT_ERR_INVALID_ARGUMENT,
                             "a triangle index is out of range for the vertex "
                             "array");
    }
  }

  JPH::VertexList vertex_list;
  vertex_list.reserve(num_vertices);
  for (uint32_t i = 0; i < num_vertices; ++i)
    vertex_list.push_back(JPH::Float3(vertices[i].x, vertices[i].y, vertices[i].z));

  JPH::IndexedTriangleList triangle_list;
  triangle_list.reserve(num_triangles);
  for (uint32_t i = 0; i < num_triangles; ++i) {
    triangle_list.push_back(JPH::IndexedTriangle(
        indices[i * 3 + 0], indices[i * 3 + 1], indices[i * 3 + 2]));
  }

  JPH::MeshShapeSettings settings(std::move(vertex_list),
                                  std::move(triangle_list));
  if (max_triangles_per_leaf > 0)
    settings.mMaxTrianglesPerLeaf = max_triangles_per_leaf;
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

//===----------------------------------------------------------------------===//
// Decorated shapes
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreateScaled(const ZJoltShape *inner,
                                   const ZJoltVec3 *scale, ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;
  if (inner == nullptr || scale == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;

  JPH::ScaledShapeSettings settings(zjolt::ToJolt(inner),
                                    zjolt::ToJolt(*scale));
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateRotatedTranslated(const ZJoltShape *inner,
                                              const ZJoltVec3 *translation,
                                              const ZJoltQuat *rotation,
                                              ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;
  if (inner == nullptr || translation == nullptr || rotation == nullptr)
    return ZJOLT_ERR_INVALID_ARGUMENT;

  JPH::RotatedTranslatedShapeSettings settings(
      zjolt::ToJolt(*translation), zjolt::ToJoltRotation(*rotation),
      zjolt::ToJolt(inner));
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateOffsetCenterOfMass(const ZJoltShape *inner,
                                               const ZJoltVec3 *offset,
                                               ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;
  if (inner == nullptr || offset == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;

  JPH::OffsetCenterOfMassShapeSettings settings(zjolt::ToJolt(*offset),
                                                zjolt::ToJolt(inner));
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

//===----------------------------------------------------------------------===//
// Lifetime and introspection
//===----------------------------------------------------------------------===//

void zjoltShapeAddRef(const ZJoltShape *shape) {
  if (shape == nullptr) return;
  zjolt::ToJolt(shape)->AddRef();
}

void zjoltShapeRelease(const ZJoltShape *shape) {
  if (shape == nullptr) return;
  zjolt::ToJolt(shape)->Release();
}

uint32_t zjoltShapeGetRefCount(const ZJoltShape *shape) {
  if (shape == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(shape)->GetRefCount());
}

ZJoltShapeSubType zjoltShapeGetSubType(const ZJoltShape *shape) {
  if (shape == nullptr) return ZJOLT_SHAPE_SUB_TYPE_OTHER;
  return ToCSubType(zjolt::ToJolt(shape)->GetSubType());
}

float zjoltShapeGetVolume(const ZJoltShape *shape) {
  if (shape == nullptr) return 0.0f;
  return zjolt::ToJolt(shape)->GetVolume();
}

void zjoltShapeGetCenterOfMass(const ZJoltShape *shape, ZJoltVec3 *out) {
  if (out == nullptr) return;
  if (shape == nullptr) {
    *out = ZJoltVec3{0.0f, 0.0f, 0.0f};
    return;
  }
  *out = zjolt::ToC(zjolt::ToJolt(shape)->GetCenterOfMass());
}

void zjoltShapeGetLocalBounds(const ZJoltShape *shape, ZJoltAABox *out) {
  if (out == nullptr) return;
  if (shape == nullptr) {
    *out = ZJoltAABox{};
    return;
  }
  const JPH::AABox bounds = zjolt::ToJolt(shape)->GetLocalBounds();
  out->min = zjolt::ToC(bounds.mMin);
  out->max = zjolt::ToC(bounds.mMax);
}

void zjoltShapeGetMassProperties(const ZJoltShape *shape,
                                 ZJoltMassProperties *out) {
  if (out == nullptr) return;
  if (shape == nullptr) {
    *out = ZJoltMassProperties{};
    return;
  }
  const JPH::MassProperties properties =
      zjolt::ToJolt(shape)->GetMassProperties();
  out->mass = properties.mMass;
  // Jolt stores inertia in the upper-left 3x3 of a Mat44. Column c, row r of
  // that matrix becomes row-major element [r][c] here.
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out->inertia[row * 3 + col] =
          properties.mInertia.GetColumn4(col)[row];
    }
  }
}

void zjoltShapeGetStats(const ZJoltShape *shape, ZJoltShapeStats *out) {
  if (out == nullptr) return;
  if (shape == nullptr) {
    *out = ZJoltShapeStats{};
    return;
  }
  JPH::Shape::VisitedShapes visited;
  const JPH::Shape::Stats stats =
      zjolt::ToJolt(shape)->GetStatsRecursive(visited);
  out->size_bytes = static_cast<uint64_t>(stats.mSizeBytes);
  out->num_triangles = static_cast<uint32_t>(stats.mNumTriangles);
}

//===----------------------------------------------------------------------===//
// Serialisation
//
// SaveWithChildren / sRestoreWithChildren rather than the plain binary state,
// because a decorated or compound shape is a graph: the plain form saves one
// node and leaves the caller to re-supply the children in the original order,
// which is not something a flat C ABI can promise.
//
// Jolt's payload is wrapped in a small container, and that is load-bearing
// rather than decorative. Jolt validates its own reads — it tests IsEOF after
// every stage — but only AFTER it has already used the first field it read.
// `Shape::sRestoreFromBinaryState` reads an EShapeSubType and passes it
// straight to `ShapeFunctions::sGet`, which indexes a fixed table; a byte
// outside the enum range is an out-of-bounds index, and in a build with
// asserts it is an abort before the EOF check is ever reached. Reproduced
// here by handing `zjoltShapeRestore` a buffer of ordinary text.
//
// So the container answers a question Jolt cannot: is this buffer a shape at
// all? A magic tag rejects the wrong file entirely; the recorded length
// rejects truncation and trailing bytes before Jolt sees them; the build
// stamp rejects a cache written by a different Jolt or a different precision
// setting, which would otherwise deserialise into a plausible-looking wrong
// shape; and the checksum rejects a payload damaged in storage.
//
// What it does NOT claim: this is not a defence against a deliberately
// crafted payload that carries a matching checksum. Treat a shape cache as
// something your own cook wrote, not as untrusted input.
//===----------------------------------------------------------------------===//

namespace {

constexpr uint8_t kMagic[4] = {'Z', 'J', 'S', 'H'};
constexpr uint32_t kFormatVersion = 1;
constexpr size_t kHeaderSize = 32;

/// CRC-32 (the usual reflected polynomial), computed without a table. A shape
/// is saved once and loaded once; the table is not worth the cache line.
uint32_t Crc32(const uint8_t *data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xedb88320u & mask);
    }
  }
  return ~crc;
}

void WriteU32(uint8_t *out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t ReadU32(const uint8_t *in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

void WriteU64(uint8_t *out, uint64_t value) {
  WriteU32(out, static_cast<uint32_t>(value));
  WriteU32(out + 4, static_cast<uint32_t>(value >> 32));
}

uint64_t ReadU64(const uint8_t *in) {
  return static_cast<uint64_t>(ReadU32(in)) |
         (static_cast<uint64_t>(ReadU32(in + 4)) << 32);
}

uint32_t JoltVersionStamp() {
  return (static_cast<uint32_t>(JPH_VERSION_MAJOR) << 16) |
         (static_cast<uint32_t>(JPH_VERSION_MINOR) << 8) |
         static_cast<uint32_t>(JPH_VERSION_PATCH);
}

}  // namespace

ZJoltResult zjoltShapeSave(const ZJoltShape *shape, void *buffer,
                           size_t capacity, size_t *out_size) {
  zjolt::ClearError();
  if (shape == nullptr || out_size == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;
  if (!zjolt::IsInitialized()) return ZJOLT_ERR_NOT_INITIALIZED;

  uint8_t *bytes = static_cast<uint8_t *>(buffer);

  // A buffer that cannot even hold the header is counted, not written into.
  // The distinction is not cosmetic: `bytes + kHeaderSize` would otherwise be
  // a pointer past the end of the caller's allocation, which is undefined
  // before anything is ever stored through it.
  const bool count_only = bytes == nullptr || capacity < kHeaderSize;

  // Counting and writing are the same traversal, so the size a query reports
  // and the size a write consumes cannot drift apart.
  zjolt::CountingStreamOut stream(count_only ? nullptr : bytes + kHeaderSize,
                                  count_only ? 0 : capacity - kHeaderSize);
  JPH::Shape::ShapeToIDMap shape_map;
  JPH::Shape::MaterialToIDMap material_map;
  zjolt::ToJolt(shape)->SaveWithChildren(stream, shape_map, material_map);

  const size_t payload_size = stream.Size();
  *out_size = kHeaderSize + payload_size;
  if (bytes == nullptr) return ZJOLT_OK;
  if (count_only || capacity < *out_size || stream.IsFailed())
    return ZJOLT_ERR_BUFFER_TOO_SMALL;

  std::memcpy(bytes, kMagic, sizeof(kMagic));
  WriteU32(bytes + 4, kFormatVersion);
  WriteU32(bytes + 8, static_cast<uint32_t>(ZJOLT_CONFIG_ID));
  WriteU32(bytes + 12, JoltVersionStamp());
  WriteU64(bytes + 16, static_cast<uint64_t>(payload_size));
  WriteU32(bytes + 24, Crc32(bytes + kHeaderSize, payload_size));
  WriteU32(bytes + 28, 0);
  return ZJOLT_OK;
}

ZJoltResult zjoltShapeRestore(const void *data, size_t size,
                              ZJoltShape **out) {
  const ZJoltResult ready = Begin(out);
  if (ready != ZJOLT_OK) return ready;
  if (data == nullptr || size == 0) {
    return zjolt::SetError(ZJOLT_ERR_INVALID_ARGUMENT,
                           "no data to restore a shape from");
  }

  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  if (size < kHeaderSize) {
    return zjolt::SetError(ZJOLT_ERR_BAD_FORMAT,
                           "too short to be a saved shape");
  }
  if (std::memcmp(bytes, kMagic, sizeof(kMagic)) != 0) {
    return zjolt::SetError(ZJOLT_ERR_BAD_FORMAT,
                           "not a shape saved by zjoltShapeSave");
  }
  if (ReadU32(bytes + 4) != kFormatVersion) {
    return zjolt::SetError(ZJOLT_ERR_BAD_FORMAT,
                           "saved by a different zjolt container version");
  }
  if (ReadU32(bytes + 8) != static_cast<uint32_t>(ZJOLT_CONFIG_ID)) {
    return zjolt::SetError(
        ZJOLT_ERR_BAD_FORMAT,
        "saved by a zjolt built with different layout-affecting settings");
  }
  if (ReadU32(bytes + 12) != JoltVersionStamp()) {
    return zjolt::SetError(ZJOLT_ERR_BAD_FORMAT,
                           "saved against a different Jolt version");
  }

  const uint64_t payload_size = ReadU64(bytes + 16);
  if (payload_size != static_cast<uint64_t>(size - kHeaderSize)) {
    return zjolt::SetError(
        ZJOLT_ERR_BAD_FORMAT,
        "the recorded payload length does not match the buffer");
  }
  if (ReadU32(bytes + 24) !=
      Crc32(bytes + kHeaderSize, static_cast<size_t>(payload_size))) {
    return zjolt::SetError(ZJOLT_ERR_BAD_FORMAT,
                           "the shape payload failed its checksum");
  }

  zjolt::ConstStreamIn stream(bytes + kHeaderSize,
                              static_cast<size_t>(payload_size));
  JPH::Shape::IDToShapeMap shape_map;
  JPH::Shape::IDToMaterialMap material_map;
  JPH::Shape::ShapeResult result =
      JPH::Shape::sRestoreWithChildren(stream, shape_map, material_map);

  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_ERR_BAD_FORMAT, result.GetError().c_str());
  }
  if (!result.IsValid() || stream.IsEOF()) {
    return zjolt::SetError(ZJOLT_ERR_BAD_FORMAT,
                           "the shape data ended before the shape did");
  }
  if (!stream.ConsumedAll()) {
    return zjolt::SetError(ZJOLT_ERR_BAD_FORMAT,
                           "trailing bytes after the shape");
  }
  return Finish(result, out);
}

}  // extern "C"
