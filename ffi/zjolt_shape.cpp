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
#include "zjolt_transformed.h"

#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>

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
    return zjolt::SetError(ZJOLT_RESULT_SHAPE_INVALID, result.GetError().c_str());
  }
  if (!result.IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_SHAPE_INVALID,
                           "shape construction produced no result");
  }
  const JPH::Shape *shape = result.Get();
  shape->AddRef();
  *out = const_cast<ZJoltShape *>(zjolt::ToC(shape));
  return ZJOLT_RESULT_OK;
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
    case JPH::EShapeSubType::Triangle:
      return ZJOLT_SHAPE_SUB_TYPE_TRIANGLE;
    case JPH::EShapeSubType::Cylinder:
      return ZJOLT_SHAPE_SUB_TYPE_CYLINDER;
    case JPH::EShapeSubType::TaperedCapsule:
      return ZJOLT_SHAPE_SUB_TYPE_TAPERED_CAPSULE;
    case JPH::EShapeSubType::TaperedCylinder:
      return ZJOLT_SHAPE_SUB_TYPE_TAPERED_CYLINDER;
    case JPH::EShapeSubType::StaticCompound:
      return ZJOLT_SHAPE_SUB_TYPE_STATIC_COMPOUND;
    case JPH::EShapeSubType::MutableCompound:
      return ZJOLT_SHAPE_SUB_TYPE_MUTABLE_COMPOUND;
    case JPH::EShapeSubType::HeightField:
      return ZJOLT_SHAPE_SUB_TYPE_HEIGHT_FIELD;
    case JPH::EShapeSubType::Plane:
      return ZJOLT_SHAPE_SUB_TYPE_PLANE;
    case JPH::EShapeSubType::Empty:
      return ZJOLT_SHAPE_SUB_TYPE_EMPTY;
    case JPH::EShapeSubType::SoftBody:
      return ZJOLT_SHAPE_SUB_TYPE_SOFT_BODY;
    default:
      // Every shape kind Jolt itself defines is now named above, so this arm
      // no longer stands for "a kind we did not get round to". What is left is
      // the sixteen User1..UserConvex8 slots the enum reserves for shape types
      // registered by C++ outside this library — unconstructible from here and
      // unnameable from here, because their meaning belongs to whoever
      // registered them. That is also why the arm cannot simply go away: it is
      // what the entry point below returns for a NULL handle, and without it a
      // null shape would report SPHERE.
      return ZJOLT_SHAPE_SUB_TYPE_OTHER;
  }
}

/// The material a caller passed, or null for Jolt's default.
///
/// Written once rather than at each settings object so that "NULL means the
/// default" is a property of the boundary instead of a habit.
const JPH::PhysicsMaterial *ToJoltMaterial(
    const ZJoltPhysicsMaterial *material) {
  return material != nullptr ? zjolt::ToJolt(material) : nullptr;
}

/// Copies a caller's material table into the list Jolt's settings want.
///
/// An empty list is a distinct state from a one-entry list, and both are
/// reachable: a shape with no materials reports the shared default for every
/// leaf, a shape with one reports that one. Jolt enforces the same distinction
/// in the other direction — it refuses a triangle whose material index is
/// non-zero when the list is empty.
ZJoltResult BuildMaterialList(const ZJoltPhysicsMaterial *const *materials,
                              uint32_t num_materials,
                              JPH::PhysicsMaterialList &out) {
  if (materials == nullptr) {
    if (num_materials != 0) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "num_materials is non-zero but the material list "
                             "is null");
    }
    return ZJOLT_RESULT_OK;
  }
  out.reserve(num_materials);
  for (uint32_t i = 0; i < num_materials; ++i) {
    if (materials[i] == nullptr) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a slot in the material list is null; give every "
                             "slot a material");
    }
    out.push_back(zjolt::ToJolt(materials[i]));
  }
  return ZJOLT_RESULT_OK;
}

/// The fewest children a compound that STAYS a compound may have.
///
/// Not a style rule. `CompoundShape::GetSubShapeIDBits` is
/// `32 - CountLeadingZeros(count - 1)`, and Jolt's CountLeadingZeros guards a
/// zero argument on x86 and on several other architectures but NOT on ARM,
/// where it is a bare `__builtin_clz` — undefined behaviour for zero. So a
/// compound of one child is undefined on some targets and merely lucky on the
/// rest, and a compound of none underflows the subtraction first.
///
/// A static compound never reaches it, because Jolt simplifies one child away
/// into the child itself or a rotated-translated shape. A mutable compound
/// does not simplify, so this is where the floor has to be enforced. See
/// UPSTREAM.md.
constexpr uint32_t kMinCompoundChildren = 2;

/// Fills a compound settings object from the caller's child array.
ZJoltResult BuildCompound(const ZJoltCompoundChild *children,
                          uint32_t num_children, uint32_t min_children,
                          JPH::CompoundShapeSettings &out) {
  if (children == nullptr || num_children < min_children) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        min_children > 1
            ? "a mutable compound needs at least two children, because Jolt "
              "cannot size the sub-shape id of one that has fewer"
            : "a compound needs at least one child");
  }
  for (uint32_t i = 0; i < num_children; ++i) {
    if (children[i].shape == nullptr) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a compound child has no shape");
    }
    out.AddShape(zjolt::ToJolt(children[i].position),
                 zjolt::ToJoltRotation(children[i].rotation),
                 zjolt::ToJolt(children[i].shape), children[i].user_data);
  }
  return ZJOLT_RESULT_OK;
}

/// The shape as a compound, or null if it is any other kind.
///
/// Dispatching on GetType rather than a dynamic_cast is not a shortcut: the
/// build compiles -fno-rtti, and this is how Jolt identifies its own shapes
/// too. It is what turns "you asked a box how many children it has" into an
/// answer rather than a bad cast.
const JPH::CompoundShape *AsCompound(const ZJoltShape *shape) {
  if (shape == nullptr) return nullptr;
  const JPH::Shape *jolt = zjolt::ToJolt(shape);
  if (jolt->GetType() != JPH::EShapeType::Compound) return nullptr;
  return static_cast<const JPH::CompoundShape *>(jolt);
}

/// Opens a mutating compound entry point: the shape must be up, must be a
/// mutable compound, and `index` must name a child it actually has.
///
/// The range check is the load-bearing half. `MutableCompoundShape::
/// RemoveShape` and `ModifyShape` index `mSubShapes` with no bounds check and
/// no assertion, so an out-of-range index there is not a diagnosable abort but
/// a write past the end of a live array.
ZJoltResult OpenMutableCompound(ZJoltShape *shape, uint32_t index,
                                bool check_index,
                                JPH::MutableCompoundShape **out_compound) {
  *out_compound = nullptr;
  if (shape == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Shape *jolt = const_cast<JPH::Shape *>(zjolt::ToJolt(shape));
  if (jolt->GetSubType() != JPH::EShapeSubType::MutableCompound) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this shape is not a mutable compound");
  }

  JPH::MutableCompoundShape *compound =
      static_cast<JPH::MutableCompoundShape *>(jolt);
  if (check_index && index >= compound->GetNumSubShapes()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "no child of this compound has that index");
  }

  *out_compound = compound;
  return ZJOLT_RESULT_OK;
}

/// The shape as a mesh, or null if it is any other kind. @see AsCompound for
/// why this is a GetType/GetSubType dispatch rather than a dynamic_cast.
const JPH::MeshShape *AsMesh(const ZJoltShape *shape) {
  if (shape == nullptr) return nullptr;
  const JPH::Shape *jolt = zjolt::ToJolt(shape);
  if (jolt->GetSubType() != JPH::EShapeSubType::Mesh) return nullptr;
  return static_cast<const JPH::MeshShape *>(jolt);
}

/// The shape as a height field, or null if it is any other kind.
const JPH::HeightFieldShape *AsHeightField(const ZJoltShape *shape) {
  if (shape == nullptr) return nullptr;
  const JPH::Shape *jolt = zjolt::ToJolt(shape);
  if (jolt->GetSubType() != JPH::EShapeSubType::HeightField) return nullptr;
  return static_cast<const JPH::HeightFieldShape *>(jolt);
}

/// The center-of-mass transform Shape-level introspection takes as a
/// position/rotation pair, folded into the Mat44 Jolt's own API wants.
/// `position`/`rotation` NULL means the origin/identity — the transform a
/// shape not yet placed anywhere is queried through.
JPH::Mat44 MakeComTransform(const ZJoltVec3 *position,
                            const ZJoltQuat *rotation) {
  return JPH::Mat44::sRotationTranslation(
      rotation != nullptr ? zjolt::ToJoltRotation(*rotation)
                          : JPH::Quat::sIdentity(),
      position != nullptr ? zjolt::ToJolt(*position) : JPH::Vec3::sZero());
}

/// Matches Jolt's own Shape::GetTrianglesContext byte for byte, checked once
/// here rather than trusted: a mismatch would silently corrupt whichever
/// walk's scratch space is smaller.
static_assert(sizeof(ZJoltShapeTrianglesContext) ==
                  sizeof(JPH::Shape::GetTrianglesContext),
              "ZJoltShapeTrianglesContext must match Shape::GetTrianglesContext");
static_assert(alignof(ZJoltShapeTrianglesContext) ==
                  alignof(JPH::Shape::GetTrianglesContext),
              "ZJoltShapeTrianglesContext must match Shape::GetTrianglesContext");

JPH::Shape::GetTrianglesContext &AsJoltContext(ZJoltShapeTrianglesContext *context) {
  return *reinterpret_cast<JPH::Shape::GetTrianglesContext *>(context);
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Convex primitives
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreateBox(const ZJoltVec3 *half_extent,
                                float convex_radius, float density,
                                const ZJoltPhysicsMaterial *material,
                                ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(half_extent, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::BoxShapeSettings settings(zjolt::ToJolt(*half_extent), convex_radius,
                                 ToJoltMaterial(material));
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateSphere(float radius, float density,
                                   const ZJoltPhysicsMaterial *material,
                                   ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SphereShapeSettings settings(radius, ToJoltMaterial(material));
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateCapsule(float half_height_of_cylinder, float radius,
                                    float density,
                                    const ZJoltPhysicsMaterial *material,
                                    ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::CapsuleShapeSettings settings(half_height_of_cylinder, radius,
                                     ToJoltMaterial(material));
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateConvexHull(const ZJoltVec3 *points,
                                       uint32_t num_points,
                                       float max_convex_radius,
                                       float hull_tolerance, float density,
                                       const ZJoltPhysicsMaterial *material,
                                       ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (points == nullptr || num_points == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a convex hull needs at least one point");
  }

  JPH::Array<JPH::Vec3> hull_points;
  hull_points.reserve(num_points);
  for (uint32_t i = 0; i < num_points; ++i)
    hull_points.push_back(zjolt::ToJolt(points[i]));

  JPH::ConvexHullShapeSettings settings(hull_points, max_convex_radius,
                                        ToJoltMaterial(material));
  if (hull_tolerance > 0.0f) settings.mHullTolerance = hull_tolerance;
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateCylinder(float half_height, float radius,
                                     float convex_radius, float density,
                                     const ZJoltPhysicsMaterial *material,
                                     ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::CylinderShapeSettings settings(
      half_height, radius,
      convex_radius > 0.0f ? convex_radius : JPH::cDefaultConvexRadius,
      ToJoltMaterial(material));
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateTriangle(const ZJoltVec3 *v1, const ZJoltVec3 *v2,
                                     const ZJoltVec3 *v3, float convex_radius,
                                     float density,
                                     const ZJoltPhysicsMaterial *material,
                                     ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(v1, v2, v3, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Clamped rather than defaulted: a triangle's convex radius is 0 upstream,
  // not cDefaultConvexRadius, because the shape is meant to be thin — and
  // TriangleShape's constructor asserts the radius is not negative.
  JPH::TriangleShapeSettings settings(
      zjolt::ToJolt(*v1), zjolt::ToJolt(*v2), zjolt::ToJolt(*v3),
      convex_radius > 0.0f ? convex_radius : 0.0f, ToJoltMaterial(material));
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateTaperedCapsule(
    float half_height_of_tapered_cylinder, float top_radius,
    float bottom_radius, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::TaperedCapsuleShapeSettings settings(half_height_of_tapered_cylinder,
                                            top_radius, bottom_radius,
                                            ToJoltMaterial(material));
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateTaperedCylinder(
    float half_height, float top_radius, float bottom_radius,
    float convex_radius, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::TaperedCylinderShapeSettings settings(
      half_height, top_radius, bottom_radius,
      convex_radius > 0.0f ? convex_radius : JPH::cDefaultConvexRadius,
      ToJoltMaterial(material));
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

//===----------------------------------------------------------------------===//
// Shapes without a volume
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreatePlane(const ZJoltVec3 *normal, float constant,
                                  float half_extent,
                                  const ZJoltPhysicsMaterial *material,
                                  ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(normal, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Refused rather than renormalised, which is the opposite of what
  // ToJoltRotation does to a quaternion — and deliberately so. A body's
  // rotation drifts off unit length by integrating it for a few thousand
  // frames, so fixing it is a kindness. A plane's normal is authored, and
  // rescaling it silently moves the surface away from where `constant` says it
  // is.
  const JPH::Vec3 plane_normal = zjolt::ToJolt(*normal);
  const float length_sq = plane_normal.LengthSq();
  if (!std::isfinite(length_sq) || std::fabs(length_sq - 1.0f) > 1.0e-3f) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a plane's normal must be unit length");
  }

  JPH::PlaneShapeSettings settings(
      JPH::Plane(plane_normal, constant), ToJoltMaterial(material),
      half_extent > 0.0f ? half_extent
                         : JPH::PlaneShapeSettings::cDefaultHalfExtent);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateEmpty(const ZJoltVec3 *center_of_mass,
                                  ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::EmptyShapeSettings settings(center_of_mass != nullptr
                                       ? zjolt::ToJolt(*center_of_mass)
                                       : JPH::Vec3::sZero());
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

//===----------------------------------------------------------------------===//
// Mesh
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreateMesh(
    const ZJoltVec3 *vertices, uint32_t num_vertices, const uint32_t *indices,
    uint32_t num_triangles, const uint32_t *triangle_materials,
    const ZJoltPhysicsMaterial *const *materials, uint32_t num_materials,
    uint32_t max_triangles_per_leaf, ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (vertices == nullptr || indices == nullptr || num_vertices == 0 ||
      num_triangles == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a mesh needs at least one vertex and one triangle");
  }

  // Checked here rather than left to Jolt: an out-of-range index would
  // otherwise be read as a vertex from beyond the caller's array, inside the
  // tree builder, with nothing to attribute the crash to.
  for (uint32_t i = 0; i < num_triangles * 3; ++i) {
    if (indices[i] >= num_vertices) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a triangle index is out of range for the vertex "
                             "array");
    }
  }

  JPH::PhysicsMaterialList material_list;
  const ZJoltResult materials_ok =
      BuildMaterialList(materials, num_materials, material_list);
  if (materials_ok != ZJOLT_RESULT_OK) return materials_ok;

  if (triangle_materials != nullptr) {
    for (uint32_t i = 0; i < num_triangles; ++i) {
      if (triangle_materials[i] >= num_materials) {
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "a triangle's material index is out of range "
                               "for the material list");
      }
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
        indices[i * 3 + 0], indices[i * 3 + 1], indices[i * 3 + 2],
        triangle_materials != nullptr ? triangle_materials[i] : 0u));
  }

  JPH::MeshShapeSettings settings(std::move(vertex_list),
                                  std::move(triangle_list),
                                  std::move(material_list));
  if (max_triangles_per_leaf > 0)
    settings.mMaxTrianglesPerLeaf = max_triangles_per_leaf;
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

//===----------------------------------------------------------------------===//
// Height field
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreateHeightField(
    const float *samples, uint32_t sample_count, const ZJoltVec3 *offset,
    const ZJoltVec3 *scale, const uint8_t *material_indices,
    const ZJoltPhysicsMaterial *const *materials, uint32_t num_materials,
    uint32_t block_size, uint32_t bits_per_sample, ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(samples, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Guarded before anything computes with it. The quad count below is
  // (sample_count - 1)^2, and Jolt's own settings do the same arithmetic — at
  // zero that underflows to a four-billion-element span rather than to an
  // error anybody could read.
  if (sample_count == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a height field needs at least one sample");
  }

  JPH::HeightFieldShapeSettings settings;
  settings.mOffset =
      offset != nullptr ? zjolt::ToJolt(*offset) : JPH::Vec3::sZero();
  settings.mScale = scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
  settings.mSampleCount = sample_count;
  if (block_size > 0) settings.mBlockSize = block_size;
  if (bits_per_sample > 0) settings.mBitsPerSample = bits_per_sample;

  settings.mHeightSamples.assign(
      samples, samples + static_cast<size_t>(sample_count) * sample_count);

  const ZJoltResult materials_ok =
      BuildMaterialList(materials, num_materials, settings.mMaterials);
  if (materials_ok != ZJOLT_RESULT_OK) return materials_ok;

  if (material_indices != nullptr) {
    const size_t quad_count =
        static_cast<size_t>(sample_count - 1) * (sample_count - 1);
    for (size_t i = 0; i < quad_count; ++i) {
      if (material_indices[i] >= num_materials) {
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "a quad's material index is out of range for "
                               "the material list");
      }
    }
    settings.mMaterialIndices.assign(material_indices,
                                     material_indices + quad_count);
  }

  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

//===----------------------------------------------------------------------===//
// Compounds
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreateStaticCompound(const ZJoltCompoundChild *children,
                                           uint32_t num_children,
                                           ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::StaticCompoundShapeSettings settings;
  const ZJoltResult children_ok =
      BuildCompound(children, num_children, 1, settings);
  if (children_ok != ZJOLT_RESULT_OK) return children_ok;

  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateMutableCompound(const ZJoltCompoundChild *children,
                                            uint32_t num_children,
                                            ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::MutableCompoundShapeSettings settings;
  const ZJoltResult children_ok =
      BuildCompound(children, num_children, kMinCompoundChildren, settings);
  if (children_ok != ZJOLT_RESULT_OK) return children_ok;

  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

uint32_t zjoltShapeCompoundGetNumChildren(const ZJoltShape *shape) {
  const JPH::CompoundShape *compound = AsCompound(shape);
  if (compound == nullptr) return 0;
  return static_cast<uint32_t>(compound->GetNumSubShapes());
}

uint32_t zjoltShapeCompoundGetChildUserData(const ZJoltShape *shape,
                                            uint32_t index) {
  const JPH::CompoundShape *compound = AsCompound(shape);
  if (compound == nullptr) return 0;
  if (index >= compound->GetNumSubShapes()) return 0;
  return compound->GetCompoundUserData(index);
}

ZJoltResult zjoltShapeMutableCompoundAddChild(ZJoltShape *shape,
                                              const ZJoltCompoundChild *child,
                                              uint32_t *out_index) {
  ZJOLT_ENTER(out_index);
  if (!zjolt::Present(shape, child, out_index))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (child->shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a compound child has no shape");
  }

  JPH::MutableCompoundShape *compound = nullptr;
  const ZJoltResult ready = OpenMutableCompound(shape, 0, false, &compound);
  if (ready != ZJOLT_RESULT_OK) return ready;

  *out_index = static_cast<uint32_t>(compound->AddShape(
      zjolt::ToJolt(child->position), zjolt::ToJoltRotation(child->rotation),
      zjolt::ToJolt(child->shape), child->user_data));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeMutableCompoundRemoveChild(ZJoltShape *shape,
                                                 uint32_t index) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::MutableCompoundShape *compound = nullptr;
  const ZJoltResult ready = OpenMutableCompound(shape, index, true, &compound);
  if (ready != ZJOLT_RESULT_OK) return ready;

  if (compound->GetNumSubShapes() <= kMinCompoundChildren) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a mutable compound cannot be taken below two children, because Jolt "
        "cannot size the sub-shape id of one that has fewer");
  }

  compound->RemoveShape(index);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeMutableCompoundMoveChild(ZJoltShape *shape,
                                               uint32_t index,
                                               const ZJoltVec3 *position,
                                               const ZJoltQuat *rotation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, position)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::MutableCompoundShape *compound = nullptr;
  const ZJoltResult ready = OpenMutableCompound(shape, index, true, &compound);
  if (ready != ZJOLT_RESULT_OK) return ready;

  compound->ModifyShape(index, zjolt::ToJolt(*position),
                        rotation != nullptr ? zjolt::ToJoltRotation(*rotation)
                                            : JPH::Quat::sIdentity());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeMutableCompoundAdjustCenterOfMass(ZJoltShape *shape) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::MutableCompoundShape *compound = nullptr;
  const ZJoltResult ready = OpenMutableCompound(shape, 0, false, &compound);
  if (ready != ZJOLT_RESULT_OK) return ready;

  compound->AdjustCenterOfMass();
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Decorated shapes
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCreateScaled(const ZJoltShape *inner,
                                   const ZJoltVec3 *scale, ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(inner, scale, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::ScaledShapeSettings settings(zjolt::ToJolt(inner),
                                    zjolt::ToJolt(*scale));
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateRotatedTranslated(const ZJoltShape *inner,
                                              const ZJoltVec3 *translation,
                                              const ZJoltQuat *rotation,
                                              ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(inner, translation, rotation, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::RotatedTranslatedShapeSettings settings(
      zjolt::ToJolt(*translation), zjolt::ToJoltRotation(*rotation),
      zjolt::ToJolt(inner));
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}

ZJoltResult zjoltShapeCreateOffsetCenterOfMass(const ZJoltShape *inner,
                                               const ZJoltVec3 *offset,
                                               ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(inner, offset, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

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

const ZJoltPhysicsMaterial *zjoltShapeGetMaterial(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id) {
  if (shape == nullptr) return nullptr;
  return zjolt::ToC(
      zjolt::ToJolt(shape)->GetMaterial(zjolt::ToJoltSubShapeId(sub_shape_id)));
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
  ZJOLT_ENTER(out_size);
  if (!zjolt::Present(shape, out_size)) return ZJOLT_RESULT_INVALID_ARGUMENT;

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
  if (bytes == nullptr) return ZJOLT_RESULT_OK;
  if (count_only || capacity < *out_size || stream.IsFailed())
    return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  std::memcpy(bytes, kMagic, sizeof(kMagic));
  WriteU32(bytes + 4, kFormatVersion);
  WriteU32(bytes + 8, static_cast<uint32_t>(ZJOLT_CONFIG_ID));
  WriteU32(bytes + 12, JoltVersionStamp());
  WriteU64(bytes + 16, static_cast<uint64_t>(payload_size));
  WriteU32(bytes + 24, Crc32(bytes + kHeaderSize, payload_size));
  WriteU32(bytes + 28, 0);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeRestore(const void *data, size_t size,
                              ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (data == nullptr || size == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "no data to restore a shape from");
  }

  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  if (size < kHeaderSize) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "too short to be a saved shape");
  }
  if (std::memcmp(bytes, kMagic, sizeof(kMagic)) != 0) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "not a shape saved by zjoltShapeSave");
  }
  if (ReadU32(bytes + 4) != kFormatVersion) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "saved by a different zjolt container version");
  }
  if (ReadU32(bytes + 8) != static_cast<uint32_t>(ZJOLT_CONFIG_ID)) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "saved by a zjolt built with different layout-affecting settings");
  }
  if (ReadU32(bytes + 12) != JoltVersionStamp()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "saved against a different Jolt version");
  }

  const uint64_t payload_size = ReadU64(bytes + 16);
  if (payload_size != static_cast<uint64_t>(size - kHeaderSize)) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "the recorded payload length does not match the buffer");
  }
  if (ReadU32(bytes + 24) !=
      Crc32(bytes + kHeaderSize, static_cast<size_t>(payload_size))) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the shape payload failed its checksum");
  }

  zjolt::ConstStreamIn stream(bytes + kHeaderSize,
                              static_cast<size_t>(payload_size));
  JPH::Shape::IDToShapeMap shape_map;
  JPH::Shape::IDToMaterialMap material_map;
  JPH::Shape::ShapeResult result =
      JPH::Shape::sRestoreWithChildren(stream, shape_map, material_map);

  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT, result.GetError().c_str());
  }
  if (!result.IsValid() || stream.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the shape data ended before the shape did");
  }
  if (!stream.ConsumedAll()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "trailing bytes after the shape");
  }
  return Finish(result, out);
}

//===----------------------------------------------------------------------===//
// Introspection Jolt puts on every leaf shape
//===----------------------------------------------------------------------===//

uint32_t zjoltShapeGetSubShapeIDBits(const ZJoltShape *shape) {
  if (shape == nullptr) return 0;
  return zjolt::ToJolt(shape)->GetSubShapeIDBitsRecursive();
}

void zjoltShapeGetSurfaceNormal(const ZJoltShape *shape,
                                ZJoltSubShapeId sub_shape_id,
                                const ZJoltVec3 *local_surface_position,
                                ZJoltVec3 *out_normal) {
  if (out_normal == nullptr) return;
  if (shape == nullptr || local_surface_position == nullptr) {
    *out_normal = ZJoltVec3{0.0f, 0.0f, 0.0f};
    return;
  }
  *out_normal = zjolt::ToC(zjolt::ToJolt(shape)->GetSurfaceNormal(
      zjolt::ToJoltSubShapeId(sub_shape_id),
      zjolt::ToJolt(*local_surface_position)));
}

ZJoltResult zjoltShapeGetSupportingFace(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id,
    const ZJoltVec3 *direction, const ZJoltVec3 *scale,
    const ZJoltVec3 *position, const ZJoltQuat *rotation,
    ZJoltVec3 *out_vertices, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape, direction, position, rotation, out_vertices,
                      out_count)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  JPH::Shape::SupportingFace face;
  zjolt::ToJolt(shape)->GetSupportingFace(
      zjolt::ToJoltSubShapeId(sub_shape_id), zjolt::ToJolt(*direction),
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne(),
      MakeComTransform(position, rotation), face);

  for (JPH::uint i = 0; i < face.size(); ++i)
    out_vertices[i] = zjolt::ToC(face[i]);
  *out_count = static_cast<uint32_t>(face.size());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetSubShapeTransformedShape(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id,
    const ZJoltVec3 *position, const ZJoltQuat *rotation,
    const ZJoltVec3 *scale, ZJoltTransformedShape **out,
    ZJoltSubShapeId *out_remainder) {
  ZJOLT_ENTER(out, out_remainder);
  if (!zjolt::Present(shape, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Vec3 jolt_position =
      position != nullptr ? zjolt::ToJolt(*position) : JPH::Vec3::sZero();
  const JPH::Quat jolt_rotation =
      rotation != nullptr ? zjolt::ToJoltRotation(*rotation)
                          : JPH::Quat::sIdentity();
  const JPH::Vec3 jolt_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  JPH::SubShapeID remainder;
  const JPH::TransformedShape child =
      zjolt::ToJolt(shape)->GetSubShapeTransformedShape(
          zjolt::ToJoltSubShapeId(sub_shape_id), jolt_position, jolt_rotation,
          jolt_scale, remainder);
  if (out_remainder != nullptr) *out_remainder = zjolt::ToC(remainder);

  // Built from the child's own public fields rather than by reaching into
  // JPH::TransformedShape's insides: zjoltTransformedShapeCreate is the one
  // place a ZJoltTransformedShape is constructed, and going through it here
  // keeps that true instead of adding a second path with its own chance to
  // drift. The body id is always invalid, matching what
  // Shape::GetSubShapeTransformedShape itself documents: this relates two
  // shapes, not a shape and a body.
  const ZJoltRVec3 child_position = zjolt::ToCR(child.mShapePositionCOM);
  const ZJoltQuat child_rotation = zjolt::ToC(child.mShapeRotation);
  const ZJoltVec3 child_scale = zjolt::ToC(child.GetShapeScale());
  return zjoltTransformedShapeCreate(zjolt::ToC(child.mShape.GetPtr()),
                                     &child_position, &child_rotation,
                                     &child_scale, ZJOLT_BODY_ID_INVALID, out);
}

ZJoltResult zjoltShapeScaleShape(const ZJoltShape *shape,
                                 const ZJoltVec3 *scale, ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape, scale, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Shape::ShapeResult result =
      zjolt::ToJolt(shape)->ScaleShape(zjolt::ToJolt(*scale));
  return Finish(result, out);
}

bool zjoltShapeIsValidScale(const ZJoltShape *shape, const ZJoltVec3 *scale) {
  if (shape == nullptr || scale == nullptr) return false;
  return zjolt::ToJolt(shape)->IsValidScale(zjolt::ToJolt(*scale));
}

void zjoltShapeMakeScaleValid(const ZJoltShape *shape, const ZJoltVec3 *scale,
                              ZJoltVec3 *out_scale) {
  if (out_scale == nullptr) return;
  if (scale == nullptr) {
    *out_scale = ZJoltVec3{0.0f, 0.0f, 0.0f};
    return;
  }
  if (shape == nullptr) {
    *out_scale = *scale;
    return;
  }
  *out_scale = zjolt::ToC(
      zjolt::ToJolt(shape)->MakeScaleValid(zjolt::ToJolt(*scale)));
}

//===----------------------------------------------------------------------===//
// Triangle read-back
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeGetTrianglesStart(const ZJoltShape *shape,
                                        ZJoltShapeTrianglesContext *context,
                                        const ZJoltAABox *box,
                                        const ZJoltVec3 *position,
                                        const ZJoltQuat *rotation,
                                        const ZJoltVec3 *scale) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, context, box)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::AABox jolt_box(zjolt::ToJolt(box->min), zjolt::ToJolt(box->max));
  zjolt::ToJolt(shape)->GetTrianglesStart(
      AsJoltContext(context), jolt_box,
      position != nullptr ? zjolt::ToJolt(*position) : JPH::Vec3::sZero(),
      rotation != nullptr ? zjolt::ToJoltRotation(*rotation)
                          : JPH::Quat::sIdentity(),
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetTrianglesNext(
    const ZJoltShape *shape, ZJoltShapeTrianglesContext *context,
    uint32_t max_triangles, ZJoltVec3 *out_vertices,
    const ZJoltPhysicsMaterial **out_materials, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape, context, out_vertices, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (max_triangles < ZJOLT_SHAPE_MIN_TRIANGLES_REQUESTED) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "max_triangles is below ZJOLT_SHAPE_MIN_TRIANGLES_REQUESTED; Jolt "
        "asserts on a smaller request rather than honouring it");
  }

  static_assert(sizeof(JPH::Float3) == sizeof(ZJoltVec3) &&
                    alignof(JPH::Float3) <= alignof(ZJoltVec3),
                "Float3 must lay out the same as ZJoltVec3 for this cast");
  JPH::Float3 *vertices = reinterpret_cast<JPH::Float3 *>(out_vertices);

  const JPH::PhysicsMaterial **materials = nullptr;
  JPH::Array<const JPH::PhysicsMaterial *> material_storage;
  if (out_materials != nullptr) {
    material_storage.resize(max_triangles);
    materials = material_storage.data();
  }

  const int found = zjolt::ToJolt(shape)->GetTrianglesNext(
      AsJoltContext(context), static_cast<int>(max_triangles), vertices,
      materials);
  *out_count = static_cast<uint32_t>(found);

  if (out_materials != nullptr) {
    for (int i = 0; i < found; ++i)
      out_materials[i] = zjolt::ToC(material_storage[static_cast<size_t>(i)]);
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetMaterialList(const ZJoltShape *shape,
                                      const ZJoltPhysicsMaterial **out_materials,
                                      uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::PhysicsMaterialList *list = nullptr;
  if (const JPH::MeshShape *mesh = AsMesh(shape)) {
    list = &mesh->GetMaterialList();
  } else if (const JPH::HeightFieldShape *hf = AsHeightField(shape)) {
    list = &hf->GetMaterialList();
  } else {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "only a mesh or a height field has a material list; ask a compound's "
        "leaves individually");
  }

  *out_count = static_cast<uint32_t>(list->size());
  if (out_materials == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < list->size()) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (size_t i = 0; i < list->size(); ++i)
    out_materials[i] = zjolt::ToC((*list)[i].GetPtr());
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Mesh specifics
//===----------------------------------------------------------------------===//

uint32_t zjoltShapeMeshGetMaterialIndex(const ZJoltShape *shape,
                                        ZJoltSubShapeId sub_shape_id) {
  const JPH::MeshShape *mesh = AsMesh(shape);
  if (mesh == nullptr) return 0;
  return mesh->GetMaterialIndex(zjolt::ToJoltSubShapeId(sub_shape_id));
}

uint32_t zjoltShapeMeshGetTriangleUserData(const ZJoltShape *shape,
                                           ZJoltSubShapeId sub_shape_id) {
  const JPH::MeshShape *mesh = AsMesh(shape);
  if (mesh == nullptr) return 0;
  return mesh->GetTriangleUserData(zjolt::ToJoltSubShapeId(sub_shape_id));
}

//===----------------------------------------------------------------------===//
// Height field specifics
//===----------------------------------------------------------------------===//

uint32_t zjoltShapeHeightFieldGetSampleCount(const ZJoltShape *shape) {
  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  return hf != nullptr ? hf->GetSampleCount() : 0;
}

uint32_t zjoltShapeHeightFieldGetBlockSize(const ZJoltShape *shape) {
  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  return hf != nullptr ? hf->GetBlockSize() : 0;
}

float zjoltShapeHeightFieldGetMinHeightValue(const ZJoltShape *shape) {
  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  return hf != nullptr ? hf->GetMinHeightValue() : 0.0f;
}

float zjoltShapeHeightFieldGetMaxHeightValue(const ZJoltShape *shape) {
  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  return hf != nullptr ? hf->GetMaxHeightValue() : 0.0f;
}

void zjoltShapeHeightFieldGetPosition(const ZJoltShape *shape, uint32_t x,
                                      uint32_t y, ZJoltVec3 *out_position) {
  if (out_position == nullptr) return;
  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  if (hf == nullptr || x >= hf->GetSampleCount() || y >= hf->GetSampleCount()) {
    *out_position = ZJoltVec3{0.0f, 0.0f, 0.0f};
    return;
  }
  *out_position = zjolt::ToC(hf->GetPosition(x, y));
}

bool zjoltShapeHeightFieldIsNoCollision(const ZJoltShape *shape, uint32_t x,
                                        uint32_t y) {
  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  if (hf == nullptr || x >= hf->GetSampleCount() || y >= hf->GetSampleCount())
    return true;
  return hf->IsNoCollision(x, y);
}

ZJoltResult zjoltShapeHeightFieldProjectOntoSurface(
    const ZJoltShape *shape, const ZJoltVec3 *local_position,
    ZJoltVec3 *out_surface_position, ZJoltSubShapeId *out_sub_shape_id,
    bool *out_found) {
  ZJOLT_ENTER(out_found);
  if (!zjolt::Present(shape, local_position, out_surface_position,
                      out_sub_shape_id, out_found)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  if (hf == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this shape is not a height field");
  }

  JPH::Vec3 surface_position;
  JPH::SubShapeID sub_shape_id;
  const bool found = hf->ProjectOntoSurface(
      zjolt::ToJolt(*local_position), surface_position, sub_shape_id);
  *out_found = found;
  if (found) {
    *out_surface_position = zjolt::ToC(surface_position);
    *out_sub_shape_id = zjolt::ToC(sub_shape_id);
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeHeightFieldGetSubShapeCoordinates(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id, uint32_t *out_x,
    uint32_t *out_y, uint32_t *out_triangle_index) {
  ZJOLT_ENTER(out_x, out_y, out_triangle_index);
  if (!zjolt::Present(shape, out_x, out_y, out_triangle_index))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  if (hf == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this shape is not a height field");
  }

  JPH::uint x = 0, y = 0, triangle_index = 0;
  hf->GetSubShapeCoordinates(zjolt::ToJoltSubShapeId(sub_shape_id), x, y,
                             triangle_index);
  *out_x = x;
  *out_y = y;
  *out_triangle_index = triangle_index;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeHeightFieldGetHeights(const ZJoltShape *shape,
                                            uint32_t x, uint32_t y,
                                            uint32_t size_x, uint32_t size_y,
                                            float *out_heights,
                                            uint32_t stride) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, out_heights)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (size_x == 0 || size_y == 0) return ZJOLT_RESULT_OK;

  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  if (hf == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this shape is not a height field");
  }

  // Jolt asserts on all four of these instead of clamping or reporting them,
  // which a build with asserts off would read straight past — into a block
  // that starts or ends outside the grid it was carved from.
  const uint32_t block_size = hf->GetBlockSize();
  const uint32_t sample_count = hf->GetSampleCount();
  if (block_size == 0 || x % block_size != 0 || y % block_size != 0 ||
      x >= sample_count || y >= sample_count ||
      x + size_x > sample_count || y + size_y > sample_count ||
      stride < size_x) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "x and y must be a multiple of the block size, the requested block "
        "must fit within the sample grid, and stride must be at least size_x");
  }

  hf->GetHeights(x, y, size_x, size_y, out_heights,
                static_cast<intptr_t>(stride));
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
