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

#include <vector>

#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/Math/Math.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/PolyhedronSubmergedVolumeCalculator.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>

namespace {

/// Turns Jolt's ShapeResult into the ABI's convention: a reference handed to
/// the caller on success, or an error surviving in zjoltLastError.
///
/// The extra AddRef is deliberate: ShapeResult's Ref drops its own reference
/// when it goes out of scope, so this is what makes "every constructor returns
/// a shape with a reference count of one" true.
ZJoltResult Finish(JPH::Shape::ShapeResult &result, ZJoltShape **out) {
  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_RESULT_SHAPE_INVALID, result.GetError().c_str());
  }
  if (!result.IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_SHAPE_INVALID,
                           "shape construction produced no result");
  }
  const JPH::Shape *shape = result.Get();
  zjolt::HostRetain(shape);
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
      // Every shape kind Jolt itself defines is named above; what's left is the
      // sixteen User1..UserConvex8 slots for types registered by C++ outside
      // this library — unconstructible and unnameable here. Deliberately NOT
      // the same value a NULL handle reports: this arm says "a shape, of a kind
      // I cannot name", ZJOLT_SHAPE_SUB_TYPE_NONE says "not a shape".
      return ZJOLT_SHAPE_SUB_TYPE_USER_DEFINED;
  }
}

// Takes the raw integer, not the enum: see zjolt::RawEnum in
// zjolt_internal.h. zjoltShapeCreateMesh converts with it at the boundary,
// before this ever sees the value.
JPH::MeshShapeSettings::EBuildQuality ToJoltBuildQuality(int32_t quality) {
  switch (quality) {
    case ZJOLT_MESH_BUILD_QUALITY_FAVOR_BUILD_SPEED:
      return JPH::MeshShapeSettings::EBuildQuality::FavorBuildSpeed;
    case ZJOLT_MESH_BUILD_QUALITY_FAVOR_RUNTIME_PERFORMANCE:
    default:
      return JPH::MeshShapeSettings::EBuildQuality::FavorRuntimePerformance;
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
/// Empty and one-entry lists are distinct, reachable states: no materials
/// reports the shared default for every leaf, one reports that one. Jolt
/// enforces the same distinction the other way — a triangle with a non-zero
/// material index is refused when the list is empty.
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

/// The fewest children a compound that STAYS a compound may have — not a style
/// rule: `CompoundShape::GetSubShapeIDBits` is `32 - CountLeadingZeros(count -
/// 1)`, UB on ARM (`__builtin_clz(0)`) for a single-child compound, guarded
/// elsewhere but not there. A static compound never reaches this (Jolt
/// simplifies one child away); a mutable one does not simplify, so the floor is
/// enforced here. See UPSTREAM.md.
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

/// The shape as a convex primitive, or null if it is any other kind. @see
/// AsCompound for why this is a GetType dispatch rather than a dynamic_cast.
/// ConvexShape::GetDensity/SetDensity and SetMaterial are the only places
/// this ABI needs the base convex type rather than one of its concrete
/// subtypes.
const JPH::ConvexShape *AsConvex(const ZJoltShape *shape) {
  if (shape == nullptr) return nullptr;
  const JPH::Shape *jolt = zjolt::ToJolt(shape);
  if (jolt->GetType() != JPH::EShapeType::Convex) return nullptr;
  return static_cast<const JPH::ConvexShape *>(jolt);
}

/// Whether `shape` or anything beneath it is one of the three leaf kinds whose
/// GetSubmergedVolume vendored Jolt 5.6.0 implements as JPH_ASSERT(false),
/// leaving its out-parameters uninitialised: Mesh, HeightField, Plane. Walks
/// every compound and decoration, since the hazard is just as real several
/// levels down as at the root.
bool ContainsSubmergedVolumeHazard(const JPH::Shape *shape) {
  if (shape == nullptr) return false;
  switch (shape->GetSubType()) {
    case JPH::EShapeSubType::Mesh:
    case JPH::EShapeSubType::HeightField:
    case JPH::EShapeSubType::Plane:
      return true;
    default:
      break;
  }
  if (shape->GetType() == JPH::EShapeType::Compound) {
    const auto *compound = static_cast<const JPH::CompoundShape *>(shape);
    for (const JPH::CompoundShape::SubShape &sub : compound->GetSubShapes()) {
      if (ContainsSubmergedVolumeHazard(sub.mShape.GetPtr())) return true;
    }
    return false;
  }
  if (shape->GetType() == JPH::EShapeType::Decorated) {
    const auto *decorated = static_cast<const JPH::DecoratedShape *>(shape);
    return ContainsSubmergedVolumeHazard(decorated->GetInnerShape());
  }
  return false;
}

/// The recursive half of zjoltShapeIsSubShapeIDValid. @see its header comment
/// for what each branch means; this is the same logic, just in the shape of
/// a function that can call itself.
bool IsSubShapeIdValid(const JPH::Shape *shape, JPH::SubShapeID id) {
  if (shape == nullptr) return false;

  const uint32_t bits = shape->GetSubShapeIDBitsRecursive();
  if (bits == 0) {
    // A leafless shape: every convex primitive, and a plane. Jolt asserts on
    // anything but the empty id here (see zjoltShapeGetMaterial's own doc).
    return id.IsEmpty();
  }

  if (shape->GetType() == JPH::EShapeType::Compound) {
    const auto *compound = static_cast<const JPH::CompoundShape *>(shape);
    // IsSubShapeIDValid, unlike GetSubShapeIndexFromID, does not assert on a
    // bad id -- that is the whole reason it exists in C++.
    if (!compound->IsSubShapeIDValid(id)) return false;
    JPH::SubShapeID remainder;
    const uint32_t index = compound->GetSubShapeIndexFromID(id, remainder);
    return IsSubShapeIdValid(compound->GetSubShape(index).mShape.GetPtr(),
                             remainder);
  }

  if (shape->GetType() == JPH::EShapeType::Decorated) {
    const auto *decorated = static_cast<const JPH::DecoratedShape *>(shape);
    return IsSubShapeIdValid(decorated->GetInnerShape(), id);
  }

  // A leaf with its own private id space (mesh, height field) or a kind this
  // ABI cannot inspect further (soft body, a shape type registered outside
  // this library). All that can be checked without duplicating Jolt's own
  // internal decode is that this shape's bit budget accounts for the whole
  // id -- the same structural guarantee CompoundShape::IsSubShapeIDValid
  // gives at its own level.
  JPH::SubShapeID remainder;
  id.PopID(bits, remainder);
  return remainder.IsEmpty();
}

/// Opens a mutating compound entry point: the shape must be up, must be a
/// mutable compound, and `index` must name a child it actually has.
///
/// The range check is load-bearing: `RemoveShape`/`ModifyShape` index
/// `mSubShapes` with no bounds check or assertion, so an out-of-range index
/// there is a write past the end of a live array, not a diagnosable abort.
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

/// The mutable counterpart of AsHeightField, for
/// zjoltShapeHeightFieldSetHeights — the one height-field entry point that
/// writes through the pointer. @see OpenMutableCompound for why this
/// const_casts rather than taking a second ToJolt overload: only
/// ffi/zjolt_internal.h converts handles, and it only hands out `const
/// JPH::Shape *`.
JPH::HeightFieldShape *AsMutableHeightField(ZJoltShape *shape) {
  if (shape == nullptr) return nullptr;
  JPH::Shape *jolt = const_cast<JPH::Shape *>(zjolt::ToJolt(shape));
  if (jolt->GetSubType() != JPH::EShapeSubType::HeightField) return nullptr;
  return static_cast<JPH::HeightFieldShape *>(jolt);
}

/// The bounds Jolt asserts on for the material block, which differ from the
/// height block's: no block-size alignment, and the end is strictly inside the
/// sample grid because the quad grid is one smaller in each direction.
ZJoltResult CheckMaterialBlock(const JPH::HeightFieldShape *hf, uint32_t x,
                               uint32_t y, uint32_t size_x, uint32_t size_y,
                               uint32_t stride) {
  const uint32_t sample_count = hf->GetSampleCount();
  if (x >= sample_count || y >= sample_count || x + size_x >= sample_count ||
      y + size_y >= sample_count || stride < size_x) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "the requested block must fit within the quad grid, which is one "
        "smaller than the sample grid in each direction, and stride must be "
        "at least size_x");
  }
  return ZJOLT_RESULT_OK;
}

/// The handle, cast to `T` if `shape` is exactly `sub_type` — null otherwise.
///
/// Every convex-primitive dimension getter below opens with this: Jolt's
/// per-subtype dimension accessors (SphereShape::GetRadius and its
/// siblings) are plain, non-virtual methods, so only GetSubType() can
/// confirm a static_cast is safe (-fno-rtti rules out dynamic_cast).
template <typename T>
const T *NarrowShape(const ZJoltShape *shape, JPH::EShapeSubType sub_type) {
  if (shape == nullptr) return nullptr;
  const JPH::Shape *jolt = zjolt::ToJolt(shape);
  if (jolt->GetSubType() != sub_type) return nullptr;
  return static_cast<const T *>(jolt);
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
                                       float hull_tolerance,
                                       float max_error_convex_radius,
                                       float density,
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
  if (max_error_convex_radius > 0.0f)
    settings.mMaxErrorConvexRadius = max_error_convex_radius;
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
    const uint32_t *triangle_user_data,
    const ZJoltPhysicsMaterial *const *materials, uint32_t num_materials,
    uint32_t max_triangles_per_leaf, float active_edge_cos_threshold_angle,
    ZJoltMeshBuildQuality build_quality, ZJoltShape **out) {
  ZJOLT_ENTER(out);
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_build_quality = zjolt::RawEnum(build_quality);
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
        triangle_materials != nullptr ? triangle_materials[i] : 0u,
        triangle_user_data != nullptr ? triangle_user_data[i] : 0u));
  }

  JPH::MeshShapeSettings settings(std::move(vertex_list),
                                  std::move(triangle_list),
                                  std::move(material_list));
  if (max_triangles_per_leaf > 0)
    settings.mMaxTrianglesPerLeaf = max_triangles_per_leaf;
  settings.mActiveEdgeCosThresholdAngle = active_edge_cos_threshold_angle;
  settings.mBuildQuality = ToJoltBuildQuality(raw_build_quality);
  // Storing a value nobody can read back would just cost memory: Jolt
  // defaults this off, and turning it on is what makes
  // zjoltShapeMeshGetTriangleUserData report what was actually asked for
  // instead of its own pre-reorder fallback.
  settings.mPerTriangleUserData = triangle_user_data != nullptr;
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
    uint32_t materials_capacity, uint32_t block_size, uint32_t bits_per_sample,
    float min_height_value, float max_height_value,
    float active_edge_cos_threshold_angle, ZJoltShape **out) {
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
  // Fixes the 16-bit quantisation range once and for all: Jolt widens it to
  // fit whatever the samples below actually need (DetermineMinAndMaxSample),
  // but never narrows it, so passing a wider pair than the samples span here
  // is what reserves headroom a later zjoltShapeHeightFieldSetHeights can
  // move a sample into without being clamped.
  settings.mMinHeightValue = min_height_value;
  settings.mMaxHeightValue = max_height_value;
  settings.mActiveEdgeCosThresholdAngle = active_edge_cos_threshold_angle;

  settings.mMaterialsCapacity = materials_capacity;
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

  settings.ClearCachedResult();
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

  settings.ClearCachedResult();
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

ZJoltResult zjoltShapeCompoundSetChildUserData(ZJoltShape *shape,
                                               uint32_t index,
                                               uint32_t user_data) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Not gated behind AsMutableCompound / OpenMutableCompound: unlike adding,
  // removing or moving a child, SetCompoundUserData is a plain field write
  // Jolt declares on the base CompoundShape, reachable on a static compound
  // too.
  JPH::Shape *jolt = const_cast<JPH::Shape *>(zjolt::ToJolt(shape));
  if (jolt->GetType() != JPH::EShapeType::Compound) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a compound");
  }
  auto *compound = static_cast<JPH::CompoundShape *>(jolt);
  if (index >= compound->GetNumSubShapes()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "no child of this compound has that index");
  }
  compound->SetCompoundUserData(index, user_data);
  return ZJOLT_RESULT_OK;
}

struct ZJoltSubShapeIdCreator {
  JPH::SubShapeIDCreator creator;
};

ZJoltResult zjoltSubShapeIdCreatorCreate(ZJoltSubShapeIdCreator **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJoltSubShapeIdCreator *handle = zjolt::New<ZJoltSubShapeIdCreator>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltSubShapeIdCreatorDestroy(ZJoltSubShapeIdCreator *creator) {
  if (creator == nullptr) return;
  zjolt::Delete(creator);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltSubShapeIdCreatorPushID(ZJoltSubShapeIdCreator *creator,
                                         uint32_t value, uint32_t bits) {
  ZJOLT_ENTER();
  if (!zjolt::Present(creator)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (bits < 32 && value >= (uint32_t(1) << bits)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "value: does not fit in `bits` bits");
  }
  if (creator->creator.GetNumBitsWritten() + bits > JPH::SubShapeID::MaxBits) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "bits: would write past SubShapeID::MaxBits (32) total");
  }
  creator->creator = creator->creator.PushID(value, bits);
  return ZJOLT_RESULT_OK;
}

ZJoltSubShapeId zjoltSubShapeIdCreatorGetID(const ZJoltSubShapeIdCreator *creator) {
  if (creator == nullptr) return ZJOLT_SUB_SHAPE_ID_EMPTY;
  return zjolt::ToC(creator->creator.GetID());
}

uint32_t zjoltSubShapeIdCreatorGetNumBitsWritten(const ZJoltSubShapeIdCreator *creator) {
  if (creator == nullptr) return 0;
  return creator->creator.GetNumBitsWritten();
}

ZJoltResult zjoltSubShapeIdPopID(ZJoltSubShapeId id, uint32_t bits,
                                 uint32_t *out_value,
                                 ZJoltSubShapeId *out_remainder) {
  ZJOLT_ENTER(out_value, out_remainder);
  if (!zjolt::Present(out_value, out_remainder)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  if (bits > JPH::SubShapeID::MaxBits) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "bits: exceeds JPH::SubShapeID::MaxBits (32)");
  }
  JPH::SubShapeID remainder;
  *out_value = zjolt::ToJoltSubShapeId(id).PopID(bits, remainder);
  *out_remainder = zjolt::ToC(remainder);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetSubShapeIDFromIndex(const ZJoltShape *shape,
                                             uint32_t index,
                                             ZJoltSubShapeId *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::CompoundShape *compound = AsCompound(shape);
  if (compound == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a compound");
  }
  // Bounds-checked here rather than left to Jolt: SubShapeIDCreator::PushID
  // asserts on an index its bit width cannot represent, which an
  // out-of-range (but in-width) index would otherwise reach silently.
  if (index >= compound->GetNumSubShapes()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "no child of this compound has that index");
  }
  const JPH::SubShapeIDCreator id = compound->GetSubShapeIDFromIndex(
      static_cast<int>(index), JPH::SubShapeIDCreator());
  *out = zjolt::ToC(id.GetID());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetSubShapeIDFromIndexInto(const ZJoltShape *shape,
                                                 uint32_t index,
                                                 ZJoltSubShapeIdCreator *creator) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, creator)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::CompoundShape *compound = AsCompound(shape);
  if (compound == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a compound");
  }
  if (index >= compound->GetNumSubShapes()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "no child of this compound has that index");
  }
  creator->creator = compound->GetSubShapeIDFromIndex(
      static_cast<int>(index), creator->creator);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetSubShapeIndexFromID(const ZJoltShape *shape,
                                             ZJoltSubShapeId sub_shape_id,
                                             uint32_t *out_index,
                                             ZJoltSubShapeId *out_remainder) {
  ZJOLT_ENTER(out_index, out_remainder);
  if (!zjolt::Present(shape, out_index, out_remainder))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::CompoundShape *compound = AsCompound(shape);
  if (compound == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a compound");
  }
  const JPH::SubShapeID id = zjolt::ToJoltSubShapeId(sub_shape_id);
  // IsSubShapeIDValid never asserts; GetSubShapeIndexFromID does on a bad id,
  // so this is what keeps that assert unreachable from here.
  if (!compound->IsSubShapeIDValid(id)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "sub_shape_id does not name a direct child of this compound");
  }
  JPH::SubShapeID remainder;
  *out_index = compound->GetSubShapeIndexFromID(id, remainder);
  *out_remainder = zjolt::ToC(remainder);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetIntersectingSubShapes(const ZJoltShape *shape,
                                               const ZJoltAABox *box,
                                               uint32_t *out_indices,
                                               uint32_t capacity,
                                               uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape, box, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::CompoundShape *compound = AsCompound(shape);
  if (compound == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a compound");
  }

  const uint32_t num_children =
      static_cast<uint32_t>(compound->GetNumSubShapes());
  if (out_indices == nullptr || capacity < num_children) {
    *out_count = num_children;
    return out_indices == nullptr ? ZJOLT_RESULT_OK
                                  : ZJOLT_RESULT_BUFFER_TOO_SMALL;
  }

  static_assert(sizeof(JPH::uint) == sizeof(uint32_t) &&
                    alignof(JPH::uint) == alignof(uint32_t),
                "JPH::uint must lay out the same as uint32_t for this cast");
  const JPH::AABox jolt_box(zjolt::ToJolt(box->min), zjolt::ToJolt(box->max));
  const int found = compound->GetIntersectingSubShapes(
      jolt_box, reinterpret_cast<JPH::uint *>(out_indices),
      static_cast<int>(capacity));
  *out_count = static_cast<uint32_t>(found);
  return ZJOLT_RESULT_OK;
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

ZJoltResult zjoltShapeMutableCompoundReplaceChild(ZJoltShape *shape,
                                                  uint32_t index,
                                                  const ZJoltShape *new_shape,
                                                  const ZJoltVec3 *position,
                                                  const ZJoltQuat *rotation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, new_shape, position))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::MutableCompoundShape *compound = nullptr;
  const ZJoltResult ready = OpenMutableCompound(shape, index, true, &compound);
  if (ready != ZJOLT_RESULT_OK) return ready;

  compound->ModifyShape(index, zjolt::ToJolt(*position),
                        rotation != nullptr ? zjolt::ToJoltRotation(*rotation)
                                            : JPH::Quat::sIdentity(),
                        zjolt::ToJolt(new_shape));
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

ZJoltResult zjoltShapeGetInnerShape(const ZJoltShape *shape,
                                    const ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Shape *jolt = zjolt::ToJolt(shape);
  if (jolt->GetType() != JPH::EShapeType::Decorated) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "shape is not decorated (scaled, rotated-translated, or "
        "offset-center-of-mass)");
  }
  *out = zjolt::ToC(static_cast<const JPH::DecoratedShape *>(jolt)->GetInnerShape());
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Lifetime and introspection
//===----------------------------------------------------------------------===//

void zjoltShapeAddRef(const ZJoltShape *shape) {
  if (shape == nullptr) return;
  zjolt::HostRetain(zjolt::ToJolt(shape));
}

void zjoltShapeRelease(const ZJoltShape *shape) {
  if (shape == nullptr) return;
  zjolt::HostRelease(zjolt::ToJolt(shape));
}

uint32_t zjoltShapeGetRefCount(const ZJoltShape *shape) {
  if (shape == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(shape)->GetRefCount());
}

uint64_t zjoltShapeGetUserData(const ZJoltShape *shape) {
  if (shape == nullptr) return 0;
  return zjolt::ToJolt(shape)->GetUserData();
}

void zjoltShapeSetUserData(ZJoltShape *shape, uint64_t user_data) {
  if (shape == nullptr) return;
  const_cast<JPH::Shape *>(zjolt::ToJolt(shape))->SetUserData(user_data);
}

ZJoltShapeSubType zjoltShapeGetSubType(const ZJoltShape *shape) {
  if (shape == nullptr) return ZJOLT_SHAPE_SUB_TYPE_NONE;
  return ToCSubType(zjolt::ToJolt(shape)->GetSubType());
}

const ZJoltPhysicsMaterial *zjoltShapeGetMaterial(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id) {
  if (shape == nullptr) return nullptr;
  return zjolt::ToC(
      zjolt::ToJolt(shape)->GetMaterial(zjolt::ToJoltSubShapeId(sub_shape_id)));
}

ZJoltResult zjoltShapeSetMaterial(ZJoltShape *shape,
                                  const ZJoltPhysicsMaterial *material) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexShape *convex = AsConvex(shape);
  if (convex == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex primitive");
  }
  // RefConst<PhysicsMaterial>'s own assignment operator does the reference
  // counting: releases whatever material this shape held, adds one to
  // `material`. NULL is a valid material here the same way it is at
  // creation, and clears the shape back to Jolt's shared default.
  const_cast<JPH::ConvexShape *>(convex)->SetMaterial(zjolt::ToJolt(material));
  return ZJOLT_RESULT_OK;
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
  // Jolt stores inertia in the upper-left 3x3 of a Mat44; zjolt::ToJolt is
  // the exact inverse of this unpacking, and lives beside it.
  zjolt::WriteMassProperties(out, zjolt::ToJolt(shape)->GetMassProperties());
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
// Convex-primitive dimension introspection
//===----------------------------------------------------------------------===//

float zjoltShapeGetInnerRadius(const ZJoltShape *shape) {
  if (shape == nullptr) return 0.0f;
  return zjolt::ToJolt(shape)->GetInnerRadius();
}

ZJoltResult zjoltShapeGetDensity(const ZJoltShape *shape, float *out_density) {
  ZJOLT_ENTER(out_density);
  if (!zjolt::Present(shape, out_density)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexShape *convex = AsConvex(shape);
  if (convex == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex primitive");
  }
  *out_density = convex->GetDensity();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeSetDensity(ZJoltShape *shape, float density) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexShape *convex = AsConvex(shape);
  if (convex == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex primitive");
  }
  const_cast<JPH::ConvexShape *>(convex)->SetDensity(density);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetRadius(const ZJoltShape *shape, float *out_radius) {
  ZJOLT_ENTER(out_radius);
  if (!zjolt::Present(shape, out_radius)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (const auto *sphere =
          NarrowShape<JPH::SphereShape>(shape, JPH::EShapeSubType::Sphere)) {
    *out_radius = sphere->GetRadius();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *capsule = NarrowShape<JPH::CapsuleShape>(
          shape, JPH::EShapeSubType::Capsule)) {
    *out_radius = capsule->GetRadius();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *cylinder = NarrowShape<JPH::CylinderShape>(
          shape, JPH::EShapeSubType::Cylinder)) {
    *out_radius = cylinder->GetRadius();
    return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                         "shape is not a sphere, capsule, or cylinder");
}

ZJoltResult zjoltShapeGetHalfExtent(const ZJoltShape *shape,
                                    ZJoltVec3 *out_half_extent) {
  ZJOLT_ENTER(out_half_extent);
  if (!zjolt::Present(shape, out_half_extent))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::BoxShape *box =
      NarrowShape<JPH::BoxShape>(shape, JPH::EShapeSubType::Box);
  if (box == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "shape is not a box");
  }
  *out_half_extent = zjolt::ToC(box->GetHalfExtent());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetHalfHeight(const ZJoltShape *shape,
                                    float *out_half_height) {
  ZJOLT_ENTER(out_half_height);
  if (!zjolt::Present(shape, out_half_height))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (const auto *cylinder = NarrowShape<JPH::CylinderShape>(
          shape, JPH::EShapeSubType::Cylinder)) {
    *out_half_height = cylinder->GetHalfHeight();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *tapered_capsule = NarrowShape<JPH::TaperedCapsuleShape>(
          shape, JPH::EShapeSubType::TaperedCapsule)) {
    *out_half_height = tapered_capsule->GetHalfHeight();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *tapered_cylinder = NarrowShape<JPH::TaperedCylinderShape>(
          shape, JPH::EShapeSubType::TaperedCylinder)) {
    *out_half_height = tapered_cylinder->GetHalfHeight();
    return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "shape is not a cylinder, tapered capsule, or tapered cylinder");
}

ZJoltResult zjoltShapeGetHalfHeightOfCylinder(
    const ZJoltShape *shape, float *out_half_height_of_cylinder) {
  ZJOLT_ENTER(out_half_height_of_cylinder);
  if (!zjolt::Present(shape, out_half_height_of_cylinder))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::CapsuleShape *capsule =
      NarrowShape<JPH::CapsuleShape>(shape, JPH::EShapeSubType::Capsule);
  if (capsule == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a capsule");
  }
  *out_half_height_of_cylinder = capsule->GetHalfHeightOfCylinder();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetTopRadius(const ZJoltShape *shape,
                                   float *out_top_radius) {
  ZJOLT_ENTER(out_top_radius);
  if (!zjolt::Present(shape, out_top_radius))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (const auto *tapered_capsule = NarrowShape<JPH::TaperedCapsuleShape>(
          shape, JPH::EShapeSubType::TaperedCapsule)) {
    *out_top_radius = tapered_capsule->GetTopRadius();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *tapered_cylinder = NarrowShape<JPH::TaperedCylinderShape>(
          shape, JPH::EShapeSubType::TaperedCylinder)) {
    *out_top_radius = tapered_cylinder->GetTopRadius();
    return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                         "shape is not a tapered capsule or tapered cylinder");
}

ZJoltResult zjoltShapeGetBottomRadius(const ZJoltShape *shape,
                                      float *out_bottom_radius) {
  ZJOLT_ENTER(out_bottom_radius);
  if (!zjolt::Present(shape, out_bottom_radius))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (const auto *tapered_capsule = NarrowShape<JPH::TaperedCapsuleShape>(
          shape, JPH::EShapeSubType::TaperedCapsule)) {
    *out_bottom_radius = tapered_capsule->GetBottomRadius();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *tapered_cylinder = NarrowShape<JPH::TaperedCylinderShape>(
          shape, JPH::EShapeSubType::TaperedCylinder)) {
    *out_bottom_radius = tapered_cylinder->GetBottomRadius();
    return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                         "shape is not a tapered capsule or tapered cylinder");
}

ZJoltResult zjoltShapeGetConvexRadius(const ZJoltShape *shape,
                                      float *out_convex_radius) {
  ZJOLT_ENTER(out_convex_radius);
  if (!zjolt::Present(shape, out_convex_radius))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (const auto *box =
          NarrowShape<JPH::BoxShape>(shape, JPH::EShapeSubType::Box)) {
    *out_convex_radius = box->GetConvexRadius();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *cylinder = NarrowShape<JPH::CylinderShape>(
          shape, JPH::EShapeSubType::Cylinder)) {
    *out_convex_radius = cylinder->GetConvexRadius();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *hull = NarrowShape<JPH::ConvexHullShape>(
          shape, JPH::EShapeSubType::ConvexHull)) {
    *out_convex_radius = hull->GetConvexRadius();
    return ZJOLT_RESULT_OK;
  }
  if (const auto *tapered_cylinder = NarrowShape<JPH::TaperedCylinderShape>(
          shape, JPH::EShapeSubType::TaperedCylinder)) {
    *out_convex_radius = tapered_cylinder->GetConvexRadius();
    return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "shape is not a box, cylinder, convex hull, or tapered cylinder");
}

ZJoltResult zjoltShapeGetNumFaces(const ZJoltShape *shape,
                                  uint32_t *out_num_faces) {
  ZJOLT_ENTER(out_num_faces);
  if (!zjolt::Present(shape, out_num_faces))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexHullShape *hull =
      NarrowShape<JPH::ConvexHullShape>(shape, JPH::EShapeSubType::ConvexHull);
  if (hull == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex hull");
  }
  *out_num_faces = hull->GetNumFaces();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetNumVerticesInFace(const ZJoltShape *shape,
                                           uint32_t face_index,
                                           uint32_t *out_num_vertices) {
  ZJOLT_ENTER(out_num_vertices);
  if (!zjolt::Present(shape, out_num_vertices))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexHullShape *hull =
      NarrowShape<JPH::ConvexHullShape>(shape, JPH::EShapeSubType::ConvexHull);
  if (hull == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex hull");
  }
  if (face_index >= hull->GetNumFaces()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "face_index is out of range for this convex hull; Jolt indexes its "
        "own face array with no bounds check at all");
  }
  *out_num_vertices = hull->GetNumVerticesInFace(face_index);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetPoints(const ZJoltShape *shape, ZJoltVec3 *out_points,
                                uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexHullShape *hull =
      NarrowShape<JPH::ConvexHullShape>(shape, JPH::EShapeSubType::ConvexHull);
  if (hull == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex hull");
  }
  const uint32_t num_points = hull->GetNumPoints();
  *out_count = num_points;
  if (out_points == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < num_points) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < num_points; ++i)
    out_points[i] = zjolt::ToC(hull->GetPoint(i));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetPlanes(const ZJoltShape *shape, ZJoltPlane *out_planes,
                                uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexHullShape *hull =
      NarrowShape<JPH::ConvexHullShape>(shape, JPH::EShapeSubType::ConvexHull);
  if (hull == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex hull");
  }
  const auto &planes = hull->GetPlanes();
  const uint32_t num_planes = static_cast<uint32_t>(planes.size());
  *out_count = num_planes;
  if (out_planes == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < num_planes) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < num_planes; ++i) {
    out_planes[i].normal = zjolt::ToC(planes[i].GetNormal());
    out_planes[i].constant = planes[i].GetConstant();
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeGetPlane(const ZJoltShape *shape, ZJoltPlane *out_plane,
                               float *out_half_extent) {
  ZJOLT_ENTER(out_plane, out_half_extent);
  if (!zjolt::Present(shape, out_plane, out_half_extent))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::PlaneShape *plane_shape =
      NarrowShape<JPH::PlaneShape>(shape, JPH::EShapeSubType::Plane);
  if (plane_shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a plane");
  }
  out_plane->normal = zjolt::ToC(plane_shape->GetPlane().GetNormal());
  out_plane->constant = plane_shape->GetPlane().GetConstant();
  *out_half_extent = plane_shape->GetHalfExtent();
  return ZJOLT_RESULT_OK;
}

static_assert(sizeof(ZJoltShapeSupportBuffer) ==
                  sizeof(JPH::ConvexShape::SupportBuffer) &&
              alignof(ZJoltShapeSupportBuffer) ==
                  alignof(JPH::ConvexShape::SupportBuffer),
             "ZJoltShapeSupportBuffer must match "
             "JPH::ConvexShape::SupportBuffer exactly");

ZJoltResult zjoltShapeGetSupportFunction(const ZJoltShape *shape,
                                         ZJoltShapeSupportMode mode,
                                         ZJoltShapeSupportBuffer *buffer,
                                         const ZJoltVec3 *scale,
                                         const ZJoltShapeSupportFunction **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape, buffer, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexShape *convex = AsConvex(shape);
  if (convex == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is not a convex primitive");
  }

  JPH::ConvexShape::ESupportMode jolt_mode;
  switch (mode) {
    case ZJOLT_SHAPE_SUPPORT_MODE_INCLUDE_CONVEX_RADIUS:
      jolt_mode = JPH::ConvexShape::ESupportMode::IncludeConvexRadius;
      break;
    case ZJOLT_SHAPE_SUPPORT_MODE_DEFAULT:
      jolt_mode = JPH::ConvexShape::ESupportMode::Default;
      break;
    case ZJOLT_SHAPE_SUPPORT_MODE_EXCLUDE_CONVEX_RADIUS:
    default:
      jolt_mode = JPH::ConvexShape::ESupportMode::ExcludeConvexRadius;
      break;
  }

  const JPH::Vec3 jolt_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
  auto *jolt_buffer =
      reinterpret_cast<JPH::ConvexShape::SupportBuffer *>(buffer);
  const JPH::ConvexShape::Support *support =
      convex->GetSupportFunction(jolt_mode, *jolt_buffer, jolt_scale);
  *out = reinterpret_cast<const ZJoltShapeSupportFunction *>(support);
  return ZJOLT_RESULT_OK;
}

void zjoltShapeSupportFunctionGetSupport(
    const ZJoltShapeSupportFunction *support, const ZJoltVec3 *direction,
    ZJoltVec3 *out) {
  if (out != nullptr) *out = ZJoltVec3{0, 0, 0};
  if (support == nullptr || direction == nullptr) return;
  const auto *jolt_support =
      reinterpret_cast<const JPH::ConvexShape::Support *>(support);
  const JPH::Vec3 result = jolt_support->GetSupport(zjolt::ToJolt(*direction));
  if (out != nullptr) *out = zjolt::ToC(result);
}

float zjoltShapeSupportFunctionGetConvexRadius(
    const ZJoltShapeSupportFunction *support) {
  if (support == nullptr) return 0.0f;
  const auto *jolt_support =
      reinterpret_cast<const JPH::ConvexShape::Support *>(support);
  return jolt_support->GetConvexRadius();
}

ZJoltResult zjoltShapeGetSubmergedVolume(
    const ZJoltShape *shape, const ZJoltMat44 *transform,
    const ZJoltVec3 *scale, const ZJoltPlane *surface,
    float *out_total_volume, float *out_submerged_volume,
    ZJoltVec3 *out_center_of_buoyancy) {
  ZJOLT_ENTER(out_total_volume, out_submerged_volume, out_center_of_buoyancy);
  if (!zjolt::Present(shape, transform, surface, out_total_volume,
                      out_submerged_volume, out_center_of_buoyancy)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::Shape *jolt_shape = zjolt::ToJolt(shape);
  if (ContainsSubmergedVolumeHazard(jolt_shape)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "shape or one of its descendants is a mesh, a height field, or a "
        "plane; Jolt's own GetSubmergedVolume for those asserts false "
        "instead of computing an answer");
  }

  const JPH::Vec3 jolt_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
  float total_volume = 0.0f;
  float submerged_volume = 0.0f;
  JPH::Vec3 center_of_buoyancy = JPH::Vec3::sZero();
  jolt_shape->GetSubmergedVolume(
      zjolt::ToJolt(*transform), jolt_scale,
      JPH::Plane(zjolt::ToJolt(surface->normal), surface->constant),
      total_volume, submerged_volume,
      center_of_buoyancy JPH_IF_DEBUG_RENDERER(, JPH::RVec3::sZero()));

  *out_total_volume = total_volume;
  *out_submerged_volume = submerged_volume;
  *out_center_of_buoyancy = zjolt::ToC(center_of_buoyancy);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Submerged-volume accumulation for an arbitrary convex polyhedron
//===----------------------------------------------------------------------===//

struct ZJoltPolyhedronSubmergedVolumeCalculator {
  /// Constructed before `calc` below (declaration order), and outlives it:
  /// Jolt's own class only borrows a pointer into this. The world origin a
  /// debug-renderer build asks for is zero: this ABI carries none, so cut
  /// geometry draws around the origin of the caller's own space, and a C
  /// signature growing a parameter under one build option would make the
  /// boundary configuration-dependent, which only the width options are.
  std::vector<JPH::PolyhedronSubmergedVolumeCalculator::Point> points;
  uint32_t num_points;
  JPH::PolyhedronSubmergedVolumeCalculator calc;

  ZJoltPolyhedronSubmergedVolumeCalculator(const JPH::Mat44 &transform,
                                           const JPH::Vec3 *positions,
                                           uint32_t count,
                                           const JPH::Plane &surface)
      : points(count),
        num_points(count),
        calc(transform, positions, sizeof(JPH::Vec3),
             static_cast<int>(count), surface, points.data()
                 JPH_IF_DEBUG_RENDERER(, JPH::RVec3::sZero())) {}
};

ZJoltResult zjoltPolyhedronSubmergedVolumeCalculatorCreate(
    const ZJoltMat44 *transform, const ZJoltVec3 *points, uint32_t num_points,
    const ZJoltPlane *surface,
    ZJoltPolyhedronSubmergedVolumeCalculator **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (transform == nullptr || points == nullptr || num_points == 0 ||
      surface == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "transform, points and surface are required, and num_points must "
        "be at least one");
  }

  std::vector<JPH::Vec3> jolt_points;
  jolt_points.reserve(num_points);
  for (uint32_t i = 0; i < num_points; ++i) {
    jolt_points.push_back(zjolt::ToJolt(points[i]));
  }
  const JPH::Plane jolt_surface(zjolt::ToJolt(surface->normal),
                                surface->constant);

  ZJoltPolyhedronSubmergedVolumeCalculator *handle =
      zjolt::New<ZJoltPolyhedronSubmergedVolumeCalculator>(
          zjolt::ToJolt(*transform), jolt_points.data(), num_points,
          jolt_surface);
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltPolyhedronSubmergedVolumeCalculatorDestroy(
    ZJoltPolyhedronSubmergedVolumeCalculator *calc) {
  if (calc == nullptr) return;
  zjolt::Delete(calc);
  zjolt::HandleDestroyed();
}

bool zjoltPolyhedronSubmergedVolumeCalculatorAreAllAbove(
    const ZJoltPolyhedronSubmergedVolumeCalculator *calc) {
  if (calc == nullptr) return false;
  return calc->calc.AreAllAbove();
}

bool zjoltPolyhedronSubmergedVolumeCalculatorAreAllBelow(
    const ZJoltPolyhedronSubmergedVolumeCalculator *calc) {
  if (calc == nullptr) return false;
  return calc->calc.AreAllBelow();
}

uint32_t zjoltPolyhedronSubmergedVolumeCalculatorGetReferencePointIdx(
    const ZJoltPolyhedronSubmergedVolumeCalculator *calc) {
  if (calc == nullptr) return 0;
  return static_cast<uint32_t>(calc->calc.GetReferencePointIdx());
}

ZJoltResult zjoltPolyhedronSubmergedVolumeCalculatorAddFace(
    ZJoltPolyhedronSubmergedVolumeCalculator *calc, uint32_t idx1,
    uint32_t idx2, uint32_t idx3) {
  ZJOLT_ENTER();
  if (!zjolt::Present(calc)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (idx1 >= calc->num_points || idx2 >= calc->num_points ||
      idx3 >= calc->num_points) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a face index is at or beyond num_points");
  }
  const uint32_t ref =
      static_cast<uint32_t>(calc->calc.GetReferencePointIdx());
  if (idx1 == ref || idx2 == ref || idx3 == ref) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a face using the reference point contributes no volume; skip it "
        "instead of adding it");
  }
  calc->calc.AddFace(static_cast<int>(idx1), static_cast<int>(idx2),
                     static_cast<int>(idx3));
  return ZJOLT_RESULT_OK;
}

void zjoltPolyhedronSubmergedVolumeCalculatorGetResult(
    const ZJoltPolyhedronSubmergedVolumeCalculator *calc,
    float *out_submerged_volume, ZJoltVec3 *out_center_of_buoyancy) {
  if (out_submerged_volume != nullptr) *out_submerged_volume = 0.0f;
  if (out_center_of_buoyancy != nullptr) {
    *out_center_of_buoyancy = ZJoltVec3{0, 0, 0};
  }
  if (calc == nullptr) return;
  float volume = 0.0f;
  JPH::Vec3 center = JPH::Vec3::sZero();
  calc->calc.GetResult(volume, center);
  if (out_submerged_volume != nullptr) *out_submerged_volume = volume;
  if (out_center_of_buoyancy != nullptr) {
    *out_center_of_buoyancy = zjolt::ToC(center);
  }
}

//===----------------------------------------------------------------------===//
// Serialisation
//
// SaveWithChildren/sRestoreWithChildren, not plain binary state, since a graph
// shape cannot re-supply children order through a flat ABI. The container
// exists because Jolt validates AFTER using an unchecked field first
// (sRestoreFromBinaryState indexes a table with the shape-type byte before its
// own EOF check) — magic/length/build-stamp/checksum catch a bad buffer first.
// NOT a defence against a deliberately crafted payload with a matching checksum
// — treat a shape cache as your own cook's output, not untrusted input.
//===----------------------------------------------------------------------===//

namespace {

/// The shared container from zjolt_internal.h, with this subsystem's own tag
/// and four reserved bytes it does not yet use.
constexpr zjolt::ContainerFormat kContainer = {
    /*magic=*/{'Z', 'J', 'S', 'H'},
    /*version=*/1,
    /*extra_size=*/4,
    /*too_short=*/"too short to be a saved shape",
    /*wrong_magic=*/"not a shape saved by zjoltShapeSave",
    /*bad_checksum=*/"the shape payload failed its checksum",
};

constexpr size_t kHeaderSize = kContainer.HeaderSize();

/// The size the public header promises, checked against the one the container
/// actually writes rather than kept in step by hand.
static_assert(kHeaderSize == ZJOLT_SHAPE_HEADER_SIZE,
              "ZJOLT_SHAPE_HEADER_SIZE no longer matches the header "
              "zjoltShapeSave writes");

static_assert(zjolt::kStreamHeaderSize == ZJOLT_SHAPE_STREAM_HEADER_SIZE,
              "ZJOLT_SHAPE_STREAM_HEADER_SIZE no longer matches the header "
              "zjoltShapeSaveStream writes");

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

  zjolt::WriteContainerHeader(kContainer, bytes, payload_size,
                              /*extra=*/nullptr);
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

  zjolt::ContainerContents contents;
  const ZJoltResult framed =
      zjolt::ReadContainer(kContainer, data, size, &contents);
  if (framed != ZJOLT_RESULT_OK) return framed;

  zjolt::ConstStreamIn stream(contents.payload, contents.payload_size);
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
// The stream form
//
// Jolt's own payload behind a twelve-byte header (not the 32-byte buffer
// container above) — @see ZJoltStream in zjolt_core.h.
// The twelve bytes do NOT close the upstream bug the buffer form's
// length/CRC-32 route around (sRestoreFromBinaryState uses the shape-type byte
// before its own EOF check — see UPSTREAM.md): a stream restore is no safer
// against a CORRUPTED payload than handing Jolt's StreamIn directly. Treat a
// shape stream like a shape cache — your own pipeline's output, not an
// arbitrary transport.
//===----------------------------------------------------------------------===//

namespace {
constexpr uint8_t kStreamMagic[4] = {'Z', 'S', 'S', 'H'};
}  // namespace

ZJoltResult zjoltShapeSaveStream(const ZJoltShape *shape,
                                 const ZJoltStream *stream) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to save through");
  }

  zjolt::HostStream host(*stream);
  zjolt::WriteStreamHeader(host, kStreamMagic);

  JPH::Shape::ShapeToIDMap shape_map;
  JPH::Shape::MaterialToIDMap material_map;
  zjolt::ToJolt(shape)->SaveWithChildren(host, shape_map, material_map);

  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while writing the shape");
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeRestoreStream(const ZJoltStream *stream,
                                    ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(stream, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "stream needs read, is_eof and is_failed to restore through");
  }

  zjolt::HostStream host(*stream);
  const ZJoltResult header = zjolt::ReadStreamHeader(
      host, kStreamMagic, "not a shape saved by zjoltShapeSaveStream");
  if (header != ZJOLT_RESULT_OK) return header;

  JPH::Shape::IDToShapeMap shape_map;
  JPH::Shape::IDToMaterialMap material_map;
  JPH::Shape::ShapeResult result =
      JPH::Shape::sRestoreWithChildren(host, shape_map, material_map);

  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT, result.GetError().c_str());
  }
  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while reading the shape");
  }
  if (!result.IsValid() || host.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the stream ended before the shape did");
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

bool zjoltShapeIsSubShapeIDValid(const ZJoltShape *shape,
                                 ZJoltSubShapeId sub_shape_id) {
  if (shape == nullptr) return false;
  return IsSubShapeIdValid(zjolt::ToJolt(shape),
                           zjolt::ToJoltSubShapeId(sub_shape_id));
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

  // Built from the child's own public fields, not by reaching into
  // JPH::TransformedShape's insides: zjoltTransformedShapeCreate is the one
  // place a ZJoltTransformedShape is constructed, and this keeps that true
  // instead of adding a second path with its own chance to drift. The body id
  // is always invalid — this relates two shapes, not a shape and a body.
  const ZJoltRVec3 child_position = zjolt::ToCR(child.mShapePositionCOM);
  const ZJoltQuat child_rotation = zjolt::ToC(child.mShapeRotation);
  const ZJoltVec3 child_scale = zjolt::ToC(child.GetShapeScale());
  return zjoltTransformedShapeCreate(zjolt::ToC(child.mShape.GetPtr()),
                                     &child_position, &child_rotation,
                                     &child_scale, ZJOLT_BODY_ID_INVALID, out);
}

const ZJoltShape *zjoltShapeGetLeafShape(const ZJoltShape *shape,
                                         ZJoltSubShapeId sub_shape_id,
                                         ZJoltSubShapeId *out_remainder) {
  if (shape == nullptr) {
    if (out_remainder != nullptr) *out_remainder = ZJOLT_SUB_SHAPE_ID_EMPTY;
    return nullptr;
  }
  JPH::SubShapeID remainder;
  const JPH::Shape *leaf = zjolt::ToJolt(shape)->GetLeafShape(
      zjolt::ToJoltSubShapeId(sub_shape_id), remainder);
  if (out_remainder != nullptr) *out_remainder = zjolt::ToC(remainder);
  return zjolt::ToC(leaf);
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

ZJoltResult zjoltShapeHeightFieldSetHeights(
    ZJoltShape *shape, uint32_t x, uint32_t y, uint32_t size_x,
    uint32_t size_y, const float *heights, uint32_t stride,
    float active_edge_cos_threshold_angle) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, heights)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (size_x == 0 || size_y == 0) return ZJOLT_RESULT_OK;

  JPH::HeightFieldShape *hf = AsMutableHeightField(shape);
  if (hf == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this shape is not a height field");
  }

  // Same bounds zjoltShapeHeightFieldGetHeights checks, for the same reason:
  // Jolt asserts on all four instead of clamping.
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

  // Jolt's doc says a height outside [Min,Max] "will be clamped", but
  // quantization (`(int)floor((h - offset) / scale)`) runs first — UB for
  // an `h` far enough outside range, reachable when a flat field's
  // zero-width-range guard shrinks `scale` enough to overflow int. So the
  // clamp runs here, before quantization; `cNoCollisionValue` stays
  // untouched, a sentinel Jolt excludes from quantization too.
  JPH::Array<float> clamped(static_cast<size_t>(size_x) * size_y);
  const float min_height = hf->GetMinHeightValue();
  const float max_height = hf->GetMaxHeightValue();
  for (uint32_t row = 0; row < size_y; ++row) {
    for (uint32_t col = 0; col < size_x; ++col) {
      const float h = heights[static_cast<size_t>(row) * stride + col];
      clamped[static_cast<size_t>(row) * size_x + col] =
          h == JPH::HeightFieldShapeConstants::cNoCollisionValue
              ? h
              : JPH::Clamp(h, min_height, max_height);
    }
  }

  // A plain malloc-backed allocator rather than the physics system's own
  // arena: this shape need not belong to any system yet, and the memory
  // SetHeights needs is bounded by the touched block plus a border of one
  // block size, not something worth threading a shared arena in for.
  JPH::TempAllocatorMalloc allocator;
  hf->SetHeights(x, y, size_x, size_y, clamped.data(),
                static_cast<intptr_t>(size_x), allocator,
                active_edge_cos_threshold_angle);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeHeightFieldGetMaterials(const ZJoltShape *shape,
                                              uint32_t x, uint32_t y,
                                              uint32_t size_x, uint32_t size_y,
                                              uint8_t *out_materials,
                                              uint32_t stride) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, out_materials)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  if (size_x == 0 || size_y == 0) return ZJOLT_RESULT_OK;

  const JPH::HeightFieldShape *hf = AsHeightField(shape);
  if (hf == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this shape is not a height field");
  }

  const ZJoltResult bounds =
      CheckMaterialBlock(hf, x, y, size_x, size_y, stride);
  if (bounds != ZJOLT_RESULT_OK) return bounds;

  hf->GetMaterials(x, y, size_x, size_y, out_materials,
                   static_cast<intptr_t>(stride));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeHeightFieldSetMaterials(
    ZJoltShape *shape, uint32_t x, uint32_t y, uint32_t size_x,
    uint32_t size_y, const uint8_t *materials, uint32_t stride,
    const ZJoltPhysicsMaterial *const *material_list, uint32_t num_materials) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, materials)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (size_x == 0 || size_y == 0) return ZJOLT_RESULT_OK;

  JPH::HeightFieldShape *hf = AsMutableHeightField(shape);
  if (hf == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this shape is not a height field");
  }

  const ZJoltResult bounds =
      CheckMaterialBlock(hf, x, y, size_x, size_y, stride);
  if (bounds != ZJOLT_RESULT_OK) return bounds;

  JPH::PhysicsMaterialList list;
  const ZJoltResult list_ok =
      BuildMaterialList(material_list, num_materials, list);
  if (list_ok != ZJOLT_RESULT_OK) return list_ok;

  JPH::TempAllocatorMalloc allocator;
  if (!hf->SetMaterials(x, y, size_x, size_y, materials,
                        static_cast<intptr_t>(stride),
                        material_list != nullptr ? &list : nullptr,
                        allocator)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "merging the given materials would take this height "
                           "field past 256, the width of one material index");
  }
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
