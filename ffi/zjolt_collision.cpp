//===----------------------------------------------------------------------===//
// zjolt — triangle collision, active-edge normals, and internal-edge removal.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"
#include "zjolt_query_internal.h"

#include <Jolt/Physics/Collision/ActiveEdges.h>
#include <Jolt/Physics/Collision/CastConvexVsTriangles.h>
#include <Jolt/Physics/Collision/CastSphereVsTriangles.h>
#include <Jolt/Physics/Collision/CollideConvexVsTriangles.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollideSphereVsTriangles.h>
#include <Jolt/Physics/Collision/InternalEdgeRemovingCollector.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>

namespace {

//===----------------------------------------------------------------------===//
// Placement — the shared math every entry point below builds transforms with.
//===----------------------------------------------------------------------===//

JPH::Vec3 ScaleOr1(const ZJoltVec3 *scale) {
  return scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
}

JPH::RVec3 OffsetOrZero(const ZJoltRVec3 *offset) {
  return offset != nullptr ? zjolt::ToJoltR(*offset) : JPH::RVec3::sZero();
}

/// The centre-of-mass transform of a real `JPH::Shape`, in `base_offset`'s
/// frame -- the same fold `zjolt_query.cpp`'s `ToDispatchSpace` does.
JPH::Mat44 ShapeComTransform(const JPH::Shape *shape, JPH::Vec3Arg scale,
                             const ZJoltRVec3 &position, const ZJoltQuat &rotation,
                             JPH::RVec3Arg base_offset) {
  const JPH::RMat44 world = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(rotation), zjolt::ToJoltR(position));
  return world.PreTranslated(scale * shape->GetCenterOfMass())
      .PostTranslated(-base_offset)
      .ToMat44();
}

/// As `ShapeComTransform`, for a triangle source that is data rather than a
/// `ZJoltShape` -- there is no centre of mass to fold in.
JPH::Mat44 PlacementTransform(const ZJoltRVec3 &position, const ZJoltQuat &rotation,
                              JPH::RVec3Arg base_offset) {
  const JPH::RMat44 world = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(rotation), zjolt::ToJoltR(position));
  return world.PostTranslated(-base_offset).ToMat44();
}

/// The `JPH::ShapeCast` a cast against triangles needs, in `base_offset`'s
/// frame -- the same construction `zjolt_query.cpp`'s `MakeDispatchShapeCast`
/// uses for shape-versus-shape.
JPH::ShapeCast MakeCast(const JPH::Shape *shape1, JPH::Vec3Arg scale1,
                        const ZJoltRVec3 &position1, const ZJoltQuat &rotation1,
                        const ZJoltVec3 &direction, JPH::RVec3Arg base_offset) {
  const JPH::RMat44 world1 = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(rotation1), zjolt::ToJoltR(position1));
  const JPH::RShapeCast world_cast =
      JPH::RShapeCast::sFromWorldTransform(shape1, scale1, world1, zjolt::ToJolt(direction));
  return JPH::ShapeCast(world_cast.PostTranslated(-base_offset));
}

//===----------------------------------------------------------------------===//
// Collectors that forward straight to a host callback -- the smaller cousin
// of `zjolt_query.cpp`'s `HitStream`. There is exactly one sink here (a host
// callback) and no closest/buffered forms, so the extra seams that file needs
// to share a traversal across three shapes would only be indirection.
//===----------------------------------------------------------------------===//

/// `material_shape`, when not NULL, resolves `hit->material` off it; NULL
/// leaves it NULL, which every triangle-collision entry point below wants --
/// there is no `ZJoltShape` on the triangle side to resolve one from.
class ForwardCollideHit final : public JPH::CollideShapeCollector {
 public:
  ForwardCollideHit(ZJoltCollideShapeHitFn on_hit, void *user,
                    const JPH::Shape *material_shape = nullptr)
      : on_hit_(on_hit), user_(user), material_shape_(material_shape) {}

  void AddHit(const JPH::CollideShapeResult &hit) override {
    // A host that ignores *out_should_stop and keeps feeding triangles must
    // not see a hit reported past STOP -- "hits already reported stand" is a
    // promise about what has already reached on_hit, not about what a
    // slow-to-notice caller might still trigger.
    if (ShouldEarlyOut()) return;

    ZJoltCollideShapeHit c_hit{};
    c_hit.body = zjolt::ToC(hit.mBodyID2);
    c_hit.sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    c_hit.contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    c_hit.contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    c_hit.penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    c_hit.penetration_depth = hit.mPenetrationDepth;
    c_hit.material = material_shape_ != nullptr
                         ? zjolt::ToC(material_shape_->GetMaterial(hit.mSubShapeID2))
                         : nullptr;

    switch (on_hit_(user_, &c_hit)) {
      case ZJOLT_HIT_ACTION_STOP:
        ForceEarlyOut();
        break;
      case ZJOLT_HIT_ACTION_NARROW: {
        const float fraction = hit.GetEarlyOutFraction();
        if (fraction < GetEarlyOutFraction()) UpdateEarlyOutFraction(fraction);
        break;
      }
      default:
        break;
    }
  }

 private:
  ZJoltCollideShapeHitFn on_hit_;
  void *user_;
  const JPH::Shape *material_shape_;
};

/// @see ForwardCollideHit. `material` is always NULL here: every caller is a
/// triangle source with no `ZJoltShape` of its own.
class ForwardCastHit final : public JPH::CastShapeCollector {
 public:
  ForwardCastHit(ZJoltShapeCastHitFn on_hit, void *user) : on_hit_(on_hit), user_(user) {}

  void AddHit(const JPH::ShapeCastResult &hit) override {
    if (ShouldEarlyOut()) return;

    ZJoltShapeCastHit c_hit{};
    c_hit.body = zjolt::ToC(hit.mBodyID2);
    c_hit.sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    c_hit.fraction = hit.mFraction;
    c_hit.contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    c_hit.contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    c_hit.penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    c_hit.penetration_depth = hit.mPenetrationDepth;
    c_hit.is_back_face_hit = hit.mIsBackFaceHit;
    c_hit.material = nullptr;

    switch (on_hit_(user_, &c_hit)) {
      case ZJOLT_HIT_ACTION_STOP:
        ForceEarlyOut();
        break;
      case ZJOLT_HIT_ACTION_NARROW: {
        const float fraction = hit.GetEarlyOutFraction();
        if (fraction < GetEarlyOutFraction()) UpdateEarlyOutFraction(fraction);
        break;
      }
      default:
        break;
    }
  }

 private:
  ZJoltShapeCastHitFn on_hit_;
  void *user_;
};

}  // namespace

//===----------------------------------------------------------------------===//
// Opaque handles (global namespace: must match the C tag names). Jolt's
// "vs triangles" helpers store constructor arguments BY REFERENCE, valid
// only for Jolt's own stack-local use within one call frame; each referent
// here is a co-located member, declared ahead of `impl` so it constructs first.
//===----------------------------------------------------------------------===//

struct ZJoltCollideConvexVsTriangles {
  JPH::CollideShapeSettings settings;
  ForwardCollideHit collector;
  JPH::CollideConvexVsTriangles impl;

  ZJoltCollideConvexVsTriangles(JPH::CollideShapeSettings settings_in,
                                ZJoltCollideShapeHitFn on_hit, void *user,
                                const JPH::ConvexShape *shape1, JPH::Vec3Arg scale1,
                                JPH::Vec3Arg scale2, JPH::Mat44Arg transform1,
                                JPH::Mat44Arg transform2, const JPH::SubShapeID &sub_shape_id1)
      : settings(settings_in),
        collector(on_hit, user),
        impl(shape1, scale1, scale2, transform1, transform2, sub_shape_id1, settings, collector) {}
};

struct ZJoltCollideSphereVsTriangles {
  JPH::CollideShapeSettings settings;
  ForwardCollideHit collector;
  JPH::CollideSphereVsTriangles impl;

  ZJoltCollideSphereVsTriangles(JPH::CollideShapeSettings settings_in,
                                ZJoltCollideShapeHitFn on_hit, void *user,
                                const JPH::SphereShape *shape1, JPH::Vec3Arg scale1,
                                JPH::Vec3Arg scale2, JPH::Mat44Arg transform1,
                                JPH::Mat44Arg transform2, const JPH::SubShapeID &sub_shape_id1)
      : settings(settings_in),
        collector(on_hit, user),
        impl(shape1, scale1, scale2, transform1, transform2, sub_shape_id1, settings, collector) {}
};

struct ZJoltCastConvexVsTriangles {
  JPH::ShapeCast shape_cast;
  JPH::ShapeCastSettings settings;
  JPH::Mat44 transform2;
  ForwardCastHit collector;
  JPH::CastConvexVsTriangles impl;

  ZJoltCastConvexVsTriangles(JPH::ShapeCast shape_cast_in, JPH::ShapeCastSettings settings_in,
                             JPH::Mat44Arg transform2_in, JPH::Vec3Arg scale2,
                             ZJoltShapeCastHitFn on_hit, void *user)
      : shape_cast(shape_cast_in),
        settings(settings_in),
        transform2(transform2_in),
        collector(on_hit, user),
        impl(shape_cast, settings, scale2, transform2, JPH::SubShapeIDCreator(), collector) {}
};

struct ZJoltCastSphereVsTriangles {
  JPH::ShapeCastSettings settings;
  JPH::Mat44 transform2;
  ForwardCastHit collector;
  JPH::CastSphereVsTriangles impl;

  ZJoltCastSphereVsTriangles(const JPH::ShapeCast &shape_cast, JPH::ShapeCastSettings settings_in,
                             JPH::Mat44Arg transform2_in, JPH::Vec3Arg scale2,
                             ZJoltShapeCastHitFn on_hit, void *user)
      : settings(settings_in),
        transform2(transform2_in),
        collector(on_hit, user),
        impl(shape_cast, settings, scale2, transform2, JPH::SubShapeIDCreator(), collector) {}
};

extern "C" {

//===----------------------------------------------------------------------===//
// Active edges
//===----------------------------------------------------------------------===//

bool zjoltActiveEdgesIsEdgeActive(const ZJoltVec3 *normal1, const ZJoltVec3 *normal2,
                                  const ZJoltVec3 *edge_direction, float cos_threshold_angle) {
  if (!zjolt::Present(normal1, normal2, edge_direction)) return false;
  return JPH::ActiveEdges::IsEdgeActive(zjolt::ToJolt(*normal1), zjolt::ToJolt(*normal2),
                                        zjolt::ToJolt(*edge_direction), cos_threshold_angle);
}

void zjoltActiveEdgesFixNormal(const ZJoltVec3 *v0, const ZJoltVec3 *v1, const ZJoltVec3 *v2,
                               const ZJoltVec3 *triangle_normal, uint8_t active_edges,
                               const ZJoltVec3 *point, const ZJoltVec3 *normal,
                               const ZJoltVec3 *movement_direction, ZJoltVec3 *out_normal) {
  if (out_normal == nullptr) return;
  *out_normal = ZJoltVec3{0.0f, 0.0f, 0.0f};
  if (!zjolt::Present(v0, v1, v2, triangle_normal, point, normal, movement_direction)) return;

  const uint8_t edges = active_edges & 0b111u;
  // JPH::ActiveEdges::FixNormal asserts when active_edges == 0b111 (all
  // edges active, `normal` already correct); that case is answered directly
  // instead of reaching the assert.
  if (edges == 0b111u) {
    *out_normal = *normal;
    return;
  }

  const JPH::Vec3 result = JPH::ActiveEdges::FixNormal(
      zjolt::ToJolt(*v0), zjolt::ToJolt(*v1), zjolt::ToJolt(*v2), zjolt::ToJolt(*triangle_normal),
      edges, zjolt::ToJolt(*point), zjolt::ToJolt(*normal), zjolt::ToJolt(*movement_direction));
  *out_normal = zjolt::ToC(result);
}

//===----------------------------------------------------------------------===//
// Triangle collision
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollideConvexVsTrianglesCreate(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1, const ZJoltRVec3 *position2,
    const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset, ZJoltSubShapeId sub_shape_id1,
    const ZJoltCollideShapeSettings *settings, ZJoltCollideShapeHitFn on_hit, void *user,
    ZJoltCollideConvexVsTriangles **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape1, position1, rotation1, position2, rotation2, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "on_hit is NULL: a collider has nowhere to report hits");
  }

  const JPH::Shape *impl1 = zjolt::ToJolt(shape1);
  if (impl1->GetType() != JPH::EShapeType::Convex) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "shape1 is not a convex shape");
  }

  const JPH::CollideShapeSettings jolt_settings = zjolt::MakeCollideShapeSettings(settings);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  const JPH::Vec3 s1 = ScaleOr1(scale1);
  const JPH::Vec3 s2 = ScaleOr1(scale2);
  const JPH::RVec3 offset = OffsetOrZero(base_offset);
  const JPH::Mat44 transform1 = ShapeComTransform(impl1, s1, *position1, *rotation1, offset);
  const JPH::Mat44 transform2 = PlacementTransform(*position2, *rotation2, offset);

  ZJoltCollideConvexVsTriangles *handle = zjolt::New<ZJoltCollideConvexVsTriangles>(
      jolt_settings, on_hit, user, static_cast<const JPH::ConvexShape *>(impl1), s1, s2,
      transform1, transform2, zjolt::ToJoltSubShapeId(sub_shape_id1));
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCollideConvexVsTrianglesCollide(ZJoltCollideConvexVsTriangles *collider,
                                                 const ZJoltVec3 *v0, const ZJoltVec3 *v1,
                                                 const ZJoltVec3 *v2, uint8_t active_edges,
                                                 ZJoltSubShapeId sub_shape_id2,
                                                 bool *out_should_stop) {
  ZJOLT_ENTER(out_should_stop);
  if (!zjolt::Present(collider, v0, v1, v2, out_should_stop))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (!collider->collector.ShouldEarlyOut()) {
    collider->impl.Collide(zjolt::ToJolt(*v0), zjolt::ToJolt(*v1), zjolt::ToJolt(*v2),
                           active_edges & 0b111u, zjolt::ToJoltSubShapeId(sub_shape_id2));
  }
  *out_should_stop = collider->collector.ShouldEarlyOut();
  return ZJOLT_RESULT_OK;
}

void zjoltCollideConvexVsTrianglesDestroy(ZJoltCollideConvexVsTriangles *collider) {
  if (collider == nullptr) return;
  zjolt::Delete(collider);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltCastConvexVsTrianglesCreate(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1, const ZJoltRVec3 *position1,
    const ZJoltQuat *rotation1, const ZJoltVec3 *direction, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltShapeCastSettings *settings, ZJoltShapeCastHitFn on_hit, void *user,
    ZJoltCastConvexVsTriangles **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape1, position1, rotation1, direction, position2, rotation2, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "on_hit is NULL: a caster has nowhere to report hits");
  }

  const JPH::Shape *impl1 = zjolt::ToJolt(shape1);
  if (impl1->GetType() != JPH::EShapeType::Convex) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "shape1 is not a convex shape");
  }

  const JPH::ShapeCastSettings jolt_settings = zjolt::MakeShapeCastSettings(settings);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  const JPH::Vec3 s1 = ScaleOr1(scale1);
  const JPH::Vec3 s2 = ScaleOr1(scale2);
  const JPH::RVec3 offset = OffsetOrZero(base_offset);
  const JPH::ShapeCast cast = MakeCast(impl1, s1, *position1, *rotation1, *direction, offset);
  const JPH::Mat44 transform2 = PlacementTransform(*position2, *rotation2, offset);

  ZJoltCastConvexVsTriangles *handle = zjolt::New<ZJoltCastConvexVsTriangles>(
      cast, jolt_settings, transform2, s2, on_hit, user);
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastConvexVsTrianglesCast(ZJoltCastConvexVsTriangles *caster,
                                           const ZJoltVec3 *v0, const ZJoltVec3 *v1,
                                           const ZJoltVec3 *v2, uint8_t active_edges,
                                           ZJoltSubShapeId sub_shape_id2,
                                           bool *out_should_stop) {
  ZJOLT_ENTER(out_should_stop);
  if (!zjolt::Present(caster, v0, v1, v2, out_should_stop))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (!caster->collector.ShouldEarlyOut()) {
    caster->impl.Cast(zjolt::ToJolt(*v0), zjolt::ToJolt(*v1), zjolt::ToJolt(*v2),
                      active_edges & 0b111u, zjolt::ToJoltSubShapeId(sub_shape_id2));
  }
  *out_should_stop = caster->collector.ShouldEarlyOut();
  return ZJOLT_RESULT_OK;
}

void zjoltCastConvexVsTrianglesDestroy(ZJoltCastConvexVsTriangles *caster) {
  if (caster == nullptr) return;
  zjolt::Delete(caster);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltCollideSphereVsTrianglesCreate(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1, const ZJoltRVec3 *position2,
    const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset, ZJoltSubShapeId sub_shape_id1,
    const ZJoltCollideShapeSettings *settings, ZJoltCollideShapeHitFn on_hit, void *user,
    ZJoltCollideSphereVsTriangles **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape1, position1, rotation1, position2, rotation2, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "on_hit is NULL: a collider has nowhere to report hits");
  }

  const JPH::Shape *impl1 = zjolt::ToJolt(shape1);
  if (impl1->GetSubType() != JPH::EShapeSubType::Sphere) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "shape1 is not a sphere");
  }

  const JPH::CollideShapeSettings jolt_settings = zjolt::MakeCollideShapeSettings(settings);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  const JPH::Vec3 s1 = ScaleOr1(scale1);
  const JPH::Vec3 s2 = ScaleOr1(scale2);
  const JPH::RVec3 offset = OffsetOrZero(base_offset);
  const JPH::Mat44 transform1 = ShapeComTransform(impl1, s1, *position1, *rotation1, offset);
  const JPH::Mat44 transform2 = PlacementTransform(*position2, *rotation2, offset);

  ZJoltCollideSphereVsTriangles *handle = zjolt::New<ZJoltCollideSphereVsTriangles>(
      jolt_settings, on_hit, user, static_cast<const JPH::SphereShape *>(impl1), s1, s2,
      transform1, transform2, zjolt::ToJoltSubShapeId(sub_shape_id1));
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCollideSphereVsTrianglesCollide(ZJoltCollideSphereVsTriangles *collider,
                                                 const ZJoltVec3 *v0, const ZJoltVec3 *v1,
                                                 const ZJoltVec3 *v2, uint8_t active_edges,
                                                 ZJoltSubShapeId sub_shape_id2,
                                                 bool *out_should_stop) {
  ZJOLT_ENTER(out_should_stop);
  if (!zjolt::Present(collider, v0, v1, v2, out_should_stop))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (!collider->collector.ShouldEarlyOut()) {
    collider->impl.Collide(zjolt::ToJolt(*v0), zjolt::ToJolt(*v1), zjolt::ToJolt(*v2),
                           active_edges & 0b111u, zjolt::ToJoltSubShapeId(sub_shape_id2));
  }
  *out_should_stop = collider->collector.ShouldEarlyOut();
  return ZJOLT_RESULT_OK;
}

void zjoltCollideSphereVsTrianglesDestroy(ZJoltCollideSphereVsTriangles *collider) {
  if (collider == nullptr) return;
  zjolt::Delete(collider);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltCastSphereVsTrianglesCreate(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1, const ZJoltRVec3 *position1,
    const ZJoltQuat *rotation1, const ZJoltVec3 *direction, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltShapeCastSettings *settings, ZJoltShapeCastHitFn on_hit, void *user,
    ZJoltCastSphereVsTriangles **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape1, position1, rotation1, direction, position2, rotation2, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "on_hit is NULL: a caster has nowhere to report hits");
  }

  const JPH::Shape *impl1 = zjolt::ToJolt(shape1);
  if (impl1->GetSubType() != JPH::EShapeSubType::Sphere) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "shape1 is not a sphere");
  }

  const JPH::ShapeCastSettings jolt_settings = zjolt::MakeShapeCastSettings(settings);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  const JPH::Vec3 s1 = ScaleOr1(scale1);
  const JPH::Vec3 s2 = ScaleOr1(scale2);
  const JPH::RVec3 offset = OffsetOrZero(base_offset);
  const JPH::ShapeCast cast = MakeCast(impl1, s1, *position1, *rotation1, *direction, offset);
  const JPH::Mat44 transform2 = PlacementTransform(*position2, *rotation2, offset);

  ZJoltCastSphereVsTriangles *handle = zjolt::New<ZJoltCastSphereVsTriangles>(
      cast, jolt_settings, transform2, s2, on_hit, user);
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastSphereVsTrianglesCast(ZJoltCastSphereVsTriangles *caster,
                                           const ZJoltVec3 *v0, const ZJoltVec3 *v1,
                                           const ZJoltVec3 *v2, uint8_t active_edges,
                                           ZJoltSubShapeId sub_shape_id2,
                                           bool *out_should_stop) {
  ZJOLT_ENTER(out_should_stop);
  if (!zjolt::Present(caster, v0, v1, v2, out_should_stop))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (!caster->collector.ShouldEarlyOut()) {
    caster->impl.Cast(zjolt::ToJolt(*v0), zjolt::ToJolt(*v1), zjolt::ToJolt(*v2),
                      active_edges & 0b111u, zjolt::ToJoltSubShapeId(sub_shape_id2));
  }
  *out_should_stop = caster->collector.ShouldEarlyOut();
  return ZJOLT_RESULT_OK;
}

void zjoltCastSphereVsTrianglesDestroy(ZJoltCastSphereVsTriangles *caster) {
  if (caster == nullptr) return;
  zjolt::Delete(caster);
  zjolt::HandleDestroyed();
}

//===----------------------------------------------------------------------===//
// Ghost-collision removal
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollideShapeWithInternalEdgeRemoval(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1, const ZJoltRVec3 *position1,
    const ZJoltQuat *rotation1, const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltCollideShapeSettings *settings, const ZJoltShapeFilter *filter,
    ZJoltCollideShapeHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape1, position1, rotation1, shape2, position2, rotation2))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "on_hit is NULL: a streaming query has nowhere to report hits");
  }

  const JPH::Shape *impl1 = zjolt::ToJolt(shape1);
  const JPH::Shape *impl2 = zjolt::ToJolt(shape2);
  const JPH::Vec3 s1 = ScaleOr1(scale1);
  const JPH::Vec3 s2 = ScaleOr1(scale2);
  if (!impl1->IsValidScale(s1) || !impl2->IsValidScale(s2)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a shape was given a scale it cannot take: a zero component, or a "
        "non-uniform scale on a shape that insists on a uniform one");
  }

  JPH::CollideShapeSettings jolt_settings = zjolt::MakeCollideShapeSettings(settings);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;
  // The two fields JPH::InternalEdgeRemovingCollector requires, forced the
  // same way zjolt_query.cpp's zjoltCollideShapeWithInternalEdgeRemoval*
  // family forces them -- the removal algorithm cannot work without both.
  jolt_settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
  jolt_settings.mCollectFacesMode = JPH::ECollectFacesMode::CollectFaces;

  const JPH::RVec3 offset = OffsetOrZero(base_offset);
  const JPH::Mat44 transform1 = ShapeComTransform(impl1, s1, *position1, *rotation1, offset);
  const JPH::Mat44 transform2 = ShapeComTransform(impl2, s2, *position2, *rotation2, offset);

  zjolt::ShapeFilterAdapter shape_filter(filter != nullptr ? *filter : ZJoltShapeFilter{});
  ForwardCollideHit collector(on_hit, user, impl2);
  JPH::SubShapeIDCreator id1, id2;
  JPH::InternalEdgeRemovingCollector::sCollideShapeVsShape(
      impl1, impl2, s1, s2, transform1, transform2, id1, id2, jolt_settings, collector,
      shape_filter);

  return ZJOLT_RESULT_OK;
}

}  // extern "C"
