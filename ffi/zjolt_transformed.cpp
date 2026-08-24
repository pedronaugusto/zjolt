//===----------------------------------------------------------------------===//
// zjolt — a shape, placed in the world, queried on its own.
//
// Every query here is the same collector-based traversal zjolt_query.cpp
// uses, over JPH::TransformedShape's own CastRay/CollideShape/CastShape/
// CollidePoint instead of NarrowPhaseQuery's. The generic collector is
// reimplemented rather than shared: zjolt_query.cpp's lives in an anonymous
// namespace, invisible outside that translation unit, and this file needs
// only two of its three sinks — there is no *Each form here, see
// zjolt_transformed.h for why a caller does not lose anything by that.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"
#include "zjolt_shape.h"
#include "zjolt_transformed.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

#include <cfloat>

/// The concrete handle. Declared only here: every other translation unit
/// that needs one reaches it through zjoltTransformedShapeCreate and treats
/// the result as opaque, so nothing else needs to see JPH::TransformedShape's
/// shape.
struct ZJoltTransformedShape {
  JPH::TransformedShape impl;
};

namespace {

//===----------------------------------------------------------------------===//
// A smaller HitStream than zjolt_query.cpp's: Closest and All only.
//===----------------------------------------------------------------------===//

template <class Base, class CHit, class Project, class Sink>
class HitStream final : public Base {
 public:
  using JoltHit = typename Base::ResultType;

  HitStream(Project project, Sink sink)
      : project_(std::move(project)), sink_(std::move(sink)) {}

  void AddHit(const JoltHit &jolt_hit) override {
    CHit hit;
    project_(&hit, jolt_hit);

    switch (sink_(hit, jolt_hit)) {
      case ZJOLT_HIT_ACTION_STOP:
        this->ForceEarlyOut();
        break;
      case ZJOLT_HIT_ACTION_NARROW: {
        // @see zjolt_query.cpp's HitStream for why this guard is load-bearing
        // rather than decorative.
        const float fraction = jolt_hit.GetEarlyOutFraction();
        if (fraction < this->GetEarlyOutFraction())
          this->UpdateEarlyOutFraction(fraction);
        break;
      }
      default:
        break;
    }
  }

  const Sink &sink() const { return sink_; }

 private:
  Project project_;
  Sink sink_;
};

template <class Base, class CHit, class Project, class Sink>
HitStream<Base, CHit, Project, Sink> MakeStream(Project project, Sink sink) {
  return HitStream<Base, CHit, Project, Sink>(std::move(project),
                                              std::move(sink));
}

template <class CHit>
struct KeepBest {
  CHit *out;
  bool *had_hit;
  float best = FLT_MAX;

  template <class JoltHit>
  ZJoltHitAction operator()(const CHit &hit, const JoltHit &jolt_hit) {
    const float fraction = jolt_hit.GetEarlyOutFraction();
    if (!*had_hit || fraction < best) {
      *out = hit;
      *had_hit = true;
      best = fraction;
    }
    return ZJOLT_HIT_ACTION_NARROW;
  }
};

template <class CHit>
struct FillBuffer {
  CHit *out;
  uint32_t capacity;
  uint32_t count = 0;

  template <class JoltHit>
  ZJoltHitAction operator()(const CHit &hit, const JoltHit &) {
    if (out != nullptr && count < capacity) out[count] = hit;
    ++count;
    return ZJOLT_HIT_ACTION_CONTINUE;
  }
};

//===----------------------------------------------------------------------===//
// Projections. Unlike zjolt_query.cpp's, these close over `ts` directly
// rather than reading it back from the collector's context: the caller
// already handed it to us, so there is nothing to recover.
//===----------------------------------------------------------------------===//

struct ProjectRayHit {
  const JPH::TransformedShape *ts;
  JPH::RRayCast ray;

  void operator()(ZJoltRayCastHit *out, const JPH::RayCastResult &hit) const {
    out->body = zjolt::ToC(hit.mBodyID);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->fraction = hit.mFraction;
    out->normal = zjolt::ToC(ts->GetWorldSpaceSurfaceNormal(
        hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction)));
    out->material = zjolt::ToC(ts->GetMaterial(hit.mSubShapeID2));
  }
};

struct ProjectShapeCastHit {
  const JPH::TransformedShape *ts;

  void operator()(ZJoltShapeCastHit *out, const JPH::ShapeCastResult &hit) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->fraction = hit.mFraction;
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->is_back_face_hit = hit.mIsBackFaceHit;
    out->material = zjolt::ToC(ts->GetMaterial(hit.mSubShapeID2));
  }
};

struct ProjectCollideHit {
  const JPH::TransformedShape *ts;

  void operator()(ZJoltCollideShapeHit *out,
                  const JPH::CollideShapeResult &hit) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->material = zjolt::ToC(ts->GetMaterial(hit.mSubShapeID2));
  }
};

struct ProjectPointHit {
  const JPH::TransformedShape *ts;

  void operator()(ZJoltCollidePointHit *out,
                  const JPH::CollidePointResult &hit) const {
    out->body = zjolt::ToC(hit.mBodyID);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->material = zjolt::ToC(ts->GetMaterial(hit.mSubShapeID2));
  }
};

//===----------------------------------------------------------------------===//
// Argument plumbing
//===----------------------------------------------------------------------===//

JPH::EBackFaceMode ToJoltBackFaceMode(ZJoltBackFaceMode mode) {
  return mode == ZJOLT_BACK_FACE_MODE_COLLIDE
             ? JPH::EBackFaceMode::CollideWithBackFaces
             : JPH::EBackFaceMode::IgnoreBackFaces;
}

JPH::RayCastSettings MakeRayCastSettings(const ZJoltRayCastSettings *settings) {
  JPH::RayCastSettings out;
  if (settings == nullptr) return out;
  out.mBackFaceModeTriangles =
      ToJoltBackFaceMode(settings->back_face_mode_triangles);
  out.mBackFaceModeConvex = ToJoltBackFaceMode(settings->back_face_mode_convex);
  out.mTreatConvexAsSolid = settings->treat_convex_as_solid;
  return out;
}

zjolt::ShapeFilterAdapter MakeShapeFilter(const ZJoltShapeFilter *filter) {
  return zjolt::ShapeFilterAdapter(filter != nullptr ? *filter
                                                      : ZJoltShapeFilter{});
}

ZJoltResult ReportCount(uint32_t count, const void *out_hits, uint32_t capacity,
                        uint32_t *out_count) {
  *out_count = count;
  if (out_hits == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  return ZJOLT_RESULT_OK;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Lifetime
//===----------------------------------------------------------------------===//

ZJoltResult zjoltTransformedShapeCreate(const ZJoltShape *shape,
                                        const ZJoltRVec3 *position,
                                        const ZJoltQuat *rotation,
                                        const ZJoltVec3 *scale,
                                        ZJoltBodyId body,
                                        ZJoltTransformedShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(shape, position, rotation, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  ZJoltTransformedShape *ts = zjolt::New<ZJoltTransformedShape>();
  if (ts == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  ts->impl = JPH::TransformedShape(zjolt::ToJoltR(*position),
                                   zjolt::ToJoltRotation(*rotation),
                                   zjolt::ToJolt(shape), zjolt::ToJolt(body));
  ts->impl.SetShapeScale(scale != nullptr ? zjolt::ToJolt(*scale)
                                          : JPH::Vec3::sOne());

  zjolt::HandleCreated();
  *out = ts;
  return ZJOLT_RESULT_OK;
}

void zjoltTransformedShapeDestroy(ZJoltTransformedShape *ts) {
  if (ts == nullptr) return;
  zjolt::Delete(ts);
  zjolt::HandleDestroyed();
}

//===----------------------------------------------------------------------===//
// Placement and geometry
//===----------------------------------------------------------------------===//

void zjoltTransformedShapeGetWorldTransform(
    const ZJoltTransformedShape *ts, ZJoltTransformedShapeTransform *out) {
  if (out == nullptr) return;
  if (ts == nullptr) {
    *out = ZJoltTransformedShapeTransform{};
    return;
  }
  const JPH::RMat44 transform = ts->impl.GetWorldTransform();
  out->position = zjolt::ToCR(transform.GetTranslation());
  out->rotation = zjolt::ToC(transform.GetQuaternion());
  out->scale = zjolt::ToC(ts->impl.GetShapeScale());
}

void zjoltTransformedShapeSetWorldTransform(ZJoltTransformedShape *ts,
                                            const ZJoltRVec3 *position,
                                            const ZJoltQuat *rotation,
                                            const ZJoltVec3 *scale) {
  if (ts == nullptr || position == nullptr || rotation == nullptr) return;
  ts->impl.SetWorldTransform(
      zjolt::ToJoltR(*position), zjolt::ToJoltRotation(*rotation),
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne());
}

void zjoltTransformedShapeGetWorldSpaceBounds(const ZJoltTransformedShape *ts,
                                              ZJoltAABox *out) {
  if (out == nullptr) return;
  if (ts == nullptr) {
    *out = ZJoltAABox{};
    return;
  }
  const JPH::AABox bounds = ts->impl.GetWorldSpaceBounds();
  out->min = zjolt::ToC(bounds.mMin);
  out->max = zjolt::ToC(bounds.mMax);
}

void zjoltTransformedShapeGetWorldSpaceSurfaceNormal(
    const ZJoltTransformedShape *ts, ZJoltSubShapeId sub_shape_id,
    const ZJoltRVec3 *position, ZJoltVec3 *out_normal) {
  if (out_normal == nullptr) return;
  if (ts == nullptr || position == nullptr) {
    *out_normal = ZJoltVec3{0.0f, 0.0f, 0.0f};
    return;
  }
  *out_normal = zjolt::ToC(ts->impl.GetWorldSpaceSurfaceNormal(
      zjolt::ToJoltSubShapeId(sub_shape_id), zjolt::ToJoltR(*position)));
}

const ZJoltPhysicsMaterial *zjoltTransformedShapeGetMaterial(
    const ZJoltTransformedShape *ts, ZJoltSubShapeId sub_shape_id) {
  if (ts == nullptr) return nullptr;
  return zjolt::ToC(
      ts->impl.GetMaterial(zjolt::ToJoltSubShapeId(sub_shape_id)));
}

uint64_t zjoltTransformedShapeGetSubShapeUserData(
    const ZJoltTransformedShape *ts, ZJoltSubShapeId sub_shape_id) {
  if (ts == nullptr) return 0;
  return ts->impl.GetSubShapeUserData(zjolt::ToJoltSubShapeId(sub_shape_id));
}

ZJoltResult zjoltTransformedShapeGetSupportingFace(
    const ZJoltTransformedShape *ts, ZJoltSubShapeId sub_shape_id,
    const ZJoltVec3 *direction, const ZJoltRVec3 *base_offset,
    ZJoltVec3 *out_vertices, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(ts, direction, base_offset, out_vertices, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Shape::SupportingFace face;
  ts->impl.GetSupportingFace(zjolt::ToJoltSubShapeId(sub_shape_id),
                             zjolt::ToJolt(*direction),
                             zjolt::ToJoltR(*base_offset), face);

  for (JPH::uint i = 0; i < face.size(); ++i)
    out_vertices[i] = zjolt::ToC(face[i]);
  *out_count = static_cast<uint32_t>(face.size());
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Triangle read-back
//===----------------------------------------------------------------------===//

namespace {

JPH::Shape::GetTrianglesContext &AsJoltContext(
    ZJoltTransformedShapeTrianglesContext *context) {
  return *reinterpret_cast<JPH::Shape::GetTrianglesContext *>(context);
}

static_assert(
    sizeof(ZJoltTransformedShapeTrianglesContext) ==
        sizeof(JPH::Shape::GetTrianglesContext),
    "ZJoltTransformedShapeTrianglesContext must match Shape::GetTrianglesContext");
static_assert(
    alignof(ZJoltTransformedShapeTrianglesContext) ==
        alignof(JPH::Shape::GetTrianglesContext),
    "ZJoltTransformedShapeTrianglesContext must match Shape::GetTrianglesContext");

}  // namespace

ZJoltResult zjoltTransformedShapeGetTrianglesStart(
    const ZJoltTransformedShape *ts,
    ZJoltTransformedShapeTrianglesContext *context, const ZJoltAABox *box,
    const ZJoltRVec3 *base_offset) {
  ZJOLT_ENTER();
  if (!zjolt::Present(ts, context, box, base_offset))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::AABox jolt_box(zjolt::ToJolt(box->min), zjolt::ToJolt(box->max));
  ts->impl.GetTrianglesStart(AsJoltContext(context), jolt_box,
                             zjolt::ToJoltR(*base_offset));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltTransformedShapeGetTrianglesNext(
    const ZJoltTransformedShape *ts,
    ZJoltTransformedShapeTrianglesContext *context, uint32_t max_triangles,
    ZJoltVec3 *out_vertices, const ZJoltPhysicsMaterial **out_materials,
    uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(ts, context, out_vertices, out_count))
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

  const int found = ts->impl.GetTrianglesNext(
      AsJoltContext(context), static_cast<int>(max_triangles), vertices,
      materials);
  *out_count = static_cast<uint32_t>(found);

  if (out_materials != nullptr) {
    for (int i = 0; i < found; ++i)
      out_materials[i] = zjolt::ToC(material_storage[static_cast<size_t>(i)]);
  }
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Queries against this one shape
//===----------------------------------------------------------------------===//

ZJoltResult zjoltTransformedShapeCastRayClosest(
    const ZJoltTransformedShape *ts, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltRayCastSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltRayCastHit *out_hit,
    bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(ts, origin, direction, out_hit, out_hit_any))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));
  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);

  bool had_hit = false;
  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{&ts->impl, ray}, KeepBest<ZJoltRayCastHit>{out_hit, &had_hit});
  ts->impl.CastRay(ray, MakeRayCastSettings(settings), collector, shape_filter);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltTransformedShapeCastRayAll(
    const ZJoltTransformedShape *ts, const ZJoltRVec3 *origin,
    const ZJoltVec3 *direction, const ZJoltRayCastSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltRayCastHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(ts, origin, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));
  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);

  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{&ts->impl, ray}, FillBuffer<ZJoltRayCastHit>{out_hits, capacity});
  ts->impl.CastRay(ray, MakeRayCastSettings(settings), collector, shape_filter);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltTransformedShapeCollidePointAll(
    const ZJoltTransformedShape *ts, const ZJoltRVec3 *point,
    const ZJoltShapeFilter *filter, ZJoltCollidePointHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(ts, point, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  auto collector = MakeStream<JPH::CollidePointCollector, ZJoltCollidePointHit>(
      ProjectPointHit{&ts->impl},
      FillBuffer<ZJoltCollidePointHit>{out_hits, capacity});
  ts->impl.CollidePoint(zjolt::ToJoltR(*point), collector, shape_filter);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltTransformedShapeCollideShapeAll(
    const ZJoltTransformedShape *ts, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, float max_separation_distance,
    const ZJoltRVec3 *base_offset, const ZJoltShapeFilter *filter,
    ZJoltCollideShapeHit *out_hits, uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(ts, shape, position, rotation, base_offset, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
  const JPH::RMat44 transform = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(*rotation), zjolt::ToJoltR(*position));

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = max_separation_distance;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{&ts->impl},
      FillBuffer<ZJoltCollideShapeHit>{out_hits, capacity});
  ts->impl.CollideShape(zjolt::ToJolt(shape), shape_scale, transform, settings,
                        zjolt::ToJoltR(*base_offset), collector, shape_filter);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltTransformedShapeCastShapeClosest(
    const ZJoltTransformedShape *ts, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltRVec3 *base_offset, const ZJoltShapeFilter *filter,
    ZJoltShapeCastHit *out_hit, bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(ts, shape, position, rotation, direction, base_offset,
                      out_hit, out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
  const JPH::RMat44 transform = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(*rotation), zjolt::ToJoltR(*position));
  const JPH::RShapeCast cast = JPH::RShapeCast::sFromWorldTransform(
      zjolt::ToJolt(shape), shape_scale, transform, zjolt::ToJolt(*direction));

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  bool had_hit = false;
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{&ts->impl},
      KeepBest<ZJoltShapeCastHit>{out_hit, &had_hit});
  ts->impl.CastShape(cast, JPH::ShapeCastSettings(),
                     zjolt::ToJoltR(*base_offset), collector, shape_filter);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltTransformedShapeCastShapeAll(
    const ZJoltTransformedShape *ts, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltRVec3 *base_offset, const ZJoltShapeFilter *filter,
    ZJoltShapeCastHit *out_hits, uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(ts, shape, position, rotation, direction, base_offset,
                      out_count)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
  const JPH::RMat44 transform = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(*rotation), zjolt::ToJoltR(*position));
  const JPH::RShapeCast cast = JPH::RShapeCast::sFromWorldTransform(
      zjolt::ToJolt(shape), shape_scale, transform, zjolt::ToJolt(*direction));

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{&ts->impl},
      FillBuffer<ZJoltShapeCastHit>{out_hits, capacity});
  ts->impl.CastShape(cast, JPH::ShapeCastSettings(),
                     zjolt::ToJoltR(*base_offset), collector, shape_filter);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

}  // extern "C"
