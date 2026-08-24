//===----------------------------------------------------------------------===//
// zjolt — ray casts, shape casts, overlap and point tests.
//
// Jolt's query API is built on collectors: an object it calls once per hit, as
// it finds them. Everything in this file is one collector — `HitStream` — with
// two seams. `Project` turns Jolt's hit into the ABI's struct; `Sink` decides
// what becomes of it. The three forms the header offers are three sinks over
// one traversal:
//
//   KeepBest      the closest / deepest hit          -> *Closest
//   FillBuffer    the caller's buffer, and the count -> *All
//   ForwardToHost the host's callback                -> *Each
//
// Doing it that way is not tidiness. The alternative — one of Jolt's own
// collectors per form — is what this file used to be, and it materialised the
// whole result set inside Jolt before copying it out.
// JPH::AllHitCollisionCollector accumulates into an unbounded array, and a
// JPH::CollideShapeResult is around a kilobyte because it embeds two
// StaticArray<Vec3, 32> faces, sized that way even when mCollectFacesMode is
// NoFaces. Measured on an overlap query returning 201 hits: nine allocations
// and 559 KiB through the host's allocator per call, twice over for a
// count-then-fill pair. Every form here now allocates nothing at all, and the
// streaming form answers the same question in one traversal rather than two
// (201 body-filter calls against 402).
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

#include <cfloat>

namespace {

//===----------------------------------------------------------------------===//
// The collector
//===----------------------------------------------------------------------===//

/// The one collector every query in this file runs on.
///
/// `Project` and `Sink` are both stored by value, so a query is a stack object
/// and nothing here allocates.
template <class Base, class CHit, class Project, class Sink>
class HitStream final : public Base {
 public:
  using JoltHit = typename Base::ResultType;

  HitStream(Project project, Sink sink)
      : project_(std::move(project)), sink_(std::move(sink)) {}

  void AddHit(const JoltHit &jolt_hit) override {
    CHit hit;
    project_(&hit, jolt_hit, *this);

    switch (sink_(hit, jolt_hit)) {
      case ZJOLT_HIT_ACTION_STOP:
        this->ForceEarlyOut();
        break;

      case ZJOLT_HIT_ACTION_NARROW: {
        // The guard is the entire reason narrowing is safe to offer.
        // UpdateEarlyOutFraction asserts the fraction never rises
        // (CollisionCollector.h:84), and Jolt does not promise that each hit
        // is better than the last — a convex shape reports its back face
        // AFTER its front face and further along the ray. Jolt's own
        // ClosestHitCollisionCollector guards the same call the same way.
        //
        // With assertions compiled out an increasing fraction is worse than a
        // failed assert: the narrow phase carries the same expectation as a
        // precondition (NarrowPhaseQuery.cpp:103, 333) and simply misbehaves.
        const float fraction = jolt_hit.GetEarlyOutFraction();
        if (fraction < this->GetEarlyOutFraction())
          this->UpdateEarlyOutFraction(fraction);
        break;
      }

      // CONTINUE, and anything a C caller passed that is not in the enum at
      // all. Doing nothing is the only answer that can neither lose a hit nor
      // break a precondition, which is what makes it the right default.
      default:
        break;
    }
  }

  const Sink &sink() const { return sink_; }

 private:
  Project project_;
  Sink sink_;
};

/// `Base` and `CHit` are named, the rest deduced.
template <class Base, class CHit, class Project, class Sink>
HitStream<Base, CHit, Project, Sink> MakeStream(Project project, Sink sink) {
  return HitStream<Base, CHit, Project, Sink>(std::move(project),
                                              std::move(sink));
}

//===----------------------------------------------------------------------===//
// Sinks
//===----------------------------------------------------------------------===//

/// Keeps the single best hit — nearest for a cast, deepest for an overlap —
/// which is what Jolt's early-out fraction already orders hits by.
///
/// This is JPH::ClosestHitCollisionCollector, rewritten as a sink so that the
/// closest-hit form shares the traversal with the other two instead of being a
/// second implementation that can drift.
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

/// Fills the caller's buffer and counts everything, including what did not
/// fit.
///
/// Counting past the capacity is the point: the count is the answer to the
/// size query, and it has to be true even when the buffer could not hold it.
/// That is what collapses the old two-call protocol from two traversals to
/// one.
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

/// Hands the hit to the host and does what it says.
///
/// The host's callback is a plain C function pointer and cannot throw across
/// this boundary; a language whose errors would unwind carries them out in
/// `user` instead. @see the note in zjolt_query.h.
template <class CHit, class Fn>
struct ForwardToHost {
  Fn on_hit;
  void *user;

  template <class JoltHit>
  ZJoltHitAction operator()(const CHit &hit, const JoltHit &) const {
    return on_hit(user, &hit);
  }
};

//===----------------------------------------------------------------------===//
// Projections
//===----------------------------------------------------------------------===//

/// A ray hit, with its surface normal resolved.
///
/// The normal is the reason this projection is handed the collector at all.
/// Jolt does not put it in RayCastResult; reaching it means asking the
/// TransformedShape the collector is holding, and that pointer is valid only
/// during AddHit and is deliberately not cleared afterwards
/// (CollisionCollector.h:73). Resolving it here is what lets the ABI report a
/// normal without exporting a pointer that is stale the moment it is useful.
/// The material at `sub_shape_id` on the shape a collector's context is
/// currently holding, or NULL when there is no context — which does not
/// happen when this is driven through NarrowPhaseQuery, but a defensive
/// default is cheaper than a precondition nobody would remember to check.
const JPH::PhysicsMaterial *ResolveMaterial(const JPH::TransformedShape *shape,
                                            const JPH::SubShapeID &sub_shape_id) {
  return shape != nullptr ? shape->GetMaterial(sub_shape_id) : nullptr;
}

struct ProjectRayHit {
  JPH::RRayCast ray;

  template <class Collector>
  void operator()(ZJoltRayCastHit *out, const JPH::RayCastResult &hit,
                  const Collector &collector) const {
    out->body = zjolt::ToC(hit.mBodyID);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->fraction = hit.mFraction;

    const JPH::TransformedShape *shape = collector.GetContext();
    out->normal = shape != nullptr
                      ? zjolt::ToC(shape->GetWorldSpaceSurfaceNormal(
                            hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction)))
                      : ZJoltVec3{0.0f, 0.0f, 0.0f};
    out->material = zjolt::ToC(ResolveMaterial(shape, hit.mSubShapeID2));
  }
};

struct ProjectShapeCastHit {
  template <class Collector>
  void operator()(ZJoltShapeCastHit *out, const JPH::ShapeCastResult &hit,
                  const Collector &collector) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->fraction = hit.mFraction;
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->is_back_face_hit = hit.mIsBackFaceHit;
    out->material =
        zjolt::ToC(ResolveMaterial(collector.GetContext(), hit.mSubShapeID2));
  }
};

struct ProjectCollideHit {
  template <class Collector>
  void operator()(ZJoltCollideShapeHit *out, const JPH::CollideShapeResult &hit,
                  const Collector &collector) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->material =
        zjolt::ToC(ResolveMaterial(collector.GetContext(), hit.mSubShapeID2));
  }
};

struct ProjectPointHit {
  template <class Collector>
  void operator()(ZJoltCollidePointHit *out, const JPH::CollidePointResult &hit,
                  const Collector &collector) const {
    out->body = zjolt::ToC(hit.mBodyID);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->material =
        zjolt::ToC(ResolveMaterial(collector.GetContext(), hit.mSubShapeID2));
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

/// A NULL ZJoltRayCastSettings means Jolt's own defaults, which is what an
/// untouched RayCastSettings already is.
JPH::RayCastSettings MakeRayCastSettings(const ZJoltRayCastSettings *settings) {
  JPH::RayCastSettings out;
  if (settings == nullptr) return out;
  out.mBackFaceModeTriangles =
      ToJoltBackFaceMode(settings->back_face_mode_triangles);
  out.mBackFaceModeConvex = ToJoltBackFaceMode(settings->back_face_mode_convex);
  out.mTreatConvexAsSolid = settings->treat_convex_as_solid;
  return out;
}

/// Builds the shape cast Jolt wants from the world transform the ABI takes.
///
/// The distinction matters: Jolt casts the shape's CENTRE OF MASS, while a
/// host thinks in world transforms. sFromWorldTransform is the conversion, and
/// doing it here means an offset-centre-of-mass shape sweeps from where the
/// caller asked rather than from wherever its centre of mass happens to be.
JPH::RShapeCast MakeShapeCast(const JPH::Shape *shape, const ZJoltVec3 *scale,
                              const ZJoltRVec3 &position,
                              const ZJoltQuat &rotation,
                              const ZJoltVec3 &direction) {
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
  const JPH::RMat44 transform = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(rotation), zjolt::ToJoltR(position));
  return JPH::RShapeCast::sFromWorldTransform(shape, shape_scale, transform,
                                              zjolt::ToJolt(direction));
}

/// CollideShape works in centre-of-mass space, so the shape's own centre of
/// mass offset is folded in here rather than left for the caller to discover.
JPH::RMat44 MakeCollideTransform(const JPH::Shape *shape,
                                 JPH::Vec3Arg shape_scale,
                                 const ZJoltRVec3 &position,
                                 const ZJoltQuat &rotation) {
  const JPH::RMat44 world = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(rotation), zjolt::ToJoltR(position));
  return world.PreTranslated(shape_scale * shape->GetCenterOfMass());
}

/// The tail of a count-then-fill entry point: report the true count, and say
/// so when the buffer could not hold it.
ZJoltResult ReportCount(uint32_t count, const void *out_hits, uint32_t capacity,
                        uint32_t *out_count) {
  *out_count = count;
  if (out_hits == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  return ZJOLT_RESULT_OK;
}

/// A streaming query with nowhere to stream to is a caller mistake, not an
/// empty result — saying so is more useful than silently traversing and
/// discarding.
ZJoltResult MissingCallback() {
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "on_hit is NULL: a streaming query has nowhere to report hits");
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Ray casts
//===----------------------------------------------------------------------===//

void zjoltRayCastSettingsInit(ZJoltRayCastSettings *settings) {
  if (settings == nullptr) return;
  const JPH::RayCastSettings defaults;
  settings->back_face_mode_triangles =
      defaults.mBackFaceModeTriangles == JPH::EBackFaceMode::CollideWithBackFaces
          ? ZJOLT_BACK_FACE_MODE_COLLIDE
          : ZJOLT_BACK_FACE_MODE_IGNORE;
  settings->back_face_mode_convex =
      defaults.mBackFaceModeConvex == JPH::EBackFaceMode::CollideWithBackFaces
          ? ZJOLT_BACK_FACE_MODE_COLLIDE
          : ZJOLT_BACK_FACE_MODE_IGNORE;
  settings->treat_convex_as_solid = defaults.mTreatConvexAsSolid;
}

ZJoltResult zjoltCastRayClosest(const ZJoltPhysicsSystem *system,
                                const ZJoltRVec3 *origin,
                                const ZJoltVec3 *direction,
                                const ZJoltRayCastSettings *settings,
                                const ZJoltQueryFilters *filters,
                                ZJoltRayCastHit *out_hit, bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, origin, direction, out_hit, out_hit_any))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  bool had_hit = false;
  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray}, KeepBest<ZJoltRayCastHit>{out_hit, &had_hit});
  system->system.GetNarrowPhaseQuery().CastRay(
      ray, MakeRayCastSettings(settings), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastRayAll(const ZJoltPhysicsSystem *system,
                            const ZJoltRVec3 *origin,
                            const ZJoltVec3 *direction,
                            const ZJoltRayCastSettings *settings,
                            const ZJoltQueryFilters *filters,
                            ZJoltRayCastHit *out_hits, uint32_t capacity,
                            uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, origin, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray}, FillBuffer<ZJoltRayCastHit>{out_hits, capacity});
  system->system.GetNarrowPhaseQuery().CastRay(
      ray, MakeRayCastSettings(settings), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCastRayEach(const ZJoltPhysicsSystem *system,
                             const ZJoltRVec3 *origin,
                             const ZJoltVec3 *direction,
                             const ZJoltRayCastSettings *settings,
                             const ZJoltQueryFilters *filters,
                             ZJoltRayCastHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, origin, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();

  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray},
      ForwardToHost<ZJoltRayCastHit, ZJoltRayCastHitFn>{on_hit, user});
  system->system.GetNarrowPhaseQuery().CastRay(
      ray, MakeRayCastSettings(settings), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Shape casts
//
// Contact points come back relative to a base offset of our choosing, and the
// caller's own `position` is the one to pick. Resolving them to absolute world
// space here would mean storing a world coordinate in a float, which in a
// double-precision build is exactly the precision this base-offset mechanism
// exists to avoid losing.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCastShapeClosest(const ZJoltPhysicsSystem *system,
                                  const ZJoltShape *shape,
                                  const ZJoltVec3 *scale,
                                  const ZJoltRVec3 *position,
                                  const ZJoltQuat *rotation,
                                  const ZJoltVec3 *direction,
                                  const ZJoltQueryFilters *filters,
                                  ZJoltShapeCastHit *out_hit,
                                  bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  bool had_hit = false;
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{}, KeepBest<ZJoltShapeCastHit>{out_hit, &had_hit});
  system->system.GetNarrowPhaseQuery().CastShape(
      cast, JPH::ShapeCastSettings(), zjolt::ToJoltR(*position), collector,
      adapters.broad_phase, adapters.object_layer, adapters.body,
      adapters.shape);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastShapeAll(const ZJoltPhysicsSystem *system,
                              const ZJoltShape *shape, const ZJoltVec3 *scale,
                              const ZJoltRVec3 *position,
                              const ZJoltQuat *rotation,
                              const ZJoltVec3 *direction,
                              const ZJoltQueryFilters *filters,
                              ZJoltShapeCastHit *out_hits, uint32_t capacity,
                              uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{}, FillBuffer<ZJoltShapeCastHit>{out_hits, capacity});
  system->system.GetNarrowPhaseQuery().CastShape(
      cast, JPH::ShapeCastSettings(), zjolt::ToJoltR(*position), collector,
      adapters.broad_phase, adapters.object_layer, adapters.body,
      adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCastShapeEach(const ZJoltPhysicsSystem *system,
                               const ZJoltShape *shape, const ZJoltVec3 *scale,
                               const ZJoltRVec3 *position,
                               const ZJoltQuat *rotation,
                               const ZJoltVec3 *direction,
                               const ZJoltQueryFilters *filters,
                               ZJoltShapeCastHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{},
      ForwardToHost<ZJoltShapeCastHit, ZJoltShapeCastHitFn>{on_hit, user});
  system->system.GetNarrowPhaseQuery().CastShape(
      cast, JPH::ShapeCastSettings(), zjolt::ToJoltR(*position), collector,
      adapters.broad_phase, adapters.object_layer, adapters.body,
      adapters.shape);

  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Overlap
//
// Same base-offset rule as the shape casts above: hits are relative to the
// caller's `position`, not resolved to an absolute world coordinate that a
// float cannot hold precisely.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollideShapeClosest(const ZJoltPhysicsSystem *system,
                                     const ZJoltShape *shape,
                                     const ZJoltVec3 *scale,
                                     const ZJoltRVec3 *position,
                                     const ZJoltQuat *rotation,
                                     float max_separation_distance,
                                     const ZJoltQueryFilters *filters,
                                     ZJoltCollideShapeHit *out_hit,
                                     bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = max_separation_distance;

  bool had_hit = false;
  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{}, KeepBest<ZJoltCollideShapeHit>{out_hit, &had_hit});
  system->system.GetNarrowPhaseQuery().CollideShape(
      impl, shape_scale,
      MakeCollideTransform(impl, shape_scale, *position, *rotation), settings,
      zjolt::ToJoltR(*position), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCollideShapeAll(const ZJoltPhysicsSystem *system,
                                 const ZJoltShape *shape,
                                 const ZJoltVec3 *scale,
                                 const ZJoltRVec3 *position,
                                 const ZJoltQuat *rotation,
                                 float max_separation_distance,
                                 const ZJoltQueryFilters *filters,
                                 ZJoltCollideShapeHit *out_hits,
                                 uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = max_separation_distance;

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{},
      FillBuffer<ZJoltCollideShapeHit>{out_hits, capacity});
  system->system.GetNarrowPhaseQuery().CollideShape(
      impl, shape_scale,
      MakeCollideTransform(impl, shape_scale, *position, *rotation), settings,
      zjolt::ToJoltR(*position), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCollideShapeEach(const ZJoltPhysicsSystem *system,
                                  const ZJoltShape *shape,
                                  const ZJoltVec3 *scale,
                                  const ZJoltRVec3 *position,
                                  const ZJoltQuat *rotation,
                                  float max_separation_distance,
                                  const ZJoltQueryFilters *filters,
                                  ZJoltCollideShapeHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = max_separation_distance;

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{},
      ForwardToHost<ZJoltCollideShapeHit, ZJoltCollideShapeHitFn>{on_hit,
                                                                 user});
  system->system.GetNarrowPhaseQuery().CollideShape(
      impl, shape_scale,
      MakeCollideTransform(impl, shape_scale, *position, *rotation), settings,
      zjolt::ToJoltR(*position), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Point
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollidePointAll(const ZJoltPhysicsSystem *system,
                                 const ZJoltRVec3 *point,
                                 const ZJoltQueryFilters *filters,
                                 ZJoltCollidePointHit *out_hits,
                                 uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, point, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollidePointCollector, ZJoltCollidePointHit>(
      ProjectPointHit{},
      FillBuffer<ZJoltCollidePointHit>{out_hits, capacity});
  system->system.GetNarrowPhaseQuery().CollidePoint(
      zjolt::ToJoltR(*point), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCollidePointEach(const ZJoltPhysicsSystem *system,
                                  const ZJoltRVec3 *point,
                                  const ZJoltQueryFilters *filters,
                                  ZJoltCollidePointHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, point)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollidePointCollector, ZJoltCollidePointHit>(
      ProjectPointHit{},
      ForwardToHost<ZJoltCollidePointHit, ZJoltCollidePointHitFn>{on_hit,
                                                                 user});
  system->system.GetNarrowPhaseQuery().CollidePoint(
      zjolt::ToJoltR(*point), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ZJOLT_RESULT_OK;
}

}  // extern "C"
