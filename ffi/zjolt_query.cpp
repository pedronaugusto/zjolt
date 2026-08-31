//===----------------------------------------------------------------------===//
// zjolt — ray casts, shape casts, overlap and point tests.
//
// Jolt's query API is built on collectors: an object it calls once per
// hit. Everything here is one collector — HitStream — with two seams:
// Project turns the hit into the ABI's struct, Sink decides what becomes of it.
//
//   KeepBest      the closest / deepest hit          -> *Closest
//   FillBuffer    the caller's buffer, and the count -> *All
//   ForwardToHost the host's callback                -> *Each
//
// One of Jolt's own collectors per form instead materialises the whole
// result set before copying it out; this file allocates nothing at all.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"
#include "zjolt_query_internal.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollideShapeVsShapePerLeaf.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Collision/ManifoldBetweenTwoFaces.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

#include <cfloat>
#include <cstdio>
#include <type_traits>

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
    // Value-initialised, not merely declared: a projection that does not
    // write every field of a C hit would otherwise hand a caller stack
    // garbage, and adding a field to a hit struct is how that happens.
    CHit hit{};
    project_(&hit, jolt_hit, *this);

    switch (sink_(hit, jolt_hit)) {
      case ZJOLT_HIT_ACTION_STOP:
        this->ForceEarlyOut();
        break;

      case ZJOLT_HIT_ACTION_NARROW: {
        // The guard is the entire reason narrowing is safe: Jolt's
        // UpdateEarlyOutFraction asserts the fraction never rises
        // (CollisionCollector.h:84), and does not promise each hit beats
        // the last — a convex shape reports its back face AFTER its
        // front face, further along the ray. With asserts compiled out,
        // an increasing fraction misbehaves instead of failing loudly.
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

  /// Forwards Jolt's per-body bracket to the sink, unchanged from what
  /// CollisionCollector::OnBody already gives every Base above: a live body,
  /// under whatever lock the query itself took, strictly before any of that
  /// body's hits reach AddHit. Every sink implements this — KeepBest and
  /// FillBuffer do nothing with it, since neither runs host code during the
  /// traversal for it to report to; ForwardToHost is the one that does.
  void OnBody(const JPH::Body &body) override { sink_.OnBody(body); }

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

  /// No host code runs during a *Closest traversal, so there is nowhere for
  /// this to report to. @see HitStream::OnBody.
  void OnBody(const JPH::Body &) const {}
};

/// Fills the caller's buffer and counts everything, including what did
/// not fit.
///
/// Counting past the capacity is the point: the count is the answer to
/// the size query, and it has to be true even when the buffer could not
/// hold it — collapsing the old two-call protocol from two traversals to one.
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

  /// No host code runs during a count-then-fill traversal either. @see
  /// HitStream::OnBody.
  void OnBody(const JPH::Body &) const {}
};

/// Hands the hit to the host and does what it says. The callback is a plain C
/// function pointer and cannot throw across this boundary; a language whose
/// errors unwind carries them out via `user`. @see the note in zjolt_query.h.
///
/// `on_body`/`on_body_user`: the *EachWithBody twin's addition, null in every
/// plain *Each (so OnBody is simply never called there).
template <class CHit, class Fn>
struct ForwardToHost {
  Fn on_hit;
  void *user;
  ZJoltOnBodyFn on_body = nullptr;
  void *on_body_user = nullptr;

  template <class JoltHit>
  ZJoltHitAction operator()(const CHit &hit, const JoltHit &jolt_hit) const {
    // The one place a contact face can be handed out: Jolt's result is still
    // alive here, and the host's callback is done with it before it is not.
    if constexpr (std::is_base_of_v<JPH::CollideShapeResult, JoltHit>) {
      ZJoltVec3 faces[2 * ZJOLT_MAX_FACE_VERTICES];
      CHit with_faces = hit;
      zjolt::AttachFaces(&with_faces, jolt_hit, faces);
      return on_hit(user, &with_faces);
    } else {
      return on_hit(user, &hit);
    }
  }

  /// @see HitStream::OnBody. `body` is never handed out past this call —
  /// the host's callback receives it as a raw ZJoltBody* whose validity ends
  /// when the call returns, the same rule zjoltPhysicsSystemTryGetBodyNoLock
  /// documents for its own result.
  void OnBody(const JPH::Body &body) const {
    if (on_body != nullptr) on_body(on_body_user, zjolt::ToC(&body));
  }
};

//===----------------------------------------------------------------------===//
// Projections
//===----------------------------------------------------------------------===//

/// The material at `sub_shape_id` on `shape`, or NULL if `shape` is NULL
/// (never happens via NarrowPhaseQuery, but cheaper than a precondition).
const JPH::PhysicsMaterial *ResolveMaterial(const JPH::TransformedShape *shape,
                                            const JPH::SubShapeID &sub_shape_id) {
  return shape != nullptr ? shape->GetMaterial(sub_shape_id) : nullptr;
}

/// A ray hit, with its surface normal resolved from the collector's
/// TransformedShape context — RayCastResult does not carry one, and that
/// pointer is valid only during AddHit (CollisionCollector.h:73), so it
/// must be resolved here rather than exported for later.
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

//===----------------------------------------------------------------------===//
// Settings
//===----------------------------------------------------------------------===//

ZJoltActiveEdgeMode ToCActiveEdgeMode(JPH::EActiveEdgeMode mode) {
  return mode == JPH::EActiveEdgeMode::CollideWithAll
             ? ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL
             : ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_ONLY_WITH_ACTIVE;
}

ZJoltCollectFacesMode ToCCollectFacesMode(JPH::ECollectFacesMode mode) {
  return mode == JPH::ECollectFacesMode::CollectFaces
             ? ZJOLT_COLLECT_FACES_MODE_COLLECT_FACES
             : ZJOLT_COLLECT_FACES_MODE_NO_FACES;
}

ZJoltBackFaceMode ToCBackFaceMode(JPH::EBackFaceMode mode) {
  return mode == JPH::EBackFaceMode::CollideWithBackFaces
             ? ZJOLT_BACK_FACE_MODE_COLLIDE
             : ZJOLT_BACK_FACE_MODE_IGNORE;
}

/// The two fields JPH::InternalEdgeRemovingCollector requires, forced the same
/// way NarrowPhaseQuery::CollideShapeWithInternalEdgeRemoval forces them
/// itself (NarrowPhaseQuery.cpp) -- silently overwritten rather than refused,
/// because that is what Jolt does with them and a stricter refusal here would
/// just be a second, disagreeing answer to the same question.
JPH::CollideShapeSettings ForceInternalEdgeRemovalSettings(
    JPH::CollideShapeSettings settings) {
  settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
  settings.mCollectFacesMode = JPH::ECollectFacesMode::CollectFaces;
  return settings;
}

//===----------------------------------------------------------------------===//
// Bodies shared by the locked/*NoLock and plain/*WithBody entry points
//
// Identical past ZJOLT_ENTER regardless of which JPH::NarrowPhaseQuery a
// caller reaches through — each takes the query by reference so the four entry
// points sharing one implementation cannot drift apart.
//===----------------------------------------------------------------------===//

ZJoltResult CastRayClosestImpl(const JPH::NarrowPhaseQuery &query,
                               const ZJoltRVec3 *origin,
                               const ZJoltVec3 *direction,
                               const ZJoltRayCastSettings *settings,
                               const ZJoltQueryFilters *filters,
                               ZJoltRayCastHit *out_hit, bool *out_hit_any) {
  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  bool had_hit = false;
  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray}, KeepBest<ZJoltRayCastHit>{out_hit, &had_hit});
  query.CastRay(ray, zjolt::MakeRayCastSettings(settings), collector,
                adapters.broad_phase, adapters.object_layer, adapters.body,
                adapters.shape);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult CastRayAllImpl(const JPH::NarrowPhaseQuery &query,
                          const ZJoltRVec3 *origin, const ZJoltVec3 *direction,
                          const ZJoltRayCastSettings *settings,
                          const ZJoltQueryFilters *filters,
                          ZJoltRayCastHit *out_hits, uint32_t capacity,
                          uint32_t *out_count) {
  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray}, FillBuffer<ZJoltRayCastHit>{out_hits, capacity});
  query.CastRay(ray, zjolt::MakeRayCastSettings(settings), collector,
                adapters.broad_phase, adapters.object_layer, adapters.body,
                adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult CastRayEachImpl(const JPH::NarrowPhaseQuery &query,
                           const ZJoltRVec3 *origin, const ZJoltVec3 *direction,
                           const ZJoltRayCastSettings *settings,
                           const ZJoltQueryFilters *filters,
                           ZJoltRayCastHitFn on_hit, void *user,
                           ZJoltOnBodyFn on_body, void *on_body_user) {
  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray},
      ForwardToHost<ZJoltRayCastHit, ZJoltRayCastHitFn>{on_hit, user, on_body,
                                                        on_body_user});
  query.CastRay(ray, zjolt::MakeRayCastSettings(settings), collector,
                adapters.broad_phase, adapters.object_layer, adapters.body,
                adapters.shape);

  return ZJOLT_RESULT_OK;
}

ZJoltResult CastShapeClosestImpl(const JPH::NarrowPhaseQuery &query,
                                 const ZJoltShape *shape,
                                 const ZJoltVec3 *scale,
                                 const ZJoltRVec3 *position,
                                 const ZJoltQuat *rotation,
                                 const ZJoltVec3 *direction,
                                 const ZJoltShapeCastSettings *settings,
                                 const ZJoltQueryFilters *filters,
                                 ZJoltShapeCastHit *out_hit,
                                 bool *out_hit_any) {
  const JPH::ShapeCastSettings jolt_settings = zjolt::MakeShapeCastSettings(
      settings, zjolt::FaceDelivery::kDiscarded);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  bool had_hit = false;
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{}, KeepBest<ZJoltShapeCastHit>{out_hit, &had_hit});
  query.CastShape(cast, jolt_settings, zjolt::ToJoltR(*position), collector,
                  adapters.broad_phase, adapters.object_layer, adapters.body,
                  adapters.shape);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult CastShapeAllImpl(const JPH::NarrowPhaseQuery &query,
                             const ZJoltShape *shape, const ZJoltVec3 *scale,
                             const ZJoltRVec3 *position,
                             const ZJoltQuat *rotation,
                             const ZJoltVec3 *direction,
                             const ZJoltShapeCastSettings *settings,
                             const ZJoltQueryFilters *filters,
                             ZJoltShapeCastHit *out_hits, uint32_t capacity,
                             uint32_t *out_count) {
  const JPH::ShapeCastSettings jolt_settings = zjolt::MakeShapeCastSettings(
      settings, zjolt::FaceDelivery::kDiscarded);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{}, FillBuffer<ZJoltShapeCastHit>{out_hits, capacity});
  query.CastShape(cast, jolt_settings, zjolt::ToJoltR(*position), collector,
                  adapters.broad_phase, adapters.object_layer, adapters.body,
                  adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult CastShapeEachImpl(const JPH::NarrowPhaseQuery &query,
                              const ZJoltShape *shape, const ZJoltVec3 *scale,
                              const ZJoltRVec3 *position,
                              const ZJoltQuat *rotation,
                              const ZJoltVec3 *direction,
                              const ZJoltShapeCastSettings *settings,
                              const ZJoltQueryFilters *filters,
                              ZJoltShapeCastHitFn on_hit, void *user,
                              ZJoltOnBodyFn on_body, void *on_body_user) {
  const JPH::ShapeCastSettings jolt_settings = zjolt::MakeShapeCastSettings(
      settings, zjolt::FaceDelivery::kDelivered);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{},
      ForwardToHost<ZJoltShapeCastHit, ZJoltShapeCastHitFn>{
          on_hit, user, on_body, on_body_user});
  query.CastShape(cast, jolt_settings, zjolt::ToJoltR(*position), collector,
                  adapters.broad_phase, adapters.object_layer, adapters.body,
                  adapters.shape);

  return ZJOLT_RESULT_OK;
}

/// `with_internal_edge_removal` picks CollideShape vs.
/// CollideShapeWithInternalEdgeRemoval — the one difference between the plain
/// overlap family and its *WithInternalEdgeRemoval twin, so the four entry
/// points behind EACH of those (locked, *NoLock, and the streaming form's
/// *WithBody and its *NoLock twin) share this rather than eight near-copies.
ZJoltResult CollideShapeClosestImpl(const JPH::NarrowPhaseQuery &query,
                                    bool with_internal_edge_removal,
                                    const ZJoltShape *shape,
                                    const ZJoltVec3 *scale,
                                    const ZJoltRVec3 *position,
                                    const ZJoltQuat *rotation,
                                    const ZJoltCollideShapeSettings *settings,
                                    const ZJoltQueryFilters *filters,
                                    ZJoltCollideShapeHit *out_hit,
                                    bool *out_hit_any) {
  JPH::CollideShapeSettings jolt_settings = zjolt::MakeCollideShapeSettings(
      settings, zjolt::FaceDelivery::kDiscarded);
  if (with_internal_edge_removal)
    jolt_settings = ForceInternalEdgeRemovalSettings(jolt_settings);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  bool had_hit = false;
  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{}, KeepBest<ZJoltCollideShapeHit>{out_hit, &had_hit});
  const JPH::RMat44 transform =
      MakeCollideTransform(impl, shape_scale, *position, *rotation);
  if (with_internal_edge_removal) {
    query.CollideShapeWithInternalEdgeRemoval(
        impl, shape_scale, transform, jolt_settings, zjolt::ToJoltR(*position),
        collector, adapters.broad_phase, adapters.object_layer, adapters.body,
        adapters.shape);
  } else {
    query.CollideShape(impl, shape_scale, transform, jolt_settings,
                       zjolt::ToJoltR(*position), collector,
                       adapters.broad_phase, adapters.object_layer,
                       adapters.body, adapters.shape);
  }

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

/// @see CollideShapeClosestImpl for `with_internal_edge_removal`.
ZJoltResult CollideShapeAllImpl(const JPH::NarrowPhaseQuery &query,
                                bool with_internal_edge_removal,
                                const ZJoltShape *shape, const ZJoltVec3 *scale,
                                const ZJoltRVec3 *position,
                                const ZJoltQuat *rotation,
                                const ZJoltCollideShapeSettings *settings,
                                const ZJoltQueryFilters *filters,
                                ZJoltCollideShapeHit *out_hits,
                                uint32_t capacity, uint32_t *out_count) {
  JPH::CollideShapeSettings jolt_settings = zjolt::MakeCollideShapeSettings(
      settings, zjolt::FaceDelivery::kDiscarded);
  if (with_internal_edge_removal)
    jolt_settings = ForceInternalEdgeRemovalSettings(jolt_settings);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{},
      FillBuffer<ZJoltCollideShapeHit>{out_hits, capacity});
  const JPH::RMat44 transform =
      MakeCollideTransform(impl, shape_scale, *position, *rotation);
  if (with_internal_edge_removal) {
    query.CollideShapeWithInternalEdgeRemoval(
        impl, shape_scale, transform, jolt_settings, zjolt::ToJoltR(*position),
        collector, adapters.broad_phase, adapters.object_layer, adapters.body,
        adapters.shape);
  } else {
    query.CollideShape(impl, shape_scale, transform, jolt_settings,
                       zjolt::ToJoltR(*position), collector,
                       adapters.broad_phase, adapters.object_layer,
                       adapters.body, adapters.shape);
  }

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

/// @see CollideShapeClosestImpl for `with_internal_edge_removal`.
ZJoltResult CollideShapeEachImpl(
    const JPH::NarrowPhaseQuery &query, bool with_internal_edge_removal,
    const ZJoltShape *shape, const ZJoltVec3 *scale,
    const ZJoltRVec3 *position, const ZJoltQuat *rotation,
    const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user, ZJoltOnBodyFn on_body, void *on_body_user) {
  JPH::CollideShapeSettings jolt_settings = zjolt::MakeCollideShapeSettings(
      settings, zjolt::FaceDelivery::kDelivered);
  if (with_internal_edge_removal)
    jolt_settings = ForceInternalEdgeRemovalSettings(jolt_settings);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{},
      ForwardToHost<ZJoltCollideShapeHit, ZJoltCollideShapeHitFn>{
          on_hit, user, on_body, on_body_user});
  const JPH::RMat44 transform =
      MakeCollideTransform(impl, shape_scale, *position, *rotation);
  if (with_internal_edge_removal) {
    query.CollideShapeWithInternalEdgeRemoval(
        impl, shape_scale, transform, jolt_settings, zjolt::ToJoltR(*position),
        collector, adapters.broad_phase, adapters.object_layer, adapters.body,
        adapters.shape);
  } else {
    query.CollideShape(impl, shape_scale, transform, jolt_settings,
                       zjolt::ToJoltR(*position), collector,
                       adapters.broad_phase, adapters.object_layer,
                       adapters.body, adapters.shape);
  }

  return ZJOLT_RESULT_OK;
}

ZJoltResult CollidePointAllImpl(const JPH::NarrowPhaseQuery &query,
                                const ZJoltRVec3 *point,
                                const ZJoltQueryFilters *filters,
                                ZJoltCollidePointHit *out_hits,
                                uint32_t capacity, uint32_t *out_count) {
  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollidePointCollector, ZJoltCollidePointHit>(
      ProjectPointHit{},
      FillBuffer<ZJoltCollidePointHit>{out_hits, capacity});
  query.CollidePoint(zjolt::ToJoltR(*point), collector, adapters.broad_phase,
                     adapters.object_layer, adapters.body, adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult CollidePointEachImpl(const JPH::NarrowPhaseQuery &query,
                                 const ZJoltRVec3 *point,
                                 const ZJoltQueryFilters *filters,
                                 ZJoltCollidePointHitFn on_hit, void *user,
                                 ZJoltOnBodyFn on_body, void *on_body_user) {
  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollidePointCollector, ZJoltCollidePointHit>(
      ProjectPointHit{},
      ForwardToHost<ZJoltCollidePointHit, ZJoltCollidePointHitFn>{
          on_hit, user, on_body, on_body_user});
  query.CollidePoint(zjolt::ToJoltR(*point), collector, adapters.broad_phase,
                     adapters.object_layer, adapters.body, adapters.shape);

  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Shape versus shape
//
// JPH::CollisionDispatch underneath every query above, handed two shapes
// directly (no physics system, broad phase, or body) — reuses this file's
// collector/sinks/count-then-fill tail; new here is Mat44 (not RMat44) argument
// plumbing, since there is no TransformedShape to resolve a material off.
//===----------------------------------------------------------------------===//

/// What a shape can present to Jolt's dispatch table, as a set.
///
/// The table is a NumSubShapeTypes-squared array each shape's sRegister
/// fills in; an empty cell asserts. Classified by LEAVES, not the shape
/// handed in: a compound/decorated shape re-dispatches whatever it
/// holds, so a compound of meshes reaches the same cell a bare mesh would.
enum LeafClasses : uint32_t {
  /// Registered against every other convex leaf (ConvexShape.cpp:556) and, one
  /// way round or the other, against every surface leaf below.
  kLeafConvex = 1u << 0,
  /// A surface with no inside: mesh, height field, plane, soft body. Each is
  /// registered against the convex leaves only (MeshShape.cpp:1291,
  /// HeightFieldShape.cpp:2742, PlaneShape.cpp:519, SoftBodyShape.cpp:340), so
  /// two of them together land in a cell nothing ever filled in.
  kLeafSurface = 1u << 1,
  /// A leaf this library cannot vouch for: the non-convex User slots, whose
  /// registration is known only to whoever defined them, and anything a Jolt
  /// upgrade adds that the switch below has not been taught.
  kLeafUnknown = 1u << 2,
};

/// How deep a shape tree this walks before giving up and refusing.
///
/// Shapes are built bottom-up and hold references downward, so there is no
/// cycle to guard against — this guards the stack. Nothing legitimate nests
/// anywhere near this deep, and refusing is the safe answer for anything that
/// does.
constexpr int kMaxShapeDepth = 64;

uint32_t CollectLeafClasses(const JPH::Shape *shape, int depth) {
  if (shape == nullptr || depth > kMaxShapeDepth) return kLeafUnknown;

  switch (shape->GetType()) {
    case JPH::EShapeType::Convex:
      return kLeafConvex;

    case JPH::EShapeType::Mesh:
    case JPH::EShapeType::HeightField:
    case JPH::EShapeType::SoftBody:
    case JPH::EShapeType::Plane:
      return kLeafSurface;

    // An empty shape has no leaves, and the functions registered for it do
    // nothing at all (EmptyShape.cpp:53). It pairs with anything.
    case JPH::EShapeType::Empty:
      return 0;

    case JPH::EShapeType::Decorated:
      return CollectLeafClasses(
          static_cast<const JPH::DecoratedShape *>(shape)->GetInnerShape(),
          depth + 1);

    case JPH::EShapeType::Compound: {
      uint32_t classes = 0;
      for (const JPH::CompoundShape::SubShape &sub :
           static_cast<const JPH::CompoundShape *>(shape)->GetSubShapes()) {
        classes |= CollectLeafClasses(sub.mShape, depth + 1);
      }
      return classes;
    }

    default:
      // EShapeType::User1..User4.
      return kLeafUnknown;
  }
}

/// Whether Jolt has a collision function for every leaf pair this would reach.
///
/// The same answer serves the collide table and the cast table: both are
/// filled in by the same registration loops, differing only in which direction
/// a surface-versus-convex pair is reversed through.
bool PairIsDispatchable(const JPH::Shape *shape1, const JPH::Shape *shape2) {
  const uint32_t classes1 = CollectLeafClasses(shape1, 0);
  const uint32_t classes2 = CollectLeafClasses(shape2, 0);
  if (((classes1 | classes2) & kLeafUnknown) != 0) return false;
  return (classes1 & kLeafSurface) == 0 || (classes2 & kLeafSurface) == 0;
}

const char *SubShapeTypeName(const JPH::Shape *shape) {
  const JPH::uint index = static_cast<JPH::uint>(shape->GetSubType());
  return index < JPH::NumSubShapeTypes ? JPH::sSubShapeTypeNames[index]
                                       : "unknown";
}

/// The two preconditions CollisionDispatch asserts on rather than
/// reports, checked here so a caller gets an error instead of silence.
/// An unregistered dispatch cell asserts internally; with assertions
/// compiled out, it returns a confident "no overlap" for a test that
/// never ran. An invalid scale is asserted far deeper, once per
/// support-function call, by which point there is nothing left to report to.
ZJoltResult CheckShapePair(const JPH::Shape *shape1, JPH::Vec3Arg scale1,
                           const JPH::Shape *shape2, JPH::Vec3Arg scale2) {
  if (!shape1->IsValidScale(scale1) || !shape2->IsValidScale(scale2)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a shape was given a scale it cannot take: a zero component, or a "
        "non-uniform scale on a shape that insists on a uniform one. "
        "zjoltShapeIsValidScale asks the same question ahead of time and "
        "zjoltShapeMakeScaleValid answers with the nearest scale that passes");
  }

  if (!PairIsDispatchable(shape1, shape2)) {
    char detail[512];
    std::snprintf(
        detail, sizeof(detail),
        "Jolt registers no collision function for %s versus %s. Two surfaces "
        "-- mesh, height field, plane, soft body -- have no inside for the "
        "narrow phase to separate, and a user-defined shape type is "
        "registered only by whoever defined it. Wrapping one in a compound or "
        "a decorated shape does not help: those re-dispatch what they hold, "
        "so one side has to be convex all the way down",
        SubShapeTypeName(shape1), SubShapeTypeName(shape2));
    return zjolt::SetError(ZJOLT_RESULT_UNSUPPORTED, detail);
  }

  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Placement
//===----------------------------------------------------------------------===//

JPH::Vec3 ShapeScaleOr1(const ZJoltVec3 *scale) {
  return scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
}

JPH::RVec3 BaseOffsetOrZero(const ZJoltRVec3 *base_offset) {
  return base_offset != nullptr ? zjolt::ToJoltR(*base_offset)
                                : JPH::RVec3::sZero();
}

zjolt::ShapeFilterAdapter MakeShapeFilter(const ZJoltShapeFilter *filter) {
  return zjolt::ShapeFilterAdapter(filter != nullptr ? *filter
                                                     : ZJoltShapeFilter{});
}

/// The transform CollisionDispatch wants: single precision, centre of mass,
/// relative to the caller's base offset. Two load-bearing conversions: the
/// shape's own centre of mass is folded in (Jolt collides there, not in the
/// given transform's space), and the base offset is subtracted before dropping
/// to single precision, keeping the loss near the query, not the origin.
JPH::Mat44 ToDispatchSpace(const JPH::Shape *shape, JPH::Vec3Arg shape_scale,
                           JPH::RMat44Arg world, JPH::RVec3Arg base_offset) {
  return world.PreTranslated(shape_scale * shape->GetCenterOfMass())
      .PostTranslated(-base_offset)
      .ToMat44();
}

/// The arguments all four shape-versus-shape entry points share, validated and
/// converted once.
struct ShapePair {
  const JPH::Shape *shape1;
  const JPH::Shape *shape2;
  JPH::Vec3 scale1;
  JPH::Vec3 scale2;
  JPH::RMat44 world1;
  JPH::RMat44 world2;
  JPH::RVec3 base_offset;

  JPH::Mat44 CenterOfMassTransform1() const {
    return ToDispatchSpace(shape1, scale1, world1, base_offset);
  }

  JPH::Mat44 CenterOfMassTransform2() const {
    return ToDispatchSpace(shape2, scale2, world2, base_offset);
  }
};

ZJoltResult PrepareShapePair(const ZJoltShape *shape1, const ZJoltVec3 *scale1,
                             const ZJoltRVec3 *position1,
                             const ZJoltQuat *rotation1,
                             const ZJoltShape *shape2, const ZJoltVec3 *scale2,
                             const ZJoltRVec3 *position2,
                             const ZJoltQuat *rotation2,
                             const ZJoltRVec3 *base_offset, ShapePair *out) {
  out->shape1 = zjolt::ToJolt(shape1);
  out->shape2 = zjolt::ToJolt(shape2);
  out->scale1 = ShapeScaleOr1(scale1);
  out->scale2 = ShapeScaleOr1(scale2);

  const ZJoltResult checked =
      CheckShapePair(out->shape1, out->scale1, out->shape2, out->scale2);
  if (checked != ZJOLT_RESULT_OK) return checked;

  out->world1 = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(*rotation1), zjolt::ToJoltR(*position1));
  out->world2 = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(*rotation2), zjolt::ToJoltR(*position2));
  out->base_offset = BaseOffsetOrZero(base_offset);
  return ZJOLT_RESULT_OK;
}

/// The shape cast the dispatch wants, in the same base-offset frame the
/// transforms above are in.
JPH::ShapeCast MakeDispatchShapeCast(const ShapePair &pair,
                                     const ZJoltVec3 &direction) {
  const JPH::RShapeCast world_cast = JPH::RShapeCast::sFromWorldTransform(
      pair.shape1, pair.scale1, pair.world1, zjolt::ToJolt(direction));
  return JPH::ShapeCast(world_cast.PostTranslated(-pair.base_offset));
}

//===----------------------------------------------------------------------===//
// Projections
//
// Unlike the projections above, these read nothing off the collector —
// no TransformedShape here; the material comes off the shape the caller passed,
// and the body id is Jolt's default (invalid), since there is no body.
//===----------------------------------------------------------------------===//

struct ProjectShapeVsShapeCollideHit {
  const JPH::Shape *shape2;

  template <class Collector>
  void operator()(ZJoltCollideShapeHit *out, const JPH::CollideShapeResult &hit,
                  const Collector &) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->material = zjolt::ToC(shape2->GetMaterial(hit.mSubShapeID2));
  }
};

struct ProjectShapeVsShapeCastHit {
  const JPH::Shape *shape2;

  template <class Collector>
  void operator()(ZJoltShapeCastHit *out, const JPH::ShapeCastResult &hit,
                  const Collector &) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->fraction = hit.mFraction;
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->is_back_face_hit = hit.mIsBackFaceHit;
    out->material = zjolt::ToC(shape2->GetMaterial(hit.mSubShapeID2));
  }
};

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
  return CastRayClosestImpl(system->system.GetNarrowPhaseQuery(), origin,
                            direction, settings, filters, out_hit,
                            out_hit_any);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCastRayClosestNoLock(const ZJoltPhysicsSystem *system,
                                      const ZJoltRVec3 *origin,
                                      const ZJoltVec3 *direction,
                                      const ZJoltRayCastSettings *settings,
                                      const ZJoltQueryFilters *filters,
                                      ZJoltRayCastHit *out_hit,
                                      bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, origin, direction, out_hit, out_hit_any))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CastRayClosestImpl(system->system.GetNarrowPhaseQueryNoLock(), origin,
                            direction, settings, filters, out_hit,
                            out_hit_any);
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
  return CastRayAllImpl(system->system.GetNarrowPhaseQuery(), origin,
                        direction, settings, filters, out_hits, capacity,
                        out_count);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCastRayAllNoLock(const ZJoltPhysicsSystem *system,
                                  const ZJoltRVec3 *origin,
                                  const ZJoltVec3 *direction,
                                  const ZJoltRayCastSettings *settings,
                                  const ZJoltQueryFilters *filters,
                                  ZJoltRayCastHit *out_hits, uint32_t capacity,
                                  uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, origin, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CastRayAllImpl(system->system.GetNarrowPhaseQueryNoLock(), origin,
                        direction, settings, filters, out_hits, capacity,
                        out_count);
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
  return CastRayEachImpl(system->system.GetNarrowPhaseQuery(), origin,
                         direction, settings, filters, on_hit, user, nullptr,
                         nullptr);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCastRayEachNoLock(const ZJoltPhysicsSystem *system,
                                   const ZJoltRVec3 *origin,
                                   const ZJoltVec3 *direction,
                                   const ZJoltRayCastSettings *settings,
                                   const ZJoltQueryFilters *filters,
                                   ZJoltRayCastHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, origin, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CastRayEachImpl(system->system.GetNarrowPhaseQueryNoLock(), origin,
                         direction, settings, filters, on_hit, user, nullptr,
                         nullptr);
}

/// @see the "Observing a hit's body before the hit" section below.
ZJoltResult zjoltCastRayEachWithBody(const ZJoltPhysicsSystem *system,
                                     const ZJoltRVec3 *origin,
                                     const ZJoltVec3 *direction,
                                     const ZJoltRayCastSettings *settings,
                                     const ZJoltQueryFilters *filters,
                                     ZJoltRayCastHitFn on_hit, void *user,
                                     ZJoltOnBodyFn on_body,
                                     void *on_body_user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, origin, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CastRayEachImpl(system->system.GetNarrowPhaseQuery(), origin,
                         direction, settings, filters, on_hit, user, on_body,
                         on_body_user);
}

//===----------------------------------------------------------------------===//
// Shape casts
//
// Contact points come back relative to a caller-chosen base offset (the
// caller's own `position` is the one to pick) — resolving to absolute world
// space would store a world coordinate in a float, losing the precision this
// mechanism exists to keep.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCastShapeClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHit *out_hit, bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  return CastShapeClosestImpl(system->system.GetNarrowPhaseQuery(), shape,
                              scale, position, rotation, direction, settings,
                              filters, out_hit, out_hit_any);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCastShapeClosestNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHit *out_hit, bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  return CastShapeClosestImpl(system->system.GetNarrowPhaseQueryNoLock(),
                              shape, scale, position, rotation, direction,
                              settings, filters, out_hit, out_hit_any);
}

ZJoltResult zjoltCastShapeAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHit *out_hits, uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CastShapeAllImpl(system->system.GetNarrowPhaseQuery(), shape, scale,
                          position, rotation, direction, settings, filters,
                          out_hits, capacity, out_count);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCastShapeAllNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHit *out_hits, uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CastShapeAllImpl(system->system.GetNarrowPhaseQueryNoLock(), shape,
                          scale, position, rotation, direction, settings,
                          filters, out_hits, capacity, out_count);
}

ZJoltResult zjoltCastShapeEach(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CastShapeEachImpl(system->system.GetNarrowPhaseQuery(), shape, scale,
                           position, rotation, direction, settings, filters,
                           on_hit, user, nullptr, nullptr);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCastShapeEachNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CastShapeEachImpl(system->system.GetNarrowPhaseQueryNoLock(), shape,
                           scale, position, rotation, direction, settings,
                           filters, on_hit, user, nullptr, nullptr);
}

/// @see the "Observing a hit's body before the hit" section below.
ZJoltResult zjoltCastShapeEachWithBody(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *direction,
    const ZJoltShapeCastSettings *settings, const ZJoltQueryFilters *filters,
    ZJoltShapeCastHitFn on_hit, void *user, ZJoltOnBodyFn on_body,
    void *on_body_user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CastShapeEachImpl(system->system.GetNarrowPhaseQuery(), shape, scale,
                           position, rotation, direction, settings, filters,
                           on_hit, user, on_body, on_body_user);
}

//===----------------------------------------------------------------------===//
// Overlap
//
// Same base-offset rule as the shape casts above: hits are relative to
// the caller's `position`, not an absolute world coordinate a float cannot hold
// precisely.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollideShapeClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  return CollideShapeClosestImpl(system->system.GetNarrowPhaseQuery(), false,
                                 shape, scale, position, rotation, settings,
                                 filters, out_hit, out_hit_any);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCollideShapeClosestNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  return CollideShapeClosestImpl(system->system.GetNarrowPhaseQueryNoLock(),
                                 false, shape, scale, position, rotation,
                                 settings, filters, out_hit, out_hit_any);
}

ZJoltResult zjoltCollideShapeAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CollideShapeAllImpl(system->system.GetNarrowPhaseQuery(), false,
                             shape, scale, position, rotation, settings,
                             filters, out_hits, capacity, out_count);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCollideShapeAllNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CollideShapeAllImpl(system->system.GetNarrowPhaseQueryNoLock(),
                             false, shape, scale, position, rotation,
                             settings, filters, out_hits, capacity,
                             out_count);
}

ZJoltResult zjoltCollideShapeEach(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollideShapeEachImpl(system->system.GetNarrowPhaseQuery(), false,
                              shape, scale, position, rotation, settings,
                              filters, on_hit, user, nullptr, nullptr);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCollideShapeEachNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollideShapeEachImpl(system->system.GetNarrowPhaseQueryNoLock(),
                              false, shape, scale, position, rotation,
                              settings, filters, on_hit, user, nullptr,
                              nullptr);
}

/// @see the "Observing a hit's body before the hit" section below.
ZJoltResult zjoltCollideShapeEachWithBody(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user, ZJoltOnBodyFn on_body, void *on_body_user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollideShapeEachImpl(system->system.GetNarrowPhaseQuery(), false,
                              shape, scale, position, rotation, settings,
                              filters, on_hit, user, on_body, on_body_user);
}

//===----------------------------------------------------------------------===//
// Overlap, with internal edge removal
//
// Same collector/sinks/tail as the plain zjoltCollideShape* family, via
// CollideShapeWithInternalEdgeRemoval — `with_internal_edge_removal` on
// CollideShape*Impl is the only difference, so the *NoLock/*WithBody twins fall
// out of the same three functions.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalClosest(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  return CollideShapeClosestImpl(system->system.GetNarrowPhaseQuery(), true,
                                 shape, scale, position, rotation, settings,
                                 filters, out_hit, out_hit_any);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalClosestNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  return CollideShapeClosestImpl(system->system.GetNarrowPhaseQueryNoLock(),
                                 true, shape, scale, position, rotation,
                                 settings, filters, out_hit, out_hit_any);
}

ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalAll(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CollideShapeAllImpl(system->system.GetNarrowPhaseQuery(), true,
                             shape, scale, position, rotation, settings,
                             filters, out_hits, capacity, out_count);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalAllNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CollideShapeAllImpl(system->system.GetNarrowPhaseQueryNoLock(), true,
                             shape, scale, position, rotation, settings,
                             filters, out_hits, capacity, out_count);
}

ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalEach(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollideShapeEachImpl(system->system.GetNarrowPhaseQuery(), true,
                              shape, scale, position, rotation, settings,
                              filters, on_hit, user, nullptr, nullptr);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalEachNoLock(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollideShapeEachImpl(system->system.GetNarrowPhaseQueryNoLock(),
                              true, shape, scale, position, rotation,
                              settings, filters, on_hit, user, nullptr,
                              nullptr);
}

/// @see the "Observing a hit's body before the hit" section below.
ZJoltResult zjoltCollideShapeWithInternalEdgeRemovalEachWithBody(
    const ZJoltPhysicsSystem *system, const ZJoltShape *shape,
    const ZJoltVec3 *scale, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltCollideShapeSettings *settings,
    const ZJoltQueryFilters *filters, ZJoltCollideShapeHitFn on_hit,
    void *user, ZJoltOnBodyFn on_body, void *on_body_user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollideShapeEachImpl(system->system.GetNarrowPhaseQuery(), true,
                              shape, scale, position, rotation, settings,
                              filters, on_hit, user, on_body, on_body_user);
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
  return CollidePointAllImpl(system->system.GetNarrowPhaseQuery(), point,
                             filters, out_hits, capacity, out_count);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCollidePointAllNoLock(const ZJoltPhysicsSystem *system,
                                       const ZJoltRVec3 *point,
                                       const ZJoltQueryFilters *filters,
                                       ZJoltCollidePointHit *out_hits,
                                       uint32_t capacity,
                                       uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, point, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  return CollidePointAllImpl(system->system.GetNarrowPhaseQueryNoLock(),
                             point, filters, out_hits, capacity, out_count);
}

ZJoltResult zjoltCollidePointEach(const ZJoltPhysicsSystem *system,
                                  const ZJoltRVec3 *point,
                                  const ZJoltQueryFilters *filters,
                                  ZJoltCollidePointHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, point)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollidePointEachImpl(system->system.GetNarrowPhaseQuery(), point,
                              filters, on_hit, user, nullptr, nullptr);
}

/// @see the "unlocked path" section below.
ZJoltResult zjoltCollidePointEachNoLock(const ZJoltPhysicsSystem *system,
                                        const ZJoltRVec3 *point,
                                        const ZJoltQueryFilters *filters,
                                        ZJoltCollidePointHitFn on_hit,
                                        void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, point)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollidePointEachImpl(system->system.GetNarrowPhaseQueryNoLock(),
                              point, filters, on_hit, user, nullptr, nullptr);
}

/// @see the "Observing a hit's body before the hit" section below.
ZJoltResult zjoltCollidePointEachWithBody(const ZJoltPhysicsSystem *system,
                                          const ZJoltRVec3 *point,
                                          const ZJoltQueryFilters *filters,
                                          ZJoltCollidePointHitFn on_hit,
                                          void *user, ZJoltOnBodyFn on_body,
                                          void *on_body_user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, point)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();
  return CollidePointEachImpl(system->system.GetNarrowPhaseQuery(), point,
                              filters, on_hit, user, on_body, on_body_user);
}

//===----------------------------------------------------------------------===//
// Shape versus shape
//
// Same collector/sinks/tail as everything above, driven against
// JPH::CollisionDispatch instead of a NarrowPhaseQuery — no *Each form
// (leaf-bounded, not broad-phase-bounded; see zjolt_transformed.h).
//===----------------------------------------------------------------------===//

void zjoltCollideShapeSettingsInit(ZJoltCollideShapeSettings *settings) {
  if (settings == nullptr) return;
  const JPH::CollideShapeSettings defaults;
  settings->active_edge_mode = ToCActiveEdgeMode(defaults.mActiveEdgeMode);
  settings->collect_faces_mode =
      ToCCollectFacesMode(defaults.mCollectFacesMode);
  settings->collision_tolerance = defaults.mCollisionTolerance;
  settings->penetration_tolerance = defaults.mPenetrationTolerance;
  settings->active_edge_movement_direction =
      zjolt::ToC(defaults.mActiveEdgeMovementDirection);
  settings->max_separation_distance = defaults.mMaxSeparationDistance;
  settings->back_face_mode = ToCBackFaceMode(defaults.mBackFaceMode);
  settings->internal_edge_removal_vertex_tolerance_sq =
      defaults.mInternalEdgeRemovalVertexToleranceSq;
}

void zjoltShapeCastSettingsInit(ZJoltShapeCastSettings *settings) {
  if (settings == nullptr) return;
  const JPH::ShapeCastSettings defaults;
  settings->active_edge_mode = ToCActiveEdgeMode(defaults.mActiveEdgeMode);
  settings->collect_faces_mode =
      ToCCollectFacesMode(defaults.mCollectFacesMode);
  settings->collision_tolerance = defaults.mCollisionTolerance;
  settings->penetration_tolerance = defaults.mPenetrationTolerance;
  settings->active_edge_movement_direction =
      zjolt::ToC(defaults.mActiveEdgeMovementDirection);
  settings->extra_convex_radius = defaults.mExtraConvexRadius;
  settings->back_face_mode_triangles =
      ToCBackFaceMode(defaults.mBackFaceModeTriangles);
  settings->back_face_mode_convex =
      ToCBackFaceMode(defaults.mBackFaceModeConvex);
  settings->use_shrunken_shape_and_convex_radius =
      defaults.mUseShrunkenShapeAndConvexRadius;
  settings->return_deepest_point = defaults.mReturnDeepestPoint;
}

ZJoltResult zjoltCollideShapeVsShapeClosest(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(shape1, position1, rotation1, shape2, position2,
                      rotation2, out_hit, out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::CollideShapeSettings jolt_settings =
      zjolt::MakeCollideShapeSettings(
          settings, zjolt::FaceDelivery::kDiscarded);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  bool had_hit = false;
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectShapeVsShapeCollideHit{pair.shape2},
      KeepBest<ZJoltCollideShapeHit>{out_hit, &had_hit});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollisionDispatch::sCollideShapeVsShape(
      pair.shape1, pair.shape2, pair.scale1, pair.scale2,
      pair.CenterOfMassTransform1(), pair.CenterOfMassTransform2(),
      sub_shape_id1, sub_shape_id2, jolt_settings, collector, shape_filter);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCollideShapeVsShapeAll(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape1, position1, rotation1, shape2, position2,
                      rotation2, out_count)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::CollideShapeSettings jolt_settings =
      zjolt::MakeCollideShapeSettings(
          settings, zjolt::FaceDelivery::kDiscarded);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectShapeVsShapeCollideHit{pair.shape2},
      FillBuffer<ZJoltCollideShapeHit>{out_hits, capacity});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollisionDispatch::sCollideShapeVsShape(
      pair.shape1, pair.shape2, pair.scale1, pair.scale2,
      pair.CenterOfMassTransform1(), pair.CenterOfMassTransform2(),
      sub_shape_id1, sub_shape_id2, jolt_settings, collector, shape_filter);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

/// Same argument handling as zjoltCollideShapeVsShapeAll, including the
/// whole-shape dispatch check, but the traversal runs
/// JPH::CollideShapeVsShapePerLeaf in place of sCollideShapeVsShape,
/// instantiated with ClosestHitCollisionCollector<CollideShapeCollector> as the
/// LeafCollector: the deepest hit per leaf pair, never an arbitrary one. @see
/// zjolt_query.h for why this is the one Jolt offers exposed here.
ZJoltResult zjoltCollideShapeVsShapePerLeafAll(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape1, position1, rotation1, shape2, position2,
                      rotation2, out_count)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::CollideShapeSettings jolt_settings =
      zjolt::MakeCollideShapeSettings(
          settings, zjolt::FaceDelivery::kDiscarded);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectShapeVsShapeCollideHit{pair.shape2},
      FillBuffer<ZJoltCollideShapeHit>{out_hits, capacity});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollideShapeVsShapePerLeaf<
      JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector>>(
      pair.shape1, pair.shape2, pair.scale1, pair.scale2,
      pair.CenterOfMassTransform1(), pair.CenterOfMassTransform2(),
      sub_shape_id1, sub_shape_id2, jolt_settings, collector, shape_filter);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCastShapeVsShapeClosest(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltVec3 *direction, const ZJoltShape *shape2,
    const ZJoltVec3 *scale2, const ZJoltRVec3 *position2,
    const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltShapeCastSettings *settings, const ZJoltShapeFilter *filter,
    ZJoltShapeCastHit *out_hit, bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(shape1, position1, rotation1, direction, shape2,
                      position2, rotation2, out_hit, out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::ShapeCastSettings jolt_settings = zjolt::MakeShapeCastSettings(
      settings, zjolt::FaceDelivery::kDiscarded);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  bool had_hit = false;
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeVsShapeCastHit{pair.shape2},
      KeepBest<ZJoltShapeCastHit>{out_hit, &had_hit});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(
      MakeDispatchShapeCast(pair, *direction), jolt_settings, pair.shape2,
      pair.scale2, shape_filter, pair.CenterOfMassTransform2(), sub_shape_id1,
      sub_shape_id2, collector);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastShapeVsShapeAll(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltVec3 *direction, const ZJoltShape *shape2,
    const ZJoltVec3 *scale2, const ZJoltRVec3 *position2,
    const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltShapeCastSettings *settings, const ZJoltShapeFilter *filter,
    ZJoltShapeCastHit *out_hits, uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape1, position1, rotation1, direction, shape2,
                      position2, rotation2, out_count)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::ShapeCastSettings jolt_settings = zjolt::MakeShapeCastSettings(
      settings, zjolt::FaceDelivery::kDiscarded);
  const ZJoltResult tolerance =
      zjolt::CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeVsShapeCastHit{pair.shape2},
      FillBuffer<ZJoltShapeCastHit>{out_hits, capacity});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(
      MakeDispatchShapeCast(pair, *direction), jolt_settings, pair.shape2,
      pair.scale2, shape_filter, pair.CenterOfMassTransform2(), sub_shape_id1,
      sub_shape_id2, collector);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

//===----------------------------------------------------------------------===//
// Predicting a contact's response, without running a step
//===----------------------------------------------------------------------===//

ZJoltResult zjoltEstimateCollisionResponse(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltCollisionEstimationManifold *manifold, float combined_friction,
    float combined_restitution, float min_velocity_for_restitution,
    uint32_t num_iterations, ZJoltCollisionEstimationResult *out_result) {
  ZJOLT_ENTER(out_result);
  if (!zjolt::Present(system, manifold, out_result))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (manifold->num_points == 0 ||
      manifold->num_points > ZJOLT_CONTACT_POINTS_CAPACITY) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "manifold->num_points must be at least 1 and at most "
        "ZJOLT_CONTACT_POINTS_CAPACITY; Jolt's own manifold cannot hold "
        "more, and a manifold with no points is not a contact");
  }
  if (!zjolt::Present(manifold->points_on_1, manifold->points_on_2))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Jolt's own contact-listener docs: reading a body inside
  // OnContactAdded/OnContactPersisted must go through the no-lock
  // interface, since those callbacks may already hold the real per-body
  // lock for this exact pair — taking it again would deadlock. This is
  // read-only and does the same, assuming a caller elsewhere has made
  // the body pair safe to read some other way.
  const JPH::BodyID ids[2] = {zjolt::ToJolt(body1), zjolt::ToJolt(body2)};
  JPH::BodyLockMultiRead lock(system->system.GetBodyLockInterfaceNoLock(),
                              ids, 2);
  const JPH::Body *jolt_body1 = lock.GetBody(0);
  const JPH::Body *jolt_body2 = lock.GetBody(1);
  if (jolt_body1 == nullptr || jolt_body2 == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_BODY_NOT_FOUND,
                           "body1 or body2 does not name a live body");
  }

  JPH::ContactManifold jolt_manifold;
  jolt_manifold.mBaseOffset = zjolt::ToJoltR(manifold->base_offset);
  jolt_manifold.mWorldSpaceNormal = zjolt::ToJolt(manifold->world_space_normal);
  jolt_manifold.mPenetrationDepth = 0.0f;  // Unread by EstimateCollisionResponse.
  for (uint32_t i = 0; i < manifold->num_points; ++i) {
    jolt_manifold.mRelativeContactPointsOn1.push_back(
        zjolt::ToJolt(manifold->points_on_1[i]));
    jolt_manifold.mRelativeContactPointsOn2.push_back(
        zjolt::ToJolt(manifold->points_on_2[i]));
  }

  JPH::CollisionEstimationResult jolt_result;
  JPH::EstimateCollisionResponse(*jolt_body1, *jolt_body2, jolt_manifold,
                                 jolt_result, combined_friction,
                                 combined_restitution,
                                 min_velocity_for_restitution, num_iterations);

  out_result->linear_velocity1 = zjolt::ToC(jolt_result.mLinearVelocity1);
  out_result->angular_velocity1 = zjolt::ToC(jolt_result.mAngularVelocity1);
  out_result->linear_velocity2 = zjolt::ToC(jolt_result.mLinearVelocity2);
  out_result->angular_velocity2 = zjolt::ToC(jolt_result.mAngularVelocity2);
  out_result->friction_point = zjolt::ToC(jolt_result.mFrictionPoint);
  out_result->tangent1 = zjolt::ToC(jolt_result.mTangent1);
  out_result->tangent2 = zjolt::ToC(jolt_result.mTangent2);
  out_result->friction_impulse1 = jolt_result.mFrictionImpulse1;
  out_result->friction_impulse2 = jolt_result.mFrictionImpulse2;
  out_result->angular_friction_impulse = jolt_result.mAngularFrictionImpulse;

  const uint32_t num_impulses =
      static_cast<uint32_t>(jolt_result.mContactImpulse.size());
  out_result->num_contact_impulses = num_impulses;
  for (uint32_t i = 0; i < num_impulses; ++i)
    out_result->contact_impulses[i] = jolt_result.mContactImpulse[i];

  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPruneContactPoints(const ZJoltVec3 *penetration_axis,
                                    ZJoltVec3 *points_on_1,
                                    ZJoltVec3 *points_on_2, uint32_t *count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(penetration_axis, points_on_1, points_on_2, count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const uint32_t num_points = *count;
  if (num_points <= 4) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "*count is 4 or fewer already; Jolt's own precondition is that "
        "pruning a manifold this small makes no sense "
        "(ManifoldBetweenTwoFaces.h), and there is no reduction to invent "
        "for input already at or under the target");
  }
  if (num_points > ZJOLT_CONTACT_POINTS_CAPACITY) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "*count exceeds ZJOLT_CONTACT_POINTS_CAPACITY; Jolt's own manifold "
        "cannot hold that many points either");
  }

  const JPH::Vec3 axis = zjolt::ToJolt(*penetration_axis);
  const float axis_length_sq = axis.LengthSq();
  if (!std::isfinite(axis_length_sq) || axis_length_sq < 1.0e-12f) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "penetration_axis is zero, non-finite, or otherwise cannot be "
        "normalized; Jolt asserts that it already is");
  }
  const JPH::Vec3 normalized_axis = axis.Normalized();

  JPH::ContactPoints on_1, on_2;
  for (uint32_t i = 0; i < num_points; ++i) {
    on_1.push_back(zjolt::ToJolt(points_on_1[i]));
    on_2.push_back(zjolt::ToJolt(points_on_2[i]));
  }

#ifdef JPH_DEBUG_RENDERER
  // The debug-renderer overload draws the discarded points at this position;
  // it does not otherwise affect which points survive. No entry point here
  // exposes a base offset for it, so this reduction always draws (if drawing
  // is even compiled in) at the origin rather than carrying a parameter
  // whose only job is a debug overlay this ABI does not otherwise offer.
  JPH::PruneContactPoints(normalized_axis, on_1, on_2, JPH::RVec3::sZero());
#else
  JPH::PruneContactPoints(normalized_axis, on_1, on_2);
#endif

  const uint32_t pruned = static_cast<uint32_t>(on_1.size());
  for (uint32_t i = 0; i < pruned; ++i) {
    points_on_1[i] = zjolt::ToC(on_1[i]);
    points_on_2[i] = zjolt::ToC(on_2[i]);
  }
  *count = pruned;
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
