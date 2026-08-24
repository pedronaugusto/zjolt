//===----------------------------------------------------------------------===//
// zjolt — ray casts, shape casts, and overlap tests.
//
// Jolt's query API is built on collectors: a callback object it feeds hits to
// as it finds them. Streaming that across a C ABI would mean a callback per
// hit, which is the wrong shape for a query whose whole result a host wants at
// once. So each entry point here uses one of Jolt's own collector
// implementations and copies the result out under the two-call protocol used
// everywhere else in this header.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>

namespace {

uint32_t ToCSubShapeId(const JPH::SubShapeID &id) {
  return static_cast<uint32_t>(id.GetValue());
}

void FillRayHit(ZJoltRayCastHit *out, const JPH::RayCastResult &hit) {
  out->body = zjolt::ToC(hit.mBodyID);
  out->sub_shape_id = ToCSubShapeId(hit.mSubShapeID2);
  out->fraction = hit.mFraction;
}

void FillShapeCastHit(ZJoltShapeCastHit *out, const JPH::ShapeCastResult &hit) {
  out->body = zjolt::ToC(hit.mBodyID2);
  out->sub_shape_id = ToCSubShapeId(hit.mSubShapeID2);
  out->fraction = hit.mFraction;
  out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
  out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
  out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
  out->penetration_depth = hit.mPenetrationDepth;
  out->is_back_face_hit = hit.mIsBackFaceHit;
}

void FillCollideHit(ZJoltCollideShapeHit *out,
                    const JPH::CollideShapeResult &hit) {
  out->body = zjolt::ToC(hit.mBodyID2);
  out->sub_shape_id = ToCSubShapeId(hit.mSubShapeID2);
  out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
  out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
  out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
  out->penetration_depth = hit.mPenetrationDepth;
}

/// Copies a collector's hits out under the two-call protocol, and reports the
/// true count even when the buffer was too small.
template <typename Hit, typename JoltHit, typename Fill>
ZJoltResult CopyHits(const JPH::Array<JoltHit> &hits, Hit *out_hits,
                     uint32_t capacity, uint32_t *out_count, Fill fill) {
  const uint32_t count = static_cast<uint32_t>(hits.size());
  *out_count = count;
  if (out_hits == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) fill(&out_hits[i], hits[i]);
  return ZJOLT_RESULT_OK;
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

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Ray casts
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCastRayClosest(const ZJoltPhysicsSystem *system,
                                const ZJoltRVec3 *origin,
                                const ZJoltVec3 *direction,
                                const ZJoltQueryFilters *filters,
                                ZJoltRayCastHit *out_hit, bool *out_hit_any) {
  ZJOLT_ENTER(out_hit_any);
  if (!zjolt::Present(system, origin, direction, out_hit, out_hit_any))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  JPH::RayCastResult hit;
  const bool did_hit = system->system.GetNarrowPhaseQuery().CastRay(
      ray, hit, adapters.broad_phase, adapters.object_layer, adapters.body);

  *out_hit_any = did_hit;
  if (did_hit) FillRayHit(out_hit, hit);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastRayAll(const ZJoltPhysicsSystem *system,
                            const ZJoltRVec3 *origin,
                            const ZJoltVec3 *direction,
                            const ZJoltQueryFilters *filters,
                            ZJoltRayCastHit *out_hits, uint32_t capacity,
                            uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, origin, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  JPH::RayCastSettings settings;
  JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
  system->system.GetNarrowPhaseQuery().CastRay(
      ray, settings, collector, adapters.broad_phase, adapters.object_layer,
      adapters.body);

  return CopyHits(collector.mHits, out_hits, capacity, out_count, FillRayHit);
}

//===----------------------------------------------------------------------===//
// Shape casts
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
  ZJOLT_ENTER(out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  JPH::ShapeCastSettings settings;
  JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
  // Contact points come back relative to a base offset of our choosing, and
  // the caller's own `position` is the one to pick. Resolving them to absolute
  // world space here would mean storing a world coordinate in a float, which
  // in a double-precision build is exactly the precision this base-offset
  // mechanism exists to avoid losing.
  system->system.GetNarrowPhaseQuery().CastShape(
      cast, settings, zjolt::ToJoltR(*position), collector,
      adapters.broad_phase, adapters.object_layer, adapters.body);

  *out_hit_any = collector.HadHit();
  if (collector.HadHit()) FillShapeCastHit(out_hit, collector.mHit);
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

  JPH::ShapeCastSettings settings;
  JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
  system->system.GetNarrowPhaseQuery().CastShape(
      cast, settings, zjolt::ToJoltR(*position), collector,
      adapters.broad_phase, adapters.object_layer, adapters.body);

  return CopyHits(collector.mHits, out_hits, capacity, out_count,
                  FillShapeCastHit);
}

//===----------------------------------------------------------------------===//
// Overlap
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollideShape(const ZJoltPhysicsSystem *system,
                              const ZJoltShape *shape, const ZJoltVec3 *scale,
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

  // CollideShape works in centre-of-mass space, so the shape's own centre of
  // mass offset is folded in here rather than left for the caller to discover.
  const JPH::RMat44 world = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(*rotation), zjolt::ToJoltR(*position));
  const JPH::RMat44 center_of_mass =
      world.PreTranslated(shape_scale * impl->GetCenterOfMass());

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = max_separation_distance;

  zjolt::QueryFilters adapters(filters);
  JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
  // As in the casts above: relative to the caller's `position`, not resolved
  // to an absolute world coordinate that a float cannot hold precisely.
  system->system.GetNarrowPhaseQuery().CollideShape(
      impl, shape_scale, center_of_mass, settings, zjolt::ToJoltR(*position),
      collector, adapters.broad_phase, adapters.object_layer, adapters.body);

  return CopyHits(collector.mHits, out_hits, capacity, out_count,
                  FillCollideHit);
}

}  // extern "C"
