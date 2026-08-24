//===----------------------------------------------------------------------===//
// zjolt — broad-phase queries.
//
// Everything here goes through JPH::BroadPhaseQuery, which tests bounding
// boxes and hands back body ids. The narrow phase is not involved and no
// body lock is taken, which is what makes these cheap and what makes their
// answers approximate; the header says so at length, because a caller who
// expects contact points from one of these has reached for the wrong query.
//
// Every one of them narrows a position from ZJoltRVec3 to a float Vec3 on the
// way in. That is not a shortcut: Jolt's broad phase is single precision in
// every build, and Jolt's own narrow-phase entry points narrow the same way
// before consulting it (NarrowPhaseQuery.cpp:216).
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Geometry/AABox.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Physics/Collision/AABoxCast.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>

namespace {

using zjolt::BroadPhaseFilters;

/// The shared preamble: the library is up, the system exists, and the count
/// out-parameter is written before anything can fail.
ZJoltResult BeginBroadPhase(const ZJoltPhysicsSystem *system,
                            uint32_t *out_count) {
  if (!zjolt::Present(system, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return ZJOLT_RESULT_OK;
}

/// Copies collected body ids out under the two-call protocol.
ZJoltResult CopyIds(const JPH::Array<JPH::BodyID> &hits, ZJoltBodyId *out_ids,
                    uint32_t capacity, uint32_t *out_count) {
  const uint32_t count = static_cast<uint32_t>(hits.size());
  *out_count = count;
  if (out_ids == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) out_ids[i] = zjolt::ToC(hits[i]);
  return ZJOLT_RESULT_OK;
}

/// The same, for the two casts, which report a fraction as well.
ZJoltResult CopyCastHits(const JPH::Array<JPH::BroadPhaseCastResult> &hits,
                         ZJoltBroadPhaseCastHit *out_hits, uint32_t capacity,
                         uint32_t *out_count) {
  const uint32_t count = static_cast<uint32_t>(hits.size());
  *out_count = count;
  if (out_hits == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) {
    out_hits[i].body = zjolt::ToC(hits[i].mBodyID);
    out_hits[i].fraction = hits[i].mFraction;
  }
  return ZJOLT_RESULT_OK;
}

JPH::AABox ToJoltBox(const ZJoltAABox &box) {
  return JPH::AABox(zjolt::ToJolt(box.min), zjolt::ToJolt(box.max));
}

}  // namespace

extern "C" {

ZJoltResult zjoltBroadPhaseCastRay(const ZJoltPhysicsSystem *system,
                                   const ZJoltRVec3 *origin,
                                   const ZJoltVec3 *direction,
                                   const ZJoltBroadPhaseFilters *filters,
                                   ZJoltBroadPhaseCastHit *out_hits,
                                   uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const ZJoltResult ready = BeginBroadPhase(system, out_count);
  if (ready != ZJOLT_RESULT_OK) return ready;
  if (!zjolt::Present(origin, direction)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const BroadPhaseFilters adapters(filters);
  const JPH::RayCast ray(JPH::Vec3(zjolt::ToJoltR(*origin)),
                         zjolt::ToJolt(*direction));

  JPH::AllHitCollisionCollector<JPH::RayCastBodyCollector> collector;
  system->system.GetBroadPhaseQuery().CastRay(
      ray, collector, adapters.broad_phase, adapters.object_layer);

  return CopyCastHits(collector.mHits, out_hits, capacity, out_count);
}

ZJoltResult zjoltBroadPhaseCollideAABox(const ZJoltPhysicsSystem *system,
                                        const ZJoltAABox *box,
                                        const ZJoltBroadPhaseFilters *filters,
                                        ZJoltBodyId *out_ids, uint32_t capacity,
                                        uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const ZJoltResult ready = BeginBroadPhase(system, out_count);
  if (ready != ZJOLT_RESULT_OK) return ready;
  if (!zjolt::Present(box)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const BroadPhaseFilters adapters(filters);
  JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
  system->system.GetBroadPhaseQuery().CollideAABox(
      ToJoltBox(*box), collector, adapters.broad_phase, adapters.object_layer);

  return CopyIds(collector.mHits, out_ids, capacity, out_count);
}

ZJoltResult zjoltBroadPhaseCollideSphere(const ZJoltPhysicsSystem *system,
                                         const ZJoltRVec3 *center, float radius,
                                         const ZJoltBroadPhaseFilters *filters,
                                         ZJoltBodyId *out_ids,
                                         uint32_t capacity,
                                         uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const ZJoltResult ready = BeginBroadPhase(system, out_count);
  if (ready != ZJOLT_RESULT_OK) return ready;
  if (!zjolt::Present(center)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const BroadPhaseFilters adapters(filters);
  JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
  system->system.GetBroadPhaseQuery().CollideSphere(
      JPH::Vec3(zjolt::ToJoltR(*center)), radius, collector,
      adapters.broad_phase, adapters.object_layer);

  return CopyIds(collector.mHits, out_ids, capacity, out_count);
}

ZJoltResult zjoltBroadPhaseCollidePoint(const ZJoltPhysicsSystem *system,
                                        const ZJoltRVec3 *point,
                                        const ZJoltBroadPhaseFilters *filters,
                                        ZJoltBodyId *out_ids, uint32_t capacity,
                                        uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const ZJoltResult ready = BeginBroadPhase(system, out_count);
  if (ready != ZJOLT_RESULT_OK) return ready;
  if (!zjolt::Present(point)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const BroadPhaseFilters adapters(filters);
  JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
  system->system.GetBroadPhaseQuery().CollidePoint(
      JPH::Vec3(zjolt::ToJoltR(*point)), collector, adapters.broad_phase,
      adapters.object_layer);

  return CopyIds(collector.mHits, out_ids, capacity, out_count);
}

ZJoltResult zjoltBroadPhaseCollideOrientedBox(
    const ZJoltPhysicsSystem *system, const ZJoltOrientedBox *box,
    const ZJoltBroadPhaseFilters *filters, ZJoltBodyId *out_ids,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const ZJoltResult ready = BeginBroadPhase(system, out_count);
  if (ready != ZJOLT_RESULT_OK) return ready;
  if (!zjolt::Present(box)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const BroadPhaseFilters adapters(filters);
  // ToJoltRotation rather than ToJolt: sRotationTranslation asserts its
  // quaternion is unit length, and a caller's rotation that has merely drifted
  // must not abort the process over a culling query.
  const JPH::Mat44 orientation = JPH::Mat44::sRotationTranslation(
      zjolt::ToJoltRotation(box->rotation),
      JPH::Vec3(zjolt::ToJoltR(box->center)));
  const JPH::OrientedBox oriented(orientation, zjolt::ToJolt(box->half_extent));

  JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
  system->system.GetBroadPhaseQuery().CollideOrientedBox(
      oriented, collector, adapters.broad_phase, adapters.object_layer);

  return CopyIds(collector.mHits, out_ids, capacity, out_count);
}

ZJoltResult zjoltBroadPhaseCastAABox(const ZJoltPhysicsSystem *system,
                                     const ZJoltAABox *box,
                                     const ZJoltVec3 *direction,
                                     const ZJoltBroadPhaseFilters *filters,
                                     ZJoltBroadPhaseCastHit *out_hits,
                                     uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const ZJoltResult ready = BeginBroadPhase(system, out_count);
  if (ready != ZJOLT_RESULT_OK) return ready;
  if (!zjolt::Present(box, direction)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const BroadPhaseFilters adapters(filters);
  const JPH::AABoxCast cast{ToJoltBox(*box), zjolt::ToJolt(*direction)};

  JPH::AllHitCollisionCollector<JPH::CastShapeBodyCollector> collector;
  system->system.GetBroadPhaseQuery().CastAABox(
      cast, collector, adapters.broad_phase, adapters.object_layer);

  return CopyCastHits(collector.mHits, out_hits, capacity, out_count);
}

void zjoltBroadPhaseGetBounds(const ZJoltPhysicsSystem *system,
                              ZJoltAABox *out) {
  if (out == nullptr) return;
  if (system == nullptr) {
    *out = ZJoltAABox{};
    return;
  }
  // Not corrected when the world is empty. Jolt returns its inside-out
  // initial box then — min at +FLT_MAX, max at -FLT_MAX — and inventing a
  // zero box instead would turn "there is nothing here" into "there is
  // something here, at the origin", which is worse.
  const JPH::AABox bounds = system->system.GetBroadPhaseQuery().GetBounds();
  out->min = zjolt::ToC(bounds.mMin);
  out->max = zjolt::ToC(bounds.mMax);
}

}  // extern "C"
