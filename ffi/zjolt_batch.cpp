//===----------------------------------------------------------------------===//
// zjolt — many bodies at once.
//
// Two things make this more than a loop over the single-body calls.
//
// The first is the broad phase: Jolt's batch insert sorts the whole set by
// broad-phase layer and builds the subtree in one pass, which is the reason to
// use it at all.
//
// The second is that Jolt's batch calls trust their arguments in a way the
// single-body ones do not. BodyInterface::AddBody takes a body lock and does
// nothing when it fails, so a stale id is harmless. BodyManager::ActivateBodies
// skips only the ALL-ONES invalid id and then dereferences `mBodies[index]`
// directly (`BodyManager.cpp:500`) — a stale id whose slot has been recycled
// reaches an assert, and a stale id whose slot is free reaches a pointer that
// is a freelist link with a tag bit set. DestroyBodies is the same shape
// (`BodyManager.cpp:349`). So every entry point here validates its ids under a
// body lock first, and either refuses the batch or drops the id, per the
// contract in the header.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Core/QuickSort.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>

//===----------------------------------------------------------------------===//
// The staged batch
//
// Global namespace, because the C header names it as an opaque handle and the
// tag has to match.
//
// It owns the id array, and that ownership is the point. Jolt's AddState holds
// POINTERS INTO the array it was given (`BroadPhaseQuadTree.cpp:185`), so the
// array has to stay put between prepare and finalize. Owning a copy here is
// what lets the caller's array be a temporary.
//===----------------------------------------------------------------------===//

struct ZJoltBodyAddBatch {
  JPH::Array<JPH::BodyID> ids;
  JPH::BodyInterface::AddState state = nullptr;
  ZJoltPhysicsSystem *owner = nullptr;
};

namespace {

/// What a batch entry point requires of each id.
enum class Require {
  /// A live body that is not in the broad phase — ready to be added.
  LiveAndNotAdded,
  /// A live body that is in the broad phase — ready to be removed, activated
  /// or deactivated.
  LiveAndAdded,
  /// A live body, added or not.
  Live,
};

bool Satisfies(const JPH::Body *body, Require require) {
  if (body == nullptr) return false;
  switch (require) {
    case Require::LiveAndNotAdded:
      return !body->IsInBroadPhase();
    case Require::LiveAndAdded:
      return body->IsInBroadPhase();
    case Require::Live:
      break;
  }
  return true;
}

/// Copies `bodies` into `out` and checks every one under a single multi-body
/// read lock.
///
/// `strict` decides what a failing id means: refuse the whole batch, or leave
/// it out. Refusing is right where a partial result is worse than none — half
/// a level inserted into the broad phase — and dropping is right where the
/// caller's list is simply older than the world, which is what a set of ids
/// collected last frame is.
ZJoltResult GatherBodies(ZJoltPhysicsSystem *system, const ZJoltBodyId *bodies,
                         uint32_t count, Require require, bool strict,
                         JPH::Array<JPH::BodyID> &out) {
  out.clear();
  if (count == 0) return ZJOLT_RESULT_OK;

  out.reserve(count);
  for (uint32_t i = 0; i < count; ++i) out.push_back(zjolt::ToJolt(bodies[i]));

  JPH::BodyLockMultiRead lock(system->system.GetBodyLockInterface(), out.data(),
                              static_cast<int>(out.size()));

  size_t kept = 0;
  for (uint32_t i = 0; i < count; ++i) {
    const JPH::Body *body = lock.GetBody(static_cast<int>(i));
    if (Satisfies(body, require)) {
      out[kept++] = out[i];
      continue;
    }
    if (!strict) continue;

    if (body == nullptr) {
      return zjolt::SetError(ZJOLT_RESULT_BODY_NOT_FOUND,
                             "the batch names a body that does not exist");
    }
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        require == Require::LiveAndNotAdded
            ? "the batch names a body that is already added to the simulation"
            : "the batch names a body that has not been added to the "
              "simulation");
  }
  out.resize(kept);
  return ZJOLT_RESULT_OK;
}

/// Refuses a batch that names the same body twice.
///
/// Jolt would not catch it: AddBodiesPrepare checks every id against the
/// broad phase BEFORE inserting any of them, so both copies pass, and
/// AddBodiesFinalize then inserts the body into the tree twice. Sorting first
/// makes duplicates adjacent, and Jolt sorts the array by broad-phase layer
/// immediately afterwards anyway, so the order this leaves behind costs
/// nothing.
ZJoltResult RefuseDuplicates(JPH::Array<JPH::BodyID> &ids) {
  if (ids.size() < 2) return ZJOLT_RESULT_OK;
  JPH::QuickSort(ids.begin(), ids.end());
  for (size_t i = 1; i < ids.size(); ++i) {
    if (ids[i] == ids[i - 1]) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "the batch names the same body twice");
    }
  }
  return ZJOLT_RESULT_OK;
}

JPH::EActivation ToJoltActivation(ZJoltActivation activation) {
  return activation == ZJOLT_ACTIVATION_DONT_ACTIVATE
             ? JPH::EActivation::DontActivate
             : JPH::EActivation::Activate;
}

void ForgetBatch(ZJoltPhysicsSystem *system, ZJoltBodyAddBatch *batch) {
  for (size_t i = 0; i < system->pending_batches.size(); ++i) {
    if (system->pending_batches[i] != batch) continue;
    system->pending_batches.erase(system->pending_batches.begin() +
                                  static_cast<ptrdiff_t>(i));
    return;
  }
}

/// True when `batch` was prepared on `system` and is still outstanding.
bool OwnsBatch(const ZJoltPhysicsSystem *system,
               const ZJoltBodyAddBatch *batch) {
  for (const ZJoltBodyAddBatch *pending : system->pending_batches)
    if (pending == batch) return true;
  return false;
}

}  // namespace

namespace zjolt {

void AbortPendingBatches(ZJoltPhysicsSystem *system) {
  for (ZJoltBodyAddBatch *batch : system->pending_batches) {
    if (!batch->ids.empty()) {
      system->system.GetBodyInterface().AddBodiesAbort(
          batch->ids.data(), static_cast<int>(batch->ids.size()), batch->state);
    }
    Delete(batch);
  }
  system->pending_batches.clear();
}

}  // namespace zjolt

extern "C" {

//===----------------------------------------------------------------------===//
// Batch insert
//===----------------------------------------------------------------------===//

ZJoltResult zjoltBodyAddBatchPrepare(ZJoltPhysicsSystem *system,
                                     const ZJoltBodyId *bodies, uint32_t count,
                                     ZJoltBodyAddBatch **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(bodies))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  ZJoltBodyAddBatch *batch = zjolt::New<ZJoltBodyAddBatch>();
  if (batch == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  batch->owner = system;

  ZJoltResult result = GatherBodies(system, bodies, count,
                                    Require::LiveAndNotAdded, true, batch->ids);
  if (result == ZJOLT_RESULT_OK) result = RefuseDuplicates(batch->ids);
  if (result != ZJOLT_RESULT_OK) {
    zjolt::Delete(batch);
    return result;
  }

  if (!batch->ids.empty()) {
    batch->state = system->system.GetBodyInterface().AddBodiesPrepare(
        batch->ids.data(), static_cast<int>(batch->ids.size()));
  }

  system->pending_batches.push_back(batch);
  *out = batch;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltBodyAddBatchFinalize(ZJoltPhysicsSystem *system,
                                      ZJoltBodyAddBatch *batch,
                                      ZJoltActivation activation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, batch)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!OwnsBatch(system, batch)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "that batch was not prepared on this physics system, or has already "
        "been finalized or aborted");
  }

  if (!batch->ids.empty()) {
    system->system.GetBodyInterface().AddBodiesFinalize(
        batch->ids.data(), static_cast<int>(batch->ids.size()), batch->state,
        ToJoltActivation(activation));
  }

  ForgetBatch(system, batch);
  zjolt::Delete(batch);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltBodyAddBatchAbort(ZJoltPhysicsSystem *system,
                                   ZJoltBodyAddBatch *batch) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, batch)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!OwnsBatch(system, batch)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "that batch was not prepared on this physics system, or has already "
        "been finalized or aborted");
  }

  if (!batch->ids.empty()) {
    system->system.GetBodyInterface().AddBodiesAbort(
        batch->ids.data(), static_cast<int>(batch->ids.size()), batch->state);
  }

  ForgetBatch(system, batch);
  zjolt::Delete(batch);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltBodyAddBatch(ZJoltPhysicsSystem *system,
                              const ZJoltBodyId *bodies, uint32_t count,
                              ZJoltActivation activation) {
  ZJoltBodyAddBatch *batch = nullptr;
  const ZJoltResult prepared =
      zjoltBodyAddBatchPrepare(system, bodies, count, &batch);
  if (prepared != ZJOLT_RESULT_OK) return prepared;
  return zjoltBodyAddBatchFinalize(system, batch, activation);
}

//===----------------------------------------------------------------------===//
// Batch remove and destroy
//===----------------------------------------------------------------------===//

ZJoltResult zjoltBodyRemoveBatch(ZJoltPhysicsSystem *system,
                                 const ZJoltBodyId *bodies, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(bodies))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Jolt shuffles the array it is given; this one is ours, so the caller's is
  // untouched and may have been a temporary.
  JPH::Array<JPH::BodyID> ids;
  const ZJoltResult gathered =
      GatherBodies(system, bodies, count, Require::LiveAndAdded, true, ids);
  if (gathered != ZJOLT_RESULT_OK) return gathered;
  if (ids.empty()) return ZJOLT_RESULT_OK;

  system->system.GetBodyInterface().RemoveBodies(ids.data(),
                                                 static_cast<int>(ids.size()));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltBodyDestroyBatch(ZJoltPhysicsSystem *system,
                                  const ZJoltBodyId *bodies, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(bodies))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Ids that no longer name a body are dropped rather than refused: a list
  // built last frame outliving one of its bodies is ordinary, and a destroy
  // that refuses the whole set over it would leave the rest alive.
  JPH::Array<JPH::BodyID> live;
  const ZJoltResult gathered =
      GatherBodies(system, bodies, count, Require::Live, false, live);
  if (gathered != ZJOLT_RESULT_OK) return gathered;
  if (live.empty()) return ZJOLT_RESULT_OK;

  const ZJoltResult duplicated = RefuseDuplicates(live);
  if (duplicated != ZJOLT_RESULT_OK) return duplicated;

  // BodyManager::RemoveBodyInternal asserts the body is out of the broad phase
  // and inactive, so anything still added has to be removed first. Splitting
  // the set is cheaper than removing them one at a time.
  JPH::Array<JPH::BodyID> added;
  {
    JPH::BodyLockMultiRead lock(system->system.GetBodyLockInterface(),
                                live.data(), static_cast<int>(live.size()));
    for (size_t i = 0; i < live.size(); ++i) {
      const JPH::Body *body = lock.GetBody(static_cast<int>(i));
      if (body != nullptr && body->IsInBroadPhase()) added.push_back(live[i]);
    }
  }
  if (!added.empty()) {
    system->system.GetBodyInterface().RemoveBodies(
        added.data(), static_cast<int>(added.size()));
  }

  system->system.GetBodyInterface().DestroyBodies(
      live.data(), static_cast<int>(live.size()));
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Batch activate
//===----------------------------------------------------------------------===//

ZJoltResult zjoltBodyActivateBatch(ZJoltPhysicsSystem *system,
                                   const ZJoltBodyId *bodies, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(bodies))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Array<JPH::BodyID> ids;
  const ZJoltResult gathered =
      GatherBodies(system, bodies, count, Require::LiveAndAdded, false, ids);
  if (gathered != ZJOLT_RESULT_OK) return gathered;
  if (ids.empty()) return ZJOLT_RESULT_OK;

  system->system.GetBodyInterface().ActivateBodies(
      ids.data(), static_cast<int>(ids.size()));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltBodyDeactivateBatch(ZJoltPhysicsSystem *system,
                                     const ZJoltBodyId *bodies,
                                     uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(bodies))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Array<JPH::BodyID> ids;
  const ZJoltResult gathered =
      GatherBodies(system, bodies, count, Require::LiveAndAdded, false, ids);
  if (gathered != ZJOLT_RESULT_OK) return gathered;
  if (ids.empty()) return ZJOLT_RESULT_OK;

  system->system.GetBodyInterface().DeactivateBodies(
      ids.data(), static_cast<int>(ids.size()));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltBodyActivateInBox(ZJoltPhysicsSystem *system,
                                   const ZJoltAABox *box,
                                   const ZJoltBroadPhaseFilters *filters) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, box)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const zjolt::BroadPhaseFilters adapters(filters);
  const JPH::AABox bounds(zjolt::ToJolt(box->min), zjolt::ToJolt(box->max));
  system->system.GetBodyInterface().ActivateBodiesInAABox(
      bounds, adapters.broad_phase, adapters.object_layer);
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
