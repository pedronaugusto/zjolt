//===----------------------------------------------------------------------===//
// zjolt — many bodies at once.
//
// Batch insert sorts by broad-phase layer, building the subtree in one
// pass — faster than looping zjoltBodyAdd. Unlike Jolt's own
// AddBodiesPrepare/RemoveBodies, which sort the caller's array IN PLACE
// and hold pointers into it, zjolt copies `bodies`: read once, untouched.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_BATCH_H_
#define ZJOLT_BATCH_H_

#include "zjolt_core.h"
// ZJoltBroadPhaseFilters (zjoltBodyActivateInBox runs a broad-phase query).
#include "zjolt_broadphase.h"
// ZJoltUnassignedBody, yielded by zjoltBodyUnassignIds.
#include "zjolt_body.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Creates and adds bodies — equivalent to prepare then finalize. Every id
/// must be live, not-yet-added, else refused whole (`count` 0 = no-op).
ZJOLT_API ZJoltResult zjoltBodyAddBatch(ZJoltPhysicsSystem *system,
                                        const ZJoltBodyId *bodies,
                                        uint32_t count,
                                        ZJoltActivation activation);

/// Stages a batch for exactly one of Finalize/Abort; until then bodies are
/// tagged but not in the tree. Owned by the system — aborted with it if
/// still outstanding on destroy (not counted by zjoltLiveHandleCount).
ZJOLT_API ZJoltResult zjoltBodyAddBatchPrepare(ZJoltPhysicsSystem *system,
                                               const ZJoltBodyId *bodies,
                                               uint32_t count,
                                               ZJoltBodyAddBatch **out);

/// Inserts the staged batch and destroys the handle. Must run while nothing
/// else modifies the same bodies, and not concurrently with a step.
ZJOLT_API ZJoltResult zjoltBodyAddBatchFinalize(ZJoltPhysicsSystem *system,
                                                ZJoltBodyAddBatch *batch,
                                                ZJoltActivation activation);

/// Throws the staged batch away and destroys the handle, leaving the bodies
/// created but not added — exactly where they were before the prepare.
ZJOLT_API ZJoltResult zjoltBodyAddBatchAbort(ZJoltPhysicsSystem *system,
                                             ZJoltBodyAddBatch *batch);

/// Removes bodies, leaving them created. Every id must be live and
/// currently added, else the batch is refused whole (`count` 0 = no-op).
ZJOLT_API ZJoltResult zjoltBodyRemoveBatch(ZJoltPhysicsSystem *system,
                                           const ZJoltBodyId *bodies,
                                           uint32_t count);

/// Destroys bodies, removing any still added first. IDs go stale — a later
/// call reports ZJOLT_RESULT_BODY_NOT_FOUND, not whatever body came next.
ZJOLT_API ZJoltResult zjoltBodyDestroyBatch(ZJoltPhysicsSystem *system,
                                            const ZJoltBodyId *bodies,
                                            uint32_t count);

/// Bulk zjoltBodyUnassignId: writes each id's still-alive object to
/// `out_bodies` in order (NULL if stale). Each non-NULL entry owns itself;
/// release via zjoltUnassignedBodyAssignId or zjoltUnassignedBodyDestroy.
ZJOLT_API ZJoltResult zjoltBodyUnassignIds(ZJoltPhysicsSystem *system,
                                           const ZJoltBodyId *bodies,
                                           uint32_t count,
                                           ZJoltUnassignedBody **out_bodies);

/// Batch activate: one wake-up for the whole set, worth it for an
/// explosion or trigger; zjoltBodyActivate is fine elsewhere. Ids naming
/// no live body are skipped, not refused — the rest still wake.
ZJOLT_API ZJoltResult zjoltBodyActivateBatch(ZJoltPhysicsSystem *system,
                                             const ZJoltBodyId *bodies,
                                             uint32_t count);

ZJOLT_API ZJoltResult zjoltBodyDeactivateBatch(ZJoltPhysicsSystem *system,
                                               const ZJoltBodyId *bodies,
                                               uint32_t count);

/// Wakes every body whose box overlaps `box` — same broad-phase caveats
/// as zjoltBroadPhaseCollideAABox; `filters` NULL accepts every layer.
ZJOLT_API ZJoltResult zjoltBodyActivateInBox(
    ZJoltPhysicsSystem *system, const ZJoltAABox *box,
    const ZJoltBroadPhaseFilters *filters);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_BATCH_H_
