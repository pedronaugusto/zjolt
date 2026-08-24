//===----------------------------------------------------------------------===//
// zjolt — many bodies at once.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// zjoltBodyAdd inserts one body into the broad phase. Called in a loop over
// ten thousand of them it produces a tree that is correct and badly shaped,
// and every query pays for that until a later step rebuilds it. The batch
// insert below sorts the whole set by broad-phase layer first and builds the
// subtree in one pass, which is both faster to do and better to query
// afterwards — Jolt's own guidance is that a batch added into roughly
// unoccupied space needs no zjoltPhysicsSystemOptimizeBroadPhase at all.
//
// WHAT JOLT DOES TO THE ARRAY, and why none of it is visible here.
// `BodyInterface::AddBodiesPrepare` SORTS the caller's array in place, and the
// state it returns holds POINTERS INTO IT (`BroadPhaseQuadTree.cpp:185`) —
// so upstream the array must stay alive, at the same address, and untouched,
// from prepare until finalize. An array that looks like an input is really
// borrowed storage with a lifetime longer than the call, and nothing in the
// signature says so.
//
// zjolt does not pass that on. Prepare copies the ids into storage the batch
// handle owns, and it is that copy Jolt sorts and points into. The caller's
// array is read once and never touched again; it may be a temporary, it may be
// `const`, and finalize does not want it back. `RemoveBodies` shuffles its
// argument too, and is treated the same way.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_BATCH_H_
#define ZJOLT_BATCH_H_

#include "zjolt_core.h"
// For ZJoltBroadPhaseFilters, which zjoltBodyActivateInBox takes because it
// runs a broad-phase query to find what to wake.
#include "zjolt_broadphase.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Batch insert
//===----------------------------------------------------------------------===//

/// Creates and adds bodies in one call. This is the one to reach for.
///
/// Equivalent to prepare followed immediately by finalize, and the right shape
/// unless the sorting is worth moving off the calling thread. Every id must
/// name a live body that is NOT already added; Jolt asserts on one that is,
/// and in a build without asserts it corrupts the broad phase's bookkeeping,
/// so this checks first and refuses the whole batch rather than adding half
/// of it.
///
/// `count` of 0 succeeds and does nothing.
ZJOLT_API ZJoltResult zjoltBodyAddBatch(ZJoltPhysicsSystem *system,
                                        const ZJoltBodyId *bodies,
                                        uint32_t count,
                                        ZJoltActivation activation);

/// Sorts and stages a batch without touching the physics system's own state.
///
/// This is the half that can run on a worker thread while the simulation
/// steps; the finalize is the half that cannot. The handle it yields must be
/// passed to exactly one of zjoltBodyAddBatchFinalize or zjoltBodyAddBatchAbort
/// — until then the bodies are marked as belonging to a broad-phase layer but
/// are not in the tree, and no query will find them.
///
/// The batch belongs to the system: one still outstanding when the system is
/// destroyed is aborted and freed with it, so it is not counted by
/// zjoltLiveHandleCount.
///
/// `bodies` is read and released; see the note at the top of this header.
ZJOLT_API ZJoltResult zjoltBodyAddBatchPrepare(ZJoltPhysicsSystem *system,
                                               const ZJoltBodyId *bodies,
                                               uint32_t count,
                                               ZJoltBodyAddBatch **out);

/// Inserts the staged batch and destroys the handle.
///
/// Must run while nothing else is modifying the same bodies, and not
/// concurrently with a step.
ZJOLT_API ZJoltResult zjoltBodyAddBatchFinalize(ZJoltPhysicsSystem *system,
                                                ZJoltBodyAddBatch *batch,
                                                ZJoltActivation activation);

/// Throws the staged batch away and destroys the handle, leaving the bodies
/// created but not added — exactly where they were before the prepare.
ZJOLT_API ZJoltResult zjoltBodyAddBatchAbort(ZJoltPhysicsSystem *system,
                                             ZJoltBodyAddBatch *batch);

//===----------------------------------------------------------------------===//
// Batch remove and destroy
//===----------------------------------------------------------------------===//

/// Removes bodies from the simulation, leaving them created.
///
/// Every id must name a live body that IS currently added; Jolt asserts
/// otherwise, so this checks first and refuses the whole batch. `count` of 0
/// succeeds and does nothing.
ZJOLT_API ZJoltResult zjoltBodyRemoveBatch(ZJoltPhysicsSystem *system,
                                           const ZJoltBodyId *bodies,
                                           uint32_t count);

/// Destroys bodies, removing any that are still added first.
///
/// The ids go stale; a later call with one reports
/// ZJOLT_RESULT_BODY_NOT_FOUND rather than reaching whatever body was created
/// next.
ZJOLT_API ZJoltResult zjoltBodyDestroyBatch(ZJoltPhysicsSystem *system,
                                            const ZJoltBodyId *bodies,
                                            uint32_t count);

//===----------------------------------------------------------------------===//
// Batch activate
//
// One wake-up for the whole set rather than one per body. Worth it where an
// explosion or a trigger wakes a region at once; the single-body
// zjoltBodyActivate is fine everywhere else.
//===----------------------------------------------------------------------===//

/// Ids that do not name a live body are skipped, not refused: a body destroyed
/// since the list was built is ordinary, and waking the rest is what the
/// caller meant.
ZJOLT_API ZJoltResult zjoltBodyActivateBatch(ZJoltPhysicsSystem *system,
                                             const ZJoltBodyId *bodies,
                                             uint32_t count);

ZJOLT_API ZJoltResult zjoltBodyDeactivateBatch(ZJoltPhysicsSystem *system,
                                               const ZJoltBodyId *bodies,
                                               uint32_t count);

/// Wakes every body whose bounding box overlaps `box`.
///
/// The box is float world space, and the test is against bounding boxes — the
/// same broad-phase caveats as zjoltBroadPhaseCollideAABox, for the same
/// reason: this runs the same query. `filters` may be NULL to accept every
/// layer.
ZJOLT_API ZJoltResult zjoltBodyActivateInBox(
    ZJoltPhysicsSystem *system, const ZJoltAABox *box,
    const ZJoltBroadPhaseFilters *filters);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_BATCH_H_
