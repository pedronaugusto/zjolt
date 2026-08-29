//===----------------------------------------------------------------------===//
// zjolt — collision groups: finer than object layers. Different groups
// always collide; a group only suppresses pairs WITHIN itself. A body
// carries a group id, sub-group id, and an optional ref-counted filter.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_GROUP_H_
#define ZJOLT_GROUP_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// The group/sub-group ids meaning "no group": collides with everything.
#define ZJOLT_COLLISION_GROUP_INVALID ((uint32_t)0xffffffffu)

/// What a body carries. `filter` NULL (default) means no exceptions of its
/// own; not owned here — the body takes its own ref once the group is set.
typedef struct ZJoltCollisionGroup {
  const ZJoltGroupFilter *filter;
  uint32_t group_id;
  uint32_t sub_group_id;
} ZJoltCollisionGroup;

//===----------------------------------------------------------------------===//
// The table filter: one bit per sub-group pair, all set until cleared.
// Same group + different filter objects never collide — give each
// ragdoll its own group id, not a shared one with separate filters.
//===----------------------------------------------------------------------===//

/// Largest `num_sub_groups` this accepts (65536 overflows 32-bit table sizing).
#define ZJOLT_GROUP_FILTER_MAX_SUB_GROUPS 65535u

/// Creates a table with `num_sub_groups` sub-groups, every pair enabled.
/// Ref count one; release once the bodies that use it have taken their own.
ZJOLT_API ZJoltResult zjoltGroupFilterTableCreate(uint32_t num_sub_groups,
                                                  ZJoltGroupFilter **out);

ZJOLT_API void zjoltGroupFilterAddRef(const ZJoltGroupFilter *filter);
ZJOLT_API void zjoltGroupFilterRelease(const ZJoltGroupFilter *filter);
ZJOLT_API uint32_t zjoltGroupFilterGetRefCount(const ZJoltGroupFilter *filter);

/// How many sub-groups the filter was created with. 0 if `filter` is NULL.
ZJOLT_API uint32_t zjoltGroupFilterGetNumSubGroups(
    const ZJoltGroupFilter *filter);

/// Stops two sub-groups colliding (below the filter's count, differing),
/// else ZJOLT_RESULT_INVALID_ARGUMENT; symmetric — order doesn't matter.
ZJOLT_API ZJoltResult zjoltGroupFilterTableDisableCollision(
    ZJoltGroupFilter *filter, uint32_t sub_group1, uint32_t sub_group2);

/// Puts a pair back. Same argument rules as the disable above.
ZJOLT_API ZJoltResult zjoltGroupFilterTableEnableCollision(
    ZJoltGroupFilter *filter, uint32_t sub_group1, uint32_t sub_group2);

/// `*out_enabled` receives whether the pair collides. Same argument rules
/// again; on a refusal `*out_enabled` is false rather than left alone.
ZJOLT_API ZJoltResult zjoltGroupFilterTableIsCollisionEnabled(
    const ZJoltGroupFilter *filter, uint32_t sub_group1, uint32_t sub_group2,
    bool *out_enabled);

/// Takes a reference on `group->filter`, drops the previous one; NULL clears
/// the group (same as INVALID ids, no filter). No effect on an
/// already-cached pair — @see zjoltBodyInvalidateContactCache.
ZJOLT_API void zjoltBodySetCollisionGroup(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltCollisionGroup *group);

/// A stale body id reports no group, same as group-less — ask
/// zjoltBodyIsAdded first if it matters. `out->filter` is BORROWED, valid
/// until the group changes or body dies; @see zjoltGroupFilterAddRef.
ZJOLT_API void zjoltBodyGetCollisionGroup(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          ZJoltCollisionGroup *out);

/// Drops the cached collision result for this body's pairs, forcing
/// re-evaluation. Call after a group change on an already-resting pair —
/// contact caching (default on) would otherwise keep the stale result.
ZJOLT_API void zjoltBodyInvalidateContactCache(ZJoltPhysicsSystem *system,
                                               ZJoltBodyId body);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_GROUP_H_
