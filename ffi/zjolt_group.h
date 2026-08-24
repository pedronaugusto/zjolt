//===----------------------------------------------------------------------===//
// zjolt — collision groups: exceptions between individual bodies.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// Object layers already answer "do these two KINDS of thing collide". A
// collision group answers the finer question layers cannot: "do these two
// PARTICULAR bodies collide", where both are the same kind. The limbs of one
// ragdoll that must not fight each other, the wheels of one vehicle, the
// crates of one stack that should pass through each other while colliding with
// every other crate — one object layer, different answers per pair.
//
// A body carries a group id, a sub-group id, and an optional filter. Two
// bodies collide when neither carries a filter, or when the first body's
// filter says they do (and if only the second has one, when the second's
// does). Filters are shared, reference-counted objects: the usual arrangement
// is one filter per ragdoll, held by every body in it.
//
// The table implementation below is Jolt's GroupFilterTable, and its rules are
// worth reading in full because two of them are easy to trip:
//
//   * a body whose group id is ZJOLT_COLLISION_GROUP_INVALID collides with
//     everything, whatever else is set;
//   * bodies with DIFFERENT group ids always collide — the table only ever
//     suppresses within one group;
//   * bodies in the same group but carrying different filter objects never
//     collide, which is a trap: two ragdolls with a filter each will pass
//     through one another if you gave them the same group id;
//   * bodies in the same group with the same sub-group id never collide;
//   * otherwise the table's bit for the pair decides.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_GROUP_H_
#define ZJOLT_GROUP_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// The group and sub-group ids that mean "no group": a body carrying them
/// collides with everything.
#define ZJOLT_COLLISION_GROUP_INVALID ((uint32_t)0xffffffffu)

/// What a body carries. `filter` may be NULL, which is the default and means
/// this body imposes no exceptions of its own.
///
/// Nothing here takes or releases a reference on `filter` by itself; the body
/// does, when the group is set on it. @see zjoltBodySetCollisionGroup
typedef struct ZJoltCollisionGroup {
  const ZJoltGroupFilter *filter;
  uint32_t group_id;
  uint32_t sub_group_id;
} ZJoltCollisionGroup;

//===----------------------------------------------------------------------===//
// The table filter
//
// One bit per unordered pair of sub-groups, all set — everything collides —
// until you clear one. That is (n * (n - 1)) / 2 bits, so a filter for a
// forty-bone ragdoll is under a hundred bytes and one for ten thousand
// sub-groups is six megabytes. Size it to one articulated object, not to a
// level.
//===----------------------------------------------------------------------===//

/// Largest `num_sub_groups` this accepts.
///
/// Not a policy: Jolt sizes the table with `(n * (n - 1)) / 2` in 32-bit
/// arithmetic, which overflows at 65536 and would allocate a table far too
/// small for the indices it then writes. One past the last value that cannot.
#define ZJOLT_GROUP_FILTER_MAX_SUB_GROUPS 65535u

/// Creates a table with `num_sub_groups` sub-groups, every pair enabled.
///
/// The filter comes back with a reference count of one, like a shape; release
/// it when the bodies that use it have taken their own.
ZJOLT_API ZJoltResult zjoltGroupFilterTableCreate(uint32_t num_sub_groups,
                                                  ZJoltGroupFilter **out);

ZJOLT_API void zjoltGroupFilterAddRef(const ZJoltGroupFilter *filter);
ZJOLT_API void zjoltGroupFilterRelease(const ZJoltGroupFilter *filter);
ZJOLT_API uint32_t zjoltGroupFilterGetRefCount(const ZJoltGroupFilter *filter);

/// How many sub-groups the filter was created with. 0 if `filter` is NULL.
ZJOLT_API uint32_t zjoltGroupFilterGetNumSubGroups(
    const ZJoltGroupFilter *filter);

/// Stops two sub-groups colliding.
///
/// Both ids must be below the filter's sub-group count and must differ. Jolt
/// asserts on either (`GroupFilterTable.h:46,52`) and, in a build without
/// asserts, indexes its bit table out of bounds — so both are checked here and
/// reported as ZJOLT_RESULT_INVALID_ARGUMENT. A sub-group never collides with
/// itself, which is why passing the same id twice is a mistake rather than a
/// no-op.
///
/// The table is symmetric: the order of the two ids does not matter.
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

//===----------------------------------------------------------------------===//
// Putting a body in a group
//===----------------------------------------------------------------------===//

/// The body takes its own reference on `group->filter`, and drops its previous
/// one. Passing NULL for `group` clears the body's group entirely, which is
/// the same as setting both ids to ZJOLT_COLLISION_GROUP_INVALID and no
/// filter.
///
/// Changing this on a body that is already simulating does not take effect
/// immediately for a pair Jolt has already cached — see
/// zjoltBodyInvalidateContactCache below.
ZJOLT_API void zjoltBodySetCollisionGroup(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltCollisionGroup *group);

/// A stale body id is NOT distinguishable from a body with no group: Jolt's
/// own getter returns `CollisionGroup::sInvalid` when the body lock fails
/// (`BodyInterface.cpp:1055`), which is exactly what a group-less body carries.
/// Ask zjoltBodyIsAdded first if the difference matters.
///
/// `out->filter` is BORROWED and takes no reference. It is valid while the
/// body holds one, which the body does until its group is changed or it is
/// destroyed; call zjoltGroupFilterAddRef if you intend to keep it.
ZJOLT_API void zjoltBodyGetCollisionGroup(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          ZJoltCollisionGroup *out);

/// Drops the cached collision result for every pair involving this body, so
/// the next step re-evaluates them.
///
/// This is here because it is what makes a collision-group change take effect
/// on a pair that is already resting. With
/// ZJoltPhysicsSettings::use_body_pair_contact_cache on — it is on by default —
/// Jolt skips the narrow phase for a pair whose relative transform has not
/// moved, and a group change moves nothing. Two bodies sitting on each other
/// would go on colliding until something disturbed them.
ZJOLT_API void zjoltBodyInvalidateContactCache(ZJoltPhysicsSystem *system,
                                               ZJoltBodyId body);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_GROUP_H_
