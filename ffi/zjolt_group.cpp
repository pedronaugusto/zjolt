//===----------------------------------------------------------------------===//
// zjolt — collision groups.
//
// The handle itself — a thin subclass of JPH::GroupFilter that HOLDS a
// GroupFilterTable rather than being one, and why — is declared in
// zjolt_internal.h. It lives there rather than here because a ZJoltBodyDesc
// carries a ZJoltCollisionGroup, so the translation units that build a body
// need the complete type too.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

namespace {

/// The two arguments every table mutator shares, checked in one place.
///
/// Both checks stand in for a Jolt assertion. Equal ids are refused rather
/// than ignored because a sub-group never collides with itself whatever the
/// table says, so asking to change that pair is a mistake and not a no-op.
ZJoltResult CheckSubGroups(const ZJoltGroupFilter *filter, uint32_t sub_group1,
                           uint32_t sub_group2) {
  if (sub_group1 == sub_group2) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a sub-group never collides with itself, so the two sub-group ids "
        "must differ");
  }
  if (sub_group1 >= filter->num_sub_groups ||
      sub_group2 >= filter->num_sub_groups) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "sub-group id is outside the range this filter was created with");
  }
  return ZJOLT_RESULT_OK;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// The table filter
//===----------------------------------------------------------------------===//

ZJoltResult zjoltGroupFilterTableCreate(uint32_t num_sub_groups,
                                        ZJoltGroupFilter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Jolt sizes the bit table with `(n * (n - 1)) / 2` in 32-bit arithmetic.
  // At n = 65536 that product wraps, and the table comes out far too small
  // for the indices the filter then writes into it — an out-of-bounds write
  // with no assertion anywhere near it.
  if (num_sub_groups > ZJOLT_GROUP_FILTER_MAX_SUB_GROUPS) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "num_sub_groups exceeds 65535, past which Jolt's table sizing "
        "overflows");
  }

  ZJoltGroupFilter *filter = zjolt::New<ZJoltGroupFilter>(num_sub_groups);
  if (filter == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  // Own() is spelled on the RefTarget base: the reference count lives in
  // RefTarget<GroupFilter>, not in RefTarget<ZJoltGroupFilter>, and Own's
  // static_assert is what says so.
  *out = static_cast<ZJoltGroupFilter *>(zjolt::Own<JPH::GroupFilter>(filter));
  return ZJOLT_RESULT_OK;
}

void zjoltGroupFilterAddRef(const ZJoltGroupFilter *filter) {
  if (filter == nullptr) return;
  filter->AddRef();
}

void zjoltGroupFilterRelease(const ZJoltGroupFilter *filter) {
  if (filter == nullptr) return;
  filter->Release();
}

uint32_t zjoltGroupFilterGetRefCount(const ZJoltGroupFilter *filter) {
  if (filter == nullptr) return 0;
  return filter->GetRefCount();
}

uint32_t zjoltGroupFilterGetNumSubGroups(const ZJoltGroupFilter *filter) {
  if (filter == nullptr) return 0;
  return static_cast<uint32_t>(filter->num_sub_groups);
}

ZJoltResult zjoltGroupFilterTableDisableCollision(ZJoltGroupFilter *filter,
                                                  uint32_t sub_group1,
                                                  uint32_t sub_group2) {
  ZJOLT_ENTER();
  if (!zjolt::Present(filter)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const ZJoltResult checked = CheckSubGroups(filter, sub_group1, sub_group2);
  if (checked != ZJOLT_RESULT_OK) return checked;

  filter->table.DisableCollision(sub_group1, sub_group2);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltGroupFilterTableEnableCollision(ZJoltGroupFilter *filter,
                                                 uint32_t sub_group1,
                                                 uint32_t sub_group2) {
  ZJOLT_ENTER();
  if (!zjolt::Present(filter)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const ZJoltResult checked = CheckSubGroups(filter, sub_group1, sub_group2);
  if (checked != ZJOLT_RESULT_OK) return checked;

  filter->table.EnableCollision(sub_group1, sub_group2);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltGroupFilterTableIsCollisionEnabled(
    const ZJoltGroupFilter *filter, uint32_t sub_group1, uint32_t sub_group2,
    bool *out_enabled) {
  ZJOLT_ENTER(out_enabled);
  if (!zjolt::Present(filter, out_enabled))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const ZJoltResult checked = CheckSubGroups(filter, sub_group1, sub_group2);
  if (checked != ZJOLT_RESULT_OK) return checked;

  *out_enabled = filter->table.IsCollisionEnabled(sub_group1, sub_group2);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Putting a body in a group
//===----------------------------------------------------------------------===//

void zjoltBodySetCollisionGroup(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                const ZJoltCollisionGroup *group) {
  if (system == nullptr) return;

  // A NULL `group` converts to the default-constructed CollisionGroup, which
  // already means "no group, no filter".
  const JPH::CollisionGroup jolt = zjolt::ToJolt(group);
  // Takes a body write lock and does nothing if the id is stale, so this is
  // safe to call with an id that has gone.
  system->system.GetBodyInterface().SetCollisionGroup(zjolt::ToJolt(body),
                                                      jolt);
}

void zjoltBodyGetCollisionGroup(const ZJoltPhysicsSystem *system,
                                ZJoltBodyId body, ZJoltCollisionGroup *out) {
  if (out == nullptr) return;
  *out = ZJoltCollisionGroup{nullptr, ZJOLT_COLLISION_GROUP_INVALID,
                             ZJOLT_COLLISION_GROUP_INVALID};
  if (system == nullptr) return;

  // Jolt returns CollisionGroup::sInvalid when the body lock fails, and
  // sInvalid is what a body with no group carries — so a stale id and a
  // group-less body are the same answer here. That is upstream's choice, and
  // it is documented rather than papered over.
  const JPH::CollisionGroup &group =
      system->system.GetBodyInterface().GetCollisionGroup(zjolt::ToJolt(body));

  // The downcast is sound because the only filter that can reach a body
  // through this ABI came out of zjoltGroupFilterTableCreate.
  out->filter = static_cast<const ZJoltGroupFilter *>(group.GetGroupFilter());
  out->group_id = group.GetGroupID();
  out->sub_group_id = group.GetSubGroupID();
}

void zjoltBodyInvalidateContactCache(ZJoltPhysicsSystem *system,
                                     ZJoltBodyId body) {
  if (system == nullptr) return;
  system->system.GetBodyInterface().InvalidateContactCache(zjolt::ToJolt(body));
}

}  // extern "C"
