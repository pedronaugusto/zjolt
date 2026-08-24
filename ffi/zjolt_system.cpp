//===----------------------------------------------------------------------===//
// zjolt — the physics system: layers, listeners, the step, and bulk read-back.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Core/Mutex.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsStepListener.h>

namespace {

/// Largest manifold Jolt will ever hand a listener. Asserted against Jolt's
/// own bound below, so a future upstream change is a build failure rather than
/// a silently truncated manifold.
constexpr uint32_t kMaxContactPoints = 64;
static_assert(JPH::ContactPoints::Capacity >= kMaxContactPoints,
              "Jolt's contact point capacity shrank below what zjolt copies");

/// Copies Jolt's SIMD contact points into caller-visible 12-byte vectors.
///
/// The copy is unavoidable: Jolt's ContactPoints holds 16-byte Vec3s with a
/// padding lane, and the ABI's ZJoltVec3 is three floats. Doing it on the
/// stack keeps a contact callback allocation-free, which matters because these
/// run on Jolt's job threads inside the step.
uint32_t CopyContactPoints(const JPH::ContactPoints &in, ZJoltVec3 *out) {
  const uint32_t count =
      static_cast<uint32_t>(in.size()) < kMaxContactPoints
          ? static_cast<uint32_t>(in.size())
          : kMaxContactPoints;
  for (uint32_t i = 0; i < count; ++i) out[i] = zjolt::ToC(in[i]);
  return count;
}

void FillContactInfo(ZJoltContactInfo *info, const JPH::Body &body1,
                     const JPH::Body &body2,
                     const JPH::ContactManifold &manifold,
                     ZJoltVec3 *points1, ZJoltVec3 *points2) {
  info->body1 = zjolt::ToC(body1.GetID());
  info->body2 = zjolt::ToC(body2.GetID());
  info->user_data1 = body1.GetUserData();
  info->user_data2 = body2.GetUserData();

  ZJoltContactManifold &out = info->manifold;
  out.base_offset = zjolt::ToCR(manifold.mBaseOffset);
  out.world_space_normal = zjolt::ToC(manifold.mWorldSpaceNormal);
  out.penetration_depth = manifold.mPenetrationDepth;
  out.sub_shape_id1 = zjolt::ToC(manifold.mSubShapeID1);
  out.sub_shape_id2 = zjolt::ToC(manifold.mSubShapeID2);
  out.num_points = CopyContactPoints(manifold.mRelativeContactPointsOn1, points1);
  CopyContactPoints(manifold.mRelativeContactPointsOn2, points2);
  out.points_on_1 = points1;
  out.points_on_2 = points2;
}

void ToCContactSettings(const JPH::ContactSettings &in,
                        ZJoltContactSettings *out) {
  out->combined_friction = in.mCombinedFriction;
  out->combined_restitution = in.mCombinedRestitution;
  out->inv_mass_scale1 = in.mInvMassScale1;
  out->inv_inertia_scale1 = in.mInvInertiaScale1;
  out->inv_mass_scale2 = in.mInvMassScale2;
  out->inv_inertia_scale2 = in.mInvInertiaScale2;
  out->is_sensor = in.mIsSensor;
  out->relative_linear_surface_velocity =
      zjolt::ToC(in.mRelativeLinearSurfaceVelocity);
  out->relative_angular_surface_velocity =
      zjolt::ToC(in.mRelativeAngularSurfaceVelocity);
}

void ToJoltContactSettings(const ZJoltContactSettings &in,
                           JPH::ContactSettings &out) {
  out.mCombinedFriction = in.combined_friction;
  out.mCombinedRestitution = in.combined_restitution;
  out.mInvMassScale1 = in.inv_mass_scale1;
  out.mInvInertiaScale1 = in.inv_inertia_scale1;
  out.mInvMassScale2 = in.inv_mass_scale2;
  out.mInvInertiaScale2 = in.inv_inertia_scale2;
  out.mIsSensor = in.is_sensor;
  out.mRelativeLinearSurfaceVelocity =
      zjolt::ToJolt(in.relative_linear_surface_velocity);
  out.mRelativeAngularSurfaceVelocity =
      zjolt::ToJolt(in.relative_angular_surface_velocity);
}

/// Shared tail of zjoltPhysicsSystemGetBodies / GetActiveBodies: copies a
/// Jolt id vector into the caller's array under the two-call protocol.
ZJoltResult CopyBodyIds(const JPH::BodyIDVector &ids, ZJoltBodyId *out_ids,
                        uint32_t capacity, uint32_t *out_count) {
  const uint32_t count = static_cast<uint32_t>(ids.size());
  *out_count = count;
  if (out_ids == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) out_ids[i] = zjolt::ToC(ids[i]);
  return ZJOLT_RESULT_OK;
}

}  // namespace

//===----------------------------------------------------------------------===//
// Contact listener adapter
//
// Declared in zjolt_internal.h; defined here because this is the translation
// unit that owns the system it is installed on.
//===----------------------------------------------------------------------===//

JPH::ValidateResult ZJoltContactListenerAdapter::OnContactValidate(
    const JPH::Body &inBody1, const JPH::Body &inBody2,
    JPH::RVec3Arg inBaseOffset,
    const JPH::CollideShapeResult &inCollisionResult) {
  if (listener_.on_contact_validate == nullptr)
    return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;

  ZJoltContactValidateInfo info;
  info.body1 = zjolt::ToC(inBody1.GetID());
  info.body2 = zjolt::ToC(inBody2.GetID());
  info.user_data1 = inBody1.GetUserData();
  info.user_data2 = inBody2.GetUserData();
  info.base_offset = zjolt::ToCR(inBaseOffset);
  info.contact_point_on_1 = zjolt::ToC(inCollisionResult.mContactPointOn1);
  info.contact_point_on_2 = zjolt::ToC(inCollisionResult.mContactPointOn2);
  info.penetration_axis = zjolt::ToC(inCollisionResult.mPenetrationAxis);
  info.penetration_depth = inCollisionResult.mPenetrationDepth;
  info.sub_shape_id1 = zjolt::ToC(inCollisionResult.mSubShapeID1);
  info.sub_shape_id2 = zjolt::ToC(inCollisionResult.mSubShapeID2);

  switch (listener_.on_contact_validate(listener_.user, &info)) {
    case ZJOLT_VALIDATE_RESULT_ACCEPT_CONTACT:
      return JPH::ValidateResult::AcceptContact;
    case ZJOLT_VALIDATE_RESULT_REJECT_CONTACT:
      return JPH::ValidateResult::RejectContact;
    case ZJOLT_VALIDATE_RESULT_REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR:
      return JPH::ValidateResult::RejectAllContactsForThisBodyPair;
    case ZJOLT_VALIDATE_RESULT_ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR:
      break;
  }
  return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
}

void ZJoltContactListenerAdapter::OnContactAdded(
    const JPH::Body &inBody1, const JPH::Body &inBody2,
    const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) {
  if (listener_.on_contact_added == nullptr) return;

  ZJoltVec3 points1[kMaxContactPoints];
  ZJoltVec3 points2[kMaxContactPoints];
  ZJoltContactInfo info;
  FillContactInfo(&info, inBody1, inBody2, inManifold, points1, points2);

  ZJoltContactSettings settings;
  ToCContactSettings(ioSettings, &settings);
  listener_.on_contact_added(listener_.user, &info, &settings);
  ToJoltContactSettings(settings, ioSettings);
}

void ZJoltContactListenerAdapter::OnContactPersisted(
    const JPH::Body &inBody1, const JPH::Body &inBody2,
    const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) {
  if (listener_.on_contact_persisted == nullptr) return;

  ZJoltVec3 points1[kMaxContactPoints];
  ZJoltVec3 points2[kMaxContactPoints];
  ZJoltContactInfo info;
  FillContactInfo(&info, inBody1, inBody2, inManifold, points1, points2);

  ZJoltContactSettings settings;
  ToCContactSettings(ioSettings, &settings);
  listener_.on_contact_persisted(listener_.user, &info, &settings);
  ToJoltContactSettings(settings, ioSettings);
}

void ZJoltContactListenerAdapter::OnContactRemoved(
    const JPH::SubShapeIDPair &inSubShapePair) {
  if (listener_.on_contact_removed == nullptr) return;
  const ZJoltSubShapeIdPair pair = {
      zjolt::ToC(inSubShapePair.GetBody1ID()),
      zjolt::ToC(inSubShapePair.GetSubShapeID1()),
      zjolt::ToC(inSubShapePair.GetBody2ID()),
      zjolt::ToC(inSubShapePair.GetSubShapeID2()),
  };
  listener_.on_contact_removed(listener_.user, &pair);
}

//===----------------------------------------------------------------------===//
// Step listener adapter
//
// Global namespace, because the C header names it as an opaque handle and the
// tag has to match. Owned by the system it is attached to.
//===----------------------------------------------------------------------===//

struct ZJoltStepListener final : public JPH::PhysicsStepListener {
  ZJoltStepListener(ZJoltStepListenerFn callback, void *user)
      : callback_(callback), user_(user) {}

  void OnStep(const JPH::PhysicsStepListenerContext &inContext) override {
    // inContext also carries the PhysicsSystem being stepped. It is not
    // forwarded: a listener runs with every body and constraint mutex held,
    // so all it could do with the system is deadlock, and handing it over
    // would suggest otherwise.
    const ZJoltStepListenerContext context = {
        inContext.mDeltaTime,
        inContext.mIsFirstStep,
        inContext.mIsLastStep,
    };
    callback_(user_, &context);
  }

 private:
  ZJoltStepListenerFn callback_;
  void *user_;
};

namespace {

//===----------------------------------------------------------------------===//
// Combine callbacks
//
// Jolt's hook is `float (*)(const Body &, const SubShapeID &, const Body &,
// const SubShapeID &)` — a bare function pointer with NO user parameter. There
// is nothing in its arguments that leads back to the physics system either, so
// a single shared trampoline could not tell which system it was called for.
//
// The only thing left to distinguish two systems by is WHICH function pointer
// they installed. So there is a fixed table of slots and one trampoline
// instantiated per slot; a system takes a slot the first time it installs a
// combine callback and gives it back when it is destroyed.
//
// The alternative would be a global pair of callbacks shared by every system,
// which is not a binding of a per-system setting.
//===----------------------------------------------------------------------===//

struct CombineSlot {
  ZJoltCombineFn friction = nullptr;
  void *friction_user = nullptr;
  ZJoltCombineFn restitution = nullptr;
  void *restitution_user = nullptr;
  bool taken = false;
};

CombineSlot g_combine_slots[ZJOLT_COMBINE_SLOT_COUNT];

/// Guards which slots are taken, and nothing else.
///
/// The callback fields are read on Jolt's job threads during a step and
/// written only by an install, which the ABI already says must not run
/// concurrently with a step on the same system. What this protects is the
/// table itself, which is process-wide: two threads creating two unrelated
/// systems would otherwise be able to claim the same slot.
JPH::Mutex g_combine_mutex;

void FillCombineInfo(ZJoltCombineInfo *info, const JPH::Body &body1,
                     const JPH::SubShapeID &sub_shape1, const JPH::Body &body2,
                     const JPH::SubShapeID &sub_shape2, float value1,
                     float value2) {
  info->body1 = zjolt::ToC(body1.GetID());
  info->sub_shape_id1 = zjolt::ToC(sub_shape1);
  info->user_data1 = body1.GetUserData();
  info->value1 = value1;
  info->body2 = zjolt::ToC(body2.GetID());
  info->sub_shape_id2 = zjolt::ToC(sub_shape2);
  info->user_data2 = body2.GetUserData();
  info->value2 = value2;
}

template <int Slot>
float CombineFrictionThunk(const JPH::Body &body1,
                           const JPH::SubShapeID &sub_shape1,
                           const JPH::Body &body2,
                           const JPH::SubShapeID &sub_shape2) {
  const CombineSlot &slot = g_combine_slots[Slot];
  const float value1 = body1.GetFriction();
  const float value2 = body2.GetFriction();
  if (slot.friction == nullptr) return std::sqrt(value1 * value2);

  ZJoltCombineInfo info;
  FillCombineInfo(&info, body1, sub_shape1, body2, sub_shape2, value1, value2);
  return slot.friction(slot.friction_user, &info);
}

template <int Slot>
float CombineRestitutionThunk(const JPH::Body &body1,
                              const JPH::SubShapeID &sub_shape1,
                              const JPH::Body &body2,
                              const JPH::SubShapeID &sub_shape2) {
  const CombineSlot &slot = g_combine_slots[Slot];
  const float value1 = body1.GetRestitution();
  const float value2 = body2.GetRestitution();
  if (slot.restitution == nullptr) return value1 > value2 ? value1 : value2;

  ZJoltCombineInfo info;
  FillCombineInfo(&info, body1, sub_shape1, body2, sub_shape2, value1, value2);
  return slot.restitution(slot.restitution_user, &info);
}

using JoltCombineFn = ZJoltPhysicsSystem::CombineFunction;

struct CombineThunkTable {
  JoltCombineFn fns[ZJOLT_COMBINE_SLOT_COUNT];
};

template <int... Slots>
constexpr CombineThunkTable MakeFrictionThunks(
    std::integer_sequence<int, Slots...>) {
  return CombineThunkTable{{CombineFrictionThunk<Slots>...}};
}

template <int... Slots>
constexpr CombineThunkTable MakeRestitutionThunks(
    std::integer_sequence<int, Slots...>) {
  return CombineThunkTable{{CombineRestitutionThunk<Slots>...}};
}

const CombineThunkTable kFrictionThunks = MakeFrictionThunks(
    std::make_integer_sequence<int, ZJOLT_COMBINE_SLOT_COUNT>{});
const CombineThunkTable kRestitutionThunks = MakeRestitutionThunks(
    std::make_integer_sequence<int, ZJOLT_COMBINE_SLOT_COUNT>{});

/// Gives `system` a slot if it has none. -1 when the table is full.
int AcquireCombineSlot(ZJoltPhysicsSystem *system) {
  if (system->combine_slot >= 0) return system->combine_slot;

  JPH::lock_guard<JPH::Mutex> lock(g_combine_mutex);
  for (int i = 0; i < ZJOLT_COMBINE_SLOT_COUNT; ++i) {
    if (g_combine_slots[i].taken) continue;
    g_combine_slots[i] = CombineSlot{};
    g_combine_slots[i].taken = true;
    system->combine_slot = i;
    return i;
  }
  return -1;
}

void ReleaseCombineSlot(ZJoltPhysicsSystem *system) {
  if (system->combine_slot < 0) return;

  JPH::lock_guard<JPH::Mutex> lock(g_combine_mutex);
  g_combine_slots[system->combine_slot] = CombineSlot{};
  system->combine_slot = -1;
}

//===----------------------------------------------------------------------===//
// Simulation settings
//===----------------------------------------------------------------------===//

void ToCSettings(const JPH::PhysicsSettings &in, ZJoltPhysicsSettings *out) {
  out->max_in_flight_body_pairs = in.mMaxInFlightBodyPairs;
  out->step_listeners_batch_size = in.mStepListenersBatchSize;
  out->step_listener_batches_per_job = in.mStepListenerBatchesPerJob;
  out->baumgarte = in.mBaumgarte;
  out->speculative_contact_distance = in.mSpeculativeContactDistance;
  out->penetration_slop = in.mPenetrationSlop;
  out->linear_cast_threshold = in.mLinearCastThreshold;
  out->linear_cast_max_penetration = in.mLinearCastMaxPenetration;
  out->manifold_tolerance = in.mManifoldTolerance;
  out->max_penetration_distance = in.mMaxPenetrationDistance;
  out->body_pair_cache_max_delta_position_sq =
      in.mBodyPairCacheMaxDeltaPositionSq;
  out->body_pair_cache_cos_max_delta_rotation_div2 =
      in.mBodyPairCacheCosMaxDeltaRotationDiv2;
  out->contact_normal_cos_max_delta_rotation =
      in.mContactNormalCosMaxDeltaRotation;
  out->contact_point_preserve_lambda_max_dist_sq =
      in.mContactPointPreserveLambdaMaxDistSq;
  out->internal_edge_removal_vertex_tolerance_sq =
      in.mInternalEdgeRemovalVertexToleranceSq;
  out->num_velocity_steps = in.mNumVelocitySteps;
  out->num_position_steps = in.mNumPositionSteps;
  out->min_velocity_for_restitution = in.mMinVelocityForRestitution;
  out->time_before_sleep = in.mTimeBeforeSleep;
  out->point_velocity_sleep_threshold = in.mPointVelocitySleepThreshold;
  out->deterministic_simulation = in.mDeterministicSimulation;
  out->constraint_warm_start = in.mConstraintWarmStart;
  out->use_body_pair_contact_cache = in.mUseBodyPairContactCache;
  out->use_manifold_reduction = in.mUseManifoldReduction;
  out->use_large_island_splitter = in.mUseLargeIslandSplitter;
  out->allow_sleeping = in.mAllowSleeping;
  out->check_active_edges = in.mCheckActiveEdges;
}

void ToJoltSettings(const ZJoltPhysicsSettings &in, JPH::PhysicsSettings &out) {
  out.mMaxInFlightBodyPairs = in.max_in_flight_body_pairs;
  out.mStepListenersBatchSize = in.step_listeners_batch_size;
  out.mStepListenerBatchesPerJob = in.step_listener_batches_per_job;
  out.mBaumgarte = in.baumgarte;
  out.mSpeculativeContactDistance = in.speculative_contact_distance;
  out.mPenetrationSlop = in.penetration_slop;
  out.mLinearCastThreshold = in.linear_cast_threshold;
  out.mLinearCastMaxPenetration = in.linear_cast_max_penetration;
  out.mManifoldTolerance = in.manifold_tolerance;
  out.mMaxPenetrationDistance = in.max_penetration_distance;
  out.mBodyPairCacheMaxDeltaPositionSq =
      in.body_pair_cache_max_delta_position_sq;
  out.mBodyPairCacheCosMaxDeltaRotationDiv2 =
      in.body_pair_cache_cos_max_delta_rotation_div2;
  out.mContactNormalCosMaxDeltaRotation =
      in.contact_normal_cos_max_delta_rotation;
  out.mContactPointPreserveLambdaMaxDistSq =
      in.contact_point_preserve_lambda_max_dist_sq;
  out.mInternalEdgeRemovalVertexToleranceSq =
      in.internal_edge_removal_vertex_tolerance_sq;
  out.mNumVelocitySteps = in.num_velocity_steps;
  out.mNumPositionSteps = in.num_position_steps;
  out.mMinVelocityForRestitution = in.min_velocity_for_restitution;
  out.mTimeBeforeSleep = in.time_before_sleep;
  out.mPointVelocitySleepThreshold = in.point_velocity_sleep_threshold;
  out.mDeterministicSimulation = in.deterministic_simulation;
  out.mConstraintWarmStart = in.constraint_warm_start;
  out.mUseBodyPairContactCache = in.use_body_pair_contact_cache;
  out.mUseManifoldReduction = in.use_manifold_reduction;
  out.mUseLargeIslandSplitter = in.use_large_island_splitter;
  out.mAllowSleeping = in.allow_sleeping;
  out.mCheckActiveEdges = in.check_active_edges;
}

/// The settings Jolt would divide by, loop on, or index with.
///
/// None of these is asserted upstream, and each one fails somewhere other than
/// here: a zero batch size makes the step-listener job loop spin forever
/// because `fetch_add(0)` never advances the read index
/// (`PhysicsSystem.cpp:702`), a zero batches-per-job is an integer division
/// (`PhysicsSystem.cpp:243`), and a non-positive in-flight pair count is a
/// buffer length passed to the temp allocator (`PhysicsSystem.cpp:232`).
const char *WhySettingsRefused(const ZJoltPhysicsSettings &settings) {
  if (settings.max_in_flight_body_pairs < 1)
    return "max_in_flight_body_pairs must be at least 1";
  if (settings.step_listeners_batch_size < 1)
    return "step_listeners_batch_size must be at least 1, or the step's "
           "listener loop never advances";
  if (settings.step_listener_batches_per_job < 1)
    return "step_listener_batches_per_job must be at least 1; it is a divisor";
  if (settings.num_velocity_steps < 1)
    return "num_velocity_steps must be at least 1, and at least 2 for friction";
  if (!(settings.min_velocity_for_restitution >= 0.0f))
    return "min_velocity_for_restitution must not be negative";
  if (!(settings.point_velocity_sleep_threshold >= 0.0f))
    return "point_velocity_sleep_threshold must not be negative";
  if (!(settings.time_before_sleep >= 0.0f))
    return "time_before_sleep must not be negative";
  return nullptr;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Lifetime
//===----------------------------------------------------------------------===//

void zjoltPhysicsSystemDescInit(ZJoltPhysicsSystemDesc *desc) {
  if (desc == nullptr) return;
  *desc = ZJoltPhysicsSystemDesc{};
  desc->max_bodies = 10240;
  desc->num_body_mutexes = 0;
  desc->max_body_pairs = 65536;
  desc->max_contact_constraints = 65536;
  desc->temp_allocator_size = 10 * 1024 * 1024;
}

ZJoltResult zjoltPhysicsSystemCreate(const ZJoltPhysicsSystemDesc *desc,
                                     ZJoltPhysicsSystem **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (desc->max_bodies == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "max_bodies must be positive");
  }
  // Jolt packs a body's index and a generation counter into one 32-bit id, so
  // there is a hard ceiling on how many bodies can exist. Past it, Jolt
  // asserts while handing out an id — long after the mistake, and fatally.
  // Reproduced with max_bodies = 100,000,000.
  if (desc->max_bodies > JPH::BodyID::cMaxBodyIndex + 1u) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "max_bodies exceeds Jolt's hard limit of 8388608 bodies");
  }
  if (desc->max_contact_constraints >
      JPH::ContactConstraintManager::cMaxContactConstraintsLimit) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "max_contact_constraints exceeds what Jolt can address");
  }
  // The two required questions. Jolt would dereference a null vtable entry
  // during the first broad-phase build; failing here names the missing piece.
  if (desc->broad_phase_layers.num_broad_phase_layers == nullptr ||
      desc->broad_phase_layers.broad_phase_layer_for_object_layer == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "broad_phase_layers needs num_broad_phase_layers and "
        "broad_phase_layer_for_object_layer");
  }

  ZJoltPhysicsSystem *system = zjolt::New<ZJoltPhysicsSystem>(*desc);
  if (system == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  const size_t temp_size = desc->temp_allocator_size != 0
                               ? desc->temp_allocator_size
                               : 10 * 1024 * 1024;
  system->temp_allocator =
      zjolt::New<JPH::TempAllocatorImplWithMallocFallback>(
          static_cast<JPH::uint>(temp_size));
  if (system->temp_allocator == nullptr) {
    zjolt::Delete(system);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }

  system->system.Init(desc->max_bodies, desc->num_body_mutexes,
                      desc->max_body_pairs, desc->max_contact_constraints,
                      system->broad_phase_layers,
                      system->object_vs_broad_phase_filter,
                      system->object_layer_pair_filter);

  // Captured now, because clearing a combine callback later has to put
  // something back and `SetCombineFriction(nullptr)` is not it — the solver
  // calls the pointer unconditionally. Reading Jolt's own defaults out of a
  // freshly initialised system is the only way to restore them exactly.
  system->default_combine_friction = system->system.GetCombineFriction();
  system->default_combine_restitution = system->system.GetCombineRestitution();

  zjolt::HandleCreated();
  *out = system;
  return ZJOLT_RESULT_OK;
}

void zjoltPhysicsSystemDestroy(ZJoltPhysicsSystem *system) {
  if (system == nullptr) return;

  // A batch that was prepared and never finalised has left its bodies marked
  // as belonging to a broad-phase layer they were never inserted into. Unwind
  // those before anything else touches the broad phase.
  zjolt::AbortPendingBatches(system);

  // Detach the listeners before the system tears its bodies down, so a
  // deactivation raised during destruction cannot reach a host callback that
  // is about to be freed.
  system->system.SetContactListener(nullptr);
  system->system.SetBodyActivationListener(nullptr);
  zjolt::Delete(system->contact_listener);
  zjolt::Delete(system->activation_listener);

  for (ZJoltStepListener *listener : system->step_listeners) {
    system->system.RemoveStepListener(listener);
    zjolt::Delete(listener);
  }
  system->step_listeners.clear();

  // Handing the slot back before the thunks can be called again. The system's
  // own combine pointers die with it, but the slot is process-wide and the
  // next system to ask for one must not inherit this one's callbacks.
  ReleaseCombineSlot(system);

  // The temp allocator outlives the system's own teardown by a line, because
  // the order the other way round would be a use-after-free the day
  // PhysicsSystem's destructor starts wanting scratch space.
  JPH::TempAllocatorImplWithMallocFallback *temp = system->temp_allocator;
  zjolt::Delete(system);
  zjolt::Delete(temp);
  zjolt::HandleDestroyed();
}

//===----------------------------------------------------------------------===//
// World properties
//===----------------------------------------------------------------------===//

void zjoltPhysicsSystemSetGravity(ZJoltPhysicsSystem *system,
                                  const ZJoltVec3 *gravity) {
  if (system == nullptr || gravity == nullptr) return;
  system->system.SetGravity(zjolt::ToJolt(*gravity));
}

void zjoltPhysicsSystemGetGravity(const ZJoltPhysicsSystem *system,
                                  ZJoltVec3 *out) {
  if (out == nullptr) return;
  if (system == nullptr) {
    *out = ZJoltVec3{0.0f, 0.0f, 0.0f};
    return;
  }
  *out = zjolt::ToC(system->system.GetGravity());
}

void zjoltPhysicsSystemOptimizeBroadPhase(ZJoltPhysicsSystem *system) {
  if (system == nullptr) return;
  system->system.OptimizeBroadPhase();
}

uint32_t zjoltPhysicsSystemGetNumBodies(const ZJoltPhysicsSystem *system) {
  if (system == nullptr) return 0;
  return system->system.GetNumBodies();
}

uint32_t zjoltPhysicsSystemGetNumActiveBodies(
    const ZJoltPhysicsSystem *system) {
  if (system == nullptr) return 0;
  return system->system.GetNumActiveBodies(JPH::EBodyType::RigidBody);
}

//===----------------------------------------------------------------------===//
// Listeners
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemSetContactListener(
    ZJoltPhysicsSystem *system, const ZJoltContactListener *listener) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (listener == nullptr) {
    system->system.SetContactListener(nullptr);
    zjolt::Delete(system->contact_listener);
    system->contact_listener = nullptr;
    return ZJOLT_RESULT_OK;
  }

  // Reported rather than swallowed. A listener that silently failed to install
  // is a physics world that silently stops telling you about collisions, and
  // nothing about the call site would suggest why.
  ZJoltContactListenerAdapter *adapter =
      zjolt::New<ZJoltContactListenerAdapter>(*listener);
  if (adapter == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  // Install first, then free the old one: the system must never point at a
  // freed adapter, even for an instant.
  system->system.SetContactListener(adapter);
  zjolt::Delete(system->contact_listener);
  system->contact_listener = adapter;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemSetBodyActivationListener(
    ZJoltPhysicsSystem *system, const ZJoltBodyActivationListener *listener) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (listener == nullptr) {
    system->system.SetBodyActivationListener(nullptr);
    zjolt::Delete(system->activation_listener);
    system->activation_listener = nullptr;
    return ZJOLT_RESULT_OK;
  }

  ZJoltBodyActivationListenerAdapter *adapter =
      zjolt::New<ZJoltBodyActivationListenerAdapter>(*listener);
  if (adapter == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  system->system.SetBodyActivationListener(adapter);
  zjolt::Delete(system->activation_listener);
  system->activation_listener = adapter;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// The step
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemStep(ZJoltPhysicsSystem *system,
                                   float delta_time, int32_t collision_steps,
                                   ZJoltJobSystem *job_system,
                                   uint32_t *out_error) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out_error,
                                 (uint32_t)ZJOLT_UPDATE_ERROR_NONE));
  if (!zjolt::Present(system, job_system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (collision_steps < 1) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "collision_steps must be at least 1");
  }
  // Jolt asserts on a non-positive delta; a host that hit a paused frame
  // should see nothing happen rather than an abort.
  if (!(delta_time > 0.0f)) return ZJOLT_RESULT_OK;

  const JPH::EPhysicsUpdateError error = system->system.Update(
      delta_time, collision_steps, system->temp_allocator, job_system->impl);
  system->has_stepped = true;

  if (out_error != nullptr) *out_error = static_cast<uint32_t>(error);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Bulk read-back
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemGetActiveBodies(const ZJoltPhysicsSystem *system,
                                              ZJoltBodyId *out_ids,
                                              uint32_t capacity,
                                              uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::BodyIDVector ids;
  system->system.GetActiveBodies(JPH::EBodyType::RigidBody, ids);
  return CopyBodyIds(ids, out_ids, capacity, out_count);
}

ZJoltResult zjoltPhysicsSystemGetBodies(const ZJoltPhysicsSystem *system,
                                        ZJoltBodyId *out_ids, uint32_t capacity,
                                        uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::BodyIDVector ids;
  system->system.GetBodies(ids);
  return CopyBodyIds(ids, out_ids, capacity, out_count);
}

void zjoltPhysicsSystemGetBodyStats(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyStats *out) {
  if (out == nullptr) return;
  if (system == nullptr) {
    *out = ZJoltBodyStats{};
    return;
  }
  const JPH::BodyManager::BodyStats stats = system->system.GetBodyStats();
  out->num_bodies = stats.mNumBodies;
  out->max_bodies = stats.mMaxBodies;
  out->num_bodies_static = stats.mNumBodiesStatic;
  out->num_bodies_dynamic = stats.mNumBodiesDynamic;
  out->num_active_bodies_dynamic = stats.mNumActiveBodiesDynamic;
  out->num_bodies_kinematic = stats.mNumBodiesKinematic;
  out->num_active_bodies_kinematic = stats.mNumActiveBodiesKinematic;
  out->num_soft_bodies = stats.mNumSoftBodies;
  out->num_active_soft_bodies = stats.mNumActiveSoftBodies;
}

//===----------------------------------------------------------------------===//
// World queries that are not queries
//===----------------------------------------------------------------------===//

uint32_t zjoltPhysicsSystemGetMaxBodies(const ZJoltPhysicsSystem *system) {
  if (system == nullptr) return 0;
  return system->system.GetMaxBodies();
}

bool zjoltPhysicsSystemWereBodiesInContact(const ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body1,
                                           ZJoltBodyId body2) {
  if (system == nullptr) return false;
  // Before the first step the cache's back buffer has never been finalised,
  // and Jolt asserts on a lookup into one that has not been. Nothing has been
  // in contact yet either, so the guard and the answer agree.
  if (!system->has_stepped) return false;
  return system->system.WereBodiesInContact(zjolt::ToJolt(body1),
                                            zjolt::ToJolt(body2));
}

//===----------------------------------------------------------------------===//
// Simulation settings
//===----------------------------------------------------------------------===//

void zjoltPhysicsSettingsInit(ZJoltPhysicsSettings *settings) {
  if (settings == nullptr) return;
  // Read out of a default-constructed PhysicsSettings rather than transcribed,
  // so a re-vendor that retunes one of these carries it across silently and
  // correctly instead of leaving a stale number here.
  ToCSettings(JPH::PhysicsSettings(), settings);
}

void zjoltPhysicsSystemGetSettings(const ZJoltPhysicsSystem *system,
                                   ZJoltPhysicsSettings *out) {
  if (out == nullptr) return;
  if (system == nullptr) {
    *out = ZJoltPhysicsSettings{};
    return;
  }
  ToCSettings(system->system.GetPhysicsSettings(), out);
}

ZJoltResult zjoltPhysicsSystemSetSettings(
    ZJoltPhysicsSystem *system, const ZJoltPhysicsSettings *settings) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (const char *why = WhySettingsRefused(*settings))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, why);

  JPH::PhysicsSettings jolt;
  ToJoltSettings(*settings, jolt);
  system->system.SetPhysicsSettings(jolt);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Combine callbacks
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemSetCombineFriction(ZJoltPhysicsSystem *system,
                                                 ZJoltCombineFn combine,
                                                 void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (combine == nullptr) {
    system->system.SetCombineFriction(system->default_combine_friction);
    if (system->combine_slot >= 0) {
      g_combine_slots[system->combine_slot].friction = nullptr;
      g_combine_slots[system->combine_slot].friction_user = nullptr;
    }
    return ZJOLT_RESULT_OK;
  }

  const int slot = AcquireCombineSlot(system);
  if (slot < 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_OUT_OF_MEMORY,
        "no combine-callback slot is free; Jolt's combine hook carries no "
        "user pointer, so at most ZJOLT_COMBINE_SLOT_COUNT physics systems "
        "may have one installed at a time");
  }

  g_combine_slots[slot].friction = combine;
  g_combine_slots[slot].friction_user = user;
  system->system.SetCombineFriction(kFrictionThunks.fns[slot]);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemSetCombineRestitution(ZJoltPhysicsSystem *system,
                                                    ZJoltCombineFn combine,
                                                    void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (combine == nullptr) {
    system->system.SetCombineRestitution(system->default_combine_restitution);
    if (system->combine_slot >= 0) {
      g_combine_slots[system->combine_slot].restitution = nullptr;
      g_combine_slots[system->combine_slot].restitution_user = nullptr;
    }
    return ZJOLT_RESULT_OK;
  }

  const int slot = AcquireCombineSlot(system);
  if (slot < 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_OUT_OF_MEMORY,
        "no combine-callback slot is free; Jolt's combine hook carries no "
        "user pointer, so at most ZJOLT_COMBINE_SLOT_COUNT physics systems "
        "may have one installed at a time");
  }

  g_combine_slots[slot].restitution = combine;
  g_combine_slots[slot].restitution_user = user;
  system->system.SetCombineRestitution(kRestitutionThunks.fns[slot]);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Step listeners
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemAddStepListener(ZJoltPhysicsSystem *system,
                                              ZJoltStepListenerFn listener,
                                              void *user,
                                              ZJoltStepListener **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (listener == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a step listener needs a callback");
  }

  ZJoltStepListener *adapter = zjolt::New<ZJoltStepListener>(listener, user);
  if (adapter == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  // push_back before AddStepListener, so a failure to record ownership cannot
  // leave Jolt holding a pointer this side has forgotten about. Jolt's own
  // array aborts on allocation failure; ours would too, so the order is about
  // exception-free bookkeeping rather than about recovery.
  system->step_listeners.push_back(adapter);
  system->system.AddStepListener(adapter);

  *out = adapter;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltPhysicsSystemRemoveStepListener(ZJoltPhysicsSystem *system,
                                                 ZJoltStepListener *listener) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, listener)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Jolt asserts that the listener is in its list (`PhysicsSystem.cpp:127`),
  // which a caller reaches by removing twice or by removing a handle from
  // another system. Checking our own list first turns both into an error.
  for (size_t i = 0; i < system->step_listeners.size(); ++i) {
    if (system->step_listeners[i] != listener) continue;
    system->system.RemoveStepListener(listener);
    system->step_listeners.erase(system->step_listeners.begin() +
                                 static_cast<ptrdiff_t>(i));
    zjolt::Delete(listener);
    return ZJOLT_RESULT_OK;
  }

  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "that step listener is not attached to this physics system");
}

}  // extern "C"
