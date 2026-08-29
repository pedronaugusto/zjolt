//===----------------------------------------------------------------------===//
// zjolt — the physics system: layers, listeners, the step, and bulk read-back.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"
#include "zjolt_query_internal.h"

#include <Jolt/Core/Mutex.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/NarrowPhaseStats.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>
#include <Jolt/Physics/IslandBuilder.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsStepListener.h>

/// A standalone JPH::IslandBuilder a host drives itself; see the section
/// comment in zjolt_system.h. `initialized`/`finalized` track what Jolt's own
/// asserts would otherwise require silently: Init before any other call, and
/// ResetIslands between one Finalize and the next Prepare*/Link*/Finalize.
struct ZJoltIslandBuilder {
  JPH::IslandBuilder impl;
  uint32_t max_active_bodies = 0;
  uint32_t max_contacts = 0;
  uint32_t num_constraints = 0;
  bool initialized = false;
  bool finalized = false;
  bool contact_constraints_prepared = false;
  bool non_contact_constraints_prepared = false;

  /// The temp allocator most recently given to a Prepare*/Finalize call,
  /// copied by value (@see ZJoltTempAllocator's own by-value-copy
  /// convention). Destroy() reads through it, via `user`, to reset islands
  /// the host never explicitly did — see zjolt_system.h.
  ZJoltTempAllocator last_temp_allocator{};
};

namespace {

/// Largest manifold Jolt will ever hand a listener. Asserted against Jolt's
/// own bound below, so a future upstream change is a build failure rather than
/// a silently truncated manifold.
constexpr uint32_t kMaxContactPoints = 64;
static_assert(JPH::ContactPoints::Capacity >= kMaxContactPoints,
              "Jolt's contact point capacity shrank below what zjolt copies");

/// Largest face OnContactValidate's CollideShapeResult::Face will ever carry.
/// Same reasoning as kMaxContactPoints, against Jolt's own bound.
constexpr uint32_t kMaxFaceVertices = 32;
static_assert(JPH::CollideShapeResult::Face::Capacity >= kMaxFaceVertices,
              "Jolt's collide-shape face capacity shrank below what zjolt copies");

/// What lets zjoltPhysicsSystemGetActiveBodiesUnsafe reinterpret_cast a whole
/// JPH::BodyID array as a ZJoltBodyId array rather than copying element by
/// element — the one thing a "zero-copy" accessor cannot do is copy. BodyID
/// is a single uint32 with no virtuals, so the two are the same size and
/// alignment; zjolt::ToC(const JPH::BodyID&) makes the same claim one id at a
/// time; this is that claim, but for the whole array at once.
static_assert(sizeof(JPH::BodyID) == sizeof(ZJoltBodyId) &&
                  alignof(JPH::BodyID) == alignof(ZJoltBodyId),
              "JPH::BodyID no longer matches ZJoltBodyId's layout");

/// Copies as many of `in`'s SIMD Vec3s into caller-visible 12-byte
/// vectors as fit in `out_capacity`, returning how many that was.
///
/// Unavoidable: Jolt's arrays here (ContactPoints, CollideShapeResult::Face) hold 16-byte Vec3s with a padding lane, ABI's ZJoltVec3 is three floats. On the stack, keeping a contact callback allocation-free — it runs on Jolt's job threads inside the step.
template <typename Points>
uint32_t CopyPoints(const Points &in, ZJoltVec3 *out, uint32_t out_capacity) {
  const uint32_t count = static_cast<uint32_t>(in.size()) < out_capacity
                             ? static_cast<uint32_t>(in.size())
                             : out_capacity;
  for (uint32_t i = 0; i < count; ++i) out[i] = zjolt::ToC(in[i]);
  return count;
}

uint32_t CopyContactPoints(const JPH::ContactPoints &in, ZJoltVec3 *out) {
  return CopyPoints(in, out, kMaxContactPoints);
}

void FillContactInfo(ZJoltContactInfo *info, const JPH::Body &body1,
                     const JPH::Body &body2,
                     const JPH::ContactManifold &manifold,
                     ZJoltVec3 *points1, ZJoltVec3 *points2) {
  info->body1 = zjolt::ToC(body1.GetID());
  info->body2 = zjolt::ToC(body2.GetID());
  info->user_data1 = body1.GetUserData();
  info->user_data2 = body2.GetUserData();
  info->live_body1 = zjolt::ToC(&body1);
  info->live_body2 = zjolt::ToC(&body2);

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

  ZJoltVec3 face1[kMaxFaceVertices];
  ZJoltVec3 face2[kMaxFaceVertices];

  ZJoltContactValidateInfo info;
  info.body1 = zjolt::ToC(inBody1.GetID());
  info.body2 = zjolt::ToC(inBody2.GetID());
  info.user_data1 = inBody1.GetUserData();
  info.user_data2 = inBody2.GetUserData();
  info.live_body1 = zjolt::ToC(&inBody1);
  info.live_body2 = zjolt::ToC(&inBody2);
  info.base_offset = zjolt::ToCR(inBaseOffset);
  info.contact_point_on_1 = zjolt::ToC(inCollisionResult.mContactPointOn1);
  info.contact_point_on_2 = zjolt::ToC(inCollisionResult.mContactPointOn2);
  info.penetration_axis = zjolt::ToC(inCollisionResult.mPenetrationAxis);
  info.penetration_depth = inCollisionResult.mPenetrationDepth;
  info.sub_shape_id1 = zjolt::ToC(inCollisionResult.mSubShapeID1);
  info.sub_shape_id2 = zjolt::ToC(inCollisionResult.mSubShapeID2);
  info.num_shape1_face_vertices =
      CopyPoints(inCollisionResult.mShape1Face, face1, kMaxFaceVertices);
  info.num_shape2_face_vertices =
      CopyPoints(inCollisionResult.mShape2Face, face2, kMaxFaceVertices);
  info.shape1_face = face1;
  info.shape2_face = face2;

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
// Jolt's hook is a bare function pointer with NO user parameter and nothing that leads back to the physics system, so a shared trampoline could not tell which system called it — only WHICH pointer was installed distinguishes two systems.
// So: a fixed table of slots and one trampoline per slot; a system takes a slot on first install and gives it back when destroyed (a global shared pair would not bind a per-system setting).
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
/// Callback fields are read on Jolt's job threads during a step and
/// written only by an install, which the ABI already forbids running
/// concurrently with a step on the same system. This protects the
/// process-wide table itself: two threads creating two unrelated systems could otherwise claim the same slot.
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
/// None of these is asserted upstream, and each fails somewhere else: a
/// zero batch size spins the step-listener job loop forever
/// (`fetch_add(0)` never advances), a zero batches-per-job is an
/// integer division, and a non-positive in-flight pair count is a buffer length passed to the temp allocator.
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

//===----------------------------------------------------------------------===//
// The step's scratch allocator
//
// PhysicsSystem::Update takes a plain JPH::TempAllocator&, so any of the three kinds below can back it interchangeably.
// They do NOT share introspection uniformly: TempAllocatorImpl has GetSize/GetUsage/CanAllocate natively, TempAllocatorImplWithMallocFallback keeps them private with no accessor. ZJoltTempAllocatorAdapter reimplements that class's strategy (same fixed block, malloc fallback, LIFO assumption) rather than composing it, since reimplementing is what makes the introspection possible.
//===----------------------------------------------------------------------===//

/// The default: a fixed block, falling back to malloc once it is exhausted,
/// with usage and capacity actually readable — unlike the Jolt class this
/// mirrors. @see the section comment above.
class ZJoltTempAllocatorAdapter final : public JPH::TempAllocator {
 public:
  explicit ZJoltTempAllocatorAdapter(JPH::uint fixed_size) : fixed_(fixed_size) {}

  void *Allocate(JPH::uint inSize) override {
    if (fixed_.CanAllocate(inSize)) return fixed_.Allocate(inSize);
    void *block = fallback_.Allocate(inSize);
    if (block != nullptr) fallback_usage_ += inSize;
    return block;
  }

  void Free(void *inAddress, JPH::uint inSize) override {
    if (inAddress == nullptr) return;
    if (fixed_.OwnsMemory(inAddress)) {
      fixed_.Free(inAddress, inSize);
    } else {
      fallback_.Free(inAddress, inSize);
      fallback_usage_ -= inSize;
    }
  }

  /// Whether `size` fits the fixed block without spilling into the (slower,
  /// always-available) malloc fallback — the same thing Jolt's own
  /// TempAllocatorImpl::CanAllocate answers for the no-fallback kind.
  bool CanAllocate(uint32_t size) const { return fixed_.CanAllocate(size); }
  size_t GetSize() const { return fixed_.GetSize(); }
  size_t GetUsage() const { return fixed_.GetUsage() + fallback_usage_; }

 private:
  JPH::TempAllocatorImpl fixed_;
  JPH::TempAllocatorMalloc fallback_;
  size_t fallback_usage_ = 0;
};

/// Adapts a host-supplied ZJoltTempAllocator into the JPH::TempAllocator
/// PhysicsSystem::Update actually calls.
class ZJoltHostTempAllocatorAdapter final : public JPH::TempAllocator {
 public:
  explicit ZJoltHostTempAllocatorAdapter(const ZJoltTempAllocator &host) : host_(host) {}

  void *Allocate(JPH::uint inSize) override {
    return host_.allocate(host_.user, static_cast<uint32_t>(inSize));
  }

  void Free(void *inAddress, JPH::uint inSize) override {
    host_.free(host_.user, inAddress, static_cast<uint32_t>(inSize));
  }

  bool CanAllocate(uint32_t size) const {
    return host_.can_allocate != nullptr ? host_.can_allocate(host_.user, size) : true;
  }
  size_t GetSize() const {
    return host_.get_size != nullptr ? host_.get_size(host_.user) : 0;
  }
  size_t GetUsage() const {
    return host_.get_usage != nullptr ? host_.get_usage(host_.user) : 0;
  }

 private:
  ZJoltTempAllocator host_;
};

/// The one check every ZJoltIslandBuilder entry point taking a temp
/// allocator shares, ahead of wrapping it in ZJoltHostTempAllocatorAdapter.
ZJoltResult ValidateHostTempAllocator(const ZJoltTempAllocator *temp_allocator) {
  if (temp_allocator == nullptr || temp_allocator->allocate == nullptr ||
      temp_allocator->free == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "temp_allocator needs allocate and free");
  }
  return ZJOLT_RESULT_OK;
}

void DestroyMallocFallbackTemp(JPH::TempAllocator *impl) {
  zjolt::Delete(static_cast<ZJoltTempAllocatorAdapter *>(impl));
}

void DestroyFixedTemp(JPH::TempAllocator *impl) {
  zjolt::Delete(static_cast<JPH::TempAllocatorImpl *>(impl));
}

void DestroyHostTemp(JPH::TempAllocator *impl) {
  zjolt::Delete(static_cast<ZJoltHostTempAllocatorAdapter *>(impl));
}

//===----------------------------------------------------------------------===//
// Body-vs-body narrow-phase collide hook
//
// JPH::PhysicsSystem::SetSimCollideBodyVsBody takes a std::function, not a
// virtual -- SimCollideThunk is what gets installed. Read-back goes through
// the entry table below it, not std::function::target<T>(), which this
// -fno-rtti library cannot use.
//===----------------------------------------------------------------------===//

ZJoltCollideShapeSettings ToC(const JPH::CollideShapeSettings &in) {
  ZJoltCollideShapeSettings out{};
  out.active_edge_mode = in.mActiveEdgeMode == JPH::EActiveEdgeMode::CollideWithAll
                             ? ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL
                             : ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_ONLY_WITH_ACTIVE;
  out.collect_faces_mode =
      in.mCollectFacesMode == JPH::ECollectFacesMode::CollectFaces
          ? ZJOLT_COLLECT_FACES_MODE_COLLECT_FACES
          : ZJOLT_COLLECT_FACES_MODE_NO_FACES;
  out.collision_tolerance = in.mCollisionTolerance;
  out.penetration_tolerance = in.mPenetrationTolerance;
  out.active_edge_movement_direction = zjolt::ToC(in.mActiveEdgeMovementDirection);
  out.max_separation_distance = in.mMaxSeparationDistance;
  out.back_face_mode = in.mBackFaceMode == JPH::EBackFaceMode::CollideWithBackFaces
                           ? ZJOLT_BACK_FACE_MODE_COLLIDE
                           : ZJOLT_BACK_FACE_MODE_IGNORE;
  out.internal_edge_removal_vertex_tolerance_sq =
      in.mInternalEdgeRemovalVertexToleranceSq;
  return out;
}

struct SimCollideThunk {
  ZJoltSimCollideFn collide;
  void *user;

  void operator()(const JPH::Body &body1, const JPH::Body &body2,
                  JPH::Mat44Arg transform1, JPH::Mat44Arg transform2,
                  JPH::CollideShapeSettings &settings,
                  JPH::CollideShapeCollector &collector,
                  const JPH::ShapeFilter &shape_filter) const {
    ZJoltCollideShapeSettings c_settings = ToC(settings);
    const ZJoltMat44 t1 = zjolt::ToC(transform1);
    const ZJoltMat44 t2 = zjolt::ToC(transform2);
    collide(user, zjolt::ToC(&body1), zjolt::ToC(&body2), &t1, &t2, &c_settings,
           reinterpret_cast<const ZJoltSimCollideShapeFilter *>(&shape_filter),
           reinterpret_cast<ZJoltSimCollideCollector *>(&collector));
  }
};

/// What zjoltPhysicsSystemGetSimCollideBodyVsBody reads back, keyed by system
/// pointer rather than a field on ZJoltPhysicsSystem: this library builds
/// with -fno-rtti, ruling out std::function::target<T>() to recover the
/// installed lambda. Fixed-size, not growable, so nothing here allocates
/// through Jolt's own allocator outside the create/destroy window it is
/// valid for.
constexpr size_t kMaxSimCollideEntries = 64;

struct SimCollideEntry {
  const ZJoltPhysicsSystem *system = nullptr;
  ZJoltSimCollideFn collide = nullptr;
  void *user = nullptr;
};

SimCollideEntry g_sim_collide_entries[kMaxSimCollideEntries];
JPH::Mutex g_sim_collide_mutex;

/// Records what `system` installed, reusing its existing entry if it has
/// one. False when the table is full and `system` has no entry yet.
bool SetSimCollideEntry(const ZJoltPhysicsSystem *system,
                        const ZJoltSimCollideBodyVsBody &hook) {
  JPH::lock_guard<JPH::Mutex> lock(g_sim_collide_mutex);
  SimCollideEntry *free_slot = nullptr;
  for (SimCollideEntry &entry : g_sim_collide_entries) {
    if (entry.system == system) {
      entry.collide = hook.collide;
      entry.user = hook.user;
      return true;
    }
    if (entry.system == nullptr && free_slot == nullptr) free_slot = &entry;
  }
  if (free_slot == nullptr) return false;
  *free_slot = SimCollideEntry{system, hook.collide, hook.user};
  return true;
}

void ClearSimCollideEntry(const ZJoltPhysicsSystem *system) {
  JPH::lock_guard<JPH::Mutex> lock(g_sim_collide_mutex);
  for (SimCollideEntry &entry : g_sim_collide_entries) {
    if (entry.system != system) continue;
    entry = SimCollideEntry{};
    return;
  }
}

ZJoltSimCollideBodyVsBody GetSimCollideEntry(const ZJoltPhysicsSystem *system) {
  JPH::lock_guard<JPH::Mutex> lock(g_sim_collide_mutex);
  for (const SimCollideEntry &entry : g_sim_collide_entries) {
    if (entry.system == system) return ZJoltSimCollideBodyVsBody{entry.collide, entry.user};
  }
  return ZJoltSimCollideBodyVsBody{};
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

  if (desc->temp_allocator_kind == ZJOLT_TEMP_ALLOCATOR_KIND_HOST &&
      (desc->temp_allocator == nullptr ||
       desc->temp_allocator->allocate == nullptr ||
       desc->temp_allocator->free == nullptr)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "ZJOLT_TEMP_ALLOCATOR_KIND_HOST needs "
                           "temp_allocator with allocate and free");
  }

  ZJoltPhysicsSystem *system = zjolt::New<ZJoltPhysicsSystem>(*desc);
  if (system == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  const size_t temp_size = desc->temp_allocator_size != 0
                               ? desc->temp_allocator_size
                               : 10 * 1024 * 1024;
  switch (desc->temp_allocator_kind) {
    case ZJOLT_TEMP_ALLOCATOR_KIND_HOST:
      system->temp_allocator =
          zjolt::New<ZJoltHostTempAllocatorAdapter>(*desc->temp_allocator);
      system->destroy_temp_allocator = DestroyHostTemp;
      break;
    case ZJOLT_TEMP_ALLOCATOR_KIND_FIXED:
      system->temp_allocator =
          zjolt::New<JPH::TempAllocatorImpl>(static_cast<JPH::uint>(temp_size));
      system->destroy_temp_allocator = DestroyFixedTemp;
      break;
    case ZJOLT_TEMP_ALLOCATOR_KIND_MALLOC_FALLBACK:
    default:
      system->temp_allocator =
          zjolt::New<ZJoltTempAllocatorAdapter>(static_cast<JPH::uint>(temp_size));
      system->destroy_temp_allocator = DestroyMallocFallbackTemp;
      break;
  }
  system->temp_allocator_kind = desc->temp_allocator_kind;
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

void zjoltPhysicsSystemGetBroadPhaseLayerInterface(
    const ZJoltPhysicsSystem *system, ZJoltBroadPhaseLayerInterface *out) {
  if (out == nullptr) return;
  *out = system != nullptr ? system->broad_phase_layers.Raw()
                           : ZJoltBroadPhaseLayerInterface{};
}

void zjoltPhysicsSystemGetObjectVsBroadPhaseLayerFilter(
    const ZJoltPhysicsSystem *system, ZJoltObjectVsBroadPhaseLayerFilter *out) {
  if (out == nullptr) return;
  *out = system != nullptr ? system->object_vs_broad_phase_filter.Raw()
                           : ZJoltObjectVsBroadPhaseLayerFilter{};
}

void zjoltPhysicsSystemGetObjectLayerPairFilter(
    const ZJoltPhysicsSystem *system, ZJoltObjectLayerPairFilter *out) {
  if (out == nullptr) return;
  *out = system != nullptr ? system->object_layer_pair_filter.Raw()
                           : ZJoltObjectLayerPairFilter{};
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
  system->system.SetSoftBodyContactListener(nullptr);
  system->system.SetSimShapeFilter(nullptr);
  zjolt::Delete(system->contact_listener);
  zjolt::Delete(system->activation_listener);
  zjolt::Delete(system->soft_body_contact_listener);
  zjolt::Delete(system->sim_shape_filter);

  for (ZJoltStepListener *listener : system->step_listeners) {
    system->system.RemoveStepListener(listener);
    zjolt::Delete(listener);
  }
  system->step_listeners.clear();

  // Handing the slot back before the thunks can be called again. The system's
  // own combine pointers die with it, but the slot is process-wide and the
  // next system to ask for one must not inherit this one's callbacks.
  ReleaseCombineSlot(system);

  // Same reasoning as the combine slot: the entry table is process-wide,
  // so a system's slot in it must not survive the system.
  system->system.SetSimCollideBodyVsBody(&JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody);
  ClearSimCollideEntry(system);

  // The temp allocator outlives the system's own teardown by a line —
  // the other order would be a use-after-free the day PhysicsSystem's
  // destructor starts wanting scratch space. Destroyed through the
  // thunk recorded at create, since `temp_allocator` is only ever held
  // as the JPH::TempAllocator base pointer here — zjolt::Delete needs the exact allocated type to free correctly (@see ZJoltJobSystem::destroy).
  JPH::TempAllocator *temp = system->temp_allocator;
  void (*destroy_temp)(JPH::TempAllocator *) = system->destroy_temp_allocator;
  zjolt::Delete(system);
  destroy_temp(temp);
  zjolt::HandleDestroyed();
}

//===----------------------------------------------------------------------===//
// The step's scratch allocator
//===----------------------------------------------------------------------===//

void zjoltPhysicsSystemGetTempAllocatorStats(const ZJoltPhysicsSystem *system,
                                             ZJoltTempAllocatorStats *out) {
  if (out == nullptr) return;
  if (system == nullptr) {
    *out = ZJoltTempAllocatorStats{};
    return;
  }
  switch (system->temp_allocator_kind) {
    case ZJOLT_TEMP_ALLOCATOR_KIND_FIXED: {
      const auto *impl = static_cast<const JPH::TempAllocatorImpl *>(system->temp_allocator);
      out->capacity = impl->GetSize();
      out->usage = impl->GetUsage();
      return;
    }
    case ZJOLT_TEMP_ALLOCATOR_KIND_HOST: {
      const auto *impl = static_cast<const ZJoltHostTempAllocatorAdapter *>(system->temp_allocator);
      out->capacity = impl->GetSize();
      out->usage = impl->GetUsage();
      return;
    }
    case ZJOLT_TEMP_ALLOCATOR_KIND_MALLOC_FALLBACK:
    default: {
      const auto *impl = static_cast<const ZJoltTempAllocatorAdapter *>(system->temp_allocator);
      out->capacity = impl->GetSize();
      out->usage = impl->GetUsage();
      return;
    }
  }
}

bool zjoltPhysicsSystemTempAllocatorCanAllocate(const ZJoltPhysicsSystem *system,
                                                uint32_t size) {
  if (system == nullptr) return false;
  switch (system->temp_allocator_kind) {
    case ZJOLT_TEMP_ALLOCATOR_KIND_FIXED:
      return static_cast<const JPH::TempAllocatorImpl *>(system->temp_allocator)
          ->CanAllocate(size);
    case ZJOLT_TEMP_ALLOCATOR_KIND_HOST:
      return static_cast<const ZJoltHostTempAllocatorAdapter *>(system->temp_allocator)
          ->CanAllocate(size);
    case ZJOLT_TEMP_ALLOCATOR_KIND_MALLOC_FALLBACK:
    default:
      return static_cast<const ZJoltTempAllocatorAdapter *>(system->temp_allocator)
          ->CanAllocate(size);
  }
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

void zjoltPhysicsSystemGetContactListener(const ZJoltPhysicsSystem *system,
                                          ZJoltContactListener *out) {
  if (out == nullptr) return;
  *out = system != nullptr && system->contact_listener != nullptr
             ? system->contact_listener->Raw()
             : ZJoltContactListener{};
}

void zjoltPhysicsSystemGetBodyActivationListener(
    const ZJoltPhysicsSystem *system, ZJoltBodyActivationListener *out) {
  if (out == nullptr) return;
  *out = system != nullptr && system->activation_listener != nullptr
             ? system->activation_listener->Raw()
             : ZJoltBodyActivationListener{};
}

void zjoltPhysicsSystemGetSoftBodyContactListener(
    const ZJoltPhysicsSystem *system, ZJoltSoftBodyContactListener *out) {
  if (out == nullptr) return;
  *out = system != nullptr && system->soft_body_contact_listener != nullptr
             ? system->soft_body_contact_listener->Raw()
             : ZJoltSoftBodyContactListener{};
}

//===----------------------------------------------------------------------===//
// Simulation shape filter
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemSetSimShapeFilter(
    ZJoltPhysicsSystem *system, const ZJoltSimShapeFilter *filter) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (filter == nullptr) {
    system->system.SetSimShapeFilter(nullptr);
    zjolt::Delete(system->sim_shape_filter);
    system->sim_shape_filter = nullptr;
    return ZJOLT_RESULT_OK;
  }

  ZJoltSimShapeFilterAdapter *adapter =
      zjolt::New<ZJoltSimShapeFilterAdapter>(*filter);
  if (adapter == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  // Install first, then free the old one: the system must never point at a
  // freed adapter, even for an instant — the same rule zjoltPhysicsSystem
  // SetContactListener follows.
  system->system.SetSimShapeFilter(adapter);
  zjolt::Delete(system->sim_shape_filter);
  system->sim_shape_filter = adapter;
  return ZJOLT_RESULT_OK;
}

void zjoltPhysicsSystemGetSimShapeFilter(const ZJoltPhysicsSystem *system,
                                         ZJoltSimShapeFilter *out) {
  if (out == nullptr) return;
  *out = system != nullptr && system->sim_shape_filter != nullptr
             ? system->sim_shape_filter->Raw()
             : ZJoltSimShapeFilter{};
}

//===----------------------------------------------------------------------===//
// Body-vs-body narrow-phase collide hook
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemSetSimCollideBodyVsBody(
    ZJoltPhysicsSystem *system, const ZJoltSimCollideBodyVsBody *hook) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (hook == nullptr || hook->collide == nullptr) {
    ClearSimCollideEntry(system);
    system->system.SetSimCollideBodyVsBody(
        &JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody);
    return ZJOLT_RESULT_OK;
  }

  // Recorded before installing: a table that is full must leave the
  // previous hook (default or otherwise) running, not a hook Get can never
  // report back correctly.
  if (!SetSimCollideEntry(system, *hook)) {
    return zjolt::SetError(
        ZJOLT_RESULT_OUT_OF_MEMORY,
        "no room to track a sim-collide hook for another physics system; "
        "at most 64 systems may have one installed at a time");
  }
  system->system.SetSimCollideBodyVsBody(SimCollideThunk{hook->collide, hook->user});
  return ZJOLT_RESULT_OK;
}

void zjoltPhysicsSystemGetSimCollideBodyVsBody(const ZJoltPhysicsSystem *system,
                                               ZJoltSimCollideBodyVsBody *out) {
  if (out == nullptr) return;
  *out = system != nullptr ? GetSimCollideEntry(system) : ZJoltSimCollideBodyVsBody{};
}

void zjoltSimCollideAddHit(ZJoltSimCollideCollector *collector, ZJoltBodyId body2,
                           const ZJoltSimCollideHit *hit) {
  if (collector == nullptr || hit == nullptr) return;

  auto *jolt_collector = reinterpret_cast<JPH::CollideShapeCollector *>(collector);
  // A pair a contact-validate listener already rejected outright has forced
  // this collector's early-out; AddHit asserts that never happens twice.
  if (jolt_collector->ShouldEarlyOut()) return;

  JPH::CollideShapeResult result;
  result.mContactPointOn1 = zjolt::ToJolt(hit->contact_point_on_1);
  result.mContactPointOn2 = zjolt::ToJolt(hit->contact_point_on_2);
  result.mPenetrationAxis = zjolt::ToJolt(hit->penetration_axis);
  result.mPenetrationDepth = hit->penetration_depth;
  result.mSubShapeID1 = zjolt::ToJoltSubShapeId(hit->sub_shape_id1);
  result.mSubShapeID2 = zjolt::ToJoltSubShapeId(hit->sub_shape_id2);
  result.mBodyID2 = zjolt::ToJolt(body2);
  jolt_collector->AddHit(result);
}

void zjoltSimCollideDefault(const ZJoltBody *live_body1, const ZJoltBody *live_body2,
                            const ZJoltMat44 *center_of_mass_transform1,
                            const ZJoltMat44 *center_of_mass_transform2,
                            const ZJoltCollideShapeSettings *settings,
                            const ZJoltSimCollideShapeFilter *shape_filter,
                            ZJoltSimCollideCollector *collector) {
  if (!zjolt::Present(live_body1, live_body2, center_of_mass_transform1,
                      center_of_mass_transform2, collector)) {
    return;
  }

  auto *jolt_collector = reinterpret_cast<JPH::CollideShapeCollector *>(collector);
  if (jolt_collector->ShouldEarlyOut()) return;

  JPH::CollideShapeSettings jolt_settings = zjolt::MakeCollideShapeSettings(settings);
  // Per-call, not shared: JPH::ShapeFilter::mBodyID2 is mutable and Jolt's
  // own dispatch writes through it, so a filter two worker threads could
  // reach at once would race.
  JPH::ShapeFilter default_filter;
  const JPH::ShapeFilter &filter =
      shape_filter != nullptr
          ? *reinterpret_cast<const JPH::ShapeFilter *>(shape_filter)
          : default_filter;

  JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody(
      *zjolt::ToJolt(live_body1), *zjolt::ToJolt(live_body2),
      zjolt::ToJolt(*center_of_mass_transform1),
      zjolt::ToJolt(*center_of_mass_transform2), jolt_settings, *jolt_collector,
      filter);
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

ZJoltResult zjoltPhysicsSystemReportBroadphaseStats(ZJoltPhysicsSystem *system) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_TRACK_BROADPHASE_STATS
  system->system.ReportBroadphaseStats();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltReportNarrowPhaseStats(void) {
  ZJOLT_ENTER();
#ifdef JPH_TRACK_NARROWPHASE_STATS
  JPH::NarrowPhaseStat::sReportStats();
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
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

const ZJoltBody *zjoltPhysicsSystemTryGetBodyNoLock(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  if (system == nullptr) return nullptr;
  return zjolt::ToC(
      system->system.GetBodyLockInterfaceNoLock().TryGetBody(zjolt::ToJolt(body)));
}

void zjoltPhysicsSystemGetActiveBodiesUnsafe(const ZJoltPhysicsSystem *system,
                                             const ZJoltBodyId **out_ids,
                                             uint32_t *out_count) {
  if (out_ids != nullptr) *out_ids = nullptr;
  if (out_count != nullptr) *out_count = 0;
  if (system == nullptr) return;

  const JPH::BodyID *ids =
      system->system.GetActiveBodiesUnsafe(JPH::EBodyType::RigidBody);
  const uint32_t count =
      system->system.GetNumActiveBodies(JPH::EBodyType::RigidBody);
  if (out_ids != nullptr) *out_ids = reinterpret_cast<const ZJoltBodyId *>(ids);
  if (out_count != nullptr) *out_count = count;
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

void zjoltPhysicsSystemGetCombineRestitution(const ZJoltPhysicsSystem *system,
                                             ZJoltCombineFn *out_combine,
                                             void **out_user) {
  if (out_combine != nullptr) *out_combine = nullptr;
  if (out_user != nullptr) *out_user = nullptr;
  if (system == nullptr || system->combine_slot < 0) return;

  const CombineSlot &slot = g_combine_slots[system->combine_slot];
  if (out_combine != nullptr) *out_combine = slot.restitution;
  if (out_user != nullptr) *out_user = slot.restitution_user;
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

  // Jolt asserts that the listener is in its list, which a caller
  // reaches by removing twice or by removing a handle from another
  // system. Checking the tracked list first turns both into an error.
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

//===----------------------------------------------------------------------===//
// Islands
//===----------------------------------------------------------------------===//

ZJoltResult zjoltIslandBuilderCreate(ZJoltIslandBuilder **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  ZJoltIslandBuilder *builder = zjolt::New<ZJoltIslandBuilder>();
  if (builder == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = builder;
  return ZJOLT_RESULT_OK;
}

void zjoltIslandBuilderDestroy(ZJoltIslandBuilder *builder) {
  if (builder == nullptr) return;

  // JPH::IslandBuilder's own destructor asserts every island-data pointer is
  // null; a host that never called ResetIslands after its last Prepare*/
  // Finalize would otherwise abort here instead of merely leaking.
  if (builder->finalized || builder->contact_constraints_prepared ||
      builder->non_contact_constraints_prepared) {
    ZJoltHostTempAllocatorAdapter adapter(builder->last_temp_allocator);
    builder->impl.ResetIslands(&adapter);
  }
  zjolt::Delete(builder);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltIslandBuilderInit(ZJoltIslandBuilder *builder,
                                   uint32_t max_active_bodies) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (builder->initialized) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this island builder has already been initialized");
  }

  builder->impl.Init(max_active_bodies);
  builder->max_active_bodies = max_active_bodies;
  builder->initialized = true;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderLinkBodies(ZJoltIslandBuilder *builder,
                                         uint32_t first, uint32_t second) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!builder->initialized) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "zjoltIslandBuilderInit must run first");
  }
  if (builder->finalized) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "call zjoltIslandBuilderResetIslands before linking again");
  }

  builder->impl.LinkBodies(first, second);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderPrepareContactConstraints(
    ZJoltIslandBuilder *builder, uint32_t max_contacts,
    const ZJoltTempAllocator *temp_allocator) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!builder->initialized) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "zjoltIslandBuilderInit must run first");
  }
  if (builder->finalized || builder->contact_constraints_prepared) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "call zjoltIslandBuilderResetIslands before preparing again");
  }
  const ZJoltResult checked = ValidateHostTempAllocator(temp_allocator);
  if (checked != ZJOLT_RESULT_OK) return checked;

  builder->last_temp_allocator = *temp_allocator;
  ZJoltHostTempAllocatorAdapter adapter(*temp_allocator);
  builder->impl.PrepareContactConstraints(max_contacts, &adapter);
  builder->max_contacts = max_contacts;
  builder->contact_constraints_prepared = true;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderPrepareNonContactConstraints(
    ZJoltIslandBuilder *builder, uint32_t num_constraints,
    const ZJoltTempAllocator *temp_allocator) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!builder->initialized) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "zjoltIslandBuilderInit must run first");
  }
  if (builder->finalized || builder->non_contact_constraints_prepared) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "call zjoltIslandBuilderResetIslands before preparing again");
  }
  const ZJoltResult checked = ValidateHostTempAllocator(temp_allocator);
  if (checked != ZJOLT_RESULT_OK) return checked;

  builder->last_temp_allocator = *temp_allocator;
  ZJoltHostTempAllocatorAdapter adapter(*temp_allocator);
  builder->impl.PrepareNonContactConstraints(num_constraints, &adapter);
  builder->num_constraints = num_constraints;
  builder->non_contact_constraints_prepared = true;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderLinkConstraint(ZJoltIslandBuilder *builder,
                                             uint32_t constraint_index,
                                             uint32_t index_in_active_body_list) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!builder->non_contact_constraints_prepared) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "zjoltIslandBuilderPrepareNonContactConstraints must run first");
  }
  if (constraint_index >= builder->num_constraints) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint_index is out of range");
  }
  if (index_in_active_body_list >= builder->max_active_bodies) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "index_in_active_body_list is out of range");
  }

  builder->impl.LinkConstraint(constraint_index, index_in_active_body_list);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderLinkContact(ZJoltIslandBuilder *builder,
                                          uint32_t contact_index,
                                          uint32_t index_in_active_body_list) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!builder->contact_constraints_prepared) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "zjoltIslandBuilderPrepareContactConstraints must run first");
  }
  if (contact_index >= builder->max_contacts) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "contact_index is out of range");
  }
  if (index_in_active_body_list >= builder->max_active_bodies) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "index_in_active_body_list is out of range");
  }

  builder->impl.LinkContact(contact_index, index_in_active_body_list);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderFinalize(ZJoltIslandBuilder *builder,
                                       const ZJoltBodyId *active_bodies,
                                       uint32_t num_active_bodies,
                                       uint32_t num_contacts,
                                       const ZJoltTempAllocator *temp_allocator) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!builder->initialized) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "zjoltIslandBuilderInit must run first");
  }
  if (builder->finalized) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "call zjoltIslandBuilderResetIslands before finalizing again");
  }
  if (num_active_bodies > builder->max_active_bodies) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "num_active_bodies exceeds max_active_bodies");
  }
  if (num_active_bodies > 0 && active_bodies == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "active_bodies is required when num_active_bodies is nonzero");
  }
  if (builder->contact_constraints_prepared) {
    if (num_contacts > builder->max_contacts) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "num_contacts exceeds the count given to PrepareContactConstraints");
    }
  } else if (num_contacts != 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "num_contacts must be 0 when PrepareContactConstraints was not called");
  }
  const ZJoltResult checked = ValidateHostTempAllocator(temp_allocator);
  if (checked != ZJOLT_RESULT_OK) return checked;

  builder->last_temp_allocator = *temp_allocator;
  // JPH::BodyID and ZJoltBodyId share layout (static_assert above, in this
  // same file), so the caller's array is read in place rather than copied.
  ZJoltHostTempAllocatorAdapter adapter(*temp_allocator);
  builder->impl.Finalize(reinterpret_cast<const JPH::BodyID *>(active_bodies),
                         num_active_bodies, num_contacts, &adapter);
  builder->finalized = true;
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltIslandBuilderGetNumIslands(const ZJoltIslandBuilder *builder) {
  if (builder == nullptr) return 0;
  return builder->impl.GetNumIslands();
}

ZJoltResult zjoltIslandBuilderGetBodiesInIsland(const ZJoltIslandBuilder *builder,
                                                uint32_t island_index,
                                                ZJoltBodyId *out_bodies,
                                                uint32_t capacity,
                                                uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(builder, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (island_index >= builder->impl.GetNumIslands()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "island_index is at or past GetNumIslands");
  }

  JPH::BodyID *begin = nullptr;
  JPH::BodyID *end = nullptr;
  builder->impl.GetBodiesInIsland(island_index, begin, end);
  const uint32_t count = static_cast<uint32_t>(end - begin);
  *out_count = count;
  if (out_bodies == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  for (uint32_t i = 0; i < count; ++i) out_bodies[i] = zjolt::ToC(begin[i]);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderGetConstraintsInIsland(
    const ZJoltIslandBuilder *builder, uint32_t island_index,
    uint32_t *out_constraints, uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(builder, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (island_index >= builder->impl.GetNumIslands()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "island_index is at or past GetNumIslands");
  }

  uint32_t *begin = nullptr;
  uint32_t *end = nullptr;
  builder->impl.GetConstraintsInIsland(island_index, begin, end);
  const uint32_t count =
      begin != nullptr ? static_cast<uint32_t>(end - begin) : 0;
  *out_count = count;
  if (out_constraints == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  if (count > 0) std::memcpy(out_constraints, begin, count * sizeof(uint32_t));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderGetContactsInIsland(
    const ZJoltIslandBuilder *builder, uint32_t island_index,
    uint32_t *out_contacts, uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(builder, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (island_index >= builder->impl.GetNumIslands()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "island_index is at or past GetNumIslands");
  }

  uint32_t *begin = nullptr;
  uint32_t *end = nullptr;
  builder->impl.GetContactsInIsland(island_index, begin, end);
  const uint32_t count =
      begin != nullptr ? static_cast<uint32_t>(end - begin) : 0;
  *out_count = count;
  if (out_contacts == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  if (count > 0) std::memcpy(out_contacts, begin, count * sizeof(uint32_t));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderGetNumPositionSteps(
    const ZJoltIslandBuilder *builder, uint32_t island_index,
    uint32_t *out_num_position_steps) {
  ZJOLT_ENTER(out_num_position_steps);
  if (!zjolt::Present(builder, out_num_position_steps))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (island_index >= builder->impl.GetNumIslands()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "island_index is at or past GetNumIslands");
  }

  *out_num_position_steps = builder->impl.GetNumPositionSteps(island_index);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderGetStats(const ZJoltIslandBuilder *builder,
                                       uint32_t island_index,
                                       ZJoltIslandStats *out_stats) {
  ZJOLT_ENTER(out_stats);
  if (!zjolt::Present(builder, out_stats)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (island_index >= builder->impl.GetNumIslands()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "island_index is at or past GetNumIslands");
  }
#ifdef JPH_TRACK_SIMULATION_STATS
  const JPH::IslandBuilder::IslandStats &stats =
      const_cast<ZJoltIslandBuilder *>(builder)->impl.GetIslandStats(island_index);
  out_stats->velocity_constraint_ticks = stats.mVelocityConstraintTicks.load();
  out_stats->position_constraint_ticks = stats.mPositionConstraintTicks.load();
  out_stats->update_bounds_ticks = stats.mUpdateBoundsTicks.load();
  out_stats->num_velocity_steps = stats.mNumVelocitySteps;
  out_stats->num_position_steps = stats.mNumPositionSteps;
  out_stats->is_large_island = stats.mIsLargeIsland;
  return ZJOLT_RESULT_OK;
#else
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

ZJoltResult zjoltIslandBuilderSetNumPositionSteps(ZJoltIslandBuilder *builder,
                                                  uint32_t island_index,
                                                  uint32_t num_position_steps) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (island_index >= builder->impl.GetNumIslands()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "island_index is at or past GetNumIslands");
  }
  if (num_position_steps >= 256) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "num_position_steps must be below 256");
  }

  builder->impl.SetNumPositionSteps(island_index, num_position_steps);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltIslandBuilderResetIslands(
    ZJoltIslandBuilder *builder, const ZJoltTempAllocator *temp_allocator) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!builder->finalized) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "zjoltIslandBuilderFinalize must run first");
  }
  const ZJoltResult checked = ValidateHostTempAllocator(temp_allocator);
  if (checked != ZJOLT_RESULT_OK) return checked;

  ZJoltHostTempAllocatorAdapter adapter(*temp_allocator);
  builder->impl.ResetIslands(&adapter);
  builder->finalized = false;
  builder->contact_constraints_prepared = false;
  builder->non_contact_constraints_prepared = false;
  builder->max_contacts = 0;
  builder->num_constraints = 0;
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
