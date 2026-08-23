//===----------------------------------------------------------------------===//
// zjolt — the physics system: layers, listeners, the step, and bulk read-back.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>

namespace {

/// Largest manifold Jolt will ever hand a listener. Asserted against Jolt's
/// own bound below, so a future upstream change is a build failure rather than
/// a silently truncated manifold.
constexpr uint32_t kMaxContactPoints = 64;
static_assert(JPH::ContactPoints::Capacity >= kMaxContactPoints,
              "Jolt's contact point capacity shrank below what zjolt copies");

uint32_t ToCSubShapeId(const JPH::SubShapeID &id) {
  return static_cast<uint32_t>(id.GetValue());
}

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
  out.sub_shape_id1 = ToCSubShapeId(manifold.mSubShapeID1);
  out.sub_shape_id2 = ToCSubShapeId(manifold.mSubShapeID2);
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
  if (out_ids == nullptr) return ZJOLT_OK;
  if (capacity < count) return ZJOLT_ERR_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) out_ids[i] = zjolt::ToC(ids[i]);
  return ZJOLT_OK;
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
  info.sub_shape_id1 = ToCSubShapeId(inCollisionResult.mSubShapeID1);
  info.sub_shape_id2 = ToCSubShapeId(inCollisionResult.mSubShapeID2);

  switch (listener_.on_contact_validate(listener_.user, &info)) {
    case ZJOLT_VALIDATE_ACCEPT_CONTACT:
      return JPH::ValidateResult::AcceptContact;
    case ZJOLT_VALIDATE_REJECT_CONTACT:
      return JPH::ValidateResult::RejectContact;
    case ZJOLT_VALIDATE_REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR:
      return JPH::ValidateResult::RejectAllContactsForThisBodyPair;
    case ZJOLT_VALIDATE_ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR:
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
      ToCSubShapeId(inSubShapePair.GetSubShapeID1()),
      zjolt::ToC(inSubShapePair.GetBody2ID()),
      ToCSubShapeId(inSubShapePair.GetSubShapeID2()),
  };
  listener_.on_contact_removed(listener_.user, &pair);
}

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
  zjolt::ClearError();
  if (out == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;
  *out = nullptr;
  if (!zjolt::IsInitialized()) return ZJOLT_ERR_NOT_INITIALIZED;
  if (desc == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;

  if (desc->max_bodies == 0) {
    return zjolt::SetError(ZJOLT_ERR_INVALID_ARGUMENT,
                           "max_bodies must be positive");
  }
  // Jolt packs a body's index and a generation counter into one 32-bit id, so
  // there is a hard ceiling on how many bodies can exist. Past it, Jolt
  // asserts while handing out an id — long after the mistake, and fatally.
  // Reproduced with max_bodies = 100,000,000.
  if (desc->max_bodies > JPH::BodyID::cMaxBodyIndex + 1u) {
    return zjolt::SetError(
        ZJOLT_ERR_INVALID_ARGUMENT,
        "max_bodies exceeds Jolt's hard limit of 8388608 bodies");
  }
  if (desc->max_contact_constraints >
      JPH::ContactConstraintManager::cMaxContactConstraintsLimit) {
    return zjolt::SetError(
        ZJOLT_ERR_INVALID_ARGUMENT,
        "max_contact_constraints exceeds what Jolt can address");
  }
  // The two required questions. Jolt would dereference a null vtable entry
  // during the first broad-phase build; failing here names the missing piece.
  if (desc->broad_phase_layers.num_broad_phase_layers == nullptr ||
      desc->broad_phase_layers.broad_phase_layer_for_object_layer == nullptr) {
    return zjolt::SetError(
        ZJOLT_ERR_INVALID_ARGUMENT,
        "broad_phase_layers needs num_broad_phase_layers and "
        "broad_phase_layer_for_object_layer");
  }

  ZJoltPhysicsSystem *system = zjolt::New<ZJoltPhysicsSystem>(*desc);
  if (system == nullptr) return ZJOLT_ERR_OUT_OF_MEMORY;

  const size_t temp_size = desc->temp_allocator_size != 0
                               ? desc->temp_allocator_size
                               : 10 * 1024 * 1024;
  system->temp_allocator =
      zjolt::New<JPH::TempAllocatorImplWithMallocFallback>(
          static_cast<JPH::uint>(temp_size));
  if (system->temp_allocator == nullptr) {
    zjolt::Delete(system);
    return ZJOLT_ERR_OUT_OF_MEMORY;
  }

  system->system.Init(desc->max_bodies, desc->num_body_mutexes,
                      desc->max_body_pairs, desc->max_contact_constraints,
                      system->broad_phase_layers,
                      system->object_vs_broad_phase_filter,
                      system->object_layer_pair_filter);

  zjolt::HandleCreated();
  *out = system;
  return ZJOLT_OK;
}

void zjoltPhysicsSystemDestroy(ZJoltPhysicsSystem *system) {
  if (system == nullptr) return;

  // Detach the listeners before the system tears its bodies down, so a
  // deactivation raised during destruction cannot reach a host callback that
  // is about to be freed.
  system->system.SetContactListener(nullptr);
  system->system.SetBodyActivationListener(nullptr);
  zjolt::Delete(system->contact_listener);
  zjolt::Delete(system->activation_listener);

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
  zjolt::ClearError();
  if (system == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;

  if (listener == nullptr) {
    system->system.SetContactListener(nullptr);
    zjolt::Delete(system->contact_listener);
    system->contact_listener = nullptr;
    return ZJOLT_OK;
  }

  // Reported rather than swallowed. A listener that silently failed to install
  // is a physics world that silently stops telling you about collisions, and
  // nothing about the call site would suggest why.
  ZJoltContactListenerAdapter *adapter =
      zjolt::New<ZJoltContactListenerAdapter>(*listener);
  if (adapter == nullptr) return ZJOLT_ERR_OUT_OF_MEMORY;

  // Install first, then free the old one: the system must never point at a
  // freed adapter, even for an instant.
  system->system.SetContactListener(adapter);
  zjolt::Delete(system->contact_listener);
  system->contact_listener = adapter;
  return ZJOLT_OK;
}

ZJoltResult zjoltPhysicsSystemSetBodyActivationListener(
    ZJoltPhysicsSystem *system, const ZJoltBodyActivationListener *listener) {
  zjolt::ClearError();
  if (system == nullptr) return ZJOLT_ERR_INVALID_ARGUMENT;

  if (listener == nullptr) {
    system->system.SetBodyActivationListener(nullptr);
    zjolt::Delete(system->activation_listener);
    system->activation_listener = nullptr;
    return ZJOLT_OK;
  }

  ZJoltBodyActivationListenerAdapter *adapter =
      zjolt::New<ZJoltBodyActivationListenerAdapter>(*listener);
  if (adapter == nullptr) return ZJOLT_ERR_OUT_OF_MEMORY;

  system->system.SetBodyActivationListener(adapter);
  zjolt::Delete(system->activation_listener);
  system->activation_listener = adapter;
  return ZJOLT_OK;
}

//===----------------------------------------------------------------------===//
// The step
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemStep(ZJoltPhysicsSystem *system,
                                   float delta_time, int32_t collision_steps,
                                   ZJoltJobSystem *job_system,
                                   uint32_t *out_error) {
  zjolt::ClearError();
  if (out_error != nullptr) *out_error = ZJOLT_UPDATE_ERROR_NONE;
  if (system == nullptr || job_system == nullptr)
    return ZJOLT_ERR_INVALID_ARGUMENT;
  if (collision_steps < 1) {
    return zjolt::SetError(ZJOLT_ERR_INVALID_ARGUMENT,
                           "collision_steps must be at least 1");
  }
  // Jolt asserts on a non-positive delta; a host that hit a paused frame
  // should see nothing happen rather than an abort.
  if (!(delta_time > 0.0f)) return ZJOLT_OK;

  const JPH::EPhysicsUpdateError error = system->system.Update(
      delta_time, collision_steps, system->temp_allocator, job_system->impl);

  if (out_error != nullptr) *out_error = static_cast<uint32_t>(error);
  return ZJOLT_OK;
}

//===----------------------------------------------------------------------===//
// Bulk read-back
//===----------------------------------------------------------------------===//

ZJoltResult zjoltPhysicsSystemGetActiveBodies(const ZJoltPhysicsSystem *system,
                                              ZJoltBodyId *out_ids,
                                              uint32_t capacity,
                                              uint32_t *out_count) {
  zjolt::ClearError();
  if (system == nullptr || out_count == nullptr)
    return ZJOLT_ERR_INVALID_ARGUMENT;

  JPH::BodyIDVector ids;
  system->system.GetActiveBodies(JPH::EBodyType::RigidBody, ids);
  return CopyBodyIds(ids, out_ids, capacity, out_count);
}

ZJoltResult zjoltPhysicsSystemGetBodies(const ZJoltPhysicsSystem *system,
                                        ZJoltBodyId *out_ids, uint32_t capacity,
                                        uint32_t *out_count) {
  zjolt::ClearError();
  if (system == nullptr || out_count == nullptr)
    return ZJOLT_ERR_INVALID_ARGUMENT;

  JPH::BodyIDVector ids;
  system->system.GetBodies(ids);
  return CopyBodyIds(ids, out_ids, capacity, out_count);
}

}  // extern "C"
