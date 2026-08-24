//===----------------------------------------------------------------------===//
// zjolt — bodies: creation, the per-body accessors, locks, and bulk read-back.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Math/Math.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>

namespace {

JPH::EActivation ToJoltActivation(ZJoltActivation activation) {
  return activation == ZJOLT_ACTIVATION_DONT_ACTIVATE
             ? JPH::EActivation::DontActivate
             : JPH::EActivation::Activate;
}

JPH::EMotionType ToJoltMotionType(ZJoltMotionType type) {
  switch (type) {
    case ZJOLT_MOTION_TYPE_STATIC:
      return JPH::EMotionType::Static;
    case ZJOLT_MOTION_TYPE_KINEMATIC:
      return JPH::EMotionType::Kinematic;
    case ZJOLT_MOTION_TYPE_DYNAMIC:
      break;
  }
  return JPH::EMotionType::Dynamic;
}

ZJoltMotionType ToCMotionType(JPH::EMotionType type) {
  switch (type) {
    case JPH::EMotionType::Static:
      return ZJOLT_MOTION_TYPE_STATIC;
    case JPH::EMotionType::Kinematic:
      return ZJOLT_MOTION_TYPE_KINEMATIC;
    case JPH::EMotionType::Dynamic:
      break;
  }
  return ZJOLT_MOTION_TYPE_DYNAMIC;
}

JPH::EMotionQuality ToJoltMotionQuality(ZJoltMotionQuality quality) {
  return quality == ZJOLT_MOTION_QUALITY_LINEAR_CAST
             ? JPH::EMotionQuality::LinearCast
             : JPH::EMotionQuality::Discrete;
}

ZJoltMotionQuality ToCMotionQuality(JPH::EMotionQuality quality) {
  return quality == JPH::EMotionQuality::LinearCast
             ? ZJOLT_MOTION_QUALITY_LINEAR_CAST
             : ZJOLT_MOTION_QUALITY_DISCRETE;
}

ZJoltBodyType ToCBodyType(JPH::EBodyType type) {
  return type == JPH::EBodyType::SoftBody ? ZJOLT_BODY_TYPE_SOFT_BODY
                                          : ZJOLT_BODY_TYPE_RIGID_BODY;
}

JPH::BodyInterface *Interface(ZJoltPhysicsSystem *system) {
  if (system == nullptr) return nullptr;
  return &system->system.GetBodyInterface();
}

const JPH::BodyInterface *Interface(const ZJoltPhysicsSystem *system) {
  if (system == nullptr) return nullptr;
  return &system->system.GetBodyInterface();
}

/// How many body ids one bulk read locks at a time.
///
/// BodyLockMultiRead computes a single mutex mask for the whole batch, so the
/// larger the chunk the fewer lock round trips. It is bounded because the ids
/// are copied into a stack array first: ZJoltBodyId and JPH::BodyID are the
/// same width, but reading one array as the other would be type punning, and
/// a memcpy through a fixed buffer costs less than the class of bug that
/// invites.
constexpr uint32_t kBulkChunk = 256;

/// Walks `ids` in chunks, taking one multi-body read lock per chunk, and
/// invokes `visit(index, body_or_null)`.
template <typename Visitor>
ZJoltResult VisitBodies(const ZJoltPhysicsSystem *system,
                        const ZJoltBodyId *ids, uint32_t count,
                        uint32_t *out_missing, Visitor visit) {
  ZJOLT_ENTER(out_missing);
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count == 0) return ZJOLT_RESULT_OK;
  if (!zjolt::Present(ids)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::BodyLockInterface &lock_interface =
      system->system.GetBodyLockInterface();

  uint32_t missing = 0;
  for (uint32_t base = 0; base < count; base += kBulkChunk) {
    const uint32_t chunk =
        (count - base) < kBulkChunk ? (count - base) : kBulkChunk;

    JPH::BodyID chunk_ids[kBulkChunk];
    for (uint32_t i = 0; i < chunk; ++i)
      chunk_ids[i] = zjolt::ToJolt(ids[base + i]);

    JPH::BodyLockMultiRead lock(lock_interface, chunk_ids,
                                static_cast<int>(chunk));
    for (uint32_t i = 0; i < chunk; ++i) {
      const JPH::Body *body = lock.GetBody(static_cast<int>(i));
      if (body == nullptr) ++missing;
      visit(base + i, body);
    }
  }

  if (out_missing != nullptr) *out_missing = missing;
  return ZJOLT_RESULT_OK;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Creation
//===----------------------------------------------------------------------===//

void zjoltBodyDescInit(ZJoltBodyDesc *desc) {
  if (desc == nullptr) return;

  // Read out of a default-constructed BodyCreationSettings rather than typed
  // in, so a Jolt upgrade that changes a default moves this too.
  const JPH::BodyCreationSettings defaults;

  *desc = ZJoltBodyDesc{};
  desc->position = zjolt::ToCR(defaults.mPosition);
  desc->rotation = zjolt::ToC(defaults.mRotation);
  desc->linear_velocity = zjolt::ToC(defaults.mLinearVelocity);
  desc->angular_velocity = zjolt::ToC(defaults.mAngularVelocity);
  desc->shape = nullptr;
  // Not all-zero: "no group" is ZJOLT_COLLISION_GROUP_INVALID, and 0 is a
  // perfectly good group id.
  desc->collision_group.filter = nullptr;
  desc->collision_group.group_id = defaults.mCollisionGroup.GetGroupID();
  desc->collision_group.sub_group_id = defaults.mCollisionGroup.GetSubGroupID();
  desc->user_data = defaults.mUserData;
  desc->object_layer = static_cast<ZJoltObjectLayer>(defaults.mObjectLayer);
  desc->motion_type = ToCMotionType(defaults.mMotionType);
  desc->motion_quality = ZJOLT_MOTION_QUALITY_DISCRETE;
  desc->allowed_dofs = ZJOLT_ALLOWED_DOFS_ALL;
  desc->override_mass_properties = ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_MASS_AND_INERTIA;
  desc->mass = 0.0f;
  desc->allow_dynamic_or_kinematic = defaults.mAllowDynamicOrKinematic;
  desc->is_sensor = defaults.mIsSensor;
  desc->allow_sleeping = defaults.mAllowSleeping;
  desc->enhanced_internal_edge_removal = defaults.mEnhancedInternalEdgeRemoval;
  desc->friction = defaults.mFriction;
  desc->restitution = defaults.mRestitution;
  desc->linear_damping = defaults.mLinearDamping;
  desc->angular_damping = defaults.mAngularDamping;
  desc->max_linear_velocity = defaults.mMaxLinearVelocity;
  desc->max_angular_velocity = defaults.mMaxAngularVelocity;
  desc->gravity_factor = defaults.mGravityFactor;
}

namespace {

ZJoltResult BuildCreationSettings(const ZJoltBodyDesc &desc,
                                  JPH::BodyCreationSettings *out) {
  if (desc.shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a body needs a shape");
  }

  out->mPosition = zjolt::ToJoltR(desc.position);
  out->mRotation = zjolt::ToJoltRotation(desc.rotation);
  out->mLinearVelocity = zjolt::ToJolt(desc.linear_velocity);
  out->mAngularVelocity = zjolt::ToJolt(desc.angular_velocity);
  out->SetShape(zjolt::ToJolt(desc.shape));
  out->mCollisionGroup = zjolt::ToJolt(&desc.collision_group);
  out->mUserData = desc.user_data;
  out->mObjectLayer = static_cast<JPH::ObjectLayer>(desc.object_layer);
  out->mMotionType = ToJoltMotionType(desc.motion_type);
  out->mMotionQuality = desc.motion_quality == ZJOLT_MOTION_QUALITY_LINEAR_CAST
                            ? JPH::EMotionQuality::LinearCast
                            : JPH::EMotionQuality::Discrete;
  out->mAllowedDOFs = static_cast<JPH::EAllowedDOFs>(desc.allowed_dofs);
  out->mAllowDynamicOrKinematic = desc.allow_dynamic_or_kinematic;
  out->mIsSensor = desc.is_sensor;
  out->mAllowSleeping = desc.allow_sleeping;
  out->mEnhancedInternalEdgeRemoval = desc.enhanced_internal_edge_removal;
  out->mFriction = desc.friction;
  out->mRestitution = desc.restitution;
  out->mLinearDamping = desc.linear_damping;
  out->mAngularDamping = desc.angular_damping;
  out->mMaxLinearVelocity = desc.max_linear_velocity;
  out->mMaxAngularVelocity = desc.max_angular_velocity;
  out->mGravityFactor = desc.gravity_factor;

  if (desc.override_mass_properties == ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_INERTIA) {
    if (!(desc.mass > 0.0f)) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "override_mass_properties asks for an explicit mass, so mass must "
          "be positive");
    }
    out->mOverrideMassProperties =
        JPH::EOverrideMassProperties::CalculateInertia;
    out->mMassPropertiesOverride.mMass = desc.mass;
  } else {
    out->mOverrideMassProperties =
        JPH::EOverrideMassProperties::CalculateMassAndInertia;
  }
  return ZJOLT_RESULT_OK;
}

}  // namespace

ZJoltResult zjoltBodyCreate(ZJoltPhysicsSystem *system,
                            const ZJoltBodyDesc *desc, ZJoltBodyId *out) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out, (ZJoltBodyId)ZJOLT_BODY_ID_INVALID));
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::BodyCreationSettings settings;
  const ZJoltResult built = BuildCreationSettings(*desc, &settings);
  if (built != ZJOLT_RESULT_OK) return built;

  JPH::Body *body = Interface(system)->CreateBody(settings);
  if (body == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_OUT_OF_MEMORY,
        "the system is already holding max_bodies bodies");
  }
  *out = zjolt::ToC(body->GetID());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltBodyCreateAndAdd(ZJoltPhysicsSystem *system,
                                  const ZJoltBodyDesc *desc,
                                  ZJoltActivation activation,
                                  ZJoltBodyId *out) {
  const ZJoltResult created = zjoltBodyCreate(system, desc, out);
  if (created != ZJOLT_RESULT_OK) return created;
  Interface(system)->AddBody(zjolt::ToJolt(*out), ToJoltActivation(activation));
  return ZJOLT_RESULT_OK;
}

void zjoltBodyDestroy(ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  const JPH::BodyID id = zjolt::ToJolt(body);
  if (iface->IsAdded(id)) iface->RemoveBody(id);
  iface->DestroyBody(id);
}

void zjoltBodyAdd(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                  ZJoltActivation activation) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->AddBody(zjolt::ToJolt(body), ToJoltActivation(activation));
}

void zjoltBodyRemove(ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->RemoveBody(zjolt::ToJolt(body));
}

bool zjoltBodyIsAdded(const ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return false;
  return iface->IsAdded(zjolt::ToJolt(body));
}

bool zjoltBodyIsActive(const ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return false;
  return iface->IsActive(zjolt::ToJolt(body));
}

void zjoltBodyActivate(ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->ActivateBody(zjolt::ToJolt(body));
}

void zjoltBodyDeactivate(ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->DeactivateBody(zjolt::ToJolt(body));
}

void zjoltBodyResetSleepTimer(ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->ResetSleepTimer(zjolt::ToJolt(body));
}

ZJoltBodyType zjoltBodyGetBodyType(const ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return ZJOLT_BODY_TYPE_RIGID_BODY;
  return ToCBodyType(iface->GetBodyType(zjolt::ToJolt(body)));
}

//===----------------------------------------------------------------------===//
// Motion type and transform
//===----------------------------------------------------------------------===//

void zjoltBodySetMotionType(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                            ZJoltMotionType type, ZJoltActivation activation) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetMotionType(zjolt::ToJolt(body), ToJoltMotionType(type),
                       ToJoltActivation(activation));
}

ZJoltMotionType zjoltBodyGetMotionType(const ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return ZJOLT_MOTION_TYPE_STATIC;
  return ToCMotionType(iface->GetMotionType(zjolt::ToJolt(body)));
}

void zjoltBodySetMotionQuality(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                               ZJoltMotionQuality quality) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetMotionQuality(zjolt::ToJolt(body), ToJoltMotionQuality(quality));
}

ZJoltMotionQuality zjoltBodyGetMotionQuality(const ZJoltPhysicsSystem *system,
                                             ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return ZJOLT_MOTION_QUALITY_DISCRETE;
  return ToCMotionQuality(iface->GetMotionQuality(zjolt::ToJolt(body)));
}

void zjoltBodySetPositionAndRotation(ZJoltPhysicsSystem *system,
                                     ZJoltBodyId body,
                                     const ZJoltRVec3 *position,
                                     const ZJoltQuat *rotation,
                                     ZJoltActivation activation) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || position == nullptr) return;
  const JPH::BodyID id = zjolt::ToJolt(body);
  const JPH::Quat orientation =
      rotation != nullptr ? zjolt::ToJoltRotation(*rotation) : iface->GetRotation(id);
  iface->SetPositionAndRotation(id, zjolt::ToJoltR(*position), orientation,
                                ToJoltActivation(activation));
}

void zjoltBodyGetPositionAndRotation(const ZJoltPhysicsSystem *system,
                                     ZJoltBodyId body,
                                     ZJoltRVec3 *out_position,
                                     ZJoltQuat *out_rotation) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  JPH::RVec3 position = JPH::RVec3::sZero();
  JPH::Quat rotation = JPH::Quat::sIdentity();
  iface->GetPositionAndRotation(zjolt::ToJolt(body), position, rotation);
  zjolt::WriteRVec3(out_position, position);
  zjolt::WriteQuat(out_rotation, rotation);
}

void zjoltBodyGetCenterOfMassPosition(const ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body, ZJoltRVec3 *out) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || out == nullptr) return;
  zjolt::WriteRVec3(out,
                    iface->GetCenterOfMassPosition(zjolt::ToJolt(body)));
}

void zjoltBodyGetWorldTransform(const ZJoltPhysicsSystem *system,
                                ZJoltBodyId body, ZJoltRMat44 *out) {
  if (out == nullptr) return;
  const JPH::BodyInterface *iface = Interface(system);
  // Identity for a NULL system too, so the promise the header makes about a
  // failed body lock holds for every way this can decline to answer.
  if (iface == nullptr) {
    zjolt::WriteRMat44(out, JPH::RMat44::sIdentity());
    return;
  }
  zjolt::WriteRMat44(out, iface->GetWorldTransform(zjolt::ToJolt(body)));
}

void zjoltBodyGetCenterOfMassTransform(const ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body, ZJoltRMat44 *out) {
  if (out == nullptr) return;
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) {
    zjolt::WriteRMat44(out, JPH::RMat44::sIdentity());
    return;
  }
  zjolt::WriteRMat44(out,
                     iface->GetCenterOfMassTransform(zjolt::ToJolt(body)));
}

ZJoltResult zjoltBodyGetInverseInertia(const ZJoltPhysicsSystem *system,
                                       ZJoltBodyId body, ZJoltMat44 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // The lock is taken here rather than left to BodyInterface's own
  // GetInverseInertia because the motion type has to be read and acted on
  // under the SAME lock: between two separate calls the body could be made
  // static, and the second would then reach Body::GetInverseInertia's
  // assertion after the first had cleared it.
  JPH::BodyLockRead lock(system->system.GetBodyLockInterface(),
                         zjolt::ToJolt(body));
  if (!lock.Succeeded()) {
    return zjolt::SetError(ZJOLT_RESULT_BODY_NOT_FOUND,
                           "body id does not name a live body in this system");
  }
  const JPH::Body &jolt_body = lock.GetBody();
  if (!jolt_body.IsDynamic()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "only a dynamic body has an inverse inertia; a static or kinematic "
        "one is driven rather than accelerated");
  }
  zjolt::WriteMat44(out, jolt_body.GetInverseInertia());
  return ZJOLT_RESULT_OK;
}

void zjoltBodySetPositionAndRotationWhenChanged(ZJoltPhysicsSystem *system,
                                                ZJoltBodyId body,
                                                const ZJoltRVec3 *position,
                                                const ZJoltQuat *rotation,
                                                ZJoltActivation activation) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || position == nullptr) return;
  const JPH::BodyID id = zjolt::ToJolt(body);
  const JPH::Quat orientation =
      rotation != nullptr ? zjolt::ToJoltRotation(*rotation) : iface->GetRotation(id);
  iface->SetPositionAndRotationWhenChanged(id, zjolt::ToJoltR(*position),
                                           orientation,
                                           ToJoltActivation(activation));
}

void zjoltBodyMoveKinematic(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                            const ZJoltRVec3 *target_position,
                            const ZJoltQuat *target_rotation,
                            float delta_time) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || target_position == nullptr) return;
  const JPH::BodyID id = zjolt::ToJolt(body);
  const JPH::Quat orientation = target_rotation != nullptr
                                    ? zjolt::ToJoltRotation(*target_rotation)
                                    : iface->GetRotation(id);
  iface->MoveKinematic(id, zjolt::ToJoltR(*target_position), orientation,
                       delta_time);
}

void zjoltBodySetPositionRotationAndVelocity(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *linear_velocity,
    const ZJoltVec3 *angular_velocity) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || position == nullptr || linear_velocity == nullptr ||
      angular_velocity == nullptr)
    return;
  const JPH::BodyID id = zjolt::ToJolt(body);
  const JPH::Quat orientation =
      rotation != nullptr ? zjolt::ToJoltRotation(*rotation) : iface->GetRotation(id);
  iface->SetPositionRotationAndVelocity(
      id, zjolt::ToJoltR(*position), orientation,
      zjolt::ToJolt(*linear_velocity), zjolt::ToJolt(*angular_velocity));
}

//===----------------------------------------------------------------------===//
// Velocity and forces
//===----------------------------------------------------------------------===//

void zjoltBodySetLinearVelocity(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                const ZJoltVec3 *velocity) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || velocity == nullptr) return;
  iface->SetLinearVelocity(zjolt::ToJolt(body), zjolt::ToJolt(*velocity));
}

void zjoltBodyGetLinearVelocity(const ZJoltPhysicsSystem *system,
                                ZJoltBodyId body, ZJoltVec3 *out) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, iface->GetLinearVelocity(zjolt::ToJolt(body)));
}

void zjoltBodySetAngularVelocity(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                 const ZJoltVec3 *velocity) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || velocity == nullptr) return;
  iface->SetAngularVelocity(zjolt::ToJolt(body), zjolt::ToJolt(*velocity));
}

void zjoltBodyGetAngularVelocity(const ZJoltPhysicsSystem *system,
                                 ZJoltBodyId body, ZJoltVec3 *out) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, iface->GetAngularVelocity(zjolt::ToJolt(body)));
}

void zjoltBodySetLinearAndAngularVelocity(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *linear_velocity,
                                          const ZJoltVec3 *angular_velocity) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || linear_velocity == nullptr ||
      angular_velocity == nullptr)
    return;
  iface->SetLinearAndAngularVelocity(zjolt::ToJolt(body),
                                     zjolt::ToJolt(*linear_velocity),
                                     zjolt::ToJolt(*angular_velocity));
}

void zjoltBodyGetLinearAndAngularVelocity(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          ZJoltVec3 *out_linear_velocity,
                                          ZJoltVec3 *out_angular_velocity) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  JPH::Vec3 linear = JPH::Vec3::sZero();
  JPH::Vec3 angular = JPH::Vec3::sZero();
  iface->GetLinearAndAngularVelocity(zjolt::ToJolt(body), linear, angular);
  zjolt::WriteVec3(out_linear_velocity, linear);
  zjolt::WriteVec3(out_angular_velocity, angular);
}

void zjoltBodyAddLinearVelocity(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                const ZJoltVec3 *linear_velocity) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || linear_velocity == nullptr) return;
  iface->AddLinearVelocity(zjolt::ToJolt(body), zjolt::ToJolt(*linear_velocity));
}

void zjoltBodyAddLinearAndAngularVelocity(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          const ZJoltVec3 *linear_velocity,
                                          const ZJoltVec3 *angular_velocity) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || linear_velocity == nullptr ||
      angular_velocity == nullptr)
    return;
  iface->AddLinearAndAngularVelocity(zjolt::ToJolt(body),
                                     zjolt::ToJolt(*linear_velocity),
                                     zjolt::ToJolt(*angular_velocity));
}

void zjoltBodyGetPointVelocity(const ZJoltPhysicsSystem *system,
                               ZJoltBodyId body, const ZJoltRVec3 *point,
                               ZJoltVec3 *out) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || point == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, iface->GetPointVelocity(zjolt::ToJolt(body),
                                                zjolt::ToJoltR(*point)));
}

void zjoltBodyAddForce(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                       const ZJoltVec3 *force) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || force == nullptr) return;
  iface->AddForce(zjolt::ToJolt(body), zjolt::ToJolt(*force));
}

void zjoltBodyAddForceAtPoint(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                              const ZJoltVec3 *force, const ZJoltRVec3 *point) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || force == nullptr || point == nullptr) return;
  iface->AddForce(zjolt::ToJolt(body), zjolt::ToJolt(*force),
                  zjolt::ToJoltR(*point));
}

void zjoltBodyAddTorque(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                        const ZJoltVec3 *torque) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || torque == nullptr) return;
  iface->AddTorque(zjolt::ToJolt(body), zjolt::ToJolt(*torque));
}

void zjoltBodyAddImpulse(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                         const ZJoltVec3 *impulse) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || impulse == nullptr) return;
  iface->AddImpulse(zjolt::ToJolt(body), zjolt::ToJolt(*impulse));
}

void zjoltBodyAddImpulseAtPoint(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                const ZJoltVec3 *impulse,
                                const ZJoltRVec3 *point) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || impulse == nullptr || point == nullptr) return;
  iface->AddImpulse(zjolt::ToJolt(body), zjolt::ToJolt(*impulse),
                    zjolt::ToJoltR(*point));
}

void zjoltBodyAddAngularImpulse(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                const ZJoltVec3 *angular_impulse) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || angular_impulse == nullptr) return;
  iface->AddAngularImpulse(zjolt::ToJolt(body),
                           zjolt::ToJolt(*angular_impulse));
}

bool zjoltBodyApplyBuoyancyImpulse(ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body,
                                   const ZJoltRVec3 *surface_position,
                                   const ZJoltVec3 *surface_normal,
                                   float buoyancy, float linear_drag,
                                   float angular_drag,
                                   const ZJoltVec3 *fluid_velocity,
                                   const ZJoltVec3 *gravity,
                                   float delta_time) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || surface_position == nullptr ||
      surface_normal == nullptr || fluid_velocity == nullptr ||
      gravity == nullptr)
    return false;
  return iface->ApplyBuoyancyImpulse(
      zjolt::ToJolt(body), zjolt::ToJoltR(*surface_position),
      zjolt::ToJolt(*surface_normal), buoyancy, linear_drag, angular_drag,
      zjolt::ToJolt(*fluid_velocity), zjolt::ToJolt(*gravity), delta_time);
}

//===----------------------------------------------------------------------===//
// Properties
//===----------------------------------------------------------------------===//

void zjoltBodySetShape(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                       const ZJoltShape *shape, bool update_mass_properties,
                       ZJoltActivation activation) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || shape == nullptr) return;
  iface->SetShape(zjolt::ToJolt(body), zjolt::ToJolt(shape),
                  update_mass_properties, ToJoltActivation(activation));
}

void zjoltBodySetObjectLayer(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                             ZJoltObjectLayer layer) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetObjectLayer(zjolt::ToJolt(body),
                        static_cast<JPH::ObjectLayer>(layer));
}

ZJoltObjectLayer zjoltBodyGetObjectLayer(const ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return 0;
  return static_cast<ZJoltObjectLayer>(
      iface->GetObjectLayer(zjolt::ToJolt(body)));
}

void zjoltBodySetUserData(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                          uint64_t user_data) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetUserData(zjolt::ToJolt(body), user_data);
}

uint64_t zjoltBodyGetUserData(const ZJoltPhysicsSystem *system,
                              ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return 0;
  return iface->GetUserData(zjolt::ToJolt(body));
}

void zjoltBodySetFriction(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                          float friction) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetFriction(zjolt::ToJolt(body), friction);
}

float zjoltBodyGetFriction(const ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return 0.0f;
  return iface->GetFriction(zjolt::ToJolt(body));
}

void zjoltBodySetRestitution(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                             float restitution) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetRestitution(zjolt::ToJolt(body), restitution);
}

float zjoltBodyGetRestitution(const ZJoltPhysicsSystem *system,
                              ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return 0.0f;
  return iface->GetRestitution(zjolt::ToJolt(body));
}

void zjoltBodySetGravityFactor(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                               float factor) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetGravityFactor(zjolt::ToJolt(body), factor);
}

float zjoltBodyGetGravityFactor(const ZJoltPhysicsSystem *system,
                                ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return 0.0f;
  return iface->GetGravityFactor(zjolt::ToJolt(body));
}

void zjoltBodySetMaxLinearVelocity(ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body, float velocity) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetMaxLinearVelocity(zjolt::ToJolt(body), velocity);
}

float zjoltBodyGetMaxLinearVelocity(const ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  // Jolt's own construction-time default, forwarded rather than 0 — a caller
  // that never overrode the ceiling should read back the ceiling, not "none".
  if (iface == nullptr) return 500.0f;
  return iface->GetMaxLinearVelocity(zjolt::ToJolt(body));
}

void zjoltBodySetMaxAngularVelocity(ZJoltPhysicsSystem *system,
                                    ZJoltBodyId body, float velocity) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetMaxAngularVelocity(zjolt::ToJolt(body), velocity);
}

float zjoltBodyGetMaxAngularVelocity(const ZJoltPhysicsSystem *system,
                                     ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return 0.25f * JPH::JPH_PI * 60.0f;
  return iface->GetMaxAngularVelocity(zjolt::ToJolt(body));
}

void zjoltBodySetUseManifoldReduction(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body, bool use_reduction) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetUseManifoldReduction(zjolt::ToJolt(body), use_reduction);
}

bool zjoltBodyGetUseManifoldReduction(const ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return true;
  return iface->GetUseManifoldReduction(zjolt::ToJolt(body));
}

void zjoltBodySetIsSensor(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                          bool is_sensor) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return;
  iface->SetIsSensor(zjolt::ToJolt(body), is_sensor);
}

bool zjoltBodyIsSensor(const ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return false;
  return iface->IsSensor(zjolt::ToJolt(body));
}

const ZJoltPhysicsMaterial *zjoltBodyGetMaterial(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body,
    ZJoltSubShapeId sub_shape_id) {
  const JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr) return nullptr;
  // Forwarded as-is, including the case a stricter ABI would want to reject:
  // BodyInterface::GetMaterial answers with PhysicsMaterial::sDefault when its
  // body lock fails, so a destroyed body reads as a shape with no materials.
  // See the header — that is documented rather than papered over.
  return zjolt::ToC(iface->GetMaterial(zjolt::ToJolt(body),
                                       zjolt::ToJoltSubShapeId(sub_shape_id)));
}

void zjoltBodyNotifyShapeChanged(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                                 const ZJoltVec3 *previous_center_of_mass,
                                 bool update_mass_properties,
                                 ZJoltActivation activation) {
  JPH::BodyInterface *iface = Interface(system);
  if (iface == nullptr || previous_center_of_mass == nullptr) return;
  iface->NotifyShapeChanged(zjolt::ToJolt(body),
                            zjolt::ToJolt(*previous_center_of_mass),
                            update_mass_properties,
                            ToJoltActivation(activation));
}

//===----------------------------------------------------------------------===//
// Bulk read-back
//===----------------------------------------------------------------------===//

ZJoltResult zjoltBodyGetTransforms(const ZJoltPhysicsSystem *system,
                                   const ZJoltBodyId *ids, uint32_t count,
                                   ZJoltRVec3 *out_positions,
                                   ZJoltQuat *out_rotations,
                                   uint32_t *out_missing) {
  return VisitBodies(
      system, ids, count, out_missing,
      [&](uint32_t index, const JPH::Body *body) {
        if (body != nullptr) {
          if (out_positions != nullptr)
            out_positions[index] = zjolt::ToCR(body->GetPosition());
          if (out_rotations != nullptr)
            out_rotations[index] = zjolt::ToC(body->GetRotation());
        } else {
          // A body destroyed between the step and the read is ordinary. The
          // identity transform keeps the arrays parallel with the caller's id
          // list, which is what makes `out_missing` enough to recover.
          if (out_positions != nullptr)
            out_positions[index] = ZJoltRVec3{0, 0, 0};
          if (out_rotations != nullptr)
            out_rotations[index] = ZJoltQuat{0, 0, 0, 1};
        }
      });
}

ZJoltResult zjoltBodyGetMotions(const ZJoltPhysicsSystem *system,
                                const ZJoltBodyId *ids, uint32_t count,
                                ZJoltRVec3 *out_center_of_mass,
                                ZJoltVec3 *out_linear_velocities,
                                uint32_t *out_missing) {
  return VisitBodies(
      system, ids, count, out_missing,
      [&](uint32_t index, const JPH::Body *body) {
        if (body != nullptr) {
          if (out_center_of_mass != nullptr)
            out_center_of_mass[index] =
                zjolt::ToCR(body->GetCenterOfMassPosition());
          if (out_linear_velocities != nullptr) {
            // A static body has no motion properties; asking it for a
            // velocity would assert inside Jolt.
            out_linear_velocities[index] =
                body->IsStatic() ? ZJoltVec3{0, 0, 0}
                                 : zjolt::ToC(body->GetLinearVelocity());
          }
        } else {
          if (out_center_of_mass != nullptr)
            out_center_of_mass[index] = ZJoltRVec3{0, 0, 0};
          if (out_linear_velocities != nullptr)
            out_linear_velocities[index] = ZJoltVec3{0, 0, 0};
        }
      });
}

//===----------------------------------------------------------------------===//
// Locks
//
// Jolt's locks are RAII; the ABI's are explicit, because a scope is not
// something a C struct can express. The two _reserved slots hold the mutex and
// the lock interface, which is what a release needs and what the caller must
// not touch.
//===----------------------------------------------------------------------===//

namespace {

void ClearLock(ZJoltBodyLock *lock) {
  lock->body = nullptr;
  lock->_reserved[0] = nullptr;
  lock->_reserved[1] = nullptr;
}

void TakeLock(const ZJoltPhysicsSystem *system, ZJoltBodyId body,
              ZJoltBodyLock *out_lock, bool write) {
  if (out_lock == nullptr) return;
  ClearLock(out_lock);
  if (system == nullptr) return;

  const JPH::BodyID id = zjolt::ToJolt(body);
  if (id.IsInvalid()) return;

  const JPH::BodyLockInterface &iface = system->system.GetBodyLockInterface();
  JPH::SharedMutex *mutex = write ? iface.LockWrite(id) : iface.LockRead(id);
  JPH::Body *held = iface.TryGetBody(id);

  out_lock->body = held != nullptr ? zjolt::ToC(held) : nullptr;
  out_lock->_reserved[0] = mutex;
  out_lock->_reserved[1] =
      const_cast<JPH::BodyLockInterface *>(&iface);
}

void DropLock(ZJoltBodyLock *lock, bool write) {
  if (lock == nullptr || lock->_reserved[1] == nullptr) return;
  JPH::BodyLockInterface *iface =
      static_cast<JPH::BodyLockInterface *>(lock->_reserved[1]);
  JPH::SharedMutex *mutex =
      static_cast<JPH::SharedMutex *>(lock->_reserved[0]);
  if (mutex != nullptr) {
    if (write)
      iface->UnlockWrite(mutex);
    else
      iface->UnlockRead(mutex);
  }
  ClearLock(lock);
}

}  // namespace

void zjoltBodyLockRead(const ZJoltPhysicsSystem *system, ZJoltBodyId body,
                       ZJoltBodyLock *out_lock) {
  TakeLock(system, body, out_lock, false);
}

void zjoltBodyLockReadRelease(ZJoltBodyLock *lock) { DropLock(lock, false); }

void zjoltBodyLockWrite(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                        ZJoltBodyLock *out_lock) {
  TakeLock(system, body, out_lock, true);
}

void zjoltBodyLockWriteRelease(ZJoltBodyLock *lock) { DropLock(lock, true); }

//===----------------------------------------------------------------------===//
// Locked accessors
//===----------------------------------------------------------------------===//

ZJoltBodyId zjoltBodyGetId(const ZJoltBody *body) {
  if (body == nullptr) return ZJOLT_BODY_ID_INVALID;
  return zjolt::ToC(zjolt::ToJolt(body)->GetID());
}

void zjoltBodyGetPosition(const ZJoltBody *body, ZJoltRVec3 *out) {
  if (body == nullptr || out == nullptr) return;
  zjolt::WriteRVec3(out, zjolt::ToJolt(body)->GetPosition());
}

void zjoltBodyGetRotation(const ZJoltBody *body, ZJoltQuat *out) {
  if (body == nullptr || out == nullptr) return;
  zjolt::WriteQuat(out, zjolt::ToJolt(body)->GetRotation());
}

void zjoltBodyGetCenterOfMassPositionLocked(const ZJoltBody *body,
                                            ZJoltRVec3 *out) {
  if (body == nullptr || out == nullptr) return;
  zjolt::WriteRVec3(out, zjolt::ToJolt(body)->GetCenterOfMassPosition());
}

void zjoltBodyGetLinearVelocityLocked(const ZJoltBody *body, ZJoltVec3 *out) {
  if (body == nullptr || out == nullptr) return;
  const JPH::Body *impl = zjolt::ToJolt(body);
  zjolt::WriteVec3(out, impl->IsStatic() ? JPH::Vec3::sZero()
                                         : impl->GetLinearVelocity());
}

void zjoltBodyGetAngularVelocityLocked(const ZJoltBody *body, ZJoltVec3 *out) {
  if (body == nullptr || out == nullptr) return;
  const JPH::Body *impl = zjolt::ToJolt(body);
  zjolt::WriteVec3(out, impl->IsStatic() ? JPH::Vec3::sZero()
                                         : impl->GetAngularVelocity());
}

uint64_t zjoltBodyGetUserDataLocked(const ZJoltBody *body) {
  if (body == nullptr) return 0;
  return zjolt::ToJolt(body)->GetUserData();
}

ZJoltObjectLayer zjoltBodyGetObjectLayerLocked(const ZJoltBody *body) {
  if (body == nullptr) return 0;
  return static_cast<ZJoltObjectLayer>(zjolt::ToJolt(body)->GetObjectLayer());
}

ZJoltMotionType zjoltBodyGetMotionTypeLocked(const ZJoltBody *body) {
  if (body == nullptr) return ZJOLT_MOTION_TYPE_STATIC;
  return ToCMotionType(zjolt::ToJolt(body)->GetMotionType());
}

bool zjoltBodyIsActiveLocked(const ZJoltBody *body) {
  if (body == nullptr) return false;
  return zjolt::ToJolt(body)->IsActive();
}

bool zjoltBodyIsSensorLocked(const ZJoltBody *body) {
  if (body == nullptr) return false;
  return zjolt::ToJolt(body)->IsSensor();
}

const ZJoltShape *zjoltBodyGetShapeLocked(const ZJoltBody *body) {
  if (body == nullptr) return nullptr;
  return zjolt::ToC(zjolt::ToJolt(body)->GetShape());
}

void zjoltBodyGetWorldBounds(const ZJoltBody *body, ZJoltAABox *out) {
  if (out == nullptr) return;
  if (body == nullptr) {
    *out = ZJoltAABox{};
    return;
  }
  const JPH::AABox &bounds = zjolt::ToJolt(body)->GetWorldSpaceBounds();
  out->min = zjolt::ToC(bounds.mMin);
  out->max = zjolt::ToC(bounds.mMax);
}

//===----------------------------------------------------------------------===//
// Locked mutators
//
// Each guards against a static body: Jolt's Body setters assert on one,
// because a static body has no motion properties to write through.
//===----------------------------------------------------------------------===//

void zjoltBodySetLinearVelocityLocked(ZJoltBody *body, const ZJoltVec3 *velocity) {
  if (body == nullptr || velocity == nullptr) return;
  JPH::Body *impl = zjolt::ToJolt(body);
  if (impl->IsStatic()) return;
  impl->SetLinearVelocity(zjolt::ToJolt(*velocity));
}

void zjoltBodySetAngularVelocityLocked(ZJoltBody *body,
                                    const ZJoltVec3 *velocity) {
  if (body == nullptr || velocity == nullptr) return;
  JPH::Body *impl = zjolt::ToJolt(body);
  if (impl->IsStatic()) return;
  impl->SetAngularVelocity(zjolt::ToJolt(*velocity));
}

void zjoltBodySetUserDataLocked(ZJoltBody *body, uint64_t user_data) {
  if (body == nullptr) return;
  zjolt::ToJolt(body)->SetUserData(user_data);
}

void zjoltBodySetFrictionLocked(ZJoltBody *body, float friction) {
  if (body == nullptr) return;
  zjolt::ToJolt(body)->SetFriction(friction);
}

void zjoltBodySetRestitutionLocked(ZJoltBody *body, float restitution) {
  if (body == nullptr) return;
  zjolt::ToJolt(body)->SetRestitution(restitution);
}

void zjoltBodyAddImpulseLocked(ZJoltBody *body, const ZJoltVec3 *impulse) {
  if (body == nullptr || impulse == nullptr) return;
  JPH::Body *impl = zjolt::ToJolt(body);
  if (impl->IsStatic() || impl->IsKinematic()) return;
  impl->AddImpulse(zjolt::ToJolt(*impulse));
}

}  // extern "C"
