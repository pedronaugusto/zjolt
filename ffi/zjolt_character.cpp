//===----------------------------------------------------------------------===//
// zjolt — CharacterVirtual.
//
// A virtual character is not a rigid body: it is a shape the host sweeps
// through the world itself, which is what lets it stop dead, turn on the spot
// and climb a step — none of which a barrel with a collision shape can do.
// The trade is that the world cannot see it, so an optional inner rigid body
// exists to give it presence.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace {

ZJoltGroundState ToCGroundState(JPH::CharacterBase::EGroundState state) {
  switch (state) {
    case JPH::CharacterBase::EGroundState::OnGround:
      return ZJOLT_GROUND_STATE_ON_GROUND;
    case JPH::CharacterBase::EGroundState::OnSteepGround:
      return ZJOLT_GROUND_STATE_ON_STEEP_GROUND;
    case JPH::CharacterBase::EGroundState::NotSupported:
      return ZJOLT_GROUND_STATE_NOT_SUPPORTED;
    case JPH::CharacterBase::EGroundState::InAir:
      break;
  }
  return ZJOLT_GROUND_STATE_IN_AIR;
}

JPH::CharacterVirtual *Impl(ZJoltCharacter *character) {
  return character != nullptr ? character->impl.GetPtr() : nullptr;
}

const JPH::CharacterVirtual *Impl(const ZJoltCharacter *character) {
  return character != nullptr ? character->impl.GetPtr() : nullptr;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Settings
//===----------------------------------------------------------------------===//

void zjoltCharacterDescInit(ZJoltCharacterDesc *desc) {
  if (desc == nullptr) return;

  // Read out of Jolt's own defaults rather than transcribed, so an upstream
  // tuning change moves this with it.
  const JPH::CharacterVirtualSettings defaults;

  *desc = ZJoltCharacterDesc{};
  desc->shape = nullptr;
  desc->position = ZJoltRVec3{0, 0, 0};
  desc->rotation = ZJoltQuat{0, 0, 0, 1};
  desc->up = zjolt::ToC(defaults.mUp);
  desc->shape_offset = zjolt::ToC(defaults.mShapeOffset);
  desc->user_data = 0;
  desc->max_slope_angle = defaults.mMaxSlopeAngle;
  desc->mass = defaults.mMass;
  desc->max_strength = defaults.mMaxStrength;
  desc->predictive_contact_distance = defaults.mPredictiveContactDistance;
  desc->character_padding = defaults.mCharacterPadding;
  desc->penetration_recovery_speed = defaults.mPenetrationRecoverySpeed;
  desc->collision_tolerance = defaults.mCollisionTolerance;
  desc->hit_reduction_cos_max_angle = defaults.mHitReductionCosMaxAngle;
  desc->max_collision_iterations = defaults.mMaxCollisionIterations;
  desc->max_constraint_iterations = defaults.mMaxConstraintIterations;
  desc->max_num_hits = defaults.mMaxNumHits;
  desc->back_face_mode =
      defaults.mBackFaceMode == JPH::EBackFaceMode::CollideWithBackFaces
          ? ZJOLT_BACK_FACE_MODE_COLLIDE
          : ZJOLT_BACK_FACE_MODE_IGNORE;
  desc->enhanced_internal_edge_removal = defaults.mEnhancedInternalEdgeRemoval;
  desc->inner_body_shape = nullptr;
  desc->inner_body_layer = static_cast<ZJoltObjectLayer>(defaults.mInnerBodyLayer);
}

void zjoltCharacterUpdateSettingsInit(ZJoltCharacterUpdateSettings *settings) {
  if (settings == nullptr) return;
  const JPH::CharacterVirtual::ExtendedUpdateSettings defaults;
  settings->stick_to_floor_step_down = zjolt::ToC(defaults.mStickToFloorStepDown);
  settings->walk_stairs_step_up = zjolt::ToC(defaults.mWalkStairsStepUp);
  settings->walk_stairs_min_step_forward = defaults.mWalkStairsMinStepForward;
  settings->walk_stairs_step_forward_test = defaults.mWalkStairsStepForwardTest;
  settings->walk_stairs_cos_angle_forward_contact =
      defaults.mWalkStairsCosAngleForwardContact;
  settings->walk_stairs_step_down_extra =
      zjolt::ToC(defaults.mWalkStairsStepDownExtra);
}

//===----------------------------------------------------------------------===//
// Lifetime
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCharacterCreate(ZJoltPhysicsSystem *system,
                                 const ZJoltCharacterDesc *desc,
                                 ZJoltCharacter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (desc->shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a character needs a shape");
  }

  JPH::CharacterVirtualSettings settings;
  settings.mShape = zjolt::ToJolt(desc->shape);
  settings.mUp = zjolt::ToJolt(desc->up);
  settings.mShapeOffset = zjolt::ToJolt(desc->shape_offset);
  settings.mMaxSlopeAngle = desc->max_slope_angle;
  settings.mMass = desc->mass;
  settings.mMaxStrength = desc->max_strength;
  settings.mPredictiveContactDistance = desc->predictive_contact_distance;
  settings.mCharacterPadding = desc->character_padding;
  settings.mPenetrationRecoverySpeed = desc->penetration_recovery_speed;
  settings.mCollisionTolerance = desc->collision_tolerance;
  settings.mHitReductionCosMaxAngle = desc->hit_reduction_cos_max_angle;
  settings.mMaxCollisionIterations = desc->max_collision_iterations;
  settings.mMaxConstraintIterations = desc->max_constraint_iterations;
  settings.mMaxNumHits = desc->max_num_hits;
  settings.mBackFaceMode = desc->back_face_mode == ZJOLT_BACK_FACE_MODE_IGNORE
                               ? JPH::EBackFaceMode::IgnoreBackFaces
                               : JPH::EBackFaceMode::CollideWithBackFaces;
  settings.mEnhancedInternalEdgeRemoval = desc->enhanced_internal_edge_removal;
  if (desc->inner_body_shape != nullptr) {
    settings.mInnerBodyShape = zjolt::ToJolt(desc->inner_body_shape);
    settings.mInnerBodyLayer =
        static_cast<JPH::ObjectLayer>(desc->inner_body_layer);
  }

  // The supporting volume is a plane below the character's centre; anything
  // contacted above it does not count as ground. Jolt's default is effectively
  // "everything supports", which reports a wall as ground. Placing it at the
  // bottom of the shape is what makes GetGroundState mean what its name says.
  const JPH::AABox local_bounds = settings.mShape->GetLocalBounds();
  const JPH::Vec3 up = settings.mUp.NormalizedOr(JPH::Vec3::sAxisY());
  settings.mSupportingVolume =
      JPH::Plane(up, -up.Dot(local_bounds.mMin) - desc->character_padding);

  ZJoltCharacter *handle = zjolt::New<ZJoltCharacter>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  JPH::CharacterVirtual *character = zjolt::New<JPH::CharacterVirtual>(
      &settings, zjolt::ToJoltR(desc->position),
      zjolt::ToJoltRotation(desc->rotation),
      desc->user_data, &system->system);
  if (character == nullptr) {
    zjolt::Delete(handle);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }

  handle->impl = character;
  handle->owner = system;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltCharacterDestroy(ZJoltCharacter *character) {
  if (character == nullptr) return;
  // Dropping the Ref runs CharacterVirtual's destructor, which removes the
  // inner body from the system if there was one.
  character->impl = nullptr;
  zjolt::Delete(character);
  zjolt::HandleDestroyed();
}

//===----------------------------------------------------------------------===//
// Update
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCharacterUpdate(ZJoltCharacter *character, float delta_time,
                                 const ZJoltVec3 *gravity,
                                 const ZJoltCharacterUpdateSettings *settings,
                                 const ZJoltQueryFilters *filters) {
  ZJOLT_ENTER();
  JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, gravity)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!(delta_time > 0.0f)) return ZJOLT_RESULT_OK;

  zjolt::QueryFilters adapters(filters);
  const JPH::ShapeFilter shape_filter;
  JPH::TempAllocator *temp = character->owner->temp_allocator;

  if (settings == nullptr) {
    impl->Update(delta_time, zjolt::ToJolt(*gravity), adapters.broad_phase,
                 adapters.object_layer, adapters.body, shape_filter, *temp);
    return ZJOLT_RESULT_OK;
  }

  JPH::CharacterVirtual::ExtendedUpdateSettings extended;
  extended.mStickToFloorStepDown =
      zjolt::ToJolt(settings->stick_to_floor_step_down);
  extended.mWalkStairsStepUp = zjolt::ToJolt(settings->walk_stairs_step_up);
  extended.mWalkStairsMinStepForward = settings->walk_stairs_min_step_forward;
  extended.mWalkStairsStepForwardTest = settings->walk_stairs_step_forward_test;
  extended.mWalkStairsCosAngleForwardContact =
      settings->walk_stairs_cos_angle_forward_contact;
  extended.mWalkStairsStepDownExtra =
      zjolt::ToJolt(settings->walk_stairs_step_down_extra);

  impl->ExtendedUpdate(delta_time, zjolt::ToJolt(*gravity), extended,
                       adapters.broad_phase, adapters.object_layer,
                       adapters.body, shape_filter, *temp);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// State
//===----------------------------------------------------------------------===//

void zjoltCharacterGetPosition(const ZJoltCharacter *character,
                               ZJoltRVec3 *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteRVec3(out, impl->GetPosition());
}

void zjoltCharacterSetPosition(ZJoltCharacter *character,
                               const ZJoltRVec3 *position) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || position == nullptr) return;
  impl->SetPosition(zjolt::ToJoltR(*position));
}

void zjoltCharacterGetRotation(const ZJoltCharacter *character,
                               ZJoltQuat *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteQuat(out, impl->GetRotation());
}

void zjoltCharacterSetRotation(ZJoltCharacter *character,
                               const ZJoltQuat *rotation) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || rotation == nullptr) return;
  impl->SetRotation(zjolt::ToJoltRotation(*rotation));
}

void zjoltCharacterGetLinearVelocity(const ZJoltCharacter *character,
                                     ZJoltVec3 *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetLinearVelocity());
}

void zjoltCharacterSetLinearVelocity(ZJoltCharacter *character,
                                     const ZJoltVec3 *velocity) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || velocity == nullptr) return;
  impl->SetLinearVelocity(zjolt::ToJolt(*velocity));
}

ZJoltGroundState zjoltCharacterGetGroundState(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_GROUND_STATE_IN_AIR;
  return ToCGroundState(impl->GetGroundState());
}

bool zjoltCharacterIsSupported(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return false;
  return impl->IsSupported();
}

void zjoltCharacterGetGroundNormal(const ZJoltCharacter *character,
                                   ZJoltVec3 *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetGroundNormal());
}

void zjoltCharacterGetGroundVelocity(const ZJoltCharacter *character,
                                     ZJoltVec3 *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetGroundVelocity());
}

void zjoltCharacterGetGroundPosition(const ZJoltCharacter *character,
                                     ZJoltRVec3 *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteRVec3(out, impl->GetGroundPosition());
}

ZJoltBodyId zjoltCharacterGetGroundBodyId(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_BODY_ID_INVALID;
  return zjolt::ToC(impl->GetGroundBodyID());
}

uint64_t zjoltCharacterGetGroundUserData(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return 0;
  return impl->GetGroundUserData();
}

void zjoltCharacterUpdateGroundVelocity(ZJoltCharacter *character) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->UpdateGroundVelocity();
}

ZJoltResult zjoltCharacterSetShape(ZJoltCharacter *character,
                                   const ZJoltShape *shape,
                                   float max_penetration_depth,
                                   const ZJoltQueryFilters *filters,
                                   bool *out_changed) {
  ZJOLT_ENTER(out_changed);
  JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, shape)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::ShapeFilter shape_filter;
  const bool changed = impl->SetShape(
      zjolt::ToJolt(shape), max_penetration_depth, adapters.broad_phase,
      adapters.object_layer, adapters.body, shape_filter,
      *character->owner->temp_allocator);

  // A refused shape change is a normal outcome — standing up under a low
  // ceiling — so it is reported through out_changed rather than as an error.
  //
  // The inner body, when there is one, has to follow. Jolt's
  // SetInnerBodyShape does not check for its absence first; it would reach
  // BodyInterface::SetShape with an invalid id, which happens to be harmless,
  // but relying on that is not the same as not doing it.
  if (changed && !impl->GetInnerBodyID().IsInvalid())
    impl->SetInnerBodyShape(zjolt::ToJolt(shape));
  if (out_changed != nullptr) *out_changed = changed;
  return ZJOLT_RESULT_OK;
}

const ZJoltShape *zjoltCharacterGetShape(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return nullptr;
  return zjolt::ToC(impl->GetShape());
}

ZJoltBodyId zjoltCharacterGetInnerBodyId(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_BODY_ID_INVALID;
  return zjolt::ToC(impl->GetInnerBodyID());
}

}  // extern "C"
