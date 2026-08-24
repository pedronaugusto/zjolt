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

#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

/// RigidCharacter — Jolt's Character, a rigid-body-backed character. Defined
/// at global scope, ahead of the anonymous namespace below, because its
/// Impl() overloads need a complete type to dereference.
struct ZJoltRigidCharacter {
  JPH::Ref<JPH::Character> impl;
};

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

JPH::EActivation ToJoltActivation(ZJoltActivation activation) {
  return activation == ZJOLT_ACTIVATION_DONT_ACTIVATE
             ? JPH::EActivation::DontActivate
             : JPH::EActivation::Activate;
}

/// A callback names the character it fired for by id rather than handle: Jolt
/// hands it a raw pointer to its own CharacterVirtual, which is not the
/// ZJoltCharacter this API gave out and cannot be turned back into one.
ZJoltCharacterId CharacterIdOf(const JPH::CharacterVirtual *character) {
  return character != nullptr ? character->GetID().GetValue()
                              : ZJOLT_CHARACTER_ID_INVALID;
}

void FillContact(ZJoltCharacterContact *out, const JPH::CharacterContact &contact) {
  if (out == nullptr) return;
  *out = ZJoltCharacterContact{};
  out->body_b = zjolt::ToC(contact.mBodyB);
  // CharacterID's invalid sentinel is 0xffffffff, the same bit pattern as
  // ZJOLT_CHARACTER_ID_INVALID, so no translation is needed either way.
  out->character_id_b = contact.mCharacterIDB.GetValue();
  out->sub_shape_id_b = zjolt::ToC(contact.mSubShapeIDB);
  out->position = zjolt::ToCR(contact.mPosition);
  out->linear_velocity = zjolt::ToC(contact.mLinearVelocity);
  out->contact_normal = zjolt::ToC(contact.mContactNormal);
  out->surface_normal = zjolt::ToC(contact.mSurfaceNormal);
  out->distance = contact.mDistance;
  out->fraction = contact.mFraction;
  // EMotionType and ZJoltMotionType share ordinal values (static, kinematic,
  // dynamic); a static_cast is exact rather than merely convenient.
  out->motion_type_b = static_cast<ZJoltMotionType>(contact.mMotionTypeB);
  out->is_sensor_b = contact.mIsSensorB;
  out->user_data = contact.mUserData;
  out->material = zjolt::ToC(contact.mMaterial);
  out->had_collision = contact.mHadCollision;
  out->was_discarded = contact.mWasDiscarded;
  out->can_push_character = contact.mCanPushCharacter;
  out->is_back_facing_contact = contact.mIsBackFacingContact;
}

JPH::Character *Impl(ZJoltRigidCharacter *character) {
  return character != nullptr ? character->impl.GetPtr() : nullptr;
}

const JPH::Character *Impl(const ZJoltRigidCharacter *character) {
  return character != nullptr ? character->impl.GetPtr() : nullptr;
}

}  // namespace

//===----------------------------------------------------------------------===//
// CharacterContactListener
//
// Forwards Jolt's virtual callbacks to plain C function pointers. A field left
// NULL in the callbacks struct behaves exactly as Jolt's own default override:
// accept every contact, change nothing.
//===----------------------------------------------------------------------===//

struct ZJoltCharacterContactListener final : public JPH::CharacterContactListener {
  ZJoltCharacterContactListenerCallbacks callbacks;

  explicit ZJoltCharacterContactListener(
      const ZJoltCharacterContactListenerCallbacks &cb)
      : callbacks(cb) {}

  void OnAdjustBodyVelocity(const JPH::CharacterVirtual *inCharacter,
                            const JPH::Body &inBody2,
                            JPH::Vec3 &ioLinearVelocity,
                            JPH::Vec3 &ioAngularVelocity) override {
    if (callbacks.on_adjust_body_velocity == nullptr) return;
    ZJoltVec3 linear = zjolt::ToC(ioLinearVelocity);
    ZJoltVec3 angular = zjolt::ToC(ioAngularVelocity);
    callbacks.on_adjust_body_velocity(callbacks.user, CharacterIdOf(inCharacter),
                                      zjolt::ToC(inBody2.GetID()), &linear,
                                      &angular);
    ioLinearVelocity = zjolt::ToJolt(linear);
    ioAngularVelocity = zjolt::ToJolt(angular);
  }

  bool OnContactValidate(const JPH::CharacterVirtual *inCharacter,
                         const JPH::CharacterContact &inContact) override {
    if (callbacks.on_contact_validate == nullptr) return true;
    ZJoltCharacterContact contact;
    FillContact(&contact, inContact);
    return callbacks.on_contact_validate(callbacks.user, CharacterIdOf(inCharacter),
                                         &contact);
  }

  void OnContactAdded(const JPH::CharacterVirtual *inCharacter,
                      const JPH::CharacterContact &inContact,
                      JPH::CharacterContactSettings &ioSettings) override {
    if (callbacks.on_contact_added == nullptr) return;
    ZJoltCharacterContact contact;
    FillContact(&contact, inContact);
    ZJoltCharacterContactSettings settings{ioSettings.mCanPushCharacter,
                                           ioSettings.mCanReceiveImpulses};
    callbacks.on_contact_added(callbacks.user, CharacterIdOf(inCharacter),
                               &contact, &settings);
    ioSettings.mCanPushCharacter = settings.can_push_character;
    ioSettings.mCanReceiveImpulses = settings.can_receive_impulses;
  }

  void OnContactPersisted(const JPH::CharacterVirtual *inCharacter,
                          const JPH::CharacterContact &inContact,
                          JPH::CharacterContactSettings &ioSettings) override {
    if (callbacks.on_contact_persisted == nullptr) return;
    ZJoltCharacterContact contact;
    FillContact(&contact, inContact);
    ZJoltCharacterContactSettings settings{ioSettings.mCanPushCharacter,
                                           ioSettings.mCanReceiveImpulses};
    callbacks.on_contact_persisted(callbacks.user, CharacterIdOf(inCharacter),
                                   &contact, &settings);
    ioSettings.mCanPushCharacter = settings.can_push_character;
    ioSettings.mCanReceiveImpulses = settings.can_receive_impulses;
  }

  void OnContactRemoved(const JPH::CharacterVirtual *inCharacter,
                        const JPH::BodyID &inBodyID2,
                        const JPH::SubShapeID &inSubShapeID2) override {
    if (callbacks.on_contact_removed == nullptr) return;
    callbacks.on_contact_removed(callbacks.user, CharacterIdOf(inCharacter),
                                 zjolt::ToC(inBodyID2), zjolt::ToC(inSubShapeID2));
  }

  bool OnCharacterContactValidate(const JPH::CharacterVirtual *inCharacter,
                                  const JPH::CharacterContact &inContact) override {
    if (callbacks.on_character_contact_validate == nullptr) return true;
    ZJoltCharacterContact contact;
    FillContact(&contact, inContact);
    return callbacks.on_character_contact_validate(
        callbacks.user, CharacterIdOf(inCharacter), &contact);
  }

  void OnCharacterContactAdded(const JPH::CharacterVirtual *inCharacter,
                               const JPH::CharacterContact &inContact,
                               JPH::CharacterContactSettings &ioSettings) override {
    if (callbacks.on_character_contact_added == nullptr) return;
    ZJoltCharacterContact contact;
    FillContact(&contact, inContact);
    ZJoltCharacterContactSettings settings{ioSettings.mCanPushCharacter,
                                           ioSettings.mCanReceiveImpulses};
    callbacks.on_character_contact_added(callbacks.user, CharacterIdOf(inCharacter),
                                         &contact, &settings);
    ioSettings.mCanPushCharacter = settings.can_push_character;
    ioSettings.mCanReceiveImpulses = settings.can_receive_impulses;
  }

  void OnCharacterContactPersisted(
      const JPH::CharacterVirtual *inCharacter,
      const JPH::CharacterContact &inContact,
      JPH::CharacterContactSettings &ioSettings) override {
    if (callbacks.on_character_contact_persisted == nullptr) return;
    ZJoltCharacterContact contact;
    FillContact(&contact, inContact);
    ZJoltCharacterContactSettings settings{ioSettings.mCanPushCharacter,
                                           ioSettings.mCanReceiveImpulses};
    callbacks.on_character_contact_persisted(
        callbacks.user, CharacterIdOf(inCharacter), &contact, &settings);
    ioSettings.mCanPushCharacter = settings.can_push_character;
    ioSettings.mCanReceiveImpulses = settings.can_receive_impulses;
  }

  void OnCharacterContactRemoved(const JPH::CharacterVirtual *inCharacter,
                                 const JPH::CharacterID &inOtherCharacterID,
                                 const JPH::SubShapeID &inSubShapeID2) override {
    if (callbacks.on_character_contact_removed == nullptr) return;
    callbacks.on_character_contact_removed(
        callbacks.user, CharacterIdOf(inCharacter),
        inOtherCharacterID.GetValue(), zjolt::ToC(inSubShapeID2));
  }

  void OnContactSolve(const JPH::CharacterVirtual *inCharacter,
                      const JPH::BodyID &inBodyID2,
                      const JPH::SubShapeID &inSubShapeID2,
                      JPH::RVec3Arg inContactPosition,
                      JPH::Vec3Arg inContactNormal,
                      JPH::Vec3Arg inContactVelocity,
                      const JPH::PhysicsMaterial *inContactMaterial,
                      JPH::Vec3Arg inCharacterVelocity,
                      JPH::Vec3 &ioNewCharacterVelocity) override {
    if (callbacks.on_contact_solve == nullptr) return;
    ZJoltRVec3 position = zjolt::ToCR(inContactPosition);
    ZJoltVec3 normal = zjolt::ToC(inContactNormal);
    ZJoltVec3 velocity = zjolt::ToC(inContactVelocity);
    ZJoltVec3 character_velocity = zjolt::ToC(inCharacterVelocity);
    ZJoltVec3 new_velocity = zjolt::ToC(ioNewCharacterVelocity);
    callbacks.on_contact_solve(
        callbacks.user, CharacterIdOf(inCharacter), zjolt::ToC(inBodyID2),
        zjolt::ToC(inSubShapeID2), &position, &normal, &velocity,
        zjolt::ToC(inContactMaterial), &character_velocity, &new_velocity);
    ioNewCharacterVelocity = zjolt::ToJolt(new_velocity);
  }

  void OnCharacterContactSolve(const JPH::CharacterVirtual *inCharacter,
                               const JPH::CharacterVirtual *inOtherCharacter,
                               const JPH::SubShapeID &inSubShapeID2,
                               JPH::RVec3Arg inContactPosition,
                               JPH::Vec3Arg inContactNormal,
                               JPH::Vec3Arg inContactVelocity,
                               const JPH::PhysicsMaterial *inContactMaterial,
                               JPH::Vec3Arg inCharacterVelocity,
                               JPH::Vec3 &ioNewCharacterVelocity) override {
    if (callbacks.on_character_contact_solve == nullptr) return;
    ZJoltRVec3 position = zjolt::ToCR(inContactPosition);
    ZJoltVec3 normal = zjolt::ToC(inContactNormal);
    ZJoltVec3 velocity = zjolt::ToC(inContactVelocity);
    ZJoltVec3 character_velocity = zjolt::ToC(inCharacterVelocity);
    ZJoltVec3 new_velocity = zjolt::ToC(ioNewCharacterVelocity);
    callbacks.on_character_contact_solve(
        callbacks.user, CharacterIdOf(inCharacter), CharacterIdOf(inOtherCharacter),
        zjolt::ToC(inSubShapeID2), &position, &normal, &velocity,
        zjolt::ToC(inContactMaterial), &character_velocity, &new_velocity);
    ioNewCharacterVelocity = zjolt::ToJolt(new_velocity);
  }
};

//===----------------------------------------------------------------------===//
// Character-vs-character collision
//
// CharacterVsCharacterCollisionSimple already implements everything; this
// exists only to give the brute-force list its own C++ type name so the
// opaque C handle has something distinct to be.
//===----------------------------------------------------------------------===//

struct ZJoltCharacterVsCharacterCollision final
    : public JPH::CharacterVsCharacterCollisionSimple {};

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

  // The supporting volume is a plane through the character; a contact above
  // it cannot count as ground however flat it is. Jolt's own default is
  // effectively "everything supports", which reports a wall the character is
  // pressed against as the ground it is standing on.
  //
  // The plane goes one inner radius above the shape's lowest point, which for
  // a capsule or a sphere is the centre of its bottom cap — Jolt's own
  // samples use exactly that height, spelled as the standing radius. Placing
  // it at the lowest point instead is the tempting mistake and it breaks
  // every slope: a capsule resting on a ramp touches it on the SIDE of its
  // bottom cap, above the lowest point, so a floor-level plane discards that
  // contact and the character reports itself unsupported on ground it is
  // plainly standing on.
  settings.mSupportingVolume = zjolt::SupportingVolumeFor(
      settings.mShape, settings.mUp);

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
  JPH::TempAllocator *temp = character->owner->temp_allocator;

  if (settings == nullptr) {
    impl->Update(delta_time, zjolt::ToJolt(*gravity), adapters.broad_phase,
                 adapters.object_layer, adapters.body, adapters.shape, *temp);
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
                       adapters.body, adapters.shape, *temp);
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
  const bool changed = impl->SetShape(
      zjolt::ToJolt(shape), max_penetration_depth, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape,
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

//===----------------------------------------------------------------------===//
// CharacterBase, on the virtual character
//===----------------------------------------------------------------------===//

ZJoltCharacterId zjoltCharacterGetId(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  return CharacterIdOf(impl);
}

void zjoltCharacterGetUp(const ZJoltCharacter *character, ZJoltVec3 *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetUp());
}

void zjoltCharacterSetUp(ZJoltCharacter *character, const ZJoltVec3 *up) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || up == nullptr) return;
  impl->SetUp(zjolt::ToJolt(*up));
}

void zjoltCharacterSetMaxSlopeAngle(ZJoltCharacter *character, float radians) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetMaxSlopeAngle(radians);
}

float zjoltCharacterGetCosMaxSlopeAngle(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return 1.0f;
  return impl->GetCosMaxSlopeAngle();
}

bool zjoltCharacterIsSlopeTooSteep(const ZJoltCharacter *character,
                                   const ZJoltVec3 *normal) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || normal == nullptr) return false;
  return impl->IsSlopeTooSteep(zjolt::ToJolt(*normal));
}

const ZJoltPhysicsMaterial *zjoltCharacterGetGroundMaterial(
    const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return nullptr;
  return zjolt::ToC(impl->GetGroundMaterial());
}

ZJoltSubShapeId zjoltCharacterGetGroundSubShapeId(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_SUB_SHAPE_ID_EMPTY;
  return zjolt::ToC(impl->GetGroundSubShapeID());
}

float zjoltCharacterGetMass(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return 0.0f;
  return impl->GetMass();
}

void zjoltCharacterSetMass(ZJoltCharacter *character, float mass) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetMass(mass);
}

float zjoltCharacterGetMaxStrength(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return 0.0f;
  return impl->GetMaxStrength();
}

void zjoltCharacterSetMaxStrength(ZJoltCharacter *character, float max_strength) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetMaxStrength(max_strength);
}

float zjoltCharacterGetPenetrationRecoverySpeed(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return 0.0f;
  return impl->GetPenetrationRecoverySpeed();
}

void zjoltCharacterSetPenetrationRecoverySpeed(ZJoltCharacter *character,
                                               float speed) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetPenetrationRecoverySpeed(speed);
}

bool zjoltCharacterGetEnhancedInternalEdgeRemoval(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return false;
  return impl->GetEnhancedInternalEdgeRemoval();
}

void zjoltCharacterSetEnhancedInternalEdgeRemoval(ZJoltCharacter *character,
                                                  bool apply) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetEnhancedInternalEdgeRemoval(apply);
}

float zjoltCharacterGetCharacterPadding(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return 0.0f;
  return impl->GetCharacterPadding();
}

uint32_t zjoltCharacterGetMaxNumHits(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return 0;
  return impl->GetMaxNumHits();
}

void zjoltCharacterSetMaxNumHits(ZJoltCharacter *character, uint32_t max_hits) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetMaxNumHits(max_hits);
}

float zjoltCharacterGetHitReductionCosMaxAngle(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return -1.0f;
  return impl->GetHitReductionCosMaxAngle();
}

void zjoltCharacterSetHitReductionCosMaxAngle(ZJoltCharacter *character,
                                              float cos_max_angle) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetHitReductionCosMaxAngle(cos_max_angle);
}

bool zjoltCharacterGetMaxHitsExceeded(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return false;
  return impl->GetMaxHitsExceeded();
}

void zjoltCharacterGetShapeOffset(const ZJoltCharacter *character,
                                  ZJoltVec3 *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetShapeOffset());
}

void zjoltCharacterSetShapeOffset(ZJoltCharacter *character,
                                  const ZJoltVec3 *offset) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || offset == nullptr) return;
  impl->SetShapeOffset(zjolt::ToJolt(*offset));
}

uint64_t zjoltCharacterGetUserData(const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return 0;
  return impl->GetUserData();
}

void zjoltCharacterSetUserData(ZJoltCharacter *character, uint64_t user_data) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetUserData(user_data);
}

void zjoltCharacterCancelVelocityTowardsSteepSlopes(
    const ZJoltCharacter *character, const ZJoltVec3 *desired_velocity,
    ZJoltVec3 *out) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || desired_velocity == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out,
                   impl->CancelVelocityTowardsSteepSlopes(zjolt::ToJolt(*desired_velocity)));
}

void zjoltCharacterStartTrackingContactChanges(ZJoltCharacter *character) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->StartTrackingContactChanges();
}

void zjoltCharacterFinishTrackingContactChanges(ZJoltCharacter *character) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->FinishTrackingContactChanges();
}

//===----------------------------------------------------------------------===//
// Stair walking and floor sticking, standalone
//===----------------------------------------------------------------------===//

bool zjoltCharacterCanWalkStairs(const ZJoltCharacter *character,
                                 const ZJoltVec3 *linear_velocity) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || linear_velocity == nullptr) return false;
  return impl->CanWalkStairs(zjolt::ToJolt(*linear_velocity));
}

ZJoltResult zjoltCharacterWalkStairs(
    ZJoltCharacter *character, float delta_time, const ZJoltVec3 *step_up,
    const ZJoltVec3 *step_forward, const ZJoltVec3 *step_forward_test,
    const ZJoltVec3 *step_down_extra, const ZJoltQueryFilters *filters,
    bool *out_stepped) {
  ZJOLT_ENTER(out_stepped);
  JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, step_up, step_forward, step_forward_test,
                      step_down_extra))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  JPH::TempAllocator *temp = character->owner->temp_allocator;
  const bool stepped = impl->WalkStairs(
      delta_time, zjolt::ToJolt(*step_up), zjolt::ToJolt(*step_forward),
      zjolt::ToJolt(*step_forward_test), zjolt::ToJolt(*step_down_extra),
      adapters.broad_phase, adapters.object_layer, adapters.body,
      adapters.shape, *temp);
  if (out_stepped != nullptr) *out_stepped = stepped;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCharacterStickToFloor(ZJoltCharacter *character,
                                       const ZJoltVec3 *step_down,
                                       const ZJoltQueryFilters *filters,
                                       bool *out_stuck) {
  ZJOLT_ENTER(out_stuck);
  JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, step_down)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  JPH::TempAllocator *temp = character->owner->temp_allocator;
  const bool stuck = impl->StickToFloor(zjolt::ToJolt(*step_down),
                                        adapters.broad_phase, adapters.object_layer,
                                        adapters.body, adapters.shape, *temp);
  if (out_stuck != nullptr) *out_stuck = stuck;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCharacterRefreshContacts(ZJoltCharacter *character,
                                          const ZJoltQueryFilters *filters) {
  ZJOLT_ENTER();
  JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  JPH::TempAllocator *temp = character->owner->temp_allocator;
  impl->RefreshContacts(adapters.broad_phase, adapters.object_layer,
                        adapters.body, adapters.shape, *temp);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCharacterGetActiveContacts(const ZJoltCharacter *character,
                                            ZJoltCharacterContact *out_contacts,
                                            uint32_t capacity,
                                            uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::CharacterVirtual::ContactList &contacts = impl->GetActiveContacts();
  const uint32_t count = static_cast<uint32_t>(contacts.size());
  *out_count = count;
  if (out_contacts == nullptr) return ZJOLT_RESULT_OK;

  const uint32_t to_copy = count < capacity ? count : capacity;
  for (uint32_t i = 0; i < to_copy; ++i) FillContact(&out_contacts[i], contacts[i]);
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  return ZJOLT_RESULT_OK;
}

bool zjoltCharacterHasCollidedWithBody(const ZJoltCharacter *character,
                                       ZJoltBodyId body) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return false;
  return impl->HasCollidedWith(zjolt::ToJolt(body));
}

bool zjoltCharacterHasCollidedWithCharacter(const ZJoltCharacter *character,
                                            ZJoltCharacterId other_character_id) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return false;
  return impl->HasCollidedWith(JPH::CharacterID(other_character_id));
}

//===----------------------------------------------------------------------===//
// CharacterContactListener
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCharacterContactListenerCreate(
    const ZJoltCharacterContactListenerCallbacks *callbacks,
    ZJoltCharacterContactListener **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(callbacks, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  ZJoltCharacterContactListener *listener =
      zjolt::New<ZJoltCharacterContactListener>(*callbacks);
  if (listener == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = listener;
  return ZJOLT_RESULT_OK;
}

void zjoltCharacterContactListenerDestroy(ZJoltCharacterContactListener *listener) {
  if (listener == nullptr) return;
  zjolt::Delete(listener);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltCharacterSetListener(ZJoltCharacter *character,
                                      ZJoltCharacterContactListener *listener) {
  ZJOLT_ENTER();
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  impl->SetListener(listener);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Character-vs-character collision
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCharacterVsCharacterCollisionCreate(
    ZJoltCharacterVsCharacterCollision **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  ZJoltCharacterVsCharacterCollision *collision =
      zjolt::New<ZJoltCharacterVsCharacterCollision>();
  if (collision == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = collision;
  return ZJOLT_RESULT_OK;
}

void zjoltCharacterVsCharacterCollisionDestroy(
    ZJoltCharacterVsCharacterCollision *collision) {
  if (collision == nullptr) return;
  zjolt::Delete(collision);
  zjolt::HandleDestroyed();
}

void zjoltCharacterVsCharacterCollisionAdd(
    ZJoltCharacterVsCharacterCollision *collision, ZJoltCharacter *character) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (collision == nullptr || impl == nullptr) return;
  collision->Add(impl);
}

void zjoltCharacterVsCharacterCollisionRemove(
    ZJoltCharacterVsCharacterCollision *collision,
    const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (collision == nullptr || impl == nullptr) return;
  collision->Remove(impl);
}

void zjoltCharacterSetCharacterVsCharacterCollision(
    ZJoltCharacter *character, ZJoltCharacterVsCharacterCollision *collision) {
  JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetCharacterVsCharacterCollision(collision);
}

//===----------------------------------------------------------------------===//
// RigidCharacter
//===----------------------------------------------------------------------===//

void zjoltRigidCharacterDescInit(ZJoltRigidCharacterDesc *desc) {
  if (desc == nullptr) return;

  // Read out of Jolt's own defaults rather than transcribed, so an upstream
  // tuning change moves this with it.
  const JPH::CharacterSettings defaults;

  *desc = ZJoltRigidCharacterDesc{};
  desc->shape = nullptr;
  desc->position = ZJoltRVec3{0, 0, 0};
  desc->rotation = ZJoltQuat{0, 0, 0, 1};
  desc->user_data = 0;
  desc->up = zjolt::ToC(defaults.mUp);
  desc->max_slope_angle = defaults.mMaxSlopeAngle;
  desc->enhanced_internal_edge_removal = defaults.mEnhancedInternalEdgeRemoval;
  desc->layer = static_cast<ZJoltObjectLayer>(defaults.mLayer);
  desc->mass = defaults.mMass;
  desc->friction = defaults.mFriction;
  desc->gravity_factor = defaults.mGravityFactor;
  desc->allowed_dofs = static_cast<uint32_t>(defaults.mAllowedDOFs);
}

ZJoltResult zjoltRigidCharacterCreate(ZJoltPhysicsSystem *system,
                                      const ZJoltRigidCharacterDesc *desc,
                                      ZJoltRigidCharacter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (desc->shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a character needs a shape");
  }

  // Character's constructor creates its rigid body eagerly and has no way to
  // report that failing: BodyInterface::CreateBody returning NULL just leaves
  // the body id at its invalid default (Character.cpp). Destroying a Character
  // in that state is not safe either — ~Character unconditionally calls
  // DestroyBody(mBodyID), and Jolt's own DestroyBodies indexes its body array
  // by the id with no validity check, so an invalid id is an out-of-bounds
  // access rather than a no-op. Checking room for one more body up front, and
  // never destroying a Character that came out without one, is how this binds
  // that safely without touching Jolt.
  if (system->system.GetNumBodies() >= system->system.GetMaxBodies()) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "the system is already holding max_bodies bodies");
  }

  JPH::CharacterSettings settings;
  settings.mShape = zjolt::ToJolt(desc->shape);
  settings.mUp = zjolt::ToJolt(desc->up);
  settings.mMaxSlopeAngle = desc->max_slope_angle;
  settings.mEnhancedInternalEdgeRemoval = desc->enhanced_internal_edge_removal;
  settings.mLayer = static_cast<JPH::ObjectLayer>(desc->layer);
  settings.mMass = desc->mass;
  settings.mFriction = desc->friction;
  settings.mGravityFactor = desc->gravity_factor;
  settings.mAllowedDOFs = static_cast<JPH::EAllowedDOFs>(desc->allowed_dofs);

  // Mirrors zjoltCharacterCreate's override of the same field, for the same
  // reason and by the same rule — see the comment there.
  settings.mSupportingVolume = zjolt::SupportingVolumeFor(
      settings.mShape, settings.mUp);

  ZJoltRigidCharacter *handle = zjolt::New<ZJoltRigidCharacter>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  JPH::Character *character = zjolt::New<JPH::Character>(
      &settings, zjolt::ToJoltR(desc->position),
      zjolt::ToJoltRotation(desc->rotation), desc->user_data, &system->system);
  if (character == nullptr) {
    zjolt::Delete(handle);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }

  if (character->GetBodyID().IsInvalid()) {
    // The pre-flight check above should make this unreachable outside a race
    // with another thread creating bodies concurrently. Deliberately leaking
    // `character` rather than deleting it: its destructor would call
    // DestroyBody on the invalid id, which is the out-of-bounds access this
    // whole function exists to avoid. A leaked Character on the rarest of
    // failure paths is a far better outcome than memory corruption.
    zjolt::Delete(handle);
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "the system is already holding max_bodies bodies");
  }

  handle->impl = character;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltRigidCharacterDestroy(ZJoltRigidCharacter *character) {
  if (character == nullptr) return;
  // Dropping the Ref runs Character's destructor, which destroys the body.
  character->impl = nullptr;
  zjolt::Delete(character);
  zjolt::HandleDestroyed();
}

void zjoltRigidCharacterAddToPhysicsSystem(ZJoltRigidCharacter *character,
                                           ZJoltActivation activation) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr) return;
  impl->AddToPhysicsSystem(ToJoltActivation(activation));
}

void zjoltRigidCharacterRemoveFromPhysicsSystem(ZJoltRigidCharacter *character) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr) return;
  impl->RemoveFromPhysicsSystem();
}

void zjoltRigidCharacterActivate(ZJoltRigidCharacter *character) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr) return;
  impl->Activate();
}

void zjoltRigidCharacterPostSimulation(ZJoltRigidCharacter *character,
                                       float max_separation_distance) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr) return;
  impl->PostSimulation(max_separation_distance);
}

void zjoltRigidCharacterSetLinearAndAngularVelocity(
    ZJoltRigidCharacter *character, const ZJoltVec3 *linear_velocity,
    const ZJoltVec3 *angular_velocity) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || linear_velocity == nullptr || angular_velocity == nullptr)
    return;
  impl->SetLinearAndAngularVelocity(zjolt::ToJolt(*linear_velocity),
                                    zjolt::ToJolt(*angular_velocity));
}

void zjoltRigidCharacterGetLinearVelocity(const ZJoltRigidCharacter *character,
                                          ZJoltVec3 *out) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetLinearVelocity());
}

void zjoltRigidCharacterSetLinearVelocity(ZJoltRigidCharacter *character,
                                          const ZJoltVec3 *velocity) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || velocity == nullptr) return;
  impl->SetLinearVelocity(zjolt::ToJolt(*velocity));
}

void zjoltRigidCharacterAddLinearVelocity(ZJoltRigidCharacter *character,
                                          const ZJoltVec3 *velocity) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || velocity == nullptr) return;
  impl->AddLinearVelocity(zjolt::ToJolt(*velocity));
}

void zjoltRigidCharacterAddImpulse(ZJoltRigidCharacter *character,
                                   const ZJoltVec3 *impulse) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || impulse == nullptr) return;
  impl->AddImpulse(zjolt::ToJolt(*impulse));
}

ZJoltBodyId zjoltRigidCharacterGetBodyId(const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_BODY_ID_INVALID;
  return zjolt::ToC(impl->GetBodyID());
}

void zjoltRigidCharacterGetPositionAndRotation(
    const ZJoltRigidCharacter *character, ZJoltRVec3 *out_position,
    ZJoltQuat *out_rotation) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return;
  JPH::RVec3 position;
  JPH::Quat rotation;
  impl->GetPositionAndRotation(position, rotation);
  zjolt::WriteRVec3(out_position, position);
  zjolt::WriteQuat(out_rotation, rotation);
}

void zjoltRigidCharacterSetPositionAndRotation(ZJoltRigidCharacter *character,
                                               const ZJoltRVec3 *position,
                                               const ZJoltQuat *rotation,
                                               ZJoltActivation activation) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || position == nullptr || rotation == nullptr) return;
  impl->SetPositionAndRotation(zjolt::ToJoltR(*position),
                               zjolt::ToJoltRotation(*rotation),
                               ToJoltActivation(activation));
}

void zjoltRigidCharacterGetPosition(const ZJoltRigidCharacter *character,
                                    ZJoltRVec3 *out) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteRVec3(out, impl->GetPosition());
}

void zjoltRigidCharacterSetPosition(ZJoltRigidCharacter *character,
                                    const ZJoltRVec3 *position,
                                    ZJoltActivation activation) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || position == nullptr) return;
  impl->SetPosition(zjolt::ToJoltR(*position), ToJoltActivation(activation));
}

void zjoltRigidCharacterGetRotation(const ZJoltRigidCharacter *character,
                                    ZJoltQuat *out) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteQuat(out, impl->GetRotation());
}

void zjoltRigidCharacterSetRotation(ZJoltRigidCharacter *character,
                                    const ZJoltQuat *rotation,
                                    ZJoltActivation activation) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || rotation == nullptr) return;
  impl->SetRotation(zjolt::ToJoltRotation(*rotation), ToJoltActivation(activation));
}

void zjoltRigidCharacterGetCenterOfMassPosition(
    const ZJoltRigidCharacter *character, ZJoltRVec3 *out) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteRVec3(out, impl->GetCenterOfMassPosition());
}

ZJoltObjectLayer zjoltRigidCharacterGetLayer(const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return 0;
  return static_cast<ZJoltObjectLayer>(impl->GetLayer());
}

void zjoltRigidCharacterSetLayer(ZJoltRigidCharacter *character,
                                 ZJoltObjectLayer layer) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetLayer(static_cast<JPH::ObjectLayer>(layer));
}

ZJoltResult zjoltRigidCharacterSetShape(ZJoltRigidCharacter *character,
                                        const ZJoltShape *shape,
                                        float max_penetration_depth,
                                        bool *out_changed) {
  ZJOLT_ENTER(out_changed);
  JPH::Character *impl = Impl(character);
  if (!zjolt::Present(impl, shape)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // A refused shape change is a normal outcome — standing up under a low
  // ceiling — so it is reported through out_changed rather than as an error.
  const bool changed = impl->SetShape(zjolt::ToJolt(shape), max_penetration_depth);
  if (out_changed != nullptr) *out_changed = changed;
  return ZJOLT_RESULT_OK;
}

const ZJoltShape *zjoltRigidCharacterGetShape(const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return nullptr;
  return zjolt::ToC(impl->GetShape());
}

ZJoltCharacterId zjoltRigidCharacterGetId(const ZJoltRigidCharacter *character) {
  // Character (unlike CharacterVirtual) has no GetID of its own to read; there
  // is nothing wrong here, RigidCharacter simply has no CharacterID to report.
  (void)character;
  return ZJOLT_CHARACTER_ID_INVALID;
}

void zjoltRigidCharacterGetUp(const ZJoltRigidCharacter *character,
                              ZJoltVec3 *out) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetUp());
}

void zjoltRigidCharacterSetUp(ZJoltRigidCharacter *character,
                              const ZJoltVec3 *up) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || up == nullptr) return;
  impl->SetUp(zjolt::ToJolt(*up));
}

void zjoltRigidCharacterSetMaxSlopeAngle(ZJoltRigidCharacter *character,
                                         float radians) {
  JPH::Character *impl = Impl(character);
  if (impl == nullptr) return;
  impl->SetMaxSlopeAngle(radians);
}

float zjoltRigidCharacterGetCosMaxSlopeAngle(const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return 1.0f;
  return impl->GetCosMaxSlopeAngle();
}

bool zjoltRigidCharacterIsSlopeTooSteep(const ZJoltRigidCharacter *character,
                                        const ZJoltVec3 *normal) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || normal == nullptr) return false;
  return impl->IsSlopeTooSteep(zjolt::ToJolt(*normal));
}

ZJoltGroundState zjoltRigidCharacterGetGroundState(
    const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_GROUND_STATE_IN_AIR;
  return ToCGroundState(impl->GetGroundState());
}

bool zjoltRigidCharacterIsSupported(const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return false;
  return impl->IsSupported();
}

void zjoltRigidCharacterGetGroundPosition(const ZJoltRigidCharacter *character,
                                          ZJoltRVec3 *out) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteRVec3(out, impl->GetGroundPosition());
}

void zjoltRigidCharacterGetGroundNormal(const ZJoltRigidCharacter *character,
                                        ZJoltVec3 *out) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetGroundNormal());
}

void zjoltRigidCharacterGetGroundVelocity(const ZJoltRigidCharacter *character,
                                          ZJoltVec3 *out) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr || out == nullptr) return;
  zjolt::WriteVec3(out, impl->GetGroundVelocity());
}

const ZJoltPhysicsMaterial *zjoltRigidCharacterGetGroundMaterial(
    const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return nullptr;
  return zjolt::ToC(impl->GetGroundMaterial());
}

ZJoltBodyId zjoltRigidCharacterGetGroundBodyId(
    const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_BODY_ID_INVALID;
  return zjolt::ToC(impl->GetGroundBodyID());
}

ZJoltSubShapeId zjoltRigidCharacterGetGroundSubShapeId(
    const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return ZJOLT_SUB_SHAPE_ID_EMPTY;
  return zjolt::ToC(impl->GetGroundSubShapeID());
}

uint64_t zjoltRigidCharacterGetGroundUserData(
    const ZJoltRigidCharacter *character) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) return 0;
  return impl->GetGroundUserData();
}

}  // extern "C"
