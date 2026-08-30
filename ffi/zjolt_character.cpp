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
#include "zjolt_transformed.h"

#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

namespace {

/// Drops `handle` from `list`, which may be null when the system that held
/// the list is already gone. Order does not matter, so the last element takes
/// the removed one's place rather than shifting the tail.
template <typename T>
void Unregister(JPH::Array<T *> *list, T *handle) {
  if (list == nullptr) return;
  for (size_t i = 0; i < list->size(); ++i) {
    if ((*list)[i] == handle) {
      (*list)[i] = list->back();
      list->pop_back();
      return;
    }
  }
}

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

// Takes the raw integer, not the enum — see zjolt::RawEnum in
// zjolt_internal.h. Every entry point below converts once, at the boundary,
// before the value ever reaches here.
JPH::EActivation ToJoltActivation(int32_t activation) {
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

/// Hands a JPH::TransformedShape out as a zjolt_transformed.h handle.
///
/// Goes through the public entry point rather than reaching into that file's
/// struct: the concrete type is declared only in zjolt_transformed.cpp on
/// purpose, and every field this needs to set is a parameter of Create.
ZJoltResult OwnTransformedShape(const JPH::TransformedShape &ts,
                                ZJoltTransformedShape **out) {
  if (ts.mShape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this character has no shape to hand out");
  }
  // Create takes the CENTRE OF MASS position, which is what mShapePositionCOM
  // is — not the character's position, which sits an offset away from it.
  const ZJoltRVec3 position = zjolt::ToCR(ts.mShapePositionCOM);
  const ZJoltQuat rotation = zjolt::ToC(ts.mShapeRotation);
  const ZJoltVec3 scale = zjolt::ToC(ts.GetShapeScale());
  return zjoltTransformedShapeCreate(zjolt::ToC(ts.mShape.GetPtr()), &position,
                                     &rotation, &scale, zjolt::ToC(ts.mBodyID),
                                     out);
}

/// Collects zjoltCharacterCheckCollision's hits.
///
/// Two facts arrive via the base class, not AddHit: CONTEXT is the hit's
/// TransformedShape, the only place to read material, valid only for the call.
/// USER DATA is the other CharacterVirtual, the only way to tell a
/// character-vs-character hit apart, since virtual characters carry no body id.
class CharacterHitCollector final : public JPH::CollideShapeCollector {
 public:
  CharacterHitCollector(ZJoltCharacterCollisionHit *out, uint32_t capacity)
      : out_(out), capacity_(capacity) {}

  void SetUserData(JPH::uint64 user_data) override {
    other_character_ =
        reinterpret_cast<const JPH::CharacterVirtual *>(user_data);
  }

  void AddHit(const JPH::CollideShapeResult &result) override {
    // Counting past the capacity is deliberate: the count is the answer to
    // the size query and has to be true even when the buffer could not hold
    // the hits, which is what keeps this to one traversal instead of two.
    if (out_ != nullptr && count_ < capacity_) {
      ZJoltCharacterCollisionHit &hit = out_[count_];
      hit = ZJoltCharacterCollisionHit{};
      hit.body = zjolt::ToC(result.mBodyID2);
      hit.character_id = CharacterIdOf(other_character_);
      hit.sub_shape_id = zjolt::ToC(result.mSubShapeID2);
      hit.contact_point_on_1 = zjolt::ToC(result.mContactPointOn1);
      hit.contact_point_on_2 = zjolt::ToC(result.mContactPointOn2);
      hit.penetration_axis = zjolt::ToC(result.mPenetrationAxis);
      hit.penetration_depth = result.mPenetrationDepth;
      const JPH::TransformedShape *context = GetContext();
      hit.material =
          context != nullptr
              ? zjolt::ToC(context->GetMaterial(result.mSubShapeID2))
              : nullptr;
    }
    ++count_;
  }

  uint32_t count() const { return count_; }

 private:
  ZJoltCharacterCollisionHit *out_;
  uint32_t capacity_;
  uint32_t count_ = 0;
  const JPH::CharacterVirtual *other_character_ = nullptr;
};

/// The tail of a count-then-fill entry point: report the true count, and say
/// so when the buffer could not hold it.
ZJoltResult ReportHitCount(uint32_t count,
                           const ZJoltCharacterCollisionHit *out_hits,
                           uint32_t capacity, uint32_t *out_count) {
  *out_count = count;
  if (out_hits == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  return ZJOLT_RESULT_OK;
}

/// A supporting-volume plane with no side is a character that never reports
/// ground again, and the symptom shows up frames later as "it will not walk".
ZJoltResult MakeSupportingVolume(const ZJoltVec3 *normal, float distance,
                                 JPH::Plane *out) {
  const JPH::Vec3 direction = zjolt::ToJolt(*normal);
  if (direction.IsNearZero()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "normal is zero-length: a supporting volume plane needs a direction");
  }
  // Stored as given, NOT normalised: `distance` is measured in units of
  // `normal`'s length, so scaling one without the other would move the plane
  // rather than tidy it. Only the sign of the signed distance is read.
  *out = JPH::Plane(direction, distance);
  return ZJOLT_RESULT_OK;
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
// Forwards Jolt's virtual callbacks to plain C function pointers. A field left NULL behaves exactly as Jolt's own default override: accept every contact, change nothing.
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
// The opaque handle is JPH::CharacterVsCharacterCollision itself (the interface, not a fixed implementation), so the built-in brute-force list and a host's own callbacks come out of Create as the same C type.
// Add/Remove are declared here, virtual (not part of Jolt's own interface), so the two concrete types below can give them their own meaning without a cast.
//===----------------------------------------------------------------------===//

struct ZJoltCharacterVsCharacterCollision : public JPH::CharacterVsCharacterCollision {
  virtual void Add(JPH::CharacterVirtual *character) { (void)character; }
  virtual void Remove(const JPH::CharacterVirtual *character) { (void)character; }
};

/// The brute-force list. Composes JPH::CharacterVsCharacterCollisionSimple
/// rather than inheriting it: that type already derives from
/// JPH::CharacterVsCharacterCollision on its own path, and giving it
/// ZJoltCharacterVsCharacterCollision as a second base to the same ancestor
/// would need virtual inheritance Jolt's own class does not use.
struct ZJoltCharacterVsCharacterCollisionSimple final
    : public ZJoltCharacterVsCharacterCollision {
  JPH::CharacterVsCharacterCollisionSimple list;

  void Add(JPH::CharacterVirtual *character) override { list.Add(character); }
  void Remove(const JPH::CharacterVirtual *character) override { list.Remove(character); }

  void CollideCharacter(const JPH::CharacterVirtual *inCharacter,
                        JPH::RMat44Arg inCenterOfMassTransform,
                        const JPH::CollideShapeSettings &inCollideShapeSettings,
                        JPH::RVec3Arg inBaseOffset,
                        JPH::CollideShapeCollector &ioCollector) const override {
    list.CollideCharacter(inCharacter, inCenterOfMassTransform,
                          inCollideShapeSettings, inBaseOffset, ioCollector);
  }

  void CastCharacter(const JPH::CharacterVirtual *inCharacter,
                     JPH::RMat44Arg inCenterOfMassTransform,
                     JPH::Vec3Arg inDirection,
                     const JPH::ShapeCastSettings &inShapeCastSettings,
                     JPH::RVec3Arg inBaseOffset,
                     JPH::CastShapeCollector &ioCollector) const override {
    list.CastCharacter(inCharacter, inCenterOfMassTransform, inDirection,
                       inShapeCastSettings, inBaseOffset, ioCollector);
  }
};

//===----------------------------------------------------------------------===//
// A custom character-vs-character broad phase
//
// ZJoltCharacterVsCharacterCollisionCustom asks the host's callback for candidates via a VISITOR, then runs exactly the narrow-phase test CharacterVsCharacterCollisionSimple runs against each accepted one — deciding nothing about HOW two characters collide, only WHO is offered.
// The `ZJoltCharacterVsCharacterVisitFn` trampoline's context travels through `visit_user`, a stack struct built fresh per CollideCharacter/CastCharacter call.
//===----------------------------------------------------------------------===//

namespace {

struct CollideVisitContext {
  const JPH::CharacterVirtual *self;
  JPH::Mat44 transform1;
  const JPH::Shape *shape1;
  JPH::AABox bounds1;
  const JPH::CollideShapeSettings *original_settings;
  JPH::CollideShapeSettings settings;
  JPH::RVec3 base_offset;
  JPH::CollideShapeCollector *collector;
};

/// Mirrors the body of CharacterVsCharacterCollisionSimple::CollideCharacter
/// exactly, one candidate at a time instead of one iteration of `for (c :
/// mCharacters)`.
bool VisitCollideCandidate(void *raw, ZJoltCharacter *candidate) {
  auto *ctx = static_cast<CollideVisitContext *>(raw);
  if (ctx->collector->ShouldEarlyOut()) return false;

  const JPH::CharacterVirtual *other = Impl(candidate);
  if (other == nullptr || other == ctx->self) return true;

  // Make shape 2 relative to the base offset, same as shape 1 already is.
  const JPH::Mat44 transform2 =
      other->GetCenterOfMassTransform().PostTranslated(-ctx->base_offset).ToMat44();

  // Adds character 2's padding so collision with its outer shell is detected.
  ctx->settings.mMaxSeparationDistance =
      ctx->original_settings->mMaxSeparationDistance + other->GetCharacterPadding();

  // Check if the bounding boxes of the characters overlap.
  const JPH::Shape *shape2 = other->GetShape();
  JPH::AABox bounds2 = shape2->GetWorldSpaceBounds(transform2, JPH::Vec3::sOne());
  bounds2.ExpandBy(JPH::Vec3::sReplicate(ctx->settings.mMaxSeparationDistance));
  if (!ctx->bounds1.Overlaps(bounds2)) return true;

  // The collector needs to know which character this hit is against.
  ctx->collector->SetUserData(reinterpret_cast<JPH::uint64>(other));

  // The query below uses the character's shape with no padding;
  // CharacterVirtual::GetContactsAtPosition corrects for that afterwards, the
  // same as it does for the built-in list.
  JPH::CollisionDispatch::sCollideShapeVsShape(
      ctx->shape1, shape2, JPH::Vec3::sOne(), JPH::Vec3::sOne(), ctx->transform1,
      transform2, JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(),
      ctx->settings, *ctx->collector);

  return !ctx->collector->ShouldEarlyOut();
}

struct CastVisitContext {
  const JPH::CharacterVirtual *self;
  const JPH::ShapeCast *shape_cast;
  JPH::Vec3 origin;
  JPH::Vec3 extents;
  const JPH::ShapeCastSettings *original_settings;
  JPH::ShapeCastSettings settings;
  JPH::Vec3 direction;
  JPH::RVec3 base_offset;
  JPH::CastShapeCollector *collector;
};

/// Mirrors CharacterVsCharacterCollisionSimple::CastCharacter, one candidate
/// at a time. @see VisitCollideCandidate.
bool VisitCastCandidate(void *raw, ZJoltCharacter *candidate) {
  auto *ctx = static_cast<CastVisitContext *>(raw);
  if (ctx->collector->ShouldEarlyOut()) return false;

  const JPH::CharacterVirtual *other = Impl(candidate);
  if (other == nullptr || other == ctx->self) return true;

  const JPH::Mat44 transform2 =
      other->GetCenterOfMassTransform().PostTranslated(-ctx->base_offset).ToMat44();

  // Adds character 2's padding so collision with its outer shell is detected.
  ctx->settings.mExtraConvexRadius =
      ctx->original_settings->mExtraConvexRadius + other->GetCharacterPadding();

  // Sweep the bounding box of the character against the bounding box of the
  // other character to see if they can collide.
  const JPH::Shape *shape2 = other->GetShape();
  JPH::AABox bounds2 = shape2->GetWorldSpaceBounds(transform2, JPH::Vec3::sOne());
  bounds2.ExpandBy(ctx->extents + JPH::Vec3::sReplicate(other->GetCharacterPadding()));
  if (!JPH::RayAABoxHits(ctx->origin, ctx->direction, bounds2.mMin, bounds2.mMax)) {
    return true;
  }

  ctx->collector->SetUserData(reinterpret_cast<JPH::uint64>(other));

  // As above, collides against the character's shape without padding;
  // CharacterVirtual::ValidateMovement corrects for that afterwards.
  JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(
      *ctx->shape_cast, ctx->settings, shape2, JPH::Vec3::sOne(), {}, transform2,
      JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(), *ctx->collector);

  return !ctx->collector->ShouldEarlyOut();
}

}  // namespace

struct ZJoltCharacterVsCharacterCollisionCustom final
    : public ZJoltCharacterVsCharacterCollision {
  ZJoltCharacterVsCharacterCollisionCallbacks callbacks;

  void CollideCharacter(const JPH::CharacterVirtual *inCharacter,
                        JPH::RMat44Arg inCenterOfMassTransform,
                        const JPH::CollideShapeSettings &inCollideShapeSettings,
                        JPH::RVec3Arg inBaseOffset,
                        JPH::CollideShapeCollector &ioCollector) const override {
    if (callbacks.collide_character == nullptr) return;

    const JPH::Mat44 transform1 =
        inCenterOfMassTransform.PostTranslated(-inBaseOffset).ToMat44();
    const JPH::Shape *shape1 = inCharacter->GetShape();
    const JPH::AABox bounds1 = shape1->GetWorldSpaceBounds(transform1, JPH::Vec3::sOne());

    CollideVisitContext ctx{};
    ctx.self = inCharacter;
    ctx.transform1 = transform1;
    ctx.shape1 = shape1;
    ctx.bounds1 = bounds1;
    ctx.original_settings = &inCollideShapeSettings;
    ctx.settings = inCollideShapeSettings;
    ctx.base_offset = inBaseOffset;
    ctx.collector = &ioCollector;

    const ZJoltRMat44 transform_c = zjolt::ToCR(inCenterOfMassTransform);
    callbacks.collide_character(callbacks.user, CharacterIdOf(inCharacter),
                                &transform_c, &VisitCollideCandidate, &ctx);
    ioCollector.SetUserData(0);
  }

  void CastCharacter(const JPH::CharacterVirtual *inCharacter,
                     JPH::RMat44Arg inCenterOfMassTransform,
                     JPH::Vec3Arg inDirection,
                     const JPH::ShapeCastSettings &inShapeCastSettings,
                     JPH::RVec3Arg inBaseOffset,
                     JPH::CastShapeCollector &ioCollector) const override {
    if (callbacks.cast_character == nullptr) return;

    const JPH::Mat44 transform1 =
        inCenterOfMassTransform.PostTranslated(-inBaseOffset).ToMat44();
    const JPH::ShapeCast shape_cast(inCharacter->GetShape(), JPH::Vec3::sOne(),
                                    transform1, inDirection);
    const JPH::Vec3 origin = shape_cast.mShapeWorldBounds.GetCenter();
    const JPH::Vec3 extents = shape_cast.mShapeWorldBounds.GetExtent() +
                              JPH::Vec3::sReplicate(inShapeCastSettings.mExtraConvexRadius);

    CastVisitContext ctx{};
    ctx.self = inCharacter;
    ctx.shape_cast = &shape_cast;
    ctx.origin = origin;
    ctx.extents = extents;
    ctx.original_settings = &inShapeCastSettings;
    ctx.settings = inShapeCastSettings;
    ctx.direction = inDirection;
    ctx.base_offset = inBaseOffset;
    ctx.collector = &ioCollector;

    const ZJoltRMat44 transform_c = zjolt::ToCR(inCenterOfMassTransform);
    const ZJoltVec3 direction_c = zjolt::ToC(inDirection);
    callbacks.cast_character(callbacks.user, CharacterIdOf(inCharacter),
                             &transform_c, &direction_c, &VisitCastCandidate, &ctx);
    ioCollector.SetUserData(0);
  }
};

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
  desc->min_time_remaining = defaults.mMinTimeRemaining;
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
  desc->inner_body_id_override = ZJOLT_BODY_ID_INVALID;
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
  settings.mMinTimeRemaining = desc->min_time_remaining;
  settings.mCollisionTolerance = desc->collision_tolerance;
  settings.mHitReductionCosMaxAngle = desc->hit_reduction_cos_max_angle;
  settings.mMaxCollisionIterations = desc->max_collision_iterations;
  settings.mMaxConstraintIterations = desc->max_constraint_iterations;
  settings.mMaxNumHits = desc->max_num_hits;
  settings.mBackFaceMode =
      zjolt::RawEnum(desc->back_face_mode) == ZJOLT_BACK_FACE_MODE_IGNORE
          ? JPH::EBackFaceMode::IgnoreBackFaces
          : JPH::EBackFaceMode::CollideWithBackFaces;
  settings.mEnhancedInternalEdgeRemoval = desc->enhanced_internal_edge_removal;
  if (desc->inner_body_shape != nullptr) {
    settings.mInnerBodyShape = zjolt::ToJolt(desc->inner_body_shape);
    if (desc->inner_body_id_override != ZJOLT_BODY_ID_INVALID) {
      settings.mInnerBodyIDOverride = zjolt::ToJolt(desc->inner_body_id_override);
    }
    settings.mInnerBodyLayer =
        static_cast<JPH::ObjectLayer>(desc->inner_body_layer);
  }

  // The supporting volume is a plane through the character; a contact above it
  // cannot count as ground. Jolt's default is effectively "everything
  // supports", reporting a pressed-against wall as ground. Placed one inner
  // radius above the shape's lowest point (a capsule/ sphere's standing radius)
  // rather than AT the lowest point: a floor-level plane discards a ramp's
  // side-of-cap contact and reports unsupported on ground plainly stood on.
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
  system->characters.push_back(handle);
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltCharacterDestroy(ZJoltCharacter *character) {
  if (character == nullptr) return;
  Unregister(character->owner == nullptr ? nullptr : &character->owner->characters,
             character);
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

ZJoltResult zjoltCharacterGetAdjustedBodyVelocity(
    const ZJoltCharacter *character, ZJoltBodyId body_b,
    ZJoltVec3 *out_linear_velocity, ZJoltVec3 *out_angular_velocity) {
  ZJOLT_ENTER(out_linear_velocity, out_angular_velocity);
  const JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::BodyLockRead lock(character->owner->system.GetBodyLockInterface(),
                         zjolt::ToJolt(body_b));
  if (!lock.Succeeded()) {
    return zjolt::SetError(
        ZJOLT_RESULT_BODY_NOT_FOUND,
        "body_b does not name a live body in this character's system");
  }
  const JPH::Body &jolt_body = lock.GetBody();

  JPH::Vec3 linear = JPH::Vec3::sZero();
  JPH::Vec3 angular = JPH::Vec3::sZero();
  if (!jolt_body.IsStatic()) {
    const JPH::MotionProperties *mp = jolt_body.GetMotionPropertiesUnchecked();
    linear = mp->GetLinearVelocity();
    angular = mp->GetAngularVelocity();
  }
  // The exact two steps CharacterVirtual::GetAdjustedBodyVelocity itself
  // takes: the body's real velocity, then the installed listener's own
  // override -- both public, unlike the private method that combines them.
  JPH::CharacterContactListener *listener = impl->GetListener();
  if (listener != nullptr) listener->OnAdjustBodyVelocity(impl, jolt_body, linear, angular);

  zjolt::WriteVec3(out_linear_velocity, linear);
  zjolt::WriteVec3(out_angular_velocity, angular);
  return ZJOLT_RESULT_OK;
}

void zjoltCharacterCalculateGroundVelocity(const ZJoltCharacter *character,
                                           const ZJoltRVec3 *center_of_mass,
                                           const ZJoltVec3 *linear_velocity,
                                           const ZJoltVec3 *angular_velocity,
                                           float delta_time, ZJoltVec3 *out) {
  if (out == nullptr) return;
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr || center_of_mass == nullptr ||
      linear_velocity == nullptr || angular_velocity == nullptr) {
    *out = ZJoltVec3{0, 0, 0};
    return;
  }

  // Literal port of CharacterVirtual::CalculateCharacterGroundVelocity,
  // reading only the character's own (public) current position.
  const JPH::Vec3 in_linear = zjolt::ToJolt(*linear_velocity);
  const JPH::Vec3 in_angular = zjolt::ToJolt(*angular_velocity);
  const float angular_len_sq = in_angular.LengthSq();
  if (angular_len_sq < 1.0e-12f) {
    zjolt::WriteVec3(out, in_linear);
    return;
  }
  const float angular_len = std::sqrt(angular_len_sq);
  const JPH::Quat rotation =
      JPH::Quat::sRotation(in_angular / angular_len, angular_len * delta_time);
  const JPH::RVec3 com = zjolt::ToJoltR(*center_of_mass);
  const JPH::RVec3 character_position = impl->GetPosition();
  const JPH::RVec3 new_position =
      com + rotation * JPH::Vec3(character_position - com);
  zjolt::WriteVec3(out, in_linear + JPH::Vec3(new_position - character_position) /
                            delta_time);
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

  // A refused shape change is a normal outcome (e.g. standing under a low
  // ceiling), reported through out_changed, not as an error.
  //
  // The inner body, if any, must follow: SetInnerBodyShape does not check
  // for its absence and would reach BodyInterface::SetShape with an invalid
  // id — harmless, but not something to rely on.
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

ZJoltResult zjoltCharacterSetInnerBodyShape(ZJoltCharacter *character,
                                            const ZJoltShape *shape) {
  ZJOLT_ENTER();
  JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, shape)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Jolt's SetInnerBodyShape does not check first; it would call
  // BodyInterface::SetShape with an invalid id, whose body lock simply fails
  // and returns. Silently doing nothing is the wrong answer to "make the
  // inner body this shape" when there is no inner body to make.
  if (impl->GetInnerBodyID().IsInvalid()) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "this character has no inner body: it was created with a NULL "
        "inner_body_shape, and one cannot be added afterwards");
  }

  impl->SetInnerBodyShape(zjolt::ToJolt(shape));
  return ZJOLT_RESULT_OK;
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

void zjoltCharacterGetSupportingVolume(const ZJoltCharacter *character,
                                       ZJoltVec3 *out_normal,
                                       float *out_distance) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) {
    if (out_normal != nullptr) *out_normal = ZJoltVec3{0, 0, 0};
    if (out_distance != nullptr) *out_distance = 0.0f;
    return;
  }
  const JPH::Plane &plane = impl->GetSupportingVolume();
  if (out_normal != nullptr) zjolt::WriteVec3(out_normal, plane.GetNormal());
  if (out_distance != nullptr) *out_distance = plane.GetConstant();
}

ZJoltResult zjoltCharacterSetSupportingVolume(ZJoltCharacter *character,
                                              const ZJoltVec3 *normal,
                                              float distance) {
  ZJOLT_ENTER();
  JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, normal)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Plane plane;
  const ZJoltResult built = MakeSupportingVolume(normal, distance, &plane);
  if (built != ZJOLT_RESULT_OK) return built;
  impl->SetSupportingVolume(plane);
  return ZJOLT_RESULT_OK;
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
// Asking about a placement the character is not at
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCharacterGetTransformedShape(const ZJoltCharacter *character,
                                              ZJoltTransformedShape **out) {
  ZJOLT_ENTER(out);
  const JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return OwnTransformedShape(impl->GetTransformedShape(), out);
}

ZJoltResult zjoltCharacterCheckCollision(
    const ZJoltCharacter *character, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *movement_direction,
    float max_separation_distance, const ZJoltShape *shape,
    const ZJoltQueryFilters *filters, ZJoltCharacterCollisionHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const JPH::CharacterVirtual *impl = Impl(character);
  if (!zjolt::Present(impl, position, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Shape *test_shape =
      shape != nullptr ? zjolt::ToJolt(shape) : impl->GetShape();
  if (test_shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is NULL and the character has none either");
  }

  const JPH::Quat test_rotation = rotation != nullptr
                                      ? zjolt::ToJoltRotation(*rotation)
                                      : impl->GetRotation();
  const JPH::Vec3 direction = movement_direction != nullptr
                                  ? zjolt::ToJolt(*movement_direction)
                                  : JPH::Vec3::sZero();

  zjolt::QueryFilters adapters(filters);
  CharacterHitCollector collector(out_hits, capacity);
  // Hits come back relative to `position` rather than the world origin, the
  // same base offset every query in zjolt_query.h uses: floats are most
  // accurate near zero, and a character far from the origin loses that
  // precision on every contact point otherwise.
  impl->CheckCollision(zjolt::ToJoltR(*position), test_rotation, direction,
                       max_separation_distance, test_shape,
                       zjolt::ToJoltR(*position), collector,
                       adapters.broad_phase, adapters.object_layer,
                       adapters.body, adapters.shape);

  return ReportHitCount(collector.count(), out_hits, capacity, out_count);
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

ZJoltCharacterContactListener *zjoltCharacterGetListener(
    const ZJoltCharacter *character) {
  const JPH::CharacterVirtual *impl = Impl(character);
  if (impl == nullptr) return nullptr;
  // The downcast is sound because zjoltCharacterSetListener is the only way
  // to install one, and it only ever installs a ZJoltCharacterContactListener.
  return static_cast<ZJoltCharacterContactListener *>(impl->GetListener());
}

//===----------------------------------------------------------------------===//
// Character-vs-character collision
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCharacterVsCharacterCollisionCreate(
    ZJoltCharacterVsCharacterCollision **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  ZJoltCharacterVsCharacterCollisionSimple *collision =
      zjolt::New<ZJoltCharacterVsCharacterCollisionSimple>();
  if (collision == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  zjolt::HandleCreated();
  *out = collision;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCharacterVsCharacterCollisionCreateCustom(
    const ZJoltCharacterVsCharacterCollisionCallbacks *callbacks,
    ZJoltCharacterVsCharacterCollision **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(callbacks, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  ZJoltCharacterVsCharacterCollisionCustom *collision =
      zjolt::New<ZJoltCharacterVsCharacterCollisionCustom>();
  if (collision == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  collision->callbacks = *callbacks;

  zjolt::HandleCreated();
  *out = collision;
  return ZJOLT_RESULT_OK;
}

/// Frees whichever concrete type Create or CreateCustom handed out. The
/// explicit destructor call inside zjolt::Delete resolves virtually — both
/// concrete types share ZJoltCharacterVsCharacterCollision's base, whose own
/// base (JPH::CharacterVsCharacterCollision) declares one — so the right
/// destructor body runs either way.
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

  // Character's constructor creates its rigid body eagerly with no way to
  // report failure: a NULL CreateBody just leaves the body id invalid.
  // Destroying a Character in that state is unsafe too — ~Character
  // unconditionally calls DestroyBody(mBodyID), an out-of-bounds access on
  // an invalid id, not a no-op. Room for one more body is checked up front,
  // and one without it is never destroyed — safe without touching Jolt.
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
  handle->owner = system;
  system->rigid_characters.push_back(handle);
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltRigidCharacterDestroy(ZJoltRigidCharacter *character) {
  if (character == nullptr) return;
  Unregister(character->owner == nullptr ? nullptr
                                         : &character->owner->rigid_characters,
             character);
  // Dropping the Ref runs Character's destructor, which destroys the body.
  character->impl = nullptr;
  zjolt::Delete(character);
  zjolt::HandleDestroyed();
}

void zjoltRigidCharacterAddToPhysicsSystem(ZJoltRigidCharacter *character,
                                           ZJoltActivation activation) {
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_activation = zjolt::RawEnum(activation);
  JPH::Character *impl = Impl(character);
  if (impl == nullptr) return;
  impl->AddToPhysicsSystem(ToJoltActivation(raw_activation));
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
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_activation = zjolt::RawEnum(activation);
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || position == nullptr || rotation == nullptr) return;
  impl->SetPositionAndRotation(zjolt::ToJoltR(*position),
                               zjolt::ToJoltRotation(*rotation),
                               ToJoltActivation(raw_activation));
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
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_activation = zjolt::RawEnum(activation);
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || position == nullptr) return;
  impl->SetPosition(zjolt::ToJoltR(*position), ToJoltActivation(raw_activation));
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
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_activation = zjolt::RawEnum(activation);
  JPH::Character *impl = Impl(character);
  if (impl == nullptr || rotation == nullptr) return;
  impl->SetRotation(zjolt::ToJoltRotation(*rotation), ToJoltActivation(raw_activation));
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

void zjoltRigidCharacterGetSupportingVolume(
    const ZJoltRigidCharacter *character, ZJoltVec3 *out_normal,
    float *out_distance) {
  const JPH::Character *impl = Impl(character);
  if (impl == nullptr) {
    if (out_normal != nullptr) *out_normal = ZJoltVec3{0, 0, 0};
    if (out_distance != nullptr) *out_distance = 0.0f;
    return;
  }
  const JPH::Plane &plane = impl->GetSupportingVolume();
  if (out_normal != nullptr) zjolt::WriteVec3(out_normal, plane.GetNormal());
  if (out_distance != nullptr) *out_distance = plane.GetConstant();
}

ZJoltResult zjoltRigidCharacterSetSupportingVolume(
    ZJoltRigidCharacter *character, const ZJoltVec3 *normal, float distance) {
  ZJOLT_ENTER();
  JPH::Character *impl = Impl(character);
  if (!zjolt::Present(impl, normal)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::Plane plane;
  const ZJoltResult built = MakeSupportingVolume(normal, distance, &plane);
  if (built != ZJOLT_RESULT_OK) return built;
  impl->SetSupportingVolume(plane);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltRigidCharacterGetTransformedShape(
    const ZJoltRigidCharacter *character, ZJoltTransformedShape **out) {
  ZJOLT_ENTER(out);
  const JPH::Character *impl = Impl(character);
  if (!zjolt::Present(impl, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  // Reads the body under a lock, so it is a shape with no body behind it
  // (mShape null) if the body has already been destroyed — OwnTransformedShape
  // reports that rather than handing back a handle that queries nothing.
  return OwnTransformedShape(impl->GetTransformedShape(), out);
}

ZJoltResult zjoltRigidCharacterCheckCollision(
    const ZJoltRigidCharacter *character, const ZJoltRVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *movement_direction,
    float max_separation_distance, const ZJoltShape *shape,
    ZJoltCharacterCollisionHit *out_hits, uint32_t capacity,
    uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  const JPH::Character *impl = Impl(character);
  if (!zjolt::Present(impl, position, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Shape *test_shape =
      shape != nullptr ? zjolt::ToJolt(shape) : impl->GetShape();
  if (test_shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "shape is NULL and the character has none either");
  }

  const JPH::Quat test_rotation =
      rotation != nullptr ? zjolt::ToJoltRotation(*rotation) : impl->GetRotation();
  const JPH::Vec3 direction = movement_direction != nullptr
                                  ? zjolt::ToJolt(*movement_direction)
                                  : JPH::Vec3::sZero();

  CharacterHitCollector collector(out_hits, capacity);
  // @see zjoltCharacterCheckCollision for why `position` is also the base
  // offset. Nothing here ever sets the collector's user data, so every hit
  // reports ZJOLT_CHARACTER_ID_INVALID: a rigid character has no
  // character-vs-character list.
  impl->CheckCollision(zjolt::ToJoltR(*position), test_rotation, direction,
                       max_separation_distance, test_shape,
                       zjolt::ToJoltR(*position), collector);

  return ReportHitCount(collector.count(), out_hits, capacity, out_count);
}

}  // extern "C"

namespace zjolt {

void ForgetCharacters(ZJoltPhysicsSystem *system) {
  for (ZJoltCharacter *character : system->characters) character->owner = nullptr;
  for (ZJoltRigidCharacter *character : system->rigid_characters) {
    character->owner = nullptr;
  }
  system->characters.clear();
  system->rigid_characters.clear();
}

}  // namespace zjolt
