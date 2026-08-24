//===----------------------------------------------------------------------===//
// zjolt — a whole world, described rather than simulated.
//
// A few conversions here are duplicated rather than shared, the same way
// zjolt_ragdoll.cpp and zjolt_vehicle.cpp duplicate theirs: the ZJoltBodyDesc
// and ZJoltSoftBodyDesc field mappings come from zjolt_body.cpp and
// zjolt_softbody.cpp, and the two handle casts come from zjolt_constraint.cpp
// and zjolt_softbody.cpp, where they are inline in their own translation
// units. This subsystem adds files and appends registrations; it does not
// edit the translation units those would otherwise be promoted out of.
//
// The container around the serialised payload is the one zjolt_shape.cpp and
// zjolt_state.cpp write, field for field, with a magic tag of its own — the
// tags differ deliberately, so a shape buffer handed to a scene restore is
// refused on the tag rather than parsed.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"
#include "zjolt_scene.h"

#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

//===----------------------------------------------------------------------===//
// Opaque handle mapping
//
// A scene is one of the handles Jolt constructs and reference counts itself,
// so it is a reinterpret_cast tag onto JPH::PhysicsScene exactly as ZJoltShape
// is onto JPH::Shape in zjolt_internal.h — never completed, never dereferenced
// as the tag, every use going back through the Jolt type first.
//===----------------------------------------------------------------------===//

namespace zjolt {

inline JPH::PhysicsScene *ToJolt(ZJoltScene *scene) {
  return reinterpret_cast<JPH::PhysicsScene *>(scene);
}
inline const JPH::PhysicsScene *ToJolt(const ZJoltScene *scene) {
  return reinterpret_cast<const JPH::PhysicsScene *>(scene);
}
inline ZJoltScene *ToC(JPH::PhysicsScene *scene) {
  return reinterpret_cast<ZJoltScene *>(scene);
}

/// Identical to zjolt_constraint.cpp's, for the reason zjolt_vehicle.cpp
/// gives for keeping its own copy: the tag is never completed or
/// dereferenced as itself, so duplicating one reinterpret_cast is a smaller
/// change than promoting it to a shared header.
inline const JPH::Constraint *ToJolt(const ZJoltConstraint *constraint) {
  return reinterpret_cast<const JPH::Constraint *>(constraint);
}

/// Likewise from zjolt_softbody.cpp. The const overload is new: that file
/// only ever hands the mutable one out, and a scene only ever reads.
inline const JPH::SoftBodySharedSettings *ToJolt(
    const ZJoltSoftBodySharedSettings *settings) {
  return reinterpret_cast<const JPH::SoftBodySharedSettings *>(settings);
}
inline const ZJoltSoftBodySharedSettings *ToC(
    const JPH::SoftBodySharedSettings *settings) {
  return reinterpret_cast<const ZJoltSoftBodySharedSettings *>(settings);
}

}  // namespace zjolt

/// The two spellings of "no body" have to be the same number, because this ABI
/// hands the caller's value straight to Jolt and reads Jolt's straight back.
static_assert(ZJOLT_SCENE_BODY_WORLD == JPH::PhysicsScene::cFixedToWorld,
              "ZJOLT_SCENE_BODY_WORLD no longer matches Jolt's cFixedToWorld");

namespace {

//===----------------------------------------------------------------------===//
// Descriptor conversion
//
// Mirrors zjolt_body.cpp's BuildCreationSettings and zjolt_softbody.cpp's
// BuildSoftBodyCreationSettings field for field, plus the read-back direction
// neither of them needs.
//===----------------------------------------------------------------------===//

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

ZJoltResult BuildCreationSettings(const ZJoltBodyDesc &desc,
                                  JPH::BodyCreationSettings *out) {
  if (desc.shape == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a body in a scene needs a shape");
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

  if (desc.override_mass_properties ==
      ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_INERTIA) {
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

/// The shape one body entry holds, or null — without building one.
///
/// GetShape() on its own is not safe to ask. When the entry carries unbuilt
/// shape SETTINGS instead of a shape (the two are mutually exclusive), it
/// builds the shape into a temporary and hands back a pointer that dies with
/// it (BodyCreationSettings.cpp:186). Nothing this ABI creates or restores is
/// ever in that state — the binary restore always stores a built shape — and
/// answering "no shape" for it is what keeps every caller here off that path.
const JPH::Shape *ShapeOf(const JPH::BodyCreationSettings &settings) {
  if (settings.GetShapeSettings() != nullptr) return nullptr;
  return settings.GetShape();
}

void ReadCreationSettings(const JPH::BodyCreationSettings &settings,
                          ZJoltBodyDesc *out) {
  *out = ZJoltBodyDesc{};
  out->position = zjolt::ToCR(settings.mPosition);
  out->rotation = zjolt::ToC(settings.mRotation);
  out->linear_velocity = zjolt::ToC(settings.mLinearVelocity);
  out->angular_velocity = zjolt::ToC(settings.mAngularVelocity);
  out->shape = zjolt::ToC(ShapeOf(settings));
  // The downcast is sound for the same reason zjoltBodyGetCollisionGroup's is:
  // the only filter that can reach a scene through this ABI came out of
  // zjoltGroupFilterTableCreate. A restored scene never carries one at all —
  // @see the note on group filters in zjolt_scene.h.
  out->collision_group.filter = static_cast<const ZJoltGroupFilter *>(
      settings.mCollisionGroup.GetGroupFilter());
  out->collision_group.group_id = settings.mCollisionGroup.GetGroupID();
  out->collision_group.sub_group_id = settings.mCollisionGroup.GetSubGroupID();
  out->user_data = settings.mUserData;
  out->object_layer = static_cast<ZJoltObjectLayer>(settings.mObjectLayer);
  out->motion_type = ToCMotionType(settings.mMotionType);
  out->motion_quality =
      settings.mMotionQuality == JPH::EMotionQuality::LinearCast
          ? ZJOLT_MOTION_QUALITY_LINEAR_CAST
          : ZJOLT_MOTION_QUALITY_DISCRETE;
  out->allowed_dofs = static_cast<uint32_t>(settings.mAllowedDOFs);
  out->allow_dynamic_or_kinematic = settings.mAllowDynamicOrKinematic;
  out->is_sensor = settings.mIsSensor;
  out->allow_sleeping = settings.mAllowSleeping;
  out->enhanced_internal_edge_removal = settings.mEnhancedInternalEdgeRemoval;
  out->friction = settings.mFriction;
  out->restitution = settings.mRestitution;
  out->linear_damping = settings.mLinearDamping;
  out->angular_damping = settings.mAngularDamping;
  out->max_linear_velocity = settings.mMaxLinearVelocity;
  out->max_angular_velocity = settings.mMaxAngularVelocity;
  out->gravity_factor = settings.mGravityFactor;

  // Jolt's third mass mode has no enumerator here. @see zjoltSceneGetBody for
  // what a caller loses by round-tripping one.
  if (settings.mOverrideMassProperties ==
      JPH::EOverrideMassProperties::CalculateMassAndInertia) {
    out->override_mass_properties =
        ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_MASS_AND_INERTIA;
    out->mass = 0.0f;
  } else {
    out->override_mass_properties =
        ZJOLT_OVERRIDE_MASS_PROPERTIES_CALCULATE_INERTIA;
    out->mass = settings.mMassPropertiesOverride.mMass;
  }
}

ZJoltResult BuildSoftBodyCreationSettings(const ZJoltSoftBodyDesc &desc,
                                          JPH::SoftBodyCreationSettings *out) {
  if (desc.shared_settings == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a soft body in a scene needs shared settings");
  }

  out->mSettings = zjolt::ToJolt(desc.shared_settings);
  out->mCollisionGroup = zjolt::ToJolt(&desc.collision_group);
  out->mPosition = zjolt::ToJoltR(desc.position);
  out->mRotation = zjolt::ToJoltRotation(desc.rotation);
  out->mUserData = desc.user_data;
  out->mObjectLayer = static_cast<JPH::ObjectLayer>(desc.object_layer);
  out->mNumIterations = desc.num_iterations;
  out->mLinearDamping = desc.linear_damping;
  out->mMaxLinearVelocity = desc.max_linear_velocity;
  out->mRestitution = desc.restitution;
  out->mFriction = desc.friction;
  out->mPressure = desc.pressure;
  out->mGravityFactor = desc.gravity_factor;
  out->mVertexRadius = desc.vertex_radius;
  out->mUpdatePosition = desc.update_position;
  out->mMakeRotationIdentity = desc.make_rotation_identity;
  out->mAllowSleeping = desc.allow_sleeping;
  out->mFacesDoubleSided = desc.faces_double_sided;
  return ZJOLT_RESULT_OK;
}

void ReadSoftBodyCreationSettings(const JPH::SoftBodyCreationSettings &settings,
                                  ZJoltSoftBodyDesc *out) {
  *out = ZJoltSoftBodyDesc{};
  out->shared_settings = zjolt::ToC(settings.mSettings.GetPtr());
  out->collision_group.filter = static_cast<const ZJoltGroupFilter *>(
      settings.mCollisionGroup.GetGroupFilter());
  out->collision_group.group_id = settings.mCollisionGroup.GetGroupID();
  out->collision_group.sub_group_id = settings.mCollisionGroup.GetSubGroupID();
  out->position = zjolt::ToCR(settings.mPosition);
  out->rotation = zjolt::ToC(settings.mRotation);
  out->user_data = settings.mUserData;
  out->object_layer = static_cast<ZJoltObjectLayer>(settings.mObjectLayer);
  out->num_iterations = settings.mNumIterations;
  out->linear_damping = settings.mLinearDamping;
  out->max_linear_velocity = settings.mMaxLinearVelocity;
  out->restitution = settings.mRestitution;
  out->friction = settings.mFriction;
  out->pressure = settings.mPressure;
  out->gravity_factor = settings.mGravityFactor;
  out->vertex_radius = settings.mVertexRadius;
  out->update_position = settings.mUpdatePosition;
  out->make_rotation_identity = settings.mMakeRotationIdentity;
  out->allow_sleeping = settings.mAllowSleeping;
  out->faces_double_sided = settings.mFacesDoubleSided;
}

//===----------------------------------------------------------------------===//
// The preconditions Jolt leaves to an assert
//
// Everything below is reachable only through zjoltSceneRestore — every
// zjoltSceneAdd* call refuses the same things at the door — but a buffer is
// input, and input is where a level that was cooked by an older tool arrives
// from. Each check names the Jolt line that would otherwise fire.
//===----------------------------------------------------------------------===//

/// What SaveBinaryState dereferences: a body's shape
/// (PhysicsScene.cpp:123 via BodyCreationSettings::SaveWithChildren) and a
/// constraint's settings (PhysicsScene.cpp:129).
ZJoltResult ValidateSavable(const JPH::PhysicsScene &scene) {
  for (const JPH::BodyCreationSettings &body : scene.GetBodies()) {
    if (ShapeOf(body) == nullptr) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a body in this scene has no shape, so there is nothing to write "
          "for it");
    }
  }
  for (const JPH::PhysicsScene::ConnectedConstraint &cc :
       scene.GetConstraints()) {
    if (cc.mSettings == nullptr) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a constraint in this scene has no settings, so there is nothing "
          "to write for it");
    }
  }
  return ZJOLT_RESULT_OK;
}

/// What CreateBodies dereferences on top of that: a soft body's shared
/// settings, and the body index each constraint end names —
/// `body_ids[cc.mBody1]` (PhysicsScene.cpp:103) indexes an array whose bounds
/// Jolt only asserts, and BodyInterface::CreateConstraint asserts that the two
/// ends are not the same body and that at least one of them is real
/// (BodyInterface.cpp:277,278).
ZJoltResult ValidateInstantiable(const JPH::PhysicsScene &scene) {
  const ZJoltResult savable = ValidateSavable(scene);
  if (savable != ZJOLT_RESULT_OK) return savable;

  for (const JPH::SoftBodyCreationSettings &body : scene.GetSoftBodies()) {
    if (body.mSettings == nullptr) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a soft body in this scene has no shared settings, so it has no "
          "geometry to simulate");
    }
  }

  const uint32_t num_bodies = static_cast<uint32_t>(scene.GetNumBodies());
  for (const JPH::PhysicsScene::ConnectedConstraint &cc :
       scene.GetConstraints()) {
    if (cc.mBody1 == cc.mBody2) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a constraint in this scene joins a body to itself, or joins the "
          "world to the world");
    }
    if ((cc.mBody1 != JPH::PhysicsScene::cFixedToWorld &&
         cc.mBody1 >= num_bodies) ||
        (cc.mBody2 != JPH::PhysicsScene::cFixedToWorld &&
         cc.mBody2 >= num_bodies)) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a constraint in this scene names a body index the scene does not "
          "have");
    }
  }
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// The container
//
// The layout zjolt_shape.cpp writes, with this subsystem's own magic tag. A
// scene buffer handed to zjoltShapeRestore, or the other way round, is refused
// on the tag rather than parsed into something plausible and wrong.
//===----------------------------------------------------------------------===//

constexpr uint8_t kMagic[4] = {'Z', 'J', 'S', 'C'};
constexpr uint32_t kFormatVersion = 1;
constexpr size_t kHeaderSize = 32;

/// CRC-32, the usual reflected polynomial, computed without a table. A level
/// is written once and read once; the table is not worth the cache line.
uint32_t Crc32(const uint8_t *data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xedb88320u & mask);
    }
  }
  return ~crc;
}

void WriteU32(uint8_t *out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t ReadU32(const uint8_t *in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

void WriteU64(uint8_t *out, uint64_t value) {
  WriteU32(out, static_cast<uint32_t>(value));
  WriteU32(out + 4, static_cast<uint32_t>(value >> 32));
}

uint64_t ReadU64(const uint8_t *in) {
  return static_cast<uint64_t>(ReadU32(in)) |
         (static_cast<uint64_t>(ReadU32(in + 4)) << 32);
}

uint32_t JoltVersionStamp() {
  return (static_cast<uint32_t>(JPH_VERSION_MAJOR) << 16) |
         (static_cast<uint32_t>(JPH_VERSION_MINOR) << 8) |
         static_cast<uint32_t>(JPH_VERSION_PATCH);
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Lifetime
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSceneCreate(ZJoltScene **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::PhysicsScene *fresh = zjolt::New<JPH::PhysicsScene>();
  if (fresh == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate a scene");
  }
  *out = zjolt::ToC(zjolt::Own(fresh));
  return ZJOLT_RESULT_OK;
}

void zjoltSceneAddRef(const ZJoltScene *scene) {
  if (scene == nullptr) return;
  zjolt::ToJolt(scene)->AddRef();
}

void zjoltSceneRelease(const ZJoltScene *scene) {
  if (scene == nullptr) return;
  zjolt::ToJolt(scene)->Release();
}

uint32_t zjoltSceneGetRefCount(const ZJoltScene *scene) {
  if (scene == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(scene)->GetRefCount());
}

//===----------------------------------------------------------------------===//
// Filling one in
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSceneAddBody(ZJoltScene *scene, const ZJoltBodyDesc *desc,
                              uint32_t *out_index) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out_index, ZJOLT_SCENE_BODY_WORLD));
  if (!zjolt::Present(scene, desc, out_index))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::BodyCreationSettings settings;
  const ZJoltResult built = BuildCreationSettings(*desc, &settings);
  if (built != ZJOLT_RESULT_OK) return built;

  JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);
  jolt->AddBody(settings);
  *out_index = static_cast<uint32_t>(jolt->GetNumBodies() - 1);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSceneAddSoftBody(ZJoltScene *scene,
                                  const ZJoltSoftBodyDesc *desc,
                                  uint32_t *out_index) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out_index, ZJOLT_SCENE_BODY_WORLD));
  if (!zjolt::Present(scene, desc, out_index))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SoftBodyCreationSettings settings;
  const ZJoltResult built = BuildSoftBodyCreationSettings(*desc, &settings);
  if (built != ZJOLT_RESULT_OK) return built;

  JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);
  jolt->AddSoftBody(settings);
  *out_index = static_cast<uint32_t>(jolt->GetNumSoftBodies() - 1);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSceneAddConstraint(ZJoltScene *scene,
                                    const ZJoltConstraint *constraint,
                                    uint32_t body1, uint32_t body2) {
  ZJOLT_ENTER();
  if (!zjolt::Present(scene, constraint))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Constraint *jolt_constraint = zjolt::ToJolt(constraint);
  if (jolt_constraint->GetType() != JPH::EConstraintType::TwoBodyConstraint) {
    return zjolt::SetError(
        ZJOLT_RESULT_UNSUPPORTED,
        "a scene holds joints between two bodies; this constraint is not one "
        "(a vehicle constraint is the one this ABI creates that is not)");
  }

  // BodyInterface::CreateConstraint asserts on both of these when the scene is
  // instantiated (BodyInterface.cpp:277,278), long after the mistake was made.
  if (body1 == body2) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        body1 == ZJOLT_SCENE_BODY_WORLD
            ? "a constraint cannot fix the world to the world"
            : "a constraint cannot join a body to itself");
  }

  JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);
  const uint32_t num_bodies = static_cast<uint32_t>(jolt->GetNumBodies());
  if ((body1 != ZJOLT_SCENE_BODY_WORLD && body1 >= num_bodies) ||
      (body2 != ZJOLT_SCENE_BODY_WORLD && body2 >= num_bodies)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a constraint names a body index this scene does not have yet; add "
        "the bodies before the constraints that join them");
  }

  JPH::Ref<JPH::ConstraintSettings> settings =
      jolt_constraint->GetConstraintSettings();
  if (settings == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not build settings for this constraint");
  }

  // The scene takes its own reference on assignment, so the local one dropping
  // at scope exit leaves the count where it belongs.
  jolt->AddConstraint(JPH::StaticCast<JPH::TwoBodyConstraintSettings>(settings),
                      body1, body2);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSceneFromPhysicsSystem(ZJoltScene *scene,
                                        const ZJoltPhysicsSystem *system) {
  ZJOLT_ENTER();
  if (!zjolt::Present(scene, system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::PhysicsSystem &jolt_system = system->system;

  // Jolt looks every constraint's two bodies up in a map it built from the
  // system's body list and asserts when one is missing (PhysicsScene.cpp:254),
  // then reads the end() iterator anyway in a build without asserts. The whole
  // system is checked first so that a refusal leaves `scene` untouched.
  JPH::BodyIDVector body_ids;
  jolt_system.GetBodies(body_ids);
  JPH::UnorderedSet<JPH::BodyID> present;
  present.reserve(body_ids.size());
  for (const JPH::BodyID &id : body_ids) present.insert(id);

  const JPH::Constraints constraints = jolt_system.GetConstraints();
  for (const JPH::Constraint *c : constraints) {
    if (c->GetType() != JPH::EConstraintType::TwoBodyConstraint) continue;
    const JPH::TwoBodyConstraint *tbc =
        static_cast<const JPH::TwoBodyConstraint *>(c);
    const JPH::BodyID id1 = tbc->GetBody1()->GetID();
    const JPH::BodyID id2 = tbc->GetBody2()->GetID();
    if ((!id1.IsInvalid() && present.find(id1) == present.end()) ||
        (!id2.IsInvalid() && present.find(id2) == present.end())) {
      return zjolt::SetError(
          ZJOLT_RESULT_BODY_NOT_FOUND,
          "a constraint in this system is attached to a body the system no "
          "longer holds, so there is no index to record it against");
    }
  }

  zjolt::ToJolt(scene)->FromPhysicsSystem(&jolt_system);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Reading one back
//===----------------------------------------------------------------------===//

uint32_t zjoltSceneGetNumBodies(const ZJoltScene *scene) {
  if (scene == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(scene)->GetNumBodies());
}

uint32_t zjoltSceneGetNumSoftBodies(const ZJoltScene *scene) {
  if (scene == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(scene)->GetNumSoftBodies());
}

uint32_t zjoltSceneGetNumConstraints(const ZJoltScene *scene) {
  if (scene == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(scene)->GetNumConstraints());
}

ZJoltResult zjoltSceneGetBody(const ZJoltScene *scene, uint32_t index,
                              ZJoltBodyDesc *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(scene, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);
  if (index >= jolt->GetNumBodies()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "index is past the last body in this scene");
  }
  ReadCreationSettings(jolt->GetBodies()[index], out);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSceneGetSoftBody(const ZJoltScene *scene, uint32_t index,
                                  ZJoltSoftBodyDesc *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(scene, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);
  if (index >= jolt->GetNumSoftBodies()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "index is past the last soft body in this scene");
  }
  ReadSoftBodyCreationSettings(jolt->GetSoftBodies()[index], out);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSceneGetConstraint(const ZJoltScene *scene, uint32_t index,
                                    ZJoltSceneConstraint *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(scene, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);
  if (index >= jolt->GetNumConstraints()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "index is past the last constraint in this scene");
  }

  const JPH::PhysicsScene::ConnectedConstraint &cc =
      jolt->GetConstraints()[index];
  out->body1 = cc.mBody1;
  out->body2 = cc.mBody2;
  if (cc.mSettings == nullptr) {
    // Only a hand-made buffer gets here, and reporting zeroes for a joint that
    // carries nothing is more useful than refusing to report the indices.
    return ZJOLT_RESULT_OK;
  }
  out->user_data = cc.mSettings->mUserData;
  out->priority = cc.mSettings->mConstraintPriority;
  out->num_velocity_steps_override =
      static_cast<uint32_t>(cc.mSettings->mNumVelocityStepsOverride);
  out->num_position_steps_override =
      static_cast<uint32_t>(cc.mSettings->mNumPositionStepsOverride);
  out->enabled = cc.mSettings->mEnabled;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Turning one into a world
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSceneCreateBodies(const ZJoltScene *scene,
                                   ZJoltPhysicsSystem *system) {
  ZJOLT_ENTER();
  if (!zjolt::Present(scene, system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);
  const ZJoltResult valid = ValidateInstantiable(*jolt);
  if (valid != ZJOLT_RESULT_OK) return valid;

  if (!jolt->CreateBodies(&system->system)) {
    return zjolt::SetError(
        ZJOLT_RESULT_OUT_OF_MEMORY,
        "the system ran out of body slots part way through the scene; the "
        "bodies created before that are still in it and no constraint was "
        "created at all");
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSceneFixInvalidScales(ZJoltScene *scene) {
  ZJOLT_ENTER();
  if (!zjolt::Present(scene)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);

  // FixInvalidScales dereferences every body's shape without checking it
  // (PhysicsScene.cpp:52).
  for (const JPH::BodyCreationSettings &body : jolt->GetBodies()) {
    if (ShapeOf(body) == nullptr) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a body in this scene has no shape, so there is no scale to fix");
    }
  }

  if (!jolt->FixInvalidScales()) {
    return zjolt::SetError(
        ZJOLT_RESULT_SHAPE_INVALID,
        "at least one shape in this scene could not be rescaled to unit "
        "scale; the ones that could be were");
  }
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Serialisation
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSceneSave(const ZJoltScene *scene, void *buffer,
                           size_t capacity, size_t *out_size) {
  ZJOLT_ENTER(out_size);
  if (!zjolt::Present(scene, out_size)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::PhysicsScene *jolt = zjolt::ToJolt(scene);
  const ZJoltResult valid = ValidateSavable(*jolt);
  if (valid != ZJOLT_RESULT_OK) return valid;

  uint8_t *bytes = static_cast<uint8_t *>(buffer);

  // A buffer that cannot even hold the header is counted, not written into:
  // `bytes + kHeaderSize` would otherwise form a pointer past the end of the
  // caller's allocation, which is undefined before anything is stored through
  // it.
  const bool count_only = bytes == nullptr || capacity < kHeaderSize;

  // Counting and writing are the same traversal, so the size a query reports
  // and the size a write consumes cannot drift apart.
  zjolt::CountingStreamOut stream(count_only ? nullptr : bytes + kHeaderSize,
                                  count_only ? 0 : capacity - kHeaderSize);
  jolt->SaveBinaryState(stream, /*inSaveShapes=*/true,
                        /*inSaveGroupFilter=*/false);

  const size_t payload_size = stream.Size();
  *out_size = kHeaderSize + payload_size;
  if (bytes == nullptr) return ZJOLT_RESULT_OK;
  if (count_only || capacity < *out_size || stream.IsFailed())
    return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  std::memcpy(bytes, kMagic, sizeof(kMagic));
  WriteU32(bytes + 4, kFormatVersion);
  WriteU32(bytes + 8, static_cast<uint32_t>(ZJOLT_CONFIG_ID));
  WriteU32(bytes + 12, JoltVersionStamp());
  WriteU64(bytes + 16, static_cast<uint64_t>(payload_size));
  WriteU32(bytes + 24, Crc32(bytes + kHeaderSize, payload_size));
  WriteU32(bytes + 28, 0);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSceneRestore(const void *data, size_t size,
                              ZJoltScene **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (data == nullptr || size == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "no data to restore a scene from");
  }

  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  if (size < kHeaderSize) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "too short to be a saved scene");
  }
  if (std::memcmp(bytes, kMagic, sizeof(kMagic)) != 0) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "not a scene saved by zjoltSceneSave");
  }
  if (ReadU32(bytes + 4) != kFormatVersion) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "saved by a different zjolt container version");
  }
  if (ReadU32(bytes + 8) != static_cast<uint32_t>(ZJOLT_CONFIG_ID)) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "saved by a zjolt built with different layout-affecting settings");
  }
  if (ReadU32(bytes + 12) != JoltVersionStamp()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "saved against a different Jolt version");
  }

  const uint64_t payload_size = ReadU64(bytes + 16);
  if (payload_size != static_cast<uint64_t>(size - kHeaderSize)) {
    return zjolt::SetError(
        ZJOLT_RESULT_BAD_FORMAT,
        "the recorded payload length does not match the buffer");
  }
  if (ReadU32(bytes + 24) !=
      Crc32(bytes + kHeaderSize, static_cast<size_t>(payload_size))) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the scene payload failed its checksum");
  }

  zjolt::ConstStreamIn stream(bytes + kHeaderSize,
                              static_cast<size_t>(payload_size));
  JPH::PhysicsScene::PhysicsSceneResult result =
      JPH::PhysicsScene::sRestoreFromBinaryState(stream);

  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT, result.GetError().c_str());
  }
  if (!result.IsValid() || stream.IsEOF()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the scene data ended before the scene did");
  }
  if (!stream.ConsumedAll()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "trailing bytes after the scene");
  }

  // `result` holds the only reference and drops it at scope exit, so this is
  // the same AddRef-once compensation Finish() makes in zjolt_shape.cpp --
  // NOT zjolt::Own's, which answers for a count that starts at zero.
  JPH::PhysicsScene *restored = result.Get();
  restored->AddRef();
  *out = zjolt::ToC(restored);
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
