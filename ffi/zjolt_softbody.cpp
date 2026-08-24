//===----------------------------------------------------------------------===//
// zjolt — soft bodies.
//
// ZJoltSoftBodySharedSettings is reference counted the same way a shape is,
// but its opaque-handle conversion lives here rather than in
// zjolt_internal.h: every OTHER reinterpret-cast tag is declared alongside
// the handles zjolt itself constructs, in a header this translation unit
// does not touch. Reopening `namespace zjolt` to add ToJolt/ToC here keeps
// the same naming convention and the same reasoning (never completed, never
// dereferenced as itself) without moving it.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

namespace zjolt {

inline const JPH::SoftBodySharedSettings *ToJolt(
    const ZJoltSoftBodySharedSettings *settings) {
  return reinterpret_cast<const JPH::SoftBodySharedSettings *>(settings);
}
inline JPH::SoftBodySharedSettings *ToJolt(
    ZJoltSoftBodySharedSettings *settings) {
  return reinterpret_cast<JPH::SoftBodySharedSettings *>(settings);
}
inline ZJoltSoftBodySharedSettings *ToC(
    JPH::SoftBodySharedSettings *settings) {
  return reinterpret_cast<ZJoltSoftBodySharedSettings *>(settings);
}

}  // namespace zjolt

namespace {

using JoltSettings = JPH::SoftBodySharedSettings;

JPH::EActivation ToJoltActivation(ZJoltActivation activation) {
  return activation == ZJOLT_ACTIVATION_DONT_ACTIVATE
             ? JPH::EActivation::DontActivate
             : JPH::EActivation::Activate;
}

JoltSettings::EBendType ToJoltBendType(ZJoltSoftBodyBendType type) {
  switch (type) {
    case ZJOLT_SOFT_BODY_BEND_TYPE_NONE:
      return JoltSettings::EBendType::None;
    case ZJOLT_SOFT_BODY_BEND_TYPE_DIHEDRAL:
      return JoltSettings::EBendType::Dihedral;
    case ZJOLT_SOFT_BODY_BEND_TYPE_DISTANCE:
      break;
  }
  return JoltSettings::EBendType::Distance;
}

JoltSettings::ELRAType ToJoltLraType(ZJoltSoftBodyLraType type) {
  switch (type) {
    case ZJOLT_SOFT_BODY_LRA_TYPE_NONE:
      return JoltSettings::ELRAType::None;
    case ZJOLT_SOFT_BODY_LRA_TYPE_GEODESIC_DISTANCE:
      return JoltSettings::ELRAType::GeodesicDistance;
    case ZJOLT_SOFT_BODY_LRA_TYPE_EUCLIDEAN_DISTANCE:
      break;
  }
  return JoltSettings::ELRAType::EuclideanDistance;
}

ZJoltSoftBodyLraType ToCLraType(JoltSettings::ELRAType type) {
  switch (type) {
    case JoltSettings::ELRAType::None:
      return ZJOLT_SOFT_BODY_LRA_TYPE_NONE;
    case JoltSettings::ELRAType::GeodesicDistance:
      return ZJOLT_SOFT_BODY_LRA_TYPE_GEODESIC_DISTANCE;
    case JoltSettings::ELRAType::EuclideanDistance:
      break;
  }
  return ZJOLT_SOFT_BODY_LRA_TYPE_EUCLIDEAN_DISTANCE;
}

/// Jolt's own Mat44 wire format: four columns of four floats each. This is
/// exactly what Float4 and sLoadFloat4x4 exist for, so the cast is the
/// sanctioned use of that type rather than an invented reinterpretation.
JPH::Mat44 ToJoltMat44(const float (&m)[16]) {
  return JPH::Mat44::sLoadFloat4x4(reinterpret_cast<const JPH::Float4 *>(m));
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Vertex attributes
//===----------------------------------------------------------------------===//

void zjoltSoftBodyVertexAttributesInit(ZJoltSoftBodyVertexAttributes *out) {
  if (out == nullptr) return;
  const JoltSettings::VertexAttributes defaults;
  out->compliance = defaults.mCompliance;
  out->shear_compliance = defaults.mShearCompliance;
  out->bend_compliance = defaults.mBendCompliance;
  out->lra_type = ToCLraType(defaults.mLRAType);
  out->lra_max_distance_multiplier = defaults.mLRAMaxDistanceMultiplier;
}

//===----------------------------------------------------------------------===//
// Shared settings: lifetime
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodySharedSettingsCreate(
    ZJoltSoftBodySharedSettings **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JoltSettings *settings = zjolt::New<JoltSettings>();
  if (settings == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate soft body shared settings");
  }
  *out = zjolt::ToC(zjolt::Own(settings));
  return ZJOLT_RESULT_OK;
}

void zjoltSoftBodySharedSettingsAddRef(
    const ZJoltSoftBodySharedSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->AddRef();
}

void zjoltSoftBodySharedSettingsRelease(
    const ZJoltSoftBodySharedSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->Release();
}

uint32_t zjoltSoftBodySharedSettingsGetRefCount(
    const ZJoltSoftBodySharedSettings *settings) {
  if (settings == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(settings)->GetRefCount());
}

//===----------------------------------------------------------------------===//
// Shared settings: geometry and constraints
//
// Every Add* here validates the WHOLE batch before appending any of it, so a
// bad entry partway through never leaves the settings half mutated.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodySharedSettingsAddVertices(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyVertex *vertices,
    uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(vertices))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  jolt_settings->mVertices.reserve(jolt_settings->mVertices.size() + count);
  for (uint32_t i = 0; i < count; ++i) {
    JoltSettings::Vertex v;
    v.mPosition = JPH::Float3(vertices[i].position.x, vertices[i].position.y,
                              vertices[i].position.z);
    v.mVelocity = JPH::Float3(vertices[i].velocity.x, vertices[i].velocity.y,
                              vertices[i].velocity.z);
    v.mInvMass = vertices[i].inv_mass;
    jolt_settings->mVertices.push_back(v);
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsAddFaces(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyFace *faces,
    uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(faces))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  for (uint32_t i = 0; i < count; ++i) {
    const ZJoltSoftBodyFace &f = faces[i];
    if (f.vertex[0] == f.vertex[1] || f.vertex[0] == f.vertex[2] ||
        f.vertex[1] == f.vertex[2]) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a face's three vertices must be distinct");
    }
  }

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  jolt_settings->mFaces.reserve(jolt_settings->mFaces.size() + count);
  for (uint32_t i = 0; i < count; ++i) {
    const ZJoltSoftBodyFace &f = faces[i];
    jolt_settings->mFaces.push_back(
        JoltSettings::Face(f.vertex[0], f.vertex[1], f.vertex[2], f.material_index));
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsAddEdges(
    ZJoltSoftBodySharedSettings *settings, const ZJoltSoftBodyEdge *edges,
    uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(edges))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  for (uint32_t i = 0; i < count; ++i) {
    if (edges[i].vertex[0] == edges[i].vertex[1]) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "an edge's two vertices must differ");
    }
  }

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  jolt_settings->mEdgeConstraints.reserve(
      jolt_settings->mEdgeConstraints.size() + count);
  for (uint32_t i = 0; i < count; ++i) {
    jolt_settings->mEdgeConstraints.push_back(JoltSettings::Edge(
        edges[i].vertex[0], edges[i].vertex[1], edges[i].compliance));
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsAddVolumeConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyVolumeConstraint *constraints, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(constraints))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t *v = constraints[i].vertex;
    if (v[0] == v[1] || v[0] == v[2] || v[0] == v[3] || v[1] == v[2] ||
        v[1] == v[3] || v[2] == v[3]) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a volume constraint's four vertices must be distinct");
    }
  }

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  jolt_settings->mVolumeConstraints.reserve(
      jolt_settings->mVolumeConstraints.size() + count);
  for (uint32_t i = 0; i < count; ++i) {
    const ZJoltSoftBodyVolumeConstraint &c = constraints[i];
    jolt_settings->mVolumeConstraints.push_back(JoltSettings::Volume(
        c.vertex[0], c.vertex[1], c.vertex[2], c.vertex[3], c.compliance));
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsAddInvBindMatrices(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyInvBind *inv_binds, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(inv_binds))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  jolt_settings->mInvBindMatrices.reserve(
      jolt_settings->mInvBindMatrices.size() + count);
  for (uint32_t i = 0; i < count; ++i) {
    jolt_settings->mInvBindMatrices.push_back(JoltSettings::InvBind(
        inv_binds[i].joint_index, ToJoltMat44(inv_binds[i].matrix)));
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsAddSkinnedConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodySkinned *constraints, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(constraints))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  jolt_settings->mSkinnedConstraints.reserve(
      jolt_settings->mSkinnedConstraints.size() + count);
  for (uint32_t i = 0; i < count; ++i) {
    const ZJoltSoftBodySkinned &c = constraints[i];
    JoltSettings::Skinned skinned(c.vertex, c.max_distance,
                                  c.back_stop_distance, c.back_stop_radius);
    for (uint32_t w = 0; w < JoltSettings::Skinned::cMaxSkinWeights; ++w) {
      skinned.mWeights[w] = JoltSettings::SkinWeight(
          c.weights[w].inv_bind_index, c.weights[w].weight);
    }
    jolt_settings->mSkinnedConstraints.push_back(skinned);
  }
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Shared settings: derived construction
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodySharedSettingsCreateConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyVertexAttributes *vertex_attributes,
    uint32_t vertex_attributes_count, ZJoltSoftBodyBendType bend_type,
    float angle_tolerance_radians) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings, vertex_attributes))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (vertex_attributes_count == 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "vertex_attributes_count must be at least 1: Jolt repeats the last "
        "element of this list for any vertex beyond it, which has no "
        "element to repeat when the list is empty");
  }

  JPH::Array<JoltSettings::VertexAttributes> attrs;
  attrs.reserve(vertex_attributes_count);
  for (uint32_t i = 0; i < vertex_attributes_count; ++i) {
    const ZJoltSoftBodyVertexAttributes &a = vertex_attributes[i];
    attrs.push_back(JoltSettings::VertexAttributes(
        a.compliance, a.shear_compliance, a.bend_compliance,
        ToJoltLraType(a.lra_type), a.lra_max_distance_multiplier));
  }

  zjolt::ToJolt(settings)->CreateConstraints(
      attrs.data(), static_cast<JPH::uint>(attrs.size()),
      ToJoltBendType(bend_type), angle_tolerance_radians);
  return ZJOLT_RESULT_OK;
}

void zjoltSoftBodySharedSettingsCalculateEdgeLengths(
    ZJoltSoftBodySharedSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->CalculateEdgeLengths();
}

void zjoltSoftBodySharedSettingsOptimize(ZJoltSoftBodySharedSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->Optimize();
}

//===----------------------------------------------------------------------===//
// Creating a soft body
//===----------------------------------------------------------------------===//

namespace {

ZJoltResult BuildSoftBodyCreationSettings(const ZJoltSoftBodyDesc &desc,
                                          JPH::SoftBodyCreationSettings *out) {
  if (desc.shared_settings == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a soft body needs shared settings");
  }

  out->mSettings = zjolt::ToJolt(desc.shared_settings);
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

}  // namespace

void zjoltSoftBodyDescInit(ZJoltSoftBodyDesc *desc) {
  if (desc == nullptr) return;

  // Read out of a default-constructed SoftBodyCreationSettings rather than
  // typed in, so a Jolt upgrade that changes a default moves this too.
  const JPH::SoftBodyCreationSettings defaults;

  *desc = ZJoltSoftBodyDesc{};
  desc->shared_settings = nullptr;
  desc->position = zjolt::ToCR(defaults.mPosition);
  desc->rotation = zjolt::ToC(defaults.mRotation);
  desc->user_data = defaults.mUserData;
  desc->object_layer = static_cast<ZJoltObjectLayer>(defaults.mObjectLayer);
  desc->num_iterations = defaults.mNumIterations;
  desc->linear_damping = defaults.mLinearDamping;
  desc->max_linear_velocity = defaults.mMaxLinearVelocity;
  desc->restitution = defaults.mRestitution;
  desc->friction = defaults.mFriction;
  desc->pressure = defaults.mPressure;
  desc->gravity_factor = defaults.mGravityFactor;
  desc->vertex_radius = defaults.mVertexRadius;
  desc->update_position = defaults.mUpdatePosition;
  desc->make_rotation_identity = defaults.mMakeRotationIdentity;
  desc->allow_sleeping = defaults.mAllowSleeping;
  desc->faces_double_sided = defaults.mFacesDoubleSided;
}

ZJoltResult zjoltSoftBodyCreate(ZJoltPhysicsSystem *system,
                                const ZJoltSoftBodyDesc *desc,
                                ZJoltBodyId *out) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out, (ZJoltBodyId)ZJOLT_BODY_ID_INVALID));
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SoftBodyCreationSettings settings;
  const ZJoltResult built = BuildSoftBodyCreationSettings(*desc, &settings);
  if (built != ZJOLT_RESULT_OK) return built;

  // CreateSoftBody, not CreateBody -- see the note at the top of
  // zjolt_softbody.h for how the two differ.
  JPH::Body *body = system->system.GetBodyInterface().CreateSoftBody(settings);
  if (body == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_OUT_OF_MEMORY,
        "the system is already holding max_bodies bodies");
  }
  *out = zjolt::ToC(body->GetID());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodyCreateAndAdd(ZJoltPhysicsSystem *system,
                                      const ZJoltSoftBodyDesc *desc,
                                      ZJoltActivation activation,
                                      ZJoltBodyId *out) {
  const ZJoltResult created = zjoltSoftBodyCreate(system, desc, out);
  if (created != ZJOLT_RESULT_OK) return created;
  system->system.GetBodyInterface().AddBody(zjolt::ToJolt(*out),
                                            ToJoltActivation(activation));
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Per-step read-back
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodyGetVertexStates(const ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body,
                                         ZJoltSoftBodyVertexState *out_states,
                                         uint32_t capacity,
                                         uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::BodyLockRead lock(system->system.GetBodyLockInterface(),
                         zjolt::ToJolt(body));
  if (!lock.Succeeded()) {
    return zjolt::SetError(ZJOLT_RESULT_BODY_NOT_FOUND,
                           "body id does not name a live body in this system");
  }
  const JPH::Body &jolt_body = lock.GetBody();
  if (!jolt_body.IsSoftBody()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "body is not a soft body");
  }

  const auto *motion = static_cast<const JPH::SoftBodyMotionProperties *>(
      jolt_body.GetMotionPropertiesUnchecked());
  const JPH::Array<JPH::SoftBodyVertex> &vertices = motion->GetVertices();
  const uint32_t count = static_cast<uint32_t>(vertices.size());
  *out_count = count;
  if (out_states == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  for (uint32_t i = 0; i < count; ++i) {
    out_states[i].position = zjolt::ToC(vertices[i].mPosition);
    out_states[i].velocity = zjolt::ToC(vertices[i].mVelocity);
  }
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
