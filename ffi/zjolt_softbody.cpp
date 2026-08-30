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

#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
// ScaleHelpers::IsInsideOut: CollideSoftBodyVerticesVsTriangles.h uses it
// without including it itself.
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVerticesVsTriangles.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodyShape.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyVertex.h>

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
inline const ZJoltSoftBodySharedSettings *ToC(
    const JPH::SoftBodySharedSettings *settings) {
  return reinterpret_cast<const ZJoltSoftBodySharedSettings *>(settings);
}

/// The manifold tag is the same never-completed, never-dereferenced kind, but
/// it names something zjolt does not own at all: a SoftBodyManifold is built
/// on the solver's stack and handed to one callback. There is deliberately no
/// ToC for a non-const one — a host may read a manifold and may not write it.
inline const JPH::SoftBodyManifold *ToJolt(
    const ZJoltSoftBodyManifold *manifold) {
  return reinterpret_cast<const JPH::SoftBodyManifold *>(manifold);
}
inline const ZJoltSoftBodyManifold *ToC(const JPH::SoftBodyManifold *manifold) {
  return reinterpret_cast<const ZJoltSoftBodyManifold *>(manifold);
}

/// zjoltCollideSoftBodyVerticesVsTrianglesCreate/Destroy own one of these
/// outright (New/Delete, not ref counted), so only one direction is needed.
inline JPH::CollideSoftBodyVerticesVsTriangles *ToJolt(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider) {
  return reinterpret_cast<JPH::CollideSoftBodyVerticesVsTriangles *>(collider);
}
inline ZJoltCollideSoftBodyVerticesVsTriangles *ToC(
    JPH::CollideSoftBodyVerticesVsTriangles *collider) {
  return reinterpret_cast<ZJoltCollideSoftBodyVerticesVsTriangles *>(collider);
}

}  // namespace zjolt

namespace {

using JoltSettings = JPH::SoftBodySharedSettings;

// These two take the raw integer, not the enum — see zjolt::RawEnum in
// zjolt_internal.h. Each entry point below converts once, at the boundary,
// before the value ever reaches here.
JPH::EActivation ToJoltActivation(int32_t activation) {
  return activation == ZJOLT_ACTIVATION_DONT_ACTIVATE
             ? JPH::EActivation::DontActivate
             : JPH::EActivation::Activate;
}

JoltSettings::EBendType ToJoltBendType(int32_t type) {
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

// Takes the raw integer, not the enum — see zjolt::RawEnum in
// zjolt_internal.h. Its one caller reads this straight out of a host-supplied
// vertex_attributes array, which is the entry point this value arrives from.
JoltSettings::ELRAType ToJoltLraType(int32_t type) {
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

/// Refuses a batch of vertex indices that name vertices the settings do
/// not have.
///
/// Nothing downstream index-checks this: the solver, volume measurement,
/// and skinning index straight into `mVertices`, reading past the end with
/// asserts off. Checks the count at call time — vertices must be added first.
bool VerticesExist(const JoltSettings &settings, const uint32_t *vertices,
                   uint32_t count) {
  const uint32_t have = static_cast<uint32_t>(settings.mVertices.size());
  for (uint32_t i = 0; i < count; ++i)
    if (vertices[i] >= have) return false;
  return true;
}

ZJoltResult NoSuchVertex(const char *what) {
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, what);
}

//===----------------------------------------------------------------------===//
// Reaching a body's soft-body motion properties
//
// SoftBodyMotionProperties is a SIBLING of MotionProperties, not a parent — `GetMotionPropertiesUnchecked` hands back the base of both, plus null for a static body — so the static_cast every Jolt sample writes is only sound once `IsSoftBody()` has been asked. Every entry point below goes through one of these two, which take the lock, check, and hand back a reference already of the right type.
//===----------------------------------------------------------------------===//

ZJoltResult BodyNotFound() {
  return zjolt::SetError(ZJOLT_RESULT_BODY_NOT_FOUND,
                         "body id does not name a live body in this system");
}

ZJoltResult NotASoftBody() {
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "body id names a rigid body; only a body created through "
      "zjoltSoftBodyCreate has soft body motion properties");
}

template <typename Fn>
ZJoltResult WithSoftBodyRead(const ZJoltPhysicsSystem *system, ZJoltBodyId body,
                             Fn &&fn) {
  const JPH::BodyLockRead lock(system->system.GetBodyLockInterface(),
                               zjolt::ToJolt(body));
  if (!lock.Succeeded()) return BodyNotFound();
  const JPH::Body &jolt_body = lock.GetBody();
  if (!jolt_body.IsSoftBody()) return NotASoftBody();
  return fn(*static_cast<const JPH::SoftBodyMotionProperties *>(
      jolt_body.GetMotionPropertiesUnchecked()));
}

/// As WithSoftBodyRead, but under a write lock, and the callback is handed the
/// body as well: skinning needs the body's own centre-of-mass transform.
template <typename Fn>
ZJoltResult WithSoftBodyWrite(ZJoltPhysicsSystem *system, ZJoltBodyId body,
                              Fn &&fn) {
  JPH::BodyLockWrite lock(system->system.GetBodyLockInterface(),
                          zjolt::ToJolt(body));
  if (!lock.Succeeded()) return BodyNotFound();
  JPH::Body &jolt_body = lock.GetBody();
  if (!jolt_body.IsSoftBody()) return NotASoftBody();
  return fn(jolt_body, *static_cast<JPH::SoftBodyMotionProperties *>(
                           jolt_body.GetMotionPropertiesUnchecked()));
}

/// Refuses a vertex index that is past the body's simulated vertices, for the
/// same reason VerticesExist refuses one at add time.
ZJoltResult VertexInRange(const JPH::SoftBodyMotionProperties &motion,
                          uint32_t index) {
  if (index < motion.GetVertices().size()) return ZJOLT_RESULT_OK;
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "vertex index is past the end of this body's vertices; "
      "zjoltSoftBodyGetVertexStates with a NULL buffer reports how many "
      "there are");
}

/// The check Jolt's own SetVertexRadius meant to make.
ZJoltResult VertexRadiusIsValid(float vertex_radius) {
  if (vertex_radius >= 0.0f) return ZJOLT_RESULT_OK;
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                         "vertex_radius must be >= 0; it is how far vertices "
                         "are held OFF a surface, not a signed offset");
}

/// As VerticesExist, for the rod indices a bend-twist constraint carries.
/// CalculateRodProperties indexes a connectivity array with them
/// (SoftBodySharedSettings.cpp:389) and the solver indexes the rod array
/// itself (:651), both unguarded.
bool RodsExist(const JoltSettings &settings, const uint32_t *rods,
               uint32_t count) {
  const uint32_t have =
      static_cast<uint32_t>(settings.mRodStretchShearConstraints.size());
  for (uint32_t i = 0; i < count; ++i)
    if (rods[i] >= have) return false;
  return true;
}

ZJoltResult NumIterationsIsValid(uint32_t num_iterations) {
  if (num_iterations != 0) return ZJOLT_RESULT_OK;
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "num_iterations must be at least 1: Jolt divides the step's delta time "
      "by it to size a sub-step, so zero makes every sub-step infinite");
}

//===----------------------------------------------------------------------===//
// ZJoltCollideSoftBodyVertexIterator plumbing
//
// position/collision_plane read or write ZJoltVec3/ZJoltPlane directly at
// the caller's own stride; Jolt's Vec3/Plane are SIMD-shaped and not byte-
// compatible with those, so every crossing into real Jolt code below gathers
// into a JPH type first and, for an output, scatters back after.
//===----------------------------------------------------------------------===//

/// Every array a real Shape::CollideSoftBodyVertices call dereferences
/// unconditionally must itself be non-NULL; a caller doing its own
/// bookkeeping outside these entry points is the only place the header's
/// NULL convention does not apply.
ZJoltResult VertexIteratorFieldsPresent(
    const ZJoltCollideSoftBodyVertexIterator &it) {
  if (it.position != nullptr && it.inv_mass != nullptr &&
      it.collision_plane != nullptr && it.largest_penetration != nullptr &&
      it.colliding_shape_index != nullptr) {
    return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "every ZJoltCollideSoftBodyVertexIterator field must be non-NULL");
}

JPH::Vec3 ReadVec3At(const uint8_t *base, uint32_t stride, uint32_t index) {
  return zjolt::ToJolt(
      *reinterpret_cast<const ZJoltVec3 *>(base + size_t(index) * stride));
}

JPH::Plane ReadPlaneAt(const uint8_t *base, uint32_t stride, uint32_t index) {
  const ZJoltPlane &p =
      *reinterpret_cast<const ZJoltPlane *>(base + size_t(index) * stride);
  return JPH::Plane(zjolt::ToJolt(p.normal), p.constant);
}

void WritePlaneAt(uint8_t *base, uint32_t stride, uint32_t index,
                  const JPH::Plane &plane) {
  ZJoltPlane &out = *reinterpret_cast<ZJoltPlane *>(base + size_t(index) * stride);
  out.normal = zjolt::ToC(plane.GetNormal());
  out.constant = plane.GetConstant();
}

float ReadFloatAt(const uint8_t *base, uint32_t stride, uint32_t index) {
  return *reinterpret_cast<const float *>(base + size_t(index) * stride);
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

ZJoltResult zjoltSoftBodySharedSettingsClone(
    const ZJoltSoftBodySharedSettings *settings,
    ZJoltSoftBodySharedSettings **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(settings, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // Jolt's Clone hands back a Ref, which already holds the one reference the
  // fresh object has. Own() adds the one that compensates for that Ref
  // dropping at the end of this scope — the same call as a fresh construction
  // makes, for the other of its two reasons (BINDING.md, Reference counting).
  JPH::Ref<JoltSettings> clone = zjolt::ToJolt(settings)->Clone();
  if (clone == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate the cloned shared settings");
  }
  *out = zjolt::ToC(zjolt::Own(clone.GetPtr()));
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Shared settings: materials
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodySharedSettingsSetMaterials(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltPhysicsMaterial *const *materials, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings, materials))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  // An empty list is not "no materials", it is an out-of-bounds read for
  // every face: SoftBodyShape::GetMaterial indexes this with the face's
  // material index and never checks it.
  if (count == 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a soft body's material list must hold at least one material; pass "
        "zjoltPhysicsMaterialDefault() rather than an empty list");
  }
  for (uint32_t i = 0; i < count; ++i) {
    if (materials[i] == nullptr) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a material in the list is NULL; pass "
                             "zjoltPhysicsMaterialDefault() for a slot with "
                             "nothing of its own");
    }
  }

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  for (const JoltSettings::Face &face : jolt_settings->mFaces) {
    if (face.mMaterialIndex >= count) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a face already added names a material index past the end of the "
          "new list; the list is not replaced");
    }
  }

  JPH::PhysicsMaterialList replacement;
  replacement.reserve(count);
  for (uint32_t i = 0; i < count; ++i)
    replacement.push_back(zjolt::ToJolt(materials[i]));
  jolt_settings->mMaterials = std::move(replacement);
  return ZJOLT_RESULT_OK;
}

namespace {

constexpr uint8_t kSharedSettingsStreamMagic[4] = {'Z', 'S', 'B', 'B'};
constexpr uint8_t kSharedSettingsWithMaterialsMagic[4] = {'Z', 'S', 'B', 'M'};

ZJoltResult SaveSharedSettings(const ZJoltSoftBodySharedSettings *settings,
                               const ZJoltStream *stream,
                               const uint8_t magic[4], bool with_materials) {
  if (!zjolt::Present(settings, stream)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanWrite(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs write and is_failed to save through");
  }

  zjolt::HostStream host(*stream);
  zjolt::WriteStreamHeader(host, magic);
  const JPH::SoftBodySharedSettings *s = zjolt::ToJolt(settings);
  if (with_materials) {
    JPH::SoftBodySharedSettings::SharedSettingsToIDMap settings_map;
    JPH::SoftBodySharedSettings::MaterialToIDMap material_map;
    s->SaveWithMaterials(host, settings_map, material_map);
  } else {
    s->SaveBinaryState(host);
  }

  if (host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_IO_ERROR,
                           "the stream failed while writing the soft body "
                           "shared settings");
  }
  return ZJOLT_RESULT_OK;
}

}  // namespace

ZJoltResult zjoltSoftBodySharedSettingsSaveBinaryState(
    const ZJoltSoftBodySharedSettings *settings, const ZJoltStream *stream) {
  ZJOLT_ENTER();
  return SaveSharedSettings(settings, stream, kSharedSettingsStreamMagic,
                            false);
}

ZJoltResult zjoltSoftBodySharedSettingsSaveWithMaterials(
    const ZJoltSoftBodySharedSettings *settings, const ZJoltStream *stream) {
  ZJOLT_ENTER();
  return SaveSharedSettings(settings, stream, kSharedSettingsWithMaterialsMagic,
                            true);
}

ZJoltResult zjoltSoftBodySharedSettingsRestoreBinaryState(
    const ZJoltStream *stream, ZJoltSoftBodySharedSettings **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(stream, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs read and is_failed to restore from");
  }

  zjolt::HostStream host(*stream);
  const ZJoltResult header = zjolt::ReadStreamHeader(
      host, kSharedSettingsStreamMagic,
      "not soft body shared settings saved by "
      "zjoltSoftBodySharedSettingsSaveBinaryState");
  if (header != ZJOLT_RESULT_OK) return header;

  JPH::Ref<JPH::SoftBodySharedSettings> restored =
      new JPH::SoftBodySharedSettings();
  restored->RestoreBinaryState(host);
  if (host.IsEOF() || host.IsFailed()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT,
                           "the stream ran out while reading the soft body "
                           "shared settings");
  }

  restored->AddRef();
  *out = zjolt::ToC(restored.GetPtr());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsRestoreWithMaterials(
    const ZJoltStream *stream, ZJoltSoftBodySharedSettings **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(stream, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!zjolt::StreamCanRead(stream)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "stream needs read and is_failed to restore from");
  }

  zjolt::HostStream host(*stream);
  const ZJoltResult header = zjolt::ReadStreamHeader(
      host, kSharedSettingsWithMaterialsMagic,
      "not soft body shared settings saved by "
      "zjoltSoftBodySharedSettingsSaveWithMaterials");
  if (header != ZJOLT_RESULT_OK) return header;

  JPH::SoftBodySharedSettings::IDToSharedSettingsMap settings_map;
  JPH::SoftBodySharedSettings::IDToMaterialMap material_map;
  JPH::SoftBodySharedSettings::SettingsResult result =
      JPH::SoftBodySharedSettings::sRestoreWithMaterials(host, settings_map,
                                                         material_map);
  if (result.HasError()) {
    return zjolt::SetError(ZJOLT_RESULT_BAD_FORMAT, result.GetError().c_str());
  }

  JPH::Ref<JPH::SoftBodySharedSettings> restored = result.Get();
  restored->AddRef();
  *out = zjolt::ToC(restored.GetPtr());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsGetMaterials(
    const ZJoltSoftBodySharedSettings *settings,
    const ZJoltPhysicsMaterial **out_materials, uint32_t capacity,
    uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(settings, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::PhysicsMaterialList &list = zjolt::ToJolt(settings)->mMaterials;
  const uint32_t count = static_cast<uint32_t>(list.size());
  *out_count = count;
  if (out_materials == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i)
    out_materials[i] = zjolt::ToC(list[i].GetPtr());
  return ZJOLT_RESULT_OK;
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
    if (!VerticesExist(*zjolt::ToJolt(settings), f.vertex, 3)) {
      return NoSuchVertex(
          "a face names a vertex that has not been added yet; add every "
          "vertex before the faces that reference them");
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
    if (!VerticesExist(*zjolt::ToJolt(settings), edges[i].vertex, 2)) {
      return NoSuchVertex(
          "an edge names a vertex that has not been added yet; add every "
          "vertex before the constraints that reference them");
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
    if (!VerticesExist(*zjolt::ToJolt(settings), v, 4)) {
      return NoSuchVertex(
          "a volume constraint names a vertex that has not been added yet; "
          "add every vertex before the constraints that reference them");
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

ZJoltResult zjoltSoftBodySharedSettingsAddRodStretchShearConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyRodStretchShear *rods, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(rods))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  for (uint32_t i = 0; i < count; ++i) {
    if (rods[i].vertex[0] == rods[i].vertex[1]) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a rod's two vertices must differ");
    }
    if (!VerticesExist(*zjolt::ToJolt(settings), rods[i].vertex, 2)) {
      return NoSuchVertex(
          "a rod names a vertex that has not been added yet; add every "
          "vertex before the constraints that reference them");
    }
  }

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  jolt_settings->mRodStretchShearConstraints.reserve(
      jolt_settings->mRodStretchShearConstraints.size() + count);
  for (uint32_t i = 0; i < count; ++i) {
    jolt_settings->mRodStretchShearConstraints.push_back(
        JoltSettings::RodStretchShear(rods[i].vertex[0], rods[i].vertex[1],
                                      rods[i].compliance));
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsAddRodBendTwistConstraints(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyRodBendTwist *constraints, uint32_t count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (count != 0 && !zjolt::Present(constraints))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  for (uint32_t i = 0; i < count; ++i) {
    if (constraints[i].rod[0] == constraints[i].rod[1]) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a rod bend twist constraint must join two DIFFERENT rods");
    }
    if (!RodsExist(*zjolt::ToJolt(settings), constraints[i].rod, 2)) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a rod bend twist constraint names a rod that has not been added "
          "yet; add every rod before the constraints that join them");
    }
  }

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);
  jolt_settings->mRodBendTwistConstraints.reserve(
      jolt_settings->mRodBendTwistConstraints.size() + count);
  for (uint32_t i = 0; i < count; ++i) {
    jolt_settings->mRodBendTwistConstraints.push_back(
        JoltSettings::RodBendTwist(constraints[i].rod[0], constraints[i].rod[1],
                                   constraints[i].compliance));
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

  for (uint32_t i = 0; i < count; ++i) {
    if (!VerticesExist(*zjolt::ToJolt(settings), &constraints[i].vertex, 1)) {
      return NoSuchVertex(
          "a skinned constraint names a vertex that has not been added yet; "
          "Jolt indexes its vertex array with it on every skinning call and "
          "on every step, so add every vertex first");
    }
  }

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
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_bend_type = zjolt::RawEnum(bend_type);
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
        ToJoltLraType(zjolt::RawEnum(a.lra_type)), a.lra_max_distance_multiplier));
  }

  zjolt::ToJolt(settings)->CreateConstraints(
      attrs.data(), static_cast<JPH::uint>(attrs.size()),
      ToJoltBendType(raw_bend_type), angle_tolerance_radians);
  return ZJOLT_RESULT_OK;
}

void zjoltSoftBodySharedSettingsCalculateEdgeLengths(
    ZJoltSoftBodySharedSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->CalculateEdgeLengths();
}

void zjoltSoftBodySharedSettingsCalculateVolumeConstraintVolumes(
    ZJoltSoftBodySharedSettings *settings) {
  if (settings == nullptr) return;
  // Jolt asserts here that the four vertices differ. AddVolumeConstraints is
  // the only way one gets in, and it already refused a repeat — so there is
  // nothing left for this to check, unlike its rod sibling below.
  zjolt::ToJolt(settings)->CalculateVolumeConstraintVolumes();
}

ZJoltResult zjoltSoftBodySharedSettingsCalculateRodProperties(
    ZJoltSoftBodySharedSettings *settings) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);

  // A rod's frame is built by normalising the vector between its two
  // vertices, and its length is what that normalisation divides by
  // (SoftBodySharedSettings.cpp:411-413). Two vertices at the same position
  // give zero, which Jolt asserts on and, with asserts off, turns into a NaN
  // quaternion that every rod downstream of it in the chain inherits. Checked
  // before anything is written, so a refusal leaves the settings untouched.
  for (const JoltSettings::RodStretchShear &rod :
       jolt_settings->mRodStretchShearConstraints) {
    const JPH::Vec3 a(jolt_settings->mVertices[rod.mVertex[0]].mPosition);
    const JPH::Vec3 b(jolt_settings->mVertices[rod.mVertex[1]].mPosition);
    if ((b - a).IsNearZero()) {
      return zjolt::SetError(
          ZJOLT_RESULT_INVALID_ARGUMENT,
          "a rod's two vertices are at the same position, so it has no "
          "length and no direction to build a rest frame from");
    }
  }

  jolt_settings->CalculateRodProperties();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsCalculateSkinnedConstraintNormals(
    ZJoltSoftBodySharedSettings *settings) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JoltSettings *jolt_settings = zjolt::ToJolt(settings);

  // Jolt packs each skinned vertex's face count into the top 8 bits of a
  // uint32 and asserts it fits (SoftBodySharedSettings.cpp:602): the number
  // of faces meeting at that vertex whose OTHER two vertices are skinned
  // too, counted the same way here rather than approximated by the
  // vertex's total degree.
  const size_t num_vertices = jolt_settings->mVertices.size();
  if (num_vertices != 0 && !jolt_settings->mSkinnedConstraints.empty()) {
    JPH::Array<uint8_t> skinned(num_vertices, 0);
    for (const JoltSettings::Skinned &s : jolt_settings->mSkinnedConstraints)
      skinned[s.mVertex] = 1;

    JPH::Array<uint32_t> faces_at(num_vertices, 0);
    for (const JoltSettings::Face &f : jolt_settings->mFaces) {
      if (skinned[f.mVertex[0]] == 0 || skinned[f.mVertex[1]] == 0 ||
          skinned[f.mVertex[2]] == 0)
        continue;
      for (uint32_t v : f.mVertex) ++faces_at[v];
    }

    for (const JoltSettings::Skinned &s : jolt_settings->mSkinnedConstraints) {
      if (faces_at[s.mVertex] >= 256) {
        return zjolt::SetError(
            ZJOLT_RESULT_INVALID_ARGUMENT,
            "more than 255 fully skinned faces meet at one skinned vertex, "
            "and Jolt stores that count in 8 bits");
      }
    }
  }

  jolt_settings->CalculateSkinnedConstraintNormals();
  return ZJOLT_RESULT_OK;
}

void zjoltSoftBodySharedSettingsOptimize(ZJoltSoftBodySharedSettings *settings) {
  if (settings == nullptr) return;
  zjolt::ToJolt(settings)->Optimize();
}

namespace {

constexpr size_t kRemapCount = 7;

/// The two structs are one list of seven written twice, so neither may grow
/// without the other: a capacity with no buffer, or a buffer with no
/// capacity, is a remap checked against the wrong length.
static_assert(sizeof(ZJoltSoftBodyRemapBuffers) ==
                  kRemapCount * sizeof(uint32_t *),
              "ZJoltSoftBodyRemapBuffers must hold exactly the seven remaps");
static_assert(sizeof(ZJoltSoftBodyRemapCounts) ==
                  kRemapCount * sizeof(uint32_t),
              "ZJoltSoftBodyRemapCounts must hold exactly the seven remaps");

ZJoltResult CheckRemapCapacity(uint32_t capacity, size_t needed,
                               const char *what) {
  if (static_cast<size_t>(capacity) < needed) {
    return zjolt::SetError(ZJOLT_RESULT_BUFFER_TOO_SMALL, what);
  }
  return ZJOLT_RESULT_OK;
}

void CopyRemap(const JPH::Array<JPH::uint> &from, uint32_t *to) {
  if (to == nullptr) return;
  for (size_t i = 0; i < from.size(); ++i) {
    to[i] = static_cast<uint32_t>(from[i]);
  }
}

}  // namespace

ZJoltResult zjoltSoftBodySharedSettingsGetRemapCounts(
    const ZJoltSoftBodySharedSettings *settings,
    ZJoltSoftBodyRemapCounts *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(settings, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::SoftBodySharedSettings *s = zjolt::ToJolt(settings);
  out->edges = static_cast<uint32_t>(s->mEdgeConstraints.size());
  out->lra = static_cast<uint32_t>(s->mLRAConstraints.size());
  out->rod_stretch_shear =
      static_cast<uint32_t>(s->mRodStretchShearConstraints.size());
  out->rod_bend_twist =
      static_cast<uint32_t>(s->mRodBendTwistConstraints.size());
  out->dihedral_bend =
      static_cast<uint32_t>(s->mDihedralBendConstraints.size());
  out->volume = static_cast<uint32_t>(s->mVolumeConstraints.size());
  out->skinned = static_cast<uint32_t>(s->mSkinnedConstraints.size());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySharedSettingsOptimizeWithRemap(
    ZJoltSoftBodySharedSettings *settings,
    const ZJoltSoftBodyRemapBuffers *out_remap,
    const ZJoltSoftBodyRemapCounts *capacity) {
  ZJOLT_ENTER();
  if (!zjolt::Present(settings, out_remap, capacity)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  // Every capacity checked before anything is reordered: a refusal here has
  // to leave the settings usable, and Optimize is not undoable.
  ZJoltSoftBodyRemapCounts needed;
  const ZJoltResult counted =
      zjoltSoftBodySharedSettingsGetRemapCounts(settings, &needed);
  if (counted != ZJOLT_RESULT_OK) return counted;

  struct Pair {
    uint32_t *buffer;
    uint32_t capacity;
    uint32_t needed;
    const char *what;
  };
  const Pair pairs[kRemapCount] = {
      {out_remap->edges, capacity->edges, needed.edges,
       "the edge remap buffer is shorter than the edge constraint list"},
      {out_remap->lra, capacity->lra, needed.lra,
       "the LRA remap buffer is shorter than the LRA constraint list"},
      {out_remap->rod_stretch_shear, capacity->rod_stretch_shear,
       needed.rod_stretch_shear,
       "the rod stretch-shear remap buffer is shorter than its list"},
      {out_remap->rod_bend_twist, capacity->rod_bend_twist,
       needed.rod_bend_twist,
       "the rod bend-twist remap buffer is shorter than its list"},
      {out_remap->dihedral_bend, capacity->dihedral_bend, needed.dihedral_bend,
       "the dihedral bend remap buffer is shorter than its list"},
      {out_remap->volume, capacity->volume, needed.volume,
       "the volume remap buffer is shorter than the volume constraint list"},
      {out_remap->skinned, capacity->skinned, needed.skinned,
       "the skinned remap buffer is shorter than the skinned constraint list"},
  };
  for (const Pair &pair : pairs) {
    if (pair.buffer == nullptr) continue;
    const ZJoltResult fits =
        CheckRemapCapacity(pair.capacity, pair.needed, pair.what);
    if (fits != ZJOLT_RESULT_OK) return fits;
  }

  JPH::SoftBodySharedSettings::OptimizationResults results;
  zjolt::ToJolt(settings)->Optimize(results);

  CopyRemap(results.mEdgeRemap, out_remap->edges);
  CopyRemap(results.mLRARemap, out_remap->lra);
  CopyRemap(results.mRodStretchShearConstraintRemap,
            out_remap->rod_stretch_shear);
  CopyRemap(results.mRodBendTwistConstraintRemap, out_remap->rod_bend_twist);
  CopyRemap(results.mDihedralBendRemap, out_remap->dihedral_bend);
  CopyRemap(results.mVolumeRemap, out_remap->volume);
  CopyRemap(results.mSkinnedRemap, out_remap->skinned);
  return ZJOLT_RESULT_OK;
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

  // Both of these reach a Jolt that does not check them: Initialize forwards
  // the radius to SetVertexRadius, whose assert reads the value it is about
  // to overwrite rather than this one, and the iteration count becomes a
  // divisor. @see zjoltSoftBodySetVertexRadius, ZJoltSoftBodyDesc.
  const ZJoltResult radius_ok = VertexRadiusIsValid(desc.vertex_radius);
  if (radius_ok != ZJOLT_RESULT_OK) return radius_ok;
  const ZJoltResult iterations_ok = NumIterationsIsValid(desc.num_iterations);
  if (iterations_ok != ZJOLT_RESULT_OK) return iterations_ok;

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

}  // namespace

void zjoltSoftBodyDescInit(ZJoltSoftBodyDesc *desc) {
  if (desc == nullptr) return;

  // Read out of a default-constructed SoftBodyCreationSettings rather than
  // typed in, so a Jolt upgrade that changes a default moves this too.
  const JPH::SoftBodyCreationSettings defaults;

  *desc = ZJoltSoftBodyDesc{};
  desc->shared_settings = nullptr;
  // Not all-zero: "no group" is ZJOLT_COLLISION_GROUP_INVALID, and 0 is a
  // perfectly good group id.
  desc->collision_group.filter = nullptr;
  desc->collision_group.group_id = defaults.mCollisionGroup.GetGroupID();
  desc->collision_group.sub_group_id = defaults.mCollisionGroup.GetSubGroupID();
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
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_activation = zjolt::RawEnum(activation);
  const ZJoltResult created = zjoltSoftBodyCreate(system, desc, out);
  if (created != ZJOLT_RESULT_OK) return created;
  system->system.GetBodyInterface().AddBody(zjolt::ToJolt(*out),
                                            ToJoltActivation(raw_activation));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodyCreateWithId(ZJoltPhysicsSystem *system,
                                      const ZJoltSoftBodyDesc *desc,
                                      ZJoltBodyId id, ZJoltBodyId *out) {
  ZJOLT_ENTER(zjolt::OutIsEmptyAs(out, (ZJoltBodyId)ZJOLT_BODY_ID_INVALID));
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (id == JPH::BodyID::cInvalidBodyID) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "id: ZJOLT_BODY_ID_INVALID does not name a body");
  }
  // See zjoltBodyCreateWithId (zjolt_body.cpp) for why this is checked ahead
  // of ever constructing a JPH::BodyID from it.
  if ((id & JPH::BodyID::cBroadPhaseBit) != 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "id: bit 31 is reserved for the broad phase and cannot be set in a "
        "custom body id");
  }

  JPH::SoftBodyCreationSettings settings;
  const ZJoltResult built = BuildSoftBodyCreationSettings(*desc, &settings);
  if (built != ZJOLT_RESULT_OK) return built;

  JPH::Body *body = system->system.GetBodyInterface().CreateSoftBodyWithID(
      zjolt::ToJolt(id), settings);
  if (body == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "id: already names a live body, or its index is beyond max_bodies");
  }
  *out = zjolt::ToC(body->GetID());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodyCreateAndAddWithId(ZJoltPhysicsSystem *system,
                                            const ZJoltSoftBodyDesc *desc,
                                            ZJoltBodyId id,
                                            ZJoltActivation activation,
                                            ZJoltBodyId *out) {
  // Converted here, at the entry point that receives it from the host — see
  // zjolt::RawEnum in zjolt_internal.h.
  const int32_t raw_activation = zjolt::RawEnum(activation);
  const ZJoltResult created = zjoltSoftBodyCreateWithId(system, desc, id, out);
  if (created != ZJOLT_RESULT_OK) return created;
  system->system.GetBodyInterface().AddBody(zjolt::ToJolt(*out),
                                            ToJoltActivation(raw_activation));
  return ZJOLT_RESULT_OK;
}

const ZJoltSoftBodySharedSettings *zjoltSoftBodyGetSharedSettings(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body) {
  if (system == nullptr) return nullptr;
  const JPH::BodyLockRead lock(system->system.GetBodyLockInterface(),
                               zjolt::ToJolt(body));
  if (!lock.Succeeded()) return nullptr;
  const JPH::Body &jolt_body = lock.GetBody();
  if (!jolt_body.IsSoftBody()) return nullptr;
  const auto *motion = static_cast<const JPH::SoftBodyMotionProperties *>(
      jolt_body.GetMotionPropertiesUnchecked());
  return zjolt::ToC(motion->GetSettings());
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

  return WithSoftBodyRead(
      system, body, [&](const JPH::SoftBodyMotionProperties &motion) {
        const JPH::Array<JPH::SoftBodyVertex> &vertices = motion.GetVertices();
        const uint32_t count = static_cast<uint32_t>(vertices.size());
        *out_count = count;
        if (out_states == nullptr) return ZJOLT_RESULT_OK;
        if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

        for (uint32_t i = 0; i < count; ++i) {
          out_states[i].position = zjolt::ToC(vertices[i].mPosition);
          out_states[i].velocity = zjolt::ToC(vertices[i].mVelocity);
        }
        return ZJOLT_RESULT_OK;
      });
}

//===----------------------------------------------------------------------===//
// Live properties
//
// Each of these guards the out-parameter, resolves the motion properties under a lock, and touches one field — written out one at a time rather than folded into a property enum, since a caller reads a name and a doc comment, not a table.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodyGetNumIterations(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body, uint32_t *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            *out = motion.GetNumIterations();
                            return ZJOLT_RESULT_OK;
                          });
}

ZJoltResult zjoltSoftBodySetNumIterations(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body,
                                          uint32_t num_iterations) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const ZJoltResult valid = NumIterationsIsValid(num_iterations);
  if (valid != ZJOLT_RESULT_OK) return valid;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        motion.SetNumIterations(num_iterations);
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodyGetPressure(const ZJoltPhysicsSystem *system,
                                     ZJoltBodyId body, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            *out = motion.GetPressure();
                            return ZJOLT_RESULT_OK;
                          });
}

ZJoltResult zjoltSoftBodySetPressure(ZJoltPhysicsSystem *system,
                                     ZJoltBodyId body, float pressure) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        motion.SetPressure(pressure);
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodyGetUpdatePosition(const ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body, bool *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            *out = motion.GetUpdatePosition();
                            return ZJOLT_RESULT_OK;
                          });
}

ZJoltResult zjoltSoftBodySetUpdatePosition(ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body,
                                           bool update_position) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        motion.SetUpdatePosition(update_position);
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodyGetFacesDoubleSided(const ZJoltPhysicsSystem *system,
                                             ZJoltBodyId body, bool *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            *out = motion.GetFacesDoubleSided();
                            return ZJOLT_RESULT_OK;
                          });
}

ZJoltResult zjoltSoftBodySetFacesDoubleSided(ZJoltPhysicsSystem *system,
                                             ZJoltBodyId body,
                                             bool double_sided) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        motion.SetFacesDoubleSided(double_sided);
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodyGetVertexRadius(const ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            *out = motion.GetVertexRadius();
                            return ZJOLT_RESULT_OK;
                          });
}

ZJoltResult zjoltSoftBodySetVertexRadius(ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body,
                                         float vertex_radius) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  // Checked BEFORE the lock, and on the incoming value: Jolt's own assert
  // reads the value it is about to overwrite instead, so a negative radius
  // gets in and is blamed on the next caller. @see VertexRadiusIsValid.
  const ZJoltResult valid = VertexRadiusIsValid(vertex_radius);
  if (valid != ZJOLT_RESULT_OK) return valid;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        motion.SetVertexRadius(vertex_radius);
        return ZJOLT_RESULT_OK;
      });
}

//===----------------------------------------------------------------------===//
// One vertex at a time
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodyGetVertexVelocity(const ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body, uint32_t index,
                                           ZJoltVec3 *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(
      system, body, [&](const JPH::SoftBodyMotionProperties &motion) {
        const ZJoltResult in_range = VertexInRange(motion, index);
        if (in_range != ZJOLT_RESULT_OK) return in_range;
        *out = zjolt::ToC(motion.GetVertex(index).mVelocity);
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodySetVertexVelocity(ZJoltPhysicsSystem *system,
                                           ZJoltBodyId body, uint32_t index,
                                           const ZJoltVec3 *velocity) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, velocity)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        const ZJoltResult in_range = VertexInRange(motion, index);
        if (in_range != ZJOLT_RESULT_OK) return in_range;
        motion.GetVertex(index).mVelocity = zjolt::ToJolt(*velocity);
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodyGetVertexInvMass(const ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body, uint32_t index,
                                          float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(
      system, body, [&](const JPH::SoftBodyMotionProperties &motion) {
        const ZJoltResult in_range = VertexInRange(motion, index);
        if (in_range != ZJOLT_RESULT_OK) return in_range;
        *out = motion.GetVertex(index).mInvMass;
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodySetVertexInvMass(ZJoltPhysicsSystem *system,
                                          ZJoltBodyId body, uint32_t index,
                                          float inv_mass) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        const ZJoltResult in_range = VertexInRange(motion, index);
        if (in_range != ZJOLT_RESULT_OK) return in_range;
        motion.GetVertex(index).mInvMass = inv_mass;
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodyCalculateMassAndInertia(ZJoltPhysicsSystem *system,
                                                 ZJoltBodyId body) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        motion.CalculateMassAndInertia();
        return ZJOLT_RESULT_OK;
      });
}

//===----------------------------------------------------------------------===//
// Cheap measurements
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodyGetVolume(const ZJoltPhysicsSystem *system,
                                   ZJoltBodyId body, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  // GetVolume walks the settings' faces and indexes the body's vertices with
  // each face's three indices, unguarded. Nothing is checked here because
  // nothing can be: the Add* calls refused an out-of-range face index at the
  // one point it was still possible to refuse it.
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            *out = motion.GetVolume();
                            return ZJOLT_RESULT_OK;
                          });
}

ZJoltResult zjoltSoftBodyGetLocalBounds(const ZJoltPhysicsSystem *system,
                                        ZJoltBodyId body, ZJoltAABox *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            const JPH::AABox &bounds = motion.GetLocalBounds();
                            out->min = zjolt::ToC(bounds.mMin);
                            out->max = zjolt::ToC(bounds.mMax);
                            return ZJOLT_RESULT_OK;
                          });
}

//===----------------------------------------------------------------------===//
// Skinning
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodySkinVertices(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body,
                                      const ZJoltMat44 *joint_matrices,
                                      uint32_t joint_count,
                                      bool hard_skin_all) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (joint_count != 0 && !zjolt::Present(joint_matrices))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  return WithSoftBodyWrite(
      system, body,
      [&](JPH::Body &jolt_body, JPH::SoftBodyMotionProperties &motion) {
        const JoltSettings &settings = *motion.GetSettings();

        // Jolt sizes the per-instance skinning state once, in Initialize, and
        // only when a skinned constraint exists to size it for — then
        // asserts it still matches the settings' vertex count. Both are
        // things a caller can get wrong: settings are shared, so another
        // body's setup code can append vertices to them after this body was
        // created.
        const size_t skin_state_size = settings.mSkinnedConstraints.empty()
                                           ? 0
                                           : motion.GetVertices().size();
        if (skin_state_size != settings.mVertices.size()) {
          return zjolt::SetError(
              ZJOLT_RESULT_INVALID_ARGUMENT,
              settings.mSkinnedConstraints.empty()
                  ? "this body's shared settings carry no skinned "
                    "constraints, so the body has no skinning state to write"
                  : "this body's shared settings gained vertices after the "
                    "body was created, so its skinning state is the wrong "
                    "size; create the body again from the finished settings");
        }

        // Every inverse bind matrix reaches into joint_matrices by joint
        // index, and that is the only thing joint_count is passed for
        // (SoftBodyMotionProperties.cpp:1150).
        for (const JoltSettings::InvBind &inv_bind :
             settings.mInvBindMatrices) {
          if (inv_bind.mJointIndex >= joint_count) {
            return zjolt::SetError(
                ZJOLT_RESULT_INVALID_ARGUMENT,
                "an inverse bind matrix names a joint index that is past the "
                "end of joint_matrices");
          }
        }

        const uint32_t num_skin_matrices =
            static_cast<uint32_t>(settings.mInvBindMatrices.size());
        for (const JoltSettings::Skinned &skinned :
             settings.mSkinnedConstraints) {
          for (const JoltSettings::SkinWeight &w : skinned.mWeights) {
            // Jolt treats the first zero weight as the end of the list and
            // stops there, so an index in a weight past the terminator is
            // never read and is not worth refusing the call over. Stopping
            // at the same place is what keeps this check exact rather than
            // stricter than Jolt (:1170).
            if (w.mWeight == 0.0f) break;
            if (w.mInvBindIndex >= num_skin_matrices) {
              return zjolt::SetError(
                  ZJOLT_RESULT_INVALID_ARGUMENT,
                  "a skin weight names an inverse bind matrix that was never "
                  "added to these shared settings");
            }
          }
        }

        JPH::Array<JPH::Mat44> matrices;
        matrices.reserve(joint_count);
        for (uint32_t i = 0; i < joint_count; ++i)
          matrices.push_back(ToJoltMat44(joint_matrices[i].m));

        // The centre-of-mass transform is read from the body rather than
        // taken as a parameter: Jolt documents it as "value of
        // Body::GetCenterOfMassTransform()", so there is exactly one right
        // answer and the body is already locked here.
        motion.SkinVertices(jolt_body.GetCenterOfMassTransform(),
                            matrices.data(), joint_count, hard_skin_all,
                            *system->temp_allocator);
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodyGetEnableSkinConstraints(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, bool *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            *out = motion.GetEnableSkinConstraints();
                            return ZJOLT_RESULT_OK;
                          });
}

ZJoltResult zjoltSoftBodySetEnableSkinConstraints(ZJoltPhysicsSystem *system,
                                                  ZJoltBodyId body,
                                                  bool enable) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        motion.SetEnableSkinConstraints(enable);
        return ZJOLT_RESULT_OK;
      });
}

ZJoltResult zjoltSoftBodyGetSkinnedMaxDistanceMultiplier(
    const ZJoltPhysicsSystem *system, ZJoltBodyId body, float *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyRead(system, body,
                          [&](const JPH::SoftBodyMotionProperties &motion) {
                            *out = motion.GetSkinnedMaxDistanceMultiplier();
                            return ZJOLT_RESULT_OK;
                          });
}

ZJoltResult zjoltSoftBodySetSkinnedMaxDistanceMultiplier(
    ZJoltPhysicsSystem *system, ZJoltBodyId body, float multiplier) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return WithSoftBodyWrite(
      system, body, [&](JPH::Body &, JPH::SoftBodyMotionProperties &motion) {
        motion.SetSkinnedMaxDistanceMultiplier(multiplier);
        return ZJOLT_RESULT_OK;
      });
}

//===----------------------------------------------------------------------===//
// Rod state
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodyGetRodStates(const ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body,
                                      ZJoltSoftBodyRodState *out_states,
                                      uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  return WithSoftBodyRead(
      system, body, [&](const JPH::SoftBodyMotionProperties &motion) {
        // The rod states are sized from, and stay parallel to, the shared
        // settings' rod list (SoftBodyMotionProperties.cpp:86) — which is
        // also where the vertex pair that identifies each rod lives, since
        // the motion properties keep only rotation and angular velocity.
        const JoltSettings &settings = *motion.GetSettings();
        const uint32_t count =
            static_cast<uint32_t>(settings.mRodStretchShearConstraints.size());
        *out_count = count;
        if (out_states == nullptr) return ZJOLT_RESULT_OK;
        if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

        for (uint32_t i = 0; i < count; ++i) {
          const JoltSettings::RodStretchShear &rod =
              settings.mRodStretchShearConstraints[i];
          out_states[i].vertex[0] = rod.mVertex[0];
          out_states[i].vertex[1] = rod.mVertex[1];
          out_states[i].rotation = zjolt::ToC(motion.GetRodRotation(i));
          out_states[i].angular_velocity =
              zjolt::ToC(motion.GetRodAngularVelocity(i));
        }
        return ZJOLT_RESULT_OK;
      });
}

//===----------------------------------------------------------------------===//
// Hit read-back and manual update
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodyGetFaceIndex(const ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body,
                                      ZJoltSubShapeId sub_shape_id,
                                      uint32_t *out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::BodyLockRead lock(system->system.GetBodyLockInterface(),
                               zjolt::ToJolt(body));
  if (!lock.Succeeded()) return BodyNotFound();
  const JPH::Body &jolt_body = lock.GetBody();
  if (!jolt_body.IsSoftBody()) return NotASoftBody();

  const JPH::SoftBodyMotionProperties &motion =
      *static_cast<const JPH::SoftBodyMotionProperties *>(
          jolt_body.GetMotionPropertiesUnchecked());
  const uint32_t num_faces =
      static_cast<uint32_t>(motion.GetSettings()->mFaces.size());

  // GetSubShapeIDBits works out its width from `num_faces - 1`, which
  // underflows to 0xffffffff for a body with no faces and asks for all 32
  // bits back (SoftBodyShape.cpp:28). Refused here rather than decoded.
  if (num_faces == 0) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "this soft body has no faces, so no sub shape id can name one");
  }

  // Decoded here rather than through SoftBodyShape::GetFaceIndex because
  // that one asserts on a leftover remainder instead of reporting it, and a
  // sub shape id from a different body's shape is exactly what leaves one.
  const JPH::SoftBodyShape &shape =
      static_cast<const JPH::SoftBodyShape &>(*jolt_body.GetShape());
  JPH::SubShapeID remainder;
  const uint32_t face_index = zjolt::ToJoltSubShapeId(sub_shape_id)
                                  .PopID(shape.GetSubShapeIDBits(), remainder);
  if (!remainder.IsEmpty() || face_index >= num_faces) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "sub shape id does not name a face of this soft body; it is the id "
        "of some other shape, or was composed by hand");
  }
  *out = face_index;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodyCustomUpdate(ZJoltPhysicsSystem *system,
                                      ZJoltBodyId body, float delta_time) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  // The sub-step is delta_time divided by the iteration count, so a
  // non-positive delta is a step backwards or an infinite one, and NaN is
  // both. Jolt's own Update refuses this for the system as a whole; nothing
  // refuses it on this path.
  if (!(delta_time > 0.0f)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "delta_time must be greater than zero");
  }

  // Deliberately the NO-LOCK interface — the one entry point here that does
  // not go through WithSoftBodyWrite. CustomUpdate takes body locks of its
  // own, including on this body via BodyInterface::SetPosition. Holding a
  // write lock across it would wait on a mutex this thread already owns —
  // Jolt's body mutexes are not recursive, so the deadlock would be
  // permanent and look like a hang in the physics step, not a bad call here.
  JPH::BodyLockWrite lock(system->system.GetBodyLockInterfaceNoLock(),
                          zjolt::ToJolt(body));
  if (!lock.Succeeded()) return BodyNotFound();
  JPH::Body &jolt_body = lock.GetBody();
  if (!jolt_body.IsSoftBody()) return NotASoftBody();

  static_cast<JPH::SoftBodyMotionProperties *>(
      jolt_body.GetMotionPropertiesUnchecked())
      ->CustomUpdate(delta_time, jolt_body, system->system);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Vertex-vs-shape collision
//===----------------------------------------------------------------------===//

ZJoltResult zjoltShapeCollideSoftBodyVertices(
    const ZJoltShape *shape, const ZJoltMat44 *center_of_mass_transform,
    ZJoltVec3 scale, const ZJoltCollideSoftBodyVertexIterator *vertices,
    uint32_t count, int32_t colliding_shape_index) {
  ZJOLT_ENTER();
  if (!zjolt::Present(shape, center_of_mass_transform, vertices))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  const ZJoltResult fields_ok = VertexIteratorFieldsPresent(*vertices);
  if (fields_ok != ZJOLT_RESULT_OK) return fields_ok;
  if (count == 0) return ZJOLT_RESULT_OK;

  // Vec3 and Plane are gathered/scattered through temporaries (see the
  // plumbing block above); inv_mass, largest_penetration and
  // colliding_shape_index are plain float/int, aliased directly.
  JPH::Array<JPH::Vec3> positions(count);
  JPH::Array<JPH::Plane> planes(count);
  JPH::Array<float> penetrations_before(count);
  for (uint32_t i = 0; i < count; ++i) {
    positions[i] = ReadVec3At(vertices->position, vertices->position_stride, i);
    planes[i] = ReadPlaneAt(vertices->collision_plane,
                            vertices->collision_plane_stride, i);
    penetrations_before[i] = ReadFloatAt(
        vertices->largest_penetration, vertices->largest_penetration_stride, i);
  }

  const JPH::CollideSoftBodyVertexIterator iterator(
      JPH::StridedPtr<const JPH::Vec3>(positions.data(), sizeof(JPH::Vec3)),
      JPH::StridedPtr<const float>(
          reinterpret_cast<const float *>(vertices->inv_mass),
          int(vertices->inv_mass_stride)),
      JPH::StridedPtr<JPH::Plane>(planes.data(), sizeof(JPH::Plane)),
      JPH::StridedPtr<float>(reinterpret_cast<float *>(vertices->largest_penetration),
                             int(vertices->largest_penetration_stride)),
      JPH::StridedPtr<int>(reinterpret_cast<int *>(vertices->colliding_shape_index),
                           int(vertices->colliding_shape_index_stride)));

  zjolt::ToJolt(shape)->CollideSoftBodyVertices(
      zjolt::ToJolt(*center_of_mass_transform), zjolt::ToJolt(scale), iterator,
      count, colliding_shape_index);

  for (uint32_t i = 0; i < count; ++i) {
    const float after = ReadFloatAt(vertices->largest_penetration,
                                    vertices->largest_penetration_stride, i);
    if (after > penetrations_before[i]) {
      WritePlaneAt(vertices->collision_plane, vertices->collision_plane_stride,
                  i, planes[i]);
    }
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCollideSoftBodyVerticesVsTrianglesCreate(
    const ZJoltMat44 *center_of_mass_transform, ZJoltVec3 scale,
    ZJoltCollideSoftBodyVerticesVsTriangles **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(center_of_mass_transform, out))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::CollideSoftBodyVerticesVsTriangles *collider =
      zjolt::New<JPH::CollideSoftBodyVerticesVsTriangles>(
          zjolt::ToJolt(*center_of_mass_transform), zjolt::ToJolt(scale));
  if (collider == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate a soft-body-vs-triangles collider");
  }
  *out = zjolt::ToC(collider);
  return ZJOLT_RESULT_OK;
}

void zjoltCollideSoftBodyVerticesVsTrianglesDestroy(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider) {
  zjolt::Delete(zjolt::ToJolt(collider));
}

void zjoltCollideSoftBodyVerticesVsTrianglesStartVertex(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider,
    const ZJoltCollideSoftBodyVertexIterator *vertex, uint32_t index) {
  if (collider == nullptr || vertex == nullptr || vertex->position == nullptr)
    return;
  const JPH::Vec3 position =
      ReadVec3At(vertex->position, vertex->position_stride, index);
  const JPH::CollideSoftBodyVertexIterator iterator(
      JPH::StridedPtr<const JPH::Vec3>(&position, 0), JPH::StridedPtr<const float>(),
      JPH::StridedPtr<JPH::Plane>(), JPH::StridedPtr<float>(),
      JPH::StridedPtr<int>());
  zjolt::ToJolt(collider)->StartVertex(iterator);
}

void zjoltCollideSoftBodyVerticesVsTrianglesProcessTriangle(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider, ZJoltVec3 v0,
    ZJoltVec3 v1, ZJoltVec3 v2) {
  if (collider == nullptr) return;
  zjolt::ToJolt(collider)->ProcessTriangle(zjolt::ToJolt(v0), zjolt::ToJolt(v1),
                                           zjolt::ToJolt(v2));
}

void zjoltCollideSoftBodyVerticesVsTrianglesFinishVertex(
    ZJoltCollideSoftBodyVerticesVsTriangles *collider,
    const ZJoltCollideSoftBodyVertexIterator *vertex, uint32_t index,
    int32_t colliding_shape_index) {
  if (collider == nullptr || vertex == nullptr) return;
  if (vertex->position == nullptr || vertex->collision_plane == nullptr ||
      vertex->largest_penetration == nullptr ||
      vertex->colliding_shape_index == nullptr) {
    return;
  }

  const JPH::Vec3 position =
      ReadVec3At(vertex->position, vertex->position_stride, index);
  float *largest_penetration = reinterpret_cast<float *>(
      vertex->largest_penetration + size_t(index) * vertex->largest_penetration_stride);
  int *colliding_shape_index_slot = reinterpret_cast<int *>(
      vertex->colliding_shape_index +
      size_t(index) * vertex->colliding_shape_index_stride);
  const float penetration_before = *largest_penetration;
  JPH::Plane plane_storage(JPH::Vec3::sAxisY(), 0.0f);

  const JPH::CollideSoftBodyVertexIterator iterator(
      JPH::StridedPtr<const JPH::Vec3>(&position, 0), JPH::StridedPtr<const float>(),
      JPH::StridedPtr<JPH::Plane>(&plane_storage, 0),
      JPH::StridedPtr<float>(largest_penetration, 0),
      JPH::StridedPtr<int>(colliding_shape_index_slot, 0));

  zjolt::ToJolt(collider)->FinishVertex(iterator, colliding_shape_index);

  if (*largest_penetration > penetration_before) {
    WritePlaneAt(vertex->collision_plane, vertex->collision_plane_stride, index,
                plane_storage);
  }
}

void zjoltSoftBodyVertexMarkCcdContact(ZJoltBodyId body,
                                       const ZJoltPlane *contact_plane,
                                       ZJoltPlane *out_collision_plane,
                                       int32_t *out_colliding_shape_index,
                                       bool *out_has_contact) {
  if (!zjolt::Present(contact_plane, out_collision_plane,
                      out_colliding_shape_index, out_has_contact)) {
    return;
  }
  JPH::SoftBodyVertex vertex{};
  vertex.MarkCCDContact(
      zjolt::ToJolt(body),
      JPH::Plane(zjolt::ToJolt(contact_plane->normal), contact_plane->constant));
  out_collision_plane->normal = zjolt::ToC(vertex.mCollisionPlane.GetNormal());
  out_collision_plane->constant = vertex.mCollisionPlane.GetConstant();
  *out_colliding_shape_index = vertex.mCollidingShapeIndex;
  *out_has_contact = vertex.mHasContact;
}

//===----------------------------------------------------------------------===//
// Contact listener
//===----------------------------------------------------------------------===//

ZJoltResult zjoltSoftBodyManifoldGetVertexContacts(
    const ZJoltSoftBodyManifold *manifold,
    ZJoltSoftBodyVertexContact *out_contacts, uint32_t capacity,
    uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(manifold, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::SoftBodyManifold &jolt_manifold = *zjolt::ToJolt(manifold);
  const JPH::Array<JPH::SoftBodyVertex> &vertices = jolt_manifold.GetVertices();

  // Counted rather than sized from the vertex count: a manifold reports the
  // vertices that TOUCHED something, which for a cloth on a table is a small
  // fraction of them, and sizing a buffer for the whole body to receive a
  // dozen entries is the cost this avoids.
  uint32_t count = 0;
  for (const JPH::SoftBodyVertex &vertex : vertices)
    if (jolt_manifold.HasContact(vertex)) ++count;
  *out_count = count;
  if (out_contacts == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  uint32_t written = 0;
  for (uint32_t i = 0; i < static_cast<uint32_t>(vertices.size()); ++i) {
    const JPH::SoftBodyVertex &vertex = vertices[i];
    if (!jolt_manifold.HasContact(vertex)) continue;
    ZJoltSoftBodyVertexContact &out = out_contacts[written++];
    out.vertex = i;
    out.body = zjolt::ToC(jolt_manifold.GetContactBodyID(vertex));
    out.local_contact_point =
        zjolt::ToC(jolt_manifold.GetLocalContactPoint(vertex));
    out.normal = zjolt::ToC(jolt_manifold.GetContactNormal(vertex));
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodyManifoldGetSensorContacts(
    const ZJoltSoftBodyManifold *manifold, ZJoltBodyId *out_bodies,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(manifold, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::SoftBodyManifold &jolt_manifold = *zjolt::ToJolt(manifold);
  const uint32_t count =
      static_cast<uint32_t>(jolt_manifold.GetNumSensorContacts());
  *out_count = count;
  if (out_bodies == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  for (uint32_t i = 0; i < count; ++i)
    out_bodies[i] = zjolt::ToC(jolt_manifold.GetSensorContactBodyID(i));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltSoftBodySetContactListener(
    ZJoltPhysicsSystem *system,
    const ZJoltSoftBodyContactListener *listener) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (listener == nullptr) {
    system->system.SetSoftBodyContactListener(nullptr);
    zjolt::Delete(system->soft_body_contact_listener);
    system->soft_body_contact_listener = nullptr;
    return ZJOLT_RESULT_OK;
  }

  ZJoltSoftBodyContactListenerAdapter *adapter =
      zjolt::New<ZJoltSoftBodyContactListenerAdapter>(*listener);
  if (adapter == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  // Install first, then free the old one, for the reason
  // zjoltPhysicsSystemSetContactListener gives: the system must never point
  // at a freed adapter, even for an instant.
  system->system.SetSoftBodyContactListener(adapter);
  zjolt::Delete(system->soft_body_contact_listener);
  system->soft_body_contact_listener = adapter;
  return ZJOLT_RESULT_OK;
}

}  // extern "C"

//===----------------------------------------------------------------------===//
// The adapter Jolt actually calls
//
// Outside extern "C": C++ virtual overrides, declared in zjolt_internal.h next to their rigid-body sibling and defined here, the translation unit that knows how a soft-body manifold is projected.
//===----------------------------------------------------------------------===//

JPH::SoftBodyValidateResult
ZJoltSoftBodyContactListenerAdapter::OnSoftBodyContactValidate(
    const JPH::Body &inSoftBody, const JPH::Body &inOtherBody,
    JPH::SoftBodyContactSettings &ioSettings) {
  if (listener_.on_contact_validate == nullptr)
    return JPH::SoftBodyValidateResult::AcceptContact;

  // Copied out and back rather than aliased: ZJoltSoftBodyContactSettings is
  // the ABI's own layout and Jolt's is free to grow a field.
  ZJoltSoftBodyContactSettings settings;
  settings.inv_mass_scale1 = ioSettings.mInvMassScale1;
  settings.inv_mass_scale2 = ioSettings.mInvMassScale2;
  settings.inv_inertia_scale2 = ioSettings.mInvInertiaScale2;
  settings.is_sensor = ioSettings.mIsSensor;

  const ZJoltSoftBodyValidateResult result = listener_.on_contact_validate(
      listener_.user, zjolt::ToC(inSoftBody.GetID()),
      zjolt::ToC(inOtherBody.GetID()), &settings);

  ioSettings.mInvMassScale1 = settings.inv_mass_scale1;
  ioSettings.mInvMassScale2 = settings.inv_mass_scale2;
  ioSettings.mInvInertiaScale2 = settings.inv_inertia_scale2;
  ioSettings.mIsSensor = settings.is_sensor;

  // Anything that is not an explicit rejection accepts, so a host that
  // returns a value this ABI does not define gets the safe half of the
  // decision rather than an unmapped enumerator.
  return result == ZJOLT_SOFT_BODY_VALIDATE_RESULT_REJECT_CONTACT
             ? JPH::SoftBodyValidateResult::RejectContact
             : JPH::SoftBodyValidateResult::AcceptContact;
}

void ZJoltSoftBodyContactListenerAdapter::OnSoftBodyContactAdded(
    const JPH::Body &inSoftBody, const JPH::SoftBodyManifold &inManifold) {
  if (listener_.on_contact_added == nullptr) return;
  listener_.on_contact_added(listener_.user, zjolt::ToC(inSoftBody.GetID()),
                             zjolt::ToC(&inManifold));
}
