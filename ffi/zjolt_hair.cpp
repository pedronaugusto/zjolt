//===----------------------------------------------------------------------===//
// zjolt — hair, and the compute backend it runs on.
//
// Two things live here, and the first exists so the second can.
//
// JPH::ComputeSystem is an abstract interface with three factory methods, and
// the objects those hand back have another twelve virtuals between them.
// Jolt implements the lot three times over — DX12, Vulkan, Metal — and zjolt
// compiles none of those, because a physics package that cannot be built
// without a graphics SDK is not a physics package. So the whole interface is
// projected onto a flat C table (ZJoltComputeInterface) and reflected back into
// Jolt by the bridge classes below: one C++ subclass per Jolt interface, each
// holding an opaque host handle and forwarding to a function pointer.
//
// The reflection is not quite mechanical, in one place that matters. Jolt's own
// queues retain the buffers bound to them until execution finishes — see
// `ComputeQueueCPU::mUsedBuffers` — because a shader reading a buffer whose
// last reference was dropped mid-frame reads freed memory. That is a contract
// on the implementation, not on the caller, so the bridge queue honours it here
// rather than writing it into the C header as a rule every host would have to
// re-derive.
//
// The second thing is hair itself, which needs no bridge: JPH::Hair is an
// ordinary C++ object that happens to do its work through a ComputeSystem.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Compute/ComputeBuffer.h>
#include <Jolt/Compute/ComputeQueue.h>
#include <Jolt/Compute/ComputeShader.h>
#include <Jolt/Compute/ComputeSystem.h>
#include <Jolt/Geometry/IndexedTriangle.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Hair/Hair.h>
#include <Jolt/Physics/Hair/HairSettings.h>
#include <Jolt/Physics/Hair/HairShaders.h>

#include <cstdio>

#ifdef JPH_USE_CPU_COMPUTE
#include <Jolt/Compute/CPU/ComputeSystemCPU.h>
#include <Jolt/Shaders/HairWrapper.h>
#endif

//===----------------------------------------------------------------------===//
// The bridge: a host's C table, seen by Jolt as a ComputeSystem
//===----------------------------------------------------------------------===//

namespace {

/// Jolt's buffer kinds, projected onto the ABI's and back. Local rather than in
/// zjolt_internal.h because nothing outside this translation unit has a compute
/// buffer to convert.
ZJoltComputeBufferType ToC(JPH::ComputeBuffer::EType type) {
  switch (type) {
    case JPH::ComputeBuffer::EType::UploadBuffer:
      return ZJOLT_COMPUTE_BUFFER_TYPE_UPLOAD;
    case JPH::ComputeBuffer::EType::ReadbackBuffer:
      return ZJOLT_COMPUTE_BUFFER_TYPE_READBACK;
    case JPH::ComputeBuffer::EType::ConstantBuffer:
      return ZJOLT_COMPUTE_BUFFER_TYPE_CONSTANT;
    case JPH::ComputeBuffer::EType::Buffer:
      return ZJOLT_COMPUTE_BUFFER_TYPE_READ_ONLY;
    case JPH::ComputeBuffer::EType::RWBuffer:
      break;
  }
  return ZJOLT_COMPUTE_BUFFER_TYPE_READ_WRITE;
}

ZJoltComputeMapMode ToC(JPH::ComputeBuffer::EMode mode) {
  return mode == JPH::ComputeBuffer::EMode::Read ? ZJOLT_COMPUTE_MAP_MODE_READ
                                                 : ZJOLT_COMPUTE_MAP_MODE_WRITE;
}

ZJoltComputeBarrier ToC(JPH::ComputeQueue::EBarrier barrier) {
  return barrier == JPH::ComputeQueue::EBarrier::Yes
             ? ZJOLT_COMPUTE_BARRIER_INSERT
             : ZJOLT_COMPUTE_BARRIER_SKIP;
}

class BridgeSystem;

/// The two things every bridge object needs: the table to call, and a reference
/// on the system that owns the table.
///
/// The reference is not decoration. Jolt hands buffers out as `Ref`s and Hair
/// keeps them for its whole life, so a host that destroys its
/// ZJoltComputeSystem handle while a hair is still alive would otherwise leave
/// every buffer forwarding into a freed ZJoltComputeInterface. Holding the
/// system from below makes the destroy order of the two handles a non-question.
struct BridgeLink {
  JPH::Ref<JPH::ComputeSystem> keep_alive;
  BridgeSystem *owner = nullptr;

  const ZJoltComputeInterface &Table() const;
};

class BridgeSystem final : public JPH::ComputeSystem {
 public:
  explicit BridgeSystem(const ZJoltComputeInterface &iface) : iface_(iface) {}

  ~BridgeSystem() override {
    if (iface_.destroy != nullptr) iface_.destroy(iface_.user);
  }

  const ZJoltComputeInterface &Table() const { return iface_; }

  JPH::ComputeShaderResult CreateComputeShader(const char *inName,
                                               JPH::uint32 inGroupSizeX,
                                               JPH::uint32 inGroupSizeY,
                                               JPH::uint32 inGroupSizeZ) override;
  JPH::ComputeBufferResult CreateComputeBuffer(JPH::ComputeBuffer::EType inType,
                                               JPH::uint64 inSize,
                                               JPH::uint inStride,
                                               const void *inData) override;
  JPH::ComputeQueueResult CreateComputeQueue() override;

 private:
  ZJoltComputeInterface iface_;
};

const ZJoltComputeInterface &BridgeLink::Table() const {
  return owner->Table();
}

class BridgeShader final : public JPH::ComputeShader {
 public:
  BridgeShader(BridgeSystem *owner, void *host, JPH::uint32 x, JPH::uint32 y,
               JPH::uint32 z)
      : JPH::ComputeShader(x, y, z), link_{owner, owner}, host_(host) {}

  ~BridgeShader() override {
    const ZJoltComputeInterface &table = link_.Table();
    table.destroy_shader(table.user, host_);
  }

  void *Host() const { return host_; }

 private:
  BridgeLink link_;
  void *host_;
};

class BridgeBuffer final : public JPH::ComputeBuffer {
 public:
  BridgeBuffer(BridgeSystem *owner, void *host, EType type, JPH::uint64 size,
               JPH::uint stride)
      : JPH::ComputeBuffer(type, size, stride), link_{owner, owner}, host_(host) {}

  ~BridgeBuffer() override {
    const ZJoltComputeInterface &table = link_.Table();
    table.destroy_buffer(table.user, host_);
  }

  void *Host() const { return host_; }

  JPH::ComputeBufferResult CreateReadBackBuffer() const override;

 protected:
  void *MapInternal(EMode inMode) override {
    const ZJoltComputeInterface &table = link_.Table();
    return table.map_buffer(table.user, host_, ToC(inMode));
  }

  void UnmapInternal() override {
    const ZJoltComputeInterface &table = link_.Table();
    table.unmap_buffer(table.user, host_);
  }

 private:
  BridgeLink link_;
  void *host_;
};

/// The host handle behind a Jolt compute buffer, or NULL for a NULL buffer.
///
/// Every ComputeBuffer that reaches this queue came out of this system's
/// CreateComputeBuffer, so the downcast is sound: there is no other way to make
/// one, and Jolt never mixes buffers between systems.
void *HostOf(const JPH::ComputeBuffer *buffer) {
  if (buffer == nullptr) return nullptr;
  return static_cast<const BridgeBuffer *>(buffer)->Host();
}

class BridgeQueue final : public JPH::ComputeQueue {
 public:
  BridgeQueue(BridgeSystem *owner, void *host)
      : link_{owner, owner}, host_(host) {}

  ~BridgeQueue() override {
    const ZJoltComputeInterface &table = link_.Table();
    table.destroy_queue(table.user, host_);
  }

  void SetShader(const JPH::ComputeShader *inShader) override {
    const ZJoltComputeInterface &table = link_.Table();
    shader_ = inShader;
    void *host = inShader != nullptr
                     ? static_cast<const BridgeShader *>(inShader)->Host()
                     : nullptr;
    table.queue_set_shader(table.user, host_, host);
  }

  void SetConstantBuffer(const char *inName,
                         const JPH::ComputeBuffer *inBuffer) override {
    const ZJoltComputeInterface &table = link_.Table();
    Retain(inBuffer);
    table.queue_set_constant_buffer(table.user, host_, inName, HostOf(inBuffer));
  }

  void SetBuffer(const char *inName,
                 const JPH::ComputeBuffer *inBuffer) override {
    const ZJoltComputeInterface &table = link_.Table();
    Retain(inBuffer);
    table.queue_set_buffer(table.user, host_, inName, HostOf(inBuffer));
  }

  void SetRWBuffer(const char *inName, JPH::ComputeBuffer *inBuffer,
                   EBarrier inBarrier) override {
    const ZJoltComputeInterface &table = link_.Table();
    Retain(inBuffer);
    table.queue_set_rw_buffer(table.user, host_, inName, HostOf(inBuffer),
                              ToC(inBarrier));
  }

  void Dispatch(JPH::uint inThreadGroupsX, JPH::uint inThreadGroupsY,
                JPH::uint inThreadGroupsZ) override {
    const ZJoltComputeInterface &table = link_.Table();
    table.queue_dispatch(table.user, host_,
                         static_cast<uint32_t>(inThreadGroupsX),
                         static_cast<uint32_t>(inThreadGroupsY),
                         static_cast<uint32_t>(inThreadGroupsZ));
  }

  void ScheduleReadback(JPH::ComputeBuffer *inDst,
                        const JPH::ComputeBuffer *inSrc) override {
    const ZJoltComputeInterface &table = link_.Table();
    Retain(inDst);
    Retain(inSrc);
    table.queue_schedule_readback(table.user, host_, HostOf(inDst),
                                  HostOf(inSrc));
  }

  void Execute() override {
    const ZJoltComputeInterface &table = link_.Table();
    table.queue_execute(table.user, host_);
  }

  /// Releases the retained resources once the work that used them has run.
  /// This is the other half of the contract in Retain, and doing it here rather
  /// than in Execute is the point: between the two, the device is still reading
  /// them.
  void Wait() override {
    const ZJoltComputeInterface &table = link_.Table();
    table.queue_wait(table.user, host_);
    bound_.clear();
    shader_ = nullptr;
  }

 private:
  /// Keeps a bound resource alive until Wait. Jolt's own backends do this and
  /// its header states it as a property of binding ("A reference to the buffer
  /// is added to make sure it stays alive until execution finishes"), so it
  /// belongs on this side of the boundary, not in the host's table.
  void Retain(const JPH::ComputeBuffer *buffer) {
    if (buffer != nullptr) bound_.push_back(buffer);
  }

  BridgeLink link_;
  void *host_;
  JPH::RefConst<JPH::ComputeShader> shader_;
  JPH::Array<JPH::RefConst<JPH::ComputeBuffer>> bound_;
};

JPH::ComputeShaderResult BridgeSystem::CreateComputeShader(
    const char *inName, JPH::uint32 inGroupSizeX, JPH::uint32 inGroupSizeY,
    JPH::uint32 inGroupSizeZ) {
  JPH::ComputeShaderResult result;

  void *host = nullptr;
  if (iface_.create_shader(iface_.user, inName, inGroupSizeX, inGroupSizeY,
                           inGroupSizeZ, &host) != ZJOLT_RESULT_OK ||
      host == nullptr) {
    result.SetError("the compute backend could not create this shader");
    return result;
  }

  BridgeShader *shader = zjolt::New<BridgeShader>(this, host, inGroupSizeX,
                                                  inGroupSizeY, inGroupSizeZ);
  if (shader == nullptr) {
    iface_.destroy_shader(iface_.user, host);
    result.SetError("out of memory wrapping a compute shader");
    return result;
  }
  result.Set(shader);
  return result;
}

JPH::ComputeBufferResult BridgeSystem::CreateComputeBuffer(
    JPH::ComputeBuffer::EType inType, JPH::uint64 inSize, JPH::uint inStride,
    const void *inData) {
  JPH::ComputeBufferResult result;

  void *host = nullptr;
  if (iface_.create_buffer(iface_.user, ToC(inType), inSize,
                           static_cast<uint32_t>(inStride), inData,
                           &host) != ZJOLT_RESULT_OK ||
      host == nullptr) {
    result.SetError("the compute backend could not create this buffer");
    return result;
  }

  BridgeBuffer *buffer =
      zjolt::New<BridgeBuffer>(this, host, inType, inSize, inStride);
  if (buffer == nullptr) {
    iface_.destroy_buffer(iface_.user, host);
    result.SetError("out of memory wrapping a compute buffer");
    return result;
  }
  result.Set(buffer);
  return result;
}

JPH::ComputeQueueResult BridgeSystem::CreateComputeQueue() {
  JPH::ComputeQueueResult result;

  void *host = nullptr;
  if (iface_.create_queue(iface_.user, &host) != ZJOLT_RESULT_OK ||
      host == nullptr) {
    result.SetError("the compute backend could not create a queue");
    return result;
  }

  BridgeQueue *queue = zjolt::New<BridgeQueue>(this, host);
  if (queue == nullptr) {
    iface_.destroy_queue(iface_.user, host);
    result.SetError("out of memory wrapping a compute queue");
    return result;
  }
  result.Set(queue);
  return result;
}

JPH::ComputeBufferResult BridgeBuffer::CreateReadBackBuffer() const {
  JPH::ComputeBufferResult result;
  const ZJoltComputeInterface &table = link_.Table();

  void *host = nullptr;
  const ZJoltResult created =
      table.create_readback_buffer != nullptr
          ? table.create_readback_buffer(table.user, host_, &host)
          : table.create_buffer(table.user, ZJOLT_COMPUTE_BUFFER_TYPE_READBACK,
                                GetSize(), static_cast<uint32_t>(GetStride()),
                                nullptr, &host);
  if (created != ZJOLT_RESULT_OK || host == nullptr) {
    result.SetError("the compute backend could not create a readback buffer");
    return result;
  }

  BridgeBuffer *buffer = zjolt::New<BridgeBuffer>(
      link_.owner, host, EType::ReadbackBuffer, GetSize(), GetStride());
  if (buffer == nullptr) {
    table.destroy_buffer(table.user, host);
    result.SetError("out of memory wrapping a readback buffer");
    return result;
  }
  result.Set(buffer);
  return result;
}

}  // namespace

//===----------------------------------------------------------------------===//
// The handles
//
// Global namespace, because the C header names them as opaque handles and the
// tags have to match.
//===----------------------------------------------------------------------===//

/// Everything a hair needs from a backend: the system, its one queue, and the
/// fifteen compiled shaders.
///
/// Reference counted rather than owned by the ZJoltComputeSystem handle so that
/// destroying that handle while a hair is still using it is legal. Jolt's own
/// reference counting does the work; this exists because the three pieces are
/// created and discarded together and a hair should hold one reference, not
/// three.
class ZJoltComputeContext : public JPH::RefTarget<ZJoltComputeContext> {
 public:
  JPH_OVERRIDE_NEW_DELETE

  JPH::Ref<JPH::ComputeSystem> system;
  JPH::Ref<JPH::ComputeQueue> queue;
  JPH::HairShaders shaders;
};

struct ZJoltComputeSystem {
  JPH::Ref<ZJoltComputeContext> context;
};

struct ZJoltHair {
  JPH::Ref<ZJoltComputeContext> context;
  JPH::Ref<JPH::HairSettings> settings;

  /// Owned outright. JPH::Hair is not reference counted — it is a plain
  /// NonCopyable — so this is a zjolt::New / zjolt::Delete pair, not an Own.
  JPH::Hair *hair = nullptr;

  /// The pose zjoltHairSetPose recorded, replayed into every update. Empty when
  /// the groom has no scalp, which is what tells Update to skip skinning.
  JPH::Mat44 joint_to_hair = JPH::Mat44::sIdentity();
  JPH::Array<JPH::Mat44> joint_matrices;

  /// The transform last handed to JPH::Hair. Kept here because Jolt's Hair
  /// takes one and never gives it back — it has SetPosition/SetRotation and
  /// no getters — and a caller needs it: every vertex zjoltHairGetVertices
  /// reports is in the hair's LOCAL space, so this is the matrix that puts
  /// them where they are drawn.
  JPH::RVec3 position = JPH::RVec3::sZero();
  JPH::Quat rotation = JPH::Quat::sIdentity();
};

namespace {

//===----------------------------------------------------------------------===//
// Shared helpers
//===----------------------------------------------------------------------===//

/// A 4x4 matrix from sixteen floats in column-major order, which is how Jolt
/// stores one and therefore the only order that does not need a transpose at
/// the boundary.
JPH::Mat44 LoadMat44(const float *m) {
  return JPH::Mat44(JPH::Vec4(m[0], m[1], m[2], m[3]),
                    JPH::Vec4(m[4], m[5], m[6], m[7]),
                    JPH::Vec4(m[8], m[9], m[10], m[11]),
                    JPH::Vec4(m[12], m[13], m[14], m[15]));
}

ZJoltHairGradient ToC(const JPH::HairSettings::Gradient &g) {
  return ZJoltHairGradient{g.mMin, g.mMax, g.mMinFraction, g.mMaxFraction};
}

JPH::HairSettings::Gradient ToJolt(const ZJoltHairGradient &g) {
  return JPH::HairSettings::Gradient(g.min, g.max, g.min_fraction,
                                     g.max_fraction);
}

ZJoltHairMaterial ToC(const JPH::HairSettings::Material &m) {
  ZJoltHairMaterial out{};
  out.enable_collision = m.mEnableCollision;
  out.enable_lra = m.mEnableLRA;
  out.linear_damping = m.mLinearDamping;
  out.angular_damping = m.mAngularDamping;
  out.max_linear_velocity = m.mMaxLinearVelocity;
  out.max_angular_velocity = m.mMaxAngularVelocity;
  out.gravity_factor = ToC(m.mGravityFactor);
  out.friction = m.mFriction;
  out.bend_compliance = m.mBendCompliance;
  out.bend_compliance_multiplier[0] = m.mBendComplianceMultiplier.x;
  out.bend_compliance_multiplier[1] = m.mBendComplianceMultiplier.y;
  out.bend_compliance_multiplier[2] = m.mBendComplianceMultiplier.z;
  out.bend_compliance_multiplier[3] = m.mBendComplianceMultiplier.w;
  out.stretch_compliance = m.mStretchCompliance;
  out.inertia_multiplier = m.mInertiaMultiplier;
  out.hair_radius = ToC(m.mHairRadius);
  out.world_transform_influence = ToC(m.mWorldTransformInfluence);
  out.grid_velocity_factor = ToC(m.mGridVelocityFactor);
  out.grid_density_force_factor = m.mGridDensityForceFactor;
  out.global_pose = ToC(m.mGlobalPose);
  out.skin_global_pose = ToC(m.mSkinGlobalPose);
  out.simulation_strands_fraction = m.mSimulationStrandsFraction;
  out.gravity_preload_factor = m.mGravityPreloadFactor;
  return out;
}

JPH::HairSettings::Material ToJolt(const ZJoltHairMaterial &m) {
  JPH::HairSettings::Material out;
  out.mEnableCollision = m.enable_collision;
  out.mEnableLRA = m.enable_lra;
  out.mLinearDamping = m.linear_damping;
  out.mAngularDamping = m.angular_damping;
  out.mMaxLinearVelocity = m.max_linear_velocity;
  out.mMaxAngularVelocity = m.max_angular_velocity;
  out.mGravityFactor = ToJolt(m.gravity_factor);
  out.mFriction = m.friction;
  out.mBendCompliance = m.bend_compliance;
  out.mBendComplianceMultiplier =
      JPH::Float4(m.bend_compliance_multiplier[0],
                  m.bend_compliance_multiplier[1],
                  m.bend_compliance_multiplier[2],
                  m.bend_compliance_multiplier[3]);
  out.mStretchCompliance = m.stretch_compliance;
  out.mInertiaMultiplier = m.inertia_multiplier;
  out.mHairRadius = ToJolt(m.hair_radius);
  out.mWorldTransformInfluence = ToJolt(m.world_transform_influence);
  out.mGridVelocityFactor = ToJolt(m.grid_velocity_factor);
  out.mGridDensityForceFactor = m.grid_density_force_factor;
  out.mGlobalPose = ToJolt(m.global_pose);
  out.mSkinGlobalPose = ToJolt(m.skin_global_pose);
  out.mSimulationStrandsFraction = m.simulation_strands_fraction;
  out.mGravityPreloadFactor = m.gravity_preload_factor;
  return out;
}

/// The name of the first hair shader the backend failed to produce, or NULL.
///
/// HairShaders::Init stores a null Ref for a shader it could not load and says
/// nothing; the first null one is dereferenced deep inside Hair::Update, a
/// frame later and several call levels down. Checking the whole set once, here,
/// is what turns that into a refused zjoltComputeSystemCreate.
const char *FirstMissingShader(const JPH::HairShaders &shaders) {
  struct Named {
    const JPH::Ref<JPH::ComputeShader> &shader;
    const char *name;
  };
  const Named all[] = {
      {shaders.mTeleportCS, "HairTeleport"},
      {shaders.mApplyDeltaTransformCS, "HairApplyDeltaTransform"},
      {shaders.mSkinVerticesCS, "HairSkinVertices"},
      {shaders.mSkinRootsCS, "HairSkinRoots"},
      {shaders.mApplyGlobalPoseCS, "HairApplyGlobalPose"},
      {shaders.mCalculateCollisionPlanesCS, "HairCalculateCollisionPlanes"},
      {shaders.mGridClearCS, "HairGridClear"},
      {shaders.mGridAccumulateCS, "HairGridAccumulate"},
      {shaders.mGridNormalizeCS, "HairGridNormalize"},
      {shaders.mIntegrateCS, "HairIntegrate"},
      {shaders.mUpdateRootsCS, "HairUpdateRoots"},
      {shaders.mUpdateStrandsCS, "HairUpdateStrands"},
      {shaders.mUpdateVelocityCS, "HairUpdateVelocity"},
      {shaders.mUpdateVelocityIntegrateCS, "HairUpdateVelocityIntegrate"},
      {shaders.mCalculateRenderPositionsCS, "HairCalculateRenderPositions"},
  };
  for (const Named &named : all) {
    if (named.shader == nullptr) return named.name;
  }
  return nullptr;
}

/// Everything the two compute-system constructors share: one queue, the hair
/// shaders, and the handle.
///
/// `system` arrives already held by the caller's Ref, so a failure here
/// destroys it on the way out rather than leaking it.
ZJoltResult FinishComputeSystem(JPH::ComputeSystem *system,
                                ZJoltComputeSystem **out) {
  ZJoltComputeContext *context = zjolt::New<ZJoltComputeContext>();
  if (context == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  // A fresh RefTarget starts at zero; this Ref is the first reference, and it
  // drops the context again if anything below fails.
  const JPH::Ref<ZJoltComputeContext> holder = context;

  context->system = system;

  JPH::ComputeQueueResult queue = system->CreateComputeQueue();
  if (!queue.IsValid()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           queue.GetError().c_str());
  }
  context->queue = queue.Get();

  context->shaders.Init(system);
  if (const char *missing = FirstMissingShader(context->shaders)) {
    // The shader's name is the useful half of the message, so it goes in it.
    char detail[160];
    std::snprintf(detail, sizeof(detail),
                  "the compute backend did not provide the hair shader '%s'",
                  missing);
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, detail);
  }

  ZJoltComputeSystem *handle = zjolt::New<ZJoltComputeSystem>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  handle->context = context;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

/// Whether the table can answer everything Jolt will ask of it. Only
/// `create_readback_buffer` and `destroy` are optional, and both have a
/// documented fallback.
///
/// Spelled out rather than run through zjolt::Present, which takes pointers to
/// objects: these are pointers to functions, and the two do not share a type.
bool TableIsComplete(const ZJoltComputeInterface &iface) {
  return iface.create_shader != nullptr && iface.create_buffer != nullptr &&
         iface.create_queue != nullptr && iface.destroy_shader != nullptr &&
         iface.destroy_buffer != nullptr && iface.destroy_queue != nullptr &&
         iface.map_buffer != nullptr && iface.unmap_buffer != nullptr &&
         iface.queue_set_shader != nullptr &&
         iface.queue_set_constant_buffer != nullptr &&
         iface.queue_set_buffer != nullptr &&
         iface.queue_set_rw_buffer != nullptr &&
         iface.queue_dispatch != nullptr &&
         iface.queue_schedule_readback != nullptr &&
         iface.queue_execute != nullptr && iface.queue_wait != nullptr;
}

//===----------------------------------------------------------------------===//
// Groom validation
//
// Every check here stands in for something Jolt does not check. The asserts it
// does have fire in HairSettings::Init, several thousand vertices into a build
// that has already allocated; in a build without asserts the same inputs divide
// by zero or index past an array.
//===----------------------------------------------------------------------===//

ZJoltResult ValidateGroom(const ZJoltHairDesc *desc, uint32_t material_count) {
  if (desc->vertex_count < 2 || desc->strand_count == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a groom needs at least one strand of at least two "
                           "vertices");
  }

  // InitCompute packs a strand's material index into a uint8
  // (`HairSettings.cpp:676`). Past 255 it truncates, which does not fail — it
  // quietly gives the strand a different material.
  if (material_count > 256) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a groom may have at most 256 materials; Jolt packs "
                           "a strand's material index into a byte");
  }

  for (uint32_t s = 0; s < desc->strand_count; ++s) {
    const ZJoltHairStrand &strand = desc->strands[s];
    if (strand.end_vertex > desc->vertex_count ||
        strand.start_vertex >= strand.end_vertex) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a strand's vertex range is empty or runs past "
                             "the end of the vertex array");
    }
    if (strand.end_vertex - strand.start_vertex < 2) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a strand needs at least two vertices to have a "
                             "direction");
    }
    // The same byte-packing as the material index above, for the strand's
    // vertex count (`HairSettings.cpp:666`). A 256-vertex strand becomes a
    // zero-vertex one, silently.
    if (strand.end_vertex - strand.start_vertex > 255) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a strand may have at most 255 vertices; Jolt "
                             "packs a strand's vertex count into a byte");
    }
    if (strand.material_index >= material_count) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a strand names a material index that does not "
                             "exist");
    }
    // "Rods of zero length are not supported!" (HairSettings.cpp:510,532).
    // Jolt divides by the rod length to build the Bishop frame, so two
    // coincident vertices are NaN in every position from then on.
    for (uint32_t v = strand.start_vertex; v + 1 < strand.end_vertex; ++v) {
      const JPH::Vec3 a = zjolt::ToJolt(desc->vertices[v].position);
      const JPH::Vec3 b = zjolt::ToJolt(desc->vertices[v + 1].position);
      if ((b - a).LengthSq() <= 0.0f) {
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "two consecutive vertices of a strand are in "
                               "the same place, which Jolt cannot orient");
      }
    }
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult ValidateScalp(const ZJoltHairDesc *desc, bool *out_has_scalp) {
  const bool any = desc->scalp_vertices != nullptr ||
                   desc->scalp_triangles != nullptr ||
                   desc->scalp_skin_weights != nullptr ||
                   desc->scalp_inverse_bind_pose != nullptr;
  const bool all = desc->scalp_vertices != nullptr &&
                   desc->scalp_triangles != nullptr &&
                   desc->scalp_skin_weights != nullptr &&
                   desc->scalp_inverse_bind_pose != nullptr;
  *out_has_scalp = all;
  if (!any) return ZJOLT_RESULT_OK;
  if (!all) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a scalp needs vertices, triangles, skin weights "
                           "and an inverse bind pose together, or none of them");
  }
  if (desc->scalp_vertex_count == 0 || desc->scalp_triangle_count == 0 ||
      desc->skin_weights_per_vertex == 0 || desc->joint_count == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a scalp with a null count in it cannot skin "
                           "anything");
  }

  for (uint32_t i = 0; i < desc->scalp_triangle_count * 3; ++i) {
    if (desc->scalp_triangles[i] >= desc->scalp_vertex_count) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a scalp triangle names a vertex that does not "
                             "exist");
    }
  }
  const uint64_t weights =
      static_cast<uint64_t>(desc->scalp_vertex_count) *
      static_cast<uint64_t>(desc->skin_weights_per_vertex);
  for (uint64_t i = 0; i < weights; ++i) {
    if (desc->scalp_skin_weights[i].joint_index >= desc->joint_count) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "a scalp skin weight names a joint that does not "
                             "exist");
    }
  }
  return ZJOLT_RESULT_OK;
}

/// One readback, shared by the simulated and the render positions.
ZJoltResult ReadBackPositions(ZJoltHair *hair, bool render,
                              ZJoltVec3 *out_positions, uint32_t capacity,
                              uint32_t *out_count) {
  const JPH::HairSettings *settings = hair->hair->GetHairSettings();
  const uint32_t count = static_cast<uint32_t>(
      render ? settings->mRenderVertices.size() : settings->mSimVertices.size());
  *out_count = count;
  if (out_positions == nullptr) return ZJOLT_RESULT_OK;

  hair->hair->ReadBackGPUState(hair->context->queue);
  hair->hair->LockReadBackBuffers();
  const JPH::Float3 *source =
      render ? hair->hair->GetRenderPositions() : hair->hair->GetPositions();
  const uint32_t n = capacity < count ? capacity : count;
  if (source != nullptr) {
    for (uint32_t i = 0; i < n; ++i) {
      out_positions[i] = ZJoltVec3{source[i].x, source[i].y, source[i].z};
    }
  }
  hair->hair->UnlockReadBackBuffers();

  return capacity < count ? ZJOLT_RESULT_BUFFER_TOO_SMALL : ZJOLT_RESULT_OK;
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Compute backends
//===----------------------------------------------------------------------===//

bool zjoltComputeIsCpuSupported(void) {
#ifdef JPH_USE_CPU_COMPUTE
  return true;
#else
  return false;
#endif
}

ZJoltResult zjoltComputeSystemCreateCpu(ZJoltComputeSystem **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

#ifdef JPH_USE_CPU_COMPUTE
  JPH::ComputeSystemCPU *cpu = zjolt::New<JPH::ComputeSystemCPU>();
  if (cpu == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  const JPH::Ref<JPH::ComputeSystem> holder = cpu;

  // The CPU backend compiles nothing: its shaders are C++ translation units
  // that register themselves by name, and a system with none registered
  // reports every shader as missing.
  JPH::HairRegisterShaders(cpu);

  return FinishComputeSystem(cpu, out);
#else
  return zjolt::SetError(
      ZJOLT_RESULT_UNSUPPORTED,
      "this build has Jolt's CPU compute path compiled out; supply a "
      "ZJoltComputeInterface instead, or build with -Dcpu_compute=true");
#endif
}

ZJoltResult zjoltComputeSystemCreate(const ZJoltComputeInterface *iface,
                                     ZJoltComputeSystem **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(iface, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!TableIsComplete(*iface)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "the compute interface is missing a required callback; only "
        "create_readback_buffer and destroy may be NULL");
  }

  BridgeSystem *bridge = zjolt::New<BridgeSystem>(*iface);
  if (bridge == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  const JPH::Ref<JPH::ComputeSystem> holder = bridge;

  return FinishComputeSystem(bridge, out);
}

void zjoltComputeSystemDestroy(ZJoltComputeSystem *compute) {
  if (compute == nullptr) return;
  compute->context = nullptr;
  zjolt::Delete(compute);
  zjolt::HandleDestroyed();
}

//===----------------------------------------------------------------------===//
// Grooms
//===----------------------------------------------------------------------===//

void zjoltHairMaterialInit(ZJoltHairMaterial *out) {
  if (out == nullptr) return;
  const JPH::HairSettings::Material defaults;
  *out = ToC(defaults);
}

ZJoltResult zjoltHairCreate(ZJoltComputeSystem *compute,
                            const ZJoltHairDesc *desc, ZJoltHair **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(compute, desc, out)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  if (!zjolt::Present(desc->vertices, desc->strands)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a groom needs both a vertex array and a strand "
                           "array");
  }

  const bool have_materials =
      desc->materials != nullptr && desc->material_count > 0;
  const uint32_t material_count =
      have_materials ? desc->material_count : 1;

  const ZJoltResult groom_ok = ValidateGroom(desc, material_count);
  if (groom_ok != ZJOLT_RESULT_OK) return groom_ok;

  bool has_scalp = false;
  const ZJoltResult scalp_ok = ValidateScalp(desc, &has_scalp);
  if (scalp_ok != ZJOLT_RESULT_OK) return scalp_ok;

  JPH::HairSettings *raw = zjolt::New<JPH::HairSettings>();
  if (raw == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  const JPH::Ref<JPH::HairSettings> settings = raw;

  // Materials first: InitRenderAndSimulationStrands reads
  // mSimulationStrandsFraction out of them to decide what to simulate.
  settings->mMaterials.reserve(material_count);
  for (uint32_t i = 0; i < material_count; ++i) {
    settings->mMaterials.push_back(have_materials
                                       ? ToJolt(desc->materials[i])
                                       : JPH::HairSettings::Material());
  }

  if (has_scalp) {
    settings->mScalpVertices.reserve(desc->scalp_vertex_count);
    for (uint32_t i = 0; i < desc->scalp_vertex_count; ++i) {
      const ZJoltVec3 &v = desc->scalp_vertices[i];
      settings->mScalpVertices.push_back(JPH::Float3(v.x, v.y, v.z));
    }
    settings->mScalpTriangles.reserve(desc->scalp_triangle_count);
    for (uint32_t i = 0; i < desc->scalp_triangle_count; ++i) {
      settings->mScalpTriangles.push_back(JPH::IndexedTriangleNoMaterial(
          desc->scalp_triangles[3 * i + 0], desc->scalp_triangles[3 * i + 1],
          desc->scalp_triangles[3 * i + 2]));
    }
    settings->mScalpInverseBindPose.reserve(desc->joint_count);
    for (uint32_t i = 0; i < desc->joint_count; ++i) {
      settings->mScalpInverseBindPose.push_back(
          LoadMat44(desc->scalp_inverse_bind_pose + 16 * i));
    }
    const uint64_t weights =
        static_cast<uint64_t>(desc->scalp_vertex_count) *
        static_cast<uint64_t>(desc->skin_weights_per_vertex);
    settings->mScalpSkinWeights.reserve(static_cast<size_t>(weights));
    for (uint64_t i = 0; i < weights; ++i) {
      JPH::HairSettings::SkinWeight weight;
      weight.mJointIdx = desc->scalp_skin_weights[i].joint_index;
      weight.mWeight = desc->scalp_skin_weights[i].weight;
      settings->mScalpSkinWeights.push_back(weight);
    }
    settings->mScalpNumSkinWeightsPerVertex = desc->skin_weights_per_vertex;
  }

  if (desc->iterations_per_second != 0) {
    settings->mNumIterationsPerSecond = desc->iterations_per_second;
  }
  if (desc->max_delta_time > 0.0f) {
    settings->mMaxDeltaTime = desc->max_delta_time;
  }
  if (desc->grid_size_x != 0 && desc->grid_size_y != 0 &&
      desc->grid_size_z != 0) {
    settings->mGridSize =
        JPH::UVec4(desc->grid_size_x, desc->grid_size_y, desc->grid_size_z, 0);
  }
  settings->mInitialGravity = zjolt::ToJolt(desc->initial_gravity);

  // An all-zero padding is taken as "unset" rather than as zero. The grid is
  // scaled by gridSize / boundsExtent, and the neutral pose of a single
  // straight strand has zero extent on two axes — so a literal zero here is a
  // division by zero and a groom full of NaN, not a tight fit.
  const JPH::Vec3 padding = zjolt::ToJolt(desc->simulation_bounds_padding);
  if (padding != JPH::Vec3::sZero()) {
    settings->mSimulationBoundsPadding = padding;
  }

  JPH::Array<JPH::HairSettings::SVertex> vertices;
  vertices.reserve(desc->vertex_count);
  for (uint32_t i = 0; i < desc->vertex_count; ++i) {
    const ZJoltHairVertex &v = desc->vertices[i];
    vertices.push_back(JPH::HairSettings::SVertex(
        JPH::Float3(v.position.x, v.position.y, v.position.z), v.inv_mass));
  }

  JPH::Array<JPH::HairSettings::SStrand> strands;
  strands.reserve(desc->strand_count);
  for (uint32_t i = 0; i < desc->strand_count; ++i) {
    const ZJoltHairStrand &s = desc->strands[i];
    strands.push_back(JPH::HairSettings::SStrand(s.start_vertex, s.end_vertex,
                                                 s.material_index));
  }

  settings->InitRenderAndSimulationStrands(vertices, strands);

  float max_dist_sq_hair_to_scalp = 0.0f;
  settings->Init(max_dist_sq_hair_to_scalp);
  settings->InitCompute(compute->context->system);

  ZJoltHair *handle = zjolt::New<ZJoltHair>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  JPH::Hair *hair = zjolt::New<JPH::Hair>(
      static_cast<const JPH::HairSettings *>(settings),
      zjolt::ToJoltR(desc->position), zjolt::ToJoltRotation(desc->rotation),
      static_cast<JPH::ObjectLayer>(desc->object_layer));
  if (hair == nullptr) {
    zjolt::Delete(handle);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }
  hair->Init(compute->context->system);

  handle->context = compute->context;
  handle->settings = settings;
  handle->hair = hair;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltHairDestroy(ZJoltHair *hair) {
  if (hair == nullptr) return;
  zjolt::Delete(hair->hair);
  hair->hair = nullptr;
  hair->settings = nullptr;
  hair->context = nullptr;
  zjolt::Delete(hair);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltHairSetTransform(ZJoltHair *hair, const ZJoltRVec3 *position,
                                  const ZJoltQuat *rotation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(hair)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (position != nullptr) {
    hair->position = zjolt::ToJoltR(*position);
    hair->hair->SetPosition(hair->position);
  }
  if (rotation != nullptr) {
    hair->rotation = zjolt::ToJoltRotation(*rotation);
    hair->hair->SetRotation(hair->rotation);
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHairGetTransform(const ZJoltHair *hair, ZJoltRVec3 *out_position,
                                  ZJoltQuat *out_rotation) {
  ZJOLT_ENTER();
  if (!zjolt::Present(hair)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (out_position != nullptr) zjolt::WriteRVec3(out_position, hair->position);
  if (out_rotation != nullptr) zjolt::WriteQuat(out_rotation, hair->rotation);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHairFollowBody(ZJoltHair *hair,
                                const ZJoltPhysicsSystem *system,
                                ZJoltBodyId body) {
  ZJOLT_ENTER();
  if (!zjolt::Present(hair, system)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::BodyLockRead lock(system->system.GetBodyLockInterface(),
                               zjolt::ToJolt(body));
  if (!lock.Succeeded()) {
    return zjolt::SetError(ZJOLT_RESULT_BODY_NOT_FOUND,
                           "the body id does not name a body in this system");
  }
  const JPH::Body &jolt_body = lock.GetBody();
  hair->position = jolt_body.GetPosition();
  hair->rotation = jolt_body.GetRotation();
  hair->hair->SetPosition(hair->position);
  hair->hair->SetRotation(hair->rotation);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHairSetPose(ZJoltHair *hair, const float *joint_to_hair,
                             const float *joint_matrices,
                             uint32_t joint_count) {
  ZJOLT_ENTER();
  if (!zjolt::Present(hair, joint_to_hair, joint_matrices)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const uint32_t expected = static_cast<uint32_t>(
      hair->hair->GetHairSettings()->mScalpInverseBindPose.size());
  if (expected == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "this groom has no scalp, so it has no skeleton to "
                           "pose");
  }
  if (joint_count != expected) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "joint_count must match the groom's inverse bind "
                           "pose; Jolt indexes it without a bound of its own");
  }

  hair->joint_to_hair = LoadMat44(joint_to_hair);
  hair->joint_matrices.clear();
  hair->joint_matrices.reserve(joint_count);
  for (uint32_t i = 0; i < joint_count; ++i) {
    hair->joint_matrices.push_back(LoadMat44(joint_matrices + 16 * i));
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHairGetJointCount(const ZJoltHair *hair,
                                   uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(hair, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  *out_count = static_cast<uint32_t>(
      hair->hair->GetHairSettings()->mScalpInverseBindPose.size());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHairOnTeleported(ZJoltHair *hair) {
  ZJOLT_ENTER();
  if (!zjolt::Present(hair)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  hair->hair->OnTeleported();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHairUpdate(ZJoltHair *hair, ZJoltPhysicsSystem *system,
                            float delta_time) {
  ZJOLT_ENTER();
  if (!zjolt::Present(hair, system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (!(delta_time >= 0.0f)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "delta_time must be zero or positive, and not NaN");
  }

  const JPH::Mat44 *joints =
      hair->joint_matrices.empty() ? nullptr : hair->joint_matrices.data();
  hair->hair->Update(delta_time, hair->joint_to_hair, joints, system->system,
                     hair->context->shaders, hair->context->system,
                     hair->context->queue);

  // Hair::Update only RECORDS work; nothing upstream submits it. Doing it here,
  // synchronously, is what makes this call mean "the hair has stepped" rather
  // than "the hair will step whenever someone flushes the queue".
  hair->context->queue->ExecuteAndWait();
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltHairReadBackPositions(ZJoltHair *hair,
                                       ZJoltVec3 *out_positions,
                                       uint32_t capacity,
                                       uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(hair, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return ReadBackPositions(hair, false, out_positions, capacity, out_count);
}

ZJoltResult zjoltHairReadBackRenderPositions(ZJoltHair *hair,
                                             ZJoltVec3 *out_positions,
                                             uint32_t capacity,
                                             uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(hair, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  return ReadBackPositions(hair, true, out_positions, capacity, out_count);
}

}  // extern "C"
