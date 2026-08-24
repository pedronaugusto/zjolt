//===----------------------------------------------------------------------===//
// zjolt — compile-time layout assertions and the runtime build report.
//
// The static_asserts here guard the one direction nothing else can see: this
// package against the vendored Jolt it was compiled with. A Jolt upgrade that
// renumbers an enumerator, changes a scalar's width or moves a constant would
// keep compiling — the conversions are switches over Jolt's own names — and
// start meaning something different. These fail the build instead.
//
// The other direction, the C header against a consumer's own declarations, is
// NOT here any more. It was a digest folded over a hand-written list of every
// field of every type, and that list was a duplicate that had to be maintained
// by hand and could be forgotten. Zig consumers now @cImport the header and
// compare by reflection (src/abi_check.zig), which needs no list.
//
// What remains for a consumer that cannot do that is zjoltAbiLayout(): the
// config id, the build flags, and the two scalar widths the build options
// decide. That is enough to refuse a mismatched library, which is the part a
// non-Zig host actually needs.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/ActiveEdgeMode.h>
#include <Jolt/Physics/Collision/CollectFacesMode.h>
#include <Jolt/Physics/EPhysicsUpdateError.h>
#include <Jolt/Physics/PhysicsSettings.h>

#include <cstddef>

namespace {

//===----------------------------------------------------------------------===//
// Scalar widths
//
// Each of these is a value the ABI copies between a Jolt type and a C type of
// a fixed width. A change upstream would truncate silently.
//===----------------------------------------------------------------------===//

static_assert(sizeof(JPH::Real) == sizeof(ZJoltReal),
              "ZJoltReal must match JPH::Real; the double-precision build "
              "setting has diverged between Jolt and zjolt.h");
static_assert(sizeof(JPH::ObjectLayer) == sizeof(ZJoltObjectLayer),
              "ZJoltObjectLayer must match JPH::ObjectLayer; "
              "JPH_OBJECT_LAYER_BITS and ZJOLT_OBJECT_LAYER_BITS disagree");
static_assert(sizeof(JPH::BroadPhaseLayer::Type) == sizeof(ZJoltBroadPhaseLayer),
              "ZJoltBroadPhaseLayer must match JPH::BroadPhaseLayer::Type");
static_assert(sizeof(JPH::BodyID) == sizeof(ZJoltBodyId),
              "ZJoltBodyId must match JPH::BodyID");
static_assert(sizeof(JPH::SubShapeID::Type) == sizeof(ZJoltSubShapeId),
              "ZJoltSubShapeId must match JPH::SubShapeID::Type");
static_assert(JPH::BodyID::cInvalidBodyID == ZJOLT_BODY_ID_INVALID,
              "ZJOLT_BODY_ID_INVALID must match Jolt's invalid body id");

//===----------------------------------------------------------------------===//
// Plain-data layout
//
// ZJoltVec3 is deliberately three floats where Jolt's Vec3 is four; these
// assert the ABI side has not silently acquired padding.
//===----------------------------------------------------------------------===//

static_assert(sizeof(ZJoltVec3) == 3 * sizeof(float),
              "ZJoltVec3 is expected to be three tightly packed floats");
static_assert(sizeof(ZJoltQuat) == 4 * sizeof(float),
              "ZJoltQuat is expected to be four tightly packed floats");
static_assert(sizeof(ZJoltRVec3) == 3 * sizeof(ZJoltReal),
              "ZJoltRVec3 is expected to be three tightly packed reals");
static_assert(sizeof(ZJoltAABox) == 2 * sizeof(ZJoltVec3),
              "ZJoltAABox is expected to be two ZJoltVec3");
static_assert(offsetof(ZJoltAABox, min) == 0, "field moved");
static_assert(offsetof(ZJoltMassProperties, mass) == 0, "field moved");
static_assert(sizeof(ZJoltAllocator) == 6 * sizeof(void *),
              "ZJoltAllocator is expected to be six pointers");

//===----------------------------------------------------------------------===//
// Enumerator agreement
//
// The conversions in the other translation units are switches over Jolt's
// enumerator NAMES, so a renumbering upstream compiles cleanly and changes
// meaning. These pin the values that cross the boundary as integers.
//===----------------------------------------------------------------------===//

static_assert(static_cast<int>(JPH::EMotionType::Static) ==
                  ZJOLT_MOTION_TYPE_STATIC,
              "EMotionType renumbered upstream");
static_assert(static_cast<int>(JPH::EMotionType::Kinematic) ==
                  ZJOLT_MOTION_TYPE_KINEMATIC,
              "EMotionType renumbered upstream");
static_assert(static_cast<int>(JPH::EMotionType::Dynamic) ==
                  ZJOLT_MOTION_TYPE_DYNAMIC,
              "EMotionType renumbered upstream");

static_assert(static_cast<int>(JPH::CharacterBase::EGroundState::OnGround) ==
                  ZJOLT_GROUND_STATE_ON_GROUND,
              "EGroundState renumbered upstream");
static_assert(static_cast<int>(JPH::CharacterBase::EGroundState::InAir) ==
                  ZJOLT_GROUND_STATE_IN_AIR,
              "EGroundState renumbered upstream");

static_assert(
    static_cast<int>(JPH::ValidateResult::AcceptAllContactsForThisBodyPair) ==
        ZJOLT_VALIDATE_RESULT_ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR,
    "ValidateResult renumbered upstream");
static_assert(
    static_cast<int>(JPH::ValidateResult::RejectAllContactsForThisBodyPair) ==
        ZJOLT_VALIDATE_RESULT_REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR,
    "ValidateResult renumbered upstream");

// The three modes a shape query is configured with. Their conversions are
// switches over Jolt's names, so a renumbering upstream would compile and mean
// something else — a query that ignored back faces would start colliding with
// them, silently.
static_assert(static_cast<int>(JPH::EBackFaceMode::IgnoreBackFaces) ==
                  ZJOLT_BACK_FACE_MODE_IGNORE,
              "EBackFaceMode renumbered upstream");
static_assert(static_cast<int>(JPH::EBackFaceMode::CollideWithBackFaces) ==
                  ZJOLT_BACK_FACE_MODE_COLLIDE,
              "EBackFaceMode renumbered upstream");

static_assert(static_cast<int>(JPH::EActiveEdgeMode::CollideOnlyWithActive) ==
                  ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_ONLY_WITH_ACTIVE,
              "EActiveEdgeMode renumbered upstream");
static_assert(static_cast<int>(JPH::EActiveEdgeMode::CollideWithAll) ==
                  ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL,
              "EActiveEdgeMode renumbered upstream");

// Note which way round this one is: CollectFaces is 0 and NoFaces is 1, so a
// zeroed settings struct asks for faces. That is upstream's numbering and the
// ABI mirrors it rather than tidying it.
static_assert(static_cast<int>(JPH::ECollectFacesMode::CollectFaces) ==
                  ZJOLT_COLLECT_FACES_MODE_COLLECT_FACES,
              "ECollectFacesMode renumbered upstream");
static_assert(static_cast<int>(JPH::ECollectFacesMode::NoFaces) ==
                  ZJOLT_COLLECT_FACES_MODE_NO_FACES,
              "ECollectFacesMode renumbered upstream");

// The update error is forwarded as a raw bit mask rather than translated, so
// its bits are part of this ABI.
static_assert(static_cast<uint32_t>(JPH::EPhysicsUpdateError::ManifoldCacheFull) ==
                  ZJOLT_UPDATE_ERROR_MANIFOLD_CACHE_FULL,
              "EPhysicsUpdateError bits renumbered upstream");
static_assert(static_cast<uint32_t>(JPH::EPhysicsUpdateError::BodyPairCacheFull) ==
                  ZJOLT_UPDATE_ERROR_BODY_PAIR_CACHE_FULL,
              "EPhysicsUpdateError bits renumbered upstream");
static_assert(
    static_cast<uint32_t>(JPH::EPhysicsUpdateError::ContactConstraintsFull) ==
        ZJOLT_UPDATE_ERROR_CONTACT_CONSTRAINTS_FULL,
    "EPhysicsUpdateError bits renumbered upstream");

// The degrees-of-freedom mask is passed through as an integer.
static_assert(static_cast<int>(JPH::EAllowedDOFs::All) == ZJOLT_ALLOWED_DOFS_ALL,
              "EAllowedDOFs renumbered upstream");
static_assert(static_cast<int>(JPH::EAllowedDOFs::TranslationX) ==
                  ZJOLT_ALLOWED_DOFS_TRANSLATION_X,
              "EAllowedDOFs renumbered upstream");
static_assert(static_cast<int>(JPH::EAllowedDOFs::RotationZ) ==
                  ZJOLT_ALLOWED_DOFS_ROTATION_Z,
              "EAllowedDOFs renumbered upstream");

//===----------------------------------------------------------------------===//
// Constants the header republishes
//===----------------------------------------------------------------------===//

static_assert(JPH::cMaxPhysicsJobs == ZJOLT_MAX_PHYSICS_JOBS,
              "ZJOLT_MAX_PHYSICS_JOBS no longer matches Jolt's recommendation");
static_assert(JPH::cMaxPhysicsBarriers == ZJOLT_MAX_PHYSICS_BARRIERS,
              "ZJOLT_MAX_PHYSICS_BARRIERS no longer matches Jolt's "
              "recommendation");

//===----------------------------------------------------------------------===//
// Lock storage
//
// ZJoltBodyLock reserves two pointers for the mutex and the lock interface. If
// Jolt ever needs more state to release a lock, this must grow — and growing
// it is an ABI break, so it fails the build rather than overflowing.
//===----------------------------------------------------------------------===//

static_assert(sizeof(ZJoltBodyLock) >= 3 * sizeof(void *),
              "ZJoltBodyLock must hold a body pointer, a mutex and an "
              "interface pointer");

uint32_t BuildFlags() {
  uint32_t flags = 0;
#ifdef ZJOLT_DOUBLE_PRECISION
  flags |= ZJOLT_BUILD_FLAG_DOUBLE_PRECISION;
#endif
#if ZJOLT_OBJECT_LAYER_BITS == 32
  flags |= ZJOLT_BUILD_FLAG_OBJECT_LAYER_32;
#endif
#ifdef JPH_ENABLE_ASSERTS
  flags |= ZJOLT_BUILD_FLAG_ASSERTS_ENABLED;
#endif
#ifdef JPH_CROSS_PLATFORM_DETERMINISTIC
  flags |= ZJOLT_BUILD_FLAG_CROSS_PLATFORM_DETERMINISTIC;
#endif
  return flags;
}

template <typename T>
constexpr uint32_t SizeOf() {
  return static_cast<uint32_t>(sizeof(T));
}

template <typename T>
constexpr uint32_t AlignOf() {
  return static_cast<uint32_t>(alignof(T));
}

}  // namespace

extern "C" {

void zjoltAbiLayout(ZJoltAbiLayout *out) {
  if (out == nullptr) return;

  out->layout_size = SizeOf<ZJoltAbiLayout>();
  out->config_id = static_cast<uint32_t>(ZJOLT_CONFIG_ID);
  out->build_flags = BuildFlags();
  out->real_size = SizeOf<ZJoltReal>();
  out->object_layer_size = SizeOf<ZJoltObjectLayer>();
  out->default_allocate_alignment =
      static_cast<uint32_t>(zjoltDefaultAllocateAlignment());
}

}  // extern "C"
