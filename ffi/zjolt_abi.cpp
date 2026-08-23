//===----------------------------------------------------------------------===//
// zjolt — compile-time layout assertions and the runtime layout report.
//
// Two independent guards live here, covering the two directions drift can go:
//
//   1. static_asserts that fail the BUILD if a vendored Jolt upgrade changes a
//      type, an enumerator or a constant this package converts to, from, or
//      relies on. Nothing else would catch a silently renumbered enum: the
//      conversions are switches over Jolt's names, so they would keep
//      compiling and start meaning something different.
//
//   2. zjoltAbiLayout(), which lets the Zig wrapper assert in a TEST that its
//      hand-written externs still match this translation unit.
//
// Together they cover C++ vs Jolt, and Zig vs C.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

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
        ZJOLT_VALIDATE_ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR,
    "ValidateResult renumbered upstream");
static_assert(
    static_cast<int>(JPH::ValidateResult::RejectAllContactsForThisBodyPair) ==
        ZJOLT_VALIDATE_REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR,
    "ValidateResult renumbered upstream");

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

/// One past the last enumerator of each C enum, for the exhaustiveness checks
/// the Zig side performs. Kept next to the enums they count so that adding an
/// enumerator without updating this is visible in one diff.
constexpr uint32_t kResultCount =
    static_cast<uint32_t>(ZJOLT_ERR_BODY_NOT_FOUND) + 1u;
constexpr uint32_t kMotionTypeCount =
    static_cast<uint32_t>(ZJOLT_MOTION_TYPE_DYNAMIC) + 1u;
constexpr uint32_t kGroundStateCount =
    static_cast<uint32_t>(ZJOLT_GROUND_STATE_IN_AIR) + 1u;
constexpr uint32_t kShapeSubTypeCount =
    static_cast<uint32_t>(ZJOLT_SHAPE_SUB_TYPE_OFFSET_CENTER_OF_MASS) + 1u;
constexpr uint32_t kValidateResultCount =
    static_cast<uint32_t>(ZJOLT_VALIDATE_REJECT_ALL_CONTACTS_FOR_THIS_BODY_PAIR) +
    1u;


//===----------------------------------------------------------------------===//
// Layout digest
//
// Folds the size, alignment and every field offset of every ABI type into one
// number. A consumer that can reflect over its own structs computes the same
// fold and compares a single value — which is what catches the changes the
// hand-listed offsets below cannot see, such as two same-sized adjacent fields
// swapping places.
//
// The field lists here are in declaration order, which is the order a
// reflecting consumer will walk them in. Adding a field to a struct in
// zjolt.h means adding it here too; forgetting to is caught by the ABI test
// on the other side of the boundary, because its reflection will have seen the
// new field and this fold will not.
//===----------------------------------------------------------------------===//

void Fold(uint32_t &hash, uint32_t value) {
  for (int byte = 0; byte < 4; ++byte) {
    hash ^= static_cast<uint8_t>(value >> (byte * 8));
    hash *= 16777619u;  // FNV-1a
  }
}

#define ZJOLT_FOLD_TYPE(T)                     \
  Fold(hash, static_cast<uint32_t>(sizeof(T)));\
  Fold(hash, static_cast<uint32_t>(alignof(T)))

/// Folds the field's NAME as well as its offset. Without the name, two
/// same-sized adjacent fields swapping places would leave the sequence of
/// offsets unchanged and the digest identical — which is the exact case a
/// hand-listed offset table also misses, and the reason this exists.
void FoldName(uint32_t &hash, const char *name) {
  for (const char *p = name; *p != '\0'; ++p)
    Fold(hash, static_cast<uint32_t>(static_cast<unsigned char>(*p)));
}

#define ZJOLT_FOLD_FIELD(T, field)  \
  FoldName(hash, #field);           \
  Fold(hash, static_cast<uint32_t>(offsetof(T, field)))

uint32_t LayoutDigest() {
  uint32_t hash = 2166136261u;  // FNV-1a offset basis

  ZJOLT_FOLD_TYPE(ZJoltVec3);
  ZJOLT_FOLD_FIELD(ZJoltVec3, x);
  ZJOLT_FOLD_FIELD(ZJoltVec3, y);
  ZJOLT_FOLD_FIELD(ZJoltVec3, z);

  ZJOLT_FOLD_TYPE(ZJoltQuat);
  ZJOLT_FOLD_FIELD(ZJoltQuat, x);
  ZJOLT_FOLD_FIELD(ZJoltQuat, y);
  ZJOLT_FOLD_FIELD(ZJoltQuat, z);
  ZJOLT_FOLD_FIELD(ZJoltQuat, w);

  ZJOLT_FOLD_TYPE(ZJoltRVec3);
  ZJOLT_FOLD_FIELD(ZJoltRVec3, x);
  ZJOLT_FOLD_FIELD(ZJoltRVec3, y);
  ZJOLT_FOLD_FIELD(ZJoltRVec3, z);

  ZJOLT_FOLD_TYPE(ZJoltAABox);
  ZJOLT_FOLD_FIELD(ZJoltAABox, min);
  ZJOLT_FOLD_FIELD(ZJoltAABox, max);

  ZJOLT_FOLD_TYPE(ZJoltMassProperties);
  ZJOLT_FOLD_FIELD(ZJoltMassProperties, mass);
  ZJOLT_FOLD_FIELD(ZJoltMassProperties, inertia);

  ZJOLT_FOLD_TYPE(ZJoltShapeStats);
  ZJOLT_FOLD_FIELD(ZJoltShapeStats, size_bytes);
  ZJOLT_FOLD_FIELD(ZJoltShapeStats, num_triangles);

  ZJOLT_FOLD_TYPE(ZJoltAllocator);
  ZJOLT_FOLD_FIELD(ZJoltAllocator, allocate);
  ZJOLT_FOLD_FIELD(ZJoltAllocator, reallocate);
  ZJOLT_FOLD_FIELD(ZJoltAllocator, free);
  ZJOLT_FOLD_FIELD(ZJoltAllocator, aligned_allocate);
  ZJOLT_FOLD_FIELD(ZJoltAllocator, aligned_free);
  ZJOLT_FOLD_FIELD(ZJoltAllocator, user);

  ZJOLT_FOLD_TYPE(ZJoltInitDesc);
  ZJOLT_FOLD_FIELD(ZJoltInitDesc, allocator);
  ZJOLT_FOLD_FIELD(ZJoltInitDesc, trace);
  ZJOLT_FOLD_FIELD(ZJoltInitDesc, assert_failed);
  ZJOLT_FOLD_FIELD(ZJoltInitDesc, hooks_user);

  ZJOLT_FOLD_TYPE(ZJoltBroadPhaseLayerInterface);
  ZJOLT_FOLD_FIELD(ZJoltBroadPhaseLayerInterface, num_broad_phase_layers);
  ZJOLT_FOLD_FIELD(ZJoltBroadPhaseLayerInterface,
                   broad_phase_layer_for_object_layer);
  ZJOLT_FOLD_FIELD(ZJoltBroadPhaseLayerInterface, broad_phase_layer_name);
  ZJOLT_FOLD_FIELD(ZJoltBroadPhaseLayerInterface, user);

  ZJOLT_FOLD_TYPE(ZJoltObjectVsBroadPhaseLayerFilter);
  ZJOLT_FOLD_FIELD(ZJoltObjectVsBroadPhaseLayerFilter, should_collide);
  ZJOLT_FOLD_FIELD(ZJoltObjectVsBroadPhaseLayerFilter, user);

  ZJOLT_FOLD_TYPE(ZJoltObjectLayerPairFilter);
  ZJOLT_FOLD_FIELD(ZJoltObjectLayerPairFilter, should_collide);
  ZJOLT_FOLD_FIELD(ZJoltObjectLayerPairFilter, user);

  ZJOLT_FOLD_TYPE(ZJoltPhysicsSystemDesc);
  ZJOLT_FOLD_FIELD(ZJoltPhysicsSystemDesc, max_bodies);
  ZJOLT_FOLD_FIELD(ZJoltPhysicsSystemDesc, num_body_mutexes);
  ZJOLT_FOLD_FIELD(ZJoltPhysicsSystemDesc, max_body_pairs);
  ZJOLT_FOLD_FIELD(ZJoltPhysicsSystemDesc, max_contact_constraints);
  ZJOLT_FOLD_FIELD(ZJoltPhysicsSystemDesc, temp_allocator_size);
  ZJOLT_FOLD_FIELD(ZJoltPhysicsSystemDesc, broad_phase_layers);
  ZJOLT_FOLD_FIELD(ZJoltPhysicsSystemDesc, object_vs_broad_phase_filter);
  ZJOLT_FOLD_FIELD(ZJoltPhysicsSystemDesc, object_layer_pair_filter);

  ZJOLT_FOLD_TYPE(ZJoltContactManifold);
  ZJOLT_FOLD_FIELD(ZJoltContactManifold, base_offset);
  ZJOLT_FOLD_FIELD(ZJoltContactManifold, world_space_normal);
  ZJOLT_FOLD_FIELD(ZJoltContactManifold, penetration_depth);
  ZJOLT_FOLD_FIELD(ZJoltContactManifold, sub_shape_id1);
  ZJOLT_FOLD_FIELD(ZJoltContactManifold, sub_shape_id2);
  ZJOLT_FOLD_FIELD(ZJoltContactManifold, num_points);
  ZJOLT_FOLD_FIELD(ZJoltContactManifold, points_on_1);
  ZJOLT_FOLD_FIELD(ZJoltContactManifold, points_on_2);

  ZJOLT_FOLD_TYPE(ZJoltContactInfo);
  ZJOLT_FOLD_FIELD(ZJoltContactInfo, body1);
  ZJOLT_FOLD_FIELD(ZJoltContactInfo, body2);
  ZJOLT_FOLD_FIELD(ZJoltContactInfo, user_data1);
  ZJOLT_FOLD_FIELD(ZJoltContactInfo, user_data2);
  ZJOLT_FOLD_FIELD(ZJoltContactInfo, manifold);

  ZJOLT_FOLD_TYPE(ZJoltContactSettings);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, combined_friction);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, combined_restitution);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, inv_mass_scale1);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, inv_inertia_scale1);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, inv_mass_scale2);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, inv_inertia_scale2);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, is_sensor);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, relative_linear_surface_velocity);
  ZJOLT_FOLD_FIELD(ZJoltContactSettings, relative_angular_surface_velocity);

  ZJOLT_FOLD_TYPE(ZJoltContactValidateInfo);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, body1);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, body2);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, user_data1);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, user_data2);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, base_offset);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, contact_point_on_1);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, contact_point_on_2);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, penetration_axis);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, penetration_depth);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, sub_shape_id1);
  ZJOLT_FOLD_FIELD(ZJoltContactValidateInfo, sub_shape_id2);

  ZJOLT_FOLD_TYPE(ZJoltSubShapeIdPair);
  ZJOLT_FOLD_FIELD(ZJoltSubShapeIdPair, body1);
  ZJOLT_FOLD_FIELD(ZJoltSubShapeIdPair, sub_shape_id1);
  ZJOLT_FOLD_FIELD(ZJoltSubShapeIdPair, body2);
  ZJOLT_FOLD_FIELD(ZJoltSubShapeIdPair, sub_shape_id2);

  ZJOLT_FOLD_TYPE(ZJoltContactListener);
  ZJOLT_FOLD_FIELD(ZJoltContactListener, on_contact_validate);
  ZJOLT_FOLD_FIELD(ZJoltContactListener, on_contact_added);
  ZJOLT_FOLD_FIELD(ZJoltContactListener, on_contact_persisted);
  ZJOLT_FOLD_FIELD(ZJoltContactListener, on_contact_removed);
  ZJOLT_FOLD_FIELD(ZJoltContactListener, user);

  ZJOLT_FOLD_TYPE(ZJoltBodyActivationListener);
  ZJOLT_FOLD_FIELD(ZJoltBodyActivationListener, on_body_activated);
  ZJOLT_FOLD_FIELD(ZJoltBodyActivationListener, on_body_deactivated);
  ZJOLT_FOLD_FIELD(ZJoltBodyActivationListener, user);

  ZJOLT_FOLD_TYPE(ZJoltBodyDesc);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, position);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, rotation);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, linear_velocity);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, angular_velocity);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, shape);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, user_data);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, object_layer);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, motion_type);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, motion_quality);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, allowed_dofs);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, override_mass_properties);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, mass);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, allow_dynamic_or_kinematic);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, is_sensor);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, allow_sleeping);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, enhanced_internal_edge_removal);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, friction);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, restitution);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, linear_damping);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, angular_damping);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, max_linear_velocity);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, max_angular_velocity);
  ZJOLT_FOLD_FIELD(ZJoltBodyDesc, gravity_factor);

  ZJOLT_FOLD_TYPE(ZJoltBodyLock);
  ZJOLT_FOLD_FIELD(ZJoltBodyLock, body);
  ZJOLT_FOLD_FIELD(ZJoltBodyLock, _reserved);

  ZJOLT_FOLD_TYPE(ZJoltBroadPhaseLayerFilter);
  ZJOLT_FOLD_FIELD(ZJoltBroadPhaseLayerFilter, should_collide);
  ZJOLT_FOLD_FIELD(ZJoltBroadPhaseLayerFilter, user);

  ZJOLT_FOLD_TYPE(ZJoltObjectLayerFilter);
  ZJOLT_FOLD_FIELD(ZJoltObjectLayerFilter, should_collide);
  ZJOLT_FOLD_FIELD(ZJoltObjectLayerFilter, user);

  ZJOLT_FOLD_TYPE(ZJoltBodyFilter);
  ZJOLT_FOLD_FIELD(ZJoltBodyFilter, should_collide);
  ZJOLT_FOLD_FIELD(ZJoltBodyFilter, user);

  ZJOLT_FOLD_TYPE(ZJoltQueryFilters);
  ZJOLT_FOLD_FIELD(ZJoltQueryFilters, broad_phase_layer);
  ZJOLT_FOLD_FIELD(ZJoltQueryFilters, object_layer);
  ZJOLT_FOLD_FIELD(ZJoltQueryFilters, body);

  ZJOLT_FOLD_TYPE(ZJoltRayCastHit);
  ZJOLT_FOLD_FIELD(ZJoltRayCastHit, body);
  ZJOLT_FOLD_FIELD(ZJoltRayCastHit, sub_shape_id);
  ZJOLT_FOLD_FIELD(ZJoltRayCastHit, fraction);

  ZJOLT_FOLD_TYPE(ZJoltShapeCastHit);
  ZJOLT_FOLD_FIELD(ZJoltShapeCastHit, body);
  ZJOLT_FOLD_FIELD(ZJoltShapeCastHit, sub_shape_id);
  ZJOLT_FOLD_FIELD(ZJoltShapeCastHit, fraction);
  ZJOLT_FOLD_FIELD(ZJoltShapeCastHit, contact_point_on_1);
  ZJOLT_FOLD_FIELD(ZJoltShapeCastHit, contact_point_on_2);
  ZJOLT_FOLD_FIELD(ZJoltShapeCastHit, penetration_axis);
  ZJOLT_FOLD_FIELD(ZJoltShapeCastHit, penetration_depth);
  ZJOLT_FOLD_FIELD(ZJoltShapeCastHit, is_back_face_hit);

  ZJOLT_FOLD_TYPE(ZJoltCollideShapeHit);
  ZJOLT_FOLD_FIELD(ZJoltCollideShapeHit, body);
  ZJOLT_FOLD_FIELD(ZJoltCollideShapeHit, sub_shape_id);
  ZJOLT_FOLD_FIELD(ZJoltCollideShapeHit, contact_point_on_1);
  ZJOLT_FOLD_FIELD(ZJoltCollideShapeHit, contact_point_on_2);
  ZJOLT_FOLD_FIELD(ZJoltCollideShapeHit, penetration_axis);
  ZJOLT_FOLD_FIELD(ZJoltCollideShapeHit, penetration_depth);

  ZJOLT_FOLD_TYPE(ZJoltCharacterDesc);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, shape);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, position);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, rotation);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, up);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, shape_offset);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, user_data);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, max_slope_angle);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, mass);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, max_strength);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, predictive_contact_distance);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, character_padding);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, penetration_recovery_speed);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, collision_tolerance);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, hit_reduction_cos_max_angle);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, max_collision_iterations);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, max_constraint_iterations);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, max_num_hits);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, back_face_mode);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, enhanced_internal_edge_removal);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, inner_body_shape);
  ZJOLT_FOLD_FIELD(ZJoltCharacterDesc, inner_body_layer);

  ZJOLT_FOLD_TYPE(ZJoltCharacterUpdateSettings);
  ZJOLT_FOLD_FIELD(ZJoltCharacterUpdateSettings, stick_to_floor_step_down);
  ZJOLT_FOLD_FIELD(ZJoltCharacterUpdateSettings, walk_stairs_step_up);
  ZJOLT_FOLD_FIELD(ZJoltCharacterUpdateSettings, walk_stairs_min_step_forward);
  ZJOLT_FOLD_FIELD(ZJoltCharacterUpdateSettings, walk_stairs_step_forward_test);
  ZJOLT_FOLD_FIELD(ZJoltCharacterUpdateSettings,
                   walk_stairs_cos_angle_forward_contact);
  ZJOLT_FOLD_FIELD(ZJoltCharacterUpdateSettings, walk_stairs_step_down_extra);

  return hash;
}

#undef ZJOLT_FOLD_TYPE
#undef ZJOLT_FOLD_FIELD

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
  out->layout_digest = LayoutDigest();

  out->real_size = SizeOf<ZJoltReal>();
  out->object_layer_size = SizeOf<ZJoltObjectLayer>();
  out->default_allocate_alignment =
      static_cast<uint32_t>(JPH_DEFAULT_ALLOCATE_ALIGNMENT);

  out->vec3_size = SizeOf<ZJoltVec3>();
  out->vec3_align = AlignOf<ZJoltVec3>();
  out->rvec3_size = SizeOf<ZJoltRVec3>();
  out->rvec3_align = AlignOf<ZJoltRVec3>();
  out->quat_size = SizeOf<ZJoltQuat>();
  out->quat_align = AlignOf<ZJoltQuat>();
  out->aabox_size = SizeOf<ZJoltAABox>();
  out->aabox_align = AlignOf<ZJoltAABox>();
  out->mass_properties_size = SizeOf<ZJoltMassProperties>();
  out->mass_properties_align = AlignOf<ZJoltMassProperties>();
  out->shape_stats_size = SizeOf<ZJoltShapeStats>();
  out->shape_stats_align = AlignOf<ZJoltShapeStats>();

  out->allocator_size = SizeOf<ZJoltAllocator>();
  out->allocator_align = AlignOf<ZJoltAllocator>();
  out->allocator_offset_allocate =
      static_cast<uint32_t>(offsetof(ZJoltAllocator, allocate));
  out->allocator_offset_reallocate =
      static_cast<uint32_t>(offsetof(ZJoltAllocator, reallocate));
  out->allocator_offset_free =
      static_cast<uint32_t>(offsetof(ZJoltAllocator, free));
  out->allocator_offset_aligned_allocate =
      static_cast<uint32_t>(offsetof(ZJoltAllocator, aligned_allocate));
  out->allocator_offset_aligned_free =
      static_cast<uint32_t>(offsetof(ZJoltAllocator, aligned_free));
  out->allocator_offset_user =
      static_cast<uint32_t>(offsetof(ZJoltAllocator, user));

  out->init_desc_size = SizeOf<ZJoltInitDesc>();
  out->init_desc_align = AlignOf<ZJoltInitDesc>();

  out->broad_phase_layer_interface_size =
      SizeOf<ZJoltBroadPhaseLayerInterface>();
  out->broad_phase_layer_interface_align =
      AlignOf<ZJoltBroadPhaseLayerInterface>();
  out->object_vs_broad_phase_filter_size =
      SizeOf<ZJoltObjectVsBroadPhaseLayerFilter>();
  out->object_layer_pair_filter_size = SizeOf<ZJoltObjectLayerPairFilter>();

  out->system_desc_size = SizeOf<ZJoltPhysicsSystemDesc>();
  out->system_desc_align = AlignOf<ZJoltPhysicsSystemDesc>();
  out->system_desc_offset_broad_phase_layers = static_cast<uint32_t>(
      offsetof(ZJoltPhysicsSystemDesc, broad_phase_layers));
  out->system_desc_offset_object_vs_broad_phase_filter = static_cast<uint32_t>(
      offsetof(ZJoltPhysicsSystemDesc, object_vs_broad_phase_filter));
  out->system_desc_offset_object_layer_pair_filter = static_cast<uint32_t>(
      offsetof(ZJoltPhysicsSystemDesc, object_layer_pair_filter));

  out->contact_manifold_size = SizeOf<ZJoltContactManifold>();
  out->contact_manifold_align = AlignOf<ZJoltContactManifold>();
  out->contact_manifold_offset_points_on_1 =
      static_cast<uint32_t>(offsetof(ZJoltContactManifold, points_on_1));
  out->contact_manifold_offset_points_on_2 =
      static_cast<uint32_t>(offsetof(ZJoltContactManifold, points_on_2));
  out->contact_info_size = SizeOf<ZJoltContactInfo>();
  out->contact_info_align = AlignOf<ZJoltContactInfo>();
  out->contact_info_offset_manifold =
      static_cast<uint32_t>(offsetof(ZJoltContactInfo, manifold));
  out->contact_settings_size = SizeOf<ZJoltContactSettings>();
  out->contact_settings_align = AlignOf<ZJoltContactSettings>();
  out->contact_validate_info_size = SizeOf<ZJoltContactValidateInfo>();
  out->contact_validate_info_align = AlignOf<ZJoltContactValidateInfo>();
  out->sub_shape_id_pair_size = SizeOf<ZJoltSubShapeIdPair>();
  out->contact_listener_size = SizeOf<ZJoltContactListener>();
  out->body_activation_listener_size = SizeOf<ZJoltBodyActivationListener>();

  out->body_desc_size = SizeOf<ZJoltBodyDesc>();
  out->body_desc_align = AlignOf<ZJoltBodyDesc>();
  out->body_desc_offset_position =
      static_cast<uint32_t>(offsetof(ZJoltBodyDesc, position));
  out->body_desc_offset_rotation =
      static_cast<uint32_t>(offsetof(ZJoltBodyDesc, rotation));
  out->body_desc_offset_shape =
      static_cast<uint32_t>(offsetof(ZJoltBodyDesc, shape));
  out->body_desc_offset_user_data =
      static_cast<uint32_t>(offsetof(ZJoltBodyDesc, user_data));
  out->body_desc_offset_object_layer =
      static_cast<uint32_t>(offsetof(ZJoltBodyDesc, object_layer));
  out->body_desc_offset_motion_type =
      static_cast<uint32_t>(offsetof(ZJoltBodyDesc, motion_type));
  out->body_desc_offset_gravity_factor =
      static_cast<uint32_t>(offsetof(ZJoltBodyDesc, gravity_factor));

  out->body_lock_size = SizeOf<ZJoltBodyLock>();
  out->body_lock_align = AlignOf<ZJoltBodyLock>();
  out->body_lock_offset_body =
      static_cast<uint32_t>(offsetof(ZJoltBodyLock, body));

  out->query_filters_size = SizeOf<ZJoltQueryFilters>();
  out->query_filters_align = AlignOf<ZJoltQueryFilters>();
  out->ray_cast_hit_size = SizeOf<ZJoltRayCastHit>();
  out->ray_cast_hit_align = AlignOf<ZJoltRayCastHit>();
  out->shape_cast_hit_size = SizeOf<ZJoltShapeCastHit>();
  out->shape_cast_hit_align = AlignOf<ZJoltShapeCastHit>();
  out->collide_shape_hit_size = SizeOf<ZJoltCollideShapeHit>();
  out->collide_shape_hit_align = AlignOf<ZJoltCollideShapeHit>();

  out->character_desc_size = SizeOf<ZJoltCharacterDesc>();
  out->character_desc_align = AlignOf<ZJoltCharacterDesc>();
  out->character_desc_offset_shape =
      static_cast<uint32_t>(offsetof(ZJoltCharacterDesc, shape));
  out->character_desc_offset_position =
      static_cast<uint32_t>(offsetof(ZJoltCharacterDesc, position));
  out->character_desc_offset_up =
      static_cast<uint32_t>(offsetof(ZJoltCharacterDesc, up));
  out->character_update_settings_size = SizeOf<ZJoltCharacterUpdateSettings>();
  out->character_update_settings_align =
      AlignOf<ZJoltCharacterUpdateSettings>();

  out->result_count = kResultCount;
  out->motion_type_count = kMotionTypeCount;
  out->ground_state_count = kGroundStateCount;
  out->shape_sub_type_count = kShapeSubTypeCount;
  out->validate_result_count = kValidateResultCount;
}

}  // extern "C"
