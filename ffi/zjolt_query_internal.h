//===----------------------------------------------------------------------===//
// zjolt — settings translation shared by zjolt_query.cpp and
// zjolt_transformed.cpp.
//
// Not in zjolt_internal.h: these need Jolt's RayCast.h, CollideShape.h and
// ShapeCast.h, and zjolt_internal.h is included by every translation unit in
// the library. Not part of the public ABI.
//
// The COLLECTORS in the two callers stay separate; see the header comment in
// zjolt_transformed.cpp.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_QUERY_INTERNAL_H_
#define ZJOLT_QUERY_INTERNAL_H_

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>

namespace zjolt {

// These three take the raw integer, not the enum: each field comes straight
// out of a host-supplied struct (ZJoltCollideShapeSettings,
// ZJoltShapeCastSettings, ZJoltRayCastSettings) that this ABI never
// validates, and reading it as the enum type would load a value C++ never
// promised is valid. Make*Settings converts with zjolt::RawEnum
// (zjolt_internal.h) at the point it pulls each field out.
inline JPH::ECollectFacesMode ToJoltCollectFacesMode(int32_t mode) {
  return mode == ZJOLT_COLLECT_FACES_MODE_COLLECT_FACES
             ? JPH::ECollectFacesMode::CollectFaces
             : JPH::ECollectFacesMode::NoFaces;
}

inline JPH::EActiveEdgeMode ToJoltActiveEdgeMode(int32_t mode) {
  return mode == ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL
             ? JPH::EActiveEdgeMode::CollideWithAll
             : JPH::EActiveEdgeMode::CollideOnlyWithActive;
}

inline JPH::EBackFaceMode ToJoltBackFaceMode(int32_t mode) {
  return mode == ZJOLT_BACK_FACE_MODE_COLLIDE
             ? JPH::EBackFaceMode::CollideWithBackFaces
             : JPH::EBackFaceMode::IgnoreBackFaces;
}

/// A NULL ZJoltRayCastSettings means Jolt's own defaults, which is what an
/// untouched RayCastSettings already is.
inline JPH::RayCastSettings MakeRayCastSettings(const ZJoltRayCastSettings *settings) {
  JPH::RayCastSettings out;
  if (settings == nullptr) return out;
  out.mBackFaceModeTriangles =
      ToJoltBackFaceMode(zjolt::RawEnum(settings->back_face_mode_triangles));
  out.mBackFaceModeConvex =
      ToJoltBackFaceMode(zjolt::RawEnum(settings->back_face_mode_convex));
  out.mTreatConvexAsSolid = settings->treat_convex_as_solid;
  return out;
}

/// A NULL settings pointer means Jolt's own defaults, which is what an
/// untouched CollideShapeSettings already is.
inline JPH::CollideShapeSettings MakeCollideShapeSettings(
    const ZJoltCollideShapeSettings *settings) {
  JPH::CollideShapeSettings out;
  if (settings == nullptr) return out;
  out.mActiveEdgeMode =
      ToJoltActiveEdgeMode(zjolt::RawEnum(settings->active_edge_mode));
  out.mCollectFacesMode =
      ToJoltCollectFacesMode(zjolt::RawEnum(settings->collect_faces_mode));
  out.mCollisionTolerance = settings->collision_tolerance;
  out.mPenetrationTolerance = settings->penetration_tolerance;
  out.mActiveEdgeMovementDirection =
      zjolt::ToJolt(settings->active_edge_movement_direction);
  out.mMaxSeparationDistance = settings->max_separation_distance;
  out.mBackFaceMode = ToJoltBackFaceMode(zjolt::RawEnum(settings->back_face_mode));
  // Read only by JPH::InternalEdgeRemovingCollector -- inert on every other
  // query here, and on this one too unless it is one of the
  // zjoltCollideShapeWithInternalEdgeRemoval* forms, which force the two
  // fields above it that the collector requires regardless of what this
  // struct carries for them.
  out.mInternalEdgeRemovalVertexToleranceSq =
      settings->internal_edge_removal_vertex_tolerance_sq;
  return out;
}

inline JPH::ShapeCastSettings MakeShapeCastSettings(
    const ZJoltShapeCastSettings *settings) {
  JPH::ShapeCastSettings out;
  if (settings == nullptr) return out;
  out.mActiveEdgeMode =
      ToJoltActiveEdgeMode(zjolt::RawEnum(settings->active_edge_mode));
  out.mCollectFacesMode =
      ToJoltCollectFacesMode(zjolt::RawEnum(settings->collect_faces_mode));
  out.mCollisionTolerance = settings->collision_tolerance;
  out.mPenetrationTolerance = settings->penetration_tolerance;
  out.mActiveEdgeMovementDirection =
      zjolt::ToJolt(settings->active_edge_movement_direction);
  out.mExtraConvexRadius = settings->extra_convex_radius;
  out.mBackFaceModeTriangles =
      ToJoltBackFaceMode(zjolt::RawEnum(settings->back_face_mode_triangles));
  out.mBackFaceModeConvex =
      ToJoltBackFaceMode(zjolt::RawEnum(settings->back_face_mode_convex));
  out.mUseShrunkenShapeAndConvexRadius =
      settings->use_shrunken_shape_and_convex_radius;
  out.mReturnDeepestPoint = settings->return_deepest_point;
  return out;
}


}  // namespace zjolt

#endif  // ZJOLT_QUERY_INTERNAL_H_
