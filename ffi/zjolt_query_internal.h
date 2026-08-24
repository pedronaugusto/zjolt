//===----------------------------------------------------------------------===//
// zjolt — settings translation shared by the two query implementations.
//
// Not part of the public ABI, and not in zjolt_internal.h either: these need
// Jolt's RayCast.h, CollideShape.h and ShapeCast.h, and zjolt_internal.h is
// included by every translation unit in the library. Putting them there would
// make every file that has nothing to do with queries parse three more Jolt
// headers.
//
// They live here rather than being written out twice because
// zjolt_query.cpp and zjolt_transformed.cpp both take every one of these
// structs, the conversion is pure data with no collector in it, and two
// copies are two things that drift. They already had: one copy carried the
// note about mInternalEdgeRemovalVertexToleranceSq and the other did not.
//
// The COLLECTORS in those two files stay separate. That split has a real
// reason and it is written at the top of zjolt_transformed.cpp.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_QUERY_INTERNAL_H_
#define ZJOLT_QUERY_INTERNAL_H_

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>

namespace zjolt {

inline JPH::ECollectFacesMode ToJoltCollectFacesMode(ZJoltCollectFacesMode mode) {
  return mode == ZJOLT_COLLECT_FACES_MODE_COLLECT_FACES
             ? JPH::ECollectFacesMode::CollectFaces
             : JPH::ECollectFacesMode::NoFaces;
}

/// reason, written at the top of zjolt_transformed.cpp.

inline JPH::EActiveEdgeMode ToJoltActiveEdgeMode(ZJoltActiveEdgeMode mode) {
  return mode == ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL
             ? JPH::EActiveEdgeMode::CollideWithAll
             : JPH::EActiveEdgeMode::CollideOnlyWithActive;
}

inline JPH::EBackFaceMode ToJoltBackFaceMode(ZJoltBackFaceMode mode) {
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
      ToJoltBackFaceMode(settings->back_face_mode_triangles);
  out.mBackFaceModeConvex = ToJoltBackFaceMode(settings->back_face_mode_convex);
  out.mTreatConvexAsSolid = settings->treat_convex_as_solid;
  return out;
}

/// A NULL settings pointer means Jolt's own defaults, which is what an
/// untouched CollideShapeSettings already is.
inline JPH::CollideShapeSettings MakeCollideShapeSettings(
    const ZJoltCollideShapeSettings *settings) {
  JPH::CollideShapeSettings out;
  if (settings == nullptr) return out;
  out.mActiveEdgeMode = ToJoltActiveEdgeMode(settings->active_edge_mode);
  out.mCollectFacesMode = ToJoltCollectFacesMode(settings->collect_faces_mode);
  out.mCollisionTolerance = settings->collision_tolerance;
  out.mPenetrationTolerance = settings->penetration_tolerance;
  out.mActiveEdgeMovementDirection =
      zjolt::ToJolt(settings->active_edge_movement_direction);
  out.mMaxSeparationDistance = settings->max_separation_distance;
  out.mBackFaceMode = ToJoltBackFaceMode(settings->back_face_mode);
  // mInternalEdgeRemovalVertexToleranceSq is deliberately not exposed: it is
  // read only by JPH::InternalEdgeRemovingCollector, which nothing in this ABI
  // wraps a query in, so a field for it would be a field that does nothing.
  return out;
}

inline JPH::ShapeCastSettings MakeShapeCastSettings(
    const ZJoltShapeCastSettings *settings) {
  JPH::ShapeCastSettings out;
  if (settings == nullptr) return out;
  out.mActiveEdgeMode = ToJoltActiveEdgeMode(settings->active_edge_mode);
  out.mCollectFacesMode = ToJoltCollectFacesMode(settings->collect_faces_mode);
  out.mCollisionTolerance = settings->collision_tolerance;
  out.mPenetrationTolerance = settings->penetration_tolerance;
  out.mActiveEdgeMovementDirection =
      zjolt::ToJolt(settings->active_edge_movement_direction);
  out.mExtraConvexRadius = settings->extra_convex_radius;
  out.mBackFaceModeTriangles =
      ToJoltBackFaceMode(settings->back_face_mode_triangles);
  out.mBackFaceModeConvex = ToJoltBackFaceMode(settings->back_face_mode_convex);
  out.mUseShrunkenShapeAndConvexRadius =
      settings->use_shrunken_shape_and_convex_radius;
  out.mReturnDeepestPoint = settings->return_deepest_point;
  return out;
}


}  // namespace zjolt

#endif  // ZJOLT_QUERY_INTERNAL_H_
