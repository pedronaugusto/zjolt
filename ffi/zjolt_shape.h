//===----------------------------------------------------------------------===//
// zjolt — collision shapes.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_SHAPE_H_
#define ZJOLT_SHAPE_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Shapes
//
// Every constructor returns a shape with a reference count of one. Adding a
// shape to a body takes its own reference, so the usual pattern is to create,
// create the body, and release.
//===----------------------------------------------------------------------===//

/// `convex_radius` rounds the box corners for cheaper, more stable collision;
/// it must be at most the smallest half extent. `density` is in kg/m^3.
ZJOLT_API ZJoltResult zjoltShapeCreateBox(const ZJoltVec3 *half_extent,
                                          float convex_radius, float density,
                                          ZJoltShape **out);

ZJOLT_API ZJoltResult zjoltShapeCreateSphere(float radius, float density,
                                             ZJoltShape **out);

/// A capsule along the Y axis: a cylinder of `half_height_of_cylinder` capped
/// by hemispheres of `radius`. Total height is 2*(half_height + radius).
ZJOLT_API ZJoltResult zjoltShapeCreateCapsule(float half_height_of_cylinder,
                                              float radius, float density,
                                              ZJoltShape **out);

/// Builds the convex hull OF the given points; interior points are allowed and
/// discarded. `hull_tolerance` is how far a point may sit outside the hull
/// (larger yields fewer vertices); pass 0 for Jolt's default.
ZJOLT_API ZJoltResult zjoltShapeCreateConvexHull(const ZJoltVec3 *points,
                                                 uint32_t num_points,
                                                 float max_convex_radius,
                                                 float hull_tolerance,
                                                 float density,
                                                 ZJoltShape **out);

/// A static triangle mesh. `indices` holds 3*num_triangles vertex indices.
/// Duplicate and degenerate triangles are removed by Jolt during the build.
///
/// A mesh shape may only be used by a static or kinematic body — Jolt has no
/// inertia for one. Building it is the expensive part of collision cooking, so
/// this is the shape most worth saving with zjoltShapeSave.
ZJOLT_API ZJoltResult zjoltShapeCreateMesh(const ZJoltVec3 *vertices,
                                           uint32_t num_vertices,
                                           const uint32_t *indices,
                                           uint32_t num_triangles,
                                           uint32_t max_triangles_per_leaf,
                                           ZJoltShape **out);

/// Non-uniformly scales an existing shape. Takes a reference on `inner`.
ZJOLT_API ZJoltResult zjoltShapeCreateScaled(const ZJoltShape *inner,
                                             const ZJoltVec3 *scale,
                                             ZJoltShape **out);

/// Places an existing shape at an offset and orientation inside its parent.
ZJOLT_API ZJoltResult zjoltShapeCreateRotatedTranslated(
    const ZJoltShape *inner, const ZJoltVec3 *translation,
    const ZJoltQuat *rotation, ZJoltShape **out);

/// Shifts a shape's centre of mass without moving its geometry.
ZJOLT_API ZJoltResult zjoltShapeCreateOffsetCenterOfMass(
    const ZJoltShape *inner, const ZJoltVec3 *offset, ZJoltShape **out);

ZJOLT_API void zjoltShapeAddRef(const ZJoltShape *shape);
ZJOLT_API void zjoltShapeRelease(const ZJoltShape *shape);
ZJOLT_API uint32_t zjoltShapeGetRefCount(const ZJoltShape *shape);

ZJOLT_API ZJoltShapeSubType zjoltShapeGetSubType(const ZJoltShape *shape);
ZJOLT_API float zjoltShapeGetVolume(const ZJoltShape *shape);
ZJOLT_API void zjoltShapeGetCenterOfMass(const ZJoltShape *shape,
                                         ZJoltVec3 *out);
ZJOLT_API void zjoltShapeGetLocalBounds(const ZJoltShape *shape,
                                        ZJoltAABox *out);
ZJOLT_API void zjoltShapeGetMassProperties(const ZJoltShape *shape,
                                           ZJoltMassProperties *out);
/// Memory footprint and triangle count of a shape and everything under it.
///
/// Jolt exposes this for logging and budgeting, and it is the cheapest way to
/// confirm a shape survived a save/restore round trip with its children
/// intact.
typedef struct ZJoltShapeStats {
  uint64_t size_bytes;
  uint32_t num_triangles;
} ZJoltShapeStats;

ZJOLT_API void zjoltShapeGetStats(const ZJoltShape *shape,
                                  ZJoltShapeStats *out);

/// Serialises `shape` and everything under it into `buffer`.
///
/// Two-call protocol: pass buffer = NULL to learn the size, then call again
/// with storage. `*out_size` is always written, so a too-small buffer reports
/// ZJOLT_ERR_BUFFER_TOO_SMALL along with what was needed.
///
/// The payload is Jolt's own binary shape state, behind a 32-byte header
/// carrying a magic tag, the container version, this library's config id, the
/// Jolt version, the payload length and a CRC-32. The header is not
/// decoration: Jolt reads the shape type out of the stream and uses it to
/// index a table BEFORE it checks whether the read succeeded, so a buffer that
/// is not a shape has to be rejected before Jolt sees it.
///
/// It remains a cooking cache, not an interchange format — a shape saved by a
/// different Jolt version, or a different precision setting, is refused rather
/// than reinterpreted. And it is not a defence against a crafted payload with
/// a matching checksum; treat a cache as something your own tools wrote.
ZJOLT_API ZJoltResult zjoltShapeSave(const ZJoltShape *shape, void *buffer,
                                     size_t capacity, size_t *out_size);

/// Rebuilds a shape from zjoltShapeSave output. The buffer is read during the
/// call only. A wrong tag, a wrong build, a length that disagrees with the
/// buffer, or a failed checksum is ZJOLT_ERR_BAD_FORMAT; zjoltLastError says
/// which.
ZJOLT_API ZJoltResult zjoltShapeRestore(const void *data, size_t size,
                                        ZJoltShape **out);

/// Bytes zjoltShapeSave prepends to Jolt's payload. Exposed so a test can
/// reach the payload itself, not so callers can build one by hand.
#define ZJOLT_SHAPE_HEADER_SIZE 32

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SHAPE_H_
