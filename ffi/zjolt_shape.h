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
//
// Where a constructor takes a `material`, NULL means Jolt's shared default —
// which is what zjoltShapeGetMaterial will then report, rather than NULL. A
// shape holds a reference on every material it was built with. See
// zjolt_material.h for what a material is, and what it is not.
//===----------------------------------------------------------------------===//

/// `convex_radius` rounds the box corners for cheaper, more stable collision;
/// it must be at most the smallest half extent. `density` is in kg/m^3.
ZJOLT_API ZJoltResult zjoltShapeCreateBox(
    const ZJoltVec3 *half_extent, float convex_radius, float density,
    const ZJoltPhysicsMaterial *material, ZJoltShape **out);

ZJOLT_API ZJoltResult zjoltShapeCreateSphere(
    float radius, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out);

/// A capsule along the Y axis: a cylinder of `half_height_of_cylinder` capped
/// by hemispheres of `radius`. Total height is 2*(half_height + radius).
ZJOLT_API ZJoltResult zjoltShapeCreateCapsule(
    float half_height_of_cylinder, float radius, float density,
    const ZJoltPhysicsMaterial *material, ZJoltShape **out);

/// Builds the convex hull OF the given points; interior points are allowed and
/// discarded. `hull_tolerance` is how far a point may sit outside the hull
/// (larger yields fewer vertices); pass 0 for Jolt's default.
ZJOLT_API ZJoltResult zjoltShapeCreateConvexHull(
    const ZJoltVec3 *points, uint32_t num_points, float max_convex_radius,
    float hull_tolerance, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out);

/// A cylinder along the Y axis, from (0, -half_height, 0) to
/// (0, half_height, 0). `convex_radius` rounds the rim without growing the
/// cylinder; pass 0 for Jolt's default.
ZJOLT_API ZJoltResult zjoltShapeCreateCylinder(
    float half_height, float radius, float convex_radius, float density,
    const ZJoltPhysicsMaterial *material, ZJoltShape **out);

/// A single triangle, wound counter-clockwise.
///
/// Infinitely thin except in shape-versus-shape collision, where
/// `convex_radius` gives it thickness; pass 0 to keep it thin, which is Jolt's
/// own default for this shape rather than the usual 0.05.
///
/// This is a query shape more than a body shape: for a world made of triangles
/// use a mesh, which puts them in a tree instead of one body each.
ZJOLT_API ZJoltResult zjoltShapeCreateTriangle(
    const ZJoltVec3 *v1, const ZJoltVec3 *v2, const ZJoltVec3 *v3,
    float convex_radius, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out);

/// A capsule whose two caps have different radii, centred on the origin, with
/// the `top_radius` cap at (0, half_height_of_tapered_cylinder, 0).
///
/// Jolt SIMPLIFIES this one as it builds: when either sphere fully contains
/// the other, the result is a sphere, or a rotated-translated sphere. So
/// zjoltShapeGetSubType can legitimately answer something other than
/// TAPERED_CAPSULE for a shape built here, and comparing against it is not a
/// way to check that this call was the one that built the shape.
ZJOLT_API ZJoltResult zjoltShapeCreateTaperedCapsule(
    float half_height_of_tapered_cylinder, float top_radius,
    float bottom_radius, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out);

/// A cylinder whose two ends have different radii, centred on the origin, with
/// the `top_radius` end at (0, half_height, 0).
///
/// Simplified the way a tapered capsule is: equal radii yield a plain
/// CYLINDER.
ZJOLT_API ZJoltResult zjoltShapeCreateTaperedCylinder(
    float half_height, float top_radius, float bottom_radius,
    float convex_radius, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out);

/// A half space: everything on the negative side of the plane
/// `dot(x, normal) + constant = 0` is solid.
///
/// Static or kinematic only — a half space has no volume to give a dynamic
/// body mass, and Jolt refuses one. `half_extent` bounds it for the broad
/// phase: it behaves as an infinite plane inside that box, inconsistently at
/// the boundary, and not at all outside, so keep it as small as the world
/// allows and reach for a box when you want something of a defined size. Pass
/// 0 for Jolt's default of 1000.
///
/// `normal` must be unit length; one that is not is refused rather than
/// normalised, because unlike a body's rotation this is authored data and a
/// silently rescaled plane sits somewhere other than where `constant` says.
ZJOLT_API ZJoltResult zjoltShapeCreatePlane(
    const ZJoltVec3 *normal, float constant, float half_extent,
    const ZJoltPhysicsMaterial *material, ZJoltShape **out);

/// A shape with no volume that collides with nothing.
///
/// For a body that must exist before its geometry is known, or one that is
/// only there to be attached to. Put it in an object layer that collides with
/// nothing as well, so the broad phase rejects it before the narrow phase has
/// to. `center_of_mass` may be NULL for the origin.
///
/// There is no material argument because there is nothing to hit: Jolt hard
/// wires this shape's material to the shared default.
ZJOLT_API ZJoltResult zjoltShapeCreateEmpty(const ZJoltVec3 *center_of_mass,
                                            ZJoltShape **out);

/// A static triangle mesh. `indices` holds 3*num_triangles vertex indices.
/// Duplicate and degenerate triangles are removed by Jolt during the build.
///
/// A mesh shape may only be used by a static or kinematic body — Jolt has no
/// inertia for one. Building it is the expensive part of collision cooking, so
/// this is the shape most worth saving with zjoltShapeSave.
///
/// `triangle_materials` holds one index per triangle into `materials`, or is
/// NULL for a mesh with no materials of its own. At most 32 materials fit,
/// because a mesh stores the index in five bits of a per-triangle flag byte.
///
/// A sub-shape id from a hit on this mesh names the triangle AFTER Jolt's own
/// spatial reordering, so it is meaningful to zjoltShapeGetMaterial and to
/// nothing else — in particular it is not an index into `indices`.
ZJOLT_API ZJoltResult zjoltShapeCreateMesh(
    const ZJoltVec3 *vertices, uint32_t num_vertices, const uint32_t *indices,
    uint32_t num_triangles, const uint32_t *triangle_materials,
    const ZJoltPhysicsMaterial *const *materials, uint32_t num_materials,
    uint32_t max_triangles_per_leaf, ZJoltShape **out);

/// A static height field of `sample_count` x `sample_count` samples, laid out
/// row major so the sample at (x, y) is `samples[y * sample_count + x]`.
///
/// The surface is `offset + scale * (x, samples[...], y)`. A sample of
/// ZJOLT_HEIGHT_FIELD_NO_COLLISION punches a hole.
///
/// `block_size` may be 0 for Jolt's default of 2, otherwise it is in [2, 8];
/// `bits_per_sample` may be 0 for Jolt's default of 8, otherwise it is in
/// [1, 16], trading memory for precision. `sample_count` need not be a
/// multiple of `block_size` — Jolt rounds it up and pads the difference with
/// holes — but the rounded count divided by `block_size` must be at least 2,
/// which makes 4 the smallest useful `sample_count` at the default block size.
///
/// `material_indices` holds one index per QUAD — (sample_count - 1)^2 of them,
/// not one per sample — into `materials`, or is NULL for a field with no
/// materials of its own. At most 256 materials fit.
ZJOLT_API ZJoltResult zjoltShapeCreateHeightField(
    const float *samples, uint32_t sample_count, const ZJoltVec3 *offset,
    const ZJoltVec3 *scale, const uint8_t *material_indices,
    const ZJoltPhysicsMaterial *const *materials, uint32_t num_materials,
    uint32_t block_size, uint32_t bits_per_sample, ZJoltShape **out);

/// The height sample that means "no collision here", punching a hole in a
/// height field. Jolt's own FLT_MAX sentinel, republished so a host does not
/// have to know that is what it is.
#define ZJOLT_HEIGHT_FIELD_NO_COLLISION 3.402823466e+38f

/// One child of a compound shape, positioned in the parent's local space.
typedef struct ZJoltCompoundChild {
  /// Borrowed for the duration of the call; the compound takes its own
  /// reference.
  const ZJoltShape *shape;
  ZJoltVec3 position;
  ZJoltQuat rotation;
  /// Opaque to the library. Read back with zjoltShapeCompoundGetChildUserData.
  uint32_t user_data;
} ZJoltCompoundChild;

/// A compound whose children are fixed once built, stored in a tree.
///
/// Jolt SIMPLIFIES the one-child case: a single child at the origin with no
/// rotation comes back as that child itself, and one that is moved or rotated
/// comes back as a rotated-translated shape. So zjoltShapeGetSubType can
/// answer something other than STATIC_COMPOUND for a shape built here.
/// `num_children` of 0 is refused, because a compound cannot encode it.
ZJOLT_API ZJoltResult zjoltShapeCreateStaticCompound(
    const ZJoltCompoundChild *children, uint32_t num_children,
    ZJoltShape **out);

/// A compound whose children can be added, removed and moved after the fact.
///
/// Cheaper to modify and more expensive to query than a static compound; reach
/// for it when the shape genuinely changes, not merely because it is built at
/// run time. Always exactly a MUTABLE_COMPOUND, and never simplified.
///
/// AT LEAST TWO CHILDREN, and that is upstream's constraint rather than a
/// preference. A compound sizes the index field of its sub-shape ids as
/// `32 - CountLeadingZeros(count - 1)`; Jolt's CountLeadingZeros guards a zero
/// argument on x86 but not on ARM, where it is a bare `__builtin_clz` and zero
/// is undefined. One child makes that argument zero, and none underflows the
/// subtraction before it. A static compound never runs into this because Jolt
/// simplifies one child away; this one does not simplify, so the floor is
/// enforced here. See UPSTREAM.md.
ZJOLT_API ZJoltResult zjoltShapeCreateMutableCompound(
    const ZJoltCompoundChild *children, uint32_t num_children,
    ZJoltShape **out);

/// Children of a compound shape, or 0 for any other kind of shape.
ZJOLT_API uint32_t zjoltShapeCompoundGetNumChildren(const ZJoltShape *shape);

/// The user data a child was added with, or 0 if `index` is out of range.
ZJOLT_API uint32_t zjoltShapeCompoundGetChildUserData(const ZJoltShape *shape,
                                                      uint32_t index);

//===----------------------------------------------------------------------===//
// Mutating a compound shape
//
// These are the one place a shape is not immutable, and they carry the
// obligations that implies. All of them:
//
//   * are NOT thread safe against a query, a step, or each other. A shape a
//     body is using must be modified under zjoltBodyLockWrite — or, better and
//     what Jolt itself recommends, not modified at all: build a fresh compound
//     and swap it in with zjoltBodySetShape, so a query already running keeps
//     the old one alive through its own reference.
//   * INVALIDATE every sub-shape id into this shape. Indices shift, and an id
//     cached from an earlier frame now names a different child or none.
//   * require zjoltBodyNotifyShapeChanged on each body using the shape, or the
//     broad phase and the contact cache go on describing the old geometry.
//
// They return ZJOLT_RESULT_INVALID_ARGUMENT for any shape that is not a
// mutable compound, and for an index out of range. The range check is not
// belt-and-braces: Jolt's own RemoveShape and ModifyShape index their array
// with no check at all, so an out-of-range index there is not an assertion but
// a write past the end.
//===----------------------------------------------------------------------===//

/// Appends a child and reports its index.
ZJOLT_API ZJoltResult zjoltShapeMutableCompoundAddChild(
    ZJoltShape *shape, const ZJoltCompoundChild *child, uint32_t *out_index);

/// Removes the child at `index`, shifting the ones after it down.
///
/// Refuses to go below two children, for the reason
/// zjoltShapeCreateMutableCompound gives: at one, Jolt's sub-shape id bit
/// count reaches `CountLeadingZeros(0)`, which is undefined on ARM.
ZJOLT_API ZJoltResult zjoltShapeMutableCompoundRemoveChild(ZJoltShape *shape,
                                                           uint32_t index);

/// Moves and reorients the child at `index`. `rotation` may be NULL to leave
/// the child unrotated.
ZJOLT_API ZJoltResult zjoltShapeMutableCompoundMoveChild(
    ZJoltShape *shape, uint32_t index, const ZJoltVec3 *position,
    const ZJoltQuat *rotation);

/// Recomputes the centre of mass and shifts the children around it.
///
/// Needed after changing the children of a shape a DYNAMIC body uses, or the
/// body spins about a point that is no longer its balance point. The centre of
/// mass moves, so read it with zjoltShapeGetCenterOfMass BEFORE calling this
/// and hand the old value to zjoltBodyNotifyShapeChanged.
ZJOLT_API ZJoltResult zjoltShapeMutableCompoundAdjustCenterOfMass(
    ZJoltShape *shape);

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

/// ZJOLT_SHAPE_SUB_TYPE_OTHER for a NULL shape, and for the sixteen `User*`
/// slots Jolt reserves for shape types registered by C++ outside this library
/// — which this ABI cannot construct, but which could still arrive on a handle
/// that did not come from it.
ZJOLT_API ZJoltShapeSubType zjoltShapeGetSubType(const ZJoltShape *shape);

/// The material of one leaf of `shape`. Never NULL for a valid shape.
///
/// `sub_shape_id` comes from a hit — ZJoltRayCastHit::sub_shape_id and its
/// friends — or from a contact manifold. For a shape with no leaves, which is
/// every convex primitive and a plane, pass ZJOLT_SUB_SHAPE_ID_EMPTY: Jolt
/// ASSERTS that the id is empty there, so passing 0 aborts a build with
/// asserts on instead of returning anything.
///
/// An id that names a child a compound does not have asserts the same way, so
/// this is for ids Jolt handed you, not ids you composed.
///
/// A shape built without a material answers with the shared default rather
/// than NULL; compare against zjoltPhysicsMaterialDefault() to tell those
/// apart. The reference is borrowed — the shape owns it — so take one with
/// zjoltPhysicsMaterialAddRef to outlive the shape.
ZJOLT_API const ZJoltPhysicsMaterial *zjoltShapeGetMaterial(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id);

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
/// ZJOLT_RESULT_BUFFER_TOO_SMALL along with what was needed.
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
/// buffer, or a failed checksum is ZJOLT_RESULT_BAD_FORMAT; zjoltLastError says
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
