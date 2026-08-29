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
// Reference count one on return; a body takes its own reference on attach.
// NULL `material` installs Jolt's shared default; see zjolt_material.h.
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

/// Builds the convex hull of `points`; interior points are discarded.
/// `hull_tolerance` bounds how far a point may sit outside the hull (larger
/// yields fewer vertices); pass 0 for Jolt's default.
/// `max_error_convex_radius` bounds how far the shrunk hull plus its convex
/// radius may sit from the true hull; pass 0 for Jolt's default (0.05). The
/// radius is lowered automatically if `max_convex_radius` would exceed this.
ZJOLT_API ZJoltResult zjoltShapeCreateConvexHull(
    const ZJoltVec3 *points, uint32_t num_points, float max_convex_radius,
    float hull_tolerance, float max_error_convex_radius, float density,
    const ZJoltPhysicsMaterial *material, ZJoltShape **out);

/// A cylinder along the Y axis, from (0, -half_height, 0) to
/// (0, half_height, 0). `convex_radius` rounds the rim without growing the
/// cylinder; pass 0 for Jolt's default.
ZJOLT_API ZJoltResult zjoltShapeCreateCylinder(
    float half_height, float radius, float convex_radius, float density,
    const ZJoltPhysicsMaterial *material, ZJoltShape **out);

/// A single triangle, wound counter-clockwise. Infinitely thin except in
/// shape-versus-shape collision, where `convex_radius` gives it thickness;
/// pass 0 to keep it thin (Jolt's default here, unlike the usual 0.05).
///
/// For a world built from many triangles, use a mesh instead of one body
/// per triangle.
ZJOLT_API ZJoltResult zjoltShapeCreateTriangle(
    const ZJoltVec3 *v1, const ZJoltVec3 *v2, const ZJoltVec3 *v3,
    float convex_radius, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out);

/// A capsule whose two caps have different radii, centred on the origin, with
/// the `top_radius` cap at (0, half_height_of_tapered_cylinder, 0).
///
/// Jolt simplifies a degenerate case to a sphere or rotated-translated
/// sphere when one cap fully contains the other: zjoltShapeGetSubType may
/// then report something other than TAPERED_CAPSULE for a shape built here.
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

/// A half space: the negative side of `dot(x, normal) + constant = 0` is
/// solid. `normal` must be unit length — Jolt refuses a non-unit normal.
///
/// Static or kinematic only; a half space has no volume for dynamic-body
/// mass. `half_extent` bounds it for the broad phase only (0 for Jolt's
/// default of 1000) — reach for a box instead when a defined size matters.
ZJOLT_API ZJoltResult zjoltShapeCreatePlane(
    const ZJoltVec3 *normal, float constant, float half_extent,
    const ZJoltPhysicsMaterial *material, ZJoltShape **out);

/// A shape with no volume that collides with nothing — for a body whose
/// geometry is not yet known, or one that exists only to attach to.
/// `center_of_mass` may be NULL for the origin.
///
/// Put it in an object layer that also collides with nothing. No material
/// argument: Jolt hard-wires this shape's material to the shared default.
ZJOLT_API ZJoltResult zjoltShapeCreateEmpty(const ZJoltVec3 *center_of_mass,
                                            ZJoltShape **out);

/// Cosine of Jolt's default active-edge threshold angle (5 degrees): past
/// this angle between two triangles, their shared edge is active. Used by
/// zjoltShapeCreateMesh, zjoltShapeCreateHeightField and
/// zjoltShapeHeightFieldSetHeights. Not a "pass 0" sentinel like other
/// defaultable parameters: 0 and negative values are meaningful settings
/// here, so match Jolt's default explicitly.
#define ZJOLT_SHAPE_DEFAULT_ACTIVE_EDGE_COS_THRESHOLD_ANGLE 0.996195f

/// How hard MeshShapeSettings::Create works to build a well balanced tree.
/// FAVOR_BUILD_SPEED trades runtime query performance for a faster build.
typedef enum ZJoltMeshBuildQuality {
  ZJOLT_MESH_BUILD_QUALITY_FAVOR_RUNTIME_PERFORMANCE = 0,
  ZJOLT_MESH_BUILD_QUALITY_FAVOR_BUILD_SPEED = 1,
} ZJoltMeshBuildQuality;

/// A static triangle mesh for a static or kinematic body only — Jolt has no
/// inertia for one. `indices` holds 3*num_triangles vertex indices;
/// `triangle_materials` indexes `materials` per triangle (NULL for none, at
/// most 32). `triangle_user_data`, if not NULL, is read back with
/// zjoltShapeMeshGetTriangleUserData. A hit's sub-shape id names the
/// triangle AFTER Jolt's own reordering, not an index into `indices`.
ZJOLT_API ZJoltResult zjoltShapeCreateMesh(
    const ZJoltVec3 *vertices, uint32_t num_vertices, const uint32_t *indices,
    uint32_t num_triangles, const uint32_t *triangle_materials,
    const uint32_t *triangle_user_data,
    const ZJoltPhysicsMaterial *const *materials, uint32_t num_materials,
    uint32_t max_triangles_per_leaf, float active_edge_cos_threshold_angle,
    ZJoltMeshBuildQuality build_quality, ZJoltShape **out);

/// Passed as `min_height_value`/`max_height_value` together, this pair asks
/// Jolt to derive the quantisation range from `samples` itself instead of
/// reserving one — zjoltShapeCreateHeightField's only behaviour before these
/// two parameters existed. They are Jolt's own sentinel (HeightFieldShape.h's
/// cLargeFloat), not a value this binding invented.
#define ZJOLT_HEIGHT_FIELD_AUTO_MIN_HEIGHT_VALUE 1.0e15f
#define ZJOLT_HEIGHT_FIELD_AUTO_MAX_HEIGHT_VALUE (-1.0e15f)

/// A static `sample_count` x `sample_count` height field, row major:
/// `samples[y*sample_count+x]`; surface at `offset + scale*(x,samples,y)`.
/// ZJOLT_HEIGHT_FIELD_NO_COLLISION punches a hole. `block_size` (0 for
/// default 2) is [2, 8]; `bits_per_sample` (0 for default 8) is [1, 16].
/// `material_indices`: one index per quad into `materials` (NULL, max 256).
/// `min_height_value`/`max_height_value`: @see zjoltShapeHeightFieldSetHeights.
ZJOLT_API ZJoltResult zjoltShapeCreateHeightField(
    const float *samples, uint32_t sample_count, const ZJoltVec3 *offset,
    const ZJoltVec3 *scale, const uint8_t *material_indices,
    const ZJoltPhysicsMaterial *const *materials, uint32_t num_materials,
    uint32_t block_size, uint32_t bits_per_sample, float min_height_value,
    float max_height_value, float active_edge_cos_threshold_angle,
    ZJoltShape **out);

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
/// `num_children` of 0 is refused; a compound cannot encode it.
///
/// Jolt simplifies a single unmoved, unrotated child to that child itself,
/// and a single moved/rotated one to a rotated-translated shape:
/// zjoltShapeGetSubType may then report something other than STATIC_COMPOUND.
ZJOLT_API ZJoltResult zjoltShapeCreateStaticCompound(
    const ZJoltCompoundChild *children, uint32_t num_children,
    ZJoltShape **out);

/// A compound whose children can be added, removed and moved after the
/// fact — cheaper to modify and more expensive to query than a static
/// compound. Always exactly MUTABLE_COMPOUND; never simplified.
///
/// Refuses fewer than two children: at one, Jolt's sub-shape id bit count
/// computation is undefined on ARM (CountLeadingZeros(0)). See UPSTREAM.md.
ZJOLT_API ZJoltResult zjoltShapeCreateMutableCompound(
    const ZJoltCompoundChild *children, uint32_t num_children,
    ZJoltShape **out);

/// Children of a compound shape, or 0 for any other kind of shape.
ZJOLT_API uint32_t zjoltShapeCompoundGetNumChildren(const ZJoltShape *shape);

/// The user data a child was added with, or 0 if `index` is out of range.
ZJOLT_API uint32_t zjoltShapeCompoundGetChildUserData(const ZJoltShape *shape,
                                                      uint32_t index);

/// Changes the user data a child was added with — a plain data field,
/// independent of zjoltShapeMutableCompoundMoveChild. Works on a STATIC
/// compound too; not gated behind mutability.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT for a non-compound or out-of-range
/// `index` — Jolt indexes with no bounds check, risking a write past the end.
ZJOLT_API ZJoltResult zjoltShapeCompoundSetChildUserData(ZJoltShape *shape,
                                                         uint32_t index,
                                                         uint32_t user_data);

/// A running SubShapeIDCreator: composes a multi-level sub-shape id one
/// level at a time, needed to address a grandchild (or deeper) of a nested
/// compound — zjoltShapeGetSubShapeIDFromIndex only reaches a direct child.
///
/// Opaque and owned outright, not reference counted: Destroy once, never
/// Release. zjoltSubShapeIdCreatorCreate starts one at the root.
typedef struct ZJoltSubShapeIdCreator ZJoltSubShapeIdCreator;

ZJOLT_API ZJoltResult zjoltSubShapeIdCreatorCreate(ZJoltSubShapeIdCreator **out);
ZJOLT_API void zjoltSubShapeIdCreatorDestroy(ZJoltSubShapeIdCreator *creator);

/// Direct bind of SubShapeIDCreator::PushID: advances `creator` in place by
/// `bits` bits set to `value`, the mechanism Jolt's own compound, mesh,
/// height-field and soft-body shapes build multi-level ids with.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT if `value` is not representable in `bits`
/// bits (< 2^bits), or the bits written so far would exceed 32.
ZJOLT_API ZJoltResult zjoltSubShapeIdCreatorPushID(
    ZJoltSubShapeIdCreator *creator, uint32_t value, uint32_t bits);

/// The id `creator` currently holds -- SubShapeIDCreator::GetID().
ZJOLT_API ZJoltSubShapeId zjoltSubShapeIdCreatorGetID(
    const ZJoltSubShapeIdCreator *creator);

/// How many bits of `creator`'s id are spoken for so far --
/// SubShapeIDCreator::GetNumBitsWritten().
ZJOLT_API uint32_t zjoltSubShapeIdCreatorGetNumBitsWritten(
    const ZJoltSubShapeIdCreator *creator);

/// Pops `bits` bits off the front of `id`, parents before children -- the
/// decode direction of zjoltSubShapeIdCreatorPushID. `out_value` gets the
/// popped bits; `out_remainder` gets what remains of `id`, for decoding
/// the next level of a nested compound, mesh, or height field by hand.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT for `bits` above 32.
ZJOLT_API ZJoltResult zjoltSubShapeIdPopID(ZJoltSubShapeId id, uint32_t bits,
                                          uint32_t *out_value,
                                          ZJoltSubShapeId *out_remainder);

/// The sub-shape id addressing `shape`'s direct child `index`, from
/// `shape`'s own root. Inverse of zjoltShapeGetSubShapeIndexFromID; for a
/// grandchild (or deeper) use zjoltShapeGetSubShapeIDFromIndexInto instead.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT for a shape that is not a compound, or for
/// an out-of-range `index`.
ZJOLT_API ZJoltResult zjoltShapeGetSubShapeIDFromIndex(
    const ZJoltShape *shape, uint32_t index, ZJoltSubShapeId *out);

/// As zjoltShapeGetSubShapeIDFromIndex, but composes onto `creator` in
/// place instead of starting at the root — addresses a nested compound's
/// grandchild (or deeper) by calling once per level, outermost first,
/// threading the same `creator` through.
///
/// Same failure modes as zjoltShapeGetSubShapeIDFromIndex.
ZJOLT_API ZJoltResult zjoltShapeGetSubShapeIDFromIndexInto(
    const ZJoltShape *shape, uint32_t index,
    ZJoltSubShapeIdCreator *creator);

/// The inverse of zjoltShapeGetSubShapeIDFromIndex: which direct child
/// `sub_shape_id` names, and what remains of the id after removing the
/// path to it — meaningful when that child is itself a compound or mesh.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT for a shape that is not a compound, or a
/// `sub_shape_id` that does not name one of its direct children.
ZJOLT_API ZJoltResult zjoltShapeGetSubShapeIndexFromID(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id,
    uint32_t *out_index, ZJoltSubShapeId *out_remainder);

/// Which of `shape`'s direct children have a bounding box overlapping
/// `box`, both in `shape`'s own local space.
///
/// `out_indices` must hold at least zjoltShapeCompoundGetNumChildren(shape)
/// entries; NULL reports that count in `*out_count` without writing.
/// ZJOLT_RESULT_INVALID_ARGUMENT for a non-compound, ZJOLT_RESULT_BUFFER_TOO_SMALL if short.
ZJOLT_API ZJoltResult zjoltShapeGetIntersectingSubShapes(
    const ZJoltShape *shape, const ZJoltAABox *box, uint32_t *out_indices,
    uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Mutating a compound shape
//
// Not thread safe against a query, a step, or each other; modify under zjoltBodyLockWrite.
// Invalidates every sub-shape id; call zjoltBodyNotifyShapeChanged afterward.
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

/// Moves, reorients, AND swaps the shape at `index` for `new_shape` — prefer
/// this over remove-and-re-add, which shifts later children's sub-shape ids.
/// `rotation` may be NULL to leave the child unrotated.
///
/// Borrowed for the call; the compound takes its own reference on
/// `new_shape`, same as when a child is first added.
ZJOLT_API ZJoltResult zjoltShapeMutableCompoundReplaceChild(
    ZJoltShape *shape, uint32_t index, const ZJoltShape *new_shape,
    const ZJoltVec3 *position, const ZJoltQuat *rotation);

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

/// The shape immediately inside a scaled, rotated-translated, or offset-
/// center-of-mass wrapper, one level only — unlike zjoltShapeGetLeafShape,
/// which drills through every wrapper down to a non-decorated leaf.
///
/// Borrowed: valid as long as `shape` is; zjoltShapeAddRef to outlive it.
/// ZJOLT_RESULT_INVALID_ARGUMENT for a shape that is not decorated.
ZJOLT_API ZJoltResult zjoltShapeGetInnerShape(const ZJoltShape *shape,
                                              const ZJoltShape **out);

ZJOLT_API void zjoltShapeAddRef(const ZJoltShape *shape);
ZJOLT_API void zjoltShapeRelease(const ZJoltShape *shape);
ZJOLT_API uint32_t zjoltShapeGetRefCount(const ZJoltShape *shape);

/// Opaque to the library, and 0 until set. Every `*ShapeSettings` class
/// inherits one of these, but the settings object itself never crosses this
/// boundary (see the file comment), so it cannot be set at construction —
/// this is the only way to reach it, and it works uniformly for every shape
/// kind rather than needing a parameter on fourteen different constructors.
/// 0 for a NULL `shape`.
ZJOLT_API uint64_t zjoltShapeGetUserData(const ZJoltShape *shape);

/// Does nothing for a NULL `shape`.
ZJOLT_API void zjoltShapeSetUserData(ZJoltShape *shape, uint64_t user_data);

/// ZJOLT_SHAPE_SUB_TYPE_OTHER for a NULL shape, and for the sixteen `User*`
/// slots Jolt reserves for shape types registered by C++ outside this library
/// — which this ABI cannot construct, but which could still arrive on a handle
/// that did not come from it.
ZJOLT_API ZJoltShapeSubType zjoltShapeGetSubType(const ZJoltShape *shape);

/// The material of one leaf of `shape`. Never NULL for a valid shape; one
/// built without a material answers with the shared default — compare
/// against zjoltPhysicsMaterialDefault() to tell those apart.
///
/// `sub_shape_id` comes from a hit or contact manifold, not one composed by
/// hand; pass ZJOLT_SUB_SHAPE_ID_EMPTY for a shape with no leaves. Borrowed.
ZJOLT_API const ZJoltPhysicsMaterial *zjoltShapeGetMaterial(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id);

/// Replaces a convex primitive's material. NULL installs Jolt's shared
/// default. Live edit to shared state: every body built from `shape` sees
/// the change immediately, since a body adds a reference rather than copies.
/// Drops the old material reference and takes one on `material`; the
/// caller's own reference is unaffected. ZJOLT_RESULT_INVALID_ARGUMENT for
/// a shape that is not a convex primitive.
ZJOLT_API ZJoltResult zjoltShapeSetMaterial(
    ZJoltShape *shape, const ZJoltPhysicsMaterial *material);

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

//===----------------------------------------------------------------------===//
// Convex-primitive dimension introspection
//
// Each getter below returns ZJOLT_RESULT_INVALID_ARGUMENT for a shape kind
// it does not apply to, except zjoltShapeGetInnerRadius/GetPlane.
//===----------------------------------------------------------------------===//

/// The radius of the largest sphere that fits inside `shape`, used as Jolt's
/// own convex-collision margin. Defined for every shape kind — 0 for one with
/// no meaningful interior, such as a plane or an empty shape — so unlike the
/// rest of this section it needs no subtype check. 0 for a NULL `shape`.
ZJOLT_API float zjoltShapeGetInnerRadius(const ZJoltShape *shape);

/// A convex shape's density in kg/m^3, as it stands right now. Changing it
/// does not retroactively rescale a body already built from this shape:
/// Jolt bakes mass properties in at body creation, so an existing body
/// needs rebuilding or a mass override to reflect the change.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT for a shape that is not a convex primitive.
ZJOLT_API ZJoltResult zjoltShapeGetDensity(const ZJoltShape *shape,
                                           float *out_density);
ZJOLT_API ZJoltResult zjoltShapeSetDensity(ZJoltShape *shape, float density);

/// Sphere, capsule or cylinder radius. ZJOLT_RESULT_INVALID_ARGUMENT for any
/// other kind, including a box or a tapered capsule/cylinder — those round a
/// different dimension, see zjoltShapeGetConvexRadius and
/// zjoltShapeGetTopRadius/zjoltShapeGetBottomRadius.
ZJOLT_API ZJoltResult zjoltShapeGetRadius(const ZJoltShape *shape,
                                          float *out_radius);

/// A box's half extent, the same value zjoltShapeCreateBox took.
/// ZJOLT_RESULT_INVALID_ARGUMENT for any other kind.
ZJOLT_API ZJoltResult zjoltShapeGetHalfExtent(const ZJoltShape *shape,
                                              ZJoltVec3 *out_half_extent);

/// Cylinder, tapered capsule or tapered cylinder half height.
/// ZJOLT_RESULT_INVALID_ARGUMENT for any other kind — a plain capsule's is
/// zjoltShapeGetHalfHeightOfCylinder instead, because Jolt gives that one a
/// distinct name (the capsule's own half height still excludes the
/// hemispherical caps zjoltShapeCreateCapsule's `radius` adds).
ZJOLT_API ZJoltResult zjoltShapeGetHalfHeight(const ZJoltShape *shape,
                                              float *out_half_height);

/// A capsule's half height of cylinder, the same value
/// zjoltShapeCreateCapsule took. ZJOLT_RESULT_INVALID_ARGUMENT for any other
/// kind.
ZJOLT_API ZJoltResult zjoltShapeGetHalfHeightOfCylinder(
    const ZJoltShape *shape, float *out_half_height_of_cylinder);

/// A tapered capsule or tapered cylinder's radius at its `+half_height` end.
/// ZJOLT_RESULT_INVALID_ARGUMENT for any other kind — including a shape
/// zjoltShapeCreateTaperedCapsule/zjoltShapeCreateTaperedCylinder SIMPLIFIED
/// into a sphere or a plain cylinder, which no longer has one.
ZJOLT_API ZJoltResult zjoltShapeGetTopRadius(const ZJoltShape *shape,
                                             float *out_top_radius);

/// The radius at the `-half_height` end. @see zjoltShapeGetTopRadius.
ZJOLT_API ZJoltResult zjoltShapeGetBottomRadius(const ZJoltShape *shape,
                                                float *out_bottom_radius);

/// The convex radius a box, cylinder, convex hull or tapered cylinder rounds
/// its edges by. ZJOLT_RESULT_INVALID_ARGUMENT for any other kind — a sphere
/// or capsule has no separate convex radius, because its own radius already
/// plays that role, and a tapered capsule has none at all.
ZJOLT_API ZJoltResult zjoltShapeGetConvexRadius(const ZJoltShape *shape,
                                                float *out_convex_radius);

/// A convex hull's face count. ZJOLT_RESULT_INVALID_ARGUMENT for any other
/// kind.
ZJOLT_API ZJoltResult zjoltShapeGetNumFaces(const ZJoltShape *shape,
                                            uint32_t *out_num_faces);

/// The number of vertices in convex hull face `face_index`.
/// ZJOLT_RESULT_INVALID_ARGUMENT for any other kind, or for a `face_index` at
/// or beyond zjoltShapeGetNumFaces — Jolt indexes its own face array with no
/// bounds check at all, so out of range there is not an assert but a read
/// past the end.
ZJOLT_API ZJoltResult zjoltShapeGetNumVerticesInFace(
    const ZJoltShape *shape, uint32_t face_index, uint32_t *out_num_vertices);

/// A convex hull's own vertices, relative to `shape`'s centre of mass.
/// ZJOLT_RESULT_INVALID_ARGUMENT for any other kind. Two-call protocol like
/// zjoltShapeGetMaterialList: pass `out_points` = NULL to learn the count.
ZJOLT_API ZJoltResult zjoltShapeGetPoints(const ZJoltShape *shape,
                                          ZJoltVec3 *out_points,
                                          uint32_t capacity,
                                          uint32_t *out_count);

/// A plane, as a unit normal and the signed distance from the origin along
/// it: `dot(x, normal) + constant = 0` on the plane, matching what
/// zjoltShapeCreatePlane took.
typedef struct ZJoltPlane {
  ZJoltVec3 normal;
  float constant;
} ZJoltPlane;

/// A convex hull's own face planes, relative to `shape`'s centre of mass.
/// ZJOLT_RESULT_INVALID_ARGUMENT for any other kind — in particular NOT a
/// plane shape's plane, which is zjoltShapeGetPlane (singular). Two-call
/// protocol like zjoltShapeGetMaterialList.
ZJOLT_API ZJoltResult zjoltShapeGetPlanes(const ZJoltShape *shape,
                                          ZJoltPlane *out_planes,
                                          uint32_t capacity,
                                          uint32_t *out_count);

/// A plane shape's own plane equation and bounding half extent — both values
/// zjoltShapeCreatePlane took. ZJOLT_RESULT_INVALID_ARGUMENT for any other
/// kind. There is no separate vertex read-back: Jolt keeps the four corners
/// of the bounded quad this describes as a PRIVATE helper with no accessor,
/// so the caller reconstructs them from `plane` and `half_extent` directly —
/// the same two values Jolt's own private helper starts from.
ZJOLT_API ZJoltResult zjoltShapeGetPlane(const ZJoltShape *shape,
                                         ZJoltPlane *out_plane,
                                         float *out_half_extent);

//===----------------------------------------------------------------------===//
// Convex support function
//
// Jolt/Geometry/GJKClosestPoint.h and EPAPenetrationDepth.h consume one
// through zjolt_geometry.h's ZJoltConvexSupport, whose `support` callback
// matches zjoltShapeSupportFunctionGetSupport's own shape.
//===----------------------------------------------------------------------===//

/// How zjoltShapeGetSupportFunction folds in a convex primitive's rounding
/// radius.
typedef enum ZJoltShapeSupportMode {
  /// GetSupport excludes the radius; GetConvexRadius reports it separately,
  /// capped at Jolt's own 0.05 regardless of the shape's own convex radius.
  ZJOLT_SHAPE_SUPPORT_MODE_EXCLUDE_CONVEX_RADIUS = 0,
  /// GetSupport includes the radius; GetConvexRadius reports 0.
  ZJOLT_SHAPE_SUPPORT_MODE_INCLUDE_CONVEX_RADIUS = 1,
  /// Whichever combination is most accurate/efficient for this shape kind.
  ZJOLT_SHAPE_SUPPORT_MODE_DEFAULT = 2,
} ZJoltShapeSupportMode;

/// Scratch space zjoltShapeGetSupportFunction places its support object
/// into. Sized and aligned to match Jolt's own `ConvexShape::SupportBuffer`
/// exactly. Must stay alive and untouched for as long as the returned
/// handle is used; there is nothing to release.
#if defined(__cplusplus)
#define ZJOLT_SUPPORT_ALIGNAS_16 alignas(16)
#else
#define ZJOLT_SUPPORT_ALIGNAS_16 _Alignas(16)
#endif
typedef struct ZJoltShapeSupportBuffer {
  ZJOLT_SUPPORT_ALIGNAS_16 uint8_t data[4160];
} ZJoltShapeSupportBuffer;
#undef ZJOLT_SUPPORT_ALIGNAS_16

typedef struct ZJoltShapeSupportFunction ZJoltShapeSupportFunction;

/// Places a support function for `shape` inside `buffer`, for GJK/EPA.
/// `scale` may be NULL for (1, 1, 1). Borrowed: valid as long as `buffer`
/// is alive and untouched.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT for a shape that is not a convex
/// primitive.
ZJOLT_API ZJoltResult zjoltShapeGetSupportFunction(
    const ZJoltShape *shape, ZJoltShapeSupportMode mode,
    ZJoltShapeSupportBuffer *buffer, const ZJoltVec3 *scale,
    const ZJoltShapeSupportFunction **out);

/// The support point along `direction`, relative to the shape's own centre
/// of mass. Zeroed for a NULL `support` or `direction`.
ZJOLT_API void zjoltShapeSupportFunctionGetSupport(
    const ZJoltShapeSupportFunction *support, const ZJoltVec3 *direction,
    ZJoltVec3 *out);

/// The convex radius this support function folds in — 0 in
/// ZJOLT_SHAPE_SUPPORT_MODE_INCLUDE_CONVEX_RADIUS mode. 0 for a NULL
/// `support`.
ZJOLT_API float zjoltShapeSupportFunctionGetConvexRadius(
    const ZJoltShapeSupportFunction *support);

/// The total and submerged volume of `shape` — placed by `transform` and
/// `scale` (NULL for (1, 1, 1)) — below `surface`, and the world-space
/// centre of mass of the submerged part. Jolt's own inputs for buoyancy.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT if `shape` or anything beneath it is a
/// mesh, a height field, or a plane — none support this in Jolt 5.6.0.
ZJOLT_API ZJoltResult zjoltShapeGetSubmergedVolume(
    const ZJoltShape *shape, const ZJoltMat44 *transform,
    const ZJoltVec3 *scale, const ZJoltPlane *surface,
    float *out_total_volume, float *out_submerged_volume,
    ZJoltVec3 *out_center_of_buoyancy);

//===----------------------------------------------------------------------===//
// Submerged-volume accumulation for an arbitrary convex polyhedron
//
// What zjoltShapeGetSubmergedVolume runs internally for a shape already
// built, exposed over a caller's own point cloud and face list — for
// buoyancy-testing geometry before it becomes a shape.
//===----------------------------------------------------------------------===//

typedef struct ZJoltPolyhedronSubmergedVolumeCalculator
    ZJoltPolyhedronSubmergedVolumeCalculator;

/// Transforms `points` by `transform` and classifies each against `surface`
/// (normal pointing up). Add every face with
/// zjoltPolyhedronSubmergedVolumeCalculatorAddFace, then read the result
/// with zjoltPolyhedronSubmergedVolumeCalculatorGetResult. Release with
/// zjoltPolyhedronSubmergedVolumeCalculatorDestroy.
ZJOLT_API ZJoltResult zjoltPolyhedronSubmergedVolumeCalculatorCreate(
    const ZJoltMat44 *transform, const ZJoltVec3 *points, uint32_t num_points,
    const ZJoltPlane *surface, ZJoltPolyhedronSubmergedVolumeCalculator **out);

ZJOLT_API void zjoltPolyhedronSubmergedVolumeCalculatorDestroy(
    ZJoltPolyhedronSubmergedVolumeCalculator *calc);

/// True once every point sits above `surface` — the submerged volume is
/// zero without adding any faces. False for a NULL `calc`.
ZJOLT_API bool zjoltPolyhedronSubmergedVolumeCalculatorAreAllAbove(
    const ZJoltPolyhedronSubmergedVolumeCalculator *calc);

/// True once every point sits below `surface` — the submerged volume is the
/// whole polyhedron's. False for a NULL `calc`.
ZJOLT_API bool zjoltPolyhedronSubmergedVolumeCalculatorAreAllBelow(
    const ZJoltPolyhedronSubmergedVolumeCalculator *calc);

/// Index into the `points` array given to Create of the point deepest below
/// `surface` (most negative signed distance). AddFace refuses a face using
/// it — its contribution to the volume is always zero, so skip that face
/// instead. 0 for a NULL `calc`.
ZJOLT_API uint32_t zjoltPolyhedronSubmergedVolumeCalculatorGetReferencePointIdx(
    const ZJoltPolyhedronSubmergedVolumeCalculator *calc);

/// Accumulates one triangular face, wound counter-clockwise, naming indices
/// into the `points` array given to Create. Fan-triangulate an N-gon face
/// from any one of its own vertices and call this once per triangle.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT if an index is at or beyond `num_points`,
/// or equals zjoltPolyhedronSubmergedVolumeCalculatorGetReferencePointIdx.
ZJOLT_API ZJoltResult zjoltPolyhedronSubmergedVolumeCalculatorAddFace(
    ZJoltPolyhedronSubmergedVolumeCalculator *calc, uint32_t idx1,
    uint32_t idx2, uint32_t idx3);

/// The accumulated submerged volume and its centre, after every face has
/// been added. Zeroed for a NULL `calc`.
ZJOLT_API void zjoltPolyhedronSubmergedVolumeCalculatorGetResult(
    const ZJoltPolyhedronSubmergedVolumeCalculator *calc,
    float *out_submerged_volume, ZJoltVec3 *out_center_of_buoyancy);

/// Serialises `shape` and everything under it into `buffer`.
///
/// Two-call protocol: buffer = NULL reports the needed size in `*out_size`;
/// a too-small buffer returns ZJOLT_RESULT_BUFFER_TOO_SMALL. A cooking
/// cache, not an interchange or adversarial-safe format: a shape saved by a
/// different Jolt version or precision setting is refused.
ZJOLT_API ZJoltResult zjoltShapeSave(const ZJoltShape *shape, void *buffer,
                                     size_t capacity, size_t *out_size);

/// Rebuilds a shape from zjoltShapeSave output. The buffer is read during the
/// call only. A wrong tag, a wrong build, a length that disagrees with the
/// buffer, or a failed checksum is ZJOLT_RESULT_BAD_FORMAT; zjoltLastError says
/// which.
ZJOLT_API ZJoltResult zjoltShapeRestore(const void *data, size_t size,
                                        ZJoltShape **out);

/// Writes `shape` through `stream` instead of a resident buffer, for
/// streaming a large cook (a mesh, a height field) without holding the
/// whole payload first. ZJOLT_RESULT_IO_ERROR if `stream` reports failure.
///
/// Header carries a magic tag, ZJOLT_CONFIG_ID and the Jolt version stamp,
/// but no length or CRC-32 — less corruption margin than zjoltShapeSave.
ZJOLT_API ZJoltResult zjoltShapeSaveStream(const ZJoltShape *shape,
                                           const ZJoltStream *stream);

/// Rebuilds a shape written by zjoltShapeSaveStream. @see zjoltShapeRestore
/// for what ZJOLT_RESULT_BAD_FORMAT covers; a stream form has no length or
/// checksum to check first, so it is caught only as far as
/// ZJOLT_RESULT_IO_ERROR or ZJOLT_RESULT_BAD_FORMAT reach on their own.
ZJOLT_API ZJoltResult zjoltShapeRestoreStream(const ZJoltStream *stream,
                                              ZJoltShape **out);

/// Bytes zjoltShapeSave prepends to Jolt's payload. Exposed so a test can
/// reach the payload itself, not so callers can build one by hand.
#define ZJOLT_SHAPE_HEADER_SIZE 32

/// Bytes zjoltShapeSaveStream prepends to Jolt's payload — the magic tag,
/// ZJOLT_CONFIG_ID and the Jolt version stamp @see ZJoltStream describes, and
/// nothing past that: shapes have no fields of their own to add. Exposed so a
/// test can reach the payload itself and compare it against
/// ZJOLT_SHAPE_HEADER_SIZE's, which must agree byte for byte.
#define ZJOLT_SHAPE_STREAM_HEADER_SIZE 12

//===----------------------------------------------------------------------===//
// Introspection Jolt puts on every leaf shape
//
// GetSurfaceNormal is what a hit's contact normal resolves from;
// GetSupportingFace is what the narrow phase builds a contact manifold from.
//===----------------------------------------------------------------------===//

typedef struct ZJoltTransformedShape ZJoltTransformedShape;

/// Sub-shape id bits `shape` needs to address any leaf beneath it. 0 for a
/// NULL shape.
ZJOLT_API uint32_t zjoltShapeGetSubShapeIDBits(const ZJoltShape *shape);

/// Whether `sub_shape_id` names something in `shape` that
/// zjoltShapeGetMaterial and friends can safely be given. False for NULL.
///
/// Valid means exactly ZJOLT_SUB_SHAPE_ID_EMPTY for a shape with no leaves;
/// a compound recurses to a leaf. A mesh or height field checks only that
/// unclaimed bits are spent, not that the id names a real triangle or quad.
ZJOLT_API bool zjoltShapeIsSubShapeIDValid(const ZJoltShape *shape,
                                           ZJoltSubShapeId sub_shape_id);

/// The FACE normal at `local_surface_position` on the leaf named by
/// `sub_shape_id`, both relative to `shape`'s own center of mass. Zeroed
/// for a NULL `shape` or `local_surface_position`. Not a vertex or edge
/// normal — for a hit's contact normal use `-penetration_axis` instead.
/// `sub_shape_id`: pass ZJOLT_SUB_SHAPE_ID_EMPTY for a shape with no leaves.
ZJOLT_API void zjoltShapeGetSurfaceNormal(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id,
    const ZJoltVec3 *local_surface_position, ZJoltVec3 *out_normal);

/// Vertices zjoltShapeGetSupportingFace can report in one call. Jolt's own
/// `Shape::SupportingFace` capacity; `out_vertices` must hold this many.
#define ZJOLT_SHAPE_MAX_SUPPORTING_FACE_VERTICES 32

/// The face of the leaf named by `sub_shape_id` that faces `direction`
/// (`shape`'s own local space) the most, placed by `position`/`rotation`/
/// `scale` (`scale` NULL for (1,1,1); `position`/`rotation` required).
///
/// Only convex shapes and triangles have one; anything else (a sphere, an
/// empty shape) reports `*out_count = 0`, not an error.
ZJOLT_API ZJoltResult zjoltShapeGetSupportingFace(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id,
    const ZJoltVec3 *direction, const ZJoltVec3 *scale,
    const ZJoltVec3 *position, const ZJoltQuat *rotation,
    ZJoltVec3 *out_vertices, uint32_t *out_count);

/// The direct child at `sub_shape_id`, and its transform, as a fresh
/// zjolt_transformed.h handle — release with zjoltTransformedShapeDestroy.
/// A shape with no children returns `shape` itself, at this placement.
///
/// `position`/`rotation`/`scale` may be NULL (identity/(1,1,1)).
/// The returned handle's body id is always ZJOLT_BODY_ID_INVALID.
ZJOLT_API ZJoltResult zjoltShapeGetSubShapeTransformedShape(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id,
    const ZJoltVec3 *position, const ZJoltQuat *rotation,
    const ZJoltVec3 *scale, ZJoltTransformedShape **out,
    ZJoltSubShapeId *out_remainder);

/// The innermost real shape at `sub_shape_id`, drilling through every
/// compound and decoration — the identity-transform counterpart of
/// zjoltShapeGetSubShapeTransformedShape. NULL for a NULL `shape`, or if
/// `sub_shape_id` did not resolve to a leaf.
/// Borrowed: valid as long as `shape` is. `out_remainder` (may be NULL)
/// gets what is left of the id.
ZJOLT_API const ZJoltShape *zjoltShapeGetLeafShape(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id,
    ZJoltSubShapeId *out_remainder);

/// A copy of `shape`, scaled by `scale` IN THE SPACE IT WAS CREATED — not the
/// space of its leaves, which is what every other `scale` parameter in this
/// header means. Not every shape supports every scale; see
/// zjoltShapeIsValidScale. Jolt matches the request as closely as it can
/// rather than refusing an unsupported one, so check the shape this produces
/// if the distinction matters.
ZJOLT_API ZJoltResult zjoltShapeScaleShape(const ZJoltShape *shape,
                                           const ZJoltVec3 *scale,
                                           ZJoltShape **out);

/// Whether `scale` can be used directly for `shape` — wrapped in
/// zjoltShapeCreateScaled, for instance — without Jolt substituting something
/// else for it. A sphere or a capsule need a uniform scale; a compound
/// refuses one that would shear a child. False for a NULL `shape` or `scale`.
ZJOLT_API bool zjoltShapeIsValidScale(const ZJoltShape *shape,
                                      const ZJoltVec3 *scale);

/// The scale nearest `scale` that zjoltShapeIsValidScale would accept for
/// `shape` — discarding, for instance, the components that would make a
/// sphere's scale non-uniform. Compare the result against `scale` to detect a
/// caller mistake worth a warning. Echoes `scale` back unchanged for a NULL
/// `shape`; zeroed for a NULL `scale`.
ZJOLT_API void zjoltShapeMakeScaleValid(const ZJoltShape *shape,
                                        const ZJoltVec3 *scale,
                                        ZJoltVec3 *out_scale);

//===----------------------------------------------------------------------===//
// Triangle read-back
//
// NOT count-then-fill: call zjoltShapeGetTrianglesStart once, then
// zjoltShapeGetTrianglesNext until it reports 0 — no more triangles.
//===----------------------------------------------------------------------===//

/// Opaque scratch space for one triangle walk. Must not be touched, moved, or
/// reused for a second walk until the first one reports 0 remaining. Sized
/// and aligned to match Jolt's own `Shape::GetTrianglesContext` exactly,
/// which zjolt_abi.cpp checks at build time.
#if defined(__cplusplus)
#define ZJOLT_ALIGNAS_16 alignas(16)
#else
#define ZJOLT_ALIGNAS_16 _Alignas(16)
#endif
typedef struct ZJoltShapeTrianglesContext {
  ZJOLT_ALIGNAS_16 uint8_t data[4288];
} ZJoltShapeTrianglesContext;
#undef ZJOLT_ALIGNAS_16

/// Fewest triangles zjoltShapeGetTrianglesNext accepts a request for. Jolt
/// asserts on fewer rather than returning a short batch, so this ABI refuses
/// instead of reaching that assert with asserts compiled out.
#define ZJOLT_SHAPE_MIN_TRIANGLES_REQUESTED 32

/// Starts a triangle walk over `shape`, transformed by
/// `position`/`rotation`/`scale` (`scale` NULL for (1, 1, 1)), restricted to
/// `box` in that same space (coarse culling only; results may extend
/// slightly outside it). Only initialises `context`; the walk itself is
/// zjoltShapeGetTrianglesNext.
ZJOLT_API ZJoltResult zjoltShapeGetTrianglesStart(
    const ZJoltShape *shape, ZJoltShapeTrianglesContext *context,
    const ZJoltAABox *box, const ZJoltVec3 *position,
    const ZJoltQuat *rotation, const ZJoltVec3 *scale);

/// Continues a walk zjoltShapeGetTrianglesStart began.
///
/// `out_vertices` holds `3 * max_triangles` entries; `out_materials`, if
/// not NULL, holds `max_triangles` pointers borrowed from `shape`.
/// `*out_count` may be less than `max_triangles`, or 0 at the walk's end.
/// ZJOLT_RESULT_INVALID_ARGUMENT if below ZJOLT_SHAPE_MIN_TRIANGLES_REQUESTED.
ZJOLT_API ZJoltResult zjoltShapeGetTrianglesNext(
    const ZJoltShape *shape, ZJoltShapeTrianglesContext *context,
    uint32_t max_triangles, ZJoltVec3 *out_vertices,
    const ZJoltPhysicsMaterial **out_materials, uint32_t *out_count);

/// The materials `shape` was built with, in the order zjoltShapeGetMaterial
/// and the triangle read-back above index them. Only a mesh or a height field
/// has one; ZJOLT_RESULT_INVALID_ARGUMENT for any other shape kind, including
/// a compound of meshes — ask each leaf individually instead. Two-call
/// protocol; pointers borrowed from `shape`, valid as long as it is.
ZJOLT_API ZJoltResult zjoltShapeGetMaterialList(
    const ZJoltShape *shape, const ZJoltPhysicsMaterial **out_materials,
    uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Mesh specifics
//===----------------------------------------------------------------------===//

/// The index into zjoltShapeGetMaterialList that `sub_shape_id` names, for a
/// mesh built with `triangle_materials`. 0 for any other shape kind, which is
/// indistinguishable from a real index 0 — use zjoltShapeGetSubType first if
/// the difference matters.
ZJOLT_API uint32_t zjoltShapeMeshGetMaterialIndex(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id);

/// The per-triangle user data zjoltShapeCreateMesh's `triangle_user_data` set
/// for this triangle, or, if that was NULL, Jolt's own default: the
/// triangle's index in `indices` BEFORE Jolt's internal reordering. 0 for any
/// other shape kind.
ZJOLT_API uint32_t zjoltShapeMeshGetTriangleUserData(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id);

//===----------------------------------------------------------------------===//
// Height field specifics
//
// Addressed by (x, y) sample coordinates below, except
// zjoltShapeHeightFieldGetSubShapeCoordinates (a hit's sub-shape id -> (x, y)).
//===----------------------------------------------------------------------===//

/// Samples per side, after Jolt rounds the construction-time count up to a
/// multiple of the block size. 0 for any other shape kind.
ZJOLT_API uint32_t zjoltShapeHeightFieldGetSampleCount(const ZJoltShape *shape);

/// 0 for any other shape kind, which is never a real block size — Jolt's
/// smallest is 2.
ZJOLT_API uint32_t zjoltShapeHeightFieldGetBlockSize(const ZJoltShape *shape);

/// The range of height values this shape can encode, i.e. the domain
/// `SetHeights` would clamp into. Both 0 for any other shape kind.
ZJOLT_API float zjoltShapeHeightFieldGetMinHeightValue(const ZJoltShape *shape);
ZJOLT_API float zjoltShapeHeightFieldGetMaxHeightValue(const ZJoltShape *shape);

/// The local-space position of sample (x, y), including its height. Zeroed
/// for any other shape kind, or for (x, y) outside [0, sample count).
ZJOLT_API void zjoltShapeHeightFieldGetPosition(const ZJoltShape *shape,
                                                uint32_t x, uint32_t y,
                                                ZJoltVec3 *out_position);

/// Whether sample (x, y) is a hole — a sample originally given
/// ZJOLT_HEIGHT_FIELD_NO_COLLISION. True (there is nothing to collide with)
/// for any other shape kind or for (x, y) outside [0, sample count), which is
/// the safe reading of "nothing is known to be here".
ZJOLT_API bool zjoltShapeHeightFieldIsNoCollision(const ZJoltShape *shape,
                                                  uint32_t x, uint32_t y);

/// Drops `local_position` straight down (well, along -Y in the height
/// field's own local space) onto the surface. `*out_found` is false when
/// `local_position` is outside the field's footprint or over a hole, in
/// which case the other two outputs are left untouched.
ZJOLT_API ZJoltResult zjoltShapeHeightFieldProjectOntoSurface(
    const ZJoltShape *shape, const ZJoltVec3 *local_position,
    ZJoltVec3 *out_surface_position, ZJoltSubShapeId *out_sub_shape_id,
    bool *out_found);

/// The grid cell and which of its two triangles `sub_shape_id` names — the
/// inverse of the encoding zjoltShapeHeightFieldProjectOntoSurface and a hit's
/// sub-shape id both use. Zeroed for any other shape kind.
ZJOLT_API ZJoltResult zjoltShapeHeightFieldGetSubShapeCoordinates(
    const ZJoltShape *shape, ZJoltSubShapeId sub_shape_id, uint32_t *out_x,
    uint32_t *out_y, uint32_t *out_triangle_index);

/// Reads back a `size_x` by `size_y` block of height samples starting at
/// (x, y), row by row into `out_heights[row * stride + col]`. A hole reads
/// back as ZJOLT_HEIGHT_FIELD_NO_COLLISION. `out_heights` holds
/// `size_y * stride` entries; `stride` is in samples, at least `size_x`.
/// `x`/`y` must be a multiple of zjoltShapeHeightFieldGetBlockSize and the
/// block must fit the grid; ZJOLT_RESULT_INVALID_ARGUMENT otherwise.
ZJOLT_API ZJoltResult zjoltShapeHeightFieldGetHeights(
    const ZJoltShape *shape, uint32_t x, uint32_t y, uint32_t size_x,
    uint32_t size_y, float *out_heights, uint32_t stride);

/// Repaints existing height field samples in place, without rebuilding.
/// `x`/`y` must be a multiple of zjoltShapeHeightFieldGetBlockSize and fit
/// the grid; `heights` holds `size_y * stride` entries. Values outside
/// [GetMinHeightValue, GetMaxHeightValue] (fixed at create time) are
/// silently clamped. NOT thread safe against a query or step; notify every
/// body afterward.
ZJOLT_API ZJoltResult zjoltShapeHeightFieldSetHeights(
    ZJoltShape *shape, uint32_t x, uint32_t y, uint32_t size_x,
    uint32_t size_y, const float *heights, uint32_t stride,
    float active_edge_cos_threshold_angle);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SHAPE_H_
