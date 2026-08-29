//===----------------------------------------------------------------------===//
// zjolt — host-defined shapes. Part of the zjolt C ABI (include <zjolt.h>).
//
// Two layers: a custom CONVEX shape needs only a support function (GJK/EPA
// do the rest, the common case); a shape that is not convex binds the full
// JPH::Shape interface.
//
// Every crossing here is once per query or batched, never once per
// primitive inside a broadphase walk. A callback Jolt declares pure virtual
// left NULL is ZJOLT_RESULT_INVALID_ARGUMENT at the create call; one with a
// Jolt default may be NULL and falls back to it.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_CUSTOMSHAPE_H_
#define ZJOLT_CUSTOMSHAPE_H_

#include "zjolt_core.h"
#include "zjolt_query.h"
#include "zjolt_shape.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Layer A — custom convex shape
//===----------------------------------------------------------------------===//

typedef struct ZJoltConvexShapeCallbacks {
  /// Farthest point of the shape along `direction`, in local space.
  ZJoltVec3 (*support)(void *user, ZJoltVec3 direction);
  /// The face in the given direction, for manifold generation. Write at most
  /// `max_vertices` and return how many. May be NULL: manifolds then fall
  /// back to a single support point, the same as a shape with no faces (a
  /// sphere).
  uint32_t (*supporting_face)(void *user, ZJoltVec3 direction, ZJoltVec3 scale,
                              ZJoltVec3 *out_vertices, uint32_t max_vertices);
  float (*inner_radius)(const void *user);
  void (*local_bounds)(const void *user, ZJoltAABox *out);
  void (*mass_properties)(const void *user, ZJoltMassProperties *out);
  float (*volume)(const void *user);
  void (*destroy)(void *user);  /// may be NULL
} ZJoltConvexShapeCallbacks;

/// Required: `support`, `inner_radius`, `local_bounds`, `mass_properties`,
/// `volume`. `material` NULL means Jolt's shared default.
///
/// Unsupported by zjoltShapeSave/zjoltShapeRestore: a save succeeds but
/// restoring yields an inert placeholder (zero volume, zero bounds, no
/// support extent), not the original.
ZJOLT_API ZJoltResult zjoltShapeCreateCustomConvex(
    const ZJoltConvexShapeCallbacks *callbacks, void *user,
    const ZJoltPhysicsMaterial *material, ZJoltShape **out);

//===----------------------------------------------------------------------===//
// Layer B — general custom shape: the full JPH::Shape interface, for a
// non-convex shape. Multi-result methods are batched into a shim-owned
// array with a count. `sub_shape_id` values a host hands back are LOCAL
// (raw integers below 2^sub_shape_id_bits_recursive()); the shim composes them.
//===----------------------------------------------------------------------===//

/// Results a batched query can report in one call without a second round
/// trip. Generous for real use; a shape wanting more should narrow its own
/// query (a smaller box, a shorter ray) rather than page through this ABI.
#define ZJOLT_CUSTOM_SHAPE_MAX_BATCH 32

typedef struct ZJoltCustomShapeRayHit {
  float fraction;
  ZJoltSubShapeId sub_shape_id;
} ZJoltCustomShapeRayHit;

/// One child Jolt should treat as its own placed shape — what
/// CollectTransformedShapes and TransformShape report. `position`/`rotation`/
/// `scale` are already fully resolved (composed with whatever placement the
/// callback itself was given), ready to hand straight to a TransformedShape
/// — not relative to this shape, so the host does the one composition rather
/// than the shim guessing at it.
typedef struct ZJoltCustomShapeChild {
  const ZJoltShape *shape;
  ZJoltVec3 position;
  ZJoltQuat rotation;
  ZJoltVec3 scale;
  ZJoltSubShapeId sub_shape_id;
} ZJoltCustomShapeChild;

typedef struct ZJoltShapeCallbacks {
  //===--------------------------------------------------------------------===//
  // Required — Jolt declares each of these pure virtual.
  //===--------------------------------------------------------------------===//

  void (*local_bounds)(const void *user, ZJoltAABox *out);
  /// Sub-shape id bits this shape and everything under it needs. 0 for a
  /// leafless shape (every hit then carries ZJOLT_SUB_SHAPE_ID_EMPTY).
  uint32_t (*sub_shape_id_bits_recursive)(const void *user);
  float (*inner_radius)(const void *user);
  void (*mass_properties)(const void *user, ZJoltMassProperties *out);
  /// Never NULL for a valid `sub_shape_id`; NULL return installs Jolt's
  /// shared default material.
  const ZJoltPhysicsMaterial *(*get_material)(const void *user,
                                              ZJoltSubShapeId sub_shape_id);
  void (*surface_normal)(const void *user, ZJoltSubShapeId sub_shape_id,
                         ZJoltVec3 local_surface_position,
                         ZJoltVec3 *out_normal);
  void (*submerged_volume)(const void *user,
                           const ZJoltMat44 *center_of_mass_transform,
                           ZJoltVec3 scale, const ZJoltPlane *surface,
                           float *out_total_volume,
                           float *out_submerged_volume,
                           ZJoltVec3 *out_center_of_buoyancy);
  /// The nearest hit, Shape::CastRay's single-hit overload. False for no
  /// hit; a hit past `max_fraction` (the caller's current best) is refused
  /// by the shim regardless of what this returns.
  bool (*cast_ray_closest)(const void *user, ZJoltVec3 origin,
                           ZJoltVec3 direction, float max_fraction,
                           float *out_fraction,
                           ZJoltSubShapeId *out_sub_shape_id);
  /// Every hit, up to ZJOLT_CUSTOM_SHAPE_MAX_BATCH, Shape::CastRay's
  /// collector overload.
  uint32_t (*cast_ray_all)(const void *user, ZJoltVec3 origin,
                           ZJoltVec3 direction,
                           const ZJoltRayCastSettings *settings,
                           ZJoltCustomShapeRayHit *out_hits,
                           uint32_t max_hits);
  /// Sub-shape ids of every leaf containing `point`, up to
  /// ZJOLT_CUSTOM_SHAPE_MAX_BATCH.
  uint32_t (*collide_point)(const void *user, ZJoltVec3 point,
                            ZJoltSubShapeId *out_sub_shape_ids,
                            uint32_t max_hits);
  /// One call for `count` soft-body vertices, in this shape's own local
  /// space (unscaled, the same frame `local_bounds`/`mass_properties` use).
  /// `out_penetration[i]`: positive is deeper, negative is far away.
  /// `out_normal[i]` is in local space; the shim rotates it to world space.
  /// `inv_mass[i]` is zero for a pinned vertex; the shim discards results
  /// for those either way, so it is there to let a shape skip the work.
  void (*collide_soft_body_vertices)(const void *user, ZJoltVec3 scale,
                                     const ZJoltVec3 *local_positions,
                                     const float *inv_mass, uint32_t count,
                                     float *out_penetration,
                                     ZJoltVec3 *out_normal);
  /// Shape::GetTrianglesStart/Next, kept exactly as batched as Jolt's own —
  /// see ffi/zjolt_shape.h for the two-call protocol these implement.
  ZJoltResult (*get_triangles_start)(const void *user,
                                     ZJoltShapeTrianglesContext *context,
                                     const ZJoltAABox *box,
                                     const ZJoltVec3 *position,
                                     const ZJoltQuat *rotation,
                                     const ZJoltVec3 *scale);
  ZJoltResult (*get_triangles_next)(const void *user,
                                    ZJoltShapeTrianglesContext *context,
                                    uint32_t max_triangles,
                                    ZJoltVec3 *out_vertices,
                                    const ZJoltPhysicsMaterial **out_materials,
                                    uint32_t *out_count);
  void (*get_stats)(const void *user, ZJoltShapeStats *out);
  float (*volume)(const void *user);

  //===--------------------------------------------------------------------===//
  // Optional — NULL uses Jolt's own default for the virtual it stands in for.
  //===--------------------------------------------------------------------===//

  /// NULL means false: the shape may be used by a dynamic or kinematic body.
  bool (*must_be_static)(const void *user);
  /// NULL means the origin.
  ZJoltVec3 (*center_of_mass)(const void *user);
  /// NULL falls back to the same default: scale and transform
  /// `local_bounds`'s own answer, which already goes through this shape's
  /// own callback either way. Only worth providing for a tighter box than
  /// that generic computation gives.
  void (*world_space_bounds)(const void *user,
                             const ZJoltMat44 *center_of_mass_transform,
                             ZJoltVec3 scale, ZJoltAABox *out);
  /// NULL means no face: GetSupportingFace's default for a shape with none.
  uint32_t (*supporting_face)(const void *user, ZJoltSubShapeId sub_shape_id,
                              ZJoltVec3 direction, ZJoltVec3 scale,
                              ZJoltVec3 *out_vertices, uint32_t max_vertices);
  /// NULL means this shape's own user data, Jolt's own default.
  uint64_t (*sub_shape_user_data)(const void *user,
                                  ZJoltSubShapeId sub_shape_id);
  /// `box`/`position`/`rotation`/`scale` are this shape's own query box and
  /// placement, exactly what Shape::CollectTransformedShapes was given —
  /// handed through rather than pre-composed so the host does the one
  /// composition onto each child's own placement. NULL means this shape
  /// reports itself as its one child, Jolt's own default for a leaf with no
  /// substructure.
  uint32_t (*collect_transformed_shapes)(const void *user,
                                         const ZJoltAABox *box,
                                         const ZJoltVec3 *position,
                                         const ZJoltQuat *rotation,
                                         ZJoltVec3 scale,
                                         ZJoltCustomShapeChild *out_children,
                                         uint32_t max_children);
  /// NULL falls back to the same default: decompose the transform and
  /// report this shape once.
  uint32_t (*transform_shape)(const void *user,
                              const ZJoltMat44 *center_of_mass_transform,
                              ZJoltCustomShapeChild *out_children,
                              uint32_t max_children);
  /// NULL means any non-zero scale is accepted, Jolt's own base default.
  bool (*is_valid_scale)(const void *user, ZJoltVec3 scale);
  /// NULL means only zero is rejected, unchanged otherwise.
  void (*make_scale_valid)(const void *user, ZJoltVec3 scale,
                           ZJoltVec3 *out_scale);
  /// Host bytes, plus the materials and child shapes this shape references
  /// — SaveBinaryState/SaveMaterialState/SaveSubShapeState. No restore
  /// counterpart: a shape restored via zjoltShapeRestore is always the
  /// inert placeholder (see zjoltShapeCreateCustom), so nothing here is
  /// read back. Saving is still real: a host reading its own stream sees
  /// exactly what these wrote.
  void (*save_binary_state)(const void *user, const ZJoltStream *stream);
  /// Up to ZJOLT_CUSTOM_SHAPE_MAX_BATCH.
  uint32_t (*save_material_state)(const void *user,
                                  const ZJoltPhysicsMaterial **out_materials,
                                  uint32_t max_materials);
  /// Up to ZJOLT_CUSTOM_SHAPE_MAX_BATCH.
  uint32_t (*save_sub_shape_state)(const void *user,
                                   const ZJoltShape **out_shapes,
                                   uint32_t max_shapes);
  void (*destroy)(void *user);  /// may be NULL
} ZJoltShapeCallbacks;

/// As zjoltShapeCreateCustomConvex, for a shape that is not convex: every
/// collision entry point above is required, not derived.
///
/// Debug drawing is not a callback: the shim draws via the already-bound
/// GetTrianglesStart/Next. zjoltShapeCreateCustomConvex's save/restore
/// limitation applies here too.
ZJOLT_API ZJoltResult zjoltShapeCreateCustom(const ZJoltShapeCallbacks *callbacks,
                                             void *user,
                                             ZJoltShape **out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_CUSTOMSHAPE_H_
