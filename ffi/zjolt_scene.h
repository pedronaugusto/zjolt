//===----------------------------------------------------------------------===//
// zjolt — a whole world, described rather than simulated.
//
// A ZJoltScene holds bodies, soft bodies and joints as DATA. It serialises
// and instantiates repeatedly into any number of systems, sharing shapes
// by reference. Bodies are named by INDEX, not body id; ids exist only
// after zjoltSceneCreateBodies runs. Reference counted: zjoltSceneRelease.
//
// Save: shapes are always written; group filters never (re-attach with
// zjoltBodySetCollisionGroup after restore). A rigid body's or
// constraint's user_data is not saved and comes back 0; a soft body's is.
// A `stream` write returns ZJOLT_RESULT_IO_ERROR on failure.

#ifndef ZJOLT_SCENE_H_
#define ZJOLT_SCENE_H_

#include "zjolt_core.h"

#include "zjolt_body.h"
#include "zjolt_constraint.h"
#include "zjolt_softbody.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ZJoltScene ZJoltScene;

/// Body index meaning "the world", for `body1`/`body2` in
/// zjoltSceneAddConstraint. Passing it for both is refused.
#define ZJOLT_SCENE_BODY_WORLD ((uint32_t)0xffffffffu)

/// Empty scene. Release with zjoltSceneRelease.
ZJOLT_API ZJoltResult zjoltSceneCreate(ZJoltScene **out);

ZJOLT_API void zjoltSceneAddRef(const ZJoltScene *scene);
ZJOLT_API void zjoltSceneRelease(const ZJoltScene *scene);
ZJOLT_API uint32_t zjoltSceneGetRefCount(const ZJoltScene *scene);

/// Appends a rigid body; `desc` is copied field for field, taking a
/// reference on its shape. `*out_index` names it for a later constraint.
/// A refusal leaves ZJOLT_SCENE_BODY_WORLD in `*out_index`, not 0.
ZJOLT_API ZJoltResult zjoltSceneAddBody(ZJoltScene *scene,
                                        const ZJoltBodyDesc *desc,
                                        uint32_t *out_index);

/// As zjoltSceneAddBody, for a soft body. Numbered separately from rigid
/// bodies (`*out_index` counts soft bodies only); cannot be named by a
/// constraint.
ZJOLT_API ZJoltResult zjoltSceneAddSoftBody(ZJoltScene *scene,
                                            const ZJoltSoftBodyDesc *desc,
                                            uint32_t *out_index);

/// Appends the joint `constraint` describes, between bodies `body1` and
/// `body2`. `constraint` is read, not kept; may be released immediately
/// after. Refused: a non-two-body `constraint` (ZJOLT_RESULT_UNSUPPORTED),
/// `body1 == body2`, or an index naming no body yet added.
ZJOLT_API ZJoltResult zjoltSceneAddConstraint(ZJoltScene *scene,
                                              const ZJoltConstraint *constraint,
                                              uint32_t body1, uint32_t body2);

/// Appends every body and two-body constraint in `system` to `scene`.
/// Call between steps. A soft body is numbered as though rigid, so a
/// constraint naming one (reachable only via other C++) comes back with
/// the wrong index. ZJOLT_RESULT_BODY_NOT_FOUND if a constraint names a
/// body the system no longer holds; `scene` is unchanged on refusal.
ZJOLT_API ZJoltResult zjoltSceneFromPhysicsSystem(
    ZJoltScene *scene, const ZJoltPhysicsSystem *system);

ZJOLT_API uint32_t zjoltSceneGetNumBodies(const ZJoltScene *scene);
ZJOLT_API uint32_t zjoltSceneGetNumSoftBodies(const ZJoltScene *scene);
ZJOLT_API uint32_t zjoltSceneGetNumConstraints(const ZJoltScene *scene);

/// The body at `index`. ZJOLT_RESULT_INVALID_ARGUMENT past
/// zjoltSceneGetNumBodies. `out->shape` is BORROWED: not ref-counted, and
/// dies with the scene entry unless you call zjoltShapeAddRef. A body
/// with Jolt's inertia-tensor mass mode reports CALCULATE_INERTIA
/// instead; round-tripping through this and zjoltSceneAddBody loses that
/// distinction.
ZJOLT_API ZJoltResult zjoltSceneGetBody(const ZJoltScene *scene,
                                        uint32_t index, ZJoltBodyDesc *out);

/// As zjoltSceneGetBody, for a soft body. `out->shared_settings` is
/// borrowed on the same terms.
ZJOLT_API ZJoltResult zjoltSceneGetSoftBody(const ZJoltScene *scene,
                                            uint32_t index,
                                            ZJoltSoftBodyDesc *out);

/// What a scene records about one joint. Which KIND of joint is not
/// stored; instantiate the scene and ask with zjoltConstraintGetSubType.
typedef struct ZJoltSceneConstraint {
  /// Index into the scene's bodies, or ZJOLT_SCENE_BODY_WORLD.
  uint32_t body1;
  uint32_t body2;
  uint64_t user_data;
  /// Higher is solved earlier.
  uint32_t priority;
  /// 0 means "use the system's setting".
  uint32_t num_velocity_steps_override;
  uint32_t num_position_steps_override;
  bool enabled;
} ZJoltSceneConstraint;

ZJOLT_API ZJoltResult zjoltSceneGetConstraint(const ZJoltScene *scene,
                                              uint32_t index,
                                              ZJoltSceneConstraint *out);

/// Creates every body/soft body in `scene` (activated), then every
/// constraint. ZJOLT_RESULT_OUT_OF_MEMORY leaves created bodies without
/// constraints — size `system` past zjoltSceneGetNumBodies +
/// zjoltSceneGetNumSoftBodies. ZJOLT_RESULT_INVALID_ARGUMENT if the scene
/// cannot be instantiated (unreachable via zjoltSceneAdd*, only through
/// zjoltSceneRestore).
ZJOLT_API ZJoltResult zjoltSceneCreateBodies(const ZJoltScene *scene,
                                             ZJoltPhysicsSystem *system);

/// Rescales any shape that cannot legally be used at unit scale, in
/// place. Invalidates any shape borrowed via zjoltSceneGetBody or
/// zjoltSceneGetSoftBody. ZJOLT_RESULT_SHAPE_INVALID if a shape could not
/// be fixed. ZJOLT_RESULT_INVALID_ARGUMENT if a body has no shape.
ZJOLT_API ZJoltResult zjoltSceneFixInvalidScales(ZJoltScene *scene);

/// Writes `scene` into `buffer`. `buffer` NULL sizes into `*out_size`;
/// short `capacity` returns ZJOLT_RESULT_BUFFER_TOO_SMALL.
/// ZJOLT_RESULT_INVALID_ARGUMENT if the scene holds something unwritable.
ZJOLT_API ZJoltResult zjoltSceneSave(const ZJoltScene *scene, void *buffer,
                                     size_t capacity, size_t *out_size);

/// Reads a scene from a buffer zjoltSceneSave wrote. Release with
/// zjoltSceneRelease. ZJOLT_RESULT_BAD_FORMAT for a malformed buffer, or
/// one Jolt itself rejects; zjoltLastError carries Jolt's own message.
/// Shapes come back newly built, not shared with the original.
ZJOLT_API ZJoltResult zjoltSceneRestore(const void *data, size_t size,
                                        ZJoltScene **out);

/// As zjoltSceneSave, through `stream` instead of a resident buffer.
ZJOLT_API ZJoltResult zjoltSceneSaveStream(const ZJoltScene *scene,
                                           const ZJoltStream *stream);

/// As zjoltSceneRestore, reading through `stream`.
ZJOLT_API ZJoltResult zjoltSceneRestoreStream(const ZJoltStream *stream,
                                              ZJoltScene **out);

/// Writes `scene` through `stream` in Jolt's own object-stream format
/// (EStreamType::Text or ::Binary), for interchange with a C++ host of
/// vanilla Jolt. A RIGID body's SHAPE does not survive; a SOFT body's
/// does. ZJOLT_RESULT_UNSUPPORTED unless built with -Dobject_stream=true.
ZJOLT_API ZJoltResult zjoltSceneSaveObjectStream(const ZJoltScene *scene,
                                                 ZJoltObjectStreamFormat format,
                                                 const ZJoltStream *stream);

/// Reads a scene written by zjoltSceneSaveObjectStream or a vanilla-Jolt
/// C++ host. Release with zjoltSceneRelease. ZJOLT_RESULT_BAD_FORMAT if
/// Jolt's reader refuses the stream. A rigid body in the result has no
/// shape.
ZJOLT_API ZJoltResult zjoltSceneRestoreObjectStream(const ZJoltStream *stream,
                                                    ZJoltScene **out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SCENE_H_
