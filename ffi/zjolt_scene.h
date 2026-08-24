//===----------------------------------------------------------------------===//
// zjolt — a whole world, described rather than simulated.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_SCENE_H_
#define ZJOLT_SCENE_H_

#include "zjolt_core.h"

// For ZJoltBodyDesc, ZJoltSoftBodyDesc and ZJoltConstraint: a scene is a list
// of exactly the descriptors the rest of this ABI creates bodies from.
#include "zjolt_body.h"
#include "zjolt_constraint.h"
#include "zjolt_softbody.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// The container a level is loaded from
//
// Everywhere else in this ABI a world is built call by call: create a shape,
// create a body, create a constraint, add each one. A scene is the same world
// held as DATA — bodies, soft bodies, and the joints between them — with
// nothing simulating and no physics system involved. Two things follow from
// that, and they are the whole reason this exists:
//
//   * it serialises. zjoltSceneSave writes one buffer that carries the
//     shapes as well as the placements, so a host can cook a level once and
//     load it without re-running the code that built it.
//   * it instantiates repeatedly. zjoltSceneCreateBodies stamps the same
//     scene into as many physics systems as you like — the second one costs
//     no shape construction, because the shapes are already built and are
//     shared by reference.
//
// A scene names its bodies by INDEX, not by body id: nothing in it has an id
// yet, because nothing in it is in a system. Constraints therefore connect
// index to index, and the ids only come into existence when
// zjoltSceneCreateBodies runs.
//
// Reference counted, like a shape: zjoltSceneRelease, not a destroy.
//===----------------------------------------------------------------------===//

typedef struct ZJoltScene ZJoltScene;

/// The body index that means "the world" — the same implicit, infinitely
/// heavy static body at the origin that ZJOLT_BODY_ID_WORLD names for a
/// constraint created against a live system.
///
/// Pass it as `body1` or `body2` to zjoltSceneAddConstraint to bolt the other
/// body to the world. Not both at once: Jolt asserts on a constraint between
/// two immovable things rather than refusing it, so this ABI refuses it here.
#define ZJOLT_SCENE_BODY_WORLD ((uint32_t)0xffffffffu)

/// An empty scene. Release it with zjoltSceneRelease.
ZJOLT_API ZJoltResult zjoltSceneCreate(ZJoltScene **out);

ZJOLT_API void zjoltSceneAddRef(const ZJoltScene *scene);
ZJOLT_API void zjoltSceneRelease(const ZJoltScene *scene);
ZJOLT_API uint32_t zjoltSceneGetRefCount(const ZJoltScene *scene);

//===----------------------------------------------------------------------===//
// Filling one in
//
// The descriptors are the ones zjoltBodyCreate and zjoltSoftBodyCreate take,
// field for field, and they are copied: nothing keeps a pointer to the caller's
// struct. What IS kept is a reference on the shape (and on a soft body's
// shared settings), for as long as the scene holds the entry — so the caller
// may release theirs as soon as the call returns.
//===----------------------------------------------------------------------===//

/// Appends a rigid body. `*out_index` receives its index, which is what a
/// constraint added afterwards refers to it by.
///
/// A refused call leaves ZJOLT_SCENE_BODY_WORLD there rather than 0, because 0
/// is a perfectly good index and a caller who ignores the result would
/// otherwise attach the next constraint to the first body in the scene.
ZJOLT_API ZJoltResult zjoltSceneAddBody(ZJoltScene *scene,
                                        const ZJoltBodyDesc *desc,
                                        uint32_t *out_index);

/// Appends a soft body. Soft bodies are numbered SEPARATELY from rigid ones —
/// `*out_index` counts soft bodies only — and a constraint cannot name one:
/// Jolt's scene attaches constraints to the rigid body list alone. A refused
/// call leaves ZJOLT_SCENE_BODY_WORLD in `*out_index`, as zjoltSceneAddBody
/// does.
ZJOLT_API ZJoltResult zjoltSceneAddSoftBody(ZJoltScene *scene,
                                            const ZJoltSoftBodyDesc *desc,
                                            uint32_t *out_index);

/// Appends the joint `constraint` describes, connecting the scene's bodies at
/// indices `body1` and `body2`.
///
/// The constraint is read, not kept: what the scene stores is the settings
/// object Jolt builds from it, so the caller may release `constraint` and
/// destroy the bodies it was created against immediately afterwards. Changing
/// the live constraint later does not change the scene's copy.
///
/// A live constraint is how a joint reaches a scene because this ABI has no
/// serialisable settings type of its own — see the note on descriptors in
/// zjolt_constraint.h. Create the joint against real bodies with any
/// zjoltConstraintCreate* call, add it to the scene, and release it; there is
/// no need to have added it to the system first.
///
/// Refused, rather than left to Jolt's asserts:
///   * a `constraint` that is not a two-body joint. A vehicle constraint is
///     the one this ABI can produce that is not, and it reports
///     ZJOLT_RESULT_UNSUPPORTED.
///   * `body1 == body2`, including both being ZJOLT_SCENE_BODY_WORLD.
///   * an index that names no body added to this scene yet. Add the bodies
///     first; a constraint cannot forward-reference one.
ZJOLT_API ZJoltResult zjoltSceneAddConstraint(ZJoltScene *scene,
                                              const ZJoltConstraint *constraint,
                                              uint32_t body1, uint32_t body2);

/// Appends everything in `system` — every body, and every two-body constraint
/// between them — to what `scene` already holds.
///
/// This is the other direction: a world built call by call, captured so it
/// can be saved. Jolt marks it as a debugging aid and it reads the system
/// without stepping it, so call it between steps.
///
/// A soft body captured this way lands in the soft-body list, but Jolt
/// numbers it as though it had landed in the rigid one, so a constraint
/// attached to a soft body comes back naming the wrong index. Nothing this
/// ABI can produce hits that — a constraint cannot name a soft body in the
/// first place — but a system whose constraints were added by other C++ can.
///
/// ZJOLT_RESULT_BODY_NOT_FOUND when a constraint in `system` names a body the
/// system no longer holds. Jolt asserts on that case and then reads a map
/// entry that is not there; the whole system is checked before anything is
/// appended, so a refusal leaves `scene` exactly as it was.
ZJOLT_API ZJoltResult zjoltSceneFromPhysicsSystem(
    ZJoltScene *scene, const ZJoltPhysicsSystem *system);

//===----------------------------------------------------------------------===//
// Reading one back
//
// The counts are separate per kind, and so are the indices: body 0, soft body
// 0 and constraint 0 are three different things.
//===----------------------------------------------------------------------===//

ZJOLT_API uint32_t zjoltSceneGetNumBodies(const ZJoltScene *scene);
ZJOLT_API uint32_t zjoltSceneGetNumSoftBodies(const ZJoltScene *scene);
ZJOLT_API uint32_t zjoltSceneGetNumConstraints(const ZJoltScene *scene);

/// The body at `index`, as a descriptor. ZJOLT_RESULT_INVALID_ARGUMENT for an
/// index at or past zjoltSceneGetNumBodies.
///
/// `out->shape` is BORROWED from the scene: it is not reference counted on the
/// way out, and it dies with the scene entry unless the caller takes a
/// reference of their own with zjoltShapeAddRef.
///
/// This is a view, and one field of it is lossy. Jolt has a third mass mode —
/// an inertia tensor supplied outright — that ZJoltOverrideMassProperties
/// cannot spell, and a scene loaded from a file may carry it. Such a body
/// reports CALCULATE_INERTIA with the mass it was given, which is the closest
/// this struct can come; the scene still instantiates the real tensor, so what
/// zjoltSceneCreateBodies builds is unaffected. Round-tripping a body through
/// this call and zjoltSceneAddBody is what loses it.
ZJOLT_API ZJoltResult zjoltSceneGetBody(const ZJoltScene *scene,
                                        uint32_t index, ZJoltBodyDesc *out);

/// The soft body at `index`. `out->shared_settings` is borrowed on the same
/// terms as zjoltSceneGetBody's shape.
ZJOLT_API ZJoltResult zjoltSceneGetSoftBody(const ZJoltScene *scene,
                                            uint32_t index,
                                            ZJoltSoftBodyDesc *out);

/// What a scene records about one joint.
///
/// Which KIND of joint it is is deliberately absent: a scene holds a settings
/// object, and asking a settings object what it would build means building it.
/// Instantiate the scene and ask the constraint with
/// zjoltConstraintGetSubType.
typedef struct ZJoltSceneConstraint {
  /// Index into the scene's bodies, or ZJOLT_SCENE_BODY_WORLD.
  uint32_t body1;
  uint32_t body2;
  uint64_t user_data;
  /// Higher is solved earlier. @see zjoltConstraintSetPriority.
  uint32_t priority;
  /// 0 means "use the system's setting".
  uint32_t num_velocity_steps_override;
  uint32_t num_position_steps_override;
  /// Whether the constraint starts out simulating.
  bool enabled;
} ZJoltSceneConstraint;

ZJOLT_API ZJoltResult zjoltSceneGetConstraint(const ZJoltScene *scene,
                                              uint32_t index,
                                              ZJoltSceneConstraint *out);

//===----------------------------------------------------------------------===//
// Turning one into a world
//===----------------------------------------------------------------------===//

/// Creates every body and soft body in `scene` in `system`, adds them all
/// (activated), and then creates and adds every constraint.
///
/// The scene is unchanged and may be instantiated again, into this system or
/// another one. The bodies share the scene's shapes rather than copying them.
///
/// ZJOLT_RESULT_OUT_OF_MEMORY when `system` runs out of body slots part of the
/// way through. That failure is NOT clean: the bodies created before it are in
/// the system and stay there, and no constraint is created at all — Jolt stops
/// before them, because indices into a half-created body list would connect
/// the wrong things. Recovering means destroying what was added, which is why
/// a system meant to hold a scene should be created with max_bodies well past
/// zjoltSceneGetNumBodies plus zjoltSceneGetNumSoftBodies.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT, before anything is created, when the scene
/// itself could not be instantiated: a body with no shape, a soft body with no
/// shared settings, a constraint with no settings, or a constraint naming a
/// body index that does not exist. None of those is reachable through
/// zjoltSceneAdd*; all of them are reachable through a hand-made buffer handed
/// to zjoltSceneRestore.
ZJOLT_API ZJoltResult zjoltSceneCreateBodies(const ZJoltScene *scene,
                                             ZJoltPhysicsSystem *system);

/// Rescales any shape in the scene that cannot legally be used at unit scale,
/// replacing it in place — so this CHANGES the scene, and the shape a
/// previously read-back descriptor borrowed may no longer be the scene's.
///
/// The case it exists for is a scaled shape baked into an asset: a scaled
/// convex hull, or any shape scaled by a negative or non-uniform factor its
/// kind forbids. Jolt refuses to create a body from one, and a level loaded
/// from a file is exactly where one shows up.
///
/// ZJOLT_RESULT_SHAPE_INVALID when some shape could not be fixed; the ones
/// that could be still were. ZJOLT_RESULT_INVALID_ARGUMENT when a body in the
/// scene has no shape at all, checked first because Jolt dereferences it
/// without looking.
ZJOLT_API ZJoltResult zjoltSceneFixInvalidScales(ZJoltScene *scene);

//===----------------------------------------------------------------------===//
// Serialisation
//
// The buffer follows the same container convention as zjoltShapeSave and
// zjoltPhysicsSystemSaveState — a magic tag of its own, a container version,
// this library's config id, the Jolt version, the payload length and a
// CRC-32, all validated before Jolt reads a byte — and carries the same
// caveat: it rejects the wrong file, a truncated file and a damaged file, and
// it is not a defence against a crafted payload that carries a matching
// checksum. Treat a cooked level as something your own tools wrote.
//
// Two fields Jolt does not write, worth knowing before a level leans on them:
// a rigid body's `user_data` and a constraint's `user_data`. Neither appears
// in Jolt's own SaveBinaryState (BodyCreationSettings.cpp:49, Constraint.cpp:26),
// so both come back as 0. A SOFT body's user_data IS written, which is what
// makes the asymmetry easy to miss. Anything a host needs after a load belongs
// in the level's own data, keyed by index, rather than in a body's user data.
//
// Jolt's own save takes two flags this ABI does not expose, because neither
// choice it offers is one this ABI could load back:
//
//   * omitting the SHAPES writes a payload whose bodies restore with no shape
//     at all. Jolt's restore accepts that quietly — it is meant for a host
//     that re-attaches shapes from its own asset system in C++ — and every
//     later call that touches such a body dereferences null. Shapes are
//     therefore always written.
//   * omitting the GROUP FILTERS is the only sound choice, so it is what this
//     always does. This ABI's group filter (zjolt_group.h) is not one of
//     Jolt's serialisable types, and writing a reference to one would record
//     an abstract base class that no restore can construct. A restored body
//     keeps its group and sub-group ids and comes back with no filter;
//     re-attach one with zjoltBodySetCollisionGroup after instantiating.
//===----------------------------------------------------------------------===//

/// Writes `scene` into `buffer`.
///
/// Two-call protocol: `buffer` NULL reports the size in `*out_size` and writes
/// nothing, and a `capacity` short of that reports
/// ZJOLT_RESULT_BUFFER_TOO_SMALL with the required size still written.
///
/// ZJOLT_RESULT_INVALID_ARGUMENT when the scene holds something that cannot be
/// written: a body with no shape, or a constraint with no settings. Neither is
/// reachable through zjoltSceneAdd*.
ZJOLT_API ZJoltResult zjoltSceneSave(const ZJoltScene *scene, void *buffer,
                                     size_t capacity, size_t *out_size);

/// Reads a scene back out of a buffer zjoltSceneSave wrote. Release the
/// result with zjoltSceneRelease.
///
/// ZJOLT_RESULT_BAD_FORMAT covers five kinds of wrong input: not a zjolt scene
/// buffer at all, written by a different zjolt or a different precision
/// setting, written against a different Jolt, truncated or carrying trailing
/// bytes, and damaged in storage. A payload Jolt itself rejects — an unknown
/// shape kind, a constraint type this build did not register — is reported the
/// same way, with Jolt's own message in zjoltLastError.
///
/// The shapes come back newly built and owned by the scene, NOT shared with
/// whatever the save was made from: two scenes restored from one buffer hold
/// two sets of shapes.
ZJOLT_API ZJoltResult zjoltSceneRestore(const void *data, size_t size,
                                        ZJoltScene **out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_SCENE_H_
