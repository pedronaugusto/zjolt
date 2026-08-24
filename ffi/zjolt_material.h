//===----------------------------------------------------------------------===//
// zjolt — physics materials.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// A material is an IDENTITY, not a property bag. It carries a debug name and a
// debug colour and NOTHING ELSE — no friction, no restitution, no user data,
// no sound or particle fields. That is worth stating first and plainly,
// because the name invites the opposite assumption. Friction and restitution
// live on the BODY (zjoltBodySetFriction, zjoltBodySetRestitution), and a
// contact combines the two bodies' values, never the two surfaces'.
//
// What a material is for is telling one surface apart from another WITHIN one
// shape. A mesh is built with a material per triangle and a height field with
// one per quad; a hit carries a sub-shape id, and zjoltShapeGetMaterial turns
// that id into the material of the exact triangle that was hit. That is how a
// single terrain mesh is gravel here and metal there. A host maps the returned
// pointer to its own surface data by IDENTITY — usually by comparing it
// against the materials it created — and keeps the properties on its side.
//
// A shape built with a material holds a reference on it, so the usual pattern
// is create, build the shapes, release. Two shapes may share one.
//
// Subclasses carrying real properties are a C++ affair, and not one this ABI
// can host: Jolt reaches a material's type through its own RTTI macros
// (JPH_DECLARE_SERIALIZABLE_VIRTUAL and friends) plus a Factory::Register
// call. That is Jolt's hand-rolled type system, not the language's — zjolt
// compiles -fno-rtti — so a custom material has to be built and registered in
// C++ against Jolt directly, and cannot be reached through this header.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_MATERIAL_H_
#define ZJOLT_MATERIAL_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Creates a material with a debug name and a debug colour, returning it with
/// a reference count of one.
///
/// `debug_name` is copied; NULL is the empty string. `debug_color` may be NULL
/// for Jolt's grey. Neither field affects the simulation in any way — they
/// exist so a debug renderer and a log line can say which surface was hit.
ZJOLT_API ZJoltResult zjoltPhysicsMaterialCreate(const char *debug_name,
                                                 const ZJoltColor *debug_color,
                                                 ZJoltPhysicsMaterial **out);

/// The material every shape built without one reports.
///
/// Shared, owned by the library, and valid between zjoltInit and zjoltDeinit —
/// Jolt creates it while registering its types and drops it while
/// unregistering them. It is a real material, not NULL, which is why "did this
/// triangle have a material of its own?" is answered by comparing against this
/// pointer rather than by testing for NULL.
///
/// NULL only when the library is not initialised.
ZJOLT_API const ZJoltPhysicsMaterial *zjoltPhysicsMaterialDefault(void);

ZJOLT_API void zjoltPhysicsMaterialAddRef(const ZJoltPhysicsMaterial *material);
ZJOLT_API void zjoltPhysicsMaterialRelease(const ZJoltPhysicsMaterial *material);
ZJOLT_API uint32_t
zjoltPhysicsMaterialGetRefCount(const ZJoltPhysicsMaterial *material);

/// Borrowed, never NULL, and valid while the material is. Jolt's own default
/// answers "Default". A material whose concrete type carries no name of its
/// own answers "Unknown", which is the base class's reply rather than
/// something a caller set.
ZJOLT_API const char *
zjoltPhysicsMaterialGetDebugName(const ZJoltPhysicsMaterial *material);

ZJOLT_API void zjoltPhysicsMaterialGetDebugColor(
    const ZJoltPhysicsMaterial *material, ZJoltColor *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_MATERIAL_H_
