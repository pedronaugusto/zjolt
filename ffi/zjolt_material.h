//===----------------------------------------------------------------------===//
// zjolt — physics materials. Part of the zjolt C ABI; see <zjolt.h>.
//
// A material is an identity, not a property bag: debug name and colour only.
// Friction and restitution live on the BODY (zjoltBodySetFriction), not here.
//
// Materials distinguish surfaces within one shape (one per mesh triangle or
// height-field quad); zjoltShapeGetMaterial maps a hit to its triangle. A
// shape holds a reference on each material it is built with.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_MATERIAL_H_
#define ZJOLT_MATERIAL_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Creates a material (ref count one). `debug_name` is copied (NULL =
/// empty); `debug_color` may be NULL (Jolt's grey). Neither affects the sim.
ZJOLT_API ZJoltResult zjoltPhysicsMaterialCreate(const char *debug_name,
                                                 const ZJoltColor *debug_color,
                                                 ZJoltPhysicsMaterial **out);

/// The material every shape built without one reports. Shared, owned by the
/// library, valid between zjoltInit and zjoltDeinit (NULL outside that). Real
/// material, not NULL — compare pointers, don't test for NULL.
ZJOLT_API const ZJoltPhysicsMaterial *zjoltPhysicsMaterialDefault(void);

ZJOLT_API void zjoltPhysicsMaterialAddRef(const ZJoltPhysicsMaterial *material);
ZJOLT_API void zjoltPhysicsMaterialRelease(const ZJoltPhysicsMaterial *material);
ZJOLT_API uint32_t
zjoltPhysicsMaterialGetRefCount(const ZJoltPhysicsMaterial *material);

/// Borrowed, never NULL, valid while the material is. Defaults to "Default"
/// or, for a nameless type, "Unknown".
ZJOLT_API const char *
zjoltPhysicsMaterialGetDebugName(const ZJoltPhysicsMaterial *material);

ZJOLT_API void zjoltPhysicsMaterialGetDebugColor(
    const ZJoltPhysicsMaterial *material, ZJoltColor *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_MATERIAL_H_
