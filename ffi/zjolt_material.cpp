//===----------------------------------------------------------------------===//
// zjolt — physics materials.
//
// The concrete type is PhysicsMaterialSimple, the only subclass Jolt ships and
// the only one that stores a name and a colour of its own; the base class's
// GetDebugName is a stub answering "Unknown". What a material is, and what it
// deliberately is not, is written at the top of zjolt_material.h.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>

extern "C" {

ZJoltResult zjoltPhysicsMaterialCreate(const char *debug_name,
                                       const ZJoltColor *debug_color,
                                       ZJoltPhysicsMaterial **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::PhysicsMaterialSimple *material = zjolt::New<JPH::PhysicsMaterialSimple>(
      JPH::string_view(debug_name != nullptr ? debug_name : ""),
      debug_color != nullptr ? zjolt::ToJolt(*debug_color) : JPH::Color::sGrey);
  if (material == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate a physics material");
  }

  // Owned through the base pointer, not the concrete one. Jolt's RefTarget is
  // a CRTP base parameterised on PhysicsMaterial, so a PhysicsMaterialSimple
  // is a RefTarget<PhysicsMaterial> and not a RefTarget<PhysicsMaterialSimple>
  // — and Own's own guard says so rather than counting the wrong object.
  JPH::PhysicsMaterial *base = material;
  *out = const_cast<ZJoltPhysicsMaterial *>(zjolt::ToC(zjolt::Own(base)));
  return ZJOLT_RESULT_OK;
}

const ZJoltPhysicsMaterial *zjoltPhysicsMaterialDefault(void) {
  // RegisterTypes creates it and UnregisterTypes drops it, so its lifetime is
  // exactly the library's. Reported as null outside that window rather than
  // as a dangling pointer.
  if (!zjolt::IsInitialized()) return nullptr;
  return zjolt::ToC(JPH::PhysicsMaterial::sDefault.GetPtr());
}

void zjoltPhysicsMaterialAddRef(const ZJoltPhysicsMaterial *material) {
  if (material == nullptr) return;
  zjolt::HostRetain(zjolt::ToJolt(material));
}

void zjoltPhysicsMaterialRelease(const ZJoltPhysicsMaterial *material) {
  if (material == nullptr) return;
  zjolt::HostRelease(zjolt::ToJolt(material));
}

uint32_t zjoltPhysicsMaterialGetRefCount(const ZJoltPhysicsMaterial *material) {
  if (material == nullptr) return 0;
  return static_cast<uint32_t>(zjolt::ToJolt(material)->GetRefCount());
}

const char *zjoltPhysicsMaterialGetDebugName(
    const ZJoltPhysicsMaterial *material) {
  if (material == nullptr) return "";
  return zjolt::ToJolt(material)->GetDebugName();
}

void zjoltPhysicsMaterialGetDebugColor(const ZJoltPhysicsMaterial *material,
                                       ZJoltColor *out) {
  if (out == nullptr) return;
  if (material == nullptr) {
    *out = ZJoltColor{0, 0, 0, 0};
    return;
  }
  *out = zjolt::ToC(zjolt::ToJolt(material)->GetDebugColor());
}

}  // extern "C"
