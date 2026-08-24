//===----------------------------------------------------------------------===//
// zjolt — wheeled and tracked vehicles.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

#include <algorithm>
#include <cmath>

//===----------------------------------------------------------------------===//
// Viewed as a plain constraint
//
// ZJoltConstraint's own ToC/ToJolt live in zjolt_constraint.cpp, the only
// other translation unit that needs them; this local, identical inline
// definition is the same call zjolt_softbody.cpp makes for
// ZJoltSoftBodySharedSettings, and for the same reason (see its own
// comment): the tag is never completed or dereferenced as itself, so
// duplicating one reinterpret_cast rather than promoting it to a shared
// header is the smaller change.
//===----------------------------------------------------------------------===//

namespace zjolt {
inline ZJoltConstraint *ToC(JPH::Constraint *constraint) {
  return reinterpret_cast<ZJoltConstraint *>(constraint);
}
}  // namespace zjolt

//===----------------------------------------------------------------------===//
// Wheel-ground collision filters
//
// zjolt_internal.h already has adapters of exactly this shape for queries,
// but they are `final` and hold their ZJolt* table by value at construction.
// A vehicle's filters are retargetable after the fact and live inside the
// handle, so these are the same three forwards written to be mutated in
// place instead.
//
// The body one is not quite a forward: it also re-applies the self-exclusion
// that installing any body filter would otherwise take away from Jolt (see
// VehicleCollisionTester.cpp, where a NULL mBodyFilter means
// IgnoreSingleBodyFilter(vehicle body)). Wheels colliding with their own
// chassis is not a thing any caller wants, so it is not a thing this ABI
// lets one ask for by omission.
//===----------------------------------------------------------------------===//

namespace {

class VehicleBroadPhaseLayerFilter final : public JPH::BroadPhaseLayerFilter {
 public:
  ZJoltBroadPhaseLayerFilter table{};

  bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override {
    if (table.should_collide == nullptr) return true;
    return table.should_collide(
        table.user, static_cast<ZJoltBroadPhaseLayer>(inLayer.GetValue()));
  }
};

class VehicleObjectLayerFilter final : public JPH::ObjectLayerFilter {
 public:
  ZJoltObjectLayerFilter table{};

  bool ShouldCollide(JPH::ObjectLayer inLayer) const override {
    if (table.should_collide == nullptr) return true;
    return table.should_collide(table.user,
                                static_cast<ZJoltObjectLayer>(inLayer));
  }
};

class VehicleBodyFilter final : public JPH::BodyFilter {
 public:
  ZJoltBodyFilter table{};
  JPH::BodyID vehicle_body;

  bool ShouldCollide(const JPH::BodyID &inBodyID) const override {
    if (inBodyID == vehicle_body) return false;
    if (table.should_collide == nullptr) return true;
    return table.should_collide(table.user, zjolt::ToC(inBodyID));
  }
};

}  // namespace

//===----------------------------------------------------------------------===//
// The handle
//
// Defined here rather than in zjolt_internal.h — the handle types living
// there are the ones shared across translation units, and nothing outside
// this file ever sees a ZJoltVehicleConstraint*. It must still match the
// incomplete tag zjolt_vehicle.h declares, exactly as that header's own
// comment on ZJoltCharacter describes.
//===----------------------------------------------------------------------===//

struct ZJoltVehicleConstraint {
  JPH::Ref<JPH::VehicleConstraint> impl;
  ZJoltPhysicsSystem *owner;

  /// Which VehicleController subclass `impl->GetController()` actually is.
  /// VehicleController carries no runtime type tag of its own (Jolt builds
  /// without RTTI), so this is what makes the downcast in WheeledController /
  /// TrackedController / MotorcycleControllerOf below sound rather than a
  /// guess.
  ZJoltVehicleControllerKind controller_kind;
  ZJoltVehicleCollisionTesterKind collision_tester_kind;

  /// The tester `impl` is using. `impl` holds it as a RefConst, so this is
  /// the only mutable route to it — which both the filters and
  /// zjoltVehicleConstraintSetCollisionTester need.
  JPH::Ref<JPH::VehicleCollisionTester> tester;

  /// Live for as long as the handle, because the tester holds bare pointers
  /// to them; `filters` is what the caller last handed over, kept verbatim so
  /// zjoltVehicleConstraintGetWheelFilters can give it back unchanged.
  ZJoltVehicleWheelFilters filters{};
  VehicleBroadPhaseLayerFilter broad_phase_filter;
  VehicleObjectLayerFilter object_layer_filter;
  VehicleBodyFilter body_filter;

  /// Every callback below is read from inside the std::function installed on
  /// Jolt's side, which captures only this handle — so clearing one here is
  /// enough to stop it being called even if Jolt still holds the shim.
  ZJoltVehicleStepCallback pre_step{};
  ZJoltVehicleStepCallback post_collide{};
  ZJoltVehicleStepCallback post_step{};
  ZJoltVehicleCombineFrictionCallback combine_friction{};
  ZJoltVehicleTireMaxImpulseCallback tire_max_impulse{};
};

namespace {

//===----------------------------------------------------------------------===//
// Unwrapping
//===----------------------------------------------------------------------===//

JPH::VehicleConstraint *Impl(ZJoltVehicleConstraint *c) {
  return c != nullptr ? c->impl.GetPtr() : nullptr;
}
const JPH::VehicleConstraint *Impl(const ZJoltVehicleConstraint *c) {
  return c != nullptr ? c->impl.GetPtr() : nullptr;
}

/// The wheel at `index`, or NULL for a NULL constraint or an out-of-range
/// index. `VehicleConstraint::GetWheel` has no bounds check of its own —
/// recon flagged `JPH_ASSERT(inWheelIndex < mWheels.size())` in
/// VehicleConstraint.cpp — so every wheel accessor below goes through this
/// rather than GetWheel directly.
JPH::Wheel *WheelAt(ZJoltVehicleConstraint *c, uint32_t index) {
  JPH::VehicleConstraint *impl = Impl(c);
  if (impl == nullptr || index >= impl->GetWheels().size()) return nullptr;
  return impl->GetWheel(index);
}
const JPH::Wheel *WheelAt(const ZJoltVehicleConstraint *c, uint32_t index) {
  const JPH::VehicleConstraint *impl = Impl(c);
  if (impl == nullptr || index >= impl->GetWheels().size()) return nullptr;
  return impl->GetWheel(index);
}

/// NULL unless `c` actually holds a wheeled-family controller. A motorcycle
/// IS a WheeledVehicleController (plus a lean spring), so it is accepted here
/// too — every wheeled entry point works on one.
JPH::WheeledVehicleController *WheeledController(ZJoltVehicleConstraint *c) {
  JPH::VehicleConstraint *impl = Impl(c);
  if (impl == nullptr) return nullptr;
  if (c->controller_kind != ZJOLT_VEHICLE_CONTROLLER_KIND_WHEELED &&
      c->controller_kind != ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE)
    return nullptr;
  return static_cast<JPH::WheeledVehicleController *>(impl->GetController());
}
const JPH::WheeledVehicleController *WheeledController(
    const ZJoltVehicleConstraint *c) {
  const JPH::VehicleConstraint *impl = Impl(c);
  if (impl == nullptr) return nullptr;
  if (c->controller_kind != ZJOLT_VEHICLE_CONTROLLER_KIND_WHEELED &&
      c->controller_kind != ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE)
    return nullptr;
  return static_cast<const JPH::WheeledVehicleController *>(
      impl->GetController());
}

JPH::TrackedVehicleController *TrackedController(ZJoltVehicleConstraint *c) {
  JPH::VehicleConstraint *impl = Impl(c);
  if (impl == nullptr || c->controller_kind != ZJOLT_VEHICLE_CONTROLLER_KIND_TRACKED)
    return nullptr;
  return static_cast<JPH::TrackedVehicleController *>(impl->GetController());
}
const JPH::TrackedVehicleController *TrackedController(
    const ZJoltVehicleConstraint *c) {
  const JPH::VehicleConstraint *impl = Impl(c);
  if (impl == nullptr || c->controller_kind != ZJOLT_VEHICLE_CONTROLLER_KIND_TRACKED)
    return nullptr;
  return static_cast<const JPH::TrackedVehicleController *>(
      impl->GetController());
}

JPH::MotorcycleController *MotorcycleControllerOf(ZJoltVehicleConstraint *c) {
  JPH::VehicleConstraint *impl = Impl(c);
  if (impl == nullptr ||
      c->controller_kind != ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE)
    return nullptr;
  return static_cast<JPH::MotorcycleController *>(impl->GetController());
}
const JPH::MotorcycleController *MotorcycleControllerOf(
    const ZJoltVehicleConstraint *c) {
  const JPH::VehicleConstraint *impl = Impl(c);
  if (impl == nullptr ||
      c->controller_kind != ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE)
    return nullptr;
  return static_cast<const JPH::MotorcycleController *>(impl->GetController());
}

/// The engine, transmission, forward input and brake input read the same
/// way regardless of which of the two driveable controllers a constraint
/// has (see the "Driver input and drivetrain readback" entry points in the
/// header) — these four dispatch on kind so those entry points do not have
/// to.
const JPH::VehicleEngine *EngineOf(const ZJoltVehicleConstraint *c) {
  if (const JPH::WheeledVehicleController *wc = WheeledController(c))
    return &wc->GetEngine();
  if (const JPH::TrackedVehicleController *tc = TrackedController(c))
    return &tc->GetEngine();
  return nullptr;
}

JPH::VehicleEngine *EngineOf(ZJoltVehicleConstraint *c) {
  if (JPH::WheeledVehicleController *wc = WheeledController(c))
    return &wc->GetEngine();
  if (JPH::TrackedVehicleController *tc = TrackedController(c))
    return &tc->GetEngine();
  return nullptr;
}

const JPH::VehicleTransmission *TransmissionOf(const ZJoltVehicleConstraint *c) {
  if (const JPH::WheeledVehicleController *wc = WheeledController(c))
    return &wc->GetTransmission();
  if (const JPH::TrackedVehicleController *tc = TrackedController(c))
    return &tc->GetTransmission();
  return nullptr;
}

JPH::VehicleTransmission *TransmissionOf(ZJoltVehicleConstraint *c) {
  if (JPH::WheeledVehicleController *wc = WheeledController(c))
    return &wc->GetTransmission();
  if (JPH::TrackedVehicleController *tc = TrackedController(c))
    return &tc->GetTransmission();
  return nullptr;
}

float ForwardInputOf(const ZJoltVehicleConstraint *c) {
  if (const JPH::WheeledVehicleController *wc = WheeledController(c))
    return wc->GetForwardInput();
  if (const JPH::TrackedVehicleController *tc = TrackedController(c))
    return tc->GetForwardInput();
  return 0.0f;
}

float BrakeInputOf(const ZJoltVehicleConstraint *c) {
  if (const JPH::WheeledVehicleController *wc = WheeledController(c))
    return wc->GetBrakeInput();
  if (const JPH::TrackedVehicleController *tc = TrackedController(c))
    return tc->GetBrakeInput();
  return 0.0f;
}

//===----------------------------------------------------------------------===//
// Curves
//===----------------------------------------------------------------------===//

ZJoltVehicleCurveDesc ToCurveDesc(const JPH::LinearCurve &curve) {
  ZJoltVehicleCurveDesc out{};
  const uint32_t n = static_cast<uint32_t>(
      std::min<size_t>(curve.mPoints.size(), ZJOLT_VEHICLE_CURVE_MAX_POINTS));
  for (uint32_t i = 0; i < n; i++) {
    out.points[i] = ZJoltVehicleCurvePoint{curve.mPoints[i].mX, curve.mPoints[i].mY};
  }
  out.count = n;
  return out;
}

/// A count of 0 leaves `curve` at whatever its owning settings object's own
/// constructor already put there — see ZJoltVehicleCurveDesc in the header.
void ApplyCurve(JPH::LinearCurve &curve, const ZJoltVehicleCurveDesc &desc) {
  if (desc.count == 0) return;
  const uint32_t n = std::min<uint32_t>(desc.count, ZJOLT_VEHICLE_CURVE_MAX_POINTS);
  curve.Clear();
  curve.Reserve(n);
  for (uint32_t i = 0; i < n; i++) curve.AddPoint(desc.points[i].x, desc.points[i].y);
}

//===----------------------------------------------------------------------===//
// Wheel settings
//===----------------------------------------------------------------------===//

void ApplyBaseWheelSettings(JPH::WheelSettings *w, const ZJoltVehicleWheelDesc &d) {
  w->mPosition = zjolt::ToJolt(d.position);
  w->mSuspensionForcePoint = zjolt::ToJolt(d.suspension_force_point);
  w->mEnableSuspensionForcePoint = d.enable_suspension_force_point;
  // Wheel.cpp asserts each of these four is normalized (recon: Wheel.cpp:70-73).
  w->mSuspensionDirection =
      zjolt::ToJolt(d.suspension_direction).NormalizedOr(JPH::Vec3(0, -1, 0));
  w->mSteeringAxis = zjolt::ToJolt(d.steering_axis).NormalizedOr(JPH::Vec3::sAxisY());
  w->mWheelUp = zjolt::ToJolt(d.wheel_up).NormalizedOr(JPH::Vec3::sAxisY());
  w->mWheelForward =
      zjolt::ToJolt(d.wheel_forward).NormalizedOr(JPH::Vec3::sAxisZ());
  w->mSuspensionMinLength = d.suspension_min_length;
  w->mSuspensionMaxLength = d.suspension_max_length;
  w->mSuspensionPreloadLength = d.suspension_preload_length;
  w->mSuspensionSpring.mMode = JPH::ESpringMode::FrequencyAndDamping;
  w->mSuspensionSpring.mFrequency = d.suspension_spring_frequency;
  w->mSuspensionSpring.mDamping = d.suspension_spring_damping;
  w->mRadius = d.radius;
  w->mWidth = d.width;
}

JPH::WheelSettingsWV *BuildWheelSettingsWV(const ZJoltVehicleWheelDesc &d) {
  JPH::WheelSettingsWV *w = zjolt::New<JPH::WheelSettingsWV>();
  if (w == nullptr) return nullptr;
  ApplyBaseWheelSettings(w, d);
  w->mInertia = d.inertia;
  w->mAngularDamping = d.angular_damping;
  w->mMaxSteerAngle = d.max_steer_angle;
  ApplyCurve(w->mLongitudinalFriction, d.longitudinal_friction);
  ApplyCurve(w->mLateralFriction, d.lateral_friction);
  w->mMaxBrakeTorque = d.max_brake_torque;
  w->mMaxHandBrakeTorque = d.max_hand_brake_torque;
  return w;
}

JPH::WheelSettingsTV *BuildWheelSettingsTV(const ZJoltVehicleWheelDesc &d) {
  JPH::WheelSettingsTV *w = zjolt::New<JPH::WheelSettingsTV>();
  if (w == nullptr) return nullptr;
  ApplyBaseWheelSettings(w, d);
  w->mLongitudinalFriction = d.tracked_longitudinal_friction;
  w->mLateralFriction = d.tracked_lateral_friction;
  return w;
}

//===----------------------------------------------------------------------===//
// Engine, transmission, differentials
//===----------------------------------------------------------------------===//

void FillEngineSettings(JPH::VehicleEngineSettings &out, const ZJoltVehicleEngineDesc &d) {
  out.mMaxTorque = d.max_torque;
  out.mMinRPM = d.min_rpm;
  out.mMaxRPM = d.max_rpm;
  ApplyCurve(out.mNormalizedTorque, d.normalized_torque);
  out.mInertia = d.inertia;
  out.mAngularDamping = d.angular_damping;
}

void FillTransmissionSettings(JPH::VehicleTransmissionSettings &out,
                              const ZJoltVehicleTransmissionDesc &d) {
  out.mMode = d.mode == ZJOLT_VEHICLE_TRANSMISSION_MODE_MANUAL
                  ? JPH::ETransmissionMode::Manual
                  : JPH::ETransmissionMode::Auto;
  if (d.forward_gear_ratios != nullptr && d.forward_gear_ratio_count > 0) {
    out.mGearRatios.clear();
    out.mGearRatios.reserve(d.forward_gear_ratio_count);
    for (uint32_t i = 0; i < d.forward_gear_ratio_count; i++)
      out.mGearRatios.push_back(d.forward_gear_ratios[i]);
  }
  if (d.reverse_gear_ratios != nullptr && d.reverse_gear_ratio_count > 0) {
    out.mReverseGearRatios.clear();
    out.mReverseGearRatios.reserve(d.reverse_gear_ratio_count);
    for (uint32_t i = 0; i < d.reverse_gear_ratio_count; i++)
      out.mReverseGearRatios.push_back(d.reverse_gear_ratios[i]);
  }
  out.mSwitchTime = d.switch_time;
  out.mClutchReleaseTime = d.clutch_release_time;
  out.mSwitchLatency = d.switch_latency;
  out.mShiftUpRPM = d.shift_up_rpm;
  out.mShiftDownRPM = d.shift_down_rpm;
  out.mClutchStrength = d.clutch_strength;
}

/// -1 is only a wheel a differential is allowed to lack; every other index
/// must name a real wheel.
bool WheelIndexValid(int32_t index, uint32_t wheel_count, bool allow_none) {
  if (index < 0) return allow_none;
  return static_cast<uint32_t>(index) < wheel_count;
}

//===----------------------------------------------------------------------===//
// Numeric-range validation
//
// Wheel.cpp, WheeledVehicleController.cpp and TrackedVehicleController.cpp
// each JPH_ASSERT a laundry list of numeric ranges on the settings they are
// handed, none of it checked on this side before now — every one below is a
// recon'd assert line, turned into a returned ZJOLT_RESULT_INVALID_ARGUMENT
// instead of a caller-trippable abort in an asserts-on build. Every function
// here validates only; none of them mutate anything, so the caller of a
// batch (every wheel, every differential) sees the first failure with
// nothing partially applied.
//===----------------------------------------------------------------------===//

/// Wheel.cpp:74-80 — every wheel regardless of controller kind.
ZJoltResult ValidateWheelBase(const ZJoltVehicleWheelDesc &d) {
  if (!(d.suspension_min_length >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel suspension_min_length must be >= 0");
  if (!(d.suspension_max_length >= d.suspension_min_length))
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "wheel suspension_max_length must be >= suspension_min_length");
  if (!(d.suspension_preload_length >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel suspension_preload_length must be >= 0");
  if (!(d.suspension_spring_frequency > 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel suspension_spring_frequency must be > 0");
  if (!(d.suspension_spring_damping >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel suspension_spring_damping must be >= 0");
  if (!(d.radius > 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel radius must be > 0");
  if (!(d.width >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel width must be >= 0");
  return ZJOLT_RESULT_OK;
}

/// WheeledVehicleController.cpp:87-91 (WheelWV::WheelWV) — wheeled and
/// motorcycle wheels only; tracked wheels (WheelSettingsTV) carry none of
/// these fields.
ZJoltResult ValidateWheelWV(const ZJoltVehicleWheelDesc &d) {
  if (!(d.inertia >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel inertia must be >= 0");
  if (!(d.angular_damping >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel angular_damping must be >= 0");
  if (!(std::abs(d.max_steer_angle) <= 0.5f * JPH::JPH_PI))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel max_steer_angle must be within +/- pi/2");
  if (!(d.max_brake_torque >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel max_brake_torque must be >= 0");
  if (!(d.max_hand_brake_torque >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "wheel max_hand_brake_torque must be >= 0");
  return ZJOLT_RESULT_OK;
}

/// WheeledVehicleController.cpp:183-184 and TrackedVehicleController.cpp:
/// 135-136 — identical check, duplicated in both controllers' constructors,
/// so validated once here regardless of controller kind.
ZJoltResult ValidateEngineDesc(const ZJoltVehicleEngineDesc &d) {
  if (!(d.min_rpm >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "engine min_rpm must be >= 0");
  if (!(d.min_rpm <= d.max_rpm))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "engine min_rpm must be <= max_rpm");
  return ZJOLT_RESULT_OK;
}

/// WheeledVehicleController.cpp:195-199 and TrackedVehicleController.cpp:
/// 147-150 — the two controllers agree on every check except clutch_strength,
/// which TrackedVehicleController's constructor never asserts at all (a
/// tracked vehicle's clutch is not user-facing the way a manual wheeled
/// transmission's is), hence `require_clutch_strength`.
ZJoltResult ValidateTransmissionDesc(const ZJoltVehicleTransmissionDesc &d,
                                     const ZJoltVehicleEngineDesc &engine,
                                     bool require_clutch_strength) {
  if (d.forward_gear_ratios != nullptr) {
    for (uint32_t i = 0; i < d.forward_gear_ratio_count; i++) {
      if (!(d.forward_gear_ratios[i] > 0.0f))
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "forward gear ratios must be > 0");
    }
  }
  if (d.reverse_gear_ratios != nullptr) {
    for (uint32_t i = 0; i < d.reverse_gear_ratio_count; i++) {
      if (!(d.reverse_gear_ratios[i] < 0.0f))
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "reverse gear ratios must be < 0");
    }
  }
  if (!(d.switch_time >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "transmission switch_time must be >= 0");
  if (!(d.shift_down_rpm > 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "transmission shift_down_rpm must be > 0");
  if (d.mode == ZJOLT_VEHICLE_TRANSMISSION_MODE_AUTO &&
      !(d.shift_up_rpm < engine.max_rpm)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "auto transmission shift_up_rpm must be < engine max_rpm");
  }
  if (!(d.shift_up_rpm > d.shift_down_rpm)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "transmission shift_up_rpm must be > shift_down_rpm");
  }
  if (require_clutch_strength && !(d.clutch_strength > 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "transmission clutch_strength must be > 0");
  return ZJOLT_RESULT_OK;
}

/// WheeledVehicleController.cpp:207-210 — wheeled and motorcycle only;
/// tracked vehicles have no VehicleDifferentialSettings.
ZJoltResult ValidateDifferentialDesc(const ZJoltVehicleDifferentialDesc &d) {
  if (!(d.differential_ratio > 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "differential_ratio must be > 0");
  if (!(d.left_right_split >= 0.0f && d.left_right_split <= 1.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "differential left_right_split must be in [0, 1]");
  if (!(d.engine_torque_ratio >= 0.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "differential engine_torque_ratio must be >= 0");
  if (!(d.limited_slip_ratio > 1.0f))
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "differential limited_slip_ratio must be > 1");
  return ZJOLT_RESULT_OK;
}

/// Shared by WheeledVehicleControllerSettings and MotorcycleControllerSettings
/// (the latter IS-A the former). Returns an error without touching `cs`'s
/// caller-visible state beyond what it already owns; the caller deletes `cs`
/// itself on failure, since ownership has not been handed anywhere yet.
ZJoltResult FillWheeledCommon(JPH::WheeledVehicleControllerSettings *cs,
                              const ZJoltVehicleConstraintDesc &desc) {
  {
    const ZJoltResult r =
        ValidateTransmissionDesc(desc.transmission, desc.engine,
                                 /*require_clutch_strength=*/true);
    if (r != ZJOLT_RESULT_OK) return r;
  }
  FillEngineSettings(cs->mEngine, desc.engine);
  FillTransmissionSettings(cs->mTransmission, desc.transmission);
  if (desc.differential_count > 0 && desc.differentials == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "differential_count without differentials");
  }
  cs->mDifferentials.reserve(desc.differential_count);
  for (uint32_t i = 0; i < desc.differential_count; i++) {
    const ZJoltVehicleDifferentialDesc &d = desc.differentials[i];
    if (!WheelIndexValid(d.left_wheel, desc.wheel_count, true) ||
        !WheelIndexValid(d.right_wheel, desc.wheel_count, true)) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "differential wheel index out of range");
    }
    {
      const ZJoltResult r = ValidateDifferentialDesc(d);
      if (r != ZJOLT_RESULT_OK) return r;
    }
    JPH::VehicleDifferentialSettings jd;
    jd.mLeftWheel = d.left_wheel;
    jd.mRightWheel = d.right_wheel;
    jd.mDifferentialRatio = d.differential_ratio;
    jd.mLeftRightSplit = d.left_right_split;
    jd.mLimitedSlipRatio = d.limited_slip_ratio;
    jd.mEngineTorqueRatio = d.engine_torque_ratio;
    cs->mDifferentials.push_back(jd);
  }
  if (!(desc.differential_limited_slip_ratio > 1.0f)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "differential_limited_slip_ratio must be > 1");
  }
  cs->mDifferentialLimitedSlipRatio = desc.differential_limited_slip_ratio;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Collision tester
//===----------------------------------------------------------------------===//

JPH::VehicleCollisionTester *BuildCollisionTester(
    const ZJoltVehicleCollisionTesterDesc &d) {
  const JPH::Vec3 up = zjolt::ToJolt(d.up).NormalizedOr(JPH::Vec3::sAxisY());
  switch (d.kind) {
    case ZJOLT_VEHICLE_COLLISION_TESTER_KIND_CAST_SPHERE:
      return zjolt::New<JPH::VehicleCollisionTesterCastSphere>(
          static_cast<JPH::ObjectLayer>(d.object_layer), d.radius, up,
          d.max_slope_angle);
    case ZJOLT_VEHICLE_COLLISION_TESTER_KIND_CAST_CYLINDER: {
      // VehicleCollisionTester.h asserts convex_radius_fraction is in [0, 1]
      // (recon: VehicleCollisionTester.h:136); clamped rather than propagated.
      float fraction = d.convex_radius_fraction;
      if (!(fraction >= 0.0f && fraction <= 1.0f)) fraction = 0.1f;
      return zjolt::New<JPH::VehicleCollisionTesterCastCylinder>(
          static_cast<JPH::ObjectLayer>(d.object_layer), fraction);
    }
    case ZJOLT_VEHICLE_COLLISION_TESTER_KIND_RAY:
    default:
      return zjolt::New<JPH::VehicleCollisionTesterRay>(
          static_cast<JPH::ObjectLayer>(d.object_layer), up, d.max_slope_angle);
  }
}

/// Points the tester at whichever of the three adapters the caller actually
/// filled in. A level with a NULL function pointer is left unset rather than
/// pointed at an accept-everything adapter, because "unset" is what makes
/// Jolt fall back to the tester's own object layer — an adapter that says yes
/// to everything would instead switch that check off.
void ApplyFilters(ZJoltVehicleConstraint *c) {
  if (c == nullptr || c->tester == nullptr) return;

  c->broad_phase_filter.table = c->filters.broad_phase_layer;
  c->object_layer_filter.table = c->filters.object_layer;
  c->body_filter.table = c->filters.body;

  const JPH::VehicleConstraint *impl = Impl(c);
  c->body_filter.vehicle_body =
      (impl != nullptr && impl->GetVehicleBody() != nullptr)
          ? impl->GetVehicleBody()->GetID()
          : JPH::BodyID();

  c->tester->SetBroadPhaseLayerFilter(
      c->filters.broad_phase_layer.should_collide != nullptr
          ? &c->broad_phase_filter
          : nullptr);
  c->tester->SetObjectLayerFilter(
      c->filters.object_layer.should_collide != nullptr ? &c->object_layer_filter
                                                        : nullptr);
  c->tester->SetBodyFilter(
      c->filters.body.should_collide != nullptr ? &c->body_filter : nullptr);
}

//===----------------------------------------------------------------------===//
// Callback shims
//
// Each captures only the handle and re-reads the C table out of it, so
// clearing a callback takes effect even while Jolt still holds the shim, and
// one pointer is all the capture std::function has to store (no allocation).
//===----------------------------------------------------------------------===//

void InvokeStepCallback(const ZJoltVehicleStepCallback &cb,
                        ZJoltVehicleConstraint *handle,
                        const JPH::PhysicsStepListenerContext &context) {
  if (cb.on_step == nullptr) return;
  const ZJoltVehicleStepContext c{context.mDeltaTime, context.mIsFirstStep,
                                  context.mIsLastStep};
  cb.on_step(cb.user, handle, &c);
}

/// Jolt's own default, respelled here because clearing a callback has to put
/// it back and Jolt only writes it in VehicleConstraint's member initialiser
/// (VehicleConstraint.h:238) — there is nothing to copy it from afterwards.
JPH::VehicleConstraint::CombineFunction DefaultCombineFriction() {
  return [](JPH::uint, float &io_longitudinal, float &io_lateral,
            const JPH::Body &body2, const JPH::SubShapeID &) {
    const float body_friction = body2.GetFriction();
    io_longitudinal = std::sqrt(io_longitudinal * body_friction);
    io_lateral = std::sqrt(io_lateral * body_friction);
  };
}

/// As DefaultCombineFriction, for WheeledVehicleController.h:191.
JPH::WheeledVehicleController::TireMaxImpulseCallback DefaultTireMaxImpulse() {
  return [](JPH::uint, float &out_longitudinal_impulse,
            float &out_lateral_impulse, float suspension_impulse,
            float longitudinal_friction, float lateral_friction, float, float,
            float) {
    out_longitudinal_impulse = longitudinal_friction * suspension_impulse;
    out_lateral_impulse = lateral_friction * suspension_impulse;
  };
}

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Descriptor defaults
//
// Every DescInit reads out of a default-constructed Jolt settings object
// rather than transcribing numbers, so an upstream tuning change moves with
// it (same reasoning as zjoltCharacterDescInit).
//===----------------------------------------------------------------------===//

void zjoltVehicleWheelDescInit(ZJoltVehicleWheelDesc *desc) {
  if (desc == nullptr) return;
  *desc = ZJoltVehicleWheelDesc{};

  const JPH::WheelSettings base;
  desc->position = zjolt::ToC(base.mPosition);
  desc->suspension_force_point = zjolt::ToC(base.mSuspensionForcePoint);
  desc->enable_suspension_force_point = base.mEnableSuspensionForcePoint;
  desc->suspension_direction = zjolt::ToC(base.mSuspensionDirection);
  desc->steering_axis = zjolt::ToC(base.mSteeringAxis);
  desc->wheel_up = zjolt::ToC(base.mWheelUp);
  desc->wheel_forward = zjolt::ToC(base.mWheelForward);
  desc->suspension_min_length = base.mSuspensionMinLength;
  desc->suspension_max_length = base.mSuspensionMaxLength;
  desc->suspension_preload_length = base.mSuspensionPreloadLength;
  desc->suspension_spring_frequency = base.mSuspensionSpring.mFrequency;
  desc->suspension_spring_damping = base.mSuspensionSpring.mDamping;
  desc->radius = base.mRadius;
  desc->width = base.mWidth;

  const JPH::WheelSettingsWV wv;
  desc->inertia = wv.mInertia;
  desc->angular_damping = wv.mAngularDamping;
  desc->max_steer_angle = wv.mMaxSteerAngle;
  desc->longitudinal_friction = ToCurveDesc(wv.mLongitudinalFriction);
  desc->lateral_friction = ToCurveDesc(wv.mLateralFriction);
  desc->max_brake_torque = wv.mMaxBrakeTorque;
  desc->max_hand_brake_torque = wv.mMaxHandBrakeTorque;

  const JPH::WheelSettingsTV tv;
  desc->tracked_longitudinal_friction = tv.mLongitudinalFriction;
  desc->tracked_lateral_friction = tv.mLateralFriction;
}

void zjoltVehicleEngineDescInit(ZJoltVehicleEngineDesc *desc) {
  if (desc == nullptr) return;
  const JPH::VehicleEngineSettings defaults;
  desc->max_torque = defaults.mMaxTorque;
  desc->min_rpm = defaults.mMinRPM;
  desc->max_rpm = defaults.mMaxRPM;
  desc->normalized_torque = ToCurveDesc(defaults.mNormalizedTorque);
  desc->inertia = defaults.mInertia;
  desc->angular_damping = defaults.mAngularDamping;
}

void zjoltVehicleTransmissionDescInit(ZJoltVehicleTransmissionDesc *desc) {
  if (desc == nullptr) return;
  *desc = ZJoltVehicleTransmissionDesc{};
  const JPH::VehicleTransmissionSettings defaults;
  desc->mode = defaults.mMode == JPH::ETransmissionMode::Manual
                   ? ZJOLT_VEHICLE_TRANSMISSION_MODE_MANUAL
                   : ZJOLT_VEHICLE_TRANSMISSION_MODE_AUTO;
  // Gear ratios are borrowed pointers in the descriptor; DescInit has no
  // storage of its own to hand back, so it reports none, and Create leaves
  // Jolt's own default ratios in place for a 0 count.
  desc->switch_time = defaults.mSwitchTime;
  desc->clutch_release_time = defaults.mClutchReleaseTime;
  desc->switch_latency = defaults.mSwitchLatency;
  desc->shift_up_rpm = defaults.mShiftUpRPM;
  desc->shift_down_rpm = defaults.mShiftDownRPM;
  desc->clutch_strength = defaults.mClutchStrength;
}

void zjoltVehicleDifferentialDescInit(ZJoltVehicleDifferentialDesc *desc) {
  if (desc == nullptr) return;
  const JPH::VehicleDifferentialSettings defaults;
  desc->left_wheel = defaults.mLeftWheel;
  desc->right_wheel = defaults.mRightWheel;
  desc->differential_ratio = defaults.mDifferentialRatio;
  desc->left_right_split = defaults.mLeftRightSplit;
  desc->limited_slip_ratio = defaults.mLimitedSlipRatio;
  desc->engine_torque_ratio = defaults.mEngineTorqueRatio;
}

void zjoltVehicleAntiRollBarDescInit(ZJoltVehicleAntiRollBarDesc *desc) {
  if (desc == nullptr) return;
  const JPH::VehicleAntiRollBar defaults;
  desc->left_wheel = defaults.mLeftWheel;
  desc->right_wheel = defaults.mRightWheel;
  desc->stiffness = defaults.mStiffness;
}

void zjoltVehicleCollisionTesterDescInit(ZJoltVehicleCollisionTesterDesc *desc) {
  if (desc == nullptr) return;
  *desc = ZJoltVehicleCollisionTesterDesc{};
  desc->kind = ZJOLT_VEHICLE_COLLISION_TESTER_KIND_RAY;
  desc->object_layer = 0;
  desc->up = zjolt::ToC(JPH::Vec3::sAxisY());
  desc->max_slope_angle = JPH::DegreesToRadians(80.0f);
  desc->radius = 0.1f;
  desc->convex_radius_fraction = 0.1f;
}

void zjoltVehicleMotorcycleDescInit(ZJoltVehicleMotorcycleDesc *desc) {
  if (desc == nullptr) return;
  const JPH::MotorcycleControllerSettings defaults;
  desc->max_lean_angle = defaults.mMaxLeanAngle;
  desc->lean_spring_constant = defaults.mLeanSpringConstant;
  desc->lean_spring_damping = defaults.mLeanSpringDamping;
  desc->lean_spring_integration_coefficient =
      defaults.mLeanSpringIntegrationCoefficient;
  desc->lean_spring_integration_coefficient_decay =
      defaults.mLeanSpringIntegrationCoefficientDecay;
  desc->lean_smoothing_factor = defaults.mLeanSmoothingFactor;
}

void zjoltVehicleConstraintDescInit(ZJoltVehicleConstraintDesc *desc) {
  if (desc == nullptr) return;
  *desc = ZJoltVehicleConstraintDesc{};
  const JPH::VehicleConstraintSettings defaults;
  desc->up = zjolt::ToC(defaults.mUp);
  desc->forward = zjolt::ToC(defaults.mForward);
  desc->max_pitch_roll_angle = defaults.mMaxPitchRollAngle;
  desc->controller_kind = ZJOLT_VEHICLE_CONTROLLER_KIND_WHEELED;
  zjoltVehicleEngineDescInit(&desc->engine);
  zjoltVehicleTransmissionDescInit(&desc->transmission);
  desc->differential_limited_slip_ratio = 1.4f;

  // VehicleTrackSettings::mDrivenWheel has no in-class default, which makes
  // the type ineligible for `const JPH::VehicleTrackSettings defaults;`
  // (a const object needs every member initialized). Its other fields'
  // defaults are transcribed from VehicleTrack.h instead.
  desc->track_inertia = 10.0f;
  desc->track_angular_damping = 0.5f;
  desc->track_max_brake_torque = 15000.0f;
  desc->track_differential_ratio = 6.0f;

  zjoltVehicleMotorcycleDescInit(&desc->motorcycle);
  zjoltVehicleCollisionTesterDescInit(&desc->collision_tester);
}

//===----------------------------------------------------------------------===//
// Lifetime
//===----------------------------------------------------------------------===//

ZJoltResult zjoltVehicleConstraintCreate(ZJoltPhysicsSystem *system,
                                         ZJoltBodyId body,
                                         const ZJoltVehicleConstraintDesc *desc,
                                         ZJoltVehicleConstraint **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(system, desc, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (desc->wheels == nullptr || desc->wheel_count == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a vehicle needs at least one wheel");
  }
  if (desc->anti_roll_bar_count > 0 && desc->anti_roll_bars == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "anti_roll_bar_count without anti_roll_bars");
  }

  const bool tracked = desc->controller_kind == ZJOLT_VEHICLE_CONTROLLER_KIND_TRACKED;

  // Engine range checks are identical in WheeledVehicleController's and
  // TrackedVehicleController's constructors (recon: WheeledVehicleController.
  // cpp:183-184, TrackedVehicleController.cpp:135-136), so validated once
  // here rather than per controller kind.
  {
    const ZJoltResult r = ValidateEngineDesc(desc->engine);
    if (r != ZJOLT_RESULT_OK) return r;
  }

  // Every wheel's base fields (Wheel.cpp:74-80), plus the wheeled/motorcycle-
  // only fields (WheeledVehicleController.cpp:87-91) when this is not a
  // tracked vehicle. Validated before any Jolt object exists, so a bad wheel
  // partway through the array never leaves anything to clean up.
  for (uint32_t i = 0; i < desc->wheel_count; i++) {
    ZJoltResult r = ValidateWheelBase(desc->wheels[i]);
    if (r != ZJOLT_RESULT_OK) return r;
    if (!tracked) {
      r = ValidateWheelWV(desc->wheels[i]);
      if (r != ZJOLT_RESULT_OK) return r;
    }
  }

  JPH::VehicleConstraintSettings settings;
  // VehicleConstraint.cpp asserts both are normalized (recon lines 87-88).
  settings.mUp = zjolt::ToJolt(desc->up).NormalizedOr(JPH::Vec3::sAxisY());
  settings.mForward = zjolt::ToJolt(desc->forward).NormalizedOr(JPH::Vec3::sAxisZ());
  settings.mMaxPitchRollAngle = desc->max_pitch_roll_angle;

  settings.mWheels.reserve(desc->wheel_count);
  for (uint32_t i = 0; i < desc->wheel_count; i++) {
    JPH::WheelSettings *w =
        tracked ? static_cast<JPH::WheelSettings *>(
                      BuildWheelSettingsTV(desc->wheels[i]))
                : static_cast<JPH::WheelSettings *>(
                      BuildWheelSettingsWV(desc->wheels[i]));
    if (w == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
    settings.mWheels.push_back(w);
  }

  for (uint32_t i = 0; i < desc->anti_roll_bar_count; i++) {
    const ZJoltVehicleAntiRollBarDesc &bar = desc->anti_roll_bars[i];
    if (!WheelIndexValid(bar.left_wheel, desc->wheel_count, false) ||
        !WheelIndexValid(bar.right_wheel, desc->wheel_count, false)) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "anti-roll bar wheel index out of range");
    }
    // VehicleConstraint.cpp asserts mStiffness >= 0 (recon line 282).
    if (bar.stiffness < 0.0f) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "anti-roll bar stiffness must be >= 0");
    }
    JPH::VehicleAntiRollBar jbar;
    jbar.mLeftWheel = bar.left_wheel;
    jbar.mRightWheel = bar.right_wheel;
    jbar.mStiffness = bar.stiffness;
    settings.mAntiRollBars.push_back(jbar);
  }

  JPH::Ref<JPH::VehicleControllerSettings> controller_settings;
  switch (desc->controller_kind) {
    case ZJOLT_VEHICLE_CONTROLLER_KIND_TRACKED: {
      auto *cs = zjolt::New<JPH::TrackedVehicleControllerSettings>();
      if (cs == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
      {
        // TrackedVehicleController's constructor never asserts
        // mClutchStrength (recon: TrackedVehicleController.cpp:135-150), so
        // require_clutch_strength is false here unlike FillWheeledCommon.
        const ZJoltResult r =
            ValidateTransmissionDesc(desc->transmission, desc->engine,
                                     /*require_clutch_strength=*/false);
        if (r != ZJOLT_RESULT_OK) {
          zjolt::Delete(cs);
          return r;
        }
      }
      FillEngineSettings(cs->mEngine, desc->engine);
      FillTransmissionSettings(cs->mTransmission, desc->transmission);

      if (desc->left_track_wheel_count == 0 || desc->right_track_wheel_count == 0 ||
          desc->left_track_wheels == nullptr || desc->right_track_wheels == nullptr) {
        zjolt::Delete(cs);
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "a tracked vehicle needs wheels on both tracks");
      }
      if (desc->left_track_driven_wheel >= desc->wheel_count ||
          desc->right_track_driven_wheel >= desc->wheel_count) {
        zjolt::Delete(cs);
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "track driven wheel index out of range");
      }
      JPH::VehicleTrackSettings &left = cs->mTracks[static_cast<int>(JPH::ETrackSide::Left)];
      JPH::VehicleTrackSettings &right = cs->mTracks[static_cast<int>(JPH::ETrackSide::Right)];
      left.mDrivenWheel = desc->left_track_driven_wheel;
      right.mDrivenWheel = desc->right_track_driven_wheel;
      for (uint32_t i = 0; i < desc->left_track_wheel_count; i++) {
        if (desc->left_track_wheels[i] >= desc->wheel_count) {
          zjolt::Delete(cs);
          return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                                 "track wheel index out of range");
        }
        left.mWheels.push_back(desc->left_track_wheels[i]);
      }
      for (uint32_t i = 0; i < desc->right_track_wheel_count; i++) {
        if (desc->right_track_wheels[i] >= desc->wheel_count) {
          zjolt::Delete(cs);
          return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                                 "track wheel index out of range");
        }
        right.mWheels.push_back(desc->right_track_wheels[i]);
      }
      // TrackedVehicleController.cpp:157-160 — asserted per track there,
      // but the two tracks share one scalar each in this ABI so one check
      // covers both.
      if (!(desc->track_inertia >= 0.0f)) {
        zjolt::Delete(cs);
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "track_inertia must be >= 0");
      }
      if (!(desc->track_angular_damping >= 0.0f)) {
        zjolt::Delete(cs);
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "track_angular_damping must be >= 0");
      }
      if (!(desc->track_max_brake_torque >= 0.0f)) {
        zjolt::Delete(cs);
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "track_max_brake_torque must be >= 0");
      }
      if (!(desc->track_differential_ratio > 0.0f)) {
        zjolt::Delete(cs);
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "track_differential_ratio must be > 0");
      }
      left.mInertia = right.mInertia = desc->track_inertia;
      left.mAngularDamping = right.mAngularDamping = desc->track_angular_damping;
      left.mMaxBrakeTorque = right.mMaxBrakeTorque = desc->track_max_brake_torque;
      left.mDifferentialRatio = right.mDifferentialRatio = desc->track_differential_ratio;
      controller_settings = cs;
      break;
    }
    case ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE: {
      auto *cs = zjolt::New<JPH::MotorcycleControllerSettings>();
      if (cs == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
      const ZJoltResult r = FillWheeledCommon(cs, *desc);
      if (r != ZJOLT_RESULT_OK) {
        zjolt::Delete(cs);
        return r;
      }
      cs->mMaxLeanAngle = desc->motorcycle.max_lean_angle;
      cs->mLeanSpringConstant = desc->motorcycle.lean_spring_constant;
      cs->mLeanSpringDamping = desc->motorcycle.lean_spring_damping;
      cs->mLeanSpringIntegrationCoefficient =
          desc->motorcycle.lean_spring_integration_coefficient;
      cs->mLeanSpringIntegrationCoefficientDecay =
          desc->motorcycle.lean_spring_integration_coefficient_decay;
      cs->mLeanSmoothingFactor = desc->motorcycle.lean_smoothing_factor;
      controller_settings = cs;
      break;
    }
    case ZJOLT_VEHICLE_CONTROLLER_KIND_WHEELED:
    default: {
      auto *cs = zjolt::New<JPH::WheeledVehicleControllerSettings>();
      if (cs == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
      const ZJoltResult r = FillWheeledCommon(cs, *desc);
      if (r != ZJOLT_RESULT_OK) {
        zjolt::Delete(cs);
        return r;
      }
      controller_settings = cs;
      break;
    }
  }
  settings.mController = controller_settings;

  JPH::BodyLockWrite lock(system->system.GetBodyLockInterface(), zjolt::ToJolt(body));
  if (!lock.Succeeded()) {
    return zjolt::SetError(ZJOLT_RESULT_BODY_NOT_FOUND,
                           "vehicle body not found");
  }

  JPH::VehicleConstraint *fresh =
      zjolt::New<JPH::VehicleConstraint>(lock.GetBody(), settings);
  if (fresh == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  JPH::VehicleCollisionTester *tester = BuildCollisionTester(desc->collision_tester);
  if (tester == nullptr) {
    zjolt::Delete(fresh);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }
  fresh->SetVehicleCollisionTester(tester);

  system->system.AddConstraint(fresh);
  system->system.AddStepListener(fresh);

  ZJoltVehicleConstraint *handle = zjolt::New<ZJoltVehicleConstraint>();
  if (handle == nullptr) {
    system->system.RemoveStepListener(fresh);
    system->system.RemoveConstraint(fresh);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }

  handle->impl = fresh;
  handle->owner = system;
  handle->controller_kind = desc->controller_kind;
  handle->collision_tester_kind = desc->collision_tester.kind;
  // A second reference on top of the one SetVehicleCollisionTester took, so
  // the handle can hand out a mutable tester (filters, swapping) for as long
  // as it lives. Dropped in zjoltVehicleConstraintDestroy.
  handle->tester = tester;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltVehicleConstraintDestroy(ZJoltVehicleConstraint *constraint) {
  if (constraint == nullptr) return;
  JPH::VehicleConstraint *impl = constraint->impl.GetPtr();
  if (impl != nullptr) {
    constraint->owner->system.RemoveStepListener(impl);
    constraint->owner->system.RemoveConstraint(impl);
  }
  // Drops the handle's own reference. If this is the last one (the usual
  // case — RemoveConstraint above dropped the system's), the constraint
  // deletes itself through Jolt's allocator here — and with it the tester
  // reference it holds, leaving the handle's own `tester` below as the last.
  constraint->impl = nullptr;
  constraint->tester = nullptr;
  zjolt::Delete(constraint);
  zjolt::HandleDestroyed();
}

//===----------------------------------------------------------------------===//
// Vehicle-level state
//===----------------------------------------------------------------------===//

ZJoltVehicleControllerKind zjoltVehicleConstraintGetControllerKind(
    const ZJoltVehicleConstraint *constraint) {
  return constraint != nullptr ? constraint->controller_kind
                               : ZJOLT_VEHICLE_CONTROLLER_KIND_WHEELED;
}

ZJoltVehicleCollisionTesterKind zjoltVehicleConstraintGetCollisionTesterKind(
    const ZJoltVehicleConstraint *constraint) {
  return constraint != nullptr ? constraint->collision_tester_kind
                               : ZJOLT_VEHICLE_COLLISION_TESTER_KIND_RAY;
}

ZJoltBodyId zjoltVehicleConstraintGetBodyId(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr || impl->GetVehicleBody() == nullptr) return ZJOLT_BODY_ID_INVALID;
  return zjolt::ToC(impl->GetVehicleBody()->GetID());
}

bool zjoltVehicleConstraintIsActive(const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  return impl != nullptr && impl->IsActive();
}

float zjoltVehicleConstraintGetMaxPitchRollAngle(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  return impl != nullptr ? impl->GetMaxPitchRollAngle() : 0.0f;
}

void zjoltVehicleConstraintSetMaxPitchRollAngle(
    ZJoltVehicleConstraint *constraint, float max_pitch_roll_angle) {
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl != nullptr) impl->SetMaxPitchRollAngle(max_pitch_roll_angle);
}

void zjoltVehicleConstraintGetLocalUp(const ZJoltVehicleConstraint *constraint,
                                      ZJoltVec3 *out) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  zjolt::WriteVec3(out, impl != nullptr ? impl->GetLocalUp() : JPH::Vec3::sZero());
}

void zjoltVehicleConstraintGetLocalForward(
    const ZJoltVehicleConstraint *constraint, ZJoltVec3 *out) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  zjolt::WriteVec3(out, impl != nullptr ? impl->GetLocalForward() : JPH::Vec3::sZero());
}

void zjoltVehicleConstraintGetWorldUp(const ZJoltVehicleConstraint *constraint,
                                      ZJoltVec3 *out) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  zjolt::WriteVec3(out, impl != nullptr ? impl->GetWorldUp() : JPH::Vec3::sZero());
}

//===----------------------------------------------------------------------===//
// Wheels — runtime state
//===----------------------------------------------------------------------===//

uint32_t zjoltVehicleConstraintGetWheelCount(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  return impl != nullptr ? static_cast<uint32_t>(impl->GetWheels().size()) : 0;
}

float zjoltVehicleConstraintGetWheelRotationAngle(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr ? w->GetRotationAngle() : 0.0f;
}

void zjoltVehicleConstraintSetWheelRotationAngle(
    ZJoltVehicleConstraint *constraint, uint32_t wheel_index, float angle) {
  JPH::Wheel *w = WheelAt(constraint, wheel_index);
  if (w != nullptr) w->SetRotationAngle(angle);
}

float zjoltVehicleConstraintGetWheelSteerAngle(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr ? w->GetSteerAngle() : 0.0f;
}

void zjoltVehicleConstraintSetWheelSteerAngle(
    ZJoltVehicleConstraint *constraint, uint32_t wheel_index, float angle) {
  JPH::Wheel *w = WheelAt(constraint, wheel_index);
  if (w != nullptr) w->SetSteerAngle(angle);
}

float zjoltVehicleConstraintGetWheelAngularVelocity(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr ? w->GetAngularVelocity() : 0.0f;
}

void zjoltVehicleConstraintSetWheelAngularVelocity(
    ZJoltVehicleConstraint *constraint, uint32_t wheel_index, float velocity) {
  JPH::Wheel *w = WheelAt(constraint, wheel_index);
  if (w != nullptr) w->SetAngularVelocity(velocity);
}

float zjoltVehicleConstraintGetWheelSuspensionLength(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr ? w->GetSuspensionLength() : 0.0f;
}

bool zjoltVehicleConstraintHasWheelContact(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr && w->HasContact();
}

ZJoltBodyId zjoltVehicleConstraintGetWheelContactBodyId(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  if (w == nullptr || !w->HasContact()) return ZJOLT_BODY_ID_INVALID;
  return zjolt::ToC(w->GetContactBodyID());
}

// Wheel.h guards every contact getter below with JPH_ASSERT(HasContact()), so
// each is called only once HasContact() is already known true.

void zjoltVehicleConstraintGetWheelContactPosition(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltRVec3 *out) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  zjolt::WriteRVec3(out, (w != nullptr && w->HasContact()) ? w->GetContactPosition()
                                                           : JPH::RVec3::sZero());
}

void zjoltVehicleConstraintGetWheelContactNormal(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  zjolt::WriteVec3(out, (w != nullptr && w->HasContact()) ? w->GetContactNormal()
                                                          : JPH::Vec3::sZero());
}

void zjoltVehicleConstraintGetWheelContactLongitudinal(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  zjolt::WriteVec3(out, (w != nullptr && w->HasContact())
                            ? w->GetContactLongitudinal()
                            : JPH::Vec3::sZero());
}

void zjoltVehicleConstraintGetWheelContactLateral(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  zjolt::WriteVec3(out, (w != nullptr && w->HasContact()) ? w->GetContactLateral()
                                                          : JPH::Vec3::sZero());
}

void zjoltVehicleConstraintGetWheelContactPointVelocity(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  zjolt::WriteVec3(out, (w != nullptr && w->HasContact())
                            ? w->GetContactPointVelocity()
                            : JPH::Vec3::sZero());
}

//===----------------------------------------------------------------------===//
// Driver input and drivetrain readback — every controller kind
//===----------------------------------------------------------------------===//

float zjoltVehicleConstraintGetForwardInput(
    const ZJoltVehicleConstraint *constraint) {
  return ForwardInputOf(constraint);
}

float zjoltVehicleConstraintGetBrakeInput(
    const ZJoltVehicleConstraint *constraint) {
  return BrakeInputOf(constraint);
}

float zjoltVehicleConstraintGetEngineRpm(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleEngine *e = EngineOf(constraint);
  return e != nullptr ? e->GetCurrentRPM() : 0.0f;
}

int32_t zjoltVehicleConstraintGetCurrentGear(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleTransmission *t = TransmissionOf(constraint);
  return t != nullptr ? t->GetCurrentGear() : 0;
}

bool zjoltVehicleConstraintIsSwitchingGear(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleTransmission *t = TransmissionOf(constraint);
  return t != nullptr && t->IsSwitchingGear();
}

float zjoltVehicleConstraintGetClutchFriction(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleTransmission *t = TransmissionOf(constraint);
  return t != nullptr ? t->GetClutchFriction() : 0.0f;
}

float zjoltVehicleConstraintGetGearRatio(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleTransmission *t = TransmissionOf(constraint);
  return t != nullptr ? t->GetCurrentRatio() : 0.0f;
}

ZJoltResult zjoltVehicleConstraintSetGear(ZJoltVehicleConstraint *constraint,
                                          int32_t gear, float clutch_friction) {
  ZJOLT_ENTER();
  JPH::VehicleTransmission *t = TransmissionOf(constraint);
  if (t == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  t->Set(gear, clutch_friction);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Wheeled and motorcycle controller
//===----------------------------------------------------------------------===//

ZJoltResult zjoltVehicleConstraintSetWheeledDriverInput(
    ZJoltVehicleConstraint *constraint, float forward, float right,
    float brake, float hand_brake) {
  ZJOLT_ENTER();
  JPH::WheeledVehicleController *wc = WheeledController(constraint);
  if (wc == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  wc->SetDriverInput(forward, right, brake, hand_brake);
  return ZJOLT_RESULT_OK;
}

float zjoltVehicleConstraintGetWheeledRightInput(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetRightInput() : 0.0f;
}

float zjoltVehicleConstraintGetWheeledHandBrakeInput(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetHandBrakeInput() : 0.0f;
}

//===----------------------------------------------------------------------===//
// Tracked controller
//===----------------------------------------------------------------------===//

ZJoltResult zjoltVehicleConstraintSetTrackedDriverInput(
    ZJoltVehicleConstraint *constraint, float forward, float left_ratio,
    float right_ratio, float brake) {
  ZJOLT_ENTER();
  JPH::TrackedVehicleController *tc = TrackedController(constraint);
  if (tc == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  // TrackedVehicleController.h asserts both ratios are nonzero; a caller's
  // exact zero becomes "full rate" rather than tripping that assert.
  const float left = left_ratio != 0.0f ? left_ratio : 1.0f;
  const float right = right_ratio != 0.0f ? right_ratio : 1.0f;
  tc->SetDriverInput(forward, left, right, brake);
  return ZJOLT_RESULT_OK;
}

float zjoltVehicleConstraintGetTrackedLeftRatio(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::TrackedVehicleController *tc = TrackedController(constraint);
  return tc != nullptr ? tc->GetLeftRatio() : 0.0f;
}

float zjoltVehicleConstraintGetTrackedRightRatio(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::TrackedVehicleController *tc = TrackedController(constraint);
  return tc != nullptr ? tc->GetRightRatio() : 0.0f;
}

//===----------------------------------------------------------------------===//
// Motorcycle controller
//===----------------------------------------------------------------------===//

ZJoltResult zjoltVehicleConstraintSetMotorcycleLeanControllerEnabled(
    ZJoltVehicleConstraint *constraint, bool enabled) {
  ZJOLT_ENTER();
  JPH::MotorcycleController *mc = MotorcycleControllerOf(constraint);
  if (mc == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  mc->EnableLeanController(enabled);
  return ZJOLT_RESULT_OK;
}

bool zjoltVehicleConstraintIsMotorcycleLeanControllerEnabled(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::MotorcycleController *mc = MotorcycleControllerOf(constraint);
  return mc != nullptr && mc->IsLeanControllerEnabled();
}

ZJoltResult zjoltVehicleConstraintSetMotorcycleLeanSteeringLimitEnabled(
    ZJoltVehicleConstraint *constraint, bool enabled) {
  ZJOLT_ENTER();
  JPH::MotorcycleController *mc = MotorcycleControllerOf(constraint);
  if (mc == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  mc->EnableLeanSteeringLimit(enabled);
  return ZJOLT_RESULT_OK;
}

bool zjoltVehicleConstraintIsMotorcycleLeanSteeringLimitEnabled(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::MotorcycleController *mc = MotorcycleControllerOf(constraint);
  return mc != nullptr && mc->IsLeanSteeringLimitEnabled();
}

float zjoltVehicleConstraintGetMotorcycleWheelBase(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::MotorcycleController *mc = MotorcycleControllerOf(constraint);
  return mc != nullptr ? mc->GetWheelBase() : 0.0f;
}

//===----------------------------------------------------------------------===//
// Gravity override
//===----------------------------------------------------------------------===//

void zjoltVehicleConstraintOverrideGravity(ZJoltVehicleConstraint *constraint,
                                           const ZJoltVec3 *gravity) {
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr || gravity == nullptr) return;
  impl->OverrideGravity(zjolt::ToJolt(*gravity));
}

void zjoltVehicleConstraintResetGravityOverride(
    ZJoltVehicleConstraint *constraint) {
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl != nullptr) impl->ResetGravityOverride();
}

bool zjoltVehicleConstraintIsGravityOverridden(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  return impl != nullptr && impl->IsGravityOverridden();
}

void zjoltVehicleConstraintGetGravityOverride(
    const ZJoltVehicleConstraint *constraint, ZJoltVec3 *out) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  zjolt::WriteVec3(out, impl != nullptr ? impl->GetGravityOverride() : JPH::Vec3::sZero());
}

//===----------------------------------------------------------------------===//
// Wheel-ground collision test frequency
//===----------------------------------------------------------------------===//

uint32_t zjoltVehicleConstraintGetNumStepsBetweenCollisionTestActive(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  return impl != nullptr ? impl->GetNumStepsBetweenCollisionTestActive() : 0;
}

void zjoltVehicleConstraintSetNumStepsBetweenCollisionTestActive(
    ZJoltVehicleConstraint *constraint, uint32_t steps) {
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl != nullptr) impl->SetNumStepsBetweenCollisionTestActive(steps);
}

uint32_t zjoltVehicleConstraintGetNumStepsBetweenCollisionTestInactive(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  return impl != nullptr ? impl->GetNumStepsBetweenCollisionTestInactive() : 0;
}

void zjoltVehicleConstraintSetNumStepsBetweenCollisionTestInactive(
    ZJoltVehicleConstraint *constraint, uint32_t steps) {
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl != nullptr) impl->SetNumStepsBetweenCollisionTestInactive(steps);
}

//===----------------------------------------------------------------------===//
// Wheel force and pose readback
//===----------------------------------------------------------------------===//

ZJoltSubShapeId zjoltVehicleConstraintGetWheelContactSubShapeId(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  if (w == nullptr || !w->HasContact()) return ZJOLT_SUB_SHAPE_ID_EMPTY;
  return zjolt::ToC(w->GetContactSubShapeID());
}

bool zjoltVehicleConstraintHasWheelHitHardPoint(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr && w->HasHitHardPoint();
}

float zjoltVehicleConstraintGetWheelSuspensionLambda(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr ? w->GetSuspensionLambda() : 0.0f;
}

float zjoltVehicleConstraintGetWheelLongitudinalLambda(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr ? w->GetLongitudinalLambda() : 0.0f;
}

float zjoltVehicleConstraintGetWheelLateralLambda(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index) {
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  return w != nullptr ? w->GetLateralLambda() : 0.0f;
}

void zjoltVehicleConstraintGetWheelLocalBasis(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out_forward, ZJoltVec3 *out_up, ZJoltVec3 *out_right) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  if (impl == nullptr || w == nullptr) {
    zjolt::WriteVec3(out_forward, JPH::Vec3::sZero());
    zjolt::WriteVec3(out_up, JPH::Vec3::sZero());
    zjolt::WriteVec3(out_right, JPH::Vec3::sZero());
    return;
  }
  JPH::Vec3 forward, up, right;
  impl->GetWheelLocalBasis(w, forward, up, right);
  zjolt::WriteVec3(out_forward, forward);
  zjolt::WriteVec3(out_up, up);
  zjolt::WriteVec3(out_right, right);
}

void zjoltVehicleConstraintGetWheelLocalTransform(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    const ZJoltVec3 *wheel_right, const ZJoltVec3 *wheel_up,
    ZJoltVec3 *out_position, ZJoltQuat *out_rotation) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  if (impl == nullptr || w == nullptr || wheel_right == nullptr || wheel_up == nullptr) {
    zjolt::WriteVec3(out_position, JPH::Vec3::sZero());
    zjolt::WriteQuat(out_rotation, JPH::Quat::sIdentity());
    return;
  }
  const JPH::Mat44 transform = impl->GetWheelLocalTransform(
      wheel_index, zjolt::ToJolt(*wheel_right), zjolt::ToJolt(*wheel_up));
  zjolt::WriteVec3(out_position, transform.GetTranslation());
  zjolt::WriteQuat(out_rotation, transform.GetQuaternion());
}

void zjoltVehicleConstraintGetWheelWorldTransform(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    const ZJoltVec3 *wheel_right, const ZJoltVec3 *wheel_up,
    ZJoltRVec3 *out_position, ZJoltQuat *out_rotation) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  const JPH::Wheel *w = WheelAt(constraint, wheel_index);
  if (impl == nullptr || w == nullptr || wheel_right == nullptr || wheel_up == nullptr) {
    zjolt::WriteRVec3(out_position, JPH::RVec3::sZero());
    zjolt::WriteQuat(out_rotation, JPH::Quat::sIdentity());
    return;
  }
  const JPH::RMat44 transform = impl->GetWheelWorldTransform(
      wheel_index, zjolt::ToJolt(*wheel_right), zjolt::ToJolt(*wheel_up));
  zjolt::WriteRVec3(out_position, transform.GetTranslation());
  zjolt::WriteQuat(out_rotation, transform.GetQuaternion());
}

//===----------------------------------------------------------------------===//
// Anti-roll bars — runtime
//===----------------------------------------------------------------------===//

uint32_t zjoltVehicleConstraintGetAntiRollBarCount(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr) return 0;
  return static_cast<uint32_t>(impl->GetAntiRollBars().size());
}

bool zjoltVehicleConstraintGetAntiRollBar(
    const ZJoltVehicleConstraint *constraint, uint32_t index,
    ZJoltVehicleAntiRollBarDesc *out) {
  if (out != nullptr) *out = ZJoltVehicleAntiRollBarDesc{};
  const JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr || out == nullptr) return false;
  const JPH::VehicleAntiRollBars &bars = impl->GetAntiRollBars();
  if (index >= bars.size()) return false;
  const JPH::VehicleAntiRollBar &bar = bars[index];
  out->left_wheel = bar.mLeftWheel;
  out->right_wheel = bar.mRightWheel;
  out->stiffness = bar.mStiffness;
  return true;
}

ZJoltResult zjoltVehicleConstraintSetAntiRollBarStiffness(
    ZJoltVehicleConstraint *constraint, uint32_t index, float stiffness) {
  ZJOLT_ENTER();
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint must not be null");
  }
  JPH::VehicleAntiRollBars &bars = impl->GetAntiRollBars();
  if (index >= bars.size()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "anti-roll bar index is past the bar count");
  }
  // VehicleConstraint::OnStep asserts this every step it runs.
  if (!(stiffness >= 0.0f)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "anti-roll bar stiffness must be >= 0");
  }
  bars[index].mStiffness = stiffness;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Differentials — runtime
//===----------------------------------------------------------------------===//

uint32_t zjoltVehicleConstraintGetDifferentialCount(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  if (wc == nullptr) return 0;
  return static_cast<uint32_t>(wc->GetDifferentials().size());
}

bool zjoltVehicleConstraintGetDifferential(
    const ZJoltVehicleConstraint *constraint, uint32_t index,
    ZJoltVehicleDifferentialDesc *out) {
  if (out != nullptr) *out = ZJoltVehicleDifferentialDesc{};
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  if (wc == nullptr || out == nullptr) return false;
  const JPH::WheeledVehicleController::Differentials &ds = wc->GetDifferentials();
  if (index >= ds.size()) return false;
  const JPH::VehicleDifferentialSettings &d = ds[index];
  out->left_wheel = d.mLeftWheel;
  out->right_wheel = d.mRightWheel;
  out->differential_ratio = d.mDifferentialRatio;
  out->left_right_split = d.mLeftRightSplit;
  out->limited_slip_ratio = d.mLimitedSlipRatio;
  out->engine_torque_ratio = d.mEngineTorqueRatio;
  return true;
}

ZJoltResult zjoltVehicleConstraintSetDifferential(
    ZJoltVehicleConstraint *constraint, uint32_t index,
    const ZJoltVehicleDifferentialDesc *desc) {
  ZJOLT_ENTER();
  if (!zjolt::Present(desc)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::WheeledVehicleController *wc = WheeledController(constraint);
  if (wc == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "differentials need a wheeled or motorcycle controller");
  }
  JPH::WheeledVehicleController::Differentials &ds = wc->GetDifferentials();
  if (index >= ds.size()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "differential index is past the differential count");
  }
  const uint32_t wheel_count =
      static_cast<uint32_t>(Impl(constraint)->GetWheels().size());
  if (!WheelIndexValid(desc->left_wheel, wheel_count, true) ||
      !WheelIndexValid(desc->right_wheel, wheel_count, true)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "differential left_wheel/right_wheel must be -1 or a real wheel index");
  }
  const ZJoltResult r = ValidateDifferentialDesc(*desc);
  if (r != ZJOLT_RESULT_OK) return r;

  JPH::VehicleDifferentialSettings &d = ds[index];
  d.mLeftWheel = desc->left_wheel;
  d.mRightWheel = desc->right_wheel;
  d.mDifferentialRatio = desc->differential_ratio;
  d.mLeftRightSplit = desc->left_right_split;
  d.mLimitedSlipRatio = desc->limited_slip_ratio;
  d.mEngineTorqueRatio = desc->engine_torque_ratio;
  return ZJOLT_RESULT_OK;
}

float zjoltVehicleConstraintGetDifferentialLimitedSlipRatio(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetDifferentialLimitedSlipRatio() : 0.0f;
}

ZJoltResult zjoltVehicleConstraintSetDifferentialLimitedSlipRatio(
    ZJoltVehicleConstraint *constraint, float ratio) {
  ZJOLT_ENTER();
  JPH::WheeledVehicleController *wc = WheeledController(constraint);
  if (wc == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "differentials need a wheeled or motorcycle controller");
  }
  // WheeledVehicleController's constructor asserts the same bound.
  if (!(ratio > 1.0f)) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "differential_limited_slip_ratio must be > 1");
  }
  wc->SetDifferentialLimitedSlipRatio(ratio);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Tracks
//===----------------------------------------------------------------------===//

float zjoltVehicleConstraintGetTrackAngularVelocity(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleTrackSide side) {
  const JPH::TrackedVehicleController *tc = TrackedController(constraint);
  if (tc == nullptr) return 0.0f;
  const int index = side == ZJOLT_VEHICLE_TRACK_SIDE_RIGHT ? 1 : 0;
  return tc->GetTracks()[index].mAngularVelocity;
}

ZJoltResult zjoltVehicleConstraintSetTrackAngularVelocity(
    ZJoltVehicleConstraint *constraint, ZJoltVehicleTrackSide side,
    float angular_velocity) {
  ZJOLT_ENTER();
  JPH::TrackedVehicleController *tc = TrackedController(constraint);
  if (tc == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "tracks need a tracked controller");
  }
  const int index = side == ZJOLT_VEHICLE_TRACK_SIDE_RIGHT ? 1 : 0;
  tc->GetTracks()[index].mAngularVelocity = angular_velocity;
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Engine and clutch — runtime
//===----------------------------------------------------------------------===//

ZJoltResult zjoltVehicleConstraintSetEngineRpm(
    ZJoltVehicleConstraint *constraint, float rpm) {
  ZJOLT_ENTER();
  JPH::VehicleEngine *e = EngineOf(constraint);
  if (e == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint has no engine");
  }
  // SetCurrentRPM clamps into [mMinRPM, mMaxRPM] itself.
  e->SetCurrentRPM(rpm);
  return ZJOLT_RESULT_OK;
}

float zjoltVehicleConstraintGetEngineTorque(
    const ZJoltVehicleConstraint *constraint, float acceleration) {
  const JPH::VehicleEngine *e = EngineOf(constraint);
  return e != nullptr ? e->GetTorque(acceleration) : 0.0f;
}

float zjoltVehicleConstraintGetWheelSpeedAtClutch(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  if (wc == nullptr) return 0.0f;
  // WheeledVehicleController::GetWheelSpeedAtClutch divides by the number of
  // driven wheels without checking it (WheeledVehicleController.cpp:217-231),
  // so a vehicle whose differentials name no wheels returns a NaN there. The
  // same count, done first.
  uint32_t driven = 0;
  for (const JPH::VehicleDifferentialSettings &d : wc->GetDifferentials()) {
    if (d.mLeftWheel >= 0) driven++;
    if (d.mRightWheel >= 0) driven++;
  }
  if (driven == 0) return 0.0f;
  return wc->GetWheelSpeedAtClutch();
}

ZJoltResult zjoltVehicleConstraintSetRpmMeter(
    ZJoltVehicleConstraint *constraint, const ZJoltVec3 *position, float size) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint, position)) return ZJOLT_RESULT_INVALID_ARGUMENT;
#ifdef JPH_DEBUG_RENDERER
  if (JPH::WheeledVehicleController *wc = WheeledController(constraint)) {
    wc->SetRPMMeter(zjolt::ToJolt(*position), size);
    return ZJOLT_RESULT_OK;
  }
  if (JPH::TrackedVehicleController *tc = TrackedController(constraint)) {
    tc->SetRPMMeter(zjolt::ToJolt(*position), size);
    return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                         "constraint has no controller with an RPM meter");
#else
  (void)size;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif
}

//===----------------------------------------------------------------------===//
// Motorcycle lean spring — runtime tuning
//===----------------------------------------------------------------------===//

bool zjoltVehicleConstraintGetMotorcycleLeanSpring(
    const ZJoltVehicleConstraint *constraint,
    ZJoltVehicleMotorcycleLeanSpring *out) {
  if (out != nullptr) *out = ZJoltVehicleMotorcycleLeanSpring{};
  const JPH::MotorcycleController *mc = MotorcycleControllerOf(constraint);
  if (mc == nullptr || out == nullptr) return false;
  out->spring_constant = mc->GetLeanSpringConstant();
  out->spring_damping = mc->GetLeanSpringDamping();
  out->spring_integration_coefficient =
      mc->GetLeanSpringIntegrationCoefficient();
  out->spring_integration_coefficient_decay =
      mc->GetLeanSpringIntegrationCoefficientDecay();
  out->lean_smoothing_factor = mc->GetLeanSmoothingFactor();
  return true;
}

ZJoltResult zjoltVehicleConstraintSetMotorcycleLeanSpring(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleMotorcycleLeanSpring *spring) {
  ZJOLT_ENTER();
  if (!zjolt::Present(spring)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::MotorcycleController *mc = MotorcycleControllerOf(constraint);
  if (mc == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "lean spring needs a motorcycle controller");
  }
  mc->SetLeanSpringConstant(spring->spring_constant);
  mc->SetLeanSpringDamping(spring->spring_damping);
  mc->SetLeanSpringIntegrationCoefficient(
      spring->spring_integration_coefficient);
  mc->SetLeanSpringIntegrationCoefficientDecay(
      spring->spring_integration_coefficient_decay);
  mc->SetLeanSmoothingFactor(spring->lean_smoothing_factor);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Wheel-ground collision filtering
//===----------------------------------------------------------------------===//

ZJoltResult zjoltVehicleConstraintSetWheelFilters(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleWheelFilters *filters) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  constraint->filters =
      filters != nullptr ? *filters : ZJoltVehicleWheelFilters{};
  ApplyFilters(constraint);
  return ZJOLT_RESULT_OK;
}

void zjoltVehicleConstraintGetWheelFilters(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleWheelFilters *out) {
  if (out == nullptr) return;
  *out = constraint != nullptr ? constraint->filters
                               : ZJoltVehicleWheelFilters{};
}

ZJoltResult zjoltVehicleConstraintSetCollisionTester(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleCollisionTesterDesc *desc) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint, desc)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "constraint must not be null");
  }
  JPH::VehicleCollisionTester *fresh = BuildCollisionTester(*desc);
  if (fresh == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  // The handle's reference first (0 -> 1), then the constraint's, which also
  // releases its reference to the old tester. The handle's own old reference
  // goes with the assignment above it, so the old one reaches zero and frees
  // exactly once.
  constraint->tester = fresh;
  impl->SetVehicleCollisionTester(fresh);
  constraint->collision_tester_kind = desc->kind;
  ApplyFilters(constraint);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Step callbacks
//===----------------------------------------------------------------------===//

ZJoltResult zjoltVehicleConstraintSetPreStepCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleStepCallback *callback) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  constraint->pre_step =
      callback != nullptr ? *callback : ZJoltVehicleStepCallback{};
  if (constraint->pre_step.on_step != nullptr) {
    ZJoltVehicleConstraint *handle = constraint;
    impl->SetPreStepCallback(
        [handle](JPH::VehicleConstraint &,
                 const JPH::PhysicsStepListenerContext &context) {
          InvokeStepCallback(handle->pre_step, handle, context);
        });
  } else {
    impl->SetPreStepCallback(JPH::VehicleConstraint::StepCallback());
  }
  return ZJOLT_RESULT_OK;
}

void zjoltVehicleConstraintGetPreStepCallback(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleStepCallback *out) {
  if (out == nullptr) return;
  *out = constraint != nullptr ? constraint->pre_step
                               : ZJoltVehicleStepCallback{};
}

ZJoltResult zjoltVehicleConstraintSetPostCollideCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleStepCallback *callback) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  constraint->post_collide =
      callback != nullptr ? *callback : ZJoltVehicleStepCallback{};
  if (constraint->post_collide.on_step != nullptr) {
    ZJoltVehicleConstraint *handle = constraint;
    impl->SetPostCollideCallback(
        [handle](JPH::VehicleConstraint &,
                 const JPH::PhysicsStepListenerContext &context) {
          InvokeStepCallback(handle->post_collide, handle, context);
        });
  } else {
    impl->SetPostCollideCallback(JPH::VehicleConstraint::StepCallback());
  }
  return ZJOLT_RESULT_OK;
}

void zjoltVehicleConstraintGetPostCollideCallback(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleStepCallback *out) {
  if (out == nullptr) return;
  *out = constraint != nullptr ? constraint->post_collide
                               : ZJoltVehicleStepCallback{};
}

ZJoltResult zjoltVehicleConstraintSetPostStepCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleStepCallback *callback) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  constraint->post_step =
      callback != nullptr ? *callback : ZJoltVehicleStepCallback{};
  if (constraint->post_step.on_step != nullptr) {
    ZJoltVehicleConstraint *handle = constraint;
    impl->SetPostStepCallback(
        [handle](JPH::VehicleConstraint &,
                 const JPH::PhysicsStepListenerContext &context) {
          InvokeStepCallback(handle->post_step, handle, context);
        });
  } else {
    impl->SetPostStepCallback(JPH::VehicleConstraint::StepCallback());
  }
  return ZJOLT_RESULT_OK;
}

void zjoltVehicleConstraintGetPostStepCallback(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleStepCallback *out) {
  if (out == nullptr) return;
  *out = constraint != nullptr ? constraint->post_step
                               : ZJoltVehicleStepCallback{};
}

//===----------------------------------------------------------------------===//
// Tire friction callbacks
//===----------------------------------------------------------------------===//

ZJoltResult zjoltVehicleConstraintSetCombineFrictionCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleCombineFrictionCallback *callback) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::VehicleConstraint *impl = Impl(constraint);
  if (impl == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  constraint->combine_friction =
      callback != nullptr ? *callback : ZJoltVehicleCombineFrictionCallback{};
  if (constraint->combine_friction.combine != nullptr) {
    ZJoltVehicleConstraint *handle = constraint;
    impl->SetCombineFriction([handle](JPH::uint wheel_index,
                                      float &io_longitudinal, float &io_lateral,
                                      const JPH::Body &body2,
                                      const JPH::SubShapeID &sub_shape_id) {
      const ZJoltVehicleCombineFrictionCallback &cb = handle->combine_friction;
      if (cb.combine == nullptr) return;
      cb.combine(cb.user, static_cast<uint32_t>(wheel_index),
                 zjolt::ToC(body2.GetID()), zjolt::ToC(sub_shape_id),
                 &io_longitudinal, &io_lateral);
    });
  } else {
    impl->SetCombineFriction(DefaultCombineFriction());
  }
  return ZJOLT_RESULT_OK;
}

void zjoltVehicleConstraintGetCombineFrictionCallback(
    const ZJoltVehicleConstraint *constraint,
    ZJoltVehicleCombineFrictionCallback *out) {
  if (out == nullptr) return;
  *out = constraint != nullptr ? constraint->combine_friction
                               : ZJoltVehicleCombineFrictionCallback{};
}

ZJoltResult zjoltVehicleConstraintSetTireMaxImpulseCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleTireMaxImpulseCallback *callback) {
  ZJOLT_ENTER();
  if (!zjolt::Present(constraint)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::WheeledVehicleController *wc = WheeledController(constraint);
  if (wc == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "tire impulses need a wheeled or motorcycle controller");
  }
  constraint->tire_max_impulse =
      callback != nullptr ? *callback : ZJoltVehicleTireMaxImpulseCallback{};
  if (constraint->tire_max_impulse.compute != nullptr) {
    ZJoltVehicleConstraint *handle = constraint;
    wc->SetTireMaxImpulseCallback(
        [handle](JPH::uint wheel_index, float &out_longitudinal_impulse,
                 float &out_lateral_impulse, float suspension_impulse,
                 float longitudinal_friction, float lateral_friction,
                 float longitudinal_slip, float lateral_slip, float delta_time) {
          const ZJoltVehicleTireMaxImpulseCallback &cb =
              handle->tire_max_impulse;
          if (cb.compute == nullptr) {
            out_longitudinal_impulse = longitudinal_friction * suspension_impulse;
            out_lateral_impulse = lateral_friction * suspension_impulse;
            return;
          }
          const ZJoltVehicleTireImpulseInputs inputs{
              suspension_impulse, longitudinal_friction, lateral_friction,
              longitudinal_slip,  lateral_slip,          delta_time};
          cb.compute(cb.user, static_cast<uint32_t>(wheel_index), &inputs,
                     &out_longitudinal_impulse, &out_lateral_impulse);
        });
  } else {
    wc->SetTireMaxImpulseCallback(DefaultTireMaxImpulse());
  }
  return ZJOLT_RESULT_OK;
}

void zjoltVehicleConstraintGetTireMaxImpulseCallback(
    const ZJoltVehicleConstraint *constraint,
    ZJoltVehicleTireMaxImpulseCallback *out) {
  if (out == nullptr) return;
  *out = constraint != nullptr ? constraint->tire_max_impulse
                               : ZJoltVehicleTireMaxImpulseCallback{};
}

//===----------------------------------------------------------------------===//
// Viewed as a plain constraint
//===----------------------------------------------------------------------===//

ZJoltConstraint *zjoltVehicleConstraintAsConstraint(
    const ZJoltVehicleConstraint *constraint) {
  if (constraint == nullptr) return nullptr;
  return zjolt::ToC(static_cast<JPH::Constraint *>(constraint->impl.GetPtr()));
}

}  // extern "C"
