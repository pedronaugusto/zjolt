//===----------------------------------------------------------------------===//
// zjolt — wheeled and tracked vehicles.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

#include <algorithm>

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

/// Shared by WheeledVehicleControllerSettings and MotorcycleControllerSettings
/// (the latter IS-A the former). Returns an error without touching `cs`'s
/// caller-visible state beyond what it already owns; the caller deletes `cs`
/// itself on failure, since ownership has not been handed anywhere yet.
ZJoltResult FillWheeledCommon(JPH::WheeledVehicleControllerSettings *cs,
                              const ZJoltVehicleConstraintDesc &desc) {
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
    JPH::VehicleDifferentialSettings jd;
    jd.mLeftWheel = d.left_wheel;
    jd.mRightWheel = d.right_wheel;
    jd.mDifferentialRatio = d.differential_ratio;
    jd.mLeftRightSplit = d.left_right_split;
    jd.mLimitedSlipRatio = d.limited_slip_ratio;
    jd.mEngineTorqueRatio = d.engine_torque_ratio;
    cs->mDifferentials.push_back(jd);
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
  // deletes itself through Jolt's allocator here.
  constraint->impl = nullptr;
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

float zjoltVehicleConstraintGetWheeledForwardInput(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetForwardInput() : 0.0f;
}

float zjoltVehicleConstraintGetWheeledRightInput(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetRightInput() : 0.0f;
}

float zjoltVehicleConstraintGetWheeledBrakeInput(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetBrakeInput() : 0.0f;
}

float zjoltVehicleConstraintGetWheeledHandBrakeInput(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetHandBrakeInput() : 0.0f;
}

float zjoltVehicleConstraintGetWheeledEngineRpm(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetEngine().GetCurrentRPM() : 0.0f;
}

int32_t zjoltVehicleConstraintGetWheeledCurrentGear(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr ? wc->GetTransmission().GetCurrentGear() : 0;
}

bool zjoltVehicleConstraintIsWheeledSwitchingGear(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::WheeledVehicleController *wc = WheeledController(constraint);
  return wc != nullptr && wc->GetTransmission().IsSwitchingGear();
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

float zjoltVehicleConstraintGetTrackedEngineRpm(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::TrackedVehicleController *tc = TrackedController(constraint);
  return tc != nullptr ? tc->GetEngine().GetCurrentRPM() : 0.0f;
}

int32_t zjoltVehicleConstraintGetTrackedCurrentGear(
    const ZJoltVehicleConstraint *constraint) {
  const JPH::TrackedVehicleController *tc = TrackedController(constraint);
  return tc != nullptr ? tc->GetTransmission().GetCurrentGear() : 0;
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

}  // extern "C"
