//===----------------------------------------------------------------------===//
// zjolt — wheeled and tracked vehicles.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// A vehicle is one constraint (`ZJoltVehicleConstraint`) tying a body to a set
// of wheels and one controller. The controller decides what "accelerate" and
// "steer" mean: wheeled, tracked, or (wheeled plus a lean spring) motorcycle.
// Which one a constraint has is fixed at creation and reported by
// `zjoltVehicleConstraintGetControllerKind`; calling a wheeled-only or
// tracked-only entry point against the other kind returns
// ZJOLT_RESULT_INVALID_ARGUMENT rather than reinterpreting memory.
//
// As everywhere in this ABI, engine/transmission/differential/wheel settings
// do not cross as Jolt objects: a descriptor is filled in, Jolt's own
// `*Settings` types are built on the stack inside zjoltVehicleConstraintCreate,
// and only the constraint handle comes back out.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_VEHICLE_H_
#define ZJOLT_VEHICLE_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// Kinds
//===----------------------------------------------------------------------===//

typedef enum ZJoltVehicleControllerKind {
  ZJOLT_VEHICLE_CONTROLLER_KIND_WHEELED = 0,
  ZJOLT_VEHICLE_CONTROLLER_KIND_TRACKED = 1,
  /// A WheeledVehicleController with a lean spring; every wheeled entry point
  /// below also accepts this kind.
  ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE = 2,
} ZJoltVehicleControllerKind;

typedef enum ZJoltVehicleCollisionTesterKind {
  /// Cheapest; treats each wheel as a single point.
  ZJOLT_VEHICLE_COLLISION_TESTER_KIND_RAY = 0,
  /// Tests a sphere of `radius` instead of a point, so a wheel cannot poke
  /// through a thin edge between the ray's sample points.
  ZJOLT_VEHICLE_COLLISION_TESTER_KIND_CAST_SPHERE = 1,
  /// Tests the wheel's own cylinder. Most accurate and most expensive.
  ZJOLT_VEHICLE_COLLISION_TESTER_KIND_CAST_CYLINDER = 2,
} ZJoltVehicleCollisionTesterKind;

typedef enum ZJoltVehicleTransmissionMode {
  /// Shifts on its own from engine RPM; see ZJoltVehicleTransmissionDesc.
  ZJOLT_VEHICLE_TRANSMISSION_MODE_AUTO = 0,
  /// Gear and clutch are whatever they were last set to; nothing in this ABI
  /// yet sets them for a manual transmission, so manual mode idles in neutral
  /// until a future entry point drives it.
  ZJOLT_VEHICLE_TRANSMISSION_MODE_MANUAL = 1,
} ZJoltVehicleTransmissionMode;

//===----------------------------------------------------------------------===//
// Curves
//
// A LinearCurve is a small set of (x, y) points sampled with linear
// interpolation. The cap below is generous for what a torque or friction
// curve actually needs (Jolt's own built-in defaults use three points) and
// keeps the descriptor flat.
//===----------------------------------------------------------------------===//

#define ZJOLT_VEHICLE_CURVE_MAX_POINTS 8

typedef struct ZJoltVehicleCurvePoint {
  float x;
  float y;
} ZJoltVehicleCurvePoint;

typedef struct ZJoltVehicleCurveDesc {
  ZJoltVehicleCurvePoint points[ZJOLT_VEHICLE_CURVE_MAX_POINTS];
  /// 0 means "leave Jolt's own built-in default curve for this field alone"
  /// rather than "an empty curve" — an empty LinearCurve samples as 0
  /// everywhere, which would silently zero every torque and friction value.
  uint32_t count;
} ZJoltVehicleCurveDesc;

//===----------------------------------------------------------------------===//
// Wheels
//
// One descriptor shape covers both controller kinds. The wheeled-only fields
// (inertia through max_hand_brake_torque) are ignored when the vehicle's
// controller is tracked, and the tracked-only friction pair is ignored when
// it is wheeled or motorcycle; zjoltVehicleWheelDescInit fills in both halves
// so a caller building one kind of vehicle need not know the other's fields
// exist.
//===----------------------------------------------------------------------===//

typedef struct ZJoltVehicleWheelDesc {
  // WheelSettings — every controller kind.
  ZJoltVec3 position;              ///< Suspension attachment, local to the body.
  ZJoltVec3 suspension_force_point;
  bool enable_suspension_force_point;
  /// Must be finite and non-zero; normalised before use. Should point down.
  ZJoltVec3 suspension_direction;
  /// Must be finite and non-zero; normalised before use. Should point up.
  ZJoltVec3 steering_axis;
  /// Must be finite and non-zero; normalised before use.
  ZJoltVec3 wheel_up;
  /// Must be finite and non-zero; normalised before use.
  ZJoltVec3 wheel_forward;
  float suspension_min_length;
  float suspension_max_length;
  float suspension_preload_length;
  /// Hz. Jolt's spring is built in FrequencyAndDamping mode; StiffnessAndDamping
  /// is not reachable through this ABI.
  float suspension_spring_frequency;
  float suspension_spring_damping;
  float radius;
  float width;

  // WheelSettingsWV — wheeled and motorcycle only.
  float inertia;
  float angular_damping;
  float max_steer_angle;
  /// Y: tire friction [0, ~1.2]. X: slip ratio. 0 points keeps Jolt's default.
  ZJoltVehicleCurveDesc longitudinal_friction;
  /// Y: tire friction. X: slip angle (degrees). 0 points keeps Jolt's default.
  ZJoltVehicleCurveDesc lateral_friction;
  float max_brake_torque;
  float max_hand_brake_torque;

  // WheelSettingsTV — tracked only. Not curves: a track's friction is one
  // coefficient, not a slip-dependent profile.
  float tracked_longitudinal_friction;
  float tracked_lateral_friction;
} ZJoltVehicleWheelDesc;

/// Fills every field with the wheeled and tracked defaults Jolt itself uses.
ZJOLT_API void zjoltVehicleWheelDescInit(ZJoltVehicleWheelDesc *desc);

//===----------------------------------------------------------------------===//
// Engine, transmission, differentials, anti-roll bars
//===----------------------------------------------------------------------===//

typedef struct ZJoltVehicleEngineDesc {
  float max_torque;      ///< Nm
  float min_rpm;         ///< Stall speed.
  float max_rpm;
  /// Y: fraction of max_torque. X: fraction of RPM range. 0 points keeps
  /// Jolt's default curve.
  ZJoltVehicleCurveDesc normalized_torque;
  float inertia;         ///< kg m^2
  float angular_damping;
} ZJoltVehicleEngineDesc;

ZJOLT_API void zjoltVehicleEngineDescInit(ZJoltVehicleEngineDesc *desc);

typedef struct ZJoltVehicleTransmissionDesc {
  ZJoltVehicleTransmissionMode mode;
  /// Borrowed for the duration of zjoltVehicleConstraintCreate only. NULL (or
  /// 0 count) keeps Jolt's built-in 5-speed ratios.
  const float *forward_gear_ratios;
  uint32_t forward_gear_ratio_count;
  /// Borrowed for the duration of zjoltVehicleConstraintCreate only. NULL (or
  /// 0 count) keeps Jolt's built-in reverse ratio.
  const float *reverse_gear_ratios;
  uint32_t reverse_gear_ratio_count;
  float switch_time;            ///< Seconds; auto mode only.
  float clutch_release_time;    ///< Seconds; auto mode only.
  float switch_latency;         ///< Seconds; auto mode only.
  float shift_up_rpm;           ///< Auto mode only.
  float shift_down_rpm;         ///< Auto mode only.
  float clutch_strength;
} ZJoltVehicleTransmissionDesc;

ZJOLT_API void zjoltVehicleTransmissionDescInit(ZJoltVehicleTransmissionDesc *desc);

typedef struct ZJoltVehicleDifferentialDesc {
  /// Index into the vehicle's wheel array, or -1 for none.
  int32_t left_wheel;
  int32_t right_wheel;
  float differential_ratio;
  /// [0, 1]; 0 = all torque left, 1 = all torque right.
  float left_right_split;
  /// > 1, or FLT_MAX for an open differential.
  float limited_slip_ratio;
  /// [0, 1]; the sum across every differential on a vehicle should be 1.
  float engine_torque_ratio;
} ZJoltVehicleDifferentialDesc;

ZJOLT_API void zjoltVehicleDifferentialDescInit(ZJoltVehicleDifferentialDesc *desc);

typedef struct ZJoltVehicleAntiRollBarDesc {
  /// Index into the vehicle's wheel array. Unlike a differential, neither side
  /// is optional.
  int32_t left_wheel;
  int32_t right_wheel;
  /// N/m. 0 disables this bar without removing it.
  float stiffness;
} ZJoltVehicleAntiRollBarDesc;

ZJOLT_API void zjoltVehicleAntiRollBarDescInit(ZJoltVehicleAntiRollBarDesc *desc);

//===----------------------------------------------------------------------===//
// Wheel-ground collision testing
//===----------------------------------------------------------------------===//

typedef struct ZJoltVehicleCollisionTesterDesc {
  ZJoltVehicleCollisionTesterKind kind;
  /// Used only when the constraint's PhysicsSystem has no per-wheel filters
  /// overriding it; every wheel of one vehicle shares one tester.
  ZJoltObjectLayer object_layer;
  /// World-space up, used to reject near-vertical surfaces. RAY and
  /// CAST_SPHERE only.
  ZJoltVec3 up;
  /// Radians. Surfaces steeper than this do not count as ground. RAY and
  /// CAST_SPHERE only.
  float max_slope_angle;
  /// Sphere radius, in metres. CAST_SPHERE only.
  float radius;
  /// [0, 1], fraction of the wheel's half-width or radius (whichever is
  /// smaller). CAST_CYLINDER only.
  float convex_radius_fraction;
} ZJoltVehicleCollisionTesterDesc;

/// Defaults to CAST_SPHERE would need a radius the caller has not supplied
/// yet, so this defaults to RAY, the only kind with no required field beyond
/// object_layer.
ZJOLT_API void zjoltVehicleCollisionTesterDescInit(
    ZJoltVehicleCollisionTesterDesc *desc);

//===----------------------------------------------------------------------===//
// Motorcycle lean controller — ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE only.
//===----------------------------------------------------------------------===//

typedef struct ZJoltVehicleMotorcycleDesc {
  float max_lean_angle;                          ///< Radians.
  float lean_spring_constant;
  float lean_spring_damping;
  float lean_spring_integration_coefficient;
  float lean_spring_integration_coefficient_decay;
  /// [0, 1]; 0 = no smoothing, 1 = the lean angle never changes.
  float lean_smoothing_factor;
} ZJoltVehicleMotorcycleDesc;

ZJOLT_API void zjoltVehicleMotorcycleDescInit(ZJoltVehicleMotorcycleDesc *desc);

//===----------------------------------------------------------------------===//
// The constraint
//===----------------------------------------------------------------------===//

typedef struct ZJoltVehicleConstraint ZJoltVehicleConstraint;

typedef struct ZJoltVehicleConstraintDesc {
  /// Local-space up/forward for the vehicle body. Must be finite and
  /// non-zero; normalised before use.
  ZJoltVec3 up;
  ZJoltVec3 forward;
  /// Radians; pi disables the limit. Keeps the vehicle from tipping over.
  float max_pitch_roll_angle;

  /// Borrowed for the duration of this call only. Required: at least one
  /// wheel.
  const ZJoltVehicleWheelDesc *wheels;
  uint32_t wheel_count;

  /// Borrowed for the duration of this call only. May be NULL (0 count).
  const ZJoltVehicleAntiRollBarDesc *anti_roll_bars;
  uint32_t anti_roll_bar_count;

  ZJoltVehicleControllerKind controller_kind;
  ZJoltVehicleEngineDesc engine;
  ZJoltVehicleTransmissionDesc transmission;

  /// Wheeled and motorcycle only. Borrowed for the duration of this call.
  const ZJoltVehicleDifferentialDesc *differentials;
  uint32_t differential_count;
  /// Wheeled and motorcycle only. > 1, or FLT_MAX for open.
  float differential_limited_slip_ratio;

  /// Tracked only. Indices into `wheels`, borrowed for the duration of this
  /// call. Each side needs at least one wheel, including its driven wheel.
  const uint32_t *left_track_wheels;
  uint32_t left_track_wheel_count;
  uint32_t left_track_driven_wheel;
  const uint32_t *right_track_wheels;
  uint32_t right_track_wheel_count;
  uint32_t right_track_driven_wheel;
  float track_inertia;
  float track_angular_damping;
  float track_max_brake_torque;
  float track_differential_ratio;

  /// Motorcycle only (controller_kind == ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE).
  ZJoltVehicleMotorcycleDesc motorcycle;

  ZJoltVehicleCollisionTesterDesc collision_tester;
} ZJoltVehicleConstraintDesc;

/// Fills in the top-level fields (up/forward/pitch-roll limit) plus every
/// nested descriptor via its own *DescInit. wheels/anti_roll_bars/
/// differentials/track pointers are left NULL with 0 counts: this describes
/// no wheels, which zjoltVehicleConstraintCreate will refuse, by design —
/// there is no default wheel to invent.
ZJOLT_API void zjoltVehicleConstraintDescInit(ZJoltVehicleConstraintDesc *desc);

/// Builds the constraint, attaches it to `body`, and both adds it to `system`
/// as a constraint and registers it as a physics step listener — a vehicle
/// constraint that is not stepped never moves its wheels.
///
/// `body` must already be added to `system`. The constraint keeps a raw
/// pointer to that body for its own lifetime; destroying the body while the
/// constraint is still alive is the caller's mistake, not this ABI's to
/// detect.
ZJOLT_API ZJoltResult zjoltVehicleConstraintCreate(
    ZJoltPhysicsSystem *system, ZJoltBodyId body,
    const ZJoltVehicleConstraintDesc *desc, ZJoltVehicleConstraint **out);

/// Removes the constraint and its step listener from its system, then frees
/// it. Accepts NULL; safe to call once.
ZJOLT_API void zjoltVehicleConstraintDestroy(ZJoltVehicleConstraint *constraint);

//===----------------------------------------------------------------------===//
// Vehicle-level state
//===----------------------------------------------------------------------===//

ZJOLT_API ZJoltVehicleControllerKind zjoltVehicleConstraintGetControllerKind(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API ZJoltVehicleCollisionTesterKind
zjoltVehicleConstraintGetCollisionTesterKind(
    const ZJoltVehicleConstraint *constraint);

ZJOLT_API ZJoltBodyId
zjoltVehicleConstraintGetBodyId(const ZJoltVehicleConstraint *constraint);

/// True when the underlying body is in the broad phase and the constraint has
/// not been idled out. False (not an error) for a NULL constraint.
ZJOLT_API bool zjoltVehicleConstraintIsActive(
    const ZJoltVehicleConstraint *constraint);

ZJOLT_API float zjoltVehicleConstraintGetMaxPitchRollAngle(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API void zjoltVehicleConstraintSetMaxPitchRollAngle(
    ZJoltVehicleConstraint *constraint, float max_pitch_roll_angle);

ZJOLT_API void zjoltVehicleConstraintGetLocalUp(
    const ZJoltVehicleConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API void zjoltVehicleConstraintGetLocalForward(
    const ZJoltVehicleConstraint *constraint, ZJoltVec3 *out);
/// World-space up used this step to judge pitch/roll; tracks inverted gravity.
ZJOLT_API void zjoltVehicleConstraintGetWorldUp(
    const ZJoltVehicleConstraint *constraint, ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Wheels — runtime state
//
// Every accessor below takes a wheel index and returns a zero/false/default
// value for an index at or past the wheel count, the same as a NULL
// constraint. Wheel settings (radius, suspension range, ...) are creation-time
// only, via ZJoltVehicleWheelDesc; there is no live getter for them.
//===----------------------------------------------------------------------===//

ZJOLT_API uint32_t
zjoltVehicleConstraintGetWheelCount(const ZJoltVehicleConstraint *constraint);

ZJOLT_API float zjoltVehicleConstraintGetWheelRotationAngle(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
ZJOLT_API void zjoltVehicleConstraintSetWheelRotationAngle(
    ZJoltVehicleConstraint *constraint, uint32_t wheel_index, float angle);

ZJOLT_API float zjoltVehicleConstraintGetWheelSteerAngle(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
ZJOLT_API void zjoltVehicleConstraintSetWheelSteerAngle(
    ZJoltVehicleConstraint *constraint, uint32_t wheel_index, float angle);

ZJOLT_API float zjoltVehicleConstraintGetWheelAngularVelocity(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
ZJOLT_API void zjoltVehicleConstraintSetWheelAngularVelocity(
    ZJoltVehicleConstraint *constraint, uint32_t wheel_index, float velocity);

ZJOLT_API float zjoltVehicleConstraintGetWheelSuspensionLength(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);

/// The five contact getters below read Jolt state that is only meaningful
/// while HasWheelContact is true; each writes all-zero (or, for the body id,
/// ZJOLT_BODY_ID_INVALID) and does nothing else when it is not.
ZJOLT_API bool zjoltVehicleConstraintHasWheelContact(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
ZJOLT_API ZJoltBodyId zjoltVehicleConstraintGetWheelContactBodyId(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
ZJOLT_API void zjoltVehicleConstraintGetWheelContactPosition(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltRVec3 *out);
ZJOLT_API void zjoltVehicleConstraintGetWheelContactNormal(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out);
ZJOLT_API void zjoltVehicleConstraintGetWheelContactLongitudinal(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out);
ZJOLT_API void zjoltVehicleConstraintGetWheelContactLateral(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out);
ZJOLT_API void zjoltVehicleConstraintGetWheelContactPointVelocity(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Wheeled and motorcycle controller — driver input and readback
//
// Every entry point here also accepts ZJOLT_VEHICLE_CONTROLLER_KIND_MOTORCYCLE,
// since a motorcycle IS a wheeled vehicle plus a lean spring. Called against a
// tracked constraint, each returns ZJOLT_RESULT_INVALID_ARGUMENT (or, for the
// infallible readbacks, the same default a NULL constraint would give).
//===----------------------------------------------------------------------===//

/// @param forward [-1, 1] auto, [0, 1] manual: desired direction and throttle.
/// @param right [-1, 1]: desired steering angle, 1 = right.
/// @param brake [0, 1].
/// @param hand_brake [0, 1]; usually only the rear wheels honour it.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetWheeledDriverInput(
    ZJoltVehicleConstraint *constraint, float forward, float right,
    float brake, float hand_brake);

ZJOLT_API float zjoltVehicleConstraintGetWheeledForwardInput(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API float zjoltVehicleConstraintGetWheeledRightInput(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API float zjoltVehicleConstraintGetWheeledBrakeInput(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API float zjoltVehicleConstraintGetWheeledHandBrakeInput(
    const ZJoltVehicleConstraint *constraint);

ZJOLT_API float
zjoltVehicleConstraintGetWheeledEngineRpm(const ZJoltVehicleConstraint *constraint);
/// -1 = reverse, 0 = neutral, 1 = 1st gear, etc.
ZJOLT_API int32_t zjoltVehicleConstraintGetWheeledCurrentGear(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API bool zjoltVehicleConstraintIsWheeledSwitchingGear(
    const ZJoltVehicleConstraint *constraint);

//===----------------------------------------------------------------------===//
// Tracked controller — driver input and readback
//===----------------------------------------------------------------------===//

/// @param forward [-1, 1] auto, [0, 1] manual.
/// @param left_ratio Nonzero multiplier on the left track's rate; steers.
/// @param right_ratio Nonzero multiplier on the right track's rate; steers.
/// @param brake [0, 1].
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetTrackedDriverInput(
    ZJoltVehicleConstraint *constraint, float forward, float left_ratio,
    float right_ratio, float brake);

ZJOLT_API float
zjoltVehicleConstraintGetTrackedEngineRpm(const ZJoltVehicleConstraint *constraint);
ZJOLT_API int32_t zjoltVehicleConstraintGetTrackedCurrentGear(
    const ZJoltVehicleConstraint *constraint);

//===----------------------------------------------------------------------===//
// Motorcycle controller — lean spring
//===----------------------------------------------------------------------===//

/// Disabling lets the motorcycle fall over instead of self-balancing.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetMotorcycleLeanControllerEnabled(
    ZJoltVehicleConstraint *constraint, bool enabled);
ZJOLT_API bool zjoltVehicleConstraintIsMotorcycleLeanControllerEnabled(
    const ZJoltVehicleConstraint *constraint);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_VEHICLE_H_
