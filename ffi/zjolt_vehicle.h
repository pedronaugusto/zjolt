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

#include "zjolt_constraint.h"
#include "zjolt_core.h"
// For ZJoltBroadPhaseLayerFilter / ZJoltObjectLayerFilter / ZJoltBodyFilter,
// which ZJoltVehicleWheelFilters below reuses rather than respelling: a wheel's
// ground test is a ray or shape cast like any other, so it filters like one.
#include "zjolt_query.h"

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
  /// Gear and clutch are whatever they were last set to, through
  /// zjoltVehicleConstraintSetGear — auto mode's shift points and timers are
  /// not consulted at all.
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
// Driver input and drivetrain readback — every controller kind
//
// forward/brake, engine RPM, current gear, clutch friction and gear ratio
// mean the same thing whichever controller a constraint has (a
// WheeledVehicleController and a TrackedVehicleController each carry their
// own VehicleEngine and VehicleTransmission), so each is one entry point
// that switches on zjoltVehicleConstraintGetControllerKind internally rather
// than two spellings a caller could pick the wrong one of. Called against a
// constraint with no driveable controller (there is none today, but the
// check is here for whatever is added next) each returns
// ZJOLT_RESULT_INVALID_ARGUMENT, or the same default a NULL constraint would
// give for the infallible readbacks.
//===----------------------------------------------------------------------===//

ZJOLT_API float
zjoltVehicleConstraintGetForwardInput(const ZJoltVehicleConstraint *constraint);
ZJOLT_API float
zjoltVehicleConstraintGetBrakeInput(const ZJoltVehicleConstraint *constraint);

ZJOLT_API float
zjoltVehicleConstraintGetEngineRpm(const ZJoltVehicleConstraint *constraint);
/// -1 = reverse, 0 = neutral, 1 = 1st gear, etc.
ZJOLT_API int32_t
zjoltVehicleConstraintGetCurrentGear(const ZJoltVehicleConstraint *constraint);
ZJOLT_API bool
zjoltVehicleConstraintIsSwitchingGear(const ZJoltVehicleConstraint *constraint);
ZJOLT_API float
zjoltVehicleConstraintGetClutchFriction(const ZJoltVehicleConstraint *constraint);
/// Current gear ratio times the differential ratio (transmission's
/// VehicleTransmission::GetCurrentRatio); 0 in neutral.
ZJOLT_API float
zjoltVehicleConstraintGetGearRatio(const ZJoltVehicleConstraint *constraint);

/// Drives a manual transmission (ZJOLT_VEHICLE_TRANSMISSION_MODE_MANUAL):
/// sets the current gear and the clutch's friction fraction directly,
/// bypassing auto mode's shift points and timers entirely. Also accepted
/// (harmlessly overwritten on the next auto shift) against an auto
/// transmission.
/// @param gear -1 = reverse, 0 = neutral, 1 = 1st gear, etc.
/// @param clutch_friction [0, 1]: 0 = clutch fully disengaged (no torque
/// reaches the wheels), 1 = fully engaged.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetGear(
    ZJoltVehicleConstraint *constraint, int32_t gear, float clutch_friction);

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

ZJOLT_API float zjoltVehicleConstraintGetWheeledRightInput(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API float zjoltVehicleConstraintGetWheeledHandBrakeInput(
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

ZJOLT_API float zjoltVehicleConstraintGetTrackedLeftRatio(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API float zjoltVehicleConstraintGetTrackedRightRatio(
    const ZJoltVehicleConstraint *constraint);

//===----------------------------------------------------------------------===//
// Motorcycle controller — lean spring
//===----------------------------------------------------------------------===//

/// Disabling lets the motorcycle fall over instead of self-balancing.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetMotorcycleLeanControllerEnabled(
    ZJoltVehicleConstraint *constraint, bool enabled);
ZJOLT_API bool zjoltVehicleConstraintIsMotorcycleLeanControllerEnabled(
    const ZJoltVehicleConstraint *constraint);

/// Caps how far the steering angle can go as the motorcycle leans, so it
/// cannot steer into itself. Disabling is for a bike that should behave like
/// a car with two wheels close together rather than lean into its turns.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetMotorcycleLeanSteeringLimitEnabled(
    ZJoltVehicleConstraint *constraint, bool enabled);
ZJOLT_API bool zjoltVehicleConstraintIsMotorcycleLeanSteeringLimitEnabled(
    const ZJoltVehicleConstraint *constraint);

/// Distance between the front and rear wheel's ground contact, used by the
/// lean controller's own geometry; 0 against a non-motorcycle constraint.
ZJOLT_API float zjoltVehicleConstraintGetMotorcycleWheelBase(
    const ZJoltVehicleConstraint *constraint);

//===----------------------------------------------------------------------===//
// Gravity override
//
// Replaces PhysicsSystem::GetGravity() for this vehicle only — for a stunt,
// a low-gravity moon buggy, or any per-vehicle deviation from the world's
// own gravity. Infallible: a NULL constraint is a no-op / returns a zero
// vector / reports "not overridden", the same as every other getter here.
//===----------------------------------------------------------------------===//

ZJOLT_API void zjoltVehicleConstraintOverrideGravity(
    ZJoltVehicleConstraint *constraint, const ZJoltVec3 *gravity);
/// Also restores the vehicle body's gravity factor to 1.
ZJOLT_API void zjoltVehicleConstraintResetGravityOverride(
    ZJoltVehicleConstraint *constraint);
ZJOLT_API bool zjoltVehicleConstraintIsGravityOverridden(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API void zjoltVehicleConstraintGetGravityOverride(
    const ZJoltVehicleConstraint *constraint, ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Wheel-ground collision test frequency
//
// Skipping steps between wheel-ground collision tests is a cheap way to
// spend less time on vehicles that are far from the camera or barely
// moving; the wheels keep their last contact between tests. Defaults come
// from VehicleConstraintSettings's own construction and are not part of
// ZJoltVehicleConstraintDesc, so they start at Jolt's built-in values (1 for
// both) until set here.
//===----------------------------------------------------------------------===//

ZJOLT_API uint32_t zjoltVehicleConstraintGetNumStepsBetweenCollisionTestActive(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API void zjoltVehicleConstraintSetNumStepsBetweenCollisionTestActive(
    ZJoltVehicleConstraint *constraint, uint32_t steps);
ZJOLT_API uint32_t
zjoltVehicleConstraintGetNumStepsBetweenCollisionTestInactive(
    const ZJoltVehicleConstraint *constraint);
ZJOLT_API void zjoltVehicleConstraintSetNumStepsBetweenCollisionTestInactive(
    ZJoltVehicleConstraint *constraint, uint32_t steps);

//===----------------------------------------------------------------------===//
// Wheel force and pose readback
//
// The lambdas are the constraint solver's own accumulated impulses for the
// wheel this step, divided by delta time to read as a force: how hard the
// suspension is pushing, and how much grip the tire is actually using in
// each direction. All four (sub-shape id included) share the contact
// getters' convention above: zero, or ZJOLT_SUB_SHAPE_ID_EMPTY, without
// contact or past the wheel count.
//===----------------------------------------------------------------------===//

ZJOLT_API ZJoltSubShapeId zjoltVehicleConstraintGetWheelContactSubShapeId(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
/// True when the suspension has bottomed out against its hard limit at
/// suspension_min_length this step.
ZJOLT_API bool zjoltVehicleConstraintHasWheelHitHardPoint(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
ZJOLT_API float zjoltVehicleConstraintGetWheelSuspensionLambda(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
ZJOLT_API float zjoltVehicleConstraintGetWheelLongitudinalLambda(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);
ZJOLT_API float zjoltVehicleConstraintGetWheelLateralLambda(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index);

/// The wheel's forward/up/right axes in the vehicle body's local space,
/// after steering — feed `out_up`/`out_right` into the two transforms below
/// to place a wheel mesh whose own local axes do not match theirs. All-zero
/// past the wheel count.
ZJOLT_API void zjoltVehicleConstraintGetWheelLocalBasis(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    ZJoltVec3 *out_forward, ZJoltVec3 *out_up, ZJoltVec3 *out_right);

/// The wheel's pose in the vehicle body's local space — suspension travel,
/// steering and spin all folded in. `wheel_right`/`wheel_up` are the wheel
/// mesh's own local axes (typically the `out_right`/`out_up` a prior
/// zjoltVehicleConstraintGetWheelLocalBasis call gave you); identity/zero
/// past the wheel count.
ZJOLT_API void zjoltVehicleConstraintGetWheelLocalTransform(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    const ZJoltVec3 *wheel_right, const ZJoltVec3 *wheel_up,
    ZJoltVec3 *out_position, ZJoltQuat *out_rotation);

/// The wheel's pose in world space: the vehicle body's own world transform
/// composed with zjoltVehicleConstraintGetWheelLocalTransform. This is what
/// a wheel mesh should be drawn at each frame.
ZJOLT_API void zjoltVehicleConstraintGetWheelWorldTransform(
    const ZJoltVehicleConstraint *constraint, uint32_t wheel_index,
    const ZJoltVec3 *wheel_right, const ZJoltVec3 *wheel_up,
    ZJoltRVec3 *out_position, ZJoltQuat *out_rotation);

//===----------------------------------------------------------------------===//
// Anti-roll bars — runtime
//
// A bar couples the two wheels it names: the further apart their suspensions
// are compressed, the harder it pushes to even them out, which is what stops
// a car rolling onto its door handles in a corner. `stiffness` is the only
// field worth changing after creation — moving a bar to different wheels is
// rebuilding the vehicle, not tuning it — so that is the only setter here.
//===----------------------------------------------------------------------===//

ZJOLT_API uint32_t zjoltVehicleConstraintGetAntiRollBarCount(
    const ZJoltVehicleConstraint *constraint);

/// Fills `out` with the bar at `index` and returns true; returns false and
/// leaves `out` all-zero for a NULL constraint or an index at or past the
/// count. False rather than a zeroed struct is the answer because a zeroed
/// ZJoltVehicleAntiRollBarDesc is a legitimate bar (wheel 0 to wheel 0, no
/// stiffness) and would not read as "no such bar".
ZJOLT_API bool zjoltVehicleConstraintGetAntiRollBar(
    const ZJoltVehicleConstraint *constraint, uint32_t index,
    ZJoltVehicleAntiRollBarDesc *out);

/// N/m; 0 disables the bar without removing it. Negative is refused —
/// VehicleConstraint::OnStep asserts stiffness >= 0, and a negative bar would
/// push the two suspensions further apart rather than together.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetAntiRollBarStiffness(
    ZJoltVehicleConstraint *constraint, uint32_t index, float stiffness);

//===----------------------------------------------------------------------===//
// Differentials — runtime; wheeled and motorcycle only
//
// Creation fixes how many differentials a vehicle has and which wheels each
// one drives; everything else about them is live. Retuning takes effect on
// the next step, so a mid-corner change is applied to that corner.
//===----------------------------------------------------------------------===//

/// 0 against a tracked constraint, which has tracks instead.
ZJOLT_API uint32_t zjoltVehicleConstraintGetDifferentialCount(
    const ZJoltVehicleConstraint *constraint);

/// As zjoltVehicleConstraintGetAntiRollBar: false and an all-zero `out` for a
/// NULL/tracked constraint or an out-of-range index.
ZJOLT_API bool zjoltVehicleConstraintGetDifferential(
    const ZJoltVehicleConstraint *constraint, uint32_t index,
    ZJoltVehicleDifferentialDesc *out);

/// Replaces the differential at `index` whole, so read it first and change
/// the field you mean. `left_wheel`/`right_wheel` are validated against the
/// vehicle's wheel count (-1 = no wheel on that side), and the same numeric
/// ranges zjoltVehicleConstraintCreate enforces apply: differential_ratio > 0,
/// left_right_split in [0, 1], engine_torque_ratio >= 0, limited_slip_ratio
/// > 1. Nothing is applied when any of them fails.
///
/// The one rule that cannot be checked here: `engine_torque_ratio` must sum
/// to 1 ACROSS every differential the vehicle has. That is a property of the
/// set, not of one member, so moving torque from one axle to another needs
/// two calls and is momentarily out of balance in between — which is fine, as
/// long as the vehicle is not stepped until both have landed. Jolt asserts
/// the sum at the top of the next step in an asserts-on build, and quietly
/// scales the engine's output oddly in one without.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetDifferential(
    ZJoltVehicleConstraint *constraint, uint32_t index,
    const ZJoltVehicleDifferentialDesc *desc);

/// The limit on how far apart two DIFFERENTIALS' average wheel speeds may
/// drift, as opposed to the per-differential `limited_slip_ratio` between one
/// differential's own two wheels. 0 against a tracked or NULL constraint.
ZJOLT_API float zjoltVehicleConstraintGetDifferentialLimitedSlipRatio(
    const ZJoltVehicleConstraint *constraint);
/// Must be > 1; FLT_MAX turns the coupling off.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetDifferentialLimitedSlipRatio(
    ZJoltVehicleConstraint *constraint, float ratio);

//===----------------------------------------------------------------------===//
// Tracks — tracked only
//===----------------------------------------------------------------------===//

typedef enum ZJoltVehicleTrackSide {
  ZJOLT_VEHICLE_TRACK_SIDE_LEFT = 0,
  ZJOLT_VEHICLE_TRACK_SIDE_RIGHT = 1,
} ZJoltVehicleTrackSide;

/// Angular velocity (rad/s) of the track's driven wheel, which is what sets
/// the speed of the whole track — the number a track mesh or a scrolling
/// tread texture should be driven from. Every wheel on the track follows it:
/// each reports this rate scaled by the driven wheel's radius over its own.
/// 0 for a non-tracked or NULL constraint.
ZJOLT_API float zjoltVehicleConstraintGetTrackAngularVelocity(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleTrackSide side);
/// Seeds the track rather than holding it there: the controller integrates
/// from whatever is in it each step, applying damping, then engine torque and
/// brakes. For restoring a track or launching one already turning; drive one
/// with zjoltVehicleConstraintSetTrackedDriverInput.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetTrackAngularVelocity(
    ZJoltVehicleConstraint *constraint, ZJoltVehicleTrackSide side,
    float angular_velocity);

//===----------------------------------------------------------------------===//
// Engine and clutch — runtime
//===----------------------------------------------------------------------===//

/// Clamped into the engine's own [min_rpm, max_rpm] rather than refused, the
/// same clamp Jolt applies after every torque and damping integration step.
/// For launching with the engine already revving, or putting a drivetrain
/// back where it was after a teleport; ordinary driving moves RPM by itself.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetEngineRpm(
    ZJoltVehicleConstraint *constraint, float rpm);

/// The torque (Nm) the engine would make at its CURRENT rpm for a throttle
/// fraction of `acceleration` — the torque curve sampled where the engine
/// actually is, which is what a dyno readout or an audio blend wants. Pass 1
/// for full throttle. 0 for a constraint with no engine.
ZJOLT_API float zjoltVehicleConstraintGetEngineTorque(
    const ZJoltVehicleConstraint *constraint, float acceleration);

/// Average speed of the driven wheels measured at the clutch, in the same RPM
/// units as zjoltVehicleConstraintGetEngineRpm — subtract that and you have
/// clutch slip. 0 (not a NaN) when no differential names a wheel, which is
/// the division Jolt itself does not guard.
ZJOLT_API float zjoltVehicleConstraintGetWheelSpeedAtClutch(
    const ZJoltVehicleConstraint *constraint);

/// Where the debug RPM meter is drawn, in the vehicle body's local space, and
/// how big. Only ever visible through zjoltPhysicsSystemDrawConstraints;
/// returns ZJOLT_RESULT_UNSUPPORTED when this library was built without
/// Jolt's debug renderer, as everything else debug-draw-shaped does.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetRpmMeter(
    ZJoltVehicleConstraint *constraint, const ZJoltVec3 *position, float size);

//===----------------------------------------------------------------------===//
// Motorcycle lean spring — runtime tuning
//
// The creation-time ZJoltVehicleMotorcycleDesc's max_lean_angle is absent
// here on purpose: MotorcycleController has no setter for it, so a runtime
// field that silently did nothing would be worse than not offering one.
//===----------------------------------------------------------------------===//

typedef struct ZJoltVehicleMotorcycleLeanSpring {
  float spring_constant;
  float spring_damping;
  /// Makes the lean spring a PID rather than a PD controller: an extra force
  /// proportional to the accumulated lean error. 0 leaves it a PD.
  float spring_integration_coefficient;
  /// How fast that accumulated error decays while the wheels are airborne.
  float spring_integration_coefficient_decay;
  /// [0, 1]; 0 = no smoothing, 1 = the lean angle never changes. Frame-rate
  /// dependent — Jolt blends previous and current by this fraction per step,
  /// so a value tuned at 60 Hz leans differently at 30.
  float lean_smoothing_factor;
} ZJoltVehicleMotorcycleLeanSpring;

/// False, with `out` all-zero, against a non-motorcycle constraint.
ZJOLT_API bool zjoltVehicleConstraintGetMotorcycleLeanSpring(
    const ZJoltVehicleConstraint *constraint,
    ZJoltVehicleMotorcycleLeanSpring *out);
/// Applies every field, so read it first and change the one you mean.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetMotorcycleLeanSpring(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleMotorcycleLeanSpring *spring);

//===----------------------------------------------------------------------===//
// Wheel-ground collision filtering
//
// The tester's `object_layer` (ZJoltVehicleCollisionTesterDesc) is what the
// wheel casts use by default. Each filter here OVERRIDES that default for its
// own level rather than narrowing it: a broad-phase-layer filter replaces the
// layer pair check entirely, and a member whose function pointer is NULL is
// not installed at all, so the default stays.
//
// One thing Jolt gets subtly wrong for a caller and this ABI does not: in
// Jolt, installing a body filter REPLACES the built-in "ignore the vehicle's
// own body" filter, so a filter that says yes to everything makes the wheels
// collide with the chassis they hang off and the vehicle tears itself apart.
// Here the vehicle's own body is rejected before `body.should_collide` is
// consulted, so a filter can only ever narrow the set — which is what every
// caller of this wanted anyway.
//
// Filters are consulted from inside PhysicsSystem::Update, on whichever job
// thread is stepping this vehicle. See the step callbacks below for what that
// means; the same rules apply.
//===----------------------------------------------------------------------===//

typedef struct ZJoltVehicleWheelFilters {
  ZJoltBroadPhaseLayerFilter broad_phase_layer;
  ZJoltObjectLayerFilter object_layer;
  ZJoltBodyFilter body;
} ZJoltVehicleWheelFilters;

/// NULL clears every filter, restoring the tester's plain object-layer test
/// and Jolt's own self-exclusion. The struct is copied; the `user` pointers
/// inside it are not, and must outlive the vehicle or be cleared first.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetWheelFilters(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleWheelFilters *filters);
/// Exactly what was last set, or all-zero if nothing was.
ZJOLT_API void zjoltVehicleConstraintGetWheelFilters(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleWheelFilters *out);

/// Replaces the wheel-ground collision tester, which is otherwise fixed at
/// creation — for dropping a distant vehicle from a cylinder cast down to a
/// ray and back as it approaches, alongside
/// zjoltVehicleConstraintSetNumStepsBetweenCollisionTestActive. Any filters
/// set above are re-applied to the new tester, and
/// zjoltVehicleConstraintGetCollisionTesterKind reports the new kind.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetCollisionTester(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleCollisionTesterDesc *desc);

//===----------------------------------------------------------------------===//
// Step callbacks
//
// Three points inside the vehicle's own slice of a physics step. They are the
// only places where a host can act on wheel contacts in the same step that
// found them: reading them afterwards, from between two
// zjoltPhysicsSystemStep calls, is always one step stale.
//
// What is running when these fire is unusual and worth stating plainly. They
// are called from PhysicsSystem::Update, while every body and constraint
// mutex is held, on whichever job thread is stepping this vehicle — several
// vehicles' callbacks can run at once. So:
//
//   * Read and write bodies and constraints; do NOT add or remove them, and
//     do not destroy this vehicle from inside its own callback.
//   * Do not assume single-threading. Two vehicles sharing a `user` pointer
//     is the caller's race to avoid.
//   * Nothing may unwind out of the callback. Jolt is built without
//     exceptions and this one is called with locks held; a throw is
//     std::terminate and an escape past the lock is a permanent deadlock in
//     the next update.
//
// `context.delta_time` is the sub-step's, not the whole update's:
// zjoltPhysicsSystemStep with N collision steps calls each of these N
// times, with `is_first_step`/`is_last_step` marking the ends.
//===----------------------------------------------------------------------===//

typedef struct ZJoltVehicleStepContext {
  /// Seconds in THIS sub-step.
  float delta_time;
  bool is_first_step;
  bool is_last_step;
} ZJoltVehicleStepContext;

typedef struct ZJoltVehicleStepCallback {
  void (*on_step)(void *user, ZJoltVehicleConstraint *constraint,
                  const ZJoltVehicleStepContext *context);
  void *user;
} ZJoltVehicleStepCallback;

/// Before anything else the vehicle does this step, and before the wheels
/// have been collision-tested. The last moment the vehicle body's position or
/// orientation can be changed, and where per-step steering belongs.
/// A NULL `callback` (or one with a NULL `on_step`) removes it.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetPreStepCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleStepCallback *callback);
ZJOLT_API void zjoltVehicleConstraintGetPreStepCallback(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleStepCallback *out);

/// Immediately after the wheels have been collision-tested and before the
/// anti-roll bars run: where the per-wheel contact getters are freshest, for
/// surface detection, tire smoke and skid marks. Do not move the vehicle
/// body here — the contacts have already been found against where it was.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetPostCollideCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleStepCallback *callback);
ZJOLT_API void zjoltVehicleConstraintGetPostCollideCallback(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleStepCallback *out);

/// After the controller has finished, before the sleep check — so a velocity
/// changed here is still seen this step. For in-air control and stunt
/// stabilisation. Do not move the vehicle body here either.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetPostStepCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleStepCallback *callback);
ZJOLT_API void zjoltVehicleConstraintGetPostStepCallback(
    const ZJoltVehicleConstraint *constraint, ZJoltVehicleStepCallback *out);

//===----------------------------------------------------------------------===//
// Tire friction callbacks
//
// Both run under the same rules as the step callbacks above — job thread,
// locks held, nothing may unwind.
//===----------------------------------------------------------------------===//

/// Combines a tire's own friction with the friction of the surface it is
/// standing on, once per wheel per step. This is where a per-material road —
/// ice, gravel, a wet patch — is implemented: `body`/`sub_shape_id` name what
/// the wheel is touching, and the two friction values arrive holding the
/// TIRE's numbers for the caller to overwrite with the combined ones.
///
/// Jolt's default, which a NULL callback restores, is the geometric mean:
/// sqrt(tire_friction * body_friction) for each direction.
typedef struct ZJoltVehicleCombineFrictionCallback {
  void (*combine)(void *user, uint32_t wheel_index, ZJoltBodyId body,
                  ZJoltSubShapeId sub_shape_id, float *longitudinal_friction,
                  float *lateral_friction);
  void *user;
} ZJoltVehicleCombineFrictionCallback;

ZJOLT_API ZJoltResult zjoltVehicleConstraintSetCombineFrictionCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleCombineFrictionCallback *callback);
ZJOLT_API void zjoltVehicleConstraintGetCombineFrictionCallback(
    const ZJoltVehicleConstraint *constraint,
    ZJoltVehicleCombineFrictionCallback *out);

/// Everything Jolt knows about one wheel's grip this step, handed to a
/// ZJoltVehicleTireMaxImpulseCallback.
typedef struct ZJoltVehicleTireImpulseInputs {
  /// N s the suspension pushed into the ground — the wheel's share of the
  /// vehicle's weight, plus load transfer. This is what makes the standard
  /// friction-times-normal-force model work out.
  float suspension_impulse;
  /// Already combined with the ground's friction (see the combine callback)
  /// and already sampled off the tire's own slip curves.
  float longitudinal_friction;
  float lateral_friction;
  /// Where on those curves the wheel currently is. Longitudinal slip is a
  /// ratio, lateral slip an angle in radians.
  float longitudinal_slip;
  float lateral_slip;
  float delta_time;
} ZJoltVehicleTireImpulseInputs;

/// Caps how much grip a tire is allowed to use this step, per wheel. Jolt's
/// default — restored by a NULL callback — is
/// `friction * suspension_impulse` in each direction, an uncoupled friction
/// circle that lets a tire brake and corner at full strength at once;
/// replacing it is how a proper elliptical friction circle, an ABS or a
/// traction-control system gets written.
///
/// The actual impulse applied may be lower than what is returned here: this
/// is a ceiling, not a demand.
typedef struct ZJoltVehicleTireMaxImpulseCallback {
  void (*compute)(void *user, uint32_t wheel_index,
                  const ZJoltVehicleTireImpulseInputs *inputs,
                  float *out_longitudinal_impulse, float *out_lateral_impulse);
  void *user;
} ZJoltVehicleTireMaxImpulseCallback;

/// Wheeled and motorcycle only; ZJOLT_RESULT_INVALID_ARGUMENT against a
/// tracked constraint, whose tracks have no slip curves to cap.
ZJOLT_API ZJoltResult zjoltVehicleConstraintSetTireMaxImpulseCallback(
    ZJoltVehicleConstraint *constraint,
    const ZJoltVehicleTireMaxImpulseCallback *callback);
ZJOLT_API void zjoltVehicleConstraintGetTireMaxImpulseCallback(
    const ZJoltVehicleConstraint *constraint,
    ZJoltVehicleTireMaxImpulseCallback *out);

//===----------------------------------------------------------------------===//
// Viewed as a plain constraint
//===----------------------------------------------------------------------===//

/// A vehicle constraint IS a JPH::Constraint underneath — it sits in the
/// same system constraint list zjoltPhysicsSystemGetNumConstraints counts
/// and zjoltPhysicsSystemDrawConstraints draws — so the generic accessors
/// (zjoltConstraintSetEnabled, zjoltConstraintGetSubType,
/// zjoltConstraintIsAdded, zjoltConstraintSetPriority, ...) work on it
/// through this borrowed view. Borrowed: never zjoltConstraintRelease it,
/// and it is valid only as long as `constraint` is — do not use it past
/// zjoltVehicleConstraintDestroy. NULL for a NULL constraint.
ZJOLT_API ZJoltConstraint *zjoltVehicleConstraintAsConstraint(
    const ZJoltVehicleConstraint *constraint);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_VEHICLE_H_
