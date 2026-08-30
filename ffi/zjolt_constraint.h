//===----------------------------------------------------------------------===//
// zjolt — constraints: the joints between two bodies.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part.
//
// Reference counted like a shape: zjoltConstraintCreate* hands back one
// reference; zjoltConstraintAdd/Remove take/drop an independent reference.
//
// Stores raw pointers to its two bodies, like Jolt: destroying a body a
// live constraint still names leaves it dangling, undetected. Remove and
// release constraints on a body before destroying it. Every Jolt
// precondition here is checked and reported as ZJOLT_RESULT_INVALID_ARGUMENT
// rather than forwarded to an abort.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_CONSTRAINT_H_
#define ZJOLT_CONSTRAINT_H_

#include "zjolt_core.h"
#include "zjolt_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// The handle
//===----------------------------------------------------------------------===//

/// A joint between two bodies. Reference counted; see Ownership above.
typedef struct ZJoltConstraint ZJoltConstraint;

/// The body id that means "the world": an implicit, infinitely heavy
/// static body at the origin with identity rotation. Pass as `body1` or
/// `body2` to any zjoltConstraintCreate* to bolt the other body to it.
///
/// Not both at once: two world bodies would constrain two things that
/// cannot move, which is refused.
#define ZJOLT_BODY_ID_WORLD ZJOLT_BODY_ID_INVALID

//===----------------------------------------------------------------------===//
// Enumerations
//===----------------------------------------------------------------------===//

/// Which kind of joint a handle actually is. Every accessor below checks
/// this first, so asking a hinge for its slider position is a returned
/// error, not a reinterpreted object.
///
/// OTHER covers what this ABI cannot produce: Jolt's vehicle constraint,
/// and three of the four `User*` slots for external constraint types.
typedef enum ZJoltConstraintSubType {
  ZJOLT_CONSTRAINT_SUB_TYPE_OTHER = 0,
  ZJOLT_CONSTRAINT_SUB_TYPE_FIXED = 1,
  ZJOLT_CONSTRAINT_SUB_TYPE_POINT = 2,
  ZJOLT_CONSTRAINT_SUB_TYPE_HINGE = 3,
  ZJOLT_CONSTRAINT_SUB_TYPE_SLIDER = 4,
  ZJOLT_CONSTRAINT_SUB_TYPE_DISTANCE = 5,
  ZJOLT_CONSTRAINT_SUB_TYPE_CONE = 6,
  ZJOLT_CONSTRAINT_SUB_TYPE_SWING_TWIST = 7,
  ZJOLT_CONSTRAINT_SUB_TYPE_SIX_DOF = 8,
  ZJOLT_CONSTRAINT_SUB_TYPE_PATH = 9,
  ZJOLT_CONSTRAINT_SUB_TYPE_GEAR = 10,
  ZJOLT_CONSTRAINT_SUB_TYPE_RACK_AND_PINION = 11,
  ZJOLT_CONSTRAINT_SUB_TYPE_PULLEY = 12,
  /// A constraint created by zjoltConstraintCreateCustom.
  ZJOLT_CONSTRAINT_SUB_TYPE_CUSTOM = 13,
} ZJoltConstraintSubType;

/// Which space a descriptor's points and axes are expressed in.
///
/// WORLD is what a host normally wants: place the two bodies, then describe
/// the joint where it visually is. LOCAL_TO_BODY_COM is relative to each
/// body's CENTRE OF MASS, not its origin — an offset-centre-of-mass or a
/// compound shape moves it, so subtract the shape's centre of mass yourself.
typedef enum ZJoltConstraintSpace {
  ZJOLT_CONSTRAINT_SPACE_LOCAL_TO_BODY_COM = 0,
  ZJOLT_CONSTRAINT_SPACE_WORLD = 1,
} ZJoltConstraintSpace;

/// What a motor is driving towards, if anything.
typedef enum ZJoltMotorState {
  ZJOLT_MOTOR_STATE_OFF = 0,
  /// Drive to the target velocity, limited only by the motor's force or
  /// torque limits.
  ZJOLT_MOTOR_STATE_VELOCITY = 1,
  /// Drive to the target position through the motor's spring.
  ZJOLT_MOTOR_STATE_POSITION = 2,
  /// Drive to both, the spring's damping term tracking the target velocity.
  ZJOLT_MOTOR_STATE_POSITION_AND_VELOCITY = 3,
} ZJoltMotorState;

/// How the numbers in ZJoltSpringSettings are to be read.
typedef enum ZJoltSpringMode {
  /// `frequency_or_stiffness` is an oscillation frequency in Hz and `damping`
  /// is a ratio (0 = none, 1 = critical).
  ZJOLT_SPRING_MODE_FREQUENCY_AND_DAMPING = 0,
  /// `frequency_or_stiffness` is k and `damping` is c in F = -k x - c v.
  ZJOLT_SPRING_MODE_STIFFNESS_AND_DAMPING = 1,
  /// As above, but both divided by the constraint's effective mass, which
  /// makes the numbers small and mass independent.
  ZJOLT_SPRING_MODE_MASS_NORMALIZED_STIFFNESS_AND_DAMPING = 2,
} ZJoltSpringMode;

/// The shape of a swing limit.
typedef enum ZJoltSwingType {
  /// A cone. Only symmetric limits — the minimum is taken to be the negated
  /// maximum — and it deforms at large angles.
  ZJOLT_SWING_TYPE_CONE = 0,
  /// A pyramid. Asymmetric limits are allowed.
  ZJOLT_SWING_TYPE_PYRAMID = 1,
} ZJoltSwingType;

/// One of the six degrees of freedom a six-DOF constraint controls
/// independently. X, Y and Z are the constraint frame's axes, not the world's.
typedef enum ZJoltSixDofAxis {
  ZJOLT_SIX_DOF_AXIS_TRANSLATION_X = 0,
  ZJOLT_SIX_DOF_AXIS_TRANSLATION_Y = 1,
  ZJOLT_SIX_DOF_AXIS_TRANSLATION_Z = 2,
  ZJOLT_SIX_DOF_AXIS_ROTATION_X = 3,
  ZJOLT_SIX_DOF_AXIS_ROTATION_Y = 4,
  ZJOLT_SIX_DOF_AXIS_ROTATION_Z = 5,
} ZJoltSixDofAxis;

/// How many degrees of freedom ZJoltSixDofAxis names. Sizes the arrays in
/// ZJoltSixDofConstraintDesc.
#define ZJOLT_SIX_DOF_AXIS_COUNT 6

/// How many of them are translations. The first three, and the only ones with
/// soft (spring) limits — Jolt does not implement soft rotation limits.
#define ZJOLT_SIX_DOF_TRANSLATION_AXIS_COUNT 3

/// What a path constraint does to the rotation of body 2.
typedef enum ZJoltPathRotationConstraintType {
  ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_FREE = 0,
  ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_AROUND_TANGENT = 1,
  ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_AROUND_NORMAL = 2,
  ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_AROUND_BINORMAL = 3,
  ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_TO_PATH = 4,
  ZJOLT_PATH_ROTATION_CONSTRAINT_TYPE_FULLY_CONSTRAINED = 5,
} ZJoltPathRotationConstraintType;

//===----------------------------------------------------------------------===//
// Springs and motors
//
// These two are shared by most of the joints below, so they are one struct
// each rather than a run of fields repeated per descriptor.
//===----------------------------------------------------------------------===//

/// A linear or angular spring.
///
/// All-zero means HARD: `frequency_or_stiffness <= 0` disables the spring,
/// as rigid as the solver's step count allows — a zeroed descriptor is a
/// usable one. Both numbers must be >= 0 and finite; negative is refused
/// (sign of a unit mix-up, not intent).
typedef struct ZJoltSpringSettings {
  ZJoltSpringMode mode;
  float frequency_or_stiffness;
  float damping;
} ZJoltSpringSettings;

/// What a powered joint may apply. Limits are asymmetric on purpose (a
/// push-only motor is `min = 0`); a zeroed struct applies NOTHING — set
/// the limits, or -FLT_MAX/FLT_MAX for unlimited, before switching on.
///
/// `min_* <= max_*` and a valid spring are Jolt preconditions, checked
/// wherever these cross the boundary.
typedef struct ZJoltMotorSettings {
  /// Used when the motor is driving to a POSITION. Ignored by a velocity
  /// motor.
  ZJoltSpringSettings spring;
  /// Newtons, for a motor along a translation axis. Ignored by an angular
  /// motor.
  float min_force_limit;
  float max_force_limit;
  /// Newton metres, for a motor around a rotation axis. Ignored by a linear
  /// motor.
  float min_torque_limit;
  float max_torque_limit;
} ZJoltMotorSettings;

//===----------------------------------------------------------------------===//
// Descriptors
//
// Flat plain data, not a Jolt `*Settings` object: not reference counted,
// no pointer kept past the call, no `*DescInit` (a zeroed axis is refused).
//===----------------------------------------------------------------------===//

/// Welds two bodies together, removing all six degrees of freedom.
typedef struct ZJoltFixedConstraintDesc {
  ZJoltConstraintSpace space;
  /// When true and `space` is WORLD, the attachment points are taken from
  /// where the bodies are now and `point1` / `point2` are ignored. This is the
  /// usual way to weld two bodies that are already positioned.
  bool auto_detect_point;
  ZJoltRVec3 point1;
  /// The constraint frame on body 1. Must be unit length and perpendicular to
  /// each other; default (1,0,0) and (0,1,0).
  ZJoltVec3 axis_x1;
  ZJoltVec3 axis_y1;
  ZJoltRVec3 point2;
  /// The constraint frame on body 2. Same rules. The rotation that takes frame
  /// 1 to frame 2 is what the weld holds fixed.
  ZJoltVec3 axis_x2;
  ZJoltVec3 axis_y2;
} ZJoltFixedConstraintDesc;

/// Pins two bodies at one point, leaving all rotation free. A ball joint.
typedef struct ZJoltPointConstraintDesc {
  ZJoltConstraintSpace space;
  ZJoltRVec3 point1;
  ZJoltRVec3 point2;
} ZJoltPointConstraintDesc;

/// One point and one axis of rotation. A door, an elbow, a wheel.
typedef struct ZJoltHingeConstraintDesc {
  ZJoltConstraintSpace space;
  ZJoltRVec3 point1;
  /// The axis rotation is allowed about. Unit length; default (0,1,0).
  ZJoltVec3 hinge_axis1;
  /// Perpendicular to `hinge_axis1`, and the reference the hinge angle is
  /// measured from. Unit length; default (1,0,0).
  ZJoltVec3 normal_axis1;
  ZJoltRVec3 point2;
  ZJoltVec3 hinge_axis2;
  ZJoltVec3 normal_axis2;
  /// Radians, and the range Jolt asserts on: `limits_min` in [-pi, 0] and
  /// `limits_max` in [0, pi]. Default -pi and pi, which is "no limit".
  ///
  /// The two may only be EQUAL when `limits_spring` is soft. A hard hinge
  /// locked to a single angle is a fixed constraint, and Jolt says so with an
  /// assertion; this reports ZJOLT_RESULT_INVALID_ARGUMENT.
  float limits_min;
  float limits_max;
  /// Makes the limits soft: past one, a spring pulls back instead of a wall.
  ZJoltSpringSettings limits_spring;
  /// Newton metres resisting rotation when no motor is driving. 0 = none.
  float max_friction_torque;
  ZJoltMotorSettings motor;
} ZJoltHingeConstraintDesc;

/// Movement along one axis and nothing else. A prismatic joint, a piston.
typedef struct ZJoltSliderConstraintDesc {
  ZJoltConstraintSpace space;
  /// As for a fixed constraint: take the attachment points from where the
  /// bodies are now, and call that slider position 0.
  bool auto_detect_point;
  ZJoltRVec3 point1;
  /// The direction movement is allowed along. Unit length; default (1,0,0).
  ZJoltVec3 slider_axis1;
  /// Perpendicular to `slider_axis1`, fixing the frame's roll. Unit length;
  /// default (0,1,0).
  ZJoltVec3 normal_axis1;
  ZJoltRVec3 point2;
  ZJoltVec3 slider_axis2;
  ZJoltVec3 normal_axis2;
  /// Metres, measured from where the two points coincide. `limits_min` must be
  /// <= 0 and `limits_max` >= 0. Default -FLT_MAX and FLT_MAX, which is "no
  /// limit"; and as for a hinge, the two may only be equal when the spring is
  /// soft.
  float limits_min;
  float limits_max;
  ZJoltSpringSettings limits_spring;
  /// Newtons resisting sliding when no motor is driving. 0 = none.
  float max_friction_force;
  ZJoltMotorSettings motor;
} ZJoltSliderConstraintDesc;

/// Keeps two points a fixed distance — or a range of distances — apart. A
/// rope, a rod, a spring.
typedef struct ZJoltDistanceConstraintDesc {
  ZJoltConstraintSpace space;
  ZJoltRVec3 point1;
  ZJoltRVec3 point2;
  /// Metres. NEGATIVE means "use the distance the points are at right now",
  /// which is the usual way to build a rope of the length you drew. Default
  /// -1 for both, giving a rigid rod at the current separation.
  ///
  /// When both are non-negative, `min_distance` must be <= `max_distance`.
  float min_distance;
  float max_distance;
  ZJoltSpringSettings limits_spring;
} ZJoltDistanceConstraintDesc;

/// Pins two bodies at a point and limits how far their twist axes may open
/// apart. The socket half of a ball-and-socket.
typedef struct ZJoltConeConstraintDesc {
  ZJoltConstraintSpace space;
  ZJoltRVec3 point1;
  /// The cone's principal axis on each body. Unit length; default (1,0,0).
  ZJoltVec3 twist_axis1;
  ZJoltRVec3 point2;
  ZJoltVec3 twist_axis2;
  /// Radians from the principal axis to the cone's edge. Must be in [0, pi];
  /// 0 locks the two axes together.
  float half_cone_angle;
} ZJoltConeConstraintDesc;

/// A ball joint with independent swing and twist limits, and motors for both.
/// What a ragdoll's shoulders and hips are made of.
typedef struct ZJoltSwingTwistConstraintDesc {
  ZJoltConstraintSpace space;
  ZJoltRVec3 position1;
  /// The twist axis and a perpendicular plane axis, forming the frame. Unit
  /// length; defaults (1,0,0) and (0,1,0).
  ZJoltVec3 twist_axis1;
  ZJoltVec3 plane_axis1;
  ZJoltRVec3 position2;
  ZJoltVec3 twist_axis2;
  ZJoltVec3 plane_axis2;
  ZJoltSwingType swing_type;
  /// Radians, both in [0, pi]. The swing limit is symmetric about the twist
  /// axis in each of the two perpendicular directions.
  float normal_half_cone_angle;
  float plane_half_cone_angle;
  /// Radians, both in [-pi, pi], with `twist_min_angle` <= `twist_max_angle`.
  float twist_min_angle;
  float twist_max_angle;
  /// Newton metres resisting rotation when no motor is driving. 0 = none.
  float max_friction_torque;
  ZJoltMotorSettings swing_motor;
  ZJoltMotorSettings twist_motor;
} ZJoltSwingTwistConstraintDesc;

/// Every degree of freedom controlled separately: free, fixed, or limited,
/// with a motor and friction on each. The general joint the others specialise.
typedef struct ZJoltSixDofConstraintDesc {
  ZJoltConstraintSpace space;
  ZJoltRVec3 position1;
  /// The constraint frame on body 1. Unit length and perpendicular; defaults
  /// (1,0,0) and (0,1,0).
  ZJoltVec3 axis_x1;
  ZJoltVec3 axis_y1;
  ZJoltRVec3 position2;
  ZJoltVec3 axis_x2;
  ZJoltVec3 axis_y2;
  ZJoltSwingType swing_type;
  /// Indexed by ZJoltSixDofAxis. Newtons for a translation, newton metres for
  /// a rotation. 0 = none.
  float max_friction[ZJOLT_SIX_DOF_AXIS_COUNT];
  /// Indexed by ZJoltSixDofAxis. Metres for translation, radians for
  /// rotation. FREE is `min = -FLT_MAX, max = FLT_MAX` (default); FIXED is
  /// `min > max`, driving that axis to zero; anything else is limited.
  /// Rotation limits are clamped to [-pi, pi] and, for CONE swing, forced
  /// symmetric — read-back may not match what was written.
  float limit_min[ZJOLT_SIX_DOF_AXIS_COUNT];
  float limit_max[ZJOLT_SIX_DOF_AXIS_COUNT];
  /// Soft limits, translations only. Jolt has no soft rotation limits.
  ZJoltSpringSettings limits_spring[ZJOLT_SIX_DOF_TRANSLATION_AXIS_COUNT];
  /// Indexed by ZJoltSixDofAxis.
  ZJoltMotorSettings motor[ZJOLT_SIX_DOF_AXIS_COUNT];
} ZJoltSixDofConstraintDesc;

/// Ties the rotation of two bodies together, as gear teeth do:
/// `rotation1 = -ratio * rotation2`.
///
/// Velocity only, unless you hand it the two hinges through
/// `zjoltGearConstraintSetConstraints`, which is what lets it correct the
/// positional drift that otherwise accumulates.
typedef struct ZJoltGearConstraintDesc {
  ZJoltConstraintSpace space;
  /// Each gear's axis of rotation. Unit length; default (1,0,0).
  ZJoltVec3 hinge_axis1;
  ZJoltVec3 hinge_axis2;
  /// Gear 2's tooth count divided by gear 1's. Must be finite and non-zero.
  float ratio;
} ZJoltGearConstraintDesc;

/// Ties the rotation of body 1 to the translation of body 2:
/// `rotation = ratio * translation`. Body 1 is the pinion, body 2 the rack.
typedef struct ZJoltRackAndPinionConstraintDesc {
  ZJoltConstraintSpace space;
  /// The pinion's axis of rotation. Unit length; default (1,0,0).
  ZJoltVec3 hinge_axis;
  /// The rack's direction of travel. Unit length; default (1,0,0).
  ZJoltVec3 slider_axis;
  /// Radians of pinion per metre of rack. Must be finite and non-zero.
  float ratio;
} ZJoltRackAndPinionConstraintDesc;

/// Two bodies on a rope over two fixed points: as one rises the other falls.
typedef struct ZJoltPulleyConstraintDesc {
  ZJoltConstraintSpace space;
  ZJoltRVec3 body_point1;
  /// The point in the WORLD the rope runs over. Always world space, whatever
  /// `space` says — it belongs to no body.
  ZJoltRVec3 fixed_point1;
  ZJoltRVec3 body_point2;
  ZJoltRVec3 fixed_point2;
  /// How much of segment 2 counts against segment 1. 1 is a plain pulley;
  /// higher is a block and tackle. Must be finite and > 0.
  float ratio;
  /// Metres of total rope. NEGATIVE means "use the length the bodies are at
  /// now". Default 0 and -1: a rope that cannot get longer than it is.
  ///
  /// When both are non-negative, `min_length` must be <= `max_length`, and
  /// neither may be negative after the sentinel is resolved.
  float min_length;
  float max_length;
} ZJoltPulleyConstraintDesc;

//===----------------------------------------------------------------------===//
// Paths
//
// A path has a lifetime of its own, normally shared by every cart on one
// track: a second reference-counted handle, not an array in the descriptor.
//===----------------------------------------------------------------------===//

/// A curve a path constraint follows. Reference counted, and shareable between
/// constraints and between systems.
typedef struct ZJoltPathConstraintPath ZJoltPathConstraintPath;

/// One control point of a cubic Hermite spline.
typedef struct ZJoltPathPoint {
  ZJoltVec3 position;
  /// The derivative at this point — NOT normalised, its length sets how far
  /// the curve reaches before it turns. Must be non-zero.
  ZJoltVec3 tangent;
  /// Which way is "up" here. Must be non-zero and must not be parallel to
  /// `tangent`: the constraint builds its frame from the cross product of the
  /// two, and a parallel pair leaves it with no frame at all.
  ZJoltVec3 normal;
} ZJoltPathPoint;

/// Builds a cubic Hermite spline through `points`.
///
/// At least two points, and for a looping path the first and last must differ
/// — Jolt asserts on both. A looping path's last segment runs from the last
/// point back to the first, so a duplicated endpoint is a zero-length segment
/// rather than a closed loop.
ZJOLT_API ZJoltResult zjoltPathConstraintPathCreateHermite(
    const ZJoltPathPoint *points, uint32_t count, bool is_looping,
    ZJoltPathConstraintPath **out);

ZJOLT_API void zjoltPathConstraintPathAddRef(const ZJoltPathConstraintPath *path);
ZJOLT_API void zjoltPathConstraintPathRelease(const ZJoltPathConstraintPath *path);
ZJOLT_API uint32_t zjoltPathConstraintPathGetRefCount(
    const ZJoltPathConstraintPath *path);

ZJOLT_API bool zjoltPathConstraintPathIsLooping(
    const ZJoltPathConstraintPath *path);

/// The largest fraction that names a point on this path. A fraction is an
/// index into the control points plus a remainder, not a 0-to-1 parameter.
ZJOLT_API float zjoltPathConstraintPathGetMaxFraction(
    const ZJoltPathConstraintPath *path);

/// The fraction of the point on the path closest to `position`.
///
/// `fraction_hint` seeds the search; pass the constraint's current fraction
/// when tracking, or 0 when placing something for the first time.
ZJOLT_API ZJoltResult zjoltPathConstraintPathGetClosestPoint(
    const ZJoltPathConstraintPath *path, const ZJoltVec3 *position,
    float fraction_hint, float *out_fraction);

/// The frame at `fraction`. Any out-parameter may be NULL.
ZJOLT_API ZJoltResult zjoltPathConstraintPathGetPointOnPath(
    const ZJoltPathConstraintPath *path, float fraction, ZJoltVec3 *out_position,
    ZJoltVec3 *out_tangent, ZJoltVec3 *out_normal, ZJoltVec3 *out_binormal);

/// Constrains body 2 to slide along a path attached to body 1.
typedef struct ZJoltPathConstraintDesc {
  /// Required. A reference is taken for as long as the constraint holds it.
  const ZJoltPathConstraintPath *path;
  /// Where the path's origin sits relative to body 1's world transform.
  ZJoltVec3 path_position;
  /// How the path is oriented relative to body 1. Renormalised if it has
  /// drifted; an unnormalisable one becomes the identity.
  ZJoltQuat path_rotation;
  /// Where along the path body 2 starts. Clamped to the path for a
  /// non-looping path, wrapped for a looping one.
  float path_fraction;
  /// Newtons resisting sliding when no motor is driving. 0 = none.
  float max_friction_force;
  ZJoltMotorSettings motor;
  ZJoltPathRotationConstraintType rotation_constraint_type;
} ZJoltPathConstraintDesc;

//===----------------------------------------------------------------------===//
// Construction
//
// Creating a constraint does NOT add it — call zjoltConstraintAdd. A body
// id naming nothing in `system` is ZJOLT_RESULT_BODY_NOT_FOUND; ZJOLT_BODY_ID_WORLD names the world.
//===----------------------------------------------------------------------===//

ZJOLT_API ZJoltResult zjoltConstraintCreateFixed(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltFixedConstraintDesc *desc, ZJoltConstraint **out);

ZJOLT_API ZJoltResult zjoltConstraintCreatePoint(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltPointConstraintDesc *desc, ZJoltConstraint **out);

ZJOLT_API ZJoltResult zjoltConstraintCreateHinge(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltHingeConstraintDesc *desc, ZJoltConstraint **out);

/// Note the asymmetry Jolt documents: the rotation part is solved from body 1,
/// so where the two differ in mass, body 1 should be the heavier.
ZJOLT_API ZJoltResult zjoltConstraintCreateSlider(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltSliderConstraintDesc *desc, ZJoltConstraint **out);

ZJOLT_API ZJoltResult zjoltConstraintCreateDistance(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltDistanceConstraintDesc *desc, ZJoltConstraint **out);

ZJOLT_API ZJoltResult zjoltConstraintCreateCone(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltConeConstraintDesc *desc, ZJoltConstraint **out);

ZJOLT_API ZJoltResult zjoltConstraintCreateSwingTwist(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltSwingTwistConstraintDesc *desc, ZJoltConstraint **out);

ZJOLT_API ZJoltResult zjoltConstraintCreateSixDof(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltSixDofConstraintDesc *desc, ZJoltConstraint **out);

/// Body 1 is gear 1, body 2 is gear 2.
ZJOLT_API ZJoltResult zjoltConstraintCreateGear(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltGearConstraintDesc *desc, ZJoltConstraint **out);

/// Body 1 is the pinion, body 2 the rack.
ZJOLT_API ZJoltResult zjoltConstraintCreateRackAndPinion(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltRackAndPinionConstraintDesc *desc, ZJoltConstraint **out);

ZJOLT_API ZJoltResult zjoltConstraintCreatePulley(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltPulleyConstraintDesc *desc, ZJoltConstraint **out);

/// Body 1 carries the path, body 2 rides it.
ZJOLT_API ZJoltResult zjoltConstraintCreatePath(
    ZJoltPhysicsSystem *system, ZJoltBodyId body1, ZJoltBodyId body2,
    const ZJoltPathConstraintDesc *desc, ZJoltConstraint **out);

//===----------------------------------------------------------------------===//
// Reference counting
//===----------------------------------------------------------------------===//

ZJOLT_API void zjoltConstraintAddRef(const ZJoltConstraint *constraint);
ZJOLT_API void zjoltConstraintRelease(const ZJoltConstraint *constraint);
ZJOLT_API uint32_t zjoltConstraintGetRefCount(const ZJoltConstraint *constraint);

//===----------------------------------------------------------------------===//
// Membership of a system
//===----------------------------------------------------------------------===//

/// Starts simulating `constraint` in `system`, and takes a reference of its
/// own — see Ownership at the top of this file.
///
/// Refuses, rather than aborting, when the constraint is already in a
/// system (Jolt asserts on a second add) or its bodies are not part of
/// this system (Jolt would index the wrong body manager).
ZJOLT_API ZJoltResult zjoltConstraintAdd(ZJoltPhysicsSystem *system,
                                         ZJoltConstraint *constraint);

/// Stops simulating `constraint` and drops the system's reference.
///
/// Refuses a constraint that is not in this system; Jolt asserts on that and
/// then corrupts its constraint list, because the index it swaps with is the
/// invalid one.
ZJOLT_API ZJoltResult zjoltConstraintRemove(ZJoltPhysicsSystem *system,
                                            ZJoltConstraint *constraint);

/// Whether `constraint` is currently in `system`.
///
/// O(number of constraints): Jolt exposes membership only as a copy of the
/// whole list, and an exact answer is worth more here than a cached one that
/// could go stale when a system is destroyed out from under it.
ZJOLT_API bool zjoltConstraintIsAdded(const ZJoltPhysicsSystem *system,
                                      const ZJoltConstraint *constraint);

ZJOLT_API uint32_t zjoltPhysicsSystemGetNumConstraints(
    const ZJoltPhysicsSystem *system);

/// Every constraint currently in `system`, in Jolt's own order. Two-call
/// protocol: `*out_count` always receives the true number, so a capacity of 0
/// with `out_constraints` NULL is a size query.
/// Each one written carries a reference of its own, as a
/// zjoltConstraintCreate* does — release every one: Jolt's list is a copy of
/// refs that dies with the call, and its only other owner is the system.
ZJOLT_API ZJoltResult zjoltPhysicsSystemGetConstraints(
    const ZJoltPhysicsSystem *system, ZJoltConstraint **out_constraints,
    uint32_t capacity, uint32_t *out_count);

//===----------------------------------------------------------------------===//
// Common state
//===----------------------------------------------------------------------===//

ZJOLT_API ZJoltConstraintSubType zjoltConstraintGetSubType(
    const ZJoltConstraint *constraint);

/// A disabled constraint stays in the system and applies nothing. This is how
/// a joint is broken, and how it is repaired.
ZJOLT_API void zjoltConstraintSetEnabled(ZJoltConstraint *constraint,
                                         bool enabled);
ZJOLT_API bool zjoltConstraintIsEnabled(const ZJoltConstraint *constraint);

/// Whether the solver will actually touch it this step: enabled, and at least
/// one of its two bodies awake and dynamic.
ZJOLT_API bool zjoltConstraintIsActive(const ZJoltConstraint *constraint);

/// Wakes both of `constraint`'s bodies — the pairwise counterpart of
/// zjoltBodyActivate, for when the caller has the joint rather than
/// either body it names.
///
/// Same two refusals as zjoltConstraintAdd: no bodies to wake if not a
/// two-body constraint, and a body mismatch would wake the wrong body.
ZJOLT_API ZJoltResult zjoltConstraintActivate(ZJoltPhysicsSystem *system,
                                              ZJoltConstraint *constraint);

ZJOLT_API void zjoltConstraintSetUserData(ZJoltConstraint *constraint,
                                          uint64_t user_data);
ZJOLT_API uint64_t zjoltConstraintGetUserData(const ZJoltConstraint *constraint);

/// Constraints with a higher priority are solved later, which makes them win.
ZJOLT_API void zjoltConstraintSetPriority(ZJoltConstraint *constraint,
                                          uint32_t priority);
ZJOLT_API uint32_t zjoltConstraintGetPriority(const ZJoltConstraint *constraint);

/// Solver iterations for this constraint alone; 0 means "use the system's".
/// Must be under 256 — Jolt stores it in a byte and asserts.
ZJOLT_API ZJoltResult zjoltConstraintSetNumVelocityStepsOverride(
    ZJoltConstraint *constraint, uint32_t steps);
ZJOLT_API uint32_t zjoltConstraintGetNumVelocityStepsOverride(
    const ZJoltConstraint *constraint);

ZJOLT_API ZJoltResult zjoltConstraintSetNumPositionStepsOverride(
    ZJoltConstraint *constraint, uint32_t steps);
ZJOLT_API uint32_t zjoltConstraintGetNumPositionStepsOverride(
    const ZJoltConstraint *constraint);

/// The two bodies, in the order they were given. ZJOLT_BODY_ID_WORLD comes
/// back for the world.
ZJOLT_API ZJoltResult zjoltConstraintGetBodies(const ZJoltConstraint *constraint,
                                               ZJoltBodyId *out_body1,
                                               ZJoltBodyId *out_body2);

/// The constraint's own frame: transform from constraint space into the
/// body's CENTRE-OF-MASS space, not its origin; compose with the body's
/// centre-of-mass transform to reach world space. A rotation-unconstrained
/// kind returns an arbitrary rotation part; a gear or rack-and-pinion
/// returns a zero translation column. Refused for a vehicle constraint.
ZJOLT_API ZJoltResult zjoltConstraintGetConstraintToBody1Matrix(
    const ZJoltConstraint *constraint, ZJoltMat44 *out);
ZJOLT_API ZJoltResult zjoltConstraintGetConstraintToBody2Matrix(
    const ZJoltConstraint *constraint, ZJoltMat44 *out);

/// How large Jolt draws this constraint through
/// zjoltPhysicsSystemDrawConstraints. Metres; Jolt's default is 1.
///
/// Both report ZJOLT_RESULT_UNSUPPORTED unless built with
/// -Ddebug_renderer=true — the declaration stays either way.
ZJOLT_API ZJoltResult zjoltConstraintSetDrawSize(ZJoltConstraint *constraint,
                                                 float size);
ZJOLT_API ZJoltResult zjoltConstraintGetDrawSize(
    const ZJoltConstraint *constraint, float *out);

/// Throws away the impulse carried over from last step. Do this after
/// teleporting a body, so the solver does not push against a change it did
/// not make.
ZJOLT_API void zjoltConstraintResetWarmStart(ZJoltConstraint *constraint);

//===----------------------------------------------------------------------===//
// Constraint settings: the read-back half of constraint authoring. Every
// constraint kind (including one reached through
// zjoltVehicleConstraintAsConstraint) overrides GetConstraintSettings, so
// one binding here covers all of them.
//
// Does not record which bodies `constraint` joins — Jolt's own doc comment
// on GetConstraintSettings. Recreating an equivalent constraint from
// zjoltConstraintSettingsRestoreBinaryState still needs body ids from
// elsewhere.
//===----------------------------------------------------------------------===//

/// A constraint's configuration, snapshotted from a live constraint or
/// restored from a stream. Reference counted like a shape.
typedef struct ZJoltConstraintSettings ZJoltConstraintSettings;

/// Snapshots `constraint`'s current settings. ZJOLT_RESULT_OUT_OF_MEMORY if
/// Jolt could not build the settings object.
ZJOLT_API ZJoltResult zjoltConstraintGetConstraintSettings(
    const ZJoltConstraint *constraint, ZJoltConstraintSettings **out);

ZJOLT_API void zjoltConstraintSettingsAddRef(
    const ZJoltConstraintSettings *settings);
ZJOLT_API void zjoltConstraintSettingsRelease(
    const ZJoltConstraintSettings *settings);
ZJOLT_API uint32_t zjoltConstraintSettingsGetRefCount(
    const ZJoltConstraintSettings *settings);

/// The same five base fields zjoltConstraintIsActive,
/// zjoltConstraintGetPriority, zjoltConstraintGetNumVelocityStepsOverride,
/// zjoltConstraintGetNumPositionStepsOverride and zjoltConstraintGetDrawSize
/// read off a live constraint, plus zjoltConstraintGetUserData — read here
/// too since a settings object restored from a stream has no live
/// constraint to ask. False/0 if `settings` is NULL.
ZJOLT_API bool zjoltConstraintSettingsGetEnabled(
    const ZJoltConstraintSettings *settings);
ZJOLT_API uint32_t zjoltConstraintSettingsGetConstraintPriority(
    const ZJoltConstraintSettings *settings);
ZJOLT_API uint32_t zjoltConstraintSettingsGetNumVelocityStepsOverride(
    const ZJoltConstraintSettings *settings);
ZJOLT_API uint32_t zjoltConstraintSettingsGetNumPositionStepsOverride(
    const ZJoltConstraintSettings *settings);
ZJOLT_API float zjoltConstraintSettingsGetDrawConstraintSize(
    const ZJoltConstraintSettings *settings);
ZJOLT_API uint64_t zjoltConstraintSettingsGetUserData(
    const ZJoltConstraintSettings *settings);

/// Jolt's own binary stream, over the same ZJoltStream seam as
/// zjoltSkeletalAnimationSaveBinaryState — the type tag travels with the
/// bytes, so zjoltConstraintSettingsRestoreBinaryState rebuilds whichever
/// concrete kind was saved, vehicle included. Does not write user_data;
/// Jolt's own ConstraintSettings::SaveBinaryState does not either.
ZJOLT_API ZJoltResult zjoltConstraintSettingsSaveBinaryState(
    const ZJoltConstraintSettings *settings, const ZJoltStream *stream);

/// Release the result with zjoltConstraintSettingsRelease.
ZJOLT_API ZJoltResult zjoltConstraintSettingsRestoreBinaryState(
    const ZJoltStream *stream, ZJoltConstraintSettings **out);

//===----------------------------------------------------------------------===//
// Per-kind state
//
// Each narrows the handle to one kind first (ZJOLT_RESULT_INVALID_ARGUMENT
// otherwise). Lambda accessors report the solver's last-step impulse.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Fixed
//===----------------------------------------------------------------------===//

ZJOLT_API ZJoltResult zjoltFixedConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltFixedConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Point
//===----------------------------------------------------------------------===//

/// Moves the attachment point on body 1. The constraint keeps no record
/// of the rotation between the bodies, so moving a point does not
/// disturb anything else.
ZJOLT_API ZJoltResult zjoltPointConstraintSetPoint1(ZJoltConstraint *constraint,
                                                    ZJoltConstraintSpace space,
                                                    const ZJoltRVec3 *point);
ZJOLT_API ZJoltResult zjoltPointConstraintSetPoint2(ZJoltConstraint *constraint,
                                                    ZJoltConstraintSpace space,
                                                    const ZJoltRVec3 *point);

/// The attachment point in the body's centre-of-mass space, whichever space it
/// was given in.
ZJOLT_API ZJoltResult zjoltPointConstraintGetLocalSpacePoint1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltPointConstraintGetLocalSpacePoint2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

ZJOLT_API ZJoltResult zjoltPointConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Hinge
//===----------------------------------------------------------------------===//

/// The hinge's frame, read back in each body's CENTRE-OF-MASS space —
/// what Jolt resolved to, whichever space (LOCAL_TO_BODY_COM or WORLD) it
/// was built in.
///
/// No setter: a hinge recomputes its rest orientation once at construction
/// and never revisits it. Rebuild the constraint to change the frame.
ZJOLT_API ZJoltResult zjoltHingeConstraintGetLocalSpacePoint1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetLocalSpacePoint2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// The axis rotation is allowed about, on each body.
ZJOLT_API ZJoltResult zjoltHingeConstraintGetLocalSpaceHingeAxis1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetLocalSpaceHingeAxis2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// The reference the hinge angle is measured from, on each body.
ZJOLT_API ZJoltResult zjoltHingeConstraintGetLocalSpaceNormalAxis1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetLocalSpaceNormalAxis2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// Radians, measured from where the two normal axes align.
ZJOLT_API ZJoltResult zjoltHingeConstraintGetCurrentAngle(
    const ZJoltConstraint *constraint, float *out);

/// `min` must be in [-pi, 0] and `max` in [0, pi] — Jolt asserts both.
///
/// Unlike creation, equal limits are allowed here: a hinge can legitimately be
/// driven to a locked state and back at run time.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetLimits(ZJoltConstraint *constraint,
                                                    float min, float max);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetLimits(
    const ZJoltConstraint *constraint, float *out_min, float *out_max);

/// Whether the limits are narrower than a full turn. False means the hinge
/// spins freely and the limit part is not solved at all.
ZJOLT_API ZJoltResult zjoltHingeConstraintHasLimits(
    const ZJoltConstraint *constraint, bool *out);

ZJOLT_API ZJoltResult zjoltHingeConstraintSetLimitsSpringSettings(
    ZJoltConstraint *constraint, const ZJoltSpringSettings *spring);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetLimitsSpringSettings(
    const ZJoltConstraint *constraint, ZJoltSpringSettings *out);

/// Replaces the motor's settings. Refused when they are not valid, because the
/// next `SetMotorState` would assert on exactly that.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out);

/// Switches the motor on or off. This is the call Jolt guards with
/// `JPH_ASSERT(state == Off || mMotorSettings.IsValid())`; every path that can
/// set those settings validates them, so the assertion is unreachable from
/// here.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetMotorState(
    ZJoltConstraint *constraint, ZJoltMotorState state);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetMotorState(
    const ZJoltConstraint *constraint, ZJoltMotorState *out);

/// Radians per second, for a velocity motor.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetTargetAngularVelocity(
    ZJoltConstraint *constraint, float angular_velocity);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetTargetAngularVelocity(
    const ZJoltConstraint *constraint, float *out);

/// Radians, for a position motor. Jolt CLAMPS this to the current limits, so
/// reading it back may give a different number than was written.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetTargetAngle(
    ZJoltConstraint *constraint, float angle);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetTargetAngle(
    const ZJoltConstraint *constraint, float *out);

/// The same target as an orientation of body 2 relative to body 1,
/// projected onto the hinge axis and clamped as above.
///
/// No getter: Jolt derives an angle from `orientation` and stores only
/// that, overwriting the same field a direct SetTargetAngle would. Read
/// zjoltHingeConstraintGetTargetAngle instead; the quaternion itself is gone.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetTargetOrientation(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation);

/// Newton metres resisting rotation when no motor is driving.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetMaxFrictionTorque(
    ZJoltConstraint *constraint, float torque);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetMaxFrictionTorque(
    const ZJoltConstraint *constraint, float *out);

ZJOLT_API ZJoltResult zjoltHingeConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// The impulse holding the two bodies to the hinge AXIS — two numbers,
/// one per rotational DOF the hinge removes. Either out-parameter may be
/// NULL. Watch this for a hinge that should snap sideways.
///
/// Separate from the LIMITS impulse (a door forced past its stop) and the
/// MOTOR impulse (what the motor itself spends).
ZJOLT_API ZJoltResult zjoltHingeConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, float *out_x, float *out_y);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetTotalLambdaRotationLimits(
    const ZJoltConstraint *constraint, float *out);

ZJOLT_API ZJoltResult zjoltHingeConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, float *out);

//===----------------------------------------------------------------------===//
// Slider
//===----------------------------------------------------------------------===//

/// Metres along the slider axis, measured from where the two points coincide.
ZJOLT_API ZJoltResult zjoltSliderConstraintGetCurrentPosition(
    const ZJoltConstraint *constraint, float *out);

/// `min` must be <= 0 and `max` >= 0 — Jolt asserts both.
ZJOLT_API ZJoltResult zjoltSliderConstraintSetLimits(ZJoltConstraint *constraint,
                                                     float min, float max);
ZJOLT_API ZJoltResult zjoltSliderConstraintGetLimits(
    const ZJoltConstraint *constraint, float *out_min, float *out_max);
ZJOLT_API ZJoltResult zjoltSliderConstraintHasLimits(
    const ZJoltConstraint *constraint, bool *out);

ZJOLT_API ZJoltResult zjoltSliderConstraintSetLimitsSpringSettings(
    ZJoltConstraint *constraint, const ZJoltSpringSettings *spring);
ZJOLT_API ZJoltResult zjoltSliderConstraintGetLimitsSpringSettings(
    const ZJoltConstraint *constraint, ZJoltSpringSettings *out);

ZJOLT_API ZJoltResult zjoltSliderConstraintSetMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor);
ZJOLT_API ZJoltResult zjoltSliderConstraintGetMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out);
ZJOLT_API ZJoltResult zjoltSliderConstraintSetMotorState(
    ZJoltConstraint *constraint, ZJoltMotorState state);
ZJOLT_API ZJoltResult zjoltSliderConstraintGetMotorState(
    const ZJoltConstraint *constraint, ZJoltMotorState *out);

/// Metres per second, for a velocity motor.
ZJOLT_API ZJoltResult zjoltSliderConstraintSetTargetVelocity(
    ZJoltConstraint *constraint, float velocity);
ZJOLT_API ZJoltResult zjoltSliderConstraintGetTargetVelocity(
    const ZJoltConstraint *constraint, float *out);

/// Metres, for a position motor. Clamped to the current limits, as a hinge's
/// target angle is.
ZJOLT_API ZJoltResult zjoltSliderConstraintSetTargetPosition(
    ZJoltConstraint *constraint, float position);
ZJOLT_API ZJoltResult zjoltSliderConstraintGetTargetPosition(
    const ZJoltConstraint *constraint, float *out);

/// Newtons resisting sliding when no motor is driving.
ZJOLT_API ZJoltResult zjoltSliderConstraintSetMaxFrictionForce(
    ZJoltConstraint *constraint, float force);
ZJOLT_API ZJoltResult zjoltSliderConstraintGetMaxFrictionForce(
    const ZJoltConstraint *constraint, float *out);

/// The impulse that held the body onto the slider AXIS — two numbers, one per
/// direction perpendicular to it. Either out-parameter may be NULL. This is
/// what a slider being bent sideways is spending; travel past a limit shows up
/// in zjoltSliderConstraintGetTotalLambdaPositionLimits instead.
ZJOLT_API ZJoltResult zjoltSliderConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, float *out_x, float *out_y);
ZJOLT_API ZJoltResult zjoltSliderConstraintGetTotalLambdaPositionLimits(
    const ZJoltConstraint *constraint, float *out);

/// The torque that kept the two bodies from rotating relative to each other.
ZJOLT_API ZJoltResult zjoltSliderConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

ZJOLT_API ZJoltResult zjoltSliderConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, float *out);

//===----------------------------------------------------------------------===//
// Distance
//===----------------------------------------------------------------------===//

/// `min` must be <= `max` — Jolt asserts it.
///
/// Unlike the descriptor, a NEGATIVE value here is not a sentinel: this is the
/// raw setter, and "use the current separation" only exists at creation.
ZJOLT_API ZJoltResult zjoltDistanceConstraintSetDistance(
    ZJoltConstraint *constraint, float min, float max);
ZJOLT_API ZJoltResult zjoltDistanceConstraintGetDistance(
    const ZJoltConstraint *constraint, float *out_min, float *out_max);

ZJOLT_API ZJoltResult zjoltDistanceConstraintSetLimitsSpringSettings(
    ZJoltConstraint *constraint, const ZJoltSpringSettings *spring);
ZJOLT_API ZJoltResult zjoltDistanceConstraintGetLimitsSpringSettings(
    const ZJoltConstraint *constraint, ZJoltSpringSettings *out);

ZJOLT_API ZJoltResult zjoltDistanceConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, float *out);

//===----------------------------------------------------------------------===//
// Cone
//===----------------------------------------------------------------------===//

/// Radians from the principal axis to the cone's edge. Must be in [0, pi] —
/// Jolt asserts it.
ZJOLT_API ZJoltResult zjoltConeConstraintSetHalfConeAngle(
    ZJoltConstraint *constraint, float half_cone_angle);

/// The COSINE of the half angle, which is what the constraint actually keeps.
/// Jolt has no getter for the angle itself.
ZJOLT_API ZJoltResult zjoltConeConstraintGetCosHalfConeAngle(
    const ZJoltConstraint *constraint, float *out);

ZJOLT_API ZJoltResult zjoltConeConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltConeConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, float *out);

//===----------------------------------------------------------------------===//
// Swing-twist
//===----------------------------------------------------------------------===//

/// The attachment point on each body, in that body's CENTRE-OF-MASS space.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetLocalSpacePosition1(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetLocalSpacePosition2(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// The rotation from constraint space into each body's centre-of-mass space.
///
/// This is the conversion the `*CS` accessors are named after and the reason
/// the `*BodySpace` setters below exist: a target read out of
/// zjoltSwingTwistConstraintGetTargetOrientation is in CONSTRAINT space, and
/// composing it with these is what puts it back in a body's own frame.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetConstraintToBody1(
    const ZJoltConstraint *constraint, ZJoltQuat *out);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetConstraintToBody2(
    const ZJoltConstraint *constraint, ZJoltQuat *out);

/// Radians, in [0, pi]. The swing limit is symmetric about the twist axis.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetNormalHalfConeAngle(
    ZJoltConstraint *constraint, float angle);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetNormalHalfConeAngle(
    const ZJoltConstraint *constraint, float *out);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetPlaneHalfConeAngle(
    ZJoltConstraint *constraint, float angle);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetPlaneHalfConeAngle(
    const ZJoltConstraint *constraint, float *out);

/// Both twist limits at once, in radians, both in [-pi, pi] and ordered.
///
/// Deliberately not two setters: Jolt recomputes the constraint part on
/// each individual setter, so setting the minimum first while raising a
/// range would pass through minimum > maximum. One call keeps every
/// intermediate state valid.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTwistLimits(
    ZJoltConstraint *constraint, float min, float max);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTwistLimits(
    const ZJoltConstraint *constraint, float *out_min, float *out_max);

ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetSwingMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetSwingMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetSwingMotorState(
    ZJoltConstraint *constraint, ZJoltMotorState state);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetSwingMotorState(
    const ZJoltConstraint *constraint, ZJoltMotorState *out);

ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTwistMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTwistMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTwistMotorState(
    ZJoltConstraint *constraint, ZJoltMotorState state);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTwistMotorState(
    const ZJoltConstraint *constraint, ZJoltMotorState *out);

ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetMaxFrictionTorque(
    ZJoltConstraint *constraint, float torque);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetMaxFrictionTorque(
    const ZJoltConstraint *constraint, float *out);

/// Radians per second, in CONSTRAINT space: x is twist, y and z are swing.
///
/// Setting a target while the motor is OFF is not an error: the value is
/// stored and takes effect once ...SwingMotorState or ...TwistMotorState
/// turns the motor on — a silent no-op until then.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTargetAngularVelocity(
    ZJoltConstraint *constraint, const ZJoltVec3 *angular_velocity);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTargetAngularVelocity(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// The same target expressed in BODY 2's own space, converted through the
/// constraint frame and stored as the constraint-space one above — the
/// getter for both is zjoltSwingTwistConstraintGetTargetAngularVelocity,
/// returning the converted value, not what was passed here. An angular
/// velocity derived from a body's own motion is in body space; using the
/// constraint-space setter instead rotates it by the frame's error.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTargetAngularVelocityBodySpace(
    ZJoltConstraint *constraint, const ZJoltVec3 *angular_velocity);

/// The target for a position motor, in constraint space. Jolt CLAMPS it to the
/// current swing and twist limits, so reading it back may differ.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTargetOrientation(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTargetOrientation(
    const ZJoltConstraint *constraint, ZJoltQuat *out);

/// The same target as the rotation of body 2 RELATIVE TO BODY 1 — the `q`
/// in `world_rotation2 = world_rotation1 * q`, the shape an animated
/// ragdoll's per-bone pose comes in, without composing the frame by hand.
///
/// Reduces to the constraint-space setter and is clamped identically; the
/// getter for both is zjoltSwingTwistConstraintGetTargetOrientation.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTargetOrientationBodySpace(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation);

/// Where the joint actually is, as a rotation in constraint space. This is
/// what to compare a target against.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetRotationInConstraintSpace(
    const ZJoltConstraint *constraint, ZJoltQuat *out);

ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// The torque each LIMIT applied over the last step, one per limited axis.
/// Zero inside the cone and twist range; non-zero only when pushed past a
/// limit. Distinct from zjoltSwingTwistConstraintGetTotalLambdaMotor
/// (what the motors spend holding their own targets).
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaTwist(
    const ZJoltConstraint *constraint, float *out);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaSwingY(
    const ZJoltConstraint *constraint, float *out);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaSwingZ(
    const ZJoltConstraint *constraint, float *out);

ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Six degrees of freedom
//
// Every call refuses an axis outside ZJoltSixDofAxis: limits, frictions
// and motors are plain arrays of six with no bounds check of Jolt's own.
//===----------------------------------------------------------------------===//

/// Metres. See ZJoltSixDofConstraintDesc::limit_min for how free, fixed and
/// limited are spelled.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetTranslationLimits(
    ZJoltConstraint *constraint, const ZJoltVec3 *min, const ZJoltVec3 *max);

/// Radians. Clamped to [-pi, pi], and forced symmetric for a CONE swing, so
/// what comes back out may not be what went in.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetRotationLimits(
    ZJoltConstraint *constraint, const ZJoltVec3 *min, const ZJoltVec3 *max);

/// The reader for both zjoltSixDofConstraintSetTranslationLimits and
/// zjoltSixDofConstraintSetRotationLimits: pass whichever axis one of them
/// touched and this returns exactly what Jolt kept for it — its own
/// sanitising included, so a rotation limit may come back clamped or made
/// symmetric as documented on the setter above.
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetLimits(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis, float *out_min,
    float *out_max);

ZJOLT_API ZJoltResult zjoltSixDofConstraintIsFixedAxis(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis, bool *out);
ZJOLT_API ZJoltResult zjoltSixDofConstraintIsFreeAxis(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis, bool *out);

ZJOLT_API ZJoltResult zjoltSixDofConstraintSetMaxFriction(
    ZJoltConstraint *constraint, ZJoltSixDofAxis axis, float friction);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetMaxFriction(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis, float *out);

ZJOLT_API ZJoltResult zjoltSixDofConstraintSetMotorSettings(
    ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    const ZJoltMotorSettings *motor);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetMotorSettings(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    ZJoltMotorSettings *out);
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetMotorState(
    ZJoltConstraint *constraint, ZJoltSixDofAxis axis, ZJoltMotorState state);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetMotorState(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    ZJoltMotorState *out);

/// TRANSLATION axes only — Jolt asserts on a rotation axis here, because it
/// has no soft rotation limits.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetLimitsSpringSettings(
    ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    const ZJoltSpringSettings *spring);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetLimitsSpringSettings(
    const ZJoltConstraint *constraint, ZJoltSixDofAxis axis,
    ZJoltSpringSettings *out);

/// Metres per second, in constraint space.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetTargetVelocity(
    ZJoltConstraint *constraint, const ZJoltVec3 *velocity);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetTargetVelocity(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// Radians per second, in constraint space.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetTargetAngularVelocity(
    ZJoltConstraint *constraint, const ZJoltVec3 *angular_velocity);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetTargetAngularVelocity(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// Metres, in constraint space.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetTargetPosition(
    ZJoltConstraint *constraint, const ZJoltVec3 *position);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetTargetPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// Clamped to the current rotation limits, as a swing-twist target is.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetTargetOrientation(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetTargetOrientation(
    const ZJoltConstraint *constraint, ZJoltQuat *out);

/// The same target as the rotation of body 2 RELATIVE TO BODY 1, as
/// zjoltSwingTwistConstraintSetTargetOrientationBodySpace is. Reduces to
/// the constraint-space setter above.
///
/// Asymmetric, unlike other constraints: TRANSLATION motors work in body
/// 1's space, ROTATION motors in body 2's; this call takes body space.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetTargetOrientationBodySpace(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation);

ZJOLT_API ZJoltResult zjoltSixDofConstraintGetRotationInConstraintSpace(
    const ZJoltConstraint *constraint, ZJoltQuat *out);

ZJOLT_API ZJoltResult zjoltSixDofConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetTotalLambdaMotorTranslation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltSixDofConstraintGetTotalLambdaMotorRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Gear
//===----------------------------------------------------------------------===//

/// Hands the gear the two HINGES its bodies are mounted on, correcting
/// positional drift instead of matching velocities only. NULL for both
/// clears them. Both must be hinge constraints — checked here rather than
/// left to Jolt's `assert(false, "Unsupported")` during position solving.
ZJOLT_API ZJoltResult zjoltGearConstraintSetConstraints(
    ZJoltConstraint *constraint, ZJoltConstraint *gear1, ZJoltConstraint *gear2);

ZJOLT_API ZJoltResult zjoltGearConstraintGetTotalLambda(
    const ZJoltConstraint *constraint, float *out);

//===----------------------------------------------------------------------===//
// Rack and pinion
//===----------------------------------------------------------------------===//

/// As for a gear, and with the same assertion behind it: `pinion` must be a
/// HINGE and `rack` must be a SLIDER. NULL for both clears them.
ZJOLT_API ZJoltResult zjoltRackAndPinionConstraintSetConstraints(
    ZJoltConstraint *constraint, ZJoltConstraint *pinion, ZJoltConstraint *rack);

ZJOLT_API ZJoltResult zjoltRackAndPinionConstraintGetTotalLambda(
    const ZJoltConstraint *constraint, float *out);

//===----------------------------------------------------------------------===//
// Pulley
//===----------------------------------------------------------------------===//

/// Metres of rope. `min` must be non-negative and no greater than `max` —
/// Jolt asserts both.
///
/// The negative sentinel the descriptor accepts does NOT apply here:
/// "use the current length" exists only at creation.
ZJOLT_API ZJoltResult zjoltPulleyConstraintSetLength(ZJoltConstraint *constraint,
                                                     float min, float max);
ZJOLT_API ZJoltResult zjoltPulleyConstraintGetLength(
    const ZJoltConstraint *constraint, float *out_min, float *out_max);

/// The length the rope is at now, both segments summed with the ratio applied.
/// Computed from the positions the last step cached, so before the first step
/// it reports the length at creation.
ZJOLT_API ZJoltResult zjoltPulleyConstraintGetCurrentLength(
    const ZJoltConstraint *constraint, float *out);

ZJOLT_API ZJoltResult zjoltPulleyConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, float *out);

//===----------------------------------------------------------------------===//
// Path
//===----------------------------------------------------------------------===//

/// Replaces the path and the point along it that body 2 sits at.
///
/// `path` may be NULL, which makes the constraint inactive — it stays in the
/// system and applies nothing. A reference is taken for as long as the
/// constraint holds it.
ZJOLT_API ZJoltResult zjoltPathConstraintSetPath(
    ZJoltConstraint *constraint, const ZJoltPathConstraintPath *path,
    float fraction);

/// The path this constraint follows, BORROWED — valid only while the
/// constraint holds it. zjoltPathConstraintPathAddRef to keep it.
///
/// A result, not a plain pointer: NULL is ambiguous between "no path set"
/// and "not a path constraint". `*out` NULL means the former;
/// ZJOLT_RESULT_INVALID_ARGUMENT means the latter.
ZJOLT_API ZJoltResult zjoltPathConstraintGetPath(
    const ZJoltConstraint *constraint, const ZJoltPathConstraintPath **out);

ZJOLT_API ZJoltResult zjoltPathConstraintGetPathFraction(
    const ZJoltConstraint *constraint, float *out);

ZJOLT_API ZJoltResult zjoltPathConstraintSetMotorSettings(
    ZJoltConstraint *constraint, const ZJoltMotorSettings *motor);
ZJOLT_API ZJoltResult zjoltPathConstraintGetMotorSettings(
    const ZJoltConstraint *constraint, ZJoltMotorSettings *out);
ZJOLT_API ZJoltResult zjoltPathConstraintSetMotorState(
    ZJoltConstraint *constraint, ZJoltMotorState state);
ZJOLT_API ZJoltResult zjoltPathConstraintGetMotorState(
    const ZJoltConstraint *constraint, ZJoltMotorState *out);

/// Metres per second along the path, for a velocity motor.
ZJOLT_API ZJoltResult zjoltPathConstraintSetTargetVelocity(
    ZJoltConstraint *constraint, float velocity);
ZJOLT_API ZJoltResult zjoltPathConstraintGetTargetVelocity(
    const ZJoltConstraint *constraint, float *out);

/// Where along the path a position motor drives to.
///
/// On a non-looping path this must lie between 0 and the path's max fraction —
/// Jolt asserts it. A looping path accepts any fraction and wraps.
ZJOLT_API ZJoltResult zjoltPathConstraintSetTargetPathFraction(
    ZJoltConstraint *constraint, float fraction);
ZJOLT_API ZJoltResult zjoltPathConstraintGetTargetPathFraction(
    const ZJoltConstraint *constraint, float *out);

ZJOLT_API ZJoltResult zjoltPathConstraintSetMaxFrictionForce(
    ZJoltConstraint *constraint, float force);
ZJOLT_API ZJoltResult zjoltPathConstraintGetMaxFrictionForce(
    const ZJoltConstraint *constraint, float *out);

/// The impulse that held body 2 ON the path — two numbers, one per direction
/// perpendicular to the tangent. Either out-parameter may be NULL.
ZJOLT_API ZJoltResult zjoltPathConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, float *out_x, float *out_y);

/// The impulse that stopped body 2 running off the end of a non-looping path.
/// Always zero on a looping one, which has no end to stop at.
ZJOLT_API ZJoltResult zjoltPathConstraintGetTotalLambdaPositionLimits(
    const ZJoltConstraint *constraint, float *out);

/// The torque the rotation constraint applied, in the two shapes it takes:
/// the hinge part is what CONSTRAIN_AROUND_TANGENT, ...NORMAL and ...BINORMAL
/// use, and the three-component one is what CONSTRAIN_TO_PATH and
/// FULLY_CONSTRAINED use. Whichever the constraint is not using reads zero, so
/// both are safe to sample without branching on the rotation type. Either
/// out-parameter of the hinge pair may be NULL.
ZJOLT_API ZJoltResult zjoltPathConstraintGetTotalLambdaRotationHinge(
    const ZJoltConstraint *constraint, float *out_x, float *out_y);
ZJOLT_API ZJoltResult zjoltPathConstraintGetTotalLambdaRotation(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

ZJOLT_API ZJoltResult zjoltPathConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, float *out);

//===----------------------------------------------------------------------===//
// Custom constraints
//
// A C++ host subclasses JPH::Constraint; a Zig one cannot. One crossing per
// solver callback instead, with body state passed by value in ZJoltSolverBody.
//===----------------------------------------------------------------------===//

/// Body state a custom constraint's solver callbacks read and write.
/// `position_delta`/`rotation_delta` start at zero each call and are
/// applied as a rotation vector when the callback returns. `linear_velocity`/
/// `angular_velocity` read and write ONLY in `warm_start_velocity`/
/// `solve_velocity` — zero and discarded elsewhere. `inverse_mass`/
/// `inverse_inertia` (world space) are zero for a non-dynamic body.
typedef struct ZJoltSolverBody {
  ZJoltVec3 linear_velocity;
  ZJoltVec3 angular_velocity;
  ZJoltVec3 position_delta;        // accumulated by a position-solve callback
  ZJoltVec3 rotation_delta;        // ditto, as a rotation vector
  float     inverse_mass;
  float     inverse_inertia[9];    // world space, row major
  ZJoltVec3 center_of_mass;        // world space
  ZJoltQuat rotation;              // world space
  bool      is_dynamic;
} ZJoltSolverBody;

typedef struct ZJoltSolverBodyPair {
  ZJoltSolverBody body1;
  ZJoltSolverBody body2;
} ZJoltSolverBodyPair;

/// What a custom constraint's SaveState / RestoreState virtuals talk to. Not
/// a ZJoltStream: Jolt hands the shim a live JPH::StateRecorder& during a
/// step, not a stream the shim built itself, so there is nothing here to
/// adapt from a host's function-pointer table — only to forward to.
typedef struct ZJoltStateRecorder ZJoltStateRecorder;

ZJOLT_API void zjoltStateRecorderWriteBytes(ZJoltStateRecorder *recorder,
                                            const void *data, size_t size);
ZJOLT_API void zjoltStateRecorderReadBytes(ZJoltStateRecorder *recorder,
                                           void *data, size_t size);

/// The solver virtuals of a custom TwoBodyConstraint, one function pointer
/// per Jolt virtual. Every one but `draw` and `destroy` is required: Jolt
/// declares the virtual it stands in for pure, so a NULL here is refused at
/// zjoltConstraintCreateCustom rather than reaching Jolt as an unimplemented
/// call.
typedef struct ZJoltCustomConstraintCallbacks {
  void (*setup_velocity)(void *user, ZJoltSolverBodyPair *bodies, float delta_time);
  void (*warm_start_velocity)(void *user, ZJoltSolverBodyPair *bodies, float warm_start_impulse_ratio);
  bool (*solve_velocity)(void *user, ZJoltSolverBodyPair *bodies, float delta_time);
  bool (*solve_position)(void *user, ZJoltSolverBodyPair *bodies, float delta_time, float baumgarte);
  void (*reset_warm_start)(void *user);
  bool (*is_active)(const void *user);
  void (*notify_shape_changed)(void *user, ZJoltBodyId body, ZJoltVec3 delta_center_of_mass);
  void (*save_state)(const void *user, ZJoltStateRecorder *recorder);
  void (*restore_state)(void *user, ZJoltStateRecorder *recorder);
  void (*draw)(const void *user, ZJoltDebugRenderer *renderer);   // may be NULL
  void (*destroy)(void *user);                                    // may be NULL
} ZJoltCustomConstraintCallbacks;

typedef struct ZJoltCustomConstraintDesc {
  ZJoltBodyId body1;
  ZJoltBodyId body2;
  ZJoltMat44  constraint_to_body1;
  ZJoltMat44  constraint_to_body2;
  bool        enabled;
  uint32_t    num_velocity_steps_override;
  uint32_t    num_position_steps_override;
  float       draw_constraint_size;
  ZJoltCustomConstraintCallbacks callbacks;
  void       *user;
} ZJoltCustomConstraintDesc;

/// Builds a TwoBodyConstraint whose solver virtuals forward to `desc`'s
/// callbacks. Same body rules as every zjoltConstraintCreate*: a body id
/// naming nothing in `system` is ZJOLT_RESULT_BODY_NOT_FOUND,
/// ZJOLT_BODY_ID_WORLD names the world for either but not both, and the
/// two bodies must differ. `GetSubType` on the result is
/// ZJOLT_CONSTRAINT_SUB_TYPE_CUSTOM.
ZJOLT_API ZJoltResult zjoltConstraintCreateCustom(
    ZJoltPhysicsSystem *system, const ZJoltCustomConstraintDesc *desc,
    ZJoltConstraint **out);

/// The `user` pointer a custom constraint was created with.
/// ZJOLT_RESULT_INVALID_ARGUMENT on a constraint that is not one.
ZJOLT_API ZJoltResult zjoltConstraintGetCustomUserData(
    const ZJoltConstraint *constraint, void **out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_CONSTRAINT_H_
