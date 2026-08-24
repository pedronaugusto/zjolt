//===----------------------------------------------------------------------===//
// zjolt — constraints: the joints between two bodies.
//
// Part of the zjolt C ABI. Include <zjolt.h>, which pulls in every part; this
// file is split out so that no single header has to carry the whole surface.
//
// ## Ownership
//
// A constraint is REFERENCE COUNTED, like a shape. `zjoltConstraintCreate*`
// hands back a handle carrying one reference, which is yours: release it with
// `zjoltConstraintRelease` exactly once.
//
// `zjoltConstraintAdd` takes a reference OF ITS OWN, and
// `zjoltConstraintRemove` drops that one. So the two lifetimes are
// independent and the order does not matter — releasing a constraint that is
// still added leaves the system's reference holding it, and removing one you
// already released destroys it there and then.
//
// ## Lifetime against the bodies
//
// A constraint stores raw pointers to its two bodies, exactly as Jolt does.
// Destroying a body that a live constraint still names leaves that constraint
// pointing at freed memory, and nothing in Jolt or here detects it. Remove and
// release the constraints on a body before destroying it.
//
// ## Preconditions
//
// This subsystem has more caller-trippable preconditions than any other in
// the library: limits with a required sign, angles with a required range,
// axes that have to be a frame rather than three arbitrary vectors, motors
// whose settings must be valid before they can be switched on. Every one of
// them is an assertion inside Jolt — an abort in an asserts-on build — and
// every one of them is checked here and reported as
// ZJOLT_RESULT_INVALID_ARGUMENT instead, with `zjoltLastError` naming which.
// Nothing in this header forwards a call that would abort.
//
// The exceptions, which are caller obligations because no check can see them,
// are named in the comments where they apply.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_CONSTRAINT_H_
#define ZJOLT_CONSTRAINT_H_

#include "zjolt_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//===----------------------------------------------------------------------===//
// The handle
//===----------------------------------------------------------------------===//

/// A joint between two bodies. Reference counted; see Ownership above.
typedef struct ZJoltConstraint ZJoltConstraint;

/// The body id that means "the world" — an implicit, infinitely heavy static
/// body at the origin with identity rotation.
///
/// Pass it as `body1` or `body2` to any `zjoltConstraintCreate*` to bolt the
/// other body to the world. It is spelled as the invalid body id because that
/// is exactly what Jolt's own fixed-to-world body carries, and because a real
/// body can never collide with the value.
///
/// Not both at once: two world bodies would be a constraint between two things
/// that cannot move, which is refused.
#define ZJOLT_BODY_ID_WORLD ZJOLT_BODY_ID_INVALID

//===----------------------------------------------------------------------===//
// Enumerations
//===----------------------------------------------------------------------===//

/// Which kind of joint a handle actually is. Every accessor below that names a
/// kind checks this first, so asking a hinge for its slider position is a
/// returned error rather than a reinterpreted object.
///
/// OTHER covers what this ABI cannot produce: Jolt's vehicle constraint, and
/// the four `User*` slots reserved for constraint types registered by C++
/// outside this library.
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
/// All-zero means HARD: `frequency_or_stiffness <= 0` disables the spring and
/// the limit becomes as rigid as the solver's step count allows. That is the
/// right default and it is why a zeroed descriptor is a usable one.
///
/// Both numbers must be >= 0 and finite. A negative one is refused rather than
/// clamped: it is the sign of a unit mix-up, not of an intent.
typedef struct ZJoltSpringSettings {
  ZJoltSpringMode mode;
  float frequency_or_stiffness;
  float damping;
} ZJoltSpringSettings;

/// What a powered joint may apply.
///
/// The limits are asymmetric on purpose — a motor that can push but not pull
/// is `min = 0`. A zeroed struct is therefore a motor that can apply NOTHING,
/// which is not usually what a host means: set the limits, or set them to
/// -FLT_MAX and FLT_MAX for unlimited, before switching a motor on.
///
/// `min_* <= max_*` and a valid spring are Jolt preconditions
/// (`MotorSettings::IsValid`), checked wherever these cross the boundary.
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
// Flat plain data in, Jolt's settings objects built on the stack inside. A
// descriptor is NOT a Jolt `*Settings` object: it does not serialise, it is
// not reference counted, and nothing keeps a pointer to it past the call.
//
// There is deliberately no `*DescInit` for these. Jolt's defaults for an axis
// are unit vectors, and a zeroed axis is not a degenerate default that quietly
// misbehaves — it is refused with ZJOLT_RESULT_INVALID_ARGUMENT naming the
// field. The defaults each field would take are written next to it.
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
  /// Indexed by ZJoltSixDofAxis. Metres for a translation, radians for a
  /// rotation.
  ///
  ///  * FREE is `min = -FLT_MAX, max = FLT_MAX`, which is the default.
  ///  * FIXED is `min > max` — Jolt's own spelling is `FLT_MAX, -FLT_MAX` —
  ///    and drives that axis to zero.
  ///  * anything else is a limited range.
  ///
  /// A rotation limit is clamped to [-pi, pi] and, for a CONE swing, forced
  /// symmetric. That is Jolt sanitising its own input rather than asserting,
  /// so it is not an error here either — but it means what you read back from
  /// `zjoltSixDofConstraintGetLimits` may not be what you wrote.
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
// A path constraint needs a path, and a path is an object with a lifetime of
// its own — one path is normally shared by every cart on one track. It is
// therefore a second reference-counted handle rather than an array inside the
// descriptor.
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
// Every one takes the system the two bodies live in, because that is where a
// body id is resolved. The constraint is NOT added to the system by creating
// it — call zjoltConstraintAdd, which is what makes it simulate.
//
// A body id that names no body in `system` is ZJOLT_RESULT_BODY_NOT_FOUND.
// ZJOLT_BODY_ID_WORLD names the world instead, for either body but not both.
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
/// Refuses, rather than aborting, when the constraint is already in a system
/// (Jolt asserts on a second add) or when its bodies do not belong to this
/// system (Jolt would index the wrong body manager). Both are exact checks,
/// not heuristics: the constraint list is what is asked.
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

/// Throws away the impulse carried over from last step. Do this after
/// teleporting a body, so the solver does not push against a change it did
/// not make.
ZJOLT_API void zjoltConstraintResetWarmStart(ZJoltConstraint *constraint);

//===----------------------------------------------------------------------===//
// Per-kind state
//
// Each of these narrows the handle to one kind first, and reports
// ZJOLT_RESULT_INVALID_ARGUMENT when it is a different one. That check is not
// defensive tidiness: without it, asking a slider for its hinge angle is a
// cast to the wrong type and a read through it.
//
// The lambda accessors report the impulse the solver applied over the last
// step, in the constraint's own frame. They are what a breakable joint is
// built from — compare against a budget, and disable the constraint when it is
// exceeded.
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

/// Moves the attachment point on body 1. Note that the constraint keeps no
/// record of the rotation between the bodies, so moving a point does not
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

/// The same target given as an orientation of body 2 relative to body 1,
/// which is projected onto the hinge axis and then clamped as above.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetTargetOrientation(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation);

/// Newton metres resisting rotation when no motor is driving.
ZJOLT_API ZJoltResult zjoltHingeConstraintSetMaxFrictionTorque(
    ZJoltConstraint *constraint, float torque);
ZJOLT_API ZJoltResult zjoltHingeConstraintGetMaxFrictionTorque(
    const ZJoltConstraint *constraint, float *out);

ZJOLT_API ZJoltResult zjoltHingeConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
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
/// Deliberately not two setters. Jolt has one per limit, and each recomputes
/// the constraint part immediately — so moving a range upwards by setting the
/// minimum first passes through a state where the minimum exceeds the maximum,
/// which is the assertion. Setting both in one call lets the order be chosen
/// so that the intermediate state is always valid.
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
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTargetAngularVelocity(
    ZJoltConstraint *constraint, const ZJoltVec3 *angular_velocity);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTargetAngularVelocity(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

/// The target for a position motor, in constraint space. Jolt CLAMPS it to the
/// current swing and twist limits, so reading it back may differ.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintSetTargetOrientation(
    ZJoltConstraint *constraint, const ZJoltQuat *orientation);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTargetOrientation(
    const ZJoltConstraint *constraint, ZJoltQuat *out);

/// Where the joint actually is, as a rotation in constraint space. This is
/// what to compare a target against.
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetRotationInConstraintSpace(
    const ZJoltConstraint *constraint, ZJoltQuat *out);

ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaPosition(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);
ZJOLT_API ZJoltResult zjoltSwingTwistConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, ZJoltVec3 *out);

//===----------------------------------------------------------------------===//
// Six degrees of freedom
//
// Every one of these takes an axis, and every one refuses an axis outside
// ZJoltSixDofAxis. That is not pedantry: the limits, frictions and motors are
// plain arrays of six, indexed by the enumerator with no bounds check of
// Jolt's own, so a seventh axis is an out-of-bounds read or write.
//===----------------------------------------------------------------------===//

/// Metres. See ZJoltSixDofConstraintDesc::limit_min for how free, fixed and
/// limited are spelled.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetTranslationLimits(
    ZJoltConstraint *constraint, const ZJoltVec3 *min, const ZJoltVec3 *max);

/// Radians. Clamped to [-pi, pi], and forced symmetric for a CONE swing, so
/// what comes back out may not be what went in.
ZJOLT_API ZJoltResult zjoltSixDofConstraintSetRotationLimits(
    ZJoltConstraint *constraint, const ZJoltVec3 *min, const ZJoltVec3 *max);

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

/// Hands the gear the two HINGES its bodies are mounted on, so it can correct
/// positional drift instead of matching velocities only. NULL for both clears
/// them again.
///
/// Both must be hinge constraints. Jolt reads the pair during position solving
/// and asserts `false, "Unsupported"` on anything else — an abort one frame
/// later, with nothing at the stack naming this call. It is checked here
/// instead.
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
/// Note that the negative sentinel the descriptor accepts does NOT apply here:
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

/// The path this constraint follows, BORROWED — it is valid only while the
/// constraint holds it. Call zjoltPathConstraintPathAddRef to keep it.
///
/// A result rather than a plain returned pointer, because NULL would
/// otherwise mean two unrelated things: a constraint with no path set, which
/// is a normal state a caller may want to act on, and a handle that is not a
/// path constraint at all, which is a programming error. `*out` NULL is the
/// first; ZJOLT_RESULT_INVALID_ARGUMENT is the second, as for every other
/// zjoltPathConstraint* accessor.
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

ZJOLT_API ZJoltResult zjoltPathConstraintGetTotalLambdaMotor(
    const ZJoltConstraint *constraint, float *out);


#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ZJOLT_CONSTRAINT_H_
