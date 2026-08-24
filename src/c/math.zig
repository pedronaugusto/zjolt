//! ZJolt C declarations for quaternion and matrix algebra.
//!
//! Mirrors `ffi/zjolt_math.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Quat = core.Quat;
pub const Vec3 = core.Vec3;
pub const RVec3 = core.RVec3;
pub const Mat44 = core.Mat44;
pub const RMat44 = core.RMat44;
pub const Result = core.Result;

pub extern fn zjoltQuatMultiply(lhs: *const Quat, rhs: *const Quat, out: *Quat) void;

pub extern fn zjoltQuatRotateVector(q: *const Quat, v: *const Vec3, out: *Vec3) Result;

pub extern fn zjoltQuatInverse(q: *const Quat, out: *Quat) void;

pub extern fn zjoltQuatConjugate(q: *const Quat, out: *Quat) void;

pub extern fn zjoltQuatDot(a: *const Quat, b: *const Quat) f32;

pub extern fn zjoltQuatIsNormalized(q: *const Quat, tolerance: f32) bool;

pub extern fn zjoltQuatNormalize(q: *const Quat, out: *Quat) void;

pub extern fn zjoltQuatFromAxisAngle(axis: *const Vec3, radians: f32, out: *Quat) Result;

pub extern fn zjoltQuatGetAxisAngle(q: *const Quat, out_axis: *Vec3, out_angle: *f32) Result;

pub extern fn zjoltQuatFromTo(from: *const Vec3, to: *const Vec3, out: *Quat) void;

pub extern fn zjoltQuatFromEulerAngles(angles_radians: *const Vec3, out: *Quat) void;

pub extern fn zjoltQuatGetEulerAngles(q: *const Quat, out: *Vec3) void;

pub extern fn zjoltQuatGetPerpendicular(q: *const Quat, out: *Quat) void;

pub extern fn zjoltQuatGetRotationAngle(q: *const Quat, axis: *const Vec3) f32;

pub extern fn zjoltQuatGetTwist(q: *const Quat, axis: *const Vec3, out: *Quat) void;

pub extern fn zjoltQuatGetSwingTwist(q: *const Quat, out_swing: *Quat, out_twist: *Quat) void;

pub extern fn zjoltQuatLerp(a: *const Quat, b: *const Quat, t: f32, out: *Quat) void;

pub extern fn zjoltQuatSlerp(a: *const Quat, b: *const Quat, t: f32, out: *Quat) void;

pub extern fn zjoltVec3Lerp(a: *const Vec3, b: *const Vec3, t: f32, out: *Vec3) void;

pub extern fn zjoltRVec3Lerp(a: *const RVec3, b: *const RVec3, t: f32, out: *RVec3) void;

pub extern fn zjoltMat44FromRotationTranslation(rotation: *const Quat, translation: *const Vec3, out: *Mat44) Result;

pub extern fn zjoltMat44Multiply(a: *const Mat44, b: *const Mat44, out: *Mat44) void;

pub extern fn zjoltMat44Inverse(m: *const Mat44, out: *Mat44) void;

pub extern fn zjoltMat44InverseRotationTranslation(m: *const Mat44, out: *Mat44) void;

pub extern fn zjoltMat44TransformPoint(m: *const Mat44, point: *const Vec3, out: *Vec3) void;

pub extern fn zjoltMat44TransformDirection(m: *const Mat44, direction: *const Vec3, out: *Vec3) void;

pub extern fn zjoltRMat44FromRotationTranslation(rotation: *const Quat, translation: *const RVec3, out: *RMat44) Result;

pub extern fn zjoltRMat44Multiply(a: *const RMat44, b: *const RMat44, out: *RMat44) void;

pub extern fn zjoltRMat44Inverse(m: *const RMat44, out: *RMat44) void;

pub extern fn zjoltRMat44InverseRotationTranslation(m: *const RMat44, out: *RMat44) void;

pub extern fn zjoltRMat44TransformPoint(m: *const RMat44, point: *const RVec3, out: *RVec3) void;

pub extern fn zjoltRMat44TransformDirection(m: *const RMat44, direction: *const Vec3, out: *Vec3) void;
