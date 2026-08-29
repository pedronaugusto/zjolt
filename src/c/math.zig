//! ZJolt C declarations for quaternion and matrix algebra.
//!
//! Mirrors `ffi/zjolt_math.h`: a declaration belongs to the module named
//! after the header that declares it. `src/c.zig` lists every one of these
//! and is what the ABI cross-check and the misuse sweep walk. `Matrix2`/
//! `Matrix3` carry their own operations as methods, declared here rather
//! than flat, for the same reason `Vec3`/`Quat`/`Mat44` do in `core.zig`: a
//! type's methods must live where the type is declared.

const core = @import("core.zig");
const err = @import("../error.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Quat = core.Quat;
pub const Vec3 = core.Vec3;
pub const RVec3 = core.RVec3;
pub const Mat44 = core.Mat44;
pub const RMat44 = core.RMat44;
pub const AABox = core.AABox;
pub const MassProperties = core.MassProperties;
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

pub extern fn zjoltMat44Decompose(m: *const Mat44, out_rotation_translation: *Mat44, out_scale: *Vec3) void;

pub extern fn zjoltRMat44Decompose(m: *const RMat44, out_rotation_translation: *RMat44, out_scale: *Vec3) void;

pub extern fn zjoltMat44GetQuaternion(m: *const Mat44, out: *Quat) void;

pub extern fn zjoltRMat44GetQuaternion(m: *const RMat44, out: *Quat) void;

pub extern fn zjoltRVec3ToVec3RoundDown(v: *const RVec3, out: *Vec3) void;

pub extern fn zjoltRVec3ToVec3RoundUp(v: *const RVec3, out: *Vec3) void;

pub extern fn zjoltMassPropertiesDecomposePrincipalMomentsOfInertia(properties: *const MassProperties, out_rotation: *Mat44, out_diagonal: *Vec3) Result;

pub extern fn zjoltGetBaryCentricCoordinatesLine(a: *const Vec3, b: *const Vec3, out_u: *f32, out_v: *f32) bool;

pub extern fn zjoltGetBaryCentricCoordinatesTriangle(a: *const Vec3, b: *const Vec3, c: *const Vec3, out_u: *f32, out_v: *f32, out_w: *f32) bool;

pub extern fn zjoltGetClosestPointOnLine(a: *const Vec3, b: *const Vec3, out_point: *Vec3, out_set: *u32) void;

pub extern fn zjoltGetClosestPointOnTriangle(a: *const Vec3, b: *const Vec3, c: *const Vec3, out_point: *Vec3, out_set: *u32) void;

pub extern fn zjoltGetClosestPointOnTetrahedron(a: *const Vec3, b: *const Vec3, c: *const Vec3, d: *const Vec3, out_point: *Vec3, out_set: *u32) void;

pub extern fn zjoltRayTriangle(origin: *const Vec3, direction: *const Vec3, v0: *const Vec3, v1: *const Vec3, v2: *const Vec3) f32;

pub extern fn zjoltRaySphere(origin: *const Vec3, direction: *const Vec3, center: *const Vec3, radius: f32) f32;

pub extern fn zjoltRaySphereMinMax(origin: *const Vec3, direction: *const Vec3, center: *const Vec3, radius: f32, out_min_fraction: *f32, out_max_fraction: *f32) u32;

pub extern fn zjoltRayCylinder(origin: *const Vec3, direction: *const Vec3, half_height: f32, radius: f32) f32;

pub extern fn zjoltRayCapsule(origin: *const Vec3, direction: *const Vec3, half_height: f32, radius: f32) f32;

pub extern fn zjoltRayAABox(origin: *const Vec3, direction: *const Vec3, box: *const AABox) f32;

pub extern fn zjoltRayAABoxMinMax(origin: *const Vec3, direction: *const Vec3, box: *const AABox, out_min: *f32, out_max: *f32) void;

pub extern fn zjoltRayAABoxHits(origin: *const Vec3, direction: *const Vec3, box: *const AABox) bool;

pub extern fn zjoltOrientedBoxOverlapsAABox(orientation: *const Mat44, half_extents: *const Vec3, box: *const AABox, epsilon: f32) bool;

pub extern fn zjoltOrientedBoxOverlapsOrientedBox(orientation_a: *const Mat44, half_extents_a: *const Vec3, orientation_b: *const Mat44, half_extents_b: *const Vec3, epsilon: f32) bool;

// Deterministic trigonometry — Jolt/Math/Trigonometry.h.

pub extern fn zjoltSin(radians: f32) f32;
pub extern fn zjoltCos(radians: f32) f32;
pub extern fn zjoltTan(radians: f32) f32;
pub extern fn zjoltASin(x: f32) f32;
pub extern fn zjoltACos(x: f32) f32;
pub extern fn zjoltATan(x: f32) f32;
pub extern fn zjoltATan2(y: f32, x: f32) f32;
pub extern fn zjoltSinCos(radians: f32, out_sin: *f32, out_cos: *f32) void;
pub extern fn zjoltACosApproximate(x: f32) f32;

pub extern fn zjoltSinCos4(in: [*]const f32, out_sin: [*]f32, out_cos: [*]f32) void;
pub extern fn zjoltTan4(in: [*]const f32, out: [*]f32) void;
pub extern fn zjoltASin4(in: [*]const f32, out: [*]f32) void;
pub extern fn zjoltACos4(in: [*]const f32, out: [*]f32) void;
pub extern fn zjoltATan4(in: [*]const f32, out: [*]f32) void;
pub extern fn zjoltATan24(in_y: [*]const f32, in_x: [*]const f32, out: [*]f32) void;

// Dense linear algebra — Jolt/Math/Matrix.h, Jolt/Math/GaussianElimination.h.
//
// Matrix2/Matrix3 carry their operations as methods, declared here rather
// than in `../math.zig`, because a type's methods must live where the type
// is declared — the same rule `Vec3`/`Quat`/`Mat44` follow in `core.zig`.

/// A dense 2-component column vector — `Matrix2`'s column, diagonal, and
/// `Matrix2.solve`'s right-hand side and solution.
pub const Vector2 = extern struct { x: f32, y: f32 };

pub const Matrix2 = extern struct {
    m: [4]f32,

    pub fn zero() Matrix2 {
        var out: Matrix2 = undefined;
        zjoltMatrix2Zero(&out);
        return out;
    }
    pub fn identity() Matrix2 {
        var out: Matrix2 = undefined;
        zjoltMatrix2Identity(&out);
        return out;
    }
    /// The diagonal matrix with `v` on the diagonal — replaces any prior
    /// content, does not merge into an existing matrix.
    pub fn diagonal(v: Vector2) Matrix2 {
        var out: Matrix2 = undefined;
        zjoltMatrix2Diagonal(&v, &out);
        return out;
    }
    pub fn rows() u32 {
        return zjoltMatrix2GetRows();
    }
    pub fn cols() u32 {
        return zjoltMatrix2GetCols();
    }
    pub fn isIdentity(m: Matrix2) bool {
        return zjoltMatrix2IsIdentity(&m);
    }
    /// `error.InvalidArgument` when `column >= 2`.
    pub fn column(m: Matrix2, idx: u32) err.Error!Vector2 {
        var out: Vector2 = undefined;
        try err.check(zjoltMatrix2GetColumn(&m, idx, &out));
        return out;
    }
    /// `m` with column `idx` replaced by `v`. `error.InvalidArgument` when
    /// `idx >= 2`.
    pub fn withColumn(m: Matrix2, idx: u32, v: Vector2) err.Error!Matrix2 {
        var out: Matrix2 = undefined;
        try err.check(zjoltMatrix2WithColumn(&m, idx, &v, &out));
        return out;
    }
    pub fn diagonalOf(m: Matrix2) Vector2 {
        var out: Vector2 = undefined;
        zjoltMatrix2GetDiagonal(&m, &out);
        return out;
    }
    pub fn isClose(a: Matrix2, b: Matrix2, max_dist_sq: f32) bool {
        return zjoltMatrix2IsClose(&a, &b, max_dist_sq);
    }
    pub fn transposed(m: Matrix2) Matrix2 {
        var out: Matrix2 = undefined;
        zjoltMatrix2Transposed(&m, &out);
        return out;
    }
    /// `error.InvalidArgument` when `m` is singular.
    pub fn inversed(m: Matrix2) err.Error!Matrix2 {
        var out: Matrix2 = undefined;
        try err.check(zjoltMatrix2Inversed(&m, &out));
        return out;
    }
    pub fn multiply(a: Matrix2, b: Matrix2) Matrix2 {
        var out: Matrix2 = undefined;
        zjoltMatrix2Multiply(&a, &b, &out);
        return out;
    }
    pub fn multiplyVector(m: Matrix2, v: Vector2) Vector2 {
        var out: Vector2 = undefined;
        zjoltMatrix2MultiplyVector(&m, &v, &out);
        return out;
    }
    pub fn multiplyScalar(m: Matrix2, s: f32) Matrix2 {
        var out: Matrix2 = undefined;
        zjoltMatrix2MultiplyScalar(&m, s, &out);
        return out;
    }
    pub fn add(a: Matrix2, b: Matrix2) Matrix2 {
        var out: Matrix2 = undefined;
        zjoltMatrix2Add(&a, &b, &out);
        return out;
    }
    pub fn subtract(a: Matrix2, b: Matrix2) Matrix2 {
        var out: Matrix2 = undefined;
        zjoltMatrix2Subtract(&a, &b, &out);
        return out;
    }
    /// Solves `a * x = b` by Gauss-Jordan elimination with partial
    /// pivoting. `error.InvalidArgument` when `a` is singular within
    /// `tolerance` (1e-16 matches Jolt's own default).
    pub fn solve(a: Matrix2, b: Vector2, tolerance: f32) err.Error!Vector2 {
        var out: Vector2 = undefined;
        try err.check(zjoltMatrix2Solve(&a, &b, tolerance, &out));
        return out;
    }
};

pub extern fn zjoltMatrix2Zero(out: *Matrix2) void;
pub extern fn zjoltMatrix2Identity(out: *Matrix2) void;
pub extern fn zjoltMatrix2Diagonal(v: *const Vector2, out: *Matrix2) void;
pub extern fn zjoltMatrix2GetRows() u32;
pub extern fn zjoltMatrix2GetCols() u32;
pub extern fn zjoltMatrix2IsIdentity(m: *const Matrix2) bool;
pub extern fn zjoltMatrix2GetColumn(m: *const Matrix2, column: u32, out: *Vector2) Result;
pub extern fn zjoltMatrix2WithColumn(m: *const Matrix2, column: u32, v: *const Vector2, out: *Matrix2) Result;
pub extern fn zjoltMatrix2GetDiagonal(m: *const Matrix2, out: *Vector2) void;
pub extern fn zjoltMatrix2IsClose(a: *const Matrix2, b: *const Matrix2, max_dist_sq: f32) bool;
pub extern fn zjoltMatrix2Transposed(m: *const Matrix2, out: *Matrix2) void;
pub extern fn zjoltMatrix2Inversed(m: *const Matrix2, out: *Matrix2) Result;
pub extern fn zjoltMatrix2Multiply(a: *const Matrix2, b: *const Matrix2, out: *Matrix2) void;
pub extern fn zjoltMatrix2MultiplyVector(m: *const Matrix2, v: *const Vector2, out: *Vector2) void;
pub extern fn zjoltMatrix2MultiplyScalar(m: *const Matrix2, s: f32, out: *Matrix2) void;
pub extern fn zjoltMatrix2Add(a: *const Matrix2, b: *const Matrix2, out: *Matrix2) void;
pub extern fn zjoltMatrix2Subtract(a: *const Matrix2, b: *const Matrix2, out: *Matrix2) void;
pub extern fn zjoltMatrix2Solve(a: *const Matrix2, b: *const Vector2, tolerance: f32, out_x: *Vector2) Result;

pub extern fn zjoltMatrix3Zero(out: *Matrix3) void;
pub extern fn zjoltMatrix3Identity(out: *Matrix3) void;
pub extern fn zjoltMatrix3Diagonal(v: *const Vec3, out: *Matrix3) void;
pub extern fn zjoltMatrix3GetRows() u32;
pub extern fn zjoltMatrix3GetCols() u32;
pub extern fn zjoltMatrix3IsIdentity(m: *const Matrix3) bool;
pub extern fn zjoltMatrix3GetColumn(m: *const Matrix3, column: u32, out: *Vec3) Result;
pub extern fn zjoltMatrix3WithColumn(m: *const Matrix3, column: u32, v: *const Vec3, out: *Matrix3) Result;
pub extern fn zjoltMatrix3GetDiagonal(m: *const Matrix3, out: *Vec3) void;
pub extern fn zjoltMatrix3IsClose(a: *const Matrix3, b: *const Matrix3, max_dist_sq: f32) bool;
pub extern fn zjoltMatrix3Transposed(m: *const Matrix3, out: *Matrix3) void;
pub extern fn zjoltMatrix3Inversed(m: *const Matrix3, out: *Matrix3) Result;
pub extern fn zjoltMatrix3Multiply(a: *const Matrix3, b: *const Matrix3, out: *Matrix3) void;
pub extern fn zjoltMatrix3MultiplyVector(m: *const Matrix3, v: *const Vec3, out: *Vec3) void;
pub extern fn zjoltMatrix3MultiplyScalar(m: *const Matrix3, s: f32, out: *Matrix3) void;
pub extern fn zjoltMatrix3Add(a: *const Matrix3, b: *const Matrix3, out: *Matrix3) void;
pub extern fn zjoltMatrix3Subtract(a: *const Matrix3, b: *const Matrix3, out: *Matrix3) void;
pub extern fn zjoltMatrix3Solve(a: *const Matrix3, b: *const Vec3, tolerance: f32, out_x: *Vec3) Result;

/// As `Matrix2`, over `JPH::Matrix<3, 3>`. Column, diagonal and solve
/// vectors are `Vec3` (also declared in `core.zig`): both are three bare
/// floats, and `MassProperties.decomposePrincipalMomentsOfInertia` already
/// crosses this ABI's `Vec3` for the equivalent 3x3-adjacent quantity.
pub const Matrix3 = extern struct {
    m: [9]f32,

    pub fn zero() Matrix3 {
        var out: Matrix3 = undefined;
        zjoltMatrix3Zero(&out);
        return out;
    }
    pub fn identity() Matrix3 {
        var out: Matrix3 = undefined;
        zjoltMatrix3Identity(&out);
        return out;
    }
    pub fn diagonal(v: Vec3) Matrix3 {
        var out: Matrix3 = undefined;
        zjoltMatrix3Diagonal(&v, &out);
        return out;
    }
    pub fn rows() u32 {
        return zjoltMatrix3GetRows();
    }
    pub fn cols() u32 {
        return zjoltMatrix3GetCols();
    }
    pub fn isIdentity(m: Matrix3) bool {
        return zjoltMatrix3IsIdentity(&m);
    }
    /// `error.InvalidArgument` when `idx >= 3`.
    pub fn column(m: Matrix3, idx: u32) err.Error!Vec3 {
        var out: Vec3 = undefined;
        try err.check(zjoltMatrix3GetColumn(&m, idx, &out));
        return out;
    }
    /// `error.InvalidArgument` when `idx >= 3`.
    pub fn withColumn(m: Matrix3, idx: u32, v: Vec3) err.Error!Matrix3 {
        var out: Matrix3 = undefined;
        try err.check(zjoltMatrix3WithColumn(&m, idx, &v, &out));
        return out;
    }
    pub fn diagonalOf(m: Matrix3) Vec3 {
        var out: Vec3 = undefined;
        zjoltMatrix3GetDiagonal(&m, &out);
        return out;
    }
    pub fn isClose(a: Matrix3, b: Matrix3, max_dist_sq: f32) bool {
        return zjoltMatrix3IsClose(&a, &b, max_dist_sq);
    }
    pub fn transposed(m: Matrix3) Matrix3 {
        var out: Matrix3 = undefined;
        zjoltMatrix3Transposed(&m, &out);
        return out;
    }
    /// `error.InvalidArgument` when `m` is singular. Unlike `Matrix2.
    /// inversed`, this genuinely runs GaussianElimination: `Matrix<3,3>`
    /// has no closed-form specialization.
    pub fn inversed(m: Matrix3) err.Error!Matrix3 {
        var out: Matrix3 = undefined;
        try err.check(zjoltMatrix3Inversed(&m, &out));
        return out;
    }
    pub fn multiply(a: Matrix3, b: Matrix3) Matrix3 {
        var out: Matrix3 = undefined;
        zjoltMatrix3Multiply(&a, &b, &out);
        return out;
    }
    pub fn multiplyVector(m: Matrix3, v: Vec3) Vec3 {
        var out: Vec3 = undefined;
        zjoltMatrix3MultiplyVector(&m, &v, &out);
        return out;
    }
    pub fn multiplyScalar(m: Matrix3, s: f32) Matrix3 {
        var out: Matrix3 = undefined;
        zjoltMatrix3MultiplyScalar(&m, s, &out);
        return out;
    }
    pub fn add(a: Matrix3, b: Matrix3) Matrix3 {
        var out: Matrix3 = undefined;
        zjoltMatrix3Add(&a, &b, &out);
        return out;
    }
    pub fn subtract(a: Matrix3, b: Matrix3) Matrix3 {
        var out: Matrix3 = undefined;
        zjoltMatrix3Subtract(&a, &b, &out);
        return out;
    }
    /// As `Matrix2.solve`, over `Matrix<3, 3>`.
    pub fn solve(a: Matrix3, b: Vec3, tolerance: f32) err.Error!Vec3 {
        var out: Vec3 = undefined;
        try err.check(zjoltMatrix3Solve(&a, &b, tolerance, &out));
        return out;
    }
};

pub extern fn zjoltRMat44ToMat44(m: *const RMat44, out: *Mat44) void;

// Half floats — Jolt/Math/HalfFloat.h.

pub const HalfFloat = u16;

pub const HalfFloatRoundingMode = enum(i32) {
    round_to_neg_inf = 0,
    round_to_pos_inf = 1,
    round_to_nearest = 2,
};

pub extern fn zjoltHalfFloatFromFloat(value: f32, mode: HalfFloatRoundingMode) HalfFloat;
pub extern fn zjoltHalfFloatFromFloatFallback(value: f32, mode: HalfFloatRoundingMode) HalfFloat;
pub extern fn zjoltHalfFloatToFloat4(in: [*]const HalfFloat, out: [*]f32) void;
pub extern fn zjoltHalfFloatToFloatFallback4(in: [*]const HalfFloat, out: [*]f32) void;

// The rest of Jolt/Math/: root finding and angle wrapping.

pub extern fn zjoltFindRoot(a: f32, b: f32, c: f32, out_x1: *f32, out_x2: *f32) u32;
pub extern fn zjoltCenterAngleAroundZero(radians: f32) f32;
