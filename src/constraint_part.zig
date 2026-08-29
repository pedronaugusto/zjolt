//! A faithful Zig port of every type in
//! `libs/JoltPhysics/Jolt/Physics/Constraints/ConstraintPart/` — what Jolt's
//! own constraints are built from. `Constraint.initCustom`
//! (`src/constraint.zig`) is the callback seam; this runs inside it as
//! plain Zig arithmetic, not FFI.
//!
//! `Body&` becomes `*const SolverBody` / `*SolverBodyPair`
//! (`ffi/zjolt_constraint.h`); rotation-taking signatures drop that
//! parameter since `SolverBody.inverse_inertia` is already
//! `GetInverseInertiaForRotation`'s result. `PointConstraintPart` is the
//! exception: it still rotates local-space points itself via
//! `body.rotation` (Rodrigues, `qrotate` below). `is_dynamic` gates only
//! inertia terms and `ApplyVelocityStep`'s writes; velocity-Jacobian reads
//! omit the guard since a static body's velocity is already zero.

const std = @import("std");
const constraint_mod = @import("constraint.zig");
const c = @import("c/constraint.zig");

pub const SolverBody = constraint_mod.SolverBody;
pub const SolverBodyPair = constraint_mod.SolverBodyPair;
pub const StateRecorder = c.StateRecorder;
pub const SpringSettings = constraint_mod.SpringSettings;

const Vec3 = c.Vec3;
const Quat = c.Quat;

//=============================================================================
// Pure-Zig vector / quaternion arithmetic
//
// No crossing back into the C library — see the file doc comment.
//=============================================================================

fn v3(x: f32, y: f32, z: f32) Vec3 {
    return .{ .x = x, .y = y, .z = z };
}
const vec3_zero: Vec3 = .{ .x = 0, .y = 0, .z = 0 };
const quat_identity: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 };

fn add3(a: Vec3, b: Vec3) Vec3 {
    return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
fn sub3(a: Vec3, b: Vec3) Vec3 {
    return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
fn neg3(a: Vec3) Vec3 {
    return v3(-a.x, -a.y, -a.z);
}
fn scale3(a: Vec3, s: f32) Vec3 {
    return v3(a.x * s, a.y * s, a.z * s);
}
fn dot3(a: Vec3, b: Vec3) f32 {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
fn cross3(a: Vec3, b: Vec3) Vec3 {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
fn lengthSq3(a: Vec3) f32 {
    return dot3(a, a);
}
fn isZero3(a: Vec3) bool {
    return a.x == 0 and a.y == 0 and a.z == 0;
}

/// Mirrors `Vec3::GetNormalizedPerpendicular`'s scalar fallback exactly,
/// including which of the two candidate axes it picks.
fn getNormalizedPerpendicular3(a: Vec3) Vec3 {
    const xx = a.x * a.x;
    const yy = a.y * a.y;
    const zz = a.z * a.z;
    const perp = if (xx > yy) v3(a.z, 0, -a.x) else v3(0, a.z, -a.y);
    return scale3(perp, 1.0 / @sqrt(@max(xx, yy) + zz));
}

/// Hamilton product, `p * q`, `w` last. Transcribed from `Quat::operator*`'s
/// scalar fallback, difference-of-products grouping and all.
fn qmul(p: Quat, q: Quat) Quat {
    return .{
        .x = (p.x * q.w + p.y * q.z) + (p.w * q.x - p.z * q.y),
        .y = (p.y * q.w + p.z * q.x) + (p.w * q.y - p.x * q.z),
        .z = (p.z * q.w + p.x * q.y) + (p.w * q.z - p.y * q.x),
        .w = -(p.x * q.x + p.y * q.y) + (p.w * q.w - p.z * q.z),
    };
}
fn qconj(q: Quat) Quat {
    return .{ .x = -q.x, .y = -q.y, .z = -q.z, .w = q.w };
}
fn qneg(q: Quat) Quat {
    return .{ .x = -q.x, .y = -q.y, .z = -q.z, .w = -q.w };
}
fn qxyz(q: Quat) Vec3 {
    return v3(q.x, q.y, q.z);
}
fn qEnsureWPositive(q: Quat) Quat {
    return if (q.w < 0) qneg(q) else q;
}
fn qLengthSq(q: Quat) f32 {
    return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}
fn qNormalized(q: Quat) Quat {
    const inv_len = 1.0 / @sqrt(qLengthSq(q));
    return .{ .x = q.x * inv_len, .y = q.y * inv_len, .z = q.z * inv_len, .w = q.w * inv_len };
}

/// `q * v`, Rodrigues form — the formula `Quat::operator*(Vec3Arg)`'s own
/// comment gives as what the SIMD swizzles below it compute:
/// `p' = p + 2 * (q.w * q.xyz x p + q.xyz x (q.xyz x p))`.
fn qrotate(q: Quat, v: Vec3) Vec3 {
    const xyz = qxyz(q);
    const t = cross3(xyz, v);
    const t2 = cross3(xyz, t);
    return add3(v, scale3(add3(scale3(t, q.w), t2), 2.0));
}
fn qRotateAxisX(q: Quat) Vec3 {
    return qrotate(q, v3(1, 0, 0));
}
fn qRotateAxisY(q: Quat) Vec3 {
    return qrotate(q, v3(0, 1, 0));
}
fn qRotateAxisZ(q: Quat) Vec3 {
    return qrotate(q, v3(0, 0, 1));
}

/// `Quat::GetSwingTwist`: decomposes `q` around its own local X axis.
pub fn quatGetSwingTwist(q: Quat) struct { swing: Quat, twist: Quat } {
    const s = @sqrt(q.w * q.w + q.x * q.x);
    if (s != 0.0) {
        return .{
            .twist = .{ .x = q.x / s, .y = 0, .z = 0, .w = q.w / s },
            .swing = .{ .x = 0, .y = (q.w * q.y - q.x * q.z) / s, .z = (q.w * q.z + q.x * q.y) / s, .w = s },
        };
    }
    return .{ .twist = quat_identity, .swing = q };
}

//=============================================================================
// Pure-Zig 3x3 matrix arithmetic, and the 2-vector / 2x2 matrix the hinge and
// dual-axis parts need. Rows, not columns — `r0`/`r1`/`r2` are the matrix's
// three rows, so `mat3MulVec` is a plain dot product per row.
//=============================================================================

const Mat3 = struct {
    r0: Vec3 = vec3_zero,
    r1: Vec3 = vec3_zero,
    r2: Vec3 = vec3_zero,
};
const mat3_zero: Mat3 = .{};

fn mat3IsZero(m: Mat3) bool {
    return isZero3(m.r0) and isZero3(m.r1) and isZero3(m.r2);
}
fn mat3MulVec(m: Mat3, v: Vec3) Vec3 {
    return v3(dot3(m.r0, v), dot3(m.r1, v), dot3(m.r2, v));
}
fn mat3Col(m: Mat3, comptime j: usize) Vec3 {
    return switch (j) {
        0 => v3(m.r0.x, m.r1.x, m.r2.x),
        1 => v3(m.r0.y, m.r1.y, m.r2.y),
        else => v3(m.r0.z, m.r1.z, m.r2.z),
    };
}
fn mat3Add(a: Mat3, b: Mat3) Mat3 {
    return .{ .r0 = add3(a.r0, b.r0), .r1 = add3(a.r1, b.r1), .r2 = add3(a.r2, b.r2) };
}
/// `a * b^T` — `Mat44::Multiply3x3RightTransposed`.
fn mat3MulTransposeRight(a: Mat3, b: Mat3) Mat3 {
    return .{
        .r0 = v3(dot3(a.r0, b.r0), dot3(a.r0, b.r1), dot3(a.r0, b.r2)),
        .r1 = v3(dot3(a.r1, b.r0), dot3(a.r1, b.r1), dot3(a.r1, b.r2)),
        .r2 = v3(dot3(a.r2, b.r0), dot3(a.r2, b.r1), dot3(a.r2, b.r2)),
    };
}
/// `a * b`, both taken as 3x3.
fn mat3Mul(a: Mat3, b: Mat3) Mat3 {
    const c0 = mat3Col(b, 0);
    const c1 = mat3Col(b, 1);
    const c2 = mat3Col(b, 2);
    return .{
        .r0 = v3(dot3(a.r0, c0), dot3(a.r0, c1), dot3(a.r0, c2)),
        .r1 = v3(dot3(a.r1, c0), dot3(a.r1, c1), dot3(a.r1, c2)),
        .r2 = v3(dot3(a.r2, c0), dot3(a.r2, c1), dot3(a.r2, c2)),
    };
}
/// The skew-symmetric matrix `S` such that `S * u == cross3(v, u)` —
/// `Mat44::sCrossProduct`.
fn mat3CrossProduct(v: Vec3) Mat3 {
    return .{
        .r0 = v3(0, -v.z, v.y),
        .r1 = v3(v.z, 0, -v.x),
        .r2 = v3(-v.y, v.x, 0),
    };
}
/// `Mat44::SetInversed3x3`: the adjugate divided by the determinant, `null`
/// exactly where Jolt's own hard `det == 0.0f` check returns `false`.
fn mat3Invert(m: Mat3) ?Mat3 {
    const m00 = m.r0.x;
    const m01 = m.r0.y;
    const m02 = m.r0.z;
    const m10 = m.r1.x;
    const m11 = m.r1.y;
    const m12 = m.r1.z;
    const m20 = m.r2.x;
    const m21 = m.r2.y;
    const m22 = m.r2.z;

    const det = m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20);
    if (det == 0.0) return null;
    const inv_det = 1.0 / det;

    return .{
        .r0 = v3((m11 * m22 - m12 * m21) * inv_det, (m02 * m21 - m01 * m22) * inv_det, (m01 * m12 - m02 * m11) * inv_det),
        .r1 = v3((m12 * m20 - m10 * m22) * inv_det, (m00 * m22 - m02 * m20) * inv_det, (m02 * m10 - m00 * m12) * inv_det),
        .r2 = v3((m10 * m21 - m11 * m20) * inv_det, (m01 * m20 - m00 * m21) * inv_det, (m00 * m11 - m01 * m10) * inv_det),
    };
}
/// `arr` is `SolverBody.inverse_inertia`: world space, row major.
fn mat3FromRowMajor9(arr: [9]f32) Mat3 {
    return .{
        .r0 = v3(arr[0], arr[1], arr[2]),
        .r1 = v3(arr[3], arr[4], arr[5]),
        .r2 = v3(arr[6], arr[7], arr[8]),
    };
}

/// `MotionProperties::MultiplyWorldSpaceInverseInertiaByVector`. The rotation
/// already happened — see the file doc comment — so this is the plain
/// multiply that call reduces to, zero for a body that is not dynamic.
pub fn multiplyWorldSpaceInverseInertiaByVector(body: *const SolverBody, v: Vec3) Vec3 {
    if (!body.is_dynamic) return vec3_zero;
    return mat3MulVec(mat3FromRowMajor9(body.inverse_inertia), v);
}

const Vec2 = struct { x: f32 = 0, y: f32 = 0 };
const vec2_zero: Vec2 = .{};
fn v2IsZero(v: Vec2) bool {
    return v.x == 0 and v.y == 0;
}

const Mat22 = struct { m00: f32 = 0, m01: f32 = 0, m10: f32 = 0, m11: f32 = 0 };
const mat22_zero: Mat22 = .{};
fn mat22IsZero(m: Mat22) bool {
    return m.m00 == 0 and m.m01 == 0 and m.m10 == 0 and m.m11 == 0;
}
fn mat22MulVec(m: Mat22, v: Vec2) Vec2 {
    return .{ .x = m.m00 * v.x + m.m01 * v.y, .y = m.m10 * v.x + m.m11 * v.y };
}
fn mat22Invert(m: Mat22) ?Mat22 {
    const det = m.m00 * m.m11 - m.m01 * m.m10;
    if (det == 0.0) return null;
    const inv_det = 1.0 / det;
    return .{ .m00 = m.m11 * inv_det, .m01 = -m.m01 * inv_det, .m10 = -m.m10 * inv_det, .m11 = m.m00 * inv_det };
}

//=============================================================================
// State recorder helpers — every part's SaveState/RestoreState write and read
// exactly the raw bytes of their POD state, matching `StateRecorder::Write`
// and `::Read`'s own byte-for-byte contract.
//=============================================================================

fn recWrite(recorder: *StateRecorder, comptime T: type, value: T) void {
    var v = value;
    c.zjoltStateRecorderWriteBytes(recorder, &v, @sizeOf(T));
}
fn recRead(recorder: *StateRecorder, comptime T: type) T {
    var v: T = undefined;
    c.zjoltStateRecorderReadBytes(recorder, &v, @sizeOf(T));
    return v;
}

//=============================================================================
// SpringPart
//=============================================================================

/// Calculates the bias term a constraint part's Lagrange multiplier needs to
/// behave as a spring instead of a hard limit. `libs/.../SpringPart.h`.
pub const SpringPart = struct {
    bias: f32 = 0,
    softness: f32 = 0,

    pub fn calculateSpringPropertiesWithBias(self: *SpringPart, bias: f32) void {
        self.softness = 0.0;
        self.bias = bias;
    }

    /// Returns the new effective mass. `inStiffness > 0 or inDamping > 0` is
    /// Jolt's own precondition; not re-checked here for the same reason nothing
    /// else in this file re-checks a precondition its caller already owns.
    pub fn calculateSpringPropertiesWithStiffnessAndDamping(
        self: *SpringPart,
        delta_time: f32,
        inv_effective_mass: f32,
        bias: f32,
        cc: f32,
        stiffness: f32,
        damping: f32,
    ) f32 {
        self.softness = 1.0 / (delta_time * (damping + delta_time * stiffness));
        self.bias = bias + delta_time * stiffness * self.softness * cc;
        return 1.0 / (inv_effective_mass + self.softness);
    }

    pub fn calculateSpringPropertiesWithFrequencyAndDamping(
        self: *SpringPart,
        delta_time: f32,
        inv_effective_mass: f32,
        bias: f32,
        cc: f32,
        frequency: f32,
        damping: f32,
    ) f32 {
        var effective_mass = 1.0 / inv_effective_mass;
        const omega = 2.0 * std.math.pi * frequency;
        const k = effective_mass * (omega * omega);
        const cd = 2.0 * effective_mass * damping * omega;
        effective_mass = self.calculateSpringPropertiesWithStiffnessAndDamping(delta_time, inv_effective_mass, bias, cc, k, cd);
        return effective_mass;
    }

    pub fn calculateSpringPropertiesWithMassNormalizedStiffnessAndDamping(
        self: *SpringPart,
        delta_time: f32,
        inv_effective_mass: f32,
        bias: f32,
        cc: f32,
        stiffness: f32,
        damping: f32,
    ) f32 {
        return self.calculateSpringPropertiesWithStiffnessAndDamping(
            delta_time,
            inv_effective_mass,
            bias,
            cc,
            stiffness / inv_effective_mass,
            damping / inv_effective_mass,
        );
    }

    /// Assumes the spring has either stiffness or damping.
    pub fn calculateSpringPropertiesWithSettings(
        self: *SpringPart,
        delta_time: f32,
        inv_effective_mass: f32,
        bias: f32,
        cc: f32,
        settings: SpringSettings,
    ) f32 {
        return switch (settings.mode) {
            .frequency_and_damping => self.calculateSpringPropertiesWithFrequencyAndDamping(delta_time, inv_effective_mass, bias, cc, settings.frequency_or_stiffness, settings.damping),
            .stiffness_and_damping => self.calculateSpringPropertiesWithStiffnessAndDamping(delta_time, inv_effective_mass, bias, cc, settings.frequency_or_stiffness, settings.damping),
            .mass_normalized_stiffness_and_damping => self.calculateSpringPropertiesWithMassNormalizedStiffnessAndDamping(delta_time, inv_effective_mass, bias, cc, settings.frequency_or_stiffness, settings.damping),
        };
    }

    pub fn isActive(self: SpringPart) bool {
        return self.softness != 0.0;
    }

    /// `softness * inTotalLambda + bias` — the total bias `lambda = J v + b`
    /// wants, spring term and caller-supplied bias combined.
    pub fn getBias(self: SpringPart, total_lambda: f32) f32 {
        return self.softness * total_lambda + self.bias;
    }
};

fn springHasStiffness(s: SpringSettings) bool {
    return s.frequency_or_stiffness > 0.0;
}
fn springHasStiffnessOrDamping(s: SpringSettings) bool {
    return s.frequency_or_stiffness > 0.0 or (s.mode != .frequency_and_damping and s.damping > 0.0);
}

//=============================================================================
// PointConstraintPart
//=============================================================================

/// Constrains movement along all 3 axes — a ball joint's position half.
/// `PointConstraintPart.h`.
///
/// The one part that still takes LOCAL-space attachment points — see the
/// file doc comment for why.
pub const PointConstraintPart = struct {
    r1: Vec3 = vec3_zero,
    r2: Vec3 = vec3_zero,
    inv_i1_r1x: Mat3 = mat3_zero,
    inv_i2_r2x: Mat3 = mat3_zero,
    effective_mass: Mat3 = mat3_zero,
    total_lambda: Vec3 = vec3_zero,
    /// Substitutes for Jolt's `mEffectiveMass(3,3) != 0` sentinel, which has
    /// no equivalent in a plain 3x3 — same lifecycle: true after a
    /// successful `calculateConstraintProperties`, false after `deactivate`.
    active: bool = false,

    fn applyVelocityStep(self: *const PointConstraintPart, bodies: *SolverBodyPair, lambda: Vec3) bool {
        if (!isZero3(lambda)) {
            if (bodies.body1.is_dynamic) {
                bodies.body1.linear_velocity = sub3(bodies.body1.linear_velocity, scale3(lambda, bodies.body1.inverse_mass));
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, mat3MulVec(self.inv_i1_r1x, lambda));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.linear_velocity = add3(bodies.body2.linear_velocity, scale3(lambda, bodies.body2.inverse_mass));
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, mat3MulVec(self.inv_i2_r2x, lambda));
            }
            return true;
        }
        return false;
    }

    /// `r1_local` / `r2_local`: local-space vector from centre of mass to the
    /// constraint point, on body 1 and body 2 respectively.
    pub fn calculateConstraintProperties(self: *PointConstraintPart, body1: *const SolverBody, r1_local: Vec3, body2: *const SolverBody, r2_local: Vec3) void {
        self.r1 = qrotate(body1.rotation, r1_local);
        self.r2 = qrotate(body2.rotation, r2_local);

        var summed_inv_mass: f32 = 0;
        var inv_effective_mass = mat3_zero;

        if (body1.is_dynamic) {
            const r1x = mat3CrossProduct(self.r1);
            self.inv_i1_r1x = mat3MulVec3x3(mat3FromRowMajor9(body1.inverse_inertia), r1x);
            summed_inv_mass = body1.inverse_mass;
            inv_effective_mass = mat3MulTransposeRight(mat3Mul(r1x, mat3FromRowMajor9(body1.inverse_inertia)), r1x);
        } else {
            self.inv_i1_r1x = mat3_zero;
        }

        if (body2.is_dynamic) {
            const r2x = mat3CrossProduct(self.r2);
            self.inv_i2_r2x = mat3MulVec3x3(mat3FromRowMajor9(body2.inverse_inertia), r2x);
            summed_inv_mass += body2.inverse_mass;
            inv_effective_mass = mat3Add(inv_effective_mass, mat3MulTransposeRight(mat3Mul(r2x, mat3FromRowMajor9(body2.inverse_inertia)), r2x));
        } else {
            self.inv_i2_r2x = mat3_zero;
        }

        inv_effective_mass = mat3Add(inv_effective_mass, mat3Scale(summed_inv_mass));

        if (mat3Invert(inv_effective_mass)) |inv| {
            self.effective_mass = inv;
            self.active = true;
        } else {
            self.deactivate();
        }
    }

    pub fn deactivate(self: *PointConstraintPart) void {
        self.effective_mass = mat3_zero;
        self.total_lambda = vec3_zero;
        self.active = false;
    }

    pub fn isActive(self: PointConstraintPart) bool {
        return self.active;
    }

    pub fn warmStart(self: *PointConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) void {
        self.total_lambda = scale3(self.total_lambda, warm_start_impulse_ratio);
        _ = self.applyVelocityStep(bodies, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *PointConstraintPart, bodies: *SolverBodyPair) bool {
        const lambda = mat3MulVec(self.effective_mass, add3(sub3(bodies.body1.linear_velocity, cross3(self.r1, bodies.body1.angular_velocity)), sub3(cross3(self.r2, bodies.body2.angular_velocity), bodies.body2.linear_velocity)));
        self.total_lambda = add3(self.total_lambda, lambda);
        return self.applyVelocityStep(bodies, lambda);
    }

    pub fn solvePositionConstraint(self: *const PointConstraintPart, bodies: *SolverBodyPair, baumgarte: f32) bool {
        const separation = add3(sub3(bodies.body2.center_of_mass, bodies.body1.center_of_mass), sub3(self.r2, self.r1));
        if (!isZero3(separation)) {
            const lambda = mat3MulVec(self.effective_mass, scale3(separation, -baumgarte));
            if (bodies.body1.is_dynamic) {
                bodies.body1.position_delta = sub3(bodies.body1.position_delta, scale3(lambda, bodies.body1.inverse_mass));
                bodies.body1.rotation_delta = sub3(bodies.body1.rotation_delta, mat3MulVec(self.inv_i1_r1x, lambda));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.position_delta = add3(bodies.body2.position_delta, scale3(lambda, bodies.body2.inverse_mass));
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, mat3MulVec(self.inv_i2_r2x, lambda));
            }
            return true;
        }
        return false;
    }

    pub fn getTotalLambda(self: PointConstraintPart) Vec3 {
        return self.total_lambda;
    }

    pub fn saveState(self: PointConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, Vec3, self.total_lambda);
    }
    pub fn restoreState(self: *PointConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, Vec3);
    }
};

/// `inv_i.Multiply3x3(r_cross)` — `Mat44::Multiply3x3`, `a * b` restricted to
/// the 3x3 block.
fn mat3MulVec3x3(a: Mat3, b: Mat3) Mat3 {
    return mat3Mul(a, b);
}
fn mat3Scale(s: f32) Mat3 {
    return .{ .r0 = v3(s, 0, 0), .r1 = v3(0, s, 0), .r2 = v3(0, 0, s) };
}

//=============================================================================
// AxisConstraintPart
//=============================================================================

/// Constrains motion along 1 axis, with an optional spring.
/// `AxisConstraintPart.h`.
pub const AxisConstraintPart = struct {
    r1_plus_u_x_axis: Vec3 = vec3_zero,
    r2_x_axis: Vec3 = vec3_zero,
    inv_i1_r1_plus_u_x_axis: Vec3 = vec3_zero,
    inv_i2_r2x_axis: Vec3 = vec3_zero,
    effective_mass: f32 = 0,
    spring: SpringPart = .{},
    total_lambda: f32 = 0,

    fn applyVelocityStep(self: *const AxisConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, lambda: f32) bool {
        if (lambda != 0.0) {
            if (bodies.body1.is_dynamic) {
                bodies.body1.linear_velocity = sub3(bodies.body1.linear_velocity, scale3(world_axis, lambda * bodies.body1.inverse_mass));
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, scale3(self.inv_i1_r1_plus_u_x_axis, lambda));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.linear_velocity = add3(bodies.body2.linear_velocity, scale3(world_axis, lambda * bodies.body2.inverse_mass));
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, scale3(self.inv_i2_r2x_axis, lambda));
            }
            return true;
        }
        return false;
    }

    fn calculateInverseEffectiveMass(self: *AxisConstraintPart, body1: *const SolverBody, r1_plus_u: Vec3, body2: *const SolverBody, r2: Vec3, world_axis: Vec3) f32 {
        var inv_effective_mass: f32 = 0;

        self.r1_plus_u_x_axis = cross3(r1_plus_u, world_axis);
        if (body1.is_dynamic) {
            self.inv_i1_r1_plus_u_x_axis = multiplyWorldSpaceInverseInertiaByVector(body1, self.r1_plus_u_x_axis);
            inv_effective_mass += body1.inverse_mass + dot3(self.inv_i1_r1_plus_u_x_axis, self.r1_plus_u_x_axis);
        } else {
            self.inv_i1_r1_plus_u_x_axis = vec3_zero;
        }

        self.r2_x_axis = cross3(r2, world_axis);
        if (body2.is_dynamic) {
            self.inv_i2_r2x_axis = multiplyWorldSpaceInverseInertiaByVector(body2, self.r2_x_axis);
            inv_effective_mass += body2.inverse_mass + dot3(self.inv_i2_r2x_axis, self.r2_x_axis);
        } else {
            self.inv_i2_r2x_axis = vec3_zero;
        }

        return inv_effective_mass;
    }

    pub fn calculateConstraintProperties(self: *AxisConstraintPart, body1: *const SolverBody, r1_plus_u: Vec3, body2: *const SolverBody, r2: Vec3, world_axis: Vec3, bias: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring.calculateSpringPropertiesWithBias(bias);
        }
    }

    pub fn calculateConstraintPropertiesWithFrequencyAndDamping(self: *AxisConstraintPart, delta_time: f32, body1: *const SolverBody, r1_plus_u: Vec3, body2: *const SolverBody, r2: Vec3, world_axis: Vec3, bias: f32, cc: f32, frequency: f32, damping: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else if (frequency > 0.0) {
            self.effective_mass = self.spring.calculateSpringPropertiesWithFrequencyAndDamping(delta_time, inv_effective_mass, bias, cc, frequency, damping);
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring.calculateSpringPropertiesWithBias(bias);
        }
    }

    pub fn calculateConstraintPropertiesWithStiffnessAndDamping(self: *AxisConstraintPart, delta_time: f32, body1: *const SolverBody, r1_plus_u: Vec3, body2: *const SolverBody, r2: Vec3, world_axis: Vec3, bias: f32, cc: f32, stiffness: f32, damping: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else if (stiffness > 0.0 or damping > 0.0) {
            self.effective_mass = self.spring.calculateSpringPropertiesWithStiffnessAndDamping(delta_time, inv_effective_mass, bias, cc, stiffness, damping);
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring.calculateSpringPropertiesWithBias(bias);
        }
    }

    /// Turns to a hard limit when `spring_settings` has stiffness/frequency 0.
    pub fn calculateConstraintPropertiesWithSettingsForLimit(self: *AxisConstraintPart, delta_time: f32, body1: *const SolverBody, r1_plus_u: Vec3, body2: *const SolverBody, r2: Vec3, world_axis: Vec3, bias: f32, cc: f32, spring_settings: SpringSettings) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else if (!springHasStiffness(spring_settings)) {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring.calculateSpringPropertiesWithBias(bias);
        } else {
            self.effective_mass = self.spring.calculateSpringPropertiesWithSettings(delta_time, inv_effective_mass, bias, cc, spring_settings);
        }
    }

    /// Assumes the spring has either stiffness or damping.
    pub fn calculateConstraintPropertiesWithSettingsForMotor(self: *AxisConstraintPart, delta_time: f32, body1: *const SolverBody, r1_plus_u: Vec3, body2: *const SolverBody, r2: Vec3, world_axis: Vec3, bias: f32, cc: f32, spring_settings: SpringSettings) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = self.spring.calculateSpringPropertiesWithSettings(delta_time, inv_effective_mass, bias, cc, spring_settings);
        }
    }

    pub fn deactivate(self: *AxisConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    pub fn isActive(self: AxisConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    pub fn warmStart(self: *AxisConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(bodies, world_axis, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *AxisConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, min_lambda: f32, max_lambda: f32) bool {
        var jv = dot3(world_axis, sub3(bodies.body1.linear_velocity, bodies.body2.linear_velocity));
        jv += dot3(self.r1_plus_u_x_axis, bodies.body1.angular_velocity);
        jv -= dot3(self.r2_x_axis, bodies.body2.angular_velocity);

        var lambda = self.effective_mass * (jv - self.spring.getBias(self.total_lambda));
        const new_lambda = std.math.clamp(self.total_lambda + lambda, min_lambda, max_lambda);
        lambda = new_lambda - self.total_lambda;
        self.total_lambda = new_lambda;

        return self.applyVelocityStep(bodies, world_axis, lambda);
    }

    pub fn getTotalLambda(self: AxisConstraintPart) f32 {
        return self.total_lambda;
    }

    pub fn solvePositionConstraint(self: *const AxisConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, cc: f32, baumgarte: f32) bool {
        if (cc != 0.0 and !self.spring.isActive()) {
            const lambda = -self.effective_mass * baumgarte * cc;
            if (bodies.body1.is_dynamic) {
                bodies.body1.position_delta = sub3(bodies.body1.position_delta, scale3(world_axis, lambda * bodies.body1.inverse_mass));
                bodies.body1.rotation_delta = sub3(bodies.body1.rotation_delta, scale3(self.inv_i1_r1_plus_u_x_axis, lambda));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.position_delta = add3(bodies.body2.position_delta, scale3(world_axis, lambda * bodies.body2.inverse_mass));
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, scale3(self.inv_i2_r2x_axis, lambda));
            }
            return true;
        }
        return false;
    }

    pub fn saveState(self: AxisConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, f32, self.total_lambda);
    }
    pub fn restoreState(self: *AxisConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, f32);
    }
};

//=============================================================================
// AngleConstraintPart
//=============================================================================

/// Constrains rotation along 1 axis, with an optional spring. A copy of
/// `AxisConstraintPart`'s shape, specialised to angular quantities.
/// `AngleConstraintPart.h`.
pub const AngleConstraintPart = struct {
    inv_i1_axis: Vec3 = vec3_zero,
    inv_i2_axis: Vec3 = vec3_zero,
    effective_mass: f32 = 0,
    spring: SpringPart = .{},
    total_lambda: f32 = 0,

    fn applyVelocityStep(self: *const AngleConstraintPart, bodies: *SolverBodyPair, lambda: f32) bool {
        if (lambda != 0.0) {
            if (bodies.body1.is_dynamic)
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, scale3(self.inv_i1_axis, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, scale3(self.inv_i2_axis, lambda));
            return true;
        }
        return false;
    }

    fn calculateInverseEffectiveMass(self: *AngleConstraintPart, body1: *const SolverBody, body2: *const SolverBody, world_axis: Vec3) f32 {
        self.inv_i1_axis = multiplyWorldSpaceInverseInertiaByVector(body1, world_axis);
        self.inv_i2_axis = multiplyWorldSpaceInverseInertiaByVector(body2, world_axis);
        return dot3(world_axis, add3(self.inv_i1_axis, self.inv_i2_axis));
    }

    pub fn calculateConstraintProperties(self: *AngleConstraintPart, body1: *const SolverBody, body2: *const SolverBody, world_axis: Vec3, bias: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring.calculateSpringPropertiesWithBias(bias);
        }
    }

    pub fn calculateConstraintPropertiesWithFrequencyAndDamping(self: *AngleConstraintPart, delta_time: f32, body1: *const SolverBody, body2: *const SolverBody, world_axis: Vec3, bias: f32, cc: f32, frequency: f32, damping: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else if (frequency > 0.0) {
            self.effective_mass = self.spring.calculateSpringPropertiesWithFrequencyAndDamping(delta_time, inv_effective_mass, bias, cc, frequency, damping);
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring.calculateSpringPropertiesWithBias(bias);
        }
    }

    pub fn calculateConstraintPropertiesWithStiffnessAndDamping(self: *AngleConstraintPart, delta_time: f32, body1: *const SolverBody, body2: *const SolverBody, world_axis: Vec3, bias: f32, cc: f32, stiffness: f32, damping: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else if (stiffness > 0.0 or damping > 0.0) {
            self.effective_mass = self.spring.calculateSpringPropertiesWithStiffnessAndDamping(delta_time, inv_effective_mass, bias, cc, stiffness, damping);
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring.calculateSpringPropertiesWithBias(bias);
        }
    }

    pub fn calculateConstraintPropertiesWithSettingsForLimit(self: *AngleConstraintPart, delta_time: f32, body1: *const SolverBody, body2: *const SolverBody, world_axis: Vec3, bias: f32, cc: f32, spring_settings: SpringSettings) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else if (!springHasStiffness(spring_settings)) {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring.calculateSpringPropertiesWithBias(bias);
        } else {
            self.effective_mass = self.spring.calculateSpringPropertiesWithSettings(delta_time, inv_effective_mass, bias, cc, spring_settings);
        }
    }

    pub fn calculateConstraintPropertiesWithSettingsForMotor(self: *AngleConstraintPart, delta_time: f32, body1: *const SolverBody, body2: *const SolverBody, world_axis: Vec3, bias: f32, cc: f32, spring_settings: SpringSettings) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_axis);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = self.spring.calculateSpringPropertiesWithSettings(delta_time, inv_effective_mass, bias, cc, spring_settings);
        }
    }

    pub fn deactivate(self: *AngleConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    pub fn isActive(self: AngleConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    pub fn warmStart(self: *AngleConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(bodies, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *AngleConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, min_lambda: f32, max_lambda: f32) bool {
        var lambda = self.effective_mass * (dot3(world_axis, sub3(bodies.body1.angular_velocity, bodies.body2.angular_velocity)) - self.spring.getBias(self.total_lambda));
        const new_lambda = std.math.clamp(self.total_lambda + lambda, min_lambda, max_lambda);
        lambda = new_lambda - self.total_lambda;
        self.total_lambda = new_lambda;
        return self.applyVelocityStep(bodies, lambda);
    }

    pub fn getTotalLambda(self: AngleConstraintPart) f32 {
        return self.total_lambda;
    }

    pub fn solvePositionConstraint(self: *const AngleConstraintPart, bodies: *SolverBodyPair, cc: f32, baumgarte: f32) bool {
        if (cc != 0.0 and !self.spring.isActive()) {
            const lambda = -self.effective_mass * baumgarte * cc;
            if (bodies.body1.is_dynamic)
                bodies.body1.rotation_delta = sub3(bodies.body1.rotation_delta, scale3(self.inv_i1_axis, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, scale3(self.inv_i2_axis, lambda));
            return true;
        }
        return false;
    }

    pub fn saveState(self: AngleConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, f32, self.total_lambda);
    }
    pub fn restoreState(self: *AngleConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, f32);
    }
};

//=============================================================================
// RotationEulerConstraintPart
//=============================================================================

/// Constrains rotation around all 3 axes, Euler-angle-error approximation.
/// `RotationEulerConstraintPart.h`.
pub const RotationEulerConstraintPart = struct {
    inv_i1: Mat3 = mat3_zero,
    inv_i2: Mat3 = mat3_zero,
    effective_mass: Mat3 = mat3_zero,
    total_lambda: Vec3 = vec3_zero,
    active: bool = false,

    pub fn sGetInvInitialOrientation(body1: *const SolverBody, body2: *const SolverBody) Quat {
        return qmul(qconj(body2.rotation), body1.rotation);
    }

    pub fn sGetInvInitialOrientationXY(axis_x1: Vec3, axis_y1: Vec3, axis_x2: Vec3, axis_y2: Vec3) Quat {
        if (vec3Eq(axis_x1, axis_x2) and vec3Eq(axis_y1, axis_y2)) return quat_identity;
        const c1 = mat3FromColumns(axis_x1, axis_y1, cross3(axis_x1, axis_y1));
        const c2 = mat3FromColumns(axis_x2, axis_y2, cross3(axis_x2, axis_y2));
        return qmul(mat3GetQuaternion(c2), qconj(mat3GetQuaternion(c1)));
    }

    pub fn sGetInvInitialOrientationXZ(axis_x1: Vec3, axis_z1: Vec3, axis_x2: Vec3, axis_z2: Vec3) Quat {
        if (vec3Eq(axis_x1, axis_x2) and vec3Eq(axis_z1, axis_z2)) return quat_identity;
        const c1 = mat3FromColumns(axis_x1, cross3(axis_z1, axis_x1), axis_z1);
        const c2 = mat3FromColumns(axis_x2, cross3(axis_z2, axis_x2), axis_z2);
        return qmul(mat3GetQuaternion(c2), qconj(mat3GetQuaternion(c1)));
    }

    fn applyVelocityStep(self: *const RotationEulerConstraintPart, bodies: *SolverBodyPair, lambda: Vec3) bool {
        if (!isZero3(lambda)) {
            if (bodies.body1.is_dynamic)
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, mat3MulVec(self.inv_i1, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, mat3MulVec(self.inv_i2, lambda));
            return true;
        }
        return false;
    }

    pub fn calculateConstraintProperties(self: *RotationEulerConstraintPart, body1: *const SolverBody, body2: *const SolverBody) void {
        self.inv_i1 = if (body1.is_dynamic) mat3FromRowMajor9(body1.inverse_inertia) else mat3_zero;
        self.inv_i2 = if (body2.is_dynamic) mat3FromRowMajor9(body2.inverse_inertia) else mat3_zero;

        var inertia_sum = mat3Add(self.inv_i1, self.inv_i2);
        if (mat3Invert(inertia_sum)) |inv| {
            self.effective_mass = inv;
            self.active = true;
        } else {
            // A locked column becomes identity: any impulse is multiplied by
            // inv_i1/inv_i2 first, which is zero for the locked coordinate
            // either way, so the choice of column here never matters.
            if (isZero3(mat3Col(inertia_sum, 0))) inertia_sum.r0.x = 1;
            if (isZero3(mat3Col(inertia_sum, 1))) inertia_sum.r1.y = 1;
            if (isZero3(mat3Col(inertia_sum, 2))) inertia_sum.r2.z = 1;
            if (mat3Invert(inertia_sum)) |inv2| {
                self.effective_mass = inv2;
                self.active = true;
            } else {
                self.deactivate();
            }
        }
    }

    pub fn deactivate(self: *RotationEulerConstraintPart) void {
        self.effective_mass = mat3_zero;
        self.total_lambda = vec3_zero;
        self.active = false;
    }

    pub fn isActive(self: RotationEulerConstraintPart) bool {
        return self.active;
    }

    pub fn warmStart(self: *RotationEulerConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) void {
        self.total_lambda = scale3(self.total_lambda, warm_start_impulse_ratio);
        _ = self.applyVelocityStep(bodies, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *RotationEulerConstraintPart, bodies: *SolverBodyPair) bool {
        const lambda = mat3MulVec(self.effective_mass, sub3(bodies.body1.angular_velocity, bodies.body2.angular_velocity));
        self.total_lambda = add3(self.total_lambda, lambda);
        return self.applyVelocityStep(bodies, lambda);
    }

    pub fn getTotalLambda(self: RotationEulerConstraintPart) Vec3 {
        return self.total_lambda;
    }

    pub fn solvePositionConstraint(self: *const RotationEulerConstraintPart, bodies: *SolverBodyPair, inv_initial_orientation: Quat, baumgarte: f32) bool {
        const diff = qmul(qmul(bodies.body2.rotation, inv_initial_orientation), qconj(bodies.body1.rotation));
        const err = scale3(qxyz(qEnsureWPositive(diff)), 2.0);
        if (!isZero3(err)) {
            const lambda = mat3MulVec(self.effective_mass, scale3(err, -baumgarte));
            if (bodies.body1.is_dynamic)
                bodies.body1.rotation_delta = sub3(bodies.body1.rotation_delta, mat3MulVec(self.inv_i1, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, mat3MulVec(self.inv_i2, lambda));
            return true;
        }
        return false;
    }

    pub fn saveState(self: RotationEulerConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, Vec3, self.total_lambda);
    }
    pub fn restoreState(self: *RotationEulerConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, Vec3);
    }
};

fn vec3Eq(a: Vec3, b: Vec3) bool {
    return a.x == b.x and a.y == b.y and a.z == b.z;
}
fn mat3FromColumns(c0: Vec3, c1: Vec3, c2: Vec3) Mat3 {
    return .{ .r0 = v3(c0.x, c1.x, c2.x), .r1 = v3(c0.y, c1.y, c2.y), .r2 = v3(c0.z, c1.z, c2.z) };
}
/// `Mat44::GetQuaternion`'s trace-based extraction, restricted to the
/// rotation columns a `Mat3` already is.
fn mat3GetQuaternion(m: Mat3) Quat {
    const tr = m.r0.x + m.r1.y + m.r2.z;
    if (tr >= 0.0) {
        const s = @sqrt(tr + 1.0);
        const is = 0.5 / s;
        return .{ .x = (m.r2.y - m.r1.z) * is, .y = (m.r0.z - m.r2.x) * is, .z = (m.r1.x - m.r0.y) * is, .w = 0.5 * s };
    }
    if (m.r1.y > m.r0.x and m.r2.z > m.r1.y) {
        const s = @sqrt(m.r2.z - (m.r0.x + m.r1.y) + 1.0);
        const is = 0.5 / s;
        return .{ .x = (m.r2.x + m.r0.z) * is, .y = (m.r1.z + m.r2.y) * is, .z = 0.5 * s, .w = (m.r0.y - m.r1.x) * is };
    }
    if (m.r1.y > m.r0.x) {
        const s = @sqrt(m.r1.y - (m.r2.z + m.r0.x) + 1.0);
        const is = 0.5 / s;
        return .{ .x = (m.r0.y + m.r1.x) * is, .y = 0.5 * s, .z = (m.r1.z + m.r2.y) * is, .w = (m.r2.x - m.r0.z) * is };
    }
    const s = @sqrt(m.r0.x - (m.r1.y + m.r2.z) + 1.0);
    const is = 0.5 / s;
    return .{ .x = 0.5 * s, .y = (m.r0.y + m.r1.x) * is, .z = (m.r0.z + m.r2.x) * is, .w = (m.r1.z - m.r2.y) * is };
}

//=============================================================================
// RotationQuatConstraintPart
//=============================================================================

/// Constrains rotation around all 3 axes with a quaternion-based error term,
/// slightly more expensive and slightly more correct than
/// `RotationEulerConstraintPart`. `RotationQuatConstraintPart.h`.
pub const RotationQuatConstraintPart = struct {
    inv_i1_jpt: Mat3 = mat3_zero,
    inv_i2_jpt: Mat3 = mat3_zero,
    effective_mass: Mat3 = mat3_zero,
    effective_mass_jp: Mat3 = mat3_zero,
    total_lambda: Vec3 = vec3_zero,
    active: bool = false,

    pub fn sGetInvInitialOrientation(body1: *const SolverBody, body2: *const SolverBody) Quat {
        return qmul(qconj(body2.rotation), body1.rotation);
    }

    fn applyVelocityStep(self: *const RotationQuatConstraintPart, bodies: *SolverBodyPair, lambda: Vec3) bool {
        if (!isZero3(lambda)) {
            if (bodies.body1.is_dynamic)
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, mat3MulVec(self.inv_i1_jpt, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, mat3MulVec(self.inv_i2_jpt, lambda));
            return true;
        }
        return false;
    }

    pub fn calculateConstraintProperties(self: *RotationQuatConstraintPart, body1: *const SolverBody, body2: *const SolverBody, inv_initial_orientation: Quat) void {
        // JP = 1/2 A ML(q1^*) MR(q2 r0^*) A^T, the 3x3 block of the quaternion
        // left/right multiplication matrices' product — see `quat4x4Mul3x3`.
        const half_conj_q1: Quat = .{ .x = -0.5 * body1.rotation.x, .y = -0.5 * body1.rotation.y, .z = -0.5 * body1.rotation.z, .w = 0.5 * body1.rotation.w };
        const jp = quatLeftRightMul3x3(half_conj_q1, qmul(body2.rotation, inv_initial_orientation));

        const inv_i1 = if (body1.is_dynamic) mat3FromRowMajor9(body1.inverse_inertia) else mat3_zero;
        const inv_i2 = if (body2.is_dynamic) mat3FromRowMajor9(body2.inverse_inertia) else mat3_zero;
        self.inv_i1_jpt = mat3MulTransposeRight(inv_i1, jp);
        self.inv_i2_jpt = mat3MulTransposeRight(inv_i2, jp);

        const inv_effective_mass = mat3MulTransposeRight(mat3Mul(jp, mat3Add(inv_i1, inv_i2)), jp);
        if (mat3Invert(inv_effective_mass)) |inv| {
            self.effective_mass = inv;
            self.effective_mass_jp = mat3Mul(inv, jp);
            self.active = true;
        } else {
            self.deactivate();
        }
    }

    pub fn deactivate(self: *RotationQuatConstraintPart) void {
        self.effective_mass = mat3_zero;
        self.effective_mass_jp = mat3_zero;
        self.total_lambda = vec3_zero;
        self.active = false;
    }

    pub fn isActive(self: RotationQuatConstraintPart) bool {
        return self.active;
    }

    pub fn warmStart(self: *RotationQuatConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) void {
        self.total_lambda = scale3(self.total_lambda, warm_start_impulse_ratio);
        _ = self.applyVelocityStep(bodies, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *RotationQuatConstraintPart, bodies: *SolverBodyPair) bool {
        const lambda = mat3MulVec(self.effective_mass_jp, sub3(bodies.body1.angular_velocity, bodies.body2.angular_velocity));
        self.total_lambda = add3(self.total_lambda, lambda);
        return self.applyVelocityStep(bodies, lambda);
    }

    pub fn solvePositionConstraint(self: *const RotationQuatConstraintPart, bodies: *SolverBodyPair, inv_initial_orientation: Quat, baumgarte: f32) bool {
        const cc = qxyz(qmul(qmul(qconj(bodies.body1.rotation), bodies.body2.rotation), inv_initial_orientation));
        if (!isZero3(cc)) {
            const lambda = scale3(mat3MulVec(self.effective_mass, cc), -baumgarte);
            if (bodies.body1.is_dynamic)
                bodies.body1.rotation_delta = sub3(bodies.body1.rotation_delta, mat3MulVec(self.inv_i1_jpt, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, mat3MulVec(self.inv_i2_jpt, lambda));
            return true;
        }
        return false;
    }

    pub fn getTotalLambda(self: RotationQuatConstraintPart) Vec3 {
        return self.total_lambda;
    }

    pub fn saveState(self: RotationQuatConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, Vec3, self.total_lambda);
    }
    pub fn restoreState(self: *RotationQuatConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, Vec3);
    }
};

/// The top-left 3x3 block of `Mat44::sQuatLeftMultiply(p) *
/// Mat44::sQuatRightMultiply(q)`, computed directly from the closed forms of
/// the two 4x4 multiplication matrices rather than by materialising them —
/// `GetRotationSafe()` on the product only zeroes elements `Multiply3x3`
/// never reads anyway, so it is not needed here.
fn quatLeftRightMul3x3(p: Quat, q: Quat) Mat3 {
    // L(p): columns (w,z,-y,-x), (-z,w,x,-y), (y,-x,w,-z), (x,y,z,w)
    const l_row0 = v3(p.w, -p.z, p.y);
    const l_row0_w = p.x;
    const l_row1 = v3(p.z, p.w, -p.x);
    const l_row1_w = p.y;
    const l_row2 = v3(-p.y, p.x, p.w);
    const l_row2_w = p.z;

    // R(q): columns (w,-z,y,-x), (z,w,-x,-y), (-y,x,w,-z), (x,y,z,w)
    const r_col0 = v3(q.w, -q.z, q.y);
    const r_col1 = v3(q.z, q.w, -q.x);
    const r_col2 = v3(-q.y, q.x, q.w);
    const r_row3 = v3(q.x, q.y, q.z);

    // (L*R)[i][j] = dot(L.row(i), R.col(j)), a 4-term sum since both are 4x4.
    return .{
        .r0 = v3(
            dot3(l_row0, r_col0) + l_row0_w * r_row3.x,
            dot3(l_row0, r_col1) + l_row0_w * r_row3.y,
            dot3(l_row0, r_col2) + l_row0_w * r_row3.z,
        ),
        .r1 = v3(
            dot3(l_row1, r_col0) + l_row1_w * r_row3.x,
            dot3(l_row1, r_col1) + l_row1_w * r_row3.y,
            dot3(l_row1, r_col2) + l_row1_w * r_row3.z,
        ),
        .r2 = v3(
            dot3(l_row2, r_col0) + l_row2_w * r_row3.x,
            dot3(l_row2, r_col1) + l_row2_w * r_row3.y,
            dot3(l_row2, r_col2) + l_row2_w * r_row3.z,
        ),
    };
}

//=============================================================================
// HingeRotationConstraintPart
//=============================================================================

/// Constrains rotation around 2 axes so that only rotation around 1 axis is
/// allowed. `HingeRotationConstraintPart.h`. No `isActive` — Jolt's own type
/// has none either.
pub const HingeRotationConstraintPart = struct {
    a1: Vec3 = vec3_zero,
    b2: Vec3 = vec3_zero,
    c2: Vec3 = vec3_zero,
    inv_i1: Mat3 = mat3_zero,
    inv_i2: Mat3 = mat3_zero,
    b2x_a1: Vec3 = vec3_zero,
    c2x_a1: Vec3 = vec3_zero,
    effective_mass: Mat22 = mat22_zero,
    total_lambda: Vec2 = vec2_zero,

    fn applyVelocityStep(self: *const HingeRotationConstraintPart, bodies: *SolverBodyPair, lambda: Vec2) bool {
        if (!v2IsZero(lambda)) {
            const impulse = add3(scale3(self.b2x_a1, lambda.x), scale3(self.c2x_a1, lambda.y));
            if (bodies.body1.is_dynamic)
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, mat3MulVec(self.inv_i1, impulse));
            if (bodies.body2.is_dynamic)
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, mat3MulVec(self.inv_i2, impulse));
            return true;
        }
        return false;
    }

    pub fn calculateConstraintProperties(self: *HingeRotationConstraintPart, body1: *const SolverBody, world_space_hinge_axis1: Vec3, body2: *const SolverBody, world_space_hinge_axis2: Vec3) void {
        self.a1 = world_space_hinge_axis1;
        var a2 = world_space_hinge_axis2;
        const d = dot3(self.a1, a2);
        if (d <= 1.0e-3) {
            var perp = sub3(a2, scale3(self.a1, d));
            if (lengthSq3(perp) < 1.0e-6) {
                perp = getNormalizedPerpendicular3(self.a1);
            }
            const perp_norm = scale3(perp, 1.0 / @sqrt(lengthSq3(perp)));
            a2 = normalize3(add3(scale3(perp_norm, 0.99), scale3(self.a1, 0.01)));
        }
        self.b2 = getNormalizedPerpendicular3(a2);
        self.c2 = cross3(a2, self.b2);

        self.inv_i1 = if (body1.is_dynamic) mat3FromRowMajor9(body1.inverse_inertia) else mat3_zero;
        self.inv_i2 = if (body2.is_dynamic) mat3FromRowMajor9(body2.inverse_inertia) else mat3_zero;
        self.b2x_a1 = cross3(self.b2, self.a1);
        self.c2x_a1 = cross3(self.c2, self.a1);

        const summed_inv_inertia = mat3Add(self.inv_i1, self.inv_i2);
        const inv_eff = Mat22{
            .m00 = dot3(self.b2x_a1, mat3MulVec(summed_inv_inertia, self.b2x_a1)),
            .m01 = dot3(self.b2x_a1, mat3MulVec(summed_inv_inertia, self.c2x_a1)),
            .m10 = dot3(self.c2x_a1, mat3MulVec(summed_inv_inertia, self.b2x_a1)),
            .m11 = dot3(self.c2x_a1, mat3MulVec(summed_inv_inertia, self.c2x_a1)),
        };
        if (mat22Invert(inv_eff)) |inv| {
            self.effective_mass = inv;
        } else {
            self.deactivate();
        }
    }

    pub fn deactivate(self: *HingeRotationConstraintPart) void {
        self.effective_mass = mat22_zero;
        self.total_lambda = vec2_zero;
    }

    pub fn warmStart(self: *HingeRotationConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) void {
        self.total_lambda = .{ .x = self.total_lambda.x * warm_start_impulse_ratio, .y = self.total_lambda.y * warm_start_impulse_ratio };
        _ = self.applyVelocityStep(bodies, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *HingeRotationConstraintPart, bodies: *SolverBodyPair) bool {
        const delta_ang = sub3(bodies.body1.angular_velocity, bodies.body2.angular_velocity);
        const jv = Vec2{ .x = dot3(self.b2x_a1, delta_ang), .y = dot3(self.c2x_a1, delta_ang) };
        const lambda = mat22MulVec(self.effective_mass, jv);
        self.total_lambda = .{ .x = self.total_lambda.x + lambda.x, .y = self.total_lambda.y + lambda.y };
        return self.applyVelocityStep(bodies, lambda);
    }

    pub fn solvePositionConstraint(self: *const HingeRotationConstraintPart, bodies: *SolverBodyPair, baumgarte: f32) bool {
        const cc = Vec2{ .x = dot3(self.a1, self.b2), .y = dot3(self.a1, self.c2) };
        if (!v2IsZero(cc)) {
            const raw = mat22MulVec(self.effective_mass, cc);
            const lambda = Vec2{ .x = -baumgarte * raw.x, .y = -baumgarte * raw.y };
            const impulse = add3(scale3(self.b2x_a1, lambda.x), scale3(self.c2x_a1, lambda.y));
            if (bodies.body1.is_dynamic)
                bodies.body1.rotation_delta = sub3(bodies.body1.rotation_delta, mat3MulVec(self.inv_i1, impulse));
            if (bodies.body2.is_dynamic)
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, mat3MulVec(self.inv_i2, impulse));
            return true;
        }
        return false;
    }

    pub fn getTotalLambda(self: HingeRotationConstraintPart) Vec2 {
        return self.total_lambda;
    }

    pub fn saveState(self: HingeRotationConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, Vec2, self.total_lambda);
    }
    pub fn restoreState(self: *HingeRotationConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, Vec2);
    }
};

fn normalize3(a: Vec3) Vec3 {
    return scale3(a, 1.0 / @sqrt(lengthSq3(a)));
}

//=============================================================================
// DualAxisConstraintPart
//=============================================================================

/// Constrains movement on 2 axes. `DualAxisConstraintPart.h`.
pub const DualAxisConstraintPart = struct {
    r1_plus_u_x_n1: Vec3 = vec3_zero,
    r1_plus_u_x_n2: Vec3 = vec3_zero,
    r2x_n1: Vec3 = vec3_zero,
    r2x_n2: Vec3 = vec3_zero,
    inv_i1_r1_plus_u_x_n1: Vec3 = vec3_zero,
    inv_i1_r1_plus_u_x_n2: Vec3 = vec3_zero,
    inv_i2_r2x_n1: Vec3 = vec3_zero,
    inv_i2_r2x_n2: Vec3 = vec3_zero,
    effective_mass: Mat22 = mat22_zero,
    total_lambda: Vec2 = vec2_zero,

    fn applyVelocityStep(self: *const DualAxisConstraintPart, bodies: *SolverBodyPair, n1: Vec3, n2: Vec3, lambda: Vec2) bool {
        if (!v2IsZero(lambda)) {
            const impulse = add3(scale3(n1, lambda.x), scale3(n2, lambda.y));
            if (bodies.body1.is_dynamic) {
                bodies.body1.linear_velocity = sub3(bodies.body1.linear_velocity, scale3(impulse, bodies.body1.inverse_mass));
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, add3(scale3(self.inv_i1_r1_plus_u_x_n1, lambda.x), scale3(self.inv_i1_r1_plus_u_x_n2, lambda.y)));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.linear_velocity = add3(bodies.body2.linear_velocity, scale3(impulse, bodies.body2.inverse_mass));
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, add3(scale3(self.inv_i2_r2x_n1, lambda.x), scale3(self.inv_i2_r2x_n2, lambda.y)));
            }
            return true;
        }
        return false;
    }

    fn calculateLagrangeMultiplier(self: *const DualAxisConstraintPart, bodies: *const SolverBodyPair, n1: Vec3, n2: Vec3) Vec2 {
        const delta_lin = sub3(bodies.body1.linear_velocity, bodies.body2.linear_velocity);
        const jv = Vec2{
            .x = dot3(n1, delta_lin) + dot3(self.r1_plus_u_x_n1, bodies.body1.angular_velocity) - dot3(self.r2x_n1, bodies.body2.angular_velocity),
            .y = dot3(n2, delta_lin) + dot3(self.r1_plus_u_x_n2, bodies.body1.angular_velocity) - dot3(self.r2x_n2, bodies.body2.angular_velocity),
        };
        return mat22MulVec(self.effective_mass, jv);
    }

    /// All input vectors are in world space.
    pub fn calculateConstraintProperties(self: *DualAxisConstraintPart, body1: *const SolverBody, r1_plus_u: Vec3, body2: *const SolverBody, r2: Vec3, n1: Vec3, n2: Vec3) void {
        self.r1_plus_u_x_n1 = cross3(r1_plus_u, n1);
        self.r1_plus_u_x_n2 = cross3(r1_plus_u, n2);
        self.r2x_n1 = cross3(r2, n1);
        self.r2x_n2 = cross3(r2, n2);

        var inv_eff = mat22_zero;
        if (body1.is_dynamic) {
            self.inv_i1_r1_plus_u_x_n1 = multiplyWorldSpaceInverseInertiaByVector(body1, self.r1_plus_u_x_n1);
            self.inv_i1_r1_plus_u_x_n2 = multiplyWorldSpaceInverseInertiaByVector(body1, self.r1_plus_u_x_n2);
            inv_eff.m00 = body1.inverse_mass + dot3(self.r1_plus_u_x_n1, self.inv_i1_r1_plus_u_x_n1);
            inv_eff.m01 = dot3(self.r1_plus_u_x_n1, self.inv_i1_r1_plus_u_x_n2);
            inv_eff.m10 = dot3(self.r1_plus_u_x_n2, self.inv_i1_r1_plus_u_x_n1);
            inv_eff.m11 = body1.inverse_mass + dot3(self.r1_plus_u_x_n2, self.inv_i1_r1_plus_u_x_n2);
        } else {
            self.inv_i1_r1_plus_u_x_n1 = vec3_zero;
            self.inv_i1_r1_plus_u_x_n2 = vec3_zero;
        }

        if (body2.is_dynamic) {
            self.inv_i2_r2x_n1 = multiplyWorldSpaceInverseInertiaByVector(body2, self.r2x_n1);
            self.inv_i2_r2x_n2 = multiplyWorldSpaceInverseInertiaByVector(body2, self.r2x_n2);
            inv_eff.m00 += body2.inverse_mass + dot3(self.r2x_n1, self.inv_i2_r2x_n1);
            inv_eff.m01 += dot3(self.r2x_n1, self.inv_i2_r2x_n2);
            inv_eff.m10 += dot3(self.r2x_n2, self.inv_i2_r2x_n1);
            inv_eff.m11 += body2.inverse_mass + dot3(self.r2x_n2, self.inv_i2_r2x_n2);
        } else {
            self.inv_i2_r2x_n1 = vec3_zero;
            self.inv_i2_r2x_n2 = vec3_zero;
        }

        if (mat22Invert(inv_eff)) |inv| {
            self.effective_mass = inv;
        } else {
            self.deactivate();
        }
    }

    pub fn deactivate(self: *DualAxisConstraintPart) void {
        self.effective_mass = mat22_zero;
        self.total_lambda = vec2_zero;
    }

    pub fn isActive(self: DualAxisConstraintPart) bool {
        return !mat22IsZero(self.effective_mass);
    }

    pub fn warmStart(self: *DualAxisConstraintPart, bodies: *SolverBodyPair, n1: Vec3, n2: Vec3, warm_start_impulse_ratio: f32) void {
        self.total_lambda = .{ .x = self.total_lambda.x * warm_start_impulse_ratio, .y = self.total_lambda.y * warm_start_impulse_ratio };
        _ = self.applyVelocityStep(bodies, n1, n2, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *DualAxisConstraintPart, bodies: *SolverBodyPair, n1: Vec3, n2: Vec3) bool {
        const lambda = self.calculateLagrangeMultiplier(bodies, n1, n2);
        self.total_lambda = .{ .x = self.total_lambda.x + lambda.x, .y = self.total_lambda.y + lambda.y };
        return self.applyVelocityStep(bodies, n1, n2, lambda);
    }

    pub fn solvePositionConstraint(self: *const DualAxisConstraintPart, bodies: *SolverBodyPair, u: Vec3, n1: Vec3, n2: Vec3, baumgarte: f32) bool {
        const cc = Vec2{ .x = dot3(u, n1), .y = dot3(u, n2) };
        if (!v2IsZero(cc)) {
            const raw = mat22MulVec(self.effective_mass, cc);
            const lambda = Vec2{ .x = -baumgarte * raw.x, .y = -baumgarte * raw.y };
            const impulse = add3(scale3(n1, lambda.x), scale3(n2, lambda.y));
            if (bodies.body1.is_dynamic) {
                bodies.body1.position_delta = sub3(bodies.body1.position_delta, scale3(impulse, bodies.body1.inverse_mass));
                bodies.body1.rotation_delta = sub3(bodies.body1.rotation_delta, add3(scale3(self.inv_i1_r1_plus_u_x_n1, lambda.x), scale3(self.inv_i1_r1_plus_u_x_n2, lambda.y)));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.position_delta = add3(bodies.body2.position_delta, scale3(impulse, bodies.body2.inverse_mass));
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, add3(scale3(self.inv_i2_r2x_n1, lambda.x), scale3(self.inv_i2_r2x_n2, lambda.y)));
            }
            return true;
        }
        return false;
    }

    pub fn setTotalLambda(self: *DualAxisConstraintPart, lambda: Vec2) void {
        self.total_lambda = lambda;
    }
    pub fn getTotalLambda(self: DualAxisConstraintPart) Vec2 {
        return self.total_lambda;
    }

    pub fn saveState(self: DualAxisConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, Vec2, self.total_lambda);
    }
    pub fn restoreState(self: *DualAxisConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, Vec2);
    }
};

//=============================================================================
// IndependentAxisConstraintPart
//=============================================================================

/// Like `AxisConstraintPart`, but both bodies have an independent axis and a
/// ratio relates the forces applied on each. `IndependentAxisConstraintPart.h`.
pub const IndependentAxisConstraintPart = struct {
    r1x_n1: Vec3 = vec3_zero,
    inv_i1_r1x_n1: Vec3 = vec3_zero,
    ratio_r2x_n2: Vec3 = vec3_zero,
    inv_i2_ratio_r2x_n2: Vec3 = vec3_zero,
    effective_mass: f32 = 0,
    total_lambda: f32 = 0,

    fn applyVelocityStep(self: *const IndependentAxisConstraintPart, bodies: *SolverBodyPair, n1: Vec3, n2: Vec3, ratio: f32, lambda: f32) bool {
        if (lambda != 0.0) {
            if (bodies.body1.is_dynamic) {
                bodies.body1.linear_velocity = add3(bodies.body1.linear_velocity, scale3(n1, bodies.body1.inverse_mass * lambda));
                bodies.body1.angular_velocity = add3(bodies.body1.angular_velocity, scale3(self.inv_i1_r1x_n1, lambda));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.linear_velocity = add3(bodies.body2.linear_velocity, scale3(n2, ratio * bodies.body2.inverse_mass * lambda));
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, scale3(self.inv_i2_ratio_r2x_n2, lambda));
            }
            return true;
        }
        return false;
    }

    pub fn calculateConstraintProperties(self: *IndependentAxisConstraintPart, body1: *const SolverBody, body2: *const SolverBody, r1: Vec3, n1: Vec3, r2: Vec3, n2: Vec3, ratio: f32) void {
        var inv_effective_mass: f32 = 0;

        if (body1.is_dynamic) {
            self.r1x_n1 = cross3(r1, n1);
            self.inv_i1_r1x_n1 = multiplyWorldSpaceInverseInertiaByVector(body1, self.r1x_n1);
            inv_effective_mass += body1.inverse_mass + dot3(self.inv_i1_r1x_n1, self.r1x_n1);
        } else {
            self.r1x_n1 = vec3_zero;
            self.inv_i1_r1x_n1 = vec3_zero;
        }

        if (body2.is_dynamic) {
            self.ratio_r2x_n2 = scale3(cross3(r2, n2), ratio);
            self.inv_i2_ratio_r2x_n2 = multiplyWorldSpaceInverseInertiaByVector(body2, self.ratio_r2x_n2);
            inv_effective_mass += ratio * ratio * body2.inverse_mass + dot3(self.inv_i2_ratio_r2x_n2, self.ratio_r2x_n2);
        } else {
            self.ratio_r2x_n2 = vec3_zero;
            self.inv_i2_ratio_r2x_n2 = vec3_zero;
        }

        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
        }
    }

    pub fn deactivate(self: *IndependentAxisConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    pub fn isActive(self: IndependentAxisConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    pub fn warmStart(self: *IndependentAxisConstraintPart, bodies: *SolverBodyPair, n1: Vec3, n2: Vec3, ratio: f32, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(bodies, n1, n2, ratio, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *IndependentAxisConstraintPart, bodies: *SolverBodyPair, n1: Vec3, n2: Vec3, ratio: f32, min_lambda: f32, max_lambda: f32) bool {
        var lambda = -self.effective_mass * (dot3(n1, bodies.body1.linear_velocity) + dot3(self.r1x_n1, bodies.body1.angular_velocity) + ratio * dot3(n2, bodies.body2.linear_velocity) + dot3(self.ratio_r2x_n2, bodies.body2.angular_velocity));
        const new_lambda = std.math.clamp(self.total_lambda + lambda, min_lambda, max_lambda);
        lambda = new_lambda - self.total_lambda;
        self.total_lambda = new_lambda;
        return self.applyVelocityStep(bodies, n1, n2, ratio, lambda);
    }

    pub fn getTotalLambda(self: IndependentAxisConstraintPart) f32 {
        return self.total_lambda;
    }

    pub fn solvePositionConstraint(self: *const IndependentAxisConstraintPart, bodies: *SolverBodyPair, n1: Vec3, n2: Vec3, ratio: f32, cc: f32, baumgarte: f32) bool {
        if (cc != 0.0) {
            const lambda = -self.effective_mass * baumgarte * cc;
            if (bodies.body1.is_dynamic) {
                bodies.body1.position_delta = add3(bodies.body1.position_delta, scale3(n1, lambda * bodies.body1.inverse_mass));
                bodies.body1.rotation_delta = add3(bodies.body1.rotation_delta, scale3(self.inv_i1_r1x_n1, lambda));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.position_delta = add3(bodies.body2.position_delta, scale3(n2, lambda * ratio * bodies.body2.inverse_mass));
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, scale3(self.inv_i2_ratio_r2x_n2, lambda));
            }
            return true;
        }
        return false;
    }

    pub fn saveState(self: IndependentAxisConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, f32, self.total_lambda);
    }
    pub fn restoreState(self: *IndependentAxisConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, f32);
    }
};

//=============================================================================
// GearConstraintPart
//=============================================================================

/// Constrains two rotations to move in opposite directions, `w1.a + r w2.b =
/// 0`. `GearConstraintPart.h`.
///
/// Neither Jolt's `CalculateConstraintProperties`/`ApplyVelocityStep` nor
/// this one guards on `IsDynamic()` before reading inertia — a gear's two
/// bodies are always the hinges' own rotating bodies, dynamic by construction.
pub const GearConstraintPart = struct {
    inv_i1_a: Vec3 = vec3_zero,
    inv_i2_b: Vec3 = vec3_zero,
    effective_mass: f32 = 0,
    total_lambda: f32 = 0,

    fn applyVelocityStep(self: *const GearConstraintPart, bodies: *SolverBodyPair, lambda: f32) bool {
        if (lambda != 0.0) {
            bodies.body1.angular_velocity = add3(bodies.body1.angular_velocity, scale3(self.inv_i1_a, lambda));
            bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, scale3(self.inv_i2_b, lambda));
            return true;
        }
        return false;
    }

    pub fn calculateConstraintProperties(self: *GearConstraintPart, body1: *const SolverBody, world_space_hinge_axis1: Vec3, body2: *const SolverBody, world_space_hinge_axis2: Vec3, ratio: f32) void {
        self.inv_i1_a = multiplyWorldSpaceInverseInertiaByVector(body1, world_space_hinge_axis1);
        self.inv_i2_b = multiplyWorldSpaceInverseInertiaByVector(body2, world_space_hinge_axis2);

        const inv_effective_mass = dot3(world_space_hinge_axis1, self.inv_i1_a) + dot3(world_space_hinge_axis2, self.inv_i2_b) * (ratio * ratio);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
        }
    }

    pub fn deactivate(self: *GearConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    pub fn isActive(self: GearConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    pub fn warmStart(self: *GearConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(bodies, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *GearConstraintPart, bodies: *SolverBodyPair, world_space_hinge_axis1: Vec3, world_space_hinge_axis2: Vec3, ratio: f32) bool {
        const lambda = -self.effective_mass * (dot3(world_space_hinge_axis1, bodies.body1.angular_velocity) + ratio * dot3(world_space_hinge_axis2, bodies.body2.angular_velocity));
        self.total_lambda += lambda;
        return self.applyVelocityStep(bodies, lambda);
    }

    pub fn getTotalLambda(self: GearConstraintPart) f32 {
        return self.total_lambda;
    }

    pub fn solvePositionConstraint(self: *const GearConstraintPart, bodies: *SolverBodyPair, cc: f32, baumgarte: f32) bool {
        if (cc != 0.0) {
            const lambda = -self.effective_mass * baumgarte * cc;
            if (bodies.body1.is_dynamic)
                bodies.body1.rotation_delta = add3(bodies.body1.rotation_delta, scale3(self.inv_i1_a, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, scale3(self.inv_i2_b, lambda));
            return true;
        }
        return false;
    }

    pub fn saveState(self: GearConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, f32, self.total_lambda);
    }
    pub fn restoreState(self: *GearConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, f32);
    }
};

//=============================================================================
// RackAndPinionConstraintPart
//=============================================================================

/// Constrains a rotation to a translation, `w1.a - r v2.b = 0`. Same
/// unguarded-inertia note as `GearConstraintPart` applies here.
/// `RackAndPinionConstraintPart.h`.
pub const RackAndPinionConstraintPart = struct {
    inv_i1_a: Vec3 = vec3_zero,
    ratio_inv_m2_b: Vec3 = vec3_zero,
    effective_mass: f32 = 0,
    total_lambda: f32 = 0,

    fn applyVelocityStep(self: *const RackAndPinionConstraintPart, bodies: *SolverBodyPair, lambda: f32) bool {
        if (lambda != 0.0) {
            bodies.body1.angular_velocity = add3(bodies.body1.angular_velocity, scale3(self.inv_i1_a, lambda));
            bodies.body2.linear_velocity = sub3(bodies.body2.linear_velocity, scale3(self.ratio_inv_m2_b, lambda));
            return true;
        }
        return false;
    }

    pub fn calculateConstraintProperties(self: *RackAndPinionConstraintPart, body1: *const SolverBody, world_space_hinge_axis: Vec3, body2: *const SolverBody, world_space_slider_axis: Vec3, ratio: f32) void {
        self.inv_i1_a = multiplyWorldSpaceInverseInertiaByVector(body1, world_space_hinge_axis);

        const inv_m2 = body2.inverse_mass;
        self.ratio_inv_m2_b = scale3(world_space_slider_axis, ratio * inv_m2);

        const inv_effective_mass = dot3(world_space_hinge_axis, self.inv_i1_a) + inv_m2 * (ratio * ratio);
        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
        }
    }

    pub fn deactivate(self: *RackAndPinionConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    pub fn isActive(self: RackAndPinionConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    pub fn warmStart(self: *RackAndPinionConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(bodies, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *RackAndPinionConstraintPart, bodies: *SolverBodyPair, world_space_hinge_axis: Vec3, world_space_slider_axis: Vec3, ratio: f32) bool {
        const lambda = self.effective_mass * (ratio * dot3(world_space_slider_axis, bodies.body2.linear_velocity) - dot3(world_space_hinge_axis, bodies.body1.angular_velocity));
        self.total_lambda += lambda;
        return self.applyVelocityStep(bodies, lambda);
    }

    pub fn getTotalLambda(self: RackAndPinionConstraintPart) f32 {
        return self.total_lambda;
    }

    pub fn solvePositionConstraint(self: *const RackAndPinionConstraintPart, bodies: *SolverBodyPair, cc: f32, baumgarte: f32) bool {
        if (cc != 0.0) {
            const lambda = -self.effective_mass * baumgarte * cc;
            if (bodies.body1.is_dynamic)
                bodies.body1.rotation_delta = add3(bodies.body1.rotation_delta, scale3(self.inv_i1_a, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.position_delta = sub3(bodies.body2.position_delta, scale3(self.ratio_inv_m2_b, lambda));
            return true;
        }
        return false;
    }

    pub fn saveState(self: RackAndPinionConstraintPart, recorder: *StateRecorder) void {
        recWrite(recorder, f32, self.total_lambda);
    }
    pub fn restoreState(self: *RackAndPinionConstraintPart, recorder: *StateRecorder) void {
        self.total_lambda = recRead(recorder, f32);
    }
};

//=============================================================================
// AngularFrictionConstraintPart
//=============================================================================

/// A copy of `AngleConstraintPart` specialised for contact friction, without
/// a spring. `AngularFrictionConstraintPart.h`.
///
/// Jolt templates this per-motion-type for the hot contact solver's memory
/// layout, not a math difference — the single, untemplated shape every
/// specialisation reduces to, `is_dynamic` guards evaluated at runtime.
pub const AngularFrictionConstraintPart = struct {
    inv_i1_axis: Vec3 = vec3_zero,
    inv_i2_axis: Vec3 = vec3_zero,
    effective_mass: f32 = 0,
    bias: f32 = 0,
    total_lambda: f32 = 0,

    fn applyVelocityStep(self: *const AngularFrictionConstraintPart, bodies: *SolverBodyPair, lambda: f32) bool {
        if (lambda != 0.0) {
            if (bodies.body1.is_dynamic)
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, scale3(self.inv_i1_axis, lambda));
            if (bodies.body2.is_dynamic)
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, scale3(self.inv_i2_axis, lambda));
            return true;
        }
        return false;
    }

    pub fn calculateConstraintProperties(self: *AngularFrictionConstraintPart, body1: *const SolverBody, body2: *const SolverBody, world_axis: Vec3, bias: f32) void {
        self.bias = bias;

        self.inv_i1_axis = multiplyWorldSpaceInverseInertiaByVector(body1, world_axis);
        self.inv_i2_axis = multiplyWorldSpaceInverseInertiaByVector(body2, world_axis);

        var inv_effective_mass: f32 = 0;
        if (body1.is_dynamic) inv_effective_mass += dot3(world_axis, self.inv_i1_axis);
        if (body2.is_dynamic) inv_effective_mass += dot3(world_axis, self.inv_i2_axis);

        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
        }
    }

    pub fn deactivate(self: *AngularFrictionConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    pub fn isActive(self: AngularFrictionConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    /// Overrides the total lambda, e.g. to seed warm starting.
    pub fn setTotalLambda(self: *AngularFrictionConstraintPart, lambda: f32) void {
        self.total_lambda = lambda;
    }
    pub fn getTotalLambda(self: AngularFrictionConstraintPart) f32 {
        return self.total_lambda;
    }

    pub fn warmStart(self: *AngularFrictionConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) bool {
        self.total_lambda *= warm_start_impulse_ratio;
        return self.applyVelocityStep(bodies, self.total_lambda);
    }

    pub fn solveVelocityConstraint(self: *AngularFrictionConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, min_lambda: f32, max_lambda: f32) bool {
        const jv = dot3(world_axis, sub3(bodies.body1.angular_velocity, bodies.body2.angular_velocity));
        var lambda = self.effective_mass * (jv - self.bias);
        const new_lambda = std.math.clamp(self.total_lambda + lambda, min_lambda, max_lambda);
        lambda = new_lambda - self.total_lambda;
        self.total_lambda = new_lambda;
        return self.applyVelocityStep(bodies, lambda);
    }
};

//=============================================================================
// ContactConstraintPart
//=============================================================================

/// A copy of `AxisConstraintPart` specialised for contact constraints,
/// splitting `SolveVelocityConstraint` into `...GetTotalLambda` /
/// `...ApplyLambda` so a caller can clamp several axes against each other
/// (Coulomb friction) before applying any of them. `ContactConstraintPart.h`.
/// Same untemplated-by-motion-type note as `AngularFrictionConstraintPart`.
pub const ContactConstraintPart = struct {
    r1_plus_u_x_axis: Vec3 = vec3_zero,
    r2_x_axis: Vec3 = vec3_zero,
    inv_i1_r1_plus_u_x_axis: Vec3 = vec3_zero,
    inv_i2_r2x_axis: Vec3 = vec3_zero,
    effective_mass: f32 = 0,
    bias: f32 = 0,
    total_lambda: f32 = 0,

    fn applyVelocityStep(self: *const ContactConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, lambda: f32) bool {
        if (lambda != 0.0) {
            if (bodies.body1.is_dynamic) {
                bodies.body1.linear_velocity = sub3(bodies.body1.linear_velocity, scale3(world_axis, lambda * bodies.body1.inverse_mass));
                bodies.body1.angular_velocity = sub3(bodies.body1.angular_velocity, scale3(self.inv_i1_r1_plus_u_x_axis, lambda));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.linear_velocity = add3(bodies.body2.linear_velocity, scale3(world_axis, lambda * bodies.body2.inverse_mass));
                bodies.body2.angular_velocity = add3(bodies.body2.angular_velocity, scale3(self.inv_i2_r2x_axis, lambda));
            }
            return true;
        }
        return false;
    }

    pub fn calculateConstraintProperties(self: *ContactConstraintPart, body1: *const SolverBody, r1_plus_u: Vec3, body2: *const SolverBody, r2: Vec3, world_axis: Vec3, bias: f32) void {
        self.bias = bias;

        var inv_effective_mass: f32 = 0;

        self.r1_plus_u_x_axis = cross3(r1_plus_u, world_axis);
        if (body1.is_dynamic) {
            self.inv_i1_r1_plus_u_x_axis = multiplyWorldSpaceInverseInertiaByVector(body1, self.r1_plus_u_x_axis);
            inv_effective_mass += body1.inverse_mass + dot3(self.inv_i1_r1_plus_u_x_axis, self.r1_plus_u_x_axis);
        } else {
            self.inv_i1_r1_plus_u_x_axis = vec3_zero;
        }

        self.r2_x_axis = cross3(r2, world_axis);
        if (body2.is_dynamic) {
            self.inv_i2_r2x_axis = multiplyWorldSpaceInverseInertiaByVector(body2, self.r2_x_axis);
            inv_effective_mass += body2.inverse_mass + dot3(self.inv_i2_r2x_axis, self.r2_x_axis);
        } else {
            self.inv_i2_r2x_axis = vec3_zero;
        }

        if (inv_effective_mass == 0.0) {
            self.deactivate();
        } else {
            self.effective_mass = 1.0 / inv_effective_mass;
        }
    }

    pub fn deactivate(self: *ContactConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    pub fn isActive(self: ContactConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    pub fn setTotalLambda(self: *ContactConstraintPart, lambda: f32) void {
        self.total_lambda = lambda;
    }
    pub fn getTotalLambda(self: ContactConstraintPart) f32 {
        return self.total_lambda;
    }

    pub fn warmStart(self: *ContactConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, warm_start_impulse_ratio: f32) bool {
        self.total_lambda *= warm_start_impulse_ratio;
        return self.applyVelocityStep(bodies, world_axis, self.total_lambda);
    }

    /// Part 1 of `solveVelocityConstraint`: what the new total lambda would
    /// be, without applying it — so a caller can clamp several axes against
    /// each other (Coulomb friction against a normal impulse) first.
    pub fn solveVelocityConstraintGetTotalLambda(self: *const ContactConstraintPart, bodies: *const SolverBodyPair, world_axis: Vec3) f32 {
        var jv = dot3(world_axis, sub3(bodies.body1.linear_velocity, bodies.body2.linear_velocity));
        jv += dot3(self.r1_plus_u_x_axis, bodies.body1.angular_velocity);
        jv -= dot3(self.r2_x_axis, bodies.body2.angular_velocity);
        const lambda = self.effective_mass * (jv - self.bias);
        return self.total_lambda + lambda;
    }

    /// Part 2: apply a total lambda `solveVelocityConstraintGetTotalLambda`
    /// (possibly clamped by the caller) computed.
    pub fn solveVelocityConstraintApplyLambda(self: *ContactConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, total_lambda: f32) bool {
        const delta_lambda = total_lambda - self.total_lambda;
        self.total_lambda = total_lambda;
        return self.applyVelocityStep(bodies, world_axis, delta_lambda);
    }

    pub fn solveVelocityConstraint(self: *ContactConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, min_lambda: f32, max_lambda: f32) bool {
        const total_lambda = std.math.clamp(self.solveVelocityConstraintGetTotalLambda(bodies, world_axis), min_lambda, max_lambda);
        return self.solveVelocityConstraintApplyLambda(bodies, world_axis, total_lambda);
    }

    pub fn solvePositionConstraint(self: *const ContactConstraintPart, bodies: *SolverBodyPair, world_axis: Vec3, cc: f32, baumgarte: f32) bool {
        if (cc != 0.0) {
            const lambda = -self.effective_mass * baumgarte * cc;
            if (bodies.body1.is_dynamic) {
                bodies.body1.position_delta = sub3(bodies.body1.position_delta, scale3(world_axis, lambda * bodies.body1.inverse_mass));
                bodies.body1.rotation_delta = sub3(bodies.body1.rotation_delta, scale3(self.inv_i1_r1_plus_u_x_axis, lambda));
            }
            if (bodies.body2.is_dynamic) {
                bodies.body2.position_delta = add3(bodies.body2.position_delta, scale3(world_axis, lambda * bodies.body2.inverse_mass));
                bodies.body2.rotation_delta = add3(bodies.body2.rotation_delta, scale3(self.inv_i2_r2x_axis, lambda));
            }
            return true;
        }
        return false;
    }
};

//=============================================================================
// SwingTwistConstraintPart
//=============================================================================

pub const SwingType = enum { cone, pyramid };

const RotationFlags = packed struct(u8) {
    twist_x_locked: bool = false,
    swing_y_locked: bool = false,
    swing_z_locked: bool = false,
    twist_x_free: bool = false,
    swing_y_free: bool = false,
    swing_z_free: bool = false,
    _reserved: u2 = 0,
};

/// The clamped-axis bits `clampSwingTwist` reports.
pub const ClampedAxis = packed struct(u32) {
    twist_min: bool = false,
    twist_max: bool = false,
    swing_y_min: bool = false,
    swing_y_max: bool = false,
    swing_z_min: bool = false,
    swing_z_max: bool = false,
    _reserved: u26 = 0,
};

/// Quaternion-based swing/twist constraint: `q = q_swing * q_twist`, twist
/// limited around the local X axis, swing limited to a cone or pyramid over Y
/// and Z. `SwingTwistConstraintPart.h`.
pub const SwingTwistConstraintPart = struct {
    // configuration
    rotation_flags: RotationFlags = .{},
    swing_type: SwingType = .cone,
    sin_twist_half_min_angle: f32 = 0,
    sin_twist_half_max_angle: f32 = 0,
    cos_twist_half_min_angle: f32 = 1,
    cos_twist_half_max_angle: f32 = 1,
    swing_y_half_min_angle: f32 = 0,
    swing_y_half_max_angle: f32 = 0,
    swing_z_half_min_angle: f32 = 0,
    swing_z_half_max_angle: f32 = 0,
    sin_swing_y_half_min_angle: f32 = 0,
    sin_swing_y_half_max_angle: f32 = 0,
    sin_swing_z_half_min_angle: f32 = 0,
    sin_swing_z_half_max_angle: f32 = 0,
    cos_swing_y_half_min_angle: f32 = 1,
    cos_swing_y_half_max_angle: f32 = 1,
    cos_swing_z_half_min_angle: f32 = 1,
    cos_swing_z_half_max_angle: f32 = 1,

    // run time
    world_space_swing_limit_y_rotation_axis: Vec3 = vec3_zero,
    world_space_swing_limit_z_rotation_axis: Vec3 = vec3_zero,
    world_space_twist_limit_rotation_axis: Vec3 = vec3_zero,
    swing_limit_y: AngleConstraintPart = .{},
    swing_limit_z: AngleConstraintPart = .{},
    twist_limit: AngleConstraintPart = .{},

    pub fn setSwingType(self: *SwingTwistConstraintPart, swing_type: SwingType) void {
        self.swing_type = swing_type;
    }
    pub fn getSwingType(self: SwingTwistConstraintPart) SwingType {
        return self.swing_type;
    }

    pub fn setLimits(self: *SwingTwistConstraintPart, twist_min_angle: f32, twist_max_angle: f32, swing_y_min_angle: f32, swing_y_max_angle: f32, swing_z_min_angle: f32, swing_z_max_angle: f32) void {
        const locked_angle = std.math.degreesToRadians(0.5);
        const free_angle = std.math.degreesToRadians(179.5);

        self.swing_y_half_min_angle = 0.5 * swing_y_min_angle;
        self.swing_y_half_max_angle = 0.5 * swing_y_max_angle;
        self.swing_z_half_min_angle = 0.5 * swing_z_min_angle;
        self.swing_z_half_max_angle = 0.5 * swing_z_max_angle;

        self.rotation_flags = .{};

        if (twist_min_angle > -locked_angle and twist_max_angle < locked_angle) {
            self.rotation_flags.twist_x_locked = true;
            self.sin_twist_half_min_angle = 0;
            self.sin_twist_half_max_angle = 0;
            self.cos_twist_half_min_angle = 1;
            self.cos_twist_half_max_angle = 1;
        } else if (twist_min_angle < -free_angle and twist_max_angle > free_angle) {
            self.rotation_flags.twist_x_free = true;
            self.sin_twist_half_min_angle = -1;
            self.sin_twist_half_max_angle = 1;
            self.cos_twist_half_min_angle = 0;
            self.cos_twist_half_max_angle = 0;
        } else {
            self.sin_twist_half_min_angle = @sin(0.5 * twist_min_angle);
            self.sin_twist_half_max_angle = @sin(0.5 * twist_max_angle);
            self.cos_twist_half_min_angle = @cos(0.5 * twist_min_angle);
            self.cos_twist_half_max_angle = @cos(0.5 * twist_max_angle);
        }

        if (swing_y_min_angle > -locked_angle and swing_y_max_angle < locked_angle) {
            self.rotation_flags.swing_y_locked = true;
            self.sin_swing_y_half_min_angle = 0;
            self.sin_swing_y_half_max_angle = 0;
            self.cos_swing_y_half_min_angle = 1;
            self.cos_swing_y_half_max_angle = 1;
        } else if (swing_y_min_angle < -free_angle and swing_y_max_angle > free_angle) {
            self.rotation_flags.swing_y_free = true;
            self.sin_swing_y_half_min_angle = -1;
            self.sin_swing_y_half_max_angle = 1;
            self.cos_swing_y_half_min_angle = 0;
            self.cos_swing_y_half_max_angle = 0;
        } else {
            self.sin_swing_y_half_min_angle = @sin(self.swing_y_half_min_angle);
            self.sin_swing_y_half_max_angle = @sin(self.swing_y_half_max_angle);
            self.cos_swing_y_half_min_angle = @cos(self.swing_y_half_min_angle);
            self.cos_swing_y_half_max_angle = @cos(self.swing_y_half_max_angle);
        }

        if (swing_z_min_angle > -locked_angle and swing_z_max_angle < locked_angle) {
            self.rotation_flags.swing_z_locked = true;
            self.sin_swing_z_half_min_angle = 0;
            self.sin_swing_z_half_max_angle = 0;
            self.cos_swing_z_half_min_angle = 1;
            self.cos_swing_z_half_max_angle = 1;
        } else if (swing_z_min_angle < -free_angle and swing_z_max_angle > free_angle) {
            self.rotation_flags.swing_z_free = true;
            self.sin_swing_z_half_min_angle = -1;
            self.sin_swing_z_half_max_angle = 1;
            self.cos_swing_z_half_min_angle = 0;
            self.cos_swing_z_half_max_angle = 0;
        } else {
            self.sin_swing_z_half_min_angle = @sin(self.swing_z_half_min_angle);
            self.sin_swing_z_half_max_angle = @sin(self.swing_z_half_max_angle);
            self.cos_swing_z_half_min_angle = @cos(self.swing_z_half_min_angle);
            self.cos_swing_z_half_max_angle = @cos(self.swing_z_half_max_angle);
        }
    }

    fn distanceToMinShorter(delta_min_in: f32, delta_max_in: f32) bool {
        var delta_min = @abs(delta_min_in);
        if (delta_min > 1.0) delta_min = 2.0 - delta_min;
        var delta_max = @abs(delta_max_in);
        if (delta_max > 1.0) delta_max = 2.0 - delta_max;
        return delta_min < delta_max;
    }

    /// Clamps `swing`/`twist` (both assumed in constraint space) against the
    /// limits, in place, and reports which axes were clamped.
    pub fn clampSwingTwist(self: SwingTwistConstraintPart, swing_in: Quat, twist_in: Quat) struct { swing: Quat, twist: Quat, clamped: ClampedAxis } {
        var clamped: ClampedAxis = .{};

        const negate_swing = swing_in.w < 0.0;
        var swing = if (negate_swing) qneg(swing_in) else swing_in;
        const negate_twist = twist_in.w < 0.0;
        var twist = if (negate_twist) qneg(twist_in) else twist_in;

        if (self.rotation_flags.twist_x_locked) {
            if (twist.x != 0.0) {
                clamped.twist_min = true;
                clamped.twist_max = true;
            }
            twist = quat_identity;
        } else if (!self.rotation_flags.twist_x_free) {
            const delta_min = self.sin_twist_half_min_angle - twist.x;
            const delta_max = twist.x - self.sin_twist_half_max_angle;
            if (delta_min > 0.0 or delta_max > 0.0) {
                if (distanceToMinShorter(delta_min, delta_max)) {
                    twist = .{ .x = self.sin_twist_half_min_angle, .y = 0, .z = 0, .w = self.cos_twist_half_min_angle };
                    clamped.twist_min = true;
                } else {
                    twist = .{ .x = self.sin_twist_half_max_angle, .y = 0, .z = 0, .w = self.cos_twist_half_max_angle };
                    clamped.twist_max = true;
                }
            }
        }

        if (self.rotation_flags.swing_y_locked) {
            if (self.rotation_flags.swing_z_locked) {
                if (swing.y != 0.0) {
                    clamped.swing_y_min = true;
                    clamped.swing_y_max = true;
                }
                if (swing.z != 0.0) {
                    clamped.swing_z_min = true;
                    clamped.swing_z_max = true;
                }
                swing = quat_identity;
            } else {
                if (swing.y != 0.0) {
                    clamped.swing_y_min = true;
                    clamped.swing_y_max = true;
                }
                const delta_min = self.sin_swing_z_half_min_angle - swing.z;
                const delta_max = swing.z - self.sin_swing_z_half_max_angle;
                if (delta_min > 0.0 or delta_max > 0.0) {
                    if (distanceToMinShorter(delta_min, delta_max)) {
                        swing = .{ .x = 0, .y = 0, .z = self.sin_swing_z_half_min_angle, .w = self.cos_swing_z_half_min_angle };
                        clamped.swing_z_min = true;
                    } else {
                        swing = .{ .x = 0, .y = 0, .z = self.sin_swing_z_half_max_angle, .w = self.cos_swing_z_half_max_angle };
                        clamped.swing_z_max = true;
                    }
                } else if (clamped.swing_y_min) {
                    const z = swing.z;
                    swing = .{ .x = 0, .y = 0, .z = z, .w = @sqrt(1.0 - z * z) };
                }
            }
        } else if (self.rotation_flags.swing_z_locked) {
            if (swing.z != 0.0) {
                clamped.swing_z_min = true;
                clamped.swing_z_max = true;
            }
            const delta_min = self.sin_swing_y_half_min_angle - swing.y;
            const delta_max = swing.y - self.sin_swing_y_half_max_angle;
            if (delta_min > 0.0 or delta_max > 0.0) {
                if (distanceToMinShorter(delta_min, delta_max)) {
                    swing = .{ .x = 0, .y = self.sin_swing_y_half_min_angle, .z = 0, .w = self.cos_swing_y_half_min_angle };
                    clamped.swing_y_min = true;
                } else {
                    swing = .{ .x = 0, .y = self.sin_swing_y_half_max_angle, .z = 0, .w = self.cos_swing_y_half_max_angle };
                    clamped.swing_y_max = true;
                }
            } else if (clamped.swing_z_min) {
                const y = swing.y;
                swing = .{ .x = 0, .y = y, .z = 0, .w = @sqrt(1.0 - y * y) };
            }
        } else if (self.swing_type == .cone) {
            const point = Vec2{ .x = swing.y, .y = swing.z };
            if (!ellipseIsInside(self.sin_swing_y_half_max_angle, self.sin_swing_z_half_max_angle, point)) {
                const closest = ellipseClosestPoint(self.sin_swing_y_half_max_angle, self.sin_swing_z_half_max_angle, point);
                swing = .{ .x = 0, .y = closest.x, .z = closest.y, .w = @sqrt(@max(0.0, 1.0 - closest.x * closest.x - closest.y * closest.y)) };
                clamped.swing_y_min = true;
                clamped.swing_y_max = true;
                clamped.swing_z_min = true;
                clamped.swing_z_max = true;
            }
        } else {
            // Pyramid: q = Rz(z) * Ry(y), so y/2 = atan2(q.y, q.w), z/2 =
            // atan2(q.z, q.w) — see the derivation in the header.
            const half_y = std.math.atan2(swing.y, swing.w);
            const half_z = std.math.atan2(swing.z, swing.w);
            const clamped_half_y = std.math.clamp(half_y, self.swing_y_half_min_angle, self.swing_y_half_max_angle);
            const clamped_half_z = std.math.clamp(half_z, self.swing_z_half_min_angle, self.swing_z_half_max_angle);
            if (half_y != clamped_half_y or half_z != clamped_half_z) {
                const sy = @sin(clamped_half_y);
                const cy = @cos(clamped_half_y);
                const sz = @sin(clamped_half_z);
                const cz = @cos(clamped_half_z);
                swing = qNormalized(.{ .x = 0, .y = sy * cz, .z = cy * sz, .w = cy * cz });
                clamped.swing_y_min = true;
                clamped.swing_y_max = true;
                clamped.swing_z_min = true;
                clamped.swing_z_max = true;
            }
        }

        if (negate_swing) swing = qneg(swing);
        if (negate_twist) twist = qneg(twist);

        return .{ .swing = swing, .twist = twist, .clamped = clamped };
    }

    pub fn calculateConstraintProperties(self: *SwingTwistConstraintPart, body1: *const SolverBody, body2: *const SolverBody, constraint_rotation: Quat, constraint_to_world: Quat) void {
        const decomposed = quatGetSwingTwist(constraint_rotation);
        const q_swing = decomposed.swing;
        const q_twist = decomposed.twist;
        const clamp_result = self.clampSwingTwist(q_swing, q_twist);
        const q_clamped_swing = clamp_result.swing;
        const clamped = clamp_result.clamped;

        if (self.rotation_flags.swing_y_locked) {
            const twist_to_world = qmul(constraint_to_world, q_swing);
            self.world_space_swing_limit_y_rotation_axis = qRotateAxisY(twist_to_world);
            self.world_space_swing_limit_z_rotation_axis = qRotateAxisZ(twist_to_world);

            if (self.rotation_flags.swing_z_locked) {
                self.swing_limit_y.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_y_rotation_axis, 0);
                self.swing_limit_z.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_z_rotation_axis, 0);
            } else {
                self.swing_limit_y.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_y_rotation_axis, 0);
                if (clamped.swing_z_min or clamped.swing_z_max) {
                    if (clamped.swing_z_min) self.world_space_swing_limit_z_rotation_axis = neg3(self.world_space_swing_limit_z_rotation_axis);
                    self.swing_limit_z.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_z_rotation_axis, 0);
                } else {
                    self.swing_limit_z.deactivate();
                }
            }
        } else if (self.rotation_flags.swing_z_locked) {
            const twist_to_world = qmul(constraint_to_world, q_swing);
            self.world_space_swing_limit_y_rotation_axis = qRotateAxisY(twist_to_world);
            self.world_space_swing_limit_z_rotation_axis = qRotateAxisZ(twist_to_world);

            if (clamped.swing_y_min or clamped.swing_y_max) {
                if (clamped.swing_y_min) self.world_space_swing_limit_y_rotation_axis = neg3(self.world_space_swing_limit_y_rotation_axis);
                self.swing_limit_y.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_y_rotation_axis, 0);
            } else {
                self.swing_limit_y.deactivate();
            }
            self.swing_limit_z.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_z_rotation_axis, 0);
        } else if (!(self.rotation_flags.swing_y_free and self.rotation_flags.swing_z_free)) {
            if (clamped.swing_y_min or clamped.swing_y_max or clamped.swing_z_min or clamped.swing_z_max) {
                const current = qRotateAxisX(qmul(constraint_to_world, q_swing));
                const desired = qRotateAxisX(qmul(constraint_to_world, q_clamped_swing));
                self.world_space_swing_limit_y_rotation_axis = cross3(desired, current);
                const len = @sqrt(lengthSq3(self.world_space_swing_limit_y_rotation_axis));
                if (len != 0.0) {
                    self.world_space_swing_limit_y_rotation_axis = scale3(self.world_space_swing_limit_y_rotation_axis, 1.0 / len);
                    self.swing_limit_y.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_y_rotation_axis, 0);
                } else {
                    self.swing_limit_y.deactivate();
                }
            } else {
                self.swing_limit_y.deactivate();
            }
            self.swing_limit_z.deactivate();
        } else {
            self.swing_limit_y.deactivate();
            self.swing_limit_z.deactivate();
        }

        if (self.rotation_flags.twist_x_locked) {
            self.world_space_twist_limit_rotation_axis = qRotateAxisX(qmul(constraint_to_world, q_swing));
            self.twist_limit.calculateConstraintProperties(body1, body2, self.world_space_twist_limit_rotation_axis, 0);
        } else if (!self.rotation_flags.twist_x_free) {
            if (clamped.twist_min or clamped.twist_max) {
                self.world_space_twist_limit_rotation_axis = qRotateAxisX(qmul(constraint_to_world, q_swing));
                if (clamped.twist_min) self.world_space_twist_limit_rotation_axis = neg3(self.world_space_twist_limit_rotation_axis);
                self.twist_limit.calculateConstraintProperties(body1, body2, self.world_space_twist_limit_rotation_axis, 0);
            } else {
                self.twist_limit.deactivate();
            }
        } else {
            self.twist_limit.deactivate();
        }
    }

    pub fn deactivate(self: *SwingTwistConstraintPart) void {
        self.swing_limit_y.deactivate();
        self.swing_limit_z.deactivate();
        self.twist_limit.deactivate();
    }

    pub fn isActive(self: SwingTwistConstraintPart) bool {
        return self.swing_limit_y.isActive() or self.swing_limit_z.isActive() or self.twist_limit.isActive();
    }

    pub fn warmStart(self: *SwingTwistConstraintPart, bodies: *SolverBodyPair, warm_start_impulse_ratio: f32) void {
        self.swing_limit_y.warmStart(bodies, warm_start_impulse_ratio);
        self.swing_limit_z.warmStart(bodies, warm_start_impulse_ratio);
        self.twist_limit.warmStart(bodies, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *SwingTwistConstraintPart, bodies: *SolverBodyPair) bool {
        var impulse = false;

        if (self.swing_limit_y.isActive())
            impulse = self.swing_limit_y.solveVelocityConstraint(bodies, self.world_space_swing_limit_y_rotation_axis, -std.math.floatMax(f32), if (self.sin_swing_y_half_min_angle == self.sin_swing_y_half_max_angle) std.math.floatMax(f32) else 0.0) or impulse;

        if (self.swing_limit_z.isActive())
            impulse = self.swing_limit_z.solveVelocityConstraint(bodies, self.world_space_swing_limit_z_rotation_axis, -std.math.floatMax(f32), if (self.sin_swing_z_half_min_angle == self.sin_swing_z_half_max_angle) std.math.floatMax(f32) else 0.0) or impulse;

        if (self.twist_limit.isActive())
            impulse = self.twist_limit.solveVelocityConstraint(bodies, self.world_space_twist_limit_rotation_axis, -std.math.floatMax(f32), if (self.sin_twist_half_min_angle == self.sin_twist_half_max_angle) std.math.floatMax(f32) else 0.0) or impulse;

        return impulse;
    }

    pub fn solvePositionConstraint(self: *const SwingTwistConstraintPart, bodies: *SolverBodyPair, constraint_rotation: Quat, constraint_to_body1: Quat, constraint_to_body2: Quat, baumgarte: f32) bool {
        const decomposed = quatGetSwingTwist(constraint_rotation);
        const clamp_result = self.clampSwingTwist(decomposed.swing, decomposed.twist);
        if (@as(u32, @bitCast(clamp_result.clamped)) != 0) {
            var part: RotationEulerConstraintPart = .{};
            const inv_initial_orientation = qmul(constraint_to_body2, qconj(qmul(qmul(constraint_to_body1, clamp_result.swing), clamp_result.twist)));
            part.calculateConstraintProperties(&bodies.body1, &bodies.body2);
            return part.solvePositionConstraint(bodies, inv_initial_orientation, baumgarte);
        }
        return false;
    }

    pub fn getTotalSwingYLambda(self: SwingTwistConstraintPart) f32 {
        return self.swing_limit_y.getTotalLambda();
    }
    pub fn getTotalSwingZLambda(self: SwingTwistConstraintPart) f32 {
        return self.swing_limit_z.getTotalLambda();
    }
    pub fn getTotalTwistLambda(self: SwingTwistConstraintPart) f32 {
        return self.twist_limit.getTotalLambda();
    }

    pub fn saveState(self: SwingTwistConstraintPart, recorder: *StateRecorder) void {
        self.swing_limit_y.saveState(recorder);
        self.swing_limit_z.saveState(recorder);
        self.twist_limit.saveState(recorder);
    }
    pub fn restoreState(self: *SwingTwistConstraintPart, recorder: *StateRecorder) void {
        self.swing_limit_y.restoreState(recorder);
        self.swing_limit_z.restoreState(recorder);
        self.twist_limit.restoreState(recorder);
    }
};

/// `Ellipse::IsInside`, centred at the origin with radii `a`, `b`.
fn ellipseIsInside(a: f32, b: f32, point: Vec2) bool {
    return (point.x / a) * (point.x / a) + (point.y / b) * (point.y / b) <= 1.0;
}

/// `Ellipse::GetClosestPoint`. Assumes `point` is outside the ellipse. Newton
/// Raphson from `t = 0`, transcribed as an unbounded loop exactly as Jolt
/// writes it — this is the reference implementation's own convergence
/// behaviour, not something to second-guess with an iteration cap.
fn ellipseClosestPoint(a: f32, b: f32, point: Vec2) Vec2 {
    const a_sq = a * a;
    const b_sq = b * b;
    var t: f32 = 0.0;
    while (true) {
        const t_plus_a_sq = t + a_sq;
        const t_plus_b_sq = t + b_sq;
        const ax = a * point.x / t_plus_a_sq;
        const by = b * point.y / t_plus_b_sq;
        const gt = ax * ax + by * by - 1.0;
        if (@abs(gt) < 1.0e-6) {
            return .{ .x = a_sq * point.x / t_plus_a_sq, .y = b_sq * point.y / t_plus_b_sq };
        }
        const gt_accent = -2.0 * (a_sq * (point.x * point.x) / (t_plus_a_sq * t_plus_a_sq * t_plus_a_sq) +
            b_sq * (point.y * point.y) / (t_plus_b_sq * t_plus_b_sq * t_plus_b_sq));
        t = t - gt / gt_accent;
    }
}

//=============================================================================
// Compile coverage
//
// Zig only analyses a function body when something calls it or takes its address — most of these twelve parts have no caller in this package yet, so a real type error inside one would compile clean and stay hidden until a future host's custom constraint hits it. `forceAnalysis` below takes every public (and load-bearing private) declaration's address to force analysis, the same technique `src/analysis_test.zig` uses for the C-ABI wrappers, kept local here to also reach private helpers.
//=============================================================================

fn forceAnalysis(comptime T: type, comptime depth: u8) void {
    if (depth == 0) return;

    const decls = switch (@typeInfo(T)) {
        .@"struct" => |s| s.decls,
        .@"union" => |u| u.decls,
        .@"enum" => |e| e.decls,
        .@"opaque" => |o| o.decls,
        else => return,
    };

    inline for (decls) |d| {
        const field = @field(T, d.name);
        const FieldType = @TypeOf(field);

        if (FieldType == type) {
            forceAnalysis(field, depth - 1);
        } else if (@typeInfo(FieldType) == .@"fn") {
            _ = &field;
        } else {
            _ = &field;
        }
    }
}

test "every ConstraintPart, and the private helpers it is built from, compiles" {
    @setEvalBranchQuota(2_000_000);
    comptime forceAnalysis(@This(), 4);
}
