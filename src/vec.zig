//! The value algebra behind `Vec3`, `Quat` and `Mat44`, over
//! `@Vector(4, f32)`.
//!
//! One home for it. `src/c/core.zig` and `src/math.zig` each carried their own
//! copy of add/sub/dot/cross, which is two definitions of one fact.
//!
//! Every function here mirrors a Jolt operation, quirks included: `quatInverse`
//! divides by the LENGTH the way `Quat::Inversed` does, and `mat44Point` adds
//! the translation column last rather than fusing it, the way Jolt's own
//! comment says to. `src/vec_test.zig` proves each one against the C entry
//! point that reaches Jolt, which is what keeps two implementations honest.
//!
//! Plain arrays in, plain arrays out: this file must not import the ABI
//! structs, because those structs' methods import this.

/// Four lanes. A `Vec3` occupies the low three with an unused w, matching
/// `JPH::Vec3`, so the same operations serve both.
pub const V4 = @Vector(4, f32);

pub const zero: V4 = @splat(0);

/// A `Vec3`- or `Quat`-shaped struct widened to four lanes. Duck-typed on
/// the field names rather than on the type, because those structs' methods
/// import this file and naming their types here would close the loop.
pub inline fn from(v: anytype) V4 {
    const T = @TypeOf(v);
    if (@hasField(T, "w")) return .{ v.x, v.y, v.z, v.w };
    return .{ v.x, v.y, v.z, 0 };
}

/// The inverse of `from`. `T` decides how many lanes are read.
pub inline fn to(comptime T: type, v: V4) T {
    if (@hasField(T, "w")) return .{ .x = v[0], .y = v[1], .z = v[2], .w = v[3] };
    return .{ .x = v[0], .y = v[1], .z = v[2] };
}

pub inline fn dot3(a: V4, b: V4) f32 {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

pub inline fn dot4(a: V4, b: V4) f32 {
    return @reduce(.Add, a * b);
}

pub inline fn lengthSq3(a: V4) f32 {
    return dot3(a, a);
}

pub inline fn lengthSq4(a: V4) f32 {
    return dot4(a, a);
}

pub inline fn cross3(a: V4, b: V4) V4 {
    const a_yzx = @shuffle(f32, a, undefined, [4]i32{ 1, 2, 0, 3 });
    const b_yzx = @shuffle(f32, b, undefined, [4]i32{ 1, 2, 0, 3 });
    const t = a * b_yzx - a_yzx * b;
    return @shuffle(f32, t, undefined, [4]i32{ 1, 2, 0, 3 });
}

/// FMA-compensated `a*b - c*d`, matching Jolt's `sDifferenceOfProducts` under
/// `JPH_USE_FMADD` — the extra error term is what makes `crossPrecise` (and so
/// `Mat44.determinant3x3`) more accurate than a plain cross product.
pub inline fn differenceOfProducts(a: V4, b: V4, c: V4, d: V4) V4 {
    const cd = c * d;
    const compensation = mulAdd(-c, d, cd);
    const dop = mulAdd(a, b, -cd);
    return dop + compensation;
}

/// Jolt's `Vec3::CrossPrecise`: a cross product built from
/// `differenceOfProducts` instead of the plain `a.y*b.z - a.z*b.y` form, so it
/// stays accurate where the plain product cancels badly.
pub inline fn crossPrecise(a: V4, b: V4) V4 {
    const a_yzx = @shuffle(f32, a, undefined, [4]i32{ 1, 2, 0, 3 });
    const b_yzx = @shuffle(f32, b, undefined, [4]i32{ 1, 2, 0, 3 });
    const diff = differenceOfProducts(a, b_yzx, a_yzx, b);
    return @shuffle(f32, diff, undefined, [4]i32{ 1, 2, 0, 3 });
}

inline fn mulAdd(a: V4, b: V4, c: V4) V4 {
    return .{
        @mulAdd(f32, a[0], b[0], c[0]),
        @mulAdd(f32, a[1], b[1], c[1]),
        @mulAdd(f32, a[2], b[2], c[2]),
        @mulAdd(f32, a[3], b[3], c[3]),
    };
}

/// `a + (b - a) * t`, the form `zjoltVec3Lerp` computes. `quatLerp` spells
/// the same function the other way; the difference is below the tolerance
/// `src/vec_test.zig` compares at, so each is written the way its side is.
pub inline fn lerp(a: V4, b: V4, t: f32) V4 {
    const ts: V4 = @splat(t);
    return a + (b - a) * ts;
}

//=============================================================================
// Quaternions. (x, y, z, w) with w LAST, matching Jolt.
//=============================================================================

/// Jolt's `Quat::operator*`, shuffle for shuffle: the Hamilton product with
/// the logical last component negated. Written out this way rather than as
/// four scalar rows because that is what makes it one SIMD sequence.
pub inline fn quatMul(lhs: V4, rhs: V4) V4 {
    const abca = @shuffle(f32, lhs, undefined, [4]i32{ 0, 1, 2, 0 });
    const bcab = @shuffle(f32, lhs, undefined, [4]i32{ 1, 2, 0, 1 });
    const cabc = @shuffle(f32, lhs, undefined, [4]i32{ 2, 0, 1, 2 });
    const dddd: V4 = @splat(lhs[3]);
    const wwwx = @shuffle(f32, rhs, undefined, [4]i32{ 3, 3, 3, 0 });
    const zxyy = @shuffle(f32, rhs, undefined, [4]i32{ 2, 0, 1, 1 });
    const yzxz = @shuffle(f32, rhs, undefined, [4]i32{ 1, 2, 0, 2 });
    const flip_w: V4 = .{ 1, 1, 1, -1 };
    const m3 = (abca * wwwx + bcab * zxyy) * flip_w;
    return dddd * rhs - cabc * yzxz + m3;
}

pub inline fn quatConjugate(q: V4) V4 {
    return q * V4{ -1, -1, -1, 1 };
}

/// Jolt's `Quat::Inversed`, which is `Conjugated() / Length()` — NOT divided
/// by the squared length. For a unit quaternion the two agree; for any other
/// this is Jolt's answer, and matching it is the point.
pub inline fn quatInverse(q: V4) V4 {
    const len: V4 = @splat(@sqrt(lengthSq4(q)));
    return quatConjugate(q) / len;
}

pub inline fn quatNormalize(q: V4) V4 {
    const len: V4 = @splat(@sqrt(lengthSq4(q)));
    return q / len;
}

/// Whether `q`'s squared length is within `tolerance` of 1. Jolt's own
/// default tolerance is `1.0e-5`.
pub inline fn quatIsNormalized(q: V4, tolerance: f32) bool {
    return @abs(lengthSq4(q) - 1.0) <= tolerance;
}

/// `(1 - t) * a + t * b`, the form `Quat::LERP` computes. See `lerp` for why
/// the other spelling is a separate function and not a shared one.
pub inline fn quatLerp(a: V4, b: V4, t: f32) V4 {
    const scale0: V4 = @splat(1.0 - t);
    const scale1: V4 = @splat(t);
    return scale0 * a + scale1 * b;
}

/// Jolt's `Quat::operator*(Vec3)`: the Euler-Rodrigues form, written out the
/// way `Quat.inl` writes it. `q` must be unit length — Jolt asserts it, and
/// the caller is what enforces it.
pub inline fn quatRotate(q: V4, v: V4) V4 {
    const xyz = V4{ q[0], q[1], q[2], 0 };
    const yzx = @shuffle(f32, xyz, undefined, [4]i32{ 1, 2, 0, 3 });
    const v_yzx = @shuffle(f32, v, undefined, [4]i32{ 1, 2, 0, 3 });
    const q_cross_p_pre = v_yzx * xyz - yzx * v;
    const q_cross_p = @shuffle(f32, q_cross_p_pre, undefined, [4]i32{ 1, 2, 0, 3 });
    const qcp_yzx = @shuffle(f32, q_cross_p, undefined, [4]i32{ 1, 2, 0, 3 });
    const q_cross_q_cross_p_pre = qcp_yzx * xyz - yzx * q_cross_p;
    const q_cross_q_cross_p = @shuffle(f32, q_cross_q_cross_p_pre, undefined, [4]i32{ 1, 2, 0, 3 });
    const w: V4 = @splat(q[3]);
    const t = w * q_cross_p + q_cross_q_cross_p;
    return v + (t + t);
}

//=============================================================================
// 4x4 matrices, as four COLUMNS: column `c`'s row `r` is `m[4 * c + r]`,
// column-major the way Jolt stores them.
//=============================================================================

pub const Cols = [4]V4;

pub inline fn loadCols(m: [16]f32) Cols {
    return .{
        .{ m[0], m[1], m[2], m[3] },
        .{ m[4], m[5], m[6], m[7] },
        .{ m[8], m[9], m[10], m[11] },
        .{ m[12], m[13], m[14], m[15] },
    };
}

pub inline fn storeCols(c: Cols) [16]f32 {
    var out: [16]f32 = undefined;
    inline for (0..4) |i| {
        inline for (0..4) |j| out[4 * i + j] = c[i][j];
    }
    return out;
}

/// `a * b`: each column of `b` combined out of `a`'s four columns, which is
/// Jolt's `Mat44::operator*` column for column.
pub inline fn mat44Mul(a: Cols, b: Cols) Cols {
    var out: Cols = undefined;
    inline for (0..4) |i| {
        const x: V4 = @splat(b[i][0]);
        const y: V4 = @splat(b[i][1]);
        const z: V4 = @splat(b[i][2]);
        const w: V4 = @splat(b[i][3]);
        out[i] = a[0] * x + a[1] * y + a[2] * z + a[3] * w;
    }
    return out;
}

/// A point through the full transform. The translation column is ADDED, never
/// folded into the first multiply as an FMA: Jolt's own comment says fusing it
/// costs precision, and this has to agree with Jolt.
pub inline fn mat44Point(m: Cols, v: V4) V4 {
    const x: V4 = @splat(v[0]);
    const y: V4 = @splat(v[1]);
    const z: V4 = @splat(v[2]);
    const t = m[0] * x + m[1] * y + m[2] * z;
    return t + m[3];
}

/// A direction through the rotation and scale only — Jolt's `Multiply3x3`.
pub inline fn mat44Direction(m: Cols, v: V4) V4 {
    const x: V4 = @splat(v[0]);
    const y: V4 = @splat(v[1]);
    const z: V4 = @splat(v[2]);
    return m[0] * x + m[1] * y + m[2] * z;
}

pub inline fn mat44Transposed3x3(m: Cols) Cols {
    var out: Cols = .{ zero, zero, zero, .{ 0, 0, 0, 1 } };
    inline for (0..3) |c| {
        inline for (0..3) |r| out[c][r] = m[r][c];
    }
    return out;
}

/// The inverse of a matrix that is a rotation and a translation and nothing
/// else — Jolt's `InversedRotationTranslation`: transpose the 3x3, then run
/// the old translation back through it negated. Wrong for a scaled or sheared
/// matrix, exactly as it is in Jolt.
pub inline fn mat44InverseRotationTranslation(m: Cols) Cols {
    var out = mat44Transposed3x3(m);
    const t = mat44Direction(out, m[3]);
    out[3] = .{ -t[0], -t[1], -t[2], 1 };
    return out;
}
