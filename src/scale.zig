//! Scale helpers ported from Jolt's `ScaleHelpers` namespace (vendored at
//! `Jolt/Physics/Collision/Shape/ScaleHelpers.h`).
//!
//! Pure `Vec3`/`Quat` arithmetic, so — like the tail of `math.zig` — nothing
//! here calls into the C ABI and none of it needs `zjolt.init`.

const math = @import("math.zig");

pub const Vec3 = math.Vec3;
pub const Quat = math.Quat;

/// `ScaleHelpers::cMinScale`: the floor `isZeroScale` refuses.
pub const min_scale: f32 = 1.0e-6;

/// `ScaleHelpers::cScaleToleranceSq`: how close two scale components have to
/// be to count as equal.
pub const scale_tolerance_sq: f32 = 1.0e-8;

/// `PhysicsSettings.h`'s `cDefaultConvexRadius`, the ceiling `convexRadius`
/// never exceeds.
pub const default_convex_radius: f32 = 0.05;

/// Whether `scale` is (1, 1, 1), within `scale_tolerance_sq`.
pub fn isNotScaled(scale: Vec3) bool {
    return scale.isClose(.{ .x = 1, .y = 1, .z = 1 }, scale_tolerance_sq);
}

/// Whether `scale`'s three components equal each other.
pub fn isUniformScale(scale: Vec3) bool {
    const swizzled: Vec3 = .{ .x = scale.y, .y = scale.z, .z = scale.x };
    return swizzled.isClose(scale, scale_tolerance_sq);
}

/// Whether `scale` is uniform between X and Z; Y is unconstrained.
pub fn isUniformScaleXZ(scale: Vec3) bool {
    const swizzled: Vec3 = .{ .x = scale.z, .y = scale.y, .z = scale.x };
    return swizzled.isClose(scale, scale_tolerance_sq);
}

/// Whether any component's magnitude is below `min_scale` — the scale a
/// shape cannot take without dividing by (near) zero.
pub fn isZeroScale(scale: Vec3) bool {
    return @abs(scale.x) < min_scale or @abs(scale.y) < min_scale or @abs(scale.z) < min_scale;
}

/// The average of `scale`'s three components, replicated — what a shape that
/// cannot take non-uniform scale is given instead.
pub fn makeUniformScale(scale: Vec3) Vec3 {
    const avg = (scale.x + scale.y + scale.z) / 3.0;
    return .{ .x = avg, .y = avg, .z = avg };
}

/// As `makeUniformScale`, but only X and Z are averaged; Y passes through.
pub fn makeUniformScaleXZ(scale: Vec3) Vec3 {
    const xz = 0.5 * (scale.x + scale.z);
    return .{ .x = xz, .y = scale.y, .z = xz };
}

/// Scales a convex radius by the smallest-magnitude component of `scale`,
/// capped at `default_convex_radius` the way every convex shape's own scaled
/// radius is.
pub fn convexRadius(convex_radius: f32, scale: Vec3) f32 {
    const min_abs = @min(@min(@abs(scale.x), @abs(scale.y)), @abs(scale.z));
    return @min(convex_radius * min_abs, default_convex_radius);
}

//=============================================================================
// Rotating a scale into a child shape's local space
//
// A parent's scale is diagonal in the PARENT's axes. Rotate the child inside
// it and the same scale, read in the CHILD's axes, is `R^T * diag(scale) * R`
// — diagonal again only when that conjugation introduces no shear, which is
// what `canScaleBeRotated` checks and `rotateScale` assumes.
//=============================================================================

/// The 3x3 conjugation `R^T * diag(scale) * R`. `R`'s columns are
/// `rotation.rotateAxisX/Y/Z()` — the same columns `Mat44.sRotation(rotation)`
/// would produce, reused rather than re-derived. `rotation` MUST be unit
/// length.
fn conjugateScale(rotation: Quat, scale: Vec3) [3][3]f32 {
    const axis = [3]Vec3{
        rotation.rotateAxisX(),
        rotation.rotateAxisY(),
        rotation.rotateAxisZ(),
    };
    const s = [3]f32{ scale.x, scale.y, scale.z };
    var out: [3][3]f32 = undefined;
    inline for (0..3) |i| {
        const ai = [3]f32{ axis[i].x, axis[i].y, axis[i].z };
        inline for (0..3) |j| {
            const aj = [3]f32{ axis[j].x, axis[j].y, axis[j].z };
            out[i][j] = s[0] * ai[0] * aj[0] + s[1] * ai[1] * aj[1] + s[2] * ai[2] * aj[2];
        }
    }
    return out;
}

/// Whether rotating `scale` by `rotation` stays a scale — no off-diagonal
/// term of `conjugateScale` above `1e-6` in magnitude, i.e. no shear
/// introduced.
pub fn canScaleBeRotated(rotation: Quat, scale: Vec3) bool {
    const cs = conjugateScale(rotation, scale);
    const epsilon: f32 = 1.0e-6;
    return @abs(cs[0][1]) < epsilon and @abs(cs[0][2]) < epsilon and
        @abs(cs[1][0]) < epsilon and @abs(cs[1][2]) < epsilon and
        @abs(cs[2][0]) < epsilon and @abs(cs[2][1]) < epsilon;
}

/// `scale`, rotated into the frame `rotation` describes — `conjugateScale`'s
/// diagonal. Meaningless where `canScaleBeRotated` is false: the conjugation
/// then carries shear a diagonal scale cannot represent, and this drops it
/// silently, exactly as Jolt's own `RotateScale` does.
pub fn rotateScale(rotation: Quat, scale: Vec3) Vec3 {
    const cs = conjugateScale(rotation, scale);
    return .{ .x = cs[0][0], .y = cs[1][1], .z = cs[2][2] };
}

/// The scale a child shape sees when its parent carries `scale` and the
/// child is rotated by `rotation` relative to the parent —
/// `RotatedTranslatedShape::TransformScale` /
/// `CompoundShape::SubShape::TransformScale`, the same body given a
/// different `rotation`. Returns `scale` unchanged when it is uniform or
/// `rotation` is identity.
pub fn transformScale(rotation: Quat, scale: Vec3) Vec3 {
    if (isUniformScale(scale)) return scale;
    return rotateScale(rotation, scale);
}
