//! Host-defined shapes: a Zig (or any C) host implements Jolt's own shape
//! interface without touching C++.
//!
//! Layer A (`initCustomConvex`) needs only a support function — GJK and EPA
//! do the rest, the same as every built-in convex primitive. It is the
//! common case: most custom shapes are convex. Layer B (`initCustom`) is for
//! a shape that is not, and binds the full `JPH::Shape` interface, so every
//! collision entry point is required rather than derived from a support
//! function.
//!
//! Every crossing is either once per query or batched, never once per
//! primitive inside a broadphase walk. @see `ffi/zjolt_customshape.h`.

const c = @import("c/customshape.zig");
const core = @import("c/core.zig");
const err = @import("error.zig");
const material_mod = @import("material.zig");
const shape_mod = @import("shape.zig");

/// The raw C declarations, for a signature (a triangle-walk context, a
/// two-call result) nothing above already re-exports.
pub const c_decls = c;

pub const Shape = shape_mod.Shape;
pub const PhysicsMaterial = material_mod.PhysicsMaterial;
pub const ConvexShapeCallbacks = c.ConvexShapeCallbacks;
pub const ShapeCallbacks = c.ShapeCallbacks;
pub const CustomShapeRayHit = c.CustomShapeRayHit;
pub const CustomShapeChild = c.CustomShapeChild;
pub const Result = c.Result;
pub const RayCastSettings = c.RayCastSettings;
pub const ShapeTrianglesContext = c.ShapeTrianglesContext;
pub const Stream = core.Stream;

/// Results a batched query can report in one call: `cast_ray_all`,
/// `collide_point`, `collide_soft_body_vertices`, `collect_transformed_shapes`,
/// `transform_shape`, `save_material_state` and `save_sub_shape_state` each
/// write at most this many entries.
pub const max_batch: u32 = c.custom_shape_max_batch;

/// A custom convex shape — the common case. `callbacks.support`,
/// `inner_radius`, `local_bounds`, `mass_properties` and `volume` are required;
/// a null one is `error.InvalidArgument`. `material` null means Jolt's shared
/// default, as with `Shape.init*`. Non-uniform (mirrored) scale needs no extra
/// effort: the shim scales `support`'s input direction and output point by the
/// same diagonal factor — the linear-scale support-function identity.
pub fn initCustomConvex(
    callbacks: ConvexShapeCallbacks,
    user: ?*anyopaque,
    material: ?PhysicsMaterial,
) err.Error!Shape {
    var handle: *c.Shape = undefined;
    try err.check(c.zjoltShapeCreateCustomConvex(
        &callbacks,
        user,
        if (material) |m| m.handle else null,
        &handle,
    ));
    return .{ .handle = handle };
}

/// A general custom shape, for non-convex geometry: no support function, so no
/// GJK/EPA, so every entry in `ShapeCallbacks`'s "required" half is mandatory,
/// not derived. Leaving one null is `error.InvalidArgument`. Either
/// constructor's shape does not survive `zjoltShapeSave`/`Shape.save`
/// round-tripping (callbacks are host code, not data): save succeeds, restore
/// returns an inert placeholder (zero volume, zero bounds), never crashing.
pub fn initCustom(callbacks: ShapeCallbacks, user: ?*anyopaque) err.Error!Shape {
    var handle: *c.Shape = undefined;
    try err.check(c.zjoltShapeCreateCustom(&callbacks, user, &handle));
    return .{ .handle = handle };
}
