//! Physics materials.
//!
//! A material is an **identity**, not a property bag: a debug name and
//! colour, nothing else. Friction/restitution live on the body
//! (`BodyInterface.setFriction`/`setRestitution`); a contact combines the two bodies' values, never the two surfaces'.
//!
//! What it is for: telling surfaces apart *within* one shape. A mesh has
//! one material per triangle, a height field one per quad; a hit's
//! sub-shape id (via `Shape.material`) gives the exact triangle/quad's
//! material. Map the result to your own data by identity (`eql`, or a hash of `.handle`).
//!
//! A shape holds a reference on its materials (create, build, `release`);
//! two shapes may share one. Real-property subclasses are a C++ affair,
//! out of reach here (Jolt's own RTTI macros, not the language's, since zjolt compiles `-fno-rtti`).

const std = @import("std");
const c = @import("c/material.zig");
const err = @import("error.zig");

/// An 8-bit-per-channel colour. Debug drawing only; nothing in the simulation
/// reads one.
pub const Color = c.Color;

pub fn color(r: u8, g: u8, b: u8) Color {
    return .{ .r = r, .g = g, .b = b, .a = 255 };
}

/// @see `Color.toVec4`.
pub const Vec4 = c.Color.Vec4;

pub const PhysicsMaterial = struct {
    /// Const for the same reason a `Shape`'s handle is: every entry point that
    /// takes a material takes it as `const ZJoltPhysicsMaterial *`. Unlike a
    /// shape, there is no mutable exception — a material has nothing to
    /// mutate.
    handle: *const c.PhysicsMaterial,

    pub const Options = struct {
        /// Copied by the library. Debug only.
        debug_name: [:0]const u8 = "",
        /// Debug only. Null keeps Jolt's grey.
        debug_color: ?Color = null,
    };

    pub fn init(opts: Options) err.Error!PhysicsMaterial {
        var handle: *c.PhysicsMaterial = undefined;
        const requested = opts.debug_color;
        try err.check(c.zjoltPhysicsMaterialCreate(
            opts.debug_name.ptr,
            if (requested) |*value| value else null,
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// The material every shape built without one reports.
    ///
    /// Shared and owned by the library, so it is valid between `init` and
    /// `deinit` and null outside that window. A real material rather than
    /// a null: test "did this leaf have a material of its own?" as
    /// `!material.eql(PhysicsMaterial.default().?)`, not a null check.
    pub fn default() ?PhysicsMaterial {
        const handle = c.zjoltPhysicsMaterialDefault() orelse return null;
        return .{ .handle = handle };
    }

    pub fn addRef(self: PhysicsMaterial) void {
        c.zjoltPhysicsMaterialAddRef(self.handle);
    }

    /// Drops one reference. The material is destroyed when the last one goes.
    pub fn release(self: PhysicsMaterial) void {
        c.zjoltPhysicsMaterialRelease(self.handle);
    }

    pub fn refCount(self: PhysicsMaterial) u32 {
        return c.zjoltPhysicsMaterialGetRefCount(self.handle);
    }

    /// Identity, which is the only thing a material has. Two handles that
    /// compare equal are the same material.
    pub fn eql(self: PhysicsMaterial, other: PhysicsMaterial) bool {
        return self.handle == other.handle;
    }

    /// Borrowed from the material, so it does not outlive one. Jolt's own
    /// default answers "Default"; a material whose concrete type stores no
    /// name answers "Unknown", which is the base class talking rather than
    /// anything a caller set.
    pub fn debugName(self: PhysicsMaterial) [:0]const u8 {
        return std.mem.span(c.zjoltPhysicsMaterialGetDebugName(self.handle));
    }

    pub fn debugColor(self: PhysicsMaterial) Color {
        var out: Color = undefined;
        c.zjoltPhysicsMaterialGetDebugColor(self.handle, &out);
        return out;
    }
};

test "Color.toVec4 divides every channel by 255" {
    const v = (Color{ .r = 255, .g = 0, .b = 128, .a = 64 }).toVec4();
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), v.x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), v.y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 128.0 / 255.0), v.z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 64.0 / 255.0), v.w, 0.0001);
}
