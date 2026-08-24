//! Physics materials.
//!
//! A material is an **identity**, not a property bag. It carries a debug name
//! and a debug colour and nothing else — there is no friction, no restitution
//! and no user data on one. Friction and restitution live on the body
//! (`BodyInterface.setFriction`, `setRestitution`), and a contact combines the
//! two bodies' values, never the two surfaces'.
//!
//! What a material is for is telling one surface apart from another *within*
//! one shape. A mesh is built with a material per triangle and a height field
//! with one per quad; a hit carries a sub-shape id, and `Shape.material` turns
//! that id into the material of the exact triangle that was hit. That is how
//! one terrain mesh is gravel here and metal there. Map the returned material
//! to your own surface data by identity — `eql`, or a hash of `.handle` — and
//! keep the properties on your side.
//!
//! A shape built with a material holds a reference on it, so the usual pattern
//! is create, build the shapes, `release`. Two shapes may share one.
//!
//! Subclasses that carry real properties are a C++ affair and out of reach
//! from here: Jolt finds a material's type through its own RTTI macros plus a
//! `Factory::Register` call — its hand-rolled type system, not the language's,
//! since zjolt compiles `-fno-rtti`.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");

/// An 8-bit-per-channel colour. Debug drawing only; nothing in the simulation
/// reads one.
pub const Color = c.Color;

pub fn color(r: u8, g: u8, b: u8) Color {
    return .{ .r = r, .g = g, .b = b, .a = 255 };
}

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
    /// `deinit` and null outside that window. It is a real material rather
    /// than a null, which is why "did this leaf have a material of its own?"
    /// is `!material.eql(PhysicsMaterial.default().?)` and not a null test.
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
