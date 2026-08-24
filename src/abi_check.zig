//! Comptime cross-check: the hand-written externs in `c.zig` against the real
//! C header.
//!
//! `c.zig` is written by hand so the wrapper gets exactly the types it wants
//! and the shipped module never runs translate-c. The cost of hand-writing is
//! drift, and nothing in either compiler notices when this file's twin stops
//! matching `ffi/zjolt.h`.
//!
//! This closes that by `@cImport`-ing the header — in a test only, so the
//! shipped module stays translate-c-free — and comparing the two namespaces
//! declaration by declaration. There is **no hand-written list of what to
//! check**: every public declaration in `c.zig` is discovered by reflection,
//! paired with its counterpart by naming convention, and compared. A
//! declaration that fits no category is a compile error rather than a silent
//! pass, so the check cannot quietly stop covering something.
//!
//! The naming conventions are therefore load-bearing, not cosmetic:
//!
//!   * a type `Foo`            pairs with `ZJoltFoo`
//!   * a function `zjoltFoo`   pairs with itself
//!   * a constant `foo_bar`    pairs with `ZJOLT_FOO_BAR`
//!   * an enum `Foo`'s field `bar` pairs with `ZJOLT_FOO_BAR`
//!
//! A declaration that breaks the convention fails this check, which is the
//! pressure that keeps the two sides legible as twins.
//!
//! ## What it does not catch
//!
//! translate-c renders every C pointer as `[*c]T`, while `c.zig` writes the
//! pointer it means (`*T`, `?*const T`, `[*]T`). Pointee types are therefore
//! compared only by size and alignment: a `float *` declared here as `*i32`
//! passes. `tests/c_smoke.c` drives the same scenarios as the Zig suite
//! through the header itself, which is what covers that residue.
//!
//! It also compares this build's externs against this build's *header*, not
//! against the *library*. Those two diverge only when the header is compiled
//! with different macros than the library was, which is why `build.zig` gives
//! the test module its macros from the same `applyBuildMacros` the library
//! uses, and why `zjoltInit` checks `ZJOLT_CONFIG_ID` at runtime as well.

const std = @import("std");
const c = @import("c.zig");

const h = @cImport({
    @cInclude("zjolt.h");
});

/// Functions the header defines inline. An inline definition emits no symbol,
/// so there is nothing for `c.zig` to declare and the reverse sweep must not
/// demand one. `zjoltInit` is inline on purpose: it exists to capture the
/// *caller's* `ZJOLT_CONFIG_ID` and pass it to `zjoltInitWithConfig`, which it
/// could not do from inside the library.
const header_inline_fns = [_][]const u8{"zjoltInit"};

//=============================================================================
// Name conventions, computed rather than tabulated
//=============================================================================

/// `MotionType` -> `MOTION_TYPE`, `body_id_invalid` -> `BODY_ID_INVALID`.
fn screaming(comptime name: []const u8) []const u8 {
    comptime {
        var out: []const u8 = "";
        var prev_lower = false;
        for (name) |ch| {
            if (std.ascii.isUpper(ch)) {
                if (prev_lower) out = out ++ "_";
                out = out ++ [_]u8{ch};
                prev_lower = false;
            } else if (ch == '_') {
                out = out ++ "_";
                prev_lower = false;
            } else {
                out = out ++ [_]u8{std.ascii.toUpper(ch)};
                prev_lower = true;
            }
        }
        return out;
    }
}

fn typeCName(comptime name: []const u8) []const u8 {
    return "ZJolt" ++ name;
}

fn constCName(comptime name: []const u8) []const u8 {
    return "ZJOLT_" ++ screaming(name);
}

fn fieldCName(comptime type_name: []const u8, comptime field_name: []const u8) []const u8 {
    return "ZJOLT_" ++ screaming(type_name) ++ "_" ++ screaming(field_name);
}

//=============================================================================
// Comparison primitives
//
// Every failure is a compile error naming both sides, because a build that
// cannot state which declaration drifted is a guard that costs more to read
// than the drift it found.
//=============================================================================

fn fail(comptime msg: []const u8) void {
    @compileError("zjolt ABI drift: " ++ msg);
}

fn theirDecl(comptime name: []const u8, comptime because: []const u8) type {
    if (!@hasDecl(h, name)) {
        fail("`" ++ because ++ "` in src/c.zig expects `" ++ name ++
            "` in ffi/zjolt.h, which does not declare it");
    }
    return @TypeOf(@field(h, name));
}

fn sameSizeAndAlign(
    comptime what: []const u8,
    comptime Ours: type,
    comptime Theirs: type,
) void {
    if (@sizeOf(Ours) != @sizeOf(Theirs)) {
        fail(what ++ " is " ++ std.fmt.comptimePrint("{d}", .{@sizeOf(Ours)}) ++
            " bytes in src/c.zig but " ++ std.fmt.comptimePrint("{d}", .{@sizeOf(Theirs)}) ++
            " in ffi/zjolt.h");
    }
    if (@alignOf(Ours) != @alignOf(Theirs)) {
        fail(what ++ " has alignment " ++ std.fmt.comptimePrint("{d}", .{@alignOf(Ours)}) ++
            " in src/c.zig but " ++ std.fmt.comptimePrint("{d}", .{@alignOf(Theirs)}) ++
            " in ffi/zjolt.h");
    }
}

/// Compares two function types by the only things translate-c preserves:
/// how many parameters there are and how each one is passed.
fn checkFnType(
    comptime what: []const u8,
    comptime Ours: type,
    comptime Theirs: type,
) void {
    const ours = @typeInfo(Ours).@"fn";
    const theirs = @typeInfo(Theirs).@"fn";

    if (ours.params.len != theirs.params.len) {
        fail(what ++ " takes " ++ std.fmt.comptimePrint("{d}", .{ours.params.len}) ++
            " parameters in src/c.zig but " ++ std.fmt.comptimePrint("{d}", .{theirs.params.len}) ++
            " in ffi/zjolt.h");
    }

    inline for (ours.params, theirs.params, 0..) |op, tp, i| {
        const OP = op.type orelse fail(what ++ " has an untyped parameter in src/c.zig");
        const TP = tp.type orelse fail(what ++ " has an untyped parameter in ffi/zjolt.h");
        sameSizeAndAlign(
            what ++ " parameter " ++ std.fmt.comptimePrint("{d}", .{i}),
            OP,
            TP,
        );
    }

    const OR = ours.return_type orelse fail(what ++ " has no return type in src/c.zig");
    const TR = theirs.return_type orelse fail(what ++ " has no return type in ffi/zjolt.h");
    sameSizeAndAlign(what ++ " return value", OR, TR);
}

/// Struct layout, compared field by NAME rather than by position.
///
/// This is the distinction that makes the check worth having. Two same-sized
/// adjacent fields swapping places leaves the *sequence* of offsets identical,
/// so a positional comparison — or a digest folded over offsets alone — passes
/// a swap that silently reinterprets both fields. Pairing each name with its
/// own offset is what catches it.
fn checkStructLayout(
    comptime what: []const u8,
    comptime Ours: type,
    comptime Theirs: type,
) void {
    sameSizeAndAlign(what, Ours, Theirs);

    const ours = @typeInfo(Ours).@"struct";
    const theirs = switch (@typeInfo(Theirs)) {
        .@"struct" => |s| s,
        else => fail(what ++ " is a struct in src/c.zig but not in ffi/zjolt.h"),
    };

    if (ours.fields.len != theirs.fields.len) {
        fail(what ++ " has " ++ std.fmt.comptimePrint("{d}", .{ours.fields.len}) ++
            " fields in src/c.zig but " ++ std.fmt.comptimePrint("{d}", .{theirs.fields.len}) ++
            " in ffi/zjolt.h");
    }

    inline for (ours.fields) |f| {
        if (!@hasField(Theirs, f.name)) {
            fail(what ++ " has field `" ++ f.name ++ "` in src/c.zig, which ffi/zjolt.h does not");
        }
        if (@offsetOf(Ours, f.name) != @offsetOf(Theirs, f.name)) {
            fail(what ++ "." ++ f.name ++ " is at byte " ++
                std.fmt.comptimePrint("{d}", .{@offsetOf(Ours, f.name)}) ++ " in src/c.zig but " ++
                std.fmt.comptimePrint("{d}", .{@offsetOf(Theirs, f.name)}) ++ " in ffi/zjolt.h");
        }
        sameSizeAndAlign(
            what ++ "." ++ f.name,
            f.type,
            @FieldType(Theirs, f.name),
        );
    }
}

/// Enumerator values, paired by the `ZJOLT_<TYPE>_<FIELD>` convention.
///
/// translate-c flattens a C enum to an integer alias and loses which
/// enumerators belonged to it, so the values cannot be recovered from the type.
/// The convention is what puts them back together — and it is why the header's
/// enumerators are named strictly, with no readable-but-irregular exceptions.
fn checkEnumValues(
    comptime what: []const u8,
    comptime Ours: type,
    comptime ours_name: []const u8,
) void {
    inline for (@typeInfo(Ours).@"enum".fields) |f| {
        const cname = fieldCName(ours_name, f.name);
        _ = theirDecl(cname, what ++ "." ++ f.name);
        if (@as(i128, @field(h, cname)) != @as(i128, f.value)) {
            fail(what ++ "." ++ f.name ++ " is " ++
                std.fmt.comptimePrint("{d}", .{f.value}) ++ " in src/c.zig but " ++ cname ++
                " is " ++ std.fmt.comptimePrint("{d}", .{@field(h, cname)}) ++ " in ffi/zjolt.h");
        }
    }
}

/// Bit-mask values for a packed struct whose C counterpart is an enum of bits.
///
/// The value checked is the mask with exactly this field set, which is the
/// same number the C enumerator names. Padding fields (leading underscore)
/// have no counterpart and are skipped.
fn checkMaskBits(
    comptime what: []const u8,
    comptime Ours: type,
    comptime ours_name: []const u8,
) void {
    const Backing = @typeInfo(Ours).@"struct".backing_integer.?;
    inline for (@typeInfo(Ours).@"struct".fields) |f| {
        if (comptime std.mem.startsWith(u8, f.name, "_")) continue;
        if (f.type != bool) {
            fail(what ++ "." ++ f.name ++ " is not a bool, so its bit value cannot be " ++
                "derived; give this mask an explicit check");
        }
        const cname = fieldCName(ours_name, f.name);
        _ = theirDecl(cname, what ++ "." ++ f.name);

        var one: Ours = std.mem.zeroes(Ours);
        @field(one, f.name) = true;
        const bit: Backing = @bitCast(one);

        if (@as(i128, @field(h, cname)) != @as(i128, bit)) {
            fail(what ++ "." ++ f.name ++ " is bit " ++
                std.fmt.comptimePrint("0x{x}", .{bit}) ++ " in src/c.zig but " ++ cname ++
                " is " ++ std.fmt.comptimePrint("0x{x}", .{@field(h, cname)}) ++ " in ffi/zjolt.h");
        }
    }
}

//=============================================================================
// The sweep
//=============================================================================

const Counts = struct {
    types: usize = 0,
    functions: usize = 0,
    constants: usize = 0,
    fields: usize = 0,
    enumerators: usize = 0,
};

/// Every public declaration in `c.zig`, classified and compared. The `else`
/// arms are compile errors: a declaration this does not know how to check is a
/// hole in the guard, and a hole should stop the build rather than be counted
/// as a pass.
fn sweepOurs() Counts {
    comptime {
        var n = Counts{};

        for (@typeInfo(c).@"struct".decls) |d| {
            const Decl = @TypeOf(@field(c, d.name));

            // ---- types -----------------------------------------------------
            if (Decl == type) {
                const Ours = @field(c, d.name);
                const cname = typeCName(d.name);
                const what = "type " ++ d.name;
                n.types += 1;

                switch (@typeInfo(Ours)) {
                    .@"opaque" => {
                        // Nothing to compare but existence: an opaque handle
                        // has no layout on either side, which is the point.
                        const Theirs = theirDecl(cname, what);
                        if (Theirs != type) fail(cname ++ " is not a type in ffi/zjolt.h");
                        if (@typeInfo(@field(h, cname)) != .@"opaque") {
                            fail(what ++ " is opaque in src/c.zig but not in ffi/zjolt.h");
                        }
                    },
                    .@"struct" => |s| {
                        _ = theirDecl(cname, what);
                        const Theirs = @field(h, cname);
                        switch (s.layout) {
                            .@"extern" => {
                                checkStructLayout(what, Ours, Theirs);
                                n.fields += s.fields.len;
                            },
                            .@"packed" => {
                                // A bit mask. C spells it as an enum, so the
                                // header side is an integer alias; compare the
                                // backing width, then every bit by name.
                                sameSizeAndAlign(what, Ours, Theirs);
                                checkMaskBits(what, Ours, d.name);
                                n.enumerators += s.fields.len - 1; // minus padding
                            },
                            .auto => fail(what ++ " has automatic layout, so it has no defined " ++
                                "ABI; declare it extern or packed"),
                        }
                    },
                    .@"enum" => |e| {
                        _ = theirDecl(cname, what);
                        sameSizeAndAlign(what, Ours, @field(h, cname));
                        checkEnumValues(what, Ours, d.name);
                        n.enumerators += e.fields.len;
                    },
                    .int, .float, .bool => {
                        _ = theirDecl(cname, what);
                        const Theirs = @field(h, cname);
                        sameSizeAndAlign(what, Ours, Theirs);
                        const oi = @typeInfo(Ours);
                        const ti = @typeInfo(Theirs);
                        if (oi == .int and ti == .int and oi.int.signedness != ti.int.signedness) {
                            fail(what ++ " is " ++ @typeName(Ours) ++ " in src/c.zig but " ++
                                @typeName(Theirs) ++ " in ffi/zjolt.h");
                        }
                    },
                    .pointer => {
                        // A callback typedef. translate-c makes every C
                        // function pointer optional; unwrap before comparing.
                        _ = theirDecl(cname, what);
                        const Theirs = @field(h, cname);
                        sameSizeAndAlign(what, Ours, Theirs);
                        const OursFn = @typeInfo(Ours).pointer.child;
                        const TheirsOpt = @typeInfo(Theirs);
                        const TheirsPtr = if (TheirsOpt == .optional) TheirsOpt.optional.child else Theirs;
                        checkFnType(what, OursFn, @typeInfo(TheirsPtr).pointer.child);
                    },
                    else => fail("type " ++ d.name ++ " is a " ++
                        @tagName(@typeInfo(Ours)) ++ ", which this check does not know how to " ++
                        "compare against the header"),
                }
                continue;
            }

            // ---- functions -------------------------------------------------
            if (@typeInfo(Decl) == .@"fn") {
                if (@typeInfo(Decl).@"fn".calling_convention == .auto) {
                    // A Zig helper, not part of the C ABI. c.zig should not
                    // really have these, but one is not drift.
                    continue;
                }
                const what = "function " ++ d.name;
                _ = theirDecl(d.name, what);
                checkFnType(what, Decl, @TypeOf(@field(h, d.name)));
                n.functions += 1;
                continue;
            }

            // ---- constants -------------------------------------------------
            if (@typeInfo(Decl) == .int or @typeInfo(Decl) == .comptime_int or
                @typeInfo(Decl) == .@"enum")
            {
                const cname = constCName(d.name);
                const what = "constant " ++ d.name;
                _ = theirDecl(cname, what);
                const ours_val: i128 = @intCast(if (@typeInfo(Decl) == .@"enum")
                    @intFromEnum(@field(c, d.name))
                else
                    @field(c, d.name));
                const theirs_val: i128 = @intCast(@field(h, cname));
                if (ours_val != theirs_val) {
                    fail(what ++ " is " ++ std.fmt.comptimePrint("{d}", .{ours_val}) ++
                        " in src/c.zig but " ++ cname ++ " is " ++
                        std.fmt.comptimePrint("{d}", .{theirs_val}) ++ " in ffi/zjolt.h");
                }
                n.constants += 1;
                continue;
            }

            fail("src/c.zig declares `" ++ d.name ++ "` as a " ++ @tagName(@typeInfo(Decl)) ++
                ", which this check does not know how to compare. Add a case rather than " ++
                "leaving it unchecked.");
        }

        return n;
    }
}

/// The other direction: a function the header exports that `c.zig` never
/// declared is invisible to the sweep above, because the sweep only walks what
/// `c.zig` has.
fn sweepTheirs() usize {
    comptime {
        var missing: usize = 0;
        var found: usize = 0;

        for (@typeInfo(h).@"struct".decls) |d| {
            // Filter by name BEFORE touching the value: translate-c emits
            // `@compileError` declarations for system macros it cannot render,
            // and evaluating one of those would fail the build for a reason
            // that has nothing to do with zjolt.
            if (!std.mem.startsWith(u8, d.name, "zjolt")) continue;
            if (@typeInfo(@TypeOf(@field(h, d.name))) != .@"fn") continue;

            var inline_in_header = false;
            for (header_inline_fns) |name| {
                if (std.mem.eql(u8, name, d.name)) inline_in_header = true;
            }
            if (inline_in_header) continue;

            found += 1;
            if (!@hasDecl(c, d.name)) {
                missing += 1;
                fail("ffi/zjolt.h exports `" ++ d.name ++ "` but src/c.zig never declares it");
            }
        }
        return found;
    }
}

//=============================================================================
// The test
//
// The comparisons above are compile errors, so reaching this body at all means
// they passed. What is left to assert is that they actually ran: a sweep that
// silently matched nothing would be indistinguishable from a sweep that
// matched everything.
//=============================================================================

test "ABI: src/c.zig agrees with ffi/zjolt.h" {
    @setEvalBranchQuota(1_000_000);

    const ours = comptime sweepOurs();
    const theirs = comptime sweepTheirs();

    try std.testing.expect(ours.types >= 40);
    try std.testing.expect(ours.functions >= 130);
    try std.testing.expect(ours.constants >= 10);
    try std.testing.expect(ours.fields >= 150);
    try std.testing.expect(ours.enumerators >= 40);
    try std.testing.expectEqual(ours.functions, theirs);
}
