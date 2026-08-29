//! Comptime cross-check: the hand-written externs in `c.zig` against the
//! real C header — `@cImport`-ed in a test only (the shipped module stays
//! translate-c-free), compared declaration by declaration by reflection.
//! No hand-written list of what to check: an unclassified declaration is
//! a compile error, not a silent pass.
//!
//! Naming conventions are load-bearing: type `Foo` pairs with `ZJoltFoo`,
//! function `zjoltFoo` with itself, constant `foo_bar` with
//! `ZJOLT_FOO_BAR`, enum `Foo`'s field `bar` with `ZJOLT_FOO_BAR`.
//!
//! Does NOT catch: pointee type mismatches (translate-c renders every C
//! pointer as `[*c]T`, so pointees compare by size/alignment only — see
//! `tests/c_smoke.c`), or drift from a header built with different
//! macros (both get `applyBuildMacros`; `zjoltInit` also checks `ZJOLT_CONFIG_ID` at runtime).

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
// Every failure is a compile error naming both sides — a guard that
// cannot state which declaration drifted costs more to read than the drift it found.
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

/// The scalar a type really is at the boundary, with the Zig-side
/// wrapper removed. translate-c renders every C enum as a plain
/// integer; this side deliberately keeps enums as `enum(c_int)` and
/// masks as `packed struct(u32)`, so each is resolved to its backing
/// integer first — not a loosening: an `enum(u32)` against a C `int`
/// enum is a real signedness disagreement, and this is what surfaces it.
fn scalarIdentity(comptime T: type) type {
    return switch (@typeInfo(T)) {
        .@"enum" => |e| e.tag_type,
        .@"struct" => |st| st.backing_integer orelse T,
        else => T,
    };
}

/// Size and alignment, plus the part of a scalar's identity they do
/// not carry: a `uint32_t` declared `i32`, or a `float` declared `u32`,
/// passes size/alignment and then silently reinterprets every value —
/// comparing signedness and int-vs-float closes that. Applied wherever
/// a type crosses (struct fields, parameters, returns), not just top-level typedefs, since that is not where the mistake gets made.
fn sameScalar(
    comptime what: []const u8,
    comptime Ours: type,
    comptime Theirs: type,
) void {
    sameSizeAndAlign(what, Ours, Theirs);

    const oi = @typeInfo(scalarIdentity(Ours));
    const ti = @typeInfo(scalarIdentity(Theirs));

    // Signedness, EXCEPT across an enum: C leaves an enum's underlying
    // type to the implementation (clang/gcc pick unsigned when no
    // enumerator is negative, MSVC uses `int`), so comparing it would
    // fail a correct binding on one toolchain and pass on another. Safe
    // to skip only because every enumerator here is non-negative —
    // `checkEnumValues` asserts that precondition rather than assuming it.
    const across_enum = scalarIdentity(Ours) != Ours or scalarIdentity(Theirs) != Theirs;
    if (!across_enum and oi == .int and ti == .int and
        oi.int.signedness != ti.int.signedness)
    {
        fail(what ++ " is " ++ @tagName(oi.int.signedness) ++ " in src/c.zig but " ++
            @tagName(ti.int.signedness) ++ " in ffi/zjolt.h");
    }
    if ((oi == .int) != (ti == .int) or (oi == .float) != (ti == .float)) {
        fail(what ++ " is a " ++ @tagName(oi) ++ " in src/c.zig but a " ++
            @tagName(ti) ++ " in ffi/zjolt.h");
    }
}

/// Compares two function types by the only things translate-c preserves:
/// how many parameters there are, how each one is passed, and whether the
/// signature is variadic.
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
    if (ours.is_var_args != theirs.is_var_args) {
        fail(what ++ " is variadic on one side of the boundary only");
    }

    inline for (ours.params, theirs.params, 0..) |op, tp, i| {
        const OP = op.type orelse fail(what ++ " has an untyped parameter in src/c.zig");
        const TP = tp.type orelse fail(what ++ " has an untyped parameter in ffi/zjolt.h");
        sameScalar(
            what ++ " parameter " ++ std.fmt.comptimePrint("{d}", .{i}),
            OP,
            TP,
        );
    }

    const OR = ours.return_type orelse fail(what ++ " has no return type in src/c.zig");
    const TR = theirs.return_type orelse fail(what ++ " has no return type in ffi/zjolt.h");
    sameScalar(what ++ " return value", OR, TR);
}

/// Struct layout, compared field by NAME rather than by position — the
/// distinction that makes the check worth having. Two same-sized
/// adjacent fields swapping places leaves the *sequence* of offsets
/// identical, so a positional comparison passes a swap that silently reinterprets both fields; pairing each name with its own offset catches it.
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
        sameScalar(
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
        // The precondition that lets sameScalar skip signedness across an
        // enum. C leaves the underlying type to the implementation, and the
        // implementations disagree; that is only unobservable while every
        // enumerator is non-negative. One negative value and the same
        // declaration means different things on MSVC and on clang.
        if (f.value < 0) {
            fail(what ++ "." ++ f.name ++ " is negative, which makes the enum's " ++
                "underlying type observable — C leaves that to the implementation " ++
                "and MSVC and clang choose differently. Use an explicit " ++
                "fixed-width field instead of an enum here.");
        }
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

        // One pass per module in `c.zig`'s list. A name in more than one
        // module is a re-export (every module re-exports the shared
        // primitives it takes, so callers see one namespace) and is
        // checked once, against the declaring module. The identity
        // assertion keeps that safe: a re-export that stops being the same declaration refuses the build rather than trusting the copy.
        for (c.modules, 0..) |m, mi| for (@typeInfo(m).@"struct".decls) |d| {
            // A name that an EARLIER module already declared is a re-export:
            // every module re-exports the shared primitives it takes so that
            // its own callers see one namespace. Check it once, where it is
            // declared. Comparing against earlier modules only — rather than
            // keeping a list of every name seen — keeps this linear in the
            // module count instead of quadratic in the declaration count.
            var earlier = false;
            for (c.modules, 0..) |other, oi| {
                if (oi < mi and @hasDecl(other, d.name)) {
                    // And if a re-export ever stops being the same
                    // declaration, refuse the build rather than checking one
                    // copy and trusting the other.
                    if (@TypeOf(@field(other, d.name)) == type and
                        @TypeOf(@field(m, d.name)) == type and
                        @field(other, d.name) != @field(m, d.name))
                    {
                        fail("`" ++ d.name ++ "` is declared in two of src/c.zig's " ++
                            "modules and they are not the same declaration. A module " ++
                            "re-exports a shared name; it must not redeclare one.");
                    }
                    earlier = true;
                }
            }
            if (earlier) continue;

            const Decl = @TypeOf(@field(m, d.name));

            // ---- types -----------------------------------------------------
            if (Decl == type) {
                const Ours = @field(m, d.name);
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
                    // A Zig helper, not an extern — right to skip (no
                    // header counterpart), but not SILENTLY: the reverse
                    // sweep only checks a name exists, so a helper on an
                    // exported symbol's name would satisfy it there while
                    // the extern it displaced vanishes unnoticed. A
                    // helper is allowed; one wearing a boundary name is not.
                    if (std.mem.startsWith(u8, d.name, "zjolt")) {
                        fail("src/c.zig declares `" ++ d.name ++ "` as a Zig function, not an " ++
                            "extern. The `zjolt` prefix is reserved for the C boundary here: a " ++
                            "helper on that name hides the extern it replaced from both " ++
                            "directions of this check. Rename the helper.");
                    }
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
                    @intFromEnum(@field(m, d.name))
                else
                    @field(m, d.name));
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
        };

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
            var home: ?type = null;
            for (c.modules) |m| {
                if (@hasDecl(m, d.name)) home = m;
            }
            if (home == null) {
                missing += 1;
                fail("ffi/zjolt.h exports `" ++ d.name ++ "` but no module in src/c.zig " ++
                    "declares it. A module missing from that list is a module neither " ++
                    "this check nor the misuse sweep covers.");
            }
            const Home = home.?;
            // Existence is not enough: the name must resolve to something
            // that actually links — a Zig helper or a type on the name
            // satisfies `@hasDecl` while the extern is gone. The forward
            // sweep rejects those first, so this is the backstop, on
            // purpose: it depends only on the header, so a future change
            // to the forward sweep's name filter cannot reopen the hole silently.
            const Ours = @TypeOf(@field(Home, d.name));
            if (@typeInfo(Ours) != .@"fn") {
                fail("ffi/zjolt.h exports `" ++ d.name ++ "` but src/c.zig declares that name " ++
                    "as a " ++ @tagName(@typeInfo(Ours)) ++ " rather than a function");
            }
            if (@typeInfo(Ours).@"fn".calling_convention == .auto) {
                fail("ffi/zjolt.h exports `" ++ d.name ++ "` but src/c.zig declares that name " ++
                    "as a Zig function rather than an extern, so nothing binds the symbol");
            }
        }
        return found;
    }
}

//=============================================================================
// The test
//
// The comparisons above are compile errors, so reaching this body means
// they passed. What's left: assert they actually ran — a sweep matching nothing silently would be indistinguishable from one matching everything.
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
