//! Forces Zig to analyse every wrapper, which it otherwise will not.
//!
//! Zig is lazy: an uncalled `pub fn` is never semantically analysed, so a
//! real type error inside it compiles clean — fine for a program, wrong
//! for a library, where code nobody here calls is code somebody else will.
//!
//! Matters more than it sounds: `zig build test` is this package's only
//! gate, and without this a wrapper with no behavioural test was not even
//! compile-checked (verified: a bogus field access compiled with zero
//! errors before this file existed).
//!
//! Taking a function's address forces analysis, so this walks every
//! public declaration by reflection — no hand-kept list, a wrapper added
//! tomorrow is covered automatically. Proves only that wrappers compile.

const std = @import("std");

/// Recursion depth. Types nest — `Shape.ConvexOptions`, `Character.Settings` —
/// and a bound is what keeps a self-referential type from looping forever.
const max_depth = 6;

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
            // A nested type: recurse, so methods on it are analysed too.
            // Skipping this would leave every `pub fn` on a struct uncovered,
            // which is where most of the wrapper actually lives.
            forceAnalysis(field, depth - 1);
        } else if (@typeInfo(FieldType) == .@"fn") {
            // The address is what forces the body. Calling it is not an
            // option — these open windows, allocate and step worlds.
            _ = &field;
        } else {
            // Binding it above already forced its evaluation; the address is
            // only here to consume it.
            _ = &field;
        }
    }
}

test "every wrapper compiles" {
    @setEvalBranchQuota(2_000_000);
    comptime forceAnalysis(@import("zjolt.zig"), max_depth);
}
