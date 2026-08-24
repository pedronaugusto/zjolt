//! Forces Zig to analyse every wrapper, which it otherwise will not.
//!
//! Zig is lazy: a `pub fn` nobody calls is never semantically analysed, so a
//! real type error inside it compiles clean. That is fine for a program and
//! wrong for a library, because the whole point of a library is that the code
//! nobody here calls is the code somebody else will.
//!
//! It matters more than it sounds. `zig build test` is this package's only
//! gate, and without this it was silently not a gate for any wrapper the test
//! suite did not happen to exercise — which, for a subsystem added without
//! behavioural tests, is all of it. Verified rather than assumed: a function
//! with a bogus field access added to `src/ragdoll.zig` compiled with zero
//! errors before this file existed.
//!
//! Taking a function's address forces its body to be analysed, so this walks
//! every public declaration by reflection and does that. There is **no list**:
//! a wrapper added tomorrow is covered the moment it is public.
//!
//! This is not a behavioural test and does not pretend to be. It proves the
//! wrappers compile, nothing more. What it replaces is the illusion that they
//! already did.

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
