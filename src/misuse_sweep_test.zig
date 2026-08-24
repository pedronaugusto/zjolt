//! Mechanical misuse sweep over the whole C boundary.
//!
//! Every entry point in `ffi/` owes two things that are pure convention and
//! easy to forget on the four hundredth one: it must refuse a call made before
//! `zjoltInit`, and it must refuse null pointers rather than dereferencing
//! them. Both are written by hand today — `ZJOLT_ENTER` at the top of a
//! result-returning function, an explicit `if (x == nullptr)` in a void one —
//! and neither is visible to the compiler.
//!
//! So this sweeps them by reflection instead of by hand. It discovers every
//! `pub extern fn` in `c.zig`, builds an argument list of nulls and zeroes,
//! and calls it — once with the library down, once with it up. There is **no
//! list of what to check**: a new entry point is covered the moment it is
//! declared, which is the only way this stays true at the size the surface is
//! heading for.
//!
//! ## Reading a failure
//!
//! A *returned* wrong answer is reported normally, naming the entry point.
//!
//! A *crash* is the other outcome, and it is the one worth explaining: this
//! test deliberately passes null where Zig's type system would not allow it,
//! because a C caller can and the C side is what must check. If an entry point
//! dereferences it, the process dies here rather than in a user's program. The
//! stack trace names the function; that is the entry point missing its guard.
//!
//! ## What it does not prove
//!
//! That the arguments are *rejected for the right reason* — only that they are
//! rejected. A behavioural test is still what says a hinge constrains to its
//! axis. This says nothing about behaviour and everything about the boundary
//! being a boundary.

const std = @import("std");
const c = @import("c.zig");
const core = c.core;

//=============================================================================
// What the sweep must not call
//
// Every exclusion is a lifecycle function: calling it would change the state
// the sweep is measuring. Nothing is excluded for being inconvenient, and each
// one says why, because a growing list here is how a sweep quietly stops
// sweeping.
//=============================================================================

const Excluded = struct {
    name: []const u8,
    why: []const u8,
};

const excluded = [_]Excluded{
    .{
        .name = "zjoltInitWithConfig",
        .why = "brings the library up. It is the one function that must work " ++
            "before init, so NOT_INITIALIZED is the wrong expectation for it, " ++
            "and calling it mid-sweep would invalidate every call after.",
    },
    .{
        .name = "zjoltDeinit",
        .why = "takes the library down, which is the state the second half of " ++
            "the sweep depends on.",
    },
};

fn isExcluded(comptime name: []const u8) bool {
    @setEvalBranchQuota(1_000_000);
    for (excluded) |e| {
        if (std.mem.eql(u8, e.name, name)) return true;
    }
    return false;
}

//=============================================================================
// Building an argument nothing should accept
//=============================================================================

/// The most hostile value of type `T` that a C caller could actually pass.
///
/// For a pointer that is null — including where Zig writes `*T` rather than
/// `?*T`. That is not a loophole in the wrapper: `*T` is what the *Zig* side
/// promises, and it is exactly why the C side has to check for itself. A C
/// host has no such type, and this test stands in for one.
///
/// Two details make that possible rather than merely desirable. Runtime safety
/// is off, because `@ptrFromInt` to a non-optional pointer is a checked cast
/// and the check would fire here on purpose rather than on a bug. And the zero
/// is laundered through a mutable variable, because a *comptime-known* zero is
/// a hard compile error — Zig will not let the type system be lied to at
/// comptime, only at runtime, which is exactly where a C caller lives.
fn hostile(comptime T: type) T {
    @setRuntimeSafety(false);
    var zero: usize = 0;
    _ = &zero;
    return switch (@typeInfo(T)) {
        .pointer => @ptrFromInt(zero),
        .optional => null,
        .@"enum" => @enumFromInt(zero),
        else => std.mem.zeroes(T),
    };
}

fn hostileArgs(comptime Fn: type) std.meta.ArgsTuple(Fn) {
    var args: std.meta.ArgsTuple(Fn) = undefined;
    inline for (@typeInfo(@TypeOf(args)).@"struct".fields) |f| {
        @field(args, f.name) = hostile(f.type);
    }
    return args;
}

/// Whether an entry point takes a pointer at all. One that does not has no
/// null to refuse, so the second sweep has nothing to ask of it.
fn takesPointer(comptime Fn: type) bool {
    inline for (@typeInfo(Fn).@"fn".params) |p| {
        const P = p.type orelse continue;
        const info = @typeInfo(P);
        if (info == .pointer) return true;
        if (info == .optional and @typeInfo(info.optional.child) == .pointer) return true;
    }
    return false;
}

const Counts = struct {
    refused_before_init: usize = 0,
    survived_nulls: usize = 0,
};

//=============================================================================
// The sweeps
//=============================================================================

/// Every result-returning entry point must refuse a call made before init.
///
/// This is what `ZJOLT_ENTER` promises, and until now nothing checked that it
/// was actually written at the top of each one. An entry point that forgets it
/// reaches Jolt with no factory registered and no allocator installed.
fn sweepBeforeInit() !usize {
    @setEvalBranchQuota(1_000_000);
    var refused: usize = 0;

    inline for (c.modules, 0..) |m, mi| {
        inline for (@typeInfo(m).@"struct".decls) |d| {
            // Skip what an earlier module re-exported: calling the same extern
            // once per module that re-exports it would inflate the count this
            // test's floor is measured against.
            comptime var earlier = false;
            inline for (c.modules, 0..) |other, oi| {
                if (oi < mi and @hasDecl(other, d.name)) earlier = true;
            }
            if (earlier) continue;
            const Decl = @TypeOf(@field(m, d.name));
            if (@typeInfo(Decl) != .@"fn") continue;
            if (@typeInfo(Decl).@"fn".calling_convention == .auto) continue;
            if (@typeInfo(Decl).@"fn".return_type != core.Result) continue;
            if (comptime isExcluded(d.name)) continue;

            const got = @call(.auto, @field(m, d.name), hostileArgs(Decl));
            if (got != .not_initialized) {
                std.debug.print(
                    "\n{s} returned .{s} before zjoltInit; every result-returning " ++
                        "entry point must open with ZJOLT_ENTER and return " ++
                        ".not_initialized\n",
                    .{ d.name, @tagName(got) },
                );
                return error.EntryPointMissingInitGuard;
            }
            refused += 1;
        }
    }

    return refused;
}

/// Every entry point that takes a pointer must survive being given null.
///
/// Result-returning ones must additionally *say* so rather than reporting
/// success. Void-returning ones can only be observed not to crash, which is
/// still the thing worth knowing: the guard in a void entry point is an
/// `if (x == nullptr) return;` a human wrote, and this is what notices when
/// the four hundredth one is missing.
fn sweepNulls() !usize {
    @setEvalBranchQuota(1_000_000);
    var survived: usize = 0;

    inline for (c.modules, 0..) |m, mi| {
        inline for (@typeInfo(m).@"struct".decls) |d| {
            // Skip what an earlier module re-exported: calling the same extern
            // once per module that re-exports it would inflate the count this
            // test's floor is measured against.
            comptime var earlier = false;
            inline for (c.modules, 0..) |other, oi| {
                if (oi < mi and @hasDecl(other, d.name)) earlier = true;
            }
            if (earlier) continue;
            const Decl = @TypeOf(@field(m, d.name));
            if (@typeInfo(Decl) != .@"fn") continue;
            if (@typeInfo(Decl).@"fn".calling_convention == .auto) continue;
            if (comptime isExcluded(d.name)) continue;
            if (comptime !takesPointer(Decl)) continue;

            const result = @call(.auto, @field(m, d.name), hostileArgs(Decl));

            if (@TypeOf(result) == core.Result and result == .ok) {
                std.debug.print(
                    "\n{s} returned .ok when every pointer it was given was null\n",
                    .{d.name},
                );
                return error.EntryPointAcceptedNull;
            }
            survived += 1;
        }
    }

    return survived;
}

//=============================================================================
// The tests
//=============================================================================

test "every entry point refuses a call made before init" {
    @setEvalBranchQuota(1_000_000);

    // Whatever ran before this must not have left the library up. Deinit is a
    // no-op when it is already down, and refuses while handles are alive —
    // in which case the premise does not hold and there is nothing to measure.
    core.zjoltDeinit();
    if (core.zjoltIsInitialized()) return error.SkipZigTest;

    const refused = try sweepBeforeInit();

    // A sweep that matched nothing would pass silently, which is the failure
    // mode this floor exists to make impossible. It is a floor, not the count:
    // it should never need lowering, and raising it is optional. Most of the
    // ABI does not return a result — it returns the value, and refuses by
    // clearing an out-parameter — so this floor is well under the total.
    try std.testing.expect(refused >= 25);
}

test "every entry point survives null pointers" {
    @setEvalBranchQuota(1_000_000);

    var gpa = std.heap.DebugAllocator(.{}){};
    defer std.debug.assert(gpa.deinit() == .ok);
    const allocator = gpa.allocator();

    var bridged = @import("memory.zig").bridge(allocator);
    var desc: core.InitDesc = .{ .allocator = &bridged };
    try std.testing.expectEqual(core.Result.ok, core.zjoltInitWithConfig(&desc, core.config_id));
    defer core.zjoltDeinit();

    const survived = try sweepNulls();
    try std.testing.expect(survived >= 100);
}
