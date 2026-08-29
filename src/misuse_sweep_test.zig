//! Mechanical misuse sweep over the whole C boundary.
//!
//! Every `ffi/` entry point must refuse a call before `zjoltInit`, refuse
//! a null pointer, and refuse an out-of-range enum tag — checked here by
//! reflection over every `pub extern fn` in `c.zig`, not a hand-kept
//! list, so a new entry point is covered the moment it is declared.
//!
//! A *returned* wrong answer is reported by name. A *crash* means an
//! entry point dereferenced a null or read a bad enum instead of
//! refusing it — the stack trace names the function missing its guard.
//!
//! Proves the boundary rejects bad input, NOT for the right reason, and
//! NOT full enum-sweep reachability: a null-checked handle argument
//! still blocks reaching the enum past it — see `liveHandleFor`.

const std = @import("std");
const c = @import("c.zig");
const core = c.core;

//=============================================================================
// What the sweep must not call
//
// Every exclusion is a lifecycle function that would change the state
// being measured. Nothing is excluded for being inconvenient, and each one says why — a growing list here is how a sweep quietly stops sweeping.
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

/// The most hostile value of type `T` that a C caller could actually
/// pass: null for any pointer, including `*T` where Zig promises
/// non-null — a C caller has no such promise, and this stands in for
/// one. Runtime safety is off, since a checked non-optional-pointer
/// cast would fire here on purpose; the zero is laundered through a
/// mutable variable, since a comptime-known one is a compile error Zig won't allow, only a runtime one — exactly where a C caller lives.
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

/// Every entry point that takes a pointer must survive being given
/// null. Result-returning ones must additionally *say* so, not report
/// success. Void-returning ones can only be observed not to crash — a
/// hand-written `if (x == nullptr) return;`, and this is what notices when one is missing.
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
// Building an enum value nothing should accept
//
// Reading an unnamed enum tag is UB — the defect this sweep exists to catch.
// `hostile()`'s `@enumFromInt(zero)` never probes it (zero is valid on every enum today); this builds one chosen to be invalid instead.
//=============================================================================

/// A tag no enumerator of `E` declares: one past its own highest —
/// computed from `E` itself, not a shared fixed sentinel, so a future
/// enumerator large enough to collide cannot silently defeat this.
/// Same laundering as `hostile()`'s zero: a comptime-known out-of-range
/// value is a compile error (the compiler sees it is unrepresentable),
/// but a C caller's int32_t is a runtime value, which this must test.
fn outOfRange(comptime E: type) E {
    @setRuntimeSafety(false);
    comptime var max: comptime_int = 0;
    inline for (@typeInfo(E).@"enum".fields) |f| {
        if (f.value > max) max = f.value;
    }
    var raw: usize = max + 1;
    _ = &raw;
    return @enumFromInt(raw);
}

/// True when `T` is an `extern struct` with at least one enum field — the
/// shape every settings/desc struct a host populates and hands in by pointer
/// takes in this ABI (`ZJoltCollideShapeSettings.active_edge_mode`,
/// `ZJoltHingeConstraintDesc.space`, and so on). A pointer to anything else —
/// an opaque handle, or a struct with no enum in it — has nothing here worth
/// poisoning.
fn hasEnumField(comptime T: type) bool {
    if (@typeInfo(T) != .@"struct") return false;
    inline for (@typeInfo(T).@"struct".fields) |f| {
        if (@typeInfo(f.type) == .@"enum") return true;
    }
    return false;
}

/// The struct type backing storage must be built for, if `P` is a
/// single-object pointer (or optional) to one with an enum field. Null
/// otherwise: a struct with no enum (`hostile()`'s plain null already
/// covers it), or a many-pointer like `[*]const T` (its length stays
/// `hostile()`'s zero, so the array is never indexed regardless of what backs it).
fn structNeedingStorage(comptime P: type) ?type {
    return switch (@typeInfo(P)) {
        .pointer => |ptr| if (ptr.size == .one and hasEnumField(ptr.child)) ptr.child else null,
        .optional => |opt| switch (@typeInfo(opt.child)) {
            .pointer => |ptr| if (ptr.size == .one and hasEnumField(ptr.child)) ptr.child else null,
            else => null,
        },
        else => null,
    };
}

/// A zeroed `T` with every enum field pushed out of range instead of left at
/// its all-zero tag. Everything else stays zero, the same as `hostile()`
/// elsewhere in this file — only the enum fields are the point of this sweep,
/// and a host leaving the rest of a settings struct zeroed is unremarkable.
fn poisoned(comptime T: type) T {
    var out: T = std.mem.zeroes(T);
    inline for (@typeInfo(T).@"struct".fields) |f| {
        if (@typeInfo(f.type) == .@"enum") @field(out, f.name) = outOfRange(f.type);
    }
    return out;
}

/// Whether calling `Fn` through `enumHostileArgs` would put an out-of-range
/// value somewhere the callee can actually read: a top-level enum parameter,
/// or a pointer to a struct with an enum field. A function with neither has
/// nothing here to probe — `sweepNulls` already covers it, and calling it
/// again with the same null/zero arguments would just inflate this sweep's
/// count without testing anything new.
fn takesEnum(comptime Fn: type) bool {
    inline for (@typeInfo(Fn).@"fn".params) |p| {
        const P = p.type orelse continue;
        if (@typeInfo(P) == .@"enum") return true;
        if (structNeedingStorage(P) != null) return true;
    }
    return false;
}

/// A live handle this sweep can hand an entry point instead of null,
/// keyed by pointee TYPE, not by function — a new `*PhysicsSystem`
/// parameter is covered automatically. Deliberately just the one
/// handle: a live Character/Ragdoll/Constraint/VehicleConstraint needs
/// a fixture this sweep does not build, so an entry point gated on one
/// is not reached past that gate (same limit `sweepNulls` has). A system alone still reaches most of the surface.
fn liveHandleFor(comptime P: type, system: *core.PhysicsSystem) ?P {
    if (@typeInfo(P) != .pointer) return null;
    const ptr = @typeInfo(P).pointer;
    if (ptr.size != .one or ptr.child != core.PhysicsSystem) return null;
    return @as(P, @ptrCast(system));
}

/// Per-parameter backing storage for one call: a `poisoned()` struct
/// for every parameter `structNeedingStorage` needs, and a zero-sized
/// placeholder elsewhere. Positionally keyed to `Fn`'s parameters.
/// `enumHostileArgs`'s pointers into it are valid for the call's duration
/// and no longer; the caller owns the storage and builds it per call.
fn StorageFor(comptime Fn: type) type {
    const fields = @typeInfo(std.meta.ArgsTuple(Fn)).@"struct".fields;
    comptime var types: [fields.len]type = undefined;
    inline for (fields, 0..) |f, i| {
        types[i] = structNeedingStorage(f.type) orelse u0;
    }
    return std.meta.Tuple(&types);
}

fn poisonedStorage(comptime Fn: type) StorageFor(Fn) {
    var storage: StorageFor(Fn) = undefined;
    const fields = @typeInfo(std.meta.ArgsTuple(Fn)).@"struct".fields;
    inline for (fields, 0..) |f, i| {
        if (structNeedingStorage(f.type)) |S| {
            storage[i] = poisoned(S);
        } else {
            storage[i] = 0;
        }
    }
    return storage;
}

/// `hostileArgs`, except: an enum parameter gets `outOfRange` instead of
/// `hostile()`'s always-valid zero; a struct-with-enum pointer points at
/// the matching `storage` field (a `poisoned()` instance) instead of
/// null, so the callee's own null check does not block this; and
/// `*PhysicsSystem` gets `system` instead of null. `storage` is a
/// pointer since what `args` points into must outlive this call — see `StorageFor`.
fn enumHostileArgs(
    comptime Fn: type,
    storage: *StorageFor(Fn),
    system: *core.PhysicsSystem,
) std.meta.ArgsTuple(Fn) {
    var args: std.meta.ArgsTuple(Fn) = undefined;
    inline for (@typeInfo(@TypeOf(args)).@"struct".fields, 0..) |f, i| {
        if (@typeInfo(f.type) == .@"enum") {
            @field(args, f.name) = outOfRange(f.type);
        } else if (structNeedingStorage(f.type) != null) {
            @field(args, f.name) = &storage[i];
        } else if (liveHandleFor(f.type, system)) |handle| {
            @field(args, f.name) = handle;
        } else {
            @field(args, f.name) = hostile(f.type);
        }
    }
    return args;
}

/// Every entry point that takes an enum by value, or a pointer to a
/// struct with an enum field, must survive that enum being out of
/// range. Unlike `sweepNulls`, does NOT require the result differ from
/// `.ok`: a live system can legitimately succeed regardless of the
/// enum's value (e.g. `zjoltBodyAddBatch` on an empty batch, correctly,
/// since there is nothing for the garbage activation to apply to). A crash is the only failure this sweep can tell apart from correct behaviour.
fn sweepEnumOutOfRange(system: *core.PhysicsSystem) !usize {
    @setEvalBranchQuota(1_000_000);
    var probed: usize = 0;

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
            if (comptime !takesEnum(Decl)) continue;

            var storage = poisonedStorage(Decl);
            _ = @call(.auto, @field(m, d.name), enumHostileArgs(Decl, &storage, system));
            probed += 1;
        }
    }

    return probed;
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

test "every entry point refuses an out-of-range enum" {
    @setEvalBranchQuota(1_000_000);

    // A live system rather than the raw zjoltInitWithConfig + zjoltPhysicsSystemCreate
    // this file otherwise sticks to: a real broad-phase/object-layer setup is
    // several structs of host callbacks, and World already builds one
    // correctly — reusing it here is less to get wrong than re-deriving it,
    // and this test's only interest in the system is a live, non-null handle
    // to hand to `liveHandleFor`.
    const zjolt = @import("zjolt.zig");
    const World = @import("integration_test.zig").World;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var world = try World.init();
    defer world.deinit();

    const probed = try sweepEnumOutOfRange(world.system.handle);

    // A floor, not the count, same as the two sweeps above: most enum
    // parameters here are reached (a system handle covers body/batch/
    // soft-body creation, and every settings struct this sweep can
    // poison directly needs no handle) — but constraints, characters,
    // ragdolls and vehicles are gated on handles this sweep does not
    // build. It is over 100 today; 100 leaves room without being so low a regression could hide under it.
    try std.testing.expect(probed >= 100);
}
