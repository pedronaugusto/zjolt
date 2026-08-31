//! Bridges a Zig `std.mem.Allocator` onto Jolt's global (process-wide)
//! allocator seam, plus three more `Core` primitives with no other home: the
//! flush-denormals guard, the profiler's measurement aggregator and
//! external-hook bridge, and a `FixedSizeFreeList`-style batched free.
//!
//! Jolt frees with `free(block)`/`aligned_free(block)` — no size or alignment
//! — but Zig's allocator interface needs both. The gap closes by allocating
//! extra and stashing length and alignment in a header just before the pointer
//! handed to Jolt: `prefixSize(alignment)` bytes on EVERY Jolt allocation, 16
//! at the default alignment, and what makes a free possible at all.
//!
//! Jolt's plain `allocate` takes no alignment yet places SIMD types in the
//! memory it returns; that minimum is read from
//! `zjoltDefaultAllocateAlignment()` at install time, not assumed to be 16.

const std = @import("std");
const c = @import("c/core.zig");
const err = @import("error.zig");

/// Recorded ahead of every block so a free can reconstruct the slice Zig's
/// allocator needs.
const Header = struct {
    /// Total bytes handed out by the backing allocator, prefix included.
    total_len: usize,
    /// Alignment the backing allocation was made with.
    alignment: std.mem.Alignment,
};

/// Bytes reserved before the payload: enough for the header, rounded up so the
/// payload keeps the requested alignment.
fn prefixSize(alignment: std.mem.Alignment) usize {
    const minimum = @max(@sizeOf(Header), @alignOf(Header));
    return alignment.forward(minimum);
}

/// The alignment the backing allocation must use: at least the caller's, and
/// at least what the header itself needs.
fn backingAlignment(alignment: std.mem.Alignment) std.mem.Alignment {
    return @enumFromInt(@max(
        @intFromEnum(alignment),
        @intFromEnum(std.mem.Alignment.of(Header)),
    ));
}

fn headerOf(payload: [*]u8) *Header {
    return @ptrCast(@alignCast(payload - @sizeOf(Header)));
}

fn allocAligned(gpa: *const std.mem.Allocator, size: usize, alignment: usize) ?[*]u8 {
    // Jolt documents alignment as a power of two; refuse anything else rather
    // than computing a bogus prefix from it.
    if (alignment == 0 or !std.math.isPowerOfTwo(alignment)) return null;

    const backing = backingAlignment(std.mem.Alignment.fromByteUnits(alignment));
    const prefix = prefixSize(backing);
    const total = std.math.add(usize, prefix, size) catch return null;

    const base = gpa.rawAlloc(total, backing, @returnAddress()) orelse return null;
    const payload = base + prefix;
    headerOf(payload).* = .{ .total_len = total, .alignment = backing };
    return payload;
}

fn freeBlock(gpa: *const std.mem.Allocator, payload: [*]u8) void {
    const header = headerOf(payload).*;
    const prefix = prefixSize(header.alignment);
    const base = payload - prefix;
    gpa.rawFree(base[0..header.total_len], header.alignment, @returnAddress());
}

fn gpaFrom(user: ?*anyopaque) ?*const std.mem.Allocator {
    return @ptrCast(@alignCast(user orelse return null));
}

//=============================================================================
// The five hooks
//=============================================================================

fn allocate(user: ?*anyopaque, size: usize) callconv(.c) ?*anyopaque {
    const gpa = gpaFrom(user) orelse return null;
    return @ptrCast(allocAligned(gpa, size, default_alignment) orelse return null);
}

fn reallocate(
    user: ?*anyopaque,
    block: ?*anyopaque,
    old_size: usize,
    new_size: usize,
) callconv(.c) ?*anyopaque {
    const gpa = gpaFrom(user) orelse return null;
    const old_payload: [*]u8 = @ptrCast(block orelse {
        return @ptrCast(allocAligned(gpa, new_size, default_alignment) orelse return null);
    });

    // Reuse the alignment the block was actually made with rather than the
    // default, so this stays correct even for a block that came from
    // aligned_allocate.
    const old_header = headerOf(old_payload).*;
    const alignment = old_header.alignment.toByteUnits();
    const prefix = prefixSize(old_header.alignment);
    const old_base = old_payload - prefix;

    // Ask the backing allocator to resize in place before copying. Jolt
    // reallocates the arrays behind its body list, its contact cache and every
    // temporary collector, so the growth path is hot and every one of those
    // arrays doubles; a copy that a page-level allocator could have avoided is
    // the whole array moved for nothing. `rawRemap` preserves the bytes,
    // header included, whether or not it moves the block.
    if (std.math.add(usize, prefix, new_size)) |total| {
        if (gpa.rawRemap(
            old_base[0..old_header.total_len],
            old_header.alignment,
            total,
            @returnAddress(),
        )) |base| {
            const payload = base + prefix;
            headerOf(payload).* = .{ .total_len = total, .alignment = old_header.alignment };
            return @ptrCast(payload);
        }
    } else |_| return null;

    const fresh = allocAligned(gpa, new_size, alignment) orelse return null;
    const copy = @min(old_size, new_size);
    if (copy != 0) @memcpy(fresh[0..copy], old_payload[0..copy]);
    freeBlock(gpa, old_payload);
    return @ptrCast(fresh);
}

fn free(user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void {
    const gpa = gpaFrom(user) orelse return;
    freeBlock(gpa, @ptrCast(block orelse return));
}

fn alignedAllocate(user: ?*anyopaque, size: usize, alignment: usize) callconv(.c) ?*anyopaque {
    const gpa = gpaFrom(user) orelse return null;
    return @ptrCast(allocAligned(gpa, size, alignment) orelse return null);
}

fn alignedFree(user: ?*anyopaque, block: ?*anyopaque) callconv(.c) void {
    const gpa = gpaFrom(user) orelse return;
    freeBlock(gpa, @ptrCast(block orelse return));
}

//=============================================================================
// Installation
//=============================================================================

/// The allocator Jolt is currently pointed at, kept alive for as long as it is
/// installed. `user` in the C struct points at this.
var installed: std.mem.Allocator = undefined;

/// Read from the library rather than assumed, because it differs by target.
var default_alignment: usize = 16;

/// Builds the C allocator table for `gpa`. The returned struct borrows
/// `installed`, so it must not outlive the process.
pub fn bridge(gpa: std.mem.Allocator) c.Allocator {
    installed = gpa;
    default_alignment = c.zjoltDefaultAllocateAlignment();
    return .{
        .allocate = allocate,
        .reallocate = reallocate,
        .free = free,
        .aligned_allocate = alignedAllocate,
        .aligned_free = alignedFree,
        .user = @ptrCast(&installed),
    };
}

//=============================================================================
// Tests
//
// Drive the hooks directly rather than through a physics system, so the header arithmetic is covered on its own — including sizes and alignments Jolt would only reach under specific workloads.
//=============================================================================

test "the bridge round-trips every alignment Jolt may ask for" {
    const gpa = std.testing.allocator;
    _ = bridge(gpa);

    var alignment: usize = 1;
    while (alignment <= 64) : (alignment *= 2) {
        const block = alignedAllocate(@ptrCast(&installed), 100, alignment) orelse {
            return error.TestUnexpectedResult;
        };
        try std.testing.expect(@intFromPtr(block) % alignment == 0);
        // Write the payload so a too-small allocation trips the test allocator.
        const bytes: [*]u8 = @ptrCast(block);
        @memset(bytes[0..100], 0xAB);
        alignedFree(@ptrCast(&installed), block);
    }
}

test "plain allocations satisfy Jolt's unstated minimum alignment" {
    const gpa = std.testing.allocator;
    _ = bridge(gpa);

    var size: usize = 1;
    while (size <= 4096) : (size *= 2) {
        const block = allocate(@ptrCast(&installed), size) orelse {
            return error.TestUnexpectedResult;
        };
        try std.testing.expect(@intFromPtr(block) % default_alignment == 0);
        free(@ptrCast(&installed), block);
    }
}

test "reallocate preserves contents in both directions" {
    const gpa = std.testing.allocator;
    _ = bridge(gpa);

    const original = allocate(@ptrCast(&installed), 64) orelse {
        return error.TestUnexpectedResult;
    };
    const bytes: [*]u8 = @ptrCast(original);
    for (0..64) |i| bytes[i] = @truncate(i);

    // Grow.
    const grown = reallocate(@ptrCast(&installed), original, 64, 256) orelse {
        return error.TestUnexpectedResult;
    };
    const grown_bytes: [*]u8 = @ptrCast(grown);
    for (0..64) |i| try std.testing.expectEqual(@as(u8, @truncate(i)), grown_bytes[i]);

    // Shrink.
    const shrunk = reallocate(@ptrCast(&installed), grown, 256, 32) orelse {
        return error.TestUnexpectedResult;
    };
    const shrunk_bytes: [*]u8 = @ptrCast(shrunk);
    for (0..32) |i| try std.testing.expectEqual(@as(u8, @truncate(i)), shrunk_bytes[i]);

    free(@ptrCast(&installed), shrunk);
}

test "reallocate grows in place when the backing allocator can, rather than copying" {
    // A FixedBufferAllocator extends its LAST allocation and nothing else,
    // which makes the two paths tell themselves apart by address: a remap
    // returns the same pointer, and allocate-copy-free bumps the arena and
    // returns a different one. Jolt grows the arrays behind its body list and
    // its contact cache constantly, so the copy is the one worth not doing.
    var buffer: [4096]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buffer);
    _ = bridge(fba.allocator());

    const original = allocate(@ptrCast(&installed), 64) orelse {
        return error.TestUnexpectedResult;
    };
    const bytes: [*]u8 = @ptrCast(original);
    for (0..64) |i| bytes[i] = @truncate(i);

    const grown = reallocate(@ptrCast(&installed), original, 64, 256) orelse {
        return error.TestUnexpectedResult;
    };
    try std.testing.expectEqual(original, grown);
    const grown_bytes: [*]u8 = @ptrCast(grown);
    for (0..64) |i| try std.testing.expectEqual(@as(u8, @truncate(i)), grown_bytes[i]);

    // The header travelled with the block: a free reading a stale total_len
    // would hand the wrong slice back, which the shrink below would trip.
    const shrunk = reallocate(@ptrCast(&installed), grown, 256, 32) orelse {
        return error.TestUnexpectedResult;
    };
    free(@ptrCast(&installed), shrunk);
}

test "reallocate from null behaves as an allocation" {
    const gpa = std.testing.allocator;
    _ = bridge(gpa);

    const block = reallocate(@ptrCast(&installed), null, 0, 128) orelse {
        return error.TestUnexpectedResult;
    };
    const bytes: [*]u8 = @ptrCast(block);
    @memset(bytes[0..128], 0);
    free(@ptrCast(&installed), block);
}

test "the bridge tolerates a zero-size request and a null free" {
    const gpa = std.testing.allocator;
    _ = bridge(gpa);

    const block = allocate(@ptrCast(&installed), 0) orelse {
        return error.TestUnexpectedResult;
    };
    free(@ptrCast(&installed), block);
    free(@ptrCast(&installed), null);
    alignedFree(@ptrCast(&installed), null);
}

test "the bridge rejects a non-power-of-two alignment" {
    const gpa = std.testing.allocator;
    _ = bridge(gpa);

    try std.testing.expect(alignedAllocate(@ptrCast(&installed), 32, 3) == null);
    try std.testing.expect(alignedAllocate(@ptrCast(&installed), 32, 0) == null);
}

//=============================================================================
// Floating-point control word
//=============================================================================

/// Forces denormal results to flush to zero on the calling thread for the
/// guard's lifetime, restoring whatever state was there before — Jolt's own
/// determinism guidance, since a call into third-party code can otherwise
/// leave the rounding/flush mode changed underneath a later step. Restore
/// with `deinit`, via `defer`. A no-op on a target with no floating-point
/// control word (WASM, RISC-V, PPC, LoongArch).
pub const FlushDenormalsGuard = struct {
    state: c.FPControlWordState,

    pub fn init() FlushDenormalsGuard {
        var self: FlushDenormalsGuard = .{ .state = undefined };
        c.zjoltFPFlushDenormalsEnter(&self.state);
        return self;
    }

    pub fn deinit(self: *FlushDenormalsGuard) void {
        c.zjoltFPFlushDenormalsLeave(&self.state);
    }
};

test "FlushDenormalsGuard enters and leaves without error" {
    var guard = FlushDenormalsGuard.init();
    defer guard.deinit();
}

/// Through a volatile load and out of line, so no optimize mode can fold the
/// multiply at compile time and answer with a denormal the FPU never saw.
noinline fn halved(x: *const volatile f32) f32 {
    return x.* * 0.5;
}

test "FlushDenormalsGuard flushes a denormal result to zero, and stops when it leaves" {
    switch (@import("builtin").target.cpu.arch) {
        .x86, .x86_64, .aarch64, .aarch64_be, .arm, .armeb, .thumb, .thumbeb => {},
        // No floating-point control word to set; the guard is a no-op there
        // and there is nothing to assert.
        else => return error.SkipZigTest,
    }

    // The smallest normal f32; half of it is a denormal, which is exactly
    // what flush-to-zero replaces.
    const smallest_normal: f32 = std.math.floatMin(f32);
    try std.testing.expect(halved(&smallest_normal) > 0.0);

    var guard = FlushDenormalsGuard.init();
    const flushed = halved(&smallest_normal);
    guard.deinit();

    try std.testing.expectEqual(@as(f32, 0.0), flushed);
    try std.testing.expect(halved(&smallest_normal) > 0.0);
}

test "FlushDenormalsGuard nests LIFO" {
    var outer = FlushDenormalsGuard.init();
    {
        var inner = FlushDenormalsGuard.init();
        inner.deinit();
    }
    outer.deinit();
}

//=============================================================================
// Profiler measurement aggregation
//
// A pure-Zig port of `Profiler::Aggregator::AccumulateMeasurement` —
// private to `Profiler` upstream, so a caller has no other way to reach it.
// No build gate: this touches nothing Jolt's own profiler owns.
//=============================================================================

/// Running statistics for repeated measurements of one named scope, the
/// same shape Jolt's own profiler aggregates into before a dump.
pub const MeasurementAggregate = struct {
    call_count: u32 = 0,
    total_cycles: u64 = 0,
    min_cycles: u64 = std.math.maxInt(u64),
    max_cycles: u64 = 0,

    /// Folds one more measurement, in processor cycles, into the running
    /// count/total/min/max.
    pub fn accumulateMeasurement(self: *MeasurementAggregate, cycles_in_call_with_children: u64) void {
        self.call_count += 1;
        self.total_cycles += cycles_in_call_with_children;
        self.min_cycles = @min(self.min_cycles, cycles_in_call_with_children);
        self.max_cycles = @max(self.max_cycles, cycles_in_call_with_children);
    }
};

test "MeasurementAggregate tracks count, total, min and max" {
    var agg = MeasurementAggregate{};
    agg.accumulateMeasurement(10);
    agg.accumulateMeasurement(30);
    agg.accumulateMeasurement(20);

    try std.testing.expectEqual(@as(u32, 3), agg.call_count);
    try std.testing.expectEqual(@as(u64, 60), agg.total_cycles);
    try std.testing.expectEqual(@as(u64, 10), agg.min_cycles);
    try std.testing.expectEqual(@as(u64, 30), agg.max_cycles);
}

//=============================================================================
// External profiler bridge
//
// Gated behind `-Dexternal_profile`; `error.Unsupported` otherwise. Jolt's
// own `ExternalProfileMeasurement` calls `start` on construction and `end`
// on destruction of every profiled scope in its source.
//=============================================================================

pub const ExternalProfilerStartFn = c.ExternalProfilerStartFn;
pub const ExternalProfilerEndFn = c.ExternalProfilerEndFn;

/// Installs the hooks called at the start/end of every profiled scope from
/// then on, until `clearExternalProfilerHooks`. Both required.
pub fn setExternalProfilerHooks(
    start: ExternalProfilerStartFn,
    end: ExternalProfilerEndFn,
    user: ?*anyopaque,
) err.Error!void {
    try err.check(c.zjoltExternalProfilerSetHooks(start, end, user));
}

/// Uninstalls the hooks; a scope measured after this call is a no-op.
pub fn clearExternalProfilerHooks() void {
    c.zjoltExternalProfilerClearHooks();
}

//=============================================================================
// Batched free
//
// `FixedSizeFreeList<Object>::AddObjectToBatch`, generalised: any object
// pool that stores one `u32` "next free" slot per object (Jolt's own
// `ObjectStorage` shape) can link objects into a `Batch` here and return the
// whole run to its pool in one shot, without a Zig port of the pool itself.
//=============================================================================

/// Sentinel: no object, or the end of a chain. Matches
/// `FixedSizeFreeList::cInvalidObjectIndex`.
pub const invalid_object_index: u32 = 0xffffffff;

/// A run of object indices linked through the caller's own `next` slots,
/// built by repeated `addObjectToBatch` calls. Freeing the whole run is the
/// caller's own pool operation: walk from `first`, following `next`, for
/// `count` objects.
pub const Batch = struct {
    first: u32 = invalid_object_index,
    last: u32 = invalid_object_index,
    count: u32 = 0,
};

/// Appends `object_index` to `batch`, threading it onto `next[object_index]`.
/// `next` is the caller's own per-object "next free" array, one entry per
/// pooled object; `next[object_index]` must equal `object_index` on entry
/// (Jolt's own "not already in a free list" precondition) and is left
/// `invalid_object_index`, the new end of the chain.
pub fn addObjectToBatch(batch: *Batch, next: []u32, object_index: u32) void {
    std.debug.assert(next[object_index] == object_index);
    next[object_index] = invalid_object_index;
    if (batch.first == invalid_object_index) {
        batch.first = object_index;
    } else {
        next[batch.last] = object_index;
    }
    batch.last = object_index;
    batch.count += 1;
}

test "addObjectToBatch threads objects into one chain in append order" {
    var next = [_]u32{ 0, 1, 2, 3 };
    var batch = Batch{};

    addObjectToBatch(&batch, &next, 2);
    addObjectToBatch(&batch, &next, 0);
    addObjectToBatch(&batch, &next, 3);

    try std.testing.expectEqual(@as(u32, 3), batch.count);
    try std.testing.expectEqual(@as(u32, 2), batch.first);
    try std.testing.expectEqual(@as(u32, 3), batch.last);

    // Walk the chain from `first`, following `next`.
    var walked: [3]u32 = undefined;
    var cur = batch.first;
    for (0..3) |i| {
        walked[i] = cur;
        cur = next[cur];
    }
    try std.testing.expectEqual(cur, invalid_object_index);
    try std.testing.expectEqualSlices(u32, &.{ 2, 0, 3 }, &walked);
}

test "addObjectToBatch on a single object starts and ends the chain" {
    var next = [_]u32{0};
    var batch = Batch{};

    addObjectToBatch(&batch, &next, 0);

    try std.testing.expectEqual(@as(u32, 1), batch.count);
    try std.testing.expectEqual(batch.first, batch.last);
    try std.testing.expectEqual(invalid_object_index, next[0]);
}
