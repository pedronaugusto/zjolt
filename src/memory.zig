//! Bridges a Zig `std.mem.Allocator` onto Jolt's global allocator seam.
//!
//! ## Why this is not a five-line shim
//!
//! Jolt frees with `free(block)` and `aligned_free(block)` — no size, no
//! alignment. Zig's allocator interface requires both back at free time. The
//! gap is closed by allocating a little extra and stashing the length and
//! alignment in a header placed immediately before the pointer handed to Jolt:
//!
//! ```text
//!   base                       returned pointer
//!    |                          |
//!    v                          v
//!   [ .... padding .... ][Header][ ..... payload ..... ]
//!    \___ prefix, a multiple of the backing alignment ___/
//! ```
//!
//! `reallocate` is the one hook Jolt hands the old size to, so it could avoid
//! the header — but it must interoperate with blocks that `allocate` produced,
//! so it reads the header too and stays consistent.
//!
//! ## Alignment
//!
//! Jolt's plain `allocate` takes no alignment yet places SIMD types in the
//! memory it returns, so it has a minimum it does not state in the signature.
//! That minimum is read from `zjoltDefaultAllocateAlignment()` at install time
//! rather than assumed to be 16 — on a 32-bit target it is 8.
//!
//! ## Global state
//!
//! Jolt's allocator is process-wide, so this one is too. That is a property of
//! Jolt, not a shortcut taken here: it is surfaced rather than hidden behind a
//! per-system allocator parameter that could not be honoured.

const std = @import("std");
const c = @import("c/core.zig");

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
// These drive the hooks directly rather than through a physics system, so the
// header arithmetic is covered on its own — including the sizes and alignments
// that Jolt would only reach under specific workloads.
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
