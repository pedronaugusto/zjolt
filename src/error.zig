//! Translation between the C layer's two out-parameters and Zig: its
//! result enum to an error set, and its count to a slice.

const std = @import("std");
const c = @import("c/core.zig");

pub const Error = error{
    /// A call was made before `init`, or after `deinit`.
    NotInitialized,
    /// `init` was called twice without an intervening `deinit`.
    AlreadyInitialized,
    /// The Zig wrapper and the C library were built with different
    /// layout-affecting settings. See `zjolt.zig` for what that means.
    ConfigMismatch,
    /// The installed allocator returned null, or a hard limit was reached.
    OutOfMemory,
    /// A null handle, a missing required callback, or an out-of-domain scalar.
    InvalidArgument,
    /// The destination buffer was smaller than the required count, which is
    /// still reported through the count out-parameter.
    BufferTooSmall,
    /// Jolt refused to build the shape. `lastError` has its message.
    ShapeInvalid,
    /// Serialised data was truncated, trailing, or not a shape.
    BadFormat,
    /// The body id does not name a body in this system.
    BodyNotFound,
    /// The call names a feature this library was not built with. Every
    /// function exists in every build; whether it does anything does not.
    Unsupported,
    /// A host-supplied stream reported failure — a write it could not
    /// complete, or a read from a stream already in an error state.
    IoError,
    /// A save would have written less than was asked of it: a whole-system
    /// state save made while the system holds character controllers the save
    /// was not handed. @see `State.SaveOptions.characters`.
    StateIncomplete,
};

/// Turns a C result into a Zig error, or void on success.
///
/// The switch is exhaustive over `c.Result` with no `else` branch: adding a
/// result to the C enum without handling it here is a compile error. The
/// enumerator values themselves are checked against the header by
/// `abi_check.zig`.
pub fn check(result: c.Result) Error!void {
    return switch (result) {
        .ok => {},
        .not_initialized => Error.NotInitialized,
        .already_initialized => Error.AlreadyInitialized,
        .config_mismatch => Error.ConfigMismatch,
        .out_of_memory => Error.OutOfMemory,
        .invalid_argument => Error.InvalidArgument,
        .buffer_too_small => Error.BufferTooSmall,
        .shape_invalid => Error.ShapeInvalid,
        .bad_format => Error.BadFormat,
        .body_not_found => Error.BodyNotFound,
        .unsupported => Error.Unsupported,
        .io_error => Error.IoError,
        .state_incomplete => Error.StateIncomplete,
    };
}

/// The tail of a two-call query: the prefix of `buffer` the count names.
///
/// The count crosses the ABI, so it is checked rather than trusted. The C
/// layer returns `ZJOLT_RESULT_BUFFER_TOO_SMALL` instead of overrunning a
/// buffer; nothing on this side can see that it did, and `buffer[0..count]`
/// on a count past the end is undefined in a release build.
pub fn filled(buffer: anytype, count: usize) Error!Slice(@TypeOf(buffer)) {
    if (count > buffer.len) return Error.BufferTooSmall;
    return buffer[0..count];
}

/// `[]T` for either shape a caller's buffer arrives in -- a slice, or a
/// pointer to a fixed array -- keeping the element type and its constness.
fn Slice(comptime Buffer: type) type {
    const ptr = @typeInfo(Buffer).pointer;
    const child = if (ptr.size == .one) @typeInfo(ptr.child).array.child else ptr.child;
    return if (ptr.is_const) []const child else []child;
}

/// Borrowed, static description of a result, for logging.
pub fn name(result: c.Result) [:0]const u8 {
    return std.mem.span(c.zjoltResultName(result));
}

/// Detail for the most recent failure on this thread, or "" if there is none.
///
/// Jolt reports shape construction and deserialisation problems as strings,
/// and a Zig error set cannot carry one. This is where they survive; the
/// buffer is borrowed and is overwritten by the next failing call on this
/// thread.
pub fn lastError() [:0]const u8 {
    return std.mem.span(c.zjoltLastError());
}

//=============================================================================
// Tests
//
// Reflective over `c.Result` on purpose: a result added to the C enum is
// covered without anyone editing this. The exhaustive switch already makes an
// unhandled result a compile error; what this holds is the part no compiler
// sees -- that no two results collapse onto one error, and that no error in
// the set is unreachable. `zjolt.zig` holds the same walk over `resultName`.
//=============================================================================

const result_fields = @typeInfo(c.Result).@"enum".fields;

test "filled refuses a count its buffer cannot hold and hands back the prefix otherwise" {
    var buffer: [4]u32 = .{ 1, 2, 3, 4 };

    try std.testing.expectEqual(@as(usize, 0), (try filled(buffer[0..], 0)).len);
    try std.testing.expectEqualSlices(u32, &.{ 1, 2 }, try filled(buffer[0..], 2));
    try std.testing.expectEqualSlices(u32, &buffer, try filled(buffer[0..], 4));
    try std.testing.expectError(Error.BufferTooSmall, filled(buffer[0..], 5));

    // Both shapes a buffer arrives in, and a const one stays const.
    try std.testing.expect(@TypeOf(try filled(&buffer, 1)) == []u32);
    const view: []const u32 = &buffer;
    try std.testing.expect(@TypeOf(try filled(view, 1)) == []const u32);
}

test "check turns ok into success and every other result into an error of its own" {
    var seen: [result_fields.len]anyerror = undefined;
    var count: usize = 0;

    inline for (result_fields) |field| {
        const result: c.Result = @enumFromInt(field.value);
        if (result == .ok) {
            try check(result);
        } else if (check(result)) |_| {
            return error.FailingResultReportedSuccess;
        } else |e| {
            for (seen[0..count]) |earlier| try std.testing.expect(earlier != e);
            seen[count] = e;
            count += 1;
        }
    }

    // Both directions. The first says every failing result produced an error;
    // the second says every error in the set was produced by one, which is
    // what makes an error nothing can raise a failure rather than dead code.
    try std.testing.expectEqual(result_fields.len - 1, count);
    try std.testing.expectEqual(@typeInfo(Error).error_set.?.len, count);
}
