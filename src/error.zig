//! Translation between the C result enum and a Zig error set.

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
