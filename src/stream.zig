//! A host-supplied byte stream.
//!
//! Every save and restore in this library that offers a two-call buffer form
//! also offers a stream form built on the same seam: instead of sizing and
//! holding the whole payload in memory, a caller hands bytes to its own pack
//! file, socket or compressor as they are produced. `State.saveStream`,
//! `Shape.saveStream`, `Scene.saveStream` and their restoring counterparts —
//! plus the object-stream methods `Scene` and `RagdollSettings` add on top of
//! that — all build one of these from whatever type implements the
//! directions they need.

const std = @import("std");
const c = @import("c/core.zig");

pub const Stream = c.Stream;

/// Which of Jolt's own object-stream formats a save writes. @see
/// `Scene.saveObjectStream` and `RagdollSettings.saveObjectStream`.
pub const ObjectStreamFormat = c.ObjectStreamFormat;

fn requireAnyDecl(comptime T: type, comptime names: []const []const u8) void {
    comptime {
        var found = false;
        for (names) |name| {
            if (@hasDecl(T, name)) found = true;
        }
        if (!found) {
            var list: []const u8 = "";
            for (names) |name| list = list ++ "\n  pub fn " ++ name;
            @compileError(@typeName(T) ++ " declares none of the methods a host stream" ++
                " looks for, so it would be installed and never usable. `@hasDecl` only" ++
                " sees `pub` declarations across files — check that yours are `pub`, and" ++
                " that they are spelled as one of:" ++ list);
        }
    }
}

/// Builds a `Stream` from `context` and whichever of `read`, `write`, `isEof`,
/// `isFailed` `T` declares — any it omits is simply never called.
///
/// `context` must outlive the call. Nothing may propagate out of these — no
/// `try`, no `error.Foo` — the no-unwinding rule every callback here carries:
/// stash a failure in `context`, read it back after the call returns.
pub fn hostStream(comptime T: type, context: *T) Stream {
    requireAnyDecl(T, &.{ "read", "write", "isEof", "isFailed" });

    const Thunks = struct {
        fn selfOf(user: ?*anyopaque) *T {
            return @ptrCast(@alignCast(user.?));
        }
        // A zero-length read or write carries a NULL pointer, because that
        // is what an empty `JPH::Array`'s `data()` is — and Jolt writes one
        // per empty constraint list. Dereferencing it to form a slice is a
        // panic in safe Zig, so the length is checked before the pointer.
        fn read(user: ?*anyopaque, data: ?*anyopaque, size: usize) callconv(.c) void {
            if (size == 0) return;
            const bytes: [*]u8 = @ptrCast(data.?);
            T.read(selfOf(user), bytes[0..size]);
        }
        fn write(user: ?*anyopaque, data: ?*const anyopaque, size: usize) callconv(.c) void {
            if (size == 0) return;
            const bytes: [*]const u8 = @ptrCast(data.?);
            T.write(selfOf(user), bytes[0..size]);
        }
        fn isEof(user: ?*anyopaque) callconv(.c) bool {
            return T.isEof(selfOf(user));
        }
        fn isFailed(user: ?*anyopaque) callconv(.c) bool {
            return T.isFailed(selfOf(user));
        }
    };

    return .{
        .read = if (@hasDecl(T, "read")) Thunks.read else null,
        .write = if (@hasDecl(T, "write")) Thunks.write else null,
        .is_eof = if (@hasDecl(T, "isEof")) Thunks.isEof else null,
        .is_failed = if (@hasDecl(T, "isFailed")) Thunks.isFailed else null,
        .user = @ptrCast(context),
    };
}

//=============================================================================
// A stream over a fixed buffer, for tests and for a caller who already has
// one but wants the stream entry points anyway — e.g. to compare a stream
// round trip against a buffer round trip byte for byte.
//=============================================================================

/// Grows-never, fails-on-overflow write side; `isFailed` is `overflowed`.
pub const BufferWriter = struct {
    buffer: []u8,
    written: usize = 0,
    overflowed: bool = false,

    pub fn write(self: *BufferWriter, data: []const u8) void {
        if (self.written + data.len > self.buffer.len) {
            self.overflowed = true;
            return;
        }
        @memcpy(self.buffer[self.written..][0..data.len], data);
        self.written += data.len;
    }

    pub fn isFailed(self: *BufferWriter) bool {
        return self.overflowed;
    }

    pub fn slice(self: *const BufferWriter) []const u8 {
        return self.buffer[0..self.written];
    }
};

/// Zero-fills past the end and reports EOF honestly, the same convention
/// every stream in this library uses.
pub const BufferReader = struct {
    buffer: []const u8,
    read_pos: usize = 0,
    eof: bool = false,

    pub fn read(self: *BufferReader, data: []u8) void {
        const available = if (self.read_pos < self.buffer.len) self.buffer.len - self.read_pos else 0;
        const taken = @min(data.len, available);
        @memcpy(data[0..taken], self.buffer[self.read_pos..][0..taken]);
        if (taken < data.len) {
            @memset(data[taken..], 0);
            self.eof = true;
        }
        self.read_pos += data.len;
    }

    pub fn isEof(self: *BufferReader) bool {
        return self.eof;
    }

    /// Always `false`: reading from memory has nothing else that can fail.
    /// Still declared, and not left `null`, because a restore entry point
    /// requires all of `read`, `is_eof` and `is_failed` together — @see
    /// `zjolt_core.h`'s `ZJoltStream`.
    pub fn isFailed(_: *BufferReader) bool {
        return false;
    }
};

//=============================================================================
// A fixed buffer written once, then read back from the start after
// `rewind` — one object standing in for a `BufferWriter` and a `BufferReader`
// over the same bytes, the way Jolt's own `StateRecorderImpl::Rewind` turns
// a single write-only stringstream around for reading.
//=============================================================================

/// Grows-never, fails-on-overflow write side; `isFailed` is `overflowed`,
/// the same convention as `BufferWriter`. `read` zero-fills past the last
/// byte written and reports EOF, the same convention as `BufferReader`.
pub const RewindableBuffer = struct {
    buffer: []u8,
    written: usize = 0,
    read_pos: usize = 0,
    overflowed: bool = false,
    eof: bool = false,

    pub fn write(self: *RewindableBuffer, data: []const u8) void {
        if (self.written + data.len > self.buffer.len) {
            self.overflowed = true;
            return;
        }
        @memcpy(self.buffer[self.written..][0..data.len], data);
        self.written += data.len;
    }

    /// Moves the read position back to the first byte written and clears
    /// `isEof`. Bytes already written are kept; a further `write` resumes
    /// after them, not at 0.
    pub fn rewind(self: *RewindableBuffer) void {
        self.read_pos = 0;
        self.eof = false;
    }

    pub fn read(self: *RewindableBuffer, data: []u8) void {
        const available = if (self.read_pos < self.written) self.written - self.read_pos else 0;
        const taken = @min(data.len, available);
        @memcpy(data[0..taken], self.buffer[self.read_pos..][0..taken]);
        if (taken < data.len) {
            @memset(data[taken..], 0);
            self.eof = true;
        }
        self.read_pos += data.len;
    }

    pub fn isEof(self: *RewindableBuffer) bool {
        return self.eof;
    }

    pub fn isFailed(self: *RewindableBuffer) bool {
        return self.overflowed;
    }

    pub fn slice(self: *const RewindableBuffer) []const u8 {
        return self.buffer[0..self.written];
    }
};
