//! RTTI over Jolt's own registered types, ObjectStream for any of them, and
//! the profiler.
//!
//! `Rtti.find`/`findByHash` look a registered type up the way
//! `zjolt.factoryFind` does, but hand back a handle `baseClass`/`attribute`
//! can walk rather than a snapshot. `createObject` builds an instance
//! through Jolt's own factory function; a member crosses through a typed
//! accessor — `readVec3`/`writeFloat` and the rest below — keyed by
//! `ReflectAttribute.primitive_type`, never as a raw pointer.
//! `saveObjectStream`/`restoreObjectStream` read/write the whole object,
//! text or binary, over the same `Stream` seam `Scene.saveObjectStream` uses.
//!
//! Every function needing `-Dobject_stream=true` or `-Dprofile=true` fails
//! with `error.Unsupported` when not built with it; `zjolt.options.object_stream`/`zjolt.options.profile` say which, without failing a call first.

const std = @import("std");
const c = @import("c/reflect.zig");
const err = @import("error.zig");
const stream_mod = @import("stream.zig");
const options = @import("zjolt_options");

/// Whether Jolt's own profiler is compiled in — what every `profiler*` and
/// `Measurement` entry point here needs. `-Dprofile` and
/// `-Dtrack_broadphase_stats` each ask for it; `-Dexternal_profile` selects a
/// different, mutually exclusive state of `Jolt/Core/Profiler.h` and wins over
/// both. Everything gated on this returns `error.Unsupported` when it is false.
pub const profiler_enabled =
    (options.profile or options.track_broadphase_stats) and !options.external_profile;

pub const RttiInfo = c.RttiInfo;
pub const ReflectBaseClass = c.ReflectBaseClass;
pub const PrimitiveType = c.PrimitiveType;
pub const ReflectAttribute = c.ReflectAttribute;
pub const Vec3 = c.Vec3;
pub const Quat = c.Quat;
pub const Mat44 = c.Mat44;
pub const Color = c.Color;
pub const Vec4 = c.Vec4;
pub const UVec4 = c.UVec4;
pub const DVec3 = c.DVec3;
pub const DMat44 = c.DMat44;

//=============================================================================
// RTTI
//=============================================================================

/// A registered type's runtime type information. Borrowed: Jolt owns every
/// RTTI it registers for the life of the process.
pub const Rtti = struct {
    handle: *const c.Rtti,

    pub fn find(name: [:0]const u8) err.Error!Rtti {
        var handle: *const c.Rtti = undefined;
        try err.check(c.zjoltReflectFind(name.ptr, &handle));
        return .{ .handle = handle };
    }

    /// As `find`, by the hash a `RttiInfo` carries.
    pub fn findByHash(hash: u32) err.Error!Rtti {
        var handle: *const c.Rtti = undefined;
        try err.check(c.zjoltReflectFindByHash(hash, &handle));
        return .{ .handle = handle };
    }

    pub fn info(self: Rtti) RttiInfo {
        var out: RttiInfo = undefined;
        c.zjoltReflectGetInfo(self.handle, &out);
        return out;
    }

    /// Direct base classes only — not flattened through a base's own bases.
    pub fn baseClassCount(self: Rtti) u32 {
        return c.zjoltReflectGetBaseClassCount(self.handle);
    }

    /// `error.InvalidArgument` past `baseClassCount`.
    pub fn baseClass(self: Rtti, index: u32) err.Error!ReflectBaseClass {
        var out: ReflectBaseClass = undefined;
        try err.check(c.zjoltReflectGetBaseClass(self.handle, index, &out));
        return out;
    }

    pub fn isKindOf(self: Rtti, base: Rtti) bool {
        return c.zjoltReflectIsKindOf(self.handle, base.handle);
    }

    /// Exact-type equality: true only when `self` and `other` name the same
    /// registered type. Unlike `isKindOf`, a derived or base type answers
    /// false. Every `Rtti` for a given type shares one handle for the life
    /// of the process, so this never crosses the ABI.
    pub fn isType(self: Rtti, other: Rtti) bool {
        return self.handle == other.handle;
    }

    /// Flattened across every base class. `error.Unsupported` unless built
    /// with `-Dobject_stream=true`.
    pub fn attributeCount(self: Rtti) err.Error!u32 {
        var out: u32 = 0;
        try err.check(c.zjoltReflectGetAttributeCount(self.handle, &out));
        return out;
    }

    /// `error.InvalidArgument` past `attributeCount`.
    pub fn attribute(self: Rtti, index: u32) err.Error!ReflectAttribute {
        var out: ReflectAttribute = undefined;
        try err.check(c.zjoltReflectGetAttribute(self.handle, index, &out));
        return out;
    }

    /// Builds an instance through Jolt's own factory function.
    /// `error.InvalidArgument` if this type is abstract, or is not a
    /// `JPH::SerializableObject` descendant. Release with `destroyObject`;
    /// read or write a member through an `ReflectAttribute` from `attribute` and
    /// `readVec3`/`writeFloat`/etc below; save or restore the whole object
    /// with `saveObjectStream`/`restoreObjectStream`.
    pub fn createObject(self: Rtti) err.Error!*anyopaque {
        var out: *anyopaque = undefined;
        try err.check(c.zjoltReflectCreateObject(self.handle, &out));
        return out;
    }

    /// Frees an object `createObject` or `restoreObjectStream` returned.
    /// Owned like any other `deinit` in this package, not reference counted.
    pub fn destroyObject(self: Rtti, object: ?*anyopaque) void {
        c.zjoltReflectDestroyObject(self.handle, object);
    }

    /// `object`'s own dynamic type.
    pub fn of(object: *const anyopaque) ?Rtti {
        const handle = c.zjoltReflectObjectGetRtti(object) orelse return null;
        return .{ .handle = handle };
    }

    /// The one checked-cast standing in for CastTo/StaticCast/DynamicCast:
    /// succeeds when `object`'s real type is `target` or transitively
    /// derived from it. `error.InvalidArgument` otherwise — never a pointer
    /// of the wrong type.
    pub fn castTo(object: *anyopaque, target: Rtti) err.Error!*anyopaque {
        var out: *anyopaque = undefined;
        try err.check(c.zjoltReflectCastTo(object, target.handle, &out));
        return out;
    }

    /// Writes `object` (an instance of this type) through `stream` in Jolt's
    /// object-stream format. `error.Unsupported` unless built with
    /// `-Dobject_stream=true`.
    pub fn saveObjectStream(
        self: Rtti,
        object: *const anyopaque,
        format: stream_mod.ObjectStreamFormat,
        stream: stream_mod.Stream,
    ) err.Error!void {
        try err.check(c.zjoltReflectObjectSaveStream(self.handle, object, format, &stream));
    }

    /// Reads an object of this type (or a type derived from it) back from
    /// `stream`; text or binary is sniffed from the stream's own header.
    /// Release with `destroyObject`. `error.Unsupported` unless built with
    /// `-Dobject_stream=true`.
    pub fn restoreObjectStream(self: Rtti, stream: stream_mod.Stream) err.Error!*anyopaque {
        var out: *anyopaque = undefined;
        try err.check(c.zjoltReflectObjectRestoreStream(self.handle, &stream, &out));
        return out;
    }
};

//=============================================================================
// Typed member access
//
// Each pair reads or writes exactly `attr`'s member on `object`, failing with `error.InvalidArgument` when `attr.primitive_type` names a different kind.
//=============================================================================

pub fn readUint8(attr: ReflectAttribute, object: *const anyopaque) err.Error!u8 {
    var out: u8 = undefined;
    try err.check(c.zjoltReflectAttributeReadUint8(&attr, object, &out));
    return out;
}
pub fn writeUint8(attr: ReflectAttribute, object: *anyopaque, value: u8) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteUint8(&attr, object, value));
}

pub fn readUint16(attr: ReflectAttribute, object: *const anyopaque) err.Error!u16 {
    var out: u16 = undefined;
    try err.check(c.zjoltReflectAttributeReadUint16(&attr, object, &out));
    return out;
}
pub fn writeUint16(attr: ReflectAttribute, object: *anyopaque, value: u16) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteUint16(&attr, object, value));
}

pub fn readInt32(attr: ReflectAttribute, object: *const anyopaque) err.Error!i32 {
    var out: i32 = undefined;
    try err.check(c.zjoltReflectAttributeReadInt32(&attr, object, &out));
    return out;
}
pub fn writeInt32(attr: ReflectAttribute, object: *anyopaque, value: i32) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteInt32(&attr, object, value));
}

pub fn readUint32(attr: ReflectAttribute, object: *const anyopaque) err.Error!u32 {
    var out: u32 = undefined;
    try err.check(c.zjoltReflectAttributeReadUint32(&attr, object, &out));
    return out;
}
pub fn writeUint32(attr: ReflectAttribute, object: *anyopaque, value: u32) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteUint32(&attr, object, value));
}

pub fn readUint64(attr: ReflectAttribute, object: *const anyopaque) err.Error!u64 {
    var out: u64 = undefined;
    try err.check(c.zjoltReflectAttributeReadUint64(&attr, object, &out));
    return out;
}
pub fn writeUint64(attr: ReflectAttribute, object: *anyopaque, value: u64) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteUint64(&attr, object, value));
}

pub fn readFloat(attr: ReflectAttribute, object: *const anyopaque) err.Error!f32 {
    var out: f32 = undefined;
    try err.check(c.zjoltReflectAttributeReadFloat(&attr, object, &out));
    return out;
}
pub fn writeFloat(attr: ReflectAttribute, object: *anyopaque, value: f32) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteFloat(&attr, object, value));
}

pub fn readDouble(attr: ReflectAttribute, object: *const anyopaque) err.Error!f64 {
    var out: f64 = undefined;
    try err.check(c.zjoltReflectAttributeReadDouble(&attr, object, &out));
    return out;
}
pub fn writeDouble(attr: ReflectAttribute, object: *anyopaque, value: f64) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteDouble(&attr, object, value));
}

pub fn readBool(attr: ReflectAttribute, object: *const anyopaque) err.Error!bool {
    var out: bool = undefined;
    try err.check(c.zjoltReflectAttributeReadBool(&attr, object, &out));
    return out;
}
pub fn writeBool(attr: ReflectAttribute, object: *anyopaque, value: bool) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteBool(&attr, object, value));
}

pub fn readVec3(attr: ReflectAttribute, object: *const anyopaque) err.Error!Vec3 {
    var out: Vec3 = undefined;
    try err.check(c.zjoltReflectAttributeReadVec3(&attr, object, &out));
    return out;
}
pub fn writeVec3(attr: ReflectAttribute, object: *anyopaque, value: Vec3) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteVec3(&attr, object, value));
}

pub fn readQuat(attr: ReflectAttribute, object: *const anyopaque) err.Error!Quat {
    var out: Quat = undefined;
    try err.check(c.zjoltReflectAttributeReadQuat(&attr, object, &out));
    return out;
}
pub fn writeQuat(attr: ReflectAttribute, object: *anyopaque, value: Quat) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteQuat(&attr, object, value));
}

pub fn readMat44(attr: ReflectAttribute, object: *const anyopaque) err.Error!Mat44 {
    var out: Mat44 = undefined;
    try err.check(c.zjoltReflectAttributeReadMat44(&attr, object, &out));
    return out;
}
pub fn writeMat44(attr: ReflectAttribute, object: *anyopaque, value: Mat44) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteMat44(&attr, object, value));
}

/// Borrowed, valid until this attribute is next written on `object` or
/// `object` is destroyed.
pub fn readString(attr: ReflectAttribute, object: *const anyopaque) err.Error![:0]const u8 {
    var out: ?[*:0]const u8 = null;
    try err.check(c.zjoltReflectAttributeReadString(&attr, object, &out));
    return std.mem.span(out.?);
}
pub fn writeString(attr: ReflectAttribute, object: *anyopaque, value: [:0]const u8) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteString(&attr, object, value.ptr));
}

pub fn readFloat3(attr: ReflectAttribute, object: *const anyopaque) err.Error!Vec3 {
    var out: Vec3 = undefined;
    try err.check(c.zjoltReflectAttributeReadFloat3(&attr, object, &out));
    return out;
}
pub fn writeFloat3(attr: ReflectAttribute, object: *anyopaque, value: Vec3) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteFloat3(&attr, object, value));
}

pub fn readFloat4(attr: ReflectAttribute, object: *const anyopaque) err.Error!Vec4 {
    var out: Vec4 = undefined;
    try err.check(c.zjoltReflectAttributeReadFloat4(&attr, object, &out));
    return out;
}
pub fn writeFloat4(attr: ReflectAttribute, object: *anyopaque, value: Vec4) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteFloat4(&attr, object, value));
}

pub fn readVec4(attr: ReflectAttribute, object: *const anyopaque) err.Error!Vec4 {
    var out: Vec4 = undefined;
    try err.check(c.zjoltReflectAttributeReadVec4(&attr, object, &out));
    return out;
}
pub fn writeVec4(attr: ReflectAttribute, object: *anyopaque, value: Vec4) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteVec4(&attr, object, value));
}

pub fn readUVec4(attr: ReflectAttribute, object: *const anyopaque) err.Error!UVec4 {
    var out: UVec4 = undefined;
    try err.check(c.zjoltReflectAttributeReadUVec4(&attr, object, &out));
    return out;
}
pub fn writeUVec4(attr: ReflectAttribute, object: *anyopaque, value: UVec4) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteUVec4(&attr, object, value));
}

pub fn readDouble3(attr: ReflectAttribute, object: *const anyopaque) err.Error!DVec3 {
    var out: DVec3 = undefined;
    try err.check(c.zjoltReflectAttributeReadDouble3(&attr, object, &out));
    return out;
}
pub fn writeDouble3(attr: ReflectAttribute, object: *anyopaque, value: DVec3) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteDouble3(&attr, object, value));
}

pub fn readDVec3(attr: ReflectAttribute, object: *const anyopaque) err.Error!DVec3 {
    var out: DVec3 = undefined;
    try err.check(c.zjoltReflectAttributeReadDVec3(&attr, object, &out));
    return out;
}
pub fn writeDVec3(attr: ReflectAttribute, object: *anyopaque, value: DVec3) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteDVec3(&attr, object, value));
}

pub fn readDMat44(attr: ReflectAttribute, object: *const anyopaque) err.Error!DMat44 {
    var out: DMat44 = undefined;
    try err.check(c.zjoltReflectAttributeReadDMat44(&attr, object, &out));
    return out;
}
pub fn writeDMat44(attr: ReflectAttribute, object: *anyopaque, value: DMat44) err.Error!void {
    try err.check(c.zjoltReflectAttributeWriteDMat44(&attr, object, value));
}

//=============================================================================
// Profiler
//=============================================================================

/// Ends the current profiling frame. `error.Unsupported` unless built with
/// `-Dprofile=true`.
pub fn profilerNextFrame() err.Error!void {
    try err.check(c.zjoltProfilerNextFrame());
}

/// Tears down the calling thread's `ProfileThread`, if any, and the
/// process-wide profiler singleton — see `ffi/zjolt_reflect.h` for what this
/// does and does not reach. A no-op with nothing started.
pub fn profilerShutdown() void {
    c.zjoltProfilerShutdown();
}

/// Writes the calling thread's recorded samples through `stream` — see
/// `ffi/zjolt_reflect.h` for the wire format and why this stands in for
/// Jolt's own file-writing Dump/DumpChart. `error.InvalidArgument` if the
/// calling thread has no `ProfileThread` active.
pub fn profilerDumpStream(tag: ?[:0]const u8, stream: stream_mod.Stream) err.Error!void {
    try err.check(c.zjoltProfilerDumpStream(if (tag) |t| t.ptr else null, &stream));
}

pub fn profilerProcessorTickCount() err.Error!u64 {
    var out: u64 = 0;
    try err.check(c.zjoltProfilerGetProcessorTickCount(&out));
    return out;
}

pub fn profilerProcessorTicksPerSecond() err.Error!u64 {
    var out: u64 = 0;
    try err.check(c.zjoltProfilerGetProcessorTicksPerSecond(&out));
    return out;
}

/// Perceived brightness of `color`, 0-255. Always available.
pub fn profilerColorIntensity(color: Color) u8 {
    return c.zjoltProfilerColorGetIntensity(color);
}

/// Registers the calling thread for instrumentation. Every `Measurement` on
/// this thread needs one active; end it with `end` before another begins on
/// the same thread.
pub const ProfileThread = struct {
    pub fn begin(name: [:0]const u8) err.Error!ProfileThread {
        try err.check(c.zjoltProfileThreadBegin(name.ptr));
        return .{};
    }

    pub fn end(self: ProfileThread) void {
        _ = self;
        c.zjoltProfileThreadEnd();
    }
};

/// One scoped timing sample. Needs a `ProfileThread` active on the calling
/// thread; without one, `end` records nothing.
pub const Measurement = struct {
    handle: *c.ProfileMeasurement,

    pub fn begin(name: [:0]const u8, color: u32) err.Error!Measurement {
        var handle: *c.ProfileMeasurement = undefined;
        try err.check(c.zjoltProfileMeasurementBegin(name.ptr, color, &handle));
        return .{ .handle = handle };
    }

    pub fn end(self: Measurement) void {
        c.zjoltProfileMeasurementEnd(self.handle);
    }
};
