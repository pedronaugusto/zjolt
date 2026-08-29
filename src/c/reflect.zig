//! ZJolt C declarations for RTTI, ObjectStream and the profiler.
//!
//! Mirrors `ffi/zjolt_reflect.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Result = core.Result;
pub const RttiInfo = core.RttiInfo;
pub const Stream = core.Stream;
pub const ObjectStreamFormat = core.ObjectStreamFormat;
pub const Vec3 = core.Vec3;
pub const Quat = core.Quat;
pub const Mat44 = core.Mat44;
pub const Color = core.Color;

//=============================================================================
// RTTI
//=============================================================================

pub const Rtti = opaque {};

pub extern fn zjoltReflectFind(name: [*:0]const u8, out: **const Rtti) Result;
pub extern fn zjoltReflectFindByHash(hash: u32, out: **const Rtti) Result;
pub extern fn zjoltReflectGetInfo(rtti: ?*const Rtti, out: *RttiInfo) void;
pub extern fn zjoltReflectGetBaseClassCount(rtti: ?*const Rtti) u32;

pub const ReflectBaseClass = extern struct {
    rtti: RttiInfo,
    offset: i32,
};

pub extern fn zjoltReflectGetBaseClass(rtti: *const Rtti, index: u32, out: *ReflectBaseClass) Result;
pub extern fn zjoltReflectIsKindOf(rtti: ?*const Rtti, base: ?*const Rtti) bool;
pub extern fn zjoltReflectCreateObject(rtti: *const Rtti, out: **anyopaque) Result;
pub extern fn zjoltReflectDestroyObject(rtti: ?*const Rtti, object: ?*anyopaque) void;
pub extern fn zjoltReflectObjectGetRtti(object: ?*const anyopaque) ?*const Rtti;
pub extern fn zjoltReflectCastTo(object: *anyopaque, target: *const Rtti, out: **anyopaque) Result;

//=============================================================================
// ObjectStream — member access
//=============================================================================

/// Mirrors Jolt/ObjectStream/ObjectStreamTypes.h, plus OBJECT for a member
/// that is a nested class instance or pointer rather than one of these.
pub const PrimitiveType = enum(c_int) {
    uint8 = 0,
    uint16 = 1,
    int32 = 2,
    uint32 = 3,
    uint64 = 4,
    float = 5,
    bool = 6,
    string = 7,
    float3 = 8,
    vec3 = 9,
    vec4 = 10,
    quat = 11,
    mat44 = 12,
    double = 13,
    dvec3 = 14,
    dmat44 = 15,
    double3 = 16,
    float4 = 17,
    uvec4 = 18,
    object = 19,
};

pub const ReflectAttribute = extern struct {
    name: ?[*:0]const u8,
    primitive_type: PrimitiveType,
    offset: u32,
};

/// A four-component float vector, no padding lane: wire type for both
/// `PrimitiveType.vec4` (JPH::Vec4) and `.float4` (JPH::Float4).
pub const Vec4 = extern struct {
    x: f32,
    y: f32,
    z: f32,
    w: f32,
};

/// A four-component unsigned-integer vector: wire type for `PrimitiveType.uvec4`.
pub const UVec4 = extern struct {
    x: u32,
    y: u32,
    z: u32,
    w: u32,
};

/// A three-component double vector, always `f64` regardless of the
/// `-Ddouble_precision` build option: wire type for both `PrimitiveType.dvec3`
/// (JPH::DVec3) and `.double3` (JPH::Double3).
pub const DVec3 = extern struct {
    x: f64,
    y: f64,
    z: f64,
};

/// A 4x4 matrix, sixteen doubles in `Mat44`'s column-major layout, always
/// `f64` regardless of `-Ddouble_precision`: wire type for `PrimitiveType.dmat44`.
pub const DMat44 = extern struct {
    m: [16]f64,
};

pub extern fn zjoltReflectGetAttributeCount(rtti: *const Rtti, out_count: *u32) Result;
pub extern fn zjoltReflectGetAttribute(rtti: *const Rtti, index: u32, out: *ReflectAttribute) Result;

pub extern fn zjoltReflectAttributeReadUint8(attr: *const ReflectAttribute, object: *const anyopaque, out: *u8) Result;
pub extern fn zjoltReflectAttributeWriteUint8(attr: *const ReflectAttribute, object: *anyopaque, value: u8) Result;

pub extern fn zjoltReflectAttributeReadUint16(attr: *const ReflectAttribute, object: *const anyopaque, out: *u16) Result;
pub extern fn zjoltReflectAttributeWriteUint16(attr: *const ReflectAttribute, object: *anyopaque, value: u16) Result;

pub extern fn zjoltReflectAttributeReadInt32(attr: *const ReflectAttribute, object: *const anyopaque, out: *i32) Result;
pub extern fn zjoltReflectAttributeWriteInt32(attr: *const ReflectAttribute, object: *anyopaque, value: i32) Result;

pub extern fn zjoltReflectAttributeReadUint32(attr: *const ReflectAttribute, object: *const anyopaque, out: *u32) Result;
pub extern fn zjoltReflectAttributeWriteUint32(attr: *const ReflectAttribute, object: *anyopaque, value: u32) Result;

pub extern fn zjoltReflectAttributeReadUint64(attr: *const ReflectAttribute, object: *const anyopaque, out: *u64) Result;
pub extern fn zjoltReflectAttributeWriteUint64(attr: *const ReflectAttribute, object: *anyopaque, value: u64) Result;

pub extern fn zjoltReflectAttributeReadFloat(attr: *const ReflectAttribute, object: *const anyopaque, out: *f32) Result;
pub extern fn zjoltReflectAttributeWriteFloat(attr: *const ReflectAttribute, object: *anyopaque, value: f32) Result;

pub extern fn zjoltReflectAttributeReadDouble(attr: *const ReflectAttribute, object: *const anyopaque, out: *f64) Result;
pub extern fn zjoltReflectAttributeWriteDouble(attr: *const ReflectAttribute, object: *anyopaque, value: f64) Result;

pub extern fn zjoltReflectAttributeReadBool(attr: *const ReflectAttribute, object: *const anyopaque, out: *bool) Result;
pub extern fn zjoltReflectAttributeWriteBool(attr: *const ReflectAttribute, object: *anyopaque, value: bool) Result;

pub extern fn zjoltReflectAttributeReadVec3(attr: *const ReflectAttribute, object: *const anyopaque, out: *Vec3) Result;
pub extern fn zjoltReflectAttributeWriteVec3(attr: *const ReflectAttribute, object: *anyopaque, value: Vec3) Result;

pub extern fn zjoltReflectAttributeReadQuat(attr: *const ReflectAttribute, object: *const anyopaque, out: *Quat) Result;
pub extern fn zjoltReflectAttributeWriteQuat(attr: *const ReflectAttribute, object: *anyopaque, value: Quat) Result;

pub extern fn zjoltReflectAttributeReadMat44(attr: *const ReflectAttribute, object: *const anyopaque, out: *Mat44) Result;
pub extern fn zjoltReflectAttributeWriteMat44(attr: *const ReflectAttribute, object: *anyopaque, value: Mat44) Result;

pub extern fn zjoltReflectAttributeReadString(attr: *const ReflectAttribute, object: *const anyopaque, out: *?[*:0]const u8) Result;
pub extern fn zjoltReflectAttributeWriteString(attr: *const ReflectAttribute, object: *anyopaque, value: [*:0]const u8) Result;

pub extern fn zjoltReflectAttributeReadFloat3(attr: *const ReflectAttribute, object: *const anyopaque, out: *Vec3) Result;
pub extern fn zjoltReflectAttributeWriteFloat3(attr: *const ReflectAttribute, object: *anyopaque, value: Vec3) Result;

pub extern fn zjoltReflectAttributeReadFloat4(attr: *const ReflectAttribute, object: *const anyopaque, out: *Vec4) Result;
pub extern fn zjoltReflectAttributeWriteFloat4(attr: *const ReflectAttribute, object: *anyopaque, value: Vec4) Result;

pub extern fn zjoltReflectAttributeReadVec4(attr: *const ReflectAttribute, object: *const anyopaque, out: *Vec4) Result;
pub extern fn zjoltReflectAttributeWriteVec4(attr: *const ReflectAttribute, object: *anyopaque, value: Vec4) Result;

pub extern fn zjoltReflectAttributeReadUVec4(attr: *const ReflectAttribute, object: *const anyopaque, out: *UVec4) Result;
pub extern fn zjoltReflectAttributeWriteUVec4(attr: *const ReflectAttribute, object: *anyopaque, value: UVec4) Result;

pub extern fn zjoltReflectAttributeReadDouble3(attr: *const ReflectAttribute, object: *const anyopaque, out: *DVec3) Result;
pub extern fn zjoltReflectAttributeWriteDouble3(attr: *const ReflectAttribute, object: *anyopaque, value: DVec3) Result;

pub extern fn zjoltReflectAttributeReadDVec3(attr: *const ReflectAttribute, object: *const anyopaque, out: *DVec3) Result;
pub extern fn zjoltReflectAttributeWriteDVec3(attr: *const ReflectAttribute, object: *anyopaque, value: DVec3) Result;

pub extern fn zjoltReflectAttributeReadDMat44(attr: *const ReflectAttribute, object: *const anyopaque, out: *DMat44) Result;
pub extern fn zjoltReflectAttributeWriteDMat44(attr: *const ReflectAttribute, object: *anyopaque, value: DMat44) Result;

//=============================================================================
// ObjectStream — whole objects
//=============================================================================

pub extern fn zjoltReflectObjectSaveStream(rtti: *const Rtti, object: *const anyopaque, format: ObjectStreamFormat, stream: *const Stream) Result;
pub extern fn zjoltReflectObjectRestoreStream(rtti: *const Rtti, stream: *const Stream, out: **anyopaque) Result;

//=============================================================================
// Profiler
//=============================================================================

pub extern fn zjoltProfilerNextFrame() Result;
pub extern fn zjoltProfilerShutdown() void;
pub extern fn zjoltProfilerDumpStream(tag: ?[*:0]const u8, stream: *const Stream) Result;
pub extern fn zjoltProfilerGetProcessorTickCount(out: *u64) Result;
pub extern fn zjoltProfilerGetProcessorTicksPerSecond(out: *u64) Result;
pub extern fn zjoltProfilerColorGetIntensity(color: Color) u8;

pub extern fn zjoltProfileThreadBegin(name: [*:0]const u8) Result;
pub extern fn zjoltProfileThreadEnd() void;

pub const ProfileMeasurement = opaque {};

pub extern fn zjoltProfileMeasurementBegin(name: [*:0]const u8, color: u32, out: **ProfileMeasurement) Result;
pub extern fn zjoltProfileMeasurementEnd(measurement: ?*ProfileMeasurement) void;
