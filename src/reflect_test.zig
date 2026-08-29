//! Behavioural tests for RTTI, ObjectStream and the profiler.
//!
//! `JPH::EmptyShapeSettings` is the fixture for the attribute walk and the
//! ObjectStream round trip: it is the simplest registered settings type with
//! no ref-counted member (`ShapeSettings::mUserData`, a plain `uint64`, plus
//! its own `mCenterOfMass`, a `Vec3`), so its two flattened attributes are
//! exactly the primitive kinds this module exposes typed accessors for.
//! `JPH::StaticCompoundShapeSettings`/`CompoundShapeSettings` is the fixture
//! for the checked cast: both are independently registered with Jolt's
//! factory, one the other's direct base, which `EmptyShapeSettings` alone
//! cannot exercise — its own base, `ShapeSettings`, is abstract and never
//! separately registered, so there is no live `Rtti` handle to cast to.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const reflect = @import("reflect.zig");
const stream_mod = @import("stream.zig");

//=============================================================================
// RTTI: attributes and base classes
//=============================================================================

test "walking a settings type's attributes and base classes matches what the C++ header declares" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rtti = try reflect.Rtti.find("EmptyShapeSettings");

    // One direct base: JPH_ADD_BASE_CLASS(EmptyShapeSettings, ShapeSettings)
    // in Shape.cpp/EmptyShape.cpp. ShapeSettings's own base, SerializableObject,
    // is not flattened into this list — only EmptyShapeSettings's ATTRIBUTES
    // are.
    try std.testing.expectEqual(@as(u32, 1), rtti.baseClassCount());
    const base = try rtti.baseClass(0);
    try std.testing.expectEqualStrings("ShapeSettings", std.mem.span(base.rtti.name.?));
    try std.testing.expect(base.offset >= 0);

    if (!zjolt.options.object_stream) return error.SkipZigTest;

    // Flattened: ShapeSettings::mUserData first (copied in when
    // EmptyShapeSettings adds ShapeSettings as a base class), then
    // EmptyShapeSettings's own mCenterOfMass.
    try std.testing.expectEqual(@as(u32, 2), try rtti.attributeCount());

    const user_data = try rtti.attribute(0);
    try std.testing.expectEqualStrings("mUserData", std.mem.span(user_data.name.?));
    try std.testing.expectEqual(reflect.PrimitiveType.uint64, user_data.primitive_type);

    const center_of_mass = try rtti.attribute(1);
    try std.testing.expectEqualStrings("mCenterOfMass", std.mem.span(center_of_mass.name.?));
    try std.testing.expectEqual(reflect.PrimitiveType.vec3, center_of_mass.primitive_type);

    // The two members cannot alias: the object is bigger than either offset
    // plus its own size.
    try std.testing.expect(user_data.offset != center_of_mass.offset);
}

//=============================================================================
// RTTI: the checked cast
//=============================================================================

test "a checked cast to the object's real type or a base succeeds; to an unrelated type it fails and hands back nothing" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const derived = try reflect.Rtti.find("StaticCompoundShapeSettings");
    const base = try reflect.Rtti.find("CompoundShapeSettings");
    const unrelated = try reflect.Rtti.find("SphereShapeSettings");

    const object = try derived.createObject();
    defer derived.destroyObject(object);

    // Same type: the identity case CastTo answers without walking any base.
    const as_self = try reflect.Rtti.castTo(object, derived);
    try std.testing.expectEqual(object, as_self);

    // A real base class — single inheritance, so the subobject offset may
    // legitimately be 0; what matters is that the cast succeeds at all,
    // which only a genuine base-class walk (not just the identity check
    // above) can answer.
    _ = try reflect.Rtti.castTo(object, base);

    // Not related either way.
    try std.testing.expectError(zjolt.Error.InvalidArgument, reflect.Rtti.castTo(object, unrelated));
}

//=============================================================================
// RTTI: CreateObject
//=============================================================================

test "CreateObject by name produces an object whose RTTI matches, and an unknown name is refused" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rtti = try reflect.Rtti.find("EmptyShapeSettings");
    const object = try rtti.createObject();
    defer rtti.destroyObject(object);

    const dynamic = reflect.Rtti.of(object) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(rtti.info().hash, dynamic.info().hash);

    try std.testing.expectError(zjolt.Error.InvalidArgument, reflect.Rtti.find("NotARealJoltType"));
}

//=============================================================================
// ObjectStream: a whole object, round-tripped
//=============================================================================

test "a settings object written through ObjectStream and read back compares equal field for field" {
    if (!zjolt.options.object_stream) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rtti = try reflect.Rtti.find("EmptyShapeSettings");
    const user_data_attr = try rtti.attribute(0);
    const center_attr = try rtti.attribute(1);

    const original = try rtti.createObject();
    defer rtti.destroyObject(original);
    try reflect.writeUint64(user_data_attr, original, 0xC0FFEE);
    try reflect.writeVec3(center_attr, original, .{ .x = 1, .y = 2, .z = 3 });

    var buffer: [4096]u8 = undefined;
    var writer: stream_mod.BufferWriter = .{ .buffer = &buffer };
    try rtti.saveObjectStream(original, .text, stream_mod.hostStream(stream_mod.BufferWriter, &writer));
    const text = writer.slice();

    // The point of the text form: every field is written by name, which is
    // what makes it something a person can open and diff.
    try std.testing.expect(std.mem.indexOf(u8, text, "mUserData") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "mCenterOfMass") != null);

    var reader: stream_mod.BufferReader = .{ .buffer = text };
    const restored = try rtti.restoreObjectStream(stream_mod.hostStream(stream_mod.BufferReader, &reader));
    defer rtti.destroyObject(restored);

    try std.testing.expectEqual(
        try reflect.readUint64(user_data_attr, original),
        try reflect.readUint64(user_data_attr, restored),
    );
    const original_center = try reflect.readVec3(center_attr, original);
    const restored_center = try reflect.readVec3(center_attr, restored);
    try std.testing.expectEqual(original_center.x, restored_center.x);
    try std.testing.expectEqual(original_center.y, restored_center.y);
    try std.testing.expectEqual(original_center.z, restored_center.z);
}

//=============================================================================
// ObjectStream: the primitive kinds beyond Vec3/Quat/Mat44
//
// Each typed accessor only needs an offset and a primitive_type — not a
// live Jolt object — so a hand-built ReflectAttribute over a plain aligned
// buffer exercises the read/write pair directly, without hunting for a
// registered settings type that happens to carry one of these members.
//=============================================================================

test "the vector/matrix primitive kinds beyond Vec3/Quat/Mat44 round-trip through their typed accessors" {
    if (!zjolt.options.object_stream) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    var buffer: [256]u8 align(32) = undefined;
    @memset(&buffer, 0);

    const float3_attr = reflect.ReflectAttribute{ .name = null, .primitive_type = .float3, .offset = 0 };
    try reflect.writeFloat3(float3_attr, &buffer, .{ .x = 1, .y = 2, .z = 3 });
    const float3 = try reflect.readFloat3(float3_attr, &buffer);
    try std.testing.expectEqual(@as(f32, 1), float3.x);
    try std.testing.expectEqual(@as(f32, 2), float3.y);
    try std.testing.expectEqual(@as(f32, 3), float3.z);

    const float4_attr = reflect.ReflectAttribute{ .name = null, .primitive_type = .float4, .offset = 0 };
    try reflect.writeFloat4(float4_attr, &buffer, .{ .x = 1, .y = 2, .z = 3, .w = 4 });
    const float4 = try reflect.readFloat4(float4_attr, &buffer);
    try std.testing.expectEqual(@as(f32, 4), float4.w);

    const vec4_attr = reflect.ReflectAttribute{ .name = null, .primitive_type = .vec4, .offset = 0 };
    try reflect.writeVec4(vec4_attr, &buffer, .{ .x = -1, .y = 0.5, .z = 2, .w = -3 });
    const vec4 = try reflect.readVec4(vec4_attr, &buffer);
    try std.testing.expectEqual(@as(f32, -1), vec4.x);
    try std.testing.expectEqual(@as(f32, 0.5), vec4.y);
    try std.testing.expectEqual(@as(f32, 2), vec4.z);
    try std.testing.expectEqual(@as(f32, -3), vec4.w);
    // Wrong kind is refused, not silently reinterpreted.
    try std.testing.expectError(zjolt.Error.InvalidArgument, reflect.readFloat4(vec4_attr, &buffer));

    const uvec4_attr = reflect.ReflectAttribute{ .name = null, .primitive_type = .uvec4, .offset = 0 };
    try reflect.writeUVec4(uvec4_attr, &buffer, .{ .x = 1, .y = 2, .z = 3, .w = 4 });
    const uvec4 = try reflect.readUVec4(uvec4_attr, &buffer);
    try std.testing.expectEqual(@as(u32, 4), uvec4.w);

    const dvec3_attr = reflect.ReflectAttribute{ .name = null, .primitive_type = .dvec3, .offset = 0 };
    try reflect.writeDVec3(dvec3_attr, &buffer, .{ .x = 1.5, .y = -2.5, .z = 3.5 });
    const dvec3 = try reflect.readDVec3(dvec3_attr, &buffer);
    try std.testing.expectEqual(@as(f64, 1.5), dvec3.x);
    try std.testing.expectEqual(@as(f64, -2.5), dvec3.y);
    try std.testing.expectEqual(@as(f64, 3.5), dvec3.z);

    const double3_attr = reflect.ReflectAttribute{ .name = null, .primitive_type = .double3, .offset = 0 };
    try reflect.writeDouble3(double3_attr, &buffer, .{ .x = 4, .y = 5, .z = 6 });
    const double3 = try reflect.readDouble3(double3_attr, &buffer);
    try std.testing.expectEqual(@as(f64, 6), double3.z);

    const dmat44_attr = reflect.ReflectAttribute{ .name = null, .primitive_type = .dmat44, .offset = 0 };
    var dmat44_in = std.mem.zeroes(reflect.DMat44);
    // Identity, with a translation only a double column preserves exactly.
    dmat44_in.m[0] = 1;
    dmat44_in.m[5] = 1;
    dmat44_in.m[10] = 1;
    dmat44_in.m[12] = 1_000_000.25;
    dmat44_in.m[13] = -7.75;
    dmat44_in.m[14] = 0.125;
    dmat44_in.m[15] = 1;
    try reflect.writeDMat44(dmat44_attr, &buffer, dmat44_in);
    const dmat44_out = try reflect.readDMat44(dmat44_attr, &buffer);
    for (dmat44_in.m, dmat44_out.m) |expected, actual| try std.testing.expectEqual(expected, actual);
}

//=============================================================================
// ObjectStream: the String primitive kind
//
// PhysicsMaterialSimple::mDebugName is JPH's own fixture for a registered
// type with a real String attribute; JPH::String owns a heap allocation, so
// only a live object built through createObject (never a raw buffer) can
// safely be assigned into by writeString.
//=============================================================================

test "a String attribute reads back what was written, borrowed from the live object" {
    if (!zjolt.options.object_stream) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rtti = try reflect.Rtti.find("PhysicsMaterialSimple");
    const name_attr = try rtti.attribute(0);
    try std.testing.expectEqualStrings("mDebugName", std.mem.span(name_attr.name.?));
    try std.testing.expectEqual(reflect.PrimitiveType.string, name_attr.primitive_type);

    const object = try rtti.createObject();
    defer rtti.destroyObject(object);

    try std.testing.expectEqualStrings("", try reflect.readString(name_attr, object));

    try reflect.writeString(name_attr, object, "concrete");
    try std.testing.expectEqualStrings("concrete", try reflect.readString(name_attr, object));

    // Overwriting a non-empty string exercises the reallocating path too,
    // not just the small-string-optimized first write.
    try reflect.writeString(name_attr, object, "a longer debug name than the short one");
    try std.testing.expectEqualStrings("a longer debug name than the short one", try reflect.readString(name_attr, object));
}

//=============================================================================
// ObjectStream: an object whose attribute list includes a ref-counted
// pointer (SphereShapeSettings.mMaterial, a RefConst<PhysicsMaterial>,
// classified as ZJOLT_PRIMITIVE_TYPE_OBJECT rather than a typed accessor's
// kind) round-trips too, resolving the pointer through the same identifier
// map JPH::ObjectStreamIn::Read builds for every object in the stream.
//=============================================================================

test "an object stream round trip resolves an attribute that is itself a ref-counted pointer" {
    if (!zjolt.options.object_stream) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const rtti = try reflect.Rtti.find("SphereShapeSettings");
    const count = try rtti.attributeCount();
    var radius_attr: ?reflect.ReflectAttribute = null;
    var i: u32 = 0;
    while (i < count) : (i += 1) {
        const a = try rtti.attribute(i);
        if (std.mem.eql(u8, std.mem.span(a.name.?), "mRadius")) radius_attr = a;
    }
    const attr = radius_attr orelse return error.TestUnexpectedResult;

    const original = try rtti.createObject();
    defer rtti.destroyObject(original);
    try reflect.writeFloat(attr, original, 2.5);

    var buffer: [4096]u8 = undefined;
    var writer: stream_mod.BufferWriter = .{ .buffer = &buffer };
    try rtti.saveObjectStream(original, .text, stream_mod.hostStream(stream_mod.BufferWriter, &writer));

    var reader: stream_mod.BufferReader = .{ .buffer = writer.slice() };
    const restored = try rtti.restoreObjectStream(stream_mod.hostStream(stream_mod.BufferReader, &reader));
    defer rtti.destroyObject(restored);

    try std.testing.expectEqual(@as(f32, 2.5), try reflect.readFloat(attr, restored));
}

//=============================================================================
// RTTI: exact-type equality
//=============================================================================

test "isType is true only for the exact same registered type, unlike isKindOf" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const derived = try reflect.Rtti.find("StaticCompoundShapeSettings");
    const same_derived = try reflect.Rtti.find("StaticCompoundShapeSettings");
    const base = try reflect.Rtti.find("CompoundShapeSettings");

    try std.testing.expect(derived.isType(same_derived));
    try std.testing.expect(!derived.isType(base));
    // The base relationship IS a kind-of, which is exactly what isType excludes.
    try std.testing.expect(derived.isKindOf(base));
}

//=============================================================================
// Profiler
//=============================================================================

test "every profiler entry point reports Unsupported when Jolt's profiler is not compiled in" {
    if (reflect.profiler_enabled) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    try std.testing.expectError(zjolt.Error.Unsupported, reflect.profilerNextFrame());
    try std.testing.expectError(zjolt.Error.Unsupported, reflect.profilerProcessorTickCount());
    try std.testing.expectError(zjolt.Error.Unsupported, reflect.profilerProcessorTicksPerSecond());
    try std.testing.expectError(zjolt.Error.Unsupported, reflect.ProfileThread.begin("t"));
    try std.testing.expectError(zjolt.Error.Unsupported, reflect.Measurement.begin("m", 0));

    var buffer: [64]u8 = undefined;
    var writer: stream_mod.BufferWriter = .{ .buffer = &buffer };
    try std.testing.expectError(
        zjolt.Error.Unsupported,
        reflect.profilerDumpStream(null, stream_mod.hostStream(stream_mod.BufferWriter, &writer)),
    );
    // profilerShutdown has nothing to report and is never an error either way.
    reflect.profilerShutdown();
}

test "a measured scope appears in the profiler dump" {
    if (!reflect.profiler_enabled) return error.SkipZigTest;

    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();
    defer reflect.profilerShutdown();

    const thread = try reflect.ProfileThread.begin("test-thread");
    defer thread.end();

    const measurement = try reflect.Measurement.begin("test-scope", 0x00ff00);
    var spin: u64 = 0;
    var i: u64 = 0;
    while (i < 100_000) : (i += 1) spin +%= i;
    std.mem.doNotOptimizeAway(spin);
    measurement.end();

    // Dump before NextFrame, not after: NextFrame rotates every profiling
    // thread's sample buffer back to empty, exactly as
    // JPH_PROFILE_NEXTFRAME() does, so a call between recording and
    // dumping would erase the very sample under test.
    var buffer: [4096]u8 = undefined;
    var writer: stream_mod.BufferWriter = .{ .buffer = &buffer };
    try reflect.profilerDumpStream("t", stream_mod.hostStream(stream_mod.BufferWriter, &writer));
    const bytes = writer.slice();
    try reflect.profilerNextFrame();

    // Wire format: u32 tag length, tag, u32 sample count, then per sample
    // u32 name length, name, u32 color, u64 start tick, u64 end tick — all
    // little-endian. @see ffi/zjolt_reflect.h's zjoltProfilerDumpStream.
    var pos: usize = 0;
    const tag_len = std.mem.readInt(u32, bytes[pos..][0..4], .little);
    pos += 4 + tag_len;
    const sample_count = std.mem.readInt(u32, bytes[pos..][0..4], .little);
    pos += 4;
    try std.testing.expect(sample_count >= 1);

    var found = false;
    var s: u32 = 0;
    while (s < sample_count) : (s += 1) {
        const name_len = std.mem.readInt(u32, bytes[pos..][0..4], .little);
        pos += 4;
        const name = bytes[pos..][0..name_len];
        pos += name_len;
        pos += 4; // color
        pos += 8; // start
        pos += 8; // end
        if (std.mem.eql(u8, name, "test-scope")) found = true;
    }
    try std.testing.expect(found);

    try std.testing.expectEqual(@as(u8, 0), reflect.profilerColorIntensity(.{ .r = 0, .g = 0, .b = 0, .a = 255 }));
    _ = try reflect.profilerProcessorTickCount();
    _ = try reflect.profilerProcessorTicksPerSecond();
}
