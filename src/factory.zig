//! What is left of `ffi/zjolt_core.h` once initialisation and the job system
//! are accounted for: Jolt's Factory, the process-wide registry
//! `Shape::sRestoreFromBinaryState` and its siblings use to look a
//! serialisable type up by name or hash.
//!
//! `init`/`deinit` live in `zjolt.zig` itself rather than here, since they
//! are the one thing every other module in this package depends on rather
//! than a subsystem of their own. `JobSystem` lives in `system.zig`,
//! alongside the physics system it steps.
//!
//! Registering a type, or constructing one from the registry, needs a C++
//! vtable this ABI does not mirror (see `zjoltFactoryRegister`/
//! `zjoltFactoryCreateObject` in `ffi/zjolt_core.h`). What crosses is a
//! plain read: a registered type's name, hash, size, and whether abstract.

const std = @import("std");
const c = @import("c/core.zig");
const err = @import("error.zig");

/// What `find`/`findByHash`/`getAllClasses` report about one type Jolt's
/// Factory knows — every one this library's `init` registered, which is
/// every type Jolt itself defines.
pub const RttiInfo = c.RttiInfo;

/// Looks a registered type up by name, such as `"SphereShapeSettings"`. Null
/// if nothing is registered under it, or before `init`.
pub fn find(name: [:0]const u8) ?RttiInfo {
    var out: RttiInfo = undefined;
    c.zjoltFactoryFind(name.ptr, &out);
    return if (out.name != null) out else null;
}

/// Same lookup, by the hash `find`'s own result carries — the cheaper
/// comparison a saved stream's type tag actually uses.
pub fn findByHash(hash: u32) ?RttiInfo {
    var out: RttiInfo = undefined;
    c.zjoltFactoryFindByHash(hash, &out);
    return if (out.name != null) out else null;
}

/// Every type this library's `init` registered with Jolt's Factory.
pub fn countClasses() usize {
    var count: u32 = 0;
    // Cannot fail: NULL out with a real count pointer only reports the size.
    _ = c.zjoltFactoryGetAllClasses(null, 0, &count);
    return count;
}

/// `getAllClasses` into memory from `allocator`. The caller owns the slice.
pub fn getAllClassesAlloc(
    allocator: std.mem.Allocator,
) (err.Error || std.mem.Allocator.Error)![]RttiInfo {
    const needed = countClasses();
    const buffer = try allocator.alloc(RttiInfo, needed);
    errdefer allocator.free(buffer);
    var count: u32 = 0;
    try err.check(c.zjoltFactoryGetAllClasses(
        buffer.ptr,
        @intCast(buffer.len),
        &count,
    ));
    return err.filled(buffer, count);
}
