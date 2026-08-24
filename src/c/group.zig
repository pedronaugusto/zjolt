//! ZJolt C declarations for collision group filters.
//!
//! Mirrors `ffi/zjolt_group.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const BodyId = core.BodyId;
pub const GroupFilter = core.GroupFilter;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Result = core.Result;

/// The group and sub-group id that mean "no group": a body carrying it
/// collides with everything.
pub const collision_group_invalid: u32 = 0xffff_ffff;

/// Past this, Jolt's `(n * (n - 1)) / 2` table sizing overflows 32 bits.
pub const group_filter_max_sub_groups: u32 = 65535;

pub const CollisionGroup = extern struct {
    filter: ?*const GroupFilter = null,
    group_id: u32 = collision_group_invalid,
    sub_group_id: u32 = collision_group_invalid,
};

pub extern fn zjoltGroupFilterTableCreate(num_sub_groups: u32, out: **GroupFilter) Result;

pub extern fn zjoltGroupFilterAddRef(filter: *const GroupFilter) void;

pub extern fn zjoltGroupFilterRelease(filter: *const GroupFilter) void;

pub extern fn zjoltGroupFilterGetRefCount(filter: *const GroupFilter) u32;

pub extern fn zjoltGroupFilterGetNumSubGroups(filter: *const GroupFilter) u32;

pub extern fn zjoltGroupFilterTableDisableCollision(filter: *GroupFilter, sub_group1: u32, sub_group2: u32) Result;

pub extern fn zjoltGroupFilterTableEnableCollision(filter: *GroupFilter, sub_group1: u32, sub_group2: u32) Result;

pub extern fn zjoltGroupFilterTableIsCollisionEnabled(filter: *const GroupFilter, sub_group1: u32, sub_group2: u32, out_enabled: *bool) Result;

pub extern fn zjoltBodySetCollisionGroup(system: *PhysicsSystem, body: BodyId, group: ?*const CollisionGroup) void;

pub extern fn zjoltBodyGetCollisionGroup(system: *const PhysicsSystem, body: BodyId, out: *CollisionGroup) void;

pub extern fn zjoltBodyInvalidateContactCache(system: *PhysicsSystem, body: BodyId) void;
