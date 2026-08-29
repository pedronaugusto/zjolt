//! Collision groups: exceptions between individual bodies.
//!
//! Object layers already answer "do these two KINDS of thing collide". A
//! collision group answers the finer question layers cannot: "do these two
//! PARTICULAR bodies collide", where both are the same kind. The limbs of one
//! ragdoll that must not fight each other, the wheels of one vehicle.
//!
//! A body carries a group id, a sub-group id and an optional filter. Two
//! bodies collide when neither carries a filter, or when the first body's
//! filter says they do (and if only the second has one, when the second's
//! does). Filters are shared and reference counted: one per ragdoll, held by
//! every body in it.

const std = @import("std");
const c = @import("c/group.zig");
const err = @import("error.zig");

/// The group and sub-group id that mean "no group": a body carrying it
/// collides with everything.
pub const invalid: u32 = c.collision_group_invalid;

/// Largest sub-group count a filter can be created with. Not a policy: past
/// it, Jolt's `(n * (n - 1)) / 2` table sizing overflows 32 bits.
pub const max_sub_groups: u32 = c.group_filter_max_sub_groups;

/// What a body carries.
pub const CollisionGroup = struct {
    /// Null means this body imposes no exceptions of its own.
    filter: ?GroupFilter = null,
    group_id: u32 = invalid,
    sub_group_id: u32 = invalid,

    fn toC(self: CollisionGroup) c.CollisionGroup {
        return .{
            .filter = if (self.filter) |f| f.handle else null,
            .group_id = self.group_id,
            .sub_group_id = self.sub_group_id,
        };
    }

    /// The one `@constCast` in this module, and it is here on purpose.
    ///
    /// Jolt stores a body's filter as a `RefConst`, so the C getter hands
    /// back a `const` pointer. `GroupFilter` is a mutable handle since two
    /// of its methods mutate the table; mutating a filter you got out of a
    /// body is legal — it is the same object the owner of the reference holds.
    fn fromC(raw: c.CollisionGroup) CollisionGroup {
        return .{
            .filter = if (raw.filter) |handle|
                GroupFilter{ .handle = @constCast(handle) }
            else
                null,
            .group_id = raw.group_id,
            .sub_group_id = raw.sub_group_id,
        };
    }
};

/// A table of one bit per unordered pair of sub-groups, all set until
/// cleared; reference counted like a `Shape` (create, set on each body,
/// release). Rules, in order: `group_id == invalid` always collides;
/// DIFFERENT group ids always collide (within-group only suppression);
/// DIFFERENT filter objects in the same group never collide (trap: two
/// ragdolls with separate filters, same group id, pass through each other); same sub-group id never collides; else the table decides.
pub const GroupFilter = struct {
    handle: *c.GroupFilter,

    /// `num_sub_groups` sub-groups, every pair enabled. Size it to one
    /// articulated object, not to a level: the table is
    /// `(n * (n - 1)) / 2` bits.
    pub fn initTable(num_sub_groups: u32) err.Error!GroupFilter {
        var handle: *c.GroupFilter = undefined;
        try err.check(c.zjoltGroupFilterTableCreate(num_sub_groups, &handle));
        return .{ .handle = handle };
    }

    pub fn addRef(self: GroupFilter) void {
        c.zjoltGroupFilterAddRef(self.handle);
    }

    pub fn release(self: GroupFilter) void {
        c.zjoltGroupFilterRelease(self.handle);
    }

    pub fn refCount(self: GroupFilter) u32 {
        return c.zjoltGroupFilterGetRefCount(self.handle);
    }

    pub fn numSubGroups(self: GroupFilter) u32 {
        return c.zjoltGroupFilterGetNumSubGroups(self.handle);
    }

    /// Stops two sub-groups colliding.
    ///
    /// Both ids must be below `numSubGroups` and must differ —
    /// `error.InvalidArgument`, not an assertion. Equal ids are a mistake,
    /// not a no-op: a sub-group never collides with itself regardless of
    /// the table. Symmetric, so id order does not matter.
    pub fn disableCollision(self: GroupFilter, sub_group1: u32, sub_group2: u32) err.Error!void {
        try err.check(c.zjoltGroupFilterTableDisableCollision(
            self.handle,
            sub_group1,
            sub_group2,
        ));
    }

    /// Puts a pair back. Same argument rules as `disableCollision`.
    pub fn enableCollision(self: GroupFilter, sub_group1: u32, sub_group2: u32) err.Error!void {
        try err.check(c.zjoltGroupFilterTableEnableCollision(
            self.handle,
            sub_group1,
            sub_group2,
        ));
    }

    pub fn isCollisionEnabled(
        self: GroupFilter,
        sub_group1: u32,
        sub_group2: u32,
    ) err.Error!bool {
        var enabled: bool = false;
        try err.check(c.zjoltGroupFilterTableIsCollisionEnabled(
            self.handle,
            sub_group1,
            sub_group2,
            &enabled,
        ));
        return enabled;
    }
};

/// Used by `BodyInterface.setCollisionGroup`; here rather than there because
/// the conversion belongs with the type it converts.
pub fn toC(group: CollisionGroup) c.CollisionGroup {
    return group.toC();
}

/// Used by `BodyInterface.getCollisionGroup`.
pub fn fromC(raw: c.CollisionGroup) CollisionGroup {
    return CollisionGroup.fromC(raw);
}
