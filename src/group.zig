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
const c = @import("c.zig");
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
    /// Jolt stores a body's filter as a `RefConst`, so the C getter hands back
    /// a `const` pointer. `GroupFilter` is a mutable handle because two of its
    /// methods mutate the table, and casting once here is better than casting
    /// at both of those. Mutating a filter you got out of a body is legal —
    /// it is the same object the owner of the reference holds.
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

/// A table of one bit per unordered pair of sub-groups, all set — everything
/// collides — until you clear one.
///
/// Reference counted like a `Shape`: `initTable` hands back one reference and
/// a body takes its own when the group is set on it, so the usual pattern is
/// create the filter, set it on every body in the object, release.
///
/// The rules the table applies are worth reading in full, because two of them
/// are easy to trip:
///
///   * a body whose `group_id` is `invalid` collides with everything;
///   * bodies with DIFFERENT group ids always collide — the table only ever
///     suppresses within one group;
///   * bodies in the same group carrying DIFFERENT filter objects never
///     collide, which catches people out: two ragdolls with a filter each pass
///     through one another if you gave them the same group id;
///   * bodies in the same group with the same sub-group id never collide;
///   * otherwise the table's bit for the pair decides.
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
    /// Both ids must be below `numSubGroups` and must differ; both are
    /// `error.InvalidArgument` rather than an assertion, and equal ids are a
    /// mistake rather than a no-op because a sub-group never collides with
    /// itself whatever the table says. The table is symmetric, so the order of
    /// the two ids does not matter.
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
