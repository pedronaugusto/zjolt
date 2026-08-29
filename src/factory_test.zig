//! Behavioural tests for the Factory subset in core.zig: `find`, `findByHash`
//! and `getAllClasses` agree with each other, and a name nothing registered
//! reports absent rather than some plausible-looking garbage.

const std = @import("std");
const zjolt = @import("zjolt.zig");
const core = @import("factory.zig");

test "getAllClasses, find and findByHash agree about every registered type" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    const classes = try core.getAllClassesAlloc(std.testing.allocator);
    defer std.testing.allocator.free(classes);

    // zjoltInit registers every type Jolt itself defines, so this is never
    // empty once the library is up.
    try std.testing.expect(classes.len > 0);

    for (classes) |class| {
        const name = class.name orelse return error.TestUnexpectedResult;
        const by_name = core.find(std.mem.span(name)) orelse return error.TestUnexpectedResult;
        try std.testing.expectEqual(class.hash, by_name.hash);
        try std.testing.expectEqual(class.size, by_name.size);
        try std.testing.expectEqual(class.is_abstract, by_name.is_abstract);

        const by_hash = core.findByHash(class.hash) orelse return error.TestUnexpectedResult;
        try std.testing.expectEqualStrings(std.mem.span(name), std.mem.span(by_hash.name.?));
    }
}

test "find reports absent for a name nothing registered, not a plausible guess" {
    try zjolt.init(.{ .allocator = std.testing.allocator });
    defer zjolt.deinit();

    try std.testing.expect(core.find("NotARealJoltClassName") == null);
    try std.testing.expect(core.findByHash(0xdeadbeef) == null);
}

test "find answers nothing before init rather than reading an uninitialised factory" {
    try std.testing.expect(core.find("SphereShape") == null);
    try std.testing.expectEqual(@as(usize, 0), core.countClasses());
}
