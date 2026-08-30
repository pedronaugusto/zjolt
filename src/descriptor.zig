//! Crossing a Zig options struct to the C descriptor it mirrors, by field
//! NAME rather than by a hand-kept list of assignments.
//!
//! A hand-kept list is the shape of defect this module exists to remove: a
//! field added to the C descriptor and not to the list is not a compile
//! error, it is a setting that silently keeps Jolt's default and that no
//! caller can reach. That happened to six `BodyCreationSettings` fields at
//! once and to two `CharacterVirtualSettings` fields after them.

const std = @import("std");

/// Copies every field of `from` into `out` by name. `skip` names the fields
/// whose TYPE differs across the ABI — a handle behind a wrapper struct, an
/// enum crossed through a converter — which the caller assigns itself.
///
/// Not a memcpy: the two structs may order and pad their fields differently,
/// and only the names are the contract.
pub fn crossByName(out: anytype, from: anytype, comptime skip: []const []const u8) void {
    const From = @TypeOf(from);
    inline for (@typeInfo(From).@"struct".fields) |field| {
        if (comptime contains(skip, field.name)) continue;
        @field(out, field.name) = @field(from, field.name);
    }
}

/// Every field of the C descriptor `C` is modelled by `Zig`, or a compile
/// error names the one that is not. The direction `crossByName` cannot
/// check on its own: a field only C has is never read, so nothing else
/// would notice it.
pub fn requireModelled(comptime Zig: type, comptime C: type) void {
    comptime {
        for (@typeInfo(C).@"struct".fields) |field| {
            if (!@hasField(Zig, field.name)) {
                @compileError(@typeName(Zig) ++ " does not model " ++
                    @typeName(C) ++ "." ++ field.name);
            }
        }
    }
}

fn contains(comptime haystack: []const []const u8, comptime needle: []const u8) bool {
    for (haystack) |entry| {
        if (std.mem.eql(u8, entry, needle)) return true;
    }
    return false;
}
