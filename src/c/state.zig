//! ZJolt C declarations for saving and restoring the state a step changes.
//!
//! Mirrors `ffi/zjolt_state.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Body = core.Body;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Result = core.Result;

/// Which parts of a simulation `zjoltPhysicsSystemSaveState` writes.
///
/// A packed struct for the same reason as `UpdateError`: the C header spells
/// it as an enum of bits only because C has no better way to name them.
pub const StateRecorderState = packed struct(u32) {
    global: bool = false,
    bodies: bool = false,
    contacts: bool = false,
    constraints: bool = false,
    _reserved: u28 = 0,

    pub const none: StateRecorderState = .{};

    /// What rollback wants. Anything less leaves part of the solver holding
    /// data from a different frame.
    pub const all: StateRecorderState = .{
        .global = true,
        .bodies = true,
        .contacts = true,
        .constraints = true,
    };
};

pub extern fn zjoltPhysicsSystemSaveState(system: *const PhysicsSystem, state: StateRecorderState, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;

pub extern fn zjoltPhysicsSystemRestoreState(system: *PhysicsSystem, data: [*]const u8, size: usize) Result;

pub extern fn zjoltPhysicsSystemSaveBodyStateLocked(system: *const PhysicsSystem, body: *const Body, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;

pub extern fn zjoltPhysicsSystemRestoreBodyStateLocked(system: *PhysicsSystem, body: *Body, data: [*]const u8, size: usize) Result;
