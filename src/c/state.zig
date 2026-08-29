//! ZJolt C declarations for saving and restoring the state a step changes.
//!
//! Mirrors `ffi/zjolt_state.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");
const constraint = @import("constraint.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Body = core.Body;
pub const BodyId = core.BodyId;
pub const Constraint = constraint.Constraint;
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

/// Selects which bodies, constraints and contacts a save writes, or which
/// contacts a restore restores. NULL (the default in every field) accepts
/// everything, exactly as if no filter were supplied at all.
pub const StateFilter = extern struct {
    should_save_body: ?*const fn (user: ?*anyopaque, body: BodyId) callconv(.c) bool = null,
    should_save_constraint: ?*const fn (user: ?*anyopaque, constraint_handle: *Constraint) callconv(.c) bool = null,
    should_save_contact: ?*const fn (user: ?*anyopaque, body1: BodyId, body2: BodyId) callconv(.c) bool = null,
    should_restore_contact: ?*const fn (user: ?*anyopaque, body1: BodyId, body2: BodyId) callconv(.c) bool = null,
    user: ?*anyopaque = null,
};

pub extern fn zjoltPhysicsSystemSaveState(system: *const PhysicsSystem, state: StateRecorderState, filter: ?*const StateFilter, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;

pub extern fn zjoltPhysicsSystemSaveStateStream(system: *const PhysicsSystem, state: StateRecorderState, filter: ?*const StateFilter, stream: *const core.Stream) Result;

pub extern fn zjoltPhysicsSystemRestoreState(system: *PhysicsSystem, data: [*]const u8, size: usize, filter: ?*const StateFilter, is_last_part: bool) Result;

pub extern fn zjoltPhysicsSystemRestoreStateStream(system: *PhysicsSystem, stream: *const core.Stream, filter: ?*const StateFilter, is_last_part: bool) Result;

pub extern fn zjoltPhysicsSystemCompareState(state_a: [*]const u8, size_a: usize, state_b: [*]const u8, size_b: usize, out_diverged: *bool, out_offset: *usize) Result;

pub extern fn zjoltPhysicsSystemSaveBodyStateLocked(system: *const PhysicsSystem, body: *const Body, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;

pub extern fn zjoltPhysicsSystemSaveBodyStateLockedStream(system: *const PhysicsSystem, body: *const Body, stream: *const core.Stream) Result;

pub extern fn zjoltPhysicsSystemRestoreBodyStateLocked(system: *PhysicsSystem, body: *Body, data: [*]const u8, size: usize) Result;

pub extern fn zjoltPhysicsSystemRestoreBodyStateLockedStream(system: *PhysicsSystem, body: *Body, stream: *const core.Stream) Result;
