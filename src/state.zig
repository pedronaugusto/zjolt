//! Saving and restoring the state of a simulation.
//!
//! The state a step CHANGES, as a flat buffer: for rollback in a networked
//! game, for a replay, or for a determinism check. Positions, rotations,
//! velocities, sleep state, and the solver's contact cache.
//!
//! What it deliberately does not carry is everything the simulation only
//! reads: shapes, layers, motion types, masses, friction, collision groups,
//! `PhysicsSettings`. A restore expects to find the same bodies, in the same
//! system, configured the same way, and puts their motion back. It is a
//! snapshot of where the world is, not of what the world is — and a body
//! created or destroyed since the save is refused rather than guessed at.
//!
//! Reached from `PhysicsSystem.state()`.

const std = @import("std");
const c = @import("c/state.zig");
const err = @import("error.zig");
const body_mod = @import("body.zig");

/// Which parts to write. `all` is what rollback wants; anything less leaves
/// part of the solver holding data from a different frame, and the step after
/// the restore is then merely close to the one that was saved.
pub const RecorderState = c.StateRecorderState;

pub const State = struct {
    handle: *c.PhysicsSystem,

    /// Bytes `save` would write. Not stable across steps — a world that
    /// gained contacts needs more — so ask each time rather than caching it.
    pub fn size(self: State, state: RecorderState) err.Error!usize {
        var needed: usize = 0;
        try err.check(c.zjoltPhysicsSystemSaveState(self.handle, state, null, 0, &needed));
        return needed;
    }

    /// Writes into `buffer`, returning the part that was used.
    /// `error.BufferTooSmall` if it does not fit; ask `size` first.
    ///
    /// Do not call this during a step: it reads bodies without locking them,
    /// because it is meant to run between steps where nothing is moving.
    pub fn save(self: State, state: RecorderState, buffer: []u8) err.Error![]u8 {
        var written: usize = 0;
        try err.check(c.zjoltPhysicsSystemSaveState(
            self.handle,
            state,
            buffer.ptr,
            buffer.len,
            &written,
        ));
        return buffer[0..written];
    }

    /// `save` into memory from `allocator`. The caller owns the slice.
    pub fn saveAlloc(
        self: State,
        allocator: std.mem.Allocator,
        state: RecorderState,
    ) (err.Error || std.mem.Allocator.Error)![]u8 {
        const needed = try self.size(state);
        const buffer = try allocator.alloc(u8, needed);
        errdefer allocator.free(buffer);
        return try self.save(state, buffer);
    }

    /// Puts a saved state back.
    ///
    /// `error.BadFormat` covers five kinds of wrong input: not a zjolt state
    /// buffer, written by a different build, truncated or with trailing bytes,
    /// damaged in storage, and a payload Jolt refused because the world no
    /// longer holds the bodies it names. The first four leave the system
    /// untouched; the last does not, because Jolt had already started reading.
    pub fn restore(self: State, data: []const u8) err.Error!void {
        try err.check(c.zjoltPhysicsSystemRestoreState(self.handle, data.ptr, data.len));
    }

    //-------------------------------------------------------------------------
    // One body at a time
    //
    // For rolling back a single body — a grabbed object a networking
    // correction dropped — without touching anything else's contacts.
    // `body` comes from a lock: see `PhysicsSystem.bodies().lockRead` /
    // `lockWrite` in body.zig.
    //-------------------------------------------------------------------------

    /// Bytes `saveBodyState` would write for `body`. Not stable across steps;
    /// ask each time rather than caching it.
    pub fn bodyStateSize(self: State, body: body_mod.Body) err.Error!usize {
        var needed: usize = 0;
        try err.check(c.zjoltPhysicsSystemSaveBodyStateLocked(
            self.handle,
            body.handle,
            null,
            0,
            &needed,
        ));
        return needed;
    }

    /// Writes `body`'s state into `buffer`, returning the part that was used.
    /// `error.BufferTooSmall` if it does not fit; ask `bodyStateSize` first.
    ///
    /// Do not call this during a step, for the same reason as `save`.
    pub fn saveBodyState(self: State, body: body_mod.Body, buffer: []u8) err.Error![]u8 {
        var written: usize = 0;
        try err.check(c.zjoltPhysicsSystemSaveBodyStateLocked(
            self.handle,
            body.handle,
            buffer.ptr,
            buffer.len,
            &written,
        ));
        return buffer[0..written];
    }

    /// `saveBodyState` into memory from `allocator`. The caller owns the
    /// slice.
    pub fn saveBodyStateAlloc(
        self: State,
        allocator: std.mem.Allocator,
        body: body_mod.Body,
    ) (err.Error || std.mem.Allocator.Error)![]u8 {
        const needed = try self.bodyStateSize(body);
        const buffer = try allocator.alloc(u8, needed);
        errdefer allocator.free(buffer);
        return try self.saveBodyState(body, buffer);
    }

    /// Puts one body's saved state back. `body` needs a write lock, since
    /// restoring can move it between the active and sleeping lists.
    ///
    /// `error.BadFormat` also covers a state saved from a body of a different
    /// motion type: Jolt reads a different number of bytes for a body with
    /// motion properties than for one without, so this is checked before
    /// Jolt reads any of them rather than left to misread the rest.
    pub fn restoreBodyState(self: State, body: body_mod.Body, data: []const u8) err.Error!void {
        try err.check(c.zjoltPhysicsSystemRestoreBodyStateLocked(
            self.handle,
            body.handle,
            data.ptr,
            data.len,
        ));
    }
};
