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
const character_mod = @import("character.zig");
const stream_mod = @import("stream.zig");

/// Which parts to write. `all` is what rollback wants; anything less leaves
/// part of the solver holding data from a different frame, and the step after
/// the restore is then merely close to the one that was saved.
pub const RecorderState = c.StateRecorderState;

/// Selects which bodies, constraints and contacts a save writes, or which
/// contacts a restore restores. `zjolt.StateFilter` directly, for building one
/// by hand; @see `stateFilter` to build one from a Zig type. A save with
/// `should_save_body`/`should_save_constraint`/`should_save_contact` non-null
/// is PARTIAL, carrying no body-set safety check on restore — @see `restore`.
pub const StateFilter = c.StateFilter;

fn requireAnyDecl(comptime T: type, comptime names: []const []const u8) void {
    comptime {
        var found = false;
        for (names) |name| {
            if (@hasDecl(T, name)) found = true;
        }
        if (!found) {
            var list: []const u8 = "";
            for (names) |name| list = list ++ "\n  pub fn " ++ name;
            @compileError(@typeName(T) ++ " declares none of the callbacks this state filter" ++
                " looks for, so it would be installed and never called. `@hasDecl` only" ++
                " sees `pub` declarations across files — check that yours are `pub`, and" ++
                " that they are spelled as one of:" ++ list);
        }
    }
}

/// Builds a `StateFilter` from `context` and whichever of `shouldSaveBody`,
/// `shouldSaveConstraint`, `shouldSaveContact`, `shouldRestoreContact` `T`
/// declares — an omitted one simply accepts everything for that question.
///
/// `context` must outlive the save/restore call; must not call back into the
/// physics system, and nothing may propagate out.
pub fn stateFilter(comptime T: type, context: *T) StateFilter {
    requireAnyDecl(T, &.{
        "shouldSaveBody", "shouldSaveConstraint", "shouldSaveContact", "shouldRestoreContact",
    });

    const Thunks = struct {
        fn selfOf(user: ?*anyopaque) *T {
            return @ptrCast(@alignCast(user.?));
        }
        fn saveBody(user: ?*anyopaque, body: c.BodyId) callconv(.c) bool {
            return T.shouldSaveBody(selfOf(user), body);
        }
        fn saveConstraint(user: ?*anyopaque, constraint: *c.Constraint) callconv(.c) bool {
            return T.shouldSaveConstraint(selfOf(user), constraint);
        }
        fn saveContact(user: ?*anyopaque, body1: c.BodyId, body2: c.BodyId) callconv(.c) bool {
            return T.shouldSaveContact(selfOf(user), body1, body2);
        }
        fn restoreContact(user: ?*anyopaque, body1: c.BodyId, body2: c.BodyId) callconv(.c) bool {
            return T.shouldRestoreContact(selfOf(user), body1, body2);
        }
    };

    return .{
        .should_save_body = if (@hasDecl(T, "shouldSaveBody")) Thunks.saveBody else null,
        .should_save_constraint = if (@hasDecl(T, "shouldSaveConstraint")) Thunks.saveConstraint else null,
        .should_save_contact = if (@hasDecl(T, "shouldSaveContact")) Thunks.saveContact else null,
        .should_restore_contact = if (@hasDecl(T, "shouldRestoreContact")) Thunks.restoreContact else null,
        .user = @ptrCast(context),
    };
}

/// The character controllers a whole-system save or restore covers.
///
/// `JPH::PhysicsSystem::SaveState` saves neither kind, so a save is handed
/// every character the system holds and is `error.StateIncomplete` otherwise.
/// Order is part of the payload: a restore passes the same characters in the
/// same order, or is `error.BadFormat`.
pub const Characters = struct {
    characters: []const character_mod.Character = &.{},
    rigid: []const character_mod.RigidCharacter = &.{},

    /// A `Character` is one handle and nothing else, so a slice of them
    /// already IS the array of handles the C ABI takes. Asserted rather than
    /// assumed: a second field, or a reordered one, would silently reinterpret
    /// the array as pointers.
    fn assertHandleShaped(comptime T: type, comptime Handle: type) void {
        comptime {
            std.debug.assert(@sizeOf(T) == @sizeOf(Handle));
            std.debug.assert(@alignOf(T) == @alignOf(Handle));
            std.debug.assert(std.meta.fields(T).len == 1);
            std.debug.assert(@offsetOf(T, "handle") == 0);
        }
    }

    fn toC(self: Characters) c.StateCharacters {
        assertHandleShaped(character_mod.Character, *c.Character);
        assertHandleShaped(character_mod.RigidCharacter, *c.RigidCharacter);
        return .{
            .characters = @ptrCast(self.characters.ptr),
            .count = @intCast(self.characters.len),
            .rigid_characters = @ptrCast(self.rigid.ptr),
            .rigid_count = @intCast(self.rigid.len),
        };
    }
};

/// The first payload byte at which two saves written by `State.save` or
/// `State.saveAlloc` differ.
pub const Divergence = struct {
    diverged: bool,
    /// Meaningful only when `diverged` is true.
    offset: usize,
};

/// What Jolt's `StateRecorder::SetValidating` buys a C++ host — "say exactly
/// where a restore disagrees with what was saved" — reshaped for an ABI whose
/// save/restore each build a fresh stream rather than keeping one recorder
/// alive across both, which SetValidating itself needs. Save state twice
/// (reference, then after replaying whatever should be deterministic) and
/// compare the two buffers with this.
pub fn compareState(state_a: []const u8, state_b: []const u8) err.Error!Divergence {
    var diverged: bool = false;
    var offset: usize = 0;
    try err.check(c.zjoltPhysicsSystemCompareState(
        state_a.ptr,
        state_a.len,
        state_b.ptr,
        state_b.len,
        &diverged,
        &offset,
    ));
    return .{ .diverged = diverged, .offset = offset };
}

pub const State = struct {
    handle: *c.PhysicsSystem,

    /// What a save covers.
    ///
    /// One struct rather than a plain and a `*Filtered` twin of every call:
    /// the filter is an ordinary part of what a save is, not a second kind of
    /// save, and two spellings of one operation is how the two drift apart.
    pub const SaveOptions = struct {
        /// Which categories of state to write.
        state: RecorderState = .all,
        /// Restricts the save to the bodies, constraints and contacts this
        /// accepts. A save made with a filter whose save-side questions are
        /// non-null is PARTIAL, and a partial save carries no body-set safety
        /// check on restore — getting two partial restores' body sets to
        /// overlap is then the caller's problem, exactly as it is in Jolt's
        /// own documentation for `StateRecorder::SetIsLastPart`.
        filter: ?*const StateFilter = null,
        /// Every character controller the system holds. @see `Characters`:
        /// leaving one out is `error.StateIncomplete`, not a quieter save.
        characters: Characters = .{},
    };

    /// What a restore does with what it is given.
    pub const RestoreOptions = struct {
        /// Consulted per contact through `should_restore_contact`. It need
        /// not be the filter the save used.
        filter: ?*const StateFilter = null,
        /// False for every part but the last when restoring a simulation
        /// split across several disjoint partial saves (@see
        /// `SaveOptions.filter`); true for the last one, which is what tells
        /// Jolt the whole set has arrived and it may run the bookkeeping a
        /// restore only needs to do once. An ordinary restore wants `true`.
        is_last_part: bool = true,
        /// The same characters, in the same order, the save was given.
        /// `error.BadFormat` otherwise. @see `Characters`.
        characters: Characters = .{},
    };

    /// Bytes `save` would write. Not stable across steps — a world that
    /// gained contacts needs more — so ask each time rather than caching it.
    pub fn size(self: State, options: SaveOptions) err.Error!usize {
        var needed: usize = 0;
        const characters = options.characters.toC();
        try err.check(c.zjoltPhysicsSystemSaveState(
            self.handle,
            options.state,
            options.filter,
            &characters,
            null,
            0,
            &needed,
        ));
        return needed;
    }

    /// Writes into `buffer`, returning the part that was used.
    /// `error.BufferTooSmall` if it does not fit; ask `size` first.
    ///
    /// Do not call this during a step: it reads bodies without locking them,
    /// because it is meant to run between steps where nothing is moving.
    pub fn save(self: State, buffer: []u8, options: SaveOptions) err.Error![]u8 {
        var written: usize = 0;
        const characters = options.characters.toC();
        try err.check(c.zjoltPhysicsSystemSaveState(
            self.handle,
            options.state,
            options.filter,
            &characters,
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
        options: SaveOptions,
    ) (err.Error || std.mem.Allocator.Error)![]u8 {
        const needed = try self.size(options);
        const buffer = try allocator.alloc(u8, needed);
        errdefer allocator.free(buffer);
        return try self.save(buffer, options);
    }

    /// `save`, through `stream` instead of a resident buffer — for streaming to
    /// a pack file, socket, or compressor rather than sizing and holding the
    /// whole payload first. @see `zjolt.hostStream`. No payload length or
    /// checksum ahead of Jolt's payload (both need the whole thing addressable)
    /// — but the magic tag, build identity, and body-set digest `restoreStream`
    /// checks still are. `error.IoError` if `stream` fails.
    pub fn saveStream(self: State, stream: stream_mod.Stream, options: SaveOptions) err.Error!void {
        const characters = options.characters.toC();
        try err.check(c.zjoltPhysicsSystemSaveStateStream(
            self.handle,
            options.state,
            options.filter,
            &characters,
            &stream,
        ));
    }

    /// Puts a saved state back. `error.BadFormat` for five kinds of bad input:
    /// wrong buffer, different build, truncated/trailing bytes, damaged in
    /// storage, or a payload naming bodies the world no longer holds. The first
    /// four leave the system untouched; the last does not (Jolt had already
    /// started reading) — skipped entirely for a save-side `StateFilter`
    /// (`SaveOptions.filter`).
    pub fn restore(self: State, data: []const u8, options: RestoreOptions) err.Error!void {
        const characters = options.characters.toC();
        try err.check(c.zjoltPhysicsSystemRestoreState(
            self.handle,
            data.ptr,
            data.len,
            options.filter,
            &characters,
            options.is_last_part,
        ));
    }

    /// Reads state written by `saveStream` back through `stream` instead of a
    /// resident buffer. Same `options` and body-set safety check as `restore`,
    /// but no length/checksum to validate first (a stream cannot rewind to have
    /// written them) — a truncated or corrupted stream is caught only as far as
    /// `error.IoError`/`error.BadFormat` reach.
    pub fn restoreStream(self: State, stream: stream_mod.Stream, options: RestoreOptions) err.Error!void {
        const characters = options.characters.toC();
        try err.check(c.zjoltPhysicsSystemRestoreStateStream(
            self.handle,
            &stream,
            options.filter,
            &characters,
            options.is_last_part,
        ));
    }

    //-------------------------------------------------------------------------
    // One body at a time
    //
    // For rolling back a single body (a grabbed object a networking
    // correction dropped) without touching anything else's contacts. `body` comes from a lock — `PhysicsSystem.bodies().lockRead`/`lockWrite` in body.zig.
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

    /// `saveBodyState`, through `stream` instead of a resident buffer. @see
    /// `saveStream` for what the header ahead of Jolt's payload keeps and
    /// what it does not, and `error.IoError`.
    pub fn saveBodyStateStream(self: State, body: body_mod.Body, stream: stream_mod.Stream) err.Error!void {
        try err.check(c.zjoltPhysicsSystemSaveBodyStateLockedStream(
            self.handle,
            body.handle,
            &stream,
        ));
    }

    /// Puts one body's saved state back. `body` needs a write lock, since
    /// restoring can move it between the active and sleeping lists.
    ///
    /// `error.BadFormat` also covers a state saved from a body of a different
    /// motion type (a different byte count with motion properties vs. without)
    /// — checked before Jolt reads any of it, not left to misread the rest.
    pub fn restoreBodyState(self: State, body: body_mod.Body, data: []const u8) err.Error!void {
        try err.check(c.zjoltPhysicsSystemRestoreBodyStateLocked(
            self.handle,
            body.handle,
            data.ptr,
            data.len,
        ));
    }

    /// Reads body state written by `saveBodyStateStream` back through
    /// `stream` instead of a resident buffer. @see `restoreStream` for what
    /// is and is not checked before Jolt reads a byte.
    pub fn restoreBodyStateStream(self: State, body: body_mod.Body, stream: stream_mod.Stream) err.Error!void {
        try err.check(c.zjoltPhysicsSystemRestoreBodyStateLockedStream(
            self.handle,
            body.handle,
            &stream,
        ));
    }
};
