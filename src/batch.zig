//! Many bodies at once.
//!
//! Adding ten thousand bodies one at a time produces a broad phase that is
//! correct and badly shaped, and every query pays for that until a later
//! step rebuilds the tree. `add` sorts the whole set by broad-phase layer
//! and builds the subtree in one pass instead — Jolt's own guidance is
//! that a batch added into roughly unoccupied space needs no
//! `optimizeBroadPhase` at all.
//!
//! The slices these take are read once and released. Jolt's own batch
//! calls shuffle the array they are given and, for the two-phase add, hold
//! pointers into it until finalize; the C layer copies into storage it
//! owns, so none of that reaches a caller here — a `[]const BodyId` may be a temporary. Reached from `PhysicsSystem.batch()`.

const std = @import("std");
const c = @import("c/batch.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const broadphase_mod = @import("broadphase.zig");

/// A batch sorted and staged by `Batch.prepare`, waiting for `finalize` or
/// `abort`.
///
/// Owned by the system it was prepared on: one still outstanding when the
/// system is destroyed is aborted with it, so forgetting to consume it
/// leaks nothing. Bodies are neither in the simulation nor findable by a query until consumed.
pub const AddBatch = struct {
    handle: *c.BodyAddBatch,
};

pub const Batch = struct {
    handle: *c.PhysicsSystem,

    //=========================================================================
    // Insert
    //=========================================================================

    /// Creates and adds in one call. This is the one to reach for.
    ///
    /// Every id must name a live body that is not already added, and no id may
    /// appear twice; the whole batch is refused rather than half applied,
    /// because half a level inserted into the broad phase is worse than none.
    pub fn add(
        self: Batch,
        bodies: []const body_mod.BodyId,
        activation: body_mod.Activation,
    ) err.Error!void {
        try err.check(c.zjoltBodyAddBatch(
            self.handle,
            bodies.ptr,
            @intCast(bodies.len),
            activation,
        ));
    }

    /// Sorts and stages a batch without touching the system's own state.
    ///
    /// This is the half that can run on a worker thread while the simulation
    /// steps; `finalize` is the half that cannot.
    pub fn prepare(self: Batch, bodies: []const body_mod.BodyId) err.Error!AddBatch {
        var handle: *c.BodyAddBatch = undefined;
        try err.check(c.zjoltBodyAddBatchPrepare(
            self.handle,
            bodies.ptr,
            @intCast(bodies.len),
            &handle,
        ));
        return .{ .handle = handle };
    }

    /// Inserts the staged batch and consumes the handle.
    pub fn finalize(
        self: Batch,
        staged: AddBatch,
        activation: body_mod.Activation,
    ) err.Error!void {
        try err.check(c.zjoltBodyAddBatchFinalize(self.handle, staged.handle, activation));
    }

    /// Throws the staged batch away, leaving the bodies created but not added
    /// — exactly where they were before the `prepare`.
    pub fn abort(self: Batch, staged: AddBatch) err.Error!void {
        try err.check(c.zjoltBodyAddBatchAbort(self.handle, staged.handle));
    }

    //=========================================================================
    // Remove and destroy
    //=========================================================================

    /// Removes bodies from the simulation, leaving them created. Every id must
    /// name a live body that is currently added, or the whole batch is
    /// refused.
    pub fn remove(self: Batch, bodies: []const body_mod.BodyId) err.Error!void {
        try err.check(c.zjoltBodyRemoveBatch(
            self.handle,
            bodies.ptr,
            @intCast(bodies.len),
        ));
    }

    /// Destroys bodies, removing any still added first.
    ///
    /// Unlike `remove`, an id that no longer names a body is skipped rather
    /// than refused: a list built last frame outliving one of its bodies is
    /// ordinary, and refusing the set over it would leave the rest alive.
    pub fn destroy(self: Batch, bodies: []const body_mod.BodyId) err.Error!void {
        try err.check(c.zjoltBodyDestroyBatch(
            self.handle,
            bodies.ptr,
            @intCast(bodies.len),
        ));
    }

    /// Bulk form of `BodyInterface.unassignId`. The returned slice has one
    /// entry per entry in `bodies`, in the same order, allocated with
    /// `allocator` and owned by the caller; an id that does not currently
    /// name a live body gets `null` instead of failing the whole batch, the
    /// same reasoning `destroy` uses.
    pub fn unassignIds(
        self: Batch,
        allocator: std.mem.Allocator,
        bodies: []const body_mod.BodyId,
    ) err.Error![]?body_mod.UnassignedBody {
        const raw = try allocator.alloc(?*c.UnassignedBody, bodies.len);
        defer allocator.free(raw);

        try err.check(c.zjoltBodyUnassignIds(
            self.handle,
            bodies.ptr,
            @intCast(bodies.len),
            raw.ptr,
        ));

        const out = try allocator.alloc(?body_mod.UnassignedBody, bodies.len);
        for (raw, out) |maybe_handle, *slot| {
            slot.* = if (maybe_handle) |handle|
                .{ .handle = handle, .owner = self.handle }
            else
                null;
        }
        return out;
    }

    //=========================================================================
    // Activate
    //=========================================================================

    /// One wake-up for the whole set rather than one per body. Ids that no
    /// longer name an added body are skipped.
    pub fn activate(self: Batch, bodies: []const body_mod.BodyId) err.Error!void {
        try err.check(c.zjoltBodyActivateBatch(
            self.handle,
            bodies.ptr,
            @intCast(bodies.len),
        ));
    }

    pub fn deactivate(self: Batch, bodies: []const body_mod.BodyId) err.Error!void {
        try err.check(c.zjoltBodyDeactivateBatch(
            self.handle,
            bodies.ptr,
            @intCast(bodies.len),
        ));
    }

    /// Wakes every body whose bounding box overlaps `box` — a broad-phase
    /// test, with the same caveats as `BroadPhase.collideBox`, because it is
    /// the same query.
    pub fn activateInBox(
        self: Batch,
        box: math.AABox,
        filters: ?*const broadphase_mod.Filters,
    ) err.Error!void {
        try err.check(c.zjoltBodyActivateInBox(self.handle, &box, filters));
    }
};
