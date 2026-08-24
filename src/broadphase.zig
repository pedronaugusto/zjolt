//! Broad-phase queries: which bodies are roughly there.
//!
//! These return body ids and nothing else. The broad phase tests only each
//! body's bounding box, so it answers "which bodies could possibly be
//! involved" and never "where did they touch" — there is no contact point, no
//! normal and no penetration depth to be had here. `Queries` in `query.zig` is
//! what runs the narrow phase and produces those.
//!
//! Reach for these when the next thing you do is your own test anyway: a blast
//! radius that computes its own falloff, an editor selection box, a streamer
//! asking what is in a region. A hit means the AXIS-ALIGNED bounding box
//! overlaps, which for a long thin body at forty-five degrees is a box several
//! times its own volume.
//!
//! Positions here are `RVec3` for consistency with the rest of the package and
//! are narrowed to float on the way in, because Jolt's broad phase is single
//! precision in every build and Jolt's own narrow-phase entry points narrow
//! the same way before consulting it.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");

/// The two filters a broad-phase query can consult.
///
/// Deliberately not `QueryFilters`: that carries a body filter as well, and
/// the broad phase has nowhere to put one. A null `Filters` accepts
/// everything, and so does a member left unset.
pub const Filters = c.BroadPhaseFilters;

/// A body whose bounding box a ray or a swept box entered, and how far along.
pub const CastHit = c.BroadPhaseCastHit;

/// A box that is not axis aligned, for `collideOrientedBox`.
pub const OrientedBox = c.OrientedBox;

/// Reached from `PhysicsSystem.broadPhase()`.
pub const BroadPhase = struct {
    handle: *const c.PhysicsSystem,

    //=========================================================================
    // Casts
    //
    // Both use the two-call protocol the rest of the package uses: ask for the
    // count, then fill a buffer. Results are unsorted — Jolt's broad-phase
    // collectors append in tree-traversal order, and sorting a candidate list
    // the caller is about to filter anyway would be work done twice.
    //=========================================================================

    pub fn countRayHits(
        self: BroadPhase,
        origin: math.RVec3,
        direction: math.Vec3,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltBroadPhaseCastRay(
            self.handle,
            &origin,
            &direction,
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Bodies whose bounding box the ray enters. `direction` carries the
    /// ray's length; nothing beyond it is reported.
    pub fn castRay(
        self: BroadPhase,
        origin: math.RVec3,
        direction: math.Vec3,
        filters: ?*const Filters,
        buffer: []CastHit,
    ) err.Error![]CastHit {
        var count: u32 = 0;
        try err.check(c.zjoltBroadPhaseCastRay(
            self.handle,
            &origin,
            &direction,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// Bodies whose bounding box the box sweeps through. The box stays axis
    /// aligned for the whole sweep — this is a moving AABB, not a shape cast.
    pub fn castBox(
        self: BroadPhase,
        box: math.AABox,
        direction: math.Vec3,
        filters: ?*const Filters,
        buffer: []CastHit,
    ) err.Error![]CastHit {
        var count: u32 = 0;
        try err.check(c.zjoltBroadPhaseCastAABox(
            self.handle,
            &box,
            &direction,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    //=========================================================================
    // Overlaps
    //=========================================================================

    /// Bodies whose bounding box overlaps `box`. World space, and float in
    /// every build.
    pub fn collideBox(
        self: BroadPhase,
        box: math.AABox,
        filters: ?*const Filters,
        buffer: []body_mod.BodyId,
    ) err.Error![]body_mod.BodyId {
        var count: u32 = 0;
        try err.check(c.zjoltBroadPhaseCollideAABox(
            self.handle,
            &box,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    pub fn countBoxOverlaps(
        self: BroadPhase,
        box: math.AABox,
        filters: ?*const Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltBroadPhaseCollideAABox(
            self.handle,
            &box,
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Bodies whose bounding box the sphere clips.
    ///
    /// Not a sphere overlap test: it is the boxes that are tested, so a body
    /// whose box the sphere touches is reported even when the body itself is
    /// nowhere near it.
    pub fn collideSphere(
        self: BroadPhase,
        center: math.RVec3,
        radius: f32,
        filters: ?*const Filters,
        buffer: []body_mod.BodyId,
    ) err.Error![]body_mod.BodyId {
        var count: u32 = 0;
        try err.check(c.zjoltBroadPhaseCollideSphere(
            self.handle,
            &center,
            radius,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// Bodies whose bounding box contains the point.
    pub fn collidePoint(
        self: BroadPhase,
        point: math.RVec3,
        filters: ?*const Filters,
        buffer: []body_mod.BodyId,
    ) err.Error![]body_mod.BodyId {
        var count: u32 = 0;
        try err.check(c.zjoltBroadPhaseCollidePoint(
            self.handle,
            &point,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// More precise than passing the oriented box's own axis-aligned bounds to
    /// `collideBox`, and more expensive: each candidate gets a
    /// separating-axis test instead of an interval overlap.
    pub fn collideOrientedBox(
        self: BroadPhase,
        box: OrientedBox,
        filters: ?*const Filters,
        buffer: []body_mod.BodyId,
    ) err.Error![]body_mod.BodyId {
        var count: u32 = 0;
        try err.check(c.zjoltBroadPhaseCollideOrientedBox(
            self.handle,
            &box,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return buffer[0..count];
    }

    /// Bounding box of every body in the broad phase.
    ///
    /// An EMPTY world does not report an empty box: Jolt returns its
    /// inside-out initial box, whose min is `+FLT_MAX` and whose max is
    /// `-FLT_MAX`. Check `numBodies` before reading this as a world extent.
    pub fn bounds(self: BroadPhase) math.AABox {
        var out: math.AABox = undefined;
        c.zjoltBroadPhaseGetBounds(self.handle, &out);
        return out;
    }
};
