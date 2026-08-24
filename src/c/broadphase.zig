//! ZJolt C declarations for broad-phase layer queries.
//!
//! Mirrors `ffi/zjolt_broadphase.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");
const query = @import("query.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const AABox = core.AABox;
pub const BodyId = core.BodyId;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Vec3 = core.Vec3;
pub const BroadPhaseLayerFilter = query.BroadPhaseLayerFilter;
pub const ObjectLayerFilter = query.ObjectLayerFilter;

pub const BroadPhaseFilters = extern struct {
    broad_phase_layer: BroadPhaseLayerFilter = .{},
    object_layer: ObjectLayerFilter = .{},
};

pub const BroadPhaseCastHit = extern struct {
    body: BodyId,
    fraction: f32,
};

pub const OrientedBox = extern struct {
    center: RVec3,
    rotation: Quat,
    half_extent: Vec3,
};

pub extern fn zjoltBroadPhaseCastRay(system: *const PhysicsSystem, origin: *const RVec3, direction: *const Vec3, filters: ?*const BroadPhaseFilters, out_hits: ?[*]BroadPhaseCastHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltBroadPhaseCollideAABox(system: *const PhysicsSystem, box: *const AABox, filters: ?*const BroadPhaseFilters, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltBroadPhaseCollideSphere(system: *const PhysicsSystem, center: *const RVec3, radius: f32, filters: ?*const BroadPhaseFilters, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltBroadPhaseCollidePoint(system: *const PhysicsSystem, point: *const RVec3, filters: ?*const BroadPhaseFilters, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltBroadPhaseCollideOrientedBox(system: *const PhysicsSystem, box: *const OrientedBox, filters: ?*const BroadPhaseFilters, out_ids: ?[*]BodyId, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltBroadPhaseCastAABox(system: *const PhysicsSystem, box: *const AABox, direction: *const Vec3, filters: ?*const BroadPhaseFilters, out_hits: ?[*]BroadPhaseCastHit, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltBroadPhaseGetBounds(system: *const PhysicsSystem, out: *AABox) void;
