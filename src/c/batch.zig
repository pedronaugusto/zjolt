//! ZJolt C declarations for batched body add and remove.
//!
//! Mirrors `ffi/zjolt_batch.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const broadphase = @import("broadphase.zig");
const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const BroadPhaseFilters = broadphase.BroadPhaseFilters;
pub const AABox = core.AABox;
pub const Activation = core.Activation;
pub const BodyAddBatch = core.BodyAddBatch;
pub const BodyId = core.BodyId;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Result = core.Result;

pub extern fn zjoltBodyAddBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32, activation: Activation) Result;

pub extern fn zjoltBodyAddBatchPrepare(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32, out: **BodyAddBatch) Result;

pub extern fn zjoltBodyAddBatchFinalize(system: *PhysicsSystem, batch: *BodyAddBatch, activation: Activation) Result;

pub extern fn zjoltBodyAddBatchAbort(system: *PhysicsSystem, batch: *BodyAddBatch) Result;

pub extern fn zjoltBodyRemoveBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32) Result;

pub extern fn zjoltBodyDestroyBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32) Result;

pub extern fn zjoltBodyActivateBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32) Result;

pub extern fn zjoltBodyDeactivateBatch(system: *PhysicsSystem, bodies: ?[*]const BodyId, count: u32) Result;

pub extern fn zjoltBodyActivateInBox(system: *PhysicsSystem, box: *const AABox, filters: ?*const BroadPhaseFilters) Result;
