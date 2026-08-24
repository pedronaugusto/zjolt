//! ZJolt C declarations for a whole world held as data, saved and loaded.
//!
//! Mirrors `ffi/zjolt_scene.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const body = @import("body.zig");
const constraint = @import("constraint.zig");
const core = @import("core.zig");
const softbody = @import("softbody.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const BodyDesc = body.BodyDesc;
pub const Constraint = constraint.Constraint;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Result = core.Result;
pub const SoftBodyDesc = softbody.SoftBodyDesc;

pub const Scene = opaque {};

/// The body index that means "the world", for a constraint end and for the
/// index a failed `zjoltSceneAdd*` leaves behind.
pub const scene_body_world: u32 = 0xffff_ffff;

pub const SceneConstraint = extern struct {
    body1: u32,
    body2: u32,
    user_data: u64,
    priority: u32,
    num_velocity_steps_override: u32,
    num_position_steps_override: u32,
    enabled: bool,
};

pub extern fn zjoltSceneCreate(out: **Scene) Result;

pub extern fn zjoltSceneAddRef(scene: *const Scene) void;

pub extern fn zjoltSceneRelease(scene: *const Scene) void;

pub extern fn zjoltSceneGetRefCount(scene: *const Scene) u32;

pub extern fn zjoltSceneAddBody(scene: *Scene, desc: *const BodyDesc, out_index: *u32) Result;

pub extern fn zjoltSceneAddSoftBody(scene: *Scene, desc: *const SoftBodyDesc, out_index: *u32) Result;

pub extern fn zjoltSceneAddConstraint(scene: *Scene, constraint: *const Constraint, body1: u32, body2: u32) Result;

pub extern fn zjoltSceneFromPhysicsSystem(scene: *Scene, system: *const PhysicsSystem) Result;

pub extern fn zjoltSceneGetNumBodies(scene: *const Scene) u32;

pub extern fn zjoltSceneGetNumSoftBodies(scene: *const Scene) u32;

pub extern fn zjoltSceneGetNumConstraints(scene: *const Scene) u32;

pub extern fn zjoltSceneGetBody(scene: *const Scene, index: u32, out: *BodyDesc) Result;

pub extern fn zjoltSceneGetSoftBody(scene: *const Scene, index: u32, out: *SoftBodyDesc) Result;

pub extern fn zjoltSceneGetConstraint(scene: *const Scene, index: u32, out: *SceneConstraint) Result;

pub extern fn zjoltSceneCreateBodies(scene: *const Scene, system: *PhysicsSystem) Result;

pub extern fn zjoltSceneFixInvalidScales(scene: *Scene) Result;

pub extern fn zjoltSceneSave(scene: *const Scene, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;

pub extern fn zjoltSceneRestore(data: [*]const u8, size: usize, out: **Scene) Result;
