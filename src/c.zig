//! The C declaration modules, and the list the guards walk.
//!
//! This file used to hold every declaration in the ABI. It was the one file
//! every subsystem appended to, so it was the only file parallel work ever
//! collided on, and at 2,389 lines it was no longer readable as anything.
//!
//! The split follows `ffi/`: one module per public header, named after it.
//! That rule is mechanical, so a new declaration has exactly one home and
//! nobody has to argue about it.
//!
//! `modules` is the point of keeping this file at all. `abi_check.zig` and
//! `misuse_sweep_test.zig` discover what to check by walking it, so a module
//! added here is swept without either of them being told — and a module NOT
//! added here is a module neither guard covers, which is why adding one is
//! the same edit as declaring it.

pub const batch = @import("c/batch.zig");
pub const body = @import("c/body.zig");
pub const broadphase = @import("c/broadphase.zig");
pub const character = @import("c/character.zig");
pub const constraint = @import("c/constraint.zig");
pub const core = @import("c/core.zig");
pub const debug = @import("c/debug.zig");
pub const group = @import("c/group.zig");
pub const hair = @import("c/hair.zig");
pub const material = @import("c/material.zig");
pub const query = @import("c/query.zig");
pub const ragdoll = @import("c/ragdoll.zig");
pub const scene = @import("c/scene.zig");
pub const shape = @import("c/shape.zig");
pub const softbody = @import("c/softbody.zig");
pub const state = @import("c/state.zig");
pub const system = @import("c/system.zig");
pub const transformed = @import("c/transformed.zig");
pub const vehicle = @import("c/vehicle.zig");

/// Every module above. Walked by `abi_check.zig` and `misuse_sweep_test.zig`.
pub const modules = .{
    batch,
    body,
    broadphase,
    character,
    constraint,
    core,
    debug,
    group,
    hair,
    material,
    query,
    ragdoll,
    scene,
    shape,
    softbody,
    state,
    system,
    transformed,
    vehicle,
};
