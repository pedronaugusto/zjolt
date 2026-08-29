//! The C declaration modules, and the list the guards walk.
//!
//! The split follows `ffi/`: one module per public header, named after it.
//! That rule is mechanical, so a new declaration has exactly one home and
//! nobody has to argue about it.
//!
//! `modules` is the point of keeping this file at all: `abi_check.zig` and
//! `misuse_sweep_test.zig` discover what to check by walking it, so a module
//! added here is swept automatically, and a module not added here is a
//! module neither guard covers.

pub const batch = @import("c/batch.zig");
pub const body = @import("c/body.zig");
pub const broadphase = @import("c/broadphase.zig");
pub const character = @import("c/character.zig");
pub const collision = @import("c/collision.zig");
pub const constraint = @import("c/constraint.zig");
pub const core = @import("c/core.zig");
pub const customshape = @import("c/customshape.zig");
pub const debug = @import("c/debug.zig");
pub const geometry = @import("c/geometry.zig");
pub const group = @import("c/group.zig");
pub const hair = @import("c/hair.zig");
pub const material = @import("c/material.zig");
pub const math = @import("c/math.zig");
pub const query = @import("c/query.zig");
pub const ragdoll = @import("c/ragdoll.zig");
pub const reflect = @import("c/reflect.zig");
pub const scene = @import("c/scene.zig");
pub const shape = @import("c/shape.zig");
pub const softbody = @import("c/softbody.zig");
pub const state = @import("c/state.zig");
pub const system = @import("c/system.zig");
pub const transformed = @import("c/transformed.zig");
pub const tree = @import("c/tree.zig");
pub const vehicle = @import("c/vehicle.zig");

/// Every module above. Walked by `abi_check.zig` and `misuse_sweep_test.zig`.
pub const modules = .{
    batch,
    body,
    broadphase,
    character,
    collision,
    constraint,
    core,
    customshape,
    debug,
    geometry,
    group,
    hair,
    material,
    math,
    query,
    ragdoll,
    reflect,
    scene,
    shape,
    softbody,
    state,
    system,
    transformed,
    tree,
    vehicle,
};
