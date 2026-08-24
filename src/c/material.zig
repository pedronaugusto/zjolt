//! ZJolt C declarations for physics materials.
//!
//! Mirrors `ffi/zjolt_material.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Color = core.Color;
pub const PhysicsMaterial = core.PhysicsMaterial;
pub const Result = core.Result;

pub extern fn zjoltPhysicsMaterialCreate(debug_name: ?[*:0]const u8, debug_color: ?*const Color, out: **PhysicsMaterial) Result;

pub extern fn zjoltPhysicsMaterialDefault() ?*const PhysicsMaterial;

pub extern fn zjoltPhysicsMaterialAddRef(material: *const PhysicsMaterial) void;

pub extern fn zjoltPhysicsMaterialRelease(material: *const PhysicsMaterial) void;

pub extern fn zjoltPhysicsMaterialGetRefCount(material: *const PhysicsMaterial) u32;

pub extern fn zjoltPhysicsMaterialGetDebugName(material: *const PhysicsMaterial) [*:0]const u8;

pub extern fn zjoltPhysicsMaterialGetDebugColor(material: *const PhysicsMaterial, out: *Color) void;
