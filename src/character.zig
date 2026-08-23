//! CharacterVirtual — a shape the host sweeps through the world itself.
//!
//! A virtual character is not a rigid body, and that is the point: it stops
//! dead, turns on the spot and climbs a step, none of which a barrel with a
//! collision shape does. The trade is that the world cannot see it, so an
//! optional inner rigid body exists to give it presence in casts and against
//! other bodies.

const std = @import("std");
const c = @import("c.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const query_mod = @import("query.zig");
const shape_mod = @import("shape.zig");
const system_mod = @import("system.zig");

pub const GroundState = c.GroundState;
pub const BackFaceMode = c.BackFaceMode;

/// Extra behaviour layered on a plain move: sticking to the floor on the way
/// down a slope, and stepping up stairs. Defaults come from Jolt.
pub const UpdateSettings = c.CharacterUpdateSettings;

pub fn defaultUpdateSettings() UpdateSettings {
    var settings: UpdateSettings = undefined;
    c.zjoltCharacterUpdateSettingsInit(&settings);
    return settings;
}

pub const Character = struct {
    handle: *c.Character,

    pub const Options = struct {
        /// Required. Its centre should sit at the character's centre; wrap a
        /// capsule in `Shape.initRotatedTranslated` to put its base at the
        /// origin instead.
        shape: shape_mod.Shape,
        position: math.RVec3 = math.rvec3_zero,
        rotation: math.Quat = math.quat_identity,
        /// Which way is up for this character. Need not be the world up.
        up: math.Vec3 = .{ .x = 0, .y = 1, .z = 0 },
        shape_offset: math.Vec3 = math.vec3_zero,
        user_data: u64 = 0,

        /// Radians. Ground steeper than this reports `.on_steep_ground`.
        max_slope_angle: f32 = std.math.degreesToRadians(50.0),
        mass: f32 = 70,
        /// Force the character can apply to push dynamic bodies, in newtons.
        max_strength: f32 = 100,
        /// How far to look ahead for contacts. Zero will get it stuck.
        predictive_contact_distance: f32 = 0.1,
        /// Gap kept between the shape and geometry so sweeps hit less.
        character_padding: f32 = 0.02,
        penetration_recovery_speed: f32 = 1,
        collision_tolerance: f32 = 1.0e-3,
        hit_reduction_cos_max_angle: f32 = 0.999,
        max_collision_iterations: u32 = 5,
        max_constraint_iterations: u32 = 15,
        max_num_hits: u32 = 256,
        back_face_mode: BackFaceMode = .collide,
        enhanced_internal_edge_removal: bool = false,

        /// Optional rigid body that follows the character so casts and other
        /// bodies can see it. Null for none.
        inner_body_shape: ?shape_mod.Shape = null,
        inner_body_layer: c.ObjectLayer = 0,
    };

    /// The character borrows `system` for its lifetime and must be `deinit`ed
    /// before it — Jolt's CharacterVirtual holds a pointer to the system, and
    /// outliving it is a dangling one.
    pub fn init(system: system_mod.PhysicsSystem, opts: Options) err.Error!Character {
        var desc: c.CharacterDesc = undefined;
        // Start from Jolt's defaults so a field this wrapper does not model
        // still gets a sensible value.
        c.zjoltCharacterDescInit(&desc);
        desc.shape = opts.shape.handle;
        desc.position = opts.position;
        desc.rotation = opts.rotation;
        desc.up = opts.up;
        desc.shape_offset = opts.shape_offset;
        desc.user_data = opts.user_data;
        desc.max_slope_angle = opts.max_slope_angle;
        desc.mass = opts.mass;
        desc.max_strength = opts.max_strength;
        desc.predictive_contact_distance = opts.predictive_contact_distance;
        desc.character_padding = opts.character_padding;
        desc.penetration_recovery_speed = opts.penetration_recovery_speed;
        desc.collision_tolerance = opts.collision_tolerance;
        desc.hit_reduction_cos_max_angle = opts.hit_reduction_cos_max_angle;
        desc.max_collision_iterations = opts.max_collision_iterations;
        desc.max_constraint_iterations = opts.max_constraint_iterations;
        desc.max_num_hits = opts.max_num_hits;
        desc.back_face_mode = opts.back_face_mode;
        desc.enhanced_internal_edge_removal = opts.enhanced_internal_edge_removal;
        desc.inner_body_shape = if (opts.inner_body_shape) |s| s.handle else null;
        desc.inner_body_layer = opts.inner_body_layer;

        var handle: *c.Character = undefined;
        try err.check(c.zjoltCharacterCreate(system.handle, &desc, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: Character) void {
        c.zjoltCharacterDestroy(self.handle);
    }

    /// Moves the character by its current velocity for `delta_time`, resolving
    /// collision. Set the velocity first with `setLinearVelocity`; this does
    /// not apply gravity on its own, because whether a grounded character
    /// should accumulate downward velocity is a game decision.
    ///
    /// `settings` null gives a plain move with no stair or floor handling.
    /// `filters` null collides with everything.
    pub fn update(
        self: Character,
        delta_time: f32,
        gravity: math.Vec3,
        settings: ?*const UpdateSettings,
        filters: ?*const query_mod.Filters,
    ) err.Error!void {
        try err.check(c.zjoltCharacterUpdate(
            self.handle,
            delta_time,
            &gravity,
            settings,
            filters,
        ));
    }

    //=========================================================================
    // State
    //=========================================================================

    pub fn getPosition(self: Character) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltCharacterGetPosition(self.handle, &out);
        return out;
    }

    pub fn setPosition(self: Character, position: math.RVec3) void {
        c.zjoltCharacterSetPosition(self.handle, &position);
    }

    pub fn getRotation(self: Character) math.Quat {
        var out: math.Quat = math.quat_identity;
        c.zjoltCharacterGetRotation(self.handle, &out);
        return out;
    }

    pub fn setRotation(self: Character, rotation: math.Quat) void {
        c.zjoltCharacterSetRotation(self.handle, &rotation);
    }

    pub fn getLinearVelocity(self: Character) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltCharacterGetLinearVelocity(self.handle, &out);
        return out;
    }

    pub fn setLinearVelocity(self: Character, velocity: math.Vec3) void {
        c.zjoltCharacterSetLinearVelocity(self.handle, &velocity);
    }

    //=========================================================================
    // Ground
    //=========================================================================

    pub fn groundState(self: Character) GroundState {
        return c.zjoltCharacterGetGroundState(self.handle);
    }

    /// True for `.on_ground` and `.on_steep_ground`.
    pub fn isSupported(self: Character) bool {
        return c.zjoltCharacterIsSupported(self.handle);
    }

    pub fn groundNormal(self: Character) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltCharacterGetGroundNormal(self.handle, &out);
        return out;
    }

    pub fn groundVelocity(self: Character) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltCharacterGetGroundVelocity(self.handle, &out);
        return out;
    }

    pub fn groundPosition(self: Character) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltCharacterGetGroundPosition(self.handle, &out);
        return out;
    }

    /// The body being stood on, or `invalid_body_id` when in the air.
    pub fn groundBodyId(self: Character) body_mod.BodyId {
        return c.zjoltCharacterGetGroundBodyId(self.handle);
    }

    pub fn groundUserData(self: Character) u64 {
        return c.zjoltCharacterGetGroundUserData(self.handle);
    }

    /// Recomputes the ground velocity, for reading after the ground body moved.
    pub fn updateGroundVelocity(self: Character) void {
        c.zjoltCharacterUpdateGroundVelocity(self.handle);
    }

    //=========================================================================
    // Shape
    //=========================================================================

    /// Swaps the shape — crouching, and standing back up. Returns false, with
    /// nothing changed, when the new shape would be more than
    /// `max_penetration_depth` inside the world: a refused stand-up under a
    /// low ceiling is a normal outcome, not an error.
    pub fn setShape(
        self: Character,
        shape: shape_mod.Shape,
        max_penetration_depth: f32,
        filters: ?*const query_mod.Filters,
    ) err.Error!bool {
        var changed: bool = false;
        try err.check(c.zjoltCharacterSetShape(
            self.handle,
            shape.handle,
            max_penetration_depth,
            filters,
            &changed,
        ));
        return changed;
    }

    pub fn getShape(self: Character) ?shape_mod.Shape {
        return .{ .handle = c.zjoltCharacterGetShape(self.handle) orelse return null };
    }

    /// The inner rigid body's id, or `invalid_body_id` when there is none.
    pub fn innerBodyId(self: Character) body_mod.BodyId {
        return c.zjoltCharacterGetInnerBodyId(self.handle);
    }
};
