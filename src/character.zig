//! CharacterVirtual — a shape the host sweeps through the world itself.
//!
//! A virtual character is not a rigid body, and that is the point: it stops
//! dead, turns on the spot and climbs a step, none of which a barrel with a
//! collision shape does. The trade is that the world cannot see it, so an
//! optional inner rigid body exists to give it presence in casts and against
//! other bodies.

const std = @import("std");
const c = @import("c/character.zig");
const err = @import("error.zig");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const descriptor = @import("descriptor.zig");
const query_mod = @import("query.zig");
const shape_mod = @import("shape.zig");
const state_c = @import("c/state.zig");
const stream_mod = @import("stream.zig");
const system_mod = @import("system.zig");
const transformed_mod = @import("transformed.zig");

pub const GroundState = c.GroundState;
pub const BackFaceMode = c.BackFaceMode;

/// The plane, in a character's LOCAL space, behind which a contact supports it
/// (in front, it only collides) — ground test's second half: rejects by WHERE a
/// contact landed, not its normal, so pressing against a wall doesn't read as
/// floor. `distance` is in units of `normal`'s length: local point p is behind
/// the plane when `dot(normal, p) + distance <= 0`. `normal` isn't normalised —
/// scaling it without `distance` moves the plane, not just rescales it.
pub const SupportingVolume = struct {
    normal: math.Vec3,
    distance: f32,
};

/// One overlap reported by `checkCollision`.
///
/// Not a `query.CollideShapeHit`: a virtual character is not a body, so an
/// overlap with another one has no body id and would come back
/// indistinguishable from an overlap with nothing. Exactly one of `body` and
/// `character_id` is set on any hit.
pub const CollisionHit = c.CharacterCollisionHit;

/// What `checkCollision` and `countCollisions` are asking about, beyond the
/// position. Every default is "the character as it stands".
pub const CollisionQuery = struct {
    /// Null uses the character's current rotation.
    rotation: ?math.Quat = null,
    /// A hint at which way the character is moving. It only steers which mesh
    /// edges count as active, so a wrong one costs accuracy on internal edges
    /// rather than correctness. Null means no hint.
    movement_direction: ?math.Vec3 = null,
    /// How far around the shape to report contacts. Zero matches the shape.
    max_separation_distance: f32 = 0,
    /// Null uses the character's current shape, which is what "does the
    /// standing pose fit here" wants. Pass another to ask about a stance the
    /// character has not switched to.
    shape: ?shape_mod.Shape = null,
};

/// Extra behaviour layered on a plain move: sticking to the floor on the way
/// down a slope, and stepping up stairs. Defaults come from Jolt.
pub const UpdateSettings = c.CharacterUpdateSettings;

pub fn defaultUpdateSettings() UpdateSettings {
    var settings: UpdateSettings = undefined;
    c.zjoltCharacterUpdateSettingsInit(&settings);
    return settings;
}

/// A pointer to an optional's payload, or null. Written once because taking
/// the address inline would be the address of a temporary.
fn optionalPtr(comptime T: type, value: *const ?T) ?*const T {
    return if (value.*) |*payload| payload else null;
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
        /// The solver stops once this little of the step is left. Jolt's
        /// early-out: smaller costs iterations, larger leaves motion
        /// unsimulated.
        min_time_remaining: f32 = 1.0e-4,
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
        /// The id the inner body takes instead of a generated one, so a
        /// rebuilt world hands the same character the same id — what a
        /// replay or a rollback compares against. `invalid_body_id` for a
        /// generated one; ignored without an `inner_body_shape`.
        inner_body_id_override: body_mod.BodyId = body_mod.invalid_body_id,
        inner_body_layer: c.ObjectLayer = 0,

        comptime {
            descriptor.requireModelled(@This(), c.CharacterDesc);
        }
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
        desc.inner_body_shape = if (opts.inner_body_shape) |s| s.handle else null;
        descriptor.crossByName(&desc, opts, &.{ "shape", "inner_body_shape" });

        var handle: *c.Character = undefined;
        try err.check(c.zjoltCharacterCreate(system.handle, &desc, &handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: Character) void {
        c.zjoltCharacterDestroy(self.handle);
    }

    /// Bytes `saveState` would write. Not stable — a character that gained
    /// contacts needs more — so ask each time rather than caching it.
    pub fn stateSize(self: Character) err.Error!usize {
        return characterStateSize(state_c.zjoltCharacterSaveState, self.handle);
    }

    /// `CharacterVirtual::SaveState` — position, rotation, velocity, ground
    /// state and the active contact list, into `buffer`, returning the part
    /// that was used. `error.BufferTooSmall` if it does not fit; ask
    /// `stateSize` first.
    pub fn saveState(self: Character, buffer: []u8) err.Error![]u8 {
        return characterSaveState(state_c.zjoltCharacterSaveState, self.handle, buffer);
    }

    /// `saveState` into memory from `allocator`. The caller owns the slice.
    pub fn saveStateAlloc(
        self: Character,
        allocator: std.mem.Allocator,
    ) (err.Error || std.mem.Allocator.Error)![]u8 {
        return characterSaveStateAlloc(state_c.zjoltCharacterSaveState, self.handle, allocator);
    }

    /// `saveState`, through `stream` instead of a resident buffer.
    pub fn saveStateStream(self: Character, stream: stream_mod.Stream) err.Error!void {
        try err.check(state_c.zjoltCharacterSaveStateStream(self.handle, &stream));
    }

    /// `CharacterVirtual::RestoreState`. `error.BadFormat` for a buffer that
    /// is not one `saveState` wrote, truncated, or damaged.
    pub fn restoreState(self: Character, data: []const u8) err.Error!void {
        try err.check(state_c.zjoltCharacterRestoreState(self.handle, data.ptr, data.len));
    }

    /// `restoreState`, reading through `stream`.
    pub fn restoreStateStream(self: Character, stream: stream_mod.Stream) err.Error!void {
        try err.check(state_c.zjoltCharacterRestoreStateStream(self.handle, &stream));
    }

    /// Moves the character by its current velocity for `delta_time`, resolving
    /// collision. Set velocity first with `setLinearVelocity`; this does not
    /// apply gravity — whether a grounded character accumulates downward
    /// velocity is a game decision. `settings` null: plain move, no stair/floor
    /// handling. `filters` null: collides with everything.
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

    /// The velocity `body_b` counts as having for ground-velocity purposes:
    /// its own linear/angular velocity (zero if STATIC), adjusted by this
    /// character's listener if one is installed — the building block
    /// `updateGroundVelocity` itself uses. `error.BodyNotFound` for a stale
    /// `body_b`.
    pub fn getAdjustedBodyVelocity(self: Character, body_b: body_mod.BodyId) err.Error!struct { linear_velocity: math.Vec3, angular_velocity: math.Vec3 } {
        var linear: math.Vec3 = math.vec3_zero;
        var angular: math.Vec3 = math.vec3_zero;
        try err.check(c.zjoltCharacterGetAdjustedBodyVelocity(self.handle, body_b, &linear, &angular));
        return .{ .linear_velocity = linear, .angular_velocity = angular };
    }

    /// What this character's own velocity would be if it stood on an object
    /// at `center_of_mass` moving with `linear_velocity`/`angular_velocity`
    /// — the other building block `updateGroundVelocity` uses, exposed
    /// standalone for a hypothetical ground body rather than the
    /// character's actual one.
    pub fn calculateGroundVelocity(
        self: Character,
        center_of_mass: math.RVec3,
        linear_velocity: math.Vec3,
        angular_velocity: math.Vec3,
        delta_time: f32,
    ) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltCharacterCalculateGroundVelocity(self.handle, &center_of_mass, &linear_velocity, &angular_velocity, delta_time, &out);
        return out;
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

    /// Gives the inner rigid body a shape of its own, independent of the
    /// one the character sweeps. `setShape` keeps the inner body in step
    /// with the new swept shape (what a crouch wants) — this undoes that
    /// for a character whose `inner_body_shape` was created DIFFERENT
    /// from `shape` (a cheap cast proxy), restoring the distinction
    /// `setShape` erases. Errors on a character with no inner body.
    pub fn setInnerBodyShape(self: Character, shape: shape_mod.Shape) err.Error!void {
        try err.check(c.zjoltCharacterSetInnerBodyShape(self.handle, shape.handle));
    }

    //=========================================================================
    // CharacterBase
    //=========================================================================

    /// This character through the surface it shares with `RigidCharacter`.
    pub fn asBase(self: Character) CharacterBase {
        return .{ .virtual_character = self };
    }

    /// Uniquely identifies this character, assigned when it was created.
    pub fn id(self: Character) CharacterId {
        return c.zjoltCharacterGetId(self.handle);
    }

    pub fn up(self: Character) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltCharacterGetUp(self.handle, &out);
        return out;
    }

    pub fn setUp(self: Character, new_up: math.Vec3) void {
        c.zjoltCharacterSetUp(self.handle, &new_up);
    }

    /// Radians. Ground steeper than this reports `.on_steep_ground`.
    pub fn setMaxSlopeAngle(self: Character, radians: f32) void {
        c.zjoltCharacterSetMaxSlopeAngle(self.handle, radians);
    }

    /// Jolt stores the angle as its cosine; this returns that, not radians.
    pub fn cosMaxSlopeAngle(self: Character) f32 {
        return c.zjoltCharacterGetCosMaxSlopeAngle(self.handle);
    }

    pub fn isSlopeTooSteep(self: Character, normal: math.Vec3) bool {
        return c.zjoltCharacterIsSlopeTooSteep(self.handle, &normal);
    }

    /// @see `SupportingVolume`. `init` installs one an inner radius above the
    /// shape's lowest point, which is what Jolt's own samples use — Jolt's
    /// bare default accepts every contact and reports a wall as ground.
    pub fn supportingVolume(self: Character) SupportingVolume {
        var volume: SupportingVolume = .{ .normal = math.vec3_zero, .distance = 0 };
        c.zjoltCharacterGetSupportingVolume(self.handle, &volume.normal, &volume.distance);
        return volume;
    }

    /// Raising the plane makes the character pickier about what counts as
    /// ground. Placing it at the shape's lowest point is the tempting mistake
    /// and breaks every slope: a capsule on a ramp touches it on the SIDE of
    /// its bottom cap, above the lowest point, so a floor-level plane wrongly
    /// reports "unsupported". A zero-length `normal` is refused: a character
    /// with one never reports ground again.
    pub fn setSupportingVolume(self: Character, volume: SupportingVolume) err.Error!void {
        try err.check(c.zjoltCharacterSetSupportingVolume(
            self.handle,
            &volume.normal,
            volume.distance,
        ));
    }

    /// Null when the character has never touched anything.
    pub fn groundMaterial(self: Character) ?*const c.PhysicsMaterial {
        return c.zjoltCharacterGetGroundMaterial(self.handle);
    }

    pub fn groundSubShapeId(self: Character) query_mod.SubShapeId {
        return c.zjoltCharacterGetGroundSubShapeId(self.handle);
    }

    pub fn mass(self: Character) f32 {
        return c.zjoltCharacterGetMass(self.handle);
    }

    pub fn setMass(self: Character, new_mass: f32) void {
        c.zjoltCharacterSetMass(self.handle, new_mass);
    }

    /// Force the character can apply to push dynamic bodies, in newtons.
    pub fn maxStrength(self: Character) f32 {
        return c.zjoltCharacterGetMaxStrength(self.handle);
    }

    pub fn setMaxStrength(self: Character, newtons: f32) void {
        c.zjoltCharacterSetMaxStrength(self.handle, newtons);
    }

    pub fn penetrationRecoverySpeed(self: Character) f32 {
        return c.zjoltCharacterGetPenetrationRecoverySpeed(self.handle);
    }

    pub fn setPenetrationRecoverySpeed(self: Character, speed: f32) void {
        c.zjoltCharacterSetPenetrationRecoverySpeed(self.handle, speed);
    }

    pub fn enhancedInternalEdgeRemoval(self: Character) bool {
        return c.zjoltCharacterGetEnhancedInternalEdgeRemoval(self.handle);
    }

    pub fn setEnhancedInternalEdgeRemoval(self: Character, apply: bool) void {
        c.zjoltCharacterSetEnhancedInternalEdgeRemoval(self.handle, apply);
    }

    pub fn characterPadding(self: Character) f32 {
        return c.zjoltCharacterGetCharacterPadding(self.handle);
    }

    pub fn maxNumHits(self: Character) u32 {
        return c.zjoltCharacterGetMaxNumHits(self.handle);
    }

    pub fn setMaxNumHits(self: Character, max_hits: u32) void {
        c.zjoltCharacterSetMaxNumHits(self.handle, max_hits);
    }

    pub fn hitReductionCosMaxAngle(self: Character) f32 {
        return c.zjoltCharacterGetHitReductionCosMaxAngle(self.handle);
    }

    pub fn setHitReductionCosMaxAngle(self: Character, cos_max_angle: f32) void {
        c.zjoltCharacterSetHitReductionCosMaxAngle(self.handle, cos_max_angle);
    }

    /// True if the last Update/WalkStairs/ExtendedUpdate hit more contacts
    /// than max_num_hits and had to discard some by distance.
    pub fn maxHitsExceeded(self: Character) bool {
        return c.zjoltCharacterGetMaxHitsExceeded(self.handle);
    }

    pub fn shapeOffset(self: Character) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltCharacterGetShapeOffset(self.handle, &out);
        return out;
    }

    /// Setting this on the fly can teleport the shape into collision; prefer
    /// setting it once at creation.
    pub fn setShapeOffset(self: Character, offset: math.Vec3) void {
        c.zjoltCharacterSetShapeOffset(self.handle, &offset);
    }

    pub fn userData(self: Character) u64 {
        return c.zjoltCharacterGetUserData(self.handle);
    }

    pub fn setUserData(self: Character, user_data: u64) void {
        c.zjoltCharacterSetUserData(self.handle, user_data);
    }

    /// Clamps `desired_velocity` so it will not carry the character further
    /// onto ground steeper than max_slope_angle. Call before `update` and feed
    /// the result to `setLinearVelocity`.
    pub fn cancelVelocityTowardsSteepSlopes(self: Character, desired_velocity: math.Vec3) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltCharacterCancelVelocityTowardsSteepSlopes(self.handle, &desired_velocity, &out);
        return out;
    }

    /// Groups a run of `update` / `walkStairs` / `stickToFloor` calls so the
    /// contact listener sees one added/persisted/removed transition per
    /// contact for the whole run. Every `startTrackingContactChanges` needs
    /// exactly one matching `finishTrackingContactChanges`.
    pub fn startTrackingContactChanges(self: Character) void {
        c.zjoltCharacterStartTrackingContactChanges(self.handle);
    }

    pub fn finishTrackingContactChanges(self: Character) void {
        c.zjoltCharacterFinishTrackingContactChanges(self.handle);
    }

    //=========================================================================
    // Stair walking and floor sticking, standalone
    //
    // `update` already runs both through `UpdateSettings` (Jolt's own
    // ExtendedUpdate). These are the two pieces on their own, for a host that wants them at a different point in its frame, or with different parameters per call.
    //=========================================================================

    /// True if the character just moved into a slope steeper than it can
    /// climb, the usual trigger for calling `walkStairs`.
    pub fn canWalkStairs(self: Character, linear_velocity: math.Vec3) bool {
        return c.zjoltCharacterCanWalkStairs(self.handle, &linear_velocity);
    }

    /// Casts up, forward and back down to try to place the character one
    /// stair higher. Returns whether a valid step was found; a refused step
    /// changes nothing and is a normal outcome, not an error.
    ///
    /// `filters` null collides with everything.
    pub fn walkStairs(
        self: Character,
        delta_time: f32,
        step_up: math.Vec3,
        step_forward: math.Vec3,
        step_forward_test: math.Vec3,
        step_down_extra: math.Vec3,
        filters: ?*const query_mod.Filters,
    ) err.Error!bool {
        var stepped: bool = false;
        try err.check(c.zjoltCharacterWalkStairs(
            self.handle,
            delta_time,
            &step_up,
            &step_forward,
            &step_forward_test,
            &step_down_extra,
            filters,
            &stepped,
        ));
        return stepped;
    }

    /// Projects the character down onto the floor by up to `step_down` when a
    /// floor is found within that distance. Meant to run right after a
    /// horizontal move that lost contact with a floor the character is still
    /// effectively standing on. False means no floor was found within range —
    /// a normal outcome (the character is airborne), not an error.
    pub fn stickToFloor(
        self: Character,
        step_down: math.Vec3,
        filters: ?*const query_mod.Filters,
    ) err.Error!bool {
        var stuck: bool = false;
        try err.check(c.zjoltCharacterStickToFloor(self.handle, &step_down, filters, &stuck));
        return stuck;
    }

    /// Recomputes contacts at the character's current position. Call after
    /// teleporting a character so its ground state reflects where it landed
    /// instead of where it was.
    pub fn refreshContacts(self: Character, filters: ?*const query_mod.Filters) err.Error!void {
        try err.check(c.zjoltCharacterRefreshContacts(self.handle, filters));
    }

    /// How many contacts the last update/walkStairs/etc. call found, without
    /// reading any of them.
    pub fn activeContactCount(self: Character) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCharacterGetActiveContacts(self.handle, null, 0, &count));
        return count;
    }

    /// Every contact the last update/walkStairs/etc. call found, written into
    /// `out` and returned as the prefix filled. Only contacts with
    /// `had_collision` set are actual touches; the rest were predictive.
    /// `error.BufferTooSmall` when `out` is shorter than `activeContactCount`
    /// — `out` still receives what fits, but the slice is lost with the error.
    /// The allocation-free form: keep one buffer per character and reuse it.
    pub fn activeContacts(
        self: Character,
        out: []CharacterContact,
    ) err.Error![]CharacterContact {
        var actual: u32 = 0;
        try err.check(c.zjoltCharacterGetActiveContacts(
            self.handle,
            out.ptr,
            @intCast(out.len),
            &actual,
        ));
        return out[0..actual];
    }

    /// `activeContacts` into a fresh allocation, for a caller that has no
    /// buffer to reuse. Two crossings rather than one, and one allocation per
    /// call: prefer `activeContacts` anywhere this runs every frame.
    pub fn getActiveContacts(
        self: Character,
        allocator: std.mem.Allocator,
    ) err.Error![]CharacterContact {
        const contacts = try allocator.alloc(CharacterContact, try self.activeContactCount());
        errdefer allocator.free(contacts);
        return self.activeContacts(contacts);
    }

    pub fn hasCollidedWithBody(self: Character, body: body_mod.BodyId) bool {
        return c.zjoltCharacterHasCollidedWithBody(self.handle, body);
    }

    pub fn hasCollidedWithCharacter(self: Character, other_character_id: CharacterId) bool {
        return c.zjoltCharacterHasCollidedWithCharacter(self.handle, other_character_id);
    }

    //=========================================================================
    // Asking about a placement the character is not at
    //=========================================================================

    /// The character's current volume, placed where the character is, as a
    /// standalone queryable shape. The caller owns it: `deinit` it. The only
    /// way to hit a virtual character with a cast: Jolt never puts a
    /// `Character` in the broad phase, so no system query finds one without an
    /// inner body. A SNAPSHOT — copied out, does not follow the character
    /// afterwards.
    pub fn getTransformedShape(self: Character) err.Error!transformed_mod.TransformedShape {
        var handle: *c.TransformedShape = undefined;
        try err.check(c.zjoltCharacterGetTransformedShape(self.handle, &handle));
        return .{ .handle = handle };
    }

    /// What the character's shape would overlap at `position`, into `buffer`.
    /// The character, contacts included, doesn't change. `error.BufferTooSmall`
    /// if overlaps exceed `buffer`; call `countCollisions` first. NOT a plain
    /// shape overlap: applies its back-face/active-edge/padding settings, skips
    /// inner body, tests whatever `setCharacterVsCharacterCollision` set, and
    /// is unreachable any other way — these characters skip the broad phase.
    pub fn checkCollision(
        self: Character,
        position: math.RVec3,
        collision_query: CollisionQuery,
        filters: ?*const query_mod.Filters,
        buffer: []CollisionHit,
    ) err.Error![]CollisionHit {
        var count: u32 = 0;
        try err.check(c.zjoltCharacterCheckCollision(
            self.handle,
            &position,
            optionalPtr(math.Quat, &collision_query.rotation),
            optionalPtr(math.Vec3, &collision_query.movement_direction),
            collision_query.max_separation_distance,
            if (collision_query.shape) |s| s.handle else null,
            filters,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return err.filled(buffer, count);
    }

    /// How many overlaps `checkCollision` would report, without a buffer to
    /// hold them. One traversal, same as the filling form.
    pub fn countCollisions(
        self: Character,
        position: math.RVec3,
        collision_query: CollisionQuery,
        filters: ?*const query_mod.Filters,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltCharacterCheckCollision(
            self.handle,
            &position,
            optionalPtr(math.Quat, &collision_query.rotation),
            optionalPtr(math.Vec3, &collision_query.movement_direction),
            collision_query.max_separation_distance,
            if (collision_query.shape) |s| s.handle else null,
            filters,
            null,
            0,
            &count,
        ));
        return count;
    }

    /// Attaches a contact listener built by `ContactListener(T).init` — pass
    /// its `.handle`. Pass `null` to detach whatever listener is currently
    /// attached.
    pub fn setListener(self: Character, listener: ?*c.CharacterContactListener) err.Error!void {
        try err.check(c.zjoltCharacterSetListener(self.handle, listener));
    }

    /// Whatever `setListener` last installed, or null for none. The character
    /// does not own it and `deinit`ing the character does not destroy it —
    /// this reports who is attached, it transfers nothing.
    pub fn getListener(self: Character) ?*c.CharacterContactListener {
        return c.zjoltCharacterGetListener(self.handle);
    }

    /// Attaches a character-vs-character collision checker built by
    /// `CharacterVsCharacterCollision.init` or
    /// `CharacterVsCharacterCollider(T).attach` — pass its `.handle`, the way
    /// `setListener` takes `ContactListener(T)`'s. `null` detaches: the
    /// character then collides with no other character.
    pub fn setCharacterVsCharacterCollision(
        self: Character,
        collision: ?*c.CharacterVsCharacterCollision,
    ) void {
        c.zjoltCharacterSetCharacterVsCharacterCollision(self.handle, collision);
    }
};

//=============================================================================
// CharacterContactListener
//
// Fires as a virtual character finds, keeps and loses contacts. A
// callback names the character by CharacterId, not handle: Jolt hands this code a raw pointer to its own internal CharacterVirtual, not the Character this API handed out.
//=============================================================================

pub const CharacterId = c.CharacterId;
pub const invalid_character_id = c.character_id_invalid;
pub const CharacterContact = c.CharacterContact;
pub const CharacterContactSettings = c.CharacterContactSettings;

const ErrorInt = std.meta.Int(.unsigned, @bitSizeOf(anyerror));

/// The first error a callback signalled. Mirrors `zjolt.StepListener`'s
/// `Failure` — duplicated rather than shared, because each subsystem file in
/// this binding stands on its own.
const Failure = struct {
    code: std.atomic.Value(ErrorInt) = std.atomic.Value(ErrorInt).init(0),

    fn record(self: *Failure, e: anyerror) void {
        _ = self.code.cmpxchgStrong(0, @intFromError(e), .monotonic, .monotonic);
    }

    fn take(self: *Failure) ?anyerror {
        const code = self.code.swap(0, .monotonic);
        return if (code == 0) null else @errorFromInt(code);
    }
};

fn returnsError(comptime Fn: type) bool {
    const ret = @typeInfo(Fn).@"fn".return_type orelse return false;
    return @typeInfo(ret) == .error_union;
}

/// A host's answer to a virtual character's contacts, built from whichever of
/// `onAdjustBodyVelocity`, `onContactValidate/Added/ Persisted/Removed`,
/// `onCharacterContact*`, and `on*ContactSolve` `T` declares (any omitted one
/// does not fire). Each may return `!...` instead: the error is stashed, not
/// unwound, and surfaces from `check`. `context` must outlive the listener; the
/// value must not move once `attach`ed.
pub fn ContactListener(comptime T: type) type {
    system_mod.requireAnyDecl(T, &.{
        "onAdjustBodyVelocity",
        "onContactValidate",
        "onContactAdded",
        "onContactPersisted",
        "onContactRemoved",
        "onCharacterContactValidate",
        "onCharacterContactAdded",
        "onCharacterContactPersisted",
        "onCharacterContactRemoved",
        "onContactSolve",
        "onCharacterContactSolve",
    });

    return struct {
        const Self = @This();

        context: *T,
        failure: Failure = .{},
        handle: ?*c.CharacterContactListener = null,

        pub fn init(context: *T) Self {
            return .{ .context = context };
        }

        const Thunks = struct {
            fn selfOf(user: ?*anyopaque) *Self {
                return @ptrCast(@alignCast(user.?));
            }

            fn adjustBodyVelocity(
                user: ?*anyopaque,
                character: CharacterId,
                body2: body_mod.BodyId,
                linear: *math.Vec3,
                angular: *math.Vec3,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onAdjustBodyVelocity))) {
                    T.onAdjustBodyVelocity(self.context, character, body2, linear, angular) catch |e|
                        self.failure.record(e);
                } else {
                    T.onAdjustBodyVelocity(self.context, character, body2, linear, angular);
                }
            }

            fn contactValidate(
                user: ?*anyopaque,
                character: CharacterId,
                contact: *const CharacterContact,
            ) callconv(.c) bool {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onContactValidate))) {
                    return T.onContactValidate(self.context, character, contact) catch |e| {
                        self.failure.record(e);
                        return false;
                    };
                }
                return T.onContactValidate(self.context, character, contact);
            }

            fn contactAdded(
                user: ?*anyopaque,
                character: CharacterId,
                contact: *const CharacterContact,
                settings: *CharacterContactSettings,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onContactAdded))) {
                    T.onContactAdded(self.context, character, contact, settings) catch |e|
                        self.failure.record(e);
                } else {
                    T.onContactAdded(self.context, character, contact, settings);
                }
            }

            fn contactPersisted(
                user: ?*anyopaque,
                character: CharacterId,
                contact: *const CharacterContact,
                settings: *CharacterContactSettings,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onContactPersisted))) {
                    T.onContactPersisted(self.context, character, contact, settings) catch |e|
                        self.failure.record(e);
                } else {
                    T.onContactPersisted(self.context, character, contact, settings);
                }
            }

            fn contactRemoved(
                user: ?*anyopaque,
                character: CharacterId,
                body2: body_mod.BodyId,
                sub_shape_id2: query_mod.SubShapeId,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onContactRemoved))) {
                    T.onContactRemoved(self.context, character, body2, sub_shape_id2) catch |e|
                        self.failure.record(e);
                } else {
                    T.onContactRemoved(self.context, character, body2, sub_shape_id2);
                }
            }

            fn characterContactValidate(
                user: ?*anyopaque,
                character: CharacterId,
                contact: *const CharacterContact,
            ) callconv(.c) bool {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onCharacterContactValidate))) {
                    return T.onCharacterContactValidate(self.context, character, contact) catch |e| {
                        self.failure.record(e);
                        return false;
                    };
                }
                return T.onCharacterContactValidate(self.context, character, contact);
            }

            fn characterContactAdded(
                user: ?*anyopaque,
                character: CharacterId,
                contact: *const CharacterContact,
                settings: *CharacterContactSettings,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onCharacterContactAdded))) {
                    T.onCharacterContactAdded(self.context, character, contact, settings) catch |e|
                        self.failure.record(e);
                } else {
                    T.onCharacterContactAdded(self.context, character, contact, settings);
                }
            }

            fn characterContactPersisted(
                user: ?*anyopaque,
                character: CharacterId,
                contact: *const CharacterContact,
                settings: *CharacterContactSettings,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onCharacterContactPersisted))) {
                    T.onCharacterContactPersisted(self.context, character, contact, settings) catch |e|
                        self.failure.record(e);
                } else {
                    T.onCharacterContactPersisted(self.context, character, contact, settings);
                }
            }

            fn characterContactRemoved(
                user: ?*anyopaque,
                character: CharacterId,
                other_character_id: CharacterId,
                sub_shape_id2: query_mod.SubShapeId,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onCharacterContactRemoved))) {
                    T.onCharacterContactRemoved(self.context, character, other_character_id, sub_shape_id2) catch |e|
                        self.failure.record(e);
                } else {
                    T.onCharacterContactRemoved(self.context, character, other_character_id, sub_shape_id2);
                }
            }

            fn contactSolve(
                user: ?*anyopaque,
                character: CharacterId,
                body2: body_mod.BodyId,
                sub_shape_id2: query_mod.SubShapeId,
                position: *const math.RVec3,
                normal: *const math.Vec3,
                velocity: *const math.Vec3,
                material: ?*const c.PhysicsMaterial,
                character_velocity: *const math.Vec3,
                new_velocity: *math.Vec3,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onContactSolve))) {
                    T.onContactSolve(
                        self.context,
                        character,
                        body2,
                        sub_shape_id2,
                        position,
                        normal,
                        velocity,
                        material,
                        character_velocity,
                        new_velocity,
                    ) catch |e| self.failure.record(e);
                } else {
                    T.onContactSolve(
                        self.context,
                        character,
                        body2,
                        sub_shape_id2,
                        position,
                        normal,
                        velocity,
                        material,
                        character_velocity,
                        new_velocity,
                    );
                }
            }

            fn characterContactSolve(
                user: ?*anyopaque,
                character: CharacterId,
                other_character: CharacterId,
                sub_shape_id2: query_mod.SubShapeId,
                position: *const math.RVec3,
                normal: *const math.Vec3,
                velocity: *const math.Vec3,
                material: ?*const c.PhysicsMaterial,
                character_velocity: *const math.Vec3,
                new_velocity: *math.Vec3,
            ) callconv(.c) void {
                const self = selfOf(user);
                if (comptime returnsError(@TypeOf(T.onCharacterContactSolve))) {
                    T.onCharacterContactSolve(
                        self.context,
                        character,
                        other_character,
                        sub_shape_id2,
                        position,
                        normal,
                        velocity,
                        material,
                        character_velocity,
                        new_velocity,
                    ) catch |e| self.failure.record(e);
                } else {
                    T.onCharacterContactSolve(
                        self.context,
                        character,
                        other_character,
                        sub_shape_id2,
                        position,
                        normal,
                        velocity,
                        material,
                        character_velocity,
                        new_velocity,
                    );
                }
            }
        };

        /// Builds the listener. `context` must outlive it.
        pub fn attach(self: *Self) err.Error!void {
            const callbacks: c.CharacterContactListenerCallbacks = .{
                .on_adjust_body_velocity = if (@hasDecl(T, "onAdjustBodyVelocity")) Thunks.adjustBodyVelocity else null,
                .on_contact_validate = if (@hasDecl(T, "onContactValidate")) Thunks.contactValidate else null,
                .on_contact_added = if (@hasDecl(T, "onContactAdded")) Thunks.contactAdded else null,
                .on_contact_persisted = if (@hasDecl(T, "onContactPersisted")) Thunks.contactPersisted else null,
                .on_contact_removed = if (@hasDecl(T, "onContactRemoved")) Thunks.contactRemoved else null,
                .on_character_contact_validate = if (@hasDecl(T, "onCharacterContactValidate")) Thunks.characterContactValidate else null,
                .on_character_contact_added = if (@hasDecl(T, "onCharacterContactAdded")) Thunks.characterContactAdded else null,
                .on_character_contact_persisted = if (@hasDecl(T, "onCharacterContactPersisted")) Thunks.characterContactPersisted else null,
                .on_character_contact_removed = if (@hasDecl(T, "onCharacterContactRemoved")) Thunks.characterContactRemoved else null,
                .on_contact_solve = if (@hasDecl(T, "onContactSolve")) Thunks.contactSolve else null,
                .on_character_contact_solve = if (@hasDecl(T, "onCharacterContactSolve")) Thunks.characterContactSolve else null,
                .user = @ptrCast(self),
            };
            var handle: *c.CharacterContactListener = undefined;
            try err.check(c.zjoltCharacterContactListenerCreate(&callbacks, &handle));
            self.handle = handle;
        }

        /// Detach from every character with `character.setListener(null)`
        /// first — a character whose listener outlives this call is left
        /// pointing at freed memory.
        pub fn deinit(self: *Self) void {
            if (self.handle) |h| c.zjoltCharacterContactListenerDestroy(h);
            self.handle = null;
        }

        /// Re-raises the first error a callback signalled since the last
        /// call, and clears it.
        pub fn check(self: *Self) anyerror!void {
            if (self.failure.take()) |e| return e;
        }
    };
}

//=============================================================================
// Character-vs-character collision
//
// CharacterVirtual is not in the broad phase; nothing sees it unless
// told to look. Jolt's CharacterVsCharacterCollisionSimple: a plain list, brute-force checked. NOT thread-safe — only one CharacterVirtual may check against it at a time.
//=============================================================================

pub const CharacterVsCharacterCollision = struct {
    handle: *c.CharacterVsCharacterCollision,

    pub fn init() err.Error!CharacterVsCharacterCollision {
        var handle: *c.CharacterVsCharacterCollision = undefined;
        try err.check(c.zjoltCharacterVsCharacterCollisionCreate(&handle));
        return .{ .handle = handle };
    }

    pub fn deinit(self: CharacterVsCharacterCollision) void {
        c.zjoltCharacterVsCharacterCollisionDestroy(self.handle);
    }

    pub fn add(self: CharacterVsCharacterCollision, character: Character) void {
        c.zjoltCharacterVsCharacterCollisionAdd(self.handle, character.handle);
    }

    pub fn remove(self: CharacterVsCharacterCollision, character: Character) void {
        c.zjoltCharacterVsCharacterCollisionRemove(self.handle, character.handle);
    }
};

//=============================================================================
// A custom character-vs-character broad phase
//
// `CharacterVsCharacterCollision` above is Jolt's brute-force list. This
// is the general interface it implements — CollideCharacter/CastCharacter, the two questions `update` asks about OTHER characters — for a host wanting a spatial structure, team filtering, or a sometimes-colliding pairing instead.
//=============================================================================

/// Handed to `onCollideCharacter`/`onCastCharacter` in place of Jolt's own
/// collector. Call `.visit` once per OTHER character `character` should be
/// tested against — this only decides WHO is tested; the real narrow-phase test
/// is the same one the brute-force list runs. Returns whether to keep visiting.
/// Valid only for the call's duration; do not store it.
pub const CandidateVisitor = struct {
    visit_fn: c.CharacterVsCharacterVisitFn,
    visit_user: ?*anyopaque,

    pub fn visit(self: CandidateVisitor, candidate: Character) bool {
        return self.visit_fn(self.visit_user, candidate.handle);
    }
};

/// A host's answer to "who does this character collide/cast against", built
/// from whichever of `onCollideCharacter`/`onCastCharacter` `T` declares — an
/// omitted one means `character` collides with nothing through that path. Each
/// may return `!void` instead: the error is stashed (@see `ContactListener`)
/// and surfaces from `check`, stopping candidate visits there. `context` must
/// outlive the collision object; the value must not move once `attach`ed.
pub fn CharacterVsCharacterCollider(comptime T: type) type {
    system_mod.requireAnyDecl(T, &.{ "onCollideCharacter", "onCastCharacter" });

    return struct {
        const Self = @This();

        context: *T,
        failure: Failure = .{},
        handle: ?*c.CharacterVsCharacterCollision = null,

        pub fn init(context: *T) Self {
            return .{ .context = context };
        }

        const Thunks = struct {
            fn selfOf(user: ?*anyopaque) *Self {
                return @ptrCast(@alignCast(user.?));
            }

            fn collideCharacter(
                user: ?*anyopaque,
                character: CharacterId,
                center_of_mass_transform: *const math.RMat44,
                visit: c.CharacterVsCharacterVisitFn,
                visit_user: ?*anyopaque,
            ) callconv(.c) void {
                const self = selfOf(user);
                const candidates: CandidateVisitor = .{ .visit_fn = visit, .visit_user = visit_user };
                if (comptime returnsError(@TypeOf(T.onCollideCharacter))) {
                    T.onCollideCharacter(self.context, character, center_of_mass_transform, candidates) catch |e|
                        self.failure.record(e);
                } else {
                    T.onCollideCharacter(self.context, character, center_of_mass_transform, candidates);
                }
            }

            fn castCharacter(
                user: ?*anyopaque,
                character: CharacterId,
                center_of_mass_transform: *const math.RMat44,
                direction: *const math.Vec3,
                visit: c.CharacterVsCharacterVisitFn,
                visit_user: ?*anyopaque,
            ) callconv(.c) void {
                const self = selfOf(user);
                const candidates: CandidateVisitor = .{ .visit_fn = visit, .visit_user = visit_user };
                if (comptime returnsError(@TypeOf(T.onCastCharacter))) {
                    T.onCastCharacter(self.context, character, center_of_mass_transform, direction, candidates) catch |e|
                        self.failure.record(e);
                } else {
                    T.onCastCharacter(self.context, character, center_of_mass_transform, direction, candidates);
                }
            }
        };

        /// Builds the collision object. `context` must outlive it. Install
        /// it exactly like `CharacterVsCharacterCollision.init`'s result,
        /// through `Character.setCharacterVsCharacterCollision`.
        pub fn attach(self: *Self) err.Error!void {
            const callbacks: c.CharacterVsCharacterCollisionCallbacks = .{
                .collide_character = if (@hasDecl(T, "onCollideCharacter")) Thunks.collideCharacter else null,
                .cast_character = if (@hasDecl(T, "onCastCharacter")) Thunks.castCharacter else null,
                .user = @ptrCast(self),
            };
            var handle: *c.CharacterVsCharacterCollision = undefined;
            try err.check(c.zjoltCharacterVsCharacterCollisionCreateCustom(&callbacks, &handle));
            self.handle = handle;
        }

        /// Detach from every character with
        /// `setCharacterVsCharacterCollision(null)` first — one that outlives
        /// this call is left pointing at freed memory.
        pub fn deinit(self: *Self) void {
            if (self.handle) |h| c.zjoltCharacterVsCharacterCollisionDestroy(h);
            self.handle = null;
        }

        /// Re-raises the first error a callback signalled since the last
        /// call, and clears it.
        pub fn check(self: *Self) anyerror!void {
            if (self.failure.take()) |e| return e;
        }
    };
}

//=============================================================================
// Saving one character's state
//
// `CharacterVirtual::SaveState` and `CharacterBase::SaveState`. Both kinds get
// the same six calls from one implementation each, parameterised by the C
// entry point, rather than two hand-written copies that can drift.
//
// A rollback of the whole simulation wants `PhysicsSystem.state()` with its
// `characters` option instead: that keeps world and characters in one payload
// that cannot be half-restored. These are for replicating ONE character.
//=============================================================================

fn characterStateSize(comptime saveFn: anytype, handle: anytype) err.Error!usize {
    var needed: usize = 0;
    try err.check(saveFn(handle, null, 0, &needed));
    return needed;
}

fn characterSaveState(comptime saveFn: anytype, handle: anytype, buffer: []u8) err.Error![]u8 {
    var written: usize = 0;
    try err.check(saveFn(handle, buffer.ptr, buffer.len, &written));
    return buffer[0..written];
}

fn characterSaveStateAlloc(
    comptime saveFn: anytype,
    handle: anytype,
    allocator: std.mem.Allocator,
) (err.Error || std.mem.Allocator.Error)![]u8 {
    const needed = try characterStateSize(saveFn, handle);
    const buffer = try allocator.alloc(u8, needed);
    errdefer allocator.free(buffer);
    return try characterSaveState(saveFn, handle, buffer);
}

//=============================================================================
// RigidCharacter
//
// The other character base: a real dynamic rigid body the host drives by setting its velocity every frame, same as Character above, but collision response, sleeping, and pushes from other dynamics fall out of the solver.
// Prefer this when the world needs to see the character as an ordinary body (felt by a trigger, knocked back by an explosion).
//=============================================================================

pub const RigidCharacter = struct {
    handle: *c.RigidCharacter,

    /// Bytes `saveState` would write.
    pub fn stateSize(self: RigidCharacter) err.Error!usize {
        return characterStateSize(state_c.zjoltRigidCharacterSaveState, self.handle);
    }

    /// `CharacterBase::SaveState` — ground state, ground body, ground
    /// position, normal and velocity. Its POSITION and VELOCITY live in its
    /// body and are saved by `PhysicsSystem.state()` like any other body's;
    /// this is the part that is not.
    pub fn saveState(self: RigidCharacter, buffer: []u8) err.Error![]u8 {
        return characterSaveState(state_c.zjoltRigidCharacterSaveState, self.handle, buffer);
    }

    /// `saveState` into memory from `allocator`. The caller owns the slice.
    pub fn saveStateAlloc(
        self: RigidCharacter,
        allocator: std.mem.Allocator,
    ) (err.Error || std.mem.Allocator.Error)![]u8 {
        return characterSaveStateAlloc(
            state_c.zjoltRigidCharacterSaveState,
            self.handle,
            allocator,
        );
    }

    /// `saveState`, through `stream` instead of a resident buffer.
    pub fn saveStateStream(self: RigidCharacter, stream: stream_mod.Stream) err.Error!void {
        try err.check(state_c.zjoltRigidCharacterSaveStateStream(self.handle, &stream));
    }

    /// `CharacterBase::RestoreState`.
    pub fn restoreState(self: RigidCharacter, data: []const u8) err.Error!void {
        try err.check(state_c.zjoltRigidCharacterRestoreState(self.handle, data.ptr, data.len));
    }

    /// `restoreState`, reading through `stream`.
    pub fn restoreStateStream(self: RigidCharacter, stream: stream_mod.Stream) err.Error!void {
        try err.check(state_c.zjoltRigidCharacterRestoreStateStream(self.handle, &stream));
    }

    pub const Options = struct {
        /// Required. Its centre should sit at the character's centre; wrap a
        /// capsule in `Shape.initRotatedTranslated` to put its base at the
        /// origin instead.
        shape: shape_mod.Shape,
        position: math.RVec3 = math.rvec3_zero,
        rotation: math.Quat = math.quat_identity,
        user_data: u64 = 0,
        /// Which way is up for this character. Need not be the world up.
        up: math.Vec3 = .{ .x = 0, .y = 1, .z = 0 },
        /// Radians. Ground steeper than this reports `.on_steep_ground`.
        max_slope_angle: f32 = std.math.degreesToRadians(50.0),
        enhanced_internal_edge_removal: bool = false,
        layer: c.ObjectLayer = 0,
        mass: f32 = 80,
        friction: f32 = 0.2,
        /// Multiplies the system's gravity for this character alone.
        gravity_factor: f32 = 1,
        /// Jolt's own default: translation on every axis, no rotation — a
        /// character that does not tip over.
        allowed_dofs: c.AllowedDofs = .{
            .rotation_x = false,
            .rotation_y = false,
            .rotation_z = false,
        },

        comptime {
            descriptor.requireModelled(@This(), c.RigidCharacterDesc);
        }
    };

    /// Builds the character's rigid body but does not add it to the
    /// system — call `addToPhysicsSystem` to make it move and collide.
    /// Fails with `error.OutOfMemory` if `system` already holds
    /// max_bodies bodies; nothing is left half-created.
    ///
    /// Borrows `system` for its lifetime; must be `deinit`ed before it.
    pub fn init(system: system_mod.PhysicsSystem, opts: Options) err.Error!RigidCharacter {
        var desc: c.RigidCharacterDesc = undefined;
        // Start from Jolt's defaults so a field this wrapper does not model
        // still gets a sensible value.
        c.zjoltRigidCharacterDescInit(&desc);
        desc.shape = opts.shape.handle;
        descriptor.crossByName(&desc, opts, &.{"shape"});

        var handle: *c.RigidCharacter = undefined;
        try err.check(c.zjoltRigidCharacterCreate(system.handle, &desc, &handle));
        return .{ .handle = handle };
    }

    /// Destroys the character's rigid body along with the character. Safe to
    /// call whether or not the character was ever added to the physics
    /// system.
    pub fn deinit(self: RigidCharacter) void {
        c.zjoltRigidCharacterDestroy(self.handle);
    }

    pub fn addToPhysicsSystem(self: RigidCharacter, activation: c.Activation) void {
        c.zjoltRigidCharacterAddToPhysicsSystem(self.handle, activation);
    }

    pub fn removeFromPhysicsSystem(self: RigidCharacter) void {
        c.zjoltRigidCharacterRemoveFromPhysicsSystem(self.handle);
    }

    pub fn activate(self: RigidCharacter) void {
        c.zjoltRigidCharacterActivate(self.handle);
    }

    /// Call once after every `PhysicsSystem.step` so ground state reflects
    /// where the character ended up.
    pub fn postSimulation(self: RigidCharacter, max_separation_distance: f32) void {
        c.zjoltRigidCharacterPostSimulation(self.handle, max_separation_distance);
    }

    pub fn setLinearAndAngularVelocity(self: RigidCharacter, linear_velocity: math.Vec3, angular_velocity: math.Vec3) void {
        c.zjoltRigidCharacterSetLinearAndAngularVelocity(self.handle, &linear_velocity, &angular_velocity);
    }

    pub fn getLinearVelocity(self: RigidCharacter) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltRigidCharacterGetLinearVelocity(self.handle, &out);
        return out;
    }

    pub fn setLinearVelocity(self: RigidCharacter, velocity: math.Vec3) void {
        c.zjoltRigidCharacterSetLinearVelocity(self.handle, &velocity);
    }

    pub fn addLinearVelocity(self: RigidCharacter, velocity: math.Vec3) void {
        c.zjoltRigidCharacterAddLinearVelocity(self.handle, &velocity);
    }

    pub fn addImpulse(self: RigidCharacter, impulse: math.Vec3) void {
        c.zjoltRigidCharacterAddImpulse(self.handle, &impulse);
    }

    pub fn bodyId(self: RigidCharacter) body_mod.BodyId {
        return c.zjoltRigidCharacterGetBodyId(self.handle);
    }

    /// Uniquely identifies this character, assigned when it was created.
    ///
    /// Distinct from `bodyId`: a rigid character owns a body, and the two
    /// identifier spaces are separate. `CharacterId` is what a contact from
    /// `Character` names when it hits this one.
    pub fn id(self: RigidCharacter) CharacterId {
        return c.zjoltRigidCharacterGetId(self.handle);
    }

    pub const PositionAndRotation = struct { position: math.RVec3, rotation: math.Quat };

    pub fn getPositionAndRotation(self: RigidCharacter) PositionAndRotation {
        var position: math.RVec3 = math.rvec3_zero;
        var rotation: math.Quat = math.quat_identity;
        c.zjoltRigidCharacterGetPositionAndRotation(self.handle, &position, &rotation);
        return .{ .position = position, .rotation = rotation };
    }

    pub fn setPositionAndRotation(self: RigidCharacter, position: math.RVec3, rotation: math.Quat, activation: c.Activation) void {
        c.zjoltRigidCharacterSetPositionAndRotation(self.handle, &position, &rotation, activation);
    }

    pub fn getPosition(self: RigidCharacter) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltRigidCharacterGetPosition(self.handle, &out);
        return out;
    }

    pub fn setPosition(self: RigidCharacter, position: math.RVec3, activation: c.Activation) void {
        c.zjoltRigidCharacterSetPosition(self.handle, &position, activation);
    }

    pub fn getRotation(self: RigidCharacter) math.Quat {
        var out: math.Quat = math.quat_identity;
        c.zjoltRigidCharacterGetRotation(self.handle, &out);
        return out;
    }

    pub fn setRotation(self: RigidCharacter, rotation: math.Quat, activation: c.Activation) void {
        c.zjoltRigidCharacterSetRotation(self.handle, &rotation, activation);
    }

    pub fn centerOfMassPosition(self: RigidCharacter) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltRigidCharacterGetCenterOfMassPosition(self.handle, &out);
        return out;
    }

    pub fn layer(self: RigidCharacter) c.ObjectLayer {
        return c.zjoltRigidCharacterGetLayer(self.handle);
    }

    pub fn setLayer(self: RigidCharacter, new_layer: c.ObjectLayer) void {
        c.zjoltRigidCharacterSetLayer(self.handle, new_layer);
    }

    /// Swaps the shape — crouching, and standing back up. Returns false, with
    /// nothing changed, when the new shape would be more than
    /// `max_penetration_depth` inside the world.
    pub fn setShape(self: RigidCharacter, shape: shape_mod.Shape, max_penetration_depth: f32) err.Error!bool {
        var changed: bool = false;
        try err.check(c.zjoltRigidCharacterSetShape(self.handle, shape.handle, max_penetration_depth, &changed));
        return changed;
    }

    pub fn getShape(self: RigidCharacter) ?shape_mod.Shape {
        return .{ .handle = c.zjoltRigidCharacterGetShape(self.handle) orelse return null };
    }

    //=========================================================================
    // CharacterBase
    //=========================================================================

    /// This character through the surface it shares with `Character`.
    pub fn asBase(self: RigidCharacter) CharacterBase {
        return .{ .rigid_character = self };
    }

    pub fn up(self: RigidCharacter) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltRigidCharacterGetUp(self.handle, &out);
        return out;
    }

    pub fn setUp(self: RigidCharacter, new_up: math.Vec3) void {
        c.zjoltRigidCharacterSetUp(self.handle, &new_up);
    }

    pub fn setMaxSlopeAngle(self: RigidCharacter, radians: f32) void {
        c.zjoltRigidCharacterSetMaxSlopeAngle(self.handle, radians);
    }

    pub fn cosMaxSlopeAngle(self: RigidCharacter) f32 {
        return c.zjoltRigidCharacterGetCosMaxSlopeAngle(self.handle);
    }

    pub fn isSlopeTooSteep(self: RigidCharacter, normal: math.Vec3) bool {
        return c.zjoltRigidCharacterIsSlopeTooSteep(self.handle, &normal);
    }

    pub fn groundState(self: RigidCharacter) GroundState {
        return c.zjoltRigidCharacterGetGroundState(self.handle);
    }

    /// True for `.on_ground` and `.on_steep_ground`.
    pub fn isSupported(self: RigidCharacter) bool {
        return c.zjoltRigidCharacterIsSupported(self.handle);
    }

    pub fn groundPosition(self: RigidCharacter) math.RVec3 {
        var out: math.RVec3 = math.rvec3_zero;
        c.zjoltRigidCharacterGetGroundPosition(self.handle, &out);
        return out;
    }

    pub fn groundNormal(self: RigidCharacter) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltRigidCharacterGetGroundNormal(self.handle, &out);
        return out;
    }

    pub fn groundVelocity(self: RigidCharacter) math.Vec3 {
        var out: math.Vec3 = math.vec3_zero;
        c.zjoltRigidCharacterGetGroundVelocity(self.handle, &out);
        return out;
    }

    /// Null when the character has never touched anything.
    pub fn groundMaterial(self: RigidCharacter) ?*const c.PhysicsMaterial {
        return c.zjoltRigidCharacterGetGroundMaterial(self.handle);
    }

    /// The body being stood on, or `invalid_body_id` when in the air.
    pub fn groundBodyId(self: RigidCharacter) body_mod.BodyId {
        return c.zjoltRigidCharacterGetGroundBodyId(self.handle);
    }

    pub fn groundSubShapeId(self: RigidCharacter) query_mod.SubShapeId {
        return c.zjoltRigidCharacterGetGroundSubShapeId(self.handle);
    }

    pub fn groundUserData(self: RigidCharacter) u64 {
        return c.zjoltRigidCharacterGetGroundUserData(self.handle);
    }

    /// @see `SupportingVolume`. A rigid character checks it in
    /// `postSimulation` rather than during a sweep, so a change takes effect
    /// at the next one.
    pub fn supportingVolume(self: RigidCharacter) SupportingVolume {
        var volume: SupportingVolume = .{ .normal = math.vec3_zero, .distance = 0 };
        c.zjoltRigidCharacterGetSupportingVolume(self.handle, &volume.normal, &volume.distance);
        return volume;
    }

    /// @see `Character.setSupportingVolume` for what raising the plane does
    /// and for the mistake to avoid.
    pub fn setSupportingVolume(self: RigidCharacter, volume: SupportingVolume) err.Error!void {
        try err.check(c.zjoltRigidCharacterSetSupportingVolume(
            self.handle,
            &volume.normal,
            volume.distance,
        ));
    }

    //=========================================================================
    // Asking about a placement the character is not at
    //=========================================================================

    /// The character's body and shape as a standalone queryable handle; the
    /// caller owns it and `deinit`s it. Less essential than the virtual
    /// character's — this one has a real body, so a cast against the system
    /// already finds it. @see `Character.getTransformedShape` for the
    /// snapshot rule, which applies here too.
    pub fn getTransformedShape(self: RigidCharacter) err.Error!transformed_mod.TransformedShape {
        var handle: *c.TransformedShape = undefined;
        try err.check(c.zjoltRigidCharacterGetTransformedShape(self.handle, &handle));
        return .{ .handle = handle };
    }

    /// Everything the character's shape would overlap if it stood at
    /// `position`. @see `Character.checkCollision` for the protocol. No filters
    /// to pass: Jolt builds them from the character's own object layer, always
    /// skipping its own body and every sensor. `character_id` on every hit is
    /// `invalid_character_id` — a rigid character collides through the broad
    /// phase, so no character-vs-character list is in play.
    pub fn checkCollision(
        self: RigidCharacter,
        position: math.RVec3,
        collision_query: CollisionQuery,
        buffer: []CollisionHit,
    ) err.Error![]CollisionHit {
        var count: u32 = 0;
        try err.check(c.zjoltRigidCharacterCheckCollision(
            self.handle,
            &position,
            optionalPtr(math.Quat, &collision_query.rotation),
            optionalPtr(math.Vec3, &collision_query.movement_direction),
            collision_query.max_separation_distance,
            if (collision_query.shape) |s| s.handle else null,
            buffer.ptr,
            @intCast(buffer.len),
            &count,
        ));
        return err.filled(buffer, count);
    }

    /// How many overlaps `checkCollision` would report, without a buffer to
    /// hold them.
    pub fn countCollisions(
        self: RigidCharacter,
        position: math.RVec3,
        collision_query: CollisionQuery,
    ) err.Error!u32 {
        var count: u32 = 0;
        try err.check(c.zjoltRigidCharacterCheckCollision(
            self.handle,
            &position,
            optionalPtr(math.Quat, &collision_query.rotation),
            optionalPtr(math.Vec3, &collision_query.movement_direction),
            collision_query.max_separation_distance,
            if (collision_query.shape) |s| s.handle else null,
            null,
            0,
            &count,
        ));
        return count;
    }
};

//=============================================================================
// CharacterBase
//
// The ground state, ground normal/velocity/body/material/sub-shape, shape,
// up vector and supporting-volume plane `Character` and `RigidCharacter`
// both carry. `asBase` on either builds one; every method here forwards to
// the same-named method the concrete type already has, so a call through it
// costs one tag check over calling the concrete type directly, and there is
// no method here either concrete type lacks.
//=============================================================================

pub const CharacterBase = union(enum) {
    virtual_character: Character,
    rigid_character: RigidCharacter,

    pub fn groundState(self: CharacterBase) GroundState {
        return switch (self) {
            inline else => |char| char.groundState(),
        };
    }

    /// True for `.on_ground` and `.on_steep_ground`.
    pub fn isSupported(self: CharacterBase) bool {
        return switch (self) {
            inline else => |char| char.isSupported(),
        };
    }

    pub fn groundNormal(self: CharacterBase) math.Vec3 {
        return switch (self) {
            inline else => |char| char.groundNormal(),
        };
    }

    pub fn groundVelocity(self: CharacterBase) math.Vec3 {
        return switch (self) {
            inline else => |char| char.groundVelocity(),
        };
    }

    pub fn groundPosition(self: CharacterBase) math.RVec3 {
        return switch (self) {
            inline else => |char| char.groundPosition(),
        };
    }

    /// The body being stood on, or `invalid_body_id` when in the air.
    pub fn groundBodyId(self: CharacterBase) body_mod.BodyId {
        return switch (self) {
            inline else => |char| char.groundBodyId(),
        };
    }

    pub fn groundUserData(self: CharacterBase) u64 {
        return switch (self) {
            inline else => |char| char.groundUserData(),
        };
    }

    /// Null when the character has never touched anything.
    pub fn groundMaterial(self: CharacterBase) ?*const c.PhysicsMaterial {
        return switch (self) {
            inline else => |char| char.groundMaterial(),
        };
    }

    pub fn groundSubShapeId(self: CharacterBase) query_mod.SubShapeId {
        return switch (self) {
            inline else => |char| char.groundSubShapeId(),
        };
    }

    pub fn getShape(self: CharacterBase) ?shape_mod.Shape {
        return switch (self) {
            inline else => |char| char.getShape(),
        };
    }

    pub fn up(self: CharacterBase) math.Vec3 {
        return switch (self) {
            inline else => |char| char.up(),
        };
    }

    pub fn setUp(self: CharacterBase, new_up: math.Vec3) void {
        switch (self) {
            inline else => |char| char.setUp(new_up),
        }
    }

    /// Radians. Ground steeper than this reports `.on_steep_ground`.
    pub fn setMaxSlopeAngle(self: CharacterBase, radians: f32) void {
        switch (self) {
            inline else => |char| char.setMaxSlopeAngle(radians),
        }
    }

    /// Jolt stores the angle as its cosine; this returns that, not radians.
    pub fn cosMaxSlopeAngle(self: CharacterBase) f32 {
        return switch (self) {
            inline else => |char| char.cosMaxSlopeAngle(),
        };
    }

    pub fn isSlopeTooSteep(self: CharacterBase, normal: math.Vec3) bool {
        return switch (self) {
            inline else => |char| char.isSlopeTooSteep(normal),
        };
    }

    /// @see `Character.setSupportingVolume` for what raising the plane does
    /// and the mistake to avoid.
    pub fn supportingVolume(self: CharacterBase) SupportingVolume {
        return switch (self) {
            inline else => |char| char.supportingVolume(),
        };
    }

    pub fn setSupportingVolume(self: CharacterBase, volume: SupportingVolume) err.Error!void {
        switch (self) {
            inline else => |char| try char.setSupportingVolume(volume),
        }
    }

    /// @see `Character.stateSize`. The two kinds save DIFFERENT amounts —
    /// a virtual character carries its own position and contacts, a rigid one
    /// only what `CharacterBase` holds — so a buffer saved through one kind is
    /// refused by the other's restore on the container tag.
    pub fn stateSize(self: CharacterBase) err.Error!usize {
        return switch (self) {
            inline else => |char| char.stateSize(),
        };
    }

    pub fn saveState(self: CharacterBase, buffer: []u8) err.Error![]u8 {
        return switch (self) {
            inline else => |char| char.saveState(buffer),
        };
    }

    pub fn saveStateAlloc(
        self: CharacterBase,
        allocator: std.mem.Allocator,
    ) (err.Error || std.mem.Allocator.Error)![]u8 {
        return switch (self) {
            inline else => |char| char.saveStateAlloc(allocator),
        };
    }

    pub fn saveStateStream(self: CharacterBase, stream: stream_mod.Stream) err.Error!void {
        switch (self) {
            inline else => |char| try char.saveStateStream(stream),
        }
    }

    pub fn restoreState(self: CharacterBase, data: []const u8) err.Error!void {
        switch (self) {
            inline else => |char| try char.restoreState(data),
        }
    }

    pub fn restoreStateStream(self: CharacterBase, stream: stream_mod.Stream) err.Error!void {
        switch (self) {
            inline else => |char| try char.restoreStateStream(stream),
        }
    }
};
