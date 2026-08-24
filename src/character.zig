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

    //=========================================================================
    // CharacterBase
    //=========================================================================

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
    // `update` already runs both of these through `UpdateSettings` when it is
    // given one (Jolt's own ExtendedUpdate). These are the two pieces on their
    // own, for a host that wants to run them at a different point in its frame
    // than `update`, or with different parameters per call.
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

    /// Every contact the last update/walkStairs/etc. call found. Only
    /// contacts with `had_collision` set are actual touches — the rest were
    /// predictive and never became real.
    pub fn getActiveContacts(self: Character, allocator: std.mem.Allocator) err.Error![]CharacterContact {
        var count: u32 = 0;
        try err.check(c.zjoltCharacterGetActiveContacts(self.handle, null, 0, &count));

        const contacts = try allocator.alloc(CharacterContact, count);
        errdefer allocator.free(contacts);
        var actual: u32 = 0;
        try err.check(c.zjoltCharacterGetActiveContacts(
            self.handle,
            contacts.ptr,
            @intCast(contacts.len),
            &actual,
        ));
        return contacts[0..actual];
    }

    pub fn hasCollidedWithBody(self: Character, body: body_mod.BodyId) bool {
        return c.zjoltCharacterHasCollidedWithBody(self.handle, body);
    }

    pub fn hasCollidedWithCharacter(self: Character, other_character_id: CharacterId) bool {
        return c.zjoltCharacterHasCollidedWithCharacter(self.handle, other_character_id);
    }

    /// Attaches a contact listener built by `ContactListener(T).init` — pass
    /// its `.handle`. Pass `null` to detach whatever listener is currently
    /// attached.
    pub fn setListener(self: Character, listener: ?*c.CharacterContactListener) void {
        c.zjoltCharacterSetListener(self.handle, listener);
    }

    /// Attaches a character-vs-character collision checker. `null` detaches:
    /// the character then collides with no other character.
    pub fn setCharacterVsCharacterCollision(self: Character, collision: ?CharacterVsCharacterCollision) void {
        c.zjoltCharacterSetCharacterVsCharacterCollision(
            self.handle,
            if (collision) |col| col.handle else null,
        );
    }
};

//=============================================================================
// CharacterContactListener
//
// Fires as a virtual character finds, keeps and loses contacts. A callback
// names the character it fired for by CharacterId, not by handle: Jolt hands
// this code a raw pointer to its own internal CharacterVirtual, which is not
// the Character this API handed out and cannot be turned back into one.
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
/// these `T` declares — any it omits simply does not fire, matching Jolt's own
/// do-nothing defaults. Each may instead return the same type wrapped in `!`;
/// a signalled error is stashed rather than unwound across the callback (Jolt
/// is built with exceptions off, and a Zig panic crossing one skips lock
/// destructors and can wedge the next physics step) and comes back out of
/// `check`. A failed validate rejects the contact — visibly wrong rather than
/// silently accepted.
///
/// ```zig
/// pub fn onAdjustBodyVelocity(self: *T, character: CharacterId, body2: BodyId, linear: *Vec3, angular: *Vec3) void
/// pub fn onContactValidate(self: *T, character: CharacterId, contact: *const CharacterContact) bool
/// pub fn onContactAdded(self: *T, character: CharacterId, contact: *const CharacterContact, settings: *CharacterContactSettings) void
/// pub fn onContactPersisted(self: *T, character: CharacterId, contact: *const CharacterContact, settings: *CharacterContactSettings) void
/// pub fn onContactRemoved(self: *T, character: CharacterId, body2: BodyId, sub_shape_id2: SubShapeId) void
/// pub fn onCharacterContactValidate(self: *T, character: CharacterId, contact: *const CharacterContact) bool
/// pub fn onCharacterContactAdded(self: *T, character: CharacterId, contact: *const CharacterContact, settings: *CharacterContactSettings) void
/// pub fn onCharacterContactPersisted(self: *T, character: CharacterId, contact: *const CharacterContact, settings: *CharacterContactSettings) void
/// pub fn onCharacterContactRemoved(self: *T, character: CharacterId, other_character_id: CharacterId, sub_shape_id2: SubShapeId) void
/// pub fn onContactSolve(self: *T, character: CharacterId, body2: BodyId, sub_shape_id2: SubShapeId, position: *const RVec3, normal: *const Vec3, velocity: *const Vec3, material: ?*const PhysicsMaterial, character_velocity: *const Vec3, new_velocity: *Vec3) void
/// pub fn onCharacterContactSolve(self: *T, character: CharacterId, other_character: CharacterId, sub_shape_id2: SubShapeId, position: *const RVec3, normal: *const Vec3, velocity: *const Vec3, material: ?*const PhysicsMaterial, character_velocity: *const Vec3, new_velocity: *Vec3) void
/// ```
///
/// `context` must outlive the listener, and the value returned here must not
/// move once `attach`ed — the C side holds a pointer to it.
pub fn ContactListener(comptime T: type) type {
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
// CharacterVirtual is not in the broad phase, so nothing sees it unless it is
// told to look. This is Jolt's CharacterVsCharacterCollisionSimple: a plain
// list of characters, checked by brute force. It is not thread-safe — only one
// CharacterVirtual may be checking collision against it at a time.
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
// RigidCharacter
//
// Jolt's own name for this is "Character" — spent above on the swept virtual
// one, which this binding settled on first. This is the other character base
// class: a real dynamic rigid body that the host drives by setting its
// velocity every frame, same as Character above, but collision response,
// sleeping and being pushed by other dynamics fall out of the ordinary
// rigid-body solver instead of a hand-rolled sweep. Prefer this one when the
// world needs to see the character as an ordinary body — felt by a trigger
// volume, knocked back by an explosion — and can live with the solver's
// collision response instead of a per-frame sweep.
//=============================================================================

pub const RigidCharacter = struct {
    handle: *c.RigidCharacter,

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
    };

    /// Builds the character's rigid body but does not add it to the system —
    /// call `addToPhysicsSystem` to make it move and collide.
    ///
    /// Fails with `error.OutOfMemory` if `system` is already holding
    /// max_bodies bodies; nothing is left half-created.
    ///
    /// The character borrows `system` for its lifetime and must be `deinit`ed
    /// before it.
    pub fn init(system: system_mod.PhysicsSystem, opts: Options) err.Error!RigidCharacter {
        var desc: c.RigidCharacterDesc = undefined;
        // Start from Jolt's defaults so a field this wrapper does not model
        // still gets a sensible value.
        c.zjoltRigidCharacterDescInit(&desc);
        desc.shape = opts.shape.handle;
        desc.position = opts.position;
        desc.rotation = opts.rotation;
        desc.user_data = opts.user_data;
        desc.up = opts.up;
        desc.max_slope_angle = opts.max_slope_angle;
        desc.enhanced_internal_edge_removal = opts.enhanced_internal_edge_removal;
        desc.layer = opts.layer;
        desc.mass = opts.mass;
        desc.friction = opts.friction;
        desc.gravity_factor = opts.gravity_factor;
        desc.allowed_dofs = opts.allowed_dofs;

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
};
