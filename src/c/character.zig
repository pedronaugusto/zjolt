//! ZJolt C declarations for both character controllers and everything they call back into.
//!
//! Mirrors `ffi/zjolt_character.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide
//! and nothing to drift. `src/c.zig` lists every one of these and is what
//! the ABI cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");
const query = @import("query.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const Activation = core.Activation;
pub const AllowedDofs = core.AllowedDofs;
pub const BackFaceMode = core.BackFaceMode;
pub const BodyId = core.BodyId;
pub const Character = core.Character;
pub const GroundState = core.GroundState;
pub const MotionType = core.MotionType;
pub const ObjectLayer = core.ObjectLayer;
pub const PhysicsMaterial = core.PhysicsMaterial;
pub const PhysicsSystem = core.PhysicsSystem;
pub const Quat = core.Quat;
pub const RVec3 = core.RVec3;
pub const Result = core.Result;
pub const Shape = core.Shape;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
pub const QueryFilters = query.QueryFilters;

pub const RigidCharacter = opaque {};

pub const CharacterContactListener = opaque {};

pub const CharacterVsCharacterCollision = opaque {};

pub const CharacterDesc = extern struct {
    shape: ?*const Shape,
    position: RVec3,
    rotation: Quat,
    up: Vec3,
    shape_offset: Vec3,
    user_data: u64,
    max_slope_angle: f32,
    mass: f32,
    max_strength: f32,
    predictive_contact_distance: f32,
    character_padding: f32,
    penetration_recovery_speed: f32,
    collision_tolerance: f32,
    hit_reduction_cos_max_angle: f32,
    max_collision_iterations: u32,
    max_constraint_iterations: u32,
    max_num_hits: u32,
    back_face_mode: BackFaceMode,
    enhanced_internal_edge_removal: bool,
    inner_body_shape: ?*const Shape,
    inner_body_layer: ObjectLayer,
};

pub const CharacterUpdateSettings = extern struct {
    stick_to_floor_step_down: Vec3,
    walk_stairs_step_up: Vec3,
    walk_stairs_min_step_forward: f32,
    walk_stairs_step_forward_test: f32,
    walk_stairs_cos_angle_forward_contact: f32,
    walk_stairs_step_down_extra: Vec3,
};

pub extern fn zjoltCharacterDescInit(desc: *CharacterDesc) void;

pub extern fn zjoltCharacterUpdateSettingsInit(settings: *CharacterUpdateSettings) void;

pub extern fn zjoltCharacterCreate(system: *PhysicsSystem, desc: *const CharacterDesc, out: **Character) Result;

pub extern fn zjoltCharacterDestroy(character: ?*Character) void;

pub extern fn zjoltCharacterUpdate(character: *Character, delta_time: f32, gravity: *const Vec3, settings: ?*const CharacterUpdateSettings, filters: ?*const QueryFilters) Result;

pub extern fn zjoltCharacterGetPosition(character: *const Character, out: *RVec3) void;

pub extern fn zjoltCharacterSetPosition(character: *Character, position: *const RVec3) void;

pub extern fn zjoltCharacterGetRotation(character: *const Character, out: *Quat) void;

pub extern fn zjoltCharacterSetRotation(character: *Character, rotation: *const Quat) void;

pub extern fn zjoltCharacterGetLinearVelocity(character: *const Character, out: *Vec3) void;

pub extern fn zjoltCharacterSetLinearVelocity(character: *Character, velocity: *const Vec3) void;

pub extern fn zjoltCharacterGetGroundState(character: *const Character) GroundState;

pub extern fn zjoltCharacterIsSupported(character: *const Character) bool;

pub extern fn zjoltCharacterGetGroundNormal(character: *const Character, out: *Vec3) void;

pub extern fn zjoltCharacterGetGroundVelocity(character: *const Character, out: *Vec3) void;

pub extern fn zjoltCharacterGetGroundPosition(character: *const Character, out: *RVec3) void;

pub extern fn zjoltCharacterGetGroundBodyId(character: *const Character) BodyId;

pub extern fn zjoltCharacterGetGroundUserData(character: *const Character) u64;

pub extern fn zjoltCharacterUpdateGroundVelocity(character: *Character) void;

pub extern fn zjoltCharacterSetShape(character: *Character, shape: *const Shape, max_penetration_depth: f32, filters: ?*const QueryFilters, out_changed: ?*bool) Result;

pub extern fn zjoltCharacterGetShape(character: *const Character) ?*const Shape;

pub extern fn zjoltCharacterGetInnerBodyId(character: *const Character) BodyId;

pub const CharacterId = u32;

pub const character_id_invalid: CharacterId = 0xffff_ffff;

pub extern fn zjoltCharacterGetId(character: *const Character) CharacterId;

pub extern fn zjoltCharacterGetUp(character: *const Character, out: *Vec3) void;

pub extern fn zjoltCharacterSetUp(character: *Character, up: *const Vec3) void;

pub extern fn zjoltCharacterSetMaxSlopeAngle(character: *Character, radians: f32) void;

pub extern fn zjoltCharacterGetCosMaxSlopeAngle(character: *const Character) f32;

pub extern fn zjoltCharacterIsSlopeTooSteep(character: *const Character, normal: *const Vec3) bool;

pub extern fn zjoltCharacterGetGroundMaterial(character: *const Character) ?*const PhysicsMaterial;

pub extern fn zjoltCharacterGetGroundSubShapeId(character: *const Character) SubShapeId;

pub extern fn zjoltCharacterGetMass(character: *const Character) f32;

pub extern fn zjoltCharacterSetMass(character: *Character, mass: f32) void;

pub extern fn zjoltCharacterGetMaxStrength(character: *const Character) f32;

pub extern fn zjoltCharacterSetMaxStrength(character: *Character, max_strength: f32) void;

pub extern fn zjoltCharacterGetPenetrationRecoverySpeed(character: *const Character) f32;

pub extern fn zjoltCharacterSetPenetrationRecoverySpeed(character: *Character, speed: f32) void;

pub extern fn zjoltCharacterGetEnhancedInternalEdgeRemoval(character: *const Character) bool;

pub extern fn zjoltCharacterSetEnhancedInternalEdgeRemoval(character: *Character, apply: bool) void;

pub extern fn zjoltCharacterGetCharacterPadding(character: *const Character) f32;

pub extern fn zjoltCharacterGetMaxNumHits(character: *const Character) u32;

pub extern fn zjoltCharacterSetMaxNumHits(character: *Character, max_hits: u32) void;

pub extern fn zjoltCharacterGetHitReductionCosMaxAngle(character: *const Character) f32;

pub extern fn zjoltCharacterSetHitReductionCosMaxAngle(character: *Character, cos_max_angle: f32) void;

pub extern fn zjoltCharacterGetMaxHitsExceeded(character: *const Character) bool;

pub extern fn zjoltCharacterGetShapeOffset(character: *const Character, out: *Vec3) void;

pub extern fn zjoltCharacterSetShapeOffset(character: *Character, offset: *const Vec3) void;

pub extern fn zjoltCharacterGetUserData(character: *const Character) u64;

pub extern fn zjoltCharacterSetUserData(character: *Character, user_data: u64) void;

pub extern fn zjoltCharacterCancelVelocityTowardsSteepSlopes(character: *const Character, desired_velocity: *const Vec3, out: *Vec3) void;

pub extern fn zjoltCharacterStartTrackingContactChanges(character: *Character) void;

pub extern fn zjoltCharacterFinishTrackingContactChanges(character: *Character) void;

pub extern fn zjoltCharacterCanWalkStairs(character: *const Character, linear_velocity: *const Vec3) bool;

pub extern fn zjoltCharacterWalkStairs(character: *Character, delta_time: f32, step_up: *const Vec3, step_forward: *const Vec3, step_forward_test: *const Vec3, step_down_extra: *const Vec3, filters: ?*const QueryFilters, out_stepped: ?*bool) Result;

pub extern fn zjoltCharacterStickToFloor(character: *Character, step_down: *const Vec3, filters: ?*const QueryFilters, out_stuck: ?*bool) Result;

pub extern fn zjoltCharacterRefreshContacts(character: *Character, filters: ?*const QueryFilters) Result;

pub const CharacterContact = extern struct {
    body_b: BodyId,
    character_id_b: CharacterId,
    sub_shape_id_b: SubShapeId,
    position: RVec3,
    linear_velocity: Vec3,
    contact_normal: Vec3,
    surface_normal: Vec3,
    distance: f32,
    fraction: f32,
    motion_type_b: MotionType,
    is_sensor_b: bool,
    user_data: u64,
    material: ?*const PhysicsMaterial,
    had_collision: bool,
    was_discarded: bool,
    can_push_character: bool,
    is_back_facing_contact: bool,
};

pub extern fn zjoltCharacterGetActiveContacts(character: *const Character, out_contacts: ?[*]CharacterContact, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltCharacterHasCollidedWithBody(character: *const Character, body: BodyId) bool;

pub extern fn zjoltCharacterHasCollidedWithCharacter(character: *const Character, other_character_id: CharacterId) bool;

pub const CharacterContactSettings = extern struct {
    can_push_character: bool,
    can_receive_impulses: bool,
};

pub const CharacterContactListenerCallbacks = extern struct {
    on_adjust_body_velocity: ?*const fn (user: ?*anyopaque, character: CharacterId, body2: BodyId, io_linear_velocity: *Vec3, io_angular_velocity: *Vec3) callconv(.c) void = null,
    on_contact_validate: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact) callconv(.c) bool = null,
    on_contact_added: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact, io_settings: *CharacterContactSettings) callconv(.c) void = null,
    on_contact_persisted: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact, io_settings: *CharacterContactSettings) callconv(.c) void = null,
    on_contact_removed: ?*const fn (user: ?*anyopaque, character: CharacterId, body_id2: BodyId, sub_shape_id2: SubShapeId) callconv(.c) void = null,
    on_character_contact_validate: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact) callconv(.c) bool = null,
    on_character_contact_added: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact, io_settings: *CharacterContactSettings) callconv(.c) void = null,
    on_character_contact_persisted: ?*const fn (user: ?*anyopaque, character: CharacterId, contact: *const CharacterContact, io_settings: *CharacterContactSettings) callconv(.c) void = null,
    on_character_contact_removed: ?*const fn (user: ?*anyopaque, character: CharacterId, other_character_id: CharacterId, sub_shape_id2: SubShapeId) callconv(.c) void = null,
    on_contact_solve: ?*const fn (user: ?*anyopaque, character: CharacterId, body_id2: BodyId, sub_shape_id2: SubShapeId, contact_position: *const RVec3, contact_normal: *const Vec3, contact_velocity: *const Vec3, contact_material: ?*const PhysicsMaterial, character_velocity: *const Vec3, io_new_character_velocity: *Vec3) callconv(.c) void = null,
    on_character_contact_solve: ?*const fn (user: ?*anyopaque, character: CharacterId, other_character: CharacterId, sub_shape_id2: SubShapeId, contact_position: *const RVec3, contact_normal: *const Vec3, contact_velocity: *const Vec3, contact_material: ?*const PhysicsMaterial, character_velocity: *const Vec3, io_new_character_velocity: *Vec3) callconv(.c) void = null,
    user: ?*anyopaque = null,
};

pub extern fn zjoltCharacterContactListenerCreate(callbacks: *const CharacterContactListenerCallbacks, out: **CharacterContactListener) Result;

pub extern fn zjoltCharacterContactListenerDestroy(listener: ?*CharacterContactListener) void;

pub extern fn zjoltCharacterSetListener(character: *Character, listener: ?*CharacterContactListener) Result;

pub extern fn zjoltCharacterVsCharacterCollisionCreate(out: **CharacterVsCharacterCollision) Result;

pub extern fn zjoltCharacterVsCharacterCollisionDestroy(collision: ?*CharacterVsCharacterCollision) void;

pub extern fn zjoltCharacterVsCharacterCollisionAdd(collision: *CharacterVsCharacterCollision, character: *Character) void;

pub extern fn zjoltCharacterVsCharacterCollisionRemove(collision: *CharacterVsCharacterCollision, character: *const Character) void;

pub extern fn zjoltCharacterSetCharacterVsCharacterCollision(character: *Character, collision: ?*CharacterVsCharacterCollision) void;

pub const RigidCharacterDesc = extern struct {
    shape: ?*const Shape,
    position: RVec3,
    rotation: Quat,
    user_data: u64,
    up: Vec3,
    max_slope_angle: f32,
    enhanced_internal_edge_removal: bool,
    layer: ObjectLayer,
    mass: f32,
    friction: f32,
    gravity_factor: f32,
    allowed_dofs: AllowedDofs,
};

pub extern fn zjoltRigidCharacterDescInit(desc: *RigidCharacterDesc) void;

pub extern fn zjoltRigidCharacterCreate(system: *PhysicsSystem, desc: *const RigidCharacterDesc, out: **RigidCharacter) Result;

pub extern fn zjoltRigidCharacterDestroy(character: ?*RigidCharacter) void;

pub extern fn zjoltRigidCharacterAddToPhysicsSystem(character: *RigidCharacter, activation: Activation) void;

pub extern fn zjoltRigidCharacterRemoveFromPhysicsSystem(character: *RigidCharacter) void;

pub extern fn zjoltRigidCharacterActivate(character: *RigidCharacter) void;

pub extern fn zjoltRigidCharacterPostSimulation(character: *RigidCharacter, max_separation_distance: f32) void;

pub extern fn zjoltRigidCharacterSetLinearAndAngularVelocity(character: *RigidCharacter, linear_velocity: *const Vec3, angular_velocity: *const Vec3) void;

pub extern fn zjoltRigidCharacterGetLinearVelocity(character: *const RigidCharacter, out: *Vec3) void;

pub extern fn zjoltRigidCharacterSetLinearVelocity(character: *RigidCharacter, velocity: *const Vec3) void;

pub extern fn zjoltRigidCharacterAddLinearVelocity(character: *RigidCharacter, velocity: *const Vec3) void;

pub extern fn zjoltRigidCharacterAddImpulse(character: *RigidCharacter, impulse: *const Vec3) void;

pub extern fn zjoltRigidCharacterGetBodyId(character: *const RigidCharacter) BodyId;

pub extern fn zjoltRigidCharacterGetPositionAndRotation(character: *const RigidCharacter, out_position: ?*RVec3, out_rotation: ?*Quat) void;

pub extern fn zjoltRigidCharacterSetPositionAndRotation(character: *RigidCharacter, position: *const RVec3, rotation: *const Quat, activation: Activation) void;

pub extern fn zjoltRigidCharacterGetPosition(character: *const RigidCharacter, out: *RVec3) void;

pub extern fn zjoltRigidCharacterSetPosition(character: *RigidCharacter, position: *const RVec3, activation: Activation) void;

pub extern fn zjoltRigidCharacterGetRotation(character: *const RigidCharacter, out: *Quat) void;

pub extern fn zjoltRigidCharacterSetRotation(character: *RigidCharacter, rotation: *const Quat, activation: Activation) void;

pub extern fn zjoltRigidCharacterGetCenterOfMassPosition(character: *const RigidCharacter, out: *RVec3) void;

pub extern fn zjoltRigidCharacterGetLayer(character: *const RigidCharacter) ObjectLayer;

pub extern fn zjoltRigidCharacterSetLayer(character: *RigidCharacter, layer: ObjectLayer) void;

pub extern fn zjoltRigidCharacterSetShape(character: *RigidCharacter, shape: *const Shape, max_penetration_depth: f32, out_changed: ?*bool) Result;

pub extern fn zjoltRigidCharacterGetShape(character: *const RigidCharacter) ?*const Shape;

pub extern fn zjoltRigidCharacterGetId(character: *const RigidCharacter) CharacterId;

pub extern fn zjoltRigidCharacterGetUp(character: *const RigidCharacter, out: *Vec3) void;

pub extern fn zjoltRigidCharacterSetUp(character: *RigidCharacter, up: *const Vec3) void;

pub extern fn zjoltRigidCharacterSetMaxSlopeAngle(character: *RigidCharacter, radians: f32) void;

pub extern fn zjoltRigidCharacterGetCosMaxSlopeAngle(character: *const RigidCharacter) f32;

pub extern fn zjoltRigidCharacterIsSlopeTooSteep(character: *const RigidCharacter, normal: *const Vec3) bool;

pub extern fn zjoltRigidCharacterGetGroundState(character: *const RigidCharacter) GroundState;

pub extern fn zjoltRigidCharacterIsSupported(character: *const RigidCharacter) bool;

pub extern fn zjoltRigidCharacterGetGroundPosition(character: *const RigidCharacter, out: *RVec3) void;

pub extern fn zjoltRigidCharacterGetGroundNormal(character: *const RigidCharacter, out: *Vec3) void;

pub extern fn zjoltRigidCharacterGetGroundVelocity(character: *const RigidCharacter, out: *Vec3) void;

pub extern fn zjoltRigidCharacterGetGroundMaterial(character: *const RigidCharacter) ?*const PhysicsMaterial;

pub extern fn zjoltRigidCharacterGetGroundBodyId(character: *const RigidCharacter) BodyId;

pub extern fn zjoltRigidCharacterGetGroundSubShapeId(character: *const RigidCharacter) SubShapeId;

pub extern fn zjoltRigidCharacterGetGroundUserData(character: *const RigidCharacter) u64;
