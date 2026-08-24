//! zjolt — Zig bindings for Jolt Physics.
//!
//! The package owns no policy: no fixed timestep, no entity system, no
//! renderer. A host drives it as `build shapes -> create bodies -> step ->
//! read back`.
//!
//! ```zig
//! try zjolt.init(.{ .allocator = gpa });
//! defer zjolt.deinit();
//!
//! const jobs = try zjolt.JobSystem.initThreadPool(.{});
//! defer jobs.deinit();
//!
//! const system = try zjolt.PhysicsSystem.init(.{
//!     .layers = zjolt.layersFromType(MyLayers),
//! });
//! defer system.deinit();
//!
//! const shape = try zjolt.Shape.initSphere(0.5, .{});
//! defer shape.release();
//!
//! const ball = try system.bodies().createAndAdd(.{
//!     .shape = shape,
//!     .object_layer = MyLayers.moving,
//!     .position = zjolt.rvec3(0, 10, 0),
//! }, .activate);
//!
//! // Per frame:
//! _ = try system.step(1.0 / 60.0, 1, jobs);
//! const transform = system.bodies().getTransform(ball);
//! ```

const std = @import("std");

/// The raw C declarations, one namespace per public header — `core.body`,
/// `core.shape`, `core.constraint` and so on, mirroring `ffi/zjolt_*.h`.
///
/// This is the escape hatch: anything the typed surface above does not wrap
/// is reachable here, ABI-checked, and documented as a first-class way to
/// call it rather than as a fallback. It used to be one flat namespace; the
/// module name is now part of the path, so `core.max_physics_jobs` is
/// `c.core.max_physics_jobs`.
pub const c = @import("c.zig");

/// Shorthand for the module everything in this file happens to need.
const core = c.core;

const error_mod = @import("error.zig");
const math_mod = @import("math.zig");
const memory_mod = @import("memory.zig");
const material_mod = @import("material.zig");
const shape_mod = @import("shape.zig");
const transformed_mod = @import("transformed.zig");
const body_mod = @import("body.zig");
const query_mod = @import("query.zig");
const system_mod = @import("system.zig");
const character_mod = @import("character.zig");
const broadphase_mod = @import("broadphase.zig");
const batch_mod = @import("batch.zig");
const group_mod = @import("group.zig");
const state_mod = @import("state.zig");
const softbody_mod = @import("softbody.zig");
const vehicle_mod = @import("vehicle.zig");
const debug_mod = @import("debug.zig");
const ragdoll_mod = @import("ragdoll.zig");
const hair_mod = @import("hair.zig");
const constraint_mod = @import("constraint.zig");
const scene_mod = @import("scene.zig");

//=============================================================================
// Public surface
//=============================================================================

pub const Error = error_mod.Error;
pub const resultName = error_mod.name;
pub const lastError = error_mod.lastError;

pub const Vec3 = math_mod.Vec3;
pub const RVec3 = math_mod.RVec3;
pub const Quat = math_mod.Quat;
pub const Real = math_mod.Real;
pub const Mat44 = math_mod.Mat44;
pub const RMat44 = math_mod.RMat44;
pub const AABox = math_mod.AABox;
pub const MassProperties = math_mod.MassProperties;
pub const ShapeStats = math_mod.ShapeStats;
pub const vec3 = math_mod.vec3;
pub const rvec3 = math_mod.rvec3;
pub const quat = math_mod.quat;
pub const quatFromAxisAngle = math_mod.quatFromAxisAngle;
pub const toRVec3 = math_mod.toRVec3;
pub const vec3_zero = math_mod.vec3_zero;
pub const rvec3_zero = math_mod.rvec3_zero;
pub const quat_identity = math_mod.quat_identity;
pub const mat44_identity = math_mod.mat44_identity;
pub const rmat44_identity = math_mod.rmat44_identity;
pub const gravity_earth = math_mod.gravity_earth;

pub const quatMultiply = math_mod.quatMultiply;
pub const quatRotateVector = math_mod.quatRotateVector;
pub const quatInverse = math_mod.quatInverse;
pub const quatConjugate = math_mod.quatConjugate;
pub const quatDot = math_mod.quatDot;
pub const quatIsNormalized = math_mod.quatIsNormalized;
pub const quatNormalize = math_mod.quatNormalize;
pub const quatGetAxisAngle = math_mod.quatGetAxisAngle;
pub const quatFromTo = math_mod.quatFromTo;
pub const quatFromEulerAngles = math_mod.quatFromEulerAngles;
pub const quatGetEulerAngles = math_mod.quatGetEulerAngles;
pub const quatGetPerpendicular = math_mod.quatGetPerpendicular;
pub const quatGetRotationAngle = math_mod.quatGetRotationAngle;
pub const quatGetTwist = math_mod.quatGetTwist;
pub const quatGetSwingTwist = math_mod.quatGetSwingTwist;
pub const quatLerp = math_mod.quatLerp;
pub const quatSlerp = math_mod.quatSlerp;
pub const vec3Lerp = math_mod.vec3Lerp;
pub const rvec3Lerp = math_mod.rvec3Lerp;
pub const mat44FromRotationTranslation = math_mod.mat44FromRotationTranslation;
pub const mat44Multiply = math_mod.mat44Multiply;
pub const mat44Inverse = math_mod.mat44Inverse;
pub const mat44InverseRotationTranslation = math_mod.mat44InverseRotationTranslation;
pub const mat44TransformPoint = math_mod.mat44TransformPoint;
pub const mat44TransformDirection = math_mod.mat44TransformDirection;
pub const rmat44FromRotationTranslation = math_mod.rmat44FromRotationTranslation;
pub const rmat44Multiply = math_mod.rmat44Multiply;
pub const rmat44Inverse = math_mod.rmat44Inverse;
pub const rmat44InverseRotationTranslation = math_mod.rmat44InverseRotationTranslation;
pub const rmat44TransformPoint = math_mod.rmat44TransformPoint;
pub const rmat44TransformDirection = math_mod.rmat44TransformDirection;

pub const Shape = shape_mod.Shape;
pub const ShapeSubType = shape_mod.SubType;
pub const MutableCompound = shape_mod.MutableCompound;
pub const CompoundChild = shape_mod.CompoundChild;
pub const compoundChild = shape_mod.compoundChild;
pub const SubShapeId = core.SubShapeId;
pub const sub_shape_id_empty = shape_mod.sub_shape_id_empty;
pub const height_field_no_collision = shape_mod.height_field_no_collision;

pub const TransformedShape = transformed_mod.TransformedShape;
pub const TransformedShapeTransform = transformed_mod.Transform;
pub const TransformedShapeTriangleWalk = transformed_mod.TriangleWalk;

pub const PhysicsMaterial = material_mod.PhysicsMaterial;
pub const Color = material_mod.Color;
pub const color = material_mod.color;

pub const BodyId = body_mod.BodyId;
pub const invalid_body_id = body_mod.invalid_body_id;
pub const BodyDesc = body_mod.BodyDesc;
pub const BodyInterface = body_mod.BodyInterface;
pub const Body = body_mod.Body;
pub const BodyLock = body_mod.Lock;
pub const Transform = body_mod.Transform;
pub const MotionType = body_mod.MotionType;
pub const MotionQuality = body_mod.MotionQuality;
pub const BodyType = body_mod.BodyType;
pub const Activation = body_mod.Activation;
pub const AllowedDofs = body_mod.AllowedDofs;
pub const OverrideMassProperties = body_mod.OverrideMassProperties;

pub const ObjectLayer = system_mod.ObjectLayer;
pub const BroadPhaseLayer = system_mod.BroadPhaseLayer;
pub const Layers = system_mod.Layers;
pub const layersFromType = system_mod.layersFromType;
pub const PhysicsSystem = system_mod.PhysicsSystem;
pub const JobSystem = system_mod.JobSystem;
pub const UpdateError = system_mod.UpdateError;
pub const contactListener = system_mod.contactListener;
pub const bodyActivationListener = system_mod.bodyActivationListener;
pub const ContactInfo = system_mod.ContactInfo;
pub const ContactManifold = system_mod.ContactManifold;
pub const ContactSettings = system_mod.ContactSettings;
pub const ContactValidateInfo = system_mod.ContactValidateInfo;
pub const SubShapeIdPair = system_mod.SubShapeIdPair;
pub const ValidateResult = system_mod.ValidateResult;
pub const contactPointsOn1 = system_mod.contactPointsOn1;
pub const contactPointsOn2 = system_mod.contactPointsOn2;

pub const PhysicsSettings = system_mod.PhysicsSettings;
pub const defaultPhysicsSettings = system_mod.defaultPhysicsSettings;
pub const BodyStats = system_mod.PhysicsSystem.BodyStats;
pub const StepListener = system_mod.StepListener;
pub const StepListenerContext = system_mod.StepListenerContext;
pub const CombineCallback = system_mod.CombineCallback;
pub const CombineInfo = system_mod.CombineInfo;

pub const BroadPhase = broadphase_mod.BroadPhase;
pub const BroadPhaseFilters = broadphase_mod.Filters;
pub const BroadPhaseCastHit = broadphase_mod.CastHit;
pub const OrientedBox = broadphase_mod.OrientedBox;

pub const Batch = batch_mod.Batch;
pub const AddBatch = batch_mod.AddBatch;

pub const GroupFilter = group_mod.GroupFilter;
pub const CollisionGroup = group_mod.CollisionGroup;
pub const collision_group_invalid = group_mod.invalid;
pub const max_sub_groups = group_mod.max_sub_groups;

pub const State = state_mod.State;
pub const StateRecorderState = state_mod.RecorderState;

pub const Scene = scene_mod.Scene;
pub const SceneConstraint = scene_mod.SceneConstraint;
pub const scene_world_body_index = scene_mod.world_body_index;

pub const SoftBodySharedSettings = softbody_mod.SharedSettings;
pub const SoftBodyDesc = softbody_mod.Desc;
pub const SoftBodyVertex = softbody_mod.Vertex;
pub const SoftBodyFace = softbody_mod.Face;
pub const SoftBodyEdge = softbody_mod.Edge;
pub const SoftBodyVolumeConstraint = softbody_mod.VolumeConstraint;
pub const SoftBodyInvBind = softbody_mod.InvBind;
pub const SoftBodySkinWeight = softbody_mod.SkinWeight;
pub const SoftBodySkinned = softbody_mod.Skinned;
pub const SoftBodyVertexAttributes = softbody_mod.VertexAttributes;
pub const SoftBodyVertexState = softbody_mod.VertexState;
pub const SoftBodyBendType = softbody_mod.BendType;
pub const SoftBodyLraType = softbody_mod.LraType;
pub const defaultSoftBodyVertexAttributes = softbody_mod.defaultVertexAttributes;
pub const createSoftBody = softbody_mod.create;
pub const createAndAddSoftBody = softbody_mod.createAndAdd;
pub const countSoftBodyVertexStates = softbody_mod.countVertexStates;
pub const getSoftBodyVertexStates = softbody_mod.getVertexStates;
pub const getSoftBodyNumIterations = softbody_mod.getNumIterations;
pub const setSoftBodyNumIterations = softbody_mod.setNumIterations;
pub const getSoftBodyPressure = softbody_mod.getPressure;
pub const setSoftBodyPressure = softbody_mod.setPressure;
pub const getSoftBodyUpdatePosition = softbody_mod.getUpdatePosition;
pub const setSoftBodyUpdatePosition = softbody_mod.setUpdatePosition;
pub const getSoftBodyFacesDoubleSided = softbody_mod.getFacesDoubleSided;
pub const setSoftBodyFacesDoubleSided = softbody_mod.setFacesDoubleSided;
pub const getSoftBodyVertexRadius = softbody_mod.getVertexRadius;
pub const setSoftBodyVertexRadius = softbody_mod.setVertexRadius;
pub const getSoftBodyVertexVelocity = softbody_mod.getVertexVelocity;
pub const setSoftBodyVertexVelocity = softbody_mod.setVertexVelocity;
pub const getSoftBodyVertexInvMass = softbody_mod.getVertexInvMass;
pub const setSoftBodyVertexInvMass = softbody_mod.setVertexInvMass;
pub const calculateSoftBodyMassAndInertia = softbody_mod.calculateMassAndInertia;
pub const getSoftBodyVolume = softbody_mod.getVolume;
pub const getSoftBodyLocalBounds = softbody_mod.getLocalBounds;
pub const skinSoftBodyVertices = softbody_mod.skinVertices;
pub const getSoftBodyEnableSkinConstraints = softbody_mod.getEnableSkinConstraints;
pub const setSoftBodyEnableSkinConstraints = softbody_mod.setEnableSkinConstraints;
pub const getSoftBodySkinnedMaxDistanceMultiplier = softbody_mod.getSkinnedMaxDistanceMultiplier;
pub const setSoftBodySkinnedMaxDistanceMultiplier = softbody_mod.setSkinnedMaxDistanceMultiplier;
pub const DebugRenderer = debug_mod.Renderer;
pub const DebugLine = debug_mod.Line;
pub const DebugTriangle = debug_mod.Triangle;
pub const DebugText = debug_mod.Text;
pub const DebugDrawBodiesSettings = debug_mod.DrawBodiesSettings;
pub const ShapeColor = debug_mod.ShapeColor;
pub const defaultDrawBodiesSettings = debug_mod.defaultDrawBodiesSettings;
pub const debugTextSlice = debug_mod.textSlice;
pub const debug_text_max_length = debug_mod.text_max_length;

pub const Queries = query_mod.Queries;
pub const QueryFilters = query_mod.Filters;
pub const RayCastHit = query_mod.RayCastHit;
pub const ShapeCastHit = query_mod.ShapeCastHit;
pub const CollideShapeHit = query_mod.CollideShapeHit;
pub const ExcludeBody = query_mod.ExcludeBody;
pub const OnlyObjectLayer = query_mod.OnlyObjectLayer;
pub const CollidePointHit = query_mod.CollidePointHit;
pub const QueryShapeFilter = query_mod.ShapeFilter;
pub const empty_sub_shape_id = query_mod.empty_sub_shape_id;
pub const HitAction = query_mod.HitAction;
pub const RayCastSettings = query_mod.RayCastSettings;
pub const CollideShapeSettings = query_mod.CollideShapeSettings;
pub const ShapeCastSettings = query_mod.ShapeCastSettings;
pub const ActiveEdgeMode = query_mod.ActiveEdgeMode;
pub const CollectFacesMode = query_mod.CollectFacesMode;

// Shape versus shape: two placed shapes, no physics system. @see query.zig.
pub const PlacedShape = query_mod.PlacedShape;
pub const ShapePair = query_mod.ShapePair;
pub const ShapeCastPair = query_mod.ShapeCastPair;
pub const collideShapeVsShapeClosest = query_mod.collideShapeVsShapeClosest;
pub const countShapeVsShapeOverlaps = query_mod.countShapeVsShapeOverlaps;
pub const collideShapeVsShapeAll = query_mod.collideShapeVsShapeAll;
pub const castShapeVsShapeClosest = query_mod.castShapeVsShapeClosest;
pub const countShapeVsShapeCastHits = query_mod.countShapeVsShapeCastHits;
pub const castShapeVsShapeAll = query_mod.castShapeVsShapeAll;

pub const Character = character_mod.Character;
pub const GroundState = character_mod.GroundState;
pub const BackFaceMode = character_mod.BackFaceMode;
pub const CharacterUpdateSettings = character_mod.UpdateSettings;
pub const defaultCharacterUpdateSettings = character_mod.defaultUpdateSettings;
pub const CharacterId = character_mod.CharacterId;
pub const invalid_character_id = character_mod.invalid_character_id;
pub const CharacterContact = character_mod.CharacterContact;
pub const CharacterContactSettings = character_mod.CharacterContactSettings;
pub const CharacterContactListener = character_mod.ContactListener;
pub const CharacterVsCharacterCollision = character_mod.CharacterVsCharacterCollision;
pub const RigidCharacter = character_mod.RigidCharacter;

pub const VehicleConstraint = vehicle_mod.VehicleConstraint;
pub const VehicleControllerKind = vehicle_mod.ControllerKind;
pub const VehicleCollisionTesterKind = vehicle_mod.CollisionTesterKind;
pub const VehicleTransmissionMode = vehicle_mod.TransmissionMode;
pub const VehicleCurvePoint = vehicle_mod.CurvePoint;
pub const VehicleCurveDesc = vehicle_mod.CurveDesc;
pub const default_vehicle_curve = vehicle_mod.default_curve;
pub const VehicleWheelDesc = vehicle_mod.WheelDesc;
pub const VehicleEngineDesc = vehicle_mod.EngineDesc;
pub const VehicleTransmissionDesc = vehicle_mod.TransmissionDesc;
pub const VehicleDifferentialDesc = vehicle_mod.DifferentialDesc;
pub const VehicleAntiRollBarDesc = vehicle_mod.AntiRollBarDesc;
pub const VehicleCollisionTesterDesc = vehicle_mod.CollisionTesterDesc;
pub const VehicleMotorcycleDesc = vehicle_mod.MotorcycleDesc;
pub const defaultVehicleWheelDesc = vehicle_mod.defaultWheelDesc;
pub const defaultVehicleEngineDesc = vehicle_mod.defaultEngineDesc;
pub const defaultVehicleTransmissionDesc = vehicle_mod.defaultTransmissionDesc;
pub const defaultVehicleDifferentialDesc = vehicle_mod.defaultDifferentialDesc;
pub const defaultVehicleAntiRollBarDesc = vehicle_mod.defaultAntiRollBarDesc;
pub const defaultVehicleCollisionTesterDesc = vehicle_mod.defaultCollisionTesterDesc;
pub const defaultVehicleMotorcycleDesc = vehicle_mod.defaultMotorcycleDesc;
pub const Skeleton = ragdoll_mod.Skeleton;
pub const SkeletonPose = ragdoll_mod.SkeletonPose;
pub const RagdollSettings = ragdoll_mod.RagdollSettings;
pub const RagdollPartDesc = ragdoll_mod.RagdollPartDesc;
pub const RagdollConstraintDesc = ragdoll_mod.RagdollConstraintDesc;
pub const Ragdoll = ragdoll_mod.Ragdoll;
/// Hair, and the compute backend it runs on. There is no renderer here: the
/// backend is either Jolt's CPU fallback, which this package compiles, or a
/// table the host fills in over a device it already owns.
pub const ComputeSystem = hair_mod.ComputeSystem;
pub const ComputeBackend = hair_mod.ComputeBackend;
pub const ComputeInterface = hair_mod.Interface;
pub const ComputeBufferType = hair_mod.BufferType;
pub const ComputeMapMode = hair_mod.MapMode;
pub const ComputeBarrier = hair_mod.Barrier;
pub const isCpuComputeSupported = hair_mod.isCpuSupported;

pub const Hair = hair_mod.Hair;
pub const Groom = hair_mod.Groom;
pub const HairScalp = hair_mod.Scalp;
pub const HairVertex = hair_mod.Vertex;
pub const HairStrand = hair_mod.Strand;
pub const HairGradient = hair_mod.Gradient;
pub const HairSkinWeight = hair_mod.SkinWeight;
pub const HairMaterial = hair_mod.Material;
pub const defaultHairMaterial = hair_mod.defaultMaterial;

pub const Constraint = constraint_mod.Constraint;
pub const ConstraintSubType = constraint_mod.SubType;
pub const ConstraintSpace = constraint_mod.Space;
pub const MotorState = constraint_mod.MotorState;
pub const MotorSettings = constraint_mod.MotorSettings;
pub const SpringMode = constraint_mod.SpringMode;
pub const SpringSettings = constraint_mod.SpringSettings;
pub const SwingType = constraint_mod.SwingType;
pub const SixDofAxis = constraint_mod.SixDofAxis;
pub const PathRotationConstraintType = constraint_mod.PathRotationConstraintType;
pub const world_body = constraint_mod.world_body;
pub const constraintCount = constraint_mod.count;
pub const FixedConstraintDesc = constraint_mod.FixedDesc;
pub const PointConstraintDesc = constraint_mod.PointDesc;
pub const HingeConstraintDesc = constraint_mod.HingeDesc;
pub const SliderConstraintDesc = constraint_mod.SliderDesc;
pub const DistanceConstraintDesc = constraint_mod.DistanceDesc;
pub const ConeConstraintDesc = constraint_mod.ConeDesc;
pub const SwingTwistConstraintDesc = constraint_mod.SwingTwistDesc;
pub const SixDofConstraintDesc = constraint_mod.SixDofDesc;
pub const GearConstraintDesc = constraint_mod.GearDesc;
pub const RackAndPinionConstraintDesc = constraint_mod.RackAndPinionDesc;
pub const PulleyConstraintDesc = constraint_mod.PulleyDesc;
pub const PathConstraintDesc = constraint_mod.PathDesc;
pub const ConstraintPath = constraint_mod.Path;
pub const PathPoint = constraint_mod.PathPoint;
pub const PathFrame = constraint_mod.PathFrame;

/// Build options the C library was actually compiled with, so a consumer can
/// branch on them instead of assuming.
pub const options = @import("zjolt_options");

//=============================================================================
// Initialisation
//=============================================================================

/// A formatted line of Jolt diagnostic output.
pub const TraceFn = core.TraceFn;
pub const AssertFailedFn = core.AssertFailedFn;

pub const InitOptions = struct {
    /// Routes every Jolt allocation through this. Null keeps Jolt's own
    /// malloc/free.
    ///
    /// Process-wide, because Jolt's allocator is. It must outlive `deinit`.
    allocator: ?std.mem.Allocator = null,
    /// Receives Jolt's log output, already formatted. Null sends it to
    /// stderr — which is zjolt's fallback, not Jolt's: Jolt's own default
    /// trace function is a stub whose body is an assertion, so an unhooked
    /// Jolt aborts the first time it has something to report.
    trace: ?TraceFn = null,
    /// Called when a Jolt assertion fails; return true to break. Only
    /// reachable when `options.enable_asserts` is set, and every assertion
    /// reachable through this API has been turned into a returned error
    /// instead — so in practice this fires only on a zjolt bug.
    assert_failed: ?AssertFailedFn = null,
    /// Passed back to `trace` and `assert_failed`.
    hooks_user: ?*anyopaque = null,
};

/// Installs the allocator and hooks, creates Jolt's factory and registers its
/// types. Must be called before anything else, and is process-wide.
///
/// Returns `error.ConfigMismatch` if this wrapper and the C library were built
/// with different layout-affecting settings. That cannot happen through
/// `build.zig`, which derives both from one option set — it is the guard for
/// a hand-assembled link.
pub fn init(opts: InitOptions) Error!void {
    var bridged: core.Allocator = undefined;
    var desc: core.InitDesc = .{
        .allocator = null,
        .trace = opts.trace,
        .assert_failed = opts.assert_failed,
        .hooks_user = opts.hooks_user,
    };
    if (opts.allocator) |gpa| {
        bridged = memory_mod.bridge(gpa);
        desc.allocator = &bridged;
    }
    try error_mod.check(core.zjoltInitWithConfig(&desc, core.config_id));
}

/// Unregisters Jolt's types, destroys the factory, and gives Jolt its own
/// allocator back. Safe to call when not initialised.
///
/// Refuses, and does nothing but trace, while any handle is still alive — see
/// `liveHandleCount`. Restoring the allocator with handles outstanding would
/// make destroying them free through an allocator they were never allocated
/// from, so leaving the library up is the safer of the two wrong situations.
pub fn deinit() void {
    core.zjoltDeinit();
}

pub fn isInitialized() bool {
    return core.zjoltIsInitialized();
}

/// Owning handles currently alive, across every kind there is: physics
/// systems, job systems, characters, character contact listeners,
/// character-vs-character collisions, debug renderers, compute systems, hair,
/// vehicle constraints, and ragdolls.
///
/// `deinit` refuses while this is non-zero, so a shutdown path can assert on
/// it rather than discover the problem as heap corruption later. The full list
/// matters for exactly that reason: a shutdown that released only the kinds it
/// remembered has nothing else to go on.
///
/// Objects that are nothing but a Jolt reference count — shapes, materials,
/// group filters, constraints, skeletons, ragdoll settings and the rest — are
/// not counted; Jolt owns their counts. A `Ragdoll` is the exception, because
/// its handle is a struct this package owns rather than a tag on Jolt's: the
/// count moves when `RagdollSettings.createRagdoll` allocates it and when the
/// `release` that frees it drops the last reference, not on every `addRef`.
pub fn liveHandleCount() u32 {
    return core.zjoltLiveHandleCount();
}

//=============================================================================
// Versions
//=============================================================================

pub const Version = struct {
    major: u8,
    minor: u8,
    patch: u8,

    fn unpack(packed_value: u32) Version {
        return .{
            .major = @truncate(packed_value >> 16),
            .minor = @truncate(packed_value >> 8),
            .patch = @truncate(packed_value),
        };
    }

    pub fn format(self: Version, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{d}.{d}.{d}", .{ self.major, self.minor, self.patch });
    }
};

/// Version of these bindings.
pub fn version() Version {
    return Version.unpack(core.zjoltVersion());
}

/// Version of the vendored Jolt Physics library.
pub fn joltVersion() Version {
    return Version.unpack(core.zjoltJoltVersion());
}

//=============================================================================
// Tests
//=============================================================================

test {
    // Pull every module in so its own tests are discovered and run.
    _ = error_mod;
    _ = math_mod;
    _ = memory_mod;
    _ = material_mod;
    _ = shape_mod;
    _ = transformed_mod;
    _ = body_mod;
    _ = query_mod;
    _ = system_mod;
    _ = character_mod;
    _ = broadphase_mod;
    _ = batch_mod;
    _ = group_mod;
    _ = state_mod;
    _ = scene_mod;
    _ = softbody_mod;
    _ = debug_mod;
    _ = hair_mod;
    _ = constraint_mod;
    _ = @import("integration_test.zig");
    _ = @import("constraint_test.zig");
    _ = @import("ragdoll_test.zig");
    _ = @import("character_test.zig");
    _ = @import("vehicle_test.zig");
    _ = @import("state_test.zig");
    _ = @import("debug_test.zig");

    // Mechanical, and it has to be: it calls every entry point in the ABI
    // with nulls, discovered by reflection rather than listed.
    _ = @import("misuse_sweep_test.zig");

    // Zig analyses only what is reached, so a pub fn nobody calls is never
    // type-checked. This walks every public declaration and forces it.
    _ = @import("analysis_test.zig");

    // Test-only: this one @cImport-s the C header. Reached from a test block
    // and nowhere else, so a normal build never analyses it and the shipped
    // module stays translate-c-free.
    _ = @import("abi_check.zig");
}

test "the library reports the build the wrapper was compiled for" {
    // What abi_check.zig cannot see. It compares this wrapper against the
    // header as THIS build's preprocessor rendered it — which says nothing
    // about whether the library linked here was compiled with the same macros.
    // ZJoltReal and ZJoltObjectLayer change width with those macros, so a
    // library built with different ones describes different structs while
    // presenting an identical header. This is the check that catches it.
    var layout: core.AbiLayout = undefined;
    core.zjoltAbiLayout(&layout);

    try std.testing.expectEqual(@as(u32, @sizeOf(core.AbiLayout)), layout.layout_size);
    try std.testing.expectEqual(@as(u32, @sizeOf(core.Real)), layout.real_size);
    try std.testing.expectEqual(@as(u32, @sizeOf(core.ObjectLayer)), layout.object_layer_size);
    try std.testing.expectEqual(core.config_id, layout.config_id);
    try std.testing.expectEqual(core.zjoltConfigId(), layout.config_id);
    try std.testing.expectEqual(
        @as(u32, @intCast(core.zjoltDefaultAllocateAlignment())),
        layout.default_allocate_alignment,
    );

    // The options module and the C library must describe the same build.
    const double_bit = (layout.build_flags & core.build_flag_double_precision) != 0;
    try std.testing.expectEqual(options.double_precision, double_bit);
    const layer32_bit = (layout.build_flags & core.build_flag_object_layer_32) != 0;
    try std.testing.expectEqual(options.object_layer_bits == 32, layer32_bit);
    const asserts_bit = (layout.build_flags & core.build_flag_asserts_enabled) != 0;
    try std.testing.expectEqual(options.enable_asserts, asserts_bit);
    const deterministic_bit =
        (layout.build_flags & core.build_flag_cross_platform_deterministic) != 0;
    try std.testing.expectEqual(options.cross_platform_deterministic, deterministic_bit);
}

test "version reporting is wired up" {
    // Against the mirrored constants rather than literals: abi_check.zig
    // already ties those to the header's ZJOLT_VERSION_* macros, so this
    // completes the chain library -> Zig mirror -> header without a third
    // copy of the number that would need editing when a version is cut.
    const v = version();
    try std.testing.expectEqual(@as(u8, @intCast(core.version_major)), v.major);
    try std.testing.expectEqual(@as(u8, @intCast(core.version_minor)), v.minor);
    try std.testing.expectEqual(@as(u8, @intCast(core.version_patch)), v.patch);

    const jolt = joltVersion();
    try std.testing.expectEqual(@as(u8, 5), jolt.major);
    try std.testing.expectEqual(@as(u8, 6), jolt.minor);
}

test "result names are never empty" {
    inline for (@typeInfo(core.Result).@"enum".fields) |field| {
        const name = resultName(@enumFromInt(field.value));
        try std.testing.expect(name.len > 0);
    }
}
