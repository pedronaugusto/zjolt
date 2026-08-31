//! zjolt — Zig bindings for Jolt Physics.
//!
//! The package owns no policy: no fixed timestep, no entity system, no
//! renderer. A host drives it as `build shapes -> create bodies -> step ->
//! read back`. Every public symbol is re-exported here, grouped by domain
//! under the section banners below.
//!
//! ```zig
//! try zjolt.init(.{ .allocator = gpa });
//! const system = try zjolt.PhysicsSystem.init(.{ .layers = layers });
//! const ball = try system.bodies().createAndAdd(desc, .activate);
//! _ = try system.step(1.0 / 60.0, 1, jobs);
//! ```

const std = @import("std");

/// The raw C declarations, one namespace per public header — `core.body`,
/// `core.shape`, `core.constraint` and so on, mirroring `ffi/zjolt_*.h`.
///
/// The escape hatch: anything the typed surface does not wrap is reachable
/// here, ABI-checked. The module name is part of the path, so
/// `core.max_physics_jobs` is `c.core.max_physics_jobs`.
pub const c = @import("c.zig");

/// Shorthand for the module everything in this file happens to need.
const core = c.core;

const error_mod = @import("error.zig");
const math_mod = @import("math.zig");
const memory_mod = @import("memory.zig");
const material_mod = @import("material.zig");
const geometry_mod = @import("geometry.zig");
const scale_mod = @import("scale.zig");
const customshape_mod = @import("customshape.zig");
const reflect_mod = @import("reflect.zig");
const tree_mod = @import("tree.zig");
const collision_mod = @import("collision.zig");
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
const factory_mod = @import("factory.zig");
const stream_mod = @import("stream.zig");

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
pub const vec3_zero = math_mod.vec3_zero;
pub const rvec3_zero = math_mod.rvec3_zero;
pub const quat_identity = math_mod.quat_identity;
pub const mat44_identity = math_mod.mat44_identity;
pub const rmat44_identity = math_mod.rmat44_identity;
pub const gravity_earth = math_mod.gravity_earth;

// `Vec3`, `RVec3`, `Quat`, `Mat44`, `RMat44`, `AABox` and `MassProperties`
// carry their math API as methods, declared on the structs themselves in
// `src/c/core.zig` — re-exporting the type above is what exposes them here,
// with no per-function line needed.

pub const Shape = shape_mod.Shape;
pub const ShapeSubType = shape_mod.SubType;
pub const Plane = shape_mod.Plane;
pub const MeshBuildQuality = shape_mod.MeshBuildQuality;
pub const MutableCompound = shape_mod.MutableCompound;
pub const CompoundChild = shape_mod.CompoundChild;
pub const compoundChild = shape_mod.compoundChild;
pub const SubShapeId = core.SubShapeId;
pub const SubShapeIdCreator = shape_mod.SubShapeIdCreator;
pub const popSubShapeId = shape_mod.popSubShapeId;
pub const ShapeSupportMode = shape_mod.SupportMode;
pub const ShapeSupportBuffer = shape_mod.SupportBuffer;
pub const ShapeSupportFunction = shape_mod.SupportFunction;
pub const PolyhedronSubmergedVolumeCalculator = shape_mod.PolyhedronSubmergedVolumeCalculator;
pub const sortReverseAndStore = shape_mod.sortReverseAndStore;
pub const countAndSortTrues = shape_mod.countAndSortTrues;
pub const sub_shape_id_empty = shape_mod.sub_shape_id_empty;
pub const height_field_no_collision = shape_mod.height_field_no_collision;
pub const default_active_edge_cos_threshold_angle = shape_mod.default_active_edge_cos_threshold_angle;

pub const TransformedShape = transformed_mod.TransformedShape;
pub const TransformedShapeTransform = transformed_mod.Transform;
pub const TransformedShapeTriangleWalk = transformed_mod.TriangleWalk;

pub const PhysicsMaterial = material_mod.PhysicsMaterial;
pub const Color = material_mod.Color;
pub const MaterialVec4 = material_mod.Vec4;
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
pub const UnassignedBody = body_mod.UnassignedBody;
pub const getBodySequenceNumber = body_mod.getSequenceNumber;
pub const shapeMustBeStatic = body_mod.shapeMustBeStatic;

pub const ObjectLayer = system_mod.ObjectLayer;
pub const BroadPhaseLayer = system_mod.BroadPhaseLayer;
pub const Layers = system_mod.Layers;
pub const layersFromType = system_mod.layersFromType;
pub const layersFromInstance = system_mod.layersFromInstance;
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
pub const SimCollideHit = system_mod.SimCollideHit;
pub const SimCollideCollector = system_mod.SimCollideCollector;
pub const SimCollideShapeFilter = system_mod.SimCollideShapeFilter;
pub const SimCollideBodyVsBody = system_mod.SimCollideBodyVsBody;
pub const addSimCollideHit = system_mod.addSimCollideHit;
pub const simCollideDefault = system_mod.simCollideDefault;
pub const simCollideBodyVsBody = system_mod.simCollideBodyVsBody;
pub const reportNarrowPhaseStats = system_mod.reportNarrowPhaseStats;
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

/// A host-supplied byte stream, for saving and restoring through a caller's own
/// pack file, socket or compressor instead of the two-call buffer form `State`,
/// `Shape` and `Scene` also offer. @see `hostStream` to build one from a Zig
/// type, and `Scene.saveObjectStream` / `RagdollSettings.saveObjectStream` for
/// the one format that is not this library's own.
pub const Stream = stream_mod.Stream;
pub const hostStream = stream_mod.hostStream;
pub const ObjectStreamFormat = stream_mod.ObjectStreamFormat;
pub const StreamBufferWriter = stream_mod.BufferWriter;
pub const StreamBufferReader = stream_mod.BufferReader;
pub const StreamRewindableBuffer = stream_mod.RewindableBuffer;

pub const State = state_mod.State;
pub const StateRecorderState = state_mod.RecorderState;

pub const Scene = scene_mod.Scene;
pub const SceneConstraint = scene_mod.SceneConstraint;
pub const scene_world_body_index = scene_mod.world_body_index;

pub const SoftBodySharedSettings = softbody_mod.SharedSettings;
pub const CollideSoftBodyVertexIterator = softbody_mod.CollideSoftBodyVertexIterator;
pub const CollideSoftBodyVerticesVsTriangles = softbody_mod.CollideVsTriangles;
pub const SoftBodyCcdContact = softbody_mod.CcdContact;
pub const collideSoftBodyVertices = softbody_mod.collideSoftBodyVertices;
pub const resetSoftBodyVertexCollision = softbody_mod.resetVertexCollision;
pub const softBodyMinVertexIndex = softbody_mod.minVertexIndex;
pub const normalizeSoftBodySkinWeights = softbody_mod.normalizeSkinWeights;
pub const markSoftBodyVertexCcdContact = softbody_mod.markCcdContact;
pub const SoftBodyDesc = softbody_mod.Desc;
pub const SoftBodyVertex = softbody_mod.Vertex;
pub const SoftBodyFace = softbody_mod.Face;
pub const SoftBodyEdge = softbody_mod.Edge;
pub const SoftBodyVolumeConstraint = softbody_mod.VolumeConstraint;
pub const SoftBodyRodStretchShear = softbody_mod.RodStretchShear;
pub const SoftBodyRodBendTwist = softbody_mod.RodBendTwist;
pub const SoftBodyRodState = softbody_mod.RodState;
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
pub const createSoftBodyWithId = softbody_mod.createWithId;
pub const createAndAddSoftBodyWithId = softbody_mod.createAndAddWithId;
pub const getSoftBodySharedSettings = softbody_mod.getSharedSettings;
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
pub const countSoftBodyRodStates = softbody_mod.countRodStates;
pub const getSoftBodyRodStates = softbody_mod.getRodStates;
pub const getSoftBodyFaceIndex = softbody_mod.getFaceIndex;
pub const customUpdateSoftBody = softbody_mod.customUpdate;
pub const SoftBodyValidateResult = softbody_mod.ValidateResult;
pub const SoftBodyContactSettings = softbody_mod.ContactSettings;
pub const SoftBodyVertexContact = softbody_mod.VertexContact;
pub const SoftBodyContactListener = softbody_mod.ContactListener;
pub const SoftBodyManifold = softbody_mod.Manifold;
pub const softBodyContactListener = softbody_mod.contactListener;
pub const setSoftBodyContactListener = softbody_mod.setContactListener;
pub const DebugRenderer = debug_mod.Renderer;
pub const DebugBatchVertex = debug_mod.BatchVertex;
pub const DebugCullMode = debug_mod.CullMode;
pub const DebugBatchLOD = debug_mod.BatchLOD;
pub const DebugBatchGeometry = debug_mod.BatchGeometry;
pub const DebugBatchCallbacks = debug_mod.BatchCallbacks;
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
pub const CharacterBase = character_mod.CharacterBase;
pub const GroundState = character_mod.GroundState;
pub const BackFaceMode = character_mod.BackFaceMode;
pub const CharacterUpdateSettings = character_mod.UpdateSettings;
pub const defaultCharacterUpdateSettings = character_mod.defaultUpdateSettings;
pub const CharacterId = character_mod.CharacterId;
pub const invalid_character_id = character_mod.invalid_character_id;
pub const CharacterContact = character_mod.CharacterContact;
pub const CharacterContactSettings = character_mod.CharacterContactSettings;
pub const CharacterSupportingVolume = character_mod.SupportingVolume;
pub const CharacterCollisionHit = character_mod.CollisionHit;
pub const CharacterCollisionQuery = character_mod.CollisionQuery;
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
pub const VehicleTrackSide = vehicle_mod.TrackSide;
pub const VehicleTrackDesc = vehicle_mod.TrackDesc;
pub const VehicleMotorcycleLeanSpring = vehicle_mod.MotorcycleLeanSpring;
pub const VehicleWheelFilters = vehicle_mod.WheelFilters;
pub const VehicleStepContext = vehicle_mod.StepContext;
pub const VehicleStepCallback = vehicle_mod.StepCallback;
pub const VehicleCombineFrictionCallback = vehicle_mod.CombineFrictionCallback;
pub const VehicleTireImpulseInputs = vehicle_mod.TireImpulseInputs;
pub const VehicleTireMaxImpulseCallback = vehicle_mod.TireMaxImpulseCallback;
pub const VehicleGroundTestInput = vehicle_mod.GroundTestInput;
pub const VehicleGroundContact = vehicle_mod.GroundContact;
pub const VehicleCollisionTesterCallback = vehicle_mod.CollisionTesterCallback;
pub const Skeleton = ragdoll_mod.Skeleton;
pub const SkeletonPose = ragdoll_mod.SkeletonPose;
pub const TriangleSplitter = tree_mod.TriangleSplitter;
pub const Triangles = tree_mod.Triangles;
pub const TriangleRange = tree_mod.TriangleRange;
pub const AABBTreeBuilder = tree_mod.AABBTreeBuilder;
pub const AABBTreeBuilderStats = tree_mod.AABBTreeBuilderStats;
pub const AABBTreeBuffer = tree_mod.AABBTreeBuffer;
pub const AABBTreeNodeHeader = tree_mod.AABBTreeNodeHeader;
pub const AABBTreeTriangleHeader = tree_mod.AABBTreeTriangleHeader;
pub const FlushDenormalsGuard = memory_mod.FlushDenormalsGuard;
pub const MeasurementAggregate = memory_mod.MeasurementAggregate;
pub const ExternalProfilerStartFn = memory_mod.ExternalProfilerStartFn;
pub const ExternalProfilerEndFn = memory_mod.ExternalProfilerEndFn;
pub const setExternalProfilerHooks = memory_mod.setExternalProfilerHooks;
pub const clearExternalProfilerHooks = memory_mod.clearExternalProfilerHooks;
pub const FreeListBatch = memory_mod.Batch;
pub const addObjectToBatch = memory_mod.addObjectToBatch;
pub const invalid_object_index = memory_mod.invalid_object_index;
pub const IslandBuilder = tree_mod.IslandBuilder;
pub const IslandStats = tree_mod.IslandStats;
pub const SplitMasks = tree_mod.SplitMasks;
pub const num_splits = tree_mod.num_splits;
pub const non_parallel_split_index = tree_mod.non_parallel_split_index;
pub const trig = math_mod.trig;
pub const half_float = math_mod.half_float;
pub const Vector2 = math_mod.Vector2;
pub const Matrix2 = math_mod.Matrix2;
pub const Matrix3 = math_mod.Matrix3;
pub const findRoot = math_mod.findRoot;
pub const centerAngleAroundZero = math_mod.centerAngleAroundZero;
pub const rmat44ToMat44 = math_mod.rmat44ToMat44;
pub const reflect = reflect_mod;
pub const ConvexShapeCallbacks = customshape_mod.ConvexShapeCallbacks;
pub const CustomShapeCallbacks = customshape_mod.ShapeCallbacks;
pub const CustomShapeRayHit = customshape_mod.CustomShapeRayHit;
pub const CustomShapeChild = customshape_mod.CustomShapeChild;
pub const initCustomConvex = customshape_mod.initCustomConvex;
pub const initCustom = customshape_mod.initCustom;
pub const custom_shape_max_batch = customshape_mod.max_batch;
/// `ScaleHelpers` — the constants and free functions a shape's scale support
/// is built from. No receiver type unifies them, so they are namespaced
/// under the module itself rather than left flat.
pub const scale = scale_mod;
pub const CollideShapeHitFn = collision_mod.CollideShapeHitFn;
pub const ShapeCastHitFn = collision_mod.ShapeCastHitFn;
pub const isEdgeActive = collision_mod.isEdgeActive;
pub const fixNormal = collision_mod.fixNormal;
pub const TrianglesQuery = collision_mod.TrianglesQuery;
pub const TrianglesCast = collision_mod.TrianglesCast;
pub const CollideConvexVsTriangles = collision_mod.CollideConvexVsTriangles;
pub const CastConvexVsTriangles = collision_mod.CastConvexVsTriangles;
pub const CollideSphereVsTriangles = collision_mod.CollideSphereVsTriangles;
pub const CastSphereVsTriangles = collision_mod.CastSphereVsTriangles;
pub const collideShapeWithInternalEdgeRemoval = collision_mod.collideShapeWithInternalEdgeRemoval;
pub const ConvexSupport = geometry_mod.ConvexSupport;
pub const ConvexSupportAdapter = geometry_mod.ConvexSupportAdapter;
pub const GJK = geometry_mod.GJK;
pub const max_simplex_points = geometry_mod.max_simplex_points;
pub const Simplex = geometry_mod.Simplex;
pub const calculatePointAAndB = geometry_mod.calculatePointAAndB;
pub const EPA = geometry_mod.EPA;
pub const ConvexHullBuilder = geometry_mod.ConvexHullBuilder;
pub const ConvexHullBuilder2D = geometry_mod.ConvexHullBuilder2D;
pub const EPAConvexHullBuilder = geometry_mod.EPAConvexHullBuilder;
pub const EPATriangle = geometry_mod.EPATriangle;
pub const EPAEdge = geometry_mod.EPAEdge;
pub const IndexedTriangleNoMaterial = geometry_mod.IndexedTriangleNoMaterial;
pub const Ellipse = geometry_mod.Ellipse;
pub const originOutsideOfPlane = geometry_mod.originOutsideOfPlane;
pub const originOutsideOfTetrahedronPlanes = geometry_mod.originOutsideOfTetrahedronPlanes;
pub const previousVertexInLoop = geometry_mod.previousVertexInLoop;
pub const calculateNormalAndCentroid = geometry_mod.calculateNormalAndCentroid;
pub const calculateNormalAndCenter2D = geometry_mod.calculateNormalAndCenter2D;
pub const clipPolyVsPlane = geometry_mod.clipPolyVsPlane;
pub const clipPolyVsPoly = geometry_mod.clipPolyVsPoly;
pub const clipPolyVsEdge = geometry_mod.clipPolyVsEdge;
pub const clipPolyVsAABox = geometry_mod.clipPolyVsAABox;
pub const IndexifyTriangle = geometry_mod.IndexifyTriangle;
pub const default_vertex_weld_distance = geometry_mod.default_vertex_weld_distance;
pub const indexify = geometry_mod.indexify;
pub const deindexify = geometry_mod.deindexify;
pub const CustomConstraintDesc = constraint_mod.CustomDesc;
pub const CustomConstraintCallbacks = constraint_mod.CustomCallbacks;
pub const SolverBody = constraint_mod.SolverBody;
pub const SolverBodyPair = constraint_mod.SolverBodyPair;
pub const StateRecorder = constraint_mod.StateRecorder;
pub const ConstraintSettings = constraint_mod.ConstraintSettings;
pub const constraint_part = @import("constraint_part.zig");
pub const SkeletalAnimation = ragdoll_mod.SkeletalAnimation;
pub const SkeletonMapper = ragdoll_mod.SkeletonMapper;
pub const RagdollSettings = ragdoll_mod.RagdollSettings;
pub const RagdollPartDesc = ragdoll_mod.RagdollPartDesc;
pub const RagdollConstraintDesc = ragdoll_mod.RagdollConstraintDesc;
pub const RagdollAdditionalConstraint = ragdoll_mod.AdditionalConstraint;
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
pub const HairGridCell = hair_mod.GridCell;
pub const HairGridLocation = hair_mod.GridLocation;
pub const HairReadBack = hair_mod.ReadBack;
pub const Groom = hair_mod.Groom;
pub const HairScalp = hair_mod.Scalp;
pub const HairVertex = hair_mod.Vertex;
pub const HairStrand = hair_mod.Strand;
pub const HairGradient = hair_mod.Gradient;
pub const HairSkinWeight = hair_mod.SkinWeight;
pub const HairMaterial = hair_mod.Material;
pub const HairVertexState = hair_mod.VertexState;
pub const HairInfo = hair_mod.Info;
pub const HairPlacement = hair_mod.Placement;
pub const defaultHairMaterial = hair_mod.defaultMaterial;
pub const sampleHairGradient = hair_mod.sampleGradient;
pub const hairBendComplianceAt = hair_mod.bendComplianceAt;

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
pub const constraintList = constraint_mod.list;
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
//
// Process-wide: `init` builds Jolt's factory and type registry; `deinit`
// tears them down. `liveHandleCount` counts every owning handle in the
// package — physics/job systems, characters and their listeners, debug
// renderers, compute systems, hair, vehicle constraints, ragdolls — and
// `deinit` refuses, tracing only, while any are alive, instead of freeing
// an allocator a live handle still depends on. Jolt's own reference-counted
// objects (shapes, materials, group filters, constraints, skeletons,
// ragdoll settings) are not counted. `Ragdoll` is the exception: its handle
// is a struct this package owns, so the count moves on `createRagdoll` and
// on the `release` that drops the last reference, not on every `addRef`.
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
    /// Called when a Jolt assertion fails; return true to break. Requires
    /// `options.enable_asserts`. Assertions reachable through this API are
    /// returned as errors instead, so this fires only on a zjolt defect.
    assert_failed: ?AssertFailedFn = null,
    /// Passed back to `trace` and `assert_failed`.
    hooks_user: ?*anyopaque = null,
};

/// Installs the allocator and hooks, creates Jolt's factory and registers its
/// types. Must be called before anything else.
///
/// Returns `error.ConfigMismatch` if this wrapper and the C library were
/// built with different layout-affecting settings; `build.zig` derives both
/// from one option set, so this only guards a hand-assembled link.
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
/// Refuses, and only traces, while any handle is alive — see the section
/// note above and `liveHandleCount`.
pub fn deinit() void {
    core.zjoltDeinit();
}

pub fn isInitialized() bool {
    return core.zjoltIsInitialized();
}

/// Owning handles currently alive. `deinit` refuses while this is non-zero;
/// see the section note above for what counts and what does not.
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

/// The instruction set Jolt's vectorised paths were COMPILED for, one flag
/// per Jolt `JPH_USE_*` macro. Not what the running CPU supports.
///
/// zjolt has no option for this. Jolt reads the compiler's own predefines,
/// so the lever is the Zig target's CPU model — `-Dcpu=x86_64_v3` and its
/// like — and a second lever could disagree with the first.
pub const CpuFeatures = packed struct(u32) {
    sse: bool = false,
    sse4_1: bool = false,
    sse4_2: bool = false,
    avx: bool = false,
    avx2: bool = false,
    avx512: bool = false,
    f16c: bool = false,
    lzcnt: bool = false,
    tzcnt: bool = false,
    fmadd: bool = false,
    neon: bool = false,
    _padding: u21 = 0,
};

pub fn cpuFeatures() CpuFeatures {
    return @bitCast(core.zjoltCpuFeatures());
}

//=============================================================================
// Tests
//=============================================================================

//=============================================================================
// Jolt's type registry
//
// Lookup only. Registering a type and constructing one by name need a C++
// vtable this ABI does not mirror (BINDING.md); every concrete type the
// registry could build has its own constructor here.
//=============================================================================

pub const factory = factory_mod;

//=============================================================================
// Debug rendering
//=============================================================================

pub const CastShadow = debug_mod.CastShadow;
pub const DrawMode = debug_mod.DrawMode;
pub const BodyDrawFilter = debug_mod.BodyDrawFilter;
pub const DebugDrawTarget = debug_mod.DrawTarget;
pub const DebugRecorder = debug_mod.Recorder;
pub const DebugPlayback = debug_mod.Playback;

//=============================================================================
// Host-supplied scratch allocation, and the simulation-time shape filter
//=============================================================================

pub const TempAllocatorKind = system_mod.TempAllocatorKind;
pub const TempAllocator = system_mod.TempAllocator;
pub const TempAllocatorStats = system_mod.TempAllocatorStats;
pub const hostTempAllocator = system_mod.hostTempAllocator;
pub const SimShapeFilter = system_mod.SimShapeFilter;
pub const simShapeFilter = system_mod.simShapeFilter;

//=============================================================================
// Selective state save and restore
//=============================================================================

pub const StateFilter = state_mod.StateFilter;
pub const stateFilter = state_mod.stateFilter;
pub const Divergence = state_mod.Divergence;
pub const compareState = state_mod.compareState;

//=============================================================================
// Collision response estimation
//=============================================================================

pub const CollisionEstimationResult = query_mod.CollisionEstimationResult;
pub const contact_points_capacity = query_mod.contact_points_capacity;
pub const pruneContactPoints = query_mod.pruneContactPoints;
pub const collideShapeVsShapePerLeafAll = query_mod.collideShapeVsShapePerLeafAll;

//=============================================================================
// Height fields, and characters that decide for themselves what they collide
// with
//=============================================================================

pub const auto_min_height_value = shape_mod.auto_min_height_value;
pub const auto_max_height_value = shape_mod.auto_max_height_value;
pub const CandidateVisitor = character_mod.CandidateVisitor;
pub const CharacterVsCharacterCollider = character_mod.CharacterVsCharacterCollider;

//=============================================================================
// Standalone geometry and matrix maths
//
// Free functions in Jolt, and free functions here: none of them needs a body,
// a shape handle or a physics system, and a caller reaching for a ray-triangle
// test should not have to build one. Most of this API lives as methods on
// `Vec3`/`RVec3`/`Quat`/`Mat44`/`RMat44`/`AABox`/`MassProperties` above, or as
// the `closest_point`/`ray` namespaces below; `orientedBoxOverlaps*` has no
// receiver type and stays flat.
//=============================================================================

pub const orientedBoxOverlapsAABox = math_mod.orientedBoxOverlapsAABox;
pub const orientedBoxOverlapsOrientedBox = math_mod.orientedBoxOverlapsOrientedBox;

/// Barycentric coordinates and closest points on a line, triangle or
/// tetrahedron. @see `src/math.zig`.
pub const closest_point = math_mod.closest_point;

/// Ray intersection primitives. @see `src/math.zig`.
pub const ray = math_mod.ray;

//=============================================================================
// Pure-Zig value-type helpers — no `ffi/` entry point, so no `zjolt.init`
// needed. Most of this API lives as methods on `Vec3`/`Quat`/`Mat44`/
// `RMat44`/`AABox` above; `Vec3x4`/`AABox4`/`RayInvDirection`/`IndexedTriangle`
// carry their own, declared in `src/math.zig`, the file that owns them.
//=============================================================================

pub const Vec3x4 = math_mod.Vec3x4;
pub const AABox4 = math_mod.AABox4;

pub const RayInvDirection = math_mod.RayInvDirection;
pub const planeSignedDistance = math_mod.planeSignedDistance;
pub const triangleCentroid = math_mod.triangleCentroid;
pub const IndexedTriangle = math_mod.IndexedTriangle;

test {
    // Pull every module in so its own tests are discovered and run.
    _ = error_mod;
    _ = math_mod;
    _ = memory_mod;
    _ = material_mod;
    _ = geometry_mod;
    _ = scale_mod;
    _ = customshape_mod;
    _ = reflect_mod;
    _ = tree_mod;
    _ = @import("tree_test.zig");
    _ = @import("vec_test.zig");
    _ = @import("reflect_test.zig");
    _ = @import("customshape_test.zig");
    _ = collision_mod;
    _ = @import("collision_test.zig");
    _ = @import("geometry_test.zig");
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
    _ = factory_mod;
    _ = @import("integration_test.zig");
    _ = @import("body_test.zig");
    _ = @import("constraint_test.zig");
    _ = @import("shape_test.zig");
    _ = @import("ragdoll_test.zig");
    _ = @import("character_test.zig");
    _ = @import("query_test.zig");
    _ = @import("vehicle_test.zig");
    _ = @import("state_test.zig");
    _ = @import("debug_test.zig");
    _ = @import("system_test.zig");
    _ = @import("softbody_test.zig");
    _ = @import("factory_test.zig");
    _ = @import("math_test.zig");

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

test "the reported instruction set is Jolt's own implication order" {
    // Jolt/Core/Core.h derives each JPH_USE_* from the wider one, so the set
    // is a chain and not a bag: AVX2 implies AVX, SSE4_2, SSE4_1, F16C,
    // LZCNT and TZCNT. A report that broke the chain would mean the macros
    // this reads are not the ones Jolt compiled its vector paths with.
    const cpu = cpuFeatures();

    if (cpu.avx512) try std.testing.expect(cpu.avx2);
    if (cpu.avx2) {
        try std.testing.expect(cpu.avx);
        try std.testing.expect(cpu.f16c);
        try std.testing.expect(cpu.lzcnt);
        try std.testing.expect(cpu.tzcnt);
    }

    // FMADD is the one link the chain alone does not carry. Core.h gates the
    // whole derivation on JPH_CROSS_PLATFORM_DETERMINISTIC being absent, a
    // fused multiply-add not rounding the way the separate operations do, so
    // AVX2 implies it only with that option off — and with it on, no target
    // may report it at all.
    if (options.cross_platform_deterministic) {
        try std.testing.expect(!cpu.fmadd);
    } else if (cpu.avx2) {
        try std.testing.expect(cpu.fmadd);
    }
    if (cpu.avx) try std.testing.expect(cpu.sse4_2);
    if (cpu.sse4_2) try std.testing.expect(cpu.sse4_1);
    if (cpu.sse4_1) try std.testing.expect(cpu.sse);

    // One of the two vector back ends is always compiled in: Vec4 and Quat
    // have an SSE form and a NEON form, and the scalar fallback is the third.
    // On the two architectures zjolt is built for, one of these holds.
    const arch = @import("builtin").cpu.arch;
    if (arch.isX86()) try std.testing.expect(cpu.sse);
    if (arch.isAARCH64()) try std.testing.expect(cpu.neon);

    // And nothing outside the eleven named bits is set.
    try std.testing.expectEqual(@as(u21, 0), cpu._padding);
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

test "every result has its own name, and none falls through" {
    // `len > 0` was the whole of this check, and `zjoltResultName`'s switch
    // ends in `return "unknown result"` — so a result the switch forgot read
    // as named. Two of twelve had been forgotten. Naming the fallthrough, and
    // requiring the names to be distinct, is what makes the check say what it
    // claims to.
    const fields = @typeInfo(core.Result).@"enum".fields;
    var seen: [fields.len][]const u8 = undefined;
    inline for (fields, 0..) |field, i| {
        const name = resultName(@enumFromInt(field.value));
        try std.testing.expect(name.len > 0);
        try std.testing.expect(!std.mem.eql(u8, name, "unknown result"));
        for (seen[0..i]) |earlier| {
            try std.testing.expect(!std.mem.eql(u8, name, earlier));
        }
        seen[i] = name;
    }
}

test "the published package version is the header's version" {
    // build.zig.zon is what a consumer fetches by; ZJOLT_VERSION_* is what
    // zjoltInit's handshake compares. Two numbers, one fact — build.zig hands
    // the .zon's own string through, so nothing here is a third copy.
    const v = version();
    var buf: [32]u8 = undefined;
    const printed = try std.fmt.bufPrint(&buf, "{f}", .{v});
    try std.testing.expectEqualStrings(options.package_version, printed);
}
