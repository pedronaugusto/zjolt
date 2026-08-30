//! ZJolt C declarations for shape construction, introspection and
//! serialisation.
//!
//! Mirrors `ffi/zjolt_shape.h` exactly: a declaration belongs to the module
//! named after the header that declares it, so there is nothing to decide and
//! nothing to drift. `src/c.zig` lists every one of these and is what the ABI
//! cross-check and the misuse sweep walk.

const std = @import("std");
const core = @import("core.zig");
const transformed = @import("transformed.zig");

// Re-exported so a caller of this module sees one namespace rather than
// having to know which header a shared primitive came from.
pub const AABox = core.AABox;
pub const Mat44 = core.Mat44;
pub const MassProperties = core.MassProperties;
pub const PhysicsMaterial = core.PhysicsMaterial;
pub const Quat = core.Quat;
pub const Result = core.Result;
pub const Shape = core.Shape;
pub const ShapeSubType = core.ShapeSubType;
pub const SubShapeId = core.SubShapeId;
pub const Vec3 = core.Vec3;
pub const sub_shape_id_empty = core.sub_shape_id_empty;
pub const TransformedShape = transformed.TransformedShape;

/// Bytes `zjoltShapeSave` prepends to Jolt's own payload.
pub const shape_header_size: usize = 32;

/// Bytes `zjoltShapeSaveStream` prepends to Jolt's own payload.
pub const shape_stream_header_size: usize = 12;

/// How hard mesh building works to balance its tree.
pub const MeshBuildQuality = enum(c_int) {
    favor_runtime_performance = 0,
    favor_build_speed = 1,
};

pub const ShapeStats = extern struct {
    size_bytes: u64,
    num_triangles: u32,
};

/// One child of a compound shape, in the parent's local space.
pub const CompoundChild = extern struct {
    shape: ?*const Shape = null,
    position: Vec3 = .{ .x = 0, .y = 0, .z = 0 },
    rotation: Quat = .{ .x = 0, .y = 0, .z = 0, .w = 1 },
    user_data: u32 = 0,
};

pub extern fn zjoltShapeCreateBox(half_extent: *const Vec3, convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreateCapsule(half_height_of_cylinder: f32, radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreateConvexHull(points: [*]const Vec3, num_points: u32, max_convex_radius: f32, hull_tolerance: f32, max_error_convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreateMesh(vertices: [*]const Vec3, num_vertices: u32, indices: [*]const u32, num_triangles: u32, triangle_materials: ?[*]const u32, triangle_user_data: ?[*]const u32, materials: ?[*]const *const PhysicsMaterial, num_materials: u32, max_triangles_per_leaf: u32, active_edge_cos_threshold_angle: f32, build_quality: MeshBuildQuality, out: **Shape) Result;

pub extern fn zjoltShapeCreateScaled(inner: *const Shape, scale: *const Vec3, out: **Shape) Result;

pub extern fn zjoltShapeCreateRotatedTranslated(inner: *const Shape, translation: *const Vec3, rotation: *const Quat, out: **Shape) Result;

pub extern fn zjoltShapeCreateOffsetCenterOfMass(inner: *const Shape, offset: *const Vec3, out: **Shape) Result;

pub extern fn zjoltShapeGetInnerShape(shape: *const Shape, out: **const Shape) Result;

pub extern fn zjoltShapeAddRef(shape: *const Shape) void;

pub extern fn zjoltShapeRelease(shape: *const Shape) void;

pub extern fn zjoltShapeGetRefCount(shape: *const Shape) u32;

pub extern fn zjoltShapeGetUserData(shape: *const Shape) u64;

pub extern fn zjoltShapeSetUserData(shape: *Shape, user_data: u64) void;

pub extern fn zjoltShapeGetSubType(shape: *const Shape) ShapeSubType;

pub extern fn zjoltShapeGetVolume(shape: *const Shape) f32;

pub extern fn zjoltShapeGetCenterOfMass(shape: *const Shape, out: *Vec3) void;

pub extern fn zjoltShapeGetLocalBounds(shape: *const Shape, out: *AABox) void;

pub extern fn zjoltShapeGetMassProperties(shape: *const Shape, out: *MassProperties) void;

pub extern fn zjoltShapeGetStats(shape: *const Shape, out: *ShapeStats) void;

//===----------------------------------------------------------------------===//
// Convex-primitive dimension introspection
//===----------------------------------------------------------------------===//

pub extern fn zjoltShapeGetInnerRadius(shape: *const Shape) f32;

pub extern fn zjoltShapeGetDensity(shape: *const Shape, out_density: *f32) Result;

pub extern fn zjoltShapeSetDensity(shape: *Shape, density: f32) Result;

pub extern fn zjoltShapeGetRadius(shape: *const Shape, out_radius: *f32) Result;

pub extern fn zjoltShapeGetHalfExtent(shape: *const Shape, out_half_extent: *Vec3) Result;

pub extern fn zjoltShapeGetHalfHeight(shape: *const Shape, out_half_height: *f32) Result;

pub extern fn zjoltShapeGetHalfHeightOfCylinder(shape: *const Shape, out_half_height_of_cylinder: *f32) Result;

pub extern fn zjoltShapeGetTopRadius(shape: *const Shape, out_top_radius: *f32) Result;

pub extern fn zjoltShapeGetBottomRadius(shape: *const Shape, out_bottom_radius: *f32) Result;

pub extern fn zjoltShapeGetConvexRadius(shape: *const Shape, out_convex_radius: *f32) Result;

pub extern fn zjoltShapeGetNumFaces(shape: *const Shape, out_num_faces: *u32) Result;

pub extern fn zjoltShapeGetNumVerticesInFace(shape: *const Shape, face_index: u32, out_num_vertices: *u32) Result;

pub extern fn zjoltShapeGetPoints(shape: *const Shape, out_points: ?[*]Vec3, capacity: u32, out_count: *u32) Result;

/// A plane, as a unit normal and the signed distance from the origin along
/// it. Layout-identical to `ffi/zjolt_shape.h`'s `ZJoltPlane`.
pub const Plane = extern struct {
    normal: Vec3,
    constant: f32,
};

pub extern fn zjoltShapeGetPlanes(shape: *const Shape, out_planes: ?[*]Plane, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltShapeGetPlane(shape: *const Shape, out_plane: *Plane, out_half_extent: *f32) Result;

/// How `zjoltShapeGetSupportFunction` folds in a convex primitive's
/// rounding radius. Mirrors `ffi/zjolt_shape.h`'s `ZJoltShapeSupportMode`.
pub const ShapeSupportMode = enum(c_int) {
    exclude_convex_radius = 0,
    include_convex_radius = 1,
    default = 2,
};

/// Scratch space `zjoltShapeGetSupportFunction` places its support object
/// into. Matches `ConvexShape::SupportBuffer` byte for byte; never read
/// from Zig.
pub const ShapeSupportBuffer = extern struct {
    data: [4160]u8 align(16) = undefined,
};

pub const ShapeSupportFunction = opaque {};

pub extern fn zjoltShapeGetSupportFunction(shape: *const Shape, mode: ShapeSupportMode, buffer: *ShapeSupportBuffer, scale: ?*const Vec3, out: **const ShapeSupportFunction) Result;

pub extern fn zjoltShapeSupportFunctionGetSupport(support: *const ShapeSupportFunction, direction: *const Vec3, out: *Vec3) void;

pub extern fn zjoltShapeSupportFunctionGetConvexRadius(support: *const ShapeSupportFunction) f32;

pub extern fn zjoltShapeGetSubmergedVolume(shape: *const Shape, transform: *const Mat44, scale: ?*const Vec3, surface: *const Plane, out_total_volume: *f32, out_submerged_volume: *f32, out_center_of_buoyancy: *Vec3) Result;

pub const PolyhedronSubmergedVolumeCalculator = opaque {};

pub extern fn zjoltPolyhedronSubmergedVolumeCalculatorCreate(transform: *const Mat44, points: [*]const Vec3, num_points: u32, surface: *const Plane, out: **PolyhedronSubmergedVolumeCalculator) Result;

pub extern fn zjoltPolyhedronSubmergedVolumeCalculatorDestroy(calc: ?*PolyhedronSubmergedVolumeCalculator) void;

pub extern fn zjoltPolyhedronSubmergedVolumeCalculatorAreAllAbove(calc: ?*const PolyhedronSubmergedVolumeCalculator) bool;

pub extern fn zjoltPolyhedronSubmergedVolumeCalculatorAreAllBelow(calc: ?*const PolyhedronSubmergedVolumeCalculator) bool;

pub extern fn zjoltPolyhedronSubmergedVolumeCalculatorGetReferencePointIdx(calc: ?*const PolyhedronSubmergedVolumeCalculator) u32;

pub extern fn zjoltPolyhedronSubmergedVolumeCalculatorAddFace(calc: *PolyhedronSubmergedVolumeCalculator, idx1: u32, idx2: u32, idx3: u32) Result;

pub extern fn zjoltPolyhedronSubmergedVolumeCalculatorGetResult(calc: ?*const PolyhedronSubmergedVolumeCalculator, out_submerged_volume: *f32, out_center_of_buoyancy: *Vec3) void;

pub extern fn zjoltShapeSave(shape: *const Shape, buffer: ?[*]u8, capacity: usize, out_size: *usize) Result;

pub extern fn zjoltShapeSaveStream(shape: *const Shape, stream: *const core.Stream) Result;

pub extern fn zjoltShapeRestore(data: [*]const u8, size: usize, out: **Shape) Result;

pub extern fn zjoltShapeRestoreStream(stream: *const core.Stream, out: **Shape) Result;

pub extern fn zjoltShapeCreateCylinder(half_height: f32, radius: f32, convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreateTriangle(v1: *const Vec3, v2: *const Vec3, v3: *const Vec3, convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreateTaperedCapsule(half_height_of_tapered_cylinder: f32, top_radius: f32, bottom_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreateTaperedCylinder(half_height: f32, top_radius: f32, bottom_radius: f32, convex_radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreatePlane(normal: *const Vec3, constant: f32, half_extent: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;

pub extern fn zjoltShapeCreateEmpty(center_of_mass: ?*const Vec3, out: **Shape) Result;

pub extern fn zjoltShapeCreateHeightField(samples: [*]const f32, sample_count: u32, offset: ?*const Vec3, scale: ?*const Vec3, material_indices: ?[*]const u8, materials: ?[*]const *const PhysicsMaterial, num_materials: u32, materials_capacity: u32, block_size: u32, bits_per_sample: u32, min_height_value: f32, max_height_value: f32, active_edge_cos_threshold_angle: f32, out: **Shape) Result;

pub extern fn zjoltShapeCreateStaticCompound(children: [*]const CompoundChild, num_children: u32, out: **Shape) Result;

pub extern fn zjoltShapeCreateMutableCompound(children: [*]const CompoundChild, num_children: u32, out: **Shape) Result;

pub extern fn zjoltShapeCompoundGetNumChildren(shape: *const Shape) u32;

pub extern fn zjoltShapeCompoundGetChildUserData(shape: *const Shape, index: u32) u32;

pub extern fn zjoltShapeCompoundSetChildUserData(shape: *Shape, index: u32, user_data: u32) Result;

pub const SubShapeIdCreator = opaque {};

pub extern fn zjoltSubShapeIdCreatorCreate(out: **SubShapeIdCreator) Result;

pub extern fn zjoltSubShapeIdCreatorDestroy(creator: ?*SubShapeIdCreator) void;

pub extern fn zjoltSubShapeIdCreatorPushID(creator: *SubShapeIdCreator, value: u32, bits: u32) Result;

pub extern fn zjoltSubShapeIdCreatorGetID(creator: *const SubShapeIdCreator) SubShapeId;

pub extern fn zjoltSubShapeIdCreatorGetNumBitsWritten(creator: *const SubShapeIdCreator) u32;

pub extern fn zjoltSubShapeIdPopID(id: SubShapeId, bits: u32, out_value: *u32, out_remainder: *SubShapeId) Result;

pub extern fn zjoltShapeGetSubShapeIDFromIndex(shape: *const Shape, index: u32, out: *SubShapeId) Result;

pub extern fn zjoltShapeGetSubShapeIDFromIndexInto(shape: *const Shape, index: u32, creator: *SubShapeIdCreator) Result;

pub extern fn zjoltShapeGetSubShapeIndexFromID(shape: *const Shape, sub_shape_id: SubShapeId, out_index: *u32, out_remainder: *SubShapeId) Result;

pub extern fn zjoltShapeGetIntersectingSubShapes(shape: *const Shape, box: *const AABox, out_indices: ?[*]u32, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltShapeMutableCompoundAddChild(shape: *Shape, child: *const CompoundChild, out_index: *u32) Result;

pub extern fn zjoltShapeMutableCompoundRemoveChild(shape: *Shape, index: u32) Result;

pub extern fn zjoltShapeMutableCompoundMoveChild(shape: *Shape, index: u32, position: *const Vec3, rotation: ?*const Quat) Result;

pub extern fn zjoltShapeMutableCompoundReplaceChild(shape: *Shape, index: u32, new_shape: *const Shape, position: *const Vec3, rotation: ?*const Quat) Result;

pub extern fn zjoltShapeMutableCompoundAdjustCenterOfMass(shape: *Shape) Result;

pub extern fn zjoltShapeGetMaterial(shape: *const Shape, sub_shape_id: SubShapeId) ?*const PhysicsMaterial;

pub extern fn zjoltShapeSetMaterial(shape: *Shape, material: ?*const PhysicsMaterial) Result;

/// Vertices `zjoltShapeGetSupportingFace` can report in one call.
pub const shape_max_supporting_face_vertices: u32 = 32;

/// Fewest triangles `zjoltShapeGetTrianglesNext` accepts a request for.
pub const shape_min_triangles_requested: u32 = 32;

/// Opaque scratch space for one Shape-level triangle walk. Matches
/// `Shape::GetTrianglesContext` byte for byte; never read from Zig.
pub const ShapeTrianglesContext = extern struct {
    data: [4288]u8 align(16) = undefined,
};

pub extern fn zjoltShapeGetSubShapeIDBits(shape: *const Shape) u32;

pub extern fn zjoltShapeIsSubShapeIDValid(shape: *const Shape, sub_shape_id: SubShapeId) bool;

pub extern fn zjoltShapeGetSurfaceNormal(shape: *const Shape, sub_shape_id: SubShapeId, local_surface_position: *const Vec3, out_normal: *Vec3) void;

pub extern fn zjoltShapeGetSupportingFace(shape: *const Shape, sub_shape_id: SubShapeId, direction: *const Vec3, scale: ?*const Vec3, position: *const Vec3, rotation: *const Quat, out_vertices: [*]Vec3, out_count: *u32) Result;

pub extern fn zjoltShapeGetSubShapeTransformedShape(shape: *const Shape, sub_shape_id: SubShapeId, position: ?*const Vec3, rotation: ?*const Quat, scale: ?*const Vec3, out: **TransformedShape, out_remainder: ?*SubShapeId) Result;

pub extern fn zjoltShapeGetLeafShape(shape: *const Shape, sub_shape_id: SubShapeId, out_remainder: ?*SubShapeId) ?*const Shape;

pub extern fn zjoltShapeScaleShape(shape: *const Shape, scale: *const Vec3, out: **Shape) Result;

pub extern fn zjoltShapeIsValidScale(shape: *const Shape, scale: *const Vec3) bool;

pub extern fn zjoltShapeMakeScaleValid(shape: *const Shape, scale: *const Vec3, out_scale: *Vec3) void;

pub extern fn zjoltShapeGetTrianglesStart(shape: *const Shape, context: *ShapeTrianglesContext, box: *const AABox, position: ?*const Vec3, rotation: ?*const Quat, scale: ?*const Vec3) Result;

pub extern fn zjoltShapeGetTrianglesNext(shape: *const Shape, context: *ShapeTrianglesContext, max_triangles: u32, out_vertices: [*]Vec3, out_materials: ?[*]?*const PhysicsMaterial, out_count: *u32) Result;

pub extern fn zjoltShapeGetMaterialList(shape: *const Shape, out_materials: ?[*]?*const PhysicsMaterial, capacity: u32, out_count: *u32) Result;

pub extern fn zjoltShapeMeshGetMaterialIndex(shape: *const Shape, sub_shape_id: SubShapeId) u32;

pub extern fn zjoltShapeMeshGetTriangleUserData(shape: *const Shape, sub_shape_id: SubShapeId) u32;

pub extern fn zjoltShapeHeightFieldGetSampleCount(shape: *const Shape) u32;

pub extern fn zjoltShapeHeightFieldGetBlockSize(shape: *const Shape) u32;

pub extern fn zjoltShapeHeightFieldGetMinHeightValue(shape: *const Shape) f32;

pub extern fn zjoltShapeHeightFieldGetMaxHeightValue(shape: *const Shape) f32;

pub extern fn zjoltShapeHeightFieldGetPosition(shape: *const Shape, x: u32, y: u32, out_position: *Vec3) void;

pub extern fn zjoltShapeHeightFieldIsNoCollision(shape: *const Shape, x: u32, y: u32) bool;

pub extern fn zjoltShapeHeightFieldProjectOntoSurface(shape: *const Shape, local_position: *const Vec3, out_surface_position: *Vec3, out_sub_shape_id: *SubShapeId, out_found: *bool) Result;

pub extern fn zjoltShapeHeightFieldGetSubShapeCoordinates(shape: *const Shape, sub_shape_id: SubShapeId, out_x: *u32, out_y: *u32, out_triangle_index: *u32) Result;

pub extern fn zjoltShapeHeightFieldGetHeights(shape: *const Shape, x: u32, y: u32, size_x: u32, size_y: u32, out_heights: [*]f32, stride: u32) Result;

pub extern fn zjoltShapeHeightFieldSetHeights(shape: *Shape, x: u32, y: u32, size_x: u32, size_y: u32, heights: [*]const f32, stride: u32, active_edge_cos_threshold_angle: f32) Result;

pub extern fn zjoltShapeHeightFieldGetMaterials(shape: *const Shape, x: u32, y: u32, size_x: u32, size_y: u32, out_materials: [*]u8, stride: u32) Result;

pub extern fn zjoltShapeHeightFieldSetMaterials(shape: *Shape, x: u32, y: u32, size_x: u32, size_y: u32, materials: [*]const u8, stride: u32, material_list: ?[*]const *const PhysicsMaterial, num_materials: u32) Result;
