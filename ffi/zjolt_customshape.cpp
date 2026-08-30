//===----------------------------------------------------------------------===//
// zjolt — host-defined shapes.
//===----------------------------------------------------------------------===//

#include "zjolt_customshape.h"
#include "zjolt_internal.h"
#include "zjolt_query_internal.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRenderer.h>
#endif

namespace {

//===----------------------------------------------------------------------===//
// Conversions this file needs and no shared internal header already owns.
// The ones that DID have a second caller -- an AABox and a MassProperties --
// have moved to zjolt_internal.h, which is where a conversion belongs the
// moment two units want it.
//===----------------------------------------------------------------------===//

ZJoltRayCastSettings ToCRayCastSettings(const JPH::RayCastSettings &s) {
  return ZJoltRayCastSettings{
      s.mBackFaceModeTriangles == JPH::EBackFaceMode::CollideWithBackFaces
          ? ZJOLT_BACK_FACE_MODE_COLLIDE
          : ZJOLT_BACK_FACE_MODE_IGNORE,
      s.mBackFaceModeConvex == JPH::EBackFaceMode::CollideWithBackFaces
          ? ZJOLT_BACK_FACE_MODE_COLLIDE
          : ZJOLT_BACK_FACE_MODE_IGNORE,
      s.mTreatConvexAsSolid,
  };
}

const JPH::PhysicsMaterial *MaterialOrDefault(const ZJoltPhysicsMaterial *m) {
  if (m != nullptr) return zjolt::ToJolt(m);
  return JPH::PhysicsMaterial::sDefault;
}

/// Whether `value` fits the sub-shape id budget a host reported through
/// sub_shape_id_bits_recursive. SubShapeIDCreator::PushID asserts this
/// itself; checking first turns a bad host value into a dropped hit instead
/// of reaching that assert with asserts compiled out.
bool FitsInBits(uint32_t value, uint32_t bits) {
  return bits >= 32 || value < (uint32_t{1} << bits);
}

/// Draws any shape generically off the triangle walk every shape here already
/// has (GetTrianglesStart/Next) — the same source Jolt's own convex shapes
/// use for the identical purpose. Debug-only and off by default; kept simple
/// rather than exact.
#ifdef JPH_DEBUG_RENDERER
void DrawViaTriangles(const JPH::Shape &shape, JPH::DebugRenderer *inRenderer,
                      JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale,
                      JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe,
                      const JPH::PhysicsMaterial *inMaterial) {
  const JPH::Color color =
      inUseMaterialColors ? inMaterial->GetDebugColor() : inColor;
  JPH::Shape::GetTrianglesContext context;
  shape.GetTrianglesStart(context, JPH::AABox::sBiggest(), JPH::Vec3::sZero(),
                          JPH::Quat::sIdentity(), inScale);
  JPH::Float3 vertices[3 * 32];
  for (;;) {
    const int found = shape.GetTrianglesNext(context, 32, vertices, nullptr);
    if (found == 0) break;
    for (int i = 0; i < found; ++i) {
      const JPH::RVec3 v0 = inCenterOfMassTransform * JPH::Vec3::sLoadFloat3Unsafe(vertices[3 * i + 0]);
      const JPH::RVec3 v1 = inCenterOfMassTransform * JPH::Vec3::sLoadFloat3Unsafe(vertices[3 * i + 1]);
      const JPH::RVec3 v2 = inCenterOfMassTransform * JPH::Vec3::sLoadFloat3Unsafe(vertices[3 * i + 2]);
      if (inDrawWireframe) inRenderer->DrawWireTriangle(v0, v1, v2, color);
      else inRenderer->DrawTriangle(v0, v1, v2, color);
    }
  }
}
#endif  // JPH_DEBUG_RENDERER

//===----------------------------------------------------------------------===//
// Layer A — custom convex shape
//===----------------------------------------------------------------------===//

/// GJK's support-function seam. `direction`/the returned point are scaled by
/// `scale_` on the way in and out — the exact support-function identity for a
/// diagonal (including non-uniform, including mirrored) linear scale M:
/// support_scaled(d) = M * support(M * d). @see ffi/zjolt_customshape.h.
class CustomConvexSupport final : public JPH::ConvexShape::Support {
 public:
  CustomConvexSupport(const ZJoltConvexShapeCallbacks &callbacks, void *user, JPH::Vec3Arg scale)
      : callbacks_(callbacks), user_(user), scale_(scale) {}

  JPH::Vec3 GetSupport(JPH::Vec3Arg inDirection) const override {
    const JPH::Vec3 scaled_direction = scale_ * inDirection;
    const ZJoltVec3 point = callbacks_.support(user_, zjolt::ToC(scaled_direction));
    return scale_ * zjolt::ToJolt(point);
  }

  float GetConvexRadius() const override { return 0.0f; }

 private:
  ZJoltConvexShapeCallbacks callbacks_;
  void *user_;
  JPH::Vec3 scale_;
};

class CustomConvexShape final : public JPH::ConvexShape {
 public:
  CustomConvexShape(const ZJoltConvexShapeCallbacks &callbacks, void *user,
                    const JPH::PhysicsMaterial *material)
      : JPH::ConvexShape(JPH::EShapeSubType::UserConvex1, material),
        callbacks_(callbacks),
        user_(user) {}

  ~CustomConvexShape() override {
    if (callbacks_.destroy != nullptr) callbacks_.destroy(user_);
  }

  const Support *GetSupportFunction(ESupportMode, SupportBuffer &inBuffer,
                                    JPH::Vec3Arg inScale) const override {
    static_assert(sizeof(CustomConvexSupport) <= sizeof(SupportBuffer),
                  "CustomConvexSupport must fit ConvexShape::SupportBuffer");
    return new (&inBuffer) CustomConvexSupport(callbacks_, user_, inScale);
  }

  float GetInnerRadius() const override { return callbacks_.inner_radius(user_); }

  JPH::AABox GetLocalBounds() const override {
    ZJoltAABox box{};
    callbacks_.local_bounds(user_, &box);
    return zjolt::ToJolt(box);
  }

  JPH::MassProperties GetMassProperties() const override {
    ZJoltMassProperties props{};
    callbacks_.mass_properties(user_, &props);
    return zjolt::ToJolt(props);
  }

  float GetVolume() const override { return callbacks_.volume(user_); }

  // Approximated from supporting_face rather than a dedicated callback:
  // the face nearest inLocalSurfacePosition (used as the query direction,
  // the same way a shape centred near its own origin already treats surface
  // position and outward direction as interchangeable) gives a face normal;
  // with no face (or no callback) this falls back to the sphere convention,
  // Shape::GetSurfaceNormal's own simplest case.
  JPH::Vec3 GetSurfaceNormal(const JPH::SubShapeID &inSubShapeID,
                             JPH::Vec3Arg inLocalSurfacePosition) const override {
    JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");
    if (callbacks_.supporting_face != nullptr) {
      ZJoltVec3 verts[ZJOLT_SHAPE_MAX_SUPPORTING_FACE_VERTICES];
      const uint32_t count = callbacks_.supporting_face(
          user_, zjolt::ToC(inLocalSurfacePosition), ZJoltVec3{1, 1, 1}, verts,
          ZJOLT_SHAPE_MAX_SUPPORTING_FACE_VERTICES);
      if (count >= 3) {
        const JPH::Vec3 v0 = zjolt::ToJolt(verts[0]);
        const JPH::Vec3 v1 = zjolt::ToJolt(verts[1]);
        const JPH::Vec3 v2 = zjolt::ToJolt(verts[2]);
        JPH::Vec3 normal = (v1 - v0).Cross(v2 - v0);
        const float len = normal.Length();
        if (len > 1.0e-12f) {
          normal /= len;
          if (normal.Dot(inLocalSurfacePosition) < 0.0f) normal = -normal;
          return normal;
        }
      }
    }
    const float len = inLocalSurfacePosition.Length();
    return len != 0.0f ? inLocalSurfacePosition / len : JPH::Vec3::sAxisY();
  }

  void GetSupportingFace(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inDirection,
                         JPH::Vec3Arg inScale, JPH::Mat44Arg inCenterOfMassTransform,
                         SupportingFace &outVertices) const override {
    JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");
    if (callbacks_.supporting_face == nullptr) return;
    ZJoltVec3 verts[ZJOLT_SHAPE_MAX_SUPPORTING_FACE_VERTICES];
    const uint32_t count = callbacks_.supporting_face(
        user_, zjolt::ToC(inDirection), zjolt::ToC(inScale), verts,
        ZJOLT_SHAPE_MAX_SUPPORTING_FACE_VERTICES);
    const JPH::Mat44 transform = inCenterOfMassTransform.PreScaled(inScale);
    const uint32_t capacity = static_cast<uint32_t>(outVertices.capacity());
    const uint32_t clamped = count < capacity ? count : capacity;
    for (uint32_t i = 0; i < clamped; ++i)
      outVertices.push_back(transform * zjolt::ToJolt(verts[i]));
  }

  // No callback exists for this in ZJoltConvexShapeCallbacks: a custom
  // convex shape does not participate in soft-body collision.
  void CollideSoftBodyVertices(JPH::Mat44Arg, JPH::Vec3Arg,
                               const JPH::CollideSoftBodyVertexIterator &, JPH::uint,
                               int) const override {}

  JPH::Shape::Stats GetStats() const override { return JPH::Shape::Stats(sizeof(*this), 0); }

#ifdef JPH_DEBUG_RENDERER
  void Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform,
           JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors,
           bool inDrawWireframe) const override {
    DrawViaTriangles(*this, inRenderer, inCenterOfMassTransform, inScale, inColor,
                     inUseMaterialColors, inDrawWireframe, GetMaterial());
  }
#endif

 private:
  ZJoltConvexShapeCallbacks callbacks_;
  void *user_;
};

const ZJoltConvexShapeCallbacks &PlaceholderConvexCallbacks() {
  static const ZJoltConvexShapeCallbacks callbacks = [] {
    ZJoltConvexShapeCallbacks c{};
    c.support = [](void *, ZJoltVec3) -> ZJoltVec3 { return ZJoltVec3{0, 0, 0}; };
    c.inner_radius = [](const void *) -> float { return 0.0f; };
    c.local_bounds = [](const void *, ZJoltAABox *out) { *out = ZJoltAABox{}; };
    c.mass_properties = [](const void *, ZJoltMassProperties *out) {
      *out = ZJoltMassProperties{};
    };
    c.volume = [](const void *) -> float { return 0.0f; };
    return c;
  }();
  return callbacks;
}

JPH::Shape *ConstructPlaceholderConvex() {
  return new CustomConvexShape(PlaceholderConvexCallbacks(), nullptr, nullptr);
}

/// Registers the placeholder constructor for UserConvex1 once, on first use.
/// Lazy rather than namespace-scope static: JPH::ShapeFunctions::sRegistry
/// has no initialisation order this file can rely on.
///
/// Without this, restoring a zjoltShapeSave stream for a live custom shape
/// calls a null mConstruct — UserConvex1 is unregistered by default.
void EnsureCustomConvexRegistered() {
  static const bool registered = [] {
    JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JPH::EShapeSubType::UserConvex1);
    f.mConstruct = &ConstructPlaceholderConvex;
    f.mColor = JPH::Color::sGrey;
    return true;
  }();
  (void)registered;
}

//===----------------------------------------------------------------------===//
// Layer B — general custom shape
//===----------------------------------------------------------------------===//

class CustomShape final : public JPH::Shape {
 public:
  CustomShape(const ZJoltShapeCallbacks &callbacks, void *user)
      : JPH::Shape(JPH::EShapeType::User1, JPH::EShapeSubType::User1),
        callbacks_(callbacks),
        user_(user) {}

  ~CustomShape() override {
    if (callbacks_.destroy != nullptr) callbacks_.destroy(user_);
  }

  uint32_t Bits() const { return callbacks_.sub_shape_id_bits_recursive(user_); }

  bool MustBeStatic() const override {
    return callbacks_.must_be_static != nullptr ? callbacks_.must_be_static(user_)
                                                 : JPH::Shape::MustBeStatic();
  }

  JPH::Vec3 GetCenterOfMass() const override {
    return callbacks_.center_of_mass != nullptr
               ? zjolt::ToJolt(callbacks_.center_of_mass(user_))
               : JPH::Shape::GetCenterOfMass();
  }

  JPH::AABox GetLocalBounds() const override {
    ZJoltAABox box{};
    callbacks_.local_bounds(user_, &box);
    return zjolt::ToJolt(box);
  }

  uint32_t GetSubShapeIDBitsRecursive() const override { return Bits(); }

  JPH::AABox GetWorldSpaceBounds(JPH::Mat44Arg inCenterOfMassTransform,
                                 JPH::Vec3Arg inScale) const override {
    if (callbacks_.world_space_bounds == nullptr)
      return JPH::Shape::GetWorldSpaceBounds(inCenterOfMassTransform, inScale);
    const ZJoltMat44 transform = zjolt::ToC(inCenterOfMassTransform);
    ZJoltAABox box{};
    callbacks_.world_space_bounds(user_, &transform, zjolt::ToC(inScale), &box);
    return zjolt::ToJolt(box);
  }

  float GetInnerRadius() const override { return callbacks_.inner_radius(user_); }

  JPH::MassProperties GetMassProperties() const override {
    ZJoltMassProperties props{};
    callbacks_.mass_properties(user_, &props);
    return zjolt::ToJolt(props);
  }

  const JPH::PhysicsMaterial *GetMaterial(const JPH::SubShapeID &inSubShapeID) const override {
    return MaterialOrDefault(callbacks_.get_material(user_, zjolt::ToC(inSubShapeID)));
  }

  JPH::Vec3 GetSurfaceNormal(const JPH::SubShapeID &inSubShapeID,
                             JPH::Vec3Arg inLocalSurfacePosition) const override {
    ZJoltVec3 normal{};
    callbacks_.surface_normal(user_, zjolt::ToC(inSubShapeID),
                              zjolt::ToC(inLocalSurfacePosition), &normal);
    return zjolt::ToJolt(normal);
  }

  void GetSupportingFace(const JPH::SubShapeID &inSubShapeID, JPH::Vec3Arg inDirection,
                         JPH::Vec3Arg inScale, JPH::Mat44Arg inCenterOfMassTransform,
                         SupportingFace &outVertices) const override {
    if (callbacks_.supporting_face == nullptr) {
      JPH::Shape::GetSupportingFace(inSubShapeID, inDirection, inScale,
                                    inCenterOfMassTransform, outVertices);
      return;
    }
    ZJoltVec3 verts[ZJOLT_SHAPE_MAX_SUPPORTING_FACE_VERTICES];
    const uint32_t count = callbacks_.supporting_face(
        user_, zjolt::ToC(inSubShapeID), zjolt::ToC(inDirection), zjolt::ToC(inScale),
        verts, ZJOLT_SHAPE_MAX_SUPPORTING_FACE_VERTICES);
    const JPH::Mat44 transform = inCenterOfMassTransform.PreScaled(inScale);
    const uint32_t capacity = static_cast<uint32_t>(outVertices.capacity());
    const uint32_t clamped = count < capacity ? count : capacity;
    for (uint32_t i = 0; i < clamped; ++i)
      outVertices.push_back(transform * zjolt::ToJolt(verts[i]));
  }

  uint64_t GetSubShapeUserData(const JPH::SubShapeID &inSubShapeID) const override {
    return callbacks_.sub_shape_user_data != nullptr
               ? callbacks_.sub_shape_user_data(user_, zjolt::ToC(inSubShapeID))
               : JPH::Shape::GetSubShapeUserData(inSubShapeID);
  }

  void GetSubmergedVolume(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale,
                          const JPH::Plane &inSurface, float &outTotalVolume,
                          float &outSubmergedVolume, JPH::Vec3 &outCenterOfBuoyancy
                          JPH_IF_DEBUG_RENDERER(, JPH::RVec3Arg)) const override {
    const ZJoltMat44 transform = zjolt::ToC(inCenterOfMassTransform);
    const ZJoltPlane surface{zjolt::ToC(inSurface.GetNormal()), inSurface.GetConstant()};
    ZJoltVec3 center{};
    callbacks_.submerged_volume(user_, &transform, zjolt::ToC(inScale), &surface,
                                &outTotalVolume, &outSubmergedVolume, &center);
    outCenterOfBuoyancy = zjolt::ToJolt(center);
  }

  bool CastRay(const JPH::RayCast &inRay, const JPH::SubShapeIDCreator &inSubShapeIDCreator,
              JPH::RayCastResult &ioHit) const override {
    float out_fraction = 0.0f;
    ZJoltSubShapeId out_id = 0;
    const bool found = callbacks_.cast_ray_closest(
        user_, zjolt::ToC(inRay.mOrigin), zjolt::ToC(inRay.mDirection), ioHit.mFraction,
        &out_fraction, &out_id);
    if (!found || out_fraction >= ioHit.mFraction) return false;
    const uint32_t bits = Bits();
    if (!FitsInBits(out_id, bits)) return false;
    ioHit.mFraction = out_fraction;
    ioHit.mSubShapeID2 = inSubShapeIDCreator.PushID(out_id, bits).GetID();
    return true;
  }

  void CastRay(const JPH::RayCast &inRay, const JPH::RayCastSettings &inRayCastSettings,
              const JPH::SubShapeIDCreator &inSubShapeIDCreator,
              JPH::CastRayCollector &ioCollector,
              const JPH::ShapeFilter &inShapeFilter) const override {
    if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID())) return;
    const ZJoltRayCastSettings settings = ToCRayCastSettings(inRayCastSettings);
    ZJoltCustomShapeRayHit hits[ZJOLT_CUSTOM_SHAPE_MAX_BATCH];
    const uint32_t count =
        callbacks_.cast_ray_all(user_, zjolt::ToC(inRay.mOrigin), zjolt::ToC(inRay.mDirection),
                                &settings, hits, ZJOLT_CUSTOM_SHAPE_MAX_BATCH);
    const uint32_t clamped = count < ZJOLT_CUSTOM_SHAPE_MAX_BATCH ? count : ZJOLT_CUSTOM_SHAPE_MAX_BATCH;
    const uint32_t bits = Bits();
    const JPH::BodyID body_id = JPH::TransformedShape::sGetBodyID(ioCollector.GetContext());
    for (uint32_t i = 0; i < clamped; ++i) {
      if (hits[i].fraction >= ioCollector.GetEarlyOutFraction()) continue;
      if (!FitsInBits(hits[i].sub_shape_id, bits)) continue;
      JPH::RayCastResult hit;
      hit.mBodyID = body_id;
      hit.mFraction = hits[i].fraction;
      hit.mSubShapeID2 = inSubShapeIDCreator.PushID(hits[i].sub_shape_id, bits).GetID();
      ioCollector.AddHit(hit);
      if (ioCollector.ShouldEarlyOut()) break;
    }
  }

  void CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator &inSubShapeIDCreator,
                    JPH::CollidePointCollector &ioCollector,
                    const JPH::ShapeFilter &inShapeFilter) const override {
    if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID())) return;
    ZJoltSubShapeId ids[ZJOLT_CUSTOM_SHAPE_MAX_BATCH];
    const uint32_t count =
        callbacks_.collide_point(user_, zjolt::ToC(inPoint), ids, ZJOLT_CUSTOM_SHAPE_MAX_BATCH);
    const uint32_t clamped = count < ZJOLT_CUSTOM_SHAPE_MAX_BATCH ? count : ZJOLT_CUSTOM_SHAPE_MAX_BATCH;
    const uint32_t bits = Bits();
    const JPH::BodyID body_id = JPH::TransformedShape::sGetBodyID(ioCollector.GetContext());
    for (uint32_t i = 0; i < clamped; ++i) {
      if (!FitsInBits(ids[i], bits)) continue;
      JPH::CollidePointResult hit;
      hit.mBodyID = body_id;
      hit.mSubShapeID2 = inSubShapeIDCreator.PushID(ids[i], bits).GetID();
      ioCollector.AddHit(hit);
    }
  }

  void CollideSoftBodyVertices(JPH::Mat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale,
                               const JPH::CollideSoftBodyVertexIterator &inVertices,
                               JPH::uint inNumVertices, int inCollidingShapeIndex) const override {
    if (inNumVertices == 0) return;
    const JPH::Mat44 inverse = inCenterOfMassTransform.InversedRotationTranslation();
    const JPH::Vec3 inv_scale = inScale.Reciprocal();
    JPH::Array<ZJoltVec3> positions(inNumVertices);
    JPH::Array<float> inv_masses(inNumVertices);
    JPH::Array<float> penetrations(inNumVertices, -FLT_MAX);
    JPH::Array<ZJoltVec3> normals(inNumVertices, ZJoltVec3{0, 1, 0});

    JPH::uint i = 0;
    for (JPH::CollideSoftBodyVertexIterator v = inVertices, end = inVertices + inNumVertices;
        v != end; ++v, ++i) {
      positions[i] = zjolt::ToC(inv_scale * (inverse * v.GetPosition()));
      inv_masses[i] = v.GetInvMass();
    }

    callbacks_.collide_soft_body_vertices(user_, zjolt::ToC(inScale), positions.data(),
                                          inv_masses.data(), inNumVertices, penetrations.data(),
                                          normals.data());

    i = 0;
    for (JPH::CollideSoftBodyVertexIterator v = inVertices, end = inVertices + inNumVertices;
        v != end; ++v, ++i) {
      if (v.GetInvMass() <= 0.0f) continue;
      if (v.UpdatePenetration(penetrations[i])) {
        const JPH::Vec3 world_normal =
            inCenterOfMassTransform.Multiply3x3(zjolt::ToJolt(normals[i]))
                .NormalizedOr(JPH::Vec3::sAxisY());
        const JPH::Vec3 world_point = v.GetPosition() + penetrations[i] * world_normal;
        v.SetCollision(JPH::Plane::sFromPointAndNormal(world_point, world_normal),
                       inCollidingShapeIndex);
      }
    }
  }

  void CollectTransformedShapes(const JPH::AABox &inBox, JPH::Vec3Arg inPositionCOM,
                                JPH::QuatArg inRotation, JPH::Vec3Arg inScale,
                                const JPH::SubShapeIDCreator &inSubShapeIDCreator,
                                JPH::TransformedShapeCollector &ioCollector,
                                const JPH::ShapeFilter &inShapeFilter) const override {
    if (callbacks_.collect_transformed_shapes == nullptr) {
      JPH::Shape::CollectTransformedShapes(inBox, inPositionCOM, inRotation, inScale,
                                           inSubShapeIDCreator, ioCollector, inShapeFilter);
      return;
    }
    if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID())) return;
    const ZJoltAABox box = zjolt::ToC(inBox);
    const ZJoltVec3 position = zjolt::ToC(inPositionCOM);
    const ZJoltQuat rotation = zjolt::ToC(inRotation);
    ZJoltCustomShapeChild children[ZJOLT_CUSTOM_SHAPE_MAX_BATCH];
    const uint32_t count = callbacks_.collect_transformed_shapes(
        user_, &box, &position, &rotation, zjolt::ToC(inScale), children,
        ZJOLT_CUSTOM_SHAPE_MAX_BATCH);
    EmitChildren(children, count, inSubShapeIDCreator, ioCollector);
  }

  void TransformShape(JPH::Mat44Arg inCenterOfMassTransform,
                      JPH::TransformedShapeCollector &ioCollector) const override {
    if (callbacks_.transform_shape == nullptr) {
      JPH::Shape::TransformShape(inCenterOfMassTransform, ioCollector);
      return;
    }
    const ZJoltMat44 transform = zjolt::ToC(inCenterOfMassTransform);
    ZJoltCustomShapeChild children[ZJOLT_CUSTOM_SHAPE_MAX_BATCH];
    const uint32_t count =
        callbacks_.transform_shape(user_, &transform, children, ZJOLT_CUSTOM_SHAPE_MAX_BATCH);
    EmitChildren(children, count, JPH::SubShapeIDCreator(), ioCollector);
  }

  void GetTrianglesStart(GetTrianglesContext &ioContext, const JPH::AABox &inBox,
                         JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation,
                         JPH::Vec3Arg inScale) const override {
    const ZJoltAABox box = zjolt::ToC(inBox);
    const ZJoltVec3 position = zjolt::ToC(inPositionCOM);
    const ZJoltQuat rotation = zjolt::ToC(inRotation);
    const ZJoltVec3 scale = zjolt::ToC(inScale);
    callbacks_.get_triangles_start(user_, reinterpret_cast<ZJoltShapeTrianglesContext *>(&ioContext),
                                   &box, &position, &rotation, &scale);
  }

  int GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested,
                       JPH::Float3 *outTriangleVertices,
                       const JPH::PhysicsMaterial **outMaterials) const override {
    static_assert(sizeof(JPH::Float3) == sizeof(ZJoltVec3),
                  "Float3 must lay out the same as ZJoltVec3 for this cast");
    uint32_t out_count = 0;
    callbacks_.get_triangles_next(
        user_, reinterpret_cast<ZJoltShapeTrianglesContext *>(&ioContext),
        static_cast<uint32_t>(inMaxTrianglesRequested),
        reinterpret_cast<ZJoltVec3 *>(outTriangleVertices),
        reinterpret_cast<const ZJoltPhysicsMaterial **>(outMaterials), &out_count);
    return static_cast<int>(out_count);
  }

  JPH::Shape::Stats GetStats() const override {
    ZJoltShapeStats stats{};
    callbacks_.get_stats(user_, &stats);
    return JPH::Shape::Stats(static_cast<size_t>(stats.size_bytes),
                             static_cast<JPH::uint>(stats.num_triangles));
  }

  float GetVolume() const override { return callbacks_.volume(user_); }

  bool IsValidScale(JPH::Vec3Arg inScale) const override {
    return callbacks_.is_valid_scale != nullptr ? callbacks_.is_valid_scale(user_, zjolt::ToC(inScale))
                                                 : JPH::Shape::IsValidScale(inScale);
  }

  JPH::Vec3 MakeScaleValid(JPH::Vec3Arg inScale) const override {
    if (callbacks_.make_scale_valid == nullptr) return JPH::Shape::MakeScaleValid(inScale);
    ZJoltVec3 out{};
    callbacks_.make_scale_valid(user_, zjolt::ToC(inScale), &out);
    return zjolt::ToJolt(out);
  }

  void SaveBinaryState(JPH::StreamOut &inStream) const override {
    JPH::Shape::SaveBinaryState(inStream);

    JPH::Array<uint8_t> payload;
    if (callbacks_.save_binary_state != nullptr) {
      zjolt::MemoryCursor sizer{};
      ZJoltStream sizer_stream = zjolt::StreamOverMemory(&sizer);
      callbacks_.save_binary_state(user_, &sizer_stream);

      payload.resize(sizer.written);
      if (!payload.empty()) {
        zjolt::MemoryCursor writer{};
        writer.out = payload.data();
        writer.capacity = payload.size();
        ZJoltStream writer_stream = zjolt::StreamOverMemory(&writer);
        callbacks_.save_binary_state(user_, &writer_stream);
      }
    }

    uint8_t len_bytes[4];
    zjolt::WriteLE32(len_bytes, static_cast<uint32_t>(payload.size()));
    inStream.WriteBytes(len_bytes, sizeof(len_bytes));
    if (!payload.empty()) inStream.WriteBytes(payload.data(), payload.size());
  }

  // Length-prefixed skip, always — @see ffi/zjolt_customshape.h's
  // save_binary_state doc for why this never hands the bytes to a real
  // callback: an object reached through RestoreBinaryState is always the
  // placeholder, which never had one.
  void RestoreBinaryState(JPH::StreamIn &inStream) override {
    JPH::Shape::RestoreBinaryState(inStream);
    uint8_t len_bytes[4];
    inStream.ReadBytes(len_bytes, sizeof(len_bytes));
    if (inStream.IsEOF() || inStream.IsFailed()) return;
    const uint32_t len = zjolt::ReadLE32(len_bytes);
    if (len == 0) return;
    JPH::Array<uint8_t> payload(len);
    inStream.ReadBytes(payload.data(), len);
  }

  void SaveMaterialState(JPH::PhysicsMaterialList &outMaterials) const override {
    if (callbacks_.save_material_state == nullptr) return;
    const JPH::PhysicsMaterial *materials[ZJOLT_CUSTOM_SHAPE_MAX_BATCH];
    const uint32_t count = callbacks_.save_material_state(
        user_, reinterpret_cast<const ZJoltPhysicsMaterial **>(materials),
        ZJOLT_CUSTOM_SHAPE_MAX_BATCH);
    const uint32_t clamped = count < ZJOLT_CUSTOM_SHAPE_MAX_BATCH ? count : ZJOLT_CUSTOM_SHAPE_MAX_BATCH;
    outMaterials.reserve(clamped);
    for (uint32_t i = 0; i < clamped; ++i) outMaterials.push_back(materials[i]);
  }

  // Discarded: a restored custom shape is the inert placeholder described in
  // ffi/zjolt_customshape.h, with no way to know what these referenced
  // materials meant. Overridden (rather than left at Shape's default) so a
  // shape that DID save some does not hit Shape::RestoreMaterialState's own
  // JPH_ASSERT(inNumMaterials == 0) on the way back.
  void RestoreMaterialState(const JPH::PhysicsMaterialRefC *, JPH::uint) override {}

  void SaveSubShapeState(JPH::ShapeList &outSubShapes) const override {
    if (callbacks_.save_sub_shape_state == nullptr) return;
    const JPH::Shape *shapes[ZJOLT_CUSTOM_SHAPE_MAX_BATCH];
    const uint32_t count = callbacks_.save_sub_shape_state(
        user_, reinterpret_cast<const ZJoltShape **>(shapes), ZJOLT_CUSTOM_SHAPE_MAX_BATCH);
    const uint32_t clamped = count < ZJOLT_CUSTOM_SHAPE_MAX_BATCH ? count : ZJOLT_CUSTOM_SHAPE_MAX_BATCH;
    outSubShapes.reserve(clamped);
    for (uint32_t i = 0; i < clamped; ++i) outSubShapes.push_back(shapes[i]);
  }

  /// @see RestoreMaterialState.
  void RestoreSubShapeState(const JPH::ShapeRefC *, JPH::uint) override {}

#ifdef JPH_DEBUG_RENDERER
  void Draw(JPH::DebugRenderer *inRenderer, JPH::RMat44Arg inCenterOfMassTransform,
           JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors,
           bool inDrawWireframe) const override {
    DrawViaTriangles(*this, inRenderer, inCenterOfMassTransform, inScale, inColor,
                     inUseMaterialColors, inDrawWireframe, JPH::PhysicsMaterial::sDefault);
  }
#endif

 private:
  /// Shared by CollectTransformedShapes and TransformShape: both report a
  /// batch of already-placed children the same way.
  void EmitChildren(const ZJoltCustomShapeChild *children, uint32_t count,
                    const JPH::SubShapeIDCreator &creator,
                    JPH::TransformedShapeCollector &collector) const {
    const uint32_t clamped = count < ZJOLT_CUSTOM_SHAPE_MAX_BATCH ? count : ZJOLT_CUSTOM_SHAPE_MAX_BATCH;
    const uint32_t bits = Bits();
    const JPH::BodyID body_id = JPH::TransformedShape::sGetBodyID(collector.GetContext());
    for (uint32_t i = 0; i < clamped; ++i) {
      if (children[i].shape == nullptr) continue;
      if (!FitsInBits(children[i].sub_shape_id, bits)) continue;
      JPH::TransformedShape ts(JPH::RVec3(zjolt::ToJolt(children[i].position)),
                               zjolt::ToJoltRotation(children[i].rotation),
                               zjolt::ToJolt(children[i].shape), body_id,
                               creator.PushID(children[i].sub_shape_id, bits));
      ts.SetShapeScale(zjolt::ToJolt(children[i].scale));
      collector.AddHit(ts);
    }
  }

  ZJoltShapeCallbacks callbacks_;
  void *user_;
};

const ZJoltShapeCallbacks &PlaceholderShapeCallbacks() {
  static const ZJoltShapeCallbacks callbacks = [] {
    ZJoltShapeCallbacks c{};
    c.local_bounds = [](const void *, ZJoltAABox *out) { *out = ZJoltAABox{}; };
    c.sub_shape_id_bits_recursive = [](const void *) -> uint32_t { return 0; };
    c.inner_radius = [](const void *) -> float { return 0.0f; };
    c.mass_properties = [](const void *, ZJoltMassProperties *out) {
      *out = ZJoltMassProperties{};
    };
    c.get_material = [](const void *, ZJoltSubShapeId) -> const ZJoltPhysicsMaterial * {
      return nullptr;
    };
    c.surface_normal = [](const void *, ZJoltSubShapeId, ZJoltVec3, ZJoltVec3 *out) {
      *out = ZJoltVec3{0, 1, 0};
    };
    c.submerged_volume = [](const void *, const ZJoltMat44 *, ZJoltVec3, const ZJoltPlane *,
                            float *out_total, float *out_submerged, ZJoltVec3 *out_center) {
      *out_total = 0.0f;
      *out_submerged = 0.0f;
      *out_center = ZJoltVec3{0, 0, 0};
    };
    c.cast_ray_closest = [](const void *, ZJoltVec3, ZJoltVec3, float, float *,
                            ZJoltSubShapeId *) -> bool { return false; };
    c.cast_ray_all = [](const void *, ZJoltVec3, ZJoltVec3, const ZJoltRayCastSettings *,
                        ZJoltCustomShapeRayHit *, uint32_t) -> uint32_t { return 0; };
    c.collide_point = [](const void *, ZJoltVec3, ZJoltSubShapeId *, uint32_t) -> uint32_t {
      return 0;
    };
    c.collide_soft_body_vertices = [](const void *, ZJoltVec3, const ZJoltVec3 *, const float *,
                                      uint32_t count, float *out_penetration,
                                      ZJoltVec3 *out_normal) {
      for (uint32_t i = 0; i < count; ++i) {
        out_penetration[i] = -FLT_MAX;
        out_normal[i] = ZJoltVec3{0, 1, 0};
      }
    };
    c.get_triangles_start = [](const void *, ZJoltShapeTrianglesContext *, const ZJoltAABox *,
                               const ZJoltVec3 *, const ZJoltQuat *,
                               const ZJoltVec3 *) -> ZJoltResult { return ZJOLT_RESULT_OK; };
    c.get_triangles_next = [](const void *, ZJoltShapeTrianglesContext *, uint32_t, ZJoltVec3 *,
                              const ZJoltPhysicsMaterial **, uint32_t *out_count) -> ZJoltResult {
      *out_count = 0;
      return ZJOLT_RESULT_OK;
    };
    c.get_stats = [](const void *, ZJoltShapeStats *out) { *out = ZJoltShapeStats{}; };
    c.volume = [](const void *) -> float { return 0.0f; };
    return c;
  }();
  return callbacks;
}

JPH::Shape *ConstructPlaceholderShape() {
  return new CustomShape(PlaceholderShapeCallbacks(), nullptr);
}

/// @see EnsureCustomConvexRegistered; the same reasoning for User1.
void EnsureCustomShapeRegistered() {
  static const bool registered = [] {
    JPH::ShapeFunctions &f = JPH::ShapeFunctions::sGet(JPH::EShapeSubType::User1);
    f.mConstruct = &ConstructPlaceholderShape;
    f.mColor = JPH::Color::sGrey;
    return true;
  }();
  (void)registered;
}

}  // namespace

extern "C" {

ZJoltResult zjoltShapeCreateCustomConvex(const ZJoltConvexShapeCallbacks *callbacks, void *user,
                                         const ZJoltPhysicsMaterial *material, ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(callbacks, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (callbacks->support == nullptr || callbacks->inner_radius == nullptr ||
      callbacks->local_bounds == nullptr || callbacks->mass_properties == nullptr ||
      callbacks->volume == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a custom convex shape needs support, inner_radius, local_bounds, mass_properties "
        "and volume; Jolt has no default for any of them");
  }

  EnsureCustomConvexRegistered();
  CustomConvexShape *shape = new CustomConvexShape(*callbacks, user, zjolt::ToJolt(material));
  shape->AddRef();
  *out = const_cast<ZJoltShape *>(zjolt::ToC(static_cast<const JPH::Shape *>(shape)));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltShapeCreateCustom(const ZJoltShapeCallbacks *callbacks, void *user,
                                   ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(callbacks, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (callbacks->local_bounds == nullptr || callbacks->sub_shape_id_bits_recursive == nullptr ||
      callbacks->inner_radius == nullptr || callbacks->mass_properties == nullptr ||
      callbacks->get_material == nullptr || callbacks->surface_normal == nullptr ||
      callbacks->submerged_volume == nullptr || callbacks->cast_ray_closest == nullptr ||
      callbacks->cast_ray_all == nullptr || callbacks->collide_point == nullptr ||
      callbacks->collide_soft_body_vertices == nullptr || callbacks->get_triangles_start == nullptr ||
      callbacks->get_triangles_next == nullptr || callbacks->get_stats == nullptr ||
      callbacks->volume == nullptr) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a general custom shape needs local_bounds, sub_shape_id_bits_recursive, "
        "inner_radius, mass_properties, get_material, surface_normal, submerged_volume, "
        "cast_ray_closest, cast_ray_all, collide_point, collide_soft_body_vertices, "
        "get_triangles_start, get_triangles_next, get_stats and volume; Jolt has no "
        "default for any of them");
  }

  EnsureCustomShapeRegistered();
  CustomShape *shape = new CustomShape(*callbacks, user);
  shape->AddRef();
  *out = const_cast<ZJoltShape *>(zjolt::ToC(static_cast<const JPH::Shape *>(shape)));
  return ZJOLT_RESULT_OK;
}

}  // extern "C"
