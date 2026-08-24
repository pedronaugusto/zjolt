//===----------------------------------------------------------------------===//
// zjolt — ray casts, shape casts, overlap and point tests.
//
// Jolt's query API is built on collectors: an object it calls once per hit, as
// it finds them. Everything in this file is one collector — `HitStream` — with
// two seams. `Project` turns Jolt's hit into the ABI's struct; `Sink` decides
// what becomes of it. The three forms the header offers are three sinks over
// one traversal:
//
//   KeepBest      the closest / deepest hit          -> *Closest
//   FillBuffer    the caller's buffer, and the count -> *All
//   ForwardToHost the host's callback                -> *Each
//
// Doing it that way is not tidiness. The alternative — one of Jolt's own
// collectors per form — is what this file used to be, and it materialised the
// whole result set inside Jolt before copying it out.
// JPH::AllHitCollisionCollector accumulates into an unbounded array, and a
// JPH::CollideShapeResult is around a kilobyte because it embeds two
// StaticArray<Vec3, 32> faces, sized that way even when mCollectFacesMode is
// NoFaces. Measured on an overlap query returning 201 hits: nine allocations
// and 559 KiB through the host's allocator per call, twice over for a
// count-then-fill pair. Every form here now allocates nothing at all, and the
// streaming form answers the same question in one traversal rather than two
// (201 body-filter calls against 402).
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

#include <cfloat>
#include <cstdio>

namespace {

//===----------------------------------------------------------------------===//
// The collector
//===----------------------------------------------------------------------===//

/// The one collector every query in this file runs on.
///
/// `Project` and `Sink` are both stored by value, so a query is a stack object
/// and nothing here allocates.
template <class Base, class CHit, class Project, class Sink>
class HitStream final : public Base {
 public:
  using JoltHit = typename Base::ResultType;

  HitStream(Project project, Sink sink)
      : project_(std::move(project)), sink_(std::move(sink)) {}

  void AddHit(const JoltHit &jolt_hit) override {
    CHit hit;
    project_(&hit, jolt_hit, *this);

    switch (sink_(hit, jolt_hit)) {
      case ZJOLT_HIT_ACTION_STOP:
        this->ForceEarlyOut();
        break;

      case ZJOLT_HIT_ACTION_NARROW: {
        // The guard is the entire reason narrowing is safe to offer.
        // UpdateEarlyOutFraction asserts the fraction never rises
        // (CollisionCollector.h:84), and Jolt does not promise that each hit
        // is better than the last — a convex shape reports its back face
        // AFTER its front face and further along the ray. Jolt's own
        // ClosestHitCollisionCollector guards the same call the same way.
        //
        // With assertions compiled out an increasing fraction is worse than a
        // failed assert: the narrow phase carries the same expectation as a
        // precondition (NarrowPhaseQuery.cpp:103, 333) and simply misbehaves.
        const float fraction = jolt_hit.GetEarlyOutFraction();
        if (fraction < this->GetEarlyOutFraction())
          this->UpdateEarlyOutFraction(fraction);
        break;
      }

      // CONTINUE, and anything a C caller passed that is not in the enum at
      // all. Doing nothing is the only answer that can neither lose a hit nor
      // break a precondition, which is what makes it the right default.
      default:
        break;
    }
  }

  const Sink &sink() const { return sink_; }

 private:
  Project project_;
  Sink sink_;
};

/// `Base` and `CHit` are named, the rest deduced.
template <class Base, class CHit, class Project, class Sink>
HitStream<Base, CHit, Project, Sink> MakeStream(Project project, Sink sink) {
  return HitStream<Base, CHit, Project, Sink>(std::move(project),
                                              std::move(sink));
}

//===----------------------------------------------------------------------===//
// Sinks
//===----------------------------------------------------------------------===//

/// Keeps the single best hit — nearest for a cast, deepest for an overlap —
/// which is what Jolt's early-out fraction already orders hits by.
///
/// This is JPH::ClosestHitCollisionCollector, rewritten as a sink so that the
/// closest-hit form shares the traversal with the other two instead of being a
/// second implementation that can drift.
template <class CHit>
struct KeepBest {
  CHit *out;
  bool *had_hit;
  float best = FLT_MAX;

  template <class JoltHit>
  ZJoltHitAction operator()(const CHit &hit, const JoltHit &jolt_hit) {
    const float fraction = jolt_hit.GetEarlyOutFraction();
    if (!*had_hit || fraction < best) {
      *out = hit;
      *had_hit = true;
      best = fraction;
    }
    return ZJOLT_HIT_ACTION_NARROW;
  }
};

/// Fills the caller's buffer and counts everything, including what did not
/// fit.
///
/// Counting past the capacity is the point: the count is the answer to the
/// size query, and it has to be true even when the buffer could not hold it.
/// That is what collapses the old two-call protocol from two traversals to
/// one.
template <class CHit>
struct FillBuffer {
  CHit *out;
  uint32_t capacity;
  uint32_t count = 0;

  template <class JoltHit>
  ZJoltHitAction operator()(const CHit &hit, const JoltHit &) {
    if (out != nullptr && count < capacity) out[count] = hit;
    ++count;
    return ZJOLT_HIT_ACTION_CONTINUE;
  }
};

/// Hands the hit to the host and does what it says.
///
/// The host's callback is a plain C function pointer and cannot throw across
/// this boundary; a language whose errors would unwind carries them out in
/// `user` instead. @see the note in zjolt_query.h.
template <class CHit, class Fn>
struct ForwardToHost {
  Fn on_hit;
  void *user;

  template <class JoltHit>
  ZJoltHitAction operator()(const CHit &hit, const JoltHit &) const {
    return on_hit(user, &hit);
  }
};

//===----------------------------------------------------------------------===//
// Projections
//===----------------------------------------------------------------------===//

/// A ray hit, with its surface normal resolved.
///
/// The normal is the reason this projection is handed the collector at all.
/// Jolt does not put it in RayCastResult; reaching it means asking the
/// TransformedShape the collector is holding, and that pointer is valid only
/// during AddHit and is deliberately not cleared afterwards
/// (CollisionCollector.h:73). Resolving it here is what lets the ABI report a
/// normal without exporting a pointer that is stale the moment it is useful.
/// The material at `sub_shape_id` on the shape a collector's context is
/// currently holding, or NULL when there is no context — which does not
/// happen when this is driven through NarrowPhaseQuery, but a defensive
/// default is cheaper than a precondition nobody would remember to check.
const JPH::PhysicsMaterial *ResolveMaterial(const JPH::TransformedShape *shape,
                                            const JPH::SubShapeID &sub_shape_id) {
  return shape != nullptr ? shape->GetMaterial(sub_shape_id) : nullptr;
}

struct ProjectRayHit {
  JPH::RRayCast ray;

  template <class Collector>
  void operator()(ZJoltRayCastHit *out, const JPH::RayCastResult &hit,
                  const Collector &collector) const {
    out->body = zjolt::ToC(hit.mBodyID);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->fraction = hit.mFraction;

    const JPH::TransformedShape *shape = collector.GetContext();
    out->normal = shape != nullptr
                      ? zjolt::ToC(shape->GetWorldSpaceSurfaceNormal(
                            hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction)))
                      : ZJoltVec3{0.0f, 0.0f, 0.0f};
    out->material = zjolt::ToC(ResolveMaterial(shape, hit.mSubShapeID2));
  }
};

struct ProjectShapeCastHit {
  template <class Collector>
  void operator()(ZJoltShapeCastHit *out, const JPH::ShapeCastResult &hit,
                  const Collector &collector) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->fraction = hit.mFraction;
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->is_back_face_hit = hit.mIsBackFaceHit;
    out->material =
        zjolt::ToC(ResolveMaterial(collector.GetContext(), hit.mSubShapeID2));
  }
};

struct ProjectCollideHit {
  template <class Collector>
  void operator()(ZJoltCollideShapeHit *out, const JPH::CollideShapeResult &hit,
                  const Collector &collector) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->material =
        zjolt::ToC(ResolveMaterial(collector.GetContext(), hit.mSubShapeID2));
  }
};

struct ProjectPointHit {
  template <class Collector>
  void operator()(ZJoltCollidePointHit *out, const JPH::CollidePointResult &hit,
                  const Collector &collector) const {
    out->body = zjolt::ToC(hit.mBodyID);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->material =
        zjolt::ToC(ResolveMaterial(collector.GetContext(), hit.mSubShapeID2));
  }
};

//===----------------------------------------------------------------------===//
// Argument plumbing
//===----------------------------------------------------------------------===//

JPH::EBackFaceMode ToJoltBackFaceMode(ZJoltBackFaceMode mode) {
  return mode == ZJOLT_BACK_FACE_MODE_COLLIDE
             ? JPH::EBackFaceMode::CollideWithBackFaces
             : JPH::EBackFaceMode::IgnoreBackFaces;
}

/// A NULL ZJoltRayCastSettings means Jolt's own defaults, which is what an
/// untouched RayCastSettings already is.
JPH::RayCastSettings MakeRayCastSettings(const ZJoltRayCastSettings *settings) {
  JPH::RayCastSettings out;
  if (settings == nullptr) return out;
  out.mBackFaceModeTriangles =
      ToJoltBackFaceMode(settings->back_face_mode_triangles);
  out.mBackFaceModeConvex = ToJoltBackFaceMode(settings->back_face_mode_convex);
  out.mTreatConvexAsSolid = settings->treat_convex_as_solid;
  return out;
}

/// Builds the shape cast Jolt wants from the world transform the ABI takes.
///
/// The distinction matters: Jolt casts the shape's CENTRE OF MASS, while a
/// host thinks in world transforms. sFromWorldTransform is the conversion, and
/// doing it here means an offset-centre-of-mass shape sweeps from where the
/// caller asked rather than from wherever its centre of mass happens to be.
JPH::RShapeCast MakeShapeCast(const JPH::Shape *shape, const ZJoltVec3 *scale,
                              const ZJoltRVec3 &position,
                              const ZJoltQuat &rotation,
                              const ZJoltVec3 &direction) {
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
  const JPH::RMat44 transform = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(rotation), zjolt::ToJoltR(position));
  return JPH::RShapeCast::sFromWorldTransform(shape, shape_scale, transform,
                                              zjolt::ToJolt(direction));
}

/// CollideShape works in centre-of-mass space, so the shape's own centre of
/// mass offset is folded in here rather than left for the caller to discover.
JPH::RMat44 MakeCollideTransform(const JPH::Shape *shape,
                                 JPH::Vec3Arg shape_scale,
                                 const ZJoltRVec3 &position,
                                 const ZJoltQuat &rotation) {
  const JPH::RMat44 world = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(rotation), zjolt::ToJoltR(position));
  return world.PreTranslated(shape_scale * shape->GetCenterOfMass());
}

/// The tail of a count-then-fill entry point: report the true count, and say
/// so when the buffer could not hold it.
ZJoltResult ReportCount(uint32_t count, const void *out_hits, uint32_t capacity,
                        uint32_t *out_count) {
  *out_count = count;
  if (out_hits == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  return ZJOLT_RESULT_OK;
}

/// A streaming query with nowhere to stream to is a caller mistake, not an
/// empty result — saying so is more useful than silently traversing and
/// discarding.
ZJoltResult MissingCallback() {
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "on_hit is NULL: a streaming query has nowhere to report hits");
}

//===----------------------------------------------------------------------===//
// Shape versus shape
//
// JPH::CollisionDispatch is the layer underneath every query above, and these
// hand it two shapes directly: no physics system, no broad phase, no body. The
// collector, the sinks and the count-then-fill tail are the ones already in
// this file. What is new is the argument plumbing, because there is no
// TransformedShape to resolve a material off and the transforms the dispatch
// takes are Mat44 rather than RMat44 — floats, relative to a base offset,
// rather than world-space Reals.
//===----------------------------------------------------------------------===//

/// What a shape can present to Jolt's dispatch table, as a set.
///
/// The table is a NumSubShapeTypes-squared array of function pointers that
/// each shape's sRegister fills in, and the cells nothing filled in assert.
/// Which cells a query reaches is decided by the LEAVES rather than by the
/// shape handed in: a compound and a decorated shape have no collision
/// function of their own, they re-dispatch whatever they hold
/// (CompoundShape.cpp:427, ScaledShape.cpp:229,
/// RotatedTranslatedShape.cpp:321), so a compound of meshes against a mesh
/// reaches exactly the cell a bare mesh would. Classifying the wrapper would
/// answer a question Jolt never asks.
enum LeafClasses : uint32_t {
  /// Registered against every other convex leaf (ConvexShape.cpp:556) and, one
  /// way round or the other, against every surface leaf below.
  kLeafConvex = 1u << 0,
  /// A surface with no inside: mesh, height field, plane, soft body. Each is
  /// registered against the convex leaves only (MeshShape.cpp:1291,
  /// HeightFieldShape.cpp:2742, PlaneShape.cpp:519, SoftBodyShape.cpp:340), so
  /// two of them together land in a cell nothing ever filled in.
  kLeafSurface = 1u << 1,
  /// A leaf this library cannot vouch for: the non-convex User slots, whose
  /// registration is known only to whoever defined them, and anything a Jolt
  /// upgrade adds that the switch below has not been taught.
  kLeafUnknown = 1u << 2,
};

/// How deep a shape tree this walks before giving up and refusing.
///
/// Shapes are built bottom-up and hold references downward, so there is no
/// cycle to guard against — this guards the stack. Nothing legitimate nests
/// anywhere near this deep, and refusing is the safe answer for anything that
/// does.
constexpr int kMaxShapeDepth = 64;

uint32_t CollectLeafClasses(const JPH::Shape *shape, int depth) {
  if (shape == nullptr || depth > kMaxShapeDepth) return kLeafUnknown;

  switch (shape->GetType()) {
    case JPH::EShapeType::Convex:
      return kLeafConvex;

    case JPH::EShapeType::Mesh:
    case JPH::EShapeType::HeightField:
    case JPH::EShapeType::SoftBody:
    case JPH::EShapeType::Plane:
      return kLeafSurface;

    // An empty shape has no leaves, and the functions registered for it do
    // nothing at all (EmptyShape.cpp:53). It pairs with anything.
    case JPH::EShapeType::Empty:
      return 0;

    case JPH::EShapeType::Decorated:
      return CollectLeafClasses(
          static_cast<const JPH::DecoratedShape *>(shape)->GetInnerShape(),
          depth + 1);

    case JPH::EShapeType::Compound: {
      uint32_t classes = 0;
      for (const JPH::CompoundShape::SubShape &sub :
           static_cast<const JPH::CompoundShape *>(shape)->GetSubShapes()) {
        classes |= CollectLeafClasses(sub.mShape, depth + 1);
      }
      return classes;
    }

    default:
      // EShapeType::User1..User4.
      return kLeafUnknown;
  }
}

/// Whether Jolt has a collision function for every leaf pair this would reach.
///
/// The same answer serves the collide table and the cast table: both are
/// filled in by the same registration loops, differing only in which direction
/// a surface-versus-convex pair is reversed through.
bool PairIsDispatchable(const JPH::Shape *shape1, const JPH::Shape *shape2) {
  const uint32_t classes1 = CollectLeafClasses(shape1, 0);
  const uint32_t classes2 = CollectLeafClasses(shape2, 0);
  if (((classes1 | classes2) & kLeafUnknown) != 0) return false;
  return (classes1 & kLeafSurface) == 0 || (classes2 & kLeafSurface) == 0;
}

const char *SubShapeTypeName(const JPH::Shape *shape) {
  const JPH::uint index = static_cast<JPH::uint>(shape->GetSubType());
  return index < JPH::NumSubShapeTypes ? JPH::sSubShapeTypeNames[index]
                                       : "unknown";
}

/// The two preconditions CollisionDispatch asserts on rather than reports.
///
/// Neither is in a signature; both are in the implementation. An unregistered
/// cell of the dispatch table is `JPH_ASSERT(false, "Unsupported shape pair")`
/// (CollisionDispatch.cpp:22) — and with assertions compiled out that cell is
/// a function that returns having reported nothing, so a caller would read a
/// confident "no overlap" for a test that never ran. A scale the shape cannot
/// take is asserted far deeper, once per support-function call
/// (SphereShape.cpp:55, CapsuleShape.cpp:154, CylinderShape.cpp:166), by which
/// point there is nothing left to report it to.
ZJoltResult CheckShapePair(const JPH::Shape *shape1, JPH::Vec3Arg scale1,
                           const JPH::Shape *shape2, JPH::Vec3Arg scale2) {
  if (!shape1->IsValidScale(scale1) || !shape2->IsValidScale(scale2)) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "a shape was given a scale it cannot take: a zero component, or a "
        "non-uniform scale on a shape that insists on a uniform one. "
        "zjoltShapeIsValidScale asks the same question ahead of time and "
        "zjoltShapeMakeScaleValid answers with the nearest scale that passes");
  }

  if (!PairIsDispatchable(shape1, shape2)) {
    char detail[320];
    std::snprintf(
        detail, sizeof(detail),
        "Jolt registers no collision function for %s versus %s. Two surfaces "
        "-- mesh, height field, plane, soft body -- have no inside for the "
        "narrow phase to separate, and a user-defined shape type is "
        "registered only by whoever defined it. Wrapping one in a compound or "
        "a decorated shape does not help: those re-dispatch what they hold, "
        "so one side has to be convex all the way down",
        SubShapeTypeName(shape1), SubShapeTypeName(shape2));
    return zjolt::SetError(ZJOLT_RESULT_UNSUPPORTED, detail);
  }

  return ZJOLT_RESULT_OK;
}

/// EPA asserts its tolerance is at least FLT_EPSILON
/// (EPAPenetrationDepth.h:154), and these settings structs are the first place
/// in this ABI where a caller can set it at all.
ZJoltResult CheckPenetrationTolerance(float tolerance) {
  // Written as an accept rather than a reject so that a NaN is refused too.
  if (tolerance >= FLT_EPSILON) return ZJOLT_RESULT_OK;
  return zjolt::SetError(
      ZJOLT_RESULT_INVALID_ARGUMENT,
      "penetration_tolerance is below FLT_EPSILON; Jolt asserts on that "
      "rather than honouring it, and a smaller one only buys iterations");
}

//===----------------------------------------------------------------------===//
// Settings
//===----------------------------------------------------------------------===//

JPH::EActiveEdgeMode ToJoltActiveEdgeMode(ZJoltActiveEdgeMode mode) {
  return mode == ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL
             ? JPH::EActiveEdgeMode::CollideWithAll
             : JPH::EActiveEdgeMode::CollideOnlyWithActive;
}

ZJoltActiveEdgeMode ToCActiveEdgeMode(JPH::EActiveEdgeMode mode) {
  return mode == JPH::EActiveEdgeMode::CollideWithAll
             ? ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_WITH_ALL
             : ZJOLT_ACTIVE_EDGE_MODE_COLLIDE_ONLY_WITH_ACTIVE;
}

JPH::ECollectFacesMode ToJoltCollectFacesMode(ZJoltCollectFacesMode mode) {
  return mode == ZJOLT_COLLECT_FACES_MODE_COLLECT_FACES
             ? JPH::ECollectFacesMode::CollectFaces
             : JPH::ECollectFacesMode::NoFaces;
}

ZJoltCollectFacesMode ToCCollectFacesMode(JPH::ECollectFacesMode mode) {
  return mode == JPH::ECollectFacesMode::CollectFaces
             ? ZJOLT_COLLECT_FACES_MODE_COLLECT_FACES
             : ZJOLT_COLLECT_FACES_MODE_NO_FACES;
}

ZJoltBackFaceMode ToCBackFaceMode(JPH::EBackFaceMode mode) {
  return mode == JPH::EBackFaceMode::CollideWithBackFaces
             ? ZJOLT_BACK_FACE_MODE_COLLIDE
             : ZJOLT_BACK_FACE_MODE_IGNORE;
}

/// A NULL settings pointer means Jolt's own defaults, which is what an
/// untouched CollideShapeSettings already is.
JPH::CollideShapeSettings MakeCollideShapeSettings(
    const ZJoltCollideShapeSettings *settings) {
  JPH::CollideShapeSettings out;
  if (settings == nullptr) return out;
  out.mActiveEdgeMode = ToJoltActiveEdgeMode(settings->active_edge_mode);
  out.mCollectFacesMode = ToJoltCollectFacesMode(settings->collect_faces_mode);
  out.mCollisionTolerance = settings->collision_tolerance;
  out.mPenetrationTolerance = settings->penetration_tolerance;
  out.mActiveEdgeMovementDirection =
      zjolt::ToJolt(settings->active_edge_movement_direction);
  out.mMaxSeparationDistance = settings->max_separation_distance;
  out.mBackFaceMode = ToJoltBackFaceMode(settings->back_face_mode);
  // mInternalEdgeRemovalVertexToleranceSq is deliberately not exposed: it is
  // read only by JPH::InternalEdgeRemovingCollector, which nothing in this ABI
  // wraps a query in, so a field for it would be a field that does nothing.
  return out;
}

JPH::ShapeCastSettings MakeShapeCastSettings(
    const ZJoltShapeCastSettings *settings) {
  JPH::ShapeCastSettings out;
  if (settings == nullptr) return out;
  out.mActiveEdgeMode = ToJoltActiveEdgeMode(settings->active_edge_mode);
  out.mCollectFacesMode = ToJoltCollectFacesMode(settings->collect_faces_mode);
  out.mCollisionTolerance = settings->collision_tolerance;
  out.mPenetrationTolerance = settings->penetration_tolerance;
  out.mActiveEdgeMovementDirection =
      zjolt::ToJolt(settings->active_edge_movement_direction);
  out.mExtraConvexRadius = settings->extra_convex_radius;
  out.mBackFaceModeTriangles =
      ToJoltBackFaceMode(settings->back_face_mode_triangles);
  out.mBackFaceModeConvex = ToJoltBackFaceMode(settings->back_face_mode_convex);
  out.mUseShrunkenShapeAndConvexRadius =
      settings->use_shrunken_shape_and_convex_radius;
  out.mReturnDeepestPoint = settings->return_deepest_point;
  return out;
}

//===----------------------------------------------------------------------===//
// Placement
//===----------------------------------------------------------------------===//

JPH::Vec3 ShapeScaleOr1(const ZJoltVec3 *scale) {
  return scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();
}

JPH::RVec3 BaseOffsetOrZero(const ZJoltRVec3 *base_offset) {
  return base_offset != nullptr ? zjolt::ToJoltR(*base_offset)
                                : JPH::RVec3::sZero();
}

zjolt::ShapeFilterAdapter MakeShapeFilter(const ZJoltShapeFilter *filter) {
  return zjolt::ShapeFilterAdapter(filter != nullptr ? *filter
                                                     : ZJoltShapeFilter{});
}

/// The transform CollisionDispatch wants: single precision, centre of mass,
/// relative to the caller's base offset.
///
/// Two conversions in one, and both are load-bearing. The caller gives a shape
/// transform while Jolt collides in centre-of-mass space, so the shape's own
/// centre of mass is folded in here — a shape built with an offset centre of
/// mass then sits where it was asked to rather than wherever its centre of
/// mass happens to be. And the dispatch takes Mat44, not RMat44, so a world
/// position has to lose precision somewhere; subtracting the base offset first
/// is what makes that loss happen near the query rather than near the origin.
JPH::Mat44 ToDispatchSpace(const JPH::Shape *shape, JPH::Vec3Arg shape_scale,
                           JPH::RMat44Arg world, JPH::RVec3Arg base_offset) {
  return world.PreTranslated(shape_scale * shape->GetCenterOfMass())
      .PostTranslated(-base_offset)
      .ToMat44();
}

/// The arguments all four shape-versus-shape entry points share, validated and
/// converted once.
struct ShapePair {
  const JPH::Shape *shape1;
  const JPH::Shape *shape2;
  JPH::Vec3 scale1;
  JPH::Vec3 scale2;
  JPH::RMat44 world1;
  JPH::RMat44 world2;
  JPH::RVec3 base_offset;

  JPH::Mat44 CenterOfMassTransform1() const {
    return ToDispatchSpace(shape1, scale1, world1, base_offset);
  }

  JPH::Mat44 CenterOfMassTransform2() const {
    return ToDispatchSpace(shape2, scale2, world2, base_offset);
  }
};

ZJoltResult PrepareShapePair(const ZJoltShape *shape1, const ZJoltVec3 *scale1,
                             const ZJoltRVec3 *position1,
                             const ZJoltQuat *rotation1,
                             const ZJoltShape *shape2, const ZJoltVec3 *scale2,
                             const ZJoltRVec3 *position2,
                             const ZJoltQuat *rotation2,
                             const ZJoltRVec3 *base_offset, ShapePair *out) {
  out->shape1 = zjolt::ToJolt(shape1);
  out->shape2 = zjolt::ToJolt(shape2);
  out->scale1 = ShapeScaleOr1(scale1);
  out->scale2 = ShapeScaleOr1(scale2);

  const ZJoltResult checked =
      CheckShapePair(out->shape1, out->scale1, out->shape2, out->scale2);
  if (checked != ZJOLT_RESULT_OK) return checked;

  out->world1 = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(*rotation1), zjolt::ToJoltR(*position1));
  out->world2 = JPH::RMat44::sRotationTranslation(
      zjolt::ToJoltRotation(*rotation2), zjolt::ToJoltR(*position2));
  out->base_offset = BaseOffsetOrZero(base_offset);
  return ZJOLT_RESULT_OK;
}

/// The shape cast the dispatch wants, in the same base-offset frame the
/// transforms above are in.
JPH::ShapeCast MakeDispatchShapeCast(const ShapePair &pair,
                                     const ZJoltVec3 &direction) {
  const JPH::RShapeCast world_cast = JPH::RShapeCast::sFromWorldTransform(
      pair.shape1, pair.scale1, pair.world1, zjolt::ToJolt(direction));
  return JPH::ShapeCast(world_cast.PostTranslated(-pair.base_offset));
}

//===----------------------------------------------------------------------===//
// Projections
//
// Unlike the projections above, these read nothing off the collector: a
// shape-versus-shape query has no TransformedShape for it to hold, and the
// shape a material comes off is the one the caller already passed. The body id
// is Jolt's own default, which is the invalid one, because there is no body.
//===----------------------------------------------------------------------===//

struct ProjectShapeVsShapeCollideHit {
  const JPH::Shape *shape2;

  template <class Collector>
  void operator()(ZJoltCollideShapeHit *out, const JPH::CollideShapeResult &hit,
                  const Collector &) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->material = zjolt::ToC(shape2->GetMaterial(hit.mSubShapeID2));
  }
};

struct ProjectShapeVsShapeCastHit {
  const JPH::Shape *shape2;

  template <class Collector>
  void operator()(ZJoltShapeCastHit *out, const JPH::ShapeCastResult &hit,
                  const Collector &) const {
    out->body = zjolt::ToC(hit.mBodyID2);
    out->sub_shape_id = zjolt::ToC(hit.mSubShapeID2);
    out->fraction = hit.mFraction;
    out->contact_point_on_1 = zjolt::ToC(hit.mContactPointOn1);
    out->contact_point_on_2 = zjolt::ToC(hit.mContactPointOn2);
    out->penetration_axis = zjolt::ToC(hit.mPenetrationAxis);
    out->penetration_depth = hit.mPenetrationDepth;
    out->is_back_face_hit = hit.mIsBackFaceHit;
    out->material = zjolt::ToC(shape2->GetMaterial(hit.mSubShapeID2));
  }
};

}  // namespace

extern "C" {

//===----------------------------------------------------------------------===//
// Ray casts
//===----------------------------------------------------------------------===//

void zjoltRayCastSettingsInit(ZJoltRayCastSettings *settings) {
  if (settings == nullptr) return;
  const JPH::RayCastSettings defaults;
  settings->back_face_mode_triangles =
      defaults.mBackFaceModeTriangles == JPH::EBackFaceMode::CollideWithBackFaces
          ? ZJOLT_BACK_FACE_MODE_COLLIDE
          : ZJOLT_BACK_FACE_MODE_IGNORE;
  settings->back_face_mode_convex =
      defaults.mBackFaceModeConvex == JPH::EBackFaceMode::CollideWithBackFaces
          ? ZJOLT_BACK_FACE_MODE_COLLIDE
          : ZJOLT_BACK_FACE_MODE_IGNORE;
  settings->treat_convex_as_solid = defaults.mTreatConvexAsSolid;
}

ZJoltResult zjoltCastRayClosest(const ZJoltPhysicsSystem *system,
                                const ZJoltRVec3 *origin,
                                const ZJoltVec3 *direction,
                                const ZJoltRayCastSettings *settings,
                                const ZJoltQueryFilters *filters,
                                ZJoltRayCastHit *out_hit, bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, origin, direction, out_hit, out_hit_any))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  bool had_hit = false;
  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray}, KeepBest<ZJoltRayCastHit>{out_hit, &had_hit});
  system->system.GetNarrowPhaseQuery().CastRay(
      ray, MakeRayCastSettings(settings), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastRayAll(const ZJoltPhysicsSystem *system,
                            const ZJoltRVec3 *origin,
                            const ZJoltVec3 *direction,
                            const ZJoltRayCastSettings *settings,
                            const ZJoltQueryFilters *filters,
                            ZJoltRayCastHit *out_hits, uint32_t capacity,
                            uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, origin, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray}, FillBuffer<ZJoltRayCastHit>{out_hits, capacity});
  system->system.GetNarrowPhaseQuery().CastRay(
      ray, MakeRayCastSettings(settings), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCastRayEach(const ZJoltPhysicsSystem *system,
                             const ZJoltRVec3 *origin,
                             const ZJoltVec3 *direction,
                             const ZJoltRayCastSettings *settings,
                             const ZJoltQueryFilters *filters,
                             ZJoltRayCastHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, origin, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();

  zjolt::QueryFilters adapters(filters);
  const JPH::RRayCast ray(zjolt::ToJoltR(*origin), zjolt::ToJolt(*direction));

  auto collector = MakeStream<JPH::CastRayCollector, ZJoltRayCastHit>(
      ProjectRayHit{ray},
      ForwardToHost<ZJoltRayCastHit, ZJoltRayCastHitFn>{on_hit, user});
  system->system.GetNarrowPhaseQuery().CastRay(
      ray, MakeRayCastSettings(settings), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Shape casts
//
// Contact points come back relative to a base offset of our choosing, and the
// caller's own `position` is the one to pick. Resolving them to absolute world
// space here would mean storing a world coordinate in a float, which in a
// double-precision build is exactly the precision this base-offset mechanism
// exists to avoid losing.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCastShapeClosest(const ZJoltPhysicsSystem *system,
                                  const ZJoltShape *shape,
                                  const ZJoltVec3 *scale,
                                  const ZJoltRVec3 *position,
                                  const ZJoltQuat *rotation,
                                  const ZJoltVec3 *direction,
                                  const ZJoltQueryFilters *filters,
                                  ZJoltShapeCastHit *out_hit,
                                  bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  bool had_hit = false;
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{}, KeepBest<ZJoltShapeCastHit>{out_hit, &had_hit});
  system->system.GetNarrowPhaseQuery().CastShape(
      cast, JPH::ShapeCastSettings(), zjolt::ToJoltR(*position), collector,
      adapters.broad_phase, adapters.object_layer, adapters.body,
      adapters.shape);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastShapeAll(const ZJoltPhysicsSystem *system,
                              const ZJoltShape *shape, const ZJoltVec3 *scale,
                              const ZJoltRVec3 *position,
                              const ZJoltQuat *rotation,
                              const ZJoltVec3 *direction,
                              const ZJoltQueryFilters *filters,
                              ZJoltShapeCastHit *out_hits, uint32_t capacity,
                              uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, direction, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{}, FillBuffer<ZJoltShapeCastHit>{out_hits, capacity});
  system->system.GetNarrowPhaseQuery().CastShape(
      cast, JPH::ShapeCastSettings(), zjolt::ToJoltR(*position), collector,
      adapters.broad_phase, adapters.object_layer, adapters.body,
      adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCastShapeEach(const ZJoltPhysicsSystem *system,
                               const ZJoltShape *shape, const ZJoltVec3 *scale,
                               const ZJoltRVec3 *position,
                               const ZJoltQuat *rotation,
                               const ZJoltVec3 *direction,
                               const ZJoltQueryFilters *filters,
                               ZJoltShapeCastHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation, direction))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();

  zjolt::QueryFilters adapters(filters);
  const JPH::RShapeCast cast = MakeShapeCast(zjolt::ToJolt(shape), scale,
                                             *position, *rotation, *direction);

  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeCastHit{},
      ForwardToHost<ZJoltShapeCastHit, ZJoltShapeCastHitFn>{on_hit, user});
  system->system.GetNarrowPhaseQuery().CastShape(
      cast, JPH::ShapeCastSettings(), zjolt::ToJoltR(*position), collector,
      adapters.broad_phase, adapters.object_layer, adapters.body,
      adapters.shape);

  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Overlap
//
// Same base-offset rule as the shape casts above: hits are relative to the
// caller's `position`, not resolved to an absolute world coordinate that a
// float cannot hold precisely.
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollideShapeClosest(const ZJoltPhysicsSystem *system,
                                     const ZJoltShape *shape,
                                     const ZJoltVec3 *scale,
                                     const ZJoltRVec3 *position,
                                     const ZJoltQuat *rotation,
                                     float max_separation_distance,
                                     const ZJoltQueryFilters *filters,
                                     ZJoltCollideShapeHit *out_hit,
                                     bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(system, shape, position, rotation, out_hit,
                      out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = max_separation_distance;

  bool had_hit = false;
  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{}, KeepBest<ZJoltCollideShapeHit>{out_hit, &had_hit});
  system->system.GetNarrowPhaseQuery().CollideShape(
      impl, shape_scale,
      MakeCollideTransform(impl, shape_scale, *position, *rotation), settings,
      zjolt::ToJoltR(*position), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCollideShapeAll(const ZJoltPhysicsSystem *system,
                                 const ZJoltShape *shape,
                                 const ZJoltVec3 *scale,
                                 const ZJoltRVec3 *position,
                                 const ZJoltQuat *rotation,
                                 float max_separation_distance,
                                 const ZJoltQueryFilters *filters,
                                 ZJoltCollideShapeHit *out_hits,
                                 uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, shape, position, rotation, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = max_separation_distance;

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{},
      FillBuffer<ZJoltCollideShapeHit>{out_hits, capacity});
  system->system.GetNarrowPhaseQuery().CollideShape(
      impl, shape_scale,
      MakeCollideTransform(impl, shape_scale, *position, *rotation), settings,
      zjolt::ToJoltR(*position), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCollideShapeEach(const ZJoltPhysicsSystem *system,
                                  const ZJoltShape *shape,
                                  const ZJoltVec3 *scale,
                                  const ZJoltRVec3 *position,
                                  const ZJoltQuat *rotation,
                                  float max_separation_distance,
                                  const ZJoltQueryFilters *filters,
                                  ZJoltCollideShapeHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, shape, position, rotation))
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();

  const JPH::Shape *impl = zjolt::ToJolt(shape);
  const JPH::Vec3 shape_scale =
      scale != nullptr ? zjolt::ToJolt(*scale) : JPH::Vec3::sOne();

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = max_separation_distance;

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectCollideHit{},
      ForwardToHost<ZJoltCollideShapeHit, ZJoltCollideShapeHitFn>{on_hit,
                                                                 user});
  system->system.GetNarrowPhaseQuery().CollideShape(
      impl, shape_scale,
      MakeCollideTransform(impl, shape_scale, *position, *rotation), settings,
      zjolt::ToJoltR(*position), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Point
//===----------------------------------------------------------------------===//

ZJoltResult zjoltCollidePointAll(const ZJoltPhysicsSystem *system,
                                 const ZJoltRVec3 *point,
                                 const ZJoltQueryFilters *filters,
                                 ZJoltCollidePointHit *out_hits,
                                 uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(system, point, out_count))
    return ZJOLT_RESULT_INVALID_ARGUMENT;

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollidePointCollector, ZJoltCollidePointHit>(
      ProjectPointHit{},
      FillBuffer<ZJoltCollidePointHit>{out_hits, capacity});
  system->system.GetNarrowPhaseQuery().CollidePoint(
      zjolt::ToJoltR(*point), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCollidePointEach(const ZJoltPhysicsSystem *system,
                                  const ZJoltRVec3 *point,
                                  const ZJoltQueryFilters *filters,
                                  ZJoltCollidePointHitFn on_hit, void *user) {
  ZJOLT_ENTER();
  if (!zjolt::Present(system, point)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (on_hit == nullptr) return MissingCallback();

  zjolt::QueryFilters adapters(filters);
  auto collector = MakeStream<JPH::CollidePointCollector, ZJoltCollidePointHit>(
      ProjectPointHit{},
      ForwardToHost<ZJoltCollidePointHit, ZJoltCollidePointHitFn>{on_hit,
                                                                 user});
  system->system.GetNarrowPhaseQuery().CollidePoint(
      zjolt::ToJoltR(*point), collector, adapters.broad_phase,
      adapters.object_layer, adapters.body, adapters.shape);

  return ZJOLT_RESULT_OK;
}


//===----------------------------------------------------------------------===//
// Shape versus shape
//
// The same collector, sinks and count-then-fill tail as everything above,
// driven against JPH::CollisionDispatch instead of a NarrowPhaseQuery. There
// is no *Each form here for the reason zjolt_transformed.h gives: the result
// set behind two already-named shapes is bounded by their own leaf counts,
// not by however much of a world a broad phase might hand back.
//===----------------------------------------------------------------------===//

void zjoltCollideShapeSettingsInit(ZJoltCollideShapeSettings *settings) {
  if (settings == nullptr) return;
  const JPH::CollideShapeSettings defaults;
  settings->active_edge_mode = ToCActiveEdgeMode(defaults.mActiveEdgeMode);
  settings->collect_faces_mode =
      ToCCollectFacesMode(defaults.mCollectFacesMode);
  settings->collision_tolerance = defaults.mCollisionTolerance;
  settings->penetration_tolerance = defaults.mPenetrationTolerance;
  settings->active_edge_movement_direction =
      zjolt::ToC(defaults.mActiveEdgeMovementDirection);
  settings->max_separation_distance = defaults.mMaxSeparationDistance;
  settings->back_face_mode = ToCBackFaceMode(defaults.mBackFaceMode);
}

void zjoltShapeCastSettingsInit(ZJoltShapeCastSettings *settings) {
  if (settings == nullptr) return;
  const JPH::ShapeCastSettings defaults;
  settings->active_edge_mode = ToCActiveEdgeMode(defaults.mActiveEdgeMode);
  settings->collect_faces_mode =
      ToCCollectFacesMode(defaults.mCollectFacesMode);
  settings->collision_tolerance = defaults.mCollisionTolerance;
  settings->penetration_tolerance = defaults.mPenetrationTolerance;
  settings->active_edge_movement_direction =
      zjolt::ToC(defaults.mActiveEdgeMovementDirection);
  settings->extra_convex_radius = defaults.mExtraConvexRadius;
  settings->back_face_mode_triangles =
      ToCBackFaceMode(defaults.mBackFaceModeTriangles);
  settings->back_face_mode_convex =
      ToCBackFaceMode(defaults.mBackFaceModeConvex);
  settings->use_shrunken_shape_and_convex_radius =
      defaults.mUseShrunkenShapeAndConvexRadius;
  settings->return_deepest_point = defaults.mReturnDeepestPoint;
}

ZJoltResult zjoltCollideShapeVsShapeClosest(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHit *out_hit,
    bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(shape1, position1, rotation1, shape2, position2,
                      rotation2, out_hit, out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::CollideShapeSettings jolt_settings =
      MakeCollideShapeSettings(settings);
  const ZJoltResult tolerance =
      CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  bool had_hit = false;
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectShapeVsShapeCollideHit{pair.shape2},
      KeepBest<ZJoltCollideShapeHit>{out_hit, &had_hit});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollisionDispatch::sCollideShapeVsShape(
      pair.shape1, pair.shape2, pair.scale1, pair.scale2,
      pair.CenterOfMassTransform1(), pair.CenterOfMassTransform2(),
      sub_shape_id1, sub_shape_id2, jolt_settings, collector, shape_filter);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCollideShapeVsShapeAll(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltShape *shape2, const ZJoltVec3 *scale2,
    const ZJoltRVec3 *position2, const ZJoltQuat *rotation2,
    const ZJoltRVec3 *base_offset, const ZJoltCollideShapeSettings *settings,
    const ZJoltShapeFilter *filter, ZJoltCollideShapeHit *out_hits,
    uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape1, position1, rotation1, shape2, position2,
                      rotation2, out_count)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::CollideShapeSettings jolt_settings =
      MakeCollideShapeSettings(settings);
  const ZJoltResult tolerance =
      CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  auto collector = MakeStream<JPH::CollideShapeCollector, ZJoltCollideShapeHit>(
      ProjectShapeVsShapeCollideHit{pair.shape2},
      FillBuffer<ZJoltCollideShapeHit>{out_hits, capacity});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollisionDispatch::sCollideShapeVsShape(
      pair.shape1, pair.shape2, pair.scale1, pair.scale2,
      pair.CenterOfMassTransform1(), pair.CenterOfMassTransform2(),
      sub_shape_id1, sub_shape_id2, jolt_settings, collector, shape_filter);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

ZJoltResult zjoltCastShapeVsShapeClosest(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltVec3 *direction, const ZJoltShape *shape2,
    const ZJoltVec3 *scale2, const ZJoltRVec3 *position2,
    const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltShapeCastSettings *settings, const ZJoltShapeFilter *filter,
    ZJoltShapeCastHit *out_hit, bool *out_hit_any) {
  ZJOLT_ENTER(out_hit, out_hit_any);
  if (!zjolt::Present(shape1, position1, rotation1, direction, shape2,
                      position2, rotation2, out_hit, out_hit_any)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::ShapeCastSettings jolt_settings = MakeShapeCastSettings(settings);
  const ZJoltResult tolerance =
      CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  bool had_hit = false;
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeVsShapeCastHit{pair.shape2},
      KeepBest<ZJoltShapeCastHit>{out_hit, &had_hit});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(
      MakeDispatchShapeCast(pair, *direction), jolt_settings, pair.shape2,
      pair.scale2, shape_filter, pair.CenterOfMassTransform2(), sub_shape_id1,
      sub_shape_id2, collector);

  *out_hit_any = had_hit;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltCastShapeVsShapeAll(
    const ZJoltShape *shape1, const ZJoltVec3 *scale1,
    const ZJoltRVec3 *position1, const ZJoltQuat *rotation1,
    const ZJoltVec3 *direction, const ZJoltShape *shape2,
    const ZJoltVec3 *scale2, const ZJoltRVec3 *position2,
    const ZJoltQuat *rotation2, const ZJoltRVec3 *base_offset,
    const ZJoltShapeCastSettings *settings, const ZJoltShapeFilter *filter,
    ZJoltShapeCastHit *out_hits, uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(shape1, position1, rotation1, direction, shape2,
                      position2, rotation2, out_count)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }

  const JPH::ShapeCastSettings jolt_settings = MakeShapeCastSettings(settings);
  const ZJoltResult tolerance =
      CheckPenetrationTolerance(jolt_settings.mPenetrationTolerance);
  if (tolerance != ZJOLT_RESULT_OK) return tolerance;

  ShapePair pair;
  const ZJoltResult prepared =
      PrepareShapePair(shape1, scale1, position1, rotation1, shape2, scale2,
                       position2, rotation2, base_offset, &pair);
  if (prepared != ZJOLT_RESULT_OK) return prepared;

  zjolt::ShapeFilterAdapter shape_filter = MakeShapeFilter(filter);
  auto collector = MakeStream<JPH::CastShapeCollector, ZJoltShapeCastHit>(
      ProjectShapeVsShapeCastHit{pair.shape2},
      FillBuffer<ZJoltShapeCastHit>{out_hits, capacity});
  JPH::SubShapeIDCreator sub_shape_id1, sub_shape_id2;
  JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(
      MakeDispatchShapeCast(pair, *direction), jolt_settings, pair.shape2,
      pair.scale2, shape_filter, pair.CenterOfMassTransform2(), sub_shape_id1,
      sub_shape_id2, collector);

  return ReportCount(collector.sink().count, out_hits, capacity, out_count);
}

}  // extern "C"
