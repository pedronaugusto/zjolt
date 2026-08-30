//===----------------------------------------------------------------------===//
// zjolt — GJK, EPA, convex hull building, polygon clipping, triangle indexing.
//===----------------------------------------------------------------------===//

#include "zjolt_internal.h"

#include <Jolt/Geometry/ClipPoly.h>
#include <Jolt/Geometry/ClosestPoint.h>
#include <Jolt/Geometry/ConvexHullBuilder.h>
#include <Jolt/Geometry/ConvexHullBuilder2D.h>
#include <Jolt/Geometry/EPAConvexHullBuilder.h>
#include <Jolt/Geometry/EPAPenetrationDepth.h>
#include <Jolt/Geometry/GJKClosestPoint.h>
#include <Jolt/Geometry/Indexify.h>

#include <climits>

namespace {

bool HasSupport(const ZJoltConvexSupport *s) { return s != nullptr && s->support != nullptr; }

/// Adapts a `ZJoltConvexSupport` to the `GetSupport(Vec3Arg)` shape GJK and
/// EPA's own templates require. Holds a reference, not a copy: the callback
/// is only ever invoked for the duration of the GJK/EPA call this lives in.
class CSupport {
 public:
  explicit CSupport(const ZJoltConvexSupport &s) : s_(s) {}
  JPH::Vec3 GetSupport(JPH::Vec3Arg direction) const {
    return zjolt::ToJolt(s_.support(s_.user, zjolt::ToC(direction)));
  }

 private:
  const ZJoltConvexSupport &s_;
};

/// Copy-out with the capacity check every buffer-form entry point in this
/// file shares: `*out_count` is always written, and a too-small `capacity`
/// is ZJOLT_RESULT_BUFFER_TOO_SMALL without touching `out`.
ZJoltResult CopyOutVec3(const JPH::Array<JPH::Vec3> &values, ZJoltVec3 *out,
                        uint32_t capacity, uint32_t *out_count) {
  *out_count = static_cast<uint32_t>(values.size());
  if (out == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < values.size()) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (size_t i = 0; i < values.size(); ++i) out[i] = zjolt::ToC(values[i]);
  return ZJOLT_RESULT_OK;
}

}  // namespace

//===----------------------------------------------------------------------===//
// The support-function seam
//===----------------------------------------------------------------------===//

namespace {

enum class AdapterKind { Transformed, AddConvexRadius, MinkowskiDifference, Polygon, Triangle };

}  // namespace

/// One C++ type behind all five factories rather than one per kind: every
/// field is small, `kind` says which are live, and this sidesteps having to
/// destroy a handle through a base-class pointer whose alignment may not
/// match the derived allocation (see AllocateFor/FreeFor's own note in
/// zjolt_internal.h on exactly that trap).
struct ZJoltConvexSupportAdapter {
  AdapterKind kind = AdapterKind::Transformed;
  JPH::Mat44 transform = JPH::Mat44::sIdentity();
  ZJoltConvexSupport inner_a{};
  ZJoltConvexSupport inner_b{};
  float radius = 0.0f;
  JPH::Array<JPH::Vec3> points;
  JPH::Vec3 v1 = JPH::Vec3::sZero();
  JPH::Vec3 v2 = JPH::Vec3::sZero();
  JPH::Vec3 v3 = JPH::Vec3::sZero();

  /// Reimplements TransformedConvexObject / AddConvexRadius /
  /// MinkowskiDifference / PolygonConvexSupport / TriangleConvexSupport's own
  /// GetSupport (Jolt/Geometry/ConvexSupport.h), over this adapter's fields
  /// instead of a templated inner object.
  JPH::Vec3 GetSupport(JPH::Vec3Arg direction) const {
    switch (kind) {
      case AdapterKind::Transformed: {
        JPH::Vec3 local_dir = transform.Multiply3x3Transposed(direction);
        JPH::Vec3 inner = zjolt::ToJolt(inner_a.support(inner_a.user, zjolt::ToC(local_dir)));
        return transform * inner;
      }
      case AdapterKind::AddConvexRadius: {
        JPH::Vec3 base = zjolt::ToJolt(inner_a.support(inner_a.user, zjolt::ToC(direction)));
        float length = direction.Length();
        return length > 0.0f ? base + (radius / length) * direction : base;
      }
      case AdapterKind::MinkowskiDifference: {
        JPH::Vec3 a = zjolt::ToJolt(inner_a.support(inner_a.user, zjolt::ToC(direction)));
        JPH::Vec3 b = zjolt::ToJolt(inner_b.support(inner_b.user, zjolt::ToC(-direction)));
        return a - b;
      }
      case AdapterKind::Polygon: {
        JPH::Vec3 best = points[0];
        float best_dot = points[0].Dot(direction);
        for (size_t i = 1; i < points.size(); ++i) {
          float dot = points[i].Dot(direction);
          if (dot > best_dot) {
            best_dot = dot;
            best = points[i];
          }
        }
        return best;
      }
      case AdapterKind::Triangle: {
        float d1 = v1.Dot(direction), d2 = v2.Dot(direction), d3 = v3.Dot(direction);
        if (d1 > d2) return d1 > d3 ? v1 : v3;
        return d2 > d3 ? v2 : v3;
      }
    }
    return JPH::Vec3::sZero();
  }
};

namespace {

ZJoltVec3 AdapterSupportTrampoline(void *user, ZJoltVec3 direction) {
  const auto *adapter = static_cast<const ZJoltConvexSupportAdapter *>(user);
  return zjolt::ToC(adapter->GetSupport(zjolt::ToJolt(direction)));
}

}  // namespace

ZJoltResult zjoltConvexSupportCreateTransformed(const ZJoltMat44 *transform,
                                                const ZJoltConvexSupport *inner,
                                                ZJoltConvexSupportAdapter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(transform, out) || !HasSupport(inner)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  ZJoltConvexSupportAdapter *handle = zjolt::New<ZJoltConvexSupportAdapter>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  handle->kind = AdapterKind::Transformed;
  handle->transform = zjolt::ToJolt(*transform);
  handle->inner_a = *inner;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConvexSupportCreateAddConvexRadius(const ZJoltConvexSupport *inner,
                                                    float radius,
                                                    ZJoltConvexSupportAdapter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out) || !HasSupport(inner)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJoltConvexSupportAdapter *handle = zjolt::New<ZJoltConvexSupportAdapter>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  handle->kind = AdapterKind::AddConvexRadius;
  handle->inner_a = *inner;
  handle->radius = radius;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConvexSupportCreateMinkowskiDifference(const ZJoltConvexSupport *a,
                                                         const ZJoltConvexSupport *b,
                                                         ZJoltConvexSupportAdapter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out) || !HasSupport(a) || !HasSupport(b)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  ZJoltConvexSupportAdapter *handle = zjolt::New<ZJoltConvexSupportAdapter>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  handle->kind = AdapterKind::MinkowskiDifference;
  handle->inner_a = *a;
  handle->inner_b = *b;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConvexSupportCreatePolygon(const ZJoltVec3 *points, uint32_t num_points,
                                            ZJoltConvexSupportAdapter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(points, out) || num_points == 0) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJoltConvexSupportAdapter *handle = zjolt::New<ZJoltConvexSupportAdapter>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  handle->kind = AdapterKind::Polygon;
  handle->points.reserve(num_points);
  for (uint32_t i = 0; i < num_points; ++i) handle->points.push_back(zjolt::ToJolt(points[i]));
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConvexSupportCreateTriangle(const ZJoltVec3 *v1, const ZJoltVec3 *v2,
                                             const ZJoltVec3 *v3,
                                             ZJoltConvexSupportAdapter **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(v1, v2, v3, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJoltConvexSupportAdapter *handle = zjolt::New<ZJoltConvexSupportAdapter>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  handle->kind = AdapterKind::Triangle;
  handle->v1 = zjolt::ToJolt(*v1);
  handle->v2 = zjolt::ToJolt(*v2);
  handle->v3 = zjolt::ToJolt(*v3);
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltConvexSupportAdapterAsSupport(const ZJoltConvexSupportAdapter *adapter,
                                        ZJoltConvexSupport *out) {
  if (out == nullptr) return;
  if (adapter == nullptr) {
    *out = ZJoltConvexSupport{};
    return;
  }
  *out = ZJoltConvexSupport{&AdapterSupportTrampoline,
                            const_cast<ZJoltConvexSupportAdapter *>(adapter)};
}

void zjoltConvexSupportAdapterDestroy(ZJoltConvexSupportAdapter *adapter) {
  if (adapter == nullptr) return;
  zjolt::Delete(adapter);
  zjolt::HandleDestroyed();
}

//===----------------------------------------------------------------------===//
// GJK
//===----------------------------------------------------------------------===//

struct ZJoltGJK {
  JPH::GJKClosestPoint gjk;
};

ZJoltResult zjoltGJKCreate(ZJoltGJK **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJoltGJK *handle = zjolt::New<ZJoltGJK>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltGJKDestroy(ZJoltGJK *gjk) {
  if (gjk == nullptr) return;
  zjolt::Delete(gjk);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltGJKIntersects(ZJoltGJK *gjk, const ZJoltConvexSupport *a,
                               const ZJoltConvexSupport *b, float tolerance,
                               ZJoltVec3 *io_v, bool *out_intersects) {
  ZJOLT_ENTER(out_intersects);
  if (!zjolt::Present(gjk, io_v, out_intersects) || !HasSupport(a) || !HasSupport(b)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  CSupport sa(*a), sb(*b);
  JPH::Vec3 v = zjolt::ToJolt(*io_v);
  *out_intersects = gjk->gjk.Intersects(sa, sb, tolerance, v);
  *io_v = zjolt::ToC(v);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltGJKGetClosestPoints(ZJoltGJK *gjk, const ZJoltConvexSupport *a,
                                     const ZJoltConvexSupport *b, float tolerance,
                                     float max_dist_sq, ZJoltVec3 *io_v,
                                     ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
                                     float *out_dist_sq) {
  ZJOLT_ENTER(out_point_a, out_point_b, out_dist_sq);
  if (!zjolt::Present(gjk, io_v, out_dist_sq) || !HasSupport(a) || !HasSupport(b)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  CSupport sa(*a), sb(*b);
  JPH::Vec3 v = zjolt::ToJolt(*io_v);
  JPH::Vec3 pa = JPH::Vec3::sZero(), pb = JPH::Vec3::sZero();
  float dist_sq = gjk->gjk.GetClosestPoints(sa, sb, tolerance, max_dist_sq, v, pa, pb);
  *io_v = zjolt::ToC(v);
  *out_dist_sq = dist_sq;
  if (dist_sq > 0.0f && dist_sq < FLT_MAX) {
    zjolt::WriteVec3(out_point_a, pa);
    zjolt::WriteVec3(out_point_b, pb);
  }
  return ZJOLT_RESULT_OK;
}

void zjoltGJKGetClosestPointsSimplex(const ZJoltGJK *gjk, ZJoltVec3 *out_y, ZJoltVec3 *out_p,
                                     ZJoltVec3 *out_q, uint32_t *out_num_points) {
  if (out_num_points != nullptr) *out_num_points = 0;
  if (gjk == nullptr || out_y == nullptr || out_p == nullptr || out_q == nullptr ||
      out_num_points == nullptr) {
    return;
  }
  JPH::Vec3 y[ZJOLT_GJK_MAX_SIMPLEX_POINTS], p[ZJOLT_GJK_MAX_SIMPLEX_POINTS],
      q[ZJOLT_GJK_MAX_SIMPLEX_POINTS];
  JPH::uint n = 0;
  gjk->gjk.GetClosestPointsSimplex(y, p, q, n);
  for (JPH::uint i = 0; i < n; ++i) {
    out_y[i] = zjolt::ToC(y[i]);
    out_p[i] = zjolt::ToC(p[i]);
    out_q[i] = zjolt::ToC(q[i]);
  }
  *out_num_points = n;
}

ZJoltResult zjoltGJKCalculatePointAAndB(const ZJoltVec3 *y, const ZJoltVec3 *p,
                                        const ZJoltVec3 *q, uint32_t num_points,
                                        ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b) {
  ZJOLT_ENTER(out_point_a, out_point_b);
  if (!zjolt::Present(y, p, q, out_point_a, out_point_b)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (num_points == 0 || num_points > ZJOLT_GJK_MAX_SIMPLEX_POINTS) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "num_points must be between 1 and ZJOLT_GJK_MAX_SIMPLEX_POINTS");
  }
  JPH::Vec3 pa, pb;
  switch (num_points) {
    case 1:
      pa = zjolt::ToJolt(p[0]);
      pb = zjolt::ToJolt(q[0]);
      break;
    case 2: {
      float u, v;
      JPH::ClosestPoint::GetBaryCentricCoordinates(zjolt::ToJolt(y[0]), zjolt::ToJolt(y[1]), u, v);
      pa = u * zjolt::ToJolt(p[0]) + v * zjolt::ToJolt(p[1]);
      pb = u * zjolt::ToJolt(q[0]) + v * zjolt::ToJolt(q[1]);
      break;
    }
    case 3: {
      float u, v, w;
      JPH::ClosestPoint::GetBaryCentricCoordinates(zjolt::ToJolt(y[0]), zjolt::ToJolt(y[1]),
                                                    zjolt::ToJolt(y[2]), u, v, w);
      pa = u * zjolt::ToJolt(p[0]) + v * zjolt::ToJolt(p[1]) + w * zjolt::ToJolt(p[2]);
      pb = u * zjolt::ToJolt(q[0]) + v * zjolt::ToJolt(q[1]) + w * zjolt::ToJolt(q[2]);
      break;
    }
    default:
      // 4: the origin is inside the simplex, so there is no well-defined
      // closest pair; leave both outputs at the zero ZJOLT_ENTER already
      // wrote, matching GJKClosestPoint::CalculatePointAAndB leaving them
      // undefined in this case.
      return ZJOLT_RESULT_OK;
  }
  zjolt::WriteVec3(out_point_a, pa);
  zjolt::WriteVec3(out_point_b, pb);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltGJKCastRay(ZJoltGJK *gjk, const ZJoltVec3 *ray_origin,
                            const ZJoltVec3 *ray_direction, float tolerance,
                            const ZJoltConvexSupport *a, float *io_lambda, bool *out_hit) {
  ZJOLT_ENTER(out_hit);
  if (!zjolt::Present(gjk, ray_origin, ray_direction, io_lambda, out_hit) || !HasSupport(a)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  CSupport sa(*a);
  float lambda = *io_lambda;
  *out_hit =
      gjk->gjk.CastRay(zjolt::ToJolt(*ray_origin), zjolt::ToJolt(*ray_direction), tolerance, sa, lambda);
  *io_lambda = lambda;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltGJKIntersectsSweep(ZJoltGJK *gjk, const ZJoltMat44 *start,
                                    const ZJoltVec3 *direction, float tolerance,
                                    const ZJoltConvexSupport *a, const ZJoltConvexSupport *b,
                                    float *io_lambda, bool *out_hit) {
  ZJOLT_ENTER(out_hit);
  if (!zjolt::Present(gjk, start, direction, io_lambda, out_hit) || !HasSupport(a) ||
      !HasSupport(b)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  CSupport sa(*a), sb(*b);
  float lambda = *io_lambda;
  *out_hit = gjk->gjk.CastShape(zjolt::ToJolt(*start), zjolt::ToJolt(*direction), tolerance, sa,
                                sb, lambda);
  *io_lambda = lambda;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltGJKCastShape(ZJoltGJK *gjk, const ZJoltMat44 *start, const ZJoltVec3 *direction,
                              float tolerance, const ZJoltConvexSupport *a,
                              const ZJoltConvexSupport *b, float convex_radius_a,
                              float convex_radius_b, float *io_lambda, ZJoltVec3 *out_point_a,
                              ZJoltVec3 *out_point_b, ZJoltVec3 *out_separating_axis,
                              bool *out_hit) {
  ZJOLT_ENTER(out_point_a, out_point_b, out_separating_axis, out_hit);
  if (!zjolt::Present(gjk, start, direction, io_lambda, out_hit) || !HasSupport(a) ||
      !HasSupport(b)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  CSupport sa(*a), sb(*b);
  float lambda = *io_lambda;
  JPH::Vec3 pa = JPH::Vec3::sZero(), pb = JPH::Vec3::sZero(), axis = JPH::Vec3::sZero();
  bool hit = gjk->gjk.CastShape(zjolt::ToJolt(*start), zjolt::ToJolt(*direction), tolerance, sa,
                                sb, convex_radius_a, convex_radius_b, lambda, pa, pb, axis);
  *io_lambda = lambda;
  *out_hit = hit;
  if (hit) {
    zjolt::WriteVec3(out_point_a, pa);
    zjolt::WriteVec3(out_point_b, pb);
    zjolt::WriteVec3(out_separating_axis, axis);
  }
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// EPA
//===----------------------------------------------------------------------===//

namespace {

ZJoltEpaStatus MapEPAStatus(JPH::EPAPenetrationDepth::EStatus s) {
  switch (s) {
    case JPH::EPAPenetrationDepth::EStatus::NotColliding:
      return ZJOLT_EPA_STATUS_NOT_COLLIDING;
    case JPH::EPAPenetrationDepth::EStatus::Colliding:
      return ZJOLT_EPA_STATUS_COLLIDING;
    case JPH::EPAPenetrationDepth::EStatus::Indeterminate:
      return ZJOLT_EPA_STATUS_INDETERMINATE;
  }
  return ZJOLT_EPA_STATUS_NOT_COLLIDING;
}

}  // namespace

struct ZJoltEPA {
  JPH::EPAPenetrationDepth epa;
};

ZJoltResult zjoltEPACreate(ZJoltEPA **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  ZJoltEPA *handle = zjolt::New<ZJoltEPA>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltEPADestroy(ZJoltEPA *epa) {
  if (epa == nullptr) return;
  zjolt::Delete(epa);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltEPAGetPenetrationDepthStepGJK(
    ZJoltEPA *epa, const ZJoltConvexSupport *a_excluding_radius, float convex_radius_a,
    const ZJoltConvexSupport *b_excluding_radius, float convex_radius_b, float tolerance,
    ZJoltVec3 *io_v, ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
    ZJoltEpaStatus *out_status) {
  ZJOLT_ENTER(out_point_a, out_point_b, out_status);
  if (!zjolt::Present(epa, io_v, out_status) || !HasSupport(a_excluding_radius) ||
      !HasSupport(b_excluding_radius)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  JPH::Vec3 v = zjolt::ToJolt(*io_v);
  if (v.IsNearZero()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "io_v: must not be near zero on entry -- pass a previous result, "
                           "or (1, 0, 0)");
  }
  CSupport sa(*a_excluding_radius), sb(*b_excluding_radius);
  JPH::Vec3 pa = JPH::Vec3::sZero(), pb = JPH::Vec3::sZero();
  JPH::EPAPenetrationDepth::EStatus status = epa->epa.GetPenetrationDepthStepGJK(
      sa, convex_radius_a, sb, convex_radius_b, tolerance, v, pa, pb);
  *io_v = zjolt::ToC(v);
  *out_status = MapEPAStatus(status);
  zjolt::WriteVec3(out_point_a, pa);
  zjolt::WriteVec3(out_point_b, pb);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltEPAGetPenetrationDepthStepEPA(ZJoltEPA *epa,
                                               const ZJoltConvexSupport *a_including_radius,
                                               const ZJoltConvexSupport *b_including_radius,
                                               float tolerance, ZJoltVec3 *out_v,
                                               ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
                                               bool *out_collided) {
  ZJOLT_ENTER(out_v, out_point_a, out_point_b, out_collided);
  if (!zjolt::Present(epa, out_collided) || !HasSupport(a_including_radius) ||
      !HasSupport(b_including_radius)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  ZJoltResult tolerance_check = zjolt::CheckPenetrationTolerance(tolerance);
  if (tolerance_check != ZJOLT_RESULT_OK) return tolerance_check;
  CSupport sa(*a_including_radius), sb(*b_including_radius);
  JPH::Vec3 v = JPH::Vec3::sZero(), pa = JPH::Vec3::sZero(), pb = JPH::Vec3::sZero();
  *out_collided = epa->epa.GetPenetrationDepthStepEPA(sa, sb, tolerance, v, pa, pb);
  zjolt::WriteVec3(out_v, v);
  zjolt::WriteVec3(out_point_a, pa);
  zjolt::WriteVec3(out_point_b, pb);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltEPAGetPenetrationDepth(
    ZJoltEPA *epa, const ZJoltConvexSupport *a_excluding_radius,
    const ZJoltConvexSupport *a_including_radius, float convex_radius_a,
    const ZJoltConvexSupport *b_excluding_radius,
    const ZJoltConvexSupport *b_including_radius, float convex_radius_b,
    float collision_tolerance_sq, float penetration_tolerance, ZJoltVec3 *io_v,
    ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b, bool *out_collided) {
  ZJOLT_ENTER(out_point_a, out_point_b, out_collided);
  if (!zjolt::Present(epa, io_v, out_collided) || !HasSupport(a_excluding_radius) ||
      !HasSupport(a_including_radius) || !HasSupport(b_excluding_radius) ||
      !HasSupport(b_including_radius)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  ZJoltResult tolerance_check = zjolt::CheckPenetrationTolerance(penetration_tolerance);
  if (tolerance_check != ZJOLT_RESULT_OK) return tolerance_check;
  JPH::Vec3 v = zjolt::ToJolt(*io_v);
  if (v.IsNearZero()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "io_v: must not be near zero on entry -- pass a previous result, "
                           "or (1, 0, 0)");
  }
  CSupport ae(*a_excluding_radius), ai(*a_including_radius), be(*b_excluding_radius),
      bi(*b_including_radius);
  JPH::Vec3 pa = JPH::Vec3::sZero(), pb = JPH::Vec3::sZero();
  *out_collided = epa->epa.GetPenetrationDepth(ae, ai, convex_radius_a, be, bi, convex_radius_b,
                                               collision_tolerance_sq, penetration_tolerance, v,
                                               pa, pb);
  *io_v = zjolt::ToC(v);
  zjolt::WriteVec3(out_point_a, pa);
  zjolt::WriteVec3(out_point_b, pb);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltEPACastShape(ZJoltEPA *epa, const ZJoltMat44 *start, const ZJoltVec3 *direction,
                              float collision_tolerance, float penetration_tolerance,
                              const ZJoltConvexSupport *a, const ZJoltConvexSupport *b,
                              float convex_radius_a, float convex_radius_b,
                              bool return_deepest_point, float *io_lambda,
                              ZJoltVec3 *out_point_a, ZJoltVec3 *out_point_b,
                              ZJoltVec3 *out_contact_normal, bool *out_hit) {
  ZJOLT_ENTER(out_point_a, out_point_b, out_contact_normal, out_hit);
  if (!zjolt::Present(epa, start, direction, io_lambda, out_hit) || !HasSupport(a) ||
      !HasSupport(b)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  ZJoltResult tolerance_check = zjolt::CheckPenetrationTolerance(penetration_tolerance);
  if (tolerance_check != ZJOLT_RESULT_OK) return tolerance_check;
  CSupport sa(*a), sb(*b);
  float lambda = *io_lambda;
  JPH::Vec3 pa = JPH::Vec3::sZero(), pb = JPH::Vec3::sZero(), normal = JPH::Vec3::sZero();
  bool hit = epa->epa.CastShape(zjolt::ToJolt(*start), zjolt::ToJolt(*direction),
                                collision_tolerance, penetration_tolerance, sa, sb,
                                convex_radius_a, convex_radius_b, return_deepest_point, lambda,
                                pa, pb, normal);
  *io_lambda = lambda;
  *out_hit = hit;
  if (hit) {
    zjolt::WriteVec3(out_point_a, pa);
    zjolt::WriteVec3(out_point_b, pb);
    zjolt::WriteVec3(out_contact_normal, normal);
  }
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// EPA's own hull builder
//===----------------------------------------------------------------------===//

static_assert(JPH::EPAConvexHullBuilder::cMaxPoints == ZJOLT_EPA_CONVEX_HULL_BUILDER_MAX_POINTS,
             "EPAConvexHullBuilder::cMaxPoints drifted from the ABI constant");

namespace {

JPH::EPAConvexHullBuilder::Triangle *ToTriangle(ZJoltEPATriangle *t) {
  return reinterpret_cast<JPH::EPAConvexHullBuilder::Triangle *>(t);
}
const JPH::EPAConvexHullBuilder::Triangle *ToTriangle(const ZJoltEPATriangle *t) {
  return reinterpret_cast<const JPH::EPAConvexHullBuilder::Triangle *>(t);
}
ZJoltEPATriangle *FromTriangle(JPH::EPAConvexHullBuilder::Triangle *t) {
  return reinterpret_cast<ZJoltEPATriangle *>(t);
}

}  // namespace

/// `positions` (fixed at construction) and the builder over them, held
/// together: `builder` stores a reference to `positions` for its own
/// lifetime, so the two must live and die together.
struct ZJoltEPAConvexHullBuilder {
  JPH::EPAConvexHullBuilder::Points positions;
  JPH::EPAConvexHullBuilder builder;

  ZJoltEPAConvexHullBuilder() : builder(positions) {}
};

ZJoltResult zjoltEPAConvexHullBuilderCreate(const ZJoltVec3 *positions, uint32_t num_positions,
                                            ZJoltEPAConvexHullBuilder **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out) || (num_positions > 0 && positions == nullptr)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  if (num_positions > ZJOLT_EPA_CONVEX_HULL_BUILDER_MAX_POINTS) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "num_positions exceeds ZJOLT_EPA_CONVEX_HULL_BUILDER_MAX_POINTS");
  }
  ZJoltEPAConvexHullBuilder *handle = zjolt::New<ZJoltEPAConvexHullBuilder>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  // Written directly through data() plus GetSizeRef() rather than a
  // push_back loop: Points is exactly the fixed-capacity buffer that pairing
  // exists for.
  JPH::Vec3 *data = handle->positions.data();
  for (uint32_t i = 0; i < num_positions; ++i) data[i] = zjolt::ToJolt(positions[i]);
  handle->positions.GetSizeRef() = num_positions;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltEPAConvexHullBuilderDestroy(ZJoltEPAConvexHullBuilder *builder) {
  if (builder == nullptr) return;
  zjolt::Delete(builder);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltEPAConvexHullBuilderInitialize(ZJoltEPAConvexHullBuilder *builder, uint32_t idx1,
                                                uint32_t idx2, uint32_t idx3) {
  ZJOLT_ENTER();
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  uint32_t n = static_cast<uint32_t>(builder->positions.size());
  if (idx1 >= n || idx2 >= n || idx3 >= n || idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "idx1/idx2/idx3 must be distinct and within the positions this builder was created with");
  }
  builder->builder.Initialize(static_cast<int>(idx1), static_cast<int>(idx2),
                              static_cast<int>(idx3));
  return ZJOLT_RESULT_OK;
}

bool zjoltEPAConvexHullBuilderHasNextTriangle(const ZJoltEPAConvexHullBuilder *builder) {
  if (builder == nullptr) return false;
  return builder->builder.HasNextTriangle();
}

ZJoltEPATriangle *zjoltEPAConvexHullBuilderPeekClosestTriangle(ZJoltEPAConvexHullBuilder *builder) {
  if (builder == nullptr || !builder->builder.HasNextTriangle()) return nullptr;
  return FromTriangle(builder->builder.PeekClosestTriangleInQueue());
}

ZJoltEPATriangle *zjoltEPAConvexHullBuilderPopClosestTriangle(ZJoltEPAConvexHullBuilder *builder) {
  if (builder == nullptr || !builder->builder.HasNextTriangle()) return nullptr;
  return FromTriangle(builder->builder.PopClosestTriangleFromQueue());
}

ZJoltEPATriangle *zjoltEPAConvexHullBuilderFindFacingTriangle(ZJoltEPAConvexHullBuilder *builder,
                                                              const ZJoltVec3 *position,
                                                              float *out_best_dist_sq) {
  if (out_best_dist_sq != nullptr) *out_best_dist_sq = 0.0f;
  if (builder == nullptr || position == nullptr) return nullptr;
  float best_dist_sq = 0.0f;
  JPH::EPAConvexHullBuilder::Triangle *t =
      builder->builder.FindFacingTriangle(zjolt::ToJolt(*position), best_dist_sq);
  if (out_best_dist_sq != nullptr) *out_best_dist_sq = best_dist_sq;
  return FromTriangle(t);
}

void zjoltEPAConvexHullBuilderFreeTriangle(ZJoltEPAConvexHullBuilder *builder,
                                           ZJoltEPATriangle *triangle) {
  if (builder == nullptr || triangle == nullptr) return;
  JPH::EPAConvexHullBuilder::Triangle *t = ToTriangle(triangle);
  // EPAConvexHullBuilder::FreeTriangle requires every edge already unlinked
  // and the triangle marked removed; done here rather than left to the
  // caller, since nothing else in this API exposes the private unlink step.
  for (auto &edge : t->mEdge) {
    if (edge.mNeighbourTriangle != nullptr) {
      edge.mNeighbourTriangle->mEdge[edge.mNeighbourEdge].mNeighbourTriangle = nullptr;
      edge.mNeighbourTriangle = nullptr;
    }
  }
  t->mRemoved = true;
  builder->builder.FreeTriangle(t);
}

bool zjoltEPATriangleIsFacing(const ZJoltEPATriangle *triangle, const ZJoltVec3 *position) {
  if (triangle == nullptr || position == nullptr) return false;
  return ToTriangle(triangle)->IsFacing(zjolt::ToJolt(*position));
}

bool zjoltEPATriangleIsFacingOrigin(const ZJoltEPATriangle *triangle) {
  if (triangle == nullptr) return false;
  return ToTriangle(triangle)->IsFacingOrigin();
}

ZJoltResult zjoltEPATriangleGetNextEdge(const ZJoltEPATriangle *triangle, uint32_t index,
                                        ZJoltEPAEdge *out_edge) {
  ZJOLT_ENTER(out_edge);
  if (!zjolt::Present(triangle, out_edge) || index >= 3) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::EPAConvexHullBuilder::Edge &edge = ToTriangle(triangle)->GetNextEdge(index);
  out_edge->neighbour_triangle = FromTriangle(edge.mNeighbourTriangle);
  out_edge->neighbour_edge = static_cast<int32_t>(edge.mNeighbourEdge);
  out_edge->start_idx = static_cast<uint32_t>(edge.mStartIdx);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Convex hull building
//===----------------------------------------------------------------------===//

namespace {

ZJoltConvexHullResult MapHullResult(JPH::ConvexHullBuilder::EResult r) {
  switch (r) {
    case JPH::ConvexHullBuilder::EResult::Success:
      return ZJOLT_CONVEX_HULL_RESULT_SUCCESS;
    case JPH::ConvexHullBuilder::EResult::MaxVerticesReached:
      return ZJOLT_CONVEX_HULL_RESULT_MAX_VERTICES_REACHED;
    case JPH::ConvexHullBuilder::EResult::TooFewPoints:
      return ZJOLT_CONVEX_HULL_RESULT_TOO_FEW_POINTS;
    case JPH::ConvexHullBuilder::EResult::TooFewFaces:
      return ZJOLT_CONVEX_HULL_RESULT_TOO_FEW_FACES;
    case JPH::ConvexHullBuilder::EResult::Degenerate:
      return ZJOLT_CONVEX_HULL_RESULT_DEGENERATE;
  }
  return ZJOLT_CONVEX_HULL_RESULT_DEGENERATE;
}

/// Walks a face's edge loop once, counting or copying out `mStartIdx`.
/// `out_indices` may be NULL to only count.
uint32_t WalkFace(const JPH::ConvexHullBuilder::Face *face, uint32_t *out_indices) {
  const JPH::ConvexHullBuilder::Edge *start = face->mFirstEdge;
  if (start == nullptr) return 0;
  uint32_t n = 0;
  const JPH::ConvexHullBuilder::Edge *e = start;
  do {
    if (out_indices != nullptr) out_indices[n] = static_cast<uint32_t>(e->mStartIdx);
    ++n;
    e = e->mNextEdge;
  } while (e != start);
  return n;
}

int ClampToInt(uint32_t v) {
  return v > static_cast<uint32_t>(INT_MAX) ? INT_MAX : static_cast<int>(v);
}

}  // namespace

struct ZJoltConvexHullBuilder {
  JPH::Array<JPH::Vec3> positions;
  JPH::ConvexHullBuilder builder;

  explicit ZJoltConvexHullBuilder(JPH::Array<JPH::Vec3> pts)
      : positions(std::move(pts)), builder(positions) {}
};

ZJoltResult zjoltConvexHullBuilderCreate(const ZJoltVec3 *positions, uint32_t num_positions,
                                         ZJoltConvexHullBuilder **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(positions, out) || num_positions == 0) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  JPH::Array<JPH::Vec3> pts;
  pts.reserve(num_positions);
  for (uint32_t i = 0; i < num_positions; ++i) pts.push_back(zjolt::ToJolt(positions[i]));
  ZJoltConvexHullBuilder *handle = zjolt::New<ZJoltConvexHullBuilder>(std::move(pts));
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltConvexHullBuilderDestroy(ZJoltConvexHullBuilder *builder) {
  if (builder == nullptr) return;
  zjolt::Delete(builder);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltConvexHullBuilderInitialize(ZJoltConvexHullBuilder *builder,
                                             uint32_t max_vertices, float tolerance,
                                             ZJoltConvexHullResult *out_hull_result) {
  ZJOLT_ENTER(out_hull_result);
  if (!zjolt::Present(builder, out_hull_result)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const char *error = nullptr;
  int max_v = max_vertices == 0 ? INT_MAX : ClampToInt(max_vertices);
  JPH::ConvexHullBuilder::EResult result = builder->builder.Initialize(max_v, tolerance, error);
  *out_hull_result = MapHullResult(result);
  if (result == JPH::ConvexHullBuilder::EResult::Success ||
      result == JPH::ConvexHullBuilder::EResult::MaxVerticesReached) {
    return ZJOLT_RESULT_OK;
  }
  return zjolt::SetError(ZJOLT_RESULT_SHAPE_INVALID,
                         error != nullptr ? error : "convex hull construction failed");
}

uint32_t zjoltConvexHullBuilderGetNumVerticesUsed(const ZJoltConvexHullBuilder *builder) {
  if (builder == nullptr) return 0;
  return static_cast<uint32_t>(builder->builder.GetNumVerticesUsed());
}

bool zjoltConvexHullBuilderContainsFace(const ZJoltConvexHullBuilder *builder,
                                        const uint32_t *indices, uint32_t num_indices) {
  if (builder == nullptr || indices == nullptr) return false;
  JPH::Array<int> idx;
  idx.reserve(num_indices);
  for (uint32_t i = 0; i < num_indices; ++i) idx.push_back(static_cast<int>(indices[i]));
  return builder->builder.ContainsFace(idx);
}

ZJoltResult zjoltConvexHullBuilderGetCenterOfMassAndVolume(const ZJoltConvexHullBuilder *builder,
                                                           ZJoltVec3 *out_center_of_mass,
                                                           float *out_volume) {
  ZJOLT_ENTER(out_center_of_mass, out_volume);
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::Vec3 com = JPH::Vec3::sZero();
  float volume = 0.0f;
  builder->builder.GetCenterOfMassAndVolume(com, volume);
  zjolt::WriteVec3(out_center_of_mass, com);
  if (out_volume != nullptr) *out_volume = volume;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConvexHullBuilderDetermineMaxError(const ZJoltConvexHullBuilder *builder,
                                                    int32_t *out_face_index,
                                                    float *out_max_error,
                                                    int32_t *out_max_error_position_index,
                                                    float *out_coplanar_distance) {
  ZJOLT_ENTER(out_face_index, out_max_error, out_max_error_position_index,
             out_coplanar_distance);
  if (!zjolt::Present(builder)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  JPH::ConvexHullBuilder::Face *face = nullptr;
  float max_error = 0.0f;
  int max_error_idx = -1;
  float coplanar_distance = 0.0f;
  builder->builder.DetermineMaxError(face, max_error, max_error_idx, coplanar_distance);
  if (out_max_error != nullptr) *out_max_error = max_error;
  if (out_max_error_position_index != nullptr) {
    *out_max_error_position_index = static_cast<int32_t>(max_error_idx);
  }
  if (out_coplanar_distance != nullptr) *out_coplanar_distance = coplanar_distance;
  if (out_face_index != nullptr) {
    int32_t index = -1;
    if (face != nullptr) {
      const JPH::ConvexHullBuilder::Faces &faces = builder->builder.GetFaces();
      for (size_t i = 0; i < faces.size(); ++i) {
        if (faces[i] == face) {
          index = static_cast<int32_t>(i);
          break;
        }
      }
    }
    *out_face_index = index;
  }
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConvexHullBuilderGetNumFaces(const ZJoltConvexHullBuilder *builder,
                                              uint32_t *out_num_faces) {
  ZJOLT_ENTER(out_num_faces);
  if (!zjolt::Present(builder, out_num_faces)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  *out_num_faces = static_cast<uint32_t>(builder->builder.GetFaces().size());
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConvexHullBuilderGetFace(const ZJoltConvexHullBuilder *builder,
                                          uint32_t face_index, ZJoltConvexHullFace *out_face) {
  ZJOLT_ENTER(out_face);
  if (!zjolt::Present(builder, out_face)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexHullBuilder::Faces &faces = builder->builder.GetFaces();
  if (face_index >= faces.size()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "face_index is out of range");
  }
  const JPH::ConvexHullBuilder::Face *face = faces[face_index];
  out_face->normal = zjolt::ToC(face->mNormal);
  out_face->centroid = zjolt::ToC(face->mCentroid);
  out_face->num_vertices = WalkFace(face, nullptr);
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltConvexHullBuilderGetFaceVertices(const ZJoltConvexHullBuilder *builder,
                                                  uint32_t face_index, uint32_t *out_indices,
                                                  uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(builder, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  const JPH::ConvexHullBuilder::Faces &faces = builder->builder.GetFaces();
  if (face_index >= faces.size()) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT, "face_index is out of range");
  }
  const JPH::ConvexHullBuilder::Face *face = faces[face_index];
  uint32_t n = WalkFace(face, nullptr);
  *out_count = n;
  if (out_indices == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < n) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  WalkFace(face, out_indices);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// 2D convex hull building
//===----------------------------------------------------------------------===//

struct ZJoltConvexHullBuilder2D {
  JPH::Array<JPH::Vec3> positions;
  JPH::ConvexHullBuilder2D builder;

  explicit ZJoltConvexHullBuilder2D(JPH::Array<JPH::Vec3> pts)
      : positions(std::move(pts)), builder(positions) {}
};

ZJoltResult zjoltConvexHullBuilder2DCreate(const ZJoltVec3 *positions, uint32_t num_positions,
                                           ZJoltConvexHullBuilder2D **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(positions, out) || num_positions == 0) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  JPH::Array<JPH::Vec3> pts;
  pts.reserve(num_positions);
  for (uint32_t i = 0; i < num_positions; ++i) pts.push_back(zjolt::ToJolt(positions[i]));
  ZJoltConvexHullBuilder2D *handle = zjolt::New<ZJoltConvexHullBuilder2D>(std::move(pts));
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltConvexHullBuilder2DDestroy(ZJoltConvexHullBuilder2D *builder) {
  if (builder == nullptr) return;
  zjolt::Delete(builder);
  zjolt::HandleDestroyed();
}

ZJoltResult zjoltConvexHullBuilder2DInitialize(ZJoltConvexHullBuilder2D *builder, uint32_t idx1,
                                               uint32_t idx2, uint32_t idx3,
                                               uint32_t max_vertices, float tolerance,
                                               uint32_t *out_edges, uint32_t capacity,
                                               uint32_t *out_count,
                                               ZJoltConvexHullResult *out_hull_result) {
  ZJOLT_ENTER(out_count, out_hull_result);
  if (!zjolt::Present(builder, out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  size_t num_positions = builder->positions.size();
  if (idx1 >= num_positions || idx2 >= num_positions || idx3 >= num_positions) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "idx1/idx2/idx3 must be within the positions this builder was "
                           "created with");
  }
  int max_v = max_vertices == 0 ? INT_MAX : ClampToInt(max_vertices);
  JPH::ConvexHullBuilder2D::Edges edges;
  JPH::ConvexHullBuilder2D::EResult result = builder->builder.Initialize(
      static_cast<int>(idx1), static_cast<int>(idx2), static_cast<int>(idx3), max_v, tolerance,
      edges);
  if (out_hull_result != nullptr) {
    *out_hull_result = result == JPH::ConvexHullBuilder2D::EResult::Success
                           ? ZJOLT_CONVEX_HULL_RESULT_SUCCESS
                           : ZJOLT_CONVEX_HULL_RESULT_MAX_VERTICES_REACHED;
  }
  *out_count = static_cast<uint32_t>(edges.size());
  if (out_edges == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < edges.size()) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (size_t i = 0; i < edges.size(); ++i) out_edges[i] = static_cast<uint32_t>(edges[i]);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Polygon clipping
//===----------------------------------------------------------------------===//

ZJoltResult zjoltClipPolyVsPlane(const ZJoltVec3 *polygon, uint32_t num_vertices,
                                 const ZJoltVec3 *plane_origin, const ZJoltVec3 *plane_normal,
                                 ZJoltVec3 *out_polygon, uint32_t capacity,
                                 uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(polygon, plane_origin, plane_normal, out_count) || num_vertices < 2) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  JPH::Array<JPH::Vec3> in;
  in.reserve(num_vertices);
  for (uint32_t i = 0; i < num_vertices; ++i) in.push_back(zjolt::ToJolt(polygon[i]));
  JPH::Array<JPH::Vec3> out;
  JPH::ClipPolyVsPlane(in, zjolt::ToJolt(*plane_origin), zjolt::ToJolt(*plane_normal), out);
  return CopyOutVec3(out, out_polygon, capacity, out_count);
}

ZJoltResult zjoltClipPolyVsPoly(const ZJoltVec3 *polygon, uint32_t num_vertices,
                                const ZJoltVec3 *clipping_polygon,
                                uint32_t num_clipping_vertices,
                                const ZJoltVec3 *clipping_polygon_normal,
                                ZJoltVec3 *out_polygon, uint32_t capacity,
                                uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(polygon, clipping_polygon, clipping_polygon_normal, out_count) ||
      num_vertices < 2 || num_clipping_vertices < 3) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  JPH::Array<JPH::Vec3> in, clip;
  in.reserve(num_vertices);
  for (uint32_t i = 0; i < num_vertices; ++i) in.push_back(zjolt::ToJolt(polygon[i]));
  clip.reserve(num_clipping_vertices);
  for (uint32_t i = 0; i < num_clipping_vertices; ++i) {
    clip.push_back(zjolt::ToJolt(clipping_polygon[i]));
  }
  JPH::Array<JPH::Vec3> out;
  JPH::ClipPolyVsPoly(in, clip, zjolt::ToJolt(*clipping_polygon_normal), out);
  return CopyOutVec3(out, out_polygon, capacity, out_count);
}

ZJoltResult zjoltClipPolyVsEdge(const ZJoltVec3 *polygon, uint32_t num_vertices,
                                const ZJoltVec3 *edge_vertex1, const ZJoltVec3 *edge_vertex2,
                                const ZJoltVec3 *clipping_edge_normal, ZJoltVec3 *out_polygon,
                                uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(polygon, edge_vertex1, edge_vertex2, clipping_edge_normal, out_count) ||
      num_vertices < 3) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  JPH::Array<JPH::Vec3> in;
  in.reserve(num_vertices);
  for (uint32_t i = 0; i < num_vertices; ++i) in.push_back(zjolt::ToJolt(polygon[i]));
  JPH::Array<JPH::Vec3> out;
  JPH::ClipPolyVsEdge(in, zjolt::ToJolt(*edge_vertex1), zjolt::ToJolt(*edge_vertex2),
                      zjolt::ToJolt(*clipping_edge_normal), out);
  return CopyOutVec3(out, out_polygon, capacity, out_count);
}

ZJoltResult zjoltClipPolyVsAABox(const ZJoltVec3 *polygon, uint32_t num_vertices,
                                 const ZJoltAABox *box, ZJoltVec3 *out_polygon,
                                 uint32_t capacity, uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(polygon, box, out_count) || num_vertices < 2) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  JPH::Array<JPH::Vec3> in;
  in.reserve(num_vertices);
  for (uint32_t i = 0; i < num_vertices; ++i) in.push_back(zjolt::ToJolt(polygon[i]));
  JPH::Array<JPH::Vec3> out;
  JPH::ClipPolyVsAABox(in, zjolt::ToJolt(*box), out);
  return CopyOutVec3(out, out_polygon, capacity, out_count);
}

//===----------------------------------------------------------------------===//
// Triangle indexing
//===----------------------------------------------------------------------===//

ZJoltResult zjoltIndexify(const ZJoltIndexifyTriangle *triangles, uint32_t num_triangles,
                          float vertex_weld_distance, ZJoltVec3 *out_vertices,
                          uint32_t vertex_capacity, uint32_t *out_num_vertices,
                          ZJoltIndexedTriangle *out_triangles, uint32_t triangle_capacity,
                          uint32_t *out_num_triangles) {
  ZJOLT_ENTER(out_num_vertices, out_num_triangles);
  if (!zjolt::Present(out_num_vertices, out_num_triangles)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (num_triangles > 0 && triangles == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::TriangleList in;
  in.reserve(num_triangles);
  for (uint32_t t = 0; t < num_triangles; ++t) {
    const ZJoltIndexifyTriangle &tri = triangles[t];
    in.emplace_back(zjolt::ToJolt(tri.v[0]), zjolt::ToJolt(tri.v[1]), zjolt::ToJolt(tri.v[2]),
                    tri.material_index, tri.user_data);
  }

  JPH::VertexList out_verts;
  JPH::IndexedTriangleList out_tris;
  JPH::Indexify(in, out_verts, out_tris, vertex_weld_distance);

  *out_num_vertices = static_cast<uint32_t>(out_verts.size());
  *out_num_triangles = static_cast<uint32_t>(out_tris.size());

  ZJoltResult result = ZJOLT_RESULT_OK;
  if (out_vertices != nullptr) {
    if (vertex_capacity < out_verts.size()) {
      result = ZJOLT_RESULT_BUFFER_TOO_SMALL;
    } else {
      for (size_t i = 0; i < out_verts.size(); ++i) {
        out_vertices[i] = ZJoltVec3{out_verts[i].x, out_verts[i].y, out_verts[i].z};
      }
    }
  }
  if (out_triangles != nullptr) {
    if (triangle_capacity < out_tris.size()) {
      result = ZJOLT_RESULT_BUFFER_TOO_SMALL;
    } else {
      for (size_t i = 0; i < out_tris.size(); ++i) {
        const JPH::IndexedTriangle &it = out_tris[i];
        out_triangles[i] = ZJoltIndexedTriangle{{it.mIdx[0], it.mIdx[1], it.mIdx[2]},
                                                it.mMaterialIndex, it.mUserData};
      }
    }
  }
  return result;
}

ZJoltResult zjoltDeindexify(const ZJoltVec3 *vertices, uint32_t num_vertices,
                            const ZJoltIndexedTriangle *triangles, uint32_t num_triangles,
                            ZJoltIndexifyTriangle *out_triangles, uint32_t capacity,
                            uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if ((num_vertices > 0 && vertices == nullptr) || (num_triangles > 0 && triangles == nullptr)) {
    return ZJOLT_RESULT_INVALID_ARGUMENT;
  }
  for (uint32_t t = 0; t < num_triangles; ++t) {
    for (int v = 0; v < 3; ++v) {
      if (triangles[t].indices[v] >= num_vertices) {
        return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                               "an index is out of range of the vertex list");
      }
    }
  }

  *out_count = num_triangles;
  if (out_triangles == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < num_triangles) return ZJOLT_RESULT_BUFFER_TOO_SMALL;

  JPH::VertexList verts;
  verts.reserve(num_vertices);
  for (uint32_t i = 0; i < num_vertices; ++i) verts.emplace_back(vertices[i].x, vertices[i].y,
                                                                 vertices[i].z);

  JPH::IndexedTriangleList tris;
  tris.reserve(num_triangles);
  for (uint32_t i = 0; i < num_triangles; ++i) {
    const ZJoltIndexedTriangle &it = triangles[i];
    tris.emplace_back(it.indices[0], it.indices[1], it.indices[2], it.material_index,
                      it.user_data);
  }

  JPH::TriangleList out;
  JPH::Deindexify(verts, tris, out);
  for (size_t i = 0; i < out.size(); ++i) {
    out_triangles[i].v[0] = ZJoltVec3{out[i].mV[0].x, out[i].mV[0].y, out[i].mV[0].z};
    out_triangles[i].v[1] = ZJoltVec3{out[i].mV[1].x, out[i].mV[1].y, out[i].mV[1].z};
    out_triangles[i].v[2] = ZJoltVec3{out[i].mV[2].x, out[i].mV[2].y, out[i].mV[2].z};
    out_triangles[i].material_index = out[i].mMaterialIndex;
    out_triangles[i].user_data = out[i].mUserData;
  }
  return ZJOLT_RESULT_OK;
}
