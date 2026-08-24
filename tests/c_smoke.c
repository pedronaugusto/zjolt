//===----------------------------------------------------------------------===//
// zjolt — the C boundary on its own.
//
// This file is plain C11 and links against nothing but the installed header.
// It exists to prove two things that a Zig-side test cannot:
//
//   * ffi/zjolt.h is a real C contract, not a private detail of the Zig
//     wrapper. Everything the wrapper reaches for is reachable from C, with no
//     C++ and no Zig anywhere in the picture.
//   * the allocator seam is genuinely in use. Every allocation is counted, and
//     the run fails if a single byte is still outstanding at the end.
//
// It walks the whole v0.1 slice: shapes, save and restore, a world, bodies,
// listeners, a step, queries, bulk read-back, a character and a soft body.
//===----------------------------------------------------------------------===//

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zjolt.h"

//===----------------------------------------------------------------------===//
// Assertions
//===----------------------------------------------------------------------===//

static int g_failures = 0;

#define CHECK(cond, ...)                                     \
  do {                                                       \
    if (!(cond)) {                                           \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);            \
      printf(__VA_ARGS__);                                   \
      printf("\n");                                          \
      ++g_failures;                                          \
    }                                                        \
  } while (0)

#define CHECK_OK(expr)                                                   \
  do {                                                                   \
    ZJoltResult r_ = (expr);                                             \
    CHECK(r_ == ZJOLT_RESULT_OK, "%s -> %s (%s)", #expr, zjoltResultName(r_),   \
          zjoltLastError());                                             \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                            \
  CHECK(fabsf((float)(a) - (float)(b)) <= (tol),                         \
        "%s (%.6f) not within %g of %s (%.6f)", #a, (double)(a), (tol),  \
        #b, (double)(b))

//===----------------------------------------------------------------------===//
// Counting allocator
//
// Jolt frees with only a pointer — no size, no alignment — so a host that
// wants to account for bytes has to record them itself. Doing that here in
// plain C is the point: it is the same asymmetry the Zig wrapper bridges, and
// this shows what it costs a C host (a header, and nothing else).
//===----------------------------------------------------------------------===//

typedef struct BlockHeader {
  void *base;
  size_t size;
} BlockHeader;

static size_t g_live_bytes = 0;
static long g_live_blocks = 0;
static long g_total_allocations = 0;

static void *countedAllocate(void *user, size_t size, size_t alignment) {
  (void)user;
  /* Over-allocate so the header fits before a suitably aligned payload. */
  const size_t prefix = sizeof(BlockHeader) + alignment;
  void *base = malloc(size + prefix);
  if (base == NULL) return NULL;

  uintptr_t raw = (uintptr_t)base + sizeof(BlockHeader);
  uintptr_t aligned = (raw + (alignment - 1)) & ~(uintptr_t)(alignment - 1);
  BlockHeader *header = (BlockHeader *)(aligned - sizeof(BlockHeader));
  header->base = base;
  header->size = size;

  g_live_bytes += size;
  ++g_live_blocks;
  ++g_total_allocations;
  return (void *)aligned;
}

static void countedFree(void *user, void *block) {
  (void)user;
  if (block == NULL) return;
  BlockHeader *header = (BlockHeader *)((uintptr_t)block - sizeof(BlockHeader));
  g_live_bytes -= header->size;
  --g_live_blocks;
  free(header->base);
}

static void *hostAllocate(void *user, size_t size) {
  return countedAllocate(user, size, zjoltDefaultAllocateAlignment());
}

static void *hostReallocate(void *user, void *block, size_t old_size,
                            size_t new_size) {
  void *fresh = hostAllocate(user, new_size);
  if (fresh == NULL) return NULL;
  if (block != NULL) {
    memcpy(fresh, block, old_size < new_size ? old_size : new_size);
    countedFree(user, block);
  }
  return fresh;
}

static void hostFree(void *user, void *block) { countedFree(user, block); }

static void *hostAlignedAllocate(void *user, size_t size, size_t alignment) {
  return countedAllocate(user, size, alignment);
}

static void hostAlignedFree(void *user, void *block) {
  countedFree(user, block);
}

//===----------------------------------------------------------------------===//
// Diagnostic hooks
//===----------------------------------------------------------------------===//

static int g_trace_count = 0;

static void onTrace(void *user, const char *message) {
  (void)user;
  ++g_trace_count;
  printf("  [jolt] %s\n", message);
}

//===----------------------------------------------------------------------===//
// Collision layers
//===----------------------------------------------------------------------===//

enum { LAYER_STATIC = 0, LAYER_MOVING = 1, LAYER_COUNT = 2 };
enum { BP_STATIC = 0, BP_MOVING = 1, BP_COUNT = 2 };

static uint32_t numBroadPhaseLayers(void *user) {
  (void)user;
  return BP_COUNT;
}

static ZJoltBroadPhaseLayer broadPhaseLayerFor(void *user,
                                               ZJoltObjectLayer layer) {
  (void)user;
  return layer == LAYER_STATIC ? BP_STATIC : BP_MOVING;
}

static bool objectVsBroadPhase(void *user, ZJoltObjectLayer layer1,
                               ZJoltBroadPhaseLayer layer2) {
  (void)user;
  /* Static things only care about moving things; moving things care about all. */
  return layer1 == LAYER_STATIC ? layer2 == BP_MOVING : true;
}

static bool objectLayerPair(void *user, ZJoltObjectLayer layer1,
                            ZJoltObjectLayer layer2) {
  (void)user;
  return layer1 == LAYER_STATIC ? layer2 == LAYER_MOVING : true;
}

/* Counted so the test can prove the filters were actually consulted rather
   than assumed. */
static int g_filter_calls = 0;

static bool countingObjectLayerPair(void *user, ZJoltObjectLayer layer1,
                                    ZJoltObjectLayer layer2) {
  ++g_filter_calls;
  return objectLayerPair(user, layer1, layer2);
}

//===----------------------------------------------------------------------===//
// Listeners
//===----------------------------------------------------------------------===//

static int g_contacts_added = 0;
static int g_contacts_persisted = 0;
static int g_contacts_removed = 0;
static int g_validates = 0;
static int g_activations = 0;
static int g_deactivations = 0;
static uint64_t g_last_contact_user_data = 0;
static uint32_t g_max_contact_points = 0;

static ZJoltValidateResult onValidate(void *user,
                                      const ZJoltContactValidateInfo *info) {
  (void)user;
  (void)info;
  ++g_validates;
  return ZJOLT_VALIDATE_RESULT_ACCEPT_ALL_CONTACTS_FOR_THIS_BODY_PAIR;
}

static void onContactAdded(void *user, const ZJoltContactInfo *info,
                           ZJoltContactSettings *settings) {
  (void)user;
  (void)settings;
  ++g_contacts_added;
  g_last_contact_user_data = info->user_data2;
  if (info->manifold.num_points > g_max_contact_points)
    g_max_contact_points = info->manifold.num_points;
}

static void onContactPersisted(void *user, const ZJoltContactInfo *info,
                               ZJoltContactSettings *settings) {
  (void)user;
  (void)info;
  (void)settings;
  ++g_contacts_persisted;
}

static void onContactRemoved(void *user, const ZJoltSubShapeIdPair *pair) {
  (void)user;
  (void)pair;
  ++g_contacts_removed;
}

static void onActivated(void *user, ZJoltBodyId body, uint64_t user_data) {
  (void)user;
  (void)body;
  (void)user_data;
  ++g_activations;
}

static void onDeactivated(void *user, ZJoltBodyId body, uint64_t user_data) {
  (void)user;
  (void)body;
  (void)user_data;
  ++g_deactivations;
}

//===----------------------------------------------------------------------===//
// Soft body contact callbacks
//
// Jolt keeps these in a slot of their own: a soft body's collisions never
// reach ZJoltContactListener, so nothing above sees them. Driving them from C
// is also the only thing that checks ZJoltSoftBodyVertexContact's field order,
// which the ABI guard compares by size and alignment alone.
//===----------------------------------------------------------------------===//

static int g_soft_validates = 0;
static int g_soft_default_settings = 0;
static int g_soft_contacts_added = 0;
static uint32_t g_soft_touching_vertices = 0;
static uint32_t g_soft_sensor_contacts = 0;
static float g_soft_contact_normal_y = 0.0f;
static ZJoltBodyId g_soft_contact_body = ZJOLT_BODY_ID_INVALID;
static ZJoltBodyId g_soft_body_seen = ZJOLT_BODY_ID_INVALID;

static ZJoltSoftBodyValidateResult onSoftValidate(
    void *user, ZJoltBodyId soft_body, ZJoltBodyId other_body,
    ZJoltSoftBodyContactSettings *settings) {
  (void)user;
  (void)other_body;
  ++g_soft_validates;
  g_soft_body_seen = soft_body;
  /* Filled in with its defaults before the call, which is what lets a
     listener that only accepts or rejects leave it alone. Counting the times
     it arrives that way is what proves the struct crossed at all. */
  if (settings->inv_mass_scale1 == 1.0f && settings->inv_mass_scale2 == 1.0f &&
      settings->inv_inertia_scale2 == 1.0f)
    ++g_soft_default_settings;
  return ZJOLT_SOFT_BODY_VALIDATE_RESULT_ACCEPT_CONTACT;
}

static void onSoftContactAdded(void *user, ZJoltBodyId soft_body,
                               const ZJoltSoftBodyManifold *manifold) {
  (void)user;
  ++g_soft_contacts_added;
  g_soft_body_seen = soft_body;

  ZJoltSoftBodyVertexContact contacts[32];
  uint32_t count = 0;
  if (zjoltSoftBodyManifoldGetVertexContacts(manifold, contacts, 32, &count) !=
      ZJOLT_RESULT_OK)
    return;
  if (count > g_soft_touching_vertices) g_soft_touching_vertices = count;
  if (count > 0) {
    g_soft_contact_body = contacts[0].body;
    g_soft_contact_normal_y = contacts[0].normal.y;
  }

  ZJoltBodyId sensors[8];
  uint32_t sensor_count = 0;
  if (zjoltSoftBodyManifoldGetSensorContacts(manifold, sensors, 8,
                                             &sensor_count) == ZJOLT_RESULT_OK) {
    if (sensor_count > g_soft_sensor_contacts)
      g_soft_sensor_contacts = sensor_count;
  }
}

//===----------------------------------------------------------------------===//
// Streaming query callbacks
//
// The point of driving these from C is not that the Zig suite cannot: it is
// that the ABI guard compares pointee types only by size and alignment, so a
// hit struct whose fields the Zig side has in the wrong order still passes it.
// Reading `fraction` and `normal` here, through the header, is what catches
// that.
//===----------------------------------------------------------------------===//

typedef struct RayStream {
  uint32_t count;
  float last_fraction;
  bool fractions_descend;
  bool normals_point_up;
  ZJoltHitAction action;
} RayStream;

static ZJoltHitAction onRayHit(void *user, const ZJoltRayCastHit *hit) {
  RayStream *stream = (RayStream *)user;
  if (stream->count > 0 && hit->fraction >= stream->last_fraction)
    stream->fractions_descend = false;
  if (hit->normal.y <= 0.0f) stream->normals_point_up = false;
  stream->last_fraction = hit->fraction;
  ++stream->count;
  return stream->action;
}

static uint32_t g_overlaps_streamed = 0;

static ZJoltHitAction onOverlapHit(void *user, const ZJoltCollideShapeHit *hit) {
  (void)user;
  (void)hit;
  ++g_overlaps_streamed;
  return ZJOLT_HIT_ACTION_CONTINUE;
}

static uint32_t g_points_streamed = 0;

static ZJoltHitAction onPointHit(void *user, const ZJoltCollidePointHit *hit) {
  (void)user;
  (void)hit;
  ++g_points_streamed;
  return ZJOLT_HIT_ACTION_CONTINUE;
}

/* Rejects one body at the shape level. */
static ZJoltBodyId g_rejected_body = ZJOLT_BODY_ID_INVALID;
static uint32_t g_shape_filter_calls = 0;
static bool g_shape_ids_were_empty = true;

static bool rejectShapesOf(void *user, ZJoltBodyId body,
                           ZJoltSubShapeId sub_shape_id,
                           ZJoltSubShapeId query_sub_shape_id) {
  (void)user;
  ++g_shape_filter_calls;
  if (sub_shape_id != ZJOLT_SUB_SHAPE_ID_EMPTY ||
      query_sub_shape_id != ZJOLT_SUB_SHAPE_ID_EMPTY)
    g_shape_ids_were_empty = false;
  return body != g_rejected_body;
}

//===----------------------------------------------------------------------===//
// Test body
//===----------------------------------------------------------------------===//

static void checkAbiLayout(void) {
  ZJoltAbiLayout layout;
  memset(&layout, 0, sizeof(layout));
  zjoltAbiLayout(&layout);

  CHECK(layout.layout_size == (uint32_t)sizeof(ZJoltAbiLayout),
        "ZJoltAbiLayout size disagrees: library %u, header %u",
        layout.layout_size, (unsigned)sizeof(ZJoltAbiLayout));
  CHECK(layout.config_id == (uint32_t)ZJOLT_CONFIG_ID,
        "config id disagrees: library %u, header %u", layout.config_id,
        (unsigned)ZJOLT_CONFIG_ID);
  CHECK(layout.real_size == (uint32_t)sizeof(ZJoltReal),
        "ZJoltReal width disagrees");
  CHECK(layout.object_layer_size == (uint32_t)sizeof(ZJoltObjectLayer),
        "ZJoltObjectLayer width disagrees");
  CHECK(layout.default_allocate_alignment ==
            (uint32_t)zjoltDefaultAllocateAlignment(),
        "default allocate alignment");
}

/* A unit box centred on the origin, as an indexed triangle mesh. Used both as
   the floor and as the subject of the save/restore round trip. */
static void makeBoxMesh(float half_x, float half_y, float half_z,
                        ZJoltVec3 *vertices, uint32_t *indices) {
  const float xs[2] = {-half_x, half_x};
  const float ys[2] = {-half_y, half_y};
  const float zs[2] = {-half_z, half_z};
  uint32_t v = 0;
  for (int ix = 0; ix < 2; ++ix)
    for (int iy = 0; iy < 2; ++iy)
      for (int iz = 0; iz < 2; ++iz) {
        vertices[v].x = xs[ix];
        vertices[v].y = ys[iy];
        vertices[v].z = zs[iz];
        ++v;
      }

  /* Vertex index is (ix<<2)|(iy<<1)|iz. Winding is chosen so every face points
     outward, which matters because a mesh collides on its front faces. */
  static const uint32_t faces[12][3] = {
      {0, 1, 3}, {0, 3, 2}, /* -x */
      {4, 6, 7}, {4, 7, 5}, /* +x */
      {0, 4, 5}, {0, 5, 1}, /* -y */
      {2, 3, 7}, {2, 7, 6}, /* +y */
      {0, 2, 6}, {0, 6, 4}, /* -z */
      {1, 5, 7}, {1, 7, 3}, /* +z */
  };
  memcpy(indices, faces, sizeof(faces));
}

int main(void) {
  printf("zjolt C smoke test\n");

  const ZJoltAllocator allocator = {
      hostAllocate, hostReallocate, hostFree,
      hostAlignedAllocate, hostAlignedFree, NULL,
  };
  ZJoltInitDesc init = {&allocator, onTrace, NULL, NULL};

  /* Before init, and this is the guard convention rather than an incidental:
     a call that cannot proceed says so, and it clears its out-parameter
     BEFORE the check that fails, so a caller who ignores the result never
     reads uninitialised storage. Both are properties a mechanical sweep of
     the entry points could quietly drop. */
  CHECK(!zjoltIsInitialized(), "should not be initialized yet");
  ZJoltShape *before_init = (ZJoltShape *)&init; /* deliberate garbage */
  CHECK(zjoltShapeCreateSphere(1.0f, 0.0f, NULL, &before_init) ==
            ZJOLT_RESULT_NOT_INITIALIZED,
        "a call before init is refused");
  CHECK(before_init == NULL, "a refused call still clears its out-parameter");
  CHECK(zjoltShapeCreateSphere(1.0f, 0.0f, NULL, NULL) ==
            ZJOLT_RESULT_NOT_INITIALIZED,
        "not being up outranks a bad argument");

  CHECK_OK(zjoltInit(&init));
  CHECK(zjoltIsInitialized(), "should be initialized");
  CHECK(zjoltVersion() == (((uint32_t)ZJOLT_VERSION_MAJOR << 16) |
                           ((uint32_t)ZJOLT_VERSION_MINOR << 8) |
                           (uint32_t)ZJOLT_VERSION_PATCH),
        "library version disagrees with the header it was built from");
  CHECK((zjoltJoltVersion() >> 16) == 5u, "Jolt major version 5");
  CHECK(g_total_allocations > 0, "the allocator seam is not being used");

  checkAbiLayout();

  /* A second init must be refused, and must not disturb the first. */
  CHECK(zjoltInit(&init) == ZJOLT_RESULT_ALREADY_INITIALIZED,
        "double init should be refused");
  CHECK(zjoltInitWithConfig(&init, 0xdeadbeefu) == ZJOLT_RESULT_ALREADY_INITIALIZED,
        "double init is caught before the config check");

  //-------------------------------------------------------------------------
  // Math: quaternion and matrix algebra, reachable from plain C.
  //-------------------------------------------------------------------------

  {
    /* Quaternion multiply matches applying two rotations in sequence to a
       vector: rhs first, then lhs, the Hamilton product order zjolt_math.h
       documents. */
    const ZJoltVec3 y_axis = {0.0f, 1.0f, 0.0f};
    const ZJoltVec3 z_axis = {0.0f, 0.0f, 1.0f};
    ZJoltQuat lhs, rhs, composed;
    CHECK_OK(zjoltQuatFromAxisAngle(&y_axis, 1.57079632679489661923f /* pi/2 */, &lhs));
    CHECK_OK(zjoltQuatFromAxisAngle(&z_axis, 1.57079632679489661923f /* pi/2 */, &rhs));
    zjoltQuatMultiply(&lhs, &rhs, &composed);

    const ZJoltVec3 v = {1.0f, 0.0f, 0.0f};
    ZJoltVec3 once, sequential, direct;
    CHECK_OK(zjoltQuatRotateVector(&rhs, &v, &once));
    CHECK_OK(zjoltQuatRotateVector(&lhs, &once, &sequential));
    CHECK_OK(zjoltQuatRotateVector(&composed, &v, &direct));
    CHECK_NEAR(sequential.x, direct.x, 1e-5f);
    CHECK_NEAR(sequential.y, direct.y, 1e-5f);
    CHECK_NEAR(sequential.z, direct.z, 1e-5f);

    /* A non-unit axis is refused rather than left to Jolt's assert. */
    const ZJoltVec3 not_unit = {2.0f, 0.0f, 0.0f};
    ZJoltQuat unused;
    CHECK(zjoltQuatFromAxisAngle(&not_unit, 1.0f, &unused) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a non-unit axis is refused");

    /* A swing-twist decomposition recomposes to the original rotation. */
    const ZJoltVec3 raw_axis = {0.3f, 0.7f, -0.2f};
    const float raw_len = sqrtf(raw_axis.x * raw_axis.x +
                                raw_axis.y * raw_axis.y +
                                raw_axis.z * raw_axis.z);
    const ZJoltVec3 axis = {raw_axis.x / raw_len, raw_axis.y / raw_len,
                            raw_axis.z / raw_len};
    ZJoltQuat q, swing, twist, recomposed;
    CHECK_OK(zjoltQuatFromAxisAngle(&axis, 1.1f, &q));
    zjoltQuatGetSwingTwist(&q, &swing, &twist);
    zjoltQuatMultiply(&swing, &twist, &recomposed);
    CHECK_NEAR(q.x, recomposed.x, 1e-5f);
    CHECK_NEAR(q.y, recomposed.y, 1e-5f);
    CHECK_NEAR(q.z, recomposed.z, 1e-5f);
    CHECK_NEAR(q.w, recomposed.w, 1e-5f);

    /* Euler angles round-trip well away from the Y = +-pi/2 gimbal lock. */
    const ZJoltVec3 angles = {0.2f, -0.5f, 0.9f};
    ZJoltQuat from_euler;
    ZJoltVec3 back_to_euler;
    zjoltQuatFromEulerAngles(&angles, &from_euler);
    zjoltQuatGetEulerAngles(&from_euler, &back_to_euler);
    CHECK_NEAR(angles.x, back_to_euler.x, 1e-4f);
    CHECK_NEAR(angles.y, back_to_euler.y, 1e-4f);
    CHECK_NEAR(angles.z, back_to_euler.z, 1e-4f);

    /* SLERP stays unit length and matches its endpoints; LERP matches its. */
    const ZJoltQuat identity_q = {0.0f, 0.0f, 0.0f, 1.0f};
    ZJoltQuat lerp_start, lerp_end, slerp_mid;
    zjoltQuatLerp(&identity_q, &rhs, 0.0f, &lerp_start);
    zjoltQuatLerp(&identity_q, &rhs, 1.0f, &lerp_end);
    CHECK_NEAR(lerp_start.w, identity_q.w, 1e-6f);
    CHECK_NEAR(lerp_end.w, rhs.w, 1e-6f);
    zjoltQuatSlerp(&identity_q, &rhs, 0.5f, &slerp_mid);
    const float slerp_len =
        sqrtf(slerp_mid.x * slerp_mid.x + slerp_mid.y * slerp_mid.y +
              slerp_mid.z * slerp_mid.z + slerp_mid.w * slerp_mid.w);
    CHECK_NEAR(slerp_len, 1.0f, 1e-6f);

    /* Vec3/RVec3 lerp, componentwise. */
    const ZJoltVec3 lo = {0.0f, 0.0f, 0.0f};
    const ZJoltVec3 hi = {10.0f, -4.0f, 2.0f};
    ZJoltVec3 mid;
    zjoltVec3Lerp(&lo, &hi, 0.5f, &mid);
    CHECK_NEAR(mid.x, 5.0f, 1e-6f);
    CHECK_NEAR(mid.y, -2.0f, 1e-6f);
    CHECK_NEAR(mid.z, 1.0f, 1e-6f);

    const ZJoltRVec3 rlo = {0.0, 0.0, 0.0};
    const ZJoltRVec3 rhi = {10.0, -4.0, 2.0};
    ZJoltRVec3 rmid;
    zjoltRVec3Lerp(&rlo, &rhi, 0.5f, &rmid);
    CHECK_NEAR((float)rmid.x, 5.0f, 1e-6f);

    /* ZJoltMat44: build, compose with the rigid inverse to the identity, and
       carry the translation through TransformPoint. */
    const ZJoltVec3 translation = {1.0f, 2.0f, 3.0f};
    ZJoltMat44 m, inv, identity_ish;
    CHECK_OK(zjoltMat44FromRotationTranslation(&lhs, &translation, &m));
    zjoltMat44InverseRotationTranslation(&m, &inv);
    zjoltMat44Multiply(&inv, &m, &identity_ish);
    for (int i = 0; i < 16; ++i) {
      const float expected = (i % 5 == 0) ? 1.0f : 0.0f; /* diagonal */
      CHECK_NEAR(identity_ish.m[i], expected, 1e-4f);
    }
    const ZJoltVec3 origin = {0.0f, 0.0f, 0.0f};
    ZJoltVec3 transformed_point;
    zjoltMat44TransformPoint(&m, &origin, &transformed_point);
    CHECK_NEAR(transformed_point.x, translation.x, 1e-5f);
    CHECK_NEAR(transformed_point.y, translation.y, 1e-5f);
    CHECK_NEAR(transformed_point.z, translation.z, 1e-5f);

    /* ZJoltRMat44: the world-space (ZJoltReal) transform used by
       zjoltBodyGetWorldTransform, composed and inverted the same way. */
    const ZJoltRVec3 r_translation = {5.0, -2.0, 100.0};
    ZJoltRMat44 rm, r_inv;
    CHECK_OK(zjoltRMat44FromRotationTranslation(&lhs, &r_translation, &rm));
    zjoltRMat44InverseRotationTranslation(&rm, &r_inv);
    const ZJoltRVec3 original_point = {1.0, 1.0, 1.0};
    ZJoltRVec3 world_point, back_point;
    zjoltRMat44TransformPoint(&rm, &original_point, &world_point);
    zjoltRMat44TransformPoint(&r_inv, &world_point, &back_point);
    CHECK_NEAR((float)back_point.x, (float)original_point.x, 1e-3f);
    CHECK_NEAR((float)back_point.y, (float)original_point.y, 1e-3f);
    CHECK_NEAR((float)back_point.z, (float)original_point.z, 1e-3f);
  }

  //-------------------------------------------------------------------------
  // Shapes
  //-------------------------------------------------------------------------

  ZJoltShape *sphere = NULL;
  CHECK_OK(zjoltShapeCreateSphere(0.5f, 1000.0f, NULL, &sphere));
  CHECK(zjoltShapeGetRefCount(sphere) == 1, "a fresh shape has one reference");
  CHECK(zjoltShapeGetSubType(sphere) == ZJOLT_SHAPE_SUB_TYPE_SPHERE,
        "sphere sub type");

  ZJoltMassProperties mass;
  zjoltShapeGetMassProperties(sphere, &mass);
  CHECK(mass.mass > 0.0f, "a sphere has mass");

  const ZJoltVec3 half_extent = {0.5f, 0.5f, 0.5f};
  ZJoltShape *box = NULL;
  CHECK_OK(zjoltShapeCreateBox(&half_extent, 0.05f, 1000.0f, NULL, &box));

  /* Decorated: the box, offset and rotated inside a parent. */
  const ZJoltVec3 offset = {0.0f, 1.0f, 0.0f};
  const ZJoltQuat identity = {0.0f, 0.0f, 0.0f, 1.0f};
  ZJoltShape *decorated = NULL;
  CHECK_OK(zjoltShapeCreateRotatedTranslated(box, &offset, &identity,
                                             &decorated));
  CHECK(zjoltShapeGetSubType(decorated) ==
            ZJOLT_SHAPE_SUB_TYPE_ROTATED_TRANSLATED,
        "decorated sub type");

  /* Convex hull from the eight corners of a cube. */
  ZJoltVec3 hull_points[8];
  uint32_t mesh_indices[36];
  makeBoxMesh(0.5f, 0.5f, 0.5f, hull_points, mesh_indices);
  ZJoltShape *hull = NULL;
  CHECK_OK(zjoltShapeCreateConvexHull(hull_points, 8, 0.05f, 0.0f, 1000.0f,
                                      NULL, &hull));
  CHECK(zjoltShapeGetSubType(hull) == ZJOLT_SHAPE_SUB_TYPE_CONVEX_HULL,
        "hull sub type");

  /* A wide, flat mesh to act as the floor. */
  ZJoltVec3 floor_vertices[8];
  uint32_t floor_indices[36];
  makeBoxMesh(20.0f, 0.5f, 20.0f, floor_vertices, floor_indices);
  ZJoltShape *floor_shape = NULL;
  CHECK_OK(zjoltShapeCreateMesh(floor_vertices, 8, floor_indices, 12, NULL,
                                NULL, 0, 0, &floor_shape));
  CHECK(zjoltShapeGetSubType(floor_shape) == ZJOLT_SHAPE_SUB_TYPE_MESH,
        "floor is a mesh");

  //-------------------------------------------------------------------------
  // Save and restore
  //-------------------------------------------------------------------------

  size_t saved_size = 0;
  CHECK_OK(zjoltShapeSave(floor_shape, NULL, 0, &saved_size));
  CHECK(saved_size > 0, "a saved mesh is not empty");

  unsigned char *saved = (unsigned char *)malloc(saved_size);
  CHECK(saved != NULL, "test allocation");
  size_t written = 0;
  CHECK_OK(zjoltShapeSave(floor_shape, saved, saved_size, &written));
  CHECK(written == saved_size, "the size query matched the write");

  /* A short buffer must be refused, with the required size still reported —
     including one too small to hold even the container header, which must not
     be written into or pointed past. */
  size_t needed = 0;
  CHECK(zjoltShapeSave(floor_shape, saved, saved_size / 2, &needed) ==
            ZJOLT_RESULT_BUFFER_TOO_SMALL,
        "a short save buffer is refused");
  CHECK(needed == saved_size, "a refused save still reports the size needed");

  unsigned char stub[8];
  needed = 0;
  CHECK(zjoltShapeSave(floor_shape, stub, sizeof(stub), &needed) ==
            ZJOLT_RESULT_BUFFER_TOO_SMALL,
        "a buffer smaller than the header is refused");
  CHECK(needed == saved_size, "...and still reports the size needed");
  needed = 0;
  CHECK(zjoltShapeSave(floor_shape, stub, 0, &needed) ==
            ZJOLT_RESULT_BUFFER_TOO_SMALL,
        "a zero-capacity buffer is refused");

  ZJoltShape *restored = NULL;
  CHECK_OK(zjoltShapeRestore(saved, saved_size, &restored));
  CHECK(zjoltShapeGetSubType(restored) == ZJOLT_SHAPE_SUB_TYPE_MESH,
        "the restored shape is still a mesh");

  ZJoltShapeStats original_stats, restored_stats;
  zjoltShapeGetStats(floor_shape, &original_stats);
  zjoltShapeGetStats(restored, &restored_stats);
  CHECK(original_stats.num_triangles == restored_stats.num_triangles,
        "triangle count survived the round trip: %u vs %u",
        original_stats.num_triangles, restored_stats.num_triangles);

  ZJoltAABox original_bounds, restored_bounds;
  zjoltShapeGetLocalBounds(floor_shape, &original_bounds);
  zjoltShapeGetLocalBounds(restored, &restored_bounds);
  CHECK(original_bounds.min.x == restored_bounds.min.x &&
            original_bounds.max.y == restored_bounds.max.y,
        "bounds survived the round trip");

  /* Truncation must be refused rather than parsed. */
  ZJoltShape *truncated = NULL;
  CHECK(zjoltShapeRestore(saved, saved_size / 2, &truncated) ==
            ZJOLT_RESULT_BAD_FORMAT,
        "a truncated shape buffer is refused");
  CHECK(truncated == NULL, "a refused restore yields no handle");

  zjoltShapeRelease(restored);
  free(saved);

  //-------------------------------------------------------------------------
  // World
  //-------------------------------------------------------------------------

  ZJoltJobSystem *jobs = NULL;
  CHECK_OK(zjoltJobSystemCreateSingleThreaded(ZJOLT_MAX_PHYSICS_JOBS, &jobs));
  CHECK(zjoltJobSystemGetMaxConcurrency(jobs) == 1,
        "the single-threaded scheduler runs one job at a time");

  ZJoltPhysicsSystemDesc system_desc;
  zjoltPhysicsSystemDescInit(&system_desc);
  system_desc.max_bodies = 1024;
  system_desc.broad_phase_layers.num_broad_phase_layers = numBroadPhaseLayers;
  system_desc.broad_phase_layers.broad_phase_layer_for_object_layer =
      broadPhaseLayerFor;
  system_desc.object_vs_broad_phase_filter.should_collide = objectVsBroadPhase;
  system_desc.object_layer_pair_filter.should_collide = countingObjectLayerPair;

  ZJoltPhysicsSystem *system = NULL;
  CHECK_OK(zjoltPhysicsSystemCreate(&system_desc, &system));

  /* A system with no broad-phase interface must be refused, not crash later. */
  ZJoltPhysicsSystemDesc bad_desc;
  zjoltPhysicsSystemDescInit(&bad_desc);
  ZJoltPhysicsSystem *bad_system = NULL;
  CHECK(zjoltPhysicsSystemCreate(&bad_desc, &bad_system) ==
            ZJOLT_RESULT_INVALID_ARGUMENT,
        "a system without a broad phase interface is refused");

  const ZJoltVec3 gravity = {0.0f, -9.81f, 0.0f};
  zjoltPhysicsSystemSetGravity(system, &gravity);
  ZJoltVec3 read_gravity;
  zjoltPhysicsSystemGetGravity(system, &read_gravity);
  CHECK(read_gravity.y == -9.81f, "gravity round-trips");

  const ZJoltContactListener contact_listener = {
      onValidate, onContactAdded, onContactPersisted, onContactRemoved, NULL,
  };
  CHECK_OK(zjoltPhysicsSystemSetContactListener(system, &contact_listener));

  const ZJoltBodyActivationListener activation_listener = {
      onActivated, onDeactivated, NULL,
  };
  CHECK_OK(zjoltPhysicsSystemSetBodyActivationListener(system, &activation_listener));

  //-------------------------------------------------------------------------
  // Bodies
  //-------------------------------------------------------------------------

  ZJoltBodyDesc floor_desc;
  zjoltBodyDescInit(&floor_desc);
  floor_desc.shape = floor_shape;
  floor_desc.motion_type = ZJOLT_MOTION_TYPE_STATIC;
  floor_desc.object_layer = LAYER_STATIC;
  floor_desc.position.y = (ZJoltReal)-0.5;
  floor_desc.user_data = 0xF1002u;

  ZJoltBodyId floor_id = ZJOLT_BODY_ID_INVALID;
  CHECK_OK(zjoltBodyCreateAndAdd(system, &floor_desc,
                                 ZJOLT_ACTIVATION_DONT_ACTIVATE, &floor_id));
  CHECK(floor_id != ZJOLT_BODY_ID_INVALID, "the floor got an id");

  ZJoltBodyDesc ball_desc;
  zjoltBodyDescInit(&ball_desc);
  ball_desc.shape = sphere;
  ball_desc.motion_type = ZJOLT_MOTION_TYPE_DYNAMIC;
  ball_desc.object_layer = LAYER_MOVING;
  ball_desc.position.y = (ZJoltReal)5.0;
  ball_desc.restitution = 0.0f;
  ball_desc.user_data = 0xBA11u;

  ZJoltBodyId ball_id = ZJOLT_BODY_ID_INVALID;
  CHECK_OK(zjoltBodyCreateAndAdd(system, &ball_desc, ZJOLT_ACTIVATION_ACTIVATE,
                                 &ball_id));
  CHECK(zjoltBodyIsAdded(system, ball_id), "the ball is in the simulation");
  CHECK(zjoltBodyGetUserData(system, ball_id) == 0xBA11u, "user data survives");
  CHECK(zjoltPhysicsSystemGetNumBodies(system) == 2, "two bodies");

  /* A body created without a shape must be refused. */
  ZJoltBodyDesc shapeless;
  zjoltBodyDescInit(&shapeless);
  ZJoltBodyId shapeless_id = ZJOLT_BODY_ID_INVALID;
  CHECK(zjoltBodyCreate(system, &shapeless, &shapeless_id) ==
            ZJOLT_RESULT_INVALID_ARGUMENT,
        "a body without a shape is refused");

  /* Jolt asserts its way out of a max_bodies past its id range; this has to
     come back as an error instead. */
  ZJoltPhysicsSystemDesc huge = system_desc;
  huge.max_bodies = 100000000u;
  ZJoltPhysicsSystem *huge_system = NULL;
  CHECK(zjoltPhysicsSystemCreate(&huge, &huge_system) == ZJOLT_RESULT_INVALID_ARGUMENT,
        "a max_bodies past Jolt's id range is refused");
  CHECK(huge_system == NULL, "a refused system yields no handle");

  zjoltPhysicsSystemOptimizeBroadPhase(system);

  //-------------------------------------------------------------------------
  // Step
  //-------------------------------------------------------------------------

  ZJoltRVec3 start_position;
  zjoltBodyGetPositionAndRotation(system, ball_id, &start_position, NULL);
  CHECK(start_position.y > (ZJoltReal)4.9, "the ball starts where it was put");

  for (int i = 0; i < 240; ++i) {
    uint32_t step_error = 0;
    CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, &step_error));
    CHECK(step_error == ZJOLT_UPDATE_ERROR_NONE, "step %d reported errors", i);
  }

  ZJoltRVec3 rest_position;
  zjoltBodyGetPositionAndRotation(system, ball_id, &rest_position, NULL);
  CHECK(rest_position.y < start_position.y, "the ball fell");
  /* The floor top is at y = 0 and the ball has radius 0.5. */
  CHECK(rest_position.y > (ZJoltReal)0.4 && rest_position.y < (ZJoltReal)0.6,
        "the ball came to rest on the floor, at y = %f",
        (double)rest_position.y);

  CHECK(g_filter_calls > 0, "the object layer pair filter was consulted");
  CHECK(g_validates > 0, "contact validate fired");
  CHECK(g_contacts_added > 0, "contact added fired");
  CHECK(g_contacts_persisted > 0, "contact persisted fired");
  CHECK(g_max_contact_points > 0, "a manifold carried at least one point");
  CHECK(g_last_contact_user_data == 0xF1002u || g_last_contact_user_data == 0xBA11u,
        "a contact reported a body's user data");
  CHECK(g_activations > 0, "body activation fired");
  CHECK(g_deactivations > 0, "the ball went to sleep on the floor");

  //-------------------------------------------------------------------------
  // Queries
  //-------------------------------------------------------------------------

  const ZJoltRVec3 ray_origin = {(ZJoltReal)0.0, (ZJoltReal)10.0,
                                 (ZJoltReal)0.0};
  const ZJoltVec3 ray_direction = {0.0f, -20.0f, 0.0f};

  /* The settings default to Jolt's, and NULL means the same thing. Both are
     exercised: the struct here, NULL everywhere below. */
  ZJoltRayCastSettings ray_settings;
  memset(&ray_settings, 0xAB, sizeof(ray_settings));
  zjoltRayCastSettingsInit(&ray_settings);
  CHECK(ray_settings.treat_convex_as_solid,
        "a ray starting inside a convex shape hits it by default");
  CHECK(ray_settings.back_face_mode_triangles == ZJOLT_BACK_FACE_MODE_IGNORE,
        "back faces are ignored by default");

  ZJoltRayCastHit hit;
  bool did_hit = false;
  CHECK_OK(zjoltCastRayClosest(system, &ray_origin, &ray_direction,
                               &ray_settings, NULL, &hit, &did_hit));
  CHECK(did_hit, "a ray straight down hits something");
  CHECK(hit.body == ball_id, "the ray hit the ball first");
  CHECK(hit.normal.y > 0.9f,
        "the top of the ball faces back up the ray: %f", (double)hit.normal.y);

  uint32_t all_hits = 0;
  CHECK_OK(zjoltCastRayAll(system, &ray_origin, &ray_direction, NULL, NULL,
                           NULL, 0, &all_hits));
  CHECK(all_hits >= 2, "the ray passes through the ball and the floor: %u",
        all_hits);

  ZJoltRayCastHit hits[8];
  uint32_t hit_count = 0;
  CHECK_OK(zjoltCastRayAll(system, &ray_origin, &ray_direction, NULL, NULL,
                           hits, 8, &hit_count));
  CHECK(hit_count == all_hits, "the size query matched the fill");

  /* The same ray, streamed. One traversal, no buffer, and the hit struct read
     field by field through the header. */
  RayStream stream;
  memset(&stream, 0, sizeof(stream));
  stream.fractions_descend = true;
  stream.normals_point_up = true;
  stream.action = ZJOLT_HIT_ACTION_CONTINUE;
  CHECK_OK(zjoltCastRayEach(system, &ray_origin, &ray_direction, NULL, NULL,
                            onRayHit, &stream));
  CHECK(stream.count == all_hits,
        "streaming saw every hit the fill did: %u vs %u", stream.count,
        all_hits);
  CHECK(stream.normals_point_up,
        "every surface hit from above faces back up the ray");

  /* Stopping ends it. */
  RayStream stop_after_one;
  memset(&stop_after_one, 0, sizeof(stop_after_one));
  stop_after_one.action = ZJOLT_HIT_ACTION_STOP;
  CHECK_OK(zjoltCastRayEach(system, &ray_origin, &ray_direction, NULL, NULL,
                            onRayHit, &stop_after_one));
  CHECK(stop_after_one.count == 1, "STOP ended the traversal at one hit: %u",
        stop_after_one.count);

  /* Narrowing means every further hit is strictly nearer than the last. */
  RayStream narrowed;
  memset(&narrowed, 0, sizeof(narrowed));
  narrowed.fractions_descend = true;
  narrowed.action = ZJOLT_HIT_ACTION_NARROW;
  CHECK_OK(zjoltCastRayEach(system, &ray_origin, &ray_direction, NULL, NULL,
                            onRayHit, &narrowed));
  CHECK(narrowed.count >= 1, "narrowing still reported a hit");
  CHECK(narrowed.fractions_descend, "narrowed hits arrive nearest last");

  /* A NULL callback is a caller mistake, not an empty result. */
  CHECK(zjoltCastRayEach(system, &ray_origin, &ray_direction, NULL, NULL, NULL,
                         NULL) == ZJOLT_RESULT_INVALID_ARGUMENT,
        "a streaming query with no callback is refused");

  /* A shape filter, the third filter Jolt takes. */
  ZJoltQueryFilters shape_filtered;
  memset(&shape_filtered, 0, sizeof(shape_filtered));
  shape_filtered.shape.should_collide = rejectShapesOf;
  g_rejected_body = ball_id;
  uint32_t filtered_hits = 0;
  CHECK_OK(zjoltCastRayAll(system, &ray_origin, &ray_direction, NULL,
                           &shape_filtered, NULL, 0, &filtered_hits));
  CHECK(g_shape_filter_calls > 0, "the shape filter was consulted");
  CHECK(filtered_hits == all_hits - 1,
        "rejecting the ball's shape dropped exactly its hit: %u of %u",
        filtered_hits, all_hits);
  CHECK(g_shape_ids_were_empty,
        "a shape with no children reports the empty sub-shape id");

  /* A ray that misses everything. */
  const ZJoltRVec3 far_origin = {(ZJoltReal)1000.0, (ZJoltReal)1000.0,
                                 (ZJoltReal)1000.0};
  bool missed = true;
  CHECK_OK(zjoltCastRayClosest(system, &far_origin, &ray_direction, NULL, NULL,
                               &hit, &missed));
  CHECK(!missed, "a ray far from anything misses");

  /* A shape cast down onto the floor. */
  const ZJoltRVec3 cast_start = {(ZJoltReal)5.0, (ZJoltReal)5.0,
                                 (ZJoltReal)0.0};
  const ZJoltVec3 cast_direction = {0.0f, -10.0f, 0.0f};
  ZJoltShapeCastHit shape_hit;
  bool shape_did_hit = false;
  CHECK_OK(zjoltCastShapeClosest(system, sphere, NULL, &cast_start, &identity,
                                 &cast_direction, NULL, NULL, &shape_hit,
                                 &shape_did_hit));
  CHECK(shape_did_hit, "a sphere swept downward reaches the floor");
  CHECK(shape_hit.body == floor_id, "the sweep hit the floor");

  /* The settings a sweep takes, driven through the header itself: the ABI
     guard compares a pointee only by size and alignment, so only C proves the
     struct crossing here is the struct the library reads. The floor is a
     closed mesh, so a sweep starting inside it meets the top face from below —
     a back face, which Jolt's defaults drop. */
  ZJoltShape *probe = NULL;
  CHECK_OK(zjoltShapeCreateSphere(0.1f, 1000.0f, NULL, &probe));
  const ZJoltRVec3 inside_floor = {(ZJoltReal)5.0, (ZJoltReal)-0.5,
                                   (ZJoltReal)0.0};
  const ZJoltVec3 upward = {0.0f, 2.0f, 0.0f};
  bool escaped = true;
  CHECK_OK(zjoltCastShapeClosest(system, probe, NULL, &inside_floor, &identity,
                                 &upward, NULL, NULL, &shape_hit, &escaped));
  CHECK(!escaped, "by default a sweep out of a mesh reports nothing");

  ZJoltShapeCastSettings sweep_settings;
  zjoltShapeCastSettingsInit(&sweep_settings);
  CHECK(sweep_settings.back_face_mode_triangles == ZJOLT_BACK_FACE_MODE_IGNORE,
        "the sweep defaults ignore back faces");
  sweep_settings.back_face_mode_triangles = ZJOLT_BACK_FACE_MODE_COLLIDE;
  CHECK_OK(zjoltCastShapeClosest(system, probe, NULL, &inside_floor, &identity,
                                 &upward, &sweep_settings, NULL, &shape_hit,
                                 &escaped));
  CHECK(escaped, "collecting back faces finds the way out of the mesh");
  CHECK(shape_hit.body == floor_id, "and the way out is through the floor");
  zjoltShapeRelease(probe);

  /* An overlap test where the ball is resting, both ways round. */
  ZJoltCollideShapeHit overlaps[8];
  uint32_t overlap_count = 0;
  CHECK_OK(zjoltCollideShapeAll(system, sphere, NULL, &rest_position, &identity,
                                NULL, NULL, overlaps, 8, &overlap_count));
  CHECK(overlap_count > 0, "a sphere at the ball's position overlaps something");

  CHECK_OK(zjoltCollideShapeEach(system, sphere, NULL, &rest_position,
                                 &identity, NULL, NULL, onOverlapHit, NULL));
  CHECK(g_overlaps_streamed == overlap_count,
        "streaming an overlap saw the same hits: %u vs %u", g_overlaps_streamed,
        overlap_count);

  /* The point tests: inside the resting ball, and in open air. */
  uint32_t inside_count = 0;
  CHECK_OK(zjoltCollidePointAll(system, &rest_position, NULL, NULL, 0,
                                &inside_count));
  CHECK(inside_count >= 1, "the ball's centre is inside the ball");

  CHECK_OK(zjoltCollidePointEach(system, &rest_position, NULL, onPointHit,
                                 NULL));
  CHECK(g_points_streamed == inside_count, "streaming a point test agreed");

  const ZJoltRVec3 empty_air = {(ZJoltReal)0.0, (ZJoltReal)40.0, (ZJoltReal)0.0};
  uint32_t nothing = 1;
  CHECK_OK(zjoltCollidePointAll(system, &empty_air, NULL, NULL, 0, &nothing));
  CHECK(nothing == 0, "open air contains nothing");

  //-------------------------------------------------------------------------
  // Shape versus shape: two placements, no system involved at all
  //-------------------------------------------------------------------------

  ZJoltCollideShapeSettings pair_settings;
  zjoltCollideShapeSettingsInit(&pair_settings);
  CHECK(pair_settings.max_separation_distance == 0.0f,
        "the collide settings default to touching contacts only");

  ZJoltShapeCastSettings pair_cast_settings;
  zjoltShapeCastSettingsInit(&pair_cast_settings);
  CHECK(pair_cast_settings.extra_convex_radius == 0.0f,
        "the cast settings default to no extra convex radius");

  /* Two half-metre boxes, pushed together until they overlap by 0.1. */
  const ZJoltRVec3 pair_origin = {(ZJoltReal)0.0, (ZJoltReal)0.0,
                                  (ZJoltReal)0.0};
  const ZJoltRVec3 pair_near = {(ZJoltReal)0.9, (ZJoltReal)0.0,
                                (ZJoltReal)0.0};
  const ZJoltRVec3 pair_far = {(ZJoltReal)1.5, (ZJoltReal)0.0,
                               (ZJoltReal)0.0};
  ZJoltCollideShapeHit pair_hit;
  bool pair_did_hit = false;
  CHECK_OK(zjoltCollideShapeVsShapeClosest(
      box, NULL, &pair_origin, &identity, box, NULL, &pair_near, &identity,
      NULL, NULL, NULL, &pair_hit, &pair_did_hit));
  CHECK(pair_did_hit, "two boxes 0.9 apart overlap");
  CHECK(pair_hit.penetration_depth > 0.05f &&
            pair_hit.penetration_depth < 0.15f,
        "the overlap is the 0.1 they were pushed together by: %f",
        (double)pair_hit.penetration_depth);
  CHECK(pair_hit.body == ZJOLT_BODY_ID_INVALID,
        "a shape-versus-shape hit names no body");

  pair_did_hit = true;
  CHECK_OK(zjoltCollideShapeVsShapeClosest(
      box, NULL, &pair_origin, &identity, box, NULL, &pair_far, &identity,
      NULL, NULL, NULL, &pair_hit, &pair_did_hit));
  CHECK(!pair_did_hit, "the same boxes 1.5 apart do not overlap");

  /* Sweeping one into the other finds it partway along. */
  const ZJoltVec3 pair_sweep = {2.0f, 0.0f, 0.0f};
  ZJoltShapeCastHit pair_cast_hit;
  bool pair_cast_did_hit = false;
  CHECK_OK(zjoltCastShapeVsShapeClosest(
      box, NULL, &pair_origin, &identity, &pair_sweep, box, NULL, &pair_far,
      &identity, NULL, NULL, NULL, &pair_cast_hit, &pair_cast_did_hit));
  CHECK(pair_cast_did_hit, "a box swept two metres reaches one 1.5 away");
  CHECK(pair_cast_hit.fraction > 0.0f && pair_cast_hit.fraction < 1.0f,
        "the sweep hit partway along: %f", (double)pair_cast_hit.fraction);

  /* And the two-call protocol reports the count either way round. */
  uint32_t pair_count = 0;
  CHECK_OK(zjoltCollideShapeVsShapeAll(box, NULL, &pair_origin, &identity, box,
                                       NULL, &pair_near, &identity, NULL, NULL,
                                       NULL, NULL, 0, &pair_count));
  CHECK(pair_count == 1, "one box against one box is one overlap: %u",
        pair_count);

  //-------------------------------------------------------------------------
  // Bulk read-back
  //-------------------------------------------------------------------------

  uint32_t body_count = 0;
  CHECK_OK(zjoltPhysicsSystemGetBodies(system, NULL, 0, &body_count));
  CHECK(body_count == 2, "two bodies in the bulk listing");

  ZJoltBodyId body_ids[4];
  CHECK_OK(zjoltPhysicsSystemGetBodies(system, body_ids, 4, &body_count));

  ZJoltRVec3 positions[4];
  ZJoltQuat rotations[4];
  uint32_t missing = 0;
  CHECK_OK(zjoltBodyGetTransforms(system, body_ids, body_count, positions,
                                  rotations, &missing));
  CHECK(missing == 0, "no live body was reported missing");

  /* A stale id is reported, not fatal. */
  ZJoltBodyId stale[1] = {0x00FFFFFEu};
  CHECK_OK(zjoltBodyGetTransforms(system, stale, 1, positions, rotations,
                                  &missing));
  CHECK(missing == 1, "a stale id is counted as missing");

  //-------------------------------------------------------------------------
  // Locks
  //-------------------------------------------------------------------------

  ZJoltBodyLock lock;
  zjoltBodyLockRead(system, ball_id, &lock);
  CHECK(lock.body != NULL, "locking a live body succeeds");
  if (lock.body != NULL) {
    CHECK(zjoltBodyGetId(lock.body) == ball_id, "the locked body is the ball");
    CHECK(zjoltBodyGetUserDataLocked(lock.body) == 0xBA11u,
          "the locked body carries its user data");
    CHECK(zjoltBodyGetShapeLocked(lock.body) == sphere,
          "the locked body still has its shape");
  }
  zjoltBodyLockReadRelease(&lock);
  CHECK(lock.body == NULL, "releasing clears the lock");

  ZJoltBodyLock stale_lock;
  zjoltBodyLockRead(system, 0x00FFFFFEu, &stale_lock);
  CHECK(stale_lock.body == NULL, "locking a stale id yields no body");
  zjoltBodyLockReadRelease(&stale_lock);

  //-------------------------------------------------------------------------
  // Teleport and kinematic motion
  //-------------------------------------------------------------------------

  const ZJoltRVec3 teleport_to = {(ZJoltReal)3.0, (ZJoltReal)2.0,
                                  (ZJoltReal)0.0};
  zjoltBodySetPositionAndRotation(system, ball_id, &teleport_to, &identity,
                                  ZJOLT_ACTIVATION_ACTIVATE);
  ZJoltRVec3 after_teleport;
  zjoltBodyGetPositionAndRotation(system, ball_id, &after_teleport, NULL);
  CHECK(after_teleport.x == (ZJoltReal)3.0, "the teleport took effect");

  ZJoltBodyDesc platform_desc;
  zjoltBodyDescInit(&platform_desc);
  platform_desc.shape = box;
  platform_desc.motion_type = ZJOLT_MOTION_TYPE_KINEMATIC;
  platform_desc.object_layer = LAYER_MOVING;
  platform_desc.position.y = (ZJoltReal)3.0;
  ZJoltBodyId platform_id = ZJOLT_BODY_ID_INVALID;
  CHECK_OK(zjoltBodyCreateAndAdd(system, &platform_desc,
                                 ZJOLT_ACTIVATION_ACTIVATE, &platform_id));

  const ZJoltRVec3 platform_target = {(ZJoltReal)0.0, (ZJoltReal)4.0,
                                      (ZJoltReal)0.0};
  zjoltBodyMoveKinematic(system, platform_id, &platform_target, &identity,
                         1.0f / 60.0f);
  CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));
  ZJoltRVec3 platform_now;
  zjoltBodyGetPositionAndRotation(system, platform_id, &platform_now, NULL);
  CHECK(platform_now.y > (ZJoltReal)3.0,
        "the kinematic platform moved toward its target");

  zjoltBodySetMotionType(system, platform_id, ZJOLT_MOTION_TYPE_DYNAMIC,
                         ZJOLT_ACTIVATION_ACTIVATE);
  CHECK(zjoltBodyGetMotionType(system, platform_id) ==
            ZJOLT_MOTION_TYPE_DYNAMIC,
        "the motion type changed");

  //-------------------------------------------------------------------------
  // Character
  //-------------------------------------------------------------------------

  ZJoltShape *character_shape = NULL;
  CHECK_OK(zjoltShapeCreateCapsule(0.5f, 0.3f, 1000.0f, NULL,
                                   &character_shape));

  ZJoltCharacterDesc character_desc;
  zjoltCharacterDescInit(&character_desc);
  character_desc.shape = character_shape;
  character_desc.position.x = (ZJoltReal)-5.0;
  character_desc.position.y = (ZJoltReal)0.81; /* base just above the floor */
  character_desc.rotation = identity;

  ZJoltCharacter *character = NULL;
  CHECK_OK(zjoltCharacterCreate(system, &character_desc, &character));
  CHECK(zjoltCharacterGetShape(character) == character_shape,
        "the character kept its shape");

  const ZJoltVec3 character_gravity = {0.0f, -9.81f, 0.0f};
  ZJoltCharacterUpdateSettings character_update;
  zjoltCharacterUpdateSettingsInit(&character_update);

  for (int i = 0; i < 60; ++i) {
    ZJoltVec3 velocity;
    zjoltCharacterGetLinearVelocity(character, &velocity);
    if (zjoltCharacterGetGroundState(character) == ZJOLT_GROUND_STATE_ON_GROUND) {
      velocity.y = 0.0f;
    } else {
      velocity.y += character_gravity.y * (1.0f / 60.0f);
    }
    zjoltCharacterSetLinearVelocity(character, &velocity);
    CHECK_OK(zjoltCharacterUpdate(character, 1.0f / 60.0f, &character_gravity,
                                  &character_update, NULL));
  }

  CHECK(zjoltCharacterGetGroundState(character) == ZJOLT_GROUND_STATE_ON_GROUND,
        "the character is standing on the floor");
  CHECK(zjoltCharacterIsSupported(character), "the character is supported");
  CHECK(zjoltCharacterGetGroundBodyId(character) == floor_id,
        "the character is standing on the floor body");

  ZJoltVec3 ground_normal;
  zjoltCharacterGetGroundNormal(character, &ground_normal);
  CHECK(ground_normal.y > 0.9f, "the ground normal points up");

  ZJoltRVec3 character_position;
  zjoltCharacterGetPosition(character, &character_position);
  CHECK(character_position.y > (ZJoltReal)0.7 &&
            character_position.y < (ZJoltReal)0.95,
        "the character settled on the floor at y = %f",
        (double)character_position.y);

  CHECK(zjoltCharacterGetListener(character) == NULL,
        "nothing is listening to this character");

  /* The supporting volume, read back and put back. Driven from C because the
     pair crosses as a vector out-parameter beside a bare float, which the ABI
     guard compares by width alone. */
  ZJoltVec3 support_normal;
  float support_distance = 0.0f;
  zjoltCharacterGetSupportingVolume(character, &support_normal,
                                    &support_distance);
  CHECK(support_normal.y > 0.9f, "the supporting volume plane faces up: %f",
        (double)support_normal.y);

  /* Lifted far above the character, no contact is behind it any more, so
     nothing may support the character — though it is still touching the
     floor and still says so. */
  const ZJoltVec3 up_normal = {0.0f, 1.0f, 0.0f};
  CHECK_OK(zjoltCharacterSetSupportingVolume(character, &up_normal, 100.0f));
  CHECK_OK(zjoltCharacterRefreshContacts(character, NULL));
  CHECK(zjoltCharacterGetGroundState(character) ==
            ZJOLT_GROUND_STATE_NOT_SUPPORTED,
        "a plane above every contact leaves the character unsupported");
  CHECK(zjoltCharacterGetGroundBodyId(character) == floor_id,
        "but the floor is still the body being touched");

  const ZJoltVec3 no_normal = {0.0f, 0.0f, 0.0f};
  CHECK(zjoltCharacterSetSupportingVolume(character, &no_normal, 0.0f) ==
            ZJOLT_RESULT_INVALID_ARGUMENT,
        "a plane with no direction is refused");

  CHECK_OK(zjoltCharacterSetSupportingVolume(character, &support_normal,
                                             support_distance));
  CHECK_OK(zjoltCharacterRefreshContacts(character, NULL));
  CHECK(zjoltCharacterGetGroundState(character) == ZJOLT_GROUND_STATE_ON_GROUND,
        "and putting the plane back puts the character back on the ground");

  /* Overlaps at a placement, into a caller-owned array of a struct that
     exists nowhere else. */
  ZJoltCharacterCollisionHit character_hits[16];
  uint32_t character_hit_count = 0;
  CHECK_OK(zjoltCharacterCheckCollision(character, &character_position, NULL,
                                        NULL, 0.1f, NULL, NULL,
                                        character_hits, 16,
                                        &character_hit_count));
  CHECK(character_hit_count > 0, "standing where it stands, it overlaps the floor");
  CHECK(character_hits[0].body == floor_id, "and the overlap names the floor");
  CHECK(character_hits[0].character_id == ZJOLT_CHARACTER_ID_INVALID,
        "a body overlap names no character");

  ZJoltRVec3 high_above = character_position;
  high_above.y += (ZJoltReal)50.0;
  uint32_t empty_count = 1;
  CHECK_OK(zjoltCharacterCheckCollision(character, &high_above, NULL, NULL,
                                        0.1f, NULL, NULL, NULL, 0,
                                        &empty_count));
  CHECK(empty_count == 0, "fifty metres up there is nothing to overlap: %u",
        empty_count);

  /* A CharacterVirtual is in no broad phase, so this is the only handle a
     cast can reach it through. */
  ZJoltTransformedShape *character_volume = NULL;
  CHECK_OK(zjoltCharacterGetTransformedShape(character, &character_volume));

  ZJoltRVec3 side_on = character_position;
  side_on.x -= (ZJoltReal)5.0;
  const ZJoltVec3 side_along = {10.0f, 0.0f, 0.0f};
  ZJoltRayCastHit character_ray;
  bool character_ray_hit = false;
  CHECK_OK(zjoltCastRayClosest(system, &side_on, &side_along, NULL, NULL,
                               &character_ray, &character_ray_hit));
  CHECK(!character_ray_hit, "no query on the system finds a virtual character");

  CHECK_OK(zjoltTransformedShapeCastRayClosest(character_volume, &side_on,
                                               &side_along, NULL, NULL,
                                               &character_ray,
                                               &character_ray_hit));
  CHECK(character_ray_hit, "its transformed shape is what a cast can hit");
  CHECK(character_ray.fraction > 0.4f && character_ray.fraction < 0.55f,
        "and it hits the front of the capsule at %f",
        (double)character_ray.fraction);
  zjoltTransformedShapeDestroy(character_volume);

  /* The inner body may carry a shape of its own. This character has none, so
     asking is refused rather than quietly doing nothing. */
  CHECK(zjoltCharacterSetInnerBodyShape(character, character_shape) ==
            ZJOLT_RESULT_INVALID_ARGUMENT,
        "a character with no inner body has nothing to give a shape to");

  //-------------------------------------------------------------------------
  // Materials, and the shapes that carry more than one
  //
  // Driven from C on purpose. The ABI cross-check compares pointee types only
  // by size and alignment, so a `const ZJoltPhysicsMaterial *const *` declared
  // on the Zig side as an array of something else the same width passes it
  // and produces a wrong answer only here.
  //-------------------------------------------------------------------------

  ZJoltColor gravel_color = {160, 150, 140, 255};
  ZJoltPhysicsMaterial *gravel = NULL;
  CHECK_OK(zjoltPhysicsMaterialCreate("gravel", &gravel_color, &gravel));
  CHECK(zjoltPhysicsMaterialGetRefCount(gravel) == 1,
        "a fresh material has one reference");
  CHECK(strcmp(zjoltPhysicsMaterialGetDebugName(gravel), "gravel") == 0,
        "a material remembers its debug name");

  ZJoltColor read_back;
  zjoltPhysicsMaterialGetDebugColor(gravel, &read_back);
  CHECK(read_back.r == 160 && read_back.g == 150 && read_back.b == 140 &&
            read_back.a == 255,
        "a material remembers its debug colour");

  ZJoltPhysicsMaterial *metal = NULL;
  CHECK_OK(zjoltPhysicsMaterialCreate("metal", NULL, &metal));

  /* A convex shape carries one material, read at the empty sub-shape id.
     Jolt ASSERTS the id is empty there, so any other value would abort this
     process rather than return. */
  ZJoltShape *pebble = NULL;
  CHECK_OK(zjoltShapeCreateSphere(0.5f, 1000.0f, gravel, &pebble));
  CHECK(zjoltPhysicsMaterialGetRefCount(gravel) == 2,
        "the shape took a reference of its own");
  CHECK(zjoltShapeGetMaterial(pebble, ZJOLT_SUB_SHAPE_ID_EMPTY) == gravel,
        "a convex shape reports the material it was built with");

  /* A shape built without one reports the shared default, not NULL. */
  ZJoltShape *plain = NULL;
  CHECK_OK(zjoltShapeCreateSphere(0.5f, 1000.0f, NULL, &plain));
  CHECK(zjoltShapeGetMaterial(plain, ZJOLT_SUB_SHAPE_ID_EMPTY) ==
            zjoltPhysicsMaterialDefault(),
        "no material means the default, which is a material and not a NULL");
  zjoltShapeRelease(plain);

  /* A mesh carries one per triangle, and a hit's sub-shape id is what maps
     back to it — Jolt reorders the triangles while it builds the tree, so
     nothing else can. Two triangles forming a flat quad, wound to face up:
     triangle 0 covers x < z, triangle 1 covers x > z. */
  const ZJoltVec3 quad_vertices[4] = {
      {0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f},
      {2.0f, 0.0f, 2.0f}, {0.0f, 0.0f, 2.0f},
  };
  const uint32_t quad_indices[6] = {0, 3, 2, 0, 2, 1};
  const uint32_t quad_triangle_materials[2] = {0, 1};
  const ZJoltPhysicsMaterial *quad_materials[2] = {gravel, metal};

  ZJoltShape *quad = NULL;
  CHECK_OK(zjoltShapeCreateMesh(quad_vertices, 4, quad_indices, 2,
                                quad_triangle_materials, quad_materials, 2, 0,
                                &quad));

  ZJoltBodyDesc quad_desc;
  zjoltBodyDescInit(&quad_desc);
  quad_desc.shape = quad;
  quad_desc.object_layer = LAYER_STATIC;
  quad_desc.motion_type = ZJOLT_MOTION_TYPE_STATIC;
  quad_desc.position.y = (ZJoltReal)3.0;

  ZJoltBodyId quad_id = ZJOLT_BODY_ID_INVALID;
  CHECK_OK(zjoltBodyCreateAndAdd(system, &quad_desc,
                                 ZJOLT_ACTIVATION_DONT_ACTIVATE, &quad_id));
  zjoltPhysicsSystemOptimizeBroadPhase(system);

  const ZJoltVec3 straight_down = {0.0f, -20.0f, 0.0f};
  ZJoltRVec3 over_gravel = {(ZJoltReal)0.5, (ZJoltReal)10.0, (ZJoltReal)1.5};
  ZJoltRVec3 over_metal = {(ZJoltReal)1.5, (ZJoltReal)10.0, (ZJoltReal)0.5};

  ZJoltRayCastHit gravel_hit, metal_hit;
  bool got_gravel = false, got_metal = false;
  CHECK_OK(zjoltCastRayClosest(system, &over_gravel, &straight_down, NULL, NULL,
                               &gravel_hit, &got_gravel));
  CHECK_OK(zjoltCastRayClosest(system, &over_metal, &straight_down, NULL, NULL,
                               &metal_hit, &got_metal));
  CHECK(got_gravel && got_metal, "both halves of the quad were hit");
  CHECK(gravel_hit.body == quad_id && metal_hit.body == quad_id,
        "both rays found the quad");
  CHECK(gravel_hit.sub_shape_id != metal_hit.sub_shape_id,
        "the two triangles are different leaves of one shape");
  CHECK(zjoltShapeGetMaterial(quad, gravel_hit.sub_shape_id) == gravel,
        "the triangle at x < z kept its material");
  CHECK(zjoltShapeGetMaterial(quad, metal_hit.sub_shape_id) == metal,
        "the triangle at x > z kept its material");
  CHECK(zjoltBodyGetMaterial(system, quad_id, metal_hit.sub_shape_id) == metal,
        "and the body answers the same as the shape");

  /* Not a way to test whether a body exists: Jolt's body lock fails and it
     answers with the shared default rather than reporting anything. */
  CHECK(zjoltBodyGetMaterial(system, ZJOLT_BODY_ID_INVALID,
                             ZJOLT_SUB_SHAPE_ID_EMPTY) ==
            zjoltPhysicsMaterialDefault(),
        "a stale body id reads as the default material, not as a failure");

  zjoltBodyDestroy(system, quad_id);
  zjoltShapeRelease(quad);
  zjoltShapeRelease(pebble);
  zjoltPhysicsMaterialRelease(metal);
  zjoltPhysicsMaterialRelease(gravel);

  //-------------------------------------------------------------------------
  // A soft body, driven the way a host drives one
  //
  // The ABI guard on the Zig side compares pointee types by size and alignment
  // only, so a `float *` declared as a pointer to a 32-bit integer passes it.
  // This is what does not: every per-instance soft-body call with a pointer in
  // it, reached through the header itself.
  //-------------------------------------------------------------------------

  {
    /* A unit cube, every face wound counter-clockwise seen from outside so the
       enclosed volume comes back positive. */
    static const float kCorners[8][3] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, 0.5f},
        {-0.5f, -0.5f, 0.5f},  {-0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, -0.5f},
        {0.5f, 0.5f, 0.5f},    {-0.5f, 0.5f, 0.5f},
    };
    static const uint32_t kTriangles[12][3] = {
        {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {3, 2, 6}, {3, 6, 7},
        {0, 4, 5}, {0, 5, 1}, {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2},
    };

    ZJoltSoftBodyVertex cube_vertices[8];
    ZJoltSoftBodyFace cube_faces[12];
    for (int i = 0; i < 8; ++i) {
      cube_vertices[i].position.x = kCorners[i][0];
      cube_vertices[i].position.y = kCorners[i][1];
      cube_vertices[i].position.z = kCorners[i][2];
      cube_vertices[i].velocity.x = 0.0f;
      cube_vertices[i].velocity.y = 0.0f;
      cube_vertices[i].velocity.z = 0.0f;
      cube_vertices[i].inv_mass = 1.0f;
    }
    for (int i = 0; i < 12; ++i) {
      cube_faces[i].vertex[0] = kTriangles[i][0];
      cube_faces[i].vertex[1] = kTriangles[i][1];
      cube_faces[i].vertex[2] = kTriangles[i][2];
      cube_faces[i].material_index = 0;
    }

    ZJoltSoftBodySharedSettings *cloth = NULL;
    CHECK_OK(zjoltSoftBodySharedSettingsCreate(&cloth));
    CHECK_OK(zjoltSoftBodySharedSettingsAddVertices(cloth, cube_vertices, 8));
    CHECK_OK(zjoltSoftBodySharedSettingsAddFaces(cloth, cube_faces, 12));

    /* The precondition that used to reach Jolt's own indexing unchecked. */
    ZJoltSoftBodyFace bad_face = cube_faces[0];
    bad_face.vertex[2] = 8;
    CHECK(zjoltSoftBodySharedSettingsAddFaces(cloth, &bad_face, 1) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a face naming a vertex that does not exist is refused");

    ZJoltSoftBodySkinned bad_skin;
    memset(&bad_skin, 0, sizeof(bad_skin));
    bad_skin.vertex = 8;
    bad_skin.max_distance = 1.0f;
    CHECK(zjoltSoftBodySharedSettingsAddSkinnedConstraints(cloth, &bad_skin,
                                                           1) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a skinned constraint naming a vertex that does not exist is refused");

    ZJoltSoftBodyVertexAttributes attributes;
    zjoltSoftBodyVertexAttributesInit(&attributes);
    CHECK_OK(zjoltSoftBodySharedSettingsCreateConstraints(
        cloth, &attributes, 1, ZJOLT_SOFT_BODY_BEND_TYPE_DIHEDRAL, 0.1396f));
    CHECK_OK(zjoltSoftBodySharedSettingsCalculateSkinnedConstraintNormals(cloth));
    zjoltSoftBodySharedSettingsOptimize(cloth);

    ZJoltSoftBodyDesc soft_desc;
    zjoltSoftBodyDescInit(&soft_desc);
    soft_desc.shared_settings = cloth;
    soft_desc.object_layer = LAYER_MOVING;
    soft_desc.position.y = 4;
    soft_desc.pressure = 20.0f;
    soft_desc.vertex_radius = 0.01f;
    soft_desc.allow_sleeping = false;

    /* Both of the values that reach a Jolt which does not check them. */
    ZJoltSoftBodyDesc refused = soft_desc;
    refused.vertex_radius = -1.0f;
    ZJoltBodyId nowhere = 0;
    CHECK(zjoltSoftBodyCreate(system, &refused, &nowhere) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a negative vertex radius is refused");
    CHECK(nowhere == ZJOLT_BODY_ID_INVALID, "and no body came back");
    refused = soft_desc;
    refused.num_iterations = 0;
    CHECK(zjoltSoftBodyCreate(system, &refused, &nowhere) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "zero solver iterations is refused");

    ZJoltBodyId soft_id = ZJOLT_BODY_ID_INVALID;
    CHECK_OK(zjoltSoftBodyCreateAndAdd(system, &soft_desc,
                                       ZJOLT_ACTIVATION_ACTIVATE, &soft_id));

    uint32_t vertex_count = 0;
    CHECK_OK(zjoltSoftBodyGetVertexStates(system, soft_id, NULL, 0,
                                          &vertex_count));
    CHECK(vertex_count == 8, "the soft body has its eight vertices");

    float volume = 0.0f;
    CHECK_OK(zjoltSoftBodyGetVolume(system, soft_id, &volume));
    CHECK(volume > 0.9f && volume < 1.1f, "a unit cube encloses a unit volume");

    ZJoltSoftBodyVertexState states[8];
    CHECK_OK(zjoltSoftBodyGetVertexStates(system, soft_id, states, 8,
                                          &vertex_count));
    ZJoltRVec3 com_before;
    memset(&com_before, 0, sizeof(com_before));
    zjoltBodyGetCenterOfMassPosition(system, soft_id, &com_before);

    for (int i = 0; i < 60; ++i)
      CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));

    ZJoltRVec3 com_after;
    memset(&com_after, 0, sizeof(com_after));
    zjoltBodyGetCenterOfMassPosition(system, soft_id, &com_after);
    CHECK(com_after.y < com_before.y - 1.0, "the soft body fell");
    CHECK_OK(zjoltSoftBodyGetVertexStates(system, soft_id, states, 8,
                                          &vertex_count));
    CHECK_OK(zjoltSoftBodyGetVolume(system, soft_id, &volume));
    CHECK(volume > 0.25f, "and pressure held it open");

    ZJoltAABox local_bounds;
    memset(&local_bounds, 0, sizeof(local_bounds));
    CHECK_OK(zjoltSoftBodyGetLocalBounds(system, soft_id, &local_bounds));
    CHECK(local_bounds.max.y > local_bounds.min.y, "the local bounds enclose it");

    /* The live properties, through the header's own types. */
    uint32_t iterations = 0;
    CHECK_OK(zjoltSoftBodyGetNumIterations(system, soft_id, &iterations));
    CHECK(iterations == 5, "iteration count is Jolt's default");
    CHECK_OK(zjoltSoftBodySetNumIterations(system, soft_id, 3));
    CHECK_OK(zjoltSoftBodyGetNumIterations(system, soft_id, &iterations));
    CHECK(iterations == 3, "and it took the change");
    CHECK(zjoltSoftBodySetNumIterations(system, soft_id, 0) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "zero iterations is refused on a live body too");

    float pressure = 0.0f;
    CHECK_OK(zjoltSoftBodyGetPressure(system, soft_id, &pressure));
    CHECK(pressure > 19.0f && pressure < 21.0f, "pressure reads back");
    CHECK_OK(zjoltSoftBodySetPressure(system, soft_id, 0.0f));

    bool flag = false;
    CHECK_OK(zjoltSoftBodyGetUpdatePosition(system, soft_id, &flag));
    CHECK(flag, "update position is on by default");
    CHECK_OK(zjoltSoftBodySetUpdatePosition(system, soft_id, false));
    CHECK_OK(zjoltSoftBodyGetFacesDoubleSided(system, soft_id, &flag));
    CHECK(!flag, "faces are single sided by default");
    CHECK_OK(zjoltSoftBodySetFacesDoubleSided(system, soft_id, true));
    CHECK_OK(zjoltSoftBodyGetEnableSkinConstraints(system, soft_id, &flag));
    CHECK(flag, "skin constraints are on by default");
    CHECK_OK(zjoltSoftBodySetEnableSkinConstraints(system, soft_id, false));

    float radius = -1.0f;
    CHECK_OK(zjoltSoftBodyGetVertexRadius(system, soft_id, &radius));
    CHECK(radius > 0.0f, "the vertex radius reads back");
    CHECK(zjoltSoftBodySetVertexRadius(system, soft_id, -1.0f) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "and a negative one is refused");

    float multiplier = 0.0f;
    CHECK_OK(
        zjoltSoftBodyGetSkinnedMaxDistanceMultiplier(system, soft_id, &multiplier));
    CHECK(multiplier > 0.9f && multiplier < 1.1f, "the skin multiplier is 1");
    CHECK_OK(
        zjoltSoftBodySetSkinnedMaxDistanceMultiplier(system, soft_id, 0.5f));

    /* Per-vertex control, and the index guard at both ends of it. */
    ZJoltVec3 vertex_velocity = {0.0f, 3.0f, 0.0f};
    CHECK_OK(zjoltSoftBodySetVertexVelocity(system, soft_id, 1,
                                            &vertex_velocity));
    vertex_velocity.y = 0.0f;
    CHECK_OK(zjoltSoftBodyGetVertexVelocity(system, soft_id, 1,
                                            &vertex_velocity));
    CHECK(vertex_velocity.y > 2.9f && vertex_velocity.y < 3.1f,
          "a hand-written vertex velocity reads back");

    float inv_mass = -1.0f;
    CHECK_OK(zjoltSoftBodyGetVertexInvMass(system, soft_id, 0, &inv_mass));
    CHECK(inv_mass > 0.9f && inv_mass < 1.1f, "the vertex inverse mass is 1");
    CHECK_OK(zjoltSoftBodySetVertexInvMass(system, soft_id, 0, 0.0f));
    CHECK_OK(zjoltSoftBodyCalculateMassAndInertia(system, soft_id));
    CHECK(zjoltSoftBodyGetVertexInvMass(system, soft_id, 8, &inv_mass) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a vertex index past the end is refused");
    CHECK(zjoltSoftBodySetVertexVelocity(system, soft_id, 8,
                                         &vertex_velocity) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "and so is writing one");

    /* Skinning a body whose settings carry no skinned constraints has no
       per-instance state to write into, which Jolt only notices by asserting. */
    CHECK(zjoltSoftBodySkinVertices(system, soft_id, NULL, 0, false) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "skinning a body with no skinned constraints is refused");

    /* A rigid body is not a soft body, and is told so. */
    CHECK(zjoltSoftBodyGetVolume(system, ball_id, &volume) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a rigid body has no soft body motion properties");

    CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));

    zjoltBodyDestroy(system, soft_id);
    zjoltSoftBodySharedSettingsRelease(cloth);
  }

  //-------------------------------------------------------------------------
  // Hair, on Jolt's CPU compute backend
  //
  // Same scenario as the Zig suite: a groom of two strands of different
  // lengths on a one-triangle scalp, stepped, read back four quantities at a
  // time, baked and restored. Every readback here hands the header an array
  // whose element type the ABI guard can only compare by size — a float3 and a
  // three-int struct are the same eight bytes to it — so this is the side that
  // says the layouts agree.
  //-------------------------------------------------------------------------

  if (zjoltComputeIsCpuSupported()) {
    ZJoltComputeSystem *compute = NULL;
    CHECK_OK(zjoltComputeSystemCreateCpu(&compute));

    ZJoltHairMaterial material;
    zjoltHairMaterialInit(&material);
    material.enable_collision = false;
    material.bend_compliance = 1.0e-3f;
    material.stretch_compliance = 1.0e-5f;
    material.gravity_factor.min = 1.0f;
    material.gravity_factor.max = 1.0f;
    material.gravity_factor.min_fraction = 0.0f;
    material.gravity_factor.max_fraction = 1.0f;
    material.global_pose.min = 0.0f;
    material.global_pose.max = 0.0f;
    material.grid_velocity_factor.min = 0.0f;
    material.grid_velocity_factor.max = 0.0f;
    material.grid_density_force_factor = 0.0f;
    material.simulation_strands_fraction = 1.0f;

    /* The compliance curve reads the four multipliers as 0%, 33%, 66% and
       100% of the strand, and the ends of the strand are the ends of it. */
    material.bend_compliance_multiplier[0] = 1.0f;
    material.bend_compliance_multiplier[1] = 10.0f;
    material.bend_compliance_multiplier[2] = 10.0f;
    material.bend_compliance_multiplier[3] = 1.0f;
    float compliance = 0.0f;
    CHECK_OK(zjoltHairMaterialGetBendCompliance(&material, 0.0f, &compliance));
    CHECK_NEAR(compliance, 1.0e-3f, 1e-9f);
    CHECK_OK(zjoltHairMaterialGetBendCompliance(&material, 0.5f, &compliance));
    CHECK_NEAR(compliance, 1.0e-2f, 1e-8f);
    CHECK(zjoltHairMaterialGetBendCompliance(&material, 1.5f, &compliance) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a strand fraction past the tip is refused rather than truncated");

    /* A gradient ramps between its two fractions and clamps outside them. */
    ZJoltHairGradient ramp = {0.0f, 1.0f, 0.2f, 0.8f};
    float sampled = -1.0f;
    CHECK_OK(zjoltHairGradientSample(&ramp, 0.0f, &sampled));
    CHECK_NEAR(sampled, 0.0f, 1e-6f);
    CHECK_OK(zjoltHairGradientSample(&ramp, 0.5f, &sampled));
    CHECK_NEAR(sampled, 0.5f, 1e-6f);
    CHECK_OK(zjoltHairGradientSample(&ramp, 1.0f, &sampled));
    CHECK_NEAR(sampled, 1.0f, 1e-6f);
    ZJoltHairGradient degenerate = {0.0f, 1.0f, 0.5f, 0.5f};
    CHECK(zjoltHairGradientSample(&degenerate, 0.5f, &sampled) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "an empty fraction range has no value anywhere");

    static const ZJoltHairVertex kStrandVertices[7] = {
        {{0.0f, 0.0f, 0.0f}, 0.0f}, {{0.1f, 0.0f, 0.0f}, 1.0f},
        {{0.2f, 0.0f, 0.0f}, 1.0f}, {{0.3f, 0.0f, 0.0f}, 1.0f},
        {{0.0f, 0.0f, 0.1f}, 0.0f}, {{0.1f, 0.0f, 0.1f}, 1.0f},
        {{0.2f, 0.0f, 0.1f}, 1.0f},
    };
    static const ZJoltHairStrand kStrands[2] = {{0, 4, 0}, {4, 7, 0}};
    static const ZJoltVec3 kScalpVertices[3] = {
        {-0.2f, 0.0f, -0.2f}, {0.4f, 0.0f, -0.2f}, {-0.2f, 0.0f, 0.4f}};
    static const uint32_t kScalpTriangles[3] = {0, 1, 2};
    static const ZJoltHairSkinWeight kScalpWeights[3] = {
        {0, 1.0f}, {0, 1.0f}, {0, 1.0f}};
    static const float kIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};

    ZJoltHairDesc hair_desc;
    memset(&hair_desc, 0, sizeof(hair_desc));
    hair_desc.vertices = kStrandVertices;
    hair_desc.strands = kStrands;
    hair_desc.materials = &material;
    hair_desc.vertex_count = 7;
    hair_desc.strand_count = 2;
    hair_desc.material_count = 1;
    hair_desc.scalp_vertices = kScalpVertices;
    hair_desc.scalp_triangles = kScalpTriangles;
    hair_desc.scalp_skin_weights = kScalpWeights;
    hair_desc.scalp_inverse_bind_pose = kIdentity;
    hair_desc.scalp_vertex_count = 3;
    hair_desc.scalp_triangle_count = 1;
    hair_desc.skin_weights_per_vertex = 1;
    hair_desc.joint_count = 1;
    hair_desc.simulation_bounds_padding.x = 0.5f;
    hair_desc.simulation_bounds_padding.y = 0.5f;
    hair_desc.simulation_bounds_padding.z = 0.5f;
    hair_desc.grid_size_x = 4;
    hair_desc.grid_size_y = 5;
    hair_desc.grid_size_z = 6;
    hair_desc.rotation.w = 1.0f;
    hair_desc.object_layer = LAYER_MOVING;

    /* A material carrying a gradient with no fraction range at all is refused
       before it can become a groom made of NaN. */
    ZJoltHairMaterial broken = material;
    broken.gravity_factor = degenerate;
    ZJoltHairDesc broken_desc = hair_desc;
    broken_desc.materials = &broken;
    ZJoltHair *no_hair = NULL;
    CHECK(zjoltHairCreate(compute, &broken_desc, &no_hair) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a gradient whose fractions are equal is refused");
    CHECK(no_hair == NULL, "and no hair came back");

    ZJoltHair *hair = NULL;
    CHECK_OK(zjoltHairCreate(compute, &hair_desc, &hair));

    ZJoltHairInfo info;
    memset(&info, 0, sizeof(info));
    CHECK_OK(zjoltHairGetInfo(hair, &info));
    CHECK(info.simulated_vertex_count == 7, "every strand is simulated");
    CHECK(info.simulated_strand_count == 2, "both of them");
    CHECK(info.render_vertex_count == 7, "and rendered");
    CHECK(info.max_vertices_per_strand == 4, "the longest strand has four");
    CHECK(info.padded_vertex_count == 8,
          "the device buffers are a rectangle, so the short strand is padded");
    CHECK(info.grid_size_x == 4 && info.grid_size_y == 5 && info.grid_size_z == 6,
          "the grid is the one that was asked for");
    CHECK(info.joint_count == 1, "one joint skins the scalp");
    CHECK(info.max_root_distance_to_scalp < 1.0e-3f,
          "every root was already on the scalp triangle");

    uint32_t strand_count = 0;
    ZJoltHairStrand sim_strands[2];
    CHECK_OK(zjoltHairGetSimulatedStrands(hair, NULL, 0, &strand_count));
    CHECK(strand_count == 2, "the count comes back without a buffer");
    CHECK_OK(zjoltHairGetSimulatedStrands(hair, sim_strands, 2, &strand_count));
    CHECK(sim_strands[0].start_vertex == 0 &&
              sim_strands[0].end_vertex == sim_strands[1].start_vertex &&
              sim_strands[1].end_vertex == info.simulated_vertex_count,
          "the strand ranges tile the simulated vertices exactly");

    CHECK_OK(zjoltHairSetPose(hair, kIdentity, kIdentity, 1));
    CHECK_OK(zjoltHairUpdate(hair, system, 1.0f / 60.0f));

    uint32_t scalp_count = 0;
    ZJoltVec3 skinned[3];
    CHECK_OK(zjoltHairReadBackScalpVertices(hair, skinned, 3, &scalp_count));
    CHECK(scalp_count == 3, "the scalp reads back a vertex per scalp vertex");
    for (uint32_t i = 0; i < 3; ++i) {
      CHECK_NEAR(skinned[i].y, kScalpVertices[i].y, 1e-4f);
    }

    /* Raise the one joint and the scalp goes with it, which is the step
       between a pose and hair that follows a head. */
    float raised[16];
    memcpy(raised, kIdentity, sizeof(raised));
    raised[13] = 0.5f;
    CHECK_OK(zjoltHairSetPose(hair, kIdentity, raised, 1));
    CHECK_OK(zjoltHairUpdate(hair, system, 1.0f / 60.0f));
    CHECK_OK(zjoltHairReadBackScalpVertices(hair, skinned, 3, &scalp_count));
    for (uint32_t i = 0; i < 3; ++i) {
      CHECK_NEAR(skinned[i].y, kScalpVertices[i].y + 0.5f, 1e-4f);
    }

    CHECK_OK(zjoltHairSetPose(hair, kIdentity, kIdentity, 1));
    for (int step = 0; step < 30; ++step) {
      CHECK_OK(zjoltHairUpdate(hair, system, 1.0f / 60.0f));
    }

    uint32_t state_count = 0;
    ZJoltHairVertexState state[7];
    ZJoltVec3 positions[7];
    CHECK_OK(zjoltHairReadBackVertexState(hair, state, 7, &state_count));
    CHECK(state_count == 7, "one state per simulated vertex");
    CHECK_OK(zjoltHairReadBackPositions(hair, positions, 7, &state_count));
    for (uint32_t i = 0; i < 7; ++i) {
      CHECK_NEAR(state[i].position.x, positions[i].x, 1e-5f);
      CHECK_NEAR(state[i].position.y, positions[i].y, 1e-5f);
      CHECK_NEAR(state[i].position.z, positions[i].z, 1e-5f);
      float length_sq = state[i].rotation.x * state[i].rotation.x +
                        state[i].rotation.y * state[i].rotation.y +
                        state[i].rotation.z * state[i].rotation.z +
                        state[i].rotation.w * state[i].rotation.w;
      CHECK_NEAR(length_sq, 1.0f, 1e-3f);
    }
    {
      const uint32_t tip = sim_strands[0].end_vertex - 1;
      float speed = fabsf(state[tip].velocity.x) + fabsf(state[tip].velocity.y) +
                    fabsf(state[tip].velocity.z);
      CHECK(speed > 1e-3f, "the free tip is moving");
      CHECK_NEAR(state[sim_strands[0].start_vertex].velocity.y, 0.0f, 1e-4f);
    }

    /* A short buffer is filled as far as it goes and says so. */
    uint32_t truncated_count = 0;
    CHECK(zjoltHairReadBackVertexState(hair, state, 3, &truncated_count) ==
              ZJOLT_RESULT_BUFFER_TOO_SMALL,
          "a short buffer is refused with the required count in hand");
    CHECK(truncated_count == 7, "and the count is the whole thing");

    /* Bake the groom, throw the hair away, and build it again from the blob. */
    size_t groom_size = 0;
    CHECK_OK(zjoltHairSaveGroom(hair, NULL, 0, &groom_size));
    CHECK(groom_size > 0, "a built groom has a size");
    unsigned char *groom = (unsigned char *)malloc(groom_size);
    CHECK(groom != NULL, "the groom buffer allocated");
    if (groom != NULL) {
      size_t written = 0;
      CHECK(zjoltHairSaveGroom(hair, groom, groom_size - 1, &written) ==
                ZJOLT_RESULT_BUFFER_TOO_SMALL,
            "a buffer one byte short is refused");
      CHECK_OK(zjoltHairSaveGroom(hair, groom, groom_size, &written));
      CHECK(written == groom_size, "the size query and the write agree");

      ZJoltHair *restored = NULL;
      ZJoltQuat identity_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
      CHECK_OK(zjoltHairCreateFromGroom(compute, groom, groom_size, NULL,
                                        &identity_rotation, LAYER_MOVING,
                                        &restored));

      ZJoltHairInfo restored_info;
      memset(&restored_info, 0, sizeof(restored_info));
      CHECK_OK(zjoltHairGetInfo(restored, &restored_info));
      CHECK(memcmp(&info, &restored_info, sizeof(info)) == 0,
            "a restored groom is the same groom, field for field");

      CHECK_OK(zjoltHairSetPose(restored, kIdentity, kIdentity, 1));
      CHECK_OK(zjoltHairUpdate(restored, system, 1.0f / 60.0f));
      zjoltHairDestroy(restored);

      groom[groom_size - 1] ^= 0xffu;
      restored = NULL;
      CHECK(zjoltHairCreateFromGroom(compute, groom, groom_size, NULL, NULL,
                                     LAYER_MOVING, &restored) ==
                ZJOLT_RESULT_BAD_FORMAT,
            "a damaged groom fails its checksum");
      CHECK(restored == NULL, "and nothing came back");
      free(groom);
    }

    zjoltHairDestroy(hair);
    zjoltComputeSystemDestroy(compute);
  }

  //-------------------------------------------------------------------------
  // Constraint frames and limit impulses
  //
  // The same scenarios the Zig suite runs, driven through the header. Two of
  // these shapes are exactly what the ABI cross-check cannot see: a
  // ZJoltMat44 out-parameter, whose sixteen floats it compares only by total
  // size, and the PAIR getters, which take two independently nullable
  // `float *` and would pass its check declared as almost anything.
  //-------------------------------------------------------------------------

  {
    /* Well clear of the floor: this block is about the joints, not contacts. */
    const ZJoltReal kY = (ZJoltReal)20.0;

    ZJoltBodyDesc arm_desc;
    zjoltBodyDescInit(&arm_desc);
    arm_desc.shape = box;
    arm_desc.motion_type = ZJOLT_MOTION_TYPE_DYNAMIC;
    arm_desc.object_layer = LAYER_MOVING;
    arm_desc.position.y = kY;

    ZJoltBodyId arm_id = ZJOLT_BODY_ID_INVALID;
    CHECK_OK(zjoltBodyCreateAndAdd(system, &arm_desc, ZJOLT_ACTIVATION_ACTIVATE,
                                   &arm_id));

    /* A frame nobody defaults to: turn about +Z, measure from +Y. */
    ZJoltHingeConstraintDesc hinge_desc;
    memset(&hinge_desc, 0, sizeof(hinge_desc));
    hinge_desc.space = ZJOLT_CONSTRAINT_SPACE_WORLD;
    hinge_desc.point1.y = kY;
    hinge_desc.point2.y = kY;
    hinge_desc.hinge_axis1.z = 1.0f;
    hinge_desc.normal_axis1.y = 1.0f;
    hinge_desc.hinge_axis2.z = 1.0f;
    hinge_desc.normal_axis2.y = 1.0f;
    hinge_desc.limits_min = -0.4f;
    hinge_desc.limits_max = 0.4f;

    ZJoltConstraint *hinge = NULL;
    CHECK_OK(zjoltConstraintCreateHinge(system, ZJOLT_BODY_ID_WORLD, arm_id,
                                        &hinge_desc, &hinge));
    CHECK_OK(zjoltConstraintAdd(system, hinge));

    /* The descriptor was in world space; these come back in centre-of-mass
       space, which for this body puts the same frame at the origin. */
    ZJoltVec3 axis = {0.0f, 0.0f, 0.0f};
    CHECK_OK(zjoltHingeConstraintGetLocalSpaceHingeAxis2(hinge, &axis));
    CHECK_NEAR(axis.z, 1.0f, 1e-4f);
    CHECK_NEAR(axis.x, 0.0f, 1e-4f);
    CHECK_OK(zjoltHingeConstraintGetLocalSpaceNormalAxis2(hinge, &axis));
    CHECK_NEAR(axis.y, 1.0f, 1e-4f);

    ZJoltVec3 point = {1.0f, 1.0f, 1.0f};
    CHECK_OK(zjoltHingeConstraintGetLocalSpacePoint2(hinge, &point));
    CHECK_NEAR(point.y, 0.0f, 1e-4f);

    /* Body 1 is the world body, whose centre of mass is the origin, so the
       same frame sits at the anchor there instead. */
    CHECK_OK(zjoltHingeConstraintGetLocalSpacePoint1(hinge, &point));
    CHECK_NEAR(point.y, (float)kY, 1e-3f);
    CHECK_OK(zjoltHingeConstraintGetLocalSpaceHingeAxis1(hinge, &axis));
    CHECK_NEAR(axis.z, 1.0f, 1e-4f);
    CHECK_OK(zjoltHingeConstraintGetLocalSpaceNormalAxis1(hinge, &axis));
    CHECK_NEAR(axis.y, 1.0f, 1e-4f);

    /* Sixteen floats, column-major: hinge axis, normal axis, their cross,
       then the attachment point. */
    ZJoltMat44 frame;
    memset(&frame, 0, sizeof(frame));
    CHECK_OK(zjoltConstraintGetConstraintToBody2Matrix(hinge, &frame));
    CHECK_NEAR(frame.m[2], 1.0f, 1e-4f);   /* column 0 is +Z */
    CHECK_NEAR(frame.m[5], 1.0f, 1e-4f);   /* column 1 is +Y */
    CHECK_NEAR(frame.m[8], -1.0f, 1e-4f);  /* column 2 is their cross */
    CHECK_NEAR(frame.m[13], 0.0f, 1e-4f);  /* column 3 is the point */
    CHECK_NEAR(frame.m[15], 1.0f, 1e-4f);

    CHECK_OK(zjoltConstraintGetConstraintToBody1Matrix(hinge, &frame));
    CHECK_NEAR(frame.m[13], (float)kY, 1e-3f);

    /* Declared in every build; honest about the one it cannot serve. */
    float draw_size = -1.0f;
    const ZJoltResult draw_result = zjoltConstraintGetDrawSize(hinge, &draw_size);
    if (draw_result == ZJOLT_RESULT_OK) {
      CHECK_OK(zjoltConstraintSetDrawSize(hinge, 2.5f));
      CHECK_OK(zjoltConstraintGetDrawSize(hinge, &draw_size));
      CHECK_NEAR(draw_size, 2.5f, 1e-4f);
    } else {
      CHECK(draw_result == ZJOLT_RESULT_UNSUPPORTED,
            "draw size without the debug renderer is UNSUPPORTED, not %s",
            zjoltResultName(draw_result));
    }

    /* Nothing has pushed the hinge into its limit yet. */
    float limit_lambda = -1.0f;
    CHECK_OK(zjoltHingeConstraintGetTotalLambdaRotationLimits(hinge,
                                                              &limit_lambda));
    CHECK_NEAR(limit_lambda, 0.0f, 1e-5f);

    /* Both halves of a pair getter, and each half on its own: a C host may
       ask for one number and not the other. */
    float lambda_x = -1.0f;
    float lambda_y = -1.0f;
    CHECK_OK(zjoltHingeConstraintGetTotalLambdaRotation(hinge, &lambda_x,
                                                        &lambda_y));
    CHECK_OK(zjoltHingeConstraintGetTotalLambdaRotation(hinge, NULL, &lambda_y));
    CHECK_OK(zjoltHingeConstraintGetTotalLambdaRotation(hinge, &lambda_x, NULL));
    CHECK_OK(zjoltHingeConstraintGetTotalLambdaRotation(hinge, NULL, NULL));

    /* Drive it about the hinge axis, where only the limit can stop it. */
    const ZJoltVec3 spin = {0.0f, 0.0f, 5.0f};
    for (int i = 0; i < 60; ++i) {
      zjoltBodySetAngularVelocity(system, arm_id, &spin);
      CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));
    }

    float angle = 0.0f;
    CHECK_OK(zjoltHingeConstraintGetCurrentAngle(hinge, &angle));
    CHECK_NEAR(angle, 0.4f, 0.1f);
    CHECK_OK(zjoltHingeConstraintGetTotalLambdaRotationLimits(hinge,
                                                              &limit_lambda));
    CHECK(fabsf(limit_lambda) > 0.0f,
          "the hinge limit applied an impulse (%f)", (double)limit_lambda);

    /* A hinge accessor on the wrong kind is refused, not reinterpreted. */
    ZJoltPointConstraintDesc point_desc;
    memset(&point_desc, 0, sizeof(point_desc));
    point_desc.space = ZJOLT_CONSTRAINT_SPACE_WORLD;
    point_desc.point1.y = kY;
    point_desc.point2.y = kY;
    ZJoltConstraint *ball_joint = NULL;
    CHECK_OK(zjoltConstraintCreatePoint(system, ZJOLT_BODY_ID_WORLD, arm_id,
                                        &point_desc, &ball_joint));
    CHECK(zjoltHingeConstraintGetLocalSpaceHingeAxis1(ball_joint, &axis) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a hinge accessor on a point constraint is refused");
    /* ...but the frame matrix is a TwoBodyConstraint accessor, so it works on
       every kind this ABI can create. */
    CHECK_OK(zjoltConstraintGetConstraintToBody1Matrix(ball_joint, &frame));
    zjoltConstraintRelease(ball_joint);

    CHECK_OK(zjoltConstraintRemove(system, hinge));
    zjoltConstraintRelease(hinge);

    /* --- A swing-twist, with a frame that is NOT the body frame, so body
       space and constraint space cannot be confused for each other. --- */

    ZJoltSwingTwistConstraintDesc st_desc;
    memset(&st_desc, 0, sizeof(st_desc));
    st_desc.space = ZJOLT_CONSTRAINT_SPACE_WORLD;
    st_desc.position1.y = kY;
    st_desc.position2.y = kY;
    st_desc.twist_axis1.z = 1.0f;
    st_desc.plane_axis1.x = 1.0f;
    st_desc.twist_axis2.z = 1.0f;
    st_desc.plane_axis2.x = 1.0f;
    st_desc.swing_type = ZJOLT_SWING_TYPE_CONE;
    st_desc.normal_half_cone_angle = 1.0f;
    st_desc.plane_half_cone_angle = 1.0f;
    st_desc.twist_min_angle = -0.3f;
    st_desc.twist_max_angle = 0.3f;

    ZJoltConstraint *joint = NULL;
    CHECK_OK(zjoltConstraintCreateSwingTwist(system, ZJOLT_BODY_ID_WORLD, arm_id,
                                             &st_desc, &joint));
    CHECK_OK(zjoltConstraintAdd(system, joint));

    ZJoltQuat to_body1 = {0.0f, 0.0f, 0.0f, 1.0f};
    ZJoltQuat to_body2 = {0.0f, 0.0f, 0.0f, 1.0f};
    CHECK_OK(zjoltSwingTwistConstraintGetConstraintToBody1(joint, &to_body1));
    CHECK_OK(zjoltSwingTwistConstraintGetConstraintToBody2(joint, &to_body2));
    CHECK(fabsf(to_body1.w) < 0.99f,
          "the constraint frame really is rotated (w = %f)", (double)to_body1.w);

    ZJoltVec3 st_point = {1.0f, 1.0f, 1.0f};
    CHECK_OK(zjoltSwingTwistConstraintGetLocalSpacePosition2(joint, &st_point));
    CHECK_NEAR(st_point.y, 0.0f, 1e-4f);
    CHECK_OK(zjoltSwingTwistConstraintGetLocalSpacePosition1(joint, &st_point));
    CHECK_NEAR(st_point.y, (float)kY, 1e-3f);

    /* Body space in, constraint space stored: the getter cannot echo what
       went in, which is the whole reason the two calls both exist. */
    const ZJoltVec3 body_space = {0.0f, 0.0f, 2.0f};
    CHECK_OK(zjoltSwingTwistConstraintSetTargetAngularVelocityBodySpace(
        joint, &body_space));
    ZJoltVec3 stored = {0.0f, 0.0f, 0.0f};
    CHECK_OK(zjoltSwingTwistConstraintGetTargetAngularVelocity(joint, &stored));
    CHECK_NEAR(stored.x, 2.0f, 1e-3f);
    CHECK_NEAR(stored.z, 0.0f, 1e-3f);

    CHECK_OK(zjoltSwingTwistConstraintSetTargetAngularVelocity(joint,
                                                               &body_space));
    CHECK_OK(zjoltSwingTwistConstraintGetTargetAngularVelocity(joint, &stored));
    CHECK_NEAR(stored.z, 2.0f, 1e-3f);
    CHECK_NEAR(stored.x, 0.0f, 1e-3f);

    /* A body-space target orientation is converted through the frame too. */
    const ZJoltQuat identity_target = {0.0f, 0.0f, 0.0f, 1.0f};
    CHECK_OK(zjoltSwingTwistConstraintSetTargetOrientationBodySpace(
        joint, &identity_target));
    /* The six-DOF twin of that call narrows on its own kind, so it refuses a
       swing-twist rather than casting to the wrong type and writing through
       it. */
    CHECK(zjoltSixDofConstraintSetTargetOrientationBodySpace(
              joint, &identity_target) == ZJOLT_RESULT_INVALID_ARGUMENT,
          "a six-DOF setter on a swing-twist is refused");

    /* Twist it past its limit; only the twist part should have to push. */
    const ZJoltVec3 twist_spin = {0.0f, 0.0f, 6.0f};
    for (int i = 0; i < 60; ++i) {
      zjoltBodySetAngularVelocity(system, arm_id, &twist_spin);
      CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));
    }

    float twist_lambda = 0.0f;
    float swing_y_lambda = -1.0f;
    float swing_z_lambda = -1.0f;
    CHECK_OK(zjoltSwingTwistConstraintGetTotalLambdaTwist(joint, &twist_lambda));
    CHECK_OK(zjoltSwingTwistConstraintGetTotalLambdaSwingY(joint,
                                                           &swing_y_lambda));
    CHECK_OK(zjoltSwingTwistConstraintGetTotalLambdaSwingZ(joint,
                                                           &swing_z_lambda));
    CHECK(fabsf(twist_lambda) > 0.0f, "the twist limit applied an impulse");
    CHECK_NEAR(swing_y_lambda, 0.0f, 1e-5f);
    CHECK_NEAR(swing_z_lambda, 0.0f, 1e-5f);

    CHECK_OK(zjoltConstraintRemove(system, joint));
    zjoltConstraintRelease(joint);

    /* --- A slider, and the difference between its axis impulse and its limit
       impulse. --- */

    ZJoltSliderConstraintDesc slider_desc;
    memset(&slider_desc, 0, sizeof(slider_desc));
    slider_desc.space = ZJOLT_CONSTRAINT_SPACE_WORLD;
    slider_desc.point1.y = kY;
    slider_desc.point2.y = kY;
    slider_desc.slider_axis1.x = 1.0f;
    slider_desc.normal_axis1.y = 1.0f;
    slider_desc.slider_axis2.x = 1.0f;
    slider_desc.normal_axis2.y = 1.0f;
    slider_desc.limits_min = -0.5f;
    slider_desc.limits_max = 0.5f;

    ZJoltConstraint *slider = NULL;
    CHECK_OK(zjoltConstraintCreateSlider(system, ZJOLT_BODY_ID_WORLD, arm_id,
                                         &slider_desc, &slider));
    CHECK_OK(zjoltConstraintAdd(system, slider));

    const ZJoltVec3 stop = {0.0f, 0.0f, 0.0f};
    zjoltBodySetAngularVelocity(system, arm_id, &stop);
    zjoltBodySetLinearVelocity(system, arm_id, &stop);
    for (int i = 0; i < 18; ++i) {
      CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));
    }

    /* Gravity pulls across the slider axis, so the AXIS pair is carrying the
       weight while the LIMIT part has applied nothing. */
    float held_x = 0.0f;
    float held_y = 0.0f;
    CHECK_OK(zjoltSliderConstraintGetTotalLambdaPosition(slider, &held_x,
                                                         &held_y));
    CHECK(fabsf(held_x) + fabsf(held_y) > 0.0f,
          "the slider axis constraint is carrying the weight");

    float slider_limit_lambda = -1.0f;
    CHECK_OK(zjoltSliderConstraintGetTotalLambdaPositionLimits(
        slider, &slider_limit_lambda));
    CHECK_NEAR(slider_limit_lambda, 0.0f, 1e-5f);

    ZJoltVec3 slider_rotation_lambda = {-1.0f, -1.0f, -1.0f};
    CHECK_OK(zjoltSliderConstraintGetTotalLambdaRotation(
        slider, &slider_rotation_lambda));

    const ZJoltVec3 push = {4.0f, 0.0f, 0.0f};
    for (int i = 0; i < 60; ++i) {
      zjoltBodySetLinearVelocity(system, arm_id, &push);
      CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));
    }

    float travel = 0.0f;
    CHECK_OK(zjoltSliderConstraintGetCurrentPosition(slider, &travel));
    CHECK_NEAR(travel, 0.5f, 0.05f);
    CHECK_OK(zjoltSliderConstraintGetTotalLambdaPositionLimits(
        slider, &slider_limit_lambda));
    CHECK(fabsf(slider_limit_lambda) > 0.0f,
          "the slider limit stopped it (%f)", (double)slider_limit_lambda);

    CHECK_OK(zjoltConstraintRemove(system, slider));
    zjoltConstraintRelease(slider);

    zjoltBodyDestroy(system, arm_id);
  }

  //-------------------------------------------------------------------------
  // Ragdolls and skeleton mapping
  //
  // A two-joint ragdoll, and a mapper pushing its pose onto a three-joint
  // render skeleton. The pose types are where the [*c] residue would show:
  // every one of these takes an array of a flat struct, and a Zig-side
  // declaration that got a pointee wrong would still pass the ABI guard.
  //-------------------------------------------------------------------------

  {
    ZJoltSkeleton *skeleton = NULL;
    CHECK_OK(zjoltSkeletonCreate(&skeleton));

    uint32_t root_joint = 0xffffffffu;
    uint32_t hand_joint = 0xffffffffu;
    CHECK_OK(zjoltSkeletonAddJoint(skeleton, "root", -1, &root_joint));
    CHECK_OK(zjoltSkeletonAddJoint(skeleton, "hand", (int32_t)root_joint,
                                   &hand_joint));
    CHECK(zjoltSkeletonGetJointCount(skeleton) == 2, "two joints");
    CHECK(zjoltSkeletonGetJointIndex(skeleton, "hand") == (int32_t)hand_joint,
          "a joint is found by name");

    ZJoltRagdollConstraintDesc to_parent;
    memset(&to_parent, 0, sizeof(to_parent));
    to_parent.position1.y = (ZJoltReal)6.5;
    to_parent.twist_axis1.y = 1.0f;
    to_parent.plane_axis1.x = 1.0f;
    to_parent.position2.y = (ZJoltReal)6.5;
    to_parent.twist_axis2.y = 1.0f;
    to_parent.plane_axis2.x = 1.0f;
    to_parent.swing_type = ZJOLT_SWING_TYPE_CONE;
    to_parent.normal_half_cone_angle = 1.0f;
    to_parent.plane_half_cone_angle = 1.0f;
    to_parent.twist_min_angle = -1.0f;
    to_parent.twist_max_angle = 1.0f;

    ZJoltRagdollPartDesc parts[2];
    zjoltBodyDescInit(&parts[0].body);
    parts[0].body.shape = box;
    parts[0].body.object_layer = LAYER_MOVING;
    parts[0].body.position.y = (ZJoltReal)6.0;
    parts[0].to_parent = NULL;
    zjoltBodyDescInit(&parts[1].body);
    parts[1].body.shape = box;
    parts[1].body.object_layer = LAYER_MOVING;
    parts[1].body.position.y = (ZJoltReal)7.0;
    parts[1].to_parent = &to_parent;

    ZJoltRagdollSettings *settings = NULL;
    CHECK_OK(zjoltRagdollSettingsCreate(&settings));
    CHECK_OK(zjoltRagdollSettingsBuild(settings, skeleton, parts, 2));
    CHECK(zjoltRagdollSettingsGetSkeleton(settings) == skeleton,
          "the settings hand back the skeleton they were built with");
    zjoltRagdollSettingsDisableParentChildCollisions(settings);
    CHECK_OK(zjoltRagdollSettingsCalculateConstraintPriorities(settings, 5));

    ZJoltRagdoll *ragdoll = NULL;
    CHECK_OK(zjoltRagdollSettingsCreateRagdoll(settings, system, 3, 0,
                                               &ragdoll));
    zjoltRagdollAddToPhysicsSystem(ragdoll, ZJOLT_ACTIVATION_ACTIVATE, true);

    CHECK(zjoltRagdollGetRagdollSettings(ragdoll) == settings,
          "and the ragdoll hands back the settings that spawned it");
    CHECK(zjoltRagdollGetConstraintCount(ragdoll) == 1,
          "one constraint: the root joint has no parent to be attached to");

    /* The priorities reached the constraints the spawn built, which is the
       only place they are observable from. */
    ZJoltConstraint *joint = zjoltRagdollGetConstraint(ragdoll, 0);
    CHECK(joint != NULL, "the ragdoll's constraint is reachable");
    CHECK(zjoltConstraintGetPriority(joint) == 5, "and carries the priority");
    CHECK(zjoltRagdollGetConstraint(ragdoll, 1) == NULL,
          "an index past the end is refused rather than indexed");

    ZJoltBodyId part_ids[2] = {ZJOLT_BODY_ID_INVALID, ZJOLT_BODY_ID_INVALID};
    uint32_t part_count = 0;
    CHECK_OK(zjoltRagdollGetBodyIds(ragdoll, part_ids, 2, &part_count));
    CHECK(part_count == 2, "two parts");

    ZJoltRVec3 root_position = {0, 0, 0};
    ZJoltQuat root_rotation = {0, 0, 0, 0};
    CHECK_OK(zjoltRagdollGetRootTransform(ragdoll, &root_position,
                                          &root_rotation, true));
    ZJoltRVec3 body_position = {0, 0, 0};
    zjoltBodyGetPositionAndRotation(system, part_ids[0], &body_position, NULL);
    CHECK_NEAR(root_position.y, body_position.y, 1e-4f);
    CHECK_NEAR(root_rotation.w, 1.0f, 1e-4f);

    ZJoltCollisionGroup group;
    zjoltBodyGetCollisionGroup(system, part_ids[1], &group);
    CHECK(group.group_id == 3, "the spawn set the group id");
    CHECK(group.sub_group_id == 1, "and the joint index is the sub-group");
    zjoltRagdollSetGroupId(ragdoll, 9, true);
    zjoltBodyGetCollisionGroup(system, part_ids[1], &group);
    CHECK(group.group_id == 9, "and it moves");
    CHECK(group.sub_group_id == 1, "without disturbing the sub-group");
    CHECK(group.filter != NULL, "or the shared filter");

    /* The render skeleton: the ragdoll's, plus a leaf no part drives. */
    ZJoltSkeleton *render = NULL;
    CHECK_OK(zjoltSkeletonCreate(&render));
    uint32_t r_root = 0;
    uint32_t r_hand = 0;
    uint32_t r_finger = 0;
    CHECK_OK(zjoltSkeletonAddJoint(render, "root", -1, &r_root));
    CHECK_OK(zjoltSkeletonAddJoint(render, "hand", (int32_t)r_root, &r_hand));
    CHECK_OK(
        zjoltSkeletonAddJoint(render, "finger", (int32_t)r_hand, &r_finger));

    const ZJoltQuat identity = {0, 0, 0, 1};
    const ZJoltQuat identities[3] = {{0, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 0, 1}};
    const ZJoltVec3 neutral1_joints[2] = {{0, 0, 0}, {0, 1, 0}};
    const ZJoltVec3 neutral2_joints[3] = {{0, 0, 0}, {0, 1, 0}, {0, 0.5f, 0}};

    ZJoltSkeletonPose *neutral1 = NULL;
    CHECK_OK(zjoltSkeletonPoseCreate(&neutral1));
    CHECK_OK(zjoltSkeletonPoseSetSkeleton(neutral1, skeleton));
    CHECK_OK(zjoltSkeletonPoseSetJoints(neutral1, identities, neutral1_joints,
                                        2));
    CHECK_OK(zjoltSkeletonPoseCalculateJointMatrices(neutral1));

    ZJoltSkeletonPose *neutral2 = NULL;
    CHECK_OK(zjoltSkeletonPoseCreate(&neutral2));
    CHECK_OK(zjoltSkeletonPoseSetSkeleton(neutral2, render));
    CHECK_OK(zjoltSkeletonPoseSetJoints(neutral2, identities, neutral2_joints,
                                        3));
    CHECK_OK(zjoltSkeletonPoseCalculateJointMatrices(neutral2));

    ZJoltSkeletonMapper *mapper = NULL;
    CHECK_OK(zjoltSkeletonMapperCreate(&mapper));
    CHECK(zjoltSkeletonMapperGetRefCount(mapper) == 1, "one reference");
    CHECK_OK(zjoltSkeletonMapperInitialize(mapper, neutral1, neutral2, NULL,
                                           NULL));
    CHECK(zjoltSkeletonMapperGetMappingCount(mapper) == 2,
          "both ragdoll joints found their namesake");
    CHECK(zjoltSkeletonMapperGetMappedJointIndex(mapper, 1) ==
              (int32_t)r_hand,
          "and the hand maps to the render skeleton's hand");
    CHECK(zjoltSkeletonMapperGetMappedJointIndex(mapper, 7) == -1,
          "a joint that does not exist maps to nothing");

    /* A pose taken off the live ragdoll, mapped onto the render skeleton. */
    ZJoltSkeletonPose *simulated = NULL;
    CHECK_OK(zjoltSkeletonPoseCreate(&simulated));
    CHECK_OK(zjoltSkeletonPoseSetSkeleton(simulated, skeleton));
    CHECK_OK(zjoltRagdollGetPose(ragdoll, simulated, true));

    ZJoltSkeletonPose *drawn = NULL;
    CHECK_OK(zjoltSkeletonPoseCreate(&drawn));
    CHECK_OK(zjoltSkeletonPoseSetSkeleton(drawn, render));
    CHECK_OK(zjoltSkeletonPoseSetJoints(drawn, identities, neutral2_joints, 3));
    CHECK_OK(zjoltSkeletonMapperMap(mapper, simulated, drawn));
    CHECK_OK(zjoltSkeletonPoseCalculateJointStates(drawn));

    ZJoltQuat drawn_rotations[3];
    ZJoltVec3 drawn_translations[3];
    uint32_t drawn_count = 0;
    CHECK_OK(zjoltSkeletonPoseGetJoints(drawn, drawn_rotations,
                                        drawn_translations, 3, &drawn_count));
    CHECK(drawn_count == 3, "three joints came back");
    /* The ragdoll's world position rode across in the ROOT OFFSET, not in the
       root joint: zjoltRagdollGetPose writes joint matrices relative to it. A
       render pose left at its own offset would draw six metres low. */
    ZJoltRVec3 drawn_offset = {0, 0, 0};
    zjoltSkeletonPoseGetRootOffset(drawn, &drawn_offset);
    CHECK_NEAR(drawn_offset.y, 6.0f, 0.5f);
    CHECK_NEAR(drawn_translations[0].y, 0.0f, 1e-3f);
    /* The finger is not driven by the ragdoll, so it kept its own offset from
       the hand that just moved under it. */
    CHECK_NEAR(drawn_translations[2].y, 0.5f, 1e-3f);
    CHECK_NEAR(drawn_translations[2].x, 0.0f, 1e-3f);

    /* Locking pins a mapped joint's translation to the neutral pose. */
    const bool locked[3] = {false, true, false};
    CHECK_OK(zjoltSkeletonMapperLockTranslations(mapper, neutral2, locked, 3));
    CHECK(zjoltSkeletonMapperIsJointTranslationLocked(mapper, 1),
          "the hand is locked");
    CHECK(!zjoltSkeletonMapperIsJointTranslationLocked(mapper, 2),
          "the finger is not");

    const bool lock_root[3] = {true, false, false};
    CHECK(zjoltSkeletonMapperLockTranslations(mapper, neutral2, lock_root, 3) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a joint with no parent cannot be locked");

    CHECK_OK(zjoltSkeletonMapperMap(mapper, simulated, drawn));
    CHECK_OK(zjoltSkeletonPoseCalculateJointStates(drawn));
    CHECK_OK(zjoltSkeletonPoseGetJoints(drawn, drawn_rotations,
                                        drawn_translations, 3, &drawn_count));
    CHECK_NEAR(drawn_translations[1].y, 1.0f, 1e-3f);

    /* And back the other way, onto a fresh pose of the ragdoll skeleton. */
    ZJoltSkeletonPose *back = NULL;
    CHECK_OK(zjoltSkeletonPoseCreate(&back));
    CHECK_OK(zjoltSkeletonPoseSetSkeleton(back, skeleton));
    CHECK_OK(zjoltSkeletonMapperMapReverse(mapper, drawn, back));
    CHECK_OK(zjoltSkeletonPoseCalculateJointStates(back));

    ZJoltQuat back_rotations[2];
    ZJoltVec3 back_translations[2];
    uint32_t back_count = 0;
    CHECK_OK(zjoltSkeletonPoseGetJoints(back, back_rotations, back_translations,
                                        2, &back_count));
    CHECK(back_count == 2, "two joints came back");
    CHECK_NEAR(back_translations[0].y, drawn_translations[0].y, 1e-3f);
    CHECK_NEAR(back_rotations[0].w, identity.w, 1e-3f);

    zjoltSkeletonPoseDestroy(back);
    zjoltSkeletonPoseDestroy(drawn);
    zjoltSkeletonPoseDestroy(simulated);
    zjoltSkeletonMapperRelease(mapper);
    zjoltSkeletonPoseDestroy(neutral2);
    zjoltSkeletonPoseDestroy(neutral1);
    zjoltSkeletonRelease(render);

    /* Releasing a still-added ragdoll takes it out of the world first. */
    zjoltRagdollRelease(ragdoll);
    zjoltRagdollSettingsRelease(settings);
    zjoltSkeletonRelease(skeleton);
  }

  //-------------------------------------------------------------------------
  // A soft body's second half: rods, materials, the manifold, and the update
  // that does not go through the system
  //
  // Every one of these carries a pointer or a struct the Zig ABI guard only
  // measures. The contact listener in particular is a slot of its own — a
  // soft body's collisions never reach ZJoltContactListener — and the sensor
  // list below does not merely go unheard without one installed, it is not
  // gathered at all.
  //-------------------------------------------------------------------------

  {
    ZJoltSoftBodySharedSettings *rope = NULL;
    CHECK_OK(zjoltSoftBodySharedSettingsCreate(&rope));

    /* Four vertices along +X, the first pinned, joined by three rods. */
    ZJoltSoftBodyVertex rope_vertices[4];
    for (int i = 0; i < 4; ++i) {
      rope_vertices[i].position.x = (float)i;
      rope_vertices[i].position.y = 0.0f;
      rope_vertices[i].position.z = 0.0f;
      rope_vertices[i].velocity.x = 0.0f;
      rope_vertices[i].velocity.y = 0.0f;
      rope_vertices[i].velocity.z = 0.0f;
      rope_vertices[i].inv_mass = (i == 0) ? 0.0f : 1.0f;
    }
    CHECK_OK(zjoltSoftBodySharedSettingsAddVertices(rope, rope_vertices, 4));

    ZJoltSoftBodyRodStretchShear rods[3];
    for (int i = 0; i < 3; ++i) {
      rods[i].vertex[0] = (uint32_t)i;
      rods[i].vertex[1] = (uint32_t)(i + 1);
      rods[i].compliance = 0.0f;
    }
    CHECK_OK(
        zjoltSoftBodySharedSettingsAddRodStretchShearConstraints(rope, rods, 3));

    /* A rod naming a vertex nobody added reaches Jolt's array unguarded. */
    ZJoltSoftBodyRodStretchShear bad_rod = rods[0];
    bad_rod.vertex[1] = 9;
    CHECK(zjoltSoftBodySharedSettingsAddRodStretchShearConstraints(
              rope, &bad_rod, 1) == ZJOLT_RESULT_INVALID_ARGUMENT,
          "a rod naming a vertex that does not exist is refused");

    ZJoltSoftBodyRodBendTwist twists[2];
    for (int i = 0; i < 2; ++i) {
      twists[i].rod[0] = (uint32_t)i;
      twists[i].rod[1] = (uint32_t)(i + 1);
      twists[i].compliance = 0.01f;
    }
    CHECK_OK(zjoltSoftBodySharedSettingsAddRodBendTwistConstraints(rope, twists,
                                                                   2));

    ZJoltSoftBodyRodBendTwist bad_twist = twists[0];
    bad_twist.rod[1] = 7;
    CHECK(zjoltSoftBodySharedSettingsAddRodBendTwistConstraints(
              rope, &bad_twist, 1) == ZJOLT_RESULT_INVALID_ARGUMENT,
          "a bend twist constraint naming a rod that does not exist is refused");

    CHECK_OK(zjoltSoftBodySharedSettingsCalculateRodProperties(rope));
    zjoltSoftBodySharedSettingsOptimize(rope);

    ZJoltSoftBodyDesc rope_desc;
    zjoltSoftBodyDescInit(&rope_desc);
    rope_desc.shared_settings = rope;
    rope_desc.object_layer = LAYER_MOVING;
    rope_desc.position.y = (ZJoltReal)6.0;
    rope_desc.update_position = false;
    rope_desc.allow_sleeping = false;

    ZJoltBodyId rope_id = ZJOLT_BODY_ID_INVALID;
    CHECK_OK(zjoltSoftBodyCreateAndAdd(system, &rope_desc,
                                       ZJOLT_ACTIVATION_ACTIVATE, &rope_id));

    uint32_t rod_count = 0;
    CHECK_OK(zjoltSoftBodyGetRodStates(system, rope_id, NULL, 0, &rod_count));
    CHECK(rod_count == 3, "the rope has its three rods");

    for (int i = 0; i < 30; ++i)
      CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));

    ZJoltSoftBodyRodState rod_states[3];
    CHECK_OK(
        zjoltSoftBodyGetRodStates(system, rope_id, rod_states, 3, &rod_count));
    CHECK(rod_count == 3, "and reads all three back");

    ZJoltSoftBodyVertexState rope_states[4];
    uint32_t rope_vertex_count = 0;
    CHECK_OK(zjoltSoftBodyGetVertexStates(system, rope_id, rope_states, 4,
                                          &rope_vertex_count));
    CHECK(rope_vertex_count == 4, "and its four vertices");

    /* A rod's rotation takes its local +Z onto the direction between its two
       vertices. That is the contract, and the whole reason a rod is worth
       more than an edge. */
    for (uint32_t i = 0; i < rod_count; ++i) {
      ZJoltVec3 a = rope_states[rod_states[i].vertex[0]].position;
      ZJoltVec3 b = rope_states[rod_states[i].vertex[1]].position;
      ZJoltVec3 delta = {b.x - a.x, b.y - a.y, b.z - a.z};
      float length =
          sqrtf(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
      CHECK(length > 0.5f, "rod %u kept its length", i);

      ZJoltVec3 z_axis = {0.0f, 0.0f, 1.0f};
      ZJoltVec3 tangent;
      CHECK_OK(zjoltQuatRotateVector(&rod_states[i].rotation, &z_axis,
                                     &tangent));
      CHECK_NEAR(tangent.x, delta.x / length, 0.05f);
      CHECK_NEAR(tangent.y, delta.y / length, 0.05f);
      CHECK_NEAR(tangent.z, delta.z / length, 0.05f);
    }

    zjoltBodyDestroy(system, rope_id);
    zjoltSoftBodySharedSettingsRelease(rope);
  }

  {
    /* A tetrahedron whose six-times-volume is 2, so Jolt's placeholder rest
       volume of 1 is visibly wrong until it is measured. */
    static const float kTet[4][3] = {
        {0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
    };
    static const uint32_t kTetFaces[4][3] = {
        {0, 2, 1}, {0, 3, 2}, {0, 1, 3}, {1, 2, 3},
    };

    ZJoltSoftBodyVertex tet_vertices[4];
    ZJoltSoftBodyFace tet_faces[4];
    for (int i = 0; i < 4; ++i) {
      tet_vertices[i].position.x = kTet[i][0];
      tet_vertices[i].position.y = kTet[i][1];
      tet_vertices[i].position.z = kTet[i][2];
      tet_vertices[i].velocity.x = 0.0f;
      tet_vertices[i].velocity.y = 0.0f;
      tet_vertices[i].velocity.z = 0.0f;
      tet_vertices[i].inv_mass = 1.0f;
      tet_faces[i].vertex[0] = kTetFaces[i][0];
      tet_faces[i].vertex[1] = kTetFaces[i][1];
      tet_faces[i].vertex[2] = kTetFaces[i][2];
      tet_faces[i].material_index = 0;
    }

    ZJoltSoftBodyVolumeConstraint tet_volume;
    tet_volume.vertex[0] = 0;
    tet_volume.vertex[1] = 1;
    tet_volume.vertex[2] = 2;
    tet_volume.vertex[3] = 3;
    tet_volume.compliance = 1.0e-5f;

    ZJoltSoftBodySharedSettings *jelly = NULL;
    CHECK_OK(zjoltSoftBodySharedSettingsCreate(&jelly));
    CHECK_OK(zjoltSoftBodySharedSettingsAddVertices(jelly, tet_vertices, 4));
    CHECK_OK(zjoltSoftBodySharedSettingsAddFaces(jelly, tet_faces, 4));
    CHECK_OK(
        zjoltSoftBodySharedSettingsAddVolumeConstraints(jelly, &tet_volume, 1));

    /* The clone is taken BEFORE the rest volumes are measured, so the two
       differ by exactly the one call this is here to show. */
    ZJoltSoftBodySharedSettings *unmeasured = NULL;
    CHECK_OK(zjoltSoftBodySharedSettingsClone(jelly, &unmeasured));
    CHECK(zjoltSoftBodySharedSettingsGetRefCount(unmeasured) == 1,
          "a clone comes back with one reference of its own");
    CHECK(zjoltSoftBodySharedSettingsGetRefCount(jelly) == 1,
          "and does not take one from the original");
    zjoltSoftBodySharedSettingsOptimize(unmeasured);

    zjoltSoftBodySharedSettingsCalculateVolumeConstraintVolumes(jelly);
    zjoltSoftBodySharedSettingsOptimize(jelly);

    ZJoltSoftBodyDesc jelly_desc;
    zjoltSoftBodyDescInit(&jelly_desc);
    jelly_desc.shared_settings = jelly;
    jelly_desc.object_layer = LAYER_MOVING;
    jelly_desc.position.y = (ZJoltReal)20.0;
    jelly_desc.gravity_factor = 0.0f;
    jelly_desc.allow_sleeping = false;

    ZJoltBodyId measured_id = ZJOLT_BODY_ID_INVALID;
    CHECK_OK(zjoltSoftBodyCreateAndAdd(system, &jelly_desc,
                                       ZJOLT_ACTIVATION_ACTIVATE,
                                       &measured_id));

    jelly_desc.shared_settings = unmeasured;
    jelly_desc.position.x = (ZJoltReal)30.0;
    ZJoltBodyId unmeasured_id = ZJOLT_BODY_ID_INVALID;
    CHECK_OK(zjoltSoftBodyCreateAndAdd(system, &jelly_desc,
                                       ZJOLT_ACTIVATION_ACTIVATE,
                                       &unmeasured_id));

    for (int i = 0; i < 60; ++i)
      CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));

    float measured_volume = 0.0f;
    float unmeasured_volume = 0.0f;
    CHECK_OK(zjoltSoftBodyGetVolume(system, measured_id, &measured_volume));
    CHECK_OK(zjoltSoftBodyGetVolume(system, unmeasured_id, &unmeasured_volume));
    CHECK_NEAR(measured_volume, 2.0f / 6.0f, 0.05f);
    CHECK_NEAR(unmeasured_volume, 1.0f / 6.0f, 0.02f);

    zjoltBodyDestroy(system, measured_id);
    zjoltBodyDestroy(system, unmeasured_id);
    zjoltSoftBodySharedSettingsRelease(unmeasured);
    zjoltSoftBodySharedSettingsRelease(jelly);
  }

  {
    /* Materials, the face index a hit resolves to, the contact listener, and
       an update that never goes through the system. */
    static const float kCorners[8][3] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, 0.5f},
        {-0.5f, -0.5f, 0.5f},  {-0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, -0.5f},
        {0.5f, 0.5f, 0.5f},    {-0.5f, 0.5f, 0.5f},
    };
    static const uint32_t kTriangles[12][3] = {
        {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {3, 2, 6}, {3, 6, 7},
        {0, 4, 5}, {0, 5, 1}, {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2},
    };

    ZJoltSoftBodyVertex cube_vertices[8];
    ZJoltSoftBodyFace cube_faces[12];
    for (int i = 0; i < 8; ++i) {
      cube_vertices[i].position.x = kCorners[i][0];
      cube_vertices[i].position.y = kCorners[i][1];
      cube_vertices[i].position.z = kCorners[i][2];
      cube_vertices[i].velocity.x = 0.0f;
      cube_vertices[i].velocity.y = 0.0f;
      cube_vertices[i].velocity.z = 0.0f;
      cube_vertices[i].inv_mass = 1.0f;
    }
    for (int i = 0; i < 12; ++i) {
      cube_faces[i].vertex[0] = kTriangles[i][0];
      cube_faces[i].vertex[1] = kTriangles[i][1];
      cube_faces[i].vertex[2] = kTriangles[i][2];
      /* Faces 2 and 3 are the top of the cube. */
      cube_faces[i].material_index = (i == 2 || i == 3) ? 1u : 0u;
    }

    ZJoltPhysicsMaterial *canvas = NULL;
    ZJoltPhysicsMaterial *lid = NULL;
    CHECK_OK(zjoltPhysicsMaterialCreate("canvas", NULL, &canvas));
    CHECK_OK(zjoltPhysicsMaterialCreate("lid", NULL, &lid));
    const ZJoltPhysicsMaterial *const material_list[2] = {canvas, lid};

    ZJoltSoftBodySharedSettings *skin = NULL;
    CHECK_OK(zjoltSoftBodySharedSettingsCreate(&skin));
    CHECK_OK(zjoltSoftBodySharedSettingsAddVertices(skin, cube_vertices, 8));
    CHECK_OK(zjoltSoftBodySharedSettingsSetMaterials(skin, material_list, 2));
    CHECK_OK(zjoltSoftBodySharedSettingsAddFaces(skin, cube_faces, 12));

    /* An empty list is an out-of-bounds read for every face, not a clear. */
    CHECK(zjoltSoftBodySharedSettingsSetMaterials(skin, material_list, 0) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "an empty material list is refused");

    uint32_t material_count = 0;
    CHECK_OK(zjoltSoftBodySharedSettingsGetMaterials(skin, NULL, 0,
                                                     &material_count));
    CHECK(material_count == 2, "both materials are on the settings");
    const ZJoltPhysicsMaterial *read_back[2] = {NULL, NULL};
    CHECK_OK(zjoltSoftBodySharedSettingsGetMaterials(skin, read_back, 2,
                                                     &material_count));
    CHECK(read_back[0] == canvas && read_back[1] == lid,
          "and come back in the order they went in");

    ZJoltSoftBodyVertexAttributes attributes;
    zjoltSoftBodyVertexAttributesInit(&attributes);
    CHECK_OK(zjoltSoftBodySharedSettingsCreateConstraints(
        skin, &attributes, 1, ZJOLT_SOFT_BODY_BEND_TYPE_DIHEDRAL, 0.1396f));
    zjoltSoftBodySharedSettingsOptimize(skin);

    ZJoltSoftBodyDesc skin_desc;
    zjoltSoftBodyDescInit(&skin_desc);
    skin_desc.shared_settings = skin;
    skin_desc.object_layer = LAYER_MOVING;
    skin_desc.position.x = (ZJoltReal)50.0;
    skin_desc.position.y = (ZJoltReal)10.0;
    skin_desc.pressure = 20.0f;
    skin_desc.vertex_radius = 0.01f;
    skin_desc.allow_sleeping = false;

    ZJoltBodyId skin_id = ZJOLT_BODY_ID_INVALID;
    CHECK_OK(zjoltSoftBodyCreateAndAdd(system, &skin_desc,
                                       ZJOLT_ACTIVATION_ACTIVATE, &skin_id));

    /* Straight down the middle from above, so the first thing in the way is
       one of the two faces carrying the second material. */
    ZJoltRVec3 ray_origin = {(ZJoltReal)50.0, (ZJoltReal)15.0, (ZJoltReal)0.0};
    ZJoltVec3 ray_direction = {0.0f, -10.0f, 0.0f};
    ZJoltRayCastHit hit;
    bool did_hit = false;
    CHECK_OK(zjoltCastRayClosest(system, &ray_origin, &ray_direction, NULL,
                                 NULL, &hit, &did_hit));
    CHECK(did_hit, "the ray found the soft body");
    if (did_hit) {
      CHECK(hit.body == skin_id, "and it is the soft body it found");

      uint32_t face_index = 0xffffffffu;
      CHECK_OK(zjoltSoftBodyGetFaceIndex(system, skin_id, hit.sub_shape_id,
                                         &face_index));
      CHECK(face_index == 2 || face_index == 3,
            "the sub shape id resolves to one of the two top faces (got %u)",
            face_index);
      CHECK(hit.material == lid,
            "and the material on it is the one that face was given");
    }

    /* A sub shape id from nothing in this body is refused rather than
       decoded into a plausible-looking face index. */
    uint32_t nowhere_face = 0;
    CHECK(zjoltSoftBodyGetFaceIndex(system, skin_id, ZJOLT_SUB_SHAPE_ID_EMPTY,
                                    &nowhere_face) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "an empty sub shape id names no face");
    CHECK(zjoltSoftBodyGetFaceIndex(system, ball_id, 0, &nowhere_face) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "and a rigid body has no faces to name");

    /* An update that never goes through the system at all: no step, no job
       system, and the body still falls. */
    ZJoltRVec3 before_update;
    ZJoltRVec3 after_update;
    zjoltBodyGetCenterOfMassPosition(system, skin_id, &before_update);
    for (int i = 0; i < 30; ++i)
      CHECK_OK(zjoltSoftBodyCustomUpdate(system, skin_id, 1.0f / 60.0f));
    zjoltBodyGetCenterOfMassPosition(system, skin_id, &after_update);
    CHECK((double)after_update.y < (double)before_update.y - 0.5,
          "a manual update moved the body without a step (%.3f -> %.3f)",
          (double)before_update.y, (double)after_update.y);
    CHECK((double)after_update.y > (double)before_update.y - 2.0,
          "by about half a second of free fall, and no more");

    CHECK(zjoltSoftBodyCustomUpdate(system, skin_id, 0.0f) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "a zero delta time is refused");
    CHECK(zjoltSoftBodyCustomUpdate(system, ball_id, 1.0f / 60.0f) ==
              ZJOLT_RESULT_INVALID_ARGUMENT,
          "and a rigid body has no soft body update to run");

    /* The listener, and a floor and a sensor for it to have something to say
       about. */
    ZJoltShape *slab = NULL;
    CHECK_OK(zjoltShapeCreateBox(&(ZJoltVec3){4.0f, 0.5f, 4.0f}, 0.05f, 0.0f,
                                 NULL, &slab));

    ZJoltBodyDesc slab_desc;
    zjoltBodyDescInit(&slab_desc);
    slab_desc.shape = slab;
    slab_desc.motion_type = ZJOLT_MOTION_TYPE_STATIC;
    slab_desc.object_layer = LAYER_STATIC;
    slab_desc.position.x = (ZJoltReal)50.0;
    slab_desc.position.y = (ZJoltReal)-0.5;
    ZJoltBodyId slab_id = ZJOLT_BODY_ID_INVALID;
    CHECK_OK(zjoltBodyCreateAndAdd(system, &slab_desc,
                                   ZJOLT_ACTIVATION_DONT_ACTIVATE, &slab_id));

    slab_desc.position.y = (ZJoltReal)4.0;
    slab_desc.is_sensor = true;
    ZJoltBodyId sensor_id = ZJOLT_BODY_ID_INVALID;
    CHECK_OK(zjoltBodyCreateAndAdd(system, &slab_desc,
                                   ZJOLT_ACTIVATION_DONT_ACTIVATE, &sensor_id));

    ZJoltSoftBodyContactListener soft_listener;
    soft_listener.on_contact_validate = onSoftValidate;
    soft_listener.on_contact_added = onSoftContactAdded;
    soft_listener.user = NULL;
    CHECK_OK(zjoltSoftBodySetContactListener(system, &soft_listener));

    for (int i = 0; i < 150; ++i)
      CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));

    CHECK(g_soft_validates > 0, "the soft body validate callback fired");
    CHECK(g_soft_default_settings == g_soft_validates,
          "with its contact settings filled in with their defaults");
    CHECK(g_soft_contacts_added > 0, "and the contact-added callback fired");
    CHECK(g_soft_body_seen == skin_id, "naming the soft body it was about");
    CHECK(g_soft_touching_vertices > 0, "with vertices actually touching");
    CHECK(g_soft_touching_vertices <= 8, "no more than the body has");
    CHECK(g_soft_contact_body == slab_id, "and they touched the floor");
    /* Pointing down, into the floor: the normal is the direction the soft
       body pushes, not the direction it is pushed. */
    CHECK(g_soft_contact_normal_y < -0.5f,
          "the contact normal points into what was touched (%.3f)",
          (double)g_soft_contact_normal_y);
    CHECK(g_soft_sensor_contacts > 0,
          "and the sensor it fell through was reported");

    CHECK_OK(zjoltSoftBodySetContactListener(system, NULL));
    int validates_before = g_soft_validates;
    CHECK_OK(zjoltPhysicsSystemStep(system, 1.0f / 60.0f, 1, jobs, NULL));
    CHECK(g_soft_validates == validates_before,
          "a cleared listener stops being called");

    zjoltBodyDestroy(system, skin_id);
    zjoltBodyDestroy(system, slab_id);
    zjoltBodyDestroy(system, sensor_id);
    zjoltShapeRelease(slab);
    zjoltSoftBodySharedSettingsRelease(skin);
    zjoltPhysicsMaterialRelease(canvas);
    zjoltPhysicsMaterialRelease(lid);
  }

  //-------------------------------------------------------------------------
  // Teardown
  //-------------------------------------------------------------------------

  zjoltCharacterDestroy(character);
  zjoltShapeRelease(character_shape);

  CHECK_OK(zjoltPhysicsSystemSetContactListener(system, NULL));
  CHECK_OK(zjoltPhysicsSystemSetBodyActivationListener(system, NULL));

  zjoltBodyDestroy(system, platform_id);
  zjoltBodyDestroy(system, ball_id);
  zjoltBodyDestroy(system, floor_id);
  CHECK(zjoltPhysicsSystemGetNumBodies(system) == 0, "every body destroyed");

  zjoltPhysicsSystemDestroy(system);

  zjoltShapeRelease(floor_shape);
  zjoltShapeRelease(hull);
  zjoltShapeRelease(decorated);
  zjoltShapeRelease(box);
  zjoltShapeRelease(sphere);

  /* Deinit must refuse while the job system is still alive: it would restore
     Jolt's allocator, and destroying anything afterwards would free through an
     allocator the memory never came from. */
  CHECK(zjoltLiveHandleCount() == 1, "the job system is still counted");
  zjoltDeinit();
  CHECK(zjoltIsInitialized(), "deinit refuses while a handle is alive");
  CHECK(g_trace_count > 0, "...and traces why");

  zjoltJobSystemDestroy(jobs);
  CHECK(zjoltLiveHandleCount() == 0, "every handle released");

  zjoltDeinit();
  CHECK(!zjoltIsInitialized(), "deinit takes effect");

  /* The point of the counting allocator: nothing may be outstanding. */
  CHECK(g_live_blocks == 0, "%ld allocations leaked (%zu bytes)", g_live_blocks,
        g_live_bytes);
  CHECK(g_live_bytes == 0, "%zu bytes leaked", g_live_bytes);

  printf("%ld allocations through the host allocator, all released\n",
         g_total_allocations);
  if (g_failures == 0) {
    printf("ok\n");
    return 0;
  }
  printf("%d failure(s)\n", g_failures);
  return 1;
}
