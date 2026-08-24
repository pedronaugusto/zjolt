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
