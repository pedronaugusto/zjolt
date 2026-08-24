/* A downstream C consumer, using the installed header and artifact directly.
 *
 * The point is the #include: it resolves only if build.zig installs the
 * umbrella header AND every header the umbrella includes, under the spellings
 * `ffi/zjolt.h` uses. In-repo that is invisible, because the whole `ffi/`
 * directory is on the include path. */
#include <stdio.h>

#include <zjolt.h>

int main(void) {
  if (zjoltInit(NULL) != ZJOLT_RESULT_OK) return 1;

  ZJoltShape *sphere = NULL;
  if (zjoltShapeCreateSphere(0.5f, 1000.0f, &sphere) != ZJOLT_RESULT_OK) return 1;

  ZJoltShapeStats stats;
  zjoltShapeGetStats(sphere, &stats);
  if (stats.size_bytes == 0) return 1;
  zjoltShapeRelease(sphere);

  /* The library's own account of its layout, which is the thing a C host
   * cannot get from the header alone -- ZJoltReal changes width with the
   * build options the library was compiled with, not the ones in scope here. */
  ZJoltAbiLayout layout;
  zjoltAbiLayout(&layout);

  const uint32_t v = zjoltVersion();
  const uint32_t j = zjoltJoltVersion();
  printf("c consumer ok: zjolt %u.%u.%u, jolt %u.%u.%u, real %u bytes\n",
         v >> 16, (v >> 8) & 0xFFu, v & 0xFFu,
         j >> 16, (j >> 8) & 0xFFu, j & 0xFFu,
         (unsigned)layout.real_size);

  zjoltDeinit();
  if (zjoltIsInitialized()) return 1;
  return 0;
}
