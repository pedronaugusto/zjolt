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
  if (zjoltShapeCreateSphere(0.5f, 1000.0f, NULL, &sphere) != ZJOLT_RESULT_OK) return 1;

  ZJoltShapeStats stats;
  zjoltShapeGetStats(sphere, &stats);
  if (stats.size_bytes == 0) return 1;
  zjoltShapeRelease(sphere);

  /* The library's own account of its layout, against what the headers in
   * scope here say. They agree because build.zig installs zjolt_config.h
   * beside the headers, carrying the options the library was compiled with;
   * without it a host had to repeat them by hand, and ZJoltReal came out the
   * wrong width. This is the check, and zjoltInit above is the second one --
   * it compares the caller's ZJOLT_CONFIG_ID with the library's. */
  ZJoltAbiLayout layout;
  zjoltAbiLayout(&layout);
  if (layout.real_size != (uint32_t)sizeof(ZJoltReal)) return 1;
  if (layout.object_layer_size != (uint32_t)sizeof(ZJoltObjectLayer)) return 1;
  if (layout.config_id != (uint32_t)ZJOLT_CONFIG_ID) return 1;

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
