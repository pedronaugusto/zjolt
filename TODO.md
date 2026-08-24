# Known gaps and debts

Written down rather than remembered. Each entry says what is wrong and why it
matters, because a list of tasks without reasons rots into a list of tasks
nobody can evaluate.

## Coverage

- **Behavioural tests are owed** for the subsystems added without them. The
  reflective sweeps prove every entry point compiles, refuses null and refuses a
  pre-init call; they prove nothing about behaviour. One characteristic test per
  subsystem is the bar: a hinge constrains to its axis, a ragdoll settles, a
  saved state restores bit-identically.
- **Shape-cast and collide-shape settings are unreachable.** Ray casts take a
  `ZJoltRayCastSettings`; shape casts and shape-vs-shape overlaps take none, at
  both the system level (`zjolt_query.h`) and the per-shape level
  (`zjolt_transformed.h`). So Jolt's `ShapeCastSettings` and
  `CollideShapeSettings` — back-face mode per convex and per triangle, active
  edge handling, whether the collector gets faces — are fixed at Jolt's
  defaults with no way to say otherwise. That is a caller-visible gap, not an
  implementation detail: back-face mode alone decides whether a sweep that
  starts inside geometry reports a hit.
- **`tools/coverage.sh` over-counts.** Its denominator includes Jolt internals
  that must never cross — SIMD helpers, quadtree nodes, per-class binary
  serialisation. Read the per-area lists; do not work the percentage.

## Structure

- **`src/c.zig` is one file.** Every subsystem appends to it, so parallel work
  collides there and nowhere else. Splitting it per subsystem means teaching
  `src/abi_check.zig` and `src/misuse_sweep_test.zig` to sweep a list of modules
  rather than one.

## Guards

- **The mutation test covers only ABI drift.** `ci/check-abi-drift.sh` proves
  the cross-check refuses twelve kinds of skew. Every other guard in the package
  — the allocator seam, the entry-point preamble, the callback error path — is
  unmutated, and a guard nothing tests is a guard nobody has checked. Widening
  it should also assert *which named test* catches each mutation, so a mutation
  that fails the build for an unrelated reason is reported rather than counted.
- **README numbers rot silently.** Test counts and entry-point counts are
  written by hand and nothing recomputes them.

## Settled

Kept here so the question is not re-opened; the reasoning lives next to the
code, and this is the pointer to it.

- **`ZJOLT_SHAPE_SUB_TYPE_OTHER` is gone, but a "kind I cannot name" value is
  not.** It stood for two different facts. Both are now named: `NONE` is zero
  and means the handle was NULL, and `USER_DEFINED` means a real shape from one
  of Jolt's sixteen `User*` slots, registered outside this library. Collapsing
  the second into the first would have `zjoltShapeGetSubType` answer "not a
  shape" about a shape. See the enum's own comment in `ffi/zjolt_core.h`.
