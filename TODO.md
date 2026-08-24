# Known gaps and debts

Written down rather than remembered. Each entry says what is wrong and why it
matters, because a list of tasks without reasons rots into a list of tasks
nobody can evaluate.

## Correctness

- **`ZJOLT_SHAPE_SUB_TYPE_OTHER` still exists** even though every sub-type Jolt
  defines is now nameable. It remains the answer for a NULL handle and for the
  16 `User*` slots — so removing it needs a zero-valued "not a shape" first,
  or `zjoltShapeGetSubType(NULL)` would report `SPHERE`.

## Coverage

- **Behavioural tests are owed** for the subsystems added without them. The
  reflective sweeps prove every entry point compiles, refuses null and refuses a
  pre-init call; they prove nothing about behaviour. One characteristic test per
  subsystem is the bar: a hinge constrains to its axis, a ragdoll settles, a
  saved state restores bit-identically.
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
