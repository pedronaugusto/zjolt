#!/usr/bin/env bash
#
# zjolt — mutation test for the guards.
#
# `src/abi_check.zig` compares every extern declaration in `src/c.zig` against
# the real `ffi/zjolt.h` by reflection. It is the one test in this repo that
# cannot test itself: a refactor that quietly makes it vacuous — a name filter
# that matches nothing, a sweep that silently skips a category — looks exactly
# like a passing build. The coverage floors in `abi_check.zig` catch the crude
# version of that. Only mutation catches the subtle one.
#
# So this applies one deliberate drift at a time, asserts the build is REFUSED
# with a `zjolt ABI drift:` message, and reverts. Each mutation is a distinct
# kind of skew, chosen because it is the kind a human review would miss:
# notably the field swap, which leaves every offset in the struct unchanged and
# so defeats any positional or offsets-only comparison.
#
# Four of the mutations aim at the other guards instead — the entry-point
# preamble, the allocator seam, the callback error path, and the analysis
# sweep. Each of those names the test that must catch it, so a mutation that
# fails the build for an unrelated reason is reported as a WRONG FAILURE
# rather than counted as the guard doing its job. That distinction is not
# theoretical: three of the four were miscounted on the first run here, and
# the check said so.
#
# Not part of the default `ci/run.sh` — it rebuilds once per mutation. It runs
# under `--full`, and should be run by hand whenever a guard is edited.
#
# Usage: ci/check-abi-drift.sh

set -uo pipefail
cd "$(dirname "$0")/.."

pass=0
fail=0
backups=()

restore() {
  local f
  for f in "${backups[@]:-}"; do
    [ -n "$f" ] && [ -f "$f.bak" ] && mv "$f.bak" "$f"
  done
  backups=()
}
# A killed run must not leave a mutated source behind.
trap 'restore; exit 130' INT TERM

# try <description> <file> <from> <to>
#
# Asserts the ABI cross-check refuses the mutation, by its own message.
try() {
  expect 'zjolt ABI drift: .*' "$@"
}

# expect <signal> <description> <file> <from> <to>
#
# `signal` is a grep pattern the build output must contain. Requiring a
# specific signal rather than merely a non-zero exit is the whole point: a
# mutation that fails the build for an unrelated reason — a typo in the
# replacement, a stale anchor landing somewhere odd — would otherwise be
# counted as the guard doing its job. Every mutation below names the thing
# that is supposed to notice it.
expect() {
  local signal="$1" what="$2" file="$3" from="$4" to="$5"

  cp "$file" "$file.bak"
  backups=("$file")

  if ! python3 - "$file" "$from" "$to" <<'PY'
import pathlib, sys
path, before, after = sys.argv[1], sys.argv[2], sys.argv[3]
p = pathlib.Path(path)
s = p.read_text()
if before not in s:
    sys.exit("anchor no longer present in %s:\n%s" % (path, before))
p.write_text(s.replace(before, after, 1))
PY
  then
    printf '  ANCHOR STALE  %s\n' "$what"
    fail=$((fail + 1))
    restore
    return
  fi

  local out status
  out=$(zig build test 2>&1)
  status=$?
  restore

  if [ $status -eq 0 ]; then
    printf '  NOT CAUGHT    %s\n' "$what"
    fail=$((fail + 1))
    return
  fi

  local msg
  msg=$(printf '%s' "$out" | grep -m1 -o "$signal")
  if [ -z "$msg" ]; then
    # The build failed, but not for the reason this mutation was aimed at.
    # That is not the guard doing its job, and a green count here would be a
    # lie about which guard is load-bearing.
    printf '  WRONG FAILURE %s\n' "$what"
    printf '                expected to see: %s\n' "$signal"
    printf '%s\n' "$out" | tail -5 | sed 's/^/      | /'
    fail=$((fail + 1))
    return
  fi

  printf '  caught        %s\n' "$what"
  printf '                -> %s\n' "$msg"
  pass=$((pass + 1))
}

# A clean tree first: a mutation is only evidence if the unmutated build passes.
if ! zig build test >/dev/null 2>&1; then
  echo "the unmutated build already fails; fix that before reading this script's output"
  exit 1
fi

echo "drift the ABI cross-check must refuse:"

# Same-sized adjacent fields swapped. The offset SEQUENCE is unchanged, so
# every positional check and every offsets-only digest passes this.
try "same-sized adjacent fields swapped" src/c.zig \
'pub const Vec3 = extern struct {
    x: f32,
    y: f32,' \
'pub const Vec3 = extern struct {
    y: f32,
    x: f32,'

try "a parameter dropped from a function" src/c.zig \
'pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;' \
'pub extern fn zjoltShapeCreateSphere(radius: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;'

try "a parameter widened (f32 -> f64)" src/c.zig \
'pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;' \
'pub extern fn zjoltShapeCreateSphere(radius: f64, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;'

try "an enumerator renumbered" src/c.zig \
'    in_air = 3,' \
'    in_air = 7,'

try "an enum tag narrowed (c_int -> i8)" src/c.zig \
'pub const MotionType = enum(c_int) {' \
'pub const MotionType = enum(i8) {'

try "a bit mask bit moved" src/c.zig \
'pub const UpdateError = packed struct(u32) {
    manifold_cache_full: bool = false,
    body_pair_cache_full: bool = false,' \
'pub const UpdateError = packed struct(u32) {
    body_pair_cache_full: bool = false,
    manifold_cache_full: bool = false,'

try "a constant drifted" src/c.zig \
'pub const max_physics_jobs: u32 = 2048;' \
'pub const max_physics_jobs: u32 = 4096;'

# The reverse direction: the header declares something c.zig does not.
try "an extern deleted from c.zig" src/c.zig \
'pub extern fn zjoltBodyGetPosition(body: *const Body, out: *RVec3) void;' \
''

# The other reverse direction: c.zig is missing a field the header has.
try "a struct field added in the header only" ffi/zjolt_shape.h \
'typedef struct ZJoltShapeStats {' \
'typedef struct ZJoltShapeStats {
  uint32_t intruder;'

# Signedness, which size and alignment cannot see: same width, same offset,
# same everything a layout digest looks at, and a value above 2^31 read back as
# negative. Mutated on the HEADER side so the Zig suite still type-checks —
# flipping c.zig instead breaks an unrelated test first, which is not evidence
# about this check.
try "a field's signedness flipped in the header" ffi/zjolt_shape.h \
'  uint32_t num_triangles;' \
'  int32_t num_triangles;'

# A negative enumerator is not drift between the two sides — it is drift
# between two toolchains. C leaves an enum's underlying type to the
# implementation, and the choice only stops mattering while every enumerator is
# non-negative. This is the precondition that lets the signedness comparison
# above skip enums, so it has to be enforced, not assumed.
try "a negative enumerator introduced" src/c.zig \
'pub const Result = enum(c_int) {
    ok = 0,' \
'pub const Result = enum(c_int) {
    ok = -1,'

# A Zig helper wearing an exported symbol's name. Both sweeps used to let this
# through: the forward sweep skips non-extern functions, and the reverse sweep
# asked only whether the name existed. The extern it displaced would be gone
# with neither direction noticing.
try "an extern replaced by a Zig helper of the same name" src/c.zig \
'pub extern fn zjoltBodyGetPosition(body: *const Body, out: *RVec3) void;' \
'pub fn zjoltBodyGetPosition(body: *const Body, out: *RVec3) void {
    _ = body;
    _ = out;
}'

#=============================================================================
# The other guards
#
# Everything above mutates the ABI cross-check. It is not the only guard in
# this repository, and a guard nothing tests is a guard nobody has checked —
# so each of the rest gets one deliberate break, and each names the test that
# is supposed to notice.
#=============================================================================

echo
echo "other guards, each named by the test that must catch it:"

# The entry-point preamble. Every entry point opens with ZJOLT_ENTER, which is
# what turns a call made before zjoltInit into ZJOLT_RESULT_NOT_INITIALIZED
# rather than a walk through Jolt's uninitialised allocator. Take it off one
# entry point and the sweep that calls all of them before init must say so.
expect 'every entry point refuses a call made before init' \
  "the entry-point preamble removed from one entry point" ffi/zjolt_shape.cpp \
'                                   ZJoltShape **out) {
  ZJOLT_ENTER(out);' \
'                                   ZJoltShape **out) {'

# The allocator seam. Every Jolt allocation is supposed to go through the
# host allocator. Leave Jolt on its own and nothing crashes — the numbers
# simply stop being true, which is exactly the kind of break that survives a
# casual read. The C smoke test counts, so it is the one that must notice.
expect 'the allocator seam is not being used' \
  "the allocator seam bypassed" ffi/zjolt_core.cpp \
'    g_allocator = host;
    JPH::Allocate = HostAllocate;
    JPH::Reallocate = HostReallocate;
    JPH::Free = HostFree;
    JPH::AlignedAllocate = HostAlignedAllocate;
    JPH::AlignedFree = HostAlignedFree;' \
'    g_allocator = host;
    JPH::RegisterDefaultAllocator();'

# The callback error path. Nothing may unwind out of a Jolt callback, so a
# failing callback stashes its error for `check` to raise afterwards. Drop the
# stash and the failure is swallowed silently, which is the worst possible
# shape for it.
expect 'the contact listener fires' \
  "a failing callback's error dropped instead of stashed" src/character.zig \
'    fn record(self: *Failure, e: anyerror) void {
        _ = self.code.cmpxchgStrong(0, @intFromError(e), .monotonic, .monotonic);
    }' \
'    fn record(self: *Failure, e: anyerror) void {
        _ = self;
        var swallowed = e;
        _ = &swallowed;
    }'

# The analysis sweep. Zig never semantically analyses a `pub fn` nobody calls,
# so a wrapper can be broken and still ship; src/analysis_test.zig takes the
# address of every one to force it. Break a wrapper no test calls, and the
# sweep is the only thing between that and a green build.
expect 'zjoltTransformedShapeGetWorldSpaceBoundsTYPO' \
  "a wrapper no test calls left referring to a name that is gone" src/transformed.zig \
'        c.zjoltTransformedShapeGetWorldSpaceBounds(self.handle, &out);' \
'        c.zjoltTransformedShapeGetWorldSpaceBoundsTYPO(self.handle, &out);'

printf '\ncaught: %d   missed: %d\n' "$pass" "$fail"
[ $fail -eq 0 ]
