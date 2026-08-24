#!/usr/bin/env bash
#
# zjolt — mutation test for the ABI cross-check.
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
# Not part of the default `ci/run.sh` — it rebuilds once per mutation. It runs
# under `--full`, and should be run by hand whenever `abi_check.zig` is edited.
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
try() {
  local what="$1" file="$2" from="$3" to="$4"

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
  msg=$(printf '%s' "$out" | grep -m1 -o 'zjolt ABI drift: .*')
  if [ -z "$msg" ]; then
    # The build failed, but for some other reason — that is not the check
    # doing its job, and a green count here would be a lie.
    printf '  WRONG FAILURE %s\n' "$what"
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
'pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, out: **Shape) Result;' \
'pub extern fn zjoltShapeCreateSphere(radius: f32, out: **Shape) Result;'

try "a parameter widened (f32 -> f64)" src/c.zig \
'pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, out: **Shape) Result;' \
'pub extern fn zjoltShapeCreateSphere(radius: f64, density: f32, out: **Shape) Result;'

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

printf '\ncaught: %d   missed: %d\n' "$pass" "$fail"
[ $fail -eq 0 ]
