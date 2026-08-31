#!/usr/bin/env bash
#
# zjolt — mutation test for the guards.
#
# `src/abi_check.zig` compares every extern declaration in `src/c/` against
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
# 19 of the mutations aim at the other guards instead — the entry-point
# preamble, the allocator seam, the callback error path, the analysis sweep,
# the coverage classifier, the host reference count, the comment budget, and
# the two guards over the documents. Only the total is written down, because it is the only one
# ci/check-numbers.sh recomputes; a sub-count spelled in words is exactly what
# that gate cannot see. Each of those names the
# test that must catch it, so a mutation that fails the build for an unrelated
# reason is reported as a WRONG FAILURE rather than counted as the guard doing
# its job. Without that distinction a mutation refused for the wrong reason
# reads as a guard that works.
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

# The document mutations below edit a COUNT, so their anchors are counts too.
# Written out, they rot the moment the tree moves and the mutation reports
# ANCHOR STALE -- a guard's own text going stale for exactly the reason the
# guard over documents exists. Both come from the one home instead.
. ci/counts.sh
readme_entry_points=$(count_api_decls)
readme_sweeps=$(count_reflective_sweeps)

# What `expect` runs to see whether a mutation is refused. Almost every
# mutation below is answered by the ABI cross-check, which lives inside the
# test build; the coverage guard is a script, and rebuilding the world to ask
# it a question it answers in a second would be silly.
BUILD='zig build test'

# The one-anchor edit every mutation below is made with, kept in a file rather
# than piped in on stdin. `python3 - <file> ...` is ambiguous wherever
# `python3` is a launcher that dispatches on a shebang: handed a shell script
# to mutate, it runs THAT and never reads the program at all, and the mutation
# is reported as a stale anchor — a false verdict about the tree drawn from a
# detail of the host.
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cat > "$work/mutate.py" <<'MUTATE'
import sys

path, before, after = sys.argv[1], sys.argv[2], sys.argv[3]

# newline='' both ways. A mutation has to change its one anchor and nothing
# else; left to translate, a Windows host rewrites every line ending in the
# file, and a guard that reads a line's exact spelling then fails for a reason
# the mutation never aimed at — a wrong failure that proves nothing.
# open() rather than Path.read_text/write_text: the newline keyword
# reached those only in Python 3.13, and hosted runners are older.
with open(path, newline='') as f:
    s = f.read()
if before not in s:
    sys.exit("anchor no longer present in %s:\n%s" % (path, before))
with open(path, 'w', newline='') as f:
    f.write(s.replace(before, after, 1))
MUTATE

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

  if ! python3 "$work/mutate.py" "$file" "$from" "$to"
  then
    printf '  ANCHOR STALE  %s\n' "$what"
    fail=$((fail + 1))
    restore
    return
  fi

  local out status
  out=$(eval "$BUILD" 2>&1)
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
try "same-sized adjacent fields swapped" src/c/core.zig \
'pub const Vec3 = extern struct {
    x: f32,
    y: f32,' \
'pub const Vec3 = extern struct {
    y: f32,
    x: f32,'

try "a parameter dropped from a function" src/c/shape.zig \
'pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;' \
'pub extern fn zjoltShapeCreateSphere(radius: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;'

try "a parameter widened (f32 -> f64)" src/c/shape.zig \
'pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;' \
'pub extern fn zjoltShapeCreateSphere(radius: f64, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;'

try "an enumerator renumbered" src/c/core.zig \
'    in_air = 3,' \
'    in_air = 7,'

try "an enum tag narrowed (c_int -> i8)" src/c/core.zig \
'pub const MotionType = enum(c_int) {' \
'pub const MotionType = enum(i8) {'

try "a bit mask bit moved" src/c/core.zig \
'pub const UpdateError = packed struct(u32) {
    manifold_cache_full: bool = false,
    body_pair_cache_full: bool = false,' \
'pub const UpdateError = packed struct(u32) {
    body_pair_cache_full: bool = false,
    manifold_cache_full: bool = false,'

try "a constant drifted" src/c/core.zig \
'pub const max_physics_jobs: u32 = 2048;' \
'pub const max_physics_jobs: u32 = 4096;'

# The reverse direction: the header declares something c.zig does not.
try "an extern deleted from the Zig side" src/c/body.zig \
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
try "a negative enumerator introduced" src/c/core.zig \
'pub const Result = enum(c_int) {
    ok = 0,' \
'pub const Result = enum(c_int) {
    ok = -1,'

# A Zig helper wearing an exported symbol's name. Both sweeps used to let this
# through: the forward sweep skips non-extern functions, and the reverse sweep
# asked only whether the name existed. The extern it displaced would be gone
# with neither direction noticing.
try "an extern replaced by a Zig helper of the same name" src/c/body.zig \
'pub extern fn zjoltBodyGetPosition(body: *const Body, out: *RVec3) void;' \
'pub fn zjoltBodyGetPosition(body: *const Body, out: *RVec3) void {
    _ = body;
    _ = out;
}'

# A module silently dropped from src/c.zig's list. This is the failure mode the
# split introduced and the one that would be worst: both guards discover what
# to check by walking that list, so a module missing from it is a module
# neither of them covers, and nothing else would say so.
try "a module dropped from src/c.zig's module list" src/c.zig \
'    vehicle,
' \
''

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
#
# The anchor names its entry point, and names one tests/c_smoke.c does not
# call before init. c_smoke checks that property by hand on
# zjoltShapeCreateSphere and runs first, so an anchor matching a nameless
# `**out) {` left the C test answering for a mutation the SWEEP has to catch:
# reported as a WRONG FAILURE rather than counted, but proving nothing either.
expect 'every entry point refuses a call made before init' \
  "the entry-point preamble removed from one entry point" ffi/zjolt_shape.cpp \
'ZJoltResult zjoltShapeCreateEmpty(const ZJoltVec3 *center_of_mass,
                                  ZJoltShape **out) {
  ZJOLT_ENTER(out);' \
'ZJoltResult zjoltShapeCreateEmpty(const ZJoltVec3 *center_of_mass,
                                  ZJoltShape **out) {'

# The sweeps' own floors. Each sweep asserts it probed at least a stated
# number of entry points, because a sweep that quietly stops matching passes
# in silence — there is nothing in its output to notice. Narrow the predicate
# deciding which entry points take a pointer at all: the null sweep goes on
# working and simply covers far less of the ABI, and its floor is the only
# thing left that can tell.
expect "every entry point survives null pointers' failed" \
  "a sweep predicate narrowed so the sweep quietly covers less" \
  src/misuse_sweep_test.zig \
'        if (info == .pointer) return true;
        if (info == .optional and @typeInfo(info.optional.child) == .pointer) return true;' \
'        if (info == .optional and @typeInfo(info.optional.child) == .pointer) return true;'

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

#-----------------------------------------------------------------------------
# The coverage guard.
#
# `ci/check-coverage.sh` is the answer to "does zjolt cover Jolt", and it is
# load-bearing in the same way the ABI cross-check is: if it goes vacuous it
# reports full coverage and nobody notices. It replaced prose that had answered
# the same question five times with five different numbers, so the failure mode
# is not hypothetical.
#
# The anchor lines all come from tools/verdicts_ragdoll.txt, the one
# classification file small and settled enough that a mutation here will not
# collide with real work.
#-----------------------------------------------------------------------------
BUILD='ci/check-coverage.sh'

# A mutation is only evidence if the unmutated run passes. That matters more
# here than above: `ci/check-coverage.sh` exits non-zero while any name is
# still open, so against a red baseline all five mutations below would report
# "caught" without having caught anything.
if ! ci/check-coverage.sh >/dev/null 2>&1; then
  echo
  echo "  SKIPPED       the five coverage mutations"
  echo "                ci/check-coverage.sh already fails, so they would all"
  echo "                report a catch without catching anything. Close the"
  echo "                open names first."
  fail=$((fail + 1))
else

expect 'is not in ffi/\*\.h' \
  "evidence naming an entry point that does not exist" tools/verdicts_ragdoll.txt \
  "$(printf 'Physics/Ragdoll\tGetBodyCount\tBOUND\tzjoltRagdollGetBodyIds')" \
  "$(printf 'Physics/Ragdoll\tGetBodyCount\tBOUND\tzjoltRagdollGetBodyIdList')"

expect 'not four tab-separated fields' \
  "a line's tabs turned into spaces" tools/verdicts_ragdoll.txt \
  "$(printf 'Physics/Ragdoll\tGetBodyCount\tBOUND\tzjoltRagdollGetBodyIds')" \
  'Physics/Ragdoll  GetBodyCount  BOUND  zjoltRagdollGetBodyIds'

expect 'no verdict' \
  "a classified name deleted" tools/verdicts_ragdoll.txt \
  "$(printf 'Physics/Ragdoll\tGetBodyCount\tBOUND\tzjoltRagdollGetBodyIds')" \
  "$(printf '#Physics/Ragdoll\tGetBodyCount\tBOUND\tzjoltRagdollGetBodyIds')"

expect 'gap(s)' \
  "a settled name reopened as a gap" tools/verdicts_ragdoll.txt \
  "$(printf 'Physics/Ragdoll\tGetBodyCount\tBOUND\tzjoltRagdollGetBodyIds')" \
  "$(printf 'Physics/Ragdoll\tGetBodyCount\tGAP\t')"

# The enumerator itself. Loosening the name match is the subtle one: it makes
# more names look bound, which shrinks the list the classification has to
# explain and would quietly turn real questions into answered ones. What
# notices is that the lines explaining them become stale.
expect 'stale line' \
  "the name matcher loosened to ignore word order" tools/coverage.sh \
'        k = 1
        for (i = 1; i <= bn[j] && k <= nw; i++) if (b[j, i] == w[k]) k++
        if (k > nw) hit = 1' \
'        k = 0
        for (i = 1; i <= bn[j]; i++) for (m = 1; m <= nw; m++) if (b[j, i] == w[m]) k++
        if (k >= nw) hit = 1'

fi

#-----------------------------------------------------------------------------
# The two guards over the documents.
#
# `ci/check-examples.sh` proves BINDING.md's walk-through is a verbatim
# excerpt of the tree, and `ci/check-numbers.sh` proves every count a document
# states is the tree's own. Both fail the same way if they go vacuous: a green
# line and a document nobody has checked. Both mutate a DOCUMENT, so neither
# can disturb a build.
#-----------------------------------------------------------------------------
BUILD='ci/check-examples.sh'

expect 'is not in src/shape.zig character for character' \
  "a documented example left with a signature the tree no longer has" BINDING.md \
  'pub fn initSphere(radius: f32, opts: ConvexOptions)' \
  'pub fn initSphere(radius: f32, density: f32)'

expect 'names no file' \
  "an example's file label removed, so nothing knows what to check it against" \
  BINDING.md \
'`src/c/shape.zig`
```zig' \
'```zig'

BUILD='ci/check-numbers.sh'

expect 'README.md says' \
  "a documented count edited away from the tree" README.md \
  "**$readme_entry_points C entry points**" \
  "**$((readme_entry_points - 1)) C entry points**"

expect 'phrase not found' \
  "a gated claim reworded until its pattern matches nothing" README.md \
  "$readme_sweeps reflective sweeps" \
  'several reflective sweeps'

expect 'is not a BOUND row naming' \
  "a documented ledger example pointed at a row the ledger does not have" \
  README.md \
  '`BodyInterface::ActivateBodiesInAABox` is `BOUND` because' \
  '`BodyInterface::ActivateBodiesInABox` is `BOUND` because'

BUILD='ci/check-comments.sh'

expect 'comment characters in one block' \
  "a comment block grown past its budget by leaving one line unwrapped" \
  ffi/zjolt_core.h \
  '/// Static, never-NULL description of a result code. Borrowed; do not free.' \
  '/// Static, never-NULL description of a result code. Borrowed; do not free. This one sentence carries more prose than the whole budget for a block above a declaration allows, and it carries it on a single line, which is the shape a rule counting newlines cannot see at all: six lines is six lines whether they hold forty characters or four hundred, so the budget has to be spent in characters or it is not a budget, and this sentence spends it, at a length no reviewer would let through and no line count would ever notice.'

#-----------------------------------------------------------------------------
# The guard over the host reference count.
#
# `ci/check-refcounts.sh` is what keeps zjoltLiveHandleCount honest, and its
# two rules fail in opposite directions: a refcount moved outside the two
# chokepoints moves Jolt's count and not this one, and an AddRef that stops
# counting while its Release does not drives the total NEGATIVE, which reads
# as "nothing outstanding". Both mutations are one line of C++.
#-----------------------------------------------------------------------------
BUILD='ci/check-refcounts.sh'

expect 'moves outside zjolt::HostRetain' \
  "a refcount moved by hand instead of through the chokepoint" \
  ffi/zjolt_shape.cpp \
  '  zjolt::HostRetain(zjolt::ToJolt(shape));' \
  '  zjolt::ToJolt(shape)->AddRef();'

expect 'do not account the same way' \
  "one end of a pair stops counting while the other keeps counting" \
  ffi/zjolt_material.cpp \
  '  zjolt::HostRetain(zjolt::ToJolt(material));' \
  '  zjolt::ToJolt(material)->AddRef();
  zjolt::HandleDestroyed();'

expect 'has 3 refcount call' \
  "a third home for the arithmetic, beside HostRetain and HostRelease" \
  ffi/zjolt_internal.h \
  '  object->Release();' \
  '  object->Release();
  object->Release();'


BUILD='zig build test'

printf '\ncaught: %d   missed: %d\n' "$pass" "$fail"
[ $fail -eq 0 ]
