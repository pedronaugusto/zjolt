#!/usr/bin/env bash
#
# zjolt — recompute every number the documentation claims.
#
# The README and TODO.md quote counts: entry points, tests, translation units,
# how long a full run takes. All of them are written by hand, and none of them
# has anything that notices when the code moves underneath. A count that is
# quietly wrong is worse than no count, because a reader has no way to tell.
#
# This prints what is actually true right now. Run it before touching a number
# in a document, and paste what it says rather than what you remember.
#
# It measures; it does not judge. Nothing here fails a build — the numbers are
# for prose, and prose is a human's job to keep honest.
#
# Usage: ci/measurements.sh

set -euo pipefail
cd "$(dirname "$0")/.."

section() { printf '\n%s\n%s\n' "$1" "$(printf '%*s' "${#1}" '' | tr ' ' -)"; }

section "C ABI"

# One ZJOLT_API per entry point, and the macro appears nowhere else.
entry_points=$(grep -rhc '^ZJOLT_API' ffi/*.h | paste -sd+ - | bc)
printf 'entry points (ZJOLT_API in ffi/*.h)   %s\n' "$entry_points"

# The Zig mirror of the same set. abi_check.zig fails the build if these ever
# disagree, so a difference here means this script is counting wrong.
externs=$(grep -c '^pub extern fn zjolt' src/c.zig)
printf 'externs (pub extern fn in src/c.zig)  %s\n' "$externs"
if [ "$entry_points" != "$externs" ]; then
  printf '  ^ these must match; src/abi_check.zig pairs them at build time\n'
fi

printf 'public headers                        %s\n' "$(ls ffi/*.h | wc -l | tr -d ' ')"

section "Tests"

# The counts the build itself reports, rather than counting `test` keywords:
# a test inside a `comptime` block or behind a build option would be counted
# but never run.
zig_tests=$(zig build test --summary all 2>&1 |
  sed -n 's/.*run test zjolt-tests \([0-9]*\) pass.*/\1/p' | head -1)
printf 'zig tests (as run)                    %s\n' "${zig_tests:-unknown}"
printf 'test files                            %s\n' \
  "$(ls src/*_test.zig 2>/dev/null | wc -l | tr -d ' ')"
printf 'C smoke assertions                    %s\n' \
  "$(grep -c 'CHECK\|assert' tests/c_smoke.c 2>/dev/null || echo 0)"

section "Build size"

# What a single configuration actually compiles. This is the number that sets
# how long everything else takes.
jolt_tu=$(find libs/JoltPhysics/Jolt -name '*.cpp' | wc -l | tr -d ' ')
own_tu=$(ls ffi/*.cpp | wc -l | tr -d ' ')
printf 'Jolt translation units                %s\n' "$jolt_tu"
printf 'zjolt translation units               %s\n' "$own_tu"
printf 'total per configuration               %s\n' "$((jolt_tu + own_tu))"
printf 'zig source lines (src/)               %s\n' \
  "$(cat src/*.zig | wc -l | tr -d ' ')"
printf 'C++ source lines (ffi/)               %s\n' \
  "$(cat ffi/*.cpp ffi/*.h | wc -l | tr -d ' ')"

section "Guards"

# One `try` per mutation. ci/run.sh names this count in a label, so it has to
# be kept in step by hand — this is where to read the true one.
printf 'ABI drift mutations                   %s\n' \
  "$(grep -c '^try ' ci/check-abi-drift.sh)"
printf 'ci/run.sh checks (static)             %s\n' \
  "$(grep -cE "^ *run ['\"]" ci/run.sh)"
printf 'ci/run.sh cross targets               %s\n' \
  "$(sed -n '/^ *for target in/,/^ *do$/p' ci/run.sh | grep -cE '^ +[a-z0-9_]+-')"

# The one number in ci/run.sh that is written into a label rather than
# computed, so it is the one that can silently disagree with the script it
# names.
label=$(sed -n "s/.*abi drift (\([0-9]*\) mutations).*/\1/p" ci/run.sh)
actual=$(grep -c '^try ' ci/check-abi-drift.sh)
if [ -n "$label" ] && [ "$label" != "$actual" ]; then
  printf "\n  ci/run.sh's label says %s mutations; check-abi-drift.sh has %s\n" \
    "$label" "$actual"
fi

printf '\n'
