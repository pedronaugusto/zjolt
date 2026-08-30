#!/usr/bin/env bash
#
# zjolt — recompute every number the documentation claims.
#
# The README, BINDING.md and UPSTREAM.md quote counts: entry points, tests,
# translation units,
# how long a full run takes. All of them are written by hand, and none of them
# has anything that notices when the code moves underneath. A count that is
# quietly wrong is worse than no count, because a reader has no way to tell.
#
# This prints what is actually true right now. Run it before touching a number
# in a document, and paste what it says rather than what you remember.
#
# It measures; it does not judge, and nothing here fails a build. The judging
# is ci/check-numbers.sh, which recomputes the subset of these that a document
# states and fails when one disagrees. This script is the wider view: it also
# prints quantities no document quotes yet.
#
# Usage: ci/measurements.sh

set -euo pipefail
cd "$(dirname "$0")/.."

section() { printf '\n%s\n%s\n' "$1" "$(printf '%*s' "${#1}" '' | tr ' ' -)"; }

# Every formula lives in ci/counts.sh. This script chooses what to print and
# in what order; it may not count anything itself.
. ci/counts.sh

section "C ABI"

entry_points=$(count_api_decls)
printf 'entry points (ZJOLT_API in ffi/*.h)   %s\n' "$entry_points"
printf 'callable names (adds static inline)   %s\n' \
  "$(list_callable_names | grep -c .)"

externs=$(count_externs)
printf 'externs (pub extern fn in src/c/)     %s\n' "$externs"
if [ "$entry_points" != "$externs" ]; then
  printf '  ^ these must match; src/abi_check.zig pairs them at build time\n'
fi

printf 'public headers                        %s\n' "$(count_public_headers)"
printf 'result-returning entry points         %s\n' "$(count_returns_result)"
printf 'void entry points                     %s\n' "$(count_returns_void)"
printf 'version                               %s\n' "$(version_triple)"

section "Tests"

# The counts the build itself reports, rather than counting `test` keywords:
# a test inside a `comptime` block or behind a build option would be counted
# but never run.
zig_tests=$(zig build test --summary all 2>&1 |
  sed -n 's/.*run test zjolt-tests \([0-9]*\) pass.*/\1/p' | head -1)
printf 'zig tests (as run)                    %s\n' "${zig_tests:-unknown}"
printf 'test files                            %s\n' \
  "$(ls src/*_test.zig 2>/dev/null | wc -l | tr -d ' ')"
# Two debug-draw tests are gated on -Ddebug_renderer, so the default run skips
# one half of each. Reported separately rather than left to look like a gap.
printf 'zig tests (debug renderer on)         %s\n' \
  "$(zig build test -Ddebug_renderer=true --summary all 2>&1 |
      sed -n 's/.*run test zjolt-tests \([0-9]*\) pass.*/\1/p' | head -1)"
printf 'C smoke assertions                    %s\n' \
  "$(grep -c 'CHECK\|assert' tests/c_smoke.c 2>/dev/null || echo 0)"

section "Build size"

# What a single configuration actually compiles. This is the number that sets
# how long everything else takes.
printf 'Jolt translation units                %s\n' "$(count_jolt_tu)"
printf 'zjolt translation units               %s\n' "$(count_own_tu)"
printf 'total per configuration               %s\n' "$(count_total_tu)"
printf 'zig source lines (src/)               %s\n' \
  "$(cat src/*.zig src/c/*.zig | wc -l | tr -d ' ')"
printf 'C declaration modules (src/c/)        %s
' "$(count_c_modules)"
printf 'C++ source lines (ffi/)               %s\n' \
  "$(cat ffi/*.cpp ffi/*.h | wc -l | tr -d ' ')"

section "Guards"

printf 'ABI drift mutations                   %s\n' "$(count_drift_mutations)"
printf 'other-guard mutations                 %s\n' "$(count_other_mutations)"
printf 'mutations in all                      %s\n' "$(count_all_mutations)"
printf 'reflective sweeps                     %s\n' "$(count_reflective_sweeps)"
printf 'ci/run.sh host checks                 %s\n' "$(count_host_checks)"
printf 'ci/run.sh cross targets               %s\n' "$(count_cross_targets)"
printf 'ci/run.sh --full checks               %s\n' "$(count_full_checks)"

# Which of these a document quotes, and whether it quotes them correctly, is
# ci/check-numbers.sh's question. This one only reports.
printf '\n'
