#!/usr/bin/env bash
#
# zjolt — the CI matrix, run locally.
#
# This mirrors .github/workflows/ci.yml so a failure can be reproduced and
# fixed on your own machine instead of in a pull request.
#
# Usage:
#   ci/run.sh          # the inner loop: hygiene, native Debug, the C smoke test
#   ci/run.sh --full   # everything CI runs, minus the jobs that need network
#
# The default is trimmed rather than complete, and that is a deliberate
# concession to what Jolt is: 130 translation units, so a single configuration
# is tens of seconds rather than the couple of seconds a smaller library would
# take. `--full` adds four more optimize modes, eight cross-compilation targets
# and six build configurations, which is several minutes. Run the default while
# working and --full before pushing something structural.
#
# `ci/verify-vendor.sh` is not run here: it needs network, so it is its own CI
# job.
#
# The one difference from the hosted run: CI executes the suite on Linux, macOS
# and Windows, whereas this executes it on whichever host you are on and
# cross-compiles the rest.
#
# Exits non-zero if any step fails, after running every step — a single failure
# should not hide the others.

set -uo pipefail
cd "$(dirname "$0")/.."

FULL=0
[ "${1:-}" = "--full" ] && FULL=1

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
else
  RED=; GREEN=; DIM=; BOLD=; OFF=
fi

PASSED=0
FAILED=0
FAILED_NAMES=()
STARTED=$(date +%s)

# run <name> <command...>
run() {
  local name="$1"; shift
  printf '  %-46s' "$name"
  local start output status
  start=$(date +%s)
  output=$("$@" 2>&1)
  status=$?
  local elapsed=$(( $(date +%s) - start ))

  if [ $status -eq 0 ]; then
    printf '%sok%s %s(%ds)%s\n' "$GREEN" "$OFF" "$DIM" "$elapsed" "$OFF"
    PASSED=$((PASSED + 1))
  else
    printf '%sFAILED%s %s(%ds)%s\n' "$RED" "$OFF" "$DIM" "$elapsed" "$OFF"
    FAILED=$((FAILED + 1))
    FAILED_NAMES+=("$name")
    printf '%s' "$output" | sed 's/^/      | /' | head -40
  fi
}

section() { printf '\n%s%s%s\n' "$BOLD" "$1" "$OFF"; }

printf '%szjolt local CI%s  %s%s%s\n' "$BOLD" "$OFF" "$DIM" "$(zig version)" "$OFF"
[ $FULL -eq 0 ] && printf '%s(trimmed — run with --full for the whole matrix)%s\n' \
  "$DIM" "$OFF"

#-----------------------------------------------------------------------------
section 'Hygiene'
#-----------------------------------------------------------------------------

# Only our own Zig sources: libs/JoltPhysics is vendored verbatim and must not
# be reformatted, or the next re-vendor becomes an unreadable diff.
run 'zig fmt (src, build.zig)' zig fmt --check src build.zig

#-----------------------------------------------------------------------------
section 'Tests — native'
#-----------------------------------------------------------------------------

# Default config: Zig's C sanitizer is on in Debug, so this is the run that
# would catch undefined behaviour in our own code.
run 'test Debug (UBSan on)' zig build test -Doptimize=Debug

# The C boundary on its own, with no Zig in the picture.
run 'test-c (C ABI standalone)' zig build test-c

if [ $FULL -eq 1 ]; then
  run 'test Debug (UBSan off)' zig build test -Doptimize=Debug -Dsanitize_c=false
  for mode in ReleaseSafe ReleaseFast ReleaseSmall; do
    run "test $mode" zig build test -Doptimize="$mode"
  done

  #---------------------------------------------------------------------------
  section 'Tests — build configurations'
  #---------------------------------------------------------------------------

  # These change what the C++ compiles to, and two of them change the width of
  # types in the ABI, so each one has to be EXECUTED rather than merely built —
  # a compile proves the macros agree, only a run proves the layouts do.
  run 'test double precision' zig build test -Ddouble_precision=true
  run 'test 32-bit object layers' zig build test -Dobject_layer_bits=32
  run 'test double + 32-bit layers' \
    zig build test -Ddouble_precision=true -Dobject_layer_bits=32
  run 'test cross-platform deterministic' \
    zig build test -Dcross_platform_deterministic=true
  run 'test asserts off' zig build test -Denable_asserts=false
  run 'build shared library' zig build -Dshared=true

  #---------------------------------------------------------------------------
  section 'Cross-compilation'
  #---------------------------------------------------------------------------

  # Compile-only. These prove the sources and build graph are portable; the
  # tests above are what prove behaviour, on this host. CI executes the suite
  # on Linux, macOS and Windows as well.
  for target in \
    x86_64-linux-gnu \
    aarch64-linux-gnu \
    x86_64-linux-musl \
    aarch64-linux-musl \
    x86_64-windows-gnu \
    aarch64-windows-gnu \
    x86_64-macos \
    aarch64-macos
  do
    run "build $target" zig build -Dtarget="$target"
  done

  # x86_64-windows-msvc is absent here because it needs the Microsoft standard
  # library, which a non-Windows host does not have. CI covers it natively on a
  # Windows runner.
fi

#-----------------------------------------------------------------------------
printf '\n'
TOTAL=$(( $(date +%s) - STARTED ))
if [ $FAILED -eq 0 ]; then
  printf '%s%d passed, 0 failed%s %s(%ds)%s\n' "$GREEN" "$PASSED" "$OFF" \
    "$DIM" "$TOTAL" "$OFF"
  exit 0
fi

printf '%s%d passed, %d FAILED%s %s(%ds)%s\n' "$RED" "$PASSED" "$FAILED" "$OFF" \
  "$DIM" "$TOTAL" "$OFF"
for name in "${FAILED_NAMES[@]}"; do
  printf '  %s- %s%s\n' "$RED" "$name" "$OFF"
done
exit 1
