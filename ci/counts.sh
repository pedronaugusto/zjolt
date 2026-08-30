#!/usr/bin/env bash
#
# zjolt — every count the tree can state about itself, defined ONCE.
#
# ci/measurements.sh prints these, ci/check-numbers.sh compares documents
# against them, ci/check-coverage.sh reports two of them beside its own
# tallies. Each carried its own copy of the formula, and two copies of one
# formula are two facts free to disagree: one quantity was printed by two of
# those scripts as two different numbers, with nothing able to say which was
# meant.
#
# Sourced, never executed: `. ci/counts.sh` from the repository root. Every
# function echoes one number, or one space-separated tuple.

#===----------------------------------------------------------------------===//
# The C ABI
#===----------------------------------------------------------------------===//

# Declarations carrying the export macro, one per line, and the macro appears
# nowhere else. This is the number a consumer links against.
count_api_decls() {
  grep -rhc '^ZJOLT_API' ffi/*.h | awk '{ n += $1 } END { print n + 0 }'
}

# Every zjolt-prefixed name a C caller can write, which is the exported set
# plus ZJOLT_API declarations whose name wraps onto a continuation line plus
# the static inline zjoltInit. Larger than count_api_decls on purpose; the
# coverage gate needs names, not link symbols.
list_callable_names() {
  grep -ho 'zjolt[A-Za-z0-9_]*(' ffi/*.h | grep -o 'zjolt[A-Za-z0-9_]*' | sort -u
}

# The Zig mirror of the exported set. src/abi_check.zig fails the build when
# the two disagree, so a difference here is a bug in the counting.
count_externs() {
  cat src/c/*.zig | grep -c '^pub extern fn zjolt'
}

# Public means reachable through the umbrella header. zjolt_internal.h and
# zjolt_query_internal.h are neither included by it nor installed.
count_public_headers() {
  grep -c '^#include "zjolt_' ffi/zjolt.h
}

count_returns_result() {
  cat ffi/zjolt_*.h | grep -c '^ZJOLT_API ZJoltResult'
}

count_returns_void() {
  cat ffi/zjolt_*.h | grep -c '^ZJOLT_API void'
}

# ffi/zjolt_core.h is the version's one home: a Zig test holds build.zig.zon
# against it, and src/c/core.zig mirrors it under abi_check.
version_triple() {
  sed -nE 's/^#define ZJOLT_VERSION_(MAJOR|MINOR|PATCH) ([0-9]+)$/\2/p' \
    ffi/zjolt_core.h | tr '\n' ' ' | sed 's/ $//'
}

# Per-subsystem export counts, for the areas a document breaks out by name.
count_api_decls_in() {
  grep -c '^ZJOLT_API' "$1"
}

# The C declaration modules. src/c.zig lists them and both reflective guards
# walk that list, so this is also how many modules those guards cover.
count_c_modules() {
  ls src/c/*.zig | wc -l | tr -d ' '
}

# Build options that change type LAYOUT, counted from the one place that
# decides it: the bits ZJOLT_CONFIG_ID folds. An option added to the header's
# handshake without a table row, or a row marked as ABI-changing with no bit
# behind it, is the disagreement this is here to expose.
count_abi_config_bits() {
  sed -n '/#define ZJOLT_CONFIG_ID/,/^$/p' ffi/zjolt_core.h |
    grep -c 'ZJOLT_CONFIG_BIT_'
}

count_abi_table_rows() {
  grep -c '^| `-D.*Changes the ABI\.' README.md
}

#===----------------------------------------------------------------------===//
# The guards
#===----------------------------------------------------------------------===//

# One `try` per ABI-drift mutation, one `expect` per other-guard mutation.
count_drift_mutations() {
  grep -c '^try ' ci/check-abi-drift.sh
}

count_other_mutations() {
  grep -c '^expect ' ci/check-abi-drift.sh
}

count_all_mutations() {
  echo $(( $(count_drift_mutations) + $(count_other_mutations) ))
}

# Sweeps that discover their subject by reflection rather than a written list.
count_reflective_sweeps() {
  grep -c '^test "every entry point' src/misuse_sweep_test.zig
}

#===----------------------------------------------------------------------===//
# What a full local run executes
#===----------------------------------------------------------------------===//

# Every `run` line, with the ones inside a loop replaced by what the loop
# expands to. A check that runs once per optimize mode is counted once per
# mode, because that is what a reader timing the run experiences.
count_host_checks() {
  local static_runs loop_runs modes
  static_runs=$(grep -cE '^ *run ' ci/run.sh)
  loop_runs=$(grep -cE '^ *run .*[$](mode|target)' ci/run.sh)
  modes=$(sed -n 's/^ *for mode in \(.*\); do$/\1/p' ci/run.sh | wc -w | tr -d ' ')
  echo $(( static_runs - loop_runs + modes ))
}

count_cross_targets() {
  sed -n '/^ *for target in/,/^ *do$/p' ci/run.sh | grep -cE '^ +[a-z0-9_]+-'
}

count_full_checks() {
  echo $(( $(count_host_checks) + $(count_cross_targets) ))
}

#===----------------------------------------------------------------------===//
# Build size
#===----------------------------------------------------------------------===//

count_jolt_tu() {
  find libs/JoltPhysics/Jolt -name '*.cpp' | wc -l | tr -d ' '
}

count_own_tu() {
  ls ffi/*.cpp | wc -l | tr -d ' '
}

count_total_tu() {
  echo $(( $(count_jolt_tu) + $(count_own_tu) ))
}
