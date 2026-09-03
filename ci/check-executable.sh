#!/usr/bin/env bash
#
# zjolt — every checked-in script is executable.
#
# CI runs these by path (`- run: ci/run.sh`), so a script committed without the
# executable bit fails on the runner with "permission denied" and passes on the
# machine that wrote it, where bash was invoked explicitly. The bit lives in
# the index, and on Windows nothing sets it by default:
# `git update-index --chmod=+x <path>` is the fix.

set -euo pipefail
cd "$(dirname "$0")/.."

not_executable=$(git ls-files -s -- '*.sh' | awk '$1 != "100755" { print $4 }')

if [ -n "$not_executable" ]; then
  printf 'these scripts are committed without the executable bit:\n'
  printf '%s\n' "$not_executable" | sed 's/^/  /'
  printf 'fix with: git update-index --chmod=+x <path>\n'
  exit 1
fi

printf 'OK  every committed script is executable\n'
