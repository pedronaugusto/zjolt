#!/usr/bin/env bash
#
# zjolt — nothing this repository ignores is tracked by it.
#
# `.gitignore` names what does not belong in the history: build output, the
# packages `zig fetch` drops beside the project, object files carrying absolute
# paths from the machine that produced them, an editor's leftovers. Tracking
# overrides those rules, so a path added before its pattern existed — or added
# past it, or swept in by a blanket `git add -A` — sits in every clone forever
# and nothing says a word.
#
# That is not hypothetical. On 2026-09-03 a `zig fetch` had left 333 files
# under ./zig-pkg in a sibling package, `git add -A` committed them, and they
# reached a public main. What surfaced it was the executable-bit gate, which
# enumerates TRACKED .sh files: the fetched CI scripts appeared as violations
# of an unrelated rule.
#
# The patterns are not restated here. `git check-ignore` is asked, so
# `.gitignore` stays the one home and a rule added there is a gate for free.
#
#   ci/check-ignored.sh
#
# Exits non-zero if any tracked path matches an ignore rule.

set -uo pipefail
cd "$(dirname "$0")/.."

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; OFF=$'\033[0m'
else RED=; GREEN=; OFF=; fi

# `--no-index` is what makes this work: git reports a tracked file as NOT
# ignored, because tracking overrides the rules. The flag asks the rules alone.
ignored=$(git ls-files -c | git check-ignore --no-index --stdin || true)

if [ -n "$ignored" ]; then
  printf '%sthese tracked paths match an ignore rule:%s\n' "$RED" "$OFF" >&2
  printf '%s\n' "$ignored" | sed 's/^/  /' >&2
  printf 'Take it out of the index (git rm -r --cached <path>) — or, if it does\n' >&2
  printf 'belong in the history, drop the .gitignore rule that excludes it. Do not\n' >&2
  printf 'force a path past its own rule.\n' >&2
  exit 1
fi

printf '%sOK%s  nothing this repository ignores is tracked\n' "$GREEN" "$OFF"
