#!/usr/bin/env bash
#
# Every header reachable by `#include` from an installed header has to be
# installed too, or a C consumer gets an umbrella that does not resolve.
# `tests/consumer` proves the same thing by compiling against the install
# tree, but only under `--full` and only after a full build.
#
# Reachability, not `ffi/*.h`: `ffi/zjolt_internal.h` and its neighbours are
# deliberately not installed, and nothing public includes them. And what
# counts as installed is the NAME build.zig gives `installHeader`, not a path
# under `ffi/`: `zjolt_config.h` is generated from the build options and has
# no source file, so comparing against paths reports it missing on a correct
# tree. Two spellings reach `installHeader` -- a literal name, and the
# basename of each `public_headers` entry -- and the call count below refuses
# to guess if a third ever appears.
#
#   ci/check-headers.sh

set -uo pipefail
cd "$(dirname "$0")/.."

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; OFF=$'\033[0m'
else RED=; GREEN=; OFF=; fi

die() { printf '%s%s%s\n' "$RED" "$1" "$OFF" >&2; exit 1; }
includes() { grep -hoE '#include "zjolt[a-z_]*\.h"' "$1" | sed 's/#include "//; s/"//'; }

literals=$(grep -oE 'installHeader\([^;]*"zjolt[a-z_]*\.h"' build.zig |
             grep -oE 'zjolt[a-z_]*\.h' | sort -u)
calls=$(grep -c 'installHeader(' build.zig)
n_literals=$(printf '%s\n' "$literals" | grep -c .)
[ "$calls" -eq $((n_literals + 1)) ] || die \
  "build.zig has $calls installHeader call(s), $n_literals of them naming a header outright; this check reads those plus one loop over public_headers"

listed=$(sed -n '/public_headers = /,/};/p' build.zig | grep -oE 'zjolt[a-z_]*\.h' | sort -u)
[ -n "$listed" ] || die 'build.zig has no public_headers list this check can read'
installed=$(printf '%s\n%s\n' "$literals" "$listed" | grep . | sort -u)

# Everything an installed header can reach, following includes until the set
# stops growing. A generated header has no file here and ends its branch.
reached=$(printf '%s\n' "$listed")
while :; do
  next=$(for h in $reached; do
           printf '%s\n' "$h"
           [ -f "ffi/$h" ] && includes "ffi/$h"
         done | sort -u)
  [ "$next" = "$reached" ] && break
  reached=$next
done

missing=$(comm -23 <(printf '%s\n' "$reached") <(printf '%s\n' "$installed"))
if [ -n "$missing" ]; then
  printf '%s\n' "$missing" | sed 's/^/  /' >&2
  die 'reachable from an installed header, and not installed'
fi

printf '%sOK%s  %d installed header(s), and every include they reach is one of them\n' \
  "$GREEN" "$OFF" "$(printf '%s\n' "$installed" | grep -c .)"
