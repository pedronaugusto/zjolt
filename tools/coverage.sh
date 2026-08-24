#!/usr/bin/env bash
#
# zjolt — what of Jolt is bound, and what is not.
#
# "How much of the library do we cover" was being answered by estimate, which
# is worth nothing when the answer decides what to work on next. This answers
# it by enumeration: every public method Jolt declares in the areas this ABI
# claims, matched against every entry point zjolt exports, with the unmatched
# ones listed by name.
#
# The matching is by NAME, deliberately naively: a Jolt method `SetPosition` is
# considered bound if some `zjolt*SetPosition*` exists. That over-counts (an
# entry point may bind an unrelated method of the same name) and under-counts
# (a deliberate rename hides a real binding). Neither matters for what this is
# for, which is producing a work list rather than a score. Every line it prints
# is a name you can grep for.
#
# Not everything unbound should be bound. Jolt has methods that exist for its
# own internals, for its debug renderer, and for serialisation this ABI routes
# differently. Read the list; do not work it blindly.
#
# Usage:
#   tools/coverage.sh                 # summary per area
#   tools/coverage.sh Constraints     # and the unbound names in that area

set -uo pipefail
cd "$(dirname "$0")/.."

JOLT=libs/JoltPhysics/Jolt
FILTER="${1:-}"

if [ -t 1 ]; then B=$'\033[1m'; D=$'\033[2m'; G=$'\033[32m'; Y=$'\033[33m'; O=$'\033[0m'
else B=; D=; G=; Y=; O=; fi

# Every name zjolt exports, lowercased, one per line.
bound=$(grep -ho 'ZJOLT_API [A-Za-z_ *]*zjolt[A-Za-z0-9_]*' ffi/*.h |
        grep -o 'zjolt[A-Za-z0-9_]*' | tr 'A-Z' 'a-z' | sort -u)

# Public method names Jolt declares in one directory.
jolt_methods() {
  find "$JOLT/$1" -name '*.h' 2>/dev/null -print0 | xargs -0 awk '
    /^[[:space:]]*(private|protected):/ { pub = 0; next }
    /^[[:space:]]*public:/              { pub = 1; next }
    /^[[:space:]]*(class|struct)[[:space:]]/ { pub = 1 }
    !pub { next }
    /JPH_ASSERT|^[[:space:]]*\/\// { next }
    match($0, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(/) {
      name = substr($0, RSTART, RLENGTH - 1)
      gsub(/[[:space:]()]/, "", name)
      if (name ~ /^(if|for|while|switch|return|sizeof|alignof|static_assert|do|else|catch)$/) next
      # Operators, macros and Jolt-internal declarations are not candidates to
      # bind: an operator has no name a C ABI can carry, and the JPH_ macros
      # expand to RTTI and allocator plumbing.
      if (name ~ /^(operator|JPH_|OZZ_)/) next
      # A name that is all caps is a macro invocation, not a method.
      if (name ~ /^[A-Z0-9_]+$/) next
      if (name !~ /^[A-Z]/) next
      if (length(name) < 4) next
      print name
    }
  ' 2>/dev/null | sort -u
}

printf '%szjolt coverage of Jolt%s  %s(by name; a work list, not a score)%s\n\n' \
  "$B" "$O" "$D" "$O"
printf '  %-34s %8s %8s %6s\n' "area" "bound" "public" "cover"

total_b=0; total_p=0
# Math, Core and Geometry are deliberately absent. They are Jolt's internal
# value and container types — Vec3, Mat44, Array, the allocator — and none of
# them crosses a C boundary: zjolt declares its own flat PODs for the few that
# a caller needs. Scoring them would put a permanent, meaningless 1% in the
# total and hide the areas that matter.
for area in Physics/Body Physics/Collision Physics/Constraints Physics/Character \
            Physics/Ragdoll Physics/Vehicle Physics/SoftBody Physics/Hair \
            Skeleton
do
  [ -d "$JOLT/$area" ] || continue
  methods=$(jolt_methods "$area")
  [ -z "$methods" ] && continue

  p=0; b=0; unbound=""
  while IFS= read -r m; do
    [ -z "$m" ] && continue
    p=$((p + 1))
    # Compare by WORDS, not by the flattened name. Flattening is the obvious
    # thing and it is WRONG on word order: Jolt's `ActivateBodiesInAABox`
    # flattens to `activatebodiesinaabox`, which does not appear in
    # `zjoltbodyactivateinaabox` even when that is exactly the binding. The
    # same bug in the zozz copy of this script reported thirteen already-bound
    # names as gaps, which is work someone would have done twice.
    #
    # Every word must appear somewhere in a single bound name, in any order.
    # Words shorter than four characters are dropped — `get`, `set`, `add` and
    # `id` appear in nearly every name and would match anything. If nothing
    # survives that filter the flattened name is used, which is right for a
    # short name like `Activate`.
    words=$(printf '%s' "$m" |
            sed 's/\([a-z0-9]\)\([A-Z]\)/\1 \2/g' |
            tr 'A-Z' 'a-z' | tr ' ' '\n' | awk 'length($0) >= 4')
    [ -z "$words" ] && words=$(printf '%s' "$m" | tr 'A-Z' 'a-z')

    hit=0
    while IFS= read -r cand; do
      [ -z "$cand" ] && continue
      all=1
      while IFS= read -r w; do
        [ -z "$w" ] && continue
        case "$cand" in *"$w"*) ;; *) all=0; break ;; esac
      done <<< "$words"
      if [ "$all" -eq 1 ]; then hit=1; break; fi
    done <<< "$bound"

    if [ "$hit" -eq 1 ]; then
      b=$((b + 1))
    else
      unbound="$unbound$m"$'\n'
    fi
  done <<< "$methods"

  pct=$(( p == 0 ? 0 : b * 100 / p ))
  colour=$Y; [ $pct -ge 70 ] && colour=$G
  printf '  %-34s %8d %8d %s%5d%%%s\n' "$area" "$b" "$p" "$colour" "$pct" "$O"
  total_b=$((total_b + b)); total_p=$((total_p + p))

  if [ -n "$FILTER" ] && printf '%s' "$area" | grep -qi "$FILTER"; then
    printf '%s' "$unbound" | grep -v '^$' | sed 's/^/      /' | head -60
    n=$(printf '%s' "$unbound" | grep -vc '^$')
    [ "$n" -gt 60 ] && printf '      %s... %d more%s\n' "$D" "$((n - 60))" "$O"
  fi
done

printf '\n  %-34s %8d %8d %5d%%\n' "TOTAL" "$total_b" "$total_p" \
  "$(( total_p == 0 ? 0 : total_b * 100 / total_p ))"
printf '\n  %sentry points exported: %s%s\n' "$D" \
  "$(printf '%s\n' "$bound" | grep -c .)" "$O"
printf '  %spass an area name to list what is unbound there%s\n' "$D" "$O"
