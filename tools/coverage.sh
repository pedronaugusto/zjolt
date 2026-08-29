#!/usr/bin/env bash
#
# zjolt — which public Jolt names in the claimed areas no entry point spells
# out. A work list, not a score: name matching is naive on purpose, and
# ci/check-coverage.sh is what holds every name it prints to a verdict.
#
#   tools/coverage.sh                 # summary per area
#   tools/coverage.sh Constraints     # and the unbound names in that area
#   tools/coverage.sh --names         # AREA<TAB>NAME, the checker's input
#   tools/coverage.sh --areas         # the claimed directory list

set -uo pipefail
cd "$(dirname "$0")/.."

JOLT=libs/JoltPhysics/Jolt

# Every directory Jolt has, and the four headers at its root. `NAME:DEPTH`
# limits the walk; only `.` and `Physics` need it, because their subdirectories
# are claimed as areas of their own. ci/check-coverage.sh fails if a directory
# under libs/JoltPhysics/Jolt is not covered by this list.
AREAS=".:1 Physics:1 Physics/Body Physics/Collision Physics/Constraints
       Physics/Character Physics/Ragdoll Physics/Vehicle Physics/SoftBody
       Physics/Hair Skeleton Core Geometry Math Renderer ObjectStream
       AABBTree TriangleSplitter Compute Shaders"

[ "${1:-}" = "--areas" ] && { printf '%s\n' $AREAS; exit 0; }

FILTER="${1:-}"
NAMES=0
[ "$FILTER" = "--names" ] && { NAMES=1; FILTER=; }

if [ -t 1 ] && [ "$NAMES" -eq 0 ]
then B=$'\033[1m'; D=$'\033[2m'; G=$'\033[32m'; Y=$'\033[33m'; O=$'\033[0m'
else B=; D=; G=; Y=; O=; fi

# Every name zjolt exports. Matched as name-followed-by-paren rather than by
# anchoring on ZJOLT_API through the return type: a character class excluding
# digits hid every entry point returning uint32_t and put 66 of them on the
# unbound list as false gaps. Casing is kept, because the matcher below splits
# at camelCase boundaries.
bound=$(grep -ho 'zjolt[A-Za-z0-9_]*(' ffi/*.h |
        grep -o 'zjolt[A-Za-z0-9_]*' | sort -u)

# Public names Jolt declares in one directory: methods, and — unlike a
# class-only API — free functions declared straight in a namespace, which is
# what RegisterTypes, EstimateCollisionResponse and most of Geometry are.
#
# Takes a maxdepth so `Physics` can claim the headers sitting directly in it
# without re-enumerating the subdirectories that are claimed separately.
jolt_methods() {
  find "$JOLT/$1" -maxdepth "${2:-99}" -name '*.h' 2>/dev/null -print0 | xargs -0 awk '
    # A new file starts public: namespace scope has no access specifier, so a
    # free function is public by default. Resetting per file rather than
    # carrying state across the xargs batch matters — one header ending inside
    # a private: section used to hide the whole of the next one.
    FNR == 1 { pub = 1 }
    /^[[:space:]]*(private|protected):/ { pub = 0; next }
    /^[[:space:]]*public:/              { pub = 1; next }
    /^[[:space:]]*(class|struct)[[:space:]]/ { pub = 1 }
    !pub { next }
    # Jolt documents its solver derivations in /* */ blocks full of prose like
    # "Jacobian (transposed) (eq 55):", and a word followed by a paren in one
    # of those reads exactly like a declaration. But a real declaration can
    # also carry a comment on its own line — every default listener method
    # ends `{ /* Do nothing */ }` — so a line cannot simply be skipped for
    # containing one. Strip the comment spans and keep what is left.
    {
      line = $0
      if (in_comment) {
        p = index(line, "*/")
        if (p == 0) next
        line = substr(line, p + 2); in_comment = 0
      }
      while ((a = index(line, "/*")) > 0) {
        rest = substr(line, a + 2)
        b = index(rest, "*/")
        if (b == 0) { line = substr(line, 1, a - 1); in_comment = 1; break }
        line = substr(line, 1, a - 1) substr(rest, b + 2)
      }
      $0 = line
    }
    /JPH_ASSERT|^[[:space:]]*\/\// { next }
    match($0, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(/) {
      # Skip `obj.name(` / `obj->name(` / `ns::name(` — a call on something
      # else, not a declaration belonging to this area.
      pre = (RSTART > 1) ? substr($0, RSTART - 1, 1) : ""
      if (pre == "." || pre == ">" || pre == ":") next
      name = substr($0, RSTART, RLENGTH - 1)
      gsub(/[[:space:]()]/, "", name)
      if (name ~ /^(if|for|while|switch|return|sizeof|alignof|static_assert|do|else|catch|new|delete|defined)$/) next
      # Operators, macros and Jolt-internal declarations are not candidates to
      # bind: an operator has no name a C ABI can carry, and the JPH_ macros
      # expand to RTTI and allocator plumbing.
      if (name ~ /^(operator|JPH_)/) next
      # A name that is all caps is a macro invocation, not a method.
      if (name ~ /^[A-Z0-9_]+$/) next
      if (name !~ /^[A-Z]/) next
      if (length(name) < 4) next
      print name
    }
  ' 2>/dev/null | sort -u
}

# Match a batch of upstream names against `bound` in one awk pass.
#
# Both sides are split into words at camelCase boundaries and underscores, and
# an upstream name matches when its words appear as an ordered subsequence of
# an entry point's. Whole words and in order, both learned the hard way: order
# alone let `AddBody` match on the shared keyword `body` across 188 entry
# points, and substring matching let `Tan` "match" inside `TestAnimation`. A
# false BOUND is the dangerous direction — nobody is ever asked about it.
#
# The cost is that a deliberate rename reads as unbound. That is the right
# trade, because tools/unbound_*.txt has to answer for every name printed here.
match_names() {
  awk '
    function split_words(m, out,   i, c, p, n) {
      s = ""
      for (i = 1; i <= length(m); i++) {
        c = substr(m, i, 1)
        p = (i > 1) ? substr(m, i - 1, 1) : ""
        if (c == "_") { s = s " "; continue }
        if (c ~ /[A-Z]/ && p ~ /[a-z0-9]/) s = s " "
        s = s c
      }
      return split(tolower(s), out, / +/)
    }
    NR == FNR { if (NF) { nb++; bn[nb] = split_words($0, bw); for (i = 1; i <= bn[nb]; i++) b[nb, i] = bw[i] } ; next }
    !NF { next }
    {
      nw = split_words($0, w)
      hit = 0
      for (j = 1; j <= nb && !hit; j++) {
        k = 1
        for (i = 1; i <= bn[j] && k <= nw; i++) if (b[j, i] == w[k]) k++
        if (k > nw) hit = 1
      }
      if (!hit) print
    }
  ' "$1" -
}

boundfile=$(mktemp); trap 'rm -f "$boundfile"' EXIT
printf '%s\n' "$bound" > "$boundfile"

if [ "$NAMES" -eq 0 ]; then
  printf '%szjolt coverage of Jolt%s  %s(by name; a work list, not a score)%s\n\n' \
    "$B" "$O" "$D" "$O"
  printf '  %-34s %8s %8s %6s\n' "area" "bound" "public" "cover"
fi

total_b=0; total_p=0
for spec in $AREAS
do
  area=${spec%%:*}
  depth=${spec#*:}; [ "$depth" = "$area" ] && depth=99
  [ -d "$JOLT/$area" ] || continue
  methods=$(jolt_methods "$area" "$depth")
  [ -z "$methods" ] && continue

  unbound=$(printf '%s\n' "$methods" | match_names "$boundfile")
  p=$(printf '%s\n' "$methods" | grep -c .)
  u=$(printf '%s\n' "$unbound" | grep -c .)
  b=$((p - u))

  label=$area; [ "$label" = "." ] && label="Jolt (root headers)"
  if [ "$NAMES" -eq 1 ]; then
    printf '%s\n' "$unbound" | grep . | sed "s|^|$label	|"
  else
    pct=$(( p == 0 ? 0 : b * 100 / p ))
    colour=$Y; [ $pct -ge 70 ] && colour=$G
    printf '  %-34s %8d %8d %s%5d%%%s\n' "$label" "$b" "$p" "$colour" "$pct" "$O"
    if [ -n "$FILTER" ] && printf '%s' "$label" | grep -qi "$FILTER"; then
      printf '%s\n' "$unbound" | grep . | sed 's/^/      /' | head -60
      [ "$u" -gt 60 ] && printf '      %s... %d more%s\n' "$D" "$((u - 60))" "$O"
    fi
  fi
  total_b=$((total_b + b)); total_p=$((total_p + p))
done

[ "$NAMES" -eq 1 ] && exit 0

printf '\n  %-34s %8d %8d %5d%%\n' "TOTAL" "$total_b" "$total_p" \
  "$(( total_p == 0 ? 0 : total_b * 100 / total_p ))"
printf '\n  %sentry points exported: %s%s\n' "$D" \
  "$(printf '%s\n' "$bound" | grep -c .)" "$O"
printf '  %sname matching is naive; %s says what each unbound name really is%s\n' \
  "$D" "ci/check-coverage.sh" "$O"
printf '  %spass an area name to list what is unbound there%s\n' "$D" "$O"
