#!/usr/bin/env bash
#
# zjolt — every public Jolt name in the areas this ABI claims has a verdict,
# and every verdict is checkable.
#
#   tools/coverage.sh --names   lists the names no entry point spells out.
#   tools/verdicts_*.txt        gives each one AREA<TAB>NAME<TAB>VERDICT<TAB>EVIDENCE,
#                               keyed by area so a name that is bound in one
#                               place cannot vouch for it in another.
#   tools/classify.sh           computes which exclusions upstream justifies.
#   tools/zig_native.txt        entry points Zig does not call because it
#                               computes the same answer, each with the
#                               declaration that does and the test that proves
#                               the two agree.
#
# INTERNAL is not something this file takes on trust: it is recomputed here and
# rejected unless classify.sh proves it. GAP fails the build.

set -uo pipefail
cd "$(dirname "$0")/.."

# The counts this script reports about the ABI itself come from one home,
# shared with ci/measurements.sh and ci/check-numbers.sh.
. ci/counts.sh

LIST=0
[ "${1:-}" = "--list" ] && LIST=1

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
else RED=; GREEN=; DIM=; BOLD=; OFF=; fi

work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
fails=0
fail() { printf '%s%s%s\n' "$RED" "$1" "$OFF" >&2; fails=$((fails + 1)); }

# Every identifier ffi/*.h declares or mentions — types and enum constants as
# well as functions. A lowercase token carrying an underscore is a header
# filename, not a symbol.
grep -hoE '\b(zjolt|ZJolt|ZJOLT_)[A-Za-z0-9_]*' ffi/*.h |
  grep -vE '^zjolt[a-z0-9]*_' | sort -u > "$work/syms"

tools/coverage.sh --names | sort -u > "$work/unbound"
tools/classify.sh | sort -t$'\t' -k1,1 > "$work/provable"

#-----------------------------------------------------------------------------
# Claimed areas. A directory missing from tools/coverage.sh is a place a
# capability can hide where nothing here would ever look, so a re-vendor that
# adds one fails until the list catches up.
#-----------------------------------------------------------------------------
find libs/JoltPhysics/Jolt -mindepth 1 -type d |
  sed 's|libs/JoltPhysics/Jolt/||' | sort -u > "$work/dirs"
# A directory is claimed when it is listed, or when an ancestor is listed with
# no depth limit.
tools/coverage.sh --areas > "$work/specs"
: > "$work/claimed"
while read -r d; do
  ok=0
  while read -r spec; do
    a=${spec%%:*}; depth=${spec#*:}; [ "$depth" = "$a" ] && depth=99
    [ "$a" = "." ] && continue
    if [ "$d" = "$a" ] || { [ "$depth" = 99 ] && case "$d" in "$a"/*) true;; *) false;; esac; }; then
      ok=1; break
    fi
  done < "$work/specs"
  [ "$ok" -eq 1 ] && printf '%s\n' "$d" >> "$work/claimed"
done < "$work/dirs"
sort -u -o "$work/claimed" "$work/claimed"
comm -23 "$work/dirs" "$work/claimed" > "$work/unclaimed"
if [ -s "$work/unclaimed" ]; then
  sed 's/^/  /' "$work/unclaimed" >&2
  fail "$(grep -c . "$work/unclaimed") Jolt directory(ies) missing from tools/coverage.sh"
fi

#-----------------------------------------------------------------------------
# Shape.
#-----------------------------------------------------------------------------
for f in tools/verdicts_*.txt; do
  awk -F'\t' -v F="$f" '
    /^#/ || !NF { next }
    NF != 4 { printf "%s:%d: not four tab-separated fields\n", F, FNR > "/dev/stderr"; next }
    $3 !~ /^(BOUND|EXTENSION|LANGUAGE|ZIG|INTERNAL|GAP)$/ {
      printf "%s:%d: unknown verdict %s\n", F, FNR, $3 > "/dev/stderr"; next }
    $3 != "GAP" && length($4) < 8 {
      printf "%s:%d: %s has no evidence\n", F, FNR, $2 > "/dev/stderr"; next }
    { print $1 "\t" $2 "\t" $3 "\t" $4 }
  ' "$f" 2>>"$work/shape" >> "$work/rows"
done
if [ -s "$work/shape" ]; then cat "$work/shape" >&2; fail "$(grep -c . "$work/shape") malformed line(s)"; fi

#-----------------------------------------------------------------------------
# Completeness.
#-----------------------------------------------------------------------------
cut -f1,2 "$work/rows" | sort -u > "$work/classified"
comm -23 "$work/unbound" "$work/classified" > "$work/missing"
if [ -s "$work/missing" ]; then
  sed 's/^/  /' "$work/missing" >&2
  fail "$(grep -c . "$work/missing") area/name pair(s) with no verdict"
fi
comm -13 "$work/unbound" "$work/classified" > "$work/stale"
if [ -s "$work/stale" ]; then
  sed 's/^/  /' "$work/stale" >&2
  fail "$(grep -c . "$work/stale") stale line(s) — an entry point now spells these out; delete them"
fi

#-----------------------------------------------------------------------------
# Evidence, one rule per verdict.
#-----------------------------------------------------------------------------
cat > "$work/facilities" <<'EOF'
@Vector
@shuffle
@reduce
@splat
@select
@bitCast
@ptrCast
@floatCast
@intCast
@as
@clz
@ctz
@popCount
@min
@max
@abs
@sqrt
@mulAdd
@byteSwap
@atomicRmw
@prefetch
std.math
std.mem
std.sort
std.ArrayList
std.HashMap
std.AutoHashMap
std.PriorityQueue
std.hash
std.Thread
std.heap
std.fmt
std.ascii
EOF

awk -F'\t' '
  FILENAME ~ /syms$/       { sym[$0] = 1; syms[++ns] = $0; next }
  FILENAME ~ /facilities$/ { fac[++nf] = $0; next }
  FILENAME ~ /provable$/   { prov[$1] = $3; next }
  { name = $2; verdict = $3; evidence = $4 }

  verdict == "BOUND" || verdict == "EXTENSION" {
    n = 0; s = evidence
    while (match(s, /(zjolt|ZJolt|ZJOLT_)[A-Za-z0-9_]*/)) {
      t = substr(s, RSTART, RLENGTH); s = substr(s, RSTART + RLENGTH)
      if (t ~ /^zjolt[a-z0-9]*_/) continue
      n++
      if (substr(s, 1, 1) == "*") {
        s = substr(s, 2); ok = 0
        for (i = 1; i <= ns && !ok; i++) if (index(syms[i], t) == 1) ok = 1
        if (!ok) printf "  %s: nothing in ffi/*.h starts with %s\n", name, t > "/dev/stderr"
        continue
      }
      if (!(t in sym)) printf "  %s: %s is not in ffi/*.h\n", name, t > "/dev/stderr"
    }
    if (n == 0) printf "  %s: %s names no zjolt symbol\n", name, verdict > "/dev/stderr"
    next
  }

  # LANGUAGE names a Zig facility from a closed list; every @builtin or std.
  # token it uses has to be on that list, so it cannot become a dumping ground.
  verdict == "LANGUAGE" {
    hit = 0; s = evidence
    while (match(s, /(@[A-Za-z]+|std\.[A-Za-z.]+)/)) {
      t = substr(s, RSTART, RLENGTH); s = substr(s, RSTART + RLENGTH)
      ok = 0
      for (i = 1; i <= nf; i++) if (index(t, fac[i]) == 1) { ok = 1; break }
      if (!ok) printf "  %s: %s is not a facility this file recognises\n", name, t > "/dev/stderr"
      else hit = 1
    }
    if (!hit && evidence !~ /Zig slices|defer/)
      printf "  %s: LANGUAGE names no facility\n", name > "/dev/stderr"
    next
  }

  # ZIG points at a declaration in src/, checked below.
  verdict == "ZIG" {
    if (evidence !~ /^src\/([A-Za-z0-9_]+\/)?[A-Za-z0-9_]+\.zig:([A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?|@"[A-Za-z_][A-Za-z0-9_]*")/)
      printf "  %s: ZIG evidence is not src/FILE.zig:decl\n", name > "/dev/stderr"
    else print name "\t" evidence
    next
  }

  # INTERNAL is recomputed, never taken on trust.
  verdict == "INTERNAL" {
    if (!(name in prov))
      printf "  %s: INTERNAL, but tools/classify.sh cannot justify it\n", name > "/dev/stderr"
    else if (evidence != prov[name])
      printf "  %s: INTERNAL evidence does not match what classify.sh computes\n", name > "/dev/stderr"
    next
  }
' "$work/syms" "$work/facilities" "$work/provable" "$work/rows" \
  2>"$work/evidence" > "$work/zigrefs"
if [ -s "$work/evidence" ]; then
  cat "$work/evidence" >&2
  fail "$(grep -c . "$work/evidence") unusable piece(s) of evidence"
fi

awk -F'\t' '/^#/ || !NF { next }
  NF != 3 { printf "  %s: not NAME<TAB>src/FILE.zig:decl<TAB>test name\n", $1 > "/dev/stderr"; next }
  $2 !~ /^src\/([A-Za-z0-9_]+\/)?[A-Za-z0-9_]+\.zig:[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?$/ {
    printf "  %s: %s is not src/FILE.zig:decl\n", $1, $2 > "/dev/stderr"; next }
  length($3) < 10 { printf "  %s: names no test\n", $1 > "/dev/stderr"; next }
  { print $1 "\t" $2 "\t" $3 }' tools/zig_native.txt 2>"$work/native_shape" > "$work/native_rows"
if [ -s "$work/native_shape" ]; then
  cat "$work/native_shape" >&2
  fail "$(grep -c . "$work/native_shape") malformed native line(s)"
fi
cut -f1 "$work/native_rows" | sort -u > "$work/native"
cut -f1,2 "$work/native_rows" >> "$work/zigrefs"

# The test a native line names has to exist, or the comparison it claims is
# being made is not being made by anything.
: > "$work/native_test_miss"
while IFS=$'\t' read -r name ref test_name; do
  grep -qrF "test \"$test_name\"" src --include='*_test.zig' ||
    printf '  %s: no test named "%s"\n' "$name" "$test_name" >> "$work/native_test_miss"
done < "$work/native_rows"
if [ -s "$work/native_test_miss" ]; then
  cat "$work/native_test_miss" >&2
  fail "$(grep -c . "$work/native_test_miss") native line(s) naming no test"
fi

while IFS=$'\t' read -r name ref; do
  file=${ref%%:*}; decl=${ref#*:}
  [ -f "$file" ] || { printf '  %s: %s does not exist\n' "$name" "$file" >&2; continue; }
  case "$decl" in
    *.*)
      # Type.method — the method must sit inside that type's own block.
      type=${decl%%.*}; member=${decl#*.}
      awk -v T="$type" -v M="$member" '
        $0 ~ "^pub const " T " = (extern |packed )?(struct|union|enum)" { inb = 1; depth = 0 }
        inb {
          depth += gsub(/{/, "{") - gsub(/}/, "}")
          if ($0 ~ "^[[:space:]]*pub (fn|const|inline fn) " M "[ (:=]") { found = 1 }
          if (depth <= 0 && NR > 1 && seen) inb = 0
          seen = 1
        }
        END { exit found ? 0 : 1 }' "$file" ||
        printf '  %s: %s declares no %s inside %s\n' "$name" "$file" "$member" "$type" >&2
      ;;
    *)
      grep -qF "pub fn $decl(" "$file" ||
        grep -qE "^[[:space:]]*pub (fn|const|inline fn) $decl\b" "$file" ||
        printf '  %s: %s declares no %s\n' "$name" "$file" "$decl" >&2
      ;;
  esac
done < "$work/zigrefs" 2>&1 >/dev/null | tee "$work/zigmiss" >&2
[ -s "$work/zigmiss" ] && fail "$(grep -c . "$work/zigmiss") ZIG line(s) pointing at nothing"

#-----------------------------------------------------------------------------
# The Zig surface. An entry point declared in C and never wrapped is
# unreachable for a Zig host, and nothing above can see it. src/c/ and c.zig
# are the extern layer; tests do not count as use.
#-----------------------------------------------------------------------------
list_callable_names > "$work/entrypoints"
# src/c/ holds the extern declarations, and a declaration is not use — but it
# now also holds the methods declared on the ABI structs themselves, and a
# method body calling an entry point IS use. Count every line in src/c/ except
# the extern declarations.
#
# Comments are stripped first. A doc comment that NAMES an entry point used to
# count as calling it, so prose could satisfy the rule below — the same defect
# tools/coverage.sh strips comments to avoid, found the day this file learned
# to tell a native implementation from a wrapper.
{
  find src -name '*.zig' ! -path 'src/c/*' ! -name 'c.zig' \
       ! -name '*_test.zig' ! -name '*_sweep*.zig' -print0 |
    xargs -0 sed -E 's://.*::' |
    grep -o 'zjolt[A-Za-z0-9_]*'
  find src/c -name '*.zig' -print0 2>/dev/null |
    xargs -0 grep -h -v '^[[:space:]]*pub extern fn' 2>/dev/null |
    sed -E 's://.*::' | grep -o 'zjolt[A-Za-z0-9_]*'
} | sort -u > "$work/wrapped"
awk -F'\t' '/^#/ || !NF { next }
  NF != 2 { printf "  %s: not NAME<TAB>reason\n", $1 > "/dev/stderr"; next }
  length($2) < 10 { printf "  %s: no reason given\n", $1 > "/dev/stderr"; next }
  { print $1 }' tools/zig_surface_exceptions.txt 2>"$work/exc_shape" | sort -u > "$work/excused"
if [ -s "$work/exc_shape" ]; then cat "$work/exc_shape" >&2; fail "$(grep -c . "$work/exc_shape") malformed exception line(s)"; fi

comm -23 "$work/entrypoints" "$work/wrapped" > "$work/unwrapped"
comm -23 "$work/unwrapped" "$work/excused" > "$work/unexcused"
comm -23 "$work/unexcused" "$work/native" > "$work/stranded"
if [ -s "$work/stranded" ]; then
  sed 's/^/  /' "$work/stranded" >&2
  fail "$(grep -c . "$work/stranded") entry point(s) with no Zig caller"
fi
comm -13 "$work/unwrapped" "$work/excused" > "$work/excess"
if [ -s "$work/excess" ]; then
  sed 's/^/  /' "$work/excess" >&2
  fail "$(grep -c . "$work/excess") excused entry point(s) that Zig does call, or that no longer exist"
fi
# Both directions for the native list too: a name here that Zig DOES call is
# not native, and one that is no longer an entry point is a line outliving
# its reason.
comm -13 "$work/unwrapped" "$work/native" > "$work/native_excess"
if [ -s "$work/native_excess" ]; then
  sed 's/^/  /' "$work/native_excess" >&2
  fail "$(grep -c . "$work/native_excess") native entry point(s) that Zig does call, or that no longer exist"
fi

#-----------------------------------------------------------------------------
# Summary.
#-----------------------------------------------------------------------------
awk -F'\t' '$3 == "GAP"' "$work/rows" > "$work/open"
if [ "$LIST" -eq 1 ] && [ -s "$work/open" ]; then
  printf '%sgaps%s\n' "$BOLD" "$OFF"
  awk -F'\t' '{ printf "%s\t%s\n", $1, $2 }' "$work/open" | sort | column -t -s$'\t' >&2
  printf '\n'
fi

callable_names=$(grep -c . "$work/entrypoints")
load_coverage_totals
spelled=$coverage_names_spelled
public=$coverage_names_public
printf '%szjolt coverage%s\n' "$BOLD" "$OFF"
printf '  %-30s %5d\n' 'entry points exported' "$(count_api_decls)"
printf '  %-30s %5d\n' 'callable names checked' "$callable_names"
printf '  %-30s %5d  %scomputed in Zig, proved equal%s\n' \
  '  of them native' "$(grep -c . "$work/native")" "$DIM" "$OFF"
printf '  %-30s %5d\n' 'public Jolt names, claimed areas' "$public"
printf '  %-30s %5d  %sspelled out by an entry point%s\n' '  matched' "$spelled" "$DIM" "$OFF"
awk -F'\t' '{ c[$3]++ } END { for (v in c) printf "    %-28s %5d\n", tolower(v), c[v] }' "$work/rows" | sort

if [ -s "$work/open" ]; then
  fail "$(grep -c . "$work/open") gap(s) — run with --list to see them"
fi

# What a document says about any of this is ci/check-numbers.sh's job, not
# this script's: every count a document states has one gate, and this one
# would have been the second.
if [ "$fails" -ne 0 ]; then
  printf '\n%sFAIL%s  %d problem(s)\n' "$RED" "$OFF" "$fails" >&2
  exit 1
fi
printf '\n%sOK%s  every public name in the claimed areas is accounted for\n' "$GREEN" "$OFF"
