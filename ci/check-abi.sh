#!/usr/bin/env bash
#
# zjolt — check that src/c.zig still declares the same functions as ffi/zjolt.h.
#
# The runtime ABI guard (zjoltAbiLayout, asserted in src/zjolt.zig) covers
# every struct: sizes, alignments, and a digest over every field name and
# offset. What it cannot see is the other half of a declaration — the
# FUNCTIONS. A parameter added, removed or reordered between the header and the
# Zig externs produces no struct change at all, links cleanly, and corrupts the
# stack at the call.
#
# So this compares the two lists directly: every function the header exports
# must appear in c.zig with the same number of parameters, and c.zig must not
# declare anything the header does not.
#
# It is a lexical check, not a type check — it will not catch `float` declared
# as `f64` in the same position. Those it leaves to the struct digest and to
# the C smoke test, which drives the same scenario as the Zig tests through the
# header instead of the externs, so a wrongly typed parameter shows up as a
# wrong answer.
#
# Usage: ci/check-abi.sh

set -euo pipefail
cd "$(dirname "$0")/.."

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; OFF=$'\033[0m'
else
  RED=; GREEN=; DIM=; OFF=
fi

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

# Flatten to one declaration per line as "name arity", for both sides.
#
# Comments go first, because a commented-out prototype is not an export, and
# `static inline` wrappers are skipped: zjoltInit is one, and Zig cannot call
# an inline C function, so its absence from c.zig is correct.
flatten() {
  # Drop preprocessor directives (the ZJOLT_API macro definition itself would
  # otherwise parse as a declaration), strip comments, then collapse to one
  # line.
  sed -e '/^[[:space:]]*#/d' -e 's://.*::' "$1" |
    tr '\n' ' ' |
    sed -e 's:/\*[^*]*\*\+\([^/*][^*]*\*\+\)*/: :g'
}

parse_decls() {
  # $1 = flattened text, $2 = marker introducing a declaration
  awk -v marker="$2" '
    function arity(params,   i, depth, count, ch, trimmed) {
      gsub(/^[ \t]+|[ \t]+$/, "", params)
      if (params == "" || params == "void") return 0
      depth = 0; count = 1
      for (i = 1; i <= length(params); i++) {
        ch = substr(params, i, 1)
        if (ch == "(") depth++
        else if (ch == ")") depth--
        else if (ch == "," && depth == 0) count++
      }
      return count
    }
    {
      text = $0
      while (match(text, marker)) {
        text = substr(text, RSTART + RLENGTH)
        # The declaration runs to the first semicolon.
        semi = index(text, ";")
        if (semi == 0) break
        decl = substr(text, 1, semi - 1)
        open_paren = index(decl, "(")
        if (open_paren == 0) continue
        # Scan to the matching close paren rather than assuming the
        # declaration ends at one: in C it does, but a Zig extern puts the
        # return type after the parameter list.
        depth = 0; close_paren = 0
        for (i = open_paren; i <= length(decl); i++) {
          ch = substr(decl, i, 1)
          if (ch == "(") depth++
          else if (ch == ")") { depth--; if (depth == 0) { close_paren = i; break } }
        }
        if (close_paren == 0) continue
        head = substr(decl, 1, open_paren - 1)
        params = substr(decl, open_paren + 1, close_paren - open_paren - 1)
        # The name is the last identifier before the parameter list.
        n = split(head, parts, /[ \t*]+/)
        name = parts[n]
        if (name != "") print name, arity(params)
      }
    }
  ' <<< "$1"
}

flatten ffi/zjolt.h > "$work/header.txt"
flatten src/c.zig > "$work/externs.txt"

parse_decls "$(cat "$work/header.txt")" "ZJOLT_API[ \t]+" | sort > "$work/header.list"
parse_decls "$(cat "$work/externs.txt")" "pub extern fn[ \t]+" | sort > "$work/externs.list"

header_count=$(wc -l < "$work/header.list" | tr -d ' ')
extern_count=$(wc -l < "$work/externs.list" | tr -d ' ')

if [ "$header_count" -eq 0 ] || [ "$extern_count" -eq 0 ]; then
  printf '%sparsed %s declarations from the header and %s from c.zig — one of ' \
    "$RED" "$header_count" "$extern_count" >&2
  printf 'them is zero, so this check is not actually checking anything.%s\n' \
    "$OFF" >&2
  exit 1
fi

printf '%s%s functions in ffi/zjolt.h, %s externs in src/c.zig%s\n' \
  "$DIM" "$header_count" "$extern_count" "$OFF"

status=0

# In the header but not in c.zig. zjoltInit is expected to be absent: it is a
# static inline wrapper, and the Zig side calls zjoltInitWithConfig directly.
missing=$(comm -23 <(cut -d' ' -f1 "$work/header.list") \
                   <(cut -d' ' -f1 "$work/externs.list") | grep -v '^zjoltInit$' || true)
if [ -n "$missing" ]; then
  printf '%sdeclared in ffi/zjolt.h but missing from src/c.zig:%s\n' "$RED" "$OFF" >&2
  printf '  %s\n' $missing >&2
  status=1
fi

extra=$(comm -13 <(cut -d' ' -f1 "$work/header.list") \
                 <(cut -d' ' -f1 "$work/externs.list") || true)
if [ -n "$extra" ]; then
  printf '%sdeclared in src/c.zig but not exported by ffi/zjolt.h:%s\n' "$RED" "$OFF" >&2
  printf '  %s\n' $extra >&2
  status=1
fi

# Same function, different number of parameters.
while read -r name arity; do
  other=$(awk -v n="$name" '$1 == n { print $2 }' "$work/externs.list")
  [ -z "$other" ] && continue
  if [ "$arity" != "$other" ]; then
    printf '%s%s takes %s parameters in ffi/zjolt.h but %s in src/c.zig%s\n' \
      "$RED" "$name" "$arity" "$other" "$OFF" >&2
    status=1
  fi
done < "$work/header.list"

if [ $status -ne 0 ]; then
  printf '\n%sthe C header and the Zig externs have drifted.%s\n' "$RED" "$OFF" >&2
  exit 1
fi

printf '%sffi/zjolt.h and src/c.zig declare the same functions%s\n' "$GREEN" "$OFF"
