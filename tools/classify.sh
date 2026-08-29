#!/usr/bin/env bash
#
# zjolt — the verdict for every unbound Jolt name that can be computed.
#
# The rule: an exclusion is legitimate only when upstream marks the name
# internal, when it has no public declaration, or when every declaration it
# has sits behind one of upstream's own scaffolding macros — the ones whose
# only #define in the tree is commented out, so no build upstream ships can
# reach them. Everything else is a binding, a Zig facility, or a gap.
#
#   tools/classify.sh          NAME<TAB>INTERNAL<TAB>evidence — provable only
#   tools/classify.sh --open   the names it cannot justify, one per line

set -uo pipefail
cd "$(dirname "$0")/.."

JOLT=libs/JoltPhysics/Jolt
t=$(mktemp -d); trap 'rm -rf "$t"' EXIT

tools/coverage.sh --names | cut -f2 | sort -u > "$t/unbound"

find "$JOLT" -name '*.h' -print0 | xargs -0 awk -f tools/jolt_internal.awk 2>/dev/null |
  sort -u | awk -F'\t' '!seen[$1]++' > "$t/marked"

find "$JOLT" -name '*.h' -print0 | xargs -0 awk -f tools/jolt_access.awk 2>/dev/null |
  awk -F'\t' '$2 == "public" { print $1 }' | sort -u > "$t/public"

scaff=$(grep -rhn '^//#define JPH_' "$JOLT" --include='*.h' | sed 's/.*#define //' | sort -u | tr '\n' ' ')
find "$JOLT" -name '*.h' -print0 |
  xargs -0 awk -v SCAFF="$scaff" -f tools/jolt_gated.awk 2>/dev/null | sort -u |
  awk -F'\t' '$2 == "open" { o[$1] = 1 } { if (!($1 in a)) a[$1] = $2 }
              END { for (n in a) if (!(n in o)) print n "\t" a[n] }' | sort -u > "$t/gated"

awk -F'\t' -v mode="${1:-}" '
  FILENAME ~ /marked$/ { if (!($1 in ev)) ev[$1] = "upstream " $2 " (" $3 ")"; next }
  FILENAME ~ /gated$/  { gate[$1] = $2; next }
  FILENAME ~ /public$/ { pub[$0] = 1; next }
  {
    if ($0 in ev)          v = ev[$0]
    else if (!($0 in pub)) v = "no public declaration"
    else if ($0 in gate)   v = "only declared under " gate[$0] ", whose sole #define upstream is commented out"
    else                   { if (mode == "--open") print $0; next }
    if (mode != "--open") print $0 "\tINTERNAL\t" v
  }
' "$t/marked" "$t/gated" "$t/public" "$t/unbound"
