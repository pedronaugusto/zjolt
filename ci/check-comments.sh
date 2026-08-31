#!/usr/bin/env bash
#
# Refuses comment blocks that have stopped being reference documentation.
# Length per block, not density: a header where every declaration carries two
# crisp lines is correct at any percentage. Three rules, and this file is the
# only statement of any of them:
#
#   1. At most MAX_DECL lines directly above one declaration, MAX_HEADER for
#      the block at the top of a file or under a banner -- measured in
#      CHARACTERS at WIDTH columns, not in newlines. A budget counted in
#      newlines is defeated by not wrapping, and a 303-character line had.
#   2. No narrative register: the first person, an appeal to a former state,
#      and the phrases that introduce an aside rather than a fact.
#   3. Nothing that dates itself -- to a day, a machine, or one run of
#      something. A reader a year later cannot tell what it referred to.
#
# Rule 1 covers the declaration files, where a block sits above one name.
# Rules 2 and 3 cover every hand-written file, documents included.
#
#   ci/check-comments.sh [--list]

set -uo pipefail
cd "$(dirname "$0")/.."

# Byte-mode, everywhere: gawk under a UTF-8 locale rejects the octal byte
# ranges width() uses to strip continuation bytes, and an awk that dies
# with its output redirected leaves an empty findings file, which reads as
# a pass. One pinned locale plus a loud stop if awk itself fails keep this
# guard a guard.
export LC_ALL=C

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; OFF=$'\033[0m'
else RED=; GREEN=; OFF=; fi

MAX_DECL=6      # comment lines directly above one declaration
MAX_HEADER=14   # the block at the top of a file
WIDTH=80        # the column width both budgets are expressed in
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT

REGISTER='\bwe\b|\bour\b|\bus\b|\bnote that\b|\bworth (stating|noting|saying)\b|\bused to be\b|\bthey used to\b|\bit used to\b|\bthe reason (is|it)\b|\bwhich is why\b|\bthat is why\b|\bturns out\b|\bin practice this\b|\bdo not be\b|\byou might (think|expect)\b|\bit is tempting\b'
DATED='\bat the time of writing\b|\bas of (today|now|this writing)\b|\bon (this|my) (machine|laptop|box)\b|\b(on|in|during) the first run\b'

# `$1` is the comment marker for the file's language; a document has none, so
# every line of it is prose.
voice() {
  local marker="$1" f="$2"
  [ -f "$f" ] || return 0
  grep -nEi "^[[:space:]]*($marker).*($REGISTER)" "$f" | sed "s|^|$f:|" >> "$work/voice"
  grep -nEi "^[[:space:]]*($marker).*($DATED)" "$f" | sed "s|^|$f:|" >> "$work/dated"
}

for f in build.zig tests/c_smoke.c tests/consumer/build.zig tests/consumer/src/*.zig; do
  voice '//|///|//!' "$f"
done
for f in ci/*.sh tools/*.sh; do
  voice '#' "$f"
done
for f in *.md docs/*.md; do
  [ -f "$f" ] || continue
  grep -nEi "$DATED" "$f" | sed "s|^|$f:|" >> "$work/dated"
done

for f in ffi/*.h ffi/*.cpp src/*.zig src/c/*.zig; do
  [ -f "$f" ] || continue
  awk -v F="$f" -v MD="$MAX_DECL" -v MH="$MAX_HEADER" -v W="$WIDTH" '
    # Characters, not bytes. A UTF-8 continuation byte is not a column, and an
    # em dash would otherwise cost three of them.
    function width(line,   t) { t = line; gsub(/[\200-\277]/, "", t); return length(t) }
    BEGIN { run = 0; chars = 0; start = 0; first = 1 }
    /^[[:space:]]*(\/\/|\/\/\/|\/\/!)/ {
      if (run == 0) { start = NR; banner = 0 }
      if ($0 ~ /^[[:space:]]*\/\/[-=][-=][-=]/) banner = 1
      run++; chars += width($0); next
    }
    {
      if (run > 0) {
        # A //===---===// banner heads a whole section, not one declaration, and
        # is where conventions covering everything below it belong. It is
        # indented inside a Zig struct, so the match cannot be anchored to
        # column 0.
        cap = ((first && start <= 3) || banner) ? MH : MD
        if (chars > cap * W)
          printf "%s:%d: %d comment characters in one block (max %d, %d lines wide)\n", F, start, chars, cap * W, cap
        first = 0; run = 0; chars = 0
      }
    }
    END { if (chars > MD * W) printf "%s:%d: %d trailing comment characters\n", F, start, chars }
  ' "$f" >> "$work/long" || { echo "check-comments: awk failed on $f" >&2; exit 2; }

  voice '//|///|//!' "$f"
done

fails=0
if [ -s "$work/long" ]; then
  sort -t: -k1,1 "$work/long" | head -40 | sed 's/^/  /' >&2
  n=$(grep -c . "$work/long")
  printf '%s%d over-long comment block(s)%s\n' "$RED" "$n" "$OFF" >&2
  fails=$((fails + 1))
fi
if [ -s "$work/voice" ]; then
  head -30 "$work/voice" | sed 's/^/  /' >&2
  n=$(grep -c . "$work/voice")
  printf '%s%d narrative comment line(s) — the register list is in ci/check-comments.sh%s\n' "$RED" "$n" "$OFF" >&2
  fails=$((fails + 1))
fi
if [ -s "$work/dated" ]; then
  head -30 "$work/dated" | sed 's/^/  /' >&2
  n=$(grep -c . "$work/dated")
  printf '%s%d line(s) dated to a day, a machine or one run%s\n' "$RED" "$n" "$OFF" >&2
  fails=$((fails + 1))
fi

[ "$fails" -ne 0 ] && exit 1
printf '%sOK%s  no over-long, narrative or self-dating comments\n' "$GREEN" "$OFF"
