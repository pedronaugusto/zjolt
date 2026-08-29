#!/usr/bin/env bash
#
# Refuses comment blocks that have stopped being reference documentation.
# Length per block, not density: a header where every declaration carries two
# crisp lines is correct at any percentage.
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
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT

for f in ffi/*.h ffi/*.cpp src/*.zig src/c/*.zig; do
  [ -f "$f" ] || continue
  awk -v F="$f" -v MD="$MAX_DECL" -v MH="$MAX_HEADER" '
    BEGIN { run = 0; start = 0; first = 1 }
    /^[[:space:]]*(\/\/|\/\/\/|\/\/!)/ {
      if (run == 0) { start = NR; banner = 0 }
      if ($0 ~ /^\/\/===/) banner = 1
      run++; next
    }
    {
      if (run > 0) {
        # A //===---===// banner heads a whole section, not one declaration, and
        # is where conventions covering everything below it belong.
        cap = ((first && start <= 3) || banner) ? MH : MD
        if (run > cap) printf "%s:%d: %d comment lines in one block (max %d)\n", F, start, run, cap
        first = 0; run = 0
      }
    }
    END { if (run > MD) printf "%s:%d: %d trailing comment lines\n", F, start, run }
  ' "$f" >> "$work/long" || { echo "check-comments: awk failed on $f" >&2; exit 2; }

  # Narrative register. These read as a person talking, not as documentation.
  grep -nEi '^[[:space:]]*(//|///|//!).*(\bwe\b|\bour\b|\bus\b|\bnote that\b|\bworth (stating|noting|saying)\b|\bused to be\b|\bthey used to\b|\bit used to\b|\bthe reason (is|it)\b|\bwhich is why\b|\bthat is why\b|\bturns out\b|\bin practice this\b|\bdo not be\b|\byou might (think|expect)\b|\bit is tempting\b)' "$f" |
    sed "s|^|$f:|" >> "$work/voice"
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
  printf '%s%d narrative comment line(s) — see COMMENT_STANDARD%s\n' "$RED" "$n" "$OFF" >&2
  fails=$((fails + 1))
fi

[ "$fails" -ne 0 ] && exit 1
printf '%sOK%s  no over-long or narrative comment blocks\n' "$GREEN" "$OFF"
