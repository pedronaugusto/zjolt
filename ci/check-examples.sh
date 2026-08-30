#!/usr/bin/env bash
#
# zjolt — every code example in BINDING.md is a verbatim excerpt of the file
# it names, and is CHECKED against it.
#
# BINDING.md's whole claim is "read this instead of reading a subsystem", so
# an example that no longer compiles teaches a signature the tree does not
# have. Its four-panel walk-through was wrong in all four panels at once: a
# parameter added to the C function, the Zig extern moved to another module,
# and the wrapper reshaped to take an options struct.
#
# The rule this enforces: a ```c / ```cpp / ```zig block must be preceded by a
# line holding nothing but a backticked path, and the block must appear in
# that file character for character.
#
# Blind spots:
#
#   * Only BINDING.md is scanned, and only its c/cpp/zig blocks. A ```sh block
#     naming a script that no longer exists passes.
#   * Substring, not identity: an excerpt is proven to BE in the file, not to
#     be the whole declaration, and not to be the only one like it.
#   * Nothing reads the prose. An excerpt can be current and the sentence
#     around it wrong.
#
# Usage: ci/check-examples.sh

set -uo pipefail
cd "$(dirname "$0")/.."

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; OFF=$'\033[0m'
else
  RED=; GREEN=; DIM=; OFF=
fi

doc=BINDING.md
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
fails=0

# Splits the document into one record per fenced block: LINE, LANG, PATH (the
# backticked line immediately above the fence, or empty), then the body.
awk -v out="$work" '
  /^```/ {
    if (!inside) {
      inside = 1; lang = substr($0, 4); start = NR; n++
      body = out "/" n ".body"
      printf "%d\t%s\t%s\n", start, lang, label
      next
    }
    inside = 0; label = ""; next
  }
  inside { print > body; next }
  { label = ($0 ~ /^`[^`]+`$/) ? substr($0, 2, length($0) - 2) : "" }
' "$doc" > "$work/blocks"

printf 'zjolt documentation examples\n'

n=0
while IFS=$'\t' read -r line lang path; do
  n=$((n + 1))
  case "$lang" in c|cpp|zig) ;; *) continue ;; esac

  if [ -z "$path" ]; then
    printf '  %s%s:%s a %s block names no file%s\n' "$RED" "$doc" "$line" "$lang" "$OFF" >&2
    fails=$((fails + 1))
    continue
  fi
  if [ ! -f "$path" ]; then
    printf '  %s%s:%s names %s, which does not exist%s\n' "$RED" "$doc" "$line" "$path" "$OFF" >&2
    fails=$((fails + 1))
    continue
  fi

  if awk -v bf="$work/$n.body" '
    BEGIN { while ((getline l < bf) > 0) block = block l "\n" }
    { hay = hay $0 "\n" }
    END { exit(index(hay, block) ? 0 : 1) }
  ' "$path"; then
    printf '  %s:%-4s %s%s%s\n' "$doc" "$line" "$DIM" "$path" "$OFF"
  else
    printf '  %s%s:%s is not in %s character for character%s\n' \
      "$RED" "$doc" "$line" "$path" "$OFF" >&2
    fails=$((fails + 1))
  fi
done < "$work/blocks"

if [ "$fails" -ne 0 ]; then
  printf '\n%sFAIL%s  %d stale example(s)\n' "$RED" "$OFF" "$fails" >&2
  exit 1
fi
printf '\n%sOK%s  every example is a verbatim excerpt of the file it names\n' "$GREEN" "$OFF"
