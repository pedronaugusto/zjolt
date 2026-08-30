#!/usr/bin/env bash
#
# zjolt — a code example in the documentation is a verbatim excerpt of the
# file it names, and is CHECKED against it.
#
# BINDING.md's whole claim is "read this instead of reading a subsystem", so
# an example that no longer compiles teaches a signature the tree does not
# have. Its four-panel walk-through was wrong in all four panels at once: a
# parameter added to the C function, the Zig extern moved to another module,
# and the wrapper reshaped to take an options struct. README.md's quick start
# was wrong the other way round: it named no file, so it was the one piece of
# code in the repository that nothing compiled.
#
# The rule: a ```c / ```cpp / ```zig block preceded by a line holding nothing
# but a backticked path must appear in that file character for character.
#
# Two documents, two strictnesses, because their claims differ:
#
#   BINDING.md   EVERY c/cpp/zig block must name a file. It is a walk-through
#                of real code and an unlabelled block there is a lapse.
#   README.md    A labelled block is checked; an unlabelled one is not. The
#                README also illustrates, and an illustration has no file.
#
# Blind spots:
#
#   * An unlabelled README block is unchecked, which is the price of letting
#     the README illustrate at all. The quick start is labelled, so the one
#     example a reader is most likely to copy is held.
#   * Only c/cpp/zig blocks. A ```sh block naming a script that no longer
#     exists passes.
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

work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
fails=0
checked=0

# doc:strictness. `all` fails an unlabelled block; `labelled` skips it.
docs="BINDING.md:all README.md:labelled"

printf 'zjolt documentation examples\n'

for entry in $docs; do
  doc=${entry%%:*}
  mode=${entry##*:}
  [ -f "$doc" ] || { printf '  %s%s is missing%s\n' "$RED" "$doc" "$OFF" >&2
                     fails=$((fails + 1)); continue; }
  d="$work/$(printf '%s' "$doc" | tr -c 'A-Za-z0-9' '_')"
  mkdir -p "$d"

  # One record per fenced block: LINE, LANG, PATH (the backticked line
  # immediately above the fence, or empty), with the body in its own file.
  awk -v out="$d" '
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
  ' "$doc" > "$d/blocks"

  n=0
  while IFS=$'\t' read -r line lang path; do
    n=$((n + 1))
    case "$lang" in c|cpp|zig) ;; *) continue ;; esac

    if [ -z "$path" ]; then
      [ "$mode" = labelled ] && continue
      printf '  %s%s:%s a %s block names no file%s\n' "$RED" "$doc" "$line" "$lang" "$OFF" >&2
      fails=$((fails + 1))
      continue
    fi
    if [ ! -f "$path" ]; then
      printf '  %s%s:%s names %s, which does not exist%s\n' "$RED" "$doc" "$line" "$path" "$OFF" >&2
      fails=$((fails + 1))
      continue
    fi

    checked=$((checked + 1))
    if awk -v bf="$d/$n.body" '
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
  done < "$d/blocks"
done

if [ "$fails" -ne 0 ]; then
  printf '\n%sFAIL%s  %d stale example(s)\n' "$RED" "$OFF" "$fails" >&2
  exit 1
fi
printf '\n%sOK%s  %d example(s), each a verbatim excerpt of the file it names\n' \
  "$GREEN" "$OFF" "$checked"
