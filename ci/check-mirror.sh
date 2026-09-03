#!/usr/bin/env bash
#
# Refuses a local roster that has stopped mirroring the hosted one.
#
# `ci/run.sh` says at the top that it mirrors `.github/workflows/ci.yml`, so a
# failure can be reproduced on the machine that caused it. Nothing held that
# claim in zjolt until this script existed, and where the same claim went
# unheld in a sibling package it was false: the workflow built a configuration
# the local roster did not, so an option declared in `build.zig` could be
# broken for as long as it took someone to open a pull request.
#
# What is compared is the set of BUILD OPTION COMBINATIONS each file executes:
# every `-D` on a build command except `-Dtarget` and `-Doptimize`, sorted so a
# step is identified by what it sets rather than by the order it sets it in,
# and taken as a set so a loop that expands to several commands counts once.
# Target and optimize mode are left out because the two files are legitimately
# different there — the workflow names a runner per platform and spells each
# mode out; the roster runs on whichever host it is invoked on and loops.
#
# Two details decide whether the comparison is honest:
#
#   * Logical lines, not physical ones. A shell command continued with `\`, or
#     a YAML folded scalar (`run: >-`), is ONE command whose options are spread
#     over several lines; read line by line, every option past the first line
#     is invisible and the step reads as unmatched. A literal block (`run: |`)
#     is the opposite case — several commands — so it is not folded.
#   * Only commands count. Options are read after the `build` word, because a
#     step's own label often quotes the option it is about ('test
#     -Dsimd=false (scalar codecs)'), and comment lines are dropped, because
#     prose naming an option beside the word "build" is not a step. Both
#     cases were found by running this against all seven packages: each
#     produced a phantom combination in exactly one of them.
#
#   ci/check-mirror.sh

set -uo pipefail
cd "$(dirname "$0")/.."

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; OFF=$'\033[0m'
else RED=; GREEN=; OFF=; fi

work=$(mktemp -d); trap 'rm -rf "$work"' EXIT

combinations() {
  awk '
    function flush() { if (buf != "") { print buf; buf = "" } }
    {
      raw = $0
      sub(/\r$/, "", raw)
      if (raw ~ /^[[:space:]]*#/) next
      if (block) {
        if (raw ~ /^[[:space:]]*$/) next
        ind = match(raw, /[^ \t]/) - 1
        if (ind > block_ind) { buf = buf " " raw; next }
        flush(); block = 0
      }
      if (cont) buf = buf " " raw; else buf = raw
      cont = 0
      if (buf ~ /\\$/) { sub(/\\$/, "", buf); cont = 1; next }
      if (buf ~ /run:[[:space:]]*>-?[[:space:]]*$/) {
        block = 1; block_ind = match(buf, /[^ \t]/) - 1
        next
      }
      flush()
    }
    END { flush() }
  ' "$1" | tr -d '\r\042\047' | awk '
    {
      start = 0
      for (i = 1; i <= NF; i++)
        if ($i == "build" || $i == "zbuild") { start = i; break }
      if (start == 0) next
      n = 0
      for (i = start + 1; i <= NF; i++)
        if ($i ~ /^-D/ && $i !~ /^-Dtarget=/ && $i !~ /^-Doptimize=/) o[++n] = $i
      if (n == 0) next
      for (i = 2; i <= n; i++) {
        v = o[i]; j = i - 1
        while (j > 0 && o[j] > v) { o[j + 1] = o[j]; j-- }
        o[j + 1] = v
      }
      s = o[1]
      for (i = 2; i <= n; i++) s = s " " o[i]
      print s
    }' | sort -u
}

combinations .github/workflows/ci.yml > "$work/hosted"
combinations ci/run.sh > "$work/local"

comm -23 "$work/hosted" "$work/local" > "$work/hosted_only"
comm -13 "$work/hosted" "$work/local" > "$work/local_only"

fails=0
if [ -s "$work/hosted_only" ]; then
  sed 's/^/  /' "$work/hosted_only" >&2
  printf '%s%d option combination(s) the workflow builds and ci/run.sh does not%s\n' \
    "$RED" "$(grep -c . "$work/hosted_only")" "$OFF" >&2
  fails=$((fails + 1))
fi
if [ -s "$work/local_only" ]; then
  sed 's/^/  /' "$work/local_only" >&2
  printf '%s%d option combination(s) ci/run.sh builds and the workflow does not%s\n' \
    "$RED" "$(grep -c . "$work/local_only")" "$OFF" >&2
  fails=$((fails + 1))
fi

[ "$fails" -ne 0 ] && exit 1
printf '%sOK%s  %s option combination(s), the same in ci/run.sh and the workflow\n' \
  "$GREEN" "$OFF" "$(grep -c . "$work/hosted")"
