#!/usr/bin/env bash
#
# zjoltLiveHandleCount is what lets zjoltDeinit refuse while the host still
# owns something, and it is only as good as the arithmetic behind it. Two
# rules hold that arithmetic, and both were broken before this existed:
#
#   1. A Jolt refcount moves on the host's behalf ONLY inside
#      zjolt::HostRetain / zjolt::HostRelease. A bare AddRef or Release
#      anywhere else in ffi/ moves Jolt's count and not this one.
#   2. Every zjolt*AddRef entry point counts, and so does every zjolt*Release.
#      An AddRef that does not count while its Release does drives the total
#      NEGATIVE, which reads as "nothing outstanding" and hides a real leak.
#      zjoltConstraintSettingsAddRef was exactly that.
#
# Text only, no build: this is about which call is written, not what runs.
# The blind spot that leaves is a handle kind with no *AddRef/*Release pair at
# all -- one whose create counts and whose destroy does not is invisible here,
# and is caught instead by the tests, where zjoltDeinit refuses.
#
#   ci/check-refcounts.sh

set -uo pipefail
cd "$(dirname "$0")/.."

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; OFF=$'\033[0m'
else RED=; GREEN=; OFF=; fi

status=0
note() { printf '  %s%s%s\n' "$RED" "$1" "$OFF" >&2; status=1; }

# ---- 1. one home for the arithmetic ---------------------------------------
stray=$(grep -nE '(->|\.)(AddRef|Release)[[:space:]]*\(\)' ffi/*.cpp ffi/*.h |
          grep -v '^ffi/zjolt_internal.h:')
if [ -n "$stray" ]; then
  printf '%s\n' "$stray" | sed 's/^/  /' >&2
  note 'a Jolt refcount moves outside zjolt::HostRetain / zjolt::HostRelease'
fi
inside=$(grep -cE '(->|\.)(AddRef|Release)[[:space:]]*\(\)' ffi/zjolt_internal.h)
[ "$inside" -eq 2 ] ||
  note "ffi/zjolt_internal.h has $inside refcount call(s); HostRetain and HostRelease are the two"

# ---- 2. both ends of every host-facing pair account the same way -----------
#
# Only PAIRS: a lone zjolt*Release is a different thing -- zjoltBodyLockRead-
# Release hands back a lock, not a reference -- and has no AddRef to agree
# with. A pair is Jolt-reference-counted (it goes through the chokepoints
# above) or zjolt-reference-counted (the handle carries its own `refs`, and
# the count moves on create and on the release that destroys it). Either is
# correct; one end of one and one end of the other is the defect.
rows=$(awk '
  function flush(   kind) {
    if (name == "") return
    kind = "none"
    if (body ~ /HostRetain|HostRelease|Own\(/) kind = "jolt"
    else if (body ~ /refs\.fetch_/) kind = "own"
    stem = name
    sub(/(AddRef|Release)$/, "", stem)
    print stem "|" name "|" kind "|" FILENAME
    name = ""
  }
  match($0, /zjolt[A-Za-z0-9_]*(AddRef|Release)\(/) && $0 !~ /^[[:space:]]/ && $0 !~ /;[[:space:]]*$/ {
    flush()
    name = substr($0, RSTART, RLENGTH - 1)
    body = ""; depth = 0; started = 0
  }
  name != "" {
    body = body $0 ";"
    depth += gsub(/\{/, "{")
    if (depth > 0) started = 1
    depth -= gsub(/\}/, "}")
    if (started && depth <= 0) flush()
  }
' ffi/*.cpp | sort)

n_pairs=0
for stem in $(printf '%s\n' "$rows" | cut -d'|' -f1 | sort -u)
do
  [ -n "$stem" ] || continue
  mine=$(printf '%s\n' "$rows" | grep "^${stem}|")
  [ "$(printf '%s\n' "$mine" | grep -c .)" -eq 2 ] || continue
  n_pairs=$((n_pairs + 1))
  kinds=$(printf '%s\n' "$mine" | cut -d'|' -f3 | sort -u | tr '\n' ' ')
  case "$kinds" in
    'jolt ' | 'own ') ;;
    *) printf '%s\n' "$mine" | sed 's/^/    /' >&2
       note "${stem}AddRef and ${stem}Release do not account the same way: $kinds" ;;
  esac
done

[ "$n_pairs" -gt 0 ] ||
  note 'no zjolt*AddRef / zjolt*Release pair was found; this check read nothing'

[ $status -eq 0 ] || exit 1
printf '%sOK%s  one home for the host refcount, and all %d *AddRef/*Release pairs agree\n' \
  "$GREEN" "$OFF" "$n_pairs"
