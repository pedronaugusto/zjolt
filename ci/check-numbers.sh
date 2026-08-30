#!/usr/bin/env bash
#
# zjolt — every count a document states, recomputed and COMPARED.
#
# ci/measurements.sh prints what is true and judges nothing. That is half a
# gate: a number in prose still rots silently, and four of them had. This
# recomputes the same quantities and FAILS when a document disagrees, so a
# count in a document is a measurement rather than a memory.
#
# Blind spots, stated here because a gate that looks total and is not is worse
# than none:
#
#   * Only the claims listed below are gated. A number added to a document
#     without a line here is not checked by anything.
#   * A count spelled in words cannot be matched. Write digits.
#   * Each claim is matched by a PHRASE, not by meaning. A reword that no
#     longer matches fails as "phrase not found" rather than passing quietly,
#     so a broken pattern is loud — but the phrase and the number it guards
#     have to move together.
#   * Nothing here reads the DOCUMENT'S sense. A gated number can be correct
#     and the sentence around it still wrong.
#
# Usage: ci/check-numbers.sh

set -uo pipefail
cd "$(dirname "$0")/.."

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; OFF=$'\033[0m'
else
  RED=; GREEN=; DIM=; OFF=
fi

fails=0

# claim <label> <file> <extended-regex> <expected numbers, space separated>
#
# Every match of the regex is reduced to the numbers inside it. All matches
# must agree, and they must equal the expected string: one number stated twice
# in a document cannot drift apart, and a phrase that matches nothing fails.
claim() {
  local label="$1" file="$2" pattern="$3" want="$4"
  local got
  got=$(grep -ohE -- "$pattern" "$file" |
        sed -E 's/[^0-9]+/ /g; s/^ //; s/ $//' | sort -u | tr '\n' '|')
  got=${got%|}
  if [ -z "$got" ]; then
    printf '  %s%-44s phrase not found in %s%s\n' "$RED" "$label" "$file" "$OFF" >&2
    fails=$((fails + 1))
    return
  fi
  if [ "$got" != "$want" ]; then
    printf '  %s%-44s %s says %s; the tree has %s%s\n' \
      "$RED" "$label" "$file" "$got" "$want" "$OFF" >&2
    fails=$((fails + 1))
    return
  fi
  printf '  %-44s %s%s%s\n' "$label" "$DIM" "$want" "$OFF"
}

#-----------------------------------------------------------------------------
# The tree's own numbers, every one of them computed by ci/counts.sh. Nothing
# in this file may recompute one itself: a second copy of a formula is what
# this gate exists to prevent, and it would be the first to break the rule.
#-----------------------------------------------------------------------------
. ci/counts.sh

entry_points=$(count_api_decls)
headers=$(count_public_headers)
returns_result=$(count_returns_result)
returns_void=$(count_returns_void)

drift_mutations=$(count_drift_mutations)
other_mutations=$(count_other_mutations)
all_mutations=$(count_all_mutations)

total_tu=$(count_total_tu)

host_checks=$(count_host_checks)
cross_targets=$(count_cross_targets)
full_checks=$(count_full_checks)

sweeps=$(count_reflective_sweeps)
c_modules=$(count_c_modules)
version=$(version_triple)

vehicle_points=$(count_api_decls_in ffi/zjolt_vehicle.h)
ragdoll_points=$(count_api_decls_in ffi/zjolt_ragdoll.h)
softbody_points=$(count_api_decls_in ffi/zjolt_softbody.h)

printf 'zjolt documented numbers\n'

claim 'README version' README.md \
  'Status: \*\*v[0-9]+\.[0-9]+\.[0-9]+' "$version"
claim 'README entry points' README.md \
  '\*\*[0-9]+ C entry points\*\*' "$entry_points"
claim 'README public headers' README.md \
  '\*\*[0-9]+ headers\*\*' "$headers"
claim 'README entry points returning an error code' README.md \
  'returning an error code . none . [0-9]+' "$returns_result"
claim 'README entry points returning void' README.md \
  'returning .void. . the large majority . [0-9]+' "$returns_void"
claim 'README ABI drift mutations' README.md \
  '[0-9]+ kinds of deliberate drift' "$drift_mutations"
claim 'README other-guard mutations' README.md \
  '[0-9]+ mutations for the other guards' "$other_mutations"
claim 'README mutations in all' README.md \
  '[0-9]+ mutations in all' "$all_mutations"
claim 'README translation units' README.md \
  '[0-9]+ translation units per configuration' "$total_tu"
claim 'README --full checks' README.md \
  '--full` is [0-9]+ checks' "$full_checks"
claim 'README --full host checks' README.md \
  '[0-9]+ of them run on this host' "$host_checks"
claim 'README --full cross builds' README.md \
  '[0-9]+ cross-compiling' "$cross_targets"
claim 'README reflective sweeps' README.md \
  '[0-9]+ reflective sweeps' "$sweeps"

claim 'BINDING C declaration modules' BINDING.md \
  'list of [0-9]+ modules' "$c_modules"

claim 'UPSTREAM vehicle/ragdoll/softbody' UPSTREAM.md \
  'carry [0-9]+, [0-9]+ and [0-9]+ entry points' \
  "$vehicle_points $ragdoll_points $softbody_points"

claim 'check-abi-drift other-guard mutations' ci/check-abi-drift.sh \
  '[0-9]+ of the mutations aim at the other guards' "$other_mutations"

claim 'workflow mutation count' .github/workflows/ci.yml \
  'mutates the sources [0-9]+ ways' "$all_mutations"

claim 'ci/run.sh translation units' ci/run.sh \
  '[0-9]+ translation units' "$total_tu"
claim 'ci/run.sh mutation label' ci/run.sh \
  'guard mutations \([0-9]+\)' "$all_mutations"

if [ "$fails" -ne 0 ]; then
  printf '\n%sFAIL%s  %d stale number(s)\n' "$RED" "$OFF" "$fails" >&2
  exit 1
fi
printf '\n%sOK%s  every gated number matches the tree\n' "$GREEN" "$OFF"
