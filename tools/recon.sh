#!/usr/bin/env bash
#
# zjolt — what a binder needs to know about a Jolt class, on one screen.
#
# Binding a C++ API is not transcription. The facts that decide whether a
# binding is correct are not in the signatures: they are `JPH_ASSERT`
# preconditions in method bodies, a base class that starts its reference count
# at zero, a private constructor, a getter that returns a default instead of
# failing. Finding those means reading the class, its parent, and its `.cpp` —
# a few thousand lines to bind a dozen functions, most of it irrelevant.
#
# This extracts exactly the parts that decide the binding, so the reading is a
# page instead of a file. It is a lead generator, not an oracle: everything it
# prints is a real line in a real file with a line number, and anything it
# flags is worth opening. It will not find a precondition nobody wrote down.
#
# Usage:
#   tools/recon.sh PhysicsMaterial
#   tools/recon.sh HingeConstraint
#   tools/recon.sh Jolt/Physics/Collision/Shape/ConvexShape.h

set -uo pipefail
cd "$(dirname "$0")/.."

JOLT=libs/JoltPhysics

if [ $# -ne 1 ]; then
  sed -n '3,22p' "$0" | sed 's/^#\{0,1\} \{0,1\}//'
  exit 2
fi

if [ -t 1 ]; then
  B=$'\033[1m'; D=$'\033[2m'; Y=$'\033[33m'; R=$'\033[31m'; O=$'\033[0m'
else
  B=; D=; Y=; R=; O=
fi

#-----------------------------------------------------------------------------
# Locate
#-----------------------------------------------------------------------------

arg="$1"
if [ -f "$JOLT/$arg" ]; then
  header="$JOLT/$arg"
  name=$(basename "$arg" .h)
elif [ -f "$arg" ]; then
  header="$arg"
  name=$(basename "$arg" .h)
else
  name="$arg"
  # A file named after the class first: Jolt is consistent about that, and it
  # skips every forward declaration in one step.
  header=$(find "$JOLT/Jolt" -name "$name.h" | head -1)
  if [ -z "$header" ]; then
    # Otherwise the class has to be DEFINED, not forward-declared. A line
    # ending in `;` is a promise that the definition is somewhere else, and
    # matching it sends the whole report to the wrong file.
    header=$(grep -rl --include='*.h' -E \
      "^[[:space:]]*(class|struct)[[:space:]]+(JPH_EXPORT[[:space:]]+)?$name\b[^;]*$" \
      "$JOLT/Jolt" 2>/dev/null | head -1)
  fi
  if [ -z "$header" ]; then
    printf '%sno header in %s defines `%s`%s\n' "$R" "$JOLT/Jolt" "$name" "$O" >&2
    printf '%stry a path, or check the spelling%s\n' "$D" "$O" >&2
    exit 1
  fi
fi

source_file="${header%.h}.cpp"
[ -f "$source_file" ] || source_file=""

section() { printf '\n%s%s%s\n' "$B" "$1" "$O"; }
none() { printf '  %s(none)%s\n' "$D" "$O"; }

printf '%s%s%s  %s%s%s\n' "$B" "$name" "$O" "$D" "$header" "$O"
[ -n "$source_file" ] && printf '%s          %s%s\n' "$D" "$source_file" "$O"

#-----------------------------------------------------------------------------
# What kind of thing it is
#
# Each line here changes how the binding is written, not merely how it reads.
#-----------------------------------------------------------------------------

section 'Kind'

decl=$(grep -nE "^[[:space:]]*(class|struct)[[:space:]]+(JPH_EXPORT[[:space:]]+)?$name\b[^;]*$" "$header" | head -1)
printf '  %s\n' "${decl:-  (declaration not found)}"

if printf '%s' "$decl" | grep -q 'RefTarget'; then
  printf '  %sREFERENCE COUNTED%s — a fresh RefTarget starts at ZERO, so a\n' "$Y" "$O"
  printf '    constructor handing out a raw pointer calls AddRef() exactly once.\n'
  printf '    Release() frees through the host allocator via JPH_OVERRIDE_NEW_DELETE.\n'
fi
if grep -q "JPH_DECLARE_RTTI" "$header" ||
   printf '%s' "$decl" | grep -q 'SerializableObject'; then
  printf '  Carries Jolt RTTI — a subclass needs Jolt'"'"'s OWN RTTI macros,\n'
  printf '    plus Factory::Register. NOT C++ RTTI: zjolt compiles -fno-rtti.\n'
fi
# A pure virtual is `virtual ... = 0;`. Matching a bare `= 0;` also matches
# every default member initialiser — `float mFoo = 0;` — which reported
# concrete settings structs as abstract and sent a binder looking for a
# factory that does not exist. The `virtual` is the whole signal.
grep -qE 'virtual[^;]*=[[:space:]]*0[[:space:]]*;' "$header" &&
  printf '  %sABSTRACT%s — has pure virtuals; it cannot be constructed directly.\n' "$Y" "$O"
if [ -f "$JOLT/Jolt/Physics/${name}Settings.h" ] ||
   grep -qE "^[[:space:]]*(class|struct)[[:space:]]+(JPH_EXPORT[[:space:]]+)?${name}Settings\b" \
     "$header" "$(dirname "$header")"/*.h 2>/dev/null; then
  printf '  Has a %sSettings companion — build it on the STACK; settings objects\n' "$name"
  printf '    do not cross the boundary (see BINDING.md).\n'
fi

#-----------------------------------------------------------------------------
# Preconditions
#
# The reason this script exists. An assert is a precondition the signature does
# not state, and a binding that forwards a call without checking it turns a
# caller mistake into a process abort.
#-----------------------------------------------------------------------------

section 'Preconditions (JPH_ASSERT) — a caller can trip these'

asserts=$(grep -nE "JPH_ASSERT" "$header" ${source_file:+"$source_file"} 2>/dev/null |
  grep -vE 'JPH_ASSERT\(false' | head -40)
if [ -z "$asserts" ]; then none; else
  printf '%s\n' "$asserts" | sed 's/^/  /'
  total=$(grep -hE "JPH_ASSERT" "$header" ${source_file:+"$source_file"} 2>/dev/null |
    grep -vcE 'JPH_ASSERT\(false')
  [ "${total:-0}" -gt 40 ] && printf '  %s... %d more, not shown%s\n' "$D" "$((total - 40))" "$O"
fi

#-----------------------------------------------------------------------------
# Getters that fail quietly
#
# A getter that returns a default when it cannot answer looks like success at
# the C boundary. The binding cannot invent a different contract; it has to
# forward the answer and say so in the header.
#-----------------------------------------------------------------------------

section 'Silent defaults — a failure path that looks like success'

# Named defaults: sDefault, sZero, sIdentity and friends.
quiet=$(grep -nE 'return[[:space:]]+.*(sDefault|sZero|sIdentity|Default\(\)|sInvalid)' \
  ${source_file:+"$source_file"} "$header" 2>/dev/null | head -15)

# And the shape those miss, which is the common one in this library: a getter
# takes a body lock and returns a bare literal when it fails.
#
#     BodyLockRead lock(...);
#     if (lock.Succeeded() && ...) return real_answer;
#     else                         return 500.0f;
#
# `500.0f` is not a symbol, so no name-based scan finds it — and it is exactly
# the case that matters, because a stale body id comes back looking like a
# successful answer. Found by reading the source, so read it here.
locked=$(grep -nA 6 -E 'lock\.Succeeded\(\)|\.Succeeded\(\)' \
  ${source_file:+"$source_file"} 2>/dev/null |
  grep -E '^[0-9]+.[[:space:]]*(else[[:space:]]+)?return' | head -20)

if [ -z "$quiet" ] && [ -z "$locked" ]; then none; else
  [ -n "$quiet" ] && printf '%s\n' "$quiet" | sed 's/^/  /'
  if [ -n "$locked" ]; then
    printf '  %son a failed lock:%s\n' "$Y" "$O"
    printf '%s\n' "$locked" | sed 's/^/    /'
  fi
fi

#-----------------------------------------------------------------------------
# What is out of reach
#
# zjolt does not pass -fno-access-control, so a private member is a real wall
# and the binding has to find the public way in.
#-----------------------------------------------------------------------------

section 'Private / protected — no -fno-access-control here'

priv=$(awk '
  /^[[:space:]]*(private|protected):/ { in_priv = 1; printf "%d:%s\n", NR, $0; next }
  /^[[:space:]]*public:/              { in_priv = 0; next }
  in_priv && /[A-Za-z_].*[(;]/        { printf "%d:%s\n", NR, $0 }
' "$header" | head -20)
if [ -z "$priv" ]; then none; else printf '%s\n' "$priv" | sed 's/^/  /'; fi

#-----------------------------------------------------------------------------
# Enums
#
# zjolt mirrors these, and its ABI cross-check refuses a negative enumerator:
# C leaves an enum's underlying type to the implementation and the choice is
# only unobservable while every value is non-negative. See BINDING.md.
#-----------------------------------------------------------------------------

section 'Enums to mirror'

enums=$(grep -nE '^[[:space:]]*enum class[[:space:]]+[A-Za-z]' "$header" | head -15)
if [ -z "$enums" ]; then none; else
  printf '%s\n' "$enums" | sed 's/^/  /'
  neg=$(grep -nE '=[[:space:]]*-[0-9]' "$header")
  if [ -n "$neg" ]; then
    printf '  %sNEGATIVE VALUE PRESENT%s — if it is an enumerator, it cannot be\n' "$R" "$O"
    printf '    mirrored as a C enum. Use a fixed-width constant instead.\n'
    printf '%s\n' "$neg" | sed 's/^/    /'
  fi
fi

#-----------------------------------------------------------------------------
# The public surface
#-----------------------------------------------------------------------------

section 'Public methods'

awk '
  /^[[:space:]]*(private|protected):/ { pub = 0; next }
  /^[[:space:]]*public:/              { pub = 1; next }
  /^[[:space:]]*(class|struct)[[:space:]]/ { pub = 0 }
  !pub                                { next }
  /JPH_ASSERT|^[[:space:]]*\/\/|^[[:space:]]*\*/ { next }
  /\(/ && /[;{]/                      { gsub(/^[[:space:]]+/, ""); printf "  %s\n", $0 }
' "$header" | head -60

count=$(awk '
  /^[[:space:]]*(private|protected):/ { pub = 0; next }
  /^[[:space:]]*public:/              { pub = 1; next }
  !pub { next }
  /JPH_ASSERT|^[[:space:]]*\/\// { next }
  /\(/ && /[;{]/ { n++ }
  END { print n + 0 }
' "$header")
[ "$count" -gt 60 ] && printf '  %s... %d more, read the header%s\n' "$D" "$((count - 60))" "$O"

printf '\n%sthen read the header. This narrows the reading; it does not replace it.%s\n' \
  "$D" "$O"
