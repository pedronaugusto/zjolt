# Adding surface to zjolt

How a new subsystem gets bound, written down so every one of them comes out
the same shape. This is the contract a change has to satisfy; the *reasons*
behind most of it are in `README.md` under Design, and are not repeated here.

## Start with recon

```sh
tools/recon.sh HingeConstraint
```

Binding a C++ API is not transcription, and the facts that decide whether a
binding is correct are not in the signatures. They are `JPH_ASSERT`
preconditions inside method bodies, a base class whose reference count starts
at zero, a private constructor, a getter that returns a default instead of
failing. `tools/recon.sh` pulls exactly those out of a class, its `.cpp` and
its declaration, with line numbers — a page instead of a file.

It is a lead generator, not an oracle. Everything it prints is a real line
worth opening, and it will not find a precondition nobody wrote down. Read the
header too; read a shorter part of it.

## The five files a subsystem touches

A subsystem is one concern — constraints, vehicles, ragdolls — and it gets:

| file | what goes in it |
|---|---|
| `ffi/zjolt_<area>.h` | the C declarations, `ZJOLT_API` on each |
| `ffi/zjolt_<area>.cpp` | the C++ implementation, `#include "zjolt_internal.h"` first |
| `src/c.zig` | one `pub extern fn` per entry point, hand-written |
| `src/<area>.zig` | the Zig wrapper — errors, slices, methods |
| `src/zjolt.zig` | re-exports of the wrapper's public names |

Plus three one-line registrations: the header in `public_headers` and the
`.cpp` in the source list, both in `build.zig`; and `#include "zjolt_<area>.h"`
in the umbrella `ffi/zjolt.h`.

Nothing in `src/` may `@cImport`. `src/c.zig` is hand-written on purpose, and
`src/abi_check.zig` is what proves it did not drift.

## Naming, which is load-bearing

`src/abi_check.zig` pairs the two sides by name and has **no hand-written
list**, so a name that breaks convention is a build failure, not a style nit:

| Zig side | C side |
|---|---|
| type `Foo` | `ZJoltFoo` |
| function `zjoltFoo` | `zjoltFoo` |
| constant `foo_bar` | `ZJOLT_FOO_BAR` |
| enum `Foo`'s field `bar` | `ZJOLT_FOO_BAR` |

Enumerators take the **full type name**, always: `ZJOLT_MOTION_TYPE_STATIC`,
not `ZJOLT_STATIC`. translate-c flattens a C enum to a plain integer alias, so
the strict prefix is the only thing that can pair an enumerator back to the
enum it came from.

**No enumerator may be negative.** C leaves an enum's underlying type to the
implementation and the implementations disagree — clang and gcc pick unsigned
when no enumerator is negative, MSVC uses `int`. Every value being non-negative
is what makes that unobservable, and the guard enforces it. If you need a
negative sentinel, use a fixed-width constant instead of an enum.

## The C ABI

- **Every fallible entry point returns `ZJoltResult`** and delivers its answer
  through an out-parameter. Infallible ones return the value directly.
- **Open every result-returning entry point with `ZJOLT_ENTER(...)`**, passing
  its out-parameters. It clears them and returns `ZJOLT_RESULT_NOT_INITIALIZED`
  from the caller if the library is not up — so a refused call still leaves the
  caller's outputs defined. Wrap an out-parameter in `zjolt::OutIsEmptyAs<T>`
  when its empty value is not zero.
- **Validate pointers with `zjolt::Present(...)`**, which reports
  `ZJOLT_RESULT_INVALID_ARGUMENT` naming the parameter.
- **Errors carry a message**: `zjolt::SetError(code, "what and why")`, readable
  through `zjoltLastError()`.
- **The declared surface never depends on build options.** A function that
  cannot work in this configuration is still declared and returns
  `ZJOLT_RESULT_UNSUPPORTED`. A header whose contents move with `-D` flags is a
  header that cannot be checked.
- **Ids and handles convert in exactly one place** — `zjolt::ToJolt` /
  `zjolt::ToC` in `ffi/zjolt_internal.h`. Do not write a local conversion.
- **Settings objects do not cross.** Jolt's `*Settings` types exist so a scene
  can be serialised; a host driving this ABI has its own asset format. Take the
  parameters directly, build the settings on the stack, hand back the result.

### Reference counting

**Use `zjolt::Own(fresh)`** to hand a newly constructed reference-counted
object to the caller. The arithmetic is not what anyone expects — a fresh
`JPH::RefTarget` starts at **zero**, not one, so the caller's reference is the
first — and it is not the same arithmetic as `Finish` in `ffi/zjolt_shape.cpp`,
which also calls `AddRef` once but to compensate for a `JPH::Ref` dropping at
scope exit. Same call, two different reasons, and mixing them up is silent:
under-counting wraps the counter to `0xFFFFFFFF` and the object outlives its
own frees; over-counting shows up as leaked bytes in `tests/c_smoke.c`.

`Own` is the answer to both, and using it means the question does not come up.

### Callbacks

**Nothing may unwind out of a callback.** Jolt compiles `-fno-exceptions`, so
an exception crossing one is `std::terminate` — and a Zig panic unwinding out
of a collector skips the broad-phase `shared_lock` destructor and *permanently
deadlocks the next `PhysicsSystem::Update`*, silently and much later. Zig shims
carry an error out in the user context and re-raise it after the call returns.

Callbacks cross as **C function pointers plus a `void *user`**, never as a
mirrored C++ vtable — see README, Design. A callback that returns a decision
returns an **enum**, not a raw float the caller can get wrong.

## The Zig wrapper

- Turn `ZJoltResult` into `Error!T` with `error_mod.check`.
- Turn count-then-fill pairs into slices; take an allocator explicitly.
- Turn C enums into Zig enums and bit masks into `packed struct(u32)`.
- Never widen a lifetime the C side did not promise. If a pointer is valid only
  during one callback, do not hand it to the caller — copy what is needed.
- Document what Jolt actually does, not what would be tidy. If a getter returns
  a default when a lock fails, say so; do not invent a `null` Jolt never
  returns.

## Tests every subsystem owes

1. **One behavioural test** asserting the characteristic behaviour — a hinge
   constrains to its axis, a ragdoll settles, a saved state restores
   bit-identically. Not one per entry point.
2. **`tests/c_smoke.c` extended** with the same scenario in plain C. This is
   what covers translate-c's `[*c]` residue: the ABI guard compares pointee
   types only by size and alignment, so a `float *` declared `*i32` passes it
   and only C catches it.
3. Every Zig test runs through `std.testing.allocator`, which fails on a leak.

## Before you call it done

```sh
zig build test
zig build test-c
zig build --build-file tests/consumer/build.zig run
ci/run.sh --full        # includes ci/check-abi-drift.sh
```

`ci/run.sh --full` executes the configurations that change ABI widths
(`-Ddouble_precision`, `-Dobject_layer_bits=32`), which a compile alone does
not prove.
