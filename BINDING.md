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
| `src/c/<area>.zig` | one `pub extern fn` per entry point, hand-written |
| `src/<area>.zig` | the Zig wrapper — errors, slices, methods |
| `src/zjolt.zig` | re-exports of the wrapper's public names |

Plus four one-line registrations: the header in `public_headers` and the
`.cpp` in the source list, both in `build.zig`; `#include "zjolt_<area>.h"` in
the umbrella `ffi/zjolt.h`; and the module in `src/c.zig`, which is the list
`src/abi_check.zig` and `src/misuse_sweep_test.zig` walk. A module missing
from that list is a module neither guard covers.

Nothing in `src/` may `@cImport`. `src/c/` is hand-written on purpose, and
`src/abi_check.zig` is what proves it did not drift.

## One entry point, all the way through

This is the whole pattern. Read it instead of reading a subsystem. Every
panel below is a verbatim excerpt, and `ci/check-examples.sh` fails the build
if it stops matching the file it names.

`ffi/zjolt_shape.h`
```c
ZJOLT_API ZJoltResult zjoltShapeCreateSphere(
    float radius, float density, const ZJoltPhysicsMaterial *material,
    ZJoltShape **out);
```

`ffi/zjolt_shape.cpp`
```cpp
ZJoltResult zjoltShapeCreateSphere(float radius, float density,
                                   const ZJoltPhysicsMaterial *material,
                                   ZJoltShape **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  JPH::SphereShapeSettings settings(radius, ToJoltMaterial(material));
  if (density > 0.0f) settings.SetDensity(density);
  JPH::Shape::ShapeResult result = settings.Create();
  return Finish(result, out);
}
```

Four things in eleven lines: `ZJOLT_ENTER` clears the out-parameters and
refuses a call made before `zjoltInit`; `zjolt::Present` reports a null
pointer as `ZJOLT_RESULT_INVALID_ARGUMENT` naming the parameter; the settings
object is built on the STACK and never crosses; and `Finish` does the
reference-count arithmetic, as `zjolt::Own(fresh)` does for an object that is
not a shape.

`src/c/shape.zig`
```zig
pub extern fn zjoltShapeCreateSphere(radius: f32, density: f32, material: ?*const PhysicsMaterial, out: **Shape) Result;
```

`src/shape.zig`
```zig
    pub fn initSphere(radius: f32, opts: ConvexOptions) err.Error!Shape {
        var handle: *c.Shape = undefined;
        try err.check(c.zjoltShapeCreateSphere(
            radius,
            opts.density,
            materialHandle(opts.material),
            &handle,
        ));
        return .{ .handle = handle };
    }
```

The wrapper is where a C parameter list becomes Zig: the optional trailing
parameters every convex shape shares collapse into one `ConvexOptions`, and
`ZJoltResult` becomes `err.Error!Shape`.

`src/zjolt.zig` — one re-export line. `build.zig` — the `.h` in `public_headers`,
the `.cpp` in the FFI source list, any new Jolt TUs in `jolt_sources`.

## Do not read anything else

Startup reading is the single largest cost in binding a subsystem, and almost
all of it is wasted. **Read this file, then run `tools/recon.sh` on your
classes, then write.** Do not read `README.md`, do not read `git log`, do not
open another subsystem "for style" — the style is above. Do not open a Jolt
header that recon already summarised unless recon pointed you at a line in it.

## Naming, which is load-bearing

`src/abi_check.zig` pairs the two sides by name and has **no hand-written
list**, so a name that breaks convention is a build failure, not a style nit:

| Zig side | C side |
|---|---|
| type `Foo` | `ZJoltFoo` |
| function `zjoltFoo` | `zjoltFoo` |
| constant `foo_bar` | `ZJOLT_FOO_BAR` |
| enum `Foo`'s field `bar` | `ZJOLT_FOO_BAR` |

**One verb per ownership model, and the verb is the documentation.** A handle
zjolt allocates and frees outright is `zjoltFooDestroy` in C and `deinit` in
Zig: the first drop is the last one, and a second is a double free. A handle
that is a Jolt `RefTarget` is `zjoltFooRelease` and `release`: there may be
other holders, and dropping yours need not free anything. Never give a
reference-counted type a `deinit` alias, however well it reads at a `defer` —
the whole value of the two verbs is that the call site says which model
applies without going to look.

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

**Never write `AddRef()` or `Release()`.** Three helpers in
`ffi/zjolt_internal.h` are the only places a Jolt reference count moves on the
host's behalf, and `ci/check-refcounts.sh` fails the build on a bare call
anywhere else in `ffi/`:

- **`zjolt::Own(fresh)`** hands a newly constructed object out. The arithmetic
  is not what anyone expects — a fresh `JPH::RefTarget` starts at **zero**, not
  one, so the caller's reference is the first.
- **`zjolt::HostRetain(held)`** hands out a reference to an object that already
  has holders: a `zjoltFooAddRef` entry point, a getter that returns something
  the caller must release, or `Finish` in `ffi/zjolt_shape.cpp` compensating
  for a `JPH::Ref` that drops at scope exit.
- **`zjolt::HostRelease(held)`** is the counterpart, and the body of every
  `zjoltFooRelease`.

Each of the three also moves `zjoltLiveHandleCount`, which is the point: an
unreleased handle has to be a refusal at `zjoltDeinit` rather than a leak
nothing reports. Getting the Jolt half right and the count half wrong is
silent in both directions — under-counting wraps to `0xFFFFFFFF` and the
object outlives its own frees, and an `AddRef` that skips the handle count
while its `Release` does not drives the total negative, which reads as
"nothing outstanding".

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

`ci/run.sh` also runs `ci/check-examples.sh` and `ci/check-numbers.sh`, so an
example or a count that this change moves fails there rather than in review.

`ci/run.sh --full` executes the configurations that change ABI widths
(`-Ddouble_precision`, `-Dobject_layer_bits=32`), which a compile alone does
not prove.

## Decisions that stay settled

Recorded so the question is not re-opened; the reasoning lives next to the
code, and each entry is the pointer to it.

- **The C declarations are one module per public header.** `src/c.zig` was a
  single file every subsystem appended to. It is now the list of 25 modules
  under `src/c/`, and that list is what `src/abi_check.zig` and
  `src/misuse_sweep_test.zig` walk — so a module missing from it is a module
  neither guard covers, which `ci/check-abi-drift.sh` mutates to prove.
- **Coverage is computed, not judged.** `tools/coverage.sh`'s raw percentage
  used to over-count, because its denominator held Jolt internals that must
  never cross. It is no longer the measure. `tools/classify.sh` proves an
  exclusion three ways — an upstream internal marker, no public declaration,
  or a scaffolding macro whose only `#define` in the tree is commented out —
  and `ci/check-coverage.sh` recomputes that proof on every run and rejects
  any `INTERNAL` line it cannot justify. Every other verdict is checked
  against the tree too: a `BOUND` or `EXTENSION` names entry points that must
  exist in `ffi/*.h`, a `ZIG` names a declaration that must exist at that
  path, a `LANGUAGE` names a facility from a closed list.
- **Numbers in documents no longer rot.** `ci/check-numbers.sh` recomputes
  every count the README, `UPSTREAM.md` and `ci/run.sh` state and fails when
  one disagrees with the tree. Its own header names what it cannot reach: a
  count written in words rather than digits, and a sentence that is wrong
  around a number that is right.
- **Accessibility is checked through every enclosing scope.**
  `tools/jolt_access.awk` used to consult only the innermost class, so a
  public member of a privately nested class read as public — Jolt's
  `CharacterVirtual::ContactCollector` is the case that exposed it. Sixty
  names across the tree were affected. `tools/jolt_internal.awk` also learned
  Jolt's sixth marker form, `INTERNAL CLASS DO NOT USE!`, which
  `SimShapeFilterWrapper.h` uses and nothing else does.
- **`ZJOLT_SHAPE_SUB_TYPE_OTHER` is gone, but a "kind I cannot name" value is
  not.** It stood for two different facts. Both are now named: `NONE` is zero
  and means the handle was NULL, and `USER_DEFINED` means a real shape from one
  of Jolt's sixteen `User*` slots, registered outside this library. Collapsing
  the second into the first would have `zjoltShapeGetSubType` answer "not a
  shape" about a shape. See the enum's own comment in `ffi/zjolt_core.h`.
