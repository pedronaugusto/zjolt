# zjolt

[![CI](https://github.com/pedronaugusto/zjolt/actions/workflows/ci.yml/badge.svg)](https://github.com/pedronaugusto/zjolt/actions/workflows/ci.yml)

Zig bindings for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — the
whole runtime, from shapes and the step to constraints, characters, vehicles,
ragdolls, soft bodies and hair, in a package with no renderer, no entity
system and no clock attached.

- Vendored, pinned upstream Jolt (5.6.0). No fork, no patches. See
  [UPSTREAM.md](UPSTREAM.md).
- A real C ABI (`ffi/zjolt.h`) that stands on its own — the Zig wrapper is one
  consumer of it, not its only reason to exist.
- **Callbacks are C function pointers, not C++ vtables.** Layer filters and
  contact listeners cross the boundary as plain function-pointer tables, so a
  host writes no `callconv(.c)` and needs no per-ABI special cases.
- Host allocator injection: every Jolt allocation can go through your
  `std.mem.Allocator`.
- Drift between the C header and the Zig externs **fails the build**, not
  production: a test compares the two by reflection, with nothing listed by
  hand. 13 kinds of deliberate drift are verified to fail it, including a
  field swap that leaves every offset in the struct unchanged, and
  13 mutations for the other guards do the same, each naming the test that
  has to catch it.
- Jolt asserts where a library for a service would return, and several of those
  assertions sit on paths an ordinary caller reaches. Each one this ABI could
  reach has been turned into a returned error, with a test that fails if the
  guard is removed.

Status: **v0.2.0.** Every Jolt subsystem is bound and the surface is complete
enough to build against. Still pre-1.0, so naming and shape can change between
minor versions — but a change will be a change, not a silent one: the ABI
cross-check makes any drift between the header and the Zig side a build
failure, and there are no compatibility aliases anywhere, so there is exactly
one spelling of everything.

Working today: every Jolt subsystem, across **1449 C entry points** — shapes,
bodies, the step, queries, constraints, both character kinds, vehicles,
ragdolls, soft bodies, hair, state save and restore, and debug draw. See
[Scope](#scope) for what that covers and what is deliberately left out.

## Usage

Every line below is quoted from `tests/consumer/src/main.zig`, which is built
through `b.dependency` and RUN — `zig build run` inside `tests/consumer`,
which `ci/run.sh` does — the way a downstream project builds it. `ci/check-examples.sh` fails if these blocks and that file
ever stop matching character for character, so the example a reader copies is
the example the suite compiles.

`tests/consumer/src/main.zig`
```zig
// Which layers exist, and what collides with what. Plain Zig functions.
const Layers = struct {
    pub const static: zjolt.ObjectLayer = 0;
    pub const moving: zjolt.ObjectLayer = 1;

    pub const bp_static: zjolt.BroadPhaseLayer = 0;
    pub const bp_moving: zjolt.BroadPhaseLayer = 1;

    pub fn broadPhaseLayerCount() u32 {
        return 2;
    }

    pub fn broadPhaseLayerFor(layer: zjolt.ObjectLayer) zjolt.BroadPhaseLayer {
        return if (layer == static) bp_static else bp_moving;
    }

    pub fn objectCanCollideWithBroadPhase(
        object: zjolt.ObjectLayer,
        broad: zjolt.BroadPhaseLayer,
    ) bool {
        return if (object == static) broad == bp_moving else true;
    }

    pub fn objectsCanCollide(a: zjolt.ObjectLayer, b: zjolt.ObjectLayer) bool {
        return if (a == static) b == moving else true;
    }
};
```

`tests/consumer/src/main.zig`
```zig
pub fn main() !void {
    var gpa_state = std.heap.DebugAllocator(.{}){};
    defer std.debug.assert(gpa_state.deinit() == .ok);
    const gpa = gpa_state.allocator();

    try zjolt.init(.{ .allocator = gpa });
    defer zjolt.deinit();

    const jobs = try zjolt.JobSystem.initThreadPool(.{});
    defer jobs.deinit();

    const system = try zjolt.PhysicsSystem.init(.{ .layers = zjolt.layersFromType(Layers) });
    defer system.deinit();

    const shape = try zjolt.Shape.initSphere(0.5, .{});
    defer shape.release();

    const ball = try system.bodies().createAndAdd(.{
        .shape = shape,
        .object_layer = Layers.moving,
        .position = zjolt.rvec3(0, 10, 0),
    }, .activate);

    // Per frame:
    var frame: usize = 0;
    while (frame < 60) : (frame += 1) {
        const update_error = try system.step(1.0 / 60.0, 1, jobs);
        if (update_error.contact_constraints_full) return error.ContactConstraintsFull;
    }

    // The ball began at y = 10 and gravity is the whole point, so this is
    // what says the library linked here really stepped.
    const transform = system.bodies().getTransform(ball);
    if (!(transform.position.y < 10)) return error.BallDidNotFall;

    try reportBuild();
}
```

`reportBuild` is that file's own last line and not part of the recipe: it
checks that the build options crossed the module boundary, which is something
only a consumer can check.

Add it as a dependency and link the module:

```zig
const zjolt_dep = b.dependency("zjolt", .{ .target = target, .optimize = optimize });
exe.root_module.addImport("zjolt", zjolt_dep.module("zjolt"));
```

Or link the C library directly, from Zig or from a C or C++ host — the header
is a real contract, not a private detail of the wrapper:

```zig
exe.root_module.linkLibrary(zjolt_dep.artifact("zjolt"));
```

Both are exercised by `tests/consumer`, which builds zjolt through
`b.dependency` the way a downstream project does. That is a different code path
from building it in-repo, and the difference is invisible to the rest of the
suite.

New surface follows one recipe, written down in [BINDING.md](BINDING.md): which
five files a subsystem touches, the naming the ABI cross-check pairs on, the
entry-point guard, reference counting, and the rule that nothing may unwind out
of a callback.

## Coming from Jolt

Every entry point is named after the Jolt method it calls, so the C++ you
already know maps mechanically:

    JPH::BodyInterface::GetLinearVelocity   ->  zjoltBodyGetLinearVelocity
    JPH::Shape::GetSubType                  ->  zjoltShapeGetSubType
    JPH::HingeConstraint::SetTargetAngle    ->  zjoltHingeConstraintSetTargetAngle
    JPH::CharacterVirtual::ExtendedUpdate   ->  zjoltCharacterUpdate

The rule is `zjolt` + the type + the method, and it is enforced rather than
merely intended: `src/abi_check.zig` pairs the two sides of the ABI by name
with no hand-written list, so a name that breaks the convention is a build
failure. Two deliberate departures from a literal transliteration:

- **The interface is dropped where Jolt has only one.** `BodyInterface`'s
  methods are `zjoltBody*`, not `zjoltBodyInterface*`, because there is nothing
  else a body method could go through. `PhysicsSystem` keeps its name because
  its methods are about the system rather than about a body.
- **Overloads become distinct names**, since C has none. Jolt's locking and
  non-locking pairs are spelled with a `Locked` suffix on the form that
  requires you to already hold the lock: `zjoltBodyGetLinearVelocity` takes the
  lock for you, `zjoltBodyGetLinearVelocityLocked` does not.

The Zig wrapper drops the prefix and the type, because the receiver supplies
both: `system.bodies().getLinearVelocity(body)`. Anything it does not wrap is
reachable through `zjolt.c`, one namespace per header — `zjolt.c.body`,
`zjolt.c.constraint`, `zjolt.c.query` — and those raw declarations are
first-class rather than a fallback.

### Where this differs from joltc

[joltc](https://github.com/amerkoleci/joltc) is the C API most other Jolt
bindings build on, and it is a reasonable thing to compare against. The
difference that matters is not spelling:

|  | joltc | zjolt |
|---|---|---|
| entry points returning an error code | none | 774 |
| entry points returning `void` | the large majority | 381 |
| build-configuration handshake | none | `zjoltAbiLayout` + config id |
| public headers | one | 25, one per subsystem, behind one umbrella |

The joltc column is `include/joltc.h` at commit `886e088`, dated 2026-07-13
and read on 2026-08-30: 1269 `JPH_CAPI` declarations, 728 of them returning
`void`, 109 returning `bool`, and none returning an error code — joltc has no
result enum. Those four figures are another project's tree, so no gate here
re-measures them; zjolt's own column is gated by `ci/check-numbers.sh`.

joltc reports failure implicitly: a bad argument trips a Jolt assertion in a
build that has them and does nothing in a build that does not. This package
turns every Jolt precondition an ordinary caller can reach into a returned
result, and refuses at `zjoltInit` a library whose layout-affecting build
settings differ from the caller's — because `ZJoltReal` and `ZJoltObjectLayer`
change width with those settings, and nothing else would notice.

What joltc does better is machine-readability: `JPH_PhysicsSystem_Create`
separates type from method with an underscore, which suits automatic binding
generators. `zjoltPhysicsSystemCreate` does not, and that is a real trade for
matching the camelCase convention Vulkan and the Zig side use.

## Design

### Callbacks cross as C, not as C++

Jolt asks the host questions through abstract C++ classes: which broad-phase
layer an object layer lives in, whether two layers collide, what to do about a
contact. A binding has two ways to carry that across a language boundary.

One is to mirror the C++ vtable into the target language and hand Jolt a
pointer to it. It works, and it makes vtable layout part of your ABI — so the
host ends up writing per-ABI variants of its own callbacks, because the
Microsoft convention returns a small struct through a hidden pointer and the
SysV one does not. That is a real bug class, and it lands in the *consumer's*
code, where it is hardest to notice.

zjolt does the other thing. Every interface is a plain function-pointer table:

```c
typedef struct ZJoltBroadPhaseLayerInterface {
  uint32_t (*num_broad_phase_layers)(void *user);
  ZJoltBroadPhaseLayer (*broad_phase_layer_for_object_layer)(void *user, ZJoltObjectLayer layer);
  const char *(*broad_phase_layer_name)(void *user, ZJoltBroadPhaseLayer layer);
  void *user;
} ZJoltBroadPhaseLayerInterface;
```

The C++ side implements the real `JPH::BroadPhaseLayerInterface` and forwards.
C++ vtable layout stops being ABI, and `zjolt.layersFromType` turns an ordinary
Zig struct into the table at comptime — no `callconv(.c)` in host code, no
branch on the target's ABI, on any platform. `user` is what makes the same
table serve a scheme that is not known until run time: `zjolt.layersFromInstance`
takes a value instead of a type, passes it as each method's first argument, and
is otherwise the same three tables.

The same shape covers `ObjectVsBroadPhaseLayerFilter`, `ObjectLayerPairFilter`,
`ContactListener`, `BodyActivationListener` and the four query filters.

### Queries stream

Jolt finds hits with a *collector* — an object it calls once per hit, as it
finds them. A binding can either surface that or hide it behind "give me all
the hits", and hiding it costs more than it looks: the caller's buffer is not
the expensive part. `AllHitCollisionCollector` accumulates into an unbounded
array, and a `CollideShapeResult` is around a kilobyte, because it embeds two
`StaticArray<Vec3, 32>` faces whether or not faces were asked for. Answering a
count and then a fill that way is two traversals and two copies of every hit.

So the collector is what zjolt exposes, and the other two forms are built on
it:

```zig
var nearby: struct {
    count: usize = 0,
    pub fn onHit(self: *@This(), hit: zjolt.CollideShapeHit) zjolt.HitAction {
        if (hit.penetration_depth > 0.1) self.count += 1;
        return .@"continue";
    }
} = .{};
try system.queries().collideShapeEach(.{ .shape = probe, .position = here }, null, &nearby);
```

Closest-hit and count-then-fill are two more sinks over the same traversal,
so nothing accumulates in any of them and the three cannot drift apart.

The callback returns an **enum**, never a fraction. Jolt's collectors assert
that the early-out fraction only ever decreases, and with assertions compiled
out an increasing one violates a precondition the narrow phase relies on. The
narrowed value is computed from the hit on the C++ side, where it cannot be got
wrong.

`onHit` may fail. It may not fail *into Jolt*: it runs inside the traversal
with a broad-phase read lock held, so the wrapper stashes the error, stops the
query, and raises it once every lock has been dropped. A caller writes `try`.

### The frame loop has its own path

Jolt names bodies by id, and reading one body's transform takes a body lock.
That is the right shape for the occasional query and the wrong shape for what a
renderer does every frame, which is read the transform of every body that
moved — one ABI crossing and one lock per body, thousands of times.

So there is a second path for exactly that:

```zig
var ids: [1024]zjolt.BodyId = undefined;
const awake = try system.getActiveBodies(.rigid_body, &ids);  // what moved
_ = try system.getTransforms(awake, &positions, &rotations);
```

Bit masks arrive as Zig types rather than integers — `update_error` above is a
`packed struct(u32)`, and a body's degrees of freedom are
`.allowed_dofs = .plane_2d` rather than an `@enumFromInt` of an OR. Both are
layout-identical to the C enums they mirror; the C header spells them as
enumerators only because C has no better way to name bits.

`getTransforms` walks the ids in chunks under `BodyLockMultiRead`, which
computes one mutex mask for a whole batch. Per-frame read-back becomes two
crossings and a handful of lock acquisitions instead of 2N of each. An id whose
body was destroyed since the step is reported through a count rather than
failing the batch, because that is ordinary rather than exceptional.

### Allocator injection, honestly scoped

`init(.{ .allocator = gpa })` routes every Jolt allocation through a
`std.mem.Allocator`. It is process-wide, because
[Jolt's own allocator is](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Jolt/Core/Memory.h) —
five global function pointers — and that is surfaced rather than hidden behind
a per-system parameter that could not be honoured.

Two wrinkles are worth knowing. Jolt frees with `free(block)` and
`aligned_free(block)`, no size and no alignment, while Zig requires both;
`src/memory.zig` bridges that with a header stored ahead of each block.
And Jolt's plain `allocate` takes no alignment argument yet puts SIMD types in
what it returns — the minimum it actually needs is read from
`zjoltDefaultAllocateAlignment()` rather than assumed to be 16, because on a
32-bit target it is 8.

The C API keeps Jolt's shape, so a plain C host can pass `malloc`/`free` in a
few lines; `tests/c_smoke.c` does exactly that, and fails if a single byte is
outstanding at the end.

### Validation at the boundary

Jolt is a library for a game engine, which means it asserts where a library for
a service would return. Several of those assertions sit on paths an ordinary
caller reaches, and in a build with asserts on they abort the process.

One kind is arithmetic drift. `Mat44::sRotation` asserts its quaternion is
unit length, and a body's rotation reaches it on every step — so a rotation
integrated over a few thousand frames eventually kills the process. Rotations
are renormalised as they cross the boundary; one that cannot be normalised at
all becomes the identity, because a visible failure to rotate is easier to find
than a silent rotation by NaN. Jolt's setters that assert on a static body get
the same treatment: they do nothing rather than abort. So does `max_bodies`
past the range of a Jolt body id, which otherwise asserts much later, while
handing one out.

A second kind is Jolt's own diagnostics. Its default trace function is a stub
whose body is `JPH_ASSERT(false)`, so an application that never installs one
dies the first time Jolt has something to say. zjolt installs a trace thunk
unconditionally and falls back to stderr, so a missing hook costs you a line of
text rather than the process.

One it does *not* work around, because the judgement is upstream's to make:
`PhysicsSystem::Update` asserts that its error mask is empty one line before
returning it. The mask exists so a host can notice it has under-provisioned a
limit and degrade; in a build with asserts on, that condition breaks into the
debugger first. It is documented where it is returned, and the suite reads the
mask by installing an assert hook that declines the breakpoint — which is what
that hook is for.

The last kind is deserialisation, and it is the interesting one. Jolt is otherwise
well-behaved about its own reads — a shape restore tests `IsEOF()` and
`IsFailed()` after every stage. There is one exception.

`Shape::sRestoreFromBinaryState` reads the shape's type from the stream and
passes it straight to a table lookup **before** checking whether that read
succeeded. A byte outside the enum's range is an out-of-bounds index. Handing
`zjoltShapeRestore` a buffer of ordinary text reproduces it.

So a saved shape here is Jolt's payload behind a 32-byte container: a magic
tag, a container version, the library's config id, the Jolt version, the
payload length and a CRC-32. Restore validates all of it before Jolt sees a
byte, which turns four different kinds of wrong input into four different clean
errors:

- not a shape at all → rejected on the tag
- written by a different Jolt, or a different precision setting → rejected on
  the stamp, rather than reinterpreted into a plausible-looking wrong shape
- truncated, or with trailing bytes → rejected on the length
- damaged in storage → rejected on the checksum

Underneath, Jolt's own truncation handling is exercised deliberately rather
than assumed: a test rebuilds the container around *every* truncated prefix of
a real payload, so the parser genuinely runs out of input each time. All of
them are refused.

What this does **not** claim: it is not a defence against a crafted payload
that carries a matching checksum. A shape cache is something your own cook
wrote to local disk, and should be treated that way.

### The ABI guard

The Zig side hand-writes `extern struct`s mirroring `zjolt.h`. Nothing in
either compiler checks those two declarations still agree — a field reordered
on one side and not the other is silent corruption, not a build error.

A test `@cImport`s `zjolt.h` and compares the two namespaces declaration by
declaration: every struct field paired **by name** with its offset, every
function's parameter count and variadic-ness, every enumerator's value, every
constant. Wherever a type crosses — field, parameter, return value — it is
compared by size, alignment **and scalar identity**: signedness, and
int-versus-float. Two integers of the same width are interchangeable to
`@sizeOf`, so a `uint32_t` declared as `i32` passes a layout comparison and
then reinterprets every value above 2^31. Nothing is listed by hand — the
check discovers what to compare by reflecting over `c.zig`, and a declaration
it cannot classify is a compile error rather than a silent pass, so it cannot
quietly stop covering something. The `@cImport` is test-only; the shipped
module never runs translate-c.

Signedness is the one comparison skipped across an enum, and skipped for a
reason rather than out of tolerance: C leaves an enum's underlying type to the
implementation, and the implementations disagree — clang and gcc pick an
unsigned type when no enumerator is negative, MSVC uses `int`. Comparing it
would fail a correct binding on one toolchain and pass it on another. What
makes skipping it safe is a precondition, and the check asserts the
precondition instead of assuming it: **no enumerator in this ABI may be
negative**, because that is exactly when the implementation's choice becomes
observable.

Pairing fields *by name* is the part that matters. Two same-sized adjacent
fields swapping places leaves the sequence of offsets identical, so a positional
comparison — or a digest folded over offsets alone — passes a swap that
reinterprets both fields.

A check that guards everything else cannot be trusted on its own word: a
refactor that quietly makes it vacuous looks exactly like a passing build. So
`ci/check-abi-drift.sh` applies 13 kinds of deliberate drift one at a time —
that swap, a dropped parameter, a widened parameter, a renumbered enumerator, a
narrowed enum tag, a moved mask bit, a drifted constant, an extern deleted from
the Zig side, a field added to the header alone, a field's signedness flipped, a
negative enumerator, an extern replaced by a Zig helper wearing the same name,
and a module dropped from `src/c.zig`'s list — and asserts each is refused
with a `zjolt ABI drift:` message.

The same script mutates the other seven guards, since a guard nothing tests is
a guard nobody has checked: the entry-point preamble that turns a call made
before `zjoltInit` into a result rather than a walk through an uninitialised
allocator, the allocator seam, the callback error path that stashes a failure
instead of unwinding across a Jolt callback, the analysis sweep that forces Zig
to look at wrappers nothing calls, the coverage classifier, and the two guards
over the documents — `ci/check-numbers.sh` and `ci/check-examples.sh`, which
are mutated by editing a document rather than a source. Each of those declares
the signal that must appear, so a mutation that fails for an unrelated reason
is reported as a wrong failure rather than counted as the guard doing its job.
26 mutations in all, none missed, and `ci/check-numbers.sh` fails the build if
that count and this sentence drift apart. It runs under `ci/run.sh --full`.

Its limit is honest: translate-c renders every C pointer as `[*c]T`, so pointee
types are compared only by size and alignment — a `float *` declared as `*i32`
would pass. `tests/c_smoke.c` drives the same scenarios through the header
itself, which is what covers that residue.

That check compares this build's externs against this build's *header*. It says
nothing about whether the *library* was compiled from the same header with the
same macros — and `ZJoltReal` and `ZJoltObjectLayer` change width with those
macros. `zjoltAbiLayout()` reports what the library actually is, and
`zjoltInit` refuses a caller whose `ZJOLT_CONFIG_ID` disagrees.

In the other direction, `static_assert`s in `ffi/zjolt_abi.cpp` fail the
**build** if a vendored Jolt upgrade changes a type, a constant, or an
enumerator's *value*. That last one matters more than it sounds: the
conversions are switches over Jolt's enumerator names, so a renumbering
upstream would compile perfectly and quietly start meaning something else.

### Build hygiene

- Source lists are explicit, never globs — a re-vendor cannot silently change
  what compiles.
- No `-fno-access-control`. The FFI layer uses only Jolt's public API, so it
  has no reason to defeat C++ access checking and no coupling to Jolt
  internals.
- UBSan is **not** blanket-disabled, and it is **not** forced on consumers
  either. `-Dsanitize_c` is off by default, because Zig's sanitizer emits calls
  into a runtime linked only into a compilation that is itself sanitized — a
  consumer who forgets to forward `optimize` would get a link failure naming
  `__ubsan_handle_shift_out_of_bounds` and nothing they could act on. zjolt's
  own Debug runs pass `-Dsanitize_c=true` explicitly, so real undefined
  behaviour in zjolt's C++ still surfaces.
- Build options are declared once and mirrored into a Zig `options` module, so
  the wrapper cannot disagree with how the C++ was compiled — and
  `zjoltInit` checks the two at run time as well, because Jolt reacts to that
  particular disagreement by calling `std::abort()`. See
  [UPSTREAM.md](UPSTREAM.md).
- One translation unit per concern on both sides of the boundary.

## Build options

| Option | Default | Effect |
|---|---|---|
| `-Ddouble_precision` | `false` | World positions become `f64` (`JPH_DOUBLE_PRECISION`), for worlds too large for float precision. Changes the ABI. |
| `-Dobject_layer_bits` | `16` | Width of an object layer, 16 or 32. Changes the ABI. |
| `-Dcross_platform_deterministic` | `false` | Trades speed for bit-identical results across platforms. |
| `-Denable_asserts` | on in Debug | Keeps Jolt's internal assertions. |
| `-Dsanitize_c` | `false` | Compiles the C and C++ with Zig's undefined-behaviour sanitizer. Off by default so the sanitizer runtime is never forced into a consumer's link; zjolt's own Debug runs turn it on. |
| `-Dshared` | `false` | Builds the C library as a shared object. |
| `-Ddebug_renderer` | `false` | Compiles Jolt's debug-draw geometry collection (`JPH_DEBUG_RENDERER`). |
| `-Dprofile` | `false` | Compiles Jolt's profiler (`JPH_PROFILE_ENABLED`), which instruments every `JPH_PROFILE` scope in Jolt's own source. |
| `-Dcpu_compute` | `true` | Compiles Jolt's CPU compute backend — what the hair solver runs on when the host lends no device. |
| `-Dobject_stream` | `true` | Compiles Jolt's reflective object stream (`JPH_OBJECT_STREAM`) and its text and binary editor format. |
| `-Dtrack_simulation_stats` | `false` | Tracks Jolt's per-body simulation stats (`JPH_TRACK_SIMULATION_STATS`). |
| `-Dtrack_broadphase_stats` | `false` | Tracks Jolt's broad-phase query stats (`JPH_TRACK_BROADPHASE_STATS`). |
| `-Dtrack_narrowphase_stats` | `false` | Tracks Jolt's per-shape-pair narrow-phase timing stats (`JPH_TRACK_NARROWPHASE_STATS`). |
| `-Dexternal_profile` | `false` | Routes Jolt's profile scopes to a host profiler (`JPH_EXTERNAL_PROFILE`) instead of Jolt's own. Mutually exclusive with `-Dprofile`, and wins over it. |

Exactly 2 rows change the ABI — the two marked "Changes the ABI." above — and
each contributes one bit to `ZJOLT_CONFIG_ID`, alongside the version. Nothing
catches a mismatch at link time: the symbol names are identical either way, so
a consumer built against a differently configured header would link cleanly
and then read a different set of types than the library writes. `zjoltInit`
compares the two ids and returns `ZJOLT_RESULT_CONFIG_MISMATCH` before any of
that can happen, which is why the header's `zjoltInit` is a `static inline`
wrapper that passes the CALLER's `ZJOLT_CONFIG_ID` rather than the library's.

The rest are compile-time scope: **every entry point is declared in every
build.** One a flag turns off still exists and returns
`ZJOLT_RESULT_UNSUPPORTED`, so a caller never has to `#ifdef` around a
declaration, and a Zig caller never sees a missing symbol.

### The instruction set is the target's, not an option

There is deliberately no `-Dsimd`. Jolt derives every `JPH_USE_*` from the
compiler's own predefines — `__AVX2__`, `__SSE4_2__`, `__F16C__`, `__FMA__`
([`Jolt/Core/Core.h`](libs/JoltPhysics/Jolt/Core/Core.h)) — so the lever
already exists and it is the Zig target's CPU model. A zjolt option that
defined `JPH_USE_AVX2` without also raising that model would turn on
intrinsics the compiler is not permitted to select; one that raised both
would be a second home for a fact the target already holds.

Which means a CROSS build is a baseline one: `-Dtarget=x86_64-linux-gnu`
resolves to the `x86_64` model, which is SSE2, and Jolt's AVX2 paths are
compiled out of it. A native build takes the host CPU instead, so on a host
that has AVX2 `zjolt.cpuFeatures().avx2` is true with no options at all.
Raise a cross build the way any Zig build does:

```
zig build -Dcpu=x86_64_v3            # AVX2, F16C, FMA, BMI, LZCNT
zig build -Dtarget=x86_64-linux-gnu -Dcpu=x86_64_v2
```

What is not acceptable is that this be invisible, which it was.
`zjolt.cpuFeatures()` (C: `zjoltCpuFeatures`) reports the `JPH_USE_*` set the
LIBRARY was compiled with, read from inside it, one flag per Jolt macro. It
is not part of `ZJOLT_CONFIG_ID`: an instruction set changes speed, never a
layout, so a consumer compiled for a different one still agrees about every
struct and `zjoltInit` has no reason to refuse it.

`JPH_DISABLE_CUSTOM_ALLOCATOR` is not an option either, and that one never
will be: it compiles away the five function pointers
[`Jolt/Core/Memory.h`](libs/JoltPhysics/Jolt/Core/Memory.h) declares, which
are the whole of `init(.{ .allocator = ... })`. Exposing it would leave that
argument accepted and ignored.

## Testing

```sh
zig build test                       # the Zig suite, and the C smoke test with it
zig build test-c                     # the C smoke test alone
zig build --build-file tests/consumer/build.zig run   # as a downstream dependency
```

The suite is self-contained: every fixture is built in code, so it ships no
third-party assets and cannot drift out of version-sync with the vendored
library.

Every Zig test runs the whole library through `std.testing.allocator`, which
fails on a leak — that is the allocator seam's assertion, not a separate one.
`zig build test-c` proves the same property from the other side: the C smoke
test drives the whole exposed surface through the header alone, with a counting
`malloc`/`free` allocator, and fails if a single byte is outstanding at the
end.

What the tests actually assert, beyond "it compiles": a body falls
under gravity and comes to rest on the floor at the right height and goes to
sleep; the same inputs step to a bit-identical state twice; a kinematic body
moves toward its target while a teleport ignores what is in the way; an impulse
changes velocity immediately and a force does not; contact and activation
callbacks fire with the right bodies, and stop when cleared; rays and sweeps
hit what is there and miss what is not, with and without filters; a streaming
query sees exactly the hits a fill query collects, stops when told to, and
leaves the system steppable after its callback fails; the bulk
read-back agrees exactly with the per-body accessors; a locked body reads and
writes; a rotation that is not unit length is renormalised rather than fatal; a
body constrained to a plane stays in it; a step that runs out of contact
constraints says which cache filled; `deinit` refuses while a handle is alive
and traces why; a character settles on the floor, reports the body under it,
walks, crouches, and climbs a step within its stride; a hinge holds its axis
and its position motor drives toward a target angle; a ragdoll settles under
gravity, and releasing one that is still in the world takes it out before
destroying it rather than leaving the broad phase holding freed bodies; and a
character on a ramp is supported by it while one pressed against a wall is
not.

The bar is one characteristic test per subsystem, never one per entry point.
These cover the parts a caller most easily gets subtly wrong: a box's mass and inertia
follow from its dimensions and density; a compound's centre of mass sits where
its children put it rather than at its origin; a material attached to one mesh
triangle comes back through a ray hit as that same material and not its
neighbour's; two bodies excluded by a collision-group filter pass through each
other while the same pair in different groups do not; `collideShapeClosest`
returns the DEEPEST overlap rather than whichever the traversal reached first;
a shape cast that starts inside a mesh reports nothing under the default
back-face mode and a hit under the other; an aborted body batch leaves the
system exactly as it was; a vehicle accelerates along its forward axis, spins
every wheel, upshifts as engine RPM climbs, and brakes to rest; a tracked
vehicle turns when its two tracks run at different rates; and a world saved,
stepped further, and restored is back where the save was taken — position and
velocity both — while a restore into a world whose body set changed is refused
rather than half-applied.

3 reflective sweeps run alongside those, and they are mechanical on purpose:
one calls every entry point in the ABI with nulls, one calls every entry point
before `init`, and one hands every enum parameter a value no enumerator names.
All three discover the list by reflection rather than being handed it, so an
entry point added tomorrow is swept without anyone remembering to add it. They
prove nothing about whether the physics is right — that is what the tests
above are for — but they prove no entry point can be reached into a crash.

Both diagnostic hooks are exercised, which took some doing — every path that
would have reached one has been turned into a returned error, so the two that
remain are deliberate: the trace hook fires on a refused `deinit`, and the
assert hook on the update-error mask described above.

### Continuous integration

CI runs the whole suite on **Linux, macOS and Windows**, in four optimize modes
with the sanitizer both on and off, plus the standalone C test. Separately it
executes five build configurations and builds a sixth, cross-compiles eight
further targets, checks the header against the externs, and re-fetches the
pinned upstream commit to prove `libs/` is unmodified. See
[`.github/workflows/ci.yml`](.github/workflows/ci.yml).

The same matrix runs locally, so a failure is reproducible on your machine
before it is a red mark on a pull request:

```sh
ci/run.sh            # the inner loop: hygiene, native Debug, the C test
ci/run.sh --full     # everything CI runs, minus the jobs that need network
ci/install-hooks.sh  # run the inner loop automatically before every push
```

The default is trimmed rather than complete, which is a concession to what Jolt
is: 179 translation units per configuration, so one is tens of seconds rather
than the couple of seconds a smaller library would take. `--full` is 28 checks
— 20 of them run on this host, 8 cross-compiling — which is minutes from a
cold cache; the default is under a minute once the cache is warm.

`ci/measurements.sh` recomputes every number this README states, and
`ci/check-numbers.sh` FAILS THE BUILD when one of them disagrees with the
tree. The second script names, in its own header, which numbers it gates and
which it cannot: a count written in words instead of digits is beyond it, and
so is a sentence that is wrong around a number that is right.

### Platform coverage

| | Suite executed by CI | Compile-checked by CI |
|---|---|---|
| Linux | x86_64 (glibc) | + aarch64, musl |
| macOS | aarch64 | + x86_64 |
| Windows | x86_64, both gnu and MSVC ABI | + aarch64 |

Compiling proves the sources and build graph are portable; only an executed
configuration proves behaviour, which is why the two are separate jobs.

That table describes the matrix, not a promise: **the badge at the top of this
file is the authority on whether those runs have actually happened and
passed.** At the time of writing the suite has been executed by hand on
macOS/aarch64 only, and the Windows MSVC configuration has never been executed
at all.

Also not claimed: nothing here has been benchmarked. The bulk read-back path
exists because 2N ABI crossings per frame is the wrong shape, not because a
measurement said so.

## Scope

Every Jolt subsystem is bound: **1449 C entry points** across **25 headers**,
each one mirrored by a Zig wrapper that a reflective cross-check pairs at build
time. Not one entry point is stranded — `tools/zig_surface_exceptions.txt` is
empty, and `ci/check-coverage.sh` fails both if an entry point loses its Zig
caller and if an excuse is written for one that has a caller after all.

**13 entry points are native rather than wrapped**: value math — a quaternion
product, a matrix transform, a lerp — that the Zig side computes itself,
because a cross-TU call cannot be inlined and costs more than the arithmetic
inside it. Two implementations of one formula is an arrangement this package
otherwise refuses, so each is a written obligation in `tools/zig_native.txt`:
the Zig declaration that computes it, and the test that compares its answer
with the entry point's. `ci/check-coverage.sh` fails if either goes missing,
and fails the other way too if Zig starts calling the entry point after all.

- **Shapes** — every kind Jolt can construct: the convex primitives (box,
  sphere, capsule, cylinder, triangle, tapered capsule, tapered cylinder,
  convex hull), mesh, height field, plane, empty, both compounds, and the
  decorated shapes (scaled, rotated-translated, offset-centre-of-mass). A
  mutable compound whose children move, are added and are removed at run time.
  Introspection, mesh and height-field read-back, and binary save and restore
  for a collision cook.
- **Physics materials**, per triangle on a mesh and per quad on a height
  field, resolved into every query hit.
- **The world** — a physics system with broad-phase and object layers and
  their filters, the step with a job-system seam, contact and activation
  listeners, step listeners, and the combine callbacks for friction and
  restitution.
- **Bodies** — create, destroy, add, remove, motion type, teleport, kinematic
  move, activate, velocities, forces and impulses, shape swap, user data,
  collision groups, and read/write body locks. Batched add and remove for
  bulk work.
- **Queries** — ray casts, shape casts, overlap and point tests, in
  closest-hit, all-hits and streaming forms, with broad-phase layer, object
  layer, body and shape filters on every one. Plus `TransformedShape`: the
  same queries against a single placed shape, with no physics system at all.
- **Constraints** — all twelve kinds (fixed, point, hinge, slider, distance,
  cone, swing-twist, six-DOF, gear, rack and pinion, pulley, path), with their
  motors, limits, springs and run-time state.
- **Characters** — `CharacterVirtual` with ground state, stair walking, shape
  swapping, contact listeners and character-versus-character collision; and
  the rigid `Character`.
- **Vehicles** — wheeled and tracked, with the drivetrain (engine,
  transmission, differentials), wheel pose and force read-back, and manual or
  automatic gears.
- **Ragdolls** — skeletons, poses, ragdoll settings and instances, driving to
  a pose kinematically or with motors, and the body ids to map a contact back
  to a limb.
- **Soft bodies** and **hair**, including the compute-backend seam hair needs.
- **State save and restore**, whole-system or one body at a time, for
  rollback, replay and determinism checks.
- **Debug draw**, as arrays of lines, triangles and text for the host to
  render.
- **Scenes** — a whole world described as data, saved and restored, so a level
  loads in one call instead of a thousand.
- **Custom shapes** — a Zig host implements Jolt's own shape interface, convex
  or general, without writing C++.
- **Geometry primitives on their own terms** — GJK and EPA against any two
  support functions, convex hull building, polygon clipping, triangle
  indexing, and the AABB tree builder and splitters that `MeshShape` is packed
  from. Usable without a physics system.
- **Triangle collision** — collide or cast a convex shape or a sphere against
  triangles fed one at a time, with active-edge normals and internal-edge
  removal, which is what a custom triangle source needs.
- **Reflection and serialisation** — Jolt's RTTI over its own registered types
  and the ObjectStream text and binary formats.
- **The solver's constraint parts**, ported to Zig rather than bound: all
  fourteen of them, so a custom constraint's inner loop makes no foreign call.
- **Deterministic maths** — Jolt's own trigonometry, matrix inversion and
  half-float conversions, bit-for-bit, for a build that has to agree with a
  C++ one.

Deliberately out of scope, and staying that way: a fixed-timestep loop, an
entity system, an asset format, and any coupling to a particular renderer.
Debug geometry comes out as data for the host to draw. Those are a host's job,
and keeping them out is what makes this package reusable.

### How complete that is, as a number

`tools/coverage.sh` walks Jolt's own headers and counts what is there;
`ci/check-coverage.sh` fails the build while any name it prints lacks a
verdict, and `ci/check-numbers.sh` fails it while any figure below disagrees
with a fresh run. No number here is typed by hand.

The claimed areas hold **2682 public Jolt names** — every method, every free
function, and every public data member of a `*Settings` type, which is an API
no method reaches. **1359 are spelled out by an entry point** of a matching
name. The other **1323 carry a recorded verdict** in `tools/verdicts_*.txt`,
one line each with its evidence: **605 `BOUND`** (the effect is reachable, by
another name), **198 `EXTENSION`** (a seam the host implements instead),
**214 `INTERNAL`** (not public in Jolt either — recomputed by
`tools/classify.sh`, never asserted), **190 `LANGUAGE`** (Zig has it already),
**116 `ZIG`** (ported to Zig rather than bound), and **0 `GAP`**. A `GAP` fails
the build, so the last figure is the only one it can be.

What those numbers do **not** say, because a completeness figure with no
stated blind spots is worse than none:

- **Matching is by name, not by behaviour.** A name counts as spelled out when
  its camelCase words appear in order inside an entry point's. That is
  deliberately strict in the safe direction — a renamed binding reads as
  unbound and has to earn a ledger line — but a coincidental word match reads
  as covered, and nothing here compares semantics.
- **`BOUND` means the effect is reachable through the entry points its
  evidence names, not that the upstream symbol itself crosses the ABI.**
  `Skeleton::CalculateParentJointIndices` is `BOUND` because
  `zjoltSkeletonAddJoint` sets what it would compute, not because it is
  callable.
- **The evidence is checked for existence, not for correctness.** A `BOUND` or
  `EXTENSION` line must name a symbol that really is in `ffi/*.h`, a `ZIG`
  line must point at a declaration that really is in `src/`, and `INTERNAL` is
  recomputed rather than believed — but none of that proves the binding
  behaves like Jolt's. The tests do that, and they reach far fewer names.
- **The harvest cannot see everything.** A settings field whose default is a
  call carries a paren and is read as a method; operators, macros and
  `JPH_`-prefixed declarations are skipped; so are names under four
  characters. Each exclusion is in `tools/coverage.sh` beside the code that
  applies it.
- **Only `libs/JoltPhysics/Jolt` is counted.** A Jolt directory missing from
  the claimed list fails the build, so the denominator cannot quietly shrink —
  but Jolt's samples and test framework are not counted at all.

Design decisions that stay settled are recorded at the end of
[BINDING.md](BINDING.md).

## Contributing

Issues and pull requests are welcome. Two things to know before opening one:

- **`libs/JoltPhysics` is vendored verbatim and must not be edited.** Changes
  there are lost at the next re-vendor. If upstream needs fixing, fix it
  upstream; if zjolt needs to work around upstream, do it in `ffi/` and record
  it in [UPSTREAM.md](UPSTREAM.md).
- **Run `ci/run.sh` before pushing** — or `ci/install-hooks.sh` once, and it
  runs itself. Run `ci/run.sh --full` for anything structural.
- **Comments state a contract, not a narrative.** `ci/check-comments.sh`
  enforces two things and will fail a pull request over either. A block above
  one declaration is at most six lines; a file header, or a block under a
  `//===---===//` banner, at most fourteen. And the register is documentation,
  not conversation — no "we", "our", "note that", "which is why", "turns out",
  "you might expect".

  The cap never justifies dropping a fact. Units, ownership, lifetime,
  nullability, error conditions and ordering constraints come first; if a block
  cannot hold them in six lines, shorten the prose around them, and do not
  lengthen a line past 80 columns to buy room.

New source files are added to the explicit lists in `build.zig` deliberately;
there are no globs, so nothing starts compiling by accident.

## Licence

MIT, see [LICENSE](LICENSE). Vendored Jolt Physics is MIT, copyright Jorrit
Rouwe and contributors.
