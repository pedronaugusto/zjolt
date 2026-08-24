# zjolt

[![CI](https://github.com/pedronaugusto/zjolt/actions/workflows/ci.yml/badge.svg)](https://github.com/pedronaugusto/zjolt/actions/workflows/ci.yml)

Zig bindings for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — rigid
bodies, shapes, queries and a character controller, in a package with no
renderer, no entity system and no clock attached.

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
  hand. Nine kinds of deliberate drift are verified to fail it.
- Jolt asserts where a library for a service would return, and several of those
  assertions sit on paths an ordinary caller reaches. Each one this ABI could
  reach has been turned into a returned error, with a test that fails if the
  guard is removed.

Status: **in development, unreleased.** No version is cut yet, and the package
is not API-stable — it is being grown into complete Jolt bindings, and naming
and shape may change until it is.

Working today: shapes, bodies, the step, queries and `CharacterVirtual`. Not
yet exposed: constraints, vehicles, ragdolls, soft bodies, hair, debug draw,
and the non-virtual `Character`. See [Scope](#scope).

## Usage

```zig
const zjolt = @import("zjolt");

// Which layers exist, and what collides with what. Plain Zig functions.
const Layers = struct {
    pub const static: zjolt.ObjectLayer = 0;
    pub const moving: zjolt.ObjectLayer = 1;

    pub fn broadPhaseLayerCount() u32 { return 2; }
    pub fn broadPhaseLayerFor(layer: zjolt.ObjectLayer) zjolt.BroadPhaseLayer {
        return if (layer == static) 0 else 1;
    }
    pub fn objectCanCollideWithBroadPhase(o: zjolt.ObjectLayer, b: zjolt.BroadPhaseLayer) bool {
        return if (o == static) b == 1 else true;
    }
    pub fn objectsCanCollide(a: zjolt.ObjectLayer, b: zjolt.ObjectLayer) bool {
        return if (a == static) b == moving else true;
    }
};

try zjolt.init(.{ .allocator = gpa });
defer zjolt.deinit();

const jobs = try zjolt.JobSystem.initThreadPool(.{});
defer jobs.deinit();

const system = try zjolt.PhysicsSystem.init(.{ .layers = zjolt.layersFromType(Layers) });
defer system.deinit();

const shape = try zjolt.Shape.initSphere(0.5, 0);
defer shape.release();

const ball = try system.bodies().createAndAdd(.{
    .shape = shape,
    .object_layer = Layers.moving,
    .position = zjolt.rvec3(0, 10, 0),
}, .activate);

// Per frame:
const update_error = try system.step(1.0 / 60.0, 1, jobs);
if (update_error.contact_constraints_full) { /* raise the limit */ }
const transform = system.bodies().getTransform(ball);
```

Add it as a dependency and link the module:

```zig
const zjolt_dep = b.dependency("zjolt", .{ .target = target, .optimize = optimize });
exe.root_module.addImport("zjolt", zjolt_dep.module("zjolt"));
```

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
branch on the target's ABI, on any platform.

The same shape covers `ObjectVsBroadPhaseLayerFilter`, `ObjectLayerPairFilter`,
`ContactListener`, `BodyActivationListener` and the three query filters.

### The frame loop has its own path

Jolt names bodies by id, and reading one body's transform takes a body lock.
That is the right shape for the occasional query and the wrong shape for what a
renderer does every frame, which is read the transform of every body that
moved — one ABI crossing and one lock per body, thousands of times.

So there is a second path for exactly that:

```zig
var ids: [1024]zjolt.BodyId = undefined;
const awake = try system.getActiveBodies(&ids);          // what moved
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
function's parameter count and per-parameter size, every enumerator's value,
every constant. Nothing is listed by hand — the check discovers what to compare
by reflecting over `c.zig`, and a declaration it cannot classify is a compile
error rather than a silent pass, so it cannot quietly stop covering something.
The `@cImport` is test-only; the shipped module never runs translate-c.

Pairing fields *by name* is the part that matters. Two same-sized adjacent
fields swapping places leaves the sequence of offsets identical, so a positional
comparison — or a digest folded over offsets alone — passes a swap that
reinterprets both fields. Nine kinds of deliberate drift are verified to fail
the build, including that swap, a dropped parameter, a widened parameter, a
renumbered enumerator, a narrowed enum tag, a moved mask bit and an extern
deleted from the Zig side.

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
- UBSan is **not** blanket-disabled. It stays on in Debug (`-Dsanitize_c`), so
  real undefined behaviour surfaces instead of being suppressed.
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
| `-Dsanitize_c` | on in Debug | Keeps Zig's C undefined-behaviour sanitizer. |
| `-Dshared` | `false` | Builds the C library as a shared object. |

## Testing

```sh
zig build test      # the Zig suite, and the C smoke test with it
zig build test-c    # the C smoke test alone
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
hit what is there and miss what is not, with and without filters; the bulk
read-back agrees exactly with the per-body accessors; a locked body reads and
writes; a rotation that is not unit length is renormalised rather than fatal; a
body constrained to a plane stays in it; a step that runs out of contact
constraints says which cache filled; `deinit` refuses while a handle is alive
and traces why; and a character settles on the floor, reports the body under
it, walks, and crouches.

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
is: 130 translation units, so a configuration is tens of seconds rather than
the couple of seconds a smaller library would take. On the machine this was
written on, `--full` is 22 checks in about four minutes from a cold cache; the
default is a few seconds once the cache is warm.

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

Exposed today:

- Shapes: box, sphere, capsule, convex hull, mesh, and the decorated shapes
  (scaled, rotated-translated, offset-centre-of-mass)
- Binary shape save and restore, for a collision cook
- A physics system with broad-phase and object layers and their filters
- Bodies: create, destroy, add, remove, motion type, teleport, kinematic move,
  activate, velocities, forces and impulses, shape swap, user data
- Body locks, read and write
- The step, with a job system seam
- Contact and activation listeners
- Ray casts, shape casts and overlap tests, closest-hit and all-hits
- `CharacterVirtual` with ground state, stair walking and shape swapping
- Bulk transform and motion read-back

Not exposed, in rough order of likely need:

- Constraints (hinge, slider, six-DOF, and the rest)
- Sensors as a first-class concept, rather than the `is_sensor` flag
- `HeightFieldShape` and compound shapes
- `StateRecorder`, for rollback and replay
- A host job-system implementation, plugged into the existing seam
- Vehicles, ragdolls, soft bodies

Nothing above is blocked — the sources are vendored and the C-boundary pattern
is established; they are simply not written yet. Deliberately out of scope: a
fixed-timestep loop, an entity system, and any coupling to a debug renderer.
Debug geometry, when it lands, will come out as arrays for the host to draw.
Those are a host's job, and keeping them out is what makes this package
reusable.

## Contributing

Issues and pull requests are welcome. Two things to know before opening one:

- **`libs/JoltPhysics` is vendored verbatim and must not be edited.** Changes
  there are lost at the next re-vendor. If upstream needs fixing, fix it
  upstream; if zjolt needs to work around upstream, do it in `ffi/` and record
  it in [UPSTREAM.md](UPSTREAM.md).
- **Run `ci/run.sh` before pushing** — or `ci/install-hooks.sh` once, and it
  runs itself. Run `ci/run.sh --full` for anything structural.

New source files are added to the explicit lists in `build.zig` deliberately;
there are no globs, so nothing starts compiling by accident.

## Licence

MIT, see [LICENSE](LICENSE). Vendored Jolt Physics is MIT, copyright Jorrit
Rouwe and contributors.
