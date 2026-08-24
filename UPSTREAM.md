# Vendored upstream

`libs/JoltPhysics` is a pinned copy of upstream **Jolt Physics**, unmodified.

| | |
|---|---|
| Source | <https://github.com/jrouwe/JoltPhysics> |
| Version | 5.6.0 |
| Tag | `v5.6.0` |
| Commit | `e77f175595e64cb44218cc9d9d56fc365ad0e36a` |
| Date | 2026-07-11 |
| License | MIT (`libs/JoltPhysics/LICENSE`) |

## What was taken, and what was left behind

Taken: the whole of upstream's `Jolt/` directory, **verbatim and entire**, plus
`LICENSE`.

| Left out | Reason |
|---|---|
| `Samples/`, `TestFramework/`, `JoltViewer/`, `HelloWorld/` | Upstream's demos and their window/renderer dependencies. |
| `UnitTests/`, `PerformanceTest/` | Upstream's own test suites. |
| `Assets/` (21 MB) | Sample assets. The tests here build their geometry in code. |
| `Docs/` (5.5 MB) | Prose and images; the upstream repository is the place for them. |
| `Build/` | CMake, superseded by `build.zig`. |

Nothing is excluded from inside `Jolt/`. That is worth stating plainly, because
it is what lets `ci/verify-vendor.sh` diff the vendored tree against upstream
with **no exclusion list at all** — "unmodified" means byte-identical, rather
than identical-modulo-a-list that could itself fall out of date.

Which translation units actually compile is decided explicitly in `build.zig`
(`jolt_sources`), never by a directory glob.

## What compiles, and what merely sits there

`build.zig` compiles 130 of Jolt's translation units, taken from upstream's own
`Jolt/Jolt.cmake` unconditional source list. Three groups are vendored but not
compiled:

| Not compiled | Reason |
|---|---|
| `Jolt/Compute/{DX12,VK,MTL,CPU}` | Jolt 5.6's GPU compute backends. They need the Direct3D 12, Vulkan or Metal SDKs, which have no place in a physics package. Their backend-agnostic headers *are* compiled, because `Jolt/Physics/Hair` includes them unconditionally. |
| `Jolt/Shaders/*.cpp` | CPU-compute shader wrappers, live only under `JPH_USE_CPU_COMPUTE`. |
| `Jolt/ObjectStream/*.cpp` | Optional upstream (`ENABLE_OBJECT_STREAM`), and unused here. Shape serialisation goes through `StreamIn`/`StreamOut`, not the object stream, so `JPH_OBJECT_STREAM` stays undefined. |

Vehicles, ragdolls and soft bodies **are** compiled. They are part of the
library and `RegisterTypes.cpp` refers to them; they are out of scope for the C
ABI, not for the build.

## Toolchain floor

Jolt 5.6 dropped support for Visual Studio before 2022, Clang before 16, and
GCC before 12. zjolt compiles Jolt through Zig's bundled Clang, which is well
past that floor on every target in the CI matrix — but a consumer linking the
static library into an older toolchain's C++ project should know the floor
exists.

## Configuration macros, and the abort they cause

This is the one upstream behaviour most likely to bite a binding, so it is
worth spelling out.

Jolt folds its build configuration into a version id (`Jolt/Core/Core.h`):
`JPH_DOUBLE_PRECISION`, `JPH_OBJECT_LAYER_BITS`, `JPH_ENABLE_ASSERTS`,
`JPH_PROFILE_ENABLED`, `JPH_OBJECT_STREAM` and six more each occupy a bit.
`RegisterTypesInternal` compares the caller's id against the library's, traces
which define disagrees, and then calls **`std::abort()`**. It is not a warning
and not an error code.

That failure is unreachable here by construction. `build.zig` defines the macro
set once, in `applyBuildMacros`, and applies it to the Jolt translation units,
to the `ffi/` translation units and to the C smoke test alike. Zig never
`@cImport`s a Jolt header, so no fourth copy of the configuration exists.

The same hazard exists one level up, between `ffi/zjolt.h` and its consumer:
`ZJoltReal` and `ZJoltObjectLayer` change width with the build, so a consumer
compiled against different settings would misread every position it is handed.
`zjoltInit` passes the header's `ZJOLT_CONFIG_ID` to the library and gets
`ZJOLT_RESULT_CONFIG_MISMATCH` if they differ — the same trick Jolt plays on its
clients, for the same reason, but returning rather than aborting.

## Known upstream behaviour worked around here

Recorded so a future re-vendor can check whether any of it has been fixed, and
so the workarounds are not mistaken for arbitrary defensiveness.

**`Shape::sRestoreFromBinaryState` uses a value before checking the read that
produced it.** It reads an `EShapeSubType` from the stream and passes it
straight to `ShapeFunctions::sGet`, which indexes a fixed table
(`Shape.cpp:191`); the `IsEOF`/`IsFailed` check comes *after*. A byte outside
the enum's range is therefore an out-of-bounds index — an assertion failure in
a build with asserts on, and a read of whatever follows the table otherwise.
Reproduced by handing `zjoltShapeRestore` a buffer of ordinary text.

Worked around by wrapping Jolt's payload in a 32-byte container carrying a
magic tag, a container version, this library's config id, the Jolt version, the
payload length and a CRC-32. Restore validates all of it before Jolt sees a
byte. It is not a defence against a crafted payload that carries a matching
checksum, and does not claim to be; it is what makes "not a shape" and "damaged
in storage" clean errors instead of undefined behaviour.

**Everything else Jolt reads, it checks.** This is the pleasant half of the
finding, and it is why the workaround above is a container rather than a
re-implementation of the parser: once the first byte is known to be sane, Jolt
tests `IsEOF()` and `IsFailed()` after every stage of a restore. `zjolt`'s
`ConstStreamIn` zero-fills past the end and reports EOF honestly, and a test
(`src/integration_test.zig`, "a truncated payload inside a well-formed
container is still refused") rebuilds the container around every truncated
prefix of a real payload to drive that path deliberately. Every one of them is
refused.

**`JPH_OVERRIDE_NEW_DELETE` does not check its allocation.** Jolt's `operator
new` is `return JPH::Allocate(inCount)` with no null check, so under a failing
allocator a `new Factory()` would placement-construct at address zero. zjolt
allocates the factory itself, through a checked helper (`zjolt::New` in
`ffi/zjolt_internal.h`), which is what makes `ZJOLT_RESULT_OUT_OF_MEMORY` at
start-up an error rather than a null dereference.

What that does **not** cover is Jolt's own internal allocations. Its containers
abort on allocation failure rather than propagating one, so a physics system
created under memory pressure will abort inside Jolt. That is not fixable from
outside, and the practical reading is the usual one for a game: run where an
allocation failure is not survivable anyway, and size `max_bodies` up front.

**`Mat44::sRotation` asserts its quaternion is unit length** (`Mat44.inl:87`).
That is a documented precondition rather than a failing, and it is listed here
because of what this package chose to do about it, not because Jolt is wrong.

A body's rotation reaches that assertion on every step and every query
placement, and a rotation integrated over a few thousand frames drifts off the
unit sphere long before it looks wrong — so the assertion fires for a reason
the caller cannot see, in a build where they were not looking. zjolt therefore
renormalises every caller-supplied rotation as it crosses the boundary
(`zjolt::ToJoltRotation`), and turns one that cannot be normalised — all
zeroes, or carrying a NaN — into the identity.

That is a trade, and worth naming: it hides a caller's drift bug rather than
reporting it. The alternative, refusing a non-unit rotation with
ZJOLT_RESULT_INVALID_ARGUMENT, would make every caller renormalise on a
schedule they have to work out for themselves, and drift is not a mistake so
much as an inevitability. Absorbing it is the same call most engines make.

**`PhysicsSystem::Update` asserts on the error code it is about to return.**
`JPH_ASSERT(errors == EPhysicsUpdateError::None, ...)` sits one line above
`return errors` (`PhysicsSystem.cpp:679`). So the mask that exists to let a
host notice it has under-provisioned `max_body_pairs` or
`max_contact_constraints` — and degrade gracefully — is fatal in the builds
where a host is most likely to hit it. Not worked around: it is upstream's
judgement that a developer should be told loudly, and swallowing it would mean
suppressing an assert globally during every step. It is documented on
`zjoltPhysicsSystemStep` instead, and the suite covers the mask by installing
an assert hook that records and declines to break — which is what that hook is
for.

**Jolt's default trace function aborts.** `TraceFunction Trace = DummyTrace`,
and `DummyTrace`'s entire body is `JPH_ASSERT(false)`
(`Core/IssueReporting.cpp:9`). An application that never installs one dies the
first time Jolt has anything to say — which, given `PhysicsSystem::Update`
above, can be the first busy frame. Worked around by installing zjolt's own
trace thunk unconditionally, falling back to stderr when the host supplied no
callback.

**`Body`'s setters assert on a static body.** `Body::SetLinearVelocity` and
friends dereference `mMotionProperties`, which a static body does not have, so
they assert (`Body.h:155`). The locked mutators in `ffi/zjolt_body.cpp` check
`IsStatic()` first and do nothing, because a C ABI should not abort a process
over a call that is merely pointless.

**`CharacterVirtualSettings::mSupportingVolume` defaults to a plane that
supports everything.** Left at Jolt's default, a character reports a wall it
brushes against as ground. `zjoltCharacterCreate` places the plane at the
bottom of the character's own shape instead, which is what makes
`zjoltCharacterGetGroundState` mean what its name says.

**A compound of one sub-shape reaches `CountLeadingZeros(0)`, which is
undefined on ARM.** `CompoundShape::GetSubShapeIDBits` sizes the index field of
a sub-shape id as `32 - CountLeadingZeros(count - 1)`
(`CompoundShape.h:329-334`). `Math.h:180-206` guards a zero argument on x86 and
on E2K, RISC-V, PowerPC and LoongArch — but the ARM path is a bare
`__builtin_clz`, for which zero is undefined. A count of one makes the argument
zero, and a count of none underflows the subtraction into `0xffffffff` first.

`StaticCompoundShapeSettings::Create` never gets there, because it simplifies a
single child into the child itself or a `RotatedTranslatedShape`
(`StaticCompoundShape.cpp:33-58`). `MutableCompoundShape` does not simplify, so
one child is constructible through Jolt's ordinary API and the resulting shape
is undefined from the constructor onward. Reproduced by building one on
aarch64-macos with `-Dsanitize_c`, which is on by default in Debug: *"passing
zero to clz(), which is not a valid argument"*, inside
`MutableCompoundShape`'s own constructor.

Worked around by refusing fewer than two children at both ends —
`zjoltShapeCreateMutableCompound` and `zjoltShapeMutableCompoundRemoveChild` —
so the shape cannot be put into that state through this ABI at all. It is the
one place this package narrows an upstream API rather than forwarding it, and
the narrowing is what makes the sanitizer's finding unreachable instead of
suppressed.

**`SoftBodyMotionProperties::SetVertexRadius` asserts on the wrong value.**
The whole body is `JPH_ASSERT(mVertexRadius >= 0.0f); mVertexRadius =
inVertexRadius;` (`SoftBodyMotionProperties.h:102`) — it checks the value it is
about to *overwrite*, not the one being set. So a negative radius is accepted
silently and blamed on the next caller, or on nobody at all if the radius is
set once. `SoftBodyCreationSettings::mVertexRadius` reaches the same setter
through `Initialize`, so a soft body can be born with one.

Checked here on the incoming value instead, at both doors —
`zjoltSoftBodyCreate` and `zjoltSoftBodySetVertexRadius` — which is what that
assert was plainly written to mean.

**A soft body's iteration count is a divisor nothing checks.**
`InitializeUpdateContext` computes `mSubStepDeltaTime = inDeltaTime /
mNumIterations` (`SoftBodyMotionProperties.cpp:953`) with no assert and no
guard, so a `SoftBodyCreationSettings::mNumIterations` of zero makes every
sub-step infinite and the body leaves for the origin on its first step. Refused
at the boundary rather than forwarded.

**Nothing between a soft body's constraint indices and Jolt's vertex array.**
`SoftBodySharedSettings` takes faces, edges, volume constraints and skinned
constraints as bare vertex indices, and every consumer of them — the solver,
`GetVolumeTimesSix`, `SkinVertices` — indexes `mVertices` with them directly.
`AddFace`'s only assert is that the three indices are pairwise distinct, and
`JPH::Array::operator[]` (`Array.h:566`) asserts in a debug build and reads past
the end of the array in a release one. `ffi/zjolt_softbody.cpp` validates each
batch against the vertex count before appending any of it, which is the last
point at which the index is still known to have come from outside.

**Vehicle braking differs materially between the precision builds.** From
19 m/s under full brakes for five seconds, a float build stops dead and a
`-Ddouble_precision` build is still doing about 7 m/s. It is not solver
convergence: raising the velocity iterations from 30 to 120 moved the
double-precision figure the wrong way, 0.37 to 0.41 of the starting speed.

Not chased to a root cause, and recorded rather than explained. Jolt's own
vehicle collision testers take `RVec3` and handle the precision themselves,
and this ABI passes only directions and settings into that path, so there is
no obvious conversion on this side to blame — but "no obvious" is not the same
as "checked". `src/vehicle_test.zig` asserts the property that survives both
(the brakes do something) rather than the figure that does not, and says why
at the assertion.

A host shipping double precision should not assume a vehicle brakes the same
as the float build it was tuned on.

## Re-vendoring procedure

`ci/verify-vendor.sh` fetches the pinned commit and diffs it against `libs/`,
so the claim that this copy is unmodified is checked rather than asserted. It
runs as its own CI job. Run it after any step below.

1. Clone upstream at the new tag; copy `Jolt/` and `LICENSE` over
   `libs/JoltPhysics/`, re-applying the exclusions above.
2. Update the table at the top of this file, and the three constants at the top
   of `ci/verify-vendor.sh`. The script refuses to run unless the tag and
   commit both appear in this file, so the two cannot drift.
3. Regenerate `jolt_sources` in `build.zig` from the new
   `Jolt/Jolt.cmake` unconditional list, and re-check the three
   not-compiled groups above against it. Adding a source is a deliberate
   act; the list exists so a re-vendor cannot silently change what compiles.
4. `zig build test` and `zig build test-c`. The `static_assert`s in
   `ffi/zjolt_abi.cpp` fail the build if a type, enumerator or constant that
   zjolt converts to or from has changed — including a silently *renumbered*
   enum, which the conversions themselves would happily keep compiling.
5. Re-read the "known upstream behaviour" section above and check whether any
   of it has been fixed. If it has, delete the workaround and the note
   together.
