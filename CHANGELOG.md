# Changelog

Every entry names what changed at the C ABI, at the Zig API, or in what the
build guarantees. A change with no visible effect on any of the three is not
listed; the git history has it.

The version lives in `ffi/zjolt_core.h` and nowhere else: `build.zig.zon` is
held against it by a Zig test, `ZJOLT_CONFIG_ID` folds it, and
`ci/check-numbers.sh` fails the build if the README's status line disagrees.

## 0.2.0

### C ABI — added

- `zjoltBodySetApplyGyroscopicForce`, `zjoltBodySetCollideKinematicVsNonDynamic`
  and `zjoltBodySetEnhancedInternalEdgeRemoval`. The first two had getters and
  no setter, so they could only ever return Jolt's default.
- `zjoltBodySetNumVelocityStepsOverride` / `zjoltBodyGetNumVelocityStepsOverride`
  and the position-steps pair.
- `zjoltShapeHeightFieldGetMaterials` and `zjoltShapeHeightFieldSetMaterials`.
  A height field's quad materials could be set once at creation and never read
  or changed, while `HeightFieldShape` supports both — a crater could be dug
  into terrain but never scorched.
- `zjoltPhysicsSystemGetConstraints`, the enumeration behind
  `zjoltPhysicsSystemGetNumConstraints`. `PhysicsSystem::GetConstraints` had
  no crossing at all, so the count was the only thing a caller could learn
  about a system's joints.
- `zjoltVehicleConstraintSetEngineDesc` and
  `zjoltVehicleConstraintSetTransmissionDesc`: the writable half of the two
  getters. Without them `zjoltVehicleConstraintEngineClampRPM` had no reachable
  purpose.
- `zjoltRagdollSettingsSetPartConstraint` / `GetPartConstraint`,
  `zjoltRagdollSettingsAddAdditionalConstraint`,
  `zjoltRagdollSettingsGetNumAdditionalConstraints` and
  `GetAdditionalConstraint`. A ragdoll part's joint could only ever be the
  swing-twist `ZJoltRagdollConstraintDesc` describes, and
  `RagdollSettings::mAdditionalConstraints` — the field that joins two parts
  that are not parent and child — had no crossing at all.
- `zjoltRagdollSetLinearAndAngularVelocity`, `zjoltRagdollSetLinearVelocity`,
  `zjoltRagdollAddLinearVelocity` and `zjoltRagdollAddImpulse`. Moving a whole
  ragdoll meant walking its body ids and calling the body interface per part.
- `zjoltSoftBodySharedSettingsOptimizeWithRemap` and
  `zjoltSoftBodySharedSettingsGetRemapCounts`. `Optimize` reorders every
  constraint list, and Jolt's `OptimizationResults` — the seven index maps
  that say where each one went — was discarded, so any index a caller had
  recorded named a different constraint afterwards with nothing to say so.
- `zjoltSoftBodySharedSettingsSaveBinaryState` / `RestoreBinaryState` and
  `SaveWithMaterials` / `RestoreWithMaterials`. A soft body's topology could
  only be saved inside a whole scene.
- `zjoltSkeletonPoseGetJointMatrices` and `zjoltSkeletonPoseSetJointMatrices`.
  The model-space matrices were computed and consumed inside the ABI and never
  readable, so a pose produced outside Jolt had no way in and a pose Jolt
  computed had no way out.
- `zjoltCpuFeatures` and the `ZJoltCpuFeature` flags, one per Jolt
  `JPH_USE_*` macro. Which instruction set Jolt's vector paths were compiled
  for was unknowable from outside the library, and a default Zig target
  resolves to a baseline CPU model, so a build could be running the SSE2
  paths with nothing able to say so.
- `zjoltGroupFilterCustomCreate`, `zjoltGroupFilterIsTable` and the
  `ZJoltCustomGroupFilter` callback. `JPH::GroupFilter` is an abstract base a
  game subclasses and `GroupFilterTable` is the one implementation Jolt ships;
  only the table crossed, so a collision rule that is not a bit table — a team
  id, a hit mask, state the host keeps elsewhere — had no crossing at all.
  The three `zjoltGroupFilterTable*` entry points refuse a callback filter
  with a message naming the entry point that made it, and
  `zjoltGroupFilterGetNumSubGroups` reports 0 for one.

- `zjoltSkeletonAddJointWithParentName` and
  `zjoltSkeletonCalculateParentJointIndices`, the second `Skeleton::AddJoint`
  overload and the pass that resolves what it records. Only the indexed form
  crossed, so a host importing a skeleton whose file stores parents BY NAME
  had to resolve them itself, and `CalculateParentJointIndices` reached the
  ABI through nothing at all.
- `ZJOLT_MAX_FACE_VERTICES`, Jolt's own cap on a contact face
  (`CollideShapeResult::Face` is `StaticArray<Vec3, 32>`).
- `zjoltSimCollideCollectorGetEarlyOutFraction`,
  `GetPositiveEarlyOutFraction`, `ResetEarlyOutFraction`,
  `UpdateEarlyOutFraction`, `ForceEarlyOut` and `ShouldEarlyOut`. A
  `ZJoltSimCollideFn` is handed the one `JPH::CollisionCollector` that crosses
  this ABI and could only add hits to it: the float every collision routine
  reads to skip work it cannot beat was unreachable, so a hook could not say
  "I already have a contact this deep" before delegating to
  `zjoltSimCollideDefault`. `Update` refuses a value that widens rather than
  narrows -- Jolt asserts on that -- and `Reset` is the entry point for a seed
  that goes either way.

### C ABI — changed

- `zjoltPhysicsSystemStep` takes a `const ZJoltTempAllocator *` before the job
  system, as `PhysicsSystem::Update` does. NULL keeps the allocator the system
  was created with; anything else serves that one step, so a host can hand the
  simulation a frame arena it resets afterwards. Both are per-call arguments
  in Jolt, and this ABI passed one per call and bound the other at creation.
- `ZJoltCollideShapeHit` and `ZJoltShapeCastHit` carry `face_on_1`,
  `face_on_1_count`, `face_on_2` and `face_on_2_count`. `collect_faces_mode`
  was settable on every overlap and cast, cost the work on every hit, and had
  nowhere to deliver a face; the header said as much. The callback forms
  deliver one now, borrowed for the callback; the forms that outlive Jolt's
  result force NO_FACES rather than pay for a face they must discard.
- `ZJOLT_CONSTRAINT_SUB_TYPE_OTHER` is replaced by
  `ZJOLT_CONSTRAINT_SUB_TYPE_NONE` (0, a NULL handle),
  `ZJOLT_CONSTRAINT_SUB_TYPE_VEHICLE` (14) and
  `ZJOLT_CONSTRAINT_SUB_TYPE_USER_DEFINED` (15). Zero stood for both "not a
  constraint" and "a kind this ABI does not name", so a vehicle -- which this
  library builds -- was indistinguishable from nothing at all. Values 1..13
  are untouched.
- `zjoltShapeCreateMesh` and `zjoltShapeCreateHeightField` build from
  `ZJoltMeshShapeDesc` and `ZJoltHeightFieldShapeDesc`, passed by const
  pointer and initialised by `zjoltMeshShapeDescInit` /
  `zjoltHeightFieldShapeDescInit` — the pattern `ZJoltBodyDesc` set. The two
  had grown to 12- and 14-parameter positional calls, where two transposed
  counts still compile and misread every argument after them; a desc names
  every field at its call site and gives the defaults one home. The height
  field's `offset` and `scale` cross by value in the desc, and a zeroed desc
  is not a default one: the init functions write the defaults zero cannot
  spell. `materials_capacity` (Jolt's `mMaterialsCapacity`) is a field of
  the height-field desc; without it the material list a repaint grows is
  reallocated under any query running in parallel.
- `zjoltShapeHeightFieldSetHeights` takes
  `active_edge_cos_threshold_angle` before `heights` and `stride`:
  everything steering the repaint — the block, then how its edges rebuild —
  precedes the samples it consumes.
- `zjoltPhysicsSystemGetNumActiveBodies`, `zjoltPhysicsSystemGetActiveBodies`
  and `zjoltPhysicsSystemGetActiveBodiesUnsafe` take a `ZJoltBodyType`. All
  three passed `JPH::EBodyType::RigidBody` unconditionally, so a system's soft
  bodies were unreachable through every active-body query this ABI had.

- `ZJoltCharacterDesc` gained `min_time_remaining` and
  `inner_body_id_override`. `CharacterVirtualSettings` has both; without the
  first the solver's early-out could not be tuned, and without the second a
  character's inner body took a generated id, so a rebuilt world handed the
  same character a different one — the thing a replay compares against.
- `ZJoltBodyDesc` gained six fields, in Jolt's own order:
  `apply_gyroscopic_force`, `collide_kinematic_vs_non_dynamic`,
  `use_manifold_reduction`, `enhanced_internal_edge_removal`,
  `num_velocity_steps_override`, `num_position_steps_override`. Together with
  `mass_properties_override`, which every path silently dropped, seven
  `BodyCreationSettings` settings were unreachable through this ABI.
- `ZJoltShapeFilter.should_collide` takes `const ZJoltShape *shape` and
  `const ZJoltShape *query_shape` beside the two sub-shape ids, mirroring
  `ZJoltSimShapeFilter`, which already carried both. The filter was handed a
  body id and two opaque ids, so it could not decide on the shape's own type,
  user data or material — the questions a SHAPE filter exists to answer, as
  opposed to the body filter one level above it. `query_shape` is NULL for a
  query with no shape of its own: a ray, a point.
- `zjoltShapeCreateSphere` and its siblings are unchanged; `BINDING.md`'s
  walk-through, which described a three-parameter version of it, is now a
  verbatim excerpt checked by `ci/check-examples.sh`.

### Zig API

- `BodyDesc` carries the same six new fields, and crosses to the C descriptor
  by field NAME rather than by a hand-kept list. A field on one side and not
  the other is now a compile error in both directions.
- `BodyInterface` gained `setApplyGyroscopicForce`,
  `setCollideKinematicVsNonDynamic`, `setEnhancedInternalEdgeRemoval`, and the
  two step-override pairs.
- `VehicleConstraint` gained `setEngineDesc` and `setTransmissionDesc`.
- `Shape.heightFieldMaterials` and `Shape.heightFieldSetMaterials`, and
  `HeightFieldOptions.materials_capacity`.
- `PhysicsSystem.numActiveBodies`, `countActiveBodies`, `getActiveBodies` and
  `getActiveBodiesUnsafe` take a `BodyType`, mirroring the C change.
- `Character.Options` carries `min_time_remaining` and
  `inner_body_id_override`. Both `Character.Options` and
  `RigidCharacter.Options` now cross to their C descriptor by field NAME, the
  way `BodyDesc` already did; the shared `descriptor.zig` is the one home for
  that rule, and a field on one side and not the other is a compile error.
- `zjolt.system.simCollideEarlyOutFraction`,
  `simCollidePositiveEarlyOutFraction`, `resetSimCollideEarlyOutFraction`,
  `updateSimCollideEarlyOutFraction`, `forceSimCollideEarlyOut` and
  `simCollideShouldEarlyOut`, beside `addSimCollideHit` and
  `simCollideDefault` -- the collector a `collide` hook is given is steerable
  now, not just addable to.
- `zjolt.constraintList` fills a caller's buffer with every constraint in a
  system, each holding a reference of its own.
- `RagdollSettings.setPartConstraint`, `partConstraint`,
  `addAdditionalConstraint`, `additionalConstraintCount` and
  `additionalConstraint`, the last returning a `RagdollAdditionalConstraint`.
  Settings that are not a two-body kind are refused with
  `error.InvalidArgument` rather than cast.
- `Ragdoll.setLinearAndAngularVelocity`, `setLinearVelocity`,
  `addLinearVelocity` and `addImpulse`.
- `SoftBodySharedSettings.optimizeWithRemap`, `remapCounts`,
  `saveBinaryState`, `restoreBinaryState`, `saveWithMaterials` and
  `restoreWithMaterials`.
- `SkeletonPose.getJointMatrices` and `setJointMatrices`.
- `zjolt.cpuFeatures()` returns a `CpuFeatures` flag struct. A test holds it
  to Jolt's own implication order — AVX2 arrives with AVX, SSE4_2, SSE4_1,
  F16C, LZCNT, TZCNT and FMADD — so a report that had come loose from the
  macros Jolt compiled with fails the build.
- `GroupFilter.initCustom` takes a `can_collide` callback and `isTable` says
  which kind a filter is. The three table methods return
  `error.InvalidArgument` on a callback filter rather than casting it.
- A `QueryFilters.shape` callback receives both shape pointers, mirroring the
  C change; `zjolt.Shape{ .handle = ... }` reads either with the ordinary
  shape accessors for the duration of the call.
- `layersFromInstance` builds the same three layer tables from a live value
  rather than from a type, so a layer scheme read from data at run time is
  expressible. `layersFromType` is unchanged; both now reject a type missing
  one of the four required declarations with a `@compileError` naming it.

### Fixed

- `-Ddebug_renderer=true` did not compile. `PolyhedronSubmergedVolumeCalculator`
  takes a world origin as one more constructor parameter under
  `JPH_DEBUG_RENDERER`, and `ffi/zjolt_shape.cpp` passed the six it takes
  without it, so the whole library failed to build in a configuration the
  hosted workflow declares. It is passed as Jolt's own
  `JPH_IF_DEBUG_RENDERER(, ...)` now, zero being the origin this ABI carries.
  With the option on, ten tests that only run when Jolt collects debug-draw
  geometry execute for the first time: 500 of 503 rather than 490.
- `zjoltLiveHandleCount` counted no reference-counted object, so a shape,
  material, group filter, skeleton, animation, scene, path, soft body
  settings or job the host never released left the count at zero and
  `zjoltDeinit` tore the library down anyway -- and destroying that handle
  afterwards freed it through an allocator it was never allocated from.
  Every reference this ABI hands out is counted now, at one home
  (`zjolt::HostRetain` / `zjolt::HostRelease`), which is also the fix for
  `zjoltConstraintSettingsAddRef`: it moved Jolt's count and not this one
  while its `Release` moved both, so an AddRef/Release pair drove the total
  NEGATIVE, and a negative total reads as "nothing outstanding". A ragdoll
  and a skeleton mapper keep their own count and so keep moving on create
  and on the release that destroys them; `ci/check-refcounts.sh` is what now
  refuses a bare `AddRef` or `Release` in `ffi/`, and a pair whose two ends
  do not account the same way.
- Reading a `ZJoltBodyType` a host filled in was undefined behaviour. The
  three active-body entry points loaded the parameter as an enum, which
  aborts under UBSan for any value no enumerator names;
  `zjolt::ToJoltBodyType` takes the raw integer, as every other enum
  conversion in this ABI already did. `ci/run.sh`'s sanitizer arm was red.
- A hit filled into a caller's buffer carried stack garbage in any field the
  projector did not write: `HitStream::AddHit` left its hit
  default-initialised. It is value-initialised now.
- `hostStream` panicked on a zero-length read or write. An empty
  `JPH::Array`'s `data()` is NULL, and Jolt writes one per empty constraint
  list, so saving soft body shared settings through a host stream aborted in
  safe Zig before reaching the first field.
- The buffer-backed streams called `memcpy`/`memset` on the same zero-length
  writes: `CountingStreamOut::WriteBytes`, `MemoryCursorWrite` and
  `HostStream::ReadBytes`'s missing-callback path all passed Jolt's
  `(nullptr, 0)` straight through, which is undefined behaviour even at zero
  bytes — glibc's nonnull annotations abort on it. Saving a shape that
  serialises any empty array — a baked hair groom, a constraint-free soft
  body — crashed on Linux Debug. All three skip the zero-size case now;
  `ConstStreamIn::ReadBytes` and `MemoryCursorRead` already guarded it.

- `Skeleton.addJointWithParentName` and `Skeleton.calculateParentJointIndices`.
- `PhysicsSystem.stepWithTempAllocator`; `step` is it with no override.
- `CollideShapeHit.faceOn1`/`faceOn2` and the same pair on `ShapeCastHit`,
  each an empty slice when the query asked for no face or the form cannot
  carry one.
- `err.filled(buffer, count)` turns a two-call query's buffer and returned
  count into a slice, refusing a count larger than the buffer instead of
  slicing past it. Forty-five call sites did the slice by hand and trusted a
  number that had crossed the ABI.

### Zig API — allocation

- `Character.activeContactCount` and `Character.activeContacts(out)` read the
  active contact list without allocating. `getActiveContacts(allocator)` is
  now the convenience built on them rather than the only way to ask.

### Performance

- The Jolt allocator bridge asks the backing allocator to grow a block in
  place before falling back to allocate-copy-free. Every `reallocate` used to
  move the whole block, and Jolt reallocates its body list, its contact cache
  and every temporary collector.
- Value math is computed in Zig over `@Vector(4, f32)` instead of crossing the
  ABI once per operation: `Vec3.lerp`, `RVec3.lerp`, `Quat.multiply`,
  `conjugate`, `inverse`, `dot`, `isNormalized`, `normalize`, `lerp`,
  `rotateVector`, and `Mat44.multiply`, `transformPoint`, `transformDirection`
  and `inverseRotationTranslation`. A quaternion product measured 1.80–1.84
  ns/op through the entry point and 0.77–0.79 ns/op computed in Zig, four runs,
  ReleaseFast, x86_64-windows-gnu, 4096 independent operand pairs per pass so
  neither arm is a dependency chain. The C entry points are unchanged and every
  one is still exported; `src/vec_test.zig` compares the two answers.
- `Quat.rotateVector` computes in Zig only for a quaternion clearly inside the
  length tolerance and hands anything near the line to the C entry point, so a
  refusal is still the library's, with the library's message.

### Documentation, and what now holds it

- `ci/check-numbers.sh` recomputes every count a document states and fails when
  one disagrees with the tree. Four were wrong.
- `ci/check-examples.sh` proves every code example in `BINDING.md` is a
  verbatim excerpt of the file it names. All four panels of its walk-through
  were stale.
- `ci/counts.sh` is the single home for every count formula; the three scripts
  that quote one now source it instead of recomputing it.
- The README's quick start is now two verbatim excerpts of
  `tests/consumer/src/main.zig`, the file that is built through
  `b.dependency` and run. `ci/check-examples.sh` reads README.md as well as
  BINDING.md and holds them character for character; the two had drifted,
  and the README's version was compiled by nothing.
- `tools/unbound_*.txt` is renamed `tools/verdicts_*.txt`. Most of its rows
  record a `BOUND` verdict, so the old name described a minority of it. The
  tally is in the README, where `ci/check-numbers.sh` recomputes it; a
  second copy here would be a second fact free to disagree, and had.
- `ci/check-coverage.sh` strips comments before deciding whether Zig calls an
  entry point. A doc comment that merely NAMED one counted as calling it, so
  prose could satisfy the rule that nothing is stranded.
- `tools/zig_native.txt` records every entry point the Zig API computes rather
  than calls, with the declaration that computes it and the test that proves
  the two agree; `ci/check-coverage.sh` holds all four parts.
- `tools/coverage.sh` harvests the public data members of Jolt's `*Settings`
  types as well as its methods, and matches them against the declarator names
  in `ffi/*.h`. A settings object is the whole public API of several
  subsystems and reaches no method, so every one of those fields sat outside
  the coverage claim. `tools/jolt_access.awk` learned the same rule, so
  `INTERNAL` stays recomputed rather than asserted for a field.
- The README's completeness claim states the recomputed figures — public
  names, how many an entry point spells out, and the tally per verdict — with
  the blind spots of the count beside them. `ci/check-numbers.sh` gates every
  one, and checks that the two halves still add to the whole.
- `zjoltVehicleConstraintGetGearRatio` was documented as returning the gear
  ratio times the differential ratio. It returns the gear ratio alone.

- `ci/check-headers.sh` proves every header reachable by `#include` from an
  installed header is installed too. The step it replaces compared includes
  against a grep for `ffi/` PATHS, so it called the generated `zjolt_config.h`
  missing on a correct tree, counted `ffi/zjolt_internal.h` installed because
  build.zig names it in a comment, and matched `ffi/zjolt.h` -- the umbrella --
  on neither side. `ci/run.sh` and the hosted workflow call the one script.
- `ci/check-comments.sh` gained a third rule: nothing that dates itself to a
  day, a machine or one run. Rules 2 and 3 now cover build.zig, the CI and
  tool scripts, the consumer and C smoke tests, and every document, not only
  the declaration files.
- `ci/run.sh --full` runs the suite and the C ABI test on the MSVC ABI when it
  is invoked on Windows. The hosted workflow already did; the local mirror of
  it did not, so the second ABI a Windows host has was never built there.

- The README's worked example of what a `BOUND` verdict means is read back
  against `tools/verdicts_*.txt` by `ci/check-numbers.sh`: the upstream name
  it quotes must have a row, that row must say `BOUND`, and its evidence must
  name the entry point the sentence credits. The example it used to carry had
  lost its row to an entry point of its own, so the rule was being taught from
  a line no file held. The guards `ci/check-abi-drift.sh` mutates are listed
  rather than counted for the same reason: the count was written in words,
  which that gate states it cannot read, and it had gone stale.
- `ci/check-comments.sh` spends its two budgets in CHARACTERS at an 80-column
  width rather than in newlines. Six lines was six lines whether they held
  forty characters or four hundred, so an unwrapped paragraph -- 303 of them
  on one line, in one case -- cost the same as a one-line note. A `//===` or
  `//---` banner is recognised wherever it is indented, which it had not been
  inside a Zig struct; and the 137 comment lines in `ffi/` past 80 columns are
  wrapped, that being the width those files are formatted to.
- `ci/check-abi-drift.sh` takes both of its document anchors from
  `ci/counts.sh`. One was a written-out entry-point count that the tree had
  moved past, so the mutation reported ANCHOR STALE and the guard it aimed at
  went unproven. Its entry-point-preamble mutation now names an entry point
  `tests/c_smoke.c` does not call before init, so the reflective sweep is the
  guard that answers rather than the C test that runs first.

- The three misuse sweeps hold floors of 700, 1300 and 175 probes, against 25,
  100 and 100. The real counts are 776 result-returning, 1422 pointer-taking
  and 192 enum-taking entry points, so most of a sweep could have disappeared
  without tripping one, and a sweep that quietly stops matching has no other
  output to notice. A 32nd mutation in `ci/check-abi-drift.sh` narrows the
  predicate deciding which entry points take a pointer and asserts the null
  sweep's floor is what fails; nothing had proved a floor could fire at all.
- `ci/check-abi-drift.sh` applies each mutation with a program written to a
  file and opened with `newline=''` at both ends. Reading it from stdin left
  `python3 - <file>` ambiguous wherever `python3` dispatches on a shebang —
  handed `tools/coverage.sh` to mutate, it ran that instead and the mutation
  was reported as a stale anchor — and the line-ending translation rewrote
  every file a mutation touched, which turned the ledger-example mutation
  into a failure for a reason it never aimed at. Both were verdicts about the
  tree drawn from a detail of the host.

- `ci/check-mirror.sh` compares the build-option combinations `ci/run.sh`
  executes against those `.github/workflows/ci.yml` executes, and refuses a
  difference. The header of `ci/run.sh` promised the two matched, nothing held
  the promise, and the workflow was a configuration ahead — which is how the
  debug-renderer break above survived. `ci/run.sh --full` also builds that
  configuration now, and mutation 33 of `ci/check-abi-drift.sh` proves the new
  guard fires.

### Build

- `src/abi_check.zig` refuses any extern fn, callback typedef or callback
  field whose f32/f64 parameter follows more than 6 integer-class
  parameters (pointers, ints, enums, bools). Zig 0.16.0's self-hosted
  x86-64 backend — the default for a consumer's Debug build targeting
  x86_64-linux — miscompiles the CALLER of such a signature, loading the
  float from the wrong register and shifting later argument slots; measured
  2026-09-01 by disassembling minimal reproductions. Exactly 6 is safe and
  7 is not, and the conservative bound also covers the aarch64 backend's
  larger register file. The desc redesign above removed the three
  signatures over the bound; this is what keeps the class from returning.
- The three misuse sweeps are one function per module behind a thin driver
  rather than one function looping over every module, and which declarations
  a sweep visits has one home instead of a copy of the filter in each of the
  three. An `inline for` unrolls into the function that contains it, so each
  sweep had been a single function body holding a probe for every entry point
  in the ABI, and a compiler's peak memory is superlinear in one function's
  size: compiling the Debug test binary peaked at 25G of resident memory,
  which no hosted runner has, so the Debug steps of the workflow and the
  mutations that build on them could not run there at all. The same compile
  peaks at 882M split per module, with the same 503 tests.
- `build.zig` writes and installs `zjolt_config.h`, carrying the two options
  that change the ABI and `ZJOLT_SHARED`; `zjolt_core.h` includes it when it
  is there. Zig propagates include paths and libraries across a link but never
  -D flags, so a C or C++ host had to repeat `-Ddouble_precision` and
  `-Dobject_layer_bits` by hand and got `ZJOLT_RESULT_CONFIG_MISMATCH` at
  init, or a lost `__declspec(dllimport)`, when it did not. A consumer reading
  `ffi/` straight out of the tree still gets the header defaults, and
  `zjoltInit` still refuses a mismatch.
- `tests/consumer` forwards both ABI options, and `ci/run.sh` runs it a second
  time with `-Ddouble_precision`. Its C half now compares the library's
  reported layout against its own `sizeof(ZJoltReal)`, `sizeof(ZJoltObjectLayer)`
  and `ZJOLT_CONFIG_ID`, which is what makes that second run a test rather
  than a repetition.
- No `-Dsimd` option, stated in the README and in `build.zig` where someone
  would look for one: Jolt derives every `JPH_USE_*` from the compiler's own
  predefines, so `-Dcpu=` already is the lever and a second one could
  disagree with it. `JPH_DISABLE_CUSTOM_ALLOCATOR` is refused for a different
  reason — it compiles away the five function pointers that ARE
  `init(.{ .allocator = ... })`.

- The hosted workflow runs `-Ddebug_renderer=true` for both the Zig suite and
  the C ABI test. The option was declared and never built there.
- `ci/run.sh --full` and the hosted workflow build `tests/consumer` on the
  MSVC ABI, and the workflow also runs it with `-Ddouble_precision`. Resolving
  the installed headers from a dependent build is the one code path neither
  the in-repo suite nor the C smoke test walks, and it had been proved on one
  Windows ABI and one configuration only.
