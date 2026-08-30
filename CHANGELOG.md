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

### C ABI — changed

- `zjoltShapeCreateHeightField` takes `materials_capacity`, Jolt's
  `mMaterialsCapacity`. Without it the material list a repaint grows is
  reallocated under any query running in parallel.
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
- `layersFromInstance` builds the same three layer tables from a live value
  rather than from a type, so a layer scheme read from data at run time is
  expressible. `layersFromType` is unchanged; both now reject a type missing
  one of the four required declarations with a `@compileError` naming it.

### Fixed

- `hostStream` panicked on a zero-length read or write. An empty
  `JPH::Array`'s `data()` is NULL, and Jolt writes one per empty constraint
  list, so saving soft body shared settings through a host stream aborted in
  safe Zig before reaching the first field.

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

### Build

- The hosted workflow runs `-Ddebug_renderer=true` for both the Zig suite and
  the C ABI test. The option was declared and never built there.
