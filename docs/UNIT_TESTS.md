# Unit Tests

YACSim includes **9 JUnit 5 test classes (67 tests)** under `vendordep/src/test/java/yacsim/` plus **10 Python exporter unit tests** (9 under `tools/onshape-exporter/test_exporter.py` + 1 under `tools/onshape-field-exporter/test_field_exporter.py`) (**77 automated tests total**). All tests run headless on desktop without requiring a roboRIO or SystemCore.

## Running Tests

### Run the Full Java Suite

```powershell
cd vendordep
.\gradlew test "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

### Run a Single Test Class

```powershell
.\gradlew test --tests "yacsim.core.manifest.CadManifestLoaderTest" "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

### Run the Python Exporter Suite

```powershell
python -m pytest tools/onshape-exporter/test_exporter.py
python -m unittest discover -s tools/onshape-field-exporter -p "test_*.py" -v
```

## Java Test Classes (67 JUnit 5 Tests)

| Test Class | Package | Tests | What It Verifies |
| --- | --- | --- | --- |
| `CadRobotMechanismTest` | `yacsim.commands3` | 1 | `mechanismStepsSimulatorSafely()` — Instantiates `CadRobotMechanism` with a Commands v3 `Mechanism`, steps `simulationPeriodic()` and `periodic()` cleanly, and verifies lifecycle cleanup. |
| `AdvantageScopeConfigGeneratorTest` | `yacsim.core.advantagescope` | 6 | `generatesValidAdvantageScopeJsonWithDrivetrainAndMechanisms`, `emitsOrderedMultiAxisRotationsInExactSequence`, `resolvesDefaultAdvantageScopeUserAssetsDir`, `appliesChassisGroundOffsetToConfigJson`, `exportsCamerasAndStarterLayoutJson`, and `importsExistingAdvantageScopeDirectoryAndBindsPoseSuppliers`. |
| `MeshParserTest` | `yacsim.core.cad` | 7 | `parsesGlb2HeaderWithoutLoadingGpuBuffers`, `allowsHighPolyGlbWithWarningWithoutCrashing`, `extractsPerPrimitiveSubBoundsForLShapedGlb`, `parsesObjWithMillimeterScale`, `parsesAsciiStlUnitCube`, `rejectsMeshExceedingTriangleBudget`, and `appliesCadCoordinateSystemPresetAndCachesRepeatedParses`. |
| `CadDrivetrainTest` | `yacsim.core.drivetrain` | 1 | `simulatesYamsSwerveDrivetrainAndChassisCollision` — Wires `YamsDrivetrainBridge.fromYamsSwerve` (4 steer forks + 4 wheels + 1 arm = 9 articulated poses), verifies gyro pitch/roll in `RobotPose3d`, and asserts `Arm <-> Chassis` collision detection. |
| `CadRobotSimulatorTest` | `yacsim.core.model` | 7 | `ignoresConfiguredCollisionPairsSymmetrically`, `ignoresExtensionEnvelopeForConfiguredComponentOnly`, `autoDerivesExtensionEnvelopeFromChassisBounds`, `autoIgnoresRestingSiblingOverlapsWhileDetectingNewDynamicCollisions`, `ignoreRestingOverlapsDoesNotSilenceExtensionEnvelopeViolations`, `autoGroundsChassisAndUsesMultiPrimitiveSubObbs`, and `samplesRollerTelemetryInSimSnapshot`. |
| `ObbCollisionDetectorTest` | `yacsim.core.physics` | 4 | `detectsSeparatedBoxes`, `detectsOverlappingRotatedBoxes`, `detectsExtensionEnvelopeViolation`, and `multiObbEliminatesFalsePositiveInsideLShapedCavity`. |
| `MinimalJsonTest` | `yacsim.core.util` | 1 | `roundTripsNestedJsonCleanly` — Round-trips nested objects, arrays, strings, booleans, nulls, and numbers through `MinimalJson.parse` and `toPrettyJson`. |
| `YamsMechanismBridgeTest` | `yacsim.core.yams` | 8 | `fromAngleSupplierComputesPivotPoseAndSupportsInversion`, `fromDistanceSupplierComputesLinearPoseAlongAxisAndZeroOffset`, `chainParentChildComposesNestedMechanismTransforms`, `estimatesParallelAxisMomentOfInertiaAndMassFromMeshMetadata`, `fromCascadingElevatorScalesStageHeightsProportionally`, `fromFourBarKeepsEndEffectorLevelAlongArc`, `fromSpinningWheelAndMountedRollerComposeParentAndSpinRotation`, and `fromAngularVelocityIntegratesSpinAngleWithoutNaN`. |
| `BallisticsAndTelemetryTest` | `yacsim.core.ballistics` | 6 | Analytical vacuum RK4 verification, aerodynamic drag + Magnus backspin lift, 3D OBB target/obstacle hit detection, `yacsim-field.json` manifest loading, `predictArc` non-mutating aim arcs, and `SimTruthPose3d` + `withDataLogEnabled` verification. |
| `CadManifestLoaderTest` | `yacsim.core.manifest` | 26 | Manifest JSON round-trip, URDF XML & native Onshape `.zip` import, Zip-Slip protection, `GlbMerger`, unified Z-grounding, `CadUrdfOptions` (fastener filtering, subtree baking, `attachLinksToJoint`, `NOSIMPLIFY`), `YacSimDiagnostic`, `YamsCommands3Bridge` (`chainFromManifest`, `attachSwerveModules`), zero-alloc OBB broadphase/SAT, glTF node-transform bounds, and `YacSimYamsFactory` real-robot pass-through. |

### `yacsim.core.manifest.CadManifestLoaderTest` Breakdown (26 Tests)

- `roundTripsManifestJsonAndProvidesYamsUnits` — Full JSON load/serialize/reload round-trip, `sourceUrl` preservation, `getLinkMass`, `getJointMomentOfInertia`, angle/distance/diameter WPILib Units helpers, and camera parsing.
- `importsStandardUrdfWithPackagePrefixAndInertia` — Zero-dependency URDF XML parsing, `package://` mesh prefix stripping, root chassis detection, negative-axis inversion, and child-link inertia lookup.
- `meshParserAppliesExactCadMassPropertiesFromManifest` — Verifies `CadRobotConfig.fromManifest` passes exact CAD `CadMassProperties` through `CadAssetConfig` into `MeshParser`.
- `bindsPoseSupplierByComponentNameAfterUrdfImport` — Verifies `CadRobotConfig.fromUrdf` + `withComponentPoseSupplier(String, Supplier)` binding by link name.
- `rejectsInvalidJointTypeAndMissingComponentName` — Clear `IllegalArgumentException` diagnostics for invalid joint types and unknown component names.
- `exposesParentPoseSupplierForMountedRollersInManifest` — `CadRobotConfig.getComponentPoseSupplier(String)` and non-colliding roller mesh links for automatic parent-mounted roller chaining.
- `matchesCadAndCodeNamesCaseAndSeparatorInsensitively` — Forgiving CAD-to-code name resolution across case, spaces, underscores, and optional `yacsim_` prefixes.
- `importsNativeOnshapeUrdfZipWithNestedMeshesAndInertia` — Nested `.zip` archive mimicking Onshape's native URDF export with `./meshes/*.stl`, limits, and exact CAD mass/inertia.
- `rejectsZipSlipTraversalInUrdfZip` — Rejects `../` Zip-Slip path traversal entries with `IOException`.
- `safeFallbackGettersReturnDefaultsWhenUrdfOmitsLimitsOrMass` — Verifies `*Or(...)` fallback getters on `CadJointManifest` and `CadRobotManifest` return caller defaults cleanly when URDF limits or mass properties are omitted.
- `mergesMultiVisualOnshapeUrdfLinkAndResolvesRosPackageRoot` — Onshape ROS package root resolution (`urdf/` sibling of `meshes/`) and pure-Java `GlbMerger` deduplication + merging of multi-`<visual>` `.glb` links.
- `unifiedUrdfGroundingShiftsChassisAndChildJointsTogether` — Unified URDF Z-grounding lifts both the chassis GLB and all root-attached joints together by `-minZ` with zero separation.
- `cadUrdfOptionsExcludesFastenersAndCustomPatternsAndAliasesJoints` — `CadUrdfOptions.recommended()` strips fastener meshes/links, custom `MAXSwerve` patterns, aliases `revolute_1 -> Turret`, and auto-enables OBB collision on bind.
- `cadUrdfOptionsKeepOnlyJointsRetainsNestedFkRestingPose` — `keepOnlyJoints` trims visual links while preserving cumulative URDF forward-kinematics resting poses.
- `diagnosticReportHighlightsBoundAndUnboundJointsWithAliasHints` — `YacSimDiagnostic.formatReport` lists `[BOUND]`/`[UNBOUND]` joints, spatial `(X, Y, Z)` coordinates, and copy-pasteable `withJointAlias(...)` hints.
- `attachSwerveModulesOverlaysSpinningWheelsOntoUrdfChassis` — `YamsCommands3Bridge.attachSwerveModules` reuses a URDF chassis and attaches 4 rotating steer forks + 4 spinning wheels.
- `chainFromManifestTracksNestedParentMotionAndIntermediateStaticLinks` — Dynamic order-independent ancestor chaining through intermediate static links (`root -> turret_base -> intermediate_bracket -> hood_link`).
- `obbBroadphaseAndZeroAllocSatDetectOverlapAndRejectSeparatedBoxes` — Bounding-sphere broadphase rejection, allocation-free 15-axis SAT collision depth, and allocation-free extension-envelope violation checks.
- `meshParserAppliesGltfSceneGraphNodeTransformsToBounds` — Recursive glTF `scenes`/`nodes` transform traversal producing exact world bounds on multi-node `.glb` meshes.
- `cadUrdfOptionsBakeUnboundLinksIntoChassisAndTagNoSimplify` — `bakeUnboundLinksIntoChassis(true)` merges non-selected static links into `root.glb` at resting pose and tags matching nodes with `NOSIMPLIFY_`.
- `cadRobotSimulatorInitializesComponentPosesToUrdfRestingPosesBeforeSimIterate` — `CadRobotSimulator` constructor samples true URDF resting poses immediately before the first `simIterate()`.
- `continuousRollerCollisionPreservationAndFuzzyAliasSuggestions` — Preserves `collisionEnabled = false` on bound continuous rollers and validates `YacSimDiagnostic.suggestBestJointMatch(...)` (`"Turret"` → `"revolute_1"`).
- `yacSimYamsFactoryShortCircuitsOnRealRobotAndReturnsMechanismUnchanged` — Verifies `YacSimYamsFactory` returns mechanisms untouched with zero sim ticks or binding when `isSimulation` is `false`.
- `yacSimYamsFactoryBindsChainsAndStepsMechanismsInSimulation` — Verifies `YacSimYamsFactory` binds components, starts `CadRobotSimulator`, and steps both mechanism and simulator hooks in simulation.
- `bakeUnboundLinksAutomaticallyBakesSubtreeChildrenIntoMovingParentJoint` — Verifies automatic moving-subtree baking merges unbound URDF children of a kept moving joint into that parent joint's `.glb` in its local frame.
- `attachLinksToJointReparentsMismatedRootPartsIntoMovingMechanismGlb` — Verifies `CadUrdfOptions.attachLinksToJoint(...)` re-parents mis-mated `root`-attached parts into a moving mechanism's `.glb` in its local frame.

## Python Onshape Exporter Tests (`tools/onshape-exporter/test_exporter.py`, 9 Tests)

- URL parsing, invalid URL rejection, mock Onshape assembly/mass/bbox extraction with striped roller GLB generation, `--augment-rollers` offline round-trip, SE(3) 4×4 zeroed-transform round-trip, `YACSIM_ORIGIN` frame alignment, `PLANAR` mate filtering, `--rename`, and `--prefix yacsim_` stripping.

## Python Field Exporter Tests (`tools/onshape-field-exporter/test_field_exporter.py`, 1 Test)

- Verifies end-to-end export from field geometry JSON into YAECE `SeasonConfig` (Andymark & Welded variants), PathPlanner `navgrid.json`, and YACSim `yacsim-field.json` 3D field OBB manifests.
