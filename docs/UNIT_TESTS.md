# Unit Tests

YACSim includes **8 JUnit 5 test classes (42 tests)** under `vendordep/src/test/java/yacsim/`. All tests run headless on desktop without requiring a roboRIO or SystemCore.

## Running Tests

### Run the Full Suite

```powershell
cd vendordep
.\gradlew test "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

### Run a Single Test Class

```powershell
.\gradlew test --tests "yacsim.core.model.CadRobotSimulatorTest" "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## Test Classes

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

### `yacsim.core.manifest.CadManifestLoaderTest` (7 tests)
- `roundTripsManifestJsonAndProvidesYamsUnits` — Verifies full JSON load/serialize/reload round-trip, `sourceUrl` preservation, `getLinkMass`, `getJointMomentOfInertia`, angle/distance/diameter WPILib Units helpers, and camera parsing.
- `importsStandardUrdfWithPackagePrefixAndInertia` — Verifies zero-dependency URDF XML parsing, `package://` mesh prefix stripping, root chassis detection, negative-axis inversion, and child-link inertia lookup.
- `meshParserAppliesExactCadMassPropertiesFromManifest` — Verifies `CadRobotConfig.fromManifest` passes exact CAD `CadMassProperties` through `CadAssetConfig` into `MeshParser`.
- `bindsPoseSupplierByComponentNameAfterUrdfImport` — Verifies `CadRobotConfig.fromUrdf` + `withComponentPoseSupplier(String, Supplier)` binding by link name.
- `rejectsInvalidJointTypeAndMissingComponentName` — Verifies clear `IllegalArgumentException` diagnostics for invalid joint types and unknown component names.
- `exposesParentPoseSupplierForMountedRollersInManifest` — Verifies `CadRobotConfig.getComponentPoseSupplier(String)` and non-colliding roller mesh links for automatic parent-mounted roller chaining.

- `matchesCadAndCodeNamesCaseAndSeparatorInsensitively` — Verifies forgiving CAD-to-code name resolution across case, spaces, underscores, and optional `yacsim_` prefixes.

### Python Onshape Exporter Tests (`tools/onshape-exporter/test_exporter.py`, 9 tests)
- URL parsing, invalid URL rejection, mock Onshape assembly/mass/bbox extraction with striped roller GLB generation, `--augment-rollers` offline round-trip, SE(3) 4x4 zeroed-transform round-trip, `YACSIM_ORIGIN` frame alignment, `PLANAR` mate filtering, `--rename`, and `--prefix yacsim_` stripping.
