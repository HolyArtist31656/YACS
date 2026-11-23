# Unit Tests

YACSim includes **8 JUnit 5 test classes (32 tests)** under `vendordep/src/test/java/yacsim/`. All tests run headless on desktop without requiring a roboRIO or SystemCore.

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
| `CadRobotSimulatorTest` | `yacsim.core.model` | 6 | `ignoresConfiguredCollisionPairsSymmetrically`, `ignoresExtensionEnvelopeForConfiguredComponentOnly`, `autoDerivesExtensionEnvelopeFromChassisBounds`, `autoIgnoresRestingSiblingOverlapsWhileDetectingNewDynamicCollisions`, `ignoreRestingOverlapsDoesNotSilenceExtensionEnvelopeViolations`, and `autoGroundsChassisAndUsesMultiPrimitiveSubObbs`. |
| `ObbCollisionDetectorTest` | `yacsim.core.physics` | 4 | `detectsSeparatedBoxes`, `detectsOverlappingRotatedBoxes`, `detectsExtensionEnvelopeViolation`, and `multiObbEliminatesFalsePositiveInsideLShapedCavity`. |
| `MinimalJsonTest` | `yacsim.core.util` | 1 | `roundTripsNestedJsonCleanly` — Round-trips nested objects, arrays, strings, booleans, nulls, and numbers through `MinimalJson.parse` and `toPrettyJson`. |
| `YamsMechanismBridgeTest` | `yacsim.core.yams` | 6 | `fromAngleSupplierComputesPivotPoseAndSupportsInversion`, `fromDistanceSupplierComputesLinearPoseAlongAxisAndZeroOffset`, `chainParentChildComposesNestedMechanismTransforms`, `estimatesParallelAxisMomentOfInertiaAndMassFromMeshMetadata`, `fromCascadingElevatorScalesStageHeightsProportionally`, and `fromFourBarKeepsEndEffectorLevelAlongArc`. |
