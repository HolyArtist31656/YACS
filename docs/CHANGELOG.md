# Changelog

All notable changes to YACSim are documented here.

## `v2027.0.5-alpha1` — CAD-to-YAMS Physics, Multi-Primitive Sub-OBBs & AdvantageScope Auto-Sync

### Added
- **CAD-to-YAMS Mass & MOI Helpers (Option 4)**:
  - Added `CadMassProperties.mass()`, `momentOfInertiaAboutPivotKgM2(pivot, axis)`, and `momentOfInertiaAboutPivot(pivot, axis)` using the parallel-axis theorem.
  - Added `YamsMechanismBridge.estimateMass`, `estimateArmMoi`, and `estimateArmMoiKgM2` for directly seeding YAMS `SmartMotorControllerConfig` and `ElevatorConfig`.
- **Multi-Primitive `.glb` Sub-OBBs (Option 5)**:
  - Extended `MeshMetadata` with `collisionSubBounds` (backwards-compatible 6-arg constructor preserved) and updated `MeshParser` to extract per-primitive sub-OBBs up to `MAX_SUB_OBBS = 64`.
  - Added `ObbCollisionDetector.testMultiObbCollision` and `testMultiEnvelopeViolation` and wired `CadRobotSimulator` to use sub-OBBs automatically.
- **AdvantageScope User Assets Auto-Install (Option 6)**:
  - Added `AdvantageScopeConfigGenerator.resolveDefaultUserAssetsDir()` (Windows/macOS/Linux), `installToAdvantageScope(...)`, and `CadRobotConfig.withAutoInstallToAdvantageScope()`.
- **Auto-Grounding Chassis CAD (Option 7)**:
  - Added `CadRobotConfig.autoGroundChassis()` and `CadRobotSimulator.getChassisGroundOffsetMeters()`, automatically shifting chassis `minZ` flush to `0.0` in exported AdvantageScope `config.json` files.
- **Tests**: Expanded JUnit 5 suite from 21 to 27 tests across all 8 test classes.

## `v2027.0.4-alpha1` — Streamlined Setup & Auto-Calibration

### Added
- **Direct YAMS Mechanism Adapters**:
  - Added `yacsim.core.yams.MotionAxis` (`X`, `Y`, `Z`) and `YamsMechanismBridge.fromAngleSupplier(...)` / `fromDistanceSupplier(...)` (with optional inversion and `Transform3d` zero offsets).
  - Added `yacsim.commands3.YamsCommands3Bridge` with direct `fromYamsArm`, `fromYamsElevator`, and `fromYamsPivot` overloads—eliminating manual `Pose3d` math in robot code.
- **Auto-Derived Extension Envelope**:
  - Added `CadRobotConfig.withExtensionMargin(xyMarginMeters, maxHeightMeters)` and `CadRobotSimulator.getResolvedExtensionEnvelope()` to compute the legal FRC extension box directly from the parsed chassis CAD bounds.
- **Auto-Calibrated Resting OBB Overlaps**:
  - Added `CadRobotConfig.ignoreRestingOverlaps()` / `withIgnoreRestingOverlaps(boolean)` and `CadRobotSimulator.getAutoIgnoredRestingPairs()` to sample initial poses once at `t = 0` and automatically silence resting sibling/chassis OBB overlaps while preserving `ExtensionEnvelope` checks.
- **Tests**: Expanded JUnit 5 suite from 15 to 21 tests (`YamsMechanismBridgeTest` + 3 new `CadRobotSimulatorTest` cases).

## `v2027.0.3-alpha1` — Initial Public Alpha

### Added
- **Core CAD Simulation (`yacsim:YACSim-core`)**:
  - Header-only `.glb`/`.gltf` and streaming `.obj`/`.stl` parser (`MeshParser`) with bounding-box, triangle-count, and `CadMassProperties` extraction.
  - 15-axis Separating Axis Theorem OBB collision detector (`ObbCollisionDetector`) supporting component-to-component, component-to-chassis, and `ExtensionEnvelope` checks.
  - Symmetric `CadRobotConfig.ignoreCollisionPair(componentA, componentB)` supporting sibling mechanisms, `"Chassis"`, and `"ExtensionEnvelope"`.
  - Ordered multi-axis zeroed rotations (`CadAssetConfig.addZeroedRotation(axis, degrees)`) exported verbatim to AdvantageScope `config.json`.
  - `YamsMechanismBridge` and `YamsDrivetrainBridge` for YAMS 2027 mechanisms and swerve drivetrains.
  - Lock-free `volatile SimSnapshot` threading between `simIterate()` and `updateTelemetry()`.
  - Zero-dependency `MinimalJson` parser and pretty-printer.
- **Commands v3 Bindings (`yacsim:YACSim-commands3`)**:
  - `CadRobotMechanism` lifecycle wrapper and `CadCollisionTriggers` (`onAnyCollision`, `onCollisionBetween`, `onExtensionViolation`).
- **Distribution & CI**:
  - `yacsim_commands3.json` declaring `CommandsV3.json` and `yams_commands3.json` in `requires`.
  - GitHub Actions CI (JDK 25, Spotless, JUnit 5 tests, Javadoc) and GitHub Pages Maven/vendordep/Javadoc publishing.
- **Examples**:
  - `examples/commands3/arm_and_elevator_2027` — Starter swerve + elevator + arm simulation with bundled `.glb` meshes.
  - `examples/commands3/darwin_6328_2027` — Real-robot port of FRC 6328 *Darwin* (2026) using YAMS 2027 + YACSim, verified under `-Xmx256m`.
