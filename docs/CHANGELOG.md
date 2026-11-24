# Changelog

### Hardening, Pass-Through Factory & Code-Side CAD Repair (`v2027.4.0-alpha1` — 61 JUnit 5 + 9 Python Tests)
- **Seamless YAMS Pass-Through Factory (`YacSimYamsFactory`)**: Pass a YAMS `*Config` + `SmartMotorController` to `createArm` / `createElevator` / `createPivot` / `createFlyWheel` (or `attachSwerve` / `registerComponent`) and get back the plain YAMS mechanism. Automatically short-circuits with zero CAD I/O on real robots (`RobotBase.isReal()`), while auto-binding, chaining nested URDF joints, and stepping `simIterate()` / `updateTelemetry()` in simulation.
- **Nested URDF Kinematic Chaining (`YACS-AUDIT-01`)**: `YamsCommands3Bridge.bindFromManifest(...)` and new `chainFromManifest(...)` automatically compose live ancestor poses and intermediate fixed transforms in any binding order.
- **Zero-Allocation Broadphase & 15-Axis SAT (`YACS-AUDIT-02`)**: `ObbCollisionDetector` and `CadRobotSimulator` now run bounding-sphere broadphase rejection and allocation-free primitive SAT/envelope checks.
- **glTF Scene-Graph Node Transforms (`YACS-AUDIT-03`)**: `MeshParser.parseGltfJson(...)` recursively traverses glTF `scenes`/`nodes` transforms (`matrix` or `T*R*S`) for exact multi-node OBBs and grounding.
- **Automatic Moving-Subtree Baking, `attachLinksToJoint` & Selective `NOSIMPLIFY` (`YACS-AUDIT-04`)**: Added `CadUrdfOptions.bakeUnboundLinksIntoChassis(true)` (automatically baking unbound children into their nearest kept moving parent's `.glb` or into `root.glb`), `attachLinksToJoint(joint, patterns...)` (re-parenting mis-mated `root`-attached parts into moving mechanism `.glb`s in Java), and `noSimplifyMeshPatterns(...)`.
- **True Resting Initial Poses (`YACS-AUDIT-05`)**: `CadRobotSimulator` initializes its snapshot from true URDF resting poses before the first `simIterate()`.
- **Roller Collision Preservation & Fuzzy Alias Hints (`YACS-AUDIT-08`)**: Bound `CONTINUOUS` rollers keep collision disabled when `disableUnboundCollisions(true)` is set, and `YacSimDiagnostic.suggestBestJointMatch(...)` suggests likely CAD aliases for unmatched Java names, and `YacSimDiagnostic.formatReport(...)` prints spatial `@ (x, y, z)m` coordinates for each movable joint.
## [2027.5.0-alpha1] — Dual-Pose Ground Truth, Direct DataLog, 3D RK4 Ballistics & Field Exporter

### Added (67 JUnit 5 + 10 Python Automated Tests)
- **Dual-Pose Ground Truth (`YACS-AUDIT-06`)**: Added `CadRobotConfig.withSimTruthPoseSupplier` / `withSimTruthPose2dSupplier`, `CadRobotSimulator.getLatestSimTruthPose3d()`, `/YACSim/SimTruthPose3d` publishing in `AdvantageScopePublisher`, and `YamsCommands3Bridge.attachSwerveModulesWithTruth(...)` wiring YAMS 2027's `SwerveDrive::getSimPose` alongside noisy fused odometry (`drive::getPose`).
- **Direct WPILib `DataLog` Struct Recording (`YACS-AUDIT-07`)**: Added `CadRobotConfig.withDataLogEnabled(boolean)` so `AdvantageScopePublisher` appends `StructLogEntry` / `StructArrayLogEntry` directly to `DataLogManager.getLog()` (`.wpilog`), preserving AdvantageScope 3D logs even when NT4 is turned off in competition.
- **3D RK4 Projectile Ballistics (`yacsim.core.ballistics`)**: Added zero-dependency `ProjectileSpec` (with `frcFuel2026()`, `frcNote2024()`, `frcCoral2025()`, and `vacuum()` presets), `FieldObstacle3d` (with `loadFieldManifest(Path)`), `ProjectileState`, `ProjectilePhysicsSim` (4th-order Runge-Kutta gravity + quadratic drag + Magnus spin lift + floor bounce + 3D OBB obstacle/target collisions + `predictArc(...)`), and `TrajectoryArcPublisher` (`/YACSim/Ballistics/PredictedArc` & `ActiveProjectiles` as `Pose3d[]`).
- **Onshape / CAD Field Exporter (`tools/onshape-field-exporter/`)**: Pure-Python CLI generating YAECE `SeasonConfig` JSON (Andymark & Welded variants), PathPlanner `navgrid.json`, and YACSim 3D field OBB manifests (`yacsim-field.json`) from a single CAD field spec.

## [2027.4.0-alpha1] — Phase 3 Step 2: Code-Side Fastener Filtering, Joint Aliasing, Diagnostic Auto-Binder & Swerve Overlay

### Added
- **Code-Side Fastener & Lag Fix (`CadUrdfOptions`)**: Added `CadUrdfOptions.defaults()` and `CadUrdfOptions.recommended()` (`excludeFasteners(true)` + `disableUnboundCollisions(true)`), `excludeMeshPatterns(...)`, `withJointAlias(...)` / `withJointAliases(...)`, and `keepOnlyJoints(...)`, integrated across `CadManifestLoader.loadUrdf*` and `CadRobotConfig.fromUrdf*`. Automatically re-enables OBB collision on bound mechanisms while skipping unbound static links at 50 Hz.
- **Startup CAD-to-Code Diagnostic & One-Call Auto-Binder**: Added `YacSimDiagnostic` (`formatReport`, `printReport`, `BindingReport`) and `YamsCommands3Bridge.autoBind(CadRobotConfig, Map<String, Object>)` to bind YAMS `Arm`/`Elevator`/`Pivot`/`FlyWheel`/`SmartMotorController` mechanisms in one call and report unmatched names with copy-pasteable alias hints.
- **Swerve Wheel Spin & Steer Module Overlay**: Added `YamsCommands3Bridge.attachSwerveModules(...)` overloads (both YAMS `SwerveDrive` and drivetrain-agnostic suppliers) that reuse an existing URDF/manifest chassis while animating 4 rotating steer forks and spinning wheels.
- **Tests & Docs**: Added 4 new JUnit 5 tests in `CadManifestLoaderTest` (**51 JUnit 5 tests across 8 classes** + 9 Python exporter tests) and updated `docs/ONSHAPE_URDF_GUIDE.md`, `docs/API_REFERENCE.md`, and `docs/UNIT_TESTS.md`.

## [2027.3.0-alpha1] — Phase 3 Step 1: Native Onshape URDF `.zip` Support, Safe Fallbacks & YAMS `getPose3d()` Bridge

### Added
- **Zero-Dependency Native Onshape URDF `.zip` Import** (`CadManifestLoader.loadUrdfZip`, `loadUrdfOrZip`, `UrdfLoadResult`, `CadRobotConfig.fromUrdfZip`, `fromUrdfOrZip`): Drop Onshape's built-in `Assembly -> Export -> Format: URDF (Geometry format: GLB)` `.zip` directly into `src/main/deploy/yacsim/` with zero Python and zero API keys. Includes Zip-Slip path traversal protection, `__MACOSX/` skipping, `./` relative mesh prefix normalization, and multi-`<visual>` mesh selection. Strictly additive—all existing Manual Java, AdvantageScope Dir, `yacsim-robot.json`, and Python Onshape Exporter workflows remain unchanged.
- **Pure-Java Multi-`<visual>` GLB Merger & ROS Package Root Detection** (`yacsim.core.cad.GlbMerger`, `CadManifestLoader.resolveUrdfBaseDir`): Automatically resolves Onshape's ROS package layout (`<pkg>/urdf/*.urdf` + `<pkg>/meshes/*.glb`) and merges multi-`<visual>` `.glb` links into single deduplicated binary `.glb` assets with each `<visual><origin>` preserved as a glTF 2.0 node.
- **Safe `*Or(...)` Fallback Getters**: Added 14 fallback getters on `CadJointManifest` (`minLimitAngleOr`, `maxLimitAngleOr`, `softMinLimitAngleOr`, `softMaxLimitAngleOr`, `startingAngleOr`, `minLimitDistanceOr`, `maxLimitDistanceOr`, `softMinLimitDistanceOr`, `softMaxLimitDistanceOr`, `startingDistanceOr`, `lengthOr`, `wheelRadiusOr`, `wheelDiameterOr`, `momentOfInertiaOr`) plus `CadRobotManifest.getLinkMassOr` and `getJointMomentOfInertiaOr` so lightweight URDFs missing optional mate limits or materials never crash at startup.
- **YAMS 2027 Native `getPose3d()` Bridge**: Added `YamsCommands3Bridge.fromYamsMechanism(Arm|Elevator|Pivot|FlyWheel)` (with optional `Transform3d` CAD mount offset).
- **Tests & Documentation**: Added 3 new JUnit 5 tests in `CadManifestLoaderTest` (**47 JUnit 5 tests across 8 classes** + 9 Python exporter tests) and added `docs/ONSHAPE_URDF_GUIDE.md`.

## [2027.2.1-alpha1] — Live Onshape Hardening, CAD-to-Code Naming Conventions & FRC 2181 Example

### Added
- **FRC Team 2181 GEARS Example (`examples/commands3/gears_2181_2027/`)**: Full port of `FRCteam2181/2026-Comp-Code` to WPILib 2027 Commands v3, REVLib 2027 (`SparkMax`/`SparkFlex` with `CANPort.CAN_S0`), YAMS 2027 (`SparkWrapper`), and YACSim (`IntakeArm`, `Turret`, `Climber`, `Shooter`, `Spindexer`, `TopIntake`, `Input`).
- **Forgiving CAD-to-Code Name Matching**: `CadRobotManifest.normalizeName(...)` + case/separator/`yacsim_`-prefix-insensitive fallback on `getJoint`, `getLink`, `hasComponent`, `getComponentPoseSupplier`, and `withComponentPoseSupplier` (with available-name lists in error messages).
- **Onshape Exporter Live-CAD Upgrades**: Automatic text `.gltf`/`.zip` -> binary GLB 2.0 conversion (~39% smaller), native Z-up default (`--gltf-yup` optional), 1-DOF actuated mate filter (skips `PLANAR`/`PARALLEL`/`TANGENT`), `coarse` default with `>30 MB` warning, `--list-mates`, `--only-named`, `--rename`, and `--prefix yacsim_`.
- **Documentation & Tests**: Added `docs/CAD_CODE_NAMING_CONVENTIONS.md`, expanded `docs/ONSHAPE_EXPORTER_GUIDE.md` with Python + Onshape API key setup and troubleshooting; **42 JUnit 5 tests across 9 classes** + **9 Python exporter unit tests**.

## [2027.2.0-alpha1] — Phase 2 Onshape Exporter & 3D Spinning Roller Visualization

### Added
- **Zero-Dependency Onshape Exporter (`tools/onshape-exporter/`)**: Pure Python 3 standard-library CLI (`yacsim_onshape_exporter.py`) calling Onshape REST API v6 (`GetAssemblyDefinition`, `GetAssemblyMassProperties`, `GetAssemblyBoundingBoxes`, async GLTF translation) to generate `yacsim-robot.json` + `.glb` assets, plus `--augment-rollers` offline mode and `roller_glb_generator.py` for high-contrast, color-striped 3D spinning roller `.glb` meshes.
- **Automatic Parent-Mounted Roller 3D Binding**: Added `CadRobotConfig.getComponentPoseSupplier(String)` and enhanced `YamsCommands3Bridge.bindFromManifest(config, jointName, rollerMotor)` so rollers parented to moving mechanisms (such as Darwin's `IntakeRoller` on `Slamtake`) automatically ride on their parent and spin in AdvantageScope 3D Field.
- **Darwin Manifest Example With 5 Live Spinning 3D Rollers**: `examples/commands3/darwin_6328_manifest_2027/` now ships with `model_4.glb`-`model_8.glb` (`FlywheelMesh`, `KickerFrontMesh`, `KickerBackMesh`, `HopperRollerMesh`, `IntakeRollerMesh`) wired directly in `yacsim-robot.json`.
- **Build & Test Fixes**: Fixed `vendordepJson.gradle` to declare `pubVersion` as a task input; switched `YamsMechanismBridge.fromAngularVelocity` to HAL-free `System.nanoTime()`; added 1 Java test (**41 JUnit 5 tests across 9 classes**) + 4 Python exporter unit tests.
- **Documentation**: Added `docs/ONSHAPE_EXPORTER_GUIDE.md` and `tools/onshape-exporter/README.md`.

## [2027.1.0-alpha1] — Phase 1 CAD-to-YACSim Automation (Manifest & URDF)

### Added
- **`yacsim.core.manifest` Package**: Zero-dependency `yacsim-robot.json` manifest schema (`CadRobotManifest`, `CadLinkManifest`, `CadJointManifest`, `CadJointType`) and XXE-hardened URDF XML importer (`CadManifestLoader.loadJson`, `loadUrdf`, `toPrettyJson`, `writeJson`).
- **Exact CAD Mass & Inertia Override**: `CadAssetConfig.withMassPropertiesOverride(CadMassProperties)` honored directly in `MeshParser` (and cache key), replacing bounding-box estimates with exact CAD mass, CoM, and principal inertia (`Ixx`/`Iyy`/`Izz`).
- **Manifest & URDF Robot Factories**: `CadRobotConfig.fromManifest(Path)`, `fromManifest(CadRobotManifest, Path)`, `fromUrdf(Path)`, `getManifest()`, `hasComponent(String)`, and name-based `withComponentPoseSupplier(String, Supplier<Pose3d>)`.
- **One-Line Manifest YAMS Binding**: `YamsCommands3Bridge.bindFromManifest(...)` and `poseSupplierFromManifest(...)` for `Arm`, `Elevator`, `Pivot`, and `SmartMotorController` (rollers/flywheels).
- **New Manifest-Driven Example (`examples/commands3/darwin_6328_manifest_2027/`)**: Full FRC 6328 *Darwin* simulation driven by `src/main/deploy/yacsim/darwin/yacsim-robot.json` referencing Darwin's Onshape assembly, keeping only CAN IDs, motor types, gear ratios, and PID/FF gains in Java.
- **Documentation & Tests**: Added `docs/CAD_MANIFEST_GUIDE.md` and `CadManifestLoaderTest` (5 new tests; **40 JUnit 5 tests across 9 test classes**).

# Changelog

All notable changes to YACSim are documented here.

## `v2027.0.7-alpha1` — Spinning Wheels, Intake Rollers & Flywheel Simulation

### Added
- **Spinning Wheel, Roller & Flywheel Adapters**:
  - Added `YamsMechanismBridge.fromSpinningWheel(...)`, `fromAngularVelocity(...)`, and `fromMountedRoller(...)` (for intake bars riding on deploying arms).
  - Added `YamsCommands3Bridge.fromYamsFlyWheel(...)`, `fromYamsRollerMotor(...)`, and `fromMountedYamsRoller(...)`.
- **Roller & Flywheel Telemetry Pipeline**:
  - Added `CadRollerTelemetryConfig` and `CadRobotConfig.addRollerTelemetry(...)` to publish `/<tableName>/Rollers/<name>/{VelocityRPM,SurfaceSpeedMPS,Spinning}` in `AdvantageScopePublisher`—supporting both standalone wheel `.glb`s and baked-in CAD rollers.
- **Full FRC 6328 *Darwin* YAMS 2027 Conversion**:
  - Ported Darwin's original `Flywheel` (4× Kraken X60 FOC), `Kicker` (`KickerFront` 2.5" + `KickerBack` 1.58"), `HopperRoller` (2× Kraken X60 FOC, 4:1), and `Slamtake` `IntakeRoller` (1× Kraken X60 FOC, 13 V) to YAMS 2027 `FlyWheel` + `TalonFXWrapper` and wired live telemetry into `DarwinCadSim`.
- **Javadoc & Tests**:
  - Updated `package-info.java` links in `advantagescope`, `cad`, and `physics`; expanded JUnit 5 suite from 32 to 35 tests.

## `v2027.0.6-alpha1` — AdvantageScope Import, CAD Presets, Linkages, Swerve Adapter & Managed Sim

### Added
- **AdvantageScope Directory Importer**: `CadRobotConfig.fromAdvantageScopeDir(Path)` and `withComponentPoseSupplier(index [, newName], supplier)` import existing `Robot_<Name>/config.json` bundles in one line.
- **CAD-Tool Coordinate Presets**: `CadCoordinateSystem` (`WPILIB_NWU`, `ONSHAPE_GLTF`, `SOLIDWORKS_MM`, `FUSION_360_MM`, `BLENDER_GLTF`) and `CadAssetConfig.withCoordinateSystem(...)`.
- **Multi-Stage Elevators & 4-Bar Linkages**: `fromCascadingElevator(...)` and `fromFourBar(...)` on both `YamsMechanismBridge` and `YamsCommands3Bridge`.
- **Direct YAMS `SwerveDrive` Adapter**: `YamsCommands3Bridge.fromYamsSwerve(SwerveDrive, ...)`.
- **Managed Sim `Notifier` & `MeshParser` Cache**: `CadRobotMechanism.startSimNotifier([periodSeconds])` and thread-safe `MeshParser` metadata caching (`MeshParser.clearCache()`).
- **Camera Frustums & Starter Layouts**: `CadCameraConfig`, `CadRobotConfig.addCamera(...)`, `AdvantageScopeConfigGenerator.buildStarterLayoutJson(...)`, and `exportStarterLayout(...)`.
- **Tests**: Expanded JUnit 5 suite from 27 to 32 tests across all 8 test classes.

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
