# Java API Reference

YACSim ships two Maven artifacts under group ID `yacsim`:
- `yacsim:YACSim-core` — Pure-Java CAD mesh parsing, OBB collision detection, drivetrain/mechanism bridges, and AdvantageScope export/telemetry.
- `yacsim:YACSim-commands3` — WPILib Commands v3 `CadRobotMechanism`, `CadCollisionTriggers`, `YamsCommands3Bridge`, and `YacSimYamsFactory`.

Full HTML Javadoc is hosted at **[https://holyartist31656.github.io/YACS/javadoc/](https://holyartist31656.github.io/YACS/javadoc/)**.

---

## `yacsim.commands3`

### `CadRobotMechanism`
Commands v3 lifecycle wrapper around `CadRobotSimulator` (`implements AutoCloseable`).

| Method | Description |
| --- | --- |
| `CadRobotMechanism(Mechanism mechanism, CadRobotConfig config)` | Builds and parses all CAD assets for the given robot config. |
| `CadRobotMechanism(Mechanism mechanism, CadRobotSimulator simulator)` | Wraps an existing `CadRobotSimulator`. |
| `void periodic()` | Publishes latest `SimSnapshot` to NetworkTables (`RobotPose`, `RobotPose3d`, `ComponentPoses`, `CollisionActive`, `CollisionDetails`). |
| `void simulationPeriodic()` | Steps suppliers and OBB collision checks (`simIterate()`). Thread-safe to call from a `Notifier`. |
| `Mechanism getMechanism()` | Returns the owning Commands v3 `Mechanism`. |
| `CadRobotSimulator getSimulator()` | Returns the underlying `CadRobotSimulator`. |
| `CadRobotMechanism startSimNotifier()` / `startSimNotifier(double periodSeconds)` | Starts a managed background `Notifier` in desktop simulation (safe no-op on real hardware; auto-closed on `close()`). |
| `void close()` | Stops any managed sim `Notifier` and closes NetworkTables publishers. |

### `YamsCommands3Bridge`
Direct adapters binding YAMS 2027 Commands v3 mechanisms to YACSim `Supplier<Pose3d>` without hand-written pose math.

| Method | Description |
| --- | --- |
| `static Supplier<Pose3d> fromYamsArm(Arm arm, Translation3d pivot, MotionAxis axis [, boolean inverted [, Transform3d zeroOffset]])` | Reads `arm.getAngle()` and rotates around `pivot` along `axis`. |
| `static Supplier<Pose3d> fromYamsElevator(Elevator elevator, Translation3d base, MotionAxis axis [, boolean inverted [, Transform3d zeroOffset]])` | Reads `elevator.getHeight()` and translates from `base` along `axis`. |
| `static Supplier<Pose3d> fromYamsPivot(Pivot pivot, Translation3d pivotPos, MotionAxis axis [, boolean inverted [, Transform3d zeroOffset]])` | Reads `pivot.getAngle()` and rotates around `pivotPos` along `axis`. |
| `static List<Supplier<Pose3d>> fromCascadingElevator(Elevator elevator, int stageCount, Translation3d base, MotionAxis axis)` | Builds proportional per-stage `Pose3d` suppliers (`1/N .. N/N`). |
| `static Supplier<Pose3d> fromFourBar(Arm arm, Translation3d pivot, double armLengthMeters, MotionAxis axis [, boolean inverted])` | Keeps a 4-bar end-effector level while following the arm's circular arc. |
| `static CadDrivetrainConfig fromYamsSwerve(SwerveDrive drive, ...)` | Binds `drive::getPose`, `drive::getGyroRotation3d`, and `drive::getModulePositions` directly. |
| `static CadRobotConfig attachSwerveModulesWithTruth(CadRobotConfig config, SwerveDrive drive, ...)` | (`v2027.5.0-alpha1`) Attaches 4 animated swerve modules AND binds `drive::getSimPose` to `/YACSim/SimTruthPose3d`. |
| `static Supplier<Pose3d> fromYamsFlyWheel(FlyWheel flywheel, Transform3d mountOffset)` | Wraps YAMS `FlyWheel.getPose3d()` with a CAD mounting offset. |
| `static Supplier<Pose3d> fromYamsRollerMotor(SmartMotorController motor, Translation3d axle, MotionAxis axis [, boolean inverted])` | Spins a roller/wheel `.glb` directly from `motor.getMechanismPosition()`. |
| `static Supplier<Pose3d> fromMountedYamsRoller(Supplier<Pose3d> parentPose, SmartMotorController motor, Translation3d localAxle, MotionAxis axis, boolean inverted)` | Mounts a spinning roller bar onto a moving parent mechanism. |
| `static Supplier<Pose3d> chainFromManifest(CadRobotConfig config, String jointName, Supplier<Pose3d> localPose)` | Dynamically composes a joint-local `Pose3d` supplier with its full URDF/manifest ancestor kinematic chain (order-independent). |
| `static YacSimDiagnostic.BindingReport autoBind(CadRobotConfig config, Map<String, Object> mechanisms)` | Forgiving one-call binder for YAMS `Arm`, `Elevator`, `Pivot`, `FlyWheel`, and `SmartMotorController` with fuzzy alias hints. |
| `static CadRobotConfig attachSwerveModules(CadRobotConfig config, SwerveDrive drive, ...)` / drivetrain-agnostic overload | Overlays 4 animated rotating steer forks and spinning wheels onto any URDF/manifest `CadRobotConfig`. |

### `YacSimYamsFactory` (`v2027.4.0-alpha1`)
Zero-overhead pass-through factory (`implements AutoCloseable`) accepting YAMS 2027 mechanism configs and returning plain YAMS mechanisms. Automatically short-circuits with zero CAD disk I/O on real hardware (`RobotBase.isReal()`).

| Method | Description |
| --- | --- |
| `static YacSimYamsFactory fromUrdfOrZip(Path path [, CadUrdfOptions options])` | Loads URDF `.zip`/`.urdf`/folder in simulation (default `CadUrdfOptions.recommended()`); returns no-op pass-through on real hardware. |
| `static YacSimYamsFactory fromManifest(Path manifestPath)` | Loads `yacsim-robot.json` in simulation; returns no-op pass-through on real hardware. |
| `static YacSimYamsFactory of(CadRobotConfig config [, BooleanSupplier isSim])` / `passthrough()` | Wraps an existing `CadRobotConfig` or creates an explicit no-op pass-through factory. |
| `YacSimYamsFactory withPublishOnRealRobot(boolean enabled)` | Optional toggle to keep lightweight pose publishing active on real hardware (default `false`). |
| `Arm createArm(String jointName, ArmConfig config, SmartMotorController motor)` | Constructs `new Arm(config, motor)`; in simulation also auto-binds (with nested parent chaining) and registers sim/telemetry hooks. |
| `Elevator createElevator(String jointName, ElevatorConfig config, SmartMotorController motor)` | Constructs `new Elevator(config, motor)` with automatic simulation binding and stepping. |
| `Pivot createPivot(String jointName, PivotConfig config, SmartMotorController motor)` | Constructs `new Pivot(config, motor)` with automatic simulation binding and stepping. |
| `FlyWheel createFlyWheel(String jointName, FlyWheelConfig config, SmartMotorController motor)` | Constructs `new FlyWheel(config, motor)` and binds spinning roller telemetry (keeping roller OBB collision disabled). |
| `SwerveDrive attachSwerve(SwerveDrive drive, CadAssetConfig fork, CadAssetConfig wheel, double radiusMeters, List<Translation3d> corners)` | Attaches 4 animated swerve modules in simulation; returns `drive` unchanged on real hardware. |
| `<T> T registerComponent(String jointOrLink, T mech, Supplier<Pose3d> localPose, Runnable simHook, Runnable telemetryHook)` | Generic pass-through registration for custom mechanisms or supplier-based components. |
| `Optional<CadRobotSimulator> startSimulator([boolean enableNT])` | Builds and caches `CadRobotSimulator` in simulation; returns `Optional.empty()` on real hardware. |
| `void simIterate()` / `void updateTelemetry()` / `void printDiagnosticReport()` / `void close()` | Steps all registered mechanisms + `CadRobotSimulator`, prints diagnostics, and releases NT4 resources. |

### `CadCollisionTriggers`
Factory methods returning WPILib Commands v3 `Trigger` instances.

| Method | Description |
| --- | --- |
| `static Trigger onAnyCollision(CadRobotSimulator sim)` | Fires when any OBB or envelope collision is active. |
| `static Trigger onCollisionBetween(CadRobotSimulator sim, String a, String b)` | Fires when components `a` and `b` are colliding (order-independent). |
| `static Trigger onExtensionViolation(CadRobotSimulator sim)` | Fires when any component breaches the configured `ExtensionEnvelope`. |

---

## `yacsim.core.model`

### `CadRobotConfig`
Fluent builder defining a simulated robot's chassis, articulated components, collision rules, and telemetry table.

| Method | Default / Notes |
| --- | --- |
| `CadRobotConfig(String robotName)` | Required; used for AdvantageScope `Robot_<robotName>` export. |
| `static CadRobotConfig fromAdvantageScopeDir(Path robotAssetDir)` | Imports an existing AdvantageScope `Robot_<Name>` directory (`config.json`, `model*.glb`, rotations, positions, cameras). |
| `withComponentPoseSupplier(int index [, String newName], Supplier<Pose3d>)` | Binds (and optionally renames) a component loaded via `fromAdvantageScopeDir`. |
| `addCamera(CadCameraConfig)` / `addCamera(name, Transform3d, fovDeg, w, h)` | Registers a 3D vision camera frustum for AdvantageScope `config.json`. |
| `addRollerTelemetry(CadRollerTelemetryConfig)` / `addRollerTelemetry(name, Supplier<AngularVelocity>, radiusMeters)` | Publishes live `VelocityRPM`, `SurfaceSpeedMPS`, and `Spinning` under `/<tableName>/Rollers/<name>/`. |
| `withDrivetrain(CadDrivetrainConfig cfg)` | Registers chassis asset, field pose supplier, and all swerve/wheel components. |
| `withChassisAsset(CadAssetConfig asset)` | Sets chassis CAD asset directly (when not using `CadDrivetrainConfig`). |
| `addComponent(CadComponentConfig comp)` | Adds an articulated mechanism component in AdvantageScope `model_N` order. |
| `withDrivetrainPoseSupplier(Supplier<Pose2d> s)` | Default: `Pose2d::new`. |
| `withSimTruthPoseSupplier(Supplier<Pose3d> s)` | Optional (`v2027.5.0-alpha1`); publishes ground-truth 3D pose to `/<tableName>/SimTruthPose3d` alongside fused odometry. |
| `withSimTruthPose2dSupplier(Supplier<Pose2d> s)` | Optional (`v2027.5.0-alpha1`); wraps a 2D ground-truth supplier (e.g., YAMS `SwerveDrive::getSimPose`) as `/<tableName>/SimTruthPose3d`. |
| `withDataLogEnabled(boolean enabled)` | Default: `false` (`v2027.5.0-alpha1`); writes `Pose3d`, `Pose3d[]`, and collision flags directly to WPILib `DataLogManager` (`.wpilog`) even when NT4 is off. |
| `withTelemetryTable(String table)` | Default: `"YACSim"`. |
| `withAdvantageScopeExportDir(Path dir)` | Optional; exports `Robot_<robotName>` bundle on simulator startup. |
| `withAutoInstallToAdvantageScope()` / `withAutoInstallToAdvantageScope(boolean)` | Default: `false`; installs `Robot_<robotName>` directly into the OS AdvantageScope `userAssets` directory. |
| `autoGroundChassis()` / `withAutoGroundChassis(boolean)` | Default: `false`; shifts chassis `minZ` to `0.0` in exported AdvantageScope `config.json`. |
| `withExtensionEnvelope(BoundingBox3d box)` | Explicit robot-frame legal extension volume (takes precedence over `withExtensionMargin`). |
| `withExtensionMargin(double xyMarginMeters, double maxHeightMeters)` | Auto-derives legal extension envelope from parsed chassis CAD bounds (`expandedForExtension`). |
| `withCollisionChecking(boolean enabled)` | Default: `true`. |
| `ignoreRestingOverlaps()` / `withIgnoreRestingOverlaps(boolean)` | Default: `false`; samples `t = 0` once at startup and auto-ignores resting sibling/chassis OBB overlaps (never ignores `ExtensionEnvelope`). |
| `ignoreCollisionPair(String a, String b)` | Symmetrically ignores OBB checks between `a` and `b` (also accepts `"Chassis"` and `"ExtensionEnvelope"`). |
| `isCollisionPairIgnored(String a, String b)` | Returns `true` if the pair is ignored. |
| `withDisableAdvantageScopeSimplification(boolean d)` | Default: `false`; sets `disableSimplification` in `config.json`. |

### `CadComponentConfig`
Defines a single articulated CAD part.

| Method | Description |
| --- | --- |
| `CadComponentConfig(String name, CadAssetConfig asset)` | Creates a named articulated component. |
| `withPoseSupplier(Supplier<Pose3d> supplier)` | Supplies the component's robot-relative `Pose3d`. |
| `withParentComponent(String parentName)` | Skips collision checks against direct parent (e.g., `"Chassis"`). |
| `withCollisionEnabled(boolean enabled)` | Default: `true`. |

### `CadRobotSimulator`
Core thread-safe simulation engine (`implements AutoCloseable`). Uses a lock-free `volatile SimSnapshot` record so `simIterate()` (e.g., on a 200 Hz `Notifier` thread) and `updateTelemetry()` (50 Hz main thread) never race.

| Method | Description |
| --- | --- |
| `CadRobotSimulator(CadRobotConfig config)` | Parses meshes, resolves envelope/resting overlaps, optionally exports AdvantageScope bundle, and opens NT4 publishers. |
| `CadRobotSimulator(CadRobotConfig config, boolean enableNetworkTables)` | Headless constructor (`enableNetworkTables = false`) for JUnit tests. |
| `void simIterate()` | Samples poses, runs OBB collision checks, and atomically publishes a new `SimSnapshot`. |
| `void updateTelemetry()` | Writes the latest `SimSnapshot` to NetworkTables. |
| `Optional<BoundingBox3d> getResolvedExtensionEnvelope()` | Returns the active extension envelope (explicit or auto-derived from chassis bounds). |
| `Set<String> getAutoIgnoredRestingPairs()` | Returns unmodifiable set of `"A <-> B"` labels auto-ignored at startup when `ignoreRestingOverlaps()` is enabled. |
| `double getChassisGroundOffsetMeters()` | Returns `-chassisMinZ` when `autoGroundChassis()` is enabled, or `0.0` otherwise. |
| `Map<String, Double> getLatestRollerVelocitiesRadPerSec()` | Returns latest sampled angular velocities (rad/s) for all registered rollers/flywheels. |
| `Pose2d getLatestRobotPose2d()` / `Pose3d getLatestRobotPose3d()` | Latest robot field pose. |
| `Pose3d[] getLatestComponentPoses()` | Defensive copy of latest component poses in registration order. |
| `List<CollisionEvent> getLatestCollisions()` | Immutable list of active collisions. |
| `boolean hasAnyCollision()` / `hasCollisionBetween(String a, String b)` / `hasExtensionViolation()` | Collision query helpers. |
| `Optional<MeshMetadata> getChassisMetadata()` / `getComponentMetadata(String name)` | Parsed mesh bounds, triangle counts, and mass properties. |
| `int getTotalTriangleCount()` | Sum of triangles across chassis and all articulated components. |

---

## `yacsim.core.cad`

### `CadAssetConfig`
Configuration for a single CAD file (`.glb`, `.gltf`, `.obj`, `.stl`).

| Method | Default / Notes |
| --- | --- |
| `CadAssetConfig(Path meshPath)` | Defaults: meter units (`1.0`), aluminum density (`2700.0 kg/m³`), `150_000` triangle budget. |
| `withUnitScaleToMeters(double scale)` / `withMillimeterUnits()` / `withInchUnits()` | Converts non-meter CAD exports into meters. |
| `withMaterialDensity(double kgPerM3)` / `withMassOverrideKg(double massKg)` | Controls computed `CadMassProperties`. |
| `withZeroedPosition(Translation3d pos)` | AdvantageScope `position` offset in meters. |
| `addZeroedRotation(String axis, double degrees)` | Appends an ordered `"x"`, `"y"`, or `"z"` rotation to AdvantageScope's `rotations` array. |
| `withZeroedRotation(Rotation3d rot)` | Fallback Euler zeroed rotation when `addZeroedRotation` is not used. |
| `withCoordinateSystem(CadCoordinateSystem preset)` | Applies unit scale + ordered rotations (`WPILIB_NWU`, `ONSHAPE_GLTF`, `SOLIDWORKS_MM`, `FUSION_360_MM`, `BLENDER_GLTF`). |
| `withMaxTriangleBudget(int maxTriangles)` | Raises/lowers triangle validation threshold. |

### `MeshParser`, `MeshMetadata`, `MeshFormat`, `CadMeshValidationException`
- `MeshParser.parse(CadAssetConfig)` returns cached `MeshMetadata(sourcePath, format, triangleCount, fileSizeBytes, localBounds, massProperties, collisionSubBounds)` (with `MeshParser.clearCache()`, `MAX_SUB_OBBS = 64`, and recursive glTF 2.0 `scenes`/`nodes` transform traversal).
- `MeshFormat`: `GLB`, `GLTF`, `OBJ`, `STL` (`MeshFormat.fromPath(Path)`).
- `CadMeshValidationException`: Thrown on missing files, unsupported formats, corrupt headers, or `.obj`/`.stl` files exceeding `maxTriangleBudget`.

---

## `yacsim.core.drivetrain` & `yacsim.core.yams`

- **`CadDrivetrainConfig`**: Configures chassis asset, field pose supplier, optional 3D gyro rotation supplier (pitch/roll), swerve modules (`addSwerveModule`), differential wheels (`addDifferentialWheel`), and chassis collision toggle.
- **`CadSwerveModuleConfig`**: Configures a swerve module's mounting `Translation3d`, wheel radius, steer fork asset, wheel asset, and `Supplier<SwerveModulePosition>`.
- **`YamsDrivetrainBridge.fromYamsSwerve(...)`**: One-call factory wiring a 4-module YAMS swerve drivetrain into `CadDrivetrainConfig`.
- **`MotionAxis`**: Enum (`X`, `Y`, `Z`) with `toRotation3d(double radians)` and `toTranslation3d(double meters)`.
- **`YamsMechanismBridge`**: `fromPoseSupplier`, `chainParentChild`, `fromAngleSupplier`, `fromDistanceSupplier`, `fromCascadingElevator`, `fromFourBar`, `fromSpinningWheel`, `fromAngularVelocity`, `fromMountedRoller`, `estimateMass`, `estimateArmMoi`, and `estimateArmMoiKgM2`.

---

## `yacsim.core.physics`

- **`BoundingBox3d`**: Immutable axis-aligned bounds record (`minX..maxZ`) with `center()`, `halfX/Y/Z()`, `sizeX/Y/Z()`, `scaled(double)`, `expandedForExtension(xyMargin, maxHeight)`, and `transformedCorners(Pose3d)`.
- **`CadMassProperties`**: Immutable mass, center of mass, and principal inertia (`ixx`, `iyy`, `izz`) record with `withTotalMassKg(double)`, `fromBoundingBox(box, density)`, `mass()`, `momentOfInertiaAboutPivotKgM2(pivot, axis)`, and `momentOfInertiaAboutPivot(pivot, axis)`.
- **`CollisionEvent`**: Immutable `(componentA, componentB, penetrationDepthMeters)` record with `involves(String)`.
- **`ObbCollisionDetector`**: Allocation-free 15-axis Separating Axis Theorem (SAT) with bounding-sphere broadphase (`boundingSphereRadius`, `spheresOverlap`, `testObbCollision`, `testEnvelopeViolation`, `testMultiObbCollision`, `testMultiEnvelopeViolation`).

---

## `yacsim.core.advantagescope` & `yacsim.core.util`

- **`AdvantageScopeConfigGenerator`**: `buildConfigJson(...)`, `exportAssetBundle(...)`, `resolveDefaultUserAssetsDir()`, `installToAdvantageScope(...)`, `buildStarterLayoutJson(...)`, and `exportStarterLayout(...)`.
- **`CadCameraConfig`**: Immutable camera frustum record `(name, robotToCamera, fovDegrees, resolutionWidthPx, resolutionHeightPx)`.
- **`AdvantageScopePublisher`**: Publishes `RobotPose`, `RobotPose3d`, `ComponentPoses`, `CollisionActive`, and `CollisionDetails` under the configured NetworkTables root table (default `/YACSim`).
- **`MinimalJson`**: Zero-dependency recursive-descent JSON parser (`parse(String)`) and pretty-printer (`toPrettyJson(Object)`).

---

## `yacsim.core.manifest` (Phase 1 CAD Automation)

| Class / Method | Purpose |
| :--- | :--- |
| `CadJointType` | Enum: `FIXED`, `REVOLUTE`, `PRISMATIC`, `CONTINUOUS` + `fromString(String)`. |
| `CadLinkManifest` | Record holding link name, relative `meshFile`, `unitScaleToMeters`, `zeroedPositionMeters`, `zeroedRotations`, optional exact `CadMassProperties`, `collisionEnabled`, and `toCadAssetConfig(Path)`. |
| `CadJointManifest` | Record holding joint `type`, `parentLink`, `childLink`, `originMeters`, `originRotation`, `zeroOffset`, `axis`, `inverted`, hard/soft limits, `startingPosition`, `lengthMeters`, `wheelRadiusMeters`, `momentOfInertiaKgM2`, plus WPILib Units helpers (`minLimitAngle()`, `maxLimitAngle()`, `softMinLimitAngle()`, `softMaxLimitAngle()`, `startingAngle()`, `minLimitDistance()`, `maxLimitDistance()`, `softMinLimitDistance()`, `softMaxLimitDistance()`, `startingDistance()`, `length()`, `wheelRadius()`, `wheelDiameter()`, `momentOfInertia()`). |
| `CadRobotManifest` | Top-level manifest record (`schemaVersion`, `robotName`, `sourceUrl`, extension margins, flags, `chassis`, `links`, `joints`, `cameras`) with `findLink`/`getLink`, `findJoint`/`getJoint`, `getLinkMass(String)`, and `getJointMomentOfInertia(String)`. |
| `CadManifestLoader` | Zero-dependency JSON & XXE-hardened URDF XML loader/serializer: `loadJson(Path)`, `parseJson(String)`, `loadUrdf(Path)`, `toPrettyJson(CadRobotManifest)`, `writeJson(CadRobotManifest, Path)`. |

### Manifest & Exact-Mass Extensions on Existing Classes

| Method | Class | Purpose |
| :--- | :--- | :--- |
| `withMassPropertiesOverride(CadMassProperties)` | `CadAssetConfig` | Overrides bounding-box mass estimation in `MeshParser` with exact CAD mass, CoM, and principal inertia tensor (`Ixx`/`Iyy`/`Izz`). |
| `fromManifest(Path)` / `fromManifest(CadRobotManifest, Path)` | `CadRobotConfig` | Populates chassis, links, exact mass properties, cameras, extension margins, and flags from `yacsim-robot.json`. |
| `fromUrdf(Path)` | `CadRobotConfig` | Imports a standard `.urdf` XML file directly into a `CadRobotConfig` with attached `CadRobotManifest`. |
| `fromUrdfZip(Path)` / `fromUrdfZip(Path, Path)` | `CadRobotConfig` | Extracts a native Onshape (or generic) URDF `.zip` archive (with Zip-Slip protection) and imports its `.urdf` + nested meshes. |
| `CadUrdfOptions` | `yacsim.core.manifest` | Code-side URDF fastener filtering (`recommended()`), `disableUnboundCollisions(true)`, `excludeMeshPatterns(...)`, `withJointAlias(...)`, `keepOnlyJoints(...)`, `bakeUnboundLinksIntoChassis(true)` (with automatic moving-subtree baking), `attachLinksToJoint(...)`, and `noSimplifyMeshPatterns(...)`. |
| `YacSimDiagnostic` | `yacsim.core.manifest` | Startup CAD-to-code reporter (`formatReport`, `printReport`, `suggestBestJointMatch`, `BindingReport`) highlighting `[BOUND]`/`[UNBOUND]` joints with spatial `@ (x, y, z)m` coordinates and fuzzy copy-pasteable alias hints. |
| `autoBind(CadRobotConfig, Map<String, Object>)` | `YamsCommands3Bridge` | One-call forgiving binder for YAMS `Arm`, `Elevator`, `Pivot`, `FlyWheel`, and `SmartMotorController` returning a `YacSimDiagnostic.BindingReport`. |
| `attachSwerveModules(...)` | `YamsCommands3Bridge` | Overlays 4 animated rotating steer forks and spinning wheels (YAMS `SwerveDrive` or generic swerve suppliers) onto any URDF/manifest `CadRobotConfig`. |
| `fromUrdfOrZip(Path)` | `CadRobotConfig` | Auto-detects and imports a `.zip` archive, `.urdf`/`.xml` file, or extracted URDF directory in one call. |
| `loadUrdfZip(...)` / `loadUrdfOrZip(Path)` / `resolveUrdfBaseDir(Path)` | `CadManifestLoader` | Zero-dependency URDF `.zip` extractor, ROS package root detector (`urdf/` + `meshes/`), and auto-discoverer returning `UrdfLoadResult(manifest, baseDir)`. |
| `GlbMerger.merge(List<VisualInstance>, Path)` | `yacsim.core.cad.GlbMerger` | Pure-Java GLB 2.0 merger combining multi-`<visual>` URDF links with binary buffer deduplication, unified Z-grounding, and selective `NOSIMPLIFY_` node tagging (`VisualInstance(..., boolean noSimplify)`). |
| `minLimitAngleOr(...)`, `lengthOr(...)`, `wheelDiameterOr(...)`, `momentOfInertiaOr(...)` (+ linear/soft/starting `*Or` variants) | `CadJointManifest` | Safe fallback getters returning caller defaults when lightweight URDFs omit mate limits, length, radius, or inertia. |
| `getLinkMassOr(String, Mass)` / `getJointMomentOfInertiaOr(String, MomentOfInertia)` | `CadRobotManifest` | Safe CAD mass/inertia lookups returning caller defaults when CAD materials/inertia are absent. |
| `fromYamsMechanism(Arm\|Elevator\|Pivot\|FlyWheel [, Transform3d])` | `YamsCommands3Bridge` | Wraps YAMS 2027's native `getPose3d()` directly without repeating pivot or motion-axis math. |
| `getManifest()` | `CadRobotConfig` | Returns `Optional<CadRobotManifest>` when created from a manifest or URDF. |
| `withComponentPoseSupplier(String, Supplier<Pose3d> [, boolean allowCollisionReenable])` | `CadRobotConfig` | Binds a live pose supplier by link/component name, with optional control over collision re-enable (set `false` for spinning rollers). |
| `bindFromManifest(CadRobotConfig, String, Arm\|Elevator\|Pivot\|SmartMotorController)` | `YamsCommands3Bridge` | Binds YAMS 2027 mechanisms and rollers directly from manifest joint origins, axes, inversions, and wheel radii. |
| `poseSupplierFromManifest(CadRobotManifest, String, Arm\|Elevator\|Pivot)` | `YamsCommands3Bridge` | Returns the manifest-derived `Supplier<Pose3d>` for composition or custom inspection. |
| `getComponentPoseSupplier(String)` | `CadRobotConfig` | Returns `Optional<Supplier<Pose3d>>` for automatic parent-mounted roller chaining in `YamsCommands3Bridge.bindFromManifest`. |
| `normalizeName(String)` | `CadRobotManifest` | Canonicalizes CAD/Java names (strips `yacsim_` prefix, spaces, hyphens, underscores, lowercases) for forgiving link/joint/component lookup. |

---

## `yacsim.core.ballistics` (`v2027.5.0-alpha1`)

| Class / Method | Purpose |
| :--- | :--- |
| `ProjectileSpec` | Immutable physical game-piece record (`frcFuel2026()`, `frcNote2024()`, `frcCoral2025()`, `vacuum(...)`). |
| `FieldObstacle3d` | Immutable 3D field obstacle or scoring target OBB (`axisAlignedObstacle`, `axisAlignedTarget`, `loadFieldManifest(Path)`). |
| `ProjectileState` | Immutable airborne projectile snapshot with `toPose3d()` velocity-aligned orientation. |
| `ProjectilePhysicsSim` | Pure-Java RK4 integrator (gravity, quadratic drag, Magnus spin lift, floor bounce, 3D OBB target/obstacle collisions, `launch`, `step`, `predictArc`, `getActiveProjectilePoses`, `getScoredCount`). |
| `TrajectoryArcPublisher` | Publishes `/YACSim/Ballistics/PredictedArc` and `ActiveProjectiles` (`Pose3d[]`) to NT4 and optional WPILib `DataLog`. |
