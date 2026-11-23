# Java API Reference

YACSim ships two Maven artifacts under group ID `yacsim`:
- `yacsim:YACSim-core` — Pure-Java CAD mesh parsing, OBB collision detection, drivetrain/mechanism bridges, and AdvantageScope export/telemetry.
- `yacsim:YACSim-commands3` — WPILib Commands v3 `CadRobotMechanism` wrapper and `CadCollisionTriggers`.

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
| `void close()` | Closes NetworkTables publishers. |

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
| `withDrivetrain(CadDrivetrainConfig cfg)` | Registers chassis asset, field pose supplier, and all swerve/wheel components. |
| `withChassisAsset(CadAssetConfig asset)` | Sets chassis CAD asset directly (when not using `CadDrivetrainConfig`). |
| `addComponent(CadComponentConfig comp)` | Adds an articulated mechanism component in AdvantageScope `model_N` order. |
| `withDrivetrainPoseSupplier(Supplier<Pose2d> s)` | Default: `Pose2d::new`. |
| `withTelemetryTable(String table)` | Default: `"YACSim"`. |
| `withAdvantageScopeExportDir(Path dir)` | Optional; exports `Robot_<robotName>` bundle on simulator startup. |
| `withExtensionEnvelope(BoundingBox3d box)` | Optional robot-frame legal extension volume. |
| `withCollisionChecking(boolean enabled)` | Default: `true`. |
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
| `CadRobotSimulator(CadRobotConfig config)` | Parses meshes, optionally exports AdvantageScope bundle, and opens NT4 publishers. |
| `CadRobotSimulator(CadRobotConfig config, boolean enableNetworkTables)` | Headless constructor (`enableNetworkTables = false`) for JUnit tests. |
| `void simIterate()` | Samples poses, runs OBB collision checks, and atomically publishes a new `SimSnapshot`. |
| `void updateTelemetry()` | Writes the latest `SimSnapshot` to NetworkTables. |
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
| `withZeroedRotation(Rotation3d rot)` / `withZeroedRotation(double rollDeg, double pitchDeg, double yawDeg)` | Sets X/Y/Z zeroed rotations. |
| `withMaxTriangleBudget(int maxTriangles)` | Raises/lowers triangle validation threshold. |

### `MeshParser`, `MeshMetadata`, `MeshFormat`, `CadMeshValidationException`
- `MeshParser.parse(CadAssetConfig)` returns `MeshMetadata(format, triangleCount, localBounds, massProperties)`.
- `MeshFormat`: `GLB`, `GLTF`, `OBJ`, `STL` (`MeshFormat.fromPath(Path)`).
- `CadMeshValidationException`: Thrown on missing files, unsupported formats, corrupt headers, or `.obj`/`.stl` files exceeding `maxTriangleBudget`.

---

## `yacsim.core.drivetrain` & `yacsim.core.yams`

- **`CadDrivetrainConfig`**: Configures chassis asset, field pose supplier, optional 3D gyro rotation supplier (pitch/roll), swerve modules (`addSwerveModule`), differential wheels (`addDifferentialWheel`), and chassis collision toggle.
- **`CadSwerveModuleConfig`**: Configures a swerve module's mounting `Translation3d`, wheel radius, steer fork asset, wheel asset, and `Supplier<SwerveModulePosition>`.
- **`YamsDrivetrainBridge.fromYamsSwerve(...)`**: One-call factory wiring a 4-module YAMS swerve drivetrain into `CadDrivetrainConfig`.
- **`YamsMechanismBridge`**: `fromPoseSupplier(Supplier<Pose3d>, Transform3d)` and `chainParentChild(Supplier<Pose3d>, Supplier<Pose3d>)`.

---

## `yacsim.core.physics`

- **`BoundingBox3d`**: Immutable axis-aligned bounds record (`minX..maxZ`) with `center()`, `halfX/Y/Z()`, `sizeX/Y/Z()`, `scaled(double)`, `expandedForExtension(xyMargin, maxHeight)`, and `transformedCorners(Pose3d)`.
- **`CadMassProperties`**: Immutable mass, center of mass, and principal inertia (`ixx`, `iyy`, `izz`) record with `withTotalMassKg(double)` and `fromBoundingBox(box, density)`.
- **`CollisionEvent`**: Immutable `(componentA, componentB, penetrationDepthMeters)` record with `involves(String)`.
- **`ObbCollisionDetector`**: Separating Axis Theorem (SAT) `testObbCollision(boxA, poseA, boxB, poseB)` and `testEnvelopeViolation(box, pose, envelope)`.

---

## `yacsim.core.advantagescope` & `yacsim.core.util`

- **`AdvantageScopeConfigGenerator`**: `buildConfigJson(CadRobotConfig)` and `exportAssetBundle(CadRobotConfig, Path)`.
- **`AdvantageScopePublisher`**: Publishes `RobotPose`, `RobotPose3d`, `ComponentPoses`, `CollisionActive`, and `CollisionDetails` under the configured NetworkTables root table (default `/YACSim`).
- **`MinimalJson`**: Zero-dependency recursive-descent JSON parser (`parse(String)`) and pretty-printer (`toPrettyJson(Object)`).
