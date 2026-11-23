# Guides

Practical how-to guides for configuring CAD assets, AdvantageScope exports, YAMS adapters, and auto-calibrated OBB collision detection in YACSim.

## 1. Preparing CAD Meshes

YACSim supports four mesh formats via `MeshParser`:

| Format | Extensions | Notes |
| --- | --- | --- |
| Binary glTF 2.0 | `.glb` | **Recommended** for AdvantageScope. Parsed from header/JSON + accessor bounds without loading GPU buffers. |
| JSON glTF 2.0 | `.gltf` | Parsed via `MinimalJson` using accessor `min`/`max` bounds. |
| Wavefront OBJ | `.obj` | Parsed vertex-by-vertex; set `.withMillimeterUnits()` or `.withInchUnits()` if not exported in meters. |
| ASCII / Binary STL | `.stl` | Auto-detects binary vs. ASCII STL and computes triangle count, bounds, and inertia. |

### Triangle Budgets & High-Poly `.glb` Models

By default, `CadAssetConfig` enforces a `150,000` triangle budget (`DEFAULT_MAX_TRIANGLES`).
- For `.obj` and `.stl` files, exceeding the budget throws `CadMeshValidationException` to protect SystemCore RAM during CPU-side vertex parsing.
- For `.glb` and `.gltf` files, `MeshParser` reads only metadata/accessor bounds (zero GPU/vertex buffer allocation), so high-poly CAD exports (such as FRC 6328's 2.1M-triangle chassis) log a warning to `System.err` instead of crashing. You can still raise the threshold explicitly with `.withMaxTriangleBudget(5_000_000)`.

### Multi-Primitive Sub-OBBs for Concave `.glb` Mechanisms

When parsing `.glb` or `.gltf` assets containing up to `64` primitives (`MeshParser.MAX_SUB_OBBS`), YACSim extracts individual per-primitive bounding boxes (`MeshMetadata.collisionSubBounds()`). `CadRobotSimulator` tests collisions against these sub-OBBs via `ObbCollisionDetector.testMultiObbCollision(...)`, eliminating false-positive overlaps inside the open cavity of L-shaped or U-shaped mechanisms (such as intakes or hoppers). Massive assemblies with more than 64 primitives automatically fall back to a single outer bounding box to preserve 50 Hz SystemCore performance.

## 2. AdvantageScope Auto-Install, Auto-Grounding & Multi-Axis Rotations

### One-Line Auto-Install & Auto-Grounding

Enable `.withAutoInstallToAdvantageScope()` and `.autoGroundChassis()` on `CadRobotConfig`:

```java
CadRobotConfig config =
    new CadRobotConfig("MyRobot")
        .withChassisAsset(new CadAssetConfig(models.resolve("chassis.glb")))
        .autoGroundChassis()               // Shifts chassis minZ to z = 0.0 in config.json
        .withAutoInstallToAdvantageScope(); // Writes Robot_MyRobot into OS AdvantageScope userAssets
```

`AdvantageScopeConfigGenerator.resolveDefaultUserAssetsDir()` resolves `%APPDATA%\AdvantageScope\userAssets` on Windows, `~/Library/Application Support/AdvantageScope/userAssets` on macOS, and `$XDG_CONFIG_HOME/AdvantageScope/userAssets` (or `~/.config/AdvantageScope/userAssets`) on Linux.

### Ordered Multi-Axis Rotations

`AdvantageScopeConfigGenerator.exportAssetBundle(config, outputDir)` writes a complete `Robot_<Name>/` directory containing `config.json`, `model.glb`, and `model_0.glb` … `model_N.glb`.

When CAD exports need sequential axis rotations to align with WPILib's NWU coordinate system (matching AdvantageScope's `rotations` array), chain `addZeroedRotation(axis, degrees)` in exact application order:

```java
CadAssetConfig chassis =
    new CadAssetConfig(models.resolve("chassis.glb"))
        .addZeroedRotation("x", 90.0)
        .addZeroedRotation("z", 180.0)
        .withZeroedPosition(new Translation3d(0.0, 0.0, 0.05));
```

This emits:

```json
"rotations": [
  { "axis": "x", "degrees": 90.0 },
  { "axis": "z", "degrees": 180.0 }
]
```

## 3. Bridging YAMS Mechanisms & Swerve Drivetrains

### Direct YAMS Commands v3 Adapters (`YamsCommands3Bridge`)

Instead of writing manual `Pose3d` / `Rotation3d` math per mechanism, pass your YAMS 2027 `Arm`, `Elevator`, or `Pivot` directly to `YamsCommands3Bridge`:

```java
// Pitching arm pivoting at (0.20, 0.0, 0.45) around Y
Supplier<Pose3d> armPose =
    YamsCommands3Bridge.fromYamsArm(arm, new Translation3d(0.20, 0.0, 0.45), MotionAxis.Y);

// Vertical elevator starting at (0.01, 0.0, 0.0) moving along Z
Supplier<Pose3d> elevatorPose =
    YamsCommands3Bridge.fromYamsElevator(
        elevator, new Translation3d(0.01, 0.0, 0.0), MotionAxis.Z);

// Turret / pivot rotating around Z (inverted if CAD rotation opposes right-hand rule)
Supplier<Pose3d> turretPose =
    YamsCommands3Bridge.fromYamsPivot(
        turret, new Translation3d(-0.15, 0.0, 0.48), MotionAxis.Z, true);
```

### Generic Suppliers & Parent–Child Chains (`YamsMechanismBridge`)

If you use raw `Supplier<Angle>`, `Supplier<Distance>`, or nested mechanisms (such as a wrist on an elevator carriage), use `YamsMechanismBridge`:

```java
// Angle supplier without coupling to YAMS Commands v3
Supplier<Pose3d> hoodPose =
    YamsMechanismBridge.fromAngleSupplier(
        hood::getAngle, new Translation3d(-0.151, 0.0, 0.486), MotionAxis.Y, true);

// Child mechanism mounted on a moving parent
Supplier<Pose3d> wristPose =
    YamsMechanismBridge.chainParentChild(elevatorPose, wristLocalPose);
```

### Seeding YAMS Mass & Moment of Inertia from CAD

Avoid duplicating CAD mass/MOI values into YAMS configs—compute WPILib `Mass` and parallel-axis `MomentOfInertia` directly from parsed `MeshMetadata`:

```java
MeshMetadata armMeta = MeshParser.parse(new CadAssetConfig(models.resolve("arm.glb")).withMassOverrideKg(6.0));
MomentOfInertia armMoi =
    YamsMechanismBridge.estimateArmMoi(armMeta, new Translation3d(0.0, 0.0, 0.0), MotionAxis.Y);
Mass carriageMass = YamsMechanismBridge.estimateMass(elevatorMeta);
```

### Swerve Drivetrains

Use `YamsDrivetrainBridge.fromYamsSwerve(...)` to bind chassis 3D pose (including gyro pitch/roll) and all four swerve modules (steer forks + spinning wheels) in one call:

```java
CadDrivetrainConfig drivetrain =
    YamsDrivetrainBridge.fromYamsSwerve(
        new CadAssetConfig(models.resolve("chassis.glb")),
        new CadAssetConfig(models.resolve("swerve_fork.glb")),
        new CadAssetConfig(models.resolve("swerve_wheel.glb")),
        Units.inchesToMeters(2.0),
        moduleCorners,
        swerve::getPose,
        gyro::getRotation3d,
        swerve::getModulePositions);

CadRobotConfig robot = new CadRobotConfig("SwerveBot").withDrivetrain(drivetrain);
```

## 4. OBB Collision Detection, Auto-Derived Envelopes & Resting Overlap Calibration

Every call to `simIterate()` evaluates oriented bounding box (OBB) collisions using the Separating Axis Theorem (`ObbCollisionDetector`):
- **Component ↔ Chassis**: Checked unless the component's parent is `"Chassis"` or chassis collision is disabled.
- **Component ↔ Component**: Checked for all enabled pairs except direct parent–child pairs.
- **Component ↔ `ExtensionEnvelope`**: Checked when either `.withExtensionMargin(xyMeters, maxHeightMeters)` or `.withExtensionEnvelope(BoundingBox3d)` is configured.

### Auto-Deriving the Legal FRC Extension Envelope

Instead of manually measuring bumpers and computing a `BoundingBox3d`, call `.withExtensionMargin(xyMarginMeters, maxHeightMeters)`:

```java
robotConfig.withExtensionMargin(0.35, 1.85);
```

YACSim automatically expands the parsed chassis CAD bounding box horizontally by `0.35 m` on all four sides and caps vertical height to `[0.0, 1.85] m`. If an explicit `.withExtensionEnvelope(BoundingBox3d)` is also provided, the explicit box takes precedence.

### Auto-Ignoring Resting Coupled-Linkage Overlaps

On compact robots, tightly packed sibling mechanisms (such as an intake and a linked hopper) often overlap as OBBs at rest even when their concave CAD meshes clear. Add `.ignoreRestingOverlaps()`:

```java
robotConfig.ignoreRestingOverlaps();
```

At startup (`t = 0`), `CadRobotSimulator` samples initial poses once, automatically ignores any component↔component or component↔chassis pairs already overlapping at rest, and logs each auto-ignored pair to stdout (`[YACSim] Auto-ignored resting OBB overlap: A <-> B`). **`ExtensionEnvelope` violations are never auto-ignored** so an illegal starting configuration is still caught.

You can also explicitly silence individual pairs (including `"Chassis"` or `"ExtensionEnvelope"`) with symmetric `ignoreCollisionPair(a, b)`:

```java
robotConfig
    .ignoreCollisionPair("Slamtake", "FrontHopper")
    .ignoreCollisionPair("Intake", "ExtensionEnvelope");
```

### Binding Commands v3 Triggers

Use `CadCollisionTriggers` to react to collisions inside Commands v3 bindings:

```java
CadCollisionTriggers.onCollisionBetween(cadSim.getSimulator(), "Arm", "Chassis")
    .onTrue(stopArmCommand);

CadCollisionTriggers.onExtensionViolation(cadSim.getSimulator())
    .onTrue(retractCommand);
```

## 5. Real-Robot Case Study: FRC 6328 "Darwin"

See `examples/commands3/darwin_6328_2027/` for a complete port of FRC Team 6328 Mechanical Advantage's 2026 robot (*Darwin*) to YAMS 2027 (`Arm` + `Elevator` + `TalonFXWrapper`) and YACSim:
- Downloads 6328's MIT-licensed `.glb` models (`model.glb` through `model_3.glb`) via `setup_darwin_cad.ps1`.
- Models the `Hood`, `Slamtake`, coupled pin-in-slot `FrontHopper`, and `TrashCompactor`.
- Verified live in AdvantageScope under a strict `-Xmx256m` robot JVM cap (`-XX:MaxHeapSize=268435456`).
