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

### Importing an Existing AdvantageScope `Robot_<Name>` Directory

If your team already maintains an AdvantageScope `Robot_<Name>/config.json`, import the chassis, all `model_0..N.glb` components, ordered zeroed rotations, positions, and cameras in one call:

```java
CadRobotConfig config =
    CadRobotConfig.fromAdvantageScopeDir(darwinDir)
        .withDrivetrainPoseSupplier(drive::getPose)
        .withExtensionMargin(0.35, 1.85)
        .ignoreRestingOverlaps()
        .withComponentPoseSupplier(0, "Hood", hood::getPose3d)
        .withComponentPoseSupplier(1, "Slamtake", slamtakePose);
```

### CAD-Tool Coordinate-System Presets

Avoid guessing axis rotation sequences for common CAD exporters by applying a `CadCoordinateSystem` preset:

```java
CadAssetConfig chassis =
    new CadAssetConfig(models.resolve("chassis.glb"))
        .withCoordinateSystem(CadCoordinateSystem.ONSHAPE_GLTF); // or SOLIDWORKS_MM, FUSION_360_MM, BLENDER_GLTF
```

### Camera Frustums & Starter AdvantageScope Layout

Register vision cameras so they appear in AdvantageScope's 3D field view and export a pre-wired layout JSON:

```java
config.addCamera("FrontCam", robotToCam, 70.0, 1280, 800);
AdvantageScopeConfigGenerator.exportStarterLayout(config, Path.of("yacsim-layout.json"));
```

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

### Multi-Stage Cascading Elevators & 4-Bar Linkages

```java
// 3-stage cascading elevator (1/3, 2/3, 3/3 travel)
List<Supplier<Pose3d>> stages =
    YamsCommands3Bridge.fromCascadingElevator(
        elevator, 3, new Translation3d(0.0, 0.0, 0.1), MotionAxis.Z);

// Level-riding 4-bar end-effector
Supplier<Pose3d> wristPose =
    YamsCommands3Bridge.fromFourBar(
        arm, new Translation3d(0.2, 0.0, 0.4), 0.55, MotionAxis.Y, true);
```

### Spinning Wheels, Intake Bars, Rollers & Flywheels

YACSim supports both CAD workflows used by FRC teams:

1. **Separate wheel/roller `.glb` meshes** (animating 3D spin in AdvantageScope):
   ```java
   // Standalone shooter flywheel from YAMS FlyWheel or SmartMotorController
   Supplier<Pose3d> flywheelPose =
       YamsCommands3Bridge.fromYamsRollerMotor(
           flywheelSmc, new Translation3d(-0.15, 0.0, 0.48), MotionAxis.Y);

   // Intake roller bar mounted on a deploying intake arm
   Supplier<Pose3d> intakeBarPose =
       YamsCommands3Bridge.fromMountedYamsRoller(
           slamtakePose, intakeRollerSmc, new Translation3d(0.28, 0.0, 0.0), MotionAxis.Y, false);

   // Or integrate directly from an AngularVelocity supplier
   Supplier<Pose3d> integratedWheel =
       YamsMechanismBridge.fromAngularVelocity(
           flywheel::getVelocity, new Translation3d(-0.15, 0.0, 0.48), MotionAxis.Y);
   ```
2. **Baked-in rollers (no separate wheel `.glb`, like FRC 6328 *Darwin*)**:
   Register each roller/flywheel with `CadRobotConfig.addRollerTelemetry(...)` to publish live `VelocityRPM`, `SurfaceSpeedMPS`, and `Spinning` under `/YACSim/Rollers/<name>/`:
   ```java
   robotConfig
       .addRollerTelemetry("Flywheel", flywheel::getVelocity, Units.inchesToMeters(2.0))
       .addRollerTelemetry("IntakeRoller", slamtake::getRollerVelocity, Units.inchesToMeters(1.0));
   ```

### Direct YAMS `SwerveDrive` Adapter

```java
CadDrivetrainConfig drivetrain =
    YamsCommands3Bridge.fromYamsSwerve(
        swerveDrive, chassisAsset, forkAsset, wheelAsset, 0.0508, moduleCorners);
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

See `examples/commands3/darwin_6328_2027/` for an independent community port referencing FRC Team 6328 Mechanical Advantage's publicly released MIT-licensed 2026 robot (*Darwin*) CAD assets using YAMS 2027 + YACSim (not affiliated with or tested by Team 6328):
- Downloads 6328's MIT-licensed `.glb` models (`model.glb` through `model_3.glb`) via `setup_darwin_cad.ps1`.
- Models the `Hood`, `Slamtake`, coupled pin-in-slot `FrontHopper`, and `TrashCompactor`.
- Verified live in AdvantageScope under a strict `-Xmx256m` robot JVM cap (`-XX:MaxHeapSize=268435456`).

---

## Automated CAD Manifests (`yacsim-robot.json`) & URDF Import

Want CAD to feed your mesh paths, AdvantageScope zeroing, exact mass/inertia tensors, joint origins, motion axes, hard/soft limits, and roller radii automatically into both YACSim and YAMS 2027 while keeping only CAN IDs, motor types, gear ratios, and PID/FF gains in Java?

- Read the full [CAD Manifest & URDF Automation Guide](CAD_MANIFEST_GUIDE.md).
- Compare the manual [`darwin_6328_2027`](../examples/commands3/darwin_6328_2027/) example against the manifest-driven [`darwin_6328_manifest_2027`](../examples/commands3/darwin_6328_manifest_2027/README.md) example.

- **CAD-to-Code Naming Conventions**: See [CAD_CODE_NAMING_CONVENTIONS.md](CAD_CODE_NAMING_CONVENTIONS.md) for aligning Onshape subassemblies and mates with your Java mechanisms.
- **Step-by-Step Implementation Walkthrough**: See [IMPLEMENTATION_WALKTHROUGH.md](IMPLEMENTATION_WALKTHROUGH.md) for a complete visual guide from blank project to AdvantageScope.

## 6. RK4 3D Ballistics, Dual-Pose Truth, WPILib DataLog & Field Exporter (`v2027.6.5-alpha1`)

### Dual-Pose Ground Truth (`SimTruthPose3d`) & Direct `.wpilog` Recording

Publish ground-truth physics pose alongside noisy fused odometry and record directly to WPILib's `.wpilog` DataLog (even when NetworkTables 4 is disabled for competition):

```java
CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(urdfZip, CadUrdfOptions.recommended())
    .withDrivetrainPoseSupplier(swerve::getPose)
    .withSimTruthPose2dSupplier(swerve::getSimPose) // publishes /YACSim/SimTruthPose3d
    .withDataLogEnabled(true);                      // records Pose3d & Pose3d[] to DataLogManager
```

Or wire animated swerve modules and ground-truth pose in one call via `YamsCommands3Bridge.attachSwerveModulesWithTruth(cfg, swerve, ...)`.

### Pure-Java RK4 3D Projectile Physics & Aim Arcs

Load 3D field obstacles and scoring targets from `yacsim-field.json` and step `ProjectilePhysicsSim` at 50 Hz alongside `TrajectoryArcPublisher`:

```java
List<FieldObstacle3d> obstacles = FieldObstacle3d.loadFieldManifest(deployDir.resolve("yacsim-field.json"));
ProjectilePhysicsSim ballistics = new ProjectilePhysicsSim()
    .withMaxActiveProjectiles(128)
    .addObstacles(obstacles);
TrajectoryArcPublisher arcPub = new TrajectoryArcPublisher("YACSim", true);

// Launch a 2026 REBUILT Fuel sphere with backspin (include robot chassis velocity for SOTM)
ballistics.launch(ProjectileSpec.frcFuel2026(), muzzleOrigin, fieldLaunchVel, backspinRadPerSec);
ballistics.step(0.02);
arcPub.publishPredictedArc(
    ballistics.predictArc(ProjectileSpec.frcFuel2026(), muzzleOrigin, fieldLaunchVel, backspinRadPerSec, 0.04, 40));
arcPub.publishActiveProjectiles(ballistics);
```

### Exporting Synced Field Polygons & 3D OBBs

Use `tools/onshape-field-exporter/field_exporter.py` to generate YAECE `SeasonConfig` JSON (Andymark & Welded), PathPlanner `navgrid.json`, and YACSim `yacsim-field.json` from a single field geometry source of truth. See [`tools/onshape-field-exporter/README.md`](../tools/onshape-field-exporter/README.md).

### Pairing YACSim With YALL (`VisionSim`) and YAECE (`v2027.6.5-alpha1`)

- Use **YALL** (`yall.sim.VisionSim` + `CameraModel`) for simulated AprilTag and object-detection cameras (PhotonVision, Limelight, Synapse).
- Use **YACSim** (`yacsim.core.sensors`, `yacsim.core.ballistics`, `yacsim.core.physics`) for 3D CAD/URDF kinematics, OBB collisions, `QuestNavSim`, `Lidar3dSim`, `FrictionCircleSlipModel`, and high-fidelity RK4 projectile physics.
- Use **YAECE** (`yaece.bridges.YallTargetBridge`, `YacSim3dBridge`) for autonomous tactical arbitration and world modeling.
