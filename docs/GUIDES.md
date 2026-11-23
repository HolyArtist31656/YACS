# Guides

Practical how-to guides for configuring CAD assets, AdvantageScope exports, YAMS bridges, and OBB collision detection in YACSim.

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

## 2. AdvantageScope Export & Ordered Multi-Axis Rotations

`AdvantageScopeConfigGenerator.exportAssetBundle(config, outputDir)` writes a complete `Robot_<Name>/` directory containing `config.json`, `model.glb`, and `model_0.glb` … `model_N.glb`.

When CAD exports need sequential axis rotations to align with WPILib's NWU coordinate system (matching AdvantageScope's `rotations` array), chain `withZeroedRotationAxis(axis, degrees)` in exact application order:

```java
CadAssetConfig chassis =
    new CadAssetConfig(models.resolve("chassis.glb"))
        .withZeroedRotationAxis("x", 90.0)
        .withZeroedRotationAxis("z", 180.0)
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

### Articulated Mechanisms

Use `YamsMechanismBridge` to connect any YAMS mechanism (`Arm`, `Elevator`, `Pivot`) or parent–child chain to a `CadComponentConfig`:

```java
// Single mechanism with mounting offset
Supplier<Pose3d> hoodPose =
    YamsMechanismBridge.fromPoseSupplier(
        hood::getPose3d,
        new Transform3d(new Translation3d(-0.151, 0.0, 0.486), new Rotation3d(0, 0, Math.PI)));

// Child mechanism mounted on a moving parent (e.g., Wrist on Elevator)
Supplier<Pose3d> wristPose =
    YamsMechanismBridge.chainParentChild(
        elevator::getPose3d,
        wrist::getLocalPose3d);
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

## 4. OBB Collision Detection, Envelopes & `ignoreCollisionPair`

Every call to `simIterate()` evaluates oriented bounding box (OBB) collisions using the Separating Axis Theorem (`ObbCollisionDetector`):
- **Component ↔ Chassis**: Checked unless the component's parent is `"Chassis"` or chassis collision is disabled.
- **Component ↔ Component**: Checked for all enabled pairs except direct parent–child pairs.
- **Component ↔ `ExtensionEnvelope`**: Checked when `.withExtensionEnvelope(BoundingBox3d)` is configured.

### Silencing Coupled Linkages

On compact robots, coupled sibling mechanisms (such as an intake and a linked hopper) can overlap as bounding boxes even when their concave CAD meshes clear. Use symmetric `ignoreCollisionPair(a, b)` on `CadRobotConfig`:

```java
robotConfig
    .ignoreCollisionPair("Slamtake", "FrontHopper")
    .ignoreCollisionPair("Intake", "ExtensionEnvelope");
```

Argument order does not matter (`ignoreCollisionPair("A", "B")` equals `ignoreCollisionPair("B", "A")`), and `"Chassis"` and `"ExtensionEnvelope"` are supported as target names.

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
