# CAD Manifest & URDF Automation Guide (`yacsim-robot.json`)

YACSim `v2027.1.0-alpha1` introduces **Phase 1 CAD-to-YACSim Automation** (`yacsim.core.manifest`), giving FRC teams a choice between three fully interoperable workflows:

1. **Manual Java Configuration** — Hand-configure `CadRobotConfig`, `CadAssetConfig`, pivots, limits, and `YamsCommands3Bridge` (all existing APIs remain 100% unchanged).
2. **Manifest-Driven (`yacsim-robot.json`)** — Feed all CAD-extractable geometry, exact mass/inertia tensors, joint pivots, axes, hard/soft limits, arm lengths, drum/wheel radii, and cameras from a single `yacsim-robot.json` file into both **YACSim** and **YAMS 2027**. Keep only electrical & control values (CAN IDs, `DCMotor`, gear ratios, PID/FF gains) in Java.
3. **URDF XML Import (`.urdf`) & Hybrid** — Import standard URDF files exported by [`onshape-to-robot`](https://pypi.org/project/onshape-to-robot/0.1.1/), Fusion 360 URDF exporters, or SolidWorks `sw_urdf_exporter` directly via `CadRobotConfig.fromUrdf(Path)`, or convert a URDF to `yacsim-robot.json` via `CadManifestLoader.writeJson(...)` and override any individual setting in Java.

> **Prerequisites for Automated CAD Fetching**: Running a `yacsim-robot.json` manifest inside your WPILib robot project requires **Java 25 only** (zero Python on the robot). If you want to **automatically fetch and generate** `yacsim-robot.json` + `.glb` meshes from Onshape using `tools/onshape-exporter/`, you will need **Python 3.10+** and an **Onshape API key pair** on your development laptop—see the step-by-step [Onshape Exporter & API Key Setup Guide](ONSHAPE_EXPORTER_GUIDE.md).

---

## 1. What Comes From CAD vs. What Stays in Java

| Category | Auto-Fed From `yacsim-robot.json` / URDF | Configured in Java (YAMS / WPILib) |
| :--- | :--- | :--- |
| **Visuals & Zeroing** | Mesh filenames (`.glb`/`.obj`/`.stl`), unit scale, AdvantageScope `zeroedPosition` & ordered `zeroedRotations` | Optional AdvantageScope export path / auto-install flag |
| **Mass & Inertia** | Exact CAD `massKilograms`, `centerOfMassMeters`, `ixxKgM2` / `iyyKgM2` / `izzKgM2`, and per-joint `momentOfInertiaKgM2` | — (overrides bounding-box approximation automatically) |
| **Kinematics & Limits** | Joint `type` (`revolute`, `prismatic`, `continuous`, `fixed`), `originMeters`, `originRpyRadians`, `axis`, `inverted`, `minLimit`, `maxLimit`, `softMinLimit`, `softMaxLimit`, `startingPosition`, `lengthMeters` | Non-standard custom linkages (e.g., pin-in-slot equations) via `withComponentPoseSupplier("LinkName", ...)` |
| **Rollers, Flywheels & Drums** | `wheelRadiusMeters` (feeds `FlyWheelConfig.withDiameter`, `SmartMotorControllerConfig.withDrumRadius`, and YACSim roller telemetry) | Target RPM / surface speed / open-loop voltages |
| **Collision & Cameras** | `extensionXyMarginMeters`, `extensionMaxHeightMeters`, `ignoreRestingOverlaps`, `autoGroundChassis`, `cameras[]` | Optional extra `ignoreCollisionPair(...)` rules |
| **Electrical & Controls** | — (never exists in CAD) | Motor controller CAN IDs, CAN bus, `DCMotor` type & count, gearbox reduction stages, stator current limits, `MotorMode`, PID & Feedforward gains |

---

## 2. Complete `yacsim-robot.json` Schema (`schemaVersion: "1.0"`)

All linear dimensions are in **meters**, all joint angles/limits/RPY orientations are in **radians**, all AdvantageScope `zeroedRotations` are in **degrees**, all masses are in **kilograms**, and all moments of inertia are in **kg·m²**.

```json
{
  "schemaVersion": "1.0",
  "robotName": "MyRobot",
  "sourceUrl": "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>",
  "extensionXyMarginMeters": 0.35,
  "extensionMaxHeightMeters": 1.85,
  "ignoreRestingOverlaps": true,
  "autoGroundChassis": false,
  "disableAdvantageScopeSimplification": false,
  "chassis": {
    "name": "Chassis",
    "meshFile": "model.glb",
    "unitScaleToMeters": 1.0,
    "zeroedPosition": [0.0, 0.0, 0.0],
    "zeroedRotations": [
      { "axis": "x", "degrees": 90.0 },
      { "axis": "z", "degrees": 90.0 }
    ],
    "massProperties": {
      "volumeCubicMeters": 0.022,
      "massKilograms": 52.0,
      "centerOfMassMeters": [0.0, 0.0, 0.22],
      "ixxKgM2": 3.5,
      "iyyKgM2": 4.2,
      "izzKgM2": 5.8
    },
    "collisionEnabled": true
  },
  "links": [
    {
      "name": "Hood",
      "meshFile": "model_0.glb",
      "zeroedPosition": [0.0, 0.0, 0.0],
      "zeroedRotations": [
        { "axis": "x", "degrees": 90.0 },
        { "axis": "z", "degrees": 90.0 }
      ],
      "massProperties": {
        "massKilograms": 1.36,
        "centerOfMassMeters": [0.0, 0.0, 0.09],
        "ixxKgM2": 0.014,
        "iyyKgM2": 0.0147,
        "izzKgM2": 0.012
      },
      "collisionEnabled": true
    }
  ],
  "joints": [
    {
      "name": "Hood",
      "type": "revolute",
      "parentLink": "Chassis",
      "childLink": "Hood",
      "originMeters": [-0.273, 0.0, 0.486],
      "originRpyRadians": [0.0, 0.0, 0.0],
      "axis": "Y",
      "inverted": false,
      "minLimit": 0.15707963,
      "maxLimit": 0.73303829,
      "softMinLimit": 0.17453293,
      "softMaxLimit": 0.71907565,
      "startingPosition": 0.17453293,
      "lengthMeters": 0.18
    },
    {
      "name": "Flywheel",
      "type": "continuous",
      "parentLink": "Chassis",
      "childLink": "FlywheelVirtual",
      "originMeters": [-0.151, 0.0, 0.486],
      "axis": "Y",
      "wheelRadiusMeters": 0.0508,
      "momentOfInertiaKgM2": 0.02
    }
  ],
  "cameras": [
    {
      "name": "FrontCamera",
      "position": [0.28, 0.0, 0.52],
      "rpyRadians": [0.0, -0.261799, 0.0],
      "fovDegrees": 70.0,
      "resolution": [1280, 800]
    }
  ]
}
```

### Key Schema Details
- **Relative Mesh Paths**: `meshFile` is always resolved relative to the directory containing `yacsim-robot.json` (or `.urdf`).
- **Virtual / Telemetry-Only Joints**: For spinning rollers or flywheels that do not have a dedicated `.glb` file (like Darwin's `Flywheel` or `KickerFront`), point `childLink` to a virtual name (e.g., `"FlywheelVirtual"`). `YamsCommands3Bridge.bindFromManifest(config, "Flywheel", smc)` automatically registers `/YACSim/Rollers/Flywheel/{VelocityRPM,SurfaceSpeedMPS,Spinning}` using `wheelRadiusMeters` and skips 3D pose binding until you add a mesh.
- **Exact CAD Mass Override**: Whenever a link defines `massProperties`, `CadLinkManifest.toCadAssetConfig(baseDir)` attaches `CadAssetConfig.withMassPropertiesOverride(...)`, so `MeshParser` uses the exact CAD mass, center of mass, and inertia tensor instead of a bounding-box approximation.

---

## 3. Feeding CAD Values Into YAMS 2027 & YACSim

Load the manifest once at startup and pass it to both your YAMS mechanisms and `CadRobotConfig.fromManifest(...)`:

```java
Path manifestDir = Filesystem.getDeployDirectory().toPath().resolve("yacsim/darwin");
CadRobotManifest manifest = CadManifestLoader.loadJson(manifestDir.resolve("yacsim-robot.json"));

// 1. Inside a YAMS Arm mechanism (zero geometry hardcoded in Java!)
CadJointManifest hoodJoint = manifest.getJoint("Hood");
SmartMotorControllerConfig motorConfig =
    new SmartMotorControllerConfig(this)
        .withMomentOfInertia(manifest.getJointMomentOfInertia("Hood"))
        .withStartingPosition(hoodJoint.startingAngle())
        .withSoftLimits(hoodJoint.softMinLimitAngle(), hoodJoint.softMaxLimitAngle())
        /* + CAN ID, DCMotor, gearing, PID/FF */;

ArmConfig armConfig =
    new ArmConfig()
        .withHardLimits(hoodJoint.minLimitAngle(), hoodJoint.maxLimitAngle())
        .withLength(hoodJoint.length());

// 2. Inside a YAMS Elevator mechanism
CadJointManifest compactorJoint = manifest.getJoint("TrashCompactor");
ElevatorConfig elevatorConfig =
    new ElevatorConfig()
        .withHardLimits(compactorJoint.minLimitDistance(), compactorJoint.maxLimitDistance())
        .withCarriageWeight(manifest.getLinkMass("TrashCompactor"));

// 3. Inside a YAMS FlyWheel / Roller mechanism
CadJointManifest fwJoint = manifest.getJoint("Flywheel");
FlyWheelConfig fwConfig =
    new FlyWheelConfig().withDiameter(fwJoint.wheelDiameter());

// 4. Bind every mechanism to YACSim in one line each!
CadRobotConfig robotConfig = CadRobotConfig.fromManifest(manifest, manifestDir)
    .withDrivetrainPoseSupplier(fieldPoseSupplier);

YamsCommands3Bridge.bindFromManifest(robotConfig, "Hood", hood.getArm());
YamsCommands3Bridge.bindFromManifest(robotConfig, "TrashCompactor", compactor.getElevator());
YamsCommands3Bridge.bindFromManifest(robotConfig, "Flywheel", flywheel.getMotor());
```

---

## 4. Direct URDF XML Import (`CadRobotConfig.fromUrdf`)

If your CAD team exports a standard `.urdf` file from Onshape (`onshape-to-robot`), Fusion 360, or SolidWorks (`sw_urdf_exporter`), YACSim imports it with **zero third-party XML libraries** (using the JDK's XXE-hardened DOM parser):

```java
CadRobotConfig config =
    CadRobotConfig.fromUrdf(deployDir.resolve("urdf/robot.urdf"))
        .withDrivetrainPoseSupplier(drive::getPose);

YamsCommands3Bridge.bindFromManifest(config, "arm_joint", arm.getArm());
```

### Converting URDF → `yacsim-robot.json`
To convert an exported URDF into an editable `yacsim-robot.json` (so you can add cameras, roller radii, or extension margins):

```java
CadRobotManifest manifest = CadManifestLoader.loadUrdf(Path.of("robot.urdf"));
CadManifestLoader.writeJson(manifest, Path.of("yacsim-robot.json"));
```

---

## 5. Complete Reference Example: FRC 6328 *Darwin* (Manifest-Driven)

See [`examples/commands3/darwin_6328_manifest_2027/`](../examples/commands3/darwin_6328_manifest_2027/README.md) for a full working 2027 Commands v3 robot powered by `src/main/deploy/yacsim/darwin/yacsim-robot.json` referencing Darwin's Onshape assembly ([Onshape](https://cad.onshape.com/documents/2da573df9d139d7a1e0f08a8/w/5266e52966df9f56fcb472f0/e/15a6a42133f059b818ad5264)).

---

## 6. Recommended CAD-to-Code Naming Conventions

See the [CAD-to-Code Naming Conventions Guide](CAD_CODE_NAMING_CONVENTIONS.md) for how to align Onshape subassembly/mate names with your Java mechanisms (plus YACSim's forgiving case/separator/`yacsim_`-prefix name matching).
