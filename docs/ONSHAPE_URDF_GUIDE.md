# Native Onshape URDF Export Guide (Zero-Python Workflow)

Starting in **YACSim `v2027.6.5-alpha1`** and expanded with code-side CAD repair and pass-through YAMS wiring in **`v2027.4.0-alpha1`**, you can drop a **native Onshape URDF `.zip` export** straight into `src/main/deploy/yacsim/` and load it in Java—with **zero Python installation, zero Onshape API keys, and zero external conversion scripts**.

> **Strictly Additive:** This workflow sits *alongside* every existing YACSim integration method. Your existing **Manual Java** (`CadRobotConfig`), **AdvantageScope Directory** (`fromAdvantageScopeDir`), **`yacsim-robot.json` Manifest** (`fromManifest`), and **Python Onshape REST API Exporter** (`tools/onshape-exporter/`) workflows remain 100% supported and unchanged.

---

## When to Use Native Onshape URDF vs. Other YACSim Methods

| Workflow | Best For | External Tools Required | Zero-Code Mesh & Physics Sync |
| :--- | :--- | :--- | :--- |
| **1. Native Onshape URDF `.zip` (`fromUrdfOrZip` / `YacSimYamsFactory`)** | Fastest zero-setup Onshape-to-YACSim path (no Python, no API keys) | **None** (Onshape built-in Export) | Meshes (`.glb`), joints, mate limits, mass & inertia, code-side CAD repair |
| **2. Python Onshape Exporter (`yacsim-robot.json`)** | Automated CI/CLI re-exports, Onshape subassembly grouping, custom `cameras[]`, `wheelRadiusMeters` | Python 3.10+ + Onshape API keys | Full `yacsim-robot.json` + AdvantageScope camera/roller config |
| **3. AdvantageScope Directory (`fromAdvantageScopeDir`)** | Teams that already maintain an AdvantageScope `Robot_<Name>/` folder | None | Preserves `config.json` transforms & cameras |
| **4. Manual Java (`new CadRobotConfig(...)`)** | Custom/procedural setups or non-Onshape CAD (`.stl`, `.obj`, `.glb`) | None | Full explicit Java control |

---

## Step 1: Export URDF + GLB from Onshape

Onshape's native Assembly URDF export (introduced in [Onshape's March 2026 release](https://forum.onshape.com/discussion/30410/improvements-to-onshape-march-13-2026) and expanded with mate limits in [Onshape's May 2026 update](https://www.onshape.com/en/resource-center/what-is-new/mate-connectors-explicit-lights-render-studio-advanced-urdf-mate-limit-support-robotics-applications)) exports links, joints, mate limits, inertia tensors, and binary `.glb` meshes in a single `.zip` file ([Onshape Help: Exporting Files](https://cad.onshape.com/help/Content/File/exporting_files.htm)).

1. **Suppress fasteners & hardware (recommended, though YACSim also filters them in code):** Because Onshape locks **`Include hidden instances in export`** ON (grayed out) for URDF exports, merely hiding parts will still include them in the URDF bundle. Suppressing bolts/washers/nuts in Onshape keeps the `.zip` smaller, and `CadUrdfOptions.recommended()` strips any remaining hardware automatically on the Java side.
2. **Use Revolute / Slider mates only on real moving joints:** Fasten static brackets/plates with **Fastened** or **Group** mates so Onshape doesn't emit dozens of unnecessary `planar_*` / `cylindrical_*` links. Name your moving mates clearly (e.g., `Turret`, `Hood`, `IntakeArm`, `Climber`), or remap default mate names (`revolute_1`) in Java with `CadUrdfOptions.withJointAlias(...)`.
3. **Assign materials (recommended for YAMS physics):** Assign materials to subassemblies/parts in Onshape so the exported `<inertial>` blocks contain real CAD mass (`<mass>`) and inertia tensors (`<inertia ixx="..." iyy="..." izz="..."/>`).
4. **Export the Assembly:**
   - Right-click your top-level **Assembly tab** at the bottom of Onshape and select **Export...**.
   - **File name:** Use a short, space-free name (for example `my_robot_urdf`).
   - **Format:** `URDF`
   - **Geometry format:** `GLB` *(recommended for AdvantageScope 3D Field; YACSim also reads `GLTF`, `STL`, and `OBJ`)*
   - **Resolution:** `Coarse` *(keeps `.glb` triangle counts lightweight for 50 Hz OBB collision & AdvantageScope rendering; bump back to `Medium` only if a part looks visibly faceted)*
   - **Options:** `Download`
   - **Check `Export rigid subassemblies as a single link`:** *(Important—unchecked by default!)* Collapses each rigid subassembly into one clean `<link>` and `.glb` mesh.
   - Click **Export** to download your `.zip` bundle (for example `my_robot_urdf.zip`).

---

## Step 2: Drop the `.zip` into Your Robot Project

Copy the downloaded `.zip` directly into your WPILib deploy folder (no manual unzipping required—YACSim also accepts an already-unzipped folder or standalone `.urdf` file):

```text
src/main/deploy/yacsim/
└── my_robot_urdf.zip
```

---

## Step 3: Load & Bind in Java (Two Ways)

### Option A: Zero-Overhead Real-Robot Pass-Through (`YacSimYamsFactory`, `v2027.4.0-alpha1`)

Pass your YAMS 2027 mechanism configs directly to `YacSimYamsFactory` and receive the standard YAMS mechanisms back. On a real robot (`RobotBase.isReal()`), `fromUrdfOrZip(...)` skips all disk I/O and GLB parsing, and `createArm` / `createPivot` / `createElevator` / `createFlyWheel` immediately return `new Arm(...)` / `new Pivot(...)` with zero simulation overhead:

```java
import java.nio.file.Path;
import java.util.List;
import org.wpilib.system.Filesystem;
import yacsim.commands3.YacSimYamsFactory;
import yacsim.core.manifest.CadUrdfOptions;
import yams.commands3.mechanisms.Arm;
import yams.commands3.mechanisms.Pivot;

Path urdfZip = Filesystem.getDeployDirectory().toPath().resolve("yacsim/my_robot_urdf.zip");

CadUrdfOptions urdfOpts =
    CadUrdfOptions.recommended()                      // strips fasteners + disables unbound OBBs
        .excludeMeshPatterns("MAXSwerve")             // strip static swerve blocks for animated overlay
        .withJointAlias("revolute_1", "Turret")       // map default Onshape mate -> Java name
        .withJointAlias("revolute_8", "Hood")
        .keepOnlyJoints(List.of("Turret", "Hood"))    // keep only real moving mechanisms
        .bakeUnboundLinksIntoChassis(true)            // bakes static plates into root & subtrees into parents
        .attachLinksToJoint("Turret", "launcher_side_plate", "kicker_back_plate"); // code-side re-parenting!

YacSimYamsFactory yacsim = YacSimYamsFactory.fromUrdfOrZip(urdfZip, urdfOpts);
Pivot turret = yacsim.createPivot("Turret", pivotConfig, turretMotor);
Arm hood = yacsim.createArm("Hood", armConfig, hoodMotor); // auto-chains onto rotating Turret!
yacsim.attachSwerve(swerveDrive, steerForkAsset, wheelAsset, 0.0381, moduleLocations);

yacsim.printDiagnosticReport();
yacsim.startSimulator();
```

Call `yacsim.updateTelemetry()` in `robotPeriodic()` and `yacsim.simIterate()` in `simulationPeriodic()`.

### Option B: Explicit `CadRobotConfig` + `YamsCommands3Bridge.autoBind`

If your subsystems construct their own YAMS mechanisms independently and you prefer to wire simulation separately in `RobotContainer` or `GearsCadSim`:

```java
CadRobotConfig robotConfig =
    CadRobotConfig.fromUrdfOrZip(urdfZip, urdfOpts)
        .withDrivetrainPoseSupplier(drive::getPose)
        .ignoreRestingOverlaps()
        .withAutoInstallToAdvantageScope();

// 1. Bind all mechanisms in one forgiving call (never throws on unmatched names)
YacSimDiagnostic.BindingReport report =
    YamsCommands3Bridge.autoBind(
        robotConfig,
        Map.of("Turret", turret.getPivot(), "Hood", hood.getArm()));
System.out.print(report.summaryText());

// 2. Overlay 4 animated rotating steer forks + spinning wheels onto the URDF chassis
YamsCommands3Bridge.attachSwerveModules(
    robotConfig, swerveDrive, steerForkAsset, wheelAsset, 0.0381, moduleLocations);

// 3. Start simulator (auto-exports Robot_<Name> bundle to AdvantageScope)
CadRobotSimulator simulator = new CadRobotSimulator(robotConfig);
```

---

## Code-Side CAD Repair (`CadUrdfOptions` in `v2027.4.0-alpha1`)

Real-world FRC CAD assemblies often have un-suppressed bolts, default mate names (`revolute_1`), or moving subparts accidentally mated to `root`. YACSim solves all of these directly in Java without requiring CAD rework:

| Option | What It Solves |
| :--- | :--- |
| `CadUrdfOptions.recommended()` | Enables `excludeFasteners(true)` (skips nuts, bolts, screws, washers, spacers, bearings, bushings, pins, collars, springs) and `disableUnboundCollisions(true)` (auto-re-enabling collision only when a non-roller mechanism is bound). |
| `.withJointAlias("revolute_1", "Turret")` | Maps default Onshape mate names to semantic Java names without editing Onshape. |
| `.keepOnlyJoints(List.of("Turret", "Hood"))` | Trims AdvantageScope `ComponentPoses` down to only the real moving joints. |
| `.bakeUnboundLinksIntoChassis(true)` | Instead of hiding non-selected structural links, bakes each unbound link into its **nearest kept moving ancestor's `.glb`** (automatic moving-subtree baking) or into `root.glb` at its exact cumulative URDF resting pose. |
| `.attachLinksToJoint("Turret", "launcher_side_plate")` | Re-parents mis-mated `root`-attached parts directly into a moving mechanism's `.glb` (transformed into that joint's local frame) so the entire subassembly moves rigidly together. |
| `.excludeMeshPatterns("MAXSwerve")` | Strips static simplified swerve blocks or unwanted meshes from `root.glb` so animated swerve modules overlay cleanly. |
| `.noSimplifyMeshPatterns("bumper", "frame_rail")` | Tags matching merged glTF nodes with `NOSIMPLIFY_` per [AdvantageScope Custom Assets](https://docs.advantagescope.org/more-features/custom-assets/) so AdvantageScope preserves fine geometry without disabling simplification globally. |

---

## Safe `*Or(...)` Fallback Getters

Lightweight URDF exports sometimes omit mate limits (for continuous joints) or mass/inertia (when a CAD part has no assigned material). Use the safe fallback getters instead of strict throwing getters:

- **`CadJointManifest`:** `minLimitAngleOr`, `maxLimitAngleOr`, `softMinLimitAngleOr`, `softMaxLimitAngleOr`, `startingAngleOr`, `minLimitDistanceOr`, `maxLimitDistanceOr`, `softMinLimitDistanceOr`, `softMaxLimitDistanceOr`, `startingDistanceOr`, `lengthOr`, `wheelRadiusOr`, `wheelDiameterOr`, `momentOfInertiaOr`.
- **`CadRobotManifest`:** `getLinkMassOr(String linkName, Mass fallback)`, `getJointMomentOfInertiaOr(String jointName, MomentOfInertia fallback)`.

---

## Tips & Troubleshooting

- **Reloading AdvantageScope after updating URDF options:** AdvantageScope caches `Robot_<Name>` bundles in memory. After changing `CadUrdfOptions` or re-exporting a URDF, click **Help → Reload Assets** (or restart AdvantageScope) to see updated meshes and component counts.
- **Finding generic part names (`part_1_15`):** `YacSimDiagnostic.printReport(robotConfig)` prints each movable joint's spatial resting position `@ (x, y, z)m` alongside fuzzy `.withJointAlias(...)` suggestions so you can identify unlabeled parts quickly.
- **Extracting to a persistent cache folder:** Call `CadRobotConfig.fromUrdfZip(zipPath, Path.of("build/yacsim-urdf"), urdfOpts)` if you want to inspect the extracted `.urdf` and merged `yacsim_merged/*.glb` files on disk.
- **Need cameras or Onshape subassembly grouping?** Chain `.addCamera(...)` and `.addRollerTelemetry(...)` onto your `CadRobotConfig`, or use the [Python Onshape Exporter](ONSHAPE_EXPORTER_GUIDE.md) to generate a full `yacsim-robot.json`.
