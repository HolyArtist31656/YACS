# Getting Started

This guide walks through installing **YACSim** in a WPILib 2027 Commands v3 project, wiring your first CAD robot simulation, and viewing live 3D motion in AdvantageScope.

## Prerequisites

| Requirement | Version |
| --- | --- |
| WPILib | `2027.0.0-alpha-7` (`2027_alpha7`) |
| JDK | Java 25 (bundled with WPILib 2027) |
| Command Framework | WPILib Commands v3 (`CommandsV3.json`) |
| Mechanism Library | YAMS Commands v3 (`yams_commands3.json`) |
| Visualizer | AdvantageScope (bundled with WPILib 2027 / SystemCore) |

## Installation

### Option 1: Online Vendordep (Recommended)

In WPILib 2027 VS Code, open the Command Palette (`Ctrl+Shift+P`) and run **WPILib: Manage Vendor Libraries → Install new libraries (online)**, then paste:

```text
https://holyartist31656.github.io/YACS/vendordeps/yacsim_commands3.json
```

Make sure `CommandsV3.json` and `yams_commands3.json` are already installed in `vendordeps/`.

### Option 2: Local Clone / Offline Maven

If you are developing YACSim locally from this repository:

```powershell
cd vendordep
.\gradlew build publish generateVendordepJson "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

This publishes `YACSim-core` and `YACSim-commands3` to `vendordep/build/maven/releases/` and writes `vendordep/build/vendordeps/yacsim_commands3.json`.

## Wiring Your First CAD Simulation

1. Place your exported `.glb` (or `.gltf`, `.obj`, `.stl`) meshes in `src/main/deploy/yacsim/models/`.
2. Create a `Mechanism` that wraps `CadRobotMechanism`:

```java
package frc.robot.mechanisms;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.system.Filesystem;
import yacsim.commands3.CadRobotMechanism;
import yacsim.commands3.YamsCommands3Bridge;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.yams.MotionAxis;
import yams.commands3.mechanisms.Arm;

public class RobotCadSim implements Mechanism {
  private final CadRobotMechanism cadSim;

  public RobotCadSim(Supplier<Pose2d> drivePose, Arm arm) throws IOException {
    Path models = Filesystem.getDeployDirectory().toPath().resolve("yacsim/models");

    CadRobotConfig config =
        new CadRobotConfig("MyRobot")
            .withChassisAsset(new CadAssetConfig(models.resolve("chassis.glb")))
            .withDrivetrainPoseSupplier(drivePose)
            .withExtensionMargin(0.35, 1.85) // Auto-derived from chassis.glb bounds
            .ignoreRestingOverlaps()         // Auto-ignores coupled sibling overlaps at t = 0
            .autoGroundChassis()               // Shifts chassis minZ flush to z = 0.0
            .withAutoInstallToAdvantageScope() // Installs Robot_<Name> straight into AdvantageScope
            .addComponent(
                new CadComponentConfig("Arm", new CadAssetConfig(models.resolve("arm.glb")))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(
                        YamsCommands3Bridge.fromYamsArm(
                            arm, new Translation3d(0.2, 0.0, 0.45), MotionAxis.Y)));

    this.cadSim = new CadRobotMechanism(this, config).startSimNotifier(0.02);
  }

  public void periodic() {
    cadSim.periodic();
  }

  public void simulationPeriodic() {
    cadSim.simulationPeriodic();
  }
}
```

3. Instantiate `RobotCadSim` in your `Robot()` constructor (WPILib 2027 removed `robotInit()`) and call `cadSim.periodic()` inside `robotPeriodic()`. Calling `.startSimNotifier(0.02)` automatically runs `simulationPeriodic()` on a background `Notifier` in desktop simulation (and is a safe no-op on real hardware).

## Running Desktop Simulation

In WPILib `2027.0.0-alpha-7`, desktop simulation uses Gradle's `application` plugin and the `:run` task:

```powershell
.\gradlew run "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## Viewing in AdvantageScope

1. With `.withAutoInstallToAdvantageScope()`, YACSim installs `Robot_<Name>` directly into your OS AdvantageScope `userAssets` folder automatically (or if you used `.withAdvantageScopeExportDir(...)`, copy `Robot_<Name>` directly into User Assets without nesting).
2. Connect AdvantageScope to `127.0.0.1` (NT4).
3. Open a **3D Field** tab, drag `NT:/YACSim/RobotPose3d` onto **Poses → Robot**, select your `Robot_<Name>` model, and drag `NT:/YACSim/ComponentPoses` onto **Components**.
4. Open a **Table** or **Line Graph** tab and add `NT:/YACSim/CollisionActive` and `NT:/YACSim/CollisionDetails` to monitor live OBB collisions.

## Optional: Automated CAD Manifest / URDF Setup

Instead of typing pivots, limits, wheel radii, and mass properties in Java, you can place a `yacsim-robot.json` manifest (or a standard `.urdf` file exported from Onshape, Fusion 360, or SolidWorks) in `src/main/deploy/yacsim/` and load it in one line:

```java
CadRobotConfig robotConfig = CadRobotConfig.fromManifest(
    Filesystem.getDeployDirectory().toPath().resolve("yacsim/darwin/yacsim-robot.json"));
YamsCommands3Bridge.bindFromManifest(robotConfig, "Hood", hood.getArm());
```

If you want to automatically fetch `yacsim-robot.json` and `.glb` meshes directly from Onshape, install **Python 3.10+** and link your Onshape API keys using the [Onshape Exporter Guide](ONSHAPE_EXPORTER_GUIDE.md) (Python is only needed for the exporter CLI, never for manual Java setup or robot runtime). See the [CAD Manifest & URDF Automation Guide](CAD_MANIFEST_GUIDE.md) and the [`darwin_6328_manifest_2027`](../examples/commands3/darwin_6328_manifest_2027/README.md) example for details.
