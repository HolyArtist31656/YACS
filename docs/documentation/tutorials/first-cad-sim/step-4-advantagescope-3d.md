---
icon: eye
---

# Step 4: View 3D Telemetry in AdvantageScope

**Goal:** Watch your robot tilt over cable protectors, articulate mechanisms, and launch 3D game pieces inside AdvantageScope.

{% stepper %}
{% step %}
### Publish your 3D chassis and mechanism poses

Publish `sample.pose3d()` (from `TerrainBumpHeightmap.projectPoseToTerrain(...)`) as your robot's `Pose3d` struct, and publish mechanism `Pose3d[]` arrays using WPILib `StructArrayPublisher<Pose3d>`.
{% endstep %}

{% step %}
### Configure AdvantageScope's 3D Field tab

1. Open **AdvantageScope** and add a **3D Field** tab.
2. Select your exported robot CAD model (or the default KitBot/custom GLB).
3. Drag `Sim/RobotPose3d` to **Robot** and `Sim/MechanismPoses` to **Components**.
4. Drag `Sim/Ballistics/Projectiles` to **Game Pieces** to watch RK4 trajectories arc into the goal in real time.
{% endstep %}
{% endstepper %}

{% code title="first/robot/CadSimTelemetry.java" %}
```java
package first.robot;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import yacsim.core.physics.TerrainBumpHeightmap;

public final class CadSimTelemetry implements AutoCloseable {
  private final StructPublisher<Pose3d> robotPosePub;
  private final StructArrayPublisher<Pose3d> mechanismPosesPub;

  public CadSimTelemetry() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    this.robotPosePub = nt.getStructTopic("Sim/RobotPose3d", Pose3d.struct).publish();
    this.mechanismPosesPub =
        nt.getStructArrayTopic("Sim/MechanismPoses", Pose3d.struct).publish();
  }

  public void publish(TerrainBumpHeightmap terrain, Pose2d pose2d, Pose3d[] mechanisms) {
    Pose3d pose3d = terrain.projectPoseToTerrain(pose2d).pose3d();
    robotPosePub.set(pose3d);
    mechanismPosesPub.set(mechanisms);
  }

  @Override
  public void close() {
    robotPosePub.close();
    mechanismPosesPub.close();
  }
}
```
{% endcode %}

## Check it

Start simulation, open AdvantageScope's **3D Field** tab, and drag `Sim/RobotPose3d` onto **Robot**. Driving over a cable protector should visibly pitch the 3D robot chassis.

## If it did not work

* **`Sim/RobotPose3d` is missing:** Verify `publish(...)` runs every loop inside `simulationPeriodic()` and AdvantageScope is connected to `localhost`.
* **Custom robot model appears sideways:** Check your CAD export coordinate convention (WPILib uses `+X` forward, `+Y` left, `+Z` up).
