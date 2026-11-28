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
