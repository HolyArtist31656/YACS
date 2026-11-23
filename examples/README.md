# YACSim Examples (WPILib 2027 / Commands v3)

Both examples are standalone WPILib `2027.0.0-alpha-7` GradleRIO projects configured to resolve YACSim from the relative local Maven repository (`../../../vendordep/build/maven/releases`).

## Prerequisites

Build and publish YACSim locally once before running either example:

```powershell
cd vendordep
.\gradlew build publish generateVendordepJson "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## 1. `commands3/arm_and_elevator_2027`

A self-contained starter robot with 5 bundled `.glb` meshes (`chassis.glb`, `swerve_fork.glb`, `swerve_wheel.glb`, `elevator_carriage.glb`, `arm.glb`) demonstrating `YamsDrivetrainBridge`, `YamsMechanismBridge.fromDistanceSupplier` / `fromAngleSupplier` / `chainParentChild`, `.withExtensionMargin(...)`, `.ignoreRestingOverlaps()`, `.autoGroundChassis()`, and `.withAutoInstallToAdvantageScope()`.

```powershell
cd examples\commands3\arm_and_elevator_2027
.\gradlew run "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## 2. `commands3/darwin_6328_2027`

A real-robot port of [FRC Team 6328 Mechanical Advantage's 2026 robot (*Darwin*)](https://github.com/Mechanical-Advantage/RobotCode2026Public) (MIT License) using YAMS 2027 (`Arm`, `Elevator`, `TalonFXWrapper`) + YACSim (`.withExtensionMargin`, `.ignoreRestingOverlaps`, `YamsMechanismBridge`, and high-poly `.glb` parsing), configured with `-Xmx256m` on the robot JVM to validate 2 GB SystemCore headroom.

1. Download 6328's `.glb` meshes and `config.json` once:
   ```powershell
   cd examples\commands3\darwin_6328_2027
   .\setup_darwin_cad.ps1
   ```
2. Run the simulation:
   ```powershell
   .\gradlew run "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
   ```
3. Switch the WPILib Sim GUI to **Teleoperated** and connect AdvantageScope to `127.0.0.1` to view `YACSim/RobotPose3d`, `YACSim/ComponentPoses`, `YACSim/CollisionActive`, and `YACSim/CollisionDetails`.
