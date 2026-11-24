# YACSim Examples (WPILib 2027 / Commands v3)

Both examples are standalone WPILib `2027.0.0-alpha-7` GradleRIO projects configured to resolve YACSim from the relative local Maven repository (`../../../vendordep/build/maven/releases`).

## Prerequisites

Build and publish YACSim locally once before running either example:

```powershell
cd vendordep
.\gradlew build publish generateVendordepJson "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## 1. `commands3/arm_and_elevator_2027`

A self-contained starter robot with 5 bundled `.glb` meshes (`chassis.glb`, `swerve_fork.glb`, `swerve_wheel.glb`, `elevator_carriage.glb`, `arm.glb`) demonstrating `YamsDrivetrainBridge`, `YamsMechanismBridge.fromDistanceSupplier` / `fromAngleSupplier` / `chainParentChild`, `.withExtensionMargin(...)`, `.ignoreRestingOverlaps()`, `.autoGroundChassis()`, `.addCamera(...)`, `.withAutoInstallToAdvantageScope()`, and `AdvantageScopeConfigGenerator.exportStarterLayout(...)`.

```powershell
cd examples\commands3\arm_and_elevator_2027
.\gradlew run "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## 2. `commands3/darwin_6328_2027`

A real-robot port of [FRC Team 6328 Mechanical Advantage's 2026 robot (*Darwin*)](https://github.com/Mechanical-Advantage/RobotCode2026Public) (MIT License) using YAMS 2027 (`Arm`, `Elevator`, `FlyWheel`, `TalonFXWrapper` — covering Hood, Slamtake arm + IntakeRoller, FrontHopper, TrashCompactor, HopperRoller, KickerFront/Back, and 4-motor Launcher Flywheel) + YACSim (`CadRobotConfig.fromAdvantageScopeDir`, `withComponentPoseSupplier`, `.withExtensionMargin`, `.ignoreRestingOverlaps`, `.addCamera`, `exportStarterLayout`, and `YamsMechanismBridge`), configured with `-Xmx256m` on the robot JVM to validate 2 GB SystemCore headroom.

1. Download 6328's `.glb` meshes and `config.json` once:
   ```powershell
   cd examples\commands3\darwin_6328_2027
   .\setup_darwin_cad.ps1
   ```
2. Run the simulation:
   ```powershell
   .\gradlew run "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
   ```
3. Switch the WPILib Sim GUI to **Teleoperated** and connect AdvantageScope to `127.0.0.1` to view `YACSim/RobotPose3d`, `YACSim/ComponentPoses`, `YACSim/CollisionActive`, `YACSim/CollisionDetails`, and `/YACSim/Rollers/<name>/{VelocityRPM,SurfaceSpeedMPS,Spinning}`.

### 3. `commands3/darwin_6328_manifest_2027/` (Manifest-Driven CAD Automation)
- **Robot**: FRC 6328 Mechanical Advantage *Darwin* (2026) referencing Darwin's [Onshape Assembly](https://cad.onshape.com/documents/2da573df9d139d7a1e0f08a8/w/5266e52966df9f56fcb472f0/e/15a6a42133f059b818ad5264).
- **Highlights**: Loads all meshes, AdvantageScope zeroing, exact CAD mass/inertia, joint pivots, axes, hard/soft limits, drum/wheel radii, and cameras from `src/main/deploy/yacsim/darwin/yacsim-robot.json` via `CadRobotConfig.fromManifest(...)` and `YamsCommands3Bridge.bindFromManifest(...)`, leaving only CAN IDs, motor types, gear ratios, and PID/FF gains in Java.

### 4. `commands3/gears_2181_2027/` (FRC Team 2181 GEARS — REVLib 2027 + YAMS + YACSim)
- **Robot**: Port of [FRCteam2181/2026-Comp-Code](https://github.com/FRCteam2181/2026-Comp-Code) (`main` branch) to WPILib 2027 Commands v3, REVLib 2027 (`SparkMax`/`SparkFlex`), YAMS 2027, and YACSim.
- **Highlights**: All 7 mechanisms (`IntakeArm`, `Turret`, `Climber`, `Shooter`, `Spindexer`, `TopIntake`, `Input`) driven from `src/main/deploy/yacsim/gears2181/yacsim-robot.json` with automatic parent-mounted roller chaining (`TopIntake` riding on `IntakeArm`, `Shooter` riding on `Turret`), ready to drop in Team 2181's Onshape export with zero Java changes.
