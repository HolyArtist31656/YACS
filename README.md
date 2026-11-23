# YACSim — Yet Another CAD Simulator

[![YACSim CI](https://github.com/HolyArtist31656/YACS/actions/workflows/ci.yml/badge.svg)](https://github.com/HolyArtist31656/YACS/actions/workflows/ci.yml)
[![WPILib 2027](https://img.shields.io/badge/WPILib-2027__alpha7-blue)](https://docs.wpilib.org/en/2027/)
[![Java 25](https://img.shields.io/badge/Java-25-orange)](https://adoptium.net/)
[![License: LGPL-3.0](https://img.shields.io/badge/License-LGPL--3.0-green.svg)](LICENSE.txt)

**YACSim** is a lightweight, zero-extra-dependency 3D CAD simulation vendordep for **WPILib 2027 (`2027_alpha7`, Java 25, Commands v3)** built to pair seamlessly with [YAMS (Yet Another Mechanism Suite)](https://yams.yassrobotics.com/) and [AdvantageScope](https://docs.advantagescope.org/) on the 2 GB RAM SystemCore.

## Online Installation (WPILib 2027 VS Code)

Install via **WPILib: Manage Vendor Libraries → Install new libraries (online)**:

```text
https://holyartist31656.github.io/YACS/vendordeps/yacsim_commands3.json
```

- **Maven Repository**: `https://holyartist31656.github.io/YACS/maven/releases/`
- **Hosted Java API Javadoc**: `https://holyartist31656.github.io/YACS/javadoc/`
- **Required Dependencies**: `CommandsV3.json` and `yams_commands3.json`

## Features

- **AdvantageScope Import, Camera Frustums & Starter Layouts**: Import an existing `Robot_<Name>/config.json` in one line (`CadRobotConfig.fromAdvantageScopeDir`), apply CAD-tool presets (`CadCoordinateSystem.ONSHAPE_GLTF`, `SOLIDWORKS_MM`, `FUSION_360_MM`, `BLENDER_GLTF`), export camera frustums (`addCamera`), and generate a pre-wired AdvantageScope layout.
- **Native CAD Mesh Parsing (with In-Memory Cache)**: Parses `.glb` / `.gltf` (glTF 2.0), `.obj`, and ASCII/binary `.stl` directly in Java without loading GPU buffers or pulling third-party JSON/mesh JARs.
- **Direct YAMS & Commands v3 Adapters**: Wire YAMS `Arm`, `Elevator`, `Pivot`, `FlyWheel`, rollers/intake bars, and `SwerveDrive` mechanisms directly with `YamsCommands3Bridge`, `YamsMechanismBridge`, and `YamsDrivetrainBridge`—plus automatic `/YACSim/Rollers/<name>/` telemetry for baked-in wheels.
- **CAD-to-YAMS Mass & MOI Estimation**: Compute WPILib `Mass` and parallel-axis `MomentOfInertia` directly from parsed CAD meshes (`YamsMechanismBridge.estimateMass` / `estimateArmMoi`) to seed YAMS physics.
- **Multi-Primitive `.glb` Sub-OBBs & Auto-Calibrated Collisions**: Automatically derive legal FRC extension envelopes from your chassis CAD (`withExtensionMargin`), auto-ignore resting coupled-linkage OBB overlaps (`ignoreRestingOverlaps`), or configure explicit `ignoreCollisionPair(a, b)` rules and Commands v3 `CadCollisionTriggers`.
- **AdvantageScope 3D Pipeline**: Publishes `RobotPose`, `RobotPose3d`, `ComponentPoses`, `CollisionActive`, and `CollisionDetails` over NT4, and exports ready-to-load `Robot_<Name>` AdvantageScope asset bundles with ordered multi-axis rotations.
- **SystemCore 2 GB Validated**: Verified on FRC Team 6328's 2026 robot (*Darwin*, 2.1M+ triangles across 5 `.glb` meshes) under a strict `-Xmx256m` robot JVM cap with `~0.64 ms` `robotPeriodic()` telemetry overhead and zero steady-state loop overruns.

## Quickstart

```java
CadRobotConfig config =
    new CadRobotConfig("MyRobot")
        .withChassisAsset(new CadAssetConfig(modelsDir.resolve("chassis.glb")))
        .withDrivetrainPoseSupplier(drive::getPose)
        .withExtensionMargin(0.35, 1.85) // Auto-derived from chassis.glb bounds
        .ignoreRestingOverlaps()         // Auto-silences resting coupled-linkage OBB overlaps
        .autoGroundChassis()             // Shifts chassis minZ to 0.0 on the field carpet
        .withAutoInstallToAdvantageScope() // Exports directly to AdvantageScope User Assets
        .addComponent(
            new CadComponentConfig("Arm", new CadAssetConfig(modelsDir.resolve("arm.glb")))
                .withParentComponent("Chassis")
                .withPoseSupplier(
                    YamsCommands3Bridge.fromYamsArm(
                        arm, new Translation3d(0.2, 0.0, 0.45), MotionAxis.Y)))
        .addComponent(
            new CadComponentConfig("Elevator", new CadAssetConfig(modelsDir.resolve("elevator.glb")))
                .withParentComponent("Chassis")
                .withPoseSupplier(
                    YamsCommands3Bridge.fromYamsElevator(
                        elevator, new Translation3d(0.0, 0.0, 0.1), MotionAxis.Z)));

CadRobotMechanism cadSim = new CadRobotMechanism(this, config);
```

Call `cadSim.periodic()` from `robotPeriodic()` and `cadSim.simulationPeriodic()` from `simulationPeriodic()` (or a `Notifier`).

## Documentation

| Guide | Description |
| --- | --- |
| [Getting Started](docs/GETTING_STARTED.md) | Installation, prerequisites, first simulation, and AdvantageScope setup |
| [Guides](docs/GUIDES.md) · [CAD Manifest & URDF Guide](docs/CAD_MANIFEST_GUIDE.md) · [Onshape Exporter Guide](docs/ONSHAPE_EXPORTER_GUIDE.md) | CAD prep, multi-axis rotations, YAMS adapters, auto-calibrated collisions, and the FRC 6328 Darwin walkthrough |
| [Java API Reference](docs/API_REFERENCE.md) | Class-by-class reference across all 8 packages (+ [Hosted Javadoc](https://holyartist31656.github.io/YACS/javadoc/)) |
| [Unit Tests](docs/UNIT_TESTS.md) | Reference for all 8 JUnit 5 test classes (41 tests) and how to run them |
| [Architecture & Performance](docs/ARCHITECTURE.md) | SystemCore 2 GB memory model, lock-free `SimSnapshot` threading, and Darwin benchmarks |
| [Examples](examples/README.md) | Runnable WPILib 2027 Commands v3 examples (`arm_and_elevator_2027` & `darwin_6328_2027`) |
| [Changelog](docs/CHANGELOG.md) | Version history and release notes |

## Building Locally

```powershell
cd vendordep
.\gradlew build test javadoc "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## License

Licensed under **LGPL-3.0**, matching Yet Another Software Suite (YASS) / YAMS.
