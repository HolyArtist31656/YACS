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

- **Native CAD Mesh Parsing**: Parses `.glb` / `.gltf` (glTF 2.0), `.obj`, and ASCII/binary `.stl` directly in Java without loading GPU buffers or pulling third-party JSON/mesh JARs.
- **YAMS & Commands v3 Integration**: Pairs with YAMS `Arm`, `Elevator`, `Pivot`, and `SwerveDrive` via `CadRobotMechanism`, `YamsMechanismBridge`, and `YamsDrivetrainBridge`.
- **AdvantageScope 3D Pipeline**: Publishes `RobotPose`, `RobotPose3d`, `ComponentPoses`, `CollisionActive`, and `CollisionDetails` over NT4, and exports ready-to-load `Robot_<Name>` AdvantageScope asset bundles with ordered multi-axis rotations.
- **OBB Collision & Envelope Checking**: Runs oriented bounding box (OBB) collision checks between mechanisms, chassis, and FRC extension envelopes, with symmetric `ignoreCollisionPair(a, b)` exclusions and Commands v3 `CadCollisionTriggers`.
- **SystemCore 2 GB Validated**: Verified on FRC Team 6328's 2026 robot (*Darwin*, 2.1M+ triangles across 5 `.glb` meshes) under a strict `-Xmx256m` robot JVM cap with `~0.64 ms` `robotPeriodic()` telemetry overhead and zero steady-state loop overruns.

## Quickstart

```java
CadRobotConfig config =
    new CadRobotConfig("MyRobot")
        .withChassisAsset(new CadAssetConfig(modelsDir.resolve("chassis.glb")))
        .withDrivetrainPoseSupplier(drive::getPose)
        .withExtensionEnvelope(new BoundingBox3d(-0.75, -0.75, 0.0, 0.75, 0.75, 1.85))
        .addComponent(
            new CadComponentConfig("Arm", new CadAssetConfig(modelsDir.resolve("arm.glb")))
                .withParentComponent("Chassis")
                .withPoseSupplier(arm::getPose3d))
        .ignoreCollisionPair("Intake", "Hopper");

CadRobotMechanism cadSim = new CadRobotMechanism(this, config);
```

Call `cadSim.periodic()` from `robotPeriodic()` and `cadSim.simulationPeriodic()` from `simulationPeriodic()` (or a `Notifier`).

## Documentation

| Guide | Description |
| --- | --- |
| [Getting Started](docs/GETTING_STARTED.md) | Installation, prerequisites, first simulation, and AdvantageScope setup |
| [Guides](docs/GUIDES.md) | CAD prep, multi-axis rotations, YAMS bridges, OBB collisions, and the FRC 6328 Darwin walkthrough |
| [Java API Reference](docs/API_REFERENCE.md) | Class-by-class reference across all 8 packages (+ [Hosted Javadoc](https://holyartist31656.github.io/YACS/javadoc/)) |
| [Unit Tests](docs/UNIT_TESTS.md) | Reference for all 7 JUnit 5 test classes (15 tests) and how to run them |
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
