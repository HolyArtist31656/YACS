# Changelog

All notable changes to YACSim are documented here.

## `v2027.0.3-alpha1` — Initial Public Alpha

### Added
- **Core CAD Simulation (`yacsim:YACSim-core`)**:
  - Header-only `.glb`/`.gltf` and streaming `.obj`/`.stl` parser (`MeshParser`) with bounding-box, triangle-count, and `CadMassProperties` extraction.
  - 15-axis Separating Axis Theorem OBB collision detector (`ObbCollisionDetector`) supporting component-to-component, component-to-chassis, and `ExtensionEnvelope` checks.
  - Symmetric `CadRobotConfig.ignoreCollisionPair(componentA, componentB)` supporting sibling mechanisms, `"Chassis"`, and `"ExtensionEnvelope"`.
  - Ordered multi-axis zeroed rotations (`CadAssetConfig.addZeroedRotation(axis, degrees)`) exported verbatim to AdvantageScope `config.json`.
  - `YamsMechanismBridge` and `YamsDrivetrainBridge` for YAMS 2027 mechanisms and swerve drivetrains.
  - Lock-free `volatile SimSnapshot` threading between `simIterate()` and `updateTelemetry()`.
  - Zero-dependency `MinimalJson` parser and pretty-printer.
- **Commands v3 Bindings (`yacsim:YACSim-commands3`)**:
  - `CadRobotMechanism` lifecycle wrapper and `CadCollisionTriggers` (`onAnyCollision`, `onCollisionBetween`, `onExtensionViolation`).
- **Distribution & CI**:
  - `yacsim_commands3.json` declaring `CommandsV3.json` and `yams_commands3.json` in `requires`.
  - GitHub Actions CI (JDK 25, Spotless, 15 JUnit 5 tests, Javadoc) and GitHub Pages Maven/vendordep/Javadoc publishing.
- **Examples**:
  - `examples/commands3/arm_and_elevator_2027` — Starter swerve + elevator + arm simulation with bundled `.glb` meshes.
  - `examples/commands3/darwin_6328_2027` — Real-robot port of FRC 6328 *Darwin* (2026) using YAMS 2027 + YACSim, verified under `-Xmx256m`.
