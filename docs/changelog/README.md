---
icon: clock-rotate-left
---

# Changelog

## `v2027.6.6-alpha1` (November 2026)

* Added `DynamicWeightTransferModel` (4-corner loads + 3-wheel lift), `TireSlipCurveModel` (Pacejka peak-to-kinetic grip), `TerrainBumpHeightmap`, and `PathTrajectoryValidator` (96/96 JUnit 5 + fuzz tests green).
* Synced to WPILib `2027.0.0-alpha-8` (Java 25) and YAMS `2026.11.06+` public Pose SPI bridges (`getCascadePose3d`, `getContinuousPose3d`, `getUpperPose3d`, `DifferentialMechanism.getPose3d`).
* Removed `NoisyVisionSim` in favor of upstream YALL `VisionSim` (clean YASS library boundaries).
