---
icon: code
---

# YACS API Reference

| Package | Key Public Classes |
| --- | --- |
| `yacsim.core.manifest` | `CadManifestLoader`, `CadRobotManifest`, `CadLinkManifest`, `CadJointManifest`, `CadUrdfOptions`, `YacSimDiagnostic` |
| `yacsim.core.advantagescope` | `AdvantageScopeConfigGenerator`, `AdvantageScopePublisher`, `CadCameraConfig` |
| `yacsim.core.physics` | `DynamicWeightTransferModel`, `TireSlipCurveModel`, `TerrainBumpHeightmap`, `PathTrajectoryValidator`, `ObbCollisionDetector`, `BoundingBox3d`, `CadMassProperties` |
| `yacsim.core.ballistics` | `ProjectilePhysicsSim`, `ProjectileSpec`, `ProjectileState`, `AerodynamicCorrectionConfig`, `BallisticCalibrationFitter`, `TrajectoryArcPublisher` |
| `yacsim.core.sensors` | `QuestNavSim`, `Lidar3dSim`, `FrictionCircleSlipModel`, `SensorFaultConfig` |
| `yacsim.core.cad` | `MeshParser`, `GlbMerger`, `CadAssetConfig`, `CadCoordinateSystem`, `CadMeshValidationException` |
| `yacsim.core.model` | `CadRobotSimulator`, `CadRobotConfig`, `CadComponentConfig`, `CadRollerTelemetryConfig` |
| `yacsim.core.drivetrain` | `YamsDrivetrainBridge` |
| `yacsim.core.yams` | `YamsMechanismBridge`, `MotionAxis` |
| `yacsim.core.util` | `MinimalJson` |
| `yacsim.commands3` | `YamsCommands3Bridge`, `CadRobotMechanism`, `CadCollisionTriggers`, `YacSimYamsFactory` |

Full Javadoc is published on [HolyArtist31656/YACS](https://github.com/HolyArtist31656/YACS).
