# API Reference

## Packages
- `yacsim.core.physics`: `DynamicWeightTransferModel`, `TireSlipCurveModel`, `TerrainBumpHeightmap`, `PathTrajectoryValidator`, `ObbCollisionDetector`, `BoundingBox3d`, `CadMassProperties`
- `yacsim.core.manifest`: `CadManifestLoader`, `CadRobotManifest`, `CadJointManifest`, `CadLinkManifest`, `CadUrdfOptions`
- `yacsim.core.model`: `CadRobotSimulator`, `CadRobotConfig`, `CadComponentConfig`
- `yacsim.core.ballistics`: `ProjectilePhysicsSim`, `ProjectileSpec`, `BallisticCalibrationFitter`, `TrajectoryArcPublisher`
- `yacsim.core.advantagescope`: `AdvantageScopeConfigGenerator`, `AdvantageScopePublisher`, `CadCameraConfig`
- `yacsim.commands3`: `CadRobotMechanism`, `YacSimYamsFactory`, `YamsCommands3Bridge`, `CadCollisionTriggers`

Generate full HTML Javadoc locally with `cd vendordep && ./gradlew javadoc`.
