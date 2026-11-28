# Yet Another CAD Simulator (YACSim / YACS)

**YACSim** is an FRC 2027 (WPILib `2027.0.0-alpha-8`, Java 25, Commands v3) 3D CAD physics, collision, chassis-dynamics, and ballistics simulation vendordep designed to work seamlessly with **YAMS**, **YAPL**, **YALL**, and **YAECE**.

## Highlights
- **Native Onshape URDF & GLB/STL/OBJ import** (`CadManifestLoader`, `MeshParser`, `GlbMerger`) with automated camera coordinate extraction (`CadCameraConfig`).
- **3D OBB collision & extension-envelope enforcement** (`ObbCollisionDetector`, `CadRobotSimulator`).
- **High-fidelity chassis dynamics & trajectory validation** (`DynamicWeightTransferModel`, `TireSlipCurveModel`, `TerrainBumpHeightmap`, `PathTrajectoryValidator`).
- **RK4 3D projectile ballistics** with drag, Magnus spin lift, spin decay, buoyancy, and added mass (`ProjectilePhysicsSim`).
