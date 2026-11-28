# Chassis Dynamics, Tire Slip & Terrain Validation

YACSim `v2027.6.6-alpha1` adds four zero-dependency physics models under `yacsim.core.physics`:
- **`DynamicWeightTransferModel`**: 4-corner normal loads from dynamic CG `(x, y, z)`, planar acceleration, and terrain pitch/roll, exposing `tipRiskRatio` and `isWheelLifting()`.
- **`TireSlipCurveModel`**: Simplified Pacejka traction curve with `NITRILE_CARPET`, `NEOPRENE_CARPET`, `COLSON_CARPET`, and `WORN_TREAD_CARPET` presets.
- **`TerrainBumpHeightmap`**: Analytic cable protectors (`COSINE_BUMP`), ramps (`LINEAR_RAMP_X`), and plateaus (`PLATEAU`) with 4-wheel `projectPoseToTerrain(...)`.
- **`PathTrajectoryValidator`**: Validates `YAPL`, Choreo, PathPlanner, or BLine trajectories against tip-over, tire grip, and terrain tilt limits before running on a real robot.
