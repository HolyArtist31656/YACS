# Contributing to YACS (Yet Another CAD Simulator)

Thanks for contributing to **YACS**!

## Architecture & YASS Boundaries
- **YACS** owns CAD/URDF manifest loading, chassis dynamics (`DynamicWeightTransferModel`, `TireSlipCurveModel`, `TerrainBumpHeightmap`), `PathTrajectoryValidator`, 3D RK4 ballistics, and AdvantageScope 3D visualization.
- Vision simulation (`VisionSim`) lives upstream in **YALL**; mechanism kinematics live upstream in **YAMS** (bridged via public Pose SPIs like `getCascadePose3d()`).

## Local Verification Before Opening a PR
```bash
cd vendordep
./gradlew spotlessApply spotlessCheck build test javadoc
```
For changes to the Onshape exporter (`tools/onshape-exporter/`), also run:
```bash
python3 -m unittest discover -s tools/onshape-exporter -p "test_*.py"
```
