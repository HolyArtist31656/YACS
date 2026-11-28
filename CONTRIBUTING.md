# Contributing to YACS (Yet Another CAD Simulator)

Thanks for contributing to **YACS**!

## Architecture & YASS Boundaries
- **YACS** owns CAD/URDF manifest loading, chassis dynamics (`DynamicWeightTransferModel`, `TireSlipCurveModel`, `TerrainBumpHeightmap`), `PathTrajectoryValidator`, 3D RK4 ballistics, non-camera 3D sensor simulation (`QuestNavSim`, `Lidar3dSim`), and AdvantageScope 3D visualization.
- Vision/camera simulation (`VisionSim`) lives upstream in **YALL**; mechanism kinematics live upstream in **YAMS** (bridged via public Pose SPIs like `getCascadePose3d()`).

## Local Verification Before Opening a PR
```bash
cd vendordep
./gradlew spotlessApply spotlessCheck build test generateVendordepJson javadoc
```
For changes to the Onshape exporters (`tools/onshape-exporter/` and `tools/onshape-field-exporter/`), also run:
```bash
python3 -m unittest discover -s tools/onshape-exporter -p "test_*.py"
python3 -m unittest discover -s tools/onshape-field-exporter -p "test_*.py"
```
