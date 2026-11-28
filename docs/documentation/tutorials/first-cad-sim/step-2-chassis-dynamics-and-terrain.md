---
icon: gauge-high
---

# Step 2: Add Chassis Dynamics & Terrain

**Goal:** Model how your robot's weight shifts under acceleration, when tires lose grip, and how the chassis pitches over field bumps.

```java
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.physics.DynamicWeightTransferModel;
import yacsim.core.physics.DynamicWeightTransferModel.CornerLoads;
import yacsim.core.physics.TerrainBumpHeightmap;
import yacsim.core.physics.TerrainBumpHeightmap.ChassisTerrainSample;
import yacsim.core.physics.TireSlipCurveModel;
import yacsim.core.physics.TireSlipCurveModel.TreadPreset;

// 1. 4-corner weight transfer (0.60 m wheelbase x 0.60 m trackwidth)
DynamicWeightTransferModel weightModel = new DynamicWeightTransferModel(0.60, 0.60);

// 2. Pacejka-style tire grip curve
TireSlipCurveModel tireModel = new TireSlipCurveModel(TreadPreset.COLSON_GRIP.config());

// 3. Field terrain heightmap (cable protector + ramp)
TerrainBumpHeightmap terrain = new TerrainBumpHeightmap();
terrain.addCableProtector("CenterCable", 2.0, 0.0, 2.4, 8.21, 0.032);
terrain.addRampX("StageRamp", 4.5, 2.0, 6.5, 6.2, 0.14);

// Evaluate at your current robot pose and acceleration:
ChassisTerrainSample sample = terrain.projectPoseToTerrain(currentPose2d, 0.60, 0.60);
CornerLoads loads = weightModel.compute(
    56.0,                            // totalMassKg
    new Translation3d(0.0, 0.0, 0.42), // CG in robot frame
    accelXMps2,
    accelYMps2,
    sample.pitchRad(),
    sample.rollRad()
);

if (loads.isWheelLifting()) {
    System.out.println("Warning: inside wheel lifted! tipRiskRatio=" + loads.tipRiskRatio());
}
```

Next: [Step 3: Validate Autonomous Paths in CI](step-3-validate-autonomous-paths.md).
