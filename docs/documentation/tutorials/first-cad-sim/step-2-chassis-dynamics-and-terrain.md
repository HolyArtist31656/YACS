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

## Check it

Run `./gradlew test` and verify `DynamicWeightTransferModel` reports `tipRiskRatio < 1.0` on flat carpet and a non-zero pitch angle when crossing a cable protector.

## If it did not work

* **Instant tip-over reported on flat ground:** Check that track width, wheelbase, and CG height are in meters (not inches).
* **Zero height change over a bump:** Confirm the robot's `(x, y)` coordinates overlap the rectangle passed to `addCableProtector(...)`.
