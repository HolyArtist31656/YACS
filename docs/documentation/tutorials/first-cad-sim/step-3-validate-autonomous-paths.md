---
icon: circle-check
---

# Step 3: Validate Autonomous Paths in CI

**Goal:** Run `PathTrajectoryValidator` in a JUnit 5 test so GitHub Actions fails if an aggressive autonomous path would tip your robot or spin out on carpet.

```java
import java.util.List;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.physics.PathTrajectoryValidator;
import yacsim.core.physics.PathTrajectoryValidator.TrajectoryWaypoint;
import yacsim.core.physics.PathTrajectoryValidator.ValidationLimits;
import yacsim.core.physics.PathTrajectoryValidator.ValidationReport;

@Test
void autoPathDoesNotTipOrSkid() {
    PathTrajectoryValidator validator = new PathTrajectoryValidator(weightModel, tireModel, terrain);
    ValidationLimits limits = new ValidationLimits(
        0.75,  // maxTipRiskRatio (1.0 = wheel lift)
        0.92,  // maxTractionDemandRatio (1.0 = tire saturation)
        12.0   // maxTerrainTiltDeg
    );

    ValidationReport report = validator.validate(
        sampledAutoWaypoints,            // List<TrajectoryWaypoint> from YAPL/Choreo
        56.0,                            // totalMassKg
        new Translation3d(0.0, 0.0, 0.42),
        limits
    );

    assertTrue(report.feasible(), "Auto failed physics check: " + report.summary());
}
```

{% hint style="tip" %}
**Real-world lesson:** If `report.feasible()` returns `false` with a tip violation at sample `k`, lower `maxLinearAccelMps2` in your YAPL `MotionConstraints` (e.g., from `4.5 m/s²` down to `3.5 m/s²`) or keep your elevator stowed while cornering!
{% endhint %}

## Check it

Run `./gradlew test` and confirm `assertTrue(report.feasible(), report.summary())` passes for your safe path and fails if you double the acceleration limit.

## If it did not work

* **Report is unexpectedly feasible on an empty path:** Ensure your sampled waypoint list contains at least two waypoints before calling `validate(...)`.
* **Slip saturation warnings on every turn:** Check that you picked the matching `TreadPreset` (such as `COLSON_GRIP` or `NITRILE_CARPET`) for your wheels.
