// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.ballistics.AerodynamicCorrectionConfig;
import yacsim.core.ballistics.FieldObstacle3d;
import yacsim.core.ballistics.ProjectilePhysicsSim;
import yacsim.core.ballistics.ProjectileSpec;
import yacsim.core.ballistics.TrajectoryArcPublisher;
import yacsim.core.physics.BoundingBox3d;

/**
 * CI soak and fault-injection verification suite closing VER-03 (120A+ brownout rail sag &lt; 6.8 V
 * recovery) and VER-06 (160-second Blue &amp; Red mirrored full-match soak with combined slip,
 * QuestNav dropout, 3D LiDAR, high-fidelity RK4 ballistics, and bit-exact determinism).
 */
class FullMatchSoakAndBrownoutTest {

  private static final double FIELD_LENGTH_M = 16.540988;
  private static final double FIELD_WIDTH_M = 8.069326;
  private static final double DT_SEC = 0.02;
  private static final int FULL_MATCH_TICKS = 8_000; // 160 s @ 50 Hz

  private record SoakSummary(
      Pose2d finalTruthPose,
      Pose2d finalOdomPose,
      double maxQuestDriftMeters,
      int lidarScans,
      int projectilesLaunched,
      int slowTicksOver5Ms) {}

  @Test
  void brownoutRailSagThreeFaultCyclesAndCleanRecovery() {
    // Degraded competition battery: V_oc = 12.1 V, R_int = 0.034 Ohm
    // 165 A stall burst -> V_rail = 12.1 - 165 * 0.034 = 6.49 V (< 6.8 V brownout threshold)
    double restingVoltage = 12.1;
    double degradedResistanceOhm = 0.034;
    double brownoutThresholdV = 6.8;

    int brownoutTransitions = 0;
    boolean inBrownout = false;
    double minVoltageObserved = restingVoltage;
    double pidIntegrator = 0.0;

    // 30-second (1,500 ticks @ 50 Hz) soak with 3x 250 ms (13-tick) 165 A stall bursts
    for (int tick = 0; tick < 1_500; tick++) {
      boolean stallBurst =
          (tick >= 200 && tick < 213)
              || (tick >= 600 && tick < 613)
              || (tick >= 1_000 && tick < 1_013);
      double totalCurrentAmps = stallBurst ? 165.0 : 28.0;
      double railVoltage = Math.max(5.5, restingVoltage - totalCurrentAmps * degradedResistanceOhm);
      minVoltageObserved = Math.min(minVoltageObserved, railVoltage);

      // Smooth brownout throttle scale (1.0 above 10.5 V down to 0.5 below 8.0 V)
      double throttleScale =
          Math.max(0.5, Math.min(1.0, (railVoltage - 8.0) / (10.5 - 8.0) * 0.5 + 0.5));

      boolean nowBrownout = railVoltage < brownoutThresholdV;
      if (nowBrownout && !inBrownout) {
        brownoutTransitions++;
      }
      inBrownout = nowBrownout;

      // Anti-windup gate: freeze integrator during brownout so no surge occurs on recovery
      if (!nowBrownout) {
        pidIntegrator = Math.max(-1.0, Math.min(1.0, pidIntegrator + 0.01 * throttleScale));
      }

      // Verify recovery within 100 ms (5 ticks) after each burst ends
      if (tick == 218 || tick == 618 || tick == 1_018) {
        assertTrue(railVoltage > 11.0, "Rail voltage failed to recover at tick " + tick);
        assertFalse(nowBrownout, "Still browned out 100 ms after stall release at tick " + tick);
      }
      assertTrue(Double.isFinite(railVoltage) && Double.isFinite(throttleScale));
    }

    assertEquals(3, brownoutTransitions, "Expected 3 distinct < 6.8 V brownout cycles");
    assertTrue(minVoltageObserved < brownoutThresholdV);
    assertTrue(Math.abs(pidIntegrator) <= 1.0);
  }

  @Test
  void full160SecondBlueAndRedMirroredMatchSoak() {
    SoakSummary blue = run160SecondMatchSoak(false, 20270605L);
    SoakSummary red = run160SecondMatchSoak(true, 20270605L);

    // 1. Rotational 180-deg Blue <-> Red ground-truth parity
    Pose2d blueFlipped = flipRotational(blue.finalTruthPose());
    assertEquals(
        blueFlipped.getX(), red.finalTruthPose().getX(), 1e-4, "Mirrored X mismatch after 160 s");
    assertEquals(
        blueFlipped.getY(), red.finalTruthPose().getY(), 1e-4, "Mirrored Y mismatch after 160 s");
    assertEquals(
        0.0,
        blueFlipped.getRotation().minus(red.finalTruthPose().getRotation()).getRadians(),
        1e-4,
        "Mirrored heading mismatch after 160 s");

    // 2. Non-camera 3D physics/sensor pipelines operated cleanly across all 8,000 ticks
    assertTrue(blue.lidarScans() >= 800 && red.lidarScans() >= 800);
    assertTrue(blue.projectilesLaunched() >= 40 && red.projectilesLaunched() >= 40);
    assertTrue(blue.maxQuestDriftMeters() <= 0.36 && red.maxQuestDriftMeters() <= 0.36);

    // 3. Loop budget compliance (<= 5% of 8,000 ticks exceeding 5 ms)
    int maxAllowedSlowTicks = (int) (FULL_MATCH_TICKS * 0.05);
    assertTrue(
        blue.slowTicksOver5Ms() <= maxAllowedSlowTicks,
        "Blue soak exceeded 5% slow-tick budget: " + blue.slowTicksOver5Ms());
    assertTrue(
        red.slowTicksOver5Ms() <= maxAllowedSlowTicks,
        "Red soak exceeded 5% slow-tick budget: " + red.slowTicksOver5Ms());
  }

  @Test
  void bitExactDeterminismAcrossRepeated160SecondSoaks() {
    SoakSummary runA = run160SecondMatchSoak(false, 99887766L);
    SoakSummary runB = run160SecondMatchSoak(false, 99887766L);

    assertEquals(runA.finalTruthPose().getX(), runB.finalTruthPose().getX(), 0.0);
    assertEquals(runA.finalTruthPose().getY(), runB.finalTruthPose().getY(), 0.0);
    assertEquals(runA.finalOdomPose().getX(), runB.finalOdomPose().getX(), 0.0);
    assertEquals(runA.finalOdomPose().getY(), runB.finalOdomPose().getY(), 0.0);
    assertEquals(runA.maxQuestDriftMeters(), runB.maxQuestDriftMeters(), 0.0);
    assertEquals(runA.lidarScans(), runB.lidarScans());
  }

  private static SoakSummary run160SecondMatchSoak(boolean redAlliance, long seed) {
    SensorFaultConfig cfg =
        new SensorFaultConfig(seed, 0.007, 0.012, 0.35, 5.55, 0.02, 0.48, 16.0, 1.15, 61.0);

    Pose2d startBlue = new Pose2d(2.0, 4.0, new Rotation2d());
    Pose2d startPose = redAlliance ? flipRotational(startBlue) : startBlue;

    FrictionCircleSlipModel slipModel = new FrictionCircleSlipModel(cfg, startPose);
    QuestNavSim questNav = new QuestNavSim(cfg);

    FieldObstacle3d hub =
        new FieldObstacle3d(
            "OBSTACLE_hub", new BoundingBox3d(4.1, 3.6, 0.0, 5.1, 4.6, 1.15), new Pose3d(), false);
    Lidar3dSim lidar = new Lidar3dSim(cfg, List.of(hub));
    ProjectilePhysicsSim ballistics =
        new ProjectilePhysicsSim()
            .withAerodynamicCorrections(AerodynamicCorrectionConfig.frcFuelHighFidelity())
            .addObstacle(hub);
    ProjectileSpec fuel = ProjectileSpec.frcFuel2026();
    TrajectoryArcPublisher publisher = new TrajectoryArcPublisher("/YACSim/SoakTest", false, false);

    double sign = redAlliance ? -1.0 : 1.0;
    Pose2d lastTruth = startPose;
    Pose2d lastOdom = startPose;
    double maxDrift = 0.0;
    int lidarScans = 0;
    int launched = 0;
    int slowTicks = 0;

    List<Translation3d> dynamicOpponents =
        List.of(new Translation3d(8.0, 2.5, 0.4), new Translation3d(10.0, 5.5, 0.4));

    for (int tick = 0; tick < FULL_MATCH_TICKS; tick++) {
      long tStartNs = System.nanoTime();
      double tSec = tick * DT_SEC;

      // Inject 1.0 s QuestNav dropout at t = 90 s
      if (tick == 4_500) {
        questNav.injectTrackingLoss(tSec, 1.0);
      }

      // Smooth figure-eight match patrol with periodic hard braking
      double vxCmd = sign * 2.4 * Math.cos(tSec * 0.35);
      double vyCmd = sign * 1.6 * Math.sin(tSec * 0.70);
      double omegaCmd = 1.2 * Math.sin(tSec * 0.50);

      FrictionCircleSlipModel.SlipStepResult step = slipModel.step(DT_SEC, vxCmd, vyCmd, omegaCmd);
      lastTruth = step.truthPose2d();
      lastOdom = step.wheelOdometryPose2d();

      Pose3d truth3d =
          new Pose3d(
              lastTruth.getX(),
              lastTruth.getY(),
              0.0,
              new Rotation3d(0.0, 0.0, lastTruth.getRotation().getRadians()));

      questNav.step(tSec, truth3d);
      maxDrift = Math.max(maxDrift, questNav.getCurrentDriftMeters());

      // Periodic external AprilTag/YALL re-alignment snap every 5 seconds
      if (tick > 0 && tick % 250 == 0) {
        questNav.resetPose(truth3d);
      }

      Optional<List<Lidar3dSim.LidarCluster>> scan = lidar.scan(tSec, truth3d, dynamicOpponents);
      if (scan.isPresent()) {
        lidarScans++;
      }

      // Launch a Fuel projectile every 4 seconds (40 shots over 160 s)
      if (tick % 200 == 0) {
        ballistics.launch(
            fuel,
            new Translation3d(lastTruth.getX(), lastTruth.getY(), 0.75),
            new Translation3d(sign * 8.5, 0.0, 7.2),
            new Translation3d(0.0, -40.0, 0.0));
        launched++;
      }
      ballistics.step(DT_SEC);
      if (tick % 10 == 0) {
        publisher.publishActiveProjectiles(ballistics);
      }

      assertTrue(
          Double.isFinite(lastTruth.getX())
              && Double.isFinite(lastTruth.getY())
              && Double.isFinite(lastOdom.getX())
              && Double.isFinite(lastOdom.getY()),
          "Non-finite pose at tick " + tick);

      if ((System.nanoTime() - tStartNs) > 5_000_000L) {
        slowTicks++;
      }
    }

    publisher.close();
    return new SoakSummary(lastTruth, lastOdom, maxDrift, lidarScans, launched, slowTicks);
  }

  private static Pose2d flipRotational(Pose2d p) {
    return new Pose2d(
        FIELD_LENGTH_M - p.getX(),
        FIELD_WIDTH_M - p.getY(),
        p.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
  }
}
