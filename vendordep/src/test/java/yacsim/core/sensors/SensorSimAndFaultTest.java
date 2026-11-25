// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.ballistics.FieldObstacle3d;
import yacsim.core.ballistics.ProjectilePhysicsSim;
import yacsim.core.ballistics.ProjectileSpec;
import yacsim.core.ballistics.TrajectoryArcPublisher;
import yacsim.core.physics.BoundingBox3d;

class SensorSimAndFaultTest {

  @Test
  void visionLatencyRingBufferDeliversDelayedFramesWithDynamicStdDev() {
    SensorFaultConfig cfg = SensorFaultConfig.realistic2027();
    NoisyVisionSim vision = new NoisyVisionSim(cfg);
    Pose3d truth = new Pose3d(3.2, 4.1, 0.0, new Rotation3d(0.0, 0.0, 0.5));

    // At t=0.0s, frame is captured into ring buffer (35 +/- 5 ms latency) -> not delivered yet
    List<NoisyVisionSim.VisionMeasurement> atZero = vision.step(0.0, truth, 3, 2.5);
    assertTrue(atZero.isEmpty(), "Delayed vision frame must not arrive instantaneously");
    assertEquals(1, vision.getPendingFrameCount());

    // Advance past 60 ms -> delayed frame(s) must arrive with distance-squared dynamic std-dev
    List<NoisyVisionSim.VisionMeasurement> received = new java.util.ArrayList<>();
    for (int tick = 1; tick <= 5; tick++) {
      received.addAll(vision.step(tick * 0.02, truth, 3, 2.5));
    }
    assertFalse(received.isEmpty(), "Delayed vision frames should drain from ring buffer");
    NoisyVisionSim.VisionMeasurement first = received.get(0);
    double expectedStd = cfg.visionBaseStdDevMeters() * (2.5 * 2.5) / 3.0;
    assertEquals(expectedStd, first.xyStdDevMeters(), 1e-6);
    assertTrue(first.captureTimestampSec() < 0.08);
  }

  @Test
  void visionBlackoutInjectsOver250MsDropoutAndRecoversCleanly() {
    SensorFaultConfig cfg =
        new SensorFaultConfig(
            42L, 0.02, 0.0, 0.04, 0.013, 0.0, 0.005, 0.01, 0.3, 5.55, 0.02, 0.48, 16.0, 1.15, 61.0);
    NoisyVisionSim vision = new NoisyVisionSim(cfg);
    Pose3d truth = new Pose3d(4.0, 4.0, 0.0, new Rotation3d());

    // Inject a 300 ms blackout starting at t=0.10s
    vision.injectBlackout(0.10, 0.30);

    int framesDuringBlackout = 0;
    for (int tick = 7; tick <= 19; tick++) {
      // t = 0.14s .. 0.38s (after 20ms pipeline drain, all frames must be suppressed)
      framesDuringBlackout += vision.step(tick * 0.02, truth, 2, 2.0).size();
    }
    assertEquals(
        0, framesDuringBlackout, "Zero vision frames should arrive during >250 ms blackout");

    // After t=0.40s, frames must recover cleanly
    int recovered = 0;
    for (int tick = 20; tick <= 25; tick++) {
      recovered += vision.step(tick * 0.02, truth, 2, 2.0).size();
    }
    assertTrue(recovered > 0, "Vision should recover cleanly after blackout expires");
  }

  @Test
  void questNavSimAccumulatesBoundedDriftAndSnapsOnResetPose() {
    SensorFaultConfig cfg = SensorFaultConfig.realistic2027();
    QuestNavSim quest = new QuestNavSim(cfg);
    Pose3d truth = new Pose3d(2.0, 3.0, 0.0, new Rotation3d(0.0, 0.0, 0.25));

    Optional<QuestNavSim.VioSample> sample = Optional.empty();
    for (int tick = 0; tick < 250; tick++) {
      sample = quest.step(tick * 0.02, truth);
    }
    assertTrue(sample.isPresent());
    assertTrue(sample.get().trackingValid());
    assertTrue(quest.getCurrentDriftMeters() > 0.0, "Realistic QuestNav should accumulate drift");
    assertTrue(
        quest.getCurrentDriftMeters() <= cfg.questNavMaxDriftMeters() * Math.sqrt(2.0) + 1e-6,
        "VIO drift must stay bounded");

    // Inject tracking loss and verify trackingValid flips false
    quest.injectTrackingLoss(5.0, 0.25);
    quest.step(5.04, truth);
    Optional<QuestNavSim.VioSample> lost = quest.step(5.06, truth);
    assertTrue(lost.isPresent());
    assertFalse(lost.get().trackingValid());

    // Phase 1 resetPose must clear accumulated drift to zero
    quest.resetPose(truth);
    assertEquals(0.0, quest.getCurrentDriftMeters(), 1e-9);
  }

  @Test
  void lidar3dSimRespectsCadenceSelfMaskingNoiseAndObbOcclusion() {
    SensorFaultConfig cfg = SensorFaultConfig.realistic2027(); // 5.55 Hz Unitree L2 (~0.18s period)
    FieldObstacle3d hubBase =
        new FieldObstacle3d(
            "OBSTACLE_blue_hub_base",
            new BoundingBox3d(4.1, 3.6, 0.0, 5.1, 4.6, 1.15),
            new Pose3d(),
            false);
    Lidar3dSim lidar = new Lidar3dSim(cfg, List.of(hubBase));

    Pose3d ego = new Pose3d(2.5, 4.1, 0.0, new Rotation3d());
    Translation3d selfChassisPoint = new Translation3d(2.7, 4.1, 0.5); // inside 0.48 m self mask
    Translation3d occludedOpponent = new Translation3d(6.5, 4.1, 0.5); // behind hubBase OBB
    Translation3d visibleOpponent = new Translation3d(3.5, 6.2, 0.5); // clear line of sight

    Optional<List<Lidar3dSim.LidarCluster>> scan0 =
        lidar.scan(0.0, ego, List.of(selfChassisPoint, occludedOpponent, visibleOpponent));
    assertTrue(scan0.isPresent(), "Initial scan at t=0.0s should fire");
    assertEquals(
        1, scan0.get().size(), "Only the unoccluded opponent outside self-mask should appear");
    assertTrue(
        scan0.get().get(0).fieldCenterMeters().getDistance(visibleOpponent) < 0.15,
        "Detected cluster should be within realistic ±2 cm Gaussian range noise");

    // Intermediate tick at t=0.06s (< 0.18s scan period) must return empty
    assertTrue(
        lidar.scan(0.06, ego, List.of(visibleOpponent)).isEmpty(),
        "LiDAR must honor 5.55 Hz circumferential scan cadence");

    // Next scan at t=0.20s must fire again
    assertTrue(lidar.scan(0.20, ego, List.of(visibleOpponent)).isPresent());
  }

  @Test
  void frictionCircleSlipDecouplesOdometryFromSimTruthPose() {
    SensorFaultConfig cfg = SensorFaultConfig.stressFaults(); // mu = 0.75 (~7.35 m/s^2 max accel)
    FrictionCircleSlipModel slip =
        new FrictionCircleSlipModel(cfg, new Pose2d(1.0, 2.0, new Rotation2d()));

    // Command an abrupt 5.0 m/s step in 20 ms (250 m/s^2 requested >> 7.35 m/s^2 grip limit)
    FrictionCircleSlipModel.SlipStepResult res = null;
    for (int i = 0; i < 15; i++) {
      res = slip.step(0.02, 5.0, 0.0, 0.0);
    }
    assertTrue(
        res.slipping(), "Abrupt acceleration on slippery carpet must trigger friction-circle slip");
    assertTrue(res.skidRatio() > 1.2, "Skid ratio should exceed 1.2 during wheel spin");
    assertTrue(
        res.wheelOdometryPose2d().getX() > res.truthPose2d().getX() + 0.25,
        "Wheel odometry must diverge ahead of SimTruthPose3d under sustained slip");
  }

  @Test
  void allianceMirroringRotationalVsReflectionPreservesBoundsAndHandlesFieldJitter() {
    double fieldLen = 16.54;
    double fieldWid = 8.07;
    Pose2d blue = new Pose2d(2.40, 4.04, Rotation2d.fromDegrees(30.0));

    // Rotational 180° symmetry preserves angular velocity sign; reflection flips heading sign
    Pose2d redRot =
        new Pose2d(
            fieldLen - blue.getX(),
            fieldWid - blue.getY(),
            blue.getRotation().plus(Rotation2d.fromDegrees(180.0)));
    Pose2d redRefl =
        new Pose2d(
            fieldLen - blue.getX(),
            blue.getY(),
            Rotation2d.fromDegrees(180.0).minus(blue.getRotation()));

    assertEquals(fieldLen, blue.getX() + redRot.getX(), 1e-9);
    assertEquals(fieldWid, blue.getY() + redRot.getY(), 1e-9);
    assertNotEquals(redRot.getRotation().getDegrees(), redRefl.getRotation().getDegrees());

    // ±13 mm Welded vs. AndyMark AprilTag jitter stays strictly bounded
    SensorFaultConfig cfg = SensorFaultConfig.realistic2027();
    assertEquals(0.013, cfg.visionFieldJitterMeters(), 1e-9);
  }

  @Test
  void batterySagModelCrossesSystemCoreBrownoutThresholdUnderDegradedResistance() {
    double openCircuitVolts = 12.2;
    double degradedResistanceOhms = 0.030;
    double surgeCurrentAmps = 210.0;
    double busVolts = openCircuitVolts - surgeCurrentAmps * degradedResistanceOhms;
    assertTrue(
        busVolts < 6.3,
        "Degraded battery under 210 A surge must cross SystemCore 6.3 V brownout cutoff");
  }

  @Test
  void unwrappedDoubleTurretAvoidsSilentRotation2dWrapping() {
    // WPILib 2027 Rotation2d getters wrap to (-pi, pi]; multi-turn turrets must track double angle
    double continuousTurretRad = Math.toRadians(225.0);
    Rotation2d wrapped = new Rotation2d(continuousTurretRad);
    assertTrue(wrapped.getRadians() < 0.0, "Rotation2d wraps 225 deg to -135 deg");
    assertEquals(Math.toRadians(225.0), continuousTurretRad, 1e-9);
  }

  @Test
  void trajectoryArcPublisherThrottlesUnchangedAndEmptyFrames() {
    ProjectilePhysicsSim sim = new ProjectilePhysicsSim();
    try (TrajectoryArcPublisher pub =
        new TrajectoryArcPublisher("YACSimThrottleTest", false, false)) {
      // 10 consecutive empty ticks should publish only ONCE (mitigating AdvantageScope #477)
      for (int i = 0; i < 10; i++) {
        pub.publishPredictedArc(List.of());
        pub.publishActiveProjectiles(sim);
      }
      assertEquals(1, pub.getPublishedArcFrames(), "Unchanged empty arc should publish only once");
      assertEquals(
          1,
          pub.getPublishedProjectileFrames(),
          "Unchanged empty projectile array should publish once");

      // Launching a projectile changes the array -> triggers a new publish frame
      sim.launch(
          ProjectileSpec.frcFuel2026(),
          new Translation3d(2.0, 4.0, 0.52),
          new Translation3d(3.0, 0.0, 7.0),
          new Translation3d());
      pub.publishActiveProjectiles(sim);
      assertEquals(2, pub.getPublishedProjectileFrames());
    }
  }

  @Test
  void deterministicSeedProducesBitExactSensorReplayAcrossRuns() {
    SensorFaultConfig cfg = SensorFaultConfig.realistic2027();
    QuestNavSim q1 = new QuestNavSim(cfg);
    QuestNavSim q2 = new QuestNavSim(cfg);
    NoisyVisionSim v1 = new NoisyVisionSim(cfg);
    NoisyVisionSim v2 = new NoisyVisionSim(cfg);

    for (int tick = 0; tick < 50; tick++) {
      double t = tick * 0.02;
      Pose3d truth = new Pose3d(1.5 + 0.04 * tick, 4.0, 0.0, new Rotation3d());
      assertEquals(q1.step(t, truth), q2.step(t, truth), "QuestNavSim must be bit-exact per seed");
      assertEquals(
          v1.step(t, truth, 2, 2.8),
          v2.step(t, truth, 2, 2.8),
          "NoisyVisionSim must be bit-exact per seed");
    }
  }
}
