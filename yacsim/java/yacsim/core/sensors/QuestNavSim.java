// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.sensors;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;
import java.util.Optional;
import java.util.Random;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Deterministic 120 Hz wired Meta Quest 3 / 3S VIO simulator ({@code QuestNav}) driven by YACSim's
 * ground-truth {@code SimTruthPose3d}. Models bounded random-walk drift, 5–10 ms wired USB-C
 * Ethernet transport latency, tracking-loss blackouts, and Phase 1 / Phase 2 {@link #resetPose}
 * field alignment snaps.
 */
public final class QuestNavSim {
  /** Timestamped VIO sample delivered after transport latency. */
  public record VioSample(
      Pose3d estimatedPose3d, double captureTimestampSec, boolean trackingValid) {}

  private final SensorFaultConfig config;
  private final Random rng;
  private final Deque<VioSample> latencyQueue = new ArrayDeque<>();

  private double driftX = 0.0;
  private double driftY = 0.0;
  private double driftYawRad = 0.0;
  private double blackoutUntilSec = -1.0;
  private double lastStepSec = Double.NaN;
  private VioSample latestDelivered = null;

  public QuestNavSim(SensorFaultConfig config) {
    this.config = Objects.requireNonNull(config, "config");
    this.rng = new Random(config.seed() ^ 0x517cc1b727220a95L);
  }

  /**
   * Injects a temporary VIO tracking-loss blackout (for example, simulating a hard impact or
   * passthrough interruption) until {@code currentTimestampSec + durationSec}.
   */
  public void injectTrackingLoss(double currentTimestampSec, double durationSec) {
    if (durationSec > 0.0) {
      this.blackoutUntilSec = Math.max(this.blackoutUntilSec, currentTimestampSec + durationSec);
    }
  }

  /**
   * Resets accumulated VIO drift to zero (mirroring QuestNav Phase 1 AprilTag hard alignment or
   * {@code /api/reset-pose}).
   */
  public void resetPose(Pose3d alignedFieldPose) {
    Objects.requireNonNull(alignedFieldPose, "alignedFieldPose");
    this.driftX = 0.0;
    this.driftY = 0.0;
    this.driftYawRad = 0.0;
    this.latencyQueue.clear();
    this.latestDelivered = new VioSample(alignedFieldPose, Double.isNaN(lastStepSec) ? 0.0 : lastStepSec, true);
  }

  /**
   * Steps the VIO simulator at {@code timestampSec} from ground-truth {@code truthPose3d} and
   * returns the latest latency-delayed {@link VioSample} ready for consumption.
   */
  public Optional<VioSample> step(double timestampSec, Pose3d truthPose3d) {
    Objects.requireNonNull(truthPose3d, "truthPose3d");
    double dt = Double.isNaN(lastStepSec) ? 0.02 : Math.max(0.0, timestampSec - lastStepSec);
    lastStepSec = timestampSec;

    if (config.questNavDriftMetersPerSec() > 0.0 && config.questNavMaxDriftMeters() > 0.0 && dt > 0.0) {
      double stepSigma = config.questNavDriftMetersPerSec() * Math.sqrt(dt);
      driftX = clamp(driftX + rng.nextGaussian() * stepSigma, -config.questNavMaxDriftMeters(), config.questNavMaxDriftMeters());
      driftY = clamp(driftY + rng.nextGaussian() * stepSigma, -config.questNavMaxDriftMeters(), config.questNavMaxDriftMeters());
      driftYawRad = clamp(driftYawRad + rng.nextGaussian() * stepSigma * 0.15, -0.08, 0.08);
    }

    boolean tracking = timestampSec >= blackoutUntilSec;
    Pose3d noisyPose =
        new Pose3d(
            new Translation3d(
                truthPose3d.getX() + driftX, truthPose3d.getY() + driftY, truthPose3d.getZ()),
            new Rotation3d(
                truthPose3d.getRotation().getX(),
                truthPose3d.getRotation().getY(),
                truthPose3d.getRotation().getZ() + driftYawRad));

    latencyQueue.addLast(new VioSample(noisyPose, timestampSec, tracking));
    while (!latencyQueue.isEmpty()
        && timestampSec - latencyQueue.peekFirst().captureTimestampSec()
            >= config.questNavLatencySec() - 1e-9) {
      latestDelivered = latencyQueue.removeFirst();
    }
    return Optional.ofNullable(latestDelivered);
  }

  /** @return Current Euclidean XY drift magnitude in meters. */
  public double getCurrentDriftMeters() {
    return Math.hypot(driftX, driftY);
  }

  private static double clamp(double v, double min, double max) {
    return Math.max(min, Math.min(max, v));
  }
}
