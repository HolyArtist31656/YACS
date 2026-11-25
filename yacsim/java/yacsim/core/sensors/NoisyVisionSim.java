// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.sensors;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Objects;
import java.util.Random;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Ring-buffered delayed AprilTag / MegaTag2 vision simulator with distance-squared covariance
 * scaling ({@code baseStdDev * d^2 / tagCount}), Welded vs. AndyMark AprilTag jitter ({@code ±13
 * mm}), stochastic frame dropouts, and explicit blackout injection ({@code >250 ms}).
 */
public final class NoisyVisionSim {
  /** Single delayed vision pose observation ready for WPILib/YAECE pose fusion. */
  public record VisionMeasurement(
      Pose3d estimatedPose3d,
      double captureTimestampSec,
      int tagCount,
      double avgTagDistanceMeters,
      double xyStdDevMeters,
      double thetaStdDevRad) {}

  private record PendingFrame(double deliverAtSec, VisionMeasurement measurement) {}

  private final SensorFaultConfig config;
  private final Random rng;
  private final Deque<PendingFrame> ringBuffer = new ArrayDeque<>();
  private double blackoutUntilSec = -1.0;

  public NoisyVisionSim(SensorFaultConfig config) {
    this.config = Objects.requireNonNull(config, "config");
    this.rng = new Random(config.seed() ^ 0x9e3779b97f4a7c15L);
  }

  /** Injects a deterministic vision blackout until {@code currentTimestampSec + durationSec}. */
  public void injectBlackout(double currentTimestampSec, double durationSec) {
    if (durationSec > 0.0) {
      this.blackoutUntilSec = Math.max(this.blackoutUntilSec, currentTimestampSec + durationSec);
    }
  }

  /**
   * Captures a noisy vision frame at {@code timestampSec} (unless dropped or blacked out) and
   * returns all delayed frames whose delivery timestamp {@code <= timestampSec}.
   *
   * @param timestampSec Current simulation timestamp in seconds.
   * @param truthPose3d Ground-truth robot 3D pose.
   * @param visibleTagCount Number of AprilTags visible in this frame.
   * @param avgTagDistanceMeters Mean distance to visible tags in meters.
   * @return Zero or more latency-delayed {@link VisionMeasurement}s ready at {@code timestampSec}.
   */
  public List<VisionMeasurement> step(
      double timestampSec, Pose3d truthPose3d, int visibleTagCount, double avgTagDistanceMeters) {
    Objects.requireNonNull(truthPose3d, "truthPose3d");

    boolean dropped =
        visibleTagCount <= 0
            || timestampSec < blackoutUntilSec
            || (config.visionDropoutProbability() > 0.0
                && rng.nextDouble() < config.visionDropoutProbability());

    if (!dropped) {
      double dist = Math.max(0.5, avgTagDistanceMeters);
      double xyStd =
          config.visionBaseStdDevMeters() * (dist * dist) / Math.max(1, visibleTagCount);
      double totalSigma = Math.hypot(xyStd, config.visionFieldJitterMeters());
      double thetaStd = visibleTagCount >= 2 ? 0.04 * dist : 9999999.0;

      double nx = truthPose3d.getX() + rng.nextGaussian() * totalSigma;
      double ny = truthPose3d.getY() + rng.nextGaussian() * totalSigma;
      double nz = truthPose3d.getZ() + rng.nextGaussian() * (totalSigma * 0.5);
      double nyaw =
          truthPose3d.getRotation().getZ()
              + (visibleTagCount >= 2 ? rng.nextGaussian() * 0.015 * dist : 0.0);

      double latency =
          Math.max(
              0.0,
              config.visionLatencyMeanSec()
                  + rng.nextGaussian() * config.visionLatencyStdDevSec());
      VisionMeasurement meas =
          new VisionMeasurement(
              new Pose3d(
                  new Translation3d(nx, ny, nz),
                  new Rotation3d(
                      truthPose3d.getRotation().getX(), truthPose3d.getRotation().getY(), nyaw)),
              timestampSec,
              visibleTagCount,
              dist,
              xyStd,
              thetaStd);
      ringBuffer.addLast(new PendingFrame(timestampSec + latency, meas));
    }

    List<VisionMeasurement> ready = new ArrayList<>();
    while (!ringBuffer.isEmpty() && ringBuffer.peekFirst().deliverAtSec() <= timestampSec + 1e-9) {
      ready.add(ringBuffer.removeFirst().measurement());
    }
    return ready;
  }

  /** @return Number of captured frames still in flight inside the latency ring buffer. */
  public int getPendingFrameCount() {
    return ringBuffer.size();
  }
}
