// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.sensors;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Random;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.ballistics.FieldObstacle3d;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.ObbCollisionDetector;

/**
 * 360-degree 3D LiDAR cluster simulator (Unitree 4D LiDAR L2 at 5.55 Hz or Livox Mid-360 at 10 Hz)
 * with self-chassis footprint masking, ±2 cm Gaussian range noise, and line-of-sight occlusion
 * against static {@link FieldObstacle3d} OBBs loaded from {@code yacsim-field.json}.
 */
public final class Lidar3dSim {
  /** Dynamic cluster centroid detected after static-field CAD subtraction. */
  public record LidarCluster(
      Translation3d fieldCenterMeters,
      double rangeMeters,
      double confidence,
      double timestampSec) {}

  private final SensorFaultConfig config;
  private final List<FieldObstacle3d> staticObstacles;
  private final Random rng;
  private double lastScanTimestampSec = Double.NEGATIVE_INFINITY;

  public Lidar3dSim(SensorFaultConfig config, List<FieldObstacle3d> staticObstacles) {
    this.config = Objects.requireNonNull(config, "config");
    this.staticObstacles = staticObstacles == null ? List.of() : List.copyOf(staticObstacles);
    this.rng = new Random(config.seed() ^ 0xbf58476d1ce4e5b9L);
  }

  /**
   * Evaluates a 360-degree scan at {@code timestampSec}. Returns {@link Optional#empty()} between
   * circumferential scans ({@code 1 / lidarScanRateHz}), or a list of unoccluded dynamic clusters
   * outside the robot's self-chassis mask with Gaussian range noise applied.
   */
  public Optional<List<LidarCluster>> scan(
      double timestampSec, Pose3d egoTruthPose3d, List<Translation3d> dynamicTargetsField) {
    Objects.requireNonNull(egoTruthPose3d, "egoTruthPose3d");
    double scanPeriodSec = 1.0 / config.lidarScanRateHz();
    if (timestampSec - lastScanTimestampSec < scanPeriodSec - 1e-9) {
      return Optional.empty();
    }
    lastScanTimestampSec = timestampSec;

    Translation3d sensorOrigin =
        new Translation3d(
            egoTruthPose3d.getX(), egoTruthPose3d.getY(), egoTruthPose3d.getZ() + 0.55);
    List<LidarCluster> detected = new ArrayList<>();
    if (dynamicTargetsField == null) {
      return Optional.of(detected);
    }

    for (Translation3d target : dynamicTargetsField) {
      double dx = target.getX() - sensorOrigin.getX();
      double dy = target.getY() - sensorOrigin.getY();
      double dz = target.getZ() - sensorOrigin.getZ();
      double horizDist = Math.hypot(dx, dy);
      double range = Math.sqrt(dx * dx + dy * dy + dz * dz);

      // Mask robot's own chassis footprint and enforce max range
      if (horizDist <= config.lidarSelfMaskRadiusMeters() || range > config.lidarMaxRangeMeters()) {
        continue;
      }
      if (isOccludedByStaticObstacle(sensorOrigin, target)) {
        continue;
      }

      double noisyRange =
          Math.max(0.05, range + rng.nextGaussian() * config.lidarRangeNoiseStdDevMeters());
      double scale = noisyRange / Math.max(1e-6, range);
      Translation3d noisyCenter =
          new Translation3d(
              sensorOrigin.getX() + dx * scale,
              sensorOrigin.getY() + dy * scale,
              sensorOrigin.getZ() + dz * scale);
      double confidence = Math.max(0.5, 1.0 - 0.025 * noisyRange);
      detected.add(new LidarCluster(noisyCenter, noisyRange, confidence, timestampSec));
    }
    return Optional.of(detected);
  }

  private boolean isOccludedByStaticObstacle(Translation3d origin, Translation3d target) {
    if (staticObstacles.isEmpty()) {
      return false;
    }
    int samples = 12;
    BoundingBox3d probe = new BoundingBox3d(-0.04, -0.04, -0.04, 0.04, 0.04, 0.04);
    for (int i = 1; i < samples; i++) {
      double t = (double) i / samples;
      Pose3d pt =
          new Pose3d(
              new Translation3d(
                  origin.getX() + (target.getX() - origin.getX()) * t,
                  origin.getY() + (target.getY() - origin.getY()) * t,
                  origin.getZ() + (target.getZ() - origin.getZ()) * t),
              new Rotation3d());
      for (FieldObstacle3d obs : staticObstacles) {
        if (!obs.scoringTarget()
            && ObbCollisionDetector.testObbCollision(probe, pt, obs.localBounds(), obs.pose())
                .isPresent()) {
          return true;
        }
      }
    }
    return false;
  }
}
