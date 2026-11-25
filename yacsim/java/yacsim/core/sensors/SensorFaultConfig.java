// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.sensors;

/**
 * Deterministic noise, latency, dropout, and traction-slip configuration for non-camera 3D
 * environment and chassis physics simulation models in {@code yacsim.core.sensors}.
 *
 * <p>Camera and AprilTag simulation is intentionally excluded from YACSim and owned by YALL
 * ({@code yall.sim.VisionSim} and {@code yall.sim.CameraModel}) to preserve strict separation of
 * purposes across the YASS ecosystem.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * SensorFaultConfig cfg = SensorFaultConfig.realistic2027();
 * QuestNavSim quest = new QuestNavSim(cfg);
 * Lidar3dSim lidar = new Lidar3dSim(cfg, fieldObstacles);
 * FrictionCircleSlipModel slip = new FrictionCircleSlipModel(cfg, startPose);
 * }</pre>
 *
 * @param seed Deterministic RNG seed for bit-exact replayability in JUnit 5 CI.
 * @param questNavLatencySec Wired USB-C Ethernet VIO transport latency in seconds (default {@code
 *     0.007}).
 * @param questNavDriftMetersPerSec Bounded random-walk XY drift rate in m/s (default {@code
 *     0.012}).
 * @param questNavMaxDriftMeters Saturation cap on cumulative VIO drift in meters (default {@code
 *     0.35}).
 * @param lidarScanRateHz Circumferential LiDAR scan rate in Hz ({@code 5.55} for Unitree L2, {@code
 *     10.0} for Livox Mid-360).
 * @param lidarRangeNoiseStdDevMeters Gaussian range noise std-dev in meters (default {@code 0.02}).
 * @param lidarSelfMaskRadiusMeters Chassis exclusion radius around robot origin (default {@code
 *     0.48} m).
 * @param lidarMaxRangeMeters Maximum effective detection range in meters (default {@code 16.0} m).
 * @param frictionCoefficientMu Carpet static friction coefficient for swerve slip (default {@code
 *     1.15}).
 * @param robotMassKg Total robot mass including battery and bumpers in kg (default {@code 61.0}).
 */
public record SensorFaultConfig(
    long seed,
    double questNavLatencySec,
    double questNavDriftMetersPerSec,
    double questNavMaxDriftMeters,
    double lidarScanRateHz,
    double lidarRangeNoiseStdDevMeters,
    double lidarSelfMaskRadiusMeters,
    double lidarMaxRangeMeters,
    double frictionCoefficientMu,
    double robotMassKg) {

  /** Validates finite non-negative/positive sensor and traction parameters. */
  public SensorFaultConfig {
    if (!Double.isFinite(questNavLatencySec)
        || !Double.isFinite(questNavDriftMetersPerSec)
        || !Double.isFinite(questNavMaxDriftMeters)
        || questNavLatencySec < 0.0
        || questNavDriftMetersPerSec < 0.0
        || questNavMaxDriftMeters < 0.0) {
      throw new IllegalArgumentException("QuestNav parameters must be finite and non-negative.");
    }
    if (!Double.isFinite(lidarScanRateHz)
        || !Double.isFinite(lidarRangeNoiseStdDevMeters)
        || !Double.isFinite(lidarSelfMaskRadiusMeters)
        || !Double.isFinite(lidarMaxRangeMeters)
        || lidarScanRateHz <= 0.0
        || lidarRangeNoiseStdDevMeters < 0.0
        || lidarSelfMaskRadiusMeters < 0.0
        || lidarMaxRangeMeters <= 0.0
        || lidarMaxRangeMeters <= lidarSelfMaskRadiusMeters) {
      throw new IllegalArgumentException(
          "LiDAR parameters must be finite, non-negative, with lidarMaxRangeMeters > lidarSelfMaskRadiusMeters.");
    }
    if (!Double.isFinite(frictionCoefficientMu)
        || !Double.isFinite(robotMassKg)
        || frictionCoefficientMu <= 0.0
        || robotMassKg <= 0.0) {
      throw new IllegalArgumentException(
          "frictionCoefficientMu and robotMassKg must be finite and positive.");
    }
  }

  /** Zero-noise, zero-latency baseline for reference comparisons. */
  public static SensorFaultConfig ideal() {
    return new SensorFaultConfig(2027L, 0.0, 0.0, 0.0, 10.0, 0.0, 0.45, 20.0, 10.0, 61.0);
  }

  /**
   * Realistic 2027 competition profile: 7 ms wired QuestNav 3S VIO, 5.55 Hz Unitree 4D LiDAR L2
   * (±2 cm noise, 0.48 m self-mask), and μ=1.15 carpet friction.
   */
  public static SensorFaultConfig realistic2027() {
    return new SensorFaultConfig(2027L, 0.007, 0.012, 0.35, 5.55, 0.02, 0.48, 16.0, 1.15, 61.0);
  }

  /** High-fault CI stress profile: elevated VIO drift, noisier LiDAR, and slippery carpet μ=0.75. */
  public static SensorFaultConfig stressFaults() {
    return new SensorFaultConfig(202706L, 0.012, 0.035, 0.60, 5.55, 0.035, 0.48, 12.0, 0.75, 61.0);
  }
}
