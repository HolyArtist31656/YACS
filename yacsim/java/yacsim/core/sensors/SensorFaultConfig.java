// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.sensors;

/**
 * Deterministic noise, latency, dropout, and traction-slip configuration for {@code
 * yacsim.core.sensors} simulation models.
 *
 * @param seed Deterministic RNG seed for bit-exact replayability in JUnit 5 CI.
 * @param visionLatencyMeanSec Mean vision pipeline latency in seconds (default {@code 0.035}).
 * @param visionLatencyStdDevSec Vision latency Gaussian std-dev in seconds (default {@code 0.005}).
 * @param visionBaseStdDevMeters Base XY std-dev at 1 m for 1 tag (scaled by {@code d^2 / tags}).
 * @param visionFieldJitterMeters Welded vs. AndyMark AprilTag placement jitter (default {@code
 *     0.013} m).
 * @param visionDropoutProbability Per-frame probability of a zero-tag frame (in {@code [0, 1]}).
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
    double visionLatencyMeanSec,
    double visionLatencyStdDevSec,
    double visionBaseStdDevMeters,
    double visionFieldJitterMeters,
    double visionDropoutProbability,
    double questNavLatencySec,
    double questNavDriftMetersPerSec,
    double questNavMaxDriftMeters,
    double lidarScanRateHz,
    double lidarRangeNoiseStdDevMeters,
    double lidarSelfMaskRadiusMeters,
    double lidarMaxRangeMeters,
    double frictionCoefficientMu,
    double robotMassKg) {

  /** Validates finite non-negative sensor parameters. */
  public SensorFaultConfig {
    if (visionLatencyMeanSec < 0.0 || visionLatencyStdDevSec < 0.0) {
      throw new IllegalArgumentException("Vision latency parameters must be non-negative.");
    }
    if (visionBaseStdDevMeters < 0.0 || visionFieldJitterMeters < 0.0) {
      throw new IllegalArgumentException("Vision noise parameters must be non-negative.");
    }
    if (visionDropoutProbability < 0.0 || visionDropoutProbability > 1.0) {
      throw new IllegalArgumentException("visionDropoutProbability must be in [0.0, 1.0].");
    }
    if (questNavLatencySec < 0.0
        || questNavDriftMetersPerSec < 0.0
        || questNavMaxDriftMeters < 0.0) {
      throw new IllegalArgumentException("QuestNav parameters must be non-negative.");
    }
    if (lidarScanRateHz <= 0.0
        || lidarRangeNoiseStdDevMeters < 0.0
        || lidarSelfMaskRadiusMeters < 0.0
        || lidarMaxRangeMeters <= 0.0) {
      throw new IllegalArgumentException("LiDAR parameters must be positive/non-negative.");
    }
    if (frictionCoefficientMu <= 0.0 || robotMassKg <= 0.0) {
      throw new IllegalArgumentException("frictionCoefficientMu and robotMassKg must be positive.");
    }
  }

  /** Zero-noise, zero-latency baseline for reference comparisons. */
  public static SensorFaultConfig ideal() {
    return new SensorFaultConfig(
        2027L, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.45, 20.0, 10.0, 61.0);
  }

  /**
   * Realistic 2027 competition profile: 35 ± 5 ms AprilTag vision with ±13 mm Welded/AndyMark
   * jitter, 7 ms wired QuestNav 3S VIO, 5.55 Hz Unitree 4D LiDAR L2 (±2 cm noise), and μ=1.15
   * carpet friction.
   */
  public static SensorFaultConfig realistic2027() {
    return new SensorFaultConfig(
        2027L, 0.035, 0.005, 0.06, 0.013, 0.05, 0.007, 0.012, 0.35, 5.55, 0.02, 0.48, 16.0, 1.15,
        61.0);
  }

  /** High-fault CI stress profile: elevated latency, 25% frame dropouts, slippery carpet μ=0.75. */
  public static SensorFaultConfig stressFaults() {
    return new SensorFaultConfig(
        202706L, 0.060, 0.015, 0.12, 0.020, 0.25, 0.012, 0.035, 0.60, 5.55, 0.035, 0.48, 12.0, 0.75,
        61.0);
  }
}
