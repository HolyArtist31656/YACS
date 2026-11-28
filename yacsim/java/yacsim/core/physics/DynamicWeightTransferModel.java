// Copyright (c) 2026 Nathan Moe / Yet Another CAD Simulator (YACSim) Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.physics;

import java.util.Objects;
import org.wpilib.math.geometry.Translation3d;

/**
 * Quasi-static 4-corner normal-load weight-transfer model for FRC swerve drivetrains.
 *
 * <p>Computes front-left, front-right, back-left, and back-right wheel normal loads from dynamic
 * center-of-gravity coordinates (including moving elevators/arms), chassis planar acceleration, and
 * terrain pitch/roll tilt.
 */
public final class DynamicWeightTransferModel {
  public static final double GRAVITY_MPS2 = 9.80665;
  private static final double MIN_DIMENSION_METERS = 0.1;

  /**
   * Immutable 4-corner load distribution result.
   *
   * @param frontLeftNewtons Normal load on the front-left module in Newtons (&ge; 0).
   * @param frontRightNewtons Normal load on the front-right module in Newtons (&ge; 0).
   * @param backLeftNewtons Normal load on the back-left module in Newtons (&ge; 0).
   * @param backRightNewtons Normal load on the back-right module in Newtons (&ge; 0).
   * @param totalNormalNewtons Total gravity-projected normal load in Newtons.
   * @param tipRiskRatio Normalized tip-over risk in {@code [0, 1]} (1.0 when any wheel lifts).
   */
  public record CornerLoads(
      double frontLeftNewtons,
      double frontRightNewtons,
      double backLeftNewtons,
      double backRightNewtons,
      double totalNormalNewtons,
      double tipRiskRatio) {
    public double minCornerNewtons() {
      return Math.min(
          Math.min(frontLeftNewtons, frontRightNewtons),
          Math.min(backLeftNewtons, backRightNewtons));
    }

    public boolean isWheelLifting() {
      return minCornerNewtons() <= 1e-6;
    }
  }

  private final double wheelbaseMeters;
  private final double trackwidthMeters;

  public DynamicWeightTransferModel(double wheelbaseMeters, double trackwidthMeters) {
    if (!Double.isFinite(wheelbaseMeters) || wheelbaseMeters < MIN_DIMENSION_METERS) {
      throw new IllegalArgumentException("wheelbaseMeters must be >= " + MIN_DIMENSION_METERS);
    }
    if (!Double.isFinite(trackwidthMeters) || trackwidthMeters < MIN_DIMENSION_METERS) {
      throw new IllegalArgumentException("trackwidthMeters must be >= " + MIN_DIMENSION_METERS);
    }
    this.wheelbaseMeters = wheelbaseMeters;
    this.trackwidthMeters = trackwidthMeters;
  }

  /**
   * Evaluates 4-corner normal loads for a robot of {@code totalMassKg} with CG at {@code
   * cgRobotFrameMeters} (robot frame: +X forward, +Y left, +Z up).
   */
  public CornerLoads compute(
      double totalMassKg,
      Translation3d cgRobotFrameMeters,
      double accelXMps2,
      double accelYMps2,
      double pitchRad,
      double rollRad) {
    Objects.requireNonNull(cgRobotFrameMeters, "cgRobotFrameMeters must not be null");
    if (!Double.isFinite(totalMassKg) || totalMassKg <= 0.0) {
      throw new IllegalArgumentException("totalMassKg must be positive");
    }
    double ax = Double.isFinite(accelXMps2) ? accelXMps2 : 0.0;
    double ay = Double.isFinite(accelYMps2) ? accelYMps2 : 0.0;
    double pitch = Double.isFinite(pitchRad) ? pitchRad : 0.0;
    double roll = Double.isFinite(rollRad) ? rollRad : 0.0;

    double cosTilt = Math.max(0.0, Math.cos(pitch) * Math.cos(roll));
    double totalNormal = totalMassKg * GRAVITY_MPS2 * cosTilt;
    if (totalNormal <= 1e-9) {
      return new CornerLoads(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    }

    double halfL = 0.5 * wheelbaseMeters;
    double halfW = 0.5 * trackwidthMeters;
    double cgX = Math.clamp(cgRobotFrameMeters.getX(), -halfL, halfL);
    double cgY = Math.clamp(cgRobotFrameMeters.getY(), -halfW, halfW);
    double cgZ = Math.max(0.0, cgRobotFrameMeters.getZ());

    // Static weight distribution from CG offset
    double frontFrac = 0.5 + (cgX / wheelbaseMeters);
    double leftFrac = 0.5 + (cgY / trackwidthMeters);

    // Effective inertial + gravity slope accelerations along robot +X and +Y
    double effAx = ax + GRAVITY_MPS2 * Math.sin(pitch);
    double effAy = ay - GRAVITY_MPS2 * Math.sin(roll);

    // Longitudinal transfer: accelerating forward (+X) shifts load from front to rear
    double deltaFront = -(totalMassKg * effAx * cgZ) / wheelbaseMeters;
    // Lateral transfer: accelerating left (+Y) shifts load from left to right
    double deltaLeft = -(totalMassKg * effAy * cgZ) / trackwidthMeters;

    double frontTotal = totalNormal * frontFrac + deltaFront;
    double rearTotal = totalNormal * (1.0 - frontFrac) - deltaFront;

    double leftShiftFrac = deltaLeft / totalNormal;
    double effLeftFrac = leftFrac + leftShiftFrac;

    double rawFL = frontTotal * effLeftFrac;
    double rawFR = frontTotal * (1.0 - effLeftFrac);
    double rawBL = rearTotal * effLeftFrac;
    double rawBR = rearTotal * (1.0 - effLeftFrac);

    double nominalPerCorner = 0.25 * totalNormal;
    double rawMin = Math.min(Math.min(rawFL, rawFR), Math.min(rawBL, rawBR));
    double tipRisk = Math.clamp(1.0 - (rawMin / Math.max(1e-6, nominalPerCorner)), 0.0, 1.0);

    // Clamp negative loads (lifted wheels carry zero load) and renormalize to preserve totalNormal
    double fl = Math.max(0.0, rawFL);
    double fr = Math.max(0.0, rawFR);
    double bl = Math.max(0.0, rawBL);
    double br = Math.max(0.0, rawBR);
    double sum = fl + fr + bl + br;
    if (sum > 1e-9) {
      double scale = totalNormal / sum;
      fl *= scale;
      fr *= scale;
      bl *= scale;
      br *= scale;
    }

    return new CornerLoads(fl, fr, bl, br, totalNormal, tipRisk);
  }

  public double getWheelbaseMeters() {
    return wheelbaseMeters;
  }

  public double getTrackwidthMeters() {
    return trackwidthMeters;
  }
}
