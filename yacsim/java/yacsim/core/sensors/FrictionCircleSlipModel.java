// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.sensors;

import java.util.Objects;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Friction-circle swerve traction and wheel-slip model clamping total carpet traction force to
 * {@code mu * m * g} (equivalently {@code mu * m * g / 4} per module). When commanded acceleration
 * exceeds carpet grip, wheel-encoder odometry advances faster than true chassis motion—decoupling
 * fused odometry from {@code SimTruthPose3d} so teams can test skid detection and vision recovery.
 */
public final class FrictionCircleSlipModel {
  private static final double GRAVITY_MPS2 = 9.80665;

  /** Output of a single 20 ms traction-clamped step. */
  public record SlipStepResult(
      Pose2d truthPose2d,
      Pose2d wheelOdometryPose2d,
      double truthVxMps,
      double truthVyMps,
      double skidRatio,
      boolean slipping) {}

  private final SensorFaultConfig config;
  private Pose2d truthPose;
  private Pose2d odomPose;
  private double truthVx = 0.0;
  private double truthVy = 0.0;

  public FrictionCircleSlipModel(SensorFaultConfig config, Pose2d initialPose) {
    this.config = Objects.requireNonNull(config, "config");
    this.truthPose = Objects.requireNonNull(initialPose, "initialPose");
    this.odomPose = initialPose;
  }

  /**
   * Advances both true chassis pose (clamped by friction circle {@code a_max = mu * g}) and
   * wheel-encoder odometry pose (following commanded wheel velocities) by {@code dtSeconds}.
   */
  public SlipStepResult step(
      double dtSeconds, double commandedVxFieldMps, double commandedVyFieldMps, double omegaRadPerSec) {
    if (dtSeconds <= 0.0) {
      return new SlipStepResult(truthPose, odomPose, truthVx, truthVy, 1.0, false);
    }

    double maxAccelMps2 = config.frictionCoefficientMu() * GRAVITY_MPS2;
    double reqAx = (commandedVxFieldMps - truthVx) / dtSeconds;
    double reqAy = (commandedVyFieldMps - truthVy) / dtSeconds;
    double reqAccel = Math.hypot(reqAx, reqAy);

    boolean slipping = reqAccel > maxAccelMps2;
    double scale = slipping ? (maxAccelMps2 / reqAccel) : 1.0;
    truthVx += reqAx * scale * dtSeconds;
    truthVy += reqAy * scale * dtSeconds;

    double dTheta = omegaRadPerSec * dtSeconds;
    truthPose =
        new Pose2d(
            truthPose.getX() + truthVx * dtSeconds,
            truthPose.getY() + truthVy * dtSeconds,
            new Rotation2d(truthPose.getRotation().getRadians() + dTheta));

    // Wheel encoders integrate commanded wheel speeds (creating realistic skid drift when slipping)
    odomPose =
        new Pose2d(
            odomPose.getX() + commandedVxFieldMps * dtSeconds,
            odomPose.getY() + commandedVyFieldMps * dtSeconds,
            new Rotation2d(odomPose.getRotation().getRadians() + dTheta));

    double cmdSpeed = Math.hypot(commandedVxFieldMps, commandedVyFieldMps);
    double trueSpeed = Math.hypot(truthVx, truthVy);
    double skidRatio = trueSpeed > 1e-4 ? Math.max(1.0, cmdSpeed / trueSpeed) : (slipping ? 2.0 : 1.0);

    return new SlipStepResult(truthPose, odomPose, truthVx, truthVy, skidRatio, slipping);
  }
}
