// Copyright (c) 2026 Nathan Moe / Yet Another CAD Simulator (YACSim) Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.physics;

import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import yacsim.core.physics.DynamicWeightTransferModel.CornerLoads;
import yacsim.core.physics.TerrainBumpHeightmap.ChassisTerrainSample;

/**
 * Validates sampled trajectories (from YAPL, Choreo, PathPlanner, or BLine) against dynamic CG
 * weight transfer, tire traction limits, and field terrain heightmaps without coupling YACSim to
 * any pathing library JAR.
 */
public final class PathTrajectoryValidator {
  /** Single timestamped trajectory sample in field coordinates. */
  public record TrajectoryWaypoint(
      double timeSeconds, Pose2d pose, ChassisVelocities fieldVelocities) {
    public TrajectoryWaypoint {
      if (!Double.isFinite(timeSeconds) || timeSeconds < 0.0) {
        throw new IllegalArgumentException("timeSeconds must be finite and >= 0");
      }
      Objects.requireNonNull(pose, "pose must not be null");
      Objects.requireNonNull(fieldVelocities, "fieldVelocities must not be null");
    }
  }

  /** Validation thresholds. */
  public record ValidationLimits(
      double maxTipRiskRatio, double maxTractionDemandRatio, double maxTerrainTiltDeg) {
    public static final ValidationLimits FRC_DEFAULT = new ValidationLimits(0.85, 0.95, 22.0);
  }

  /** Aggregated trajectory feasibility report. */
  public record ValidationReport(
      boolean feasible,
      double peakLinearAccelMps2,
      double minCornerLoadNewtons,
      double peakTipRiskRatio,
      double peakTractionDemandRatio,
      double maxTerrainTiltDeg,
      int firstViolationIndex,
      String summary) {}

  private final DynamicWeightTransferModel weightTransfer;
  private final TireSlipCurveModel tireModel;
  private final TerrainBumpHeightmap terrain;

  public PathTrajectoryValidator(
      DynamicWeightTransferModel weightTransfer,
      TireSlipCurveModel tireModel,
      TerrainBumpHeightmap terrain) {
    this.weightTransfer = Objects.requireNonNull(weightTransfer, "weightTransfer must not be null");
    this.tireModel = Objects.requireNonNull(tireModel, "tireModel must not be null");
    this.terrain = Objects.requireNonNull(terrain, "terrain must not be null");
  }

  public ValidationReport validate(
      List<TrajectoryWaypoint> samples,
      double totalMassKg,
      Translation3d cgRobotFrameMeters,
      ValidationLimits limits) {
    Objects.requireNonNull(samples, "samples must not be null");
    Objects.requireNonNull(cgRobotFrameMeters, "cgRobotFrameMeters must not be null");
    Objects.requireNonNull(limits, "limits must not be null");
    if (samples.isEmpty()) {
      return new ValidationReport(true, 0.0, 0.0, 0.0, 0.0, 0.0, -1, "Empty trajectory");
    }

    double peakAccel = 0.0;
    double minCornerLoad = Double.POSITIVE_INFINITY;
    double peakTipRisk = 0.0;
    double peakTractionRatio = 0.0;
    double maxTiltDeg = 0.0;
    int firstViolation = -1;
    String violationReason = "OK";

    for (int i = 0; i < samples.size(); i++) {
      TrajectoryWaypoint curr = samples.get(i);
      double axField = 0.0;
      double ayField = 0.0;
      if (i > 0) {
        TrajectoryWaypoint prev = samples.get(i - 1);
        double dt = Math.max(1e-3, curr.timeSeconds() - prev.timeSeconds());
        axField = (curr.fieldVelocities().vx - prev.fieldVelocities().vx) / dt;
        ayField = (curr.fieldVelocities().vy - prev.fieldVelocities().vy) / dt;
      }

      // Rotate field acceleration into robot frame
      double cos = curr.pose().getRotation().getCos();
      double sin = curr.pose().getRotation().getSin();
      double axRobot = axField * cos + ayField * sin;
      double ayRobot = -axField * sin + ayField * cos;
      double accelMag = Math.hypot(axRobot, ayRobot);
      peakAccel = Math.max(peakAccel, accelMag);

      ChassisTerrainSample terrainSample =
          terrain.projectPoseToTerrain(
              curr.pose(),
              weightTransfer.getWheelbaseMeters(),
              weightTransfer.getTrackwidthMeters());
      double tiltDeg =
          Math.toDegrees(Math.hypot(terrainSample.pitchRad(), terrainSample.rollRad()));
      maxTiltDeg = Math.max(maxTiltDeg, tiltDeg);

      CornerLoads loads =
          weightTransfer.compute(
              totalMassKg,
              cgRobotFrameMeters,
              axRobot,
              ayRobot,
              terrainSample.pitchRad(),
              terrainSample.rollRad());
      minCornerLoad = Math.min(minCornerLoad, loads.minCornerNewtons());
      peakTipRisk = Math.max(peakTipRisk, loads.tipRiskRatio());

      double totalPeakTraction = tireModel.peakTractionForceNewtons(loads.totalNormalNewtons());
      double requiredForce = totalMassKg * accelMag;
      double tractionRatio =
          totalPeakTraction > 1e-6 ? requiredForce / totalPeakTraction : Double.POSITIVE_INFINITY;
      peakTractionRatio = Math.max(peakTractionRatio, tractionRatio);

      if (firstViolation < 0) {
        if (loads.tipRiskRatio() > limits.maxTipRiskRatio()) {
          firstViolation = i;
          violationReason = "Tip risk exceeded at sample " + i;
        } else if (tractionRatio > limits.maxTractionDemandRatio()) {
          firstViolation = i;
          violationReason = "Traction demand exceeded tire grip at sample " + i;
        } else if (tiltDeg > limits.maxTerrainTiltDeg()) {
          firstViolation = i;
          violationReason = "Terrain tilt exceeded limit at sample " + i;
        }
      }
    }

    boolean feasible = firstViolation < 0;
    return new ValidationReport(
        feasible,
        peakAccel,
        Double.isFinite(minCornerLoad) ? minCornerLoad : 0.0,
        peakTipRisk,
        peakTractionRatio,
        maxTiltDeg,
        firstViolation,
        feasible ? "Feasible" : violationReason);
  }
}
