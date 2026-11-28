// Copyright (c) 2026 Nathan Moe / Yet Another CAD Simulator (YACSim) Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import yacsim.core.physics.DynamicWeightTransferModel.CornerLoads;
import yacsim.core.physics.PathTrajectoryValidator.TrajectoryWaypoint;
import yacsim.core.physics.PathTrajectoryValidator.ValidationLimits;
import yacsim.core.physics.PathTrajectoryValidator.ValidationReport;
import yacsim.core.physics.TerrainBumpHeightmap.ChassisTerrainSample;
import yacsim.core.physics.TireSlipCurveModel.TreadPreset;

final class ChassisDynamicsTerrainAndPathValidationTest {

  @Test
  void weightTransferConservesNormalLoadAndShowsHigherTipRiskWithElevatedCg() {
    DynamicWeightTransferModel model = new DynamicWeightTransferModel(0.60, 0.60);
    double massKg = 60.0;

    CornerLoads stowed =
        model.compute(massKg, new Translation3d(0.0, 0.0, 0.25), 4.0, 0.0, 0.0, 0.0);
    CornerLoads extended =
        model.compute(massKg, new Translation3d(0.0, 0.0, 0.95), 4.0, 0.0, 0.0, 0.0);

    double expectedTotal = massKg * DynamicWeightTransferModel.GRAVITY_MPS2;
    assertEquals(
        expectedTotal,
        stowed.frontLeftNewtons()
            + stowed.frontRightNewtons()
            + stowed.backLeftNewtons()
            + stowed.backRightNewtons(),
        1e-6);
    assertTrue(stowed.backLeftNewtons() > stowed.frontLeftNewtons());
    assertTrue(extended.tipRiskRatio() > stowed.tipRiskRatio());
    assertTrue(extended.isWheelLifting());
  }

  @Test
  void tireSlipCurvePeaksAtPeakSlipRatioAndDropsUnderWheelspin() {
    TireSlipCurveModel nitrile = new TireSlipCurveModel(TreadPreset.NITRILE_CARPET);
    TireSlipCurveModel worn = new TireSlipCurveModel(TreadPreset.WORN_TREAD_CARPET);

    double muZero = nitrile.frictionCoefficient(0.0);
    double muPeak = nitrile.frictionCoefficient(nitrile.getConfig().peakSlipRatio());
    double muSpin = nitrile.frictionCoefficient(0.85);

    assertEquals(0.0, muZero, 1e-9);
    assertEquals(nitrile.getConfig().peakMu(), muPeak, 1e-6);
    assertTrue(muSpin < muPeak);
    assertTrue(muSpin >= nitrile.getConfig().kineticMu() - 1e-6);
    assertTrue(nitrile.peakTractionForceNewtons(150.0) > worn.peakTractionForceNewtons(150.0));
  }

  @Test
  void terrainHeightmapComputesBumpElevationAndPitchSlope() {
    TerrainBumpHeightmap terrain =
        new TerrainBumpHeightmap()
            .addCableProtector("CableBump", 4.0, 0.0, 4.6, 8.0, 0.05)
            .addRampX("StageRamp", 8.0, 2.0, 10.0, 6.0, 0.30);

    assertEquals(0.05, terrain.heightAt(4.3, 4.0), 1e-6);
    assertEquals(0.0, terrain.heightAt(1.0, 4.0), 1e-9);

    ChassisTerrainSample onRamp =
        terrain.projectPoseToTerrain(new Pose2d(9.0, 4.0, Rotation2d.ZERO), 0.60, 0.60);
    assertTrue(onRamp.meanHeightMeters() > 0.10);
    // Climbing +X ramp -> nose up -> negative pitch around +Y
    assertTrue(onRamp.pitchRad() < -0.10);
    assertEquals(0.0, onRamp.rollRad(), 1e-6);
  }

  @Test
  void trajectoryValidatorPassesConservativePathAndFlagsAggressiveHighCgPath() {
    DynamicWeightTransferModel wt = new DynamicWeightTransferModel(0.60, 0.60);
    TireSlipCurveModel tire = new TireSlipCurveModel(TreadPreset.NITRILE_CARPET);
    TerrainBumpHeightmap terrain =
        new TerrainBumpHeightmap().addCableProtector("CableBump", 3.0, 0.0, 3.6, 8.0, 0.04);
    PathTrajectoryValidator validator = new PathTrajectoryValidator(wt, tire, terrain);

    List<TrajectoryWaypoint> gentle =
        List.of(
            new TrajectoryWaypoint(
                0.0, new Pose2d(1.0, 4.0, Rotation2d.ZERO), new ChassisVelocities(0.0, 0.0, 0.0)),
            new TrajectoryWaypoint(
                0.5, new Pose2d(1.5, 4.0, Rotation2d.ZERO), new ChassisVelocities(1.5, 0.0, 0.0)),
            new TrajectoryWaypoint(
                1.0, new Pose2d(2.5, 4.0, Rotation2d.ZERO), new ChassisVelocities(2.0, 0.0, 0.0)));
    ValidationReport gentleReport =
        validator.validate(
            gentle, 55.0, new Translation3d(0.0, 0.0, 0.25), ValidationLimits.FRC_DEFAULT);
    assertTrue(gentleReport.feasible());

    List<TrajectoryWaypoint> aggressive =
        List.of(
            new TrajectoryWaypoint(
                0.0, new Pose2d(1.0, 4.0, Rotation2d.ZERO), new ChassisVelocities(0.0, 0.0, 0.0)),
            new TrajectoryWaypoint(
                0.1, new Pose2d(1.2, 4.0, Rotation2d.ZERO), new ChassisVelocities(2.5, 0.0, 0.0)));
    ValidationReport aggressiveReport =
        validator.validate(
            aggressive, 55.0, new Translation3d(0.0, 0.0, 0.85), ValidationLimits.FRC_DEFAULT);
    assertFalse(aggressiveReport.feasible());
    assertTrue(aggressiveReport.firstViolationIndex() >= 0);
  }

  @Test
  void fuzzChassisDynamicsAndValidationRemainFiniteAndNonNegative() {
    SplittableRandom rng = new SplittableRandom(20270601L);
    DynamicWeightTransferModel wt = new DynamicWeightTransferModel(0.62, 0.58);
    TireSlipCurveModel tire = new TireSlipCurveModel(TreadPreset.COLSON_CARPET);
    TerrainBumpHeightmap terrain =
        new TerrainBumpHeightmap().addCableProtector("Bump", 4.0, 0.0, 4.8, 8.0, 0.06);
    PathTrajectoryValidator validator = new PathTrajectoryValidator(wt, tire, terrain);

    for (int i = 0; i < 200; i++) {
      double mass = rng.nextDouble(25.0, 75.0);
      Translation3d cg =
          new Translation3d(
              rng.nextDouble(-0.15, 0.15), rng.nextDouble(-0.15, 0.15), rng.nextDouble(0.10, 1.10));
      CornerLoads loads =
          wt.compute(
              mass,
              cg,
              rng.nextDouble(-10.0, 10.0),
              rng.nextDouble(-10.0, 10.0),
              rng.nextDouble(-0.35, 0.35),
              rng.nextDouble(-0.35, 0.35));
      assertTrue(Double.isFinite(loads.totalNormalNewtons()));
      assertTrue(loads.minCornerNewtons() >= 0.0);
      assertTrue(loads.tipRiskRatio() >= 0.0 && loads.tipRiskRatio() <= 1.0);

      double mu = tire.frictionCoefficient(rng.nextDouble(-1.5, 1.5));
      assertTrue(Double.isFinite(mu) && mu >= 0.0 && mu <= tire.getConfig().peakMu() + 1e-9);
    }

    List<TrajectoryWaypoint> path = new ArrayList<>();
    for (int k = 0; k < 20; k++) {
      path.add(
          new TrajectoryWaypoint(
              k * 0.05,
              new Pose2d(1.0 + k * 0.2, 4.0, Rotation2d.ZERO),
              new ChassisVelocities(3.0, 0.0, 0.0)));
    }
    ValidationReport rep =
        validator.validate(
            path, 58.0, new Translation3d(0.0, 0.0, 0.30), ValidationLimits.FRC_DEFAULT);
    assertTrue(Double.isFinite(rep.peakLinearAccelMps2()));
    assertTrue(Double.isFinite(rep.maxTerrainTiltDeg()));
  }
}
