// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.ballistics;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.SplittableRandom;
import org.wpilib.math.geometry.Translation3d;

/**
 * Zero-dependency two-observable {@code (landingRangeMeters, entryAngleRad)} ballistic calibration
 * utility for fitting aerodynamic drag ({@code Cd}), Magnus spin lift ({@code Cm}), and spin-decay
 * rate ({@code lambda_spin}) from empirical shooter test shots.
 *
 * <p>Implements a deterministic train/validation split (typically ~20 varied-hood shots with 25%
 * held out for out-of-sample validation) and multi-scale coordinate pattern search over {@link
 * ProjectilePhysicsSim#simulateToLanding} at {@code dt = 0.001 s}.
 */
public final class BallisticCalibrationFitter {

  /**
   * One measured empirical calibration shot.
   *
   * @param launchSpeedMps Exit speed at muzzle in m/s.
   * @param hoodPitchRad Launch elevation angle above horizontal in radians.
   * @param backspinRadPerSec Backspin magnitude in rad/s (positive produces upward Magnus lift for
   *     +X launch via -Y spin axis).
   * @param measuredRangeMeters Observed horizontal landing distance in meters.
   * @param measuredEntryAngleRad Observed downward entry angle below horizontal in radians.
   * @param weight Relative inverse-variance weight ({@code > 0.0}, default {@code 1.0}).
   */
  public record CalibrationShot(
      double launchSpeedMps,
      double hoodPitchRad,
      double backspinRadPerSec,
      double measuredRangeMeters,
      double measuredEntryAngleRad,
      double weight) {

    /** Validates positive speed, range, and weight. */
    public CalibrationShot {
      if (launchSpeedMps <= 0.0 || !Double.isFinite(launchSpeedMps)) {
        throw new IllegalArgumentException("launchSpeedMps must be positive and finite.");
      }
      if (!Double.isFinite(hoodPitchRad) || hoodPitchRad <= 0.0 || hoodPitchRad >= Math.PI * 0.5) {
        throw new IllegalArgumentException("hoodPitchRad must be in (0, pi/2).");
      }
      if (!Double.isFinite(backspinRadPerSec)) {
        throw new IllegalArgumentException("backspinRadPerSec must be finite.");
      }
      if (measuredRangeMeters <= 0.0 || !Double.isFinite(measuredRangeMeters)) {
        throw new IllegalArgumentException("measuredRangeMeters must be positive and finite.");
      }
      if (!Double.isFinite(measuredEntryAngleRad)) {
        throw new IllegalArgumentException("measuredEntryAngleRad must be finite.");
      }
      if (weight <= 0.0 || !Double.isFinite(weight)) {
        throw new IllegalArgumentException("weight must be positive and finite.");
      }
    }
  }

  /**
   * Fitted aerodynamic parameters and train/validation residual metrics.
   *
   * @param fittedSpec Fitted {@link ProjectileSpec} with calibrated {@code Cd} and {@code Cm}.
   * @param fittedAero Fitted {@link AerodynamicCorrectionConfig} with calibrated spin decay.
   * @param trainRangeRmseMeters Training set horizontal range RMSE in meters.
   * @param validationRangeRmseMeters Held-out validation horizontal range RMSE in meters.
   * @param trainEntryAngleRmseDeg Training set entry-angle RMSE in degrees.
   * @param validationEntryAngleRmseDeg Held-out validation entry-angle RMSE in degrees.
   * @param trainCount Number of shots used for fitting.
   * @param validationCount Number of shots held out for validation.
   */
  public record CalibrationResult(
      ProjectileSpec fittedSpec,
      AerodynamicCorrectionConfig fittedAero,
      double trainRangeRmseMeters,
      double validationRangeRmseMeters,
      double trainEntryAngleRmseDeg,
      double validationEntryAngleRmseDeg,
      int trainCount,
      int validationCount) {}

  private static final double RANGE_NORM_METERS = 0.05;
  private static final double ANGLE_NORM_RAD = Math.toRadians(2.0);
  private static final double SIM_DT_SECONDS = 0.001;
  private static final double MAX_FLIGHT_SECONDS = 5.0;

  private BallisticCalibrationFitter() {}

  /**
   * Fits {@code (dragCoefficient, magnusLiftCoefficient, spinDecayRatePerSec)} against measured
   * {@code (landingRangeMeters, entryAngleRad)} observations using a deterministic holdout split.
   *
   * @param baseline Initial {@link ProjectileSpec}.
   * @param baseAero Initial {@link AerodynamicCorrectionConfig}.
   * @param launchHeightMeters Muzzle release height Z in meters.
   * @param targetHeightMeters Target plane elevation Z in meters.
   * @param shots Measured calibration shots (at least 4 recommended).
   * @param validationFraction Fraction of shots held out for validation in {@code [0.0, 0.5]}.
   * @param seed Deterministic {@link SplittableRandom} seed for reproducible splits.
   * @return Fitted {@link CalibrationResult}.
   */
  public static CalibrationResult fit(
      ProjectileSpec baseline,
      AerodynamicCorrectionConfig baseAero,
      double launchHeightMeters,
      double targetHeightMeters,
      List<CalibrationShot> shots,
      double validationFraction,
      long seed) {
    Objects.requireNonNull(baseline, "baseline");
    Objects.requireNonNull(baseAero, "baseAero");
    Objects.requireNonNull(shots, "shots");
    if (shots.size() < 2) {
      throw new IllegalArgumentException("At least 2 calibration shots are required.");
    }
    if (validationFraction < 0.0 || validationFraction > 0.5) {
      throw new IllegalArgumentException("validationFraction must be in [0.0, 0.5].");
    }

    // Deterministic Fisher-Yates shuffle
    List<CalibrationShot> shuffled = new ArrayList<>(shots);
    SplittableRandom rng = new SplittableRandom(seed);
    for (int i = shuffled.size() - 1; i > 0; i--) {
      int j = rng.nextInt(i + 1);
      CalibrationShot tmp = shuffled.get(i);
      shuffled.set(i, shuffled.get(j));
      shuffled.set(j, tmp);
    }

    int valCount = (int) Math.round(shuffled.size() * validationFraction);
    if (valCount >= shuffled.size()) {
      valCount = shuffled.size() - 1;
    }
    int trainCount = shuffled.size() - valCount;
    List<CalibrationShot> trainShots = shuffled.subList(0, trainCount);
    List<CalibrationShot> valShots =
        valCount > 0 ? shuffled.subList(trainCount, shuffled.size()) : trainShots;

    double cd = baseline.dragCoefficient();
    double cm = baseline.magnusLiftCoefficient();
    double decay = baseAero.spinDecayRatePerSec();

    double bestLoss =
        evaluateLoss(baseline, baseAero, launchHeightMeters, targetHeightMeters, trainShots, cd, cm, decay);

    // Multi-scale coordinate pattern search (zero allocations, fast convergence on 3D smooth bowl)
    double[] stepsCd = {0.10, 0.04, 0.015, 0.005, 0.0015};
    double[] stepsCm = {0.06, 0.025, 0.01, 0.003, 0.001};
    double[] stepsDecay = {0.20, 0.08, 0.03, 0.01, 0.003};

    for (int scale = 0; scale < stepsCd.length; scale++) {
      double dCd = stepsCd[scale];
      double dCm = stepsCm[scale];
      double dDecay = stepsDecay[scale];

      for (int iter = 0; iter < 8; iter++) {
        boolean improved = false;

        // Optimize Cd in [0.05, 1.50]
        for (double dir : new double[] {-1.0, 1.0}) {
          double cand = clamp(cd + dir * dCd, 0.05, 1.50);
          double loss =
              evaluateLoss(
                  baseline, baseAero, launchHeightMeters, targetHeightMeters, trainShots, cand, cm, decay);
          if (loss < bestLoss - 1e-9) {
            cd = cand;
            bestLoss = loss;
            improved = true;
            break;
          }
        }

        // Optimize Cm in [0.0, 0.60]
        for (double dir : new double[] {-1.0, 1.0}) {
          double cand = clamp(cm + dir * dCm, 0.0, 0.60);
          double loss =
              evaluateLoss(
                  baseline, baseAero, launchHeightMeters, targetHeightMeters, trainShots, cd, cand, decay);
          if (loss < bestLoss - 1e-9) {
            cm = cand;
            bestLoss = loss;
            improved = true;
            break;
          }
        }

        // Optimize spinDecay in [0.0, 2.50]
        for (double dir : new double[] {-1.0, 1.0}) {
          double cand = clamp(decay + dir * dDecay, 0.0, 2.50);
          double loss =
              evaluateLoss(
                  baseline, baseAero, launchHeightMeters, targetHeightMeters, trainShots, cd, cm, cand);
          if (loss < bestLoss - 1e-9) {
            decay = cand;
            bestLoss = loss;
            improved = true;
            break;
          }
        }

        if (!improved) {
          break;
        }
      }
    }

    ProjectileSpec fittedSpec = withCoeffs(baseline, cd, cm);
    AerodynamicCorrectionConfig fittedAero = baseAero.withSpinDecayRatePerSec(decay);

    double[] trainMetrics =
        evaluateRmse(fittedSpec, fittedAero, launchHeightMeters, targetHeightMeters, trainShots);
    double[] valMetrics =
        evaluateRmse(fittedSpec, fittedAero, launchHeightMeters, targetHeightMeters, valShots);

    return new CalibrationResult(
        fittedSpec,
        fittedAero,
        trainMetrics[0],
        valMetrics[0],
        Math.toDegrees(trainMetrics[1]),
        Math.toDegrees(valMetrics[1]),
        trainCount,
        valCount);
  }

  private static double evaluateLoss(
      ProjectileSpec baseline,
      AerodynamicCorrectionConfig baseAero,
      double launchZ,
      double targetZ,
      List<CalibrationShot> shots,
      double cd,
      double cm,
      double decay) {
    ProjectileSpec spec = withCoeffs(baseline, cd, cm);
    AerodynamicCorrectionConfig aero = baseAero.withSpinDecayRatePerSec(decay);
    ProjectilePhysicsSim sim = new ProjectilePhysicsSim().withAerodynamicCorrections(aero);

    double totalLoss = 0.0;
    double totalWeight = 0.0;
    Translation3d launchPos = new Translation3d(0.0, 0.0, launchZ);

    for (CalibrationShot shot : shots) {
      double vx = shot.launchSpeedMps() * Math.cos(shot.hoodPitchRad());
      double vz = shot.launchSpeedMps() * Math.sin(shot.hoodPitchRad());
      // Backspin about -Y produces upward Magnus lift (+Z) for +X launch: (-Y) x (+X) = +Z
      Translation3d spin = new Translation3d(0.0, -shot.backspinRadPerSec(), 0.0);
      Optional<ProjectilePhysicsSim.LandingObservation> obs =
          sim.simulateToLanding(
              spec,
              launchPos,
              new Translation3d(vx, 0.0, vz),
              spin,
              targetZ,
              SIM_DT_SECONDS,
              MAX_FLIGHT_SECONDS);
      if (obs.isEmpty()) {
        totalLoss += 1e4 * shot.weight();
      } else {
        double dr = (obs.get().horizontalRangeMeters() - shot.measuredRangeMeters()) / RANGE_NORM_METERS;
        double da = (obs.get().entryAngleRad() - shot.measuredEntryAngleRad()) / ANGLE_NORM_RAD;
        totalLoss += shot.weight() * (dr * dr + da * da);
      }
      totalWeight += shot.weight();
    }
    return totalWeight > 0.0 ? totalLoss / totalWeight : 1e6;
  }

  private static double[] evaluateRmse(
      ProjectileSpec spec,
      AerodynamicCorrectionConfig aero,
      double launchZ,
      double targetZ,
      List<CalibrationShot> shots) {
    ProjectilePhysicsSim sim = new ProjectilePhysicsSim().withAerodynamicCorrections(aero);
    Translation3d launchPos = new Translation3d(0.0, 0.0, launchZ);
    double sumSqRange = 0.0;
    double sumSqAngle = 0.0;
    int n = 0;

    for (CalibrationShot shot : shots) {
      double vx = shot.launchSpeedMps() * Math.cos(shot.hoodPitchRad());
      double vz = shot.launchSpeedMps() * Math.sin(shot.hoodPitchRad());
      Translation3d spin = new Translation3d(0.0, -shot.backspinRadPerSec(), 0.0);
      Optional<ProjectilePhysicsSim.LandingObservation> obs =
          sim.simulateToLanding(
              spec,
              launchPos,
              new Translation3d(vx, 0.0, vz),
              spin,
              targetZ,
              SIM_DT_SECONDS,
              MAX_FLIGHT_SECONDS);
      if (obs.isPresent()) {
        double dr = obs.get().horizontalRangeMeters() - shot.measuredRangeMeters();
        double da = obs.get().entryAngleRad() - shot.measuredEntryAngleRad();
        sumSqRange += dr * dr;
        sumSqAngle += da * da;
        n++;
      }
    }
    if (n == 0) {
      return new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
    }
    return new double[] {Math.sqrt(sumSqRange / n), Math.sqrt(sumSqAngle / n)};
  }

  private static ProjectileSpec withCoeffs(ProjectileSpec base, double cd, double cm) {
    return new ProjectileSpec(
        base.name(),
        base.massKg(),
        base.radiusMeters(),
        cd,
        base.airDensityKgPerM3(),
        cm,
        base.gravityMps2(),
        base.restitution());
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
