// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.ballistics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Waypoint B (v2027.6.5-alpha1) tests covering buoyancy, added-mass, spin-decay RK4 at 0.001 s,
 * two-observable (landing, entryAngle) calibration recovery, and seeded SplittableRandom
 * property-based fuzz checks (MIT/LGPL-clean alternative to jqwik v1.10+).
 */
class WaypointBHighFidelityBallisticsTest {

  private static final double FIELD_LENGTH_M = 16.540988;
  private static final double FIELD_WIDTH_M = 8.069326;

  @Test
  void buoyancyAndAddedMassReduceEffectiveGravityAndIncreaseInertia() {
    ProjectileSpec fuel = ProjectileSpec.frcFuel2026();
    AerodynamicCorrectionConfig aero = AerodynamicCorrectionConfig.frcFuelHighFidelity();

    double displacedVol = AerodynamicCorrectionConfig.displacedVolumeM3(fuel);
    double buoyancy = aero.buoyancyForceNewtons(fuel);
    double weight = fuel.massKg() * fuel.gravityMps2();
    double buoyancyPct = buoyancy / weight;

    // For 0.22 kg, 0.075 m radius foam Fuel: displaced air mass ~ 2.16 g -> ~1% buoyancy
    assertTrue(displacedVol > 0.0015 && displacedVol < 0.0020);
    assertTrue(buoyancyPct > 0.005 && buoyancyPct < 0.05);
    assertTrue(aero.effectiveMassKg(fuel) > fuel.massKg());

    ProjectilePhysicsSim baselineSim = new ProjectilePhysicsSim();
    ProjectilePhysicsSim aeroSim = new ProjectilePhysicsSim().withAerodynamicCorrections(aero);

    Translation3d launch = new Translation3d(0.0, 0.0, 0.8);
    Translation3d vel = new Translation3d(9.0, 0.0, 7.0);
    // Zero spin isolates buoyancy + added mass
    Optional<ProjectilePhysicsSim.LandingObservation> baseLand =
        baselineSim.simulateToLanding(fuel, launch, vel, new Translation3d(), 0.2, 0.001, 5.0);
    Optional<ProjectilePhysicsSim.LandingObservation> aeroLand =
        aeroSim.simulateToLanding(fuel, launch, vel, new Translation3d(), 0.2, 0.001, 5.0);

    assertTrue(baseLand.isPresent());
    assertTrue(aeroLand.isPresent());
    // Buoyancy + added mass extends horizontal carry measurably over baseline
    assertTrue(aeroLand.get().horizontalRangeMeters() > baseLand.get().horizontalRangeMeters());
  }

  @Test
  void spinDecayAttenuatesLateFlightMagnusLift() {
    ProjectileSpec fuel = ProjectileSpec.frcFuel2026();
    AerodynamicCorrectionConfig noDecay = new AerodynamicCorrectionConfig(true, 0.5, 0.0, 0.001);
    AerodynamicCorrectionConfig withDecay = new AerodynamicCorrectionConfig(true, 0.5, 0.8, 0.001);

    ProjectilePhysicsSim simNoDecay =
        new ProjectilePhysicsSim().withAerodynamicCorrections(noDecay);
    ProjectilePhysicsSim simWithDecay =
        new ProjectilePhysicsSim().withAerodynamicCorrections(withDecay);

    Translation3d launch = new Translation3d(0.0, 0.0, 0.8);
    Translation3d vel = new Translation3d(10.0, 0.0, 6.5);
    Translation3d backspin = new Translation3d(0.0, -60.0, 0.0);

    simWithDecay.launch(fuel, launch, vel, backspin);
    simWithDecay.step(0.5);
    double decayedSpinY = Math.abs(simWithDecay.getProjectiles().get(0).spinRadPerSec().getY());
    assertEquals(60.0 * Math.exp(-0.4), decayedSpinY, 0.05);

    Optional<ProjectilePhysicsSim.LandingObservation> landNoDecay =
        simNoDecay.simulateToLanding(fuel, launch, vel, backspin, 0.2, 0.001, 5.0);
    Optional<ProjectilePhysicsSim.LandingObservation> landWithDecay =
        simWithDecay.simulateToLanding(fuel, launch, vel, backspin, 0.2, 0.001, 5.0);

    assertTrue(landNoDecay.isPresent());
    assertTrue(landWithDecay.isPresent());
    // Decaying backspin produces less late-flight upward Magnus lift -> shorter range
    assertTrue(
        landWithDecay.get().horizontalRangeMeters() < landNoDecay.get().horizontalRangeMeters());
  }

  @Test
  void twoObservableCalibrationRecoversAerodynamicCoefficientsWithHoldoutValidation() {
    ProjectileSpec truthSpec =
        new ProjectileSpec(
            "FuelTruth",
            0.22,
            0.075,
            0.54,
            ProjectileSpec.STANDARD_AIR_DENSITY_KG_PER_M3,
            0.22,
            ProjectileSpec.STANDARD_GRAVITY_MPS2,
            0.35);
    AerodynamicCorrectionConfig truthAero = new AerodynamicCorrectionConfig(true, 0.5, 0.45, 0.001);
    ProjectilePhysicsSim truthSim =
        new ProjectilePhysicsSim().withAerodynamicCorrections(truthAero);

    double launchZ = 0.75;
    double targetZ = 1.80;
    List<BallisticCalibrationFitter.CalibrationShot> shots = new ArrayList<>();
    SplittableRandom rng = new SplittableRandom(20270605L);

    // Generate 20 varied-hood, varied-speed empirical calibration shots
    for (int i = 0; i < 20; i++) {
      double speed = 10.5 + (i % 5) * 0.9;
      double pitchDeg = 46.0 + (i / 5) * 4.5;
      double pitchRad = Math.toRadians(pitchDeg);
      double spin = 35.0 + (i % 4) * 12.0;
      Translation3d vel =
          new Translation3d(speed * Math.cos(pitchRad), 0.0, speed * Math.sin(pitchRad));
      Optional<ProjectilePhysicsSim.LandingObservation> obs =
          truthSim.simulateToLanding(
              truthSpec,
              new Translation3d(0.0, 0.0, launchZ),
              vel,
              new Translation3d(0.0, -spin, 0.0),
              targetZ,
              0.001,
              5.0);
      assertTrue(obs.isPresent(), "Synthetic shot " + i + " should reach target height");
      // Tiny realistic measurement noise (+/- 3 mm range, +/- 0.1 deg entry angle)
      double noisyRange = obs.get().horizontalRangeMeters() + (rng.nextDouble() - 0.5) * 0.006;
      double noisyAngle =
          obs.get().entryAngleRad() + Math.toRadians((rng.nextDouble() - 0.5) * 0.2);
      shots.add(
          new BallisticCalibrationFitter.CalibrationShot(
              speed, pitchRad, spin, noisyRange, noisyAngle, 1.0));
    }

    // Start from intentionally perturbed baseline guesses
    ProjectileSpec initialGuess = ProjectileSpec.frcFuel2026();
    AerodynamicCorrectionConfig initialAero =
        new AerodynamicCorrectionConfig(true, 0.5, 0.15, 0.001);

    BallisticCalibrationFitter.CalibrationResult result =
        BallisticCalibrationFitter.fit(
            initialGuess, initialAero, launchZ, targetZ, shots, 0.25, 4242L);

    assertEquals(15, result.trainCount());
    assertEquals(5, result.validationCount());
    assertTrue(
        result.validationRangeRmseMeters() < 0.025,
        "Held-out validation range RMSE too high: " + result.validationRangeRmseMeters());
    assertTrue(
        result.validationEntryAngleRmseDeg() < 1.0,
        "Held-out validation entry-angle RMSE too high: " + result.validationEntryAngleRmseDeg());
    assertEquals(truthSpec.dragCoefficient(), result.fittedSpec().dragCoefficient(), 0.03);
    assertEquals(
        truthSpec.magnusLiftCoefficient(), result.fittedSpec().magnusLiftCoefficient(), 0.04);
  }

  @Test
  void seededPropertyFuzzAllianceFlipSymmetryAndDistancePreservation() {
    long seed = 2027060501L;
    SplittableRandom rng = new SplittableRandom(seed);

    for (int trial = 0; trial < 500; trial++) {
      Pose3d a =
          new Pose3d(
              rng.nextDouble(0.0, FIELD_LENGTH_M),
              rng.nextDouble(0.0, FIELD_WIDTH_M),
              rng.nextDouble(0.0, 3.0),
              new Rotation3d(0.0, 0.0, rng.nextDouble(-Math.PI, Math.PI)));
      Pose3d b =
          new Pose3d(
              rng.nextDouble(0.0, FIELD_LENGTH_M),
              rng.nextDouble(0.0, FIELD_WIDTH_M),
              rng.nextDouble(0.0, 3.0),
              new Rotation3d(0.0, 0.0, rng.nextDouble(-Math.PI, Math.PI)));

      // Rotational 180-deg alliance symmetry (REBUILT / CRESCENDO / REEFSCAPE)
      Pose3d aFlip = flipRotational(a);
      Pose3d aDoubleFlip = flipRotational(aFlip);
      Pose3d bFlip = flipRotational(b);

      assertEquals(a.getX(), aDoubleFlip.getX(), 1e-9, "Seed " + seed + " trial " + trial);
      assertEquals(a.getY(), aDoubleFlip.getY(), 1e-9, "Seed " + seed + " trial " + trial);
      assertEquals(a.getZ(), aDoubleFlip.getZ(), 1e-9, "Seed " + seed + " trial " + trial);
      assertEquals(
          a.getTranslation().getDistance(b.getTranslation()),
          aFlip.getTranslation().getDistance(bFlip.getTranslation()),
          1e-9,
          "Pairwise distance changed under flip, seed " + seed + " trial " + trial);
    }
  }

  @Test
  void seededPropertyFuzzRk4DeterminismAndFiniteness() {
    long seed = 2027060502L;
    SplittableRandom rng = new SplittableRandom(seed);
    ProjectileSpec fuel = ProjectileSpec.frcFuel2026();
    AerodynamicCorrectionConfig aero = AerodynamicCorrectionConfig.frcFuelHighFidelity();

    for (int trial = 0; trial < 200; trial++) {
      double speed = rng.nextDouble(4.0, 20.0);
      double pitch = Math.toRadians(rng.nextDouble(15.0, 80.0));
      double yaw = rng.nextDouble(-Math.PI, Math.PI);
      Translation3d vel =
          new Translation3d(
              speed * Math.cos(pitch) * Math.cos(yaw),
              speed * Math.cos(pitch) * Math.sin(yaw),
              speed * Math.sin(pitch));
      Translation3d spin =
          new Translation3d(
              rng.nextDouble(-80.0, 80.0),
              rng.nextDouble(-80.0, 80.0),
              rng.nextDouble(-80.0, 80.0));
      Translation3d launch = new Translation3d(1.0, 2.0, 0.7);

      ProjectilePhysicsSim simA = new ProjectilePhysicsSim().withAerodynamicCorrections(aero);
      ProjectilePhysicsSim simB = new ProjectilePhysicsSim().withAerodynamicCorrections(aero);

      List<Pose3d> arcA = simA.predictArc(fuel, launch, vel, spin, 0.02, 40);
      List<Pose3d> arcB = simB.predictArc(fuel, launch, vel, spin, 0.02, 40);

      assertEquals(arcA.size(), arcB.size(), "Seed " + seed + " trial " + trial);
      for (int k = 0; k < arcA.size(); k++) {
        Pose3d pa = arcA.get(k);
        Pose3d pb = arcB.get(k);
        assertTrue(
            Double.isFinite(pa.getX()) && Double.isFinite(pa.getY()) && Double.isFinite(pa.getZ()));
        assertEquals(pa.getX(), pb.getX(), 0.0, "Bit-exact replay mismatch X");
        assertEquals(pa.getY(), pb.getY(), 0.0, "Bit-exact replay mismatch Y");
        assertEquals(pa.getZ(), pb.getZ(), 0.0, "Bit-exact replay mismatch Z");
      }
    }
  }

  private static Pose3d flipRotational(Pose3d p) {
    return new Pose3d(
        FIELD_LENGTH_M - p.getX(),
        FIELD_WIDTH_M - p.getY(),
        p.getZ(),
        new Rotation3d(
            p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ() + Math.PI));
  }
}
