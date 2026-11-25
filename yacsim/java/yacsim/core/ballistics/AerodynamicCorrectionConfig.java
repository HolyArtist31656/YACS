// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.ballistics;

/**
 * Optional high-fidelity aerodynamic corrections for {@link ProjectilePhysicsSim} RK4 integration
 * targeting lightweight hollow FRC projectiles (such as 2026 REBUILT foam Fuel).
 *
 * <p>Models three second-order physical effects that contribute roughly 5% range and entry-angle
 * error on lightweight spheres when omitted:
 *
 * <ul>
 *   <li><b>Air buoyancy:</b> upward Archimedes force {@code rho_air * (4/3 * pi * r^3) * g} (~3.3%
 *       weight reduction for hollow foam Fuel).
 *   <li><b>Added mass:</b> fluid inertia accelerated with the sphere {@code m_eff = m + C_m *
 *       rho_air * V} ({@code C_m = 0.5} for a sphere, ~1.7% effective inertia increase).
 *   <li><b>Aerodynamic spin decay:</b> exponential angular spin damping {@code omega(t + dt) =
 *       omega(t) * exp(-lambda * dt)} reducing late-flight Magnus lift.
 *   <li><b>Fine RK4 sub-stepping:</b> internal sub-step ceiling (default {@code 0.001 s}) for high
 *       numerical accuracy under strong Magnus and drag coupling.
 * </ul>
 *
 * @param buoyancyEnabled {@code true} to include upward air buoyancy force.
 * @param addedMassCoefficient Dimensionless fluid added-mass coefficient ({@code 0.5} for a sphere,
 *     {@code 0.0} to disable).
 * @param spinDecayRatePerSec Exponential spin decay rate in 1/s ({@code >= 0.0}).
 * @param maxInternalSubstepSeconds Maximum RK4 internal sub-step in seconds ({@code 0.0} disables
 *     sub-stepping and uses the caller's {@code dt}).
 */
public record AerodynamicCorrectionConfig(
    boolean buoyancyEnabled,
    double addedMassCoefficient,
    double spinDecayRatePerSec,
    double maxInternalSubstepSeconds) {

  /** Compact constructor validating non-negative finite parameters. */
  public AerodynamicCorrectionConfig {
    if (addedMassCoefficient < 0.0 || !Double.isFinite(addedMassCoefficient)) {
      throw new IllegalArgumentException("addedMassCoefficient must be non-negative and finite.");
    }
    if (spinDecayRatePerSec < 0.0 || !Double.isFinite(spinDecayRatePerSec)) {
      throw new IllegalArgumentException("spinDecayRatePerSec must be non-negative and finite.");
    }
    if (maxInternalSubstepSeconds < 0.0 || !Double.isFinite(maxInternalSubstepSeconds)) {
      throw new IllegalArgumentException(
          "maxInternalSubstepSeconds must be non-negative and finite.");
    }
  }

  /**
   * Disables all second-order aerodynamic corrections and sub-stepping (bit-exact match with
   * v2027.5.0/v2027.6.0 baseline RK4).
   *
   * @return Disabled configuration.
   */
  public static AerodynamicCorrectionConfig disabled() {
    return new AerodynamicCorrectionConfig(false, 0.0, 0.0, 0.0);
  }

  /**
   * High-fidelity preset for hollow spherical FRC Fuel: air buoyancy enabled, potential-flow sphere
   * added mass ({@code 0.5}), {@code 0.35 / s} spin decay, and {@code 0.001 s} RK4 sub-stepping.
   *
   * @return High-fidelity Fuel configuration.
   */
  public static AerodynamicCorrectionConfig frcFuelHighFidelity() {
    return new AerodynamicCorrectionConfig(true, 0.5, 0.35, 0.001);
  }

  /**
   * Returns a copy with an updated spin decay rate.
   *
   * @param newSpinDecayRatePerSec New non-negative spin decay rate in 1/s.
   * @return Updated configuration.
   */
  public AerodynamicCorrectionConfig withSpinDecayRatePerSec(double newSpinDecayRatePerSec) {
    return new AerodynamicCorrectionConfig(
        buoyancyEnabled, addedMassCoefficient, newSpinDecayRatePerSec, maxInternalSubstepSeconds);
  }

  /**
   * Computes displaced spherical air volume {@code (4/3) * pi * r^3} in cubic meters.
   *
   * @param spec Projectile specification.
   * @return Displaced volume in cubic meters.
   */
  public static double displacedVolumeM3(ProjectileSpec spec) {
    double r = spec.radiusMeters();
    return (4.0 / 3.0) * Math.PI * r * r * r;
  }

  /**
   * Computes effective inertial mass including fluid added mass {@code m + C_m * rho_air * V}.
   *
   * @param spec Projectile specification.
   * @return Effective inertial mass in kilograms.
   */
  public double effectiveMassKg(ProjectileSpec spec) {
    if (addedMassCoefficient <= 0.0 || spec.airDensityKgPerM3() <= 0.0) {
      return spec.massKg();
    }
    return spec.massKg()
        + addedMassCoefficient * spec.airDensityKgPerM3() * displacedVolumeM3(spec);
  }

  /**
   * Computes upward buoyancy force {@code rho_air * V * g} in Newtons.
   *
   * @param spec Projectile specification.
   * @return Upward buoyancy force in Newtons.
   */
  public double buoyancyForceNewtons(ProjectileSpec spec) {
    if (!buoyancyEnabled || spec.airDensityKgPerM3() <= 0.0) {
      return 0.0;
    }
    return spec.airDensityKgPerM3() * displacedVolumeM3(spec) * spec.gravityMps2();
  }
}
