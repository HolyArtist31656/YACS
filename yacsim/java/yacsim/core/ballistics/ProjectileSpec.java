// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.ballistics;

import java.util.Objects;

/**
 * Physical specification of an FRC airborne game piece for RK4 3D ballistic simulation with
 * gravity, quadratic aerodynamic drag, and Magnus spin lift.
 *
 * @param name Display name of the game piece (for example {@code "Fuel"} or {@code "Note"}).
 * @param massKg Mass in kilograms (must be positive).
 * @param radiusMeters Effective collision/aerodynamic radius in meters (must be positive).
 * @param dragCoefficient Dimensionless drag coefficient ({@code Cd >= 0}, ~0.47 for a smooth
 *     sphere).
 * @param airDensityKgPerM3 Ambient air density in kg/m³ (sea-level standard {@code 1.225}).
 * @param magnusLiftCoefficient Dimensionless Magnus spin-lift scaling factor ({@code >= 0}).
 * @param gravityMps2 Downward gravitational acceleration magnitude in m/s² (standard {@code 9.80665}).
 * @param restitution Floor bounce restitution in {@code [0.0, 1.0]} ({@code 0.0} stops on floor).
 */
public record ProjectileSpec(
    String name,
    double massKg,
    double radiusMeters,
    double dragCoefficient,
    double airDensityKgPerM3,
    double magnusLiftCoefficient,
    double gravityMps2,
    double restitution) {

  /** Standard sea-level air density (kg/m³). */
  public static final double STANDARD_AIR_DENSITY_KG_PER_M3 = 1.225;

  /** Standard Earth gravity (m/s²). */
  public static final double STANDARD_GRAVITY_MPS2 = 9.80665;

  /** Compact constructor validating positive mass/radius and non-negative coefficients. */
  public ProjectileSpec {
    Objects.requireNonNull(name, "name");
    if (name.isBlank()) {
      throw new IllegalArgumentException("Projectile name must not be blank.");
    }
    if (massKg <= 0.0 || !Double.isFinite(massKg)) {
      throw new IllegalArgumentException("massKg must be positive and finite.");
    }
    if (radiusMeters <= 0.0 || !Double.isFinite(radiusMeters)) {
      throw new IllegalArgumentException("radiusMeters must be positive and finite.");
    }
    if (dragCoefficient < 0.0 || !Double.isFinite(dragCoefficient)) {
      throw new IllegalArgumentException("dragCoefficient must be non-negative and finite.");
    }
    if (airDensityKgPerM3 < 0.0 || !Double.isFinite(airDensityKgPerM3)) {
      throw new IllegalArgumentException("airDensityKgPerM3 must be non-negative and finite.");
    }
    if (magnusLiftCoefficient < 0.0 || !Double.isFinite(magnusLiftCoefficient)) {
      throw new IllegalArgumentException("magnusLiftCoefficient must be non-negative and finite.");
    }
    if (gravityMps2 < 0.0 || !Double.isFinite(gravityMps2)) {
      throw new IllegalArgumentException("gravityMps2 must be non-negative and finite.");
    }
    if (restitution < 0.0 || restitution > 1.0 || !Double.isFinite(restitution)) {
      throw new IllegalArgumentException("restitution must be in [0.0, 1.0].");
    }
  }

  /** @return Cross-sectional area ({@code pi * r^2}) in square meters. */
  public double crossSectionAreaM2() {
    return Math.PI * radiusMeters * radiusMeters;
  }

  /**
   * Preset for a 2026 REBUILT-style 5.91-inch (0.15 m diameter) foam spherical Fuel projectile.
   *
   * @return {@link ProjectileSpec} for spherical foam Fuel.
   */
  public static ProjectileSpec frcFuel2026() {
    return new ProjectileSpec(
        "Fuel",
        0.22,
        0.075,
        0.47,
        STANDARD_AIR_DENSITY_KG_PER_M3,
        0.15,
        STANDARD_GRAVITY_MPS2,
        0.35);
  }

  /**
   * Preset for a 2024 CRESCENDO foam torus Note.
   *
   * @return {@link ProjectileSpec} for a Note.
   */
  public static ProjectileSpec frcNote2024() {
    return new ProjectileSpec(
        "Note",
        0.235,
        0.178,
        0.55,
        STANDARD_AIR_DENSITY_KG_PER_M3,
        0.05,
        STANDARD_GRAVITY_MPS2,
        0.15);
  }

  /**
   * Preset for a 2025 REEFSCAPE PVC Coral tube.
   *
   * @return {@link ProjectileSpec} for Coral.
   */
  public static ProjectileSpec frcCoral2025() {
    return new ProjectileSpec(
        "Coral",
        0.65,
        0.057,
        0.82,
        STANDARD_AIR_DENSITY_KG_PER_M3,
        0.0,
        STANDARD_GRAVITY_MPS2,
        0.20);
  }

  /**
   * Vacuum ballistic preset (zero drag and zero Magnus lift) for analytical comparison tests.
   *
   * @param name Display name.
   * @param massKg Mass in kilograms.
   * @param radiusMeters Radius in meters.
   * @return Zero-drag {@link ProjectileSpec}.
   */
  public static ProjectileSpec vacuum(String name, double massKg, double radiusMeters) {
    return new ProjectileSpec(
        name, massKg, radiusMeters, 0.0, 0.0, 0.0, STANDARD_GRAVITY_MPS2, 0.0);
  }
}
