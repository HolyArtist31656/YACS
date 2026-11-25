// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.physics;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;

import java.util.Objects;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import yacsim.core.yams.MotionAxis;

/**
 * Immutable mass, center of mass, and principal moments of inertia for a CAD component, with
 * parallel-axis theorem helpers for seeding YAMS physics simulations.
 *
 * @param volumeCubicMeters Volume in m³.
 * @param massKilograms Mass in kg.
 * @param centerOfMassMeters Center of mass in meters.
 * @param ixxKgM2 Moment of inertia about X through center of mass in kg·m².
 * @param iyyKgM2 Moment of inertia about Y through center of mass in kg·m².
 * @param izzKgM2 Moment of inertia about Z through center of mass in kg·m².
 */
public record CadMassProperties(
    double volumeCubicMeters,
    double massKilograms,
    Translation3d centerOfMassMeters,
    double ixxKgM2,
    double iyyKgM2,
    double izzKgM2) {

  /**
   * Returns a copy scaled to {@code targetMassKg}, scaling principal inertia values proportionally.
   *
   * @param targetMassKg Positive target mass in kilograms.
   * @return Scaled {@link CadMassProperties}.
   */
  public CadMassProperties withTotalMassKg(double targetMassKg) {
    if (targetMassKg <= 0.0) {
      throw new IllegalArgumentException("Target mass must be positive.");
    }
    if (massKilograms <= 0.0) {
      return new CadMassProperties(
          volumeCubicMeters, targetMassKg, centerOfMassMeters, ixxKgM2, iyyKgM2, izzKgM2);
    }
    double r = targetMassKg / massKilograms;
    return new CadMassProperties(
        volumeCubicMeters, targetMassKg, centerOfMassMeters, ixxKgM2 * r, iyyKgM2 * r, izzKgM2 * r);
  }

  /**
   * Returns this component's total mass as a WPILib {@link Mass} measure (suitable for YAMS {@code
   * ElevatorConfig.withCarriageWeight(Mass)}).
   *
   * @return Total mass as a WPILib {@link Mass}.
   */
  public Mass mass() {
    return Kilograms.of(massKilograms);
  }

  /**
   * Computes the moment of inertia about an arbitrary pivot point along {@code rotationAxis} using
   * the parallel-axis theorem ({@code I = I_cm + m * d_perp^2}).
   *
   * @param pivotMeters Pivot coordinates in the same local frame as {@link #centerOfMassMeters()}.
   * @param rotationAxis Axis of rotation ({@link MotionAxis#X}, {@link MotionAxis#Y}, or {@link
   *     MotionAxis#Z}).
   * @return Moment of inertia about the pivot in kg·m².
   */
  public double momentOfInertiaAboutPivotKgM2(
      Translation3d pivotMeters, MotionAxis rotationAxis) {
    Objects.requireNonNull(pivotMeters, "pivotMeters");
    Objects.requireNonNull(rotationAxis, "rotationAxis");
    double dx = centerOfMassMeters.getX() - pivotMeters.getX();
    double dy = centerOfMassMeters.getY() - pivotMeters.getY();
    double dz = centerOfMassMeters.getZ() - pivotMeters.getZ();
    return switch (rotationAxis) {
      case X -> ixxKgM2 + massKilograms * (dy * dy + dz * dz);
      case Y -> iyyKgM2 + massKilograms * (dx * dx + dz * dz);
      case Z -> izzKgM2 + massKilograms * (dx * dx + dy * dy);
    };
  }

  /**
   * Computes the moment of inertia about an arbitrary pivot point as a WPILib {@link
   * MomentOfInertia} measure (suitable for YAMS {@code
   * SmartMotorControllerConfig.withMomentOfInertia(MomentOfInertia)}).
   *
   * @param pivotMeters Pivot coordinates in the same local frame as {@link #centerOfMassMeters()}.
   * @param rotationAxis Axis of rotation.
   * @return Moment of inertia about the pivot as a WPILib {@link MomentOfInertia}.
   */
  public MomentOfInertia momentOfInertiaAboutPivot(
      Translation3d pivotMeters, MotionAxis rotationAxis) {
    return KilogramSquareMeters.of(momentOfInertiaAboutPivotKgM2(pivotMeters, rotationAxis));
  }

  /**
   * Computes uniform-density rectangular prism mass properties from a bounding box.
   *
   * @param box Local bounding box in meters.
   * @param densityKgPerM3 Material density in kg/m³.
   * @return Estimated {@link CadMassProperties}.
   */
  public static CadMassProperties fromBoundingBox(BoundingBox3d box, double densityKgPerM3) {
    double lx = Math.max(1e-6, box.sizeX());
    double ly = Math.max(1e-6, box.sizeY());
    double lz = Math.max(1e-6, box.sizeZ());
    double vol = lx * ly * lz;
    double m = vol * densityKgPerM3;
    return new CadMassProperties(
        vol,
        m,
        box.center(),
        (m / 12.0) * (ly * ly + lz * lz),
        (m / 12.0) * (lx * lx + lz * lz),
        (m / 12.0) * (lx * lx + ly * ly));
  }
}
