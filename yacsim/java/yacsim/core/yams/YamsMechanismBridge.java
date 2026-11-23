// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.yams;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import yacsim.core.cad.MeshMetadata;

/**
 * Pose-supplier composition helpers for bridging YAMS mechanism kinematics (angles, linear
 * distances, and parent–child chains) to YACSim articulated CAD components without depending on a
 * specific WPILib command framework.
 */
public final class YamsMechanismBridge {
  private YamsMechanismBridge() {}

  /**
   * Applies a fixed CAD mounting offset transform to a mechanism pose supplier.
   *
   * @param yamsPoseSupplier Supplier returning the mechanism's simulated {@link Pose3d}.
   * @param cadMountOffset Fixed transform from mechanism origin to CAD mesh origin.
   * @return Composed {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromPoseSupplier(
      Supplier<Pose3d> yamsPoseSupplier, Transform3d cadMountOffset) {
    Objects.requireNonNull(yamsPoseSupplier, "yamsPoseSupplier");
    Objects.requireNonNull(cadMountOffset, "cadMountOffset");
    return () -> yamsPoseSupplier.get().transformBy(cadMountOffset);
  }

  /**
   * Chains a moving parent mechanism's pose with a child mechanism's local pose (for example, a
   * wrist mounted on an elevator carriage).
   *
   * @param parentPoseSupplier Supplier returning the parent mechanism's {@link Pose3d}.
   * @param childLocalPoseSupplier Supplier returning the child's local {@link Pose3d} relative to
   *     the parent.
   * @return Composed robot-relative {@link Pose3d} supplier for the child.
   */
  public static Supplier<Pose3d> chainParentChild(
      Supplier<Pose3d> parentPoseSupplier, Supplier<Pose3d> childLocalPoseSupplier) {
    Objects.requireNonNull(parentPoseSupplier, "parentPoseSupplier");
    Objects.requireNonNull(childLocalPoseSupplier, "childLocalPoseSupplier");
    return () -> {
      Pose3d parent = parentPoseSupplier.get();
      Pose3d child = childLocalPoseSupplier.get();
      return parent.transformBy(new Transform3d(child.getTranslation(), child.getRotation()));
    };
  }

  /**
   * Builds a {@link Pose3d} supplier for a rotational mechanism (arm, hood, turret, or pivot) from
   * a WPILib {@link Angle} supplier.
   *
   * @param angleSupplier Supplier returning the current mechanism {@link Angle}.
   * @param pivotMeters Pivot location in robot-frame meters.
   * @param rotationAxis Axis of rotation ({@link MotionAxis#Y} for pitch arms, {@link MotionAxis#Z}
   *     for turrets/pivots).
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromAngleSupplier(
      Supplier<Angle> angleSupplier, Translation3d pivotMeters, MotionAxis rotationAxis) {
    return fromAngleSupplier(angleSupplier, pivotMeters, rotationAxis, false, new Transform3d());
  }

  /**
   * Builds a {@link Pose3d} supplier for a rotational mechanism with optional direction inversion.
   *
   * @param angleSupplier Supplier returning the current mechanism {@link Angle}.
   * @param pivotMeters Pivot location in robot-frame meters.
   * @param rotationAxis Axis of rotation.
   * @param inverted {@code true} to negate the angle (useful when CAD rotation direction is
   *     opposite WPILib's right-hand rule).
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromAngleSupplier(
      Supplier<Angle> angleSupplier,
      Translation3d pivotMeters,
      MotionAxis rotationAxis,
      boolean inverted) {
    return fromAngleSupplier(angleSupplier, pivotMeters, rotationAxis, inverted, new Transform3d());
  }

  /**
   * Builds a {@link Pose3d} supplier for a rotational mechanism with optional inversion and a fixed
   * local CAD zero offset transform.
   *
   * @param angleSupplier Supplier returning the current mechanism {@link Angle}.
   * @param pivotMeters Pivot location in robot-frame meters.
   * @param rotationAxis Axis of rotation.
   * @param inverted {@code true} to negate the angle.
   * @param zeroOffset Fixed transform applied after rotating around the pivot.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromAngleSupplier(
      Supplier<Angle> angleSupplier,
      Translation3d pivotMeters,
      MotionAxis rotationAxis,
      boolean inverted,
      Transform3d zeroOffset) {
    Objects.requireNonNull(angleSupplier, "angleSupplier");
    Objects.requireNonNull(pivotMeters, "pivotMeters");
    Objects.requireNonNull(rotationAxis, "rotationAxis");
    Objects.requireNonNull(zeroOffset, "zeroOffset");
    double sign = inverted ? -1.0 : 1.0;
    return () -> {
      Angle angle = angleSupplier.get();
      double rad = angle != null ? sign * angle.in(Radians) : 0.0;
      return new Pose3d(pivotMeters, rotationAxis.toRotation3d(rad)).transformBy(zeroOffset);
    };
  }

  /**
   * Builds a {@link Pose3d} supplier for a linear mechanism (elevator, climber, or linear slide)
   * from a WPILib {@link Distance} supplier.
   *
   * @param distanceSupplier Supplier returning the current extension {@link Distance}.
   * @param baseMeters Base origin in robot-frame meters when extension is zero.
   * @param translationAxis Axis of linear travel (typically {@link MotionAxis#Z} for elevators).
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromDistanceSupplier(
      Supplier<Distance> distanceSupplier, Translation3d baseMeters, MotionAxis translationAxis) {
    return fromDistanceSupplier(
        distanceSupplier, baseMeters, translationAxis, false, new Transform3d());
  }

  /**
   * Builds a {@link Pose3d} supplier for a linear mechanism with optional direction inversion.
   *
   * @param distanceSupplier Supplier returning the current extension {@link Distance}.
   * @param baseMeters Base origin in robot-frame meters.
   * @param translationAxis Axis of linear travel.
   * @param inverted {@code true} to negate the displacement direction.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromDistanceSupplier(
      Supplier<Distance> distanceSupplier,
      Translation3d baseMeters,
      MotionAxis translationAxis,
      boolean inverted) {
    return fromDistanceSupplier(
        distanceSupplier, baseMeters, translationAxis, inverted, new Transform3d());
  }

  /**
   * Builds a {@link Pose3d} supplier for a linear mechanism with optional direction inversion and a
   * fixed local CAD zero offset transform.
   *
   * @param distanceSupplier Supplier returning the current extension {@link Distance}.
   * @param baseMeters Base origin in robot-frame meters.
   * @param translationAxis Axis of linear travel.
   * @param inverted {@code true} to negate the displacement direction.
   * @param zeroOffset Fixed transform applied after translation (for example, an inclined elevator
   *     angle).
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromDistanceSupplier(
      Supplier<Distance> distanceSupplier,
      Translation3d baseMeters,
      MotionAxis translationAxis,
      boolean inverted,
      Transform3d zeroOffset) {
    Objects.requireNonNull(distanceSupplier, "distanceSupplier");
    Objects.requireNonNull(baseMeters, "baseMeters");
    Objects.requireNonNull(translationAxis, "translationAxis");
    Objects.requireNonNull(zeroOffset, "zeroOffset");
    double sign = inverted ? -1.0 : 1.0;
    return () -> {
      Distance dist = distanceSupplier.get();
      double meters = dist != null ? sign * dist.in(Meters) : 0.0;
      Translation3d pos = baseMeters.plus(translationAxis.toTranslation3d(meters));
      return new Pose3d(pos, new Rotation3d()).transformBy(zeroOffset);
    };
  }

  /**
   * Estimates a CAD component's total mass as a WPILib {@link Mass} measure for seeding YAMS
   * configurations (for example {@code ElevatorConfig.withCarriageWeight(Mass)}).
   *
   * @param metadata Parsed {@link MeshMetadata} for the component.
   * @return Estimated or overridden {@link Mass}.
   */
  public static Mass estimateMass(MeshMetadata metadata) {
    Objects.requireNonNull(metadata, "metadata");
    return metadata.massProperties().mass();
  }

  /**
   * Estimates a rotational CAD component's moment of inertia about {@code pivotMeters} along {@code
   * rotationAxis} in kg·m² using the parallel-axis theorem.
   *
   * @param metadata Parsed {@link MeshMetadata} for the arm/hood/turret mesh.
   * @param pivotMeters Pivot coordinates in the mesh's local frame in meters.
   * @param rotationAxis Axis of rotation.
   * @return Moment of inertia in kg·m².
   */
  public static double estimateArmMoiKgM2(
      MeshMetadata metadata, Translation3d pivotMeters, MotionAxis rotationAxis) {
    Objects.requireNonNull(metadata, "metadata");
    return metadata.massProperties().momentOfInertiaAboutPivotKgM2(pivotMeters, rotationAxis);
  }

  /**
   * Estimates a rotational CAD component's moment of inertia about {@code pivotMeters} as a WPILib
   * {@link MomentOfInertia} measure for direct use with YAMS {@code
   * SmartMotorControllerConfig.withMomentOfInertia(MomentOfInertia)}.
   *
   * @param metadata Parsed {@link MeshMetadata} for the arm/hood/turret mesh.
   * @param pivotMeters Pivot coordinates in the mesh's local frame in meters.
   * @param rotationAxis Axis of rotation.
   * @return Moment of inertia as a WPILib {@link MomentOfInertia}.
   */
  public static MomentOfInertia estimateArmMoi(
      MeshMetadata metadata, Translation3d pivotMeters, MotionAxis rotationAxis) {
    Objects.requireNonNull(metadata, "metadata");
    return metadata.massProperties().momentOfInertiaAboutPivot(pivotMeters, rotationAxis);
  }
}
