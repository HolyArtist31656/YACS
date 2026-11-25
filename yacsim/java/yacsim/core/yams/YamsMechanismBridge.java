// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.yams;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
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

  /**
   * Builds per-stage {@link Pose3d} suppliers for a multi-stage cascading elevator where stage
   * {@code k} ({@code 1..stageCount}) travels {@code (k / stageCount)} of total elevator height.
   *
   * @param totalHeightSupplier Supplier returning total carriage height as a {@link Distance}.
   * @param stageCount Number of moving stages (must be at least 1).
   * @param baseMeters Base origin in robot-frame meters when height is zero.
   * @param translationAxis Axis of linear travel (typically {@link MotionAxis#Z}).
   * @return Unmodifiable list of {@code stageCount} {@link Pose3d} suppliers from stage 1 to carriage.
   */
  public static List<Supplier<Pose3d>> fromCascadingElevator(
      Supplier<Distance> totalHeightSupplier,
      int stageCount,
      Translation3d baseMeters,
      MotionAxis translationAxis) {
    Objects.requireNonNull(totalHeightSupplier, "totalHeightSupplier");
    Objects.requireNonNull(baseMeters, "baseMeters");
    Objects.requireNonNull(translationAxis, "translationAxis");
    if (stageCount < 1) {
      throw new IllegalArgumentException("stageCount must be at least 1.");
    }
    List<Supplier<Pose3d>> stages = new ArrayList<>(stageCount);
    for (int i = 1; i <= stageCount; i++) {
      final double fraction = (double) i / stageCount;
      stages.add(
          () -> {
            Distance d = totalHeightSupplier.get();
            double meters = d != null ? d.in(Meters) * fraction : 0.0;
            return new Pose3d(
                baseMeters.plus(translationAxis.toTranslation3d(meters)), new Rotation3d());
          });
    }
    return Collections.unmodifiableList(stages);
  }

  /**
   * Builds a {@link Pose3d} supplier for a level-riding 4-bar linkage end-effector that translates
   * along a circular arc of radius {@code armLengthMeters} around {@code pivotMeters} while keeping
   * zero rotation in the robot frame.
   *
   * @param armAngleSupplier Supplier returning the driving arm {@link Angle}.
   * @param pivotMeters Pivot location in robot-frame meters.
   * @param armLengthMeters Positive linkage arm length in meters.
   * @param rotationAxis Axis of arm rotation (typically {@link MotionAxis#Y} for XZ-plane arms).
   * @return Robot-relative {@link Pose3d} supplier for the level end-effector.
   */
  public static Supplier<Pose3d> fromFourBar(
      Supplier<Angle> armAngleSupplier,
      Translation3d pivotMeters,
      double armLengthMeters,
      MotionAxis rotationAxis) {
    return fromFourBar(armAngleSupplier, pivotMeters, armLengthMeters, rotationAxis, false);
  }

  /**
   * Builds a {@link Pose3d} supplier for a level-riding 4-bar linkage end-effector with optional
   * angle inversion.
   *
   * @param armAngleSupplier Supplier returning the driving arm {@link Angle}.
   * @param pivotMeters Pivot location in robot-frame meters.
   * @param armLengthMeters Positive linkage arm length in meters.
   * @param rotationAxis Axis of arm rotation.
   * @param inverted {@code true} to negate the driving angle.
   * @return Robot-relative {@link Pose3d} supplier for the level end-effector.
   */
  public static Supplier<Pose3d> fromFourBar(
      Supplier<Angle> armAngleSupplier,
      Translation3d pivotMeters,
      double armLengthMeters,
      MotionAxis rotationAxis,
      boolean inverted) {
    Objects.requireNonNull(armAngleSupplier, "armAngleSupplier");
    Objects.requireNonNull(pivotMeters, "pivotMeters");
    Objects.requireNonNull(rotationAxis, "rotationAxis");
    if (armLengthMeters <= 0.0) {
      throw new IllegalArgumentException("armLengthMeters must be positive.");
    }
    double sign = inverted ? -1.0 : 1.0;
    return () -> {
      Angle angle = armAngleSupplier.get();
      double rad = angle != null ? sign * angle.in(Radians) : 0.0;
      Translation3d armTipOffset =
          new Translation3d(armLengthMeters, 0.0, 0.0).rotateBy(rotationAxis.toRotation3d(rad));
      return new Pose3d(pivotMeters.plus(armTipOffset), new Rotation3d());
    };
  }

  /**
   * Builds a {@link Pose3d} supplier for a spinning wheel, roller bar, or flywheel from a
   * continuous rotor/shaft {@link Angle} supplier (such as {@code
   * SmartMotorController::getMechanismPosition}).
   *
   * @param wheelAngleSupplier Supplier returning the cumulative wheel {@link Angle}.
   * @param axleOriginMeters Axle center coordinates in robot-frame meters.
   * @param spinAxis Axis about which the wheel spins (typically {@link MotionAxis#Y} for horizontal
   *     intake/shooter rollers).
   * @return Robot-relative {@link Pose3d} supplier for the spinning wheel mesh.
   */
  public static Supplier<Pose3d> fromSpinningWheel(
      Supplier<Angle> wheelAngleSupplier, Translation3d axleOriginMeters, MotionAxis spinAxis) {
    return fromSpinningWheel(wheelAngleSupplier, axleOriginMeters, spinAxis, false);
  }

  /**
   * Builds a {@link Pose3d} supplier for a spinning wheel, roller bar, or flywheel with optional
   * spin-direction inversion.
   *
   * @param wheelAngleSupplier Supplier returning the cumulative wheel {@link Angle}.
   * @param axleOriginMeters Axle center coordinates in robot-frame meters.
   * @param spinAxis Axis about which the wheel spins.
   * @param inverted {@code true} to reverse visual spin direction.
   * @return Robot-relative {@link Pose3d} supplier for the spinning wheel mesh.
   */
  public static Supplier<Pose3d> fromSpinningWheel(
      Supplier<Angle> wheelAngleSupplier,
      Translation3d axleOriginMeters,
      MotionAxis spinAxis,
      boolean inverted) {
    return fromAngleSupplier(wheelAngleSupplier, axleOriginMeters, spinAxis, inverted);
  }

  /**
   * Builds a {@link Pose3d} supplier for a spinning wheel, roller, or flywheel by integrating an
   * {@link AngularVelocity} supplier over elapsed simulation time.
   *
   * @param velocitySupplier Supplier returning the current {@link AngularVelocity}.
   * @param axleOriginMeters Axle center coordinates in robot-frame meters.
   * @param spinAxis Axis about which the wheel spins.
   * @return Robot-relative {@link Pose3d} supplier integrating spin angle automatically.
   */
  public static Supplier<Pose3d> fromAngularVelocity(
      Supplier<AngularVelocity> velocitySupplier,
      Translation3d axleOriginMeters,
      MotionAxis spinAxis) {
    return fromAngularVelocity(velocitySupplier, axleOriginMeters, spinAxis, false);
  }

  /**
   * Builds a {@link Pose3d} supplier for a spinning wheel, roller, or flywheel by integrating an
   * {@link AngularVelocity} supplier with optional direction inversion.
   *
   * @param velocitySupplier Supplier returning the current {@link AngularVelocity}.
   * @param axleOriginMeters Axle center coordinates in robot-frame meters.
   * @param spinAxis Axis about which the wheel spins.
   * @param inverted {@code true} to reverse visual spin direction.
   * @return Robot-relative {@link Pose3d} supplier integrating spin angle automatically.
   */
  public static Supplier<Pose3d> fromAngularVelocity(
      Supplier<AngularVelocity> velocitySupplier,
      Translation3d axleOriginMeters,
      MotionAxis spinAxis,
      boolean inverted) {
    Objects.requireNonNull(velocitySupplier, "velocitySupplier");
    Objects.requireNonNull(axleOriginMeters, "axleOriginMeters");
    Objects.requireNonNull(spinAxis, "spinAxis");
    double sign = inverted ? -1.0 : 1.0;
    final double[] state = {Double.NaN, 0.0}; // [lastTimestampSec, accumulatedRad]
    return () -> {
      double now = System.nanoTime() * 1e-9;
      double dt = Double.isNaN(state[0]) ? 0.02 : Math.min(0.1, Math.max(0.0, now - state[0]));
      state[0] = now;
      AngularVelocity vel = velocitySupplier.get();
      double radPerSec = vel != null ? sign * vel.in(RadiansPerSecond) : 0.0;
      state[1] = (state[1] + radPerSec * dt) % (2.0 * Math.PI);
      return new Pose3d(axleOriginMeters, spinAxis.toRotation3d(state[1]));
    };
  }

  /**
   * Mounts a spinning roller bar or wheel (for example, an intake roller on a deploying intake arm)
   * onto a moving parent mechanism by chaining the parent pose with the roller's local spinning
   * axle pose.
   *
   * @param parentPoseSupplier Supplier returning the moving parent mechanism's {@link Pose3d}.
   * @param rollerAngleSupplier Supplier returning the roller's cumulative {@link Angle}.
   * @param localAxleOffsetMeters Roller axle offset in the parent mechanism's local frame (meters).
   * @param spinAxis Axis about which the roller spins in the parent frame.
   * @return Robot-relative {@link Pose3d} supplier for the mounted spinning roller.
   */
  public static Supplier<Pose3d> fromMountedRoller(
      Supplier<Pose3d> parentPoseSupplier,
      Supplier<Angle> rollerAngleSupplier,
      Translation3d localAxleOffsetMeters,
      MotionAxis spinAxis) {
    return fromMountedRoller(
        parentPoseSupplier, rollerAngleSupplier, localAxleOffsetMeters, spinAxis, false);
  }

  /**
   * Mounts a spinning roller bar or wheel onto a moving parent mechanism with optional spin
   * direction inversion.
   *
   * @param parentPoseSupplier Supplier returning the moving parent mechanism's {@link Pose3d}.
   * @param rollerAngleSupplier Supplier returning the roller's cumulative {@link Angle}.
   * @param localAxleOffsetMeters Roller axle offset in the parent mechanism's local frame (meters).
   * @param spinAxis Axis about which the roller spins in the parent frame.
   * @param inverted {@code true} to reverse visual spin direction.
   * @return Robot-relative {@link Pose3d} supplier for the mounted spinning roller.
   */
  public static Supplier<Pose3d> fromMountedRoller(
      Supplier<Pose3d> parentPoseSupplier,
      Supplier<Angle> rollerAngleSupplier,
      Translation3d localAxleOffsetMeters,
      MotionAxis spinAxis,
      boolean inverted) {
    Supplier<Pose3d> localSpin =
        fromSpinningWheel(rollerAngleSupplier, localAxleOffsetMeters, spinAxis, inverted);
    return chainParentChild(parentPoseSupplier, localSpin);
  }
}
