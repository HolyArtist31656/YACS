// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.commands3;

import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.yams.MotionAxis;
import yacsim.core.yams.YamsMechanismBridge;
import yams.commands3.mechanisms.Arm;
import yams.commands3.mechanisms.Elevator;
import yams.commands3.mechanisms.Pivot;

/**
 * Direct WPILib Commands v3 adapters connecting YAMS 2027 mechanisms ({@link Arm}, {@link
 * Elevator}, and {@link Pivot}) to YACSim {@link Pose3d} suppliers without hand-written pose math.
 */
public final class YamsCommands3Bridge {
  private YamsCommands3Bridge() {}

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Arm}.
   *
   * @param arm YAMS {@link Arm} instance.
   * @param pivotMeters Pivot point in robot-frame meters.
   * @param rotationAxis Axis of arm rotation (typically {@link MotionAxis#Y}).
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsArm(
      Arm arm, Translation3d pivotMeters, MotionAxis rotationAxis) {
    Objects.requireNonNull(arm, "arm");
    return YamsMechanismBridge.fromAngleSupplier(arm::getAngle, pivotMeters, rotationAxis);
  }

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Arm} with optional
   * direction inversion.
   *
   * @param arm YAMS {@link Arm} instance.
   * @param pivotMeters Pivot point in robot-frame meters.
   * @param rotationAxis Axis of arm rotation.
   * @param inverted {@code true} to negate the angle.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsArm(
      Arm arm, Translation3d pivotMeters, MotionAxis rotationAxis, boolean inverted) {
    Objects.requireNonNull(arm, "arm");
    return YamsMechanismBridge.fromAngleSupplier(
        arm::getAngle, pivotMeters, rotationAxis, inverted);
  }

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Arm} with optional
   * direction inversion and a fixed CAD zero offset transform.
   *
   * @param arm YAMS {@link Arm} instance.
   * @param pivotMeters Pivot point in robot-frame meters.
   * @param rotationAxis Axis of arm rotation.
   * @param inverted {@code true} to negate the angle.
   * @param zeroOffset Fixed transform applied after rotating around the pivot.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsArm(
      Arm arm,
      Translation3d pivotMeters,
      MotionAxis rotationAxis,
      boolean inverted,
      Transform3d zeroOffset) {
    Objects.requireNonNull(arm, "arm");
    return YamsMechanismBridge.fromAngleSupplier(
        arm::getAngle, pivotMeters, rotationAxis, inverted, zeroOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Elevator}.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param baseMeters Base origin in robot-frame meters when elevator height is zero.
   * @param translationAxis Axis of elevator travel (typically {@link MotionAxis#Z}).
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsElevator(
      Elevator elevator, Translation3d baseMeters, MotionAxis translationAxis) {
    Objects.requireNonNull(elevator, "elevator");
    return YamsMechanismBridge.fromDistanceSupplier(
        elevator::getHeight, baseMeters, translationAxis);
  }

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Elevator} with
   * optional direction inversion.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param baseMeters Base origin in robot-frame meters.
   * @param translationAxis Axis of elevator travel.
   * @param inverted {@code true} to negate the displacement direction.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsElevator(
      Elevator elevator, Translation3d baseMeters, MotionAxis translationAxis, boolean inverted) {
    Objects.requireNonNull(elevator, "elevator");
    return YamsMechanismBridge.fromDistanceSupplier(
        elevator::getHeight, baseMeters, translationAxis, inverted);
  }

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Elevator} with
   * optional direction inversion and a fixed CAD zero offset transform.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param baseMeters Base origin in robot-frame meters.
   * @param translationAxis Axis of elevator travel.
   * @param inverted {@code true} to negate the displacement direction.
   * @param zeroOffset Fixed transform applied after translating along the axis.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsElevator(
      Elevator elevator,
      Translation3d baseMeters,
      MotionAxis translationAxis,
      boolean inverted,
      Transform3d zeroOffset) {
    Objects.requireNonNull(elevator, "elevator");
    return YamsMechanismBridge.fromDistanceSupplier(
        elevator::getHeight, baseMeters, translationAxis, inverted, zeroOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Pivot} (such as a
   * turret or horizontal wrist).
   *
   * @param pivot YAMS {@link Pivot} instance.
   * @param pivotMeters Pivot point in robot-frame meters.
   * @param rotationAxis Axis of rotation (typically {@link MotionAxis#Z}).
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsPivot(
      Pivot pivot, Translation3d pivotMeters, MotionAxis rotationAxis) {
    Objects.requireNonNull(pivot, "pivot");
    return YamsMechanismBridge.fromAngleSupplier(pivot::getAngle, pivotMeters, rotationAxis);
  }

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Pivot} with
   * optional direction inversion.
   *
   * @param pivot YAMS {@link Pivot} instance.
   * @param pivotMeters Pivot point in robot-frame meters.
   * @param rotationAxis Axis of rotation.
   * @param inverted {@code true} to negate the angle.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsPivot(
      Pivot pivot, Translation3d pivotMeters, MotionAxis rotationAxis, boolean inverted) {
    Objects.requireNonNull(pivot, "pivot");
    return YamsMechanismBridge.fromAngleSupplier(
        pivot::getAngle, pivotMeters, rotationAxis, inverted);
  }

  /**
   * Creates a {@link Pose3d} supplier bound directly to a YAMS Commands v3 {@link Pivot} with
   * optional direction inversion and a fixed CAD zero offset transform.
   *
   * @param pivot YAMS {@link Pivot} instance.
   * @param pivotMeters Pivot point in robot-frame meters.
   * @param rotationAxis Axis of rotation.
   * @param inverted {@code true} to negate the angle.
   * @param zeroOffset Fixed transform applied after rotating around the pivot.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsPivot(
      Pivot pivot,
      Translation3d pivotMeters,
      MotionAxis rotationAxis,
      boolean inverted,
      Transform3d zeroOffset) {
    Objects.requireNonNull(pivot, "pivot");
    return YamsMechanismBridge.fromAngleSupplier(
        pivot::getAngle, pivotMeters, rotationAxis, inverted, zeroOffset);
  }
}
