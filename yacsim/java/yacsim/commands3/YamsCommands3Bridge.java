// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.commands3;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Optional;
import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadJointType;
import yacsim.core.manifest.CadLinkManifest;
import yacsim.core.manifest.CadRobotManifest;
import yacsim.core.manifest.YacSimDiagnostic;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.yams.MotionAxis;
import yacsim.core.yams.YamsMechanismBridge;
import yams.commands3.mechanisms.Arm;
import yams.commands3.mechanisms.DifferentialMechanism;
import yams.commands3.mechanisms.DoubleJointedArm;
import yams.commands3.mechanisms.Elevator;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.drivetrain.CadDrivetrainConfig;
import yacsim.core.drivetrain.YamsDrivetrainBridge;
import yams.commands3.mechanisms.FlyWheel;
import yams.commands3.mechanisms.Pivot;
import yams.core.motorcontrollers.SmartMotorController;
import yams.commands3.swerve.SwerveDrive;

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

  /**
   * Builds per-stage {@link Pose3d} suppliers for a multi-stage cascading YAMS {@link Elevator}.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param stageCount Number of moving stages (must be at least 1).
   * @param baseMeters Base origin in robot-frame meters when height is zero.
   * @param translationAxis Axis of elevator travel (typically {@link MotionAxis#Z}).
   * @return Unmodifiable list of {@code stageCount} {@link Pose3d} suppliers.
   */
  public static List<Supplier<Pose3d>> fromCascadingElevator(
      Elevator elevator, int stageCount, Translation3d baseMeters, MotionAxis translationAxis) {
    Objects.requireNonNull(elevator, "elevator");
    return YamsMechanismBridge.fromCascadingElevator(
        elevator::getHeight, stageCount, baseMeters, translationAxis);
  }

  /**
   * Creates a level-riding 4-bar end-effector {@link Pose3d} supplier driven by a YAMS {@link Arm}.
   *
   * @param arm Driving YAMS {@link Arm}.
   * @param pivotMeters Pivot point in robot-frame meters.
   * @param armLengthMeters Positive arm length in meters.
   * @param rotationAxis Axis of arm rotation (typically {@link MotionAxis#Y}).
   * @return Robot-relative {@link Pose3d} supplier for the level end-effector.
   */
  public static Supplier<Pose3d> fromFourBar(
      Arm arm, Translation3d pivotMeters, double armLengthMeters, MotionAxis rotationAxis) {
    Objects.requireNonNull(arm, "arm");
    return YamsMechanismBridge.fromFourBar(
        arm::getAngle, pivotMeters, armLengthMeters, rotationAxis);
  }

  /**
   * Creates a level-riding 4-bar end-effector {@link Pose3d} supplier driven by a YAMS {@link Arm}
   * with optional angle inversion.
   *
   * @param arm Driving YAMS {@link Arm}.
   * @param pivotMeters Pivot point in robot-frame meters.
   * @param armLengthMeters Positive arm length in meters.
   * @param rotationAxis Axis of arm rotation.
   * @param inverted {@code true} to negate the driving angle.
   * @return Robot-relative {@link Pose3d} supplier for the level end-effector.
   */
  public static Supplier<Pose3d> fromFourBar(
      Arm arm,
      Translation3d pivotMeters,
      double armLengthMeters,
      MotionAxis rotationAxis,
      boolean inverted) {
    Objects.requireNonNull(arm, "arm");
    return YamsMechanismBridge.fromFourBar(
        arm::getAngle, pivotMeters, armLengthMeters, rotationAxis, inverted);
  }

  /**
   * Wires a YAMS Commands v3 {@link SwerveDrive} directly into a {@link CadDrivetrainConfig},
   * automatically binding {@code getPose()}, {@code getGyroRotation3d()}, and {@code
   * getModulePositions()}.
   *
   * @param drive YAMS {@link SwerveDrive} instance.
   * @param chassisAsset Chassis CAD mesh asset.
   * @param steerForkAsset Optional shared steer-fork CAD asset (may be {@code null}).
   * @param wheelAsset Optional shared wheel CAD asset (may be {@code null}).
   * @param wheelRadiusMeters Positive wheel radius in meters.
   * @param moduleLocations Module mounting locations in chassis frame (typically FL, FR, BL, BR).
   * @return Configured {@link CadDrivetrainConfig}.
   */
  public static CadDrivetrainConfig fromYamsSwerve(
      SwerveDrive drive,
      CadAssetConfig chassisAsset,
      CadAssetConfig steerForkAsset,
      CadAssetConfig wheelAsset,
      double wheelRadiusMeters,
      List<Translation3d> moduleLocations) {
    Objects.requireNonNull(drive, "drive");
    return YamsDrivetrainBridge.fromYamsSwerve(
        chassisAsset,
        steerForkAsset,
        wheelAsset,
        wheelRadiusMeters,
        moduleLocations,
        drive::getPose,
        drive::getGyroRotation3d,
        drive::getModulePositions);
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS Commands v3 {@link FlyWheel}'s native
   * {@code getPose3d()} with an optional fixed CAD mount offset.
   *
   * @param flywheel YAMS {@link FlyWheel} instance.
   * @param cadMountOffset Fixed transform from flywheel origin to CAD mesh origin.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsFlyWheel(FlyWheel flywheel, Transform3d cadMountOffset) {
    Objects.requireNonNull(flywheel, "flywheel");
    return YamsMechanismBridge.fromPoseSupplier(flywheel::getPose3d, cadMountOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link Arm}'s native {@code
   * getPose3d()}, eliminating duplicate pivot/axis entry when YAMS already knows the mechanism
   * geometry.
   *
   * @param arm YAMS {@link Arm} instance.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(Arm arm) {
    return fromYamsMechanism(arm, new Transform3d());
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link Arm}'s native {@code
   * getPose3d()} with a fixed CAD mount offset.
   *
   * @param arm YAMS {@link Arm} instance.
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(Arm arm, Transform3d cadMountOffset) {
    Objects.requireNonNull(arm, "arm");
    return YamsMechanismBridge.fromPoseSupplier(arm::getPose3d, cadMountOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link Elevator}'s native {@code
   * getPose3d()}.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(Elevator elevator) {
    return fromYamsMechanism(elevator, new Transform3d());
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link Elevator}'s native {@code
   * getPose3d()} with a fixed CAD mount offset.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(Elevator elevator, Transform3d cadMountOffset) {
    Objects.requireNonNull(elevator, "elevator");
    return YamsMechanismBridge.fromPoseSupplier(elevator::getPose3d, cadMountOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link Pivot}'s native {@code
   * getPose3d()}.
   *
   * @param pivot YAMS {@link Pivot} instance.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(Pivot pivot) {
    return fromYamsMechanism(pivot, new Transform3d());
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link Pivot}'s native {@code
   * getPose3d()} with a fixed CAD mount offset.
   *
   * @param pivot YAMS {@link Pivot} instance.
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(Pivot pivot, Transform3d cadMountOffset) {
    Objects.requireNonNull(pivot, "pivot");
    return YamsMechanismBridge.fromPoseSupplier(pivot::getPose3d, cadMountOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link FlyWheel}'s native {@code
   * getPose3d()}.
   *
   * @param flywheel YAMS {@link FlyWheel} instance.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(FlyWheel flywheel) {
    return fromYamsFlyWheel(flywheel, new Transform3d());
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link FlyWheel}'s native {@code
   * getPose3d()} with a fixed CAD mount offset.
   *
   * @param flywheel YAMS {@link FlyWheel} instance.
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(FlyWheel flywheel, Transform3d cadMountOffset) {
    return fromYamsFlyWheel(flywheel, cadMountOffset);
  }

  /**
   * Builds per-stage {@link Pose3d} suppliers directly from a YAMS 2027 {@link Elevator}'s native
   * {@code getCascadePose3d()} SPI, honoring {@code ElevatorConfig.withStages(...)} and any parent
   * mechanism mount configured in YAMS.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param stageCount Number of stages to expose (must be at least 1).
   * @return Unmodifiable list of {@code stageCount} {@link Pose3d} suppliers.
   */
  public static List<Supplier<Pose3d>> fromYamsCascadeElevator(Elevator elevator, int stageCount) {
    return fromYamsCascadeElevator(elevator, stageCount, new Transform3d());
  }

  /**
   * Builds per-stage {@link Pose3d} suppliers directly from a YAMS 2027 {@link Elevator}'s native
   * {@code getCascadePose3d()} SPI with a fixed CAD mount offset.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param stageCount Number of stages to expose (must be at least 1).
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Unmodifiable list of {@code stageCount} {@link Pose3d} suppliers.
   */
  public static List<Supplier<Pose3d>> fromYamsCascadeElevator(
      Elevator elevator, int stageCount, Transform3d cadMountOffset) {
    Objects.requireNonNull(elevator, "elevator");
    return YamsMechanismBridge.fromPoseListSuppliers(
        elevator::getCascadePose3d, stageCount, cadMountOffset);
  }

  /**
   * Builds per-stage {@link Pose3d} suppliers directly from a YAMS 2027 {@link Elevator}'s native
   * {@code getContinuousPose3d(boolean)} SPI.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param stageCount Number of stages to expose (must be at least 1).
   * @param carriageFirst {@code true} if the carriage moves before the intermediate stages.
   * @return Unmodifiable list of {@code stageCount} {@link Pose3d} suppliers.
   */
  public static List<Supplier<Pose3d>> fromYamsContinuousElevator(
      Elevator elevator, int stageCount, boolean carriageFirst) {
    return fromYamsContinuousElevator(elevator, stageCount, carriageFirst, new Transform3d());
  }

  /**
   * Builds per-stage {@link Pose3d} suppliers directly from a YAMS 2027 {@link Elevator}'s native
   * {@code getContinuousPose3d(boolean)} SPI with a fixed CAD mount offset.
   *
   * @param elevator YAMS {@link Elevator} instance.
   * @param stageCount Number of stages to expose (must be at least 1).
   * @param carriageFirst {@code true} if the carriage moves before the intermediate stages.
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Unmodifiable list of {@code stageCount} {@link Pose3d} suppliers.
   */
  public static List<Supplier<Pose3d>> fromYamsContinuousElevator(
      Elevator elevator, int stageCount, boolean carriageFirst, Transform3d cadMountOffset) {
    Objects.requireNonNull(elevator, "elevator");
    return YamsMechanismBridge.fromPoseListSuppliers(
        () -> elevator.getContinuousPose3d(carriageFirst), stageCount, cadMountOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier for the lower link of a YAMS 2027 {@link DoubleJointedArm}
   * using its native {@code getPose3d()} SPI.
   *
   * @param arm YAMS {@link DoubleJointedArm} instance.
   * @return Robot-relative {@link Pose3d} supplier for the lower arm link.
   */
  public static Supplier<Pose3d> fromYamsMechanism(DoubleJointedArm arm) {
    return fromYamsMechanism(arm, new Transform3d());
  }

  /**
   * Creates a {@link Pose3d} supplier for the lower link of a YAMS 2027 {@link DoubleJointedArm}
   * using its native {@code getPose3d()} SPI with a fixed CAD mount offset.
   *
   * @param arm YAMS {@link DoubleJointedArm} instance.
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Robot-relative {@link Pose3d} supplier for the lower arm link.
   */
  public static Supplier<Pose3d> fromYamsMechanism(
      DoubleJointedArm arm, Transform3d cadMountOffset) {
    Objects.requireNonNull(arm, "arm");
    return YamsMechanismBridge.fromPoseSupplier(arm::getPose3d, cadMountOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier for the upper link of a YAMS 2027 {@link DoubleJointedArm}
   * using its native {@code getUpperPose3d()} SPI.
   *
   * @param arm YAMS {@link DoubleJointedArm} instance.
   * @return Robot-relative {@link Pose3d} supplier for the upper arm link.
   */
  public static Supplier<Pose3d> fromYamsDoubleJointedArmUpper(DoubleJointedArm arm) {
    return fromYamsDoubleJointedArmUpper(arm, new Transform3d());
  }

  /**
   * Creates a {@link Pose3d} supplier for the upper link of a YAMS 2027 {@link DoubleJointedArm}
   * using its native {@code getUpperPose3d()} SPI with a fixed CAD mount offset.
   *
   * @param arm YAMS {@link DoubleJointedArm} instance.
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Robot-relative {@link Pose3d} supplier for the upper arm link.
   */
  public static Supplier<Pose3d> fromYamsDoubleJointedArmUpper(
      DoubleJointedArm arm, Transform3d cadMountOffset) {
    Objects.requireNonNull(arm, "arm");
    return YamsMechanismBridge.fromPoseSupplier(arm::getUpperPose3d, cadMountOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link DifferentialMechanism}'s
   * native {@code getPose3d()} SPI.
   *
   * @param mechanism YAMS {@link DifferentialMechanism} instance.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(DifferentialMechanism mechanism) {
    return fromYamsMechanism(mechanism, new Transform3d());
  }

  /**
   * Creates a {@link Pose3d} supplier directly from a YAMS 2027 {@link DifferentialMechanism}'s
   * native {@code getPose3d()} SPI with a fixed CAD mount offset.
   *
   * @param mechanism YAMS {@link DifferentialMechanism} instance.
   * @param cadMountOffset Fixed transform from YAMS mechanism origin to CAD mesh origin.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsMechanism(
      DifferentialMechanism mechanism, Transform3d cadMountOffset) {
    Objects.requireNonNull(mechanism, "mechanism");
    return YamsMechanismBridge.fromPoseSupplier(mechanism::getPose3d, cadMountOffset);
  }

  /**
   * Creates a {@link Pose3d} supplier for a spinning roller, intake bar, or shooter wheel directly
   * from its YAMS {@link SmartMotorController}'s encoder angle ({@code getMechanismPosition()}).
   *
   * @param rollerMotor YAMS {@link SmartMotorController} driving the wheel/roller.
   * @param axleOriginMeters Axle center coordinates in robot-frame meters.
   * @param spinAxis Axis about which the roller spins.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsRollerMotor(
      SmartMotorController rollerMotor, Translation3d axleOriginMeters, MotionAxis spinAxis) {
    return fromYamsRollerMotor(rollerMotor, axleOriginMeters, spinAxis, false);
  }

  /**
   * Creates a {@link Pose3d} supplier for a spinning roller, intake bar, or shooter wheel directly
   * from its YAMS {@link SmartMotorController} with optional spin inversion.
   *
   * @param rollerMotor YAMS {@link SmartMotorController} driving the wheel/roller.
   * @param axleOriginMeters Axle center coordinates in robot-frame meters.
   * @param spinAxis Axis about which the roller spins.
   * @param inverted {@code true} to reverse visual spin direction.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromYamsRollerMotor(
      SmartMotorController rollerMotor,
      Translation3d axleOriginMeters,
      MotionAxis spinAxis,
      boolean inverted) {
    Objects.requireNonNull(rollerMotor, "rollerMotor");
    return YamsMechanismBridge.fromSpinningWheel(
        rollerMotor::getMechanismPosition, axleOriginMeters, spinAxis, inverted);
  }

  /**
   * Mounts a YAMS roller motor's spinning bar/wheel directly onto a moving parent mechanism (such
   * as an intake roller riding on a deploying slamtake arm).
   *
   * @param parentPoseSupplier Supplier returning the moving parent mechanism's {@link Pose3d}.
   * @param rollerMotor YAMS {@link SmartMotorController} driving the roller.
   * @param localAxleOffsetMeters Roller axle offset in the parent mechanism's local frame (meters).
   * @param spinAxis Axis about which the roller spins in the parent frame.
   * @param inverted {@code true} to reverse visual spin direction.
   * @return Robot-relative {@link Pose3d} supplier for the mounted spinning roller.
   */
  public static Supplier<Pose3d> fromMountedYamsRoller(
      Supplier<Pose3d> parentPoseSupplier,
      SmartMotorController rollerMotor,
      Translation3d localAxleOffsetMeters,
      MotionAxis spinAxis,
      boolean inverted) {
    Objects.requireNonNull(rollerMotor, "rollerMotor");
    return YamsMechanismBridge.fromMountedRoller(
        parentPoseSupplier,
        rollerMotor::getMechanismPosition,
        localAxleOffsetMeters,
        spinAxis,
        inverted);
  }

  /**
   * Builds a {@link Pose3d} supplier for a YAMS {@link Arm} using the joint origin, orientation,
   * motion axis, inversion, and zero offset stored in a {@link CadRobotManifest}.
   *
   * @param manifest Loaded {@link CadRobotManifest}.
   * @param jointName Name of the revolute joint in the manifest.
   * @param arm YAMS {@link Arm} instance.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> poseSupplierFromManifest(
      CadRobotManifest manifest, String jointName, Arm arm) {
    Objects.requireNonNull(manifest, "manifest");
    Objects.requireNonNull(arm, "arm");
    CadJointManifest joint = manifest.getJoint(jointName);
    return buildRevoluteSupplier(joint, arm::getAngle);
  }

  /**
   * Builds a {@link Pose3d} supplier for a YAMS {@link Pivot} using the joint geometry stored in a
   * {@link CadRobotManifest}.
   *
   * @param manifest Loaded {@link CadRobotManifest}.
   * @param jointName Name of the revolute/continuous joint in the manifest.
   * @param pivot YAMS {@link Pivot} instance.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> poseSupplierFromManifest(
      CadRobotManifest manifest, String jointName, Pivot pivot) {
    Objects.requireNonNull(manifest, "manifest");
    Objects.requireNonNull(pivot, "pivot");
    CadJointManifest joint = manifest.getJoint(jointName);
    return buildRevoluteSupplier(joint, pivot::getAngle);
  }

  /**
   * Builds a {@link Pose3d} supplier for a YAMS {@link Elevator} using the joint origin,
   * orientation, translation axis, inversion, and zero offset stored in a {@link CadRobotManifest}.
   *
   * @param manifest Loaded {@link CadRobotManifest}.
   * @param jointName Name of the prismatic joint in the manifest.
   * @param elevator YAMS {@link Elevator} instance.
   * @return Robot-relative {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> poseSupplierFromManifest(
      CadRobotManifest manifest, String jointName, Elevator elevator) {
    Objects.requireNonNull(manifest, "manifest");
    Objects.requireNonNull(elevator, "elevator");
    CadJointManifest joint = manifest.getJoint(jointName);
    return () -> {
      double d =
          elevator.getHeight().in(org.wpilib.units.Units.Meters) * (joint.inverted() ? -1.0 : 1.0);
      Translation3d delta =
          switch (joint.axis()) {
            case X -> new Translation3d(d, 0.0, 0.0);
            case Y -> new Translation3d(0.0, d, 0.0);
            case Z -> new Translation3d(0.0, 0.0, d);
          };
      return new Pose3d(joint.originMeters(), joint.originRotation())
          .transformBy(new Transform3d(delta, new Rotation3d()))
          .transformBy(joint.zeroOffset());
    };
  }

  /**
   * Binds a YAMS {@link Arm} directly to its child link in a manifest-loaded {@link
   * CadRobotConfig}, reading pivot, axis, orientation, and inversion automatically from the
   * manifest.
   *
   * @param config {@link CadRobotConfig} loaded via {@link CadRobotConfig#fromManifest} or {@link
   *     CadRobotConfig#fromUrdf}.
   * @param jointName Name of the joint in the manifest.
   * @param arm YAMS {@link Arm} instance.
   * @return The same {@code config} for fluent chaining.
   */
  public static CadRobotConfig bindFromManifest(
      CadRobotConfig config, String jointName, Arm arm) {
    Objects.requireNonNull(config, "config");
    CadRobotManifest manifest = requireManifest(config);
    CadJointManifest joint = manifest.getJoint(jointName);
    Supplier<Pose3d> chained =
        chainFromManifest(config, jointName, poseSupplierFromManifest(manifest, jointName, arm));
    return config.withComponentPoseSupplier(joint.childLink(), chained);
  }

  /**
   * Binds a YAMS {@link Elevator} directly to its child link in a manifest-loaded {@link
   * CadRobotConfig}, reading base origin, axis, orientation, and inversion automatically from the
   * manifest and automatically chaining any parent URDF kinematic links.
   *
   * @param config {@link CadRobotConfig} loaded via {@link CadRobotConfig#fromManifest} or {@link
   *     CadRobotConfig#fromUrdf}.
   * @param jointName Name of the prismatic joint in the manifest.
   * @param elevator YAMS {@link Elevator} instance.
   * @return The same {@code config} for fluent chaining.
   */
  public static CadRobotConfig bindFromManifest(
      CadRobotConfig config, String jointName, Elevator elevator) {
    Objects.requireNonNull(config, "config");
    CadRobotManifest manifest = requireManifest(config);
    CadJointManifest joint = manifest.getJoint(jointName);
    Supplier<Pose3d> chained =
        chainFromManifest(
            config, jointName, poseSupplierFromManifest(manifest, jointName, elevator));
    return config.withComponentPoseSupplier(joint.childLink(), chained);
  }

  /**
   * Binds a YAMS {@link Pivot} directly to its child link in a manifest-loaded {@link
   * CadRobotConfig}, automatically chaining any parent URDF kinematic links.
   *
   * @param config {@link CadRobotConfig} loaded via {@link CadRobotConfig#fromManifest} or {@link
   *     CadRobotConfig#fromUrdf}.
   * @param jointName Name of the revolute/continuous joint in the manifest.
   * @param pivot YAMS {@link Pivot} instance.
   * @return The same {@code config} for fluent chaining.
   */
  public static CadRobotConfig bindFromManifest(
      CadRobotConfig config, String jointName, Pivot pivot) {
    Objects.requireNonNull(config, "config");
    CadRobotManifest manifest = requireManifest(config);
    CadJointManifest joint = manifest.getJoint(jointName);
    Supplier<Pose3d> chained =
        chainFromManifest(config, jointName, poseSupplierFromManifest(manifest, jointName, pivot));
    return config.withComponentPoseSupplier(joint.childLink(), chained);
  }

  /**
   * Wraps a joint-local {@link Pose3d} supplier with its full URDF/manifest ancestor kinematic
   * chain (for example, a shooter hood mounted on a rotating turret with intermediate static links),
   * resolving the nearest live ancestor component dynamically so binding order never matters.
   *
   * @param config Manifest- or URDF-loaded {@link CadRobotConfig}.
   * @param jointName Name of the joint in the manifest.
   * @param localJointPoseSupplier Supplier returning the joint's local {@link Pose3d} relative to
   *     its parent link.
   * @return Robot-relative {@link Pose3d} supplier incorporating all parent transforms.
   */
  public static Supplier<Pose3d> chainFromManifest(
      CadRobotConfig config, String jointName, Supplier<Pose3d> localJointPoseSupplier) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(localJointPoseSupplier, "localJointPoseSupplier");
    CadRobotManifest manifest = requireManifest(config);
    CadJointManifest joint = manifest.getJoint(jointName);
    String chassisName = manifest.chassis().map(CadLinkManifest::name).orElse("Chassis");
    String parentLink = joint.parentLink();
    if (parentLink == null
        || parentLink.isBlank()
        || parentLink.equals(chassisName)
        || parentLink.equalsIgnoreCase("Chassis")) {
      return localJointPoseSupplier;
    }

    Map<String, CadJointManifest> parentJointByChild = new LinkedHashMap<>();
    for (CadJointManifest j : manifest.joints()) {
      parentJointByChild.putIfAbsent(j.childLink(), j);
    }

    List<CadJointManifest> intermediateJoints = new ArrayList<>();
    Set<String> visited = new LinkedHashSet<>();
    String cur = parentLink;
    String liveAncestorLink = null;
    while (cur != null
        && !cur.isBlank()
        && !cur.equals(chassisName)
        && !cur.equalsIgnoreCase("Chassis")
        && visited.add(cur)) {
      if (config.hasComponent(cur)) {
        liveAncestorLink = cur;
        break;
      }
      CadJointManifest ancestorJoint = parentJointByChild.get(cur);
      if (ancestorJoint == null) {
        break;
      }
      intermediateJoints.add(ancestorJoint);
      cur = ancestorJoint.parentLink();
    }

    Collections.reverse(intermediateJoints);
    Pose3d intermediatePose = new Pose3d();
    for (CadJointManifest j : intermediateJoints) {
      intermediatePose =
          intermediatePose.transformBy(new Transform3d(j.originMeters(), j.originRotation()));
      if (!j.zeroOffset().equals(new Transform3d())) {
        intermediatePose = intermediatePose.transformBy(j.zeroOffset());
      }
    }
    final Transform3d intermediateTransform =
        new Transform3d(intermediatePose.getTranslation(), intermediatePose.getRotation());
    final boolean hasIntermediate = !intermediateJoints.isEmpty();

    if (liveAncestorLink != null) {
      final String ancestorKey = liveAncestorLink;
      Supplier<Pose3d> dynamicAncestor =
          () -> {
            Pose3d base =
                config.getComponentPoseSupplier(ancestorKey).map(Supplier::get).orElseGet(Pose3d::new);
            return hasIntermediate ? base.transformBy(intermediateTransform) : base;
          };
      return YamsMechanismBridge.chainParentChild(dynamicAncestor, localJointPoseSupplier);
    }

    if (hasIntermediate) {
      final Pose3d fixedPrefix = intermediatePose;
      return YamsMechanismBridge.chainParentChild(() -> fixedPrefix, localJointPoseSupplier);
    }
    return localJointPoseSupplier;
  }

  /**
   * Binds a YAMS roller, intake bar, kicker, or flywheel {@link SmartMotorController} to a
   * manifest-loaded {@link CadRobotConfig}. Automatically registers NT4 roller telemetry using the
   * manifest's {@code wheelRadiusMeters}, and if the joint's {@code childLink} has a visual mesh in
   * {@code config}, also binds its 3D spinning {@link Pose3d} supplier.
   *
   * @param config {@link CadRobotConfig} loaded via {@link CadRobotConfig#fromManifest} or {@link
   *     CadRobotConfig#fromUrdf}.
   * @param jointName Name of the continuous joint in the manifest.
   * @param rollerMotor YAMS {@link SmartMotorController} driving the wheel or roller.
   * @return The same {@code config} for fluent chaining.
   */
  public static CadRobotConfig bindFromManifest(
      CadRobotConfig config, String jointName, SmartMotorController rollerMotor) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(rollerMotor, "rollerMotor");
    CadRobotManifest manifest = requireManifest(config);
    CadJointManifest joint = manifest.getJoint(jointName);
    Supplier<Pose3d> parentPose =
        config.getComponentPoseSupplier(joint.parentLink()).orElse(null);
    return bindFromManifest(config, jointName, rollerMotor, parentPose);
  }

  /**
   * Binds a YAMS roller, intake bar, kicker, or flywheel {@link SmartMotorController} to a
   * manifest-loaded {@link CadRobotConfig} with an explicit parent mechanism {@link Pose3d}
   * supplier. If {@code parentPoseSupplier} is non-null (or automatically discovered from {@code
   * joint.parentLink()}), the roller rides on the moving parent mechanism while spinning around its
   * own local axle in AdvantageScope.
   *
   * @param config {@link CadRobotConfig} loaded via {@link CadRobotConfig#fromManifest} or {@link
   *     CadRobotConfig#fromUrdf}.
   * @param jointName Name of the continuous joint in the manifest.
   * @param rollerMotor YAMS {@link SmartMotorController} driving the wheel or roller.
   * @param parentPoseSupplier Optional moving parent {@link Pose3d} supplier, or {@code null} for
   *     chassis-mounted rollers.
   * @return The same {@code config} for fluent chaining.
   */
  public static CadRobotConfig bindFromManifest(
      CadRobotConfig config,
      String jointName,
      SmartMotorController rollerMotor,
      Supplier<Pose3d> parentPoseSupplier) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(rollerMotor, "rollerMotor");
    CadRobotManifest manifest = requireManifest(config);
    CadJointManifest joint = manifest.getJoint(jointName);
    if (joint.wheelRadiusMeters().isPresent()) {
      config.addRollerTelemetry(
          joint.name(),
          rollerMotor::getMechanismVelocity,
          joint.wheelRadiusMeters().getAsDouble());
    }
    if (config.hasComponent(joint.childLink())) {
      Supplier<Pose3d> poseSupplier =
          parentPoseSupplier != null
              ? YamsMechanismBridge.fromMountedRoller(
                  parentPoseSupplier,
                  rollerMotor::getMechanismPosition,
                  joint.originMeters(),
                  joint.axis(),
                  joint.inverted())
              : buildRevoluteSupplier(joint, rollerMotor::getMechanismPosition);
      Supplier<Pose3d> finalSupplier =
          parentPoseSupplier != null
              ? poseSupplier
              : chainFromManifest(config, jointName, poseSupplier);
      boolean allowCollisionReenable = joint.type() != CadJointType.CONTINUOUS;
      config.withComponentPoseSupplier(joint.childLink(), finalSupplier, allowCollisionReenable);
    }
    return config;
  }

  private static Supplier<Pose3d> buildRevoluteSupplier(
      CadJointManifest joint, Supplier<org.wpilib.units.measure.Angle> angleSupplier) {
    return () -> {
      double rad =
          angleSupplier.get().in(org.wpilib.units.Units.Radians) * (joint.inverted() ? -1.0 : 1.0);
      Rotation3d rot =
          switch (joint.axis()) {
            case X -> new Rotation3d(rad, 0.0, 0.0);
            case Y -> new Rotation3d(0.0, rad, 0.0);
            case Z -> new Rotation3d(0.0, 0.0, rad);
          };
      return new Pose3d(joint.originMeters(), joint.originRotation())
          .transformBy(new Transform3d(new Translation3d(), rot))
          .transformBy(joint.zeroOffset());
    };
  }

  /**
   * Automatically binds a map of Java mechanism names to YAMS 2027 mechanisms ({@link Arm}, {@link
   * Elevator}, {@link Pivot}, {@link FlyWheel}, or {@link SmartMotorController}) against a
   * manifest- or URDF-loaded {@link CadRobotConfig}.
   *
   * <p>Never throws if a name is missing from CAD—instead returns a {@link
   * YacSimDiagnostic.BindingReport} listing every bound mechanism, unmatched Java name, and
   * unbound CAD joint (with copy-pasteable {@code CadUrdfOptions.withJointAlias(...)} hints).
   *
   * @param config {@link CadRobotConfig} loaded from a manifest or URDF.
   * @param mechanisms Map of {@code mechanismName -> YAMS mechanism or SmartMotorController}.
   * @return {@link YacSimDiagnostic.BindingReport} summarizing the binding results.
   */
  public static YacSimDiagnostic.BindingReport autoBind(
      CadRobotConfig config, Map<String, Object> mechanisms) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(mechanisms, "mechanisms");
    CadRobotManifest manifest = requireManifest(config);

    List<String> bound = new ArrayList<>();
    List<String> unmatched = new ArrayList<>();

    for (Map.Entry<String, Object> entry : mechanisms.entrySet()) {
      String javaName = entry.getKey();
      Object mech = entry.getValue();
      if (javaName == null || mech == null) {
        continue;
      }
      Optional<CadJointManifest> optJoint = manifest.findJoint(javaName);
      if (optJoint.isEmpty()) {
        unmatched.add(javaName);
        continue;
      }
      CadJointManifest joint = optJoint.get();
      if (mech instanceof Arm arm) {
        if (config.hasComponent(joint.childLink())) {
          bindFromManifest(config, joint.name(), arm);
          bound.add(javaName + " -> " + joint.name());
        } else {
          unmatched.add(javaName);
        }
      } else if (mech instanceof Elevator elevator) {
        if (config.hasComponent(joint.childLink())) {
          bindFromManifest(config, joint.name(), elevator);
          bound.add(javaName + " -> " + joint.name());
        } else {
          unmatched.add(javaName);
        }
      } else if (mech instanceof Pivot pivot) {
        if (config.hasComponent(joint.childLink())) {
          bindFromManifest(config, joint.name(), pivot);
          bound.add(javaName + " -> " + joint.name());
        } else {
          unmatched.add(javaName);
        }
      } else if (mech instanceof FlyWheel flywheel) {
        if (config.hasComponent(joint.childLink())) {
          config.withComponentPoseSupplier(joint.childLink(), fromYamsMechanism(flywheel));
          bound.add(javaName + " -> " + joint.name());
        } else {
          unmatched.add(javaName);
        }
      } else if (mech instanceof DoubleJointedArm dja) {
        if (config.hasComponent(joint.childLink())) {
          config.withComponentPoseSupplier(joint.childLink(), fromYamsMechanism(dja));
          bound.add(javaName + " -> " + joint.name());
        } else {
          unmatched.add(javaName);
        }
      } else if (mech instanceof DifferentialMechanism diff) {
        if (config.hasComponent(joint.childLink())) {
          config.withComponentPoseSupplier(joint.childLink(), fromYamsMechanism(diff));
          bound.add(javaName + " -> " + joint.name());
        } else {
          unmatched.add(javaName);
        }
      } else if (mech instanceof SmartMotorController smc) {
        bindFromManifest(config, joint.name(), smc);
        bound.add(javaName + " -> " + joint.name());
      } else {
        throw new IllegalArgumentException(
            "Unsupported mechanism type for '"
                + javaName
                + "': "
                + mech.getClass().getName()
                + " (expected Arm, Elevator, Pivot, FlyWheel, DoubleJointedArm, DifferentialMechanism, or SmartMotorController).");
      }
    }

    List<String> unboundJoints =
        manifest.joints().stream()
            .filter(j -> j.type() != CadJointType.FIXED)
            .filter(j -> !config.isComponentBound(j.childLink()))
            .map(CadJointManifest::name)
            .toList();

    StringBuilder summary = new StringBuilder();
    summary.append(YacSimDiagnostic.formatReport(config));
    if (!unmatched.isEmpty()) {
      summary.append("  Unmatched Java mechanisms: ").append(unmatched).append('\n');
      for (String u : unmatched) {
        YacSimDiagnostic.suggestBestJointMatch(u, manifest.joints())
            .ifPresent(
                cadJoint ->
                    summary
                        .append("    Suggested alias: .withJointAlias(\"")
                        .append(cadJoint)
                        .append("\", \"")
                        .append(u)
                        .append("\")\n"));
      }
    }
    return new YacSimDiagnostic.BindingReport(bound, unmatched, unboundJoints, summary.toString());
  }

  /**
   * Attaches animated 4-corner swerve steer forks and spinning wheels from a YAMS {@link
   * SwerveDrive} directly onto an existing URDF- or manifest-loaded {@link CadRobotConfig},
   * automatically reusing its configured chassis CAD asset.
   *
   * <p>Combine with {@code CadUrdfOptions.recommended().excludeMeshPatterns("MAXSwerve")} if your
   * CAD chassis contains static simplified swerve blocks that would otherwise overlap the animated
   * modules.
   *
   * @param config Existing {@link CadRobotConfig} with a chassis asset.
   * @param drive YAMS {@link SwerveDrive} instance.
   * @param steerForkAsset Optional shared steer-fork CAD asset (may be {@code null}).
   * @param wheelAsset Optional shared wheel CAD asset (may be {@code null}).
   * @param wheelRadiusMeters Positive wheel radius in meters.
   * @param moduleLocations Module mounting locations in chassis frame (typically FL, FR, BL, BR).
   * @return The same {@code config} for fluent chaining.
   */
  public static CadRobotConfig attachSwerveModules(
      CadRobotConfig config,
      SwerveDrive drive,
      CadAssetConfig steerForkAsset,
      CadAssetConfig wheelAsset,
      double wheelRadiusMeters,
      List<Translation3d> moduleLocations) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(drive, "drive");
    CadAssetConfig chassis =
        config
            .getChassisAsset()
            .orElseThrow(
                () ->
                    new IllegalStateException(
                        "CadRobotConfig '"
                            + config.getRobotName()
                            + "' has no chassis asset. Load a chassis via fromUrdfZip/fromManifest/withChassisAsset first."));
    CadDrivetrainConfig dt =
        fromYamsSwerve(
            drive, chassis, steerForkAsset, wheelAsset, wheelRadiusMeters, moduleLocations);
    return config.withDrivetrain(dt);
  }

  /**
   * Attaches animated 4-corner swerve steer forks and spinning wheels from a YAMS 2027 {@link
   * SwerveDrive} AND binds {@code drive::getSimPose} as the ground-truth {@code SimTruthPose3d}
   * alongside noisy fused odometry ({@code drive::getPose}).
   *
   * @param config Existing {@link CadRobotConfig} with a chassis asset.
   * @param drive YAMS {@link SwerveDrive} instance.
   * @param steerForkAsset Optional shared steer-fork CAD asset (may be {@code null}).
   * @param wheelAsset Optional shared wheel CAD asset (may be {@code null}).
   * @param wheelRadiusMeters Positive wheel radius in meters.
   * @param moduleLocations Module mounting locations in chassis frame (typically FL, FR, BL, BR).
   * @return The same {@code config} for fluent chaining.
   */
  public static CadRobotConfig attachSwerveModulesWithTruth(
      CadRobotConfig config,
      SwerveDrive drive,
      CadAssetConfig steerForkAsset,
      CadAssetConfig wheelAsset,
      double wheelRadiusMeters,
      List<Translation3d> moduleLocations) {
    attachSwerveModules(
        config, drive, steerForkAsset, wheelAsset, wheelRadiusMeters, moduleLocations);
    return config.withSimTruthPose2dSupplier(drive::getSimPose);
  }

  /**
   * Drivetrain-agnostic overload attaching animated rotating steer forks and spinning wheels from
   * any swerve implementation (CTRE Phoenix 6 Swerve, REV MAXSwerve, AdvantageKit, or custom
   * odometry) onto an existing URDF- or manifest-loaded {@link CadRobotConfig}.
   *
   * @param config Existing {@link CadRobotConfig} with a chassis asset.
   * @param steerForkAsset Optional shared steer-fork CAD asset (may be {@code null}).
   * @param wheelAsset Optional shared wheel CAD asset (may be {@code null}).
   * @param wheelRadiusMeters Positive wheel radius in meters.
   * @param moduleLocations Module mounting locations in chassis frame (typically FL, FR, BL, BR).
   * @param fieldPoseSupplier Live 2D field pose supplier.
   * @param gyroRotationSupplier Live 3D gyro rotation supplier.
   * @param modulePositionsSupplier Live swerve module positions supplier.
   * @return The same {@code config} for fluent chaining.
   */
  public static CadRobotConfig attachSwerveModules(
      CadRobotConfig config,
      CadAssetConfig steerForkAsset,
      CadAssetConfig wheelAsset,
      double wheelRadiusMeters,
      List<Translation3d> moduleLocations,
      Supplier<Pose2d> fieldPoseSupplier,
      Supplier<Rotation3d> gyroRotationSupplier,
      Supplier<SwerveModulePosition[]> modulePositionsSupplier) {
    Objects.requireNonNull(config, "config");
    CadAssetConfig chassis =
        config
            .getChassisAsset()
            .orElseThrow(
                () ->
                    new IllegalStateException(
                        "CadRobotConfig '"
                            + config.getRobotName()
                            + "' has no chassis asset. Load a chassis via fromUrdfZip/fromManifest/withChassisAsset first."));
    CadDrivetrainConfig dt =
        YamsDrivetrainBridge.fromYamsSwerve(
            chassis,
            steerForkAsset,
            wheelAsset,
            wheelRadiusMeters,
            moduleLocations,
            fieldPoseSupplier,
            gyroRotationSupplier,
            modulePositionsSupplier);
    return config.withDrivetrain(dt);
  }

  private static CadRobotManifest requireManifest(CadRobotConfig config) {
    return config
        .getManifest()
        .orElseThrow(
            () ->
                new IllegalStateException(
                    "CadRobotConfig '"
                        + config.getRobotName()
                        + "' was not created from a manifest or URDF. Use CadRobotConfig.fromManifest(...) or fromUrdf(...)."));
  }
}
