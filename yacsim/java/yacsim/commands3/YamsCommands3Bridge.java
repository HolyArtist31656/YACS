// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.commands3;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadRobotManifest;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.yams.MotionAxis;
import yacsim.core.yams.YamsMechanismBridge;
import yams.commands3.mechanisms.Arm;
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
    return config.withComponentPoseSupplier(
        joint.childLink(), poseSupplierFromManifest(manifest, jointName, arm));
  }

  /**
   * Binds a YAMS {@link Elevator} directly to its child link in a manifest-loaded {@link
   * CadRobotConfig}, reading base origin, axis, orientation, and inversion automatically from the
   * manifest.
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
    return config.withComponentPoseSupplier(
        joint.childLink(), poseSupplierFromManifest(manifest, jointName, elevator));
  }

  /**
   * Binds a YAMS {@link Pivot} directly to its child link in a manifest-loaded {@link
   * CadRobotConfig}.
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
    return config.withComponentPoseSupplier(
        joint.childLink(), poseSupplierFromManifest(manifest, jointName, pivot));
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
      config.withComponentPoseSupplier(joint.childLink(), poseSupplier);
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
