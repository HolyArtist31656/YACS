// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Manifest-driven FRC 6328 "Darwin" Slamtake & Intake Roller (YAMS 2027 Commands v3 + YACSim)

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadRobotManifest;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Darwin Slamtake Arm + Intake Roller fed from {@code yacsim-robot.json}. Only motor CAN IDs
 * (12, 14), Kraken X60 motor types, gearing, and control gains live in Java.
 */
public class SlamtakeMechanism implements Mechanism {
  public static final double ROLLER_INTAKE_VOLTS = 13.0;
  public static final double ROLLER_OUTTAKE_VOLTS = -6.0;

  // Non-standard pin-in-slot linkage constants for coupled FrontHopper (6328 DarwinMechanism3d)
  private static final double PIVOT_TO_SLOT_Y_OFFSET = Units.inchesToMeters(2.393);
  private static final double PIVOT_TO_WASHER_NORM = Units.inchesToMeters(7.204);
  private static final double SLOT_START_TO_HOPPER_X = Units.inchesToMeters(6.789);
  private static final double SLOT_ANGLE_RAD = Math.toRadians(70.0);

  private final double robotToPivotX;
  private final Arm slamtake;
  private final SmartMotorController rollerSmc;
  private final FlyWheel intakeRoller;

  public SlamtakeMechanism(CadRobotManifest manifest) {
    CadJointManifest armJoint = manifest.getJoint("Slamtake");
    CadJointManifest rollerJoint = manifest.getJoint("IntakeRoller");
    this.robotToPivotX = armJoint.originMeters().getX();

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(35.0, 0.0, 0.4)
            .withSimClosedLoopController(35.0, 0.0, 0.4)
            .withTrapezoidalProfile(DegreesPerSecond.of(220), DegreesPerSecondPerSecond.of(440))
            .withFeedforward(new ArmFeedforward(0.0, 0.25, 0.0, 0.0))
            .withSimFeedforward(new ArmFeedforward(0.0, 0.25, 0.0, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("Slamtake"))
            .withMotorInverted(false)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withStartingPosition(armJoint.startingAngle())
            .withSoftLimits(armJoint.softMinLimitAngle(), armJoint.softMaxLimitAngle())
            .withTelemetry("SlamtakeMotor", TelemetryVerbosity.HIGH);

    SmartMotorController armSmc =
        new TalonFXWrapper(
            new TalonFX(12, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60(1), motorConfig);

    ArmConfig armConfig =
        new ArmConfig()
            .withHardLimits(armJoint.minLimitAngle(), armJoint.maxLimitAngle())
            .withLength(armJoint.length())
            .withTelemetry("Slamtake", TelemetryVerbosity.HIGH);

    this.slamtake = new Arm(armConfig, armSmc);

    SmartMotorControllerConfig rollerMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("IntakeRoller"))
            .withZeroPower(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(60))
            .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH);

    this.rollerSmc =
        new TalonFXWrapper(
            new TalonFX(14, new CANBus(CANPort.CAN_S0)),
            DCMotor.getKrakenX60Foc(1),
            rollerMotorConfig);

    this.intakeRoller =
        new FlyWheel(
            new FlyWheelConfig()
                .withDiameter(rollerJoint.wheelDiameter())
                .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH),
            rollerSmc);
  }

  public Arm getArm() {
    return slamtake;
  }

  public SmartMotorController getRollerMotor() {
    return rollerSmc;
  }

  public void setTargetAngle(Angle angle) {
    slamtake.setMechanismPositionSetpoint(angle);
  }

  public void setRollerVoltage(double volts) {
    rollerSmc.setVoltage(Volts.of(volts));
  }

  /** Coupled pin-in-slot FrontHopper Pose3d derived from the manifest's Slamtake pivot X. */
  public Pose3d getFrontHopperPose3d() {
    double rad = slamtake.getAngle().in(Radians);
    double pivotToWasherX = PIVOT_TO_WASHER_NORM * Math.cos(rad);
    double slotToWasherY = PIVOT_TO_WASHER_NORM * Math.sin(rad) - PIVOT_TO_SLOT_Y_OFFSET;
    double washerToSlotX = slotToWasherY / Math.tan(SLOT_ANGLE_RAD);
    double robotToHopperX =
        robotToPivotX + pivotToWasherX + washerToSlotX + SLOT_START_TO_HOPPER_X;
    return new Pose3d(robotToHopperX, 0.0, 0.37, new Rotation3d());
  }

  public void periodic() {
    slamtake.updateTelemetry();
    intakeRoller.updateTelemetry();
  }

  public void simulationPeriodic() {
    slamtake.simIterate();
    intakeRoller.simIterate();
  }
}
