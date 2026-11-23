// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Port of FRC 6328 "Darwin" Slamtake & coupled Front Hopper to YAMS 2027 (Commands v3) + YACSim
// Reference: https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Volts;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Radians;

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
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class SlamtakeMechanism implements Mechanism {
  // Exact 6328 Darwin Slam.java & DarwinMechanism3d.java geometry
  public static final Angle MIN_ANGLE = Degrees.of(17.4088627799);
  public static final Angle MAX_ANGLE = Degrees.of(145.8170667799);
  private static final double ROBOT_TO_PIVOT_X = Units.inchesToMeters(12.480);
  private static final double PIVOT_TO_SLOT_Y_OFFSET = Units.inchesToMeters(2.393);
  private static final double PIVOT_TO_WASHER_NORM = Units.inchesToMeters(7.204);
  private static final double SLOT_START_TO_HOPPER_X = Units.inchesToMeters(6.789);
  private static final double SLOT_ANGLE_RAD = Math.toRadians(70.0);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(35.0, 0.0, 0.4)
          .withSimClosedLoopController(35.0, 0.0, 0.4)
          .withTrapezoidalProfile(DegreesPerSecond.of(220), DegreesPerSecondPerSecond.of(440))
          .withFeedforward(new ArmFeedforward(0.0, 0.25, 0.0, 0.0))
          .withSimFeedforward(new ArmFeedforward(0.0, 0.25, 0.0, 0.0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5)))
          .withMomentOfInertia(Meters.of(0.32), Pounds.of(8.0))
          .withMotorInverted(false)
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withStartingPosition(Degrees.of(140.0))
          .withSoftLimits(MIN_ANGLE, MAX_ANGLE)
          .withTelemetry("SlamtakeMotor", TelemetryVerbosity.HIGH);

  private final TalonFX slamMotor = new TalonFX(12, new CANBus(CANPort.CAN_S0));
  private final SmartMotorController smc =
      new TalonFXWrapper(slamMotor, DCMotor.getKrakenX60(1), motorConfig);

  private final ArmConfig armConfig =
      new ArmConfig()
          .withHardLimits(Degrees.of(15.0), Degrees.of(148.0))
          .withLength(Meters.of(0.32))
          .withTelemetry("Slamtake", TelemetryVerbosity.HIGH);

  // Exact intake roller constants from 6328 Slamtake.java & RobotContainer.java (SIMBOT)
  public static final double ROLLER_INTAKE_VOLTS = 13.0;
  public static final double ROLLER_OUTTAKE_VOLTS = -6.0;
  public static final double ROLLER_RADIUS_METERS = Inches.of(1.0).in(Meters);

  private final Arm slamtake = new Arm(armConfig, smc);

  private final SmartMotorControllerConfig rollerMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.OPEN_LOOP)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
          .withMomentOfInertia(KilogramSquareMeters.of(0.005))
          .withZeroPower(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(60))
          .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController rollerSmc =
      new TalonFXWrapper(
          new TalonFX(14, new CANBus(CANPort.CAN_S0)),
          DCMotor.getKrakenX60Foc(1),
          rollerMotorConfig);

  private final FlyWheel intakeRoller =
      new FlyWheel(
          new FlyWheelConfig()
              .withDiameter(Inches.of(2.0))
              .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH),
          rollerSmc);

  public void setTargetAngle(Angle angle) {
    slamtake.setMechanismPositionSetpoint(angle);
  }

  public Angle getAngle() {
    return slamtake.getAngle();
  }

  /** Exact AdvantageScope Pose3d for Slamtake (model_1.glb). */
  public Pose3d getIntakePose3d() {
    double rad = getAngle().in(Radians);
    return new Pose3d(ROBOT_TO_PIVOT_X, 0.0, 0.186, new Rotation3d(0.0, -rad, 0.0));
  }

  /** Exact pin-in-slot coupled Front Hopper Pose3d (model_2.glb) from DarwinMechanism3d.java. */
  public Pose3d getFrontHopperPose3d() {
    double rad = getAngle().in(Radians);
    double pivotToWasherX = PIVOT_TO_WASHER_NORM * Math.cos(rad);
    double slotToWasherY = PIVOT_TO_WASHER_NORM * Math.sin(rad) - PIVOT_TO_SLOT_Y_OFFSET;
    double washerToSlotX = slotToWasherY / Math.tan(SLOT_ANGLE_RAD);
    double robotToHopperX =
        ROBOT_TO_PIVOT_X + pivotToWasherX + washerToSlotX + SLOT_START_TO_HOPPER_X;
    return new Pose3d(robotToHopperX, 0.0, 0.37, new Rotation3d());
  }

  public void setRollerVoltage(double volts) {
    rollerSmc.setVoltage(Volts.of(volts));
  }

  public AngularVelocity getRollerVelocity() {
    return rollerSmc.getMechanismVelocity();
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
