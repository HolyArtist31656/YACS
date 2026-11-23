// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Port of FRC 6328 "Darwin" Trash Compactor to YAMS 2027 (Commands v3) + YACSim
// Reference: https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Pounds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Elevator;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class CompactorMechanism implements Mechanism {
  public static final Distance DEFAULT_HEIGHT = Meters.of(0.554692);
  public static final Distance COMPACT_HEIGHT = Meters.of(0.36);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(30.0, 0.0, 0.3)
          .withSimClosedLoopController(30.0, 0.0, 0.3)
          .withTrapezoidalProfile(MetersPerSecond.of(0.8), MetersPerSecondPerSecond.of(1.6))
          .withFeedforward(new ElevatorFeedforward(0.0, 0.25, 0.0, 0.0))
          .withSimFeedforward(new ElevatorFeedforward(0.0, 0.25, 0.0, 0.0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withDrumRadius(Inches.of(1.0))
          .withMotorInverted(false)
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withStartingPosition(DEFAULT_HEIGHT)
          .withSoftLimits(Meters.of(0.32), Meters.of(0.62))
          .withTelemetry("CompactorMotor", TelemetryVerbosity.HIGH);

  private final TalonFX compactorMotor = new TalonFX(13, new CANBus(CANPort.CAN_S0));
  private final SmartMotorController smc =
      new TalonFXWrapper(compactorMotor, DCMotor.getKrakenX60(1), motorConfig);

  private final ElevatorConfig elevatorConfig =
      new ElevatorConfig()
          .withHardLimits(Meters.of(0.30), Meters.of(0.65))
          .withCarriageWeight(Pounds.of(6.0))
          .withTelemetry("Compactor", TelemetryVerbosity.HIGH);

  private final Elevator compactor = new Elevator(elevatorConfig, smc);

  public void setTargetHeight(Distance height) {
    compactor.setMeasurementPositionSetpoint(height);
  }

  public Distance getHeight() {
    return compactor.getHeight();
  }

  /** Exact AdvantageScope Pose3d for Trash Compactor (model_3.glb). */
  public Pose3d getPose3d() {
    return new Pose3d(0.01, 0.0, getHeight().in(Meters), new Rotation3d());
  }

  public void periodic() {
    compactor.updateTelemetry();
  }

  public void simulationPeriodic() {
    compactor.simIterate();
  }
}
