// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Port of FRC 6328 "Darwin" Hopper Roller to YAMS 2027 (Commands v3) + YACSim
// Reference: https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class HopperRollerMechanism implements Mechanism {
  // Exact SIMBOT constants from 6328 Hopper.java & RobotContainer.java
  public static final double LAUNCH_VOLTS = 12.0;
  public static final double CONSERVATIVE_LAUNCH_VOLTS = 3.0;
  public static final double OUTTAKE_VOLTS = -12.0;
  public static final double ROLLER_RADIUS_METERS = Inches.of(1.0).in(Meters);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.OPEN_LOOP)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(4.0)))
          .withMomentOfInertia(KilogramSquareMeters.of(0.005))
          .withZeroPower(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(60))
          .withTelemetry("HopperRollerMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController smc =
      new TalonFXWrapper(
          new TalonFX(15, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60Foc(2), motorConfig);

  private final FlyWheel hopperRoller =
      new FlyWheel(
          new FlyWheelConfig()
              .withDiameter(Inches.of(2.0))
              .withTelemetry("HopperRoller", TelemetryVerbosity.HIGH),
          smc);

  public void setVoltage(double volts) {
    smc.setVoltage(Volts.of(volts));
  }

  public AngularVelocity getVelocity() {
    return smc.getMechanismVelocity();
  }

  public void periodic() {
    hopperRoller.updateTelemetry();
  }

  public void simulationPeriodic() {
    hopperRoller.simIterate();
  }
}
