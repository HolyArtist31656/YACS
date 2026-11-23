// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Port of FRC 6328 "Darwin" Launcher Flywheel to YAMS 2027 (Commands v3) + YACSim
// Reference: https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
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

public class FlywheelMechanism implements Mechanism {
  // Exact constants from 6328 Flywheel.java & LaunchCalculator.java
  public static final AngularVelocity IDLE_SPEED = RadiansPerSecond.of(80.0);
  public static final AngularVelocity LAUNCH_SPEED = RadiansPerSecond.of(175.0);
  // Representative 4.0 in diameter (not stated in 6328's Java subsystem)
  public static final double WHEEL_RADIUS_METERS = Inches.of(2.0).in(org.wpilib.units.Units.Meters);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(0.6, 0.0, 0.0)
          .withSimClosedLoopController(0.6, 0.0, 0.0)
          .withFeedforward(new SimpleMotorFeedforward(0.28, 0.0192, 0.007))
          .withSimFeedforward(new SimpleMotorFeedforward(0.28, 0.0192, 0.007))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
          .withMomentOfInertia(KilogramSquareMeters.of(0.02))
          .withZeroPower(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(80))
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH);

  private final TalonFX motor = new TalonFX(18, new CANBus(CANPort.CAN_S0));
  private final SmartMotorController smc =
      new TalonFXWrapper(motor, DCMotor.getKrakenX60Foc(4), motorConfig);

  private final FlyWheelConfig flyWheelConfig =
      new FlyWheelConfig()
          .withDiameter(Inches.of(4.0))
          .withTelemetry("Flywheel", TelemetryVerbosity.HIGH);

  private final FlyWheel flywheel = new FlyWheel(flyWheelConfig, smc);

  public void setTargetVelocity(AngularVelocity speed) {
    smc.setVelocity(speed);
  }

  public AngularVelocity getVelocity() {
    return smc.getMechanismVelocity();
  }

  public void periodic() {
    flywheel.updateTelemetry();
  }

  public void simulationPeriodic() {
    flywheel.simIterate();
  }
}
