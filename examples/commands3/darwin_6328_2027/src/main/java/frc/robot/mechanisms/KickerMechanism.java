// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Port of FRC 6328 "Darwin" Kicker (Front & Back Rollers) to YAMS 2027 (Commands v3) + YACSim
// Reference: https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
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

public class KickerMechanism implements Mechanism {
  // Exact radii and surface speeds from 6328 Kicker.java & LaunchCalculator.java
  public static final double FRONT_ROLLER_RADIUS_METERS = Inches.of(1.25).in(Meters);
  public static final double BACK_ROLLER_RADIUS_METERS = Inches.of(0.79).in(Meters);
  public static final double LAUNCH_SURFACE_SPEED_MPS = 6.0;

  private final SmartMotorControllerConfig frontMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(1.5, 0.0, 0.0)
          .withSimClosedLoopController(1.5, 0.0, 0.0)
          .withFeedforward(new SimpleMotorFeedforward(0.48, 0.056, 0.0))
          .withSimFeedforward(new SimpleMotorFeedforward(0.48, 0.056, 0.0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
          .withMomentOfInertia(KilogramSquareMeters.of(0.005))
          .withZeroPower(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(60))
          .withTelemetry("KickerFrontMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorControllerConfig backMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(3.0, 0.0, 0.0)
          .withSimClosedLoopController(3.0, 0.0, 0.0)
          .withFeedforward(new SimpleMotorFeedforward(0.40, 0.0575, 0.0))
          .withSimFeedforward(new SimpleMotorFeedforward(0.40, 0.0575, 0.0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
          .withMomentOfInertia(KilogramSquareMeters.of(0.005))
          .withZeroPower(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(60))
          .withTelemetry("KickerBackMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController frontSmc =
      new TalonFXWrapper(
          new TalonFX(16, new CANBus(CANPort.CAN_S0)),
          DCMotor.getKrakenX60Foc(1),
          frontMotorConfig);
  private final SmartMotorController backSmc =
      new TalonFXWrapper(
          new TalonFX(17, new CANBus(CANPort.CAN_S0)),
          DCMotor.getKrakenX60Foc(1),
          backMotorConfig);

  private final FlyWheel frontRoller =
      new FlyWheel(
          new FlyWheelConfig()
              .withDiameter(Inches.of(2.50))
              .withTelemetry("KickerFront", TelemetryVerbosity.HIGH),
          frontSmc);

  private final FlyWheel backRoller =
      new FlyWheel(
          new FlyWheelConfig()
              .withDiameter(Inches.of(1.58))
              .withTelemetry("KickerBack", TelemetryVerbosity.HIGH),
          backSmc);

  /** Commands both kicker rollers to match the requested surface speed (m/s). */
  public void setTargetSurfaceSpeedMps(double surfaceSpeedMps) {
    frontSmc.setVelocity(RadiansPerSecond.of(surfaceSpeedMps / FRONT_ROLLER_RADIUS_METERS));
    backSmc.setVelocity(RadiansPerSecond.of(surfaceSpeedMps / BACK_ROLLER_RADIUS_METERS));
  }

  public AngularVelocity getFrontVelocity() {
    return frontSmc.getMechanismVelocity();
  }

  public AngularVelocity getBackVelocity() {
    return backSmc.getMechanismVelocity();
  }

  public void periodic() {
    frontRoller.updateTelemetry();
    backRoller.updateTelemetry();
  }

  public void simulationPeriodic() {
    frontRoller.simIterate();
    backRoller.simIterate();
  }
}
