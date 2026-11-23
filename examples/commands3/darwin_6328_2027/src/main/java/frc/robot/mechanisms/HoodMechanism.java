// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Port of FRC 6328 "Darwin" Launcher Hood to YAMS 2027 (Commands v3) + YACSim
// Reference: https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class HoodMechanism implements Mechanism {
  // Exact 6328 Darwin LauncherConstants.robotToLauncher & Hood.java limits
  public static final Transform3d ROBOT_TO_LAUNCHER =
      new Transform3d(-0.151, 0.0, 0.486, new Rotation3d(0.0, 0.0, Math.PI));
  public static final Angle MIN_ANGLE = Degrees.of(10.0);
  public static final Angle MAX_ANGLE = Degrees.of(41.2);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(45.0, 0.0, 0.5)
          .withSimClosedLoopController(45.0, 0.0, 0.5)
          .withTrapezoidalProfile(DegreesPerSecond.of(160), DegreesPerSecondPerSecond.of(320))
          .withFeedforward(new ArmFeedforward(0.0, 0.15, 0.0, 0.0))
          .withSimFeedforward(new ArmFeedforward(0.0, 0.15, 0.0, 0.0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
          .withMomentOfInertia(Meters.of(0.18), Pounds.of(3.0))
          .withMotorInverted(false)
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withStartingPosition(MIN_ANGLE)
          .withSoftLimits(MIN_ANGLE, MAX_ANGLE)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH);

  private final TalonFX hoodMotor = new TalonFX(11, new CANBus(CANPort.CAN_S0));
  private final SmartMotorController smc =
      new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX60(1), motorConfig);

  private final ArmConfig armConfig =
      new ArmConfig()
          .withHardLimits(Degrees.of(9.0), Degrees.of(42.0))
          .withLength(Meters.of(0.18))
          .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  private final Arm hood = new Arm(armConfig, smc);

  public void setTargetAngle(Angle angle) {
    hood.setMechanismPositionSetpoint(angle);
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  /** Exact AdvantageScope Pose3d from 6328 DarwinMechanism3d.java (model_0.glb). */
  public Pose3d getPose3d() {
    double rad = getAngle().in(org.wpilib.units.Units.Radians);
    return new Pose3d()
        .transformBy(ROBOT_TO_LAUNCHER)
        .transformBy(new Transform3d(0.122, 0.0, 0.0, new Rotation3d(0.0, -rad, Math.PI)));
  }

  public void periodic() {
    hood.updateTelemetry();
  }

  public void simulationPeriodic() {
    hood.simIterate();
  }
}
