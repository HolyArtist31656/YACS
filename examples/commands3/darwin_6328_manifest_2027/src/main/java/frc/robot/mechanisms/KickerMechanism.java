// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Manifest-driven FRC 6328 "Darwin" Kicker Rollers (YAMS 2027 Commands v3 + YACSim)

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadRobotManifest;
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
  public static final double LAUNCH_SURFACE_SPEED_MPS = 6.0;

  private final double frontRadiusMeters;
  private final double backRadiusMeters;
  private final SmartMotorController frontSmc;
  private final SmartMotorController backSmc;
  private final FlyWheel frontRoller;
  private final FlyWheel backRoller;

  public KickerMechanism(CadRobotManifest manifest) {
    CadJointManifest frontJoint = manifest.getJoint("KickerFront");
    CadJointManifest backJoint = manifest.getJoint("KickerBack");
    this.frontRadiusMeters = frontJoint.wheelRadius().in(Meters);
    this.backRadiusMeters = backJoint.wheelRadius().in(Meters);

    SmartMotorControllerConfig frontMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(1.5, 0.0, 0.0)
            .withSimClosedLoopController(1.5, 0.0, 0.0)
            .withFeedforward(new SimpleMotorFeedforward(0.48, 0.056, 0.0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.48, 0.056, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("KickerFront"))
            .withZeroPower(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(60))
            .withTelemetry("KickerFrontMotor", TelemetryVerbosity.HIGH);

    SmartMotorControllerConfig backMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(3.0, 0.0, 0.0)
            .withSimClosedLoopController(3.0, 0.0, 0.0)
            .withFeedforward(new SimpleMotorFeedforward(0.40, 0.0575, 0.0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.40, 0.0575, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("KickerBack"))
            .withZeroPower(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(60))
            .withTelemetry("KickerBackMotor", TelemetryVerbosity.HIGH);

    this.frontSmc =
        new TalonFXWrapper(
            new TalonFX(16, new CANBus(CANPort.CAN_S0)),
            DCMotor.getKrakenX60Foc(1),
            frontMotorConfig);
    this.backSmc =
        new TalonFXWrapper(
            new TalonFX(17, new CANBus(CANPort.CAN_S0)),
            DCMotor.getKrakenX60Foc(1),
            backMotorConfig);

    this.frontRoller =
        new FlyWheel(
            new FlyWheelConfig()
                .withDiameter(frontJoint.wheelDiameter())
                .withTelemetry("KickerFront", TelemetryVerbosity.HIGH),
            frontSmc);
    this.backRoller =
        new FlyWheel(
            new FlyWheelConfig()
                .withDiameter(backJoint.wheelDiameter())
                .withTelemetry("KickerBack", TelemetryVerbosity.HIGH),
            backSmc);
  }

  public SmartMotorController getFrontMotor() {
    return frontSmc;
  }

  public SmartMotorController getBackMotor() {
    return backSmc;
  }

  public void setTargetSurfaceSpeedMps(double surfaceSpeedMps) {
    frontSmc.setVelocity(RadiansPerSecond.of(surfaceSpeedMps / frontRadiusMeters));
    backSmc.setVelocity(RadiansPerSecond.of(surfaceSpeedMps / backRadiusMeters));
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
