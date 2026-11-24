// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Port of FRCteam2181/2026-Comp-Code InputSubsystem to WPILib 2027 Commands v3 + YAMS + YACSim

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CanIds;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
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
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class InputMechanism implements Mechanism {
  private final SmartMotorController smc;
  private final FlyWheel inputFeeder;

  public InputMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("Input");

    SparkFlex spark = new SparkFlex(CANPort.CAN_S0, CanIds.INPUT_FEEDER, MotorType.kBrushless);

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(0.015, 0.0, 0.0)
            .withSimClosedLoopController(0.015, 0.0, 0.175)
            .withFeedforward(new SimpleMotorFeedforward(0.14, 0.1055, 0.0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.025, 0.011858, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0, 1.0)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("Input"))
            .withMotorInverted(false)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("InputMotor", TelemetryVerbosity.HIGH);

    this.smc = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);

    this.inputFeeder =
        new FlyWheel(
            new FlyWheelConfig()
                .withDiameter(joint.wheelDiameter())
                .withTelemetry("Input", TelemetryVerbosity.HIGH),
            smc);
  }

  public SmartMotorController getMotor() {
    return smc;
  }

  public void setTargetVelocity(AngularVelocity speed) {
    smc.setVelocity(speed);
  }

  public void periodic() {
    inputFeeder.updateTelemetry();
  }

  public void simulationPeriodic() {
    inputFeeder.simIterate();
  }
}
