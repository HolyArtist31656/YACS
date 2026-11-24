// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Port of FRCteam2181/2026-Comp-Code TopIntakeSubsystem to WPILib 2027 Commands v3 + YAMS + YACSim

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

public class TopIntakeMechanism implements Mechanism {
  private final SmartMotorController smc;
  private final FlyWheel topIntake;

  public TopIntakeMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("TopIntake");

    SparkFlex spark = new SparkFlex(CANPort.CAN_S0, CanIds.TOP_INTAKE, MotorType.kBrushless);

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(1.0, 0.0, 0.0)
            .withSimClosedLoopController(1.0, 0.0, 0.0)
            .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0.0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5.0)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("TopIntake"))
            .withMotorInverted(false)
            .withZeroPower(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(60))
            .withTelemetry("TopIntakeMotor", TelemetryVerbosity.HIGH);

    this.smc = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);

    this.topIntake =
        new FlyWheel(
            new FlyWheelConfig()
                .withDiameter(joint.wheelDiameter())
                .withTelemetry("TopIntake", TelemetryVerbosity.HIGH),
            smc);
  }

  public SmartMotorController getMotor() {
    return smc;
  }

  public void setTargetVelocity(AngularVelocity speed) {
    smc.setVelocity(speed);
  }

  public void periodic() {
    topIntake.updateTelemetry();
  }

  public void simulationPeriodic() {
    topIntake.simIterate();
  }
}
