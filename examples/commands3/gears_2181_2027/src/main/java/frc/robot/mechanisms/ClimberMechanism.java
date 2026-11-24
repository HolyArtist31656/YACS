// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Port of FRCteam2181/2026-Comp-Code ClimberSubsystem upgraded to YAMS 2027 Elevator + YACSim

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.CanIds;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadRobotManifest;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Elevator;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ClimberMechanism implements Mechanism {
  private final Elevator climber;

  public ClimberMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("Climber");

    SparkMax climberMotor = new SparkMax(CANPort.CAN_S0, CanIds.CLIMBER_LEFT, MotorType.kBrushless);

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(25.0, 0.0, 0.2)
            .withSimClosedLoopController(25.0, 0.0, 0.2)
            .withTrapezoidalProfile(MetersPerSecond.of(0.6), MetersPerSecondPerSecond.of(1.2))
            .withFeedforward(new ElevatorFeedforward(0.0, 0.2, 0.0, 0.0))
            .withSimFeedforward(new ElevatorFeedforward(0.0, 0.2, 0.0, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(25.0)))
            .withDrumRadius(joint.wheelRadius())
            .withMotorInverted(false)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withStartingPosition(joint.startingDistance())
            .withSoftLimits(joint.softMinLimitDistance(), joint.softMaxLimitDistance())
            .withTelemetry("ClimberMotor", TelemetryVerbosity.HIGH);

    SmartMotorController smc = new SparkWrapper(climberMotor, DCMotor.getNEO(1), motorConfig);

    ElevatorConfig elevatorConfig =
        new ElevatorConfig()
            .withHardLimits(joint.minLimitDistance(), joint.maxLimitDistance())
            .withCarriageWeight(manifest.getLinkMass("Climber"))
            .withTelemetry("Climber", TelemetryVerbosity.HIGH);

    this.climber = new Elevator(elevatorConfig, smc);
  }

  public Elevator getElevator() {
    return climber;
  }

  public void setTargetHeight(Distance height) {
    climber.setMeasurementPositionSetpoint(height);
  }

  public void periodic() {
    climber.updateTelemetry();
  }

  public void simulationPeriodic() {
    climber.simIterate();
  }
}
