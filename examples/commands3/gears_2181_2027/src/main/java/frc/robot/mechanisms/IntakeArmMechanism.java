// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Port of FRCteam2181/2026-Comp-Code IntakeArmSubsystem to WPILib 2027 Commands v3 + YAMS + YACSim

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.CanIds;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.util.Pair;
import org.wpilib.units.measure.Angle;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadRobotManifest;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class IntakeArmMechanism implements Mechanism {
  private final Arm arm;

  public IntakeArmMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("IntakeArm");

    SparkMax armLeader = new SparkMax(CANPort.CAN_S0, CanIds.INTAKE_ARM_LEADER, MotorType.kBrushless);
    SparkMax armFollower = new SparkMax(CANPort.CAN_S0, CanIds.INTAKE_ARM_FOLLOWER, MotorType.kBrushless);

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(2.5, 0.0, 0.0)
            .withSimClosedLoopController(50.0, 0.0, 0.0)
            .withFeedforward(new ArmFeedforward(0.15, 0.0, 0.0, 0.0))
            .withSimFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("IntakeArm"))
            .withFollowers(Pair.of(armFollower, true))
            .withMotorInverted(true)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withStartingPosition(joint.startingAngle())
            .withSoftLimits(joint.softMinLimitAngle(), joint.softMaxLimitAngle())
            .withTelemetry("IntakeArmMotor", TelemetryVerbosity.HIGH);

    SmartMotorController smc = new SparkWrapper(armLeader, DCMotor.getNEO(2), motorConfig);

    ArmConfig armConfig =
        new ArmConfig()
            .withHardLimits(joint.minLimitAngle(), joint.maxLimitAngle())
            .withLength(joint.length())
            .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH);

    this.arm = new Arm(armConfig, smc);
  }

  public Arm getArm() {
    return arm;
  }

  public void setTargetAngle(Angle angle) {
    arm.setMechanismPositionSetpoint(angle);
  }

  public void periodic() {
    arm.updateTelemetry();
  }

  public void simulationPeriodic() {
    arm.simIterate();
  }
}
