// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Manifest-driven FRC 6328 "Darwin" Launcher Hood (YAMS 2027 Commands v3 + YACSim)

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
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
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Darwin Launcher Hood where all CAD geometry (hard/soft limits, starting angle, length, pivot,
 * axis, and MOI) comes directly from {@code yacsim-robot.json}. Only electrical/control parameters
 * (CAN ID 11, Kraken X60, 20:1 gearing, PID/FF) are specified in Java.
 */
public class HoodMechanism implements Mechanism {
  private final Arm hood;
  private final Angle minSoftAngle;

  public HoodMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("Hood");
    this.minSoftAngle = joint.softMinLimitAngle();

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(45.0, 0.0, 0.5)
            .withSimClosedLoopController(45.0, 0.0, 0.5)
            .withTrapezoidalProfile(DegreesPerSecond.of(160), DegreesPerSecondPerSecond.of(320))
            .withFeedforward(new ArmFeedforward(0.0, 0.15, 0.0, 0.0))
            .withSimFeedforward(new ArmFeedforward(0.0, 0.15, 0.0, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("Hood"))
            .withMotorInverted(false)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withStartingPosition(joint.startingAngle())
            .withSoftLimits(joint.softMinLimitAngle(), joint.softMaxLimitAngle())
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH);

    SmartMotorController smc =
        new TalonFXWrapper(
            new TalonFX(11, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60(1), motorConfig);

    ArmConfig armConfig =
        new ArmConfig()
            .withHardLimits(joint.minLimitAngle(), joint.maxLimitAngle())
            .withLength(joint.length())
            .withTelemetry("Hood", TelemetryVerbosity.HIGH);

    this.hood = new Arm(armConfig, smc);
  }

  public Arm getArm() {
    return hood;
  }

  public Angle getMinSoftAngle() {
    return minSoftAngle;
  }

  public void setTargetAngle(Angle angle) {
    hood.setMechanismPositionSetpoint(angle);
  }

  public void periodic() {
    hood.updateTelemetry();
  }

  public void simulationPeriodic() {
    hood.simIterate();
  }
}
