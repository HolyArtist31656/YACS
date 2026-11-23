// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Manifest-driven FRC 6328 "Darwin" Trash Compactor (YAMS 2027 Commands v3 + YACSim)

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
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
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Darwin Trash Compactor fed from {@code yacsim-robot.json}: hard/soft travel limits, starting
 * height, drum radius, and carriage mass all come straight from CAD.
 */
public class CompactorMechanism implements Mechanism {
  public static final Distance COMPACT_HEIGHT = Meters.of(0.36);

  private final Elevator compactor;
  private final Distance defaultHeight;

  public CompactorMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("TrashCompactor");
    this.defaultHeight = joint.startingDistance();

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(30.0, 0.0, 0.3)
            .withSimClosedLoopController(30.0, 0.0, 0.3)
            .withTrapezoidalProfile(MetersPerSecond.of(0.8), MetersPerSecondPerSecond.of(1.6))
            .withFeedforward(new ElevatorFeedforward(0.0, 0.25, 0.0, 0.0))
            .withSimFeedforward(new ElevatorFeedforward(0.0, 0.25, 0.0, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withDrumRadius(joint.wheelRadius())
            .withMotorInverted(false)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withStartingPosition(joint.startingDistance())
            .withSoftLimits(joint.softMinLimitDistance(), joint.softMaxLimitDistance())
            .withTelemetry("CompactorMotor", TelemetryVerbosity.HIGH);

    SmartMotorController smc =
        new TalonFXWrapper(
            new TalonFX(13, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60(1), motorConfig);

    ElevatorConfig elevatorConfig =
        new ElevatorConfig()
            .withHardLimits(joint.minLimitDistance(), joint.maxLimitDistance())
            .withCarriageWeight(manifest.getLinkMass("TrashCompactor"))
            .withTelemetry("Compactor", TelemetryVerbosity.HIGH);

    this.compactor = new Elevator(elevatorConfig, smc);
  }

  public Elevator getElevator() {
    return compactor;
  }

  public Distance getDefaultHeight() {
    return defaultHeight;
  }

  public void setTargetHeight(Distance height) {
    compactor.setMeasurementPositionSetpoint(height);
  }

  public void periodic() {
    compactor.updateTelemetry();
  }

  public void simulationPeriodic() {
    compactor.simIterate();
  }
}
