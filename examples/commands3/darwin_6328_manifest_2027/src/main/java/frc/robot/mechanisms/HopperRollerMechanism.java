// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Manifest-driven FRC 6328 "Darwin" Hopper Roller (YAMS 2027 Commands v3 + YACSim)

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
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

public class HopperRollerMechanism implements Mechanism {
  public static final double LAUNCH_VOLTS = 12.0;
  public static final double CONSERVATIVE_LAUNCH_VOLTS = 3.0;
  public static final double OUTTAKE_VOLTS = -12.0;

  private final SmartMotorController smc;
  private final FlyWheel hopperRoller;

  public HopperRollerMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("HopperRoller");

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(4.0)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("HopperRoller"))
            .withZeroPower(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(60))
            .withTelemetry("HopperRollerMotor", TelemetryVerbosity.HIGH);

    this.smc =
        new TalonFXWrapper(
            new TalonFX(15, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60Foc(2), motorConfig);

    this.hopperRoller =
        new FlyWheel(
            new FlyWheelConfig()
                .withDiameter(joint.wheelDiameter())
                .withTelemetry("HopperRoller", TelemetryVerbosity.HIGH),
            smc);
  }

  public SmartMotorController getMotor() {
    return smc;
  }

  public void setVoltage(double volts) {
    smc.setVoltage(Volts.of(volts));
  }

  public void periodic() {
    hopperRoller.updateTelemetry();
  }

  public void simulationPeriodic() {
    hopperRoller.simIterate();
  }
}
