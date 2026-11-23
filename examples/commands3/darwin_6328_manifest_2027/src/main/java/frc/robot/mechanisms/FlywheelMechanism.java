// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Manifest-driven FRC 6328 "Darwin" Launcher Flywheel (YAMS 2027 Commands v3 + YACSim)

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
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
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class FlywheelMechanism implements Mechanism {
  public static final AngularVelocity IDLE_SPEED = RadiansPerSecond.of(80.0);
  public static final AngularVelocity LAUNCH_SPEED = RadiansPerSecond.of(175.0);

  private final SmartMotorController smc;
  private final FlyWheel flywheel;

  public FlywheelMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("Flywheel");

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(0.6, 0.0, 0.0)
            .withSimClosedLoopController(0.6, 0.0, 0.0)
            .withFeedforward(new SimpleMotorFeedforward(0.28, 0.0192, 0.007))
            .withSimFeedforward(new SimpleMotorFeedforward(0.28, 0.0192, 0.007))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0)))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("Flywheel"))
            .withZeroPower(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(80))
            .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH);

    this.smc =
        new TalonFXWrapper(
            new TalonFX(18, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60Foc(4), motorConfig);

    this.flywheel =
        new FlyWheel(
            new FlyWheelConfig()
                .withDiameter(joint.wheelDiameter())
                .withTelemetry("Flywheel", TelemetryVerbosity.HIGH),
            smc);
  }

  public SmartMotorController getMotor() {
    return smc;
  }

  public void setTargetVelocity(AngularVelocity speed) {
    smc.setVelocity(speed);
  }

  public void periodic() {
    flywheel.updateTelemetry();
  }

  public void simulationPeriodic() {
    flywheel.simIterate();
  }
}
