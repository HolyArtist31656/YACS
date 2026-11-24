// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Port of FRCteam2181/2026-Comp-Code TurretSubsystem to WPILib 2027 Commands v3 + YAMS + YACSim

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Amps;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CanIds;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadRobotManifest;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Pivot;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class TurretMechanism implements Mechanism {
  private final Pivot turret;

  public TurretMechanism(CadRobotManifest manifest) {
    CadJointManifest joint = manifest.getJoint("Turret");

    SparkFlex turretMotor = new SparkFlex(CANPort.CAN_S0, CanIds.TURRET, MotorType.kBrushless);

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(29.68, 0.0, 2.6489)
            .withSimClosedLoopController(2.596, 0.0, 0.0)
            .withFeedforward(new SimpleMotorFeedforward(0.30397, 4.1323, 0.2806))
            .withSimFeedforward(new SimpleMotorFeedforward(0.45746, 2.1323, 2.2316))
            .withGearing(new MechanismGearing(GearBox.fromStages("4:1", "10:1")))
            .withMomentOfInertia(manifest.getJointMomentOfInertia("Turret"))
            .withMotorInverted(true)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withStartingPosition(joint.startingAngle())
            .withSoftLimits(joint.softMinLimitAngle(), joint.softMaxLimitAngle())
            .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH);

    SmartMotorController smc = new SparkWrapper(turretMotor, DCMotor.getNeoVortex(1), motorConfig);

    PivotConfig pivotConfig =
        new PivotConfig()
            .withHardLimits(joint.minLimitAngle(), joint.maxLimitAngle())
            .withTelemetry("Turret", TelemetryVerbosity.HIGH);

    this.turret = new Pivot(pivotConfig, smc);
  }

  public Pivot getPivot() {
    return turret;
  }

  public void setTargetAngle(Angle angle) {
    turret.setMechanismPositionSetpoint(angle);
  }

  public void periodic() {
    turret.updateTelemetry();
  }

  public void simulationPeriodic() {
    turret.simIterate();
  }
}
