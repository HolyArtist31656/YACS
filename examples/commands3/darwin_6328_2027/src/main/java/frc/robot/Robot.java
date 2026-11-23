// Copyright (c) Yet Another Software Suite / YACSim Contributors
// FRC 6328 "Darwin" (2026) running on YAMS 2027 (Commands v3) + YACSim

package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import frc.robot.mechanisms.CompactorMechanism;
import frc.robot.mechanisms.DarwinCadSim;
import frc.robot.mechanisms.HoodMechanism;
import frc.robot.mechanisms.SlamtakeMechanism;
import java.io.IOException;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Notifier;

public class Robot extends TimedRobot {
  private final Scheduler scheduler = Scheduler.getDefault();
  private final HoodMechanism hood = new HoodMechanism();
  private final SlamtakeMechanism slamtake = new SlamtakeMechanism();
  private final CompactorMechanism compactor = new CompactorMechanism();
  private final DarwinCadSim darwinCadSim;

  private Notifier simNotifier;
  private double simTimeSeconds = 0.0;
  private Pose2d simulatedFieldPose = new Pose2d(3.0, 4.0, new Rotation2d());

  public Robot() {
    try {
      darwinCadSim = new DarwinCadSim(() -> simulatedFieldPose, hood, slamtake, compactor);
    } catch (IOException e) {
      throw new RuntimeException("Failed to load Darwin CAD meshes. Run setup_darwin_cad.ps1 first.", e);
    }
  }

  @Override
  public void robotPeriodic() {
    hood.periodic();
    slamtake.periodic();
    compactor.periodic();
    darwinCadSim.periodic();
    scheduler.run();
  }

  @Override
  public void simulationInit() {
    simNotifier = new Notifier(this::stepSimulation);
    simNotifier.startPeriodic(0.02);
  }

  private void stepSimulation() {
    simTimeSeconds += 0.02;

    // Gentle field trajectory so Darwin drives smoothly in AdvantageScope 3D Field
    double t = simTimeSeconds * 0.4;
    double x = 4.0 + 1.5 * Math.cos(t);
    double y = 4.0 + 1.0 * Math.sin(2.0 * t);
    double dx = -1.5 * Math.sin(t);
    double dy = 2.0 * Math.cos(2.0 * t);
    simulatedFieldPose = new Pose2d(x, y, new Rotation2d(Math.atan2(dy, dx)));

    // 12-second match cycle driving real YAMS PID + physics loops:
    //   0-3s: Deploy Slamtake (intaking)
    //   3-6s: Retract Slamtake + press Trash Compactor
    //   6-9s: Raise Compactor + aim Launcher Hood up to 38 deg
    //   9-12s: Return Hood to 10 deg stow
    double phase = simTimeSeconds % 12.0;
    if (phase < 3.0) {
      slamtake.setTargetAngle(Degrees.of(22.0));
      compactor.setTargetHeight(CompactorMechanism.DEFAULT_HEIGHT);
      hood.setTargetAngle(HoodMechanism.MIN_ANGLE);
    } else if (phase < 6.0) {
      slamtake.setTargetAngle(Degrees.of(135.0));
      compactor.setTargetHeight(CompactorMechanism.COMPACT_HEIGHT);
      hood.setTargetAngle(Degrees.of(18.0));
    } else if (phase < 9.0) {
      slamtake.setTargetAngle(Degrees.of(140.0));
      compactor.setTargetHeight(Meters.of(0.58));
      hood.setTargetAngle(Degrees.of(38.0));
    } else {
      slamtake.setTargetAngle(Degrees.of(140.0));
      compactor.setTargetHeight(CompactorMechanism.DEFAULT_HEIGHT);
      hood.setTargetAngle(HoodMechanism.MIN_ANGLE);
    }

    hood.simulationPeriodic();
    slamtake.simulationPeriodic();
    compactor.simulationPeriodic();
    darwinCadSim.simulationPeriodic();
  }
}
