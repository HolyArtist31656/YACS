// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Manifest-driven FRC 6328 "Darwin" running on YAMS 2027 (Commands v3) + YACSim

package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import frc.robot.mechanisms.CompactorMechanism;
import frc.robot.mechanisms.DarwinManifestCadSim;
import frc.robot.mechanisms.FlywheelMechanism;
import frc.robot.mechanisms.HoodMechanism;
import frc.robot.mechanisms.HopperRollerMechanism;
import frc.robot.mechanisms.KickerMechanism;
import frc.robot.mechanisms.SlamtakeMechanism;
import java.io.IOException;
import java.nio.file.Path;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Filesystem;
import org.wpilib.system.Notifier;
import yacsim.core.manifest.CadManifestLoader;
import yacsim.core.manifest.CadRobotManifest;

public class Robot extends TimedRobot {
  private final Scheduler scheduler = Scheduler.getDefault();
  private final HoodMechanism hood;
  private final SlamtakeMechanism slamtake;
  private final CompactorMechanism compactor;
  private final FlywheelMechanism flywheel;
  private final KickerMechanism kicker;
  private final HopperRollerMechanism hopperRoller;
  private final DarwinManifestCadSim darwinCadSim;

  private Notifier simNotifier;
  private double simTimeSeconds = 0.0;
  private Pose2d simulatedFieldPose = new Pose2d(3.0, 4.0, new Rotation2d());

  public Robot() {
    try {
      Path manifestDir = Filesystem.getDeployDirectory().toPath().resolve("yacsim/darwin");
      CadRobotManifest manifest =
          CadManifestLoader.loadJson(manifestDir.resolve("yacsim-robot.json"));

      this.hood = new HoodMechanism(manifest);
      this.slamtake = new SlamtakeMechanism(manifest);
      this.compactor = new CompactorMechanism(manifest);
      this.flywheel = new FlywheelMechanism(manifest);
      this.kicker = new KickerMechanism(manifest);
      this.hopperRoller = new HopperRollerMechanism(manifest);
      this.darwinCadSim =
          new DarwinManifestCadSim(
              manifest,
              manifestDir,
              () -> simulatedFieldPose,
              hood,
              slamtake,
              compactor,
              flywheel,
              kicker,
              hopperRoller);
    } catch (IOException e) {
      throw new RuntimeException(
          "Failed to load Darwin manifest or CAD meshes. Run setup_darwin_manifest_cad.ps1 first.",
          e);
    }
  }

  @Override
  public void robotPeriodic() {
    hood.periodic();
    slamtake.periodic();
    compactor.periodic();
    flywheel.periodic();
    kicker.periodic();
    hopperRoller.periodic();
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

    double t = simTimeSeconds * 0.4;
    double x = 4.0 + 1.5 * Math.cos(t);
    double y = 4.0 + 1.0 * Math.sin(2.0 * t);
    double dx = -1.5 * Math.sin(t);
    double dy = 2.0 * Math.cos(2.0 * t);
    simulatedFieldPose = new Pose2d(x, y, new Rotation2d(Math.atan2(dy, dx)));

    double phase = simTimeSeconds % 12.0;
    if (phase < 3.0) {
      slamtake.setTargetAngle(Degrees.of(22.0));
      slamtake.setRollerVoltage(SlamtakeMechanism.ROLLER_INTAKE_VOLTS);
      compactor.setTargetHeight(compactor.getDefaultHeight());
      hood.setTargetAngle(hood.getMinSoftAngle());
      flywheel.setTargetVelocity(FlywheelMechanism.IDLE_SPEED);
      kicker.setTargetSurfaceSpeedMps(0.0);
      hopperRoller.setVoltage(HopperRollerMechanism.CONSERVATIVE_LAUNCH_VOLTS);
    } else if (phase < 6.0) {
      slamtake.setTargetAngle(Degrees.of(135.0));
      slamtake.setRollerVoltage(0.0);
      compactor.setTargetHeight(CompactorMechanism.COMPACT_HEIGHT);
      hood.setTargetAngle(Degrees.of(18.0));
      flywheel.setTargetVelocity(FlywheelMechanism.LAUNCH_SPEED);
      kicker.setTargetSurfaceSpeedMps(0.0);
      hopperRoller.setVoltage(HopperRollerMechanism.CONSERVATIVE_LAUNCH_VOLTS);
    } else if (phase < 9.0) {
      slamtake.setTargetAngle(Degrees.of(140.0));
      slamtake.setRollerVoltage(0.0);
      compactor.setTargetHeight(Meters.of(0.58));
      hood.setTargetAngle(Degrees.of(38.0));
      flywheel.setTargetVelocity(FlywheelMechanism.LAUNCH_SPEED);
      kicker.setTargetSurfaceSpeedMps(KickerMechanism.LAUNCH_SURFACE_SPEED_MPS);
      hopperRoller.setVoltage(HopperRollerMechanism.LAUNCH_VOLTS);
    } else {
      slamtake.setTargetAngle(Degrees.of(140.0));
      slamtake.setRollerVoltage(0.0);
      compactor.setTargetHeight(compactor.getDefaultHeight());
      hood.setTargetAngle(hood.getMinSoftAngle());
      flywheel.setTargetVelocity(FlywheelMechanism.IDLE_SPEED);
      kicker.setTargetSurfaceSpeedMps(0.0);
      hopperRoller.setVoltage(0.0);
    }

    hood.simulationPeriodic();
    slamtake.simulationPeriodic();
    compactor.simulationPeriodic();
    flywheel.simulationPeriodic();
    kicker.simulationPeriodic();
    hopperRoller.simulationPeriodic();
    darwinCadSim.simulationPeriodic();
  }
}
