// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// FRC Team 2181 GEARS (2026-Comp-Code) ported to WPILib 2027 Commands v3 + YAMS + YACSim

package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import frc.robot.Constants.Setpoints;
import frc.robot.mechanisms.ClimberMechanism;
import frc.robot.mechanisms.GearsCadSim;
import frc.robot.mechanisms.InputMechanism;
import frc.robot.mechanisms.IntakeArmMechanism;
import frc.robot.mechanisms.ShooterMechanism;
import frc.robot.mechanisms.SpindexerMechanism;
import frc.robot.mechanisms.TopIntakeMechanism;
import frc.robot.mechanisms.TurretMechanism;
import java.io.IOException;
import java.nio.file.Files;
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
  private final IntakeArmMechanism intakeArm;
  private final TurretMechanism turret;
  private final ClimberMechanism climber;
  private final ShooterMechanism shooter;
  private final SpindexerMechanism spindexer;
  private final TopIntakeMechanism topIntake;
  private final InputMechanism inputFeeder;
  private final GearsCadSim gearsCadSim;

  private Notifier simNotifier;
  private double simTimeSeconds = 0.0;
  private Pose2d simulatedFieldPose = new Pose2d(3.0, 4.0, new Rotation2d());

  public Robot() {
    try {
      Path liveDir = Filesystem.getDeployDirectory().toPath().resolve("yacsim/gears2181_onshape_live");
      Path manifestDir = Files.exists(liveDir.resolve("yacsim-robot.json")) ? liveDir : Filesystem.getDeployDirectory().toPath().resolve("yacsim/gears2181");
      CadRobotManifest manifest =
          CadManifestLoader.loadJson(manifestDir.resolve("yacsim-robot.json"));

      this.intakeArm = new IntakeArmMechanism(manifest);
      this.turret = new TurretMechanism(manifest);
      this.climber = new ClimberMechanism(manifest);
      this.shooter = new ShooterMechanism(manifest);
      this.spindexer = new SpindexerMechanism(manifest);
      this.topIntake = new TopIntakeMechanism(manifest);
      this.inputFeeder = new InputMechanism(manifest);
      this.gearsCadSim =
          new GearsCadSim(
              manifest,
              manifestDir,
              () -> simulatedFieldPose,
              intakeArm,
              turret,
              climber,
              shooter,
              spindexer,
              topIntake,
              inputFeeder);
    } catch (IOException e) {
      throw new RuntimeException(
          "Failed to load GEARS 2181 manifest or CAD meshes in deploy/yacsim/gears2181/.", e);
    }
  }

  @Override
  public void robotPeriodic() {
    intakeArm.periodic();
    turret.periodic();
    climber.periodic();
    shooter.periodic();
    spindexer.periodic();
    topIntake.periodic();
    inputFeeder.periodic();
    gearsCadSim.periodic();
    scheduler.run();
  }

  @Override
  public void simulationInit() {
    simNotifier = new Notifier(this::stepSimulation);
    simNotifier.startPeriodic(0.02);
  }

  private void stepSimulation() {
    simTimeSeconds += 0.02;

    // Smooth figure-8 field path
    double t = simTimeSeconds * 0.4;
    double x = 4.0 + 1.5 * Math.cos(t);
    double y = 4.0 + 1.0 * Math.sin(2.0 * t);
    double dx = -1.5 * Math.sin(t);
    double dy = 2.0 * Math.cos(2.0 * t);
    simulatedFieldPose = new Pose2d(x, y, new Rotation2d(Math.atan2(dy, dx)));

    // 12-second GEARS match cycle exercising all 7 mechanisms:
    //   0-3s: Deploy IntakeArm (95 deg), spin TopIntake (800 RPM) + Spindexer (400 RPM)
    //   3-6s: Stow IntakeArm (10 deg), aim Turret (+60 deg), spin up Shooter (6000 RPM)
    //   6-9s: Sweep Turret (-45 deg), shoot with Shooter (6000 RPM) + Input (2400 RPM) + Spindexer
    //   9-12s: Center Turret (0 deg), idle Shooter, extend Climber (0.45 m)
    double phase = simTimeSeconds % 12.0;
    if (phase < 3.0) {
      intakeArm.setTargetAngle(Degrees.of(95.0));
      topIntake.setTargetVelocity(Setpoints.TOP_INTAKE_RPM);
      spindexer.setTargetVelocity(Setpoints.SPINDEXER_FEED_RPM);
      inputFeeder.setTargetVelocity(RPM.of(0.0));
      turret.setTargetAngle(Degrees.of(0.0));
      shooter.setTargetVelocity(Setpoints.SHOOTER_IDLE_RPM);
      climber.setTargetHeight(Meters.of(0.0));
    } else if (phase < 6.0) {
      intakeArm.setTargetAngle(Degrees.of(10.0));
      topIntake.setTargetVelocity(RPM.of(0.0));
      spindexer.setTargetVelocity(Setpoints.SPINDEXER_FEED_RPM);
      inputFeeder.setTargetVelocity(RPM.of(0.0));
      turret.setTargetAngle(Degrees.of(60.0));
      shooter.setTargetVelocity(Setpoints.SHOOTER_LAUNCH_RPM);
      climber.setTargetHeight(Meters.of(0.0));
    } else if (phase < 9.0) {
      intakeArm.setTargetAngle(Degrees.of(10.0));
      topIntake.setTargetVelocity(RPM.of(0.0));
      spindexer.setTargetVelocity(Setpoints.SPINDEXER_FEED_RPM);
      inputFeeder.setTargetVelocity(Setpoints.INPUT_FEED_RPM);
      turret.setTargetAngle(Degrees.of(-45.0));
      shooter.setTargetVelocity(Setpoints.SHOOTER_LAUNCH_RPM);
      climber.setTargetHeight(Meters.of(0.0));
    } else {
      intakeArm.setTargetAngle(Degrees.of(5.0));
      topIntake.setTargetVelocity(RPM.of(0.0));
      spindexer.setTargetVelocity(RPM.of(0.0));
      inputFeeder.setTargetVelocity(RPM.of(0.0));
      turret.setTargetAngle(Degrees.of(0.0));
      shooter.setTargetVelocity(Setpoints.SHOOTER_IDLE_RPM);
      climber.setTargetHeight(Meters.of(0.45));
    }

    intakeArm.simulationPeriodic();
    turret.simulationPeriodic();
    climber.simulationPeriodic();
    shooter.simulationPeriodic();
    spindexer.simulationPeriodic();
    topIntake.simulationPeriodic();
    inputFeeder.simulationPeriodic();
    gearsCadSim.simulationPeriodic();
  }
}
