// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Manifest-driven YACSim 3D CAD Simulator wiring for FRC Team 2181 GEARS

package frc.robot.mechanisms;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Filesystem;
import yacsim.commands3.CadRobotMechanism;
import yacsim.commands3.YamsCommands3Bridge;
import yacsim.core.advantagescope.AdvantageScopeConfigGenerator;
import yacsim.core.manifest.CadRobotManifest;
import yacsim.core.model.CadRobotConfig;

/**
 * Wires every FRC Team 2181 GEARS mechanism to YACSim strictly from {@code yacsim-robot.json}.
 * Replace {@code src/main/deploy/yacsim/gears2181/} with your Onshape exporter output to visualize
 * Team 2181's full CAD assembly with zero Java changes.
 */
public class GearsCadSim implements Mechanism {
  private final CadRobotMechanism cadSim;

  public GearsCadSim(
      CadRobotManifest manifest,
      Path manifestDir,
      Supplier<Pose2d> fieldPoseSupplier,
      IntakeArmMechanism intakeArm,
      TurretMechanism turret,
      ClimberMechanism climber,
      ShooterMechanism shooter,
      SpindexerMechanism spindexer,
      TopIntakeMechanism topIntake,
      InputMechanism inputFeeder)
      throws IOException {

    CadRobotConfig robotConfig =
        CadRobotConfig.fromManifest(manifest, manifestDir)
            .withDrivetrainPoseSupplier(fieldPoseSupplier)
            .withAutoInstallToAdvantageScope()
            .withAdvantageScopeExportDir(
                Filesystem.getOperatingDirectory().toPath().resolve("advantagescope-assets"));

    // 1. Bind articulated parent mechanisms first
    if (robotConfig.hasComponent("IntakeArm")) { YamsCommands3Bridge.bindFromManifest(robotConfig, "IntakeArm", intakeArm.getArm()); }
    if (robotConfig.hasComponent("Turret")) { YamsCommands3Bridge.bindFromManifest(robotConfig, "Turret", turret.getPivot()); }
    if (robotConfig.hasComponent("Climber")) { YamsCommands3Bridge.bindFromManifest(robotConfig, "Climber", climber.getElevator()); }

    // 2. Bind all 4 spinning wheels/rollers (TopIntake auto-mounts onto IntakeArm; Shooter auto-mounts onto Turret!)
    YamsCommands3Bridge.bindFromManifest(robotConfig, "Shooter", shooter.getMotor());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "Spindexer", spindexer.getMotor());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "TopIntake", topIntake.getMotor());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "Input", inputFeeder.getMotor());

    AdvantageScopeConfigGenerator.exportStarterLayout(
        robotConfig,
        Filesystem.getOperatingDirectory()
            .toPath()
            .resolve("advantagescope-assets/gears-2181-layout.json"));

    this.cadSim = new CadRobotMechanism(this, robotConfig);
  }

  public void periodic() {
    cadSim.periodic();
  }

  public void simulationPeriodic() {
    cadSim.simulationPeriodic();
  }
}
