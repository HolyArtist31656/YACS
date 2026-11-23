// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Manifest-driven YACSim 3D CAD Simulator wiring for FRC 6328 "Darwin"

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
 * Wires every Darwin mechanism to YACSim strictly via {@code yacsim-robot.json} with zero
 * hardcoded pivot coordinates, rotation axes, wheel radii, or camera transforms in Java.
 */
public class DarwinManifestCadSim implements Mechanism {
  private final CadRobotMechanism cadSim;

  public DarwinManifestCadSim(
      CadRobotManifest manifest,
      Path manifestDir,
      Supplier<Pose2d> fieldPoseSupplier,
      HoodMechanism hood,
      SlamtakeMechanism slamtake,
      CompactorMechanism compactor,
      FlywheelMechanism flywheel,
      KickerMechanism kicker,
      HopperRollerMechanism hopperRoller)
      throws IOException {

    CadRobotConfig robotConfig =
        CadRobotConfig.fromManifest(manifest, manifestDir)
            .withDrivetrainPoseSupplier(fieldPoseSupplier)
            .withAutoInstallToAdvantageScope()
            .withAdvantageScopeExportDir(
                Filesystem.getOperatingDirectory().toPath().resolve("advantagescope-assets"));

    // Bind articulated links directly from manifest joint origins, axes, and inversions
    YamsCommands3Bridge.bindFromManifest(robotConfig, "Hood", hood.getArm());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "Slamtake", slamtake.getArm());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "TrashCompactor", compactor.getElevator());

    // Coupled pin-in-slot FrontHopper bound by link name
    robotConfig.withComponentPoseSupplier("FrontHopper", slamtake::getFrontHopperPose3d);

    // Bind all 5 spinning roller/flywheel telemetry channels using manifest wheel radii
    YamsCommands3Bridge.bindFromManifest(robotConfig, "Flywheel", flywheel.getMotor());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "KickerFront", kicker.getFrontMotor());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "KickerBack", kicker.getBackMotor());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "HopperRoller", hopperRoller.getMotor());
    YamsCommands3Bridge.bindFromManifest(robotConfig, "IntakeRoller", slamtake.getRollerMotor());

    AdvantageScopeConfigGenerator.exportStarterLayout(
        robotConfig,
        Filesystem.getOperatingDirectory()
            .toPath()
            .resolve("advantagescope-assets/darwin-manifest-layout.json"));

    this.cadSim = new CadRobotMechanism(this, robotConfig);
  }

  public void periodic() {
    cadSim.periodic();
  }

  public void simulationPeriodic() {
    cadSim.simulationPeriodic();
  }
}
