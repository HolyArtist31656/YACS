// Copyright (c) Yet Another Software Suite / YACSim Contributors
// YACSim 3D CAD Simulator wiring for FRC 6328 "Darwin" (2026)

package frc.robot.mechanisms;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.system.Filesystem;
import yacsim.commands3.CadRobotMechanism;
import yacsim.core.advantagescope.AdvantageScopeConfigGenerator;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.yams.MotionAxis;
import yacsim.core.yams.YamsMechanismBridge;

public class DarwinCadSim implements Mechanism {
  private final CadRobotMechanism cadSim;

  public DarwinCadSim(
      Supplier<Pose2d> fieldPoseSupplier,
      HoodMechanism hood,
      SlamtakeMechanism slamtake,
      CompactorMechanism compactor,
      FlywheelMechanism flywheel,
      KickerMechanism kicker,
      HopperRollerMechanism hopperRoller)
      throws IOException {
    Path darwinDir = Filesystem.getDeployDirectory().toPath().resolve("yacsim/models/darwin");
    Path ascopeDir =
        Filesystem.getOperatingDirectory().toPath().resolve("advantagescope-assets/Robot_Darwin");

    var slamtakePose =
        YamsMechanismBridge.fromAngleSupplier(
            slamtake::getAngle,
            new Translation3d(Units.inchesToMeters(12.480), 0.0, 0.186),
            MotionAxis.Y,
            true);
    var compactorPose =
        YamsMechanismBridge.fromDistanceSupplier(
            compactor::getHeight, new Translation3d(0.01, 0.0, 0.0), MotionAxis.Z);

    CadRobotConfig robotConfig;
    if (Files.exists(darwinDir.resolve("config.json"))) {
      // One-line import of 6328's AdvantageScope Robot_Darwin/config.json + model*.glb
      robotConfig =
          CadRobotConfig.fromAdvantageScopeDir(darwinDir)
              .withComponentPoseSupplier(0, "Hood", hood::getPose3d)
              .withComponentPoseSupplier(1, "Slamtake", slamtakePose)
              .withComponentPoseSupplier(2, "FrontHopper", slamtake::getFrontHopperPose3d)
              .withComponentPoseSupplier(3, "TrashCompactor", compactorPose);
    } else if (Files.exists(ascopeDir.resolve("config.json"))) {
      robotConfig =
          CadRobotConfig.fromAdvantageScopeDir(ascopeDir)
              .withComponentPoseSupplier(0, "Hood", hood::getPose3d)
              .withComponentPoseSupplier(1, "Slamtake", slamtakePose)
              .withComponentPoseSupplier(2, "FrontHopper", slamtake::getFrontHopperPose3d)
              .withComponentPoseSupplier(3, "TrashCompactor", compactorPose);
    } else {
      robotConfig =
          new CadRobotConfig("Darwin")
              .withChassisAsset(new CadAssetConfig(darwinDir.resolve("model.glb")))
              .addComponent(
                  new CadComponentConfig(
                          "Hood", new CadAssetConfig(darwinDir.resolve("model_0.glb")))
                      .withParentComponent("Chassis")
                      .withPoseSupplier(hood::getPose3d))
              .addComponent(
                  new CadComponentConfig(
                          "Slamtake", new CadAssetConfig(darwinDir.resolve("model_1.glb")))
                      .withParentComponent("Chassis")
                      .withPoseSupplier(slamtakePose))
              .addComponent(
                  new CadComponentConfig(
                          "FrontHopper", new CadAssetConfig(darwinDir.resolve("model_2.glb")))
                      .withParentComponent("Chassis")
                      .withPoseSupplier(slamtake::getFrontHopperPose3d))
              .addComponent(
                  new CadComponentConfig(
                          "TrashCompactor", new CadAssetConfig(darwinDir.resolve("model_3.glb")))
                      .withParentComponent("Chassis")
                      .withPoseSupplier(compactorPose));
    }

    robotConfig
        .withDrivetrainPoseSupplier(fieldPoseSupplier)
        .withExtensionMargin(0.35, 1.85)
        .ignoreRestingOverlaps()
        .addRollerTelemetry(
            "Flywheel", flywheel::getVelocity, FlywheelMechanism.WHEEL_RADIUS_METERS)
        .addRollerTelemetry(
            "KickerFront", kicker::getFrontVelocity, KickerMechanism.FRONT_ROLLER_RADIUS_METERS)
        .addRollerTelemetry(
            "KickerBack", kicker::getBackVelocity, KickerMechanism.BACK_ROLLER_RADIUS_METERS)
        .addRollerTelemetry(
            "HopperRoller", hopperRoller::getVelocity, HopperRollerMechanism.ROLLER_RADIUS_METERS)
        .addRollerTelemetry(
            "IntakeRoller", slamtake::getRollerVelocity, SlamtakeMechanism.ROLLER_RADIUS_METERS)
        .addCamera(
            "FrontCamera",
            new Transform3d(
                new Translation3d(0.28, 0.0, 0.52),
                new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)),
            70.0,
            1280,
            800);

    AdvantageScopeConfigGenerator.exportStarterLayout(
        robotConfig,
        Filesystem.getOperatingDirectory()
            .toPath()
            .resolve("advantagescope-assets/darwin-layout.json"));

    this.cadSim = new CadRobotMechanism(this, robotConfig);
  }

  public void periodic() {
    cadSim.periodic();
  }

  public void simulationPeriodic() {
    cadSim.simulationPeriodic();
  }
}
