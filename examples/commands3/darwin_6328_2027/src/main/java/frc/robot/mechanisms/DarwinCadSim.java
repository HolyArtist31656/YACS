// Copyright (c) Yet Another Software Suite / YACSim Contributors
// YACSim 3D CAD Simulator wiring for FRC 6328 "Darwin" (2026)

package frc.robot.mechanisms;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.util.Units;
import org.wpilib.system.Filesystem;
import yacsim.commands3.CadRobotMechanism;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.physics.BoundingBox3d;

public class DarwinCadSim implements Mechanism {
  private final CadRobotMechanism cadSim;

  public DarwinCadSim(
      Supplier<Pose2d> fieldPoseSupplier,
      HoodMechanism hood,
      SlamtakeMechanism slamtake,
      CompactorMechanism compactor)
      throws IOException {
    Path darwinDir = Filesystem.getDeployDirectory().toPath().resolve("yacsim/models/darwin");

    // Darwin frame with bumpers: 33.0" (X) x 34.0" (Y) per DriveConstants.java
    double halfX = Units.inchesToMeters(33.0 / 2.0) + 0.35;
    double halfY = Units.inchesToMeters(34.0 / 2.0) + 0.35;

    CadRobotConfig robotConfig =
        new CadRobotConfig("Darwin")
            .withChassisAsset(
                new CadAssetConfig(darwinDir.resolve("model.glb")).withMaxTriangleBudget(5_000_000))
            .withDrivetrainPoseSupplier(fieldPoseSupplier)
            .withExtensionEnvelope(new BoundingBox3d(-halfX, -halfY, 0.0, halfX, halfY, 1.85))
            // Component order matches 6328 Robot_Darwin/config.json: model_0 .. model_3
            .addComponent(
                new CadComponentConfig(
                        "Hood",
                        new CadAssetConfig(darwinDir.resolve("model_0.glb"))
                            .withMaxTriangleBudget(5_000_000))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(hood::getPose3d))
            .addComponent(
                new CadComponentConfig(
                        "Slamtake",
                        new CadAssetConfig(darwinDir.resolve("model_1.glb"))
                            .withMaxTriangleBudget(5_000_000))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(slamtake::getIntakePose3d))
            .addComponent(
                new CadComponentConfig(
                        "FrontHopper",
                        new CadAssetConfig(darwinDir.resolve("model_2.glb"))
                            .withMaxTriangleBudget(5_000_000))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(slamtake::getFrontHopperPose3d))
            .addComponent(
                new CadComponentConfig(
                        "TrashCompactor",
                        new CadAssetConfig(darwinDir.resolve("model_3.glb"))
                            .withMaxTriangleBudget(5_000_000))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(compactor::getPose3d));

    this.cadSim = new CadRobotMechanism(this, robotConfig);
  }

  public void periodic() {
    cadSim.periodic();
  }

  public void simulationPeriodic() {
    cadSim.simulationPeriodic();
  }
}
