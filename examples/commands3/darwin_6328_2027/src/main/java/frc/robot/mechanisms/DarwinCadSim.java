// Copyright (c) Yet Another Software Suite / YACSim Contributors
// YACSim 3D CAD Simulator wiring for FRC 6328 "Darwin" (2026)

package frc.robot.mechanisms;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.system.Filesystem;
import yacsim.commands3.CadRobotMechanism;
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
      CompactorMechanism compactor)
      throws IOException {
    Path darwinDir = Filesystem.getDeployDirectory().toPath().resolve("yacsim/models/darwin");

    CadRobotConfig robotConfig =
        new CadRobotConfig("Darwin")
            .withChassisAsset(new CadAssetConfig(darwinDir.resolve("model.glb")))
            .withDrivetrainPoseSupplier(fieldPoseSupplier)
            // Auto-derive legal FRC extension envelope directly from model.glb chassis bounds
            .withExtensionMargin(0.35, 1.85)
            // Auto-silence resting coupled-linkage OBB overlaps at t = 0 (never ignores envelope)
            .ignoreRestingOverlaps()
            // Component order matches 6328 Robot_Darwin/config.json: model_0 .. model_3
            .addComponent(
                new CadComponentConfig("Hood", new CadAssetConfig(darwinDir.resolve("model_0.glb")))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(hood::getPose3d))
            .addComponent(
                new CadComponentConfig(
                        "Slamtake", new CadAssetConfig(darwinDir.resolve("model_1.glb")))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(
                        YamsMechanismBridge.fromAngleSupplier(
                            slamtake::getAngle,
                            new Translation3d(Units.inchesToMeters(12.480), 0.0, 0.186),
                            MotionAxis.Y,
                            true)))
            .addComponent(
                new CadComponentConfig(
                        "FrontHopper", new CadAssetConfig(darwinDir.resolve("model_2.glb")))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(slamtake::getFrontHopperPose3d))
            .addComponent(
                new CadComponentConfig(
                        "TrashCompactor", new CadAssetConfig(darwinDir.resolve("model_3.glb")))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(
                        YamsMechanismBridge.fromDistanceSupplier(
                            compactor::getHeight,
                            new Translation3d(0.01, 0.0, 0.0),
                            MotionAxis.Z)));

    this.cadSim = new CadRobotMechanism(this, robotConfig);
  }

  public void periodic() {
    cadSim.periodic();
  }

  public void simulationPeriodic() {
    cadSim.simulationPeriodic();
  }
}