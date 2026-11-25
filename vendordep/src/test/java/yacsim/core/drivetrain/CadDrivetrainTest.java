// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.model.CadRobotSimulator;

class CadDrivetrainTest {

  @Test
  void simulatesYamsSwerveDrivetrainAndChassisCollision(@TempDir Path tempDir) throws IOException {
    Path chassisMesh = tempDir.resolve("chassis.stl");
    Path moduleMesh = tempDir.resolve("module.stl");
    Path armMesh = tempDir.resolve("arm.stl");
    Files.writeString(chassisMesh, cubeStl(0.7, 0.7, 0.15), StandardCharsets.UTF_8);
    Files.writeString(moduleMesh, cubeStl(0.1, 0.1, 0.1), StandardCharsets.UTF_8);
    Files.writeString(armMesh, cubeStl(0.2, 0.1, 0.1), StandardCharsets.UTF_8);

    SwerveModulePosition[] positions = {
      new SwerveModulePosition(0.0508 * Math.PI, Rotation2d.fromDegrees(45.0)),
      new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0.0)),
      new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0.0)),
      new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0.0))
    };

    List<Translation3d> corners =
        List.of(
            new Translation3d(0.3, 0.3, 0.05),
            new Translation3d(0.3, -0.3, 0.05),
            new Translation3d(-0.3, 0.3, 0.05),
            new Translation3d(-0.3, -0.3, 0.05));

    CadDrivetrainConfig drivetrain =
        YamsDrivetrainBridge.fromYamsSwerve(
            new CadAssetConfig(chassisMesh),
            new CadAssetConfig(moduleMesh),
            new CadAssetConfig(moduleMesh),
            0.0508,
            corners,
            () -> new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(90.0)),
            () -> new Rotation3d(Math.toRadians(3.0), Math.toRadians(-2.0), Math.toRadians(90.0)),
            () -> positions);

    AtomicReference<Pose3d> armPose =
        new AtomicReference<>(new Pose3d(new Translation3d(0.2, 0.2, 0.4), new Rotation3d()));

    CadRobotConfig robot =
        new CadRobotConfig("SwerveCADBot")
            .withDrivetrain(drivetrain)
            .addComponent(
                new CadComponentConfig("Arm", new CadAssetConfig(armMesh))
                    .withPoseSupplier(armPose::get));

    try (CadRobotSimulator sim = new CadRobotSimulator(robot, false)) {
      sim.simIterate();
      Pose3d[] poses = sim.getLatestComponentPoses();
      assertEquals(9, poses.length);
      assertEquals(Math.toRadians(45.0), poses[0].getRotation().getZ(), 1e-6);
      assertEquals(2.0, sim.getLatestRobotPose3d().getX(), 1e-6);
      assertEquals(Math.toRadians(3.0), sim.getLatestRobotPose3d().getRotation().getX(), 1e-6);
      assertFalse(sim.hasAnyCollision());

      armPose.set(new Pose3d(new Translation3d(0.2, 0.2, 0.05), new Rotation3d()));
      sim.simIterate();
      assertTrue(sim.hasCollisionBetween("Arm", "Chassis"));
    }
  }

  private static String cubeStl(double sx, double sy, double sz) {
    double[][] v = {
      {0, 0, 0}, {sx, 0, 0}, {sx, sy, 0}, {0, sy, 0},
      {0, 0, sz}, {sx, 0, sz}, {sx, sy, sz}, {0, sy, sz}
    };
    int[][] tris = {
      {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
      {0, 1, 5}, {0, 5, 4}, {2, 3, 7}, {2, 7, 6},
      {1, 2, 6}, {1, 6, 5}, {3, 0, 4}, {3, 4, 7}
    };
    StringBuilder sb = new StringBuilder("solid c\n");
    for (int[] t : tris) {
      sb.append("facet normal 0 0 0\n  outer loop\n");
      for (int idx : t) {
        sb.append(String.format("    vertex %f %f %f\n", v[idx][0], v[idx][1], v[idx][2]));
      }
      sb.append("  endloop\nendfacet\n");
    }
    return sb.append("endsolid c\n").toString();
  }
}
