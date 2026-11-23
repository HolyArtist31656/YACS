// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.model;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.physics.BoundingBox3d;

class CadRobotSimulatorTest {

  @Test
  void ignoresConfiguredCollisionPairsSymmetrically(@TempDir Path tempDir) throws IOException {
    Path boxMesh = tempDir.resolve("box.stl");
    Files.writeString(boxMesh, cubeStl(0.2, 0.2, 0.2), StandardCharsets.UTF_8);

    Pose3d origin = new Pose3d(new Translation3d(0.0, 0.0, 0.2), new Rotation3d());
    Pose3d offset = new Pose3d(new Translation3d(0.5, 0.0, 0.2), new Rotation3d());

    CadRobotConfig robot =
        new CadRobotConfig("IgnorePairBot")
            .addComponent(
                new CadComponentConfig("Slamtake", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> origin))
            .addComponent(
                new CadComponentConfig("FrontHopper", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> origin))
            .addComponent(
                new CadComponentConfig("Hood", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> offset))
            .addComponent(
                new CadComponentConfig("TrashCompactor", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> offset))
            // Reversed order relative to component registration to verify symmetry
            .ignoreCollisionPair("FrontHopper", "Slamtake");

    try (CadRobotSimulator sim = new CadRobotSimulator(robot, false)) {
      sim.simIterate();
      assertFalse(sim.hasCollisionBetween("Slamtake", "FrontHopper"));
      assertTrue(sim.hasCollisionBetween("Hood", "TrashCompactor"));
    }
  }

  @Test
  void ignoresExtensionEnvelopeForConfiguredComponentOnly(@TempDir Path tempDir)
      throws IOException {
    Path boxMesh = tempDir.resolve("box.stl");
    Files.writeString(boxMesh, cubeStl(0.2, 0.2, 0.2), StandardCharsets.UTF_8);

    Pose3d outside = new Pose3d(new Translation3d(1.0, 0.0, 0.2), new Rotation3d());
    Pose3d inside = new Pose3d(new Translation3d(0.0, 0.0, 0.2), new Rotation3d());

    CadRobotConfig robot =
        new CadRobotConfig("EnvelopeIgnoreBot")
            .withExtensionEnvelope(new BoundingBox3d(-0.5, -0.5, 0.0, 0.5, 0.5, 1.0))
            .addComponent(
                new CadComponentConfig("Slamtake", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> outside))
            .addComponent(
                new CadComponentConfig("Hood", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> inside))
            .ignoreCollisionPair("Slamtake", "ExtensionEnvelope");

    try (CadRobotSimulator sim = new CadRobotSimulator(robot, false)) {
      sim.simIterate();
      assertFalse(sim.hasExtensionViolation());
      assertFalse(sim.hasAnyCollision());
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
