// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.model;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.util.MinimalJson;

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

  @Test
  void autoDerivesExtensionEnvelopeFromChassisBounds(@TempDir Path tempDir) throws IOException {
    Path chassisMesh = tempDir.resolve("chassis.stl");
    Path armMesh = tempDir.resolve("arm.stl");
    // Chassis spans [0, 0.8] x [0, 0.8] x [0, 0.2]
    Files.writeString(chassisMesh, cubeStl(0.8, 0.8, 0.2), StandardCharsets.UTF_8);
    Files.writeString(armMesh, cubeStl(0.2, 0.2, 0.2), StandardCharsets.UTF_8);

    AtomicReference<Pose3d> armPose =
        new AtomicReference<>(new Pose3d(new Translation3d(0.3, 0.3, 0.5), new Rotation3d()));

    CadRobotConfig robot =
        new CadRobotConfig("AutoEnvelopeBot")
            .withChassisAsset(new CadAssetConfig(chassisMesh))
            .withExtensionMargin(0.30, 1.50)
            .addComponent(
                new CadComponentConfig("Arm", new CadAssetConfig(armMesh))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(armPose::get));

    try (CadRobotSimulator sim = new CadRobotSimulator(robot, false)) {
      BoundingBox3d envelope = sim.getResolvedExtensionEnvelope().orElseThrow();
      assertEquals(-0.30, envelope.minX(), 1e-6);
      assertEquals(1.10, envelope.maxX(), 1e-6);
      assertEquals(1.50, envelope.maxZ(), 1e-6);

      sim.simIterate();
      assertFalse(sim.hasExtensionViolation());

      // Extend past maxX = 1.10 m
      armPose.set(new Pose3d(new Translation3d(1.05, 0.3, 0.5), new Rotation3d()));
      sim.simIterate();
      assertTrue(sim.hasExtensionViolation());
    }
  }

  @Test
  void autoIgnoresRestingSiblingOverlapsWhileDetectingNewDynamicCollisions(@TempDir Path tempDir)
      throws IOException {
    Path boxMesh = tempDir.resolve("box.stl");
    Files.writeString(boxMesh, cubeStl(0.2, 0.2, 0.2), StandardCharsets.UTF_8);

    Pose3d restingOrigin = new Pose3d(new Translation3d(0.0, 0.0, 0.3), new Rotation3d());
    AtomicReference<Pose3d> hoodPose =
        new AtomicReference<>(new Pose3d(new Translation3d(0.8, 0.0, 0.3), new Rotation3d()));

    CadRobotConfig robot =
        new CadRobotConfig("RestingOverlapBot")
            .ignoreRestingOverlaps()
            .addComponent(
                new CadComponentConfig("Slamtake", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> restingOrigin))
            .addComponent(
                new CadComponentConfig("FrontHopper", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> restingOrigin))
            .addComponent(
                new CadComponentConfig("Hood", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(hoodPose::get));

    try (CadRobotSimulator sim = new CadRobotSimulator(robot, false)) {
      assertTrue(sim.getAutoIgnoredRestingPairs().contains("Slamtake <-> FrontHopper"));

      // At rest, CollisionActive is false because resting sibling overlap was calibrated out
      sim.simIterate();
      assertFalse(sim.hasAnyCollision());

      // Moving Hood into Slamtake triggers a real dynamic collision
      hoodPose.set(restingOrigin);
      sim.simIterate();
      assertTrue(sim.hasCollisionBetween("Hood", "Slamtake"));
      assertFalse(sim.hasCollisionBetween("Slamtake", "FrontHopper"));
    }
  }

  @Test
  void ignoreRestingOverlapsDoesNotSilenceExtensionEnvelopeViolations(@TempDir Path tempDir)
      throws IOException {
    Path boxMesh = tempDir.resolve("box.stl");
    Files.writeString(boxMesh, cubeStl(0.2, 0.2, 0.2), StandardCharsets.UTF_8);

    Pose3d outside = new Pose3d(new Translation3d(1.2, 0.0, 0.3), new Rotation3d());

    CadRobotConfig robot =
        new CadRobotConfig("EnvelopeSafetyBot")
            .withExtensionEnvelope(new BoundingBox3d(-0.5, -0.5, 0.0, 0.5, 0.5, 1.0))
            .ignoreRestingOverlaps()
            .addComponent(
                new CadComponentConfig("Arm", new CadAssetConfig(boxMesh))
                    .withPoseSupplier(() -> outside));

    try (CadRobotSimulator sim = new CadRobotSimulator(robot, false)) {
      assertTrue(sim.getAutoIgnoredRestingPairs().isEmpty());
      sim.simIterate();
      assertTrue(sim.hasExtensionViolation());
    }
  }

  @Test
  @SuppressWarnings("unchecked")
  void autoGroundsChassisAndUsesMultiPrimitiveSubObbs(@TempDir Path tempDir) throws IOException {
    // Chassis with minZ = -0.05 -> autoGroundChassis should shift Z by +0.05
    String chassisJson =
        """
        {
          "asset": {"version": "2.0"},
          "accessors": [
            {"count": 36, "type": "SCALAR", "componentType": 5123},
            {"count": 8, "type": "VEC3", "componentType": 5126, "min": [-0.4, -0.4, -0.05], "max": [0.4, 0.4, 0.15]}
          ],
          "meshes": [{"primitives": [{"attributes": {"POSITION": 1}, "indices": 0}]}]
        }
        """;
    // L-shaped component: bottom rail + vertical post
    String lJson =
        """
        {
          "asset": {"version": "2.0"},
          "accessors": [
            {"count": 36, "type": "SCALAR", "componentType": 5123},
            {"count": 8, "type": "VEC3", "componentType": 5126, "min": [0.0, 0.0, 0.0], "max": [0.8, 0.1, 0.1]},
            {"count": 8, "type": "VEC3", "componentType": 5126, "min": [0.0, 0.0, 0.0], "max": [0.1, 0.1, 0.8]}
          ],
          "meshes": [
            {
              "primitives": [
                {"attributes": {"POSITION": 1}, "indices": 0},
                {"attributes": {"POSITION": 2}, "indices": 0}
              ]
            }
          ]
        }
        """;
    Path chassisGlb = tempDir.resolve("chassis.glb");
    Path lGlb = tempDir.resolve("l_frame.glb");
    Path cubeStl = tempDir.resolve("cube.stl");
    Files.write(chassisGlb, buildGlb(chassisJson));
    Files.write(lGlb, buildGlb(lJson));
    Files.writeString(cubeStl, cubeStl(0.15, 0.1, 0.15), StandardCharsets.UTF_8);

    Path exportDir = tempDir.resolve("export");
    CadRobotConfig robot =
        new CadRobotConfig("GroundedMultiObbBot")
            .withChassisAsset(new CadAssetConfig(chassisGlb))
            .autoGroundChassis()
            .withAdvantageScopeExportDir(exportDir)
            .addComponent(
                new CadComponentConfig("LFrame", new CadAssetConfig(lGlb))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(() -> new Pose3d(0.0, 0.0, 0.2, new Rotation3d())))
            .addComponent(
                new CadComponentConfig("PartInCavity", new CadAssetConfig(cubeStl))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(() -> new Pose3d(0.4, 0.0, 0.55, new Rotation3d())));

    try (CadRobotSimulator sim = new CadRobotSimulator(robot, false)) {
      assertEquals(0.05, sim.getChassisGroundOffsetMeters(), 1e-6);
      sim.simIterate();
      // Multi-primitive sub-OBBs avoid false-positive hit inside LFrame's open cavity
      assertFalse(sim.hasCollisionBetween("LFrame", "PartInCavity"));

      Map<String, Object> exported =
          (Map<String, Object>)
              MinimalJson.parse(
                  Files.readString(exportDir.resolve("Robot_GroundedMultiObbBot/config.json")));
      List<Number> pos = (List<Number>) exported.get("position");
      assertEquals(0.05, pos.get(2).doubleValue(), 1e-6);
    }
  }

  private static byte[] buildGlb(String gltfJson) {
    byte[] raw = gltfJson.getBytes(StandardCharsets.UTF_8);
    int rem = raw.length % 4;
    byte[] jsonBytes = rem == 0 ? raw : new byte[raw.length + (4 - rem)];
    System.arraycopy(raw, 0, jsonBytes, 0, raw.length);
    for (int i = raw.length; i < jsonBytes.length; i++) jsonBytes[i] = 0x20;
    ByteBuffer glb = ByteBuffer.allocate(20 + jsonBytes.length).order(ByteOrder.LITTLE_ENDIAN);
    glb.putInt(0x46546C67);
    glb.putInt(2);
    glb.putInt(20 + jsonBytes.length);
    glb.putInt(jsonBytes.length);
    glb.putInt(0x4E4F534A);
    glb.put(jsonBytes);
    return glb.array();
  }

  @Test
  void samplesRollerTelemetryInSimSnapshot(@TempDir Path tempDir) throws IOException {
    Path boxMesh = tempDir.resolve("box.stl");
    Files.writeString(boxMesh, cubeStl(0.4, 0.4, 0.2), StandardCharsets.UTF_8);

    CadRobotConfig robot =
        new CadRobotConfig("RollerSimBot")
            .withChassisAsset(new CadAssetConfig(boxMesh))
            .addRollerTelemetry("Flywheel", () -> RadiansPerSecond.of(175.0), 0.0508)
            .addRollerTelemetry("IntakeRoller", () -> RadiansPerSecond.of(40.0), 0.0254);

    try (CadRobotSimulator sim = new CadRobotSimulator(robot, false)) {
      sim.simIterate();
      assertEquals(175.0, sim.getLatestRollerVelocitiesRadPerSec().get("Flywheel"), 1e-6);
      assertEquals(40.0, sim.getLatestRollerVelocitiesRadPerSec().get("IntakeRoller"), 1e-6);
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
