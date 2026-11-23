// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.advantagescope;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.drivetrain.CadDrivetrainConfig;
import yacsim.core.drivetrain.CadSwerveModuleConfig;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.util.MinimalJson;

class AdvantageScopeConfigGeneratorTest {

  @Test
  @SuppressWarnings("unchecked")
  void generatesValidAdvantageScopeJsonWithDrivetrainAndMechanisms(@TempDir Path tempDir)
      throws IOException {
    CadDrivetrainConfig drivetrain =
        new CadDrivetrainConfig(
                new CadAssetConfig(tempDir.resolve("chassis.glb"))
                    .withZeroedRotation(new Rotation3d(0, 0, Math.toRadians(90))))
            .addSwerveModule(
                new CadSwerveModuleConfig("FL", new Translation3d(0.3, 0.3, 0.05))
                    .withSteerForkAsset(new CadAssetConfig(tempDir.resolve("fork.glb")))
                    .withWheelAsset(new CadAssetConfig(tempDir.resolve("wheel.glb"))));

    CadRobotConfig robot =
        new CadRobotConfig("2181 GEARS 2027")
            .withDrivetrain(drivetrain)
            .addComponent(
                new CadComponentConfig(
                    "Elevator",
                    new CadAssetConfig(tempDir.resolve("elevator.glb"))
                        .withZeroedPosition(new Translation3d(0.1, 0.0, 0.2))))
            .addComponent(
                new CadComponentConfig("Arm", new CadAssetConfig(tempDir.resolve("arm.glb"))));

    Path bundleDir = AdvantageScopeConfigGenerator.exportAssetBundle(robot, tempDir);
    Path configFile = bundleDir.resolve("config.json");
    assertTrue(Files.exists(configFile));

    Map<String, Object> json =
        (Map<String, Object>) MinimalJson.parse(Files.readString(configFile));
    assertEquals("2181 GEARS 2027", json.get("name"));
    assertFalse((Boolean) json.get("disableSimplification"));
    assertEquals(4, ((List<Object>) json.get("components")).size());
  }

  @Test
  @SuppressWarnings("unchecked")
  void emitsOrderedMultiAxisRotationsInExactSequence(@TempDir Path tempDir) throws IOException {
    CadRobotConfig robot =
        new CadRobotConfig("Darwin")
            .withChassisAsset(
                new CadAssetConfig(tempDir.resolve("model.glb"))
                    .addZeroedRotation("x", 90)
                    .addZeroedRotation("z", 90))
            .addComponent(
                new CadComponentConfig(
                    "Slamtake",
                    new CadAssetConfig(tempDir.resolve("model_1.glb"))
                        .addZeroedRotation("x", 90)
                        .addZeroedRotation("z", 90)
                        .addZeroedRotation("y", 147.5)
                        .withZeroedPosition(new Translation3d(0.167, 0.0, 0.327))));

    Map<String, Object> json =
        (Map<String, Object>)
            MinimalJson.parse(AdvantageScopeConfigGenerator.buildConfigJson(robot));
    List<Map<String, Object>> chassisRots = (List<Map<String, Object>>) json.get("rotations");
    assertEquals(2, chassisRots.size());
    assertEquals("x", chassisRots.get(0).get("axis"));
    assertEquals("z", chassisRots.get(1).get("axis"));

    List<Map<String, Object>> components = (List<Map<String, Object>>) json.get("components");
    List<Map<String, Object>> compRots =
        (List<Map<String, Object>>) components.get(0).get("zeroedRotations");
    assertEquals(3, compRots.size());
    assertEquals("x", compRots.get(0).get("axis"));
    assertEquals("z", compRots.get(1).get("axis"));
    assertEquals("y", compRots.get(2).get("axis"));
    assertEquals(147.5, ((Number) compRots.get(2).get("degrees")).doubleValue(), 1e-6);
  }

  @Test
  void resolvesDefaultAdvantageScopeUserAssetsDir() {
    assertTrue(AdvantageScopeConfigGenerator.resolveDefaultUserAssetsDir().isPresent());
    assertTrue(
        AdvantageScopeConfigGenerator.resolveDefaultUserAssetsDir()
            .orElseThrow()
            .endsWith(Path.of("AdvantageScope", "userAssets")));
  }

  @Test
  @SuppressWarnings("unchecked")
  void appliesChassisGroundOffsetToConfigJson(@TempDir Path tempDir) throws IOException {
    CadRobotConfig robot =
        new CadRobotConfig("GroundedBot")
            .withChassisAsset(
                new CadAssetConfig(tempDir.resolve("chassis.glb"))
                    .withZeroedPosition(new Translation3d(0.0, 0.0, 0.02)));

    Map<String, Object> json =
        (Map<String, Object>)
            MinimalJson.parse(AdvantageScopeConfigGenerator.buildConfigJson(robot, 0.05));
    List<Number> pos = (List<Number>) json.get("position");
    assertEquals(0.07, pos.get(2).doubleValue(), 1e-6);
  }

  @Test
  @SuppressWarnings("unchecked")
  void exportsCamerasAndStarterLayoutJson(@TempDir Path tempDir) throws IOException {
    CadRobotConfig robot =
        new CadRobotConfig("VisionBot")
            .withTelemetryTable("YACSim")
            .addCamera(
                "FrontCam",
                new Transform3d(
                    new Translation3d(0.25, 0.1, 0.5), new Rotation3d(0, Math.toRadians(-15), 0)),
                70.0,
                1280,
                800);

    Map<String, Object> configJson =
        (Map<String, Object>)
            MinimalJson.parse(AdvantageScopeConfigGenerator.buildConfigJson(robot));
    List<Map<String, Object>> cams = (List<Map<String, Object>>) configJson.get("cameras");
    assertEquals(1, cams.size());
    assertEquals("FrontCam", cams.get(0).get("name"));
    assertEquals(70.0, ((Number) cams.get(0).get("fov")).doubleValue(), 1e-6);

    Path layoutFile =
        AdvantageScopeConfigGenerator.exportStarterLayout(robot, tempDir.resolve("layout.json"));
    assertTrue(Files.exists(layoutFile));
    Map<String, Object> layout =
        (Map<String, Object>) MinimalJson.parse(Files.readString(layoutFile));
    assertEquals(2, ((List<Object>) layout.get("tabs")).size());
  }

  @Test
  void importsExistingAdvantageScopeDirectoryAndBindsPoseSuppliers(@TempDir Path tempDir)
      throws IOException {
    Path robotDir = tempDir.resolve("Robot_ImportTest");
    Files.createDirectories(robotDir);
    Files.writeString(robotDir.resolve("model.glb"), "placeholder");
    Files.writeString(robotDir.resolve("model_0.glb"), "placeholder");
    String configJson =
        """
        {
          "name": "ImportTest",
          "disableSimplification": true,
          "rotations": [{"axis": "x", "degrees": 90}, {"axis": "z", "degrees": 90}],
          "position": [0.0, 0.0, 0.05],
          "cameras": [{"name": "Limelight", "position": [0.2, 0.0, 0.4], "rotations": [], "resolution": [1280, 960], "fov": 62.5}],
          "components": [
            {"zeroedRotations": [{"axis": "y", "degrees": 45}], "zeroedPosition": [0.1, 0.0, 0.2]}
          ]
        }
        """;
    Files.writeString(robotDir.resolve("config.json"), configJson);

    Pose3d expectedPose = new Pose3d(0.1, 0.0, 0.5, new Rotation3d());
    CadRobotConfig imported =
        CadRobotConfig.fromAdvantageScopeDir(robotDir)
            .withComponentPoseSupplier(0, "Hood", () -> expectedPose);

    assertEquals("ImportTest", imported.getRobotName());
    assertTrue(imported.isDisableAdvantageScopeSimplification());
    assertEquals(2, imported.getChassisAsset().orElseThrow().getOrderedZeroedRotations().size());
    assertEquals(1, imported.getCameras().size());
    assertEquals("Limelight", imported.getCameras().get(0).name());
    assertEquals(1, imported.getComponents().size());
    assertEquals("Hood", imported.getComponents().get(0).getName());
    assertEquals(0.5, imported.getComponents().get(0).getPoseSupplier().get().getZ(), 1e-6);
  }
}
