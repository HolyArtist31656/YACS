// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.advantagescope;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.cad.MeshFormat;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.util.MinimalJson;

/**
 * Generates AdvantageScope {@code config.json} files, exports {@code Robot_<Name>} custom asset
 * bundles, and supports direct OS-aware installation into AdvantageScope's User Assets directory.
 */
public final class AdvantageScopeConfigGenerator {
  private AdvantageScopeConfigGenerator() {}

  /**
   * Resolves the standard OS-specific AdvantageScope {@code userAssets} directory:
   *
   * <ul>
   *   <li>Windows: {@code %APPDATA%/AdvantageScope/userAssets}
   *   <li>macOS: {@code ~/Library/Application Support/AdvantageScope/userAssets}
   *   <li>Linux: {@code $XDG_CONFIG_HOME/AdvantageScope/userAssets} or {@code
   *       ~/.config/AdvantageScope/userAssets}
   * </ul>
   *
   * @return Resolved {@code userAssets} path, or empty if home/config environment variables are
   *     unavailable.
   */
  public static Optional<Path> resolveDefaultUserAssetsDir() {
    String os = System.getProperty("os.name", "").toLowerCase(Locale.ROOT);
    String userHome = System.getProperty("user.home", "");
    if (os.contains("win")) {
      String appData = System.getenv("APPDATA");
      if (appData != null && !appData.isBlank()) {
        return Optional.of(Path.of(appData, "AdvantageScope", "userAssets"));
      }
      if (!userHome.isBlank()) {
        return Optional.of(Path.of(userHome, "AppData", "Roaming", "AdvantageScope", "userAssets"));
      }
      return Optional.empty();
    }
    if (os.contains("mac")) {
      if (!userHome.isBlank()) {
        return Optional.of(
            Path.of(userHome, "Library", "Application Support", "AdvantageScope", "userAssets"));
      }
      return Optional.empty();
    }
    String xdg = System.getenv("XDG_CONFIG_HOME");
    if (xdg != null && !xdg.isBlank()) {
      return Optional.of(Path.of(xdg, "AdvantageScope", "userAssets"));
    }
    if (!userHome.isBlank()) {
      return Optional.of(Path.of(userHome, ".config", "AdvantageScope", "userAssets"));
    }
    return Optional.empty();
  }

  /**
   * Builds the AdvantageScope {@code config.json} payload for the given robot configuration.
   *
   * @param robotConfig The robot CAD configuration.
   * @return Pretty-printed JSON string ready to write as {@code config.json}.
   */
  public static String buildConfigJson(CadRobotConfig robotConfig) {
    return buildConfigJson(robotConfig, 0.0);
  }

  /**
   * Builds the AdvantageScope {@code config.json} payload with an additional chassis vertical
   * grounding offset (typically {@code -chassisMinZ} when {@link CadRobotConfig#autoGroundChassis()}
   * is enabled).
   *
   * @param robotConfig The robot CAD configuration.
   * @param chassisGroundOffsetZ Additional Z offset in meters added to the chassis position.
   * @return Pretty-printed JSON string ready to write as {@code config.json}.
   */
  public static String buildConfigJson(CadRobotConfig robotConfig, double chassisGroundOffsetZ) {
    Map<String, Object> root = new LinkedHashMap<>();
    root.put("name", robotConfig.getRobotName());
    root.put("disableSimplification", robotConfig.isDisableAdvantageScopeSimplification());
    if (robotConfig.getChassisAsset().isPresent()) {
      CadAssetConfig chassis = robotConfig.getChassisAsset().get();
      Translation3d basePos = chassis.getZeroedPositionMeters();
      Translation3d groundedPos =
          new Translation3d(basePos.getX(), basePos.getY(), basePos.getZ() + chassisGroundOffsetZ);
      root.put("rotations", rotationList(chassis));
      root.put("position", positionList(groundedPos));
    } else {
      root.put("rotations", List.of());
      root.put("position", positionList(new Translation3d(0.0, 0.0, chassisGroundOffsetZ)));
    }
    List<Object> cameraList = new ArrayList<>();
    for (CadCameraConfig cam : robotConfig.getCameras()) {
      Map<String, Object> camObj = new LinkedHashMap<>();
      camObj.put("name", cam.name());
      camObj.put("rotations", rotationList(cam.robotToCamera().getRotation()));
      camObj.put("position", positionList(cam.robotToCamera().getTranslation()));
      camObj.put("resolution", List.of(cam.resolutionWidthPx(), cam.resolutionHeightPx()));
      camObj.put("fov", cam.fovDegrees());
      cameraList.add(camObj);
    }
    root.put("cameras", cameraList);
    List<Object> components = new ArrayList<>();
    for (CadComponentConfig comp : robotConfig.getAllArticulatedComponents()) {
      Map<String, Object> compObj = new LinkedHashMap<>();
      compObj.put("zeroedRotations", rotationList(comp.getAssetConfig()));
      compObj.put(
          "zeroedPosition", positionList(comp.getAssetConfig().getZeroedPositionMeters()));
      components.add(compObj);
    }
    root.put("components", components);
    return MinimalJson.toPrettyJson(root);
  }

  /**
   * Exports a ready-to-load {@code Robot_<Name>} AdvantageScope directory containing {@code
   * config.json}, {@code model.glb}, and {@code model_0.glb..model_N.glb}.
   *
   * @param robotConfig The robot CAD configuration.
   * @param parentAssetsDir Parent directory in which {@code Robot_<Name>} will be created.
   * @return Path to the generated {@code Robot_<Name>} directory.
   * @throws IOException If creating directories or copying mesh files fails.
   */
  public static Path exportAssetBundle(CadRobotConfig robotConfig, Path parentAssetsDir)
      throws IOException {
    return exportAssetBundle(robotConfig, parentAssetsDir, 0.0);
  }

  /**
   * Exports a ready-to-load {@code Robot_<Name>} AdvantageScope directory with an optional chassis
   * vertical grounding offset applied to {@code config.json}.
   *
   * @param robotConfig The robot CAD configuration.
   * @param parentAssetsDir Parent directory in which {@code Robot_<Name>} will be created.
   * @param chassisGroundOffsetZ Additional Z offset in meters added to the chassis position.
   * @return Path to the generated {@code Robot_<Name>} directory.
   * @throws IOException If creating directories or copying mesh files fails.
   */
  public static Path exportAssetBundle(
      CadRobotConfig robotConfig, Path parentAssetsDir, double chassisGroundOffsetZ)
      throws IOException {
    String safeName = robotConfig.getRobotName().replaceAll("[^A-Za-z0-9_-]", "_");
    Path robotDir = parentAssetsDir.resolve("Robot_" + safeName);
    Files.createDirectories(robotDir);
    Files.writeString(
        robotDir.resolve("config.json"),
        buildConfigJson(robotConfig, chassisGroundOffsetZ),
        StandardCharsets.UTF_8);

    if (robotConfig.getChassisAsset().isPresent()) {
      Path chassisPath = robotConfig.getChassisAsset().get().getMeshPath();
      if (Files.exists(chassisPath) && MeshFormat.fromPath(chassisPath) == MeshFormat.GLB) {
        Files.copy(
            chassisPath, robotDir.resolve("model.glb"), StandardCopyOption.REPLACE_EXISTING);
      }
    }
    int idx = 0;
    for (CadComponentConfig comp : robotConfig.getAllArticulatedComponents()) {
      Path compPath = comp.getAssetConfig().getMeshPath();
      if (Files.exists(compPath) && MeshFormat.fromPath(compPath) == MeshFormat.GLB) {
        Files.copy(
            compPath,
            robotDir.resolve("model_" + idx + ".glb"),
            StandardCopyOption.REPLACE_EXISTING);
      }
      idx++;
    }
    return robotDir;
  }

  /**
   * Installs {@code Robot_<Name>} directly into the current OS's default AdvantageScope {@code
   * userAssets} directory without nesting.
   *
   * @param robotConfig The robot CAD configuration.
   * @return Path to the installed {@code Robot_<Name>} directory, or empty if the OS directory
   *     could not be resolved.
   * @throws IOException If writing the asset bundle fails.
   */
  public static Optional<Path> installToAdvantageScope(CadRobotConfig robotConfig)
      throws IOException {
    return installToAdvantageScope(robotConfig, 0.0);
  }

  /**
   * Installs {@code Robot_<Name>} directly into the current OS's default AdvantageScope {@code
   * userAssets} directory with an optional chassis vertical grounding offset.
   *
   * @param robotConfig The robot CAD configuration.
   * @param chassisGroundOffsetZ Additional Z offset in meters added to the chassis position.
   * @return Path to the installed {@code Robot_<Name>} directory, or empty if the OS directory
   *     could not be resolved.
   * @throws IOException If writing the asset bundle fails.
   */
  public static Optional<Path> installToAdvantageScope(
      CadRobotConfig robotConfig, double chassisGroundOffsetZ) throws IOException {
    Optional<Path> userAssetsDir = resolveDefaultUserAssetsDir();
    if (userAssetsDir.isEmpty()) {
      return Optional.empty();
    }
    Path installed = exportAssetBundle(robotConfig, userAssetsDir.get(), chassisGroundOffsetZ);
    System.out.println("[YACSim] Installed AdvantageScope asset bundle: " + installed);
    return Optional.of(installed);
  }


  /**
   * Builds a starter AdvantageScope layout JSON pre-binding {@code RobotPose3d}, {@code
   * ComponentPoses}, {@code CollisionActive}, and {@code CollisionDetails} under the configured
   * telemetry table.
   *
   * @param robotConfig Robot CAD configuration.
   * @return Pretty-printed layout JSON string.
   */
  public static String buildStarterLayoutJson(CadRobotConfig robotConfig) {
    String prefix = "NT:/" + robotConfig.getTelemetryTable() + "/";
    Map<String, Object> fieldTab = new LinkedHashMap<>();
    fieldTab.put("type", 4);
    fieldTab.put("title", "YACSim 3D Field");
    Map<String, Object> fieldConfig = new LinkedHashMap<>();
    fieldConfig.put("robot", robotConfig.getRobotName());
    fieldConfig.put("robotPose3dTopic", prefix + "RobotPose3d");
    fieldConfig.put("componentPosesTopic", prefix + "ComponentPoses");
    fieldTab.put("config", fieldConfig);

    Map<String, Object> collisionTab = new LinkedHashMap<>();
    collisionTab.put("type", 2);
    collisionTab.put("title", "YACSim Collisions");
    collisionTab.put("topics", List.of(prefix + "CollisionActive", prefix + "CollisionDetails"));

    Map<String, Object> layout = new LinkedHashMap<>();
    layout.put("version", "4.0.0");
    layout.put("tabs", List.of(fieldTab, collisionTab));
    return MinimalJson.toPrettyJson(layout);
  }

  /**
   * Writes a starter AdvantageScope layout JSON file pre-wired to this robot's YACSim NT4 topics.
   *
   * @param robotConfig Robot CAD configuration.
   * @param outputFile Destination {@code .json} layout path.
   * @return Path to the written layout file.
   * @throws IOException If writing the layout file fails.
   */
  public static Path exportStarterLayout(CadRobotConfig robotConfig, Path outputFile)
      throws IOException {
    if (outputFile.getParent() != null) {
      Files.createDirectories(outputFile.getParent());
    }
    Files.writeString(outputFile, buildStarterLayoutJson(robotConfig), StandardCharsets.UTF_8);
    return outputFile;
  }

  private static List<Object> rotationList(CadAssetConfig asset) {
    List<CadAssetConfig.AxisRotation> ordered = asset.getOrderedZeroedRotations();
    if (!ordered.isEmpty()) {
      List<Object> arr = new ArrayList<>(ordered.size());
      for (CadAssetConfig.AxisRotation r : ordered) {
        arr.add(axisRot(r.axis(), r.degrees()));
      }
      return arr;
    }
    return rotationList(asset.getZeroedRotation());
  }

  private static List<Object> rotationList(Rotation3d rot) {
    List<Object> arr = new ArrayList<>();
    double rx = Math.toDegrees(rot.getX()),
        ry = Math.toDegrees(rot.getY()),
        rz = Math.toDegrees(rot.getZ());
    if (Math.abs(rx) > 1e-6) arr.add(axisRot("x", rx));
    if (Math.abs(ry) > 1e-6) arr.add(axisRot("y", ry));
    if (Math.abs(rz) > 1e-6) arr.add(axisRot("z", rz));
    return arr;
  }

  private static Map<String, Object> axisRot(String axis, double degrees) {
    Map<String, Object> obj = new LinkedHashMap<>();
    obj.put("axis", axis);
    obj.put("degrees", degrees);
    return obj;
  }

  private static List<Object> positionList(Translation3d pos) {
    return List.of(pos.getX(), pos.getY(), pos.getZ());
  }
}
