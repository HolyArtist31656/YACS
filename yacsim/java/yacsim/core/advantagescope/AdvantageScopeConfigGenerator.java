// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.advantagescope;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.cad.MeshFormat;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.util.MinimalJson;

public final class AdvantageScopeConfigGenerator {
  private AdvantageScopeConfigGenerator() {}

  public static String buildConfigJson(CadRobotConfig robotConfig) {
    Map<String, Object> root = new LinkedHashMap<>();
    root.put("name", robotConfig.getRobotName());
    root.put("disableSimplification", robotConfig.isDisableAdvantageScopeSimplification());
    if (robotConfig.getChassisAsset().isPresent()) {
      CadAssetConfig chassis = robotConfig.getChassisAsset().get();
      root.put("rotations", rotationList(chassis));
      root.put("position", positionList(chassis.getZeroedPositionMeters()));
    } else {
      root.put("rotations", List.of());
      root.put("position", positionList(new Translation3d()));
    }
    root.put("cameras", List.of());
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

  public static Path exportAssetBundle(CadRobotConfig robotConfig, Path parentAssetsDir)
      throws IOException {
    String safeName = robotConfig.getRobotName().replaceAll("[^A-Za-z0-9_-]", "_");
    Path robotDir = parentAssetsDir.resolve("Robot_" + safeName);
    Files.createDirectories(robotDir);
    Files.writeString(
        robotDir.resolve("config.json"), buildConfigJson(robotConfig), StandardCharsets.UTF_8);

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
