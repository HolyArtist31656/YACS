// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.model;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.advantagescope.CadCameraConfig;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.util.MinimalJson;
import yacsim.core.drivetrain.CadDrivetrainConfig;
import yacsim.core.physics.BoundingBox3d;

/**
 * Top-level fluent configuration for a simulated CAD robot, combining chassis, drivetrain,
 * articulated components, OBB collision rules, and AdvantageScope export settings.
 */
public final class CadRobotConfig {
  private final String robotName;
  private CadDrivetrainConfig drivetrainConfig = null;
  private CadAssetConfig chassisAsset = null;
  private final List<CadComponentConfig> components = new ArrayList<>();
  private final List<CadCameraConfig> cameras = new ArrayList<>();
  private final Set<String> ignoredCollisionPairs = new LinkedHashSet<>();
  private Supplier<Pose2d> drivetrainPoseSupplier = () -> new Pose2d();
  private String telemetryTable = "YACSim";
  private Path advantageScopeOutputDir = null;
  private BoundingBox3d extensionEnvelope = null;
  private OptionalDouble extensionXyMarginMeters = OptionalDouble.empty();
  private OptionalDouble extensionMaxHeightMeters = OptionalDouble.empty();
  private boolean collisionCheckingEnabled = true;
  private boolean ignoreRestingOverlaps = false;
  private boolean autoInstallToAdvantageScope = false;
  private boolean autoGroundChassis = false;
  private boolean disableAdvantageScopeSimplification = false;

  /**
   * Creates a robot CAD configuration with the given display/export name.
   *
   * @param robotName Robot name (used for AdvantageScope {@code Robot_<robotName>} bundles).
   */
  public CadRobotConfig(String robotName) {
    this.robotName = Objects.requireNonNull(robotName);
  }

  /**
   * Attaches a {@link CadDrivetrainConfig}, populating the chassis asset, field pose supplier, and
   * articulated swerve/wheel components.
   *
   * @param cfg Drivetrain configuration.
   * @return This config for chaining.
   */
  public CadRobotConfig withDrivetrain(CadDrivetrainConfig cfg) {
    this.drivetrainConfig = Objects.requireNonNull(cfg);
    this.chassisAsset = cfg.getChassisAsset();
    this.drivetrainPoseSupplier = cfg.getFieldPoseSupplier();
    return this;
  }

  /**
   * Sets the chassis CAD asset directly without a {@link CadDrivetrainConfig}.
   *
   * @param a Chassis CAD asset.
   * @return This config for chaining.
   */
  public CadRobotConfig withChassisAsset(CadAssetConfig a) {
    this.chassisAsset = Objects.requireNonNull(a);
    return this;
  }

  /**
   * Registers an articulated mechanism component in AdvantageScope {@code model_N} order.
   *
   * @param c Component configuration.
   * @return This config for chaining.
   */
  public CadRobotConfig addComponent(CadComponentConfig c) {
    this.components.add(Objects.requireNonNull(c));
    return this;
  }

  /**
   * Sets the robot's 2D field pose supplier.
   *
   * @param s Field pose supplier.
   * @return This config for chaining.
   */
  public CadRobotConfig withDrivetrainPoseSupplier(Supplier<Pose2d> s) {
    this.drivetrainPoseSupplier = Objects.requireNonNull(s);
    return this;
  }

  /**
   * Sets the root NetworkTables table name (default {@code "YACSim"}).
   *
   * @param t Root NetworkTables table name.
   * @return This config for chaining.
   */
  public CadRobotConfig withTelemetryTable(String t) {
    this.telemetryTable = Objects.requireNonNull(t);
    return this;
  }

  /**
   * Sets an output directory to automatically export an AdvantageScope {@code Robot_<Name>} bundle
   * on simulator startup.
   *
   * @param p Parent output directory, or {@code null} to disable automatic export.
   * @return This config for chaining.
   */
  public CadRobotConfig withAdvantageScopeExportDir(Path p) {
    this.advantageScopeOutputDir = p;
    return this;
  }

  /**
   * Sets an explicit legal robot-frame extension envelope checked against every collision-enabled
   * component. Takes precedence over {@link #withExtensionMargin(double, double)}.
   *
   * @param b Allowed {@link BoundingBox3d} in meters, or {@code null} to disable.
   * @return This config for chaining.
   */
  public CadRobotConfig withExtensionEnvelope(BoundingBox3d b) {
    this.extensionEnvelope = b;
    return this;
  }

  /**
   * Automatically derives the legal FRC extension envelope from the parsed chassis CAD bounding box
   * by expanding horizontally by {@code xyMarginMeters} on all four sides and capping vertical
   * height to {@code [0.0, maxHeightMeters]}.
   *
   * @param xyMarginMeters Non-negative horizontal extension margin beyond chassis bounds in meters.
   * @param maxHeightMeters Positive maximum legal height in meters.
   * @return This config for chaining.
   * @throws IllegalArgumentException If {@code xyMarginMeters < 0} or {@code maxHeightMeters <= 0}.
   */
  public CadRobotConfig withExtensionMargin(double xyMarginMeters, double maxHeightMeters) {
    if (xyMarginMeters < 0.0) {
      throw new IllegalArgumentException("xyMarginMeters must be non-negative.");
    }
    if (maxHeightMeters <= 0.0) {
      throw new IllegalArgumentException("maxHeightMeters must be positive.");
    }
    this.extensionXyMarginMeters = OptionalDouble.of(xyMarginMeters);
    this.extensionMaxHeightMeters = OptionalDouble.of(maxHeightMeters);
    return this;
  }

  /**
   * Globally enables or disables OBB collision and extension-envelope checking.
   *
   * @param e {@code true} to evaluate collisions during {@code simIterate()}.
   * @return This config for chaining.
   */
  public CadRobotConfig withCollisionChecking(boolean e) {
    this.collisionCheckingEnabled = e;
    return this;
  }

  /**
   * Enables automatic startup calibration that samples initial component poses once at {@code t =
   * 0} and ignores any component-to-component or component-to-chassis OBB pairs already overlapping
   * at rest. {@code ExtensionEnvelope} violations are never auto-ignored.
   *
   * @return This config for chaining.
   */
  public CadRobotConfig ignoreRestingOverlaps() {
    this.ignoreRestingOverlaps = true;
    return this;
  }

  /**
   * Enables or disables automatic startup ignoring of resting component/chassis OBB overlaps.
   *
   * @param enabled {@code true} to auto-ignore resting sibling/chassis OBB overlaps.
   * @return This config for chaining.
   */
  public CadRobotConfig withIgnoreRestingOverlaps(boolean enabled) {
    this.ignoreRestingOverlaps = enabled;
    return this;
  }

  /**
   * Ignores OBB collision checks between two named components (order-independent). Also supports
   * {@code "Chassis"} and {@code "ExtensionEnvelope"} as target names.
   *
   * @param componentA First component name.
   * @param componentB Second component name.
   * @return This config for chaining.
   */
  public CadRobotConfig ignoreCollisionPair(String componentA, String componentB) {
    Objects.requireNonNull(componentA, "componentA");
    Objects.requireNonNull(componentB, "componentB");
    this.ignoredCollisionPairs.add(canonicalPairKey(componentA, componentB));
    return this;
  }

  /**
   * Checks whether OBB collision checks between two named components are ignored
   * (order-independent).
   *
   * @param componentA First component name.
   * @param componentB Second component name.
   * @return {@code true} if the pair was registered via {@link #ignoreCollisionPair}.
   */
  public boolean isCollisionPairIgnored(String componentA, String componentB) {
    if (componentA == null || componentB == null) {
      return false;
    }
    return ignoredCollisionPairs.contains(canonicalPairKey(componentA, componentB));
  }

  static String canonicalPairKey(String a, String b) {
    return a.compareTo(b) <= 0 ? a + "\u0000" + b : b + "\u0000" + a;
  }

  /**
   * Controls AdvantageScope's {@code disableSimplification} flag in generated {@code config.json}.
   *
   * @param d {@code true} to disable mesh simplification in AdvantageScope.
   * @return This config for chaining.
   */
  public CadRobotConfig withDisableAdvantageScopeSimplification(boolean d) {
    this.disableAdvantageScopeSimplification = d;
    return this;
  }


  /**
   * Automatically installs the exported {@code Robot_<Name>} bundle directly into the current OS's
   * default AdvantageScope {@code userAssets} directory on simulator startup.
   *
   * @return This config for chaining.
   */
  public CadRobotConfig withAutoInstallToAdvantageScope() {
    this.autoInstallToAdvantageScope = true;
    return this;
  }

  /**
   * Enables or disables automatic installation into the OS AdvantageScope {@code userAssets}
   * directory.
   *
   * @param enabled {@code true} to auto-install into AdvantageScope on startup.
   * @return This config for chaining.
   */
  public CadRobotConfig withAutoInstallToAdvantageScope(boolean enabled) {
    this.autoInstallToAdvantageScope = enabled;
    return this;
  }

  /**
   * Automatically shifts the chassis CAD vertical zero position by {@code -chassisBounds.minZ()} in
   * exported AdvantageScope {@code config.json} files so the lowest point of the chassis sits flush
   * at {@code z = 0.0} on the field carpet.
   *
   * @return This config for chaining.
   */
  public CadRobotConfig autoGroundChassis() {
    this.autoGroundChassis = true;
    return this;
  }

  /**
   * Enables or disables automatic chassis vertical grounding in exported AdvantageScope configs.
   *
   * @param enabled {@code true} to shift chassis {@code minZ} to {@code 0.0}.
   * @return This config for chaining.
   */
  public CadRobotConfig withAutoGroundChassis(boolean enabled) {
    this.autoGroundChassis = enabled;
    return this;
  }


  /**
   * Loads an existing AdvantageScope {@code Robot_<Name>} directory containing {@code config.json},
   * {@code model.glb}, and {@code model_0.glb..model_N.glb}, preserving robot name, ordered zeroed
   * rotations, zeroed positions, and camera definitions. Attach pose suppliers afterward via {@link
   * #withComponentPoseSupplier(int, String, Supplier)}.
   *
   * @param robotAssetDir Path to an existing AdvantageScope {@code Robot_<Name>} directory.
   * @return Pre-populated {@link CadRobotConfig}.
   * @throws IOException If {@code config.json} cannot be read.
   */
  @SuppressWarnings("unchecked")
  public static CadRobotConfig fromAdvantageScopeDir(Path robotAssetDir) throws IOException {
    Objects.requireNonNull(robotAssetDir, "robotAssetDir");
    Path configFile = robotAssetDir.resolve("config.json");
    Map<String, Object> root =
        (Map<String, Object>) MinimalJson.parse(Files.readString(configFile, StandardCharsets.UTF_8));
    String name =
        root.get("name") instanceof String s && !s.isBlank()
            ? s
            : robotAssetDir.getFileName().toString().replaceFirst("^Robot_", "");
    CadRobotConfig cfg = new CadRobotConfig(name);
    if (root.get("disableSimplification") instanceof Boolean b) {
      cfg.withDisableAdvantageScopeSimplification(b);
    }

    Path chassisPath = robotAssetDir.resolve("model.glb");
    if (Files.exists(chassisPath)) {
      CadAssetConfig chassis = new CadAssetConfig(chassisPath);
      applyZeroedTransform(
          chassis,
          (List<Object>) root.get("position"),
          (List<Object>) root.get("rotations"));
      cfg.withChassisAsset(chassis);
    }

    List<Object> compList = (List<Object>) root.get("components");
    if (compList != null) {
      for (int i = 0; i < compList.size(); i++) {
        Map<String, Object> compObj = (Map<String, Object>) compList.get(i);
        Path compMesh = robotAssetDir.resolve("model_" + i + ".glb");
        CadAssetConfig compAsset = new CadAssetConfig(compMesh);
        applyZeroedTransform(
            compAsset,
            (List<Object>) compObj.get("zeroedPosition"),
            (List<Object>) compObj.get("zeroedRotations"));
        cfg.addComponent(
            new CadComponentConfig("Component_" + i, compAsset).withParentComponent("Chassis"));
      }
    }

    List<Object> camList = (List<Object>) root.get("cameras");
    if (camList != null) {
      for (Object camObj : camList) {
        Map<String, Object> c = (Map<String, Object>) camObj;
        String camName = c.get("name") instanceof String s ? s : "Camera";
        List<Object> pos = (List<Object>) c.get("position");
        List<Object> rots = (List<Object>) c.get("rotations");
        List<Object> res = (List<Object>) c.get("resolution");
        double fov = c.get("fov") instanceof Number n ? n.doubleValue() : 70.0;
        int w = (res != null && res.size() >= 2) ? ((Number) res.get(0)).intValue() : 1280;
        int h = (res != null && res.size() >= 2) ? ((Number) res.get(1)).intValue() : 720;
        Translation3d t = parseTranslation(pos);
        Rotation3d r = parseRotation(rots);
        cfg.addCamera(new CadCameraConfig(camName, new Transform3d(t, r), fov, w, h));
      }
    }
    return cfg;
  }

  @SuppressWarnings("unchecked")
  private static void applyZeroedTransform(
      CadAssetConfig asset, List<Object> posList, List<Object> rotList) {
    if (posList != null && posList.size() >= 3) {
      asset.withZeroedPosition(parseTranslation(posList));
    }
    if (rotList != null) {
      for (Object obj : rotList) {
        Map<String, Object> rot = (Map<String, Object>) obj;
        String axis = String.valueOf(rot.get("axis"));
        double deg = ((Number) rot.get("degrees")).doubleValue();
        asset.addZeroedRotation(axis, deg);
      }
    }
  }

  private static Translation3d parseTranslation(List<Object> pos) {
    if (pos == null || pos.size() < 3) {
      return new Translation3d();
    }
    return new Translation3d(
        ((Number) pos.get(0)).doubleValue(),
        ((Number) pos.get(1)).doubleValue(),
        ((Number) pos.get(2)).doubleValue());
  }

  @SuppressWarnings("unchecked")
  private static Rotation3d parseRotation(List<Object> rotList) {
    Rotation3d rot = new Rotation3d();
    if (rotList == null) {
      return rot;
    }
    for (Object obj : rotList) {
      Map<String, Object> r = (Map<String, Object>) obj;
      String axis = String.valueOf(r.get("axis")).toLowerCase();
      double rad = Math.toRadians(((Number) r.get("degrees")).doubleValue());
      Rotation3d step =
          switch (axis) {
            case "x" -> new Rotation3d(rad, 0.0, 0.0);
            case "y" -> new Rotation3d(0.0, rad, 0.0);
            case "z" -> new Rotation3d(0.0, 0.0, rad);
            default -> new Rotation3d();
          };
      rot = rot.rotateBy(step);
    }
    return rot;
  }

  /**
   * Binds a {@link Pose3d} supplier to an existing component by 0-based index (especially useful
   * after {@link #fromAdvantageScopeDir(Path)}).
   *
   * @param index 0-based index in {@link #getComponents()}.
   * @param poseSupplier Pose supplier for the component.
   * @return This config for chaining.
   */
  public CadRobotConfig withComponentPoseSupplier(int index, Supplier<Pose3d> poseSupplier) {
    Objects.requireNonNull(poseSupplier, "poseSupplier");
    components.get(index).withPoseSupplier(poseSupplier);
    return this;
  }

  /**
   * Renames and binds a {@link Pose3d} supplier to an existing component by 0-based index (useful
   * after {@link #fromAdvantageScopeDir(Path)} so collision details use meaningful mechanism names).
   *
   * @param index 0-based index in {@link #getComponents()}.
   * @param newName New component name (for example {@code "Hood"}).
   * @param poseSupplier Pose supplier for the component.
   * @return This config for chaining.
   */
  public CadRobotConfig withComponentPoseSupplier(
      int index, String newName, Supplier<Pose3d> poseSupplier) {
    Objects.requireNonNull(newName, "newName");
    Objects.requireNonNull(poseSupplier, "poseSupplier");
    CadComponentConfig prev = components.get(index);
    CadComponentConfig replacement =
        new CadComponentConfig(newName, prev.getAssetConfig())
            .withCollisionEnabled(prev.isCollisionEnabled())
            .withPoseSupplier(poseSupplier);
    prev.getParentComponentName().ifPresent(replacement::withParentComponent);
    components.set(index, replacement);
    return this;
  }

  /**
   * Registers an AdvantageScope 3D vision camera frustum exported into {@code config.json}.
   *
   * @param camera Camera frustum configuration.
   * @return This config for chaining.
   */
  public CadRobotConfig addCamera(CadCameraConfig camera) {
    this.cameras.add(Objects.requireNonNull(camera, "camera"));
    return this;
  }

  /**
   * Convenience overload registering an AdvantageScope 3D vision camera frustum.
   *
   * @param name Camera name.
   * @param robotToCamera Robot-to-camera {@link Transform3d}.
   * @param fovDegrees Horizontal FOV in degrees.
   * @param resolutionWidthPx Width in pixels.
   * @param resolutionHeightPx Height in pixels.
   * @return This config for chaining.
   */
  public CadRobotConfig addCamera(
      String name,
      Transform3d robotToCamera,
      double fovDegrees,
      int resolutionWidthPx,
      int resolutionHeightPx) {
    return addCamera(
        new CadCameraConfig(name, robotToCamera, fovDegrees, resolutionWidthPx, resolutionHeightPx));
  }

  /** @return Configured robot name. */
  public String getRobotName() {
    return robotName;
  }

  /** @return Attached drivetrain configuration, if present. */
  public Optional<CadDrivetrainConfig> getDrivetrainConfig() {
    return Optional.ofNullable(drivetrainConfig);
  }

  /** @return Chassis CAD asset, if configured. */
  public Optional<CadAssetConfig> getChassisAsset() {
    return Optional.ofNullable(chassisAsset);
  }

  /** @return Unmodifiable list of non-drivetrain mechanism components. */
  public List<CadComponentConfig> getComponents() {
    return Collections.unmodifiableList(components);
  }

  /** @return Unmodifiable list of drivetrain plus mechanism components in export order. */
  public List<CadComponentConfig> getAllArticulatedComponents() {
    List<CadComponentConfig> all = new ArrayList<>();
    if (drivetrainConfig != null) {
      all.addAll(drivetrainConfig.toArticulatedComponents());
    }
    all.addAll(components);
    return Collections.unmodifiableList(all);
  }

  /** @return Robot 2D field pose supplier. */
  public Supplier<Pose2d> getDrivetrainPoseSupplier() {
    return drivetrainPoseSupplier;
  }

  /** @return Root NetworkTables table name. */
  public String getTelemetryTable() {
    return telemetryTable;
  }

  /** @return AdvantageScope export directory, if configured. */
  public Optional<Path> getAdvantageScopeOutputDir() {
    return Optional.ofNullable(advantageScopeOutputDir);
  }

  /** @return Explicit legal extension envelope, if configured. */
  public Optional<BoundingBox3d> getExtensionEnvelope() {
    return Optional.ofNullable(extensionEnvelope);
  }

  /** @return Auto-derived extension horizontal margin in meters, if configured. */
  public OptionalDouble getExtensionXyMarginMeters() {
    return extensionXyMarginMeters;
  }

  /** @return Auto-derived extension maximum height in meters, if configured. */
  public OptionalDouble getExtensionMaxHeightMeters() {
    return extensionMaxHeightMeters;
  }

  /** @return {@code true} if collision checking is enabled. */
  public boolean isCollisionCheckingEnabled() {
    return collisionCheckingEnabled;
  }

  /** @return {@code true} if resting sibling/chassis OBB overlaps are auto-ignored at startup. */
  public boolean isIgnoreRestingOverlapsEnabled() {
    return ignoreRestingOverlaps;
  }

  /** @return {@code true} if AdvantageScope mesh simplification is disabled. */
  public boolean isDisableAdvantageScopeSimplification() {
    return disableAdvantageScopeSimplification;
  }

  /** @return {@code true} if automatic installation into AdvantageScope User Assets is enabled. */
  public boolean isAutoInstallToAdvantageScopeEnabled() {
    return autoInstallToAdvantageScope;
  }

  /** @return {@code true} if automatic chassis Z-grounding is enabled. */
  public boolean isAutoGroundChassisEnabled() {
    return autoGroundChassis;
  }

  /** @return Unmodifiable list of configured AdvantageScope cameras. */
  public List<CadCameraConfig> getCameras() {
    return Collections.unmodifiableList(cameras);
  }
}
