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
import java.util.LinkedHashMap;
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
import org.wpilib.units.measure.AngularVelocity;
import yacsim.core.advantagescope.CadCameraConfig;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadLinkManifest;
import yacsim.core.manifest.CadManifestLoader;
import yacsim.core.manifest.CadRobotManifest;
import yacsim.core.manifest.CadUrdfOptions;
import yacsim.core.util.MinimalJson;
import yacsim.core.drivetrain.CadDrivetrainConfig;
import yacsim.core.physics.BoundingBox3d;

/**
 * Top-level fluent configuration for a simulated CAD robot, combining chassis, drivetrain,
 * articulated components, OBB collision rules, and AdvantageScope export settings.
 */
public final class CadRobotConfig {
  private final String robotName;
  private CadRobotManifest manifest = null;
  private CadDrivetrainConfig drivetrainConfig = null;
  private CadAssetConfig chassisAsset = null;
  private final List<CadComponentConfig> components = new ArrayList<>();
  private final List<CadCameraConfig> cameras = new ArrayList<>();
  private final List<CadRollerTelemetryConfig> rollerTelemetries = new ArrayList<>();
  private final Set<String> ignoredCollisionPairs = new LinkedHashSet<>();
  private final Set<String> boundComponentKeys = new LinkedHashSet<>();
  private boolean reenableCollisionOnBind = false;
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

  /**
   * Loads a {@link CadRobotConfig} from a {@code yacsim-robot.json} manifest, automatically
   * populating chassis/link meshes, AdvantageScope zeroed transforms, exact CAD mass properties,
   * cameras, and collision/extension settings.
   *
   * <p>Bind live mechanism poses afterward by link name via {@link
   * #withComponentPoseSupplier(String, Supplier)} or {@code YamsCommands3Bridge.bindFromManifest}.
   *
   * @param manifestPath Path to {@code yacsim-robot.json}.
   * @return Pre-populated {@link CadRobotConfig}.
   * @throws IOException If the manifest file cannot be read.
   */
  public static CadRobotConfig fromManifest(Path manifestPath) throws IOException {
    Objects.requireNonNull(manifestPath, "manifestPath");
    CadRobotManifest loaded = CadManifestLoader.loadJson(manifestPath);
    Path baseDir = manifestPath.toAbsolutePath().normalize().getParent();
    return fromManifest(loaded, baseDir != null ? baseDir : Path.of("."));
  }

  /**
   * Builds a {@link CadRobotConfig} from an already-parsed {@link CadRobotManifest} (useful when
   * mechanisms share the same loaded manifest instance).
   *
   * @param manifest Parsed {@link CadRobotManifest}.
   * @param baseDir Base directory used to resolve relative mesh paths in the manifest.
   * @return Pre-populated {@link CadRobotConfig}.
   */
  public static CadRobotConfig fromManifest(CadRobotManifest manifest, Path baseDir) {
    Objects.requireNonNull(manifest, "manifest");
    Objects.requireNonNull(baseDir, "baseDir");
    CadRobotConfig cfg = new CadRobotConfig(manifest.robotName());
    cfg.manifest = manifest;
    cfg.withIgnoreRestingOverlaps(manifest.ignoreRestingOverlaps());
    cfg.withAutoGroundChassis(manifest.autoGroundChassis());
    cfg.withDisableAdvantageScopeSimplification(manifest.disableAdvantageScopeSimplification());
    if (manifest.extensionXyMarginMeters().isPresent()
        && manifest.extensionMaxHeightMeters().isPresent()) {
      cfg.withExtensionMargin(
          manifest.extensionXyMarginMeters().getAsDouble(),
          manifest.extensionMaxHeightMeters().getAsDouble());
    }
    if (manifest.chassis().isPresent() && manifest.chassis().get().hasMesh()) {
      cfg.withChassisAsset(manifest.chassis().get().toCadAssetConfig(baseDir));
    }
    for (CadLinkManifest link : manifest.links()) {
      if (!link.hasMesh()) {
        continue;
      }
      String parentName =
          manifest.joints().stream()
              .filter(j -> j.childLink().equals(link.name()))
              .map(j -> j.parentLink())
              .findFirst()
              .orElse("Chassis");
      cfg.addComponent(
          new CadComponentConfig(link.name(), link.toCadAssetConfig(baseDir))
              .withParentComponent(parentName)
              .withCollisionEnabled(link.collisionEnabled()));
    }
    for (CadCameraConfig cam : manifest.cameras()) {
      cfg.addCamera(cam);
    }
    return cfg;
  }

  /**
   * Imports a standard URDF XML file ({@code .urdf} or {@code .xml}) exported by Onshape ({@code
   * onshape-to-robot}), Fusion 360, or SolidWorks directly into a {@link CadRobotConfig}.
   *
   * @param urdfPath Path to the URDF XML file.
   * @return Pre-populated {@link CadRobotConfig} with attached {@link CadRobotManifest}.
   * @throws IOException If reading or parsing the URDF fails.
   */
  public static CadRobotConfig fromUrdf(Path urdfPath) throws IOException {
    return fromUrdf(urdfPath, CadUrdfOptions.defaults());
  }

  /**
   * Imports a standard URDF XML file with code-side {@link CadUrdfOptions} (fastener filtering,
   * joint aliasing, unbound collision disabling).
   *
   * @param urdfPath Path to the URDF XML file.
   * @param options Filtering, aliasing, and performance options.
   * @return Pre-populated {@link CadRobotConfig}.
   * @throws IOException If reading or parsing fails.
   */
  public static CadRobotConfig fromUrdf(Path urdfPath, CadUrdfOptions options) throws IOException {
    Objects.requireNonNull(urdfPath, "urdfPath");
    Objects.requireNonNull(options, "options");
    Path baseDir = CadManifestLoader.resolveUrdfBaseDir(urdfPath);
    CadRobotManifest loaded = CadManifestLoader.loadUrdf(urdfPath, baseDir, options);
    CadRobotConfig cfg = applyUrdfRestingPoses(fromManifest(loaded, baseDir), loaded);
    cfg.reenableCollisionOnBind = options.isDisableUnboundCollisions();
    return cfg;
  }

  /**
   * Extracts and imports a native Onshape (or generic) URDF {@code .zip} bundle directly into a
   * {@link CadRobotConfig}.
   *
   * @param zipPath Path to the exported URDF {@code .zip} archive.
   * @return Pre-populated {@link CadRobotConfig} with attached {@link CadRobotManifest}.
   * @throws IOException If extracting or parsing the URDF archive fails.
   */
  public static CadRobotConfig fromUrdfZip(Path zipPath) throws IOException {
    return fromUrdfZip(zipPath, CadUrdfOptions.defaults());
  }

  /**
   * Extracts and imports a native Onshape (or generic) URDF {@code .zip} bundle with {@link
   * CadUrdfOptions}.
   *
   * @param zipPath Path to the exported URDF {@code .zip} archive.
   * @param options Filtering, aliasing, and performance options.
   * @return Pre-populated {@link CadRobotConfig}.
   * @throws IOException If extracting or parsing fails.
   */
  public static CadRobotConfig fromUrdfZip(Path zipPath, CadUrdfOptions options)
      throws IOException {
    CadManifestLoader.UrdfLoadResult result = CadManifestLoader.loadUrdfZip(zipPath, options);
    CadRobotConfig cfg =
        applyUrdfRestingPoses(fromManifest(result.manifest(), result.baseDir()), result.manifest());
    cfg.reenableCollisionOnBind = options.isDisableUnboundCollisions();
    return cfg;
  }

  /**
   * Extracts a native Onshape (or generic) URDF {@code .zip} bundle into {@code extractDir} and
   * imports it into a {@link CadRobotConfig}.
   *
   * @param zipPath Path to the exported URDF {@code .zip} archive.
   * @param extractDir Directory where archive entries are extracted.
   * @return Pre-populated {@link CadRobotConfig} with attached {@link CadRobotManifest}.
   * @throws IOException If extracting or parsing the URDF archive fails.
   */
  public static CadRobotConfig fromUrdfZip(Path zipPath, Path extractDir) throws IOException {
    return fromUrdfZip(zipPath, extractDir, CadUrdfOptions.defaults());
  }

  /**
   * Extracts a native Onshape (or generic) URDF {@code .zip} bundle into {@code extractDir} and
   * imports it with {@link CadUrdfOptions}.
   *
   * @param zipPath Path to the exported URDF {@code .zip} archive.
   * @param extractDir Directory where archive entries are extracted.
   * @param options Filtering, aliasing, and performance options.
   * @return Pre-populated {@link CadRobotConfig}.
   * @throws IOException If extracting or parsing fails.
   */
  public static CadRobotConfig fromUrdfZip(
      Path zipPath, Path extractDir, CadUrdfOptions options) throws IOException {
    CadManifestLoader.UrdfLoadResult result =
        CadManifestLoader.loadUrdfZip(zipPath, extractDir, options);
    CadRobotConfig cfg =
        applyUrdfRestingPoses(fromManifest(result.manifest(), result.baseDir()), result.manifest());
    cfg.reenableCollisionOnBind = options.isDisableUnboundCollisions();
    return cfg;
  }

  /**
   * Auto-detects and imports a URDF robot from a {@code .zip} archive, a {@code .urdf} / {@code
   * .xml} file, or an extracted folder containing a {@code .urdf} file.
   *
   * @param path Path to a URDF {@code .zip} bundle, {@code .urdf}/{@code .xml} file, or directory.
   * @return Pre-populated {@link CadRobotConfig} with attached {@link CadRobotManifest}.
   * @throws IOException If reading, extraction, or parsing fails.
   */
  public static CadRobotConfig fromUrdfOrZip(Path path) throws IOException {
    return fromUrdfOrZip(path, CadUrdfOptions.defaults());
  }

  /**
   * Auto-detects and imports a URDF robot from a {@code .zip} archive, a {@code .urdf} / {@code
   * .xml} file, or an extracted folder using {@link CadUrdfOptions}.
   *
   * @param path Path to a URDF {@code .zip} bundle, {@code .urdf}/{@code .xml} file, or directory.
   * @param options Filtering, aliasing, and performance options.
   * @return Pre-populated {@link CadRobotConfig}.
   * @throws IOException If reading, extraction, or parsing fails.
   */
  public static CadRobotConfig fromUrdfOrZip(Path path, CadUrdfOptions options) throws IOException {
    CadManifestLoader.UrdfLoadResult result = CadManifestLoader.loadUrdfOrZip(path, options);
    CadRobotConfig cfg =
        applyUrdfRestingPoses(fromManifest(result.manifest(), result.baseDir()), result.manifest());
    cfg.reenableCollisionOnBind = options.isDisableUnboundCollisions();
    return cfg;
  }

  private static CadRobotConfig applyUrdfRestingPoses(
      CadRobotConfig cfg, CadRobotManifest manifest) {
    Map<String, CadJointManifest> parentJointByChild = new LinkedHashMap<>();
    for (CadJointManifest j : manifest.joints()) {
      parentJointByChild.putIfAbsent(j.childLink(), j);
    }
    for (CadComponentConfig comp : cfg.components) {
      List<CadJointManifest> chain = new ArrayList<>();
      Set<String> visited = new LinkedHashSet<>();
      String cur = comp.getName();
      while (cur != null && visited.add(cur)) {
        CadJointManifest parentJoint = parentJointByChild.get(cur);
        if (parentJoint == null) {
          break;
        }
        chain.add(parentJoint);
        cur = parentJoint.parentLink();
      }
      Collections.reverse(chain);
      Pose3d resting = new Pose3d();
      for (CadJointManifest j : chain) {
        resting = resting.transformBy(new Transform3d(j.originMeters(), j.originRotation()));
        if (!j.zeroOffset().equals(new Transform3d())) {
          resting = resting.transformBy(j.zeroOffset());
        }
      }
      final Pose3d restingPose = resting;
      comp.withPoseSupplier(() -> restingPose);
    }
    return cfg;
  }

  /**
   * Binds a {@link Pose3d} supplier to an existing component by its link/component name (case
   * sensitive), ideal after {@link #fromManifest(Path)} or {@link #fromUrdf(Path)}.
   *
   * @param componentName Registered component name (for example {@code "Hood"} or {@code
   *     "Slamtake"}).
   * @param poseSupplier Live {@link Pose3d} supplier.
   * @return This config for chaining.
   * @throws IllegalArgumentException If no component with {@code componentName} is registered.
   */
  public CadRobotConfig withComponentPoseSupplier(
      String componentName, Supplier<Pose3d> poseSupplier) {
    return withComponentPoseSupplier(componentName, poseSupplier, true);
  }

  /**
   * Binds a {@link Pose3d} supplier to an existing component by its link/component name, with
   * explicit control over whether {@code CadUrdfOptions.disableUnboundCollisions(true)} should
   * auto-re-enable OBB collision checking on bind (pass {@code false} for spinning rollers/wheels).
   *
   * @param componentName Registered component name.
   * @param poseSupplier Live {@link Pose3d} supplier.
   * @param allowCollisionReenable {@code true} to re-enable collision when unbound collisions were
   *     disabled; {@code false} to keep collision disabled (recommended for continuous rollers).
   * @return This config for chaining.
   * @throws IllegalArgumentException If no component with {@code componentName} is registered.
   */
  public CadRobotConfig withComponentPoseSupplier(
      String componentName, Supplier<Pose3d> poseSupplier, boolean allowCollisionReenable) {
    Objects.requireNonNull(componentName, "componentName");
    Objects.requireNonNull(poseSupplier, "poseSupplier");
    Optional<CadComponentConfig> match = findComponentByName(componentName);
    if (match.isPresent()) {
      CadComponentConfig comp = match.get();
      comp.withPoseSupplier(poseSupplier);
      markComponentBound(comp.getName());
      if (reenableCollisionOnBind && allowCollisionReenable) {
        comp.withCollisionEnabled(true);
      }
      return this;
    }
    List<String> avail = components.stream().map(CadComponentConfig::getName).toList();
    throw new IllegalArgumentException(
        "No component named '"
            + componentName
            + "' in CadRobotConfig '"
            + robotName
            + "'. Available components: "
            + avail);
  }

  /**
   * Returns whether a visual component with the given name is registered in this config (exact
   * match first, then case/separator-insensitive via {@link CadRobotManifest#normalizeName}).
   *
   * @param componentName Component name to check.
   * @return {@code true} if present.
   */
  public boolean hasComponent(String componentName) {
    return findComponentByName(componentName).isPresent();
  }

  /**
   * Marks the named component as explicitly bound to a live mechanism pose supplier (used by {@link
   * yacsim.core.manifest.YacSimDiagnostic}).
   *
   * @param componentName Component or link name.
   * @return This config for chaining.
   */
  public CadRobotConfig markComponentBound(String componentName) {
    if (componentName != null && !componentName.isBlank()) {
      boundComponentKeys.add(CadRobotManifest.normalizeName(componentName));
    }
    return this;
  }

  /**
   * Returns whether the named component has been bound to a live pose supplier.
   *
   * @param componentName Component or link name.
   * @return {@code true} if explicitly bound.
   */
  public boolean isComponentBound(String componentName) {
    return componentName != null
        && boundComponentKeys.contains(CadRobotManifest.normalizeName(componentName));
  }

  /**
   * Returns the live {@link Pose3d} supplier registered for the named component, if present.
   *
   * @param componentName Component name to look up.
   * @return Registered pose supplier, or empty if no component with that name exists.
   */
  public Optional<Supplier<Pose3d>> getComponentPoseSupplier(String componentName) {
    return findComponentByName(componentName).map(CadComponentConfig::getPoseSupplier);
  }

  private Optional<CadComponentConfig> findComponentByName(String componentName) {
    if (componentName == null) {
      return Optional.empty();
    }
    Optional<CadComponentConfig> exact =
        components.stream().filter(c -> c.getName().equals(componentName)).findFirst();
    if (exact.isPresent()) {
      return exact;
    }
    String target = CadRobotManifest.normalizeName(componentName);
    return components.stream()
        .filter(c -> CadRobotManifest.normalizeName(c.getName()).equals(target))
        .findFirst();
  }

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


  /**
   * Registers a spinning wheel, roller bar, or flywheel for automatic NetworkTables telemetry
   * ({@code VelocityRPM}, {@code SurfaceSpeedMPS}, and {@code Spinning}) under {@code
   * /<telemetryTable>/Rollers/<name>/}. Works whether or not a separate {@code .glb} mesh exists
   * for the roller.
   *
   * @param rollerConfig Roller telemetry configuration.
   * @return This config for chaining.
   */
  public CadRobotConfig addRollerTelemetry(CadRollerTelemetryConfig rollerConfig) {
    this.rollerTelemetries.add(Objects.requireNonNull(rollerConfig, "rollerConfig"));
    return this;
  }

  /**
   * Convenience overload registering a spinning wheel, roller, or flywheel for automatic NT4
   * telemetry.
   *
   * @param name Unique roller/flywheel name (for example {@code "Flywheel"}).
   * @param angularVelocitySupplier Supplier returning the current {@link AngularVelocity}.
   * @param wheelRadiusMeters Positive wheel or roller radius in meters.
   * @return This config for chaining.
   */
  public CadRobotConfig addRollerTelemetry(
      String name, Supplier<AngularVelocity> angularVelocitySupplier, double wheelRadiusMeters) {
    return addRollerTelemetry(
        new CadRollerTelemetryConfig(name, angularVelocitySupplier, wheelRadiusMeters));
  }

  /** @return Configured robot name. */
  public String getRobotName() {
    return robotName;
  }

  /** @return Underlying CAD manifest if created via {@link #fromManifest} or {@link #fromUrdf}. */
  public Optional<CadRobotManifest> getManifest() {
    return Optional.ofNullable(manifest);
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

  /** @return Unmodifiable list of registered spinning roller/flywheel telemetry configs. */
  public List<CadRollerTelemetryConfig> getRollerTelemetries() {
    return Collections.unmodifiableList(rollerTelemetries);
  }
}
