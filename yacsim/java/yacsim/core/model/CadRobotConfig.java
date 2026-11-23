// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.model;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import yacsim.core.cad.CadAssetConfig;
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
  private final Set<String> ignoredCollisionPairs = new LinkedHashSet<>();
  private Supplier<Pose2d> drivetrainPoseSupplier = () -> new Pose2d();
  private String telemetryTable = "YACSim";
  private Path advantageScopeOutputDir = null;
  private BoundingBox3d extensionEnvelope = null;
  private boolean collisionCheckingEnabled = true;
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
   * Attaches a {@link CadDrivetrainConfig}, populating the chassis asset, field pose supplier,
   * and articulated swerve/wheel components.
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
   * Sets an output directory to automatically export an AdvantageScope {@code Robot_<Name>}
   * bundle on simulator startup.
   *
   * @param p Parent output directory, or {@code null} to disable automatic export.
   * @return This config for chaining.
   */
  public CadRobotConfig withAdvantageScopeExportDir(Path p) {
    this.advantageScopeOutputDir = p;
    return this;
  }

  /**
   * Sets the legal robot-frame extension envelope checked against every collision-enabled
   * component.
   *
   * @param b Allowed {@link BoundingBox3d} in meters, or {@code null} to disable.
   * @return This config for chaining.
   */
  public CadRobotConfig withExtensionEnvelope(BoundingBox3d b) {
    this.extensionEnvelope = b;
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
   * Ignores OBB collision checks between two named components (order-independent). Also supports
   * {@code "Chassis"} and {@code "ExtensionEnvelope"} as target names.
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

  private static String canonicalPairKey(String a, String b) {
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

  /** @return Legal extension envelope, if configured. */
  public Optional<BoundingBox3d> getExtensionEnvelope() {
    return Optional.ofNullable(extensionEnvelope);
  }

  /** @return {@code true} if collision checking is enabled. */
  public boolean isCollisionCheckingEnabled() {
    return collisionCheckingEnabled;
  }

  /** @return {@code true} if AdvantageScope mesh simplification is disabled. */
  public boolean isDisableAdvantageScopeSimplification() {
    return disableAdvantageScopeSimplification;
  }
}
