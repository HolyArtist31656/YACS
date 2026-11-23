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

  public CadRobotConfig(String robotName) {
    this.robotName = Objects.requireNonNull(robotName);
  }

  public CadRobotConfig withDrivetrain(CadDrivetrainConfig cfg) {
    this.drivetrainConfig = Objects.requireNonNull(cfg);
    this.chassisAsset = cfg.getChassisAsset();
    this.drivetrainPoseSupplier = cfg.getFieldPoseSupplier();
    return this;
  }

  public CadRobotConfig withChassisAsset(CadAssetConfig a) {
    this.chassisAsset = Objects.requireNonNull(a);
    return this;
  }

  public CadRobotConfig addComponent(CadComponentConfig c) {
    this.components.add(Objects.requireNonNull(c));
    return this;
  }

  public CadRobotConfig withDrivetrainPoseSupplier(Supplier<Pose2d> s) {
    this.drivetrainPoseSupplier = Objects.requireNonNull(s);
    return this;
  }

  public CadRobotConfig withTelemetryTable(String t) {
    this.telemetryTable = Objects.requireNonNull(t);
    return this;
  }

  public CadRobotConfig withAdvantageScopeExportDir(Path p) {
    this.advantageScopeOutputDir = p;
    return this;
  }

  public CadRobotConfig withExtensionEnvelope(BoundingBox3d b) {
    this.extensionEnvelope = b;
    return this;
  }

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

  public boolean isCollisionPairIgnored(String componentA, String componentB) {
    if (componentA == null || componentB == null) {
      return false;
    }
    return ignoredCollisionPairs.contains(canonicalPairKey(componentA, componentB));
  }

  private static String canonicalPairKey(String a, String b) {
    return a.compareTo(b) <= 0 ? a + "\u0000" + b : b + "\u0000" + a;
  }

  public CadRobotConfig withDisableAdvantageScopeSimplification(boolean d) {
    this.disableAdvantageScopeSimplification = d;
    return this;
  }

  public String getRobotName() {
    return robotName;
  }

  public Optional<CadDrivetrainConfig> getDrivetrainConfig() {
    return Optional.ofNullable(drivetrainConfig);
  }

  public Optional<CadAssetConfig> getChassisAsset() {
    return Optional.ofNullable(chassisAsset);
  }

  public List<CadComponentConfig> getComponents() {
    return Collections.unmodifiableList(components);
  }

  public List<CadComponentConfig> getAllArticulatedComponents() {
    List<CadComponentConfig> all = new ArrayList<>();
    if (drivetrainConfig != null) {
      all.addAll(drivetrainConfig.toArticulatedComponents());
    }
    all.addAll(components);
    return Collections.unmodifiableList(all);
  }

  public Supplier<Pose2d> getDrivetrainPoseSupplier() {
    return drivetrainPoseSupplier;
  }

  public String getTelemetryTable() {
    return telemetryTable;
  }

  public Optional<Path> getAdvantageScopeOutputDir() {
    return Optional.ofNullable(advantageScopeOutputDir);
  }

  public Optional<BoundingBox3d> getExtensionEnvelope() {
    return Optional.ofNullable(extensionEnvelope);
  }

  public boolean isCollisionCheckingEnabled() {
    return collisionCheckingEnabled;
  }

  public boolean isDisableAdvantageScopeSimplification() {
    return disableAdvantageScopeSimplification;
  }
}
