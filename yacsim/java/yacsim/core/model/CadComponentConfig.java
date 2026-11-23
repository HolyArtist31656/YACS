// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.model;

import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import yacsim.core.cad.CadAssetConfig;

/**
 * Configuration for a single articulated CAD mechanism component (such as an arm, hood, elevator
 * carriage, or intake).
 */
public final class CadComponentConfig {
  private final String name;
  private final CadAssetConfig assetConfig;
  private Supplier<Pose3d> poseSupplier = () -> new Pose3d();
  private String parentComponentName = null;
  private boolean collisionEnabled = true;

  /**
   * Creates a named articulated CAD component.
   *
   * @param name Unique component name used in collision reports and AdvantageScope mapping.
   * @param assetConfig CAD mesh asset configuration.
   */
  public CadComponentConfig(String name, CadAssetConfig assetConfig) {
    this.name = Objects.requireNonNull(name);
    this.assetConfig = Objects.requireNonNull(assetConfig);
  }

  /**
   * Sets the supplier returning this component's robot-relative {@link Pose3d}.
   *
   * @param s Pose supplier.
   * @return This config for chaining.
   */
  public CadComponentConfig withPoseSupplier(Supplier<Pose3d> s) { this.poseSupplier = Objects.requireNonNull(s); return this; }
  /**
   * Declares the logical parent component name (for example {@code "Chassis"}). Direct
   * parent–child pairs are excluded from OBB self-collision checks.
   *
   * @param p Parent component name, or {@code null} for no parent.
   * @return This config for chaining.
   */
  public CadComponentConfig withParentComponent(String p) { this.parentComponentName = p; return this; }
  /**
   * Enables or disables OBB collision checking for this component.
   *
   * @param e {@code true} to include this component in collision checks.
   * @return This config for chaining.
   */
  public CadComponentConfig withCollisionEnabled(boolean e) { this.collisionEnabled = e; return this; }

  /** @return Component name. */
  public String getName() { return name; }
  /** @return CAD mesh asset configuration. */
  public CadAssetConfig getAssetConfig() { return assetConfig; }
  /** @return Robot-relative pose supplier. */
  public Supplier<Pose3d> getPoseSupplier() { return poseSupplier; }
  /** @return Parent component name, if configured. */
  public Optional<String> getParentComponentName() { return Optional.ofNullable(parentComponentName); }
  /** @return {@code true} if collision checking is enabled for this component. */
  public boolean isCollisionEnabled() { return collisionEnabled; }
}
