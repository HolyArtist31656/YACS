// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.model;

import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import yacsim.core.cad.CadAssetConfig;

public final class CadComponentConfig {
  private final String name;
  private final CadAssetConfig assetConfig;
  private Supplier<Pose3d> poseSupplier = () -> new Pose3d();
  private String parentComponentName = null;
  private boolean collisionEnabled = true;

  public CadComponentConfig(String name, CadAssetConfig assetConfig) {
    this.name = Objects.requireNonNull(name);
    this.assetConfig = Objects.requireNonNull(assetConfig);
  }

  public CadComponentConfig withPoseSupplier(Supplier<Pose3d> s) { this.poseSupplier = Objects.requireNonNull(s); return this; }
  public CadComponentConfig withParentComponent(String p) { this.parentComponentName = p; return this; }
  public CadComponentConfig withCollisionEnabled(boolean e) { this.collisionEnabled = e; return this; }

  public String getName() { return name; }
  public CadAssetConfig getAssetConfig() { return assetConfig; }
  public Supplier<Pose3d> getPoseSupplier() { return poseSupplier; }
  public Optional<String> getParentComponentName() { return Optional.ofNullable(parentComponentName); }
  public boolean isCollisionEnabled() { return collisionEnabled; }
}
