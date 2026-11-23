// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.model.CadComponentConfig;

/**
 * CAD configuration for a single swerve module, animating both steer-fork yaw and wheel spin
 * from a WPILib {@link SwerveModulePosition} supplier.
 */
public final class CadSwerveModuleConfig {
  private final String name;
  private final Translation3d mountingLocationMeters;
  private double wheelRadiusMeters = 0.0508;
  private CadAssetConfig steerForkAsset = null;
  private CadAssetConfig wheelAsset = null;
  private Supplier<SwerveModulePosition> modulePositionSupplier = SwerveModulePosition::new;

  /**
   * Creates a swerve module configuration.
   *
   * @param name Unique module name (for example {@code "FrontLeft"}).
   * @param mountingLocationMeters Module mounting position relative to the chassis origin in meters.
   */
  public CadSwerveModuleConfig(String name, Translation3d mountingLocationMeters) {
    this.name = Objects.requireNonNull(name);
    this.mountingLocationMeters = Objects.requireNonNull(mountingLocationMeters);
  }

  /**
   * Sets the wheel radius used to convert drive distance into wheel spin radians.
   *
   * @param r Positive wheel radius in meters.
   * @return This config for chaining.
   */
  public CadSwerveModuleConfig withWheelRadiusMeters(double r) {
    if (r <= 0.0) throw new IllegalArgumentException("wheelRadiusMeters must be positive.");
    this.wheelRadiusMeters = r;
    return this;
  }

  /**
   * Sets the CAD asset for the rotating steer fork.
   *
   * @param a Steer fork CAD asset.
   * @return This config for chaining.
   */
  public CadSwerveModuleConfig withSteerForkAsset(CadAssetConfig a) { this.steerForkAsset = Objects.requireNonNull(a); return this; }
  /**
   * Sets the CAD asset for the spinning drive wheel.
   *
   * @param a Wheel CAD asset.
   * @return This config for chaining.
   */
  public CadSwerveModuleConfig withWheelAsset(CadAssetConfig a) { this.wheelAsset = Objects.requireNonNull(a); return this; }
  /**
   * Sets the supplier for this module's {@link SwerveModulePosition}.
   *
   * @param s Module position supplier.
   * @return This config for chaining.
   */
  public CadSwerveModuleConfig withModulePositionSupplier(Supplier<SwerveModulePosition> s) { this.modulePositionSupplier = Objects.requireNonNull(s); return this; }

  /** @return Module name. */
  public String getName() { return name; }
  /** @return Mounting location in meters relative to chassis origin. */
  public Translation3d getMountingLocationMeters() { return mountingLocationMeters; }
  /** @return Wheel radius in meters. */
  public double getWheelRadiusMeters() { return wheelRadiusMeters; }
  /** @return Steer fork CAD asset, if configured. */
  public Optional<CadAssetConfig> getSteerForkAsset() { return Optional.ofNullable(steerForkAsset); }
  /** @return Wheel CAD asset, if configured. */
  public Optional<CadAssetConfig> getWheelAsset() { return Optional.ofNullable(wheelAsset); }

  /**
   * Computes the current robot-relative 3D pose of the steer fork.
   *
   * @return Steer fork {@link Pose3d}.
   */
  public Pose3d computeSteerForkPose3d() {
    SwerveModulePosition pos = modulePositionSupplier.get();
    double yawRad = pos.angle != null ? pos.angle.getRadians() : 0.0;
    return new Pose3d(mountingLocationMeters, new Rotation3d(0.0, 0.0, yawRad));
  }

  /**
   * Computes the current robot-relative 3D pose of the wheel including both steer yaw and spin.
   *
   * @return Wheel {@link Pose3d}.
   */
  public Pose3d computeWheelPose3d() {
    SwerveModulePosition pos = modulePositionSupplier.get();
    double yawRad = pos.angle != null ? pos.angle.getRadians() : 0.0;
    double spinRad = pos.distance / wheelRadiusMeters;
    return new Pose3d(mountingLocationMeters, new Rotation3d(0.0, spinRad, 0.0).rotateBy(new Rotation3d(0.0, 0.0, yawRad)));
  }

  /**
   * Converts this module's configured steer fork and wheel assets into articulated components.
   *
   * @return List containing 0 to 2 {@link CadComponentConfig} entries.
   */
  public List<CadComponentConfig> toCadComponents() {
    List<CadComponentConfig> list = new ArrayList<>(2);
    if (steerForkAsset != null) {
      list.add(new CadComponentConfig(name + "_Steer", steerForkAsset)
          .withParentComponent("Chassis").withCollisionEnabled(false).withPoseSupplier(this::computeSteerForkPose3d));
    }
    if (wheelAsset != null) {
      list.add(new CadComponentConfig(name + "_Wheel", wheelAsset)
          .withParentComponent(steerForkAsset != null ? name + "_Steer" : "Chassis")
          .withCollisionEnabled(false).withPoseSupplier(this::computeWheelPose3d));
    }
    return list;
  }
}
