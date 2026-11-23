// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.drivetrain;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.model.CadComponentConfig;

/**
 * CAD configuration for a robot drivetrain chassis, 3D gyro orientation, and articulated swerve
 * modules or differential wheels.
 */
public final class CadDrivetrainConfig {
  private final CadAssetConfig chassisAsset;
  private Supplier<Pose2d> fieldPoseSupplier = () -> new Pose2d();
  private Supplier<Rotation3d> gyroRotation3dSupplier = () -> new Rotation3d();
  private final List<CadSwerveModuleConfig> swerveModules = new ArrayList<>();
  private final List<CadComponentConfig> differentialWheels = new ArrayList<>();
  private boolean chassisCollisionEnabled = true;

  /**
   * Creates a drivetrain configuration with the given chassis CAD asset.
   *
   * @param chassisAsset Chassis mesh asset configuration.
   */
  public CadDrivetrainConfig(CadAssetConfig chassisAsset) {
    this.chassisAsset = Objects.requireNonNull(chassisAsset);
  }

  /**
   * Sets the supplier for the robot's 2D field pose.
   *
   * @param s Supplier returning the current field-relative {@link Pose2d}.
   * @return This config for chaining.
   */
  public CadDrivetrainConfig withFieldPoseSupplier(Supplier<Pose2d> s) { this.fieldPoseSupplier = Objects.requireNonNull(s); return this; }
  /**
   * Sets the supplier for 3D gyro roll and pitch angles.
   *
   * @param s Supplier returning the current {@link Rotation3d} (roll/pitch used with 2D yaw).
   * @return This config for chaining.
   */
  public CadDrivetrainConfig withGyroRotation3dSupplier(Supplier<Rotation3d> s) { this.gyroRotation3dSupplier = Objects.requireNonNull(s); return this; }
  /**
   * Registers an articulated swerve module (steer fork and/or wheel).
   *
   * @param m Swerve module configuration.
   * @return This config for chaining.
   */
  public CadDrivetrainConfig addSwerveModule(CadSwerveModuleConfig m) { this.swerveModules.add(Objects.requireNonNull(m)); return this; }

  /**
   * Registers a spinning differential-drive wheel component.
   *
   * @param name Unique component name.
   * @param wheelAsset Wheel CAD asset.
   * @param mount Mounting location relative to chassis origin in meters.
   * @param wheelRadiusMeters Positive wheel radius in meters.
   * @param distSupplier Supplier returning cumulative wheel travel distance in meters.
   * @return This config for chaining.
   */
  public CadDrivetrainConfig addDifferentialWheel(
      String name, CadAssetConfig wheelAsset, Translation3d mount, double wheelRadiusMeters, DoubleSupplier distSupplier) {
    if (wheelRadiusMeters <= 0.0) throw new IllegalArgumentException("wheelRadiusMeters must be positive.");
    Objects.requireNonNull(distSupplier);
    differentialWheels.add(new CadComponentConfig(name, wheelAsset)
        .withParentComponent("Chassis").withCollisionEnabled(false)
        .withPoseSupplier(() -> new Pose3d(mount, new Rotation3d(0.0, distSupplier.getAsDouble() / wheelRadiusMeters, 0.0))));
    return this;
  }

  /**
   * Enables or disables OBB collision checks against the chassis bounding box.
   *
   * @param e {@code true} to test non-child mechanisms against the chassis.
   * @return This config for chaining.
   */
  public CadDrivetrainConfig withChassisCollisionEnabled(boolean e) { this.chassisCollisionEnabled = e; return this; }
  /** @return Configured chassis CAD asset. */
  public CadAssetConfig getChassisAsset() { return chassisAsset; }
  /** @return Supplier for the robot's 2D field pose. */
  public Supplier<Pose2d> getFieldPoseSupplier() { return fieldPoseSupplier; }
  /** @return Supplier for 3D gyro orientation. */
  public Supplier<Rotation3d> getGyroRotation3dSupplier() { return gyroRotation3dSupplier; }
  /** @return {@code true} if chassis collision checking is enabled. */
  public boolean isChassisCollisionEnabled() { return chassisCollisionEnabled; }
  /** @return Unmodifiable list of registered swerve modules. */
  public List<CadSwerveModuleConfig> getSwerveModules() { return Collections.unmodifiableList(swerveModules); }

  /**
   * Combines the 2D field pose with 3D gyro roll and pitch into a {@link Pose3d}.
   *
   * @return Current 3D field pose of the robot chassis.
   */
  public Pose3d computeRobotPose3d() {
    Pose2d pose2d = fieldPoseSupplier.get();
    Rotation3d gyro = gyroRotation3dSupplier.get();
    return new Pose3d(
        new Translation3d(pose2d.getX(), pose2d.getY(), 0.0),
        new Rotation3d(gyro.getX(), gyro.getY(), pose2d.getRotation().getRadians()));
  }

  /**
   * Flattens all registered swerve modules and differential wheels into articulated components.
   *
   * @return List of {@link CadComponentConfig} entries in registration order.
   */
  public List<CadComponentConfig> toArticulatedComponents() {
    List<CadComponentConfig> out = new ArrayList<>();
    for (CadSwerveModuleConfig mod : swerveModules) out.addAll(mod.toCadComponents());
    out.addAll(differentialWheels);
    return out;
  }

  /**
   * Looks up a registered swerve module by name.
   *
   * @param name Module name (for example {@code "FrontLeft"}).
   * @return Matching module configuration, if present.
   */
  public Optional<CadSwerveModuleConfig> getSwerveModule(String name) {
    return swerveModules.stream().filter(m -> m.getName().equals(name)).findFirst();
  }
}
