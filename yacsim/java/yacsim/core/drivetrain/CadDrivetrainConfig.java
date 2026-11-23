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

public final class CadDrivetrainConfig {
  private final CadAssetConfig chassisAsset;
  private Supplier<Pose2d> fieldPoseSupplier = () -> new Pose2d();
  private Supplier<Rotation3d> gyroRotation3dSupplier = () -> new Rotation3d();
  private final List<CadSwerveModuleConfig> swerveModules = new ArrayList<>();
  private final List<CadComponentConfig> differentialWheels = new ArrayList<>();
  private boolean chassisCollisionEnabled = true;

  public CadDrivetrainConfig(CadAssetConfig chassisAsset) {
    this.chassisAsset = Objects.requireNonNull(chassisAsset);
  }

  public CadDrivetrainConfig withFieldPoseSupplier(Supplier<Pose2d> s) { this.fieldPoseSupplier = Objects.requireNonNull(s); return this; }
  public CadDrivetrainConfig withGyroRotation3dSupplier(Supplier<Rotation3d> s) { this.gyroRotation3dSupplier = Objects.requireNonNull(s); return this; }
  public CadDrivetrainConfig addSwerveModule(CadSwerveModuleConfig m) { this.swerveModules.add(Objects.requireNonNull(m)); return this; }

  public CadDrivetrainConfig addDifferentialWheel(
      String name, CadAssetConfig wheelAsset, Translation3d mount, double wheelRadiusMeters, DoubleSupplier distSupplier) {
    if (wheelRadiusMeters <= 0.0) throw new IllegalArgumentException("wheelRadiusMeters must be positive.");
    Objects.requireNonNull(distSupplier);
    differentialWheels.add(new CadComponentConfig(name, wheelAsset)
        .withParentComponent("Chassis").withCollisionEnabled(false)
        .withPoseSupplier(() -> new Pose3d(mount, new Rotation3d(0.0, distSupplier.getAsDouble() / wheelRadiusMeters, 0.0))));
    return this;
  }

  public CadDrivetrainConfig withChassisCollisionEnabled(boolean e) { this.chassisCollisionEnabled = e; return this; }
  public CadAssetConfig getChassisAsset() { return chassisAsset; }
  public Supplier<Pose2d> getFieldPoseSupplier() { return fieldPoseSupplier; }
  public Supplier<Rotation3d> getGyroRotation3dSupplier() { return gyroRotation3dSupplier; }
  public boolean isChassisCollisionEnabled() { return chassisCollisionEnabled; }
  public List<CadSwerveModuleConfig> getSwerveModules() { return Collections.unmodifiableList(swerveModules); }

  public Pose3d computeRobotPose3d() {
    Pose2d pose2d = fieldPoseSupplier.get();
    Rotation3d gyro = gyroRotation3dSupplier.get();
    return new Pose3d(
        new Translation3d(pose2d.getX(), pose2d.getY(), 0.0),
        new Rotation3d(gyro.getX(), gyro.getY(), pose2d.getRotation().getRadians()));
  }

  public List<CadComponentConfig> toArticulatedComponents() {
    List<CadComponentConfig> out = new ArrayList<>();
    for (CadSwerveModuleConfig mod : swerveModules) out.addAll(mod.toCadComponents());
    out.addAll(differentialWheels);
    return out;
  }

  public Optional<CadSwerveModuleConfig> getSwerveModule(String name) {
    return swerveModules.stream().filter(m -> m.getName().equals(name)).findFirst();
  }
}
