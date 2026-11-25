// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.drivetrain;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import yacsim.core.cad.CadAssetConfig;

/** Factory helpers for bridging a YAMS swerve drivetrain directly into {@link CadDrivetrainConfig}. */
public final class YamsDrivetrainBridge {
  private YamsDrivetrainBridge() {}

  /**
   * Builds a {@link CadDrivetrainConfig} wired to a YAMS swerve drivetrain's pose, gyro, and
   * module position suppliers.
   *
   * @param chassisAsset Chassis CAD mesh asset.
   * @param steerForkAsset Optional shared steer-fork CAD asset (may be {@code null}).
   * @param wheelAsset Optional shared wheel CAD asset (may be {@code null}).
   * @param wheelRadiusMeters Positive wheel radius in meters.
   * @param moduleLocations Module mounting locations in chassis frame (typically FL, FR, BL, BR).
   * @param fieldPoseSupplier Supplier returning the robot's 2D field pose.
   * @param gyroRotation3dSupplier Supplier returning 3D gyro orientation.
   * @param modulePositionsSupplier Supplier returning current {@link SwerveModulePosition} array.
   * @return Configured {@link CadDrivetrainConfig}.
   */
  public static CadDrivetrainConfig fromYamsSwerve(
      CadAssetConfig chassisAsset,
      CadAssetConfig steerForkAsset,
      CadAssetConfig wheelAsset,
      double wheelRadiusMeters,
      List<Translation3d> moduleLocations,
      Supplier<Pose2d> fieldPoseSupplier,
      Supplier<Rotation3d> gyroRotation3dSupplier,
      Supplier<SwerveModulePosition[]> modulePositionsSupplier) {
    Objects.requireNonNull(moduleLocations);
    Objects.requireNonNull(modulePositionsSupplier);

    CadDrivetrainConfig cfg =
        new CadDrivetrainConfig(chassisAsset)
            .withFieldPoseSupplier(fieldPoseSupplier)
            .withGyroRotation3dSupplier(gyroRotation3dSupplier);

    String[] defaultNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
    for (int i = 0; i < moduleLocations.size(); i++) {
      final int idx = i;
      String modName = i < defaultNames.length ? defaultNames[i] : ("Module" + i);
      CadSwerveModuleConfig mod =
          new CadSwerveModuleConfig(modName, moduleLocations.get(i))
              .withWheelRadiusMeters(wheelRadiusMeters)
              .withModulePositionSupplier(
                  () -> {
                    SwerveModulePosition[] positions = modulePositionsSupplier.get();
                    return (positions != null && idx < positions.length)
                        ? positions[idx]
                        : new SwerveModulePosition();
                  });
      if (steerForkAsset != null) mod.withSteerForkAsset(steerForkAsset);
      if (wheelAsset != null) mod.withWheelAsset(wheelAsset);
      cfg.addSwerveModule(mod);
    }
    return cfg;
  }
}
