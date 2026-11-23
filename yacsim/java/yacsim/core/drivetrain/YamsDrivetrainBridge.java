// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.drivetrain;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import yacsim.core.cad.CadAssetConfig;

public final class YamsDrivetrainBridge {
  private YamsDrivetrainBridge() {}

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
