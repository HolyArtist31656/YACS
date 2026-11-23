// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.yams;

import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

public final class YamsMechanismBridge {
  private YamsMechanismBridge() {}

  public static Supplier<Pose3d> fromPoseSupplier(
      Supplier<Pose3d> yamsPoseSupplier, Transform3d cadMountOffset) {
    Objects.requireNonNull(yamsPoseSupplier);
    Objects.requireNonNull(cadMountOffset);
    return () -> yamsPoseSupplier.get().transformBy(cadMountOffset);
  }

  public static Supplier<Pose3d> chainParentChild(
      Supplier<Pose3d> parentPoseSupplier, Supplier<Pose3d> childLocalPoseSupplier) {
    Objects.requireNonNull(parentPoseSupplier);
    Objects.requireNonNull(childLocalPoseSupplier);
    return () -> {
      Pose3d parent = parentPoseSupplier.get();
      Pose3d child = childLocalPoseSupplier.get();
      return parent.transformBy(new Transform3d(child.getTranslation(), child.getRotation()));
    };
  }
}
