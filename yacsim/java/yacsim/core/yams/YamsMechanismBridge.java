// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.yams;

import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * Pose-supplier composition helpers for bridging YAMS mechanism kinematics to YACSim articulated
 * CAD components.
 */
public final class YamsMechanismBridge {
  private YamsMechanismBridge() {}

  /**
   * Applies a fixed CAD mounting offset transform to a mechanism pose supplier.
   *
   * @param yamsPoseSupplier Supplier returning the mechanism's simulated {@link Pose3d}.
   * @param cadMountOffset Fixed transform from mechanism origin to CAD mesh origin.
   * @return Composed {@link Pose3d} supplier.
   */
  public static Supplier<Pose3d> fromPoseSupplier(
      Supplier<Pose3d> yamsPoseSupplier, Transform3d cadMountOffset) {
    Objects.requireNonNull(yamsPoseSupplier);
    Objects.requireNonNull(cadMountOffset);
    return () -> yamsPoseSupplier.get().transformBy(cadMountOffset);
  }

  /**
   * Chains a moving parent mechanism's pose with a child mechanism's local pose (for example, a
   * wrist mounted on an elevator carriage).
   *
   * @param parentPoseSupplier Supplier returning the parent mechanism's {@link Pose3d}.
   * @param childLocalPoseSupplier Supplier returning the child's local {@link Pose3d} relative to the parent.
   * @return Composed robot-relative {@link Pose3d} supplier for the child.
   */
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
