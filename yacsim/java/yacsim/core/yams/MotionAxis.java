// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.yams;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Principal robot-frame axis along which a mechanism rotates or translates in WPILib NWU
 * coordinates (X forward / roll, Y left / pitch, Z up / yaw).
 */
public enum MotionAxis {
  /** X axis (forward translation or roll rotation). */
  X,
  /** Y axis (left translation or pitch rotation, typical for FRC arms and hoods). */
  Y,
  /** Z axis (vertical translation for elevators, or yaw rotation for turrets/pivots). */
  Z;

  /**
   * Creates a {@link Rotation3d} rotating by {@code radians} around this axis.
   *
   * @param radians Rotation angle in radians.
   * @return Single-axis {@link Rotation3d}.
   */
  public Rotation3d toRotation3d(double radians) {
    return switch (this) {
      case X -> new Rotation3d(radians, 0.0, 0.0);
      case Y -> new Rotation3d(0.0, radians, 0.0);
      case Z -> new Rotation3d(0.0, 0.0, radians);
    };
  }

  /**
   * Creates a {@link Translation3d} translating by {@code meters} along this axis.
   *
   * @param meters Linear displacement in meters.
   * @return Single-axis {@link Translation3d}.
   */
  public Translation3d toTranslation3d(double meters) {
    return switch (this) {
      case X -> new Translation3d(meters, 0.0, 0.0);
      case Y -> new Translation3d(0.0, meters, 0.0);
      case Z -> new Translation3d(0.0, 0.0, meters);
    };
  }
}
