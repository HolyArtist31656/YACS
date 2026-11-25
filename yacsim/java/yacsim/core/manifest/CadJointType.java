// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.manifest;

import java.util.Locale;

/**
 * Kinematic joint classification used in {@code yacsim-robot.json} manifests and imported URDF
 * robot descriptions.
 */
public enum CadJointType {
  /** Rigid attachment between parent and child links with zero degrees of freedom. */
  FIXED,
  /** Bounded rotational joint (for example an arm, hood, wrist, or limited turret). */
  REVOLUTE,
  /** Bounded linear translation joint (for example an elevator, climber, or compactor). */
  PRISMATIC,
  /** Unbounded rotational joint (for example a flywheel, intake roller, kicker, or wheel). */
  CONTINUOUS;

  /**
   * Parses a case-insensitive joint type string from JSON or URDF XML.
   *
   * @param raw Joint type string (for example {@code "revolute"} or {@code "PRISMATIC"}).
   * @return Matching {@link CadJointType}.
   * @throws IllegalArgumentException If {@code raw} is {@code null} or unrecognized.
   */
  public static CadJointType fromString(String raw) {
    if (raw == null || raw.isBlank()) {
      throw new IllegalArgumentException("Joint type cannot be null or blank.");
    }
    return switch (raw.trim().toLowerCase(Locale.ROOT)) {
      case "fixed" -> FIXED;
      case "revolute" -> REVOLUTE;
      case "prismatic" -> PRISMATIC;
      case "continuous" -> CONTINUOUS;
      default -> throw new IllegalArgumentException("Unsupported CAD joint type: " + raw);
    };
  }
}
