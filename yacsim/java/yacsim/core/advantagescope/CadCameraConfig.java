// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.advantagescope;

import java.util.Objects;
import org.wpilib.math.geometry.Transform3d;

/**
 * AdvantageScope 3D camera frustum definition exported into the {@code cameras} array of {@code
 * config.json}.
 *
 * @param name Camera display name in AdvantageScope (for example {@code "FrontLeftCam"}).
 * @param robotToCamera Robot-to-camera mounting transform in meters and radians.
 * @param fovDegrees Horizontal field of view in degrees (must be positive).
 * @param resolutionWidthPx Image width in pixels (must be positive).
 * @param resolutionHeightPx Image height in pixels (must be positive).
 */
public record CadCameraConfig(
    String name,
    Transform3d robotToCamera,
    double fovDegrees,
    int resolutionWidthPx,
    int resolutionHeightPx) {

  /** Compact constructor validating camera parameters. */
  public CadCameraConfig {
    Objects.requireNonNull(name, "name");
    Objects.requireNonNull(robotToCamera, "robotToCamera");
    if (!Double.isFinite(fovDegrees) || fovDegrees <= 0.0 || fovDegrees >= 180.0) {
      throw new IllegalArgumentException("fovDegrees must be in (0, 180).");
    }
    if (resolutionWidthPx <= 0 || resolutionHeightPx <= 0) {
      throw new IllegalArgumentException("Camera resolution must be positive.");
    }
  }
}
