// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

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
    if (fovDegrees <= 0.0 || fovDegrees >= 180.0) {
      throw new IllegalArgumentException("fovDegrees must be in (0, 180).");
    }
    if (resolutionWidthPx <= 0 || resolutionHeightPx <= 0) {
      throw new IllegalArgumentException("Camera resolution must be positive.");
    }
  }
}
