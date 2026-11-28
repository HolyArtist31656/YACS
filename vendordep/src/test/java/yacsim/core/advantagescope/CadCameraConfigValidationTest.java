// Copyright (c) 2026 Nathan Moe / Yet Another CAD Simulator
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.advantagescope;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/** Regression tests ensuring CadCameraConfig rejects NaN and non-finite FOV values. */
class CadCameraConfigValidationTest {
  private static final Transform3d ORIGIN = new Transform3d(new Translation3d(), new Rotation3d());

  @Test
  void rejectsNonFiniteAndOutOfRangeFovDegrees() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new CadCameraConfig("front_cam", ORIGIN, Double.NaN, 1280, 720));
    assertThrows(
        IllegalArgumentException.class,
        () -> new CadCameraConfig("front_cam", ORIGIN, Double.POSITIVE_INFINITY, 1280, 720));
    assertThrows(
        IllegalArgumentException.class,
        () -> new CadCameraConfig("front_cam", ORIGIN, Double.NEGATIVE_INFINITY, 1280, 720));
    assertThrows(
        IllegalArgumentException.class,
        () -> new CadCameraConfig("front_cam", ORIGIN, 0.0, 1280, 720));
    assertThrows(
        IllegalArgumentException.class,
        () -> new CadCameraConfig("front_cam", ORIGIN, 180.0, 1280, 720));
  }

  @Test
  void acceptsValidFiniteFovDegrees() {
    CadCameraConfig cfg = new CadCameraConfig("front_cam", ORIGIN, 70.0, 1280, 720);
    assertEquals(70.0, cfg.fovDegrees(), 1e-9);
  }
}
