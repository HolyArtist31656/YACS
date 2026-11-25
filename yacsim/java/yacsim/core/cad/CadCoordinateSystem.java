// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.cad;

import java.util.List;

/**
 * Standard CAD export coordinate-system presets that configure unit scaling and ordered
 * AdvantageScope zeroed rotations to align CAD axes with WPILib's NWU frame (+X forward, +Y left,
 * +Z up, meters).
 */
public enum CadCoordinateSystem {
  /** Native WPILib NWU frame in meters with no additional rotation. */
  WPILIB_NWU(1.0, List.of()),
  /** Onshape glTF/GLB export (meters, Y-up): rotates X +90° then Z +90°. */
  ONSHAPE_GLTF(
      1.0,
      List.of(
          new CadAssetConfig.AxisRotation("x", 90.0),
          new CadAssetConfig.AxisRotation("z", 90.0))),
  /** SolidWorks millimeter export (Y-up): scales by 0.001 and rotates X +90° then Z +90°. */
  SOLIDWORKS_MM(
      0.001,
      List.of(
          new CadAssetConfig.AxisRotation("x", 90.0),
          new CadAssetConfig.AxisRotation("z", 90.0))),
  /** Autodesk Fusion 360 millimeter export (Z-up): scales by 0.001 with no rotation. */
  FUSION_360_MM(0.001, List.of()),
  /** Blender default glTF export (meters, Y-up): rotates X +90°. */
  BLENDER_GLTF(1.0, List.of(new CadAssetConfig.AxisRotation("x", 90.0)));

  private final double unitScaleToMeters;
  private final List<CadAssetConfig.AxisRotation> zeroedRotations;

  CadCoordinateSystem(double unitScaleToMeters, List<CadAssetConfig.AxisRotation> zeroedRotations) {
    this.unitScaleToMeters = unitScaleToMeters;
    this.zeroedRotations = zeroedRotations;
  }

  /** @return Unit scale factor converting exported CAD units to meters. */
  public double getUnitScaleToMeters() {
    return unitScaleToMeters;
  }

  /** @return Ordered AdvantageScope zeroed axis rotations for this preset. */
  public List<CadAssetConfig.AxisRotation> getZeroedRotations() {
    return zeroedRotations;
  }
}
