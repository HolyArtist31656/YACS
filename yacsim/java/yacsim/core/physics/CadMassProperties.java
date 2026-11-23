// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.physics;

import org.wpilib.math.geometry.Translation3d;

public record CadMassProperties(
    double volumeCubicMeters,
    double massKilograms,
    Translation3d centerOfMassMeters,
    double ixxKgM2,
    double iyyKgM2,
    double izzKgM2) {

  public CadMassProperties withTotalMassKg(double targetMassKg) {
    if (targetMassKg <= 0.0) throw new IllegalArgumentException("Target mass must be positive.");
    if (massKilograms <= 0.0) {
      return new CadMassProperties(
          volumeCubicMeters, targetMassKg, centerOfMassMeters, ixxKgM2, iyyKgM2, izzKgM2);
    }
    double r = targetMassKg / massKilograms;
    return new CadMassProperties(
        volumeCubicMeters, targetMassKg, centerOfMassMeters, ixxKgM2 * r, iyyKgM2 * r, izzKgM2 * r);
  }

  public static CadMassProperties fromBoundingBox(BoundingBox3d box, double densityKgPerM3) {
    double lx = Math.max(1e-6, box.sizeX());
    double ly = Math.max(1e-6, box.sizeY());
    double lz = Math.max(1e-6, box.sizeZ());
    double vol = lx * ly * lz;
    double m = vol * densityKgPerM3;
    return new CadMassProperties(
        vol, m, box.center(),
        (m / 12.0) * (ly * ly + lz * lz),
        (m / 12.0) * (lx * lx + lz * lz),
        (m / 12.0) * (lx * lx + ly * ly));
  }
}
