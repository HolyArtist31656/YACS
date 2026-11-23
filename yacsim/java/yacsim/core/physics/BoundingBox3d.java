// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.physics;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;

public record BoundingBox3d(
    double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {

  public BoundingBox3d {
    if (minX > maxX || minY > maxY || minZ > maxZ) {
      throw new IllegalArgumentException("BoundingBox3d min must be <= max.");
    }
  }

  public Translation3d center() {
    return new Translation3d((minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5);
  }

  public double halfX() { return (maxX - minX) * 0.5; }
  public double halfY() { return (maxY - minY) * 0.5; }
  public double halfZ() { return (maxZ - minZ) * 0.5; }
  public double sizeX() { return maxX - minX; }
  public double sizeY() { return maxY - minY; }
  public double sizeZ() { return maxZ - minZ; }

  public BoundingBox3d scaled(double scaleFactor) {
    if (scaleFactor <= 0.0) throw new IllegalArgumentException("Scale factor must be positive.");
    return new BoundingBox3d(
        minX * scaleFactor, minY * scaleFactor, minZ * scaleFactor,
        maxX * scaleFactor, maxY * scaleFactor, maxZ * scaleFactor);
  }

  public BoundingBox3d expandedForExtension(double xyMarginMeters, double maxHeightMeters) {
    return new BoundingBox3d(
        minX - xyMarginMeters, minY - xyMarginMeters, 0.0,
        maxX + xyMarginMeters, maxY + xyMarginMeters, maxHeightMeters);
  }

  public Translation3d[] transformedCorners(Pose3d pose) {
    double[] xs = {minX, maxX}, ys = {minY, maxY}, zs = {minZ, maxZ};
    Translation3d[] corners = new Translation3d[8];
    int idx = 0;
    for (double x : xs)
      for (double y : ys)
        for (double z : zs)
          corners[idx++] = new Translation3d(x, y, z).rotateBy(pose.getRotation()).plus(pose.getTranslation());
    return corners;
  }
}
