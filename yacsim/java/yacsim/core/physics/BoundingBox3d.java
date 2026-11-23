// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.physics;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Immutable axis-aligned 3D bounding box in meters.
 *
 * @param minX Minimum X coordinate in meters.
 * @param minY Minimum Y coordinate in meters.
 * @param minZ Minimum Z coordinate in meters.
 * @param maxX Maximum X coordinate in meters.
 * @param maxY Maximum Y coordinate in meters.
 * @param maxZ Maximum Z coordinate in meters.
 */
public record BoundingBox3d(
    double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {

  public BoundingBox3d {
    if (minX > maxX || minY > maxY || minZ > maxZ) {
      throw new IllegalArgumentException("BoundingBox3d min must be <= max.");
    }
  }

  /** @return Center point of this bounding box in meters. */
  public Translation3d center() {
    return new Translation3d((minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5);
  }

  /** @return Half-extent along the X axis in meters. */
  public double halfX() { return (maxX - minX) * 0.5; }
  /** @return Half-extent along the Y axis in meters. */
  public double halfY() { return (maxY - minY) * 0.5; }
  /** @return Half-extent along the Z axis in meters. */
  public double halfZ() { return (maxZ - minZ) * 0.5; }
  /** @return Full width along the X axis in meters. */
  public double sizeX() { return maxX - minX; }
  /** @return Full width along the Y axis in meters. */
  public double sizeY() { return maxY - minY; }
  /** @return Full height along the Z axis in meters. */
  public double sizeZ() { return maxZ - minZ; }

  /**
   * Returns a uniformly scaled copy of this bounding box.
   *
   * @param scaleFactor Positive scale multiplier.
   * @return Scaled {@link BoundingBox3d}.
   */
  public BoundingBox3d scaled(double scaleFactor) {
    if (scaleFactor <= 0.0) throw new IllegalArgumentException("Scale factor must be positive.");
    return new BoundingBox3d(
        minX * scaleFactor, minY * scaleFactor, minZ * scaleFactor,
        maxX * scaleFactor, maxY * scaleFactor, maxZ * scaleFactor);
  }

  /**
   * Expands this bounding box horizontally by {@code xyMarginMeters} and sets vertical bounds to
   * {@code [0.0, maxHeightMeters]} for FRC extension-envelope checks.
   *
   * @param xyMarginMeters Horizontal expansion margin on all four sides in meters.
   * @param maxHeightMeters Legal maximum height in meters.
   * @return Expanded extension envelope {@link BoundingBox3d}.
   */
  public BoundingBox3d expandedForExtension(double xyMarginMeters, double maxHeightMeters) {
    return new BoundingBox3d(
        minX - xyMarginMeters, minY - xyMarginMeters, 0.0,
        maxX + xyMarginMeters, maxY + xyMarginMeters, maxHeightMeters);
  }

  /**
   * Transforms all 8 corners of this local bounding box by {@code pose}.
   *
   * @param pose 3D pose transform to apply.
   * @return Array of 8 transformed corner points.
   */
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
