// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.cad;

import java.nio.file.Path;
import java.util.List;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.CadMassProperties;

/**
 * Immutable metadata extracted from a parsed CAD mesh without retaining GPU vertex buffers.
 *
 * @param sourcePath Path to the source CAD file.
 * @param format Detected {@link MeshFormat}.
 * @param triangleCount Total triangle count reported by the mesh.
 * @param fileSizeBytes Size of the source file in bytes.
 * @param localBounds Overall axis-aligned local bounding box in meters.
 * @param massProperties Estimated or overridden mass and principal inertia properties.
 * @param collisionSubBounds One or more local sub-bounding boxes used for multi-primitive OBB
 *     collision checks (defaults to {@code List.of(localBounds)}).
 */
public record MeshMetadata(
    Path sourcePath,
    MeshFormat format,
    int triangleCount,
    long fileSizeBytes,
    BoundingBox3d localBounds,
    CadMassProperties massProperties,
    List<BoundingBox3d> collisionSubBounds) {

  /** Compact constructor ensuring {@code collisionSubBounds} is always non-empty and immutable. */
  public MeshMetadata {
    collisionSubBounds =
        (collisionSubBounds == null || collisionSubBounds.isEmpty())
            ? List.of(localBounds)
            : List.copyOf(collisionSubBounds);
  }

  /**
   * Convenience constructor defaulting {@code collisionSubBounds} to {@code List.of(localBounds)}.
   *
   * @param sourcePath Path to the source CAD file.
   * @param format Detected {@link MeshFormat}.
   * @param triangleCount Total triangle count reported by the mesh.
   * @param fileSizeBytes Size of the source file in bytes.
   * @param localBounds Overall axis-aligned local bounding box in meters.
   * @param massProperties Estimated or overridden mass and principal inertia properties.
   */
  public MeshMetadata(
      Path sourcePath,
      MeshFormat format,
      int triangleCount,
      long fileSizeBytes,
      BoundingBox3d localBounds,
      CadMassProperties massProperties) {
    this(
        sourcePath,
        format,
        triangleCount,
        fileSizeBytes,
        localBounds,
        massProperties,
        List.of(localBounds));
  }
}
