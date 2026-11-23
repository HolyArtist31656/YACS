// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.cad;

import java.nio.file.Path;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.CadMassProperties;

/**
 * Immutable metadata extracted from a parsed CAD mesh without retaining GPU vertex buffers.
 *
 * @param sourcePath Path to the source CAD file.
 * @param format Detected {@link MeshFormat}.
 * @param triangleCount Total triangle count reported by the mesh.
 * @param fileSizeBytes Size of the source file in bytes.
 * @param localBounds Axis-aligned local bounding box in meters.
 * @param massProperties Estimated or overridden mass and principal inertia properties.
 */
public record MeshMetadata(
    Path sourcePath,
    MeshFormat format,
    int triangleCount,
    long fileSizeBytes,
    BoundingBox3d localBounds,
    CadMassProperties massProperties) {}
