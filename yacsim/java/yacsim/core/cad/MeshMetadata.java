// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.cad;

import java.nio.file.Path;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.CadMassProperties;

public record MeshMetadata(
    Path sourcePath,
    MeshFormat format,
    int triangleCount,
    long fileSizeBytes,
    BoundingBox3d localBounds,
    CadMassProperties massProperties) {}
