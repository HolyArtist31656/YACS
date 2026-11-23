// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.cad;

import java.nio.file.Path;
import java.util.Locale;

public enum MeshFormat {
  STL(".stl"),
  OBJ(".obj"),
  GLB(".glb"),
  GLTF(".gltf");

  private final String extension;

  MeshFormat(String extension) { this.extension = extension; }
  public String getExtension() { return extension; }

  public static MeshFormat fromPath(Path path) {
    if (path == null || path.getFileName() == null) {
      throw new CadMeshValidationException("Mesh path cannot be null.");
    }
    String name = path.getFileName().toString().toLowerCase(Locale.ROOT);
    for (MeshFormat format : values()) {
      if (name.endsWith(format.extension)) return format;
    }
    throw new CadMeshValidationException(
        "Unsupported CAD mesh format for file '" + name + "'. Expected: .glb, .gltf, .stl, .obj");
  }
}
