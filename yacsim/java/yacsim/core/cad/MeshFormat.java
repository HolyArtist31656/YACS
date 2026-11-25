// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.cad;

import java.nio.file.Path;
import java.util.Locale;

/** Supported CAD mesh file formats parsed by {@link MeshParser}. */
public enum MeshFormat {
  STL(".stl"),
  OBJ(".obj"),
  GLB(".glb"),
  GLTF(".gltf");

  private final String extension;

  MeshFormat(String extension) { this.extension = extension; }
  /**
   * Returns the lowercase file extension including the leading dot.
   *
   * @return File extension (for example {@code ".glb"}).
   */
  public String getExtension() { return extension; }

  /**
   * Infers the {@link MeshFormat} from a file path's extension.
   *
   * @param path Path to the CAD mesh file.
   * @return Matching {@link MeshFormat}.
   * @throws CadMeshValidationException If the path is null or has an unsupported extension.
   */
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
