// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.cad;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class MeshParserTest {

  @Test
  void parsesAsciiStlUnitCube(@TempDir Path tempDir) throws IOException {
    Path stl = tempDir.resolve("cube.stl");
    Files.writeString(stl, buildAsciiCubeStl(1.0), StandardCharsets.UTF_8);

    MeshMetadata meta = MeshParser.parse(new CadAssetConfig(stl).withMaterialDensity(1000.0));

    assertEquals(MeshFormat.STL, meta.format());
    assertEquals(12, meta.triangleCount());
    assertEquals(1.0, meta.localBounds().sizeX(), 1e-6);
    assertEquals(1.0, meta.massProperties().volumeCubicMeters(), 1e-6);
    assertEquals(1000.0, meta.massProperties().massKilograms(), 1e-4);
    assertEquals(1000.0 / 6.0, meta.massProperties().ixxKgM2(), 1e-3);
  }

  @Test
  void parsesObjWithMillimeterScale(@TempDir Path tempDir) throws IOException {
    Path obj = tempDir.resolve("plate.obj");
    String content =
        """
        v 0 0 0
        v 1000 0 0
        v 1000 500 0
        v 0 500 0
        f 1 2 3 4
        """;
    Files.writeString(obj, content, StandardCharsets.UTF_8);

    MeshMetadata meta = MeshParser.parse(new CadAssetConfig(obj).withMillimeterUnits());
    assertEquals(MeshFormat.OBJ, meta.format());
    assertEquals(2, meta.triangleCount());
    assertEquals(1.0, meta.localBounds().sizeX(), 1e-6);
    assertEquals(0.5, meta.localBounds().sizeY(), 1e-6);
  }

  @Test
  void parsesGlb2HeaderWithoutLoadingGpuBuffers(@TempDir Path tempDir) throws IOException {
    String gltfJson =
        """
        {
          "asset": {"version": "2.0"},
          "accessors": [
            {"count": 36, "type": "SCALAR", "componentType": 5123},
            {"count": 8, "type": "VEC3", "componentType": 5126, "min": [-0.3, -0.3, 0.0], "max": [0.3, 0.3, 0.2]}
          ],
          "meshes": [
            {"primitives": [{"attributes": {"POSITION": 1}, "indices": 0}]}
          ]
        }
        """;
    byte[] jsonBytes = padTo4Bytes(gltfJson.getBytes(StandardCharsets.UTF_8));
    ByteBuffer glb = ByteBuffer.allocate(20 + jsonBytes.length).order(ByteOrder.LITTLE_ENDIAN);
    glb.putInt(0x46546C67);
    glb.putInt(2);
    glb.putInt(20 + jsonBytes.length);
    glb.putInt(jsonBytes.length);
    glb.putInt(0x4E4F534A);
    glb.put(jsonBytes);

    Path glbFile = tempDir.resolve("chassis.glb");
    Files.write(glbFile, glb.array());

    MeshMetadata meta = MeshParser.parse(new CadAssetConfig(glbFile).withMassOverrideKg(45.0));
    assertEquals(MeshFormat.GLB, meta.format());
    assertEquals(12, meta.triangleCount());
    assertEquals(0.6, meta.localBounds().sizeX(), 1e-6);
    assertEquals(45.0, meta.massProperties().massKilograms(), 1e-6);
  }

  @Test
  void rejectsMeshExceedingTriangleBudget(@TempDir Path tempDir) throws IOException {
    Path stl = tempDir.resolve("cube.stl");
    Files.writeString(stl, buildAsciiCubeStl(1.0), StandardCharsets.UTF_8);

    CadAssetConfig strict = new CadAssetConfig(stl).withMaxTriangleBudget(6);
    assertThrows(CadMeshValidationException.class, () -> MeshParser.parse(strict));
  }

  @Test
  void allowsHighPolyGlbWithWarningWithoutCrashing(@TempDir Path tempDir) throws IOException {
    String gltfJson =
        """
        {
          "asset": {"version": "2.0"},
          "accessors": [
            {"count": 6313638, "type": "SCALAR", "componentType": 5125},
            {"count": 8, "type": "VEC3", "componentType": 5126, "min": [-0.4, -0.4, 0.0], "max": [0.4, 0.4, 0.5]}
          ],
          "meshes": [
            {"primitives": [{"attributes": {"POSITION": 1}, "indices": 0}]}
          ]
        }
        """;
    byte[] jsonBytes = padTo4Bytes(gltfJson.getBytes(StandardCharsets.UTF_8));
    ByteBuffer glb = ByteBuffer.allocate(20 + jsonBytes.length).order(ByteOrder.LITTLE_ENDIAN);
    glb.putInt(0x46546C67);
    glb.putInt(2);
    glb.putInt(20 + jsonBytes.length);
    glb.putInt(jsonBytes.length);
    glb.putInt(0x4E4F534A);
    glb.put(jsonBytes);

    Path glbFile = tempDir.resolve("darwin_high_poly.glb");
    Files.write(glbFile, glb.array());

    MeshMetadata meta = MeshParser.parse(new CadAssetConfig(glbFile));
    assertEquals(MeshFormat.GLB, meta.format());
    assertEquals(2_104_546, meta.triangleCount());
  }

  private static byte[] padTo4Bytes(byte[] raw) {
    int rem = raw.length % 4;
    if (rem == 0) return raw;
    byte[] padded = new byte[raw.length + (4 - rem)];
    System.arraycopy(raw, 0, padded, 0, raw.length);
    for (int i = raw.length; i < padded.length; i++) padded[i] = 0x20;
    return padded;
  }

  static String buildAsciiCubeStl(double s) {
    double[][] v = {
      {0, 0, 0}, {s, 0, 0}, {s, s, 0}, {0, s, 0},
      {0, 0, s}, {s, 0, s}, {s, s, s}, {0, s, s}
    };
    int[][] tris = {
      {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
      {0, 1, 5}, {0, 5, 4}, {2, 3, 7}, {2, 7, 6},
      {1, 2, 6}, {1, 6, 5}, {3, 0, 4}, {3, 4, 7}
    };
    StringBuilder sb = new StringBuilder("solid cube\n");
    for (int[] t : tris) {
      sb.append("facet normal 0 0 0\n  outer loop\n");
      for (int idx : t) {
        sb.append(String.format("    vertex %f %f %f\n", v[idx][0], v[idx][1], v[idx][2]));
      }
      sb.append("  endloop\nendfacet\n");
    }
    sb.append("endsolid cube\n");
    return sb.toString();
  }
}
