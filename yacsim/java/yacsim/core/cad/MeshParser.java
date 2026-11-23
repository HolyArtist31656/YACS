// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.cad;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.CadMassProperties;
import yacsim.core.util.MinimalJson;

/**
 * Zero-dependency CAD mesh parser supporting header-only glTF 2.0 ({@code .glb}/{@code .gltf})
 * plus streaming Wavefront {@code .obj} and ASCII/binary {@code .stl} files.
 */
public final class MeshParser {
  private static final int GLB_MAGIC = 0x46546C67;
  private static final int GLB_JSON_CHUNK = 0x4E4F534A;
  /** Maximum per-primitive sub-OBBs retained per glTF/GLB asset to keep 50 Hz SystemCore checks fast. */
  public static final int MAX_SUB_OBBS = 64;

  private MeshParser() {}

  /**
   * Parses the CAD asset described by {@code config} and returns its bounding box, triangle count,
   * and mass properties.
   *
   * @param config Asset configuration specifying mesh path, unit scale, density, and triangle budget.
   * @return Extracted {@link MeshMetadata}.
   * @throws IOException If reading the mesh file fails.
   * @throws CadMeshValidationException If the file is missing, malformed, or exceeds budget for CPU-parsed formats.
   */
  public static MeshMetadata parse(CadAssetConfig config) throws IOException {
    Path path = config.getMeshPath();
    if (!Files.exists(path)) throw new CadMeshValidationException("CAD mesh file not found: " + path);
    MeshFormat format = MeshFormat.fromPath(path);
    long fileSize = Files.size(path);

    MeshMetadata raw =
        switch (format) {
          case STL -> parseStl(path, fileSize, config);
          case OBJ -> parseObj(path, fileSize, config);
          case GLB -> parseGlb(path, fileSize, config);
          case GLTF -> parseGltfJson(
              path, Files.readString(path, StandardCharsets.UTF_8), fileSize, MeshFormat.GLTF, config);
        };

    if (raw.triangleCount() > config.getMaxTriangleBudget()) {
      if (format == MeshFormat.GLB || format == MeshFormat.GLTF) {
        System.err.printf(
            "[YACSim] Warning: %s mesh '%s' has %d triangles (recommended SystemCore budget <= %d); "
                + "continuing because glTF/GLB header parsing uses minimal heap.%n",
            format, path.getFileName(), raw.triangleCount(), config.getMaxTriangleBudget());
      } else {
        throw new CadMeshValidationException(
            String.format(
                "Mesh '%s' has %d triangles, exceeding budget %d.",
                path.getFileName(), raw.triangleCount(), config.getMaxTriangleBudget()));
      }
    }

    CadMassProperties massProps = raw.massProperties();
    if (config.getMassOverrideKg().isPresent()) {
      massProps = massProps.withTotalMassKg(config.getMassOverrideKg().getAsDouble());
    }
    return new MeshMetadata(
        raw.sourcePath(), raw.format(), raw.triangleCount(), raw.fileSizeBytes(),
        raw.localBounds(), massProps, raw.collisionSubBounds());
  }

  private static MeshMetadata parseStl(Path path, long fileSize, CadAssetConfig config) throws IOException {
    return isBinaryStl(path, fileSize)
        ? parseBinaryStl(path, fileSize, config)
        : parseAsciiStl(path, fileSize, config);
  }

  private static boolean isBinaryStl(Path path, long fileSize) throws IOException {
    if (fileSize < 84) return false;
    byte[] header = new byte[84];
    try (var in = Files.newInputStream(path)) {
      if (in.readNBytes(header, 0, 84) < 84) return false;
    }
    int triCount = ByteBuffer.wrap(header, 80, 4).order(ByteOrder.LITTLE_ENDIAN).getInt();
    return 84L + (Integer.toUnsignedLong(triCount) * 50L) == fileSize;
  }

  private static MeshMetadata parseBinaryStl(Path path, long fileSize, CadAssetConfig config) throws IOException {
    double scale = config.getUnitScaleToMeters();
    MeshAccumulator acc = new MeshAccumulator();
    try (DataInputStream in = new DataInputStream(new BufferedInputStream(Files.newInputStream(path), 16_384))) {
      byte[] header = new byte[84];
      in.readFully(header);
      int triCount = ByteBuffer.wrap(header, 80, 4).order(ByteOrder.LITTLE_ENDIAN).getInt();
      if (triCount > config.getMaxTriangleBudget()) {
        throw new CadMeshValidationException("Binary STL triangle count exceeds budget.");
      }
      byte[] record = new byte[50];
      ByteBuffer buf = ByteBuffer.wrap(record).order(ByteOrder.LITTLE_ENDIAN);
      for (int i = 0; i < triCount; i++) {
        in.readFully(record);
        buf.rewind();
        buf.position(12);
        acc.addTriangle(
            buf.getFloat() * scale, buf.getFloat() * scale, buf.getFloat() * scale,
            buf.getFloat() * scale, buf.getFloat() * scale, buf.getFloat() * scale,
            buf.getFloat() * scale, buf.getFloat() * scale, buf.getFloat() * scale);
      }
    }
    return acc.toMetadata(path, MeshFormat.STL, fileSize, config.getDensityKgPerM3());
  }

  private static MeshMetadata parseAsciiStl(Path path, long fileSize, CadAssetConfig config) throws IOException {
    double scale = config.getUnitScaleToMeters();
    MeshAccumulator acc = new MeshAccumulator();
    double[] v = new double[9];
    int vIdx = 0;
    try (BufferedReader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
      String line;
      while ((line = reader.readLine()) != null) {
        String trimmed = line.trim();
        if (trimmed.startsWith("vertex ")) {
          String[] parts = trimmed.split("\\s+");
          if (parts.length >= 4) {
            v[vIdx++] = Double.parseDouble(parts[1]) * scale;
            v[vIdx++] = Double.parseDouble(parts[2]) * scale;
            v[vIdx++] = Double.parseDouble(parts[3]) * scale;
            if (vIdx == 9) {
              acc.addTriangle(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
              vIdx = 0;
            }
          }
        }
      }
    }
    return acc.toMetadata(path, MeshFormat.STL, fileSize, config.getDensityKgPerM3());
  }

  private static MeshMetadata parseObj(Path path, long fileSize, CadAssetConfig config) throws IOException {
    double scale = config.getUnitScaleToMeters();
    List<double[]> vertices = new ArrayList<>();
    MeshAccumulator acc = new MeshAccumulator();
    try (BufferedReader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
      String line;
      while ((line = reader.readLine()) != null) {
        String trimmed = line.trim();
        if (trimmed.startsWith("v ")) {
          String[] p = trimmed.split("\\s+");
          if (p.length >= 4) {
            vertices.add(new double[] {
              Double.parseDouble(p[1]) * scale,
              Double.parseDouble(p[2]) * scale,
              Double.parseDouble(p[3]) * scale
            });
          }
        } else if (trimmed.startsWith("f ")) {
          String[] p = trimmed.split("\\s+");
          if (p.length >= 4) {
            int[] idx = new int[p.length - 1];
            for (int i = 1; i < p.length; i++) {
              int rawIdx = Integer.parseInt(p[i].split("/")[0]);
              idx[i - 1] = rawIdx > 0 ? rawIdx - 1 : vertices.size() + rawIdx;
            }
            for (int i = 1; i < idx.length - 1; i++) {
              double[] a = vertices.get(idx[0]), b = vertices.get(idx[i]), c = vertices.get(idx[i + 1]);
              acc.addTriangle(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
            }
          }
        }
      }
    }
    return acc.toMetadata(path, MeshFormat.OBJ, fileSize, config.getDensityKgPerM3());
  }

  private static MeshMetadata parseGlb(Path path, long fileSize, CadAssetConfig config) throws IOException {
    if (fileSize < 20) throw new CadMeshValidationException("GLB file too small: " + path);
    try (DataInputStream in = new DataInputStream(new BufferedInputStream(Files.newInputStream(path), 16_384))) {
      byte[] headerBytes = new byte[20];
      in.readFully(headerBytes);
      ByteBuffer h = ByteBuffer.wrap(headerBytes).order(ByteOrder.LITTLE_ENDIAN);
      int magic = h.getInt(), version = h.getInt();
      h.getInt();
      int chunkLength = h.getInt(), chunkType = h.getInt();
      if (magic != GLB_MAGIC || version != 2 || chunkType != GLB_JSON_CHUNK) {
        throw new CadMeshValidationException("Invalid GLB 2.0 header in file: " + path);
      }
      byte[] jsonBytes = new byte[chunkLength];
      in.readFully(jsonBytes);
      return parseGltfJson(path, new String(jsonBytes, StandardCharsets.UTF_8), fileSize, MeshFormat.GLB, config);
    }
  }

  @SuppressWarnings("unchecked")
  private static MeshMetadata parseGltfJson(
      Path path, String jsonText, long fileSize, MeshFormat format, CadAssetConfig config) {
    Map<String, Object> root = (Map<String, Object>) MinimalJson.parse(jsonText);
    List<Object> accessors = (List<Object>) root.get("accessors");
    List<Object> meshes = (List<Object>) root.get("meshes");
    double scale = config.getUnitScaleToMeters();
    double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
    int totalTriangles = 0;
    List<BoundingBox3d> subBoxes = new ArrayList<>();
    boolean subBoxOverflow = false;

    if (meshes != null && accessors != null) {
      for (Object meshObj : meshes) {
        List<Object> primitives = (List<Object>) ((Map<String, Object>) meshObj).get("primitives");
        if (primitives == null) continue;
        for (Object primObj : primitives) {
          Map<String, Object> prim = (Map<String, Object>) primObj;
          if (prim.containsKey("indices")) {
            Map<String, Object> acc = (Map<String, Object>) accessors.get(((Number) prim.get("indices")).intValue());
            totalTriangles += ((Number) acc.get("count")).intValue() / 3;
          }
          Map<String, Object> attrs = (Map<String, Object>) prim.get("attributes");
          if (attrs != null && attrs.containsKey("POSITION")) {
            Map<String, Object> posAcc = (Map<String, Object>) accessors.get(((Number) attrs.get("POSITION")).intValue());
            if (!prim.containsKey("indices")) totalTriangles += ((Number) posAcc.get("count")).intValue() / 3;
            List<Object> mn = (List<Object>) posAcc.get("min");
            List<Object> mx = (List<Object>) posAcc.get("max");
            if (mn != null && mx != null && mn.size() >= 3 && mx.size() >= 3) {
              double pMinX = ((Number) mn.get(0)).doubleValue() * scale;
              double pMinY = ((Number) mn.get(1)).doubleValue() * scale;
              double pMinZ = ((Number) mn.get(2)).doubleValue() * scale;
              double pMaxX = ((Number) mx.get(0)).doubleValue() * scale;
              double pMaxY = ((Number) mx.get(1)).doubleValue() * scale;
              double pMaxZ = ((Number) mx.get(2)).doubleValue() * scale;
              minX = Math.min(minX, pMinX);
              minY = Math.min(minY, pMinY);
              minZ = Math.min(minZ, pMinZ);
              maxX = Math.max(maxX, pMaxX);
              maxY = Math.max(maxY, pMaxY);
              maxZ = Math.max(maxZ, pMaxZ);
              if (!subBoxOverflow) {
                if (subBoxes.size() < MAX_SUB_OBBS) {
                  subBoxes.add(new BoundingBox3d(pMinX, pMinY, pMinZ, pMaxX, pMaxY, pMaxZ));
                } else {
                  subBoxOverflow = true;
                  subBoxes.clear();
                }
              }
            }
          }
        }
      }
    }
    if (!Double.isFinite(minX)) {
      throw new CadMeshValidationException("glTF/GLB mesh has no POSITION accessor min/max bounds.");
    }
    BoundingBox3d bounds = new BoundingBox3d(minX, minY, minZ, maxX, maxY, maxZ);
    List<BoundingBox3d> effectiveSubBounds = subBoxes.isEmpty() ? List.of(bounds) : subBoxes;
    return new MeshMetadata(
        path, format, totalTriangles, fileSize, bounds,
        CadMassProperties.fromBoundingBox(bounds, config.getDensityKgPerM3()),
        effectiveSubBounds);
  }

  private static final class MeshAccumulator {
    private int triCount = 0;
    private double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
    private double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
    private double signedVol6 = 0.0, cxSum = 0.0, cySum = 0.0, czSum = 0.0;
    private double xxSum = 0.0, yySum = 0.0, zzSum = 0.0;

    void addTriangle(
        double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2) {
      triCount++;
      updateBounds(x0, y0, z0); updateBounds(x1, y1, z1); updateBounds(x2, y2, z2);
      double v6 = x0 * (y1 * z2 - y2 * z1) + y0 * (z1 * x2 - z2 * x1) + z0 * (x1 * y2 - x2 * y1);
      signedVol6 += v6;
      cxSum += v6 * (x0 + x1 + x2); cySum += v6 * (y0 + y1 + y2); czSum += v6 * (z0 + z1 + z2);
      xxSum += v6 * (x0 * x0 + x1 * x1 + x2 * x2 + x0 * x1 + x1 * x2 + x2 * x0);
      yySum += v6 * (y0 * y0 + y1 * y1 + y2 * y2 + y0 * y1 + y1 * y2 + y2 * y0);
      zzSum += v6 * (z0 * z0 + z1 * z1 + z2 * z2 + z0 * z1 + z1 * z2 + z2 * z0);
    }

    private void updateBounds(double x, double y, double z) {
      if (x < minX) minX = x; if (y < minY) minY = y; if (z < minZ) minZ = z;
      if (x > maxX) maxX = x; if (y > maxY) maxY = y; if (z > maxZ) maxZ = z;
    }

    MeshMetadata toMetadata(Path path, MeshFormat format, long fileSize, double density) {
      if (triCount == 0) throw new CadMeshValidationException("Mesh contains zero triangles: " + path);
      BoundingBox3d bounds = new BoundingBox3d(minX, minY, minZ, maxX, maxY, maxZ);
      double volume = Math.abs(signedVol6) / 6.0;
      CadMassProperties massProps;
      if (volume > 1e-12) {
        double sign = signedVol6 >= 0.0 ? 1.0 : -1.0;
        double cx = (sign * cxSum) / (4.0 * Math.abs(signedVol6));
        double cy = (sign * cySum) / (4.0 * Math.abs(signedVol6));
        double cz = (sign * czSum) / (4.0 * Math.abs(signedVol6));
        double mass = volume * density;
        double cXX = Math.max(0.0, (sign * xxSum) / 60.0 - volume * cx * cx);
        double cYY = Math.max(0.0, (sign * yySum) / 60.0 - volume * cy * cy);
        double cZZ = Math.max(0.0, (sign * zzSum) / 60.0 - volume * cz * cz);
        massProps = new CadMassProperties(
            volume, mass, new Translation3d(cx, cy, cz),
            density * (cYY + cZZ), density * (cXX + cZZ), density * (cXX + cYY));
      } else {
        massProps = CadMassProperties.fromBoundingBox(bounds, density);
      }
      return new MeshMetadata(path, format, triCount, fileSize, bounds, massProps);
    }
  }
}
