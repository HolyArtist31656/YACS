// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

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
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.CadMassProperties;
import yacsim.core.util.MinimalJson;

/**
 * Zero-dependency CAD mesh parser supporting header-only glTF 2.0 ({@code .glb}/{@code .gltf})
 * with full scene-graph node-transform traversal, plus streaming Wavefront {@code .obj} and
 * ASCII/binary {@code .stl} files.
 */
public final class MeshParser {
  private static final int GLB_MAGIC = 0x46546C67;
  private static final int GLB_JSON_CHUNK = 0x4E4F534A;
  /** Maximum per-primitive sub-OBBs retained per glTF/GLB asset to keep 50 Hz SystemCore checks fast. */
  public static final int MAX_SUB_OBBS = 64;

  private record CacheKey(
      Path absolutePath,
      long lastModifiedMillis,
      long fileSizeBytes,
      double unitScale,
      double densityKgPerM3,
      OptionalDouble massOverrideKg,
      Optional<CadMassProperties> massPropertiesOverride,
      int maxTriangleBudget) {}

  private static final Map<CacheKey, MeshMetadata> METADATA_CACHE = new ConcurrentHashMap<>();

  /** Clears the in-memory parsed {@link MeshMetadata} cache. */
  public static void clearCache() {
    METADATA_CACHE.clear();
  }

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
    long lastMod = Files.getLastModifiedTime(path).toMillis();
    CacheKey cacheKey =
        new CacheKey(
            path.toAbsolutePath().normalize(),
            lastMod,
            fileSize,
            config.getUnitScaleToMeters(),
            config.getDensityKgPerM3(),
            config.getMassOverrideKg(),
            config.getMassPropertiesOverride(),
            config.getMaxTriangleBudget());
    MeshMetadata cached = METADATA_CACHE.get(cacheKey);
    if (cached != null) {
      return cached;
    }

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

    CadMassProperties massProps =
        config.getMassPropertiesOverride().orElseGet(raw::massProperties);
    if (config.getMassOverrideKg().isPresent()) {
      massProps = massProps.withTotalMassKg(config.getMassOverrideKg().getAsDouble());
    }
    MeshMetadata result =
        new MeshMetadata(
            raw.sourcePath(), raw.format(), raw.triangleCount(), raw.fileSizeBytes(),
            raw.localBounds(), massProps, raw.collisionSubBounds());
    METADATA_CACHE.put(cacheKey, result);
    return result;
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

  private static final class GltfBoundsAccumulator {
    double minX = Double.POSITIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double minZ = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    double maxZ = Double.NEGATIVE_INFINITY;
    int totalTriangles = 0;
    final List<BoundingBox3d> subBoxes = new ArrayList<>();
    boolean subBoxOverflow = false;
  }

  @SuppressWarnings("unchecked")
  private static MeshMetadata parseGltfJson(
      Path path, String jsonText, long fileSize, MeshFormat format, CadAssetConfig config) {
    Map<String, Object> root = (Map<String, Object>) MinimalJson.parse(jsonText);
    List<Object> accessors = (List<Object>) root.get("accessors");
    List<Object> meshes = (List<Object>) root.get("meshes");
    List<Object> nodes = (List<Object>) root.get("nodes");
    double scale = config.getUnitScaleToMeters();
    GltfBoundsAccumulator acc = new GltfBoundsAccumulator();

    if (meshes != null && accessors != null) {
      if (nodes != null && !nodes.isEmpty()) {
        List<Integer> rootNodes = extractGltfSceneRoots(root, nodes);
        double[] identity = identityMatrix4();
        Set<Integer> visiting = new HashSet<>();
        for (int rootIdx : rootNodes) {
          traverseGltfNode(rootIdx, identity, nodes, meshes, accessors, scale, acc, visiting);
        }
      }
      // Fallback for minimal synthetic glTFs without scenes/nodes or unreferenced meshes
      if (!Double.isFinite(acc.minX)) {
        double[] identity = identityMatrix4();
        for (int mIdx = 0; mIdx < meshes.size(); mIdx++) {
          accumulateMeshWithMatrix(mIdx, identity, meshes, accessors, scale, acc);
        }
      }
    }
    if (!Double.isFinite(acc.minX)) {
      throw new CadMeshValidationException("glTF/GLB mesh has no POSITION accessor min/max bounds.");
    }
    BoundingBox3d bounds = new BoundingBox3d(acc.minX, acc.minY, acc.minZ, acc.maxX, acc.maxY, acc.maxZ);
    List<BoundingBox3d> effectiveSubBounds = acc.subBoxes.isEmpty() ? List.of(bounds) : acc.subBoxes;
    return new MeshMetadata(
        path, format, acc.totalTriangles, fileSize, bounds,
        CadMassProperties.fromBoundingBox(bounds, config.getDensityKgPerM3()),
        effectiveSubBounds);
  }

  @SuppressWarnings("unchecked")
  private static List<Integer> extractGltfSceneRoots(Map<String, Object> root, List<Object> nodes) {
    List<Object> scenes = (List<Object>) root.get("scenes");
    if (scenes != null && !scenes.isEmpty()) {
      int sceneIdx = root.get("scene") instanceof Number n ? n.intValue() : 0;
      if (sceneIdx < 0 || sceneIdx >= scenes.size()) {
        sceneIdx = 0;
      }
      Map<String, Object> sceneObj = (Map<String, Object>) scenes.get(sceneIdx);
      List<Object> sceneNodes = (List<Object>) sceneObj.get("nodes");
      if (sceneNodes != null && !sceneNodes.isEmpty()) {
        List<Integer> roots = new ArrayList<>(sceneNodes.size());
        for (Object o : sceneNodes) {
          if (o instanceof Number n) {
            roots.add(n.intValue());
          }
        }
        return roots;
      }
    }
    Set<Integer> children = new HashSet<>();
    for (Object nodeObj : nodes) {
      Map<String, Object> node = (Map<String, Object>) nodeObj;
      List<Object> childList = (List<Object>) node.get("children");
      if (childList != null) {
        for (Object c : childList) {
          if (c instanceof Number n) {
            children.add(n.intValue());
          }
        }
      }
    }
    List<Integer> roots = new ArrayList<>();
    for (int i = 0; i < nodes.size(); i++) {
      if (!children.contains(i)) {
        roots.add(i);
      }
    }
    return roots;
  }

  @SuppressWarnings("unchecked")
  private static void traverseGltfNode(
      int nodeIdx,
      double[] parentMatrix,
      List<Object> nodes,
      List<Object> meshes,
      List<Object> accessors,
      double unitScale,
      GltfBoundsAccumulator acc,
      Set<Integer> visiting) {
    if (nodeIdx < 0 || nodeIdx >= nodes.size() || !visiting.add(nodeIdx)) {
      return;
    }
    try {
      Map<String, Object> node = (Map<String, Object>) nodes.get(nodeIdx);
      double[] local = nodeLocalMatrix4(node);
      double[] world = multiplyMatrix4(parentMatrix, local);

      if (node.get("mesh") instanceof Number meshNum) {
        int meshIdx = meshNum.intValue();
        if (meshIdx >= 0 && meshIdx < meshes.size()) {
          accumulateMeshWithMatrix(meshIdx, world, meshes, accessors, unitScale, acc);
        }
      }
      List<Object> childList = (List<Object>) node.get("children");
      if (childList != null) {
        for (Object c : childList) {
          if (c instanceof Number childNum) {
            traverseGltfNode(
                childNum.intValue(), world, nodes, meshes, accessors, unitScale, acc, visiting);
          }
        }
      }
    } finally {
      visiting.remove(nodeIdx);
    }
  }

  @SuppressWarnings("unchecked")
  private static void accumulateMeshWithMatrix(
      int meshIdx,
      double[] worldMatrix,
      List<Object> meshes,
      List<Object> accessors,
      double unitScale,
      GltfBoundsAccumulator acc) {
    Map<String, Object> meshObj = (Map<String, Object>) meshes.get(meshIdx);
    List<Object> primitives = (List<Object>) meshObj.get("primitives");
    if (primitives == null) {
      return;
    }
    for (Object primObj : primitives) {
      Map<String, Object> prim = (Map<String, Object>) primObj;
      if (prim.containsKey("indices")) {
        int idxAcc = ((Number) prim.get("indices")).intValue();
        if (idxAcc >= 0 && idxAcc < accessors.size()) {
          Map<String, Object> accMap = (Map<String, Object>) accessors.get(idxAcc);
          acc.totalTriangles += ((Number) accMap.get("count")).intValue() / 3;
        }
      }
      Map<String, Object> attrs = (Map<String, Object>) prim.get("attributes");
      if (attrs != null && attrs.containsKey("POSITION")) {
        int posIdx = ((Number) attrs.get("POSITION")).intValue();
        if (posIdx < 0 || posIdx >= accessors.size()) {
          continue;
        }
        Map<String, Object> posAcc = (Map<String, Object>) accessors.get(posIdx);
        if (!prim.containsKey("indices")) {
          acc.totalTriangles += ((Number) posAcc.get("count")).intValue() / 3;
        }
        List<Object> mn = (List<Object>) posAcc.get("min");
        List<Object> mx = (List<Object>) posAcc.get("max");
        if (mn != null && mx != null && mn.size() >= 3 && mx.size() >= 3) {
          double lx0 = ((Number) mn.get(0)).doubleValue();
          double ly0 = ((Number) mn.get(1)).doubleValue();
          double lz0 = ((Number) mn.get(2)).doubleValue();
          double lx1 = ((Number) mx.get(0)).doubleValue();
          double ly1 = ((Number) mx.get(1)).doubleValue();
          double lz1 = ((Number) mx.get(2)).doubleValue();

          double pMinX = Double.POSITIVE_INFINITY, pMinY = Double.POSITIVE_INFINITY, pMinZ = Double.POSITIVE_INFINITY;
          double pMaxX = Double.NEGATIVE_INFINITY, pMaxY = Double.NEGATIVE_INFINITY, pMaxZ = Double.NEGATIVE_INFINITY;
          for (int ix = 0; ix < 2; ix++) {
            double x = (ix == 0) ? lx0 : lx1;
            for (int iy = 0; iy < 2; iy++) {
              double y = (iy == 0) ? ly0 : ly1;
              for (int iz = 0; iz < 2; iz++) {
                double z = (iz == 0) ? lz0 : lz1;
                double wx = (worldMatrix[0] * x + worldMatrix[4] * y + worldMatrix[8] * z + worldMatrix[12]) * unitScale;
                double wy = (worldMatrix[1] * x + worldMatrix[5] * y + worldMatrix[9] * z + worldMatrix[13]) * unitScale;
                double wz = (worldMatrix[2] * x + worldMatrix[6] * y + worldMatrix[10] * z + worldMatrix[14]) * unitScale;
                if (wx < pMinX) pMinX = wx;
                if (wy < pMinY) pMinY = wy;
                if (wz < pMinZ) pMinZ = wz;
                if (wx > pMaxX) pMaxX = wx;
                if (wy > pMaxY) pMaxY = wy;
                if (wz > pMaxZ) pMaxZ = wz;
              }
            }
          }
          acc.minX = Math.min(acc.minX, pMinX);
          acc.minY = Math.min(acc.minY, pMinY);
          acc.minZ = Math.min(acc.minZ, pMinZ);
          acc.maxX = Math.max(acc.maxX, pMaxX);
          acc.maxY = Math.max(acc.maxY, pMaxY);
          acc.maxZ = Math.max(acc.maxZ, pMaxZ);
          if (!acc.subBoxOverflow) {
            if (acc.subBoxes.size() < MAX_SUB_OBBS) {
              acc.subBoxes.add(new BoundingBox3d(pMinX, pMinY, pMinZ, pMaxX, pMaxY, pMaxZ));
            } else {
              acc.subBoxOverflow = true;
              acc.subBoxes.clear();
            }
          }
        }
      }
    }
  }

  private static double[] identityMatrix4() {
    return new double[] {
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
    };
  }

  @SuppressWarnings("unchecked")
  private static double[] nodeLocalMatrix4(Map<String, Object> node) {
    List<Object> mat = (List<Object>) node.get("matrix");
    if (mat != null && mat.size() >= 16) {
      double[] m = new double[16];
      for (int i = 0; i < 16; i++) {
        m[i] = ((Number) mat.get(i)).doubleValue();
      }
      return m;
    }
    double tx = 0.0, ty = 0.0, tz = 0.0;
    List<Object> t = (List<Object>) node.get("translation");
    if (t != null && t.size() >= 3) {
      tx = ((Number) t.get(0)).doubleValue();
      ty = ((Number) t.get(1)).doubleValue();
      tz = ((Number) t.get(2)).doubleValue();
    }
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    List<Object> r = (List<Object>) node.get("rotation");
    if (r != null && r.size() >= 4) {
      qx = ((Number) r.get(0)).doubleValue();
      qy = ((Number) r.get(1)).doubleValue();
      qz = ((Number) r.get(2)).doubleValue();
      qw = ((Number) r.get(3)).doubleValue();
    }
    double sx = 1.0, sy = 1.0, sz = 1.0;
    List<Object> s = (List<Object>) node.get("scale");
    if (s != null && s.size() >= 3) {
      sx = ((Number) s.get(0)).doubleValue();
      sy = ((Number) s.get(1)).doubleValue();
      sz = ((Number) s.get(2)).doubleValue();
    }
    double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    double r10 = 2.0 * (qx * qy + qw * qz);
    double r20 = 2.0 * (qx * qz - qw * qy);
    double r01 = 2.0 * (qx * qy - qw * qz);
    double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    double r21 = 2.0 * (qy * qz + qw * qx);
    double r02 = 2.0 * (qx * qz + qw * qy);
    double r12 = 2.0 * (qy * qz - qw * qx);
    double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
    return new double[] {
      r00 * sx, r10 * sx, r20 * sx, 0.0,
      r01 * sy, r11 * sy, r21 * sy, 0.0,
      r02 * sz, r12 * sz, r22 * sz, 0.0,
      tx, ty, tz, 1.0
    };
  }

  private static double[] multiplyMatrix4(double[] a, double[] b) {
    double[] out = new double[16];
    for (int col = 0; col < 4; col++) {
      int c4 = col * 4;
      double b0 = b[c4], b1 = b[c4 + 1], b2 = b[c4 + 2], b3 = b[c4 + 3];
      for (int row = 0; row < 4; row++) {
        out[c4 + row] =
            a[row] * b0 + a[4 + row] * b1 + a[8 + row] * b2 + a[12 + row] * b3;
      }
    }
    return out;
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
