// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.cad;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.util.MinimalJson;

/**
 * Zero-dependency pure-Java GLB 2.0 merger for combining multi-{@code <visual>} URDF links (such as
 * Onshape's native URDF export, where a single {@code <link>} references multiple part {@code .glb}
 * files with individual {@code <origin xyz="..." rpy="..."/>} transforms) into one self-contained
 * binary {@code .glb} file for AdvantageScope.
 *
 * <p>Input {@code .glb} files referenced multiple times within the same link (for example repeated
 * bearings or fasteners) are deduplicated at the binary buffer, accessor, material, and mesh level
 * while cloning lightweight glTF scene nodes so the resulting file stays compact and 100% glTF 2.0
 * tree-compliant.
 */
public final class GlbMerger {
  private static final int GLB_MAGIC = 0x46546C67; // "glTF"
  private static final int GLB_VERSION = 2;
  private static final int CHUNK_JSON = 0x4E4F534A; // "JSON"
  private static final int CHUNK_BIN = 0x004E4942; // "BIN\0"

  private GlbMerger() {}

  /**
   * One visual part instance to merge into a single {@code .glb} link asset.
   *
   * @param glbPath Path to the source binary {@code .glb} file.
   * @param translationMeters Local {@code <origin xyz>} translation in meters.
   * @param rotation Local {@code <origin rpy>} rotation.
   * @param scale Uniform scale factor (typically {@code 1.0}).
   */
  public record VisualInstance(
      Path glbPath,
      Translation3d translationMeters,
      Rotation3d rotation,
      double scale,
      boolean noSimplify) {
    /** Backwards-compatible 4-argument constructor defaulting {@code noSimplify} to {@code false}. */
    public VisualInstance(
        Path glbPath, Translation3d translationMeters, Rotation3d rotation, double scale) {
      this(glbPath, translationMeters, rotation, scale, false);
    }

    /** Compact constructor validating fields. */
    public VisualInstance {
      Objects.requireNonNull(glbPath, "glbPath");
      translationMeters = translationMeters == null ? new Translation3d() : translationMeters;
      rotation = rotation == null ? new Rotation3d() : rotation;
      if (scale <= 0.0) {
        scale = 1.0;
      }
    }
  }

  private record ParsedGlb(Map<String, Object> json, byte[] binChunk) {}

  private record ImportedAsset(List<Object> rawNodes, List<Integer> sceneRoots, int meshOffset) {}

  /**
   * Merges one or more positioned {@link VisualInstance} {@code .glb} files into a single binary
   * GLB 2.0 file at {@code outputGlb}.
   *
   * @param instances Positioned {@code .glb} visual instances to combine.
   * @param outputGlb Destination {@code .glb} file path.
   * @throws IOException If reading any input {@code .glb} or writing {@code outputGlb} fails.
   */
  public static void merge(List<VisualInstance> instances, Path outputGlb) throws IOException {
    merge(instances, outputGlb, false);
  }

  /**
   * Merges one or more positioned {@link VisualInstance} {@code .glb} files into a single binary
   * GLB 2.0 file at {@code outputGlb}, optionally wrapping the scene in an {@code Rx(-90 deg)}
   * root node to convert URDF Z-up coordinates into standard glTF 2.0 Y-up coordinates.
   *
   * @param instances Positioned {@code .glb} visual instances to combine.
   * @param outputGlb Destination {@code .glb} file path.
   * @param convertZUpToYUp {@code true} to apply a single outer {@code Rx(-90 deg)} glTF root node
   *     after composing all URDF Z-up visual transforms.
   * @throws IOException If reading any input {@code .glb} or writing {@code outputGlb} fails.
   */
  /**
   * Computes the minimum Z coordinate (in URDF Z-up meters) across all positioned {@link
   * VisualInstance} {@code .glb} meshes by transforming each part's local 8-corner AABB through its
   * scale, rotation, and translation. Returns {@code 0.0} if no POSITION bounds are found.
   *
   * @param instances Positioned {@code .glb} visual instances.
   * @return Minimum transformed Z coordinate in meters, or {@code 0.0} if unavailable.
   * @throws IOException If reading any input {@code .glb} fails.
   */
  @SuppressWarnings("unchecked")
  public static double computeTransformedMinZ(List<VisualInstance> instances) throws IOException {
    Objects.requireNonNull(instances, "instances");
    Map<Path, double[]> localBoundsCache = new LinkedHashMap<>();
    double globalMinZ = Double.POSITIVE_INFINITY;

    for (VisualInstance inst : instances) {
      Path canonical = inst.glbPath().toAbsolutePath().normalize();
      double[] b = localBoundsCache.get(canonical);
      if (b == null) {
        ParsedGlb parsed = readGlb(canonical);
        b = extractLocalPosBounds(parsed.json());
        localBoundsCache.put(canonical, b);
      }
      if (b == null) {
        continue;
      }
      double sc = inst.scale();
      Translation3d t = inst.translationMeters();
      Rotation3d rot = inst.rotation();
      double[] xs = {b[0] * sc, b[3] * sc};
      double[] ys = {b[1] * sc, b[4] * sc};
      double[] zs = {b[2] * sc, b[5] * sc};
      for (double x : xs) {
        for (double y : ys) {
          for (double z : zs) {
            Translation3d corner = new Translation3d(x, y, z).rotateBy(rot).plus(t);
            if (corner.getZ() < globalMinZ) {
              globalMinZ = corner.getZ();
            }
          }
        }
      }
    }
    return Double.isFinite(globalMinZ) ? globalMinZ : 0.0;
  }

  @SuppressWarnings("unchecked")
  private static double[] extractLocalPosBounds(Map<String, Object> gltf) {
    List<Object> accessors = asList(gltf.get("accessors"));
    List<Object> meshes = asList(gltf.get("meshes"));
    double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
    for (Object mObj : meshes) {
      Map<String, Object> mesh = (Map<String, Object>) mObj;
      for (Object pObj : asList(mesh.get("primitives"))) {
        Map<String, Object> prim = (Map<String, Object>) pObj;
        if (prim.get("attributes") instanceof Map<?, ?> attrs
            && attrs.get("POSITION") instanceof Number posIdx) {
          int idx = posIdx.intValue();
          if (idx >= 0 && idx < accessors.size()) {
            Map<String, Object> acc = (Map<String, Object>) accessors.get(idx);
            List<Object> mn = asList(acc.get("min"));
            List<Object> mx = asList(acc.get("max"));
            if (mn.size() >= 3 && mx.size() >= 3) {
              minX = Math.min(minX, ((Number) mn.get(0)).doubleValue());
              minY = Math.min(minY, ((Number) mn.get(1)).doubleValue());
              minZ = Math.min(minZ, ((Number) mn.get(2)).doubleValue());
              maxX = Math.max(maxX, ((Number) mx.get(0)).doubleValue());
              maxY = Math.max(maxY, ((Number) mx.get(1)).doubleValue());
              maxZ = Math.max(maxZ, ((Number) mx.get(2)).doubleValue());
            }
          }
        }
      }
    }
    if (!Double.isFinite(minX)) {
      return null;
    }
    return new double[] {minX, minY, minZ, maxX, maxY, maxZ};
  }

  public static void merge(
      List<VisualInstance> instances, Path outputGlb, boolean convertZUpToYUp) throws IOException {
    merge(instances, outputGlb, convertZUpToYUp, 0.0);
  }

  /**
   * Merges one or more positioned {@link VisualInstance} {@code .glb} files into a single binary
   * GLB 2.0 file at {@code outputGlb}, optionally applying a vertical {@code zShiftMeters} offset
   * in URDF Z-up before wrapping in an {@code Rx(-90 deg)} root node.
   *
   * @param instances Positioned {@code .glb} visual instances to combine.
   * @param outputGlb Destination {@code .glb} file path.
   * @param convertZUpToYUp {@code true} to apply a single outer {@code Rx(-90 deg)} glTF root node.
   * @param zShiftMeters Vertical Z shift (meters) applied in URDF Z-up space for unified grounding.
   * @throws IOException If reading any input {@code .glb} or writing {@code outputGlb} fails.
   */
  @SuppressWarnings("unchecked")
  public static void merge(
      List<VisualInstance> instances,
      Path outputGlb,
      boolean convertZUpToYUp,
      double zShiftMeters)
      throws IOException {
    Objects.requireNonNull(instances, "instances");
    Objects.requireNonNull(outputGlb, "outputGlb");
    if (instances.isEmpty()) {
      throw new IllegalArgumentException("At least one VisualInstance is required to merge GLBs.");
    }

    ByteArrayOutputStream combinedBin = new ByteArrayOutputStream();
    List<Object> outBufferViews = new ArrayList<>();
    List<Object> outAccessors = new ArrayList<>();
    List<Object> outImages = new ArrayList<>();
    List<Object> outSamplers = new ArrayList<>();
    List<Object> outTextures = new ArrayList<>();
    List<Object> outMaterials = new ArrayList<>();
    List<Object> outMeshes = new ArrayList<>();
    List<Object> outNodes = new ArrayList<>();
    List<Object> outSceneRootNodes = new ArrayList<>();

    Map<Path, ImportedAsset> importedCache = new LinkedHashMap<>();

    for (VisualInstance inst : instances) {
      Path canonical = inst.glbPath().toAbsolutePath().normalize();
      ImportedAsset asset = importedCache.get(canonical);
      if (asset == null) {
        ParsedGlb parsed = readGlb(canonical);
        Map<String, Object> src = parsed.json();

        int binOffset = pad4(combinedBin, (byte) 0);
        if (parsed.binChunk().length > 0) {
          combinedBin.write(parsed.binChunk());
        }

        int bvOffset = outBufferViews.size();
        int accOffset = outAccessors.size();
        int imgOffset = outImages.size();
        int samplerOffset = outSamplers.size();
        int texOffset = outTextures.size();
        int matOffset = outMaterials.size();
        int meshOffset = outMeshes.size();

        List<Object> srcBvs = asList(src.get("bufferViews"));
        for (Object obj : srcBvs) {
          Map<String, Object> bv = deepCopyMap((Map<String, Object>) obj);
          bv.put("buffer", 0);
          long prevByteOffset = asLong(bv.get("byteOffset"), 0L);
          bv.put("byteOffset", prevByteOffset + binOffset);
          outBufferViews.add(bv);
        }

        List<Object> srcAccs = asList(src.get("accessors"));
        for (Object obj : srcAccs) {
          Map<String, Object> acc = deepCopyMap((Map<String, Object>) obj);
          remapIntField(acc, "bufferView", bvOffset);
          if (acc.get("sparse") instanceof Map<?, ?> sparseRaw) {
            Map<String, Object> sparse = (Map<String, Object>) sparseRaw;
            if (sparse.get("indices") instanceof Map<?, ?> idxRaw) {
              remapIntField((Map<String, Object>) idxRaw, "bufferView", bvOffset);
            }
            if (sparse.get("values") instanceof Map<?, ?> valRaw) {
              remapIntField((Map<String, Object>) valRaw, "bufferView", bvOffset);
            }
          }
          outAccessors.add(acc);
        }

        List<Object> srcImages = asList(src.get("images"));
        for (Object obj : srcImages) {
          Map<String, Object> img = deepCopyMap((Map<String, Object>) obj);
          remapIntField(img, "bufferView", bvOffset);
          outImages.add(img);
        }

        List<Object> srcSamplers = asList(src.get("samplers"));
        for (Object obj : srcSamplers) {
          outSamplers.add(deepCopyMap((Map<String, Object>) obj));
        }

        List<Object> srcTextures = asList(src.get("textures"));
        for (Object obj : srcTextures) {
          Map<String, Object> tex = deepCopyMap((Map<String, Object>) obj);
          remapIntField(tex, "source", imgOffset);
          remapIntField(tex, "sampler", samplerOffset);
          outTextures.add(tex);
        }

        List<Object> srcMaterials = asList(src.get("materials"));
        for (Object obj : srcMaterials) {
          Map<String, Object> mat = deepCopyMap((Map<String, Object>) obj);
          remapMaterialTextures(mat, texOffset);
          outMaterials.add(mat);
        }

        List<Object> srcMeshes = asList(src.get("meshes"));
        for (Object obj : srcMeshes) {
          Map<String, Object> mesh = deepCopyMap((Map<String, Object>) obj);
          List<Object> primitives = asList(mesh.get("primitives"));
          for (Object pObj : primitives) {
            Map<String, Object> prim = (Map<String, Object>) pObj;
            if (prim.get("attributes") instanceof Map<?, ?> attrsRaw) {
              Map<String, Object> attrs = (Map<String, Object>) attrsRaw;
              for (String key : List.copyOf(attrs.keySet())) {
                remapIntField(attrs, key, accOffset);
              }
            }
            remapIntField(prim, "indices", accOffset);
            remapIntField(prim, "material", matOffset);
            List<Object> targets = asList(prim.get("targets"));
            for (Object tObj : targets) {
              if (tObj instanceof Map<?, ?> targetRaw) {
                Map<String, Object> target = (Map<String, Object>) targetRaw;
                for (String key : List.copyOf(target.keySet())) {
                  remapIntField(target, key, accOffset);
                }
              }
            }
          }
          outMeshes.add(mesh);
        }

        List<Object> rawNodes = asList(src.get("nodes"));
        List<Integer> sceneRoots = extractSceneRoots(src, rawNodes.size());
        asset = new ImportedAsset(rawNodes, sceneRoots, meshOffset);
        importedCache.put(canonical, asset);
      }

      // Clone the lightweight node hierarchy for this visual instance so every node has a single
      // parent in the glTF 2.0 scene graph.
      int nodeOffset = outNodes.size();
      for (Object obj : asset.rawNodes()) {
        Map<String, Object> node = deepCopyMap((Map<String, Object>) obj);
        remapIntField(node, "mesh", asset.meshOffset());
        if (node.get("children") instanceof List<?> childrenRaw) {
          List<Object> remapped = new ArrayList<>(childrenRaw.size());
          for (Object c : childrenRaw) {
            remapped.add(((Number) c).intValue() + nodeOffset);
          }
          node.put("children", remapped);
        }
        // Strip skin/camera references that are not carried across merged visual links
        node.remove("skin");
        node.remove("camera");
        outNodes.add(node);
      }

      List<Object> wrapperChildren = new ArrayList<>();
      for (int rootIdx : asset.sceneRoots()) {
        wrapperChildren.add(rootIdx + nodeOffset);
      }

      Map<String, Object> wrapper = new LinkedHashMap<>();
      if (inst.noSimplify()) {
        String baseName = inst.glbPath().getFileName().toString().replaceFirst("\\.[^.]+$", "");
        wrapper.put("name", "NOSIMPLIFY_" + baseName);
      }
      Translation3d t = inst.translationMeters();
      double tz = t.getZ() + zShiftMeters;
      if (Math.abs(t.getX()) > 1e-12 || Math.abs(t.getY()) > 1e-12 || Math.abs(tz) > 1e-12) {
        wrapper.put("translation", List.of(t.getX(), t.getY(), tz));
      }
      Quaternion q = inst.rotation().getQuaternion();
      if (Math.abs(q.getX()) > 1e-12 || Math.abs(q.getY()) > 1e-12 || Math.abs(q.getZ()) > 1e-12) {
        // glTF 2.0 quaternion order is [x, y, z, w]
        wrapper.put("rotation", List.of(q.getX(), q.getY(), q.getZ(), q.getW()));
      }
      if (Math.abs(inst.scale() - 1.0) > 1e-12) {
        wrapper.put("scale", List.of(inst.scale(), inst.scale(), inst.scale()));
      }
      if (!wrapperChildren.isEmpty()) {
        wrapper.put("children", wrapperChildren);
      }
      int wrapperIdx = outNodes.size();
      outNodes.add(wrapper);
      outSceneRootNodes.add(wrapperIdx);
    }

    pad4(combinedBin, (byte) 0);
    byte[] binBytes = combinedBin.toByteArray();

    Map<String, Object> outRoot = new LinkedHashMap<>();
    Map<String, Object> assetMeta = new LinkedHashMap<>();
    assetMeta.put("version", "2.0");
    assetMeta.put("generator", "YACSim GlbMerger");
    outRoot.put("asset", assetMeta);
    outRoot.put("scene", 0);

    List<Object> finalSceneRoots = outSceneRootNodes;
    if (convertZUpToYUp) {
      double s = Math.sqrt(0.5);
      Map<String, Object> zUpToYUp = new LinkedHashMap<>();
      zUpToYUp.put("name", "yacsim_urdf_zup_to_gltf_yup");
      // glTF 2.0 quaternion [x, y, z, w] for Rx(-90 deg): maps URDF (x, y, z) -> glTF (x, z, -y)
      zUpToYUp.put("rotation", List.of(-s, 0.0, 0.0, s));
      zUpToYUp.put("children", outSceneRootNodes);
      int rootIdx = outNodes.size();
      outNodes.add(zUpToYUp);
      finalSceneRoots = List.of(rootIdx);
    }

    Map<String, Object> scene0 = new LinkedHashMap<>();
    scene0.put("nodes", finalSceneRoots);
    outRoot.put("scenes", List.of(scene0));
    outRoot.put("nodes", outNodes);
    outRoot.put("meshes", outMeshes);
    if (!outMaterials.isEmpty()) {
      outRoot.put("materials", outMaterials);
    }
    if (!outTextures.isEmpty()) {
      outRoot.put("textures", outTextures);
    }
    if (!outImages.isEmpty()) {
      outRoot.put("images", outImages);
    }
    if (!outSamplers.isEmpty()) {
      outRoot.put("samplers", outSamplers);
    }
    outRoot.put("accessors", outAccessors);
    outRoot.put("bufferViews", outBufferViews);
    if (binBytes.length > 0) {
      Map<String, Object> buf0 = new LinkedHashMap<>();
      buf0.put("byteLength", binBytes.length);
      outRoot.put("buffers", List.of(buf0));
    }

    byte[] rawJsonBytes = MinimalJson.toPrettyJson(outRoot).getBytes(StandardCharsets.UTF_8);
    int jsonPad = (4 - (rawJsonBytes.length % 4)) % 4;
    int jsonChunkLen = rawJsonBytes.length + jsonPad;

    int totalLength = 12 + 8 + jsonChunkLen + (binBytes.length > 0 ? (8 + binBytes.length) : 0);
    ByteBuffer outBuf = ByteBuffer.allocate(totalLength).order(ByteOrder.LITTLE_ENDIAN);
    outBuf.putInt(GLB_MAGIC);
    outBuf.putInt(GLB_VERSION);
    outBuf.putInt(totalLength);
    outBuf.putInt(jsonChunkLen);
    outBuf.putInt(CHUNK_JSON);
    outBuf.put(rawJsonBytes);
    for (int i = 0; i < jsonPad; i++) {
      outBuf.put((byte) 0x20);
    }
    if (binBytes.length > 0) {
      outBuf.putInt(binBytes.length);
      outBuf.putInt(CHUNK_BIN);
      outBuf.put(binBytes);
    }

    if (outputGlb.getParent() != null) {
      Files.createDirectories(outputGlb.getParent());
    }
    Files.write(outputGlb, outBuf.array());
  }

  @SuppressWarnings("unchecked")
  private static ParsedGlb readGlb(Path path) throws IOException {
    byte[] all = Files.readAllBytes(path);
    if (all.length < 20) {
      throw new IOException("GLB file too small: " + path);
    }
    ByteBuffer buf = ByteBuffer.wrap(all).order(ByteOrder.LITTLE_ENDIAN);
    int magic = buf.getInt();
    int version = buf.getInt();
    int totalLen = buf.getInt();
    if (magic != GLB_MAGIC || version != GLB_VERSION) {
      throw new IOException("Invalid GLB 2.0 header in file: " + path);
    }
    Map<String, Object> json = null;
    byte[] bin = new byte[0];
    int limit = Math.min(all.length, totalLen);
    while (buf.position() + 8 <= limit) {
      int chunkLen = buf.getInt();
      int chunkType = buf.getInt();
      if (chunkLen < 0 || buf.position() + chunkLen > all.length) {
        throw new IOException("Corrupt GLB chunk length in file: " + path);
      }
      byte[] chunkData = new byte[chunkLen];
      buf.get(chunkData);
      if (chunkType == CHUNK_JSON && json == null) {
        String text = new String(chunkData, StandardCharsets.UTF_8).trim();
        json = (Map<String, Object>) MinimalJson.parse(text);
      } else if (chunkType == CHUNK_BIN && bin.length == 0) {
        bin = chunkData;
      }
    }
    if (json == null) {
      throw new IOException("GLB file missing JSON chunk: " + path);
    }
    return new ParsedGlb(json, bin);
  }

  @SuppressWarnings("unchecked")
  private static List<Integer> extractSceneRoots(Map<String, Object> src, int nodeCount) {
    List<Object> scenes = asList(src.get("scenes"));
    int sceneIdx = src.get("scene") instanceof Number n ? n.intValue() : 0;
    if (!scenes.isEmpty() && sceneIdx >= 0 && sceneIdx < scenes.size()) {
      Map<String, Object> sc = (Map<String, Object>) scenes.get(sceneIdx);
      List<Object> roots = asList(sc.get("nodes"));
      if (!roots.isEmpty()) {
        List<Integer> out = new ArrayList<>(roots.size());
        for (Object r : roots) {
          out.add(((Number) r).intValue());
        }
        return out;
      }
    }
    List<Integer> fallback = new ArrayList<>();
    for (int i = 0; i < nodeCount; i++) {
      fallback.add(i);
    }
    return fallback;
  }

  @SuppressWarnings("unchecked")
  private static void remapMaterialTextures(Map<String, Object> mat, int texOffset) {
    if (texOffset == 0) {
      return;
    }
    for (String key : List.of("normalTexture", "occlusionTexture", "emissiveTexture")) {
      if (mat.get(key) instanceof Map<?, ?> texInfo) {
        remapIntField((Map<String, Object>) texInfo, "index", texOffset);
      }
    }
    if (mat.get("pbrMetallicRoughness") instanceof Map<?, ?> pbrRaw) {
      Map<String, Object> pbr = (Map<String, Object>) pbrRaw;
      for (String key : List.of("baseColorTexture", "metallicRoughnessTexture")) {
        if (pbr.get(key) instanceof Map<?, ?> texInfo) {
          remapIntField((Map<String, Object>) texInfo, "index", texOffset);
        }
      }
    }
  }

  private static void remapIntField(Map<String, Object> map, String key, int offset) {
    if (offset != 0 && map.get(key) instanceof Number n) {
      map.put(key, n.intValue() + offset);
    }
  }

  private static long asLong(Object val, long fallback) {
    return val instanceof Number n ? n.longValue() : fallback;
  }

  @SuppressWarnings("unchecked")
  private static List<Object> asList(Object val) {
    return val instanceof List<?> l ? (List<Object>) l : List.of();
  }

  @SuppressWarnings("unchecked")
  private static Map<String, Object> deepCopyMap(Map<String, Object> src) {
    Map<String, Object> copy = new LinkedHashMap<>();
    for (Map.Entry<String, Object> e : src.entrySet()) {
      copy.put(e.getKey(), deepCopyValue(e.getValue()));
    }
    return copy;
  }

  @SuppressWarnings("unchecked")
  private static Object deepCopyValue(Object val) {
    if (val instanceof Map<?, ?> m) {
      return deepCopyMap((Map<String, Object>) m);
    }
    if (val instanceof List<?> l) {
      List<Object> copy = new ArrayList<>(l.size());
      for (Object item : l) {
        copy.add(deepCopyValue(item));
      }
      return copy;
    }
    return val;
  }

  private static int pad4(ByteArrayOutputStream baos, byte padByte) {
    int rem = baos.size() % 4;
    if (rem != 0) {
      for (int i = 0; i < 4 - rem; i++) {
        baos.write(padByte);
      }
    }
    return baos.size();
  }
}
