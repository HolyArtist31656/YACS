// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.manifest;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.stream.Stream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import javax.xml.XMLConstants;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.xml.sax.SAXException;
import yacsim.core.advantagescope.CadCameraConfig;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.cad.GlbMerger;
import yacsim.core.physics.CadMassProperties;
import yacsim.core.util.MinimalJson;
import yacsim.core.yams.MotionAxis;

/**
 * Zero-dependency loader and serializer for {@code yacsim-robot.json} CAD manifests and standard
 * URDF ({@code .urdf} / {@code .xml}) robot descriptions. Uses {@link MinimalJson} for JSON and
 * the JDK's built-in XXE-hardened {@link DocumentBuilderFactory} for URDF XML so zero third-party
 * runtime JARs are added to SystemCore.
 */
public final class CadManifestLoader {
  private static final double DEFAULT_CAMERA_FOV_DEGREES = 70.0;
  private static final int DEFAULT_CAMERA_WIDTH_PX = 1280;
  private static final int DEFAULT_CAMERA_HEIGHT_PX = 720;

  private static final List<String> CAMERA_NAME_PREFIXES =
      List.of("cam_", "cam-", "camera_", "camera-", "yacsim_cam_", "yacsim_camera_");

  private static final List<String> CAMERA_NAME_KEYWORDS =
      List.of(
          "limelight",
          "photon",
          "arducam",
          "ov9281",
          "ov2311",
          "questnav",
          "apriltag_cam",
          "vision_cam");

  private record CameraOpticsPreset(
      String token, double fovDegrees, int widthPx, int heightPx) {}

  private static final List<CameraOpticsPreset> CAMERA_OPTICS_PRESETS =
      List.of(
          new CameraOpticsPreset("limelight4", 82.0, 1280, 800),
          new CameraOpticsPreset("limelight_4", 82.0, 1280, 800),
          new CameraOpticsPreset("ll4", 82.0, 1280, 800),
          new CameraOpticsPreset("limelight3g", 82.0, 1280, 960),
          new CameraOpticsPreset("limelight_3g", 82.0, 1280, 960),
          new CameraOpticsPreset("ll3g", 82.0, 1280, 960),
          new CameraOpticsPreset("limelight3", 63.3, 1280, 960),
          new CameraOpticsPreset("limelight_3", 63.3, 1280, 960),
          new CameraOpticsPreset("ll3", 63.3, 1280, 960),
          new CameraOpticsPreset("limelight2", 59.6, 960, 720),
          new CameraOpticsPreset("limelight_2", 59.6, 960, 720),
          new CameraOpticsPreset("ll2", 59.6, 960, 720),
          new CameraOpticsPreset("ov9281", 70.0, 1280, 800),
          new CameraOpticsPreset("ov2311", 70.0, 1600, 1200),
          new CameraOpticsPreset("questnav", 110.0, 1280, 720));

  private CadManifestLoader() {}

  /**
   * Loads a {@link CadRobotManifest} from a {@code yacsim-robot.json} file.
   *
   * @param manifestPath Path to the JSON manifest file.
   * @return Parsed {@link CadRobotManifest}.
   * @throws IOException If the file cannot be read.
   */
  public static CadRobotManifest loadJson(Path manifestPath) throws IOException {
    Objects.requireNonNull(manifestPath, "manifestPath");
    String text = Files.readString(manifestPath, StandardCharsets.UTF_8);
    return parseJson(text);
  }

  /**
   * Parses a {@link CadRobotManifest} from raw JSON text.
   *
   * @param jsonText JSON string adhering to the {@code yacsim-robot.json} schema.
   * @return Parsed {@link CadRobotManifest}.
   */
  @SuppressWarnings("unchecked")
  public static CadRobotManifest parseJson(String jsonText) {
    Objects.requireNonNull(jsonText, "jsonText");
    Map<String, Object> root = (Map<String, Object>) MinimalJson.parse(jsonText);
    String schemaVersion = stringOrDefault(root, "schemaVersion", CadRobotManifest.CURRENT_SCHEMA_VERSION);
    String robotName = requireString(root, "robotName");
    Optional<String> sourceUrl = optionalString(root, "sourceUrl");
    OptionalDouble extXy = optionalDouble(root, "extensionXyMarginMeters");
    OptionalDouble extMaxZ = optionalDouble(root, "extensionMaxHeightMeters");
    boolean ignoreResting = booleanOrDefault(root, "ignoreRestingOverlaps", false);
    boolean autoGround = booleanOrDefault(root, "autoGroundChassis", false);
    boolean disableSimp = booleanOrDefault(root, "disableAdvantageScopeSimplification", false);

    Optional<CadLinkManifest> chassis =
        root.get("chassis") instanceof Map<?, ?> m
            ? Optional.of(parseLink((Map<String, Object>) m))
            : Optional.empty();

    List<CadLinkManifest> links = new ArrayList<>();
    if (root.get("links") instanceof List<?> linkList) {
      for (Object obj : linkList) {
        links.add(parseLink((Map<String, Object>) obj));
      }
    }

    List<CadJointManifest> joints = new ArrayList<>();
    if (root.get("joints") instanceof List<?> jointList) {
      for (Object obj : jointList) {
        joints.add(parseJoint((Map<String, Object>) obj));
      }
    }

    List<CadCameraConfig> cameras = new ArrayList<>();
    if (root.get("cameras") instanceof List<?> camList) {
      for (Object obj : camList) {
        cameras.add(parseCamera((Map<String, Object>) obj));
      }
    }

    return new CadRobotManifest(
        schemaVersion,
        robotName,
        sourceUrl,
        extXy,
        extMaxZ,
        ignoreResting,
        autoGround,
        disableSimp,
        chassis,
        links,
        joints,
        cameras);
  }

  /**
   * Imports a standard URDF robot description ({@code .urdf} or {@code .xml}) generated by CAD
   * exporters such as {@code onshape-to-robot}, Fusion 360 URDF exporters, or SolidWorks {@code
   * sw_urdf_exporter} directly into a {@link CadRobotManifest}.
   *
   * <p>The root link (any link not referenced as a joint {@code <child>}) becomes {@link
   * CadRobotManifest#chassis()}, and all remaining links become {@link CadRobotManifest#links()}.
   * URDF {@code package://<pkg>/} mesh prefixes are stripped so mesh paths resolve relative to the
   * URDF file's directory.
   *
   * @param urdfPath Path to a URDF XML file.
   * @return Imported {@link CadRobotManifest}.
   * @throws IOException If reading or parsing the URDF file fails.
   */
  /**
   * Resolves the mesh base directory for a URDF file. For ROS-style packages exported by Onshape
   * ({@code <pkg>/urdf/<robot>.urdf} with sibling {@code <pkg>/meshes/*.glb}), automatically
   * returns the package root {@code <pkg>/} so {@code package://<pkg>/meshes/...} resolves cleanly.
   *
   * @param urdfPath Path to the {@code .urdf} or {@code .xml} file.
   * @return Base directory for resolving relative mesh paths.
   */
  public static Path resolveUrdfBaseDir(Path urdfPath) {
    Objects.requireNonNull(urdfPath, "urdfPath");
    Path parent = urdfPath.toAbsolutePath().normalize().getParent();
    if (parent == null) {
      return Path.of(".");
    }
    if (parent.getFileName() != null
        && "urdf".equalsIgnoreCase(parent.getFileName().toString())
        && parent.getParent() != null
        && Files.isDirectory(parent.getParent().resolve("meshes"))) {
      return parent.getParent();
    }
    return parent;
  }

  public static CadRobotManifest loadUrdf(Path urdfPath) throws IOException {
    return loadUrdf(urdfPath, resolveUrdfBaseDir(urdfPath));
  }

  /**
   * Imports a standard URDF XML file using an explicit {@code baseDir} to resolve relative mesh
   * paths and merge multi-{@code <visual>} {@code .glb} links (such as native Onshape URDF
   * assemblies).
   *
   * @param urdfPath Path to a URDF XML file.
   * @param baseDir Base directory used to resolve relative mesh paths.
   * @return Imported {@link CadRobotManifest}.
   * @throws IOException If reading or parsing the URDF file fails.
   */
  public static CadRobotManifest loadUrdf(Path urdfPath, Path baseDir) throws IOException {
    return loadUrdf(urdfPath, baseDir, CadUrdfOptions.defaults());
  }

  /**
   * Imports a standard URDF XML file using an explicit {@code baseDir} and {@link CadUrdfOptions}
   * (code-side fastener filtering, custom mesh exclusions, joint aliasing, and unbound collision
   * control).
   *
   * @param urdfPath Path to a URDF XML file.
   * @param baseDir Base directory used to resolve relative mesh paths.
   * @param options Filtering, aliasing, and performance options.
   * @return Imported {@link CadRobotManifest}.
   * @throws IOException If reading or parsing the URDF file fails.
   */
  public static CadRobotManifest loadUrdf(
      Path urdfPath, Path baseDir, CadUrdfOptions options) throws IOException {
    Objects.requireNonNull(urdfPath, "urdfPath");
    Objects.requireNonNull(baseDir, "baseDir");
    Objects.requireNonNull(options, "options");
    Document doc;
    try (InputStream in = Files.newInputStream(urdfPath)) {
      DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
      factory.setFeature(XMLConstants.FEATURE_SECURE_PROCESSING, true);
      factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
      factory.setFeature("http://xml.org/sax/features/external-general-entities", false);
      factory.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
      factory.setXIncludeAware(false);
      factory.setExpandEntityReferences(false);
      DocumentBuilder builder = factory.newDocumentBuilder();
      doc = builder.parse(in);
    } catch (ParserConfigurationException | SAXException e) {
      throw new IOException("Failed to parse URDF XML: " + urdfPath, e);
    }

    Element robotEl = doc.getDocumentElement();
    if (robotEl == null || !"robot".equals(robotEl.getTagName())) {
      throw new IOException("URDF file missing root <robot> element: " + urdfPath);
    }
    String robotName = robotEl.getAttribute("name");
    if (robotName == null || robotName.isBlank()) {
      robotName = urdfPath.getFileName().toString().replaceFirst("\\.[^.]+$", "");
    }

    List<Element> linkElements = new ArrayList<>();
    List<CadJointManifest> joints = new ArrayList<>();
    Set<String> childLinkNames = new LinkedHashSet<>();

    NodeList children = robotEl.getChildNodes();
    for (int i = 0; i < children.getLength(); i++) {
      Node node = children.item(i);
      if (!(node instanceof Element el)) {
        continue;
      }
      if ("link".equals(el.getTagName())) {
        linkElements.add(el);
      } else if ("joint".equals(el.getTagName())) {
        CadJointManifest joint = parseUrdfJoint(el, options);
        joints.add(joint);
        childLinkNames.add(joint.childLink());
      }
    }

    // Identify the root/chassis link first so unified URDF Z-grounding can shift both the chassis
    // GLB and all root-attached joints together by the exact same -minZ offset.
    Element chassisEl = null;
    for (Element el : linkElements) {
      if (!childLinkNames.contains(el.getAttribute("name"))) {
        chassisEl = el;
        break;
      }
    }
    if (chassisEl == null && !linkElements.isEmpty()) {
      chassisEl = linkElements.get(0);
    }

    Set<String> allowedChildLinks = null;
    if (!options.getKeepOnlyJoints().isEmpty()) {
      allowedChildLinks = new LinkedHashSet<>();
      for (int i = 0; i < children.getLength(); i++) {
        if (children.item(i) instanceof Element el && "joint".equals(el.getTagName())) {
          String rawName = el.getAttribute("name");
          String aliased = options.resolveJointAlias(rawName);
          if (options.isJointAllowed(rawName, aliased)) {
            Element childEl = firstChildElement(el, "child");
            if (childEl != null && !childEl.getAttribute("link").isBlank()) {
              allowedChildLinks.add(childEl.getAttribute("link"));
            }
          }
        }
      }
    }

    Map<String, List<GlbMerger.VisualInstance>> bakedInstancesByHostLink = new LinkedHashMap<>();
    String chassisLinkName = chassisEl != null ? chassisEl.getAttribute("name") : "root";
    if ((allowedChildLinks != null && options.isBakeUnboundLinksIntoChassis())
        || !options.getJointLinkAttachments().isEmpty()) {
      Map<String, CadJointManifest> parentJointByChild = new LinkedHashMap<>();
      Map<String, String> childLinkByNormalizedJointName = new LinkedHashMap<>();
      for (CadJointManifest j : joints) {
        parentJointByChild.putIfAbsent(j.childLink(), j);
        childLinkByNormalizedJointName.putIfAbsent(
            CadRobotManifest.normalizeName(j.name()), j.childLink());
      }
      // Also map raw (pre-alias) joint names so attachLinksToJoint works with either raw or aliased names
      for (int i = 0; i < children.getLength(); i++) {
        if (children.item(i) instanceof Element el && "joint".equals(el.getTagName())) {
          String rawName = el.getAttribute("name");
          Element childEl = firstChildElement(el, "child");
          if (rawName != null && !rawName.isBlank() && childEl != null) {
            childLinkByNormalizedJointName.putIfAbsent(
                CadRobotManifest.normalizeName(rawName), childEl.getAttribute("link"));
          }
        }
      }

      Map<String, Pose3d> restingPoseCache = new LinkedHashMap<>();
      for (Element el : linkElements) {
        if (el == chassisEl) {
          continue;
        }
        String lName = el.getAttribute("name");
        boolean isAllowedMovingLink = allowedChildLinks != null && allowedChildLinks.contains(lName);
        if (isAllowedMovingLink || options.shouldExcludeMesh(lName)) {
          continue;
        }
        List<ParsedUrdfVisual> visuals = collectUrdfMeshVisuals(el, options);
        if (visuals.isEmpty()) {
          continue;
        }

        // Determine host link: 1) explicit attachLinksToJoint, 2) nearest kept moving URDF ancestor, 3) chassis
        String hostLink = null;
        Optional<String> explicitTargetJoint = options.findAttachedTargetJoint(lName, null);
        if (explicitTargetJoint.isEmpty()) {
          for (ParsedUrdfVisual v : visuals) {
            explicitTargetJoint = options.findAttachedTargetJoint(lName, v.meshFile());
            if (explicitTargetJoint.isPresent()) {
              break;
            }
          }
        }
        if (explicitTargetJoint.isPresent()) {
          hostLink = childLinkByNormalizedJointName.get(explicitTargetJoint.get());
        }
        if (hostLink == null && allowedChildLinks != null && options.isBakeUnboundLinksIntoChassis()) {
          hostLink = findNearestKeptAncestor(lName, allowedChildLinks, parentJointByChild);
          if (hostLink == null) {
            hostLink = chassisLinkName;
          }
        }
        if (hostLink == null || hostLink.equals(lName)) {
          continue;
        }

        Pose3d linkRestingPose =
            restingPoseCache.computeIfAbsent(
                lName, k -> computeCumulativeUrdfPose(k, parentJointByChild));
        Pose3d hostRestingPose =
            hostLink.equals(chassisLinkName)
                ? new Pose3d()
                : restingPoseCache.computeIfAbsent(
                    hostLink, k -> computeCumulativeUrdfPose(k, parentJointByChild));

        List<GlbMerger.VisualInstance> hostList =
            bakedInstancesByHostLink.computeIfAbsent(hostLink, k -> new ArrayList<>());
        for (ParsedUrdfVisual v : visuals) {
          if (!v.meshFile().toLowerCase(Locale.ROOT).endsWith(".glb")) {
            continue;
          }
          Path resolved = baseDir.resolve(v.meshFile()).normalize();
          if (!Files.isRegularFile(resolved)) {
            continue;
          }
          Pose3d visualWorld =
              linkRestingPose.transformBy(
                  new Transform3d(v.pos(), new Rotation3d(v.rpy()[0], v.rpy()[1], v.rpy()[2])));
          Transform3d relToHost = new Transform3d(hostRestingPose, visualWorld);
          boolean noSimp =
              options.shouldTagNoSimplify(lName) || options.shouldTagNoSimplify(v.meshFile());
          hostList.add(
              new GlbMerger.VisualInstance(
                  resolved,
                  relToHost.getTranslation(),
                  relToHost.getRotation(),
                  v.unitScale(),
                  noSimp));
        }
      }
    }

    double unifiedGroundShiftZ = 0.0;
    CadLinkManifest chassis = null;
    List<CadLinkManifest> articulated = new ArrayList<>();
    for (Element el : linkElements) {
      if (el == chassisEl) {
        List<GlbMerger.VisualInstance> extraChassis =
            bakedInstancesByHostLink.getOrDefault(chassisLinkName, List.of());
        ParsedLinkResult res =
            parseUrdfLinkWithGrounding(el, baseDir, true, options, true, extraChassis);
        chassis = res.link();
        unifiedGroundShiftZ = res.groundShiftZ();
      } else {
        String lName = el.getAttribute("name");
        boolean reattachedElsewhere =
            options.findAttachedTargetJoint(lName, null).isPresent()
                && (allowedChildLinks == null || !allowedChildLinks.contains(lName));
        boolean keepMesh =
            !reattachedElsewhere && (allowedChildLinks == null || allowedChildLinks.contains(lName));
        List<GlbMerger.VisualInstance> extraForLink =
            keepMesh ? bakedInstancesByHostLink.getOrDefault(lName, List.of()) : List.of();
        articulated.add(
            parseUrdfLinkWithGrounding(el, baseDir, false, options, keepMesh, extraForLink).link());
      }
    }

    if (chassis != null && Math.abs(unifiedGroundShiftZ) > 1e-6) {
      String chassisName = chassis.name();
      double shiftZ = unifiedGroundShiftZ;
      List<CadJointManifest> shiftedJoints = new ArrayList<>(joints.size());
      for (CadJointManifest j : joints) {
        if (j.parentLink().equals(chassisName)) {
          Translation3d o = j.originMeters();
          shiftedJoints.add(
              new CadJointManifest(
                  j.name(),
                  j.type(),
                  j.parentLink(),
                  j.childLink(),
                  new Translation3d(o.getX(), o.getY(), o.getZ() + shiftZ),
                  j.originRotation(),
                  j.zeroOffset(),
                  j.axis(),
                  j.inverted(),
                  j.minLimit(),
                  j.maxLimit(),
                  j.softMinLimit(),
                  j.softMaxLimit(),
                  j.startingPosition(),
                  j.lengthMeters(),
                  j.wheelRadiusMeters(),
                  j.momentOfInertiaKgM2()));
        } else {
          shiftedJoints.add(j);
        }
      }
      joints = shiftedJoints;
    }

    List<CadCameraConfig> cameras = extractUrdfCameras(linkElements, chassisEl, joints);

    return new CadRobotManifest(
        CadRobotManifest.CURRENT_SCHEMA_VERSION,
        robotName,
        Optional.of(urdfPath.getFileName().toString()),
        OptionalDouble.empty(),
        OptionalDouble.empty(),
        true,
        false,
        false,
        Optional.ofNullable(chassis),
        articulated,
        joints,
        cameras);
  }

  /**
   * Pair of a parsed {@link CadRobotManifest} and the directory used to resolve relative mesh paths
   * (such as binary {@code .glb} or {@code .stl} files extracted from a native Onshape URDF {@code
   * .zip} export).
   *
   * @param manifest Parsed {@link CadRobotManifest}.
   * @param baseDir Base directory containing the URDF file and its relative mesh paths.
   */
  public record UrdfLoadResult(CadRobotManifest manifest, Path baseDir) {
    /** Compact constructor validating non-null result fields. */
    public UrdfLoadResult {
      Objects.requireNonNull(manifest, "manifest");
      Objects.requireNonNull(baseDir, "baseDir");
    }
  }

  /**
   * Extracts a native Onshape (or generic) URDF {@code .zip} bundle into a temporary directory,
   * auto-discovers the {@code .urdf} descriptor inside the archive, and loads its {@link
   * CadRobotManifest} alongside the extracted base directory for mesh resolution.
   *
   * <p>Zero third-party dependencies are required; uses {@link ZipInputStream} with Zip-Slip path
   * traversal protection.
   *
   * @param zipPath Path to the exported URDF {@code .zip} archive.
   * @return {@link UrdfLoadResult} containing the parsed manifest and extracted mesh base directory.
   * @throws IOException If extraction fails, a Zip-Slip traversal entry is detected, or no URDF
   *     file is found inside the archive.
   */
  public static UrdfLoadResult loadUrdfZip(Path zipPath) throws IOException {
    return loadUrdfZip(zipPath, CadUrdfOptions.defaults());
  }

  /**
   * Extracts a native Onshape (or generic) URDF {@code .zip} bundle into a temporary directory and
   * loads it with the supplied {@link CadUrdfOptions}.
   *
   * @param zipPath Path to the exported URDF {@code .zip} archive.
   * @param options Filtering, aliasing, and performance options.
   * @return {@link UrdfLoadResult} containing the parsed manifest and extracted mesh base directory.
   * @throws IOException If extraction or URDF parsing fails.
   */
  public static UrdfLoadResult loadUrdfZip(Path zipPath, CadUrdfOptions options)
      throws IOException {
    Objects.requireNonNull(zipPath, "zipPath");
    Path tempDir = Files.createTempDirectory("yacsim-urdf-");
    return loadUrdfZip(zipPath, tempDir, options);
  }

  /**
   * Extracts a native Onshape (or generic) URDF {@code .zip} bundle into {@code extractDir},
   * auto-discovers the {@code .urdf} descriptor inside the archive, and loads its {@link
   * CadRobotManifest}.
   *
   * @param zipPath Path to the exported URDF {@code .zip} archive.
   * @param extractDir Target directory where archive entries are extracted.
   * @return {@link UrdfLoadResult} containing the parsed manifest and extracted mesh base directory.
   * @throws IOException If extraction fails, a Zip-Slip traversal entry is detected, or no URDF
   *     file is found inside the archive.
   */
  public static UrdfLoadResult loadUrdfZip(Path zipPath, Path extractDir) throws IOException {
    return loadUrdfZip(zipPath, extractDir, CadUrdfOptions.defaults());
  }

  /**
   * Extracts a native Onshape (or generic) URDF {@code .zip} bundle into {@code extractDir} and
   * loads it with the supplied {@link CadUrdfOptions}.
   *
   * @param zipPath Path to the exported URDF {@code .zip} archive.
   * @param extractDir Target directory where archive entries are extracted.
   * @param options Filtering, aliasing, and performance options.
   * @return {@link UrdfLoadResult} containing the parsed manifest and extracted mesh base directory.
   * @throws IOException If extraction or URDF parsing fails.
   */
  public static UrdfLoadResult loadUrdfZip(
      Path zipPath, Path extractDir, CadUrdfOptions options) throws IOException {
    Objects.requireNonNull(zipPath, "zipPath");
    Objects.requireNonNull(extractDir, "extractDir");
    Objects.requireNonNull(options, "options");
    Files.createDirectories(extractDir);
    Path extractRoot = extractDir.toAbsolutePath().normalize();

    try (InputStream fis = Files.newInputStream(zipPath);
        ZipInputStream zis = new ZipInputStream(fis)) {
      ZipEntry entry;
      while ((entry = zis.getNextEntry()) != null) {
        String entryName = entry.getName().replace('\\', '/');
        if (entryName.startsWith("__MACOSX/") || entryName.isBlank()) {
          zis.closeEntry();
          continue;
        }
        Path dest = extractRoot.resolve(entryName).normalize();
        if (!dest.startsWith(extractRoot)) {
          throw new IOException(
              "Blocked Zip-Slip path traversal entry in URDF archive: " + entry.getName());
        }
        if (entry.isDirectory()) {
          Files.createDirectories(dest);
        } else {
          if (dest.getParent() != null) {
            Files.createDirectories(dest.getParent());
          }
          Files.copy(zis, dest, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
        }
        zis.closeEntry();
      }
    }

    Path urdfFile = findUrdfFileInDirectory(extractRoot);
    Path baseDir = resolveUrdfBaseDir(urdfFile);
    return new UrdfLoadResult(loadUrdf(urdfFile, baseDir, options), baseDir);
  }

  /**
   * Auto-detects and loads a URDF robot description from a {@code .zip} archive, a {@code .urdf} /
   * {@code .xml} file, or an extracted directory containing a {@code .urdf} file.
   *
   * @param path Path to a URDF {@code .zip} bundle, {@code .urdf}/{@code .xml} file, or folder.
   * @return {@link UrdfLoadResult} containing the parsed manifest and mesh base directory.
   * @throws IOException If reading, extraction, or URDF discovery fails.
   */
  public static UrdfLoadResult loadUrdfOrZip(Path path) throws IOException {
    return loadUrdfOrZip(path, CadUrdfOptions.defaults());
  }

  /**
   * Auto-detects and loads a URDF robot description from a {@code .zip} archive, a {@code .urdf} /
   * {@code .xml} file, or an extracted folder using the supplied {@link CadUrdfOptions}.
   *
   * @param path Path to a URDF {@code .zip} bundle, {@code .urdf}/{@code .xml} file, or folder.
   * @param options Filtering, aliasing, and performance options.
   * @return {@link UrdfLoadResult} containing the parsed manifest and mesh base directory.
   * @throws IOException If reading, extraction, or URDF discovery fails.
   */
  public static UrdfLoadResult loadUrdfOrZip(Path path, CadUrdfOptions options) throws IOException {
    Objects.requireNonNull(path, "path");
    Objects.requireNonNull(options, "options");
    if (Files.isDirectory(path)) {
      Path dir = path.toAbsolutePath().normalize();
      Path urdfFile = findUrdfFileInDirectory(dir);
      Path baseDir = resolveUrdfBaseDir(urdfFile);
      return new UrdfLoadResult(loadUrdf(urdfFile, baseDir, options), baseDir);
    }
    String lower = path.getFileName().toString().toLowerCase(Locale.ROOT);
    if (lower.endsWith(".zip")) {
      return loadUrdfZip(path, options);
    }
    Path baseDir = resolveUrdfBaseDir(path);
    CadRobotManifest manifest = loadUrdf(path, baseDir, options);
    return new UrdfLoadResult(manifest, baseDir);
  }

  private static Path findUrdfFileInDirectory(Path rootDir) throws IOException {
    List<Path> urdfMatches;
    List<Path> xmlMatches;
    try (Stream<Path> walk = Files.walk(rootDir)) {
      List<Path> regularFiles =
          walk.filter(Files::isRegularFile)
              .sorted(
                  Comparator.comparingInt((Path p) -> rootDir.relativize(p).getNameCount())
                      .thenComparing(p -> rootDir.relativize(p).toString()))
              .toList();
      urdfMatches =
          regularFiles.stream()
              .filter(p -> p.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".urdf"))
              .toList();
      xmlMatches =
          regularFiles.stream()
              .filter(p -> p.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".xml"))
              .toList();
    }
    if (!urdfMatches.isEmpty()) {
      return urdfMatches.get(0);
    }
    if (!xmlMatches.isEmpty()) {
      return xmlMatches.get(0);
    }
    throw new IOException("No .urdf or .xml file found in URDF directory/archive: " + rootDir);
  }

  /**
   * Serializes a {@link CadRobotManifest} into formatted {@code yacsim-robot.json} text.
   *
   * @param manifest Manifest to serialize.
   * @return Pretty-printed JSON string.
   */
  public static String toPrettyJson(CadRobotManifest manifest) {
    Objects.requireNonNull(manifest, "manifest");
    Map<String, Object> root = new LinkedHashMap<>();
    root.put("schemaVersion", manifest.schemaVersion());
    root.put("robotName", manifest.robotName());
    manifest.sourceUrl().ifPresent(url -> root.put("sourceUrl", url));
    manifest.extensionXyMarginMeters().ifPresent(v -> root.put("extensionXyMarginMeters", v));
    manifest.extensionMaxHeightMeters().ifPresent(v -> root.put("extensionMaxHeightMeters", v));
    root.put("ignoreRestingOverlaps", manifest.ignoreRestingOverlaps());
    root.put("autoGroundChassis", manifest.autoGroundChassis());
    root.put(
        "disableAdvantageScopeSimplification", manifest.disableAdvantageScopeSimplification());
    manifest.chassis().ifPresent(c -> root.put("chassis", linkToMap(c)));

    List<Object> links = new ArrayList<>();
    for (CadLinkManifest link : manifest.links()) {
      links.add(linkToMap(link));
    }
    root.put("links", links);

    List<Object> joints = new ArrayList<>();
    for (CadJointManifest joint : manifest.joints()) {
      joints.add(jointToMap(joint));
    }
    root.put("joints", joints);

    List<Object> cameras = new ArrayList<>();
    for (CadCameraConfig cam : manifest.cameras()) {
      cameras.add(cameraToMap(cam));
    }
    root.put("cameras", cameras);
    return MinimalJson.toPrettyJson(root);
  }

  /**
   * Writes a {@link CadRobotManifest} to disk as formatted {@code yacsim-robot.json}.
   *
   * @param manifest Manifest to write.
   * @param outputPath Destination path.
   * @throws IOException If writing fails.
   */
  public static void writeJson(CadRobotManifest manifest, Path outputPath) throws IOException {
    Objects.requireNonNull(outputPath, "outputPath");
    if (outputPath.getParent() != null) {
      Files.createDirectories(outputPath.getParent());
    }
    Files.writeString(outputPath, toPrettyJson(manifest), StandardCharsets.UTF_8);
  }

  // ---- JSON parsing helpers ----

  @SuppressWarnings("unchecked")
  private static CadLinkManifest parseLink(Map<String, Object> m) {
    String name = requireString(m, "name");
    String meshFile = optionalString(m, "meshFile").orElse(null);
    double unitScale = optionalDouble(m, "unitScaleToMeters").orElse(1.0);
    Translation3d pos = parseTranslation((List<Object>) m.get("zeroedPosition"));
    List<CadAssetConfig.AxisRotation> rots =
        parseAxisRotations((List<Object>) m.get("zeroedRotations"));
    Optional<CadMassProperties> massProps =
        m.get("massProperties") instanceof Map<?, ?> mp
            ? Optional.of(parseMassProperties((Map<String, Object>) mp))
            : Optional.empty();
    boolean collisionEnabled = booleanOrDefault(m, "collisionEnabled", true);
    return new CadLinkManifest(
        name, meshFile, unitScale, pos, rots, massProps, collisionEnabled);
  }

  @SuppressWarnings("unchecked")
  private static CadMassProperties parseMassProperties(Map<String, Object> mp) {
    double volume = optionalDouble(mp, "volumeCubicMeters").orElse(0.0);
    double mass = requireDouble(mp, "massKilograms");
    Translation3d com = parseTranslation((List<Object>) mp.get("centerOfMassMeters"));
    double ixx = optionalDouble(mp, "ixxKgM2").orElse(0.0);
    double iyy = optionalDouble(mp, "iyyKgM2").orElse(0.0);
    double izz = optionalDouble(mp, "izzKgM2").orElse(0.0);
    return new CadMassProperties(volume, mass, com, ixx, iyy, izz);
  }

  @SuppressWarnings("unchecked")
  private static CadJointManifest parseJoint(Map<String, Object> m) {
    String name = requireString(m, "name");
    CadJointType type = CadJointType.fromString(requireString(m, "type"));
    String parent = stringOrDefault(m, "parentLink", "Chassis");
    String child = stringOrDefault(m, "childLink", name);
    Translation3d origin = parseTranslation((List<Object>) m.get("originMeters"));
    Rotation3d originRot = parseRpyRadians((List<Object>) m.get("originRpyRadians"));
    Transform3d zeroOffset = new Transform3d();
    if (m.get("zeroOffset") instanceof Map<?, ?> zMap) {
      Map<String, Object> zm = (Map<String, Object>) zMap;
      zeroOffset =
          new Transform3d(
              parseTranslation((List<Object>) zm.get("translationMeters")),
              parseRpyRadians((List<Object>) zm.get("rpyRadians")));
    }
    MotionAxis axis =
        optionalString(m, "axis").map(CadManifestLoader::parseMotionAxis).orElse(null);
    boolean inverted = booleanOrDefault(m, "inverted", false);
    OptionalDouble minLimit = optionalDouble(m, "minLimit");
    OptionalDouble maxLimit = optionalDouble(m, "maxLimit");
    OptionalDouble softMin = optionalDouble(m, "softMinLimit");
    OptionalDouble softMax = optionalDouble(m, "softMaxLimit");
    OptionalDouble startPos = optionalDouble(m, "startingPosition");
    OptionalDouble length = optionalDouble(m, "lengthMeters");
    OptionalDouble wheelRadius = optionalDouble(m, "wheelRadiusMeters");
    OptionalDouble moi = optionalDouble(m, "momentOfInertiaKgM2");
    return new CadJointManifest(
        name,
        type,
        parent,
        child,
        origin,
        originRot,
        zeroOffset,
        axis,
        inverted,
        minLimit,
        maxLimit,
        softMin,
        softMax,
        startPos,
        length,
        wheelRadius,
        moi);
  }

  @SuppressWarnings("unchecked")
  private static CadCameraConfig parseCamera(Map<String, Object> c) {
    String name = requireString(c, "name");
    Translation3d pos = parseTranslation((List<Object>) c.get("position"));
    Rotation3d rot =
        c.get("rpyRadians") instanceof List<?> rpy
            ? parseRpyRadians((List<Object>) rpy)
            : composeAxisRotations(parseAxisRotations((List<Object>) c.get("rotations")));
    double fov = optionalDouble(c, "fovDegrees").orElse(optionalDouble(c, "fov").orElse(70.0));
    int w = 1280;
    int h = 720;
    if (c.get("resolution") instanceof List<?> res && res.size() >= 2) {
      w = ((Number) res.get(0)).intValue();
      h = ((Number) res.get(1)).intValue();
    }
    return new CadCameraConfig(name, new Transform3d(pos, rot), fov, w, h);
  }

  // ---- URDF XML parsing helpers ----

  private record ParsedUrdfVisual(
      String meshFile,
      double unitScale,
      Translation3d pos,
      double[] rpy,
      List<CadAssetConfig.AxisRotation> axisRots) {}

  private record ParsedLinkResult(CadLinkManifest link, double groundShiftZ) {}

  private static String findNearestKeptAncestor(
      String linkName,
      Set<String> allowedChildLinks,
      Map<String, CadJointManifest> parentJointByChild) {
    Set<String> visited = new LinkedHashSet<>();
    String cur = linkName;
    while (cur != null && visited.add(cur)) {
      CadJointManifest parentJoint = parentJointByChild.get(cur);
      if (parentJoint == null) {
        return null;
      }
      String parent = parentJoint.parentLink();
      if (allowedChildLinks.contains(parent)) {
        return parent;
      }
      cur = parent;
    }
    return null;
  }

  private static boolean isCameraLinkName(String rawName) {
    if (rawName == null || rawName.isBlank()) {
      return false;
    }
    String low = rawName.trim().toLowerCase(Locale.ROOT);
    for (String prefix : CAMERA_NAME_PREFIXES) {
      if (low.startsWith(prefix)) {
        return true;
      }
    }
    for (String kw : CAMERA_NAME_KEYWORDS) {
      if (low.contains(kw)) {
        return true;
      }
    }
    return false;
  }

  private static String cleanCameraDisplayName(String rawName) {
    String cleaned = rawName.trim();
    String low = cleaned.toLowerCase(Locale.ROOT);
    for (String prefix :
        new String[] {
          "yacsim_camera_", "yacsim_cam_", "yacsim_", "camera_", "camera-", "cam_", "cam-"
        }) {
      if (low.startsWith(prefix) && cleaned.length() > prefix.length()) {
        cleaned = cleaned.substring(prefix.length());
        break;
      }
    }
    while (cleaned.startsWith("_") || cleaned.startsWith("-")) {
      cleaned = cleaned.substring(1);
    }
    return cleaned.isBlank() ? rawName.trim() : cleaned;
  }

  private static CameraOpticsPreset inferCameraOptics(String rawName) {
    String low = rawName.trim().toLowerCase(Locale.ROOT).replace('-', '_');
    String compact = low.replace("_", "").replace(" ", "");
    for (CameraOpticsPreset preset : CAMERA_OPTICS_PRESETS) {
      if (low.contains(preset.token())
          || compact.contains(preset.token().replace("_", ""))) {
        return preset;
      }
    }
    return new CameraOpticsPreset(
        "default", DEFAULT_CAMERA_FOV_DEGREES, DEFAULT_CAMERA_WIDTH_PX, DEFAULT_CAMERA_HEIGHT_PX);
  }

  private static List<CadCameraConfig> extractUrdfCameras(
      List<Element> linkElements, Element chassisEl, List<CadJointManifest> joints) {
    Map<String, CadJointManifest> parentJointByChild = new LinkedHashMap<>();
    for (CadJointManifest j : joints) {
      parentJointByChild.putIfAbsent(j.childLink(), j);
    }
    List<CadCameraConfig> cameras = new ArrayList<>();
    Set<String> seenKeys = new LinkedHashSet<>();
    for (Element el : linkElements) {
      if (el == chassisEl) {
        continue;
      }
      String linkName = el.getAttribute("name");
      if (!isCameraLinkName(linkName)) {
        continue;
      }
      String key = CadRobotManifest.normalizeCameraName(linkName);
      if (!seenKeys.add(key)) {
        continue;
      }
      Pose3d pose = computeCumulativeUrdfPose(linkName, parentJointByChild);
      Transform3d robotToCamera = new Transform3d(pose.getTranslation(), pose.getRotation());
      CameraOpticsPreset optics = inferCameraOptics(linkName);
      cameras.add(
          new CadCameraConfig(
              cleanCameraDisplayName(linkName),
              robotToCamera,
              optics.fovDegrees(),
              optics.widthPx(),
              optics.heightPx()));
    }
    return cameras;
  }

  private static Pose3d computeCumulativeUrdfPose(
      String linkName, Map<String, CadJointManifest> parentJointByChild) {
    List<CadJointManifest> chain = new ArrayList<>();
    Set<String> visited = new LinkedHashSet<>();
    String cur = linkName;
    while (cur != null && visited.add(cur)) {
      CadJointManifest parentJoint = parentJointByChild.get(cur);
      if (parentJoint == null) {
        break;
      }
      chain.add(parentJoint);
      cur = parentJoint.parentLink();
    }
    java.util.Collections.reverse(chain);
    Pose3d pose = new Pose3d();
    for (CadJointManifest j : chain) {
      pose = pose.transformBy(new Transform3d(j.originMeters(), j.originRotation()));
      if (!j.zeroOffset().equals(new Transform3d())) {
        pose = pose.transformBy(j.zeroOffset());
      }
    }
    return pose;
  }

  private static List<ParsedUrdfVisual> collectUrdfMeshVisuals(
      Element linkEl, CadUrdfOptions options) {
    List<ParsedUrdfVisual> meshVisuals = new ArrayList<>();
    NodeList linkChildren = linkEl.getChildNodes();
    for (int i = 0; i < linkChildren.getLength(); i++) {
      if (linkChildren.item(i) instanceof Element el && "visual".equals(el.getTagName())) {
        Element geom = firstChildElement(el, "geometry");
        Element mesh = geom != null ? firstChildElement(geom, "mesh") : null;
        if (mesh == null) {
          continue;
        }
        String rawFilename = mesh.getAttribute("filename");
        if (rawFilename == null || rawFilename.isBlank()) {
          continue;
        }
        String cleanedMesh = stripUrdfPackagePrefix(rawFilename);
        if (options.shouldExcludeMesh(cleanedMesh)) {
          continue;
        }
        double scale = 1.0;
        String scaleAttr = mesh.getAttribute("scale");
        if (scaleAttr != null && !scaleAttr.isBlank()) {
          double[] sc = parseUrdfVec3Array(scaleAttr);
          if (sc[0] > 0.0) {
            scale = sc[0];
          }
        }
        Translation3d pos = new Translation3d();
        double[] rpy = new double[] {0.0, 0.0, 0.0};
        List<CadAssetConfig.AxisRotation> rots = new ArrayList<>();
        Element origin = firstChildElement(el, "origin");
        if (origin != null) {
          pos = parseUrdfVec3(origin.getAttribute("xyz"));
          rpy = parseUrdfVec3Array(origin.getAttribute("rpy"));
          if (Math.abs(rpy[0]) > 1e-9) {
            rots.add(new CadAssetConfig.AxisRotation("x", Math.toDegrees(rpy[0])));
          }
          if (Math.abs(rpy[1]) > 1e-9) {
            rots.add(new CadAssetConfig.AxisRotation("y", Math.toDegrees(rpy[1])));
          }
          if (Math.abs(rpy[2]) > 1e-9) {
            rots.add(new CadAssetConfig.AxisRotation("z", Math.toDegrees(rpy[2])));
          }
        }
        meshVisuals.add(new ParsedUrdfVisual(cleanedMesh, scale, pos, rpy, rots));
      }
    }
    return meshVisuals;
  }

  private static ParsedLinkResult parseUrdfLinkWithGrounding(
      Element linkEl,
      Path baseDir,
      boolean computeGroundShift,
      CadUrdfOptions options,
      boolean keepVisualMesh,
      List<GlbMerger.VisualInstance> extraChassisInstances)
      throws IOException {
    String name = linkEl.getAttribute("name");
    String meshFile = null;
    double unitScale = 1.0;
    double appliedGroundShiftZ = 0.0;
    Translation3d visualPos = new Translation3d();
    List<CadAssetConfig.AxisRotation> visualRots = new ArrayList<>();

    boolean skipEntireLinkMesh =
        !keepVisualMesh || (!computeGroundShift && options.shouldExcludeMesh(name));

    List<ParsedUrdfVisual> meshVisuals =
        skipEntireLinkMesh ? List.of() : collectUrdfMeshVisuals(linkEl, options);

    if (!meshVisuals.isEmpty() || !extraChassisInstances.isEmpty()) {
      List<GlbMerger.VisualInstance> glbInstances = new ArrayList<>();
      boolean allGlbPresent = true;
      for (ParsedUrdfVisual v : meshVisuals) {
        if (!v.meshFile().toLowerCase(Locale.ROOT).endsWith(".glb")) {
          allGlbPresent = false;
          break;
        }
        Path resolved = baseDir.resolve(v.meshFile()).normalize();
        if (!Files.isRegularFile(resolved)) {
          allGlbPresent = false;
          break;
        }
        boolean noSimp =
            options.shouldTagNoSimplify(name) || options.shouldTagNoSimplify(v.meshFile());
        glbInstances.add(
            new GlbMerger.VisualInstance(
                resolved,
                v.pos(),
                new Rotation3d(v.rpy()[0], v.rpy()[1], v.rpy()[2]),
                v.unitScale(),
                noSimp));
      }
      if (allGlbPresent && !extraChassisInstances.isEmpty()) {
        glbInstances.addAll(extraChassisInstances);
      }
      if (allGlbPresent && !glbInstances.isEmpty()) {
        // Bake every <visual><origin xyz rpy> directly in URDF Z-up (matching Onshape URDF .glb
        // meshes and AdvantageScope's native Z-up rendering with zero config.json rotations).
        String safeName = name.replaceAll("[^A-Za-z0-9._-]", "_");
        if (safeName.isBlank()) {
          safeName = "link";
        }
        double groundShiftZ = 0.0;
        if (computeGroundShift) {
          double minZ = GlbMerger.computeTransformedMinZ(glbInstances);
          if (Math.abs(minZ) > 1e-4) {
            groundShiftZ = -minZ;
          }
        }
        String relMerged = "yacsim_merged/" + safeName + ".glb";
        appliedGroundShiftZ = groundShiftZ;
        GlbMerger.merge(glbInstances, baseDir.resolve(relMerged), false, groundShiftZ);
        meshFile = relMerged;
        unitScale = 1.0;
        visualPos = new Translation3d();
        visualRots = List.of();
      } else {
        ParsedUrdfVisual first = meshVisuals.get(0);
        meshFile = first.meshFile();
        unitScale = first.unitScale();
        visualPos = first.pos();
        visualRots = first.axisRots();
      }
    }

    Optional<CadMassProperties> massProps = Optional.empty();
    Element inertial = firstChildElement(linkEl, "inertial");
    if (inertial != null) {
      Element massEl = firstChildElement(inertial, "mass");
      if (massEl != null && !massEl.getAttribute("value").isBlank()) {
        double massKg = Double.parseDouble(massEl.getAttribute("value").trim());
        Translation3d com = new Translation3d();
        Element inOrigin = firstChildElement(inertial, "origin");
        if (inOrigin != null && !inOrigin.getAttribute("xyz").isBlank()) {
          com = parseUrdfVec3(inOrigin.getAttribute("xyz"));
        }
        double ixx = 0.0, iyy = 0.0, izz = 0.0;
        Element inertiaEl = firstChildElement(inertial, "inertia");
        if (inertiaEl != null) {
          ixx = parseDoubleAttr(inertiaEl, "ixx", 0.0);
          iyy = parseDoubleAttr(inertiaEl, "iyy", 0.0);
          izz = parseDoubleAttr(inertiaEl, "izz", 0.0);
        }
        massProps = Optional.of(new CadMassProperties(0.0, massKg, com, ixx, iyy, izz));
      }
    }

    boolean collisionEnabled = computeGroundShift || !options.isDisableUnboundCollisions();
    return new ParsedLinkResult(
        new CadLinkManifest(
            name, meshFile, unitScale, visualPos, visualRots, massProps, collisionEnabled),
        appliedGroundShiftZ);
  }

  private static CadJointManifest parseUrdfJoint(Element jointEl, CadUrdfOptions options) {
    String name = options.resolveJointAlias(jointEl.getAttribute("name"));
    CadJointType type = CadJointType.fromString(jointEl.getAttribute("type"));
    Element parentEl = firstChildElement(jointEl, "parent");
    Element childEl = firstChildElement(jointEl, "child");
    String parent = parentEl != null ? parentEl.getAttribute("link") : "Chassis";
    String child = childEl != null ? childEl.getAttribute("link") : name;

    Translation3d originPos = new Translation3d();
    Rotation3d originRot = new Rotation3d();
    Element originEl = firstChildElement(jointEl, "origin");
    if (originEl != null) {
      originPos = parseUrdfVec3(originEl.getAttribute("xyz"));
      double[] rpy = parseUrdfVec3Array(originEl.getAttribute("rpy"));
      originRot = new Rotation3d(rpy[0], rpy[1], rpy[2]);
    }

    MotionAxis axis = type == CadJointType.FIXED ? null : MotionAxis.Z;
    boolean inverted = false;
    Element axisEl = firstChildElement(jointEl, "axis");
    if (axisEl != null && !axisEl.getAttribute("xyz").isBlank()) {
      double[] a = parseUrdfVec3Array(axisEl.getAttribute("xyz"));
      double ax = Math.abs(a[0]), ay = Math.abs(a[1]), az = Math.abs(a[2]);
      if (ax >= ay && ax >= az) {
        axis = MotionAxis.X;
        inverted = a[0] < 0.0;
      } else if (ay >= ax && ay >= az) {
        axis = MotionAxis.Y;
        inverted = a[1] < 0.0;
      } else {
        axis = MotionAxis.Z;
        inverted = a[2] < 0.0;
      }
    }

    OptionalDouble minLimit = OptionalDouble.empty();
    OptionalDouble maxLimit = OptionalDouble.empty();
    Element limitEl = firstChildElement(jointEl, "limit");
    if (limitEl != null) {
      if (!limitEl.getAttribute("lower").isBlank()) {
        minLimit = OptionalDouble.of(Double.parseDouble(limitEl.getAttribute("lower").trim()));
      }
      if (!limitEl.getAttribute("upper").isBlank()) {
        maxLimit = OptionalDouble.of(Double.parseDouble(limitEl.getAttribute("upper").trim()));
      }
    }

    return new CadJointManifest(
        name,
        type,
        parent,
        child,
        originPos,
        originRot,
        new Transform3d(),
        axis,
        inverted,
        minLimit,
        maxLimit,
        OptionalDouble.empty(),
        OptionalDouble.empty(),
        OptionalDouble.empty(),
        OptionalDouble.empty(),
        OptionalDouble.empty(),
        OptionalDouble.empty());
  }

  private static String stripUrdfPackagePrefix(String raw) {
    String cleaned = raw.trim().replace('\\', '/');
    if (cleaned.startsWith("package://")) {
      cleaned = cleaned.substring("package://".length());
      int slash = cleaned.indexOf('/');
      if (slash >= 0) {
        cleaned = cleaned.substring(slash + 1);
      }
    } else if (cleaned.startsWith("file://")) {
      cleaned = cleaned.substring("file://".length());
    }
    while (cleaned.startsWith("./")) {
      cleaned = cleaned.substring(2);
    }
    return cleaned;
  }

  private static Element firstChildElement(Element parent, String tagName) {
    NodeList list = parent.getChildNodes();
    for (int i = 0; i < list.getLength(); i++) {
      if (list.item(i) instanceof Element el && tagName.equals(el.getTagName())) {
        return el;
      }
    }
    return null;
  }

  private static double[] parseUrdfVec3Array(String attr) {
    if (attr == null || attr.isBlank()) {
      return new double[] {0.0, 0.0, 0.0};
    }
    String[] parts = attr.trim().split("\\s+");
    return new double[] {
      parts.length > 0 ? Double.parseDouble(parts[0]) : 0.0,
      parts.length > 1 ? Double.parseDouble(parts[1]) : 0.0,
      parts.length > 2 ? Double.parseDouble(parts[2]) : 0.0
    };
  }

  private static Translation3d parseUrdfVec3(String attr) {
    double[] v = parseUrdfVec3Array(attr);
    return new Translation3d(v[0], v[1], v[2]);
  }

  private static double parseDoubleAttr(Element el, String attr, double fallback) {
    String s = el.getAttribute(attr);
    return (s == null || s.isBlank()) ? fallback : Double.parseDouble(s.trim());
  }

  // ---- JSON serialization helpers ----

  private static Map<String, Object> linkToMap(CadLinkManifest link) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("name", link.name());
    if (link.hasMesh()) {
      m.put("meshFile", link.meshFile());
    }
    if (Math.abs(link.unitScaleToMeters() - 1.0) > 1e-12) {
      m.put("unitScaleToMeters", link.unitScaleToMeters());
    }
    m.put(
        "zeroedPosition",
        List.of(
            link.zeroedPositionMeters().getX(),
            link.zeroedPositionMeters().getY(),
            link.zeroedPositionMeters().getZ()));
    List<Object> rots = new ArrayList<>();
    for (CadAssetConfig.AxisRotation r : link.zeroedRotations()) {
      Map<String, Object> rm = new LinkedHashMap<>();
      rm.put("axis", r.axis());
      rm.put("degrees", r.degrees());
      rots.add(rm);
    }
    m.put("zeroedRotations", rots);
    link.massProperties()
        .ifPresent(
            mp -> {
              Map<String, Object> mpm = new LinkedHashMap<>();
              mpm.put("volumeCubicMeters", mp.volumeCubicMeters());
              mpm.put("massKilograms", mp.massKilograms());
              mpm.put(
                  "centerOfMassMeters",
                  List.of(
                      mp.centerOfMassMeters().getX(),
                      mp.centerOfMassMeters().getY(),
                      mp.centerOfMassMeters().getZ()));
              mpm.put("ixxKgM2", mp.ixxKgM2());
              mpm.put("iyyKgM2", mp.iyyKgM2());
              mpm.put("izzKgM2", mp.izzKgM2());
              m.put("massProperties", mpm);
            });
    m.put("collisionEnabled", link.collisionEnabled());
    return m;
  }

  private static Map<String, Object> jointToMap(CadJointManifest j) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("name", j.name());
    m.put("type", j.type().name().toLowerCase(Locale.ROOT));
    m.put("parentLink", j.parentLink());
    m.put("childLink", j.childLink());
    m.put(
        "originMeters",
        List.of(j.originMeters().getX(), j.originMeters().getY(), j.originMeters().getZ()));
    m.put(
        "originRpyRadians",
        List.of(
            j.originRotation().getX(), j.originRotation().getY(), j.originRotation().getZ()));
    if (!j.zeroOffset().equals(new Transform3d())) {
      Map<String, Object> zm = new LinkedHashMap<>();
      zm.put(
          "translationMeters",
          List.of(
              j.zeroOffset().getX(), j.zeroOffset().getY(), j.zeroOffset().getZ()));
      zm.put(
          "rpyRadians",
          List.of(
              j.zeroOffset().getRotation().getX(),
              j.zeroOffset().getRotation().getY(),
              j.zeroOffset().getRotation().getZ()));
      m.put("zeroOffset", zm);
    }
    if (j.axis() != null) {
      m.put("axis", j.axis().name());
    }
    if (j.inverted()) {
      m.put("inverted", true);
    }
    j.minLimit().ifPresent(v -> m.put("minLimit", v));
    j.maxLimit().ifPresent(v -> m.put("maxLimit", v));
    j.softMinLimit().ifPresent(v -> m.put("softMinLimit", v));
    j.softMaxLimit().ifPresent(v -> m.put("softMaxLimit", v));
    j.startingPosition().ifPresent(v -> m.put("startingPosition", v));
    j.lengthMeters().ifPresent(v -> m.put("lengthMeters", v));
    j.wheelRadiusMeters().ifPresent(v -> m.put("wheelRadiusMeters", v));
    j.momentOfInertiaKgM2().ifPresent(v -> m.put("momentOfInertiaKgM2", v));
    return m;
  }

  private static Map<String, Object> cameraToMap(CadCameraConfig cam) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("name", cam.name());
    m.put(
        "position",
        List.of(
            cam.robotToCamera().getX(), cam.robotToCamera().getY(), cam.robotToCamera().getZ()));
    m.put(
        "rpyRadians",
        List.of(
            cam.robotToCamera().getRotation().getX(),
            cam.robotToCamera().getRotation().getY(),
            cam.robotToCamera().getRotation().getZ()));
    m.put("fovDegrees", cam.fovDegrees());
    m.put("resolution", List.of(cam.resolutionWidthPx(), cam.resolutionHeightPx()));
    return m;
  }

  // ---- Primitive parsing helpers ----

  private static String requireString(Map<String, Object> m, String key) {
    if (m.get(key) instanceof String s && !s.isBlank()) {
      return s;
    }
    throw new IllegalArgumentException("Missing required string field '" + key + "' in manifest.");
  }

  private static String stringOrDefault(Map<String, Object> m, String key, String fallback) {
    return m.get(key) instanceof String s && !s.isBlank() ? s : fallback;
  }

  private static Optional<String> optionalString(Map<String, Object> m, String key) {
    return m.get(key) instanceof String s && !s.isBlank() ? Optional.of(s) : Optional.empty();
  }

  private static double requireDouble(Map<String, Object> m, String key) {
    if (m.get(key) instanceof Number n) {
      return n.doubleValue();
    }
    throw new IllegalArgumentException("Missing required numeric field '" + key + "' in manifest.");
  }

  private static OptionalDouble optionalDouble(Map<String, Object> m, String key) {
    return m.get(key) instanceof Number n
        ? OptionalDouble.of(n.doubleValue())
        : OptionalDouble.empty();
  }

  private static boolean booleanOrDefault(Map<String, Object> m, String key, boolean fallback) {
    return m.get(key) instanceof Boolean b ? b : fallback;
  }

  private static Translation3d parseTranslation(List<Object> list) {
    if (list == null || list.size() < 3) {
      return new Translation3d();
    }
    return new Translation3d(
        ((Number) list.get(0)).doubleValue(),
        ((Number) list.get(1)).doubleValue(),
        ((Number) list.get(2)).doubleValue());
  }

  private static Rotation3d parseRpyRadians(List<Object> list) {
    if (list == null || list.size() < 3) {
      return new Rotation3d();
    }
    return new Rotation3d(
        ((Number) list.get(0)).doubleValue(),
        ((Number) list.get(1)).doubleValue(),
        ((Number) list.get(2)).doubleValue());
  }

  @SuppressWarnings("unchecked")
  private static List<CadAssetConfig.AxisRotation> parseAxisRotations(List<Object> list) {
    List<CadAssetConfig.AxisRotation> out = new ArrayList<>();
    if (list == null) {
      return out;
    }
    for (Object obj : list) {
      Map<String, Object> r = (Map<String, Object>) obj;
      out.add(
          new CadAssetConfig.AxisRotation(
              String.valueOf(r.get("axis")), ((Number) r.get("degrees")).doubleValue()));
    }
    return out;
  }

  private static Rotation3d composeAxisRotations(List<CadAssetConfig.AxisRotation> rots) {
    Rotation3d rot = new Rotation3d();
    for (CadAssetConfig.AxisRotation r : rots) {
      double rad = Math.toRadians(r.degrees());
      Rotation3d step =
          switch (r.axis()) {
            case "x" -> new Rotation3d(rad, 0.0, 0.0);
            case "y" -> new Rotation3d(0.0, rad, 0.0);
            case "z" -> new Rotation3d(0.0, 0.0, rad);
            default -> new Rotation3d();
          };
      rot = rot.rotateBy(step);
    }
    return rot;
  }

  private static MotionAxis parseMotionAxis(String raw) {
    return switch (raw.trim().toUpperCase(Locale.ROOT)) {
      case "X" -> MotionAxis.X;
      case "Y" -> MotionAxis.Y;
      case "Z" -> MotionAxis.Z;
      default -> throw new IllegalArgumentException("Invalid motion axis: " + raw);
    };
  }
}
