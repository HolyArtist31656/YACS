// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.manifest;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

/**
 * Fluent code-side options for loading native Onshape (or generic) URDF archives and XML files.
 *
 * <p>Solves real-world FRC CAD export pain points directly in Java—even when the CAD assembly has
 * un-suppressed fasteners, default Onshape mate names ({@code revolute_1}), or static simplified
 * swerve blocks baked into the chassis:
 *
 * <ul>
 *   <li>{@link #excludeFasteners(boolean)} skips bolts, nuts, screws, washers, spacers, bearings,
 *       bushings, pins, collars, and springs during URDF parsing and {@code GlbMerger} merging.
 *   <li>{@link #disableUnboundCollisions(boolean)} disables 50 Hz OBB collision checks on unbound
 *       static links (automatically re-enabling collision when a mechanism is bound) to eliminate
 *       simulation lag on high-link-count URDFs.
 *   <li>{@link #withJointAlias(String, String)} / {@link #withJointAliases(Map)} remaps default CAD
 *       joint names (e.g. {@code "revolute_1"}) to Java subsystem names (e.g. {@code "Turret"})
 *       without modifying Onshape.
 *   <li>{@link #excludeMeshPatterns(String...)} strips arbitrary static meshes (such as {@code
 *       "MAXSwerve"}) so animated swerve modules can be overlaid cleanly.
 *   <li>{@link #keepOnlyJoints(Collection)} optionally retains only the specified moving joints as
 *       articulated visual components.
 *   <li>{@link #bakeUnboundLinksIntoChassis(boolean)} merges non-fastener links excluded by {@link
 *       #keepOnlyJoints(Collection)} into {@code root.glb} at their cumulative URDF resting pose so
 *       structural plates stay visible without bloating {@code ComponentPoses}.
 *   <li>{@link #noSimplifyMeshPatterns(String...)} tags matching merged glTF nodes with {@code
 *       NOSIMPLIFY} so AdvantageScope preserves fine geometry (such as thin tubes or bumpers).
 * </ul>
 */
public final class CadUrdfOptions {
  /** Default case-insensitive substring tokens identifying FRC fasteners and small hardware. */
  public static final List<String> DEFAULT_FASTENER_TOKENS =
      List.of(
          "nylock",
          "jam_nut",
          "hex_nut",
          "locknut",
          "bolt",
          "screw",
          "shcs",
          "bhcs",
          "fhcs",
          "washer",
          "spacer",
          "bearing",
          "bushing",
          "clevis_pin",
          "retaining_ring",
          "standoff",
          "shaft_collar",
          "collar_clamp",
          "machine_key",
          "rivet",
          "zip_tie",
          "spring");

  private boolean excludeFasteners = false;
  private boolean disableUnboundCollisions = false;
  private boolean bakeUnboundLinksIntoChassis = false;
  private final List<String> excludedMeshPatterns = new ArrayList<>();
  private final List<String> noSimplifyMeshPatterns = new ArrayList<>();
  private final Map<String, String> jointAliases = new LinkedHashMap<>();
  private final Set<String> keepOnlyJoints = new LinkedHashSet<>();
  private final Map<String, List<String>> jointLinkAttachments = new LinkedHashMap<>();

  private CadUrdfOptions() {}

  /**
   * Creates default options with all code-side filters disabled (preserves 100% raw URDF behavior).
   *
   * @return Default {@link CadUrdfOptions}.
   */
  public static CadUrdfOptions defaults() {
    return new CadUrdfOptions();
  }

  /**
   * Creates recommended FRC options: {@link #excludeFasteners(boolean) excludeFasteners(true)} and
   * {@link #disableUnboundCollisions(boolean) disableUnboundCollisions(true)} for fast 50 Hz
   * simulation even on raw Onshape URDF exports.
   *
   * @return Recommended {@link CadUrdfOptions}.
   */
  public static CadUrdfOptions recommended() {
    return new CadUrdfOptions().excludeFasteners(true).disableUnboundCollisions(true);
  }

  /**
   * Enables or disables automatic code-side filtering of fastener/hardware meshes and links
   * (screws, nuts, bolts, washers, spacers, bearings, bushings, pins, collars, standoffs, springs).
   *
   * @param enabled {@code true} to skip fastener meshes during URDF load and GLB merging.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions excludeFasteners(boolean enabled) {
    this.excludeFasteners = enabled;
    return this;
  }

  /**
   * Disables 50 Hz OBB collision checks on unbound URDF links so raw multi-link URDFs never lag;
   * collision checking is automatically re-enabled on any articulated component once bound via
   * {@code withComponentPoseSupplier} or {@code YamsCommands3Bridge.bindFromManifest} / {@code
   * autoBind}.
   *
   * @param enabled {@code true} to disable OBB collision on unbound links.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions disableUnboundCollisions(boolean enabled) {
    this.disableUnboundCollisions = enabled;
    return this;
  }

  /**
   * When combined with {@link #keepOnlyJoints(Collection)}, merges non-excluded, non-selected URDF
   * links directly into the chassis {@code root.glb} at their cumulative URDF forward-kinematics
   * resting pose instead of hiding them—preserving 100% visual fidelity while keeping {@code
   * ComponentPoses} lean.
   *
   * @param enabled {@code true} to bake non-selected structural links into {@code root.glb}.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions bakeUnboundLinksIntoChassis(boolean enabled) {
    this.bakeUnboundLinksIntoChassis = enabled;
    return this;
  }

  /**
   * Adds one or more case-insensitive substring patterns to exclude from URDF link/mesh loading
   * (for example {@code "MAXSwerve"} when overlaying animated swerve modules).
   *
   * @param patterns Case-insensitive substring patterns to skip.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions excludeMeshPatterns(String... patterns) {
    Objects.requireNonNull(patterns, "patterns");
    for (String p : patterns) {
      if (p != null && !p.isBlank()) {
        this.excludedMeshPatterns.add(p.trim().toLowerCase(Locale.ROOT));
      }
    }
    return this;
  }

  /**
   * Adds one or more case-insensitive substring patterns whose matching GLB nodes are tagged with
   * {@code NOSIMPLIFY} during merging so AdvantageScope preserves their full geometry without
   * disabling simplification globally.
   *
   * @param patterns Case-insensitive substring patterns to tag with {@code NOSIMPLIFY}.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions noSimplifyMeshPatterns(String... patterns) {
    Objects.requireNonNull(patterns, "patterns");
    for (String p : patterns) {
      if (p != null && !p.isBlank()) {
        this.noSimplifyMeshPatterns.add(p.trim().toLowerCase(Locale.ROOT));
      }
    }
    return this;
  }

  /**
   * Maps a raw CAD joint name (for example {@code "revolute_1"}) to a clean Java mechanism name
   * (for example {@code "Turret"}) so code can bind by semantic name without renaming Onshape
   * mates.
   *
   * @param cadJointName Original joint name in the URDF.
   * @param javaMechanismName Target semantic name to expose in {@link CadRobotManifest}.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions withJointAlias(String cadJointName, String javaMechanismName) {
    Objects.requireNonNull(cadJointName, "cadJointName");
    Objects.requireNonNull(javaMechanismName, "javaMechanismName");
    if (!cadJointName.isBlank() && !javaMechanismName.isBlank()) {
      this.jointAliases.put(CadRobotManifest.normalizeName(cadJointName), javaMechanismName.trim());
    }
    return this;
  }

  /**
   * Registers multiple CAD-to-Java joint name aliases.
   *
   * @param aliases Map of {@code cadJointName -> javaMechanismName}.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions withJointAliases(Map<String, String> aliases) {
    Objects.requireNonNull(aliases, "aliases");
    for (Map.Entry<String, String> e : aliases.entrySet()) {
      withJointAlias(e.getKey(), e.getValue());
    }
    return this;
  }

  /**
   * Restricts exported articulated links to only those driven by the named joints (matching either
   * original CAD names or aliased Java names, case/separator-insensitively). Leave empty to keep
   * all non-filtered visual links.
   *
   * @param jointNames Allowed joint names.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions keepOnlyJoints(Collection<String> jointNames) {
    Objects.requireNonNull(jointNames, "jointNames");
    for (String name : jointNames) {
      if (name != null && !name.isBlank()) {
        this.keepOnlyJoints.add(CadRobotManifest.normalizeName(name));
      }
    }
    return this;
  }

  /**
   * Re-parents non-selected URDF links matching any of {@code linkOrMeshPatterns} (case-insensitive
   * substring match against link name or mesh filename) onto the specified moving joint's child
   * link, baking them into that joint's {@code .glb} in its local frame so mis-mated CAD subparts
   * (such as launcher side plates accidentally mated to {@code root}) rotate/move rigidly with the
   * mechanism without editing CAD.
   *
   * @param jointName Target moving joint name (raw CAD name or configured alias).
   * @param linkOrMeshPatterns Case-insensitive substring patterns identifying links/meshes to
   *     attach to {@code jointName}.
   * @return This options instance for chaining.
   */
  public CadUrdfOptions attachLinksToJoint(String jointName, String... linkOrMeshPatterns) {
    Objects.requireNonNull(jointName, "jointName");
    Objects.requireNonNull(linkOrMeshPatterns, "linkOrMeshPatterns");
    if (jointName.isBlank()) {
      return this;
    }
    String key = CadRobotManifest.normalizeName(jointName);
    List<String> list = jointLinkAttachments.computeIfAbsent(key, k -> new ArrayList<>());
    for (String p : linkOrMeshPatterns) {
      if (p != null && !p.isBlank()) {
        list.add(p.trim().toLowerCase(Locale.ROOT));
      }
    }
    return this;
  }

  /** @return {@code true} if fastener/hardware filtering is enabled. */
  public boolean isExcludeFasteners() {
    return excludeFasteners;
  }

  /** @return {@code true} if unbound URDF links start with collision checking disabled. */
  public boolean isDisableUnboundCollisions() {
    return disableUnboundCollisions;
  }

  /** @return {@code true} if non-selected structural links are baked into {@code root.glb}. */
  public boolean isBakeUnboundLinksIntoChassis() {
    return bakeUnboundLinksIntoChassis;
  }

  /** @return Unmodifiable list of custom lowercase mesh exclusion patterns. */
  public List<String> getExcludedMeshPatterns() {
    return Collections.unmodifiableList(excludedMeshPatterns);
  }

  /** @return Unmodifiable list of lowercase mesh patterns tagged with {@code NOSIMPLIFY}. */
  public List<String> getNoSimplifyMeshPatterns() {
    return Collections.unmodifiableList(noSimplifyMeshPatterns);
  }

  /** @return Unmodifiable map of normalized CAD joint names to Java mechanism names. */
  public Map<String, String> getJointAliases() {
    return Collections.unmodifiableMap(jointAliases);
  }

  /** @return Unmodifiable set of normalized joint names in the allowlist (empty = keep all). */
  public Set<String> getKeepOnlyJoints() {
    return Collections.unmodifiableSet(keepOnlyJoints);
  }

  /**
   * @return Unmodifiable map of normalized target joint names to lowercase link/mesh attachment
   *     patterns configured via {@link #attachLinksToJoint(String, String...)}.
   */
  public Map<String, List<String>> getJointLinkAttachments() {
    return Collections.unmodifiableMap(jointLinkAttachments);
  }

  /**
   * Checks whether a link name or mesh path matches any {@link #attachLinksToJoint} rule and
   * returns the normalized target joint name if matched.
   *
   * @param linkName URDF link name.
   * @param meshFile Optional mesh filename/path (may be {@code null}).
   * @return Normalized target joint name, or empty if no explicit attachment rule matches.
   */
  public Optional<String> findAttachedTargetJoint(String linkName, String meshFile) {
    if (jointLinkAttachments.isEmpty()) {
      return Optional.empty();
    }
    String lowerLink = linkName == null ? "" : linkName.toLowerCase(Locale.ROOT);
    String lowerMesh = meshFile == null ? "" : meshFile.toLowerCase(Locale.ROOT);
    for (Map.Entry<String, List<String>> entry : jointLinkAttachments.entrySet()) {
      for (String pat : entry.getValue()) {
        if ((!lowerLink.isEmpty() && lowerLink.contains(pat))
            || (!lowerMesh.isEmpty() && lowerMesh.contains(pat))) {
          return Optional.of(entry.getKey());
        }
      }
    }
    return Optional.empty();
  }

  /**
   * Resolves any alias configured for {@code rawCadJointName}, returning the aliased Java name if
   * registered or {@code rawCadJointName} unchanged otherwise.
   *
   * @param rawCadJointName Raw joint name from the URDF.
   * @return Aliased or original joint name.
   */
  public String resolveJointAlias(String rawCadJointName) {
    if (rawCadJointName == null || jointAliases.isEmpty()) {
      return rawCadJointName;
    }
    String mapped = jointAliases.get(CadRobotManifest.normalizeName(rawCadJointName));
    return mapped != null ? mapped : rawCadJointName;
  }

  /**
   * Checks whether a mesh path or link name should be excluded by {@link #isExcludeFasteners()} or
   * {@link #getExcludedMeshPatterns()}.
   *
   * @param nameOrPath Link name or mesh filename/path.
   * @return {@code true} if the mesh/link should be skipped.
   */
  public boolean shouldExcludeMesh(String nameOrPath) {
    if (nameOrPath == null || nameOrPath.isBlank()) {
      return false;
    }
    String lower = nameOrPath.toLowerCase(Locale.ROOT);
    for (String pat : excludedMeshPatterns) {
      if (lower.contains(pat)) {
        return true;
      }
    }
    if (excludeFasteners) {
      for (String token : DEFAULT_FASTENER_TOKENS) {
        if (lower.contains(token)) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Checks whether a mesh path or link name matches any configured {@link
   * #noSimplifyMeshPatterns(String...)} entry.
   *
   * @param nameOrPath Link name or mesh filename/path.
   * @return {@code true} if the merged glTF node should be tagged with {@code NOSIMPLIFY}.
   */
  public boolean shouldTagNoSimplify(String nameOrPath) {
    if (nameOrPath == null || nameOrPath.isBlank() || noSimplifyMeshPatterns.isEmpty()) {
      return false;
    }
    String lower = nameOrPath.toLowerCase(Locale.ROOT);
    for (String pat : noSimplifyMeshPatterns) {
      if (lower.contains(pat)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Checks whether a joint (given its raw and aliased names) passes the optional {@link
   * #getKeepOnlyJoints()} filter.
   *
   * @param rawJointName Original URDF joint name.
   * @param aliasedJointName Aliased joint name.
   * @return {@code true} if the joint should be retained.
   */
  public boolean isJointAllowed(String rawJointName, String aliasedJointName) {
    if (keepOnlyJoints.isEmpty()) {
      return true;
    }
    return keepOnlyJoints.contains(CadRobotManifest.normalizeName(rawJointName))
        || keepOnlyJoints.contains(CadRobotManifest.normalizeName(aliasedJointName));
  }
}
