// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.manifest;

import java.nio.file.Path;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.physics.CadMassProperties;

/**
 * CAD-extracted rigid body (link) definition inside a {@link CadRobotManifest}, holding visual mesh
 * path, AdvantageScope zeroing transforms, collision flags, and optional exact CAD mass/inertia
 * properties.
 *
 * @param name Unique link name (for example {@code "Chassis"}, {@code "Hood"}, or {@code
 *     "TrashCompactor"}).
 * @param meshFile Mesh path relative to the manifest directory (for example {@code "model_0.glb"}),
 *     or {@code null} for virtual/physics-only links without a visual mesh.
 * @param unitScaleToMeters Multiplicative scale factor converting mesh coordinates to meters
 *     (default {@code 1.0}).
 * @param zeroedPositionMeters AdvantageScope zeroed position offset in meters.
 * @param zeroedRotations Ordered AdvantageScope axis rotations applied to zero the mesh.
 * @param massProperties Optional exact CAD mass, center of mass, and principal inertia tensor.
 * @param collisionEnabled Whether OBB collision checks are enabled for this link.
 */
public record CadLinkManifest(
    String name,
    String meshFile,
    double unitScaleToMeters,
    Translation3d zeroedPositionMeters,
    List<CadAssetConfig.AxisRotation> zeroedRotations,
    Optional<CadMassProperties> massProperties,
    boolean collisionEnabled) {

  /** Compact constructor validating link manifest invariants. */
  public CadLinkManifest {
    Objects.requireNonNull(name, "name");
    if (name.isBlank()) {
      throw new IllegalArgumentException("Link name cannot be blank.");
    }
    if (unitScaleToMeters <= 0.0) {
      throw new IllegalArgumentException("unitScaleToMeters must be positive.");
    }
    zeroedPositionMeters =
        zeroedPositionMeters == null ? new Translation3d() : zeroedPositionMeters;
    zeroedRotations = zeroedRotations == null ? List.of() : List.copyOf(zeroedRotations);
    massProperties = massProperties == null ? Optional.empty() : massProperties;
  }

  /**
   * Returns whether this link has a visual CAD mesh file.
   *
   * @return {@code true} if {@link #meshFile()} is non-null and non-blank.
   */
  public boolean hasMesh() {
    return meshFile != null && !meshFile.isBlank();
  }

  /**
   * Converts this link manifest into a {@link CadAssetConfig} resolved against {@code baseDir},
   * preserving unit scale, AdvantageScope zeroed transforms, and exact CAD mass properties.
   *
   * @param baseDir Directory containing the manifest or URDF file.
   * @return Configured {@link CadAssetConfig}.
   * @throws IllegalStateException If this link does not define a {@code meshFile}.
   */
  public CadAssetConfig toCadAssetConfig(Path baseDir) {
    Objects.requireNonNull(baseDir, "baseDir");
    if (!hasMesh()) {
      throw new IllegalStateException("Link '" + name + "' does not define a meshFile.");
    }
    Path resolved = baseDir.resolve(meshFile).normalize();
    CadAssetConfig asset =
        new CadAssetConfig(resolved)
            .withUnitScaleToMeters(unitScaleToMeters)
            .withZeroedPosition(zeroedPositionMeters);
    for (CadAssetConfig.AxisRotation rot : zeroedRotations) {
      asset.addZeroedRotation(rot.axis(), rot.degrees());
    }
    massProperties.ifPresent(asset::withMassPropertiesOverride);
    return asset;
  }
}
