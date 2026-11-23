// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

/**
 * CAD-extracted robot manifest ({@code yacsim-robot.json}) and zero-dependency URDF importer.
 *
 * <p>Teams can either configure YACSim manually using {@link yacsim.core.model.CadRobotConfig} or
 * load all CAD-extractable geometry, exact mass/inertia tensors, joint origins, motion axes,
 * hard/soft limits, wheel radii, and cameras from a {@code yacsim-robot.json} or {@code .urdf}
 * file via {@link yacsim.core.manifest.CadManifestLoader} and {@link
 * yacsim.core.model.CadRobotConfig#fromManifest(java.nio.file.Path)}.
 */
package yacsim.core.manifest;
