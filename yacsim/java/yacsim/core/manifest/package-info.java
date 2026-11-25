// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

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
