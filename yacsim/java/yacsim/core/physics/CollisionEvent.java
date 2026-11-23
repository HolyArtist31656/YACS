// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.physics;

/**
 * Immutable record describing a single detected OBB overlap or extension-envelope breach.
 *
 * @param componentA First component name.
 * @param componentB Second component name (or {@code "Chassis"} / {@code "ExtensionEnvelope"}).
 * @param penetrationDepthMeters Minimum penetration or violation depth in meters.
 */
public record CollisionEvent(String componentA, String componentB, double penetrationDepthMeters) {
  /**
   * Checks whether either side of this collision matches {@code name}.
   *
   * @param name Component or target name to test.
   * @return {@code true} if {@code componentA} or {@code componentB} equals {@code name}.
   */
  public boolean involves(String name) { return componentA.equals(name) || componentB.equals(name); }
}
