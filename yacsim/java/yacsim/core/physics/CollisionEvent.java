// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.physics;

public record CollisionEvent(String componentA, String componentB, double penetrationDepthMeters) {
  public boolean involves(String name) { return componentA.equals(name) || componentB.equals(name); }
}
