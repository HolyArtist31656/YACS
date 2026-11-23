// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.commands3;

import org.wpilib.command3.Trigger;
import yacsim.core.model.CadRobotSimulator;

/** Commands v3 {@link Trigger} factories for simulated CAD collisions and envelope checks. */
public final class CadCollisionTriggers {
  private CadCollisionTriggers() {}

  public static Trigger onAnyCollision(CadRobotSimulator simulator) {
    return new Trigger(simulator::hasAnyCollision);
  }

  public static Trigger onCollisionBetween(
      CadRobotSimulator simulator, String componentA, String componentB) {
    return new Trigger(() -> simulator.hasCollisionBetween(componentA, componentB));
  }

  public static Trigger onExtensionViolation(CadRobotSimulator simulator) {
    return new Trigger(simulator::hasExtensionViolation);
  }
}
