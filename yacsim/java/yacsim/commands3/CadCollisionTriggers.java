// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.commands3;

import org.wpilib.command3.Trigger;
import yacsim.core.model.CadRobotSimulator;

/**
 * WPILib Commands v3 {@link Trigger} factories for simulated CAD collisions and FRC extension
 * envelope violations.
 */
public final class CadCollisionTriggers {
  private CadCollisionTriggers() {}

  /**
   * Creates a trigger that is active whenever any OBB collision or envelope violation is detected.
   *
   * @param simulator The {@link CadRobotSimulator} to observe.
   * @return A Commands v3 {@link Trigger} reflecting {@link CadRobotSimulator#hasAnyCollision()}.
   */
  public static Trigger onAnyCollision(CadRobotSimulator simulator) {
    return new Trigger(simulator::hasAnyCollision);
  }

  /**
   * Creates a trigger that is active whenever the two named components collide (order-independent).
   *
   * @param simulator The {@link CadRobotSimulator} to observe.
   * @param componentA First component name (or {@code "Chassis"} / {@code "ExtensionEnvelope"}).
   * @param componentB Second component name (or {@code "Chassis"} / {@code "ExtensionEnvelope"}).
   * @return A Commands v3 {@link Trigger} reflecting {@link CadRobotSimulator#hasCollisionBetween}.
   */
  public static Trigger onCollisionBetween(
      CadRobotSimulator simulator, String componentA, String componentB) {
    return new Trigger(() -> simulator.hasCollisionBetween(componentA, componentB));
  }

  /**
   * Creates a trigger that is active whenever any component breaches the configured extension
   * envelope.
   *
   * @param simulator The {@link CadRobotSimulator} to observe.
   * @return A Commands v3 {@link Trigger} reflecting {@link CadRobotSimulator#hasExtensionViolation()}.
   */
  public static Trigger onExtensionViolation(CadRobotSimulator simulator) {
    return new Trigger(simulator::hasExtensionViolation);
  }
}
