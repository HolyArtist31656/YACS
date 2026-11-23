// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.commands3;

import java.io.IOException;
import org.wpilib.command3.Mechanism;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.model.CadRobotSimulator;

/**
 * WPILib Commands v3 wrapper around {@link CadRobotSimulator}, following the YAMS 2027 Commands v3
 * lifecycle where {@code Robot.robotPeriodic()} calls {@link #periodic()} and {@code
 * Robot.simulationInit()} schedules {@link #simulationPeriodic()}.
 */
public class CadRobotMechanism implements AutoCloseable {
  private final Mechanism mechanism;
  private final CadRobotSimulator simulator;

  /**
   * Constructs a Commands v3 CAD robot mechanism and initializes a new {@link CadRobotSimulator}
   * with NetworkTables publishing enabled.
   *
   * @param mechanism The owning WPILib Commands v3 {@link Mechanism}.
   * @param config The robot CAD and collision configuration.
   * @throws IOException If any configured CAD mesh file cannot be read or exported.
   */
  public CadRobotMechanism(Mechanism mechanism, CadRobotConfig config) throws IOException {
    this(mechanism, new CadRobotSimulator(config));
  }

  /**
   * Constructs a Commands v3 CAD robot mechanism wrapping an existing {@link CadRobotSimulator}.
   *
   * @param mechanism The owning WPILib Commands v3 {@link Mechanism}.
   * @param simulator Pre-configured simulator instance.
   */
  public CadRobotMechanism(Mechanism mechanism, CadRobotSimulator simulator) {
    this.mechanism = mechanism;
    this.simulator = simulator;
  }

  /** Publishes the latest simulation snapshot to NetworkTables. Call from {@code robotPeriodic()}. */
  public void periodic() { simulator.updateTelemetry(); }
  /** Steps pose suppliers and OBB collision checks. Safe to call from {@code simulationPeriodic()} or a {@code Notifier}. */
  public void simulationPeriodic() { simulator.simIterate(); }
  /**
   * Returns the owning WPILib Commands v3 mechanism.
   *
   * @return The owning {@link Mechanism}.
   */
  public Mechanism getMechanism() { return mechanism; }
  /**
   * Returns the underlying CAD simulator.
   *
   * @return The {@link CadRobotSimulator} instance.
   */
  public CadRobotSimulator getSimulator() { return simulator; }

  /** Closes underlying NetworkTables publishers. */
  @Override
  public void close() { simulator.close(); }
}
