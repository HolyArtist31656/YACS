// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.commands3;

import java.io.IOException;
import org.wpilib.command3.Mechanism;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Notifier;
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
  private Notifier simNotifier = null;

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


  /**
   * Starts a managed background {@link Notifier} calling {@link #simulationPeriodic()} every {@code
   * 0.02} seconds (50 Hz) when running in desktop simulation. Safe no-op on real robot hardware;
   * automatically stopped and closed by {@link #close()}.
   *
   * @return This mechanism wrapper for chaining.
   */
  public CadRobotMechanism startSimNotifier() {
    return startSimNotifier(0.02);
  }

  /**
   * Starts a managed background {@link Notifier} calling {@link #simulationPeriodic()} at the given
   * period when running in desktop simulation. Safe no-op on real robot hardware; automatically
   * stopped and closed by {@link #close()}.
   *
   * @param periodSeconds Positive simulation period in seconds (for example {@code 0.02}).
   * @return This mechanism wrapper for chaining.
   */
  public CadRobotMechanism startSimNotifier(double periodSeconds) {
    if (periodSeconds <= 0.0) {
      throw new IllegalArgumentException("periodSeconds must be positive.");
    }
    if (RobotBase.isSimulation() && simNotifier == null) {
      simNotifier = new Notifier(this::simulationPeriodic);
      simNotifier.setName("YACSim-SimNotifier");
      simNotifier.startPeriodic(periodSeconds);
    }
    return this;
  }

  /** Stops any managed simulation {@link Notifier} and closes NetworkTables publishers. */
  @Override
  public void close() {
    if (simNotifier != null) {
      simNotifier.stop();
      simNotifier.close();
      simNotifier = null;
    }
    simulator.close();
  }
}
