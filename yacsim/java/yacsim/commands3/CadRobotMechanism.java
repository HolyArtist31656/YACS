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

  public CadRobotMechanism(Mechanism mechanism, CadRobotConfig config) throws IOException {
    this(mechanism, new CadRobotSimulator(config));
  }

  public CadRobotMechanism(Mechanism mechanism, CadRobotSimulator simulator) {
    this.mechanism = mechanism;
    this.simulator = simulator;
  }

  public void periodic() { simulator.updateTelemetry(); }
  public void simulationPeriodic() { simulator.simIterate(); }
  public Mechanism getMechanism() { return mechanism; }
  public CadRobotSimulator getSimulator() { return simulator; }

  @Override
  public void close() { simulator.close(); }
}
