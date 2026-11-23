// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package frc.robot;

import frc.robot.mechanisms.SuperstructureMechanism;
import java.io.IOException;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.Notifier;

public class Robot extends TimedRobot {
  private final Scheduler scheduler = Scheduler.getDefault();
  private final SuperstructureMechanism superstructure;
  private Notifier simNotifier;

  public Robot() {
    try {
      superstructure = new SuperstructureMechanism();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void robotPeriodic() {
    superstructure.periodic();
    scheduler.run();
  }

  @Override
  public void simulationInit() {
    simNotifier = new Notifier(superstructure::simulationPeriodic);
    simNotifier.startPeriodic(0.02);
  }
}
