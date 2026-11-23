// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Adapted from FRC Team 6328 Mechanical Advantage "Darwin" (MIT License)
// https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot;

import org.wpilib.framework.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
