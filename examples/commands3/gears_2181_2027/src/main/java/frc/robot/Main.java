// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Port of FRCteam2181/2026-Comp-Code to WPILib 2027 Commands v3 + YAMS 2027 + YACSim

package frc.robot;

import org.wpilib.framework.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
