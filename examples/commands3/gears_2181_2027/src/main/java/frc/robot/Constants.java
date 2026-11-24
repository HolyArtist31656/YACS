// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Exact electrical CAN IDs & setpoints from https://github.com/FRCteam2181/2026-Comp-Code

package frc.robot;

import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.AngularVelocity;

/** Electrical CAN IDs and operator setpoints for FRC Team 2181 GEARS (2026 robot ported to 2027). */
public final class Constants {
  private Constants() {}

  public static final class CanIds {
    public static final int SHOOTER_LEADER = 9;
    public static final int SHOOTER_FOLLOWER = 10;
    public static final int SPINDEXER = 11;
    public static final int TOP_INTAKE = 12;
    public static final int INTAKE_ARM_FOLLOWER = 13;
    public static final int INTAKE_ARM_LEADER = 14;
    public static final int INPUT_FEEDER = 15;
    public static final int TURRET = 16;
    public static final int CLIMBER_LEFT = 18;
  }

  public static final class Setpoints {
    public static final AngularVelocity SHOOTER_LAUNCH_RPM = RPM.of(6000.0);
    public static final AngularVelocity SHOOTER_IDLE_RPM = RPM.of(1200.0);
    public static final AngularVelocity SPINDEXER_FEED_RPM = RPM.of(400.0);
    public static final AngularVelocity TOP_INTAKE_RPM = RPM.of(800.0);
    public static final AngularVelocity INPUT_FEED_RPM = RPM.of(2400.0);
    public static final double CLIMBER_DUTY_CYCLE = 0.7;
  }
}
