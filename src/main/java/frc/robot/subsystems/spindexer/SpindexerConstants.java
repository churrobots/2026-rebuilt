package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;

public class SpindexerConstants {
  // NEO on a SPARK MAX. Gearbox = three 5:1 stages + a 1:1 belt (checked
  // 2026-10-28)
  public static final DCMotor spindexerGearbox = DCMotor.getNEO(1);
  public static final double spindexerMotorReduction = 125.0; // 5 x 5 x 5

  public static final int spindexerCanId = 3; // orange sticky note

  // TODO: verify on the Throne (these are from the 2026 robot on main)
  public static final boolean spindexerInverted = true;
  public static final int spindexerCurrentLimit = 40; // amps

  // Open-loop voltages. TODO: tune on the Throne (watch the RPM on the dashboard)
  public static final double spindexerFeedVolts = 8.0;
  public static final double spindexerUnjamVolts = -4.0;
}

