package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;

public class FeederConstants {
  // One Kraken X60 on a TalonFX, 1:1 (checked 2026-11-03)
  public static final DCMotor feederGearbox = DCMotor.getKrakenX60(1);
  public static final double feederMotorReduction = 1.0;
  public static final int feederCanId = 14;

  // TODO: verify on the robot (2026 feeder was not inverted)
  public static final boolean feederInverted = false;
  public static final int feederCurrentLimit = 60; // stator amps

  // A light roller, only used by the sim.
  public static final double feederMoiKgMetersSq = 0.0005;

  // Feed speed. TODO: tune. (2026 fed faster than the shooter, but a Kraken tops out
  // around 6000 RPM, so start here.)
  public static final double feedRpm = 5000.0;

  // Gains, in TalonFX units: volts per (rotation per second). From the 2026 feeder. TODO: tune.
  public static final double kS = 0.0;
  public static final double kV = 0.111;
  public static final double kP = 0.25;
}
