package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  // One Kraken X60 on a TalonFX, 1:1 (checked 2026-11-03)
  public static final DCMotor shooterGearbox = DCMotor.getKrakenX60(1);
  public static final double shooterMotorReduction = 1.0;
  public static final int shooterCanId = 16;

  // TODO: verify on the robot (values from the 2026 robot on main)
  public static final boolean shooterInverted = true;
  public static final int shooterCurrentLimit = 60; // stator amps

  // Flywheel size, only used by the sim. TODO: measure (2026: 4 in wheel, 1 lb).
  private static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  private static final double wheelMassKg = Units.lbsToKilograms(1.0);
  public static final double flywheelMoiKgMetersSq =
      0.5 * wheelMassKg * wheelRadiusMeters * wheelRadiusMeters; // a solid disc

  // Speeds. TODO: tune (2026 robot used these)
  public static final double shotRpm = 4550.0;
  public static final double idleRpm = 400.0; // keep it rolling so it spins up faster
  public static final double readyToleranceRpm = 100.0; // "close enough" to shoot

  // Where the ball leaves the shooter, measured from the robot's center, in the robot's own
  // directions: x = forward (+) / backward (-), y = left (+) / right (-). Students measured it
  // from the spindexer on 2026-11-07: 6" backward and 5.5" left.
  // TODO: verify the spindexer is centered on the robot (we assumed it is).
  public static final Translation2d shooterExitOffset =
      new Translation2d(Units.inchesToMeters(-6.0), Units.inchesToMeters(5.5));

  // Shot trim: added to every RPM from the table. Negative = slower = shorter shots.
  // Auto-aim was landing about a foot too far (students, 2026-11-07); -150 RPM is roughly a
  // foot at mid range. This is the starting value. The Shooter card's trim buttons change it
  // live, but that resets when the robot restarts: once a number works, copy it here.
  // TODO: tune on the field. If shots still go long, make it more negative; short, less.
  public static final double shotRpmTrim = -50.0;

  // Gains, in TalonFX units: volts per (rotation per second). The Kraken runs these
  // itself, 1000 times a second. Starting values from the 2026 robot. TODO: tune.
  public static final double kS = 0.0; // volts to get it moving
  public static final double kV = 0.112; // volts per rot/s: does most of the work
  public static final double kP = 0.3; // volts per rot/s of error: recovers after a shot
}
