package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Intake arm numbers. All angles are in RADIANS, measured the way our encoder reads them:
 * 0 = stowed (tucked in), and the angle gets BIGGER as the arm swings DOWN toward the floor.
 */
public class IntakeArmConstants {
  // NEO on a SPARK MAX (checked 2026-11-03). Same gearing as 2026: a 25:1 gearbox
  // then 80:24 pulleys. Written with ".0" so Java doesn't chop 80/24 down to 3.
  public static final DCMotor armGearbox = DCMotor.getNEO(1);
  public static final double armMotorReduction = 25.0 * 80.0 / 24.0; // about 83.3
  public static final int armCanId = 12;

  // The motor must be set up so POSITIVE volts make the angle BIGGER (arm goes down).
  // Flipped 2026-11-04: on the robot, +1 V lifted the arm and -1 V lowered it (backwards).
  public static final boolean armInverted = true;
  // Flipped 2026-11-04: turning the arm by hand read backwards (got smaller going down).
  public static final boolean armEncoderInverted = true;
  public static final int armCurrentLimit = 40; // amps
  // Absolute encoder reading (radians) when the arm is stowed, so stowed reads 0.
  public static final double armEncoderOffsetRad = 0.0; // TODO: measure

  // Positions (students, 2026-11-03).
  public static final double stowedRad = 0.0; // tucked in
  public static final double retractedRad = 1.538; // resting spot
  public static final double extendedRad = 2.376; // down on the floor

  // Our angle when the arm is exactly level (flat, pointing straight out). Gravity pulls
  // hardest here, and the gravity math needs it. Level = extended (students, 2026-11-03).
  public static final double armLevelRad = extendedRad;

  // Soft limits (targets get clamped here) sit inside the hard stops (sim only).
  public static final double softMinRad = -0.05;
  public static final double softMaxRad = 2.45;
  public static final double hardMinRad = -0.1;
  public static final double hardMaxRad = 2.5;

  // Arm size, only used by the sim. TODO: measure the real arm (2026 sim used 1 ft, 15 lb).
  public static final double armLengthMeters = Units.feetToMeters(1.0);
  public static final double armMassKg = Units.lbsToKilograms(15.0);

  // Gains: all 0 on purpose! Tuning lesson: kG first (stop the droop), then kP, then kD.
  public static final double kS = 0.0; // volts to get it moving
  public static final double kG = 0.5; // volts to hold it level against gravity
  public static final double kV = 0.0; // volts per rad/sec
  public static final double kP = 6.0; // volts per radian of error
  public static final double kD = 0.0; // volts per rad/sec of error change

  public static final double atTargetToleranceRad = 0.05; // about 3 degrees

  // Safety (coach, 2026-11-04). The first real run slammed the arm into the robot, so:
  // never send more than maxVolts, and send 0 V if the reading is outside safeMin..safeMax
  // (bad offset, wrong direction, or the encoder wrapped around).
  // Raise maxVolts slowly, watching for overshoot. 3 V and 6 V both stopped cleanly.
  public static final double maxVolts = 6.0;
  public static final double safeMinRad = hardMinRad - 0.2;
  public static final double safeMaxRad = hardMaxRad + 0.2;

  // "Pulse" while shooting (like 2026): the arm swings back and forth between almost-extended
  // and almost-stowed, which pushes the balls left in the intake up into the spindexer.
  // These match 2026's sweep (19% to 91% of the way from extended to stowed). TODO: tune.
  public static final double pulseOutRad = 1.93; // the "out" end, near extended
  public static final double pulseInRad = 0.22; // the "in" end, near stowed
  public static final double pulsePeriodSeconds = 1.0; // one full out-and-back
  // Hold still for a moment at each end, so the gears don't slam back and forth.
  public static final double pulseFlatPercent = 0.1;
}
