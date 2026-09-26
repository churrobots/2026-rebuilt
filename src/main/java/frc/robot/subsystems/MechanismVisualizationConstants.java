package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

/** Robot-relative locations used by AdvantageScope mechanism component poses. */
public final class MechanismVisualizationConstants {
  private MechanismVisualizationConstants() {}

  // Coordinate system: +X forward, +Y left, +Z up. The origin is the center of
  // the 28x28-inch base at floor level.
  private static final double HALF_BASE_METERS = Units.inchesToMeters(14.0);

  // Intake is centered at the front frame/bumper line. The 8-inch height is an
  // initial estimate and should be replaced with the measured pivot height.
  public static final double INTAKE_X_METERS = HALF_BASE_METERS;
  public static final double INTAKE_Y_METERS = 0.0;
  public static final double INTAKE_Z_METERS = Units.inchesToMeters(8.0);
  public static final double INTAKE_ARM_LENGTH_METERS = Units.inchesToMeters(12.0);

  // Spindexer is horizontal at robot center. The 8-inch height is an estimate.
  public static final double SPINDEXER_X_METERS = 0.0;
  public static final double SPINDEXER_Y_METERS = 0.0;
  public static final double SPINDEXER_Z_METERS = Units.inchesToMeters(8.0);

  public static final double FEEDER_X_METERS = 0.0;
  public static final double FEEDER_Y_METERS = HALF_BASE_METERS;
  public static final double FEEDER_Z_METERS = Units.inchesToMeters(15.0);

  public static final double SHOOTER_X_METERS = -HALF_BASE_METERS;
  public static final double SHOOTER_Y_METERS = HALF_BASE_METERS;
  public static final double SHOOTER_Z_METERS = Units.inchesToMeters(20.0);

  public static final double SPIN_VISUALIZATION_REDUCTION = 10.0;
}
