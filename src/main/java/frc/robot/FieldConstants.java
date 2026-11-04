package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Where things are on the field, in meters. The origin (0, 0) is always the corner at the blue
 * wall, for both alliances, so the red side is a mirror of the blue side.
 */
public final class FieldConstants {
  // Hub centers from the game manual (FuelPhysicsSim on main).
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.5974, 4.035);
  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.938, 4.035);

  // Field length from the game manual (FuelPhysicsSim on main).
  public static final double FIELD_LENGTH = 16.541;

  // Alliance zone lines, measured from the blue wall (the 2026 robot used 170" and 457").
  // Blue's zone is everything closer to the blue wall than BLUE_ZONE_LINE_X; red's is
  // everything past RED_ZONE_LINE_X. The neutral zone is in between.
  public static final double BLUE_ZONE_LINE_X = Units.inchesToMeters(170);
  public static final double RED_ZONE_LINE_X = Units.inchesToMeters(457);

  private FieldConstants() {}
}
