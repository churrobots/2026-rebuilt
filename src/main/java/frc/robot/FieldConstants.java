package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Where things are on the field, in meters. The origin (0, 0) is always the corner at the blue
 * wall, for both alliances, so the red side is a mirror of the blue side.
 */
public final class FieldConstants {
  // Hub centers from the game manual (FuelPhysicsSim on main).
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.5974, 4.035);
  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.938, 4.035);

  private FieldConstants() {}
}
