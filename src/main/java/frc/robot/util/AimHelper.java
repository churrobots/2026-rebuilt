package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

/**
 * The math for aiming at the hub. These are plain functions: give them where the robot is and
 * which alliance we're on, and they give back an answer. They don't touch any motors, so they're
 * easy to test (see AimHelperTest).
 */
public final class AimHelper {
  private AimHelper() {}

  /** Where our hub is. The field's origin is always at the blue wall, so red's hub is far away. */
  public static Translation2d hubPosition(Alliance alliance) {
    return alliance == Alliance.Red ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;
  }

  /** How far the robot's center is from our hub, in meters: √(dx² + dy²). */
  public static double distanceToHub(Pose2d robotPose, Alliance alliance) {
    return robotPose.getTranslation().getDistance(hubPosition(alliance));
  }

  /** Which way the arrow from the robot to our hub points: atan2(dy, dx). */
  public static Rotation2d angleToHub(Pose2d robotPose, Alliance alliance) {
    Translation2d robotToHub = hubPosition(alliance).minus(robotPose.getTranslation());
    return robotToHub.getAngle();
  }

  /**
   * The heading the robot should turn to so it can shoot. The shooter faces out the back, so the
   * robot's front has to point the opposite way from the hub: angle to hub + 180°. Rotation2d
   * wraps the answer to ±180° for us.
   *
   * <p>TODO: measure whether the new shooter is centered. The 2026 robot's shooter was 9" off
   * center and needed an offset here.
   */
  public static Rotation2d shootHeading(Pose2d robotPose, Alliance alliance) {
    return angleToHub(robotPose, alliance).plus(Rotation2d.k180deg);
  }
}
