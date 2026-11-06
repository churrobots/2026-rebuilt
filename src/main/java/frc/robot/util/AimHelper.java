package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import java.util.Map;

/**
 * The math for aiming at the hub. These are plain functions: give them where the robot is and
 * which alliance we're on, and they give back an answer. They don't touch any motors, so they're
 * easy to test (see AimHelperTest).
 */
public final class AimHelper {
  private AimHelper() {}

  /**
   * Distance to the hub (inches) → flywheel speed (RPM). Between two rows, the table fills in
   * the gap with a straight line. Past either end, it uses the end value.
   *
   * <p>Measured by the students on 2026-11-05 with the Calibration card (branch
   * madtown-shooter-calibration): the robot's own distance, and the middle RPM that scored.
   * Re-measure after new shooter wheels, worn balls, or a changed shooter angle.
   */
  private static final InterpolatingDoubleTreeMap distanceInchesToRpm =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(79.0, 3025.0),
          Map.entry(119.0, 3200.0),
          Map.entry(144.0, 3525.0),
          Map.entry(199.0, 4050.0));

  /** The flywheel speed (RPM) for a shot from this far away (meters), from the table. */
  public static double shooterRpmForDistance(double distanceMeters) {
    return distanceInchesToRpm.get(Units.metersToInches(distanceMeters));
  }

  /** The flywheel speed (RPM) for a shot at our hub from where the robot is now. */
  public static double shooterRpmForHub(Pose2d robotPose, Alliance alliance) {
    return shooterRpmForDistance(distanceToHub(robotPose, alliance));
  }

  /**
   * Distance to our alliance wall (inches) → flywheel speed (RPM) for a PASS.
   * TODO: 2026 robot's numbers (SemiAutoHelper on main). Re-measure for our shooter.
   */
  private static final InterpolatingDoubleTreeMap passDistanceInchesToRpm =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(200.0, 3400.0), Map.entry(323.0, 4200.0), Map.entry(467.0, 5000.0));

  /** SHOOT at our hub (in our alliance zone), or PASS back toward our side (anywhere else). */
  public enum AimMode {
    SHOOT,
    PASS
  }

  /** In our alliance zone we can score, so SHOOT. Out in the neutral zone (or past it), PASS. */
  public static AimMode aimMode(Pose2d robotPose, Alliance alliance) {
    double x = robotPose.getX();
    boolean inOurZone =
        alliance == Alliance.Red
            ? x > FieldConstants.RED_ZONE_LINE_X
            : x < FieldConstants.BLUE_ZONE_LINE_X;
    return inOurZone ? AimMode.SHOOT : AimMode.PASS;
  }

  /** How far the robot is from our alliance wall, in meters (straight back, along x). */
  public static double distanceToOurWall(Pose2d robotPose, Alliance alliance) {
    return alliance == Alliance.Red
        ? FieldConstants.FIELD_LENGTH - robotPose.getX()
        : robotPose.getX();
  }

  /**
   * The heading for a pass: the shooter (on the back) points straight at our wall. Blue's wall
   * is toward −x (180°), so the front faces 0°. Red is the opposite.
   */
  public static Rotation2d passHeading(Alliance alliance) {
    return alliance == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero;
  }

  /** Where the robot should face right now: at the hub to SHOOT, or at our wall to PASS. */
  public static Rotation2d aimHeading(Pose2d robotPose, Alliance alliance) {
    return aimMode(robotPose, alliance) == AimMode.SHOOT
        ? shootHeading(robotPose, alliance)
        : passHeading(alliance);
  }

  /** How far away the current target is (meters): the hub to SHOOT, our wall to PASS. */
  public static double aimDistance(Pose2d robotPose, Alliance alliance) {
    return aimMode(robotPose, alliance) == AimMode.SHOOT
        ? distanceToHub(robotPose, alliance)
        : distanceToOurWall(robotPose, alliance);
  }

  /**
   * Our heading for a preset shot. The 2026 robot's shooter faced FORWARD and ours faces the
   * BACK, so we take 2026's heading for that spot and turn it 180°.
   */
  public static Rotation2d presetHeading(double heading2026BlueDeg, double heading2026RedDeg, Alliance alliance) {
    double deg = alliance == Alliance.Red ? heading2026RedDeg : heading2026BlueDeg;
    return Rotation2d.fromDegrees(deg).plus(Rotation2d.k180deg);
  }

  /** The diagonal headings that drive straight over the bump (same as 2026). */
  private static final double[] bumpHeadingsDeg = {45.0, 135.0, -45.0, -135.0};

  /** The bump heading closest to where the robot faces now, so it turns the least. */
  public static Rotation2d bumpHeading(Rotation2d currentHeading) {
    Rotation2d best = Rotation2d.fromDegrees(bumpHeadingsDeg[0]);
    for (double deg : bumpHeadingsDeg) {
      Rotation2d candidate = Rotation2d.fromDegrees(deg);
      if (Math.abs(candidate.minus(currentHeading).getRadians())
          < Math.abs(best.minus(currentHeading).getRadians())) {
        best = candidate;
      }
    }
    return best;
  }

  /** The flywheel speed (RPM) for right now: the shot table or the pass table. */
  public static double aimRpm(Pose2d robotPose, Alliance alliance) {
    return aimMode(robotPose, alliance) == AimMode.SHOOT
        ? shooterRpmForHub(robotPose, alliance)
        : passDistanceInchesToRpm.get(Units.metersToInches(distanceToOurWall(robotPose, alliance)));
  }

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
