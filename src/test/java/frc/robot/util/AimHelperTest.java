package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

/** Checks AimHelper's math against answers worked out on paper. */
class AimHelperTest {
  private static final double HUB_Y = 4.035;

  /** Checks two headings point the same way (so 180° and −180° count as equal). */
  private static void assertSameHeading(double expectedDegrees, Rotation2d actual) {
    double difference = actual.minus(Rotation2d.fromDegrees(expectedDegrees)).getDegrees();
    assertEquals(0.0, difference, 0.1, "expected " + expectedDegrees + "°, got " + actual.getDegrees() + "°");
  }

  @Test
  void hubStraightBehindNeedsNoTurn() {
    // Blue robot left of the hub, front already pointing at our wall (180°): its back faces the hub.
    Pose2d robot = new Pose2d(2.0, HUB_Y, Rotation2d.k180deg);
    Rotation2d heading = AimHelper.shootHeading(robot, Alliance.Blue);
    assertSameHeading(180, heading);
    assertSameHeading(0, heading.minus(robot.getRotation())); // no turn needed
  }

  @Test
  void hubStraightAheadNeedsHalfTurn() {
    // Same spot, but the front (intake) faces the hub, so the shooter faces away.
    Pose2d robot = new Pose2d(2.0, HUB_Y, Rotation2d.kZero);
    Rotation2d heading = AimHelper.shootHeading(robot, Alliance.Blue);
    assertSameHeading(180, heading.minus(robot.getRotation())); // turn all the way around
  }

  @Test
  void robotPastTheHub() {
    // The atan trap from step 2: robot at (7, 6), hub to the left and below.
    Pose2d robot = new Pose2d(7.0, 6.0, Rotation2d.kZero);
    assertSameHeading(-140.7, AimHelper.angleToHub(robot, Alliance.Blue));
    assertSameHeading(39.3, AimHelper.shootHeading(robot, Alliance.Blue));
    assertEquals(3.10, AimHelper.distanceToHub(robot, Alliance.Blue), 0.01);
  }

  @Test
  void redAllianceAimsAtTheRedHub() {
    // Red robot near the red wall, right of the red hub (x = 11.938).
    Pose2d robot = new Pose2d(14.0, HUB_Y, Rotation2d.kZero);
    assertEquals(2.062, AimHelper.distanceToHub(robot, Alliance.Red), 0.01);
    assertSameHeading(180, AimHelper.angleToHub(robot, Alliance.Red)); // hub is toward −x
    assertSameHeading(0, AimHelper.shootHeading(robot, Alliance.Red)); // back faces the hub
  }

  @Test
  void rpmTableUsesTheRowsAndFillsTheGaps() {
    // Right on a row: 159" → 3400 RPM.
    assertEquals(3400, AimHelper.shooterRpmForDistance(Units.inchesToMeters(159)), 1);
    // Halfway between 135" (3100) and 159" (3400) → halfway between: 3250.
    assertEquals(3250, AimHelper.shooterRpmForDistance(Units.inchesToMeters(147)), 1);
  }

  @Test
  void rpmTableHoldsItsEndsOutsideTheRange() {
    // Closer than the first row or farther than the last: use the end value.
    assertEquals(2800, AimHelper.shooterRpmForDistance(Units.inchesToMeters(40)), 1);
    assertEquals(3800, AimHelper.shooterRpmForDistance(Units.inchesToMeters(300)), 1);
  }

  @Test
  void inOurZoneWeShootAtTheHub() {
    // Blue robot at x = 2 m (zone line is 170" ≈ 4.32 m): SHOOT, same heading as before.
    Pose2d robot = new Pose2d(2.0, 6.0, Rotation2d.kZero);
    assertEquals(AimHelper.AimMode.SHOOT, AimHelper.aimMode(robot, Alliance.Blue));
    assertSameHeading(
        AimHelper.shootHeading(robot, Alliance.Blue).getDegrees(),
        AimHelper.aimHeading(robot, Alliance.Blue));
  }

  @Test
  void inTheNeutralZoneWePassToOurWall() {
    // Blue robot at mid-field: PASS, back to the blue wall means the front faces 0°.
    Pose2d blue = new Pose2d(8.0, 2.0, Rotation2d.kZero);
    assertEquals(AimHelper.AimMode.PASS, AimHelper.aimMode(blue, Alliance.Blue));
    assertSameHeading(0, AimHelper.aimHeading(blue, Alliance.Blue));
    // Red robot at mid-field: back to the red wall means the front faces 180°.
    Pose2d red = new Pose2d(8.0, 2.0, Rotation2d.kZero);
    assertEquals(AimHelper.AimMode.PASS, AimHelper.aimMode(red, Alliance.Red));
    assertSameHeading(180, AimHelper.aimHeading(red, Alliance.Red));
  }

  @Test
  void presetShotsTurnTheBackToThe2026Direction() {
    // 2026 tower shot faced 0° on blue with its FRONT shooter; our back faces there instead.
    assertSameHeading(180, AimHelper.presetHeading(0, 180, Alliance.Blue));
    assertSameHeading(0, AimHelper.presetHeading(0, 180, Alliance.Red));
  }

  @Test
  void bumpHeadingPicksTheClosestDiagonal() {
    assertSameHeading(45, AimHelper.bumpHeading(Rotation2d.fromDegrees(10)));
    assertSameHeading(-135, AimHelper.bumpHeading(Rotation2d.fromDegrees(-170)));
  }
}
