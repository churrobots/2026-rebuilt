package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
}
