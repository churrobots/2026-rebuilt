package frc.robot.sim;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Sim-only "walls": keeps the simulated robot inside the field and out of the hubs. Real robots
 * have real walls, so this only runs in simulation.
 */
public class FieldBoundaries {
  // Bumper-to-bumper size of last year's robot (robotWidth/robotLength in PathPlanner's
  // settings.json on main). TODO: measure the new robot with its bumpers on.
  public static final double ROBOT_SIZE_METERS = 0.9;
  private static final double ROBOT_HALF_SIZE = ROBOT_SIZE_METERS / 2.0;

  // Hubs from the game manual (FuelPhysicsSim on main): a 47 in square base.
  private static final double HUB_SIDE = Units.inchesToMeters(47);
  private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.5974, 4.035);
  private static final Translation2d RED_HUB_CENTER = new Translation2d(11.938, 4.035);

  private final double fieldLength = aprilTagLayout.getFieldLength();
  private final double fieldWidth = aprilTagLayout.getFieldWidth();
  private static final List<Box> hubs = List.of(hubAt(BLUE_HUB_CENTER), hubAt(RED_HUB_CENTER));
  // Tags sit right on the hub's sides, so shrink the hub a little when checking what
  // blocks a camera. Otherwise a tag would be "hidden" by the very side it's stuck to.
  private static final double SIGHT_MARGIN = 0.05;

  /** An axis-aligned rectangle on the field, in meters. */
  private record Box(double minX, double minY, double maxX, double maxY) {}

  private static Box hubAt(Translation2d center) {
    double half = HUB_SIDE / 2.0;
    return new Box(
        center.getX() - half, center.getY() - half, center.getX() + half, center.getY() + half);
  }

  /**
   * Returns true if a hub is in the way between two spots on the field (top-down), so a
   * camera at one spot can't see a tag at the other.
   */
  public static boolean isBlocked(Translation2d from, Translation2d to) {
    for (Box hub : hubs) {
      if (segmentHitsBox(
          from,
          to,
          new Box(
              hub.minX() + SIGHT_MARGIN,
              hub.minY() + SIGHT_MARGIN,
              hub.maxX() - SIGHT_MARGIN,
              hub.maxY() - SIGHT_MARGIN))) {
        return true;
      }
    }
    return false;
  }

  /** Does the straight line from a to b pass through the box? ("slab" method) */
  private static boolean segmentHitsBox(Translation2d a, Translation2d b, Box box) {
    double enter = 0.0; // how far along the line (0 = a, 1 = b) we enter the box
    double exit = 1.0; // ...and leave it
    double[] start = {a.getX(), a.getY()};
    double[] step = {b.getX() - a.getX(), b.getY() - a.getY()};
    double[] min = {box.minX(), box.minY()};
    double[] max = {box.maxX(), box.maxY()};
    for (int axis = 0; axis < 2; axis++) {
      if (Math.abs(step[axis]) < 1e-9) {
        // Moving parallel to this side: blocked only if already between the sides.
        if (start[axis] < min[axis] || start[axis] > max[axis]) return false;
      } else {
        double t1 = (min[axis] - start[axis]) / step[axis];
        double t2 = (max[axis] - start[axis]) / step[axis];
        enter = Math.max(enter, Math.min(t1, t2));
        exit = Math.min(exit, Math.max(t1, t2));
        if (enter > exit) return false;
      }
    }
    return true;
  }

  /** Pushes the robot back out if it went through a wall or into a hub. Call once per loop. */
  public void apply(Drive drive) {
    Pose2d pose = drive.getPose();

    // How far the robot sticks out in x and y, which depends on how it's turned.
    double cos = Math.abs(pose.getRotation().getCos());
    double sin = Math.abs(pose.getRotation().getSin());
    double reach = ROBOT_HALF_SIZE * (cos + sin);

    // Stay inside the field walls.
    double x = Math.max(reach, Math.min(fieldLength - reach, pose.getX()));
    double y = Math.max(reach, Math.min(fieldWidth - reach, pose.getY()));

    // Stay out of the hubs: if inside, slide out the shortest way.
    for (Box hub : hubs) {
      double left = x - (hub.minX() - reach);
      double right = (hub.maxX() + reach) - x;
      double below = y - (hub.minY() - reach);
      double above = (hub.maxY() + reach) - y;
      if (left > 0 && right > 0 && below > 0 && above > 0) {
        double shortest = Math.min(Math.min(left, right), Math.min(below, above));
        if (shortest == left) x -= left;
        else if (shortest == right) x += right;
        else if (shortest == below) y -= below;
        else y += above;
      }
    }

    if (x != pose.getX() || y != pose.getY()) {
      drive.setPose(new Pose2d(x, y, pose.getRotation()));
    }

    // Share the hub boxes so the dashboard draws them exactly where the walls are.
    double[] hubCorners = new double[hubs.size() * 4];
    for (int i = 0; i < hubs.size(); i++) {
      Box hub = hubs.get(i);
      hubCorners[i * 4] = hub.minX();
      hubCorners[i * 4 + 1] = hub.minY();
      hubCorners[i * 4 + 2] = hub.maxX();
      hubCorners[i * 4 + 3] = hub.maxY();
    }
    Logger.recordOutput("Field/Hubs", hubCorners);
    Logger.recordOutput("Field/Size", new double[] {fieldLength, fieldWidth});
    Logger.recordOutput("Field/RobotSizeMeters", ROBOT_SIZE_METERS);
  }
}
