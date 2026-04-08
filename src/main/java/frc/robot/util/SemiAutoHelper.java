// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ControlsConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.sotm.ProjectileSimulator;
import frc.robot.subsystems.sotm.ProjectileSimulator.GeneratedLUT;
import frc.robot.subsystems.sotm.ProjectileSimulator.SimParameters;
import frc.robot.subsystems.sotm.ShotCalculator;

/** Add your docs here. */
public class SemiAutoHelper {

  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private ShotCalculator shotCalculator;
  private Drive drive;

  private static TunableNumber tunableShooterAddedRpm = new TunableNumber("SHOOTER_BOOST", 0);
  // Drive angle range from [-180, 180]
  private static List<Double> bumpAngles = List.of(45.0, 45.0 + 90, -45.0, -45.0 - 90);

  // Use this to automatically select an RPM for a given distance (Inches, RPM)
  static InterpolatingDoubleTreeMap lookupDistanceInInchesToRPM = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(207., 4000.),
      Map.entry(207., 4000.),
      Map.entry(183., 3750.),
      Map.entry(159., 3400.),
      Map.entry(135., 3100.),
      Map.entry(111., 2900.),
      Map.entry(99., 2850.),
      Map.entry(88., 2775.),
      Map.entry(75., 2800.));

  public SemiAutoHelper(final Drive drive) {
    this.drive = drive;

    System.out.println("=== SOTM LUT Generation Started ===");
    long startTime = System.currentTimeMillis();

    GeneratedLUT shootOnTheMoveLookupTable = createShootOnTheMoveLookupTable();

    long elapsed = System.currentTimeMillis() - startTime;
    System.out.println("✓ LUT Generated in " + elapsed + "ms");
    System.out.println("  - Reachable entries: " + shootOnTheMoveLookupTable.reachableCount());
    System.out.println("  - Max range: " + String.format("%.2f", shootOnTheMoveLookupTable.maxRangeM()) + "m");
    System.out.println("=== SOTM Ready ===");

    this.shotCalculator = setupShotCalculator(shootOnTheMoveLookupTable);
  }

  private Translation2d getHubPosition() {
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    Distance blueHubX = Distance.ofBaseUnits(4.63, Meters);
    Distance redHubX = Distance.ofBaseUnits(aprilTagLayout.getFieldLength(), Meters).minus(blueHubX);
    Distance hubY = Distance.ofBaseUnits(4.035, Meters);
    Distance hubX = isRedAlliance ? redHubX : blueHubX;

    return new Translation2d(hubX, hubY);
  }

  private Translation2d getHubFacingDirection() {
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    if (isRedAlliance) {
      return new Translation2d(-1, 0);
    }
    return new Translation2d(1, 0);
  }

  public static Rotation2d getAngleToHub(Drive drive) {
    Pose2d robotPose = drive.getPose();
    Distance robotX = Distance.ofBaseUnits(robotPose.getX(), Meters);
    Distance robotY = Distance.ofBaseUnits(robotPose.getY(), Meters);

    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    Distance blueHubY = Distance.ofBaseUnits(4.035, Meters);
    Distance redHubY = Distance.ofBaseUnits(4.035, Meters);
    Distance hubY = isRedAlliance ? redHubY : blueHubY;

    boolean isOnLeftSide = isRedAlliance ? robotY.lte(redHubY) : robotY.gte(blueHubY);
    Distance offsetToCompensateForLeftCannon = isOnLeftSide ? Inches.of(-9.0) : Inches.of(9.0);
    Distance blueHubX = Distance.ofBaseUnits(4.63 + offsetToCompensateForLeftCannon.in(Meters), Meters);
    Distance redHubX = Distance.ofBaseUnits(aprilTagLayout.getFieldLength(), Meters).minus(blueHubX);
    Distance hubX = isRedAlliance ? redHubX : blueHubX;

    double targetAngleInRadians = Math.atan2(
        hubY.minus(robotY).in(Meters),
        hubX.minus(robotX).in(Meters));
    SmartDashboard.putNumber("getAngleToHub", Radians.of(targetAngleInRadians).in(Degrees));
    return Rotation2d.fromRadians(targetAngleInRadians);
  }

  public static Distance getDistanceToHub(Drive drive) {
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    Distance blueHubX = Distance.ofBaseUnits(4.63, Meters);
    Distance redHubX = Distance.ofBaseUnits(aprilTagLayout.getFieldLength(), Meters).minus(blueHubX);
    Distance hubY = Distance.ofBaseUnits(4.035, Meters);
    Distance hubX = isRedAlliance ? redHubX : blueHubX;
    Pose2d robotPose = drive.getPose();
    Distance robotX = Distance.ofBaseUnits(robotPose.getX(), Meters);
    Distance robotY = Distance.ofBaseUnits(robotPose.getY(), Meters);
    Translation2d robotPos = new Translation2d(robotX, robotY);
    Translation2d hubPos = new Translation2d(hubX, hubY);
    double distance = robotPos.getDistance(hubPos);
    SmartDashboard.putNumber("getDistance", Meters.of(distance).in(Inches));
    return Meters.of(distance);
  }

  public static AngularVelocity getShooterVelocity(Distance distanceToHub) {
    double inputInInches = distanceToHub.in(Inches);
    double shooterRpm = lookupDistanceInInchesToRPM.get(inputInInches) + tunableShooterAddedRpm.getLatest();
    SmartDashboard.putNumber("getVelocity", shooterRpm);
    return RPM.of(shooterRpm);
  }

  public static AngularVelocity getFeederVelocity(Distance distanceToHub) {
    double inputInInches = distanceToHub.in(Inches);
    double shooterRpm = lookupDistanceInInchesToRPM.get(inputInInches);
    double feederRpm = ControlsConstants.FEEDER_TO_SHOOTER_RPM_RATIO * shooterRpm;
    return RPM.of(feederRpm);
  }

  public static AngularVelocity getShooterVelocityForHubDistance(Drive drive) {
    return getShooterVelocity(getDistanceToHub(drive));
  }

  public static AngularVelocity getShooterVelocityForPassing() {
    // Could update to be dependent on distance to alliance zone
    return RPM.of(3400);
  }

  public static AngularVelocity getFeederVelocityForHubDistance(Drive drive) {
    return getFeederVelocity(getDistanceToHub(drive));
  }

  public boolean isByBlueAlliance() {
    Pose2d currentPose = drive.getPose();
    if (currentPose.getTranslation().getX() < Units.inchesToMeters(325.65)) {
      return true;
    }
    return false;
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
  }

  public boolean isByOutpost() {
    Pose2d currentPose = drive.getPose();
    if ((!isRedAlliance() && currentPose.getTranslation().getY() < Units.inchesToMeters(158.32))
        || (isRedAlliance() && currentPose.getTranslation().getY() > Units.inchesToMeters(158.32))) {
      return true;
    }
    return false;
  }

  public static boolean isInNeutralZone(Drive drive) {
    Pose2d currentPose = drive.getPose();
    Distance robotX = Meters.of(currentPose.getTranslation().getX());
    // Neutral zone begins at 170" and goes to 457" according to
    // the field map documentation.
    return robotX.in(Inches) > 170 && robotX.in(Inches) < 457;
  }

  public static boolean isInTrenchBumpZone(Drive drive) {
    Pose2d currentPose = drive.getPose();
    Distance robotX = Meters.of(currentPose.getTranslation().getX());
    // 158" from alliance wall --> 205" from alliance wall
    Distance bufferZone = Inches.of(19);
    Distance fieldLength = Inches.of(650.12);
    Distance blueStartX = Inches.of(158).minus(bufferZone);
    Distance blueEndX = Inches.of(205).plus(bufferZone);
    Distance redStartX = fieldLength.minus(blueStartX);
    Distance redEndX = fieldLength.minus(blueEndX);
    boolean isInRedTrench = robotX.gte(redEndX) && robotX.lte(redStartX);
    boolean isInBlueTrench = robotX.gte(blueStartX) && robotX.lte(blueEndX);
    return isInBlueTrench || isInRedTrench;
  }

  public static Rotation2d getAngleToAlliance() {
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    if (isRedAlliance) {
      return Rotation2d.fromDegrees(0);
    } else {
      return Rotation2d.fromDegrees(180);
    }
  }

  public static AngularVelocity getFullAutoShooterVelocity(Drive drive) {
    if (isInNeutralZone(drive)) {
      return getShooterVelocityForPassing();
    } else {
      return getShooterVelocityForHubDistance(drive);
    }
  }

  public static Rotation2d getFullAutoDriveAngle(Drive drive) {
    if (isInNeutralZone(drive)) {
      return getAngleToAlliance();
    } else {
      return getAngleToHub(drive);
    }
  }

  public static Rotation2d getAngleToDriveOverBump(Drive drive) {
    double driveAngle = drive.getPose().getRotation().getDegrees();
    double minDiff = Double.MAX_VALUE;
    double angleToDrive = 0;
    for (Double candidateAngle : bumpAngles) {
      double angleDiff = Math.abs(driveAngle - candidateAngle);
      if (angleDiff < minDiff) {
        minDiff = angleDiff;
        angleToDrive = candidateAngle;
      }
    }

    return Rotation2d.fromDegrees(angleToDrive);
  }

  /**
   * Creates the shoot-on-the-move lookup table using physics simulation.
   * Configure these parameters based on your robot's CAD measurements and testing.
   */
  private GeneratedLUT createShootOnTheMoveLookupTable() {
    // Configure your robot's physical parameters
    // TODO: MEASURE THESE VALUES FROM YOUR CAD AND TUNE ON ROBOT
    SimParameters params = new SimParameters(
        0.215,    // ball mass kg (from game manual - 7.5 oz)
        0.1501,   // ball diameter m (from game manual - 5.9 inches)
        0.47,     // drag coefficient (smooth sphere, tune if needed)
        0.2,      // Magnus coefficient (tune on robot for backspin/topspin effects)
        1.225,    // air density kg/m^3 (sea level standard)
        0.43,     // exit height from floor in meters (MEASURE FROM YOUR CAD)
        0.1016,   // wheel diameter in meters (MEASURE YOUR ACTUAL WHEELS - 4 inches default)
        1.83,     // target height in meters (hub height from game manual)
        0.6,      // slip factor 0-1 (TUNE ON ROBOT - how much wheel speed transfers to ball)
        45.0,     // launch angle in degrees (MEASURE FROM YOUR SHOOTER ANGLE)
        0.001,    // simulation timestep (smaller = more accurate but slower)
        1500,     // minimum RPM to test
        6000,     // maximum RPM to test
        25,       // binary search iterations for RPM solving
        5.0       // max simulation time in seconds
    );

    ProjectileSimulator sim = new ProjectileSimulator(params);
    return sim.generateLUT();
  }

  /**
   * Sets up the ShotCalculator with the generated lookup table and configuration.
   */
  private ShotCalculator setupShotCalculator(GeneratedLUT lut) {
    ShotCalculator.Config config = new ShotCalculator.Config();

    // Launcher position relative to robot center (MEASURE FROM CAD)
    config.launcherOffsetX = 0.20;  // meters forward of robot center
    config.launcherOffsetY = 0.0;   // meters left of robot center (0 = centered)

    // Scoring range limits
    config.minScoringDistance = 0.5;  // minimum distance to shoot from (meters)
    config.maxScoringDistance = 5.0;  // maximum distance to shoot from (meters)

    // SOTM (Shoot On The Move) speed thresholds
    config.minSOTMSpeed = 0.1;  // below this speed (m/s), use static aim
    config.maxSOTMSpeed = 3.0;  // above this speed (m/s), don't shoot (too fast)

    // Latency compensation (milliseconds)
    config.phaseDelayMs = 30.0;   // vision pipeline processing lag
    config.mechLatencyMs = 20.0;  // mechanism response time

    // Drag compensation for horizontal velocity (tune if needed)
    config.sotmDragCoeff = 0.24;  // default for FRC ball at ~10 m/s

    ShotCalculator calculator = new ShotCalculator(config);

    // Load all reachable entries from the generated LUT
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        calculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
      }
    }

    return calculator;
  }

  /**
   * Calculate the shoot-on-the-move firing solution.
   * Accounts for robot velocity, drag, and latency.
   * @return LaunchParameters with RPM, drive angle, TOF, and confidence (0-100)
   */
  public ShotCalculator.LaunchParameters calculateSOTMShot() {
    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        drive.getPose(),
        drive.getFieldVelocity(),    // field-relative velocity
        drive.getChassisSpeeds(),    // robot-relative velocity
        getHubPosition(),
        getHubFacingDirection(),
        1.0  // vision confidence (0-1) - can wire to actual vision system later
    );

    return shotCalculator.calculate(inputs);
  }

  /**
   * Publishes detailed SOTM telemetry to SmartDashboard for debugging.
   * Call this periodically to monitor SOTM performance.
   */
  public void publishSOTMTelemetry() {
    ShotCalculator.LaunchParameters shot = calculateSOTMShot();
    edu.wpi.first.math.kinematics.ChassisSpeeds fieldVel = drive.getFieldVelocity();

    // Core SOTM outputs
    SmartDashboard.putNumber("SOTM/RPM", shot.rpm());
    SmartDashboard.putNumber("SOTM/DriveAngleDeg", shot.driveAngle().getDegrees());
    SmartDashboard.putNumber("SOTM/TOF", shot.timeOfFlightSec());
    SmartDashboard.putNumber("SOTM/Confidence", shot.confidence());
    SmartDashboard.putBoolean("SOTM/Valid", shot.isValid());
    SmartDashboard.putNumber("SOTM/SolvedDistance", shot.solvedDistanceM());
    SmartDashboard.putNumber("SOTM/Iterations", shot.iterationsUsed());
    SmartDashboard.putBoolean("SOTM/WarmStart", shot.warmStartUsed());

    // Robot state
    SmartDashboard.putNumber("SOTM/RobotSpeedMPS",
        Math.hypot(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond));
    SmartDashboard.putNumber("SOTM/RobotVelX", fieldVel.vxMetersPerSecond);
    SmartDashboard.putNumber("SOTM/RobotVelY", fieldVel.vyMetersPerSecond);
    SmartDashboard.putNumber("SOTM/RobotOmega", fieldVel.omegaRadiansPerSecond);

    // Distance to target
    SmartDashboard.putNumber("SOTM/DistanceToHub", getDistanceToHub(drive).in(Meters));
  }

  /**
   * Gets the ShotCalculator instance for advanced operations.
   * @return The ShotCalculator instance, or null if not initialized
   */
  public ShotCalculator getShotCalculator() {
    return shotCalculator;
  }
}
