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
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.sotm.ShotCalculator.LaunchParameters;

/** Add your docs here. */
public class SemiAutoHelper {
  public record ShotParams(AngularVelocity shooterVelocity, Rotation2d driveAngle) {
  }

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
  private static int BLUE_ALLIANCE_X = 170;
  private static int RED_ALLIANCE_X = 457;

  public SemiAutoHelper(final Drive drive) {
    // TODO: add back in
    // GeneratedLUT shootOnTheMoveLookupTable = createShootOnTheMoveLookupTable();
    // this.shotCalculator = setupShotCalculator(shootOnTheMoveLookupTable);
    this.drive = drive;
  }

  private static Translation2d getHubPosition() {
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
    Pose2d robotPose = drive.getPose();
    Distance robotX = Distance.ofBaseUnits(robotPose.getX(), Meters);
    Distance robotY = Distance.ofBaseUnits(robotPose.getY(), Meters);
    Translation2d robotPos = new Translation2d(robotX, robotY);
    Translation2d hubPos = getHubPosition();
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
    // TODO: Replace with interpolating map
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
    return robotX.in(Inches) > BLUE_ALLIANCE_X && robotX.in(Inches) < RED_ALLIANCE_X;
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
    if (isInAllianceZone(drive)) {
      return getShooterVelocityForHubDistance(drive);
    } else {
      return getShooterVelocityForPassing();
    }
  }

  public static Rotation2d getFullAutoDriveAngle(Drive drive) {
    if (isInAllianceZone(drive)) {
      return getAngleToHub(drive);
    } else {
      return getAngleToAlliance();
    }
  }

  private static boolean isInAllianceZone(Drive drive) {
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    Pose2d currentPose = drive.getPose();
    Distance robotX = Meters.of(currentPose.getTranslation().getX());
    if (isRedAlliance) {
      return robotX.in(Inches) > RED_ALLIANCE_X;
    } else {
      return robotX.in(Inches) < BLUE_ALLIANCE_X;
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

  // Shoot on the move
  private GeneratedLUT createShootOnTheMoveLookupTable() {
    // plug in YOUR robot's measurements from CAD
    ProjectileSimulator.SimParameters params = new ProjectileSimulator.SimParameters(
        0.215, // ball mass kg
        0.1501, // ball diameter m
        0.47, // drag coeff (smooth sphere)
        0.2, // Magnus coeff
        1.225, // air density
        0.43, // exit height (m), floor to where the ball leaves the shooter
        0.1016, // flywheel diameter (m), measure with calipers
        1.83, // target height (m), from game manual
        0.6, // slip factor (0=no grip, 1=perfect), tune this on the real robot
        45.0, // launch angle from horizontal, measure from CAD
        0.001, // sim timestep
        1500, 6000, 25, 5.0 // RPM search range, iterations, max sim time
    );

    ProjectileSimulator sim = new ProjectileSimulator(params);
    ProjectileSimulator.GeneratedLUT lut = sim.generateLUT();

    // print it out
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        System.out.printf("Shoot on the move lookup value: %.2fm -> %.0f RPM, %.3fs TOF%n",
            entry.distanceM(), entry.rpm(), entry.tof());
      }
    }

    return lut;
  }

  private ShotCalculator setupShotCalculator(GeneratedLUT lut) {
    // in RobotContainer or wherever you set stuff up
    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = 0.23; // how far forward the launcher is from robot center (m)
    config.launcherOffsetY = 0.0; // how far left, 0 if centered
    config.phaseDelayMs = 30.0; // your vision pipeline latency
    config.mechLatencyMs = 20.0; // how long the mechanism takes to respond
    config.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps)
    config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
    config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub
    config.maxSOTMSpeed = 10.0;

    ShotCalculator shotCalc = new ShotCalculator(config);

    // load the LUT you generated
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
      }
    }

    return shotCalc;
  }

  public LaunchParameters shootOnTheMoveCalculation() {
    // call this every cycle in robotPeriodic()
    Translation2d hubCenter = getHubPosition();
    Translation2d hubForward = getHubFacingDirection(); // which way the hub faces

    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        drive.getPose(),
        drive.getFieldVelocity(),
        drive.getChassisSpeeds(),
        hubCenter,
        hubForward,
        0.9, // vision confidence, 0 to 1
        0.0,
        0.0);

    return this.shotCalculator.calculate(inputs);
  }
}
