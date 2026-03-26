// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

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

  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private ShotCalculator shotCalculator;
  private Drive drive;

  private static TunableNumber tunableShooterAddedRpm = new TunableNumber("SHOOTER_BOOST", 0);

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
    GeneratedLUT shootOnTheMoveLookupTable = createShootOnTheMoveLookupTable();
    this.shotCalculator = setupShotCalculator(shootOnTheMoveLookupTable);
    this.drive = drive;
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
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    Distance blueHubX = Distance.ofBaseUnits(4.63, Meters);
    Distance redHubX = Distance.ofBaseUnits(aprilTagLayout.getFieldLength(), Meters).minus(blueHubX);
    Distance hubY = Distance.ofBaseUnits(4.035, Meters);
    Distance hubX = isRedAlliance ? redHubX : blueHubX;
    Pose2d robotPose = drive.getPose();
    Distance robotX = Distance.ofBaseUnits(robotPose.getX(), Meters);
    Distance robotY = Distance.ofBaseUnits(robotPose.getY(), Meters);
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
    return RPM.of(2900);
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

    }
  }
}
