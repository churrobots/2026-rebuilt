// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ControlsConstants;

/** Add your docs here. */
public class SemiAutoHelper {

  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private Supplier<Pose2d> robotPoseSupplier;

  private TunableNumber tunableShooterAddedRpm = new TunableNumber("SHOOTER_BOOST", 0);

  // Use this to automatically select an RPM for a given distance
  InterpolatingDoubleTreeMap lookupDistanceInInchesToRPM = new InterpolatingDoubleTreeMap();

  public SemiAutoHelper(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    // First Value is INCHES btw
    // TODO: Howard said we mis-measured these by 8 inches, need to subtract 8
    // inches from all the distance measurements (the first number)
    lookupDistanceInInchesToRPM.put(207., 4300.);
    lookupDistanceInInchesToRPM.put(183., 3900.);
    lookupDistanceInInchesToRPM.put(159., 3500.);
    lookupDistanceInInchesToRPM.put(135., 3200.);
    lookupDistanceInInchesToRPM.put(111., 3000.);
    lookupDistanceInInchesToRPM.put(99., 2900.);
    lookupDistanceInInchesToRPM.put(88., 2800.);
    lookupDistanceInInchesToRPM.put(75., 2750.);
    lookupDistanceInInchesToRPM.put(69., 2750.);
  }

  public Rotation2d getAngleToHub() {
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    Distance blueHubX = Distance.ofBaseUnits(4.63, Meters);
    Distance redHubX = Distance.ofBaseUnits(aprilTagLayout.getFieldLength(), Meters).minus(blueHubX);
    Distance hubY = Distance.ofBaseUnits(4.035, Meters);
    Distance hubX = isRedAlliance ? redHubX : blueHubX;
    Pose2d robotPose = robotPoseSupplier.get();
    Distance robotX = Distance.ofBaseUnits(robotPose.getX(), Meters);
    Distance robotY = Distance.ofBaseUnits(robotPose.getY(), Meters);
    double targetAngleInRadians = Math.atan2(
        hubY.minus(robotY).in(Meters),
        hubX.minus(robotX).in(Meters));
    return Rotation2d.fromRadians(targetAngleInRadians);

  }

  public Distance getDistanceToHub() {
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    Distance blueHubX = Distance.ofBaseUnits(4.63, Meters);
    Distance redHubX = Distance.ofBaseUnits(aprilTagLayout.getFieldLength(), Meters).minus(blueHubX);
    Distance hubY = Distance.ofBaseUnits(4.035, Meters);
    Distance hubX = isRedAlliance ? redHubX : blueHubX;
    Pose2d robotPose = robotPoseSupplier.get();
    Distance robotX = Distance.ofBaseUnits(robotPose.getX(), Meters);
    Distance robotY = Distance.ofBaseUnits(robotPose.getY(), Meters);
    Translation2d robotPos = new Translation2d(robotX, robotY);
    Translation2d hubPos = new Translation2d(hubX, hubY);
    double distance = robotPos.getDistance(hubPos);
    SmartDashboard.putNumber("getDistance", Meters.of(distance).in(Inches));
    return Meters.of(distance);
  }

  public AngularVelocity getShooterVelocity(Distance distanceToHub) {
    double inputInInches = distanceToHub.in(Inches);
    double shooterRpm = lookupDistanceInInchesToRPM.get(inputInInches) + tunableShooterAddedRpm.getLatest();
    SmartDashboard.putNumber("getVelocity", shooterRpm);
    return RPM.of(shooterRpm);
  }

  public AngularVelocity getFeederVelocity(Distance distanceToHub) {
    double inputInInches = distanceToHub.in(Inches);
    double shooterRpm = lookupDistanceInInchesToRPM.get(inputInInches);
    double feederRpm = ControlsConstants.FEEDER_TO_SHOOTER_RPM_RATIO * shooterRpm;
    return RPM.of(feederRpm);
  }

  public AngularVelocity getShooterVelocityForHubDistance() {
    return getShooterVelocity(getDistanceToHub());
  }

  public AngularVelocity getFeederVelocityForHubDistance() {
    return getFeederVelocity(getDistanceToHub());
  }

  public boolean isByBlueAlliance() {
    Pose2d currentPose = robotPoseSupplier.get();
    if (currentPose.getTranslation().getX() < Units.inchesToMeters(325.65)) {
      return true;
    }
    return false;
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
  }

  public boolean isByOutpost() {
    Pose2d currentPose = robotPoseSupplier.get();
    if ((!isRedAlliance() && currentPose.getTranslation().getY() < Units.inchesToMeters(158.32))
        || (isRedAlliance() && currentPose.getTranslation().getY() > Units.inchesToMeters(158.32))) {
      return true;
    }
    return false;
  }
}
