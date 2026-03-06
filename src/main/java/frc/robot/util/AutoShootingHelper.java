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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class AutoShootingHelper {

  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private Supplier<Pose2d> m_robotPoseSupplier;

  // Use this to automatically select an RPM for a given distance
  InterpolatingDoubleTreeMap lookupDistanceInInchesToRPM = new InterpolatingDoubleTreeMap();

  public AutoShootingHelper(Supplier<Pose2d> robotPoseSupplier) {
    m_robotPoseSupplier = robotPoseSupplier;

    // First Value is INCHES btw
    lookupDistanceInInchesToRPM.put(215., 4300.);
    lookupDistanceInInchesToRPM.put(191., 3900.);
    lookupDistanceInInchesToRPM.put(167., 3500.);
    lookupDistanceInInchesToRPM.put(143., 3200.);
    lookupDistanceInInchesToRPM.put(119., 3000.);
    lookupDistanceInInchesToRPM.put(107., 2900.);
    lookupDistanceInInchesToRPM.put(95., 2800.);
    lookupDistanceInInchesToRPM.put(83., 2750.);
    lookupDistanceInInchesToRPM.put(77., 2750.);
  }

  public Rotation2d getAngleToHub() {
    boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    Distance blueHubX = Distance.ofBaseUnits(4.63, Meters);
    Distance redHubX = Distance.ofBaseUnits(aprilTagLayout.getFieldLength(), Meters).minus(blueHubX);
    Distance hubY = Distance.ofBaseUnits(4.035, Meters);
    Distance hubX = isRedAlliance ? redHubX : blueHubX;
    Pose2d robotPose = m_robotPoseSupplier.get();
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
    Pose2d robotPose = m_robotPoseSupplier.get();
    Distance robotX = Distance.ofBaseUnits(robotPose.getX(), Meters);
    Distance robotY = Distance.ofBaseUnits(robotPose.getY(), Meters);
    Translation2d robotPos = new Translation2d(robotX, robotY);
    Translation2d hubPos = new Translation2d(hubX, hubY);
    double distance = robotPos.getDistance(hubPos);
    return Meters.of(distance);
  }

  public AngularVelocity getShooterVelocityToHitHub() {
    Distance distance = getDistanceToHub();
    double inputInInches = distance.in(Inches);
    double rpm = lookupDistanceInInchesToRPM.get(inputInInches);
    return RPM.of(rpm);
  }
}
