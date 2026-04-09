// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String cameraFrontRight = "camera_frontright";
  public static String cameraBackRight = "camera_backright";
  public static String cameraFrontLeft = "camera_frontleft";
  public static String cameraBackLeft = "camera_backleft";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCameraFrontRight = new Transform3d(
      Units.inchesToMeters(0.875), Units.inchesToMeters(-13),
      Units.inchesToMeters(16.875),
      new Rotation3d(
          Math.PI,
          Units.degreesToRadians(-15),
          Units.degreesToRadians(-43) // NOTE: CAD says 45deg but this works better
      ));

  public static Transform3d robotToCameraBackRight = new Transform3d(
      Units.inchesToMeters(-11.875), Units.inchesToMeters(-10.875),
      Units.inchesToMeters(16.75),
      new Rotation3d(
          Math.PI, // camera is upside down
          Units.degreesToRadians(-15), // pitched back
          Units.degreesToRadians(-150) // facing out and back
      ));

  public static Transform3d robotToCameraFrontLeft = new Transform3d(
      Units.inchesToMeters(0.875), Units.inchesToMeters(13), Units.inchesToMeters(
          16.875),
      new Rotation3d(
          Math.PI,
          Units.degreesToRadians(-15),
          Units.degreesToRadians(43) // NOTE: CAD says 45deg but this works better
      ));

  public static Transform3d robotToCameraBackLeft = new Transform3d(
      Units.inchesToMeters(-11.875), Units.inchesToMeters(12.125),
      Units.inchesToMeters(10),
      new Rotation3d(
          Math.PI, // camera is upside down
          Units.degreesToRadians(-18), // pitched back
          Units.degreesToRadians(150) // facing out and back
      ));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.1;
  public static Distance maxDistance = Feet.of(25);
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {
      0.6, // Camera 0 - frontright
      0.4, // Camera 1 - backright
      0.4, // Camera 2 - frontleft
      0.6, // Camera 3 - backleft
  };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
