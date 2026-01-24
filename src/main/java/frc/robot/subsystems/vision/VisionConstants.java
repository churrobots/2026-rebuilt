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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String cameraFrontName = "camera_front";
  public static String cameraBackName = "camera_back";
  public static String cameraRightName = "camera_right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  public static Transform3d robotToCameraFront = new Transform3d(-10.25 * Constants.INCHES_TO_METERS,
      -8.5 * Constants.INCHES_TO_METERS, 18.5 * Constants.INCHES_TO_METERS, new Rotation3d(-1 * Math.PI / 2, 0, 0.0));
  public static Transform3d robotToCameraBack = new Transform3d(-15 * Constants.INCHES_TO_METERS,
      8.5 * Constants.INCHES_TO_METERS, 19 * Constants.INCHES_TO_METERS, new Rotation3d(1 * Math.PI / 2, 0, Math.PI));
  public static Transform3d robotToCameraRight = new Transform3d(-14 * Constants.INCHES_TO_METERS,
      -9.5 * Constants.INCHES_TO_METERS, 15 * Constants.INCHES_TO_METERS,
      new Rotation3d(-1 * Math.PI / 2, 0, -1 * Math.PI / 2));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {
      1.0, // Camera 0
      1.0 // Camera 1
  };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
