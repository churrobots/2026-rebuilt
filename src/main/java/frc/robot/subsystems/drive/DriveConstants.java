// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
  };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(3.065);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-3.101);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(-3.108);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(3.107);

  // Device CAN IDs
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 5;
  public static final int backLeftDriveCanId = 7;
  public static final int frontRightDriveCanId = 6;
  public static final int backRightDriveCanId = 8;

  public static final int frontLeftTurnCanId = 1;
  public static final int backLeftTurnCanId = 3;
  public static final int frontRightTurnCanId = 2;
  public static final int backRightTurnCanId = 4;

  // Drive motor configuration
  // This is how to tune all the values here
  // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/#tuning
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.479);
  public static final double driveMotorReduction = 5.143; // (45.0 * 20.0) / (16.0 * 15.0); // MAXSwerve with 14 pinion
                                                          // teeth and
                                                          // 22 spur teeth
                                                          // we think the 15 teeth is the bevel
                                                          // drive shaft, and the 45 is the
                                                          // wheel bevel
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel
                                                                                             // Radians
  public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
                                                                                                      // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.17859;
  public static final double driveKv = 0.06385;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 22.6796;
  public static final double robotMOI = 2;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig = new RobotConfig(
      Mass.ofBaseUnits(robotMassKg, Kilograms),
      MomentOfInertia.ofBaseUnits(robotMOI, KilogramSquareMeters),
      new ModuleConfig(
          Distance.ofBaseUnits(wheelRadiusMeters, Meters),
          LinearVelocity.ofBaseUnits(maxSpeedMetersPerSec, MetersPerSecond),
          wheelCOF,
          driveGearbox.withReduction(driveMotorReduction),
          Current.ofBaseUnits(driveMotorCurrentLimit, Amps),
          1),
      moduleTranslations);
}
