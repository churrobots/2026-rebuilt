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
import frc.robot.Constants;

public class DriveConstants {
  // Kraken X60 + our 4:1 gearing: (12 V - kS) / kV = about 155 wheel rad/s x 1.483 in
  // wheel = about 5.8 m/s. (It was 4.25 when the code wrongly thought we had 5.5:1 gearing.)
  // TODO: re-check after re-running the characterization autos.
  public static final double maxSpeedMetersPerSec = 5.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(24);
  public static final double wheelBase = Units.inchesToMeters(24);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
  };

  // // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0);
  // module 1
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0);
  // module 2
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0);
  // module 3
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0);

  // Device CAN IDs
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 10;
  public static final int backLeftDriveCanId = 2;
  public static final int frontRightDriveCanId = 8;
  public static final int backRightDriveCanId = 6;

  public static final int frontLeftTurnCanId = 11;
  public static final int backLeftTurnCanId = 1;
  public static final int frontRightTurnCanId = 7;
  public static final int backRightTurnCanId = 4;

  // Drive motor configuration
  // This is how to tune all the values here
  // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/#tuning

  // NOTE: this driveMotorCurrentLimit is a stator limit, so it can be above
  // 40A and is also typically above 60A even, according to some basic
  // research we did
  public static final int driveMotorCurrentLimit = 56; // contra costa: down from 60A to prevent wheel slip

  public static final double wheelRadiusMeters = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> Units.inchesToMeters(1.470);
    case Constants.ROBOT_COMP -> Units.inchesToMeters(1.483);
    default -> Units.inchesToMeters(1.483);
  };
  // MAXSwerve gearing: the motor's pinion drives the spur gear, then a 15:45 bevel to the wheel,
  // so the reduction is 3 x spur / pinion.
  //   COMP (new Kraken robot): 15T pinion, 20T spur = 4:1 (mechanical team, 2026-11-07).
  //     The code used to say 12T/22T = 5.5:1, and the wheel-turn test went about 1 3/8 turns
  //     (5.5 / 4 = 1.375). We tried 16T/20T = 3.75:1 next, and it came up a bit short of 1 turn.
  //   ALPHA (old Vortex robot): Low gear = 12T pinion, 22T spur = 5.5:1.
  // NOTE: this must CHANGE if you change the gear kit. https://www.revrobotics.com/rev-21-3005/
  public static final double pinionTeeth = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> 12.0;
    default -> 15.0;
  };
  public static final double spurTeeth = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> 22.0;
    default -> 20.0;
  };
  public static final double driveMotorReduction = (45.0 * spurTeeth) / (pinionTeeth * 15.0);
  // Drive motor is a Kraken X60 on a TalonFX (was a NEO Vortex on a SparkFlex). Used by
  // ModuleIOSim and the PathPlanner ppConfig to model the real motor.
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

  // Drive encoder configuration
  public static final boolean driveEncoderInverted = true;
  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel
                                                                                             // Radians
  public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
                                                                                                      // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  // kS (volts) and kV (volts per wheel rad/s). COMP/default = the new Kraken robot, measured
  // 2026-11-05 with "Drive Simple FF Characterization" on carpet (average of 2 runs:
  // kS 0.15231 / 0.15613, kV 0.10564 / 0.10463). ALPHA is still the old Vortex robot's.
  public static final double driveKs = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> 0.14945;
    case Constants.ROBOT_COMP -> 0.15422;
    default -> 0.15422;
  };
  // COMP kV: measured as 0.10514 on 2026-11-05, but back then the code thought the gearing was
  // 5.5:1, so its "wheel speed" was wrong. Converted for the real 4:1: 0.10514 x 4 / 5.5.
  // kS (just volts to get moving) doesn't depend on gearing.
  // TODO: re-run "Drive Simple FF Characterization" to measure kV for real.
  public static final double driveKv = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> 0.06761;
    case Constants.ROBOT_COMP -> 0.07647;
    default -> 0.07647;
  };
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Kraken (TalonFX) drive configuration -- used by ModuleIOKraken only.
  // Positive voltage must drive the robot forward; verify on-robot and flip if needed.
  // On the real robot, flipping the right side made everything drive backwards,
  // so it's the left side that needs to be inverted.
  public static final boolean frontLeftDriveInverted = true;
  public static final boolean frontRightDriveInverted = false;
  public static final boolean backLeftDriveInverted = true;
  public static final boolean backRightDriveInverted = false;
  // Supply current limit (stator limit reuses driveMotorCurrentLimit above).
  public static final int driveSupplyCurrentLimit = 60;
  // Slot0 gains in TalonFX NATIVE units. Because Feedback.SensorToMechanismRatio =
  // driveMotorReduction, the closed loop runs in WHEEL rotations:
  //   kV is volts per (wheel rotation / sec), kS is volts, kP/kD act on rotation error.
  // kS/kV come from the measured rad-based gains above (kVnative = kVrad * 2*pi).
  // kP is still a starting guess; tune it if the robot is slow to recover speed.
  public static final double driveKrakenKp = 0.5;
  public static final double driveKrakenKd = 0.0;
  public static final double driveKrakenKs = driveKs;
  public static final double driveKrakenKv = driveKv * 2.0 * Math.PI;

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
  public static final double robotMassKg = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> 22.6796;
    case Constants.ROBOT_COMP -> 40;
    default -> 40;
  };
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
