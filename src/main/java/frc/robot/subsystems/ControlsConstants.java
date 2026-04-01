// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ControlsConstants {

  // TODO: reduce this for competition
  public static final TelemetryVerbosity YAMS_VERBOSITY = HardwareConstants.REDUCE_ROBORIO_RESOURCE_USAGE
      ? TelemetryVerbosity.LOW
      : TelemetryVerbosity.HIGH;

  // ========== ClimberTW ==========
  public static final double CLIMBER_KP = 1;
  public static final double CLIMBER_KI = 0;
  public static final double CLIMBER_KD = 0;
  public static final LinearVelocity CLIMBER_MAX_VEL = MetersPerSecond.of(0.1);
  public static final LinearAcceleration CLIMBER_MAX_ACCEL = MetersPerSecondPerSecond.of(0.1);
  public static final double CLIMBER_KS = 0;
  public static final double CLIMBER_KG = 0;
  public static final double CLIMBER_KV = 0;
  public static final Distance CLIMBER_DEFAULT_HEIGHT = Meters.of(0);

  // ========== Feeder ==========
  public static final double FEEDER_KP = .25;
  public static final double FEEDER_KI = 0;
  public static final double FEEDER_KD = 0;
  public static final double FEEDER_DEFAULT_DUTY_CYCLE = 0;
  public static final double FEEDER_KV = 0.111;

  // ========== IntakeRoller ==========
  public static final double INTAKE_ROLLER_KP = 0.8;
  public static final double INTAKE_ROLLER_KI = 0;
  public static final double INTAKE_ROLLER_KD = 0;
  public static final double INTAKE_ROLLER_DEFAULT_DUTY_CYCLE = 0;
  public static final double INTAKE_ROLLER_KV = 0.113;

  // ========== Shooter ==========
  public static final double SHOOTER_KP = 0.3;
  public static final double SHOOTER_KI = 0;
  public static final double SHOOTER_KD = 0;
  public static final double SHOOTER_DEFAULT_DUTY_CYCLE = 0;
  public static final double SHOOTER_KV = 0.112;
  public static final double FEEDER_TO_SHOOTER_RPM_RATIO = 6000.0 / 3400.0;

  // ========== Spindexer ==========
  public static final double SPINDEXER_KP = 0.3;
  public static final double SPINDEXER_KI = 0;
  public static final double SPINDEXER_KD = 0;
  public static final double SPINDEXER_DEFAULT_DUTY_CYCLE = 0;
  public static final double SPINDEXER_KV = 0.9;

  // ========== Controller Binding Constants ==========
  public static final AngularVelocity SPINDEXER_VELOCITY = RPM.of(1500); // was 600

  // Math for minimum RPM required, which surface speed should be 2x robot speed
  // Drivetrain maxes at 4.6 m/s --> rollers need to run 2x = 9.2 m/s
  // Rollers are 3" = 0.076m diameter x PI --> 0.2388m circumference
  // Rollers therefore give 0.2388m / rotation
  // Target Roller RPM = (9.2 m/sec DIVIDED BY 0.2388 m/rot) = 38.5 rot/sec
  // 38.5 rot/sec x 60 sec/minute = 2310 rot/min (RPM)
  public static final AngularVelocity INTAKE_ROLLER_VELOCITY = RPM.of(2500);

  public static final AngularVelocity SHOOTER_VELOCITY = RPM.of(4550);
  public static final AngularVelocity SHOOTER_IDLE_VELOCITY = RPM.of(400);
}
