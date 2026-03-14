// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
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
  public static final double INTAKE_ROLLER_KP = 2.1;
  public static final double INTAKE_ROLLER_KI = 0;
  public static final double INTAKE_ROLLER_KD = 0;
  public static final double INTAKE_ROLLER_DEFAULT_DUTY_CYCLE = 0;
  public static final double INTAKE_ROLLER_KV = 0.118;

  // ========== Shooter ==========
  public static final double SHOOTER_KP = 0.3;
  public static final double SHOOTER_KI = 0;
  public static final double SHOOTER_KD = 0;
  public static final double SHOOTER_DEFAULT_DUTY_CYCLE = 0;
  public static final double SHOOTER_KV = 0.112;
  public static final double FEEDER_TO_SHOOTER_RPM_RATIO = 3500.0 / 3000.0;

  // ========== Spindexer ==========
  public static final double SPINDEXER_KP = 0.3;
  public static final double SPINDEXER_KI = 0;
  public static final double SPINDEXER_KD = 0;
  public static final double SPINDEXER_DEFAULT_DUTY_CYCLE = 0;
  public static final double SPINDEXER_KV = 0.9;

  // ========== Controller Binding Constants ==========
  public static final AngularVelocity SPINDEXER_VELOCITY = RPM.of(800);
  public static final AngularVelocity INTAKE_ROLLER_VELOCITY = RPM.of(3000);
  public static final AngularVelocity FEEDER_VELOCITY = RPM.of(5000);
  public static final AngularVelocity SHOOTER_VELOCITY = RPM.of(4550);
  public static final AngularVelocity SHOOTER_IDLE_VELOCITY = RPM.of(400);
}
