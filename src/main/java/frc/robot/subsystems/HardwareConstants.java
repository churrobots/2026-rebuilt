// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

public class HardwareConstants {
  public static final int FEEDER_MOTOR_ID = 10;
  public static final int SPINDEXER_MOTOR_ID = 14;
  public static final int SHOOTER_MOTOR_ID = 15;
  public static final int INTAKE_ARM_MOTOR_ID = 11;
  public static final int INTAKE_ROLLERS_MOTOR_ID = 20;
  public static final int CLIMBER_MOTOR_ID = 10;

  public static final boolean HAS_SPINDEXER = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> false;
    case Constants.ROBOT_COMP -> true;
    default -> true;
  };

  public static final boolean HAS_SHOOTER = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> false;
    case Constants.ROBOT_COMP -> true;
    default -> true;
  };
  public static final boolean HAS_FEEDER = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> false;
    case Constants.ROBOT_COMP -> true;
    default -> true;
  };

  public static final boolean HAS_INTAKE_ARM = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> false;
    case Constants.ROBOT_COMP -> false;
    default -> true;
  };

  public static final boolean HAS_INTAKE_ROLLER = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> false;
    case Constants.ROBOT_COMP -> false;
    default -> true;
  };

  public static final boolean HAS_CLIMBER = switch (Constants.robotName) {
    case Constants.ROBOT_ALPHA -> true;
    case Constants.ROBOT_COMP -> false;
    default -> true;
  };
}
