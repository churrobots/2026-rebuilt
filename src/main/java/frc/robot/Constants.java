// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // This helps us differentiate multiple robots. Each robot stores
  // a persisted string in NetworkTables, so that we know which
  // robot we got deployed to, in case there are specific constants
  // that have different values between those robots.
  public static final String ROBOT_SIMULATION = "simulation";
  public static final String ROBOT_ALPHA = "alpha";
  public static final String ROBOT_BETA = "beta";
  public static final String robotName = NetworkTableInstance
      .getDefault()
      .getEntry("robotName")
      .getString(RobotBase.isSimulation() ? ROBOT_SIMULATION : ROBOT_BETA);

  public static final CalibrationMode calibrationMode = CalibrationMode.ENABLED;

  /** Calibrating AdvantageKit Constants **/
  public static enum CalibrationMode {
    /** Not in calibration mode. */
    DISABLED,

    /** In calibration mode. */
    ENABLED
  }
}
