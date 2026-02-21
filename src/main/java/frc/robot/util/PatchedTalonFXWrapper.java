package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * We discovered a bug in YAMS that does an infinite do...while loop when
 * setting position. We should submit a patch upstream at some point, but
 * this will at least allow the robot to boot up if a TalonFX (e.g.
 * Falcon500) has CAN issues
 */
public class PatchedTalonFXWrapper extends TalonFXWrapper {

  private TalonFX m_controller;

  public PatchedTalonFXWrapper(TalonFX controller, DCMotor motor, SmartMotorControllerConfig smartConfig) {
    super(controller, motor, smartConfig);
    m_controller = controller;
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config) {
    if (config.getStartingPosition().isPresent()) {
      DriverStation.reportError(
          "ALERT: No motor configs applied for TalonFX CAN ID = " + m_controller.getDeviceID()
              + " - startingPosition is dangerous in YAMS TalonFX as of February 21, 2026 (ask Matt and Joel about it)",
          true);
      return false;
    }
    return super.applyConfig(config);
  }
}
