package frc.robot.util;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class YAMSUtil {

  private static final int NUM_CONNECTION_RETRIES = 5;
  private static final Time RETRY_DELAY = Milliseconds.of(10);

  public static final SmartMotorController createSmartMotorController(
      TalonFX controller,
      DCMotor motor,
      SmartMotorControllerConfig smartConfig) {
    if (_isConnected(controller)) {
      return new PatchedTalonFXWrapper(controller, motor, smartConfig);
    } else {
      DriverStation.reportError(
          "MECHANISMS MAY NOT FUNCTION: Could not find TalonFX (e.g. Falcon 500) with CAN ID="
              + controller.getDeviceID(),
          false);
      return new DisconnectedMotorController(motor, smartConfig);
    }
  }

  public static final SmartMotorController createSmartMotorController(
      SparkBase controller,
      DCMotor motor,
      SmartMotorControllerConfig smartConfig) {
    if (_isConnected(controller)) {
      return new SparkWrapper(controller, motor, smartConfig);
    } else {
      DriverStation.reportError(
          "MECHANISMS MAY NOT FUNCTION: Could not find Spark (e.g. NEO motor) with CAN ID=" + controller.getDeviceId(),
          false);
      return new DisconnectedMotorController(motor, smartConfig);
    }
  }

  private static boolean _isConnected(TalonFX controller) {
    if (controller == null) {
      return false;
    }

    for (int i = 0; i < NUM_CONNECTION_RETRIES; i++) {
      if (controller.isConnected()) {
        return true;
      }
      Timer.delay(RETRY_DELAY.in(Seconds));
    }
    return false;
  }

  private static boolean _isConnected(SparkBase controller) {
    if (controller == null) {
      return false;
    }

    for (int i = 0; i < NUM_CONNECTION_RETRIES; i++) {
      int version = controller.getFirmwareVersion();
      boolean isConnected = (version != 0);
      if (isConnected) {
        return true;
      }
      Timer.delay(RETRY_DELAY.in(Seconds));
    }
    return false;
  }

}
