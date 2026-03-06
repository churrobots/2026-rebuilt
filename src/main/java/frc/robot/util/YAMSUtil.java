package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class YAMSUtil {

  public static final SmartMotorController createSmartMotorController(
      TalonFX controller,
      DCMotor motor,
      SmartMotorControllerConfig smartConfig) {
    if (controller != null) {
      return new PatchedTalonFXWrapper(controller, motor, smartConfig);
    } else {
      return new DisconnectedMotorController(motor, smartConfig);
    }
  }

  public static final SmartMotorController createSmartMotorController(
      SparkBase controller,
      DCMotor motor,
      SmartMotorControllerConfig smartConfig) {
    if (controller != null) {
      return new SparkWrapper(controller, motor, smartConfig);
    } else {
      return new DisconnectedMotorController(motor, smartConfig);
    }
  }

}
