// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.HardwareMonitor;

public class SpindexerWithoutYAMS extends SubsystemBase {
  private static final double GEARING = 36.0;
  private static final int STATOR_CURRENT_LIMIT = 40;

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;

  /** Creates a new Spindexer. */
  public SpindexerWithoutYAMS() {
    motor = new SparkMax(HardwareConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();

    var config = new SparkMaxConfig();
    config
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(STATOR_CURRENT_LIMIT);
    config.encoder
        .velocityConversionFactor(1.0 / GEARING);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ControlsConstants.SPINDEXER_KP, ControlsConstants.SPINDEXER_KI,
            ControlsConstants.SPINDEXER_KD);
    // config.signals.setSetpointAlwaysOn(true);
    tryUntilOk(
        motor,
        5,
        () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    setDefaultCommand(set(ControlsConstants.SPINDEXER_DEFAULT_DUTY_CYCLE));
    HardwareMonitor.registerHardware("spindexerMotor", motor);
  }

  /**
   * Gets the current velocity of the spindexer.
   *
   * @return spindexer velocity.
   */
  public AngularVelocity getVelocity() {
    return Units.RPM.of(encoder.getVelocity());
  }

  /**
   * Set the spindexer velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return run(() -> {
      double rpm = speed.in(Units.RPM);
      double ffVolts = ControlsConstants.SPINDEXER_KV * rpm;
      closedLoopController.setSetpoint(
          rpm,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          ffVolts,
          ArbFFUnits.kVoltage);
    });
  }

  /**
   * Set the spindexer velocity.
   *
   * @param speedSupplier Supplier for the speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(Supplier<AngularVelocity> speedSupplier) {
    return run(() -> {
      double rpm = speedSupplier.get().in(Units.RPM);
      double ffVolts = ControlsConstants.SPINDEXER_KV * rpm;
      closedLoopController.setSetpoint(
          rpm,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          ffVolts,
          ArbFFUnits.kVoltage);
    });
  }

  /**
   * Set the dutycycle of the spindexer.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return run(() -> motor.set(dutyCycle));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("spindexer_setpoint", closedLoopController.getSetpoint());
    SmartDashboard.putNumber("spindexer_setpointVelocity", closedLoopController.getMAXMotionSetpointVelocity());
    SmartDashboard.putNumber("spindexer_actualVelocity", encoder.getVelocity());
  }
}