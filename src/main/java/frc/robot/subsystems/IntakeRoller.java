// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.HardwareMonitor;
import frc.robot.util.YAMSUtil;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class IntakeRoller extends SubsystemBase {
  private static final String MOTOR_TELEMETRY = "IntakeRollerMotor";
  private static final String MECHANISM_TELEMETRY = "IntakeRoller";

  // Stable physical constants
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
  private static final Distance DIAMETER = Inches.of(3);
  private static final Mass MASS = Pounds.of(0.50);
  private static final AngularVelocity UPPER_SOFT_LIMIT = RPM.of(1000);

  // Sim constants
  private static final double SIM_KP = 50;
  private static final double SIM_KI = 0;
  private static final double SIM_KD = 0;

  private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withFeedforward(new SimpleMotorFeedforward(0, ControlsConstants.INTAKE_ROLLER_KV))
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
          ControlsConstants.INTAKE_ROLLER_KP, ControlsConstants.INTAKE_ROLLER_KI, ControlsConstants.INTAKE_ROLLER_KD)
      .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
      // Telemetry name and verbosity level
      .withTelemetry(MOTOR_TELEMETRY, ControlsConstants.YAMS_VERBOSITY)
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withGearing(1)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);

  // Vendor motor controller object
  private TalonFX motor = new TalonFX(HardwareConstants.INTAKE_ROLLERS_MOTOR_ID);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController controller = YAMSUtil.safeGetSmartMotorController(motor, DCMotor.getNEO(1),
      motorConfig);

  private final FlyWheelConfig rollerConfig = new FlyWheelConfig(controller)
      // Diameter of the flywheel.
      .withDiameter(DIAMETER)
      // Mass of the flywheel.
      .withMass(MASS)
      // Maximum speed of the intake.
      .withUpperSoftLimit(UPPER_SOFT_LIMIT)
      // Telemetry name and verbosity for the mechanism.
      .withTelemetry(MECHANISM_TELEMETRY, ControlsConstants.YAMS_VERBOSITY);

  // Intake Roller Mechanism
  private FlyWheel roller = new FlyWheel(rollerConfig);

  /** Creates a new IntakeRoller. */
  public IntakeRoller() {
    setDefaultCommand(stopFeeding());
    HardwareMonitor.registerHardware("intakeRollerMotor", motor);
  }

  /**
   * Gets the current velocity of the intake.
   *
   * @return IntakeRoller velocity.
   */
  public AngularVelocity getVelocity() {
    return roller.getSpeed();
  }

  /**
   * Set the intake velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return roller.setSpeed(speed);
  }

  public Command setVelocity(Supplier<AngularVelocity> speedSupplier) {
    return roller.setSpeed(speedSupplier);
  }

  public Command feedToSpindexer() {
    return roller.setSpeed(ControlsConstants.INTAKE_ROLLER_VELOCITY);
  }

  public Command feedToSpindexerFaster() {
    // @hannah: raised to 1.2 times after quals 7
    // @hannah: raised to 1.5 times after quals 23
    return roller.setSpeed(ControlsConstants.INTAKE_ROLLER_VELOCITY.times(1.5));
  }

  public Command stopFeeding() {
    return set(ControlsConstants.INTAKE_ROLLER_DEFAULT_DUTY_CYCLE);
  }

  /**
   * Set the dutycycle of the intake.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return roller.set(dutyCycle);
  }

  @Override
  public void periodic() {
    roller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    roller.simIterate();
  }
}
