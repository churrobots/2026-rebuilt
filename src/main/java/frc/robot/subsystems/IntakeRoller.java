// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeRoller extends SubsystemBase {
  private static final String MOTOR_TELEMETRY = "IntakeRollerMotor";
  private static final String MECHANISM_TELEMETRY = "IntakeRoller";

  // Stable physical constants
  private static final int GEAR_STAGE_1 = 3;
  private static final int GEAR_STAGE_2 = 4;
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
  private static final Distance DIAMETER = Inches.of(4);
  private static final Mass MASS = Pounds.of(1);
  private static final AngularVelocity UPPER_SOFT_LIMIT = RPM.of(1000);

  // Sim constants
  private static final double SIM_KP = 50;
  private static final double SIM_KI = 0;
  private static final double SIM_KD = 0;
  private static final AngularVelocity SIM_MAX_VEL = DegreesPerSecond.of(90);
  private static final AngularAcceleration SIM_MAX_ACCEL = DegreesPerSecondPerSecond.of(45);
  private static final double SIM_KS = 0;
  private static final double SIM_KV = 0;
  private static final double SIM_KA = 0;

  private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
          ControlsConstants.INTAKE_ROLLER_KP, ControlsConstants.INTAKE_ROLLER_KI, ControlsConstants.INTAKE_ROLLER_KD,
          ControlsConstants.INTAKE_ROLLER_MAX_VEL, ControlsConstants.INTAKE_ROLLER_MAX_ACCEL)
      .withSimClosedLoopController(
          SIM_KP, SIM_KI, SIM_KD,
          SIM_MAX_VEL, SIM_MAX_ACCEL)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(
          ControlsConstants.INTAKE_ROLLER_KS, ControlsConstants.INTAKE_ROLLER_KV, ControlsConstants.INTAKE_ROLLER_KA))
      .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
      // Telemetry name and verbosity level
      .withTelemetry(MOTOR_TELEMETRY, TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_STAGE_1, GEAR_STAGE_2)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);

  // Vendor motor controller object
  private SparkMax motor = new SparkMax(HardwareConstants.INTAKE_ROLLERS_MOTOR_ID, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController controller = new SparkWrapper(motor, DCMotor.getNEO(1),
      motorConfig);

  private final FlyWheelConfig rollerConfig = new FlyWheelConfig(controller)
      // Diameter of the flywheel.
      .withDiameter(DIAMETER)
      // Mass of the flywheel.
      .withMass(MASS)
      // Maximum speed of the intake.
      .withUpperSoftLimit(UPPER_SOFT_LIMIT)
      // Telemetry name and verbosity for the mechanism.
      .withTelemetry(MECHANISM_TELEMETRY, TelemetryVerbosity.HIGH);

  // Intake Roller Mechanism
  private FlyWheel roller = new FlyWheel(rollerConfig);

  /** Creates a new IntakeRoller. */
  public IntakeRoller() {
    setDefaultCommand(set(ControlsConstants.INTAKE_ROLLER_DEFAULT_DUTY_CYCLE));
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
