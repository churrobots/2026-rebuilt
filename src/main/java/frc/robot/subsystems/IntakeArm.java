// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeArm extends SubsystemBase {
  private static final String MOTOR_TELEMETRY = "IntakeArmMotor";
  private static final String MECHANISM_TELEMETRY = "IntakeArm";

  // Stable physical constants
  private static final int GEAR_STAGE_1 = 3;
  private static final int GEAR_STAGE_2 = 4;
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
  private static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.1);
  private static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
  private static final Angle SOFT_LIMIT_LOW = Degrees.of(-90);
  private static final Angle SOFT_LIMIT_HIGH = Degrees.of(90);
  private static final Angle HARD_LIMIT_LOW = Degrees.of(-100);
  private static final Angle HARD_LIMIT_HIGH = Degrees.of(100);
  private static final Angle STARTING_POSITION = Degrees.of(0);
  private static final Distance LENGTH = Feet.of(1);
  private static final Mass MASS = Pounds.of(5);

  // Sim constants
  private static final double SIM_KP = 4;
  private static final double SIM_KI = 0;
  private static final double SIM_KD = 0.2;
  private static final AngularVelocity SIM_MAX_VEL = DegreesPerSecond.of(180);
  private static final AngularAcceleration SIM_MAX_ACCEL = DegreesPerSecondPerSecond.of(90);
  private static final double SIM_KS = 0;
  private static final double SIM_KG = 1.302;
  private static final double SIM_KV = 0;
  private static final double SIM_KA = 0;

  private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
          ControlsConstants.INTAKE_ARM_KP, ControlsConstants.INTAKE_ARM_KI, ControlsConstants.INTAKE_ARM_KD,
          ControlsConstants.INTAKE_ARM_MAX_VEL, ControlsConstants.INTAKE_ARM_MAX_ACCEL)
      .withSimClosedLoopController(
          SIM_KP, SIM_KI, SIM_KD,
          SIM_MAX_VEL, SIM_MAX_ACCEL)
      // Feedforward Constants
      .withFeedforward(new ArmFeedforward(
          ControlsConstants.INTAKE_ARM_KS, ControlsConstants.INTAKE_ARM_KG, ControlsConstants.INTAKE_ARM_KV))
      .withSimFeedforward(new ArmFeedforward(SIM_KS, SIM_KG, SIM_KV, SIM_KA))
      // Telemetry name and verbosity level
      .withTelemetry(MOTOR_TELEMETRY, TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_STAGE_1, GEAR_STAGE_2)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
      .withClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE)
      .withOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);

  // Vendor motor controller object
  private SparkMax motor = new SparkMax(HardwareConstants.INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController controller = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

  private ArmConfig armConfig = new ArmConfig(controller)
      // Soft limit is applied to the SmartMotorControllers PID
      .withSoftLimits(SOFT_LIMIT_LOW, SOFT_LIMIT_HIGH)
      // Hard limit is applied to the simulation.
      .withHardLimit(HARD_LIMIT_LOW, HARD_LIMIT_HIGH)
      // Starting position is where your arm starts
      .withStartingPosition(STARTING_POSITION)
      // Length and mass of your arm for sim.
      .withLength(LENGTH)
      .withMass(MASS)
      // Telemetry name and verbosity for the arm.
      .withTelemetry(MECHANISM_TELEMETRY, TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm arm = new Arm(armConfig);

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    setDefaultCommand(setAngle(ControlsConstants.INTAKE_ARM_DEFAULT_ANGLE));
  }

  /**
   * Set the angle of the arm.
   *
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  /**
   * Move the arm up and down.
   *
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) {
    return arm.set(dutycycle);
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // YAMS automatically applies the sim feedforward (including kG * cos(angle))
    // when computing motor voltage in closed-loop mode. No manual FF needed.
    arm.simIterate();
  }
}
