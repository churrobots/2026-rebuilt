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
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PatchedTalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Shooter extends SubsystemBase {
  private static final String MOTOR_TELEMETRY = "ShooterMotor";
  private static final String MECHANISM_TELEMETRY = "Shooter";

  // Stable physical constants
  private static final int GEAR_STAGE_1 = 3;
  private static final int GEAR_STAGE_2 = 4;
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
  private static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
  private static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
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
          ControlsConstants.SHOOTER_KP, ControlsConstants.SHOOTER_KI, ControlsConstants.SHOOTER_KD,
          ControlsConstants.SHOOTER_MAX_VEL, ControlsConstants.SHOOTER_MAX_ACCEL)
      .withSimClosedLoopController(
          SIM_KP, SIM_KI, SIM_KD,
          SIM_MAX_VEL, SIM_MAX_ACCEL)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(
          ControlsConstants.SHOOTER_KS, ControlsConstants.SHOOTER_KV, ControlsConstants.SHOOTER_KA))
      .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
      // Telemetry name and verbosity level
      .withTelemetry(MOTOR_TELEMETRY, TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
      // corresponds to the gearbox attached to your motor.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_STAGE_1, GEAR_STAGE_2)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
      .withClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE)
      .withOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);

  // Vendor motor controller object
  private TalonFX motor = new TalonFX(HardwareConstants.SHOOTER_MOTOR_ID);

  // Create our SmartMotorController wrapping the TalonFX.
  private SmartMotorController controller = new PatchedTalonFXWrapper(motor, DCMotor.getFalcon500(1),
      motorConfig);

  private FlyWheelConfig shooterConfig = new FlyWheelConfig(controller)
      // Diameter of the flywheel.
      .withDiameter(DIAMETER)
      // Mass of the flywheel.
      .withMass(MASS)
      // Maximum speed of the shooter.
      .withUpperSoftLimit(UPPER_SOFT_LIMIT)
      // Telemetry name and verbosity for the mechanism.
      .withTelemetry(MECHANISM_TELEMETRY, TelemetryVerbosity.HIGH);

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig);

  /** Creates a new Shooter. */
  public Shooter() {
    setDefaultCommand(setVelocity(ControlsConstants.SHOOTER_DEFAULT_VELOCITY));
  }

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }
}
