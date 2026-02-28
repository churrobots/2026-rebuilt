// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FaultMonitor;
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

public class Spindexer extends SubsystemBase {
  private static final String MOTOR_TELEMETRY = "SpindexerMotor";
  private static final String MECHANISM_TELEMETRY = "Spindexer";

  // Stable physical constants
  private static final int GEAR_STAGE_1 = 3;
  private static final int GEAR_STAGE_2 = 3;
  private static final int GEAR_STAGE_3 = 4;
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
  private static final Distance DIAMETER = Inches.of(17.5);
  private static final Mass MASS = Pounds.of(1);
  private static final AngularVelocity UPPER_SOFT_LIMIT = RPM.of(150);

  // Sim constants
  private static final double SIM_KP = 0.5;
  private static final double SIM_KI = 0;
  private static final double SIM_KD = 0;

  private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(ControlsConstants.SPINDEXER_KP, ControlsConstants.SPINDEXER_KI,
          ControlsConstants.SPINDEXER_KD)
      .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
      // TODO: figure out why SparkBase doesn't like feedforward values?
      // Feedforward Constants
      // .withFeedforward(new SimpleMotorFeedforward(0, 9.17, 0))
      // .withSimFeedforward(new SimpleMotorFeedforward(0, 9.17, 0))
      // Telemetry name and verbosity level
      .withTelemetry(MOTOR_TELEMETRY, TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_STAGE_1, GEAR_STAGE_2, GEAR_STAGE_3)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);

  // Vendor motor controller object
  private SparkMax motor = new SparkMax(HardwareConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);
  private SmartMotorController controller = new SparkWrapper(motor, DCMotor.getNEO(1),
      motorConfig);

  private final FlyWheelConfig spindexerConfig = new FlyWheelConfig(controller)
      // Diameter of the flywheel.
      .withDiameter(DIAMETER)
      // Mass of the flywheel.
      .withMass(MASS)
      // Maximum speed of the spindexer.
      .withUpperSoftLimit(UPPER_SOFT_LIMIT)
      // Telemetry name and verbosity for the mechanism.
      .withTelemetry(MECHANISM_TELEMETRY, TelemetryVerbosity.HIGH);

  // Spindexer Mechanism
  private FlyWheel spindexer = new FlyWheel(spindexerConfig);

  /** Creates a new Spindexer. */
  public Spindexer() {
    setDefaultCommand(set(ControlsConstants.SPINDEXER_DEFAULT_DUTY_CYCLE));
  }

  /**
   * Gets the current velocity of the spindexer.
   *
   * @return spindexer velocity.
   */
  public AngularVelocity getVelocity() {
    return spindexer.getSpeed();
  }

  /**
   * Set the spindexer velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return spindexer.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the spindexer.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return spindexer.set(dutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    spindexer.updateTelemetry();
    boolean hasFaults = FaultMonitor.hasAnyDisconnectsOrFaults(motor);
    SmartDashboard.putBoolean("spindexerFaults", hasFaults);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    spindexer.simIterate();
  }

}
