// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FaultMonitor;
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

public class Feeder extends SubsystemBase {
  private static final String MOTOR_TELEMETRY = "FeederMotor";
  private static final String MECHANISM_TELEMETRY = "Feeder";

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

  private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(ControlsConstants.FEEDER_KP, ControlsConstants.FEEDER_KI, ControlsConstants.FEEDER_KD)
      .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
      // Telemetry name and verbosity level
      .withTelemetry(MOTOR_TELEMETRY, TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
      // corresponds to the gearbox attached to your motor.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_STAGE_1, GEAR_STAGE_2)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);

  // Vendor motor controller object
  private TalonFX motor = new TalonFX(HardwareConstants.FEEDER_MOTOR_ID);

  // Create our SmartMotorController wrapping the TalonFX.
  private SmartMotorController controller = new PatchedTalonFXWrapper(motor, DCMotor.getFalcon500(1),
      motorConfig);

  private FlyWheelConfig feederConfig = new FlyWheelConfig(controller)
      // Diameter of the flywheel.
      .withDiameter(DIAMETER)
      // Mass of the flywheel.
      .withMass(MASS)
      // Maximum speed of the feeder.
      .withUpperSoftLimit(UPPER_SOFT_LIMIT)
      // Telemetry name and verbosity for the mechanism.
      .withTelemetry(MECHANISM_TELEMETRY, TelemetryVerbosity.HIGH);

  // Feeder Mechanism
  private FlyWheel feeder = new FlyWheel(feederConfig);

  /** Creates a new Feeder. */
  public Feeder() {
    setDefaultCommand(set(ControlsConstants.FEEDER_DEFAULT_DUTY_CYCLE));
  }

  /**
   * Gets the current velocity of the intake.
   *
   * @return Feeder velocity.
   */
  public AngularVelocity getVelocity() {
    return feeder.getSpeed();
  }

  /**
   * Set the intake velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return feeder.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the feeder.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return feeder.set(dutyCycle);
  }

  @Override
  public void periodic() {
    feeder.updateTelemetry();
    boolean hasFaults = FaultMonitor.hasAnyDisconnectsOrFaults(motor);
    SmartDashboard.putBoolean("feederfaults", hasFaults);
  }

  @Override
  public void simulationPeriodic() {
    feeder.simIterate();
  }
}
