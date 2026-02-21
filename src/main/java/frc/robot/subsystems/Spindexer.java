// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Spindexer extends SubsystemBase {

  /** Creates a new Spinnymabobthing. */
  public Spindexer() {
  }

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(1, 0, 0)
      .withSimClosedLoopController(1, 0, 0)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("SpindexerMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  // Vendor motor controller object
  private TalonFX talonFX = new TalonFX(14);
  private SmartMotorController sparkSmartMotorController = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1),
      smcConfig);

  private final FlyWheelConfig spindexerConfig = new FlyWheelConfig(sparkSmartMotorController)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      // Mass of the flywheel.
      .withMass(Pounds.of(1))
      // Maximum speed of the spindexer.
      .withUpperSoftLimit(RPM.of(1000))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Spindexer", TelemetryVerbosity.HIGH);

  // Spindexer Mechanism
  private FlyWheel spindexer = new FlyWheel(spindexerConfig);

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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    spindexer.simIterate();
  }

}
