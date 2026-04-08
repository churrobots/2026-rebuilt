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
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.HardwareMonitor;
import frc.robot.util.TunableNumber;
import frc.robot.util.YAMSUtil;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class Shooter extends SubsystemBase {
  private static final String MOTOR_TELEMETRY = "ShooterMotor";
  private static final String MECHANISM_TELEMETRY = "Shooter";

  public static TunableNumber TUNABLE_VOMIT_RPM = new TunableNumber("OUTTAKE_RPM",
      ControlsConstants.SHOOTER_OUTTAKE_VELOCITY.in(RPM));

  // Stable physical constants
  private static final double GEARING = 1.0;
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
  private static final Distance DIAMETER = Inches.of(4);
  private static final Mass MASS = Pounds.of(1);
  private static final AngularVelocity UPPER_SOFT_LIMIT = RPM.of(1000);

  // Sim constants
  private static final double SIM_KP = 50;
  private static final double SIM_KI = 0;
  private static final double SIM_KD = 0;

  private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withFeedforward(new SimpleMotorFeedforward(0, ControlsConstants.SHOOTER_KV, 0))
      // Feedback Constants (PID Constants)
      .withClosedLoopController(ControlsConstants.SHOOTER_KP, ControlsConstants.SHOOTER_KI,
          ControlsConstants.SHOOTER_KD)
      .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
      // Telemetry name and verbosity level
      // TODO: disable telemetry in competition mode?
      .withTelemetry(MOTOR_TELEMETRY, ControlsConstants.YAMS_VERBOSITY)
      .withGearing(GEARING)
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT);

  // Vendor motor controller object
  private TalonFX motor = new TalonFX(HardwareConstants.SHOOTER_MOTOR_ID);

  // Create our SmartMotorController wrapping the TalonFX.
  private SmartMotorController controller = YAMSUtil.safeGetSmartMotorController(motor, DCMotor.getFalcon500(1),
      motorConfig);

  private FlyWheelConfig shooterConfig = new FlyWheelConfig(controller)
      // Diameter of the flywheel.
      .withDiameter(DIAMETER)
      // Mass of the flywheel.
      .withMass(MASS)
      // Maximum speed of the shooter.
      .withUpperSoftLimit(UPPER_SOFT_LIMIT)
      // Telemetry name and verbosity for the mechanism.
      .withTelemetry(MECHANISM_TELEMETRY, ControlsConstants.YAMS_VERBOSITY);

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig);

  /** Creates a new Shooter. */
  public Shooter() {
    setDefaultCommand(setVelocityFixed(ControlsConstants.SHOOTER_IDLE_VELOCITY));
    HardwareMonitor.registerHardware("shooterMotor", motor);
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
  public Command setVelocityFixed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return shooter.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
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
