// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FaultMonitor;
import frc.robot.util.PatchedTalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ClimberTW extends SubsystemBase {
  private static final String MOTOR_TELEMETRY = "ElevatorMotor";
  private static final String MECHANISM_TELEMETRY = "Elevator";

  // Stable physical constants
  private static final Distance MECHANISM_CIRCUMFERENCE = Meters.of(Inches.of(0.25).in(Meters) * 22);
  private static final double GEAR_REDUCTION = 10; // 10:1
  private static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
  private static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
  private static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
  private static final Distance STARTING_HEIGHT = Meters.of(0.5);
  private static final Distance HARD_LIMIT_LOW = Meters.of(0);
  private static final Distance HARD_LIMIT_HIGH = Meters.of(1);
  private static final Mass MASS = Pounds.of(16);

  // Sim constants
  private static final double SIM_KP = 4;
  private static final double SIM_KI = 0;
  private static final double SIM_KD = 0;
  private static final LinearVelocity SIM_MAX_VEL = MetersPerSecond.of(0.5);
  private static final LinearAcceleration SIM_MAX_ACCEL = MetersPerSecondPerSecond.of(0.5);
  private static final double SIM_KS = 0;
  private static final double SIM_KG = 0;
  private static final double SIM_KV = 0;

  private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Mechanism Circumference is the distance traveled by each mechanism rotation
      // converting rotations to meters.
      .withMechanismCircumference(MECHANISM_CIRCUMFERENCE)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
          ControlsConstants.CLIMBER_KP, ControlsConstants.CLIMBER_KI, ControlsConstants.CLIMBER_KD,
          ControlsConstants.CLIMBER_MAX_VEL, ControlsConstants.CLIMBER_MAX_ACCEL)
      .withSimClosedLoopController(
          SIM_KP, SIM_KI, SIM_KD,
          SIM_MAX_VEL, SIM_MAX_ACCEL)
      // Feedforward Constants
      .withFeedforward(new ElevatorFeedforward(
          ControlsConstants.CLIMBER_KS, ControlsConstants.CLIMBER_KG, ControlsConstants.CLIMBER_KV))
      .withSimFeedforward(new ElevatorFeedforward(SIM_KS, SIM_KG, SIM_KV))
      // Telemetry name and verbosity level
      .withTelemetry(MOTOR_TELEMETRY, TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_REDUCTION, 1)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
      .withClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE)
      .withOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);

  // private DigitalInput dio = new DigitalInput(0); // Standard DIO
  // private final Sensor climbSensor = new SensorConfig("switchgoclickclick") //
  // Name of the sensor
  // .withField("Beam", dio::get, false) // Add a Field to the sensor named "Beam"
  // whose value is dio.get() and
  // // defaults to false
  // .getSensor(); // Get the sensor.

  // Vendor motor controller object
  private TalonFX motor = new TalonFX(HardwareConstants.CLIMBER_MOTOR_ID);

  // Create our SmartMotorController wrapping the TalonFX.
  private SmartMotorController controller = new PatchedTalonFXWrapper(motor, DCMotor.getFalcon500(1),
      motorConfig);

  private ElevatorConfig elevatorConfig = new ElevatorConfig(controller)
      .withStartingHeight(STARTING_HEIGHT)
      .withHardLimits(HARD_LIMIT_LOW, HARD_LIMIT_HIGH)
      .withTelemetry(MECHANISM_TELEMETRY, TelemetryVerbosity.HIGH)
      .withMass(MASS);

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevatorConfig);

  /** Creates a new ClimberTW. */
  public ClimberTW() {
    setDefaultCommand(setHeight(ControlsConstants.CLIMBER_DEFAULT_HEIGHT));
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Distance to go to.
   */
  public Command setHeight(Distance height) {
    return elevator.setHeight(height);
  }

  /**
   * Move the elevator up and down.
   *
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    return elevator.set(dutycycle);
  }

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() {
    return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
    boolean hasFaults = FaultMonitor.hasAnyDisconnectsOrFaults(motor);
    SmartDashboard.putBoolean("climberfaults", hasFaults);
  }
}
