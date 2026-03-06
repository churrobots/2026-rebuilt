package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.Pair;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * A fully self-contained {@link SmartMotorController} implementation for use when no physical motor
 * controller is connected. All methods are safe no-ops that track minimal internal state.
 */
public class DisconnectedMotorController extends SmartMotorController {

  private final DCMotor dcMotor;
  private double dutyCycle = 0.0;
  private double voltage = 0.0;
  private double position = 0.0;   // rotations
  private double velocity = 0.0;   // rotations per second
  private double linearPosition = 0.0; // meters
  private double linearVelocity = 0.0; // meters per second

  /**
   * Create a {@link DisconnectedMotorController}.
   *
   * @param motor  {@link DCMotor} that this controller would drive.
   * @param config {@link SmartMotorControllerConfig} for configuration.
   */
  public DisconnectedMotorController(DCMotor motor, SmartMotorControllerConfig config) {
    this.dcMotor = motor;
    this.m_config = config;
    applyConfig(config);
  }

  @Override
  public void setupSimulation() {
  }

  @Override
  public void seedRelativeEncoder() {
  }

  @Override
  public void synchronizeRelativeEncoder() {
  }

  @Override
  public void simIterate() {
  }

  @Override
  public void setIdleMode(MotorMode mode) {
  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity) {
    this.velocity = velocity.in(RotationsPerSecond);
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity) {
    this.linearVelocity = velocity.in(MetersPerSecond);
  }

  @Override
  public void setEncoderPosition(Angle angle) {
    this.position = angle.in(Rotations);
  }

  @Override
  public void setEncoderPosition(Distance distance) {
    this.linearPosition = distance.in(Meters);
  }

  @Override
  public void setPosition(Angle angle) {
    setpointPosition = Optional.of(angle);
    setpointVelocity = Optional.empty();
  }

  @Override
  public void setPosition(Distance distance) {
    setpointPosition = Optional.of(m_config.convertToMechanism(distance));
    setpointVelocity = Optional.empty();
  }

  @Override
  public void setVelocity(LinearVelocity velocity) {
    setpointVelocity = Optional.of(m_config.convertToMechanism(velocity));
    setpointPosition = Optional.empty();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    setpointVelocity = Optional.of(velocity);
    setpointPosition = Optional.empty();
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config) {
    this.m_config = config;
    return true;
  }

  @Override
  public double getDutyCycle() {
    return dutyCycle;
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    this.dutyCycle = dutyCycle;
    this.voltage = dutyCycle * 12.0;
  }

  @Override
  public Optional<Current> getSupplyCurrent() {
    return Optional.of(Amps.zero());
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.zero();
  }

  @Override
  public Voltage getVoltage() {
    return Volts.of(voltage);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    this.voltage = voltage.in(Volts);
    this.dutyCycle = this.voltage / 12.0;
  }

  @Override
  public DCMotor getDCMotor() {
    return dcMotor;
  }

  @Override
  public LinearVelocity getMeasurementVelocity() {
    return MetersPerSecond.of(linearVelocity);
  }

  @Override
  public Distance getMeasurementPosition() {
    return Meters.of(linearPosition);
  }

  @Override
  public AngularVelocity getMechanismVelocity() {
    return RotationsPerSecond.of(velocity);
  }

  @Override
  public Angle getMechanismPosition() {
    return Rotations.of(position);
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    return RotationsPerSecond.of(velocity);
  }

  @Override
  public Angle getRotorPosition() {
    return Rotations.of(position);
  }

  @Override
  public void setMotorInverted(boolean inverted) {
  }

  @Override
  public void setEncoderInverted(boolean inverted) {
  }

  @Override
  public void setMotionProfileMaxVelocity(LinearVelocity maxVelocity) {
  }

  @Override
  public void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration) {
  }

  @Override
  public void setMotionProfileMaxVelocity(AngularVelocity maxVelocity) {
  }

  @Override
  public void setMotionProfileMaxAcceleration(AngularAcceleration maxAcceleration) {
  }

  @Override
  public void setMotionProfileMaxJerk(Velocity<AngularAccelerationUnit> maxJerk) {
  }

  @Override
  public void setExponentialProfile(OptionalDouble kV, OptionalDouble kA, Optional<Voltage> maxInput) {
  }

  @Override
  public void setKp(double kP) {
  }

  @Override
  public void setKi(double kI) {
  }

  @Override
  public void setKd(double kD) {
  }

  @Override
  public void setFeedback(double kP, double kI, double kD) {
  }

  @Override
  public void setKs(double kS) {
  }

  @Override
  public void setKv(double kV) {
  }

  @Override
  public void setKa(double kA) {
  }

  @Override
  public void setKg(double kG) {
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA, double kG) {
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit) {
  }

  @Override
  public void setSupplyCurrentLimit(Current currentLimit) {
  }

  @Override
  public void setClosedLoopRampRate(Time rampRate) {
  }

  @Override
  public void setOpenLoopRampRate(Time rampRate) {
  }

  @Override
  public void setMeasurementUpperLimit(Distance upperLimit) {
  }

  @Override
  public void setMeasurementLowerLimit(Distance lowerLimit) {
  }

  @Override
  public void setMechanismUpperLimit(Angle upperLimit) {
  }

  @Override
  public void setMechanismLowerLimit(Angle lowerLimit) {
  }

  @Override
  public Temperature getTemperature() {
    return Celsius.of(25);
  }

  @Override
  public SmartMotorControllerConfig getConfig() {
    return m_config;
  }

  @Override
  public Object getMotorController() {
    return null;
  }

  @Override
  public Object getMotorControllerConfig() {
    return null;
  }

  @Override
  public Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>> getUnsupportedTelemetryFields() {
    return new Pair<>(Optional.empty(), Optional.empty());
  }

  @Override
  public String getName() {
    return m_config.getTelemetryName().orElse("Disconnected");
  }
}
