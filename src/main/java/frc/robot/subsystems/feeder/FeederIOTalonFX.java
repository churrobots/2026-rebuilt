package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Feeder IO for a Kraken X60 on a TalonFX. The TalonFX runs the speed control itself. */
public class FeederIOTalonFX implements FeederIO {
  private final TalonFX talon = new TalonFX(feederCanId);

  // Status signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  // Control requests (reused to avoid per-loop allocation)
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private final Debouncer connectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FeederIOTalonFX() {
    // Coast, like the other rollers.
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        feederInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = feederMotorReduction;
    config.CurrentLimits.StatorCurrentLimit = feederCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);
    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    // TalonFX works in rotations per second
    talon.setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }
}
