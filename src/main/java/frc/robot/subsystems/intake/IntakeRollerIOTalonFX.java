package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

/** Intake roller IO implementation for a Falcon 500 on a TalonFX. */
public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX talon = new TalonFX(rollerCanId);

  // Status signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  // Control request (reused to avoid per-loop allocation)
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Connection debouncer
  private final Debouncer connectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeRollerIOTalonFX() {
    // Configure motor. Coast so the roller spins down freely when stopped.
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        rollerInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = rollerMotorReduction;
    config.CurrentLimits.StatorCurrentLimit = rollerCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Grab and rate-limit the status signals. optimizeBusUtilization()
    // disables everything not given a frequency, so set them first.
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);
    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }
}
