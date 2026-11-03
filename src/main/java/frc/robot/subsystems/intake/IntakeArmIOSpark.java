package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeArmConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/** Intake arm IO implementation for a NEO on a SPARK MAX with an absolute encoder. */
public class IntakeArmIOSpark implements IntakeArmIO {
  private final SparkMax spark = new SparkMax(armCanId, MotorType.kBrushless);
  private final AbsoluteEncoder encoder = spark.getAbsoluteEncoder();
  private final Debouncer connectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeArmIOSpark() {
    var config = new SparkMaxConfig();
    config
        .inverted(armInverted)
        .idleMode(IdleMode.kCoast) // 2026 used coast. TODO: consider brake to hold the arm when disabled
        .smartCurrentLimit(armCurrentLimit)
        .voltageCompensation(12.0);
    // The absolute encoder turns with the arm: 1 rotation = 2*pi radians.
    // TODO: confirm the encoder is on the arm itself (1:1), not on the motor
    config.absoluteEncoder
        .inverted(armEncoderInverted)
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI / 60.0);
    tryUntilOk(
        spark,
        5,
        () -> spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    sparkStickyFault = false;
    // Shift so 0 = stowed, then wrap to -pi..pi so "a hair past stowed" reads as a small
    // negative angle instead of almost a full turn.
    ifOk(
        spark,
        encoder::getPosition,
        (value) -> inputs.positionRad = MathUtil.angleModulus(value - armEncoderOffsetRad));
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }
}
