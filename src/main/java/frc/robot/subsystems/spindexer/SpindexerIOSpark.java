package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/** Spindexer IO implementation for a NEO on a SPARK MAX. */
public class SpindexerIOSpark implements SpindexerIO {
  private final SparkMax spark = new SparkMax(spindexerCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = spark.getEncoder();
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public SpindexerIOSpark() {
    var config = new SparkMaxConfig();
    config
        .inverted(spindexerInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(spindexerCurrentLimit)
        .voltageCompensation(12.0);
    // Turn motor rotations into spindexer radians (and motor RPM into spindexer
    // rad/sec)
    config.encoder
        .positionConversionFactor(2 * Math.PI / spindexerMotorReduction)
        .velocityConversionFactor(2 * Math.PI / 60.0 / spindexerMotorReduction);
    tryUntilOk(
        spark,
        5,
        () -> spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(spark, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] { spark::getAppliedOutput, spark::getBusVoltage },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }
}