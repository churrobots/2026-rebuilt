package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionRad = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SpindexerIOInputs inputs) {
  }

  /** Run the spindexer at the given voltage. */
  public default void setVoltage(double volts) {
  }
}
