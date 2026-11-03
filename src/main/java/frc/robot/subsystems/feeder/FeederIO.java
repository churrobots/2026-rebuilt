package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Spin the feeder at this speed. The motor controller keeps it there. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Run the feeder at the given voltage (0 = let it coast to a stop). */
  public default void setVoltage(double volts) {}
}
