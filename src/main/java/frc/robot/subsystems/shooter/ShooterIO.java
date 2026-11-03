package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Spin the flywheel at this speed. The motor controller keeps it there. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Run the flywheel at the given voltage (0 = let it coast to a stop). */
  public default void setVoltage(double volts) {}
}
