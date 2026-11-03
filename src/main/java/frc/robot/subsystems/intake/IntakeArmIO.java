package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
  @AutoLog
  public static class IntakeArmIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0; // arm angle, from the absolute encoder
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeArmIOInputs inputs) {}

  /** Run the arm motor at the given voltage. */
  public default void setVoltage(double volts) {}
}
