package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The intake roller: one motor that grabs game pieces off the floor. */
public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);
  }

  /** Spins the roller inward to grab game pieces. Stops it when the command ends. */
  public Command intake() {
    return runEnd(() -> io.setVoltage(rollerIntakeVolts), () -> io.setVoltage(0.0));
  }

  /** Spins the roller outward to spit game pieces out. Stops it when the command ends. */
  public Command outtake() {
    return runEnd(() -> io.setVoltage(rollerOuttakeVolts), () -> io.setVoltage(0.0));
  }

  /** Stops the roller. */
  public Command stop() {
    return run(() -> io.setVoltage(0.0));
  }
}
