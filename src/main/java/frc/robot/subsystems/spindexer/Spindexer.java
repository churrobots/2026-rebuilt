package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The spindexer: the spinning hopper floor that carries game pieces to the shooter. */
public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  /** Spins the spindexer toward the shooter. */
  public Command feed() {
    return run(() -> io.setVoltage(spindexerFeedVolts));
  }

  /** Spins the spindexer backwards to free a stuck piece. */
  public Command unjam() {
    return run(() -> io.setVoltage(spindexerUnjamVolts));
  }

  /** Stops the spindexer. */
  public Command stop() {
    return run(() -> io.setVoltage(0.0));
  }
}
