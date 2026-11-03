package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The feeder: pushes each game piece into the spinning flywheel. */
public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  /** Pushes game pieces into the flywheel. */
  public Command feed() {
    return run(() -> io.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(feedRpm)));
  }

  /** Stops the feeder. */
  public Command stop() {
    return run(() -> io.setVoltage(0.0));
  }
}
