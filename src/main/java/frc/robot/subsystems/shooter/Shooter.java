package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** The shooter: a flywheel that spins at an exact speed and flings game pieces out. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double targetRpm = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Tutorial/Shooter/TargetRPM", targetRpm);
    Logger.recordOutput("Tutorial/Shooter/MeasuredRPM", getRpm());
    Logger.recordOutput("Tutorial/Shooter/Ready", isReady());
  }

  /** How fast the flywheel is actually spinning, in RPM. */
  public double getRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /**
   * True when the flywheel is spinning for a shot (faster than idle) and is close enough to
   * its target speed, so a shot will go where we aim.
   */
  public boolean isReady() {
    return targetRpm > idleRpm && Math.abs(getRpm() - targetRpm) < readyToleranceRpm;
  }

  /** Spins the flywheel at a speed and keeps it there. */
  public Command spinAt(double rpm) {
    return spinAt(() -> rpm);
  }

  /** Spins the flywheel at a speed that can change while it runs (like from auto-aim). */
  public Command spinAt(DoubleSupplier rpm) {
    return run(
        () -> {
          targetRpm = rpm.getAsDouble();
          io.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(targetRpm));
        });
  }

  /** Spins up to shot speed. */
  public Command shoot() {
    return spinAt(shotRpm);
  }

  /** Keeps the flywheel rolling slowly so it's quicker to spin up. Good default command. */
  public Command idle() {
    return spinAt(idleRpm);
  }

  /** Lets the flywheel coast to a stop. */
  public Command stop() {
    return run(
        () -> {
          targetRpm = 0.0;
          io.setVoltage(0.0);
        });
  }
}
