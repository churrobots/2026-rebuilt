package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The intake arm: swings the intake out to the floor and back in. It goes to exact
 * angles and holds them against gravity using feedforward (kG) plus PID (kP, kD).
 * Angles are in radians: 0 = stowed, bigger = lower (see IntakeArmConstants).
 */
public class IntakeArm extends SubsystemBase {
  private final IntakeArmIO io;
  private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

  private final PIDController pid = new PIDController(kP, 0.0, kD);
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

  // Where we're trying to go, in radians. null = no target yet, so don't push.
  private Double targetRad = null;

  public IntakeArm(IntakeArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeArm", inputs);

    if (targetRad != null) {
      // Feedforward holds the arm up against gravity; PID fixes whatever error is left.
      // ArmFeedforward wants "angle from level, up = bigger," and pushes UP with positive
      // volts. Ours is the other way around, so flip the angle and the volts.
      double gravityVolts = -feedforward.calculate(armLevelRad - targetRad, 0.0);
      double pidVolts = pid.calculate(inputs.positionRad, targetRad);
      io.setVoltage(gravityVolts + pidVolts);
    }

    // For the dashboard. TargetRad is NaN ("not a number") while there's no target yet.
    Logger.recordOutput("Tutorial/Arm/MeasuredRad", inputs.positionRad);
    Logger.recordOutput("Tutorial/Arm/TargetRad", targetRad == null ? Double.NaN : targetRad);
    Logger.recordOutput("Tutorial/Arm/AtTarget", isAtTarget());
    Logger.recordOutput("Tutorial/Arm/LevelRad", armLevelRad);
  }

  /** True when the arm is within the tolerance of its target. */
  public boolean isAtTarget() {
    return targetRad != null && Math.abs(inputs.positionRad - targetRad) < atTargetToleranceRad;
  }

  /**
   * Moves the arm to an angle (radians) and holds it there. The target is saved when the
   * command RUNS, not when it's created, so isAtTarget() always checks the right target.
   */
  public Command goToAngle(double radians) {
    double clamped = MathUtil.clamp(radians, softMinRad, softMaxRad);
    return run(() -> targetRad = clamped);
  }

  /** Swings the intake down onto the floor. */
  public Command extend() {
    return goToAngle(extendedRad);
  }

  /** Brings the intake to its resting spot. */
  public Command retract() {
    return goToAngle(retractedRad);
  }

  /** Tucks the intake all the way in. */
  public Command stow() {
    return goToAngle(stowedRad);
  }
}
