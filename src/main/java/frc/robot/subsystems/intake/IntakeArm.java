package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
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
  // A fixed test voltage while a test button is held. null = not testing.
  private Double manualVolts = null;

  public IntakeArm(IntakeArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeArm", inputs);

    double volts = 0.0;
    if (manualVolts != null) {
      volts = manualVolts;
    } else if (targetRad != null) {
      // Feedforward holds the arm up against gravity; PID fixes whatever error is left.
      // ArmFeedforward wants "angle from level, up = bigger," and pushes UP with positive
      // volts. Ours is the other way around, so flip the angle and the volts.
      double gravityVolts = -feedforward.calculate(armLevelRad - targetRad, 0.0);
      double pidVolts = pid.calculate(inputs.positionRad, targetRad);
      volts = gravityVolts + pidVolts;
    }

    // Safety: stop if the encoder is gone, or (unless we're testing by hand) the reading
    // doesn't make sense. A bad reading must never turn into full power.
    boolean readingOk = inputs.positionRad >= safeMinRad && inputs.positionRad <= safeMaxRad;
    boolean safetyStop = !inputs.connected || (manualVolts == null && !readingOk);
    if (safetyStop) {
      volts = 0.0;
    }
    io.setVoltage(MathUtil.clamp(volts, -maxVolts, maxVolts));

    Logger.recordOutput("Tutorial/Arm/SafetyStop", safetyStop);
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

  /**
   * Bring-up test: pushes with a fixed voltage only while held, then lets go (0 V, no target).
   * Positive volts should make the angle BIGGER (arm goes down). If not, flip armInverted.
   */
  public Command testVolts(double volts) {
    return startEnd(
        () -> {
          targetRad = null;
          manualVolts = volts;
        },
        () -> manualVolts = null);
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

  /**
   * Swings the arm back and forth between near-extended and near-stowed while it runs, pushing
   * leftover balls up into the spindexer (like the 2026 robot did while shooting).
   *
   * <p>The start time is saved when the command STARTS. (The 2026 version saved it when the
   * command was created at robot boot, so its "elapsed time" was already huge on the first shot.)
   */
  public Command pulse() {
    double[] startTime = {0.0};
    return startRun(
        () -> startTime[0] = Timer.getFPGATimestamp(),
        () -> targetRad = pulseAngle(Timer.getFPGATimestamp() - startTime[0]));
  }

  /**
   * Where the pulse should be, this many seconds in. A sine wave goes smoothly out and back;
   * stretching it a little and then clamping it makes short flat spots at each end.
   */
  private static double pulseAngle(double seconds) {
    double wave = (Math.sin(2 * Math.PI * seconds / pulsePeriodSeconds) + 1) / 2; // 0 to 1
    double stretched = (wave - 0.5) * (1 + pulseFlatPercent) + 0.5;
    double fraction = MathUtil.clamp(stretched, 0.0, 1.0);
    return pulseInRad + (pulseOutRad - pulseInRad) * fraction;
  }
}
