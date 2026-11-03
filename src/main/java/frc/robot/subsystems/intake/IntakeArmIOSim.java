package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Physics sim implementation of intake arm IO. Gravity is on, so the arm droops.
 *
 * <p>WPILib's arm sim measures angles from LEVEL, with up = bigger. Our angles start at
 * stowed and get bigger going DOWN. So this file flips between the two: sim angle =
 * armLevelRad - our angle. It also flips the volts, since positive volts lower our arm.
 */
public class IntakeArmIOSim implements IntakeArmIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          armGearbox,
          armMotorReduction,
          SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg),
          armLengthMeters,
          armLevelRad - hardMaxRad, // our lowest spot, in sim angles
          armLevelRad - hardMinRad, // our highest spot, in sim angles
          true, // simulate gravity
          armLevelRad - stowedRad); // start tucked in

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    // Update simulation state (positive volts lower our arm = lower the sim angle)
    sim.setInputVoltage(-MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    // Update inputs, converted back to our angles
    inputs.connected = true;
    inputs.positionRad = armLevelRad - sim.getAngleRads();
    inputs.velocityRadPerSec = -sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
