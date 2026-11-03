package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of spindexer IO. */
public class SpindexerIOSim implements SpindexerIO {
  private static final double MOI_KG_METERS_SQ = 0.001; // a light roller

  private final DCMotorSim sim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(spindexerGearbox, MOI_KG_METERS_SQ, spindexerMotorReduction),
      spindexerGearbox);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    // Update simulation state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    // Update inputs
    inputs.connected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.positionRad = sim.getAngularPositionRad();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
