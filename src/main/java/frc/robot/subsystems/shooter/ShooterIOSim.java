package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Physics sim implementation of shooter IO. A real TalonFX runs the speed control itself,
 * so this fakes that: volts = kV * target speed + kP * error (same gains as the TalonFX).
 */
public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              shooterGearbox, flywheelMoiKgMetersSq, shooterMotorReduction),
          shooterGearbox);

  // The gains are per rotation/sec; the sim works in radians/sec, so divide by 2*pi.
  private final PIDController pid = new PIDController(kP / (2 * Math.PI), 0.0, 0.0);

  private boolean closedLoop = false;
  private double targetRadPerSec = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Run the fake "motor controller" speed control
    if (closedLoop) {
      appliedVolts =
          kS * Math.signum(targetRadPerSec)
              + (kV / (2 * Math.PI)) * targetRadPerSec
              + pid.calculate(sim.getAngularVelocityRadPerSec(), targetRadPerSec);
    } else {
      pid.reset();
    }

    // Update simulation state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    // Update inputs
    inputs.connected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    closedLoop = true;
    targetRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }
}
