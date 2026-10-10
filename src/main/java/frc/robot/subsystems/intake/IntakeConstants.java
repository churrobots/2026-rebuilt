package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {
  // Roller motor: Falcon 500 (TalonFX), verified on robot 2026-10-10
  public static final DCMotor rollerGearbox = DCMotor.getFalcon500(1);
  public static final double rollerMotorReduction = 1.0;

  // TODO: verify on the new robot in Phase 3 (values from the 2026 robot on main)
  public static final int rollerCanId = 20;
  public static final boolean rollerInverted = true; // positive volts = intake
  public static final int rollerCurrentLimit = 60; // stator amps

  // Open-loop voltages. The 2026 robot aimed for 2900 RPM; with its kV of
  // 0.113 V per rot/s, that's about 48 rot/s x 0.113 = 5.5 V. Tune in sim.
  public static final double rollerIntakeVolts = 6.0;
  public static final double rollerOuttakeVolts = -6.0;
}
