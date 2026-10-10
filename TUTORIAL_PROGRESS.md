# Tutorial progress
Student: (not given yet)   Current phase: 2 – First mechanism, the intake roller   Current step: 7

## Log
- 2026-10-07: Started Guided Tutorial Mode. Skipped making a personal branch (step 2) at student's request; still working on `fresh-start`.
- 2026-10-07: Logged robot speed (`Tutorial/Drive/SpeedMetersPerSec`) in `Drive.periodic()` (student wrote it). Claude added the speed readout to the Field card.
- 2026-10-07: Made branch `madtown` from `fresh-start` and made the first commit. Phase 0 done.
- 2026-10-07: Phase 1 steps 1–4: traced stick → wheels, default commands, deadband/squaring/negation, field-relative, the 20 ms loop, IO layers. Skipped the step 5 exercise.
- 2026-10-07: Phase 1 step 6: B button press counter (onTrue) and held light (whileTrue) in RobotContainer, plus a "B Button" dashboard card (Claude wrote the code at the student's request).
- 2026-10-07: Phase 1 done (the student skipped the final stick-to-wheels explanation).
- 2026-10-07: Phase 2 step 3: wrote `subsystems/intake/IntakeRollerIO.java` (4 inputs + setVoltage). Next: IntakeRollerIOSim.
- 2026-10-10: New session. Coaches reordered the plan (vision and autos now come sooner). Phase 2 itself didn't change. Resuming at IntakeRollerIOSim.
- 2026-10-10: Claude wrote `IntakeRollerIOSim.java` at the student's request (placeholder Kraken X60, 1:1, MOI 0.001). 
- Hardware question to verify: on `main`, the intake roller uses a **TalonFX** (CAN ID 20, gearing 1, inverted, coast, 60 A stator limit), but YAMS was given `DCMotor.getNEO(1)`. Probably a copy-paste mistake. Check which motor is really on the new robot before choosing the sim model.
  - 2026-10-10: Student says the new robot's intake roller is "still TalonFX." Student checked: the motor is a **Falcon 500** (TalonFX controller). Use `DCMotor.getFalcon500(1)` in the sim.
- 2026-10-10: Phase 2 step 4: Claude wrote `IntakeConstants.java` (motor, reduction, CAN ID 20, inverted, 60 A stator, the last three marked TODO: verify in Phase 3) and `IntakeRollerIOTalonFX.java` (coast, VoltageOut). The sim now reads its motor and gearing from IntakeConstants. 
- 2026-10-10: Phase 2 step 5: Claude wrote `IntakeRoller.java` (updateInputs + processInputs in periodic) and wired up REAL/SIM/replay IO in RobotContainer at the student's request. 
- 2026-10-10: Phase 2 step 6: intake()/outtake()/stop() commands (open-loop, ±6 V, worked out from the 2026 robot's 2900 RPM target and kV 0.113). Stop is the default command. Left trigger = intake, left bumper = outtake. Claude added the "Intake" dashboard card (volts, RPM, spinning light) = the Phase 2 dashboard milestone. Tested in sim: 3209 RPM at 6 V. Committed. Next: steps 7–8 discussion (open-loop C18, YAMS vs. IO C27).

## Where we differ from the reference (on purpose)
- Claude writes dashboard code (coach request, 2026-10-07): students focus on Java.
