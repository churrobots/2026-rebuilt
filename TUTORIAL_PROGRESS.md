# Tutorial progress
Student: (not given yet)   Current phase: 2 – First mechanism, the intake roller   Current step: 3

## Log
- 2026-10-07: Started Guided Tutorial Mode. Skipped making a personal branch (step 2) at student's request; still working on `fresh-start`.
- 2026-10-07: Logged robot speed (`Tutorial/Drive/SpeedMetersPerSec`) in `Drive.periodic()` (student wrote it). Claude added the speed readout to the Field card.
- 2026-10-07: Made branch `madtown` from `fresh-start` and made the first commit. Phase 0 done.
- 2026-10-07: Phase 1 steps 1–4: traced stick → wheels, default commands, deadband/squaring/negation, field-relative, the 20 ms loop, IO layers. Skipped the step 5 exercise.
- 2026-10-07: Phase 1 step 6: B button press counter (onTrue) and held light (whileTrue) in RobotContainer, plus a "B Button" dashboard card (Claude wrote the code at the student's request).
- 2026-10-07: Phase 1 done (the student skipped the final stick-to-wheels explanation).
- 2026-10-07: Phase 2 step 3: wrote `subsystems/intake/IntakeRollerIO.java` (4 inputs + setVoltage). Next: IntakeRollerIOSim.
- Hardware question to verify: on `main`, the intake roller uses a **TalonFX** (CAN ID 20, gearing 1, inverted, coast, 60 A stator limit), but YAMS was given `DCMotor.getNEO(1)`. Probably a copy-paste mistake. Check which motor is really on the new robot before choosing the sim model.

## Where we differ from the reference (on purpose)
- Claude writes dashboard code (coach request, 2026-10-07): students focus on Java.
