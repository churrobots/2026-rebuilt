# Guided Tutorial Plan: Build Our Robot From Scratch

This is the roadmap for building our **new robot** starting from a bare AdvantageKit swerve template (the `fresh-start` branch): a **Kraken swerve drivetrain** with a **turret** shooter. The AI assistant follows it in **Guided Tutorial Mode** (see `AGENTS.md`), and a student or coach can follow it by hand.

- Concept IDs like **C19** refer to [CONCEPTS.md](CONCEPTS.md).
- "Reference" means our existing code on the reference branches listed in `AGENTS.md`. Peek at a file with `git show <branch>:<path>`. The drivetrain reference is `unleash-the-kraken`, the turret reference is `churret` (unfinished), and most other mechanisms come from the 2026 competition robot on `main`. If the new robot's mechanisms differ, measure the real robot. The phases still apply.
- Every phase ends with a **dashboard milestone**, so you can *see* the new feature working at `http://localhost:5800`.
- 🧑‍🏫 marks a **coach checkpoint**: stop and get a coach before continuing.
- Always get each feature working in **simulation first**, then on the real robot.
- **Every mechanism is built with AdvantageKit IO layers (C12, C27), not YAMS.** It's the same pattern the drivetrain already uses (`ModuleIO` → `ModuleIOKraken` / `ModuleIOSim`). Each mechanism gets an `XxxIO` interface with `@AutoLog` inputs, a real-hardware IO class, and an `XxxIOSim` built on WPILib's physics simulators (`DCMotorSim`, `FlywheelSim`, `SingleJointedArmSim`, `ElevatorSim`).
  - The reference mechanisms on `main` and `churret` use YAMS. Take **hardware numbers** from them (CAN IDs, gear ratios, limits, starting gains), not their structure.
  - For structure, use our drivetrain IO files first, then public AdvantageKit mechanism examples: the AdvantageKit docs (https://docs.advantagekit.org, "IO interfaces"), team 6328 Mechanical Advantage's public robot code (https://github.com/Mechanical-Advantage), and other teams' AdvantageKit code found on chiefdelphi.com. When a mechanism step starts, look up a matching example (an AdvantageKit roller, arm, flywheel, turret, or elevator) and tell the student where it came from.

Your code does not have to match the reference line-for-line. If it works and you can explain why, that's a win.

---

## Phase 0: Drive the robot in simulation

**Goal:** Within the first session, the student is driving our swerve robot around a simulated field with a real controller and watching it on the dashboard.
**Concepts:** C02, C05, C41, C42, C13 (lightly; details come in Phase 1)
**Zero to Robot:** Step 2 (`zero-to-robot/step-2/wpilib-setup.html`) for installing WPILib

1. Install WPILib 2026 and open the project in WPILib VS Code. (The vendor tools, Phoenix Tuner X and the REV Hardware Client, can wait until Phase 3.)
2. Check out the tutorial starting branch (`fresh-start`) and make your own branch from it (C05). It's the **AdvantageKit Spark Swerve template** already adapted for our Kraken MAXSwerve modules (`ModuleIOKraken`). It also includes the team's tooling: the browser dashboard, `SimSupervisor`, the Sim Driver Station bridge, and every vendor library we use (Phoenix 6, REVLib, PhotonVision, PathPlanner, and so on). There are no mechanisms yet. You'll build those.
3. Plug an Xbox-style controller into the laptop. Press ⌘⇧B (Ctrl+Shift+B on Windows) and pick **ChurroBots: Develop and Simulate** to start the simulator, then do the same with **ChurroBots: Open Simulation Robot Dashboard** to open `http://localhost:5800`.
4. In the dashboard's Sim Driver Station, check that the controller shows as connected, then enable **teleop** and **drive**. The template already has the controls:
   - left stick moves the robot
   - right stick turns it
   - hold A to face 0°
   - press X to lock the wheels in an X
5. Watch the robot move on the dashboard's **Field** card. It's drawn from the driver's point of view: your alliance wall is at the bottom, and it flips when you switch alliance in the Sim Driver Station. Explain that this dashboard is where they'll see the results of everything they build from here on (C42), and that they can change anything about it.
6. **First code edit (robot):** in `Drive.periodic()`, log the robot's speed as a simple number, for example `Logger.recordOutput("Tutorial/Drive/SpeedMetersPerSec", ...)` using `getChassisSpeeds()`. Save, and watch the sim restart by itself (C41, C13).
7. **First code edit (dashboard):** show that speed on the Field card, next to the pose readout. Then let them change something just for fun, like the robot's color or size in `drawRobot`.
8. Make your first commit (C05).

**Dashboard milestone:** The Field card shows the robot moving live, plus the student's own speed readout.
**Done when:** The student can drive in sim with the controller, and the robot moves on the Field card the way they expect (pushing forward drives away from your alliance wall, C04).

---

## Phase 1: How that just worked

**Goal:** Understand the code behind what the student just did, so they can change it.
**Concepts:** C02, C03, C04, C06–C11, C12, C13, C25, C28, C45

1. **Follow the stick press through the code:** controller → `RobotContainer`'s default command → `DriveCommands.joystickDrive` → `Drive.runVelocity` → each module. Along the way, explain the command-based pieces: subsystem, command, default command, and trigger (C06–C11).
2. In `joystickDrive`, explain the deadband, input squaring, and why Y is negated (C28). Explain field-relative driving (C25): spin the robot in sim and push forward. It still goes "up the field."
3. Walk through `Robot.java`: the modes, `robotPeriodic`, and the command scheduler running every 20 ms (C02).
4. Explain IO layers using `ModuleIO` / `ModuleIOKraken` / `ModuleIOSim` (C12). `RobotContainer` picks Kraken hardware on the real robot and physics simulation on a laptop, and `Drive.java` never knows the difference. That's why they could drive without a robot. Point out that every mechanism they build from here on will follow this same pattern.
5. **Exercise:** Change the deadband or max speed, save, and *feel* the difference with the controller.
6. **Exercise:** Bind a new button to a `Commands.runOnce(...)` that logs a message. Try `onTrue` vs. `whileTrue` and watch the difference on the dashboard (C08).

**Dashboard milestone:** Add the button exercise's value (for example a press counter, or "held/not held") to the dashboard.
**Done when:** The student can explain, in their own words, the path from stick to wheels, and what a command and a subsystem are.

---

## Phase 2: First mechanism, the intake roller (in simulation)

**Goal:** Write your first AdvantageKit IO layer from scratch, for the simplest mechanism there is: one motor spinning a roller. Then watch it spin in simulation.
**Concepts:** C06, C07, C09, C12, C13, C15, C16, C18, C27, C34, C41, C45
**Reference:** structure: `drive/ModuleIO.java`, `ModuleIOSim.java`, `ModuleIOKraken.java`, plus a public AdvantageKit roller example; hardware values: `main`: `subsystems/IntakeRoller.java`, `ControlsConstants.java` (see the roller RPM math comment)

1. Show what the intake does on the real robot, or in a match video. The roller is the part that grabs game pieces.
2. **Design the IO layer on paper first (C12).** What does the robot need to *read* (inputs: connected, velocity, applied volts, current) and what does it need to *do* (outputs: `setVoltage`)? Compare with `ModuleIO`, and with a public AdvantageKit roller example.
3. Write `IntakeRollerIO` (the interface, with an `@AutoLog` inputs class and do-nothing default methods) and `IntakeRollerIOSim`, using WPILib's `DCMotorSim` with the right `DCMotor` model and gear ratio.
4. Write the real-hardware IO (`IntakeRollerIOSpark` or `IntakeRollerIOTalonFX`, depending on the motor; check the reference). Put the CAN ID, inversion, idle mode, and current limit in constants (C15, C16, C45). It can't be tested until Phase 3, but writing it now shows that only this file knows about the vendor library.
5. Create the `IntakeRoller` subsystem: `periodic()` calls `io.updateInputs(...)` and `Logger.processInputs(...)` (C13). In `RobotContainer`, pick the real or sim IO the same way `Drive` does.
6. Add `intake()` and `outtake()` **commands**, and a default command that stops the roller (C07, C09). Bind intake to the left trigger (`whileTrue`) and outtake to the left bumper.
7. Discuss why open-loop voltage control means the speed changes with battery voltage and game pieces, and how the team chose its target RPM (C18, C34). Keep it open-loop for now. Closed-loop comes in Phase 7.
8. Discuss C27: the 2026 robot used a library (YAMS) for mechanisms. What do we gain by writing the IO layer ourselves, and what does it cost?

**Dashboard milestone:** An "Intake" card: the commanded volts, the measured RPM, and a spinning/stopped light. The arm joins this card in Phase 6.
**Done when:** Holding the trigger spins the roller on the dashboard in sim, and the student can explain which file would have to change to run it on different hardware (only the IO class), and why `RobotContainer` doesn't touch the motor directly.

---

## Phase 3: moved (see below Phase 5)

**Moved:** do this phase after Phase 5 step 3. It now sits right after Phase 5 below.

---

## Phase 4: Vision

**Goal:** Use AprilTags to know exactly where the robot is on the field. Everything after this gets a more accurate pose, and later the turret needs a good pose to aim.
**Concepts:** C29, C30, C31, C12, C26
**Reference:** `main`: `subsystems/vision/` (AdvantageKit vision template), `VisionConstants.java`

1. Explain AprilTags and why odometry drifts (C26, C29).
2. Add the AdvantageKit `Vision` subsystem with `VisionIOPhotonVisionSim` for the new robot's cameras. Wire it to `drive::addVisionMeasurement`.
3. Measure and enter each robot-to-camera transform on the new robot (C30). Discuss the old robot's lesson: "all cameras are flipped on this robot."
4. In sim, drive around and watch accepted vs. rejected vision poses in AdvantageScope. Explain the standard deviations and rejection rules in `Vision.java`.
5. 🧑‍🏫 Real robot (after Phase 3 step 7 has the coprocessors mounted and streaming): configure the PhotonVision pipelines, and check the pose against a tape-measured position on the field.

**Dashboard milestone:** A "Vision" card showing, per camera: connected, number of tags seen, and whether the last pose was accepted.
**Done when:** Pushing the robot by hand (or "slipping" it in sim) is corrected by vision within about a second.

---

## Phase 5: Autonomous, part 1: follow a path

**Goal:** The robot drives a PathPlanner path all by itself. No mechanisms yet, just the drivetrain, with vision keeping the pose accurate.
**Concepts:** C32 (paths and the auto chooser; named commands come in Phase 9)
**Reference:** `main`: `getAutonomousCommand()`, `deploy/pathplanner/`

1. Install PathPlanner and configure the robot settings (mass, module config with the Kraken X60, and so on).
2. Draw a simple "drive out of the starting zone" path and run it in sim.
3. Pick it from the auto chooser and run it once in sim. Watch it work on the Field card. (The template's chooser also has the characterization autos that Phase 3 uses.)
4. 🧑‍🏫 After Phase 3: test the path on the real field at slow speed first.

**Dashboard milestone:** Show the selected auto name and an auto timer. Stretch: draw the planned path on the field view.
**Done when:** The drive-out path runs in sim and you've watched it work. **Next: Phase 3** (real robot).

---

## Phase 3: Drivetrain on the real robot (done after Phase 5)

**When:** After Phase 5 step 3. The robot wasn't ready earlier, so vision and autos were built in sim first (coach reorder, 2026-10-16). The spindexer is built here too, before the bench test, so it can be checked on the Throne along with the drivetrain (coach reorder, 2026-10-16).
**Goal:** Everything that worked in sim now works on carpet with the Krakens. Along the way, build the spindexer (your second roller) so it's ready for the bench test.
**Concepts:** C01, C04, C12, C15, C17, C22, C24, C26, C30, C34, C44, C45
**Reference:** `unleash-the-kraken`: `drive/ModuleIOKraken.java`, `drive/DriveConstants.java` (the Kraken section), `util/PhoenixUtil.java`; spindexer hardware values: `main`: `subsystems/Spindexer.java`, `HardwareConstants.java`, `ControlsConstants.java` (YAMS: take its numbers, not its structure)
**Zero to Robot:** Step 2 (`frc-game-tools.html`) and Step 4 (`running-test-program.html`)

1. Install the FRC Game Tools (Driver Station) on Windows, plus **Phoenix Tuner X** and the **REV Hardware Client**. Tour the control system hardware on the robot with a coach: roboRIO, radio, PDH, CAN chain, and motor controllers (C01).
2. **Real CAN IDs from the orange sticky note.** Put the drivetrain's CAN IDs (drive motors, turn motors, gyro) into `DriveConstants`. They start with the old robot's values. One ID on the note is the **spindexer**: write it down in `TUTORIAL_PROGRESS.md` now, and use it when you build the spindexer in step 5. While you're in `DriveConstants`, go over the other numbers too (track width and wheelbase, wheel radius, drive gearing, current limits, inversions): *where does each value come from?* Note that `driveGearbox = DCMotor.getKrakenX60(1)` is what makes the simulation model a Kraken (C17).
3. **Check the robot still runs in sim.** CAN IDs don't change the simulation, but a typo can break the build. Drive around on the dashboard like in Phase 0.
4. **The IO layer lesson (C12).** Read through `ModuleIOKraken` together. The team wrote it to swap the template's NEO Vortex drive motors for Krakens:
   - **Drive:** a TalonFX using Phoenix 6: `TalonFXConfiguration` for inversion, brake mode, stator and supply current limits, and `SensorToMechanismRatio` (so the closed loop works in *wheel* rotations). It uses `VelocityVoltage` and `VoltageOut` control requests, and status signals for position and velocity.
   - **Turn:** a SPARK MAX + absolute encoder, the same as the template.
   - Compare it with the template's original `ModuleIOSpark` (`git show main:src/main/java/frc/robot/subsystems/drive/ModuleIOSpark.java`). Point out that `Drive.java` didn't have to change at all.
5. **Build the spindexer, in sim first (C34).** The spindexer is the spinning floor of the hopper: it carries game pieces around to the feeder. It's a roller, just like the intake roller, so the **student writes it** using the Phase 2 pattern, climbing the hint ladder instead of copying their intake files.
   - **Hardware facts first.** On `main` it's a SPARK MAX + NEO, 20:1, inverted, coast, 40 A stator limit (`git show main:src/main/java/frc/robot/subsystems/Spindexer.java`). Use the CAN ID from the sticky note (step 2), and confirm the motor and gearing on the new robot with mechanical. Don't guess. Mark anything unconfirmed `TODO: verify on the Throne`.
   - **Design on paper:** what's the same as the intake roller, and what's different? (Different motor controller, so a different vendor library in the real IO; much bigger gear reduction, so a much slower output.)
   - Write `SpindexerIO` (with `@AutoLog` inputs, including position so the dashboard can show it turning), `SpindexerIOSim` (`DCMotorSim`), the real IO (`SpindexerIOSpark` or `SpindexerIOTalonFX`, whichever is on the robot), and `SpindexerConstants` (C12, C15, C16).
   - Write the `Spindexer` subsystem with a stop default command and open-loop `feed()` (toward the shooter) and `unjam()` (backwards) commands (C07, C09, C18). The student picks unused buttons and binds them in `RobotContainer`. Closed-loop comes in Phase 7.
   - **C45 discussion:** the spindexer and intake roller IO look almost the same. Could they share one `RollerIO`? What would we gain, and what would get harder? Keep them separate for now unless the student wants to try it.
   - **Visualize it** (see the dashboard milestone). Before building the card, ask the student what they'd want to see to know it works, and let them design it.
   - Test in sim: holding the button spins the spindexer on the dashboard, the other button spins it backwards, and letting go stops it.
6. 🧑‍🏫 **Bench test on the Throne** (the robot up on blocks, wheels in the air) (C44). Deploy the code. Drive **forward, left, right, and back**, and rotate in place: the wheels should angle into a circle. Check the gyro direction too (counter-clockwise positive, C04). If a module points or spins the wrong way:
   - First confirm that Electrical calibrated the wheels (module zeros) properly.
   - If they did, check the turn motor offsets in the code (`frontLeftZeroRotation` and friends in `DriveConstants`).
   **Also check the spindexer:** it shows up on CAN, `feed()` turns it toward the shooter (flip the inversion in `SpindexerConstants` if not), and the measured speed on the dashboard matches what you see. This is the first time its real-hardware IO class runs.
   Then pick one of the **characterization autos** from the auto chooser and run it. On the Throne it just spins the wheels in the air, so ignore the numbers: the point is that the chooser and autos work on the real robot.
7. 🧑‍🏫 **Install vision.** Put the coprocessors in their mounts and power them, and mount the cameras on the base. From the driver station laptop, connect to **both** PhotonVision coprocessors and check that you can see **every** camera stream. (Measuring camera transforms and checking the pose against the tape measure is Phase 4 steps 3 and 5.)
8. 🧑‍🏫 **On the carpet.** Once everything works on the bench, put the robot on the carpet and drive it around. Make sure basic field-relative driving works and the dashboard pose follows the robot.
9. 🧑‍🏫 **Drivetrain characterization** on the carpet at Bloomhouse. Follow the AdvantageKit instructions for the template's characterization autos (wheel radius, then feedforward for the Kraken kS and kV) and update the code with the results as instructed (C22). Explain why the Kraken gains are in TalonFX units (volts per wheel rotation/second) and are measured, not converted from the old numbers. **Note:** we'll re-characterize after the superstructure is mounted, because it adds weight (see Phase 13).
10. Add a "reset heading" button (reference: `resetPoseFacingAway()` on `main`).
11. 🧑‍🏫 Test the intake roller from Phase 2 on the real robot: check the CAN ID, spin direction, and current limit. This is the first time its real-hardware IO class runs.

**Dashboard milestone:** A "Spindexer" card (step 5), designed with the student. Suggested: a top-down disc that turns with the spindexer's **measured** position (so it stops when the motor stops, and lags when it's loaded), with an arrow for direction, plus commanded volts, measured RPM, and a spinning/stopped light. Then the Field card tracks the *real* robot (connect the dashboard to the robot's address). Add a "module health" row showing each module's measured wheel angle, which makes a mis-zeroed module obvious on the Throne.
**Done when:** The spindexer spins both ways on the dashboard in sim and on the Throne, the robot drives field-relative smoothly on carpet, the dashboard pose tracks the real movement, both PhotonVision coprocessors show all camera streams, and the characterization values are in the code.

---

## Phase 6: The intake arm (position control)

**Goal:** An arm that goes to exact angles and holds them against gravity, built as your third IO layer (after the intake roller and spindexer). Then combine it with the roller into one intake action.
**Concepts:** C06, C07, C09, C10, C12, C15, C16, C17, C19, C20, C21, C36, C45
**Reference:** structure: your `IntakeRollerIO` from Phase 2, plus a public AdvantageKit arm example; hardware values: `main`: `subsystems/IntakeArm.java` (YAMS: take its CAN ID, gearing, limits, and starting gains, not its structure); `sandbox-sim`: `subsystems/MechanismVisualizer.java` (AdvantageScope visualization)

1. Show what the intake arm does on the real robot, or in a match video. Name its positions: stowed, retracted, and extended.
2. **Design the IO layer.** Start from the roller's IO: what's the same, and what's new? (Position in radians, and the absolute encoder.)
3. Write `IntakeArmIO`, `IntakeArmIOSim`, and the real-hardware IO. The sim uses WPILib's `SingleJointedArmSim` with the gear ratio, the arm's length and mass, the min/max angles, and gravity turned on, so the simulated arm droops like the real one. The real IO gets the SPARK MAX CAN ID, gear reduction, current limit, and absolute encoder. Put the numbers in constants (C16, C17, C45).
4. **Control in the subsystem.** `IntakeArm` uses a WPILib `PIDController` plus an `ArmFeedforward` to compute volts, then calls `io.setVoltage(...)`. That way the same control code runs in sim and on the robot. Clamp every target inside soft limits that sit inside the hard limits (C16). Mention the other option: running PID on the motor controller itself. What's the trade-off?
5. Add `extend()`, `retract()`, and `stow()` angle commands, with `retract()` as the default command (C07, C09). Bind them to the D-pad, since A and X are already used by the drivetrain.
6. Log the target angle, measured angle, and "at target" (for example `Tutorial/Arm/TargetDegrees`) (C13).
7. **Build the arm visualization on the dashboard** (see the milestone). Let the student design how it looks.
8. **Tuning lesson, the fun part (C19–C21).** All in sim, watching the dashboard. Change one gain at a time and save; the sim restarts in seconds:
   - All gains at 0 → the arm **droops** under gravity.
   - Add **kG** until it holds still wherever it is (C20).
   - Add **P** → it moves to the target, but too much P makes it **wobble** (C19).
   - Add a little **D** to calm the wobble.

   A live setpoint vs. measured graph makes this obvious.
9. **One intake action:** in `RobotContainer`, make the left trigger extend the arm *and* spin the roller together (`Commands.parallel`, C10). Neither subsystem reaches into the other.
10. Optional: show the same arm in AdvantageScope with a `LoggedMechanism2d` (C14).
11. 🧑‍🏫 Real robot: verify the absolute encoder direction and zero *before* closing the loop, start with low gains, then re-tune. Mention the reference's comment about measuring angles empirically (C17).

**Dashboard milestone:** Add the arm to the "Intake" card: a side-view drawing with a pivot point, a solid line for where the arm *is*, and a faint "ghost" line for where it's *trying to go*. Also show the two angles as numbers and an **AT TARGET** light, next to the roller's spinning light. Stretch: a small live graph of target vs. measured.
**Done when:** Pressing the D-pad swings the arm on the dashboard to each position without drooping or wobbling, one trigger pull extends the arm and runs the roller, and the student can explain what kG and P each do.

---

## Phase 7: Flywheel and indexing

**Goal:** A velocity-controlled flywheel and indexing, fed only when the flywheel is ready. (For now the turret stays still. It comes next.)
**Concepts:** C12, C18, C20, C21, C35, C34, C10, C37
**Reference:** structure: `drive/ModuleIOKraken.java` (TalonFX velocity control) plus a public AdvantageKit flywheel example; hardware values and gains: `main`: `Shooter.java` (YAMS `FlyWheel`), `Feeder.java`, `Spindexer.java`, `ControlsConstants.java`; command flow: `RobotContainer.autoShoot()`

1. Build `Shooter` with its own IO layer: `ShooterIO`, `ShooterIOTalonFX` (coast mode, current limit, Phoenix 6 `VelocityVoltage` running on the motor controller), and `ShooterIOSim` (WPILib `FlywheelSim` + a `PIDController`). It's the same split `ModuleIOKraken` / `ModuleIOSim` use for drive velocity. Idle speed is the default command. Explain why coast mode is used (C15).
2. Tune the flywheel: kV first (the feedforward does most of the work), then P for recovery after each shot (C21, `tuning-flywheel.html`).
3. Build `Feeder` (a velocity roller) with its own IO layer. Reuse what you learned writing the intake roller and spindexer. The `Spindexer` already exists from Phase 3 step 5: switch it and the intake roller to closed-loop too, by adding a `setVelocity(...)` to their IO (C18).
4. Compose the shoot command: spin up → wait until at speed → feed + index (C10, C37). Discuss why feeding early wastes shots.
5. Add `MechanismVisualizer` (from `sandbox-sim`) so AdvantageScope shows all the mechanisms moving together: arm, roller, and flywheel (C14).

**Dashboard milestone:** A "Shooter" card: target RPM, actual RPM, and a big **READY** light when within tolerance.
**Done when:** In sim, pressing shoot waits for READY before feeding, every time.

---

## Phase 8: The turret

**Goal:** A turret that moves safely to any angle in its range, then holds a *field-relative* direction while the robot spins.
**Concepts:** C46, C04, C12, C16, C17, C19, C20, C23, C45
**Reference:** `churret`: `subsystems/Churret.java`. It's an unfinished prototype built on YAMS, so read the "Turret prototype caveats" in `AGENTS.md` first, and take only its ideas and numbers. For structure, use your arm IO from Phase 6 plus a public AdvantageKit turret example.

1. 🧑‍🏫 **Measure the real turret with the mechanical team:** gear reduction, range of travel and hard stops, which way 0° points relative to the robot's front, and how the code will know the turret's position at power-on (absolute encoder vs. "always start centered") (C17, C46).
2. Build `Turret` with its own IO layer: `TurretIO`, the real-hardware IO (brake mode, a current limit), and `TurretIOSim` (WPILib `DCMotorSim`, with no gravity). Compare it with the arm's IO: what can be reused? In the subsystem, add soft limits *inside* the hard stops and a motion profile (max velocity and acceleration, using WPILib `TrapezoidProfile` or TalonFX Motion Magic, C23). Discuss why there's no kG (C20).
3. Add manual commands: face front, face left, and D-pad nudges. Tune in sim: kS for friction, then P, then adjust the profile (C21, `tuning-turret.html`).
4. **Angle math, on paper first:** `turretAngle = fieldAngleToTarget − robotHeading`, wrapped to the turret's range, with out-of-range targets going to the **nearest** limit. Put it in its own small method.
5. **Good practice:** write a JUnit test for that method using a few cases, including the out-of-range ones (C45). Then have the student find the clamp bug in `Churret.java` and explain why the test would catch it.
6. Add a "hold field direction" command: the turret keeps pointing at a fixed field angle while you spin the robot in sim.
7. Add the turret to `MechanismVisualizer`.
8. 🧑‍🏫 Real robot: low speed and a small range first, checking the direction and soft limits before any full-speed motion.

**Dashboard milestone:** A "Turret" card with a top-down dial: robot heading arrow, turret arrow, target vs. actual angle, and an **IN RANGE** / **AT TARGET** indicator.
**Done when:** In sim, the turret stays pointed at the same spot on the field while the robot drives in circles, and it never tries to go past its limits.

---

## Phase 9: Autonomous, part 2: autos that score

**Goal:** Score points with nobody driving, using the mechanisms from Phases 6–8. Shots use **preset** turret angles and flywheel speeds for now; live aiming comes in Phase 11.
**Concepts:** C32 (named commands), C33, C10, C22
**Reference:** `main`: `RobotContainer.bindCommandsForAuto()`, `deploy/pathplanner/`

1. Register named commands (intake, prep flywheel, shoot) and build a 2-piece auto (C32). Each shot uses a preset turret angle and flywheel RPM for a known spot on the path. With a turret, the robot can aim while the path is still driving.
2. Add mechanism safety for auto, like pulling the intake in near the trench (C33).
3. Add the SysId routines behind calibration mode (C22).
4. 🧑‍🏫 Test autos on the real field at slow speed first.

**Dashboard milestone:** Add the running named command to the auto card, next to the auto name and timer.
**Done when:** The 2-piece auto works 3 times in a row in sim.

---

## Phase 10: Driver assist, turret aim and shot distance

**Goal:** One button aims at the hub and picks the right shooter speed, while the driver keeps driving.
**Concepts:** C38, C39, C46, C19, C04, C37
**Reference:** `main`: `util/SemiAutoHelper.java` (hub positions, the distance → RPM table); `churret`: `Churret.aimAtHub(...)`, `fullAutoAim(...)`

1. Calculate the distance and field angle from the robot pose to the hub, using the alliance-correct field positions (C04).
2. Aim the turret at the hub using the Phase 8 angle math (C46). Compare with the old robot's heading-lock approach (C38): which one lets the driver dodge defense?
3. **Out-of-range fallback:** when the hub is outside the turret's range, use heading lock to rotate the drivetrain just enough to bring it back in range (C38).
4. Build the distance → RPM lookup table. Collect real data points on a practice field, then interpolate (C39).
5. Combine them (C37): while held, the turret aims and the flywheel spins up. The trigger feeds only when the turret is **AT TARGET** *and* the flywheel is **READY**.

**Dashboard milestone:** An "Aim" card: distance to the hub, target RPM from the table, and an **AIM LOCKED** indicator (turret at target *and* flywheel ready).
**Done when:** Auto-aim scores from 3 different distances in sim while the robot is facing 3 different directions.

---

## Phase 11: Autonomous, part 3: auto-aim in auto

**Goal:** Autos shoot with the live auto-aim from Phase 10, so shots work from anywhere in range, not just the preset spots from Phase 9.
**Concepts:** C32 (named commands), C37, C40 (preview only; shoot on the move is the Stretch)
**Reference:** `main`: `RobotContainer.bindCommandsForAuto()`; `churret`: `Churret.fullAutoAim(...)`

1. Swap the preset-shot named commands from Phase 9 for the Phase 10 auto-aim command. What can be reused as-is, and what has to change?
2. Let the turret track the hub *while the path is driving*, and fire as soon as **AIM LOCKED** is true.
3. Build a 3-piece auto that includes a shot from a spot you never preset.
4. 🧑‍🏫 Test autos on the real field at slow speed first.

**Dashboard milestone:** Show **AIM LOCKED** and the distance to the hub on the auto card while auto runs.
**Done when:** The auto scores every piece 3 times in a row in sim, including the new spot.

---

## Phase 12: Robot health and driver polish

**Goal:** Make the robot competition-safe and easy to drive.
**Concepts:** C43, C16, C45, C28

1. Add fault reporting the AdvantageKit way (C43). Every IO's inputs should already have a `connected` value, like `driveConnected` in `ModuleIO`. Have each subsystem raise a WPILib `Alert` when a motor is disconnected, the way `Module.java` already does for the drive motors. (The 2026 robot used `HardwareMonitor` and `YAMSUtil` on `main` instead; compare the two approaches.) Give a motor a wrong CAN ID and watch the fault appear.
2. Review every button binding with a driver. Put controller constants in `DriveTeamConstants`.
3. Review the default commands so the robot, including the turret, is in a safe state when no buttons are pressed.

**Dashboard milestone:** A "Mechanism faults" card that lists the active alerts. Use the card in the `sandbox-sim` `custom-dashboard.js` as inspiration, but have the student rebuild it step by step (that version read YAMS-era fault data).
**Done when:** A deliberately broken CAN ID shows up clearly on the dashboard, and the rest of the robot still works.

---

## Phase 13: Competition readiness

**Concepts:** C43, C44, C14, C02

- Pre-match checklist: faults clear, the auto is selected, battery voltage, the turret starts in its known position.
- Check loop time and the roboRIO resource flags (`HardwareConstants.REDUCE_ROBORIO_RESOURCE_USAGE` on `main`), and explain why they exist.
- Logging to USB for post-match replay (C14).
- Driver practice, with feedback turned into code changes.
- 🧑‍🏫 Re-run the drivetrain characterization (Phase 3 step 9) now that the superstructure is mounted, since its weight changes the numbers.

---

## Stretch: Shoot on the move

**Concepts:** C40, C46
**Reference:** `main`: `subsystems/sotm/`; `churret`: `Churret.aimWithSOTM(...)`

Use projectile physics and robot velocity to aim ahead of the hub while driving. The turret makes this practical. This one is for students who finished everything else, and it relies heavily on simulation.
