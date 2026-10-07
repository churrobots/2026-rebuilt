# Guided Tutorial Plan: Build Our Robot From Scratch

This is the roadmap for building our **new robot** starting from a bare AdvantageKit swerve template (the `fresh-start` branch): a **Kraken swerve drivetrain** with a **turret** shooter. The AI assistant follows it in **Guided Tutorial Mode** (see `AGENTS.md`), and a student or coach can follow it by hand.

- Concept IDs like **C19** refer to [CONCEPTS.md](CONCEPTS.md).
- "Reference" means our existing code on the reference branches listed in `AGENTS.md`. Peek at a file with `git show <branch>:<path>`. The drivetrain reference is `unleash-the-kraken`, the turret reference is `churret` (unfinished), and most other mechanisms come from the 2026 competition robot on `main`. If the new robot's mechanisms differ, measure the real robot. The phases still apply.
- Every phase ends with a **dashboard milestone**, so you can *see* the new feature working at `http://localhost:5800`.
- **Students don't write the dashboard code by hand.** Student effort goes into the Java robot code. The student decides *what* each card shows (which values, and how: numbers, lights, a dial, a graph) and writes the `Logger.recordOutput(...)` calls on the robot side. An AI assistant like Claude writes the `custom-dashboard.js` / `.css` code, then explains it in a sentence or two.
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
3. Plug an Xbox-style controller into the laptop. Click **ChurroSim** (or press ⌘⇧B) to start the simulator, then **ChurroDashboard** to open `http://localhost:5800`.
4. In the dashboard's Sim Driver Station, check that the controller shows as connected, then enable **teleop** and **drive**. The template already has the controls:
   - left stick moves the robot
   - right stick turns it
   - hold A to face 0°
   - press X to lock the wheels in an X
5. Watch the robot move on the dashboard's **Field** card. It's drawn from the driver's point of view: your alliance wall is at the bottom, and it flips when you switch alliance in the Sim Driver Station. Explain that this dashboard is where they'll see the results of everything they build from here on (C42), and that they can change anything about it.
6. **First code edit (robot):** in `Drive.periodic()`, log the robot's speed as a simple number, for example `Logger.recordOutput("Tutorial/Drive/SpeedMetersPerSec", ...)` using `getChassisSpeeds()`. Save, and watch the sim restart by itself (C41, C13).
7. **See it on the dashboard:** the student asks Claude to show that speed on the Field card, next to the pose readout. Claude writes the dashboard code and points out the topic name it reads (`/AdvantageKit/RealOutputs/Tutorial/Drive/SpeedMetersPerSec`), so the student sees how their `recordOutput` key turns into a dashboard topic (C13, C42). Then let them ask for something just for fun, like a different robot color or size.
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

## Phase 3: Drivetrain on the real robot

**Goal:** Everything that worked in sim now works on carpet with the Krakens.
**Concepts:** C01, C04, C12, C15, C17, C22, C24, C26, C44
**Reference:** `unleash-the-kraken`: `drive/ModuleIOKraken.java`, `drive/DriveConstants.java` (the Kraken section), `util/PhoenixUtil.java`
**Zero to Robot:** Step 2 (`frc-game-tools.html`) and Step 4 (`running-test-program.html`)

1. Install the FRC Game Tools (Driver Station) on Windows, plus **Phoenix Tuner X** and the **REV Hardware Client**. Tour the control system hardware on the robot with a coach: roboRIO, radio, PDH, CAN chain, and motor controllers (C01).
2. Review `DriveConstants` against the new robot: CAN IDs, track width and wheelbase, wheel radius, drive gearing, current limits, and inversions. They start with values from the old robot. Note that `driveGearbox = DCMotor.getKrakenX60(1)` is what makes the simulation model a Kraken. Explain every number and verify it on the real robot. Don't trust it blindly: *where does each value come from?* (C17)
3. **The IO layer lesson (C12).** Read through `ModuleIOKraken` together. The team wrote it to swap the template's NEO Vortex drive motors for Krakens:
   - **Drive:** a TalonFX using Phoenix 6: `TalonFXConfiguration` for inversion, brake mode, stator and supply current limits, and `SensorToMechanismRatio` (so the closed loop works in *wheel* rotations). It uses `VelocityVoltage` and `VoltageOut` control requests, and status signals for position and velocity.
   - **Turn:** a SPARK MAX + absolute encoder, the same as the template.
   - Compare it with the template's original `ModuleIOSpark` (`git show main:src/main/java/frc/robot/subsystems/drive/ModuleIOSpark.java`). Point out that `Drive.java` didn't have to change at all.
4. Explain why the Kraken gains are in different units (TalonFX native: volts per wheel rotation/second) and must be re-tuned. Don't just convert them.
5. 🧑‍🏫 Deploy, with the robot on blocks first (C44). Check the CAN IDs in Phoenix Tuner X and the REV Hardware Client. Set the module zero rotations, check that each module turns and drives the right way, and check the gyro direction (counter-clockwise positive, C04).
6. 🧑‍🏫 Run the template's wheel radius and feedforward characterization from the auto chooser to find the Kraken kS and kV (C22).
7. Add a "reset heading" button (reference: `resetPoseFacingAway()` on `main`).
8. 🧑‍🏫 Test the intake roller from Phase 2 on the real robot: check the CAN ID, spin direction, and current limit. This is the first time its real-hardware IO class runs.

**Dashboard milestone:** The Field card now tracks the *real* robot (connect the dashboard to the robot's address). Add a "module health" row showing each module's measured wheel angle, which makes a mis-zeroed module obvious.
**Done when:** The robot drives field-relative smoothly on carpet, and the dashboard pose tracks the real movement.

---

## Phase 4: Vision

**Goal:** Use AprilTags to know exactly where the robot is on the field. Everything after this gets a more accurate pose, and later the turret needs a good pose to aim.
**Concepts:** C29, C30, C31, C12, C26
**Reference:** `main`: `subsystems/vision/` (AdvantageKit vision template), `VisionConstants.java`

1. Explain AprilTags and why odometry drifts (C26, C29).
2. Add the AdvantageKit `Vision` subsystem with `VisionIOPhotonVisionSim` for the new robot's cameras. Wire it to `drive::addVisionMeasurement`.
3. Measure and enter each robot-to-camera transform on the new robot (C30). Discuss the old robot's lesson: "all cameras are flipped on this robot."
4. In sim, drive around and watch accepted vs. rejected vision poses in AdvantageScope. Explain the standard deviations and rejection rules in `Vision.java`.
5. 🧑‍🏫 Real robot: configure the PhotonVision coprocessor, and check the pose against a tape-measured position on the field.

**Dashboard milestone:** A "Vision" card showing, per camera: connected, number of tags seen, and whether the last pose was accepted.
**Done when:** Pushing the robot by hand (or "slipping" it in sim) is corrected by vision within about a second.

---

## Phase 5: Autonomous, part 1: follow a path

**Goal:** The robot drives a PathPlanner path all by itself. No mechanisms yet, just the drivetrain, with vision keeping the pose accurate.
**Concepts:** C32 (paths and the auto chooser; named commands come in Phase 9)
**Reference:** `main`: `getAutonomousCommand()`, `deploy/pathplanner/`

1. Install PathPlanner and configure the robot settings (mass, module config with the Kraken X60, and so on).
2. Draw a simple "drive out of the starting zone" path and run it in sim.
3. Pick it from the auto chooser. The template already has one (Phase 3 used it for characterization).
4. 🧑‍🏫 Test the path on the real field at slow speed first.

**Dashboard milestone:** Show the selected auto name and an auto timer. Stretch: draw the planned path on the field view.
**Done when:** The drive-out path works 3 times in a row in sim.

---

## Phase 6: The intake arm (position control)

**Goal:** An arm that goes to exact angles and holds them against gravity, built as your second IO layer. Then combine it with the roller into one intake action.
**Concepts:** C06, C07, C09, C10, C12, C15, C16, C17, C19, C20, C21, C36, C45
**Reference:** structure: your `IntakeRollerIO` from Phase 2, plus a public AdvantageKit arm example; hardware values: `main`: `subsystems/IntakeArm.java` (YAMS: take its CAN ID, gearing, limits, and starting gains, not its structure); `sandbox-sim`: `subsystems/MechanismVisualizer.java` (AdvantageScope visualization)

1. Show what the intake arm does on the real robot, or in a match video. Name its positions: stowed, retracted, and extended.
2. **Design the IO layer.** Start from the roller's IO: what's the same, and what's new? (Position in radians, and the absolute encoder.)
3. Write `IntakeArmIO`, `IntakeArmIOSim`, and the real-hardware IO. The sim uses WPILib's `SingleJointedArmSim` with the gear ratio, the arm's length and mass, the min/max angles, and gravity turned on, so the simulated arm droops like the real one. The real IO gets the SPARK MAX CAN ID, gear reduction, current limit, and absolute encoder. Put the numbers in constants (C16, C17, C45).
4. **Control in the subsystem.** `IntakeArm` uses a WPILib `PIDController` plus an `ArmFeedforward` to compute volts, then calls `io.setVoltage(...)`. That way the same control code runs in sim and on the robot. Clamp every target inside soft limits that sit inside the hard limits (C16). Mention the other option: running PID on the motor controller itself. What's the trade-off?
5. Add `extend()`, `retract()`, and `stow()` angle commands, with `retract()` as the default command (C07, C09). Bind them to the D-pad, since A and X are already used by the drivetrain.
6. Log the target angle, measured angle, and "at target" (for example `Tutorial/Arm/TargetDegrees`) (C13).
7. **Design the arm visualization on the dashboard** (see the milestone). The student decides how it looks and what it shows; Claude writes the dashboard code.
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
3. Build `Feeder` and `Spindexer` (velocity rollers), each with its own IO layer. Reuse what you learned writing the intake roller. Then go back and switch the intake roller to closed-loop too, by adding a `setVelocity(...)` to its IO (C18).
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

**Dashboard milestone:** A "Mechanism faults" card that lists the active alerts. Use the card in the `sandbox-sim` `custom-dashboard.js` as inspiration (that version read YAMS-era fault data). The student decides what the card shows; Claude writes it.
**Done when:** A deliberately broken CAN ID shows up clearly on the dashboard, and the rest of the robot still works.

---

## Phase 13: Competition readiness

**Concepts:** C43, C44, C14, C02

- Pre-match checklist: faults clear, the auto is selected, battery voltage, the turret starts in its known position.
- Check loop time and the roboRIO resource flags (`HardwareConstants.REDUCE_ROBORIO_RESOURCE_USAGE` on `main`), and explain why they exist.
- Logging to USB for post-match replay (C14).
- Driver practice, with feedback turned into code changes.

---

## Stretch: Shoot on the move

**Concepts:** C40, C46
**Reference:** `main`: `subsystems/sotm/`; `churret`: `Churret.aimWithSOTM(...)`

Use projectile physics and robot velocity to aim ahead of the hub while driving. The turret makes this practical. This one is for students who finished everything else, and it relies heavily on simulation.
