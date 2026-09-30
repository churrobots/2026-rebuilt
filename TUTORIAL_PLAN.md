# Guided Tutorial Plan: Build Our Robot From Scratch

This is the roadmap for building our **new robot** starting from a bare AdvantageKit swerve template (the `fresh-start` branch): a **Kraken swerve drivetrain** with a **turret** shooter. The AI assistant follows it in **Guided Tutorial Mode** (see `AGENTS.md`), and a student or coach can follow it by hand.

- Concept IDs like **C19** refer to [CONCEPTS.md](CONCEPTS.md).
- "Reference" means our existing code on the reference branches listed in `AGENTS.md`. Peek at a file with `git show <branch>:<path>`. The drivetrain reference is `unleash-the-kraken`, the turret reference is `churret` (unfinished), and most other mechanisms come from the 2026 competition robot on `main`. If the new robot's mechanisms differ, measure the real robot. The phases still apply.
- Every phase ends with a **dashboard milestone**, so you can *see* the new feature working at `http://localhost:5800`.
- 🧑‍🏫 marks a **coach checkpoint**: stop and get a coach before continuing.
- Always get each feature working in **simulation first**, then on the real robot.

Your code does not have to match the reference line-for-line. If it works and you can explain why, that's a win.

---

## Phase 0: Drive the robot in simulation

**Goal:** Within the first session, the student is driving our swerve robot around a simulated field with a real controller and watching it on the dashboard.
**Concepts:** C02, C05, C41, C42, C13 (lightly; details come in Phase 1)
**Zero to Robot:** Step 2 (`zero-to-robot/step-2/wpilib-setup.html`) for installing WPILib

1. Install WPILib 2026 and open the project in WPILib VS Code. (The vendor tools, Phoenix Tuner X and the REV Hardware Client, can wait until Phase 3.)
2. Check out the tutorial starting branch (`fresh-start`) and make your own branch from it (C05). It's the **AdvantageKit Spark Swerve template** already adapted for our Kraken MAXSwerve modules (`ModuleIOKraken`). It also includes the team's tooling: the browser dashboard, `SimSupervisor`, the Sim Driver Station bridge, and every vendor library we use (YAMS, PhotonVision, PathPlanner, and so on). There are no mechanisms yet. You'll build those.
3. Plug an Xbox-style controller into the laptop. Click **ChurroSim** (or press ⌘⇧B) to start the simulator, then **ChurroDashboard** to open `http://localhost:5800`.
4. In the dashboard's Sim Driver Station, check that the controller shows as connected, then enable **teleop** and **drive**. The template already has the controls:
   - left stick moves the robot
   - right stick turns it
   - hold A to face 0°
   - press X to lock the wheels in an X
5. Explain that this dashboard is where they'll see the results of everything they build from here on (C42). Right now it only shows the alliance, so let's make it show the robot.
6. **First code edit:** in `Drive.periodic()`, log the pose as simple numbers:
   - `Logger.recordOutput("Tutorial/Drive/X", getPose().getX())`
   - `Tutorial/Drive/Y`
   - `Tutorial/Drive/HeadingDegrees`

   Save, and watch the sim restart by itself (C41).
7. Make your first commit (C05).

**Dashboard milestone:** A "Drivetrain" card with x, y (meters), and heading (degrees) that changes live as you drive. Stretch: a small top-down field dot with a heading arrow.
**Done when:** The student can drive in sim with the controller, and the dashboard numbers change the way they expect (forward is +X, turning left increases heading, C04).

---

## Phase 1: How that just worked

**Goal:** Understand the code behind what the student just did, so they can change it.
**Concepts:** C02, C03, C04, C06–C11, C12, C13, C25, C28, C45

1. **Follow the stick press through the code:** controller → `RobotContainer`'s default command → `DriveCommands.joystickDrive` → `Drive.runVelocity` → each module. Along the way, explain the command-based pieces: subsystem, command, default command, and trigger (C06–C11).
2. In `joystickDrive`, explain the deadband, input squaring, and why Y is negated (C28). Explain field-relative driving (C25): spin the robot in sim and push forward. It still goes "up the field."
3. Walk through `Robot.java`: the modes, `robotPeriodic`, and the command scheduler running every 20 ms (C02).
4. Explain IO layers using `ModuleIO` / `ModuleIOKraken` / `ModuleIOSim` (C12). `RobotContainer` picks Kraken hardware on the real robot and physics simulation on a laptop, and `Drive.java` never knows the difference. That's why they could drive without a robot.
5. **Exercise:** Change the deadband or max speed, save, and *feel* the difference with the controller.
6. **Exercise:** Bind a new button to a `Commands.runOnce(...)` that logs a message. Try `onTrue` vs. `whileTrue` and watch the difference on the dashboard (C08).

**Dashboard milestone:** Add the button exercise's value (for example a press counter, or "held/not held") to the dashboard.
**Done when:** The student can explain, in their own words, the path from stick to wheels, and what a command and a subsystem are.

---

## Phase 2: First mechanism, the intake arm (in simulation)

**Goal:** Build a real mechanism early, and *see* it move in simulation: an arm that goes to exact angles and holds them against gravity.
**Concepts:** C06, C07, C09, C15, C16, C17, C19, C20, C21, C27, C36, C45
**Reference:** `main`: `subsystems/IntakeArm.java` (YAMS `Arm`, `ArmFeedforward`, soft/hard limits); `sandbox-sim`: `subsystems/MechanismVisualizer.java` (AdvantageScope visualization)

1. Show what the intake arm does on the real robot, or in a match video. Name its positions: stowed, retracted, and extended.
2. Introduce YAMS (C27): one config object describes the motor, gearing, limits, PID, and feedforward, and YAMS simulates the arm's physics for free.
3. Create an `IntakeArm` subsystem with a YAMS `Arm`: the SPARK MAX CAN ID, gear reduction, a current limit, soft limits inside hard limits, and the arm's length and mass (so the simulation behaves like the real thing). Put the numbers in constants (C16, C17, C45). Call `simIterate()` in `simulationPeriodic()`.
4. Add `extend()`, `retract()`, and `stow()` angle commands, with `retract()` as the default command (C07, C09). Bind them to the D-pad, since A and X are already used by the drivetrain.
5. Log the target angle, measured angle, and "at target" (for example `Tutorial/Arm/TargetDegrees`) (C13).
6. **Build the arm visualization on the dashboard** (see the milestone). Let the student design how it looks.
7. **Tuning lesson, the fun part (C19–C21).** All in sim, watching the dashboard. Change one gain at a time and save; the sim restarts in seconds:
   - All gains at 0 → the arm **droops** under gravity.
   - Add **kG** until it holds still wherever it is (C20).
   - Add **P** → it moves to the target, but too much P makes it **wobble** (C19).
   - Add a little **D** to calm the wobble.

   A live setpoint vs. measured graph makes this obvious.
8. Optional: show the same arm in AdvantageScope with a `LoggedMechanism2d` (C14).
9. 🧑‍🏫 **Later, on the real robot** (after Phase 3): verify the absolute encoder direction and zero *before* closing the loop, then re-tune the gains. Mention the reference's comment about measuring angles empirically (C17).

**Dashboard milestone:** An "Intake arm" card with a side-view drawing: a pivot point, a solid line for where the arm *is*, and a faint "ghost" line for where it's *trying to go*. Also show the two angles as numbers and an **AT TARGET** light. Stretch: a small live graph of target vs. measured.
**Done when:** Pressing the D-pad swings the arm on the dashboard to each position without drooping or wobbling, and the student can explain what kG and P each do.

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
8. 🧑‍🏫 Do the real-robot arm check from Phase 2, step 9.

**Dashboard milestone:** The Drivetrain card from Phase 0 now tracks the *real* robot (connect the dashboard to the robot's address). Add a "module health" row showing each module's measured wheel angle, which makes a mis-zeroed module obvious.
**Done when:** The robot drives field-relative smoothly on carpet, and the dashboard pose tracks the real movement.

---

## Phase 4: Under the hood, the intake roller by hand

**Goal:** Write a simple mechanism *without* YAMS, to see what YAMS was doing for you. Then combine it with the arm into one intake action.
**Concepts:** C06, C07, C09, C10, C15, C16, C18, C27, C34, C45
**Reference:** `main`: `subsystems/IntakeRoller.java`, `ControlsConstants.java` (see the roller RPM math comment)

1. Create `IntakeRoller` with a raw vendor motor controller object: CAN ID, inversion, idle mode, and current limit in constants (C15, C16, C45). Compare it with how much YAMS handled for the arm (C27).
2. Add `runRoller(double percent)` and `stop()` as **commands**, and a default command that stops it (C09).
3. Combine them into one intake command in `RobotContainer`: extend the arm *and* spin the roller together (`Commands.parallel`, C10). Bind it to the left trigger (`whileTrue`), with outtake on the left bumper.
4. Discuss why open-loop speed changes with battery voltage and game pieces, and how the team chose its target RPM (C18, C34). Keep it open-loop for now. Closed-loop comes in Phase 5.
5. 🧑‍🏫 Test on the real robot.

**Dashboard milestone:** Add the roller to the intake arm card: the commanded output and a spinning/stopped indicator, so the whole intake is on one card.
**Done when:** One trigger pull extends the arm and runs the roller, the student wrote most of the roller themselves, and they can explain why `RobotContainer` doesn't touch the motor directly.

---

## Phase 5: Flywheel and indexing

**Goal:** A velocity-controlled flywheel and indexing, fed only when the flywheel is ready. (For now the turret stays still. It comes next.)
**Concepts:** C18, C20, C21, C35, C34, C10, C37
**Reference:** `main`: `Shooter.java` (TalonFX `FlyWheel`), `Feeder.java`, `Spindexer.java`, `ControlsConstants.java`, `RobotContainer.autoShoot()`

1. Build `Shooter` as a YAMS `FlyWheel`. Idle speed is the default command. Explain why coast mode is used (C15).
2. Tune the flywheel: kV first (the feedforward does most of the work), then P for recovery after each shot (C21, `tuning-flywheel.html`).
3. Build `Feeder` and `Spindexer` (velocity rollers). Go back and switch the intake roller to closed-loop too (C18).
4. Compose the shoot command: spin up → wait until at speed → feed + index (C10, C37). Discuss why feeding early wastes shots.
5. Add `MechanismVisualizer` (from `sandbox-sim`) so AdvantageScope shows all the mechanisms moving together: arm, roller, and flywheel (C14).

**Dashboard milestone:** A "Shooter" card: target RPM, actual RPM, and a big **READY** light when within tolerance.
**Done when:** In sim, pressing shoot waits for READY before feeding, every time.

---

## Phase 6: The turret

**Goal:** A turret that moves safely to any angle in its range, then holds a *field-relative* direction while the robot spins.
**Concepts:** C46, C04, C16, C17, C19, C20, C23, C27, C45
**Reference:** `churret`: `subsystems/Churret.java`. It's an unfinished prototype, so read the "Turret prototype caveats" in `AGENTS.md` first.

1. 🧑‍🏫 **Measure the real turret with the mechanical team:** gear reduction, range of travel and hard stops, which way 0° points relative to the robot's front, and how the code will know the turret's position at power-on (absolute encoder vs. "always start centered") (C17, C46).
2. Build `Turret` with a YAMS `Pivot`: brake mode, a current limit, soft limits *inside* the hard stops, and a motion profile (max velocity and acceleration, C23). Discuss why there's no kG (C20).
3. Add manual commands: face front, face left, and D-pad nudges. Tune in sim: kS for friction, then P, then adjust the profile (C21, `tuning-turret.html`).
4. **Angle math, on paper first:** `turretAngle = fieldAngleToTarget − robotHeading`, wrapped to the turret's range, with out-of-range targets going to the **nearest** limit. Put it in its own small method.
5. **Good practice:** write a JUnit test for that method using a few cases, including the out-of-range ones (C45). Then have the student find the clamp bug in `Churret.java` and explain why the test would catch it.
6. Add a "hold field direction" command: the turret keeps pointing at a fixed field angle while you spin the robot in sim.
7. Add the turret to `MechanismVisualizer`.
8. 🧑‍🏫 Real robot: low speed and a small range first, checking the direction and soft limits before any full-speed motion.

**Dashboard milestone:** A "Turret" card with a top-down dial: robot heading arrow, turret arrow, target vs. actual angle, and an **IN RANGE** / **AT TARGET** indicator.
**Done when:** In sim, the turret stays pointed at the same spot on the field while the robot drives in circles, and it never tries to go past its limits.

---

## Phase 7: Robot health and driver polish

**Goal:** Make the robot competition-safe and easy to drive.
**Concepts:** C43, C16, C45, C28

1. Add `HardwareMonitor` fault reporting and `YAMSUtil`'s safe motor creation (C43), from `main`. Register every motor controller, including the drive modules and the gyro. Unplug a motor in sim (or give it a wrong CAN ID) and watch the fault appear.
2. Review every button binding with a driver. Put controller constants in `DriveTeamConstants`.
3. Review the default commands so the robot, including the turret, is in a safe state when no buttons are pressed.

**Dashboard milestone:** Bring back the "Mechanism faults" card from the `sandbox-sim` `custom-dashboard.js`, but have the student rebuild it step by step.
**Done when:** A deliberately broken CAN ID shows up clearly on the dashboard, and the rest of the robot still works.

---

## Phase 8: Vision

**Goal:** Use AprilTags to know exactly where the robot is on the field. The turret can only aim well if the pose is right.
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

## Phase 9: Driver assist, turret aim and shot distance

**Goal:** One button aims at the hub and picks the right shooter speed, while the driver keeps driving.
**Concepts:** C38, C39, C46, C19, C04, C37
**Reference:** `main`: `util/SemiAutoHelper.java` (hub positions, the distance → RPM table); `churret`: `Churret.aimAtHub(...)`, `fullAutoAim(...)`

1. Calculate the distance and field angle from the robot pose to the hub, using the alliance-correct field positions (C04).
2. Aim the turret at the hub using the Phase 6 angle math (C46). Compare with the old robot's heading-lock approach (C38): which one lets the driver dodge defense?
3. **Out-of-range fallback:** when the hub is outside the turret's range, use heading lock to rotate the drivetrain just enough to bring it back in range (C38).
4. Build the distance → RPM lookup table. Collect real data points on a practice field, then interpolate (C39).
5. Combine them (C37): while held, the turret aims and the flywheel spins up. The trigger feeds only when the turret is **AT TARGET** *and* the flywheel is **READY**.

**Dashboard milestone:** An "Aim" card: distance to the hub, target RPM from the table, and an **AIM LOCKED** indicator (turret at target *and* flywheel ready).
**Done when:** Auto-aim scores from 3 different distances in sim while the robot is facing 3 different directions.

---

## Phase 10: Autonomous

**Goal:** Score points with nobody driving.
**Concepts:** C32, C33, C10, C22
**Reference:** `main`: `RobotContainer.bindCommandsForAuto()`, `getAutonomousCommand()`, `deploy/pathplanner/`

1. Install PathPlanner and configure the robot settings (mass, module config with the Kraken X60, and so on).
2. Draw a simple "drive out of the starting zone" path and run it in sim.
3. Register named commands (intake, prep flywheel, aim + shoot) and build a 2-piece auto (C32). With a turret, the robot can aim while the path is still driving.
4. Add mechanism safety for auto, like pulling the intake in near the trench (C33).
5. Add the auto chooser, plus the SysId routines behind calibration mode (C22).
6. 🧑‍🏫 Test autos on the real field at slow speed first.

**Dashboard milestone:** Show the selected auto name and an auto timer. Stretch: draw the planned path on the field view.
**Done when:** The 2-piece auto works 3 times in a row in sim.

---

## Phase 11: Climber (bonus, if the new robot has one)

**Goal:** A linear mechanism with motion profiling.
**Concepts:** C23, C20, C36
**Reference:** `main`: `subsystems/ClimberTW.java` (YAMS `Elevator`, `ElevatorFeedforward`, max velocity/acceleration)

Build it like the arm, but with height instead of angle. Use kG for gravity, and a trapezoid profile so it moves smoothly.
**Dashboard milestone:** Climber height and state.

---

## Phase 12: Competition readiness

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
