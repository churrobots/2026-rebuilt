# Guided Tutorial Plan: Build Our Robot From Scratch

This is the roadmap for building our **new robot** from a blank AdvantageKit template: a **Kraken swerve drivetrain** with a **turret** shooter. The AI assistant follows it in **Guided Tutorial Mode** (see `AGENTS.md`), and a student or coach can follow it by hand.

- Concept IDs like **C19** refer to [CONCEPTS.md](CONCEPTS.md).
- "Reference" means our existing code on the reference branches listed in `AGENTS.md`. Peek at a file with `git show <branch>:<path>`. The drivetrain reference is `unleash-the-kraken`, the turret reference is `churret` (unfinished), and most other mechanisms come from the 2026 competition robot on `main`. If the new robot's mechanisms differ, measure the real robot. The phases still apply.
- Every phase ends with a **dashboard milestone**, so you can *see* the new feature working at `http://localhost:5800`.
- 🧑‍🏫 marks a **coach checkpoint**: stop and get a coach before continuing.
- Always get each feature working in **simulation first**, then on the real robot.

Your code does not have to match the reference line-for-line. If it works and you can explain why, that's a win.

---

## Phase 0: Setup and your first program

**Goal:** Tools installed, a fresh project running in simulation, and the dashboard showing something.
**Concepts:** C01, C02, C05, C41, C42, C44
**Zero to Robot:** Step 2 (`zero-to-robot/step-2/wpilib-setup.html`, `frc-game-tools.html`) and Step 4 (`creating-test-drivetrain-program-cpp-java-python.html`)

1. Install WPILib 2026, the FRC Game Tools if you're on Windows, and the vendor tools: **Phoenix Tuner X** (Krakens and TalonFX) and the **REV Hardware Client** (SPARK MAX). Open the project in WPILib VS Code.
2. Tour the control system hardware on the real robot with a coach: roboRIO, radio, PDH, CAN chain, and motor controllers (C01).
3. Create the project from the **AdvantageKit Spark Swerve template**. Our turn motors are SPARK MAX with absolute encoders, like that template. In Phase 2 you'll swap the drive motors to Krakens yourself.
4. 🧑‍🏫 Bring over the team's simulation and dashboard tooling, if it isn't already there. The template doesn't include it. From `sandbox-sim`:
   - `src/main/deploy/dashboard/` (the whole folder; `core/` is protected)
   - `tools/SimSupervisor.java`, `tools/README.md`
   - `src/main/java/frc/robot/sim/SimulationControllerBridge.java`
   - In `Robot.java`: the `WebServer.start(5800, ...)` line, and creating and updating `SimulationControllerBridge` in SIM mode
   - `.vscode/settings.json` (hides plumbing files) and `.vscode/tasks.json` (the **Start Supervised Robot Simulation** task)
5. Click **ChurroSim** (or press ⌘⇧B) to start the simulator, then **ChurroDashboard** to open `http://localhost:5800`. Use the Sim Driver Station there to enable teleop. Explain that this dashboard is where they'll see the results of everything they build from here on (C42).
6. Make your first commit on your own branch (C05).

**Dashboard milestone:** Replace the starter card with a "Hello, robot" card that shows whether the dashboard is connected and the current alliance (the existing `/FMSInfo/IsRedAlliance` pattern).
**Done when:** Saving a Java file restarts the sim automatically, and the dashboard shows the alliance change when you switch it in the Sim Driver Station.

---

## Phase 1: How robot code is organized

**Goal:** Understand the template before changing it.
**Concepts:** C02, C03, C06–C11, C12, C13, C45

1. Walk through `Robot.java` (modes, `robotPeriodic`, the command scheduler), then `RobotContainer.java`, `Constants.java`, and the `drive/` folder at a high level.
2. Explain IO layers using the drive template's `ModuleIO` / `ModuleIOSpark` / `ModuleIOSim` (C12). Mention that in Phase 2 we'll *add* an IO implementation for Krakens. That's exactly what IO layers are for.
3. **Exercise:** In `robotPeriodic()`, log a loop counter with `Logger.recordOutput("Tutorial/LoopCount", count)`.
4. **Exercise:** Bind a controller button to a `Commands.runOnce(...)` that logs a message. See how `onTrue` and `whileTrue` behave differently (C08).

**Dashboard milestone:** Show `Tutorial/LoopCount`. The topic is `/AdvantageKit/RealOutputs/Tutorial/LoopCount`. It should count up at about 50 per second, which proves the 20 ms loop (C02).
**Done when:** The student can explain, in their own words, what runs every 20 ms and what a command is.

---

## Phase 2: Kraken swerve drivetrain

**Goal:** Drive our Kraken swerve robot, first in sim, then for real.
**Concepts:** C04, C12, C15, C17, C24, C25, C26, C28, C22
**Reference:** `unleash-the-kraken`: `drive/ModuleIOKraken.java`, `drive/DriveConstants.java` (the Kraken section), `util/PhoenixUtil.java`, and the `RobotContainer` swap from `ModuleIOSpark` to `ModuleIOKraken`. Also `DriveCommands.java`.

1. Fill in `DriveConstants` for our robot: CAN IDs, track width and wheelbase, wheel radius, drive gearing, current limits, and inversions. Set `driveGearbox = DCMotor.getKrakenX60(1)` so the simulation models a Kraken. Explain every number. Don't copy blindly: *where does each value come from?* (C17)
2. Set the drivetrain's default command to joystick driving. Explain the deadband, input squaring, and negated Y (C28). Drive in sim and compare field-relative with robot-relative driving (C25).
3. Watch odometry build up in AdvantageScope's 2D field (C26).
4. **The IO layer lesson (C12).** Write `ModuleIOKraken`, a new `ModuleIO` implementation:
   - **Drive:** a TalonFX using Phoenix 6: `TalonFXConfiguration` for inversion, brake mode, stator and supply current limits, and `SensorToMechanismRatio` (so the closed loop works in *wheel* rotations). Use `VelocityVoltage` and `VoltageOut` control requests, and status signals for position and velocity.
   - **Turn:** a SPARK MAX + absolute encoder. Reuse the turn half of `ModuleIOSpark`.
   - Swap `ModuleIOSpark` → `ModuleIOKraken` in `RobotContainer`'s REAL case. Point out that `Drive.java` didn't change at all.
5. Explain why the Kraken gains are in different units (TalonFX native: volts per wheel rotation/second) and must be re-tuned. Don't just convert them.
6. 🧑‍🏫 **Real robot:** robot on blocks first. Check the CAN IDs in Phoenix Tuner X and the REV Hardware Client. Set the module zero rotations, check that each module turns and drives the right way, and check the gyro direction (counter-clockwise positive, C04).
7. 🧑‍🏫 Run the template's wheel radius and feedforward characterization from the auto chooser to find the Kraken kS and kV (C22).
8. Add a "reset heading" button (reference: `resetPoseFacingAway()` on `main`).

**Dashboard milestone:** A "Drivetrain" card with the robot's x, y (meters), and heading (degrees). Log them as simple numbers (for example `Tutorial/Drive/X`) so the dashboard doesn't have to decode `Pose2d` structs. Stretch: a small top-down field view.
**Done when:** The robot drives field-relative smoothly in sim *and* on carpet with the Krakens, and the dashboard pose tracks the real movement.

---

## Phase 3: First mechanism, the intake roller

**Goal:** Write a subsystem from scratch, by hand.
**Concepts:** C06, C07, C09, C15, C16, C18, C34, C45
**Reference:** `main`: `subsystems/IntakeRoller.java`, `HardwareConstants.java`, `ControlsConstants.java` (see the roller RPM math comment)

1. Create `IntakeRoller` with a raw vendor motor controller object. Set the CAN ID, inversion, idle mode, and current limit in a constants file (C16, C45).
2. Add `runRoller(double percent)` and `stop()` as **commands**, and a default command that stops it (C09).
3. Bind the left trigger to intake (`whileTrue`) and the left bumper to outtake.
4. Test in sim, then 🧑‍🏫 on the robot.
5. Discuss why open-loop speed changes with battery voltage and game pieces, and how the team chose its target RPM (C18, C34). Keep this open-loop for now. Closed-loop comes in Phase 5.

**Dashboard milestone:** An "Intake" card showing the roller's commanded output and whether it's running (a colored status pill).
**Done when:** The student wrote most of the subsystem themselves and can explain why `RobotContainer` doesn't touch the motor directly.

---

## Phase 4: Position control, the intake arm

**Goal:** Move an arm to exact angles and hold them against gravity.
**Concepts:** C17, C19, C20, C21, C23, C27, C36
**Reference:** `main`: `subsystems/IntakeArm.java` (YAMS `Arm`, absolute encoder, `ArmFeedforward`)

1. Explain absolute encoders, zero offsets, and gear ratios using our arm (C17). Mention the reference's comment about measuring angles empirically.
2. Introduce YAMS (C27): compare what the hand-written roller needed with what `SmartMotorControllerConfig` + `Arm` provide.
3. Build `IntakeArm` with `extend`, `retract`, and `stow` angle commands, soft limits, and a current limit. Default command: retract.
4. **Tuning lesson in sim:** start with all gains at 0 and add kG until the arm holds its position, then add P, then D if it overshoots (C19–C21). Use `TunableNumber`s and an AdvantageScope graph of setpoint vs. measured.
5. Update the intake command in `RobotContainer`: extend the arm *and* spin the roller together (`Commands.parallel`, C10).
6. 🧑‍🏫 Real robot: verify the encoder direction and zero *before* closing the loop, then re-tune the gains on the real arm.

**Dashboard milestone:** An arm card with target angle vs. measured angle (two numbers, or a simple gauge) and an "at target" indicator.
**Done when:** The arm reaches each angle without oscillating, and the student can explain what kG and P each do.

---

## Phase 5: Flywheel and indexing

**Goal:** A velocity-controlled flywheel and indexing, fed only when the flywheel is ready. (For now the turret stays still. It comes next.)
**Concepts:** C18, C20, C21, C35, C34, C10, C37
**Reference:** `main`: `Shooter.java` (TalonFX `FlyWheel`), `Feeder.java`, `Spindexer.java`, `ControlsConstants.java`, `RobotContainer.autoShoot()`

1. Build `Shooter` as a YAMS `FlyWheel`. Idle speed is the default command. Explain why coast mode is used (C15).
2. Tune the flywheel: kV first (the feedforward does most of the work), then P for recovery after each shot (C21, `tuning-flywheel.html`).
3. Build `Feeder` and `Spindexer` (velocity rollers). Go back and switch the intake roller to closed-loop too (C18).
4. Compose the shoot command: spin up → wait until at speed → feed + index (C10, C37). Discuss why feeding early wastes shots.
5. Add `MechanismVisualizer` (from `sandbox-sim`) so AdvantageScope shows the mechanisms moving (C14).

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

1. Add `HardwareMonitor` fault reporting and `YAMSUtil`'s safe motor creation (C43). Unplug a motor in sim (or give it a wrong CAN ID) and watch the fault appear.
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
