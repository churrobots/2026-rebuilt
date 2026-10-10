# Learnings
Student: (not given yet)

## Concept checklist
Legend: [ ] not yet · [~] seen, still shaky · [x] understood

- [ ] C01 · The FRC control system
- [~] C02 · Robot modes and the 20 ms loop (2026-10-07, Phase 1 step 3)
- [ ] C03 · Java you'll see in robot code
- [ ] C04 · Units and coordinates
- [~] C05 · Git and branches (2026-10-07, Phase 0 step 8)
- [ ] C06 · Subsystems
- [~] C07 · Commands and requirements (2026-10-07, Phase 1 step 1)
- [~] C08 · Triggers and button bindings (2026-10-07, Phase 1 step 6)
- [x] C09 · Default commands (2026-10-07, Phase 1 step 1)
- [~] C10 · Composing commands (2026-10-10, side quest)
- [~] C11 · RobotContainer (2026-10-07, Phase 1 step 1)
- [x] C12 · IO layers (hardware abstraction) (2026-10-07, Phase 1 step 4)
- [~] C13 · Logging inputs and outputs (2026-10-07, Phase 0 step 6)
- [ ] C14 · AdvantageScope and log replay
- [~] C15 · Motors and motor controllers (2026-10-10, Phase 2 step 3)
- [~] C16 · Current limits and safety (2026-10-10, Phase 2 step 4)
- [ ] C17 · Encoders, gear ratios, and zero offsets
- [~] C18 · Open-loop vs. closed-loop control (2026-10-10, Phase 2 step 7)
- [ ] C19 · PID feedback control
- [ ] C20 · Feedforward
- [ ] C21 · Tuning a mechanism
- [ ] C22 · System identification (SysId)
- [ ] C23 · Motion profiles
- [ ] C24 · Swerve drive
- [~] C25 · Kinematics and field-relative driving (2026-10-07, Phase 1 step 2)
- [x] C26 · Odometry (2026-10-10, Phase 4 step 1)
- [x] C27 · Writing IO layers vs. using a mechanism library (2026-10-10, Phase 2 step 8)
- [x] C28 · Driver input shaping (2026-10-07, Phase 1 step 2)
- [~] C29 · AprilTags (2026-10-10, Phase 4 step 1)
- [~] C30 · Camera transforms and pose estimation (2026-10-10, Phase 4 step 4)
- [ ] C31 · PhotonVision
- [ ] C32 · PathPlanner and named commands
- [ ] C33 · Mechanism state in autonomous
- [ ] C34 · Rollers and indexers
- [ ] C35 · Flywheels (velocity control)
- [ ] C36 · Arms and elevators (position control)
- [ ] C37 · Superstructure coordination
- [ ] C46 · Turrets
- [ ] C38 · Heading lock and auto-aim
- [ ] C39 · Lookup tables and interpolation
- [ ] C40 · Shoot on the move
- [~] C41 · Simulation (2026-10-07, Phase 0 step 3)
- [~] C42 · NetworkTables and dashboards (2026-10-07, Phase 0 step 5)
- [ ] C43 · Fault detection and alerts
- [ ] C44 · Deploying and the Driver Station
- [ ] C45 · Software design habits

## Notes by concept

### C05 · Git and branches
- 2026-10-07 (Phase 0 step 2): Student chose to skip making a personal branch for now. Revisit before the first commit (step 8).
- 2026-10-07 (Phase 0 step 8): Chose the branch name `madtown`. Claude made the branch and the commit. The student hasn't run git commands themselves yet or answered the "Check yourself" question.

### C42 · Dashboards (Phase 0 step 7)
- 2026-10-07 (Phase 0 step 7): Student added `SPEED_TOPIC` to the redraw list and renamed it to match the other names. Missed the `/AdvantageKit/RealOutputs/` prefix twice. A coach then asked Claude to write the rest of the dashboard code so students can focus on Java.

### C41 · Simulation / C42 · Dashboards
- 2026-10-07 (Phase 0 steps 3–5): Sim was already running. Student drove in teleop and reported "all good." Didn't answer the field-relative prediction question (push forward while robot faces sideways), so C25 is still untested.
### C13 · Logging inputs and outputs
- 2026-10-07 (Phase 0 step 6): Found `getChassisSpeeds()` on their own. Answered the 3-4-5 speed puzzle correctly ("5m/s"). Filled in the fill-in-the-blank snippet correctly (vx/vy with `Math.hypot`) and logged `Tutorial/Drive/SpeedMetersPerSec`. Hasn't seen the dashboard side yet.
### C07 · Commands / C09 · Default commands / C11 · RobotContainer
- 2026-10-07 (Phase 1 step 1): Found `getLeftY` inside `configureButtonBindings()` and said it "maps the commands." Commands were explained with a worker/job analogy (subsystem = worker, command = job). Predicted default commands correctly in their own words: "if its not doing another command it defaults to the setdefaultcommand."
- 2026-10-07 (Phase 1 step 1): After releasing A, saw the robot "stays at 0 degrees" but didn't say why. Explained that the default command resumes and the right stick is centered, so there's no rotation (steering-wheel analogy). When testing the right stick while holding A, said "it sets it back to a certain angle." Seems to understand that the A command overrides the turn input. Not yet asked to explain interruption in their own words.
### C28 · Driver input shaping
- 2026-10-07 (Phase 1 step 2): Deadband in their own words: "if its close enough to zero, its basically zero." Didn't say what would happen without it (the robot creeping).
- 2026-10-07 (Phase 1 step 2): Squaring: got 0.5→0.25 and 1→1 right, and explained why: "more precise at slow speeds." 
- 2026-10-07 (Phase 1 step 2): Y negation: "so pushing forward drives forward, otherwise it goes backwards." Correct. C28 is understood (deadband, squaring, and negation all explained in their own words).
### C25 · Kinematics and field-relative driving
- 2026-10-07 (Phase 1 step 2): Explained with the RC-car vs. "up the field" comparison. Correctly said our robot uses "field relative driving" (from the sim test and line 93). Kinematics (ChassisSpeeds → module states) only mentioned so far.
### C02 · Robot modes and the 20 ms loop
- 2026-10-07 (Phase 1 step 3): Explained with a flipbook analogy and `CommandScheduler.run()` in `robotPeriodic()`. Check-yourself answer: "that takes a lot of time you dont want to be waiting on it." On the right track, but didn't say that the *whole robot* freezes (no other subsystem or command runs, and the motors are stuck on their last output). Re-explained with a single-cashier analogy. Worth revisiting when they write their first subsystem periodic().
### C12 · IO layers
- 2026-10-07 (Phase 1 step 4): Guessed the purpose from the file names alone: "they both use the ModuleIO since we want to simulate when we dont have the physical robot, and want a different one for when we do." Good intuition before any explanation. Taught with the outlet/plug analogy and the RobotContainer switch. Check-yourself ("new motor brand, which file changes?"): answered "ModuleIOKraken." Correct. Added that in practice you'd write a new ModuleIO<Brand> and change the RobotContainer lines.
### Phase 1 step 5 (deadband / max speed exercise)
- 2026-10-07: Asked "whats next" without changing any values (no diff), so the exercise was skipped. Could come back to it when tuning drive feel with a driver (Phase 7).
### C08 · Triggers and button bindings
- 2026-10-07 (Phase 1 step 6): Explained onTrue (doorbell) vs. whileTrue (flashlight). Student started on their own: added `private int COUNT_B;` and `controller.b().onTrue(Commands.runOnce(() -> COUNT_B++))`, which is the right idea. Then asked Claude to "write the rest." Claude renamed the variable to `bPressCount` (camelCase for variables) and added logging plus a whileTrue/startEnd "held" binding. Asked why holding B for 3 s only counts once: "because of the runONCE command." Half right. It's also that onTrue fires only on the press (false→true). Explained with a `whileTrue(Commands.run(...))` counter-example (that would count about 150).
### Phase 1 "Done when" check
- 2026-10-07: Asked to explain the stick → wheels path in their own words. Answered only "robotcontainer java file," then skipped the fill-in-the-blank. Phase 1 marked done, but the full path explanation was **not** confirmed. Revisit (e.g., ask where intake commands go in Phase 2).
### C12 · IO layers: designing IntakeRollerIO (Phase 2 step 2)
- 2026-10-07: Asked what to read and what to command. Said read "the limits and the time running and speed," and command "spinning the motor." Speed and spinning are right. "Time running" isn't a sensor (the code can track it itself), and "limits" is close to current draw. Introduced the 4 standard inputs: connected, velocity, applied volts, current.
- 2026-10-07 (Phase 2 step 3): Asked Claude to copy ModuleIO as a starting point, then trimmed it to 4 inputs plus setVoltage on their own. Structure correct. Claude did the final renames (drop the `drive` prefix) and comments when asked ("do it"). **Note for coach:** Claude accidentally overwrote a pre-existing IntakeRollerIO.java the student had just created. The student was told how to recover it.
### C41 · Simulation: IntakeRollerIOSim (Phase 2 step 3)
- 2026-10-10: Asked for a one-paragraph summary followed by detailed bullets. That format seemed to help. Then asked Claude to "make the file for us." Claude wrote it and walked through it. The student hasn't explained DCMotorSim in their own words yet.
- 2026-10-10: Checked with mechanical and found the roller motor is a Falcon 500. Changed the sim model themselves. Predicted the Falcon would be "slower." Claude had wrongly said it was "weaker and slower." It's weaker (less torque) but has a *higher* free speed (6380 vs 6000 RPM). Corrected this and used it to teach speed vs. torque.
- 2026-10-10: Didn't know which IO RobotContainer should create for the intake in sim mode ("idk"). Claude answered it (IntakeRollerIOSim). The C12 pattern may not be fully transferred to new mechanisms yet. Re-check when wiring up RobotContainer.
- 2026-10-10 (Phase 2 step 4): Asked which ModuleIOKraken settings the roller needs, they said "all the driving ones and not the turning." Right direction. Claude pointed out the drive-only parts (odometry/position signal, closed-loop gains, brake mode). Didn't answer the coast vs. brake question. Asked Claude to write the TalonFX IO and constants. Asked for line numbers so they could read the code themselves (good sign). Understood that the TODO values get verified in Phase 3.
- 2026-10-10 (Phase 2 step 5): Asked Claude to write the subsystem ("do the coding for us NOW!"). The student is eager for visible results. So far they've written very little Java themselves in Phase 2.
- 2026-10-10 (Phase 2 step 6): Very excited to make it spin ("YES OFCCC I FEINING TO GO"). Claude wrote the commands, bindings, and Intake card.
- 2026-10-10 (Phase 2 step 6): Tested in sim: the roller hits **3209 RPM** at 6 V. The dashboard milestone works. Didn't give a prediction first.
### C18 · Open-loop vs. closed-loop control
- 2026-10-10 (Phase 2 step 7): Explained open-loop with a bike analogy (same pedaling effort, slower uphill). Asked what happens to RPM when a piece jams, and they answered "if it gets stuck you push harder." That's the idea behind closed-loop control, found on their own! Clarified that our current open-loop code *doesn't* push harder, so the speed drops. Closed-loop comes in Phase 7.
### C27 · Writing IO layers vs. using a library
- 2026-10-10 (Phase 2 step 8): Pre-built vs. custom PC analogy. Pro, in their words: "we can change the values when something isnt working and know exactly whats going on because we know where the robot code is at." Con: "it would take long to search for the things" (more code to write and look through). Solid, own-words answer.
- 2026-10-10 (Phase 2 done check): "New motor brand, which file?" They answered `IntakeRollerIOTalonFX` right away. C12 now transfers to their own mechanism.
### C10 · Composing commands
- 2026-10-10 (side quest after Phase 2): Panic button, `onTrue(outtake().withTimeout(1.0))`. Given 3 clues (existing command, onTrue, withTimeout as a microwave timer). Asked Claude to write the one line instead of trying. Not yet checked whether they understand why it stops on its own.
### C26 · Odometry / C29 · AprilTags
- 2026-10-10 (Phase 4 step 1): Odometry explained as "counting steps with your eyes closed," and AprilTags as "opening your eyes to check." Asked what happens to turret shots if the pose is wrong: "it wont shoot at the right place because its supposed to be somewhere else." Correct, own words.
- 2026-10-10 (Phase 4 step 2): Came up with their own visualization idea: "ray cast vision," with cones green if accepted and red if rejected. Good instinct for using the dashboard to see what the robot sees (C42).
### C30 · Pose estimation: rejection and trust
- 2026-10-10 (Phase 4 step 4): Explained with the "asking for directions" analogy. Read the rejectPose rules in Vision.java and picked maxDistance on their own: "our cameras arent so good so i dont think we can see too far so rejecting the ones we cant see might be a smart move." Correct reasoning. Also suggested a different color for tags that are seen but too far to use, which Claude added (dashed yellow rays). Standard deviations ("how much to trust") not covered yet.
- 2026-10-10: Asked "why do we want to make the distance smaller?" (good question). Changed maxDistance to 15 ft themselves (first code edit they typed in a while!) and saw the result: "it rejects too much, we need a sweet spot!" Found the idea of tuning a trade-off on their own (C21 preview).
- 2026-10-10: Noticed on their own that poses get rejected near the hub. When asked to check the tag count, they reported "4 tags, 2 slanted, 2 straight on," which disproved Claude's first guess (single-tag ambiguity). Good habit: checking data instead of accepting a guess. Claude added a rejection-reason readout so they can find the real cause.

## Tutorial feedback
- 2026-10-07 (Phase 0): Student wanted to move fast through setup ("skip this," "its already running"). The setup steps could be shorter for students who already have the sim running.
- 2026-10-07 (Phase 0 step 7): Coach direction: students should focus on Java. Claude writes the dashboard JS from now on (the student still picks what to show). Consider changing Phase 0 step 7 and the dashboard milestones in the plan to match.
- 2026-10-07 (Phase 1): Student answers in short phrases and often says "skip" or "whats next." Short, concrete questions (fill-in-the-blank, puzzles with numbers) got real answers. Open-ended "explain in your own words" questions got skipped.
- 2026-10-10 (Phase 2): Student asked for instructions as "one paragraph, then bullet points with more detail." Consider using that format for multi-part steps.
- 2026-10-10 (Phase 2): Steps 3–5 produce nothing visible on the dashboard until step 6, and the student got impatient. Consider moving the subsystem and one command earlier (sim IO → subsystem → command → see it spin), and doing the real-hardware IO after that.
- 2026-10-10 (Phase 2 step 7): Students asked Claude to "humanize" and "code switch": "we are not pros so we dont understand everything." Claude's messages had gotten too long and full of jargon (tables, line numbers, terms like stator/open-loop without explanation). Switched to a shorter, casual, plain-English style.
- 2026-10-10: Even a one-line exercise with all the pieces given got "write it for us." The students may need a stronger nudge, or a coach's expectation, to type code themselves. Consider asking them to type Claude's line by hand, or to predict before Claude writes.
- 2026-10-10 (Phase 5): A real debugging session. Students noticed the auto restarting. Claude's hypothesis (heartbeat timeout disables the robot) was tested with a dashboard counter before changing any code, and confirmed (19 restarts). This was a good live example of "measure first, then fix." Next: change one variable at a time (camera streams off). → A coach found the real cause: a second open dashboard was sending "disabled." Claude's guess about *why* was wrong. Worth telling students: the counter proved *what* happened, and the *why* still needed more digging.
