# Learnings
Student: (not given yet)

## Concept checklist
Legend: [ ] not yet · [~] seen, still shaky · [x] understood

- [ ] C01 · The FRC control system
- [ ] C02 · Robot modes and the 20 ms loop
- [ ] C03 · Java you'll see in robot code
- [ ] C04 · Units and coordinates
- [ ] C05 · Git and branches
- [ ] C06 · Subsystems
- [ ] C07 · Commands and requirements
- [ ] C08 · Triggers and button bindings
- [ ] C09 · Default commands
- [ ] C10 · Composing commands
- [ ] C11 · RobotContainer
- [ ] C12 · IO layers (hardware abstraction)
- [~] C13 · Logging inputs and outputs (2026-10-07, Phase 0 step 6)
- [ ] C14 · AdvantageScope and log replay
- [ ] C15 · Motors and motor controllers
- [ ] C16 · Current limits and safety
- [ ] C17 · Encoders, gear ratios, and zero offsets
- [ ] C18 · Open-loop vs. closed-loop control
- [ ] C19 · PID feedback control
- [ ] C20 · Feedforward
- [ ] C21 · Tuning a mechanism
- [ ] C22 · System identification (SysId)
- [ ] C23 · Motion profiles
- [ ] C24 · Swerve drive
- [ ] C25 · Kinematics and field-relative driving
- [ ] C26 · Odometry
- [ ] C27 · Writing IO layers vs. using a mechanism library
- [ ] C28 · Driver input shaping
- [ ] C29 · AprilTags
- [ ] C30 · Camera transforms and pose estimation
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

### C42 · Dashboards (Phase 0 step 7)
- 2026-10-07 (Phase 0 step 7): Student added `SPEED_TOPIC` to the redraw list and renamed it to match the other names. Missed the `/AdvantageKit/RealOutputs/` prefix twice. A coach then asked Claude to write the rest of the dashboard code so students can focus on Java.

### C41 · Simulation / C42 · Dashboards
- 2026-10-07 (Phase 0 steps 3–5): Sim was already running. Student drove in teleop and reported "all good." Didn't answer the field-relative prediction question (push forward while robot faces sideways), so C25 is still untested.
### C13 · Logging inputs and outputs
- 2026-10-07 (Phase 0 step 6): Found `getChassisSpeeds()` on their own. Answered the 3-4-5 speed puzzle correctly ("5m/s"). Filled in the fill-in-the-blank snippet correctly (vx/vy with `Math.hypot`) and logged `Tutorial/Drive/SpeedMetersPerSec`. Hasn't seen the dashboard side yet.

## Tutorial feedback
- 2026-10-07 (Phase 0): Student wanted to move fast through setup ("skip this," "its already running"). The setup steps could be shorter for students who already have the sim running.
- 2026-10-07 (Phase 0 step 7): Coach direction: students should focus on Java. Claude writes the dashboard JS from now on (the student still picks what to show). Consider changing Phase 0 step 7 and the dashboard milestones in the plan to match.
