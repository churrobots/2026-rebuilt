# FRC Robot Concepts

These are the ideas you need to understand to build a competition robot from scratch. You don't have to learn them all at once. The [tutorial plan](TUTORIAL_PLAN.md) introduces each one right before you need it.

Each concept has:

- **Level.** *Core* means every programmer should understand it. *Deeper* is needed to tune or debug. *Stretch* is advanced and optional.
- **The idea.** A plain-language explanation.
- **In our robot.** Where our code uses it, on a reference branch (see `AGENTS.md`). Most of it is on `main` (the 2026 competition robot). The new robot's Kraken drivetrain is on `unleash-the-kraken`, and the turret prototype is on `churret`. Look at a file with `git show <branch>:<path>`.
- **Check yourself.** A question you should be able to answer in your own words.
- **Learn more.** Official docs. WPILib links are relative to `https://docs.wpilib.org/en/stable/docs/`.

---

## A. Foundations

### C01 · The FRC control system
**Level:** Core
**The idea:** The **roboRIO** is the robot's brain; it runs our Java code. The **radio** connects it to the **Driver Station** laptop. The **power distribution hub (PDH)** sends battery power to everything through breakers. Motor controllers (SPARK MAX, SPARK Flex, TalonFX) are daisy-chained on the **CAN bus**, a shared wire where every device has a unique **CAN ID**, like a house number on a street.
**In our robot:** `HardwareConstants.java` and `drive/DriveConstants.java` list every CAN ID.
**Check yourself:** Two motor controllers both have CAN ID 11. What goes wrong?
**Learn more:** `controls-overviews/control-system-hardware.html`, `zero-to-robot/step-1/intro-to-frc-robot-wiring.html`, `software/can-devices/index.html`

### C02 · Robot modes and the 20 ms loop
**Level:** Core
**The idea:** A robot is always in one mode: **disabled**, **autonomous**, **teleop**, or **test**. Our code doesn't run once from top to bottom. WPILib calls the `periodic()` methods **50 times a second** (every 20 ms), forever. Each loop reads sensors, makes decisions, and sets motor outputs. Code that takes too long (like `Thread.sleep`) causes a "loop overrun" and makes the whole robot laggy.
**In our robot:** `Robot.java` has `robotPeriodic()`, `autonomousInit()`, `teleopInit()`, and so on.
**Check yourself:** Why is a `while` loop that waits for a button press a bad idea inside `periodic()`?
**Learn more:** `zero-to-robot/step-4/creating-test-drivetrain-program-cpp-java-python.html`

### C03 · Java you'll see in robot code
**Level:** Core
**The idea:** Robot code uses classes (a blueprint, like `Shooter`), objects (one real shooter made from that blueprint), `private` fields (data only the class can touch), and methods. Two newer Java features show up everywhere:
- **Lambdas / suppliers**: `() -> -controller.getLeftY()` means "a recipe for getting the value later." The robot calls it every loop, so it always has a fresh joystick reading.
- **Method references**: `drive::getPose` is shorthand for `() -> drive.getPose()`.

**In our robot:** `RobotContainer.driveWithJoysticks()`
**Check yourself:** Why do we pass `() -> controller.getLeftY()` instead of `controller.getLeftY()`?
**Learn more:** `zero-to-robot/introduction.html` (lists Java learning resources)

### C04 · Units and coordinates
**Level:** Core
**The idea:** Mixing up inches and meters, or degrees and radians, is the most common robot bug. The WPILib **Units library** (`Degrees.of(90)`, `RPM.of(3000)`) makes the unit part of the value. The field uses a standard coordinate system: +X points away from the blue alliance wall, +Y points left, and rotation is **counter-clockwise positive**, measured in meters and radians.
**In our robot:** `IntakeArm.java` (`Degrees.of(...)`), `ControlsConstants.java` (`RPM.of(...)`)
**Check yourself:** The robot turns right when you expect it to turn left. Which convention might be wrong?
**Learn more:** `software/basic-programming/java-units.html`, `software/basic-programming/coordinate-system.html`

### C05 · Git and branches
**Level:** Core
**The idea:** Git saves snapshots of your code, called **commits**. A **branch** is a separate line of work, so you can experiment without breaking the version that works. Commit whenever something works, with a message saying what changed. Our finished robot lives on the `main` branch, and you're rebuilding it on your own branch.
**Check yourself:** You broke something, and it worked an hour ago. How does git help?
**Learn more:** `software/basic-programming/git-getting-started.html`

---

## B. Command-based programming

### C06 · Subsystems
**Level:** Core
**The idea:** A **subsystem** is one physical part of the robot, such as the drivetrain, intake, or shooter. It owns its motors and sensors, and nothing else is allowed to control them directly. This is **encapsulation**: the shooter knows how to spin, and nobody else has to know how.
**In our robot:** Each file in `subsystems/` (`Shooter.java`, `IntakeArm.java`, …)
**Check yourself:** Why shouldn't `RobotContainer` call `shooterMotor.set(0.5)` directly?
**Learn more:** `software/commandbased/what-is-command-based.html`, `software/commandbased/subsystems.html`

### C07 · Commands and requirements
**Level:** Core
**The idea:** A **command** is one action, like "extend the intake" or "spin up to 3000 RPM." It has a lifecycle: `initialize` → `execute` (every loop) → `isFinished?` → `end`. A command **requires** the subsystems it uses. If a new command needs the same subsystem, the old one is **interrupted**, so two commands never fight over one motor.
**In our robot:** `IntakeArm.extendIntake()` returns a command; `Shooter.setVelocity(...)` returns a command.
**Check yourself:** A command to extend the intake is running, and the driver presses "stow." What happens and why?
**Learn more:** `software/commandbased/commands.html`

### C08 · Triggers and button bindings
**Level:** Core
**The idea:** A **trigger** connects a condition (a button pressed, a sensor tripped) to a command. `whileTrue` runs the command while the button is held and cancels it on release. `onTrue` starts it once. `toggleOnTrue` switches it on and off with each press.
**In our robot:** `RobotContainer.bindCommandsForTeleop()`
**Check yourself:** Should the intake run with `whileTrue` or `toggleOnTrue`? What would the driver prefer?
**Learn more:** `software/commandbased/binding-commands-to-triggers.html`, `software/basic-programming/joystick.html`

### C09 · Default commands
**Level:** Core
**The idea:** A **default command** runs on a subsystem whenever nothing else is using it, so the subsystem always knows what to do. Drivetrain: drive with joysticks. Intake arm: retract. Shooter: idle slowly.
**In our robot:** `drive.setDefaultCommand(driveWithJoysticks())`, `setDefaultCommand(retractIntake())` in `IntakeArm`
**Check yourself:** What does the shooter do the moment the driver releases the shoot button?

### C10 · Composing commands
**Level:** Core
**The idea:** You can build big actions out of small commands, like LEGO. `Commands.parallel(a, b)` runs them together. `a.andThen(b)` runs one after the other. `.withTimeout(2)` stops after 2 seconds. `Commands.waitUntil(...)` waits for a condition.
**In our robot:** `RobotContainer.autoShoot()`, `shootWithAutoAimForAutonomous(...)`
**Check yourself:** Write, in plain English, what `autoShoot()` does.
**Learn more:** `software/commandbased/command-compositions.html`

### C11 · RobotContainer
**Level:** Core
**The idea:** `RobotContainer` is where the robot gets put together: it creates the subsystems, binds buttons to commands, and picks the autonomous routine. `Robot.java` stays tiny. This **separation of concerns** means you always know where to look.
**In our robot:** `RobotContainer.java`

---

## C. AdvantageKit

### C12 · IO layers (hardware abstraction)
**Level:** Core
**The idea:** AdvantageKit splits a subsystem into **logic** (for example `Drive.java`) and **IO** (the part that talks to hardware). The IO is an interface with several versions: `ModuleIOSpark` for the real robot, `ModuleIOSim` for simulation, and an empty one for replay. The logic doesn't know or care which one it has. That's how the same code runs in simulation and on the robot.
**In our robot:** `drive/ModuleIO.java`, `ModuleIOSpark.java`, `ModuleIOSim.java`; `vision/VisionIO*.java`. A real example of why this matters: our new robot switched to Kraken drive motors by adding `ModuleIOKraken.java` (branch `unleash-the-kraken`). `Drive.java` didn't change at all. On the new robot, every mechanism (intake, shooter, turret) gets its own IO layer in exactly the same way. The 2026 robot used the YAMS library for mechanisms instead; see C27.
**Check yourself:** You want to test a new drive feature at home without the robot. Which IO does `RobotContainer` pick, and how does it know?
**Learn more:** https://docs.advantagekit.org (start with the Spark Swerve template)

### C13 · Logging inputs and outputs
**Level:** Core
**The idea:** AdvantageKit records everything that goes **into** the code (sensor readings, via `Logger.processInputs`) and anything interesting that comes **out** (via `Logger.recordOutput("Name", value)`). Those values are published to NetworkTables, and that's how our dashboard and AdvantageScope see them. Rule of thumb: if you want to see a value, `recordOutput` it.
**In our robot:** `Shooter.periodic()` logs `Mechanisms/Shooter/VelocityRPM`.
**Check yourself:** You log `Mechanisms/Arm/Angle`. What topic name does the dashboard subscribe to? (Answer: `/AdvantageKit/RealOutputs/Mechanisms/Arm/Angle`)

### C14 · AdvantageScope and log replay
**Level:** Deeper
**The idea:** **AdvantageScope** graphs logged values and shows the robot in 2D or 3D. Plotting *setpoint vs. measured* is how you tune a mechanism. Because every input is logged, AdvantageKit can **replay** a match: it re-runs your code on the recorded inputs to find out why something happened.
**In our robot:** `MechanismVisualizer.java` publishes 3D component poses for AdvantageScope. `Constants.Mode.REPLAY`.

---

## D. Motors and sensors

### C15 · Motors and motor controllers
**Level:** Core
**The idea:** A **motor** (NEO, NEO Vortex, Kraken/Falcon) makes things spin. A **motor controller** (SPARK MAX/Flex for REV, TalonFX for CTRE) decides how much power the motor gets. Each vendor has its own library (a **vendordep**) and a tuning app (REV Hardware Client, Phoenix Tuner). Settings that matter: **inversion** (which way is positive), **idle mode** (**brake** holds position, **coast** spins freely), and the **current limit**.
**In our robot:** `Shooter.java` (TalonFX, coast, 80 A limit), `IntakeArm.java` (SPARK MAX, 40 A)
**Check yourself:** Why is the shooter set to coast, not brake?
**Learn more:** `software/hardware-apis/motors/index.html`, `software/vscode-overview/3rd-party-libraries.html`

### C16 · Current limits and safety
**Level:** Core
**The idea:** A stalled motor draws huge current. That can trip breakers, brown out the robot (the roboRIO shuts down), or melt the motor. A **current limit** caps it. **Soft limits** stop a mechanism in software before it reaches its **hard limits** (physical stops). Always test new code in simulation first, and put the robot on blocks for the first real enable.
**In our robot:** `.withStatorCurrentLimit(...)`, `.withSoftLimit(...)` in `IntakeArm.java`; `DriveConstants.driveMotorCurrentLimit` (the comment explains it was lowered to stop wheel slip)

### C17 · Encoders, gear ratios, and zero offsets
**Level:** Core
**The idea:** An **encoder** measures rotation. A **relative** encoder counts from wherever it started. An **absolute** encoder always knows the true angle, even right after power-on, but needs a **zero offset** to say which reading counts as "0°". A **gear ratio** converts motor rotations to mechanism rotations: with a 25:1 gearbox, the motor turns 25 times for every 1 turn of the arm.
**In our robot:** `IntakeArm` uses an absolute encoder with `.withExternalEncoderZeroOffset(...)` and `.withGearing(...)`. Look at the comment on why its angles were measured empirically.
**Check yourself:** The arm reads 30° when it's actually straight down. What do you change?
**Learn more:** `software/hardware-apis/sensors/encoders-software.html`

### C18 · Open-loop vs. closed-loop control
**Level:** Core
**The idea:** **Open-loop** means "apply 50% power" and hope. The speed changes as the battery drains or a game piece gets stuck. **Closed-loop** means "go to 3000 RPM": the code measures the actual speed and keeps correcting. Open-loop is fine for simple rollers. Anything that has to be *precise* needs closed-loop.
**In our robot:** `Shooter.set(dutyCycle)` (open) vs. `Shooter.setVelocity(...)` (closed)
**Check yourself:** Why does the shooter use closed-loop but a simple roller might not?

---

## E. Control theory

### C19 · PID feedback control
**Level:** Core
**The idea:** **Error** = where you want to be (**setpoint**) − where you are. PID computes the motor output from the error:
- **P (proportional)**: push harder the farther away you are. Too much P and it overshoots and oscillates.
- **I (integral)**: adds up error over time to fix a small steady miss. Use sparingly; it can "wind up."
- **D (derivative)**: reacts to how fast the error is changing, which damps overshoot, like a shock absorber.

A car analogy: P is how hard you steer toward the lane, and D stops you from swerving past it.
**In our robot:** `KP`, `KI`, `KD` in `IntakeArm.java`; `SHOOTER_KP` etc. in `ControlsConstants.java`; the heading controller in `DriveCommands.joystickDriveAtAngle`
**Check yourself:** The arm wobbles back and forth around its target. Which gain do you lower first?
**Learn more:** `software/advanced-controls/introduction/control-system-basics.html`, `software/advanced-controls/introduction/introduction-to-pid.html`, `software/advanced-controls/controllers/pidcontroller.html`

### C20 · Feedforward
**Level:** Core
**The idea:** PID only reacts *after* there's error. **Feedforward** predicts the output you'll need *before* there's error, using physics:
- **kS**: the voltage to overcome friction and just start moving.
- **kV**: the voltage per unit of speed. "To spin at 3000 RPM I need about X volts."
- **kA**: extra voltage to accelerate.
- **kG**: the voltage to hold against gravity (arms and elevators).

Good feedforward does most of the work, and PID just cleans up the rest.
**In our robot:** `SHOOTER_KV` (flywheel, `SimpleMotorFeedforward`); `KG` in `IntakeArm` (`ArmFeedforward`)
**Check yourself:** Why does an arm need kG but a flywheel doesn't? Why does the arm's gravity term depend on its angle?
**Learn more:** `software/advanced-controls/introduction/introduction-to-feedforward.html`, `software/advanced-controls/controllers/feedforward.html`

### C21 · Tuning a mechanism
**Level:** Deeper
**The idea:** Tune in order: (1) feedforward (kS, kG, then kV) until it *almost* tracks the setpoint on its own; (2) raise P until it responds quickly without oscillating; (3) add a little D if it overshoots. Always look at a graph of **setpoint vs. measured**. Change one number at a time. **Tunable numbers** let you change gains live from the dashboard instead of redeploying.
**In our robot:** `util/TunableNumber.java`; `IntakeArm`'s tunable pulsing angles
**Learn more:** `software/advanced-controls/introduction/tuning-flywheel.html`, `software/advanced-controls/introduction/tuning-vertical-arm.html`, `software/advanced-controls/introduction/tuning-elevator.html`, `software/advanced-controls/introduction/common-control-issues.html`

### C22 · System identification (SysId)
**Level:** Deeper
**The idea:** Instead of guessing feedforward gains, **SysId** runs the mechanism through test motions (slow ramps called *quasistatic*, fast steps called *dynamic*) and fits kS, kV, and kA from the data.
**In our robot:** `RobotContainer.bindCommandsForAuto()` adds the SysId and characterization routines to the auto chooser when calibration mode is enabled.
**Learn more:** `software/advanced-controls/system-identification/index.html`

### C23 · Motion profiles
**Level:** Deeper
**The idea:** Jumping a setpoint straight from 0 to 90° slams the mechanism. A **trapezoidal motion profile** ramps the setpoint up to a max velocity, cruises, then slows down, respecting max velocity and max acceleration. It's smoother, safer, and more repeatable.
**In our robot:** the new robot's turret (you'll add its max velocity and acceleration in the turret phase, using WPILib `TrapezoidProfile` or TalonFX Motion Magic)
**Learn more:** `software/advanced-controls/controllers/trapezoidal-profiles.html`

---

## F. Drivetrain

### C24 · Swerve drive
**Level:** Core
**The idea:** Each of our 4 **swerve modules** has a **drive motor** (wheel speed) and a **turn motor** (wheel direction). Because every wheel can point anywhere, the robot can move in any direction while facing any direction. Our new robot uses **Kraken X60** drive motors (on TalonFX controllers) and **NEO 550** turn motors (on SPARK MAX, with an absolute encoder on each module). A Pigeon 2 **gyro** measures which way the robot faces. The 2026 competition robot used NEO Vortex drive motors instead.
**In our robot:** `subsystems/drive/` (started from the AdvantageKit Spark Swerve template); `drive/ModuleIOKraken.java` on `unleash-the-kraken`
**Check yourself:** What does a tank drive robot have to do to move sideways that a swerve robot doesn't?

### C25 · Kinematics and field-relative driving
**Level:** Core
**The idea:** **Kinematics** is the math that turns "robot moves forward at 2 m/s while spinning" (**ChassisSpeeds**) into a speed and angle for each wheel (**module states**). **Field-relative** driving uses the gyro so that pushing the stick forward always moves the robot away from the driver, no matter which way the robot faces. Drivers strongly prefer it.
**In our robot:** `Drive.runVelocity(...)`, `DriveCommands.joystickDrive(...)`
**Learn more:** `software/kinematics-and-odometry/intro-and-chassis-speeds.html`, `software/kinematics-and-odometry/swerve-drive-kinematics.html`

### C26 · Odometry
**Level:** Core
**The idea:** **Odometry** estimates the robot's position (**pose**: x, y, heading) by adding up how far each wheel has rolled and which way the gyro says the robot faces. It's like counting your steps with your eyes closed: good for a few seconds, but wheel slip slowly makes it drift. Vision fixes the drift (C30).
**In our robot:** `Drive.periodic()`; the high-rate `SparkOdometryThread.java`
**Learn more:** `software/kinematics-and-odometry/swerve-drive-odometry.html`

### C27 · Writing IO layers vs. using a mechanism library
**Level:** Deeper
**The idea:** Some teams use a mechanism library like **YAMS**, which bundles the motor controller, PID, feedforward, limits, simulation, and telemetry into ready-made `Arm`, `FlyWheel`, and `Elevator` objects. Our 2026 robot did. That's less code, but more is hidden, and the sensor readings don't go through `Logger.processInputs`, so AdvantageKit can't replay them. On the new robot, **we write every mechanism ourselves as an AdvantageKit IO layer** (C12): an `XxxIO` interface, a real-hardware IO class, and an `XxxIOSim` built on WPILib's physics simulators (`DCMotorSim`, `FlywheelSim`, `SingleJointedArmSim`, `ElevatorSim`). It's more code, but every line is visible, every mechanism follows the same pattern as the drivetrain, and replay works. It's a real engineering trade-off: control vs. convenience.
**In our robot:** the new robot's mechanism `*IO.java` / `*IOSim.java` files (you'll write them), following `drive/ModuleIO*.java`. The 2026 YAMS versions on `main`: `Shooter.java` (`FlyWheel`), `IntakeArm.java` (`Arm`). Public AdvantageKit mechanism examples: team 6328's robot code (https://github.com/Mechanical-Advantage) and other teams' code on chiefdelphi.com.
**Check yourself:** Your arm works in sim but not on the real robot. Which file is most likely wrong, and why doesn't `IntakeArm.java` need to change?
**Learn more:** https://docs.advantagekit.org (IO interfaces), `software/wpilib-tools/robot-simulation/physics-sim.html`

### C28 · Driver input shaping
**Level:** Core
**The idea:** Joysticks never rest at exactly 0, so a **deadband** ignores tiny values. Squaring the input gives fine control near the center and full speed at the edge. Negating Y is normal: pushing a stick forward reads *negative*.
**In our robot:** `DriveCommands.joystickDrive(...)`, the `-controller.getLeftY()` calls in `RobotContainer`

---

## G. Vision and localization

### C29 · AprilTags
**Level:** Core
**The idea:** **AprilTags** are QR-code-like squares at known spots on the field. A camera that sees a tag knows which tag it is and exactly where it is relative to the camera. So it can work out where the *robot* is on the field. The official field layout lists every tag's position.
**In our robot:** `AprilTagFieldLayout.loadField(...)` in `SemiAutoHelper.java`
**Learn more:** `software/vision-processing/apriltag/apriltag-intro.html`

### C30 · Camera transforms and pose estimation
**Level:** Deeper
**The idea:** To go from "where is the tag relative to the *camera*" to "where is the *robot*," you need the **robot-to-camera transform**: exactly where each camera is mounted and how it's tilted. A **pose estimator** blends fast-but-drifting odometry with slower-but-absolute vision. **Standard deviations** tell it how much to trust each vision reading (less when far away or seeing only one tag), and bad readings get **rejected**.
**In our robot:** `vision/VisionConstants.java` (camera transforms; note the git commit "all cameras are flipped on this robot"), `Vision.java` (filtering), `drive::addVisionMeasurement`
**Learn more:** `software/advanced-controls/state-space/state-space-pose-estimators.html`, `software/vision-processing/index.html`

### C31 · PhotonVision
**Level:** Deeper
**The idea:** **PhotonVision** runs on a coprocessor attached to the cameras, finds AprilTags, and sends results over the network. `VisionIOPhotonVisionSim` fakes camera images in simulation, so you can test vision without a robot.
**In our robot:** `vision/VisionIOPhotonVision.java`, `VisionIOPhotonVisionSim.java`

---

## H. Autonomous

### C32 · PathPlanner and named commands
**Level:** Core
**The idea:** **PathPlanner** is an app for drawing paths on the field and chaining them into **autos**. Paths are followed with a holonomic controller, which uses PID on x, y, and heading. **Named commands** let an auto trigger robot actions like "start intake" at markers along the path. An **auto chooser** on the dashboard picks which auto runs.
**In our robot:** `bindCommandsForAuto()` registers `NamedCommands` and builds `AutoBuilder.buildAutoChooser()`; paths are in `deploy/pathplanner/`
**Learn more:** `software/pathplanning/index.html`

### C33 · Mechanism state in autonomous
**Level:** Deeper
**The idea:** In auto, nobody is watching to save the robot. The code has to protect itself, for example by pulling the intake in while driving under the trench. A small **state machine** (a list of named states, with rules for what each state does) keeps this readable.
**In our robot:** `IntakeArm.AutonomousState` and its `periodic()` logic using `SemiAutoHelper.isInTrenchBumpZone`

---

## I. Mechanism patterns

### C34 · Rollers and indexers
**Level:** Core
**The idea:** The simplest mechanism: a motor spins a roller to grab or move game pieces. It usually starts open-loop, then moves to velocity control for consistency. Pick the speed with math, not guesses. The intake roller's surface should move faster than the robot drives.
**In our robot:** `IntakeRoller.java`, `Spindexer.java`, `Feeder.java`; see the RPM math comment above `INTAKE_ROLLER_VELOCITY` in `ControlsConstants.java`

### C35 · Flywheels (velocity control)
**Level:** Core
**The idea:** A **flywheel** shooter needs a precise, repeatable speed, because a different speed means a different shot. It relies mostly on kV feedforward, plus P to recover when a game piece takes speed away. You only feed the game piece once the flywheel is *at speed*.
**In our robot:** `Shooter.java`, `FEEDER_TO_SHOOTER_RPM_RATIO`

### C36 · Arms and elevators (position control)
**Level:** Core
**The idea:** Position mechanisms move to an angle (arm) or a height (elevator) and hold it against gravity. They need kG feedforward, soft limits, and usually an absolute encoder or a **homing** routine, so the code knows where "zero" is.
**In our robot:** `IntakeArm.java` (arm)

### C37 · Superstructure coordination
**Level:** Deeper
**The idea:** Scoring usually takes several subsystems at once: spin the shooter up, aim the drivetrain, *then* run the feeder and spindexer. Build that coordination in `RobotContainer` out of each subsystem's commands. Don't make subsystems reach into each other.
**In our robot:** `RobotContainer.autoShoot()`, `driveWithAutoAim()`, `runIntake()`

### C46 · Turrets
**Level:** Core
**The idea:** A **turret** spins the shooter on top of the robot, so the robot can aim without turning the whole drivetrain. It's a position mechanism like an arm, but it spins flat, so there's **no gravity** and no kG. It usually has a big gear reduction, brake mode, and a motion profile. Three new ideas come with it:
- **Robot-relative vs. field-relative angles.** The turret angle you need = (field angle to the target) − (robot heading). If the robot spins, the turret has to spin the other way to stay on target.
- **Limited range.** Wires and hard stops mean most turrets can't spin forever (ours is planned for 180°). When the target is out of range, decide what to do: go to the *nearest* limit, or rotate the drivetrain to help.
- **Knowing where zero is.** At power-on the code must know which way the turret points, using an absolute encoder or by always starting it in a known position (C17).

**In our robot:** `subsystems/Churret.java` on the `churret` branch, an *unfinished* prototype. It uses a YAMS `Pivot` with a 100:1 reduction, TalonFX, brake mode, a max-velocity/acceleration profile, and `aimAtHub(...)`. The new robot's turret will be built with an AdvantageKit IO layer instead (C27), so use the prototype for its ideas and numbers, not its structure.
**Check yourself:** The robot is facing 90° and the hub is at a field angle of 30°. What turret angle do you need? If the turret can only reach 0–180°, what should it do?
**Learn more:** `software/advanced-controls/introduction/tuning-turret.html`

---

## J. Driver assist

### C38 · Heading lock and auto-aim
**Level:** Deeper
**The idea:** The target angle comes from the robot's pose (C30) and the known target location. There are two ways to point at it:
- **Heading lock** (2026 competition robot): the driver controls where the robot *moves*, and a PID controller turns the *whole robot* to face the target.
- **Turret aim** (new robot): the turret points at the target (C46), and the driver keeps full control of rotation. Heading lock is still useful as a backup when the target is outside the turret's range, and for fixed angles like driving over the bump.

**In our robot:** `DriveCommands.joystickDriveAtAngle(...)`, `SemiAutoHelper.getFullAutoDriveAngle(...)`, `RobotContainer.driveWithAutoAim()`; `Churret.aimAtHub(...)` on `churret`

### C39 · Lookup tables and interpolation
**Level:** Deeper
**The idea:** Physics models of shots are hard, so teams *measure*: "at 100 inches, 2900 RPM scores." They save those pairs in a table, and **interpolation** fills in the gaps (at 105 inches, use about 2925 RPM).
**In our robot:** `InterpolatingDoubleTreeMap lookupDistanceInInchesToRPM` in `SemiAutoHelper.java`

### C40 · Shoot on the move
**Level:** Stretch
**The idea:** When the robot is moving, the game piece inherits the robot's velocity, so you have to aim *ahead* of the target and adjust the speed. It uses projectile physics, and simulation is essential to test it. A turret makes this much more practical, because the robot can keep driving in any direction while the turret aims.
**In our robot:** `subsystems/sotm/`; `Churret.aimWithSOTM(...)` on `churret`

---

## K. Testing, debugging, and competition

### C41 · Simulation
**Level:** Core
**The idea:** Simulation runs your real robot code on a laptop, with physics models standing in for motors and sensors. Test *everything* here first. It's free, fast, and can't break anything. Our `SimSupervisor` automatically restarts the simulator every time you save a file.
**In our robot:** `tools/SimSupervisor.java`, `sim/SimulationControllerBridge.java`, the `simulationPeriodic()` methods
**Learn more:** `software/wpilib-tools/robot-simulation/introduction.html`

### C42 · NetworkTables and dashboards
**Level:** Core
**The idea:** **NetworkTables** is a shared set of named values (**topics**) that the robot and laptops can publish and subscribe to, like a group chat for numbers. Dashboards read these topics to show what the robot is thinking. We have our own browser dashboard at `http://localhost:5800` (sim) or `http://10.TE.AM.2:5800` (robot). In VS Code, the **ChurroDashboard** button opens it. Only `custom-dashboard.js` and `.css` change. You decide what goes on the dashboard and publish the values from Java; an AI assistant like Claude writes the dashboard code for you.

**The dashboard is how you see your work.** Anything the robot knows can go on it: the target and actual speed of a flywheel, whether the arm is at its angle, which command is running, what the cameras see, which motors are faulted. You can show it as numbers, lights, dials, live graphs, or a map of the field. When something doesn't work, your first move should be "let's put that on the dashboard" to see what the robot is *actually* doing. Guessing comes second.
**In our robot:** `src/main/deploy/dashboard/custom-dashboard.js` (on `sandbox-sim`)
**Check yourself:** Your shooter "sometimes" misses. What would you put on the dashboard to figure out why?
**Learn more:** `software/networktables/index.html`, `software/dashboards/index.html`, `software/telemetry/index.html`

### C43 · Fault detection and alerts
**Level:** Deeper
**The idea:** At competition, a loose CAN wire can silently kill a mechanism. Our code checks that each motor controller is really connected and reports faults to the dashboard. With AdvantageKit, each IO layer reports a `connected` input, and the subsystem raises a WPILib `Alert` when it's false, so the rest of the robot keeps working and the drive team sees the problem.
**In our robot:** `driveConnected` / `turnConnected` in `drive/ModuleIO.java` and the alerts in `Module.java`; the "Mechanism faults" dashboard card. The 2026 robot did this with YAMS-specific helpers on `main`: `util/HardwareMonitor.java`, `util/YAMSUtil.java`, `util/DisconnectedMotorController.java`.

### C44 · Deploying and the Driver Station
**Level:** Core
**The idea:** **Deploy** builds your code and copies it to the roboRIO. The **Driver Station** app enables and disables the robot, chooses the mode, and shows battery voltage and errors. **Space bar = emergency stop.** Always know where it is before you enable.
**Learn more:** `zero-to-robot/step-4/running-test-program.html`, `zero-to-robot/step-2/frc-game-tools.html`

### C45 · Software design habits
**Level:** Core
**The idea:** These keep a robot codebase working all season:
- **Constants files**: one place for every tunable number and CAN ID.
- **Encapsulation**: subsystems hide their hardware.
- **Separation of concerns**: bindings in `RobotContainer`, hardware in subsystems, display in the dashboard.
- **Don't repeat yourself.**
- **Small commits.**

**In our robot:** `ControlsConstants.java`, `HardwareConstants.java`, `DriveTeamConstants.java`
