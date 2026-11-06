// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.sim.FieldBoundaries;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKraken;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.intake.IntakeArm;
import frc.robot.subsystems.intake.IntakeArmIO;
import frc.robot.subsystems.intake.IntakeArmIOSim;
import frc.robot.subsystems.intake.IntakeArmIOSpark;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.ArrayList;
import java.util.List;
import frc.robot.util.AimHelper;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final IntakeRoller intakeRoller;
  private final IntakeArm intakeArm;
  private final Spindexer spindexer;
  private final Shooter shooter;
  private final Feeder feeder;
  private final Vision vision;

  // True while the right trigger (shoot) is held, so aiming can x-lock the
  // wheels.
  private boolean shootHeld = false;

  // Sim-only walls around the field and hubs
  private final FieldBoundaries fieldBoundaries = new FieldBoundaries();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // Show the drive characterization routines in the auto list? (see the constructor)
  private static final boolean SHOW_CHARACTERIZATION_AUTOS = false;

  // Names of the commands PathPlanner autos can use (see configureNamedCommands)
  private final List<String> namedCommandNames = new ArrayList<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOKraken(0, "FrontLeft"),
            new ModuleIOKraken(1, "FrontRight"),
            new ModuleIOKraken(2, "BackLeft"),
            new ModuleIOKraken(3, "BackRight"));
        intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
        intakeArm = new IntakeArm(new IntakeArmIOSpark());
        spindexer = new Spindexer(new SpindexerIOSpark());
        shooter = new Shooter(new ShooterIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(cameraFrontRight, robotToCameraFrontRight),
            new VisionIOPhotonVision(cameraBackRight, robotToCameraBackRight),
            new VisionIOPhotonVision(cameraFrontLeft, robotToCameraFrontLeft),
            new VisionIOPhotonVision(cameraBackLeft, robotToCameraBackLeft));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
        intakeArm = new IntakeArm(new IntakeArmIOSim());
        spindexer = new Spindexer(new SpindexerIOSim());
        shooter = new Shooter(new ShooterIOSim());
        feeder = new Feeder(new FeederIOSim());
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(cameraFrontRight, robotToCameraFrontRight, drive::getPose),
            new VisionIOPhotonVisionSim(cameraBackRight, robotToCameraBackRight, drive::getPose),
            new VisionIOPhotonVisionSim(cameraFrontLeft, robotToCameraFrontLeft, drive::getPose),
            new VisionIOPhotonVisionSim(cameraBackLeft, robotToCameraBackLeft, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {
        });
        intakeArm = new IntakeArm(new IntakeArmIO() {
        });
        spindexer = new Spindexer(new SpindexerIO() {
        });
        shooter = new Shooter(new ShooterIO() {
        });
        feeder = new Feeder(new FeederIO() {
        });
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIO() {
            },
            new VisionIO() {
            },
            new VisionIO() {
            },
            new VisionIO() {
            });
        break;
    }

    // Set up auto routines (named commands must be registered before building autos)
    configureNamedCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Characterization and SysId routines (for measuring the drivetrain). Hidden from the
    // auto list so drivers can't pick them by accident. Set SHOW_CHARACTERIZATION_AUTOS to
    // true when you need to re-measure (for example, the wheel radius).
    if (SHOW_CHARACTERIZATION_AUTOS) {
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    configureButtonBindings();
  }

  /** Gives commands names that PathPlanner autos can use. Names must match exactly. */
  private void configureNamedCommands() {
    // NOTE: during an auto, PathPlanner owns every subsystem its named commands use, so those
    // subsystems' default commands don't run until auto ends. That's why each command below
    // sets everything it needs itself, and the feeder/spindexer/roller stop when it ends.

    // Event marker zone: drop the intake and run the roller while driving over balls.
    // Never ends on its own, so use it as a zone, not as an auto step.
    registerNamedCommand("RunIntake", intakeRoller.intake().alongWith(intakeArm.extend()));

    // Event marker near the end of a path: spin the flywheel up early, while still driving,
    // so it's at speed when the robot stops. Doesn't use the drive, so it's safe as a marker.
    registerNamedCommand("AimAndPrepShooter", shooter.spinAt(this::hubRpm));

    // Auto step right before a shot: stand still and turn our back to the hub until aimed
    // (max 1 s). Uses the drive, so it must be its own step, never a marker.
    registerNamedCommand("AimAtHub", DriveCommands.aimAtHubInPlace(drive, 1.0));

    // Auto steps: keep the flywheel at speed, wait until READY, then feed + spindexer + arm
    // pulse + roller keep-inside (like the right trigger), for this many seconds in total.
    // Doesn't use the drive, so it also works as a marker during a path.
    registerNamedCommand("ShootPartialHopper", shootFor(2.0));
    registerNamedCommand("ShootFullHopper", shootFor(6.5));

    // The dashboard reads this list to know which commands to show
    Logger.recordOutput("Auto/NamedCommands/Names", namedCommandNames.toArray(new String[0]));
  }

  /** Registers a named command, logging whether it's running. */
  private void registerNamedCommand(String name, Command command) {
    String key = "Auto/NamedCommands/" + name + "/Running";
    Logger.recordOutput(key, false);
    namedCommandNames.add(name);
    NamedCommands.registerCommand(
        name,
        command
            .beforeStarting(() -> Logger.recordOutput(key, true))
            .finallyDo(() -> Logger.recordOutput(key, false)));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // ===== Same controls as the 2026 robot =====
    // (Our shooter faces the BACK, so every aim turns the robot's back to the
    // target.)

    // A (hold): auto-aim. Turn to the hub and spin up for that distance (SHOOT), or
    // turn to
    // our wall from the neutral zone (PASS). You can still drive with the left
    // stick.
    controller
        .a()
        .whileTrue(
            DriveCommands.autoAim(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> shootHeld)
                .alongWith(shooter.spinAt(() -> AimHelper.aimRpm(drive.getPose(), alliance()))));

    // Y / X / B (hold): preset shots for when vision is off. Face a set angle and
    // spin up for
    // a set distance. Angles are the 2026 ones (blue, red); AimHelper turns them
    // 180°.
    controller.y().whileTrue(presetShot(0, 180, 10.5)); // tower
    controller.x().whileTrue(presetShot(-90, 90, 11.5)); // left trench
    controller.b().whileTrue(presetShot(100, -100, 11.5)); // right trench

    // Right bumper (hold): turn to the closest diagonal to drive over the bump.
    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> AimHelper.bumpHeading(drive.getRotation()),
                () -> false));

    // Back: reset heading. Point the robot away from you and press it.
    controller.back().onTrue(DriveCommands.resetHeading(drive));

    // Left trigger (hold): intake. Arm out + roller in.
    // Left bumper (hold): outtake. Arm out + roller out.
    // D-pad down (tap): stow the intake on/off.
    intakeRoller.setDefaultCommand(intakeRoller.stop());
    // When nothing else is using the arm: dropped (extended) during auto so the robot can
    // pick up balls along its paths, and retracted the rest of the time.
    intakeArm.setDefaultCommand(intakeArm.extendWhile(DriverStation::isAutonomousEnabled));
    controller.leftTrigger().whileTrue(intakeRoller.intake().alongWith(intakeArm.extend()));
    controller.leftBumper().whileTrue(intakeRoller.outtake().alongWith(intakeArm.extend()));
    controller.povDown().toggleOnTrue(intakeArm.stow().alongWith(intakeRoller.stop()));

    // Right trigger (hold): shoot (like 2026). Feeder + spindexer push pieces into
    // the
    // flywheel, the intake arm pulses between out and stowed to push leftover balls
    // up into
    // the spindexer, and the roller gently keeps them in. While it's held, aiming
    // x-locks the
    // wheels once it's on target.
    shooter.setDefaultCommand(shooter.idle());
    feeder.setDefaultCommand(feeder.stop());
    spindexer.setDefaultCommand(spindexer.stop());
    controller
        .rightTrigger()
        .whileTrue(
            feeder
                .feed()
                .alongWith(spindexer.feed(), intakeArm.pulse(), intakeRoller.keepInside())
                .beforeStarting(() -> shootHeld = true)
                .finallyDo(() -> shootHeld = false));

    // Extra (2026 didn't have this): D-pad up (hold) runs the spindexer backwards
    // to unjam.
    controller.povUp().whileTrue(spindexer.unjam());
  }

  /** Flywheel speed for a shot at our hub from where the robot is now (from the RPM table). */
  private double hubRpm() {
    return AimHelper.shooterRpmForHub(drive.getPose(), alliance());
  }

  /**
   * Shoot for a while (for autos): hold the flywheel at the hub speed, wait until it's READY,
   * then feed. Once feeding starts it keeps going, because each shot dips the speed a little.
   */
  private Command shootFor(double seconds) {
    return Commands.parallel(
            shooter.spinAt(this::hubRpm),
            Commands.waitUntil(shooter::isReady)
                .andThen(
                    Commands.parallel(
                        feeder.feed(),
                        spindexer.feed(),
                        intakeArm.pulse(),
                        intakeRoller.keepInside())))
        .withTimeout(seconds);
  }

  /** Our alliance, or blue if the Driver Station hasn't told us yet. */
  private Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /**
   * A preset shot: face a set heading (from the 2026 robot's blue/red angles) and
   * spin the
   * flywheel for a set distance, while still driving with the left stick.
   */
  private Command presetShot(double heading2026BlueDeg, double heading2026RedDeg, double feet) {
    double rpm = AimHelper.shooterRpmForDistance(Units.feetToMeters(feet));
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> AimHelper.presetHeading(heading2026BlueDeg, heading2026RedDeg, alliance()),
        () -> shootHeld)
        .alongWith(shooter.spinAt(rpm));
  }

  /** Called every loop, only in simulation. */
  public void simulationPeriodic() {
    fieldBoundaries.apply(drive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
