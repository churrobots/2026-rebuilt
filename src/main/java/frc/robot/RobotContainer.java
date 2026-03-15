// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CalibrationMode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.ControlsConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.SemiAutoHelper;
import frc.robot.util.TunableNumber;

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

  private final Drive drive;
  private final Spindexer spindexer = new Spindexer();
  private final IntakeRoller intakeRoller = new IntakeRoller();
  private final IntakeArm intakeArm = new IntakeArm();
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();

  // Tunable speeds.
  TunableNumber tunableShooterRpm = new TunableNumber("SHOOTER_SPEED",
      ControlsConstants.SHOOTER_VELOCITY.in(RPM));
  TunableNumber tunableFeederRpm = new TunableNumber("FEEDER_SPEED",
      ControlsConstants.FEEDER_VELOCITY.in(RPM));
  TunableNumber tunableSpindexerRpm = new TunableNumber("SPINDEXER_SPEED",
      ControlsConstants.SPINDEXER_VELOCITY.in(RPM));
  TunableNumber tunableIntakeRpm = new TunableNumber("INTAKE_SPEED",
      ControlsConstants.INTAKE_ROLLER_VELOCITY.in(RPM));

  // Helpers for automatic aiming and shooting
  private final SemiAutoHelper semiAutoHelper;

  // Make xlock work.
  private boolean isXlocked = false;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(Hardware.DriverStation.driverXboxPort);

  // Dashboard inputs, for calibration only
  private LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOSpark(0, "FrontLeft"),
            new ModuleIOSpark(1, "FrontRight"),
            new ModuleIOSpark(2, "BackLeft"),
            new ModuleIOSpark(3, "BackRight"));

        new Vision(
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

        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(cameraFrontRight, robotToCameraFrontRight, drive::getPose),
            new VisionIOPhotonVisionSim(cameraBackRight, robotToCameraBackRight,
                drive::getPose),
            new VisionIOPhotonVisionSim(cameraFrontLeft, robotToCameraFrontLeft,
                drive::getPose),
            new VisionIOPhotonVisionSim(cameraBackLeft, robotToCameraBackLeft,
                drive::getPose));
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

        // TODO: if we try replay we will probably have to update this
        // (Use same number of dummy implementations as the real robot)
        new Vision(drive::addVisionMeasurement, new VisionIO() {
        }, new VisionIO() {
        });
        break;
    }

    semiAutoHelper = new SemiAutoHelper(drive::getPose);
    bindCommandsForTeleop();
    bindCommandsForAuto();
  }

  /**
   * Separate the auto command binding to ensure we don't accidentally
   * share commands between teleop and auto.
   */
  void bindCommandsForAuto() {

    // Register all our auto commands.
    NamedCommands.registerCommand("runIntake", runIntakeFaster().withTimeout(0));
    NamedCommands.registerCommand("stopIntake", stopIntake().withTimeout(0));
    NamedCommands.registerCommand("autoPrepFlywheels", autoPrepFlywheels().withTimeout(0));
    NamedCommands.registerCommand("autoShoot", autoShootWithRePreppedFlywheels().withTimeout(1));
    NamedCommands.registerCommand("stopAllShooting", stopAllShooting().withTimeout(0));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add SysId routines if we are in Calibration mode
    // TODO: we might want the Wheel Radius characterization more accessible so we
    // can recharacterize in the pit as wheels wear down and get changed
    if (Constants.calibrationMode == CalibrationMode.ENABLED) {
      autoChooser.addOption(
          "Drive Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addOption(
          "Drive Simple FF Characterization",
          DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)",
          drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)",
          drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  void bindCommandsForTeleop() {

    // TODO: what is resetGyro useful for?
    // Command resetGyro = Commands.runOnce(
    // () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
    // Rotation2d.kZero)), drive)
    // .ignoringDisable(true);
    // TODO: bring back autoclimb
    // Command autoGoToClimb = new DriveToTower(drive, semiAutoHelper).andThen(new
    // InstantCommand(drive::stop, drive));

    drive.setDefaultCommand(driveWithJoysticks());

    controller.back().whileTrue(resetPoseFacingAway());
    // controller.leftBumper().whileTrue(xLock());
    controller.rightTrigger(DriveTeamConstants.XBOX_TRIGGER_SENSITIVITY).whileTrue(autoShoot());
    controller.leftTrigger(DriveTeamConstants.XBOX_TRIGGER_SENSITIVITY).whileTrue(runIntake());
    controller.x().whileTrue(driveWithLeftTrenchManualAim());
    controller.y().whileTrue(driveWithTowerManualAim());
    controller.b().whileTrue(driveWithRightTrenchManualAim());
    controller.a().whileTrue(driveWithAutoAim());
    controller.povDown().toggleOnTrue(stowIntake());

    // This is how you can run using our TunableNumbers
    // controller.povLeft().whileTrue(runIntakeWithTunableSpeed());
    // controller.povUp().whileTrue(runSpindexerWithTunableSpeed());
    // controller.povLeft().onTrue(intakeArm.setAngle(Degrees.of(250)));
    // controller.povRight().onTrue(intakeArm.setAngle(Degrees.of(290)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Always make sure to retract the intake before starting ANY auto
    // TODO: this gives an exception every second time you run it
    // @hannah: dropped from 0.8 to 0.5 after the first quals
    return intakeArm.retractIntake().withTimeout(0.5).andThen(autoChooser.get());
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

  public AngularVelocity getExpectedShooterVelocityForHub() {
    return semiAutoHelper.getShooterVelocityForHubDistance();
  }

  public AngularVelocity getActualShooterVelocity() {
    return shooter.getVelocity();
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
  }

  // ========================================================================
  // COMMANDS FOR AUTO
  // ========================================================================

  public Command autoPrepFlywheels() {
    return shooter.setVelocity(semiAutoHelper::getShooterVelocityForHubDistance);
  }

  public Command autoShootWithRePreppedFlywheels() {
    return autoPrepFlywheels().withTimeout(0.2).andThen(autoShoot());
  }

  public Command stopAllShooting() {
    return shooter.set(ControlsConstants.SHOOTER_DEFAULT_DUTY_CYCLE)
        .alongWith(feeder.set(ControlsConstants.FEEDER_DEFAULT_DUTY_CYCLE))
        .alongWith(spindexer.set(ControlsConstants.SPINDEXER_DEFAULT_DUTY_CYCLE));
  }

  // ========================================================================
  // COMMANDS FOR TELEOP
  // ========================================================================

  public Command driveWithJoysticks() {
    return DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX());
  }

  public Command driveWithAutoAim() {
    return Commands.parallel(
        shooter.setVelocity(() -> semiAutoHelper.getShooterVelocityForHubDistance()),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> semiAutoHelper.getAngleToHub(),
            () -> isXlocked));
  }

  public Command driveWithLeftTrenchManualAim() {
    return Commands.parallel(
        shooter.setVelocity(() -> semiAutoHelper.getShooterVelocity(Feet.of(11.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(isRedAlliance() ? 90 : -90),
            () -> isXlocked));
  }

  public Command driveWithRightTrenchManualAim() {
    return Commands.parallel(
        shooter.setVelocity(() -> semiAutoHelper.getShooterVelocity(Feet.of(11.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(isRedAlliance() ? -100 : 100),
            () -> isXlocked));
  }

  public Command driveWithTowerManualAim() {
    return Commands.parallel(
        shooter.setVelocity(() -> semiAutoHelper.getShooterVelocity(Feet.of(10.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(isRedAlliance() ? 180 : 0),
            () -> isXlocked));
  }

  public Command resetPoseFacingAway() {
    return drive.recalibrateDrivetrain();
  }

  public Command xLock() {
    return Commands.runOnce(drive::stopWithX, drive);
  }

  public Command stowIntake() {
    return intakeArm.stowIntake();
  }

  public Command runIntakeWithTunableSpeed() {
    return intakeRoller.setVelocity(() -> RPM.of(tunableIntakeRpm.getLatest()));
  }

  public Command runSpindexerWithTunableSpeed() {
    return spindexer.setVelocity(() -> RPM.of(tunableSpindexerRpm.getLatest()));
  }

  // ========================================================================
  // COMMANDS FOR TELEOP *AND* AUTO
  // ========================================================================

  public Command runIntake() {
    return Commands.parallel(
        intakeArm.extendIntake(),
        spindexer.agitate(),
        intakeRoller.feedToSpindexer());
  }

  public Command runIntakeFaster() {
    return Commands.parallel(
        intakeArm.extendIntake(),
        spindexer.agitate(),
        intakeRoller.feedToSpindexerFaster());
  }

  public Command stopIntake() {
    return Commands.parallel(
        intakeArm.retractIntake(),
        intakeRoller.stopFeeding());
  }

  public Command autoShoot() {
    return Commands.parallel(
        feeder.setVelocity(semiAutoHelper::getFeederVelocityForHubDistance),
        // We don't want to xlock right away since the aim might not have completed
        // fully. Instead give it slightly more time to finish aiming before xlock.
        new WaitCommand(0.5).andThen(this.enableXlock()),
        spindexer.feedToShooter());
  }

  // Helpers for xlocking
  public Command enableXlock() {
    return Commands.run(this::setXlockToTrue).finallyDo(this::setXlockToFalse);
  }

  public void setXlockToTrue() {
    isXlocked = true;
  }

  public void setXlockToFalse() {
    isXlocked = false;
  }
}
