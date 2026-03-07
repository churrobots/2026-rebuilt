// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  // Helpers for automatic aiming and shooting
  private final SemiAutoHelper semiAutoHelper;

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

    NamedCommands.registerCommand(
        "runIntake",
        intakeArm.setAngle(ControlsConstants.INTAKE_ARM_EXTENDED_ANGLE).alongWith(
            intakeRoller.setVelocity(ControlsConstants.INTAKE_ROLLER_VELOCITY)));

    NamedCommands.registerCommand(
        "stopIntake",
        intakeArm.setAngle(ControlsConstants.INTAKE_ARM_DEFAULT_ANGLE).alongWith(
            intakeRoller.set(ControlsConstants.INTAKE_ROLLER_DEFAULT_DUTY_CYCLE)));

    NamedCommands.registerCommand(
        "autoPrepFlywheels",
        shooter.setVelocity(semiAutoHelper::getShooterVelocityForHubDistance));

    // TODO: do we want to have a feeder.setVelocityBasedOnFlywheelVelocity?
    NamedCommands.registerCommand(
        "autoShoot",
        feeder.setVelocity(ControlsConstants.FEEDER_VELOCITY)
            .alongWith(spindexer.setVelocity(ControlsConstants.SPINDEXER_VELOCITY)));

    NamedCommands.registerCommand(
        "stopAllShooting",
        shooter.set(ControlsConstants.SHOOTER_DEFAULT_DUTY_CYCLE)
            .alongWith(feeder.set(ControlsConstants.FEEDER_DEFAULT_DUTY_CYCLE))
            .alongWith(spindexer.set(ControlsConstants.SPINDEXER_DEFAULT_DUTY_CYCLE)));

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

    // TODO: reenable tunables if we want to try speeds again
    // TunableNumber tunableShooterSpeed = new TunableNumber("SHOOTER_SPEED",
    // ControlsConstants.SHOOTER_VELOCITY.in(RPM));
    // TunableNumber tunableFeederSpeed = new TunableNumber("FEEDER_SPEED",
    // ControlsConstants.FEEDER_VELOCITY.in(RPM));
    // TunableNumber tunableSpindexerSpeed = new TunableNumber("SPINDEXER_SPEED",
    // ControlsConstants.SPINDEXER_VELOCITY.in(RPM));

    // TODO: what is resetGyro useful for?
    // Command resetGyro = Commands.runOnce(
    // () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
    // Rotation2d.kZero)), drive)
    // .ignoringDisable(true);
    // TODO: bring back autoclimb
    // Command autoGoToClimb = new DriveToTower(drive, semiAutoHelper).andThen(new
    // InstantCommand(drive::stop, drive));

    Command driveWithJoysticks = DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX());

    Command driveWithAutoAim = Commands.parallel(
        shooter.setVelocity(() -> semiAutoHelper.getShooterVelocityForHubDistance()),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> semiAutoHelper.getAngleToHub()));

    Command driveWithLeftTrenchManualAim = Commands.parallel(
        shooter.setVelocity(() -> semiAutoHelper.getShooterVelocity(Feet.of(11.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(90)));

    Command driveWithRightTrenchManualAim = Commands.parallel(
        shooter.setVelocity(() -> semiAutoHelper.getShooterVelocity(Feet.of(11.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(-90)));

    Command driveWithTowerManualAim = Commands.parallel(
        shooter.setVelocity(() -> semiAutoHelper.getShooterVelocity(Feet.of(11.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(180)));

    Command resetPoseFacingAway = drive.recalibrateDrivetrain();
    Command xLock = Commands.runOnce(drive::stopWithX, drive);
    Command intake = Commands.parallel(
        intakeArm.extendIntake(),
        intakeRoller.feedToShooter());
    Command shoot = Commands.parallel(
        feeder.setVelocity(semiAutoHelper::getFeederVelocityForHubDistance),
        spindexer.feedToShooter());

    drive.setDefaultCommand(driveWithJoysticks);

    controller.back().whileTrue(resetPoseFacingAway);
    controller.leftBumper().whileTrue(xLock);
    controller.rightTrigger(0.75).whileTrue(shoot);
    controller.leftTrigger(0.75).whileTrue(intake);
    controller.x().whileTrue(driveWithLeftTrenchManualAim);
    controller.y().whileTrue(driveWithTowerManualAim);
    controller.b().whileTrue(driveWithRightTrenchManualAim);
    controller.a().whileTrue(driveWithAutoAim);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

}
