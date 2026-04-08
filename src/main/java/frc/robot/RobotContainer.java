// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.vision.VisionConstants.cameraBackLeft;
import static frc.robot.subsystems.vision.VisionConstants.cameraBackRight;
import static frc.robot.subsystems.vision.VisionConstants.cameraFrontLeft;
import static frc.robot.subsystems.vision.VisionConstants.cameraFrontRight;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraBackLeft;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraBackRight;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraFrontLeft;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraFrontRight;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final IntakeRoller intakeRoller;
  private final IntakeArm intakeArm;
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();

  // Make xlock work.
  private boolean isRequestingXLock = false;

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

    intakeArm = new IntakeArm(drive);
    intakeRoller = new IntakeRoller(drive);
    bindCommandsForTeleop();
    bindCommandsForAuto();
  }

  /**
   * Separate the auto command binding to ensure we don't accidentally
   * share commands between teleop and auto.
   */
  void bindCommandsForAuto() {

    // Used only in old autos. Do NOT use these commands anymore.
    NamedCommands.registerCommand("runIntake", runIntakeFaster().withTimeout(0));
    NamedCommands.registerCommand("stopIntake", stopIntake().withTimeout(0));
    NamedCommands.registerCommand("autoShoot", autoShootWithRePreppedFlywheels().withTimeout(1));

    // Used in old autos AND new (3V + 5V) autos.
    NamedCommands.registerCommand("autoPrepFlywheels", autoPrepFlywheels().withTimeout(0));
    NamedCommands.registerCommand("stopAllShooting", stopAllShooting().withTimeout(0));

    // New commands added for Contra Costa to make our 3V and 5V autos work
    NamedCommands.registerCommand("shootWithAutoAimBriefly", shootWithAutoAimForAutonomous(6.5));
    NamedCommands.registerCommand("shootWithAutoAimLonger", shootWithAutoAimForAutonomous(10));
    NamedCommands.registerCommand("runIntakeWithSafety", enableIntakeWithSafety());
    NamedCommands.registerCommand("stopIntakeWithSafety", stopIntakeWithSafety());

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
    controller.rightTrigger(DriveTeamConstants.XBOX_TRIGGER_SENSITIVITY).whileTrue(autoShoot());
    controller.leftTrigger(DriveTeamConstants.XBOX_TRIGGER_SENSITIVITY).whileTrue(runIntake());
    controller.x().whileTrue(driveWithLeftTrenchManualAim());
    controller.y().whileTrue(driveWithTowerManualAim());
    controller.b().whileTrue(driveWithRightTrenchManualAim());
    controller.a().whileTrue(driveWithAutoAim());
    controller.povDown().toggleOnTrue(stowIntake());
    controller.rightBumper().whileTrue(driveWithBumpAngle());
    controller.leftBumper().whileTrue(runOuttake());
    // For tuning arm:
    // controller.povUp().toggleOnTrue(extendIntake());
    // controller.povRight().toggleOnTrue(retractIntake());
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
    return retractIntake().withTimeout(0.5).andThen(autoChooser.get());
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

  public AngularVelocity getExpectedShooterVelocityForHub() {
    return SemiAutoHelper.getShooterVelocityForHubDistance(drive);
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
    return shooter.setVelocity(() -> SemiAutoHelper.getShooterVelocityForHubDistance(drive));
  }

  public Command autoShootWithRePreppedFlywheels() {
    return autoPrepFlywheels().withTimeout(0.2).andThen(autoShoot());
  }

  public Command stopAllShooting() {
    return shooter.set(ControlsConstants.SHOOTER_DEFAULT_DUTY_CYCLE)
        .alongWith(feeder.set(ControlsConstants.FEEDER_DEFAULT_DUTY_CYCLE))
        .alongWith(spindexer.set(ControlsConstants.SPINDEXER_DEFAULT_DUTY_CYCLE));
  }

  public Command enableIntakeWithSafety() {
    return Commands.parallel(
        intakeArm.setDesiredAutonomousState(IntakeArm.AutonomousState.INTAKE),
        intakeRoller.setDesiredAutonomousState(IntakeRoller.AutonomousState.INTAKE));
  }

  public Command stopIntakeWithSafety() {
    return Commands.parallel(
        intakeArm.setDesiredAutonomousState(IntakeArm.AutonomousState.CHILLOUT),
        intakeRoller.setDesiredAutonomousState(IntakeRoller.AutonomousState.CHILLOUT));
  }

  public Command disableAutonomousStateSafeties() {
    return Commands.parallel(
        intakeArm.setDesiredAutonomousState(IntakeArm.AutonomousState.OFF),
        intakeRoller.setDesiredAutonomousState(IntakeRoller.AutonomousState.OFF));
  }

  public Command shootWithAutoAimForAutonomous(double howLongInSeconds) {
    Command autonomousAutoAimPrep = Commands.parallel(
        shooter.setVelocity(() -> SemiAutoHelper.getFullAutoShooterVelocity(drive)),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> 0,
            () -> 0,
            () -> SemiAutoHelper.getFullAutoDriveAngle(drive),
            () -> false));
    Command autonomousAutoAimHold = Commands.parallel(
        shooter.setVelocity(() -> SemiAutoHelper.getFullAutoShooterVelocity(drive)),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> 0,
            () -> 0,
            () -> SemiAutoHelper.getFullAutoDriveAngle(drive),
            () -> false));
    Command pullShootingTrigger = Commands.parallel(intakeArm.pulseArm(),
        intakeRoller.keepFuelInside(),
        feeder.setVelocity(SemiAutoHelper.getFeederVelocityForHubDistance(drive)),
        spindexer.spinToShooter());
    return autonomousAutoAimPrep.withTimeout(0.3)
        // disable autonomous state periodic, so trigger can pulse the intakeArm
        .andThen(disableAutonomousStateSafeties())
        .andThen(Commands.parallel(autonomousAutoAimHold, pullShootingTrigger))
        .withTimeout(howLongInSeconds)
        // send intake back to normal so it stops pulsing
        .andThen(stopIntakeWithSafety());
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

  public Command driveWithBumpAngle() {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> SemiAutoHelper.getAngleToDriveOverBump(drive),
        () -> false);
  }

  public Command driveWithAutoAim() {
    return Commands.parallel(
        shooter.setVelocity(() -> SemiAutoHelper.getFullAutoShooterVelocity(drive)),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> SemiAutoHelper.getFullAutoDriveAngle(drive),
            () -> isRequestingXLock));
  }

  public Command driveWithLeftTrenchManualAim() {
    return Commands.parallel(
        shooter.setVelocity(() -> SemiAutoHelper.getShooterVelocity(Feet.of(11.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(isRedAlliance() ? 90 : -90),
            () -> isRequestingXLock));
  }

  public Command driveWithRightTrenchManualAim() {
    return Commands.parallel(
        shooter.setVelocity(() -> SemiAutoHelper.getShooterVelocity(Feet.of(11.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(isRedAlliance() ? -100 : 100),
            () -> isRequestingXLock));
  }

  public Command driveWithTowerManualAim() {
    return Commands.parallel(
        shooter.setVelocity(() -> SemiAutoHelper.getShooterVelocity(Feet.of(10.5))),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(isRedAlliance() ? 180 : 0),
            () -> isRequestingXLock));
  }

  public Command resetPoseFacingAway() {
    return drive.recalibrateDrivetrain();
  }

  public Command xLock() {
    return Commands.runOnce(drive::stopWithX, drive);
  }

  public Command stowIntake() {
    return Commands.parallel(
        intakeArm.stowIntake(),
        intakeRoller.stopFeeding());
  }

  // ========================================================================
  // COMMANDS FOR TELEOP *AND* AUTO
  // ========================================================================

  public Command runIntake() {
    return Commands.parallel(
        extendIntake(),
        // NOTE: no longer need agitate with new dome spindexer
        // spindexer.agitate(),
        intakeRoller.feedToSpindexer());
  }

  public Command runOuttake() {
    return Commands.parallel(
        extendIntake(),
        intakeRoller.spillFuelOutside());
  }

  public Command runIntakeFaster() {
    return Commands.parallel(
        extendIntake(),
        // NOTE: no longer need agitate with new dome spindexer
        // spindexer.agitate(),
        intakeRoller.feedToSpindexerFaster());
  }

  public Command stopIntake() {
    return Commands.parallel(
        retractIntake(),
        intakeRoller.stopFeeding());
  }

  public Command autoShoot() {
    return Commands.parallel(
        this.requestXLock(),
        intakeArm.pulseArm(),
        intakeRoller.keepFuelInside(),
        feeder.setVelocity(SemiAutoHelper.getFeederVelocityForHubDistance(drive)),
        spindexer.spinToShooter());
  }

  // Helpers for xlocking
  public Command requestXLock() {
    return Commands.run(this::requestXlockWhenAimingAtHub).finallyDo(this::dropXLock);
  }

  public void requestXlockWhenAimingAtHub() {
    // only x-lock when shooting at the hub. Neutral zone we can pass on
    // the fly so we don't want the trigger to xlock us.
    boolean isAimingAtHub = !SemiAutoHelper.isInNeutralZone(drive);
    if (isAimingAtHub) {
      isRequestingXLock = true;
    } else {
      isRequestingXLock = false;
    }
  }

  public void dropXLock() {
    isRequestingXLock = false;
  }

  private Command retractIntake() {
    return intakeArm.retractIntake();
  }

  private Command extendIntake() {
    return intakeArm.extendIntake();
  }

}
