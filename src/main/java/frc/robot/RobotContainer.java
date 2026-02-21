// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CalibrationMode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.ClimberTW;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.commands.DriveToTower;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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

  private final ClimberTW climber = null;
  private final Spindexer spindexer = new Spindexer();
  private final IntakeRoller intakeRoller = null;
  private final IntakeArm intakeArm = null;
  private final Shooter shooter = null;
  private final Feeder feeder = null;

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
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));

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
            new VisionIOPhotonVisionSim(cameraBackRight, robotToCameraBackRight, drive::getPose),
            new VisionIOPhotonVisionSim(cameraFrontLeft, robotToCameraFrontLeft, drive::getPose),
            new VisionIOPhotonVisionSim(cameraBackLeft, robotToCameraBackLeft, drive::getPose));

        climberSub = new ClimberTW();
        intakeRoller = new IntakeRoller();
        intakeArm = new IntakeArm();
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

    bindCommandsForTeleop();
    bindCommandsForAuto();
  }

  /**
   * Separate the auto command binding to ensure we don't accidentally
   * share commands between teleop and auto.
   */
  void bindCommandsForAuto() {
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.a().whileTrue(new DriveToTower(drive)).onFalse(new InstantCommand(() -> drive.stop(), drive));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller.back().whileTrue(drive.recalibrateDrivetrain());
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
