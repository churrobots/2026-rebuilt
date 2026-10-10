// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private final Vision vision;

  // Tutorial: how many times B has been pressed
  private int bPressCount = 0;

  // Sim-only walls around the field and hubs
  private final FieldBoundaries fieldBoundaries = new FieldBoundaries();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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

    configureButtonBindings();
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

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero,
                () -> false));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Intake roller: stop by default, intake on left trigger, outtake on left bumper
    intakeRoller.setDefaultCommand(intakeRoller.stop());
    controller.leftTrigger().whileTrue(intakeRoller.intake());
    controller.leftBumper().whileTrue(intakeRoller.outtake());
    // Panic button: tap Y to spit for 1 second
    controller.y().onTrue(intakeRoller.outtake().withTimeout(1.0));
    

    // Tutorial: count B presses (onTrue runs once per press)
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  bPressCount++;
                  Logger.recordOutput("Tutorial/BPressCount", bPressCount);
                }));

    // Tutorial: show whether B is held (whileTrue runs until release)
    controller
        .b()
        .whileTrue(
            Commands.startEnd(
                () -> Logger.recordOutput("Tutorial/BHeld", true),
                () -> Logger.recordOutput("Tutorial/BHeld", false)));
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
