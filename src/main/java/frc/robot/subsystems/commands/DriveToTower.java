package frc.robot.subsystems.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveToTower extends Command {

  private Drive drive;
  private Command pathCommand;

  public DriveToTower(Drive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    try {
      PathPlannerPath path;
      if (drive.isByOutpost()) {
        path = PathPlannerPath.fromPathFile("outpost to tower");
      } else {
        path = PathPlannerPath.fromPathFile("depot to tower");
      }
      if (drive.isByBlueAlliance()) {
        path = path.flipPath();
      }
      pathCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (Exception e) {
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }
}