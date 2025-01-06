package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drive;

public class PathCommand extends Command {
  private final Drive drive;
  private final boolean flipAlliance;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;

  public PathCommand(PathPlannerPath path, boolean flipAlliance, Drive drive) {
    this.drive = drive;
    this.flipAlliance = flipAlliance;
    this.path = flipAlliance ? path.flipPath() : path;

    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return drive.isTrajectoryComplete();
  }

  @Override
  public void end(boolean interrupted) {
    drive.clearTrajectory();
  }
}
