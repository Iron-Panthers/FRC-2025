package frc.robot.autonomous;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.BooleanSupplier;

public class PathCommand extends Command {
  private final Drive drive;
  private final BooleanSupplier flipAlliance;
  private final PathPlannerPath originalPath;
  private RobotConfig robotConfig;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;


  public PathCommand(
      PathPlannerPath path, BooleanSupplier flipAlliance, Drive drive, RobotConfig robotConfig) {
    this.drive = drive;
    this.robotConfig = robotConfig;
    this.flipAlliance = flipAlliance;
    this.originalPath = path;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    if (flipAlliance.getAsBoolean() && !originalPath.preventFlipping)
      path = originalPath.flipPath();
    else path = originalPath;

    trajectory =
        path.generateTrajectory(
            drive.getRobotSpeeds(),
            RobotState.getInstance().getOdometryPose().getRotation(),
            robotConfig);

    drive.setTrajectory(trajectory);
  }

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
