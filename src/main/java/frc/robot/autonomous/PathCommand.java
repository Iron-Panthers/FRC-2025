package frc.robot.autonomous;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.BooleanSupplier;

public class PathCommand extends Command {
  private final Drive drive;
  private final BooleanSupplier flipAlliance;
  private final PathPlannerPath originalPath;
  private final RobotConfig robotConfig;
  private final EventScheduler eventScheduler;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;

  private Timer timer = new Timer();

  public PathCommand(
      PathPlannerPath path, BooleanSupplier flipAlliance, Drive drive, RobotConfig robotConfig) {
    this.drive = drive;
    this.robotConfig = robotConfig;
    this.flipAlliance = flipAlliance;
    this.originalPath = path;
    eventScheduler = new EventScheduler();

    addRequirements(drive);

    var eventRequirements = EventScheduler.getSchedulerRequirements(path);
    if (eventRequirements.contains(drive)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventRequirements);
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


    eventScheduler.initialize(trajectory);

    drive.setTrajectory(trajectory);

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    eventScheduler.execute(timer.get());
  }

  @Override
  public boolean isFinished() {
    return drive.isTrajectoryComplete();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();

    drive.clearTrajectory();

    eventScheduler.end();
  }
}
