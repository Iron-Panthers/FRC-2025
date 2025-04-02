package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.BooleanSupplier;

public class FollowPath extends Command {
  private final Drive drive;
  private final BooleanSupplier flipAlliance;
  private final PathPlannerPath originalPath;
  private final RobotConfig robotConfig;
  private final EventScheduler eventScheduler;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;

  private Timer timer = new Timer();

  public FollowPath(
      PathPlannerPath path, Drive drive, BooleanSupplier flipAlliance, RobotConfig robotConfig) {
    this.originalPath = path;
    this.drive = drive;
    this.flipAlliance = flipAlliance;
    this.robotConfig = robotConfig;
    this.eventScheduler = new EventScheduler();

    this.path = this.originalPath;

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
            RobotState.getInstance().getEstimatedPose().getRotation(),
            robotConfig);

    PathPlannerAuto.setCurrentTrajectory(trajectory);
    PathPlannerAuto.currentPathName = originalPath.name;

    PathPlannerLogging.logActivePath(path);
    PPLibTelemetry.setCurrentPath(path);

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

    PathPlannerAuto.currentPathName = "";
    PathPlannerAuto.setCurrentTrajectory(null);
    PathPlannerLogging.logActivePath(null);

    eventScheduler.end();
  }
}
