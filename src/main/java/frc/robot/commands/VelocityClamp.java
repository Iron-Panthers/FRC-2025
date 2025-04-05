package frc.robot.commands;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class VelocityClamp extends Command {
  private final Drive drive;

  public VelocityClamp(Drive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    DriveConstants.HOLONOMIC_DRIVE_CONTROLLER.reset(
        RobotState.getInstance().getEstimatedPose(), drive.getRobotSpeeds());
  }

  @Override
  public void execute() {
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();

    PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
    targetState.fieldSpeeds = new ChassisSpeeds();
    targetState.pose = estimatedPose;

    drive.setTrajectorySpeeds(
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER.calculateRobotRelativeSpeeds(
            estimatedPose, targetState));
  }

  @Override
  public boolean isFinished() {
    ChassisSpeeds robotSpeeds = drive.getRobotSpeeds();
    double speed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
    return speed < 2;
  }
}
