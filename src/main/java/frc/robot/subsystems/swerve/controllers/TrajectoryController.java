package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController {
  private PathPlannerTrajectory trajectory;
  private PIDController yController;
  private PIDController xController;
  private PIDController rotationController;
  private Timer timer = new Timer();

  public TrajectoryController(PathPlannerTrajectory trajectory) {
    this.trajectory = trajectory;
    xController = new PIDController(TRAJECTORY_CONFIG.linearKP(), 0, TRAJECTORY_CONFIG.linearKD());
    yController = new PIDController(TRAJECTORY_CONFIG.linearKP(), 0, TRAJECTORY_CONFIG.linearKD());
    rotationController =
        new PIDController(TRAJECTORY_CONFIG.rotationKP(), 0, TRAJECTORY_CONFIG.rotationKD());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    timer.reset();
    timer.start();
  }

  // reference pathplannerlib/**/PPHolonomicDriveController.java
  public ChassisSpeeds update() {
    Pose2d currentPose =
        RobotState.getInstance().getOdometryPose(); // FIXME change to estimatedpose

    // sample trajectory
    PathPlannerTrajectoryState setpointState = trajectory.sample(timer.get());
    ChassisSpeeds setpointSpeeds = setpointState.fieldSpeeds;

    // error for logging
    double translationError =
        currentPose.getTranslation().getDistance(setpointState.pose.getTranslation());
    Rotation2d rotationError = currentPose.getRotation().minus(setpointState.pose.getRotation());

    // feedback control
    double xFeedback = xController.calculate(currentPose.getX(), setpointState.pose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), setpointState.pose.getY());
    double rotationFeedback =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), setpointState.pose.getRotation().getRadians());

    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            setpointSpeeds.vxMetersPerSecond + xFeedback,
            setpointSpeeds.vyMetersPerSecond + yFeedback,
            setpointSpeeds.omegaRadiansPerSecond + rotationFeedback,
            currentPose.getRotation());

    Logger.recordOutput("Trajectory/SetpointPose", setpointState.pose);
    Logger.recordOutput("Trajectory/SetpointSpeeds", setpointSpeeds);
    Logger.recordOutput("Trajectory/OutputSpeeds", outputSpeeds);
    Logger.recordOutput("Trajectory/TranslationError", translationError);
    Logger.recordOutput("Trajectory/RotationError", rotationError);

    return outputSpeeds;
  }

  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
