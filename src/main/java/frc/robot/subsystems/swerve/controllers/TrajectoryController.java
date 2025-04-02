package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;

public class TrajectoryController {
  private final Drive drive;

  private PathPlannerTrajectory trajectory;
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private PIDController rotationController;
  private Timer timer = new Timer();

  public TrajectoryController(PathPlannerTrajectory trajectory, Drive drive) {
    this.drive = drive;
    this.trajectory = trajectory;
    xController =
        new ProfiledPIDController(
            TRAJECTORY_CONFIG.linearPID().kP,
            0,
            TRAJECTORY_CONFIG.linearPID().kD,
            TRAJECTORY_CONFIG.motionProfileConstraints());
    yController =
        new ProfiledPIDController(
            TRAJECTORY_CONFIG.linearPID().kP,
            0,
            TRAJECTORY_CONFIG.linearPID().kD,
            TRAJECTORY_CONFIG.motionProfileConstraints());
    rotationController =
        new PIDController(
            TRAJECTORY_CONFIG.rotationPID().kP, 0, TRAJECTORY_CONFIG.rotationPID().kD);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    timer.reset();
    timer.start();
  }

  // reference pathplannerlib/**/PPHolonomicDriveController.java
  public ChassisSpeeds update() {
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    // sample trajectory
    PathPlannerTrajectoryState setpointState = trajectory.sample(timer.get());
    ChassisSpeeds setpointSpeeds = setpointState.fieldSpeeds;

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

    PPLibTelemetry.setCurrentPose(currentPose);
    PathPlannerLogging.logCurrentPose(currentPose);

    PPLibTelemetry.setTargetPose(setpointState.pose);
    PathPlannerLogging.logTargetPose(setpointState.pose);

    ChassisSpeeds currentSpeeds = drive.getRobotSpeeds();
    double speed = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    PPLibTelemetry.setVelocities(
        speed,
        setpointState.linearVelocity,
        currentSpeeds.omegaRadiansPerSecond,
        setpointSpeeds.omegaRadiansPerSecond);

    return outputSpeeds;
  }

  public boolean isFinished() {
    double distance =
        RobotState.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(trajectory.sample(timer.get()).pose.getTranslation());
    double speed =
        Math.hypot(
            drive.getRobotSpeeds().vxMetersPerSecond, drive.getRobotSpeeds().vyMetersPerSecond);

    return timer.get() > trajectory.getTotalTimeSeconds()
        && distance < TRAJECTORY_TOLERANCE
        && (speed < TRAJECTORY_SPEED_CUTOFF
            && drive.getRobotSpeeds().omegaRadiansPerSecond < TRAJECTORY_SPEED_CUTOFF);
  }
}
