package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TeleopController {

  private double controllerX = 0;
  private double controllerY = 0;
  private double controllerOmega = 0;

  /* accept driver input from joysticks */
  public void acceptJoystickInput(double controllerX, double controllerY, double controllerOmega) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.controllerOmega = controllerOmega;
  }

  /* update controller with current desired state */
  public ChassisSpeeds update(Rotation2d yaw) {
    Translation2d linearVelocity = calculateLinearVelocity(controllerX, controllerY);

    double omega = MathUtil.applyDeadband(controllerOmega, 0.1);
    omega = Math.copySign(Math.pow(Math.abs(omega), 1.5), omega);

    // eventaully run off of pose estimation?
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        linearVelocity.getX() * DRIVE_CONFIG.maxLinearVelocity(),
        linearVelocity.getY() * DRIVE_CONFIG.maxLinearVelocity(),
        omega * DRIVE_CONFIG.maxAngularVelocity(),
        yaw);

    /* ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * DRIVE_CONFIG.maxLinearVelocity(),
            linearVelocity.getY() * DRIVE_CONFIG.maxLinearVelocity(),
            omega * DRIVE_CONFIG.maxAngularVelocity());
    // eventually run off of pose estimation?
    speeds.toRobotRelativeSpeeds(yaw);

    return speeds; */
  }

  public Translation2d calculateLinearVelocity(double x, double y) {
    // apply deadband, raise magnitude to exponent
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    magnitude = Math.pow(magnitude, 1.5);

    Rotation2d theta = new Rotation2d(x, y);

    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), theta)
            .transformBy(new Transform2d(magnitude, 0, new Rotation2d()))
            .getTranslation();
    return linearVelocity;
  }
}
