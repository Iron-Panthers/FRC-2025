package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class TeleopController {
  private final Supplier<Rotation2d> yawSupplier;
  private double controllerX = 0;
  private double controllerY = 0;
  private double controllerOmega = 0;
  private Translation2d pastLinearVelocity = new Translation2d();
  private double clampedVelocityDiff = 0;
  private double acceleration;

  /* teleop control with specified yaw supplier, typically "arbitrary" yaw */
  public TeleopController(Supplier<Rotation2d> yawSupplier) {
    this.yawSupplier = yawSupplier;
  }

  /* accept driver input from joysticks */
  public void acceptJoystickInput(
      double controllerX, double controllerY, double controllerOmega, double acceleration) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.controllerOmega = controllerOmega;
    this.acceleration = acceleration;
  }

  /* accept driver input from joysticks */
  public void acceptJoystickInput(double controllerX, double controllerY, double controllerOmega) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.controllerOmega = controllerOmega;
    this.acceleration = DRIVE_CONFIG.maxLinearAcceleration();
  }

  /* update controller with current desired state */
  public ChassisSpeeds update() {
    Translation2d linearVelocity = calculateLinearVelocity(controllerX, controllerY);

    double omega = MathUtil.applyDeadband(controllerOmega, 0.001);
    omega = Math.copySign(Math.pow(Math.abs(omega), 1.5), omega);

    // acceleration limiting
    Translation2d linearVelocityDiff = linearVelocity.minus(pastLinearVelocity);
    clampedVelocityDiff =
        MathUtil.clamp(
            Math.abs(linearVelocity.getDistance(pastLinearVelocity)),
            0,
            acceleration * (Constants.PERIODIC_LOOP_SEC));
    Rotation2d velocityTheta;
    if (linearVelocityDiff.getX() != 0 || linearVelocityDiff.getY() != 0) {
      velocityTheta = linearVelocityDiff.getAngle();
    } else velocityTheta = new Rotation2d();
    Translation2d newVelocity =
        pastLinearVelocity.plus(new Translation2d(clampedVelocityDiff, velocityTheta));
    pastLinearVelocity = newVelocity;

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        newVelocity.getX() * DRIVE_CONFIG.maxLinearVelocity(),
        newVelocity.getY() * DRIVE_CONFIG.maxLinearVelocity(),
        omega * DRIVE_CONFIG.maxAngularVelocity(),
        yawSupplier.get());
  }

  public Translation2d calculateLinearVelocity(double x, double y) {

    Rotation2d theta;
    if (x != 0 || y != 0) theta = new Rotation2d(x, y);
    else theta = new Rotation2d(0);

    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);

    // apply deadband, raise magnitude to exponent
    magnitude = Math.pow(magnitude, 1.5);

    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), theta)
            .transformBy(new Transform2d(magnitude, 0, new Rotation2d()))
            .getTranslation();
    return linearVelocity;
  }

  @AutoLogOutput(key = "Swerve/Acceleration")
  private double getAccerlation() {
    return clampedVelocityDiff;
  }

  public void setPastLinearVelocity(Translation2d pastLinearVelocity) {
    this.pastLinearVelocity = pastLinearVelocity;
  }
}
