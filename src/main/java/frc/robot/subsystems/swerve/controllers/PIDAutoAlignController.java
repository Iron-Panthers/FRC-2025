package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import java.util.function.Supplier;

public class PIDAutoAlignController {

  // supplies the position values
  private ProfiledPIDController controller;
  private Supplier<Pose2d> positionSupplier;

  // target position
  private Pose2d targetPosition;
  private double xVel;
  private double yVel;
  private final Supplier<Rotation2d> yawSupplier;

  public PIDAutoAlignController(
      Supplier<Pose2d> positionSupplier, Supplier<Rotation2d> yawSupplier, Pose2d targetPosition) {
    this.positionSupplier = positionSupplier;
    this.targetPosition = targetPosition;
    this.yawSupplier = yawSupplier;
    // setting up the ProfiledPIDCawontroller
    controller =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            0,
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
  }
  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    // basically this code says that
    // yVel    (y-y1)
    // ---- =  ------
    // xVel    (x-x1)
    double dy = positionSupplier.get().getY() - targetPosition.getY();
    double dx = positionSupplier.get().getX() - targetPosition.getX();
    double desiredSlope = (dy / dx);
    if (dx > dy) { // will take longer to get to X than to Y
      xVel = controller.calculate(positionSupplier.get().getX(), targetPosition.getX());
      yVel = xVel * desiredSlope;
    } else { // will take longer to get to Y than to X
      yVel = controller.calculate(positionSupplier.get().getY(), targetPosition.getY());
      xVel = yVel / desiredSlope;
    }
  }

  // update the values
  public ChassisSpeeds update() {
    calculateLinearMovement();
    xVel = Math.abs(xVel) > 0.02 ? xVel : 0;
    yVel = Math.abs(yVel) > 0.02 ? yVel : 0;
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(-xVel, -yVel, 0), yawSupplier.get());
  }
  // log your data in advantage kit
  public Pose2d getTargetPosition() {
    return targetPosition;
  }

  public void setTargetPosition(Pose2d targetPosition) {
    this.targetPosition = targetPosition;
  }
}
