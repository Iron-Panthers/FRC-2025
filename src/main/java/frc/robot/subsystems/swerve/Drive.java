package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.swerve.DriveConstants.KINEMATICS;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.controllers.HeadingController;
import frc.robot.subsystems.swerve.controllers.TeleopController;
import frc.robot.subsystems.swerve.controllers.TrajectoryController;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public enum DriveModes {
    TELEOP,
    TRAJECTORY;
  }

  private DriveModes driveMode = DriveModes.TELEOP;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private Module[] modules = new Module[4];

  @AutoLogOutput(key = "Swerve/ArbitraryYaw")
  private Rotation2d arbitraryYaw = new Rotation2d();

  @AutoLogOutput(key = "Swerve/YawOffset")
  private Rotation2d gyroYawOffset = new Rotation2d();

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  private final TeleopController teleopController;
  private TrajectoryController trajectoryController = null;
  private HeadingController headingController = null;

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;

    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);

    teleopController = new TeleopController(() -> arbitraryYaw);
  }

  @Override
  public void periodic() {
    // update inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    arbitraryYaw =
        Rotation2d.fromDegrees(
            (gyroInputs.yawPosition.minus(gyroYawOffset).getDegrees() % 360 + 360) % 360);

    for (Module module : modules) {
      module.updateInputs();
    }

    // pass odometry data to robotstate
    SwerveModulePosition[] wheelPositions =
        Arrays.stream(modules)
            .map(module -> module.getModulePosition())
            .toArray(SwerveModulePosition[]::new);
    RobotState.getInstance()
        .addOdometryMeasurement(
            new RobotState.OdometryMeasurement(
                wheelPositions, arbitraryYaw, Timer.getFPGATimestamp()));

    switch (driveMode) {
      case TELEOP -> {
        targetSpeeds = teleopController.update();
        if (headingController != null) {
          // 0.0001 to make the wheels stop in a diamond shape instead of straight so they do not
          // vibrate
          targetSpeeds.omegaRadiansPerSecond = headingController.update() + 0.0001;
        }
      }
      case TRAJECTORY -> {
        targetSpeeds = trajectoryController.update();
        // add heading controll override
      }
    }

    // run modules

    /* use kinematics to get desired module states */
    ChassisSpeeds discretizedSpeeds =
        ChassisSpeeds.discretize(targetSpeeds, Constants.PERIODIC_LOOP_SEC);

    SwerveModuleState[] moduleTargetStates = KINEMATICS.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleTargetStates, DRIVE_CONFIG.maxLinearVelocity());

    for (int i = 0; i < modules.length; i++) {
      modules[i].runToSetpoint(moduleTargetStates[i]);
    }
    Logger.recordOutput("Swerve/ModuleStates", moduleTargetStates);
    Logger.recordOutput("Swerve/TargetSpeeds", targetSpeeds);
    Logger.recordOutput("Swerve/DriveMode", driveMode);
    Logger.recordOutput(
        "Swerve/Magnitude",
        MathUtil.clamp(
            Math.hypot(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond), 0, 3));
    if (headingController != null) {
      Logger.recordOutput(
          "Swerve/HeadingTarget", headingController.getTargetHeading().getRadians());
      Logger.recordOutput("Swerve/HeadingOutput", headingController.update());
    }
  }

  public void driveTeleopController(double xAxis, double yAxis, double omega) {
    if (DriverStation.isTeleopEnabled()) {
      if (driveMode != DriveModes.TELEOP) {
        driveMode = DriveModes.TELEOP;
      }
      teleopController.acceptJoystickInput(xAxis, yAxis, omega);
    }
  }

  public void setTrajectory(PathPlannerTrajectory trajectory) {
    if (DriverStation.isAutonomousEnabled()) {
      driveMode = DriveModes.TRAJECTORY;
      trajectoryController = new TrajectoryController(trajectory);
    }
  }

  public void clearTrajectory() {
    driveMode = DriveModes.TELEOP;
    trajectoryController = null;
  }

  public boolean isTrajectoryComplete() {
    return trajectoryController != null && trajectoryController.isFinished();
  }

  private void zeroGyro() {
    gyroYawOffset = gyroInputs.yawPosition;
    headingController = null;
  }

  public Command zeroGyroCommand() {
    return this.runOnce(() -> zeroGyro());
  }

  @AutoLogOutput(key = "Swerve/ModuleStates")
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules)
        .map(module -> module.getModuleState())
        .toArray(SwerveModuleState[]::new);
  }

  @AutoLogOutput(key = "Swerve/RobotSpeeds")
  public ChassisSpeeds getRobotSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public double setTargetHeading(Rotation2d targetHeading) {
    if (headingController == null) {
      headingController = new HeadingController(targetHeading);
    } else {
      headingController.setTargeHeading(targetHeading);
    }

    return targetHeading.getRadians();
  }

  public void clearHeadingControl() {
    headingController = null;
  }
}
