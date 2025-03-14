package frc.robot.subsystems.swerve;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // measures in meters (per sec) and radians (per sec)
  public static final DrivebaseConfig DRIVE_CONFIG =
      switch (getRobotType()) {
        case COMP -> new DrivebaseConfig(
            Units.inchesToMeters(2),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(34),
            Units.inchesToMeters(34),
            4.5, // FIXME
            5,
            3);
        case PROG, SIM -> new DrivebaseConfig(
            Units.inchesToMeters(2),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(38.5),
            Units.inchesToMeters(33),
            4.5, // FIXME
            5,
            3);
        case ALPHA -> new DrivebaseConfig(
            Units.inchesToMeters(1.903),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(34),
            Units.inchesToMeters(34),
            // 5.4764, // FIXME
            // 6.7759);
            5,
            5,
            4);
      };

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(DRIVE_CONFIG.trackWidth() / 2.0, DRIVE_CONFIG.trackWidth() / 2.0),
        new Translation2d(DRIVE_CONFIG.trackWidth() / 2.0, -DRIVE_CONFIG.trackWidth() / 2.0),
        new Translation2d(-DRIVE_CONFIG.trackWidth() / 2.0, DRIVE_CONFIG.trackWidth() / 2.0),
        new Translation2d(-DRIVE_CONFIG.trackWidth() / 2.0, -DRIVE_CONFIG.trackWidth() / 2.0)
      }; // meters relative to center, NWU convention; fl, fr, bl, br

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  public static final int GYRO_ID = 0;

  // fl, fr, bl, br; negate offsets
  public static final ModuleConfig[] MODULE_CONFIGS =
      switch (getRobotType()) {
          // FIXME
        case COMP -> new ModuleConfig[] {
          new ModuleConfig(19, 18, 2, new Rotation2d(-1.148), true, false),
          new ModuleConfig(17, 16, 1, new Rotation2d(-0.405), true, true),
          new ModuleConfig(21, 20, 3, new Rotation2d(1.012), true, false),
          new ModuleConfig(23, 22, 4, new Rotation2d(-2.831), true, true)
        };
        case PROG -> new ModuleConfig[] {
          new ModuleConfig(5, 6, 1, new Rotation2d(-0.1503), false, false),
          new ModuleConfig(7, 8, 2, new Rotation2d(-0.18254), false, true),
          new ModuleConfig(11, 12, 3, new Rotation2d(2.9314), false, false),
          new ModuleConfig(9, 10, 4, new Rotation2d(2.2426), false, true)
        };
        case ALPHA -> new ModuleConfig[] {
          new ModuleConfig(7, 8, 2, new Rotation2d(-0.244), true, true),
          new ModuleConfig(5, 6, 1, new Rotation2d(-2.642), true, false),
          new ModuleConfig(11, 12, 3, new Rotation2d(0.9802), true, false),
          new ModuleConfig(9, 10, 4, new Rotation2d(2.023), true, false)
        };
        case SIM -> new ModuleConfig[] {
          new ModuleConfig(0, 0, 0, new Rotation2d(0), true, false),
          new ModuleConfig(0, 0, 0, new Rotation2d(0), true, true),
          new ModuleConfig(0, 0, 0, new Rotation2d(0), true, false),
          new ModuleConfig(0, 0, 0, new Rotation2d(0), true, true)
        };
      };

  public static final ModuleConstants MODULE_CONSTANTS =
      switch (getRobotType()) {
        case COMP -> new ModuleConstants(
            new Gains(0.25, 2.26, 0, 50, 0, 0),
            new MotionProfileGains(4, 64, 640),
            new Gains(0.3, 0.63, 0, 1.5, 0, 0), // FIXME diff gear ratio
            (45.0 / 15) * (17.0 / 27) * (50.0 / 16), // MK4i L2.5 16 tooth
            150.0 / 7,
            3.125);
        case PROG, SIM -> new ModuleConstants(
            new Gains(0.25, 2.26, 0, 50, 0, 0), // revisit kP
            new MotionProfileGains(4, 64, 640), // revisit all
            new Gains(0.3, 0.63, 0, 1.5, 0, 0), // FIXME placeholder, to do
            12.8,
            6.75,
            3.125);
        case ALPHA -> new ModuleConstants(
            new Gains(0.18, 3, 0, 50, 0, 0),
            new MotionProfileGains(4, 64, 640),
            new Gains(0.3, 0.5, 0, 2, 0, 0),
            5.357142857142857,
            21.428571428571427,
            3.125);
      };

  public static final TrajectoryFollowerConstants TRAJECTORY_CONFIG =
      switch (getRobotType()) {
        case COMP -> new TrajectoryFollowerConstants(0, 0, 0, 0);
        case ALPHA -> new TrajectoryFollowerConstants(13, 0, 11, 0);
        default -> new TrajectoryFollowerConstants(0, 0, 0, 0);
      };

  public static final HeadingControllerConstants HEADING_CONTROLLER_CONSTANTS =
      switch (getRobotType()) {
        case COMP -> new HeadingControllerConstants(3, 0, 5, 200, 0.002);
        case ALPHA -> new HeadingControllerConstants(3, 0, 5, 200, 0.002);
        default -> new HeadingControllerConstants(0, 0, 0, 0, 0);
      };

  public static final double[] REEF_SNAP_ANGLES = {-120, -60, 0, 60, 120, 180};

  // FIXME
  public static final Pose2d INITAL_POSE = new Pose2d(2.9, 3.8, new Rotation2d());

  public record DrivebaseConfig(
      double wheelRadius,
      double trackWidth,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxAngularVelocity,
      double maxLinearAcceleration) {}

  public record ModuleConfig(
      int driveID,
      int steerID,
      int encoderID,
      Rotation2d absoluteEncoderOffset,
      boolean steerInverted,
      boolean driveInverted) {}

  public record ModuleConstants(
      Gains steerGains,
      MotionProfileGains steerMotionGains,
      Gains driveGains,
      double driveReduction,
      double steerReduction,
      double couplingGearReduction) {}

  public record TrajectoryFollowerConstants(
      double linearKP, double linearKD, double rotationKP, double rotationKD) {}

  public record Gains(double kS, double kV, double kA, double kP, double kI, double kD) {}

  public record MotionProfileGains(double cruiseVelocity, double acceleration, double jerk) {}

  /* tolerance in degrees */
  public record HeadingControllerConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration, double tolerance) {}

  private enum Mk4iReductions {
    MK4I_L3((50 / 14) * (16 / 28) * (45 / 15)),
    STEER(150 / 7);

    double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
