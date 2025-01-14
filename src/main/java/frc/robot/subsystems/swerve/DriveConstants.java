package frc.robot.subsystems.swerve;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // measures in meters (per sec) and radians (per sec)
  public static final DrivebaseConfig DRIVE_CONFIG =
      switch (getRobotType()) {
        case PROG, SIM -> new DrivebaseConfig(
            Units.inchesToMeters(2),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(38.5),
            Units.inchesToMeters(33),
            5, // FIXME
            5);
        case ALPHA -> new DrivebaseConfig(
            Units.inchesToMeters(2),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(38.5),
            Units.inchesToMeters(33),
            // 5.4764, // FIXME
            // 6.7759);
            4,
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
        case PROG -> new ModuleConfig[] {
          new ModuleConfig(5, 6, 1, new Rotation2d(-0.1503), false, false),
          new ModuleConfig(7, 8, 2, new Rotation2d(-0.18254), false, true),
          new ModuleConfig(11, 12, 3, new Rotation2d(2.9314), false, false),
          new ModuleConfig(9, 10, 4, new Rotation2d(2.2426), false, true)
        };
        case ALPHA -> new ModuleConfig[] {
          new ModuleConfig(5, 6, 1, new Rotation2d(1.1612), true, false),
          new ModuleConfig(7, 8, 2, new Rotation2d(0.8099), true, true),
          new ModuleConfig(11, 12, 3, new Rotation2d(1.4327), true, false),
          new ModuleConfig(9, 10, 4, new Rotation2d(-1.8392), true, true)
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
        case PROG, SIM -> new ModuleConstants(
            new Gains(0, 0, 0, 50, 0, 0), // revisit kP
            new MotionProfileGains(4, 64, 640), // revisit all
            new Gains(0, 0, 0, 0, 0, 0), // FIXME placeholder, to do
            12.8,
            6.75,
            3.125);
        case ALPHA -> new ModuleConstants(
            new Gains(0, 0, 0, 50, 0, 0),
            new MotionProfileGains(4, 64, 640),
            new Gains(0, 0, 0, 1.5, 0, 0),
            5.357142857142857,
            21.428571428571427,
            3.125);
      };

  public static final TrajectoryFollowerConstants TRAJECTORY_CONFIG =
      switch (getRobotType()) {
        case PROG, SIM -> new TrajectoryFollowerConstants(0, 0, 0, 0);
        case ALPHA -> new TrajectoryFollowerConstants(0, 0, 0, 0);
      };

  public record DrivebaseConfig(
      double wheelRadius,
      double trackWidth,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxAngularVelocity) {}

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

  private enum Mk4iReductions {
    MK4I_L3((50 / 14) * (16 / 28) * (45 / 15)),
    STEER(150 / 7);

    double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
