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
        case COMP, PROG, SIM -> new DrivebaseConfig(
            Units.inchesToMeters(2),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(38.5),
            Units.inchesToMeters(33),
            5, // FIXME
            5);
        case DEV -> new DrivebaseConfig(
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
        case COMP -> new ModuleConfig[] {
          new ModuleConfig(5, 6, 1, new Rotation2d(1.1397), true, false),
          new ModuleConfig(7, 8, 2, new Rotation2d(0.8038), true, true),
          new ModuleConfig(11, 12, 3, new Rotation2d(1.4327), true, false),
          new ModuleConfig(9, 10, 4, new Rotation2d(-1.8208), true, true)
        };
        case PROG -> new ModuleConfig[] {
          new ModuleConfig(5, 6, 1, new Rotation2d(-2.2656), false, true),
          new ModuleConfig(7, 8, 2, new Rotation2d(-0.1794), false, true),
          new ModuleConfig(11, 12, 3, new Rotation2d(2.9406), false, true),
          new ModuleConfig(9, 10, 4, new Rotation2d(-0.1365), false, true)
        };
        case DEV -> new ModuleConfig[] {
          new ModuleConfig(2, 1, 27, new Rotation2d(0), true, false),
          new ModuleConfig(13, 12, 26, new Rotation2d(0), true, true),
          new ModuleConfig(4, 3, 24, new Rotation2d(0), true, false),
          new ModuleConfig(11, 10, 25, new Rotation2d(0), true, true)
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
        case COMP, SIM -> new ModuleConstants(
            new Gains(0.25, 2.62, 0, 100, 0, 0), // revisit kP
            new MotionProfileGains(4, 64, 640), // revisit all
            new Gains(0.3, 0.63, 0, 2, 0, 0), // FIXME placeholder, to do
            5.357142857142857,
            21.428571428571427,
            3.125);
        case PROG -> new ModuleConstants(
            new Gains(0.4, 0.6, 0, 11, 0, 0),
            new MotionProfileGains(4, 64, 640),
            new Gains(0.3, 0.11, 0, 1.5, 0, 0),
            5.357142857142857,
            21.428571428571427,
            3.125);
        case DEV -> new ModuleConstants(
            new Gains(0, 0, 0, 11, 0, 0),
            new MotionProfileGains(0, 0, 0),
            new Gains(0, 0, 0, 1.5, 0, 0),
            5.357142857142857,
            21.428571428571427,
            3.125);
      };

  /**
   * Configuration for the drivebase.
   *
   * @param wheelRadius The radius of the wheels.
   * @param trackWidth The width of the track.
   * @param bumperWidthX The width of the bumper in the X direction.
   * @param bumperWidthY The width of the bumper in the Y direction.
   * @param maxLinearVelocity The maximum linear velocity.
   * @param maxAngularVelocity The maximum angular velocity.
   */
  public record DrivebaseConfig(
        double wheelRadius,
        double trackWidth,
        double bumperWidthX,
        double bumperWidthY,
        double maxLinearVelocity,
        double maxAngularVelocity) {}
  
  /**
   * Configuration for a module.
   *
   * @param driveID The ID of the drive motor.
   * @param steerID The ID of the steer motor.
   * @param encoderID The ID of the encoder.
   * @param absoluteEncoderOffset The offset for the absolute encoder.
   * @param steerInverted Whether the steering is inverted.
   * @param driveInverted Whether the drive is inverted.
   */
  public record ModuleConfig(
        int driveID,
        int steerID,
        int encoderID,
        Rotation2d absoluteEncoderOffset,
        boolean steerInverted,
        boolean driveInverted) {}
  
  /**
   * Constants for a module.
   *
   * @param steerGains The gains for steering.
   * @param steerMotionGains The motion profile gains for steering.
   * @param driveGains The gains for driving.
   * @param driveReduction The reduction ratio for driving.
   * @param steerReduction The reduction ratio for steering.
   * @param couplingGearReduction The reduction ratio for the coupling gear.
   */
  public record ModuleConstants(
        Gains steerGains,
        MotionProfileGains steerMotionGains,
        Gains driveGains,
        double driveReduction,
        double steerReduction,
        double couplingGearReduction) {}

  /**
   * Constants for the trajectory follower.
   */
  public record TrajectoryFollowerConstants() {}
  
  /**
   * PID gains for a controller.
   *
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kP The proportional gain.
   * @param kI The integral gain.
   * @param kD The derivative gain.
   */
  public record Gains(double kS, double kV, double kA, double kP, double kI, double kD) {}
  
  /**
   * Motion profile gains for a controller.
   *
   * @param cruiseVelocity The cruise velocity.
   * @param acceleration The acceleration.
   * @param jerk The jerk.
   */
  public record MotionProfileGains(double cruiseVelocity, double acceleration, double jerk) {}
  
  /**
   * Enum representing the gear reductions for the Mk4i module.
   */
  private enum Mk4iReductions {
    /**
     * Gear reduction for the Mk4i L3 configuration.
     */
    MK4I_L3((50 / 14) * (16 / 28) * (45 / 15)),
    
    /**
     * Gear reduction for the steering.
     */
    STEER(150 / 7);
  
    double reduction;
  
    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
