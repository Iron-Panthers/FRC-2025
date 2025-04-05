package frc.robot.subsystems.swerve;

import static frc.robot.Constants.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;
import java.util.ArrayList;
import java.util.List;

public class DriveConstants {
  // measures in meters (per sec) and radians (per sec)
  public static final DrivebaseConfig DRIVE_CONFIG =
      switch (getRobotType()) {
        case COMP -> new DrivebaseConfig(
            Units.inchesToMeters(1.925),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(34),
            Units.inchesToMeters(34),
            4.5, // FIXME
            5,
            20);
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
            2.7,
            5,
            3);
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
          new ModuleConfig(
              CAN.at(19, "FL Drive"),
              CAN.at(18, "FL Steer"),
              2,
              new Rotation2d(-1.148),
              true,
              false),
          new ModuleConfig(
              CAN.at(17, "FR Drive"),
              CAN.at(16, "FR Steer"),
              1,
              new Rotation2d(-0.405),
              true,
              true),
          new ModuleConfig(
              CAN.at(21, "BL Drive"),
              CAN.at(20, "BLSteer"),
              3,
              new Rotation2d(1.0139),
              true,
              false),
          new ModuleConfig(
              CAN.at(23, "BR Drive"), CAN.at(22, "BRSteer"), 4, new Rotation2d(-2.8148), true, true)
        };
        case PROG -> new ModuleConfig[] {
          new ModuleConfig(
              CAN.at(5, "FR Drive"),
              CAN.at(6, "FR Steer"),
              1,
              new Rotation2d(-0.1503),
              false,
              false),
          new ModuleConfig(
              CAN.at(7, "FL Drive"),
              CAN.at(8, "FL Steer"),
              2,
              new Rotation2d(-0.18254),
              false,
              true),
          new ModuleConfig(
              CAN.at(11, "BR Drive"),
              CAN.at(12, "BR Steer"),
              3,
              new Rotation2d(2.9314),
              false,
              false),
          new ModuleConfig(
              CAN.at(9, "BL Drive"), CAN.at(10, "BL Steer"), 4, new Rotation2d(2.2426), false, true)
        };
        case ALPHA -> new ModuleConfig[] {
          new ModuleConfig(
              CAN.at(5, "FR Drive"), CAN.at(6, "FR Steer"), 1, new Rotation2d(1.1612), true, false),
          new ModuleConfig(
              CAN.at(7, "FL Drive"), CAN.at(8, "FL Steer"), 2, new Rotation2d(0.8099), true, true),
          new ModuleConfig(
              CAN.at(11, "BR Drive"),
              CAN.at(12, "BR Steer"),
              3,
              new Rotation2d(1.4327),
              true,
              false),
          new ModuleConfig(
              CAN.at(9, "BL Drive"), CAN.at(10, "BL Steer"), 4, new Rotation2d(-1.8392), true, true)
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
            new Gains(0.16, 0.67, 0, 1.5, 0, 0),
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
        case COMP -> new TrajectoryFollowerConstants(
            new PIDConstants(8, 0), new PIDConstants(4, 0));
        case ALPHA -> new TrajectoryFollowerConstants(
            new PIDConstants(13, 0), new PIDConstants(11, 0));
        default -> new TrajectoryFollowerConstants(new PIDConstants(0, 0), new PIDConstants(0, 0));
      };

  public static final HeadingControllerConstants HEADING_CONTROLLER_CONSTANTS =
      switch (getRobotType()) {
        case COMP -> new HeadingControllerConstants(6, 0, 5, 200, 0.002);
        case ALPHA -> new HeadingControllerConstants(3, 0, 5, 200, 0.002);
        default -> new HeadingControllerConstants(0, 0, 0, 0, 0);
      };

  public static final double[] REEF_SNAP_ANGLES = {-120, -60, 0, 60, 120, 180};

  public static final Pose2d INITAL_POSE = new Pose2d(2.9, 3.8, new Rotation2d());

  public static final PPHolonomicDriveController HOLONOMIC_DRIVE_CONTROLLER =
      new PPHolonomicDriveController(
          TRAJECTORY_CONFIG.linearPID(),
          TRAJECTORY_CONFIG.rotationPID(),
          Constants.PERIODIC_LOOP_SEC);

  public static final PathConstraints PP_PATH_CONSTRAINTS =
      new PathConstraints(
          3, 3, Units.degreesToRadians(540), Units.degreesToRadians(5000), 12, false);

  public static final PathConstraints ALIGN_PATH_CONSTRAINTS =
      new PathConstraints(
          3, 2, Units.degreesToRadians(540), Units.degreesToRadians(720), 12, false);

  public static final PathConstraints APPROACH_PATH_CONSTRAINTS =
      new PathConstraints(
          1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720), 12, false);

  public static final Translation2d BLUE_REEF_ORIGIN = new Translation2d(4.5, 4.025);

  public static final Translation2d REEF_TRANSLATION2D =
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
          ? new Translation2d(4.5, 4)
          : new Translation2d(13, 4);

  public static final Translation2d LEFT_CORNER =
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
          ? new Translation2d(0, 8)
          : new Translation2d(17.5, 0);

  public static final Translation2d RIGHT_CORNER =
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
          ? new Translation2d(0, 0)
          : new Translation2d(17.5, 8);

  public static final Translation2d CLIMB_ZONE_CENTER =
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
          ? new Translation2d(8.765, 6)
          : new Translation2d(8.765, 2);

  // blue alliance, will automatically flip, clockwise from top right (vertical)
  public static final ApproachPose[] REEF_APPROACH_POSES =
      ApproachPose.fromPose2ds(
          new Pose2d(6.05, 4.175, Rotation2d.kZero),
          new Pose2d(6.05, 3.85, Rotation2d.kZero),
          new Pose2d(5.4, 2.775, Rotation2d.fromDegrees(-60)),
          new Pose2d(5.1, 2.6, Rotation2d.fromDegrees(-60)),
          new Pose2d(3.85, 2.625, Rotation2d.fromDegrees(-120)),
          new Pose2d(3.575, 2.775, Rotation2d.fromDegrees(-120)),
          new Pose2d(2.95, 3.85, Rotation2d.kPi),
          new Pose2d(2.95, 4.175, Rotation2d.kPi),
          new Pose2d(3.575, 5.275, Rotation2d.fromDegrees(120)),
          new Pose2d(3.85, 5.45, Rotation2d.fromDegrees(120)),
          new Pose2d(5.1, 5.45, Rotation2d.fromDegrees(60)),
          new Pose2d(5.4, 5.275, Rotation2d.fromDegrees(60)));

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

  public record TrajectoryFollowerConstants(PIDConstants linearPID, PIDConstants rotationPID) {}

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

  public record ApproachPose(Pose2d pose) {
    public static ApproachPose[] fromPose2ds(Pose2d... poses) {
      List<ApproachPose> approachPoses = new ArrayList<ApproachPose>();
      for (Pose2d pose : poses) {
        approachPoses.add(new ApproachPose(pose));
      }
      return approachPoses.toArray(new ApproachPose[approachPoses.size()]);
    }

    public Pose2d getAlliancePose() {
      return DriverStation.getAlliance().get() == Alliance.Red
          ? FlippingUtil.flipFieldPose(pose)
          : pose;
    }

    public Pose2d getPose() {
      return pose;
    }

    public PathPlannerPath generatePath() {
      // approach @ 12 inch off, advance to 6 in.
      List<Waypoint> waypoints =
          PathPlannerPath.waypointsFromPoses(
              getAlliancePose(), getAlliancePose().exp(new Twist2d(0.1524, 0, 0)));

      return new PathPlannerPath(
          waypoints,
          PP_PATH_CONSTRAINTS,
          new IdealStartingState(2, getAlliancePose().getRotation()),
          new GoalEndState(0, getAlliancePose().getRotation()));
    }
  }
}
