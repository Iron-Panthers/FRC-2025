package frc.robot.subsystems.superstructure.pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants;
import java.util.Optional;

public class PivotConstants {
  public static final PivotConfig PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new PivotConfig(8, Optional.of(28), Optional.of(-0.278), 1);
        case ALPHA -> new PivotConfig(15, Optional.empty(), Optional.empty(), 21.6 / 360);
        case PROG -> new PivotConfig(0, Optional.empty(), Optional.empty(), 1);
        case SIM -> new PivotConfig(0, Optional.empty(), Optional.empty(), 1);
      };
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(40, 0, 0, 0, 3.6144, 0.1807, 0.53);
        case ALPHA -> new PIDGains(1.5, 0, 0.01, 0.03, 0.09, 0, 0.51);
        case PROG -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(2, 10);
        case PROG -> new MotionMagicConfig(0, 0);
        case ALPHA -> new MotionMagicConfig(0, 0);
        case SIM -> new MotionMagicConfig(0, 0);
      };

  public record PivotConfig(
      int motorID,
      Optional<Integer> canCoderID,
      Optional<Double> canCoderOffset,
      double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final boolean INVERT_MOTOR = true;

  public static final double POSITION_TARGET_EPSILON = 0.01;

  // SOFT LIMITS
  public static final Optional<Double> UPPER_EXTENSION_LIMIT = Optional.of(0.465);
  public static final Optional<Double> LOWER_EXTENSION_LIMIT = Optional.empty();

  // top limit is 121 rotations

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 6;
  public static final double LOWER_VOLT_LIMIT = -6;
  public static final double SUPPLY_CURRENT_LIMIT = 30;

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = 1;
  public static final double ZEROING_OFFSET = 0; // offset in degrees
  public static final double ZEROING_VOLTAGE_THRESHOLD = 5;
}
