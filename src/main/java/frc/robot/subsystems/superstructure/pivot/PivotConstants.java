package frc.robot.subsystems.superstructure.pivot;

import java.util.Optional;

import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants;

public class PivotConstants {
  public static final PivotConfig PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new PivotConfig(15, Optional.empty(), 21.6 / 360);//FIXME
        case DEV -> new PivotConfig(0, Optional.empty(), 1); // FIXME
        case SIM -> new PivotConfig(0, Optional.empty(), 1); // FIXME
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(0.02, 0, 0.02, 0.03, 0.09, 0, 0.4);
        case DEV -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };


  public record PivotConfig(int motorID, Optional<Integer> canCoderID, double reduction) {}

  public record PIDGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final boolean INVERT_MOTOR = true;

  // SOFT LIMITS
  public static final double UPPER_EXTENSION_LIMIT = 110; // top limit is 121 rotations

  // top limit is 121 rotations

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 3;
  public static final double LOWER_VOLT_LIMIT = -3;
  public static final double SUPPLY_CURRENT_LIMIT = 30;
  public static final int ZEROING_CURRENT_LIMIT = 20; // FIXME currently doesn't exist lmao
}