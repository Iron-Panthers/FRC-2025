package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants;
import java.util.Optional;

public class ElevatorConstants {
  public static final ElevatorConfig ELEVATOR_CONFIG =
      switch (Constants.getRobotType()) {
        case PROG -> new ElevatorConfig(0, 1);
        case ALPHA -> new ElevatorConfig(37, 9.0 / 4.0); // FIXME
        case SIM -> new ElevatorConfig(0, 1); // FIXME
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case PROG -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case ALPHA -> new PIDGains(2, 0, 0.2, 0, 0.09, 0, .34);
        case SIM -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public record ElevatorConfig(int motorID, double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

  public static final boolean INVERT_MOTOR = true;

  public static final double POSITION_TARGET_EPSILON = 1;

  // SOFT LIMITS
  public static final Optional<Double> UPPER_EXTENSION_LIMIT =
      Optional.of(121d); // top limit is 121 rotations
  public static final Optional<Double> LOWER_EXTENSION_LIMIT = Optional.empty();

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 10;
  public static final double LOWER_VOLT_LIMIT = -7;
  public static final double SUPPLY_CURRENT_LIMIT = 30;
  public static final int ZEROING_CURRENT_LIMIT = 20; // FIXME currently doesn't exist lmao

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = -1;
  public static final double ZEROING_OFFSET = 2.25; // offset in inches
  public static final double ZEROING_VOLTAGE_THRESHOLD = 4;
}
