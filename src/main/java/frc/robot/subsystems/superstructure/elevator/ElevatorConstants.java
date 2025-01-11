package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants;

public class ElevatorConstants {
  public static final ElevatorConfig ELEVATOR_CONFIG =
      switch (Constants.getRobotType()) {
        case PROG -> new ElevatorConfig(0, 1);
        case DEV -> new ElevatorConfig(37, 9.0 / 4.0); // FIXME
        case SIM -> new ElevatorConfig(0, 1); // FIXME
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case PROG -> new PIDGains(0, 0, 0, 0, 0, 0);
        case DEV -> new PIDGains(2, 0, 0.2, 0, 0.09, 0);
        case SIM -> new PIDGains(0, 0, 0, 0, 0, 0);
      };

  public record ElevatorConfig(int motorID, double reduction) {}

  public record PIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

  public static final boolean INVERT_MOTOR = true;

  // SOFT LIMITS
  public static final double UPPER_EXTENSION_LIMIT = 121; // top limit is 121 rotations

  // top limit is 121 rotations

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 10;
  public static final double LOWER_VOLT_LIMIT = -7;
  public static final double SUPPLY_CURRENT_LIMIT = 30;
  public static final int ZEROING_CURRENT_LIMIT = 20; // FIXME currently doesn't exist lmao
}
