package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;

public class ElevatorIOTalonFX extends GenericSuperstructureIOTalonFX implements ElevatorIO {
  public static final double REDUCTION = 1.892 / 1; // rotations to inches
  public static final boolean INVERTED = false; // FIXME

  public static final double SUPPLY_CURRENT_LIMIT = 30; // FIXME
  public static final int ZEROING_CURRENT_LIMIT = 20; // FIXME

  public static final int ZEROING_VOLTS = -2; // FIXME

  public static final int ID = 18;

  public static final double P = 3;
  public static final double I = 0;
  public static final double D = 0;

  public static final double S = 0.25;
  public static final double G = 0.15;

  public static final double UPPER_LIMIT = 33;

  public static final double UPPER_VOLT_LIMIT = 10;
  public static final double LOWER_VOLT_LIMIT = -7;

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

  public ElevatorIOTalonFX() {
    super(
        ID,
        INVERTED,
        SUPPLY_CURRENT_LIMIT,
        Optional.empty(),
        REDUCTION,
        UPPER_LIMIT,
        UPPER_VOLT_LIMIT,
        LOWER_VOLT_LIMIT);
    setSlot0(P, I, D, S, 0, 0, GRAVITY_TYPE);
  }
}
