package frc.robot.subsystems.superstructure.pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;

public class PivotIOTalonFX extends GenericSuperstructureIOTalonFX implements PivotIO {
  // gear ratio 21.6 : 1

  public static final double REDUCTION = 360 / 21.6; // rotations to degrees
  public static final boolean INVERTED = true; // FIXME

  public static final double SUPPLY_CURRENT_LIMIT = 30; // FIXME

  public static final int ZEROING_VOLTS = -2; // FIXME

  public static final int ID = 15;
  public static final int ENCODER_ID = 28;

  public static final double P = 0.27;
  public static final double I = 0;
  public static final double D = 0;

  public static final double S = 0.03;
  public static final double G = 0.4;

  public static final double UPPER_LIMIT = 70;

  public static final double UPPER_VOLT_LIMIT = 5;
  public static final double LOWER_VOLT_LIMIT = -4;
  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public PivotIOTalonFX() {
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

  /**
   * Runs the characterization of the pivot subsystem with a positivie voltage output so that it
   * zeros at the hardstop above it (set at 90 degrees)
   */
  @Override
  public void runCharacterization() {
    talon.setControl(voltageOutput.withOutput(1));
  }

  /**
   * Sets the offset of the pivot to 90 degrees instead of 0 degrees this is because the hardstop is
   * at 90 degrees above where we want the zero to be
   */
  @Override
  public void setOffset() {
    talon.getConfigurator().setPosition(90);
  }
}
