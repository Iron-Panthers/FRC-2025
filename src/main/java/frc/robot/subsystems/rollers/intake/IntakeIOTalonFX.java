package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollersIOTalonFX;

public class IntakeIOTalonFX extends GenericRollersIOTalonFX implements IntakeIO {
  private static final int id = 13; // FIXME
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = false;
  private static final boolean brake = false;
  private static final double reduction = 1 / 1;

  public IntakeIOTalonFX() {
    super(id, currentLimitAmps, inverted, brake, reduction);
  }
}
