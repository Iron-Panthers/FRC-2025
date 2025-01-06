package frc.robot.subsystems.superstructure.pivot;

import frc.robot.subsystems.superstructure.GenericSuperstructure;

public class Pivot extends GenericSuperstructure<Pivot.PivotTarget> {
  public enum PivotTarget implements GenericSuperstructure.PositionTarget {
    STOW(0),
    ZERO(0),
    SUBWOOF_SHOT(70),
    SHUTTLE(45);
    private double position;

    private PivotTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public Pivot(PivotIO io) {
    super("Pivot", io);
  }
}
