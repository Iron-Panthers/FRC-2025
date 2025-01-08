package frc.robot.subsystems.superstructure.pivot;

import frc.robot.subsystems.superstructure.GenericSuperstructure;

public class Pivot extends GenericSuperstructure<Pivot.PivotTarget> {
  public enum PivotTarget implements GenericSuperstructure.PositionTarget {
    L4(0),    //TODO: fix these modes
    L3(0),
    L2(0),
    L1(53),
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
