package frc.robot.subsystems.superstructure.pivot;

import frc.robot.subsystems.superstructure.GenericSuperstructure;
// for intaking a note - TODO: ask the intake people what to do, L4 - 45 degrees, for L2 and L3 - 35
// degrees, L1 - TODO: need to test

public class Pivot extends GenericSuperstructure<Pivot.PivotTarget> {
  public enum PivotTarget implements GenericSuperstructure.PositionTarget {
    TOP(90), // FIXME: Add zeroing code to make sure the robot ACTUALLY goes to 0
    SETUP_L1(0),
    SETUP_L2(30),
    SETUP_L3(30),
    SETUP_L4(30),
    SCORE_L1(0),
    SCORE_L2(20),
    SCORE_L3(20),
    SCORE_L4(0),
    INTAKE(-90);

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
