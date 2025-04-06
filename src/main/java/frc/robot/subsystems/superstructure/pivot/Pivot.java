package frc.robot.subsystems.superstructure.pivot;

import frc.robot.subsystems.superstructure.GenericSuperstructure;

public class Pivot extends GenericSuperstructure<Pivot.PivotTarget> {
  public enum PivotTarget implements GenericSuperstructure.PositionTarget {
    TOP(-79),
    INTAKE(-96),
    STOW(-96),
    L1(-110),
    L2(-107),
    SCORE_L3(-117),
    SETUP_L3(-79),
    CLIMB(-115),
    ZERO_LOW(-95.2),
    ZERO_HIGH(90),
    SETUP_L4(144),
    SCORE_L4(149),
    DESCORE_HIGH(0),
    INTAKE_SIDE(40),
    SCORE_SIDE(100),

    // for the algae on L2
    DESCORE_LOW(-15);

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
    setPositionTarget(PivotTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(super.getPosition() - (super.getPositionTarget().getPosition() / 360d))
        <= superstructureIO.getPositionTargetEpsilon();
  }

  public double getPosition() {
    return super.getPosition() * 360.0;
  }
}
