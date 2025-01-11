package frc.robot.subsystems.superstructure.elevator;

import frc.robot.subsystems.superstructure.GenericSuperstructure;

public class Elevator extends GenericSuperstructure<Elevator.ElevatorTarget> {
  public enum ElevatorTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(0),
    L1(0),
    L2(25),
    L3(50),
    L4(54),
    SOURCE(20);
    private double position = 0;

    private ElevatorTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public Elevator(ElevatorIO io) {
    super("Elevator", io);
    setPositionTarget(ElevatorTarget.BOTTOM);
  }
}
