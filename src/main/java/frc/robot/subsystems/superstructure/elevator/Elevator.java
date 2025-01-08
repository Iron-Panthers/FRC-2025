package frc.robot.subsystems.superstructure.elevator;

import frc.robot.subsystems.superstructure.GenericSuperstructure;

public class Elevator extends GenericSuperstructure<Elevator.ElevatorTarget> {
  public enum ElevatorTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(0),
    L2(20),
    L3(15),
    L4(29),
    SOURCE(5);
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
  }
}
