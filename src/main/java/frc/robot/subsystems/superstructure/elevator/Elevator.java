package frc.robot.subsystems.superstructure.elevator;

import frc.robot.subsystems.superstructure.GenericSuperstructure;

public class Elevator extends GenericSuperstructure<Elevator.ElevatorTarget> {
  public enum ElevatorTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(2.25), // 25 and 7.25
    L1(5.5), // FIXME: 26 and 21.5
    L2(7.25), // 24 and 53.75
    L3(21.5), // 0 and 53.75
    L4(53.75),
    SOURCE(20),
    SETUP_INTAKE(22),
    INTAKE(17.5); // FIXME
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
