package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    ZERO,
    STOP,
    MIDDLE,
    BOTTOM,
  }

  private SuperstructureState targetState = SuperstructureState.ZERO;

  private final Elevator elevator;

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    switch (targetState) {
      case MIDDLE -> { // TODO: change this to the actual middle position
        elevator.setPositionTarget(ElevatorTarget.L2);
      }
      case BOTTOM -> {
        elevator.setPositionTarget(ElevatorTarget.BOTTOM);
      }
      case ZERO -> {
        elevator.runCharacterization();
      }
      case STOP -> {
        elevator.stop();
      }
    }
    elevator.periodic();
    // pivot.periodic(); don't do this yet lmao
    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public void setTargetState(SuperstructureState superstructureState) {
    targetState = superstructureState;
  }

  public double getElevatorPosition() {
    return elevator.position();
  }

  public double getElevatorSupplyCurrentAmps() {
    return elevator.supplyCurrentAmps();
  }

  // public double getPivotSupplyCurrentAmps() {
  //   return pivot.supplyCurrentAmps();
  // }

  public SuperstructureState getTargetState() {
    return targetState;
  }
}
