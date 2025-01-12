package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    STOP, // Stop the superstructure
    SCORE_L4, // Scoring in L4
    SCORE_L3, // Scoring in L3
    SCORE_L2, // Scoring in L2
    SCORE_L1, // Scoring in the trough
    STOW, // Going to the lowest position
    ZERO, // Zero the motor
  }

  private SuperstructureState targetState = SuperstructureState.STOW; // current target state

  private final Elevator elevator;
  private final Pivot pivot;

  public Superstructure(Elevator elevator, Pivot pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
    pivot.setPositionTarget(PivotTarget.TOP);
    elevator.setPositionTarget(ElevatorTarget.BOTTOM);
  }

  @Override
  public void periodic() {
    switch (targetState) { // switch on the target state
      case SCORE_L1 -> {
        elevator.setPositionTarget(ElevatorTarget.L1);
        pivot.setPositionTarget(PivotTarget.L1);
      }
      case SCORE_L2 -> {
        elevator.setPositionTarget(ElevatorTarget.L2);
        pivot.setPositionTarget(PivotTarget.L2);
      }
      case SCORE_L3 -> {
        elevator.setPositionTarget(ElevatorTarget.L3);
        pivot.setPositionTarget(PivotTarget.L3);
      }
      case SCORE_L4 -> {
        elevator.setPositionTarget(ElevatorTarget.L4);
        pivot.setPositionTarget(PivotTarget.L4);
      }
      case STOW -> {
        elevator.setPositionTarget(ElevatorTarget.BOTTOM);
        pivot.setPositionTarget(PivotTarget.TOP);
      }
      case ZERO -> {
        if (notZeroing()) { // set our mechanisms to zero if they aren't already
          elevator.setControlMode(ControlMode.ZERO);
          pivot.setControlMode(ControlMode.ZERO);
        } else { // if our mechanisms are currently zeroing run this logic
          if (elevator.getFilteredSupplyCurrentAmps()
              > 4) { // check if the elevator is done zeroing and set offsets accordingly
            elevator.setOffset();
            elevator.setControlMode(ControlMode.POSITION);
          }
          if (pivot.getFilteredSupplyCurrentAmps()
              > 5) { // check if pivot is done zeroing and set offsets accordingly
            pivot.setOffset();
            pivot.setControlMode(ControlMode.POSITION);
          }
          if (notZeroing()) { // if both of our mechanisms aren't zeroing anymore, exit out of this
            // madness
            setTargetState(SuperstructureState.STOW);
          }
        }
      }
      case STOP -> {
        elevator.setControlMode(ControlMode.STOP);
        pivot.setControlMode(ControlMode.STOP);
      }
    }
    elevator.periodic();
    pivot.periodic();

    Logger.recordOutput("Superstructure/TargetState", targetState);
  }

  // Target state getter and setter
  public void setTargetState(SuperstructureState superstructureState) {
    targetState = superstructureState;
  }

  public SuperstructureState getTargetState() {
    return targetState;
  }

  /**
   * Get the position of the elevator
   *
   * @return the position of the elevator
   */
  public double getElevatorPosition() {
    return elevator.position();
  }

  /**
   * Get the position of the pivot
   *
   * @return the position of the pivot
   */
  public double getPivotPosition() {
    return pivot.position();
  }

  /**
   * Get the supply current of the elevator
   *
   * @return the supply current of the elevator
   */
  public double getElevatorSupplyCurrentAmps() {
    return elevator.getSupplyCurrentAmps();
  }
  /**
   * Get the supply current of the pivot
   *
   * @return the supply current of the pivot
   */
  public double getPivotSupplyCurrentAmps() {
    return pivot.getSupplyCurrentAmps();
  }

  /**
   * @return a boolean that says weather or not both of our mechanisms have finished zeroing
   */
  public boolean notZeroing() {
    return elevator.getControlMode() != ControlMode.ZERO
        && pivot.getControlMode() != ControlMode.ZERO;
  }
}
