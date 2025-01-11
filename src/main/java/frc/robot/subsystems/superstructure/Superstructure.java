package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    ZERO, // Zero the superstructure
    STOP, // Stop the superstructure
    SCORE_L4, // Scoring in L4
    SCORE_L3, // Scoring in L3
    SCORE_L2, // Scoring in L2
    SCORE_L1, // Scoring in the trough
    STOW, // Going to the lowest position
  }

  private SuperstructureState targetState = SuperstructureState.STOP; // current target state

  private final Elevator elevator;
  private final Pivot pivot;

  // linear filter for elevator
  private final LinearFilter elevatorSupplyCurrentFilter;
  private final LinearFilter pivotSupplyCurrentFilter;

  private double elevatorFilteredSupplyCurrentAmps = 0;
  private double pivotFilteredSupplyCurrentAmps = 0;


  public Superstructure(Elevator elevator, Pivot pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
    pivot.setPositionTarget(PivotTarget.TOP);
    elevator.setPositionTarget(ElevatorTarget.BOTTOM);

    // setup the linear filter
    elevatorSupplyCurrentFilter = LinearFilter.movingAverage(10);
    pivotSupplyCurrentFilter = LinearFilter.movingAverage(10);
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
        elevator.setControlMode(ControlMode.ZERO);
        pivot.setControlMode(ControlMode.ZERO);
        //FIXME zeroing BAD 
        if (elevatorFilteredSupplyCurrentAmps > 4 && pivotFilteredSupplyCurrentAmps > 4 ) { // then stop it when it hits the bottom
          elevator.setOffset(); // set the offset
          pivot.setOffset();
          targetState = SuperstructureState.STOW; // then go to the stow position
        }
      }
      case STOP -> {
        elevator.setControlMode(ControlMode.STOP);
        pivot.setControlMode(ControlMode.STOP);
      }
    }
    elevator.periodic();
    pivot.periodic();

    // calculate our new filtered supply current for the elevator
    elevatorFilteredSupplyCurrentAmps =
        elevatorSupplyCurrentFilter.calculate(elevator.getSupplyCurrentAmps());

    pivotFilteredSupplyCurrentAmps =
    pivotSupplyCurrentFilter.calculate(pivot.getSupplyCurrentAmps());

    Logger.recordOutput("Superstructure/TargetState", targetState);
    Logger.recordOutput(
        "Superstructure/Elevator Filtered Supply Current", elevatorFilteredSupplyCurrentAmps);
    Logger.recordOutput(
      "Superstructure/Pivot Filtered Supply Current", pivotFilteredSupplyCurrentAmps);
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
}
