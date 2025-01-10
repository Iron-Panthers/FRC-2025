package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
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

  // linear filter for elevator
  private final LinearFilter elevatorSupplyCurrentFilter;
  private double elevatorFilteredSupplyCurrentAmps = 0;

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
    elevator.setPositionTarget(ElevatorTarget.BOTTOM);

    // setup the linear filter
    elevatorSupplyCurrentFilter = LinearFilter.movingAverage(10);
  }

  @Override
  public void periodic() {
    switch (targetState) {// switch on the target state
      case SCORE_L1 -> {
        elevator.setPositionTarget(ElevatorTarget.L1);
      }
      case SCORE_L2 -> {
        elevator.setPositionTarget(ElevatorTarget.L2);
      }
      case SCORE_L3 -> {
        elevator.setPositionTarget(ElevatorTarget.L3);
      }
      case SCORE_L4 -> {
        elevator.setPositionTarget(ElevatorTarget.L4);
      }
      case STOW -> {
        elevator.setPositionTarget(ElevatorTarget.BOTTOM);
      }
      case ZERO -> {
        elevator.setControlMode(ControlMode.ZERO);
        if (elevatorFilteredSupplyCurrentAmps > 4) { // then stop it when it hits the bottom
          elevator.setOffset(); // set the offset
          targetState = SuperstructureState.STOW; // then go to the stow position
        }
      }
      case STOP -> {
        elevator.setControlMode(ControlMode.STOP);
        ;
      }
    }
    elevator.periodic();

    // calculate our new filtered supply current
    elevatorFilteredSupplyCurrentAmps =
        elevatorSupplyCurrentFilter.calculate(elevator.getSupplyCurrentAmps());

    Logger.recordOutput("Superstructure/TargetState", targetState);
    Logger.recordOutput(
        "Superstructure/Filtered Supply Current", elevatorFilteredSupplyCurrentAmps);
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
   * @return the position of the elevator
   */
  public double getElevatorPosition() {
    return elevator.position();
  }

  /**
   * Get the supply current of the elevator
   * @return the supply current of the elevator
   */
  public double getElevatorSupplyCurrentAmps() {
    return elevator.getSupplyCurrentAmps();
  }

}
