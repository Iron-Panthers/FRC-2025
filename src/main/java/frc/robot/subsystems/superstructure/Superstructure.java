package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import frc.robot.subsystems.superstructure.tongue.Tongue;
import frc.robot.subsystems.superstructure.tongue.Tongue.TongueTarget;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    SETUP_L4, // Setting up in L4
    SCORE_L4, // Scoring in L4
    SETUP_L3, // Setting up in L3
    SCORE_L3, // Scoring in L3
    L2, // Scoring in L2
    L1, // Scoring in the trough
    TOP, // Apex
    INTAKE,
    STOW, // Going to the lowest position
    ZERO; // Zero the motor
  }

  private SuperstructureState currentState = SuperstructureState.ZERO; // current state
  private SuperstructureState targetState = SuperstructureState.ZERO; // current target state
  private boolean stop = false;

  private final Elevator elevator;
  private final Pivot pivot;
  private final Tongue tongue;

  public Superstructure(Elevator elevator, Pivot pivot, Tongue tongue) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.tongue = tongue;
    pivot.setPositionTarget(PivotTarget.STOW);
    elevator.setPositionTarget(ElevatorTarget.BOTTOM);
    tongue.setPositionTarget(TongueTarget.STOW);
  }

  @Override
  public void periodic() {
    if (!stop) {
      switch (currentState) { // switch on the target state
        case L1 -> {
          elevator.setPositionTarget(ElevatorTarget.L1);
          pivot.setPositionTarget(PivotTarget.L1);
          tongue.setPositionTarget(TongueTarget.L1);

          // check for state transitions
          if (this.superstructureReachedTarget()) {
            if (targetState == SuperstructureState.L2) {
              setCurrentState(SuperstructureState.L2);
            } else if (targetState != currentState) {
              setCurrentState(SuperstructureState.STOW);
            }
          }
        }
        case L2 -> {
          elevator.setPositionTarget(ElevatorTarget.L2);
          pivot.setPositionTarget(PivotTarget.L2);
          tongue.setPositionTarget(TongueTarget.L2);

          // check for state transitions
          if (this.superstructureReachedTarget()) {
            if (targetState != currentState) {
              if (targetState == SuperstructureState.L1) {
                setCurrentState(SuperstructureState.L1);
              } else {
                setCurrentState(SuperstructureState.STOW);
              }
            }
          }
        }

        case SETUP_L3 -> {
          elevator.setPositionTarget(ElevatorTarget.L3);
          pivot.setPositionTarget(PivotTarget.SETUP_L3);
          tongue.setPositionTarget(TongueTarget.L3);

          // check for state transitions
          if (this.superstructureReachedTarget()) {
            if (targetState != currentState) {
              if (targetState == SuperstructureState.SCORE_L3) {
                setCurrentState(SuperstructureState.SCORE_L3);
              } else {
                setCurrentState(SuperstructureState.TOP);
              }
            }
          }
        }

        case SCORE_L3 -> {
          elevator.setPositionTarget(ElevatorTarget.L3);
          pivot.setPositionTarget(PivotTarget.SCORE_L3);
          tongue.setPositionTarget(TongueTarget.L3);

          // check for state transitions
          if (this.superstructureReachedTarget()) {
            if (targetState != currentState) {
              setCurrentState(SuperstructureState.SETUP_L3);
            }
          }
        }
        case SETUP_L4 -> {
          elevator.setPositionTarget(ElevatorTarget.SETUP_L4);
          pivot.setPositionTarget(PivotTarget.SETUP_L4);
          tongue.setPositionTarget(TongueTarget.L4);
          // check for state transitions
          if (this.superstructureReachedTarget()) {
            if (targetState == SuperstructureState.SETUP_L3
                || targetState == SuperstructureState.SCORE_L3) {
              setCurrentState(SuperstructureState.SETUP_L3);
            } else if (targetState == SuperstructureState.SCORE_L4) {
              if (tonguePoleDetected()) {
                setCurrentState(SuperstructureState.SCORE_L4);
              }
            } else if (targetState != currentState) {
              setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case SCORE_L4 -> {
          elevator.setPositionTarget(ElevatorTarget.SCORE_L4);
          pivot.setPositionTarget(PivotTarget.SCORE_L4);
          tongue.setPositionTarget(TongueTarget.STOW);
          // check for state transitions
          if (targetState != currentState && this.superstructureReachedTarget()) {
            if (targetState == SuperstructureState.SETUP_L4
                || targetState == SuperstructureState.SETUP_L3
                || targetState == SuperstructureState.SCORE_L3) {
              setCurrentState(SuperstructureState.SETUP_L4);
            } else {
              setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case TOP -> {
          elevator.setPositionTarget(ElevatorTarget.TOP);
          if (elevator.getPosition() > 5) {
            pivot.setPositionTarget(PivotTarget.TOP);
          }
          tongue.setPositionTarget(TongueTarget.TOP);

          // check for state transitions
          if (this.superstructureReachedTarget()) {
            if (targetState == SuperstructureState.SETUP_L4
                || targetState == SuperstructureState.SCORE_L4
                || targetState == SuperstructureState.SETUP_L3
                || targetState == SuperstructureState.SCORE_L3) {
              setCurrentState(SuperstructureState.SETUP_L4);
            } else if (targetState != currentState) {
              setCurrentState(SuperstructureState.STOW);
            }
          }
        }

        case STOW -> {
          elevator.setPositionTarget(ElevatorTarget.BOTTOM);
          pivot.setPositionTarget(PivotTarget.STOW);
          tongue.setPositionTarget(TongueTarget.STOW);

          // check for state transitions
          if (this.superstructureReachedTarget()) {
            if (targetState == SuperstructureState.INTAKE) {
              setCurrentState(SuperstructureState.INTAKE);
            } else if (targetState == SuperstructureState.L1) {
              setCurrentState(SuperstructureState.L1);
            } else if (targetState == SuperstructureState.L2) {
              setCurrentState(SuperstructureState.L2);
            } else if (targetState != currentState) {
              setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case INTAKE -> {
          elevator.setPositionTarget(ElevatorTarget.INTAKE);
          pivot.setPositionTarget(PivotTarget.INTAKE);
          tongue.setPositionTarget(TongueTarget.INTAKE);

          // check for state transitions
          if (elevator.reachedTarget()) {
            if (targetState == SuperstructureState.STOW) {
              setCurrentState(SuperstructureState.INTAKE);
            } else if (targetState == SuperstructureState.L1
                || targetState == SuperstructureState.L2) {
              setCurrentState(SuperstructureState.L1);
            } else if (targetState != currentState) {
              setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case ZERO -> {
          pivot.setPositionTarget(PivotTarget.STOW);
          elevator.setZeroing(true);
          tongue.setPositionTarget(TongueTarget.STOW);
          if (elevator.getFilteredSupplyCurrentAmps()
              > ElevatorConstants
                  .ZEROING_VOLTAGE_THRESHOLD) { // check if the elevator is done zeroing and set
            // offsets accordingly
            elevator.setOffset();
            elevator.setControlMode(ControlMode.POSITION);
            elevator.setZeroing(false);

            setTargetState(SuperstructureState.STOW);
            setCurrentState(SuperstructureState.STOW);
          }
        }
      }
    } else {
      elevator.setControlMode(ControlMode.STOP);
      pivot.setControlMode(ControlMode.STOP);
    }

    elevator.periodic();
    pivot.periodic();
    tongue.periodic();

    Logger.recordOutput("Superstructure/TargetState", targetState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
    Logger.recordOutput("Superstructure/Elevator reached target", elevator.reachedTarget());
    Logger.recordOutput("Superstructure/Pivot reached target", pivot.reachedTarget());
    Logger.recordOutput("Superstructure/Reached Target", superstructureReachedTarget());
  }

  // Target state getter and setter
  public void setTargetState(SuperstructureState superstructureState) {
    this.stop = false;
    this.targetState = superstructureState;
  }

  public SuperstructureState getTargetState() {
    return targetState;
  }

  // Current state getter and setter
  public void setCurrentState(SuperstructureState superstructureState) {
    this.stop = false;
    this.currentState = superstructureState;
  }

  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public void setStopped(boolean stopped) {
    this.stop = stopped;
  }

  public boolean getStopped() {
    return stop;
  }

  // go to target state command factory
  public Command goToStateCommand(SuperstructureState superstructureState) {
    return new FunctionalCommand(
        () -> {
          setTargetState(superstructureState);
        },
        () -> {},
        (e) -> {},
        () -> {
          return currentState == targetState && superstructureReachedTarget();
        },
        this);
  }

  /**
   * Get the position of the elevator
   *
   * @return the position of the elevator
   */
  public double getElevatorPosition() {
    return elevator.getPosition();
  }

  /**
   * Get the position of the pivot
   *
   * @return the position of the pivot
   */
  public double getPivotPosition() {
    return pivot.getPosition();
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
   * @return a boolean that says whether or not both of our mechanisms have finished zeroing
   */
  public boolean notZeroing() {
    return !elevator.isZeroing();
  }

  /**
   * @return if both subsystems in the superstructure have reached their target
   */
  public boolean superstructureReachedTarget() {
    return elevator.reachedTarget()
        && pivot.reachedTarget()
        && currentState != SuperstructureState.ZERO;
  }

  public boolean tonguePoleDetected() {
    return tongue.poleDetected();
  }
}
