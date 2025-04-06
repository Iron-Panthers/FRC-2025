package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
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
    CLIMB,
    PREVENT_TIPPING,
    DESCORE_HIGH, // Algae hitting on L3
    DESCORE_LOW, // Algae hitting on L2
    ZERO; // Zero the motor
  }

  private SuperstructureState currentState = SuperstructureState.STOW; // current state
  private SuperstructureState targetState = SuperstructureState.STOW; // current target state
  private SuperstructureState bufferCurrentState = SuperstructureState.STOW;
  private boolean stop = false;

  private final Elevator elevator;
  private final Pivot pivot;
  private final Tongue tongue;

  private boolean overrideIsAtTarget = false;

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
    currentState = bufferCurrentState;
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
            } else if (targetState == SuperstructureState.INTAKE
                || targetState == SuperstructureState.STOW) {
              setCurrentState(SuperstructureState.STOW);
            } else if (targetState == SuperstructureState.CLIMB) {
              setCurrentState(SuperstructureState.CLIMB);
            } else if (targetState != currentState) {
              setCurrentState(SuperstructureState.TOP);
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
              } else if (targetState == SuperstructureState.INTAKE
                  || targetState == SuperstructureState.STOW) {
                setCurrentState(SuperstructureState.STOW);
              } else if (targetState == SuperstructureState.CLIMB) {
                setCurrentState(SuperstructureState.CLIMB);
              } else if (targetState != currentState) {
                setCurrentState(SuperstructureState.TOP);
              }
            }
          }
        }

        case SETUP_L3 -> {
          if (elevator.getPosition() > 32) {
            pivot.setPositionTarget(PivotTarget.SETUP_L3);
          }
          elevator.setPositionTarget(ElevatorTarget.L3);
          tongue.setPositionTarget(TongueTarget.L3);

          // check for state transitions
          if (this.superstructureReachedTarget() && targetState != currentState) {
            switch (targetState) {
              case SCORE_L3 -> setCurrentState(SuperstructureState.SCORE_L3);
              case DESCORE_HIGH -> setCurrentState(SuperstructureState.DESCORE_HIGH);
              case DESCORE_LOW -> setCurrentState(SuperstructureState.DESCORE_LOW);
              case SCORE_L4, SETUP_L4 -> setCurrentState(SuperstructureState.PREVENT_TIPPING);
              default -> setCurrentState(SuperstructureState.TOP);
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

          // ALL OF OUR TOP TO SCORE_SIDE STATES
        case PREVENT_TIPPING -> {
          // this one we have to make the elevator manually go to the correct position to avoid the
          // pivot touching the intake

          // switch our pivot based on our next state
          switch (targetState) {
            case SETUP_L4, SCORE_L4, PREVENT_TIPPING -> pivot.setPositionTarget(
                PivotTarget.SCORE_SIDE);
            default -> pivot.setPositionTarget(PivotTarget.INTAKE_SIDE);
          }

          if (pivot.getPosition() > -60.0
              && (pivot.getPositionTarget() == PivotTarget.INTAKE_SIDE
                  || (pivot.getPositionTarget() == PivotTarget.SCORE_SIDE
                      && pivot.getPosition() < 90.0))) { // not in constants
            elevator.setPositionTarget(ElevatorTarget.INTAKE_SIDE);
          } else {
            switch (targetState) { // set elevator pos based on target state
              case PREVENT_TIPPING -> elevator.setPositionTarget(
                  pivot.getPosition() > -60.0 ? ElevatorTarget.INTAKE_SIDE : ElevatorTarget.TOP);
              case SETUP_L4, SCORE_L4 -> elevator.setPositionTarget(ElevatorTarget.SETUP_L4);
              case SETUP_L3, SCORE_L3 -> elevator.setPositionTarget(ElevatorTarget.L3);
              default -> elevator.setPositionTarget(ElevatorTarget.TOP);
            }
          }

          if (pivot.getPosition() > 90.0) {
            tongue.setPositionTarget(TongueTarget.L4);
          } else {
            tongue.setPositionTarget(TongueTarget.STOW);
          }

          if (currentState != targetState
              && elevator.reachedTarget()
              && (Math.abs(
                      pivot.getPositionTarget().getPosition() / 360d - pivot.getPosition() / 360d)
                  < 0.05)) {
            switch (targetState) {
              case SETUP_L4, SCORE_L4 -> setCurrentState(SuperstructureState.SETUP_L4);
              case DESCORE_HIGH -> setCurrentState(SuperstructureState.DESCORE_HIGH);
              case DESCORE_LOW -> setCurrentState(SuperstructureState.DESCORE_LOW);
              default -> setCurrentState(SuperstructureState.TOP);
            }
          }
        }

        case SETUP_L4 -> {
          elevator.setPositionTarget(ElevatorTarget.SETUP_L4);
          pivot.setPositionTarget(PivotTarget.SETUP_L4);
          tongue.setPositionTarget(TongueTarget.L4);
          // check for state transitions
          if (currentState != targetState && this.superstructureReachedTarget()) {
            switch (targetState) {
              case SCORE_L4 -> {
                if (tonguePoleDetected()) {
                  setCurrentState(SuperstructureState.SCORE_L4);
                }
              }
              default -> setCurrentState(SuperstructureState.PREVENT_TIPPING);
            }
          }
        }
        case SCORE_L4 -> {
          elevator.setPositionTarget(ElevatorTarget.SCORE_L4);
          pivot.setPositionTarget(PivotTarget.SCORE_L4);
          tongue.setPositionTarget(TongueTarget.STOW);
          // check for state transitions
          if (targetState != currentState && this.superstructureReachedTarget()) {
            switch (targetState) {
              case SETUP_L4 -> setCurrentState(SuperstructureState.SETUP_L4);
              default -> setCurrentState(SuperstructureState.PREVENT_TIPPING);
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
          if (currentState != targetState && this.superstructureReachedTarget()) {
            switch (targetState) {
              case SETUP_L4, SCORE_L4, PREVENT_TIPPING -> setCurrentState(
                  SuperstructureState.PREVENT_TIPPING);
              case CLIMB -> setCurrentState(SuperstructureState.CLIMB);
              case L2 -> setCurrentState(SuperstructureState.L2);
              case L1 -> setCurrentState(SuperstructureState.L1);
              case DESCORE_HIGH -> setCurrentState(SuperstructureState.DESCORE_HIGH);
              case DESCORE_LOW -> setCurrentState(SuperstructureState.DESCORE_LOW);
              case SETUP_L3, SCORE_L3 -> setCurrentState(SuperstructureState.SETUP_L3);
              default -> setCurrentState(SuperstructureState.STOW);
            }
          }
        }

        case STOW -> {
          if (pivot.getPosition() < -0.27) {
            elevator.setPositionTarget(ElevatorTarget.BOTTOM);
          }
          pivot.setPositionTarget(PivotTarget.STOW);
          tongue.setPositionTarget(TongueTarget.STOW);

          // check for state transitions
          if (currentState != targetState && this.superstructureReachedTarget()) {
            switch (targetState) {
              case INTAKE -> setCurrentState(SuperstructureState.INTAKE);
              case L1 -> setCurrentState(SuperstructureState.L1);
              case L2 -> setCurrentState(SuperstructureState.L2);
              case CLIMB -> setCurrentState(SuperstructureState.CLIMB);
              default -> setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case INTAKE -> {
          elevator.setPositionTarget(ElevatorTarget.INTAKE);
          pivot.setPositionTarget(PivotTarget.INTAKE);
          tongue.setPositionTarget(TongueTarget.INTAKE);

          // check for state transitions
          if (currentState != targetState && this.superstructureReachedTarget()) {
            switch (targetState) {
              case STOW -> setCurrentState(SuperstructureState.INTAKE);
              case L1 -> setCurrentState(SuperstructureState.L1);
              case L2 -> setCurrentState(SuperstructureState.L2);
              case CLIMB -> setCurrentState(SuperstructureState.CLIMB);
              default -> setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case CLIMB -> {
          if (elevator.getPosition() < 27 && elevator.getPosition() > 4) {
            pivot.setPositionTarget(PivotTarget.CLIMB);
          }
          elevator.setPositionTarget(ElevatorTarget.CLIMB);
          tongue.setPositionTarget(TongueTarget.CLIMB);

          // check for state transitions
          if (this.superstructureReachedTarget() && targetState != currentState) {
            if (targetState == SuperstructureState.L1) {
              setCurrentState(SuperstructureState.L1);
            } else if (targetState == SuperstructureState.INTAKE
                || targetState == SuperstructureState.STOW) {
              setCurrentState(SuperstructureState.STOW);
            } else if (targetState == SuperstructureState.L2) {
              setCurrentState(SuperstructureState.L2);
            } else {
              setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case DESCORE_HIGH -> {
          if (pivot.getPosition() > -60) {
            elevator.setPositionTarget(ElevatorTarget.DESCORE_HIGH);
          }
          pivot.setPositionTarget(PivotTarget.DESCORE_HIGH);
          tongue.setPositionTarget(TongueTarget.DESCORE);
          if (targetState != currentState && this.superstructureReachedTarget()) {
            switch (targetState) {
              case SETUP_L4, SCORE_L4, PREVENT_TIPPING -> setCurrentState(
                  SuperstructureState.PREVENT_TIPPING);
              case DESCORE_LOW -> setCurrentState(SuperstructureState.DESCORE_LOW);
              case SETUP_L3, SCORE_L3 -> setCurrentState(SuperstructureState.SETUP_L3);
              default -> setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case DESCORE_LOW -> {
          if (pivot.getPosition() > -60) {
            elevator.setPositionTarget(ElevatorTarget.DESCORE_LOW);
          }
          pivot.setPositionTarget(PivotTarget.DESCORE_LOW);
          tongue.setPositionTarget(TongueTarget.DESCORE);
          if (targetState != currentState && this.superstructureReachedTarget()) {
            switch (targetState) {
              case SETUP_L4, SCORE_L4, PREVENT_TIPPING -> setCurrentState(
                  SuperstructureState.PREVENT_TIPPING);
              case DESCORE_HIGH -> setCurrentState(SuperstructureState.DESCORE_HIGH);
              case SETUP_L3, SCORE_L3 -> setCurrentState(SuperstructureState.SETUP_L3);
              default -> setCurrentState(SuperstructureState.TOP);
            }
          }
        }
        case ZERO -> {
          // zeroing system for not killing the robot on zero

          // set our pivot pos
          if (pivot.getPosition() < PivotConstants.ZEROING_HIGH_THRESHOLD) {
            pivot.setPositionTarget(PivotTarget.ZERO_LOW);
            tongue.setPositionTarget(TongueTarget.STOW);
          } else {
            pivot.setPositionTarget(PivotTarget.ZERO_HIGH);
            tongue.setPositionTarget(TongueTarget.L4);
          }

          // wait for pivot to go to safe pos before zeroing
          if (pivot.reachedTarget()) {
            elevator.setZeroing(true);
          } else {
            elevator.setZeroing(false);
          }

          // check if we have have hit our hardstop, if so we can zero the elevator
          if (elevator.getFilteredSupplyCurrentAmps()
              > ElevatorConstants
                  .ZEROING_VOLTAGE_THRESHOLD) { // check if the elevator is done zeroing and set
            // offsets accordingly
            elevator.setOffset();
            elevator.setControlMode(ControlMode.POSITION);
            elevator.setZeroing(false);

            if (pivot.getPositionTarget() == PivotTarget.ZERO_HIGH) {
              setTargetState(SuperstructureState.STOW);
              setCurrentState(SuperstructureState.TOP);
            } else {
              setTargetState(SuperstructureState.STOW);
              setCurrentState(SuperstructureState.STOW);
            }
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
    this.bufferCurrentState = superstructureState;
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
    boolean output =
        (elevator.reachedTarget()
                && pivot.reachedTarget()
                && currentState != SuperstructureState.ZERO)
            || overrideIsAtTarget;

    overrideIsAtTarget = false;
    return output;
  }

  public void oneTimeOverride() {
    overrideIsAtTarget = true;
  }

  public Command oneTimeOverrideCommand() {
    return new InstantCommand(() -> oneTimeOverride());
  }

  public boolean tonguePoleDetected() {
    return tongue.poleDetected();
  }
}
