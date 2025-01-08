package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    STOW,
    INTAKE,
    SUBWOOF_SHOT,
    SHUTTLE,
    AMP,
    ZERO,
    STOP
  }

  private SuperstructureState targetState = SuperstructureState.STOW;

  private final Elevator elevator;
  private final Pivot pivot;

  public Superstructure(Elevator elevator, Pivot pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  public void periodic() {
    switch (targetState) {
      case STOW -> {
        pivot.setPositionTarget(PivotTarget.STOW);
        elevator.setPositionTarget(ElevatorTarget.STOW);
      }
      case INTAKE -> {
        pivot.setPositionTarget(PivotTarget.STOW);
        elevator.setPositionTarget(ElevatorTarget.STOW);
      }
      case SUBWOOF_SHOT -> {
        pivot.setPositionTarget(PivotTarget.SUBWOOF_SHOT);
        elevator.setPositionTarget(ElevatorTarget.STOW);
      }
      case SHUTTLE -> {
        pivot.setPositionTarget(PivotTarget.SHUTTLE);
        elevator.setPositionTarget(ElevatorTarget.STOW);
      }
      case AMP -> {
        elevator.setPositionTarget(ElevatorTarget.AMP);
        pivot.setPositionTarget(PivotTarget.STOW);
      }
      case ZERO -> {
        elevator.runCharacterization();
        pivot.runCharacterization();
      }
      case STOP -> {
        elevator.stop();
        pivot.stop();
        setTargetState(SuperstructureState.STOW);
      }
    }
    elevator.periodic();
    pivot.periodic();
    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public void setTargetState(SuperstructureState superstructureState) {
    targetState = superstructureState;
  }

  public double elevatorPosition() {
    return elevator.position();
  }

  public double getElevatorSupplyCurrentAmps() {
    return elevator.supplyCurrentAmps();
  }

  public double getPivotSupplyCurrentAmps() {
    return pivot.supplyCurrentAmps();
  }

  public SuperstructureState getTargetState() {
    return targetState;
  }
}
